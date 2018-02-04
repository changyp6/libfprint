/*
 * Elan driver for libfprint
 *
 * Copyright (C) 2017 Igor Filatov <ia.filatov@gmail.com>
 * Copyright (C) 2018 Sébastien Béchet <sebastien.bechet@osinix.com >
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "elan"

#include <errno.h>
#include <libusb.h>
#include <assembling.h>
#include <fp_internal.h>
#include <fprint.h>
#include <stdlib.h>

#include "elan.h"
#include "driver_ids.h"

unsigned char elan_get_pixel(struct fpi_frame_asmbl_ctx *ctx,
			     struct fpi_frame *frame, unsigned int x,
			     unsigned int y)
{
	return frame->data[x + y * ctx->frame_width];
}

static struct fpi_frame_asmbl_ctx assembling_ctx = {
	.frame_width = 0,
	.frame_height = 0,
	.image_width = 0,
	.get_pixel = elan_get_pixel,
};

struct elan_dev {
	/* device config */
	unsigned short dev_type;
	unsigned short fw_ver;
	void (*process_frame) (unsigned short *raw_frame, GSList ** frames);
	/* end device config */

	/* commands */
	const struct elan_cmd *cmd;
	int cmd_timeout;
	struct libusb_transfer *cur_transfer;
	/* end commands */

	/* state */
	gboolean deactivating;
	unsigned char *last_read;
	unsigned char calib_atts_left;
	unsigned char calib_status;
	unsigned short *background;
	unsigned char frame_width;
	unsigned char frame_height;
	unsigned char raw_frame_width;
	int num_frames;
	GSList *frames;
	/* end state */
};

int cmp_short(const void *a, const void *b)
{
	return (int)(*(short *)a - *(short *)b);
}

static void elan_dev_reset(struct elan_dev *elandev)
{
	fp_dbg("");

	BUG_ON(elandev->cur_transfer);

	elandev->cmd = NULL;
	elandev->cmd_timeout = ELAN_CMD_TIMEOUT;

	elandev->deactivating = FALSE;
	elandev->calib_status = 0;

	g_free(elandev->last_read);
	elandev->last_read = NULL;

	g_slist_free_full(elandev->frames, g_free);
	elandev->frames = NULL;
	elandev->num_frames = 0;
}

static void elan_save_frame(struct elan_dev *elandev, unsigned short *frame)
{
	fp_dbg("");

	unsigned char raw_height = elandev->frame_width;
	unsigned char raw_width = elandev->raw_frame_width;

	/* Raw images are vertical and perpendicular to swipe direction of a
	 * normalized image, which means we need to make them horizontal before
	 * assembling. We also discard stripes of 'frame_margin' along raw
	 * height. */
	unsigned char frame_margin = (raw_width - elandev->frame_height) / 2;
	for (int y = 0; y < raw_height; y++)
		for (int x = frame_margin; x < raw_width - frame_margin; x++) {
			int frame_idx = y + (x - frame_margin) * raw_height;
			int raw_idx = x + y * raw_width;
			frame[frame_idx] =
			    ((unsigned short *)elandev->last_read)[raw_idx];
		}
}

static void elan_save_background(struct elan_dev *elandev)
{
	fp_dbg("");

	g_free(elandev->background);
	elandev->background =
	    g_malloc(elandev->frame_width * elandev->frame_height *
		     sizeof(short));
	elan_save_frame(elandev, elandev->background);
}

/* save a frame as part of the fingerprint image
 * background needs to have been captured for this routine to work
 * Elantech recommends 2-step non-linear normalization in order to reduce
 * 2^14 ADC resolution to 2^8 image:
 *
 * 1. background is subtracted (done here)
 *
 * 2. pixels are grouped in 3 groups by intensity and each group is mapped
 *    separately onto the normalized frame (done in elan_process_frame_*)
 *    ==== 16383     ____> ======== 255
 *                  /
 *    ----- lvl3 __/
 *                   35% pixels
 *
 *    ----- lvl2 --------> ======== 156
 *
 *                   30% pixels
 *    ----- lvl1 --------> ======== 99
 *
 *                   35% pixels
 *    ----- lvl0 __
 *                 \
 *    ======== 0    \____> ======== 0
 *
 * For some devices we don't do 2. but instead do a simple linear mapping
 * because it seems to produce better results (or at least as good):
 *    ==== 16383      ___> ======== 255
 *                   /
 *    ------ max  __/
 *
 *
 *    ------ min  __
 *                  \
 *    ======== 0     \___> ======== 0
 */
static void elan_save_img_frame(struct elan_dev *elandev)
{
	fp_dbg("");

	unsigned int frame_size = elandev->frame_width * elandev->frame_height;
	unsigned short *frame = g_malloc(frame_size * sizeof(short));
	elan_save_frame(elandev, frame);

	for (int i = 0; i < frame_size; i++)
		if (elandev->background[i] > frame[i])
			frame[i] = 0;
		else
			frame[i] -= elandev->background[i];

	elandev->frames = g_slist_prepend(elandev->frames, frame);
	elandev->num_frames += 1;
}

static void elan_process_frame_linear(unsigned short *raw_frame,
				      GSList ** frames)
{
	fp_dbg("");

	unsigned int frame_size =
	    assembling_ctx.frame_width * assembling_ctx.frame_height;
	struct fpi_frame *frame =
	    g_malloc(frame_size + sizeof(struct fpi_frame));

	unsigned short min = 0xffff, max = 0;
	for (int i = 0; i < frame_size; i++) {
		if (raw_frame[i] < min)
			min = raw_frame[i];
		if (raw_frame[i] > max)
			max = raw_frame[i];
	}

	unsigned short px;
	for (int i = 0; i < frame_size; i++) {
		px = raw_frame[i];
		px = (px - min) * 0xff / (max - min);
		frame->data[i] = (unsigned char)px;
	}

	*frames = g_slist_prepend(*frames, frame);
}

static void elan_process_frame_thirds(unsigned short *raw_frame,
				      GSList ** frames)
{
	fp_dbg("");

	unsigned int frame_size =
	    assembling_ctx.frame_width * assembling_ctx.frame_height;
	struct fpi_frame *frame =
	    g_malloc(frame_size + sizeof(struct fpi_frame));

	unsigned short lvl0, lvl1, lvl2, lvl3;
	unsigned short *sorted = g_malloc(frame_size * sizeof(short));
	memcpy(sorted, raw_frame, frame_size * sizeof(short));
	qsort(sorted, frame_size, sizeof(short), cmp_short);
	lvl0 = sorted[0];
	lvl1 = sorted[frame_size * 3 / 10];
	lvl2 = sorted[frame_size * 65 / 100];
	lvl3 = sorted[frame_size - 1];
	g_free(sorted);

	unsigned short px;
	for (int i = 0; i < frame_size; i++) {
		px = raw_frame[i];
		if (lvl0 <= px && px < lvl1)
			px = (px - lvl0) * 99 / (lvl1 - lvl0);
		else if (lvl1 <= px && px < lvl2)
			px = 99 + ((px - lvl1) * 56 / (lvl2 - lvl1));
		else		// (lvl2 <= px && px <= lvl3)
			px = 155 + ((px - lvl2) * 100 / (lvl3 - lvl2));
		frame->data[i] = (unsigned char)px;
	}

	*frames = g_slist_prepend(*frames, frame);
}

static void elan_submit_image(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;
	GSList *frames = NULL;
	struct fp_img *img;

	for (int i = 0; i < ELAN_SKIP_LAST_FRAMES; i++)
		elandev->frames = g_slist_next(elandev->frames);

	assembling_ctx.frame_width = elandev->frame_width;
	assembling_ctx.frame_height = elandev->frame_height;
	assembling_ctx.image_width = elandev->frame_width * 3 / 2;
	g_slist_foreach(elandev->frames, (GFunc) elandev->process_frame,
			&frames);
	fpi_do_movement_estimation(&assembling_ctx, frames,
				   elandev->num_frames - ELAN_SKIP_LAST_FRAMES);
	img =
	    fpi_assemble_frames(&assembling_ctx, frames,
				elandev->num_frames - ELAN_SKIP_LAST_FRAMES);

	img->flags |= FP_IMG_PARTIAL;
	fpi_imgdev_image_captured(dev, img);
}

static void elan_cmd_done(struct fpi_ssm *ssm)
{
	fp_dbg("");
	fpi_ssm_next_state(ssm);
}

static void elan_cmd_cb(struct libusb_transfer *transfer)
{
	fp_dbg("");

	struct fpi_ssm *ssm = transfer->user_data;
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	elandev->cur_transfer = NULL;

	switch (transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED:
		if (transfer->length != transfer->actual_length) {
			fp_dbg("transfer length error: expected %d, got %d",
			       transfer->length, transfer->actual_length);
			elan_dev_reset(elandev);
			fpi_ssm_mark_aborted(ssm, -EPROTO);
		} else if (transfer->endpoint & LIBUSB_ENDPOINT_IN)
			/* just finished receiving */
			elan_cmd_done(ssm);
		else
			/* just finished sending */
			elan_cmd_read(ssm);
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		fp_dbg("transfer cancelled");
		fpi_ssm_mark_aborted(ssm, -ECANCELED);
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		fp_dbg("transfer timed out");
		fpi_ssm_mark_aborted(ssm, -ETIMEDOUT);
		break;
	default:
		fp_dbg("transfer failed: %d", transfer->status);
		elan_dev_reset(elandev);
		fpi_ssm_mark_aborted(ssm, -EIO);
	}
}

static void elan_cmd_read(struct fpi_ssm *ssm)
{
	fp_dbg("");

	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;
	int response_len = elandev->cmd->response_len;

	if (elandev->cmd->response_len == ELAN_CMD_SKIP_READ) {
		fp_dbg("skipping read, not expecting anything");
		elan_cmd_done(ssm);
		return;
	}

	if (elandev->cmd->cmd == get_image_cmd.cmd)
		/* raw data has 2-byte "pixels" and the frame is vertical */
		response_len =
		    elandev->raw_frame_width * elandev->frame_width * 2;

	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		fpi_ssm_mark_aborted(ssm, -ENOMEM);
		return;
	}
	elandev->cur_transfer = transfer;

	g_free(elandev->last_read);
	elandev->last_read = g_malloc(response_len);

	libusb_fill_bulk_transfer(transfer, dev->udev,
				  elandev->cmd->response_in, elandev->last_read,
				  response_len, elan_cmd_cb, ssm,
				  elandev->cmd_timeout);
	transfer->flags = LIBUSB_TRANSFER_FREE_TRANSFER;
	int r = libusb_submit_transfer(transfer);
	if (r < 0)
		fpi_ssm_mark_aborted(ssm, r);
}

static void elan_run_cmd(struct fpi_ssm *ssm, const struct elan_cmd *cmd,
			 int cmd_timeout)
{
	fp_dbg("%04hx", (cmd->cmd[0] << 8 | cmd->cmd[1]));

	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	elandev->cmd = cmd;
	if (cmd_timeout != -1)
		elandev->cmd_timeout = cmd_timeout;

	if (elandev->dev_type && cmd->devices
	    && !(cmd->devices & elandev->dev_type)) {
		fp_dbg("skipping for this device");
		elan_cmd_done(ssm);
		return;
	}

	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		fpi_ssm_mark_aborted(ssm, -ENOMEM);
		return;
	}
	elandev->cur_transfer = transfer;

	libusb_fill_bulk_transfer(transfer, dev->udev, ELAN_EP_CMD_OUT,
				  cmd->cmd, ELAN_CMD_LEN, elan_cmd_cb, ssm,
				  elandev->cmd_timeout);
	transfer->flags = LIBUSB_TRANSFER_FREE_TRANSFER;
	int r = libusb_submit_transfer(transfer);
	if (r < 0)
		fpi_ssm_mark_aborted(ssm, r);
}

enum deactivate_states {
	DEACTIVATE,
	DEACTIVATE_NUM_STATES,
};

static void deactivate_run_state(struct fpi_ssm *ssm)
{
	fp_dbg("");

	switch (ssm->cur_state) {
	case DEACTIVATE:
		elan_run_cmd(ssm, &stop_cmd, ELAN_CMD_TIMEOUT);
		break;
	}
}

static void deactivate_complete(struct fpi_ssm *ssm)
{
	fp_dbg("");

	struct fp_img_dev *dev = ssm->priv;

	fpi_imgdev_deactivate_complete(dev);
}

static void elan_deactivate(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;

	elan_dev_reset(elandev);

	struct fpi_ssm *ssm =
	    fpi_ssm_new(dev->dev, deactivate_run_state, DEACTIVATE_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, deactivate_complete);
}

enum capture_states {
	CAPTURE_LED_ON,
	CAPTURE_WAIT_FINGER,
	CAPTURE_READ_DATA,
	CAPTURE_CHECK_ENOUGH_FRAMES,
	CAPTURE_NUM_STATES,
};

static void capture_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	switch (ssm->cur_state) {
	case CAPTURE_LED_ON:
		elan_run_cmd(ssm, &led_on_cmd, ELAN_CMD_TIMEOUT);
		break;
	case CAPTURE_WAIT_FINGER:
		elan_run_cmd(ssm, &pre_scan_cmd, -1);
		break;
	case CAPTURE_READ_DATA:
		/* 0x55 - finger present
		 * 0xff - device not calibrated (probably) */
		if (elandev->last_read && elandev->last_read[0] == 0x55) {
			fpi_imgdev_report_finger_status(dev, TRUE);
			elan_run_cmd(ssm, &get_image_cmd, ELAN_CMD_TIMEOUT);
		} else
			fpi_ssm_mark_aborted(ssm, FP_VERIFY_RETRY);
		break;
	case CAPTURE_CHECK_ENOUGH_FRAMES:
		elan_save_img_frame(elandev);
		if (elandev->num_frames < ELAN_MAX_FRAMES) {
			/* quickly stop if finger is removed */
			elandev->cmd_timeout = ELAN_FINGER_TIMEOUT;
			fpi_ssm_jump_to_state(ssm, CAPTURE_WAIT_FINGER);
		} else {
			fpi_ssm_next_state(ssm);
		}
		break;
	}
}

static void elan_capture_async(void *data)
{
	elan_capture((struct fp_img_dev *)data);
}

static void capture_complete(struct fpi_ssm *ssm)
{
	fp_dbg("");

	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	if (elandev->deactivating)
		elan_deactivate(dev);

	/* either max frames captured or timed out waiting for the next frame */
	else if (!ssm->error
		 || (ssm->error == -ETIMEDOUT
		     && ssm->cur_state == CAPTURE_WAIT_FINGER))
		if (elandev->num_frames >= ELAN_MIN_FRAMES) {
			elan_submit_image(dev);
			fpi_imgdev_report_finger_status(dev, FALSE);
		} else {
			fp_dbg("swipe too short: want >= %d frames, got %d",
			       ELAN_MIN_FRAMES, elandev->num_frames);
			fpi_imgdev_session_error(dev,
						 FP_VERIFY_RETRY_TOO_SHORT);
		}

	/* other error
	 * It says "...session_error" but repotring 1 during verification
	 * makes it successful! */
	else
		fpi_imgdev_session_error(dev, FP_VERIFY_NO_MATCH);

	/* When enrolling the lib won't restart the capture after a stage has
	 * completed, so we need to keep feeding it images till it's had enough.
	 * But after that it can't finalize enrollemnt until this callback exits.
	 * That's why we schedule elan_capture instead of running it directly. */
	if (dev->dev->state == DEV_STATE_ENROLLING
	    && !fpi_timeout_add(10, elan_capture_async, dev))
		fpi_imgdev_session_error(dev, -ETIME);

	fpi_ssm_free(ssm);
}

static void elan_capture(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;

	elan_dev_reset(elandev);
	struct fpi_ssm *ssm =
	    fpi_ssm_new(dev->dev, capture_run_state, CAPTURE_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, capture_complete);
}

static void fpi_ssm_next_state_async(void *data)
{
	fpi_ssm_next_state((struct fpi_ssm *)data);
}

/* this function needs to have elandev->background and elandev->last_read to be
 * the calibration mean */
static int elan_need_calibration(struct elan_dev *elandev)
{
	fp_dbg("");

	unsigned short calib_mean =
	    elandev->last_read[0] * 0xff + elandev->last_read[1];
	unsigned int bg_mean = 0, delta;
	unsigned int frame_size = elandev->frame_width * elandev->frame_height;

	for (int i = 0; i < frame_size; i++)
		bg_mean += elandev->background[i];
	bg_mean /= frame_size;

	delta =
	    bg_mean > calib_mean ? bg_mean - calib_mean : calib_mean - bg_mean;

	fp_dbg("calibration mean: %d, bg mean: %d, delta: %d", calib_mean,
	       bg_mean, delta);

	return delta > ELAN_CALIBRATION_MAX_DELTA ? 1 : 0;
}

enum calibrate_states {
	CALIBRATE_GET_BACKGROUND,
	CALIBRATE_SAVE_BACKGROUND,
	CALIBRATE_GET_MEAN,
	CALIBRATE_CHECK_NEEDED,
	CALIBRATE_GET_STATUS,
	CALIBRATE_CHECK_STATUS,
	CALIBRATE_REPEAT_STATUS,
	CALIBRATE_NUM_STATES,
};

static void calibrate_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	switch (ssm->cur_state) {
	case CALIBRATE_GET_BACKGROUND:
		elan_run_cmd(ssm, &get_image_cmd, ELAN_CMD_TIMEOUT);
		break;
	case CALIBRATE_SAVE_BACKGROUND:
		elan_save_background(elandev);
		if (elandev->fw_ver < ELAN_MIN_CALIBRATION_FW) {
			fp_dbg("FW does not support calibration");
			fpi_ssm_mark_completed(ssm);
		} else
			fpi_ssm_next_state(ssm);
		break;
	case CALIBRATE_GET_MEAN:
		elan_run_cmd(ssm, &get_calib_mean_cmd, ELAN_CMD_TIMEOUT);
		break;
	case CALIBRATE_CHECK_NEEDED:
		if (elan_need_calibration(elandev)) {
			elandev->calib_status = 0;
			fpi_ssm_next_state(ssm);
		} else
			fpi_ssm_mark_completed(ssm);
		break;
	case CALIBRATE_GET_STATUS:
		elandev->calib_atts_left -= 1;
		if (elandev->calib_atts_left)
			elan_run_cmd(ssm, &get_calib_status_cmd,
				     ELAN_CMD_TIMEOUT);
		else {
			fp_dbg("calibration failed");
			fpi_ssm_mark_aborted(ssm, -1);
		}
		break;
	case CALIBRATE_CHECK_STATUS:
		/* 0x01 - retry, 0x03 - ok
		 * It appears that when reading the response soon after 0x4023 the device
		 * can return 0x03, and only after some time (up to 100 ms) the response
		 * changes to 0x01. It stays that way for some time and then changes back
		 * to 0x03. Because of this we don't just expect 0x03, we want to see 0x01
		 * first. This is to make sure that a full calibration loop has completed */
		fp_dbg("calibration status: 0x%02x", elandev->last_read[0]);
		if (elandev->calib_status == 0x01
		    && elandev->last_read[0] == 0x03) {
			elandev->calib_status = 0x03;
			fpi_ssm_jump_to_state(ssm, CALIBRATE_GET_BACKGROUND);
		} else {
			if (elandev->calib_status == 0x00
			    && elandev->last_read[0] == 0x01)
				elandev->calib_status = 0x01;
			if (!fpi_timeout_add(50, fpi_ssm_next_state_async, ssm))
				fpi_ssm_mark_aborted(ssm, -ETIME);
		}
		break;
	case CALIBRATE_REPEAT_STATUS:
		fpi_ssm_jump_to_state(ssm, CALIBRATE_GET_STATUS);
		break;
	}
}

static void calibrate_complete(struct fpi_ssm *ssm)
{
	fp_dbg("");

	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	if (elandev->deactivating)
		elan_deactivate(dev);
	else if (ssm->error)
		fpi_imgdev_activate_complete(dev, ssm->error);
	else {
		fpi_imgdev_activate_complete(dev, 0);
		elan_capture(dev);
	}

	fpi_ssm_free(ssm);
}

static void elan_calibrate(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;

	elan_dev_reset(elandev);
	elandev->calib_atts_left = ELAN_CALIBRATION_ATTEMPTS;

	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, calibrate_run_state,
					  CALIBRATE_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, calibrate_complete);
}

enum activate_states {
	ACTIVATE_GET_FW_VER,
	ACTIVATE_SET_FW_VER,
	ACTIVATE_GET_SENSOR_DIM,
	ACTIVATE_SET_SENSOR_DIM,
	ACTIVATE_CMD_1,
	ACTIVATE_NUM_STATES,
};

static void activate_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	switch (ssm->cur_state) {
	case ACTIVATE_GET_FW_VER:
		elan_run_cmd(ssm, &get_fw_ver_cmd, ELAN_CMD_TIMEOUT);
		break;
	case ACTIVATE_SET_FW_VER:
		elandev->fw_ver =
		    (elandev->last_read[0] << 8 | elandev->last_read[1]);
		fp_dbg("FW ver 0x%04hx", elandev->fw_ver);
		fpi_ssm_next_state(ssm);
		break;
	case ACTIVATE_GET_SENSOR_DIM:
		elan_run_cmd(ssm, &get_sensor_dim_cmd, ELAN_CMD_TIMEOUT);
		break;
	case ACTIVATE_SET_SENSOR_DIM:
		elandev->frame_width = elandev->last_read[2];
		elandev->raw_frame_width = elandev->last_read[0];
		if (elandev->raw_frame_width < ELAN_MAX_FRAME_HEIGHT)
			elandev->frame_height = elandev->raw_frame_width;
		else
			elandev->frame_height = ELAN_MAX_FRAME_HEIGHT;
		/* see elan_save_frame for why it's width x raw_width */
		fp_dbg("sensor dimensions, WxH: %dx%d", elandev->frame_width,
		       elandev->raw_frame_width);
		fpi_ssm_next_state(ssm);
		break;
	case ACTIVATE_CMD_1:
		/* TODO: find out what this does, if we need it */
		elan_run_cmd(ssm, &activate_cmd_1, ELAN_CMD_TIMEOUT);
		break;
	}
}

static void activate_complete(struct fpi_ssm *ssm)
{
	fp_dbg("");

	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

	if (elandev->deactivating)
		elan_deactivate(dev);
	else if (ssm->error)
		fpi_imgdev_activate_complete(dev, ssm->error);
	else
		elan_calibrate(dev);

	fpi_ssm_free(ssm);
}

static void elan_activate(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;
	elan_dev_reset(elandev);

	struct fpi_ssm *ssm =
	    fpi_ssm_new(dev->dev, activate_run_state, ACTIVATE_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, activate_complete);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	fp_dbg("");

	struct elan_dev *elandev;

	int r = libusb_claim_interface(dev->udev, 0);
	if (r < 0) {
		fp_err("could not claim interface 0: %s", libusb_error_name(r));
		return r;
	}

	dev->priv = elandev = g_malloc0(sizeof(struct elan_dev));
	elandev->dev_type = driver_data;
	elandev->background = NULL;
	elandev->process_frame = elan_process_frame_thirds;

	switch (driver_data) {
	case ELAN_0907:
		elandev->process_frame = elan_process_frame_linear;
		break;
	}

	fpi_imgdev_open_complete(dev, 0);
	return 0;
}

static void dev_deinit(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;

	elan_dev_reset(elandev);
	g_free(elandev->background);
	g_free(elandev);
	libusb_release_interface(dev->udev, 0);
	fpi_imgdev_close_complete(dev);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	fp_dbg("");
	elan_activate(dev);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	fp_dbg("");

	struct elan_dev *elandev = dev->priv;

	elandev->deactivating = TRUE;

	if (elandev->cur_transfer)
		libusb_cancel_transfer(elandev->cur_transfer);
	else
		elan_deactivate(dev);
}

struct fp_img_driver elan_driver = {
	.driver = {
		   .id = ELAN_ID,
		   .name = FP_COMPONENT,
		   .full_name = "ElanTech Fingerprint Sensor",
		   .id_table = elan_id_table,
		   .scan_type = FP_SCAN_TYPE_SWIPE,
		   },
	.flags = 0,

	.bz3_threshold = 22,

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};
