/*
 * Elan driver for libfprint
 *
 * Copyright (C) 2017 Igor Filatov <ia.filatov@gmail.com>
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

#include "elan.h"
#include "driver_ids.h"

unsigned char elan_get_pixel(struct fpi_frame_asmbl_ctx *ctx,
        struct fpi_frame *frame, unsigned int x, unsigned int y)
{
    return frame->data[x + y * ELAN_FRAME_WIDTH];
}

static struct fpi_frame_asmbl_ctx assembling_ctx = {
	.frame_width = ELAN_FRAME_WIDTH,
	.frame_height = ELAN_FRAME_HEIGHT,
	.image_width = ELAN_IMG_WIDTH,
	.get_pixel = elan_get_pixel,
};

struct elan_dev {
	gboolean deactivating;
	gboolean finger_present;
	const struct elan_cmd *cmds;
    size_t cmds_len;
	int cmd_idx;
    int cmd_timeout;
    unsigned char *last_read;
    GSList *frames;
    int num_frames;
};

static void elan_dev_reset(struct elan_dev *elandev)
{
    fp_dbg("");

    elandev->finger_present = FALSE;

    elandev->cmds = NULL;
	elandev->cmd_idx = 0;
    elandev->cmd_timeout = ELAN_BULK_TIMEOUT;

    g_free(elandev->last_read);
    elandev->last_read = NULL;

    g_slist_free_full(elandev->frames, g_free);
    elandev->frames = NULL;
    elandev->num_frames = 0;
}

static void elan_deactivate(struct fp_img_dev *dev)
{
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    elan_dev_reset(elandev);
	elandev->deactivating = FALSE;
    fpi_imgdev_deactivate_complete(dev);
}

static void elan_set_finger_present(struct fp_img_dev *dev, gboolean status)
{
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    if (elandev->finger_present != status)
        fpi_imgdev_report_finger_status(dev, status);
    elandev->finger_present = status;
}

static void elan_save_frame(struct fp_img_dev *dev)
{
    struct elan_dev *elandev = dev->priv;
    unsigned short *frame = g_malloc(ELAN_FRAME_SIZE * 2);

    fp_dbg("");

    /* Raw images are vertical and perpendicular to swipe direction,
     * which means we need to make them horizontal before assembling. */
    for (int y = 0; y < ELAN_RAW_FRAME_HEIGHT; y++)
        for (int x = ELAN_FRAME_MARGIN;
                x < ELAN_RAW_FRAME_WIDTH - ELAN_FRAME_MARGIN ; x++) {
            int frame_idx = y + (x - ELAN_FRAME_MARGIN) * ELAN_RAW_FRAME_HEIGHT;
            int raw_idx = x + y * ELAN_RAW_FRAME_WIDTH;
            frame[frame_idx] = ((unsigned short*)elandev->last_read)[raw_idx];
        }

    elandev->frames = g_slist_prepend(elandev->frames, frame);
    elandev->num_frames += 1;
}

/* Transform raw sesnsor data to normalized 8-bit grayscale image. */
static void elan_process_frame(unsigned short *raw_frame, GSList **frames)
{
    struct fpi_frame *frame = g_malloc(ELAN_FRAME_SIZE
            + sizeof(struct fpi_frame));

    fp_dbg("");

    unsigned short min = 0xffff, max = 0;
    for (int i = 0; i < ELAN_FRAME_SIZE; i++) {
        if (raw_frame[i] < min)
            min = raw_frame[i];
        if (raw_frame[i] > max)
            max = raw_frame[i];
    }

    unsigned short px;
    for (int i = 0; i < ELAN_FRAME_SIZE; i++) {
        px = raw_frame[i];
        if (px <= min)
            px = 0;
        else if (px >= max)
            px = 0xff;
        else
            px = (px - min) * 0xff / (max - min);
        frame->data[i] = (unsigned char)px;
    }

    *frames = g_slist_prepend(*frames, frame);
}

static void elan_submit_image(struct fp_img_dev *dev)
{
    struct elan_dev *elandev = dev->priv;
    GSList *frames = NULL;
    struct fp_img *img;
 
    fp_dbg("");
    
    for (int i = 0; i < ELAN_SKIP_LAST_FRAMES; i++)
        elandev->frames = g_slist_next(elandev->frames);

    g_slist_foreach(elandev->frames, (GFunc)elan_process_frame, &frames);

    fpi_do_movement_estimation(&assembling_ctx, frames,
            elandev->num_frames - ELAN_SKIP_LAST_FRAMES);
    img = fpi_assemble_frames(&assembling_ctx, frames,
            elandev->num_frames - ELAN_SKIP_LAST_FRAMES);
    img->flags |= FP_IMG_PARTIAL;
	fpi_imgdev_image_captured(dev, img);
}

static void elan_cmd_done(struct fpi_ssm *ssm)
{
    struct fp_img_dev *dev = ssm->priv;
    struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    elandev->cmd_idx += 1;
    if (elandev->cmd_idx < elandev->cmds_len)
        elan_run_next_cmd(ssm);
    else
        fpi_ssm_next_state(ssm);
}

static void elan_cmd_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;
    struct fp_img_dev *dev = ssm->priv;
    struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        fp_dbg("transfer not completed");
        fpi_ssm_mark_aborted(ssm, -EIO);
    }
    else if (transfer->length != transfer->actual_length)
    {
        fp_dbg("unexpected transfer length");
        fpi_ssm_mark_aborted(ssm, -EPROTO);
    }
    else if (transfer->endpoint & LIBUSB_ENDPOINT_IN)
        /* just finished receiving */
        elan_cmd_done(ssm);
    else {
        /* just finished sending */
        if(elandev->cmds[elandev->cmd_idx].response_len)
            elan_cmd_read(ssm);
        else
            elan_cmd_done(ssm);
    }
}

static void elan_cmd_read(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    if (!transfer) {
        fpi_ssm_mark_aborted(ssm, -ENOMEM);
        return;
    }
    g_free(elandev->last_read);
    elandev->last_read = g_malloc(elandev->cmds[elandev->cmd_idx].response_len);
    libusb_fill_bulk_transfer(transfer, dev->udev,
            elandev->cmds[elandev->cmd_idx].response_in, elandev->last_read,
            elandev->cmds[elandev->cmd_idx].response_len, elan_cmd_cb, ssm,
            elandev->cmd_timeout);
    transfer->flags = LIBUSB_TRANSFER_FREE_TRANSFER;
    int r = libusb_submit_transfer(transfer);
    if (r < 0)
        fpi_ssm_mark_aborted(ssm, r);

}

static void elan_run_next_cmd(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    if (!transfer) {
        fpi_ssm_mark_aborted(ssm, -ENOMEM);
        return;
    }
    libusb_fill_bulk_transfer(transfer, dev->udev, ELAN_EP_CMD_OUT,
            (unsigned char*)elandev->cmds[elandev->cmd_idx].cmd,
            ELAN_CMD_LEN, elan_cmd_cb, ssm, elandev->cmd_timeout);
    transfer->flags = LIBUSB_TRANSFER_FREE_TRANSFER;
    int r = libusb_submit_transfer(transfer);
    if (r < 0)
        fpi_ssm_mark_aborted(ssm, r);

}

static void elan_run_cmds(struct fpi_ssm *ssm, const struct elan_cmd *cmds,
        size_t cmds_len, int cmd_timeout)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    elandev->cmds = cmds;
    elandev->cmds_len = cmds_len;
    elandev->cmd_idx = 0;
    if (cmd_timeout != -1)
        elandev->cmd_timeout = cmd_timeout;
    elan_run_next_cmd(ssm);
}

enum capture_states {
    CAPTURE_START,
    CAPTURE_WAIT_FINGER,
    CAPTURE_READ_DATA,
    CAPTURE_SAVE_FRAME,
    CAPTURE_SUBMIT_IMAGE,
    CAPTURE_END,
    CAPTURE_NUM_STATES,
};

static void elan_capture_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;
    
	switch (ssm->cur_state) {
	case CAPTURE_START:
        elan_run_cmds(ssm, capture_start_cmds, capture_start_cmds_len,
                ELAN_BULK_TIMEOUT);
	break;
	case CAPTURE_WAIT_FINGER:
        elan_run_cmds(ssm, capture_wait_finger_cmds,
                capture_wait_finger_cmds_len, -1);
	break;
	case CAPTURE_READ_DATA:
        /* 0x55 - finger present
         * 0xff - device not calibrated
         * timeout in CAPTURE_WAIT_FINGER -
         *         finger not present (will resume in CAPTURE_SUBMIT_IMAGE) */
        if (elandev->last_read && elandev->last_read[0] == 0x55) {
            elan_set_finger_present(dev, TRUE);
            elan_run_cmds(ssm, read_cmds, read_cmds_len, ELAN_BULK_TIMEOUT);
        }
        else
            fpi_ssm_mark_aborted(ssm, FP_VERIFY_RETRY);
	break;
	case CAPTURE_SAVE_FRAME:
        elan_save_frame(dev);
        if (!elandev->finger_present || elandev->num_frames == ELAN_MAX_FRAMES)
            fpi_ssm_next_state(ssm);
        else {
            /* quickly stop if finger is removed */
            elandev->cmd_timeout = ELAN_FINGER_TIMEOUT;
            fpi_ssm_jump_to_state(ssm, CAPTURE_WAIT_FINGER);
        }
    break;
	case CAPTURE_SUBMIT_IMAGE:
        elan_submit_image(dev);
        fpi_ssm_next_state(ssm);
    break;
	case CAPTURE_END:
        elan_set_finger_present(dev, FALSE);
        elan_run_cmds(ssm, capture_end_cmds, capture_end_cmds_len,
                ELAN_BULK_TIMEOUT);
    }
}

static void capture_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    if (elandev->deactivating)
        elan_deactivate(dev);
    else
        elan_capture(dev);

    fpi_ssm_free(ssm);
}

static void capture_check(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    /* CAPTURE_END done */
    if (!ssm->error)
        capture_complete(ssm);

    /* one more led turn off command in case it's still on */
    else if (ssm->cur_state != CAPTURE_WAIT_FINGER) {
        elan_dev_reset(elandev);
        fpi_ssm_start_at_state(ssm, capture_complete, CAPTURE_END);
        fpi_imgdev_session_error(dev, ssm->error);
    }

    /* finger removed, got enough frames */
    else if (elandev->num_frames >= ELAN_MIN_FRAMES)
        fpi_ssm_start_at_state(ssm, capture_complete, CAPTURE_SUBMIT_IMAGE);

    /* finger wasn't placed on scanner */
    else if (elandev->cmd_timeout == ELAN_BULK_TIMEOUT) {
        /* it says "error" but repotring 1 during verification
         * makes is successful! */
        fpi_imgdev_session_error(dev, FP_VERIFY_NO_MATCH);
        fpi_ssm_start_at_state(ssm, capture_complete, CAPTURE_END);
    }

    /* not enough frames - finger removed too early */
    else {
        fpi_imgdev_session_error(dev, FP_VERIFY_RETRY_TOO_SHORT);
        fpi_ssm_start_at_state(ssm, capture_complete, CAPTURE_END);
    }

}

static void elan_capture(struct fp_img_dev *dev)
{
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    elan_dev_reset(elandev);

	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, elan_capture_run_state,
            CAPTURE_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, capture_check);
}

enum calibrate_states {
    CALIBRATE_START_1,
    CALIBRATE_READ_DATA_1,
    CALIBRATE_END_1,
    CALIBRATE_START_2,
    CALIBRATE_READ_DATA_2,
    CALIBRATE_END_2,
    CALIBRATE_NUM_STATES,
};

static void elan_calibrate_run_state(struct fpi_ssm *ssm)
{
	switch (ssm->cur_state) {
	case CALIBRATE_START_1:
	case CALIBRATE_START_2:
        elan_run_cmds(ssm, calibrate_start_cmds, calibrate_start_cmds_len,
                ELAN_BULK_TIMEOUT);
	break;
	case CALIBRATE_READ_DATA_1:
	case CALIBRATE_READ_DATA_2:
        elan_run_cmds(ssm, read_cmds, read_cmds_len, ELAN_BULK_TIMEOUT);
	break;
	case CALIBRATE_END_1:
	case CALIBRATE_END_2:
        elan_run_cmds(ssm, calibrate_end_cmds, calibrate_end_cmds_len,
                ELAN_BULK_TIMEOUT);
    }
}

static void calibrate_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    fpi_imgdev_activate_complete(dev, ssm->error);

    if (elandev->deactivating)
        elan_deactivate(dev);
    else if (ssm->error)
        fpi_imgdev_session_error(dev, ssm->error);
    else
        elan_capture(dev);
    fpi_ssm_free(ssm);
}

static void elan_calibrate(struct fp_img_dev *dev)
{
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    elan_dev_reset(elandev);

	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, elan_calibrate_run_state,
            CALIBRATE_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, calibrate_complete);
}

enum init_states {
    INIT_START,
    INIT_READ_DATA,
    INIT_END,
    INIT_NUM_STATES,
};

static void elan_init_run_state(struct fpi_ssm *ssm)
{
	switch (ssm->cur_state) {
	case INIT_START:
        elan_run_cmds(ssm, init_start_cmds, init_start_cmds_len,
                ELAN_BULK_TIMEOUT);
	break;
	case INIT_READ_DATA:
        elan_run_cmds(ssm, read_cmds, read_cmds_len, ELAN_BULK_TIMEOUT);
	break;
	case INIT_END:
        elan_run_cmds(ssm, init_end_cmds, init_end_cmds_len,
                ELAN_BULK_TIMEOUT);
    }
}

static void init_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    if (elandev->deactivating)
        elan_deactivate(dev);
    else if (ssm->error)
        fpi_imgdev_session_error(dev, ssm->error);
    else
        elan_calibrate(dev);
    fpi_ssm_free(ssm);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
    fp_dbg("");

	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

    elan_dev_reset(elandev);
	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, elan_init_run_state,
            INIT_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, init_complete);

	return 0;
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	struct elan_dev *elandev;
	int r;

    fp_dbg("");

	r = libusb_claim_interface(dev->udev, 0);
	if (r < 0) {
		fp_err("could not claim interface 0: %s", libusb_error_name(r));
		return r;
	}

	dev->priv = elandev = g_malloc0(sizeof(struct elan_dev));
	fpi_imgdev_open_complete(dev, 0);
	return 0;
}

static void dev_deinit(struct fp_img_dev *dev)
{
    fp_dbg("");

	g_free(dev->priv);
	libusb_release_interface(dev->udev, 0);
	fpi_imgdev_close_complete(dev);
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	struct elan_dev *elandev = dev->priv;

    fp_dbg("");

	elandev->deactivating = TRUE;
}

static const struct usb_id id_table[] = {
	{ .vendor = 0x04f3, .product = 0x0907 },
	{ 0, 0, 0, },
};

struct fp_img_driver elan_driver = {
	.driver = {
		.id = ELAN_ID,
		.name = FP_COMPONENT,
		.full_name = "ElanTech Fingerprint Sensor",
		.id_table = id_table,
		.scan_type = FP_SCAN_TYPE_SWIPE,
	},
	.flags = 0,
	.img_height = -1,
	.img_width = ELAN_IMG_WIDTH,

	.bz3_threshold = 20,

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};
