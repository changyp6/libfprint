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

#ifndef __ELAN_H
#define __ELAN_H

#include <string.h>
#include <libusb.h>


#define ELAN_RAW_FRAME_HEIGHT 144
#define ELAN_RAW_FRAME_WIDTH 64
/* raw data has 2-byte "pixels" */
#define ELAN_RAW_FRAME_SIZE (ELAN_RAW_FRAME_WIDTH * ELAN_RAW_FRAME_HEIGHT * 2)

/* number of pixels to discard on left and right (along raw image height) */
#define ELAN_FRAME_MARGIN 12

#define ELAN_FRAME_WIDTH ELAN_RAW_FRAME_HEIGHT
#define ELAN_FRAME_HEIGHT (ELAN_RAW_FRAME_WIDTH - 2 * ELAN_FRAME_MARGIN)
#define ELAN_FRAME_SIZE (ELAN_FRAME_WIDTH * ELAN_FRAME_HEIGHT)

#define ELAN_IMG_WIDTH (ELAN_FRAME_WIDTH * 3 / 2) 
#define ELAN_MIN_FRAMES 7
#define ELAN_MAX_FRAMES 30
/* lasf frame(s) before the finger has been lifted can be bad */
#define ELAN_SKIP_LAST_FRAMES 1

#define ELAN_CMD_LEN 0x2
#define ELAN_EP_CMD_OUT (0x1 | LIBUSB_ENDPOINT_OUT)
#define ELAN_EP_CMD_IN (0x3 | LIBUSB_ENDPOINT_IN)
#define ELAN_EP_IMG_IN (0x2 | LIBUSB_ENDPOINT_IN)
#define ELAN_BULK_TIMEOUT 10000
#define ELAN_FINGER_TIMEOUT 200


struct elan_cmd {
	unsigned char cmd[ELAN_CMD_LEN];
	int response_len;
    int response_in;
};

static const struct elan_cmd init_start_cmds[] = {
	{
		.cmd = { 0x00, 0x0c },
		.response_len = 0x4,
        .response_in = ELAN_EP_CMD_IN
	},
	{
		.cmd = { 0x40, 0x19 },
		.response_len = 0x2,
        .response_in = ELAN_EP_CMD_IN
	}, {
		.cmd = { 0x40, 0x2a },
		.response_len = 0x2,
        .response_in = ELAN_EP_CMD_IN
	},
};

static const size_t init_start_cmds_len = array_n_elements(init_start_cmds);

static const struct elan_cmd read_cmds[] = {
	{
		.cmd = { 0x00, 0x09 },
		.response_len = ELAN_RAW_FRAME_SIZE,
        .response_in = ELAN_EP_IMG_IN
	},
};

const size_t read_cmds_len = array_n_elements(read_cmds);

/* issued after data reads during init and calibration */
static const struct elan_cmd init_end_cmds[] = {
	{
		.cmd = { 0x40, 0x24 },
		.response_len = 0x2,
        .response_in = ELAN_EP_CMD_IN
	},
};

static const size_t init_end_cmds_len = array_n_elements(init_end_cmds);

/* same command 2 times
 * original driver may observe return value to determine how many times it
 * should be repeated */
static const struct elan_cmd calibrate_start_cmds[] = {
	{
		.cmd = { 0x40, 0x23 },
		.response_len = 0x1,
        .response_in = ELAN_EP_CMD_IN
	},
	{
		.cmd = { 0x40, 0x23 },
		.response_len = 0x1,
        .response_in = ELAN_EP_CMD_IN
	},
};

static const size_t calibrate_start_cmds_len = array_n_elements(
        calibrate_start_cmds);

/* issued after data reads during init and calibration */
static const struct elan_cmd calibrate_end_cmds[] = {
	{
		.cmd = { 0x40, 0x24 },
		.response_len = 0x2,
        .response_in = ELAN_EP_CMD_IN
	},
};

static const size_t calibrate_end_cmds_len = array_n_elements(
        calibrate_end_cmds);

static const struct elan_cmd capture_start_cmds[] = {
    /* led on */
	{
		.cmd = { 0x40, 0x31 },
		.response_len = 0x0,
        .response_in = ELAN_EP_CMD_IN
	},
};

static size_t capture_start_cmds_len = array_n_elements(capture_start_cmds);
    
static const struct elan_cmd capture_wait_finger_cmds[] = {
    /* wait for finger
     * subsequent read will block until finger is placed on the reader */
	{
		.cmd = { 0x40, 0x3f },
		.response_len = 0x1,
        .response_in = ELAN_EP_CMD_IN
	},
};

static size_t capture_wait_finger_cmds_len = array_n_elements(
        capture_wait_finger_cmds);

static const struct elan_cmd capture_end_cmds[] = {
    /* led off */
	{
		.cmd = { 0x00, 0x0b },
		.response_len = 0x0,
        .response_in = ELAN_EP_CMD_IN
	},
};

static const size_t capture_end_cmds_len = array_n_elements(capture_end_cmds);


static void elan_cmd_cb(struct libusb_transfer *transfer);
static void elan_cmd_read(struct fpi_ssm *ssm);
static void elan_run_next_cmd(struct fpi_ssm *ssm);

static void elan_capture(struct fp_img_dev *dev);

#endif
