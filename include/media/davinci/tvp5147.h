/*
 * Copyright (C) 2007 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* tvp5147.h file */

#ifndef TVP5147_H
#define TVP5147_H

#ifdef __KERNEL__

/* Kernel Header files */
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>
#include <media/davinci/vid_decoder_if.h>

#endif				/* __KERNEL__ */

#define VPFE_STD_625_50_SQP ((v4l2_std_id)0x0000000100000000ULL)
#define VPFE_STD_525_60_SQP ((v4l2_std_id)0x0000000200000000ULL)
#define VPFE_STD_AUTO       ((v4l2_std_id)0x0000000400000000ULL)
#define VPFE_STD_AUTO_SQP   ((v4l2_std_id)0x0000000800000000ULL)

#define V4L2_STD_TVP5147_ALL         (V4L2_STD_525_60 | V4L2_STD_625_50 |\
	VPFE_STD_AUTO | VPFE_STD_625_50_SQP |\
	VPFE_STD_525_60_SQP | VPFE_STD_AUTO_SQP)

typedef struct {
	v4l2_std_id std;
	int inputidx;
	struct v4l2_format fmt;
} tvp5147_params;

#ifdef __KERNEL__

#define TVP5147_NUM_CHANNELS                    2

#define TVP5147_MAX_VBI_SERVICES		2	/* This gives maximum 
							   number of VBI service 
							   supported */

typedef enum {
	TVP5147_MODE_INV = -1,
	TVP5147_MODE_AUTO = 0,	/* autoswitch mode (default)   */
	TVP5147_MODE_NTSC = 1,	/* (M, J) NTSC      525-line   */
	TVP5147_MODE_PAL = 2,	/* (B, D, G, H, I, N) PAL      */
	TVP5147_MODE_PAL_M = 3,	/* (M) PAL          525-line   */
	TVP5147_MODE_PAL_CN = 4,	/* (Combination-N) PAL         */
	TVP5147_MODE_NTSC_443 = 5,	/* NTSC 4.43        525-line   */
	TVP5147_MODE_SECAM = 6,	/* SECAM                       */
	TVP5147_MODE_PAL_60 = 7,	/* PAL 60          525-line    */
	TVP5147_MODE_AUTO_SQP = 8,	/* autoswitch mode (default)   */
	TVP5147_MODE_NTSC_SQP = 9,	/* (M, J) NTSC      525-line   */
	TVP5147_MODE_PAL_SQP = 0xA,	/* (B, D, G, H, I, N) PAL      */
	TVP5147_MODE_PAL_M_SQP = 0xB,	/* (M) PAL          525-line   */
	TVP5147_MODE_PAL_CN_SQP = 0xC,	/* (Combination-N) PAL         */
	TVP5147_MODE_NTSC_443_SQP = 0xD,	/* NTSC 4.43 525-line          */
	TVP5147_MODE_SECAM_SQP = 0xE,	/* SECAM                       */
	TVP5147_MODE_PAL_60_SQP = 0xF,	/* PAL 60          525-line    */
} tvp5147_mode;

/* decoder standard related strctures */
#define TVP5147_MAX_NO_INPUTS           (1)
#define TVP5147_MAX_NO_STANDARDS        (6)
#define TVP5147_MAX_NO_CONTROLS         (5)
/* for AUTO mode, stdinfo structure will not be filled */
#define TVP5147_STANDARD_INFO_SIZE      (TVP5147_MAX_NO_STANDARDS - 2)

#define TVP5147_MAX_NO_MODES           (3)

struct tvp5147_control_info {
	int register_address;
	struct v4l2_queryctrl query_control;
};

struct tvp5147_config {
	int no_of_inputs;
	struct {
		int input_type;
		u8 lock_mask;
		struct v4l2_input input_info;
		int no_of_standard;
		struct v4l2_standard *standard;
		v4l2_std_id def_std;
		 tvp5147_mode(*mode)[TVP5147_MAX_NO_MODES];
		int no_of_controls;
		struct tvp5147_control_info *controls;
	} input[TVP5147_MAX_NO_INPUTS];
	struct v4l2_sliced_vbi_cap sliced_cap;
	u8 num_services;
};

struct tvp5147_channel {
	struct {
		struct i2c_client client;
		struct i2c_driver driver;
		u32 i2c_addr;
		int i2c_registration;
	} i2c_dev;
	struct decoder_device *dec_device;
	tvp5147_params params;
};

struct tvp5147_service_data_reg {
	u32 service;
	struct {
		u8 addr[3];
	} field[2];
	u8 bytestoread;
};

struct tvp5147_sliced_reg {
	u32 service;
	u8 line_addr_value;
	u16 line_start, line_end;
	struct {
		u8 fifo_line_addr[3];
		u8 fifo_mode_value;
	} field[2];
	v4l2_std_id std;
	struct {
		u8 index;
		u16 value;
	} service_line[2];
};

/* Defines for TVP5147 register address */

#define TVP5147_INPUT_SEL                                       (0x00)
#define TVP5147_AFE_GAIN_CTRL                                   (0x01)
#define TVP5147_VIDEO_STD                                       (0x02)
#define TVP5147_OPERATION_MODE                                  (0x03)
#define TVP5147_AUTOSWT_MASK                                    (0x04)
#define TVP5147_COLOR_KILLER                                    (0x05)
#define TVP5147_LUMA_CONTROL1                                   (0x06)
#define TVP5147_LUMA_CONTROL2                                   (0x07)
#define TVP5147_LUMA_CONTROL3                                   (0x08)
#define TVP5147_BRIGHTNESS                                      (0x09)
#define TVP5147_CONTRAST                                        (0x0A)
#define TVP5147_SATURATION                                      (0x0B)
#define TVP5147_HUE                                             (0x0C)
#define TVP5147_CHROMA_CONTROL1                                 (0x0D)
#define TVP5147_CHROMA_CONTROL2                                 (0x0E)
#define TVP5147_OUTPUT1                                         (0x33)
#define TVP5147_OUTPUT2                                         (0x34)
#define TVP5147_OUTPUT3                                         (0x35)
#define TVP5147_OUTPUT4                                         (0x36)
#define TVP5147_OUTPUT5                                         (0x37)
#define TVP5147_OUTPUT6                                         (0x38)
#define TVP5147_CLEAR_LOST_LOCK                                 (0x39)
#define TVP5147_STATUS1                                         (0x3A)
#define TVP5147_VID_STD_STATUS                                  (0x3F)
#define TVP5147_FIFO_OUTPUT_CTRL				(0xC0)

/* masks */

#define TVP5147_LOST_LOCK_MASK                                  (0x10)
/* mask to enable autoswitch for all standards*/

#define TVP5147_AUTOSWITCH_MASK                                 (0x7F)
#define TVP5147_COMPOSITE_INPUT                                 (0x05)
#define TVP5147_SVIDEO_INPUT                                    (0x46)

/* DEFAULTS */

#define TVP5147_OPERATION_MODE_RESET				(0x1)
#define TVP5147_OPERATION_MODE_DEFAULT                          (0x0)
#define TVP5147_AFE_GAIN_CTRL_DEFAULT                           (0x0F)
#define TVP5147_COLOR_KILLER_DEFAULT                            (0x10)
#define TVP5147_LUMA_CONTROL1_DEFAULT                           (0x10)
#define TVP5147_LUMA_CONTROL2_DEFAULT                           (0x00)
#define TVP5147_LUMA_CONTROL3_DEFAULT                           (0x02)
#define TVP5147_BRIGHTNESS_DEFAULT                              (0x80)
#define TVP5147_CONTRAST_DEFAULT                                (0x80)
#define TVP5147_SATURATION_DEFAULT                              (0x80)
#define TVP5147_HUE_DEFAULT                                     (0x00)
#define TVP5147_CHROMA_CONTROL1_DEFAULT                         (0x00)
#define TVP5147_CHROMA_CONTROL2_DEFAULT                         (0x0E)
#define TVP5147_OUTPUT1_DEFAULT					(0x40)
#define TVP5147_OUTPUT2_DEFAULT					(0x11)
#define TVP5147_OUTPUT3_DEFAULT					(0xFF)
#define TVP5147_OUTPUT4_DEFAULT					(0xFF)
#define TVP5147_OUTPUT5_DEFAULT					(0xFF)
#define TVP5147_OUTPUT6_DEFAULT					(0xFF)
#define TVP5147_FIFO_OUTPUT_CTRL_DEFAULT			(0x01)

#define TVP5147_VBUS_ADDRESS_ACCESS0				(0xE8)
#define TVP5147_VBUS_ADDRESS_ACCESS1				(0xE9)
#define TVP5147_VBUS_ADDRESS_ACCESS2				(0xEA)
#define TVP5147_VBUS_DATA_ACCESS				(0xE0)
#define TVP5147_VBUS_DATA_ACCESS_AUTO_INCR			(0xE1)

#define TVP5147_LINE_ADDRESS_START				(0x80)
#define TVP5147_LINE_ADDRESS_MIDDLE				(0x06)
#define TVP5147_LINE_ADDRESS_END				(0x00)

#define TVP5147_LINE_ADDRESS_DEFAULT				(0x00)
#define TVP5147_LINE_MODE_DEFAULT				(0xFF)

#define TVP5147_VDP_LINE_START					(0xD6)
#define TVP5147_VDP_LINE_STOP					(0xD7)

#define TVP5147_VBI_NUM_SERVICES				(3)
#define TVP5147_SLICED_BUF_SIZE					(128)

#endif				/* __KERNEL__ */

#endif				/* TVP5147_H */
