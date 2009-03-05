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
/* tvp7002.h file */
#ifndef TVP7002_H
#define TVP7002_H

#ifdef __KERNEL__
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>
#include <media/davinci/vid_decoder_if.h>
#endif				/* __KERNEL__ */

#define V4L2_STD_TVP7002_ALL        (V4L2_STD_720P_60 | \
					V4L2_STD_720P_50 | \
					V4L2_STD_1080I_60 | \
					V4L2_STD_1080I_50 | \
					V4L2_STD_525P_60)

/* enum */
typedef enum {
	TVP7002_MODE_480i_30FPS = 0,
	TVP7002_MODE_576i_25FPS,
	TVP7002_MODE_480p_30FPS,
	TVP7002_MODE_576p_25FPS,
	TVP7002_MODE_720p_30FPS,
	TVP7002_MODE_1080i_30FPS,
	TVP7002_MODE_1080p_30FPS,
	TVP7002_MODE_1080i_25FPS
} tvp7002_mode;

typedef enum {
	TVP7002_VCOEFF_1,
	TVP7002_VCOEFF_2,
	TVP7002_VCOEFF_4,
	TVP7002_VCOEFF_8,
	TVP7002_VCOEFF_16,
	TVP7002_VCOEFF_32,
	TVP7002_VCOEFF_64,
	TVP7002_VCOEFF_128,
	TVP7002_VCOEFF_256,
	TVP7002_VCOEFF_512,
	TVP7002_VCOEFF_1024
} tvp7002_alc_vertical_coeff;

typedef enum {
	TVP7002_HCOEFF_2,
	TVP7002_HCOEFF_4,
	TVP7002_HCOEFF_8,
	TVP7002_HCOEFF_16,
	TVP7002_HCOEFF_32,
	TVP7002_HCOEFF_64,
	TVP7002_HCOEFF_128,
	TVP7002_HCOEFF_256,
	TVP7002_HCOEFF_512
} tvp7002_alc_horizontal_coeff;

typedef enum {
	HPLL_POST_DIVIDE_1 = 0,
	HPLL_POST_DIVIDE_2
} tvp7002_post_divider;

typedef enum {
	VCO_GAIN_ULTRA_LOW = 0,
	VCO_GAIN_LOW,
	VCO_GAIN_MEDIUM,
	VCO_GAIN_HIGH
} tvp7002_vco_gain;

typedef enum {
	CP_CURRENT_SMALL,
	CP_CURRENT_DEFAULT,
	CP_CURRENT_LARGE
} tvp7002_cp_current;

/*structures*/

struct alc_filter {
	int alc_enable;
	tvp7002_alc_vertical_coeff vcoeff;
	tvp7002_alc_horizontal_coeff hcoeff;
};

struct tvp7002_format_params {
	unsigned char hpll_divider_msb;
	unsigned char hpll_divider_lsb;
	tvp7002_vco_gain hpll_vco_control;
	tvp7002_cp_current hpll_cp_current;
	unsigned char hpll_phase_select;
	tvp7002_post_divider hpll_post_divider;
	unsigned char hpll_control;
	unsigned char avid_start_msb;
	unsigned char avid_start_lsb;
	unsigned char avid_stop_msb;
	unsigned char avid_stop_lsb;
	unsigned char vblk_start_f0_line_offset;
	unsigned char vblk_start_f1_line_offset;
	unsigned char vblk_f0_duration;
	unsigned char vblk_f1_duration;
	unsigned char alc_placement;
	unsigned char clamp_start, clamp_width;
	unsigned char hpll_pre_coast;
	unsigned char hpll_post_coast;
	unsigned char reserved;
};

struct tvp7002_offset {
	unsigned char blue_fine_offset;
	unsigned char green_fine_offset;
	unsigned char red_fine_offset;
	unsigned char blue_fine_offset_lsb;
	unsigned char green_fine_offset_lsb;
	unsigned char red_fine_offset_lsb;
	unsigned char blue_coarse_offset;
	unsigned char green_coarse_offset;
	unsigned char red_coarse_offset;
};

struct tvp7002_gain {
	unsigned char blue_fine_gain;
	unsigned char green_fine_gain;
	unsigned char red_fine_gain;
	unsigned char blue_coarse_gain;
	unsigned char green_coarse_gain;
	unsigned char red_coarse_gain;
};

typedef struct {
	v4l2_std_id std;
	int inputidx;
	struct alc_filter alc;
	struct tvp7002_gain gain;
	struct tvp7002_offset offset;
	struct tvp7002_format_params format;
} tvp7002_params;

#ifdef __KERNEL__

#define TVP7002_NUM_CHANNELS                    1

/* Macros */
#define TVP7002_LINES_720       0x2EE
#define TVP7002_LINES_1080_60   0x465
#define TVP7002_LINES_1080_50   0x465
#define TVP7002_CLOCKS_PER_LINE_1080_60 188
#define TVP7002_CLOCKS_PER_LINE_1080_50 226

#define INTERLACED_VIDEO                         (0)
#define PROGRESSIVE_VIDEO                        (1)

#define GENERATE_MASK(bits, pos) ((((0xFFFFFFFF) << (32-bits)) >> \
		(32-bits)) << pos)

/* Defines for input supported */
#define TVP7002_HD (0)

/* Macros for default register values */
#define TVP7002_HPLL_MSB_DEFAULT                (0x67)
#define TVP7002_HPLL_LSB_DEFAULT                (0x20)
#define TVP7002_HPLL_CONTROL_DEFAULT            (0xA0)
#define TVP7002_HPLL_PHASE_SEL_DEFAULT          (0x80)
#define TVP7002_CLAMP_START_DEFAULT             (0x32)
#define TVP7002_CLAMP_WIDTH_DEFAULT             (0x20)
#define TVP7002_HSYNC_OUTWIDTH_DEFAULT          (0x60)
#define TVP7002_BLUE_FINE_GAIN_DEFAULT          (0x00)
#define TVP7002_GREEN_FINE_GAIN_DEFAULT         (0x00)
#define TVP7002_RED_FINE_GAIN_DEFAULT           (0x00)
#define TVP7002_BLUEF_OFFSETMSB_DEFAULT         (0x80)
#define TVP7002_GREENF_OFFSETMSB_DEFAULT        (0x80)
#define TVP7002_REDF_OFFSETMSB_DEFAULT          (0x80)
#define TVP7002_SYNC_CONTROL1_DEFAULT           (0x20)
#define TVP7002_HPLL_CLAMP_CTRL_DEFAULT         (0x2E)
#define TVP7002_SYNC_GREEN_THLD_DEFAULT         (0x5D)
#define TVP7002_SYNC_SEP_THLD_DEFAULT           (0x47)
#define TVP7002_HPLL_PRE_COAST_DEFAULT          (0x00)
#define TVP7002_HPLL_POST_COAST_DEFAULT         (0x00)
#define TVP7002_OUTPUT_FORMATTER_DEFAULT        (0x47)
#define TVP7002_MISC_CONTROL1_DEFAULT           (0x01)
#define TVP7002_MISC_CONTROL2_DEFAULT           (0x00)
#define TVP7002_MISC_CONTROL3_DEFAULT           (0x01)
#define TVP7002_INPUT_MUX_SELECT1_DEFAULT       (0x00)
#define TVP7002_INPUT_MUX_SELECT2_DEFAULT       (0x67)
#define TVP7002_BLUE_GREEN_COARSE_GAIN_DEFAULT  (0x77)
#define TVP7002_RED_COARSE_GAIN_DEFAULT         (0x07)
#define TVP7002_FINE_OFFSET_LSBS_DEFAULT        (0x00)
#define TVP7002_BLUE_COARSE_OFFSET_DEFUALT      (0x10)
#define TVP7002_GREEN_COARSE_OFFSET_DEFAULT     (0x10)
#define TVP7002_RED_COARSE_OFFSET_DEFAULT       (0x10)
#define TVP7002_RED_COARSE_OFFSET_DEFAULT       (0x10)
#define TVP7002_HSOUT_OUTPUT_START_DEFAULT      (0x08)
#define TVP7002_MISC_CONTROL4_DEFAULT           (0x00)
#define TVP7002_ALC_ENABLE_DEFAULT              (0x80)
#define TVP7002_ALC_FILTER_DEFAULT              (0x53)
#define TVP7002_FINE_CLAMP_CONTROL_DEFAULT      (0x07)
#define TVP7002_POWER_CONTROL_DEFAULT           (0x00)
#define TVP7002_ADC_SETUP_DEFAULT               (0x50)
#define TVP7002_COARSE_CLAMP_CONTROL_DEFAULT    (0x00)
#define TVP7002_SOG_CLAMP_DEFAULT               (0x80)
#define TVP7002_ALC_PLACEMENT_DEFAULT           (0x5A)
#define TVP7002_VSYNC_ALIGNMENT_DEFAULT         (0x10)
#define TVP7002_SYNC_BYPASS_DEFAULT             (0x00)
#define TVP7002_LINE_LENGTH_TOLERENCE_DEFAULT   (0x03)
#define TVP7002_ADC_REF_SETUP_DEFAULT           (0x04)
#define TVP7002_POWER_DOWN			(0x7F)
#define TVP7002_VIDEO_BANDWIDTH_CONTROL_DEFAULT	(0x01)
#define TVP7002_AVID_START_PIXEL_DEFAULT	(0x01)

/* Macros for horizontal PLL */
#define FEEDBACK_DIVIDER_MSB_720p               (0x67)
#define FEEDBACK_DIVIDER_LSB_720p               (0x02)
#define VCO_CONTROL_720p                        (0x02)
#define CP_CURRENT_720p                         (0x04)
#define PHASE_SELECT_720p                       (0x16)
#define POST_DIVIDER_720p                       (0x0)
#define HPLL_CONTROL_720p			(0xA0)
#define AVID_START_PIXEL_LSB_720p		(0x47)
#define AVID_START_PIXEL_MSB_720p		(0x01)
#define AVID_STOP_PIXEL_LSB_720p		(0x4B)
#define AVID_STOP_PIXEL_MSB_720p		(0x06)
#define VBLK_F0_START_LINE_OFFSET_720p		(0x05)
#define VBLK_F1_START_LINE_OFFSET_720p		(0x00)
#define VBLK_F0_DURATION_720p			(0x2D)
#define VBLK_F1_DURATION_720p			(0x00)
#define RESERVED_720p				(0x03)

#define FEEDBACK_DIVIDER_MSB_720p_50            (0x7B)
#define FEEDBACK_DIVIDER_LSB_720p_50            (0x0C)
#define VCO_CONTROL_720p_50                     (0x02)
#define CP_CURRENT_720p_50                      (0x03)
#define PHASE_SELECT_720p_50                    (0x16)
#define POST_DIVIDER_720p_50                    (0x0)
#define HPLL_CONTROL_720p_50			(0x98)
#define AVID_START_PIXEL_LSB_720p_50		(0x47)
#define AVID_START_PIXEL_MSB_720p_50		(0x01)
#define AVID_STOP_PIXEL_LSB_720p_50		(0x4B)
#define AVID_STOP_PIXEL_MSB_720p_50		(0x06)
#define VBLK_F0_START_LINE_OFFSET_720p_50	(0x05)
#define VBLK_F1_START_LINE_OFFSET_720p_50	(0x00)
#define VBLK_F0_DURATION_720p_50		(0x2D)
#define VBLK_F1_DURATION_720p_50		(0x00)
#define RESERVED_720p				(0x03)

#define FEEDBACK_DIVIDER_MSB_1080i              (0x89)
#define FEEDBACK_DIVIDER_LSB_1080i              (0x08)
#define VCO_CONTROL_1080i                       (0x02)
#define CP_CURRENT_1080i                        (0x03)
#define PHASE_SELECT_1080i                      (0x14)
#define POST_DIVIDER_1080i                      (0x0)
#define HPLL_CONTROL_1080i			(0x98)
#define AVID_START_PIXEL_LSB_1080i		(0x06)
#define AVID_START_PIXEL_MSB_1080i		(0x01)
#define AVID_STOP_PIXEL_LSB_1080i		(0x8A)
#define AVID_STOP_PIXEL_MSB_1080i		(0x08)
#define VBLK_F0_START_LINE_OFFSET_1080i		(0x02)
#define VBLK_F1_START_LINE_OFFSET_1080i		(0x02)
#define VBLK_F0_DURATION_1080i			(0x16)
#define VBLK_F1_DURATION_1080i			(0x17)
#define RESERVED_1080i				(0x02)

#define FEEDBACK_DIVIDER_MSB_1080i_50           (0xA5)
#define FEEDBACK_DIVIDER_LSB_1080i_50           (0x00)
#define VCO_CONTROL_1080i_50                    (0x02)
#define CP_CURRENT_1080i_50                     (0x02)
#define PHASE_SELECT_1080i_50                   (0x14)
#define POST_DIVIDER_1080i_50                   (0x0)
#define HPLL_CONTROL_1080i_50			(0x90)
#define AVID_START_PIXEL_LSB_1080i_50		(0x06)
#define AVID_START_PIXEL_MSB_1080i_50		(0x01)
#define AVID_STOP_PIXEL_LSB_1080i_50		(0x8A)
#define AVID_STOP_PIXEL_MSB_1080i_50		(0x08)
#define VBLK_F0_START_LINE_OFFSET_1080i_50	(0x02)
#define VBLK_F1_START_LINE_OFFSET_1080i_50	(0x02)
#define VBLK_F0_DURATION_1080i_50		(0x16)
#define VBLK_F1_DURATION_1080i_50		(0x17)
#define RESERVED_1080i_50			(0x02)

#define FEEDBACK_DIVIDER_MSB_480P		(0x35)
#define FEEDBACK_DIVIDER_LSB_480P		(0x0A)
#define VCO_CONTROL_480P			(0x02)
#define CP_CURRENT_480P				(0x02)
#define PHASE_SELECT_480P			(0x14)
#define POST_DIVIDER_480P			(0x0)
#define HPLL_CONTROL_480P			(0x18)
#define AVID_START_PIXEL_LSB_480P		(0x91)
#define AVID_START_PIXEL_MSB_480P		(0x00)
#define AVID_STOP_PIXEL_LSB_480P		(0x0B)
#define AVID_STOP_PIXEL_MSB_480P		(0x00)
#define VBLK_F0_START_LINE_OFFSET_480P		(0x03)
#define VBLK_F1_START_LINE_OFFSET_480P		(0x01)
#define VBLK_F0_DURATION_480P			(0x13)
#define VBLK_F1_DURATION_480P			(0x13)
#define RESERVED_1080i_50                       (0x02)

#define FEEDBACK_DIVIDER_MSB_576P               (0x36)
#define FEEDBACK_DIVIDER_LSB_576P               (0x00)
#define VCO_CONTROL_576P                        (0x02)
#define CP_CURRENT_576P                         (0x02)
#define PHASE_SELECT_576P                       (0x14)
#define POST_DIVIDER_576P                       (0x0)
#define HPLL_CONTROL_576P                       (0x18)
#define AVID_START_PIXEL_LSB_576P               (0x9B)
#define AVID_START_PIXEL_MSB_576P               (0x00)
#define AVID_STOP_PIXEL_LSB_576P                (0x0F)
#define AVID_STOP_PIXEL_MSB_576P                (0x00)
#define VBLK_F0_START_LINE_OFFSET_576P          (0x00)
#define VBLK_F1_START_LINE_OFFSET_576P          (0x00)
#define VBLK_F0_DURATION_576P                   (0x2D)
#define VBLK_F1_DURATION_576P                   (0x00)
#define RESERVED_1080i_50                       (0x02)

#define TVP7002_HD_ALC_PLACEMENT		(0x5A)
#define TVP7002_ED_ALC_PLACEMENT		(0x18)

#define TVP7002_HD_CLAMP_START			(0x32)
#define TVP7002_ED_CLAMP_START			(0x06)

#define TVP7002_HD_CLAMP_WIDTH			(0x20)
#define TVP7002_ED_CLAMP_WIDTH			(0x10)

#define TVP7002_HD_PRE_COAST			(0x0)
#define TVP7002_ED_PRE_COAST			(0x03)

#define TVP7002_HD_POST_COAST			(0x0)
#define TVP7002_ED_POST_COAST			(0x0C)

/* HPLL masks and shifts */
#define HPLL_DIVIDER_LSB_MASK                   GENERATE_MASK(4, 0)
#define HPLL_DIVIDER_LSB_SHIFT                  4
#define VCO_CONTROL_MASK                        GENERATE_MASK(2, 0)
#define CP_CURRENT_MASK                         GENERATE_MASK(3, 0)
#define VCO_CONTROL_SHIFT                       6
#define CP_CURRENT_SHIFT                        3
#define PHASE_SELECT_MASK                       GENERATE_MASK(5, 0)
#define PHASE_SELECT_SHIFT                      3

#define POST_DIVIDER_MASK                       GENERATE_MASK(1, 0)

#define LINES_PER_FRAME_MSB_MASK                GENERATE_MASK(4, 8)
#define LINES_PER_FRAME_MSB_SHIFT               8

#define VIDEO_DETECTION_MASK                    GENERATE_MASK(1, 5)
#define VIDEO_DETECTION_SHIFT			5

/* Gain and offset masks */

#define BLUE_COARSE_GAIN_MASK                   GENERATE_MASK(4, 0)
#define GREEN_COARSE_GAIN_MASK                  GENERATE_MASK(4, 0)
#define RED_COARSE_GAIN_MASK                    GENERATE_MASK(4, 0)
#define GREEN_COARSE_GAIN_SHIFT                 4

#define FINE_OFFSET_LSB_MASK                    0x03

#define FINE_OFFSET_LSB_SHIFT_GREEN             2
#define FINE_OFFSET_LSB_SHIFT_RED               4

#define COARSE_OFFSET_MASK                      GENERATE_MASK(6, 0)

/* Defines for TVP7002 register address */
#define TVP7002_HPLL_DIVIDER_MSB                (0x01)
#define TVP7002_HPLL_DIVIDER_LSB                (0x02)
#define TVP7002_HPLL_CONTROL                    (0x03)
#define TVP7002_HPLL_PHASE_SELECT               (0x04)
#define TVP7002_CLAMP_START                     (0x05)
#define TVP7002_CLAMP_WIDTH                     (0x06)
#define TVP7002_HSYNC_OUTPUT_WIDTH              (0x07)

#define TVP7002_BLUE_FINE_GAIN                  (0x08)
#define TVP7002_GREEN_FINE_GAIN                 (0x09)
#define TVP7002_RED_FINE_GAIN                   (0x0A)
#define TVP7002_BLUE_FINE_OFFSETMSB             (0x0B)
#define TVP7002_GREEN_FINE_OFFSETMSB            (0x0C)
#define TVP7002_RED_FINE_OFFSETMSB              (0x0D)

#define TVP7002_SYNC_CONTROL_1                  (0x0E)
#define TVP7002_HPLL_CLAMP_CONTROL              (0x0F)
#define TVP7002_SYNC_ON_GREEN_THLD              (0x10)
#define TVP7002_SYNC_SEPARATER_THLD             (0x11)
#define TVP7002_HPLL_PRE_COAST                  (0x12)
#define TVP7002_HPLL_POST_COAST                 (0x13)

#define TVP7002_OUTPUT_FORMATTER                (0x15)
#define TVP7002_MISC_CONTROL_1                  (0x16)
#define TVP7002_MISC_CONTROL_2                  (0x17)
#define TVP7002_MISC_CONTROL_3                  (0x18)
#define TVP7002_INPUT_MUX_SELECT_1              (0x19)
#define TVP7002_INPUT_MUX_SELECT_2              (0x1A)
#define TVP7002_BLUE_GREEN_COARSE_GAIN          (0x1B)
#define TVP7002_RED_COARSE_GAIN                 (0x1C)
#define TVP7002_FINE_OFFSET_LSBS                (0x1D)
#define TVP7002_BLUE_COARSE_OFFSET              (0x1E)
#define TVP7002_GREEN_COARSE_OFFSET             (0x1F)
#define TVP7002_RED_COARSE_OFFSET               (0x20)
#define TVP7002_HSOUT_OUTPUT_START              (0x21)
#define TVP7002_MISC_CONTROL_4                  (0x22)
#define TVP7002_ALC_ENABLE                      (0x26)
#define TVP7002_ALC_FILTER                      (0x28)
#define TVP7002_FINE_CLAMP_CONTROL              (0x2A)
#define TVP7002_POWER_CONTROL                   (0x2B)
#define TVP7002_ADC_SETUP                       (0x2C)
#define TVP7002_COARSE_CLAMP_CONTROL            (0x2D)
#define TVP7002_SOG_CLAMP                       (0x2E)
#define TVP7002_ALC_PLACEMENT                   (0x31)
#define TVP7002_VSYNC_ALIGNMENT                 (0x35)
#define TVP7002_SYNC_BYPASS                     (0x36)
#define TVP7002_LINES_PER_FRAME_STATUS_LOW      (0x37)
#define TVP7002_LINES_PER_FRAME_STATUS_HIGH     (0x38)
#define TVP7002_CLOCK_PER_LINE_STATUS_LSB	(0x39)
#define TVP7002_CLOCK_PER_LINE_STATUS_MSB	(0x3A)

#define TVP7002_LINE_LENGTH_TOLERENCE           (0x3D)
#define TVP7002_ADC_REF_SETUP                   (0x3E)
#define TVP7002_VIDEO_BANDWIDTH_CONTROL		(0x3F)
#define TVP7002_AVID_START_PIXEL_LOW            (0x40)
#define TVP7002_AVID_START_PIXEL_HIGH           (0x41)
#define TVP7002_AVID_STOP_PIXEL_LOW             (0x42)
#define TVP7002_AVID_STOP_PIXEL_HIGH            (0x43)
#define TVP7002_VBLK_FIELD0_START_OFFSET        (0x44)
#define TVP7002_VBLK_FIELD1_START_OFFSET        (0x45)
#define TVP7002_VBLK_FIELD0_DURATION            (0x46)
#define TVP7002_VBLK_FIELD1_DURATION            (0x47)
#define TVP7002_FBIT_FIELD0_START_OFFSET        (0x48)
#define TVP7002_FBIT_FIELD1_START_OFFSET        (0x49)

#define TVP7002_HD_INPUT                        (0x00)

/* decoder standard related strctures */
#define TVP7002_MAX_NO_INPUTS           (1)
#define TVP7002_MAX_NO_STANDARDS        (4+1+1)
#define TVP7002_MAX_NO_CONTROLS         (0)

#define TVP7002_ALC_VCOEFF_SHIFT	(4)

#define TVP7002_STANDARD_INFO_SIZE      (TVP7002_MAX_NO_STANDARDS)

struct tvp7002_control_info {
	int register_address;
	struct v4l2_queryctrl query_control;
};

struct tvp7002_config {
	int no_of_inputs;
	struct {
		int input_type;
		struct v4l2_input input_info;
		int no_of_standard;
		struct v4l2_standard *standard;
		v4l2_std_id def_std;
		struct tvp7002_format_params *format;
		int no_of_controls;
		struct tvp7002_control_info *controls;
	} input[TVP7002_MAX_NO_INPUTS];
	tvp7002_params def_params;
};

struct tvp7002_channel {
	struct {
		struct i2c_client client;
		struct i2c_driver driver;
		u32 i2c_addr;
		int i2c_registration;
	} i2c_dev;
	struct decoder_device *dec_device;
	tvp7002_params params;
};

#endif				/* __KERNEL__ */

#endif
