/*
 * Copyright (C) 2006 Texas Instruments Inc
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*mt9t001.h*/

#ifndef	MT9T001_H
#define	MT9T001_H

/*********************************
 * Defines and Macros and globals
 ********************************/

#ifdef	TRUE
#undef	TRUE
#endif

#ifdef	FALSE
#undef	FALSE
#endif

#define	TRUE 	1
#define	FALSE	0

#ifdef DEBUG
#undef DEBUG
#endif

#ifndef TYPES
#define TYPES
typedef unsigned char bool;
#endif

#define DEBUG

#ifdef __KERNEL__

extern int mt9t001_ctrl(unsigned int cmd, void *arg, void *params);

/* defines for extra pixels/line added */
#define MT9T001_EXTRAPIXELS 16
#define MT9T001_EXTRALINES	  8

/* Definitions to access the various sensor registers */
#define MT9T001_CHIP_VERSION			(0x00)
#define MT9T001_ROW_START			(0x01)
#define MT9T001_COL_START			(0x02)
#define MT9T001_HEIGHT				(0x03)
#define MT9T001_WIDTH				(0x04)
#define MT9T001_HBLANK				(0x05)
#define MT9T001_VBLANK				(0x06)
#define MT9T001_OUTPUT_CTRL			(0x07)
#define MT9T001_SHUTTER_WIDTH_UPPER		(0x08)
#define MT9T001_SHUTTER_WIDTH			(0x09)
#define MT9T001_PIXEL_CLK_CTRL			(0x0A)
#define MT9T001_RESTART				(0x0B)
#define MT9T001_SHUTTER_DELAY			(0x0C)
#define MT9T001_RESET				(0x0D)
#define MT9T001_READ_MODE1			(0x1E)
#define MT9T001_READ_MODE2			(0x20)
#define MT9T001_READ_MODE3			(0x21)
#define MT9T001_ROW_ADDR_MODE			(0x22)
#define MT9T001_COL_ADDR_MODE			(0x23)
#define MT9T001_RESERVED_27_REG                 (0x27)
#define MT9T001_GREEN1_GAIN			(0x2B)
#define MT9T001_BLUE_GAIN			(0x2C)
#define MT9T001_RED_GAIN			(0x2D)
#define MT9T001_GREEN2_GAIN			(0x2E)
#define MT9T001_GLOBAL_GAIN			(0x35)
#define MT9T001_BLACK_LEVEL			(0x49)
#define MT9T001_ROW_BLK_DEF_OFFSET		(0x4B)
#define MT9T001_RESERVED_4E_REG                 (0x4e)
#define MT9T001_RESERVED_50_REG                 (0x50)
#define MT9T001_RESERVED_51_REG                 (0x51)
#define MT9T001_RESERVED_52_REG                 (0x52)
#define MT9T001_RESERVED_53_REG                 (0x53)
#define MT9T001_CAL_COARSE			(0x5D)
#define MT9T001_CAL_TARGET			(0x5F)
#define MT9T001_GREEN1_OFFSET			(0x60)
#define MT9T001_GREEN2_OFFSET			(0x61)
#define MT9T001_BLK_LVL_CALIB			(0x62)
#define MT9T001_RED_OFFSET			(0x63)
#define MT9T001_BLUE_OFFSET			(0x64)
#define MT9T001_CHIP_ENABLE_SYNC		(0xF8)
#define MT9T001_CHIP_VERSION_END		(0xFF)

/* Define Shift and Mask for gain register*/

#define	MT9T001_ANALOG_GAIN_SHIFT	(0x0000)
#define	MT9T001_DIGITAL_GAIN_SHIFT	(8)
#define	MT9T001_ANALOG_GAIN_MASK	(0x007F)
#define	MT9T001_DIGITAL_GAIN_MASK	(0x7F00)

/* Define Shift and Mask for black level caliberation register*/

#define	MT9T001_MANUAL_OVERRIDE_MASK		(0x0001)
#define	MT9T001_DISABLE_CALLIBERATION_SHIFT	(1)
#define	MT9T001_DISABLE_CALLIBERATION_MASK	(0x0002)
#define	MT9T001_RECAL_BLACK_LEVEL_SHIFT		(12)
#define	MT9T001_RECAL_BLACK_LEVEL_MASK		(0x1000)
#define	MT9T001_LOCK_RB_CALIBRATION_SHIFT	(13)
#define	MT9T001_LOCK_RB_CALLIBERATION_MASK	(0x2000)
#define	MT9T001_LOCK_GREEN_CALIBRATION_SHIFT	(14)
#define	MT9T001_LOCK_GREEN_CALLIBERATION_MASK	(0x4000)
#define	MT9T001_LOW_COARSE_THELD_MASK		(0x007F)
#define	MT9T001_HIGH_COARSE_THELD_SHIFT		(8)
#define	MT9T001_HIGH_COARSE_THELD_MASK		(0x7F00)
#define	MT9T001_LOW_TARGET_THELD_MASK		(0x007F)
#define	MT9T001_HIGH_TARGET_THELD_SHIFT		(8)
#define	MT9T001_HIGH_TARGET_THELD_MASK		(0x7F00)
#define	MT9T001_SHUTTER_WIDTH_LOWER_MASK	(0xFFFF)
#define	MT9T001_SHUTTER_WIDTH_UPPER_SHIFT	(16)
#define	MT9T001_SHUTTER_WIDTH_UPPER_MASK	(0xFFFF)
#define	MT9T001_ROW_START_MASK			(0x07FF)
#define	MT9T001_COL_START_MASK			(0x0FFF)
#define	 MT9T001_GREEN1_OFFSET_MASK 		(0x01FF)
#define	 MT9T001_GREEN2_OFFSET_MASK 		(0x01FF)
#define	 MT9T001_RED_OFFSET_MASK 		(0x01FF)
#define	 MT9T001_BLUE_OFFSET_MASK 		(0x01FF)

/* defines for MT9T001 register values */
#define	MT9T001_NORMAL_OPERATION_MODE		(0x0002)
#define	MT9T001_HALT_MODE			(0x0003)
#define MT9T001_RESET_ENABLE			(0x0001)
#define MT9T001_RESET_DISABLE			(0x0000)
#define	MT9T001_INVERT_PIXEL_CLK		(0x8000)
#define MT9T001_GAIN_MINVAL			(0)
#define MT9T001_GAIN_MAXVAL			(128)
#define MT9T001_GAIN_STEP			(1)
#define MT9T001_GAIN_DEFAULTVAL			(8)

/* Default values for MT9T001 registers */
#define MT9T001_ROW_START_DEFAULT		(0x14)
#define MT9T001_COL_START_DEFAULT		(0x20)
#define MT9T001_HEIGHT_DEFAULT			(0x5FF)
#define MT9T001_WIDTH_DEFAULT			(0x7FF)
#define MT9T001_HBLANK_DEFAULT			(0x8E)
#define MT9T001_VBLANK_DEFAULT			(0x19)
#define MT9T001_OUTPUT_CTRL_DEFAULT		(0x02)
#define MT9T001_SHUTTER_WIDTH_UPPER_DEFAULT	(0x0)
#define MT9T001_SHUTTER_WIDTH_DEFAULT		(0x619)
#define MT9T001_PIXEL_CLK_CTRL_DEFAULT		(0x0)
#define MT9T001_RESTART_DEFAULT			(0x0)
#define MT9T001_SHUTTER_DELAY_DEFAULT		(0x0)
#define MT9T001_READ_MODE1_DEFAULT		(0xC040)
#define MT9T001_READ_MODE2_DEFAULT		(0x0)
#define MT9T001_READ_MODE3_DEFAULT		(0x0)
#define MT9T001_ROW_ADDR_MODE_DEFAULT		(0x0)
#define MT9T001_COL_ADDR_MODE_DEFAULT		(0x0)
#define MT9T001_GREEN1_GAIN_DEFAULT		(0x08)
#define MT9T001_BLUE_GAIN_DEFAULT		(0x08)
#define MT9T001_RED_GAIN_DEFAULT		(0x08)
#define MT9T001_GREEN2_GAIN_DEFAULT		(0x08)
#define MT9T001_GLOBAL_GAIN_DEFAULT		(0x08)
#define MT9T001_BLACK_LEVEL_DEFAULT		(0xA8)
#define MT9T001_CAL_COARSE_DEFAULT		(0x2D13)
#define MT9T001_CAL_TARGET_DEFAULT		(0x231D)
#define MT9T001_GREEN1_OFFSET_DEFAULT		(0x20)
#define MT9T001_GREEN2_OFFSET_DEFAULT		(0x20)
#define MT9T001_BLK_LVL_CALIB_DEFAULT		(0x0)
#define MT9T001_RED_OFFSET_DEFAULT		(0x20)
#define MT9T001_BLUE_OFFSET_DEFAULT		(0x20)
#define MT9T001_CHIP_ENABLE_SYNC_DEFAULT	(0x01)

#define MT9T001_I2C_REGISTERED			(1)
#define MT9T001_I2C_UNREGISTERED		(0)

/*	Defines for mt9t001_ctrl() functions command*/

#define	MT9T001_SET_PARAMS	1
#define	MT9T001_GET_PARAMS	2
#define	MT9T001_SET_GAIN	3
#define	MT9T001_SET_STD		4
#define	MT9T001_INIT		5
#define	MT9T001_CLEANUP		6
#define MT9T001_ENABLE_I2C_SWITCH 7

/* define for various video format supported by MT9T001 driver */
/* Here all mode defines will be assigned values of v4l2 mode defines */
#define	MT9T001_MODE_VGA_30FPS  	(10)
#define	MT9T001_MODE_VGA_60FPS		(11)
#define	MT9T001_MODE_SVGA_30FPS		(12)
#define	MT9T001_MODE_SVGA_60FPS		(13)
#define	MT9T001_MODE_XGA_30FPS		(14)
#define	MT9T001_MODE_480p_30FPS		(15)
#define	MT9T001_MODE_480p_60FPS		(16)
#define	MT9T001_MODE_576p_25FPS		(17)
#define	MT9T001_MODE_576p_50FPS		(18)
#define	MT9T001_MODE_720p_24FPS		(19)
#define	MT9T001_MODE_720p_30FPS		(20)
#define	MT9T001_MODE_1080p_18FPS	(21)

/*i2c adress for MT9T001*/
#define MT9T001_I2C_ADDR  		(0xBA >>1)

/*i2c adress for ECP*/
#define ECP_I2C_ADDR  			(0x25)
#define ECP_I2C_CONFIG			(2)
#define ECP_REGADDR			(0x08)
#define ECP_REGVAL			(0x80)
#define ECP_RSTADDR			(0x02)

/*i2c adress for PCA9543A*/
#define PCA9543A_I2C_ADDR  		(0x73)
#define PCA9543A_I2C_CONFIG		(0)
#define PCA9543A_REGVAL			(0x01)

#define MT9T001_I2C_CONFIG		(1)
#define I2C_ONE_BYTE_TRANSFER		(1)
#define I2C_TWO_BYTE_TRANSFER		(2)
#define I2C_THREE_BYTE_TRANSFER		(3)
#define I2C_TXRX_DATA_MASK		(0x00FF)
#define I2C_TXRX_DATA_MASK_UPPER	(0xFF00)
#define I2C_TXRX_DATA_SHIFT		(8)

#endif				/* endif of __KERNEL__ */

/* Structure containing video standard dependent settings */
struct mt9t001_format_params {
	unsigned short col_size;	/* width */
	unsigned short row_size;	/* Height */
	unsigned short h_blank;
	unsigned short v_blank;
	unsigned int shutter_width;
	unsigned short row_addr_mode;
	unsigned short col_addr_mode;
	unsigned short black_level;
	unsigned short pixel_clk_control;
	unsigned short row_start;
	unsigned short col_start;
};

/* Structure for gain settings */
struct mt9t001_rgb_gain {
	unsigned char green1_analog_gain;
	unsigned char red_analog_gain;
	unsigned char blue_analog_gain;
	unsigned char green2_analog_gain;
	unsigned char green1_digital_gain;
	unsigned char red_digital_gain;
	unsigned char blue_digital_gain;
	unsigned char green2_digital_gain;
};
/* structure for black level calibration setttings*/
struct mt9t001_black_level_calibration {
	bool manual_override;
	bool disable_calibration;
	bool recalculate_black_level;
	bool lock_red_blue_calibration;
	bool lock_green_calibration;
	unsigned char low_coarse_thrld;
	unsigned char high_coarse_thrld;
	unsigned char low_target_thrld;
	unsigned char high_target_thrld;
	unsigned short green1_offset;
	unsigned short green2_offset;
	unsigned short red_offset;
	unsigned short blue_offset;
};

/* structure for MT9T001 configuration setttings passed by application*/
struct mt9t001_params {
	struct mt9t001_format_params format;
	struct mt9t001_rgb_gain rgb_gain;
	struct mt9t001_black_level_calibration black_calib;
};

#endif				/*for  ifndef MT9T001 */
