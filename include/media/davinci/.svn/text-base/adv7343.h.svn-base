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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*adv7343.h*/

#ifndef ADV7343_H
#define ADV7343_H

#ifdef __KERNEL__

/* Kernel Header files */
#include <linux/i2c.h>
#include <linux/device.h>
#include <media/davinci/vid_encoder_types.h>

#endif				/* __KERNEL__ */

/* Macros */
#define ADV7343_MAX_GAMMA_COEFFS	10	/* Maximum Gamma Coefficients */

/* Enums */
/* Enum for type of parameters */
enum adv7343_params_type {
	ADV7343_SDPARAMS = 0,	/* Indicates SD parameters */
	ADV7343_HDPARAMS	/* Indicates HD parameters */
};

/* Enum for DNR modes */
enum adv7343_dnr_mode {
	ADV7343_MODE_DNR = 0,	/* Indicates DNR Mode */
	ADV7343_MODE_DNR_SHARPNESS	/* Indicates DNR Sharpness Mode */
};

/* Enum for DNR block size */
enum adv7343_dnr_blocksize {
	ADV7343_DNR_BLOCK_8 = 0,	/* Indicates DNR block of 8x8 */
	ADV7343_DNR_BLOCK_16	/* Indicates DNR block of 16x16 */
};

/* Enum for SD Luma filters */
enum adv7343_sd_luma_filter {
	ADV7343_LUMA_LPF_NTSC = 0,	/* HD Luma LPF Filter for NTSC */
	ADV7343_LUMA_LPF_PAL,	/* HD Luma LPF Filter for PAL */
	ADV7343_LUMA_NOTCH_NTSC,	/* HD Luma Notch Filter for NTSC */
	ADV7343_LUMA_NOTCH_PAL,	/* HD Luma Notch Filter for NTSC */
	ADV7343_LUMA_SSAF,	/* HD Luma SSAF Filter */
	ADV7343_LUMA_CIF,	/* HD Luma CIF Filter */
	ADV7343_LUMA_QCIF	/* HD Luma QCIF Filter */
};

/* Enum for SD Chroma filters */
enum adv7343_sd_chroma_filter {
	ADV7343_CHROMA_0_65 = 0,	/* SD Chroma 0.65MHz LPF */
	ADV7343_CHROMA_1_0,	/* SD Chroma 1.0MHz LPF */
	ADV7343_CHROMA_1_3,	/* SD Chroma 1.3MHz LPF */
	ADV7343_CHROMA_2_0,	/* SD Chroma 2.0MHz LPF */
	ADV7343_CHROMA_2_3,	/* SD Chroma 2.3MHz LPF */
	ADV7343_CHROMA_CIF,	/* SD Chroma CIF Response LPF */
	ADV7343_CHROMA_QCIF,	/* SD Chroma QCIF Response LPF */
	ADV7343_CHROMA_SSAF,	/* SD SSAF Color difference filter */
};

/* Enum for HD Filters */
enum adv7343_hd_filter {
	ADV7343_SINC_FILTER = 1,	/* HD Sync filter on DAC D,E,F */
	ADV7343_HD_CHROMA_SSAF = 2	/* HD Chroma SSAF filter */
};

/* Enum for selecting DNR filter */
enum adv7343_dnr_filter {
	ADV7343_DNR_FILTER_A = 0,	/* DNR Filter A */
	ADV7343_DNR_FILTER_B,	/* DNR Filter B */
	ADV7343_DNR_FILTER_C,	/* DNR Filter C */
	ADV7343_DNR_FILTER_D	/* DNR Filter D */
};

/* Enum for selecting adaptive filter modes */
enum adv7343_filter_mode {
	ADV7343_FILTER_MODE_SHARPNESS = 0,	/* Adaptive Filter Sharpness
						   mode */
	ADV7343_FILTER_MODE_A,	/* Adaptive Filter Mode A */
	ADV7343_FILTER_MODE_B	/* Adaptive Filter Mode B */
};

/* Enum for Border AREA DNR */
enum adv7343_dnr_border_area {
	ADV7343_BORDER_AREA_2 = 0,	/* Border area size is 2 pixels */
	ADV7343_BORDER_AREA_4	/* Border area size is 4 pixels */
};

/* Enum for feature control */
enum adv7343_feature {
	ADV7343_DISABLE = 0,
	ADV7343_ENABLE
};
/* structure */
/* RGB Matrix for HD */
struct adv7343_rgb_matrix {
	enum adv7343_feature en_rgb_matrix;	/* Enables/Disabled 
						   RGB matrix */
	unsigned short gy;	/* RGB Coeffs */
	unsigned short gu;
	unsigned short gv;
	unsigned short bu;
	unsigned short rv;
};

/* Filter parameters for HD */
struct adv7343_filter_params {
	enum adv7343_feature en_filter;	/* Used to enable/disable filters */
	enum adv7343_filter_mode mode;	/* mode of the sharpness and adaptive 
					   filter */
	unsigned char threshold_a;	/* Adaptive filter threshold A */
	unsigned char threshold_b;	/* Adaptive filter threshold B */
	unsigned char threshold_c;	/* Adaptive filter threshold C */
	unsigned char gain1_vala;	/* Adaptive filter Gain1 Value A */
	unsigned char gain1_valb;	/* Adaptive filter Gain1 Value B */
	unsigned char gain2_vala;	/* Adaptive filter Gain2 Value A */
	unsigned char gain2_valb;	/* Adaptive filter Gain2 Value B */
	unsigned char gain3_vala;	/* Adaptive filter Gain3 Value A */
	unsigned char gain3_valb;	/* Adaptive filter Gain3 Value B */
	unsigned char sharpness;	/* Sharpness value */
};

/* DNR parameters structure */
struct adv7343_dnr_params {
	enum adv7343_feature en_dnr;	/* enables/disables DNR */
	enum adv7343_dnr_mode mode;	/* Selects DNR Mode */
	enum adv7343_dnr_blocksize block_size;	/* block size for DNR */
	enum adv7343_dnr_border_area area;	/* Block size */
	unsigned char border_gain;	/* Gain for the border of 
					   block */
	unsigned char data_gain;	/* Gain for data except 
					   border of block */
	enum adv7343_dnr_filter filt_select;	/* Selects on filter */
	unsigned char block_offset;	/* Block offset */
	unsigned char threshold;	/* DNR threshold */
};

/* SSAF Parameters structure */
struct adv7343_ssaf_params {
	enum adv7343_feature en_gain;	/* Enables/disables gain */
	unsigned char gain;	/* Value of Gain */
};

/* SD Y and PbPr scaling parameters */
struct adv7343_scale_params {
	enum adv7343_feature en_scale;	/* This enabled/disabled scaling */
	unsigned short y_scale;	/* Value of scaling for Y data */
	unsigned short pb_scale;	/* Value of scaling for Pb data */
	unsigned short pr_scale;	/* Value of scaling for Pr data */
};

/* Used to select gamma curve */
enum adv7343_gamma_curve {
	ADV7343_GAMMA_CURVE_A = 0,
	ADV7343_GAMMA_CURVE_B
};

struct adv7343_gamma_params {
	enum adv7343_feature en_gamma;	/* enables/disables 
					   gamma correction */
	enum adv7343_gamma_curve curve;	/* Selects gamma 
					   correction curve */
	unsigned char coeff[ADV7343_MAX_GAMMA_COEFFS];	/* Gamma correction 
							   coefficients */
};

/* SD Parameters structure */
struct adv7343_sd_params {
	enum adv7343_sd_luma_filter luma_filter;	/* SD Luma Filter */
	enum adv7343_sd_chroma_filter chroma_filter;	/* SD Chroma Filter */
	struct adv7343_ssaf_params ssaf;	/* SSAF Parameters */
	struct adv7343_scale_params scale;	/* scaling parameters */
	struct adv7343_dnr_params dnr;	/* DNR Parameters */
	enum adv7343_feature active_step_edge_scale;	/* Enabled/disables 
							   active edge and 
							   step edge scale */
	struct adv7343_gamma_params gamma;	/* gamma parameters */
};

/* HD Parameter structure */
struct adv7343_hd_params {
	enum adv7343_feature sinc_filter;	/* HD Sinc Filter */
	enum adv7343_feature ssaf_filter;	/* HD ssaf Filter */
	struct adv7343_rgb_matrix rgb;	/* RGB Matrix coefficients */
	struct adv7343_filter_params filt_params;	/* Filter parameters */
	struct adv7343_gamma_params gamma;	/* Gamma parameters */
};

/* ADV7343 parameters structure */
typedef struct {
	enum adv7343_params_type type;	/* Type of parameters SD or 
					   HD */
	union {
		struct adv7343_sd_params sd;	/* SD Parameters */
		struct adv7343_hd_params hd;	/* HD Parameters */
	} params;
} adv7343_params;

#ifdef __KERNEL__

#define ADV7343_COMPOSITE_OUTPUT_NAME  	"COMPOSITE"
#define ADV7343_COMPONENT_OUTPUT_NAME  	"COMPONENT"
#define ADV7343_SVIDEO_OUTPUT_NAME	"SVIDEO"

#define ADV7343_COMPOSITE_ID		0
#define ADV7343_COMPONENT_ID		1
#define ADV7343_SVIDEO_ID		2

#define ADV7343_COMPOSITE_NO_CONTROLS		(3)
#define ADV7343_COMPONENT_NO_CONTROLS		(1)
#define ADV7343_SVIDEO_NO_CONTROLS		(3)

#define ADV7343_NUM_CHANNELS			(1)

/* encoder standard related strctures */
#define ADV7343_MAX_NO_OUTPUTS			(3)
#define ADV7343_COMPOSITE_NUM_STD		(2)
#define ADV7343_COMPONENT_NUM_STD		(7+3+3)
#define ADV7343_SVIDEO_NUM_STD			(2)
#define ADV7343_MAX_NO_CONTROLS			(3)
#define ADV7343_VBI_NUM_SERVICES		(3)

struct adv7343_std_info {
	unsigned char set_std_register;
	volatile unsigned char *value;
	u32 outputmode_val1;
	u32 standard_val2;
	u32 standard_val3;
	u8 fsc0_reg, fsc0_val;
	u8 fsc1_reg, fsc1_val;
	u8 fsc2_reg, fsc2_val;
	u8 fsc3_reg, fsc3_val;
};

struct adv7343_control_info {
	int register_address;
	int value;
	enum vid_enc_ctrl_id id;
	int minimum, maximum;
};

struct adv7343_config {
	int no_of_outputs;
	struct {
		unsigned char power_val;
		int output_type;
		char output_name[VID_ENC_NAME_MAX_CHARS];
		int no_of_standard;
		struct vid_enc_mode_info *standard;
		struct vid_enc_mode_info *def_std;
		struct adv7343_std_info *std_info;
		int no_of_controls;
		struct adv7343_control_info *controls;
	} output[ADV7343_MAX_NO_OUTPUTS];
	unsigned short services_set;
	u8 num_services;
};

struct adv7343_channel {
	struct {
		struct i2c_client client;
		struct i2c_driver driver;
		u8 i2c_addr;
		int i2c_registration;
	} i2c_dev;
	struct vid_encoder_device *enc_device;
	u8 current_output;
	struct vid_enc_mode_info *mode_info;
	unsigned short services_set;
	adv7343_params params;
};

struct adv7343_service_data_reg {
	u32 service_set;
	struct {
		u8 addr[2];
	} field[2];
	u8 bytestowrite;
};

struct adv7343_service_reg {
	u32 service;
	u8 reg;
	u8 *reg_val;
	u8 enable_val;
	u8 disable_val;
	struct vid_enc_mode_info *std;
};

#define ADV7343_VALID_FEATURE_VAL(val) ((ADV7343_DISABLE == (val))|| \
					(ADV7343_ENABLE == (val)))
#define ADV7343_VALID_GAMMA_CURVE(val) ((ADV7343_GAMMA_CURVE_A == (val))|| \
					(ADV7343_GAMMA_CURVE_B == (val)))

/* Register Macros */
#define ADV7343_POWER_MODE_REG			0x00
#define ADV7343_MODE_SELECT_REG			0X01
#define ADV7343_MODE_REG0			0x02
#define ADV7343_CSC_MATRIX0			0x03
#define ADV7343_CSC_MATRIX1			0x04
#define ADV7343_CSC_MATRIX2			0x05
#define ADV7343_CSC_MATRIX3			0x06
#define ADV7343_CSC_MATRIX4			0x07
#define ADV7343_CSC_MATRIX5			0x08
#define ADV7343_CSC_MATRIX6			0x09
#define ADV7343_DAC1_OUTPUT_LEVEL		0x0a
#define ADV7343_DAC2_OUTPUT_LEVEL		0x0b
#define ADV7343_DAC_POWER_MODE			0x0d
#define ADV7343_CABLE_DETECTION			0x10
#define ADV7343_SBUS_READ			0x12
#define ADV7343_YBUS_READ			0x13
#define ADV7343_CBUS_READ			0x14
#define ADV7343_CONTROL_READ			0x16
#define ADV7343_SOFT_RESET			0x17
#define ADV7343_HD_MODE_REG1			0x30
#define ADV7343_HD_MODE_REG2			0x31
#define ADV7343_HD_MODE_REG3			0x32
#define ADV7343_HD_MODE_REG4			0x33
#define ADV7343_HD_MODE_REG5			0x34
#define ADV7343_HD_MODE_REG6			0x35
#define ADV7343_HD_Y_LEVEL			0x36
#define ADV7343_HD_CR_LEVEL			0x37
#define ADV7343_HD_CB_LEVEL			0x38
#define ADV7343_HD_MODE_REG7			0x39
#define ADV7343_HD_SHARPNESS_FLTR_GAIN		0x40
#define ADV7343_HD_CGMS_DATA_0			0x41
#define ADV7343_HD_CGMS_DATA_1			0x42
#define ADV7343_HD_CGMS_DATA_2			0x43
#define ADV7343_HD_GAMMA_A0			0x44
#define ADV7343_HD_GAMMA_A1			0x45
#define ADV7343_HD_GAMMA_A2			0x46
#define ADV7343_HD_GAMMA_A3			0x47
#define ADV7343_HD_GAMMA_A4			0x48
#define ADV7343_HD_GAMMA_A5			0x49
#define ADV7343_HD_GAMMA_A6			0x4a
#define ADV7343_HD_GAMMA_A7			0x4b
#define ADV7343_HD_GAMMA_A8			0x4c
#define ADV7343_HD_GAMMA_A9			0x4d
#define ADV7343_HD_GAMMA_B0			0x4E
#define ADV7343_HD_GAMMA_B1			0x4F
#define ADV7343_HD_GAMMA_B2			0x50
#define ADV7343_HD_GAMMA_B3			0x51
#define ADV7343_HD_GAMMA_B4			0x52
#define ADV7343_HD_GAMMA_B5			0x53
#define ADV7343_HD_GAMMA_B6			0x54
#define ADV7343_HD_GAMMA_B7			0x55
#define ADV7343_HD_GAMMA_B8			0x56
#define ADV7343_HD_GAMMA_B9			0x57
#define ADV7343_HD_ADPT_FLTR_GAIN1		0x58
#define ADV7343_HD_ADPT_FLTR_GAIN2		0x59
#define ADV7343_HD_ADPT_FLTR_GAIN3		0x5a
#define ADV7343_HD_ADPT_FLTR_THRLDA		0x5b
#define ADV7343_HD_ADPT_FLTR_THRLDB		0x5c
#define ADV7343_HD_ADPT_FLTR_THRLDC		0x5d
#define ADV7343_HD_CGMS_B0			0x5E
#define ADV7343_HD_CGMS_B1			0x5F
#define ADV7343_HD_CGMS_B2			0x60
#define ADV7343_HD_CGMS_B3			0x61
#define ADV7343_HD_CGMS_B4			0x62
#define ADV7343_HD_CGMS_B5			0x63
#define ADV7343_HD_CGMS_B6			0x64
#define ADV7343_HD_CGMS_B7			0x65
#define ADV7343_HD_CGMS_B8			0x66
#define ADV7343_HD_CGMS_B9			0x67
#define ADV7343_HD_CGMS_B10			0x68
#define ADV7343_HD_CGMS_B11			0x69
#define ADV7343_HD_CGMS_B12			0x6A
#define ADV7343_HD_CGMS_B13			0x6B
#define ADV7343_HD_CGMS_B14			0x6C
#define ADV7343_HD_CGMS_B15			0x6D
#define ADV7343_HD_CGMS_B16			0x6E

#define ADV7343_SD_MODE_REG1			0x80
#define ADV7343_SD_MODE_REG2			0x82
#define ADV7343_SD_MODE_REG3			0x83
#define ADV7343_SD_MODE_REG4			0x84
#define ADV7343_SD_MODE_REG5			0x86
#define ADV7343_SD_MODE_REG6			0x87
#define ADV7343_SD_MODE_REG7			0x88
#define ADV7343_SD_MODE_REG8			0x89
#define ADV7343_SD_TIMING_REG0			0x8A
#define ADV7343_SD_TIMING_REG1			0x8B
#define ADV7343_SD_FSC_REG0			0x8C
#define ADV7343_SD_FSC_REG1			0x8D
#define ADV7343_SD_FSC_REG2			0x8E
#define ADV7343_SD_FSC_REG3			0x8F
#define ADV7343_SD_FSC_PHASE			0x90
#define ADV7343_SD_CLOSE_CAPTION_EVEN0		0x91
#define ADV7343_SD_CLOSE_CAPTION_EVEN1		0x92
#define ADV7343_SD_CLOSE_CAPTION_ODD0		0x93
#define ADV7343_SD_CLOSE_CAPTION_ODD1		0x94
#define ADV7343_SD_PEDESTAL_REG0		0x95
#define ADV7343_SD_PEDESTAL_REG1		0x96
#define ADV7343_SD_PEDESTAL_REG2		0x97
#define ADV7343_SD_PEDESTAL_REG3		0x98
#define ADV7343_SD_CGMS_WSS0			0x99
#define ADV7343_SD_CGMS_WSS1			0x9A
#define ADV7343_SD_CGMS_WSS2			0x9B

#define ADV7343_SD_SCALE_LSB			0x9C
#define ADV7343_SD_Y_SCALE			0x9D
#define ADV7343_SD_CB_SCALE			0x9E
#define ADV7343_SD_CR_SCALE			0x9F

#define ADV7343_SD_HUE_REG			0xA0
#define ADV7343_SD_BRIGHTNESS_WSS		0xA1
#define ADV7343_SD_LUMA_SSAF			0xA2
#define ADV7343_SD_DNR0				0xA3
#define ADV7343_SD_DNR1				0xA4
#define ADV7343_SD_DNR2				0xA5

#define ADV7343_SD_GAMMA_A0			0xA6
#define ADV7343_SD_GAMMA_A1			0xA7
#define ADV7343_SD_GAMMA_A2			0xA8
#define ADV7343_SD_GAMMA_A3			0xA9
#define ADV7343_SD_GAMMA_A4			0xAA
#define ADV7343_SD_GAMMA_A5			0xAB
#define ADV7343_SD_GAMMA_A6			0xAC
#define ADV7343_SD_GAMMA_A7			0xAD
#define ADV7343_SD_GAMMA_A8			0xAE
#define ADV7343_SD_GAMMA_A9			0xAF
#define ADV7343_SD_GAMMA_B0			0xB0
#define ADV7343_SD_GAMMA_B1			0xB1
#define ADV7343_SD_GAMMA_B2			0xB2
#define ADV7343_SD_GAMMA_B3			0xB3
#define ADV7343_SD_GAMMA_B4			0xB4
#define ADV7343_SD_GAMMA_B5			0xB5
#define ADV7343_SD_GAMMA_B6			0xB6
#define ADV7343_SD_GAMMA_B7			0xB7
#define ADV7343_SD_GAMMA_B8			0xB8
#define ADV7343_SD_GAMMA_B9			0xB9
#define ADV7343_SD_BRIGHTNESS_DETECT		0xBA
#define ADV7343_FIELD_COUNT_REG			0xBB
#define ADV7343_10_BIT_INPUT			0x7C

/* Default values for the registers */
#define ADV7343_POWER_MODE_REG_DEFAULT		0x10	/* Changed */
#define ADV7343_MODE_SELECT_REG_DEFAULT		0X00
#define ADV7343_MODE_REG0_DEFAULT		0x20	/* Doubt on Sync 
							   signals */
#define ADV7343_DAC1_OUTPUT_LEVEL_DEFAULT	0x00
#define ADV7343_DAC2_OUTPUT_LEVEL_DEFAULT	0x00
#define ADV7343_HD_MODE_REG1_DEFAULT		0x3C	/* Changed Default 
							   720p and EAV/SAV 
							   code */
#define ADV7343_HD_MODE_REG2_DEFAULT		0x01	/* Changed Pixel data 
							   valid */
#define ADV7343_HD_MODE_REG3_DEFAULT		0x00	/* Color delay is 0 
							   clks */
#define ADV7343_HD_MODE_REG4_DEFAULT		0xE8	/* Changed */
#define ADV7343_HD_MODE_REG5_DEFAULT		0x08
#define ADV7343_HD_MODE_REG6_DEFAULT		0x00
#define ADV7343_HD_MODE_REG7_DEFAULT		0x00

#define ADV7343_SD_MODE_REG1_DEFAULT		0x00
#define ADV7343_SD_MODE_REG2_DEFAULT		0xC9	/* Changed */
#define ADV7343_SD_MODE_REG3_DEFAULT		0x10	/* Changed */
#define ADV7343_SD_MODE_REG4_DEFAULT		0x01
#define ADV7343_SD_MODE_REG5_DEFAULT		0x02
#define ADV7343_SD_MODE_REG6_DEFAULT		0x0C	/*0x00 */
#define ADV7343_SD_MODE_REG7_DEFAULT		0x04
#define ADV7343_SD_MODE_REG8_DEFAULT		0x00
#define ADV7343_SD_TIMING_REG0_DEFAULT		0x08
#define ADV7343_SD_TIMING_REG1_DEFAULT		0x00
#define ADV7343_SOFT_RESET_DEFAULT		0x02
#define ADV7343_COMPOSITE_POWER_VALUE		0x80
#define ADV7343_COMPONENT_POWER_VALUE		0x1C
#define ADV7343_SVIDEO_POWER_VALUE		0x60
#define ADV7343_SD_HUE_REG_DEFAULT		127
#define ADV7343_SD_BRIGHTNESS_WSS_DEFAULT	0x03
#define ADV7343_SD_CGMS_WSS0_DEFAULT		0x10

#define ADV7343_SLICED_BUF_SIZE			256
#define ADV7343_SERVICE_LINES_SIZE		(2*24*2)

/* Macros for the Mode Select Register */
#ifdef GENERATE_MASK
#undef GENERATE_MASK
#endif
#define GENERATE_MASK(bits, pos)		((((0xFF) << (8-bits)) >> \
		(8-bits)) << pos)

/* Bit masks for Mode Select Register */
#define YC_Y_BUS				0x80
#define YC_S_BUS				0x7F
#define INPUT_MODE_MASK				0x70
#define SD_INPUT_MODE				0x00
#define HD_720P_INPUT_MODE			0x10
#define HD_1080I_INPUT_MODE			0x10
#define HD_DDR_INPUT_MODE			0x20
#define SD_HD_SDR_INPUT_MODE			0x30
#define SD_HD_DDR_INPUT_MODE			0x40
#define ED_INPUT_MODE				0x70

/* Bit masks for Mode Register 0 */
#define TEST_PATTERN_BLACK_BAR_EN		0x04
#define TEST_PATTERN_BLACK_BAR_DI		0xFB
#define YUV_OUTPUT_SELECT			0x20
#define RGB_OUTPUT_SELECT			0xDF
#define SD_SYNC_OUTPUT_EN			0x40
#define SD_SYNC_OUTPUT_DI			0xBF
#define HD_SYNC_OUTPUT_EN			0x80
#define HD_SYNC_OUTPUT_DI			0x7F
#define CSC_MATRIX_EN				0x08
#define CSC_MATRIX_DI				0xF7

/* Bit masks for CSC matrices */
#define CSC_LSB_MASK				0x03
#define CSC_MSB_MASK				0xFF
#define CSC_SHIFT				2
#define CSC_MATRIX_MASK				0x3FF

/* Bit masks for DAC output levels */
#define DAC_OUTPUT_LEVEL_MASK			0xFF
#define POSITIVE_GAIN_MAX			0x40
#define POSITIVE_GAIN_MIN			0x00
#define NEGATIVE_GAIN_MAX			0xFF
#define NEGATIVE_GAIN_MIN			0xC0

/* Bit masks for soft reset register */
#define SOFT_RESET				0x02

/* Bit masks for HD Mode Register 1 */
#define OUTPUT_STD_MASK				0x03
#define OUTPUT_STD_SHIFT			0
#define OUTPUT_STD_EIA0_2			0x00
#define OUTPUT_STD_EIA0_1			0x01
#define OUTPUT_STD_FULL				0x02
#define EMBEDDED_SYNC				0x04
#define EXTERNAL_SYNC				0xFB
#define STD_MODE_SHIFT				3
#define STD_MODE_MASK				0x1F
#define STD_MODE_720P				0x05
#define STD_MODE_720P_25			0x08
#define STD_MODE_720P_30			0x07
#define STD_MODE_720P_50			0x06
#define STD_MODE_1080I				0x0D
#define STD_MODE_1080I_25fps			0x0E
#define STD_MODE_1080P_24			0x12
#define STD_MODE_1080P_25			0x10
#define STD_MODE_1080P_30			0x0F
#define STD_MODE_525P				0x00
#define STD_MODE_625P				0x03

/* Bit masks for HD Mode Register 2 */
#define HD_PIXEL_DATA_VALID			0x01
#define HD_TEST_PATTERN_EN			0x04
#define HD_TEST_PATTERN_DI			0xFB
#define HD_TEST_PATTERN_HATCH			0xF7
#define HD_TEST_PATTERN_FRAME			0x08
#define HD_VBI_EN				0x10
#define HD_VBI_DI				0xEF
#define HD_UNDERSHOOTER_DI			0x9F
#define HD_UNDERSHOOTER_11IRE			0x20
#define HD_UNDERSHOOTER_6IRE			0x40
#define HD_UNDERSHOOTER_1_5IRE			0x60
#define HD_SHARPNESS_FLTR_EN			0x80
#define HD_SHARPNESS_FLTR_DI			0x7F

/* Bit masks for HD Mode Register 3 */
#define HD_HSYNC_Y_DELAY_SHIFT			0
#define HD_HSYNC_Y_DELAY_MASK			0x07
#define HD_HSYNC_C_DELAY_SHIFT			3
#define HD_HSYNC_C_DELAY_MASK			0x38
#define HD_CGMS_EN				0x40
#define HD_CGMS_DI				0xBF
#define HD_CGMS_CRC_EN				0x80
#define HD_CGMS_CRC_DI				0x7F

/* Bit masks for HD Mode Register 4 */
#define HD_HSYNC_CR				0x01
#define HD_HSYNC_CB				0xFE
#define HD_SYNC_FLTR_EN				0x08
#define HD_SYNC_FLTR_DI				0xF7
#define HD_CHROMA_SSAF_EN			0x20
#define HD_CHROMA_SSAF_DI			0xDF
#define HD_CHROMA_INPUT_422			0x40
#define HD_CHROMA_INPUT_444			0xBF
#define HD_DOUBLE_BUFFERING_EN			0x80
#define HD_DOUBLE_BUFFERING_DI			0x7F

/* Bit masks for HD Mode Register 5 */
#define HD_MACROVISION_EN			0x10
#define HD_MACROVISION_DI			0xEF

/* Bit masks for HD Mode Register 6 */
#define HD_RGB_INPUT_EN				0x02
#define HD_RGB_INPUT_DI				0xFD
#define HD_PBPR_SYNC_EN				0x04
#define HD_PBPR_SYNC_DI				0xFB
#define HD_DAC_SWAP_EN				0x08
#define HD_DAC_SWAP_DI				0xF7
#define HD_GAMMA_CURVE_A			0xEF
#define HD_GAMMA_CURVE_B			0x10
#define HD_GAMMA_EN				0x20
#define HD_GAMMA_DI				0xDF
#define HD_ADPT_FLTR_MODEB			0x40
#define HD_ADPT_FLTR_MODEA			0xBF
#define HD_ADPT_FLTR_EN				0x80
#define HD_ADPT_FLTR_DI				0x7F

/* Bit masks for HD Sharpness filter */
#define HD_SHARPNESS_FLTR_A_SHIFT		0
#define HD_SHARPNESS_FLTR_A_MASK		0x0F
#define HD_SHARPNESS_FLTR_B_SHIFT		4
#define HD_SHARPNESS_FLTR_B_MASK		0x0F

#define GAMMA_MASK				0xFF

#define HD_ADPT_FLTR_GAIN_A_SHIFT		0
#define HD_ADPT_FLTR_GAIN_A_MASK		0x0F
#define HD_ADPT_FLTR_GAIN_B_SHIFT		4
#define HD_ADPT_FLTR_GAIN_B_MASK		0x0F
#define HD_ADPT_FLTR_THRLD_MASK			0xFF

/* Bit masks for SD Mode Register 1 */
#define SD_STD_MASK				0x03
#define SD_STD_NTSC				0x00
#define SD_STD_PAL_BDGHI			0x01
#define SD_STD_PAL_M				0x02
#define SD_STD_PAL_N				0x03
#define SD_LUMA_FLTR_MASK			0x7
#define SD_LUMA_FLTR_SHIFT			0x2
#define SD_CHROMA_FLTR_MASK			0x7
#define SD_CHROMA_FLTR_SHIFT			0x5

/* Bit masks for SD Mode Register 2 */
#define SD_PBPR_SSAF_EN				0x01
#define SD_PBPR_SSAF_DI				0xFE
#define SD_DAC_1_DI				0xFD
#define SD_DAC_2_DI				0xFB
#define SD_PEDESTAL_EN				0x08
#define SD_PEDESTAL_DI				0xF7
#define SD_SQUARE_PIXEL_EN			0x10
#define SD_SQUARE_PIXEL_DI			0xEF
#define SD_PIXEL_DATA_VALID			0x40
#define SD_ACTIVE_EDGE_EN			0x80
#define SD_ACTIVE_EDGE_DI			0x7F

/* Bit masks for SD Mode Register 3 */
#define SD_CLOSE_CAPTION_DI			0x9F
#define SD_CLOSE_CAPTION_EVEN			0x40
#define SD_CLOSE_CAPTION_ODD			0x20
#define SD_CLOSE_CAPTION_BOTH			0x60
#define SD_VBI_EN				0x10
#define SD_VBI_DI				0xEF
#define SD_PEDESTAL_YPBPR_EN			0X01
#define SD_PEDESTAL_YPBPR_DI			0xFE

/* Bit masks for SD Mode Register 4 */
#define SD_CHROMA_EN				0x10
#define SD_CHROMA_DI				0xEF
#define SD_BURST_EN				0x20
#define SD_BURST_DI				0xDF
#define SD_COLOR_BARS_EN			0x40
#define SD_COLOR_BARS_DI			0xBF

/* Bit masks for SD Mode Register 6 */
#define SD_PBPR_SCALE_EN			0x01
#define SD_PBPR_SCALE_DI			0xFE
#define SD_Y_SCALE_EN				0x02
#define SD_Y_SCALE_DI				0xFD
#define SD_HUE_ADJST_EN				0x04
#define SD_HUE_ADJST_DI				0xFB
#define SD_BRIGHTNESS_EN			0x08
#define SD_BRIGHTNESS_DI			0xF7
#define SD_LUMA_SSAF_GAIN_EN			0x10
#define SD_LUMA_SSAF_GAIN_DI			0xEF
#define SD_AUTO_DETECT_EN			0x40
#define SD_AUTO_DETECT_DI			0xBF
#define SD_RGB_INPUT_EN				0x80
#define SD_RGB_INPUT_DI				0x7F

/* Bit masks for SD Mode Register 7 */
#define SD_NON_INTERLACED_EN			0x02
#define SD_NON_INTERLACED_DI			0xFD
#define SD_DOUBLE_BUFFERING_EN			0x04
#define SD_DOUBLE_BUFFERING_DI			0xFB
#define SD_DNR_EN				0x20
#define SD_DNR_DI				0xDF
#define SD_GAMMA_EN				0x40
#define SD_GAMMA_DI				0xBF
#define SD_GAMMA_CURVE_B			0x80
#define SD_GAMMA_CURVE_A			0x7F
#define SD_INPUT_FORMAT_8BIT			0x00
#define SD_INPUT_FORMAT_16BIT			0x08

/* Bit masks for SD Timing Register 0 */
#define SD_MASTER_MODE				0x01
#define SD_SLAVE_MODE				0xFE
#define SD_TIMING_MODE_SHIFT			1
#define SD_TIMING_MODE_MASK			0x03
#define SD_TIMING_MODE0				0xFC
#define SD_TIMING_MODE1				0x01
#define SD_TIMING_MODE2				0x02
#define SD_TIMING_MODE3				0x03
#define SD_LUMA_DELAY				0
#define SD_MIN_LUMA_VAL_7_5IRE			0x40
#define SD_MIN_LUMA_VAL_40IRE			0xBF
#define SD_TIMING_RESET				0x80

/* Macros for Timing Mode Register 1 */
#define SD_HSYNC_DELAY				0
#define SD_HSYNC2VSYNC_DELAY			0
#define SD_HSYNC2DATA_DELAY			0
#define SD_HSYNC2PIXELDATA_ADJUST		0

#define SD_FSC_REG_MASK				0xFF
#define SD_CLOSED_CAPT_MASK			0xFF
#define SD_PEDESTAL_MASK			0xFF

#define SD_CGMS_CRC_EN				0x10
#define SD_CGMS_CRC_DI				0xEF
#define SD_CGMS_EN				0x60
#define SD_CGMS_ODD_FIELD_EN			0x20
#define SD_CGMS_ODD_FIELD_DI			0xDF
#define SD_CGMS_EVEN_FIELD_EN			0x40
#define SD_CGMS_EVEN_FIELD_DI			0xBF
#define SD_CGMS_DI				0x9F
#define SD_WSS_EN				0x80
#define SD_WSS_DI				0x7F

#define SD_BRIGHTNESS_MASK			0x7F

#define SD_DNR_BORDER_GAIN_MASK			0x0F
#define SD_DNR_BORDER_GAIN_SHIFT		0x0
#define SD_DNR_DATA_GAIN_MASK			0x0F
#define SD_DNR_DATA_GAIN_SHIFT			0x4

#define SD_DNR_THRESHOLD_MASK			0x3F
#define SD_DNR_THRESHOLD_MAX			0x3F
#define SD_BORDER_AREA_4PIXELS			0x40
#define SD_BORDER_AREA_2PIXELS			0xBF
#define SD_BLOCK_SIZE_16x16			0x80
#define SD_BLOCK_SIZE_8x8			0x7F
#define SD_DNR_INPUT_SELECT_MASK		0x07
#define SD_DNR_SHARPNESS_MODE			0x08
#define SD_DNR_MODE				0xF7
#define SD_DNR_BLOCK_OFFSET_MASK		0x0F
#define SD_DNR_BLOCK_OFFSET_SHIFT		0x04
#define SD_DNR_GAIN_MAX				0x08

#define SD_YPBPR_SCALE_SHIFT			0x02
#define SD_YPBPR_SCALE_LMASK			0x03
#define SD_YPBPR_SCALE_HMASK			0x3FC
#define SD_YPBPR_SCALE_MASK			0x3FF

#define SD_LUMA_SSAF_GAIN_MAX			0x0C

#endif				/* End of #ifdef __KERNEL__ */

#endif				/* End of #ifndef ADV7343_H */
