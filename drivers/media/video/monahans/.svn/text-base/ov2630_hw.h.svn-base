/*
   Copyright (C) 2005, Intel Corporation.
   Copyright (C) 2006, Marvell International Ltd.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Abstract:
 *	contains all OV2630 specific macros, typedefs, and prototypes.
 *	Declares no storage.
 *
 * Notes:
 *	Only valid for processor code named Monahans.
 */

#ifndef __MONAHANS_CAM_OV2630_HW_HEADER__
#define __MONAHANS_CAM_OV2630_HW_HEADER__

/*
 * Constants & Structures
 */
/* Revision constants */
#define PIDH_OV2630              0x26
#define PIDL_OV2630              0x33

/* Return codes */
#define OV_ERR_NONE             0x00
#define OV_ERR_TIMEOUT          -1
#define OV_ERR_PARAMETER        -2
#define OV_COMM_ERR             -3

/* OV2630 Mode */
typedef enum {
	OV2630_UXGA = 0,
	OV2630_CIF,
	OV2630_SVGA,
	OV2630_INVALID
}OV2630_MODE;

/* Camera Mode */
#define VIEWFINDER_MODE         0x10
#define STILLFRAME_MODE         0x20

/* Others */
#define OV2630_TIMEOUT          1000    /* ms to timeout */

#define OV2630_GAIN             0x00    /* AGC Gain Control */
#define OV2630_BLUE_GAIN        0x01    /* Blue Gain Control */
#define OV2630_RED_GAIN         0x02    /* Red Gain Control */
#define OV2630_COMA             0x03    /* Common Control A */
#define OV2630_COMB             0x04    /* Common Control B */
#define OV2630_BAVG             0x05    /* B Channel Average */
/* G Channel Average - Picked G pixels in the same line with B pixels */
#define OV2630_GbAVE            0x06
/* G Channel Average - Picked G pixels in the same line with R pixels */
#define OV2630_GrAVE            0x07
#define OV2630_RAVG             0x08	/* R Channel Average */
#define OV2630_COMC             0x09	/* Common control C */
#define OV2630_PIDH             0x0A    /* Product ID Number MSBs */
#define OV2630_PIDL             0x0B    /* Product ID Number LSBs */
#define OV2630_COMD             0x0C    /* Common control D */
#define OV2630_COME             0x0D    /* Common control E */
#define OV2630_COMF             0x0E    /* Common control F */
#define OV2630_COMG             0x0F    /* Common control G */
#define OV2630_AEC              0x10    /* Automatic Exposure [10:3] */
#define OV2630_CLKRC            0x11    /* Clock Rate Control */
#define OV2630_COMH             0x12    /* Common Control H */
#define OV2630_COMI             0x13    /* Common control I */
#define OV2630_COMJ             0x14    /* Common Control J */
#define OV2630_COMK             0x15    /* Common Control K */
#define OV2630_HREFST           0x17    /* Horizontal Window Start */
#define OV2630_HREFEND          0x18    /* Horizontal window End */
#define OV2630_VSTRT            0x19    /* Vertical Window Line Start */
#define OV2630_VEND             0x1A    /* Vertical Window Line End */
#define OV2630_PSHFT            0x1B    /* Pixel Shift */
#define OV2630_MIDH             0x1C    /* Manufacturer ID Byte - High */
#define OV2630_MIDL             0x1D    /* Manufacturer ID Byte - Low */
#define OV2630_BOFF             0x20    /* B Channel Offset Adjustment */
#define OV2630_GbOFF            0x21    /* Gb Channel Offset Adjustment */
#define OV2630_GrOFF            0x22    /* Gr Channel Offset Adjustment */
#define OV2630_ROFF             0x23    /* R Channel Offset Adjustment */

/* Luminance Signal High Range for AEC/AGC Operation */
#define OV2630_AEW              0x24
/* Luminance Signal Low Range for AEC/AGC Operation */
#define OV2630_AEB              0x25
/* Fast Mode Large Step Range Threshold */
#define OV2630_VV               0x26
/* B Channel Offset Manual Adjustment Value */
#define OV2630_BBIAS            0x27
/* Gb Channel Offset Manual Adjustment Value */
#define OV2630_GbBIAS           0x28
/* Gr Channel Offset Manual Adjustment Value */
#define OV2630_GrBIAS           0x29
#define OV2630_COML             0x2A    /* Common Control L */
/* Line Interval Adjustment Value LSB 8 bits */
#define OV2630_FRARL            0x2B
/* R Channel Offset Manual Adjustment Value */
#define OV2630_RBIAS            0x2C
/* VSYNC Pulse Width LSB 8 bits */
#define OV2630_ADVSL            0x2D
/* VSYNC Pulse Width MSB 8 bits */
#define OV2630_ADVSH            0x2E
#define OV2630_YAVG             0x2F    /* Luminance Average */
/* HSYNC Position and Width Start Point LSB 8 bits */
#define OV2630_HSDY             0x30
/* HSYNC Position and Width End Lower 8 bits */
#define OV2630_HENY             0x31
#define OV2630_COMM             0x32    /* Common Control M */
#define OV2630_REG33            0x33    /* Current Control */
#define OV2630_VCHG             0x36    /* Sensor Precharge Voltage Control */
#define OV2630_ADC              0x37    /* ADC Reference Control */
#define OV2630_ACOM             0x38    /* Analog Common Control */
#define OV2630_REG39            0x39    /* R39 */
#define OV2630_REG3A            0x3A    /* Sensor Internal Reference Control */
#define OV2630_REG3C            0x3C    /* Sensor Internal Reference Control */
/* Gb/Gr Channel Manual Offset Compensation */
#define OV2630_DKOFFBR          0x3E
#define OV2630_REG3F            0x3F    /* Sensor Offset Control */
#define OV2630_BBLC             0x40    /* B Channel Black Level Value */
#define OV2630_RBLC             0x41    /* R Channel Black Level Value */
#define OV2630_GbBLC            0x42    /* Gb Channel Black Level Value */

/* End of OV2630 register */
#define OV2630_REGEND        (0x8D + 1)

/*
 * Function Prototype
 */
int ov2630hw_set_regs(const u8 *regP);
int ov2630hw_read_all_regs(u8 *bufP, u32 numRegs);

void ov2630hw_power_down(u8 powerMode);
void ov2630hw_reset(void);
void ov2630hw_wait(int ms);

int  ov2630hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision);
void ov2630hw_set_hsync(void);
void ov2630hw_auto_function_on(void);
void ov2630hw_auto_function_off(void);

int ov2630hw_viewfinder_on(void);
int ov2630hw_viewfinder_off(void);
int ov2630hw_halt_video_output(void);
int ov2630hw_resumeto_full_output_mode(void);
int ov2630hw_get_single_image(void);

int ov2630hw_set_format(u32 captureWidth,
		u32 captureHeight,
		u32 *winStartX,
		u32 *winStartY,
		u32 *winEndX,
		u32 *winEndY);

int ov2630hw_read_sensor_reg(const u8 subAddress, u8 *bufP);
int ov2630hw_write_sensor_reg(const u8 subAddress, u8 *bufP);
int ov2630hw_set_contrast(u32 value);
int ov2630hw_set_exposure(u32 value);
int ov2630hw_set_white_balance(u32 value);


#endif

