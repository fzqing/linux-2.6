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
 *    contains all OV2620 specific macros, typedefs, and prototypes.
 * Declares no storage.
 *	Notes: Only valid for processor code named Monahans.
 */

#ifndef __MONAHANS_CAM_OV2620_HW_HEADER__
#define __MONAHANS_CAM_OV2620_HW_HEADER__

/*
 * Constants & Structures
 */
/*
 * Revision constants
 */
#define PID_OV26XX              0x26
#define PID_2620                0x20

/*
 * Return codes
 */
#define OV_ERR_NONE             0x00
#define OV_ERR_TIMEOUT          -1
#define OV_ERR_PARAMETER        -2
#define OV_COMM_ERR             -3

/*
 * OV2620 Mode
 */
typedef enum {
    OV2620_UXGA = 0,
    OV2620_CIF,
    OV2620_SVGA,
    OV2620_INVALID
}OV2620_MODE;

/*
 * Camera Mode
 */
#define VIEWFINDER_MODE         0x10
#define STILLFRAME_MODE         0x20

/*
 * Others
 */
#define OV2620_TIMEOUT          1000    /* ms to timeout. */

#define OV2620_GAIN             0x00    /* AGC Gain Control */
#define OV2620_BLUE_GAIN        0x01    /* Blue Gain Control */
#define OV2620_RED_GAIN         0x02    /* Red Gain Control */
#define OV2620_COMA             0x03    /* Common Control A */
#define OV2620_COMB             0x04    /* Common Control B */
#define OV2620_BAVG             0x05    /* B Channel Average */
#define OV2620_GbAVE            0x06    /* G Channel Average - Picked G pixels in the same line with B pixels */
#define OV2620_RAVG             0x08    /* R Channel Average */
#define OV2620_COMC             0x09    /* Common control C */
#define OV2620_PIDH             0x0A    /* Product ID Number MSBs */
#define OV2620_PIDL             0x0B    /* Product ID Number LSBs */
#define OV2620_COMD             0x0C    /* Common control D */
#define OV2620_COME             0x0D    /* Common control E */
#define OV2620_COMF             0x0E    /* Common control F */
#define OV2620_COMG             0x0F    /* Common control G */
#define OV2620_AEC              0x10    /* Automatic Exposure [10:3] */
#define OV2620_CLKRC            0x11    /* Clock Rate Control */
#define OV2620_COMH             0x12    /* Common Control H */
#define OV2620_COMI             0x13    /* Common control I */
#define OV2620_COMJ             0x14    /* Common Control J */
#define OV2620_COMK             0x15    /* Common Control K */
#define OV2620_HREFST           0x17    /* Horizontal Window Start */
#define OV2620_HREFEND          0x18    /* Horizontal window End */
#define OV2620_VSTRT            0x19    /* Vertical Window Line Start */
#define OV2620_VEND             0x1A    /* Vertical Window Line End */
#define OV2620_PSHFT            0x1B    /* Pixel Shift */
#define OV2620_MIDH             0x1C    /* Manufacturer ID Byte - High */
#define OV2620_MIDL             0x1D    /* Manufacturer ID Byte - Low */
#define OV2620_BOFF             0x20    /* B Channel Offset Adjustment */
#define OV2620_GbOFF            0x21    /* Gb Channel Offset Adjustment */
#define OV2620_GrOFF            0x22    /* Gr Channel Offset Adjustment */
#define OV2620_ROFF             0x23    /* R Channel Offset Adjustment */
#define OV2620_AEW              0x24    /* Luminance Signal High Range for AEC/AGC Operation */
#define OV2620_AEB              0x25    /* Luminance Signal Low Range for AEC/AGC Operation */
#define OV2620_VV               0x26    /* Fast Mode Large Step Range Threshold */
#define OV2620_BBIAS            0x27    /* B Channel Offset Manual Adjustment Value */
#define OV2620_GbBIAS           0x28    /* Gb Channel Offset Manual Adjustment Value */
#define OV2620_GrBIAS           0x29    /* Gr Channel Offset Manual Adjustment Value */
#define OV2620_COML             0x2A    /* Common Control L */
#define OV2620_FRARL            0x2B    /* Line Interval Adjustment Value LSB 8 bits */
#define OV2620_RBIAS            0x2C    /* R Channel Offset Manual Adjustment Value */
#define OV2620_ADVSL            0x2D    /* VSYNC Pulse Width LSB 8 bits */
#define OV2620_ADVSH            0x2E    /* VSYNC Pulse Width MSB 8 bits */
#define OV2620_YAVG             0x2F    /* Luminance Average */
#define OV2620_HSDY             0x30    /* HSYNC Position and Width Start Point LSB 8 bits */
#define OV2620_HENY             0x31    /* HSYNC Position and Width End Lower 8 bits */
#define OV2620_COMM             0x32    /* Common Control M */
#define OV2620_CHLF             0x33    /* Current Control */
#define OV2620_VCHG             0x36    /* Sensor Precharge Voltage Control */
#define OV2620_ADC              0x37    /* ADC Reference Control */
#define OV2620_ACOM             0x38    /* Analog Common Control */
#define OV2620_R39              0x39    /* R39 */
#define OV2620_HV               0x3A    /* Sensor Internal Reference Control */
#define OV2620_FVO              0x3C    /* Sensor Internal Reference Control */
#define OV2620_BOFFS            0x3E    /* Analog B Channel BLC Offset control */
#define OV2620_GbOFFS           0x3F    /* Analog Gb Channel BLC Offset control */
#define OV2620_GrOFFS           0x40    /* Analog Gr Channel BLC Offset control */
#define OV2620_ROFFS            0x41    /* Analog R Channel BLC Offset control */
#define OV2620_COMO             0x42    /* Common Control O */

/*
 * End of OV2620 register
 */
#define OV2620_REGEND        (OV2620_COMO + 1)


/*
 * Function Prototype
 */
int ov2620hw_set_regs(const u8 *regP);
int ov2620hw_read_all_regs(u8 *bufP, u32 numRegs);

void ov2620hw_power_down(u8 powerMode);
void ov2620hw_reset(void);
void ov2620hw_wait(int ms);

int  ov2620hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision);
void ov2620hw_set_hsync(void);
void ov2620hw_auto_function_on(void);
void ov2620hw_auto_function_off(void);

int ov2620hw_view_finder_on(void);
int ov2620hw_view_finder_off(void);
int ov2620hw_halt_video_output(void);
int ov2620hw_resume_full_output_mode(void);
int ov2620hw_get_single_image(void);

int ov2620hw_set_format(u32 captureWidth,
                              u32 captureHeight,
                              u32 *winStartX,
                              u32 *winStartY,
                              u32 *winEndX,
                              u32 *winEndY);

int ov2620hw_read_sensor_reg(const u8 subAddress, u8 *bufP);
int ov2620hw_write_sensor_reg(const u8 subAddress, u8 *bufP);
int ov2620hw_set_contrast(u32 value);
int ov2620hw_set_exposure(u32 value);
int ov2620hw_set_white_balance(u32 value);

#endif

