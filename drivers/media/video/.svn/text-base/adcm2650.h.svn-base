/*
 * drivers/media/video/adcm2650.h
 *
 * ADCM 2650 Camera Module driver.
 *
 * Author: Intel Corporation
 *
 * 2003 (c) Intel Corporation
 * 2003-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _PXA_ADCM_2650_HW_H__
#define _PXA_ADCM_2650_HW_H__

/* i2c command */
#define ADCM2650_BLOCK_SWITCH_CMD	((u8)0xfe)
#define ADCM2650_BLOCK(a)		(u8)(((a) >> 7) & 0xff )	/* register's module block address. */
#define ADCM2650_OFFSET(a)		(u8)(((a) << 1) & 0xff )	/* register's offset to this block. */

/* Registers and it's bitfields */
#define ADCM2650_REV		0x0000

#define ADCM2650_CMD_1          0x0002
#define ADCM2650_CMD_1_VE        0x0001
#define ADCM2650_CMD_1_LPE       0x0002

#define ADCM2650_CMD_2          0x0004
#define ADCM2650_CMD_2_SNAP     0x0001
#define ADCM2650_CMD_2_ACS      0x0002
#define ADCM2650_CMD_2_AVT      0x0004
#define ADCM2650_CMD_2_AST      0x0008
#define ADCM2650_CMD_2_SGO      0x0010
#define ADCM2650_CMD_2_P_RESET  0x0020
#define ADCM2650_CMD_2_S_RESET  0x0040
#define ADCM2650_CMD_2_U_RESET  0x0080
#define ADCM2650_CMD_2_PLL_ON   0x0100
#define ADCM2650_CMD_2_UCGS     0x0200
#define ADCM2650_CMD_2_PLL_OFF  0x0400
#define ADCM2650_CMD_2_IA       0x1000
#define ADCM2650_CMD_2_UART_F   0x2000
#define ADCM2650_CMD_2_SVWC     0x4000
#define ADCM2650_CMD_2_SVHC     0x8000

#define ADCM2650_VIDEO_CONFIG   0x000c
#define ADCM2650_VIDEO_CONFIG_SS             0x0004
#define ADCM2650_VIDEO_CONFIG_SHP            0x0008
#define ADCM2650_VIDEO_CONFIG_H_MIRROR       0x0010
#define ADCM2650_VIDEO_CONFIG_V_MIRROR       0x0020
#define ADCM2650_VIDEO_CONFIG_O_FORMAT_SHIFT 8
#define ADCM2650_VIDEO_CONFIG_O_FORMAT_MASK  (0xF<<ADCM2650_VIDEO_CONFIG_O_FORMAT_SHIFT)
#define ADCM2650_VIDEO_CONFIG_JPEG_MASK      0x2000

#define ADCM2650_STILL_CONFIG   0x000e
#define ADCM2650_STILL_CONFIG_SS             0x0004
#define ADCM2650_STILL_CONFIG_SHP            0x0008
#define ADCM2650_STILL_CONFIG_H_MIRROR       0x0010
#define ADCM2650_STILL_CONFIG_V_MIRROR       0x0020
#define ADCM2650_STILL_CONFIG_O_FORMAT_SHIFT 8
#define ADCM2650_STILL_CONFIG_O_FORMAT_MASK  (0xF<<ADCM2650_STILL_CONFIG_O_FORMAT_SHIFT)
#define ADCM2650_STILL_CONFIG_JPEG_MASK      0x2000

#define ADCM2650_TM_SELECT       0x0010

#define ADCM2650_BYPASS_CTRL     0x0012
#define ADCM2650_BYPASS_CTRL_BPA     0x0001
#define ADCM2650_BYPASS_CTRL_G1G2    0x0002
#define ADCM2650_BYPASS_CTRL_SB      0x0004
#define ADCM2650_BYPASS_CTRL_AQA     0x0008
#define ADCM2650_BYPASS_CTRL_PCD     0x0010
#define ADCM2650_BYPASS_CTRL_QT_HEAD 0x0020
#define ADCM2650_BYPASS_CTRL_WGF     0x0040
#define ADCM2650_BYPASS_CTRL_OLP     0x0080
#define ADCM2650_BYPASS_CTRL_CAM_CON 0x0100
#define ADCM2650_BYPASS_CTRL_DDFF    0x0200

#define ADCM2650_PROCESS_CTRL    0x0014
#define ADCM2650_OUTPUT_CTRL     0x0016
#define ADCM2650_CCIR_TIMING_1   0x0018
#define ADCM2650_Y_MAX_MIN       0x001a
#define ADCM2650_CbCr_MAX_MIN    0x001c
#define ADCM2650_BAD_FRAME_DIS   0x001e
#define ADCM2650_UART_PCKT_SIZE  0x0020
#define ADCM2650_UART_CRDT_ADD   0x0022
#define ADCM2650_UART_CREDITS    0x0024

#define ADCM2650_SZR_IN_W_VID   0x0026
#define ADCM2650_SZR_IN_H_VID   0x0028
#define ADCM2650_SZR_OUT_W_VID  0x002a
#define ADCM2650_SZR_OUT_H_VID  0x002c
#define ADCM2650_SZR_IN_W_STL   0x002e
#define ADCM2650_SZR_IN_H_STL   0x0030
#define ADCM2650_SZR_OUT_W_STL  0x0032
#define ADCM2650_SZR_OUT_H_STL  0x0034

#define ADCM2650_SENSOR_ADDRESS	0x004a

#define ADCM2650_SENSOR_DATA_1  0x004c

#define ADCM2650_SENSOR_DATA_2  0x004e

#define ADCM2650_SENSOR_CTRL    0x0050
#define ADCM2650_SENSOR_CTRL_RW 0x0010
#define ADCM2650_SENSOR_CTRL_GO 0x0020

#define ADCM2650_EXT_DIVBY_VID 0x005a
#define ADCM2650_EXT_DIVBY_STL 0x005c

#define ADCM2650_S_EXT_DIVBY_VID 0x012e
#define ADCM2650_S_EXT_DIVBY_STL 0x0130

#define ADCM2650_AF_STATUS      0x2024
#define ADCM2650_AF_STATUS_CC   0x0008

#define ADCM2650_MASTER_CLK_FREQ 0x2026

/* Sensor Register Offset */
#define ADCM2650_SENSOR_IDENT           0x00
#define ADCM2650_SENSOR_STATUS          0x01
#define ADCM2650_SENSOR_ICTRL           0x05
#define ADCM2650_SENSOR_ITMG            0x06
#define ADCM2650_SENSOR_FWROW           0x0a
#define ADCM2650_SENSOR_FWCOL           0x0b
#define ADCM2650_SENSOR_LWROW           0x0c
#define ADCM2650_SENSOR_LWCOL           0x0d
#define ADCM2650_SENSOR_TCTRL           0x0e
#define ADCM2650_SENSOR_ERECPGA         0x0f
#define ADCM2650_SENSOR_EROCPGA         0x10
#define ADCM2650_SENSOR_ORECPGA         0x11
#define ADCM2650_SENSOR_OROCPGA         0x12
#define ADCM2650_SENSOR_ROWEXPL         0x13
#define ADCM2650_SENSOR_ROWEXPH         0x14
#define ADCM2650_SENSOR_SROWEXP         0x15
#define ADCM2650_SENSOR_ERROR           0x16
#define ADCM2650_SENSOR_HBLANK          0x19
#define ADCM2650_SENSOR_VBLANK          0x1a
#define ADCM2650_SENSOR_CONFIG          0x1b
#define ADCM2650_SENSOR_CONTROL         0x1c
#define ADCM2650_SENSOR_TEST0           0x1d
#define ADCM2650_SENSOR_TEST1           0x1e
#define ADCM2650_SENSOR_TEST2           0x1f
#define ADCM2650_SENSOR_TEST3           0x20
#define ADCM2650_SENSOR_TEST4           0x24
#define ADCM2650_SENSOR_TEST5           0x25
#define ADCM2650_SENSOR_CONFIG_2        0x27

/* Revision constants */
#define ADCM2650_REV_PIPE	0x0600
#define ADCM2650_REV_SENSOR	0x60

/* Auto Exposure Frequency */
#define ADCM2650_AEF_50HZ	0x20
#define ADCM2650_AEF_60HZ	0x40

/* Non JEPG Output Format */
#define ADCM2650_O_FORMAT_888RGB         0	/* 888 RGB (1 pixel in 3 bytes ) */
#define ADCM2650_O_FORMAT_666_A_RGB      1	/* 666 A RGB (tight pack, 4 pixels in 9 bytes) */
#define ADCM2650_O_FORMAT_666_B_RGB      2	/* 666 B RGB (loose pack, 1 pixel in 3 bytes,left or right justified) */
#define ADCM2650_O_FORMAT_565_RGB        3	/* 565 RGB (1 pixel in 2 bytes) */
#define ADCM2650_O_FORMAT_444_A_RGB      4	/* 444 A RGB (tight pack, 2 pixels per 3 bytes, RG BR GB) */
#define ADCM2650_O_FORMAT_444_B_RGB      5	/* 444 B RGB (loose pack, 1 pixel per 2 bytes,RG B0 or 0R GB) */
#define ADCM2650_O_FORMAT_444_C_RGV      6	/* 444 C RGB (sparse pack, 1 pixel per three bytes,R0 G0 B0 or 0R 0G 0B) */
#define ADCM2650_O_FORMAT_332_RGB        7	/* 332 RGB (1 pixel in 1 byte) */
#define ADCM2650_O_FORMAT_422_A_YCbYCr   8	/* 4:2:2 A YCbYCr (Y1 Cb12 Y2 CRL2 order) */
#define ADCM2650_O_FORMAT_422_B_YCbYCr   9	/* 4:2:2 B YCbYCr (Cb12 Y1 CRL2 Y2 order) */
#define ADCM2650_O_FORMAT_422_C_YCbYCr   10	/* 4:2:2 C YCbYCr (Y1 CRL2 Y2 Cb12 order) */
#define ADCM2650_O_FORMAT_422_D_YCbYCr   11	/* 4:2:2 D YCbYCr (CRL2 Y1 Cb12 Y2 order) */
#define ADCM2650_O_FORMAT_444_YCbYCr     12	/* 4:4:4 YCbCr (1 pixels per 3 bytes) */
#define ADCM2650_O_FORMAT_400_B_YCbYCr   13	/* 4:0:0 YCbCr (Greyscale, 1 pixel per 1 byte) */
#define ADCM2650_O_FORMAT_RAWBPA         14	/* RAWBPA (with AWB and BPA) */
#define ADCM2650_O_FORMAT_RAW            15	/* RAW (without AWB and BPA) */

/* Camera Mode */
#define ADCM2650_MODE_VIEWFINDER	0x10
#define ADCM2650_MODE_STILLFRAME	0x20

/* misc */
#define ADCM2650_TIMEOUT		1000
#define ADCM2650_SENSOR_SLAVE_ADDR	0x0055

extern int adcm2650_init(void);
extern void adcm2650_exit(void);

/* Read and Write from/to Image Pipeline Registers */
extern int adcm2650_pipeline_read(u16 reg_addr, u16 * reg_value);
extern int adcm2650_pipeline_write(u16 reg_addr, u16 reg_value);
extern int adcm2650_pipeline_read_rl(u16 reg_addr, u16 mask, u16 field);
extern int adcm2650_pipeline_write_wa(u16 reg_addr, u16 mask);
extern int adcm2650_pipeline_write_wo(u16 reg_addr, u16 mask);

/*
 * Read and Write from/to Image Sensor Registers:
 * The image sensor registers are 8bit wide
 */
extern int adcm2650_sensor_read_rs(u8 reg_addr, u8 * reg_value);
extern int adcm2650_sensor_write_ws(u8 reg_addr, u8 reg_value);

/* helper functions */
extern int adcm2650_set_output_format(unsigned int output_format);
extern int adcm2650_firmware_upgrade(void);
extern int adcm2650_switch_to_normal(void);

#endif
