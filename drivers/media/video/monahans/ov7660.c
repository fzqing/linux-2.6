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
 *	contains all primitive functions for OV7660
 *
 * Notes:
 *	Only valid for processor code named Monahans.
 */

#include <linux/module.h>
#include "camera.h"
#include "ci.h"
#include "ov7660.h"
#include "ov7660_hw.h"
#include <linux/delay.h>

#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>

#ifdef HW_IP_OV7660

static unsigned char lut_table[] = {
	/* RED LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,

	/* BLUE LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,

	/* GREEN LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,
};

static CI_CMU_COE_MATRIX ov7660_cRGB_sRGB_COE = {
	RGB_FLOAT_TO_INT(0),  RGB_FLOAT_TO_INT(0), RGB_FLOAT_TO_INT(1),
	RGB_FLOAT_TO_INT(0),  RGB_FLOAT_TO_INT(1), RGB_FLOAT_TO_INT(0),
	RGB_FLOAT_TO_INT(1),  RGB_FLOAT_TO_INT(0), RGB_FLOAT_TO_INT(0)
};

static CI_CMU_COE_MATRIX ov7660_cRGB_YUV_COE = {
	YUV_FLOAT_TO_INT(0.098), YUV_FLOAT_TO_INT(0.504),
	YUV_FLOAT_TO_INT(0.257),

	YUV_FLOAT_TO_INT(0.439), YUV_FLOAT_TO_INT(-0.291),
	YUV_FLOAT_TO_INT(-0.148),

	YUV_FLOAT_TO_INT(-0.071), YUV_FLOAT_TO_INT(-0.368),
	YUV_FLOAT_TO_INT(0.439),
}; /* This is not provided by OmniVision yet */
#endif

/*
 * OV7660 Functions
 */
int ov7660_init(p_camera_context_t camera_context)
{
	u8 cm_rev, cm_pid;
	u32 timeout;
	int status;

	/* provide informat about the capabilities of the sensor */
	camera_context->sensor_status.caps |= SENSOR_CAP_MANUAL_CONTRAST |
		SENSOR_CAP_MANUAL_WHITEBALANCE |
		SENSOR_CAP_MANUAL_EXPOSURE;

	/* Configure CI according to OV7660's hardware
	 * master parallel with 8 data pins
	 */
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);

	/* enable pixel clock(sensor will provide pclock)
	 * and master clock = 26MHZ
	 */
	/* software work around*/
#if defined(CONFIG_PXA310)
	ci_set_clock(1, 1, 1300);
#else
	ci_set_clock(1, 1, 2600);
#endif

	/* data sample on rising and h,vsync active high */
	ci_set_polarity(0, 0, 0);

	/* fifo control */
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_0_TO_7);


	/* OV7660 Power on sequence
	 * Take out of Power down mode (GPIO_103), PWRDWN=1, NORMAL=0
	 * Assert Reset
	 * Delay
	 * Remove reset
	 * Delay
	 */
	ov7660hw_power_down(CAMERA_POWER_FULL);
	ov7660hw_reset();

	/* read out version */
	timeout = 50;
	do {
		cm_pid = cm_rev = 0;
		status = ov7660hw_version_revision(&cm_pid, &cm_rev);

		/* Check to make sure we are working with an OV7660 */
		if (cm_pid != PID_OV76XX || cm_rev != PID_7660) {
			ov7660hw_power_down(CAMERA_POWER_OFF);
			ov7660hw_power_down(CAMERA_POWER_FULL);
			ov7660hw_reset();
			mdelay(1);
		}
		if (--timeout == 0) {
			return -EIO;
		}
	} while (cm_pid != PID_OV76XX);

	/* turn sensor output off */
	ov7660hw_viewfinder_off();

	return 0;
}

int ov7660_deinit(p_camera_context_t camera_context)
{
	/* power off the external module */
	ov7660hw_power_down(CAMERA_POWER_OFF);

	return 0;
}

int ov7660_sleep(p_camera_context_t camera_context)
{
	return ov7660_deinit(camera_context);
}

int ov7660_wake(p_camera_context_t camera_context)
{
	return ov7660_init(camera_context);
}

int ov7660_set_capture_format(p_camera_context_t camera_context)
{
	CI_MP_TIMING timing;
	int status;
	u32 ovSizeFormat, ovFormat;

	/* Set the current mode */
	switch (camera_context->capture_input_format) {
		case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
		case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
			ovFormat = OV_FORMAT_YUV_422;
			break;
		case CAMERA_IMAGE_FORMAT_RGB565:
			ovFormat = OV_FORMAT_RGB_565;
			break;
		case CAMERA_IMAGE_FORMAT_RAW8:
			ovFormat = OV_FORMAT_RAW8;
			break;
		default:
			ovFormat = OV_FORMAT_YUV_422;
			break;
	}
	if (camera_context->capture_input_width == 88 &&
			camera_context->capture_input_height == 72)
		ovSizeFormat = OV_SIZE_QQCIF;
	else if (camera_context->capture_input_width == 176 &&
			camera_context->capture_input_height == 144)
		ovSizeFormat = OV_SIZE_QCIF;
	else if (camera_context->capture_input_width == 352 &&
			camera_context->capture_input_height == 288)
		ovSizeFormat = OV_SIZE_CIF;
	else if (camera_context->capture_input_width == 160 &&
			camera_context->capture_input_height == 120)
		ovSizeFormat = OV_SIZE_QQVGA;
	else if (camera_context->capture_input_width == 320 &&
			camera_context->capture_input_height == 240)
		ovSizeFormat = OV_SIZE_QVGA;
	else if (camera_context->capture_input_width == 640 &&
			camera_context->capture_input_height == 480)
		ovSizeFormat = OV_SIZE_VGA;
	else
		ovSizeFormat = OV_SIZE_VGA;

#ifdef HW_IP_OV7660
	if (camera_context->cmu_usage == CI_CMU_OUTPUT_YUV) {
		ci_cmu_set_color_correction_coe(&ov7660_cRGB_YUV_COE);
	}

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_RGB) {
		ci_cmu_set_color_correction_coe(&ov7660_cRGB_sRGB_COE);
	}

	ci_cmu_enable(camera_context->cmu_usage);
#endif

	status = ov7660hw_set_format(ovSizeFormat,
			ovFormat, camera_context->capture_mode);

	/* set capture width/height and timing */
	if (ovFormat == OV_FORMAT_RAW8) {
		timing.BFW = 0x01;
		timing.BLW = 0x01; /*GRBG to RGGB*/
	} else {
		timing.BFW = 0x00;
		timing.BLW = 0x00;
	}
	ci_configure_mp(camera_context->capture_input_width-1,
			camera_context->capture_input_height-1, &timing);

#if defined(CONFIG_PXA310)
	ci_set_ycbcr_420_down_sample (camera_context->ycbcr_ds);
#endif

	return 0;
}

int ov7660_start_capture(p_camera_context_t camera_context,
		unsigned int frames)
{
#ifdef HW_IP_OV7660
	if (camera_context->cgu_enable) {
		ci_cgu_load_lut_ram(
				(unsigned int*)camera_context->
					histogram_lut_buffer_virtual,
				camera_context->histogram_lut_buffer_physical,
				(unsigned int*)camera_context->
					histogram_lut_dma_descriptors_virtual,
				camera_context->
					histogram_lut_dma_descriptors_physical,
				lut_table);
	}
	ci_cgu_enable(camera_context->cgu_enable);
#endif

	/* turn auto function on only doing continues capture */
	if (frames == 0) {
		ov7660hw_auto_function_on();
	} else {
		ov7660hw_auto_function_off();
	}

	/* turn viewfinder on */
	ov7660hw_viewfinder_on();
	return 0;
}

int ov7660_stop_capture(p_camera_context_t camera_context)
{
	/* turn auto function off */
	ov7660hw_auto_function_off();

	/* turn viewfinder off */
	ov7660hw_viewfinder_off();

	return 0;
}

int ov7660_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	ov7660hw_power_down(mode);
	return 0;
}

int ov7660_read_8bit(p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val)
{
	ov7660hw_read_sensor_reg(reg_addr, reg_val);
	return 0;
}

int ov7660_write_8bit(p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val)
{
	u8 buffer;
	int status;

	buffer = reg_val;
	status = (ov7660hw_write_sensor_reg(reg_addr, &buffer) == 0) ?
		0 : -EIO;

	return status;
}

int ov7660_set_contrast(p_camera_context_t camera_context, u8 mode, u32 value)
{
	if (mode == SENSOR_MANUAL_CONTRAST)
		return ov7660hw_set_contrast(value);
	else
		return 0;
}

int ov7660_set_exposure(p_camera_context_t camera_context, u8 mode, u32 value)
{
	if (mode == SENSOR_MANUAL_EXPOSURE)
		return ov7660hw_set_exposure(value);
	else
		return 0;
}

int ov7660_set_white_balance(p_camera_context_t camera_context,
		u8 mode, u32 value)
{
	if (mode == SENSOR_MANUAL_WHITEBALANCE)
		return ov7660hw_set_white_balance(value);
	else
		return 0;
}
