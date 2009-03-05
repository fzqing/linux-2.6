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
/* adv7343.c */
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <media/davinci/adv7343.h>
#include <asm/arch/i2c-client.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <media/davinci/vid_encoder_if.h>

unsigned char reg0 = 0x20;	/* MODE_REG0 */
unsigned char reg1 = 0x00;	/* MODE_SELECT_REG */
unsigned char reg2 = ADV7343_SD_MODE_REG2_DEFAULT;	/* SD_MODE_REG2 */
unsigned char reg6 = 0x00;	/* HD_MODE_REG6 */
unsigned char reg80 = ADV7343_SD_MODE_REG1_DEFAULT;	/* SD_MODE_REG1 */

unsigned char reg3 = 0x0C;	/* SD_MODE_REG6 */
unsigned char reg4 = 0x00;	/* SD_SCALE_LSB */
unsigned char reg5 = 0x04;	/* SD_MODE_REG7 */
unsigned char reg7 = 0xE8;	/* HD_MODE_REG4 */
unsigned char reg9 = 0x01;	/* HD_MODE_REG2 */
unsigned char reg10 = 0x3C;	/* HD_MODE_REG1 */
unsigned char reg11 = 0x80;	/* Power Mode register */
unsigned char reg12 = 0x10;	/* SD_CGMS_WSS */
unsigned char reg13 = 0x10;	/* SD_MODE_REG3 */

static int adv7343_initialize(struct vid_encoder_device *enc, int flag);
static int adv7343_deinitialize(struct vid_encoder_device *enc);

static int adv7343_setstd(struct vid_enc_mode_info *mode_info,
			  struct vid_encoder_device *enc);
static int adv7343_getstd(struct vid_enc_mode_info *mode_info,
			  struct vid_encoder_device *enc);

static int adv7343_setoutput(char *output, struct vid_encoder_device *enc);
static int adv7343_getoutput(char *output, struct vid_encoder_device *enc);
static int adv7343_enumoutput(int index,
			      char *output, struct vid_encoder_device *enc);

static int adv7343_set_params(void *params, struct vid_encoder_device *enc);
static int adv7343_get_params(void *params, struct vid_encoder_device *enc);

static int adv7343_setcontrol(enum vid_enc_ctrl_id ctrl,
			      unsigned char val,
			      struct vid_encoder_device *enc);
static int adv7343_getcontrol(enum vid_enc_ctrl_id ctrl,
			      unsigned char *val,
			      struct vid_encoder_device *enc);

static int adv7343_set_sdparams(struct adv7343_sd_params *, void *enc);
static int adv7343_set_hdparams(struct adv7343_hd_params *, void *enc);
static int adv7343_set_dnrparams(struct adv7343_dnr_params *, void *enc);
static int adv7343_set_ssafparams(struct adv7343_ssaf_params *, void *enc);
static int adv7343_set_filterparams(struct adv7343_filter_params *, void *enc);
static int adv7343_set_rgbmatrix(struct adv7343_rgb_matrix *, void *enc);
static int adv7343_set_gammaparams(struct adv7343_gamma_params *params,
				   enum adv7343_params_type type, void *enc);

static int adv7343_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);
static int adv7343_write_vbi_data(struct vid_enc_sliced_vbi_data *data,
				  struct vid_encoder_device *enc);
static int adv7343_set_vbi_services(struct vid_enc_sliced_vbi_service
				    *services, struct vid_encoder_device *enc);
static int adv7343_get_sliced_cap(struct vid_enc_sliced_vbi_service
				  *service, struct vid_encoder_device *enc);

/* i2c function prototypes */
static int adv7343_i2c_attach_client(struct i2c_client *,
				     struct i2c_driver *,
				     struct i2c_adapter *, int);
static int adv7343_i2c_detach_client(struct i2c_client *);
static int adv7343_i2c_probe_adapter(struct i2c_adapter *);
static int adv7343_i2c_init(void);
static void adv7343_i2c_cleanup(void);

static int adv7343_start_display(struct vid_encoder_device *);
static int adv7343_stop_display(struct vid_encoder_device *);

/* following function is used to set ths7303 */
static int ths7303_setvalue(struct vid_enc_mode_info *mode_info);

static struct adv7343_control_info
    adv7343_controls[ADV7343_COMPOSITE_NO_CONTROLS] = {
	{
	 .register_address = ADV7343_DAC1_OUTPUT_LEVEL,
	 .value = 0,
	 .id = VID_ENC_CTRL_GAIN,
	 .minimum = 0,
	 .maximum = 255},
	{
	 .register_address = ADV7343_SD_BRIGHTNESS_WSS,
	 .value = 0,
	 .id = VID_ENC_CTRL_BRIGHTNESS,
	 .minimum = 0,
	 .maximum = 127},
	{
	 .register_address = ADV7343_SD_HUE_REG,
	 .value = 0,
	 .id = VID_ENC_CTRL_HUE,
	 .minimum = 0,
	 .maximum = 255}
};

static struct vid_enc_mode_info
    adv7343_composite_standards[ADV7343_COMPOSITE_NUM_STD] = {
	{
	 .name = VID_ENC_STD_NTSC,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT656,
	 .interlaced = 1,
	 .xres = 720,
	 .yres = 480,
	 .fps = {30000, 1001},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_PAL,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT656,
	 .interlaced = 1,
	 .xres = 720,
	 .yres = 576,
	 .fps = {25, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0}
};

static struct vid_enc_mode_info
    adv7343_component_standards[ADV7343_COMPONENT_NUM_STD] = {
	{
	 .name = VID_ENC_STD_720P_60,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1280,
	 .yres = 720,
	 .fps = {60000, 1000},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_720P_25,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1280,
	 .yres = 720,
	 .fps = {25, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_720P_30,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1280,
	 .yres = 720,
	 .fps = {30, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_720P_50,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1280,
	 .yres = 720,
	 .fps = {50, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_1080I_30,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 1,
	 .xres = 1920,
	 .yres = 1080,
	 .fps = {30000, 1001},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_1080I_25,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 1,
	 .xres = 1920,
	 .yres = 1080,
	 .fps = {25, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_480P_60,
	 .std = 1,
	 .if_type = VID_ENC_IF_YCC16,
	 .interlaced = 0,
	 .xres = 720,
	 .yres = 480,
	 .fps = {60, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_576P_50,
	 .std = 1,
	 .if_type = VID_ENC_IF_YCC16,
	 .interlaced = 0,
	 .xres = 720,
	 .yres = 576,
	 .fps = {50, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_NTSC,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT656,
	 .interlaced = 1,
	 .xres = 720,
	 .yres = 480,
	 .fps = {30000, 1001},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_PAL,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT656,
	 .interlaced = 1,
	 .xres = 720,
	 .yres = 576,
	 .fps = {25, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_1080P_24,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1920,
	 .yres = 1080,
	 .fps = {24, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_1080P_25,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1920,
	 .yres = 1080,
	 .fps = {25, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_1080P_30,
	 .std = 1,
	 .if_type = VID_ENC_IF_BT1120,
	 .interlaced = 0,
	 .xres = 1920,
	 .yres = 1080,
	 .fps = {30, 1},
	 .left_margin = 0,
	 .right_margin = 0,
	 .upper_margin = 0,
	 .lower_margin = 0,
	 .hsync_len = 0,
	 .vsync_len = 0,
	 .flags = 0},
};

struct adv7343_std_info adv7343_composite_std_info[ADV7343_COMPOSITE_NUM_STD] = {
	{
	 ADV7343_SD_MODE_REG1, &reg80, SD_INPUT_MODE, (~(SD_STD_MASK)),
	 SD_STD_NTSC, 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_SD_MODE_REG1, &reg80, SD_INPUT_MODE, (~(SD_STD_MASK)),
	 SD_STD_PAL_BDGHI, 0x8C, 0xCB, 0x8D, 0x8A, 0x8E, 0x09, 0x8F, 0x2A}
};

struct adv7343_std_info adv7343_component_std_info[ADV7343_COMPONENT_NUM_STD] = {
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_720P_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_720P << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_720P_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_720P_25 << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_720P_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_720P_30 << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_720P_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_720P_50 << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_1080I_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_1080I << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_1080I_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_1080I_25fps << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_720P_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_525P << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_720P_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_625P << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_SD_MODE_REG1, &reg80, SD_INPUT_MODE, (~(SD_STD_MASK)),
	 SD_STD_NTSC, 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_SD_MODE_REG1, &reg80, SD_INPUT_MODE, (~(SD_STD_MASK)),
	 SD_STD_PAL_BDGHI, 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_1080I_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_1080P_24 << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_1080I_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_1080P_25 << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21},
	{
	 ADV7343_HD_MODE_REG1, &reg10, HD_1080I_INPUT_MODE,
	 (~(STD_MODE_MASK << STD_MODE_SHIFT)),
	 (STD_MODE_1080P_30 << STD_MODE_SHIFT)
	 , 0x8C, 0x1F, 0x8D, 0x7C, 0x8E, 0xF0, 0x8F, 0x21}
};

static struct adv7343_config adv7343_configuration[ADV7343_NUM_CHANNELS] = {
	{
	 .no_of_outputs = ADV7343_MAX_NO_OUTPUTS,
	 .output[0] = {
		       .output_type = ADV7343_COMPOSITE_ID,
		       .output_name = VID_ENC_OUTPUT_COMPOSITE,
		       .no_of_standard = ADV7343_COMPOSITE_NUM_STD,
		       .standard = (struct vid_enc_mode_info *)
		       &adv7343_composite_standards,
		       .def_std = &adv7343_composite_standards[0],
		       .std_info = (struct adv7343_std_info *)
		       &adv7343_composite_std_info,
		       .no_of_controls = ADV7343_COMPOSITE_NO_CONTROLS,
		       .controls = (struct adv7343_control_info *)
		       &adv7343_controls,
		       .power_val = ADV7343_COMPOSITE_POWER_VALUE},
	 .output[1] = {
		       .output_type = ADV7343_COMPONENT_ID,
		       .output_name = VID_ENC_OUTPUT_COMPONENT,
		       .no_of_standard = ADV7343_COMPONENT_NUM_STD,
		       .standard = (struct vid_enc_mode_info *)
		       &adv7343_component_standards,
		       .def_std = &adv7343_component_standards[0],
		       .std_info = (struct adv7343_std_info *)
		       &adv7343_component_std_info,
		       .no_of_controls = ADV7343_COMPONENT_NO_CONTROLS,
		       .controls = (struct adv7343_control_info *)
		       &adv7343_controls,
		       .power_val = ADV7343_COMPONENT_POWER_VALUE},
	 .output[2] = {
		       .output_type = ADV7343_SVIDEO_ID,
		       .output_name = VID_ENC_OUTPUT_SVIDEO,
		       .no_of_standard = ADV7343_SVIDEO_NUM_STD,
		       .standard = (struct vid_enc_mode_info *)
		       &adv7343_composite_standards,
		       .def_std = &adv7343_composite_standards[0],
		       .std_info = (struct adv7343_std_info *)
		       &adv7343_composite_std_info,
		       .no_of_controls = ADV7343_SVIDEO_NO_CONTROLS,
		       .controls = (struct adv7343_control_info *)
		       &adv7343_controls,
		       .power_val = ADV7343_SVIDEO_POWER_VALUE},
	 .services_set = VID_ENC_SLICED_VBI_WSS_PAL |
	 VID_ENC_SLICED_VBI_CGMS_NTSC | VID_ENC_SLICED_VBI_CC_NTSC,
	 .num_services = 0}
};

static struct adv7343_service_data_reg
    adv7343_services_regs[ADV7343_VBI_NUM_SERVICES] = {
	{
	 .service_set = VID_ENC_SLICED_VBI_CC_NTSC,
	 .field = {
		   {
		    .addr = {ADV7343_SD_CLOSE_CAPTION_ODD0,
			     ADV7343_SD_CLOSE_CAPTION_ODD1}
		    },
		   {
		    .addr = {ADV7343_SD_CLOSE_CAPTION_EVEN0,
			     ADV7343_SD_CLOSE_CAPTION_EVEN1}
		    }
		   },
	 .bytestowrite = 2},
	{
	 .service_set = VID_ENC_SLICED_VBI_WSS_PAL,
	 .field = {
		   {
		    .addr = {ADV7343_SD_CGMS_WSS2,
			     ADV7343_SD_CGMS_WSS1}
		    },
		   {
		    .addr = {ADV7343_SD_CGMS_WSS2,
			     ADV7343_SD_CGMS_WSS1}
		    }
		   },
	 .bytestowrite = 2}
};

static struct adv7343_service_reg adv7343_sliced_reg[ADV7343_VBI_NUM_SERVICES] = {

	{
	 .service = VID_ENC_SLICED_VBI_WSS_PAL,
	 .reg = ADV7343_SD_CGMS_WSS0,
	 .reg_val = &reg12,
	 .enable_val = SD_WSS_EN,
	 .disable_val = SD_WSS_DI,
	 .std = &adv7343_composite_standards[1]},
	{
	 .service = VID_ENC_SLICED_VBI_CC_NTSC,
	 .reg = ADV7343_SD_MODE_REG3,
	 .reg_val = &reg13,
	 .enable_val = SD_CLOSE_CAPTION_BOTH,
	 .disable_val = SD_CLOSE_CAPTION_DI,
	 .std = &adv7343_composite_standards[0]},
	{
	 .service = VID_ENC_SLICED_VBI_CGMS_NTSC,
	 .reg = ADV7343_SD_CGMS_WSS0,
	 .reg_val = &reg12,
	 .enable_val = SD_CGMS_EN,
	 .disable_val = SD_CGMS_DI,
	 .std = &adv7343_composite_standards[0]}
};

static struct adv7343_channel adv7343_channel_info[ADV7343_NUM_CHANNELS] = {
	{
	 .current_output = ADV7343_COMPOSITE_ID,
	 .mode_info = &adv7343_composite_standards[0],
	 .i2c_dev = {
		     .i2c_addr = (0x54 >> 1),
		     .i2c_registration = 0},
	 .enc_device = NULL}
};

/* Global variables */
static struct device *adv7343_i2c_dev[ADV7343_NUM_CHANNELS];

/* Global structures variables */
static struct vid_enc_param_ops params_ops = {
	.setparams = adv7343_set_params,
	.getparams = adv7343_get_params
};

static struct vid_enc_control_ops controls_ops = {
	.setcontrol = adv7343_setcontrol,
	.getcontrol = adv7343_getcontrol
};

static struct vid_enc_output_ops outputs_ops = {
	.count = ADV7343_MAX_NO_OUTPUTS,
	.enumoutput = adv7343_enumoutput,
	.setoutput = adv7343_setoutput,
	.getoutput = adv7343_getoutput
};

static struct vid_enc_mode_ops standards_ops = {
	.setmode = adv7343_setstd,
	.getmode = adv7343_getstd,
};

static struct vid_encoder_device adv7343_dev[ADV7343_NUM_CHANNELS] = {
	{
	 .name = "ADV7343",
	 .channel_id = 0,
	 .capabilities = 0,
	 .initialize = adv7343_initialize,
	 .mode_ops = &standards_ops,
	 .ctrl_ops = &controls_ops,
	 .output_ops = &outputs_ops,
	 .params_ops = &params_ops,
	 .deinitialize = adv7343_deinitialize,
	 .misc_ops = NULL,
	 .write_vbi_data = adv7343_write_vbi_data,
	 .enable_vbi = NULL,
	 .enable_hbi = NULL,
	 .set_vbi_services = adv7343_set_vbi_services,
	 .get_sliced_cap = adv7343_get_sliced_cap,
	 .start_display = adv7343_start_display,
	 .stop_display = adv7343_stop_display}
};

/* This function is called by the vpif driver to initialize ADV7343 driver.
 * It initializes all registers of adv7343 with the default values
 */
static int adv7343_initialize(struct vid_encoder_device *enc, int flag)
{
	int err = 0;
	int ch_id;
	struct i2c_client *ch_client;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	ch_client = &adv7343_channel_info[ch_id].i2c_dev.client;
	if (adv7343_channel_info[ch_id].i2c_dev.i2c_registration & 0x01) {
		printk(KERN_ERR "adv7343 driver is already initialized..\n");
		return err;
	}

	/* Register ADV7343 I2C client */
	err = i2c_add_driver(&adv7343_channel_info[ch_id].i2c_dev.driver);
	if (err) {
		printk(KERN_ERR "Failed to register ADV7343 I2C client.\n");
		return -EINVAL;
	}
	adv7343_channel_info[ch_id].i2c_dev.i2c_registration |= 1;
	adv7343_channel_info[ch_id].enc_device = enc;
	if (VID_ENC_I2C_BIND_FLAG == flag) {
		return err;
	}

	dev_dbg(adv7343_i2c_dev[ch_id], "ADV7343 driver registered\n");

	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SOFT_RESET,
				     ADV7343_SOFT_RESET_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_POWER_MODE_REG,
				     ADV7343_POWER_MODE_REG_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_MODE_SELECT_REG,
				     ADV7343_MODE_SELECT_REG_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_MODE_REG0,
				     ADV7343_MODE_REG0_DEFAULT);

	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_DAC1_OUTPUT_LEVEL,
				     ADV7343_DAC1_OUTPUT_LEVEL_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_DAC2_OUTPUT_LEVEL,
				     ADV7343_DAC2_OUTPUT_LEVEL_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG1,
				     ADV7343_HD_MODE_REG1_DEFAULT);

	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG2,
				     ADV7343_HD_MODE_REG2_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG3,
				     ADV7343_HD_MODE_REG3_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG4,
				     ADV7343_HD_MODE_REG4_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG5,
				     ADV7343_HD_MODE_REG5_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG6,
				     ADV7343_HD_MODE_REG6_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_HD_MODE_REG7,
				     ADV7343_HD_MODE_REG7_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG1,
				     ADV7343_SD_MODE_REG1_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG2,
				     ADV7343_SD_MODE_REG2_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG3,
				     ADV7343_SD_MODE_REG3_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG4,
				     ADV7343_SD_MODE_REG4_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG5,
				     ADV7343_SD_MODE_REG5_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG6,
				     ADV7343_SD_MODE_REG6_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG7,
				     ADV7343_SD_MODE_REG7_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_MODE_REG8,
				     ADV7343_SD_MODE_REG8_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_TIMING_REG0,
				     ADV7343_SD_TIMING_REG0_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_TIMING_REG0,
				     ADV7343_SD_TIMING_REG0_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_HUE_REG,
				     ADV7343_SD_HUE_REG_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_CGMS_WSS0,
				     ADV7343_SD_CGMS_WSS0_DEFAULT);
	err |= adv7343_i2c_write_reg(ch_client,
				     ADV7343_SD_BRIGHTNESS_WSS,
				     ADV7343_SD_BRIGHTNESS_WSS_DEFAULT);

	if (err < 0) {
		err = -EINVAL;
		adv7343_deinitialize(enc);
		return err;
	} else {
		adv7343_channel_info[ch_id].current_output = 0;
		adv7343_channel_info[ch_id].mode_info =
		    &adv7343_composite_standards[0];
		/* Configure for default video standard */
		/* call set standard */
		err |=
		    adv7343_setoutput(adv7343_configuration[ch_id].
				      output[0].output_name, enc);
		err |=
		    adv7343_setstd(adv7343_configuration[ch_id].output[0].
				   def_std, enc);
		if (err < 0) {
			err = -EINVAL;
			adv7343_deinitialize(enc);
			return err;
		}
	}
	adv7343_channel_info[ch_id].i2c_dev.i2c_registration |= 2;
	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_initialize>\n");
	return err;
}

static int adv7343_deinitialize(struct vid_encoder_device *enc)
{
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "ADV7343 ch \
			deinitialization called\n");

	if (adv7343_channel_info[ch_id].i2c_dev.i2c_registration & 0x01) {
		i2c_del_driver(&adv7343_channel_info[ch_id].i2c_dev.driver);
		adv7343_channel_info[ch_id].i2c_dev.client.adapter = NULL;
		adv7343_channel_info[ch_id].i2c_dev.i2c_registration &= ~(0x01);
		adv7343_channel_info[ch_id].enc_device = NULL;
	}
	return 0;
}

int adv7343_start_display(struct vid_encoder_device *enc)
{
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "ADV7343 ch \
		start display called\n");

	try_module_get(THIS_MODULE);
	return 0;
}

int adv7343_stop_display(struct vid_encoder_device *enc)
{
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "ADV7343 ch \
			start display called\n");

	module_put(THIS_MODULE);
	return 0;
}

static int adv7343_set_vbi_services(struct vid_enc_sliced_vbi_service
				    *services, struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	unsigned char val1 = 0;
	int i;
	u8 num_services = 0;

	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "adv7343_set_vbi_services\n");

	if (NULL == services) {
		dev_err(adv7343_i2c_dev[ch_id], "NULL pointer.\n");
		return -EINVAL;
	}
	if ((services->service_set |
	     adv7343_configuration[ch_id].services_set) !=
	    adv7343_configuration[ch_id].services_set) {
		dev_err(adv7343_i2c_dev[ch_id], "Invalid service\n");
		return -EINVAL;
	}
	for (i = 0; i < ADV7343_VBI_NUM_SERVICES; i++) {
		if ((services->service_set & adv7343_sliced_reg[i].service)
		    && (0 !=
			strcmp(adv7343_sliced_reg[i].std->name,
			       adv7343_channel_info[ch_id].mode_info->name))) {
			dev_err(adv7343_i2c_dev[ch_id],
				"Standard doesn't " "support this service\n");
			return -EINVAL;
		}
		val1 = *adv7343_sliced_reg[i].reg_val;

		if (services->service_set & adv7343_sliced_reg[i].service) {
			val1 |= adv7343_sliced_reg[i].enable_val;
			num_services++;
		} else {
			val1 &= adv7343_sliced_reg[i].disable_val;
		}

		err |=
		    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].
					  i2c_dev.client,
					  adv7343_sliced_reg[i].reg, val1);
		*adv7343_sliced_reg[i].reg_val = val1;
	}
	adv7343_configuration[ch_id].num_services = num_services;
	adv7343_channel_info[ch_id].services_set = services->service_set;
	return (err < 0) ? err : num_services;
}

/* This function is used to write the vbi data to the encoder device */
static int adv7343_write_vbi_data(struct vid_enc_sliced_vbi_data *data,
				  struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	int i = 0, j, k;
	unsigned char val1 = 0, val2;
	u8 num_services;

	if (NULL == enc) {
		printk(KERN_ERR "adv7343_write_vbi_data:NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "Start of adv7343_write_vbi_data..\n");
	if (NULL == data) {
		dev_err(adv7343_i2c_dev[ch_id], "adv7343_write_vbi_data:"
			"NULL pointer.\n");
		return -EINVAL;
	}
	num_services = adv7343_configuration[ch_id].num_services;
	for (i = 0; i < num_services; i++) {
		if (0 == data[i].service_id)
			continue;
		if ((data[i].service_id | adv7343_channel_info[ch_id].
		     services_set) !=
		    adv7343_channel_info[ch_id].services_set) {
			printk(KERN_ERR "%d Service Id = %x\n", i,
			       data[i].service_id);
			dev_err(adv7343_i2c_dev[ch_id], "Invalid Service\n");
			return -EINVAL;
		}
		for (j = 0; j < ADV7343_VBI_NUM_SERVICES; j++) {
			if (!(adv7343_services_regs[j].service_set &
			      data[i].service_id))
				if (data[i].service_id !=
				    VID_ENC_SLICED_VBI_CGMS_NTSC)
					continue;
			if (data[i].service_id & VID_ENC_SLICED_VBI_CGMS_NTSC) {
				err |=
				    adv7343_i2c_write_reg
				    (&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_CGMS_WSS2,
				     data[i].data[0]);

				err |=
				    adv7343_i2c_write_reg
				    (&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_CGMS_WSS1,
				     data[i].data[1]);

				val1 = *adv7343_sliced_reg[2].reg_val;
				val2 = data[i].data[2] & 0x0F;
				val1 |= val2;
				err |=
				    adv7343_i2c_write_reg
				    (&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_CGMS_WSS0, val1);
				if (err < 0) {
					dev_err(adv7343_i2c_dev[ch_id],
						"adv7343_write_vbi_data failed\n");
					return err;
				}
			} else {

				for (k = 0; k <
				     adv7343_services_regs[j].bytestowrite;
				     k++) {
					err |=
					    adv7343_i2c_write_reg
					    (&adv7343_channel_info[ch_id].
					     i2c_dev.client,
					     adv7343_services_regs[j].
					     field[data[i].field].addr[k],
					     data[i].data[0 + k]);
					if (err < 0) {
						dev_err(adv7343_i2c_dev
							[ch_id],
							"adv7343_write_vbi_data"
							"failed\n");
						return err;
					}
				}
			}
		}
	}
	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_write_vbi_data>\n");
	return err;
}

/* This function is used to set the value of the control like brightness,
   hue */
static int adv7343_setcontrol(enum vid_enc_ctrl_id ctrl,
			      unsigned char val, struct vid_encoder_device *enc)
{
	int err = 0;
	int value;
	int output_idx;
	int ch_id, i = 0;
	struct adv7343_control_info *control = NULL;
	int no_of_controls;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_setcontrol>\n");
	output_idx = adv7343_channel_info[ch_id].current_output;

	value = val;
	if ((strcmp(adv7343_channel_info[ch_id].mode_info->name,
		    VID_ENC_STD_NTSC) == 0) ||
	    (strcmp(adv7343_channel_info[ch_id].mode_info->name,
		    VID_ENC_STD_PAL) == 0)) {
		no_of_controls = 3;
	} else {
		no_of_controls
		    = adv7343_configuration[ch_id].output[output_idx].
		    no_of_controls;
	}

	for (i = 0; i < no_of_controls; i++) {
		control = &adv7343_configuration[ch_id].output[output_idx].
		    controls[i];
		if (control->id == ctrl) {
			break;
		}
	}
	if (i == no_of_controls) {
		return -EINVAL;
	}
	if ((control->minimum > value)
	    || (control->maximum < value)) {
		return -EINVAL;
	}
	if (VID_ENC_CTRL_GAIN == ctrl) {
		if ((value > POSITIVE_GAIN_MAX)
		    && (value < NEGATIVE_GAIN_MIN)) {
			return -EINVAL;
		} else {
			err |= adv7343_i2c_write_reg(&adv7343_channel_info
						     [ch_id].i2c_dev.
						     client,
						     ADV7343_DAC2_OUTPUT_LEVEL,
						     value);
		}
	}

	err = adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				    client, control->register_address, value);

	if (err < 0) {
		dev_err(adv7343_i2c_dev[ch_id],
			"ADV7343 set control fails...\n");
		return err;
	}
	control->value = value;

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_setcontrol>\n");
	return err;
}

/* This function is used to get the value of the control */
static int adv7343_getcontrol(enum vid_enc_ctrl_id ctrl,
			      unsigned char *val,
			      struct vid_encoder_device *enc)
{
	int err = 0, i;
	int ch_id;
	struct adv7343_control_info *control = NULL;
	int output_idx;
	int no_of_controls;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "Starting getctrl of adv7343...\n");
	output_idx = adv7343_channel_info[ch_id].current_output;

	/* check for null pointer */
	if (val == NULL) {
		dev_err(adv7343_i2c_dev[ch_id], "NULL pointer\n");
		return -EINVAL;
	}
	if ((strcmp(adv7343_channel_info[ch_id].mode_info->name,
		    VID_ENC_STD_NTSC) == 0) ||
	    (strcmp(adv7343_channel_info[ch_id].mode_info->name,
		    VID_ENC_STD_PAL) == 0)) {
		no_of_controls = 3;
	} else {
		no_of_controls
		    = adv7343_configuration[ch_id].output[output_idx].
		    no_of_controls;
	}

	for (i = 0; i < no_of_controls; i++) {
		control =
		    &adv7343_configuration[ch_id].output[output_idx].
		    controls[i];
		if (control->id == ctrl) {
			break;
		}
	}
	if (i == no_of_controls) {
		dev_err(adv7343_i2c_dev[ch_id], "Invalid id...\n");
		return -EINVAL;
	}

	*val = control->value;

	if (err < 0) {
		dev_err(adv7343_i2c_dev[ch_id],
			"ADV7343 get control fails...\n");
		return err;
	}
	if (VID_ENC_CTRL_BRIGHTNESS == ctrl) {
		(*val) &= SD_BRIGHTNESS_MASK;
	}

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_getcontrol>\n");
	return err;
}

/* following function is used to set ths7303 */
static int ths7303_setvalue(struct vid_enc_mode_info *mode)
{
	int err = 0;
	u8 val[2];
	u8 val1;
	u16 ths7303_i2c_addr = 0x2C;

	if (NULL == mode) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	if ((strcmp(mode->name, VID_ENC_STD_NTSC) == 0) ||
	    (strcmp(mode->name, VID_ENC_STD_PAL) == 0))
		val1 = 0x02;
	else if ((strcmp(mode->name, VID_ENC_STD_480P_60) == 0) ||
		 (strcmp(mode->name, VID_ENC_STD_576P_50) == 0))
		val1 = 0x4A;
	else
		val1 = 0x92;

	val[0] = 0x01;
	val[1] = val1;
	err = davinci_i2c_write(2, val, ths7303_i2c_addr);
	val[0] = 0x02;
	val[1] = val1;
	err = davinci_i2c_write(2, val, ths7303_i2c_addr);
	val[0] = 0x03;
	val[1] = val1;
	err = davinci_i2c_write(2, val, ths7303_i2c_addr);
	if (err) {
		printk(KERN_ERR "ths7303\n");
	}
	mdelay(100);
	return err;
}

static int adv7343_setstd(struct vid_enc_mode_info *mode_info,
			  struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	int i = 0;
	struct vid_enc_mode_info *standard;
	int output_idx;
	unsigned char val1, val2;
	u8 reg, val;
	struct vid_enc_sliced_vbi_service services;

	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "Start of adv7343_setstd..\n");
	output_idx = adv7343_channel_info[ch_id].current_output;

	if (mode_info == NULL) {
		dev_err(adv7343_i2c_dev[ch_id], "NULL pointer.\n");
		return -EINVAL;
	}
	for (i = 0; i < adv7343_configuration[ch_id].output[output_idx].
	     no_of_standard; i++) {
		standard =
		    &adv7343_configuration[ch_id].output[output_idx].
		    standard[i];
		if (strcmp(standard->name, mode_info->name) == 0) {
			break;
		}
	}
	if (i == adv7343_configuration[ch_id].output[output_idx].no_of_standard) {
		dev_err(adv7343_i2c_dev[ch_id], "Invalid id...\n");
		return -EINVAL;
	}

	/* Read Mode Select Register */
	val1 = reg1;

	val2 = *(adv7343_configuration[ch_id].
		 output[output_idx].std_info[i].value);

	if (err < 0) {
		dev_err(adv7343_i2c_dev[ch_id], "Set standard failed\n");
		return err;
	}
	val2 &=
	    adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    standard_val2;
	val2 |=
	    adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    standard_val3;
	err |=
	    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				  client,
				  adv7343_configuration[ch_id].
				  output[output_idx].std_info[i].
				  set_std_register, val2);
	*(adv7343_configuration[ch_id].output[output_idx].std_info[i].
	  value) = val2;

	val1 &= (~((u8) INPUT_MODE_MASK));

	val1 |=
	    adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    outputmode_val1;

	/* Write val1 to Mode select register */

	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_MODE_SELECT_REG, val1);
	if (err < 0) {
		dev_err(adv7343_i2c_dev[ch_id], "Set standard failed\n");
		return err;
	}
	reg1 = val1;

	/* Store the standard in global object of adv7343 */
	adv7343_channel_info[ch_id].mode_info =
	    &adv7343_configuration[ch_id].output[output_idx].standard[i];
	ths7303_setvalue(adv7343_channel_info[ch_id].mode_info);

	reg = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc0_reg;
	val = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc0_val;
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_FSC_REG0,
				     adv7343_configuration[ch_id].
				     output[output_idx].std_info[i].fsc0_val);

	reg = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc1_reg;
	val = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc1_val;
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_FSC_REG1,
				     adv7343_configuration[ch_id].
				     output[output_idx].std_info[i].fsc1_val);

	reg = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc2_reg;
	val = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc2_val;
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_FSC_REG2,
				     adv7343_configuration[ch_id].
				     output[output_idx].std_info[i].fsc2_val);

	reg = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc3_reg;
	val = adv7343_configuration[ch_id].output[output_idx].std_info[i].
	    fsc3_val;
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_FSC_REG3,
				     adv7343_configuration[ch_id].
				     output[output_idx].std_info[i].fsc3_val);
	val1 = reg80;
	if (0 == strcmp(mode_info->name, VID_ENC_STD_NTSC)) {
		val1 &= 0x03;
	} else if (0 == strcmp(mode_info->name, VID_ENC_STD_PAL)) {
		val1 |= 0x04;
	}
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_MODE_REG1, val1);
	reg80 = val1;

	/* disable all VBI sliced vbi services */
	services.service_set = 0;
	err |= adv7343_set_vbi_services(&services, enc);

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_setstd>\n");
	return err;
}

/* Following function is used to get currently selected standard.*/
static int adv7343_getstd(struct vid_enc_mode_info *mode_info,
			  struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "Starting getstd function.\n");

	if (mode_info == NULL) {
		dev_err(adv7343_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}
	/* Read the video standard */
	memcpy(mode_info, adv7343_channel_info[ch_id].mode_info,
	       sizeof(struct vid_enc_mode_info));
	dev_dbg(adv7343_i2c_dev[ch_id], "End of getstd function.\n");
	return err;
}

static int adv7343_get_sliced_cap(struct vid_enc_sliced_vbi_service
				  *service, struct vid_encoder_device *enc)
{
	int ch_id;
	if (NULL == enc || NULL == service) {
		return -EINVAL;
	}
	ch_id = enc->channel_id;

	service->service_set = adv7343_configuration[ch_id].services_set;
	return 0;
}
static int adv7343_enumoutput(int index, char *output,
			      struct vid_encoder_device *enc)
{
	int err = 0, ch_id;
	if (NULL == enc || NULL == output) {
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	if (index >= adv7343_configuration[ch_id].no_of_outputs) {
		return -EINVAL;
	}
	strncpy(output,
		adv7343_configuration[ch_id].output[index].output_name,
		VID_ENC_NAME_MAX_CHARS);
	return err;
}

/* Following function is used to set output format in ADV7343 device. The index
   of the output format is  passed as the argument to this function. */
static int adv7343_setoutput(char *output, struct vid_encoder_device *enc)
{
	int err = 0;
	unsigned char val1, val2;
	int ch_id, i;
	int index;

	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "Start of set output function.\n");

	/* check for null pointer */
	if (output == NULL) {
		dev_err(adv7343_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}
	for (i = 0; i < adv7343_configuration[ch_id].no_of_outputs; i++) {
		if (0 == strcmp(output,
				adv7343_configuration[ch_id].output[i].
				output_name))
			break;
	}
	if (i == adv7343_configuration[ch_id].no_of_outputs)
		return -EINVAL;
	index = i;
	/* Enable Appropriate DAC */
	val1 = reg11;
	val1 &= 0x03;
	val1 |= adv7343_configuration[ch_id].output[index].power_val;
	err = adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				    client, ADV7343_POWER_MODE_REG, val1);
	reg11 = val1;
	/* Enable YUV output mode in Mode Register 0 */

	/* Read Mode register 0 */
	val1 = reg0;

	/* Enable YUV output */
	val1 |= YUV_OUTPUT_SELECT;

	/* write Mode register 0 */
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_MODE_REG0, val1);
	reg0 = val1;

	/* Read SD MODE REGISTER 2 */
	val2 = reg2;

	/* configure SD DAC Output 2 and SD DAC Output 1 bit to zero */
	val2 &= (SD_DAC_1_DI & SD_DAC_2_DI);

	/* write SD MODE REGISTER 2 */
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_MODE_REG2, val2);
	if (err < 0) {
		return err;
	}
	reg2 = val2;

	/* Read HD MODE REGISTER 6 */
	val2 = reg6;

	/* configure ED/HD Color DAC Swap and ED/HD RGB Input Enable bit to
	 * zero */
	val2 &= (HD_RGB_INPUT_DI & HD_DAC_SWAP_DI);

	/* write HD MODE REGISTER 6 */
	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_HD_MODE_REG6, val2);
	if (err < 0) {
		return err;
	}
	reg6 = val2;

	adv7343_channel_info[ch_id].current_output = index;

	/* set default standard */
	adv7343_channel_info[ch_id].mode_info
	    = adv7343_configuration[ch_id].output[index].def_std;
	err |= adv7343_setstd(adv7343_channel_info[ch_id].mode_info, enc);
	if (err < 0) {
		return err;
	}

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_setoutput>\n");
	return err;
}

/* Following function is used to get index of the output currently selected.*/
static int adv7343_getoutput(char *output, struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	int index;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "Start of get output function.\n");
	/* check for null pointer */
	if (output == NULL) {
		dev_err(adv7343_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}
	index = adv7343_channel_info[ch_id].current_output;
	strncpy(output, adv7343_configuration[ch_id].output[index].
		output_name, VID_ENC_NAME_MAX_CHARS);
	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_getoutput>\n");
	return err;
}

/* This function is used to set parameters depending on the type */
static int adv7343_set_params(void *p, struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	adv7343_params params;

	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}

	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_params>\n");
	/* Check for null value */
	if (!(adv7343_params *) p) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_hdparams:NULL pointer\n");
		return -EINVAL;
	}

	if (copy_from_user(&params, (adv7343_params *) p, sizeof(params))) {
		return -EFAULT;
	}

	if (ADV7343_SDPARAMS == params.type) {
		/* If parameter type is SD parameters, call
		 * adv7343_set_sdparams function to set SD parameters */
		err = adv7343_set_sdparams(&(params.params.sd), enc);
	} else if (ADV7343_HDPARAMS == params.type) {
		/* If parameter type is HD parameters, call
		 * adv7343_set_hdparams function to set HD parameters */
		err = adv7343_set_hdparams(&(params.params.hd), enc);
	} else {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_params:invalid type of parameter\n");
		return -EINVAL;
	}
	adv7343_channel_info[ch_id].params = params;
	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_params>\n");
	return err;
}

/* This function is used to get parameters depending on the type */
static int adv7343_get_params(void *p, struct vid_encoder_device *enc)
{
	int err = 0;
	int ch_id;
	adv7343_params *params = (adv7343_params *) p;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}

	ch_id = enc->channel_id;
	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_get_params>\n");
	/* Check for null value */
	if (!params) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_get_params:NULL pointer\n");
		return -EINVAL;
	}

	if (copy_to_user(params, &(adv7343_channel_info[ch_id].params),
			 sizeof(*params))) {
		return -EFAULT;
	}
	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_get_params>\n");
	return err;
}

/* This function is used to set SD parameters */
static int adv7343_set_sdparams(struct adv7343_sd_params *sd, void *enc)
{
	int err = 0;
	unsigned char val1, val2, val3;
	int ch_id;
	struct i2c_client *ch_client;

	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;
	ch_client = &adv7343_channel_info[ch_id].i2c_dev.client;
	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_sdparams>\n");
	/* Check for null value */
	if (!sd) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_sdparams:NULL pointer\n");
		return -EINVAL;
	}
	/* Check for valid value */
	if (!(ADV7343_VALID_FEATURE_VAL(sd->active_step_edge_scale))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_sdparams:invalid value of"
			" active_step_edge_scale\n");
		return -EINVAL;
	}
	if (!(ADV7343_VALID_FEATURE_VAL(sd->scale.en_scale))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_sdparams:invalid value of"
			" scale.en_scale\n");
		return -EINVAL;
	}
	/* Set Luma and chroma filter in SD Mode register 0 */
	val1 = reg80;

	/* Set PbPr color difference filter in SD Mode register 1 */
	val3 = reg2;

	val1 &= (~(SD_LUMA_FLTR_MASK << SD_LUMA_FLTR_SHIFT));
	val1 |= ((sd->luma_filter & SD_LUMA_FLTR_MASK) << SD_LUMA_FLTR_SHIFT);

	if (sd->chroma_filter != ADV7343_CHROMA_SSAF) {
		val1 &= (~(SD_CHROMA_FLTR_MASK << SD_CHROMA_FLTR_SHIFT));
		val1 |= ((sd->chroma_filter & SD_CHROMA_FLTR_MASK) <<
			 SD_CHROMA_FLTR_SHIFT);
		val3 &= SD_PBPR_SSAF_DI;
	} else {
		val3 |= SD_PBPR_SSAF_EN;
	}

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_SD_MODE_REG1, val1);
	err |= adv7343_i2c_write_reg(ch_client, ADV7343_SD_MODE_REG2, val3);
	reg80 = val1;
	reg2 = val3;

	/* Call adv7343_set_ssafparams to set ssaf parameters */
	err = adv7343_set_ssafparams(&sd->ssaf, enc);
	if (err < 0)
		return err;

	/* Call adv7343_set_dnrparams to set dnr parameters */
	err = adv7343_set_dnrparams(&sd->dnr, enc);
	if (err < 0)
		return err;

	/* Set scalling parameters in ADV7343 SD Y,U,V Scale registers and
	 * enable it on SD Mode register 4 */
	val3 = reg3;
	if (sd->scale.en_scale == ADV7343_ENABLE) {
		/* Enable scaling in ADV7343 register */
		val3 |= SD_PBPR_SCALE_EN;
		val3 |= SD_Y_SCALE_EN;

		val1 = 0;

		sd->scale.y_scale &= SD_YPBPR_SCALE_MASK;
		sd->scale.pb_scale &= SD_YPBPR_SCALE_MASK;
		sd->scale.pr_scale &= SD_YPBPR_SCALE_MASK;

		/* Write pr scale value in ADV Register */
		val2 = (u8) ((sd->scale.pr_scale & SD_YPBPR_SCALE_HMASK) >>
			     SD_YPBPR_SCALE_SHIFT);
		val1 = (sd->scale.pr_scale & SD_YPBPR_SCALE_LMASK);
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_SD_CR_SCALE, val2);

		/* Write pb scale value in ADV Register */
		val2 = (u8) ((sd->scale.pb_scale & SD_YPBPR_SCALE_HMASK) >>
			     SD_YPBPR_SCALE_SHIFT);
		val1 <<= SD_YPBPR_SCALE_SHIFT;
		val1 |= (sd->scale.pb_scale & SD_YPBPR_SCALE_LMASK);
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_SD_CB_SCALE, val2);

		/* Write y scale value in ADV Register */
		val2 = (u8) ((sd->scale.y_scale & SD_YPBPR_SCALE_HMASK) >>
			     SD_YPBPR_SCALE_SHIFT);
		val1 <<= SD_YPBPR_SCALE_SHIFT;
		val1 |= sd->scale.y_scale & SD_YPBPR_SCALE_LMASK;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_SD_Y_SCALE, val2);

		val2 = reg4;
		val2 |= val1;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_SD_SCALE_LSB, val2);
		reg4 = val2;
	} else {
		/* Disable scaling in ADV7343 register */
		val3 &= SD_PBPR_SCALE_DI;
		val3 &= SD_Y_SCALE_DI;
	}
	err |= adv7343_i2c_write_reg(ch_client, ADV7343_SD_MODE_REG6, val3);
	reg3 = val3;

	/* Enable active and step edge control in SD Mode register 1 if it is
	   enabled in sd */
	val1 = reg2;
	if (sd->active_step_edge_scale == ADV7343_ENABLE) {
		val1 |= SD_ACTIVE_EDGE_EN;
	} else {
		val1 &= SD_ACTIVE_EDGE_DI;
	}
	err |= adv7343_i2c_write_reg(ch_client, ADV7343_SD_MODE_REG2, val1);
	reg2 = val1;

	/* Call adv7343_set_gammaparams to set gamma correction parameters */
	err = adv7343_set_gammaparams(&sd->gamma, ADV7343_SDPARAMS, enc);

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_sdparams>\n");
	return err;
}

/* This function is used to set Gamma Correction parameters */
static int adv7343_set_gammaparams(struct adv7343_gamma_params *params,
				   enum adv7343_params_type type, void *enc)
{
	int err = 0, i;
	unsigned char val1, reg = 0;
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;

	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_gammaparams>\n");
	/* Check for null value */
	if (!params) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_gammaparams:NULL pointer\n");
		return -EINVAL;
	}
	/* Check for valid value */
	if (!(ADV7343_VALID_FEATURE_VAL(params->en_gamma))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_gammaparams:invalid value"
			" of en_gamma\n");
		return -EINVAL;
	}
	/* Check for invalid curve */
	if ((ADV7343_ENABLE == params->en_gamma)
	    && (!(ADV7343_VALID_GAMMA_CURVE(params->curve)))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_gammaparams:invalid curve type\n");
		return -EINVAL;
	}

	/* Write the gamma coefficient in */
	if (ADV7343_SDPARAMS == type) {
		val1 = reg5;
	} else {
		val1 = reg6;
	}

	/* If gamm enabled, enable gamma correction in SD Mode Register 5
	 * or HD Mode register 6 */
	if (params->en_gamma == ADV7343_ENABLE) {
		if (ADV7343_SDPARAMS == type) {
			val1 |= SD_GAMMA_EN;
			/* Select gamm curve in the same register */
			(params->curve == ADV7343_GAMMA_CURVE_B)
			    ? (val1 |= SD_GAMMA_CURVE_B)
			    : (val1 &= SD_GAMMA_CURVE_A);

			/* Select base register for gamma coefficients as per
			 * the curve selected */
			(params->curve == ADV7343_GAMMA_CURVE_B)
			    ? (reg = ADV7343_SD_GAMMA_B0)
			    : (reg = ADV7343_SD_GAMMA_A0);

		} else {
			val1 |= HD_GAMMA_EN;

			/* Select gamm curve in the same register */
			(params->curve == ADV7343_GAMMA_CURVE_B)
			    ? (val1 |= HD_GAMMA_CURVE_B)
			    : (val1 &= HD_GAMMA_CURVE_A);

			/* Select base register for gamma coefficients as per
			 * the curve selected */
			(params->curve == ADV7343_GAMMA_CURVE_B)
			    ? (reg = ADV7343_HD_GAMMA_B0)
			    : (reg = ADV7343_HD_GAMMA_A0);
		}
		/* Write the gamma coefficient in */
		for (i = reg; i < reg + ADV7343_MAX_GAMMA_COEFFS; i++) {
			err |=
			    adv7343_i2c_write_reg(&adv7343_channel_info
						  [ch_id].i2c_dev.client,
						  i, params->coeff[i - reg]);
		}
	} else {
		/* Disable the gamma curve */
		if (ADV7343_SDPARAMS == type) {
			val1 &= SD_GAMMA_DI;
		} else {
			val1 &= HD_GAMMA_DI;
		}
	}

	/* Write the value to the register */
	if (ADV7343_SDPARAMS == type) {
		err |=
		    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].
					  i2c_dev.client,
					  ADV7343_SD_MODE_REG7, val1);
		reg5 = val1;
	} else {
		err |=
		    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].
					  i2c_dev.client,
					  ADV7343_HD_MODE_REG6, val1);
		reg6 = val1;
	}

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_gammaparams>\n");
	return err;
}

/* This function is used to set HD parameters */
static int adv7343_set_hdparams(struct adv7343_hd_params *hd, void *enc)
{
	int err = 0;
	unsigned char val;
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;

	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_hdparams>\n");
	if (!hd) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_hdparams:NULL pointer\n");
		return -EINVAL;
	}
	/* Check for valid value */
	if (!ADV7343_VALID_FEATURE_VAL(hd->sinc_filter)) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_hdparams:invalid value"
			" of sinc filter\n");
		return -EINVAL;
	}
	if (!ADV7343_VALID_FEATURE_VAL(hd->ssaf_filter)) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_hdparams:invalid value"
			" of ssaf filter\n");
		return -EINVAL;
	}

	/* Set the HD filters in HD Mode register 4 */
	val = reg7;

	if (hd->sinc_filter == ADV7343_ENABLE) {
		val |= HD_SYNC_FLTR_EN;
	} else {
		val &= HD_SYNC_FLTR_DI;
	}

	if (hd->ssaf_filter == ADV7343_ENABLE) {
		val |= HD_CHROMA_SSAF_EN;
	} else {
		val &= HD_CHROMA_SSAF_DI;
	}

	err |=
	    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				  client, ADV7343_HD_MODE_REG4, val);
	reg7 = val;

	/* Call adv7343_set_rgbparams function to set RGB params */
	err = adv7343_set_rgbmatrix(&hd->rgb, enc);
	if (err < 0)
		return err;

	/* Call adv7343_set_filterparams function to set filter params */
	err = adv7343_set_filterparams(&hd->filt_params, enc);
	if (err < 0)
		return err;

	/* Call adv7343_set_gammaparams function to set gamma correction
	   parameters */
	err = adv7343_set_gammaparams(&hd->gamma, ADV7343_HDPARAMS, enc);
	if (err < 0)
		return err;

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_hdparams>\n");
	return err;
}

/* This function is used to set DNR parameters */
static int adv7343_set_dnrparams(struct adv7343_dnr_params *dnr, void *enc)
{
	int err = 0;
	unsigned char val1, val2;
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;

	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_dnrparams>\n");
	/* Check for null value */
	if (!dnr) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_dnrparams:NULL pointer\n");
		return -EINVAL;
	}
	/* Check for valid value */
	if (!(ADV7343_VALID_FEATURE_VAL(dnr->en_dnr))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_dnrparams:invalid value" " of en_dnr\n");
		return -EINVAL;
	}
	/* If DNR is disabled, disable it in ADV registers */

	/* Read SD Mode Register 5 to enable/disable DNR */
	val1 = reg5;

	if (dnr->en_dnr == ADV7343_DISABLE) {

		/* Disable DNR in SD Mode register 5 */
		val1 &= SD_DNR_DI;

		err |=
		    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].
					  i2c_dev.client,
					  ADV7343_SD_MODE_REG7, val1);
		reg5 = val1;
		return err;
	}

	/* Check for invalid values */
	if (dnr->data_gain > SD_DNR_GAIN_MAX
	    || dnr->border_gain > SD_DNR_GAIN_MAX
	    || dnr->threshold > SD_DNR_THRESHOLD_MAX) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_dnrparams:gain is out of bound\n");
		return -EINVAL;
	}
	if (ADV7343_DNR_BLOCK_16 != dnr->block_size &&
	    ADV7343_DNR_BLOCK_8 != dnr->block_size) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_dnrparams:invalid block size\n");
		return -EINVAL;
	}
	if (ADV7343_BORDER_AREA_4 != dnr->area &&
	    ADV7343_BORDER_AREA_4 != dnr->area) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_dnrparams:invalid border area\n");
		return -EINVAL;
	}
	if (ADV7343_MODE_DNR_SHARPNESS != dnr->mode &&
	    ADV7343_MODE_DNR != dnr->mode) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_dnrparams:invalid mode\n");
		return -EINVAL;
	}

	/* Enable DNR in SD Mode register 5 */
	val1 |= SD_DNR_EN;

	err |= adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				     client, ADV7343_SD_MODE_REG7, val1);
	reg5 = val1;

	/* Write value of Data and Border gain in DNR 0 Register */
	val1 = (dnr->data_gain & SD_DNR_DATA_GAIN_MASK);
	val1 <<= SD_DNR_DATA_GAIN_SHIFT;
	val1 |= (dnr->border_gain & SD_DNR_BORDER_GAIN_MASK);

	err |=
	    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				  client, ADV7343_SD_DNR0, val1);

	val1 = dnr->threshold & SD_DNR_THRESHOLD_MASK;
	if (ADV7343_DNR_BLOCK_16 == dnr->block_size) {
		val1 |= SD_BLOCK_SIZE_16x16;
	} else {
		val1 &= SD_BLOCK_SIZE_8x8;
	}

	if (ADV7343_BORDER_AREA_4 == dnr->area) {
		val1 |= SD_BORDER_AREA_4PIXELS;
	} else {
		val1 &= SD_BORDER_AREA_2PIXELS;
	}

	/* Write the value of border size, block size and threshold in DNR 1
	   Register */
	err |=
	    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				  client, ADV7343_SD_DNR1, val1);

	val1 = dnr->filt_select;

	if (ADV7343_MODE_DNR_SHARPNESS == dnr->mode) {
		val1 |= SD_DNR_SHARPNESS_MODE;
	} else {
		val1 &= SD_DNR_MODE;
	}

	val2 = (dnr->block_offset & SD_DNR_BLOCK_OFFSET_MASK);
	val2 <<= SD_DNR_BLOCK_OFFSET_SHIFT;
	val1 |= val2;
	/* Write the value of block offset and DNR mode in DNR 2 register */
	err |=
	    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				  client, ADV7343_SD_DNR2, val1);

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_dnrparams>\n");
	return err;
}

/* This function is used to set SSAF filter parameters */
static int adv7343_set_ssafparams(struct adv7343_ssaf_params *ssaf, void *enc)
{
	int err = 0;
	unsigned char val1;
	int ch_id;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;

	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_ssafparams>\n");

	/* Check for null value */
	if (!ssaf) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_ssafparams:NULL pointer\n");
		return -EINVAL;
	}
	/* Check for valid value */
	if (!(ADV7343_VALID_FEATURE_VAL(ssaf->en_gain))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_ssafparams:invalid value" " of en_gain\n");
		return -EINVAL;
	}

	/* Read SD Mode register 4 */
	val1 = reg3;

	/* If ssaf gain is enabled */
	if (ssaf->en_gain == ADV7343_ENABLE) {
		/* If gain value is out of bound return error */
		if (ssaf->gain > SD_LUMA_SSAF_GAIN_MAX) {
			dev_err(adv7343_i2c_dev[ch_id],
				"adv7343_set_ssafparams:Invalid"
				" value of gain\n");
			return -EINVAL;
		}

		/* Enable SSAF Gain in SD Mode Register 4 */
		val1 |= SD_LUMA_SSAF_GAIN_EN;

		/* Write value of gain in SD LUMA SSAF register */
		err |=
		    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].
					  i2c_dev.client,
					  ADV7343_SD_LUMA_SSAF, ssaf->gain);
	} else {
		/* Disable ssaf gain */
		/* Disable SSAF Gain in SD Mode Register 4 */
		val1 &= SD_LUMA_SSAF_GAIN_DI;
	}
	err |=
	    adv7343_i2c_write_reg(&adv7343_channel_info[ch_id].i2c_dev.
				  client, ADV7343_SD_MODE_REG6, val1);
	reg3 = val1;

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_ssafparams>\n");
	return err;
}

/* This function is used to set filter parameters for HD */
static int adv7343_set_filterparams(struct adv7343_filter_params *filt,
				    void *enc)
{
	int err = 0;
	unsigned char val1;
	int ch_id;
	struct i2c_client *ch_client;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;
	ch_client = &adv7343_channel_info[ch_id].i2c_dev.client;

	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_filterparams>\n");

	/* Check for null value */
	if (!filt) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_filterparams:NULL pointer\n");
		return -EINVAL;
	}

	/* Check for valid value */
	if (!(ADV7343_VALID_FEATURE_VAL(filt->en_filter))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_filterparams:invalid value of "
			"en_filter\n");
		return -EINVAL;
	}
	/* If filter is not enabled, return */
	if (filt->en_filter == ADV7343_DISABLE) {
		/* Disable both sharpness and adaptive filters */
		val1 = reg9;

		val1 &= HD_SHARPNESS_FLTR_DI;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG2, val1);
		reg9 = val1;

		val1 = reg6;

		val1 &= HD_ADPT_FLTR_DI;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG6, val1);
		reg6 = val1;

		dev_dbg(adv7343_i2c_dev[ch_id], "adv7343_set_filterparams L\n");
		return 0;
	}

	/* Switch on the value of mode */
	switch (filt->mode) {
		/* If the case is for mode sharpness */
	case ADV7343_FILTER_MODE_SHARPNESS:
		/* Read HD Mode Register 2 to enable sharpness filter */
		val1 = reg9;
		val1 |= HD_SHARPNESS_FLTR_EN;

		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG2, val1);
		reg9 = val1;

		/* Disable adaptive filter in HD Mode Register 2 */
		val1 = reg6;

		val1 &= HD_ADPT_FLTR_DI;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG6, val1);
		reg6 = val1;

		/* Write the value of sharpness in HD Sharpness filter
		 *  gain register */
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_SHARPNESS_FLTR_GAIN,
					     filt->sharpness);
		break;

		/* If the case is for mode A */
	case ADV7343_FILTER_MODE_A:
		/* enable sharpness filter in HD Mode Register 2 */
		val1 = reg9;

		val1 |= HD_SHARPNESS_FLTR_EN;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG2, val1);
		reg9 = val1;
		/* Enable adaptive filter and mode A in HD Mode
		 * Register 6 */
		val1 = reg6;

		val1 |= HD_ADPT_FLTR_EN;
		val1 &= HD_ADPT_FLTR_MODEA;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG6, val1);
		reg6 = val1;

		/* Write the value of sharpness in HD Sharpness filter
		 *  gain register */
		val1 = (filt->sharpness & HD_SHARPNESS_FLTR_B_MASK)
		    << HD_SHARPNESS_FLTR_B_SHIFT;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_SHARPNESS_FLTR_GAIN,
					     val1);
		filt->sharpness = val1;

		/* Write the value of thresholds in HD thresholds
		 * register */
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_THRLDA,
					     filt->threshold_a);

		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_THRLDB,
					     filt->threshold_b);

		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_THRLDC,
					     filt->threshold_c);
		filt->gain1_valb &= HD_ADPT_FLTR_GAIN_B_MASK;
		filt->gain2_valb &= HD_ADPT_FLTR_GAIN_B_MASK;
		filt->gain3_valb &= HD_ADPT_FLTR_GAIN_B_MASK;

		/* Write value of filter gains in HD Gains registers */
		val1 = (filt->gain1_valb & HD_ADPT_FLTR_GAIN_B_MASK);
		val1 <<= HD_ADPT_FLTR_GAIN_B_SHIFT;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_GAIN1, val1);

		val1 = (filt->gain2_valb & HD_ADPT_FLTR_GAIN_B_MASK);
		val1 <<= HD_ADPT_FLTR_GAIN_B_SHIFT;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_GAIN2, val1);

		val1 = (filt->gain3_valb & HD_ADPT_FLTR_GAIN_B_MASK);
		val1 <<= HD_ADPT_FLTR_GAIN_B_SHIFT;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_GAIN3, val1);

		break;

		/* If the case is for mode B */
	case ADV7343_FILTER_MODE_B:
		/* enable sharpness filter in HD Mode Register 2 */
		val1 = reg9;

		val1 |= HD_SHARPNESS_FLTR_EN;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG2, val1);
		reg9 = val1;

		/* Enable adaptive filter and mode B in HD Mode
		 * Register 6 */
		val1 = reg6;
		val1 |= HD_ADPT_FLTR_EN;
		val1 |= HD_ADPT_FLTR_MODEB;
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_MODE_REG6, val1);
		reg6 = val1;

		/* Write the value of sharpness in HD Sharpness filter
		 * gain register */
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_SHARPNESS_FLTR_GAIN,
					     filt->sharpness);
		filt->gain1_valb &= HD_ADPT_FLTR_GAIN_B_MASK;
		filt->gain2_valb &= HD_ADPT_FLTR_GAIN_B_MASK;
		filt->gain3_valb &= HD_ADPT_FLTR_GAIN_B_MASK;
		filt->gain1_vala &= HD_ADPT_FLTR_GAIN_A_MASK;
		filt->gain2_vala &= HD_ADPT_FLTR_GAIN_A_MASK;
		filt->gain3_vala &= HD_ADPT_FLTR_GAIN_A_MASK;

		/* Write the value of thresholds in HD thresholds
		 * register */
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_THRLDA,
					     filt->threshold_a);

		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_THRLDB,
					     filt->threshold_b);

		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_THRLDC,
					     filt->threshold_c);

		/* Write value of filter gains in HD Gains registers */
		val1 = (filt->gain1_valb & HD_ADPT_FLTR_GAIN_B_MASK);
		val1 <<= HD_ADPT_FLTR_GAIN_B_SHIFT;
		val1 |= (filt->gain1_vala & HD_ADPT_FLTR_GAIN_A_MASK);
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_GAIN1, val1);

		val1 = (filt->gain2_valb & HD_ADPT_FLTR_GAIN_B_MASK);
		val1 <<= HD_ADPT_FLTR_GAIN_B_SHIFT;
		val1 |= (filt->gain2_vala & HD_ADPT_FLTR_GAIN_A_MASK);
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_GAIN2, val1);

		val1 = (filt->gain3_valb & HD_ADPT_FLTR_GAIN_B_MASK);
		val1 <<= HD_ADPT_FLTR_GAIN_B_SHIFT;
		val1 |= (filt->gain3_vala & HD_ADPT_FLTR_GAIN_A_MASK);
		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_HD_ADPT_FLTR_GAIN3, val1);
		break;

	default:
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_filterparams:invalid Mode\n");
	}
	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_filterparams>\n");
	return err;
}

/* This function is used to set RGB matrix coeffiecients */
static int adv7343_set_rgbmatrix(struct adv7343_rgb_matrix *rgb, void *enc)
{
	int err = 0;
	unsigned char val1, val2;
	int ch_id;
	struct i2c_client *ch_client;
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct vid_encoder_device *)enc)->channel_id;
	ch_client = &adv7343_channel_info[ch_id].i2c_dev.client;

	dev_dbg(adv7343_i2c_dev[ch_id], "<adv7343_set_rgbmatrix>\n");

	/* Check for null value */
	if (!rgb) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_rgbmatrix:NULL pointer\n");
		return -EINVAL;
	}

	/* Check for valid value */
	if (!(ADV7343_VALID_FEATURE_VAL(rgb->en_rgb_matrix))) {
		dev_err(adv7343_i2c_dev[ch_id],
			"adv7343_set_rgbmatrix:invalid value of "
			"en_rgb_matrix\n");
		return -EINVAL;
	}
	/* Disable RGB matrix in Mode Register 0 */
	val1 = reg0;

	/* If rgb matrix is not enabled, return */
	if (rgb->en_rgb_matrix == ADV7343_DISABLE) {
		val1 &= CSC_MATRIX_DI;

		err |= adv7343_i2c_write_reg(ch_client,
					     ADV7343_MODE_REG0, val1);
		reg0 = val1;
		dev_dbg(adv7343_i2c_dev[ch_id], "adv7343_set_rgbmatrix L\n");
		return err;
	}

	/* Enable RGB matrix in Mode Register 0 */
	val1 |= CSC_MATRIX_EN;

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_MODE_REG0, val1);
	reg0 = val1;

	rgb->gy &= CSC_MATRIX_MASK;
	rgb->gu &= CSC_MATRIX_MASK;
	rgb->gv &= CSC_MATRIX_MASK;
	rgb->bu &= CSC_MATRIX_MASK;
	rgb->rv &= CSC_MATRIX_MASK;

	/* Write the value GY in RGB matrix 2 register */
	val1 = (rgb->gy & CSC_LSB_MASK);
	val2 = (u8) (rgb->gy >> CSC_SHIFT);

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX0, val1);
	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX2, val2);

	/* Write the value GU in RGB matrix 3 register */
	val1 = (rgb->gu & CSC_LSB_MASK);
	val2 = (rgb->gu >> CSC_SHIFT);

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX3, val2);

	/* Write the value GR in RGB matrix 4 register */
	val1 <<= CSC_SHIFT;
	val1 |= (rgb->gv & CSC_LSB_MASK);
	val2 = (rgb->gv >> CSC_SHIFT);

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX4, val2);

	/* Write the value BU in RGB matrix 5 register */
	val1 <<= CSC_SHIFT;
	val1 |= (rgb->bu & CSC_LSB_MASK);
	val2 = rgb->bu >> CSC_SHIFT;

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX5, val2);

	/* Write the value RV in RGB matrix 6 register */
	val1 <<= CSC_SHIFT;
	val1 |= (rgb->rv & CSC_LSB_MASK);
	val2 = rgb->rv >> CSC_SHIFT;

	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX6, val2);
	err |= adv7343_i2c_write_reg(ch_client, ADV7343_CSC_MATRIX1, val1);

	dev_dbg(adv7343_i2c_dev[ch_id], "</adv7343_set_rgbmatrix>\n");
	return err;
}

/*This function is used to write value into register using i2c client. */
static int adv7343_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter) {
		err = -ENODEV;
	} else {
		dev_dbg(adv7343_i2c_dev[0], "adv7343 i2c WRITE start \n");
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;
		data[0] = reg;
		data[1] = val;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	dev_dbg(adv7343_i2c_dev[0], "adv7343 i2c WRITE end \n");

	return ((err < 0) ? err : 0);
}

/* This function is used to attach i2c client */
static int adv7343_i2c_attach_client(struct i2c_client *client,
				     struct i2c_driver *driver,
				     struct i2c_adapter *adap, int addr)
{
	int err = 0;

	if (client->adapter) {
		err = -EBUSY;	/* our client is already attached */
	} else {
		client->addr = addr;
		client->flags = I2C_CLIENT_ALLOW_USE;
		client->driver = driver;
		client->adapter = adap;

		err = i2c_attach_client(client);
		if (err) {
			client->adapter = NULL;
		}
	}
	return err;
}

/* This function is used to detach i2c client */
static int adv7343_i2c_detach_client(struct i2c_client *client)
{
	int err = 0;
	if (!client->adapter) {
		return -ENODEV;	/* our client isn't attached */
	} else {
		err = i2c_detach_client(client);
		client->adapter = NULL;
	}
	return err;
}

static int adv7343_i2c_probe_adapter(struct i2c_adapter *adap)
{
	int err = 0;
	adv7343_i2c_dev[0] = &(adap->dev);
	dev_dbg(adv7343_i2c_dev[0], "ADV7343 i2c probe adapter called...\n");
	/* Attach the client */
	err = adv7343_i2c_attach_client(&adv7343_channel_info[0].i2c_dev.
					client,
					&adv7343_channel_info[0].i2c_dev.
					driver, adap,
					adv7343_channel_info[0].i2c_dev.
					i2c_addr);
	dev_dbg(adv7343_i2c_dev[0], "ADV7343 i2c probe adapter ends...\n");
	return err;

}

/* This function used to initialize the i2c driver */
static int adv7343_i2c_init(void)
{
	int err = 0;
	int i = 0, j;
	/* Take instance of driver */
	struct i2c_driver *driver;

	char strings[ADV7343_NUM_CHANNELS][80] = {
		"ADV7343 encoder I2C driver"
	};

	for (i = 0; i < ADV7343_NUM_CHANNELS; i++) {
		driver = &adv7343_channel_info[i].i2c_dev.driver;
		driver->owner = THIS_MODULE;
		strlcpy(driver->name, strings[i], sizeof(strings[i]));
		driver->id = I2C_DRIVERID_EXP0;
		driver->flags = I2C_DF_NOTIFY;
		if (0 == i) {
			driver->attach_adapter = adv7343_i2c_probe_adapter;
		}

		driver->detach_client = adv7343_i2c_detach_client;
		err |= vid_enc_register_encoder(&adv7343_dev[i]);
		if (err < 0) {
			for (j = i - 1; j > 0; j--) {
				vid_enc_unregister_encoder(&adv7343_dev[j]);
			}
			return err;
		}
	}
	return err;
}

/* Function used to cleanup i2c driver */
static void adv7343_i2c_cleanup(void)
{
	int i;
	for (i = 0; i < ADV7343_NUM_CHANNELS; i++) {
		if (vid_enc_unregister_encoder(&adv7343_dev[i]) < 0)
			return;
		if (adv7343_channel_info[i].i2c_dev.i2c_registration & 0x01) {
			i2c_del_driver(&adv7343_channel_info[i].i2c_dev.driver);
			adv7343_channel_info[i].i2c_dev.client.adapter = NULL;
			adv7343_channel_info[i].i2c_dev.i2c_registration = 0;
		}
	}
}

module_init(adv7343_i2c_init);
module_exit(adv7343_i2c_cleanup);

MODULE_LICENSE("GPL");
