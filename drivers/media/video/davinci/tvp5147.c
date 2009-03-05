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
/* tvp5147.c */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <media/davinci/tvp5147.h>
#include <asm/arch/video_hdevm.h>

/* Prototypes */
static int tvp5147_initialize(void *dec, int flag);
static int tvp5147_deinitialize(void *dec);
static int tvp5147_setcontrol(struct v4l2_control *ctrl, void *dec);
static int tvp5147_getcontrol(struct v4l2_control *ctrl, void *dec);
static int tvp5147_querycontrol(struct v4l2_queryctrl *queryctrl, void *dec);
static int tvp5147_setstd(v4l2_std_id * id, void *dec);
static int tvp5147_getstd(v4l2_std_id * id, void *dec);
static int tvp5147_enumstd(struct v4l2_standard *std, void *dec);
static int tvp5147_querystd(v4l2_std_id * id, void *dec);
static int tvp5147_setinput(int *index, void *dec);
static int tvp5147_getinput(int *index, void *dec);
static int tvp5147_enuminput(struct v4l2_input *input, void *dec);
static int tvp5147_setformat(struct v4l2_format *fmt, void *dec);
static int tvp5147_tryformat(struct v4l2_format *fmt, void *dec);
static int tvp5147_getformat(struct v4l2_format *fmt, void *dec);
static int tvp5147_setparams(void *params, void *dec);
static int tvp5147_getparams(void *params, void *dec);
static int tvp5147_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val);
static int tvp5147_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);
static int tvp5147_i2c_attach_client(struct i2c_client *client,
				     struct i2c_driver *driver,
				     struct i2c_adapter *adap, int addr);
static int tvp5147_i2c_detach_client(struct i2c_client *client);
static int tvp5147A_i2c_probe_adapter(struct i2c_adapter *adap);
static int tvp5147B_i2c_probe_adapter(struct i2c_adapter *adap);
static int tvp5147_i2c_init(void);
static void tvp5147_i2c_cleanup(void);
static int tvp5147_get_sliced_vbi_cap(struct v4l2_sliced_vbi_cap *cap,
				      void *dec);
static int tvp5147_read_vbi_data(struct v4l2_sliced_vbi_data *data, void *dec);

static struct v4l2_standard tvp5147_standards[TVP5147_MAX_NO_STANDARDS] = {
	{
	 .index = 0,
	 .id = V4L2_STD_525_60,
	 .name = "NTSC",
	 .frameperiod = {1001, 30000},
	 .framelines = 525},
	{
	 .index = 1,
	 .id = V4L2_STD_625_50,
	 .name = "PAL",
	 .frameperiod = {1, 25},
	 .framelines = 625},
	{
	 .index = 2,
	 .id = VPFE_STD_AUTO,
	 .name = "auto detect",
	 .frameperiod = {1, 1},
	 .framelines = 1},
	{
	 .index = 3,
	 .id = VPFE_STD_525_60_SQP,
	 .name = "NTSC-SQP",
	 .frameperiod = {1001, 30000},
	 .framelines = 525},
	{
	 .index = 4,
	 .id = VPFE_STD_625_50_SQP,
	 .name = "PAL-SQP",
	 .frameperiod = {1, 25},
	 .framelines = 625},
	{
	 .index = 5,
	 .id = VPFE_STD_AUTO_SQP,
	 .name = "auto detect sqp pixel",
	 .frameperiod = {1, 1},
	 .framelines = 1}
};

static tvp5147_mode
    tvp5147_modes[TVP5147_MAX_NO_STANDARDS][TVP5147_MAX_NO_MODES] = {
	{TVP5147_MODE_NTSC, TVP5147_MODE_NTSC_443, 0xFF},
	{TVP5147_MODE_PAL, TVP5147_MODE_PAL_M, TVP5147_MODE_PAL_CN},
	{TVP5147_MODE_AUTO, 0xFF, 0xFF},
	{TVP5147_MODE_NTSC_SQP, TVP5147_MODE_NTSC_443_SQP, 0xFF},
	{TVP5147_MODE_PAL_SQP, TVP5147_MODE_PAL_M_SQP,
	 TVP5147_MODE_PAL_CN_SQP},
	{TVP5147_MODE_AUTO_SQP, 0xFF, 0xFF}
};

static struct tvp5147_control_info
    tvp5147_control_information[TVP5147_MAX_NO_CONTROLS]
    = {
	{
	 .register_address = TVP5147_BRIGHTNESS,
	 .query_control = {
			   .id = V4L2_CID_BRIGHTNESS,
			   .name = "BRIGHTNESS",
			   .type = V4L2_CTRL_TYPE_INTEGER,
			   .minimum = 0,
			   .maximum = 255,
			   .step = 1,
			   .default_value = 128}
	 },
	{
	 .register_address = TVP5147_CONTRAST,
	 .query_control = {
			   .id = V4L2_CID_CONTRAST,
			   .name = "CONTRAST",
			   .type = V4L2_CTRL_TYPE_INTEGER,
			   .minimum = 0,
			   .maximum = 255,
			   .step = 1,
			   .default_value = 128}

	 },
	{
	 .register_address = TVP5147_SATURATION,
	 .query_control = {
			   .id = V4L2_CID_SATURATION,
			   .name = "SATURATION",
			   .type = V4L2_CTRL_TYPE_INTEGER,
			   .minimum = 0,
			   .maximum = 255,
			   .step = 1,
			   .default_value = 128}
	 },
	{
	 .register_address = TVP5147_HUE,
	 .query_control = {
			   .id = V4L2_CID_HUE,
			   .name = "HUE",
			   .type = V4L2_CTRL_TYPE_INTEGER,
			   .minimum = -128,
			   .maximum = 127,
			   .step = 1,
			   .default_value = 0}
	 },
	{
	 .register_address = TVP5147_AFE_GAIN_CTRL,
	 .query_control = {
			   .id = V4L2_CID_AUTOGAIN,
			   .name = "Automatic Gain Control",
			   .type = V4L2_CTRL_TYPE_BOOLEAN,
			   .minimum = 0,
			   .maximum = 1,
			   .step = 1,
			   .default_value = 1}
	 }
};

static struct tvp5147_config tvp5147_configuration[TVP5147_NUM_CHANNELS] = {
	{
	 .no_of_inputs = TVP5147_MAX_NO_INPUTS,
	 .input[0] = {
		      .input_type = TVP5147_COMPOSITE_INPUT,
		      .lock_mask = 0x0E,
		      .input_info = {
				     .index = 0,
				     .name = "COMPOSITE",
				     .type = V4L2_INPUT_TYPE_CAMERA,
				     .std = V4L2_STD_TVP5147_ALL},
		      .no_of_standard = TVP5147_MAX_NO_STANDARDS,
		      .standard = (struct v4l2_standard *)tvp5147_standards,
		      .def_std = VPFE_STD_AUTO,
		      .mode = (tvp5147_mode(*)[]) & tvp5147_modes,
		      .no_of_controls = TVP5147_MAX_NO_CONTROLS,
		      .controls = (struct tvp5147_control_info *)
		      &tvp5147_control_information},
	 .sliced_cap = {
			.service_set = (V4L2_SLICED_CAPTION_525 |
					V4L2_SLICED_WSS_625 |
					V4L2_SLICED_CGMS_525),
			},
	 .num_services = 0},
	{
	 .no_of_inputs = TVP5147_MAX_NO_INPUTS,
	 .input[0] = {
		      .input_type = TVP5147_SVIDEO_INPUT,
		      .lock_mask = 0x06,
		      .input_info = {
				     .index = 0,
				     .name = "SVIDEO",
				     .type = V4L2_INPUT_TYPE_CAMERA,
				     .std = V4L2_STD_TVP5147_ALL},
		      .no_of_standard = TVP5147_MAX_NO_STANDARDS,
		      .standard = (struct v4l2_standard *)tvp5147_standards,
		      .def_std = VPFE_STD_AUTO,
		      .mode = (tvp5147_mode(*)[]) & tvp5147_modes,
		      .no_of_controls = TVP5147_MAX_NO_CONTROLS,
		      .controls = (struct tvp5147_control_info *)
		      &tvp5147_control_information},
	 .sliced_cap = {.service_set = (V4L2_SLICED_CAPTION_525 |
					V4L2_SLICED_WSS_625 |
					V4L2_SLICED_CGMS_525),
			},
	 .num_services = 0}
};

static struct tvp5147_service_data_reg tvp5147_services_regs
    [TVP5147_VBI_NUM_SERVICES] = {
	{
	 .service = V4L2_SLICED_WSS_625,
	 .field[0].addr = {0x20, 0x05, 0x80},
	 .field[1].addr = {0x24, 0x05, 0x80},
	 .bytestoread = 2},
	{
	 .service = V4L2_SLICED_CAPTION_525,
	 .field[0].addr = {0x1C, 0x05, 0x80},
	 .field[1].addr = {0x1E, 0x05, 0x80},
	 .bytestoread = 2},
	{
	 .service = V4L2_SLICED_CGMS_525,
	 .field[0].addr = {0x20, 0x05, 0x80},
	 .field[1].addr = {0x24, 0x05, 0x80},
	 .bytestoread = 3},
};

static struct tvp5147_sliced_reg tvp5147_sliced_regs[TVP5147_VBI_NUM_SERVICES] = {
	{
	 .service = V4L2_SLICED_CAPTION_525,
	 .std = V4L2_STD_525_60,
	 .line_addr_value = 0x15,
	 .line_start = 19,
	 .line_end = 23,
	 .field[0] = {
		      .fifo_line_addr = {0x00, 0x06, 0x80},
		      .fifo_mode_value = 0x01},
	 .field[1] = {
		      .fifo_line_addr = {0x02, 0x06, 0x80},
		      .fifo_mode_value = 0x09},
	 .service_line = {{21, 21}, {21, 284} }
	 },
	{
	 .service = V4L2_SLICED_WSS_625,
	 .std = V4L2_STD_625_50,
	 .line_addr_value = 0x17,
	 .line_start = 21,
	 .line_end = 25,
	 .field[0] = {
		      .fifo_line_addr = {0x04, 0x06, 0x80},
		      .fifo_mode_value = 0x02},
	 .field[1] = {
		      .fifo_line_addr = {0x04, 0x06, 0x80},
		      .fifo_mode_value = 0x02},
	 .service_line = {{23, 23}, {0, 0} }
	 },
	{
	 .service = V4L2_SLICED_CGMS_525,
	 .std = V4L2_STD_525_60,
	 .line_addr_value = 0x14,
	 .line_start = 18,
	 .line_end = 22,
	 .field[0] = {
		      .fifo_line_addr = {0x08, 0x06, 0x80},
		      .fifo_mode_value = 0x02},
	 .field[1] = {
		      .fifo_line_addr = {0x06, 0x06, 0x80},
		      .fifo_mode_value = 0x02},
	 .service_line = {{20, 20}, {20, 283} }
	 }
};

static struct tvp5147_channel tvp5147_channel_info[TVP5147_NUM_CHANNELS] = {
	{
	 .params.inputidx = 0,
	 .params.std = VPFE_STD_AUTO,
	 .i2c_dev = {
		     .i2c_addr = (0xBA >> 1),
		     .i2c_registration = 0},
	 .dec_device = NULL},
	{
	 .params.inputidx = 0,
	 .params.std = VPFE_STD_AUTO,
	 .i2c_dev = {
		     .i2c_addr = (0xB8 >> 1),
		     .i2c_registration = 0},
	 .dec_device = NULL}
};

/* Global variables */
static struct param_ops params_ops = {
	.setparams = tvp5147_setparams,
	.getparams = tvp5147_getparams
};
static struct control_ops controls_ops = {
	.count = TVP5147_MAX_NO_CONTROLS,
	.queryctrl = tvp5147_querycontrol,
	.setcontrol = tvp5147_setcontrol,
	.getcontrol = tvp5147_getcontrol
};

static struct input_ops inputs_ops = {
	.count = TVP5147_MAX_NO_INPUTS,
	.enuminput = tvp5147_enuminput,
	.setinput = tvp5147_setinput,
	.getinput = tvp5147_getinput
};
static struct standard_ops standards_ops = {
	.count = TVP5147_MAX_NO_STANDARDS,
	.setstd = tvp5147_setstd,
	.getstd = tvp5147_getstd,
	.enumstd = tvp5147_enumstd,
	.querystd = tvp5147_querystd,
};
static struct format_ops formats_ops = {
	.count = 0,
	.enumformat = NULL,
	.setformat = tvp5147_setformat,
	.getformat = tvp5147_getformat,
	.tryformat = tvp5147_tryformat,
};

static struct device *tvp5147_i2c_dev[TVP5147_NUM_CHANNELS];
static struct decoder_device dec_dev[TVP5147_NUM_CHANNELS] = {
	{
	 .name = "TVP5147",
	 .if_type = INTERFACE_TYPE_BT656,
	 .channel_id = 0,
	 .capabilities = V4L2_CAP_SLICED_VBI_CAPTURE | V4L2_CAP_VBI_CAPTURE,
	 .initialize = tvp5147_initialize,
	 .std_ops = &standards_ops,
	 .ctrl_ops = &controls_ops,
	 .input_ops = &inputs_ops,
	 .fmt_ops = &formats_ops,
	 .params_ops = &params_ops,
	 .deinitialize = tvp5147_deinitialize,
	 .get_sliced_vbi_cap = tvp5147_get_sliced_vbi_cap,
	 .read_vbi_data = tvp5147_read_vbi_data},
	{
	 .name = "TVP5147",
	 .if_type = INTERFACE_TYPE_BT656,
	 .channel_id = 1,
	 .capabilities = V4L2_CAP_SLICED_VBI_CAPTURE | V4L2_CAP_VBI_CAPTURE,
	 .initialize = tvp5147_initialize,
	 .std_ops = &standards_ops,
	 .ctrl_ops = &controls_ops,
	 .input_ops = &inputs_ops,
	 .fmt_ops = &formats_ops,
	 .params_ops = &params_ops,
	 .deinitialize = tvp5147_deinitialize,
	 .get_sliced_vbi_cap = tvp5147_get_sliced_vbi_cap,
	 .read_vbi_data = tvp5147_read_vbi_data}
};

static u8 tvp5147_after_reset_reg[][2] = {
	{0xE8, 0x02},
	{0xE9, 0x00},
	{0xEA, 0x80},
	{0xE0, 0x01},
	{0xE8, 0x60},
	{0xE9, 0x00},
	{0xEA, 0xB0},
	{0xE0, 0x01},
	{0xE8, 0x16},
	{0xE9, 0x00},
	{0xEA, 0xA0},
	{0xE0, 0x16},
	{0xE8, 0x60},
	{0xE9, 0x00},
	{0xEA, 0xB0},
	{0xE0, 0x00}
};

/* tvp5147_init :
 * This function called by vpif driver to initialize the decoder with default
 * values
 */
static int tvp5147_initialize(void *dec, int flag)
{
	int err = 0, i;
	int ch_id;
	int index;
	struct i2c_client *i2c_client;
	v4l2_std_id std;
	if (NULL == dec) {
		printk("dec:NULL pointer");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	if (tvp5147_channel_info[ch_id].i2c_dev.i2c_registration & 0x01) {
		printk("tvp5147 driver is already initialized..\n");
		err = -EINVAL;
		return err;
	}
	i2c_client = &tvp5147_channel_info[ch_id].i2c_dev.client;

	/* Register tvp5147 I2C client */
	err = i2c_add_driver(&tvp5147_channel_info[ch_id].i2c_dev.driver);
	if (err) {
		printk("Failed to register TVP5147 I2C client.\n");
		return -EINVAL;
	}
	try_module_get(THIS_MODULE);
	tvp5147_channel_info[ch_id].i2c_dev.i2c_registration |= 1;
	tvp5147_channel_info[ch_id].dec_device = (struct decoder_device *)dec;
	if (DECODER_I2C_BIND_FLAG == flag) {
		/* check that decoder is set with default values once or not,
		 * if yes return, if not continue */
		if (tvp5147_channel_info[ch_id].i2c_dev.i2c_registration & 0x02)
			return err;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	dev_dbg(tvp5147_i2c_dev[ch_id],
		"Starting default settings tvp5147..\n");

	/* Reset TVP5147 */
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OPERATION_MODE,
				     TVP5147_OPERATION_MODE_RESET);

	/*Put _tvp5147 in normal power mode */
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OPERATION_MODE,
				     TVP5147_OPERATION_MODE_DEFAULT);

	for (i = 0; i < 16; i++) {
		err |= tvp5147_i2c_write_reg(i2c_client,
					     tvp5147_after_reset_reg[i][0],
					     tvp5147_after_reset_reg[i]
					     [1]);
	}
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_AFE_GAIN_CTRL,
				     TVP5147_AFE_GAIN_CTRL_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_COLOR_KILLER,
				     TVP5147_COLOR_KILLER_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_LUMA_CONTROL1,
				     TVP5147_LUMA_CONTROL1_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_LUMA_CONTROL2,
				     TVP5147_LUMA_CONTROL2_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_LUMA_CONTROL3,
				     TVP5147_LUMA_CONTROL3_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_BRIGHTNESS,
				     TVP5147_BRIGHTNESS_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_CONTRAST,
				     TVP5147_CONTRAST_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_SATURATION,
				     TVP5147_SATURATION_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_HUE,
				     TVP5147_HUE_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_CHROMA_CONTROL1,
				     TVP5147_CHROMA_CONTROL1_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_CHROMA_CONTROL2,
				     TVP5147_CHROMA_CONTROL2_DEFAULT);

	/* Configuration for 8-bit BT656 mode */
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OUTPUT5,
				     TVP5147_OUTPUT5_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OUTPUT6,
				     TVP5147_OUTPUT6_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OUTPUT1,
				     TVP5147_OUTPUT1_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OUTPUT2,
				     TVP5147_OUTPUT2_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OUTPUT3,
				     TVP5147_OUTPUT3_DEFAULT);
	err |= tvp5147_i2c_write_reg(i2c_client, TVP5147_OUTPUT4,
				     TVP5147_OUTPUT4_DEFAULT);

	/* Call setinput for setting default input */
	index = 0;
	err |= tvp5147_setinput(&index, dec);
	if (err < 0) {
		err = -EINVAL;
		tvp5147_deinitialize(dec);
		return err;
	}

	/* call set standard to set default standard */
	std = tvp5147_configuration[ch_id].input[index].def_std;
	err |= tvp5147_setstd(&std, dec);
	err = tvp5147_querystd(&std, dec);
	if (err < 0) {
		err = -EINVAL;
		tvp5147_deinitialize(dec);
		return err;
	}
	tvp5147_channel_info[ch_id].i2c_dev.i2c_registration |= 0x2;
	return err;
}
static int tvp5147_deinitialize(void *dec)
{
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Tvp5147 ch deinitialization"
		" called\n");
	if (tvp5147_channel_info[ch_id].i2c_dev.i2c_registration & 0x01) {
		i2c_del_driver(&tvp5147_channel_info[ch_id].i2c_dev.driver);
		module_put(THIS_MODULE);
		tvp5147_channel_info[ch_id].i2c_dev.client.adapter = NULL;
		tvp5147_channel_info[ch_id].i2c_dev.i2c_registration &= ~(0x01);
		tvp5147_channel_info[ch_id].dec_device = NULL;
	}
	return 0;
}

/* tvp5147_setcontrol :
 * Function to set the control parameters
 */
static int tvp5147_setcontrol(struct v4l2_control *ctrl, void *dec)
{
	int err = 0;
	int value;
	int ch_id;
	int i = 0;
	int input_idx;
	struct tvp5147_control_info *control = NULL;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	input_idx = tvp5147_channel_info[ch_id].params.inputidx;

	/* check for null pointer */
	if (ctrl == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL pointer\n");
		return -EINVAL;
	}
	dev_dbg(tvp5147_i2c_dev[ch_id], "Starting setctrl...\n");
	value = (__s32) ctrl->value;
	for (i = 0; i < tvp5147_configuration[ch_id].input[input_idx].
	     no_of_controls; i++) {
		control = &tvp5147_configuration[ch_id].input[input_idx].
		    controls[i];
		if ((control->query_control).id == ctrl->id) {
			break;
		}
	}
	if (i == tvp5147_configuration[ch_id].input[input_idx].no_of_controls)
		return -EINVAL;

	if (V4L2_CID_AUTOGAIN == ctrl->id) {
		if (value == 1) {
			value = 0xF;
		} else if (value == 0) {
			value = 0xC;
		} else {
			return -EINVAL;
		}
	} else {
		if (((control->query_control).minimum > value)
		    || ((control->query_control).maximum < value)) {
			return -EINVAL;
		}
	}
	err = tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, control->register_address, value);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id],
			"TVP5147 set control fails...\n");
		return err;
	}
	dev_dbg(tvp5147_i2c_dev[ch_id], "End of setcontrol...\n");
	return err;
}

/* tvp5147_getcontrol :
 * Function to get the control parameters
 */
static int tvp5147_getcontrol(struct v4l2_control *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	int i = 0;
	struct tvp5147_control_info *control = NULL;
	int input_idx;
	u8 val;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();

	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Starting getctrl of TVP5147...\n");
	input_idx = tvp5147_channel_info[ch_id].params.inputidx;

	/* check for null pointer */
	if (ctrl == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL pointer\n");
		return -EINVAL;
	}
	for (i = 0; i < tvp5147_configuration[ch_id].input[input_idx].
	     no_of_controls; i++) {
		control = &tvp5147_configuration[ch_id].input[input_idx].
		    controls[i];
		if ((control->query_control).id == ctrl->id) {
			break;
		}
	}
	if (i == tvp5147_configuration[ch_id].input[input_idx].no_of_controls) {
		dev_err(tvp5147_i2c_dev[ch_id], "Invalid id...\n");
		return -EINVAL;
	}
	err = tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				   client, control->register_address, &val);
	ctrl->value = (int)val;
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id],
			"TVP5147 get control fails...\n");
		return err;
	}
	if (V4L2_CID_AUTOGAIN == ctrl->id) {
		if ((ctrl->value & 0x3) == 3) {
			ctrl->value = 1;
		} else {
			ctrl->value = 0;
		}
	}
	dev_dbg(tvp5147_i2c_dev[ch_id], "End of tvp5147_getcontrol...\n");
	return err;
}

/* This function is used to write the vbi data to the decoder device */
static int tvp5147_read_vbi_data(struct v4l2_sliced_vbi_data *data, void *dec)
{
	int err = 0;
	int ch_id;
	int i = 0, j, k;
	unsigned char value;
	u8 num_services;

	if (NULL == dec) {
		printk("tvp5147_write_vbi_data:NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "tvp5147_write_vbi_data:"
		"Start of tvp5147_write_vbi_data..\n");
	if (data == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_write_vbi_data:"
			"NULL pointer.\n");
		return -EINVAL;
	}
	num_services = tvp5147_configuration[ch_id].num_services;
	for (i = 0; i < num_services; i++) {
		if (0 == data[i].id)
			continue;
		if ((data[i].id | tvp5147_channel_info[ch_id].
		     params.fmt.fmt.sliced.service_set) !=
		    tvp5147_channel_info[ch_id].
		    params.fmt.fmt.sliced.service_set) {
			return -EINVAL;
		}

		for (j = 0; j < TVP5147_VBI_NUM_SERVICES; j++) {
			if (!(tvp5147_services_regs[j].service & data[i].id))
				continue;
			for (k = 0; k < 3; k++)
				tvp5147_i2c_write_reg(&tvp5147_channel_info
						      [ch_id].i2c_dev.client,
						      TVP5147_VBUS_ADDRESS_ACCESS0
						      + k,
						      tvp5147_services_regs[j].
						      field[data[i].field].
						      addr[k]);

			for (k = 0; k <
			     tvp5147_services_regs[j].bytestoread; k++) {
				tvp5147_i2c_read_reg(&tvp5147_channel_info
						     [ch_id].i2c_dev.client,
						     TVP5147_VBUS_DATA_ACCESS_AUTO_INCR,
						     &value);
				data[i].data[k] = value;
			}
		}
	}
	dev_dbg(tvp5147_i2c_dev[ch_id], "</tvp5147_write_vbi_data>\n");
	return err;
}

/* This function is used to get the sliced vbi services supported 
   by the decoder device */
static int tvp5147_get_sliced_vbi_cap(struct v4l2_sliced_vbi_cap *cap,
				      void *dec)
{
	int ch_id;

	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Start of "
		"tvp5147_get_sliced_vbi_cap\n");
	if (cap == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id],
			"tvp5147_get_sliced_vbi_cap:" "NULL pointer\n");
		return -EINVAL;
	}
	*cap = tvp5147_configuration[ch_id].sliced_cap;
	return 0;
}

/* tvp5147_querycontrol :
 * Function to query control parameters
 */
static int tvp5147_querycontrol(struct v4l2_queryctrl *ctrl, void *dec)
{
	int err = 0;
	int id;
	int ch_id;
	int i = 0;
	struct tvp5147_control_info *control = NULL;
	int input_idx;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Starting tvp5147_queryctrl...\n");
	input_idx = tvp5147_channel_info[ch_id].params.inputidx;
	if (ctrl == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL Pointer\n");
		return -EINVAL;
	}
	id = ctrl->id;
	memset(ctrl, 0, sizeof(struct v4l2_queryctrl));
	ctrl->id = id;
	for (i = 0; i < tvp5147_configuration[ch_id].input[input_idx].
	     no_of_controls; i++) {
		control = &tvp5147_configuration[ch_id].input[input_idx].
		    controls[i];
		if ((control->query_control).id == ctrl->id) {
			break;
		}
	}
	if (i == tvp5147_configuration[ch_id].input[input_idx].no_of_controls) {
		dev_err(tvp5147_i2c_dev[ch_id], "Invalid id...\n");
		return -EINVAL;
	}
	memcpy(ctrl, &control->query_control, sizeof(struct v4l2_queryctrl));
	dev_dbg(tvp5147_i2c_dev[ch_id], "End of tvp5147_querycontrol...\n");
	return err;
}

static int tvp5147_raw_vbi_setformat(struct v4l2_vbi_format *fmt)
{
	/* TBD */
}

static int tvp5147_sliced_vbi_setformat(struct v4l2_sliced_vbi_format *fmt,
					void *dec)
{
	int ch_id, i, j, k, index;
	u16 val;
	u8 num_services = 0;
	ch_id = ((struct decoder_device *)dec)->channel_id;
	if ((fmt->service_set |
	     tvp5147_configuration[ch_id].sliced_cap.service_set) !=
	    tvp5147_configuration[ch_id].sliced_cap.service_set) {
		dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_setformat:"
			"Invalid service\n");
		return -EINVAL;
	}
	memset(fmt->service_lines, 0, 2 * 24 * 2);
	for (i = 0; i < TVP5147_VBI_NUM_SERVICES; i++) {
		if ((fmt->service_set & tvp5147_sliced_regs[i].service) &&
		    !(tvp5147_sliced_regs[i].std &
		      tvp5147_channel_info[ch_id].params.std)) {
			dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_setformat:"
				"Invalid service for this standard\n");
			return -EINVAL;
		}

		if (tvp5147_sliced_regs[i].std &
		    tvp5147_channel_info[ch_id].params.std) {
			for (j = 0; j < 2; j++) {
				for (k = 0; k < 3; k++)
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VBUS_ADDRESS_ACCESS0 + k,
					     tvp5147_sliced_regs[i].field[j].
					     fifo_line_addr[k]);

				if (fmt->service_set &
				    tvp5147_sliced_regs[i].service) {
					num_services++;
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VBUS_DATA_ACCESS_AUTO_INCR,
					     tvp5147_sliced_regs[i].
					     line_addr_value);
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VBUS_DATA_ACCESS_AUTO_INCR,
					     tvp5147_sliced_regs[i].field[j].
					     fifo_mode_value);
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VDP_LINE_START,
					     tvp5147_sliced_regs[i].line_start);
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VDP_LINE_STOP,
					     tvp5147_sliced_regs[i].line_end);
					for (k = 0; k < 2; k++) {
						index = tvp5147_sliced_regs[i].
						    service_line[k].index;
						val = tvp5147_sliced_regs[i].
						    service_line[k].value;
						fmt->service_lines[k][index] =
						    val;
					}
				} else {
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VBUS_DATA_ACCESS_AUTO_INCR,
					     TVP5147_LINE_ADDRESS_DEFAULT);
					tvp5147_i2c_write_reg
					    (&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_VBUS_DATA_ACCESS_AUTO_INCR,
					     TVP5147_LINE_MODE_DEFAULT);
				}
			}
		}
	}
	fmt->io_size = (TVP5147_SLICED_BUF_SIZE * num_services) / 2;
	tvp5147_configuration[ch_id].num_services = num_services / 2;
	return num_services / 2;
}

/*  tvp5147_setformat :
 * This function is used to set sliced vbi services
 */
static int tvp5147_setformat(struct v4l2_format *fmtp, void *dec)
{
	int err = 0, ch_id;
	struct v4l2_sliced_vbi_format fmta;
	struct v4l2_vbi_format fmtb;
	if (NULL == dec || NULL == fmtp) {
		printk("tvp5147_setformat:NULL Pointer\n");
		return -EINVAL;
	}
	if ((V4L2_BUF_TYPE_VIDEO_CAPTURE != fmtp->type) &&
	    (V4L2_BUF_TYPE_VBI_CAPTURE != fmtp->type) &&
	    (V4L2_BUF_TYPE_SLICED_VBI_CAPTURE != fmtp->type))
		return -EINVAL;
	ch_id = ((struct decoder_device *)dec)->channel_id;

	if (V4L2_BUF_TYPE_SLICED_VBI_CAPTURE == fmtp->type) {
		fmta = fmtp->fmt.sliced;
		err = tvp5147_sliced_vbi_setformat(&fmta, dec);
	} else {
		fmtb = fmtp->fmt.vbi;
		err = tvp5147_raw_vbi_setformat(&fmtb);
	}

	tvp5147_channel_info[ch_id].params.fmt = *fmtp;
	dev_dbg(tvp5147_i2c_dev[ch_id],
		"tvp5147_setformat:End of" " tvp5147_querycontrol...\n");
	return err;
}

/* tvp5147_getformat: 
 * This function is used to set sliced vbi services
 */
static int tvp5147_getformat(struct v4l2_format *fmtp, void *dec)
{
	int err = 0;
	int ch_id;

	if (NULL == dec || NULL == fmtp) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "tvp5147_getformat:"
		"Starting getformat function.\n");

	/* Read sliced vbi format */
	fmtp->fmt.sliced = tvp5147_channel_info[ch_id].params.fmt.fmt.sliced;
	dev_dbg(tvp5147_i2c_dev[ch_id], "tvp5147_getformat:"
		"End of getformat function.\n");
	return err;
}

/* tvp5147_tryformat:
 * This function is used to set sliced vbi services
 */
static int tvp5147_tryformat(struct v4l2_format *fmtp, void *dec)
{
	int err = 0;
	int ch_id;
	int i = 0;
	struct v4l2_sliced_vbi_format *fmt;
	int num_services = 0;

	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "tvp5147_tryformat:"
		"Start of tvp5147_tryformat..\n");
	if (fmtp == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_tryformat:"
			"NULL pointer\n");
		return -EINVAL;
	}
	fmt = &(fmtp->fmt.sliced);
	if ((fmt->service_set |
	     tvp5147_configuration[ch_id].sliced_cap.service_set) !=
	    tvp5147_configuration[ch_id].sliced_cap.service_set) {
		fmt->service_set =
		    tvp5147_configuration[ch_id].sliced_cap.service_set;
		dev_err(tvp5147_i2c_dev[ch_id], "Invalid service\n");
		return -EINVAL;
	}
	for (i = 0; i < TVP5147_VBI_NUM_SERVICES; i++) {
		if (((tvp5147_sliced_regs[i].service & fmt->service_set) ==
		     tvp5147_sliced_regs[i].service) &&
		    (tvp5147_channel_info[ch_id].params.std
		     != tvp5147_sliced_regs[i].std)) {
			dev_err(tvp5147_i2c_dev[ch_id],
				"service not supported for the standard\n");
			return -EINVAL;
		}
	}
	num_services = 0;
	for (i = 0; i < TVP5147_VBI_NUM_SERVICES; i++) {
		if (tvp5147_sliced_regs[i].service & fmt->service_set)
			num_services++;
	}
	fmt->io_size = num_services * TVP5147_SLICED_BUF_SIZE;
	dev_dbg(tvp5147_i2c_dev[ch_id], "</tvp5147_tryformat>\n");
	return err;
}

/* tvp5147_setstd :
 * This function is used to configure TVP5147 for video standard passed
 * by application
 */
static int tvp5147_setstd(v4l2_std_id * id, void *dec)
{
	int err = 0;
	unsigned char output1;
	int ch_id;
	int i = 0;
	struct v4l2_standard *standard;
	int input_idx;
	struct v4l2_sliced_vbi_format fmt;

	tvp5147_mode mode = TVP5147_MODE_INV;
	v4l2_std_id std;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();

	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Start of tvp5147_setstd..\n");
	input_idx = tvp5147_channel_info[ch_id].params.inputidx;
	if (id == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL pointer.\n");
		return -EINVAL;
	}
	std = *id;
	for (i = 0; i < tvp5147_configuration[ch_id].input[input_idx].
	     no_of_standard; i++) {
		standard = &(tvp5147_configuration[ch_id].input[input_idx].
			     standard[i]);
		if (standard->id & std) {
			break;
		}
	}
	if (i == tvp5147_configuration[ch_id].input[input_idx].no_of_standard) {
		dev_err(tvp5147_i2c_dev[ch_id], "Invalid id...\n");
		return -EINVAL;
	}
	mode = tvp5147_configuration[ch_id].input[input_idx].mode[i][0];

	/* for square pixel */
	err |= tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_OUTPUT1, &output1);
	if (err < 0) {
		return err;
	}
	output1 |= ((mode & 0x8) << 4);
	err |= tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				     client, TVP5147_OUTPUT1, output1);

	/* setup the video standard */
	err |= tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				     client, TVP5147_VIDEO_STD, (mode & 0x07));

	/* if autoswitch mode, enable all modes for autoswitch */
	if ((mode & 0x07) == TVP5147_MODE_AUTO) {
		err |= tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].
					     i2c_dev.client,
					     TVP5147_AUTOSWT_MASK,
					     TVP5147_AUTOSWITCH_MASK);
	}
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "Set standard failed\n");
		return err;
	}
	tvp5147_channel_info[ch_id].params.std = *id;
	if (*id == VPFE_STD_AUTO || *id == VPFE_STD_AUTO_SQP) {
		err = tvp5147_querystd(id, dec);
	}
	/* disable all vbi services */
	fmt.service_set = 0;
	tvp5147_sliced_vbi_setformat(&fmt, dec);

	dev_dbg(tvp5147_i2c_dev[ch_id], "End of tvp5147 set standard...\n");
	return err;
}

/* tvp5147_getstd :
 * Function to get the video standard
 */
static int tvp5147_getstd(v4l2_std_id * id, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Starting getstd function..\n");
	if (id == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL Pointer\n");
		return -EINVAL;
	}
	*id = tvp5147_channel_info[ch_id].params.std;
	dev_dbg(tvp5147_i2c_dev[ch_id], "End of tvp5147_getstd function\n");
	return err;
}

/* tvp5147_enumstd :
 * Function to enumerate standards supported
 */
static int tvp5147_enumstd(struct v4l2_standard *std, void *dec)
{
	int index, index1;
	int err = 0;
	int ch_id;
	int input_idx, sumstd = 0;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	if (std == NULL) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	index = std->index;
	index1 = index;
	/* Check for valid value of index */
	for (input_idx = 0;
	     input_idx < tvp5147_configuration[ch_id].no_of_inputs;
	     input_idx++) {
		sumstd +=
		    tvp5147_configuration[ch_id].input[input_idx].
		    no_of_standard;
		if (index < sumstd) {
			sumstd -= tvp5147_configuration[ch_id].
			    input[input_idx].no_of_standard;
			break;
		}
	}
	if (input_idx == tvp5147_configuration[ch_id].no_of_inputs)
		return -EINVAL;
	index -= sumstd;
	memset(std, 0, sizeof(*std));
	memcpy(std, &tvp5147_configuration[ch_id].input[input_idx].
	       standard[index], sizeof(struct v4l2_standard));
	std->index = index1;
	return err;
}

/* tvp5147_querystd :
 *
 * Function to return standard detected by decoder
 */
static int tvp5147_querystd(v4l2_std_id * id, void *dec)
{
	int err = 0;

	unsigned char std;
	int ch_id;
	unsigned char output1 = 0;
	int i = 0, j = 0;
	unsigned char lock_status = 0xFF;
	int input_idx, flag = 1;
	struct v4l2_sliced_vbi_format fmt;

	tvp5147_mode mode;
	u8 lock_mask;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();

	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id],
		"Start of tvp5147 standard detection.\n");
	input_idx = tvp5147_channel_info[ch_id].params.inputidx;
	if (id == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL Pointer\n");
		return -EINVAL;
	}
	err = tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				   client, TVP5147_VIDEO_STD, &std);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "Standard detection failed\n");
		return err;
	}

	std &= 0x7;
	if (std == TVP5147_MODE_AUTO) {

		err |= tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].
					    i2c_dev.client,
					    TVP5147_VID_STD_STATUS, &std);
	}
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "Standard detection failed\n");
		return err;
	}

	/* to keep standard without square pixel */
	std &= 0x7;

	/* for square pixel */
	err |=
	    tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				 client, TVP5147_OUTPUT1, &output1);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id],
			"Setting square pixel failed.\n");
		return err;
	}
	mode = std | ((output1 & 0x80) >> 4);	/* square pixel status */

	/* check lock status */
	err |= tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_STATUS1, &lock_status);
	if (err < 0) {
		return err;
	}
	err = -EAGAIN;
	lock_mask = tvp5147_configuration[ch_id].input[input_idx].lock_mask;
	if (lock_mask != (lock_status & lock_mask)) {
		return err;
	}
	for (i = 0; i < tvp5147_configuration[ch_id].input[input_idx].
	     no_of_standard; i++) {
		for (j = 0; j < TVP5147_MAX_NO_MODES; j++) {
			if (mode == tvp5147_configuration[ch_id].
			    input[input_idx].mode[i][j]) {
				flag = 0;
				break;
			}
		}
		if (!flag)
			break;
	}

	if ((i == tvp5147_configuration[ch_id].input[input_idx].
	     no_of_standard) && (TVP5147_MAX_NO_MODES == j)) {
		dev_err(tvp5147_i2c_dev[ch_id],
			"tvp5147_querystd:Invalid std\n");
		return -EINVAL;
	}
	*id = tvp5147_configuration[ch_id].input[input_idx].standard[i].id;
	tvp5147_channel_info[ch_id].params.std = *id;

	/* disable all vbi services */
	fmt.service_set = 0;
	tvp5147_sliced_vbi_setformat(&fmt, dec);

	dev_dbg(tvp5147_i2c_dev[ch_id], "End of detection...\n");
	return 0;
}

/* tvp5147_setinput:
 * Function to set the input
 */
static int tvp5147_setinput(int *index, void *dec)
{
	int err = 0;
	unsigned char input_sel;
	int ch_id;
	u8 status;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();

	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Start of set input function\n");

	/* check for null pointer */
	if (index == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL Pointer\n");
		return -EINVAL;
	}
	if ((*index >= tvp5147_configuration[ch_id].no_of_inputs)
	    || (*index < 0)) {
		dev_err(tvp5147_i2c_dev[ch_id], "Invalid Index.\n");
		return -EINVAL;
	}
	input_sel = tvp5147_configuration[ch_id].input[*index].input_type;
	err = tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_INPUT_SEL, input_sel);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "Set input failed\n");
		return -EINVAL;
	}
	mdelay(500);
	err = tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_CLEAR_LOST_LOCK, 0x01);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_getinput:error "
			"writing  clear lost lock register\n");
		return err;
	}

	/* wait here so that if lock is lost, it can be detected */
	mdelay(500);
	err |= tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_STATUS1, &status);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_getinput:error "
			"reading status register\n");
		return -EINVAL;
	}
	if (TVP5147_LOST_LOCK_MASK == (status & TVP5147_LOST_LOCK_MASK)) {
		return -EINVAL;
	}
	tvp5147_channel_info[ch_id].params.inputidx = *index;
	dev_dbg(tvp5147_i2c_dev[ch_id], "End of set input function\n");
	return err;
}

/* tvp5147_getinput :
 * Function to get the input
 */
static int tvp5147_getinput(int *index, void *dec)
{
	int err = 0;
	int ch_id;
	unsigned char input_sel;

	unsigned char status;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();

	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp5147_i2c_dev[ch_id], "Start of get input function.\n");

	/* check for null pointer */
	if (index == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL Pointer\n");
		return -EINVAL;
	}
	input_sel = tvp5147_configuration[ch_id].input[0].input_type;
	err = tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_INPUT_SEL, input_sel);
	mdelay(500);
	if (err < 0) {
		return err;
	}
	err = tvp5147_i2c_write_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_CLEAR_LOST_LOCK, 0x01);
	if (err < 0) {
		return err;
	}

	/* wait here so that if lock is lost, it can be detected */
	mdelay(500);
	err |= tvp5147_i2c_read_reg(&tvp5147_channel_info[ch_id].i2c_dev.
				    client, TVP5147_STATUS1, &status);
	if (err < 0) {
		dev_err(tvp5147_i2c_dev[ch_id], "tvp5147_getinput:error "
			"reading status register\n");
		return -EINVAL;
	}
	if (TVP5147_LOST_LOCK_MASK == (status & TVP5147_LOST_LOCK_MASK)) {
		return -EINVAL;
	}
	*index = 0;

	/* Store the input type in index */
	tvp5147_channel_info[ch_id].params.inputidx = *index;
	dev_dbg(tvp5147_i2c_dev[ch_id], "End of tvp5147_getinput.\n");
	return 0;
}

/* tvp5147_enuminput :
 * Function to enumerate the input
 */
static int tvp5147_enuminput(struct v4l2_input *input, void *dec)
{
	int err = 0;
	int index = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;

	/* check for null pointer */
	if (input == NULL) {
		dev_err(tvp5147_i2c_dev[ch_id], "NULL Pointer\n");
		return -EINVAL;
	}

	/* Only one input is available */
	if (input->index >= tvp5147_configuration[ch_id].no_of_inputs) {
		return -EINVAL;
	}
	index = input->index;
	memset(input, 0, sizeof(*input));
	input->index = index;
	memcpy(input,
	       &tvp5147_configuration[ch_id].input[index].input_info,
	       sizeof(struct v4l2_input));
	return err;
}

/* tvp5147_setparams :
 * Function to set the parameters for tvp5147
 */
static int tvp5147_setparams(void *params, void *dec)
{
	int err = 0;
	int ch_id;
	tvp5147_params tvp5147params;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	if (copy_from_user(&tvp5147params, (tvp5147_params *) params,
			   sizeof(tvp5147params)))
		return -EFAULT;

	if (0 == ((struct decoder_device *)dec)->channel_id)
		err |= set_cpld_for_tvp5147();

	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;

	err |= tvp5147_setinput(&(tvp5147params.inputidx), dec);
	if (err < 0) {
		return err;
	}
	err |= tvp5147_setstd(&(tvp5147params.std), dec);
	if (err < 0) {
		return err;
	}
	err |= tvp5147_setformat(&(tvp5147params.fmt), dec);
	if (err < 0) {
		return err;
	}
	tvp5147_channel_info[ch_id].params = tvp5147params;
	return err;
}

/*  tvp5147_getparams :
 *  Function to get the parameters for tvp5147
 */
static int tvp5147_getparams(void *params, void *dec)
{
	int err = 0;
	int ch_id;
	tvp5147_params *tvp5147params = (tvp5147_params *) params;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;

	if (copy_to_user(tvp5147params, &(tvp5147_channel_info[ch_id].params),
			 sizeof(*tvp5147params)))
		return -EFAULT;

	return err;
}

/* tvp5147_i2c_read_reg :This function is used to read value from
 * register for i2c client.
 */
static int tvp5147_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[1];
	if (!client->adapter) {
		err = -ENODEV;
	} else {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 1;
		msg->buf = data;
		data[0] = reg;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			msg->flags = I2C_M_RD;
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				*val = data[0];
			}
		}
	}
	return ((err < 0) ? err : 0);
}

/* tvp5147_i2c_write_reg :This function is used to write value into register
 * for i2c client.
 */
static int tvp5147_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[2];
	if (!client->adapter) {
		err = -ENODEV;
	} else {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;
		data[0] = reg;
		data[1] = val;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	return ((err < 0) ? err : 0);
}

/* tvp5147_i2c_attach_client : This function is used to attach i2c client
 */
static int tvp5147_i2c_attach_client(struct i2c_client *client,
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

/* tvp5147_i2c_detach_client:
 * This function is used to detach i2c client
 */
static int tvp5147_i2c_detach_client(struct i2c_client *client)
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

/* tvp5147A_i2c_probe_adapter : This function is used to probe i2c adapter
 */
static int tvp5147A_i2c_probe_adapter(struct i2c_adapter *adap)
{
	int err = 0;
	tvp5147_i2c_dev[0] = &(adap->dev);
	dev_dbg(tvp5147_i2c_dev[0], "Tvp5147A i2c probe adapter called...\n");
	err = tvp5147_i2c_attach_client(&tvp5147_channel_info[0].i2c_dev.
					client,
					&tvp5147_channel_info[0].i2c_dev.
					driver, adap,
					tvp5147_channel_info[0].i2c_dev.
					i2c_addr);
	dev_dbg(tvp5147_i2c_dev[0], "Tvp5147A i2c probe adapter ends...\n");
	return err;
}

/* tvp5147B_i2c_probe_adapter: This function is used to probe i2c adapter
 */
static int tvp5147B_i2c_probe_adapter(struct i2c_adapter *adap)
{
	int err = 0;
	tvp5147_i2c_dev[1] = &(adap->dev);
	dev_dbg(tvp5147_i2c_dev[1], "Tvp5147B i2c probe adapter called...\n");
	err = tvp5147_i2c_attach_client(&tvp5147_channel_info[1].i2c_dev.
					client,
					&tvp5147_channel_info[1].i2c_dev.
					driver, adap,
					tvp5147_channel_info[1].i2c_dev.
					i2c_addr);
	dev_dbg(tvp5147_i2c_dev[1], "Tvp5147B i2c probe adapter ends...\n");
	return err;
}

/* tvp5147_i2c_init : This function is used initialize TVP5147 i2c client
 */
static int tvp5147_i2c_init(void)
{
	int err = 0;
	int i = 0, j = 0;

	/* Take instance of driver */
	struct i2c_driver *driver;
	char strings[TVP5147_NUM_CHANNELS][80] =
	    { "TVP5147 channel0 Video Decoder I2C driver",
		"TVP5147 channel1 Video Decoder I2C driver"
	};
	for (i = 0; i < TVP5147_NUM_CHANNELS; i++) {
		driver = &tvp5147_channel_info[i].i2c_dev.driver;
		driver->owner = THIS_MODULE;
		strlcpy(driver->name, strings[i], sizeof(strings[i]));
		driver->id = I2C_DRIVERID_EXP0;
		driver->flags = I2C_DF_NOTIFY;
		if (0 == i) {
			driver->attach_adapter = tvp5147A_i2c_probe_adapter;
		} else {
			driver->attach_adapter = tvp5147B_i2c_probe_adapter;
		}
		driver->detach_client = tvp5147_i2c_detach_client;
		err = vpif_register_decoder(&dec_dev[i]);
		if (err < 0) {
			for (j = i - 1; j > 0; j--) {
				vpif_unregister_decoder(&dec_dev[j]);
			}
			return err;
		}
	}
	return err;
}

/* tvp5147_i2c_cleanup : This function is used detach TVP5147 i2c client
 */
static void tvp5147_i2c_cleanup(void)
{
	int i;
	for (i = 0; i < TVP5147_NUM_CHANNELS; i++) {
		if (tvp5147_channel_info[i].i2c_dev.i2c_registration & 0x01) {
			i2c_del_driver(&tvp5147_channel_info[i].i2c_dev.driver);
			tvp5147_channel_info[i].i2c_dev.client.adapter = NULL;
			tvp5147_channel_info[i].i2c_dev.i2c_registration = 0;
		}
		vpif_unregister_decoder(&dec_dev[i]);
	}
}

module_init(tvp5147_i2c_init);
module_exit(tvp5147_i2c_cleanup);
MODULE_LICENSE("GPL");
