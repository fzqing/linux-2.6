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
/* tvp7002.c */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <media/davinci/tvp7002.h>
#include <asm/arch/i2c-client.h>
#include <asm/arch/video_hdevm.h>

/* Function Prototypes */
static int tvp7002_initialize(void *dec, int flag);
static int tvp7002_deinitialize(void *dec);
static int tvp7002_setcontrol(struct v4l2_control *ctrl, void *dec);
static int tvp7002_getcontrol(struct v4l2_control *ctrl, void *dec);
static int tvp7002_querycontrol(struct v4l2_queryctrl *ctrl, void *dec);
static int tvp7002_setstd(v4l2_std_id * id, void *dec);
static int tvp7002_getstd(v4l2_std_id * id, void *dec);
static int tvp7002_querystd(v4l2_std_id * id, void *dec);
static int tvp7002_enumstd(struct v4l2_standard *std, void *dec);
static int tvp7002_setinput(int *index, void *dec);
static int tvp7002_getinput(int *index, void *dec);
static int tvp7002_enuminput(struct v4l2_input *input, void *dec);
static int tvp7002_set_format_params(struct tvp7002_format_params
				     *tvpformats, void *dec);
static int tvp7002_setparams(void *params, void *dec);
static int tvp7002_getparams(void *params, void *dec);
static int tvp7002_i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val);
static int tvp7002_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);
static int tvp7002_i2c_attach_client(struct i2c_client *client,
				     struct i2c_driver *driver,
				     struct i2c_adapter *adap, int addr);
static int tvp7002_i2c_detach_client(struct i2c_client *client);
static int tvp7002_i2c_probe_adapter(struct i2c_adapter *adap);
static int tvp7002_i2c_init(void);
static void tvp7002_i2c_cleanup(void);

static int ths7353_setvalue(void);

static struct v4l2_standard tvp7002_standards[TVP7002_MAX_NO_STANDARDS] = {
	{
	 .index = 0,
	 .id = V4L2_STD_720P_60,
	 .name = "720P-60",
	 .frameperiod = {1, 60},
	 .framelines = 720},
	{
	 .index = 1,
	 .id = V4L2_STD_1080I_60,
	 .name = "1080I-30",
	 .frameperiod = {1, 30},
	 .framelines = 1080},
	{
	 .index = 2,
	 .id = V4L2_STD_1080I_50,
	 .name = "1080I-25",
	 .frameperiod = {1, 25},
	 .framelines = 1080},
	{
	 .index = 3,
	 .id = V4L2_STD_720P_50,
	 .name = "720P-50",
	 .frameperiod = {1, 50},
	 .framelines = 720},
	{
	 .index = 4,
	 .id = V4L2_STD_525P_60,
	 .name = "480P-60",
	 .frameperiod = {1, 60},
	 .framelines = 525},
	{
	 .index = 5,
	 .id = V4L2_STD_625P_50,
	 .name = "576P-50",
	 .frameperiod = {1, 50},
	 .framelines = 625},
};

static struct tvp7002_format_params tvp7002_formats[TVP7002_MAX_NO_STANDARDS] = {
	{
	 .hpll_divider_msb = FEEDBACK_DIVIDER_MSB_720p,
	 .hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_720p,
	 .hpll_vco_control = VCO_CONTROL_720p,
	 .hpll_cp_current = CP_CURRENT_720p,
	 .hpll_phase_select = PHASE_SELECT_720p,
	 .hpll_post_divider = POST_DIVIDER_720p,
	 .hpll_control = HPLL_CONTROL_720p,
	 .avid_start_msb = AVID_START_PIXEL_MSB_720p,
	 .avid_start_lsb = AVID_START_PIXEL_LSB_720p,
	 .avid_stop_lsb = AVID_STOP_PIXEL_LSB_720p,
	 .avid_stop_msb = AVID_STOP_PIXEL_MSB_720p,
	 .vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_720p,
	 .vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_720p,
	 .vblk_f0_duration = VBLK_F0_DURATION_720p,
	 .vblk_f1_duration = VBLK_F1_DURATION_720p,
	 .alc_placement = TVP7002_HD_ALC_PLACEMENT,
	 .clamp_start = TVP7002_HD_CLAMP_START,
	 .clamp_width = TVP7002_HD_CLAMP_WIDTH,
	 .hpll_pre_coast = TVP7002_HD_PRE_COAST,
	 .hpll_post_coast = TVP7002_HD_POST_COAST,
	 .reserved = RESERVED_720p},
	{
	 .hpll_divider_msb = FEEDBACK_DIVIDER_MSB_1080i,
	 .hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_1080i,
	 .hpll_vco_control = VCO_CONTROL_1080i,
	 .hpll_cp_current = CP_CURRENT_1080i,
	 .hpll_phase_select = PHASE_SELECT_1080i,
	 .hpll_post_divider = POST_DIVIDER_1080i,
	 .hpll_control = HPLL_CONTROL_1080i,
	 .avid_start_msb = AVID_START_PIXEL_MSB_1080i,
	 .avid_start_lsb = AVID_START_PIXEL_LSB_1080i,
	 .avid_stop_lsb = AVID_STOP_PIXEL_LSB_1080i,
	 .avid_stop_msb = AVID_STOP_PIXEL_MSB_1080i,
	 .vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_1080i,
	 .vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_1080i,
	 .vblk_f0_duration = VBLK_F0_DURATION_1080i,
	 .vblk_f1_duration = VBLK_F1_DURATION_1080i,
	 .alc_placement = TVP7002_HD_ALC_PLACEMENT,
	 .clamp_start = TVP7002_HD_CLAMP_START,
	 .clamp_width = TVP7002_HD_CLAMP_WIDTH,
	 .hpll_pre_coast = TVP7002_HD_PRE_COAST,
	 .hpll_post_coast = TVP7002_HD_POST_COAST,
	 .reserved = RESERVED_1080i},
	{
	 .hpll_divider_msb = FEEDBACK_DIVIDER_MSB_1080i_50,
	 .hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_1080i_50,
	 .hpll_vco_control = VCO_CONTROL_1080i_50,
	 .hpll_cp_current = CP_CURRENT_1080i_50,
	 .hpll_phase_select = PHASE_SELECT_1080i_50,
	 .hpll_post_divider = POST_DIVIDER_1080i_50,
	 .hpll_control = HPLL_CONTROL_1080i_50,
	 .avid_start_msb = AVID_START_PIXEL_MSB_1080i_50,
	 .avid_start_lsb = AVID_START_PIXEL_LSB_1080i_50,
	 .avid_stop_lsb = AVID_STOP_PIXEL_LSB_1080i_50,
	 .avid_stop_msb = AVID_STOP_PIXEL_MSB_1080i_50,
	 .vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_1080i_50,
	 .vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_1080i_50,
	 .vblk_f0_duration = VBLK_F0_DURATION_1080i_50,
	 .vblk_f1_duration = VBLK_F1_DURATION_1080i_50,
	 .alc_placement = TVP7002_HD_ALC_PLACEMENT,
	 .clamp_start = TVP7002_HD_CLAMP_START,
	 .clamp_width = TVP7002_HD_CLAMP_WIDTH,
	 .hpll_pre_coast = TVP7002_HD_PRE_COAST,
	 .hpll_post_coast = TVP7002_HD_POST_COAST,
	 .reserved = RESERVED_1080i_50},
	{
	 .hpll_divider_msb = FEEDBACK_DIVIDER_MSB_720p_50,
	 .hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_720p_50,
	 .hpll_vco_control = VCO_CONTROL_720p_50,
	 .hpll_cp_current = CP_CURRENT_720p_50,
	 .hpll_phase_select = PHASE_SELECT_720p_50,
	 .hpll_post_divider = POST_DIVIDER_720p_50,
	 .hpll_control = HPLL_CONTROL_720p_50,
	 .avid_start_msb = AVID_START_PIXEL_MSB_720p_50,
	 .avid_start_lsb = AVID_START_PIXEL_LSB_720p_50,
	 .avid_stop_lsb = AVID_STOP_PIXEL_LSB_720p_50,
	 .avid_stop_msb = AVID_STOP_PIXEL_MSB_720p_50,
	 .vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_720p_50,
	 .vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_720p_50,
	 .vblk_f0_duration = VBLK_F0_DURATION_720p_50,
	 .vblk_f1_duration = VBLK_F1_DURATION_720p_50,
	 .alc_placement = TVP7002_HD_ALC_PLACEMENT,
	 .clamp_start = TVP7002_HD_CLAMP_START,
	 .clamp_width = TVP7002_HD_CLAMP_WIDTH,
	 .hpll_pre_coast = TVP7002_HD_PRE_COAST,
	 .hpll_post_coast = TVP7002_HD_POST_COAST,
	 .reserved = RESERVED_720p},
	{
	 .hpll_divider_msb = FEEDBACK_DIVIDER_MSB_480P,
	 .hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_480P,
	 .hpll_vco_control = VCO_CONTROL_480P,
	 .hpll_cp_current = CP_CURRENT_480P,
	 .hpll_phase_select = PHASE_SELECT_480P,
	 .hpll_post_divider = POST_DIVIDER_480P,
	 .hpll_control = HPLL_CONTROL_480P,
	 .avid_start_msb = AVID_START_PIXEL_MSB_480P,
	 .avid_start_lsb = AVID_START_PIXEL_LSB_480P,
	 .avid_stop_lsb = AVID_STOP_PIXEL_LSB_480P,
	 .avid_stop_msb = AVID_STOP_PIXEL_MSB_480P,
	 .vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_480P,
	 .vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_480P,
	 .vblk_f0_duration = VBLK_F0_DURATION_480P,
	 .vblk_f1_duration = VBLK_F1_DURATION_480P,
	 .alc_placement = TVP7002_ED_ALC_PLACEMENT,
	 .clamp_start = TVP7002_ED_CLAMP_START,
	 .clamp_width = TVP7002_ED_CLAMP_WIDTH,
	 .hpll_pre_coast = TVP7002_ED_PRE_COAST,
	 .hpll_post_coast = TVP7002_ED_POST_COAST,
	 .reserved = RESERVED_720p},
	{
	 .hpll_divider_msb = FEEDBACK_DIVIDER_MSB_576P,
	 .hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_576P,
	 .hpll_vco_control = VCO_CONTROL_576P,
	 .hpll_cp_current = CP_CURRENT_576P,
	 .hpll_phase_select = PHASE_SELECT_576P,
	 .hpll_post_divider = POST_DIVIDER_576P,
	 .hpll_control = HPLL_CONTROL_576P,
	 .avid_start_msb = AVID_START_PIXEL_MSB_576P,
	 .avid_start_lsb = AVID_START_PIXEL_LSB_576P,
	 .avid_stop_lsb = AVID_STOP_PIXEL_LSB_576P,
	 .avid_stop_msb = AVID_STOP_PIXEL_MSB_576P,
	 .vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_576P,
	 .vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_576P,
	 .vblk_f0_duration = VBLK_F0_DURATION_576P,
	 .vblk_f1_duration = VBLK_F1_DURATION_576P,
	 .alc_placement = TVP7002_ED_ALC_PLACEMENT,
	 .clamp_start = TVP7002_ED_CLAMP_START,
	 .clamp_width = TVP7002_ED_CLAMP_WIDTH,
	 .hpll_pre_coast = TVP7002_ED_PRE_COAST,
	 .hpll_post_coast = TVP7002_ED_POST_COAST,
	 .reserved = RESERVED_720p}
};

static struct tvp7002_config tvp7002_configuration[TVP7002_NUM_CHANNELS] = {
	{
	 .no_of_inputs = TVP7002_MAX_NO_INPUTS,
	 .input[0] = {
		      .input_type = TVP7002_HD_INPUT,
		      .input_info = {
				     .index = 0,
				     .name = "COMPONENT",
				     .type = V4L2_INPUT_TYPE_CAMERA,
				     .std = V4L2_STD_TVP7002_ALL},
		      .no_of_standard = TVP7002_MAX_NO_STANDARDS,
		      .standard = (struct v4l2_standard *)&tvp7002_standards,
		      .def_std = V4L2_STD_720P_60,
		      .format =
		      (struct tvp7002_format_params *)&tvp7002_formats,
		      .no_of_controls = TVP7002_MAX_NO_CONTROLS,
		      .controls = NULL},
	 .def_params = {V4L2_STD_720P_60, 0, {1, 0xa, 0x6}, {0, 0, 0, 7, 7, 7},
			{0x80, 0x80, 0x80, 0, 0, 0, 0x10, 0x10, 0x10}}
	 }
};

static struct tvp7002_channel tvp7002_channel_info[TVP7002_NUM_CHANNELS] = {
	{
	 .params.inputidx = 0,
	 .params.std = V4L2_STD_720P_60,
	 .i2c_dev = {
		     .i2c_addr = (0xBA >> 1),
		     .i2c_registration = 0},
	 .dec_device = NULL}
};

/* Global variables */
static struct device *tvp7002_i2c_dev[TVP7002_NUM_CHANNELS];
static struct param_ops params_ops = {
	.setparams = tvp7002_setparams,
	.getparams = tvp7002_getparams
};
static struct control_ops controls_ops = {
	.count = TVP7002_MAX_NO_CONTROLS,
	.queryctrl = tvp7002_querycontrol,
	.setcontrol = tvp7002_setcontrol,
	.getcontrol = tvp7002_getcontrol
};
static struct input_ops inputs_ops = {
	.count = TVP7002_MAX_NO_INPUTS,
	.enuminput = tvp7002_enuminput,
	.setinput = tvp7002_setinput,
	.getinput = tvp7002_getinput
};
static struct standard_ops standards_ops = {
	.count = TVP7002_MAX_NO_STANDARDS,
	.enumstd = tvp7002_enumstd,
	.setstd = tvp7002_setstd,
	.getstd = tvp7002_getstd,
	.querystd = tvp7002_querystd,
};
static struct decoder_device tvp7002_dev[TVP7002_NUM_CHANNELS] = {
	{
	 .name = "TVP7002",
	 .if_type = INTERFACE_TYPE_BT1120,
	 .channel_id = 0,
	 .capabilities = 0,
	 .initialize = tvp7002_initialize,
	 .std_ops = &standards_ops,
	 .ctrl_ops = &controls_ops,
	 .input_ops = &inputs_ops,
	 .fmt_ops = NULL,
	 .params_ops = &params_ops,
	 .deinitialize = tvp7002_deinitialize}
};

/* tvp7002_initialize :
 * This function will set the video format standard
 */
static int tvp7002_initialize(void *dec, int flag)
{
	int err = 0;
	int ch_id;
	v4l2_std_id std;
	struct i2c_client *ch_client;
	int index;
	if (NULL == dec) {
		printk("dec:NULL pointer");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	ch_client = &tvp7002_channel_info[ch_id].i2c_dev.client;
	if (tvp7002_channel_info[ch_id].i2c_dev.i2c_registration & 0x01) {
		printk("tvp7002 driver is already initialized..\n");
		err = -EINVAL;
		return err;
	}

	/* Register TVP7002 I2C client */
	err = i2c_add_driver(&tvp7002_channel_info[ch_id].i2c_dev.driver);
	if (err) {
		printk("Failed to register TVP7002 I2C client.\n");
		return -EINVAL;
	}
	try_module_get(THIS_MODULE);
	tvp7002_channel_info[ch_id].i2c_dev.i2c_registration |= 1;
	tvp7002_channel_info[ch_id].dec_device = (struct decoder_device *)dec;
	if (DECODER_I2C_BIND_FLAG == flag) {
		/* check that decoder is set with default values once or not,
		 * if yes return, if not continue */
		if (tvp7002_channel_info[ch_id].i2c_dev.i2c_registration & 0x02)
			return err;

	}

	err |= set_cpld_for_tvp7002();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	dev_dbg(tvp7002_i2c_dev[ch_id], "Tvp7002 driver registered\n");
	/*Configure the TVP7002 in default 720p 60 Hz standard for normal
	   power up mode */
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_DIVIDER_MSB,
				     TVP7002_HPLL_MSB_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_DIVIDER_LSB,
				     TVP7002_HPLL_LSB_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_CONTROL,
				     TVP7002_HPLL_CONTROL_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_PHASE_SELECT,
				     TVP7002_HPLL_PHASE_SEL_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_CLAMP_START,
				     TVP7002_CLAMP_START_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_CLAMP_WIDTH,
				     TVP7002_CLAMP_WIDTH_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HSYNC_OUTPUT_WIDTH,
				     TVP7002_HSYNC_OUTWIDTH_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_SYNC_CONTROL_1,
				     TVP7002_SYNC_CONTROL1_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_CLAMP_CONTROL,
				     TVP7002_HPLL_CLAMP_CTRL_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_SYNC_ON_GREEN_THLD,
				     TVP7002_SYNC_GREEN_THLD_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_SYNC_SEPARATER_THLD,
				  TVP7002_SYNC_SEP_THLD_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_PRE_COAST,
				  TVP7002_HPLL_PRE_COAST_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_HPLL_POST_COAST,
				  TVP7002_HPLL_POST_COAST_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_OUTPUT_FORMATTER,
				  TVP7002_OUTPUT_FORMATTER_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_MISC_CONTROL_1,
				  TVP7002_MISC_CONTROL1_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_MISC_CONTROL_3,
				  TVP7002_MISC_CONTROL3_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_MISC_CONTROL_2,
				  TVP7002_MISC_CONTROL2_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_INPUT_MUX_SELECT_1,
				  TVP7002_INPUT_MUX_SELECT1_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_INPUT_MUX_SELECT_2,
				  TVP7002_INPUT_MUX_SELECT2_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_HSOUT_OUTPUT_START,
				     TVP7002_HSOUT_OUTPUT_START_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_MISC_CONTROL_4,
				     TVP7002_MISC_CONTROL4_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_ALC_ENABLE,
				     TVP7002_ALC_ENABLE_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_ALC_FILTER,
				     TVP7002_ALC_FILTER_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_FINE_CLAMP_CONTROL,
				     TVP7002_FINE_CLAMP_CONTROL_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_POWER_CONTROL,
				     TVP7002_POWER_CONTROL_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_ADC_SETUP,
				     TVP7002_ADC_SETUP_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client,
				     TVP7002_COARSE_CLAMP_CONTROL,
				     TVP7002_COARSE_CLAMP_CONTROL_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_SOG_CLAMP,
				     TVP7002_SOG_CLAMP_DEFAULT);
	err |= tvp7002_i2c_write_reg(ch_client, TVP7002_ALC_PLACEMENT,
				     TVP7002_ALC_PLACEMENT_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client,
				  TVP7002_VIDEO_BANDWIDTH_CONTROL,
				  TVP7002_VIDEO_BANDWIDTH_CONTROL_DEFAULT);
	err |=
	    tvp7002_i2c_write_reg(ch_client, TVP7002_AVID_START_PIXEL_HIGH,
				  TVP7002_AVID_START_PIXEL_DEFAULT);

	if (err < 0) {
		err = -EINVAL;
		tvp7002_deinitialize(dec);
		return err;
	} else {

		memcpy(&tvp7002_channel_info[ch_id].params,
		       &tvp7002_configuration[ch_id].def_params,
		       sizeof(tvp7002_params));
		/* Configure for default video standard */
		/* call set standard */
		index = tvp7002_channel_info[ch_id].params.inputidx;
		std = tvp7002_configuration[ch_id].input[index].def_std;
		err |= tvp7002_setstd(&std, dec);

		if (err < 0) {
			err = -EINVAL;
			tvp7002_deinitialize(dec);
			return err;
		}
	}
	tvp7002_channel_info[ch_id].i2c_dev.i2c_registration |= 0x2;
	dev_dbg(tvp7002_i2c_dev[ch_id], "End of tvp7002_init.\n");
	return err;
}
static int tvp7002_deinitialize(void *dec)
{
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Tvp7002 ch deinitialization"
		" called\n");
	if (tvp7002_channel_info[ch_id].i2c_dev.i2c_registration & 0x01) {
		i2c_del_driver(&tvp7002_channel_info[ch_id].i2c_dev.driver);
		module_put(THIS_MODULE);
		tvp7002_channel_info[ch_id].i2c_dev.client.adapter = NULL;
		tvp7002_channel_info[ch_id].i2c_dev.i2c_registration &= ~(0x01);
		tvp7002_channel_info[ch_id].dec_device = NULL;
	}
	return 0;
}

/* tvp7002_setcontrol : Function to set the control parameter
 */
static int tvp7002_setcontrol(struct v4l2_control *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id],
		"TVP7002 supports no control parameter to be set..\n");
	err = -EINVAL;
	return err;
}

/* tvp7002_getcontrol :
 * Function to get the control parameter
 */
static int tvp7002_getcontrol(struct v4l2_control *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id],
		"TVP7002 supports no control parameters..\n");
	err = -EINVAL;
	return err;
}

/* tvp7002_querycontrol :
 * Function to query control parameter
 */
static int tvp7002_querycontrol(struct v4l2_queryctrl *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id],
		"TVP7002 has no control parameters to return..\n");
	err = -EINVAL;
	return err;
}

/* following function is used to set THS7353 */
static int ths7353_setvalue(void)
{
	int err = 0;
	u8 val[2];
	u16 ths7353_i2c_addr = 0x5C >> 1;
	val[1] = 0x95;

	val[0] = 0x01;
	val[1] = 0x94;
	err = davinci_i2c_write(2, val, ths7353_i2c_addr);
	val[0] = 0x02;
	val[1] = 0x95;
	err |= davinci_i2c_write(2, val, ths7353_i2c_addr);
	val[0] = 0x03;
	val[1] = 0x94;
	err |= davinci_i2c_write(2, val, ths7353_i2c_addr);
	if (err) {
		printk("THS7353\n");
	}
	return err;
}

/* tvp7002_setstd :
 * Function to set the video standard
 */
static int tvp7002_setstd(v4l2_std_id * id, void *dec)
{
	int err = 0;
	struct tvp7002_format_params *tvp7002formats;
	int ch_id;
	int i = 0;
	struct v4l2_standard *standard;
	int input_idx;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}

	err |= set_cpld_for_tvp7002();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Start of tvp7002_setstd..\n");
	input_idx = tvp7002_channel_info[ch_id].params.inputidx;
	if (id == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL pointer.\n");
		return -EINVAL;
	}
	for (i = 0; i < tvp7002_configuration[ch_id].input[input_idx].
	     no_of_standard; i++) {
		standard = &tvp7002_configuration[ch_id].input[input_idx].
		    standard[i];
		if (standard->id & *id) {
			break;
		}
	}
	if (i == tvp7002_configuration[ch_id].input[input_idx].no_of_standard) {
		dev_err(tvp7002_i2c_dev[ch_id], "Invalid id...\n");
		return -EINVAL;
	}

	ths7353_setvalue();

	tvp7002formats =
	    &tvp7002_configuration[ch_id].input[input_idx].format[i];

	err = tvp7002_set_format_params(tvp7002formats, dec);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "Set standard failed\n");
		return err;
	}

	/* Lock the structure variable and assign std to the member
	   variable */
	tvp7002_channel_info[ch_id].params.std = *id;

	dev_dbg(tvp7002_i2c_dev[ch_id], "End of tvp7002 set standard...\n");
	return err;
}

/* tvp7002_getstd :
 * Function to get the video standard
 */
static int tvp7002_getstd(v4l2_std_id * id, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Starting getstd function.\n");
	if (id == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}

	/* Read the video standard */
	*id = tvp7002_channel_info[ch_id].params.std;
	dev_dbg(tvp7002_i2c_dev[ch_id], "End of getstd function.\n");
	return err;
}

/* tvp7002_querystd :
 * Function to return standard detected by decoder
 */
static int tvp7002_querystd(v4l2_std_id * id, void *dec)
{
	int err = 0;
	unsigned char val;
	unsigned short val1;
	int ch_id;
	unsigned char val_t;
	if (NULL == dec) {
		printk("NULL Pointer.\n");
		return -EINVAL;
	}

	err |= set_cpld_for_tvp7002();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Starting querystd function...\n");
	if (id == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}

	mdelay(100);
	/* Query the standards */
	err = tvp7002_i2c_read_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				   client,
				   TVP7002_LINES_PER_FRAME_STATUS_LOW, &val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id],
			"I2C read fails...Lines per frame low\n");
		return err;
	}
	val1 = val;
	err = tvp7002_i2c_read_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				   client,
				   TVP7002_LINES_PER_FRAME_STATUS_HIGH, &val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id],
			"I2C read fails...Lines per frame high\n");
		return err;
	}
	val1 |= (val << LINES_PER_FRAME_MSB_SHIFT) & LINES_PER_FRAME_MSB_MASK;
	val = (val & VIDEO_DETECTION_MASK) >> VIDEO_DETECTION_SHIFT;

	err = tvp7002_i2c_read_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				   client,
				   TVP7002_CLOCK_PER_LINE_STATUS_LSB, &val_t);

	if ((INTERLACED_VIDEO == val)
	    && (TVP7002_LINES_1080_60 == val1) && (val_t > 180 && val_t < 190)) {
		*id = V4L2_STD_1080I_60;
	} else if ((INTERLACED_VIDEO == val)
		   && (TVP7002_LINES_1080_50 == val1)) {
		*id = V4L2_STD_1080I_50;
	} else if ((PROGRESSIVE_VIDEO == val)
		   && (TVP7002_LINES_720 == val1) &&
		   (val_t > 160 && val_t < 170)) {
		*id = V4L2_STD_720P_50;
	} else if ((PROGRESSIVE_VIDEO == val)
		   && (TVP7002_LINES_720 == val1)) {
		*id = V4L2_STD_720P_60;
	} else if ((PROGRESSIVE_VIDEO == val)
		   && (525 == val1)) {
		*id = V4L2_STD_525P_60;
	} else if ((PROGRESSIVE_VIDEO == val)
		   && (625 == val1)) {
		*id = V4L2_STD_625P_50;
	} else {
		return -EINVAL;
	}

	tvp7002_channel_info[ch_id].params.std = *id;
	err = tvp7002_setstd(id, dec);
	dev_dbg(tvp7002_i2c_dev[ch_id], "End of querystd function.\n");
	return err;
}

/* tvp7002_enumstd : Function to enumerate standards supported
 */
static int tvp7002_enumstd(struct v4l2_standard *std, void *dec)
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
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}
	index = std->index;
	index1 = index;
	/* Check for valid value of index */
	for (input_idx = 0;
	     input_idx < tvp7002_configuration[ch_id].no_of_inputs;
	     input_idx++) {
		sumstd += tvp7002_configuration[ch_id].input[input_idx]
		    .no_of_standard;
		if (index < sumstd) {
			sumstd -= tvp7002_configuration[ch_id]
			    .input[input_idx].no_of_standard;
			break;
		}
	}
	if (input_idx == tvp7002_configuration[ch_id].no_of_inputs)
		return -EINVAL;
	index -= sumstd;

	memset(std, 0, sizeof(*std));

	memcpy(std, &tvp7002_configuration[ch_id].input[input_idx].
	       standard[index], sizeof(struct v4l2_standard));
	std->index = index1;
	return err;
}

/* tvp7002_setinput :
 * Function to set the input
 */
static int tvp7002_setinput(int *index, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Start of set input function.\n");

	/* check for null pointer */
	if (index == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}
	if ((*index >= tvp7002_configuration[ch_id].no_of_inputs)
	    || (*index < 0)) {
		return -EINVAL;
	}
	if (*index == 0) {	/* HD */
		tvp7002_channel_info[ch_id].params.inputidx = *index;
	} else {
		dev_err(tvp7002_i2c_dev[ch_id], "Invalid index.\n");
		return -EINVAL;
	}
	dev_dbg(tvp7002_i2c_dev[ch_id], "End of set input function.\n");
	return err;
}

/* tvp7002_getinput : Function to get the input
 */
static int tvp7002_getinput(int *index, void *dec)
{
	int err = 0;
	int ch_id;
	v4l2_std_id id;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Start of get input function.\n");

	/* check for null pointer */
	if (index == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}
	err |= tvp7002_querystd(&id, dec);
	if (err < 0) {
		return err;
	}
	*index = 0;
	*index = tvp7002_channel_info[ch_id].params.inputidx;
	dev_dbg(tvp7002_i2c_dev[ch_id], "End of get input function.\n");
	return err;
}

/* tvp7002_enuminput :
 * Function to enumerate the input
 */
static int tvp7002_enuminput(struct v4l2_input *input, void *dec)
{
	int err = 0;
	int index = 0;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;

	/* check for null pointer */
	if (input == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}

	/* Only one input is available */
	if (input->index >= tvp7002_configuration[ch_id].no_of_inputs) {
		return -EINVAL;
	}
	index = input->index;
	memset(input, 0, sizeof(*input));
	input->index = index;
	memcpy(input,
	       &tvp7002_configuration[ch_id].input[index].input_info,
	       sizeof(struct v4l2_input));
	return err;
}

/* tvp7002_set_format_params :
 * Function to set the format parameters
 */
static int tvp7002_set_format_params(struct tvp7002_format_params
				     *tvpformats, void *dec)
{
	int err = 0;
	unsigned char val;
	int ch_id;
	if (NULL == dec) {
		printk("NULL Pointer.\n");
		return -EINVAL;
	}

	err |= set_cpld_for_tvp7002();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id],
		"Tvp7002 set format params started...\n");
	if (tvpformats == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "NULL Pointer.\n");
		return -EINVAL;
	}

	/* Write the HPLL related registers */
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_HPLL_DIVIDER_MSB,
				    tvpformats->hpll_divider_msb);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id],
			"I2C write fails...Divider MSB\n");
		return err;
	}

	val = ((tvpformats->
		hpll_divider_lsb & HPLL_DIVIDER_LSB_MASK) <<
	       HPLL_DIVIDER_LSB_SHIFT);
	err =
	    tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				  client, TVP7002_HPLL_DIVIDER_LSB, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id],
			"I2C write fails...Divider LSB.\n");
		return err;
	}
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_HPLL_CONTROL,
				    tvpformats->hpll_control);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_AVID_START_PIXEL_LOW,
				    tvpformats->avid_start_lsb);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_AVID_START_PIXEL_HIGH,
				    tvpformats->avid_start_msb);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_AVID_STOP_PIXEL_LOW,
				    tvpformats->avid_stop_lsb);
	err =
	    tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				  client, TVP7002_AVID_STOP_PIXEL_HIGH,
				  tvpformats->avid_stop_msb);
	err =
	    tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				  client, TVP7002_VBLK_FIELD0_START_OFFSET,
				  tvpformats->vblk_start_f0_line_offset);
	err =
	    tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				  client, TVP7002_VBLK_FIELD1_START_OFFSET,
				  tvpformats->vblk_start_f1_line_offset);
	err =
	    tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				  client, TVP7002_VBLK_FIELD0_DURATION,
				  tvpformats->vblk_f0_duration);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_VBLK_FIELD1_DURATION,
				    tvpformats->vblk_f1_duration);

	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_ALC_PLACEMENT,
				    tvpformats->alc_placement);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_CLAMP_START,
				    tvpformats->clamp_start);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_CLAMP_WIDTH,
				    tvpformats->clamp_width);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_HPLL_PRE_COAST,
				    tvpformats->hpll_pre_coast);
	err = tvp7002_i2c_write_reg(&tvp7002_channel_info[ch_id].i2c_dev.
				    client, TVP7002_HPLL_POST_COAST,
				    tvpformats->hpll_post_coast);

	tvp7002_channel_info[ch_id].params.format = *tvpformats;

	dev_dbg(tvp7002_i2c_dev[ch_id],
		"End of tvp7002 set format params...\n");
	return err;
}

/* tvp7002_setparams : This function will set parameters for tvp7002
 */
static int tvp7002_setparams(void *params, void *dec)
{
	int err = 0;
	unsigned char val;
	int ch_id;
	struct i2c_client *ch_client;
	tvp7002_params tvp7002params;

	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	if (copy_from_user(&tvp7002params, (tvp7002_params *) params,
			   sizeof(tvp7002params))) {
		return -EFAULT;
	}
	err |= set_cpld_for_tvp7002();
	if (err) {
		printk("Failed to set cpld bit.\n");
		return -EINVAL;
	}

	ch_id = ((struct decoder_device *)dec)->channel_id;
	ch_client = &tvp7002_channel_info[ch_id].i2c_dev.client;
	dev_dbg(tvp7002_i2c_dev[ch_id],
		"Start of tvp7002 set params function.\n");

	/* check for null pointer */
	err |= tvp7002_setinput(&(tvp7002params.inputidx), dec);
	if (err < 0) {
		dev_dbg(tvp7002_i2c_dev[ch_id],
			"Set format parameters failed.\n");
		return err;
	}
	err |= tvp7002_setstd(&(tvp7002params.std), dec);
	if (err < 0) {
		dev_dbg(tvp7002_i2c_dev[ch_id],
			"Set format parameters failed.\n");
		return err;
	}

	/* set video format related parameters */
	err = tvp7002_set_format_params(&tvp7002params.format, dec);
	if (err < 0) {
		dev_dbg(tvp7002_i2c_dev[ch_id],
			"Set format parameters failed.\n");
		return err;
	}

	/* Write the gain information */
	err = tvp7002_i2c_write_reg(ch_client, TVP7002_BLUE_FINE_GAIN,
				    tvp7002params.gain.blue_fine_gain);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	err = tvp7002_i2c_write_reg(ch_client, TVP7002_GREEN_FINE_GAIN,
				    tvp7002params.gain.green_fine_gain);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	err = tvp7002_i2c_write_reg(ch_client, TVP7002_RED_FINE_GAIN,
				    tvp7002params.gain.red_fine_gain);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}

	/* Write coarse gain information */
	val = 0;
	val = tvp7002params.gain.blue_coarse_gain & BLUE_COARSE_GAIN_MASK;
	tvp7002params.gain.blue_coarse_gain = val;

	val |= ((tvp7002params.gain.green_coarse_gain &
		 GREEN_COARSE_GAIN_MASK) << GREEN_COARSE_GAIN_SHIFT);
	tvp7002params.gain.green_coarse_gain =
	    tvp7002params.gain.green_coarse_gain & GREEN_COARSE_GAIN_MASK;

	err = tvp7002_i2c_write_reg(ch_client,
				    TVP7002_BLUE_GREEN_COARSE_GAIN, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}

	val = tvp7002params.gain.red_coarse_gain & RED_COARSE_GAIN_MASK;
	tvp7002params.gain.red_coarse_gain = val;
	err = tvp7002_i2c_write_reg(ch_client, TVP7002_RED_COARSE_GAIN, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}

	/*Write the offset value in register */
	err = tvp7002_i2c_write_reg(ch_client,
				    TVP7002_BLUE_FINE_OFFSETMSB,
				    tvp7002params.offset.blue_fine_offset);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	err = tvp7002_i2c_write_reg(ch_client,
				    TVP7002_GREEN_FINE_OFFSETMSB,
				    tvp7002params.offset.green_fine_offset);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	err = tvp7002_i2c_write_reg(ch_client, TVP7002_RED_FINE_OFFSETMSB,
				    tvp7002params.offset.red_fine_offset);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	val = 0;
	val = tvp7002params.offset.blue_fine_offset_lsb & FINE_OFFSET_LSB_MASK;
	tvp7002params.offset.blue_fine_offset_lsb = val;

	val |= ((tvp7002params.
		 offset.green_fine_offset_lsb & FINE_OFFSET_LSB_MASK) <<
		FINE_OFFSET_LSB_SHIFT_GREEN);
	tvp7002params.offset.green_fine_offset_lsb =
	    (tvp7002params.offset.green_fine_offset_lsb & FINE_OFFSET_LSB_MASK);

	val |= ((tvp7002params.
		 offset.red_fine_offset_lsb & FINE_OFFSET_LSB_MASK) <<
		FINE_OFFSET_LSB_SHIFT_RED);
	tvp7002params.offset.red_fine_offset_lsb =
	    (tvp7002params.offset.red_fine_offset_lsb & FINE_OFFSET_LSB_MASK);

	err = tvp7002_i2c_write_reg(ch_client, TVP7002_FINE_OFFSET_LSBS, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	val = tvp7002params.offset.blue_coarse_offset & COARSE_OFFSET_MASK;
	tvp7002params.offset.blue_coarse_offset = val;

	err = tvp7002_i2c_write_reg(ch_client, TVP7002_BLUE_COARSE_OFFSET, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	val = tvp7002params.offset.green_coarse_offset & COARSE_OFFSET_MASK;
	tvp7002params.offset.green_coarse_offset = val;

	err =
	    tvp7002_i2c_write_reg(ch_client, TVP7002_GREEN_COARSE_OFFSET, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	val = tvp7002params.offset.red_coarse_offset & COARSE_OFFSET_MASK;
	tvp7002params.offset.red_coarse_offset = val;

	err = tvp7002_i2c_write_reg(ch_client, TVP7002_RED_COARSE_OFFSET, val);
	if (err < 0) {
		dev_err(tvp7002_i2c_dev[ch_id], "I2C write fails...\n");
		return err;
	}
	if (tvp7002params.alc.alc_enable) {
		err =
		    tvp7002_i2c_write_reg(ch_client, TVP7002_ALC_ENABLE, 0x80);
	} else {
		err =
		    tvp7002_i2c_write_reg(ch_client, TVP7002_ALC_ENABLE, 0x00);
	}
	val = (tvp7002params.alc.vcoeff << TVP7002_ALC_VCOEFF_SHIFT) |
	    (tvp7002params.alc.hcoeff);
	err = tvp7002_i2c_write_reg(ch_client, TVP7002_ALC_FILTER, val);
	if (err < 0)
		return err;

	tvp7002_channel_info[ch_id].params = tvp7002params;
	dev_dbg(tvp7002_i2c_dev[ch_id], "End of configTVP7002...\n");
	return err;
}

/* tvp7002_getparams : This function will get parameters for tvp7002
 */
static int tvp7002_getparams(void *params, void *dec)
{
	int ch_id;
	int err = 0;
	tvp7002_params *tvp7002params = (tvp7002_params *) params;
	if (NULL == dec) {
		printk("NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	dev_dbg(tvp7002_i2c_dev[ch_id], "Starting tvp7002_getparams\n");

	/* check for null pointer */
	if (tvp7002params == NULL) {
		dev_err(tvp7002_i2c_dev[ch_id], "Null pointer\n");
		return -EINVAL;
	}
	if (copy_to_user(tvp7002params, &(tvp7002_channel_info[ch_id].params),
			 sizeof(*tvp7002params))) {
		return -EFAULT;
	}

	dev_dbg(tvp7002_i2c_dev[ch_id], "End of getparamsTVP7002...\n");
	return err;
}

/* tvp7002_i2c_read_reg :This function is used to read value from register
 * for i2c client.
 */
static int tvp7002_i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val)
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

/* tvp7002_i2c_write_reg :This function is used to write value into register
 * for i2c client.
 */
static int tvp7002_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
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

/* tvp7002_i2c_attach_client : This function is used to attach i2c client
 */
static int tvp7002_i2c_attach_client(struct i2c_client *client,
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

/* tvp7002_i2c_detach_client : This function is used to detach i2c client
 */
static int tvp7002_i2c_detach_client(struct i2c_client *client)
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

/* tvp7002_i2c_probe_adapter : This function is used to probe i2c adapter
 */
static int tvp7002_i2c_probe_adapter(struct i2c_adapter *adap)
{
	int err = 0;
	tvp7002_i2c_dev[0] = &(adap->dev);
	dev_dbg(tvp7002_i2c_dev[0], "Tvp7002 i2c probe adapter called...\n");

	/* Attach the client */
	err = tvp7002_i2c_attach_client(&tvp7002_channel_info[0].i2c_dev.
					client,
					&tvp7002_channel_info[0].i2c_dev.
					driver, adap,
					tvp7002_channel_info[0].i2c_dev.
					i2c_addr);
	dev_dbg(tvp7002_i2c_dev[0], "Tvp7002 i2c probe adapter ends...\n");
	return err;
}

/* tvp7002_i2c_init : This function is used initialize tvp7002 i2c client
 */
static int tvp7002_i2c_init(void)
{
	int err = 0;
	int i = 0, j = 0;

	/* Take instance of driver */
	struct i2c_driver *driver;
	char strings[TVP7002_NUM_CHANNELS][80] =
	    { "TVP channel0 Video Decoder I2C driver"
	};
	for (i = 0; i < TVP7002_NUM_CHANNELS; i++) {
		driver = &tvp7002_channel_info[i].i2c_dev.driver;
		driver->owner = THIS_MODULE;
		strlcpy(driver->name, strings[i], sizeof(strings[i]));
		driver->id = I2C_DRIVERID_EXP0;
		driver->flags = I2C_DF_NOTIFY;
		if (0 == i) {
			driver->attach_adapter = tvp7002_i2c_probe_adapter;
		}
		driver->detach_client = tvp7002_i2c_detach_client;
		err = vpif_register_decoder(&tvp7002_dev[i]);
		if (err < 0) {
			for (j = i - 1; j > 0; j--) {
				vpif_unregister_decoder(&tvp7002_dev[j]);
			}
			return err;
		}
	}
	return err;
}

/* tvp7002_i2c_cleanup : This function is used detach tvp7002 i2c client
 */
static void tvp7002_i2c_cleanup(void)
{
	int i;
	for (i = 0; i < TVP7002_NUM_CHANNELS; i++) {
		if (tvp7002_channel_info[i].i2c_dev.i2c_registration & 0x01) {
			i2c_del_driver(&tvp7002_channel_info[i].i2c_dev.driver);
			tvp7002_channel_info[i].i2c_dev.client.adapter = NULL;
			tvp7002_channel_info[i].i2c_dev.i2c_registration = 0;
		}
		vpif_unregister_decoder(&tvp7002_dev[i]);
	}
}

module_init(tvp7002_i2c_init);
module_exit(tvp7002_i2c_cleanup);
MODULE_LICENSE("GPL");
