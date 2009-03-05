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
/* ths8200_encoder.c */

/* Kernel Specific header files */
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <media/davinci/ths8200_encoder.h>

/* Function prototypes */
static int ths8200_initialize(struct vid_encoder_device *enc, int flag);
static int ths8200_deinitialize(struct vid_encoder_device *enc);

static int ths8200_setmode(struct vid_enc_mode_info *mode_info,
			   struct vid_encoder_device *enc);
static int ths8200_getmode(struct vid_enc_mode_info *mode_info,
			   struct vid_encoder_device *enc);

static int ths8200_setoutput(char *output, struct vid_encoder_device *enc);
static int ths8200_getoutput(char *output, struct vid_encoder_device *enc);
static int ths8200_enumoutput(int index,
			      char *output, struct vid_encoder_device *enc);

static int ths8200_i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val);
static int ths8200_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);

/* i2c function prototypes */
static int ths8200_i2c_attach_client(struct i2c_client *,
				     struct i2c_driver *,
				     struct i2c_adapter *, int);
static int ths8200_i2c_detach_client(struct i2c_client *);
static int ths8200_i2c_probe_adapter(struct i2c_adapter *);
static int ths8200_init(void);
static void ths8200_cleanup(void);
static int ths8200_soft_reset(struct vid_encoder_device *enc);
static int ths8200_encoder_enable(int flag, struct vid_encoder_device *enc);

static struct vid_enc_mode_info
    ths8200_component_standards[THS8200_COMPONENT_NUM_STD] = {
	{
	 .name = VID_ENC_STD_720P_60,
	 .std = 1,
	 .if_type = VID_ENC_IF_YCC16,	/* TBD */
	 .interlaced = 0,
	 .xres = 1280,
	 .yres = 720,
	 .fps = {60, 1},
	 .left_margin = 300,
	 .right_margin = 70,
	 .upper_margin = 26,
	 .lower_margin = 3,
	 .hsync_len = 80,
	 .vsync_len = 5,
	 .flags = 0},
	{
	 .name = VID_ENC_STD_1080I_30,
	 .std = 1,
	 .if_type = VID_ENC_IF_YCC16,	/* TBD */
	 .interlaced = 1,
	 .xres = 1920,
	 .yres = 1080,
	 .fps = {30, 1},
	 .left_margin = 200,
	 .right_margin = 80,
	 .upper_margin = 13,
	 .lower_margin = 31,
	 .hsync_len = 88,
	 .vsync_len = 5,
	 .flags = 0},
};

#define THS8200_MAX_REGISTERS 40
static struct ths8200_std_info
    ths8200_component_std_info[THS8200_COMPONENT_NUM_STD][THS8200_MAX_REGISTERS]
    = {
	{
	 {THS8200_DTG2_CNTL, THS8200_DTG2_CNTL_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_A, THS8200_DTG1_SPEC_A_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_B, THS8200_DTG1_SPEC_B_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_C, THS8200_DTG1_SPEC_C_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_D_LSB, THS8200_DTG1_SPEC_D_LSB_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_E_LSB, THS8200_DTG1_SPEC_E_LSB_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_DEH_MSB, THS8200_DTG1_SPEC_DEH_MSB_720P_DEFAULT},
	 {THS8200_DTG1_SPEC_K_LSB, THS8200_DTG1_SPEC_K_LSB_720P_DEFAULT},
	 {THS8200_DTG1_TOT_PIXELS_MSB,
	  THS8200_DTG1_TOT_PIXELS_MSB_720P_DEFAULT},
	 {THS8200_DTG1_TOT_PIXELS_LSB,
	  THS8200_DTG1_TOT_PIXELS_LSB_720P_DEFAULT},
	 {THS8200_DTG1_MODE, THS8200_DTG1_MODE_720P_DEFAULT},
	 {THS8200_DTG1_FRAME_FIELD_SZ_MSB,
	  THS8200_DTG1_FRAME_FIELD_SZ_MSB_720P_DEFAULT},
	 {THS8200_DTG1_FRAME_SZ_LSB, THS8200_DTG1_FRAME_SZ_LSB_720P_DEFAULT},
	 {THS8200_DTG1_FIELD_SZ_LSB, THS8200_DTG1_FIELD_SZ_LSB_720P_DEFAULT},
	 {THS8200_DTG2_HS_IN_DLY_LSB, THS8200_DTG2_HS_IN_DLY_LSB_720P_DEFAULT},
	 {THS8200_DTG2_VS_IN_DLY_MSB, THS8200_DTG2_VS_IN_DLY_MSB_720P_DEFAULT},
	 {THS8200_DTG2_VS_IN_DLY_LSB, THS8200_DTG2_VS_IN_DLY_LSB_720P_DEFAULT},
	 {0, 0},
	 },
	{
	 {THS8200_TST_CNTL1, THS8200_TST_CNTL1_1080I_DEFAULT},
	 {THS8200_TST_CNTL2, THS8200_TST_CNTL2_1080I_DEFAULT},
	 {THS8200_CSM_GY_CNTL_MULT_MSB,
	  THS8200_CSM_GY_CNTL_MULT_MSB_1080I_DEFAULT},
	 {THS8200_DTG2_CNTL, THS8200_DTG2_CNTL_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_A, THS8200_DTG1_SPEC_A_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_B, THS8200_DTG1_SPEC_B_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_C, THS8200_DTG1_SPEC_C_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_D1, THS8200_DTG1_SPEC_D1_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_D_LSB, THS8200_DTG1_SPEC_D_LSB_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_E_LSB, THS8200_DTG1_SPEC_E_LSB_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_DEH_MSB, THS8200_DTG1_SPEC_DEH_MSB_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_K_LSB, THS8200_DTG1_SPEC_K_LSB_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_G_LSB, THS8200_DTG1_SPEC_G_LSB_1080I_DEFAULT},
	 {THS8200_DTG1_SPEC_G_MSB, THS8200_DTG1_SPEC_G_MSB_1080I_DEFAULT},
	 {THS8200_DTG1_TOT_PIXELS_MSB,
	  THS8200_DTG1_TOT_PIXELS_MSB_1080I_DEFAULT},
	 {THS8200_DTG1_TOT_PIXELS_LSB,
	  THS8200_DTG1_TOT_PIXELS_LSB_1080I_DEFAULT},
	 {THS8200_DTG1_MODE, THS8200_DTG1_MODE_1080I_DEFAULT},
	 {THS8200_DTG1_FRAME_FIELD_SZ_MSB,
	  THS8200_DTG1_FRAME_FIELD_SZ_MSB_1080I_DEFAULT},
	 {THS8200_DTG1_FRAME_SZ_LSB, THS8200_DTG1_FRAME_SZ_LSB_1080I_DEFAULT},
	 {THS8200_DTG1_FIELD_SZ_LSB, THS8200_DTG1_FIELD_SZ_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_HLENGTH_LSB, THS8200_DTG2_HLENGTH_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_HLENGTH_LSB_HDLY_MSB,
	  THS8200_DTG2_HLENGTH_LSB_HDLY_MSB_1080I_DEFAULT},
	 {THS8200_DTG2_HLENGTH_HDLY_LSB,
	  THS8200_DTG2_HLENGTH_HDLY_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_VLENGTH1_LSB, THS8200_DTG2_VLENGTH1_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_VLENGTH1_MSB_VDLY1_MSB,
	  THS8200_DTG2_VLENGTH1_MSB_VDLY1_MSB_1080I_DEFAULT},
	 {THS8200_DTG2_VDLY1_LSB, THS8200_DTG2_VDLY1_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_VLENGTH2_LSB, THS8200_DTG2_VLENGTH2_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_VDLY2_LSB, THS8200_DTG2_VDLY2_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_VLENGTH2_MSB_VDLY2_MSB,
	  THS8200_DTG2_VLENGTH2_MSB_VDLY2_MSB_1080I_DEFAULT},
	 {THS8200_DTG2_VDLY1_LSB, THS8200_DTG2_VDLY1_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_HS_IN_DLY_LSB, THS8200_DTG2_HS_IN_DLY_LSB_1080I_DEFAULT},
	 {THS8200_DTG2_VS_IN_DLY_MSB, THS8200_DTG2_VS_IN_DLY_MSB_1080I_DEFAULT},
	 {THS8200_DTG2_VS_IN_DLY_LSB, THS8200_DTG2_VS_IN_DLY_LSB_1080I_DEFAULT},
	 {0, 0}
	 }
};

static struct ths8200_config ths8200_configuration = {
	.no_of_outputs = THS8200_MAX_NO_OUTPUTS,
	.output[0] = {
		      .output_name = VID_ENC_OUTPUT_COMPONENT1,
		      .no_of_standard = THS8200_COMPONENT_NUM_STD,
		      .standards = {VID_ENC_STD_720P_60, VID_ENC_STD_1080I_30},
		      .std_info = (struct ths8200_std_info *)
		      &ths8200_component_std_info,
		      },
};

static struct ths8200_channel ths8200_channel_info = {
	.params.outindex = 0,
	.params.mode = VID_ENC_STD_720P_60,
	.i2c_dev.clients[0] = {
			       .i2c_addr = THS8200_I2C_ADDR},
	.i2c_dev.clients[1] = {
			       .i2c_addr = CDCE_I2C_ADDR},
	.i2c_dev.clients[2] = {
			       .i2c_addr = THS7303_I2C_ADDR0},
	.i2c_dev.i2c_registration = 0,
	.enc_device = NULL
};

/* Global variables */
static struct device *ths8200_i2c_dev;

static struct vid_enc_output_ops ths8200_outputs_ops = {
	.count = THS8200_MAX_NO_OUTPUTS,
	.enumoutput = ths8200_enumoutput,
	.setoutput = ths8200_setoutput,
	.getoutput = ths8200_getoutput
};
static struct vid_enc_mode_ops ths8200_modes_ops = {
	.setmode = ths8200_setmode,
	.getmode = ths8200_getmode,
};

static struct vid_enc_misc_ops ths8200_miscs_ops = {
	.reset = ths8200_soft_reset,
	.enable = ths8200_encoder_enable,
};

static struct vid_encoder_device ths8200_dev = {
	.name = "THS8200_ENCODER",
	.channel_id = 0,
	.capabilities = 0,
	.initialize = ths8200_initialize,
	.mode_ops = &ths8200_modes_ops,
	.ctrl_ops = NULL,
	.output_ops = &ths8200_outputs_ops,
	.params_ops = NULL,
	.misc_ops = &ths8200_miscs_ops,
	.deinitialize = ths8200_deinitialize
};

static int ths8200_encoder_enable(int flag, struct vid_encoder_device *enc)
{

	struct i2c_client *client;
	int err = 0;
	u8 val;

	client = &ths8200_channel_info.i2c_dev.clients[THS8200].client;
	err |= ths8200_i2c_read_reg(client, THS8200_CHIP_CTL, &val);
	if (err < 0) {
		dev_err(ths8200_i2c_dev,
			"Error reading i2c register 0x%x\n", THS8200_CHIP_CTL);
		return -1;
	}
	if (flag) {
		/* power down the dac */
		val = (val & 0xf7);
		err |= ths8200_i2c_write_reg(client, THS8200_CHIP_CTL, val);
	} else {
		val |= 0x8;
		err |= ths8200_i2c_write_reg(client, THS8200_CHIP_CTL, val);
	}
	if (err < 0) {
		dev_err(ths8200_i2c_dev, "Error in writing to register 0x%x\n",
			THS8200_CHIP_CTL);
		return -1;
	}
	return 0;
}

static int ths8200_soft_reset(struct vid_encoder_device *enc)
{

	struct i2c_client *client;
	int err = 0;
	u8 val;

	client = &ths8200_channel_info.i2c_dev.clients[THS8200].client;
	err |= ths8200_i2c_read_reg(client, THS8200_CHIP_CTL, &val);
	dev_info(ths8200_i2c_dev, "Resetting THS8200 card\n");
	/* reset consists of toggling the reset bit from low to high */

	val &= 0xfe;
	err |= ths8200_i2c_write_reg(&ths8200_channel_info.i2c_dev.
				     clients[THS8200].client, THS8200_CHIP_CTL,
				     val);
	val |= 0x1;
	err |= ths8200_i2c_write_reg(&ths8200_channel_info.i2c_dev.
				     clients[THS8200].client, THS8200_CHIP_CTL,
				     val);

	return err;
}

/* This function is called by the vpif driver to initialize ADV7343 driver.
 * It initializes all registers of ths8200 with the default values
 */
static int ths8200_initialize(struct vid_encoder_device *enc, int flag)
{
	int err = 0;
	char *std, *output;
	int outindex;
	struct i2c_client *ch_client;

	dev_dbg(ths8200_i2c_dev, "ths8200_initialize\n");
	if (NULL == enc) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}

	/* Register THS8200 I2C client */
	err = i2c_add_driver(&ths8200_channel_info.i2c_dev.driver);
	if (err) {
		dev_err(ths8200_i2c_dev,
			"Failed to register THS8200 I2C client.\n");
		return -EINVAL;
	}
	ths8200_channel_info.i2c_dev.i2c_registration |= 1;
	ths8200_channel_info.enc_device = enc;

	dev_dbg(ths8200_i2c_dev, "THS8200 driver registered\n");

	/* Following sets default values to the THS8200 registers */
	err = ths8200_soft_reset(enc);

	/* Program the clock to output 74.25MHz to VPBE ext clock in */
	ch_client =
	    &ths8200_channel_info.i2c_dev.clients[CDCE_CLK_SYNTH].client;
	err |= ths8200_i2c_write_reg(ch_client, 0x16, 0x4);
	err |= ths8200_i2c_write_reg(ch_client, 0x03, 2);
	err |= ths8200_i2c_write_reg(ch_client, 0x18, 2);
	err |= ths8200_i2c_write_reg(ch_client, 0x19, 0xc0);
	err |= ths8200_i2c_write_reg(ch_client, 0x1a, 2);
	err |= ths8200_i2c_write_reg(ch_client, 0x1b, 0xc8);
	err |= ths8200_i2c_write_reg(ch_client, 0x14, 0x6f);
	if (err < 0) {
		err = -EINVAL;
		dev_err(ths8200_i2c_dev,
			"Failed to Program CDCE Clock Synthesiser\n");
		ths8200_deinitialize(enc);
		return -EINVAL;
	}

	msleep(2000);

	ch_client =
	    &ths8200_channel_info.i2c_dev.clients[THS7303_VIDEO_BUFFER].client;

	/* Configure the 7303 video buffer to enable output with HD LPF */
	err |=
	    ths8200_i2c_write_reg(ch_client, THS7303_CHANNEL_1,
				  THS7303_DEFAULT_CHANNEL_VAL);
	err |=
	    ths8200_i2c_write_reg(ch_client, THS7303_CHANNEL_2,
				  THS7303_DEFAULT_CHANNEL_VAL);
	err |=
	    ths8200_i2c_write_reg(ch_client, THS7303_CHANNEL_3,
				  THS7303_DEFAULT_CHANNEL_VAL);

	if (err < 0) {
		err = -EINVAL;
		dev_err(ths8200_i2c_dev, "Error in init code, quitting...\n");
		ths8200_deinitialize(enc);
		return err;
	} else {
		/* Configure for default video standard */
		/* call set standard */
		std = ths8200_channel_info.params.mode;
		outindex = ths8200_channel_info.params.outindex;
		output = ths8200_configuration.output[outindex].output_name;
		err |= ths8200_setoutput(output, enc);
		if (err < 0) {
			dev_err(ths8200_i2c_dev,
				"Error in init code, quitting...\n");
			err = -EINVAL;
			ths8200_deinitialize(enc);
			return err;
		}
	}
	dev_dbg(ths8200_i2c_dev, "ths8200 initialized ...\n");
	ths8200_channel_info.i2c_dev.i2c_registration |= 2;
	dev_dbg(ths8200_i2c_dev, "</ths8200_initialize>\n");
	return err;
}

static int ths8200_deinitialize(struct vid_encoder_device *enc)
{
	int i;
	if (NULL == enc) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}
	dev_dbg(ths8200_i2c_dev, "ths8200 ch deinitialization \
		 called\n");

	if (ths8200_channel_info.i2c_dev.i2c_registration & 0x01) {
		i2c_del_driver(&ths8200_channel_info.i2c_dev.driver);
		for (i = 0; i < THS8200_MAX_I2C_DEVICES; i++) {
			ths8200_channel_info.i2c_dev.clients[i].client.adapter =
			    NULL;
		}
		ths8200_channel_info.i2c_dev.i2c_registration &= ~(0x01);
		ths8200_channel_info.enc_device = NULL;
	}
	return 0;
}

/* Following function is used to set the standard */
static int ths8200_setmode(struct vid_enc_mode_info *mode_info,
			   struct vid_encoder_device *enc)
{
	int err = 0, outindex, i, std_index;
	struct i2c_client *ch_client;
	char *mode;
	u8 reg, val;

	if ((NULL == enc) || (NULL == mode_info)) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}

	mode = mode_info->name;
	if (NULL == mode) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}
	dev_dbg(ths8200_i2c_dev, "<ths8200_setmode>\n");
	ch_client = &ths8200_channel_info.i2c_dev.clients[THS8200].client;
	dev_dbg(ths8200_i2c_dev, "Start of ths8200_setmode..\n");
	outindex = ths8200_channel_info.params.outindex;

	if (mode_info->std) {

		char *mymode = NULL;
		for (std_index = 0;
		     std_index <
		     ths8200_configuration.output[outindex].no_of_standard;
		     std_index++) {
			if (!strcmp
			    (ths8200_configuration.output[outindex].
			     standards[std_index], mode)) {
				mymode =
				    ths8200_configuration.output[outindex].
				    standards[std_index];
				break;
			}
		}
		if ((std_index ==
		     ths8200_configuration.output[outindex].no_of_standard)
		    || (NULL == mymode)) {
			dev_err(ths8200_i2c_dev, "Invalid id...\n");
			return -EINVAL;
		}
		/* Store the standard in global object of ths8200 encoder */
		ths8200_channel_info.params.mode = mymode;
		dev_dbg(ths8200_i2c_dev, "Setting mode to  = %s\n",
			mode_info->name);
		ths8200_soft_reset(enc);

		for (i = THS8200_CSC_R11; i <= THS8200_CSC_OFFS3; i++) {
			/* reset color space conversion registers */
			err |= ths8200_i2c_write_reg(ch_client, i, 0x0);
		}

		/* CSC bypassed and Under overflow protection ON */
		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_CSC_OFFS3,
					     ((THS8200_CSC_BYPASS <<
					       THS8200_CSC_BYPASS_SHIFT) |
					      THS8200_CSC_UOF_CNTL));

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DATA_CNTL,
					     THS8200_DATA_CNTL_MODE_20BIT_YCBCR);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_Y_SYNC1_LSB,
					     THS8200_DTG1_CBCR_SYNC1_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_Y_SYNC2_LSB,
					     THS8200_DTG1_Y_SYNC2_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_Y_SYNC3_LSB,
					     THS8200_DTG1_Y_SYNC3_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_CBCR_SYNC1_LSB,
					     THS8200_DTG1_CBCR_SYNC1_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_CBCR_SYNC2_LSB,
					     THS8200_DTG1_CBCR_SYNC2_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_CBCR_SYNC3_LSB,
					     THS8200_DTG1_CBCR_SYNC3_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_Y_SYNC_MSB,
					     THS8200_DTG1_Y_SYNC_MSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_CBCR_SYNC_MSB,
					     THS8200_DTG1_CBCR_SYNC_MSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_SPEC_H_LSB,
					     THS8200_DTG1_SPEC_H_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_SPEC_K_MSB,
					     THS8200_DTG1_SPEC_K_MSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_FLD_FLIP_LINECNT_MSB,
					     THS8200_DTG1_FLD_FLIP_LINECNT_MSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG1_LINECNT_LSB,
					     THS8200_DTG1_LINECNT_LSB_DEFAULT);

		err |= ths8200_i2c_write_reg(ch_client,
					     THS8200_DTG2_HS_IN_DLY_MSB,
					     THS8200_DTG2_HS_IN_DLY_MSB_DEFAULT);

		i = 0;

		do {
			reg = ths8200_component_std_info[std_index][i].reg;
			val = ths8200_component_std_info[std_index][i].val;
			if (!reg)
				break;

			err |= ths8200_i2c_write_reg(ch_client, reg, val);

			if (err < 0) {
				dev_err(ths8200_i2c_dev,
					"Set mode i2c write error\n");
				break;
			}
			i++;
		}
		while (i < THS8200_MAX_REGISTERS);

		if (err < 0) {
			dev_err(ths8200_i2c_dev, "Set standard failed\n");
			return err;
		}
		ths8200_soft_reset(enc);
	} else {
		/* Non-standard mode not supported */
		return -1;
	}
	dev_dbg(ths8200_i2c_dev, "</ths8200_setmode>\n");
	return 0;
}

static struct vid_enc_mode_info *ths8200_get_modeinfo(char *mode_name)
{
	int i;
	for (i = 0; i < THS8200_MAX_NUM_STD; i++) {
		if (!strcmp(ths8200_configuration.output[0].standards[i],
			    mode_name)) {
			return &ths8200_component_standards[i];
		}
	}
	return NULL;
}

/* Following function is used to get currently selected mode .*/
static int ths8200_getmode(struct vid_enc_mode_info *mode_info,
			   struct vid_encoder_device *enc)
{
	int err = 0;
	struct vid_enc_mode_info *my_mode_info;
	if ((NULL == enc) || (NULL == mode_info)) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}
	dev_dbg(ths8200_i2c_dev, "<ths8200_getmode>\n");
	my_mode_info = ths8200_get_modeinfo(ths8200_channel_info.params.mode);
	if (NULL == my_mode_info) {
		dev_err(ths8200_i2c_dev,
			"NULL Pointer for current mode info\n");
		return -EINVAL;
	}
	memcpy(mode_info, my_mode_info, sizeof(struct vid_enc_mode_info));
	dev_dbg(ths8200_i2c_dev, "</ths8200_getmode>\n");
	return err;
}

/* Following function is used to set output format in ths8200 device. The index
   of the output format is  passed as the argument to this function. */
static int ths8200_setoutput(char *output, struct vid_encoder_device *enc)
{
	int err = 0;
	struct vid_enc_mode_info *my_mode_info;

	if ((NULL == enc) || (NULL == output)) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}
	dev_dbg(ths8200_i2c_dev, "<ths8200_setoutput>\n");
	if (strcmp(output, ths8200_configuration.output[0].output_name)) {
		dev_err(ths8200_i2c_dev, "No matching output: %s\n", output);
		return -EINVAL;
	}
	ths8200_channel_info.params.mode
	    = ths8200_configuration.output[0].standards[0];

	my_mode_info = ths8200_get_modeinfo(ths8200_channel_info.params.mode);
	if (NULL == my_mode_info) {
		dev_err(ths8200_i2c_dev, "No matching mode_info entry found\n");
		return -EINVAL;
	}
	err |= ths8200_setmode(my_mode_info, enc);
	if (err < 0) {
		dev_err(ths8200_i2c_dev, "Erron in setting default mode\n");
		return err;
	}
	dev_dbg(ths8200_i2c_dev, "</ths8200_setoutput>\n");
	return err;
}

/* Following function is used to get index of the output currently selected.*/
static int ths8200_getoutput(char *output, struct vid_encoder_device *enc)
{
	int err = 0;
	if ((NULL == enc) || (NULL == output)) {
		dev_err(ths8200_i2c_dev, "NULL Pointer\n");
		return -EINVAL;
	}
	dev_dbg(ths8200_i2c_dev, "<ths8200_getoutput>\n");
	strcpy(output, ths8200_configuration.output[0].output_name);
	dev_dbg(ths8200_i2c_dev, "</ths8200_getoutput>\n");
	return err;
}

/* Following function is used to enumerate outputs supported by the driver.
   It fills in information about the output in the outp. */
static int ths8200_enumoutput(int index, char *output,
			      struct vid_encoder_device *enc)
{
	int err = 0;
	if ((NULL == enc) || (NULL == output)) {
		dev_err(ths8200_i2c_dev, "NULL Pointer.\n");
		return -EINVAL;
	}
	/* Only one output is available */
	if (index >= ths8200_configuration.no_of_outputs) {
		return -EINVAL;
	}
	strncpy(output,
		ths8200_configuration.output[index].output_name,
		VID_ENC_NAME_MAX_CHARS);
	return err;
}

/* This function is used to read value from register using i2c client. */
static int ths8200_i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val)
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
		if (client->addr == CDCE_I2C_ADDR) {
			data[0] = (reg | 0x80);
		} else
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
	if (err < 0)
		dev_err(ths8200_i2c_dev, "i2c read error\n");
	return err;
}

/*This function is used to write value into register using i2c client. */
static int ths8200_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	u8 read_val;

	dev_dbg(ths8200_i2c_dev,
		"ths8200_i2c_write_reg, reg = %x, val = %x\n", reg, val);
	if (!client->adapter) {
		err = -ENODEV;
	} else {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;
		if (client->addr == CDCE_I2C_ADDR) {
			data[0] = (reg | 0x80);
		} else
			data[0] = reg;
		data[1] = val;
		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0) {
			ths8200_i2c_read_reg(client, reg, &read_val);

			if (read_val != val) {
				dev_err(ths8200_i2c_dev,
					"i2c write verification failed\n");
				return -1;
			}
		}

		if (err < 0) {
			dev_err(ths8200_i2c_dev, "i2c write error = %x\n", err);
			return err;
		}
	}
	return err;
}

/* This function is used to attach i2c client */
static int ths8200_i2c_attach_client(struct i2c_client *client,
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
static int ths8200_i2c_detach_client(struct i2c_client *client)
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

static int ths8200_i2c_probe_adapter(struct i2c_adapter *adap)
{
	int err = 0;
	int i;
	ths8200_i2c_dev = &(adap->dev);
	dev_dbg(ths8200_i2c_dev, "THS8200 i2c probe adapter called...\n");
	/* Attach the clients */
	for (i = 0; i < THS8200_MAX_I2C_DEVICES; i++) {
		err |=
		    ths8200_i2c_attach_client(&ths8200_channel_info.i2c_dev.
					      clients[i].client,
					      &ths8200_channel_info.i2c_dev.
					      driver, adap,
					      ths8200_channel_info.i2c_dev.
					      clients[i].i2c_addr);
	}
	dev_info(ths8200_i2c_dev, "THS8200 encoder initialized\n");
	return err;
}

/* This function used to initialize the i2c driver */
static int ths8200_init(void)
{
	int err = 0;
	/* Take instance of driver */
	struct i2c_driver *driver;
	driver = &ths8200_channel_info.i2c_dev.driver;
	driver->owner = THIS_MODULE;
	strcpy(driver->name, "THS8200 encoder I2C driver");
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = ths8200_i2c_probe_adapter;
	driver->detach_client = ths8200_i2c_detach_client;
	err |= vid_enc_register_encoder(&ths8200_dev);
	if (err < 0) {
		vid_enc_unregister_encoder(&ths8200_dev);
		return err;
	}
	return err;
}

/* Function used to cleanup i2c driver */
static void ths8200_cleanup(void)
{
	int j;
	if (ths8200_channel_info.i2c_dev.i2c_registration & 0x01) {
		i2c_del_driver(&ths8200_channel_info.i2c_dev.driver);
		for (j = 0; j < THS8200_MAX_I2C_DEVICES; j++) {
			ths8200_channel_info.i2c_dev.clients[j].client.adapter =
			    NULL;
		}
		ths8200_channel_info.i2c_dev.i2c_registration = 0;
		vid_enc_unregister_encoder(&ths8200_dev);
	}
}

subsys_initcall(ths8200_init);
module_exit(ths8200_cleanup);

MODULE_LICENSE("GPL");
