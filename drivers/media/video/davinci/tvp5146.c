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
/* tvp5146.c */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/device.h>
#include <media/davinci/tvp5146.h>

static struct i2c_client tvp5146_i2c_client;
static struct i2c_driver tvp5146_i2c_driver;
static int tvp5146_i2c_registration = 0;
struct device *tvp5146_i2c_dev;

static int i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val);
static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);

static int configtvp5146(void *arg);
static int clrtvp5146lostlock(void);
static int enabletvp5146agc(int arg);
static int getctrl(void *arg);
static int gettvp5146status(void *arg);
static int powerdowntvp5146(int powerdownenable);
static int queryctrl(void *arg);
static int resettvp5146(void);
static int setctrl(void *arg);
static int settvp5146amuxmode(int mode);
static int settvp5146brightness(int arg);
static int settvp5146contrast(int arg);
static int settvp5146hue(int arg);
static int settvp5146saturation(int arg);
static int settvp5146std(int arg);
static int setup656sync(tvp5146_params * tvp5146params);
static int enable_ccdc2tvp5146(struct i2c_client *client);

#define IS_DM355 (1)

/*
 * ======== tvp5146_init  ========
 */
/* This function is used initialize TVP5146 i2c client */
static int tvp5146_init(void)
{
	int err;
	struct i2c_driver *driver = &tvp5146_i2c_driver;

	err = i2c_add_driver(driver);
	if (err) {
		printk(KERN_ERR "Failed to register TVP5146 I2C client.\n");
	} else {
		tvp5146_i2c_registration = TVP5146_I2C_REGISTERED;
	}
	if (IS_DM355)
		enable_ccdc2tvp5146(&tvp5146_i2c_client);
	return err;
}

/*
 * ======== tvp5146_cleanup  ========
 */
/* This function is used detach TVP5146 i2c client */
static void tvp5146_cleanup(void)
{
	struct i2c_driver *driver = &tvp5146_i2c_driver;
	if (tvp5146_i2c_registration) {
		i2c_detach_client(&tvp5146_i2c_client);
		i2c_del_driver(driver);
		tvp5146_i2c_client.adapter = NULL;
		tvp5146_i2c_registration = TVP5146_I2C_UNREGISTERED;
	}
}

/*
 * ======== configtvp5146 ========
 */
/*This function will configure TVP5146 as per arguments passed*/
static int configtvp5146(void *arg)
{
	tvp5146_params *tvp5146params = (tvp5146_params *) arg;
	int ret = 0;
	dev_dbg(tvp5146_i2c_dev, "\nStarting configtvp5146...");
	ret |= setup656sync(tvp5146params);
	ret |= settvp5146amuxmode(tvp5146params->amuxmode);
	ret |= settvp5146std(tvp5146params->mode);
	dev_dbg(tvp5146_i2c_dev, "\nEnd of configtvp5146...");
	return ret;
}

/*
 * ======== clrtvp5146lostlock  ========
 */
 /*This function is used to clear lost lock bit in TVP5146 register. */
static int clrtvp5146lostlock(void)
{
	int ret = 0;
	u8 clr = 1;
	dev_dbg(tvp5146_i2c_dev, "\nStarting clrtvp5146lostlock...");
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x39, clr);
	dev_dbg(tvp5146_i2c_dev, "\nEnd clrtvp5146lostlock...");
	return ret;
}

/*
 * ========  enabletvp5146agc ========
 */
 /* This function is used to enable automatic gain control in TVP5146 */
static int enabletvp5146agc(int arg)
{
	int ret = 0;
	int agc;
	dev_dbg(tvp5146_i2c_dev, "\nStarting enabletvp5146agc...");
	if (arg == TRUE) {
		agc = 0xF;
	} else {
		agc = 0xC;
	}
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x01, agc);
	dev_dbg(tvp5146_i2c_dev, "\nEnd enabletvp5146agc...");
	return ret;
}

/*
 * ========  gettvpctrl ========
 */
 /* This function is used to get control value for different control commands */
static int getctrl(void *arg)
{
	struct v4l2_control *ctrl = arg;
	int ret = 0;
	u8 value;

	dev_dbg(tvp5146_i2c_dev, "\nStarting getctrl of TVP5146...");
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x09, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_CONTRAST:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x0A, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_SATURATION:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x0B, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_HUE:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x0C, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_AUTOGAIN:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x01, &value);
		if ((value & 0x3) == 3) {
			ctrl->value = TRUE;
		} else {
			ctrl->value = FALSE;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of getctrl...");
	return ret;
}

/*
 * ========  gettvp5146std ========
 */
 /* This function returns detected TVP5146 input standard */
static int gettvp5146std(tvp5146_mode * mode)
{
	int ret = 0;
	u8 output1;
	u8 std;
	u8 lock_status;

	dev_dbg(tvp5146_i2c_dev, "\nStarting of gettvp5146std...");
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x2, &std);
	std &= 0x7;
	if (std == TVP5146_MODE_AUTO) {
		ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3F, &std);
	}
	std &= 0x7;
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x33, &output1);
	*mode = std | ((output1 & 0x80) >> 4);	/* square pixel status */
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3A, &lock_status);
	if ((lock_status & 0xe) != 0xe) {
		/* not quite locked */
		ret = -EAGAIN;
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of gettvp5146std...");
	return ret;
}

/*
 * ========  gettvp5146status ========
 */
 /* This function gets TVP5146 configuration values */
static int gettvp5146status(void *arg)
{
	int ret = 0;
	tvp5146_status *status = (tvp5146_status *) arg;
	u8 agc, brightness, contrast, hue, saturation;
	u8 status_byte;
	u8 std;
	u8 output1;

	dev_dbg(tvp5146_i2c_dev, "\nStarting gettvp5146status...");
	ret = i2c_read_reg(&tvp5146_i2c_client, 0x01, &agc);
	if ((agc & 0x3) == 3) {
		status->agc_enable = TRUE;
	} else {
		status->agc_enable = FALSE;
	}
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x09, &brightness);
	status->brightness = brightness;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x0A, &contrast);
	status->contrast = contrast;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x0B, &saturation);
	status->saturation = saturation;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x0C, &hue);
	status->hue = hue;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3A, &status_byte);
	status->field_rate = (status_byte & 0x20) ? 50 : 60;
	status->lost_lock = (status_byte & 0x10) >> 4;
	status->csubc_lock = (status_byte & 0x8) >> 3;
	status->v_lock = (status_byte & 0x4) >> 2;
	status->h_lock = (status_byte & 0x2) >> 1;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3F, &std);
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x33, &output1);
	if (std | 0x80) {	/* auto switch mode */
		status->video_std = TVP5146_MODE_AUTO;
	} else {
		status->video_std = std;
	}
	status->video_std |= ((output1 & 0x80) >> 4);	/* square pixel status */
	dev_dbg(tvp5146_i2c_dev, "\nEnd of gettvp5146status...");
	return ret;
}

/*
 * ======== powerdowntvp5146 ========
 */
 /* This function will put TVP5146 in power down/up mode */
static int powerdowntvp5146(int powerdownenable)
{
	u8 powerdownsettings = 0x01;
	dev_dbg(tvp5146_i2c_dev, "\nStarting powerdowntvp5146...");
	/*Put _tvp5146 in power down mode */
	if (!powerdownenable) {
		powerdownsettings = 0x00;
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of powerdowntvp5146...");
	return i2c_write_reg(&tvp5146_i2c_client, 0x03, powerdownsettings);
}

/*
 * ======== resettvp5146========
 */
  /* This function will configure TVP5146 with default values */
static int resettvp5146(void)
{
	tvp5146_params tvp5146params = { 0 };
	dev_dbg(tvp5146_i2c_dev, "\nStarting resettvp5146...");

	tvp5146params.enablebt656sync = TRUE;
	tvp5146params.data_width = TVP5146_WIDTH_8BIT;

	setup656sync(&tvp5146params);

	settvp5146amuxmode(TVP5146_AMUX_COMPOSITE);
	dev_dbg(tvp5146_i2c_dev, "\nEnd of resettvp5146...");
	return powerdowntvp5146(FALSE);
}

/*
 * ======== queryctrl ========
 */
  /* This function will return parameter values for control command passed */
static int queryctrl(void *arg)
{
	struct v4l2_queryctrl *queryctrl = arg;
	int ret = 0;
	int id = queryctrl->id;

	dev_dbg(tvp5146_i2c_dev, "\nStarting queryctrl...");
	memset(queryctrl, 0, sizeof(*queryctrl));
	queryctrl->id = id;
	switch (id) {
	case V4L2_CID_BRIGHTNESS:
		strcpy(queryctrl->name, "BRIGHTNESS");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = 0;
		queryctrl->maximum = 255;
		queryctrl->step = 1;
		queryctrl->default_value = 128;
		break;
	case V4L2_CID_CONTRAST:
		strcpy(queryctrl->name, "CONTRAST");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = 0;
		queryctrl->maximum = 255;
		queryctrl->step = 1;
		queryctrl->default_value = 128;
		break;

	case V4L2_CID_SATURATION:
		strcpy(queryctrl->name, "SATURATION");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = 0;
		queryctrl->maximum = 255;
		queryctrl->step = 1;
		queryctrl->default_value = 128;
		break;
	case V4L2_CID_HUE:
		strcpy(queryctrl->name, "HUE");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = -128;	/* -180 DEGREE */
		queryctrl->maximum = 127;	/* 180  DEGREE */
		queryctrl->step = 1;
		queryctrl->default_value = 0;	/* 0 DEGREE */
		break;

	case V4L2_CID_AUTOGAIN:
		strcpy(queryctrl->name, "Automatic Gain Control");
		queryctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		queryctrl->minimum = 0;
		queryctrl->maximum = 1;
		queryctrl->step = 1;
		queryctrl->default_value = 1;
		break;
	default:
		if (id < V4L2_CID_LASTP1)
			queryctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		else
			ret = -EINVAL;
		break;
	}			/* end switch (id) */
	dev_dbg(tvp5146_i2c_dev, "\nEnd of queryctrl...");
	return ret;
}

/*
 * ======== setctrl ========
 */
  /* This function will set parameter values for control command passed */
static int setctrl(void *arg)
{
	struct v4l2_control *ctrl = arg;
	int ret = 0;

	dev_dbg(tvp5146_i2c_dev, "\nStarting setctrl...");
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = settvp5146brightness(ctrl->value);
		break;
	case V4L2_CID_CONTRAST:
		ret = settvp5146contrast(ctrl->value);
		break;
	case V4L2_CID_SATURATION:
		ret = settvp5146saturation(ctrl->value);
		break;
	case V4L2_CID_HUE:
		ret = settvp5146hue(ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = enabletvp5146agc(ctrl->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of setctrl...");
	return ret;
}

/*
 * ======== settvp5146amuxmode ========
 */
 /* This function is used to configure analog interface */
static int settvp5146amuxmode(int arg)
{
	u8 input_sel;

	dev_dbg(tvp5146_i2c_dev, "\nStarting settvp5146amuxmode...");
	if (arg == TVP5146_AMUX_COMPOSITE) {	/* composite */
		input_sel = 0x05;
	} else if (arg == TVP5146_AMUX_SVIDEO) {	/* s-video */
		input_sel = 0x46;
	} else {
		return -EINVAL;
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of settvp5146amuxmode...");
	return i2c_write_reg(&tvp5146_i2c_client, 0x00, input_sel);
}

/*
 * ======== settvp5146brightness ========
 */
 /* This function is used to configure brightness */
static int settvp5146brightness(int arg)
{
	int ret = 0;
	u8 brightness = (u8) arg;
	dev_dbg(tvp5146_i2c_dev, "\nStarting settvp5146brightness...");
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x09, brightness);
	dev_dbg(tvp5146_i2c_dev, "\nEnd of settvp5146brightness...");
	return ret;
}

/*
* ======== settvp5146contrast ========
*/
 /* This function is used to configure contrast */
static int settvp5146contrast(int arg)
{
	int ret = 0;
	u8 contrast = (u8) arg;
	dev_dbg(tvp5146_i2c_dev, "\nStarting settvp5146contrast...");
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x0A, contrast);
	dev_dbg(tvp5146_i2c_dev, "\nEnd of settvp5146contrast...");
	return ret;
}

/*
* ======== settvp5146hue ========
*/
 /* This function is used to configure hue value */
static int settvp5146hue(int arg)
{
	int ret = 0;
	u8 hue = (u8) arg;
	dev_dbg(tvp5146_i2c_dev, "\nStarting settvp5146hue...");
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x0C, hue);
	dev_dbg(tvp5146_i2c_dev, "\nEnd of settvp5146hue...");
	return ret;
}

/*
* ======== settvp5146saturation ========
*/
 /* This function is used to configure saturation value */
static int settvp5146saturation(int arg)
{
	int ret = 0;
	u8 saturation = (u8) arg;
	dev_dbg(tvp5146_i2c_dev, "\nStarting settvp5146saturation...");
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x0B, saturation);
	dev_dbg(tvp5146_i2c_dev, "\nEnd of settvp5146saturation...");
	return ret;
}

/*
* ======== settvp5146std ========
*/
/* This function is used to configure TVP5146 for video standard passed 
  *by application
  */
static int settvp5146std(int arg)
{
	int ret = 0;
	u8 std = (u8) arg & 0x7;	/* the 4th-bit is for squre pixel sampling */
	u8 output1;
	dev_dbg(tvp5146_i2c_dev, "\nStart settvp5146std...");
	/* setup the sampling rate: 601 or square pixel */
	dev_dbg(tvp5146_i2c_dev, "reading i2c registers.\n");
	ret = i2c_read_reg(&tvp5146_i2c_client, 0x33, &output1);
	output1 |= ((arg & 0x8) << 4);
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x33, output1);

	/* setup the video standard */
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x02, std);
	/* if autoswitch mode, enable all modes for autoswitch */
	if (std == TVP5146_MODE_AUTO) {
		u8 mask = 0x3F;	/* enable autoswitch for  all standards */
		ret = i2c_write_reg(&tvp5146_i2c_client, 0x04, mask);
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of settvp5146std...");
	return ret;
}

/*
 * ======== setup656sync ========
 */
 /* This function will configure TVP5146 output data formatting */
static int setup656sync(tvp5146_params * tvp5146params)
{
	int output1, output2, output3, output4;
	int output5, output6;
	int ret = 0;

	dev_dbg(tvp5146_i2c_dev, "\nStarting setup656sync...");
	if ((tvp5146params->enablebt656sync)
	    && (tvp5146params->data_width == TVP5146_WIDTH_8BIT)) {
		/* Configuration for 8-bit BT656 mode */
		output1 = 0x40;
		output4 = 0xFF;
		output5 = 0x4;
		output6 = 0;
		ret |=
		    i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT5,
				  output5);
		ret |=
		    i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT6,
				  output6);
	} else if ((!tvp5146params->enablebt656sync)
		   && (tvp5146params->data_width == TVP5146_WIDTH_8BIT)) {

		/* Configuration for 8-bit seperate sync mode */
		output1 = 0x43;
		output4 = 0xAF;
		output5 = 0x4;
		output6 = 0x1E;
		ret |=
		    i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT5,
				  output5);
		ret |=
		    i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT6,
				  output6);
	} else if ((!tvp5146params->enablebt656sync)
		   && (tvp5146params->data_width == TVP5146_WIDTH_16BIT)) {

		/* Configuration for 16-bit seperate sync mode */
		output1 = 0x41;
		output4 = 0xAF;
	} else {
		return -EINVAL;
	}

	output2 = 0x11;		/* enable clock, enable Y[9:0] */
	output3 = 0x0;

	ret |= i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT1, output1);
	ret |= i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT2, output2);
	ret |= i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT3, output3);
	ret |= i2c_write_reg(&tvp5146_i2c_client, TVP5146_OUTPUT4, output4);

	dev_dbg(tvp5146_i2c_dev, "\nEnd of setup656sync...");

	return ret;
}

/*
 * ======== tvp5146_ctrl ========
 */
 /* This function handles all TVP5146 control commands */
int tvp5146_ctrl(tvp5146_cmd cmd, void *arg)
{
	int ret = 0;
	dev_dbg(tvp5146_i2c_dev,
		"\nStarting tvp5146_ctrl with %d command...", cmd);
	switch (cmd) {
	case TVP5146_CONFIG:
		ret = configtvp5146(arg);
		break;
	case TVP5146_RESET:
		ret = resettvp5146();
		break;
	case TVP5146_POWERDOWN:
		ret = powerdowntvp5146(*(int *)arg);
		break;
	case TVP5146_SET_AMUXMODE:
		ret = settvp5146amuxmode(*(int *)arg);
		break;
	case TVP5146_SET_BRIGHTNESS:
		ret = settvp5146brightness(*(int *)arg);
		break;
	case TVP5146_SET_CONTRAST:
		ret = settvp5146contrast(*(int *)arg);
		break;
	case TVP5146_SET_HUE:
		ret = settvp5146hue(*(int *)arg);
		break;
	case TVP5146_SET_SATURATION:
		ret = settvp5146saturation(*(int *)arg);
		break;
	case TVP5146_SET_AGC:
		ret = enabletvp5146agc(*(int *)arg);
		break;
	case TVP5146_SET_VIDEOSTD:
		ret = settvp5146std(*(int *)arg);
		break;
	case TVP5146_CLR_LOSTLOCK:
		ret = clrtvp5146lostlock();
		break;
	case TVP5146_GET_STATUS:
		ret = gettvp5146status(arg);
		break;
	case TVP5146_GET_STD:
		ret = gettvp5146std(arg);
		break;
	case VIDIOC_QUERYCTRL:
		ret = queryctrl(arg);
		break;
	case VIDIOC_G_CTRL:
		ret = getctrl(arg);
		break;
	case VIDIOC_S_CTRL:
		ret = setctrl(arg);
		break;
	case TVP5146_INIT:
		ret = tvp5146_init();
		break;
	case TVP5146_CLEANUP:
		tvp5146_cleanup();
		break;
	default:
		ret = -EINVAL;
	}
	dev_dbg(tvp5146_i2c_dev, "\nEnd of tvp5146_ctrl...");
	return ret;
}

/*
 * ======== i2c_read_reg  ========
 */
/*This function is used to read value from register for i2c client. */
static int i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[1];
	dev_dbg(tvp5146_i2c_dev, "\nStarting tvp5146 i2c read...");
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
	dev_dbg(tvp5146_i2c_dev, "\nEnd of tvp5146 i2c read...");
	return err;
}

/*
 * ======== i2c_write_reg  ========
 */
/*This function is used to write value into register for i2c client. */
static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[2];

	dev_dbg(tvp5146_i2c_dev, "\nEnd of tvp5146 i2c write...");
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
	dev_dbg(tvp5146_i2c_dev, " i2c data write \n");

	return err;
}

/*This function is used to write value into register for i2c client. */
static int enable_ccdc2tvp5146(struct i2c_client *client)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter) {
		err = -ENODEV;
	} else {
		msg->addr = 0x25;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;
		data[0] = 0x8;
		data[1] = 0x0;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	dev_dbg(tvp5146_i2c_dev, " i2c data write \n");

	return err;
}

/*
 * ======== _i2c_attach_client  ========
 */
/* This function is used to attach i2c client */
static int _i2c_attach_client(struct i2c_client *client,
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

/*
 * ======== _i2c_detach_client  ========
 */
/* This function is used to detach i2c client */
static int _i2c_detach_client(struct i2c_client *client)
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

/*
 * ======== tvp5146_i2c_probe_adapter  ========
 */
/* This function is used to probe i2c adapter */
static int tvp5146_i2c_probe_adapter(struct i2c_adapter *adap)
{
	tvp5146_i2c_dev = &(adap->dev);
	return _i2c_attach_client(&tvp5146_i2c_client, &tvp5146_i2c_driver,
				  adap, TVP5146_I2C_ADDR);
}

/*
 * ======== tvp5146_i2c_init  ========
 */
/* This function is used initialize TVP5146 i2c client */
static int tvp5146_i2c_init(void)
{
	int err = 0;
	struct i2c_driver *driver = &tvp5146_i2c_driver;

	driver->owner = THIS_MODULE;
	strlcpy(driver->name, "TVP5146 Video Decoder I2C driver",
		sizeof(driver->name));
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = tvp5146_i2c_probe_adapter;
	driver->detach_client = _i2c_detach_client;

	return err;
}

/*
 * ======== tvp5146_i2c_cleanup  ========
 */
/* This function is used detach TVP5146 i2c client */
static void tvp5146_i2c_cleanup(void)
{
	struct i2c_driver *driver = &tvp5146_i2c_driver;

	if (tvp5146_i2c_registration) {
		i2c_detach_client(&tvp5146_i2c_client);
		i2c_del_driver(driver);
		tvp5146_i2c_client.adapter = NULL;
		tvp5146_i2c_registration = TVP5146_I2C_UNREGISTERED;
	}
}

module_init(tvp5146_i2c_init);
module_exit(tvp5146_i2c_cleanup);

EXPORT_SYMBOL(tvp5146_ctrl);
MODULE_LICENSE("GPL");

/**************************************************************************/
/* End of file                                                            */
/**************************************************************************/
