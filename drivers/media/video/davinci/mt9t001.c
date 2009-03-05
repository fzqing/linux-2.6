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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

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
#include <media/davinci/mt9t001.h>

#define MT9T001_I2C_ENABLE 1
/* i2c global variable for mt9t001*/
static struct i2c_client mt9t001_i2c_client;
static struct i2c_driver mt9t001_i2c_driver;
static int mt9t001_i2c_registration = 0;
struct device *mt9t001_i2c_dev;
static int CONFIG_PCA9543A = 0;

/*	Function prototype*/
int mt9t001_ctrl(unsigned int cmd, void *arg, void *params);
static int mt9t001_init(void *arg, void **params);
static int mt9t001_cleanup(void *params);
static int mt9t001_configpca9543a(void);
static int mt9t001_setstd(void *arg, void *params);
static int mt9t001_setformat(struct mt9t001_format_params *mt9tformats);
static int mt9t001_getformat(struct mt9t001_format_params *mt9tformats);
static int mt9t001_queryctrl(void *arg);
static int mt9t001_setgain(int arg);
static int mt9t001_getgain(int *arg);
static int mt9t001_setparams(void *arg);
static int mt9t001_getparams(void *arg);

/*i2c function proto types*/
static int i2c_read_reg(struct i2c_client *, unsigned char,
			unsigned short *, bool);
static int i2c_write_reg(struct i2c_client *, unsigned char,
			 unsigned short, bool);
static int _i2c_attach_client(struct i2c_client *, struct i2c_driver *,
			      struct i2c_adapter *, int);
static int _i2c_detach_client(struct i2c_client *);
static int mt9t001_i2c_probe_adapter(struct i2c_adapter *);
static int mt9t001_i2c_init(void);
void mt9t001_i2c_cleanup(void);

/* Parameters for  various format supported  */
/*Format  is
{
	NUMBER OF PIXELS PER LINE, NUMBER OF LINES, 
	HRIZONTAL BLANKING WIDTH, VERTICAL BLANKING WIDTH, 
	SHUTTER WIDTH, ROW ADDRESS MODE, COL ADDRESS MODE,
	BLACK_LEVEL,PIXEL CLOCK CONTROL, 
	ROW START, COL START
}
*/

const struct mt9t001_format_params MT9T001_VGA_30FPS =
    { 1979, 1467, 21, 31, 822, 0x22, 0x22, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_VGA_60FPS =
    { 1979, 1467, 21, 31, 582, 0x12, 0x12, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_SVGA_30FPS =
    { 1639, 1239, 21, 31, 1042, 0x11, 0x11, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_SVGA_60FPS =
    { 1639, 1239, 21, 31, 661, 0x01, 0x01, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_XGA_30FPS =
    { 1039, 775, 100, 283, 783, 0x00, 0x00, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_480P_30FPS =
    { 1471, 975, 350, 350, 898, 0x11, 0x11, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_480P_60FPS =
    { 1471, 975, 52, 50, 480, 0x11, 0x11, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_576P_25FPS =
    { 1471, 1167, 424, 450, 500, 0x11, 0x11, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_576P_50FPS =
    { 1471, 1167, 84, 48, 480, 0x11, 0x11, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_720P_24FPS =
    { 1299, 729, 300, 282, 568, 0x00, 0x00, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_720P_30FPS =
    { 1299, 729, 22, 220, 568, 0x00, 0x00, 64, 0x8000, 0, 0 };
const struct mt9t001_format_params MT9T001_1080P_18FPS =
    { 1919, 1079, 108, 31, 1095, 0x00, 0x00, 64, 0x8000, 0, 0 };

void mt9t001_readregs(void)
{
	unsigned short temp = 0;
	int err = 0;
	unsigned char regcnt;
	/* Here, 0x64 is number of registers for MT9T001 */
	for (regcnt = 0; regcnt < 0x64; regcnt++) {
		err = i2c_read_reg(&mt9t001_i2c_client, regcnt,
				   &temp, MT9T001_I2C_CONFIG);
		if (err >= 0) {
			dev_dbg(mt9t001_i2c_dev,
				"\nread back 0x%x = 0x%x...", regcnt, temp);
		}
	}
}

/*
 * ======== mt9t001_ctrl  ========
 */

/*This function will provide different control commands for MT9T001 
		configuration.*/
int mt9t001_ctrl(unsigned int cmd, void *arg, void *params)
{
	int err = 0;
	switch (cmd) {
	case MT9T001_SET_PARAMS:
		{

			struct mt9t001_params *vpfe_mt9t001params =
			    (struct mt9t001_params *)params;
			struct mt9t001_params *user_mt9t001params =
			    (struct mt9t001_params *)arg;

			/* Update the global parameter of vpfe_obj */

			if ((arg == NULL) || (params == NULL)) {
				dev_err(mt9t001_i2c_dev, "Invalid argument for \
							MT9T001_SET_PARAMS ");
				return -1;
			}

			memcpy(vpfe_mt9t001params, user_mt9t001params,
			       sizeof(struct mt9t001_params));

			err = mt9t001_setparams(arg);
			if (err < 0) {
				dev_err(mt9t001_i2c_dev,
					"\nMT9T001 set parameters fails...");
				return err;
			}
			break;

		}
	case MT9T001_SET_STD:
		{
			err = mt9t001_setstd(arg, params);
			if (err < 0) {
				dev_err(mt9t001_i2c_dev,
					"\nMT9T001 set standard fails...");
				return err;
			} else {
				//mt9t001_readregs();
			}
			break;
		}
	case MT9T001_GET_PARAMS:
		{

			struct mt9t001_params *vpfe_mt9t001params =
			    (struct mt9t001_params *)params;
			struct mt9t001_params *user_mt9t001params =
			    (struct mt9t001_params *)arg;

			err = mt9t001_getparams(arg);
			if (err < 0) {
				dev_err(mt9t001_i2c_dev,
					"\nMT9T001 get parameters  fails...");
				return err;
			}
			/* Update the global parameter of vpfe_obj */
			memcpy(vpfe_mt9t001params, user_mt9t001params,
			       sizeof(struct mt9t001_params));
			break;
		}
	case MT9T001_ENABLE_I2C_SWITCH:
		/* enable the i2c switch on the MT9T031 head board */
		CONFIG_PCA9543A = 1;
		break;
	case MT9T001_INIT:
		{
			err = mt9t001_init(arg, params);
			if (err < 0) {
				printk(KERN_ERR
				       "\n Unable to initialize MT9T001...");
				return err;
			}
			break;
		}
	case MT9T001_CLEANUP:
		{
			mt9t001_cleanup(params);

			break;
		}
	case VIDIOC_S_CTRL:
		{
			struct v4l2_control *ctrl = arg;

			if (ctrl->id == V4L2_CID_GAIN) {
				err = mt9t001_setgain((int)ctrl->value);
				if (err < 0) {
					dev_err(mt9t001_i2c_dev,
						"\n MT9T001 set gain fails...");
					return err;
				}
			} else {
				err = -EINVAL;
			}
			break;
		}
	case VIDIOC_G_CTRL:
		{
			struct v4l2_control *ctrl = arg;

			if (ctrl->id == V4L2_CID_GAIN) {
				err = mt9t001_getgain((int *)
						      &(ctrl->value));
				if (err < 0) {
					dev_err(mt9t001_i2c_dev,
						"\n MT9T001 get gain fails...");
					return err;
				}
			} else {
				err = -EINVAL;
			}
			break;
		}
	case VIDIOC_QUERYCTRL:
		{
			err = mt9t001_queryctrl(arg);
			break;
		}
	default:
		{
			dev_err(mt9t001_i2c_dev, "\n Undefined command");
			return -1;
		}

	}
	return err;
}

/*
 * ======== mt9t001_init  ========
 */
/*	This function will set the video format standart*/
static int mt9t001_init(void *arg, void **params)
{
	struct i2c_driver *driver = &mt9t001_i2c_driver;
	struct mt9t001_params *temp_params = NULL;
	int err = 0;

#if MT9T001_I2C_ENABLE
	/* Register MT9T001 I2C client */
	err = i2c_add_driver(driver);
	if (err) {
		printk(KERN_ERR "Failed to register MT9T001 I2C client.\n");
		return -1;
	}
	mt9t001_i2c_registration = MT9T001_I2C_REGISTERED;
#endif
	temp_params = kmalloc(sizeof(struct mt9t001_params), GFP_KERNEL);
	if (temp_params == NULL) {
		printk(KERN_ERR "\n Unable to allocate memory...");
		return -1;
	}
	(*params) = temp_params;
	/* enable path from mt9t001 to */
	err |= i2c_write_reg(&mt9t001_i2c_client, ECP_REGADDR,
			     ECP_REGVAL, ECP_I2C_CONFIG);
	if (CONFIG_PCA9543A)
		mt9t001_configpca9543a();
	/*Configure the MT9T001 in normalpower up mode */
	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_ROW_START,
			     MT9T001_ROW_START_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_COL_START,
			     MT9T001_COL_START_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_WIDTH,
			     MT9T001_WIDTH_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_HEIGHT,
			     MT9T001_HEIGHT_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_HBLANK,
			     MT9T001_HBLANK_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_VBLANK,
			     MT9T001_VBLANK_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_OUTPUT_CTRL,
			     MT9T001_OUTPUT_CTRL_DEFAULT, MT9T001_I2C_CONFIG);

	err |=
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_WIDTH_UPPER,
			  MT9T001_SHUTTER_WIDTH_UPPER_DEFAULT,
			  MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_WIDTH,
			     MT9T001_SHUTTER_WIDTH_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_PIXEL_CLK_CTRL,
			     MT9T001_PIXEL_CLK_CTRL_DEFAULT,
			     MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_RESTART,
			     MT9T001_RESTART_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_DELAY,
			     MT9T001_SHUTTER_DELAY_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_READ_MODE1,
			     MT9T001_READ_MODE1_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_READ_MODE2,
			     MT9T001_READ_MODE2_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_READ_MODE3,
			     MT9T001_READ_MODE3_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_ROW_ADDR_MODE,
			     MT9T001_ROW_ADDR_MODE_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_COL_ADDR_MODE,
			     MT9T001_COL_ADDR_MODE_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN1_GAIN,
			     MT9T001_GREEN1_GAIN_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLUE_GAIN,
			     MT9T001_BLUE_GAIN_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_RED_GAIN,
			     MT9T001_RED_GAIN_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN2_GAIN,
			     MT9T001_GREEN2_GAIN_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_GLOBAL_GAIN,
			     MT9T001_GLOBAL_GAIN_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLACK_LEVEL,
			     MT9T001_BLACK_LEVEL_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_CAL_COARSE,
			     MT9T001_CAL_COARSE_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_CAL_TARGET,
			     MT9T001_CAL_TARGET_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN1_OFFSET,
			     MT9T001_GREEN1_OFFSET_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN2_OFFSET,
			     MT9T001_GREEN2_OFFSET_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_RED_OFFSET,
			     MT9T001_RED_OFFSET_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLUE_OFFSET,
			     MT9T001_BLUE_OFFSET_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLK_LVL_CALIB,
			     MT9T001_BLK_LVL_CALIB_DEFAULT, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_CHIP_ENABLE_SYNC,
			     MT9T001_CHIP_ENABLE_SYNC_DEFAULT,
			     MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_RESET,
			     MT9T001_RESET_ENABLE, MT9T001_I2C_CONFIG);
	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_RESET,
			     MT9T001_RESET_DISABLE, MT9T001_I2C_CONFIG);
	/* delay applying changes  */
	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_OUTPUT_CTRL,
			     MT9T001_HALT_MODE, MT9T001_I2C_CONFIG);

	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_PIXEL_CLK_CTRL,
			     MT9T001_INVERT_PIXEL_CLK, MT9T001_I2C_CONFIG);

	/*Configure the MT9T001 in normalpower up mode */
	err |= i2c_write_reg(&mt9t001_i2c_client, MT9T001_OUTPUT_CTRL,
			     MT9T001_NORMAL_OPERATION_MODE, MT9T001_I2C_CONFIG);

	if (err < 0) {
		mt9t001_cleanup((*params));
	} else {
		/* Configure for default video standard */
		err = mt9t001_setstd(arg, (*params));

		if (err < 0) {
			mt9t001_cleanup(params);
		}
	}
	return err;
}

static int mt9t001_getformat(struct mt9t001_format_params *mt9tformats)
{
	int err = 0;
	unsigned short val = 0;

	/*Read the height width and blanking information required 
	   for particular format */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_HEIGHT,
			   &mt9tformats->row_size, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...height");
		return err;
	}

	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_WIDTH,
			   &mt9tformats->col_size, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...width");
		return err;
	}

	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_HBLANK,
			   &mt9tformats->h_blank, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...hblk");
		return err;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_VBLANK,
			   &mt9tformats->v_blank, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...vblk");
		return err;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_WIDTH,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...shutterwidth");
		return err;
	}
	mt9tformats->shutter_width = val & MT9T001_SHUTTER_WIDTH_LOWER_MASK;

	err =
	    i2c_read_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_WIDTH_UPPER,
			 &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...upper");
		return err;
	}
	mt9tformats->shutter_width |= ((val & MT9T001_SHUTTER_WIDTH_UPPER_MASK)
				       << MT9T001_SHUTTER_WIDTH_UPPER_SHIFT);

	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_ROW_ADDR_MODE,
			   &mt9tformats->row_addr_mode, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...addrmoderow");
		return err;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_COL_ADDR_MODE,
			   &mt9tformats->col_addr_mode, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...addrmodecol");
		return err;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_BLACK_LEVEL,
			   &mt9tformats->black_level, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...black_level");
		return err;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_ROW_START,
			   &mt9tformats->row_start, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...rowstart");
		return err;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_COL_START,
			   &mt9tformats->col_start, MT9T001_I2C_CONFIG);

	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...colstart");
		return err;
	}

	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_PIXEL_CLK_CTRL,
			   &mt9tformats->pixel_clk_control, MT9T001_I2C_CONFIG);

	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...clkctrl");
		return err;
	}
	return err;
}

static int mt9t001_setformat(struct mt9t001_format_params *mt9tformats)
{
	int err = 0;

	/*Write the height width and blanking information required 
	   for particular format */
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_HEIGHT,
			    mt9tformats->row_size, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...height");
		return err;
	}

	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_WIDTH,
			    mt9tformats->col_size, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...width");
		return err;
	}
	/* Configure for default video standard */

	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_HBLANK,
			    mt9tformats->h_blank, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...hblk");
		return err;
	}
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_VBLANK,
			    mt9tformats->v_blank, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...vblk");
		return err;
	}
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_WIDTH,
			    (unsigned short)(mt9tformats->
					     shutter_width &
					     MT9T001_SHUTTER_WIDTH_LOWER_MASK),
			    MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...shutterwidth");
		return err;
	}

	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_SHUTTER_WIDTH_UPPER,
			  (unsigned short)(mt9tformats->
					   shutter_width >>
					   MT9T001_SHUTTER_WIDTH_UPPER_SHIFT),
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...upper");
		return err;
	}

	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_ROW_ADDR_MODE,
			    mt9tformats->row_addr_mode, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...addrmoderow");
		return err;
	}
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_COL_ADDR_MODE,
			    mt9tformats->col_addr_mode, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...addrmodecol");
		return err;
	}
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLACK_LEVEL,
			    mt9tformats->black_level, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...black_level");
		return err;
	}
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_ROW_START,
			    mt9tformats->row_start, MT9T001_I2C_CONFIG);
	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...rowstart");
		return err;
	}
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_COL_START,
			    mt9tformats->col_start, MT9T001_I2C_CONFIG);

	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...colstart");
		return err;
	}

	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_PIXEL_CLK_CTRL,
			    mt9tformats->pixel_clk_control, MT9T001_I2C_CONFIG);

	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...clkctrl");
		return err;
	}
	/* applying changes  */
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_OUTPUT_CTRL,
			    MT9T001_NORMAL_OPERATION_MODE, MT9T001_I2C_CONFIG);

	if (err < 0) {
		printk(KERN_ERR "\n I2C write fails...outputctrl");
		return err;
	}

	return err;
}

/*
 * ======== configPCA9543A  ========
 */
/*	This function will configure PCA9543 control register*/
static int mt9t001_configpca9543a()
{
	int err = 0;
	/* enable path from mt9t001 to */
	err |= i2c_write_reg(&mt9t001_i2c_client, ECP_REGADDR,
			     ECP_REGVAL, ECP_I2C_CONFIG);

	/* Configure PCA9543A, here discard register address */
	err = i2c_write_reg(&mt9t001_i2c_client, 0,
			    PCA9543A_REGVAL, PCA9543A_I2C_CONFIG);
	return err;
}

/*
 * ======== mt9t001_cleanup  ========
 */

/*This function will free the memory allocated for mt9t001_params*/
static int mt9t001_cleanup(void *params)
{
	struct i2c_driver *driver = &mt9t001_i2c_driver;
	struct mt9t001_params *temp_params = (struct mt9t001_params *)params;
	if (temp_params != NULL)
		kfree(temp_params);
	params = NULL;
#if MT9T001_I2C_ENABLE
	if (mt9t001_i2c_registration) {
		i2c_detach_client(&mt9t001_i2c_client);
		i2c_del_driver(driver);
		mt9t001_i2c_client.adapter = NULL;
		mt9t001_i2c_registration = MT9T001_I2C_UNREGISTERED;
	}
#endif
	return 0;
}

/*
 * ======== mt9t001_setstd  ========
 */

/*	Function to set the video frame format*/
static int mt9t001_setstd(void *arg, void *params)
{
	v4l2_std_id mode = *(v4l2_std_id *) arg;
	struct mt9t001_format_params mt9tformats;
	int err = 0;

	/* Select configuration parameters as per video mode  */
	if (mode == MT9T001_MODE_VGA_30FPS) {
		mt9tformats = MT9T001_VGA_30FPS;
	} else if (mode == MT9T001_MODE_VGA_60FPS) {
		mt9tformats = MT9T001_VGA_60FPS;
	} else if (mode == MT9T001_MODE_SVGA_30FPS) {
		mt9tformats = MT9T001_SVGA_30FPS;
	} else if (mode == MT9T001_MODE_SVGA_60FPS) {
		mt9tformats = MT9T001_SVGA_60FPS;
	} else if (mode == MT9T001_MODE_XGA_30FPS) {
		mt9tformats = MT9T001_XGA_30FPS;
	} else if (mode == MT9T001_MODE_480p_30FPS) {
		mt9tformats = MT9T001_480P_30FPS;
	} else if (mode == MT9T001_MODE_480p_60FPS) {
		mt9tformats = MT9T001_480P_60FPS;
	} else if (mode == MT9T001_MODE_576p_25FPS) {
		mt9tformats = MT9T001_576P_25FPS;
	} else if (mode == MT9T001_MODE_576p_50FPS) {
		mt9tformats = MT9T001_576P_50FPS;
	} else if (mode == MT9T001_MODE_720p_24FPS) {
		mt9tformats = MT9T001_720P_24FPS;
	} else if (mode == MT9T001_MODE_720p_30FPS) {
		mt9tformats = MT9T001_720P_30FPS;
	} else if (mode == MT9T001_MODE_1080p_18FPS) {
		mt9tformats = MT9T001_1080P_18FPS;
	} else {
		printk(KERN_ERR "\n Invalid frame format");
		return -1;
	}

	err = mt9t001_setformat(&mt9tformats);

	return err;

}

/*
 * ======== mt9t001_setparams  ========
 */

/* This function will configure MT9T001 for bayer pattern capture.*/
static int mt9t001_setparams(void *arg)
{
	/*variable to store the return value of i2c read, write function */
	int err = 0;
	struct mt9t001_params *mt9t001params = (struct mt9t001_params *)arg;
	unsigned short val;
	if (arg == NULL) {
		dev_err(mt9t001_i2c_dev, "Invalid argument in config MT9T001");
		return -1;
	}
	/* set video format related parameters */
	err = mt9t001_setformat(&mt9t001params->format);

	/*Write the gain information */
	val = (unsigned short)(mt9t001params->rgb_gain.green1_analog_gain
			       & MT9T001_ANALOG_GAIN_MASK);

	val |= ((mt9t001params->rgb_gain.green1_digital_gain
		 << MT9T001_DIGITAL_GAIN_SHIFT) & MT9T001_DIGITAL_GAIN_MASK);
	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN1_GAIN, val,
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = (unsigned short)(mt9t001params->rgb_gain.red_analog_gain)
	    & MT9T001_ANALOG_GAIN_MASK;
	val |= (((mt9t001params->rgb_gain.red_digital_gain)
		 << MT9T001_DIGITAL_GAIN_SHIFT)
		& MT9T001_DIGITAL_GAIN_MASK);
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_RED_GAIN,
			    val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = (unsigned short)(mt9t001params->rgb_gain.blue_analog_gain)
	    & MT9T001_ANALOG_GAIN_MASK;
	val |= (((mt9t001params->rgb_gain.blue_digital_gain)
		 << MT9T001_DIGITAL_GAIN_SHIFT)
		& MT9T001_DIGITAL_GAIN_MASK);
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLUE_GAIN,
			    val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = (unsigned short)((mt9t001params->rgb_gain.green2_analog_gain)
			       << MT9T001_ANALOG_GAIN_SHIFT) &
	    MT9T001_ANALOG_GAIN_MASK;
	val |= (((mt9t001params->rgb_gain.green2_digital_gain)
		 << MT9T001_DIGITAL_GAIN_SHIFT) & MT9T001_DIGITAL_GAIN_MASK);
	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN2_GAIN, val,
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	/*Write the offset value in register */

	val = mt9t001params->black_calib.green1_offset
	    & MT9T001_GREEN1_OFFSET_MASK;
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN1_OFFSET,
			    val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = mt9t001params->black_calib.green2_offset
	    & MT9T001_GREEN2_OFFSET_MASK;
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_GREEN2_OFFSET,
			    val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = mt9t001params->black_calib.red_offset & MT9T001_RED_OFFSET_MASK;
	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_RED_OFFSET, val,
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = mt9t001params->black_calib.blue_offset & MT9T001_BLUE_OFFSET_MASK;
	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLUE_OFFSET, val,
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	/*Write other black caliberation information */

	val = (unsigned short)(mt9t001params->black_calib.manual_override)
	    & MT9T001_MANUAL_OVERRIDE_MASK;
	val |= ((mt9t001params->black_calib.disable_calibration)
		<< MT9T001_DISABLE_CALLIBERATION_SHIFT)
	    & MT9T001_DISABLE_CALLIBERATION_MASK;
	val |= ((mt9t001params->black_calib.recalculate_black_level)
		<< MT9T001_RECAL_BLACK_LEVEL_SHIFT)
	    & MT9T001_RECAL_BLACK_LEVEL_MASK;
	val |= ((mt9t001params->black_calib.lock_red_blue_calibration)
		<< MT9T001_LOCK_RB_CALIBRATION_SHIFT)
	    & MT9T001_LOCK_RB_CALLIBERATION_MASK;
	val |= ((mt9t001params->black_calib.lock_green_calibration)
		<< MT9T001_LOCK_GREEN_CALIBRATION_SHIFT)
	    & MT9T001_LOCK_GREEN_CALLIBERATION_MASK;
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_BLK_LVL_CALIB,
			    val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	/*Write Thresholds Value */

	val = (unsigned short)mt9t001params->black_calib.low_coarse_thrld
	    & MT9T001_LOW_COARSE_THELD_MASK;
	val |= (mt9t001params->black_calib.high_coarse_thrld
		<< MT9T001_HIGH_COARSE_THELD_SHIFT) &
	    MT9T001_HIGH_COARSE_THELD_MASK;
	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_CAL_COARSE, val,
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	val = (unsigned short)mt9t001params->black_calib.low_target_thrld
	    & MT9T001_LOW_TARGET_THELD_MASK;
	val |= (mt9t001params->black_calib.high_target_thrld
		<< MT9T001_HIGH_TARGET_THELD_SHIFT)
	    & MT9T001_HIGH_TARGET_THELD_MASK;
	err = i2c_write_reg(&mt9t001_i2c_client, MT9T001_CAL_TARGET,
			    val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}

	dev_dbg(mt9t001_i2c_dev, "\nEnd of configMT9T001...");
	return 0;

}

/*
 * ======== queryctrl ========
 */
 /* This function will return parameter values for control command passed */
static int mt9t001_queryctrl(void *arg)
{
	struct v4l2_queryctrl *queryctrl = arg;
	int ret = 0;
	int id = queryctrl->id;
	dev_dbg(mt9t001_i2c_dev, "\nStarting mt9t001_queryctrl...");
	if (queryctrl == NULL) {
		dev_err(mt9t001_i2c_dev,
			"\n Invalid argument in query control");
		return -1;
	}

	memset(queryctrl, 0, sizeof(*queryctrl));
	queryctrl->id = id;
	switch (id) {
	case V4L2_CID_GAIN:
		strcpy(queryctrl->name, "GAIN");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = MT9T001_GAIN_MINVAL;
		queryctrl->maximum = MT9T001_GAIN_MAXVAL;
		queryctrl->step = MT9T001_GAIN_STEP;
		queryctrl->default_value = MT9T001_GAIN_DEFAULTVAL;
		break;
	default:
		if (id < V4L2_CID_LASTP1)
			queryctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		else
			ret = -EINVAL;
		break;
	}			/* end switch (id) */
	dev_dbg(mt9t001_i2c_dev, "\nEnd of mt9t001_queryctrl...");
	return ret;
}

/*
 * ======== mt9t001_setgain  ========
 */

/*	This function will set the global gain of MT9T001*/
static int mt9t001_setgain(int arg)
{

	unsigned short gain = (unsigned short)arg;
	int err = 0;
	dev_dbg(mt9t001_i2c_dev,
		"\nStarting mt9t001_setgain with gain = %d...", (int)gain);
	err =
	    i2c_write_reg(&mt9t001_i2c_client, MT9T001_GLOBAL_GAIN, gain,
			  MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C write fails...");
		return err;
	}
	dev_dbg(mt9t001_i2c_dev, "\nEnd of mt9t001_setgain...");
	return err;
}

/*
 * ======== mt9t001_getgain  ========
 */

/*	This function will get the global gain of MT9T001*/
static int mt9t001_getgain(int *arg)
{
	unsigned short gain;
	int err = 0;
	dev_dbg(mt9t001_i2c_dev, "\nStarting mt9t001_getgain...");
	if (arg == NULL) {
		dev_err(mt9t001_i2c_dev,
			"\nInvalid argument pointer in get gain function");
		return -1;
	}
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_GLOBAL_GAIN,
			   &gain, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C reads fails...");
		return err;
	}
	*arg = gain;
	dev_dbg(mt9t001_i2c_dev, "\nEnd of mt9t001_getgain...");
	return err;
}

/*
 * ======== mt9t001_getparams  ========
 */

/*This function will get MT9T001 configuration values.*/

static int mt9t001_getparams(void *arg)
{

	struct mt9t001_params *params = (struct mt9t001_params *)arg;
	unsigned short val;
	int err = 0;
	dev_dbg(mt9t001_i2c_dev, "\nStarting mt9t001_getparams");

	/* get video format related parameters */
	err = mt9t001_getformat(&params->format);

	if (err < 0) {
		return err;
	}
	/*      Read green1 gain */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_GREEN1_GAIN,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->rgb_gain.green1_analog_gain = val & MT9T001_ANALOG_GAIN_MASK;
	params->rgb_gain.green1_digital_gain =
	    (val & MT9T001_DIGITAL_GAIN_MASK) >> MT9T001_DIGITAL_GAIN_SHIFT;

	/*      Read blue gain */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_BLUE_GAIN,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->rgb_gain.blue_analog_gain = val & MT9T001_ANALOG_GAIN_MASK;
	params->rgb_gain.blue_digital_gain =
	    (val & MT9T001_DIGITAL_GAIN_MASK) >> MT9T001_DIGITAL_GAIN_SHIFT;

	/*      Read red gain */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_RED_GAIN,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->rgb_gain.red_analog_gain = val & MT9T001_ANALOG_GAIN_MASK;
	params->rgb_gain.red_digital_gain =
	    (val & MT9T001_DIGITAL_GAIN_MASK) >> MT9T001_DIGITAL_GAIN_SHIFT;

	/*      Read green2 gain */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_GREEN2_GAIN,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->rgb_gain.green2_analog_gain = val & MT9T001_ANALOG_GAIN_MASK;
	params->rgb_gain.green2_digital_gain =
	    (val & MT9T001_DIGITAL_GAIN_MASK) >> MT9T001_DIGITAL_GAIN_SHIFT;

	/*      Read green1 offset */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_GREEN1_OFFSET,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.green1_offset = val & MT9T001_GREEN1_OFFSET_MASK;

	/*      Read green2 offset */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_GREEN2_OFFSET,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.green2_offset = val & MT9T001_GREEN2_OFFSET_MASK;

	/*      Read red offset */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_RED_OFFSET,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.red_offset = val & MT9T001_RED_OFFSET_MASK;

	/*      Read blue offset */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_BLUE_OFFSET,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.blue_offset = val & MT9T001_BLUE_OFFSET_MASK;

	/*      Read Black level caliberation information */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_BLK_LVL_CALIB,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.manual_override =
	    val & MT9T001_MANUAL_OVERRIDE_MASK;
	params->black_calib.disable_calibration =
	    (val & MT9T001_DISABLE_CALLIBERATION_MASK)
	    >> MT9T001_DISABLE_CALLIBERATION_SHIFT;
	params->black_calib.recalculate_black_level =
	    (val & MT9T001_RECAL_BLACK_LEVEL_MASK)
	    >> MT9T001_RECAL_BLACK_LEVEL_SHIFT;
	params->black_calib.lock_red_blue_calibration =
	    (val & MT9T001_LOCK_RB_CALLIBERATION_MASK)
	    >> MT9T001_LOCK_RB_CALIBRATION_SHIFT;
	params->black_calib.lock_green_calibration =
	    (val & MT9T001_LOCK_GREEN_CALLIBERATION_MASK)
	    >> MT9T001_LOCK_GREEN_CALIBRATION_SHIFT;

	/*      Read Black Level Caliberation Coarse Threshold Value */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_CAL_COARSE,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.low_coarse_thrld = val
	    & MT9T001_LOW_COARSE_THELD_MASK;
	params->black_calib.high_coarse_thrld =
	    (val & MT9T001_HIGH_COARSE_THELD_MASK)
	    >> MT9T001_HIGH_COARSE_THELD_SHIFT;

	/*      Read Black Level Caliberation Target Threshold Value */
	err = i2c_read_reg(&mt9t001_i2c_client, MT9T001_CAL_TARGET,
			   &val, MT9T001_I2C_CONFIG);
	if (err < 0) {
		dev_err(mt9t001_i2c_dev, "\n I2C read fails...");
		return err;
	}
	params->black_calib.low_target_thrld = val
	    & MT9T001_LOW_TARGET_THELD_MASK;
	params->black_calib.high_target_thrld =
	    (val & MT9T001_HIGH_COARSE_THELD_MASK)
	    >> MT9T001_HIGH_COARSE_THELD_SHIFT;

	dev_dbg(mt9t001_i2c_dev, "\nEnd of getparamsMT9T001...");
	return 0;
}

/*
 * ======== i2c_read_reg  ========
 */

/*This function is used to read value from register for i2c client. */

static int i2c_read_reg(struct i2c_client *client, unsigned char reg,
			unsigned short *val, bool configdev)
{
	int err = 0;
#ifdef MT9T001_I2C_ENABLE
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter) {
		err = -ENODEV;
	} else if (configdev == ECP_I2C_CONFIG) {
		msg->addr = ECP_I2C_ADDR;
		msg->flags = 0;
		msg->len = I2C_ONE_BYTE_TRANSFER;
		msg->buf = data;
		data[0] = reg;

		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			msg->flags = I2C_M_RD;
			msg->len = I2C_ONE_BYTE_TRANSFER;	/* 1 byte read */
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				*val = data[0];
			} else {
				dev_err(mt9t001_i2c_dev,
					"\n ERROR in ECP control register read\n");

			}
		} else {
			dev_err(mt9t001_i2c_dev,
				"\n ERROR in ECP control register read\n");
		}

	} else if (configdev == MT9T001_I2C_CONFIG) {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = I2C_ONE_BYTE_TRANSFER;
		msg->buf = data;
		data[0] = reg;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			msg->flags = I2C_M_RD;
			msg->len = I2C_TWO_BYTE_TRANSFER;	/* 2 byte read */
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				*val = ((data[0] & I2C_TXRX_DATA_MASK)
					<< I2C_TXRX_DATA_SHIFT)
				    | (data[1] & I2C_TXRX_DATA_MASK);
			}
		}
	}
#endif
	return err;
}

/*
 * ======== i2c_write_reg  ========
 */
/*This function is used to write value into register for i2c client. */
static int i2c_write_reg(struct i2c_client *client, unsigned char reg,
			 unsigned short val, bool configdev)
{
	int err = 0;
	int trycnt = 0;
	unsigned short readval = 0;

#ifdef MT9T001_I2C_ENABLE

	struct i2c_msg msg[1];
	unsigned char data[3];
	err = -1;
	while ((err < 0) && (trycnt < 5)) {
		trycnt++;
		if (!client->adapter) {
			err = -ENODEV;
		} else if (configdev == ECP_I2C_CONFIG) {
			/* if device is ECP then discard reg value
			 * and set ECP I2C address 
			 */
			msg->addr = ECP_I2C_ADDR;
			msg->flags = 0;
			msg->len = I2C_TWO_BYTE_TRANSFER;
			msg->buf = data;
			data[0] = (unsigned char)(reg & I2C_TXRX_DATA_MASK);
			data[1] = (unsigned char)(val & I2C_TXRX_DATA_MASK);

			err = i2c_transfer(client->adapter, msg, 1);
			if (err < 0) {
				printk(KERN_INFO
				       "\n ERROR in ECP  register write\n");
			}
		} else if (configdev == PCA9543A_I2C_CONFIG) {
			/* if device is ECP then discard reg value
			 * and set ECP I2C address 
			 */
			msg->addr = PCA9543A_I2C_ADDR;
			msg->flags = 0;
			msg->len = I2C_ONE_BYTE_TRANSFER;
			msg->buf = data;
			data[0] = (unsigned char)(val & I2C_TXRX_DATA_MASK);

			err = i2c_transfer(client->adapter, msg, 1);
			if (err < 0) {
				printk(KERN_INFO
				       "\n ERROR in PCA543a  register write\n");
			}
		} else if (configdev == MT9T001_I2C_CONFIG) {
			msg->addr = client->addr;
			msg->flags = 0;
			msg->len = I2C_THREE_BYTE_TRANSFER;
			msg->buf = data;
			data[0] = reg;
			data[1] = (val & I2C_TXRX_DATA_MASK_UPPER) >>
			    I2C_TXRX_DATA_SHIFT;
			data[2] = (val & I2C_TXRX_DATA_MASK);
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				err =
				    i2c_read_reg(&mt9t001_i2c_client, reg,
						 &readval, MT9T001_I2C_CONFIG);
				if ((err >= 0) && (val != readval)) {
					printk
					    (KERN_INFO
					     "\n ERROR: i2c readback failed, val = %d, readval = %d, reg = %d\n",
					     val, readval, reg);
				}
				readval = 0;
			}
		}
	}
#endif
	if (err < 0) {
		printk(KERN_INFO "\n I2C write failed");
	}
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
#ifdef MT9T001_I2C_ENABLE
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
#endif
	return err;
}

/*
 * ======== _i2c_detach_client  ========
 */
/* This function is used to detach i2c client */
static int _i2c_detach_client(struct i2c_client *client)
{
	int err = 0;
#ifdef MT9T001_I2C_ENABLE
	if (!client->adapter) {
		return -ENODEV;	/* our client isn't attached */
	} else {
		err = i2c_detach_client(client);
		client->adapter = NULL;
	}
#endif
	return err;
}

/*
 * ======== mt9t001_i2c_probe_adapter  ========
 */

/*This function is used to probe adapter for i2c_client. It returns
    0 if i2c_client is attached to adapter and error code on failure.*/
static int mt9t001_i2c_probe_adapter(struct i2c_adapter *adap)
{
	mt9t001_i2c_dev = &(adap->dev);
	return _i2c_attach_client(&mt9t001_i2c_client, &mt9t001_i2c_driver,
				  adap, MT9T001_I2C_ADDR);
}

/*
 * ======== mt9t001_i2c_init  ========
 */

/*	This function is used to initialize the i2c*/
static int mt9t001_i2c_init(void)
{
	int err = 0;
#ifdef MT9T001_I2C_ENABLE
	struct i2c_driver *driver = &mt9t001_i2c_driver;

	driver->owner = THIS_MODULE;
	strlcpy(driver->name, "MT9T001 CMOS sensor I2C driver",
		sizeof(driver->name));
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = mt9t001_i2c_probe_adapter;
	driver->detach_client = _i2c_detach_client;
#endif
	return err;
}

/*
 * ======== mt9t001_i2c_cleanup  ========
 */

void mt9t001_i2c_cleanup(void)
{
#ifdef MT9T001_I2C_ENABLE
	struct i2c_driver *driver = &mt9t001_i2c_driver;

	if (mt9t001_i2c_registration) {
		i2c_detach_client(&mt9t001_i2c_client);
		i2c_del_driver(driver);
		mt9t001_i2c_client.adapter = NULL;
		mt9t001_i2c_registration = MT9T001_I2C_UNREGISTERED;
	}
#endif
}

module_init(mt9t001_i2c_init);
module_exit(mt9t001_i2c_cleanup);

EXPORT_SYMBOL(mt9t001_ctrl);
MODULE_LICENSE("GPL");

/**********************************************************************/
/* End of file                                                        */
/**********************************************************************/
