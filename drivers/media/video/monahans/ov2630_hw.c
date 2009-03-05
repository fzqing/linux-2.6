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
 *	contains all hardware related functions for OV2630
 *
 * Notes:
 *	Only valid for processor code named Monahans.
 */
#include <linux/init.h>
#include <linux/module.h>
#include "camera.h"
#include "ov2630_hw.h"
#include <linux/delay.h>

#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>
#include <asm/hardware.h>
#include <asm/string.h>

/*
 * Register Settings
 */
const static u8 ov2630InitSetting[]=
{
	0x12,	0x80,
	0x0e,	0x0,
	0x0f,	0x42,
	0x13,	0xe7,
	0x14,	0x4e,
	0x24,	0x6c,
	0x25,	0x60,
	0x35,	0x90,
	0x36,	0x88,
	0x37,	0x44,
	0x3a,	0x90,
	0x3b,	0x14,
	0x3f,	0x0f,
	0x40,	0x0,
	0x41,	0x0,
	0x42,	0x0,
	0x43,	0x0,
	0x44,	0x80,
	0x4b,	0x00,
	0x4c,	0x28,
	0x50,	0xf4,
	0x58,	0x7,
	0x59,	0x20,
	0x5f,	0x40,
	0x75,	0x0f,
	0x78,	0x40,
	0x7a,	0x10,
	0x84,	0x4,
	0x86,	0x20,
	0x88,	0x0c,
	0x89,	0x08,
	0x8a,	0x2,
	OV2630_REGEND,   0x00
};

/*
 *  Register Settings, Get from OmniVision
 */
static u8 UXGA[] =
{
#if defined(CONFIG_PXA310)
	0x11,   0x01,
#else
	0x11,	0x02,   /* workaroud for issue 193451 */
#endif
	0x34,	0xf0,
	0x03,	0x48,
	0x17,	0x2d,
	0x18,	0x02,
	0x19,	0x01,
	0x1a,	0x97,
#if defined(CONFIG_PXA310)
	0x1e,   0x40,
	0x32,   0x0b,
#else
	0x1e,	0xc0,
	0x32,	0x1b,
#endif
	0x4d,	0xc0,
	0x5a,	0x00,
	0x87,	0x10,

	/* for non-zoom */
	0x0c,	0x21,
	0x16,	0x00,
	0x12,	0x00,
	0x48,	0x80,
	0x4a,	0x00,
	0x4e,	0x18,
	0x4f,	0x08,
	OV2630_REGEND,   0x00
};

static u8 SVGA[] =
{
	0x11,	0x00,
	0x34,   0x70,
	0x3,	0x0e,
	0x17,	0x3f,
	0x18,	0x02,
	0x19,	0x0,
	0x1a,	0x4b,
	0x1e,	0x00,
	0x32,	0x1f,
	0x4d,	0xc0,
	0x5a,	0x00,
	0x87,	0x00,

	/* for non-zoom */
	0x0c,	0xa0,
	0x16,	0x00,
	0x12,	0x41,
	0x48,	0x00,
	0x4a,	0x00,
	0x4e,	0x08,
	0x4f,	0x00,
	OV2630_REGEND,   0x00
};

static u8 CIF[] =
{
	0x11,	0x01,
	0x34,	0x70,
	0x03,	0x0a,
	0x17,	0x3f,
	0x18,	0x01,
	0x19,	0x00,
	0x1a,	0x25,
	0x1e,	0x00,
	0x32,	0xbf,
	0x4d,	0xc0,
	0x5a,	0x80,
	0x87,	0x00,

	/* for non-zoom */
	0x0c,	0xa0,
	0x16,	0x00,
	0x12,	0x21,
	0x48,	0x00,
	0x4a,	0x00,
	0x4e,	0x08,
	0x4f,	0x00,
	OV2630_REGEND,   0x00
};

const static u8 gSensorSlaveAddr = 0x30;
static int read_sensor_reg(const u8 subAddress, u8 *bufP);
static int write_sensor_reg(const u8 subAddress, u8 *bufP);

/*
 * Private/helper api
 */
#ifdef DEBUG_PARAM_CHECK
static int get_reg_value(u8 *regP, u8 regAddr, u8 *regValueP)
{
	unsigned int index = 0;
	u8 curReg = 0;

	while (curReg < OV2630_REGEND) {
		curReg = regP[index << 1];
		if (curReg == regAddr) {
			*regValueP = regP[(index << 1) + 1];
			return 0;
		}
		index ++;
	}

	return -EIO;

}

static int set_reg_value(u8 *regP, u8 regAddr, u8 regValue)
{
	unsigned int index = 0;
	u8 curReg = 0;

	while (curReg < OV2630_REGEND) {
		curReg = regP[index << 1];
		if (curReg == regAddr) {
			regP[(index << 1) + 1] = regValue;
			return 0;
		}
		index ++;
	}

	return -EIO;

}
#endif

/*
 *  Sensor read/write
 */

static int rmw_sensor_reg(const u8 subAddress, u8 *bufP,
		u8 andMask, u8 orMask)
{
	int status;
	status = read_sensor_reg(subAddress, bufP);
	if (!status) {
		*bufP &= andMask;
		*bufP |= orMask;
		status = write_sensor_reg(subAddress, bufP);
	}
	return status;
}

int ov2630hw_read_sensor_reg(const u8 subAddress, u8 *bufP)
{
	return read_sensor_reg(subAddress, bufP);
}

int ov2630hw_write_sensor_reg(const u8 subAddress, u8 *bufP)
{
	return write_sensor_reg(subAddress, bufP);
}

int ov2630hw_set_regs(const u8 *regP)
{
	u32 curReg = 0;
	int    status = 0;

	/* The list is a register number followed by the value */
	while (regP[curReg << 1] < OV2630_REGEND) {
		u8 regVal = regP[(curReg << 1) + 1];

		status = (write_sensor_reg(regP[curReg << 1], &regVal) == 0) ?
			0 : -EIO;

		if (curReg == 0)
			ov2630hw_wait(5);

		curReg++;
	}

	return status;
}

int ov2630hw_read_all_regs(u8 *bufP, u32 numRegs)
{
	u32 curReg;

	for (curReg = 0; curReg < numRegs; curReg++, bufP++)
		read_sensor_reg((u8)curReg, bufP);


	return 0;
}

/*
 * Power & Reset
 */
void ov2630hw_power_down(u8 powerMode)
{
	/* OV2630 PWRDWN, 0 = NORMAL, 1=POWER DOWN */
	if (powerMode == CAMERA_POWER_OFF)
		mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	else
		mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_LOW);

	mdelay(100);
}

void ov2630hw_reset()
{
	ov2630hw_set_regs(ov2630InitSetting);
}

void ov2630hw_wait(int ms)
{
	mdelay(ms);
}

/*
 * Settings
 */
int ov2630hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision)
{
	read_sensor_reg(OV2630_PIDH, pCmRevision);
	read_sensor_reg(OV2630_PIDL, pSensorRevision);
	return 0;
}

void ov2630hw_set_hsync()
{
	u8 val;

	/* Makes HREF become HSYNC */
	read_sensor_reg(OV2630_COMK, &val);
	val |= 0x40;
	write_sensor_reg(OV2630_COMK, &val);
}

void ov2630hw_auto_function_on()
{
	u8 val;
	read_sensor_reg(OV2630_COMI, &val);
	val |= 0x07;    /* don't disturb AWB */
	write_sensor_reg(OV2630_COMI, &val);
}

void ov2630hw_auto_function_off()
{
	u8 val;
	read_sensor_reg(OV2630_COMI, &val);
	val &= ~0x07;    /* don't disturb AWB */
	write_sensor_reg(OV2630_COMI, &val);
}

/*
 * Viewfinder, still
 */
int ov2630hw_viewfinder_on()
{
	u8 com3;

	read_sensor_reg(OV2630_COMD, &com3);
	com3 &= ~0x01;
	write_sensor_reg(OV2630_COMD, &com3);

	return OV_ERR_NONE;
}


int ov2630hw_viewfinder_off()
{
	u8 com3;


	read_sensor_reg(OV2630_COMD, &com3);
	com3 |= 0x01;
	write_sensor_reg(OV2630_COMD, &com3);

	return OV_ERR_NONE;
}


int ov2630hw_halt_video_output()
{
	u8 com3;

	/* Set the camera to only output 1 frame */
	read_sensor_reg(OV2630_COMD, &com3);
	com3 |= 1;
	write_sensor_reg(OV2630_COMD, &com3);

	return OV_ERR_NONE;
}

int ov2630hw_resumeto_full_output_mode()
{
	u8 mode;

	/* Output still frames continuously
	 * Turn off single capture mode COM3.
	 */
	rmw_sensor_reg(OV2630_COMD, (&mode), ((u8) ~1), 0);
	return OV_ERR_NONE;
}

int ov2630hw_get_single_image()
{
	u8 mode;

	rmw_sensor_reg(OV2630_COMD, &mode, (u8) ~1, 1);
	return OV_ERR_NONE;
}

/*
 * Format
 */
int ov2630hw_set_format(u32 captureWidth,
		u32 captureHeight,
		u32 *winStartX,
		u32 *winStartY,
		u32 *winEndX,
		u32 *winEndY)
{
	OV2630_MODE mode;
	unsigned short hStart;
	unsigned short vStart;
	unsigned char regVal;

	/* let the sensor work on proper mode */
	if ((captureWidth <= 400) && (captureHeight <= 292)) {
		mode = OV2630_CIF;
	} else if ((captureWidth <= 800) && (captureHeight <= 600)) {
		mode = OV2630_SVGA;
	} else if ((captureWidth <= 1600) && (captureHeight <= 1200)) {
		mode = OV2630_UXGA;
	} else {
		return -EINVAL;
	}

	if (mode == OV2630_CIF) {
		ov2630hw_set_regs(CIF);
	} else if (mode == OV2630_SVGA) {
		ov2630hw_set_regs(SVGA);
	} else{
		ov2630hw_set_regs(UXGA);
	}

	/* set cropping window */
	if (mode == OV2630_CIF) {
		captureWidth *= 2;
	}


	if (mode == OV2630_CIF) {
		hStart = (unsigned short)(511 + (800 -captureWidth)/2);
		vStart = (unsigned short)(2 + (292 - captureHeight)/4);
	} else if (mode == OV2630_SVGA) {
		hStart = (unsigned short)(511 + (800 -captureWidth)/2);
		vStart = (unsigned short)(2 + (600 - captureHeight)/4);
	} else {
		hStart = (unsigned short)(363 + (1600 -captureWidth)/2);
		vStart = (unsigned short)(4 + (1200 - captureHeight)/4);
	}

	/* set Horizontal Window Start */
	regVal = hStart>>3;
	write_sensor_reg(OV2630_HREFST, &regVal);
	read_sensor_reg(OV2630_COMM, &regVal);
	regVal &= ~0x07;
	regVal |= hStart & 0x07;
	write_sensor_reg(OV2630_COMM, &regVal);


	/* set Vertical Window Start */
	regVal = vStart>>2;
	write_sensor_reg(OV2630_VSTRT, &regVal);
	read_sensor_reg(OV2630_COMA, &regVal);
	regVal &= ~0x03;
	regVal |= vStart & 0x03;
	write_sensor_reg(OV2630_COMA, &regVal);

	/* return window region */
	*winStartX = hStart;
	*winStartY = vStart;
	*winEndX   = hStart + captureWidth;
	*winEndY   = vStart + captureHeight;

	return 0;
}

/*
 * Contrast
 */
/* FIX ME: TBD */
const static u8 ContrastLowestSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastLowSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastMiddleSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastHighSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastHighestSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

int ov2630hw_set_contrast(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch (value) {
		case SENSOR_CONTRAST_LOWEST:
			regP = ContrastLowestSettings;
			break;
		case SENSOR_CONTRAST_LOW:
			regP = ContrastLowSettings;
			break;
		case SENSOR_CONTRAST_MIDDLE:
			regP = ContrastMiddleSettings;
			break;
		case SENSOR_CONTRAST_HIGH:
			regP = ContrastHighSettings;
			break;
		case SENSOR_CONTRAST_HIGHEST:
			regP = ContrastHighestSettings;
			break;
		default:
			regP = ContrastMiddleSettings;
			break;
	}

	/* set hw */
	if (regP)
		ov2630hw_set_regs(regP);
	return 0;
}

/*
 * Exposure
 */
/* FIX ME: TBD */
const static u8 ExposureSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

int ov2630hw_set_exposure(u32 value)
{
	u8 aew, aeb, vv;
	int index = -1;

	switch (value) {
		case SENSOR_EXPOSURE_LOWEST:
			index = 0;
			break;
		case SENSOR_EXPOSURE_LOW:
			index = 3;
			break;
		case SENSOR_EXPOSURE_MIDDLE:
			index = 6;
			break;
		case SENSOR_EXPOSURE_HIGH:
			index = 9;
			break;
		case SENSOR_EXPOSURE_HIGHEST:
			index = 12;
			break;
		default:
			break;
	}

	aew = aeb = vv = 0;

	if (index != -1) {
		aew = ExposureSettings[index];
		aeb = ExposureSettings[index + 1];
		vv  = ExposureSettings[index + 2];
	}

	/* set hw */
	if (aew || aeb || vv) {
		ov2630hw_write_sensor_reg(OV2630_AEW, &aew);
		ov2630hw_write_sensor_reg(OV2630_AEB, &aeb);
		ov2630hw_write_sensor_reg(OV2630_VV, &vv);
	}

	return 0;
}

/*
 * Auto White Balance
 */
/* FIX ME: TBD */
const static u8 AWBAuto[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBFluorescent[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBOutdoor[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBIncandescent[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

int ov2630hw_set_white_balance(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch (value) {
		case SENSOR_WHITEBALANCE_AUTO:		/* Auto */
			regP = AWBAuto;
			break;
		case SENSOR_WHITEBALANCE_INCANDESCENT:	/* Incandescent */
			regP = AWBIncandescent;
			break;
		case SENSOR_WHITEBALANCE_SUNNY:		/* Sunny */
			regP = AWBOutdoor;
			break;
		case SENSOR_WHITEBALANCE_FLUORESCENT:	/* Fluorescent */
			regP = AWBFluorescent;
			break;
		default:
			break;
	}

	/* set hw */
	if (regP) {
		ov2630hw_set_regs(regP);
	}
	return 0;
}

/*
 * OV2630 I2C Client Driver
 */
#include <linux/i2c.h>
static int i2c_ov2630_attach_adapter(struct i2c_adapter *adapter);
static int i2c_ov2630_detect_client(struct i2c_adapter *, int,  int);
static int i2c_ov2630_detach_client(struct i2c_client *client);
#define I2C_DRIVERID_OV2630   I2C_DRIVERID_EXP1
#define	OV2630_ADDRESS	0x30

static struct i2c_driver ov2630_driver  =
{
	.owner		= THIS_MODULE,
	.name		= "ov2630",
	.id		= I2C_DRIVERID_OV2630,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= &i2c_ov2630_attach_adapter,
	.detach_client	= &i2c_ov2630_detach_client,
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = {OV2630_ADDRESS, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

static int read_sensor_reg(const u8 subAddress, u8 *bufP)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	ret = i2c_smbus_read_byte_data(g_client, subAddress);
	if (ret >= 0) {
		*bufP = ret;
	}
	return ret;
}

static int write_sensor_reg(const u8 subAddress, u8 *bufP)
{
	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	return i2c_smbus_write_byte_data(g_client, subAddress, *bufP);
}

static int i2c_ov2630_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int i2c_ov2630_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, i2c_ov2630_detect_client);
}

static int i2c_ov2630_detect_client(struct i2c_adapter *adapter,
		int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;

	/* Let's see whether this adapter can support what we need.
	   Please substitute the things you need here!  */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		goto ERROR0;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 * But it allows us to access several i2c functions safely
	 */

	/* Note that we reserve some space for ov2630_data too. If you don't
	 * need it, remove it. We do it here to help to lessen memory
	 * fragmentation.
	 */
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client)  {
		err = -ENOMEM;
		goto ERROR0;
	}

	memset(new_client, 0, sizeof(struct i2c_client));

	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ov2630_driver;
	new_client->flags = 0;

	/* detect OV2630 */
	pxa_set_cken(CKEN_CAMERA, 1);
	mhn_mfp_set_afds(MFP_CIF_MCLK, MFP_AF0, MFP_DS04X);
	ci_set_clock(1, 1, 2600);
	mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_OUT);
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_LOW);
	mdelay(1);
	if ((i2c_ov2630_read(new_client, OV2630_PIDH) != PIDH_OV2630) ||
		(i2c_ov2630_read(new_client, OV2630_PIDL) != PIDL_OV2630)) {
		ci_set_clock(0, 0, 2600);
		mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
		mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_IN);
		pxa_set_cken(CKEN_CAMERA, 0);
		goto ERROR1;
	}
	else {
		extern int ov2630_detected;
		ov2630_detected = 1;
		pr_info("OV2630 detected.\n");
	}

	ci_set_clock(0, 0, 2600);
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_IN);
	pxa_set_cken(CKEN_CAMERA, 0);

	g_client = new_client;

	strcpy(new_client->name, "OV2630");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	return 0;

ERROR1:
	kfree(new_client);
ERROR0:
	return err;
}

static int i2c_ov2630_detach_client(struct i2c_client *client)
{
	int err;

	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		return err;
	}

	/* Frees client data too, if allocated at the same time */
	kfree(client);
	g_client = NULL;
	return 0;
}

static int __init i2c_ov2630_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&ov2630_driver))) {
		return ret;
	}

	return 0;
}

static void __exit i2c_ov2630_exit(void)
{
	i2c_del_driver(&ov2630_driver);
}

MODULE_DESCRIPTION("I2C OV2630 driver");
MODULE_LICENSE("GPL");

module_init(i2c_ov2630_init);
module_exit(i2c_ov2630_exit);

