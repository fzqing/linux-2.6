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
 *	contains all hardware related functions for OV2620
 *
 * Notes:
 *	Only valid for processor code named Monahans.
 */

#include "camera.h"
#include "ov2620_hw.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>
#include <asm/hardware.h>
#include <asm/arch/zylonite.h>
#include <asm/string.h>

/*
 * Register Settings
 */
const static u8 ov2620InitSetting[]=
{
	OV2620_COMH,     0x80,        /* Initiates system reset */
	OV2620_COMF,     0x05,
	OV2620_CLKRC,    0x81,
	OV2620_COMH,     0x24,
	OV2620_FVO,      0x0c,
	OV2620_CHLF,     0x37,
	OV2620_VCHG,     0x4c,
	OV2620_R39,      0xf3,
	OV2620_AEW,      0x80,
	OV2620_AEB,      0x70,
	OV2620_COMI,     0x8f,
	OV2620_REGEND,   0x00
};

/*
 * Register Settings, Get from OmniVision
 */
static u8 UXGA[] =
{
	OV2620_COMH,    0x80,
	OV2620_COMF,    0x05,
	OV2620_COMH,    0x04,
	OV2620_FVO,     0x0c,
	OV2620_CHLF,    0x07,
	OV2620_VCHG,    0x4c,
	OV2620_R39,     0xf3,
	OV2620_CLKRC,   0x81,
	OV2620_COML,    0x00,
	OV2620_FRARL,   0x00,
	OV2620_REGEND,  0x00
};

static u8 SVGA[] =
{
	OV2620_COMH,    0x80,
	OV2620_COMF,    0x05,
	OV2620_COMH,    0x44,
	OV2620_FVO,     0x0c,
	OV2620_CHLF,    0x07,
	OV2620_VCHG,    0x4c,
	OV2620_R39,     0xf3,
	OV2620_CLKRC,   0x81,
	OV2620_COML,    0x10,
	OV2620_FRARL,   0x00,
	OV2620_HREFST,  0x3c,
	OV2620_HREFEND, 0xa0,
	OV2620_REGEND,  0x00
};

static u8 CIF[] =
{
	OV2620_COMH,    0x80,
	OV2620_COMF,    0x05,
	OV2620_COMH,    0x24,
	OV2620_FVO,     0x0c,
	OV2620_CHLF,    0x07,
	OV2620_VCHG,    0x4c,
	OV2620_R39,     0xf3,
	OV2620_CLKRC,   0x81,
	OV2620_COML,    0x40,
	OV2620_FRARL,   0x10,
	OV2620_REGEND,  0x00
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

	while (curReg < OV2620_REGEND) {
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

	while (curReg < OV2620_REGEND)	{
		curReg = regP[index << 1];
		if (curReg == regAddr)	{
			regP[(index << 1) + 1] = regValue;
			return 0;
		}
		index ++;
	}

	return -EIO;

}
#endif

/*
 * Sensor read/write
 */

static int rmw_sensor_reg(const u8 subAddress, u8 *bufP, u8 andMask, u8 orMask)
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

int ov2620hw_read_sensor_reg(const u8 subAddress, u8 *bufP)
{
	return read_sensor_reg(subAddress, bufP);
}

int ov2620hw_write_sensor_reg(const u8 subAddress, u8 *bufP)
{
	return write_sensor_reg(subAddress, bufP);
}

int ov2620hw_set_regs(const u8 *regP)
{
	u32 curReg = 0;
	int    status = 0;

	/* The list is a register number followed by the value */
	while (regP[curReg << 1] < OV2620_REGEND) {
		u8 regVal = regP[(curReg << 1) + 1];

		status = (write_sensor_reg(regP[curReg << 1], &regVal) == 0)?
			0 : -EIO;

		if (curReg == 0)
			ov2620hw_wait(5);

		curReg++;
	}

	return status;
}

int ov2620hw_read_all_regs(u8 *bufP, u32 numRegs)
{
	int curReg;

	for (curReg = 0; curReg < numRegs; curReg++, bufP++)
		read_sensor_reg((u8)curReg, bufP);

	return 0;
}

/*
 * Power & Reset
 */
void ov2620hw_power_down(u8 powerMode)
{
	/* OV2620 PWRDWN, 0 = NORMAL, 1=POWER DOWN */
	if (powerMode == CAMERA_POWER_OFF)
		mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	else
		mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_LOW);

	mdelay(100);
}

void ov2620hw_reset()
{
	ov2620hw_set_regs(ov2620InitSetting);
}

void ov2620hw_wait(int ms)
{
	mdelay(ms);
}

/*
 * Settings
 */
int ov2620hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision)
{
	read_sensor_reg(OV2620_PIDH, pCmRevision);
	read_sensor_reg(OV2620_PIDL, pSensorRevision);
	return 0;
}

void ov2620hw_set_hsync()
{
	u8	val;

	/* Makes HREF become HSYNC */
	read_sensor_reg(OV2620_COMK, &val);
	val |= 0x40;
	write_sensor_reg(OV2620_COMK, &val);
}

void ov2620hw_auto_function_on()
{
	u8 val;
	read_sensor_reg(OV2620_COMI, &val);
	val |= 0x07;    /* don't disturb AWB */
	write_sensor_reg(OV2620_COMI, &val);
}

void ov2620hw_auto_function_off()
{
	u8 val;
	read_sensor_reg(OV2620_COMI, &val);
	val &= ~0x07;    /* don't disturb AWB */
	write_sensor_reg(OV2620_COMI, &val);
}

/*
 * Viewfinder, still
 */
int ov2620hw_view_finder_on()
{
	u8 com3;

	read_sensor_reg(OV2620_COMD, &com3);
	com3 &= ~0x01;
	write_sensor_reg(OV2620_COMD, &com3);

	return OV_ERR_NONE;
}


int ov2620hw_view_finder_off()
{
	u8 com3;

	read_sensor_reg(OV2620_COMD, &com3);
	com3 |= 0x01;
	write_sensor_reg(OV2620_COMD, &com3);

	return OV_ERR_NONE;
}


int ov2620hw_halt_video_output()
{
	u8 com3;

	/* Set the camera to only output 1 frame */
	read_sensor_reg(OV2620_COMD, &com3);
	com3 |= 1;
	write_sensor_reg(OV2620_COMD, &com3);

	return OV_ERR_NONE;
}

int ov2620hw_resume_full_output_mode()
{
	u8 mode;

	/* Output still frames continuously
	 * Turn off single capture mode COM3.
	 */
	rmw_sensor_reg(OV2620_COMD, (&mode), ((u8) ~1), 0);
	return OV_ERR_NONE;
}

int ov2620hw_get_single_image()
{
	u8 mode;

	rmw_sensor_reg(OV2620_COMD, &mode, (u8) ~1, 1);
	return OV_ERR_NONE;
}

/*
 * Format
 */
int ov2620hw_set_format(u32 captureWidth,
		u32 captureHeight,
		u32 *winStartX,
		u32 *winStartY,
		u32 *winEndX,
		u32 *winEndY)
{
	OV2620_MODE mode;
	unsigned short hStart, hEnd;
	unsigned short vStart, vEnd;
	unsigned char regVal;
	unsigned short hMiddle, hProtection, vProtection;
	int bConfigROI = 1;

	hProtection = 1;
	vProtection =1;

	/* let the sensor work on proper mode */
	if ((captureWidth <= 403) && (captureHeight <= 301)) {
		mode = OV2620_CIF;
	} else if ((captureWidth <= 807) && (captureHeight <= 603)) {
		mode = OV2620_SVGA;
	} else if ((captureWidth <= 1615) && (captureHeight <= 1207)) {
		mode = OV2620_UXGA;
	} else {
		return -EINVAL;
	}

	if (mode == OV2620_CIF) {
		ov2620hw_set_regs(CIF);
	} else if (mode == OV2620_SVGA) {
		ov2620hw_set_regs(SVGA);
	} else {
		ov2620hw_set_regs(UXGA);
	}

	/* set cropping window */
	if (mode == OV2620_CIF) {
		captureWidth *= 2;
	}

	if (mode == OV2620_CIF) {
		hMiddle = 888;
		hStart = (unsigned short)(hMiddle - captureWidth/2);
		if (1 == hStart%2) {
			hStart++;
		}
		hEnd = (unsigned short)(hStart+captureWidth+hProtection);
		vStart = 1;
		vEnd = (unsigned short)(vStart + captureHeight/2)+vProtection;

		if ((hStart<=488)||(hEnd>=1288)||(vEnd>=151)) {
			bConfigROI = 0;
		}

	} else if (mode == OV2620_SVGA) {
		hMiddle = 880;
		hStart = (unsigned short)(hMiddle - captureWidth/2);
		if (1 == hStart%2) {
			hStart++;
		}
		hEnd = (unsigned short)(hStart+captureWidth+hProtection);
		vStart = 1;
		vEnd = (unsigned short)(vStart + captureHeight/2)+vProtection;

		if ((hStart<=480)||(hEnd>=1280)||(vEnd>=303)) {
			bConfigROI = 0;
		}

	} else {
		hMiddle = 1140;
		hStart = (unsigned short)(hMiddle - captureWidth/2);
		if (1 == hStart%2) {
			hStart++;
		}
		hEnd = (unsigned short)(hStart+captureWidth+hProtection);
		vStart = 1;
		vEnd = (unsigned short)(vStart + captureHeight/2)+vProtection;

		if ((hStart<=340)||(hEnd>=1940)||(vEnd>=603)) {
			bConfigROI = 0;
		}
	}

	if (bConfigROI) {
		/* set Horizontal Window Start */
		regVal = hStart>>3;
		write_sensor_reg(OV2620_HREFST, &regVal);
		read_sensor_reg(OV2620_COMM, &regVal);
		regVal &= ~0x07;
		regVal |= hStart & 0x07;
		write_sensor_reg(OV2620_COMM, &regVal);

		/* set Horizontal Window End */
		regVal = hEnd>>3;
		write_sensor_reg(OV2620_HREFEND, &regVal);
		read_sensor_reg(OV2620_COMM, &regVal);
		regVal &= ~(0x07 << 3);
		regVal |= (hEnd & 0x07) << 3;
		write_sensor_reg(OV2620_COMM, &regVal);

		/* set Vertical Window Start */
		regVal = vStart>>2;
		write_sensor_reg(OV2620_VSTRT, &regVal);
		read_sensor_reg(OV2620_COMA, &regVal);
		regVal &= ~0x03;
		regVal |= vStart & 0x03;
		write_sensor_reg(OV2620_COMA, &regVal);

		/* set Vertical Window End */
		regVal = vEnd>>2;
		write_sensor_reg(OV2620_VEND, &regVal);
		read_sensor_reg(OV2620_COMA, &regVal);
		regVal &= ~(0x03 << 2);
		regVal |= (vEnd & 0x03) << 2;
		write_sensor_reg(OV2620_COMA, &regVal);

	}

	/* return window region */

	*winStartX = hStart;
	*winStartY = vStart;
	*winEndX   = hEnd;
	*winEndY   = vEnd;

	return 0;
}

/*
 * Contrast
 */
/* FIX ME: TBD */
const static u8 ContrastLowestSettings[] = {
	0x6C, 0x80,
	0x6D, 0xa0,
	0x6E, 0x78,
	0x6F, 0x50,
	0x70, 0x48,
	0x71, 0x40,
	0x72, 0x48,
	0x73, 0x40,
	0x74, 0x40,
	0x75, 0x40,
	0x76, 0x40,
	0x77, 0x40,
	0x78, 0x3e,
	0x79, 0x3c,
	0x7A, 0x3c,
	0x7B, 0x28,
	0x7C, 0x8,
	0x7D, 0x12,
	0x7E, 0x21,
	0x7F, 0x35,
	0x80, 0x3e,
	0x81, 0x46,
	0x82, 0x4f,
	0x83, 0x57,
	0x84, 0x5f,
	0x85, 0x67,
	0x86, 0x77,
	0x87, 0x87,
	0x88, 0xa6,
	0x89, 0xc4,
	0x8A, 0xe2,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastLowSettings[] = {
	0x6C, 0x70,
	0x6D, 0x80,
	0x6E, 0x40,
	0x6F, 0x54,
	0x70, 0x58,
	0x71, 0x60,
	0x72, 0x60,
	0x73, 0x60,
	0x74, 0x50,
	0x75, 0x58,
	0x76, 0x44,
	0x77, 0x3c,
	0x78, 0x30,
	0x79, 0x28,
	0x7A, 0x22,
	0x7B, 0x44,
	0x7C, 0x7,
	0x7D, 0xf,
	0x7E, 0x17,
	0x7F, 0x2c,
	0x80, 0x37,
	0x81, 0x43,
	0x82, 0x4f,
	0x83, 0x5b,
	0x84, 0x65,
	0x85, 0x70,
	0x86, 0x81,
	0x87, 0x90,
	0x88, 0xa8,
	0x89, 0xbc,
	0x8A, 0xcd,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastMiddleSettings[] = {
	0x6C, 0x40,
	0x6D, 0x30,
	0x6E, 0x4B,
	0x6F, 0x60,
	0x70, 0x70,
	0x71, 0x70,
	0x72, 0x70,
	0x73, 0x70,
	0x74, 0x60,
	0x75, 0x60,
	0x76, 0x50,
	0x77, 0x48,
	0x78, 0x3A,
	0x79, 0x2E,
	0x7A, 0x28,
	0x7B, 0x22,
	0x7C, 0x4,
	0x7D, 0x7,
	0x7E, 0x10,
	0x7F, 0x28,
	0x80, 0x36,
	0x81, 0x44,
	0x82, 0x52,
	0x83, 0x60,
	0x84, 0x6C,
	0x85, 0x78,
	0x86, 0x8C,
	0x87, 0x9E,
	0x88, 0xBB,
	0x89, 0xD2,
	0x8A, 0xE6,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastHighSettings[] = {
	0x6c, 0x10,
	0x6d, 0x10,
	0x6e, 0x18,
	0x6f, 0x5c,
	0x70, 0x90,
	0x71, 0x90,
	0x72, 0x90,
	0x73, 0x90,
	0x74, 0x80,
	0x75, 0x80,
	0x76, 0x60,
	0x77, 0x5c,
	0x78, 0x44,
	0x79, 0x24,
	0x7a, 0x1a,
	0x7b, 0x10,
	0x7c, 0x1,
	0x7d, 0x2,
	0x7e, 0x5,
	0x7f, 0x1c,
	0x80, 0x2e,
	0x81, 0x40,
	0x82, 0x52,
	0x83, 0x64,
	0x84, 0x74,
	0x85, 0x84,
	0x86, 0x9c,
	0x87, 0xb3,
	0x88, 0xd5,
	0x89, 0xe7,
	0x8a, 0xf4,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastHighestSettings[] = {
	0x6c, 0x20,
	0x6d, 0x40,
	0x6e, 0x10,
	0x6f, 0x38,
	0x70, 0x80,
	0x71, 0xe0,
	0x72, 0xd0,
	0x73, 0xe8,
	0x74, 0xa0,
	0x75, 0x80,
	0x76, 0x80,
	0x77, 0x54,
	0x78, 0x30,
	0x79, 0x22,
	0x7a, 0x8,
	0x7b, 0x1,
	0x7c, 0x2,
	0x7d, 0x6,
	0x7e, 0x8,
	0x7f, 0x16,
	0x80, 0x26,
	0x81, 0x42,
	0x82, 0x5c,
	0x83, 0x79,
	0x84, 0x8d,
	0x85, 0x9d,
	0x86, 0xbd,
	0x87, 0xd2,
	0x88, 0xea,
	0x89, 0xfb,
	0x8a, 0xff,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

int ov2620hw_set_contrast(u32 value)
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
		ov2620hw_set_regs(regP);
	return 0;
}

/*
 * Exposure
 */
/* FIX ME: TBD */
const static u8 ExposureSettings[] = {
	0x40, 0x30, 0x81, /* EV-2 */
	0x58, 0x48, 0x91, /* EV-1 */
	0x88, 0x7c, 0x93, /* EV0  */
	0xa0, 0x90, 0xb4, /* EV+1 */
	0xc0, 0xb0, 0xd6, /* EV+2 */
};

int ov2620hw_set_exposure(u32 value)
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
		ov2620hw_write_sensor_reg(OV2620_AEW, &aew);
		ov2620hw_write_sensor_reg(OV2620_AEB, &aeb);
		ov2620hw_write_sensor_reg(OV2620_VV, &vv);
	}

	return 0;
}

/*
 * Auto White Balance
 */
/* FIX ME: TBD */
const static u8 AWBAuto[] = {
	0x13, 0xad,
	0x01, 0x80,
	0x02, 0x80,
	0x60, 0x14,
	0x5f, 0x05,
	0x13, 0xaf,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBFluorescent[] = {
	0x13, 0xad,
	0x01, 0x6c,
	0x02, 0x2e,
	0x5f, 0x05,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBOutdoor[] = {
	0x13, 0xad,
	0x01, 0x44,
	0x02, 0x44,
	0x5f, 0x05,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBIncandescent[] = {
	0x13, 0xad,
	0x01, 0x6c,
	0x02, 0x20,
	0x5f, 0x05,
	OV2620_REGEND,     0x00        /* End of list delimiter */
};


int ov2620hw_set_white_balance(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch (value) {
		case SENSOR_WHITEBALANCE_AUTO:                /* Auto */
			regP = AWBAuto;
			break;
		case SENSOR_WHITEBALANCE_INCANDESCENT:        /* Incandescent */
			regP = AWBIncandescent;
			break;
		case SENSOR_WHITEBALANCE_SUNNY:               /* Sunny */
			regP = AWBOutdoor;
			break;
		case SENSOR_WHITEBALANCE_FLUORESCENT:         /* Fluorescent */
			regP = AWBFluorescent;
			break;
		default:
			break;
	}

	/* set hw */
	if (regP) {
		ov2620hw_set_regs(regP);
	}
	return 0;
}

/*
 * OV2620 I2C Client Driver
 */
#include <linux/i2c.h>
static int i2c_ov2620_attach_adapter(struct i2c_adapter *adapter);
static int i2c_ov2620_detect_client(struct i2c_adapter *, int, int);
static int i2c_ov2620_detach_client(struct i2c_client *client);
#define I2C_DRIVERID_OV2620   I2C_DRIVERID_EXP1
#define	OV2620_ADDRESS	0x30

static struct i2c_driver ov2620_driver  =
{
	.owner		= THIS_MODULE,
	.name		= "ov2620",
	.id		= I2C_DRIVERID_OV2620,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= &i2c_ov2620_attach_adapter,
	.detach_client	= &i2c_ov2620_detach_client,
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = {OV2620_ADDRESS, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

static int read_sensor_reg(const u8 subAddress, u8 *bufP)
{
	int ret;

	if (g_client == NULL)	/*	No global client pointer?	*/
		return -1;

	ret = i2c_smbus_read_byte_data(g_client, subAddress);
	if (ret >= 0) {
		*bufP = ret;
	}
	return ret;
}

static int write_sensor_reg(const u8 subAddress, u8 *bufP)
{
	if (g_client == NULL)	/*	No global client pointer?	*/
		return -1;

	return i2c_smbus_write_byte_data(g_client, subAddress, *bufP);
}

static int i2c_ov2620_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int i2c_ov2620_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &i2c_ov2620_detect_client);
}

static int i2c_ov2620_detect_client(struct i2c_adapter *adapter,
		int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;

	/* Let's see whether this adapter can support what we need.
	 * Please substitute the things you need here!
	 */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		goto ERROR0;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 * But it allows us to access several i2c functions safely
	 */

	/* Note that we reserve some space for ov2620_data too. If you don't
	 * need it, remove it. We do it here to help to lessen memory
	 * fragmentation.
	 */

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client)  {
		err = -ENOMEM;
		goto ERROR0;
	}

	memset(new_client, 0, sizeof(struct i2c_client));

	/* FIXME */
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ov2620_driver;
	new_client->flags = 0;

	/* detect OV2620 */
	pxa_set_cken(CKEN_CAMERA, 1);
	mhn_mfp_set_afds(MFP_CIF_MCLK, MFP_AF0, MFP_DS04X);
	ci_set_clock(1, 1, 2600);
	mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_OUT);
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_LOW);
	mdelay(1);

	if ((i2c_ov2620_read(new_client, OV2620_PIDH) != PID_OV26XX) ||
		(i2c_ov2620_read(new_client, OV2620_PIDL) != PID_2620)) {
		ci_set_clock(0, 0, 2600);
		mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
		mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_IN);
		pxa_set_cken(CKEN_CAMERA, 0);
		goto ERROR1;
	} else {
		extern int ov2620_detected;
		ov2620_detected = 1;
		pr_info("OV2620 detected.\n");
	}
	ci_set_clock(0, 0, 2600);
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_IN);
	pxa_set_cken(CKEN_CAMERA, 0);

	g_client = new_client;

	strcpy(new_client->name, "OV2620");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	return 0;

ERROR1:
	kfree(new_client);
ERROR0:
	return err;
}

static int i2c_ov2620_detach_client(struct i2c_client *client)
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

static int __init i2c_ov2620_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&ov2620_driver))) {
		return ret;
	}

	return 0;
}

static void __exit i2c_ov2620_exit(void)
{
	i2c_del_driver(&ov2620_driver);
}

MODULE_DESCRIPTION("I2C OV2620 driver");
MODULE_LICENSE("GPL");

module_init(i2c_ov2620_init);
module_exit(i2c_ov2620_exit);
