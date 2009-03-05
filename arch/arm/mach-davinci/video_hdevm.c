/*
 *
 * Copyright (C) 2007 Texas Instruments	Inc
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under the	terms of the GNU General Public	License	as published by
 * the Free Software Foundation; either	version	2 of the License, or
 * (at your option)any	later version.
 *
 * This	program	is distributed in the hope that	it will	be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the	Free Software
 * Foundation, Inc., 59	Temple Place, Suite 330, Boston, MA  02111-1307	USA
 */
/* video_hdevm.h */
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <asm/arch/video_hdevm.h>
#include <asm/arch/i2c-client.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>

#define CPLD_BASE_ADDRESS	(0x3A)
#define CPLD_RESET_POWER_REG	(0)
#define CPLD_VIDEO_REG		(0x3B)
#define CDCE949			(0x6C)
static int cpld_initialized = 0;


int cpld_init() {
	int err = 0;
	/* power up tvp5147 and tvp7002 */
	u8 val=0x0;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if (!err) {
		cpld_initialized = 1;
	}
	return err;
}
void cpld_cleanup() {
}
int set_cpld_for_tvp5147() {
	int err = 0;
        u8 val;

	err = davinci_i2c_read(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	val &= 0xEF;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	return err;
}

int set_cpld_for_tvp7002() {
	int err = 0;
	u8 val;

	err = davinci_i2c_read(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	val |= 0x10;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	return err;
}

int set_vid_in_mode_for_tvp5147() {
	int err = 0;
	u8 val;
	unsigned int value;
	unsigned int sys_vsclk =
		(unsigned int)IO_ADDRESS(0x01C40038);
	err = davinci_i2c_read(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	val &= 0xDF;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;

	value = inl(sys_vsclk);
	value |= (1<<4);
	outl(value, sys_vsclk);

	return err;
}

int set_vid_in_mode_for_tvp7002() {

	int err = 0;
	u8 val;
	unsigned int value;
	unsigned int sys_vsclk =
		(unsigned int)IO_ADDRESS(0x01C40038);

	err = davinci_i2c_read(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	val |= 0x20;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;

	value = inl(sys_vsclk);
	value &= ~(1<<4);
	outl(value, sys_vsclk);

	return err;
}
int set_vid_out_mode_for_sd() {

	int err = 0;
	u8 val;

	unsigned int value;
	unsigned int sys_vsclk =
		(unsigned int)IO_ADDRESS(0x01C40038);

	err = davinci_i2c_read(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	val &= 0xBF;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;

	value = inl(sys_vsclk);
	value &= ~(7<<8);
	value |= (3<<8);
	outl(value, sys_vsclk);

	return err;
}

int set_vid_clock(int hd) 
{
	int err = 0;
	unsigned int value;
	unsigned int sys_vsclk =
		(unsigned int)IO_ADDRESS(0x01C40038);
	if(hd >= 1) {
		value = inl(sys_vsclk);
		value &= ~(7<<8);
		value &= ~(7<<12);
		value |= (2<<8);
		value |= (2<<12);
		outl(value, sys_vsclk);
	} else {
		value = inl(sys_vsclk);
		value &= ~(7<<8);
		value |= (3<<8);
		value |= (3<<12);
		outl(value, sys_vsclk);
	}
	return err;
}

int set_vid_out_mode_for_hd() {

	int err = 0;
	u8 val;
	unsigned int value;
	unsigned int sys_vsclk =
		(unsigned int)IO_ADDRESS(0x01C40038);
	err = davinci_i2c_read(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;
	val |= 0x40;
	err = davinci_i2c_write(1, &val, CPLD_VIDEO_REG);
	if(err)
		return err;

        value = inl(sys_vsclk);
        value &= ~(7<<8);
        value &= ~(7<<12);
	value |= (2<<8);
	value |= (2<<12);
	outl(value, sys_vsclk);

	return err;
}

/*
Capture :: bits PTSIMUX = 0x, PTSOMUX = 0x, STSIMUX = 00
Display :: bits STSOMUX = 0x, PTSOMUX = 0x, STSIMUX = 00

bits
17:16 PTSIMUX
19:18 PTSOMUX
21:20 STSIMUX
23:22 STSOMUX

*/
void set_vpif_pinmux()
{

	unsigned int pinmux0
		= (unsigned int)IO_ADDRESS(0x01C40000);
	unsigned int sys_vsclkdis =
		(unsigned int)IO_ADDRESS(0x01C4006C);
	unsigned int sys_vdd3p3vpwdn =
		(unsigned int)IO_ADDRESS(0x01C40048);
	unsigned int value;

	/* make 17th bit(PTSIMUX), 19th bit(PTSOMUX), 21:20 bits(STSIMUX), 23rd
	 * bit(STSOMUX) to zero */
	value = inl(pinmux0);
	value &= ~(unsigned int)(0x00FF0000);
	outl(value, pinmux0);

	value = inl(sys_vsclkdis);
	value &= ~(unsigned int)(0x00000F00);
	outl(value, sys_vsclkdis);

	value = inl(sys_vdd3p3vpwdn);
	value &= ~(unsigned int)(0x0000000F);
	outl(value, sys_vdd3p3vpwdn);
	
	return;
}

EXPORT_SYMBOL(set_cpld_for_tvp5147);
EXPORT_SYMBOL(set_cpld_for_tvp7002);
EXPORT_SYMBOL(set_vid_in_mode_for_tvp5147);
EXPORT_SYMBOL(set_vid_in_mode_for_tvp7002);
EXPORT_SYMBOL(set_vid_out_mode_for_sd);
EXPORT_SYMBOL(set_vid_out_mode_for_hd);
EXPORT_SYMBOL(set_vpif_pinmux);
EXPORT_SYMBOL(set_vid_clock);
MODULE_LICENSE("GPL");
/* Function for module initialization and cleanup */
module_init(cpld_init);
module_exit(cpld_cleanup);
