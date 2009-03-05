/*
 * DAVINCI pin multiplexing configurations
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on linux/arch/arm/mach-omap1/mux.c:
 * Copyright (C) 2003 - 2005 Nokia Corporation
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/spinlock.h>

#include <asm/arch/mux.h>

#ifdef CONFIG_DAVINCI_MUX

struct pin_config __initdata_or_module davinci_dm644x_pins[] = {
/*
 *	 description		mux  mode   mode  mux	 dbg
 *				reg  offset mask  mode
 */
MUX_CFG("HDIREN",		 0,   16,    1,	  1,	 1)
MUX_CFG("ATAEN",		 0,   17,    1,	  1,	 1)

MUX_CFG("MSTK",			 1,   9,     1,	  0,	 0)

MUX_CFG("I2C",			 1,   7,     1,	  1,	 0)

MUX_CFG("MCBSP0",		 1,   10,    1,	  1,	 0)

MUX_CFG("PWM0",			 1,   4,     1,	  1,	 0)

MUX_CFG("PWM1",			 1,   5,     1,	  1,	 0)

MUX_CFG("PWM2",			 1,   6,     1,	  1,	 0)

MUX_CFG("VLINQEN",		 0,   15,    1,	  1,	 0)
MUX_CFG("VLINQWD",		 0,   12,    3,	  3,	 0)

MUX_CFG("EMACEN",		 0,   31,    1,	  1,	 1)

MUX_CFG("GPIO3V",		 0,   31,    1,	  0,	 1)

MUX_CFG("GPIO0",		 0,   24,    1,	  0,	 1)
MUX_CFG("GPIO3",		 0,   25,    1,	  0,	 0)
MUX_CFG("GPIO43_44",		 1,   7,     1,	  0,	 0)
MUX_CFG("GPIO46_47",		 0,   22,    1,	  0,	 1)

MUX_CFG("RGB666",		 0,   22,    1,	  1,	 1)

MUX_CFG("LOEEN",		 0,   24,    1,	  1,	 1)
MUX_CFG("LFLDEN",		 0,   25,    1,	  1,	 0)
};

struct pin_config __initdata_or_module davinci_dm646x_pins[] = {
/*
 *	 description		mux  mode   mode  mux	 dbg
 *				reg  offset mask  mode
 */
MUX_CFG("ATAEN",		 0,   0,     1,	  1,	 1)

MUX_CFG("VBUSDIS",		 0,   31,    1,	  0,	 0)
};

struct pin_config __initdata_or_module davinci_dm355_pins[] = {
/*
 *	 description		mux  mode   mode  mux	 dbg
 *				reg  offset mask  mode
 */
MUX_CFG("MMCSD0",		 4,   2,     1,	  0,	 0)

MUX_CFG("SD1_CLK",		 3,   6,     1,	  1,	 0)
MUX_CFG("SD1_CMD",		 3,   7,     1,	  1,	 0)
MUX_CFG("SD1_DATA3",		 3,   8,     3,	  1,	 0)
MUX_CFG("SD1_DATA2",		 3,   10,    3,	  1,	 0)
MUX_CFG("SD1_DATA1",		 3,   12,    3,	  1,	 0)
MUX_CFG("SD1_DATA0",		 3,   14,    3,	  1,	 0)

MUX_CFG("I2C_SDA",		 3,   19,    1,	  1,	 0)
MUX_CFG("I2C_SCL",		 3,   20,    1,	  1,	 0)

MUX_CFG("MCBSP0_BDX",		 3,   0,     1,	  1,	 0)
MUX_CFG("MCBSP0_X",		 3,   1,     1,	  1,	 0)
MUX_CFG("MCBSP0_BFSX",		 3,   2,     1,	  1,	 0)
MUX_CFG("MCBSP0_BDR",		 3,   3,     1,	  1,	 0)
MUX_CFG("MCBSP0_R",		 3,   4,     1,	  1,	 0)
MUX_CFG("MCBSP0_BFSR",		 3,   5,     1,	  1,	 0)

MUX_CFG("PWM0",			 1,   0,     3,	  2,	 1)

MUX_CFG("PWM1",			 1,   2,     3,	  2,	 1)

MUX_CFG("PWM2_G76",		 1,   10,    3,	  2,	 1)
MUX_CFG("PWM2_G77",		 1,   8,     3,	  2,	 1)
MUX_CFG("PWM2_G78",		 1,   6,     3,	  2,	 1)
MUX_CFG("PWM2_G79",		 1,   4,     3,	  2,	 1)

MUX_CFG("PWM3_G69",		 1,   20,    3,	  3,	 0)
MUX_CFG("PWM3_G70",		 1,   18,    3,	  3,	 0)
MUX_CFG("PWM3_G74",		 1,   14,    3,	  2,	 1)
MUX_CFG("PWM3_G75",		 1,   12,    3,	  2,	 1)

MUX_CFG("SPI0_SDI",		 4,   1,     1,	  0,	 0)
MUX_CFG("SPI0_SDENA0",		 4,   0,     1,	  0,	 0)
MUX_CFG("SPI0_SDENA1",		 3,   28,    1,	  1,	 0)

MUX_CFG("SPI1_SCLK",		 3,   24,    1,	  1,	 0)
MUX_CFG("SPI1_SDO",		 3,   27,    1,	  1,	 0)
MUX_CFG("SPI1_SDENA0",		 3,   23,    1,	  1,	 0)
MUX_CFG("SPI1_SDENA1",		 3,   25,    3,	  2,	 0)

MUX_CFG("SPI2_SCLK",		 0,   0,     3,	  2,	 0)
MUX_CFG("SPI2_SDO",		 0,   2,     3,	  2,	 0)
MUX_CFG("SPI2_SDENA0",		 0,   4,     3,	  2,	 0)
MUX_CFG("SPI2_SDENA1",		 0,   6,     3,	  3,	 0)

MUX_CFG("GPIO14",		 3,   20,    1,	  0,	 0)
MUX_CFG("GPIO15",		 3,   19,    1,	  0,	 0)
MUX_CFG("GPIO71",		 1,   17,    1,	  1,	 0)

MUX_CFG("VOUT_FIELD",		 1,   18,    3,	  1,	 1)
MUX_CFG("VOUT_FIELD_G70",	 1,   18,    3,	  0,	 1)
MUX_CFG("VOUT_HVSYNC",		 1,   16,    1,	  0,	 0)
MUX_CFG("VOUT_COUTL_EN",	 1,   0,     0xff, 0x55,  1)
MUX_CFG("VOUT_COUTH_EN",	 1,   8,     0xff, 0x55,  1)

MUX_CFG("VIN_PCLK",		 0,   14,    1,	  1,	 0)
MUX_CFG("VIN_CAM_WEN",		 0,   13,    1,	  1,	 0)
MUX_CFG("VIN_CAM_VD",		 0,   12,    1,	  1,	 0)
MUX_CFG("VIN_CAM_HD",		 0,   11,    1,	  1,	 0)
MUX_CFG("VIN_YIN_EN",		 0,   10,    1,	  1,	 0)
MUX_CFG("VIN_CINL_EN",		 0,   0,     0xff, 0x55,  0)
MUX_CFG("VIN_CINH_EN",		 0,   8,     3,	  3,	 0)
};

int __init davinci_mux_init(void)
{
	if (cpu_is_davinci_dm644x())
		davinci_mux_register(davinci_dm644x_pins,
					ARRAY_SIZE(davinci_dm644x_pins));
	else if (cpu_is_davinci_dm6467())
		davinci_mux_register(davinci_dm646x_pins,
					ARRAY_SIZE(davinci_dm646x_pins));
	else if (cpu_is_davinci_dm355())
		davinci_mux_register(davinci_dm355_pins,
					ARRAY_SIZE(davinci_dm355_pins));
	else
		printk(KERN_WARNING "DaVinci variant not supported\n");

	return 0;
}

#endif
