/*
 * TI DaVinci EVM board
 *
 * Copyright (C) 2007 Texas Instruments.
 * Copyright (C) 2007 Monta Vista Software Inc.
 *
 * ----------------------------------------------------------------------------
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 *  This file came directly from spi_platform_init.c.  This file has been
 *  generalized to all DaVinci variants.  This file should replace
 *  spi_platform_init.c
 *
 */

/*
 * Platform device support for TI SoCs.
 *
 */
#include <linux/config.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/spi/spi.h>
#include <linux/spi/davinci_spi.h>
#include <linux/spi/flash.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/arch/hardware.h>
#include <linux/spi/at25xxA_eeprom.h>
#include <linux/spi/davinci_spi_master.h>
#include <asm/arch/cpu.h>
#include <asm/io.h>

static struct davinci_spi_platform_data dm355_spi_platform_data = {
	.initial_spmode = 0,
	.bus_num = -1,
	.max_chipselect = 2,
	.activate_cs = NULL,
	.deactivate_cs = NULL,
	.sysclk = 108 * 1000 * 1000,
};

static struct davinci_spi_platform_data dm646x_spi_platform_data = {
	.initial_spmode = 0,
	.bus_num = -1,
	.max_chipselect = 2,
	.activate_cs = NULL,
	.deactivate_cs = NULL,
	.sysclk = 67.5 * 1000 * 1000,
};

static struct resource dm646x_spi_resources[] = {
	[0] = {
	       .start = DAVINCI_SPI_BASE,
	       .end = DAVINCI_SPI_BASE + (SZ_4K/2),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SPINT0,
	       .end = IRQ_SPINT0,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct resource dm355_spi_resources[] = {
	[0] = {
	       .start = DAVINCI_DM355_SPI0_BASE,
	       .end = DAVINCI_DM355_SPI0_BASE + (SZ_4K/2),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_DM355_SPINT0_0,
	       .end = IRQ_DM355_SPINT0_0,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device davinci_spi_device = {
	.name = "dm_spi",
	.id = 0,
	.num_resources = ARRAY_SIZE(dm646x_spi_resources),
	.resource = dm646x_spi_resources,
};

#if defined (CONFIG_DAVINCI_SPI_EEPROM_MODULE) || \
    defined (CONFIG_DAVINCI_SPI_EEPROM)
static struct mtd_partition spi_partitions[] = {
	/* UBL in first sector */
	{
	 .name = "UBL",
	 .offset = 0,
	 .size = SZ_16K,
	 .mask_flags = MTD_WRITEABLE,
	 },
	/* User data in the next sector */
	{
	 .name = "data",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	 .mask_flags = 0,
	 }
};

struct davinci_eeprom_info davinci_8k_spi_eeprom_info = {
	.eeprom_size = 8192,
	.page_size = 32,
	.page_mask = 0x001F,
	.chip_sel = SCS0_SELECT,
	.parts = NULL,
	.nr_parts = 0,
	.commit_delay = 3,
};

struct davinci_eeprom_info davinci_32k_spi_eeprom_info = {
	.eeprom_size = 32768,
	.page_size = 64,
	.page_mask = 0x003F,
	.chip_sel = SCS0_SELECT,
	.parts = spi_partitions,
	.nr_parts = ARRAY_SIZE(spi_partitions),
	.commit_delay = 5,
};
#endif

/*Put slave specific information in this array.*/
/*For more information refer the table at the end of file tnetd84xx_spi_cs.c*/
static struct spi_board_info dm6467_spi_board_info[] = {
#if defined (CONFIG_DAVINCI_SPI_EEPROM_MODULE) || \
    defined (CONFIG_DAVINCI_SPI_EEPROM)
	{
	 .modalias = DAVINCI_SPI_EEPROM_NAME,
	 .platform_data = &davinci_32k_spi_eeprom_info,
	 .mode = SPI_MODE_0,
	 .irq = 0,
	 .max_speed_hz = 2 * 1000 * 1000 /* max sample rate at 3V */ ,
	 .bus_num = 65535,
	 .chip_select = 0,
	 },
#endif
};

static struct spi_board_info dm355_spi_board_info[] = {
#if defined (CONFIG_DAVINCI_SPI_EEPROM_MODULE) || \
    defined (CONFIG_DAVINCI_SPI_EEPROM)
	{
	 .modalias = DAVINCI_SPI_EEPROM_NAME,
	 .platform_data = &davinci_8k_spi_eeprom_info,
	 .mode = SPI_MODE_0,
	 .irq = 0,
	 .max_speed_hz = 2 * 1000 * 1000 /* max sample rate at 3V */ ,
	 .bus_num = 65535,
	 .chip_select = 0,
	 },
#endif
#if defined (CONFIG_LOOPBACK_SPI_MODULE) || defined (CONFIG_LOOPBACK_SPI)
	{
	 .modalias = "LOOPB",
	 .platform_data = &eeprom1_info,
	 .mode = SPI_MODE_0,
	 .irq = 0,
	 .max_speed_hz = 2 * 1000 * 1000 /* max sample rate at 3V */ ,
	 .bus_num = 0,
	 .chip_select = 0,
	 },
#endif
#if defined (CONFIG_DLCD_SPI) || defined (CONFIG_DLCD_SPI_MODULE)
	{
	 .modalias = "DLCD_SPI",
	 .platform_data = &eeprom1_info,
	 .mode = SPI_MODE_0,
	 .irq = 0,
	 .max_speed_hz = 2 * 1000 * 1000 /* max sample rate at 3V */ ,
	 .bus_num = 0,
	 .chip_select = 0,
	 },
#endif
};

/*
 * This function initializes the GPIOs used by the SPI module
 * and it also registers the spi mastere device with the platform
 * and the spi slave devices with the spi bus
 */
static int __init davinci_spi_board_init(void)
{
	int ret = 0;
	int size;
	struct spi_board_info *davinci_board_info;

	if (cpu_is_davinci_dm6467()) {
		davinci_board_info = dm6467_spi_board_info;
		size = ARRAY_SIZE(dm6467_spi_board_info);
		davinci_spi_device.resource = dm646x_spi_resources;
		davinci_spi_device.num_resources =
		    ARRAY_SIZE(dm646x_spi_resources);
		davinci_spi_device.dev.platform_data =
		    &dm646x_spi_platform_data;
	} else if (cpu_is_davinci_dm355()) {
		davinci_board_info = dm355_spi_board_info;
		size = ARRAY_SIZE(dm355_spi_board_info);
		davinci_spi_device.resource = dm355_spi_resources;
		davinci_spi_device.num_resources =
		    ARRAY_SIZE(dm355_spi_resources);
		davinci_spi_device.dev.platform_data = &dm355_spi_platform_data;
	} else {
		printk (KERN_INFO "davinci_spi_board_init: NO spi support\n");
		return 0;
	}

	/* Register the slave devices present in the board with SPI subsytem */
	ret = spi_register_board_info(davinci_board_info, size);

	/* Register the master controller with platform */
	(void)platform_device_register(&davinci_spi_device);

	return 0;
}

static void __exit davinci_spi_board_exit(void)
{
	/* nothing to be done */
}

module_init(davinci_spi_board_init);
module_exit(davinci_spi_board_exit);
