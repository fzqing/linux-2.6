/*
 * Mapping for Luan user flash
 *
 * This was dirived from the ocotea.c
 *
 * Matt Porter <mporter@mvista.com>
 *
 * Copyright 2002-2003 MontaVista Software Inc.
 * (C) Copyright IBM Corp. 2004
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Changes by IBM Corp.
 * $00 03/04/2004 MN creation for the Luan board
 * $BU 04/09/04   FM  Large Flash partionning changes
 *                    (Linux Bring Up)
 *
 * 24/03/2005 Added SRAM and FRAM MTD support
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/config.h>
#include <linux/version.h>
#include <asm/io.h>

static struct mtd_info *flash;

static struct map_info luan_sram_map = {
	.name = "Luan SRAM",
	.size = LUAN_SRAM_SIZE,
	.bankwidth = 1,
};

static struct map_info luan_fram_map = {
	.name = "Luan FRAM",
	.size = LUAN_FRAM_SIZE,
	.bankwidth = 1,
};

static struct map_info luan_small_map = {
	.name = "Luan small flash",
	.size = LUAN_SMALL_FLASH_SIZE,
	.bankwidth = 1,
};

static struct map_info luan_large_map = {
	.name = "Luan large flash",
	.size = LUAN_LARGE_FLASH_SIZE,
	.bankwidth = 1,
};

static struct mtd_partition luan_sram_partitions[] = {
	{
	 .name = "SRAM",
	 .offset = 0x0,
	 .size = 0x100000,
	 }
};

static struct mtd_partition luan_fram_partitions[] = {
	{
	 .name = "FRAM",
	 .offset = 0x0,
	 .size = 0x8000,
	 }
};

static struct mtd_partition luan_small_partitions[] = {
	{
	 .name = "Pibs",
	 .offset = 0x0,
	 .size = 0x100000,
	 }
};

static struct mtd_partition luan_large_partitions[] = {
	{
	 .name = "Linux Kernel",
	 .offset = 0,
	 .size = 0x100000,	/* new size:    change from 0x300000 to 0x100000 */
	 },
	{
	 .name = "Free Area",
	 .offset = 0x100000,	/* new offset:  change from 0x300000 to 0x100000 */
	 .size = 0x300000,	/* new size:    change from 0x100000 to 0x300000 */
	 }
};

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

int __init init_luan(void)
{
	u8 fpga0_reg;
	u8 *fpga0_adr;
	unsigned long long small_flash_base, large_flash_base;
	unsigned long long sram_base;

	fpga0_adr = ioremap64(LUAN_FPGA_REG_0, 16);
	if (!fpga0_adr)
		return -ENOMEM;

	fpga0_reg = readb((unsigned long)fpga0_adr);
	iounmap(fpga0_adr);

	if (LUAN_BOOT_LARGE_FLASH(fpga0_reg)) {

		large_flash_base = LUAN_LARGE_FLASH_LOW;

		if ((fpga0_reg & LUAN_CONFIG_MASK) == LUAN_CONFIG_1) {
			small_flash_base = LUAN_SMALL_FLASH_HIGH;
			sram_base = LUAN_SRAM_HIGH;
		} else if ((fpga0_reg & LUAN_CONFIG_MASK) == LUAN_CONFIG_2) {
			small_flash_base = LUAN_SMALL_FLASH_HIGH2;
			sram_base = LUAN_SRAM_HIGH2;
		} else {
			printk("invalid board config: fpga0_reg= %x\n",
			       fpga0_reg);
			return -EIO;
		}
	} else {
		large_flash_base = LUAN_LARGE_FLASH_HIGH;

		if ((fpga0_reg & LUAN_CONFIG_MASK) == LUAN_CONFIG_3) {
			small_flash_base = LUAN_SMALL_FLASH_LOW;
			sram_base = LUAN_SRAM_LOW;
		} else if ((fpga0_reg & LUAN_CONFIG_MASK) == LUAN_CONFIG_4) {
			small_flash_base = LUAN_SMALL_FLASH_LOW4;
			sram_base = LUAN_SRAM_LOW4;
		} else {
			printk("invalid board config: fpga0_reg= %x\n",
			       fpga0_reg);
			return -EIO;
		}
	}

	luan_small_map.phys = small_flash_base;
	luan_small_map.virt =
	    (unsigned long)ioremap64(small_flash_base, luan_small_map.size);

	if (!luan_small_map.virt) {
		printk("Failed to ioremap small flash\n");
		return -EIO;
	}

	simple_map_init(&luan_small_map);

	flash = do_map_probe("map_rom", &luan_small_map);
	if (flash) {
		flash->owner = THIS_MODULE;
		add_mtd_partitions(flash, luan_small_partitions,
				   NB_OF(luan_small_partitions));
	} else {
		printk("map probe failed for small flash\n");
		return -ENXIO;
	}

	luan_large_map.phys = large_flash_base;
	luan_large_map.virt =
	    (unsigned long)ioremap64(large_flash_base, luan_large_map.size);

	if (!luan_large_map.virt) {
		printk("Failed to ioremap large flash\n");
		return -EIO;
	}

	simple_map_init(&luan_large_map);

	flash = do_map_probe("cfi_probe", &luan_large_map);
	if (flash) {
		flash->owner = THIS_MODULE;
		add_mtd_partitions(flash, luan_large_partitions,
				   NB_OF(luan_large_partitions));
	} else {
		printk("map probe failed for large flash\n");
		return -ENXIO;
	}

	luan_sram_map.phys = sram_base;
	luan_sram_map.virt =
	    (unsigned long)ioremap64(sram_base, luan_sram_map.size);

	if (!luan_sram_map.virt) {
		printk("Failed to ioremap SRAM\n");
		return -EIO;
	}

	simple_map_init(&luan_sram_map);

	flash = do_map_probe("map_ram", &luan_sram_map);
	if (flash) {
		flash->owner = THIS_MODULE;
		add_mtd_partitions(flash, luan_sram_partitions,
				   NB_OF(luan_sram_partitions));
	} else {
		printk("map probe failed for SRAM\n");
		return -ENXIO;
	}

	luan_fram_map.phys = LUAN_FRAM_ADDR;
	luan_fram_map.virt =
	    (unsigned long)ioremap64(LUAN_FRAM_ADDR, luan_fram_map.size);

	if (!luan_fram_map.virt) {
		printk("Failed to ioremap FRAM\n");
		return -EIO;
	}

	simple_map_init(&luan_fram_map);

	flash = do_map_probe("map_ram", &luan_fram_map);
	if (flash) {
		flash->owner = THIS_MODULE;
		add_mtd_partitions(flash, luan_fram_partitions,
				   NB_OF(luan_fram_partitions));
	} else {
		printk("map probe failed for FRAM\n");
		return -ENXIO;
	}
	return 0;
}

static void __exit cleanup_luan(void)
{
	if (flash) {
		del_mtd_partitions(flash);
		map_destroy(flash);
	}

	if (luan_small_map.virt) {
		iounmap((void *)luan_small_map.virt);
		luan_small_map.virt = 0;
	}

	if (luan_large_map.virt) {
		iounmap((void *)luan_large_map.virt);
		luan_large_map.virt = 0;
	}

	if (luan_fram_map.virt) {
		iounmap((void *)luan_fram_map.virt);
		luan_fram_map.virt = 0;
	}

	if (luan_sram_map.virt) {
		iounmap((void *)luan_sram_map.virt);
		luan_sram_map.virt = 0;
	}
}

module_init(init_luan);
module_exit(cleanup_luan);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matt Porter <mporter@mvista.com>");
MODULE_DESCRIPTION("MTD map and partitions for IBM 440SP Luan boards");
