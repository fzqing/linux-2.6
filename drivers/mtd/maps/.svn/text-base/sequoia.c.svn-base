/*
 * Mapping for Sequoia (440EPX) and Rainier (440GRX) flash
 *
 * Copyright 2006 MontaVista Software Inc.
 *
 * Copyright (c) 2005 DENX Software Engineering
 * Stefan Roese <sr@denx.de>
 *
 * Based on original work by
 *      Matt Porter <mporter@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#define WINDOW_ADDR 0x1fc000000ull
#define WINDOW_SIZE 0x04000000

static struct mtd_partition sequoia_partitions[] = {
	{
		.name = "kernel",	/* linux kernel */
		.size = 0x180000,
	},
	{
		.name = "ramdisk",	/* ramdisk */
		.size = 0x200000,
	},
	{
		.name = "file system",	/* jffs2 file system */
	},
	{
		.name = "kozio",	/* diagnostics */
		.size = 0x140000,
	},
	{
		.name = "env",		/* u-boot environment */
		.size = 0x40000,
	},
	{
		.name = "u-boot",	/* boot loader */
		.size = 0x60000,
	}
};
#define NUM_PARTITIONS (sizeof(sequoia_partitions)/sizeof(sequoia_partitions[0]))

struct map_info sequoia_flash_map = {
	.name = "AMCC440-flash",
	.size = WINDOW_SIZE,
	.bankwidth = 2,
};

static struct mtd_info *sequoia_mtd;

int __init init_sequoia_flash(void)
{
	printk(KERN_NOTICE "sequoia: flash mapping: %x at %llx\n",
	       WINDOW_SIZE, WINDOW_ADDR);

	sequoia_flash_map.virt = ioremap64(WINDOW_ADDR, WINDOW_SIZE);

	if (!sequoia_flash_map.virt) {
		printk("init_sequoia_flash: failed to ioremap\n");
		return  -EIO;
	}
	simple_map_init(&sequoia_flash_map);

	sequoia_mtd = do_map_probe("cfi_probe", &sequoia_flash_map);

	if (sequoia_mtd) {
		unsigned offset, u, size = sequoia_mtd->size;
		/* subtract partition sizes from total space */
		for (u=0; u<NUM_PARTITIONS; u++) {
			size -= sequoia_partitions[u].size;
		}
		/* set user size to any unused space */
		if (size < sequoia_mtd->size) {
			for (u=0; u<NUM_PARTITIONS; u++) {
				if (!sequoia_partitions[u].size) {
					sequoia_partitions[u].size = size;
					break;
				}
			}
		}
		/* set offsets for all partitions */
		for (u=offset=0; u<NUM_PARTITIONS; u++) {
			sequoia_partitions[u].offset = offset;
			offset += sequoia_partitions[u].size;
		}

		sequoia_mtd->owner = THIS_MODULE;
		return  add_mtd_partitions(sequoia_mtd,
					   sequoia_partitions,
					   NUM_PARTITIONS);
	}

	return  -ENXIO;
}

static void __exit cleanup_sequoia_flash(void)
{
	if (sequoia_mtd) {
		del_mtd_partitions(sequoia_mtd);
		/* moved iounmap after map_destroy - armin */
		map_destroy(sequoia_mtd);
		iounmap((void *)sequoia_flash_map.virt);
	}
}

module_init(init_sequoia_flash);
module_exit(cleanup_sequoia_flash);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("MTD map and partitions for AMCC 440EPx/GRx boards");

