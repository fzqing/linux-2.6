/*
 * Mapping for PPC970MP TigerWood2 Eval Board BootROM flash and FRAM
 *
 * This was derived from the luan.c
 *
 * Ruslan Sushko <rsushko@ru.mvista.com>
 *
 * Copyright (C) 2002-2006 MontaVista Software Inc.
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
#include <linux/version.h>
#include <asm/io.h>

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

static struct mtd_info *tw2_bootrom_mtd = NULL;
static struct mtd_info *tw2_fram_mtd = NULL;

static struct map_info tw2_bootrom_map = {
        .name =         "TigerWood BootROM",
        .size =         1024 * 1024,
        .bankwidth =	1,
	.phys =		0xFFF00000
};

static struct map_info tw2_fram_map = {
        .name =         "TigerWood FRAM",
        .size =         1024 * 1024,
        .bankwidth =	1,
	.phys =		0xFFC00000
};

static struct mtd_partition tw2_bootflash_partitions[] = {
        {
                .name =   "PIBS",
                .offset = 0x0,
                .size =   0x100000,
        }
};

static struct mtd_partition tw2_fram_partitions[] = {
        {
                .name =   "FRAM",
                .offset = 0x0,
                .size =   0x8000,
        }
};

static int __init
init_tw2_mtd_bootrom(void)
{
	tw2_bootrom_map.virt =
		ioremap(tw2_bootrom_map.phys, tw2_bootrom_map.size);
	if (!tw2_bootrom_map.virt) {
		printk("Failed to ioremap BootROM flash\n");
                return -EIO;

	}
	simple_map_init(&tw2_bootrom_map);

        tw2_bootrom_mtd = do_map_probe("jedec_probe", &tw2_bootrom_map);
        if (tw2_bootrom_mtd) {
                tw2_bootrom_mtd->owner = THIS_MODULE;
                add_mtd_partitions(tw2_bootrom_mtd, tw2_bootflash_partitions,
				NB_OF(tw2_bootflash_partitions));
        } else {
		iounmap(tw2_bootrom_map.virt);
                printk("map probe failed for BootROM Flash\n");
                return -ENXIO;
        }
	return 0;
}


static int __init
init_tw2_mtd_fram(void)
{
	tw2_fram_map.virt= ioremap(tw2_fram_map.phys, tw2_bootrom_map.size);
	if (!tw2_fram_map.virt) {
		printk("Failed to ioremap FRAM flash\n");
                return -EIO;

	}
	simple_map_init(&tw2_bootrom_map);

        tw2_fram_mtd = do_map_probe("map_ram", &tw2_fram_map);
        if (tw2_fram_mtd) {
                tw2_fram_mtd->owner = THIS_MODULE;
                add_mtd_partitions(tw2_fram_mtd, tw2_fram_partitions,
				NB_OF(tw2_fram_partitions));
        } else {
                printk("map probe failed for FRAM\n");
		iounmap(tw2_fram_map.virt);
                return -ENXIO;
        }
	return 0;
}


static int __init
init_tw2_flash(void)
{
	int fram_res, brom_res;
	brom_res = init_tw2_mtd_bootrom();
	fram_res = init_tw2_mtd_fram();
	if ( fram_res && brom_res ) {
		/* Both MTD drivers failed */
		return -EIO;
	}
	return 0;
}

static void __exit
cleanup_tw2_flash(void)
{
        if (tw2_bootrom_mtd) {
		del_mtd_partitions(tw2_bootrom_mtd);
		map_destroy(tw2_bootrom_mtd);
		iounmap(tw2_bootrom_map.virt);
	}
	if (tw2_fram_mtd) {
		del_mtd_partitions(tw2_fram_mtd);
		map_destroy(tw2_fram_mtd);
		iounmap(tw2_fram_map.virt);
	}
}

module_init(init_tw2_flash);
module_exit(cleanup_tw2_flash);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ruslan V. Sushko <rsushko@ru.mvista.com>");
MODULE_DESCRIPTION("MTD map and partitions for IBM Tigerwood 2 board");

