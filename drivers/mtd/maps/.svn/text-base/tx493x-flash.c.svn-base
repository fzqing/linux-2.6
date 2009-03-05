/*
 * drivers/mtd/maps/tx493x-flash.c
 *
 * Mapping of flash on RBTX493x board.
 *
 * Based on cstm_mips_ixx.c by MontaVista Software Inc.
 *
 * Author: source@mvista.com
 *
 * Copyright (C) 2000-2001,2005 Toshiba Corporation
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX493x in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/bootinfo.h>

/* board and partition description */

#define MAX_PHYSMAP_PARTITIONS	4
struct tx493x_info {
	char *name;
	unsigned long window_addr;
	unsigned long window_size;
	int buswidth;
	char *drvname;
	struct mtd_info *mtd_info;
#ifdef CONFIG_MTD_PARTITIONS
	int num_partitions;
	char *part_names[MAX_PHYSMAP_PARTITIONS];
	struct mtd_partition partitions[MAX_PHYSMAP_PARTITIONS];
#endif
};

#define PHYSMAP_NUMBER  8	/* number of board desc structs needed, one per contiguous flash type */
static struct tx493x_info tx493x_desc[PHYSMAP_NUMBER];
static struct map_info tx493x_maps[PHYSMAP_NUMBER];

int __init init_txmtd(void)
{
	int i;
	struct mtd_info *mymtd = NULL;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *parts;
	int j;
#endif

	/* tx493x_desc[] is initialized by early_txmtd_setup() */
	/* Initialize mapping */
	for (i = 0; i < PHYSMAP_NUMBER; i++) {
		if (!tx493x_desc[i].name)
			continue;
		pr_debug("tx493x flash device: probe %lx at %lx\n",
		       tx493x_desc[i].window_size, tx493x_desc[i].window_addr);
		tx493x_maps[i].phys = tx493x_desc[i].window_addr;
		tx493x_maps[i].virt =
		    (unsigned long)ioremap(tx493x_desc[i].window_addr,
					   tx493x_desc[i].window_size);
		if (!tx493x_maps[i].virt) {
			printk(KERN_WARNING "Failed to ioremap\n");
			return -EIO;
		}
		tx493x_maps[i].name = tx493x_desc[i].name;
		tx493x_maps[i].size = tx493x_desc[i].window_size;
		tx493x_maps[i].bankwidth = tx493x_desc[i].buswidth;
		simple_map_init(&tx493x_maps[i]);
		pr_debug("tx493x: ioremap is %x\n", (unsigned int)(tx493x_maps[i].virt));
	}

	for (i = 0; i < PHYSMAP_NUMBER; i++) {
		struct tx493x_info *txinfo = &tx493x_desc[i];
		if (!tx493x_maps[i].name)
			continue;
		if (txinfo->drvname) {
			/* probe only specified chipdriver */
			mymtd =
			    (struct mtd_info *)do_map_probe(txinfo->drvname,
							    &tx493x_maps[i]);
		} else {
			/* probe cfi then try jedec */
			mymtd =
			    (struct mtd_info *)do_map_probe("cfi_probe",
							    &tx493x_maps[i]);
			pr_debug("phymap %d cfi_probe: mymtd is %x\n", i, (unsigned int)mymtd);
			if (!mymtd) {
				mymtd =
				    (struct mtd_info *)
				    do_map_probe("jedec_probe",
						 &tx493x_maps[i]);
				pr_debug("tx493x %d jedec: mymtd is %x\n", i, (unsigned int)mymtd);
			}
		}
		if (!mymtd)
			continue;
		mymtd->owner = THIS_MODULE;
		/* true map size */
		tx493x_maps[i].size = mymtd->size;
		/* If this window contains boot vector, adjust the map area.
		 * 1f000000-1f3fffff to 1fc00000-1fffffff,
		 * 1f000000-1f7fffff to 1f800000-1fffffff, etc. */
		if (txinfo->window_addr <= 0x1fc00000 &&
		    txinfo->window_addr + txinfo->window_size > 0x1fc00000) {
			txinfo->window_addr =
			    0x1fc00000 / tx493x_maps[i].size *
			    tx493x_maps[i].size;
			iounmap((void *)tx493x_maps[i].virt);
			tx493x_maps[i].virt =
			    (unsigned long)ioremap(txinfo->window_addr,
						   tx493x_maps[i].size);
		}
		pr_debug("tx493x flash device(%s): %lx at %lx\n",
		       tx493x_maps[i].name, tx493x_maps[i].size,
		       txinfo->window_addr);
		txinfo->mtd_info = mymtd;
		tx493x_maps[i].map_priv_2 = 1;	/* mark invalidate */
#ifdef CONFIG_MTD_PARTITIONS
		parts = &txinfo->partitions[0];
		if (txinfo->num_partitions == 0) {
			/* initialize txinfo->partitions[] */
			if (txinfo->window_addr < 0x1fc00000 &&
			    txinfo->window_addr + mymtd->size > 0x1fc00000) {
				/* split boot mtd device */
				parts[0].offset =
				    0x1fc00000 - txinfo->window_addr;
				parts[0].size =
				    txinfo->window_addr + mymtd->size -
				    0x1fc00000;
				parts[1].offset = 0;
				parts[1].size =
				    0x1fc00000 - txinfo->window_addr;
				txinfo->num_partitions = 2;
			} else {
				if(!strcmp(txinfo->name, "System ROM")) {
					/* split boot config mtd device */
					parts[0].offset = 0xfe0000;
					parts[0].size = 0x20000;
					parts[0].mask_flags = MTD_WRITEABLE;
					parts[0].name = kmalloc(strlen("boot config") + 1, GFP_KERNEL);
					strcpy(parts[0].name, "boot config");
					parts[1].offset = 0;
					parts[1].size = 0xfe0000;
					parts[1].name = kmalloc(strlen("User FS") + 1, GFP_KERNEL);
					strcpy(parts[1].name, "User FS");
					txinfo->num_partitions = 2;
				} else {
					parts->size = txinfo->window_size;
					txinfo->num_partitions = 1;
				}
			}
		}
		for (j = 0; j < txinfo->num_partitions; j++) {
			int isboot =
			    (txinfo->window_addr + parts[j].offset
			     == 0x1fc00000);
			char buf[128];
			if (parts[j].name)
				strcpy(buf, parts[j].name);
			else if (txinfo->num_partitions == 1)
				strcpy(buf, mymtd->name);
			else
				sprintf(buf, "%s (part%d)", mymtd->name, j);
			if (isboot)
				strcat(buf, " (boot)");
			txinfo->part_names[j] =
			    kmalloc(strlen(buf) + 1, GFP_KERNEL);
			if (txinfo->part_names[j]) {
				strcpy(txinfo->part_names[j], buf);
				parts[j].name = txinfo->part_names[j];
			} else {
				parts[j].name = mymtd->name;
			}
		}
		add_mtd_partitions(mymtd, parts, txinfo->num_partitions);
#else
		add_mtd_device(mymtd);
#endif
	}
	if (!mymtd)
		return -ENXIO;
	return 0;
}

static void __exit cleanup_txmtd(void)
{
	int i, j;
	struct mtd_info *mymtd;

	for (i = 0; i < PHYSMAP_NUMBER; i++) {
		mymtd = tx493x_desc[i].mtd_info;
		if (mymtd) {
#ifdef CONFIG_MTD_PARTITIONS
			del_mtd_partitions(mymtd);
#else
			del_mtd_device(mymtd);
#endif
			map_destroy(mymtd);
		}
		if (tx493x_maps[i].virt) {
			iounmap((void *)tx493x_maps[i].virt);
			tx493x_maps[i].virt = 0;
		}
#ifdef CONFIG_MTD_PARTITIONS
		for (j = 0; j < MAX_PHYSMAP_PARTITIONS; j++) {
			if (tx493x_desc[i].part_names[j])
				kfree(tx493x_desc[i].part_names[j]);
			tx493x_desc[i].part_names[j] = 0;
		}
#endif
	}
}

module_init(init_txmtd);
module_exit(cleanup_txmtd);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MTD map driver for TX493x boards");

#ifndef MODULE
int __init early_txmtd_setup(int no, char *name,
			     unsigned long addr, unsigned long size,
			     int buswidth, int num_partitions,
			     struct mtd_partition *parts, char *drvname)
{
	int i;

	if (no < 0 || no >= PHYSMAP_NUMBER)
		return -EINVAL;
	if (num_partitions < 0 || num_partitions >= MAX_PHYSMAP_PARTITIONS)
		return -EINVAL;
	tx493x_desc[no].name = name;
	tx493x_desc[no].window_addr = addr;
	tx493x_desc[no].window_size = size;
	tx493x_desc[no].buswidth = buswidth;
#ifdef CONFIG_MTD_PARTITIONS
	tx493x_desc[no].num_partitions = num_partitions;
	for (i = 0; i < num_partitions; i++)
		tx493x_desc[no].partitions[i] = parts[i];
#endif
	tx493x_desc[no].drvname = drvname;
	return 0;
}

#ifdef CONFIG_MTD_PARTITIONS
/* mtdpart=SZ[@[OFS]][+SZ[@[OFS]]]...[,SZ[@[OFS]][+SZ[@[OFS]]]...]... */
static int __init txmtd_part_setup(char *str)
{
	char buf[128];
	char *p1, *p2, *s;
	int i, j;

	strncpy(buf, str, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = 0;
	p1 = buf;
	/* for each mtd device... */
	for (i = 0; p1 && i < PHYSMAP_NUMBER; i++) {
		p2 = strsep(&p1, ",");
		if (!*p2)
			continue;	/* keep early configurations */
		memset(tx493x_desc[i].partitions, 0,
		       sizeof(tx493x_desc[i].partitions));
		/* for each mtd partition... */
		for (j = 0; p2 && j < MAX_PHYSMAP_PARTITIONS; j++) {
			struct mtd_partition *part;
			s = strsep(&p2, "+");
			part = &tx493x_desc[i].partitions[j];
			part->size = memparse(s, &s);
			part->offset = (*s == '@') ?
			    memparse(s + 1, &s) : MTDPART_OFS_NXTBLK;
		}
		tx493x_desc[i].num_partitions = j;
	}
	return 1;
}

__setup("mtdpart=", txmtd_part_setup);
#endif				/* CONFIG_MTD_PARTITIONS */
#endif				/* !MODULE */
