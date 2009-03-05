/*******************************************************************************

	omap-nor-flash.c
	Self-Contained MTD driver for OMAP NOR Flash
	- Supports Intel 28F256L18T and Intel 28F128L18T
	- Automatically detects which one of the supported flashes is used
        - Automatically detects the location of the supported flashes (CS3 or CS2B)

	Author: MontaVista Software, Inc. <source@mvista.com>
	Copyright (c) 2003 MontaVista Software, Inc.

	The code for this driver is based on board/omap1610inn/flash.c from 
	U-Boot 0.4.3:
	
	 * (C) Copyright 2001
	 * Kyle Harris, Nexus Technologies, Inc. kharris@nexus-tech.net
	 *
	 * (C) Copyright 2001
	 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
	 *
	 * (C) Copyright 2003
	 * Texas Instruments, <www.ti.com>
	 * Kshitij Gupta <Kshitij@ti.com>

	The code for this driver is also based on syncflash.c:

	 * MTD driver for Micron SyncFlash flash memory.
	 *
	 * Author: Jon McClintock <jonm@bluemug.com>
	 *
	 * Based loosely upon the LART flash driver, authored by 
	 * Abraham vd Merwe
	 * <abraham@2d3d.co.za>.
	 *
	 * Copyright 2003, Blue Mug, Inc. for Motorola, Inc.
	 * Copyright 2003, MontaVista Software, Inc
	 *
	 * References:
	 *
	 *
	 *     [1] MTD internal API documentation
	 *             - http://www.linux-mtd.infradead.org/tech/
	 *

	Modifications:
	Feb 2004, Texas Instruments, <www.ti.com>
	- Ported to 2.6 Kernel (Feb 2004)
	- Added address probing (Sept 2004)

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/mtd/mtd.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/mtd/partitions.h>
#include <asm/arch/hardware.h>

#ifdef CONFIG_MTD_PARTITIONS_MODULE
#define CONFIG_MTD_PARTITIONS 1
#endif

#define NB_OF(x) (sizeof (x) / sizeof (x[0]))

/*
 * General flash configuration parameters.
 */
#define OMAP_BOOTLOADER_LEN 0x20000
#define OMAP_PARAMS_LEN 0x20000
#define OMAP_KERNEL_LEN 0x200000
#define BUSWIDTH               2	/*warning: this is used only in some functions, the code is specific to 16-bit data bus */
#define INTEL_MANUFACT	0x0089
#define INTEL_ID_28F256L18T  0x880D	/* 256M = 128K x 255 + 32k x 4  */
#define INTEL_ID_28F128L18T  0x880C	/* 128M = 128K x 127 + 32k x 4  */

void __exit omap_flash_exit(void);
int __init omap_flash_init(void);
static int omap_flash_write(struct mtd_info *mtd, loff_t to,
				size_t len, size_t * retlen,
				const u_char * buf);
static int omap_flash_write_word(__u32 offset, __u16 x);
static int omap_flash_read(struct mtd_info *mtd, loff_t from,
			       size_t len, size_t * retlen, u_char * buf);
static int omap_flash_erase(struct mtd_info *mtd, struct erase_info *instr);
static inline int omap_flash_sector_erase(__u32 offset);
static int omap_flash_probe(void);

static struct mtd_info mtd;

static struct mtd_erase_region_info erase_regions[2];

static unsigned long omap_nor_flash_base;

DECLARE_MUTEX(omap_flash_lock);

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition omap_partitions[] = {
	/* bootloader */
	{
	      name:"bootloader",
	      offset:0,
	      size:OMAP_BOOTLOADER_LEN,
	      mask_flags:MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params */
	{
	      name:"params",
	      offset:MTDPART_OFS_APPEND,
	      size:OMAP_PARAMS_LEN,
	      mask_flags:0, 
	},
	/* kernel */
	{
	      name:"kernel",
	      offset:MTDPART_OFS_APPEND,
	      size:OMAP_KERNEL_LEN,
	      mask_flags:0
	},
	/* file system */
	{
	      name:"file system",
	      offset:MTDPART_OFS_APPEND,
	      size:MTDPART_SIZ_FULL,
	      mask_flags:0
	}
};
#endif

static char module_name[] = "OMAP NOR FLASH";

#define omap_flash_inw(o) readw((omap_nor_flash_base+(__u32)o))
#define omap_flash_outw(o,d) writew((__u16)(d),(omap_nor_flash_base+(__u32)o))

/* Probe for NOR Flash on OMAP board at address omap_nor_flash_base */
static int
omap_flash_probe(void)
{
	__u16 mfgid, devid;
        __u32 offset;
 	int i, j;

	memset(&mtd, 0, sizeof (mtd));
	down(&omap_flash_lock);
	/* Write auto select command: read Manufacturer ID */
	omap_flash_outw(0, 0x0090);

	mfgid = omap_flash_inw(0);	/*manufacturer ID */

	switch (mfgid) {

	case INTEL_MANUFACT:
		break;
	default:
		omap_flash_outw(0, 0x00FF);	/* restore read mode */
		up(&omap_flash_lock);
		printk(KERN_ERR
		       "%s ERROR: no supported Flash device found\n",
		       module_name);
		return -ENXIO;	/* no or unknown flash  */
	}

	devid = omap_flash_inw(2);	/* device ID  */

	switch (devid) {

	case INTEL_ID_28F256L18T:
		erase_regions[0].offset = 0;
		erase_regions[0].erasesize = 128 * 1024;
		erase_regions[0].numblocks = 255;
		erase_regions[1].offset =
		    erase_regions[0].erasesize * erase_regions[0].numblocks;
		erase_regions[1].erasesize = 32 * 1024;
		erase_regions[1].numblocks = 4;
		printk(KERN_INFO "%s: Intel 28F256L18T found at 0x%lx\n", module_name, omap_nor_flash_base);
		break;

	case INTEL_ID_28F128L18T:
		erase_regions[0].offset = 0;
		erase_regions[0].erasesize = 128 * 1024;
		erase_regions[0].numblocks = 127;
		erase_regions[1].offset =
		    erase_regions[0].erasesize * erase_regions[0].numblocks;
		erase_regions[1].erasesize = 32 * 1024;
		erase_regions[1].numblocks = 4;
		printk(KERN_INFO "%s: Intel 28F128L18T found at 0x%lx\n", module_name, omap_nor_flash_base);
		break;

	default:
		omap_flash_outw(0, 0x00FF);	/* restore read mode */
		up(&omap_flash_lock);
		printk(KERN_ERR
		       "%s ERROR: no supported Flash device found\n",
		       module_name);
		return -ENXIO;	/* no or unknown flash  */
	}
	omap_flash_outw(0, 0x00FF);	/* restore read mode */

	mtd.name = module_name;
	mtd.type = MTD_NORFLASH;
	mtd.flags = MTD_CAP_NORFLASH;

	mtd.erasesize = 128 * 1024;
	mtd.numeraseregions = NB_OF(erase_regions);
	mtd.eraseregions = erase_regions;
	for (i = 0; i < NB_OF(erase_regions); i++) {
		mtd.size +=
		    erase_regions[i].erasesize * erase_regions[i].numblocks;
		offset = erase_regions[i].offset;
		for (j = 0; j < erase_regions[i].numblocks; j++) {

			/* this sends the clear lock bit command */
			omap_flash_outw(offset, 0x0060);
			omap_flash_outw(offset, 0x00D0);
			omap_flash_outw(offset, 0x00FF);/* reset to read mode */
			offset += erase_regions[i].erasesize;
		}
	}

	up(&omap_flash_lock);

	mtd.owner = THIS_MODULE;

	mtd.erase = omap_flash_erase;
	mtd.read = omap_flash_read;
	mtd.write = omap_flash_write;

	return 0;
}

/* Erase a flash sector */
static inline int
omap_flash_sector_erase(__u32 offset)
{
	__u16 status;

	down(&omap_flash_lock);
	/*erase sector */
	omap_flash_outw(offset, 0x0050);	/* clear status register */
	omap_flash_outw(offset, 0x0020);	/* erase setup */
	omap_flash_outw(offset, 0x00D0);	/* erase confirm */

	/* wait for erase complete while polling the status register */
	while (((status = omap_flash_inw(offset)) & 0x0080) != 0x0080)
		schedule_timeout(1);

	omap_flash_outw(offset, 0x0050);	/* clear status register */
	omap_flash_outw(offset, 0x00FF);	/* reset to read mode    */
	up(&omap_flash_lock);

	/* process status register */
	if (status & (1 << 3)) {
		printk(KERN_ERR
		       "%s ERROR: erasing at address 0x%.8x: Vpp range error \n",
		       module_name, offset);
		return -EPERM;
	}
	if ((status & (3 << 4)) == (3 << 4)) {
		printk(KERN_ERR
		       "%s ERROR: erasing at address 0x%.8x: command sequence error\n",
		       module_name, offset);
		return -EPERM;
	}
	if (status & (1 << 5)) {
		printk(KERN_ERR
		       "%s ERROR: erasing at address 0x%.8x: block erase error\n",
		       module_name, offset);
		return -ETIMEDOUT;
	}
	if (status & (1 << 1)) {
		printk(KERN_ERR
		       "%s ERROR: erasing at address 0x%.8x: block locked error\n",
		       module_name, offset);
		return -EPERM;
	}

	return 0;
}

/* Pick the proper sector addresses for erasing */
static int
omap_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	__u32 addr, len;
	int i, first;

	/* sanity checks */
	if (instr->addr + instr->len > mtd->size)
		return (-EINVAL);

	/*
	 * check that both start and end of the requested erase are
	 * aligned with the erasesize at the appropriate addresses.
	 *
	 * skip all erase regions which are ended before the start of
	 * the requested erase. Actually, to save on the calculations,
	 * we skip to the first erase region which starts after the
	 * start of the requested erase, and then go back one.
	 */
	for (i = 0; (i < mtd->numeraseregions) &&
	     (instr->addr >= mtd->eraseregions[i].offset); i++) ;
	i--;

	/*
	 * ok, now i is pointing at the erase region in which this
	 * erase request starts. Check the start of the requested
	 * erase range is aligned with the erase size which is in
	 * effect here.
	 */
	if (instr->addr & (mtd->eraseregions[i].erasesize - 1))
		return (-EINVAL);

	/* Remember the erase region we start on */
	first = i;

	/*
	 * next, check that the end of the requested erase is aligned
	 * with the erase region at that address.
	 *
	 * as before, drop back one to point at the region in which
	 * the address actually falls
	 */
	for (;
	     (i < mtd->numeraseregions) &&
	     ((instr->addr + instr->len) >= mtd->eraseregions[i].offset); i++) ;
	i--;

	/* is the end aligned on a block boundary? */
	if ((instr->addr + instr->len) & (mtd->eraseregions[i].erasesize - 1))
		return (-EINVAL);

	addr = instr->addr;
	len = instr->len;

	i = first;

	/* now erase those blocks */
	while (len) {
		if (omap_flash_sector_erase(addr)) {
			instr->state = MTD_ERASE_FAILED;
			return (-EIO);
		}

		addr += mtd->eraseregions[i].erasesize;
		len -= mtd->eraseregions[i].erasesize;

		if (addr == (mtd->eraseregions[i].offset +
			     (mtd->eraseregions[i].erasesize *
			      mtd->eraseregions[i].numblocks)))
			i++;
	}

	instr->state = MTD_ERASE_DONE;
	if (instr->callback)
		instr->callback(instr);

	return (0);
}

/* Read a block of data from flash */
static int
omap_flash_read(struct mtd_info *mtd, loff_t from,
		    size_t len, size_t * retlen, u_char * buf)
{
	/* Sanity checks. */
	if (!len)
		return (0);
	if (from + len > mtd->size)
		return (-EINVAL);

	/* We always read len bytes. */
	*retlen = len;
	down(&omap_flash_lock);
	memcpy(buf, (void *) (omap_nor_flash_base + (__u32) from), len);
	up(&omap_flash_lock);

	return (0);
}

/* Write a flash word. The offset must be on a word boudary */
static int
omap_flash_write_word(__u32 offset, __u16 x)
{
	__u16 status;

	down(&omap_flash_lock);
	/* Check if Flash is (sufficiently) erased */
	if ((omap_flash_inw(offset) & x) != x) {
		printk(KERN_ERR
		       "%s ERROR: cannot write, flash not erased at address 0x%.8x.\n",
		       module_name, offset);
		up(&omap_flash_lock);
		return -EIO;	/*flash not erased */
	}

	omap_flash_outw(offset, 0x0050);	/* clear status register */
	omap_flash_outw(offset, 0x0040);	/* write setup */
	omap_flash_outw(offset, x);

	/* wait for program complete while polling the status register */
	while (((status = omap_flash_inw(offset)) & 0x0080) != 0x0080)
		schedule_timeout(1);

	omap_flash_outw(offset, 0x0050);	/* clear status register */
	omap_flash_outw(offset, 0x00FF);	/* restore read mode */
	up(&omap_flash_lock);

	/* process status register */
	if (status & (1 << 3)) {
		printk(KERN_ERR
		       "%s ERROR: writing at address 0x%.8x: Vpp range error \n",
		       module_name, offset);
		return -EPERM;
	}
	if (status & (1 << 4)) {
		printk(KERN_ERR
		       "%s ERROR: writing at address 0x%.8x: program error \n",
		       module_name, offset);
		return -ETIMEDOUT;
	}
	if (status & (1 << 1)) {
		printk(KERN_ERR
		       "%s ERROR: writing at address 0x%.8x: device protect error\n",
		       module_name, offset);
		return -EPERM;
	}

	return 0;
}

/* Write a block of data to flash. Takes care of the word boundaries */
static int
omap_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
		     size_t * retlen, const u_char * buf)
{
	__u8 tmp[BUSWIDTH];
	int i, n;

	*retlen = 0;

	/* Sanity checks */
	if (!len)
		return (0);
	if (to + len > mtd->size)
		return (-EINVAL);

	/* First, we write a byte padded with original data until we reach a
	 * word boundary. */
	if (to & (BUSWIDTH - 1)) {
		__u32 aligned = to & ~(BUSWIDTH - 1);
		i = to - aligned;

		n = 0;
		down(&omap_flash_lock);
		*((__u16 *) tmp) = omap_flash_inw(aligned);
		up(&omap_flash_lock);

		while (len && i < BUSWIDTH)
			tmp[i++] = buf[n++], len--;

		if (omap_flash_write_word(aligned, *((__u16 *) tmp)))
			return (-EIO);

		to += n;
		buf += n;
		*retlen += n;
	}

	/* Now we write words until we reach a non-word boundary. */
	while (len >= BUSWIDTH) {
		if (omap_flash_write_word(to, *((__u16 *) buf)))
			return (-EIO);

		to += BUSWIDTH;
		buf += BUSWIDTH;
		*retlen += BUSWIDTH;
		len -= BUSWIDTH;
	}

	/* Top up the last unaligned bytes, padded with original data.... */
	if (len & (BUSWIDTH - 1)) {
		i = n = 0;

		down(&omap_flash_lock);
		*((__u16 *) tmp) = omap_flash_inw(to);
		up(&omap_flash_lock);

		while (len--)
			tmp[i++] = buf[n++];

		if (omap_flash_write_word(to, *((__u16 *) tmp)))
			return (-EIO);

		*retlen += n;
	}

	return 0;
}

int __init
omap_flash_init(void)
{
	int result;

	printk("%s: MTD Self-Contained Driver ver. 1.0 size=0x%lx\n", module_name, OMAP_NOR_FLASH_SIZE);

        omap_nor_flash_base = (unsigned long) ioremap(OMAP_NOR_FLASH_START1, OMAP_NOR_FLASH_SIZE);
	result = omap_flash_probe();
	if (result < 0) {
		iounmap((void *) omap_nor_flash_base);
        	omap_nor_flash_base = (unsigned long) ioremap(OMAP_NOR_FLASH_START2, OMAP_NOR_FLASH_SIZE);
 	        result = omap_flash_probe();
	        if (result < 0) {
	                iounmap((void *) omap_nor_flash_base);
 			return (-ENXIO);
		}
	}
#ifndef CONFIG_MTD_PARTITIONS
	result = add_mtd_device(&mtd);
#else
	result = add_mtd_partitions(&mtd,
				    omap_partitions,
				    NB_OF(omap_partitions));
#endif

	/* check result */
	if (result)
		iounmap((void *) omap_nor_flash_base);

	return (result);
}

void __exit
omap_flash_exit(void)
{
#ifndef CONFIG_MTD_PARTITIONS
	del_mtd_device(&mtd);
#else
	del_mtd_partitions(&mtd);
#endif
	iounmap((void *) omap_nor_flash_base);
}

module_init(omap_flash_init);
module_exit(omap_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MontaVista Software Inc.");
MODULE_DESCRIPTION("Self-Contained MTD driver for NOR Flash on OMAP board");
