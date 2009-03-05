/*
 *  drivers/mtd/nand/sequoia_nand.c
 *
 *  Overview:
 *   This is a device driver for the NAND flash devices found on the
 *   IBM 440EPx and 440GRx Evaluation Boards (Sequoia/Rainier).
 *
 *  Author: Wade Farnsworth <wfarnsworth@mvista.com>
 *
 *  Copyright 2005 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/ibm44x.h>
#include <platforms/4xx/sequoia.h>

struct ppc440ep_ndfc_regs {
	uint cmd;
	uint addr;
	uint data;
	uint reserved1;
	uint ecc0;
	uint ecc1;
	uint ecc2;
	uint ecc3;
	uint ecc4;
	uint ecc5;
	uint ecc6;
	uint ecc7;
	uint b0cr;
	uint b1cr;
	uint b2cr;
	uint b3cr;
	uint cr;
	uint sr;
	uint hwctl;
	uint reserved2;
	uint revid;
};

static u8 nand_cs;			/* chip select 0 if boot from nand else 3 */
static struct mtd_info *sequoia_nand0_mtd;
static u8 hwctl;
static struct ppc440ep_ndfc_regs *sequoia_ndfc;

#define UBOOT_SIZE		0x0080000	/* reserve 512k for u-boot */
#define KERNEL_SIZE		0x0180000	/* reserve 1.5m for kernel */
#define NAND_SIZE		0x2000000	/* 32mb device */

static struct mtd_partition nand0_partition_info[] = {
	{
		.name   = "u-boot-nand",
		.offset = 0,
		.size   = UBOOT_SIZE,
	},
	{
		.name   = "kernel-nand",
		.offset = UBOOT_SIZE,
		.size   = KERNEL_SIZE,
	},
	{
	 	.name   = "filesystem",
	 	.offset = UBOOT_SIZE+KERNEL_SIZE,
	 	.size   = NAND_SIZE-UBOOT_SIZE-KERNEL_SIZE,
	 },
};


/*
 * The 440EPx has a NAND Flash Controller (NDFC) that handles all accesses to
 * the NAND devices.  The NDFC has command, address and data registers that
 * when accessed will set up the NAND flash pins appropriately.  We'll use the
 * hwcontrol function to save the configuration in a global variable.
 * We can then use this information in the read and write functions to
 * determine which NDFC register to access. For the NCE commands, we'll just
 * set or clear the Bank Enable bit in the NDFC Bank Config registers.
 *
 * There is 1 NAND devices on the board, a Samsung K9F5608U0B (32 MB).
 */
static void
sequoia_hwcontrol(struct mtd_info *mtd, int cmd)
{
	switch (cmd) {
	case NAND_CTL_SETCLE:
		hwctl |= 0x1;
		break;
	case NAND_CTL_CLRCLE:
		hwctl &= ~0x1;
		break;
	case NAND_CTL_SETALE:
		hwctl |= 0x2;
		break;
	case NAND_CTL_CLRALE:
		hwctl &= ~0x2;
		break;
	}
}

static void
sequoia_nand0_hwcontrol(struct mtd_info *mtd, int cmd)
{
	switch(cmd) {
	case NAND_CTL_SETNCE:
		if (nand_cs == 0)
		    sequoia_ndfc->b0cr |= 0x80000000;
		else
		    sequoia_ndfc->b3cr |= 0x80000000;
		break;
	case NAND_CTL_CLRNCE:
		if (nand_cs == 0)
		    sequoia_ndfc->b0cr &= ~0x80000000;
		else
		    sequoia_ndfc->b3cr &= ~0x80000000;
		break;
	default:
		sequoia_hwcontrol(mtd, cmd);
	}
}

static void
sequoia_nand0_enable(void)
{
	sequoia_ndfc->cr = nand_cs << 24;  /* nand chip select is 0 or 3 */
}

static void
sequoia_write_byte(struct mtd_info *mtd, u_char byte)
{
	if (hwctl & 0x1)
		writeb(byte, &(sequoia_ndfc->cmd));
	else if (hwctl & 0x2)
		writeb(byte, &(sequoia_ndfc->addr));
	else
		writeb(byte, &(sequoia_ndfc->data));
}

static void
sequoia_nand0_write_byte(struct mtd_info *mtd, u_char byte)
{
	sequoia_nand0_enable();
	sequoia_write_byte(mtd, byte);
}

static u_char
sequoia_read_byte(struct mtd_info *mtd)
{
	u_char retval;

	if (hwctl & 0x1)
		retval = readb(&(sequoia_ndfc->cmd));
	else if (hwctl & 0x2)
		retval = readb(&(sequoia_ndfc->addr));
	else
		retval = readb( &sequoia_ndfc->data );

	return retval;
}

static u_char
sequoia_nand0_read_byte(struct mtd_info *mtd)
{
	sequoia_nand0_enable();
	return sequoia_read_byte(mtd);
}

static void
sequoia_nand_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		if (hwctl & 0x1)
			writeb(buf[i], &(sequoia_ndfc->cmd));
		else if (hwctl & 0x2)
			writeb(buf[i], &(sequoia_ndfc->addr));
		else
			writeb(buf[i], &(sequoia_ndfc->data));
	}
}

static void
sequoia_nand0_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	sequoia_nand0_enable();
	sequoia_nand_write_buf(mtd, buf, len);
}

static void
sequoia_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (hwctl & 0x1)
			buf[i] = readb(&(sequoia_ndfc->cmd));
		else if (hwctl & 0x2)
			buf[i] = readb(&(sequoia_ndfc->addr));
		else
			buf[i] = readb(&(sequoia_ndfc->data));
	}
}

static void
sequoia_nand0_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	sequoia_nand0_enable();
	sequoia_nand_read_buf(mtd, buf, len);
}

static int
sequoia_nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (hwctl & 0x1) {
			if (buf[i] != readb(&(sequoia_ndfc->cmd)))
				return i;
		} else if (hwctl & 0x2) {
			if (buf[i] != readb(&(sequoia_ndfc->addr)))
				return i;
		} else {
			if (buf[i] != readb(&(sequoia_ndfc->data)))
				return i;
		}

	}

	return 0;
}

static int
sequoia_nand0_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	sequoia_nand0_enable();
	return sequoia_nand_verify_buf(mtd, buf, len);
}

static int
sequoia_dev_ready(struct mtd_info *mtd)
{
	return ((sequoia_ndfc->sr) & 0x01000000) ? 1 : 0;
}

int __init
sequoia_init(void)
{
	int err = 0;
	struct nand_chip *this;
	unsigned sdr0_sdstp1, rl;

	mtdcr( SDR0_CFGADDR, SDR0_SDSTP1 ); /* serial device strap register 1 */
	sdr0_sdstp1 = mfdcr( SDR0_CFGDATA );
	rl = sdr0_sdstp1 >> 11  &  3;	/* ROM location is 0 if EBC, 2 if NAND */

	nand_cs = rl == 2 ? 0 : 3;	/* chip select 0 if NAND boot, else 3 */

	hwctl = 0;

	sequoia_nand0_mtd = kmalloc(sizeof(struct mtd_info) +
				   sizeof(struct nand_chip),
				   GFP_KERNEL);

	sequoia_ndfc = ioremap64(SEQUOIA_NAND_FLASH_REG_ADDR,
			        SEQUOIA_NAND_FLASH_REG_SIZE);
	if (!sequoia_ndfc) {
		printk("Ioremap to access NDFC Registers failed \n");
		err = -EIO;
		goto out;
	}

	/* Initialize structures */
	memset((char *) sequoia_nand0_mtd, 0,
	       sizeof (struct mtd_info) + sizeof (struct nand_chip));

	/* Get pointer to private data */
	this = (struct nand_chip *) (&sequoia_nand0_mtd[1]);
	/* Link the private data with the MTD structure */
	sequoia_nand0_mtd->priv = this;

	/* Set address of NAND IO lines (Using Linear Data Access Region) */
	this->IO_ADDR_R = (void __iomem *) ((ulong) sequoia_ndfc + 0x1000);
	this->IO_ADDR_W = (void __iomem *) ((ulong) sequoia_ndfc + 0x1000);
	/* Reference hardware control function */
	this->hwcontrol  = sequoia_nand0_hwcontrol;
	/* Set command delay time */
	this->chip_delay = 12;
	this->eccmode    = NAND_ECC_SOFT;
	this->write_byte = sequoia_nand0_write_byte;
	this->read_byte  = sequoia_nand0_read_byte;
	this->write_buf  = sequoia_nand0_write_buf;
	this->read_buf   = sequoia_nand0_read_buf;
	this->verify_buf = sequoia_nand0_verify_buf;
	this->dev_ready  = sequoia_dev_ready;

	/* Scan to find existance of the device */
	if (nand_scan(sequoia_nand0_mtd, 1)) {
		err = -ENXIO;
		goto out_ior;
	}

	add_mtd_partitions(sequoia_nand0_mtd, nand0_partition_info,
			sizeof nand0_partition_info/sizeof nand0_partition_info[0]);

	goto out;

out_ior:
	iounmap((void *)sequoia_ndfc);

	kfree(sequoia_nand0_mtd);
out:
	return err;
}

static void __exit
sequoia_cleanup(void)
{
	/* Unregister partitions */
	del_mtd_partitions(sequoia_nand0_mtd);

	/* Release resources, unregister device */
	del_mtd_device(sequoia_nand0_mtd);

	/* unmap physical address */
	iounmap((void *) sequoia_ndfc);

	/* Free the MTD device structure */
	kfree(sequoia_nand0_mtd);
}

module_init(sequoia_init);
module_exit(sequoia_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wade Farnsworth <wfarnsworth@mvista.com>");
MODULE_DESCRIPTION
    ("Board-specific glue layer for NAND flash on IBM 440EPx/440GRx eval board");

