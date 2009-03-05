/*
 *  drivers/mtd/bamboo_nand.c
 *
 *  Overview:
 *   This is a device driver for the NAND flash devices found on the
 *   IBM 440EP Evaluation Board (Bamboo).
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
#include <platforms/4xx/bamboo.h>

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

static struct mtd_info *bamboo_nand0_mtd;
static struct mtd_info *bamboo_nand1_mtd;
static u8 hwctl;
static struct ppc440ep_ndfc_regs *bamboo_ndfc;

#define NAND0_NUM_PARTITIONS 1
static struct mtd_partition nand0_partition_info[] = {
	{
	 	.name = "filesystem",
	 	.offset = 0x0,
	 	.size = 0x4000000,
	 },
};

#define NAND1_NUM_PARTITIONS 1
static struct mtd_partition nand1_partition_info[] = {
	{
		.name = "filesystem",
		.offset = 0x0,
		.size = 0x10000000,
	}
};

/* 
 * The 440EP has a NAND Flash Controller (NDFC) that handles all accesses to 
 * the NAND devices.  The NDFC has command, address and data registers that 
 * when accessed will set up the NAND flash pins appropriately.  We'll use the 
 * hwcontrol function to save the configuration in a global variable.  
 * We can then use this information in the read and write functions to 
 * determine which NDFC register to access. For the NCE commands, we'll just
 * set or clear the Bank Enable bit in the NDFC Bank Config registers.
 *
 * There are 2 NAND devices on the board, a Samsung K9F1208U0A (64 MB) and a
 * Samsung K9K2G08U0M (256 MB).
 */
static void
bamboo_hwcontrol(struct mtd_info *mtd, int cmd)
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
bamboo_nand0_hwcontrol(struct mtd_info *mtd, int cmd)
{
	switch(cmd) {
	case NAND_CTL_SETNCE:
		bamboo_ndfc->b1cr |= 0x80000000;
		break;
	case NAND_CTL_CLRNCE:
		bamboo_ndfc->b1cr &= ~0x80000000;
		break;
	default:
		bamboo_hwcontrol(mtd, cmd);
	}
}

static void
bamboo_nand1_hwcontrol(struct mtd_info *mtd, int cmd)
{
	switch(cmd) {
	case NAND_CTL_SETNCE:
		bamboo_ndfc->b2cr |= 0x80000000;
		break;
	case NAND_CTL_CLRNCE:
		bamboo_ndfc->b2cr &= ~0x80000000;
		break;
	default:
		bamboo_hwcontrol(mtd, cmd);
	}
}

static void
bamboo_nand0_enable(void)
{
	bamboo_ndfc->cr = 0x01001000;
}

static void
bamboo_nand1_enable(void)
{
	bamboo_ndfc->cr = 0x02003000;
}

static void
bamboo_write_byte(struct mtd_info *mtd, u_char byte)
{
	if (hwctl & 0x1)
		writeb(byte, &(bamboo_ndfc->cmd));
	else if (hwctl & 0x2)
		writeb(byte, &(bamboo_ndfc->addr));
	else
		writeb(byte, &(bamboo_ndfc->data));
}

static void
bamboo_nand0_write_byte(struct mtd_info *mtd, u_char byte)
{
	bamboo_nand0_enable();
	bamboo_write_byte(mtd, byte);
}

static void
bamboo_nand1_write_byte(struct mtd_info *mtd, u_char byte)
{
	bamboo_nand1_enable();
	bamboo_write_byte(mtd,byte);
}

static u_char
bamboo_read_byte(struct mtd_info *mtd)
{
	u_char retval;
	if (hwctl & 0x1)
		retval = readb(&(bamboo_ndfc->cmd));
	else if (hwctl & 0x2)
		retval = readb(&(bamboo_ndfc->addr));
	else
		retval = readb(&(bamboo_ndfc->data));
	return retval;
}

static u_char
bamboo_nand0_read_byte(struct mtd_info *mtd)
{
	bamboo_nand0_enable();
	return bamboo_read_byte(mtd);
}

static u_char
bamboo_nand1_read_byte(struct mtd_info *mtd)
{
	bamboo_nand1_enable();
	return bamboo_read_byte(mtd);
}

static void
bamboo_nand_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		if (hwctl & 0x1)
			writeb(buf[i], &(bamboo_ndfc->cmd));
		else if (hwctl & 0x2)
			writeb(buf[i], &(bamboo_ndfc->addr));
		else
			writeb(buf[i], &(bamboo_ndfc->data));
	}
}

static void
bamboo_nand0_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	bamboo_nand0_enable();
	bamboo_nand_write_buf(mtd, buf, len);
}

static void
bamboo_nand1_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	bamboo_nand1_enable();
	bamboo_nand_write_buf(mtd, buf, len);
}

static void
bamboo_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (hwctl & 0x1)
			buf[i] = readb(&(bamboo_ndfc->cmd));
		else if (hwctl & 0x2)
			buf[i] = readb(&(bamboo_ndfc->addr));
		else
			buf[i] = readb(&(bamboo_ndfc->data));
	}
}

static void
bamboo_nand0_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	bamboo_nand0_enable();
	bamboo_nand_read_buf(mtd, buf, len);
}

static void
bamboo_nand1_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	bamboo_nand1_enable();
	bamboo_nand_read_buf(mtd, buf, len);
}

static int
bamboo_nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (hwctl & 0x1) {
			if (buf[i] != readb(&(bamboo_ndfc->cmd)))
				return i;
		} else if (hwctl & 0x2) {
			if (buf[i] != readb(&(bamboo_ndfc->addr)))
				return i;
		} else {
			if (buf[i] != readb(&(bamboo_ndfc->data)))
				return i;
		}

	}

	return 0;
}

static int
bamboo_nand0_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	bamboo_nand0_enable();
	return bamboo_nand_verify_buf(mtd, buf, len);
}

static int
bamboo_nand1_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	bamboo_nand1_enable();
	return bamboo_nand_verify_buf(mtd, buf, len);
}

static int
bamboo_dev_ready(struct mtd_info *mtd)
{
	return ((bamboo_ndfc->sr) & 0x01000000) ? 1 : 0; 
}

int __init
bamboo_init(void)
{
	struct nand_chip *this;
	uint * selection1_base, * gpio_base;
	u8 selection1_val;
	int err = 0;
	
	hwctl = 0;

	/* 
	 * Bank 0 was set up by the firmware already.  Bank 1 wasn't, so set it
	 * up now. 
	 */

	selection1_base = ioremap64(BAMBOO_FPGA_SELECTION1_REG_ADDR, 8);
	if(!selection1_base){
		printk("Ioremap to access FPGA Selection Register 1 failed \n");
		err = -EIO;
		goto out;
	}
	selection1_val = readb(selection1_base);
	selection1_val |= 0x02;
	writeb(selection1_val, selection1_base);
	iounmap((void *)(selection1_base));

	SDR_WRITE(DCRN_SDR_CUST0, SDR_READ(DCRN_SDR_CUST0) | 0x2);

	gpio_base = ioremap64(0x0EF600B00ULL, 0x80);
	if(!gpio_base) {
		printk("Ioremap to access GPIO Registers failed \n");
		err = -EIO;
		goto out;
	}
	*(uint *) (gpio_base + 0x2) |= 0x00010000;
	*(uint *) (gpio_base + 0x4) |= 0x00010000;
	iounmap((void *) gpio_base);
	
	bamboo_nand0_mtd = kmalloc(sizeof(struct mtd_info) +
				   sizeof(struct nand_chip),
				   GFP_KERNEL);
	if (!bamboo_nand0_mtd) {
		printk("Unable to allocate NAND 0 MTD device structure.\n");
		err = -ENOMEM;
		goto out;
	}
	
	bamboo_nand1_mtd = kmalloc(sizeof (struct mtd_info) +
				   sizeof (struct nand_chip),
				   GFP_KERNEL);
	if (!bamboo_nand1_mtd) {
		printk("Unable to allocate NAND 1 MTD device structure.\n");
		err = -ENOMEM;
		goto out_mtd0;
	}

	bamboo_ndfc = ioremap64(BAMBOO_NAND_FLASH_REG_ADDR, 
			        BAMBOO_NAND_FLASH_REG_SIZE); 
	if (!bamboo_ndfc) {
		printk("Ioremap to access NDFC Registers failed \n");
		err = -EIO;
		goto out_mtd1;
	}
	bamboo_ndfc->b2cr = 0xC0007777;

	/* Initialize structures */
	memset((char *) bamboo_nand0_mtd, 0,
	       sizeof (struct mtd_info) + sizeof (struct nand_chip));

	memset((char *) bamboo_nand1_mtd, 0,
	       sizeof (struct mtd_info) + sizeof (struct nand_chip));

	/* Get pointer to private data */
	this = (struct nand_chip *) (&bamboo_nand0_mtd[1]);
	/* Link the private data with the MTD structure */
	bamboo_nand0_mtd->priv = this;

	/* Set address of NAND IO lines (Using Linear Data Access Region) */
	this->IO_ADDR_R = (void __iomem *) ((ulong) bamboo_ndfc + 0x1000);
	this->IO_ADDR_W = (void __iomem *) ((ulong) bamboo_ndfc + 0x1000);
	/* Reference hardware control function */
	this->hwcontrol  = bamboo_nand0_hwcontrol;
	/* Set command delay time */
	this->chip_delay = 12;
	this->eccmode    = NAND_ECC_SOFT;
	this->write_byte = bamboo_nand0_write_byte;
	this->read_byte  = bamboo_nand0_read_byte;
	this->write_buf  = bamboo_nand0_write_buf;
	this->read_buf   = bamboo_nand0_read_buf;
	this->verify_buf = bamboo_nand0_verify_buf;
	this->dev_ready  = bamboo_dev_ready;

	/* Scan to find existance of the device */
	if (nand_scan(bamboo_nand0_mtd, 1)) {
		err = -ENXIO;
		goto out_ior;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&bamboo_nand1_mtd[1]);
	/* Link the private data with the MTD structure */
	bamboo_nand1_mtd->priv = this;

	/* Set address of NAND IO lines (Using Linear Data Access Region) */
	this->IO_ADDR_R = (void __iomem *) ((ulong) bamboo_ndfc + 0x1000);
	this->IO_ADDR_W = (void __iomem *) ((ulong) bamboo_ndfc + 0x1000);
	/* Reference hardware control function */
	this->hwcontrol  = bamboo_nand1_hwcontrol;
	/* Set command delay time */
	this->chip_delay = 25;
	this->eccmode    = NAND_ECC_SOFT;
	this->write_byte = bamboo_nand1_write_byte;
	this->read_byte  = bamboo_nand1_read_byte;
	this->write_buf  = bamboo_nand1_write_buf;
	this->read_buf   = bamboo_nand1_read_buf;
	this->verify_buf = bamboo_nand1_verify_buf;
	this->dev_ready  = NULL;

	/* Scan to find existance of the device */
	if (nand_scan(bamboo_nand1_mtd, 1)) {
		err = -ENXIO;
		goto out_ior;
	}

	
	add_mtd_partitions(bamboo_nand0_mtd, nand0_partition_info, 
			   NAND0_NUM_PARTITIONS);

	add_mtd_partitions(bamboo_nand1_mtd, nand1_partition_info, 
			   NAND1_NUM_PARTITIONS);
	goto out;

out_ior:
	iounmap((void *)bamboo_ndfc);
out_mtd1:
	kfree(bamboo_nand1_mtd);
out_mtd0:
	kfree(bamboo_nand0_mtd);
out:
	return err;
}

static void __exit
bamboo_cleanup(void)
{
	/* Unregister partitions */
	del_mtd_partitions(bamboo_nand0_mtd);
	del_mtd_partitions(bamboo_nand1_mtd);

	/* Release resources, unregister device */
	del_mtd_device(bamboo_nand0_mtd);
	del_mtd_device(bamboo_nand1_mtd);

	/* unmap physical address */
	iounmap((void *) bamboo_ndfc);

	/* Free the MTD device structure */
	kfree(bamboo_nand0_mtd);
	kfree(bamboo_nand1_mtd);
}

module_init(bamboo_init);
module_exit(bamboo_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wade Farnsworth <wfarnsworth@mvista.com>");
MODULE_DESCRIPTION
    ("Board-specific glue layer for NAND flash on IBM 440EP eval board");
