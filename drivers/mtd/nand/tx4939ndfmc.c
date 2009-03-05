/*
 * drivers/mtd/nand/tx4939ndfmc.c
 *
 *  Overview:
 *   This is a device driver for the NAND flash device connected to
 *   TX4939 internal NAND Memory Controller.
 *   TX4939 NDFMC is almost same as TX4938 NDFMC.
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 *
 * Author: source@mvista.com
 *
 * Based on spia.c by Steven J. Hill
 *
 * $Id: tx4939ndfmc.c,v 1.1.2.8.2.1 2005/08/04 12:09:00 issey Exp $
 *
 * Copyright (C) 2000-2001 Toshiba Corporation
 *
 * 2003-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */
#include <linux/config.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/tx4939/tx4939.h>

/*
 * Configuration
 */
#define USE_DMA
#define TRANSFER_SIZE_512BYTE

#ifdef TRANSFER_SIZE_512BYTE
#define TRANSFER_SIZE	512
#else
#define TRANSFER_SIZE	256
#endif

extern struct nand_oobinfo jffs2_oobinfo;

/*
 * MTD structure for TX4939 NDFMC
 */
static struct mtd_info *tx4939ndfmc_mtd;

/*
 * Define partitions for flash device
 */
#define flush_wb()	reg_rd16(&tx4939_ndfmcptr->mcr);

#define NUM_PARTITIONS  	1
#define NUMBER_OF_CIS_BLOCKS	24
#define SIZE_OF_BLOCK		0x00004000
#define NUMBER_OF_BLOCK_PER_ZONE 1024
#define SIZE_OF_ZONE		(NUMBER_OF_BLOCK_PER_ZONE * SIZE_OF_BLOCK)
#ifndef CONFIG_MTD_CMDLINE_PARTS

static struct mtd_partition partition_info[NUM_PARTITIONS] = {
	{
	 .name = "TX4939 NAND",
	 .offset = 0,
	 .size = MTDPART_SIZ_FULL,
	 },
};
#endif

/* Define bad / good block scan pattern which are used
 * while scanning a device for factory marked good / bad blocks. */
static uint8_t bbt_pattern[] = { 0xff };

static struct nand_bbt_descr tx4939ndfmc_bbt_main_descr = {
	.options = 0,
	.offs = 5,
	.len = 1,
	.pattern = bbt_pattern
};

/**
 * tx4939ndfmc_hwcontrol - send command to NDFMC
 * @mtd_info: device information
 * @cmd: command to send to NDFMC
 *
 */
static void tx4939ndfmc_hwcontrol(struct mtd_info *mtd, int cmd)
{
	u16 s = reg_rd16(&tx4939_ndfmcptr->mcr);
	switch (cmd) {
	case NAND_CTL_SETCLE:
		reg_wr16(&tx4939_ndfmcptr->mcr, s | TX4939_NDFMCR_CLE);
		break;
	case NAND_CTL_CLRCLE:
		reg_wr16(&tx4939_ndfmcptr->mcr, s & ~TX4939_NDFMCR_CLE);
		if (!(s & TX4939_NDFMCR_WE))
			reg_wr16(&tx4939_ndfmcptr->dtr, 0);
		break;
	case NAND_CTL_SETALE:
		reg_wr16(&tx4939_ndfmcptr->mcr, s | TX4939_NDFMCR_ALE);
		break;
	case NAND_CTL_CLRALE:
		reg_wr16(&tx4939_ndfmcptr->mcr, s & ~TX4939_NDFMCR_ALE);
		break;
		/* TX4939_NDFMCR_CE bit is 0:high 1:low */
	case NAND_CTL_SETNCE:
		reg_wr16(&tx4939_ndfmcptr->mcr, s | TX4939_NDFMCR_CE);
		break;
	case NAND_CTL_CLRNCE:
		reg_wr16(&tx4939_ndfmcptr->mcr, s & ~TX4939_NDFMCR_CE);
		break;
	case NAND_CTL_SETWP:
		reg_wr16(&tx4939_ndfmcptr->mcr, s | TX4939_NDFMCR_WE);
		break;
	case NAND_CTL_CLRWP:
		reg_wr16(&tx4939_ndfmcptr->mcr, s & ~TX4939_NDFMCR_WE);
		break;
	}
}

/**
 * tx4939ndfmc_dev_ready - return NDFMC status
 * @mtd: device information
 *
 */
static int tx4939ndfmc_dev_ready(struct mtd_info *mtd)
{
	flush_wb();
	return !(reg_rd16(&tx4939_ndfmcptr->sr) & TX4939_NDFSR_BUSY);
}

/**
 * tx4939ndfmc_calculate_ecc - read ecc data
 * @mtd: device information
 * @dat:
 * @ecc_code: ecc data
 *
 */
static int
tx4939ndfmc_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
			  u_char * ecc_code)
{
	u16 mcr = reg_rd16(&tx4939_ndfmcptr->mcr);
	mcr &= ~TX4939_NDFMCR_ECC_ALL;
	reg_wr16(&tx4939_ndfmcptr->mcr, mcr | TX4939_NDFMCR_ECC_DISABLE);
	reg_wr16(&tx4939_ndfmcptr->mcr, mcr | TX4939_NDFMCR_ECC_READ);
	ecc_code[1] = reg_rd16(&tx4939_ndfmcptr->dtr);
	ecc_code[0] = reg_rd16(&tx4939_ndfmcptr->dtr);
	ecc_code[2] = reg_rd16(&tx4939_ndfmcptr->dtr);
#ifdef TRANSFER_SIZE_512BYTE
	ecc_code[4] = reg_rd16(&tx4939_ndfmcptr->dtr);
	ecc_code[3] = reg_rd16(&tx4939_ndfmcptr->dtr);
	ecc_code[5] = reg_rd16(&tx4939_ndfmcptr->dtr);
#endif
	reg_wr16(&tx4939_ndfmcptr->mcr, mcr | TX4939_NDFMCR_ECC_DISABLE);

	return 0;
}

/**
 * tx4939ndfmc_enable_hwecc - enable hardware ecc
 * @mtd: device information
 * @mode:
 *
 */
static void tx4939ndfmc_enable_hwecc(struct mtd_info *mtd, int mode)
{
	u32 mcr = reg_rd16(&tx4939_ndfmcptr->mcr);
	mcr &= ~TX4939_NDFMCR_ECC_ALL;
	reg_wr16(&tx4939_ndfmcptr->mcr, mcr | TX4939_NDFMCR_ECC_RESET);
	reg_wr16(&tx4939_ndfmcptr->mcr, mcr | TX4939_NDFMCR_ECC_DISABLE);
	reg_wr16(&tx4939_ndfmcptr->mcr, mcr | TX4939_NDFMCR_ECC_ENABLE);
}

/**
 * tx4939ndfmc_nand_read_byte - read 1-byte data from NDFMC
 * @mtd: device information
 *
 */
static u_char tx4939ndfmc_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	return tx4939_read_nfmc(this->IO_ADDR_R);
}

/**
 * tx4939ndfmc_nand_write_byte - write 1-byte data to NDFMC
 * @mtd: device information
 * @byte: data to write
 *
 */
static void tx4939ndfmc_nand_write_byte(struct mtd_info *mtd, u_char byte)
{
	struct nand_chip *this = mtd->priv;
	tx4939_write_nfmc(byte, this->IO_ADDR_W);
}

/**
 * dma_irq_handler - interrupt handler for dma transfer
 * @irq: interrupt request number
 * @dev_id:
 * @regs:
 *
 */
#ifdef USE_DMA
static unsigned int interrupt_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(txx9_dma_wait);
static irqreturn_t dma_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	u32 l;
	/* clear DMA interrupt status */
	l = reg_rd32(&tx4939_dmacptr(0)->ch[3].csr);
	reg_wr32(&tx4939_dmacptr(0)->ch[3].csr, l | TX4939_DMCSR_NTRNFC);
	interrupt_flag = 1;
	wake_up(&txx9_dma_wait);

	return IRQ_HANDLED;
}
#endif

/**
 * tx4939ndfmc_nand_write_buf - write data to NDFMC
 * @mtd: device information
 * @buf: data to write
 * @len: length of data
 *
 */
static void
tx4939ndfmc_nand_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	for (i = 0; i < len; i++)
		tx4939_write_nfmc(buf[i], this->IO_ADDR_W);
}

/**
 * tx4939ndfmc_nand_read_buf - read data from NDFMC
 * @mtd: device information
 * @buf: read data
 * @len: length of data
 *
 */
static void
tx4939ndfmc_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	int i;
	u32 l;
	u16 s;
	struct nand_chip *this = mtd->priv;

#ifdef USE_DMA
	/* To use DMA, buf must be alined with 8byte */
	if (len == TRANSFER_SIZE && ((unsigned long)buf & 7) == 0) {
		dma_cache_inv((unsigned long)buf, len);
		reg_wr32(&tx4939_dmacptr(0)->ch[3].cntr, TRANSFER_SIZE);
		reg_wr64(&tx4939_dmacptr(0)->ch[3].sar, CPHYSADDR(buf));
		l = reg_rd32(&tx4939_dmacptr(0)->ch[3].ccr);
		reg_wr32(&tx4939_dmacptr(0)->ch[3].ccr, l | TX4939_DMCCR_XFACT);

		s = reg_rd16(&tx4939_ndfmcptr->mcr);
#ifdef TRANSFER_SIZE_512BYTE
		reg_wr16(&tx4939_ndfmcptr->mcr, s|TX4939_NDFMCR_DMAREQ_512BYTE);
#else
		reg_wr16(&tx4939_ndfmcptr->mcr, s|TX4939_NDFMCR_DMAREQ_256BYTE);
#endif
		wait_event_interruptible(txx9_dma_wait, interrupt_flag);
		interrupt_flag = 0;
		return;
	}
#endif
	for (i = 0; i < len; i++)
		buf[i] = tx4939_read_nfmc(this->IO_ADDR_R);
}

/**
 * tx4939ndfmc_nand_verify_buf - verify data
 * @mtd: device information
 * @buf: data to compare
 * @len: length of data
 *
 */
static int
tx4939ndfmc_nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	for (i = 0; i < len; i++)
		if (buf[i] != tx4939_read_nfmc(this->IO_ADDR_R))
			return i;

	return 0;
}

/**
 * tx4939ndfmc_nand_command - send command to NAND device
 * @mtd: device information
 * @cmd: command to send to NAND device
 * @column:
 * @page_addr:
 */
static void
tx4939ndfmc_nand_command(struct mtd_info *mtd, unsigned command, int column,
			 int page_addr)
{
	register struct nand_chip *this = mtd->priv;

	/* Begin command latch cycle */
	this->hwcontrol(mtd, NAND_CTL_SETCLE);
	/*
	 * Write out the command to the device.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= mtd->oobblock) {
			/* OOB area */
			column -= mtd->oobblock;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		this->write_byte(mtd, readcmd);
	}
	this->write_byte(mtd, command);

	/* Set ALE and clear CLE to start address cycle */
	this->hwcontrol(mtd, NAND_CTL_CLRCLE);

	if (column != -1 || page_addr != -1) {
		this->hwcontrol(mtd, NAND_CTL_SETALE);

		/* Serially input address */
		if (column != -1)
			this->write_byte(mtd, column);
		if (page_addr != -1) {
			this->write_byte(mtd,
					 (unsigned char)(page_addr & 0xff));
			this->write_byte(mtd,
					 (unsigned char)((page_addr >> 8) &
							 0xff));
			/* One more address cycle for higher density devices */
			if (mtd->size & 0x0c000000)
				this->write_byte(mtd,
						 (unsigned
						  char)((page_addr >> 16) &
							0x0f));
		}
		/* Latch in address */
		this->hwcontrol(mtd, NAND_CTL_CLRALE);
	}

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
		/* Turn off WE */
		this->hwcontrol(mtd, NAND_CTL_CLRWP);
		return;

	case NAND_CMD_SEQIN:
		/* Turn on WE */
		this->hwcontrol(mtd, NAND_CTL_SETWP);
		return;

	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
		if (this->dev_ready)
			break;
		this->hwcontrol(mtd, NAND_CTL_SETCLE);
		this->write_byte(mtd, NAND_CMD_STATUS);
		this->hwcontrol(mtd, NAND_CTL_CLRCLE);
		while (!(this->read_byte(mtd) & 0x40)) ;
		return;

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!this->dev_ready) {
			udelay(this->chip_delay);
			return;
		}
	}

	/* wait until command is processed */
	while (!this->dev_ready(mtd)) ;
}

#ifdef CONFIG_MTD_CMDLINE_PARTS
extern int parse_cmdline_partitions(struct mtd_info *master,
				    struct mtd_partition **pparts, char *);
#endif
extern void rbtx4939_get_tx4939_nandc_parameter(int *hold, int *spw);

/**
 * tx4939ndfmc_init - main initialization routine
 *
 */
int __init tx4939ndfmc_init(void)
{
	struct nand_chip *this;
	int bsprt = 0, hold = 0xf, spw = 0xf;
	int protected = 0;
	u64 q;
	u32 l;
	int ret;

	bsprt = 1;

	/* reset NDFMC */
	q = reg_rd64(&tx4939_ccfgptr->clkctr);
	reg_wr64(&tx4939_ccfgptr->clkctr, q | TX4939_CLKCTR_NDCRST);
	udelay(1);		/* wait 128 CPU clocks (2.5ns * 128 = 320ns) */
	q = reg_rd64(&tx4939_ccfgptr->clkctr);
	reg_wr64(&tx4939_ccfgptr->clkctr, q & ~TX4939_CLKCTR_NDCRST);
	udelay(1);		/* wait 128 CPU clocks (2.5ns * 128 = 320ns) */
	reg_wr16(&tx4939_ndfmcptr->sr, 0);
	/* setup  Hold Time, Strobe Pulse Width */
	rbtx4939_get_tx4939_nandc_parameter(&hold, &spw);
	reg_wr16(&tx4939_ndfmcptr->spr, hold << 4 | spw);
	/* setup BusSeparete */
	reg_wr16(&tx4939_ndfmcptr->mcr, bsprt ? TX4939_NDFMCR_BSPRT : 0);
	reg_wr16(&tx4939_ndfmcptr->dtr, 0);

#ifdef USE_DMA
	/* Initialize DMAC */
	reg_wr32(&tx4939_dmacptr(0)->mcr, TX4939_DMMCR_RSFIF | TX4939_DMMCR_MSTEN);
	reg_wr32(&tx4939_dmacptr(0)->mcr, TX4939_DMMCR_MSTEN);

	q = reg_rd64(&tx4939_ccfgptr->pcfg);
	reg_wr64(&tx4939_ccfgptr->pcfg, q & ~TX4939_PCFG_DMASEL3_MASK);

	l = reg_rd32(&tx4939_dmacptr(0)->ch[3].ccr);
	reg_wr32(&tx4939_dmacptr(0)->ch[3].ccr, l |TX4939_DMCCR_CHRST);
	l = reg_rd32(&tx4939_dmacptr(0)->ch[3].ccr);
	reg_wr32(&tx4939_dmacptr(0)->ch[3].ccr, l & (~TX4939_DMCCR_CHRST));
	l = reg_rd32(&tx4939_dmacptr(0)->ch[3].ccr);
	reg_wr32(&tx4939_dmacptr(0)->ch[3].ccr, l | (TX4939_DMCCR_XFSZ_8BYTE
						     | TX4939_DMCCR_SNGAD | TX4939_DMCCR_EXTRQ));
	reg_wr32(&tx4939_dmacptr(0)->ch[3].sair, 8);
	ret = request_irq(TX4939_IRQ_DMA0(3), dma_irq_handler, SA_INTERRUPT,
			  "DMAC-INT", NULL);
	if (ret) {
		printk(KERN_ERR "tx4939 nand: cannot register IRQ %d\n", TX4939_IRQ_DMA0(3)) ;
		return -EIO;

	}

	interrupt_flag = 0;
	l = reg_rd32(&tx4939_dmacptr(0)->ch[3].ccr);
	reg_wr32(&tx4939_dmacptr(0)->ch[3].ccr, l | TX4939_DMCCR_INTENT);
#endif

	/* Allocate memory for MTD device structure and private data */
	tx4939ndfmc_mtd =
	    kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip),
		    GFP_KERNEL);
	if (!tx4939ndfmc_mtd) {
		printk
		    (KERN_ERR "Unable to allocate TX4939 NDFMC MTD device structure.\n");
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&tx4939ndfmc_mtd[1]);

	/* Initialize structures */
	memset((char *)tx4939ndfmc_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	tx4939ndfmc_mtd->priv = this;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (void __iomem *)tx4939_ndfmcptr;
	this->IO_ADDR_W = (void __iomem *)tx4939_ndfmcptr;
	this->hwcontrol = tx4939ndfmc_hwcontrol;
	this->dev_ready = tx4939ndfmc_dev_ready;
	this->calculate_ecc = tx4939ndfmc_calculate_ecc;
	this->correct_data = nand_correct_data;
	this->enable_hwecc = tx4939ndfmc_enable_hwecc;
#ifdef TRANSFER_SIZE_512BYTE
	this->eccmode = NAND_ECC_HW6_512;
#else
	this->eccmode = NAND_ECC_HW3_256;
#endif
	this->chip_delay = 100;
	this->read_byte = tx4939ndfmc_nand_read_byte;
	this->write_byte = tx4939ndfmc_nand_write_byte;
	this->cmdfunc = tx4939ndfmc_nand_command;
	this->write_buf = tx4939ndfmc_nand_write_buf;
	this->read_buf = tx4939ndfmc_nand_read_buf;
	this->verify_buf = tx4939ndfmc_nand_verify_buf;

	this->bbt_td = &tx4939ndfmc_bbt_main_descr;

	/* Scan to find existance of the device */
	if (nand_scan(tx4939ndfmc_mtd, 1)) {
		kfree(tx4939ndfmc_mtd);
		return -ENXIO;
	}

	/* Allocate memory for internal data buffer */
	this->data_buf =
	    kmalloc(sizeof(u_char) *
		    (tx4939ndfmc_mtd->oobblock + tx4939ndfmc_mtd->oobsize),
		    GFP_KERNEL);
	if (!this->data_buf) {
		printk(KERN_ERR "Unable to allocate NAND data buffer for TX4939.\n");
		kfree(tx4939ndfmc_mtd);
		return -ENOMEM;
	}

	if (protected) {
		printk(KERN_INFO "TX4939 NDFMC: write protected.\n");
		tx4939ndfmc_mtd->flags &= ~(MTD_WRITEABLE | MTD_ERASEABLE);
	}
#ifdef CONFIG_MTD_CMDLINE_PARTS
	{
		int mtd_parts_nb = 0;
		struct mtd_partition *mtd_parts = 0;
		mtd_parts_nb =
		    parse_cmdline_partitions(tx4939ndfmc_mtd, &mtd_parts,
					     "tx4939ndfmc");
		if (mtd_parts_nb > 0)
			add_mtd_partitions(tx4939ndfmc_mtd, mtd_parts,
					   mtd_parts_nb);
		else
			add_mtd_device(tx4939ndfmc_mtd);
	}
#else
	add_mtd_partitions(tx4939ndfmc_mtd, partition_info, NUM_PARTITIONS);
#endif

	return 0;
}

module_init(tx4939ndfmc_init);

/**
 * tx4939ndfmc_cleanup - clean up routine
 *
 */
static void __exit tx4939ndfmc_cleanup(void)
{
	struct nand_chip *this = (struct nand_chip *)tx4939ndfmc_mtd->priv;

	/* Unregister the device */
#ifdef CONFIG_MTD_CMDLINE_PARTS
	del_mtd_partitions(tx4939ndfmc_mtd);
#endif
	del_mtd_device(tx4939ndfmc_mtd);

	/* Free the MTD device structure */
	kfree(tx4939ndfmc_mtd);

	/* Free internal data buffer */
	kfree(this->data_buf);
}

module_exit(tx4939ndfmc_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alice Hennessy <ahennessy@mvista.com>");
MODULE_DESCRIPTION("Board-specific glue layer for NAND flash on TX4939 NDFMC");
