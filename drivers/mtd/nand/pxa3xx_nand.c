/*
 *  drivers/mtd/nand/pxa3xx_nand.c
 *
 *  Copyright (C) 2005 Intel Coporation (chao.xie@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device on zylonite board
 *   which utilizes the Samsung K9K1216Q0C parts. This is a 64Mibit NAND
 *   flash device.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/dma.h>
#include <asm/arch/cpu-freq-voltage-mhn.h>

/* DFC timing 0 register */
#define DFC_TIMING_tRP		0
#define DFC_TIMING_tRH		3
#define DFC_TIMING_tWP		8
#define DFC_TIMING_tWH		11
#define DFC_TIMING_tCS		16
#define DFC_TIMING_tCH		19

/* DFC timing 1 register */
#define DFC_TIMING_tAR		0
#define DFC_TIMING_tWHR		4
#define DFC_TIMING_tR		16

/* max value for each timing setting in DFC */
#define DFC_TIMING_MAX_tCH	7
#define DFC_TIMING_MAX_tCS	7
#define DFC_TIMING_MAX_tWH	7
#define DFC_TIMING_MAX_tWP	7
#define DFC_TIMING_MAX_tRH	7
#define DFC_TIMING_MAX_tRP	7
#define DFC_TIMING_MAX_tR	65535
#define DFC_TIMING_MAX_tWHR	15
#define DFC_TIMING_MAX_tAR	15

/*
 * The Data Flash Controller Flash timing structure
 * For NAND flash used on Zylonite board(Samsung K9K1216Q0C),
 * user should use value at end of each row of following member
 * bracketed.
 */
struct dfc_flash_timing {
	uint32_t tCH;		/* Enable signal hold time */
	uint32_t tCS;		/* Enable signal setup time */
	uint32_t tWH;		/* ND_nWE high duration */
	uint32_t tWP;		/* ND_nWE pulse time */
	uint32_t tRH;		/* ND_nRE high duration */
	uint32_t tRP;		/* ND_nRE pulse width */
	uint32_t tR;		/* ND_nWE high to ND_nRE low for read */
	uint32_t tWHR;		/* ND_nWE high to ND_nRE low delay for status read */
	uint32_t tAR;		/* ND_ALE low to ND_nRE low delay */
};

/*
 * The Data Flash Controller Flash specification structure
 * For NAND flash used on Zylonite board(Samsung K9K1216Q0C),
 * user should use value at end of each row of following member
 * bracketed.
 */
struct dfc_flash_info {
	struct dfc_flash_timing timing;	/* NAND Flash timing */

	int enable_arbiter;	/* Data flash bus arbiter enable (ND_ARB_EN) */
	uint32_t page_per_block;	/* Pages per block (PG_PER_BLK) */
	uint32_t row_addr_start;	/* Row address start position (RA_START) */
	uint32_t read_id_bytes;	/* returned ID bytes(RD_ID_CNT) */
	uint32_t dfc_mode;	/* NAND, CARBONDALE, PIXLEY... (ND_MODE) */
	uint32_t ncsx;		/* Chip select don't care bit (NCSX) */
	uint32_t page_size;	/* Page size in bytes (PAGE_SZ) */
	uint32_t oob_size;	/* OOB size */
	uint32_t flash_width;	/* Width of Flash memory (DWIDTH_M) */
	uint32_t dfc_width;	/* Width of flash controller(DWIDTH_C) */
	uint32_t num_blocks;	/* Number of physical blocks in Flash */
	uint32_t chip_id;
	uint32_t read_prog_cycles;	/* Read Program address cycles */

	/* command codes */
	uint32_t read1;		/* Read */
	uint32_t read2;		/* unused, DFC don't support yet */
	uint32_t program;	/* two cycle command */
	uint32_t read_status;
	uint32_t read_id;
	uint32_t erase;		/* two cycle command */
	uint32_t reset;
	uint32_t lock;		/* lock whole flash */
	uint32_t unlock;	/* two cycle command, supporting partial unlock */
	uint32_t lock_status;	/* read block lock status */

	/* addr2ndcb1 - encode address cycles into register NDCB1 */
	/* ndbbr2addr - convert register NDBBR to bad block address */
	int (*addr2ndcb1) (uint16_t cmd, uint32_t addr, uint32_t * p);
	int (*ndbbr2addr) (uint16_t cmd, uint32_t ndbbr, uint32_t * p);
};

enum {
	DFC_FLASH_NULL = 0,
	DFC_FLASH_Samsung_512Mb_X_16 = 1,
	DFC_FLASH_Micron_1Gb_X_8 = 2,
	DFC_FLASH_Micron_1Gb_X_16 = 3,
	DFC_FLASH_STM_1Gb_X_16 = 4,
	DFC_FLASH_STM_2Gb_X_16 = 5,
	DFC_FLASH_STM_MCP_1Gb_X_16 = 6,
	DFC_FLASH_Toshiba2GbX16= 7,
	DFC_FLASH_END,
};

static int dfc_get_flash_info(int type, struct dfc_flash_info **flash_info);

#define		DFC_NDCR	0
#define		DFC_NDTR0CS0	1
#define		DFC_NDTR1CS0	3
#define		DFC_NDSR	5
#define		DFC_NDPCR	6
#define		DFC_NDBDR0	7
#define		DFC_NDBDR1	8
#define		DFC_NDDB	16
#define		DFC_NDCB0	18
#define		DFC_NDCB1	19
#define		DFC_NDCB2	20

/* The Data Flash Controller Mode structure */
struct dfc_mode {
	int enable_dma;		/* DMA, or nonDMA mode */
	int enable_ecc;		/* ECC on/off */
	int enable_spare;	/* Spare enable */
	int chip_select;	/* CS0 or CS1 */
};

/* The Data Flash Controller Context structure */
struct dfc_context {
	unsigned char __iomem *membase;	/* DFC register base */
	struct dfc_mode *dfc_mode;	/* DFC mode */
	int data_dma_ch;	/* Data DMA channel number */
	int cmd_dma_ch;		/* CMD  DMA channel number */
	struct dfc_flash_info *flash_info;	/* Flash Spec */
};

#define NDCB0_DMA_ADDR	0x43100048
#define NDDB_DMA_ADDR	0x43100040

#define NDSR_MASK	0xFFF

/* The following data is a rough evaluation */

/* microsecond, for readID/readStatus/reset */
#define NAND_OTHER_TIMEOUT 		10
/* microsecond, for readID/readStatus/reset */
#define NAND_CMD_TIMEOUT		10

#define BBT_BLOCK_BAD	0x03
#define BBT_BLOCK_GOOD	0x00
#define BBT_BLOCK_REV1	0x01
#define BBT_BLOCK_REV2	0x02

#define BUFLEN		(2048 + 64)

/*
 * DFC data size enumeration transfered from/to controller,
 * including padding (zero)to be a multiple of 32.
 */
enum {
	DFC_DATA_SIZE_STATUS = 8,	/* ReadStatus/ReadBlockLockStatus */
	DFC_DATA_SIZE_ID = 7,	/* ReadID */

	DFC_DATA_SIZE_32 = 32,
	DFC_DATA_SIZE_512 = 512,	/* R/W disabling spare area */
	DFC_DATA_SIZE_520 = 520,	/* Spare=1, ECC=1 */
	DFC_DATA_SIZE_528 = 528,	/* Spare=1, ECC=0 */
	DFC_DATA_SIZE_544 = 544,	/* R/W enabling spare area.(DMA mode) */

	DFC_DATA_SIZE_64 = 64,
	DFC_DATA_SIZE_2048 = 2048,	/* R/W disabling spare area */
	DFC_DATA_SIZE_2088 = 2088,	/* R/W enabling spare area with ecc */
	DFC_DATA_SIZE_2112 = 2112,	/* R/W enabling spare area without ecc */
	DFC_DATA_SIZE_2096 = 2096,	/* R/W enabling spare area */
	DFC_DATA_SIZE_UNUSED = 0xFFFF
};

/* DFC padding size enumeration transfered from/to controller */
enum {
	/*
	 * ReadStatus/ReadBlockLockStatus/ReadID/
	 * Read/Program disabling spare area(Both 512 and 2048)
	 * Read/Program enabling spare area, disabling ECC
	 */
	DFC_PADDING_SIZE_0 = 0,

	/* Read/program with SPARE_EN=1, ECC_EN=0, pgSize=512 */
	DFC_PADDING_SIZE_16 = 16,
	/* for read/program with SPARE_EN=1, ECC_EN=1, pgSize=512 and 2048 */
	DFC_PADDING_SIZE_24 = 24,
	DFC_PADDING_SIZE_UNUSED = 0xFFFF
};

/* use cache read to avoid STM 2G NAND failure */
#define CACHE_READ

static unsigned int flash_config = DFC_FLASH_NULL;

static void dfc_set_timing(struct dfc_context *context,
			   struct dfc_flash_timing *t);
static void dfc_set_dma(struct dfc_context *context);
static void dfc_set_ecc(struct dfc_context *context);
static void dfc_set_spare(struct dfc_context *context);

static int dfc_get_pattern(struct dfc_context *context, uint16_t cmd,
			   int *data_size, int *padding);

static int dfc_wait_event(struct dfc_context *context, uint32_t event,
			  uint32_t * event_out, uint32_t timeout,
			  int enable_int);

static int dfc_send_cmd(struct dfc_context *context, uint16_t cmd,
			uint32_t addr, int num_pages);

static void dfc_stop(struct dfc_context *context);
static void dfc_read_fifo_partial(struct dfc_context *context, uint8_t * buffer,
				  int nbytes, int data_size);
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
static void dfc_write_fifo_partial(struct dfc_context *context,
				   uint8_t * buffer, int nbytes, int data_size);
#endif

static void dfc_clear_int(struct dfc_context *context, uint32_t int_mask);
static void dfc_enable_int(struct dfc_context *context, uint32_t int_mask);
static void dfc_disable_int(struct dfc_context *context, uint32_t int_mask);

/* high level primitives */
static int dfc_init(struct dfc_context *context, int type);
static int dfc_reset_flash(struct dfc_context *context);

#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifndef  CONFIG_MTD_NAND_PXA310_FIX
static int dfc_setup_cmd_dma(struct dfc_context *context,
			     uint16_t cmd, uint32_t addr, int num_pages,
			     uint32_t * buf, uint32_t buf_phys,
			     uint32_t next_desc_phys, uint32_t dma_int_en,
			     struct pxa_dma_desc *dma_desc);
#endif
static int dfc_setup_data_dma(struct dfc_context *context,
			      uint16_t cmd, uint32_t buf_phys,
			      uint32_t next_desc_phys, uint32_t dma_int_en,
			      struct pxa_dma_desc *dma_desc);
#ifndef  CONFIG_MTD_NAND_PXA310_FIX
static void dfc_start_cmd_dma(struct dfc_context *context,
			      struct pxa_dma_desc *dma_desc);
#endif
static void dfc_start_data_dma(struct dfc_context *context,
			       struct pxa_dma_desc *dma_desc);
#endif
static int monahans_df_dev_ready(struct mtd_info *mtd);

static unsigned short search_rel_block(int block, struct mtd_info *mtd);

/*****************************************************************************
 * The DFC registers read/write routines
 *****************************************************************************/
static inline void dfc_write(struct dfc_context *context, int offset,
			     unsigned long value)
{
#ifdef CONFIG_MTD_NAND_PXA310_FIX
	unsigned long flags;

	local_irq_save(flags);
	pxa_set_cken(CKEN_NAND, 0);
#endif
	offset <<= 2;
	writel(value, context->membase + offset);
	__raw_readl(context->membase + offset);
#ifdef CONFIG_MTD_NAND_PXA310_FIX
	pxa_set_cken(CKEN_NAND, 1);
	local_irq_restore(flags);
#endif

}

static inline unsigned int dfc_read(struct dfc_context *context, int offset)
{
	offset <<= 2;
	return __raw_readl(context->membase + offset);
}

/****************************************************************************
 * Flash Information
 ***************************************************************************/

static int Samsung512MbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p);
static int Samsung512MbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr,
				     uint32_t * p);

static struct dfc_flash_info samsung512MbX16 = {
	.timing = {
		   .tCH = 10,	/* tCH, Enable signal hold time */
		   .tCS = 0,	/* tCS, Enable signal setup time */
		   .tWH = 20,	/* tWH, ND_nWE high duration */
		   .tWP = 40,	/* tWP, ND_nWE pulse time */
		   .tRH = 30,	/* tRH, ND_nRE high duration */
		   .tRP = 40,	/* tRP, ND_nRE pulse width */
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 11123,
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 11123,
		   /* tWHR, ND_nWE high to ND_nRE low delay for status read */
		   .tWHR = 110,
		   .tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
		   },
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 32,	/* Pages per block */
	.row_addr_start = 0,	/* Second cycle start, Row address start position */
	.read_id_bytes = 2,	/* 2 bytes, returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 512,	/* Page size in bytes */
	.oob_size = 16,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 4096,	/* Number of physical blocks in Flash */
	.chip_id = 0x46ec,
	.read_prog_cycles = 4,	/* Read, Program Cycles */

	/* command codes */
	.read1 = 0x0000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase = 0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Samsung512MbX16Addr2NDCB1,
	.ndbbr2addr = Samsung512MbX16NDBBR2Addr,
};

static int Samsung512MbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p)
{
	uint32_t ndcb1 = 0;

	if (addr >= 0x4000000)
		return -EINVAL;

	if (cmd == samsung512MbX16.read1 || cmd == samsung512MbX16.program) {
		ndcb1 = (addr & 0xFF) | ((addr >> 1) & 0x01FFFF00);
	} else if (cmd == samsung512MbX16.erase) {
		ndcb1 = ((addr >> 9) & 0x00FFFFFF);
	}

	*p = ndcb1;
	return 0;

}

static int Samsung512MbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p)
{
	*p = ndbbr << 9;
	return 0;
}

static int Micron1GbX8Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p);
static int Micron1GbX8NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p);

static struct dfc_flash_info micron1GbX8 = {
	.timing = {
		   .tCH = 10,	/* tCH, Enable signal hold time */
		   .tCS = 25,	/* tCS, Enable signal setup time */
		   .tWH = 15,	/* tWH, ND_nWE high duration */
		   .tWP = 25,	/* tWP, ND_nWE pulse time */
		   .tRH = 15,	/* tRH, ND_nRE high duration */
		   .tRP = 25,	/* tRP, ND_nRE pulse width */
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 25000,
		   /* tWHR, ND_nWE high to ND_nRE low delay for status read */
		   .tWHR = 60,
		   .tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
		   },
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 8,	/* Width of Flash memory */
	.dfc_width = 8,		/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id = 0xa12c,
	.read_prog_cycles = 4,	/* Read, Program Cycles */
	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase = 0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Micron1GbX8Addr2NDCB1,
	.ndbbr2addr = Micron1GbX8NDBBR2Addr,
};

static int Micron1GbX8Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / micron1GbX8.page_size;
	addr = (page / micron1GbX8.page_per_block) << 18 |
	    (page % micron1GbX8.page_per_block) << 12;

	if (cmd == micron1GbX8.read1 || cmd == micron1GbX8.program) {
		ndcb1 = ((addr << 4) & 0xFFFF0000);
	} else if (cmd == micron1GbX8.erase) {
		ndcb1 = ((addr >> 18) << 6) & 0xFFFF;
	}

	*p = ndcb1;
	return 0;
}

static int Micron1GbX8NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p)
{
	if (cmd == micron1GbX8.read1 || cmd == micron1GbX8.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	} else if (cmd == micron1GbX8.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}

static int Micron1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p);
static int Micron1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p);

static struct dfc_flash_info micron1GbX16 = {
	.timing = {
		   .tCH = 10,	/* tCH, Enable signal hold time */
		   .tCS = 25,	/* tCS, Enable signal setup time */
		   .tWH = 15,	/* tWH, ND_nWE high duration */
		   .tWP = 25,	/* tWP, ND_nWE pulse time */
		   .tRH = 15,	/* tRH, ND_nRE high duration */
		   .tRP = 25,	/* tRP, ND_nRE pulse width */
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 25000,
		   /* tWHR, ND_nWE high to ND_nRE low delay for status read */
		   .tWHR = 60,
		   .tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
		   },
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id = 0xb12c,
	.read_prog_cycles = 4,	/* Read, Program Cycles */

	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase = 0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Micron1GbX16Addr2NDCB1,
	.ndbbr2addr = Micron1GbX16NDBBR2Addr,
};

static int Micron1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / micron1GbX16.page_size;
	addr = (page / micron1GbX16.page_per_block) << 17 |
	    (page % micron1GbX16.page_per_block) << 11;

	if (cmd == micron1GbX16.read1 || cmd == micron1GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	} else if (cmd == micron1GbX16.erase) {
		ndcb1 = ((addr >> 17) << 6) & 0xFFFF;
	}
	*p = ndcb1;
	return 0;
}

static int Micron1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p)
{
	if (cmd == micron1GbX16.read1 || cmd == micron1GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	} else if (cmd == micron1GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int STM1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p);
static int STM1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p);

static struct dfc_flash_info stm1GbX16 = {
	.timing = {
		   .tCH = 10,	/* tCH, Enable signal hold time */
		   .tCS = 10,	/* tCS, Enable signal setup time */
		   .tWH = 20,	/* tWH, ND_nWE high duration */
		   .tWP = 25,	/* tWP, ND_nWE pulse time */
		   .tRH = 20,	/* tRH, ND_nRE high duration */
		   .tRP = 25,	/* tRP, ND_nRE pulse width */
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 25000,
		   /* tWHR, ND_nWE high to ND_nRE low delay for status read */
		   .tWHR = 60,
		   .tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
		   },
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id = 0xc120,
	.read_prog_cycles = 4,	/* Read, Program Cycles */

	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase = 0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = STM1GbX16Addr2NDCB1,
	.ndbbr2addr = STM1GbX16NDBBR2Addr,
};

static int STM1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / stm1GbX16.page_size;
	addr = (page / stm1GbX16.page_per_block) << 17 |
	    (page % stm1GbX16.page_per_block) << 11;

	if (cmd == stm1GbX16.read1 || cmd == stm1GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	} else if (cmd == stm1GbX16.erase) {
		ndcb1 = ((addr >> 17) << 6) & 0xFFFF;
	}
	*p = ndcb1;
	return 0;
}

static int STM1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p)
{
	if (cmd == stm1GbX16.read1 || cmd == stm1GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	} else if (cmd == stm1GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int STM70nm1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p);
static int STM70nm1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p);

static struct dfc_flash_info stm70nm1GbX16 = {
	.timing = {
		   .tCH = 30,	/* tCH, Enable signal hold time */
		   .tCS = 35,	/* tCS, Enable signal setup time */
		   .tWH = 15,	/* tWH, ND_nWE high duration */
		   .tWP = 25,	/* tWP, ND_nWE pulse time */
		   .tRH = 15,	/* tRH, ND_nRE high duration */
		   .tRP = 25,	/* tRP, ND_nRE pulse width */
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 25000,
		   /* tWHR, ND_nWE high to ND_nRE low delay for status read */
		   .tWHR = 60,
		   .tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
		   },
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id = 0xb120,
	.read_prog_cycles = 4,	/* Read, Program Cycles */

	/* command codes */
#ifdef CACHE_READ
	.read1 = 0x3100,	/* Cache Read */
#else
	.read1 = 0x3000,	/* Read */
#endif
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase = 0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = STM70nm1GbX16Addr2NDCB1,
	.ndbbr2addr = STM70nm1GbX16NDBBR2Addr,
};

static int STM70nm1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / stm70nm1GbX16.page_size;
	addr = (page / stm70nm1GbX16.page_per_block) << 17 |
	    (page % stm70nm1GbX16.page_per_block) << 11;

	if (cmd == stm70nm1GbX16.read1 || cmd == stm70nm1GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	} else if (cmd == stm70nm1GbX16.erase) {
		ndcb1 = ((addr >> 17) << 6) & 0xFFFF;
	}
	*p = ndcb1;
	return 0;
}

static int STM70nm1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p)
{
	if (cmd == stm70nm1GbX16.read1 || cmd == stm70nm1GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	} else if (cmd == stm70nm1GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int STM2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p);
static int STM2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p);

static struct dfc_flash_info stm2GbX16 = {
	.timing = {
		   .tCH = 10,	/* tCH, Enable signal hold time */
		   .tCS = 35,	/* tCS, Enable signal setup time */
		   .tWH = 15,	/* tWH, ND_nWE high duration */
		   .tWP = 25,	/* tWP, ND_nWE pulse time */
		   .tRH = 15,	/* tRH, ND_nRE high duration */
		   .tRP = 25,	/* tRP, ND_nRE pulse width */
		   /* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		   .tR = 25000,
		   /* tWHR, ND_nWE high to ND_nRE low delay for status read */
		   .tWHR = 60,
		   .tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
		   },
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 2048,	/* Number of physical blocks in Flash */
	.chip_id = 0xba20,
	.read_prog_cycles = 5,	/* Read, Program Cycles */

	/* command codes */
#ifdef CACHE_READ
	.read1 = 0x3100,        /* Cache Read */
#else
	.read1 = 0x3000,	/* Read */
#endif
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase = 0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = STM2GbX16Addr2NDCB1,
	.ndbbr2addr = STM2GbX16NDBBR2Addr,
};

static int STM2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t * p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x10000000)
		return -EINVAL;
	page = addr / stm2GbX16.page_size;
	addr = (page / stm2GbX16.page_per_block) << 17 |
	    (page % stm2GbX16.page_per_block) << 11;

	if (cmd == stm2GbX16.read1 || cmd == stm2GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	} else if (cmd == stm2GbX16.erase) {
		ndcb1 = ((addr >> 17) << 6) & 0x1FFFF;
	}
	*p = ndcb1;
	return 0;
}

static int STM2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t * p)
{
	if (cmd == stm2GbX16.read1 || cmd == stm2GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	} else if (cmd == stm2GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int TOSHIBA2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int TOSHIBA2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info toshiba2GbX16 =
{
	.timing = {
		.tCH =  6,      /* tCH, Enable signal hold time */
		.tCS = 10,      /* tCS, Enable signal setup time */
		.tWH = 15,      /* tWH, ND_nWE high duration */
		.tWP = 30,      /* tWP, ND_nWE pulse time */
		.tRH = 25,      /* tRH, ND_nRE high duration */
		.tRP = 50,      /* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,      /* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,    /* Data flash bus arbiter enable */
	.page_per_block = 64,   /* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,     /* Returned ID bytes */
	.dfc_mode = 0,          /* NAND mode */
	.ncsx = 0,
	.page_size = 2048,      /* Page size in bytes */
	.oob_size = 64,         /* OOB size in bytes */
	.flash_width = 16,      /* Width of Flash memory */
	.dfc_width = 16,        /* Width of flash controller */
	.num_blocks = 2048,     /* Number of physical blocks in Flash */
	.chip_id =  0xba98,
	.read_prog_cycles = 5,	/* Read, Program Cycles */

	/* command codes */
	.read1 = 0x3000,        /* Read */
	.read2 = 0x0050,        /* Read1 unused, current DFC don't support */
	.program = 0x1080,      /* Write, two cycle command */
	.read_status = 0x0070,  /* Read status */
	.read_id = 0x0090,      /* Read ID */
	.erase =  0xD060,       /* Erase, two cycle command */
	.reset = 0x00FF,        /* Reset */
	.lock = 0x002A,         /* Lock whole flash */
	.unlock = 0x2423,	/* Unlock, two cycle command, supporting partial unlock */
	.lock_status = 0x007A,  /* Read block lock status */
	.addr2ndcb1 = TOSHIBA2GbX16Addr2NDCB1,
	.ndbbr2addr = TOSHIBA2GbX16NDBBR2Addr,
};

static int TOSHIBA2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x10000000)
		return -EINVAL;
	page = addr / toshiba2GbX16.page_size;
	addr =  (page / toshiba2GbX16.page_per_block) << 18 |
		(page % toshiba2GbX16.page_per_block) << 12;

	if (cmd == toshiba2GbX16.read1 || cmd == toshiba2GbX16.program) {
		ndcb1 = (addr & 0xFFF) | ((addr << 4) & 0xFFFF0000);
	}
	else if (cmd == toshiba2GbX16.erase) {
		ndcb1 = ((addr >> 18) << 6) & 0x1FFFF;
	}
	*p = ndcb1;
	return 0;
}

static int TOSHIBA2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == toshiba2GbX16.read1 || cmd == toshiba2GbX16.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == toshiba2GbX16.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}

static struct {
	int type;
	struct dfc_flash_info *flash_info;
} type_info[] = {
	{ DFC_FLASH_Samsung_512Mb_X_16, &samsung512MbX16},
	{ DFC_FLASH_Micron_1Gb_X_8, &micron1GbX8},
	{ DFC_FLASH_Micron_1Gb_X_16, &micron1GbX16},
	{ DFC_FLASH_STM_1Gb_X_16, &stm1GbX16},
	{ DFC_FLASH_STM_2Gb_X_16, &stm2GbX16},
	{ DFC_FLASH_STM_MCP_1Gb_X_16, &stm70nm1GbX16},
	{ DFC_FLASH_Toshiba2GbX16, &toshiba2GbX16},
	{ DFC_FLASH_NULL, NULL},
};

int dfc_get_flash_info(int type, struct dfc_flash_info **flash_info)
{
	uint32_t i = 0;

	while (type_info[i].type != DFC_FLASH_NULL) {
		if (type_info[i].type == type) {
			*flash_info = type_info[i].flash_info;
			return 0;
		}
		i++;
	}
	*flash_info = NULL;
	return -EINVAL;
}

/******************************************************************************
  dfc_set_timing

  Description:
	This function sets flash timing property in DFC timing register
	according to input timing value embodied in context structure.
	It is called once during the hardware initialization.
  Input Parameters:
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/

#define DFC_CLOCK	208
#define CLOCK_NS	DFC_CLOCK/1000

static void dfc_set_timing(struct dfc_context *context,
			   struct dfc_flash_timing *t)
{
	struct dfc_flash_timing timing = *t;

	uint32_t r0 = 0;
	uint32_t r1 = 0;

	/*
	 * num of clock cycles = time (ns) / one clock sycle (ns) + 1
	 * - integer division will truncate the result, so add a 1 in all cases
	 * - subtract the extra 1 cycle added to all register timing values
	 */
	timing.tCH = min(((int)(timing.tCH * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tCH);
	timing.tCS = min(((int)(timing.tCS * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tCS);
	timing.tWH = min(((int)(timing.tWH * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tWH);
	timing.tWP = min(((int)(timing.tWP * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tWP);
	timing.tRH = min(((int)(timing.tRH * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tRH);
	timing.tRP = min(((int)(timing.tRP * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tRP);

	r0 = (timing.tCH << DFC_TIMING_tCH) |
	    (timing.tCS << DFC_TIMING_tCS) |
	    (timing.tWH << DFC_TIMING_tWH) |
	    (timing.tWP << DFC_TIMING_tWP) |
	    (timing.tRH << DFC_TIMING_tRH) |
	    (timing.tRP << DFC_TIMING_tRP);
	/* FIXME:hardware issue cause it to be the max value */
	dfc_write(context, DFC_NDTR0CS0, r0);

	timing.tR = min(((int)(timing.tR * CLOCK_NS) + 1), DFC_TIMING_MAX_tR);
	timing.tWHR = min(((int)(timing.tWHR * CLOCK_NS) + 1),
			  DFC_TIMING_MAX_tWHR);
	timing.tAR = min(((int)(timing.tAR * CLOCK_NS) + 1),
			 DFC_TIMING_MAX_tAR);

	r1 = (timing.tR << DFC_TIMING_tR) |
	    (timing.tWHR << DFC_TIMING_tWHR) | (timing.tAR << DFC_TIMING_tAR);

	/* FIXME:hardware issue cause it to be the max value */
	dfc_write(context, DFC_NDTR1CS0, r1);
}

/******************************************************************************
  dfc_set_dma

  Description:
		Enables or Disables DMA in line with setting in DFC mode of context
		structure. DMA mode of DFC. Performs a read-modify-write operation that
		only changes the driven DMA_EN bit field In DMA mode, all commands and
		data are transferred by DMA.  DMA can be enable/disable on the fly.
  Input Parameters:
	context -Pointer to DFC context structure
  	Output Parameters:
		None
	Returns:
		None
*******************************************************************************/
static void dfc_set_dma(struct dfc_context *context)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	if (context->dfc_mode->enable_dma)
		ndcr |= NDCR_DMA_EN;
	else
		ndcr &= ~NDCR_DMA_EN;

	dfc_write(context, DFC_NDCR, ndcr);

	/* Read again to make sure write work */
	ndcr = dfc_read(context, DFC_NDCR);
}

/******************************************************************************
  dfc_set_ecc

  Description:
		This function enables or disables hardware ECC capability of DFC in line
		with setting in DFC mode of context structure.
  Input Parameters:
	context -Pointer to DFC context structure
	Output Parameters:
		None
	Returns:
		None
*******************************************************************************/
static void dfc_set_ecc(struct dfc_context *context)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	if (context->dfc_mode->enable_ecc)
		ndcr |= NDCR_ECC_EN;
	else
		ndcr &= ~NDCR_ECC_EN;

	dfc_write(context, DFC_NDCR, ndcr);

	/* Read again to make sure write work */
	ndcr = dfc_read(context, DFC_NDCR);
}

/******************************************************************************
  dfc_set_spare

  Description:
		This function enables or disables accesses to spare area of NAND Flash
		through DFC in line with setting in DFC mode of context structure.
  Input Parameters:
	context -Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_set_spare(struct dfc_context *context)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	if (context->dfc_mode->enable_spare)
		ndcr |= NDCR_SPARE_EN;
	else
		ndcr &= ~NDCR_SPARE_EN;

	dfc_write(context, DFC_NDCR, ndcr);

	/* Read again to make sure write work */
	ndcr = dfc_read(context, DFC_NDCR);
}

static unsigned int get_delta(unsigned int start)
{
	unsigned int stop = OSCR;
	return (stop - start);
}

static int dfc_wait_event(struct dfc_context *context, uint32_t event,
			  uint32_t * event_out, uint32_t timeout,
			  int enable_int)
{
	uint32_t ndsr;
	uint32_t to = 3 * timeout;	/* 3 ticks ~ 1us */
	int status;
	int start = OSCR;

	if (enable_int)
		dfc_enable_int(context, event);

	while (1) {
		ndsr = dfc_read(context, DFC_NDSR);
		ndsr &= NDSR_MASK;
		if (ndsr & event) {
			/* event happened */
			*event_out = ndsr & event;
			dfc_clear_int(context, *event_out);
			status = 0;
			break;
		} else if (get_delta(start) > to) {
			status = -ETIME;
			break;
		}
	}

	if (enable_int)
		dfc_disable_int(context, event);
	return status;
}

/******************************************************************************
  dfc_get_pattern

  Description:
	This function is used to retrieve buffer size setting for a transaction
	based on cmd.
  Input Parameters:
	context - Pointer to DFC context structure
	cmd
	  Specifies type of command to be sent to NAND flash .The LSB of this
	  parameter defines the first command code for 2-cycles command. The
	  MSB defines the second command code for 2-cycles command. If MSB is
	  set to zero, this indicates that one cycle command
	Output Parameters:
	data_size
	  It is used to retrieve  length of data transferred to/from DFC,
	  which includes padding bytes
	padding
	  It is used to retrieve how many padding bytes there should be
	  in buffer of data_size.
	Returns:
	0
	  If size setting is returned successfully
	-EINVAL
	  If page size specified in flash spec of context structure is not 512 or
	  2048;If specified command index is not read1/program/erase/reset/readID/
	  readStatus.
*******************************************************************************/
static int dfc_get_pattern(struct dfc_context *context, uint16_t cmd,
			   int *data_size, int *padding)
{
	struct dfc_mode *dfc_mode = context->dfc_mode;
	struct dfc_flash_info *flash_info = context->flash_info;
	uint32_t page_size = context->flash_info->page_size;	/* 512 or 2048 */

	if (cmd == flash_info->read1 || cmd == flash_info->program) {
		if (512 == page_size) {
			/* add for DMA */
			if (dfc_mode->enable_dma) {
				*data_size = DFC_DATA_SIZE_544;
				if (dfc_mode->enable_ecc)
					*padding = DFC_PADDING_SIZE_24;
				else
					*padding = DFC_PADDING_SIZE_16;
			} else if (!dfc_mode->enable_spare) {
				*data_size = DFC_DATA_SIZE_512;
				*padding = DFC_PADDING_SIZE_0;
			} else {

				if (dfc_mode->enable_ecc)
					*data_size = DFC_DATA_SIZE_520;
				else
					*data_size = DFC_DATA_SIZE_528;

				*padding = DFC_PADDING_SIZE_0;
			}
		} else if (2048 == page_size) {
			/* add for DMA */
			if (dfc_mode->enable_dma) {
				*data_size = DFC_DATA_SIZE_2112;
				if (dfc_mode->enable_ecc)
					*padding = DFC_PADDING_SIZE_24;
				else
					*padding = DFC_PADDING_SIZE_0;
			} else if (!dfc_mode->enable_spare) {
				*data_size = DFC_DATA_SIZE_2048;
				*padding = DFC_PADDING_SIZE_0;
			} else {

				if (dfc_mode->enable_ecc)
					*data_size = DFC_DATA_SIZE_2088;
				else
					*data_size = DFC_DATA_SIZE_2112;

				*padding = DFC_PADDING_SIZE_0;
			}
		} else		/* if the page_size is neither 512 or 2048 */
			return -EINVAL;
	} else if (cmd == flash_info->read_id) {
		*data_size = DFC_DATA_SIZE_ID;
		*padding = DFC_PADDING_SIZE_0;
	} else if (cmd == flash_info->read_status) {
		*data_size = DFC_DATA_SIZE_STATUS;
		*padding = DFC_PADDING_SIZE_0;
	} else if (cmd == flash_info->erase || cmd == flash_info->reset) {
		*data_size = DFC_DATA_SIZE_UNUSED;
		*padding = DFC_PADDING_SIZE_UNUSED;
	} else
		return -EINVAL;
	return 0;
}

/******************************************************************************
  dfc_send_cmd

  Description:
	This function configures DFC to send command through DFC to NAND flash
  Input Parameters:
	context
	  Pointer to DFC context structure
	cmd
	  Specifies type of command to be sent to NAND flash .The LSB of this
	  parameter defines the first command code for 2-cycles command. The
	  MSB defines the second command code for 2-cycles command. If MSB is
	  set to zero, this indicates that one cycle command
	addr
	  Address sent out to the flash device withthis command. For page read/
	  program commands , 4-cycles address is sent. For erase command only
	  3-cycles address is sent. If it is equal to 0xFFFFFFFF, the address
	  should not be used.
	num_pages
	  It specifies the number of pages of data to be transferred for
	  a program or read commands. Unused for any other commands than
	  read/program.

  Output Parameters:
	None
  Returns:
	0
	  If size setting is returned successfully
	-EINVAL
	  If specified command index is not read1/program/erase/reset/readID/
	  readStatus.
*******************************************************************************/
static int dfc_send_cmd(struct dfc_context *context, uint16_t cmd,
			uint32_t addr, int num_pages)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	struct dfc_mode *dfc_mode = context->dfc_mode;
	uint8_t cmd2;
	uint32_t event_out;
	uint32_t ndcb0 = 0, ndcb1 = 0, ndcb2 = 0, ndcr;
	int status;

	/* It is a must to set ND_RUN firstly, then write command buffer
	 * If conversely,it does not work
	 */
	dfc_write(context, DFC_NDSR, NDSR_MASK);

	/* Set ND_RUN */
	ndcr = dfc_read(context, DFC_NDCR);
	dfc_write(context, DFC_NDCR, (ndcr | NDCR_ND_RUN));

	/*  Wait for write command request */
	status = dfc_wait_event(context, NDSR_WRCMDREQ,
				&event_out, NAND_CMD_TIMEOUT, 0);

	if (status)		/* Timeout */
		return status;

	cmd2 = (cmd >> 8) & 0xFF;
	ndcb0 = cmd | (dfc_mode->chip_select << 24) | ((cmd2 ? 1 : 0) << 19);

	if (cmd == flash_info->read1) {
		if (0xFFFFFFFF != addr) {
			ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);
			status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
			if (status)
				return status;
			ndcb2 = (num_pages - 1) << 8;
		}
		/* If has A27, we need to set addr5 */
		if (addr >= 0x8000000)
			ndcb2 |= 0x1;
	} else if (cmd == flash_info->program) {
		ndcb0 |= NDCB0_CMD_TYPE(1) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);
		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
		ndcb2 = (num_pages - 1) << 8;
		/* If has A27, we need to set addr5 */
		if (addr >= 0x8000000)
			ndcb2 |= 0x1;
	} else if (cmd == flash_info->erase) {
		ndcb0 |= NDCB0_CMD_TYPE(2) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(3);
		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
	} else if (cmd == flash_info->read_id) {
		ndcb0 |= NDCB0_CMD_TYPE(3);
	} else if (cmd == flash_info->read_status) {
		ndcb0 |= NDCB0_CMD_TYPE(4);
	} else if (cmd == flash_info->reset) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
	} else if (cmd == flash_info->lock) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
#ifdef CACHE_READ
	} else if(cmd == 0x34) {
		ndcb0 |= NDCB0_CMD_TYPE(4);
#endif
	} else
		return -EINVAL;

	/* Write to DFC command register */
	dfc_write(context, DFC_NDCB0, ndcb0);
	dfc_write(context, DFC_NDCB0, ndcb1);
	dfc_write(context, DFC_NDCB0, ndcb2);

	return 0;
}

/******************************************************************************
  dfc_stop

  Description:
	This function clears ND_RUN bit of NDCR.
  Input Parameters:
	context--Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_stop(struct dfc_context *context)
{
	unsigned int ndcr;
	ndcr = dfc_read(context, DFC_NDCR);
	dfc_write(context, DFC_NDCR, (ndcr & ~NDCR_ND_RUN));
	ndcr = dfc_read(context, DFC_NDCR);
}

#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifndef  CONFIG_MTD_NAND_PXA310_FIX
static int dfc_setup_cmd_dma(struct dfc_context *context,
			     uint16_t cmd, uint32_t addr, int num_pages,
			     uint32_t * buf, uint32_t buf_phys,
			     uint32_t next_desc_phys, uint32_t dma_int_en,
			     struct pxa_dma_desc *dma_desc)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	struct dfc_mode *dfc_mode = context->dfc_mode;
	uint8_t cmd2;
	uint32_t event_out;
	uint32_t ndcb0 = 0, ndcb1 = 0, ndcb2 = 0, ndcr;
	int status;

	/*
	 * It is a must to set ND_RUN firstly, then write command buffer
	 * If conversely,it does not work
	 */
	dfc_write(context, DFC_NDSR, NDSR_MASK);

	/* Set ND_RUN */
	ndcr = dfc_read(context, DFC_NDCR);
	ndcr |= NDCR_ND_RUN;
	dfc_write(context, DFC_NDCR, ndcr);

	/* Wait for write command request */
	status = dfc_wait_event(context, NDSR_WRCMDREQ,
				&event_out, NAND_CMD_TIMEOUT, 0);

	if (status)
		return status;	/* Timeout */

	cmd2 = (cmd >> 8) & 0xFF;
	ndcb0 = cmd | (dfc_mode->chip_select << 24) | ((cmd2 ? 1 : 0) << 19);

	if (cmd == flash_info->read1) {
		if (0xFFFFFFFF != addr) {
			ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);
			status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
			if (status)
				return status;
			ndcb2 = (num_pages - 1) << 8;
		}
		/* If has A27, we need to set addr5 */
		if (addr >= 0x8000000)
			ndcb2 |= 0x1;
	} else if (cmd == flash_info->program) {
		ndcb0 |= NDCB0_CMD_TYPE(1) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);

		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
		ndcb2 = (num_pages - 1) << 8;
		/* If has A27, we need to set addr5 */
		if (addr >= 0x8000000)
			ndcb2 |= 0x1;
	} else if (cmd == flash_info->erase) {
		ndcb0 |= NDCB0_CMD_TYPE(2) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(3);

		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
	} else if (cmd == flash_info->read_id) {
		ndcb0 |= NDCB0_CMD_TYPE(3);
	} else if (cmd == flash_info->read_status) {
		ndcb0 |= NDCB0_CMD_TYPE(4);
	} else if (cmd == flash_info->reset) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
	} else if (cmd == flash_info->lock) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
#ifdef CACHE_READ
	} else if (cmd == 0x34) {
		ndcb0 |= NDCB0_CMD_TYPE(4);
#endif
	} else
		return -EINVAL;

	*((uint32_t *) buf) = ndcb0;
	*((uint32_t *) buf + 1) = ndcb1;
	*((uint32_t *) buf + 2) = ndcb2;

	dma_int_en &= (DCMD_STARTIRQEN | DCMD_ENDIRQEN);

	dma_desc->ddadr = next_desc_phys;
	dma_desc->dsadr = buf_phys;
	dma_desc->dtadr = NDCB0_DMA_ADDR;
	dma_desc->dcmd = DCMD_INCSRCADDR | DCMD_FLOWTRG | dma_int_en |
	    DCMD_WIDTH4 | DCMD_BURST16 | 12;
	return 0;
}
#endif

static int dfc_setup_data_dma(struct dfc_context *context,
			      uint16_t cmd, uint32_t buf_phys,
			      uint32_t next_desc_phys, uint32_t dma_int_en,
			      struct pxa_dma_desc *dma_desc)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	int data_size, padding;

	dfc_get_pattern(context, cmd, &data_size, &padding);

	dma_desc->ddadr = next_desc_phys;
	dma_int_en &= (DCMD_STARTIRQEN | DCMD_ENDIRQEN);

	if (cmd == flash_info->program) {

		dma_desc->dsadr = buf_phys;
		dma_desc->dtadr = NDDB_DMA_ADDR;
		dma_desc->dcmd = DCMD_INCSRCADDR | DCMD_FLOWTRG | dma_int_en |
		    DCMD_WIDTH4 | DCMD_BURST32 | data_size;

	} else if (cmd == flash_info->read1 || cmd == flash_info->read_id ||
		   cmd == flash_info->read_status) {

		dma_desc->dsadr = NDDB_DMA_ADDR;
		dma_desc->dtadr = buf_phys;
		dma_desc->dcmd = DCMD_INCTRGADDR | DCMD_FLOWSRC | dma_int_en |
		    DCMD_WIDTH4 | DCMD_BURST32 | data_size;
	} else
		return -EINVAL;
	return 0;
}

#ifndef  CONFIG_MTD_NAND_PXA310_FIX
static void dfc_start_cmd_dma(struct dfc_context *context,
			      struct pxa_dma_desc *dma_desc)
{
	DRCMR99 = DRCMR_MAPVLD | context->cmd_dma_ch;	/* NAND CMD DRCMR */
	DDADR(context->cmd_dma_ch) = (uint32_t) dma_desc;
	DCSR(context->cmd_dma_ch) |= DCSR_RUN;
}
#endif

static void dfc_start_data_dma(struct dfc_context *context,
			       struct pxa_dma_desc *dma_desc)
{
	DRCMR97 = DRCMR_MAPVLD | context->data_dma_ch;
	DDADR(context->data_dma_ch) = (uint32_t) dma_desc;
	DCSR(context->data_dma_ch) |= DCSR_RUN;
}
#endif

/******************************************************************************
  dfc_read_fifo_partial

  Description:
	This function reads data from data buffer of DFC.Bytes can be any less than
	or equal to data_size, the left is ignored by ReadFIFO though they will be
	read from NDDB to clear data buffer.
  Input Parameters:
	context
	  Pointer to DFC context structure
	nbytes
	  Indicating how much data should be read into buffer.
	data_size
	  Specifing length of data transferred to/from DFC, which includes
	  padding bytes
  Output Parameters:
	pBuffer
	  Pointer to the data buffer where data should be placed.
	Returns:
	  None
*******************************************************************************/
static void dfc_read_fifo_partial(struct dfc_context *context, uint8_t * buffer,
				  int nbytes, int data_size)
{
	uint32_t data = 0;
	uint32_t i = 0;
	uint32_t bytes_multi;
	uint32_t bytes_remain;

	if (1 == data_size) {
		data = dfc_read(context, DFC_NDDB) & 0xFF;
		*buffer++ = (uint8_t) data;
	} else if (2 == data_size) {
		data = dfc_read(context, DFC_NDDB) & 0xFFFF;
		*buffer++ = data & 0xFF;
		*buffer++ = (data >> 8) & 0xFF;
	} else {
		bytes_multi = (nbytes & 0xFFFFFFFC);
		bytes_remain = nbytes & 0x03;

		i = 0;
		/* Read the bytes_multi*4 bytes data */
		while (i < bytes_multi) {
			data = dfc_read(context, DFC_NDDB);
			/* FIXME: we don't know whether the buffer
			 * align to 4 bytes or not. Cast the buffer
			 * to int is not safe here. Especially under
			 * gcc 4.x. Used memcpy here. But the memcpy
			 * may be not correct on BE architecture.
			 * --by Yin, Fengwei
			 */
			memcpy(buffer, &data, sizeof(data));
			i += sizeof(data);
			buffer += sizeof(data);
		}

		/* Read the left bytes_remain bytes data */
		if (bytes_remain) {
			data = dfc_read(context, DFC_NDDB);
			for (i = 0; i < bytes_remain; i++)
				*buffer++ =
				    (uint8_t) ((data >> (8 * i)) & 0xFF);
		}

		/* When read the remain bytes, we always read 4 bytes data
		 * to DFC. So the data_size should subtract following number.
		 */
		data_size -= bytes_multi + (bytes_remain ? sizeof(data) : 0);

		/* We need Read data_size bytes data totally */
		while (data_size > 0) {
			data = dfc_read(context, DFC_NDDB);
			data_size -= sizeof(data);
		}

	}
}

#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
/******************************************************************************
  dfc_write_fifo_partial

  Description:
	Write to data buffer of DFC from a buffer. Bytes can be same as
	data_size, also can be data_size-padding, but can¡¯t be random value,
	the left will be automatically padded by WriteFIFO.
  Input Parameters:
	context
	  Pointer to DFC context structure
	bytes
	  Indicating how much data should be read into buffer.
	data_size
	  Specifing length of data transferred to/from DFC, which includes
	  padding bytes
	buffer
	  Pointer to the data buffer where data will be taken from to be written
	  to DFC data buffer
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_write_fifo_partial(struct dfc_context *context,
				   uint8_t * buffer, int nbytes, int data_size)
{
	uint32_t i = 0;

	uint32_t bytes_multi = (nbytes & 0xFFFFFFFC);
	uint32_t bytes_remain = nbytes & 0x03;
	uint32_t temp;
	/*
	 * caller guarantee buffer contains appropriate data thereby
	 * it is impossible for nbytes not to be a multiple of 4 byte
	 */

	/* Write the bytes_multi*4 bytes data */
	while (i < bytes_multi) {
		temp = buffer[0] | buffer[1] << 8 |
		    buffer[2] << 16 | buffer[3] << 24;
		dfc_write(context, DFC_NDDB, temp);
		buffer += 4;
		i += 4;
	}

	/* Write the left bytes_remain bytes data */
	if (bytes_remain) {
		temp = 0xFFFFFFFF;
		for (i = 0; i < bytes_remain; i++)
			temp &= *buffer++ << i * 8;

		dfc_write(context, DFC_NDDB, temp);
	}

	/* When write the remain bytes, we always write 4 bytes data
	 * to DFC. So the data_size should subtract following number.
	 */
	data_size -= bytes_multi + (bytes_remain ? sizeof(temp) : 0);

	while (data_size > 0) {
		dfc_write(context, DFC_NDDB, 0xFFFFFFFF);
		data_size -= 4;
	}

}
#endif

/******************************************************************************
  dfc_enable_int

  Description:
	This function is used to enable DFC interrupts.	The bits in int_mask
	will be used to unmask NDCR register to enable corresponding interrupts.
  Input Parameters:
	context
	  Pointer to DFC context structure
	int_mask
	  Specifies what interrupts to enable
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_enable_int(struct dfc_context *context, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	ndcr &= ~int_mask;
	dfc_write(context, DFC_NDCR, ndcr);

	ndcr = dfc_read(context, DFC_NDCR);
}

/******************************************************************************
  dfc_disable_int

  Description:
	This function is used to disable DFC interrupts.
	The bits inint_mask will be used to mask NDCR register to disable
	corresponding interrupts.
  Input Parameters:
	context
	  Pointer to DFC context structure
	int_mask
	  Specifies what interrupts to disable
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_disable_int(struct dfc_context *context, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	ndcr |= int_mask;
	dfc_write(context, DFC_NDCR, ndcr);

	ndcr = dfc_read(context, DFC_NDCR);
}

/******************************************************************************
  dfc_clear_int

  Description:
	This function is used to disable DFC interrupts.
	The bits in int_mask will be used to clear corresponding interrupts
	in NDCR register
  Input Parameters:
	context
	  Pointer to DFC context structure
	int_mask
	  Specifies what interrupts to clear
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_clear_int(struct dfc_context *context, uint32_t int_mask)
{
	dfc_write(context, DFC_NDSR, int_mask);
	dfc_read(context, DFC_NDSR);
}

/*
 * high level primitives
 */

/******************************************************************************
  dfc_init

  Description:
	This function does entire DFC initialization according to the NAND
	flash type currently used with platform, including setting MFP, set
	flash timing, set DFC mode, configuring specified flash parameters
	in DFC, clear ECC logic and page count register.
  Input Parameters:
	context
	  Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	0
	  if MFPRs are set correctly
	-EINVAL
	  if specified flash is not support by check bytes per page and pages per
	  block
******************************************************************************/
extern void zylonite_enable_dfc_pins(void);

static int dfc_init(struct dfc_context *context, int type)
{
	int status;
	struct dfc_flash_info *flash_info;
	uint32_t ndcr = 0x00000FFF;	/* disable all interrupts */

	status = dfc_get_flash_info(type, &flash_info);
	if (status)
		return status;
	context->flash_info = flash_info;

	zylonite_enable_dfc_pins();

	dfc_set_timing(context, &context->flash_info->timing);

	if (flash_info->enable_arbiter)
		ndcr |= NDCR_ND_ARB_EN;

	if (64 == flash_info->page_per_block)
		ndcr |= NDCR_PG_PER_BLK;
	else if (32 != flash_info->page_per_block)
		return -EINVAL;

	if (flash_info->row_addr_start)
		ndcr |= NDCR_RA_START;

	ndcr |= (flash_info->read_id_bytes) << 16;

	ndcr |= (flash_info->dfc_mode) << 21;

	if (flash_info->ncsx)
		ndcr |= NDCR_NCSX;

	if (2048 == flash_info->page_size)
		ndcr |= NDCR_PAGE_SZ;
	else if (512 != flash_info->page_size)
		return -EINVAL;

	if (16 == flash_info->flash_width)
		ndcr |= NDCR_DWIDTH_M;
	else if (8 != flash_info->flash_width)
		return -EINVAL;

	if (16 == flash_info->dfc_width)
		ndcr |= NDCR_DWIDTH_C;
	else if (8 != flash_info->dfc_width)
		return -EINVAL;

	dfc_write(context, DFC_NDCR, ndcr);

	dfc_set_dma(context);
	dfc_set_ecc(context);
	dfc_set_spare(context);

	return 0;
}

/*
 * This macro will be used in following NAND operation functions.
 * It is used to clear command buffer to ensure cmd buffer is empty
 * in case of operation is timeout
 */
#define ClearCMDBuf() 	do {					\
				dfc_stop(context); 		\
				udelay(NAND_OTHER_TIMEOUT);	\
			} while (0)

/******************************************************************************
  dfc_reset_flash

  Description:
	It reset the flash. The function can be called at any time when the
	device is in Busy state during random read/program/erase mode and
	reset operation will abort all these operations. After reset operation
	the device is ready to wait for next command
  Input Parameters:
	context
	  Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	0
	  execution succeeds
	-ETIME
	  if timeout
*******************************************************************************/
static int dfc_reset_flash(struct dfc_context *context)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	uint32_t event, event_out;
	unsigned long timeo;
	int status;

	/* Send command */
	dfc_send_cmd(context, (uint16_t) flash_info->reset, 0xFFFFFFFF, 0);

	event = (context->dfc_mode->chip_select) ?
	    NDSR_CS1_CMDD : NDSR_CS0_CMDD;

	/* Wait for CMDDM(command done successfully) */
	status = dfc_wait_event(context, event, &event_out,
				NAND_OTHER_TIMEOUT, 0);

	if (status) {
		ClearCMDBuf();
		return status;
	}

	/* Wait until flash device is stable or timeout (10ms) */
	timeo = jiffies + HZ;
	do {
		if (monahans_df_dev_ready(NULL))
			break;
	} while (time_before(jiffies, timeo));

	return 0;
}

static int dfc_readid(struct dfc_context *context, uint32_t * id)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	uint32_t event_out;
	int status;
	char tmp[DFC_DATA_SIZE_ID];

	/* Send command */
	status = dfc_send_cmd(context, (uint16_t) flash_info->read_id,
			      0xFFFFFFFF, 0);
	if (status) {
		ClearCMDBuf();
		return status;
	}

	/* Wait for CMDDM(command done successfully) */
	status = dfc_wait_event(context, NDSR_RDDREQ, &event_out,
				NAND_OTHER_TIMEOUT, 0);
	if (status) {
		ClearCMDBuf();
		return status;
	}
	dfc_read_fifo_partial(context, (unsigned char *)tmp,
			      context->flash_info->read_id_bytes,
			      DFC_DATA_SIZE_ID);

	*id = tmp[0] | (tmp[1] << 8);
	return 0;
}

#define ERR_NONE		0x0
#define ERR_DMABUSERR		(-0x01)
#define ERR_SENDCMD		(-0x02)
#define ERR_DBERR		(-0x03)
#define ERR_BBERR		(-0x04)
#define ERR_BUSY		(-0x05)

#define STATE_CMD_SEND		0x1
#define STATE_CMD_HANDLE	0x2
#define STATE_DMA_TRANSFER	0x3
#define STATE_DMA_DONE		0x4
#define STATE_READY		0x5
#define STATE_SUSPENDED		0x6
#define	STATE_DATA_TRANSFER	0x7

#define NAND_RELOC_MAX		127
#define NAND_RELOC_HEADER	0x524e
#define MAX_CHIP		1
#define NAND_CMD_DMA_LEN	12

#define MAX_TIM_SIZE	0x1000
#define MAX_BBT_SLOTS	24

struct reloc_item {
	unsigned short from;
	unsigned short to;
};

struct reloc_table {
	unsigned short header;
	unsigned short total;
	struct reloc_item reloc[NAND_RELOC_MAX];
};

struct monahans_dfc_info {
	unsigned int state;
	struct dfc_context *context;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	dma_addr_t data_buf_addr;
	char *data_buf;
	int data_dma;
	struct pxa_dma_desc *data_desc;
	dma_addr_t data_desc_addr;
	dma_addr_t cmd_buf_addr;
	char *cmd_buf;
	int cmd_dma;
	struct pxa_dma_desc *cmd_desc;
	dma_addr_t cmd_desc_addr;
	u64 dma_mask;
#else
	char *data_buf;
#endif
	u32 current_slot;
	struct reloc_table table;
	unsigned int table_init;
	/* relate to the command */
	unsigned int cmd;
	unsigned int cur_cmd;
	unsigned int addr;
	unsigned int column;
	int retcode;
	unsigned int buf_count;
	struct completion cmd_complete;
};

static struct dfc_mode dfc_mode = {
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	1,			/* enable DMA */
#else
	0,
#endif
	1,			/* enable ECC */
	1,			/* enable SPARE */
	0,			/* CS0 */
};

struct dfc_context dfc_context = {
	0,			/* Initialized at function monahans_df_init() */
	&dfc_mode,
	0,			/* data dma channel */
	0,			/* cmd dma channel */
	NULL,			/* &zylonite_flashinfo */
};

/*
 * MTD structure for Zylonite board
 */
static struct mtd_info *monahans_mtd = NULL;

/*
 * BootRom and XDB will use last 127 block, and they will keep all the status
 * of the bootloader and image, so skip the first 2M size and last 2M size
 */
static struct mtd_partition partition_info[] = {
	{
		name:		"Bootloader",
		offset:		0,
		mask_flags:	MTD_WRITEABLE  /* force read-only */
	},{
		name:		"Kernel",
		size:		0x00200000,
	},{
		name:		"Filesystem",
		size:		0x03000000,	/* only mount 48M fs */
	}, {
		name:		"MassStorage",
		size:		0x0, /* It will be set at probe function */
		offset:		MTDPART_OFS_APPEND /* Append after fs section */
	}, {
		name:		"BBT",
		size:		0x0, /* It will be set at probe function */
		offset:		MTDPART_OFS_APPEND,/* Append after fs section */
		mask_flags:	MTD_WRITEABLE  /* force read-only */
	}
};

#define		PART_NUM	ARRAY_SIZE(partition_info)

/* MHN_OBM_V2 is related to BBT in MOBM V2
 * MHN_OBM_V3 is related to BBT in MOBM V3
 */
enum {
	MHN_OBM_NULL = 0,
	MHN_OBM_V1,
	MHN_OBM_V2,
	MHN_OBM_V3,
	MHN_OBM_INVAL
} MHN_OBM_TYPE;

static uint8_t scan_ff_pattern[] = { 0xff, 0xff };
static uint8_t scan_main_bbt_pattern[] = { 'p', 'x', 'a', '1' };
static uint8_t scan_mirror_bbt_pattern[] = { '0', 'a', 'x', 'p' };

static struct nand_bbt_descr monahans_bbt_default = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_2BIT | NAND_BBT_VERSION,
	.maxblocks = 2,
	.len = 2,
	.offs = 0,
	.pattern = scan_ff_pattern,
};

static struct nand_bbt_descr monahans_bbt_main = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_2BIT | NAND_BBT_VERSION,
	.veroffs = 6,
	.maxblocks = 2,
	.offs = 2,
	.len = 4,
	.pattern = scan_main_bbt_pattern,
};

static struct nand_bbt_descr monahans_bbt_mirror = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_2BIT | NAND_BBT_VERSION,
	.veroffs = 6,
	.maxblocks = 2,
	.offs = 2,
	.len = 4,
	.pattern = scan_mirror_bbt_pattern,
};

static struct nand_oobinfo monahans_lb_nand_oob = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 24,
	.eccpos = {
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {{2, 38}}
};

/*
 * Monahans OOB size is only 8 bytes, and the rest 8 bytes is controlled by
 * hardware for ECC. We construct virutal ECC buffer. Acutally, ECC is 6 bytes
 * and the remain 2 bytes are reserved.
 */
static struct nand_oobinfo monahans_sb_nand_oob = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 6,
	.eccpos = {8, 9, 10, 11, 12, 13},
	.oobfree = {{2, 6}}
};

static inline int is_buf_blank(u8 * buf, int size)
{
	int i = 0;
	while (i < size) {
		if (*((unsigned long *)(buf + i)) != 0xFFFFFFFF)
			return 0;
		i += 4;
	}
	if (i > size) {
		i -= 4;
		while (i < size) {
			if (*(buf + i) != 0xFF)
				return 0;
			i++;
		}
	}
	return 1;
}

static void print_buf(char *buf, int num)
{
	int i = 0;

	while (i < num) {
		pr_debug(KERN_ERR "0x%08x: %02x %02x %02x %02x %02x %02x %02x"
			 " %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			 (unsigned int)(i), buf[i], buf[i + 1], buf[i + 2],
			 buf[i + 3], buf[i + 4], buf[i + 5], buf[i + 6],
			 buf[i + 7], buf[i + 8], buf[i + 9], buf[i + 10],
			 buf[i + 11], buf[i + 12], buf[i + 13], buf[i + 14],
			 buf[i + 15]);
		i += 16;
	}
}

static int inline enable_dfc_dma(struct dfc_context *context, int enable)
{
	int ret = dfc_mode.enable_dma;
	unsigned long ndcr;

	if (!enable) {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr &= ~NDCR_DMA_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_dma = 0;
	} else {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr |= NDCR_DMA_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_dma = 1;
	}
	return ret;
}

static void inline dump_info(struct monahans_dfc_info *info)
{
	if (!info)
		return;

	printk(KERN_ERR "cmd:0x%x; addr:0x%x; retcode:%d; state:%d \n",
	       info->cur_cmd, info->addr, info->retcode, info->state);
}

static void inline enable_hw_ecc(struct dfc_context *context, int enable)
{
	unsigned long ndcr;

	if (!enable) {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr &= ~NDCR_ECC_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_ecc = 0;
	} else {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr |= NDCR_ECC_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_ecc = 1;
	}
}

/*
 * Now, we are not sure that the NDSR_RDY mean the flash is ready.
 * Need more test.
 */
static int monahans_df_dev_ready(struct mtd_info *mtd)
{
	return ((NDSR & NDSR_RDY));
}

/* each read, we can only read 4bytes from NDDB, we must buffer it */
static u_char monahans_df_read_byte(struct mtd_info *mtd)
{
	char retval = 0xFF;
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	if (info->column < info->buf_count) {
		/* Has just send a new command? */
		retval = info->data_buf[info->column++];
	}
	return retval;
}

static void monahans_df_write_byte(struct mtd_info *mtd, u8 byte)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);
	info->data_buf[info->column++] = byte;
}

static u16 monahans_df_read_word(struct mtd_info *mtd)
{
	u16 retval = 0xFFFF;
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	if (!(info->column & 0x01) && info->column < info->buf_count) {
		retval = *((u16 *) (info->data_buf + info->column));
		info->column += 2;
	}
	return retval;
}

static void monahans_df_write_word(struct mtd_info *mtd, u16 word)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	if (!(info->column & 0x01) && info->column < info->buf_count) {
		*((u16 *) (info->data_buf + info->column)) = word;
		info->column += 2;
	}
}

static void monahans_df_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);
	int real_len = min((unsigned int)len, info->buf_count - info->column);

	memcpy(buf, info->data_buf + info->column, real_len);
	info->column += real_len;
}

static void monahans_df_write_buf(struct mtd_info *mtd,
				  const u_char * buf, int len)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);
	int real_len = min((unsigned int)len, info->buf_count - info->column);

	memcpy(info->data_buf + info->column, buf, real_len);
	info->column += real_len;
}

static int monahans_df_verify_buf(struct mtd_info *mtd,
				  const u_char * buf, int len)
{
	return 0;
}

#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifndef CONFIG_MTD_NAND_PXA310_FIX
static void monahans_dfc_cmd_dma_irq(int channel, void *data,
				     struct pt_regs *regs)
{
	unsigned int dcsr;
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)data;
	struct dfc_context *context = info->context;
	struct dfc_mode *dfc_mode = context->dfc_mode;
	unsigned int intm;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	intm = (dfc_mode->chip_select) ?
	    (NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);

	pr_debug("cmd dma interrupt, channel:%d, DCSR:0x%08x\n", channel, dcsr);

	if (dcsr & DCSR_BUSERR) {
		info->retcode = ERR_DMABUSERR;
		complete(&info->cmd_complete);
	} else {
		if ((info->cmd == NAND_CMD_READ0) ||
		    (info->cmd == NAND_CMD_READOOB) ||
		    (info->cmd == NAND_CMD_READID) ||
		    (info->cmd == NAND_CMD_STATUS)) {
			dfc_enable_int(context, NDSR_RDDREQ | NDSR_DBERR);
		} else if (info->cmd == NAND_CMD_PAGEPROG)
			dfc_enable_int(context, NDSR_WRDREQ);
		else if (info->cmd == NAND_CMD_ERASE1)
			dfc_enable_int(context, intm);
	}
}
#endif

static void monahans_dfc_data_dma_irq(int channel, void *data,
				      struct pt_regs *regs)
{
	unsigned int dcsr, intm;
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)data;
	struct dfc_context *context = info->context;
	struct dfc_mode *dfc_mode = context->dfc_mode;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	intm = (dfc_mode->chip_select) ?
	    (NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);

	pr_debug("data dma interrupt, channel:%d, DCSR:0x%08x\n", channel,
		 dcsr);

	if (dcsr & DCSR_BUSERR) {
		info->retcode = ERR_DMABUSERR;
		complete(&info->cmd_complete);
	}

	if (info->cmd == NAND_CMD_PAGEPROG) {
		/* DMA interrupt may be interrupted by other IRQs */
		info->state = STATE_DMA_DONE;
		dfc_enable_int(context, intm);
	} else {
		info->state = STATE_READY;
		complete(&info->cmd_complete);
	}

}
#endif

static irqreturn_t monahans_dfc_irq(int irq, void *devid, struct pt_regs *regs)
{
	unsigned int status, event, intm, cmd;
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)devid;
	struct dfc_context *context = info->context;
	struct dfc_mode *dfc_mode = context->dfc_mode;

	intm = (dfc_mode->chip_select) ?
	    (NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);
	event = (dfc_mode->chip_select) ?
	    (NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);

	status = dfc_read(context, DFC_NDSR);
	pr_debug("DFC irq, NDSR:0x%x\n", status);
	if (status & (NDSR_RDDREQ | NDSR_DBERR)) {
		if (status & NDSR_DBERR) {
			info->retcode = ERR_DBERR;
		}

		dfc_disable_int(context, NDSR_RDDREQ | NDSR_DBERR);
		dfc_clear_int(context, NDSR_RDDREQ | NDSR_DBERR);
		if (info->cmd == NAND_CMD_READID)
			cmd = context->flash_info->read_id;
		else if (info->cmd == NAND_CMD_STATUS)
			cmd = context->flash_info->read_status;
		else if (info->cmd == NAND_CMD_READ0 ||
			 info->cmd == NAND_CMD_READOOB)
			cmd = context->flash_info->read1;
		else {
			printk(KERN_ERR "No according command:0x%x happens\n",
			       info->cmd);
			goto out;
		}
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifdef CACHE_READ
		if (info->cur_cmd == 0x34) {
			info->state = STATE_DATA_TRANSFER;
			complete(&info->cmd_complete);
			goto out;
		}
#endif
		info->state = STATE_DMA_TRANSFER;
		dfc_start_data_dma(context,
				   (struct pxa_dma_desc *)info->data_desc_addr);
#else
		info->state = STATE_DATA_TRANSFER;
		complete(&info->cmd_complete);
#endif
	} else if (status & NDSR_WRDREQ) {
		dfc_disable_int(context, NDSR_WRDREQ);
		dfc_clear_int(context, NDSR_WRDREQ);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
		info->state = STATE_DMA_TRANSFER;
		dfc_start_data_dma(context,
				   (struct pxa_dma_desc *)info->data_desc_addr);
#else
		info->state = STATE_DATA_TRANSFER;
		complete(&info->cmd_complete);
#endif
	} else if (status & event) {
		if (status & NDSR_CS0_BBD) {
			info->retcode = ERR_BBERR;
		}

		dfc_disable_int(context, intm);
		dfc_clear_int(context, event);
		info->state = STATE_READY;
		complete(&info->cmd_complete);
	}
      out:
	return IRQ_HANDLED;
}

static int dfc_send_command(struct mtd_info *mtd, unsigned int cmd,
			    unsigned int addr, unsigned int num_pages,
			    unsigned int event)
{

	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);
	struct dfc_context *context = info->context;
	int status;
	int ret;

	pr_debug("ready send command, cmd:0x%x, at address:0x%x,"
		 " num_pages:%d, wait event:0x%x\n", cmd, addr, num_pages,
		 event);

	info->state = STATE_CMD_SEND;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	info->cur_cmd = cmd;
#ifdef CONFIG_MTD_NAND_PXA310_FIX
	enable_dfc_dma(context, 0);
	status = dfc_send_cmd(context, cmd, addr, num_pages);
#else
	status = dfc_setup_cmd_dma(context, cmd, addr, num_pages,
				   (uint32_t *) info->cmd_buf,
				   info->cmd_buf_addr, DDADR_STOP,
				   DCMD_ENDIRQEN, info->cmd_desc);
#endif

#else
	status = dfc_send_cmd(context, cmd, addr, num_pages);
#endif
	if (status) {
		info->retcode = ERR_SENDCMD;
		dfc_stop(context);
		udelay(20);
		printk(KERN_ERR "fail send command\n");
		return info->retcode;
	}
	info->state = STATE_CMD_HANDLE;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	dfc_setup_data_dma(context, cmd, info->data_buf_addr,
			   DDADR_STOP, DCMD_ENDIRQEN, info->data_desc);
#ifdef CONFIG_MTD_NAND_PXA310_FIX
	enable_dfc_dma(context, 1);
	dfc_enable_int(context, event);
#else
	dfc_start_cmd_dma(context, (struct pxa_dma_desc *)info->cmd_desc_addr);
#endif
#endif
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
	dfc_enable_int(context, event);
#endif
	ret = wait_for_completion_timeout(&info->cmd_complete, 2 * HZ);
	if (!ret) {
		printk(KERN_ERR "Command time out\n");
		dump_info(info);
	}
	pr_debug("command return, cmd:0x%x, retcode:%d\n",
		 info->cmd, info->retcode);
	return 0;
}

static void monahans_df_command(struct mtd_info *mtd, unsigned command,
				int column, int page_addr)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	struct monahans_dfc_info *info =
	    (struct monahans_dfc_info *)(this->priv);
	struct dfc_context *context = info->context;
	struct dfc_flash_info *flash_info = context->flash_info;
	int ret, pages_shift;
	int status;
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
	unsigned int datasize;
	unsigned int paddingsize;
#endif
	unsigned int to;

	pr_debug("command:0x%x at address:0x%x, column:0x%x\n",
		 command, page_addr, column);

	if (info->state != STATE_READY) {
		printk(KERN_ERR "CHIP is not ready.\n");
		dump_info(info);
		info->retcode = ERR_BUSY;
		return;
	}
	info->retcode = ERR_NONE;
	pages_shift = this->phys_erase_shift - this->page_shift;
	if (info->table_init) {
		to = search_rel_block((page_addr >> pages_shift), mtd);
		if (to) {
			page_addr = (to << pages_shift) | (page_addr
							   & ((1 << pages_shift)
							      - 1));
		}
	}

	switch (command) {
	case NAND_CMD_READOOB:
		/*
		 * DFC has mark the last 8 bytes OOB data if HARDEARE_ECC is
		 * enabled. We must first disable the HARDWARE_ECC for getting
		 * all the 16 bytes OOB
		 */
		enable_hw_ecc(context, 0);
		info->buf_count = mtd->oobblock + mtd->oobsize;
		info->column = mtd->oobblock + column;
		info->cmd = command;
		info->addr = page_addr << this->page_shift;
		ret = dfc_send_command(mtd, flash_info->read1, info->addr,
				       1, NDSR_RDDREQ | NDSR_DBERR);
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(info->context, flash_info->read1, &datasize,
				&paddingsize);
		dfc_read_fifo_partial(info->context, info->data_buf,
				      min(info->buf_count, datasize), datasize);
		info->state = STATE_READY;
#endif
#ifdef CACHE_READ
		if ((flash_info->chip_id == 0xba20) || (flash_info->chip_id == 0xb120)){
			ret = dfc_send_command(mtd, 0x34,
					0xFFFFFFFF, 0, NDSR_RDDREQ);
			ClearCMDBuf();
			info->state = STATE_READY;
		}
#endif
		/* We only are OOB, so if the data has error, does not matter */
		if (info->retcode == ERR_DBERR)
			info->retcode = ERR_NONE;
		enable_hw_ecc(context, 1);
		break;

	case NAND_CMD_READ0:
		enable_hw_ecc(context, 1);
		info->column = column;
		info->cmd = command;
		info->buf_count = mtd->oobblock + mtd->oobsize;
		memset(info->data_buf, 0xFF, info->buf_count);
		info->addr = page_addr << this->page_shift;

		ret = dfc_send_command(mtd, flash_info->read1, info->addr,
				       1, NDSR_RDDREQ | NDSR_DBERR);
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(info->context, flash_info->read1, &datasize,
				&paddingsize);
		dfc_read_fifo_partial(info->context, info->data_buf,
				      min(info->buf_count, datasize), datasize);
		info->state = STATE_READY;
#endif
#ifdef CACHE_READ
		if ((flash_info->chip_id == 0xba20) || (flash_info->chip_id == 0xb120)){
			ret = dfc_send_command(mtd, 0x34,
					0xFFFFFFFF, 0, NDSR_RDDREQ);
			ClearCMDBuf();
			info->state = STATE_READY;
		}
#endif
		/* When the data buf is blank, the DFC will report DB error */
		if (info->retcode == ERR_DBERR && is_buf_blank(info->data_buf,
							       mtd->oobblock))
			info->retcode = ERR_NONE;

		if (info->retcode == ERR_DBERR) {
			printk(KERN_ERR "DB error at address 0x%x\n",
			       info->addr);
			print_buf(info->data_buf, info->buf_count);
		}
		break;
	case NAND_CMD_SEQIN:
		/* Write only OOB? */

		info->cmd = command;
		if (column >= mtd->oobblock) {
			info->buf_count = mtd->oobblock + mtd->oobsize;
			enable_hw_ecc(context, 0);
		} else {
			info->buf_count = mtd->oobblock + mtd->oobsize;
			enable_hw_ecc(context, 1);
		}
		memset(info->data_buf, 0xFF, mtd->oobblock + mtd->oobsize);
		info->column = column;
		info->addr = page_addr << this->page_shift;
		break;
	case NAND_CMD_PAGEPROG:
		/* prevois command is NAND_CMD_SEIN ? */
		if (info->cmd != NAND_CMD_SEQIN) {
			info->cmd = command;
			info->retcode = ERR_SENDCMD;
			printk(KERN_ERR "Monahans NAND device: "
			       "No NAND_CMD_SEQIN executed before.\n");
			enable_hw_ecc(context, 1);
			break;
		}
		info->cmd = command;
		ret = dfc_send_command(mtd, flash_info->program, info->addr,
				       1, NDSR_WRDREQ);

#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		if (ret != 0)
			break;

		dfc_get_pattern(info->context, flash_info->program, &datasize,
				&paddingsize);
		dfc_write_fifo_partial(info->context, info->data_buf, datasize,
				       datasize);

		if (info->context->dfc_mode->chip_select)
			dfc_enable_int(info->context,
				       NDSR_CS1_BBD | NDSR_CS1_CMDD);
		else
			dfc_enable_int(info->context,
				       NDSR_CS0_BBD | NDSR_CS0_CMDD);

		ret = wait_for_completion_timeout(&info->cmd_complete, 2 * HZ);
		if (!ret) {
			printk(KERN_ERR "Programm Command time out\n");
			dump_info(info);
		}

		if (info->retcode == ERR_BBERR) {
			mtd->block_markbad(mtd, info->addr);
		}
#endif
		break;
	case NAND_CMD_ERASE1:
		info->cmd = command;
		info->addr =
		    (page_addr >> pages_shift) << this->phys_erase_shift;

		if (info->context->dfc_mode->chip_select)
			ret = dfc_send_command(mtd, flash_info->erase,
					       info->addr, 0,
					       NDSR_CS1_BBD | NDSR_CS1_CMDD);
		else
			ret = dfc_send_command(mtd, flash_info->erase,
					       info->addr, 0,
					       NDSR_CS0_BBD | NDSR_CS0_CMDD);

		if (info->retcode == ERR_BBERR) {
			mtd->block_markbad(mtd, info->addr);
		}
		break;
	case NAND_CMD_ERASE2:
		break;
	case NAND_CMD_READID:
		info->cmd = command;
		info->buf_count = flash_info->read_id_bytes;
		info->column = 0;
		info->addr = 0xFFFFFFFF;
		ret = dfc_send_command(mtd, flash_info->read_id, info->addr,
				       0, NDSR_RDDREQ);
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(info->context, flash_info->read_id, &datasize,
				&paddingsize);
		dfc_read_fifo_partial(info->context, info->data_buf,
				      info->buf_count, datasize);
		info->state = STATE_READY;
#endif
		pr_debug("ReadID, [1]:0x%x, [2]:0x%x\n",
			 info->data_buf[0], info->data_buf[1]);
		break;
	case NAND_CMD_STATUS:
		info->cmd = command;
		info->buf_count = 1;
		info->column = 0;
		info->addr = 0xFFFFFFFF;
		ret = dfc_send_command(mtd, flash_info->read_status,
				       info->addr, 0, NDSR_RDDREQ);
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(info->context, flash_info->read_status,
				&datasize, &paddingsize);
		dfc_read_fifo_partial(info->context, info->data_buf,
				      info->buf_count, datasize);
		info->state = STATE_READY;
#endif
		break;

	case NAND_CMD_RESET:
		status = dfc_reset_flash(&dfc_context);
		if (status) {
			printk(KERN_WARNING "Monahans NAND device:"
			       "NAND_CMD_RESET error\n");
		}
		break;
	default:
		printk(KERN_WARNING "Monahans NAND device:"
		       "Non-support the command.\n");
		break;
	}

	if (info->retcode != ERR_NONE){
		ClearCMDBuf();
		info->state = STATE_READY;
        }
}

static void monahans_df_select_chip(struct mtd_info *mtd, int chip)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	if (chip <= MAX_CHIP)
		info->context->dfc_mode->chip_select = chip;
	else
		printk(KERN_ERR "Monahans NAND device:"
		       "not select the NAND chips!\n");
}

static int monahans_df_waitfunc(struct mtd_info *mtd,
				struct nand_chip *this, int state)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	/* monahans_df_send_command has waited for command complete */
	if (state == FL_WRITING || state == FL_ERASING) {
		if (info->retcode == ERR_NONE)
			return 0;
		else {
			/*
			 * any error make it return 0x01 which will tell
			 * the caller the erase and write fail
			 */
			return 0x01;
		}
	}

	return 0;
}

static int monahans_df_calculate_ecc(struct mtd_info *mtd,
				     const u_char * dat, u_char * ecc_code)
{
	return 0;
}

static int monahans_df_correct_data(struct mtd_info *mtd,
				    u_char * dat, u_char * read_ecc,
				    u_char * calc_ecc)
{
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	/*
	 * Any error include ERR_SEND_CMD, ERR_DBERR, ERR_BUSERR, we
	 * consider it as a ecc error which will tell the caller the
	 * read fail We have distinguish all the errors, but the
	 * nand_read_ecc only check this function return value
	 */
	if (info->retcode != ERR_NONE)
		return -1;

	return 0;
}

static void monahans_df_enable_hwecc(struct mtd_info *mtd, int mode)
{

}

/*
 * The relocation table management is different between MOBM V2 and V3.
 *
 * MOBM V2 is applied on chips taped out before MhnLV A0.
 * MOBM V3 is applied on chips taped out after MhnLV A0. It's also applied
 * on MhnLV A0.
 */
static int calc_obm_ver(void)
{
	unsigned int cpuid;
	/* read CPU ID */
      __asm__("mrc p15, 0, %0, c0, c0, 0\n":"=r"(cpuid)
	    );
	/* It's not xscale chip. */
	if ((cpuid & 0xFFFF0000) != 0x69050000)
		return MHN_OBM_INVAL;
	/* It's MhnP Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006420)
		return MHN_OBM_V2;
	/* It's MhnP Bx */
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		if ((cpuid & 0x0F) <= 6)
			return MHN_OBM_V2;
		else
			return MHN_OBM_V3;
	}
	/* It's MhnL Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		if ((cpuid & 0x0F) == 0)
			return MHN_OBM_V2;
		else
			return MHN_OBM_V3;
	}
	/* It's MhnLV Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006890)
		return MHN_OBM_V3;
	return MHN_OBM_INVAL;
}

/*
 * MOBM maintains a relocation table. It's used to replace bad blocks.
 * If block A is bad, it will use block B instead.
 * There're 127 relocated blocks. All of them reside in the bottom of NAND
 * flash. So they're reserved and can't be calculated in mtd size and chip
 * size.
 */
static int read_reloc_table(struct mtd_info *mtd)
{
	struct nand_chip *this = NULL;
	struct monahans_dfc_info *info = NULL;
	struct dfc_context *context = NULL;
	struct reloc_table *table = NULL;
	int page, maxslot;
	int obm, valid, num_blocks;

	obm = calc_obm_ver();
	this = (struct nand_chip *)(mtd->priv);
	info = (struct monahans_dfc_info *)(this->priv);
	context = info->context;

	num_blocks = this->chipsize >> this->phys_erase_shift;
	if ((num_blocks + NAND_RELOC_MAX) & NAND_RELOC_MAX) {
		mtd->size -= (NAND_RELOC_MAX * mtd->erasesize);
		this->chipsize -= (NAND_RELOC_MAX << this->phys_erase_shift);
	}

	page = (1 << (this->phys_erase_shift - this->page_shift)) - 1;

	this->select_chip(mtd, 0);
	valid = 0;
	if (obm == MHN_OBM_V2) {
		/* On MOBM V2, the relocation table resides in the last page
		 * of the first block.
		 */
		memset(info->data_buf, 0, BUFLEN);
		monahans_df_command(mtd, NAND_CMD_READ0, 0, page);
		memcpy(((unsigned char *)&(info->table)), info->data_buf,
		       sizeof(struct reloc_table));
		if (info->table.header == NAND_RELOC_HEADER)
			valid = 1;
	} else if (obm == MHN_OBM_V3) {
		/* On MOBM V3, there're several relocation tables in the first
		 * block.
		 * When new bad blocks are found, a new relocation table will
		 * be generated and written back to the first block. But the
		 * original relocation table won't be erased. Even if the new
		 * relocation table is written wrong, system can still find an
		 * old one.
		 * One page contains one slot.
		 */
		maxslot = 1 << (this->phys_erase_shift - this->page_shift);
		page = maxslot - MAX_BBT_SLOTS;
		for (; page < maxslot; page++) {
			monahans_df_command(mtd, NAND_CMD_READ0, 0, page);
			table = (struct reloc_table *)info->data_buf;
			if (info->retcode == ERR_NONE) {
				if (table->header != NAND_RELOC_HEADER) {
					continue;
				} else {
					memcpy(((unsigned char *)
						&(info->table)), table,
					       sizeof(struct reloc_table));
					valid = 1;
					break;
				}
			}
		}

	} else {
		printk(KERN_ERR "The version of MOBM isn't supported\n");
	}
	if (valid) {
		memcpy(((unsigned char *)&(info->table)), info->data_buf,
		       sizeof(struct reloc_table));
		printk(KERN_DEBUG "relocation table at page:%d\n", page);
		info->table_init = 1;
	} else {
		/* There should be a valid relocation table slot at least. */
		printk(KERN_ERR "NO VALID relocation table can be \
				recognized\n");
		printk(KERN_ERR "CAUTION: It may cause unpredicated error\n");
		printk(KERN_ERR "Please re-initialize the NAND flash.\n");
		memset((unsigned char *)&(info->table), 0,
		       sizeof(struct reloc_table));
		info->table_init = 0;
		return -EINVAL;
	}
	return 0;
}

/* add the relocation entry into the relocation table
 * It's valid on MOBM V3.
 * If the relocated block is bad, an new entry will be added into the
 * bottom of the relocation table.
 */
static int update_rel_table(struct mtd_info *mtd, int block)
{
	struct nand_chip *this = NULL;
	struct monahans_dfc_info *info = NULL;
	struct reloc_table *table = NULL;
	int obm, reloc_block;

	this = (struct nand_chip *)(mtd->priv);
	info = (struct monahans_dfc_info *)(this->priv);
	obm = calc_obm_ver();
	if (obm == MHN_OBM_V3) {
		table = &info->table;
		if (info->table_init == 0) {
			printk(KERN_ERR "Error: the initial relocation \
					table can't be read\n");
			memset(table, 0, sizeof(struct reloc_table));
			table->header = NAND_RELOC_HEADER;
			info->table_init = 1;
		}
		if (table->total == 0) {
			/* Point to the first relocated block.
			 * It resides in the last block of flash.
			 * the relocation entry has calculated in
			 * chipsize
			 */
			reloc_block = (this->chipsize >> this->phys_erase_shift)
			    + NAND_RELOC_MAX - 1;
		} else if (table->total < NAND_RELOC_MAX) {
			reloc_block = table->reloc[table->total - 1].to - 1;
		} else {
			printk(KERN_ERR "Relocation table exceed max number, \
				cannot mark block 0x%x as bad block\n", block);
			return -ENOSPC;
		}
		/* Make sure that reloc_block is pointing to a valid block */
		for (;; reloc_block--) {
			/* The relocate table is full */
			if (reloc_block < (this->chipsize
					   >> this->phys_erase_shift))
				return -ENOSPC;
			this->cmdfunc(mtd, NAND_CMD_ERASE1, 0, reloc_block
				      << (this->phys_erase_shift
					  - this->page_shift));
			if (info->retcode == ERR_NONE)
				break;
		}
		/* Create the relocated block information in the table */
		table->reloc[table->total].from = block;
		table->reloc[table->total].to = reloc_block;
		table->total++;
	}
	return 0;
}

/* Write the relocation table back to device, if there's room. */
static int sync_rel_table(struct mtd_info *mtd, int *idx)
{
	struct nand_chip *this = NULL;
	struct monahans_dfc_info *info = NULL;
	int obm, start_page, len;

	if (*idx >= MAX_BBT_SLOTS) {
		printk(KERN_ERR "Can't write relocation table to device \
				any more.\n");
		return -1;
	}
	if (*idx < 0) {
		printk(KERN_ERR "Wrong Slot is specified.\n");
		return -1;
	}
	this = (struct nand_chip *)(mtd->priv);
	info = (struct monahans_dfc_info *)(this->priv);
	len = 4;
	len += info->table.total << 2;
	obm = calc_obm_ver();
	if (obm == MHN_OBM_V3) {
		/* write to device */
		start_page = 1 << (this->phys_erase_shift - this->page_shift);
		start_page = start_page - 1 - *idx;
		memset(&(info->data_buf), 0xFF, BUFLEN);
		memcpy(&(info->data_buf), &(info->table), len);

		printk(KERN_DEBUG "DUMP relocation table before write. \
				page:0x%x\n", start_page);
		monahans_df_command(mtd, NAND_CMD_SEQIN, 0, start_page);
		monahans_df_command(mtd, NAND_CMD_PAGEPROG, 0, start_page);
		/* write to idx */
		(*idx)++;
		/* dump it */
		memset(&(info->data_buf), 0, BUFLEN);
		monahans_df_command(mtd, NAND_CMD_READOOB, 0, start_page);
	}
	return 0;
}

/* Find the relocated block of the bad one.
 * If it's a good block, return 0. Otherwise, return a relocated one.
 * idx points to the next relocation entry
 * If the relocated block is bad, an new entry will be added into the
 * bottom of the relocation table.
 */
static unsigned short search_rel_block(int block, struct mtd_info *mtd)
{
	struct nand_chip *this = NULL;
	struct monahans_dfc_info *info = NULL;
	struct reloc_table *table = NULL;
	int i, max, reloc_block = 0;

	this = (struct nand_chip *)(mtd->priv);
	info = (struct monahans_dfc_info *)(this->priv);
	table = &(info->table);
	if ((block <= 0) || (block > this->chipsize)
	    || (info->table_init == 0) || (table->total == 0))
		return 0;
	if (table->total > NAND_RELOC_MAX)
		table->total = NAND_RELOC_MAX;
	max = table->total;
	for (i = 0; i < max; i++) {
		if (block == table->reloc[i].from)
			reloc_block = table->reloc[i].to;
	}
	return reloc_block;
}

/*
 * Check whether the block is a bad one.
 * At first, it will search the relocation table.
 * If necessary, it will search the BBT. Because relocation table can only
 * maintain limited record. If there're more bad blocks, they can't be
 * recorded in relocation table. They can only be recorded in BBT.
 */
static int monahans_df_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	struct nand_chip *this = NULL;
	int page, block, reloc_block, chipnr, res = 0;
	u16 bad;

	/* At here, we only support one flash chip */
	this = (struct nand_chip *)mtd->priv;
	block = (int)(ofs >> this->phys_erase_shift);
	/* search the block in the relocation table */
	reloc_block = search_rel_block(block, mtd);
	if (reloc_block) {
		ofs = ((reloc_block << this->phys_erase_shift) |
		       (ofs & ((1 << this->phys_erase_shift) - 1)));
	}

	/* search BBT
	 * Maybe the relocation table is full, but some bad blocks aren't
	 * recordered in it.
	 * The below code are copied from nand_block_bad().
	 */
	if (getchip) {
		page = (int)(ofs >> this->page_shift);
		chipnr = (int)(ofs >> this->chip_shift);

		/* Select the NAND chips */
		this->select_chip(mtd, chipnr);
	} else
		page = (int)ofs;

	if (this->options & NAND_BUSWIDTH_16) {
		this->cmdfunc(mtd, NAND_CMD_READOOB, this->badblockpos & 0xFE,
			      page & this->pagemask);
		bad = cpu_to_le16(this->read_word(mtd));
		if (this->badblockpos & 0x1)
			bad >>= 1;
		if ((bad & 0xFF) != 0xFF)
			res = 1;
	} else {
		this->cmdfunc(mtd, NAND_CMD_READOOB, this->badblockpos,
			      page & this->pagemask);
		if (this->read_byte(mtd) != 0xFF)
			res = 1;
	}

	return res;
}

static int monahans_df_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *this = NULL;
	struct monahans_dfc_info *info = NULL;
	unsigned char buf[2] = { 0, 0 };
	int block, reloc_block, page, ret;

	this = (struct nand_chip *)mtd->priv;
	info = (struct monahans_dfc_info *)(this->priv);
	/* Get block number */
	block = ((int)ofs) >> this->bbt_erase_shift;
	ret = update_rel_table(mtd, block);
	if (!ret) {
		sync_rel_table(mtd, &(info->current_slot));
		return 0;
	} else {
		reloc_block = search_rel_block(block, mtd);
		if (reloc_block)
			block = reloc_block;
		if (this->bbt)
			this->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);
	}

	/* Do we have a flash based bad block table ? */
	if (this->options & NAND_USE_FLASH_BBT)
		return nand_update_bbt(mtd, ofs);

	/* mark the bad block flag at the first two pages */
	page = block << (this->phys_erase_shift - this->page_shift);
	ofs = mtd->oobblock + this->badblockpos;
	this->cmdfunc(mtd, NAND_CMD_SEQIN, ofs, page);
	this->write_buf(mtd, buf, 2);
	this->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	page++;
	this->cmdfunc(mtd, NAND_CMD_SEQIN, ofs, page);
	this->write_buf(mtd, buf, 2);
	this->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	return 0;
}

static int dump_bbt_mem(struct mtd_info *mtd)
{
	struct nand_chip *this = NULL;

	this = (struct nand_chip *)mtd->priv;
	print_buf(this->bbt, 225);
	return 0;
}

static int monahans_df_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *this = NULL;
	int ret;

	this = (struct nand_chip *)mtd->priv;
	ret = read_reloc_table(mtd);
	if (ret) {
		printk(KERN_ERR "Failed to get relocation table\n");
		printk(KERN_ERR "Try to build a new BBT. It may result \
				unpredicated error.\n");
		/* Create new memory based and flash based BBT */
	}
	nand_scan_bbt(mtd, &monahans_bbt_default);

	dump_bbt_mem(mtd);
	return 0;
}

static int monahans_df_probe(struct device *dev)
{
	struct nand_chip *this;
	struct monahans_dfc_info *info;
	int status = -1;
	unsigned int data_buf_len;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	unsigned int buf_len;
#endif
	int i, ret = 0;
	int obm;
	struct platform_device *pdev = to_platform_device(dev);

	dfc_context.membase = (void *)(&NDCR);

	pxa_set_cken(CKEN_NAND, 1);

	for (i = DFC_FLASH_NULL + 1; i < DFC_FLASH_END; i++) {
		uint32_t id;

		status = dfc_init(&dfc_context, i);
		if (status)
			continue;
		status = dfc_readid(&dfc_context, &id);
		if (status)
			continue;
		printk(KERN_DEBUG "id:0x%x, chipid:0x%x\n",
		       id, dfc_context.flash_info->chip_id);
		if (id == dfc_context.flash_info->chip_id)
			break;
	}

	if (i == DFC_FLASH_END) {
		printk(KERN_ALERT "Monahans NAND device:"
		       "Nand Flash initialize failure!\n");
		ret = -ENXIO;
		goto out;
	}
	flash_config = i;

	monahans_mtd =
	    kzalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip) +
		    sizeof(struct monahans_dfc_info), GFP_KERNEL);
	if (!monahans_mtd) {
		printk(KERN_ERR "Monahans NAND device:"
		       "Unable to allocate NAND MTD device structure.\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Get pointer to private data */
	this =
	    (struct nand_chip *)((void *)monahans_mtd +
				 sizeof(struct mtd_info));
	info =
	    (struct monahans_dfc_info *)((void *)this +
					 sizeof(struct nand_chip));

	monahans_mtd->priv = this;
	this->priv = info;
	data_buf_len = dfc_context.flash_info->page_size +
	    dfc_context.flash_info->oob_size;
	info->state = STATE_READY;
	init_completion(&info->cmd_complete);
	info->table_init = 0;
	memset(&info->table, 0x0, sizeof(struct reloc_table));
	printk(KERN_DEBUG
	       "%s: this->controller: 0x%x, &this->controller: 0x%x\n",
	       __func__, (unsigned int)this->controller,
	       (unsigned int)&(this->controller));
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	info->dma_mask = 0xffffffffUL;

	dev->dma_mask = &info->dma_mask;
	dev->coherent_dma_mask = 0xffffffffUL;

#ifndef CONFIG_MTD_NAND_PXA310_FIX
	/* alloc dma data buffer for data
	 * buffer + 2*descriptor + command buffer
	 */
	buf_len = ALIGN(2 * sizeof(struct pxa_dma_desc), 32) +
	    ALIGN(data_buf_len, 32) + ALIGN(NAND_CMD_DMA_LEN, 32);
#else
	buf_len = ALIGN(sizeof(struct pxa_dma_desc), 32) +
	    ALIGN(data_buf_len, 32);
#endif
	printk(KERN_INFO "Try to allocate dma buffer(len:%d)"
	       "for data buffer + 2*descriptor + command buffer\n", buf_len);
	info->data_desc = (struct pxa_dma_desc *)dma_alloc_writecombine(dev,
									buf_len,
									&info->
									data_desc_addr,
									GFP_KERNEL);
	if (!info->data_desc) {
		printk(KERN_ERR "Monahans NAND device:"
		       "Unable to alloc dma buffer\n");
		ret = -ENOMEM;
		goto free_mtd;
	}
#ifndef CONFIG_MTD_NAND_PXA310_FIX
	info->cmd_desc = (struct pxa_dma_desc *)((char *)info->data_desc +
						 sizeof(struct pxa_dma_desc));
	info->cmd_desc_addr = (dma_addr_t) ((char *)info->data_desc_addr +
					    sizeof(struct pxa_dma_desc));
#endif
	info->data_buf = (char *)info->data_desc +
	    ALIGN(2 * sizeof(struct pxa_dma_desc), 32);
	info->data_buf_addr = (dma_addr_t) ((char *)info->data_desc_addr +
					    ALIGN(2 *
						  sizeof(struct pxa_dma_desc),
						  32));
#ifndef CONFIG_MTD_NAND_PXA310_FIX
	info->cmd_buf = (char *)info->data_buf + ALIGN(data_buf_len, 32);
	info->cmd_buf_addr = (dma_addr_t) ((char *)info->data_buf_addr +
					   ALIGN(data_buf_len, 32));
#endif
	pr_debug
	    ("Get dma buffer for data dma descriptor, virt:0x%x, phys0x:%x\n",
	     (unsigned int)info->data_desc, info->data_desc_addr);
	pr_debug("Get dma buffer for command dma descriptors, virt:0x%x,"
		 "phys0x:%x\n", (unsigned int)info->cmd_desc,
		 info->cmd_desc_addr);
	pr_debug("Get dma buffer for data, virt:0x%x, phys0x:%x\n",
		 (unsigned int)info->data_buf, info->data_buf_addr);
	pr_debug("Get dma buffer for command, virt:0x%x, phys0x:%x\n",
		 (unsigned int)info->cmd_buf, info->cmd_buf_addr);

	pr_debug("Try to allocate dma channel for data\n");

	info->data_dma = pxa_request_dma("NAND DATA", DMA_PRIO_LOW,
					 monahans_dfc_data_dma_irq, info);
	if (info->data_dma < 0) {
		printk(KERN_ERR "Monahans NAND device:"
		       "Unable to alloc dma channel for data\n");
		ret = info->data_dma;
		goto free_buf;
	}
	pr_debug("Get dma channel:%d for data\n", info->data_dma);

#ifndef CONFIG_MTD_NAND_PXA310_FIX
	pr_debug("Try to allocate dma channel for command\n");
	info->cmd_dma = pxa_request_dma("NAND CMD", DMA_PRIO_LOW,
					monahans_dfc_cmd_dma_irq, info);
	if (info->cmd_dma < 0) {
		printk(KERN_ERR "Monahans NAND device:"
		       "Unable to alloc dma channel for command\n");
		ret = info->cmd_dma;
		goto free_data_dma;
	}
	pr_debug("Get dma channel:%d for command\n", info->cmd_dma);

	dfc_context.cmd_dma_ch = info->cmd_dma;
#endif
	dfc_context.data_dma_ch = info->data_dma;
#else
	printk(KERN_DEBUG "Try to allocate data buffer(len:%d)\n",
	       data_buf_len);
	info->data_buf = kmalloc(data_buf_len, GFP_KERNEL);
	if (!info->data_buf) {
		printk(KERN_ERR "Monahans NAND device:"
		       "Unable to alloc data buffer\n");
		ret = -ENOMEM;
		goto free_mtd;
	}
#endif

	pr_debug("Try to request irq:%d\n", IRQ_NAND);
	ret = request_irq(IRQ_NAND, monahans_dfc_irq, 0, pdev->name, info);
	if (ret < 0) {
		printk(KERN_ERR
		       "Monahans NAND device: Unable to request irq\n");
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
		goto free_cmd_dma;
#else
		goto free_buf;
#endif
	}

	pr_debug("Success request irq\n");

	/* set address of NAND IO lines */
	this->options = (dfc_context.flash_info->flash_width == 16) ?
	    NAND_BUSWIDTH_16 : 0 | NAND_USE_FLASH_BBT;

	/* this->IO_ADDR_R = this->IO_ADDR_W = NDDB */
	this->waitfunc = monahans_df_waitfunc;
	this->select_chip = monahans_df_select_chip;
	this->dev_ready = monahans_df_dev_ready;
	this->cmdfunc = monahans_df_command;
	this->read_word = monahans_df_read_word;
	this->write_word = monahans_df_write_word;
	this->read_byte = monahans_df_read_byte;
	this->write_byte = monahans_df_write_byte;
	this->read_buf = monahans_df_read_buf;
	this->write_buf = monahans_df_write_buf;
	this->verify_buf = monahans_df_verify_buf;
	this->enable_hwecc = monahans_df_enable_hwecc;
	this->calculate_ecc = monahans_df_calculate_ecc;
	this->correct_data = monahans_df_correct_data;
	this->block_bad = monahans_df_block_bad;
	this->block_markbad = monahans_df_block_markbad;
	this->scan_bbt = monahans_df_scan_bbt;
	this->chip_delay = 25;
	this->bbt_td = &monahans_bbt_main;
	this->bbt_md = &monahans_bbt_mirror;

	/* If the NAND flash is small block flash, only 512-byte pagesize
	 * is supported.
	 * Adjust parameters of BBT what is depended on large block nand
	 * flash or small block nand flash.
	 */
	if (dfc_context.flash_info->oob_size > 16) {
		this->autooob = &monahans_lb_nand_oob;
		this->eccmode = NAND_ECC_HW24_2048;
		this->bbt_td->offs = 2;
		this->bbt_td->veroffs = 6;
		this->bbt_md->offs = 2;
		this->bbt_md->veroffs = 6;
		this->badblockpos = NAND_LARGE_BADBLOCK_POS;
		monahans_bbt_default.offs = NAND_LARGE_BADBLOCK_POS;
		monahans_bbt_default.len = 2;
		/* when scan_bbt() is executed, bbt version can get */
		monahans_bbt_default.veroffs = 2;
	} else {
		this->autooob = &monahans_sb_nand_oob;
		this->eccmode = NAND_ECC_HW6_512;
		this->bbt_td->offs = 8;
		this->bbt_td->veroffs = 12;
		this->bbt_md->offs = 8;
		this->bbt_md->veroffs = 12;
		this->badblockpos = NAND_SMALL_BADBLOCK_POS;
		monahans_bbt_default.offs = NAND_SMALL_BADBLOCK_POS;
		monahans_bbt_default.len = 1;
		monahans_bbt_default.veroffs = 8;
	}

	info->context = &dfc_context;
	/* TODO: allocate dma buffer and channel */

	dev_set_drvdata(dev, monahans_mtd);

	if (nand_scan(monahans_mtd, 1)) {
		printk(KERN_ERR "Nand scan failed\n");
		ret = -ENXIO;
		goto free_irq;
	}

	/* There is a potential limitation that no more partition can be
	 * added between MassStorage and BBT(last block).
	 *
	 * The last 127 blocks is reserved for relocation table, they aren't
	 * statistical data of mtd size and chip size.
	 *
	 * BBT partitions contains 4 blocks. Two blocks are used to store
	 * main descriptor, the other two are used to store mirror descriptor.
	 */

	/* boot loader partition */
	obm = calc_obm_ver();
	if (obm == MHN_OBM_V3) {
		partition_info[0].size = 0x60000;
	} else {
		partition_info[0].size = 0x40000;
	}

	/* kernel partition */
	partition_info[1].offset =
	    partition_info[0].offset + partition_info[0].size;

	/* rootfs partition */
	partition_info[2].offset =
	    partition_info[1].offset + partition_info[1].size;

	/* bbt partition */
	partition_info[PART_NUM - 1].size = (monahans_bbt_main.maxblocks
					     + monahans_bbt_mirror.maxblocks)
	    << this->phys_erase_shift;
	partition_info[PART_NUM - 1].offset = this->chipsize
	    - partition_info[PART_NUM - 1].size;

	/* mass storage partition */
	partition_info[PART_NUM - 2].offset =
	    partition_info[PART_NUM - 3].offset + partition_info[PART_NUM -
								 3].size;
	partition_info[PART_NUM - 2].size =
	    this->chipsize - partition_info[PART_NUM - 2].offset -
	    partition_info[PART_NUM - 1].size;
	add_mtd_partitions(monahans_mtd, partition_info, PART_NUM);

	return 0;

      free_irq:
	free_irq(IRQ_NAND, info);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
      free_cmd_dma:
#ifndef CONFIG_MTD_NAND_PXA310_FIX
	pxa_free_dma(info->cmd_dma);
      free_data_dma:
	pxa_free_dma(info->data_dma);
#endif
      free_buf:
	dma_free_writecombine(dev, buf_len, info->data_desc,
			      info->data_desc_addr);
#else
      free_buf:
	kfree(info->data_buf);
#endif
      free_mtd:
	kfree(monahans_mtd);
      out:
	return ret;

}

static int __devexit monahans_df_remove(struct device *dev)
{
	struct mtd_info *mtd = (struct mtd_info *)dev_get_drvdata(dev);
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	unsigned int data_buf_len = dfc_context.flash_info->page_size +
	    dfc_context.flash_info->oob_size;
	unsigned int buf_len = ALIGN(2 * sizeof(struct pxa_dma_desc), 32) +
	    ALIGN(data_buf_len, 32) + ALIGN(NAND_CMD_DMA_LEN, 32);
#endif

	dev_set_drvdata(dev, NULL);

	del_mtd_device(mtd);
	del_mtd_partitions(mtd);
	free_irq(IRQ_NAND, info);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifndef CONFIG_MTD_NAND_PXA310_FIX
	pxa_free_dma(info->cmd_dma);
#endif
	pxa_free_dma(info->data_dma);
	dma_free_writecombine(dev, buf_len, info->data_desc,
			      info->data_desc_addr);
#else
	kfree(info->data_buf);
#endif
	kfree(mtd);

	return 0;
}

#ifdef CONFIG_PM
static int monahans_df_suspend(struct device *dev, u32 state,
			       u32 level)
{
	struct mtd_info *mtd = (struct mtd_info *)dev_get_drvdata(dev);
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);

	if (SUSPEND_DISABLE == level) {	/*SUSPEND_NOTIFY */
		if (info->state != STATE_READY) {
			printk(KERN_ERR "current state is %d\n", info->state);
			return -EAGAIN;
		}
		info->state = STATE_SUSPENDED;
		/*
		 * The PM code need read the mobm from NAND.
		 * So the NAND clock can't be stop here.
		 * The PM code will cover this.
		 */
		/* pxa_set_cken(CKEN_NAND, 0); */
	}
	return 0;
}

static int monahans_df_resume(struct device *dev, u32 level)
{
	struct mtd_info *mtd = (struct mtd_info *)dev_get_drvdata(dev);
	struct monahans_dfc_info *info = (struct monahans_dfc_info *)
	    (((struct nand_chip *)(mtd->priv))->priv);
	int status;

	if (RESUME_ENABLE == level) {
		if (info->state != STATE_SUSPENDED)
			printk(KERN_WARNING "Error State after resume back\n");

		info->state = STATE_READY;

		pxa_set_cken(CKEN_NAND, 1);

		status = dfc_init(&dfc_context, flash_config);
		if (status) {
			printk(KERN_ALERT "Monahans NAND device:"
			       "Nand Flash initialize failure!\n");
			return -ENXIO;
		}
	}
	return 0;
}
#endif

static struct device_driver monahans_df_driver = {
	.name = "PXA3xx-nand-flash",
	.bus = &platform_bus_type,
	.probe = monahans_df_probe,
	.remove = __devexit_p(monahans_df_remove),
#ifdef CONFIG_PM
	.suspend = monahans_df_suspend,
	.resume = monahans_df_resume,
#endif
};

static void __exit monahans_df_cleanup(void)
{
	driver_unregister(&monahans_df_driver);
}

static int __init monahans_df_init(void)
{
	return driver_register(&monahans_df_driver);
}

module_init(monahans_df_init);
module_exit(monahans_df_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jingqing.xu (jingqing.xu@intel.com)");
MODULE_DESCRIPTION("Glue logic layer for NAND flash on PXA3xx DFC");
