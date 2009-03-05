/*
 * linux/drivers/mtd/nand/nand_davinci.c
 *
 * NAND Flash Driver
 *
 * Copyright (C) 2006 Texas Instruments.
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
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   DaVinci board which utilizes the Samsung k9k2g08 part.
 *
 Modifications:
 ver. 1.0: Feb 2005, Vinod/Sudhakar
 -
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/nand_davinci.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>
#include <asm/arch/hardware.h>

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/*
 * Some NAND devices have two chip selects on the same device.  The driver
 * supports devices with either one or two chip selects.
 */
#define MAX_CHIPS 2

/*
 * Convert a physical EMIF address to the corresponding chip enable.
 *
 *	address range			chip enable
 *	-----------------------		-----------
 *	0x02000000 - 0x03FFFFFF		0
 *	0x04000000 - 0x05FFFFFF		1
 *	0x06000000 - 0x07FFFFFF		2
 *	0x08000000 - 0x09FFFFFF		3
 */
#define EMIF_ADDR_TO_CE(a) ((((a) >> 25) - 1) & 3)
#define MAX_EMIF_CHIP_ENABLES 4

struct nand_davinci_info {
	struct resource *reg_res;
	void __iomem *emifregs;
	unsigned ce;		/* emif chip enable */
	unsigned cle_mask;
	unsigned ale_mask;
	struct mtd_info *mtd;
	struct mtd_partition *parts;
	struct resource *data_res[MAX_CHIPS];
	void __iomem *ioaddr[MAX_CHIPS];
	struct clk *clk;
};

#define NAND_READ_START    0x00
#define NAND_READ_END      0x30
#define NAND_STATUS        0x70

/* EMIF Register Offsets */
#define NANDFCR			0x60
#define NANDFSR			0x64
#define NANDF1ECC		0x70
#define NANDF2ECC		0x74
#define NANDF3ECC		0x78
#define NANDF4ECC		0x7C
#define NAND4BITECCLOAD		0xBC
#define NAND4BITECC1		0xC0
#define NAND4BITECC2		0xC4
#define NAND4BITECC3		0xC8
#define NAND4BITECC4		0xCC
#define NANDERRADD1		0xD0
#define NANDERRADD2		0xD4
#define NANDERRVAL1		0xD8
#define NANDERRVAL2		0xDC
#define EMIF_REG_SIZE		0x1000

/* Definitions for 1-bit hardware ECC */
#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)

#define NAND_Ecc_P1o            (1 << 16)
#define NAND_Ecc_P2o            (1 << 17)
#define NAND_Ecc_P4o            (1 << 18)
#define NAND_Ecc_P8o            (1 << 19)
#define NAND_Ecc_P16o           (1 << 20)
#define NAND_Ecc_P32o           (1 << 21)
#define NAND_Ecc_P64o           (1 << 22)
#define NAND_Ecc_P128o          (1 << 23)
#define NAND_Ecc_P256o          (1 << 24)
#define NAND_Ecc_P512o          (1 << 25)
#define NAND_Ecc_P1024o         (1 << 26)
#define NAND_Ecc_P2048o         (1 << 27)

#define TF(value)       (value ? 1 : 0)

#define P2048e(a)       (TF(a & NAND_Ecc_P2048e)        << 0 )
#define P2048o(a)       (TF(a & NAND_Ecc_P2048o)        << 1 )
#define P1e(a)		(TF(a & NAND_Ecc_P1e)           << 2 )
#define P1o(a)		(TF(a & NAND_Ecc_P1o)           << 3 )
#define P2e(a)		(TF(a & NAND_Ecc_P2e)           << 4 )
#define P2o(a)		(TF(a & NAND_Ecc_P2o)           << 5 )
#define P4e(a)		(TF(a & NAND_Ecc_P4e)           << 6 )
#define P4o(a)		(TF(a & NAND_Ecc_P4o)           << 7 )

#define P8e(a)          (TF(a & NAND_Ecc_P8e)           << 0 )
#define P8o(a)          (TF(a & NAND_Ecc_P8o)           << 1 )
#define P16e(a)         (TF(a & NAND_Ecc_P16e)          << 2 )
#define P16o(a)         (TF(a & NAND_Ecc_P16o)          << 3 )
#define P32e(a)         (TF(a & NAND_Ecc_P32e)          << 4 )
#define P32o(a)         (TF(a & NAND_Ecc_P32o)          << 5 )
#define P64e(a)         (TF(a & NAND_Ecc_P64e)          << 6 )
#define P64o(a)         (TF(a & NAND_Ecc_P64o)          << 7 )

#define P128e(a)        (TF(a & NAND_Ecc_P128e)         << 0 )
#define P128o(a)        (TF(a & NAND_Ecc_P128o)         << 1 )
#define P256e(a)        (TF(a & NAND_Ecc_P256e)         << 2 )
#define P256o(a)        (TF(a & NAND_Ecc_P256o)         << 3 )
#define P512e(a)        (TF(a & NAND_Ecc_P512e)         << 4 )
#define P512o(a)        (TF(a & NAND_Ecc_P512o)         << 5 )
#define P1024e(a)       (TF(a & NAND_Ecc_P1024e)        << 6 )
#define P1024o(a)       (TF(a & NAND_Ecc_P1024o)        << 7 )

#define P8e_s(a)        (TF(a & NAND_Ecc_P8e)           << 0 )
#define P8o_s(a)        (TF(a & NAND_Ecc_P8o)           << 1 )
#define P16e_s(a)       (TF(a & NAND_Ecc_P16e)          << 2 )
#define P16o_s(a)       (TF(a & NAND_Ecc_P16o)          << 3 )
#define P1e_s(a)        (TF(a & NAND_Ecc_P1e)           << 4 )
#define P1o_s(a)        (TF(a & NAND_Ecc_P1o)           << 5 )
#define P2e_s(a)        (TF(a & NAND_Ecc_P2e)           << 6 )
#define P2o_s(a)        (TF(a & NAND_Ecc_P2o)           << 7 )

#define P4e_s(a)		(TF(a & NAND_Ecc_P4e)           << 0 )
#define P4o_s(a)		(TF(a & NAND_Ecc_P4o)           << 1 )

/* Definitions for 4-bit hardware ECC */
#define NAND_4BITECC_MASK		0x03FF03FF
#define EMIF_NANDFSR_ECC_STATE_MASK  	0x00000F00
#define ECC_STATE_NO_ERR		0x0
#define ECC_STATE_TOO_MANY_ERRS		0x1
#define ECC_STATE_ERR_CORR_COMP_P	0x2
#define ECC_STATE_ERR_CORR_COMP_N	0x3
#define ECC_MAX_CORRECTABLE_ERRORS	0x4

/*
 * nand_davinci_select_chip
 * Select a chip in a multi-chip device
 */
static void nand_davinci_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;

	switch (chip) {
	case -1:
		/* deselect all chips */
		break;
	case 0:
	case 1:
		this->IO_ADDR_R = this->IO_ADDR_W = info->ioaddr[chip];
		break;
	default:
		BUG();
	}
}

/*
 *      hardware specific access to control-lines
 */
static void nand_davinci_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;
	u32 IO_ADDR_W = (u32) this->IO_ADDR_W;

	IO_ADDR_W &= ~(info->ale_mask | info->cle_mask);
	switch (cmd) {
	case NAND_CTL_SETCLE:
		IO_ADDR_W |= info->cle_mask;
		break;
	case NAND_CTL_SETALE:
		IO_ADDR_W |= info->ale_mask;
		break;
	}
	this->IO_ADDR_W = (void __iomem *)IO_ADDR_W;
}

/*
 * 1-bit ECC routines
 */

static void nand_davinci_1bit_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;
	void __iomem *nandfcr = info->emifregs + NANDFCR;

	switch (mode) {
	case NAND_ECC_WRITE:
	case NAND_ECC_READ:
		__raw_writel(__raw_readl(nandfcr) | (1 << (8 + info->ce)),
			     nandfcr);
		break;
	default:
		break;
	}
}

/*
 * Read the NAND ECC register corresponding to chip enable ce, where 0<=ce<=3.
 */
static u32 nand_davinci_1bit_readecc(struct mtd_info *mtd, u32 ce)
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;

	return __raw_readl(info->emifregs + NANDF1ECC + 4 * ce);
}

static int nand_davinci_1bit_calculate_ecc(struct mtd_info *mtd,
					   const u_char * dat,
					   u_char * ecc_code)
{
	unsigned int l;
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;

	l = nand_davinci_1bit_readecc(mtd, info->ce);
	*ecc_code++ = l;	// P128e, ..., P1e
	*ecc_code++ = l >> 16;	// P128o, ..., P1o
	// P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e
	*ecc_code++ = ((l >> 8) & 0x0f) | ((l >> 20) & 0xf0);

	return 0;
}

static void nand_davinci_1bit_gen_true_ecc(u8 * ecc_buf)
{
	u32 tmp =
	    ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) |
	    ((ecc_buf[2] & 0x0F) << 8);

	ecc_buf[0] =
	    ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) | P16o(tmp) |
	      P16e(tmp) | P8o(tmp) | P8e(tmp));
	ecc_buf[1] =
	    ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) | P256o(tmp) |
	      P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[2] =
	    ~(P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) | P1e(tmp) |
	      P2048o(tmp) | P2048e(tmp));
}

static int nand_davinci_1bit_compare_ecc(u8 * ecc_data1,	/* read from NAND memory */
					 u8 * ecc_data2,	/* read from register */
					 u8 * page_data)
{
	u32 i;
	u8 tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
	u8 comp0_bit[8], comp1_bit[8], comp2_bit[8];
	u8 ecc_bit[24];
	u8 ecc_sum = 0;
	u8 find_bit = 0;
	u32 find_byte = 0;
	int isEccFF;

	isEccFF = ((*(u32 *) ecc_data1 & 0xFFFFFF) == 0xFFFFFF);

	nand_davinci_1bit_gen_true_ecc(ecc_data1);
	nand_davinci_1bit_gen_true_ecc(ecc_data2);

	for (i = 0; i <= 2; i++) {
		*(ecc_data1 + i) = ~(*(ecc_data1 + i));
		*(ecc_data2 + i) = ~(*(ecc_data2 + i));
	}

	for (i = 0; i < 8; i++) {
		tmp0_bit[i] = *ecc_data1 % 2;
		*ecc_data1 = *ecc_data1 / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp1_bit[i] = *(ecc_data1 + 1) % 2;
		*(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp2_bit[i] = *(ecc_data1 + 2) % 2;
		*(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp0_bit[i] = *ecc_data2 % 2;
		*ecc_data2 = *ecc_data2 / 2;
	}

	for (i = 0; i < 8; i++) {
		comp1_bit[i] = *(ecc_data2 + 1) % 2;
		*(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp2_bit[i] = *(ecc_data2 + 2) % 2;
		*(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
	}

	for (i = 0; i < 6; i++)
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

	for (i = 0; i < 24; i++)
		ecc_sum += ecc_bit[i];

	switch (ecc_sum) {
	case 0:
		/* Not reached because this function is not called if
		   ECC values are equal */
		return 0;

	case 1:
		/*
		 * This case corresponds to a 1-bit error in the ECC code
		 * itself.  We'll return 1 to indicate that a 1-bit error was
		 * detected and corrected, but there is no need to correct
		 * anything.
		 */
		DEBUG(MTD_DEBUG_LEVEL0, "Detected single-bit error in ECC\n");
		return 1;

	case 12:
		/* Correctable error */
		find_byte = (ecc_bit[23] << 8) +
		    (ecc_bit[21] << 7) +
		    (ecc_bit[19] << 6) +
		    (ecc_bit[17] << 5) +
		    (ecc_bit[15] << 4) +
		    (ecc_bit[13] << 3) +
		    (ecc_bit[11] << 2) + (ecc_bit[9] << 1) + ecc_bit[7];

		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

		DEBUG(MTD_DEBUG_LEVEL0,
		      "Correcting single bit ECC error at offset: %d, bit: %d\n",
		      find_byte, find_bit);

		page_data[find_byte] ^= (1 << find_bit);

		return 1;

	default:
		if (isEccFF) {
			if (ecc_data2[0] == 0 && ecc_data2[1] == 0
			    && ecc_data2[2] == 0)
				return 0;
		}
		DEBUG(MTD_DEBUG_LEVEL0, "UNCORRECTED_ERROR default\n");
		return -1;
	}
}

static int nand_davinci_1bit_correct_data(struct mtd_info *mtd, u_char * dat,
					  u_char * read_ecc, u_char * calc_ecc)
{
	int r = 0;

	if (memcmp(read_ecc, calc_ecc, 3) != 0) {
		u_char read_ecc_copy[3], calc_ecc_copy[3];
		int i;

		for (i = 0; i < 3; i++) {
			read_ecc_copy[i] = read_ecc[i];
			calc_ecc_copy[i] = calc_ecc[i];
		}
		r = nand_davinci_1bit_compare_ecc(read_ecc_copy, calc_ecc_copy,
						  dat);
	}

	return r;
}

/*
 * 4-bit ECC routines
 */

/*
 * Instead of placing the spare data at the end of the page, the 4-bit ECC
 * hardware generator requires that the page be subdivided into 4 subpages,
 * each with its own spare data area.  This structure defines the format of
 * each of these subpages.
 */
static struct page_layout_item nand_davinci_hw10_512_layout[] = {
	{.type = ITEM_TYPE_DATA,.length = 512},
	{.type = ITEM_TYPE_OOB,.length = 6,},
	{.type = ITEM_TYPE_ECC,.length = 10,},
	{.type = 0,.length = 0,},
};

static struct nand_oobinfo nand_davinci_hw10_512_oobinfo = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	/*
	 * We actually have 40 bytes of ECC per page, but the nand_oobinfo
	 * structure definition limits us to a maximum of 32 bytes.  This
	 * doesn't matter, because out page_layout_item structure definition
	 * determines where our ECC actually goes in the flash page.
	 */
	.eccbytes = 32,
	.eccpos = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		   22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
		   38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		   54, 55,
		   },
	.oobfree = {{0, 6}, {16, 6}, {32, 6}, {48, 6}},
};

/*
 * We should always have a flash-based bad block table.  However, if one isn't
 * found then all blocks will be scanned to look for factory-marked bad blocks.
 * We supply a null pattern so that no blocks will be detected as bad.
 */
static struct nand_bbt_descr nand_davinci_hw10_512_badblock_pattern = {
	.options = 0,
	.offs = 0,
	.len = 0,
	.pattern = NULL,
};

/*
 * When using 4-bit ECC with a 2048-byte data + 64-byte spare page size, the
 * oob is scattered throughout the page in 4 16-byte chunks instead of being
 * grouped together at the end of the page.  This means that the factory
 * bad-block markers at offsets 2048 and 2049 will be overwritten when data
 * is written to the flash.  Thus, we cannot use the factory method to mark
 * or detect bad blocks and must rely on a flash-based bad block table instead.
 *
 */
static int nand_davinci_hw10_512_block_bad(struct mtd_info *mtd, loff_t ofs,
					   int getchip)
{
	return 0;
}

static int nand_davinci_hw10_512_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *this = mtd->priv;
	int block;

	/* Get block number */
	block = ((int)ofs) >> this->bbt_erase_shift;
	if (this->bbt)
		this->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (this->options & NAND_USE_FLASH_BBT)
		return nand_update_bbt(mtd, ofs);

	return 0;
}

static void nand_davinci_4bit_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;
	void __iomem *nandfcr;
	u32 val;

	switch (mode) {
	case NAND_ECC_WRITE:
	case NAND_ECC_READ:
		/*
		 * Start a new ECC calculation for reading or writing 512 bytes
		 *  of data.
		 */
		nandfcr = info->emifregs + NANDFCR;
		val = (__raw_readl(nandfcr) & ~(3 << 4))
		    | (info->ce << 4) | (1 << 12);
		__raw_writel(val, nandfcr);
		break;
	case NAND_ECC_WRITEOOB:
	case NAND_ECC_READOOB:
		/*
		 * Terminate ECC calculation by performing a dummy read of an
		 * ECC register.  Our hardware ECC generator supports including
		 * the OOB in the ECC calculation, but the NAND core code
		 * doesn't really support that.  We will only calculate the ECC
		 * on the data; errors in the non-ECC bytes in the OOB will not
		 * be detected or corrected.
		 */
		__raw_readl(info->emifregs + NAND4BITECC1);
		break;
	case NAND_ECC_WRITESYN:
	case NAND_ECC_READSYN:
		/*
		 * Our ECC calculation has already been terminated, so no need
		 * to do anything here.
		 */
		break;
	default:
		break;
	}
}

static u32 nand_davinci_4bit_readecc(struct mtd_info *mtd, unsigned int ecc[4])
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;

	ecc[0] = __raw_readl(info->emifregs + NAND4BITECC1) & NAND_4BITECC_MASK;
	ecc[1] = __raw_readl(info->emifregs + NAND4BITECC2) & NAND_4BITECC_MASK;
	ecc[2] = __raw_readl(info->emifregs + NAND4BITECC3) & NAND_4BITECC_MASK;
	ecc[3] = __raw_readl(info->emifregs + NAND4BITECC4) & NAND_4BITECC_MASK;

	return 0;
}

static int nand_davinci_4bit_calculate_ecc(struct mtd_info *mtd,
					   const u_char * dat,
					   u_char * ecc_code)
{
	unsigned int hw_4ecc[4] = { 0, 0, 0, 0 };
	unsigned int const1 = 0, const2 = 0;
	unsigned char count1 = 0;

	/*
	 * Since the NAND_HWECC_SYNDROME option is enabled, this routine is
	 * only called just after the data and oob have been written.  The
	 * ECC value calculated by the hardware ECC generator is available
	 * for us to read.
	 */
	nand_davinci_4bit_readecc(mtd, hw_4ecc);

	/*Convert 10 bit ecc value to 8 bit */
	for (count1 = 0; count1 < 2; count1++) {
		const2 = count1 * 5;
		const1 = count1 * 2;

		/* Take first 8 bits from val1 (count1=0) or val5 (count1=1) */
		ecc_code[const2] = hw_4ecc[const1] & 0xFF;

		/*
		 * Take 2 bits as LSB bits from val1 (count1=0) or val5
		 * (count1=1) and 6 bits from val2 (count1=0) or val5 (count1=1)
		 */
		ecc_code[const2 + 1] =
		    ((hw_4ecc[const1] >> 8) & 0x3) | ((hw_4ecc[const1] >> 14) &
						      0xFC);

		/*
		 * Take 4 bits from val2 (count1=0) or val5 (count1=1) and
		 * 4 bits from val3 (count1=0) or val6 (count1=1)
		 */
		ecc_code[const2 + 2] =
		    ((hw_4ecc[const1] >> 22) & 0xF) |
		    ((hw_4ecc[const1 + 1] << 4) & 0xF0);

		/*
		 * Take 6 bits from val3(count1=0) or val6 (count1=1) and
		 * 2 bits from val4 (count1=0) or  val7 (count1=1)
		 */
		ecc_code[const2 + 3] =
		    ((hw_4ecc[const1 + 1] >> 4) & 0x3F) |
		    ((hw_4ecc[const1 + 1] >> 10) & 0xC0);

		/* Take 8 bits from val4 (count1=0) or val7 (count1=1) */
		ecc_code[const2 + 4] = (hw_4ecc[const1 + 1] >> 18) & 0xFF;
	}

	return 0;
}

static int nand_davinci_4bit_compare_ecc(struct mtd_info *mtd, u8 * read_ecc,	/* read from NAND */
					 u8 * page_data)
{
	struct nand_chip *this = mtd->priv;
	struct nand_davinci_info *info = this->priv;
	unsigned short ecc_10bit[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	int i;
	unsigned int hw_4ecc[4] = { 0, 0, 0, 0 }, iserror = 0;
	unsigned short *pspare = NULL, *pspare1 = NULL;
	unsigned int numErrors, errorAddress, errorValue;

	/*
	 * Check for an ECC where all bytes are 0xFF.  If this is the case, we
	 * will assume we are looking at an erased page and we should ignore the
	 * ECC.
	 */
	for (i = 0; i < 10; i++) {
		if (read_ecc[i] != 0xFF)
			break;
	}
	if (i == 10)
		return 0;

	/* Convert 8 bit in to 10 bit */
	pspare = (unsigned short *)&read_ecc[2];
	pspare1 = (unsigned short *)&read_ecc[0];
	/* Take 10 bits from 0th and 1st bytes */
	ecc_10bit[0] = (*pspare1) & 0x3FF;	/* 10 */
	/* Take 6 bits from 1st byte and 4 bits from 2nd byte */
	ecc_10bit[1] = (((*pspare1) >> 10) & 0x3F)
	    | (((pspare[0]) << 6) & 0x3C0);	/* 6 + 4 */
	/* Take 4 bits form 2nd bytes and 6 bits from 3rd bytes */
	ecc_10bit[2] = ((pspare[0]) >> 4) & 0x3FF;	/* 10 */
	/*Take 2 bits from 3rd byte and 8 bits from 4th byte */
	ecc_10bit[3] = (((pspare[0]) >> 14) & 0x3)
	    | ((((pspare[1])) << 2) & 0x3FC);	/* 2 + 8 */
	/* Take 8 bits from 5th byte and 2 bits from 6th byte */
	ecc_10bit[4] = ((pspare[1]) >> 8)
	    | ((((pspare[2])) << 8) & 0x300);	/* 8 + 2 */
	/* Take 6 bits from 6th byte and 4 bits from 7th byte */
	ecc_10bit[5] = (pspare[2] >> 2) & 0x3FF;	/* 10 */
	/* Take 4 bits from 7th byte and 6 bits from 8th byte */
	ecc_10bit[6] = (((pspare[2]) >> 12) & 0xF)
	    | ((((pspare[3])) << 4) & 0x3F0);	/* 4 + 6 */
	/*Take 2 bits from 8th byte and 8 bits from 9th byte */
	ecc_10bit[7] = ((pspare[3]) >> 6) & 0x3FF;	/* 10 */

	/*
	 * Write the parity values in the NAND Flash 4-bit ECC Load register.
	 * Write each parity value one at a time starting from 4bit_ecc_val8
	 * to 4bit_ecc_val1.
	 */
	for (i = 7; i >= 0; i--)
		__raw_writel(ecc_10bit[i], info->emifregs + NAND4BITECCLOAD);

	/*
	 * Perform a dummy read to the EMIF Revision Code and Status register.
	 * This is required to ensure time for syndrome calculation after
	 * writing the ECC values in previous step.
	 */
	__raw_readl(info->emifregs + NANDFSR);

	/*
	 * Read the syndrome from the NAND Flash 4-Bit ECC 1-4 registers.
	 * A syndrome value of 0 means no bit errors. If the syndrome is
	 * non-zero then go further otherwise return.
	 */
	nand_davinci_4bit_readecc(mtd, hw_4ecc);

	if (hw_4ecc[0] == ECC_STATE_NO_ERR && hw_4ecc[1] == ECC_STATE_NO_ERR &&
	    hw_4ecc[2] == ECC_STATE_NO_ERR && hw_4ecc[3] == ECC_STATE_NO_ERR)
		return 0;

	/*
	 * Clear any previous address calculation by doing a dummy read of an
	 * error address register.
	 */
	__raw_readl(info->emifregs + NANDERRADD1);

	/*
	 * Set the addr_calc_st bit(bit no 13) in the NAND Flash Control
	 * register to 1.
	 */
	__raw_writel(__raw_readl(info->emifregs + NANDFCR) | (1 << 13),
		     info->emifregs + NANDFCR);

	/*
	 * Wait for the corr_state field (bits 8 to 11)in the
	 * NAND Flash Status register to be equal to 0x0, 0x1, 0x2, or 0x3.
	 */
	do {
		iserror = __raw_readl(info->emifregs + NANDFSR);
		iserror &= EMIF_NANDFSR_ECC_STATE_MASK;
		iserror = iserror >> 8;
	} while ((ECC_STATE_NO_ERR != iserror) &&
		 (ECC_STATE_TOO_MANY_ERRS != iserror) &&
		 (ECC_STATE_ERR_CORR_COMP_P != iserror) &&
		 (ECC_STATE_ERR_CORR_COMP_N != iserror));

	/*
	 * ECC_STATE_TOO_MANY_ERRS (0x1) means errors cannot be
	 * corrected (five or more errors).  The number of errors
	 * calculated (err_num field) differs from the number of errors
	 * searched.  ECC_STATE_ERR_CORR_COMP_P (0x2) means error
	 * correction complete (errors on bit 8 or 9).
	 * ECC_STATE_ERR_CORR_COMP_N (0x3) means error correction
	 * complete (error exists).
	 */

	if (iserror == ECC_STATE_NO_ERR)
		return 0;
	else if (iserror == ECC_STATE_TOO_MANY_ERRS)
		return -1;

	numErrors = ((__raw_readl(info->emifregs + NANDFSR) >> 16) & 0x3) + 1;

	/* Read the error address, error value and correct */
	for (i = 0; i < numErrors; i++) {
		if (i > 1) {
			errorAddress =
			    ((__raw_readl(info->emifregs + NANDERRADD2) >>
			      (16 * (i & 1))) & 0x3FF);
			errorAddress = ((512 + 7) - errorAddress);
			errorValue =
			    ((__raw_readl(info->emifregs + NANDERRVAL2) >>
			      (16 * (i & 1))) & 0xFF);
		} else {
			errorAddress =
			    ((__raw_readl(info->emifregs + NANDERRADD1) >>
			      (16 * (i & 1))) & 0x3FF);
			errorAddress = ((512 + 7) - errorAddress);
			errorValue =
			    ((__raw_readl(info->emifregs + NANDERRVAL1) >>
			      (16 * (i & 1))) & 0xFF);
		}
		/* xor the corrupt data with error value */
		if (errorAddress < 512)
			page_data[errorAddress] ^= errorValue;
	}

	return numErrors;
}

static int nand_davinci_4bit_correct_data(struct mtd_info *mtd, u_char * dat,
					  u_char * read_ecc, u_char * calc_ecc)
{
	int r;

	/*
	 * dat points to 512 bytes of data.  read_ecc points to the start of the
	 * oob area for this subpage, so the ecc values start at offset 6.
	 * The calc_ecc pointer is not needed since our caclulated ECC is
	 * already latched in the hardware ECC generator.
	 */
	r = nand_davinci_4bit_compare_ecc(mtd, read_ecc + 6, dat);

	return r;
}

static int nand_flash_init(struct nand_davinci_info *info)
{
	__raw_writel((1 << info->ce), info->emifregs + NANDFCR);

	return 0;
}

#define res_size(_r) (((_r)->end - (_r)->start) + 1)

static int __devinit nand_davinci_probe(struct device *dev)
{
	int err = 0, cs;
	struct nand_davinci_info *info;
	struct platform_device *pdev = to_platform_device(dev);
	struct nand_davinci_platform_data *pdata = pdev->dev.platform_data;
	struct nand_chip *this;
	struct resource *res;
	u32 rev_code;

	info = kzalloc(sizeof(struct nand_davinci_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || (res_size(res) < EMIF_REG_SIZE)) {
		dev_err(dev, "insufficient resources\n");
		err = -ENOENT;
		goto out_free_info;
	}

	/*
	 * We exclude the EMIF registers prior to NANDFCR (the chip select
	 * timing registers) from our resource reservation request because we
	 * don't use them and another module might need them.
	 */
	info->reg_res = request_mem_region(res->start + NANDFCR,
					   res_size(res) - NANDFCR, pdev->name);
	if (!info->reg_res) {
		dev_err(dev, "cannot claim register memory region\n");
		err = -EIO;
		goto out_free_info;
	}
	info->emifregs = (void __iomem *)IO_ADDRESS(res->start);

	for (cs = 0; cs < MAX_CHIPS; cs++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, cs + 1);
		if (!res)
			break;

		if (cs == 0)
			info->ce = EMIF_ADDR_TO_CE(res->start);
		else {
			if (info->ce != EMIF_ADDR_TO_CE(res->start)) {
				dev_err(dev,
					"bad secondary nand address 0x%08lx\n",
					res->start);
				err = -EIO;
				goto out_release_mem;
			}
		}

		info->data_res[cs] = request_mem_region(res->start,
							res_size(res),
							pdev->name);
		if (!info->data_res[cs]) {
			dev_err(dev,
				"cannot claim nand memory region at 0x%08lx\n",
				res->start);
			err = -EIO;
			goto out_release_mem;
		}

		info->ioaddr[cs] = ioremap_nocache(res->start, res_size(res));
		if (!info->ioaddr[cs]) {
			dev_err(dev,
				"cannot ioremap nand memory region at 0x%08lx\n",
				res->start);
			err = -ENOMEM;
			goto out_release_mem;
		}
	}
	if (!info->data_res[0]) {
		dev_err(dev, "insufficient resources\n");
		err = -ENOENT;
		goto out_release_mem;
	}

	/* Allocate memory for the MTD device structure */
	info->mtd = kzalloc(sizeof(struct mtd_info) +
			    sizeof(struct nand_chip), GFP_KERNEL);
	if (!info->mtd) {
		err = -ENOMEM;
		goto out_release_mem;
	}

	/* Get pointer to the nand private data */
	this = (struct nand_chip *)(&info->mtd[1]);
	/* Link the nand private data with the MTD structure */
	info->mtd->priv = this;
	/* Link our driver private data with the nand private data */
	this->priv = info;

	this->select_chip = nand_davinci_select_chip;
	this->hwcontrol = nand_davinci_hwcontrol;
	this->options = pdata->options;
	this->eccmode = pdata->eccmode;
	this->bbt_td = pdata->bbt_td;
	this->bbt_md = pdata->bbt_md;

	info->cle_mask = pdata->cle_mask;
	info->ale_mask = pdata->ale_mask;

	switch (this->eccmode) {
	case NAND_ECC_NONE:
		dev_warn(dev, "Warning: NAND ECC is disabled\n");
		break;
	case NAND_ECC_SOFT:
		dev_info(dev, "Using soft ECC\n");
		break;
	case NAND_ECC_HW3_512:
		dev_info(dev, "Using 1-bit hardware ECC\n");
		this->calculate_ecc = nand_davinci_1bit_calculate_ecc;
		this->correct_data = nand_davinci_1bit_correct_data;
		this->enable_hwecc = nand_davinci_1bit_enable_hwecc;
		break;
	case NAND_ECC_HW10_512:
		dev_info(dev, "Using 4-bit hardware ECC\n");
		this->options |= NAND_USE_FLASH_BBT | NAND_HWECC_SYNDROME;
		this->autooob = &nand_davinci_hw10_512_oobinfo;
		this->layout = nand_davinci_hw10_512_layout;
		this->calculate_ecc = nand_davinci_4bit_calculate_ecc;
		this->correct_data = nand_davinci_4bit_correct_data;
		this->enable_hwecc = nand_davinci_4bit_enable_hwecc;
		this->block_bad = nand_davinci_hw10_512_block_bad;
		this->block_markbad = nand_davinci_hw10_512_block_markbad;
		this->badblock_pattern =
		    &nand_davinci_hw10_512_badblock_pattern;
		break;
	default:
		dev_err(dev, "Unsupported ECC mode %d requested\n",
			this->eccmode);
		goto out_release_mem;
	}

	info->mtd->name = pdev->dev.bus_id;
	info->mtd->owner = THIS_MODULE;

	info->clk = clk_get(dev, "AEMIFCLK");
	if (IS_ERR(info->clk)) {
		err = -ENXIO;
		goto out_free_mtd;
	}
	clk_use(info->clk);
	clk_enable(info->clk);

	nand_flash_init(info);

	/* Scan for the device */
	if (nand_scan(info->mtd, info->data_res[1] ? 2 : 1)) {
		dev_err(dev, "no nand device detected\n");
		err = -ENODEV;
		goto out_unuse_clk;
	}

	/* Terminate any ECC calculation already in progress */
	switch (this->eccmode) {
	case NAND_ECC_HW3_512:
		nand_davinci_1bit_enable_hwecc(info->mtd, NAND_ECC_WRITE);
		nand_davinci_1bit_readecc(info->mtd, info->ce);
		break;
	case NAND_ECC_HW10_512:
		{
			unsigned int ecc[4];

			nand_davinci_4bit_enable_hwecc(info->mtd,
						       NAND_ECC_WRITE);
			nand_davinci_4bit_readecc(info->mtd, ecc);
		}
	default:
		break;
	}

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(info->mtd, info->parts, err);
	else if (err < 0 && pdata->parts)
		add_mtd_partitions(info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		add_mtd_device(info->mtd);

	dev_set_drvdata(dev, info);

	/* show rev code */
	rev_code = __raw_readl(info->emifregs);
	dev_info(dev, "hardware revision: %d.%d\n",
		 (rev_code >> 8) & 0xff, rev_code & 0xff);

	return 0;

      out_unuse_clk:
	clk_unuse(info->clk);
	clk_disable(info->clk);
      out_free_mtd:
	kfree(info->mtd);
      out_release_mem:
	for (cs = 0; cs < MAX_CHIPS; cs++) {
		if (info->ioaddr[cs])
			iounmap(info->ioaddr[cs]);
		if (info->data_res[cs]) {
			release_resource(info->data_res[cs]);
			kfree(info->data_res[cs]);
		}
	}
	release_resource(info->reg_res);
	kfree(info->reg_res);
      out_free_info:
	kfree(info);
      out:
	return err;
}

static int __devexit nand_davinci_remove(struct device *dev)
{
	struct nand_davinci_info *info = dev_get_drvdata(dev);
	int cs;

	if (info) {
		/* Release NAND device, internal structures, and partitions */
		nand_release(info->mtd);

		clk_unuse(info->clk);
		clk_disable(info->clk);

		kfree(info->mtd);

		for (cs = 0; cs < MAX_CHIPS; cs++) {
			if (info->ioaddr[cs])
				iounmap(info->ioaddr[cs]);
			if (info->data_res[cs]) {
				release_resource(info->data_res[cs]);
				kfree(info->data_res[cs]);
			}
		}

		release_resource(info->reg_res);
		kfree(info->reg_res);

		kfree(info);

		dev_set_drvdata(dev, NULL);
	}

	return 0;
}

static struct device_driver nand_davinci_driver = {
	.name = "nand_davinci",
	.bus = &platform_bus_type,
	.probe = nand_davinci_probe,
	.remove = __devexit_p(nand_davinci_remove),
};

static int __init nand_davinci_init(void)
{
	return driver_register(&nand_davinci_driver);
}

static void __exit nand_davinci_exit(void)
{
	driver_unregister(&nand_davinci_driver);
}

module_init(nand_davinci_init);

/* FIXME: rmmod of module with more than 1 partition oopses 
 *        (c.f MV Bugz 21264) */
/* module_exit(nand_davinci_exit); */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Board-specific driver for NAND flash on davinci board");
