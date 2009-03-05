/*
 * Header file for DaVinci NAND platform data.
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef DAVINCI_NAND_PLAT_H
#define DAVINCI_NAND_PLAT_H

#include <linux/mtd/nand.h>

/**
 * struct nand_davinci_platform_data - platform data describing NAND flash banks
 * @options:	bitmask for nand_chip.options
 * @eccmode:	ECC mode for nand_chip eccmode
 * @cle_mask:	bitmask with address bit to set to activate CLE
 *		(command latch enable)
 * @ale_mask:	bitmask with address bit to set to activate ALE
 *		(address latch enable)
 * @bbt_td:	pointer to nand_bbt_descr for primary bad block table
 * @bbt_md:	pointer to nand_bbt_descr for mirror bad block table
 * @parts:	optional array of mtd_partitions for static partitioning
 * @nr_parts:	number of mtd_partitions for static partitioning
 * @
 */
struct nand_davinci_platform_data {
	unsigned int options;
	int eccmode;
	unsigned int cle_mask;
	unsigned int ale_mask;
	struct nand_bbt_descr *bbt_td;
	struct nand_bbt_descr *bbt_md;
	struct mtd_partition *parts;
	unsigned int nr_parts;
};

#endif	/* ifndef DAVINCI_NAND_PLAT_H */
