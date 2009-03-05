/*
 * arch/ppc/qe_io/ucc/ucc.c
 *
 * QE UCC API Set - UCC specific routines implementations.
 *
 * Author: Shlomi Gridish <gridish@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/stddef.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/immap_qe.h>
#include <asm/qe.h>

#include "ucc.h"

int ucc_set_qe_mux_mii_mng(unsigned int ucc_num)
{
	unsigned long flags;

	local_irq_save(flags);
	out_be32(&qe_immr->qmx.cmxgcr,
		 ((in_be32(&qe_immr->qmx.cmxgcr) &
		   ~QE_CMXGCR_MII_ENET_MNG) |
		  (ucc_num << QE_CMXGCR_MII_ENET_MNG_SHIFT)));
	local_irq_restore(flags);

	return 0;
}

int ucc_set_type(unsigned int ucc_num, struct ucc_common *regs,
		 enum ucc_speed_type speed)
{
	u8 guemr = 0;

	/* check if the UCC number is in range. */
	if (ucc_num > UCC_MAX_NUM)
		return -EINVAL;

	guemr = regs->guemr;
	guemr &= ~(UCC_GUEMR_MODE_MASK_RX | UCC_GUEMR_MODE_MASK_TX);
	switch (speed) {
	case UCC_SPEED_TYPE_SLOW:
		guemr |= (UCC_GUEMR_MODE_SLOW_RX | UCC_GUEMR_MODE_SLOW_TX);
		break;
	case UCC_SPEED_TYPE_FAST:
		guemr |= (UCC_GUEMR_MODE_FAST_RX | UCC_GUEMR_MODE_FAST_TX);
		break;
	default:
		return -EINVAL;
	}
	regs->guemr = guemr;

	return 0;
}

int ucc_init_guemr(struct ucc_common *regs)
{
	u8 guemr = 0;

	if (!regs)
		return -EINVAL;

	/* Set bit 3 (which is reserved in the GUEMR register) to 1 */
	guemr = UCC_GUEMR_SET_RESERVED3;

	regs->guemr = guemr;

	return 0;
}

static void get_cmxucr_reg(unsigned int ucc_num, u8 * reg_num, u8 * shift)
{
	struct reg_shift {
		u32 reg;
		u32 shift;
	};

	static struct reg_shift rs[] = {
		{0, 16},
		{2, 16},
		{0, 0},
		{2, 0},
		{1, 16},
		{3, 16},
		{1, 0},
		{3, 0}
	};

	*reg_num = rs[ucc_num].reg;
	*shift = rs[ucc_num].shift;
}

int ucc_mux_set_grant_tsa_bkpt(unsigned int ucc_num, int set, u32 mask)
{
	volatile u32 *p_cmxucr;
	u8 reg_num;
	u8 shift;

	/* check if the UCC number is in range. */
	if (ucc_num > UCC_MAX_NUM)
		return -EINVAL;

	get_cmxucr_reg(ucc_num, &reg_num, &shift);

	p_cmxucr = &qe_immr->qmx.cmxucrx[reg_num];

	if (set)
		out_be32(p_cmxucr, in_be32(p_cmxucr) | (mask << shift));
	else
		out_be32(p_cmxucr, in_be32(p_cmxucr) & ~(mask << shift));

	return 0;
}

int ucc_set_qe_mux_rxtx(unsigned int ucc_num, qe_clock_e clock, comm_dir_e mode)
{
	volatile u32 *p_cmxucr;
	u8 reg_num;
	u8 shift;
	u32 clockBits;
	u32 clockMask;
	int source = -1;

	/* check if the UCC number is in range. */
	if (ucc_num > UCC_MAX_NUM)
		return -EINVAL;

	if (!((mode == COMM_DIR_RX) || (mode == COMM_DIR_TX))) {
		printk(KERN_ERR
		       "ucc_set_qe_mux_rxtx: bad comm mode type passed.");
		return -EINVAL;
	}

	get_cmxucr_reg(ucc_num, &reg_num, &shift);

	p_cmxucr = &qe_immr->qmx.cmxucrx[reg_num];

	switch (reg_num) {
	case (0):
		switch (clock) {
		case (QE_BRG1):
			source = 1;
			break;
		case (QE_BRG2):
			source = 2;
			break;
		case (QE_BRG7):
			source = 3;
			break;
		case (QE_BRG8):
			source = 4;
			break;
		case (QE_CLK9):
			source = 5;
			break;
		case (QE_CLK10):
			source = 6;
			break;
		case (QE_CLK11):
			source = 7;
			break;
		case (QE_CLK12):
			source = 8;
			break;
		case (QE_CLK15):
			source = 9;
			break;
		case (QE_CLK16):
			source = 10;
			break;
		default:
			source = -1;
			break;
		}
		break;
	case (1):
		switch (clock) {
		case (QE_BRG5):
			source = 1;
			break;
		case (QE_BRG6):
			source = 2;
			break;
		case (QE_BRG7):
			source = 3;
			break;
		case (QE_BRG8):
			source = 4;
			break;
		case (QE_CLK13):
			source = 5;
			break;
		case (QE_CLK14):
			source = 6;
			break;
		case (QE_CLK19):
			source = 7;
			break;
		case (QE_CLK20):
			source = 8;
			break;
		case (QE_CLK15):
			source = 9;
			break;
		case (QE_CLK16):
			source = 10;
			break;
		default:
			source = -1;
			break;
		}
		break;
	case (2):
		switch (clock) {
		case (QE_BRG9):
			source = 1;
			break;
		case (QE_BRG10):
			source = 2;
			break;
		case (QE_BRG15):
			source = 3;
			break;
		case (QE_BRG16):
			source = 4;
			break;
		case (QE_CLK3):
			source = 5;
			break;
		case (QE_CLK4):
			source = 6;
			break;
		case (QE_CLK17):
			source = 7;
			break;
		case (QE_CLK18):
			source = 8;
			break;
		case (QE_CLK7):
			source = 9;
			break;
		case (QE_CLK8):
			source = 10;
			break;
		default:
			source = -1;
			break;
		}
		break;
	case (3):
		switch (clock) {
		case (QE_BRG13):
			source = 1;
			break;
		case (QE_BRG14):
			source = 2;
			break;
		case (QE_BRG15):
			source = 3;
			break;
		case (QE_BRG16):
			source = 4;
			break;
		case (QE_CLK5):
			source = 5;
			break;
		case (QE_CLK6):
			source = 6;
			break;
		case (QE_CLK21):
			source = 7;
			break;
		case (QE_CLK22):
			source = 8;
			break;
		case (QE_CLK7):
			source = 9;
			break;
		case (QE_CLK8):
			source = 10;
			break;
		default:
			source = -1;
			break;
		}
		break;
	default:
		source = -1;
		break;
	}

	if (source == -1) {
		printk(KERN_ERR
		       "ucc_set_qe_mux_rxtx: Bad combination of clock and UCC.");
		return -ENOENT;
	}

	clockBits = (u32) source;
	clockMask = QE_CMXUCR_TX_CLK_SRC_MASK;
	if (mode == COMM_DIR_RX) {
		clockBits <<= 4;	/* Rx field is 4 bits to left of Tx field */
		clockMask <<= 4;	/* Rx field is 4 bits to left of Tx field */
	}
	clockBits <<= shift;
	clockMask <<= shift;

	out_be32(p_cmxucr, (in_be32(p_cmxucr) & ~clockMask) | clockBits);

	return 0;
}
