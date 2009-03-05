/*
 * arch/ppc/qe_io/ucc/ucc_slow.c
 *
 * QE UCC Slow API Set - UCC Slow specific routines implementations.
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
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/immap_qe.h>
#include <asm/qe.h>

#include "ucc.h"
#include "ucc_slow.h"

#define uccs_printk(level, format, arg...)  \
        printk(level format "\n", ## arg)

#define uccs_dbg(format, arg...)            \
        uccs_printk(KERN_DEBUG , format , ## arg)
#define uccs_err(format, arg...)            \
        uccs_printk(KERN_ERR , format , ## arg)
#define uccs_info(format, arg...)           \
        uccs_printk(KERN_INFO , format , ## arg)
#define uccs_warn(format, arg...)           \
        uccs_printk(KERN_WARNING , format , ## arg)

#ifdef UCCS_VERBOSE_DEBUG
#define uccs_vdbg uccs_dbg
#else
#define uccs_vdbg(fmt, args...) do { } while (0)
#endif				/* UCCS_VERBOSE_DEBUG */

u32 ucc_slow_get_qe_cr_subblock(int uccs_num)
{
	switch (uccs_num) {
	case (0):
		return (QE_CR_SUBBLOCK_UCCSLOW1);
	case (1):
		return (QE_CR_SUBBLOCK_UCCSLOW2);
	case (2):
		return (QE_CR_SUBBLOCK_UCCSLOW3);
	case (3):
		return (QE_CR_SUBBLOCK_UCCSLOW4);
	case (4):
		return (QE_CR_SUBBLOCK_UCCSLOW5);
	case (5):
		return (QE_CR_SUBBLOCK_UCCSLOW6);
	case (6):
		return (QE_CR_SUBBLOCK_UCCSLOW7);
	case (7):
		return (QE_CR_SUBBLOCK_UCCSLOW8);
	default:
		return QE_CR_SUBBLOCK_INVALID;
	}
}

void ucc_slow_poll_transmitter_now(ucc_slow_private_t * uccs)
{
	out_be16(&uccs->us_regs->utodr, UCC_SLOW_TOD);
}

void ucc_slow_graceful_stop_tx(ucc_slow_private_t * uccs)
{
	ucc_slow_info_t *us_info = uccs->us_info;
	u32 id;

	id = ucc_slow_get_qe_cr_subblock(us_info->ucc_num);
	qe_issue_cmd(QE_GRACEFUL_STOP_TX, id, (u8) QE_CR_PROTOCOL_UNSPECIFIED,
		     0);
}

void ucc_slow_stop_tx(ucc_slow_private_t * uccs)
{
	ucc_slow_info_t *us_info = uccs->us_info;
	u32 id;

	id = ucc_slow_get_qe_cr_subblock(us_info->ucc_num);
	qe_issue_cmd(QE_STOP_TX, id, (u8) QE_CR_PROTOCOL_UNSPECIFIED, 0);
}

void ucc_slow_restart_tx(ucc_slow_private_t * uccs)
{
	ucc_slow_info_t *us_info = uccs->us_info;
	u32 id;

	id = ucc_slow_get_qe_cr_subblock(us_info->ucc_num);
	qe_issue_cmd(QE_RESTART_TX, id, (u8) QE_CR_PROTOCOL_UNSPECIFIED, 0);
}

void ucc_slow_enable(ucc_slow_private_t * uccs, comm_dir_e mode)
{
	ucc_slow_t *us_regs;
	u32 gumr_l;

	us_regs = uccs->us_regs;

	/* Enable reception and/or transmission on this UCC. */
	gumr_l = in_be32(&us_regs->gumr_l);
	if (mode & COMM_DIR_TX) {
		gumr_l |= UCC_SLOW_GUMR_L_ENT;
		uccs->enabled_tx = 1;
	}
	if (mode & COMM_DIR_RX) {
		gumr_l |= UCC_SLOW_GUMR_L_ENR;
		uccs->enabled_rx = 1;
	}
	out_be32(&us_regs->gumr_l, gumr_l);
}

void ucc_slow_disable(ucc_slow_private_t * uccs, comm_dir_e mode)
{
	ucc_slow_t *us_regs;
	u32 gumr_l;

	us_regs = uccs->us_regs;

	/* Disable reception and/or transmission on this UCC. */
	gumr_l = in_be32(&us_regs->gumr_l);
	if (mode & COMM_DIR_TX) {
		gumr_l &= ~UCC_SLOW_GUMR_L_ENT;
		uccs->enabled_tx = 0;
	}
	if (mode & COMM_DIR_RX) {
		gumr_l &= ~UCC_SLOW_GUMR_L_ENR;
		uccs->enabled_rx = 0;
	}
	out_be32(&us_regs->gumr_l, gumr_l);
}

int ucc_slow_init(ucc_slow_info_t * us_info, ucc_slow_private_t ** uccs_ret)
{
	u32 i;
	ucc_slow_t *us_regs;
	u32 gumr;
	u8 function_code = 0;
	u8 *bd;
	ucc_slow_private_t *uccs;
	u32 id;
	u32 command;
	int ret;

	uccs_vdbg("%s: IN", __FUNCTION__);

	if (!us_info)
		return -EINVAL;

	/* check if the UCC port number is in range. */
	if (us_info->ucc_num > UCC_MAX_NUM) {
		uccs_err("ucc_slow_init: Illegal UCC number!");
		return -EINVAL;
	}

	/* Set mrblr */
	/* Check that 'max_rx_buf_length' is properly aligned (4), unless rfw is 1,  */
	/* meaning that QE accepts one byte at a time, unlike normal case when QE    */
	/* accepts 32 bits at a time.                                                */
	if ((!us_info->rfw)
	    && (us_info->max_rx_buf_length & (UCC_SLOW_MRBLR_ALIGNMENT - 1))) {
		uccs_err("max_rx_buf_length not aligned.");
		return -EINVAL;
	}

	uccs =
	    (ucc_slow_private_t *) kmalloc(sizeof(ucc_slow_private_t),
					   GFP_KERNEL);
	if (!uccs) {
		uccs_err
		    ("ucc_slow_init: No memory for UCC slow data structure!");
		return -ENOMEM;
	}
	memset(uccs, 0, sizeof(ucc_slow_private_t));

	/* Fill slow UCC structure */
	uccs->us_info = us_info;
	uccs->saved_uccm = 0;
	uccs->p_rx_frame = 0;
	uccs->us_regs = us_info->us_regs;
	us_regs = uccs->us_regs;
	uccs->p_ucce = (u16 *) & (us_regs->ucce);
	uccs->p_uccm = (u16 *) & (us_regs->uccm);
#ifdef STATISTICS
	uccs->rx_frames = 0;
	uccs->tx_frames = 0;
	uccs->rx_discarded = 0;
#endif				/* STATISTICS */

	/* Get PRAM base */
	uccs->us_pram_offset =
	    qe_muram_alloc(UCC_SLOW_PRAM_SIZE, ALIGNMENT_OF_UCC_SLOW_PRAM);
	if (IS_MURAM_ERR(uccs->us_pram_offset)) {
		uccs_err
		    ("ucc_slow_init: Can not allocate MURAM memory for Slow UCC.");
		ucc_slow_free(uccs);
		return -ENOMEM;
	}
	id = ucc_slow_get_qe_cr_subblock(us_info->ucc_num);
	qe_issue_cmd(QE_ASSIGN_PAGE_TO_DEVICE, id, QE_CR_PROTOCOL_UNSPECIFIED,
		     (u32) uccs->us_pram_offset);

	uccs->us_pram = qe_muram_addr(uccs->us_pram_offset);

	/* Init Guemr register */
	if ((ret = ucc_init_guemr((ucc_common_t *) (us_info->us_regs)))) {
		uccs_err("ucc_slow_init: Could not init the guemr register.");
		ucc_slow_free(uccs);
		return ret;
	}

	/* Set UCC to slow type */
	if ((ret = ucc_set_type(us_info->ucc_num,
				(ucc_common_t *) (us_info->us_regs),
				UCC_SPEED_TYPE_SLOW))) {
		uccs_err("ucc_slow_init: Could not init the guemr register.");
		ucc_slow_free(uccs);
		return ret;
	}

	out_be16(&uccs->us_pram->mrblr, us_info->max_rx_buf_length);

	INIT_LIST_HEAD(&uccs->confQ);

	/* Allocate BDs. */
	uccs->rx_base_offset =
	    qe_muram_alloc(us_info->rx_bd_ring_len * UCC_SLOW_SIZE_OF_BD,
			   QE_ALIGNMENT_OF_BD);
	if (IS_MURAM_ERR(uccs->rx_base_offset)) {
		uccs_err("ucc_slow_init: No memory for Rx BD's.");
		uccs->rx_base_offset = 0;
		ucc_slow_free(uccs);
		return -ENOMEM;
	}

	uccs->tx_base_offset =
	    qe_muram_alloc(us_info->tx_bd_ring_len * UCC_SLOW_SIZE_OF_BD,
			   QE_ALIGNMENT_OF_BD);
	if (IS_MURAM_ERR(uccs->tx_base_offset)) {
		uccs_err("ucc_slow_init: No memory for Tx BD's.");
		uccs->tx_base_offset = 0;
		ucc_slow_free(uccs);
		return -ENOMEM;
	}

	/* Init Tx bds */
	bd = uccs->confBd = uccs->tx_bd = qe_muram_addr(uccs->tx_base_offset);
	for (i = 0; i < us_info->tx_bd_ring_len; i++) {
		BD_BUFFER_CLEAR(bd);
		BD_STATUS_AND_LENGTH_SET(bd, 0);
		bd += QE_SIZEOF_BD;
	}
	bd -= QE_SIZEOF_BD;
	BD_STATUS_AND_LENGTH_SET(bd, T_W);	/* for last BD set Wrap bit */

	/* Init Rx bds */
	bd = uccs->rx_bd = qe_muram_addr(uccs->rx_base_offset);
	for (i = 0; i < us_info->rx_bd_ring_len; i++) {
		BD_STATUS_AND_LENGTH_SET(bd, 0);
		BD_BUFFER_CLEAR(bd);
		bd += QE_SIZEOF_BD;
	}
	bd -= QE_SIZEOF_BD;
	BD_STATUS_AND_LENGTH_SET(bd, R_W);	/* for last BD set Wrap bit */

	/* Set GUMR (For more details see the hardware spec.). */
	/* gumr_h */
	gumr = 0;
	gumr |= us_info->tcrc;
	if (us_info->cdp)
		gumr |= UCC_SLOW_GUMR_H_CDP;
	if (us_info->ctsp)
		gumr |= UCC_SLOW_GUMR_H_CTSP;
	if (us_info->cds)
		gumr |= UCC_SLOW_GUMR_H_CDS;
	if (us_info->ctss)
		gumr |= UCC_SLOW_GUMR_H_CTSS;
	if (us_info->tfl)
		gumr |= UCC_SLOW_GUMR_H_TFL;
	if (us_info->rfw)
		gumr |= UCC_SLOW_GUMR_H_RFW;
	if (us_info->txsy)
		gumr |= UCC_SLOW_GUMR_H_TXSY;
	if (us_info->rtsm)
		gumr |= UCC_SLOW_GUMR_H_RTSM;
	out_be32(&us_regs->gumr_h, gumr);

	/* gumr_l */
	gumr = 0;
	if (us_info->tci)
		gumr |= UCC_SLOW_GUMR_L_TCI;
	if (us_info->rinv)
		gumr |= UCC_SLOW_GUMR_L_RINV;
	if (us_info->tinv)
		gumr |= UCC_SLOW_GUMR_L_TINV;
	if (us_info->tend)
		gumr |= UCC_SLOW_GUMR_L_TEND;
	gumr |= us_info->tdcr;
	gumr |= us_info->rdcr;
	gumr |= us_info->tenc;
	gumr |= us_info->renc;
	gumr |= us_info->diag;
	gumr |= us_info->mode;
	out_be32(&us_regs->gumr_l, gumr);

	/* Function code registers */
	/* function_code has initial value 0 */

	/* if the data is in cachable memory, the 'global' */
	/* in the function code should be set.             */
	function_code |= us_info->data_mem_part;
	function_code |= QE_BMR_BYTE_ORDER_BO_MOT;	/* Required for QE */
	uccs->us_pram->tfcr = function_code;
	uccs->us_pram->rfcr = function_code;

	/* rbase, tbase are offsets from MURAM base */
	out_be16(&uccs->us_pram->rbase, uccs->us_pram_offset);
	out_be16(&uccs->us_pram->tbase, uccs->us_pram_offset);

	/* Mux clocking */
	/* Grant Support */
	ucc_set_qe_mux_grant(us_info->ucc_num, us_info->grant_support);
	/* Breakpoint Support */
	ucc_set_qe_mux_bkpt(us_info->ucc_num, us_info->brkpt_support);
	/* Set Tsa or NMSI mode. */
	ucc_set_qe_mux_tsa(us_info->ucc_num, us_info->tsa);
	/* If NMSI (not Tsa), set Tx and Rx clock. */
	if (!us_info->tsa) {
		/* Rx clock routing */
		if (ucc_set_qe_mux_rxtx
		    (us_info->ucc_num, us_info->rx_clock, COMM_DIR_RX)) {
			uccs_err
			    ("ucc_slow_init: Illegal value for parameter 'RxClock'.");
			ucc_slow_free(uccs);
			return -EINVAL;
		}
		/* Tx clock routing */
		if (ucc_set_qe_mux_rxtx
		    (us_info->ucc_num, us_info->tx_clock, COMM_DIR_TX)) {
			uccs_err
			    ("ucc_slow_init: Illegal value for parameter 'TxClock'.");
			ucc_slow_free(uccs);
			return -EINVAL;
		}
	}

	/*
	 * INTERRUPTS
	 */
	/* Set interrupt mask register at UCC level. */
	out_be16(&us_regs->uccm, us_info->uccm_mask);

	/* First, clear anything pending at UCC level, */
	/* otherwise, old garbage may come through     */
	/* as soon as the dam is opened.               */

	/* Writing '1' clears */
	out_be16(&us_regs->ucce, 0xffff);

	/* Issue QE Init command */
	if (us_info->init_tx && us_info->init_rx)
		command = QE_INIT_TX_RX;
	else if (us_info->init_tx)
		command = QE_INIT_TX;
	else
		command = QE_INIT_RX;	/* We know at least one is TRUE */
	id = ucc_slow_get_qe_cr_subblock(us_info->ucc_num);
	qe_issue_cmd(command, id, (u8) QE_CR_PROTOCOL_UNSPECIFIED, 0);

	*uccs_ret = uccs;
	return 0;
}

void ucc_slow_free(ucc_slow_private_t * uccs)
{
	if (!uccs)
		return;

	if (uccs->rx_base_offset)
		qe_muram_free(uccs->rx_base_offset);

	if (uccs->tx_base_offset)
		qe_muram_free(uccs->tx_base_offset);

	if (uccs->us_pram) {
		qe_muram_free(uccs->us_pram_offset);
		uccs->us_pram = NULL;
	}

	kfree(uccs);
}
