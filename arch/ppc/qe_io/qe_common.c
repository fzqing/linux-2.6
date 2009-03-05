/*
 * arch/ppc/qe_io/qe_common.c
 *
 * General Purpose functions for the global management of the
 * QUICC Engine (QE).
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
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/bootmem.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/immap_qe.h>
#include <asm/qe.h>
#include <asm/rheap.h>

#define QE_MAP_SIZE    (0x100000)	/* 1MB */

/* QE snum state
*/
typedef enum qe_snum_state {
	QE_SNUM_STATE_USED,	/* used */
	QE_SNUM_STATE_FREE	/* free */
} qe_snum_state_e;

/* QE snum
*/
typedef struct qe_snum {
	u8 num;			/* snum  */
	qe_snum_state_e state;	/* state */
} qe_snum_t;

/* We allocate this here because it is used almost exclusively for
 * the communication processor devices.
 */
EXPORT_SYMBOL(qe_immr);
qe_map_t *qe_immr;
static qe_snum_t snums[QE_NUM_OF_SNUM];	/* Dynamically allocated SNUMs  */

static void qe_snums_init(void);
static void qe_muram_init(void);
static int qe_sdma_init(void);

void qe_reset(void)
{
	qe_immr = (qe_map_t *) ioremap(QE_MAP_ADDR, QE_MAP_SIZE);

	qe_snums_init();

	qe_issue_cmd(QE_RESET, QE_CR_SUBBLOCK_INVALID,
		     (u8) QE_CR_PROTOCOL_UNSPECIFIED, 0);

	/* Reclaim the MURAM memory for our use. */
	qe_muram_init();

	if (qe_sdma_init())
		panic("sdma init failed!");
}

EXPORT_SYMBOL(qe_issue_cmd);
int qe_issue_cmd(uint cmd, uint device, u8 mcn_protocol, u32 cmd_input)
{
	unsigned long flags;
	u32 cecr;
	u8 mcn_shift = 0, dev_shift = 0;

	local_irq_save(flags);
	if (cmd == QE_RESET) {
		out_be32(&qe_immr->cp.cecr, (u32) (cmd | QE_CR_FLG));
	} else {
		if (cmd == QE_ASSIGN_PAGE) {
			/* Here device is the SNUM, not sub-block */
			dev_shift = QE_CR_SNUM_SHIFT;
		} else if (cmd == QE_ASSIGN_RISC) {
			/* Here device is the SNUM, and mcnProtocol is e_QeCmdRiscAssignment value */
			dev_shift = QE_CR_SNUM_SHIFT;
			mcn_shift = QE_CR_MCN_RISC_ASSIGN_SHIFT;
		} else {
			if (device == QE_CR_SUBBLOCK_USB)
				mcn_shift = QE_CR_MCN_USB_SHIFT;
			else
				mcn_shift = QE_CR_MCN_NORMAL_SHIFT;
		}

		out_be32(&qe_immr->cp.cecdr, cmd_input);
		out_be32(&qe_immr->cp.cecr,
			 (cmd | QE_CR_FLG | ((u32) device << dev_shift) | (u32)
			  mcn_protocol << mcn_shift));
	}

	/* wait for the QE_CR_FLG to clear */
	do {
		cecr = in_be32(&qe_immr->cp.cecr);
	} while (cecr & QE_CR_FLG);
	local_irq_restore(flags);

	return 0;
}

/* Set a baud rate generator. This needs lots of work. There are
 * 16 BRGs, which can be connected to the QE channels or output
 * as clocks. The BRGs are in two different block of internal
 * memory mapped space.
 * The baud rate clock is the system clock divided by something.
 * It was set up long ago during the initial boot phase and is
 * is given to us.
 * Baud rate clocks are zero-based in the driver code (as that maps
 * to port numbers). Documentation uses 1-based numbering.
 */
#define BRG_CLK    (((bd_t *)__res)->bi_brgfreq)

/* This function is used by UARTS, or anything else that uses a 16x
 * oversampled clock.
 */
void qe_setbrg(uint brg, uint rate)
{
	volatile uint *bp;
	u32 divisor;
	int div16 = 0;

	bp = (uint *) & qe_immr->brg.brgc1;
	bp += brg;

	divisor = (BRG_CLK / rate);
	if (divisor > QE_BRGC_DIVISOR_MAX + 1) {
		div16 = 1;
		divisor /= 16;
	}

	*bp = ((divisor - 1) << QE_BRGC_DIVISOR_SHIFT) | QE_BRGC_ENABLE;
	if (div16)
		*bp |= QE_BRGC_DIV16;
}

static void qe_snums_init(void)
{
	int i;

	/* Initialize the SNUMs array. */
	for (i = 0; i < QE_NUM_OF_SNUM; i++)
		snums[i].state = QE_SNUM_STATE_FREE;

	/* Initialize SNUMs (thread serial numbers) according to QE spec chapter 4, SNUM table */
	i = 0;
	snums[i++].num = 0x04;
	snums[i++].num = 0x05;
	snums[i++].num = 0x0C;
	snums[i++].num = 0x0D;
	snums[i++].num = 0x14;
	snums[i++].num = 0x15;
	snums[i++].num = 0x1C;
	snums[i++].num = 0x1D;
	snums[i++].num = 0x24;
	snums[i++].num = 0x25;
	snums[i++].num = 0x2C;
	snums[i++].num = 0x2D;
	snums[i++].num = 0x34;
	snums[i++].num = 0x35;
	snums[i++].num = 0x88;
	snums[i++].num = 0x89;
	snums[i++].num = 0x98;
	snums[i++].num = 0x99;
	snums[i++].num = 0xA8;
	snums[i++].num = 0xA9;
	snums[i++].num = 0xB8;
	snums[i++].num = 0xB9;
	snums[i++].num = 0xC8;
	snums[i++].num = 0xC9;
	snums[i++].num = 0xD8;
	snums[i++].num = 0xD9;
	snums[i++].num = 0xE8;
	snums[i++].num = 0xE9;
}

int qe_get_snum(void)
{
	unsigned long flags;
	int snum = -EBUSY;
	int i;

	local_irq_save(flags);
	for (i = 0; i < QE_NUM_OF_SNUM; i++) {
		if (snums[i].state == QE_SNUM_STATE_FREE) {
			snums[i].state = QE_SNUM_STATE_USED;
			snum = snums[i].num;
			break;
		}
	}
	local_irq_restore(flags);

	return snum;
}

EXPORT_SYMBOL(qe_get_snum);

void qe_put_snum(u8 snum)
{
	int i;

	for (i = 0; i < QE_NUM_OF_SNUM; i++) {
		if (snums[i].num == snum) {
			snums[i].state = QE_SNUM_STATE_FREE;
			break;
		}
	}
}

EXPORT_SYMBOL(qe_put_snum);

static int qe_sdma_init(void)
{
	sdma_t *sdma = &qe_immr->sdma;
	uint sdma_buf_offset;

	if (!sdma)
		return -ENODEV;

	/* allocate 2 internal temporary buffers (512 bytes size each) for the SDMA */
	sdma_buf_offset = qe_muram_alloc(512 * 2, 64);
	if (IS_MURAM_ERR(sdma_buf_offset))
		return -ENOMEM;

	out_be32(&sdma->sdebcr, sdma_buf_offset & QE_SDEBCR_BA_MASK);
	out_be32(&sdma->sdmr, (QE_SDMR_GLB_1_MSK | (0x1 >> QE_SDMR_CEN_SHIFT)));

	return 0;
}

/*
 * muram_alloc / muram_free bits.
 */
static spinlock_t qe_muram_lock;
/* 16 blocks should be enough to satisfy all requests
 * until the memory subsystem goes up... */
static rh_block_t qe_boot_muram_rh_block[16];
static rh_info_t qe_muram_info;

static void qe_muram_init(void)
{
	spin_lock_init(&qe_muram_lock);

	/* initialize the info header */
	rh_init(&qe_muram_info, 1,
		sizeof(qe_boot_muram_rh_block) /
		sizeof(qe_boot_muram_rh_block[0]), qe_boot_muram_rh_block);

	/* Attach the usable muram area */
	/* XXX: This is actually crap. QE_DATAONLY_BASE and
	 * QE_DATAONLY_SIZE is only a subset of the available muram. It
	 * varies with the processor and the microcode patches activated.
	 * But the following should be at least safe.
	 */
	rh_attach_region(&qe_muram_info,
			 (void *)QE_MURAM_DATAONLY_BASE,
			 QE_MURAM_DATAONLY_SIZE);
}

/* This function returns an index into the MURAM area.
 */
uint qe_muram_alloc(uint size, uint align)
{
	void *start;
	unsigned long flags;

	spin_lock_irqsave(&qe_muram_lock, flags);
	start = rh_alloc_align(&qe_muram_info, size, align, "QE");
	spin_unlock_irqrestore(&qe_muram_lock, flags);

	return (uint) start;
}

EXPORT_SYMBOL(qe_muram_alloc);

int qe_muram_free(uint offset)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&qe_muram_lock, flags);
	ret = rh_free(&qe_muram_info, (void *)offset);
	spin_unlock_irqrestore(&qe_muram_lock, flags);

	return ret;
}

EXPORT_SYMBOL(qe_muram_free);

/* not sure if this is ever needed */
uint qe_muram_alloc_fixed(uint offset, uint size)
{
	void *start;
	unsigned long flags;

	spin_lock_irqsave(&qe_muram_lock, flags);
	start =
	    rh_alloc_fixed(&qe_muram_info, (void *)offset, size, "commproc");
	spin_unlock_irqrestore(&qe_muram_lock, flags);

	return (uint) start;
}

EXPORT_SYMBOL(qe_muram_alloc_fixed);

void qe_muram_dump(void)
{
	rh_dump(&qe_muram_info);
}

EXPORT_SYMBOL(qe_muram_dump);

void *qe_muram_addr(uint offset)
{
	return (void *)&qe_immr->muram[offset];
}

EXPORT_SYMBOL(qe_muram_addr);
