/*
 * arch/ppc/syslib/qe_ic.c
 *
 * QE IC routines implementations.
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
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/qe_ic.h>

#include "qe_ic.h"

static struct qe_ic_private p_qe_ic;
static struct qe_ic_private *primary_qe_ic;
static DEFINE_RAW_SPINLOCK(qe_ic_lock);

static struct qe_ic_info qe_ic_info[] = {
	[1] = {
	       .mask = 0x00008000,
	       .qimr = 1,
	       .pri_code = 0},
	[2] = {
	       .mask = 0x00004000,
	       .qimr = 1,
	       .pri_code = 1},
	[3] = {
	       .mask = 0x00002000,
	       .qimr = 1,
	       .pri_code = 2},
	[10] = {
		.mask = 0x00000040,
		.qimr = 1,
		.pri_code = 1},
	[11] = {
		.mask = 0x00000020,
		.qimr = 1,
		.pri_code = 2},
	[12] = {
		.mask = 0x00000010,
		.qimr = 1,
		.pri_code = 3},
	[13] = {
		.mask = 0x00000008,
		.qimr = 1,
		.pri_code = 4},
	[14] = {
		.mask = 0x00000004,
		.qimr = 1,
		.pri_code = 5},
	[15] = {
		.mask = 0x00000002,
		.qimr = 1,
		.pri_code = 6},
	[20] = {
		.mask = 0x10000000,
		.qimr = 0,
		.pri_code = 3},
	[25] = {
		.mask = 0x00800000,
		.qimr = 0,
		.pri_code = 0},
	[26] = {
		.mask = 0x00400000,
		.qimr = 0,
		.pri_code = 1},
	[27] = {
		.mask = 0x00200000,
		.qimr = 0,
		.pri_code = 2},
	[28] = {
		.mask = 0x00100000,
		.qimr = 0,
		.pri_code = 3},
	[32] = {
		.mask = 0x80000000,
		.qimr = 1,
		.pri_code = 0},
	[33] = {
		.mask = 0x40000000,
		.qimr = 1,
		.pri_code = 1},
	[34] = {
		.mask = 0x20000000,
		.qimr = 1,
		.pri_code = 2},
	[35] = {
		.mask = 0x10000000,
		.qimr = 1,
		.pri_code = 3},
	[36] = {
		.mask = 0x08000000,
		.qimr = 1,
		.pri_code = 4},
	[40] = {
		.mask = 0x00800000,
		.qimr = 1,
		.pri_code = 0},
	[41] = {
		.mask = 0x00400000,
		.qimr = 1,
		.pri_code = 1},
	[42] = {
		.mask = 0x00200000,
		.qimr = 1,
		.pri_code = 2},
	[43] = {
		.mask = 0x00100000,
		.qimr = 1,
		.pri_code = 3},
};

struct hw_interrupt_type qe_ic = {
	.typename = " QE IC  ",
	.enable = qe_ic_enable_irq,
	.disable = qe_ic_disable_irq,
	.ack = qe_ic_disable_irq_and_ack,
	.end = qe_ic_end_irq,
};

static int qe_ic_get_low_irq(struct pt_regs *regs)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;
	int irq = -1;

	/* get the low byte of SIVEC to get the interrupt source vector. */
	irq = (in_be32(&p_qe_ic->regs->qivec) >> 24) >> 2;

	if (irq == 0)		/* 0 --> no irq is pending */
		return -1;

	return irq + p_qe_ic->irq_offset;
}

static int qe_ic_get_high_irq(struct pt_regs *regs)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;
	int irq = -1;

	/* get the high byte of SIVEC to get the interrupt source vector. */
	irq = (in_be32(&p_qe_ic->regs->qhivec) >> 24) >> 2;

	if (irq == 0)		/* 0 --> no irq is pending */
		return -1;

	return irq + p_qe_ic->irq_offset;
}

static irqreturn_t qe_ic_cascade_low(int irq, void *dev_id,
				     struct pt_regs *regs)
{
	while ((irq = qe_ic_get_low_irq(regs)) >= 0)
		__do_IRQ(irq, regs);
	return IRQ_HANDLED;
}

static irqreturn_t qe_ic_cascade_high(int irq, void *dev_id,
				      struct pt_regs *regs)
{
	while ((irq = qe_ic_get_high_irq(regs)) >= 0)
		__do_IRQ(irq, regs);
	return IRQ_HANDLED;
}

static struct irqaction qe_ic_low_irqaction = {
	.handler = qe_ic_cascade_low,
	.flags = SA_INTERRUPT | SA_NODELAY,
	.mask = CPU_MASK_NONE,
	.name = "qe_ic_cascade_low",
};

static struct irqaction qe_ic_high_irqaction = {
	.handler = qe_ic_cascade_high,
	.flags = SA_INTERRUPT | SA_NODELAY,
	.mask = CPU_MASK_NONE,
	.name = "qe_ic_cascade_high",
};

int qe_ic_init(phys_addr_t phys_addr,
	       unsigned int flags, unsigned int irq_offset)
{
	struct qe_ic_map *regs;
	u8 grp, pri, shift = 0;
	u32 tmp_qicr = 0, tmp_qricr = 0, tmp_qicnr = 0, tmp_mask;
	int i, high_hctive = 0;
	const u32 high_signal = 2;

	primary_qe_ic = &p_qe_ic;
	memset(primary_qe_ic, 0, sizeof(struct qe_ic_private));

	/* initialize QE interrupt controller registers */
	primary_qe_ic->regs = regs =
	    (struct qe_ic_map *)ioremap(phys_addr, QE_IC_SIZE);
	primary_qe_ic->irq_offset = irq_offset;

	/* default priority scheme is grouped. If spread mode is    */
	/* required, configure sicr accordingly.                    */
	if (flags & QE_IC_SPREADMODE_GRP_W)
		tmp_qicr |= QICR_GWCC;
	if (flags & QE_IC_SPREADMODE_GRP_X)
		tmp_qicr |= QICR_GXCC;
	if (flags & QE_IC_SPREADMODE_GRP_Y)
		tmp_qicr |= QICR_GYCC;
	if (flags & QE_IC_SPREADMODE_GRP_Z)
		tmp_qicr |= QICR_GZCC;
	if (flags & QE_IC_SPREADMODE_GRP_RISCA)
		tmp_qicr |= QICR_GRTA;
	if (flags & QE_IC_SPREADMODE_GRP_RISCB)
		tmp_qicr |= QICR_GRTB;

	/* choose destination signal for highest priority interrupt */
	if (flags & QE_IC_HIGH_SIGNAL) {
		tmp_qicr |= (high_signal << QICR_HPIT_SHIFT);
		high_hctive = 1;
	}

	out_be32(&regs->qicr, tmp_qicr);

	tmp_mask = (1 << QE_IC_GRP_W_DEST_SIGNAL_SHIFT);
	/* choose destination signal for highest priority interrupt in each group */
	for (grp = 0; grp < NUM_OF_QE_IC_GROUPS; grp++) {
		/* the first 2 priorities in each group have a choice of destination signal */
		for (pri = 0; pri <= 1; pri++) {
			if (flags & ((tmp_mask << (grp << 1)) << pri)) {
				/* indicate whether QE High signal is required */
				if (!high_hctive)
					high_hctive = 1;

				/* The location of the bits relevant to priority 0 in the   */
				/* registers is always 2 bits left comparing to priority 1. */
				if (pri == 0)
					shift = 2;

				switch (grp) {
				case (QE_IC_GRP_W):
					shift += QICNR_WCC1T_SHIFT;
					tmp_qicnr |= high_signal << shift;
					break;
				case (QE_IC_GRP_X):
					shift += QICNR_XCC1T_SHIFT;
					tmp_qicnr |= high_signal << shift;
					break;
				case (QE_IC_GRP_Y):
					shift += QICNR_YCC1T_SHIFT;
					tmp_qicnr |= high_signal << shift;
					break;
				case (QE_IC_GRP_Z):
					shift += QICNR_ZCC1T_SHIFT;
					tmp_qicnr |= high_signal << shift;
					break;
				case (QE_IC_GRP_RISCA):
					shift += QRICR_RTA1T_SHIFT;
					tmp_qricr |= high_signal << shift;
					break;
				case (QE_IC_GRP_RISCB):
					shift += QRICR_RTB1T_SHIFT;
					tmp_qricr |= high_signal << shift;
					break;
				default:
					break;
				}
			}
		}
	}

	if (tmp_qicnr)
		out_be32(&regs->qicnr, tmp_qicnr);
	if (tmp_qricr)
		out_be32(&regs->qricr, tmp_qricr);

	for (i = primary_qe_ic->irq_offset;
	     i < (NR_QE_IC_INTS + primary_qe_ic->irq_offset); i++) {
		irq_desc[i].handler = &qe_ic;
		irq_desc[i].status = IRQ_LEVEL;
	}

	/* register QE_IC interrupt controller in the a higher hirarchy controller */
	setup_irq(IRQ_QE_LOW, &qe_ic_low_irqaction);

	if (high_hctive)
		/* register QE_IC high interrupt source in the higher hirarchy controller */
		setup_irq(IRQ_QE_HIGH, &qe_ic_high_irqaction);

	printk("QE IC (%d IRQ sources) at %p\n", NR_QE_IC_INTS,
	       primary_qe_ic->regs);
	return 0;
}

void qe_ic_free(void)
{
}

void qe_ic_enable_irq(unsigned int irq)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;
	unsigned int src = irq - p_qe_ic->irq_offset;
	u32 qimr;
	unsigned long flags;

	spin_lock_irqsave(&qe_ic_lock, flags);
	if (qe_ic_info[src].qimr) {
		qimr = in_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qimr);
		out_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qimr,
			 qimr | (qe_ic_info[src].mask));
	} else {
		qimr = in_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qrimr);
		out_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qrimr,
			 qimr | (qe_ic_info[src].mask));
	}
	spin_unlock_irqrestore(&qe_ic_lock, flags);
}

void qe_ic_disable_irq(unsigned int irq)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;
	unsigned int src = irq - p_qe_ic->irq_offset;
	u32 qimr;
	unsigned long flags;

	spin_lock_irqsave(&qe_ic_lock, flags);
	if (qe_ic_info[src].qimr) {
		qimr = in_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qimr);
		out_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qimr,
			 qimr & ~(qe_ic_info[src].mask));
	} else {
		qimr = in_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qrimr);
		out_be32(&((struct qe_ic_private *)p_qe_ic)->regs->qrimr,
			 qimr & ~(qe_ic_info[src].mask));
	}
	spin_unlock_irqrestore(&qe_ic_lock, flags);
}

void qe_ic_disable_irq_and_ack(unsigned int irq)
{
	qe_ic_disable_irq(irq);
}

void qe_ic_end_irq(unsigned int irq)
{

	if (!(irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS))
	    && irq_desc[irq].action)
		qe_ic_enable_irq(irq);
}

void qe_ic_modify_highest_priority(unsigned int irq)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;
	unsigned int src = irq - p_qe_ic->irq_offset;
	u32 tmp_qicr = 0;
	unsigned long flags;

	spin_lock_irqsave(&qe_ic_lock, flags);
	tmp_qicr = in_be32(&p_qe_ic->regs->qicr);
	out_be32(&p_qe_ic->regs->qicr, (u32) (tmp_qicr | ((u8) src << 24)));
	spin_unlock_irqrestore(&qe_ic_lock, flags);
}

void qe_ic_modify_priority(enum qe_ic_grp_id grp,
			   unsigned int pri0,
			   unsigned int pri1,
			   unsigned int pri2,
			   unsigned int pri3,
			   unsigned int pri4,
			   unsigned int pri5,
			   unsigned int pri6, unsigned int pri7)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;
	volatile u32 *p_qip = 0;
	u32 tmp_qip = 0;
	u8 tmp_array[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	signed char code_array[8], i = 0, j = 0;

	code_array[0] = (signed char)(pri0 ? qe_ic_info[pri0].pri_code : -1);
	code_array[1] = (signed char)(pri1 ? qe_ic_info[pri1].pri_code : -1);
	code_array[2] = (signed char)(pri2 ? qe_ic_info[pri2].pri_code : -1);
	code_array[3] = (signed char)(pri3 ? qe_ic_info[pri3].pri_code : -1);
	code_array[4] = (signed char)(pri4 ? qe_ic_info[pri4].pri_code : -1);
	code_array[5] = (signed char)(pri5 ? qe_ic_info[pri5].pri_code : -1);
	code_array[6] = (signed char)(pri6 ? qe_ic_info[pri6].pri_code : -1);
	code_array[7] = (signed char)(pri7 ? qe_ic_info[pri7].pri_code : -1);

	for (i = 0; i < 8; i++) {
		if (code_array[i] == -1)
			break;
		tmp_array[code_array[i]] = 1;
	}

	for (; i < 8; i++) {
		while (tmp_array[j] && j < 8)
			j++;
		code_array[i] = j;
		tmp_array[j] = 1;
	}

	tmp_qip = (u32) (code_array[0] << QIPCC_SHIFT_PRI0 |
			 code_array[1] << QIPCC_SHIFT_PRI1 |
			 code_array[2] << QIPCC_SHIFT_PRI2 |
			 code_array[3] << QIPCC_SHIFT_PRI3 |
			 code_array[4] << QIPCC_SHIFT_PRI4 |
			 code_array[5] << QIPCC_SHIFT_PRI5 |
			 code_array[6] << QIPCC_SHIFT_PRI6 |
			 code_array[7] << QIPCC_SHIFT_PRI7);

	switch (grp) {
	case (QE_IC_GRP_W):
		p_qip = &(p_qe_ic->regs->qipwcc);
		break;
	case (QE_IC_GRP_X):
		p_qip = &(p_qe_ic->regs->qipxcc);
		break;
	case (QE_IC_GRP_Y):
		p_qip = &(p_qe_ic->regs->qipycc);
		break;
	case (QE_IC_GRP_Z):
		p_qip = &(p_qe_ic->regs->qipzcc);
		break;
	case (QE_IC_GRP_RISCA):
		p_qip = &(p_qe_ic->regs->qiprta);
		break;
	case (QE_IC_GRP_RISCB):
		p_qip = &(p_qe_ic->regs->qiprtb);
		break;
	default:
		break;
	}

	out_be32(p_qip, tmp_qip);
}

void qe_ic_dump_regs(void)
{
	struct qe_ic_private *p_qe_ic = primary_qe_ic;

	printk(KERN_INFO "QE IC registars:\n");
	printk(KERN_INFO "Base address: 0x%08x\n", (u32) p_qe_ic->regs);
	printk(KERN_INFO "qicr  : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qicr, in_be32(&p_qe_ic->regs->qicr));
	printk(KERN_INFO "qivec : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qivec, in_be32(&p_qe_ic->regs->qivec));
	printk(KERN_INFO "qripnr: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qripnr, in_be32(&p_qe_ic->regs->qripnr));
	printk(KERN_INFO "qipnr : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qipnr, in_be32(&p_qe_ic->regs->qipnr));
	printk(KERN_INFO "qipxcc: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qipxcc, in_be32(&p_qe_ic->regs->qipxcc));
	printk(KERN_INFO "qipycc: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qipycc, in_be32(&p_qe_ic->regs->qipycc));
	printk(KERN_INFO "qipwcc: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qipwcc, in_be32(&p_qe_ic->regs->qipwcc));
	printk(KERN_INFO "qipzcc: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qipzcc, in_be32(&p_qe_ic->regs->qipzcc));
	printk(KERN_INFO "qimr  : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qimr, in_be32(&p_qe_ic->regs->qimr));
	printk(KERN_INFO "qrimr : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qrimr, in_be32(&p_qe_ic->regs->qrimr));
	printk(KERN_INFO "qicnr : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qicnr, in_be32(&p_qe_ic->regs->qicnr));
	printk(KERN_INFO "qiprta: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qiprta, in_be32(&p_qe_ic->regs->qiprta));
	printk(KERN_INFO "qiprtb: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qiprtb, in_be32(&p_qe_ic->regs->qiprtb));
	printk(KERN_INFO "qricr : addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qricr, in_be32(&p_qe_ic->regs->qricr));
	printk(KERN_INFO "qhivec: addr - 0x%08x, val - 0x%08x\n",
	       (u32) & p_qe_ic->regs->qhivec, in_be32(&p_qe_ic->regs->qhivec));
}
