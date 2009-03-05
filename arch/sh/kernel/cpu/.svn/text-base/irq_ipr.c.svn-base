/* $Id: irq_ipr.c,v 1.1.2.1 2002/11/17 10:53:43 mrbrown Exp $
 *
 * linux/arch/sh/kernel/irq_ipr.c
 *
 * Copyright (C) 1999  Niibe Yutaka & Takeshi Yaegashi
 * Copyright (C) 2000  Kazumoto Kojima
 * Copyright (C) 2003 Takashi Kusuda <kusuda-takashi@hitachi-ul.co.jp>
 * Copyright (C) 2004 Takashi Kusuda
 * Copyright (C) 2005 Nobuhiro Iwamatsu
 *
 * Interrupt handling for IPR-based IRQ.
 *
 * Supported system:
 *	On-chip supporting modules (TMU, RTC, etc.).
 *	On-chip supporting modules for SH7300/SH7705/SH7706/SH7709/SH7709A/
 *                                     SH7709S/SH7710/SH7720/SH7727/SH7729.
 *
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/machvec.h>

struct ipr_data {
	unsigned int addr;	/* Address of Interrupt Priority Register */
	int shift;		/* Shifts of the 16-bit data */
	int priority;		/* The priority */
};
static struct ipr_data ipr_data[NR_IRQS];

static void enable_ipr_irq(unsigned int irq);
static void disable_ipr_irq(unsigned int irq);

/* shutdown is same as "disable" */
#define shutdown_ipr_irq disable_ipr_irq

static void mask_and_ack_ipr(unsigned int);
static void end_ipr_irq(unsigned int irq);

static unsigned int startup_ipr_irq(unsigned int irq)
{
	enable_ipr_irq(irq);
	return 0; /* never anything pending */
}

static struct hw_interrupt_type ipr_irq_type = {
	"IPR-IRQ",
	startup_ipr_irq,
	shutdown_ipr_irq,
	enable_ipr_irq,
	disable_ipr_irq,
	mask_and_ack_ipr,
	end_ipr_irq
};

static void disable_ipr_irq(unsigned int irq)
{
	unsigned long val, flags;
	unsigned int addr = ipr_data[irq].addr;
	unsigned short mask = 0xffff ^ (0x0f << ipr_data[irq].shift);

	/* Set the priority in IPR to 0 */
	local_irq_save(flags);
	val = ctrl_inw(addr);
	val &= mask;
	ctrl_outw(val, addr);
	local_irq_restore(flags);
}

static void enable_ipr_irq(unsigned int irq)
{
	unsigned long val, flags;
	unsigned int addr = ipr_data[irq].addr;
	int priority = ipr_data[irq].priority;
	unsigned short value = (priority << ipr_data[irq].shift);

	/* Set priority in IPR back to original value */
	local_irq_save(flags);
	val = ctrl_inw(addr);
	val |= value;
	ctrl_outw(val, addr);
	local_irq_restore(flags);
}

static void mask_and_ack_ipr(unsigned int irq)
{
	disable_ipr_irq(irq);

#if defined(IRQ0_IRQ)
	/* This is needed when we use edge triggered setting */
	/* XXX: Is it really needed? */
	if (IRQ0_IRQ <= irq && irq <= IRQ5_IRQ) {
		/* Clear external interrupt request */
		int a = ctrl_inb(INTC_IRR0);
		a &= ~(1 << (irq - IRQ0_IRQ));
		ctrl_outb(a, INTC_IRR0);
	}
#endif
}

static void end_ipr_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		enable_ipr_irq(irq);
}

void make_ipr_irq(unsigned int irq, unsigned int addr, int pos, int priority)
{
	disable_irq_nosync(irq);
	ipr_data[irq].addr = addr;
	ipr_data[irq].shift = pos*4; /* POSition (0-3) x 4 means shift */
	ipr_data[irq].priority = priority;

	irq_desc[irq].handler = &ipr_irq_type;
	disable_ipr_irq(irq);
}

#if defined(SH_IPR_INIT_TABLE)
	static struct sh_ipr_irq_info sh_ipr_regs_init[] __initdata = SH_IPR_INIT_TABLE;
#endif

void __init init_ipr_IRQ(void)
{
#if defined(SH_IPR_INIT_TABLE)
	int i;

	for(i = 0; sh_ipr_regs_init[i].irq != 0; i++)
	{
		make_ipr_irq(sh_ipr_regs_init[i].irq,
		sh_ipr_regs_init[i].addr,
		sh_ipr_regs_init[i].pos,
		sh_ipr_regs_init[i].priority);
	}
#endif

	/* Perform the machine specific initialisation */
	if (sh_mv.mv_init_irq != NULL) {
		sh_mv.mv_init_irq();
	}
}

EXPORT_SYMBOL(make_ipr_irq);

