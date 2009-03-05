/*
 * arch/mps/tx4939/common/irq.c
 *
 * Common tx4939 irq handler
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/mipsregs.h>
#include <asm/tx4939/tx4939.h>
#include <asm/tx4939/rbtx4939.h>

/**
 * tx4939_irq_handler - TX4939 interrupt exception handler
 * @regs: The stack pointer which store the registers during a system
 * call/exception.
 *
 * This is TX4939 interrupt execption handler.
 * This checks CP0.status register, and calls do_IRQ routine.
 */

extern asmlinkage void tx4939_irq_handler(void);

/***************************************************
 * Forward definitions for all pic's
 ***************************************************/

static unsigned int tx4939_irq_cp0_startup(unsigned int irq);
static void tx4939_irq_cp0_shutdown(unsigned int irq);
static void tx4939_irq_cp0_enable(unsigned int irq);
static void tx4939_irq_cp0_disable(unsigned int irq);
static void tx4939_irq_cp0_mask_and_ack(unsigned int irq);
static void tx4939_irq_cp0_end(unsigned int irq);

static unsigned int tx4939_irq_irc_startup(unsigned int irq);
static void tx4939_irq_irc_shutdown(unsigned int irq);
static void tx4939_irq_irc_enable(unsigned int irq);
static void tx4939_irq_irc_disable(unsigned int irq);
static void tx4939_irq_irc_mask_and_ack(unsigned int irq);
static void tx4939_irq_irc_end(unsigned int irq);

/***************************************************
 * Kernel structs for all pic's
 ***************************************************/

static raw_spinlock_t tx4939_cp0_lock = RAW_SPIN_LOCK_UNLOCKED;
static raw_spinlock_t tx4939_irc_lock = RAW_SPIN_LOCK_UNLOCKED;

#define TX4939_CP0_NAME "TX4939-CP0"
static struct hw_interrupt_type tx4939_irq_cp0_type = {
	.typename = TX4939_CP0_NAME,
	.startup = tx4939_irq_cp0_startup,
	.shutdown = tx4939_irq_cp0_shutdown,
	.enable = tx4939_irq_cp0_enable,
	.disable = tx4939_irq_cp0_disable,
	.ack = tx4939_irq_cp0_mask_and_ack,
	.end = tx4939_irq_cp0_end,
	.set_affinity = NULL
};

#define TX4939_IRC_NAME "TX4939-IRC"
static struct hw_interrupt_type tx4939_irq_irc_type = {
	.typename = TX4939_IRC_NAME,
	.startup = tx4939_irq_irc_startup,
	.shutdown = tx4939_irq_irc_shutdown,
	.enable = tx4939_irq_irc_enable,
	.disable = tx4939_irq_irc_disable,
	.ack = tx4939_irq_irc_mask_and_ack,
	.end = tx4939_irq_irc_end,
	.set_affinity = NULL
};

static struct irqaction tx4939_irq_irc_action = {
	.handler = no_action,
	.name = TX4939_IRC_NAME
};

/***************************************************
 * Functions for cp0
 ***************************************************/

#define tx4939_irq_cp0_mask(irq) ( 0x1 << ( (irq) - TX4939_IRQ_CP0_BEG + CAUSEB_IP) )

/**
 * tx4939_irq_cp0_init - Initialize TX49 core interrupt routine
 *
 * This function sets up TX4939 core interrupt routine irq_dest_t
 * descriptor.
 */
static void __init tx4939_irq_cp0_init(void)
{
	int i;
	for (i = TX4939_IRQ_CP0_BEG; i <= TX4939_IRQ_CP0_END; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = 0;
		irq_desc[i].depth = 1;
		irq_desc[i].handler = &tx4939_irq_cp0_type;
	}
}

static unsigned int tx4939_irq_cp0_startup(unsigned int irq)
{
	tx4939_irq_cp0_enable(irq);
	return 0;
}

static void tx4939_irq_cp0_shutdown(unsigned int irq)
{
	tx4939_irq_cp0_disable(irq);
}

static void tx4939_irq_cp0_enable(unsigned int irq)
{
	unsigned long flags;
	spin_lock_irqsave(&tx4939_cp0_lock, flags);
	set_c0_status(tx4939_irq_cp0_mask(irq));
	spin_unlock_irqrestore(&tx4939_cp0_lock, flags);
}

static void tx4939_irq_cp0_disable(unsigned int irq)
{
	unsigned long flags;
	spin_lock_irqsave(&tx4939_cp0_lock, flags);
	clear_c0_status(tx4939_irq_cp0_mask(irq));
	spin_unlock_irqrestore(&tx4939_cp0_lock, flags);
}

static void tx4939_irq_cp0_mask_and_ack(unsigned int irq)
{
	tx4939_irq_cp0_disable(irq);
}

static void tx4939_irq_cp0_end(unsigned int irq)
{
	if (irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS))
		return;

	tx4939_irq_cp0_enable(irq);
}

/***************************************************
 * Functions for pic
 ***************************************************/

static void tx4939_irq_set_irc_lvl(unsigned int ic, unsigned int lvl)
{
	volatile u32 *irlvl;
	unsigned int offset;

	ic = (ic - 1 + ((ic & 0x20) >> 5)) & 0x3f;
	irlvl = (volatile u32 *)(TX4939_IRC_REG + 0x20);

	offset = ((ic & 0x1) << 3) + ((ic & 0x10));
	irlvl += (ic & 0xe) + ((ic & 0x20) >> 1);

	*irlvl &= ~(0x7 << offset);
	*irlvl |= ((lvl & 0x7) << offset);

	{			/* prevent spurious interrupt */
		volatile u32 tmp;
		__asm__ __volatile__("sync\n\t");
		tmp = *irlvl;
	}
}

/**
 * tx4939_irq_irc_init - Initialize TX4939 IRC routine
 *
 * This function sets up TX4939 IRC irq_desc_t descriptoer. and sets
 * up IRC to enable.
 */

static void __init tx4939_irq_irc_init(void)
{
	unsigned long flags;
	int i;
	u32 v;

	for (i = TX4939_IRQ_IRC_BEG; i <= TX4939_IRQ_IRC_END; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = 0;
		irq_desc[i].depth = 2;
		irq_desc[i].handler = &tx4939_irq_irc_type;
	}

	setup_irq(TX4939_IRQ_IRC_CP0, &tx4939_irq_irc_action);

	spin_lock_irqsave(&tx4939_irc_lock, flags);
	/* set compatible mode */
	v = reg_rd32(&tx4939_ircptr->iscipb);
	reg_wr32(&tx4939_ircptr->iscipb, v | TX4939_ISCIPB_CMM_COMPATIBLE);
	/* irq level mask -- only accept hightest */
	reg_wr32(&tx4939_ircptr->msk, TX4939_IRMSK_IML_LEVEL(0x6));
	/* irq enable */
	v = reg_rd32(&tx4939_ircptr->den);
	reg_wr32(&tx4939_ircptr->den, v | TX4939_IRDEN_IDE_START);

	spin_unlock_irqrestore(&tx4939_irc_lock, flags);
}

static unsigned int tx4939_irq_irc_startup(unsigned int irq)
{
	tx4939_irq_irc_enable(irq);
	return 0;
}

static void tx4939_irq_irc_shutdown(unsigned int irq)
{
	tx4939_irq_irc_disable(irq);
}

static void tx4939_irq_irc_enable(unsigned int irq)
{
	unsigned long flags;
	spin_lock_irqsave(&tx4939_irc_lock, flags);
	tx4939_irq_set_irc_lvl(irq - TX4939_IRQ_IRC_BEG, 0x7);
	spin_unlock_irqrestore(&tx4939_irc_lock, flags);
}

static void tx4939_irq_irc_disable(unsigned int irq)
{
	unsigned long flags;
	spin_lock_irqsave(&tx4939_irc_lock, flags);
	tx4939_irq_set_irc_lvl(irq - TX4939_IRQ_IRC_BEG, 0x0);
	spin_unlock_irqrestore(&tx4939_irc_lock, flags);
}

static void tx4939_irq_irc_mask_and_ack(unsigned int irq)
{
	tx4939_irq_irc_disable(irq);
}

static void tx4939_irq_irc_end(unsigned int irq)
{
	if (irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS))
		return;

	tx4939_irq_irc_enable(irq);
}

/***************************************************
 * Main init functions
 ***************************************************/

/**
 * tx4939_irq_nested - return irq number on TX4939 IRC
 *
 * This function give the irq number from IRCS.CAUSE register.
 */

int tx4939_irq_nested(void)
{
	int irq, ircs;

	ircs = reg_rd32(&tx4939_ircptr->cs);

	if (ircs & TX4939_IRCS_IF_MASK)
		return 0;

	irq = (ircs & TX4939_IRCS_CAUSE_MASK) + TX4939_IRQ_IRC_BEG;
#if defined(CONFIG_TOSHIBA_RBTX4939)
	if (irq == RBTX4939_IRQ_IOC)
		irq = rbtx4939_irq_nested();
#endif
	return irq;
}

/**
 * tx4939_irq_init - tx4939 interrupt setup routine
 *
 * This function runs tx4939_irq_cp0_init and tx4939_irq_irc_init
 * routines. And sets a vector for tx4939 interrupt exception.
 */

static void __init tx4939_irq_init(void)
{
	tx4939_irq_cp0_init();
	tx4939_irq_irc_init();
	set_except_vector(0, tx4939_irq_handler);
}

void tx4939_irc_set_irdm(unsigned int ic, u32 isc)
{
	volatile u32 *irdm;
	unsigned int offset;
	unsigned long l;

	if (ic & 0x20) {
		if (ic & 0x8)
			irdm = &tx4939_ircptr->dm3;
		else
			irdm = &tx4939_ircptr->dm2;
	} else {
		ic--;
		if (ic & 0x8)
			irdm = &tx4939_ircptr->dm1;
		else
			irdm = &tx4939_ircptr->dm0;
	}

	offset = (ic & 0x10) + ((ic & 0x7) << 1);

	l = reg_rd32(irdm);
	l &= ~(0x3 << offset);
	l |= (isc & 0x3) << offset;
	reg_wr32(irdm, l);
}

/**
 * arch_init_IRQ - Initialize IRQ routines.
 *
 * This function initiate IRQ routines.
 */

void __init arch_init_irq(void)
{
	local_irq_disable();

	/* Now, interrupt control disabled, */
	/* all IRC interrupts are masked, */
	/* all IRC interrupt mode are Low Active. */

	tx4939_irq_init();
#if defined(CONFIG_TOSHIBA_RBTX4939)
	rbtx4939_irq_init();
#endif

#if defined(CONFIG_TX4939_SPI)
	txx9_spi_irqinit(RBTX4939_IRQ_IRC_SPI);
#endif

#if defined(CONFIG_PCI)
	tx4939_pci_setup_irq();
#endif
	wbflush();
}

/**
 * mips_spurious_interrupt - cannot specify the interruption cause
 *
 * This function calls interruption which cannot specify the
 * interruption cause. (Usually not use)
 */

void mips_spurious_interrupt(struct pt_regs *regs)
{
	unsigned long status, cause;
	printk(KERN_WARNING "got spurious interrupt\n");
	status = read_c0_status();
	cause = read_c0_cause();
	printk(KERN_WARNING "interrupt current status register %x\n",
	       reg_rd32(&tx4939_ircptr->cs));
	printk(KERN_WARNING "interrupt pending0 register %x\n",
	       reg_rd32(&tx4939_ircptr->pnd0));
	printk(KERN_WARNING "interrupt pending1 register %x\n",
	       reg_rd32(&tx4939_ircptr->pnd1));
	printk(KERN_WARNING "status %lx cause %lx\n", status, cause);
	printk(KERN_WARNING "epc %lx badvaddr %lx \n", regs->cp0_epc,
		regs->cp0_badvaddr);
}
