/*
 * arch/mps/tx4939/rbtx4939/irq.c
 *
 * RBTX4939 irq handler
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
#include <asm/tx4939/tx4939.h>
#include <asm/tx4939/rbtx4939.h>

extern void tx4939_irc_set_irdm(unsigned int ic, u32 isc);

static unsigned int rbtx4939_irq_ioc_startup(unsigned int irq);
static void rbtx4939_irq_ioc_shutdown(unsigned int irq);
static void rbtx4939_irq_ioc_enable(unsigned int irq);
static void rbtx4939_irq_ioc_disable(unsigned int irq);
static void rbtx4939_irq_ioc_mask_and_ack(unsigned int irq);
static void rbtx4939_irq_ioc_end(unsigned int irq);

static raw_spinlock_t rbtx4939_ioc_lock = RAW_SPIN_LOCK_UNLOCKED;

#define RBTX4939_IOC_NAME "RBTX4939-IOC"
static struct hw_interrupt_type rbtx4939_irq_ioc_type = {
	.typename = RBTX4939_IOC_NAME,
	.startup = rbtx4939_irq_ioc_startup,
	.shutdown = rbtx4939_irq_ioc_shutdown,
	.enable = rbtx4939_irq_ioc_enable,
	.disable = rbtx4939_irq_ioc_disable,
	.ack = rbtx4939_irq_ioc_mask_and_ack,
	.end = rbtx4939_irq_ioc_end,
	.set_affinity = NULL
};


static struct irqaction rbtx4939_irq_ioc_action = {
	.handler = no_action,
	.name = RBTX4939_IOC_NAME
};

int rbtx4939_irq_nested(void)
{
	int irq;
	u8 intf2;

	intf2 = reg_rd08(rbtx4939_intf2_ptr);
	if (!intf2) {
		printk(KERN_ERR "%s(%03d): why ioc invoked?\n", __FUNCTION__, __LINE__);
		return 0;
	}

	irq = RBTX4939_IRQ_IOC_BEG + fls(intf2) - 1;

	return irq;
}

/***************************************************
 * Functions for ioc
 ***************************************************/

static void rbtx4939_irq_ioc_mask_modify(unsigned bit, int irq)
{
	u8 c;
	unsigned int offset = (irq - RBTX4939_IRQ_IOC_BEG);

	c = reg_rd08(rbtx4939_inte_ptr);
	reg_wr08(rbtx4939_intf1_ptr, c & ~(0x1 << offset));
	c = reg_rd08(rbtx4939_inte_ptr);
	reg_wr08(rbtx4939_intf1_ptr, c | (bit << offset));

	/* prevent spurious interrupt */
	__asm__ __volatile__("sync\n\t");
	reg_rd08(rbtx4939_inte_ptr);
}

/**
 * rbtx4939_irq_ioc_init -
 *
 *
 */

static void __init rbtx4939_irq_ioc_init(void)
{
	int i;

	for (i = RBTX4939_IRQ_IOC_BEG; i <= RBTX4939_IRQ_IOC_END; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = 0;
		irq_desc[i].depth = 3;
		irq_desc[i].handler = &rbtx4939_irq_ioc_type;
	}

	setup_irq(RBTX4939_IRQ_IOC, &rbtx4939_irq_ioc_action);
}

static unsigned int rbtx4939_irq_ioc_startup(unsigned int irq)
{
	rbtx4939_irq_ioc_enable(irq);
	return 0;
}

static void rbtx4939_irq_ioc_shutdown(unsigned int irq)
{
	rbtx4939_irq_ioc_disable(irq);
}

static void rbtx4939_irq_ioc_enable(unsigned int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&rbtx4939_ioc_lock, flags);
	rbtx4939_irq_ioc_mask_modify(0x1, irq);
	spin_unlock_irqrestore(&rbtx4939_ioc_lock, flags);
}

static void rbtx4939_irq_ioc_disable(unsigned int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&rbtx4939_ioc_lock, flags);
	rbtx4939_irq_ioc_mask_modify(0x0, irq);
	spin_unlock_irqrestore(&rbtx4939_ioc_lock, flags);
}

static void rbtx4939_irq_ioc_mask_and_ack(unsigned int irq)
{
	rbtx4939_irq_ioc_disable(irq);
}

static void rbtx4939_irq_ioc_end(unsigned int irq)
{
	if (irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS))
		return;
	rbtx4939_irq_ioc_enable(irq);
}

/**
 * rbtx4939_irq_init -
 *
 *
 */

void __init rbtx4939_irq_init(void)
{
	/* mask all IOC interrupts */
	/* clear SoftInt interrupts */

	/* Onboard 10M Ether: High Active */
	tx4939_irc_set_irdm(RBTX4939_IRQ_DEBUG_ETHER - TX4939_IRQ_IRC_BEG,
			    TX4939_IRDM_LOW_LEVEL);

	rbtx4939_irq_ioc_init();
}
