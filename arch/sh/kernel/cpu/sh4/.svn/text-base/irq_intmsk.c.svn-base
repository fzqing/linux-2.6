/*
 * linux/arch/sh/kernel/cpu/sh4/irq_intmsk.c
 *
 * Copyright (C) 2004  Takashi Kusuda
 *
 * Interrupt handling for INTMSK based IRQ.
 *
 *  Support for Renesas HD6417760(SH7760).
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/machvec.h>


struct intmsk_data {
	unsigned int addr;	/* address of INTMSKxx */
	unsigned int pos;	/* position of INTMSKxx */
};

#define NR_INTMSK_IRQS	(INTMSK_MAX_IRQ - INTMSK_MIN_IRQ + 1)

static struct intmsk_data intmsk_data[NR_INTMSK_IRQS];

static void enable_intmsk_irq(unsigned int irq);
static void disable_intmsk_irq(unsigned int irq);

/* shutdown is same as "disable" */
#define shutdown_intmsk_irq disable_intmsk_irq

static void mask_and_ack_intmsk(unsigned int);
static void end_intmsk_irq(unsigned int irq);

static unsigned int startup_intmsk_irq(unsigned int irq)
{
	enable_intmsk_irq(irq);
	return 0; /* never anything pending */
}

static struct hw_interrupt_type intmsk_irq_type = {
	"INTMSK-IRQ",
	startup_intmsk_irq,
	shutdown_intmsk_irq,
	enable_intmsk_irq,
	disable_intmsk_irq,
	mask_and_ack_intmsk,
	end_intmsk_irq
};

static void disable_intmsk_irq(unsigned int irq)
{
	unsigned long val, flags;
	unsigned int offset = irq - INTMSK_MIN_IRQ;
	unsigned int addr, mask;

	/* Sanity check */
	if(offset < 0 || offset >= NR_INTMSK_IRQS)
		return;

	mask = 1 << (intmsk_data[offset].pos);
	addr = intmsk_data[offset].addr;

	local_irq_save(flags);
	val = ctrl_inl(addr);
	val |= mask;
	ctrl_outl(val, addr);
	local_irq_restore(flags);
}

static void enable_intmsk_irq(unsigned int irq)
{
	unsigned long val, flags;
	unsigned int offset = irq - INTMSK_MIN_IRQ;
	unsigned int addr, mask_clr;

	/* Sanity check */
	if(offset < 0 || offset >= NR_INTMSK_IRQS)
		return;

	mask_clr = 1 << (intmsk_data[offset].pos);
	addr = (intmsk_data[offset].addr + INTMSKCLR_OFFSET);

	local_irq_save(flags);
	val = ctrl_inl(addr);
	val |= mask_clr;
	ctrl_outl(val, addr);
	local_irq_restore(flags);
}

static void mask_and_ack_intmsk(unsigned int irq)
{
	disable_intmsk_irq(irq);
}

static void end_intmsk_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		enable_intmsk_irq(irq);
}

void make_intmsk_irq(unsigned int irq, unsigned int pri_addr,
		     unsigned int pri_pos, unsigned int priority,
		     unsigned int msk_addr, unsigned int msk_pos)
{
	unsigned long val, flags;
	unsigned int offset = irq - INTMSK_MIN_IRQ;

	/* Sanity check */
	if(offset < 0 || offset >= NR_INTMSK_IRQS)
		return;

	disable_irq_nosync(irq);

	intmsk_data[offset].addr = msk_addr;
	intmsk_data[offset].pos = msk_pos;

	/* Set the priority level */
	local_irq_save(flags);
	val = ctrl_inl(pri_addr);
	val |= priority << (4 * pri_pos);
	ctrl_outl(val, pri_addr);
	local_irq_restore(flags);

	irq_desc[irq].handler=&intmsk_irq_type;
	disable_intmsk_irq(irq);
}

#if defined(SH_INTMSK_INIT_TABLE)
static struct sh_intmsk_irq_info sh_intmsk_regs_init[] __initdata = SH_INTMSK_INIT_TABLE;
#endif

void __init init_intmsk_IRQ(void)
{
#if defined(SH_INTMSK_INIT_TABLE)
	int i;

	for(i = 0; sh_intmsk_regs_init[i].irq != 0; i++){
		make_intmsk_irq(sh_intmsk_regs_init[i].irq,
				sh_intmsk_regs_init[i].pri_addr,
				sh_intmsk_regs_init[i].pri_pos,
				sh_intmsk_regs_init[i].priority,
				sh_intmsk_regs_init[i].mask_addr,
				sh_intmsk_regs_init[i].mask_pos);
	}
#endif
}

