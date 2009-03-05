/*
 *  arch/sh/kernel/cpu/sh4/irq_sh4a_intc.c
 *
 *  Copyright (C)  Takashi Kusuda (Nov. 2004)
 *
 *   SH-4A INTC/INTC2 interrupt handler.
 *   Support CPUs are SH7780 and SH7781.
 *
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/machvec.h>

/* for initializing SH7780 INTC/INTC2 regs */
struct sh4a_intc_irq_info
{
	unsigned int irq;
	unsigned int mask_addr;
	unsigned int unmask_addr;
	int  pos;
};

struct sh4a_intc_sys {
	unsigned long mask_addr;
	unsigned long unmask_addr;
	unsigned long pos;
};

static struct sh4a_intc_sys sh4a_intc_data[NR_IRQS];

static void enable_sh4a_intc_irq(unsigned int irq);
static void disable_sh4a_intc_irq(unsigned int irq);

/* shutdown is same as "disable" */
#define shutdown_sh4a_intc_irq disable_sh4a_intc_irq

static void mask_and_ack_sh4a_intc(unsigned int);
static void end_sh4a_intc_irq(unsigned int irq);

static unsigned int startup_sh4a_intc_irq(unsigned int irq)
{
	enable_sh4a_intc_irq(irq);
	return 0; /* never anything pending */
}

static struct hw_interrupt_type sh4a_intc_irq_type = {
	"SH4A INTC/INTC2-IRQ",
	startup_sh4a_intc_irq,
	shutdown_sh4a_intc_irq,
	enable_sh4a_intc_irq,
	disable_sh4a_intc_irq,
	mask_and_ack_sh4a_intc,
	end_sh4a_intc_irq
};

static void disable_sh4a_intc_irq(unsigned int irq)
{
	unsigned long flags;

        local_irq_save(flags);
	*(volatile unsigned long *)(sh4a_intc_data[irq].mask_addr) = (1 << sh4a_intc_data[irq].pos);
        local_irq_restore(flags);
}

static void enable_sh4a_intc_irq(unsigned int irq)
{
	unsigned long flags;

	local_irq_save(flags);
	*(volatile unsigned long *)(sh4a_intc_data[irq].unmask_addr) = (1 << sh4a_intc_data[irq].pos);
	local_irq_restore(flags);
}

static void mask_and_ack_sh4a_intc(unsigned int irq)
{
	disable_sh4a_intc_irq(irq);
}

static void end_sh4a_intc_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		enable_sh4a_intc_irq(irq);
}

void make_sh4a_intc_irq(unsigned int irq, unsigned int mask_addr,
			  unsigned int unmask_addr, int pos)
{
	disable_irq_nosync(irq);
	sh4a_intc_data[irq].mask_addr = mask_addr;
	sh4a_intc_data[irq].unmask_addr = unmask_addr;
	sh4a_intc_data[irq].pos = pos;

	irq_desc[irq].handler = &sh4a_intc_irq_type;
	disable_sh4a_intc_irq(irq);
}

#if defined(SH4A_INTC2_INIT_TABLE)
static struct sh4a_intc_irq_info sh4a_intc2_regs_init[] __initdata = SH4A_INTC2_INIT_TABLE;
#endif

void __init init_sh4a_intc2_IRQ(void)
{
	int i;

	/*
	 *  Initialize INTC2 (These are for internal modules).
	 *  IRL[0:7](IRQ0-7) depends on Platform.
	 *  So, these INTC setting on arch/sh/boards/xx/xx/irq.c
	 */

	/* setting INTC2 interrupt priority */
#if defined(SH4A_INTC2_PRIORITY_SETTING)
	SH4A_INTC2_PRIORITY_SETTING;
#endif

	/* register INTC2 IRQs */
#if defined(SH4A_INTC2_INIT_TABLE)
	for(i = 0; sh4a_intc2_regs_init[i].irq != 0; i++){
		make_sh4a_intc_irq(sh4a_intc2_regs_init[i].irq,
				   sh4a_intc2_regs_init[i].mask_addr,
				   sh4a_intc2_regs_init[i].unmask_addr,
				   sh4a_intc2_regs_init[i].pos);
	}
#endif
}

EXPORT_SYMBOL(make_sh4a_intc_irq);
