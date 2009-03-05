/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/errno.h>
#include <asm/signal.h>
#include <asm/system.h>
#include <asm/ptrace.h>
#include <asm/mipsregs.h>

#include <asm/rmi/sim.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/pic.h>
#include <asm/rmi/debug.h>

/*
 * These are the routines that handle all the low level interrupt stuff. 
 * Actions handled here are: initialization of the interrupt map, requesting of
 * interrupt lines by handlers, dispatching if interrupts to handlers, probing
 * for interrupt lines 
 */

extern void *ht_config_base;

__u64 phnx_irq_mask;
spinlock_t phnx_pic_lock = SPIN_LOCK_UNLOCKED;

static unsigned int pic_startup(unsigned int irq)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	unsigned long flags;
	phoenix_reg_t reg;


	if (!PIC_IRQ_IS_IRT(irq)) return 1;

	spin_lock_irqsave(&phnx_pic_lock, flags);

	reg = phoenix_read_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE);
	/* By default all the interrupts are initialized as level senstive - 
	 * fix for the PCMCIA flash */
	phoenix_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE, 
					reg | (1<<6)|(1<<30)| (1<<31));
	 

	spin_unlock_irqrestore(&phnx_pic_lock, flags);

	return 0;
}

static void pic_ack(unsigned int irq)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	phoenix_reg_t *pci_mmio = phoenix_io_mmio(PHOENIX_IO_PCIX_OFFSET);
	phoenix_reg_t *ht_mmio  = phoenix_io_mmio(PHOENIX_IO_HT_OFFSET);
	unsigned long i, flags;
	phoenix_reg_t reg;


	if (!PIC_IRQ_IS_IRT(irq)) return;

	/*Interrupt (level sensitive )acknowledge method for the PCMCIA flash */

	if (irq == PIC_PCMCIA_IRQ ) {
		reg = *(unsigned char *)(PIC_PCMCIA_STATUS_1);
		reg = *(unsigned int *)(PIC_PCMCIA_STATUS_2);
		for ( i = 0; i<0x100; i++)
			;
		*(unsigned int *)(PIC_PCMCIA_STATUS_2) = reg ;
		for ( i = 0; i<0x1000; i++)
			;
		reg = *(unsigned int *)(PIC_PCMCIA_STATUS_2);

	}

	/* Deal with All PCI-Interrupts.. Brigde ACK */
	if (irq == PIC_PCIX_IRQ)
		phoenix_read_reg(pci_mmio, (0x140 >> 2));

	if (irq == 23) {
		/* Clear INT Status */
		phoenix_read_reg (ht_mmio, (0x700 >> 2)); 
        
		/* 
		   Generating EOI.
		   On B0 Silicon, the EOI broadcast don't work. Hence, we clear the
		   interrupt by directly writing to PLX's Configuration Space. For
		   that, first we setup the offset value in register 0xB8
		   (Interrupt Discovery Configuration, bits 23-16). Then we
		   clear the interrupt by setting the IRR bit (bit 63) in register
		   0xBC (IRDR). 

		   Note that, this technique works only because we have one device
		   on the HT (PLX bridge). If more devices are added to HT, we have
		   to use the EOI broadcast.

		   NOTE: We need to do the EOI for all interrupts (INT A, B, C and D).
		   Bridge Cards, if plugged into the slot, may re-route interrupts.
		   e.g. Intel Bridge 31154 eval board re-routes INTA of the endpoint
		   to INTC of PLX.
	  
		   Since we only have PLX in our HT, the PLX's bus/device/function
		   value is 0/1/0. Hence, its configuration space address is 
	            
		   ht_config_base + (device <<11) + cfg_offset.

		   Since CPU is Big Endian, we need to do swapping as well.
		   For e.g. to generate EOI for INTA, the config space address is 
		   ht_config_base + SWAP32((1 << 11) + 0xb8) = ht_config_base + 0x8b8.
		   and the offset value is SWAP32(0x8011c008).
		   See PLX manual v1.0 pp 3-19 for more details on how to derive
		   the offset value. And to clear the interrupt, write SWAP32(0x8000000)
		   to (ht_config_base + SWAP32((1<<11) + 0xbc).

		   TODO: With B1 silicon, the EOI broadcast should work. Should thrash
		   this story and the code then.
		*/

		/* Generate EOI for INTA */
		*(unsigned long *)(ht_config_base + 0x008b8) = 0x08c01180; 
		*(unsigned long *)(ht_config_base + 0x008bc) = 0x00000080;

		/* Generate EOI for INTB */
		*(unsigned long *)(ht_config_base + 0x008b8) = 0x08c01380; 
		*(unsigned long *)(ht_config_base + 0x008bc) = 0x00000080;

		/* Generate EOI for INTC */
		*(unsigned long *)(ht_config_base + 0x008b8) = 0x08c01580; 
		*(unsigned long *)(ht_config_base + 0x008bc) = 0x00000080;

		/* Generate EOI for INTD */
		*(unsigned long *)(ht_config_base + 0x008b8) = 0x08c01780; 
		*(unsigned long *)(ht_config_base + 0x008bc) = 0x00000080;
	}


	/* If edge triggered IRQ, ack it immediately, else when the device
	 * interrupt condition is cleared, we may lose interrupts 
	 */
	if (PIC_IRQ_IS_EDGE_TRIGGERED(irq)) {
		spin_lock_irqsave(&phnx_pic_lock, flags);
		phoenix_write_reg(mmio, PIC_INT_ACK, 
					(1 << (irq - PIC_IRQ_BASE)));
		spin_unlock_irqrestore(&phnx_pic_lock, flags);
	}
}

static void pic_end(unsigned int irq)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	unsigned long flags;


	if (!PIC_IRQ_IS_IRT(irq)) return;

	/* If level triggered, ack it after the device condition is cleared */
	if (! PIC_IRQ_IS_EDGE_TRIGGERED(irq)) {
		spin_lock_irqsave(&phnx_pic_lock, flags);
		phoenix_write_reg(mmio, PIC_INT_ACK, 
						(1 << (irq - PIC_IRQ_BASE)));
		spin_unlock_irqrestore(&phnx_pic_lock, flags);
	}
}

static void pic_shutdown(unsigned int irq)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	unsigned long flags;
	phoenix_reg_t reg;


	if (!PIC_IRQ_IS_IRT(irq)) return;

	spin_lock_irqsave(&phnx_pic_lock, flags);
	  
	reg = phoenix_read_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE);
	phoenix_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE, 
					(reg & ~(1<<31)));
	  
	spin_unlock_irqrestore(&phnx_pic_lock, flags);
}

static void pic_set_affinity(unsigned int irq, cpumask_t dest)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	unsigned long flags;
	unsigned long mask = dest.bits[0];


	if (!PIC_IRQ_IS_IRT(irq)) return;

	spin_lock_irqsave(&phnx_pic_lock, flags);

	phoenix_write_reg(mmio, PIC_IRT_0_BASE + irq - PIC_IRQ_BASE, mask);

	spin_unlock_irqrestore(&phnx_pic_lock, flags);
}

static struct hw_interrupt_type phnx_pic = {
  .typename     =          "Phoenix-PIC",
  .startup      =          pic_startup,
  .shutdown     =          pic_shutdown,
  .enable       =          (void (*)(unsigned int))pic_startup,
  .disable      =          pic_shutdown,
  .ack          =          pic_ack,
  .end          =          pic_end,
  .set_affinity =          pic_set_affinity
};

static unsigned int rsvd_pic_handler_1_1(unsigned int irq)
{
	rmi_dbg_msg("Requesting a reserved irq (%d)??", irq);
	return -EBUSY;
}

static void rsvd_pic_handler_1(unsigned int irq)
{
	rmi_dbg_msg("handler called for a reserved irq (%d)\n", irq);
}

static void rsvd_pic_handler_2(unsigned int irq, cpumask_t mask)
{
	rmi_dbg_msg("handler called for a reserved irq (%d)\n", irq);
}

static struct hw_interrupt_type phnx_rsvd_pic = {
  .typename     =          "Phoenix-RSVD-PIC",
  .startup      =          rsvd_pic_handler_1_1,
  .shutdown     =          rsvd_pic_handler_1,
  .enable       =          rsvd_pic_handler_1,
  .disable      =          rsvd_pic_handler_1,
  .ack          =          rsvd_pic_handler_1,
  .end          =          rsvd_pic_handler_1,
  .set_affinity =          rsvd_pic_handler_2
};

static irqreturn_t phnx_rsvd_irq_handler(int irq, void *dev_id, 
			struct pt_regs *regs)
{
	rmi_dbg_msg("handler for reserved irq %d\n", irq);
	return IRQ_NONE;
}

static struct irqaction phnx_rsvd_action = {
  .handler = phnx_rsvd_irq_handler,
  .flags = 0,
  .mask = CPU_MASK_NONE,
  .name = "phnx_rsvd_action", 
  .dev_id = 0,
  .next = 0
};

void __init init_phoenix_irqs(void)
{
	int i;

	for (i = 0; i < NR_IRQS; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = 0,
		irq_desc[i].depth = 1;
		irq_desc[i].handler = &phnx_pic;
	}

#ifdef CONFIG_SMP
	irq_desc[IRQ_IPI_SMP_FUNCTION].handler = &phnx_rsvd_pic;
	irq_desc[IRQ_IPI_SMP_FUNCTION].action = &phnx_rsvd_action;

	irq_desc[IRQ_IPI_SMP_RESCHEDULE].handler = &phnx_rsvd_pic;
	irq_desc[IRQ_IPI_SMP_RESCHEDULE].action = &phnx_rsvd_action;

	phnx_irq_mask |= 
		((1ULL<<IRQ_IPI_SMP_FUNCTION)|(1ULL<<IRQ_IPI_SMP_RESCHEDULE));

#ifdef CONFIG_CPU_TIMER
	 phnx_irq_mask |= ((1ULL<<IRQ_TIMER));
#endif

#endif

	/* msgring interrupt */
	irq_desc[IRQ_MSGRING].handler = &phnx_rsvd_pic;
	irq_desc[IRQ_MSGRING].action = &phnx_rsvd_action;
	phnx_irq_mask |= (1ULL<<IRQ_MSGRING);

	/* msgring timeout interrupt */
	irq_desc[PIC_TIMER_0_IRQ].handler = &phnx_rsvd_pic;
	irq_desc[PIC_TIMER_0_IRQ].action = &phnx_rsvd_action;
	phnx_irq_mask |= (1ULL<<PIC_TIMER_0_IRQ);

	/* timekeeping timer interrupt for cpu 0 */
	irq_desc[PIC_TIMER_7_IRQ].handler = &phnx_rsvd_pic;
	irq_desc[PIC_TIMER_7_IRQ].action = &phnx_rsvd_action;
	phnx_irq_mask |= (1ULL<<PIC_TIMER_7_IRQ);

	/* profiling/process accounting timer interrupt for non-zero cpus */
	irq_desc[IRQ_TIMER].handler = &phnx_rsvd_pic;
	irq_desc[IRQ_TIMER].action = &phnx_rsvd_action;

	/* The driver installs a handler so, don't reserve it */
	phnx_irq_mask |= (1ULL<<IRQ_DUMMY_UART);

	/* unmask all PIC related interrupts. If no handler is installed by the 
	 * drivers, it'll just ack the interrupt and return 
	 */
	for(i=PIC_IRT_FIRST_IRQ;i<=PIC_IRT_LAST_IRQ;i++) 
		phnx_irq_mask |= (1ULL << i);

#ifdef CONFIG_CPU_TIMER
	change_c0_status(ST0_IM, phnx_irq_mask);
#endif	
}
 
extern void phoenix_timer_interrupt(struct pt_regs *regs, int irq);
void do_phnx_IRQ(unsigned int irq, struct pt_regs *regs)
{
	/* first process timer interrupts */
	if (irq == IRQ_TIMER || irq == PIC_TIMER_7_IRQ) {
		phoenix_timer_interrupt(regs, irq);
		return;
	}

#ifdef CONFIG_SMP
	if  (irq == IRQ_IPI_SMP_FUNCTION || irq == IRQ_IPI_SMP_RESCHEDULE) {
		phoenix_ipi_handler(irq, regs);
		return;
	}
#endif
	if (irq == IRQ_MSGRING || irq == PIC_TIMER_0_IRQ) 
		phnx_msgring_int_handler(irq, regs);
	else 
		do_IRQ(irq, regs);
}

extern void phoenix_smp_time_init(void);
void __init phoenix_smp_init(void)
{
	/* Set up kseg0 to be cachable coherent */
	change_c0_config(CONF_CM_CMASK, CONF_CM_DEFAULT);

	/* set interrupt mask for non-zero cpus */
	write_64bit_cp0_eimr(phnx_irq_mask | (1<<IRQ_TIMER));

	phoenix_smp_time_init();
}

void __init arch_init_irq(void)
{
	/* Defined in arch/mips/rmi/phoenix/irq_handler.S */
	extern void phoenix_irq_handler(void);

	/* TODO:
	 * Initialize the irq registers on the PIC to route
	 * interrupts to appropriate pins
	 */
	
	/* Initialize the irq descriptors */
	init_phoenix_irqs();

	write_64bit_cp0_eimr(phnx_irq_mask);

	set_except_vector(0, phoenix_irq_handler);

}

