/*
 * (C) Copyright 2005 Tundra Semiconductor Corp.
 * Alex Bounine, <alexandreb@tundra.com).
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 *  arch/ppc/syslib/tsi108_pic.c - Tsi108 Interrupt Controller Handling
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <asm/ptrace.h>
#include <asm/signal.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/prom.h>
#include <asm/sections.h>
#include <asm/hardirq.h>
#include <asm/machdep.h>

#include <asm/tsi108.h>
#include <asm/tsi108_irq.h>
#include "tsi108_pic.h"

extern u32 tsi108_csr_base;

static int tsi108_pic_cascade_irq = -1;
static int (*tsi108_pic_cascade_fn) (struct pt_regs *);

/* Global Operations */
static void tsi108_pic_set_task_priority(u_int pri);
static void tsi108_pic_set_spurious(u_int vector);
void tsi108_pic_mask_all(void);

/* Timer Interrupts */
static void tsi108_pic_inittimer(u_int timer, u_int pri, u_int vector);
static void tsi108_pic_maptimer(u_int timer, u_int cpumask);

/* Interrupt Sources */
static void tsi108_pic_enable_irq(u_int irq);
static void tsi108_pic_disable_irq(u_int irq);
static void tsi108_pic_initirq(u_int irq, u_int pri, u_int vector, int polarity,
			       int is_level);
static void tsi108_pic_mapirq(u_int irq, u_int cpumask, u_int keepmask);
static void init_pci_source(void);
static inline int get_pci_source(int vector);
int tsi108_pic_set_irq_sense(int irq, int pol, int sense);

/*
 * tsi108_pic interface routines
 */
static void tsi108_pic_end_irq(unsigned int irq_nr);
static void tsi108_pic_ack_irq(unsigned int irq_nr);
void tsi108_pic_set_affinity(unsigned int irq_nr, unsigned long cpumask);

static struct hw_interrupt_type tsi108_pic_irq = {
	"tsi108_pic",
	NULL,
	NULL,
	tsi108_pic_enable_irq,
	tsi108_pic_disable_irq,
	tsi108_pic_ack_irq,
	tsi108_pic_end_irq,
	NULL
};

static void tsi108_pci_irq_enable(u_int irq);
static void tsi108_pci_irq_disable(u_int irq);
static void tsi108_pci_irq_ack(u_int irq);
static void tsi108_pci_irq_end(u_int irq);

static struct hw_interrupt_type tsi108_pci_irq = {
	"tsi108_PCI_int",
	NULL,
	NULL,
	tsi108_pci_irq_enable,
	tsi108_pci_irq_disable,
	tsi108_pci_irq_ack,
	tsi108_pci_irq_end,
	NULL
};

#ifdef DBG_TSI108_INTERRUPT
#define ASSERT(expr)	if (!(expr)) { \
				printk("tsi108pic :" \
					"assertion failed! %s[%d]: %s\n", \
					__FUNCTION__, __LINE__, #expr); \
				dump_stack(); \
			}
#else
#define ASSERT(expr)	do {} while (0)
#endif

static inline u_int get_vector_offset(u_int irq)
{
	u_int offset;

	if (irq < TSI108_IRQ_BASE || irq >= IRQ_PCI_INTAD_BASE)
		return 0;

	if (irq < IRQ_TSI108_MBOX0)
		offset = TSI108_INT_IVPR(irq - TSI108_IRQ_BASE);
	else if (irq < IRQ_TSI108_DBELL0)
		offset = TSI108_INT_MBVPR(irq - IRQ_TSI108_MBOX0);
	else if (irq < IRQ_TSI108_TIMER0)
		offset = TSI108_INT_DVPR(irq - IRQ_TSI108_DBELL0);
	else
		offset = TSI108_INT_GTVPR(irq - IRQ_TSI108_TIMER0);

	return offset;
}

static inline u_int tsi108_pic_read_reg(u_int reg_offset)
{
	return
	    in_be32((volatile u32 *)(tsi108_csr_base + TSI108_MPIC_OFFSET +
				     reg_offset));
}

static inline void tsi108_pic_write_reg(u_int reg_offset, u_int val)
{
	out_be32((volatile u32 *)(tsi108_csr_base + TSI108_MPIC_OFFSET +
				  reg_offset), val);
}

void tsi108_pic_reset(void)
{
	tsi108_pic_write_reg(TSI108_INT_GCR, TSI108PIC_INT_GCR_R);
	while (tsi108_pic_read_reg(TSI108_INT_GCR) & TSI108PIC_INT_GCR_R)
		mb();
}

void tsi108_pic_set_output(int dest_num, u32 sense, u32 polarity)
{
	u32 temp = 0;
	temp |= (IRQ_SENSE_LEVEL == sense) ?
	    (TSI108PIC_INT_CSR_S_LEVEL) : (TSI108PIC_INT_CSR_S_EDGE);
	temp |= (IRQ_POLARITY_POSITIVE == polarity) ?
	    (TSI108PIC_INT_CSR_P_HIGH) : (TSI108PIC_INT_CSR_P_LOW);
	tsi108_pic_write_reg(TSI108_INT_CSR(dest_num), temp);
	mb();
}

int tsi108_pic_source_cfg(int src_num,	/* interrupt source number */
			  u32 sense,	/* interrupt source Sense */
			  u32 polarity,	/* interrupt source Polarity */
			  TSI108_IRQ_MODE mode	/* interrupt delivery Mode */
    )
{
	unsigned temp;

	temp = tsi108_pic_read_reg(TSI108_INT_IVPR(src_num));

	if (temp & TSI108PIC_ACTIVITY)	/* error if source is active */
		return -1;

	if (0 == (temp & TSI108PIC_MASK)) {
		temp |= TSI108PIC_MASK;	/* mask IRQ prior making changes */
		tsi108_pic_write_reg(TSI108_INT_IVPR(src_num), temp);
	}

	temp &= ~(TSI108PIC_INT_IVPR_MODE |
		  TSI108PIC_INT_IVPR_S | TSI108PIC_INT_IVPR_P);

	temp |= (IRQ_SENSE_LEVEL == sense) ?
	    (TSI108PIC_INT_CSR_S_LEVEL) : (TSI108PIC_INT_CSR_S_EDGE);
	temp |= (IRQ_POLARITY_POSITIVE == polarity) ?
	    (TSI108PIC_INT_CSR_P_HIGH) : (TSI108PIC_INT_CSR_P_LOW);

	tsi108_pic_write_reg(TSI108_INT_IVPR(src_num),
			     TSI108PIC_MASK | (mode << 29) | temp);
	return (0);
}

int tsi108_pic_set_vector(int src_num,	/* source number */
			  int vect,	/* vector number */
			  int prio	/* interrupt source priority */
    )
{
	unsigned tmp;

	tmp = tsi108_pic_read_reg(TSI108_INT_IVPR(src_num));

	if (tmp & TSI108PIC_ACTIVITY)	/* error if source is active */
		return -1;

	if (0 == (tmp & TSI108PIC_MASK)) {
		tmp |= TSI108PIC_MASK;	/* mask IRQ prior making changes */
		tsi108_pic_write_reg(TSI108_INT_IVPR(src_num), tmp);
	}

	/* clear bits to be changed */
	tmp &= ~(TSI108PIC_VECTOR_MASK | TSI108PIC_PRIORITY_MASK);

	tmp |= (prio << 16) | vect;
	tsi108_pic_write_reg(TSI108_INT_IVPR(src_num), tmp);
	return 0;
}

void tsi108_pic_mask_all()
{
	int i;
	unsigned int vp;

	/* Mask all external and internal interrupt sources */
	for (i = 0; i < TSI108PIC_MAX_SOURCES; i++) {
		vp = tsi108_pic_read_reg(TSI108_INT_IVPR(i));
		tsi108_pic_write_reg(TSI108_INT_IVPR(i), vp | TSI108PIC_MASK);
		mb();

		/* Make sure that irq is masked */
		do {
			vp = tsi108_pic_read_reg(TSI108_INT_IVPR(i));
		} while ((vp & TSI108PIC_ACTIVITY) && !(vp & TSI108PIC_MASK));
	}

	/* Mask all timer interrupts */
	for (i = 0; i < TSI108PIC_NUM_TIMERS; i++) {
		vp = tsi108_pic_read_reg(TSI108_INT_GTVPR(i));
		tsi108_pic_write_reg(TSI108_INT_GTVPR(i), vp | TSI108PIC_MASK);
		mb();

		do {
			vp = tsi108_pic_read_reg(TSI108_INT_GTVPR(i));
		} while ((vp & TSI108PIC_ACTIVITY) && !(vp & TSI108PIC_MASK));
	}

	/* Mask all doorbell interrupts */
	for (i = 0; i < TSI108PIC_NUM_DBELLS; i++) {
		vp = tsi108_pic_read_reg(TSI108_INT_DVPR(i));
		tsi108_pic_write_reg(TSI108_INT_IVPR(i), vp | TSI108PIC_MASK);
		mb();

		do {
			vp = tsi108_pic_read_reg(TSI108_INT_DVPR(i));
		} while ((vp & TSI108PIC_ACTIVITY) && !(vp & TSI108PIC_MASK));
	}

	/* Mask all mailbox interrupts */
	for (i = 0; i < 4; i++) {
		vp = tsi108_pic_read_reg(TSI108_INT_MBVPR(i));
		tsi108_pic_write_reg(TSI108_INT_MBVPR(i), vp | TSI108PIC_MASK);
		mb();

		do {
			vp = tsi108_pic_read_reg(TSI108_INT_MBVPR(i));
		} while ((vp & TSI108PIC_ACTIVITY) && !(vp & TSI108PIC_MASK));
	}
}

/*
 * The Tsi108 PC initialization routine.
 * A caller routine (usually from platform-specific code has to provide
 * sense/polarity configuration information for four external interrupt
 * sources INT0 - INT3. This should be done in form of four-byte array
 * (one byte per source ) that contains combination of sensitivity/polarity
 * flags defined in asm-ppc/irq.h.
 *
 * Example of PIC initialization call is shown below:
 *
 *   u_char your_board_pic_initsenses[] __initdata = {
 *	    (IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),  // INT[0]
 *	    (IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),  // INT[1]
 *	    (IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),  // INT[2]
 *	    (IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE)   // INT[3]
 *          };
 *
 * tsi108_pic_init(your_board_pic_initsenses);
 */

void __init tsi108_pic_init(u_char * board_init_senses)
{
	u_int i;
	u32 sense;

	if (!tsi108_csr_base) {
		printk("No tsi108 PIC found !\n");
		return;
	}

	tsi108_pic_reset();

	if (ppc_md.progress)
		ppc_md.progress("tsi108_pic_init: enter", 0x122);

	/* Initialize timer interrupts */
	for (i = 0; i < TSI108PIC_NUM_TIMERS; i++) {
		/* Disabled, Priority 0 */
		tsi108_pic_inittimer(i, 0, IRQ_TSI108_TIMER0 + i);
		/* No processor */
		tsi108_pic_maptimer(i, 0);
	}

	/* Init board-specific external sources */
	for (i = 0; i < 4; i++) {
		sense = board_init_senses[i];

		if (sense & IRQ_SENSE_MASK)
			irq_desc[TSI108_IRQ(i)].status |= IRQ_LEVEL;

		/* Enabled, Priority 8 */
		tsi108_pic_initirq(i, 8, TSI108_IRQ(i),
				   (sense & IRQ_POLARITY_MASK),
				   (sense & IRQ_SENSE_MASK));
		/* Map to CPU #0 */
		tsi108_pic_mapirq(TSI108_IRQ(i), 1 << 0, 0);
	}

	/* Init remaining internal sources. */
	for (; i < TSI108PIC_MAX_SOURCES; i++) {
		/* Disabled, Priority 8, by default - Positive Edge */
		tsi108_pic_initirq(i, 8, TSI108_IRQ(i),
				   IRQ_POLARITY_POSITIVE, IRQ_SENSE_EDGE);
		/* Map to CPU #0 */
		tsi108_pic_mapirq(TSI108_IRQ(i), (1 << 0), 0);
	}

	/*
	 * Change sensitivity to level for sources that require it.
	 */

	irq_desc[IRQ_TSI108_GIGE0].status |= IRQ_LEVEL;
	irq_desc[IRQ_TSI108_GIGE1].status |= IRQ_LEVEL;
	irq_desc[IRQ_TSI108_PCI].status |= IRQ_LEVEL;

	/* Init descriptors */
	for (i = 0; i < TSI108PIC_MAX_SOURCES; i++)
		irq_desc[i + TSI108_IRQ_BASE].handler = &tsi108_pic_irq;

	for (i = 0; i < NUM_PCI_IRQS; i++) {
		irq_desc[i + IRQ_PCI_INTAD_BASE].handler = &tsi108_pci_irq;
		irq_desc[i + IRQ_PCI_INTAD_BASE].status |= IRQ_LEVEL;
	}

	/* Initialize the spurious interrupt */
	tsi108_pic_set_spurious(TSI108_IRQ_SPURIOUS);
	tsi108_pic_set_task_priority(0);

	init_pci_source();
	tsi108_pic_enable_irq(IRQ_TSI108_PCI);

	i = tsi108_pic_read_reg(TSI108_INT_VECTOR(0));
	tsi108_pic_write_reg(TSI108_INT_EOI(0), 0);

	if (ppc_md.progress)
		ppc_md.progress("tsi108_pic_init: exit", 0x222);
}

/*
 *  Find out the current interrupt
 */
static u_int tsi108_pic_get_vect(void)
{
	u_int vec;

	vec = tsi108_pic_read_reg(TSI108_INT_VECTOR(0)) & TSI108PIC_VECTOR_MASK;

#ifdef DBG_TSI108_INTERRUPT
	if (vec == TSI108_IRQ_SPURIOUS)
		printk("TSI108: SPURIOUS vec=0x%08x\n", vec);
	else
		printk("TSI108: read vec=0x%08x\n", vec);
#endif
	return (vec);
}

static inline void tsi108_pic_eoi(void)
{
	tsi108_pic_write_reg(TSI108_INT_EOI(0), 0);
	mb();
}

static void __init tsi108_pic_set_task_priority(u_int pri)
{
	ASSERT(pri >= 0 && pri < TSI108PIC_NUM_PRI);

	tsi108_pic_write_reg(TSI108_INT_TASKP(0),
			     pri & TSI108PIC_INT_TASKP_TASKP);
	mb();
}

static void tsi108_pic_set_spurious(u_int vec)
{
	ASSERT(vec == TSI108_IRQ_SPURIOUS);
	tsi108_pic_write_reg(TSI108_INT_SVR, vec);
	mb();
}

#ifdef CONFIG_SMP
/*
 * Convert a cpu mask from logical to physical cpu numbers.
 */
static inline u32 physmask(u32 cpumask)
{
	int i;
	u32 mask = 0;

	for (i = 0; i < NR_CPUS; ++i, cpumask >>= 1)
		if (cpu_online(i))
			mask |= (cpumask & 1) << smp_hw_index[i];
	return mask;
}
#else
#define physmask(cpumask)	(cpumask)
#endif

/*
 *  Initialize a timer interrupt (and disable it)
 *
 *  timer: timer number
 *  pri:   interrupt source priority
 *  vec:   the vector it will produce
 */
static void __init tsi108_pic_inittimer(u_int timer, u_int pri, u_int vec)
{
	unsigned int gtvpr;

	ASSERT(timer >= 0 && timer < TSI108PIC_NUM_TIMERS);
	ASSERT(pri >= 0 && pri < TSI108PIC_NUM_PRI);
	ASSERT(vec >= 0 && vec < TSI108PIC_NUM_VECTORS);

	gtvpr = tsi108_pic_read_reg(TSI108_INT_GTVPR(timer));
	gtvpr &= ~(TSI108PIC_PRIORITY_MASK | TSI108PIC_VECTOR_MASK);
	gtvpr |= (pri << 16) | vec;
	tsi108_pic_write_reg(TSI108_INT_GTVPR(timer), gtvpr | TSI108PIC_MASK);
	mb();
}

/*
 *  Map a timer interrupt to one or more CPUs
 */
static void __init tsi108_pic_maptimer(u_int timer, u_int cpumask)
{
	ASSERT(timer >= 0 && timer < TSI108PIC_NUM_TIMERS);

	tsi108_pic_write_reg(TSI108_INT_GTDR(timer), physmask(cpumask));
	mb();
}

/*
 * Initalize the interrupt source which will generate an NMI.
 * This raises the interrupt's priority from 8 to 9.
 *
 * irq: The logical IRQ which generates an NMI.
 */
void __init tsi108_pic_init_nmi_irq(u_int irq)
{
	u_int offset = get_vector_offset(irq);
	u_int vpr = tsi108_pic_read_reg(offset);
	vpr &= ~TSI108PIC_PRIORITY_MASK;
	tsi108_pic_write_reg(offset, vpr | (9 << 16));
	mb();
}

/*
 *
 * All functions below take an offset'ed irq argument
 *
 */

/*
 * Hookup a cascade to the tsi108 PIC.
 */
void __init
tsi108_pic_hookup_cascade(u_int irq, char *name,
			  int (*cascade_fn) (struct pt_regs *))
{
	tsi108_pic_cascade_irq = irq;
	tsi108_pic_cascade_fn = cascade_fn;
	if (request_irq(irq, no_action, SA_INTERRUPT, name, NULL))
		printk("Unable to get Tsi108 PIC IRQ %d for cascade\n",
		       irq - TSI108_IRQ_BASE);
}

/*
 *  Enable/disable an external interrupt source
 *
 *  Externally called, irq is an offseted system-wide interrupt number
 */
static void tsi108_pic_enable_irq(u_int irq)
{
	u32 offset = get_vector_offset(irq);
	u32 vpr = tsi108_pic_read_reg(offset);

	/*
	 * Undo sensitivity change (see tsi108_pic_disable_irq())
	 */
	if (irq_desc[irq].status & IRQ_LEVEL)
		vpr |= TSI108PIC_INT_IVPR_S;

	tsi108_pic_write_reg(offset, vpr & ~TSI108PIC_MASK);
	mb();
}

static void tsi108_pic_disable_irq(u_int irq)
{
	u32 offset = get_vector_offset(irq);
	u32 vpr = tsi108_pic_read_reg(offset);

	/*
	 * Switch level interrupt to edge sensitivity to avoid generation
	 * of spurious interrupt request. See design note in Tsi108 PIC
	 * section of Tsi108 manual.
	 */
	if (irq_desc[irq].status & IRQ_LEVEL)
		vpr &= ~TSI108PIC_INT_IVPR_S;

	tsi108_pic_write_reg(offset, vpr | TSI108PIC_MASK);
	mb();
	vpr = tsi108_pic_read_reg(offset);
	if (!(vpr & TSI108PIC_MASK))
		printk("TSI108_PIC: Error - Unable disable IRQ %d\n", irq);
}

/*
 *  Initialize an interrupt source (and disable it!)
 *
 *  irq: Tsi108 PIC interrupt source number
 *  pri: interrupt source priority
 *  vec: the vector it will produce
 *  pol: polarity (1 for positive, 0 for negative)
 *  sense: 1 for level, 0 for edge
 */
static void __init
tsi108_pic_initirq(u_int irq, u_int pri, u_int vec, int pol, int sense)
{
	unsigned int ivpr;

	ivpr = TSI108PIC_MASK | (pri << 16) | vec;
	ivpr |= (IRQ_SENSE_LEVEL == sense) ?
	    TSI108PIC_INT_IVPR_S_LEVEL : TSI108PIC_INT_IVPR_S_EDGE;
	ivpr |= (IRQ_POLARITY_POSITIVE == pol) ?
	    TSI108PIC_INT_IVPR_P_HIGH : TSI108PIC_INT_IVPR_P_LOW;
	tsi108_pic_write_reg(TSI108_INT_IVPR(irq), ivpr);
	mb();
}

int tsi108_pic_set_irq_sense(int irq,	/* PIC source number */
			     int pol,	/* interrupt source polarity */
			     int sense	/* interrupt source sense */
    )
{
	unsigned int ivpr;

	ivpr = tsi108_pic_read_reg(TSI108_INT_IVPR(irq));

	if (ivpr & TSI108PIC_ACTIVITY)	/* error if source is active */
		return -1;

	if (0 == (ivpr & TSI108PIC_MASK)) {
		ivpr |= TSI108PIC_MASK;	/* mask IRQ prior making changes */
		tsi108_pic_write_reg(TSI108_INT_IVPR(irq), ivpr);
	}

	/* clear bits to be changed */
	ivpr &= ~(TSI108PIC_INT_IVPR_P | TSI108PIC_INT_IVPR_S);

	ivpr |= (IRQ_SENSE_LEVEL == sense) ?
	    TSI108PIC_INT_IVPR_S_LEVEL : TSI108PIC_INT_IVPR_S_EDGE;
	ivpr |= (IRQ_POLARITY_POSITIVE == pol) ?
	    TSI108PIC_INT_IVPR_P_HIGH : TSI108PIC_INT_IVPR_P_LOW;

	tsi108_pic_write_reg(TSI108_INT_IVPR(irq), ivpr);
	return 0;
}

/*
 *  Map an interrupt source to one or more CPUs
 */
static void tsi108_pic_mapirq(u_int irq, u_int physmask, u_int keepmask)
{
	u_int offset = get_vector_offset(irq);

	if (0 == offset)
		return;
	if (keepmask != 0)
		physmask |= tsi108_pic_read_reg(offset + 4);
	tsi108_pic_write_reg(offset + 4, physmask);
	mb();
}

/* No spinlocks, should not be necessary with the Tsi108 PIC
 * (1 register = 1 interrupt and we have the desc lock).
 */
static void tsi108_pic_ack_irq(unsigned int irq_nr)
{
	tsi108_pic_disable_irq(irq_nr);
	tsi108_pic_eoi();
}

static void tsi108_pic_end_irq(unsigned int irq_nr)
{
	if (!(irq_desc[irq_nr].status & (IRQ_DISABLED|IRQ_INPROGRESS))
	    && irq_desc[irq_nr].action)
		tsi108_pic_enable_irq(irq_nr);
}

void tsi108_pic_set_affinity(unsigned int irq_nr, unsigned long cpumask)
{
	tsi108_pic_mapirq(irq_nr, physmask(cpumask), 0);
}

int tsi108_pic_get_irq(struct pt_regs *regs)
{
	int vector = tsi108_pic_get_vect();

	if (vector == TSI108_IRQ_SPURIOUS) {
		vector = -1;
	}

	if (vector == IRQ_TSI108_PCI) {
		vector = get_pci_source(vector);
	}

	if (vector == -1) {
		tsi108_pic_write_reg(TSI108_INT_EOI(0), 0);
	}

	return vector;
}

static void tsi108_pci_int_mask(u_int irq)
{
	u_int irp_cfg;
	int int_line = (irq - IRQ_PCI_INTAD_BASE);

	irp_cfg = tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL);
	mb();
	irp_cfg |= (1 << int_line);	/* INTx_DIR = output */
	irp_cfg &= ~(3 << (8 + (int_line * 2)));	/* INTx_TYPE = unused */
	tsi108_write_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL, irp_cfg);
	mb();
	irp_cfg = tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL);
}

static void tsi108_pci_int_unmask(u_int irq)
{
	u_int irp_cfg;
	int int_line = (irq - IRQ_PCI_INTAD_BASE);

	irp_cfg = tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL);
	mb();
	irp_cfg &= ~(1 << int_line);
	irp_cfg |= (3 << (8 + (int_line * 2)));
	tsi108_write_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL, irp_cfg);
	mb();
}

static void tsi108_pci_irq_enable(u_int irq)
{
	tsi108_pci_int_unmask(irq);
}

static void tsi108_pci_irq_disable(u_int irq)
{
	tsi108_pci_int_mask(irq);
}

static void tsi108_pci_irq_ack(u_int irq)
{
	tsi108_pci_int_mask(irq);
}

static void tsi108_pci_irq_end(u_int irq)
{
	tsi108_pic_eoi();	/* eoi IRQ_TSI108_PCI */
	tsi108_pci_int_unmask(irq);
}

static inline int get_pci_source(int vector)
{
	u_int temp = 0;
	int irq = -1;
	int i;
	u_int pci_irp_stat;
	static int mask = 0;

	/* Read PCI/X block interrupt status register */
	pci_irp_stat = tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_STAT);
	mb();

	if (pci_irp_stat & TSI108_PCI_IRP_STAT_P_INT) {
		/* Process Interrupt from PCI bus INTA# - INTD# lines */
		temp =
		    tsi108_read_reg(TSI108_PCI_OFFSET +
				    TSI108_PCI_IRP_INTAD) & 0xf;
		mb();
		for (i = 0; i < 4; i++, mask++) {
			if (temp & (1 << mask % 4)) {
				irq = IRQ_PCI_INTA + mask % 4;
				mask++;
				break;
			}
		}
	}
#ifdef DBG_TSI108_INTERRUPT
	else {
		printk("TSI108_PIC: error in TSI108_PCI_IRP_STAT\n");
		pci_irp_stat =
		    tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_STAT);
		temp =
		    tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_INTAD);
		mb();
		printk(">> stat=0x%08x intad=0x%08x ", pci_irp_stat, temp);
		temp =
		    tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL);
		mb();
		printk("cfg_ctl=0x%08x ", temp);
		temp =
		    tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_ENABLE);
		mb();
		printk("irp_enable=0x%08x\n", temp);
	}
#endif				/* DBG_TSI108_INTERRUPT */

	return irq;
}

static void init_pci_source(void)
{
	tsi108_write_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_CFG_CTL,
			 0x0000ff00);
	tsi108_write_reg(TSI108_PCI_OFFSET + TSI108_PCI_IRP_ENABLE, 0x00400000);
	mb();
}

static struct sysdev_class tsi108_pic_sysclass = {
	set_kset_name("tsi108_pic"),
};

static struct sys_device device_tsi108_pic = {
	.id = 0,
	.cls = &tsi108_pic_sysclass,
};

static struct sysdev_driver driver_tsi108_pic = {
#ifdef CONFIG_PM		/* FIXME: placeholder for future development */
	.suspend = &tsi108_pic_suspend,
	.resume = &tsi108_pic_resume,
#endif				/* CONFIG_PM */
};

static int __init init_tsi108_pic_sysfs(void)
{
	int rc;

	if (!tsi108_csr_base)
		return -ENODEV;
	printk(KERN_DEBUG "Registering tsi108_pic with sysfs...\n");
	rc = sysdev_class_register(&tsi108_pic_sysclass);
	if (rc) {
		printk(KERN_ERR "Failed registering tsi108_pic sys class\n");
		return -ENODEV;
	}
	rc = sysdev_register(&device_tsi108_pic);
	if (rc) {
		printk(KERN_ERR "Failed registering tsi108_pic sys device\n");
		return -ENODEV;
	}
	rc = sysdev_driver_register(&tsi108_pic_sysclass, &driver_tsi108_pic);
	if (rc) {
		printk(KERN_ERR "Failed registering tsi108_pic sys driver\n");
		return -ENODEV;
	}
	return 0;
}

subsys_initcall(init_tsi108_pic_sysfs);
