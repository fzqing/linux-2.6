/*
 * arch/mips/vr5701/vr5701_sg2/irq_vr5701.c
 *
 * This file defines the irq handler for NEC Electronics Corporation VR5701 SolutionGearII
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
/*
 * NEC Electronics Corporation VR5701 SolutionGearII defines 32 IRQs.
 *
 */
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/ptrace.h>

#include <asm/debug.h>
#include <asm/vr5701/vr5701_sg2.h>

static int vr5701_irq_base = -1;

void ll_vr5701_irq_disable(int vr5701_irq, int ack);

static void vr5701_irq_enable(unsigned int irq)
{
	ll_vr5701_irq_enable(irq - vr5701_irq_base);
}

static void vr5701_irq_disable(unsigned int irq)
{
	ll_vr5701_irq_disable(irq - vr5701_irq_base, 0);
}

static unsigned int vr5701_irq_startup(unsigned int irq)
{
	vr5701_irq_enable(irq);
	return 0;
}

static void vr5701_irq_ack(unsigned int irq)
{
	unsigned long flags;
	local_irq_save(flags);

	/* clear the interrupt bit for edge trigger */
	/* some irqs require the driver to clear the sources */
	if (irq < vr5701_irq_base + NUM_5701_IRQ) {
		ddb_out32(INT_CLR, 1 << (irq - vr5701_irq_base));
	}
	/* don't need for PCIs, for they are level triger */

	/* disable interrupt - some handler will re-enable the irq
	 * and if the interrupt is leveled, we will have infinite loop
	 */
	ll_vr5701_irq_disable(irq - vr5701_irq_base, 1);
	local_irq_restore(flags);
}

static void vr5701_irq_end(unsigned int irq)
{
	unsigned long flags;
	local_irq_save(flags);

	if (!(irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS))) {
		ll_vr5701_irq_enable(irq - vr5701_irq_base);
	}
	local_irq_restore(flags);
}

struct hw_interrupt_type vr5701_irq_type = {
	"vr5701_irq",
	vr5701_irq_startup,
	vr5701_irq_disable,
	vr5701_irq_enable,
	vr5701_irq_disable,
	vr5701_irq_ack,
	vr5701_irq_end,
	NULL			/* no affinity stuff for UP */
};

void vr5701_irq_init(u32 irq_base)
{
	extern irq_desc_t irq_desc[];
	u32 i;
	vr5701_irq_base = irq_base;
	for (i = irq_base;
	     i < irq_base + NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = 0;
		irq_desc[i].depth = 1;
		irq_desc[i].handler = &vr5701_irq_type;
	}
}

int vr5701_irq_to_irq(int irq)
{
	return irq + vr5701_irq_base;
}

void ll_vr5701_irq_route(int vr5701_irq, int ip)
{
	u32 reg_value;
	u32 reg_bitmask;
	u32 reg_index;

	if (vr5701_irq >= NUM_5701_IRQ) {
		if (vr5701_irq >= NUM_5701_IRQ + NUM_EPCI_IRQ) {
			vr5701_irq = 7;
		} else {
			vr5701_irq = 6;
		}
	}
	reg_index = INT_ROUTE0 + vr5701_irq / 8 * 4;
	reg_value = ddb_in32(reg_index);
	reg_bitmask = 7 << (vr5701_irq % 8 * 4);
	reg_value &= ~reg_bitmask;
	reg_value |= ip << (vr5701_irq % 8 * 4);
	ddb_out32(reg_index, reg_value);
}

void ll_vr5701_irq_enable(int vr5701_irq)
{
	u16 reg_value;
	u32 reg_bitmask;
	unsigned long flags;

	local_irq_save(flags);
	irq_desc[vr5701_irq_base + vr5701_irq].depth++;

	if (vr5701_irq >= NUM_5701_IRQ) {
		if (vr5701_irq >= NUM_5701_IRQ + NUM_EPCI_IRQ) {
			reg_value = ddb_in32(IPCI_INTM);
			reg_bitmask =
			    1 << (vr5701_irq - NUM_5701_IRQ - NUM_EPCI_IRQ);
			ddb_out32(IPCI_INTM, reg_value | reg_bitmask);
			vr5701_irq = 7;
		} else {
			reg_value = ddb_in32(EPCI_INTM);
			reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ);
			ddb_out32(EPCI_INTM, reg_value | reg_bitmask);
			vr5701_irq = 6;
		}
	}
	reg_value = ddb_in32(INT_MASK);
	ddb_out32(INT_MASK, reg_value | (1 << vr5701_irq));
	local_irq_restore(flags);
}

void ll_vr5701_irq_disable(int vr5701_irq, int ack)
{
	u16 reg_value;
	u32 udummy;
	u32 reg_bitmask;
	unsigned long flags;

	local_irq_save(flags);
	if (!ack) {
		irq_desc[vr5701_irq_base + vr5701_irq].depth--;
		if (irq_desc[vr5701_irq_base + vr5701_irq].depth) {
			local_irq_restore(flags);
			return;
		}
	}

	if (vr5701_irq >= NUM_5701_IRQ) {
		if (vr5701_irq >= NUM_5701_IRQ + NUM_EPCI_IRQ) {
			goto DISABLE_IRQ_IPCI;
		} else {
			goto DISABLE_IRQ_EPCI;
		}
	}
	reg_value = ddb_in32(INT_MASK);
	ddb_out32(INT_MASK, reg_value & ~(1 << vr5701_irq));
	udummy = ddb_in32(INT_MASK);
	local_irq_restore(flags);
	return;

      DISABLE_IRQ_IPCI:
	reg_value = ddb_in32(IPCI_INTM);
	reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ - NUM_EPCI_IRQ);
	ddb_out32(IPCI_INTM, reg_value & ~reg_bitmask);
	local_irq_restore(flags);
	return;

      DISABLE_IRQ_EPCI:
	reg_value = ddb_in32(EPCI_INTM);
	reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ);
	ddb_out32(EPCI_INTM, reg_value & ~reg_bitmask);
	local_irq_restore(flags);
}
