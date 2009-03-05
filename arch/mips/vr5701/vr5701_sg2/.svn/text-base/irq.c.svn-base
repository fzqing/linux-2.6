/*
 * arch/mips/vr5701/vr5701_sg2/irq.c
 *
 * The irq setup and misc routines for NEC Electronics Corporation VR5701 SolutionGearII
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <asm/system.h>
#include <asm/mipsregs.h>
#include <asm/debug.h>
#include <asm/vr5701/vr5701_sg2.h>
/*
 * IRQ mapping
 *
 *  0-7: 8 CPU interrupts
 *	0 -	software interrupt 0
 *	1 -	software interrupt 1
 *	2 -	most Vrc5477 interrupts are routed to this pin
 *	3 -	(optional) some other interrupts routed to this pin for debugg
 *	4 -	not used
 *	5 -	not used
 *	6 -	not used
 *	7 -	cpu timer (used by default)
 *
 */

void vr5701_sg2_irq_setup(void)
{
	pr_debug("NEC Electronics Corporation VR5701 SolutionGearII irq setup invoked.\n");

	/* by default, we disable all interrupts and route all vr5701
	 * interrupts to pin 0 (irq 2) */
	ddb_out32(INT_ROUTE0, 0);
	ddb_out32(INT_ROUTE1, 0);
	ddb_out32(INT_ROUTE2, 0);
	ddb_out32(INT_ROUTE3, 0);
	ddb_out32(INT_MASK, 0);
	ddb_out32(INT_CLR, ~0x0);

	clear_c0_status(0xff00);
	set_c0_status(0x0400);

	ll_vr5701_irq_route(24, 1);
	ll_vr5701_irq_enable(24);
	ll_vr5701_irq_route(25, 1);
	ll_vr5701_irq_enable(25);
	ll_vr5701_irq_route(28, 1);
	ll_vr5701_irq_enable(28);
	ll_vr5701_irq_route(29, 1);
	ll_vr5701_irq_enable(29);
	ll_vr5701_irq_route(30, 1);
	ll_vr5701_irq_enable(30);
	ll_vr5701_irq_route(31, 1);
	ll_vr5701_irq_enable(31);
	set_c0_status(0x0800);
	set_except_vector(0, vr5701_sg2_handle_int);
	/* init all controllers */
	mips_cpu_irq_init(0);
	vr5701_irq_init(8);
}

/*
 * the first level int-handler will jump here if it is a vr7701 irq
 */

asmlinkage void vr5701_sg2_irq_dispatch(struct pt_regs *regs)
{
	u32 intStatus;
	u32 bitmask;
	u32 i;
	u32 intPCIStatus;

	intStatus = ddb_in32(INT0_STAT);

	if (intStatus & 1 << 6)
		goto IRQ_EPCI;

	if (intStatus & 1 << 7)
		goto IRQ_IPCI;

	for (i = 0, bitmask = 1; i <= NUM_5701_IRQS; bitmask <<= 1, i++) {
		/* do we need to "and" with the int mask? */
		if (intStatus & bitmask) {
			do_IRQ(8 + i, regs);
		}
	}
	return;

      IRQ_EPCI:
	intStatus &= ~(1 << 6);	/* unset Status flag */
	intPCIStatus = ddb_in32(EPCI_INTS);
	for (i = 0, bitmask = 1; i < NUM_5701_EPCI_IRQS; bitmask <<= 1, i++) {
		if (intPCIStatus & bitmask) {
			do_IRQ(8 + NUM_5701_IRQS + i, regs);
		}
	}
	return;

      IRQ_IPCI:
	intStatus &= ~(1 << 7);
	intPCIStatus = ddb_in32(IPCI_INTS);
	for (i = 0, bitmask = 1; i < NUM_5701_IPCI_IRQS; bitmask <<= 1, i++) {
		if (intPCIStatus & bitmask) {
			do_IRQ(8 + NUM_5701_IRQS + NUM_5701_EPCI_IRQS + i,
			       regs);
		}
	}
	return;
}

void __init arch_init_irq(void)
{
	/* invoke board-specific irq setup */
	vr5701_sg2_irq_setup();
}
