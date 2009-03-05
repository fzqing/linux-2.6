/*
 * linux/arch/arm/mach-pxa/hrtime.c
 *
 * Author:	Nicolas Pitre
 * Created:	Jul 3, 2003
 * Copyright:	MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Note: on PXA the timer match register #2 is reserved for hr time.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/hrtime.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/pxa-regs.h>

int schedule_hr_timer_int(unsigned ref_jiffies, int cycles)
{
	long oier, match;

	/* Note: OSMR0 is always the OSCR match value for jiffies+1 */
	match = OSMR0 + (ref_jiffies - jiffies - 1) * arch_cycles_per_jiffy + cycles;
	oier = OIER;
	OSMR2 = match;
	OSSR = OSSR_M2;
	OIER = oier | OIER_E2;	
	/* Here we compare (match - OSCR)  against 8 instead of 0 -- see
	   comment in pxa_timer_interrupt() for explanation. */
	if ( (signed)(match - OSCR) <= 8 && !(OSSR & OSSR_M2) ) {
		/* specified time is already passed */
		OIER = oier & ~OIER_E2;
		return -ETIME;
	}
	return 0;
}

static irqreturn_t hr_timer2_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	OIER &= ~OIER_E2;
	OSSR = OSSR_M2;
	do_hr_timer_int();
	return IRQ_HANDLED;
}

static int __init hr_time_init(void)
{
	int ret;

	ret = request_irq(IRQ_OST2, hr_timer2_irq, SA_INTERRUPT,
			  "hr timer", NULL);
	if (ret)
		printk(KERN_ERR "can't register IRQ for hrtimer\n");
	return ret;
}

__initcall(hr_time_init);

