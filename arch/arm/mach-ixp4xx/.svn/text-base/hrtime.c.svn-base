
/** arch/arm/mach-ixp425/hrtime.c
 *
 * High-Res Timer Implementation for IXP4XX boards
 *
 * Author: Deepak Saxena <dsaxena@mvista.com>
 *
 * Copyright 2003-2005 (c) MontaVista, Software, Inc. 
 * 
 * This file is licensed under  the terms of the GNU General Public 
 * License version 2. This program is licensed "as is" without any 
 * warranty of any kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/hrtime.h>
#include <linux/timex.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/errno.h>
#include <asm/hardware.h>
#include <asm/preempt.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>

unsigned int scaled_arch_cycles_per_nsec, 
		scaled_nsec_per_arch_cycle;


unsigned int scaled_arch_cycles_per_tick;

int
schedule_hr_timer_int(unsigned ref_jiffies, int ref_cycles)
{
	int temp_cycles;

	BUG_ON(unlikely(ref_cycles < 0));

	temp_cycles = (ref_jiffies - jiffies) * arch_cycles_per_jiffy + ref_cycles - get_arch_cycles(jiffies);
	if(unlikely(temp_cycles <= 0))
		return -ETIME;

	*IXP4XX_OSRT2 = (temp_cycles & ~IXP4XX_OST_RELOAD_MASK) |
				IXP4XX_OST_ENABLE | IXP4XX_OST_ONE_SHOT;

	return 0;
}

int
get_arch_cycles(unsigned ref_jiffies)
{
	int ret;
	unsigned temp_jiffies;
	unsigned temp;

	do {
		extern unsigned last_jiffy_time;
		temp_jiffies = jiffies;
		barrier();

		if(unlikely(ref_jiffies > jiffies)) {
			ret = (ref_jiffies - jiffies) * arch_cycles_per_jiffy;
			ret -= last_jiffy_time - *IXP4XX_OSTS;
		} else {
			ret = *IXP4XX_OSTS - last_jiffy_time;
			if(unlikely(ref_jiffies < temp_jiffies))
				ret += (temp_jiffies - ref_jiffies) * 
						arch_cycles_per_jiffy;
		}
		barrier();	

	} while(unlikely(temp_jiffies != jiffies));

	return ret;
}

int
arch_cycle_to_nsec(int arch_cycles)
{
	int sign = arch_cycles < 1 ? -1 : 1;

	return sign * mpy_sc24(sign * arch_cycles, scaled_nsec_per_arch_cycle);
}

int
nsec_to_arch_cycle(int nsec)
{
	int sign = nsec < 1 ? -1 : 1;

	return sign * mpy_sc24(sign * nsec, scaled_arch_cycles_per_nsec);
}

static irqreturn_t
hr_time_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	*IXP4XX_OSST = IXP4XX_OSST_TIMER_2_PEND;
	*IXP4XX_OSRT2 = 0;

	do_hr_timer_int();

	return IRQ_HANDLED;
}

static struct irqaction hr_timer_irq = {
	.name		= "high-res timer",
	.handler	= hr_time_interrupt,
	.flags		= SA_INTERRUPT | SA_NODELAY
};

#define CLOCK_TICKS_PER_USEC	((CLOCK_TICK_RATE + USEC_PER_SEC/2) / USEC_PER_SEC)

void 
hr_time_init(void)
{
	scaled_arch_cycles_per_nsec = div_sc24(CLOCK_TICK_RATE, NSEC_PER_SEC);

	scaled_nsec_per_arch_cycle = div_sc24(NSEC_PER_SEC, CLOCK_TICK_RATE);

	scaled_arch_cycles_per_tick = 
		div_sc_n(18, (CLOCK_TICK_RATE / USEC_PER_SEC), 
				CLOCK_TICKS_PER_USEC);

	*IXP4XX_OSRT2 = 0;

	setup_irq(IRQ_IXP4XX_TIMER2, &hr_timer_irq);
}

