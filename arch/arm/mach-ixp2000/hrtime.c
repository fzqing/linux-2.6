/*
 * arch/arm/mach-ixp2000/hrtime.h
 *
 * High-Res Timer Implementation for IXP2000 boards.
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

unsigned volatile last_jiffy_time;

int arch_cycles_per_jiffy = 0;

#define USEC_PER_JIFFY (USEC_PER_SEC / HZ)

int
schedule_hr_timer_int(unsigned ref_jiffies, int ref_cycles)
{
	int temp_cycles;
	extern unsigned long processor_id;

	if ((processor_id & 0xf) < 4) {
		return -EIO;
	}	

	/*
	 * Get offset from last jiffy
	 */
	temp_cycles = (ref_jiffies - jiffies) * arch_cycles_per_jiffy + ref_cycles - get_arch_cycles(jiffies);
	if(unlikely(temp_cycles <= 0))
		return -ETIME;

	ixp2000_reg_write(IXP2000_T2_CTL, 0);
	ixp2000_reg_write(IXP2000_T2_CLD, temp_cycles);
	ixp2000_reg_write(IXP2000_T2_CTL, (1 << 7));

	return 0;
}

int
get_arch_cycles(unsigned ref_jiffies)
{
	int ret;
	unsigned temp_jiffies;

	do {
		temp_jiffies = jiffies;
		barrier();

		if(unlikely(ref_jiffies > jiffies)) {
			ret = (ref_jiffies - jiffies) * arch_cycles_per_jiffy;
			ret += next_jiffy_time - *IXP2000_T4_CSR;
			ret *= -1;
		} else {
			ret = next_jiffy_time - *IXP2000_T4_CSR;
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
	ixp2000_reg_write(IXP2000_T2_CTL, 0);
	ixp2000_reg_write(IXP2000_T2_CLR, 1);

	do_hr_timer_int();
	return IRQ_HANDLED;
}

static struct irqaction hr_timer_irq = {
	.name		= "high-res timer",
	.handler	= hr_time_interrupt,
	.flags		= SA_INTERRUPT | SA_NODELAY
};

void 
hr_time_init(void)
{
	extern unsigned long processor_id;
	unsigned long prod_id = *IXP2000_PROD_ID;

	/*
	 * HRT will only work on revision B0 or B1 CPUs
	 */
	if ((processor_id & 0xf) < 4) {
		printk(KERN_ERR "IXP2800 Rev A%d - HRT disabled\n",
				processor_id & 0xf);
		return;
	}	

	scaled_arch_cycles_per_nsec = div_sc24(ixp2000_tick_rate, NSEC_PER_SEC);

	scaled_nsec_per_arch_cycle = div_sc24(NSEC_PER_SEC, ixp2000_tick_rate);

	ixp2000_reg_write(IXP2000_T2_CTL, 0);

	setup_irq(IRQ_IXP2000_TIMER2, &hr_timer_irq);
}

