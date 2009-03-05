/*
 * arch/arm/mach-versatile/hrtime.c
 *
 * Author: Manish Lachwani (mlachwani@mvista.com)
 * Copyright: 2005 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * High Resolution Timer support for ARM Versatile Board.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/hrtime.h>
                                                                                                     
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/arch/hardware.h>
#include <asm/arch/hrtime.h>

#define TIMER0_VA_BASE		IO_ADDRESS(VERSATILE_TIMER0_1_BASE)
#define TIMER1_VA_BASE		(IO_ADDRESS(VERSATILE_TIMER0_1_BASE) + 0x20)
#define TIMER2_VA_BASE		IO_ADDRESS(VERSATILE_TIMER2_3_BASE)

/*
 * Timer Structure
 */
typedef struct TimerStruct {
	unsigned long TimerLoad;
	unsigned long TimerValue;
	unsigned long TimerControl;
	unsigned long TimerClear;
} TimerStruct_t;

#define TIMER_INTERVAL		(TICKS_PER_uSEC * mSEC_10)
#if TIMER_INTERVAL >= 0x100000
#define TIMER_CTRL		0x8a		/* Enable, Clock / 256 */
#define TICKS2USECS(x)		(256 * (x) / TICKS_PER_uSEC)
#elif TIMER_INTERVAL >= 0x10000
#define TIMER_CTRL		0x86		/* Enable, Clock / 16 */
#define TICKS2USECS(x)		(16 * (x) / TICKS_PER_uSEC)
#else
#define TIMER_CTRL		0x82		/* Enable */
#define TICKS2USECS(x)  	((x) / TICKS_PER_uSEC)
#endif

#define TIMER_CTRL_IE		(1 << 5)	/* Interrupt Enable */

/*
 * HRT stuff goes here.
 */
int schedule_hr_timer_int(unsigned ref_jiffies, int ref_cycles)
{
	unsigned long temp_cycles;
	volatile TimerStruct_t * subhz_timer = 
		(volatile TimerStruct_t *) TIMER2_VA_BASE;

	temp_cycles = (ref_jiffies - jiffies) * arch_cycles_per_jiffy +
	ref_cycles - get_arch_cycles(jiffies);
	
	if(unlikely(temp_cycles <= 0))
		return -ETIME;

	subhz_timer->TimerLoad = temp_cycles;
	subhz_timer->TimerControl = TIMER_CTRL | TIMER_CTRL_IE;

	return 0;
}

int get_arch_cycles(unsigned ref_jiffies)
{
	int ret;
	unsigned temp_jiffies, diff_jiffies;
	volatile TimerStruct_t * hz_timer = 
		(volatile TimerStruct_t *)TIMER0_VA_BASE;

	do {
		temp_jiffies = jiffies;
		barrier();

		ret = TIMER_RELOAD - (hz_timer->TimerValue & 0xffff);
		if (unlikely(diff_jiffies = jiffies - ref_jiffies))
			ret += diff_jiffies * arch_cycles_per_jiffy;

		barrier();
	} while(unlikely(temp_jiffies != jiffies));

	return ret;
}

/*
 * HR interrupt.
 */
static irqreturn_t 
hr_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	volatile TimerStruct_t *subhz_timer = 
		(volatile TimerStruct_t *) TIMER2_VA_BASE;

	subhz_timer->TimerClear = 1;
	subhz_timer->TimerControl = 0;
	do_hr_timer_int();

	return IRQ_HANDLED;
}

/*
 * IRQ structure
 */
static struct irqaction hr_timer_irq = {
	.name		= "ARM Versatile High Resolution Timer",
	.handler	= hr_timer_interrupt,
	.flags		= SA_INTERRUPT
};

/*
 * Initialize the HRT
 */
static int hr_timer_init(void)
{
	int ret;
	volatile TimerStruct_t * subhz_timer = 
		(volatile TimerStruct_t *) TIMER2_VA_BASE;

	subhz_timer->TimerControl = 0;
	subhz_timer->TimerLoad = 0;
	subhz_timer->TimerClear = 1;

	ret = setup_irq(IRQ_TIMERINT2_3, &hr_timer_irq);
	return ret;
}

__initcall(hr_timer_init);
