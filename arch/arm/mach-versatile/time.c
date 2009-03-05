/*
 * arch/arm/mach-versatile/time.c
 *
 * Author: Manish Lachwani (mlachwani@mvista.com)
 * Copyright: 2005 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Timer code for ARM Versatile. Brief description of the Timer 
 * Registers:
 *
 * TIMER LOAD Register 		- R/W register for the timer counter.
 * TIMER VALUE Register 	- Read only register
 * TIMER CONTROL Register	- Set IE, free running mode, pre-scale,
 *				  Use of 32-bit counter
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

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/arch/hardware.h>

#define TIMER0_VA_BASE	IO_ADDRESS(VERSATILE_TIMER0_1_BASE)
#define TIMER1_VA_BASE	(IO_ADDRESS(VERSATILE_TIMER0_1_BASE) + 0x20)
#define VA_IC_BASE	IO_ADDRESS(VERSATILE_VIC_BASE)

/*
 * Timer Structure
 */
typedef struct TimerStruct {
	unsigned long TimerLoad;
	unsigned long TimerValue;
	unsigned long TimerControl;
	unsigned long TimerClear;
} TimerStruct_t;

#define TIMER_INTERVAL	(TICKS_PER_uSEC * mSEC_10)
#if TIMER_INTERVAL >= 0x100000
#define TIMER_RELOAD	(TIMER_INTERVAL >> 8)		/* Divide by 256 */
#define TIMER_CTRL	0x8a				/* Enable, Clock / 256 */
#define TICKS2USECS(x)	(256 * (x) / TICKS_PER_uSEC)
#elif TIMER_INTERVAL >= 0x10000
#define TIMER_RELOAD	(TIMER_INTERVAL >> 4)		/* Divide by 16 */
#define TIMER_CTRL	0x86				/* Enable, Clock / 16 */
#define TICKS2USECS(x)	(16 * (x) / TICKS_PER_uSEC)
#else
#define TIMER_RELOAD	(TIMER_INTERVAL)
#define TIMER_CTRL	0x82				/* Enable */
#define TICKS2USECS(x)	((x) / TICKS_PER_uSEC)
#endif
#define TIMER_CTRL_IE	(1 << 5)			/* Interrupt Enable */

#define RTC_DR		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x000)
#define RTC_MR		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x004)
#define RTC_LR		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x008)
#define RTC_CR		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x00C)
#define RTC_IMSC	(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x010)
#define RTC_RIS		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x014)
#define RTC_MIS		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x018)
#define RTC_ICR		(IO_ADDRESS(VERSATILE_RTC_BASE) + 0x01C)

/*
 * RTC set and get functions
 */
static inline unsigned long versatile_get_rtc_time(void)
{
	return __raw_readl(RTC_DR);
}

static int versatile_set_rtc(void)
{
	__raw_writel(xtime.tv_sec, RTC_LR);
	return 1;
}

/* 
 * IRQs are disabled before entering here from do_gettimeofday() 
 */
static unsigned long versatile_gettimeoffset (void)
{
	volatile TimerStruct_t *timer0 = (TimerStruct_t *)TIMER0_VA_BASE;
	unsigned long ticks = LATCH - (signed long)timer0->TimerValue;

	while (ticks > TIMER_RELOAD)
	ticks -= LATCH;
        
	return TICKS2USECS(ticks);
}

unsigned long versatile_timer_read(int nr)
{
	volatile TimerStruct_t *timer = NULL;

	/* Timer 0 is used by versatile_gettimeoffset */
	switch (nr) {
	case 1:
		timer = (TimerStruct_t *) TIMER1_VA_BASE;
		break;
	default:
		printk(KERN_ERR "versatile_timer_read(%d): unknown timer\n", nr);
		BUG();
	}
	return timer->TimerValue;
}
EXPORT_SYMBOL(versatile_timer_read);

/*
 * Timer interrupt handler
 */
static irqreturn_t
versatile_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	volatile TimerStruct_t *timer0 = (volatile TimerStruct_t *)TIMER0_VA_BASE;
	unsigned int next_match = 0xFFFFFFFF;

	write_seqlock(&xtime_lock);
	timer0->TimerClear = 0;
	do {
		timer_tick(regs);
		next_match -= LATCH;
	} while ((signed long)(next_match - timer0->TimerValue) >= 0);
	
	timer0->TimerLoad = timer0->TimerValue - next_match;
	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

/*
 * IRQ structure
 */
static struct irqaction versatile_timer_irq = {
	.name		= "Versatile Timer Tick",
	.flags		= SA_INTERRUPT,
	.handler	= versatile_timer_interrupt
};

/*
 * Initialize the timer
 */
static void __init versatile_timer_init(void)
{
	struct timespec tv;
	volatile TimerStruct_t *timer0 = (TimerStruct_t *)TIMER0_VA_BASE;
	volatile TimerStruct_t *timer1 = (volatile TimerStruct_t *)TIMER1_VA_BASE;

	__raw_writel(1, RTC_CR);   /* enable RTC */
	__raw_writel(0, RTC_IMSC); /* mask interrupts for now */
	__raw_writel(1, RTC_ICR);  /* clear RTC interrupt */

	set_rtc = versatile_set_rtc;

	tv.tv_nsec = 0;
	tv.tv_sec = versatile_get_rtc_time();
	do_settimeofday(&tv);

	timer0->TimerControl = 0;
	timer1->TimerControl = 0;

	/*
	 * set clock frequency:
	 *	VERSATILE_REFCLK is 32KHz
	 *	VERSATILE_TIMCLK is 1MHz
	 */
	*(volatile unsigned int *)IO_ADDRESS(VERSATILE_SCTL_BASE) |=
		((VERSATILE_TIMCLK << VERSATILE_TIMER1_EnSel) | 
		 (VERSATILE_TIMCLK << VERSATILE_TIMER2_EnSel) |
		 (VERSATILE_TIMCLK << VERSATILE_TIMER3_EnSel) | 
		 (VERSATILE_TIMCLK << VERSATILE_TIMER4_EnSel));

	timer0->TimerLoad = TIMER_RELOAD;  /* Interrupt right away */
	timer0->TimerControl = TIMER_CTRL | TIMER_CTRL_IE;

#ifdef CONFIG_PREEMPT_RT
	timer1->TimerLoad    = ~0;
	timer1->TimerValue   = ~0;
	timer1->TimerControl = 0x82;    /* periodic, 32bit */
#endif
	setup_irq(IRQ_TIMERINT0_1, &versatile_timer_irq);
}

struct sys_timer versatile_timer = {
	.init		= versatile_timer_init,
	.offset		= versatile_gettimeoffset,
};
