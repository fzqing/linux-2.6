/*
 *  include/asm-arm/arch-davinci/frd.h
 *
 * Author: Kevin Hilman <khilman@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_ARCH_FRD_H
#define _ASM_ARCH_FRD_H

#include <linux/config.h>
#include <asm/arch/preempt.h>
#include <asm/arch/timex.h>

#if !defined(CONFIG_FRD_USE_TIMER_IRQ)
#error "Only FRD_USE_TIMER_IRQ supported"
#endif

extern u32 davinci_timer_read(int);

#define FRD_MPU_TIMER   3
#define FRD_TIMER_INIT	(DAVINCI_CLOCK_TICK_RATE / HZ) - 1
#define FRD_TIMER_LATCH	davinci_timer_read(FRD_MPU_TIMER)

#define FRD_SCALE_ABS_TICKS 	1

/* frd default clock function using sched_clock  */
static inline unsigned long long frd_clock(void)
{
        return davinci_get_cycles();
}

#endif
