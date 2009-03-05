/*
 * include/asm-mips/frd.h
 *
 * Timing support for RT fast domain kernel preemption measurements
 *
 * Author: Sven Thorsten Dietrich <sdietrich@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_FRD_H
#define _ASM_FRD_H

#include <linux/autoconf.h>
#include <asm/preempt.h>
extern u32 last_count;

#define FRD_TIMER_LATCH		(read_c0_count() - last_count)
#define FRD_TIMER_LATCH_ABS(IRQ_TIME, TICKS_AT_IRQ) \
				(read_c0_count() - IRQ_TIME + TICKS_AT_IRQ)
#define FRD_TIMER_INIT		0
#define FRD_64BIT_TIME		1

/* alternative function for reading a timestamp */
/* frd default clock function using sched_clock  */
static inline unsigned long long frd_clock(void)
{
	/* using common code from preempt.h */
        return (unsigned long long)(read_c0_count());
}
#endif
