/*
 * include/asm-ppc64/frd.h
 *
 * Timing support for RT fast domain kernel preemption measurements
 *
 * Author: Tom Rini <trini@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_FRD_H
#define _ASM_FRD_H

#include <linux/config.h>
#include <asm/time.h>

#define FRD_TIMER_START
#define FRD_TIMER_INIT  0
#define FRD_IRQ_TIMER_COUNTS_DOWN

#define FRD_TIMER_LATCH get_dec()
#define FRD_TIMER_LATCH_ABS(IRQ_TIME, TICKS_AT_IRQ) \
	((get_tb() - IRQ_TIME) + (0UL - TICKS_AT_IRQ))

/* frd default clock function using the timebase. */
static inline unsigned long long frd_clock(void)
{
	return get_tb();
}
#endif
