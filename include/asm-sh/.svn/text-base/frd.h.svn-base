/*
 * include/asm-mips/frd.h
 *
 * Timing support for RT fast domain kernel preemption measurements
 *
 * Author: MontaVista Software, Inc.
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_FRD_H
#define _ASM_FRD_H

#include <linux/autoconf.h>
#include <asm/preempt.h>

#define FRD_TIMER_LATCH	ctrl_inl(TMU0_TCNT)
#define FRD_TIMER_INIT ctrl_inl(TMU0_TCOR)
#define FRD_TIMER_COUNTS_DOWN	1
#define FRD_IRQ_TIMER_COUNTS_DOWN	1
#define FRD_SCALE_ABS_TICKS

/* alternative function for reading a timestamp */
/* frd default clock function using sched_clock  */
static inline unsigned long long frd_clock(void)
{
	return (unsigned long long)(~ctrl_inl(TMU1_TCNT));
}
#endif
