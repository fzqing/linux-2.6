/*
 * FILE NAME asm-sh/preempt.h
 *
 * Author: MontaVista Software, Inc.
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_PREEMPT_H
#define _ASM_PREEMPT_H

#include <asm/io.h>
#include <asm/timex.h>

#if defined(__sh3__)
#if defined(CONFIG_CPU_SUBTYPE_SH7300)
#define TMU0_TCOR     0xA412FE94      /* Long access */
#define TMU0_TCNT     0xA412FE98      /* Long access */
#else
#define TMU0_TCOR     0xfffffe94      /* Long access */
#define TMU0_TCNT     0xfffffe98      /* Long access */
#define TMU1_TCNT     0xfffffea4      /* Long access */
#endif
#elif defined(__SH4__)
#define TMU0_TCOR     0xffd80008      /* Long access */
#define TMU0_TCNT     0xffd8000c      /* Long access */
#define TMU1_TCNT     0xffd80018      /* Long access */
#endif

/*
 * We have a timer which counts down, not up, so stop - start.
 */
static inline unsigned long clock_diff(unsigned long start, unsigned long stop)
{
	return (start - stop);
}
#define readclock() (unsigned long)ctrl_inl(TMU1_TCNT)
#define clock_to_usecs(clocks)	((clocks)/(CLOCK_TICK_RATE/1000000))
#define ARCH_PREDEFINES_TICKS_PER_USEC
#define TICKS_PER_USEC CLOCK_TICK_RATE/1000000
#define INTERRUPTS_ENABLED(x) ((x & 0x000000f0) == 0)

#endif
