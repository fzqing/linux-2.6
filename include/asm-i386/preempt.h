/*
 * include/asm-i386/preempt.h
 *
 * Timing support for preemption latencies, kfi and ilatency patches
 *
 * Author: dsingleton <dsingleton@mvista.com>
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_PREEMT_H
#define _ASM_PREEMT_H
#include <asm/msr.h>           /* To be sure we have rdtscl definition */
#include <asm/timex.h>         /* To be sure we have cpu_khz declaration */
#include <asm/div64.h>         /* do_div */

static inline unsigned long clock_diff(unsigned long start, unsigned long stop)
{
        return (stop - start);
}

#define readclock() ({\
	unsigned long cur;\
	rdtscl(cur);\
	cur;})

#define clock_to_usecs(clocks) ({ \
	unsigned long long quot = (unsigned long long) (clocks) * 10; \
	do_div(quot, (unsigned long)(cpu_khz / 100)); \
	quot; })

#define        INTR_IENABLE            0x200
#define INTERRUPTS_ENABLED(x)   (x & INTR_IENABLE)
#define ARCH_PREDEFINES_TICKS_PER_USEC
#define TICKS_PER_USEC cpu_khz / 1000


#endif
