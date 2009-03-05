/*
 * FILE NAME asm-ppc/preempt.h
 *
 * Author: David Singleton dsingleton@mvista.com MontaVista Software, Inc.
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_PREEMPT_H
#define _ASM_PREEMPT_H

#include <asm/time.h>

static inline unsigned long clock_diff(unsigned long start, unsigned long stop)
{
        return (stop - start);
}

#define readclock()		get_tbl()
#define clock_to_usecs(x)	mulhwu(tb_to_us,(x))
#define INTR_IENABLE            MSR_EE
#define INTERRUPTS_ENABLED(x)   (x & INTR_IENABLE)
#define ARCH_PREDEFINES_TICKS_PER_USEC
#define TICKS_PER_USEC		us_to_tb
#endif
