/*
 * FILE NAME asm-mips/preempt.h
 *
 * Author: jsun@mvista.com MontaVista Software, Inc.
 *
 * 2001-2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_PREEMPT_H
#define _ASM_PREEMPT_H

#include <linux/config.h>
#include <asm/mipsregs.h>
#include <asm/debug.h>
#include <asm/cpu.h>

static inline unsigned long clock_diff(unsigned long start, unsigned long stop)
{
        return (stop - start);
}
/*
 * Boards that don't use NEW_TIME, should define their own variable
 * mips_counter_frequency and readclock macros.
 */
#define readclock()		read_c0_count()
extern unsigned int mips_hpt_frequency;
#define clock_to_usecs(clocks) ((clocks) / ((mips_hpt_frequency / 1000000)))
#define INTR_IENABLE    1
#define INTERRUPTS_ENABLED(x)   (x & INTR_IENABLE)
#define ARCH_PREDEFINES_TICKS_PER_USEC
#define TICKS_PER_USEC (mips_hpt_frequency / 1000000)

#endif
