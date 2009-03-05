/*
 * include/asm-i386/frd.h
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

#include <linux/config.h>
#include "irq_vectors.h"

extern unsigned long get_offset_pit(void);

#define FRD_TIMER_IRQ   TIMER_IRQ
#define FRD_TIMER_INIT  0
#define FRD_TIMER_LATCH get_offset_pit()
#define FRD_TIMER_START
#define FRD_64BIT_TIME 1

/* frd default clock function using sched_clock  */
static inline unsigned long long frd_clock(void)
{
        unsigned long long cur;
        rdtscll(cur);
        return cur;
}

#endif

