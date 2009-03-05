/*
 * Copyright (C) 2005 MontaVista Software Inc.
 * Author: Manish Lachwani 
 * 	   (mlachwani@mvista.com or manish_lachwani@koffee-break.com)
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef _ASM_HRTIMER_H
#define _ASM_HRTIMER_H

#define HRTIME_PER_CPU 

#include <linux/config.h>
#include <linux/sc_math.h>

#ifndef CONFIG_CPU_TIMER
#error "This board cannot support high resolution timer at this moment."
#endif /* CONFIG_CPU_TIMER */

#include <asm/mipsregs.h>
#include <asm/bitops.h>

extern int schedule_hr_timer_int(unsigned long ref_jiffies, int cycles);
extern int get_arch_cycles(unsigned long ref_jiffies);
extern unsigned long  cycles_per_jiffy;
extern int nsec_to_arch_cycle(int nsecs);
extern int arch_cycle_to_nsec(int cycles);
extern int hr_time_resolution;

#define arch_cycles_per_jiffy	((long)cycles_per_jiffy)

int schedule_jiffies_int(unsigned long ref_jiffies);

#define hrtimer_use    1

#endif /* _ASM_HRTIMER_H */

