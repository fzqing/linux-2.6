/*
 * High-resolution timer header file for SH.
 *
 * Copyright (C) 2003 MontaVista Software Inc.
 * Author: Ken Sumrall, ken@mvista.com, stolen from asm-mips/hrtime.h
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef _ASM_HRTIMER_H
#define _ASM_HRTIMER_H

#include <linux/sc_math.h>

extern int schedule_hr_timer_int(unsigned long ref_jiffies, int cycles);
extern int get_arch_cycles(unsigned long ref_jiffies);

extern int arch_cycles_per_jiffy;

extern int nsec_to_arch_cycle(int nsecs);
extern int arch_cycle_to_nsec(int cycles);

extern int hr_time_resolution;

#define schedule_jiffies_int(x)	(get_arch_cycles(x) >= arch_cycles_per_jiffy)

#define hrtimer_use (1)

#endif /* _ASM_HRTIMER_H */
