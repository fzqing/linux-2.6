/*
 * include/linux/asm-i386/hrtime-Macpi.h
 *
 *
 * 2003-7-7  Posix Clocks & timers 
 *                           by George Anzinger george@mvista.com
 *
 *			     Copyright (C) 2003 by MontaVista Software.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * MontaVista Software | 1237 East Arques Avenue | Sunnyvale | CA 94085 | USA 
 */
#include <asm/msr.h>
#include <asm/io.h>
#include "mach_timer.h"

#ifndef _ASM_HRTIME_Macpi_H
#define _ASM_HRTIME_Macpi_H

#ifdef __KERNEL__

#define CLEAR_REF_TSC /* we don't use the tsc */

/*
 * These are specific to the ACPI pm counter
 * The spec says the counter can be either 32 or 24 bits wide.  We treat them
 * all as 24 bits.  Its faster than doing the test.
 */
#define SIZE_MASK 0xffffff

extern int acpi_pm_tmr_address;
extern struct init_timer_opts hrtimer_pm_init;

/*
 * There is an errata that says the pmtimer may be wrong for a few nano
 * seconds (while it is settling) and that we should read it twice to
 * get a good reading.  This takes far too long to do in gettimeofday()
 * so we do the following, prefering to be smart about what is should be
 * and doing the double read only when we get something that looks a
 * bit wonkey.  The likely case of a reasonable count takes only one read.
 */
#ifdef _INCLUDED_FROM_TIME_C
unsigned long
quick_get_cpuctr(void)
{
	static  unsigned long last_read = 0;
	static  int qgc_max = 0;
	int i;

	unsigned long rd_delta, rd_ans, rd = inl(acpi_pm_tmr_address);

	/*
	 * This will be REALLY big if ever we move backward in time...
	 */
	rd_delta = (rd - last_read) & SIZE_MASK;
	last_read = rd;

	rd_ans =  (rd - last_update) & SIZE_MASK;

	if (likely((rd_ans < (arch_cycles_per_jiffy << 1)) && 
		   (rd_delta < (arch_cycles_per_jiffy << 1))))
		return rd_ans;

	for (i = 0; i < 10; i++) {
		rd = inl(acpi_pm_tmr_address);
		rd_delta = (rd - last_read) & SIZE_MASK;
		last_read = rd;
		if (unlikely(i > qgc_max))
			qgc_max = i;
		/*
		 * On my test machine (800MHZ dual PIII) this is always
		 * seven.  Seems long, but we will give it some slack...
		 * We note that rd_delta (and all the vars) unsigned so
		 * a backward movement will show as a really big number.
		 */
		if (likely(rd_delta < 20))
			return (rd - last_update) & SIZE_MASK;
	}
	return (rd - last_update) & SIZE_MASK;
}
EXPORT_SYMBOL(quick_get_cpuctr);
#else
extern unsigned long
quick_get_cpuctr(void);
#endif
/*
 * This function moves the last_update value to the closest to the
 * current 1/HZ boundry.  This MUST be called under the write xtime_lock.
 */
static inline unsigned long
stake_cpuctr(void)
{

	if ((unsigned)quick_get_cpuctr() > arch_cycles_per_jiffy) {
		last_update = (last_update + arch_cycles_per_jiffy) & SIZE_MASK;
	}
}


/* 
 * We use various scaling.  The sc32 scales by 2**32, sc_n by the first
 * parm.  When working with constants, choose a scale such that
 * x/n->(32-scale)< 1/2.  So for 1/3 <1/2 so scale of 32, where as 3/1
 * must be shifted 3 times (3/8) to be less than 1/2 so scale should be
 * 29
 *
 */
#define HR_SCALE_ARCH_NSEC 22
#define HR_SCALE_ARCH_USEC 32
#define HR_SCALE_NSEC_ARCH 32
#define HR_SCALE_USEC_ARCH 29

#ifndef  PM_TIMER_FREQUENCY
#define PM_TIMER_FREQUENCY  3579545	/*45   counts per second */
#endif
#define PM_TIMER_FREQUENCY_x_100  357954545	/* counts per second * 100 */

#define cf_arch_to_usec (SC_32(100000000)/(long long)PM_TIMER_FREQUENCY_x_100)

static inline int
arch_cycle_to_usec(unsigned long update)
{
	return (mpy_sc32(update, arch_to_usec));
}
/* 
 * Note: In the SMP case this value will be overwritten when the APIC
 * clocks are figured out using the "compute_latch function below.  If
 * the system is not SMP, the PIT is the ticker and this is the
 * conversion for that.
 */
#define cf_arch_to_latch SC_32(CLOCK_TICK_RATE)/(long long)(CLOCK_TICK_RATE * 3)
/*
 * APIC clocks run from a low of 33MH to say 200MH.  The PM timer runs
 * about 3.5 MH.  We want to scale so that ( APIC << scale )/PM is less
 * 2 ^ 32.  Lets use 2 ^ 19, leaves plenty of room.
 */
#define HR_SCALE_ARCH_LATCH 19

#ifndef USE_APIC_TIMERS
/*
 * We need to take 1/3 of the presented value (or more exactly)
 * CLOCK_TICK_RATE /PM_TIMER_FREQUENCY.  Note that these two timers are
 * on the same cyrstal so will be EXACTLY 1/3.
 */
static inline int
arch_cycle_to_latch(unsigned long update)
{
	return (mpy_sc32(update, arch_to_latch));
}
#else
static inline int
arch_cycle_to_latch(unsigned long update)
{
	return (mpy_sc_n(HR_SCALE_ARCH_LATCH, update, arch_to_latch));
}

#endif

#define compute_latch(APIC_clocks_jiffie) arch_to_latch = div_sc_n(   \
                                                    HR_SCALE_ARCH_LATCH,   \
				                    APIC_clocks_jiffie,   \
				                    arch_cycles_per_jiffy);

#define cf_arch_to_nsec (SC_n(HR_SCALE_ARCH_NSEC,100000000000LL)/ \
                           (long long)PM_TIMER_FREQUENCY_x_100)

static inline int
arch_cycle_to_nsec(long update)
{
	return mpy_sc_n(HR_SCALE_ARCH_NSEC, update, arch_to_nsec);
}
/* 
 * And the other way...
 */
#define cf_usec_to_arch (SC_n( HR_SCALE_USEC_ARCH,PM_TIMER_FREQUENCY_x_100)/ \
                                            (long long)100000000)
static inline int
usec_to_arch_cycle(unsigned long usec)
{
	return mpy_sc_n(HR_SCALE_USEC_ARCH, usec, usec_to_arch);
}
#define cf_nsec_to_arch (SC_n( HR_SCALE_NSEC_ARCH,PM_TIMER_FREQUENCY)/ \
                                            (long long)1000000000)
static inline int
nsec_to_arch_cycle(unsigned long nsec)
{
	return mpy_sc32(nsec, nsec_to_arch);
}

extern int hrt_get_acpi_pm_ptr(void);

#ifdef _INCLUDED_FROM_TIME_C

#include <asm/io.h>
struct timer_conversion_bits timer_conversion_bits = {
	_arch_cycles_per_jiffy:(LATCH * 3),
	_nsec_to_arch:cf_nsec_to_arch,
	_usec_to_arch:cf_usec_to_arch,
	_arch_to_nsec:cf_arch_to_nsec,
	_arch_to_usec:cf_arch_to_usec,
	_arch_to_latch:cf_arch_to_latch
};
int acpi_pm_tmr_address;
#endif


/*
 * No run time conversion factors need to be set up as the pm timer has a fixed
 * speed.
 *
 * Here we have a local udelay for our init use only.  The system delay has
 * has not yet been calibrated when we use this, however, we do know
 * tsc_cycles_per_5_jiffies...
 */
extern unsigned long tsc_cycles_per_50_ms;

static inline __init void hrt_udelay(int usec)
{
        long now,end;
        rdtscl(end);
        end += (usec * tsc_cycles_per_50_ms) / (CALIBRATE_TIME);
        do {rdtscl(now);} while((end - now) > 0);

}

#if defined( CONFIG_HIGH_RES_TIMER_ACPI_PM_ADD) && CONFIG_HIGH_RES_TIMER_ACPI_PM_ADD > 0
#define default_pm_add CONFIG_HIGH_RES_TIMER_ACPI_PM_ADD
#define message "High-res-timers: ACPI pm timer not found.  Trying specified address %d\n"
#else
#define default_pm_add 0
#define message \
        "High-res-timers: ACPI pm timer not found(%d) and no backup."\
        "\nCheck BIOS settings or supply a backup.  See configure documentation.\n"
#endif
#define fail_message \
"High-res-timers: >-<--><-->-<-->-<-->-<--><-->-<-->-<-->-<-->-<-->-<-->-<-->-<\n"\
"High-res-timers: >Failed to find the ACPI pm timer                           <\n"\
"High-res-timers: >-<--><-->-<-->-<-->-<-->Boot will fail in Calibrate Delay  <\n"\
"High-res-timers: >Supply a valid default pm timer address                    <\n"\
"High-res-timers: >or get your BIOS to turn on ACPI support.                  <\n"\
"High-res-timers: >See CONFIGURE help for more information.                   <\n"\
"High-res-timers: >-<--><-->-<-->-<-->-<--><-->-<-->-<-->-<-->-<-->-<-->-<-->-<\n"
/*
 * After we get the address, we set last_update to the current timer value
 */
#endif				/* __KERNEL__ */
#endif				/* _ASM_HRTIME-Mapic_H */
