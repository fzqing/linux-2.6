/*
 * include/linux/asm-i386/hrtime.h
 *
 *
 * 2003-7-7  Posix Clocks & timers 
 *                           by George Anzinger george@mvista.com
 *
 *			     Copyright (C) 2002 2003 by MontaVista Software.
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
#ifndef _I386_HRTIME_H
#define _I386_HRTIME_H
#ifdef __KERNEL__

#include <linux/config.h>	/* for CONFIG_APM etc... */
#include <asm/types.h>		/* for u16s */
#include <asm/io.h>
#include <linux/sc_math.h>	/* scaling math routines */
#include <asm/delay.h>
#include <linux/smp.h>
#include <linux/jiffies.h>	/* for LATCH */
#include <asm/percpu.h>
#include <asm/i8253.h>

#define HRT_PM 1
#define HRT_TSC 2



/*
 * Here is a road map on timer discipline: Timers need to be disciplined
 * when there are two time source that need to stay in sync, but which
 * are driven by different source Xtals.  For example, the PIT and the
 * TSC are driven by two different sources as are the PIT and the IOAPIC
 * timers.  On the other hand, the PIT and the PMTIMER are driven by the
 * same Xtal. Further, if we treat the pmtimer and the TSC as fill in
 * timers, i.e. timers to "fill in" the arch_cycle part of the time, we
 * don't need to discipline them, but just reset the base or reference
 * each tick.  On the other hand, if we treat them as the actual clock,
 * then the PIT (which generates the tick interrupts) would need to bd
 * disciplined.  Why do one or the other?  If we program the PIT to
 * generate arch_cycle interrupts we can no longer rely on it for the
 * time and so must use either the pmtimer or the TSC as the actual
 * clock.  We would prefer to not program the PIT in this way, but
 * require another arch_cycle interrupt source to allow us to free the
 * PIT in this regard.  That other interrupt source, at this time, is
 * the IOAPIC timer.  So here is a nice little table that shows us (and
 * you) what we are up to with the various hardware options.

 *-----------------------------------------------------------------------|
 *               |    TSC            |     PMTIMER     |  SMP tick notify
 *---------------|-------------------|-----------------|--------------------|
 * SMP=> IOAPIC  | no discipline,    | no discipline,  | Use IPI to notify
 *               | TSC synced to PIT | Hard sync       | other cpus of tick  
 *---------------|-------------------|-----------------|--------------------|
 * IOAPIC no SMP | no discipline,    | no discipline,  |
 *               | TSC synced to PIT | Hard sync       |   
 *---------------|-------------------|-----------------|--------------------|
 * no IOAPIC     | discipline        | discipline      |
 *               | PIT synced to TSC | PIT programmed  |
 *---------------|-------------------|-----------------|--------------------|

 * Now in the case of not needing to discipline the timer, we still need
 * a reasonable way to reset the fillin timer. Just doing it at
 * interrupt time leads to a lot of jitter because interrupts jitter and
 * are subject to latency.  It turns out that the same way we discipline
 * a timer is useful here.  So, below, we have a function to generate
 * resets for the fillin timer, be it TSC or pmtimer.  But, wait, the
 * pmtimer needs to be synced only once and then should not need any
 * thing more.  It should just be advanced by 3 times the LATCH each
 * tick.  Also we can zero it as part of starting the PIT.  So this
 * means that in the case of the IOAPIC system we sync the TSC to the
 * PIT while in the case of no IOAPIC we sync the PIT to the TSC.

 * We always want the timer, if not touched otherwise, to give periodic
 * 1/HZ interrupts.  This is done by programing the interrupt we want
 * and, once it it loaded, (in the case of the PIT) dropping a 1/HZ
 * program on top of it.  For other timers, well, we just use them to
 * generate the arch_cycle interrupt and try to set them up as single
 * shot timers. The The PIT will give us the desired interrupt and, at
 * interrupt time, load the 1/HZ program.  So...

 * If no arch_cycle 1/HZ ticks are needed AND we are aligned with the 1/HZ 
 * boundry, we don't need to touch the PIT.  Otherwise we do the above.

 * There are two reasons to keep this:
 * 1. The NMI watchdog uses the timer interrupt to generate the NMI interrupts.
 * 2. We don't have to touch the PIT unless we have a arch_cycle event in
 *    the next 1/HZ interval (unless we drift away from the 1/HZ boundry).
 */

/*
 * The high-res-timers option is set up to self configure with different 
 * platforms.  It is up to the platform to provide certian macros which
 * override the default macros defined in system without (or with disabled)
 * high-res-timers.
 *
 * To do high-res-timers at some fundamental level the timer interrupt must
 * be seperated from the time keeping tick.  A tick can still be generated
 * by the timer interrupt, but it may be surrounded by non-tick interrupts.
 * It is up to the platform to determine if a particular interrupt is a tick,
 * and up to the timer code (in timer.c) to determine what time events have
 * expired.
 *
 * Details on the functions and macros can be found in the file:
 * include/hrtime.h
 */
struct timer_conversion_bits {
	unsigned long _arch_to_usec;
	unsigned long _arch_to_nsec;
	unsigned long _usec_to_arch;
	unsigned long _nsec_to_arch;
	long _arch_cycles_per_jiffy;
	unsigned long _arch_to_latch;
	unsigned long volatile _last_update;
};
extern struct timer_conversion_bits timer_conversion_bits;
/*
 * The following four values are not used for machines 
 * without a TSC.  For machines with a TSC they
 * are caculated at boot time. They are used to 
 * calculate "cycles" to jiffies or usec.  Don't get
 * confused into thinking they are simple multiples or
 * divisors, however.  
 */
#define arch_to_usec timer_conversion_bits._arch_to_usec
#define arch_to_nsec timer_conversion_bits._arch_to_nsec
#define usec_to_arch timer_conversion_bits._usec_to_arch
#define nsec_to_arch timer_conversion_bits._nsec_to_arch
#define arch_cycles_per_jiffy timer_conversion_bits._arch_cycles_per_jiffy
#define arch_to_latch timer_conversion_bits._arch_to_latch
#define last_update timer_conversion_bits._last_update

/*
 * no of usecs less than which events cannot be scheduled
 */
#ifndef CONFIG_HIGH_RES_RESOLUTION
#define CONFIG_HIGH_RES_RESOLUTION 1000	/* nano second resolution
					   we will use for high res. */
#endif

#define TIMER_DELTA  5

#ifdef _INCLUDED_FROM_TIME_C
EXPORT_SYMBOL(timer_conversion_bits);
#define EXTERN
int timer_delta = TIMER_DELTA;
int hr_time_resolution = CONFIG_HIGH_RES_RESOLUTION;
#else
#define EXTERN  extern
extern int timer_delta;
extern int hr_time_resolution;
#endif

#ifdef CONFIG_X86_LOCAL_APIC 
#define USE_APIC_TIMERS
#define  discipline_PIT_timer()
#else
#define TIMER_NEEDS_DISCIPLINE 
#define  discipline_PIT_timer() discipline_timer()
#endif

/*

 * Interrupt generators need to be disciplined to generate the interrupt
 * on the 1/HZ boundry (assuming we don't need arch_cycle interrupts) if
 * the timer clock is other than the interrupt generator clock.  In the
 * I386 case this includes the PIT and TSC or pm combinations and the
 * apic and TSC or pm combinations, i.e. all but the PIT/PIT
 * combination.

 */
#ifdef TIMER_NEEDS_DISCIPLINE
#define IF_DISCIPLINE(x) x
EXTERN int timer_discipline_diff;
EXTERN int min_hz_arch_cycle;
EXTERN int max_hz_arch_cycle;
#else
#define IF_DISCIPLINE(x)
#endif

#if defined(CONFIG_SMP) &&  defined(USE_APIC_TIMERS)
#define HRTIME_PER_CPU      /* we have per cpu timers */
EXTERN int _last_was_long[NR_CPUS];
#define __last_was_long  _last_was_long[smp_processor_id()]
#else
EXTERN int _last_was_long;
#define __last_was_long  _last_was_long
#endif

EXTERN int hrtimer_use;

#define USEC_PER_JIFFIES  (1000000/HZ)
#define PIT0_LATCH_STATUS 0xe2
#define PIT0 0x40
#define PIT1 0x41
#define PIT_COMMAND 0x43
#define PIT0_ONE_SHOT 0x38
#define PIT0_PERIODIC 0x34
#define PIT0_LATCH_COUNT 0xd2
#define PIT01_LATCH_COUNT 0xd6
#define PIT_NULL_COUNT 0x40
#define READ_CNT0(varr) {varr = inb(PIT0);varr += (inb(PIT0))<<8;}
#define READ_CNT1(var) { var = inb(PIT1); }
#define LATCH_CNT0() { outb(PIT0_LATCH_COUNT,PIT_COMMAND); }
#define LATCH_CNT0_AND_CNT1() { outb(PIT01_LATCH_COUNT,PIT_COMMAND); }

/*
 * This is really: x*(CLOCK_TICK_RATE+HZ/2)/1000000
 * Note that we can not figure the constant part at
 * compile time because we would loose precision.
 */
#define TO_LATCH(x) (((x)*LATCH)/USEC_PER_JIFFIES)

#define schedule_hr_timer_int(a,b)  _schedule_next_int(a,b)
#define schedule_jiffies_int(a) _schedule_jiffies_int(a)


extern volatile unsigned long jiffies;
extern u64 jiffies_64;

extern int _schedule_next_int(unsigned long jiffie_f, long arch_cycle_in);
extern int _schedule_jiffies_int(unsigned long jiffie_f);

/*
 * Now go ahead and include the clock specific file 586/386/acpi
 * These asm files have static inline functions to do a lot of
 * stuff as well as the conversion routines.
 */
#ifdef CONFIG_HIGH_RES_TIMER_ACPI_PM
#include <asm/hrtime-Macpi.h>
#elif defined(CONFIG_HIGH_RES_TIMER_TSC)
#include <asm/hrtime-M586.h>
#else
#error "Need one of: CONFIG_HIGH_RES_TIMER_ACPI_PM CONFIG_HIGH_RES_TIMER_TSC"
#endif

/*
 * This routine is always called under the write_lockirq(xtime_lock)
 * We provide something like a sequence lock for local use.  We need to
 * keep cycles, arch_cycle and jiffie aligned (be nice to get 
 * wall_to_monotonic, but, sigh, another day).
 */

static inline long
get_arch_cycles(unsigned long ref) 
{
	return (long)(jiffies - ref) * arch_cycles_per_jiffy + quick_get_cpuctr();
}

static inline void
start_PIT(void)
{
	spin_lock(&i8253_lock);
	outb_p(PIT0_PERIODIC, PIT_COMMAND);
	outb_p(LATCH & 0xff, PIT0);	/* LSB */
	outb(LATCH >> 8, PIT0);	/* MSB */
	spin_unlock(&i8253_lock);
	last_update -= quick_get_cpuctr();
}


#define SC_32_TO_USEC (SC_32(1000000)/ (long long)CLOCK_TICK_RATE)

	/*
	 * We program the PIT to give periodic interrupts and, if no
	 * arch_cycle timers are due, leave it alone.  This means that
	 * it can drift WRT the clock (TSC or pm timer).  What we are
	 * trying to do is to program the next interrupt to occure on
	 * exactly the requested time.  If we are not doing arch_cycle HZ
	 * interrupts we expect to find a small excess of time beyond
	 * the 1/HZ, i.e. _arch_cycle will have some small value.  This
	 * value will drift AND may jump upward from time to time.  The
	 * drift is due to not having precise tracking between the two
	 * timers (the PIT and either the TSC or the PM timer) and the
	 * jump is caused by interrupt delays, cache misses etc.  We
	 * need to correct for the drift.  To correct all we need to do
	 * is to set "last_was_long" to zero and a new timer program
	 * will be started to "do the right thing".

	 * Detecting the need to do this correction is another issue.
	 * Here is what we do:
	 *
	 * Each interrupt where last_was_long is !=0 (indicates the
	 * interrupt should be on a 1/HZ boundry) we check the resulting
	 * _arch_cycle.  If it is smaller than some MIN value, we do the
	 * correction.  (Note that drift that makes the value smaller is
	 * the easy one.)  We also require that _arch_cycle <= some max
	 * at least once over a period of 1 second.  I.e.  with HZ =
	 * 100, we will allow up to 99 "late" interrupts before we do a
	 * correction.

	 * The values we use for min_hz_arch_cycle and max_hz_arch_cycle
	 * depend on the units and we will start by, during boot,
	 * observing what MIN appears to be.  We will set
	 * max_hz_arch_cycle to be about 100 machine cycles more than
	 * this.

	 * Note that with min_hz_arch_cycle and max_hz_arch_cycle set to
	 * 0, this code will reset the PIT every HZ.
	 */
#ifdef TIMER_NEEDS_DISCIPLINE
static inline void
discipline_timer(void)
{
	int *last_was_long = &__last_was_long;

	if (!*last_was_long)
		return;

	timer_discipline_diff = quick_get_cpuctr();
	while (timer_discipline_diff > arch_cycles_per_jiffy) {
		timer_discipline_diff -= arch_cycles_per_jiffy;
	}
	if (timer_discipline_diff < min_hz_arch_cycle) {
		*last_was_long = 0;
		return;
	}
	if (timer_discipline_diff <= max_hz_arch_cycle) {
		*last_was_long = 1;
		return;
	}
	if (++*last_was_long > HZ) {
		*last_was_long = 0;
		return;
	}
}
#else
#define discipline_timer()
#endif

#if defined(CONFIG_X86_LOCAL_APIC)
/*
 * This code will only be called as the first thing of a 1/HZ tick.  It
 * is only used when we can believe the PIT (i.e. we are not messing
 * with it).
 *
 * If ever we appear to be early, we immeadiatly correct.  When late, we 
 * require a consistant lateness for DRIFT_TIME ticks before making 
 * a correction.  A further possibility is to only take part of the late
 * correction.  Something to think about.
 */
static inline void
drift_reset(long reset) 
{
	last_update += reset;
}
#define DRIFT_TIME 50             /* in ticks */
#define LARGE_MIN_DRIFT 1000000000
#define MAX_DRIFT_TIME (LARGE_MIN_DRIFT + DRIFT_TIME)
static long drift_min;
static long drift_time;
static inline void
reset_fillin_timer(void)
{
 	long timer_reset_diff = quick_get_cpuctr() - arch_cycles_per_jiffy;

	if (!timer_reset_diff ) {
		drift_time = drift_min = LARGE_MIN_DRIFT;
		return;
	}	
	if (timer_reset_diff < 0) {
		/*
		 * Seem to be drifting to an earlier time.  Use it.
		 */
		drift_reset(timer_reset_diff);
		drift_time = drift_min = LARGE_MIN_DRIFT;
		return;
	}
	if (timer_reset_diff < drift_min )
		drift_min = timer_reset_diff;
	if (++drift_time > MAX_DRIFT_TIME) {
		/*
		 * consistantly late by drift_min over MAX_DRIFT_TIME
		 * adjust by that amount.
		 */
		drift_reset(drift_min);
		drift_time = drift_min = LARGE_MIN_DRIFT;
		return;
	}
}
#else
#define reset_fillin_timer()
#endif

#ifdef USE_APIC_TIMERS
#include <asm/apic.h>
/*
 * If we have a local APIC, we will use its counter to get the needed 
 * interrupts.  Here is where we program it.
 */
DECLARE_PER_CPU(int, prof_counter);

extern void __setup_APIC_LVTT(unsigned int);
static inline void
reload_timer_chip(int new_latch_value)
{
	int new_latch = arch_cycle_to_latch(new_latch_value);
	/*
	 * We may want to do more in line code for speed here.
	 * For now, however...

	 * Note: The PIT handles the 1/HZ interrupt.  We only do the 
	 * arch_cycle stuff.
	 */
	if (new_latch < timer_delta)
		new_latch = timer_delta;
	/*
	 * The profile counter may be set causing us to ignor (or 
	 * really just profile) the interrupt.  Force it to roll over
	 * and give us the interrupt.  This may cause a hic cup in
	 * the profile, but it will resume on the next tick.
	 * There are, clearly, more complicated ways to deal with
	 * profiling.
	 */
	per_cpu(prof_counter, smp_processor_id()) = 0;
	apic_write_around(APIC_TMICT, new_latch);
}


#else /* ! USE_APIC_TIMERS */
static inline void
reload_timer_chip(int new_latch_value)
{
	unsigned char pit_status;
	/*
	 * The input value is in arch cycles
	 * We must be called with irq disabled.
	 */

	new_latch_value = arch_cycle_to_latch(new_latch_value);
	if (new_latch_value < TIMER_DELTA) {
		new_latch_value = TIMER_DELTA;
	}
	spin_lock(&i8253_lock);
	outb_p(PIT0_PERIODIC, PIT_COMMAND);
	outb_p(new_latch_value & 0xff, PIT0);	/* LSB */
	outb(new_latch_value >> 8, PIT0);	/* MSB */
	do {
		outb_p(PIT0_LATCH_STATUS, PIT_COMMAND);
		pit_status = inb(PIT0);
	} while (pit_status & PIT_NULL_COUNT);
	outb_p(LATCH & 0xff, PIT0);	/* LSB */
	outb(LATCH >> 8, PIT0);	/* MSB */
	spin_unlock(&i8253_lock);
	return;
}
#endif				//  ! USE_APIC_TIMERS
/*
 * Time out for a discussion.  Because the PIT and TSC (or the PIT and
 * pm timer) may drift WRT each other, we need a way to get the jiffie
 * interrupt to happen as near to the jiffie roll as possible.  This
 * insures that we will get the interrupt when the timer is to be
 * delivered, not before (we would not deliver) or later, making the
 * jiffie timers different from the arch_cycle deliveries.  We would
 * also like any latency between a "requested" interrupt and the
 * automatic jiffie interrupts from the PIT to be the same.  Since it
 * takes some time to set up the PIT, we assume that requested
 * interrupts may be a bit late when compared to the automatic
 * interrupts.  When we request a jiffie interrupt, we want the
 * interrupt to happen at the requested time, which will be a bit before
 * we get to the jiffies update code. 
 *
 * What we want to determine here is a.) how long it takes (min) to get
 * from a requested interrupt to the jiffies update code and b.) how
 * long it takes when the interrupt is automatic (i.e. from the PIT
 * reset logic).  When we set "last_was_long" to zero, the next tick
 * setup code will "request" a jiffies interrupt (as long as we do not
 * have any arch_cycle timers pending).  The interrupt after the
 * requested one will be automatic.  Ignoring drift over this 2/HZ time
 * we then get two latency values, the requested latency and the
 * automatic latency.  We set up the difference to correct the requested
 * time and the second one as the center of a window which we will use
 * to detect the need to resync the PIT.  We do this for HZ ticks and
 * take the min.
 */
#ifdef TIMER_NEEDS_DISCIPLINE
#define NANOSEC_SYNC_LIMIT 2000	// Try for 2 usec. max drift
#define final_clock_init()						\
{ 									\
	unsigned long end = jiffies + HZ + HZ; 				\
	int min_a =  arch_cycles_per_jiffy;				\
        int min_b =  arch_cycles_per_jiffy;				\
        unsigned long seq;						\
        int * last_was_long = &__last_was_long;	                        \
									\
        while (time_before(jiffies,end)){ 				\
		unsigned long f_jiffies = jiffies;			\
                while (jiffies == f_jiffies);				\
                *last_was_long = 0;					\
                while (jiffies == f_jiffies + 1);			\
                do{							\
			seq = read_seqbegin(&xtime_lock); 		\
			if (  timer_discipline_diff < min_a) 		\
				min_a =   timer_discipline_diff; 	\
		}while (read_seqretry(&xtime_lock, seq)); 		\
		while (jiffies == f_jiffies + 2);			\
                do{ 							\
			seq = read_seqbegin(&xtime_lock);		\
                        if (  timer_discipline_diff < min_b) 		\
				min_b =   timer_discipline_diff; 	\
		}while (read_seqretry(&xtime_lock, seq)); 		\
	}                             					\
        min_hz_arch_cycle = min_b -  					\
		nsec_to_arch_cycle(NANOSEC_SYNC_LIMIT);			\
        if( min_hz_arch_cycle < 0)  					\
                min_hz_arch_cycle = 0; 					\
        max_hz_arch_cycle = min_b +  					\
		nsec_to_arch_cycle(NANOSEC_SYNC_LIMIT);			\
        timer_delta = 							\
		arch_cycle_to_latch(usec_to_arch_cycle(TIMER_DELTA));	\
}
#else
#define final_clock_init()
#endif				// TIMER_NEEDS_DISCIPLINE
#endif				/* __KERNEL__ */
#endif				/* _I386_HRTIME_H */
