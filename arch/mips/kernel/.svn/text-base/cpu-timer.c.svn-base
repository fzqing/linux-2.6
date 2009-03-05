/*
 * Copyright 2004 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * This routine provides time routines for boards that use cpu count/compare
 * as their system timer.  A couple of requirements:
 *   . Must have count/compare register and use them as your system timer
 *     (obviously)
 *   . Timer interrupt must go through do_IRQ() or ll_timer_interrupt()
 *   . You must know or calibrate cpu timer frequency.
 *
 * See more in Documentation/mips/time.README.
 *
 * Copyright 2004-2005 MontaVista Software Inc.
 * Author: Manish Lachwani (mlachwani@mvista.com)
 *
 * HRT and Count/Compare Support for SMP - Manish Lachwani (mlachwani@mvista.com)
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/smp.h>
#include <linux/kernel_stat.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/div64.h>
#include <asm/hardirq.h>
#include <asm/sections.h>
#include <asm/time.h>
#include <asm/debug.h>
#include <asm/compiler.h>
#include <linux/hrtime.h>

/*
 * The integer part of the number of usecs per jiffy is taken from tick,
 * but the fractional part is not recorded, so we calculate it using the
 * initial value of HZ.  This aids systems where tick isn't really an
 * integer (e.g. for HZ = 128).
 */
#define USECS_PER_JIFFY		TICK_SIZE

#define TICK_SIZE	(tick_nsec / 1000)

u64 jiffies_64 = INITIAL_JIFFIES;

EXPORT_SYMBOL(jiffies_64);

#ifdef CONFIG_SMP
/*
 * Master processor indicates that all slaves have
 * sync'ed the count with the it
 */
static int smp_c0_counts_synchronized;
#else
#define smp_c0_counts_synchronized	1
#endif

/*
 * forward reference
 */
extern volatile unsigned long wall_jiffies;

/*
 * ICK - we can't use time_after and its friends because it only operates
 * on unsigned long type.  What MIPS needs is similar macros for u32 type.
 */
#define u32_time_after(a,b)		\
	(typecheck(u32, a) && \
	 typecheck(u32, b) && \
	 ((s32)(b) - (s32)(a) < 0))
#define u32_time_before(a,b)	u32_time_after(b,a)

#define u32_time_after_eq(a,b)	\
	(typecheck(u32, a) && \
	 typecheck(u32, b) && \
	 ((s32)(a) - (s32)(b) >= 0))
#define u32_time_before_eq(a,b)	u32_time_after_eq(b,a)

/*
 * By default we provide the null RTC ops
 */
static unsigned long null_rtc_get_time(void)
{
	return mktime(2000, 1, 1, 0, 0, 0);
}

static int null_rtc_set_time(unsigned long sec)
{
	return 0;
}

unsigned long (*rtc_get_time)(void) = null_rtc_get_time;
int (*rtc_set_time)(unsigned long) = null_rtc_set_time;
int (*rtc_set_mmss)(unsigned long);


/* usecs per counter cycle, shifted to left by 32 bits */
static u32 sll32_usecs_per_cycle;

/* how many counter cycles in a jiffy */
unsigned long cycles_per_jiffy;

/* Cycle counter value at the last timer interrupt.. */
u32 last_count;

/* last time when xtime and rtc are sync'ed up */
static long last_rtc_update;

/* any missed timer interrupts */
int missed_timer_count;

/*
 * Gettimeoffset routines.  These routines returns the time duration
 * since last timer interrupt in usecs.
 */
static unsigned long get_intra_jiffy_offset(void)
{
	u32 count;
	unsigned long res;

	/* Get last timer tick in absolute kernel time */
#if !defined(CONFIG_SOC_PNX8550)
	count = read_c0_count();

	/* 
	 * .. relative to previous jiffy (32 bits is enough).
	 * This routine should be protected by xtime_lock.  No race condition.
	 * In SMP case, count may occasionally be behind last_count.
	 */ 
	if (u32_time_after(count, last_count - (u32)cycles_per_jiffy))	 
	       count -= last_count-cycles_per_jiffy; 	
	else
		count = 0;
#else
	count = read_c0_count2();

	if (u32_time_after(count, last_count))	 
		count -= last_count;
	else
		count = 0;
#endif

	if (!smp_c0_counts_synchronized)
		count = 0;

	__asm__("multu	%1,%2"
		: "=h" (res)
		: "r" (count), "r" (sll32_usecs_per_cycle)
		: "lo", GCC_REG_ACCUM);

	/*
	 * Due to possible jiffies inconsistencies, we need to check
	 * the result so that we'll get a timer that is monotonic.
	 */
	if (res >= USECS_PER_JIFFY)
		res = USECS_PER_JIFFY;

	return res;
}

/*
 * This version of gettimeofday has better than microsecond precision.
 */
void do_gettimeofday(struct timeval *tv)
{
	unsigned long seq;
	unsigned long lost;
	unsigned long usec, sec;
	unsigned long max_ntp_tick;

	do {
		seq = read_seqbegin(&xtime_lock);
		usec = get_intra_jiffy_offset();
		lost = jiffies - wall_jiffies;

		/*
		 * If time_adjust is negative then NTP is slowing the clock
		 * so make sure not to go into next possible interval.
		 * Better to lose some accuracy than have time go backwards..
		 */
		if (unlikely(time_adjust < 0)) {
			max_ntp_tick = (USEC_PER_SEC / HZ) - tickadj;
			usec = min(usec, max_ntp_tick);

			if (lost)
				usec += lost * max_ntp_tick;
		} else if (unlikely(lost))
			usec += lost * (USEC_PER_SEC / HZ);

		sec = xtime.tv_sec;
		usec += (xtime.tv_nsec / 1000);
	} while (read_seqretry(&xtime_lock, seq));

	while (usec >= 1000000) {
		usec -= 1000000;
		sec++;
	}
	
	tv->tv_sec = sec;
	tv->tv_usec = usec;
}

EXPORT_SYMBOL(do_gettimeofday);

int do_settimeofday(struct timespec *tv)
{
	time_t wtm_sec, sec = tv->tv_sec;
	long wtm_nsec, nsec = tv->tv_nsec;

	if ((unsigned long)tv->tv_nsec >= NSEC_PER_SEC)
		return -EINVAL;

	write_seqlock_irq(&xtime_lock);

	/*
	 * This is revolting.  We need to set "xtime" correctly.  However,
	 * the value in this location is the value at the most recent update
	 * of wall time.  Discover what correction gettimeofday() would have
	 * made, and then undo it!
	 */
	nsec -= get_intra_jiffy_offset() * NSEC_PER_USEC;
	nsec -= (jiffies - wall_jiffies) * tick_nsec;

	wtm_sec  = wall_to_monotonic.tv_sec + (xtime.tv_sec - sec);
	wtm_nsec = wall_to_monotonic.tv_nsec + (xtime.tv_nsec - nsec);

	set_normalized_timespec(&xtime, sec, nsec);
	set_normalized_timespec(&wall_to_monotonic, wtm_sec, wtm_nsec);

	time_adjust = 0;			/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;

	write_sequnlock_irq(&xtime_lock);
	clock_was_set();
	return 0;
}

EXPORT_SYMBOL(do_settimeofday);

/* High resolution timer stuff */
#ifdef CONFIG_HIGH_RES_TIMERS

int hr_time_resolution = CONFIG_HIGH_RES_RESOLUTION;
int get_arch_cycles(unsigned long ref_jiffies)
{
	int ret, diff;

	/*
	 * We have read lock on xtime, and interrupt is off.
	 * Therefore, jiffies and last_count should be consistent to each
	 * other.
	 */

#if !defined(CONFIG_SOC_PNX8550)
	ret = read_c0_count() - last_count + cycles_per_jiffy;
	if (ret < 0) {
		/* this should happen very rarely on smp with small diff */
		if (ret > -1000) 
			ret = 0;
	}

	/*
	 * Before c0 counts are synchronized, we stick with jiffy resolution
	 */
	if (!smp_c0_counts_synchronized)
		ret = 0;

	if (unlikely(diff = jiffies - ref_jiffies))
	ret += diff * arch_cycles_per_jiffy;
#else
	unsigned int temp_jiffies;

	do {
		/* snapshot jiffies */
		temp_jiffies = jiffies;
		barrier();

		ret = read_c0_count2() - last_count;

		if (unlikely(diff = jiffies - ref_jiffies))
			ret += diff * arch_cycles_per_jiffy;

		barrier();
		/* repeat if we didn't have a consistent view of the world */
	} while (unlikely(temp_jiffies != jiffies));
#endif
	return ret;
}

static inline unsigned int
calc_next_expire(unsigned long ref_jiffies, u32 cycles)
{
	u32 last_count1;
	unsigned long jiffies1, seq;

	/* make a consistent read of last_count and jiffies */
	do {
		seq = read_seqbegin(&xtime_lock);
		last_count1 = last_count;
		jiffies1 = jiffies;
	} while (read_seqretry(&xtime_lock, seq));

	return last_count1 + cycles +
		(ref_jiffies - jiffies1 - 1) * arch_cycles_per_jiffy;
}

int schedule_hr_timer_int(unsigned long ref_jiffies, int cycles)
{
	u32 count;

#if !defined(CONFIG_SOC_PNX8550)
	count = calc_next_expire(ref_jiffies, (u32)cycles);

	if (u32_time_after((u32)read_c0_count(), count))
		return -ETIME;

	write_c0_compare(count);
	return 0;
#else
	u32 jiffies_f = jiffies;
	
	count = (ref_jiffies - jiffies_f) * arch_cycles_per_jiffy +
		cycles - get_arch_cycles(jiffies);

	if ((long) (ref_jiffies - jiffies_f) <= 0 && (long) count <  0)
		return -ETIME;

	write_c0_count3(0);
	write_c0_compare3(count);
	/* Timer 3 start */
	write_c0_config7(read_c0_config7() & ~0x00000020);

	return 0;
#endif
}

static inline void schedule_timer_int_asap(void)
{
	unsigned inc = 0;
	unsigned inc_base=2;
	u32 compare;

	do {
		inc += inc_base;
		compare = read_c0_count() + inc;
		write_c0_compare(compare);
	} while (u32_time_after((u32)read_c0_count(), compare));
}

int schedule_jiffies_int(unsigned long ref_jiffies)
{
#if !defined(CONFIG_SOC_PNX8550)
	int ret;
	u32 count;

	/* we ignore this scheduling hint on all CPUs except CPU 0 */
	if (smp_processor_id())
		return get_arch_cycles(ref_jiffies) >= arch_cycles_per_jiffy;

	count = calc_next_expire(ref_jiffies, (u32)arch_cycles_per_jiffy);
	write_c0_compare(count);
	ret = u32_time_after((u32)read_c0_count(), count);

	if (ret)
		schedule_timer_int_asap();

	return ret;
#else
	return get_arch_cycles(ref_jiffies) >= arch_cycles_per_jiffy;
#endif
}

/*
 * return (u32)( ((u64)x * (u64)y) >> shift ), where 0 < shift <= 32
 */
static inline int scaled_mult(int x, int y, int shift)
{
	int hi;
	unsigned lo;

	__asm__("mult\t%2,%3\n\t"
		"mfhi\t%0\n\t"
		"mflo\t%1"
		:"=r" (hi), "=r" (lo)
		:"r" (x), "r" (y));

	if (shift == 32)
		return hi;
	else
		return (hi << (32 - shift)) | (lo >> shift);
}

#define nsec2cycle_shift	32
#define cycle2nsec_shift	24
unsigned scaled_cycles_per_nsec, scaled_nsecs_per_cycle;

int nsec_to_arch_cycle(int nsec)
{
	return scaled_mult(nsec, scaled_cycles_per_nsec, nsec2cycle_shift);
}

int arch_cycle_to_nsec(int cycles)
{
	return scaled_mult(cycles, scaled_nsecs_per_cycle, cycle2nsec_shift);
}

static void hr_time_init(void)
{
	u64 temp;

	/*
	* the current setting allow counter freq to range from
	* a few MHz to 1GHz.
	*/
	temp = (u64)mips_hpt_frequency << nsec2cycle_shift;
	do_div(temp, 1000000000);
	scaled_cycles_per_nsec = (unsigned)temp;

	temp = (u64) 1000000000 << cycle2nsec_shift;
	do_div(temp, mips_hpt_frequency);
	scaled_nsecs_per_cycle = (unsigned)temp;
}

#endif /* CONFIG_HIGH_RES_TIMERS */

/*
 * local_timer_interrupt() does profiling and process accounting
 * on a per-CPU basis.
 *
 * In UP mode, it is invoked from the (global) timer_interrupt.
 *
 * In SMP mode, it might invoked by per-CPU timer interrupt, or
 * a broadcasted inter-processor interrupt which itself is triggered
 * by the global timer interrupt.
 */
void local_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	if (current->pid)
		profile_tick(CPU_PROFILING, regs);

	update_process_times(user_mode(regs));
}

/*
 * Timer interrupt service routines.  This function
 * is set as irqaction->handler and is invoked through do_IRQ.
 */
irqreturn_t timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	 u32 count;  
#if defined(CONFIG_HIGH_RES_TIMERS) && !defined(CONFIG_SOC_PNX8550)
	/* Non-zero CPUs only have hrt interrupts */
	if (smp_processor_id()) {
		write_c0_compare(read_c0_compare());    /* ack the interrupt */
		do_hr_timer_int();
		return IRQ_HANDLED;
	}
#endif
	write_seqlock(&xtime_lock);
	{

		int loops=0;
		for(;;) {
#if !defined(CONFIG_SOC_PNX8550)
			write_c0_compare(last_count);
			count = read_c0_count();
			if (u32_time_before(count, last_count))
			break;
#else
			write_c0_compare(cycles_per_jiffy);
#endif
			do_timer(regs);
			local_timer_interrupt(irq, dev_id, regs);

			loops++;
#if !defined(CONFIG_SOC_PNX8550)
			last_count += cycles_per_jiffy;
#else
			count = read_c0_count(); // latency during the interrupt
			last_count = read_c0_count2() - count;
			break;
#endif
		}
#if defined(CONFIG_HIGH_RES_TIMERS) && !defined(CONFIG_SOC_PNX8550)
		/* if we are not doing jiffy timer, we must be doing hr timer */
		if (!loops) {
			do_hr_timer_int();
			write_sequnlock(&xtime_lock);
			return IRQ_HANDLED;
		}
		else
#endif
		missed_timer_count += loops - 1;
	}

	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. rtc_set_time() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */
	if ((time_status & STA_UNSYNC) == 0 &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    (xtime.tv_nsec / 1000) >= 500000 - ((unsigned) TICK_SIZE) / 2 &&
	    (xtime.tv_nsec / 1000) <= 500000 + ((unsigned) TICK_SIZE) / 2) {
		if (rtc_set_mmss(xtime.tv_sec) == 0) {
			last_rtc_update = xtime.tv_sec;
		} else {
			/* do it again in 60 s */
			last_rtc_update = xtime.tv_sec - 600;
		}
	}

	write_sequnlock(&xtime_lock);

	/*
	 * We call local_timer_interrupt() to do profiling and process 
	 * accouting.
	 */

#ifdef CONFIG_SMP
	{
		int cpus = 0;
		/*
		 * Signal the second core to do the same
		 */
		for_each_online_cpu(cpus) {
			core_send_ipi(cpus, SMP_LOCAL_TIMER);
		}
	}
#endif

	return IRQ_HANDLED;
}

asmlinkage void ll_timer_interrupt(int irq, struct pt_regs *regs)
{
	irq_enter();
	kstat_this_cpu.irqs[irq]++;

	/* we keep interrupt disabled all the time */
	timer_interrupt(irq, NULL, regs);

	irq_exit();
}

#ifdef CONFIG_SMP
/*
 * We have to synchronize the master CPU with all the slave CPUs
 */
static atomic_t cpus_started;
static atomic_t cpus_ready;
static atomic_t cpus_count;

/*
 * Master processor inits
 */
static void sync_cpus_init(int v)
{
	atomic_set(&cpus_count, 0);
	mb();
	atomic_set(&cpus_started, v);
	mb();
	atomic_set(&cpus_ready, v);
	mb();
}

/*
 * Called by the master processor
 */
static void sync_cpus_master(int v)
{
	atomic_set(&cpus_count, 0);
	mb();
	atomic_set(&cpus_started, v);
	mb();

	/* Wait here till all other CPUs are now ready */
	while (atomic_read(&cpus_count) != (num_online_cpus() -1) )
		mb();

	atomic_set(&cpus_ready, v);
	mb();
}

/*
 * Called by the slave processors
 */
static void sync_cpus_slave(int v)
{
	/* Check if the master has been through this */
	while (atomic_read(&cpus_started) != v)
		mb();

	atomic_inc(&cpus_count);
	mb();

	while (atomic_read(&cpus_ready) != v)
		mb();
}

/*
 * Called by the slave CPUs when done syncing the count register 
 * with the master processor
 */
static void sync_cpus_slave_exit(int v)
{
	while (atomic_read(&cpus_started) != v)
		mb();

	atomic_inc(&cpus_count);
	mb();
}

#ifdef CONFIG_CPU_CAVIUM_OCTEON
#define LOOPS	100
#else
#define LOOPS	20 			/* More loops, more precision */
#endif

static u32 c0_count[NR_CPUS]; 		/* Count register per CPU */
static u32 c[NR_CPUS][LOOPS + 1]; 	/* Count register per CPU per loop for syncing */

/*
 * Slave processors execute this via IPI 
 */
static void sync_c0_count_slave(void *info)
{
	int cpus = 1, loop, prev_count = 0, cpu = smp_processor_id();
	unsigned long flags;
	u32 diff_count; /* CPU count registers are 32-bit */

	local_irq_save(flags);

	for(loop = 0; loop <= LOOPS; loop++) {
		/* Sync with the Master processor */
		sync_cpus_slave(cpus++);

		c[cpu][loop] = c0_count[cpu] = read_c0_count();
		mb();

		sync_cpus_slave(cpus++);
		
		diff_count = c0_count[0] - c0_count[cpu];
		diff_count += prev_count;
		diff_count += read_c0_count();
		write_c0_count(diff_count);

		prev_count = (prev_count >> 1) + 
				((int)(c0_count[0] - c0_count[cpu]) >> 1);
	}

	/* Slave processor is done syncing count register with Master */
	sync_cpus_slave_exit(cpus++);
	
	printk("SMP: Slave processor %d done syncing count \n", cpu);

	local_irq_restore(flags);
}	

/*
 * Master kicks off the syncing process
 */
void sync_c0_count_master(void)
{
	int cpus = 0, loop, cpu = smp_processor_id();
	unsigned long flags;

	printk("SMP: Starting to sync the c0 count register ... \n");

	sync_cpus_init(cpus++);

	/* Kick off the slave processors to also start the syncing process */
	smp_call_function(sync_c0_count_slave, NULL, 0, 0);

	local_irq_save(flags);

	for (loop = 0; loop <= LOOPS; loop++) {
		/* Wait for all the CPUs here */
		sync_cpus_master(cpus++);

		c[cpu][loop] = c0_count[cpu] = read_c0_count();
		mb();

		/* Do syncing once more */
		sync_cpus_master(cpus++);
	}

	sync_cpus_master(cpus++);
	local_irq_restore(flags);

	/* Now that counts have been sync'ed */
	smp_c0_counts_synchronized = 1;
	
	printk("SMP: Syncing process completed accross CPUs ... \n");
}

#endif /* CONFIG_SMP */

/*
 * time_init() - it does the following things.
 *
 * .) board_time_init() (or in board setup routine) -
 * 	a) set up RTC routines,
 *      b) calibrate and set the mips_hpt_frequency
 * .) set rtc_set_mmss if it is not set by board code
 * .) setup xtime based on rtc_get_time().
 * .) init walt_to_monotonic
 * .) calculate a couple of cached variables for later usage
 * .) board_timer_setup() -
 * 	. If you use ll_timer_interrupt(), do
 *			set_c0_status(IE_IRQ5);
 *		
 *	. Otherwise if you are using IRQ_CPU, do
 *		setup_irq(CPU_IRQ_BASE + 7, irq)
 *
 *	. If you are not using ll_timer_interrupt() (i.e., go through
 *	  do_IRQ()) and you are not using IRQ_CPU, you can work around,
 *	  but you probably really should ask yourself why.
 */

void (*board_time_init)(void);
void (*board_timer_setup)(struct irqaction *irq);

unsigned int mips_hpt_frequency;

static struct irqaction timer_irqaction = {
	.handler = timer_interrupt,
	.flags = SA_NODELAY | SA_INTERRUPT,
	.name = "timer",
};

void __init time_init(void)
{
	if (board_time_init)
		board_time_init();

	if (!rtc_set_mmss)
		rtc_set_mmss = rtc_set_time;

	xtime.tv_sec = rtc_get_time();
	xtime.tv_nsec = 0;

	set_normalized_timespec(&wall_to_monotonic,
	                        -xtime.tv_sec, -xtime.tv_nsec);

	/* Calculate cache parameters.  */
	cycles_per_jiffy = (mips_hpt_frequency + HZ / 2) / HZ;

	{ 
		u64 div = ((u64)1000000 << 32) + mips_hpt_frequency / 2;
		do_div(div, mips_hpt_frequency);
		sll32_usecs_per_cycle = div;
	}

	/* Report the high precision timer rate for a reference.  */
	printk("Using %u.%03u MHz cpu timer.\n",
		       ((mips_hpt_frequency + 500) / 1000) / 1000,
		       ((mips_hpt_frequency + 500) / 1000) % 1000);

	/* initialize cp0 count and compare */
	write_c0_compare(cycles_per_jiffy);
	write_c0_count(0);
	last_count = 0;

	/*
	 * Call board specific timer interrupt setup.
	 *
	 * this pointer must be setup in machine setup routine.
	 *
	 * Even if a machine chooses to use a low-level timer interrupt,
	 * it still needs to setup the timer_irqaction.
	 * In that case, it might be better to set timer_irqaction.handler
	 * to be NULL function so that we are sure the high-level code
	 * is not invoked accidentally.
	 */
	board_timer_setup(&timer_irqaction);


#ifdef CONFIG_HIGH_RES_TIMERS
       if ( (read_c0_status() & CAUSEF_IP7) == 0)
               panic("Enabling high res timers without using CPU counter!");

       hr_time_init();
#endif
}

#define FEBRUARY		2
#define STARTOFTIME		1970
#define SECDAY			86400L
#define SECYR			(SECDAY * 365)
#define leapyear(y)		((!((y) % 4) && ((y) % 100)) || !((y) % 400))
#define days_in_year(y)		(leapyear(y) ? 366 : 365)
#define days_in_month(m)	(month_days[(m) - 1])

static int month_days[12] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

void to_tm(unsigned long tim, struct rtc_time *tm)
{
	long hms, day, gday;
	int i;

	gday = day = tim / SECDAY;
	hms = tim % SECDAY;

	/* Hours, minutes, seconds are easy */
	tm->tm_hour = hms / 3600;
	tm->tm_min = (hms % 3600) / 60;
	tm->tm_sec = (hms % 3600) % 60;

	/* Number of years in days */
	for (i = STARTOFTIME; day >= days_in_year(i); i++)
		day -= days_in_year(i);
	tm->tm_year = i;

	/* Number of months in days left */
	if (leapyear(tm->tm_year))
		days_in_month(FEBRUARY) = 29;
	for (i = 1; day >= days_in_month(i); i++)
		day -= days_in_month(i);
	days_in_month(FEBRUARY) = 28;
	tm->tm_mon = i - 1;		/* tm_mon starts from 0 to 11 */

	/* Days are what is left over (+1) from all that. */
	tm->tm_mday = day + 1;

	/*
	 * Determine the day of week
	 */
	tm->tm_wday = (gday + 4) % 7;	/* 1970/1/1 was Thursday */
}

EXPORT_SYMBOL(to_tm);
EXPORT_SYMBOL(rtc_set_time);
EXPORT_SYMBOL(rtc_get_time);

unsigned long long sched_clock(void)
{ 
	return (unsigned long long)jiffies*(1000000000/HZ);
} 

