/*
 *  linux/arch/i386/kernel/time.c
 *
 *  Copyright (C) 1991, 1992, 1995  Linus Torvalds
 *
 * This file contains the PC-specific time handling details:
 * reading the RTC at bootup, etc..
 * 1994-07-02    Alan Modra
 *	fixed set_rtc_mmss, fixed time.year for >= 2000, new mktime
 * 1995-03-26    Markus Kuhn
 *      fixed 500 ms bug at call to set_rtc_mmss, fixed DS12887
 *      precision CMOS clock update
 * 1996-05-03    Ingo Molnar
 *      fixed time warps in do_[slow|fast]_gettimeoffset()
 * 1997-09-10	Updated NTP code according to technical memorandum Jan '96
 *		"A Kernel Model for Precision Timekeeping" by Dave Mills
 * 1998-09-05    (Various)
 *	More robust do_fast_gettimeoffset() algorithm implemented
 *	(works with APM, Cyrix 6x86MX and Centaur C6),
 *	monotonic gettimeofday() with fast_get_timeoffset(),
 *	drift-proof precision TSC calibration on boot
 *	(C. Scott Ananian <cananian@alumni.princeton.edu>, Andrew D.
 *	Balsa <andrebalsa@altern.org>, Philip Gladstone <philip@raptor.com>;
 *	ported from 2.0.35 Jumbo-9 by Michael Krause <m.krause@tu-harburg.de>).
 * 1998-12-16    Andrea Arcangeli
 *	Fixed Jumbo-9 code in 2.1.131: do_gettimeofday was missing 1 jiffy
 *	because was not accounting lost_ticks.
 * 1998-12-24 Copyright (C) 1998  Andrea Arcangeli
 *	Fixed a xtime SMP race (we need the xtime_lock rw spinlock to
 *	serialize accesses to xtime/lost_ticks).
 */
/* 2002-8-13 George Anzinger  Modified for High res timers: 
 *                            Copyright (C) 2002 MontaVista Software
*/
#define _INCLUDED_FROM_TIME_C
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/bcd.h>
#include <linux/efi.h>

#include <asm/io.h>
#include <asm/smp.h>
#include <asm/irq.h>
#include <asm/msr.h>
#include <asm/delay.h>
#include <asm/mpspec.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/timer.h>

#include "mach_time.h"

#include <linux/timex.h>
#include <linux/config.h>

#include <asm/hpet.h>

#include <asm/arch_hooks.h>
#include <linux/hrtime.h>

#include "io_ports.h"

#include <asm/i8259.h>

int pit_latch_buggy;              /* extern */

#include "do_timer.h"

u64 jiffies_64 = INITIAL_JIFFIES;

EXPORT_SYMBOL(jiffies_64);

unsigned long cpu_khz;	/* Detected as we calibrate the TSC */

extern unsigned long wall_jiffies;

DEFINE_RAW_SPINLOCK(rtc_lock);

#include <asm/i8253.h>

DEFINE_RAW_SPINLOCK(i8253_lock);
EXPORT_SYMBOL(i8253_lock);

struct timer_opts *cur_timer = &timer_none;
/*
 * This version of gettimeofday has microsecond resolution
 * and better than microsecond precision on fast x86 machines with TSC.
 */

/*
 * High res timers changes: First we want to use full nsec for all
 * the math to avoid the double round off (on the offset and xtime).
 * Second, we want to allow a boot with HRT turned off at boot time.
 * This will cause hrtimer_use to be false, and we then fall back to 
 * the old code.  We also shorten the xtime lock region and eliminate
 * the lost tick code as this kernel will never have lost ticks under
 * the lock (i.e. wall_jiffies will never differ from jiffies except
 * when the write xtime lock is held).
 */
void do_gettimeofday(struct timeval *tv)
{
	unsigned long seq;
	unsigned long sec, nsec, clk_nsec;
	unsigned long max_ntp_tick;

	do {
		seq = read_seqbegin(&xtime_lock);
#ifdef CONFIG_HIGH_RES_TIMERS
		if (hrtimer_use) 
			nsec = arch_cycle_to_nsec(get_arch_cycles(wall_jiffies));
		else 
#endif
			nsec = cur_timer->get_offset() * NSEC_PER_USEC;
		

		sec = xtime.tv_sec;
		clk_nsec = xtime.tv_nsec;
		max_ntp_tick = current_tick_length() >> (SHIFT_SCALE - 10);
	} while (read_seqretry(&xtime_lock, seq));

	/* ensure we don't advance beyond the current tick length */
	nsec = min(nsec, max_ntp_tick);

	nsec += clk_nsec;
				
	while (nsec >= NSEC_PER_SEC) {
		nsec -=  NSEC_PER_SEC;
		sec++;
	}

	tv->tv_sec = sec;
	tv->tv_usec = nsec / NSEC_PER_USEC;
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
	 * This is revolting. We need to set "xtime" correctly. However, the
	 * value in this location is the value at the most recent update of
	 * wall time.  Discover what correction gettimeofday() would have
	 * made, and then undo it!
	 */
	nsec -= cur_timer->get_offset() * NSEC_PER_USEC;
	nsec -= (jiffies - wall_jiffies) * TICK_NSEC;

	wtm_sec  = wall_to_monotonic.tv_sec + (xtime.tv_sec - sec);
	wtm_nsec = wall_to_monotonic.tv_nsec + (xtime.tv_nsec - nsec);

	set_normalized_timespec(&xtime, sec, nsec);
	set_normalized_timespec(&wall_to_monotonic, wtm_sec, wtm_nsec);

	time_adjust = 0;		/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;
	write_sequnlock_irq(&xtime_lock);
	clock_was_set();
	return 0;
}

EXPORT_SYMBOL(do_settimeofday);

static int set_rtc_mmss(unsigned long nowtime)
{
	int retval;

	/* gets recalled with irq locally disabled */
	spin_lock(&rtc_lock);
	if (efi_enabled)
		retval = efi_set_rtc_mmss(nowtime);
	else
		retval = mach_set_rtc_mmss(nowtime);
	spin_unlock(&rtc_lock);

	return retval;
}

/* last time the cmos clock got updated */
static long last_rtc_update;

int timer_ack;

/* monotonic_clock(): returns # of nanoseconds passed since time_init()
 *		Note: This function is required to return accurate
 *		time even in the absence of multiple timer ticks.
 */
unsigned long long monotonic_clock(void)
{
	return cur_timer->monotonic_clock();
}
EXPORT_SYMBOL(monotonic_clock);

#if defined(CONFIG_SMP) && defined(CONFIG_FRAME_POINTER)
unsigned long notrace profile_pc(struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);

	if (in_lock_functions(pc))
		return *(unsigned long *)(regs->ebp + 4);

	return pc;
}
EXPORT_SYMBOL(profile_pc);
#endif

/*
 * timer_interrupt() needs to keep up the real-time clock,
 * as well as call the "do_timer()" routine every clocktick
 */
static inline void do_timer_interrupt(struct pt_regs *regs)
{
#ifdef CONFIG_X86_IO_APIC
	if (timer_ack) {
		unsigned long flags;
		/*
		 * Subtle, when I/O APICs are used we have to ack timer IRQ
		 * manually to reset the IRR bit for do_slow_gettimeoffset().
		 * This will also deassert NMI lines for the watchdog if run
		 * on an 82489DX-based system.
		 */
		spin_lock_irqsave(&i8259A_lock, flags);
		outb(0x0c, PIC_MASTER_OCW3);
		/* Ack the IRQ; AEOI will end it automatically. */
		inb(PIC_MASTER_POLL);
		spin_unlock_irqrestore(&i8259A_lock, flags);
	}
#endif
	do_timer_interrupt_hook(regs);

	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */
	if ((time_status & STA_UNSYNC) == 0 &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    (xtime.tv_nsec / 1000)
			>= USEC_AFTER - ((unsigned) TICK_SIZE) / 2 &&
	    (xtime.tv_nsec / 1000)
			<= USEC_BEFORE + ((unsigned) TICK_SIZE) / 2) {
		/* horrible...FIXME */
		if (efi_enabled) {
	 		if (efi_set_rtc_mmss(xtime.tv_sec) == 0)
				last_rtc_update = xtime.tv_sec;
			else
				last_rtc_update = xtime.tv_sec - 600;
		} else if (set_rtc_mmss(xtime.tv_sec) == 0)
			last_rtc_update = xtime.tv_sec;
		else
			/* do it again in 60 s */
			last_rtc_update = xtime.tv_sec - 600; 
	}

}

/*
 * This is the same as the above, except we _also_ save the current
 * Time Stamp Counter value at the time of the timer interrupt, so that
 * we later on can estimate the time of day more exactly.
 */
irqreturn_t timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 * Here we are in the timer irq handler. We just have irqs locally
	 * disabled but we don't know if the timer_bh is running on the other
	 * CPU. We need to avoid to SMP race with it. NOTE: we don' t need
	 * the irq version of write_lock because as just said we have irq
	 * locally disabled. -arca
	 */
 
	do_timer_interrupt(regs);
#ifdef CONFIG_MCA
        /*
         * This code moved here from do_timer_interrupt() as part of the
         * high-res timers change because it should be done every interrupt
         * but do_timer_interrupt() wants to return early if it is not a 
         * "1/HZ" tick interrupt.  For non-high-res systems the code is in
         * exactly the same location (i.e. it is moved from the tail of the
         * above called function to the next thing after the function).
         */
	if( MCA_bus ) {
		int irq;
		/* The PS/2 uses level-triggered interrupts.  You can't
		turn them off, nor would you want to (any attempt to
		enable edge-triggered interrupts usually gets intercepted by a
		special hardware circuit).  Hence we have to acknowledge
		the timer interrupt.  Through some incredibly stupid
		design idea, the reset for IRQ 0 is done by setting the
		high bit of the PPI port B (0x61).  Note that some PS/2s,
		notably the 55SX, work fine if this is removed.  */

		irq = inb_p( 0x61 );	/* read the current state */
		outb_p( irq|0x80, 0x61 );	/* reset the IRQ */
	}
#endif
	return IRQ_HANDLED;
}
#ifdef CONFIG_HIGH_RES_TIMERS
/*
 * We always continue to provide interrupts even if they are not
 * serviced.  To do this, we leave the chip in periodic mode programmed
 * to interrupt every jiffie.  This is done by, for short intervals,
 * programming a short time, waiting till it is loaded and then
 * programming the 1/HZ.  The chip will not load the 1/HZ count till the
 * short count expires.  If the last interrupt was programmed to be
 * short, we need to program another short to cover the remaining part
 * of the jiffie and can then just leave the chip alone.  Note that it
 * is also a low overhead way of doing things as we do not have to mess
 * with the chip MOST of the time. 
 */
#ifdef USE_APIC_TIMERS
int _schedule_jiffies_int(unsigned long jiffie_f)
{
	long past;
	unsigned long seq;
	if (unlikely(!hrtimer_use)) return 0;
	do {
		seq = read_seqbegin(&xtime_lock);
		past = get_arch_cycles(jiffie_f);
	} while (read_seqretry(&xtime_lock, seq));

	return (past >= arch_cycles_per_jiffy); 
}
#else 
int _schedule_jiffies_int(unsigned long jiffie_f)
{
	int rtn;
	if (!hrtimer_use || __last_was_long) return 0;

	rtn = _schedule_next_int(jiffie_f, arch_cycles_per_jiffy);
	if (unlikely(!__last_was_long))
		/*
		 * We need to force a timer interrupt here.  Timer chip code
		 * will boost the 1 to some min. value.
		 */
		reload_timer_chip(1);
	return rtn;
}
#endif
int _schedule_next_int(unsigned long jiffie_f,long arch_cycle_in)
{
	long arch_cycle_offset; 
	unsigned long seq;
	/* 
	 * First figure where we are in time. 
	 * A note on locking.  We are under the timerlist_lock here.  This
	 * means that interrupts are off already, so don't use irq versions.
	 */
	if (unlikely(!hrtimer_use)){
		return 0;
	}
	do {
		seq = read_seqbegin(&xtime_lock);
		arch_cycle_offset = arch_cycle_in - get_arch_cycles(jiffie_f);
	} while (read_seqretry(&xtime_lock, seq));
	/*
	 * If time is already passed, just return saying so.
	 */
	if (arch_cycle_offset <= 0)
		return 1;

	__last_was_long = arch_cycles_per_jiffy == arch_cycle_in;
	reload_timer_chip(arch_cycle_offset);
	return 0;
}

#ifdef CONFIG_APM
void restart_timer(void)
{
        start_PIT();
}
#endif /* CONFIG_APM */
#endif /* CONFIG_HIGH_RES_TIMERS */


/* not static: needed by APM */

unsigned long get_cmos_time(void)
{
	unsigned long retval;

	spin_lock(&rtc_lock);

	if (efi_enabled)
		retval = efi_get_time();
	else
		retval = mach_get_cmos_time();

	spin_unlock(&rtc_lock);

	return retval;
}

static long clock_cmos_diff, sleep_start;

static int timer_suspend(struct sys_device *dev, u32 state)
{
	/*
	 * Estimate time zone so that set_time can update the clock
	 */
	clock_cmos_diff = -get_cmos_time();
	clock_cmos_diff += get_seconds();
	sleep_start = get_cmos_time();
	return 0;
}

static int timer_resume(struct sys_device *dev)
{
	unsigned long flags;
	unsigned long sec;
	unsigned long sleep_length;

#ifdef CONFIG_HPET_TIMER
	if (is_hpet_enabled())
		hpet_reenable();
#endif
	sec = get_cmos_time() + clock_cmos_diff;
	sleep_length = get_cmos_time() - sleep_start;
	write_seqlock_irqsave(&xtime_lock, flags);
	xtime.tv_sec = sec;
	xtime.tv_nsec = 0;
	write_sequnlock_irqrestore(&xtime_lock, flags);
	jiffies += sleep_length * HZ;
	return 0;
}

static struct sysdev_class timer_sysclass = {
	.resume = timer_resume,
	.suspend = timer_suspend,
	set_kset_name("timer"),
};


/* XXX this driverfs stuff should probably go elsewhere later -john */
static struct sys_device device_timer = {
	.id	= 0,
	.cls	= &timer_sysclass,
};

static int time_init_device(void)
{
	int error = sysdev_class_register(&timer_sysclass);
	if (!error)
		error = sysdev_register(&device_timer);
	return error;
}

device_initcall(time_init_device);

#ifdef CONFIG_HPET_TIMER
extern void (*late_time_init)(void);
/* Duplicate of time_init() below, with hpet_enable part added */
void __init hpet_time_init(void)
{
	xtime.tv_sec = get_cmos_time();
	xtime.tv_nsec = (INITIAL_JIFFIES % HZ) * (NSEC_PER_SEC / HZ);
	set_normalized_timespec(&wall_to_monotonic,
		-xtime.tv_sec, -xtime.tv_nsec);

	if (hpet_enable() >= 0) {
		printk("Using HPET for base-timer\n");
	}

	cur_timer = select_timer();
	printk(KERN_INFO "Using %s for high-res timesource\n",cur_timer->name);

	time_init_hook();
}
#endif

void __init time_init(void)
{
#ifdef CONFIG_HPET_TIMER
	if (is_hpet_capable()) {
		/*
		 * HPET initialization needs to do memory-mapped io. So, let
		 * us do a late initialization after mem_init().
		 */
		late_time_init = hpet_time_init;
		return;
	}
#endif
	xtime.tv_sec = get_cmos_time();
	xtime.tv_nsec = (INITIAL_JIFFIES % HZ) * (NSEC_PER_SEC / HZ);
	set_normalized_timespec(&wall_to_monotonic,
		-xtime.tv_sec, -xtime.tv_nsec);

	cur_timer = select_timer();
	printk(KERN_INFO "Using %s for high-res timesource\n",cur_timer->name);

	time_init_hook();
}
