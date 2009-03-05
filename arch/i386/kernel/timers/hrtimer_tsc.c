/*
 * This code largely moved from arch/i386/kernel/time.c.
 * See comments there for proper credits.
 */

#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>
#include <linux/hrtime.h>

#include <asm/timer.h>
#include <asm/io.h>

#include "mach_timer.h"

extern int x86_udelay_tsc;
extern unsigned long tsc_cycles_per_50_ms;

#ifndef CONFIG_SMP
static int use_tsc;
#endif

/* Cached *multiplier* to convert TSC counts to microseconds.
 * (see the equation below).
 * Equal to 2^32 * (1 / (clocks per usec) ).
 * Initialized in time_init.
 */
static unsigned long fast_gettimeoffset_quotient;

static unsigned long do_highres_gettimeoffset(void)
{
        /*
         * We are under the xtime_lock here.
         */
	return arch_cycle_to_usec(get_arch_cycles(jiffies));
}

static void high_res_mark_offset_tsc(void)
{
	reset_fillin_timer();
}
static unsigned long long monotonic_clock_hr_tsc(void)
{
	unsigned long long timestamp;
	unsigned long seq;	
	do {
		seq = read_seqbegin(&xtime_lock);
		timestamp = jiffies_64 * (NSEC_PER_SEC / HZ)
				+ arch_cycle_to_nsec(get_arch_cycles(jiffies));
	} while (read_seqretry(&xtime_lock, seq));
	return timestamp;
}
static void delay_tsc(unsigned long loops)
{
	unsigned long bclock, now;
	
	rdtscl(bclock);
	do
	{
		rep_nop();
		rdtscl(now);
	} while ((now-bclock) < loops);
}

#ifdef CONFIG_CPU_FREQ
/* If the CPU frequency is scaled, TSC-based delays will need a different
 * loops_per_jiffy value to function properly. An exception to this
 * are modern Intel Pentium 4 processors, where the TSC runs at a constant
 * speed independent of frequency scaling. 
 */
static unsigned long ref_arch_to_usec;
static unsigned long ref_arch_to_latch;
static unsigned long ref_arch_to_nsec;
static unsigned long ref_usec_to_arch;
static unsigned long ref_nsec_to_arch;
static          long ref_arch_cycles_per_jiffy;
static unsigned int  ref_freq = 0;
static unsigned long loops_per_jiffy_ref = 0;
static unsigned int  variable_tsc = 1;

#ifndef CONFIG_SMP
static unsigned long cpu_khz_ref = 0;
#endif

static int
time_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		       void *data)
{
	struct cpufreq_freqs *freq = data;

	if (!ref_freq) {
		ref_freq = freq->old;
		loops_per_jiffy_ref = cpu_data[freq->cpu].loops_per_jiffy;
		ref_arch_to_usec = arch_to_usec;
		ref_arch_to_latch = arch_to_latch;
		ref_arch_to_nsec = arch_to_nsec;
		ref_nsec_to_arch = nsec_to_arch;
		ref_usec_to_arch = usec_to_arch;
		ref_arch_cycles_per_jiffy = arch_cycles_per_jiffy;
#if 0 /* ndef CONFIG_SMP  cpu_khz is done in timer_tsc.c */
		cpu_khz_ref = cpu_khz;
#endif
	}

	if ((val == CPUFREQ_PRECHANGE  && freq->old < freq->new) ||
	    (val == CPUFREQ_POSTCHANGE && freq->old > freq->new)) {

		if (variable_tsc) {
			cpu_data[freq->cpu].loops_per_jiffy = 
				cpufreq_scale(loops_per_jiffy_ref, 
					      ref_freq, freq->new);

		        arch_to_usec = 
				/* fast_gettimeoffset_quotient is done 
				 * timer_tsc.c
				fast_gettimeoffset_quotient = 
				*/
				cpufreq_scale(ref_arch_to_usec, 
 					      freq->new, ref_freq);
			arch_to_latch = 
				cpufreq_scale(ref_arch_to_latch, 
					      freq->new, ref_freq);
			arch_to_nsec =
				cpufreq_scale(ref_arch_to_nsec, 
					      freq->new, ref_freq);
			nsec_to_arch =
				cpufreq_scale(ref_nsec_to_arch, 
					      ref_freq, freq->new);
			usec_to_arch =
				cpufreq_scale(ref_usec_to_arch, 
					      ref_freq, freq->new);
			arch_cycles_per_jiffy =
				cpufreq_scale(ref_arch_cycles_per_jiffy, 
					      ref_freq, freq->new);
		}
#if 0 /* ndef CONFIG_SMP_use_timer_tsc */
		if (use_tsc) 
			cpu_khz = cpufreq_scale(cpu_khz_ref, ref_freq, freq->new);
#endif
	}

	return 0;
}

static struct notifier_block time_cpufreq_notifier_block = {
	notifier_call:	time_cpufreq_notifier
};
#endif


static int high_res_init_tsc(char * override)
{
 	if (override[0] && strncmp(override,"hrtsc",5))
		return -ENODEV;

	if (tsc_disable) {
		printk(KERN_WARNING "notsc: Kernel compiled with "
		       "CONFIG_HIGH_RES_TIMERS"
		       " TSC, cannot disable TSC.\n");
		tsc_disable = 0;
	}
	/*
	 * If we have APM enabled or the CPU clock speed is variable
	 * (CPU stops clock on HLT or slows clock to save power)
	 * then the TSC timestamps may diverge by up to 1 jiffy from
	 * 'real time' but nothing will break.
	 * The most frequent case is that the CPU is "woken" from a halt
	 * state by the timer interrupt itself, so we get 0 error. In the
	 * rare cases where a driver would "wake" the CPU and request a
	 * timestamp, the maximum error is < 1 jiffy. But timestamps are
	 * still perfectly ordered.
	 * Note that the TSC counter will be reset if APM suspends
	 * to disk; this won't break the kernel, though, 'cuz we're
	 * smart.  See arch/i386/kernel/apm.c.
	 */
 	/*
 	 *	Firstly we have to do a CPU check for chips with
 	 * 	a potentially buggy TSC. At this point we haven't run
 	 *	the ident/bugs checks so we must run this hook as it
 	 *	may turn off the TSC flag.
 	 *
 	 *	NOTE: this doesnt yet handle SMP 486 machines where only
 	 *	some CPU's have a TSC. Thats never worked and nobody has
 	 *	moaned if you have the only one in the world - you fix it!
 	 */
 
 	dodgy_tsc();
 	
	if (cpu_has_tsc) {
		unsigned long tsc_quotient = calibrate_tsc();
		if (tsc_quotient) {
			fast_gettimeoffset_quotient = tsc_quotient;
			/*
			 *	We could be more selective here I suspect
			 *	and just enable this for the next intel chips ?
			 */
                        /*
                         * Kick off the high res timers
                         */
			/*
			 * The init_hrtimers macro is in the choosen
			 * support package depending on the clock
			 *  source, TSC, or ACPI pm timer.
			 */
			arch_to_usec = fast_gettimeoffset_quotient;
 
			arch_to_latch = div_ll_X_l(
				mpy_l_X_l_ll(fast_gettimeoffset_quotient, 
					     CLOCK_TICK_RATE),
				(USEC_PER_SEC));

			arch_to_nsec = div_sc_n(HR_TIME_SCALE_NSEC,
						CALIBRATE_TIME * NSEC_PER_USEC,
						tsc_cycles_per_50_ms);

			nsec_to_arch = div_sc_n(HR_TIME_SCALE_NSEC,
						tsc_cycles_per_50_ms,
						CALIBRATE_TIME * NSEC_PER_USEC);

			usec_to_arch = div_sc_n(HR_TIME_SCALE_USEC,
						tsc_cycles_per_50_ms,
						CALIBRATE_TIME );

			arch_cycles_per_jiffy = nsec_to_arch_cycle(tick_nsec);  

			start_PIT();

			/* report CPU clock rate in Hz.
			 * The formula is:
			 * (10^6 * 2^32) / (2^32 * 1 / (clocks/us)) =
			 * clock/second. Our precision is about 100 ppm.
			 */
			cpu_khz = div_sc32( 1000, tsc_quotient);
			{	
				printk("Detected %lu.%03lu MHz processor.\n", 
				       cpu_khz / 1000, cpu_khz % 1000);
			}
#ifdef CONFIG_CPU_FREQ
		/* 
		 * P4 and above CPU TSC freq doesn't change when 
		 * CPU frequency changes
		 */
		if ((boot_cpu_data.x86 >= 15) && 
		    (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL))
			variable_tsc = 0;
		
		cpufreq_register_notifier(&time_cpufreq_notifier_block, 
					  CPUFREQ_TRANSITION_NOTIFIER);
#endif
		hrtimer_use = HRT_TSC;
		return 0;
		}
	}
	return -ENODEV;
}

/************************************************************/

/* tsc timer_opts struct */
struct timer_opts hrtimer_tsc = {
	.name =		"hrt_tsc",
	.mark_offset =	high_res_mark_offset_tsc, 
	.get_offset =	do_highres_gettimeoffset,
	.monotonic_clock = monotonic_clock_hr_tsc,
	.delay = delay_tsc,
};

struct init_timer_opts __initdata hrtimer_tsc_init = {
	.init = high_res_init_tsc,
	.opts = &hrtimer_tsc,
};
