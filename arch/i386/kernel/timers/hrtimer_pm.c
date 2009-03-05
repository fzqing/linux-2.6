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

#define OK_TO_DO_IO_LOOP  // How to do the delay stuff.


extern unsigned long do_highres_gettimeoffset_pm(void)
{
        /*
         * We are under the xtime_lock here.
         */
	return arch_cycle_to_usec(get_arch_cycles(jiffies));
}

static void high_res_mark_offset_pm(void)
{
	reset_fillin_timer();
}
unsigned long long monotonic_clock_hr_pm(void)
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

#ifdef OK_TO_DO_IO_LOOP
/*
 * This routine is I/O intensive.  If this is a problem we will have to
 * use a compute loop as in the PIT code.  It is NOT affected by the
 * cpu clock, however.
 */
static void delay_pm(unsigned long loops)
{
	unsigned long bclock = inl(acpi_pm_tmr_address);

	/*
	 * XXX it doesn't depend on a number of processor cycles so
	 * the value may be very different from the usual one, is that
	 * a problem? -eric
	 */
	do {
		rep_nop();
	} while (((inl(acpi_pm_tmr_address) - bclock) & SIZE_MASK) < loops);
}
#else
/*
 * Avoids the I/O intense stuff but is affected by cpu clock shifting.
 */
static void delay_pm(unsigned long loops)
{
	int d0;
	__asm__ __volatile__(
		"\tjmp 1f\n"
		".align 16\n"
		"1:\tjmp 2f\n"
		".align 16\n"
		"2:\tdecl %0\n\tjns 2b"
		:"=&a" (d0)
		:"0" (loops));
}

#endif

#ifdef CONFIG_CPU_FREQ
static unsigned int  ref_freq = 0;
static unsigned int  variable_tsc = 1;

#ifdef OK_TO_DO_IO_LOOP
static unsigned long loops_per_jiffy_ref = 0;
#endif

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
#ifdef OK_TO_DO_IO_LOOP
		loops_per_jiffy_ref = cpu_data[freq->cpu].loops_per_jiffy;
#endif
	}

	if ((val == CPUFREQ_PRECHANGE  && freq->old < freq->new) ||
	    (val == CPUFREQ_POSTCHANGE && freq->old > freq->new)) {
#ifdef OK_TO_DO_IO_LOOP
		if (variable_tsc)
			cpu_data[freq->cpu].loops_per_jiffy = 
				cpufreq_scale(loops_per_jiffy_ref, 
					      ref_freq, freq->new);
#endif
	}

	return 0;
}

static struct notifier_block time_cpufreq_notifier_block = {
.notifier_call	= time_cpufreq_notifier
};
#endif


static int high_res_init_pm(char * override)
{
 	if (override[0] && strncmp(override,"hr_pm",5))
		return -ENODEV;

	/* report CPU clock rate in Hz.
	 * The formula is:
	 * (10^6 * 2^32) / (2^32 * 1 / (clocks/us)) =
	 * clock/second. Our precision is about 100 ppm.
	 */
        if (cpu_has_tsc) {
		unsigned long tsc_quotient = calibrate_tsc();
		if(tsc_quotient){
			cpu_khz = div_sc32( 1000, tsc_quotient);
			{	
				printk("Detected %lu.%03lu MHz processor.\n", 
				       cpu_khz / 1000, cpu_khz % 1000);
			}
		}
	}
        acpi_pm_tmr_address = hrt_get_acpi_pm_ptr(); 
        if (!acpi_pm_tmr_address){                    
                printk(message,default_pm_add);
                if ( (acpi_pm_tmr_address = default_pm_add)){
                        last_update +=  quick_get_cpuctr();
                        hrt_udelay(4);
			if (!quick_get_cpuctr()){
                                printk("High-res-timers: No ACPI pm "
				       "timer found at %d.\n",
                                       acpi_pm_tmr_address);
                                acpi_pm_tmr_address = 0;
                        } 
                } 
        } else {
                if (default_pm_add != acpi_pm_tmr_address) {
                        printk("High-res-timers: Ignoring supplied "
			       "default ACPI pm timer address.\n"); 
                }
                last_update +=  quick_get_cpuctr();
        }
	start_PIT();
        if (!acpi_pm_tmr_address){
                printk(fail_message);
		return -EINVAL;
        } else {
                printk("High-res-timers: Found ACPI pm timer at %d\n",
                       acpi_pm_tmr_address);
        }
#ifdef CONFIG_CPU_FREQ
	/* P4 and above CPU TSC freq doesn't change when CPU frequency changes*/
	if ((boot_cpu_data.x86 >= 15) && 
	    (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL))
		variable_tsc = 0;

        cpufreq_register_notifier(&time_cpufreq_notifier_block,	
				  CPUFREQ_TRANSITION_NOTIFIER);
#endif
	hrtimer_use = HRT_PM;
	return 0;
}

/************************************************************/


/* hr_pm timer_opts struct */
struct timer_opts hrtimer_pm = {
	.name =		"hrt_pm",
	.mark_offset =	high_res_mark_offset_pm, 
	.get_offset =	do_highres_gettimeoffset_pm,
	.monotonic_clock = monotonic_clock_hr_pm,
	.delay = delay_pm,
};

struct init_timer_opts __initdata hrtimer_pm_init = {
	.init = high_res_init_pm,
	.opts = &hrtimer_pm,
};
