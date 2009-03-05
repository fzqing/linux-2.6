/* defines for inline arch setup functions */
#include <asm/voyager.h>

static inline void do_timer_interrupt_hook(struct pt_regs *regs)
{
#ifdef CONFIG_HIGH_RES_TIMERS
	long arch_cycles = get_arch_cycles(jiffies);

	/*
	 * We use unsigned here to correct a little problem when
	 * the TSC is reset during the SMP sync TSC stuff at
	 * boot time.  The unsigned on the compare will force
	 * the code into a loop updating the "stake"
	 * (last_update) until we get a positive result.  By
	 * using unsigned we don't incure any additional over
	 * head while still traping the problem of a negative
	 * return.
	 */
	if ((unsigned)arch_cycles < arch_cycles_per_jiffy) { 
		do_hr_timer_int();
		return;
	}
	discipline_PIT_timer();
	do {
		do_timer(regs);
		stake_cpuctr();
	} while ((unsigned)get_arch_cycles(jiffies) > arch_cycles_per_jiffy);
#else
	do_timer(regs);
#endif
#ifndef CONFIG_SMP
	update_process_times(user_mode(regs));
#endif

	voyager_timer_interrupt(regs);
}

static inline int do_timer_overflow(int count)
{
	/* can't read the ISR, just assume 1 tick
	   overflow */
	if(count > LATCH || count < 0) {
		printk(KERN_ERR "VOYAGER PROBLEM: count is %d, latch is %d\n", count, LATCH);
		count = LATCH;
	}
	count -= LATCH;

	return count;
}
