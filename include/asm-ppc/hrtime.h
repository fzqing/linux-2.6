/*
 * include/asm-ppc/hrtime.h
 *
 * PPC High-Res Timers header file
 *
 * Author: source@mvista.com
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASMPPC_HRTIME_H
#define _ASMPPC_HRTIME_H

#ifdef __KERNEL__
#include <linux/time.h>
#include <asm/types.h>
#include <asm/time.h>
#include <linux/sc_math.h>

#define CONFIG_HIGH_RES_RESOLUTION 1000 /* 1 uSec expressed as nSecs */

#define arch_cycles_per_jiffy ((int)tb_ticks_per_jiffy)
#define schedule_jiffies_int(x) (get_arch_cycles(x) >= arch_cycles_per_jiffy)

#define hrtimer_use    1

extern int hr_time_resolution;

static inline int get_arch_cycles(unsigned ref_jiffies)
{

	int ret_val, diff;

	/* calculate cycles since the current jiffy */
	ret_val = get_tbl() - tb_last_stamp;

	/* compensate for ref_jiffies in the past */
	if (unlikely(diff = jiffies - ref_jiffies))
		ret_val += diff * arch_cycles_per_jiffy;

        return ret_val;
}

extern int schedule_hr_timer_int(unsigned long ref_jiffies, int cycles);

/* 
 * We use various scaling.  The ex32 scales by 2**32, sc_n by the first parm.
 * When working with constants, choose a scale such that x/n->(32-scale)< 1/2.
 * So for 1/3 <1/2 so scale of 32, where as 3/1 must be shifted 3 times (3/8) to
 * be less than 1/2 so scale should be 29.
 *
 * For PPC, the lower limit for the decrementer clock was assumed to be 1.25
 * MHz. Since 1,000,000,000 / 1,250,000 = 800, the scale factor was chosen to
 * be 21 (800/2048 < 0.5).
 */
#define HR_TIME_SCALE_NSEC 21
#define HR_TIME_SCALE_CYCLES 32

extern int arch_to_nsec;
extern int nsec_to_arch;

static inline int scaled_mult(int x, int y, int shift) {

	int hi;
	unsigned lo;
		
	if ((unsigned)shift > 32)
		BUG();
	asm ("mulhw %0,%1,%2" : "=r" (hi) : "r" (x), "r" (y));
	if (shift == 32)
		return hi;
	asm ("mullw %0,%1,%2" : "=r" (lo) : "r" (x), "r" (y));

	return (hi << (32-shift)) | (lo >> shift);
}

static inline int arch_cycle_to_nsec(long update)
{
        return scaled_mult(update, arch_to_nsec, HR_TIME_SCALE_NSEC);
}
static inline int nsec_to_arch_cycle(unsigned long nsec)
{
        return scaled_mult(nsec, nsec_to_arch, HR_TIME_SCALE_CYCLES);
}

#endif /*__KERNEL__*/
#endif /* _ASMPPC_HRTIME_H */
