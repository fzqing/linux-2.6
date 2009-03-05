/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */

#include <linux/config.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include <asm/irq.h>
#include <asm/ptrace.h>
#include <asm/addrspace.h>
#include <asm/time.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/rmi/iomap.h>
#include <asm/rmi/pic.h>
#include <asm/rmi/interrupt.h>
#include <asm/rmi/mips-exts.h>

extern spinlock_t phnx_pic_lock;

#ifdef CONFIG_CPU_TIMER
extern void local_timer_interrupt(int irq,  void *dev_id, struct pt_regs *regs);
#else
extern void ll_local_timer_interrupt(int irq, struct pt_regs *regs);
#endif
static unsigned long phoenix_timer_stats[NR_CPUS] ____cacheline_aligned;
static unsigned long phoenix_timer_diff[NR_CPUS] ____cacheline_aligned;
static unsigned long phoenix_timer_count[NR_CPUS] ____cacheline_aligned;
static unsigned long phoenix_timer_epc[NR_CPUS] ____cacheline_aligned;
static unsigned long phoenix_timer_cpu[NR_CPUS] ____cacheline_aligned;
static unsigned long phoenix_timer_gettimeoffset[NR_CPUS] ____cacheline_aligned;
extern unsigned int mips_hpt_frequency;
unsigned int xlr_cycles_per_jiffy;
#ifdef CONFIG_PHOENIX_RAW_PERF_COUNTERS
extern void phnx_read_local_perf_ctrs(void);
extern void phnx_read_global_perf_ctrs(void);
#endif
void save_epc(unsigned long *epc)
{
	__asm__ __volatile__ (
			".set push\n"
			".set noreorder\n"
			"mfc0 %0, $14\n"
			".set pop\n"
			: "=r" (*epc) );
}


void xlr_timer_ack(void)
{
	write_c0_count(0);
	write_c0_compare(xlr_cycles_per_jiffy);
	write_c0_count(0);
}

void phoenix_timer_interrupt(struct pt_regs *regs, int irq)
{
	int cpu = smp_processor_id();
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);

	if (irq != PIC_TIMER_7_IRQ && irq != IRQ_TIMER) {
		printk("cpu_%d: bad timer irq = %x\n", cpu, irq);
		BUG();
	}

	/* PIC timer is used for cpu 0 */
	if (cpu == 0 && irq == IRQ_TIMER) {write_c0_compare(0); return;}

	phoenix_timer_stats[cpu]++;
	phoenix_timer_diff[cpu] = read_c0_count() - phoenix_timer_count[cpu];
	phoenix_timer_count[cpu] = read_c0_count();
	save_epc(&phoenix_timer_epc[cpu]);
	phoenix_timer_cpu[cpu] += irq;
	
	if (cpu == 0) {
	  /* ack the pic */
		phoenix_write_reg(mmio, PIC_INT_ACK, 
			  (1 << (PIC_TIMER_7_IRQ - PIC_IRQ_BASE) ));
	
#ifdef CONFIG_PHOENIX_RAW_PERF_COUNTERS
		/* Collect Bridge, DRAM and L2 cache counter */
		phnx_read_global_perf_ctrs();
#endif
		/* global timer interrupt */
		ll_timer_interrupt(irq, regs);
	}  else {
		xlr_timer_ack();
#ifdef CONFIG_CPU_TIMER
		ll_timer_interrupt(irq, regs);
#else
		ll_local_timer_interrupt(irq, regs);
#endif
	}
#ifdef CONFIG_PHOENIX_RAW_PERF_COUNTERS
	/* Collect COP0 perf counters which are per vCPU */
	phnx_read_local_perf_ctrs();
#endif
	
	
}

void phoenix_smp_time_init(void)
{
	xlr_cycles_per_jiffy = (mips_hpt_frequency + HZ / 2) / HZ;
	/* non-zero cpus use count/compare as the timer */
	xlr_timer_ack();
}

#define PIC_CLKS_PER_SEC 66000000ULL
// can't do floating in the kernel, so use 64 as an approximation 
#define PIC_CLKS_PER_USEC 64 //(PIC_CLKS_PER_SEC / 1000000)
#define PIC_CLKS_PER_TIMER_TICK (PIC_CLKS_PER_SEC / HZ)

#ifndef CONFIG_CPU_TIMER
extern unsigned long (*do_gettimeoffset)(void);
#endif
/* 
 * this routine returns the time duration since the last timer interrupt
 * in micro seconds
 * irrespective of whether gettimeofday is called on cpu 0 or on a non-zero cpu, 
 * it gets the usecs from the last timer interrupt on cpu 0 which uses the PIC
 * timer interrupt. the count/compare interrupt on non-zero cpu is only for process
 * accounting and such but not time keeping
 */ 
unsigned long phoenix_gettimeoffset(void)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	__u32 counter = 0;
	__u32 maxval = PIC_CLKS_PER_TIMER_TICK;
	
	/* PIC TIMERs are 64-bit counters but the timeout values are small
	 * enough to fit in 32-bit, so use only lower 32-bit regs
	 */
	counter = phoenix_read_reg(mmio, PIC_TIMER_7_COUNTER_0);

	/* store the # of pic clks since last interrupt in counter */
	counter = (maxval - counter) / PIC_CLKS_PER_USEC;

	phoenix_timer_gettimeoffset[smp_processor_id()] = counter;

	return counter;
}

void phoenix_timer_setup(void)
{
	__u64 maxval = PIC_CLKS_PER_TIMER_TICK;
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	unsigned long flags = 0;
	int cpu = (phoenix_cpu_id()<<2)|phoenix_thr_id();

	spin_lock_irqsave(&phnx_pic_lock, flags);
	phoenix_write_reg(mmio, PIC_TIMER_7_MAXVAL_0, (maxval & 0xffffffff));
	phoenix_write_reg(mmio, PIC_TIMER_7_MAXVAL_1, 
				(maxval >> 32) & 0xffffffff);
	phoenix_write_reg(mmio, PIC_IRT_0_TIMER_7, (1 << cpu));
	phoenix_write_reg(mmio, PIC_IRT_1_TIMER_7, 
			(1<<31)|(0<<30)|(1<<6)|(PIC_TIMER_7_IRQ));
	/* Enable the timer */
	pic_update_control(1<<(8+7));
	spin_unlock_irqrestore(&phnx_pic_lock, flags);

#ifndef CONFIG_CPU_TIMER
	do_gettimeoffset = phoenix_gettimeoffset;

	printk("%s: phoenix_timer_stats = %p, phoenix_timer_diff = %p,"
		"phoenix_timer_count = %p, phoenix_timer_epc = %p," 
		"phoenix_timer_cpu = %p, phoenix_timer_gettimeoffset = %p\n",
	 	__FUNCTION__, phoenix_timer_stats, phoenix_timer_diff, 
		phoenix_timer_count, phoenix_timer_epc,
	 	phoenix_timer_cpu, phoenix_timer_gettimeoffset);
#endif
}
