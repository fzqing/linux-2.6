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

#include <asm/addrspace.h>
#include <asm/smp.h>
#include <linux/sched.h>
#include <linux/types.h>

#include <asm/rmi/sim.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/pic.h>
#include <asm/rmi/msgring.h>

/* ipi statistics counters for debugging */
__u32 ipi_3_counter_tx[NR_CPUS][NR_CPUS];
__u32 ipi_3_counter_rx[NR_CPUS];

extern void save_epc(unsigned long *epc);
extern void smp_call_function_interrupt(void);
extern void phoenix_smp_time_init(void);

static int phoenix_ipi_stats[NR_CPUS];
static unsigned long phoenix_ipi_epc[NR_CPUS];

void core_send_ipi(int logical_cpu, unsigned int action)
{
	int cpu = cpu_logical_map(logical_cpu);
	__u32 ipi = 0;
	__u32 tid = cpu & 0x3;
	__u32 pid = (cpu >> 2) & 0x07;

	if (action & SMP_CALL_FUNCTION) {
		ipi = (tid << 16) | (pid << 20) | IRQ_IPI_SMP_FUNCTION;
#ifdef IPI_PRINTK_DEBUG
		printk("[%s]: cpu_%d sending ipi_3 to cpu_%d \t\t\t[->%u] \n", 
				__FUNCTION__, smp_processor_id(), cpu, 
				ipi_3_counter_tx[smp_processor_id()][cpu]+1);
#endif
		++ipi_3_counter_tx[smp_processor_id()][cpu]; 

	}
	else if (action & SMP_LOCAL_TIMER) {
		ipi = (tid << 16) | (pid << 20) | IRQ_IPI_SMP_RESCHEDULE;
	}
	else if (action & SMP_RESCHEDULE_YOURSELF) {
		ipi = (tid << 16) | (pid << 20) | IRQ_IPI_SMP_RESCHEDULE;
#ifdef IPI_PRINTK_DEBUG
		printk("[%s]: cpu_%d sending ipi_4 to cpu_%d\n", __FUNCTION__,
			smp_processor_id(), cpu);     
#endif
	}
	else
		BUG();
	
	pic_send_ipi(ipi);
}

extern __u64 phnx_irq_mask;

void phoenix_smp_finish(void)
{
	phoenix_msgring_cpu_init();
}

void phoenix_ipi_handler(int irq, struct pt_regs *regs)
{
	phoenix_ipi_stats[smp_processor_id()]++;
	save_epc(&phoenix_ipi_epc[smp_processor_id()]);

	if (irq == IRQ_IPI_SMP_FUNCTION) {
#ifdef IPI_PRINTK_DEBUG
		printk("[%s]: cpu_%d processing ipi_%d [->%u]\n", __FUNCTION__, 
	   		smp_processor_id(), irq, 
			ipi_3_counter_rx[smp_processor_id()]++);
#endif
		++ipi_3_counter_rx[smp_processor_id()];
		smp_call_function_interrupt();
	}
	else {
#ifdef IPI_PRINTK_DEBUG
		printk("[%s]: cpu_%d processing ipi_%d\n", __FUNCTION__, 
			smp_processor_id(), irq);
#endif

#ifndef CONFIG_CPU_TIMER
		/* Announce that we are for reschduling */
		set_need_resched();
#else
		local_timer_interrupt(0, NULL, regs);
#endif
	}
	phoenix_ipi_stats[smp_processor_id()]--;
}
