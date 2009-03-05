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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/smp.h>

#include <asm/mipsregs.h>
#include <asm/mmu_context.h>
#include <asm/atomic.h>

#include <asm/rmi/sim.h>
#include <asm/rmi/msgring.h>
#include <asm/rmi/mips-exts.h>

extern volatile cpumask_t cpu_callin_map;
extern void phoenix_smp_finish(void);

extern void smp_tune_scheduling (void);

int phys_proc_id[NR_CPUS]; /* cpuid+thrid of each logical CPU */

extern __u32 rmios_user_mac_thr_mask;
extern void asmlinkage smp_bootstrap(void);

/* Boot all other cpus in the system, initialize them, and
   bring them into the boot fn */
void prom_boot_secondary(int logical_cpu, struct task_struct *idle)
{
	unsigned long gp = (unsigned long)idle->thread_info;
	unsigned long sp = gp + THREAD_SIZE - 32;
	int cpu = cpu_logical_map(logical_cpu);
	int bucket = ((cpu >> 2)<<3)|(cpu & 0x03);

	printk("(PROM): waking up phys cpu# %d, bucket_%d, gp = %lx\n", 
			cpu, bucket, gp);
  
#ifdef CONFIG_PHOENIX_PSB
	smp_boot.boot_info[cpu].sp = sp;
	smp_boot.boot_info[cpu].gp = gp;
	smp_boot.boot_info[cpu].fn = (unsigned long)&smp_bootstrap;  
	/* barrier */
	__sync();
	smp_boot.boot_info[cpu].ready = 1;
	
#else
	{
	  unsigned long flags = 0, msgrng_flags=0;
	  __u64 status = 0;
	
	  smp_boot_info[cpu].sp    = sp;
	  smp_boot_info[cpu].gp    = gp;
	  smp_boot_info[cpu].fn    = (__u32)&smp_bootstrap;
	  /* set the ready field last! sleeping cpus poll on this field */
	  smp_boot_info[cpu].ready = 1;  
	
	  /* here cpu is the bit position in BOOT_CPU_MAP_ADDR 
	   * for now, assume that BOOT_CPU_MAP_ADDR is 8 x 4
	   */
	
	  msgrng_enable(flags);
	
	  do {
		status = msgrng_read_status();
		if (!(status & 0x1)) break;
	  }while (1);
	
	  msgrng_send((cpu >> 2)<<3);
	
	  msgrng_disable(flags);
	}
#endif

	printk("(PROM): sent a wakeup message to bucket %d\n", bucket);
}

#ifdef CONFIG_PHOENIX_PSB
extern void ptr_smp_boot(unsigned long, unsigned long, unsigned long);
static spinlock_t smp_boot_lock;
struct smp_boot_info smp_boot;
void prom_boot_cpus_secondary(void *args)
{
	int cpu = hard_smp_processor_id();
	unsigned long flags;

#ifndef CONFIG_PREEMPT	
	spin_lock_irqsave(&smp_boot_lock, flags);
#endif
	smp_boot.online_map |= (1<<cpu);
#ifndef CONFIG_PREEMPT
	spin_unlock_irqrestore(&smp_boot_lock, flags);
#endif

	for(;;) {
		if (smp_boot.boot_info[cpu].ready) break;
	}
	__sync();

#ifdef CONFIG_64BIT 
#ifdef CONFIG_PHOENIX_USER_MAC
	if(rmios_user_mac_thr_mask & (1<<cpu)){
		//enable xkphys access in status register.
		__asm__ volatile(
		"mfc0	$8, $12\n"
		"li $9,0x1040009f\n"
		"or $8, $8, $9\n"
		"li $9,0x0040001f\n"
		"xor $8,$8,$9\n"
		"mtc0	$8, $12\n"
		"nop\n"
		"nop\n"
		);
	}
#endif
#endif
	ptr_smp_boot(smp_boot.boot_info[cpu].fn, smp_boot.boot_info[cpu].sp, 
		       smp_boot.boot_info[cpu].gp);
}
#endif

extern void phoenix_smp_init(void);
void prom_init_secondary(void)
{
	phoenix_smp_init();
}

void prom_smp_finish(void)
{
	phoenix_smp_finish();
	local_irq_enable();
}


void prom_cpus_done(void)
{
#ifdef CONFIG_CPU_TIMER
	extern void sync_c0_count_master(void);
	sync_c0_count_master();
#endif
}

void __init prom_build_cpu_map(void)
{
	int num_cpus = 0;
	__u32 boot_cpu_online_map = 0;

	extern __u32 ipi_3_counter_tx[NR_CPUS][NR_CPUS];
	extern __u32 ipi_3_counter_rx[NR_CPUS];
	int i=0, j=0;

	cpus_clear(phys_cpu_present_map);
	cpu_set(0, phys_cpu_present_map);

	cpus_clear(cpu_present_map);
	cpu_set(0, cpu_present_map);
	__cpu_number_map[0] = 0;
	__cpu_logical_map[0] = 0;

	/* Initialize the ipi debug stat variables */
	for(i=0;i<NR_CPUS;i++) {
		for(j=0;j<NR_CPUS;j++)
			ipi_3_counter_tx[i][j] = 0;
	
		ipi_3_counter_rx[i] = 0;
	}

	/* skip the last thread in every core */
	if (xlr_hybrid_user_mac()) {
		int user_mac_thr = 0;

		for (i=0;i<32;i+=4) {
			for(j=i+1;j<i+4;j++) 
				if (smp_boot.online_map & (1 << j)) user_mac_thr = j;
			rmios_user_mac_thr_mask |= (1 << user_mac_thr);
		}
		boot_cpu_online_map = smp_boot.online_map & ~rmios_user_mac_thr_mask;          
		printk("[%s]: smp_boot.online_map=%x, boot_cpu_online_map=%x, rmios_user_mac_thr_mask=%x\n",
				__FUNCTION__, smp_boot.online_map, boot_cpu_online_map, rmios_user_mac_thr_mask);
	}
	else {

#ifdef CONFIG_PHOENIX_PSB
	boot_cpu_online_map = smp_boot.online_map;
#else
	boot_cpu_online_map = *(int *)BOOT_CPU_MAP_ADDR;
#endif
	}

	printk("(PROM) CPU present map: %x\n", boot_cpu_online_map);

	for(i=1;i<NR_CPUS;i++) {
		if (boot_cpu_online_map & (1<<i)) {
			cpu_set(i, phys_cpu_present_map);
			++num_cpus;
			__cpu_number_map[i] = num_cpus;
			__cpu_logical_map[num_cpus] = i;
			cpu_set(num_cpus, cpu_present_map);
		}
	}

	printk("Phys CPU present map: %lx, CPU Present map %lx\n", 
		(unsigned long)phys_cpu_present_map.bits[0], 
	   	(unsigned long)cpu_present_map.bits[0]);
	

	printk("Detected %i Slave CPU(s)\n", num_cpus);
}

void prom_prepare_cpus(unsigned int max_cpus)
{
	  prom_build_cpu_map();
}

