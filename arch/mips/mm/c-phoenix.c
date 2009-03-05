/*
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 * Copyright (C) 1997, 2001 Ralf Baechle (ralf@gnu.org)
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
#include <linux/init.h>
#include <asm/mmu_context.h>
#include <asm/bootinfo.h>
#include <asm/hazards.h>
#include <asm/cacheops.h>
#include <asm/cpu.h>
#include <asm/rmi/mips-exts.h>
#include <asm/uaccess.h>
#include <asm/asm.h>
#include <linux/smp.h>
#include <linux/kallsyms.h>

#include <asm/rmi/debug.h>

static unsigned int icache_linesz;
static unsigned int icache_lines;

#define cacheop(op, base) __asm__ __volatile__ (".set push\n.set mips4\ncache %0, 0(%1)\n.set pop\n" : : "i"(op), "r"(base))

#define cacheop_extable(op, base) do {                    \
  __asm__ __volatile__(                                    \
		       "    .set push                \n"   \
		       "    .set noreorder           \n"   \
		       "    .set mips4               \n"   \
		       "1:  cache %0, 0(%1)           \n"  \
		       "2:  .set pop                 \n"   \
		       "    .section __ex_table,\"a\"\n"   \
			STR(PTR)"\t1b, 2b\n\t"			\
		       "     .previous               \n"   \
		       : : "i" (op), "r" (base));          \
  } while (0) 


static __inline__ void cacheop_sync_istream(void)
{
	cacheop_hazard();
	sync_istream();
}


/*****************************************************************************
 * 
 * These routines support Generic Kernel cache flush requirements
 *
 ***************************************************************************/
void phoenix_flush_dcache_page(struct page *page)
{
	ClearPageDcacheDirty(page);    
}

static void phoenix_local_flush_icache_range(long start, 
						long end)
{
	long addr;
  
	for(addr = (start & ~(icache_linesz - 1)); addr < end; 
			addr += icache_linesz) {
		cacheop(Hit_Invalidate_I, addr);
	}

	cacheop_sync_istream();
}

struct flush_icache_range_args {
  long start;
  long end;
};

static void phoenix_flush_icache_range_ipi(void *info)
{
	struct flush_icache_range_args *args = info;

	phoenix_local_flush_icache_range(args->start, args->end);
}

void phoenix_flush_icache_range(unsigned long start, unsigned long end)
{
	struct flush_icache_range_args args;

#ifdef CONFIG_PHOENIX_VM_DEBUG
	rmi_dbg_msg("return address: ");
	print_symbol("ra[0]=%s\n", (unsigned long) __builtin_return_address(0));
#endif
  
	args.start = start;
	args.end = end;
	on_each_cpu(phoenix_flush_icache_range_ipi, &args, 1, 1);
}

static void phoenix_flush_cache_sigtramp_ipi(void *info)
{
	unsigned long addr = (unsigned long)info;

	addr = addr & ~((unsigned long)icache_linesz - 1);
	cacheop_extable(Hit_Invalidate_I, addr );
	cacheop_sync_istream();
}

static void phoenix_flush_cache_sigtramp(unsigned long addr)
{
	on_each_cpu(phoenix_flush_cache_sigtramp_ipi, (void *) addr, 1, 1);
}

/**************************************************************************
 * 
 * These routines support MIPS specific cache flush requirements.
 * These are called only during bootup or special system calls 
 *
 **************************************************************************/

static void phoenix_local_flush_icache(void)
{
	int i=0;
	unsigned long base = CKSEG0;

	/* Index Invalidate all the lines and the ways */
	for(i=0;i<icache_lines;i++) {
		cacheop(Index_Invalidate_I, base);
		base += icache_linesz;
	}

	cacheop_sync_istream(); 

}

static void phoenix_local_flush_dcache(void)
{
	int i=0;
	unsigned long base = CKSEG0;
	unsigned int lines;

	lines = current_cpu_data.dcache.ways * current_cpu_data.dcache.sets;
	
	/* Index Invalidate all the lines and the ways */  
	for(i=0;i<lines;i++) {
		cacheop(Index_Writeback_Inv_D, base);
		base += current_cpu_data.dcache.linesz;
	}

	cacheop_hazard(); 

}

static void phoenix_flush_l1_caches_ipi(void *info)
{
	phoenix_local_flush_dcache();
	phoenix_local_flush_icache();
}

static void phoenix_flush_l1_caches(void)
{
	on_each_cpu(phoenix_flush_l1_caches_ipi, (void *)NULL, 1, 1);
}

static void phoenix_noflush(void) { /* do nothing */ }

static __init void probe_l1_cache(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	unsigned int config1 = read_c0_config1();
	int lsize = 0;
	int icache_size=0, dcache_size=0;

	if ((lsize = ((config1 >> 19) & 7)))
		c->icache.linesz = 2 << lsize;
	else
		c->icache.linesz = lsize;
	c->icache.sets = 64 << ((config1 >> 22) & 7);
	c->icache.ways = 1 + ((config1 >> 16) & 7);

	icache_size = c->icache.sets *
			c->icache.ways * c->icache.linesz;
	c->icache.waybit = ffs(icache_size/c->icache.ways) - 1;

	c->dcache.flags = 0;

	if ((lsize = ((config1 >> 10) & 7)))
		c->dcache.linesz = 2 << lsize;
	else
		c->dcache.linesz= lsize;
	c->dcache.sets = 64 << ((config1 >> 13) & 7);
	c->dcache.ways = 1 + ((config1 >> 7) & 7);

	dcache_size = c->dcache.sets *
			c->dcache.ways * c->dcache.linesz;
	c->dcache.waybit = ffs(dcache_size/c->dcache.ways) - 1;

	if (smp_processor_id()==0) {
		printk("Primary instruction cache %dkB, %d-way," 
			"linesize %d bytes.\n", icache_size >> 10, 
			c->icache.ways, c->icache.linesz);
		printk("Primary data cache %dkB %d-way, linesize %d bytes.\n",
			dcache_size >> 10, c->dcache.ways, c->dcache.linesz);
	}

}

static __inline__ void install_cerr_handler(void)
{
	extern char except_vec2_generic;

	memcpy((void *)(CAC_BASE + 0x100), &except_vec2_generic, 0x80);
}

static void update_kseg0_coherency(void)
{
	int attr = read_c0_config() & CONF_CM_CMASK;

	if (attr != CONF_CM_DEFAULT) {

		phoenix_local_flush_dcache();
		phoenix_local_flush_icache();

		change_c0_config(CONF_CM_CMASK, CONF_CM_DEFAULT);

		sync_istream();
	}

}

void xlr_cache_init(void)
{
	/* update cpu_data */
	probe_l1_cache();

	if (smp_processor_id()) {  

		/* flush the exception vector region to make sure 
		* not to execute bootloader's exception code 
		*/
		phoenix_local_flush_icache_range(CAC_BASE, CAC_BASE + 0x400);

		update_kseg0_coherency();

		return;
	}

	/* These values are assumed to be the same for all cores */
	icache_lines = 
		current_cpu_data.icache.ways * current_cpu_data.icache.sets;
	icache_linesz = current_cpu_data.icache.linesz;

	/* When does this function get called? Looks like MIPS has some syscalls
	 * to flush the caches. 
	 */
	__flush_cache_all = phoenix_flush_l1_caches;

	/* flush_cache_all: makes all kernel data coherent.
	 * This gets called just before changing or removing
	 * a mapping in the page-table-mapped kernel segment (kmap). 
	 * Physical Cache -> do nothing
	 */
	flush_cache_all = phoenix_noflush;

	/* flush_icache_range: makes the range of addresses coherent w.r.t 
	 * I-cache and D-cache 
	 * This gets called after the instructions are written to memory
	 * All addresses are valid kernel or mapped user-space virtual addresses
	 */
	flush_icache_range = phoenix_flush_icache_range;

	/* flush_cache_{mm, range, page}: make these memory locations, 
	 * that may have been written by a user process, coherent
	 * These get called when virtual->physical translation of a user 
	 * address space is about to be changed. 
	 * These are closely related to TLB coherency 
	 * (flush_tlb_{mm, range, page})
	 */
	flush_cache_mm = (void (*)(struct mm_struct *))phoenix_noflush;
	flush_cache_range = (void *) phoenix_noflush;
	flush_cache_page = (void *) phoenix_noflush;

	/* flush_icache_page: flush_dcache_page + update_mmu_cache takes 
	 * care of this
	 */
	flush_icache_page = (void *) phoenix_noflush;
	flush_data_cache_page = (void *) phoenix_noflush;

	/* flush_cache_sigtramp: flush the single I-cache line with the 
	 * proper fixup code
	 */
	flush_cache_sigtramp = phoenix_flush_cache_sigtramp;

	/* flush_icache_all: This should get called only for Virtuall Tagged 
	 * I-Caches
	 */
	flush_icache_all = (void *)phoenix_noflush;

	install_cerr_handler();

	update_kseg0_coherency();
}

#define cacheop_paddr(op, base) __asm__ __volatile__ ( \
                         ".set push\n"           \
                         ".set noreorder\n"      \
                         ".set mips64\n"          \
                         "dli $8, 0x9800000000000000\n"              \
                         "daddu $8, $8, %1\n"       \
                         "cache %0, 0($8)\n"     \
                         ".set pop\n"            \
                         : : "i"(op), "r"(base) : "$8")

#define enable_KX(flags)   \
 __asm__ __volatile__ (          \
	".set push\n"              \
	".set noat\n"               \
	".set noreorder\n"     \
	"mfc0 %0, $12\n\t"             \
	"ori $1, %0, 0x81\n\t"   \
	"xori $1, 1\n\t"      \
	"mtc0 $1, $12\n"       \
        ".set pop\n"          \
        : "=r"(flags) )
	
#define disable_KX(flags)   \
 __asm__ __volatile__ (          \
	".set push\n"              \
	"mtc0 %0, $12\n"       \
        ".set pop\n"          \
        : : "r"(flags) )
	

static void phoenix_local_flush_icache_range_paddr(unsigned long start, 
					unsigned long end)
{
	unsigned long addr;
	unsigned long flags;

	enable_KX(flags);
	for(addr = (start & ~(icache_linesz - 1)); addr < end; 
			addr += icache_linesz) {
		unsigned long long temp = addr & 0xffffffff;
		cacheop_paddr(Hit_Invalidate_I, temp);
	}
	disable_KX(flags);

	cacheop_sync_istream();
}

static void phoenix_flush_icache_range_paddr_ipi(void *info)
{
	struct flush_icache_range_args *args = info;

	phoenix_local_flush_icache_range_paddr(args->start, args->end);
}

void phoenix_flush_icache_range_paddr(unsigned long start)
{
	struct flush_icache_range_args args;

#ifdef CONFIG_PHOENIX_VM_DEBUG
	rmi_dbg_msg("return address: ");
	print_symbol("ra[0]=%s\n", (unsigned long) __builtin_return_address(0));
#endif
	
	args.start = start;
	args.end = start + PAGE_SIZE;
	/* TODO: don't even send ipi to non-zero thread ids 
	 * This may require some changes to smp_call_function interface, 
	 * for now just avoid redundant cache ops
	 */
	on_each_cpu(phoenix_flush_icache_range_paddr_ipi, &args, 1, 1);
}
