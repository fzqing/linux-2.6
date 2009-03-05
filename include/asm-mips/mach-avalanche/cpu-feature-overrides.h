/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2005 Sekhar Nori
 */
#ifndef __ASM_MACH_AVALANCHE_CPU_FEATURE_OVERRIDES_H
#define __ASM_MACH_AVALANCHE_CPU_FEATURE_OVERRIDES_H

/* Setting to cache line size of 4kc. Still a grey area for me. This just works
 */

#ifndef PLAT_TRAMPOLINE_STUFF_LINE
#define PLAT_TRAMPOLINE_STUFF_LINE	16UL
#endif

/* These lines are just copied from MIPS32 part of mach-mips. Should be good. */

#define cpu_has_tlb		    1
#define cpu_has_4kex		1
#define cpu_has_4ktlb		1
#define cpu_has_fpu		    0 
#define cpu_has_32fpr	    0
#define cpu_has_counter		1
#define cpu_has_watch	    1 /* Guess this is not used by linux */ 
#define cpu_has_divec		1
#define cpu_has_vce		    0
#define cpu_has_cache_cdex_p    0
#define cpu_has_cache_cdex_s	0
#define cpu_has_prefetch	1
#define cpu_has_mcheck		1
#define cpu_has_ejtag	1
#define cpu_has_llsc		1
#define cpu_has_vtag_icache	0
/* #define cpu_has_dc_aliases	? */
/* #define cpu_has_ic_fills_f_dc ? */
#define cpu_has_nofpuex		0
#define cpu_has_64bits	    0
#define cpu_has_64bit_zero_reg 0
#define cpu_has_subset_pcaches 0


#endif /* __ASM_MACH_AVALANCHE_CPU_FEATURE_OVERRIDES_H */
