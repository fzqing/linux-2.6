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
#include <linux/smp.h>

#include <asm/page.h>
#include <asm/bug.h>
#include <asm/asm.h>
#include <asm/system.h>

/* These are the functions hooked by the memory management function pointers */
void clear_page(void *page)
{
#ifdef CONFIG_PHOENIX_VM_DEBUG
  printk("[%s]: page = %lx\n", __FUNCTION__, (unsigned long)page);
#endif
  __asm__ __volatile__(
		       ".set push                  \n"
		       ".set noreorder             \n"
		       ".set noat                  \n"
		       ".set mips4                 \n"

		       /* store the address of last cacheline in $1 */
		       STR(LONG_ADDIU)"\t$1, %0, %1  \n"  

		       /* pref with prep_for_store and zero the cacheline */
		       "1:   pref      30,  0(%0)  \n"

		       "     sd        $0,  0(%0)  \n"  
		       "     sd        $0,  8(%0)  \n"
		       "     sd        $0, 16(%0)  \n"
		       "     sd        $0, 24(%0)  \n"

		       /* loop till the last cacheline */
		       STR(LONG_ADDIU)"\t%0, %0, 32  \n"  
		       "     bne       %0, $1, 1b  \n"
		       "     nop                   \n"

		       ".set pop                   \n"
		       :
		       :"r" (page), "I" (PAGE_SIZE)
		       :"$1","memory");
}

extern int xlr_request_dma(uint64_t src, uint64_t dest, uint32_t len);
void copy_page(void *to, void *from)
{
#ifdef PHOENIX_DMA_PAGEOPS

	u64 from_phys = CPHYSADDR(from);
	u64 to_phys = CPHYSADDR(to);

	/* if any page is not in KSEG0, use old way */
	if ((long)KSEGX(to) != (long)CKSEG0
	    || (long)KSEGX(from) != (long)CKSEG0) 
		goto cpu_copy;

	if((xlr_request_dma(from_phys, to_phys, PAGE_SIZE)) == 0) {
		return;
	}
	/* try the old way */

cpu_copy:
#endif
  __asm__ __volatile__(
		       ".set push                  \n"
		       ".set noreorder             \n"
		       ".set noat                  \n"
		       ".set mips64                 \n"

		       /* store the address of last cacheline in $1 */
		       STR(LONG_ADDIU)"\t$1, %1, %2  \n"  

		       /* pref with prep_for_store the current cacheline */
		       /* the stores should merge into this pref cacheline */
		       "1:   pref      1,   0(%0)  \n"

		       /* pref the next cacheline "from" */
		       "     pref      0,    32(%1)  \n"

		       /* copy the cacheline */
#ifdef CONFIG_64BIT
 		       "2:   ld        $8,   0(%1)  \n"
		       "     ld        $9,   8(%1)  \n"
		       "     ld        $10,  16(%1)  \n"
		       "     ld        $11,  24(%1)  \n"
		       "     sd        $8,   0(%0)  \n"
		       "     sd        $9,   8(%0)  \n"
		       "     sd        $10,  16(%0)  \n"
		       "     sd        $11,  24(%0)  \n"
#else
 		       "2:   lw        $8,   0(%1)  \n"
		       "     lw        $9,   4(%1)  \n"
		       "     lw        $10,  8(%1)  \n"
		       "     lw        $11,  12(%1)  \n"
		       "     sw        $8,   0(%0)  \n"
		       "     sw        $9,   4(%0)  \n"
		       "     sw        $10,  8(%0)  \n"
		       "     sw        $11,  12(%0)  \n"
 		       "     lw        $8,   16(%1)  \n"
		       "     lw        $9,   20(%1)  \n"
		       "     lw        $10,  24(%1)  \n"
		       "     lw        $11,  28(%1)  \n"
		       "     sw        $8,   16(%0)  \n"
		       "     sw        $9,   20(%0)  \n"
		       "     sw        $10,  24(%0)  \n"
		       "     sw        $11,  28(%0)  \n"
#endif

		       /* loop till the last cacheline */		       
		       STR(LONG_ADDIU)"\t%1, %1, 32  \n"  
		       STR(LONG_ADDIU)"\t%0, %0, 32  \n"  
		       "     bne       %1, $1, 1b  \n"
		       "     nop                   \n"

		       ".set pop                   \n"

		       :
		       :"r" (to), "r"(from), "I" (PAGE_SIZE)
		       :"$1","$8", "$9", "$10", "$11", "memory");
}
