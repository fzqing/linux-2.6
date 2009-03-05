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
#ifndef _ASM_RMI_MIPS_EXTS_H
#define _ASM_RMI_MIPS_EXTS_H

#define PHOENIX_OSS_SEL_TLB_STATS 0
#define PHOENIX_OSS_SEL_BIGTLB_STATS 1
#define PHOENIX_OSS_SEL_PAGEMASK 2
#define PHOENIX_OSS_SEL_VADDR 3
#define PHOENIX_OSS_SEL_PFN0 4
#define PHOENIX_OSS_SEL_PFN1 5
#define PHOENIX_OSS_SEL_K0 6
#define PHOENIX_OSS_SEL_K1 7


#ifndef __ASSEMBLY__

#include <linux/types.h>
#include <asm/rmi/interrupt.h>

#define DMFC0_AT_EIRR 0x40214806
#define DMFC0_AT_EIMR 0x40214807
#define DMTC0_AT_EIRR 0x40a14806
#define DMTC0_AT_EIMR 0x40a14807

/* functions to write to and read from the extended 
 * cp0 registers.
 * EIRR : Extended Interrupt Request Register 
 *        cp0 register 9 sel 6
 *        bits 0...7 are same as cause register 8...15
 * EIMR : Extended Interrupt Mask Register
 *        cp0 register 9 sel 7
 *        bits 0...7 are same as status register 8...15
 */

static inline __u64 read_64bit_cp0_eirr(void)
{
  __u32 high, low;

  __asm__ __volatile__ (
			".set push\n"
			".set noreorder\n"
			".set noat\n"
			".set mips4\n"
			
			".word 0x40214806  \n\t"			
			"nop               \n\t"
			"dsra32 %0, $1, 0  \n\t"
			"sll    %1, $1, 0  \n\t"

			".set pop\n"
			
			: "=r" (high), "=r" (low)
			);

  return ( ((__u64)high) << 32) | low;
}

static inline __u64 read_64bit_cp0_eimr(void)
{
  __u32 high, low;

  __asm__ __volatile__ (
			".set push\n"
			".set noreorder\n"
			".set noat\n"
			".set mips4\n"
			
			".word 0x40214807  \n\t"			
			"nop               \n\t"
			"dsra32 %0, $1, 0  \n\t"
			"sll    %1, $1, 0  \n\t"

			".set pop\n"
			
			: "=r" (high), "=r" (low)
			);

  return ( ((__u64)high) << 32) | low;
}

static inline void write_64bit_cp0_eirr(__u64 value) 
{  
  __u32 low, high;

  high = value >> 32;
  low  = value & 0xffffffff;

	__asm__ __volatile__ (
	".set push\n"
	".set noreorder\n"
	".set noat\n"
	".set mips4\n\t"

	"dsll32 $2, %1, 0  \n\t"
	"dsll32 $1, %0, 0  \n\t"
	"dsrl32 $2, $2, 0  \n\t"
	"or     $1, $1, $2 \n\t"
	".word  0x40a14806 \n\t"
	"nop               \n\t"

	".set pop\n"

	:
	: "r" (high), "r" (low)
	: "$1", "$2");
}  

static inline void write_64bit_cp0_eimr(__u64 value)
{
  __u32 low, high;

  high = value >> 32;
  low  = value & 0xffffffff;

	__asm__ __volatile__ (
	".set push\n"
	".set noreorder\n"
	".set noat\n"
	".set mips4\n\t"

	"dsll32 $2, %1, 0  \n\t"
	"dsll32 $1, %0, 0  \n\t"
	"dsrl32 $2, $2, 0  \n\t"
	"or     $1, $1, $2 \n\t"
	".word  0x40a14807 \n\t"
	"nop               \n\t"

	".set pop\n"

	:
	: "r" (high), "r" (low)
	: "$1", "$2");
}

#define phoenix_id()                                            \
({int __id;                                                     \
 __asm__ __volatile__ (                                         \
		       ".set push\n"                            \
		       ".set noreorder\n"                       \
                       ".word 0x40088007\n"                     \
		       "srl  $8, $8, 10\n"                      \
		       "andi %0, $8, 0x3f\n"                    \
		       ".set pop\n"                             \
		       : "=r" (__id) : : "$8");                 \
 __id;})

#define phoenix_cpu_id()                                        \
({int __id;                                                     \
 __asm__ __volatile__ (                                         \
		       ".set push\n"                            \
		       ".set noreorder\n"                       \
                       ".word 0x40088007\n"                     \
		       "srl  $8, $8, 4\n"                       \
		       "andi %0, $8, 0x3f\n"                    \
		       ".set pop\n"                             \
		       : "=r" (__id) : : "$8");                 \
 __id;})

#define phoenix_thr_id()                                        \
({int __id;                                                     \
 __asm__ __volatile__ (                                         \
		       ".set push\n"                            \
		       ".set noreorder\n"                       \
                       ".word 0x40088007\n"                     \
		       "andi %0, $8, 0x0f\n"                    \
		       ".set pop\n"                             \
		       : "=r" (__id) : : "$8");                 \
 __id;})

static __inline__ int hard_smp_processor_id(void)
{
  return (phoenix_cpu_id() << 2) + phoenix_thr_id();
}

#define phoenix_cpu_to_thrid(cpu) (phys_proc_id[(cpu)] >> 2)
#define phoenix_cpu_to_cpuid(cpu) (phys_proc_id[(cpu)] & 0x3)

#define CPU_BLOCKID_IFU      0
#define CPU_BLOCKID_ICU      1
#define CPU_BLOCKID_IEU      2
#define CPU_BLOCKID_LSU      3
#define CPU_BLOCKID_MMU      4
#define CPU_BLOCKID_PRF      5

#define LSU_CERRLOG_REGID    9

static __inline__ unsigned int read_32bit_phnx_ctrl_reg(int block, int reg) 
{ 
  unsigned int __res;                                    
  
  __asm__ __volatile__(                                   
		       ".set\tpush\n\t"					
		       ".set\tnoreorder\n\t" 
		       "move $9, %1\n" 
/* 		       "mfcr\t$8, $9\n\t"          */
		       ".word 0x71280018\n"
		       "move %0, $8\n"
		       ".set\tpop"	 
		       : "=r" (__res) : "r"((block<<8)|reg)
		       : "$8", "$9"
		       );   
  return __res;
}

static __inline__ void write_32bit_phnx_ctrl_reg(int block, int reg, unsigned int value)
{
  __asm__ __volatile__(                                   
		       ".set\tpush\n\t"					
		       ".set\tnoreorder\n\t" 
		       "move $8, %0\n"
		       "move $9, %1\n"
/* 		       "mtcr\t$8, $9\n\t"  */
		       ".word 0x71280019\n"
		       ".set\tpop"	 
		       : 
		       : "r" (value), "r"((block<<8)|reg)  
		       : "$8", "$9"
		       );
}

static __inline__ unsigned long long read_64bit_phnx_ctrl_reg(int block, int reg)
{	
	unsigned int high, low;						
	
	__asm__ __volatile__(					
		".set\tmips64\n\t"				
		"move    $9, %2\n"
		/* "mfcr    $8, $9\n" */
		".word   0x71280018\n"
		"dsrl32  %0, $8, 0\n\t"			        
		"dsll32  $8, $8, 0\n\t"                         
		"dsrl32  %1, $8, 0\n\t"                         
		".set mips0"					
		: "=r" (high), "=r"(low)
		: "r"((block<<8)|reg)
		: "$8", "$9"
		);	
		
	return ( (((unsigned long long)high)<<32) | low);
}

static __inline__ void write_64bit_phnx_ctrl_reg(int block, int reg,unsigned long long value)
{
	__u32 low, high;
	high = value >> 32;
	low = value & 0xffffffff;

	__asm__ __volatile__(
		".set push\n"
		".set noreorder\n"
		".set mips4\n\t"
		/* Set up "rs" */
		"move $9, %0\n"

		/* Store 64 bit value in "rt" */
		"dsll32 $10, %1, 0  \n\t"
		"dsll32 $8, %2, 0  \n\t"
		"dsrl32 $8, $8, 0  \n\t"
		"or     $10, $8, $8 \n\t"

		".word 0x71280019\n" /* mtcr $8, $9 */

		".set pop\n"

		:  /* No outputs */
		: "r"((block<<8)|reg), "r" (high), "r" (low)
		: "$8", "$9", "$10"
		);
}
typedef struct { volatile int value; } phnx_atomic_t;

static __inline__ int phnx_test_and_set(phnx_atomic_t *lock)
{
  int oldval = 0;

  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			"move $9, %2\n"
			"li $8, 1\n"
			//"swapw $8, $9\n"
			".word 0x71280014\n"
			"move %1, $8\n"
			".set pop\n"
			: "+m" (lock->value), "=r" (oldval)
			: "r" ((unsigned long)&lock->value)
			: "$8", "$9"
			);
  return (oldval == 0 ? 1/*success*/ : 0/*failure*/);
}

#endif

#endif /* _ASM_RMI_MIPS_EXTS_H */
