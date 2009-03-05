#ifdef __KERNEL__
#ifndef _ASM_KGDB_H_
#define _ASM_KGDB_H_

/*
 * Copyright (C) 2001-2004 Amit S. Kale
 */

#include <asm-generic/kgdb.h>

/*
 *  Note that this register image is in a different order than
 *  the register image that Linux produces at interrupt time.
 *
 *  Linux's register image is defined by struct pt_regs in ptrace.h.
 *  Just why GDB uses a different order is a historical mystery.
 */
#define _RAX	0
#define _RDX	1
#define _RCX	2
#define _RBX	3
#define _RSI	4
#define _RDI	5
#define _RBP	6
#define _RSP	7
#define _R8	8
#define _R9	9
#define _R10	10
#define _R11	11
#define _R12	12
#define _R13	13
#define _R14	14
#define _R15	15
#define _PC	16
#define _PS	17

/* Number of bytes of registers.  */
#define NUMREGBYTES		((_PS+1)*8)
#define NUMCRITREGBYTES		(8 * 8)		/* 8 registers. */

/* Help GDB to know when to stop backtracing. */
#define CFI_END_FRAME(func)	__CFI_END_FRAME(_PC,_RSP,func)
#ifndef __ASSEMBLY__
/* BUFMAX defines the maximum number of characters in inbound/outbound
 * buffers at least NUMREGBYTES*2 are needed for register packets, and
 * a longer buffer is needed to list all threads. */
#define BUFMAX			1024
#define BREAKPOINT()		asm("   int $3");
#define BREAK_INSTR_SIZE	1
#define CHECK_EXCEPTION_STACK() ((&__get_cpu_var(init_tss))[0].ist[0])
#define CACHE_FLUSH_IS_SAFE	1
#endif				/* !__ASSEMBLY__ */
#endif				/* _ASM_KGDB_H_ */
#endif				/* __KERNEL__ */
