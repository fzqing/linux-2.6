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
#define _EAX	0
#define _ECX	1
#define _EDX	2
#define _EBX	3
#define _ESP	4
#define _EBP	5
#define _ESI	6
#define _EDI	7
#define _PC	8
#define _EIP	8
#define _PS	9
#define _EFLAGS	9
#define _CS	10
#define _SS	11
#define _DS	12
#define _ES	13
#define _FS	14
#define _GS	15

/* So that we can denote the end of a frame for tracing, in the simple
 * case. */
#define CFI_END_FRAME(func)	__CFI_END_FRAME(_EIP,_ESP,func)

/************************************************************************/
/* BUFMAX defines the maximum number of characters in inbound/outbound buffers*/
/* at least NUMREGBYTES*2 are needed for register packets */
/* Longer buffer is needed to list all threads */
#define BUFMAX			1024

/* Number of bytes of registers.  */
#define NUMREGBYTES		64
/* Number of bytes of registers we need to save for a setjmp/longjmp. */
#define NUMCRITREGBYTES		24

#ifndef __ASSEMBLY__
#define BREAKPOINT()		asm("   int $3");
#define BREAK_INSTR_SIZE	1
#define CHECK_EXCEPTION_STACK()	1
#define CACHE_FLUSH_IS_SAFE	1
#endif				/* !__ASSEMBLY__ */
#endif				/* _ASM_KGDB_H_ */
#endif				/* __KERNEL__ */
