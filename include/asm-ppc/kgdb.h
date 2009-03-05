/*
 * kgdb.h: Defines and declarations for serial line source level
 *         remote debugging of the Linux kernel using gdb.
 *
 * PPC Mods (C) 2004 Tom Rini (trini@mvista.com)
 * PPC Mods (C) 2003 John Whitney (john.whitney@timesys.com)
 * PPC Mods (C) 1998 Michael Tesch (tesch@cs.wisc.edu)
 *
 * Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 */
#ifdef __KERNEL__
#ifndef _PPC_KGDB_H
#define _PPC_KGDB_H

#include <asm-generic/kgdb.h>
#ifndef __ASSEMBLY__

#define BREAK_INSTR_SIZE	4
#ifndef CONFIG_E500
#define MAXREG			(PT_FPSCR+1)
#else
/* 32 GPRs (8 bytes), nip, msr, ccr, link, ctr, xer, acc (8 bytes), spefscr*/
#define MAXREG                 ((32*2)+6+2+1) 
#endif
#define NUMREGBYTES		(MAXREG * 4) 
#define BUFMAX			((NUMREGBYTES * 2) + 512)
#define OUTBUFMAX		((NUMREGBYTES * 2) + 512)
/* CR/LR, R1, R2, R13-R31 inclusive. */
#define NUMCRITREGBYTES		(23 * sizeof(int))
#define BREAKPOINT()		asm(".long 0x7d821008"); /* twge r2, r2 */
#define CHECK_EXCEPTION_STACK()	1
#define CACHE_FLUSH_IS_SAFE	1

/* For taking exceptions
 * these are defined in traps.c
 */
struct pt_regs;
extern void (*debugger)(struct pt_regs *regs);
extern int (*debugger_ipi)(struct pt_regs *regs);
extern int (*debugger_bpt)(struct pt_regs *regs);
extern int (*debugger_sstep)(struct pt_regs *regs);
extern int (*debugger_iabr_match)(struct pt_regs *regs);
extern int (*debugger_dabr_match)(struct pt_regs *regs);
extern void (*debugger_fault_handler)(struct pt_regs *regs);
#endif /* !(__ASSEMBLY__) */
#endif /* !(_PPC_KGDB_H) */
#endif /* __KERNEL__ */
