/*
 * KGDB Support for XTensa
 *
 * Copyright (C) 2004-2005 MontaVista Software Inc.
 * Author: Manish Lachwani, mlachwani@mvista.com
 */

#ifdef __KERNEL__
#ifndef _ASM_KGDB_H_
#define _ASM_KGDB_H_

#include <asm-generic/kgdb.h>
#include <asm/config/gdb.h>

#ifndef __ASSEMBLY__
#define BUFMAX			2048
#define NUMREGBYTES		XTENSA_GDB_REGISTERS_SIZE
#define NUMCRITREGBYTES		XTENSA_GDB_REGISTERS_SIZE
#if XCHAL_HAVE_DENSITY
#define BREAK_INSTR_SIZE       2
#define BREAKPOINT()           __asm__ __volatile__ (          \
                                        ".globl breakinst\n\t"  \
                                        "breakinst:\tbreak.n 0\n\t")
#else
#define BREAK_INSTR_SIZE       3
#define BREAKPOINT()           __asm__ __volatile__ (          \
                                        ".globl breakinst\n\t"  \
                                        "breakinst:\tbreak 0, 0\n\t")
#endif 

#define CHECK_EXCEPTION_STACK()	1
#define CACHE_FLUSH_IS_SAFE	1

extern int kgdb_early_setup;

#endif				/* !__ASSEMBLY__ */
#endif				/* _ASM_KGDB_H_ */
#endif				/* __KERNEL__ */
