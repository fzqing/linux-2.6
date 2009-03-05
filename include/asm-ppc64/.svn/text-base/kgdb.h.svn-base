/*
 * kgdb.h: Defines and declarations for serial line source level
 *         remote debugging of the Linux kernel using gdb.
 *
 * copied from include/asm-ppc, modified for ppc64
 *
 * PPC64 Mods (C) 2005 Frank Rowand (frowand@mvista.com)
 * PPC Mods (C) 2004 Tom Rini (trini@mvista.com)
 * PPC Mods (C) 2003 John Whitney (john.whitney@timesys.com)
 * PPC Mods (C) 1998 Michael Tesch (tesch@cs.wisc.edu)
 *
 * Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 */
#ifdef __KERNEL__
#ifndef _PPC64_KGDB_H
#define _PPC64_KGDB_H

#include <asm-generic/kgdb.h>
#ifndef __ASSEMBLY__

#define BREAK_INSTR_SIZE	4
				/*
				 * 64 bit (8 byte) registers:
				 *   32 gpr, 32 fpr, nip, msr, link, ctr
				 * 32 bit (4 byte) registers:
				 *   ccr, xer, fpscr, vscr, vrsave
				 *
				 * The final "+ 4" is to make the value a
				 * multiple of sizeof(long) to match the
				 * expectations of gdb_reg[] in kernel/kgdb.c.
				 * In the long term, the expectations will be
				 * fixed and this extra "+ 4" can be removed.
				 */
#define NUMREGBYTES		((68 * 8) + (5 * 4) + 4)
#define NUMCRITREGBYTES		184
#define BUFMAX			((NUMREGBYTES * 2) + 512)
#define OUTBUFMAX		((NUMREGBYTES * 2) + 512)
#define BREAKPOINT()		asm(".long 0x7d821008"); /* twge r2, r2 */
#define CHECK_EXCEPTION_STACK()	1
#define CACHE_FLUSH_IS_SAFE	1

#endif /* !(__ASSEMBLY__) */

#endif /* !(_PPC64_KGDB_H) */

#endif /* __KERNEL__ */
