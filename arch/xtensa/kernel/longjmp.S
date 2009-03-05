/* longjmp for Xtensa Processors.

   Adopted from the Glibc implementation of longjmp.S

   Copyright (C) 2004-2005 MontaVista Software Inc.
   Author: Manish Lachwani, mlachwani@mvista.com

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with the GNU C Library; see the file COPYING.LIB.  If not,
   write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

#include <linux/linkage.h>
#include <asm/regs.h>
#include <asm/processor.h>
#include <asm/ptrace.h>
#include <asm/config/core.h>
#include <asm/xtensa/coreasm.h>

#define _SETJMP_H

ENTRY(kgdb_longjmp)

	entry	sp, 32

        movi    a5, PS_EXCM_MASK | LOCKLEVEL
        xsr     a5, PS
        rsr     a3, EXCSAVE_1
        mov    a6, a0
        s32i    a3, a3, EXC_TABLE_FIXUP
        call0   _spill_registers
        rsr     a3, EXCSAVE_1           # exc_table
        mov    a0, a6
        movi    a6, 0
        s32i    a6, a3, EXC_TABLE_FIXUP
        wsr     a5, PS
        rsync


	/* Return to the return address of the setjmp, using the
	   window size bits from the setjmp call so that the caller
	   will be able to find the return value that we put in a2. */

	l32i	a0, a2, 64

	/* Copy the first 4 saved registers from jmp_buf into the save area
	   at the current sp so that the values will be restored to registers
	   when longjmp returns. */

	addi	a7, a1, -16
	l32i	a4, a2, 0
	l32i	a5, a2, 4
	s32i	a4, a7, 0
	s32i	a5, a7, 4
	l32i	a4, a2, 8
	l32i	a5, a2, 12
	s32i	a4, a7, 8
	s32i	a5, a7, 12

	/* Copy the remaining 0-8 saved registers. */
	extui	a7, a0, 30, 2
	blti	a7, 2, .Lendlj
	l32i	a8, a2, 52
	slli	a4, a7, 4
	sub	a6, a8, a4
	addi	a5, a2, 16
	addi	a8, a8, -16		# a8 = end of register overflow area
.Lljloop:
	l32i	a7, a5, 0
	l32i	a4, a5, 4
	s32i	a7, a6, 0
	s32i	a4, a6, 4
	l32i	a7, a5, 8
	l32i	a4, a5, 12
	s32i	a7, a6, 8
	s32i	a4, a6, 12
	addi	a5, a5, 16
	addi	a6, a6, 16
	blt	a6, a8, .Lljloop
.Lendlj:

	/* The 4 words saved from the register save area at the target's
	   sp are copied back to the target procedure's save area.  The
	   only point of this is to prevent a catastrophic failure in
	   case the contents were moved by an alloca after calling
	   setjmp.  This is a bit paranoid but it doesn't cost much. */

	l32i	a7, a2, 4		# load the target stack pointer
	addi	a7, a7, -16		# find the destination save area
	l32i	a4, a2, 48
	l32i	a5, a2, 52
	s32i	a4, a7, 0
	s32i	a5, a7, 4
	l32i	a4, a2, 56
	l32i	a5, a2, 60
	s32i	a4, a7, 8
	s32i	a5, a7, 12

	/* Restore the special registers. */
#if XCHAL_HAVE_LOOPS
	l32i	a4, a2, 68
	l32i	a5, a2, 72
	wsr	a4, SAR
	wsr	a5, LCOUNT

	l32i	a4, a2, 76
	l32i	a5, a2, 80
	wsr	a4, LBEG
	wsr	a5, LEND

#else /* !XCHAL_HAVE_LOOPS */
	l32i	a4, a2, 68
	wsr	a4, SAR
#endif /* !XCHAL_HAVE_LOOPS */

#if XCHAL_HAVE_MAC16
	l32i	a4, a2, 84
	l32i	a5, a2, 88
	wsr	a4, ACCLO
	wsr	a5, ACCHI

	l32i	a4, a2, 92
	l32i	a5, a2, 96
	wsr	a4, MR_0
	wsr	a5, MR_1

	l32i	a4, a2, 100
	l32i	a5, a2, 104
	wsr	a4, MR_2
	wsr	a5, MR_3

#endif /* XCHAL_HAVE_MAC16 */
	/* wait for the special registers to be updated */
	isync

	/* return v ? v : 1 */
	movi	a2, 1

	retw
