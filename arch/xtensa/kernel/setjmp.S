/*
   setjmp for Xtensa Processors.

   Adopted from Glibc implementation of setjmp.S

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
#include <asm/ptrace.h>
#include <asm/processor.h>
#include <asm/config/core.h>
#include <asm/xtensa/coreasm.h>

ENTRY(kgdb_fault_setjmp_aux)

	entry   sp, 16

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

	/* copy the register save area at (sp - 16) */
	addi	a5, a1, -16
	l32i	a3, a5, 0
	l32i	a4, a5, 4
	s32i	a3, a2, 0
	s32i	a4, a2, 4
	l32i	a3, a5, 8
	l32i	a4, a5, 12
	s32i	a3, a2, 8
	s32i	a4, a2, 12

	/* copy 0-8 words from the register overflow area */
	extui	a3, a0, 30, 2
	blti	a3, 2, .Lendsj
	l32i	a7, a1, 4
	slli	a4, a3, 4
	sub	a5, a7, a4
	addi	a6, a2, 16
	addi	a7, a7, -16		# a7 = end of register overflow area
.Lsjloop:
	l32i	a3, a5, 0
	l32i	a4, a5, 4
	s32i	a3, a6, 0
	s32i	a4, a6, 4
	l32i	a3, a5, 8
	l32i	a4, a5, 12
	s32i	a3, a6, 8
	s32i	a4, a6, 12
	addi	a5, a5, 16
	addi	a6, a6, 16
	blt	a5, a7, .Lsjloop
.Lendsj:

	/* copy the register save area at (sp) = (sp + 16 - 16) */
	l32i	a3, a1, 0
	l32i	a4, a1, 4
	s32i	a3, a2, 48
	s32i	a4, a2, 52
	l32i	a3, a1, 8
	l32i	a4, a1, 12
	s32i	a3, a2, 56
	s32i	a4, a2, 60

	/* save the return address, including the window size bits */
	s32i	a0, a2, 64

	/* save the special registers */
#if XCHAL_HAVE_LOOPS
	rsr	a3, SAR
	rsr	a4, LCOUNT
	s32i	a3, a2, 68
	s32i	a4, a2, 72

	rsr	a3, LBEG
	rsr	a4, LEND
	s32i	a3, a2, 76
	s32i	a4, a2, 80

#else /* !XCHAL_HAVE_LOOPS */
	rsr	a3, SAR
	movi	a4, 0
	s32i	a3, a2, 68
	s32i	a4, a2, 72

	s32i	a4, a2, 76
	s32i	a4, a2, 80
#endif /* !XCHAL_HAVE_LOOPS */

#if XCHAL_HAVE_MAC16
	rsr	a3, ACCLO
	rsr	a4, ACCHI
	s32i	a3, a2, 84
	s32i	a4, a2, 88

	rsr	a3, MR_0
	rsr	a4, MR_1
	s32i	a3, a2, 92
	s32i	a4, a2, 96

	rsr	a3, MR_2
	rsr	a4, MR_3
	s32i	a3, a2, 100
	s32i	a4, a2, 104
#endif /* XCHAL_HAVE_MAC16 */

	movi    a2, 0
	retw
