/*
 * Copyright (C) 1996 Paul Mackerras
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program as licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/config.h>
#include <asm/processor.h>
#include <asm/ppc_asm.h>

	.text

/*
 * Save/restore state in case a memory access causes a fault.
 *
 * int kgdb_fault_setjmp(unsigned long *curr_context);
 * void kgdb_fault_longjmp(unsigned long *curr_context);
 */

_GLOBAL(kgdb_fault_setjmp)
	mflr	r0
	stw	r0,0(r3)
	stw	r1,4(r3)
	stw	r2,8(r3)
	mfcr	r0
	stw	r0,12(r3)
	stmw	r13,16(r3)
	li	r3,0
	blr

_GLOBAL(kgdb_fault_longjmp)
	lmw     r13,16(r3)
	lwz     r0,12(r3)
	mtcrf   0x38,r0
	lwz     r0,0(r3)
	lwz     r1,4(r3)
	lwz     r2,8(r3)
	mtlr    r0
	mr      r3,r1
	blr
