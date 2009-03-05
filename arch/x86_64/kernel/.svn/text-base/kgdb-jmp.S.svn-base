/*
 * arch/x86_64/kernel/kgdb-jmp.S
 *
 * Save and restore system registers so that within a limited frame we
 * may have a fault and "jump back" to a known safe location.
 *
 * Author: Tom Rini <trini@kernel.crashing.org>
 *
 * Cribbed from glibc, which carries the following:
 * Copyright (C) 2001, 2003, 2004 Free Software Foundation, Inc.
 * Copyright (C) 2005 by MontaVista Software.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program as licensed "as is" without any warranty of
 * any kind, whether express or implied.
 */

#include <linux/linkage.h>

#define JB_RBX		0
#define JB_RBP		1
#define JB_R12		2
#define JB_R13		3
#define JB_R14		4
#define JB_R15		5
#define JB_RSP		6
#define JB_PC		7

	.code64

/* This must be called prior to kgdb_fault_longjmp and
 * kgdb_fault_longjmp must not be called outside of the context of the
 * last call to kgdb_fault_setjmp.
 */
ENTRY(kgdb_fault_setjmp)
	/* Save registers. */
	movq %rbx, (JB_RBX*8)(%rdi)
	movq %rbp, (JB_RBP*8)(%rdi)
	movq %r12, (JB_R12*8)(%rdi)
	movq %r13, (JB_R13*8)(%rdi)
	movq %r14, (JB_R14*8)(%rdi)
	movq %r15, (JB_R15*8)(%rdi)
	leaq 8(%rsp), %rdx	/* Save SP as it will be after we return. */
	movq %rdx, (JB_RSP*8)(%rdi)
	movq (%rsp), %rax	/* Save PC we are returning to now. */
	movq %rax, (JB_PC*8)(%rdi)
	/* Set return value for setjmp. */
	mov $0,%eax
	movq (JB_PC*8)(%rdi),%rdx
	movq (JB_RSP*8)(%rdi),%rsp
	jmpq *%rdx

ENTRY(kgdb_fault_longjmp)
	/* Restore registers. */
	movq (JB_RBX*8)(%rdi),%rbx
	movq (JB_RBP*8)(%rdi),%rbp
	movq (JB_R12*8)(%rdi),%r12
	movq (JB_R13*8)(%rdi),%r13
	movq (JB_R14*8)(%rdi),%r14
	movq (JB_R15*8)(%rdi),%r15
	/* Set return value for setjmp. */
	movq (JB_PC*8)(%rdi),%rdx
	movq (JB_RSP*8)(%rdi),%rsp
	mov $1,%eax
	jmpq *%rdx
