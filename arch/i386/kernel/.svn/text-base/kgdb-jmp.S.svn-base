/*
 * arch/i386/kernel/kgdb-jmp.S
 *
 * Save and restore system registers so that within a limited frame we
 * may have a fault and "jump back" to a known safe location.
 *
 * Author: George Anzinger <george@mvista.com>
 *
 * Cribbed from glibc, which carries the following:
 * Copyright (C) 1996, 1996, 1997, 2000, 2001 Free Software Foundation, Inc.
 * Copyright (C) 2005 by MontaVista Software.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program as licensed "as is" without any warranty of
 * any kind, whether express or implied.
 */

#include <linux/linkage.h>

#define PCOFF		0
#define LINKAGE		4		/* just the return address */
#define PTR_SIZE	4
#define PARMS		LINKAGE		/* no space for saved regs */
#define JMPBUF		PARMS
#define VAL		JMPBUF+PTR_SIZE

#define JB_BX		0
#define JB_SI		1
#define JB_DI		2
#define JB_BP		3
#define JB_SP		4
#define JB_PC		5

/* This must be called prior to kgdb_fault_longjmp and
 * kgdb_fault_longjmp must not be called outside of the context of the
 * last call to kgdb_fault_setjmp.
 * kgdb_fault_setjmp(int *jmp_buf[6])
 */
ENTRY(kgdb_fault_setjmp)
	movl JMPBUF(%esp), %eax

	/* Save registers.  */
	movl	%ebx, (JB_BX*4)(%eax)
	movl	%esi, (JB_SI*4)(%eax)
	movl	%edi, (JB_DI*4)(%eax)
	/* Save SP as it will be after we return.  */
	leal	JMPBUF(%esp), %ecx
	movl	%ecx, (JB_SP*4)(%eax)
	movl	PCOFF(%esp), %ecx	/* Save PC we are returning to now.  */
	movl	%ecx, (JB_PC*4)(%eax)
	movl	%ebp, (JB_BP*4)(%eax)	/* Save caller's frame pointer.  */

	/* Restore state so we can now try the access. */
	movl	JMPBUF(%esp), %ecx	/* User's jmp_buf in %ecx.  */
	/* Save the return address now.  */
	movl	(JB_PC*4)(%ecx), %edx
	/* Restore registers.  */
	movl	$0, %eax
	movl	(JB_SP*4)(%ecx), %esp
	jmp	*%edx		/* Jump to saved PC. */

/* kgdb_fault_longjmp(int *jmp_buf[6]) */
ENTRY(kgdb_fault_longjmp)
	movl	JMPBUF(%esp), %ecx	/* User's jmp_buf in %ecx.  */
	/* Save the return address now.  */
	movl	(JB_PC*4)(%ecx), %edx
	/* Restore registers.  */
	movl	(JB_BX*4)(%ecx), %ebx
	movl	(JB_SI*4)(%ecx), %esi
	movl	(JB_DI*4)(%ecx), %edi
	movl	(JB_BP*4)(%ecx), %ebp
	movl	$1, %eax
	movl	(JB_SP*4)(%ecx), %esp
	jmp	*%edx		/* Jump to saved PC. */
