/*
 *  linux/arch/i386/mcount-wrapper.S
 *
 *  Copyright (C) 2004 Ingo Molnar
 */

.globl mcount
mcount:

	cmpl $0, mcount_enabled
	jz out

	push %ebp
	mov %esp, %ebp
	pushl %eax
	pushl %ecx
	pushl %edx

	call __mcount

	popl %edx
	popl %ecx
	popl %eax
	popl %ebp
out:
	ret

