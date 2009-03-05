/*
 * Copyright (C) 2005 Cavium Networks
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * This code is a workaround for a bug in the first two batches of Octeon
 * Pass 1. The bug sometimes causes bus errors in userspace if a kernel
 * address is in the internal return stack used for code prefetching. This
 * workaround is mapped into a userspace address and called at the end of
 * interrupt or exception processing. It performs four "jal" calls to make
 * sure kernel addresses are flushed out of the internal return stack. A
 * fix has already been sent, so all these chips will be recalled june 2005.
 * This code assumes it was called with a "jal k0" instruction.
 */
#include <linux/config.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/addrspace.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

	.text
	.set	noat
	.set 	noreorder
	.align  12
	NESTED(octeon_adel_workaround, PT_SIZE, sp)
	or	k0, ra, ra
	bgezal	$0, 1f
	nop
1:	bgezal	$0, 2f
	nop
2:	bgezal	$0, 3f
	nop
3:	bgezal	$0, 4f
	nop
4:	jr	k0
	nop
	.align	12
	END(octeon_adel_workaround)

