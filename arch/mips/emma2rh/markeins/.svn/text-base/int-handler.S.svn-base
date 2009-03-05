/*
 *  arch/mips/emma2rh/markeins/int-handler.S
 *      first level dispatcher for EMMA2RH Mark-eins.
 *
 *  Copyright (C) NEC Electronics Corporation 2004-2006
 *
 *  This file is based on the arch/mips/ddb5xxx/ddb5477/int-handler.S
 *
 *	Copyright 2001 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/addrspace.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/emma2rh/emma2rh.h>

/*
 * first level interrupt dispatcher for Mark-eins board -
 * We check for the timer first, then check PCI ints A and D.
 * Then check for serial IRQ and fall through.
 */
	.align	5
	NESTED(emma2rh_handle_int, PT_SIZE, sp)
	SAVE_ALL
	CLI
	.set	at
	.set	noreorder
	mfc0	t0, CP0_CAUSE
	mfc0	t2, CP0_STATUS

	and	t0, t2

	andi	t1, t0, STATUSF_IP7	/* cpu timer */
	bnez	t1, ll_cputimer_irq
	andi	t1, t0, STATUSF_IP2	/* emma2rh int 2 */
	bnez	t1, ll_emma2rh_irq
	andi	t1, t0, STATUSF_IP1	/* software int 1 */
	bnez	t1, ll_cpu_ip1
	andi	t1, t0, STATUSF_IP0	/* software int 0 */
	bnez	t1, ll_cpu_ip0
	nop
	.set	reorder

	/* wrong alarm or masked ... */
	j	spurious_interrupt
	nop
	END(emma2rh_handle_int)

	.align	5

ll_emma2rh_irq:
	move	a0, sp
	jal	emma2rh_irq_dispatch
	j	ret_from_irq

ll_cputimer_irq:
	li	a0, CPU_IRQ_BASE + 7
	move	a1, sp
	jal	do_IRQ
	j	ret_from_irq


ll_cpu_ip0:
	li	a0, CPU_IRQ_BASE + 0
	move	a1, sp
	jal	do_IRQ
	j	ret_from_irq

ll_cpu_ip1:
	li	a0, CPU_IRQ_BASE + 1
	move	a1, sp
	jal	do_IRQ
	j	ret_from_irq
