/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2004 Cavium Networks
 */
#include <linux/config.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/addrspace.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

	.text
	.set	noat
	.set	noreorder
	.align  5
        NESTED(octeon_handle_irq, PT_SIZE, sp)
        SAVE_ALL
        CLI
        .set    at
        .set    noreorder
        mfc0    t0, CP0_CAUSE
        mfc0    t2, CP0_STATUS

        and     t0, t2

        andi    t1, t0, STATUSF_IP7     /* cpu timer */
        bnez    t1, ll_cputimer_irq
        andi    t1, t0, STATUSF_IP2
        bnez    t1, ll_ciu_interrupt_check
        andi    t1, t0, STATUSF_IP3
        bnez    t1, ll_cpu_ip3
        andi    t1, t0, STATUSF_IP4
        bnez    t1, ll_cpu_ip4
        andi    t1, t0, STATUSF_IP5
        bnez    t1, ll_cpu_ip5
        andi    t1, t0, STATUSF_IP6
        bnez    t1, ll_cpu_ip6
        andi    t1, t0, STATUSF_IP0     /* software int 0 */
        bnez    t1, ll_cpu_ip0
        andi    t1, t0, STATUSF_IP1     /* software int 1 */
        bnez    t1, ll_cpu_ip1
        nop

        /* wrong alarm or masked ... */
        j       spurious_interrupt
	nop

ll_cputimer_irq:
        li      a0, 7
        jal     do_IRQ
        move    a1, sp
        j       ret_from_irq
	nop


ll_cpu_ip0:
        li      a0, 0
        jal     do_IRQ
        move    a1, sp
        j       ret_from_irq
	nop

ll_cpu_ip1:
        li      a0, 1
        jal     do_IRQ
        move    a1, sp
        j       ret_from_irq
	nop

ll_cpu_ip3:
        li      a0, 3
#ifdef CONFIG_SMP
        jal     mailbox_interrupt
#else
        jal     do_IRQ
#endif
        move    a1, sp
        j       ret_from_irq
	nop

ll_cpu_ip4:
        li      a0, 4
        jal     do_IRQ
        move    a1, sp
        j       ret_from_irq
	nop

ll_cpu_ip5:
        li      a0, 5
        jal     do_IRQ
        move    a1, sp
        j       ret_from_irq
	nop

ll_cpu_ip6:
        li      a0, 6
        jal     do_IRQ
        move    a1, sp
        j       ret_from_irq
	nop

ll_ciu_interrupt_check:
	dli	t0, 0x8001070000000000	// OCTEON_CIU_INTX_SUM0
	rdhwr	t1, $0			// Get the core id
	li	a0, 8			// Start at interrupt 8
	dsll	t1, 4			// core id * 2 * 8
	or	t0, t0, t1		// OCTEON_CIU_INTX_SUM0 + core id * 2 * 8
	ld	t2, 0(t0)
	dli	t0, 0x8001070000000200	// OCTEON_CIU_INTX_EN0
	dsll	t1, 1			// (core id * 2 * 8) * 2
	or	t0, t0, t1		// OCTEON_CIU_INTX_EN0 + core id * 2 * 16
	ld	t0, 0(t0)
	and	t0, t0, t2		// OCTEON_CIU_INTX_SUM0 and OCTEON_CIU_INTX_EN0

ll_ciu_interrupt_check_loop:
#ifdef __OCTEON__
	bbit1	t0, 0, ll_ciu_interrupt
#else
	and	t1, t0, 1
	bnez	t1, ll_ciu_interrupt
#endif
	dsrl	t0, t0, 1
	bne	t0, 0, ll_ciu_interrupt_check_loop
	add	a0, 1
        j       spurious_interrupt
	nop

ll_ciu_interrupt:
        jal     do_IRQ
	move	a1, sp
        j       ret_from_irq
	nop

        END(octeon_handle_irq)

