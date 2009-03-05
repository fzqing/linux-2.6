/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */
#include <linux/config.h>

#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>	
#include <asm/stackframe.h>

#include <asm/rmi/interrupt.h>

	.text
	.set	push
	.set	noreorder
	.set	mips64
	.align	5
	NESTED(phoenix_irq_handler, PT_SIZE, sp)
	SAVE_ALL
	CLI

	/* Read EIRR :	 */
	/* dmfc0 s0, eirr */
	.word   0x40304806        
		
	/* If no interrupts, return */
	beqz    s0, _done
	nop

	/* Check for IP7 (Count - Compare interrupt) specially */
	andi    s1, s0, (1 << IRQ_TIMER)
	beqz    s1, 1f
	nop
	
	/* No need to clear the EIRR here. the handler is gonna
	 *  write to compare which clears eirr also
	 */
	
	/* phoenix_timer_interrupt((a0 = sp)) */
#ifdef CONFIG_CPU_TIMER
	li	a0, IRQ_TIMER
	move	a1, sp
	jal     ll_timer_interrupt
	nop
	j       ret_from_irq
	nop
#else
	li	a1, IRQ_TIMER
	move	a0, sp
	jal	phoenix_timer_interrupt
	nop
	j	ret_from_irq
	nop
#endif

	/* All other interrupts */
1:
	/* retrieve the highest priority interrupt */
	.word   0x72118824        /* dclz s1 s0 */
	dsubu   a0, zero, s1
	daddiu  a0, a0, 63
	/* a0 now has irq# */
	move    a1, sp
	/* a1 now has sp */

	/* first things first :	clear the irq in eirr
	*  note that setting a bit in eirr actually clears it
	*/
	li      s0, 1
	dsllv   s0, s0, a0
	.word   0x40b04806        /* dmtc0 s0, eirr */  

	/* a0 = irq, a1 = sp (regs) */
	jal	do_phnx_IRQ
	nop

_done:		
	j        ret_from_irq
	 nop

	.set pop
	END(phoenix_irq_handler)
