/*
 * Low-level Monahans suspend/resume support
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */

#include <linux/config.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/hardware.h>
#include <asm/arch/mhn_pm.h>

/* mhn_cpu_resume()
 * Entry point for bootloader resume to kernel
 *
 * It will invoke pm_resume_from_sleep which use area_phy_address as parameter
 */
/* Note: The following code is located into the .data section.
 *       This is to allow area_phy_address to be accessed with a relative load
 *       while we can't rely on any MMU translation.  We could have put
 *       area_phy_address in the .text section as well, but some setups might
 *       insist on it to be truly read-only.
 */

	.data
	.align 5
ENTRY(mhn_cpu_resume)
	ldr	r1, =0x80000004
	ldr	r0, [r1]
	bl	pm_resume_from_sleep
	cmp	r0, #1
	@ maybe turn on some lights for warning
error_ret:
	nop
	beq	error_ret


/* mhn_cpu_standby()
 *
 * Entry point for entering standby(S0D2C2).
 */
	.text
	.align 5
ENTRY(mhn_cpu_standby)
/* lr register will be the instruction just after invoke of mhn_cpu_standby */
	b	pm_enter_standby

/* mhn_cpu_lcdrefresh()
 *
 * Entry point for entering lcdrefresh(S0D1C2).
 */
	.text
	.align 5
ENTRY(mhn_cpu_lcdrefresh)
/* lr register will be the instruction just after invoke of mhn_cpu_lcdrefresh */
	b	pm_enter_lcd_refresh

/* mhn_cpu_sleep(unsigned int a, unsigned int b)
 *
 * Entry point for entering sleep mode(S2D3C4).
 * a:
 *	vitual address of the data save area for Monahans content
 * b:
 *	physical address of the data save area for Monahans content
 *
 * The API pm_enter_sleep will use the first parameter "a". The "b" will
 * be stored in area_phy_adress which will be used by pm_resume_from_sleep.
 */

ENTRY(mhn_cpu_sleep)
	b	pm_enter_sleep

/* mhn_cpu_deepsleep(unsigned int a, unsigned int b)
 *
 * Entry point for entering sleep mode(S2D3C4).
 * a:
 *	vitual address of the data save area for Monahans content
 * b:
 *	physical address of the data save area for Monahans content
 *
 * The pm_enter_sleep will use the first parameter "a". The "b" will
 * be stored in area_phy_adress which will be used by pm_resume_from_sleep.
 */

ENTRY(mhn_cpu_deepsleep)
	b	pm_enter_deep_sleep


@*****************************************************************************
@ pm_checksum_calculate
@
@ Calculate checksum
@
@ Inputs:
@	r0: the virutal address of the data area which will be calculated the
@		checksum
@	r1: the toltal word of the data area. Checksum is done on 4-byte word
@
@ Output:
@	checksum
@
@ Registers used
@	r0, r1, r2, r3
@

pm_checksum_calculate:

	@ pick a non-zero seed
	ldr	r2, =(0x5A72)
calculate:
	@ get value and increment pointer
	ldr	r3, [r0], #4
	add	r2, r2, r3
	@ rotate left by one bit position
	mov	r2, r2, ROR #31
	subs	r1, r1, #1
	bne	calculate
	mov	r0, r2
	mov	pc, lr



@******************************************************************************
@
@ pm_resume_from_deep_sleep
@
@ Restore saved content and return back
@
@ Inputs:
@	r0: The physical address of the saved data area
@
@ Outputs:
@	None
@
pm_resume_from_deep_sleep:

@******************************************************************************
@
@ pm_resume_from_sleep
@
@ Restore saved content and return back
@
@ Inputs:
@	r0: The physical address of the saved data area
@
@ Outputs:
@	None
@

pm_resume_from_sleep:
	@ make sure that we are in SVC mode with irq and fiq off
	mov     r1, #(CPSR_Mode_SVC | CPSR_I_Bit | CPSR_F_Bit)
	msr     cpsr_c, r1

	@ Step 1
	@ validate checksum
	@ get the address of the first word that is checksumable
	mov	r9, r0
	ldr	r1, [r9, #SleepState_wordCount]
	mov	r8, lr
	add	r0, r9, #4
	bl	pm_checksum_calculate
	ldr	r3, [r9, #SleepState_checksum]
	subs	r1, r3, r0
	mov	r0, #0
	movne	r0, #1
	@ return if checksum is wrong
	movne	pc, r8
	mov	r0, r9

	@ Step 2
	ldr	r9, [r0, #SleepState_Cp15_ACR_MMU]
	ldr	r8, [r0, #SleepState_Cp15_AUXCR_MMU]
	ldr	r7, [r0, #SleepState_Cp15_TTBR_MMU]
	ldr	r6, [r0, #SleepState_Cp15_DACR_MMU]
	ldr	r5, [r0, #SleepState_Cp15_PID_MMU]
	ldr	r4, [r0, #SleepState_Cp15_CPAR]
	ldr	r0, [r0, #SleepState_areaAddress]

	@ invalidate I, D caches & BTB
	mcr     p15, 0, ip, c7, c7, 0
	@ Drain Write (& Fill) Buffer
	mcr     p15, 0, ip, c7, c10, 4
	@ Prefetch Flush
	mcr     p15, 0, ip, c7, c5, 4
	@ invalidate I, D TLBs
	mcr     p15, 0, ip, c8, c7, 0

	@ Step 3
	@ Rrestore MMU settings and turn on MMU
	mcr	p15, 0, r4, c15, c1, 0
	mcr	p15, 0, r5, c13, c0, 0
	mcr	p15, 0, r6, c3, c0, 0
	mcr	p15, 0, r7, c2, c0, 0
	mcr	p15, 0, r8, c1, c0, 1

	@ Get page table address
	mrc     p15, 0, r1, c2, c0, 0
	bic     r1, r1, #0xff
	bic     r1, r1, #0x3f00
	ldr	r2, =0x542e

	@ Mapping resume_turn_on_mmu in the pagetable
	adr	r3, resume_turn_on_mmu
	mov	r3, r3, lsr #20
	orr	r4, r2, r3, lsl #20
	ldr	r5, [r1, r3, lsl #2]
	str     r4, [r1, r3, lsl #2]

	@ Mapping page table address in the page table
	mov	r6, r1, lsr #20
	orr	r7, r2, r6, lsl #20
	ldr	r8, [r1, r6, lsl #2]
	str	r7, [r1, r6, lsl #2]

	ldr	r10, =resume_after_turn_on_mmu
	mov	r10, r10
        b	resume_turn_on_mmu

	.align	5

resume_turn_on_mmu:
	mcr	p15, 0, r9, c1, c0, 0

	@ cp_wait
	mrc	p15, 0, r2, c2, c0, 0
	mov	r2, r2
	mov	r2, r2
	mov	pc, r10
	nop
	nop
	nop
	nop

resume_after_turn_on_mmu:
	@ Restore the Mappings in page table
	str	r5, [r1, r3, lsl #2]
	str	r8, [r1, r6, lsl #2]

	@ Step 4
	@ r0 stores the virtual address of the content save area
	@ compare "modeSaveFlag" to decide which mode will be saved
	ldr	r6, [r0, #SleepState_modeSaveFlags]
1:
	@ restore SVC content?
	ands     r1, r6, #(PM_MODE_SAVE_FLAG_SVC)
	beq	2f
	add	r7, r0, #SleepState_SVC_REGS
        ldmia   r7, {r2, sp, lr}
	msr     spsr, r2

2:
	@ restore UND mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_UND)
	beq	3f
	bic	r3, r3, #(CPSR_Mode_MASK)
    	orr	r3, r3, #(CPSR_Mode_UND | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_UND_REGS
        ldmia   r7, {r2, sp, lr}
	msr     spsr, r2

3:
	@ restore ABT mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_ABT)
	beq	4f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_ABT | CPSR_I_Bit | CPSR_F_Bit)
	msr     cpsr_c, r3
	add	r7, r0, #SleepState_ABT_REGS
        ldmia   r7, {r2, sp, lr}
	msr     spsr, r2

4:
	@ restore IRQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_IRQ)
	beq	5f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_IRQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_IRQ_REGS
        ldmia   r7, {r2, sp, lr}
	msr	spsr, r2

5:
	@ restore FIQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_FIQ)
	beq	6f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_FIQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_FIQ_REGS
        ldmia   r7, {r2, r8-r12, sp, lr}
	msr     spsr, r2

6:
	@ restore SYS mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_SYS)
	beq	7f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_SYS | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_SYS_REGS
        ldmia   r7, {sp, lr}

7:
	@ Step 5
	@ Re-establish whatever mode was in use at the time pm_enter_sleep()
	@ was invoked and restore complete register context.  Before restoring
	@ the SPSR, make sure that the entry mode was not SYS mode, which has
	@ no SPSR.

	@  Load CPSR, sp and (if not SYS mode) SPSR
	ldr	r3, [r0, #SleepState_ENTRY_CPSR]
	msr	cpsr, r3
	ldr	r2, =CPSR_Mode_SYS
	and	r3, r3, r2
	cmp	r3, r2
	ldrne	r2, [r0, #SleepState_ENTRY_SPSR]
	msrne	spsr, r2
	add	r0, r0, #SleepState_ENTRY_R0
	@ use "increase after" to skip r0 register restore,
	ldmib	r0, {r1 - r12, sp, lr}
	@ restore r0 reigster
	ldr	r0, [r0]

	@ return to next instruction after pm_enter_sleep
	mov	pc, lr

@******************************************************************************
@
@ pm_enter_standby
@
@ Put the system into S0D2C2 state
@
@ Inputs:
@	None
@
@ Outputs:
@	None
@

pm_enter_standby:

	@save registers on stack
	stmfd	sp!, {r2 - r10, lr}
	ldr	r2, =pm_enter_standby_start
	ldr	r3, =pm_enter_standby_end

	mov	r4, r0		@ ISRAM start address
	@ copy standby routine to ISRAM
rel_sram:
	ldmia	r2!, {r5-r9}
	stmia	r4!, {r5-r9}
	cmp	r2, r3
	ble	rel_sram

	ldr	r4, =0xF8D00000		@ DMEMC_REG_BASE (MDCNFG)
	ldr	r5, [r4]

	mov	pc, r0

pm_enter_standby_start:
	b	1f

	.align  5

1:
	@ enter S0D2C2 state
	mov	r5, #MHN_PM_S0D2C2
	mcr     p14, 0, r5, c7, c0, 0

	@ wait for standby
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	@ r4 stores MDCNFG address
        @ disable DDR_HCAL[HCEN]
	ldr	r5, [r4, #0x60]		@ DDR_HCAL offset 0x60
        bic     r5, r5, #0x80000000     @ clear HCEN
        str     r5, [r4, #0x60]
1:
        ldr     r5, [r4, #0x60]
        tst     r5, #0x80000000
        bne     1b

        @ initiate RCOMP[SWEAL]
        ldr     r6, [r4, #0x100]	@ RCOMP offset 0x100
        orr     r6, r6, #0x80000000
        str     r6, [r4, #0x100]

        @ clear EDLP interrupt
        ldr     r7, =0xFFFFFFFF
        str     r7, [r4, #0x78]		@ DMCISR offset 0x78

                @ set DMCIER[EDLP]
        ldr     r7, [r4, #0x70]
        orr     r7, r7, #0x20000000
        str     r7, [r4, #0x70]		@ DMCIER offset 0x70

                @ set DDR_HCAL[HCEN]
                @ set DDR_HCAL[PROG]
                @ clear DDR_HCAL[HCRNG]
        ldr     r8, [r4, #0x60]
        bic     r8, r8, #0x0000001F
        orr     r8, r8, #0x09
        str     r8, [r4, #0x60]

        @ enable MDCNFG[DMCEN]
        ldr     r5, [r4]		@ MDCNFG offset 0x00
        orr     r5, r5, #0x40000000
        str     r5, [r4]
3:
        ldr     r5, [r4]
        tst     r5, #0x40000000
        beq     3b

        @ set DDR_HCAL[HCRNG]
        ldr     r6, [r4, #0x60]
        orr     r6, r6, #2
        str     r6, [r4, #0x60]


        @ clear interrupt
        ldr     r8, [r4, #0x70]		@ DMCIER offset 0x70
        bic     r8, r8, #0x20000000
        str     r8, [r4, #0x70]

        ldmfd   sp!, {r2 - r10, pc}
pm_enter_standby_end:
        nop

@******************************************************************************
@
@ pm_enter_lcd_refresh
@
@ Put the system into S0D1C2 state
@
@ Inputs:
@	None
@
@ Outputs:
@	None
@

pm_enter_lcd_refresh:

	@save registers on stack
	stmfd	sp!, {r2 - r10, lr}
	ldr	r2, =pm_enter_lcd_ref_start
	ldr	r3, =pm_enter_lcd_ref_end

	mov	r4, r0		@ ISRAM start address
	@ copy standby routine to ISRAM
rel_sram_lcd_ref:
	ldmia	r2!, {r5-r9}
	stmia	r4!, {r5-r9}
	cmp	r2, r3
	ble	rel_sram_lcd_ref

	ldr	r4, =0xF8D00000		@ DMEMC_REG_BASE (MDCNFG)
	ldr	r5, [r4]

	mov	pc, r0

pm_enter_lcd_ref_start:
	b	1f

	.align  5

1:
	@ enter S0D1C2 state
	mov	r5, #MHN_PM_S0D1C2
	mcr     p14, 0, r5, c7, c0, 0

	@ wait for lcd refresh
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	@ r4 stores MDCNFG address
        @ disable DDR_HCAL[HCEN]
	ldr	r5, [r4, #0x60]		@ DDR_HCAL offset 0x60
        bic     r5, r5, #0x80000000     @ clear HCEN
        str     r5, [r4, #0x60]
1:
        ldr     r5, [r4, #0x60]
        tst     r5, #0x80000000
        bne     1b

        @ initiate RCOMP[SWEAL]
        ldr     r6, [r4, #0x100]	@ RCOMP offset 0x100
        orr     r6, r6, #0x80000000
        str     r6, [r4, #0x100]

        @ clear EDLP interrupt
        ldr     r7, =0xFFFFFFFF
        str     r7, [r4, #0x78]		@ DMCISR offset 0x78

                @ set DMCIER[EDLP]
        ldr     r7, [r4, #0x70]
        orr     r7, r7, #0x20000000
        str     r7, [r4, #0x70]		@ DMCIER offset 0x70

                @ set DDR_HCAL[HCEN]
                @ set DDR_HCAL[PROG]
                @ clear DDR_HCAL[HCRNG]
        ldr     r8, [r4, #0x60]
        bic     r8, r8, #0x0000001F
        orr     r8, r8, #0x09
        str     r8, [r4, #0x60]

        @ enable MDCNFG[DMCEN]
        ldr     r5, [r4]		@ MDCNFG offset 0x00
        orr     r5, r5, #0x40000000
        str     r5, [r4]
3:
        ldr     r5, [r4]
        tst     r5, #0x40000000
        beq     3b

        @ set DDR_HCAL[HCRNG]
        ldr     r6, [r4, #0x60]
        orr     r6, r6, #2
        str     r6, [r4, #0x60]

        @ clear interrupt
        ldr     r8, [r4, #0x70]		@ DMCIER offset 0x70
        bic     r8, r8, #0x20000000
        str     r8, [r4, #0x70]

        ldmfd   sp!, {r2 - r10, pc}
pm_enter_lcd_ref_end:
        nop



@*****************************************************************************
@ pm_enter_sleep_or_deep_sleep
@
@ Put the system into S2D3C4 or S3D4C4 state
@
@ Inputs:
@	r0: the virutal address of the data area to save the content of core
@	r1: sleep type, 6(sleep), 7(deep sleep)
@
@ Outputs:
@	None
@
@ Notes:
@      r1 should be saved previously
@

pm_enter_sleep_or_deep_sleep:

	@ Step 1
	@ store registers(r0-r12), sp, lr of current mode in the data array
	@ ENTRY_REGS
	@ the r0 changes to be virutal address of ENTRY_REGS
	add	r0, r0, #SleepState_ENTRY_R0
	@ skip r0 and r1 save
	add     r0, r0, #4
	stmib	r0, {r2 - r12, sp, lr}
	sub     r0, r0, #4
	mov     r11, r1 @save the sleep type
	@ save r0 register
	sub	r5, r0, #SleepState_ENTRY_R0
	str	r5, [r0]

	@ store cpsr of current mode in the data array ENTRY_REGS.
	mrs	r3, cpsr
	str	r3, [r5, #SleepState_ENTRY_CPSR]

	@ store spsr(if not SYS mode) of current mode in the content area
	ldr	r2, =CPSR_Mode_SYS
	and	r1, r3, r2
	cmp	r1, r2
	mrsne	r2, spsr
	strne	r2, [r5, #SleepState_ENTRY_SPSR]

	@ Step 2
	@ compare "modeSaveFlag" to decide which mode will be saved
	@ the private registers are saved in an array. the consequence should
	@ be "spsr", "r8-r12", sp, lr
	@ the data array stores registers from low address to high address.
	ldr	r6, [r5, #SleepState_modeSaveFlags]

1:
	@ save SYS mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_SYS)
	beq	2f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_SYS | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r5, #SleepState_SYS_REGS
	stmia	r7, {sp, lr}

2:
	@ save FIQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_FIQ)
	beq	3f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_FIQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs     r2, spsr
	add	r7, r5, #SleepState_FIQ_REGS
	stmia   r7, {r2, r8 - r12, sp, lr}

3:
	@ save IRQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_IRQ)
	beq	4f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_IRQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs	r2, spsr
	add	r7, r5, #SleepState_IRQ_REGS
	stmia	r7, {r2, sp, lr}

4:
	@ save ABT mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_ABT)
	beq	5f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_ABT | CPSR_I_Bit | CPSR_F_Bit)
	msr     cpsr_c, r3
	mrs     r2, spsr
	add	r7, r5, #SleepState_ABT_REGS
	stmia	r7, {r2, sp, lr}

5:
	@ save UND mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_UND)
	beq	6f
	bic	r3, r3, #(CPSR_Mode_MASK)
    	orr	r3, r3, #(CPSR_Mode_UND | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs     r2, spsr
	add	r7, r5, #SleepState_UND_REGS
	stmia	r7, {r2, sp, lr}

6:
	@ save SVC mode content?
	ands    r1, r6, #(PM_MODE_SAVE_FLAG_SVC)
	beq	7f
	bic     r3, r3, #(CPSR_Mode_MASK)
        orr     r3, r3, #(CPSR_Mode_SVC | CPSR_I_Bit | CPSR_F_Bit)
        msr     cpsr_c, r3
	mrs     r2, spsr
	add	r7, r5, #SleepState_SVC_REGS
	stmia	r7, {r2, sp, lr}

7:
	@ Step 3
	@ save MMU settings
	@ r5 is pointer to sleep save data area

	@ Cp15_ACR_MMU
	mrc	p15, 0, r0, c1, c0, 0
	str	r0, [r5, #SleepState_Cp15_ACR_MMU]

	@ Cp15_AUXCR_MMU;
	mrc	p15, 0, r0, c1, c0, 1
	str	r0, [r5, #SleepState_Cp15_AUXCR_MMU]

	@ Cp15_TTBR_MMU;
	mrc	p15, 0, r0, c2, c0, 0
	str	r0, [r5, #SleepState_Cp15_TTBR_MMU]

        @ Cp15_DACR_MMU;
	mrc	p15, 0, r0, c3, c0, 0
	str	r0, [r5, #SleepState_Cp15_DACR_MMU]

        @ Cp15_PID_MMU;
	mrc	p15, 0, r0, c13, c0, 0
	str	r0, [r5, #SleepState_Cp15_PID_MMU]

	@ Cp15_CPAR;
	mrc	p15, 0, r0, c15, c1, 0
	str	r0, [r5, #SleepState_Cp15_CPAR]

	@ Now enable access to all valid coprocessors
	mcr	p15, 0, r1, c15, c1, 0

	@ cp_wait
	mrc	p15, 0, r0, c2, c0, 0
	mov	r0, r0
	sub	pc, pc, #4

	@ Step 4
	@ The block 0 of nand flash should be copied to SRAM 0x5c014000
	@ The OS should save the resume back address and the content save area address
	@ load current pspr to r12 register
	ldr	r12, [r5, #SleepState_psprAddress]
	@ Store 0x5c014000 to PSPR
	ldr	r1, =0x5c014000
	str	r1, [r12]

	@ Step 5
	@ calculate checksum
	@ get total word count for ckecksum and should not include "checksum"
	mov	r1, #SleepState_size - 4
	ldr	r2, [r5, #SleepState_extendedChecksumByteCount]
	add	r1, r1, r2
	@ get the word count by /4
	mov	r1, r1, lsr #2
	mov	r0, r5
	str	r1, [r0, #SleepState_wordCount]!
	bl	pm_checksum_calculate
	str	r0, [r5, #SleepState_checksum]
	@ Step 6
	@ invoke user flush function
	ldr	r0, [r5, #SleepState_flushFunc]
	cmp	r0, #0
	movne	lr, pc
	movne	pc, r0

       	b	1f
	.align	5
1:
     	@ Step 7
	@ enter sleep or deep sleep
	mcr     p14, 0, r11, c7, c0, 0

	@ wait for sleep
20:
	nop
        b       20b


@*****************************************************************************
@ pm_enter_sleep
@
@ Put the system into S2D3C4 state
@
@ Inputs:
@	r0: the virutal address of the data area to save the content of core
@
@ Outputs:
@	None
@

pm_enter_sleep:
	str     r1, [r0, #SleepState_ENTRY_R1]
	mov	r1, #MHN_PM_S2D3C4
	b       pm_enter_sleep_or_deep_sleep

@*****************************************************************************
@ pm_enter_deep_sleep
@
@ Put the system into S3D4C4 state
@
@ Inputs:
@	r0: the virutal address of the data area to save the content of core
@
@ Outputs:
@	None
@

pm_enter_deep_sleep:
	str     r1, [r0, #SleepState_ENTRY_R1]
	mov	r1, #MHN_PM_S3D4C4
	b       pm_enter_sleep_or_deep_sleep

