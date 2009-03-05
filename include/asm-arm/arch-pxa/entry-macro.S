/*
 * include/asm-arm/arch-pxa/entry-macro.S
 *
 * Low-level IRQ helper macros for PXA-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

		.macro	disable_fiq
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
		mrc	p6, 0, \irqstat, c0, c0, 0		@ ICIP
		mrc	p6, 0, \irqnr, c1, c0, 0		@ ICMR
#else
		mov	\base, #io_p2v(0x40000000)	@ IIR Ctl = 0x40d00000
		add	\base, \base, #0x00d00000
		ldr	\irqstat, [\base, #0]		@ ICIP
		ldr	\irqnr, [\base, #4]		@ ICMR
#endif
		ands	\irqnr, \irqstat, \irqnr
		beq	1001f
		rsb	\irqstat, \irqnr, #0
		and	\irqstat, \irqstat, \irqnr
		clz	\irqnr, \irqstat
		rsb	\irqnr, \irqnr, #(31 - PXA_IRQ_SKIP)
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
		b	1002f
#endif
1001:
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
		mrc	p6, 0, \irqstat, c6, c0, 0      @ ICIP2
		mrc	p6, 0, \irqnr, c7, c0, 0        @ ICMR2
		ands	\irqstat, \irqstat, \irqnr
		beq	1002f
		rsb	\irqnr, \irqstat, #0
		and	\irqstat, \irqstat, \irqnr
		clz	\irqnr, \irqstat
		rsb	\irqnr, \irqnr, #31
		add	\irqnr, \irqnr, #(32 - PXA_IRQ_SKIP)
1002:
#endif
		.endm
