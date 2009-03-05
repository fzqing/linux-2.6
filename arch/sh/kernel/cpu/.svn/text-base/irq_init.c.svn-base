/*
 * linux/arch/sh/kernel/cpu/irq_init.c
 *
 * Copyright (C) 2004 Takashi Kusuda
 *
 * Initialize SuperH internal IRQs.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/machvec.h>


extern void init_sh4a_intc2_IRQ(void);
extern void init_ipr_IRQ(void);
extern void init_intmsk_IRQ(void);
extern void init_IRQ_intc2(void);

void __init init_IRQ(void)
{

#if defined(CONFIG_CPU_SH4A)
	/*
	 *  SH4A INTC consist of INTC and INTC2 modules.
	 *  Peripheral module interrupt use INTC2.
	 */
	init_sh4a_intc2_IRQ();
#else
/*
 * Pins(IRQ4/IRQ5 and PINT0-15 are multiplexed. These initializing
 * (make_xxx_irq()) are done at platform depend IRQ setup section
 * (at sh_mv.mv_init_irq()). So below init_ipr_IRQ and init_pint_IRQ
 * don't initialize IRQ4/IRQ5 and PINT0-15.
 *
 * If you need these IRQ initialiation, add make_xxx_irq() at
 * sh_mv.mv_init_irq() function.
 */

	/* Initialize IPR-IRQ */
	init_ipr_IRQ();

#if defined(SH_INTMSK_INIT_TABLE)
	/* Initialize INTMSK-IRQ(SH7760) */
	init_intmsk_IRQ();
#endif /* SH_INTMSK_INIT_TABLE */

#ifdef CONFIG_CPU_SUBTYPE_ST40
	init_IRQ_intc2();
#endif

#endif

	/* Perform the machine specific initialisation */
	if (sh_mv.mv_init_irq != NULL) {
		sh_mv.mv_init_irq();
	}
}
