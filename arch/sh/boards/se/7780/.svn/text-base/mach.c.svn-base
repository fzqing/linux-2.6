/*
 * linux/arch/sh/boards/se/7780/mach.c
 *
 * Copyright (C)  Takashi Kusuda (Nov, 2004)
 *
 * Machine vector for the Hitachi 7780 SolutionEngine
 */

#include <linux/config.h>
#include <linux/init.h>

#include <asm/machvec.h>
#include <asm/rtc.h>
#include <asm/machvec_init.h>

#include <asm/se7780/io.h>
#include <asm/irq.h>

void heartbeat_7780se(void);
void init_sh7780se_IRQ(void);
int sh7780se_irq_demux(int);
void *sh7780se_ioremap(unsigned long, unsigned long);
void sh7780se_iounmap(void *);

/*
 * The Machine Vector
 */

struct sh_machine_vector mv_7780se __initmv = {
	.mv_nr_irqs		= NR_IRQS,

	.mv_inb			= sh7780se_inb,
	.mv_inw			= sh7780se_inw,
	.mv_inl			= sh7780se_inl,
	.mv_outb		= sh7780se_outb,
	.mv_outw		= sh7780se_outw,
	.mv_outl		= sh7780se_outl,

	.mv_inb_p		= sh7780se_inb_p,
	.mv_inw_p		= sh7780se_inw,
	.mv_inl_p		= sh7780se_inl,
	.mv_outb_p		= sh7780se_outb_p,
	.mv_outw_p		= sh7780se_outw,
	.mv_outl_p		= sh7780se_outl,

	.mv_insb		= sh7780se_insb,
	.mv_insw		= sh7780se_insw,
	.mv_insl		= sh7780se_insl,
	.mv_outsb		= sh7780se_outsb,
	.mv_outsw		= sh7780se_outsw,
	.mv_outsl		= sh7780se_outsl,

	.mv_ioremap		= sh7780se_ioremap,
	.mv_iounmap		= sh7780se_iounmap,

	.mv_irq_demux		= sh7780se_irq_demux,
	.mv_init_irq		= init_sh7780se_IRQ,

#ifdef CONFIG_HEARTBEAT
	.mv_heartbeat		= heartbeat_7780se,
#endif
};
ALIAS_MV(7780se)
