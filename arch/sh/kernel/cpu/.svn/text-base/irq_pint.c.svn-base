/*
 * linux/arch/sh/kernel/cpu/irq_pint.c
 *
 * Copyright (C) 2004 Takashi Kusuda
 *
 * Interrupt handling for PINT IRQ.
 * move PINT interrupt hander from irq_ipr.c, and add SH7727 pint code.
 *
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/machvec.h>

static unsigned char pint_map[256];
static unsigned long portcr_mask = 0;

static void enable_pint_irq(unsigned int irq);
static void disable_pint_irq(unsigned int irq);

/* shutdown is same as "disable" */
#define shutdown_pint_irq disable_pint_irq

static void mask_and_ack_pint(unsigned int);
static void end_pint_irq(unsigned int irq);

static unsigned int startup_pint_irq(unsigned int irq)
{
	enable_pint_irq(irq);
	return 0; /* never anything pending */
}

static struct hw_interrupt_type pint_irq_type = {
	"PINT-IRQ",
	startup_pint_irq,
	shutdown_pint_irq,
	enable_pint_irq,
	disable_pint_irq,
	mask_and_ack_pint,
	end_pint_irq
};

static void disable_pint_irq(unsigned int irq)
{
	unsigned long val, flags;

	local_irq_save(flags);
	val = ctrl_inw(INTC_INTER);
	val &= ~(1 << (irq - PINT_IRQ_BASE));
	ctrl_outw(val, INTC_INTER);	/* disable PINTn */
	portcr_mask &= ~(3 << (irq - PINT_IRQ_BASE)*2);
	local_irq_restore(flags);
}

static void enable_pint_irq(unsigned int irq)
{
	unsigned long val, flags;

	local_irq_save(flags);
	val = ctrl_inw(INTC_INTER);
	val |= 1 << (irq - PINT_IRQ_BASE);
	ctrl_outw(val, INTC_INTER);	/* enable PINTn */
	portcr_mask |= 3 << (irq - PINT_IRQ_BASE)*2;
	local_irq_restore(flags);
}

static void mask_and_ack_pint(unsigned int irq)
{
	disable_pint_irq(irq);
}

static void end_pint_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		enable_pint_irq(irq);
}

void make_pint_irq(unsigned int irq)
{
	disable_irq_nosync(irq);
	irq_desc[irq].handler = &pint_irq_type;
	disable_pint_irq(irq);
}

void __init init_pint_IRQ(void)
{
	int i;

	for(i = 0; i < 16; i++)
		make_pint_irq(PINT_IRQ_BASE + i);
	for(i = 0; i < 256; i++)
	{
		if(i & 1) pint_map[i] = 0;
		else if(i & 2) pint_map[i] = 1;
		else if(i & 4) pint_map[i] = 2;
		else if(i & 8) pint_map[i] = 3;
		else if(i & 0x10) pint_map[i] = 4;
		else if(i & 0x20) pint_map[i] = 5;
		else if(i & 0x40) pint_map[i] = 6;
		else if(i & 0x80) pint_map[i] = 7;
	}
}

int pint_irq_demux(int irq)
{
#if defined(CONFIG_CPU_SUBTYPE_SH7727)
	unsigned long d, sav1, sav2;
#else
	unsigned long d, creg, dreg;
#endif

	if(irq == PINT0_IRQ) {
#if defined(CONFIG_CPU_SUBTYPE_SH7727)
		sav1 = ctrl_inw(PORT_PCCR); /* PINT3:0 */
		sav2 = ctrl_inw(PORT_PMCR); /* PINT7:4 */
		ctrl_outw(sav1 | ((portcr_mask&0x000000ff)<<8), PORT_PCCR); /* PINT3:0 */
		ctrl_outw(sav2 | (portcr_mask&0x0000ff00), PORT_PMCR); /* PINT7:4 */

		d = ((~((ctrl_inb(PORT_PCDR)>>4) || (ctrl_inb(PORT_PMDR)&0xf0))) ^ ctrl_inw(INTC_ICR2)) & ctrl_inw(INTC_INTER) & 0xff;

		ctrl_outw(sav1, PORT_PCCR);
		ctrl_outw(sav2, PORT_PMCR);
#else
#if defined(CONFIG_CPU_SUBTYPE_SH7705) || defined(CONFIG_CPU_SUBTYPE_SH7707)
		creg = PORT_PACR;
		dreg = PORT_PADR;
#else
		creg = PORT_PCCR;
		dreg = PORT_PCDR;
#endif
		sav1 = ctrl_inw(creg);
		ctrl_outw(sav1 | portcr_mask, creg);
		d = (~ctrl_inb(dreg) ^ ctrl_inw(INTC_ICR2)) & ctrl_inw(INTC_INTER) & 0xff;
		ctrl_outw(sav1, creg);
#endif
		if(d == 0) return irq;
		return PINT_IRQ_BASE + pint_map[d];
        } else if(irq == PINT8_IRQ) {
#if defined(CONFIG_CPU_SUBTYPE_SH7727)
		sav1 = ctrl_inw(PORT_PMCR); /* PINT10:8 */
		sav2 = ctrl_inw(PORT_PFCR); /* PINT15:11 */
		ctrl_outw(sav1 | ((portcr_mask&0x003f0000)>>14), PORT_PMCR); /* PINT10:8 */
		ctrl_outw(sav2 | ((portcr_mask&0xffc00000)>>16), PORT_PFCR); /* PINT15:11 */

		d = ((~(((ctrl_inb(PORT_PMDR)&0x0e)>>1) || (ctrl_inb(PORT_PFDR)&0xf8))) ^ (ctrl_inw(INTC_ICR2)>>8)) & (ctrl_inw(INTC_INTER)>>8) & 0xff;

		ctrl_outw(sav1, PORT_PMCR);
		ctrl_outw(sav2, PORT_PFCR);
#else
#if defined(CONFIG_CPU_SUBTYPE_SH7705) || defined(CONFIG_CPU_SUBTYPE_SH7707)
		creg = PORT_PBCR;
		dreg = PORT_PBDR;
#else
		creg = PORT_PFCR;
		dreg = PORT_PFDR;
#endif
		sav1 = ctrl_inw(creg);
		ctrl_outw(sav1 | (portcr_mask >> 16), creg);
		d = (~ctrl_inb(dreg) ^ (ctrl_inw(INTC_ICR2) >> 8)) & (ctrl_inw(INTC_INTER) >> 8) & 0xff;
		ctrl_outw(sav1, creg);
#endif
		if(d == 0) return irq;
		return PINT_IRQ_BASE + 8 + pint_map[d];
	}
	return irq;
}

