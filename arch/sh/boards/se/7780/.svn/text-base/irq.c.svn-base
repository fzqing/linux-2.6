/*
 * linux/arch/sh/boards/se/7780/irq.c
 *
 * Copyright (C) 2005  Takashi Kusuda (Feb.4 2005)
 *
 * Hitachi SH7780 SolutionEngine Support.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

/*
 * interrupt demuxer
 */
int sh7780se_irq_demux(int irq)
{
	return irq;
}

/*
 * Initialize IRQ setting
 */
void __init init_sh7780se_IRQ(void)
{
    /* setup SH7780 INTC (board depends). INTC2 initialized at "init_IRQ()" */
    ctrl_outl(0x00C00020, INTC_ICR0);       /* ICR0: IRL=use separately */

#if defined(CONFIG_SH7780_1ST_CUT)
    ctrl_outl(0xFFFF0000, INTC_ICR1);	/* ICR1: detect low level(for 1stcut) */
#else
    ctrl_outl(0xAAAA0000, INTC_ICR1);	/* ICR1: detect low level(for 2ndcut) */
#endif
    ctrl_outl(0x33333333, INTC_INTPRI);	/* INTPRI: priority=3(all) */

    /* interrupt controlled at CPU's INTC(INTC2) */
    ctrl_outw(0, FPGA_INTMSK1); /* enable all interrupt at FPGA */
    ctrl_outw((ctrl_inw(FPGA_INTMSK1) | 0x0002), FPGA_INTMSK1); /* mask SM501 interrupt */
    ctrl_outw(0, FPGA_INTMSK2); /* enable all interrupt at FPGA */

    /*
     * IRQ0:  daughter board EXTINT1
     * IRQ1:  daughter board EXTINT2
     * IRQ2:  daughter board EXTINT3
     * IRQ3:  LAN (SMC91C111)
     * IRQ4:  daughter board EXTINT4
     * IRQ5:  PC Card 0
     * IRQ6:  PC Card 2
     * IRQ7:  SM501/PC Card PowerIC
     */

    /* set FPGA INTSEL register */
    ctrl_outw(((IRQPIN_PCC2 << IRQPOS_PCC2) |
	       (IRQPIN_PCC0 << IRQPOS_PCC0) |
	       (IRQPIN_SM501 << IRQPOS_SM501) |
	       (IRQPIN_SMC91CX << IRQPOS_SMC91CX)), FPGA_INTSEL1);

    ctrl_outw(((IRQPIN_EXTINT4 << IRQPOS_EXTINT4) |
	       (IRQPIN_EXTINT3 << IRQPOS_EXTINT3) |
	       (IRQPIN_EXTINT2 << IRQPOS_EXTINT2) |
	       (IRQPIN_EXTINT1 << IRQPOS_EXTINT1)), FPGA_INTSEL2);

    ctrl_outw((IRQPIN_PCCPW << IRQPOS_PCCPW), FPGA_INTSEL3);

    /* register IRQ interrupts */
    make_sh4a_intc_irq(SH7780_IRQ0_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ0_POS);
    make_sh4a_intc_irq(SH7780_IRQ1_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ1_POS);
    make_sh4a_intc_irq(SH7780_IRQ2_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ2_POS);
    make_sh4a_intc_irq(SH7780_IRQ3_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ3_POS);
    make_sh4a_intc_irq(SH7780_IRQ4_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ4_POS);
    make_sh4a_intc_irq(SH7780_IRQ5_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ5_POS);
    make_sh4a_intc_irq(SH7780_IRQ6_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ6_POS);
    make_sh4a_intc_irq(SH7780_IRQ7_INTNUM, INTC_INTMSK0, INTC_INTMSKCLR0, SH7780_IRQ7_POS);
}

