/*
 * linux/arch/arm/mach-pxa/pxa3xx.c
 *
 * Porting to PXA3xx based on PXA27x.c
 * Copyright (C) 2006 Marvell International Ltd .
 *
 * Code specific to PXA3xx.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/cpu-freq-voltage-mhn.h>
#include "generic.h"

/* Crystal clock: 13MHz */
#define BASE_CLK	13000000

/* ACSR field */
#define ACSR_SMC_MASK	0x03800000	/* Static Memory Controller Frequency Select */
#define ACSR_SRAM_MASK	0x000c0000	/* SRAM Controller Frequency Select */
#define ACSR_FC_MASK	0x00030000	/* Frequency Change Frequency Select */
#define ACSR_HSIO_MASK	0x0000c000	/* High Speed IO Frequency Select */
#define ACSR_DDR_MASK	0x00003000	/* DDR Memory Controller Frequency Select */
#define ACSR_XN_MASK	0x00000700	/* Run Mode Frequency to Turbo Mode Frequency Multiplier */
#define ACSR_XL_MASK	0x0000001f	/* Crystal Frequency to Memory Frequency Multiplier */
#define ACSR_XPDIS	(1 << 31)
#define ACSR_SPDIS	(1 << 30)
#define ACSR_13MEND1	(1 << 27)
#define ACSR_D0CS	(1 << 26)
#define ACSR_13MEND2	(1 << 21)
/*
 * Get the clock frequency as reflected by CCSR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 */
unsigned int get_clk_frequency_khz(int info)
{
	unsigned long acsr_val;
	int XL, XN, HSS;
	int clk, s_clk = 0, d0cs = 0;

	acsr_val = ACSR;
	XL = acsr_val & ACSR_XL_MASK;
	XN = (acsr_val & ACSR_XN_MASK) >> 8;

	clk = XL * XN * BASE_CLK;

	if (acsr_val & ACSR_D0CS) {
		d0cs = 1;
		clk = 60000000;
	}

	if (info) {
		if (d0cs) {
			s_clk = 60000000;
			printk(KERN_INFO "Run Mode Clock: %dMHz\n",
					(clk / 1000000));
			printk(KERN_INFO "High Speed I/O Bus Clock: %dMHz\n",
					(s_clk / 1000000));
		} else {
			HSS = (acsr_val & ACSR_HSIO_MASK) >> 14;
			switch (HSS) {
			case 0:
				s_clk = 104000000;
				break;
			case 1:
				s_clk = 156000000;
				break;
			case 2:
				s_clk = 208000000;
				break;
			default:
				break;
			}
			printk(KERN_INFO "Run Mode Clock: %dMHz\n",
				(XL * BASE_CLK / 1000000));
			if (XN > 1)
				printk(KERN_INFO "Turbo Mode Clock: %dMHz\n",
					(XL * XN * BASE_CLK / 1000000));
			printk(KERN_INFO "High Speed I/O Bus Clock: %dMHz\n",
					(s_clk / 1000000));
		}
	}

	return (clk / 1000);
}

/*
 * Return the current mem clock frequency in units of 10kHz
 */
unsigned int get_memclk_frequency_10khz(void)
{
	unsigned long acsr_val;
	int ddr_clk = 0;

	acsr_val = ACSR;
	if (acsr_val & ACSR_D0CS) {
		/* Ring Oscillator mode */
		ddr_clk = 30;
	} else {
		switch ((acsr_val & ACSR_DDR_MASK) >> 12) {
		case 0:
			ddr_clk = 26;
			break;
		case 3:
			ddr_clk = 260;
			break;
		default:
			break;
		}
	}
	return (ddr_clk * 100);
}

/*
 * Return the current LCD clock frequency in units of 10kHz as
 * LCLK is from High Speed IO Bus Clock
 */
unsigned int get_lcdclk_frequency_10khz(void)
{
	unsigned long acsr_val;
	int s_clk=0, HSS;

	acsr_val = ACSR;
	if (acsr_val & ACSR_D0CS) {
		/* Ring Oscillator mode */
		s_clk = 60;
	} else {
		HSS = (acsr_val & ACSR_HSIO_MASK) >> 14;
		switch (HSS) {
		case 0:
			s_clk = 104;
			break;
		case 1:
			s_clk = 156;
			break;
		case 2:
			s_clk = 208;
			break;
		default:
			break;
		}
	}
	return (s_clk * 100);
}

static int oscc_pout_count = 0;
void enable_oscc_pout(void)
{
	unsigned long val;

	if (!oscc_pout_count++) {
		val = __raw_readl((void*)&OSCC);
		val |= OSCC_PEN;
		__raw_writel(val, (void*)&OSCC);
	}
	return;
}

void disable_oscc_pout(void)
{
	unsigned long val;
	if (!--oscc_pout_count) {
		val = __raw_readl((void*)&OSCC);
		val &= ~OSCC_PEN;
		__raw_writel(val, (void*)&OSCC);
	}
	return;
}

EXPORT_SYMBOL_GPL(get_clk_frequency_khz);
EXPORT_SYMBOL_GPL(get_memclk_frequency_10khz);
EXPORT_SYMBOL_GPL(get_lcdclk_frequency_10khz);
EXPORT_SYMBOL(enable_oscc_pout);
EXPORT_SYMBOL(disable_oscc_pout);
