/*
 * linux/sound/pxa2xx-ac97.c -- AC97 support for the Intel PXA2xx chip.
 *
 * Author:	Nicolas Pitre
 * Created:	Dec 02, 2004
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>

#include <asm/irq.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mhn_gpio.h>
#include <asm/arch/mfp.h>

static DECLARE_MUTEX(car_mutex);
static DECLARE_WAIT_QUEUE_HEAD(gsr_wq);
static volatile long gsr_bits;

/*
 * delay 50ms as suggested in Errata, E54
 */
#define AC97_DELAY (HZ/20)

unsigned short pxa2xx_ac97_read(int num, unsigned short reg)
{
	unsigned short val = -1;
	volatile u32 *reg_addr;

	down(&car_mutex);

	/*
	 * Intel PXA27x Processor Family Specification Update February 2005
	 * E54: suggest not to use CAR_CAIP, but SDONE/CDONE instead
	 */

	/* set up primary or secondary codec space */
	reg_addr = (num & 1) ? &SAC_REG_BASE : &PAC_REG_BASE;
	reg_addr += (reg >> 1);

	/* start read access across the ac97 link */
	gsr_bits = 0;
	val = *reg_addr;
	wait_event_timeout(gsr_wq, gsr_bits & GSR_SDONE, AC97_DELAY);
	if (!(gsr_bits & GSR_SDONE)) {
		printk(KERN_ERR "%s: read error (ac97_reg=%d GSR=%#lx)\n",
				__FUNCTION__, reg, gsr_bits);
		val = -1;
		goto out;
	}

	/* valid data now */
	gsr_bits = 0;
	val = *reg_addr;			
	/* but we've just started another cycle... */
	wait_event_timeout(gsr_wq, gsr_bits & GSR_SDONE, AC97_DELAY);
	if (!(gsr_bits & GSR_SDONE)) {
		printk(KERN_ERR "%s: read error (ac97_reg=%d GSR=%#lx)\n",
				__FUNCTION__, reg, gsr_bits);
		val = -1;
		goto out;
	}

out:	up(&car_mutex);
	return val;
}

void pxa2xx_ac97_write(int num, unsigned short reg, unsigned short val)
{
	volatile u32 *reg_addr;

	down(&car_mutex);

	/* set up primary or secondary codec space */
	reg_addr = (num & 1) ? &SAC_REG_BASE : &PAC_REG_BASE;
	reg_addr += (reg >> 1);
	gsr_bits = 0;
	*reg_addr = val;
	wait_event_timeout(gsr_wq, gsr_bits & GSR_CDONE, AC97_DELAY);
	/*
	 * If reg == AC97_GPIO_STATUS, then CDONE is not issued
	 */
	if ((reg != AC97_GPIO_STATUS) && !(gsr_bits & GSR_CDONE))
		printk(KERN_ERR "%s: write error (ac97_reg=%d GSR=%#lx)\n",
				__FUNCTION__, reg, gsr_bits);

	up(&car_mutex);
}

#ifndef CONFIG_PXA3xx
void pxa2xx_ac97_reset(void)
{
	/* First, try cold reset */
	GCR &=  GCR_COLD_RST;  /* clear everything but nCRST */
	GCR &= ~GCR_COLD_RST;  /* then assert nCRST */

	gsr_bits = 0;
#ifdef CONFIG_PXA27x
	/* PXA27x Developers Manual section 13.5.2.2.1 */
	pxa_set_cken(1 << 31, 1);
	udelay(5);
	pxa_set_cken(1 << 31, 0);
	GCR = GCR_COLD_RST;
	udelay(50);
#else
	GCR = GCR_COLD_RST;
	GCR |= GCR_PRIRDY_IEN|GCR_SECRDY_IEN;
	wait_event_timeout(gsr_wq, gsr_bits & (GSR_PCR | GSR_SCR), 1);
#endif

	if (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR))) {
		printk(KERN_INFO "%s: cold reset timeout (GSR=%#lx)\n",
				 __FUNCTION__, gsr_bits);

		/* let's try warm reset */
		gsr_bits = 0;
#ifdef CONFIG_PXA27x
		/* warm reset broken on Bulverde,
		   so manually keep AC97 reset high */
		pxa_gpio_mode(113 | GPIO_OUT | GPIO_DFLT_HIGH); 
		udelay(10);
		GCR |= GCR_WARM_RST;
		pxa_gpio_mode(113 | GPIO_ALT_FN_2_OUT);
		udelay(50);
#else
		GCR |= GCR_WARM_RST|GCR_PRIRDY_IEN|GCR_SECRDY_IEN;;
		wait_event_timeout(gsr_wq, gsr_bits & (GSR_PCR | GSR_SCR), 1);
#endif			

		if (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR)))
			printk(KERN_INFO "%s: warm reset timeout (GSR=%#lx)\n",
					 __FUNCTION__, gsr_bits);
	}

	GCR &= ~(GCR_PRIRDY_IEN|GCR_SECRDY_IEN);
	GCR |= GCR_SDONE_IE|GCR_CDONE_IE;
}
#else

static struct mhn_pin_config zylonite_ac97_pins_low[] = {
	MHN_MFP_CFG("AC97 SDOUT",  MFP_PIN_GPIO27, MFP_PIN_GPIO27_AF_GPIO_27,
			MFP_DS03X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
	MHN_MFP_CFG("AC97 SYNC",   MFP_PIN_GPIO28, MFP_PIN_GPIO28_AF_GPIO_28,
			MFP_DS03X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
};

static struct mhn_pin_config zylonite_ac97_pins_ac97[] = {
	MHN_MFP_CFG("AC97 SDOUT",  MFP_AC97_SDATA_OUT, MFP_AC97_SDATA_OUT_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
	MHN_MFP_CFG("AC97 SYNC",   MFP_AC97_SYNC, MFP_AC97_SYNC_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
};

static void pins_low(void)
{
	mhn_gpio_set_direction(MFP_PIN_GPIO27, GPIO_DIR_OUT);
	mhn_gpio_set_level(MFP_PIN_GPIO27, GPIO_LEVEL_LOW);

	mhn_gpio_set_direction(MFP_PIN_GPIO28, GPIO_DIR_OUT);
	mhn_gpio_set_level(MFP_PIN_GPIO28, GPIO_LEVEL_LOW);

	mhn_mfp_set_configs(zylonite_ac97_pins_low, ARRAY_SIZE(zylonite_ac97_pins_low));

}

static void pins_ac97(void)
{
	mhn_mfp_set_configs(zylonite_ac97_pins_ac97, ARRAY_SIZE(zylonite_ac97_pins_ac97));
}

void pxa2xx_ac97_reset(void)
{
	gsr_bits = 0;
	GSR = ~0;             /* Clear status bits */

	GCR &= GCR_COLD_RST;  /* Clear everything but nCRST */
	/* Turn on internal clock - see 6.4.9.2.2; enable interrupts on reset done */
	GCR |= GCR_CLKBPB | GCR_PRIRDY_IEN | GCR_SECRDY_IEN;
	pins_low();           /* Drive SYNC and OUT pins low; see WM9713L datasheet */
	GCR &= ~GCR_COLD_RST; /* Assert nCRST */
	udelay(100);          /* Hold reset active for a minimum time */
	GCR |= GCR_COLD_RST;
	pins_ac97();
	udelay(10);           /* For some reason, without this the sequence does not work */
	GCR &= ~GCR_CLKBPB;

	wait_event_timeout(gsr_wq, (GSR | gsr_bits) & (GSR_PCR | GSR_SCR), 10);

	GCR &= ~(GCR_PRIRDY_IEN|GCR_SECRDY_IEN);
	GCR |= GCR_SDONE_IE|GCR_CDONE_IE;
}

#endif

static irqreturn_t pxa2xx_ac97_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	long status;

	status = GSR;
	if (status) {
		GSR = status;
		gsr_bits |= status;
		wake_up(&gsr_wq);

		/* Although we don't use those we still need to clear them
		   since they tend to spuriously trigger when MMC is used
		   (hardware bug? go figure)... */
		MISR = MISR_EOC;
		PISR = PISR_EOC;
		MCSR = MCSR_EOC;

		/*
		 * GSR_PCR and GSR_SCR monitors bits received from ac97 link
		 * So it is not enough to clear it -- at the next ac97 period
		 * it will be set again
		 */
		if (status & GSR_PCR)
			GCR &= ~GCR_PRIRDY_IEN;

		if (status & GSR_SCR)
			GCR &= ~GCR_SECRDY_IEN;

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static inline void pxa2xx_ac97_do_suspend(void) {
	GCR |= GCR_ACLINK_OFF;
#ifndef CONFIG_PXA3xx
	pxa_set_cken(CKEN2_AC97, 0);
#else
	pxa_set_cken(CKEN_AC97, 0);
#endif
}

static inline void pxa2xx_ac97_do_resume(void) {
#ifndef CONFIG_PXA3xx
	pxa_set_cken(CKEN2_AC97, 1);
#else
	pxa_set_cken(CKEN_AC97, 1);
#endif
}

#ifdef CONFIG_PM

void pxa2xx_ac97_suspend(void) {
	pxa2xx_ac97_do_suspend();
}

void pxa2xx_ac97_resume(void) {
	pxa2xx_ac97_do_resume();
}

EXPORT_SYMBOL(pxa2xx_ac97_suspend);
EXPORT_SYMBOL(pxa2xx_ac97_resume);

#endif

#ifndef CONFIG_PXA3xx

static void set_pins(void)
{
	pxa_gpio_mode(GPIO31_SYNC_AC97_MD);
	pxa_gpio_mode(GPIO30_SDATA_OUT_AC97_MD);
	pxa_gpio_mode(GPIO28_BITCLK_AC97_MD);
	pxa_gpio_mode(GPIO29_SDATA_IN_AC97_MD);
#ifdef CONFIG_PXA27x
	/* Use GPIO 113 as AC97 Reset on Bulverde */
	pxa_gpio_mode(113 | GPIO_ALT_FN_2_OUT);
#endif
}

#else

extern void zylonite_enable_ac97_pins(void);
static void set_pins(void)
{
	unsigned short val;
	zylonite_enable_ac97_pins();
	pxa2xx_ac97_reset();
	val = pxa2xx_ac97_read(0, AC97_RESET);
	if (val == (unsigned short)-1 || val == 0) {
		/*
		 * there is a bug on MonahansL/MonhansPL PC card: AC97_SDATA_IN is not connected to CODEC
		 * ECO 72: Connect PWM_0(MFP_RSVD_AC97_SDATA_IN_0) to CODEC as AC97_SDATA_IN
		 */
		printk(KERN_INFO "couldn't read ac97 chip id; trying other path\n");
		mhn_mfp_set_afds(MFP_RSVD_AC97_SDATA_IN_0, MFP_RSVD_AC97_SDATA_IN_0_AF, MFP_DS03X);
		mhn_mfp_set_afds(MFP_AC97_SDATA_IN_0, MFP_AF0, MFP_DS01X);
		pxa2xx_ac97_reset();
	}
}

#endif

int pxa2xx_ac97_init(void)
{
	int ret;

	ret = request_irq(IRQ_AC97, pxa2xx_ac97_irq, 0, "AC97", NULL);
	if (ret < 0)
		return ret;

	pxa2xx_ac97_do_resume();

	set_pins();

	return 0;
}

void pxa2xx_ac97_exit(void)
{
	pxa2xx_ac97_do_suspend();
	free_irq(IRQ_AC97, NULL);
}

EXPORT_SYMBOL(pxa2xx_ac97_read);
EXPORT_SYMBOL(pxa2xx_ac97_write);
EXPORT_SYMBOL(pxa2xx_ac97_reset);

MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("AC97 driver for the Intel PXA2xx chip");
MODULE_LICENSE("GPL");
