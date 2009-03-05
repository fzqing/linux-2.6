/*
 * arch/arm/mach-pxa/mhn_gpio.c
 *
 * Copyright (C) 2006,  Marvell International Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>

#ifdef CONFIG_GPIO_DEBUG
#define GPIO_ID_VERIFY(gpio)						\
	do {								\
		if ((gpio) > GPIO_ID_MAX) {				\
			printk(KERN_WARNING "%s: exceeds GPIO range\n",	\
				__func__);				\
			return -EINVAL;					\
		}							\
	} while (0)
#else
#define GPIO_ID_VERIFY(gpio)	do {} while (0)
#endif

static struct {
	uint32_t gplr0;
	uint32_t gplr1;
	uint32_t gplr2;
	uint32_t gplr3;
	uint32_t gpdr0;
	uint32_t gpdr1;
	uint32_t gpdr2;
	uint32_t gpdr3;
	uint32_t grer0;
	uint32_t grer1;
	uint32_t grer2;
	uint32_t grer3;
	uint32_t gfer0;
	uint32_t gfer1;
	uint32_t gfer2;
	uint32_t gfer3;
} gpio_saved_reg;

static DEFINE_SPINLOCK(gpio_spin_lock);

int mhn_gpio_set_direction(int gpio_id, int dir)
{
	unsigned long flags;
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);

	spin_lock_irqsave(&gpio_spin_lock, flags);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START) {
		spin_unlock_irqrestore(&gpio_spin_lock, flags);
		return gpio_exp_set_direction(gpio, dir);
	}
#endif
	if (dir == GPIO_DIR_IN)
		GCDR(gpio) = 1u << (gpio & 0x1f);
	else
		GSDR(gpio) = 1u << (gpio & 0x1f);

	spin_unlock_irqrestore(&gpio_spin_lock, flags);

	return 0;
}

int mhn_gpio_get_direction(int gpio_id)
{
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return gpio_exp_get_direction(gpio);
#endif

	if (GPDR(gpio) & (1u << (gpio & 0x1f)))
		return GPIO_DIR_OUT;
	else
		return GPIO_DIR_IN;
}

int mhn_gpio_set_level(int gpio_id, int level)
{
	unsigned long flags;
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);

	spin_lock_irqsave(&gpio_spin_lock, flags);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START) {
		spin_unlock_irqrestore(&gpio_spin_lock, flags);
		return gpio_exp_set_level(gpio, level);
	}
#endif

	if (level == GPIO_LEVEL_LOW)
		GPCR(gpio) = 1u << (gpio & 0x1f);
	else
		GPSR(gpio) = 1u << (gpio & 0x1f);

	spin_unlock_irqrestore(&gpio_spin_lock, flags);

	return 0;
}

int mhn_gpio_get_level(int gpio_id)
{
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return gpio_exp_get_level(gpio);
#endif

	if (GPLR(gpio) & (1u << (gpio & 0x1f)))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}

/* GPIO Expander can be set as only rising or falling edge detect.
 * It will trigger interrupt when pin state is changed.
 */
int mhn_gpio_set_rising_edge_detect(int gpio_id, int enable)
{
	unsigned long flags;
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return 0;
#endif

	spin_lock_irqsave(&gpio_spin_lock, flags);

	if (enable == 0)
		GCRER(gpio) = 1u << (gpio & 0x1f);
	else
		GSRER(gpio) = 1u << (gpio & 0x1f);

	spin_unlock_irqrestore(&gpio_spin_lock, flags);

	return 0;
}

int mhn_gpio_get_rising_edge_detect(int gpio_id)
{
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return 0;
#endif

	if (GRER(gpio) & (1u << (gpio & 0x1f)))
		return 1;

	return 0;
}

int mhn_gpio_set_falling_edge_detect(int gpio_id, int enable)
{
	unsigned long flags;
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return 0;
#endif

	spin_lock_irqsave(&gpio_spin_lock, flags);

	if (enable == 0)
		GCRER(gpio) = 1u << (gpio & 0x1f);
	else
		GSRER(gpio) = 1u << (gpio & 0x1f);

	spin_unlock_irqrestore(&gpio_spin_lock, flags);

	return 0;
}

int mhn_gpio_get_falling_edge_detect(int gpio_id)
{
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return 0;
#endif

	if (GFER(gpio) & (1u << (gpio & 0x1f)))
		return 1;

	return 0;
}

int mhn_gpio_get_edge_detect_status(int gpio_id)
{
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return 0;
#endif

	if (GEDR(gpio) & (1u << (gpio & 0x1f)))
		return 1;

	return 0;
}

int mhn_gpio_clear_edge_detect_status(int gpio_id)
{
	unsigned long flags;
	int gpio = MFP2GPIO(gpio_id);

	GPIO_ID_VERIFY(gpio);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (gpio >= GPIO_EXP_START)
		return 0;
#endif

	spin_lock_irqsave(&gpio_spin_lock, flags);

	GEDR(gpio) = 1u << (gpio & 0x1f);

	spin_unlock_irqrestore(&gpio_spin_lock, flags);

	return 0;
}

/* The routine of save/restore gpio is used in PM.
 * The setting of GPIO will be restored.
 * The status of GPIO Edge Status will be lost.
 */
void mhn_gpio_save(void)
{
	gpio_saved_reg.gpdr0 = GPDR0;
	gpio_saved_reg.gpdr1 = GPDR1;
	gpio_saved_reg.gpdr2 = GPDR2;
	gpio_saved_reg.gpdr3 = GPDR3;

	gpio_saved_reg.gplr0 = GPLR0;
	gpio_saved_reg.gplr1 = GPLR1;
	gpio_saved_reg.gplr2 = GPLR2;
	gpio_saved_reg.gplr3 = GPLR3;

	gpio_saved_reg.grer0 = GRER0;
	gpio_saved_reg.grer1 = GRER1;
	gpio_saved_reg.grer2 = GRER2;
	gpio_saved_reg.grer3 = GRER3;

	gpio_saved_reg.gfer0 = GFER0;
	gpio_saved_reg.gfer1 = GFER1;
	gpio_saved_reg.gfer2 = GFER2;
	gpio_saved_reg.gfer3 = GFER3;
}

void mhn_gpio_restore(void)
{
	GPDR0 = gpio_saved_reg.gpdr0;
	GPDR1 = gpio_saved_reg.gpdr1;
	GPDR2 = gpio_saved_reg.gpdr2;
	GPDR3 = gpio_saved_reg.gpdr3;

	GPSR0 = gpio_saved_reg.gplr0;
	GPSR1 = gpio_saved_reg.gplr1;
	GPSR2 = gpio_saved_reg.gplr2;
	GPSR3 = gpio_saved_reg.gplr3;
	GPCR0 = ~(gpio_saved_reg.gplr0);
	GPCR1 = ~(gpio_saved_reg.gplr1);
	GPCR2 = ~(gpio_saved_reg.gplr2);
	GPCR3 = ~(gpio_saved_reg.gplr3);

	GRER0 = gpio_saved_reg.grer0;
	GRER1 = gpio_saved_reg.grer1;
	GRER2 = gpio_saved_reg.grer2;
	GRER3 = gpio_saved_reg.grer3;

	GFER0 = gpio_saved_reg.gfer0;
	GFER1 = gpio_saved_reg.gfer1;
	GFER2 = gpio_saved_reg.gfer2;
	GFER3 = gpio_saved_reg.gfer3;
}

EXPORT_SYMBOL_GPL(mhn_gpio_get_level);
EXPORT_SYMBOL_GPL(mhn_gpio_set_level);
EXPORT_SYMBOL_GPL(mhn_gpio_get_direction);
EXPORT_SYMBOL_GPL(mhn_gpio_set_direction);
EXPORT_SYMBOL_GPL(mhn_gpio_set_rising_edge_detect);
EXPORT_SYMBOL_GPL(mhn_gpio_get_rising_edge_detect);
EXPORT_SYMBOL_GPL(mhn_gpio_set_falling_edge_detect);
EXPORT_SYMBOL_GPL(mhn_gpio_get_falling_edge_detect);
EXPORT_SYMBOL_GPL(mhn_gpio_get_edge_detect_status);
EXPORT_SYMBOL_GPL(mhn_gpio_clear_edge_detect_status);
EXPORT_SYMBOL_GPL(mhn_gpio_save);
EXPORT_SYMBOL_GPL(mhn_gpio_restore);

