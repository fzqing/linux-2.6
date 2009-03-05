/*
 *  arch/arm/mach-pxa/mfp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/types.h>
#include <asm/hardware.h>
#include <asm/arch/mfp.h>

#define CONFIG_MFP_DEBUG

#define MAX_MFP_PINS	419
#define PADBASE		0x00E10000
/* This io mem space follows the definition in include/asm/arch/hardware.h */
#undef KSEG0
#define KSEG0(a)	(((a) & 0x01ffffff) + 0xf6000000)

struct mfp_regs {
	unsigned char __iomem	*membase;
	unsigned int		mfp[MAX_MFP_PINS];
};

/* Writing to an MFP register is slow, a read-back of the register is
 * necessary for most functions here to make sure that the write is
 * finished. For configuration of multiple MFP pins, a read-back of the
 * last register is enough.
 */

/* mfp_spin_lock is used to ensure that MFP register configuration
 * (most likely a read-modify-write operation) is atomic.
 */
static spinlock_t mfp_spin_lock = SPIN_LOCK_UNLOCKED;

int mhn_mfp_set_config(struct mhn_pin_config *pin_config)
{
	mfp_pin_t mfp_pin;
	uint32_t  mfp_reg;

	_raw_spin_lock(&mfp_spin_lock);

	mfp_pin = pin_config->mfp_pin;
	mfp_reg = PIN2REG(pin_config);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(mfp_pin)){
		_raw_spin_unlock(&mfp_spin_lock);
		return 0;
	}
#endif

#ifdef CONFIG_MFP_DEBUG
	if ((pin_config == NULL) ||
	    (MFP_OFFSET(mfp_pin) > MHN_MAX_MFP_OFFSET) ||
	    (MFP_OFFSET(mfp_pin) < MHN_MIN_MFP_OFFSET)) {
		_raw_spin_unlock(&mfp_spin_lock);
		return -1;
	}
#endif

	MFP_REG(mfp_pin) = mfp_reg;
	mfp_reg = MFP_REG(mfp_pin);	/* read back */

	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

int mhn_mfp_set_configs(struct mhn_pin_config *pin_configs, int n)
{
	mfp_pin_t mfp_pin = 0;
	uint32_t  mfp_reg = 0;
	int i;

	if (n <= 0)
		return 0;

	_raw_spin_lock(&mfp_spin_lock);

	for (i = 0; i < n; i++) {
		mfp_pin = pin_configs->mfp_pin;
		mfp_reg = PIN2REG(pin_configs);

#if defined(CONFIG_PXA3xx_GPIOEX)
		if (IS_GPIO_EXP_PIN(mfp_pin)) {
			pin_configs++;
			continue;
		}
#endif

#ifdef CONFIG_MFP_DEBUG
		if ((pin_configs == NULL) ||
		    (MFP_OFFSET(mfp_pin) > MHN_MAX_MFP_OFFSET) ||
		    (MFP_OFFSET(mfp_pin) < MHN_MIN_MFP_OFFSET)) {
			_raw_spin_unlock(&mfp_spin_lock);
			return -1;
		}
#endif
		MFP_REG(mfp_pin) = mfp_reg;
		pin_configs++;
	}

	mfp_reg = MFP_REG(mfp_pin);
	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

int mhn_mfp_set_afds(mfp_pin_t pin, int af, int ds)
{
	uint32_t mfp_reg;

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

#ifdef CONFIG_MFP_DEBUG
	if ((MFP_OFFSET(pin) > MHN_MAX_MFP_OFFSET) ||
	    (MFP_OFFSET(pin) < MHN_MIN_MFP_OFFSET))
		return -1;
#endif

	_raw_spin_lock(&mfp_spin_lock);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~(MFP_AF_MASK | MFP_DRV_MASK);
	mfp_reg |= (((af & 0x7) << MFPR_ALT_OFFSET) |
		    ((ds & 0x7) << MFPR_DRV_OFFSET));
	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

int mhn_mfp_set_rdh(mfp_pin_t pin, int rdh)
{
	uint32_t mfp_reg;

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

#ifdef CONFIG_MFP_DEBUG
	if ((MFP_OFFSET(pin) > MHN_MAX_MFP_OFFSET) ||
	    (MFP_OFFSET(pin) < MHN_MIN_MFP_OFFSET))
		return -1;
#endif

	_raw_spin_lock(&mfp_spin_lock);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~MFP_RDH_MASK;

	if (likely(rdh))
		mfp_reg |= (1u << MFPR_SS_OFFSET);

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

int mhn_mfp_set_lpm(mfp_pin_t pin, int lpm)
{
	uint32_t mfp_reg;

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

#ifdef CONFIG_MFP_DEBUG
	if ((MFP_OFFSET(pin) > MHN_MAX_MFP_OFFSET) ||
	    (MFP_OFFSET(pin) < MHN_MIN_MFP_OFFSET))
		return -1;
#endif
	_raw_spin_lock(&mfp_spin_lock);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~(MFP_LPM_MASK);

	if (lpm & 0x1) mfp_reg |= 1u << MFPR_SON_OFFSET;
	if (lpm & 0x2) mfp_reg |= 1u << MFPR_SD_OFFSET;
	if (lpm & 0x4) mfp_reg |= 1u << MFPR_PU_OFFSET;
	if (lpm & 0x8) mfp_reg |= 1u << MFPR_PD_OFFSET;
	if (lpm &0x10) mfp_reg |= 1u << MFPR_PS_OFFSET;

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

int mhn_mfp_set_edge(mfp_pin_t pin, int edge)
{
	uint32_t mfp_reg;

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

#ifdef CONFIG_MFP_DEBUG
	if ((MFP_OFFSET(pin) > MHN_MAX_MFP_OFFSET) ||
	    (MFP_OFFSET(pin) < MHN_MIN_MFP_OFFSET))
		return -1;
#endif
	_raw_spin_lock(&mfp_spin_lock);

	mfp_reg = MFP_REG(pin);

	/* Clear bits - EDGE_CLEAR, EDGE_RISE_EN, EDGE_FALL_EN */
	mfp_reg &= ~(MFP_EDGE_MASK);

	switch (edge) {
	case MFP_EDGE_RISE:
		mfp_reg |= (1u << MFPR_ERE_OFFSET);
		break;
	case MFP_EDGE_FALL:
		mfp_reg |= (1u << MFPR_EFE_OFFSET);
		break;
	case MFP_EDGE_BOTH:
		mfp_reg |= (3u << MFPR_ERE_OFFSET);
		break;
	case MFP_EDGE_NONE:
		mfp_reg |= (1u << MFPR_EC_OFFSET);
		break;
	default:
		_raw_spin_unlock(&mfp_spin_lock);
		return -EINVAL;
	}

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

/*
 * The pullup and pulldown state of the MFP pin is by default determined by
 * selected alternate function. In case some buggy devices need to override
 * this default behavior,  this function can be invoked by setting/clearing
 * bit PULL_SEL of MFPRxx.
 */
int mhn_mfp_set_pull(mfp_pin_t pin, int pull)
{
	uint32_t mfp_reg;

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

#ifdef CONFIG_MFP_DEBUG
	if ((MFP_OFFSET(pin) > MHN_MAX_MFP_OFFSET) ||
	    (MFP_OFFSET(pin) < MHN_MIN_MFP_OFFSET))
		return -1;
#endif
	_raw_spin_lock(&mfp_spin_lock);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~MFP_PULL_MASK;

	mfp_reg |= (pull & 0x7u) << MFPR_PD_OFFSET;

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	_raw_spin_unlock(&mfp_spin_lock);

	return 0;
}

static struct mfp_regs context;
void mhn_mfp_save(void)
{
	int i, offset;

	/* specify the membase */
	context.membase = (unsigned char *)KSEG0(PADBASE);

	for (i = 0; i < MAX_MFP_PINS; i++) {
		offset = i << 2;
		context.mfp[i] = readl(context.membase + offset);
	}
}

void mhn_mfp_restore(void)
{
	int i, offset;

	/* check the membase */
	if (context.membase == NULL)
		return;

	for (i = 0; i < MAX_MFP_PINS; i++) {
		offset = i << 2;
		writel(context.mfp[i], context.membase + offset);
	}
}

EXPORT_SYMBOL_GPL(mhn_mfp_set_config);
EXPORT_SYMBOL_GPL(mhn_mfp_set_configs);
EXPORT_SYMBOL_GPL(mhn_mfp_set_afds);
EXPORT_SYMBOL_GPL(mhn_mfp_set_rdh);
EXPORT_SYMBOL_GPL(mhn_mfp_set_lpm);
EXPORT_SYMBOL_GPL(mhn_mfp_set_edge);
EXPORT_SYMBOL_GPL(mhn_mfp_set_pull);
EXPORT_SYMBOL_GPL(mhn_mfp_save);
EXPORT_SYMBOL_GPL(mhn_mfp_restore);

