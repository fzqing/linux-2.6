/*
 * TI DaVinci GPIO Support
 *
 * Copyright (c) 2006 David Brownell
 * Copyright (c) 2007, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/bitops.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>

#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/cpu.h>

#include <asm/mach/irq.h>

static DEFINE_SPINLOCK(gpio_lock);
static DECLARE_BITMAP(gpio_in_use, DAVINCI_N_GPIO);

int gpio_request(unsigned gpio, const char *tag)
{
	if (gpio >= DAVINCI_N_GPIO)
		return -EINVAL;

	if (test_and_set_bit(gpio, gpio_in_use))
		return -EBUSY;

	return 0;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	if (gpio >= DAVINCI_N_GPIO)
		return;

	clear_bit(gpio, gpio_in_use);
}
EXPORT_SYMBOL(gpio_free);

/* create a non-inlined version */
static struct gpio_controller *__iomem gpio2controller(unsigned gpio)
{
	return __gpio_to_controller(gpio);
}

/*
 * Assuming the pin is muxed as a gpio output, set its output value.
 */
void __gpio_set(unsigned gpio, int value)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);

	__raw_writel(__gpio_mask(gpio), value ? &g->set_data : &g->clr_data);
}
EXPORT_SYMBOL(__gpio_set);


/*
 * Read the pin's value (works even if it's set up as output);
 * returns zero/nonzero.
 *
 * Note that changes are synched to the GPIO clock, so reading values back
 * right after you've set them may give old values.
 */
int __gpio_get(unsigned gpio)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);

	return !!(__gpio_mask(gpio) & __raw_readl(&g->in_data));
}
EXPORT_SYMBOL(__gpio_get);


/*--------------------------------------------------------------------------*/

/*
 * board setup code *MUST* set PINMUX0 and PINMUX1 as
 * needed, and enable the GPIO clock.
 */

int gpio_direction_input(unsigned gpio)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);
	u32 temp;
	u32 mask;

	if (!g)
		return -EINVAL;

	spin_lock(&gpio_lock);
	mask = __gpio_mask(gpio);
	temp = __raw_readl(&g->dir);
	temp |= mask;
	__raw_writel(temp, &g->dir);
	spin_unlock(&gpio_lock);
	return 0;
}
EXPORT_SYMBOL(gpio_direction_input);

int gpio_direction_output(unsigned gpio, int value)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);
	u32 temp;
	u32 mask;

	if (!g)
		return -EINVAL;

	spin_lock(&gpio_lock);
	mask = __gpio_mask(gpio);
	temp = __raw_readl(&g->dir);
	temp &= ~mask;
	__raw_writel(mask, value ? &g->set_data : &g->clr_data);
	__raw_writel(temp, &g->dir);
	spin_unlock(&gpio_lock);
	return 0;
}
EXPORT_SYMBOL(gpio_direction_output);

void gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(value)) {
		struct gpio_controller *__iomem g;
		u32 mask;

		if (gpio >= DAVINCI_N_GPIO)
			__error_inval_gpio();

		g = __gpio_to_controller(gpio);
		mask = __gpio_mask(gpio);
		if (value)
			__raw_writel(mask, &g->set_data);
		else
			__raw_writel(mask, &g->clr_data);
		return;
	}

	__gpio_set(gpio, value);
}
EXPORT_SYMBOL(gpio_set_value);

int gpio_get_value(unsigned gpio)
{
	struct gpio_controller *__iomem g;

	if (!__builtin_constant_p(gpio))
		return __gpio_get(gpio);

	if (gpio >= DAVINCI_N_GPIO)
		return __error_inval_gpio();

	g = __gpio_to_controller(gpio);
	return !!(__gpio_mask(gpio) & __raw_readl(&g->in_data));
}
EXPORT_SYMBOL(gpio_get_value);

/*
 * We expect irqs will normally be set up as input pins, but they can also be
 * used as output pins ... which is convenient for testing.
 *
 * NOTE:  GPIO0..GPIO7 also have direct INTC hookups, which work in addition
 * to their GPIOBNK0 irq (but with a bit less overhead).  But we don't have
 * a good way to hook those up ...
 *
 * All those INTC hookups (GPIO0..GPIO7 plus five IRQ banks) can also
 * serve as EDMA event triggers.
 */

static void gpio_irq_disable(unsigned irq)
{
	struct gpio_controller *__iomem g = get_irq_chipdata(irq);
	u32 mask = __gpio_mask(irq_to_gpio(irq));

	__raw_writel(mask, &g->clr_falling);
	__raw_writel(mask, &g->clr_rising);
}

static void gpio_irq_enable(unsigned irq)
{
	struct gpio_controller *__iomem g = get_irq_chipdata(irq);
	u32 mask = __gpio_mask(irq_to_gpio(irq));

	if (irq_desc[irq].status & IRQT_FALLING)
		__raw_writel(mask, &g->set_falling);
	if (irq_desc[irq].status & IRQT_RISING)
		__raw_writel(mask, &g->set_rising);
}

static int gpio_irq_type(unsigned irq, unsigned trigger)
{
	struct gpio_controller *__iomem g = get_irq_chipdata(irq);
	u32 mask = __gpio_mask(irq_to_gpio(irq));

	if (trigger & ~(IRQT_FALLING | IRQT_RISING))
		return -EINVAL;

	irq_desc[irq].status &= ~IRQT_BOTHEDGE;
	irq_desc[irq].status |= trigger;

	__raw_writel(mask, (trigger & IRQT_FALLING)
		     ? &g->set_falling : &g->clr_falling);
	__raw_writel(mask, (trigger & IRQT_RISING)
		     ? &g->set_rising : &g->clr_rising);
	return 0;
}

static struct irqchip gpio_irqchip = {
	.unmask		= gpio_irq_enable,
	.mask		= gpio_irq_disable,
	.type		= gpio_irq_type,
};

static void
gpio_irq_handler(unsigned irq, struct irqdesc *desc, struct pt_regs *regs)
{
	struct gpio_controller *__iomem g = get_irq_chipdata(irq);
	u32 mask = 0xffff;

	/* we only care about one bank */
	if (irq & 1)
		mask <<= 16;

	/* temporarily mask (level sensitive) parent IRQ */
	desc->chip->ack(irq);
	while (1) {
		u32		status;
		struct irqdesc	*gpio;
		int		n;
		int		res;

		/* ack any irqs */
		status = __raw_readl(&g->intstat) & mask;
		if (!status)
			break;
		__raw_writel(status, &g->intstat);
		if (irq & 1)
			status >>= 16;

		/* now demux them to the right lowlevel handler */
		n = (int)get_irq_data(irq);
		gpio = &irq_desc[n];
		while (status) {
			res = ffs(status);
			n += res;
			gpio += res;
			desc_handle_irq(n - 1, gpio - 1, regs);
			status >>= res;
		}
	}
	desc->chip->unmask(irq);
	/* now it may re-trigger */
}

/*
 * NOTE:  for suspend/resume, probably best to make a sysdev (and class)
 * with its suspend/resume calls hooking into the results of the set_wake()
 * calls ... so if no gpios are wakeup events the clock can be disabled,
 * with outputs left at previously set levels, and so that VDD3P3V.IOPWDN0
 * can be set appropriately for GPIOV33 pins.
 */


int __init davinci_gpio_irq_setup(void)
{
	unsigned	gpio, irq, bank, banks;
	struct clk	*clk;

	clk = clk_get(NULL, "gpio");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "Error %ld getting gpio clock?\n",
		       PTR_ERR(clk));
		return 0;
	}

	clk_enable(clk);

	for (gpio = 0, irq = gpio_to_irq(0), bank = (cpu_is_davinci_dm355() ?
	     IRQ_DM355_GPIOBNK0 : (cpu_is_davinci_dm6467() ?
	     IRQ_DM646X_GPIOBNK0 : IRQ_GPIOBNK0));
	     gpio < DAVINCI_N_GPIO; bank++) {
		struct gpio_controller	*__iomem g = gpio2controller(gpio);
		unsigned		i;

		__raw_writel(~0, &g->clr_falling);
		__raw_writel(~0, &g->clr_rising);

		/* set up all irqs in this bank */
		set_irq_chained_handler(bank, gpio_irq_handler);
		set_irq_chipdata(bank, g);
		set_irq_data(bank, (void *)irq);

		for (i = 0; i < 16 && gpio < DAVINCI_N_GPIO;
		     i++, irq++, gpio++) {
			set_irq_chip(irq, &gpio_irqchip);
			set_irq_chipdata(irq, g);
			set_irq_handler(irq, do_simple_IRQ);
			set_irq_flags(irq, IRQF_VALID);
		}
	}

	/* BINTEN -- per-bank interrupt enable. genirq would also let these
	 * bits be set/cleared dynamically.
	 */
	if (cpu_is_davinci_dm355())
		banks = 0x3f;
	else
		banks = 0x1f;

	__raw_writel(banks, (void *__iomem)
		     IO_ADDRESS(DAVINCI_GPIO_BASE + 0x08));

	printk(KERN_INFO "DaVinci: %d gpio irqs\n", irq - gpio_to_irq(0));

	return 0;
}

