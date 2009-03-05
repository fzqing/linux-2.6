/*
 *  linux/drivers/misc/ucb1x00-core.c
 *
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 *  The UCB1x00 core driver provides basic services for handling IO,
 *  the ADC, interrupts, and accessing registers.  It is designed
 *  such that everything goes through this layer, thereby providing
 *  a consistent locking methodology, as well as allowing the drivers
 *  to be used on other non-MCP-enabled hardware platforms.
 *
 *  Note that all locks are private to this file.  Nothing else may
 *  touch them.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>

#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include "ucb1x00.h"

/**
 *	ucb1x00_io_set_dir - set IO direction
 *	@ucb: UCB1x00 structure describing chip
 *	@in:  bitfield of IO pins to be set as inputs
 *	@out: bitfield of IO pins to be set as outputs
 *
 *	Set the IO direction of the ten general purpose IO pins on
 *	the UCB1x00 chip.  The @in bitfield has priority over the
 *	@out bitfield, in that if you specify a pin as both input
 *	and output, it will end up as an input.
 *
 *	ucb1x00_enable must have been called to enable the comms
 *	before using this function.
 *
 *	This function takes a spinlock, disabling interrupts.
 */
void ucb1x00_io_set_dir(struct ucb1x00 *ucb, unsigned int in, unsigned int out)
{
	unsigned long flags;

	spin_lock_irqsave(&ucb->io_lock, flags);
	ucb->io_dir |= out;
	ucb->io_dir &= ~in;
	spin_unlock_irqrestore(&ucb->io_lock, flags);

	ucb1x00_reg_write(ucb, UCB_IO_DIR, ucb->io_dir);
}

/**
 *	ucb1x00_io_write - set or clear IO outputs
 *	@ucb:   UCB1x00 structure describing chip
 *	@set:   bitfield of IO pins to set to logic '1'
 *	@clear: bitfield of IO pins to set to logic '0'
 *
 *	Set the IO output state of the specified IO pins.  The value
 *	is retained if the pins are subsequently configured as inputs.
 *	The @clear bitfield has priority over the @set bitfield -
 *	outputs will be cleared.
 *
 *	ucb1x00_enable must have been called to enable the comms
 *	before using this function.
 *
 *	This function takes a spinlock, disabling interrupts.
 */
void ucb1x00_io_write(struct ucb1x00 *ucb, unsigned int set, unsigned int clear)
{
	unsigned long flags;

	spin_lock_irqsave(&ucb->io_lock, flags);
	ucb->io_out |= set;
	ucb->io_out &= ~clear;
	spin_unlock_irqrestore(&ucb->io_lock, flags);

	ucb1x00_reg_write(ucb, UCB_IO_DATA, ucb->io_out);
}

/**
 *	ucb1x00_io_read - read the current state of the IO pins
 *	@ucb: UCB1x00 structure describing chip
 *
 *	Return a bitfield describing the logic state of the ten
 *	general purpose IO pins.
 *
 *	ucb1x00_enable must have been called to enable the comms
 *	before using this function.
 *
 *	This function does not take any semaphores or spinlocks.
 */
unsigned int ucb1x00_io_read(struct ucb1x00 *ucb)
{
	return ucb1x00_reg_read(ucb, UCB_IO_DATA);
}

/*
 * UCB1300 data sheet says we must:
 *  1. enable ADC	=> 5us (including reference startup time)
 *  2. select input	=> 51*tsibclk  => 4.3us
 *  3. start conversion	=> 102*tsibclk => 8.5us
 * (tsibclk = 1/11981000)
 * Period between SIB 128-bit frames = 10.7us
 */

/**
 *	ucb1x00_adc_enable - enable the ADC converter
 *	@ucb: UCB1x00 structure describing chip
 *
 *	Enable the ucb1x00 and ADC converter on the UCB1x00 for use.
 *	Any code wishing to use the ADC converter must call this
 *	function prior to using it.
 *
 *	This function takes the ADC semaphore to prevent two or more
 *	concurrent uses, and therefore may sleep.  As a result, it
 *	can only be called from process context, not interrupt
 *	context.
 *
 *	You should release the ADC as soon as possible using
 *	ucb1x00_adc_disable.
 */
void ucb1x00_adc_enable(struct ucb1x00 *ucb)
{
	down(&ucb->adc_sem);

	ucb->adc_cr |= UCB_ADC_ENA;

	ucb1x00_enable(ucb);
	ucb1x00_reg_write(ucb, UCB_ADC_CR, ucb->adc_cr);
}

/**
 *	ucb1x00_adc_read - read the specified ADC channel
 *	@ucb: UCB1x00 structure describing chip
 *	@adc_channel: ADC channel mask
 *	@sync: wait for syncronisation pulse.
 *
 *	Start an ADC conversion and wait for the result.  Note that
 *	synchronised ADC conversions (via the ADCSYNC pin) must wait
 *	until the trigger is asserted and the conversion is finished.
 *
 *	This function currently spins waiting for the conversion to
 *	complete (2 frames max without sync).
 *
 *	If called for a synchronised ADC conversion, it may sleep
 *	with the ADC semaphore held.
 */
unsigned int ucb1x00_adc_read(struct ucb1x00 *ucb, int adc_channel, int sync)
{
	unsigned int val;

	if (sync)
		adc_channel |= UCB_ADC_SYNC_ENA;

	ucb1x00_reg_write(ucb, UCB_ADC_CR, ucb->adc_cr | adc_channel);
	ucb1x00_reg_write(ucb, UCB_ADC_CR, ucb->adc_cr | adc_channel | UCB_ADC_START);

	for (;;) {
		val = ucb1x00_reg_read(ucb, UCB_ADC_DATA);
		if (val & UCB_ADC_DAT_VAL)
			break;
		/* yield to other processes */
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(1);
	}

	return UCB_ADC_DAT(val);
}

/**
 *	ucb1x00_adc_disable - disable the ADC converter
 *	@ucb: UCB1x00 structure describing chip
 *
 *	Disable the ADC converter and release the ADC semaphore.
 */
void ucb1x00_adc_disable(struct ucb1x00 *ucb)
{
	ucb->adc_cr &= ~UCB_ADC_ENA;
	ucb1x00_reg_write(ucb, UCB_ADC_CR, ucb->adc_cr);
	ucb1x00_disable(ucb);

	up(&ucb->adc_sem);
}

/*
 * UCB1x00 Interrupt handling.
 *
 * The UCB1x00 can generate interrupts when the SIBCLK is stopped.
 * Since we need to read an internal register, we must re-enable
 * SIBCLK to talk to the chip.  We leave the clock running until
 * we have finished processing all interrupts from the chip.
 */
static irqreturn_t ucb1x00_irq(int irqnr, void *devid, struct pt_regs *regs)
{
	struct ucb1x00 *ucb = devid;
	struct ucb1x00_irq *irq;
	unsigned int isr, i;

	ucb1x00_enable(ucb);
	isr = ucb1x00_reg_read(ucb, UCB_IE_STATUS);
	ucb1x00_reg_write(ucb, UCB_IE_CLEAR, isr);
	ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 0);

	for (i = 0, irq = ucb->irq_handler; i < 16 && isr; i++, isr >>= 1, irq++)
		if (isr & 1 && irq->fn)
			irq->fn(i, irq->devid);
	ucb1x00_disable(ucb);

	return IRQ_HANDLED;
}

/*
 * A restriction with interrupts exists when using the ucb1400, as
 * the codec read/write routines may sleep while waiting for codec
 * access completion and uses semaphores for access control to the
 * AC97 bus.  A complete codec read cycle could take  anywhere from
 * 60 to 100uSec so we *definitely* don't want to spin inside the
 * interrupt handler waiting for codec access.  So, we handle the
 * interrupt by scheduling a RT kernel thread to run in process
 * context instead of interrupt context.
 */
static int ucb1x00_thread(void *_ucb)
{
	struct task_struct *tsk = current;
	struct ucb1x00 *ucb = _ucb;

	ucb->irq_task = tsk;
	daemonize("kUCB1x00d");
	allow_signal(SIGKILL);
	tsk->policy = SCHED_FIFO;
	tsk->rt_priority = 1;

	for (;;) {
		wait_for_completion_interruptible(&ucb->irq_wait);
		if (signal_pending(tsk))
			break;
		ucb1x00_irq(-1, ucb, NULL);
		enable_irq(ucb->irq);
	}

	ucb->irq_task = NULL;
	complete_and_exit(&ucb->complete, 0);
}

static irqreturn_t ucb1x00_threaded_irq(int irqnr, void *devid, struct pt_regs *regs)
{
	struct ucb1x00 *ucb = devid;
	if (irqnr == ucb->irq) {
		disable_irq(ucb->irq);
		complete(&ucb->irq_wait);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/**
 *	ucb1x00_hook_irq - hook a UCB1x00 interrupt
 *	@ucb:   UCB1x00 structure describing chip
 *	@idx:   interrupt index
 *	@fn:    function to call when interrupt is triggered
 *	@devid: device id to pass to interrupt handler
 *
 *	Hook the specified interrupt.  You can only register one handler
 *	for each interrupt source.  The interrupt source is not enabled
 *	by this function; use ucb1x00_enable_irq instead.
 *
 *	Interrupt handlers will be called with other interrupts enabled.
 *
 *	Returns zero on success, or one of the following errors:
 *	 -EINVAL if the interrupt index is invalid
 *	 -EBUSY if the interrupt has already been hooked
 */
int ucb1x00_hook_irq(struct ucb1x00 *ucb, unsigned int idx, void (*fn)(int, void *), void *devid)
{
	struct ucb1x00_irq *irq;
	int ret = -EINVAL;

	if (idx < 16) {
		irq = ucb->irq_handler + idx;
		ret = -EBUSY;

		spin_lock_irq(&ucb->lock);
		if (irq->fn == NULL) {
			irq->devid = devid;
			irq->fn = fn;
			ret = 0;
		}
		spin_unlock_irq(&ucb->lock);
	}
	return ret;
}

/**
 *	ucb1x00_enable_irq - enable an UCB1x00 interrupt source
 *	@ucb: UCB1x00 structure describing chip
 *	@idx: interrupt index
 *	@edges: interrupt edges to enable
 *
 *	Enable the specified interrupt to trigger on %UCB_RISING,
 *	%UCB_FALLING or both edges.  The interrupt should have been
 *	hooked by ucb1x00_hook_irq.
 */
void ucb1x00_enable_irq(struct ucb1x00 *ucb, unsigned int idx, int edges)
{
	unsigned long flags;

	if (idx < 16) {
		spin_lock_irqsave(&ucb->lock, flags);
		if (edges & UCB_RISING)
			ucb->irq_ris_enbl |= 1 << idx;
		if (edges & UCB_FALLING)
			ucb->irq_fal_enbl |= 1 << idx;
		spin_unlock_irqrestore(&ucb->lock, flags);

		ucb1x00_enable(ucb);

		/* This prevents spurious interrupts on the UCB1400 */
		ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 1 << idx);
		ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 0);

		ucb1x00_reg_write(ucb, UCB_IE_RIS, ucb->irq_ris_enbl);
		ucb1x00_reg_write(ucb, UCB_IE_FAL, ucb->irq_fal_enbl);

		ucb1x00_disable(ucb);
	}
}

/**
 *	ucb1x00_disable_irq - disable an UCB1x00 interrupt source
 *	@ucb: UCB1x00 structure describing chip
 *	@edges: interrupt edges to disable
 *
 *	Disable the specified interrupt triggering on the specified
 *	(%UCB_RISING, %UCB_FALLING or both) edges.
 */
void ucb1x00_disable_irq(struct ucb1x00 *ucb, unsigned int idx, int edges)
{
	unsigned long flags;

	if (idx < 16) {
		spin_lock_irqsave(&ucb->lock, flags);
		if (edges & UCB_RISING)
			ucb->irq_ris_enbl &= ~(1 << idx);
		if (edges & UCB_FALLING)
			ucb->irq_fal_enbl &= ~(1 << idx);
		spin_unlock_irqrestore(&ucb->lock, flags);

		ucb1x00_enable(ucb);
		ucb1x00_reg_write(ucb, UCB_IE_RIS, ucb->irq_ris_enbl);
		ucb1x00_reg_write(ucb, UCB_IE_FAL, ucb->irq_fal_enbl);
		ucb1x00_disable(ucb);
	}
}

/**
 *	ucb1x00_free_irq - disable and free the specified UCB1x00 interrupt
 *	@ucb: UCB1x00 structure describing chip
 *	@idx: interrupt index
 *	@devid: device id.
 *
 *	Disable the interrupt source and remove the handler.  devid must
 *	match the devid passed when hooking the interrupt.
 *
 *	Returns zero on success, or one of the following errors:
 *	 -EINVAL if the interrupt index is invalid
 *	 -ENOENT if devid does not match
 */
int ucb1x00_free_irq(struct ucb1x00 *ucb, unsigned int idx, void *devid)
{
	struct ucb1x00_irq *irq;
	int ret;

	if (idx >= 16)
		goto bad;

	irq = ucb->irq_handler + idx;
	ret = -ENOENT;

	spin_lock_irq(&ucb->lock);
	if (irq->devid == devid) {
		ucb->irq_ris_enbl &= ~(1 << idx);
		ucb->irq_fal_enbl &= ~(1 << idx);

		irq->fn = NULL;
		irq->devid = NULL;
		ret = 0;
	}
	spin_unlock_irq(&ucb->lock);

	ucb1x00_enable(ucb);
	ucb1x00_reg_write(ucb, UCB_IE_RIS, ucb->irq_ris_enbl);
	ucb1x00_reg_write(ucb, UCB_IE_FAL, ucb->irq_fal_enbl);
	ucb1x00_disable(ucb);

	return ret;

bad:
	printk(KERN_ERR "Freeing bad UCB1x00 irq %d\n", idx);
	return -EINVAL;
}

/*
 * Try to probe our interrupt, rather than relying on lots of
 * hard-coded machine dependencies.  For reference, the expected
 * IRQ mappings are:
 *
 *  	Machine		Default IRQ
 *	adsbitsy	IRQ_GPCIN4
 *	cerf		IRQ_GPIO_UCB1200_IRQ
 *	flexanet	IRQ_GPIO_GUI
 *	freebird	IRQ_GPIO_FREEBIRD_UCB1300_IRQ
 *	graphicsclient	ADS_EXT_IRQ(8)
 *	graphicsmaster	ADS_EXT_IRQ(8)
 *	lart		LART_IRQ_UCB1200
 *	omnimeter	IRQ_GPIO23
 *	pfs168		IRQ_GPIO_UCB1300_IRQ
 *	simpad		IRQ_GPIO_UCB1300_IRQ
 *	shannon		SHANNON_IRQ_GPIO_IRQ_CODEC
 *	yopy		IRQ_GPIO_UCB1200_IRQ
 */
static int ucb1x00_detect_irq(struct ucb1x00 *ucb)
{
	unsigned long mask;

	mask = probe_irq_on();
	if (!mask)
		return NO_IRQ;

	/*
	 * Enable the ADC interrupt.
	 */
	ucb1x00_reg_write(ucb, UCB_IE_RIS, UCB_IE_ADC);
	ucb1x00_reg_write(ucb, UCB_IE_FAL, UCB_IE_ADC);
	ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 0xffff);
	ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 0);

	/*
	 * Cause an ADC interrupt.
	 */
	ucb1x00_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA);
	ucb1x00_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA | UCB_ADC_START);

	/*
	 * Wait for the conversion to complete.
	 */
	while ((ucb1x00_reg_read(ucb, UCB_ADC_DATA) & UCB_ADC_DAT_VAL) == 0);
	ucb1x00_reg_write(ucb, UCB_ADC_CR, 0);

	/*
	 * Disable and clear interrupt.
	 */
	ucb1x00_reg_write(ucb, UCB_IE_RIS, 0);
	ucb1x00_reg_write(ucb, UCB_IE_FAL, 0);
	ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 0xffff);
	ucb1x00_reg_write(ucb, UCB_IE_CLEAR, 0);

	/*
	 * Read triggered interrupt.
	 */
	return probe_irq_off(mask);
}

static int ucb1x00_probe(struct mcp *mcp)
{
	struct ucb1x00 *ucb;
	unsigned int id;
	int ret = -ENODEV;

	mcp_enable(mcp);
	id = mcp_reg_read(mcp, UCB_ID);

	if (id != UCB_ID_1200 && id != UCB_ID_1300 && id != UCB_ID_1400) {
		printk(KERN_WARNING "UCB1x00 ID not found: %04x\n", id);
		goto err_disable;
	}

	ucb = kmalloc(sizeof(struct ucb1x00), GFP_KERNEL);
	ret = -ENOMEM;
	if (!ucb)
		goto err_disable;

	memset(ucb, 0, sizeof(struct ucb1x00));

	ucb->cdev.class = &ucb1x00_class;
	ucb->cdev.dev = mcp->attached_device;
	strlcpy(ucb->cdev.class_id, "ucb1x00", sizeof(ucb->cdev.class_id));

	spin_lock_init(&ucb->lock);
	spin_lock_init(&ucb->io_lock);
	sema_init(&ucb->adc_sem, 1);
	init_completion(&ucb->irq_wait);

	ucb->mcp = mcp;
	ucb->id  = id;
	/* distinguish between UCB1400 revs 1B and 2A */
	if (id == UCB_ID_1400 && mcp_reg_read(mcp, 0x00) == 0x002a)
		ucb->id = UCB_ID_1400_BUGGY;

	ucb->irq = ucb1x00_detect_irq(ucb);
	if (ucb->irq == NO_IRQ) {
		printk(KERN_ERR "UCB1x00: IRQ probe failed\n");
		ret = -ENODEV;
		goto err_free;
	}

	ret = request_irq(ucb->irq,
			  id != UCB_ID_1400 ? ucb1x00_irq : ucb1x00_threaded_irq,
			  0, "UCB1x00", ucb);
	if (ret) {
		printk(KERN_ERR "ucb1x00: unable to grab irq%d: %d\n",
			ucb->irq, ret);
		goto err_free;
	}
	set_irq_type(ucb->irq, IRQT_RISING);

	mcp_set_drvdata(mcp, ucb);
	ret = class_device_register(&ucb->cdev);

	if (!ret && id == UCB_ID_1400) {
		init_completion(&ucb->complete);
		ret = kernel_thread(ucb1x00_thread, ucb, CLONE_KERNEL);
		if (ret > 0)
			ret = 0;
	}

	if (ret) {
		free_irq(ucb->irq, ucb);
 err_free:
		kfree(ucb);
	}
 err_disable:
	mcp_disable(mcp);
	return ret;
}

static void ucb1x00_remove(struct mcp *mcp)
{
	struct ucb1x00 *ucb = mcp_get_drvdata(mcp);

	class_device_unregister(&ucb->cdev);
	if (ucb->id == UCB_ID_1400 || ucb->id == UCB_ID_1400_BUGGY) {
		send_sig(SIGKILL, ucb->irq_task, 1);
		wait_for_completion(&ucb->complete);
	}
	free_irq(ucb->irq, ucb);
}

static void ucb1x00_release(struct class_device *dev)
{
	struct ucb1x00 *ucb = classdev_to_ucb1x00(dev);
	kfree(ucb);
}

static struct class ucb1x00_class = {
	.name		= "ucb1x00",
	.release	= ucb1x00_release,
};

int ucb1x00_register_interface(struct class_interface *intf)
{
	intf->class = &ucb1x00_class;
	return class_interface_register(intf);
}

void ucb1x00_unregister_interface(struct class_interface *intf)
{
	class_interface_unregister(intf);
}

static struct mcp_driver ucb1x00_driver = {
	.drv		= {
		.name	= "ucb1x00",
	},
	.probe		= ucb1x00_probe,
	.remove		= ucb1x00_remove,
};

static int __init ucb1x00_init(void)
{
	int ret = class_register(&ucb1x00_class);
	if (ret == 0) {
		ret = mcp_driver_register(&ucb1x00_driver);
		if (ret)
			class_unregister(&ucb1x00_class);
	}
	return ret;
}

static void __exit ucb1x00_exit(void)
{
	mcp_driver_unregister(&ucb1x00_driver);
	class_unregister(&ucb1x00_class);
}

module_init(ucb1x00_init);
module_exit(ucb1x00_exit);

EXPORT_SYMBOL(ucb1x00_class);

EXPORT_SYMBOL(ucb1x00_io_set_dir);
EXPORT_SYMBOL(ucb1x00_io_write);
EXPORT_SYMBOL(ucb1x00_io_read);

EXPORT_SYMBOL(ucb1x00_adc_enable);
EXPORT_SYMBOL(ucb1x00_adc_read);
EXPORT_SYMBOL(ucb1x00_adc_disable);

EXPORT_SYMBOL(ucb1x00_hook_irq);
EXPORT_SYMBOL(ucb1x00_free_irq);
EXPORT_SYMBOL(ucb1x00_enable_irq);
EXPORT_SYMBOL(ucb1x00_disable_irq);

EXPORT_SYMBOL(ucb1x00_register_interface);
EXPORT_SYMBOL(ucb1x00_unregister_interface);

MODULE_AUTHOR("Russell King <rmk@arm.linux.org.uk>");
MODULE_DESCRIPTION("UCB1x00 core driver");
MODULE_LICENSE("GPL");
