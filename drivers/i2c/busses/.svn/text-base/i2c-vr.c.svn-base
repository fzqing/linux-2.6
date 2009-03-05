/*
 *  drivers/i2c/busses/i2c-vr.c
 *
 *  I2C driver for the Vermilion Range I2C controller.
 *
 *  Copyright (C) 2006 MontaVista Software, Inc.
 *
 *  Derived from drivers/i2c/busses/i2c-pxa.c
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *  Copyright (C) 2004 Deep Blue Solutions Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#include "i2c-vr.h"

#ifdef DEBUG

struct bits {
	u32 mask;
	const char *set;
	const char *unset;
};
#define BIT(m, s, u)	{ .mask = m, .set = s, .unset = u }

static inline void
decode_bits(const char *prefix, const struct bits *bits, int num, u32 val)
{
	printk("%s %08x: ", prefix, val);
	while (num--) {
		const char *str = val & bits->mask ? bits->set : bits->unset;
		if (str)
			printk("%s ", str);
		bits++;
	}
}

static const struct bits isr_bits[] = {
	BIT(ISR_RWM, "RX", "TX"),
	BIT(ISR_ACKNAK, "NAK", "ACK"),
	BIT(ISR_UB, "Bsy", "Rdy"),
	BIT(ISR_IBB, "BusBsy", "BusRdy"),
	BIT(ISR_SSD, "SlaveStop", NULL),
	BIT(ISR_ALD, "ALD", NULL),
	BIT(ISR_ITE, "TxEmpty", NULL),
	BIT(ISR_IRF, "RxFull", NULL),
	BIT(ISR_GCAD, "GenCall", NULL),
	BIT(ISR_SAD, "SlaveAddr", NULL),
	BIT(ISR_BED, "BusErr", NULL),
};

static void decode_ISR(unsigned int val)
{
	decode_bits(KERN_DEBUG "ISR", isr_bits, ARRAY_SIZE(isr_bits), val);
	printk("\n");
}

static const struct bits icr_bits[] = {
	BIT(ICR_START, "START", NULL),
	BIT(ICR_STOP, "STOP", NULL),
	BIT(ICR_ACKNAK, "ACKNAK", NULL),
	BIT(ICR_TB, "TB", NULL),
	BIT(ICR_MA, "MA", NULL),
	BIT(ICR_SCLE, "SCLE", "scle"),
	BIT(ICR_IUE, "IUE", "iue"),
	BIT(ICR_GCD, "GCD", NULL),
	BIT(ICR_ITEIE, "ITEIE", NULL),
	BIT(ICR_IRFIE, "IRFIE", NULL),
	BIT(ICR_BEIE, "BEIE", NULL),
	BIT(ICR_SSDIE, "SSDIE", NULL),
	BIT(ICR_ALDIE, "ALDIE", NULL),
	BIT(ICR_SADIE, "SADIE", NULL),
	BIT(ICR_UR, "UR", "ur"),
};

static void decode_ICR(unsigned int val)
{
	decode_bits(KERN_DEBUG "ICR", icr_bits, ARRAY_SIZE(icr_bits), val);
	printk("\n");
}

static unsigned int i2c_debug = DEBUG;

static void i2c_vr_show_state(struct vr_i2c *i2c, int lno, const char *fname)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;

	dev_dbg(&i2c->adap.dev, "state:%s:%d: ISR=%08x, ICR=%08x, IBMR=%02x\n",
		fname, lno, readl(&regs->isr), readl(&regs->icr),
		readl(&regs->ibmr));
}

#define show_state(i2c) i2c_vr_show_state(i2c, __LINE__, __FUNCTION__)
#else
#define i2c_debug	0

#define show_state(i2c) do { } while (0)
#define decode_ISR(val) do { } while (0)
#define decode_ICR(val) do { } while (0)
#endif

static void i2c_vr_scream_blue_murder(struct vr_i2c *i2c, const char *why)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	int i;

	dev_err(&i2c->adap.dev, "error: %s\n", why);
	dev_err(&i2c->adap.dev, "msg_num: %d msg_idx: %d msg_ptr: %d\n",
	       i2c->msg_num, i2c->msg_idx, i2c->msg_ptr);
	dev_err(&i2c->adap.dev, "ICR: %08x ISR: %08x\n",
		readl(&regs->icr), readl(&regs->isr));
	dev_err(&i2c->adap.dev, "log: ");
	for (i = 0; i < i2c->irqlogidx; i++)
		printk("[%08x:%08x] ", i2c->isrlog[i], i2c->icrlog[i]);
	printk("\n");
}

static void i2c_vr_abort(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	unsigned long timeout = jiffies + HZ / 4;
	u32 icr;

	while (time_before(jiffies, timeout) && (readl(&regs->ibmr) & 0x1) == 0) {
		icr = readl(&regs->icr);

		icr &= ~ICR_START;
		icr |= ICR_ACKNAK | ICR_STOP | ICR_TB;

		writel(icr, &regs->icr);

		show_state(i2c);

		msleep(1);
	}

	icr = readl(&regs->icr) & ~(ICR_MA | ICR_START | ICR_STOP);
	writel(icr, &regs->icr);
}

static int i2c_vr_wait_bus_not_busy(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	int timeout = I2C_VR_DEF_TIMEOUT;

	while (timeout-- && (readl(&regs->isr) & (ISR_IBB | ISR_UB))) {
		if ((readl(&regs->isr) & ISR_SAD) != 0)
			timeout += 4;

		msleep(2);
		show_state(i2c);
	}

	if (timeout <= 0)
		show_state(i2c);

	return timeout <= 0 ? I2C_RETRY : 0;
}

static int i2c_vr_wait_master(struct vr_i2c *i2c)
{
	struct i2c_vr_regs *__iomem regs = i2c->regs;
	unsigned long timeout = jiffies + HZ * 4;
	u32 isr;

	while (time_before(jiffies, timeout)) {
		if (i2c_debug > 1)
			dev_dbg(&i2c->adap.dev,
				"%s: %ld: ISR=%08x, ICR=%08x, IBMR=%02x\n",
				__func__, (long)jiffies, readl(&regs->isr),
				readl(&regs->icr), readl(&regs->ibmr));

		/* wait for unit and bus being not busy, and we also do a
		 * quick check of the i2c lines themselves to ensure they've
		 * gone high...
		 */
		isr = readl(&regs->isr);
		if ((isr & (ISR_UB | ISR_IBB)) == 0 &&
		    (readl(&regs->ibmr) == 3)) {
			if (i2c_debug > 0)
				dev_dbg(&i2c->adap.dev, "%s: done\n", __func__);
			return 1;
		}

		msleep(1);
	}

	if (i2c_debug > 0)
		dev_dbg(&i2c->adap.dev, "%s: did not free\n", __func__);

	return 0;
}

static int i2c_vr_set_master(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	u32 val;

	if (i2c_debug)
		dev_dbg(&i2c->adap.dev, "setting to bus master\n");

	val = readl(&regs->isr);
	if ((val & (ISR_UB | ISR_IBB)) != 0) {
		dev_dbg(&i2c->adap.dev, "%s: unit is busy\n", __func__);
		if (!i2c_vr_wait_master(i2c)) {
			dev_dbg(&i2c->adap.dev, "%s: error: unit busy\n",
				__func__);
			return I2C_RETRY;
		}
	}

	val = readl(&regs->icr);
	writel(val | ICR_SCLE, &regs->icr);
	return 0;
}

static void i2c_vr_reset(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;

	dev_dbg(&i2c->adap.dev, "Resetting I2C Controller Unit\n");

	/* abort any transfer currently under way */
	i2c_vr_abort(i2c);

	/* reset */
	writel(ICR_UR, &regs->icr);
	writel(I2C_ISR_INIT, &regs->isr);
	writel(readl(&regs->icr) & ~ICR_UR, &regs->icr);

	/* set control register values and enable unit */
	writel(I2C_ICR_INIT | ICR_IUE, &regs->icr);

	udelay(100);
}

#ifdef CONFIG_PM
static void i2c_vr_stop(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	u32 icr;

	icr = readl(&regs->icr);

	show_state(i2c);

	icr |= ICR_STOP;
	icr &= ~(ICR_START);
	writel(icr, &regs->icr);

	show_state(i2c);
}

static void i2c_vr_start(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	u32 icr;

	icr = readl(&regs->icr);
	icr |= ICR_START;
	icr &= ~(ICR_STOP | ICR_ALDIE | ICR_ACKNAK);
	writel(icr, &regs->icr);
}

static int i2c_vr_controller_suspend(struct pci_dev *dev, uint32_t state)
{
	struct vr_i2c *i2c = (struct vr_i2c *)pci_get_drvdata(dev);
	if (i2c) {
		i2c_vr_wait_bus_not_busy(&i2c[0]);
		i2c_vr_stop(&i2c[0]);

		i2c_vr_wait_bus_not_busy(&i2c[1]);
		i2c_vr_stop(&i2c[1]);
	}
	return 0;
}

static int i2c_vr_controller_resume(struct pci_dev *dev)
{
	struct vr_i2c *i2c = (struct vr_i2c *)pci_get_drvdata(dev);

	if (i2c) {
		i2c_vr_reset(&i2c[0]);
		i2c_vr_reset(&i2c[1]);

		i2c_vr_start(&i2c[0]);
		i2c_vr_start(&i2c[1]);
	}
	return 0;
}
#endif				/* ifdef CONFIG_PM */

/*
 * VR I2C Master mode
 */

static inline unsigned int i2c_vr_addr_byte(struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD)
		addr |= 1;

	return addr;
}

static inline void i2c_vr_start_message(struct vr_i2c *i2c)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	u32 icr;

	/*
	 * Step 1: target slave address into IDBR
	 */
	writel(i2c_vr_addr_byte(i2c->msg), &regs->idbr);

	/*
	 * Step 2: initiate the write.
	 */
	icr = readl(&regs->icr) & ~(ICR_STOP | ICR_ALDIE);
	writel(icr | ICR_START | ICR_TB, &regs->icr);
}

/*
 * We are protected by the adapter bus semaphore.
 */
static int i2c_vr_do_xfer(struct vr_i2c *i2c, struct i2c_msg *msg, int num)
{
	long timeout;
	int ret;

	/*
	 * Wait for the bus to become free.
	 */
	ret = i2c_vr_wait_bus_not_busy(i2c);
	if (ret) {
		dev_err(&i2c->adap.dev,
			"i2c_vr: timeout waiting for bus free\n");
		goto out;
	}

	/*
	 * Set master mode.
	 */
	ret = i2c_vr_set_master(i2c);
	if (ret) {
		dev_err(&i2c->adap.dev, "i2c_vr_set_master: error %d\n", ret);
		goto out;
	}

	spin_lock_irq(&i2c->lock);

	i2c->msg = msg;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	i2c->msg_ptr = 0;
	i2c->irqlogidx = 0;

	i2c_vr_start_message(i2c);

	spin_unlock_irq(&i2c->lock);

	/*
	 * The rest of the processing occurs in the interrupt handler.
	 */
	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 5);

	/*
	 * We place the return code in i2c->msg_idx.
	 */
	ret = i2c->msg_idx;

	if (timeout == 0)
		i2c_vr_scream_blue_murder(i2c, "timeout");

      out:
	return ret;
}

/*
 * i2c_vr_master_complete - complete the message and wake up.
 */
static void i2c_vr_master_complete(struct vr_i2c *i2c, int ret)
{
	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;
	wake_up(&i2c->wait);
}

static void i2c_vr_irq_txempty(struct vr_i2c *i2c, u32 isr)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	u32 icr;

	icr = readl(&regs->icr) & ~(ICR_START | ICR_STOP | ICR_ACKNAK | ICR_TB);

      again:
	/*
	 * If ISR_ALD is set, we lost arbitration.
	 */
	if (isr & ISR_ALD) {
		/*
		 * Do we need to do anything here?  The docs
		 * are vague about what happens.
		 */
		i2c_vr_scream_blue_murder(i2c, "ALD set");

		/*
		 * We ignore this error.  We seem to see spurious ALDs
		 * for seemingly no reason.  If we handle them as I think
		 * they should, we end up causing an I2C error, which
		 * is painful for some systems.
		 */
		return;		/* ignore */
	}

	if (isr & ISR_BED) {
		int ret = I2C_VR_BUS_ERROR;

		/*
		 * I2C bus error - either the device NAK'd us, or
		 * something more serious happened.  If we were NAK'd
		 * on the initial address phase, we can retry.
		 */
		if (isr & ISR_ACKNAK) {
			if (i2c->msg_ptr == 0 && i2c->msg_idx == 0)
				ret = I2C_RETRY;
			else
				ret = I2C_VR_XFER_NAKED;
		}
		i2c_vr_master_complete(i2c, ret);
	} else if (isr & ISR_RWM) {
		/*
		 * Read mode.  We have just sent the address byte, and
		 * now we must initiate the transfer.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1 &&
		    i2c->msg_idx == i2c->msg_num - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
	} else if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * Write mode.  Write the next data byte.
		 */
		writel(i2c->msg->buf[i2c->msg_ptr++], &regs->idbr);

		icr |= ICR_ALDIE | ICR_TB;

		/*
		 * If this is the last byte of the last message, send
		 * a STOP.
		 */

		if (i2c->msg_ptr == i2c->msg->len &&
		    i2c->msg_idx == i2c->msg_num - 1)
			icr |= ICR_STOP;
	} else if (i2c->msg_idx < i2c->msg_num - 1) {
		/*
		 * Next segment of the message.
		 */
		i2c->msg_ptr = 0;
		i2c->msg_idx++;
		i2c->msg++;

		/*
		 * If we aren't doing a repeated start and address,
		 * go back and try to send the next byte.  Note that
		 * we do not support switching the R/W direction here.
		 */
		if (i2c->msg->flags & I2C_M_NOSTART)
			goto again;

		/*
		 * Write the next address.
		 */
		writel(i2c_vr_addr_byte(i2c->msg), &regs->idbr);

		/*
		 * And trigger a repeated start, and send the byte.
		 */
		icr &= ~ICR_ALDIE;
		icr |= ICR_START | ICR_TB;
	} else {
		if (i2c->msg->len == 0) {
			/*
			 * Device probes have a message length of zero
			 * and need the bus to be reset before it can
			 * be used again.
			 */
			i2c_vr_reset(i2c);
		}
		i2c_vr_master_complete(i2c, 0);
	}

	i2c->icrlog[i2c->irqlogidx - 1] = icr;

	writel(icr, &regs->icr);
	show_state(i2c);
}

static void i2c_vr_irq_rxfull(struct vr_i2c *i2c, u32 isr)
{
	struct i2c_vr_regs __iomem *regs = i2c->regs;
	u32 icr;

	icr = readl(&regs->icr) & ~(ICR_START | ICR_STOP | ICR_ACKNAK | ICR_TB);

	/*
	 * Read the byte.
	 */
	i2c->msg->buf[i2c->msg_ptr++] = readl(&regs->idbr);

	if (i2c->msg_ptr < i2c->msg->len) {
		/*
		 * If this is the last byte of the last
		 * message, send a STOP.
		 */
		if (i2c->msg_ptr == i2c->msg->len - 1)
			icr |= ICR_STOP | ICR_ACKNAK;

		icr |= ICR_ALDIE | ICR_TB;
	} else {
		i2c_vr_master_complete(i2c, 0);
	}

	i2c->icrlog[i2c->irqlogidx - 1] = icr;

	writel(icr, &regs->icr);
}

static irqreturn_t i2c_vr_handler(int this_irq, void *dev_id,
				  struct pt_regs *pt_regs)
{
	struct vr_i2c *i2cs = dev_id;
	u32 isr[I2C_VR_ADAP_NR];
	int i;

	isr[0] = readl(&i2cs[0].regs->isr);
	isr[1] = readl(&i2cs[1].regs->isr);

	if (!(isr[0] & I2C_ISR_IRQ) && !(isr[1] & I2C_ISR_IRQ)) {
		/*
		 * This isn't our interrupt.  It must be for another device
		 * sharing this IRQ.
		 */
		return IRQ_NONE;
	}

	for (i = 0; i < I2C_VR_ADAP_NR; i++) {
		struct vr_i2c *i2c = &i2cs[i];
		struct i2c_vr_regs __iomem *regs = i2c->regs;

		if (!(isr[i] & I2C_ISR_IRQ))
			continue;

		if (i2c_debug > 2) {
			dev_dbg(&i2c->adap.dev,
				"%s: ISR=%08x, ICR=%08x, IBMR=%02x\n",
				__func__, isr[i], readl(&regs->icr),
				readl(&regs->ibmr));
			decode_ISR(isr[i]);
			decode_ICR(readl(&regs->icr));
		}

		if (i2c->irqlogidx < ARRAY_SIZE(i2c->isrlog))
			i2c->isrlog[i2c->irqlogidx++] = isr[i];

		show_state(i2c);

		/*
		* Always clear all pending IRQs.
		*/
		writel(isr[i] & I2C_ISR_IRQ, &regs->isr);

		if (i2c->msg) {
			if (isr[i] & ISR_ITE)
				i2c_vr_irq_txempty(i2c, isr[i]);
			if (isr[i] & ISR_IRF)
				i2c_vr_irq_rxfull(i2c, isr[i]);
		} else {
			i2c_vr_scream_blue_murder(i2c, "spurious irq");
		}
	}

	return IRQ_HANDLED;
}

static int i2c_vr_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct vr_i2c *i2c = i2c_get_adapdata(adap);
	struct i2c_vr_regs __iomem *regs;
	int ret, i;

	regs = i2c->regs;

	/* If the I2C controller is disabled we need to reset it (probably due
	  to a suspend/resume destroying state). We do this here as we can then
	  avoid worrying about resuming the controller before its users. */
	if (!(readl(&regs->icr) & ICR_IUE))
		i2c_vr_reset(i2c);

	for (i = adap->retries; i >= 0; i--) {
		ret = i2c_vr_do_xfer(i2c, msgs, num);
		if (ret != I2C_RETRY)
			goto out;

		if (i2c_debug)
			dev_dbg(&adap->dev, "Retrying transmission\n");
		udelay(100);
	}
	dev_dbg(&adap->dev, "Exhausted retries\n");
	ret = -EREMOTEIO;
      out:
	return ret;
}

static u32 i2c_vr_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm i2c_vr_algorithm = {
	.id = I2C_ALGO_PXA,
	.master_xfer = i2c_vr_xfer,
	.functionality = i2c_vr_functionality,
};

static struct vr_i2c i2c_vr[I2C_VR_ADAP_NR] = {
	{
	 .lock = SPIN_LOCK_UNLOCKED,
	 .wait = __WAIT_QUEUE_HEAD_INITIALIZER(i2c_vr[0].wait),
	 .adap = {
		  .owner = THIS_MODULE,
		  .id = I2C_ALGO_PXA,
		  .algo = &i2c_vr_algorithm,
		  .name = "vr_i2c0",
		  .retries = 5,
		  },
	 },
	{
	 .lock = SPIN_LOCK_UNLOCKED,
	 .wait = __WAIT_QUEUE_HEAD_INITIALIZER(i2c_vr[1].wait),
	 .adap = {
		  .owner = THIS_MODULE,
		  .id = I2C_ALGO_PXA,
		  .algo = &i2c_vr_algorithm,
		  .name = "vr_i2c1",
		  .retries = 5,
		  },
	 },
};

static int i2c_vr_mapregs(struct pci_dev *dev, int idx)
{
	void __iomem *base;

	if (!dev || idx >= I2C_VR_ADAP_NR)
		return -EINVAL;

	base = pci_iomap(dev, idx, 0);
	if (!base) {
		dev_dbg(&dev->dev, "error mapping memory\n");
		return -EFAULT;
	}
	i2c_vr[idx].regs = base;
	return 0;
}

static int i2c_vr_setdata(struct pci_dev *dev, int idx)
{
	int retval;

	if (!dev || idx >= I2C_VR_ADAP_NR)
		return -EINVAL;

	i2c_vr[idx].adap.dev.parent = &dev->dev;
	i2c_set_adapdata(&i2c_vr[idx].adap, &i2c_vr[idx]);
	retval = i2c_add_adapter(&i2c_vr[idx].adap);
	return retval;
}

static int i2c_vr_unmapregs(struct pci_dev *dev, int idx)
{
	if (!dev || idx >= I2C_VR_ADAP_NR)
		return -EINVAL;

	if (i2c_vr[idx].regs)
		pci_iounmap(dev, i2c_vr[idx].regs);

	return 0;
}

static int i2c_vr_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int retval;

	retval = pci_enable_device(dev);
	if (retval)
		goto out;

	retval = pci_request_regions(dev, "vr_i2c");
	if (retval)
		goto disable_dev;

	retval = i2c_vr_mapregs(dev, 0);
	if (retval)
		goto release_bars;

	retval = i2c_vr_mapregs(dev, 1);
	if (retval)
		goto unmap_0;

	/* enable Message Signaled Interrupts */
	pci_enable_msi(dev);

	retval = request_irq(dev->irq, i2c_vr_handler, SA_SHIRQ, "vr_i2c",
				i2c_vr);
	if (retval) {
		dev_err(&dev->dev, "request irq %s fail, %d\n",
			pci_name(dev), retval);
		goto unmap_1;
	}

	retval = i2c_vr_setdata(dev, 0);
	if (retval)
		goto fail_irq;
	i2c_vr_reset(&i2c_vr[0]);

	retval = i2c_vr_setdata(dev, 1);
	if (retval)
		goto fail_setdata;
	i2c_vr_reset(&i2c_vr[1]);

	pci_set_drvdata(dev, i2c_vr);
	return 0;

      fail_setdata:
	i2c_del_adapter(&i2c_vr[0].adap);
      fail_irq:
	free_irq(dev->irq, i2c_vr);
      unmap_1:
	pci_disable_msi(dev);
	i2c_vr_unmapregs(dev, 1);
      unmap_0:
	i2c_vr_unmapregs(dev, 0);
      release_bars:
	pci_release_regions(dev);
      disable_dev:
	pci_disable_device(dev);
      out:
	return retval;
}

static void i2c_vr_remove(struct pci_dev *dev)
{
	struct i2c_vr_regs __iomem *regs;

	/* disable device */
	regs = i2c_vr[0].regs;
	writel(0, &regs->icr);
	regs = i2c_vr[1].regs;
	writel(0, &regs->icr);

	pci_set_drvdata(dev, NULL);
	i2c_del_adapter(&i2c_vr[1].adap);
	i2c_del_adapter(&i2c_vr[0].adap);

	free_irq(dev->irq, i2c_vr);
	pci_disable_msi(dev);

	i2c_vr_unmapregs(dev, 1);
	i2c_vr_unmapregs(dev, 0);

	pci_release_regions(dev);
	pci_disable_device(dev);
}

#define PCI_DEVICE_ID_VR_I2C	0x5010

static struct pci_device_id i2c_vr_ids[] = {
	{
	 .vendor = PCI_VENDOR_ID_INTEL,
	 .device = PCI_DEVICE_ID_VR_I2C,
	 .subvendor = PCI_ANY_ID,
	 .subdevice = PCI_ANY_ID,
	 },
	{0,}
};

static struct pci_driver i2c_vr_driver = {
	.name = "vr_i2c",
	.id_table = i2c_vr_ids,
	.probe = i2c_vr_probe,
	.remove = __devexit_p(i2c_vr_remove),
#ifdef CONFIG_PM
	.resume = i2c_vr_controller_resume,
	.suspend = i2c_vr_controller_suspend,
#endif
};

static int __init i2c_vr_init(void)
{
	return pci_register_driver(&i2c_vr_driver);
}

static void i2c_vr_exit(void)
{
	return pci_unregister_driver(&i2c_vr_driver);
}

module_init(i2c_vr_init);
module_exit(i2c_vr_exit);

MODULE_DESCRIPTION("Intel Vermilion Range I2C controller driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, i2c_vr_ids);
