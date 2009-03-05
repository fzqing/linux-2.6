/*
   -------------------------------------------------------------------------
   i2c-emma2rh.c i2c-hw access for the I2C peripheral on the NEC EMMA2RH
   -------------------------------------------------------------------------

    Copyright (C) NEC Electronics Corporation 2005-2006

    Changes made to support the I2C peripheral on the NEC EMMA2RH

   -------------------------------------------------------------------------
    This file was highly leveraged from i2c-elektor.c, which was created
    by Simon G. Vogl and Hans Berglund:

     Copyright (C) 1995-97 Simon G. Vogl
                   1998-99 Hans Berglund

    With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
    Frodo Looijaard <frodol@dds.nl>

    Partialy rewriten by Oleg I. Vdovikin for mmapped support of
    for Alpha Processor Inc. UP-2000(+) boards

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
   -------------------------------------------------------------------------
*/

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/i2c-algo-emma2rh.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/emma2rh/emma2rh.h>

#define I2C_EMMA2RH "emma2rh-iic"
static int i2c_debug = 0;

/* ----- global defines -----------------------------------------------	*/
#define DEB(x)	if (i2c_debug>=1) x
#define DEB2(x) if (i2c_debug>=2) x
#define DEB3(x) if (i2c_debug>=3) x
#define DEBE(x)	x		/* error messages */

struct i2c_drvdata {
	struct i2c_algo_emma_data alg;
	struct i2c_adapter adap;
	spinlock_t lock;
	atomic_t pending;
	u32 base;
	int irq;
	int clock;
	int own;
	wait_queue_head_t wait;
};

/* ----- local functions ----------------------------------------------	*/
static void i2c_emma_setbyte(void *data, int ctl, int val)
{
	int address = ((struct i2c_drvdata *)data)->base + ctl;

	DEB3(printk
	     (KERN_DEBUG "i2c_emma_setbyte: Write 0x%08x 0x%08x\n", address,
	      val));
	__raw_writel(val, (void *)address);
}

static int i2c_emma_getbyte(void *data, int ctl)
{
	int address = ((struct i2c_drvdata *)data)->base + ctl;
	int val = __raw_readl((void *)address);

	DEB3(printk
	     (KERN_DEBUG "i2c_emma_getbyte: Read 0x%08x 0x%08x\n", address,
	      val));
	return (val);
}

static int i2c_emma_getown(void *data)
{
	return (((struct i2c_drvdata *)data)->own);
}

static int i2c_emma_getclock(void *data)
{
	return (((struct i2c_drvdata *)data)->clock);
}



static int i2c_emma_reg(struct i2c_client *client)
{
	return 0;
}

static int i2c_emma_unreg(struct i2c_client *client)
{
	return 0;
}


static irqreturn_t i2c_emma_handler(int this_irq, void *dev_id,
				    struct pt_regs *regs)
{
	struct i2c_drvdata *dd = dev_id;

	DEB2(printk("i2c_emma_handler: status = 0x%08x\n",
		    __raw_readl((void *)(dd->base + I2C_EMMA_INT))));
	/* clear interrupt */
	__raw_writel(0, (void *)(dd->base + I2C_EMMA_INT));

	atomic_set(&dd->pending, 1);
	wake_up(&dd->wait);
	return IRQ_HANDLED;
}

static void i2c_emma_waitforpin(void *data)
{
	int timeout = 2;
	struct i2c_drvdata *dd = data;

	if (dd->irq >=0) {
		timeout = wait_event_timeout(dd->wait,
					     atomic_read(&dd->pending),
					     timeout * HZ);
		atomic_set(&dd->pending,0);
	} else
		udelay(100);
}

static int __devinit i2c_emma_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct i2c_drvdata *dd;
	int err = 0;
	struct resource *r;

	dd = kzalloc (sizeof *dd, SLAB_KERNEL);
	if (dd == NULL) {
		err = -ENOMEM;
		goto out;
	}
	dd->alg.data = dd;
	dd->alg.setemma = i2c_emma_setbyte;
	dd->alg.getemma = i2c_emma_getbyte;
	dd->alg.getown = i2c_emma_getown;
	dd->alg.getclock = i2c_emma_getclock;
	dd->alg.waitforpin = i2c_emma_waitforpin;
	dd->alg.udelay = 80;
	dd->alg.mdelay = 80;
	dd->alg.timeout = 200;

	strcpy(dd->adap.name, dev->bus_id);
	dd->adap.id = 0x00;
	dd->adap.algo = NULL;
	dd->adap.algo_data = &dd->alg;
	dd->adap.client_register = i2c_emma_reg;
	dd->adap.client_unregister = i2c_emma_unreg;

	spin_lock_init(&dd->lock);

	atomic_set(&dd->pending,0);
	init_waitqueue_head(&dd->wait);

	dev_set_drvdata(dev, dd);

	r = platform_get_resource(pdev, 0, 0);
	/* get resource of type '0' with #0 */

	if (!r) {
		printk("Cannot get resource\n");
		err = -ENODEV;
		goto out_free;
	}
	dd->base = r->start;

	dd->irq = platform_get_irq(pdev,0);
	dd->clock = FAST397;
	dd->own = 0x40 + pdev->id * 4;

	err = request_irq(dd->irq, i2c_emma_handler, 0, dev->bus_id, dd);
	if (err < 0)
		goto out_free;

	if ((err = i2c_emma_add_bus(&dd->adap)) < 0)
		goto out_irq;

	return 0;
out_irq:
	free_irq(dd->irq, dev->bus_id);
out_free:
	kfree(dd);
out:
	return err;
}

static int __devexit i2c_emma_remove (struct device *dev)
{
	struct i2c_drvdata* dd = dev_get_drvdata(dev);

	if (dd) {
		disable_irq(dd->irq);
		free_irq(dd->irq, dev->bus_id);
		i2c_emma_del_bus(&dd->adap);
		kfree(dd);
	}
	return 0;
}

static struct device_driver i2c_emma_driver = {
	.bus = &platform_bus_type,
	.name = I2C_EMMA2RH,
	.probe = i2c_emma_probe,
	.remove = i2c_emma_remove,
};

static int __init i2c_emma_init(void)
{
	return driver_register(&i2c_emma_driver);
}

static void __exit i2c_emma_exit(void)
{
	driver_unregister(&i2c_emma_driver);
}

MODULE_AUTHOR("NEC Electronics Corporation <www.necel.com>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for EMMA2RH I2C bus adapter");
MODULE_LICENSE("GPL");

module_param(i2c_debug, int, S_IRUGO | S_IWUSR);
module_init(i2c_emma_init);
module_exit(i2c_emma_exit);
