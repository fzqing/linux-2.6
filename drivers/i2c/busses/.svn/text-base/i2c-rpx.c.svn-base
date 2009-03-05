/*
 * Embedded Planet RPX Lite MPC8xx CPM I2C interface.
 * Copyright (c) 1999 Dan Malek (dmalek@jlc.net).
 *
 * moved into proper i2c interface;
 * Brad Parker (brad@heeltoe.com)
 *
 * RPX lite specific parts of the i2c interface
 * Update:  There actually isn't anything RPXLite-specific about this module.
 * This should work for most any 8xx board.  The console messages have been 
 * changed to eliminate RPXLite references.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-8xx.h>
#include <asm/mpc8xx.h>
#include <asm/commproc.h>

struct m8xx_i2c {
	char *base;
	struct device *dev;
	struct i2c_adapter adap;
	struct i2c_algo_8xx_data *algo_8xx;
};

static struct i2c_algo_8xx_data rpx_data;

static struct i2c_adapter rpx_ops = {
	.owner		= THIS_MODULE,
	.name		= "m8xx",
	.id		= I2C_HW_MPC8XX_EPON,
	.algo_data	= &rpx_data,
};

static void rpx_iic_init(struct m8xx_i2c *i2c)
{
	volatile cpm8xx_t *cp;
	volatile immap_t *immap;
	struct resource *r;
	struct i2c_algo_8xx_data *data = i2c->algo_8xx;
	struct platform_device *pdev = to_platform_device(i2c->dev);

	immap = (immap_t *)IMAP_ADDR;	/* pointer to internal registers */
	cp = cpmp;		/* pointer to Communication Processor */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pram");
	data->iip = (void *)r->start;

	/* Check for and use a microcode relocation patch.
	 */
	if ((data->reloc = data->iip->iic_rpbase))
		data->iip = (iic_t *)&cp->cp_dpmem[data->iip->iic_rpbase];
		
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	data->i2c = (void *)r->start;

	data->cp = cp;

	/* Allocate space for two transmit and two receive buffer
	 * descriptors in the DP ram.
	 */
	data->dp_addr = cpm_dpalloc(sizeof(cbd_t) * 4, 8);
}


static int i2c_rpx_probe(struct device *device)
{
	int result = 0;
	struct m8xx_i2c *i2c;
	struct platform_device *pdev = to_platform_device(device);

	if (!(i2c = kmalloc(sizeof(*i2c), GFP_KERNEL))) {
		return -ENOMEM;
	}
	memset(i2c, 0, sizeof(*i2c));
	i2c->dev = device;
	i2c->algo_8xx = &rpx_data;
	
	rpx_iic_init(i2c);

	dev_set_drvdata(device, i2c);

	i2c->adap = rpx_ops;
	i2c_set_adapdata(&i2c->adap, i2c);
	i2c->adap.dev.parent = &pdev->dev;

	if ((result = i2c_8xx_add_bus(&rpx_ops) < 0)) {
		printk(KERN_ERR "i2c-rpx: Unable to register with I2C\n");
		kfree(i2c);
	}

	return result;
}


static int i2c_rpx_remove(struct device *device)
{
	struct m8xx_i2c *i2c = dev_get_drvdata(device);

	i2c_8xx_add_bus(&i2c->adap);
	dev_set_drvdata(device, NULL);

	kfree(i2c);
	return 0;
}
	

/* Structure for a device driver */
static struct device_driver i2c_rpx_driver = {
	.name = "fsl-cpm-i2c",
	.bus = &platform_bus_type,
	.probe = i2c_rpx_probe,
	.remove = i2c_rpx_remove,
};

static int __init i2c_rpx_init(void)
{
	return driver_register(&i2c_rpx_driver);
}

static void __exit i2c_rpx_exit(void)
{
	driver_unregister(&i2c_rpx_driver);
}

module_init(i2c_rpx_init);
module_exit(i2c_rpx_exit);

MODULE_AUTHOR("Dan Malek <dmalek@jlc.net>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for MPC8xx boards");
