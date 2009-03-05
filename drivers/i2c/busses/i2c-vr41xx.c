/*
 * drivers/i2c/busses/i2c-vr41xx.c
 *
 * The NEC VR41XX series does not have an I2C controller, but some boards, such
 * as the NEC CMB-VR4133, use GPIO pins to create an I2C bus.
 *
 * Author: Wade Farnsworth <wfarnsworth@mvista.com>
 *
 * Copyright (c) 2006 MontaVista Software Inc.
 *
 * This is based on i2c-ixp4xx.c by Deepak Saxena <dsaxena@plexity.net>
 * Copyright (c) 2003-2004 MontaVista Software Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/config.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/delay.h>

#include <asm/vr41xx/giu.h>

static inline int vr41xx_scl_pin(void *data)
{
	return ((struct vr41xx_i2c_pins*)data)->scl_pin;
}

static inline int vr41xx_sda_pin(void *data)
{
	return ((struct vr41xx_i2c_pins*)data)->sda_pin;
}

static void vr41xx_bit_setscl(void *data, int val)
{
	vr41xx_gpio_set_pin(vr41xx_scl_pin(data), val ? GPIO_DATA_HIGH :
							GPIO_DATA_LOW);
}

static void vr41xx_bit_setsda(void *data, int val)
{
	vr41xx_gpio_set_direction(vr41xx_sda_pin(data), GPIO_OUTPUT);
	if (val) {
		vr41xx_gpio_set_pin(vr41xx_sda_pin(data), GPIO_DATA_HIGH);
		vr41xx_gpio_set_direction(vr41xx_sda_pin(data), GPIO_INPUT);
	} else
		vr41xx_gpio_set_pin(vr41xx_sda_pin(data), GPIO_DATA_LOW);
}

static int vr41xx_bit_getsda(void *data)
{
	return vr41xx_gpio_get_pin(vr41xx_sda_pin(data));
}

struct vr41xx_i2c_data {
	struct vr41xx_i2c_pins *gpio_pins;
	struct i2c_adapter adapter;
	struct i2c_algo_bit_data algo_data;
};

static int vr41xx_i2c_remove(struct device *dev)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct vr41xx_i2c_data *drv_data = dev_get_drvdata(&plat_dev->dev);

	dev_set_drvdata(&plat_dev->dev, NULL);

	i2c_bit_del_bus(&drv_data->adapter);

	kfree(drv_data);

	return 0;
}

static int vr41xx_i2c_probe(struct device *dev)
{
	int err;
	struct platform_device *plat_dev = to_platform_device(dev);
	struct vr41xx_i2c_pins *gpio = plat_dev->dev.platform_data;
	struct vr41xx_i2c_data *drv_data =
		kmalloc(sizeof(struct vr41xx_i2c_data), GFP_KERNEL);

	if(!drv_data)
		return -ENOMEM;

	memset(drv_data, 0, sizeof(struct vr41xx_i2c_data));
	drv_data->gpio_pins = gpio;

	/*
	 * We could make a lot of these structures static, but
	 * certain platforms may have multiple GPIO-based I2C
	 * buses for various device domains, so we need per-device
	 * algo_data->data.
	 */
	drv_data->algo_data.data = gpio;
	drv_data->algo_data.setsda = vr41xx_bit_setsda;
	drv_data->algo_data.setscl = vr41xx_bit_setscl;
	drv_data->algo_data.getsda = vr41xx_bit_getsda;
	drv_data->algo_data.udelay = 40;
	drv_data->algo_data.mdelay = 40;
	drv_data->algo_data.timeout = 400;
	drv_data->adapter.id = I2C_HW_B_VR41XX,
	drv_data->adapter.algo_data = &drv_data->algo_data,

	drv_data->adapter.dev.parent = &plat_dev->dev;

	vr41xx_gpio_set_direction(gpio->scl_pin, GPIO_OUTPUT);
	vr41xx_gpio_set_pin(gpio->scl_pin, GPIO_DATA_HIGH);
	vr41xx_gpio_set_direction(gpio->sda_pin, GPIO_OUTPUT);
	vr41xx_gpio_set_pin(gpio->sda_pin, GPIO_DATA_HIGH);

	if ((err = i2c_bit_add_bus(&drv_data->adapter) != 0)) {
		printk(KERN_ERR "ERROR: Could not install %s\n", dev->bus_id);

		kfree(drv_data);
		return err;
	}

	dev_set_drvdata(&plat_dev->dev, drv_data);

	return 0;
}

static struct device_driver vr41xx_i2c_driver = {
	.name	= "VR41XX-I2C",
	.bus	= &platform_bus_type,
	.probe	= vr41xx_i2c_probe,
	.remove	= vr41xx_i2c_remove,
};

static int __init vr41xx_i2c_init(void)
{
	return driver_register(&vr41xx_i2c_driver);
}

static void __exit vr41xx_i2c_exit(void)
{
	driver_unregister(&vr41xx_i2c_driver);
}

module_init(vr41xx_i2c_init);
module_exit(vr41xx_i2c_exit);

MODULE_DESCRIPTION("GPIO-based I2C adapter for VR41xx systems");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wade Farnsworth <wfarnsworth@mvista.com>");
