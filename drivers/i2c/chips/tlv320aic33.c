/*
 * drivers/i2c/chips/tlv320aic33.c
 *
 * Copyright (C) 2005 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>


#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <asm/arch/cpu.h>


#define I2C_AIC33_REG_SIZE                102
/**< Number of registers in AIC33                                           */

struct aic33_serial_bus_ops {
	unsigned char version;
	int (*init)(void);
	void (*cleanup)(void);
	int (*read)(u8 reg, u8 *val);
	int (*write)(u8 reg, u8 val);
};

struct aic33_i2c_param {
	/* I2C parameters */
	struct i2c_client client;
	struct i2c_driver driver;
};

/* Global structure to store the i2c driver info */
static struct aic33_i2c_param aic33_i2c_dev;

static int
aic33_i2c_read_reg(u8 reg, u8 *val)
{
	int err;
	struct i2c_client *client = &aic33_i2c_dev.client;

	struct i2c_msg msg[1];
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

/*Rishi*/
	data[0]=reg;
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
/*Rishi*/
	//*data = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = *data;
		return 0;
	}

	return err;
}

static int
aic33_i2c_write_reg(u8 reg, u8  val)
{
	int err;
	struct i2c_client *client = &aic33_i2c_dev.client;

	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = reg;
	data[1] = val;
	err = i2c_transfer(client->adapter, msg, 1);

	//printk(KERN_INFO "i2c write: error = %d\n", err);

	if (err >= 0)
		return 0;
	else
		printk(KERN_INFO "i2c write: error = %d\n", err);

	return err;
}

static int
aic33_i2c_attach_client(struct i2c_adapter *adap, int addr)
{
	struct aic33_i2c_param *aic33_i2c_if = &aic33_i2c_dev;
	struct i2c_client *client = &aic33_i2c_if->client;
	int err;

	if (client->adapter)
		return -EBUSY;  /* our client is already attached */

	client->addr = addr;
	client->flags = I2C_CLIENT_ALLOW_USE;
	client->driver = &aic33_i2c_if->driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	return 0;
}

static int
aic33_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV; /* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;
	return err;
}

static int
aic33_i2c_probe_adapter(struct i2c_adapter *adap)
{
	/* i2c client can be up to 4 devices with device addresses 0x18, 0x19, 0x1A, 0x1B*/
	if (cpu_is_davinci_dm6467())
		return aic33_i2c_attach_client(adap, 0x18);
	else
		return aic33_i2c_attach_client(adap, 0x1B);
}

static int
aic33_i2c_init(void)
{
	int err;
	struct i2c_driver *driver = &aic33_i2c_dev.driver;

	driver->owner = THIS_MODULE;
	strlcpy(driver->name, "Audio Codec I2C driver", sizeof(driver->name));
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = aic33_i2c_probe_adapter;
	driver->detach_client = aic33_i2c_detach_client;

	err = i2c_add_driver(driver);
	if (err) {
		printk(KERN_ERR "Failed to register Audio Codec I2C client.\n");
		return err;
	}
	return 0;
}

static void aic33_i2c_cleanup (void)
{
	struct i2c_driver *driver = &aic33_i2c_dev.driver;

	i2c_detach_client(&aic33_i2c_dev.client);
	i2c_del_driver(driver);
	aic33_i2c_dev.client.adapter = NULL;

	return;
}

struct aic33_serial_bus_ops aic33_i2c_fops = {
	version : 0x01,
	init    : aic33_i2c_init,
	cleanup : aic33_i2c_cleanup,
	read    : aic33_i2c_read_reg,
	write   : aic33_i2c_write_reg,
};


int
 tlv320aic33_write_value(u8 reg, u16 value)
{

	u8     data1, data2;
	mdelay(10);

	data2 = value & 0xFF;
	data1 = reg;

     	if(aic33_i2c_fops.write(data1, data2))
	{
	        printk(KERN_INFO "tlv320aic33_write_value: aic33 write failed\n");
	        return -1;
	}

	return 0;
}

int
 tlv320aic33_read_value(u8 reg, u8 *value)
{
#if 0
	printk(KERN_INFO "%x %x \n", reg, *value);

	if(aic33_i2c_fops.init())
	{
	      	printk(KERN_INFO " aic33 init failure\n");
	      	return -1;
	}

	if(aic33_i2c_fops.read(reg, value))
	{
	      	printk(KERN_INFO "aic33 read failed\n");
	      	aic33_i2c_fops.cleanup();
	      	return -1;
	}

	aic33_i2c_fops.cleanup();
#endif
	return 0;
}

EXPORT_SYMBOL(tlv320aic33_write_value);
EXPORT_SYMBOL(tlv320aic33_read_value);

module_init(aic33_i2c_init);
module_exit(aic33_i2c_cleanup);
