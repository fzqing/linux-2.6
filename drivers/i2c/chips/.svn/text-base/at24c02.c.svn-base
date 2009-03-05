/*
 * Copyright © 2005-2007 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version two of the
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.
 * Notwithstanding the foregoing, you understand and agree that this program
 * is provided by RMI .as is,. and without any warranties, whether express,
 * implied or statutory, including without limitation any implied warranty of
 * non-infringement, merchantability or fitness for a particular purpose.
 * In no event will RMI be liable for any loss of data, lost profits, cost
 * of procurement of substitute technology or services or for any direct,
 * indirect, incidental, consequential or special damages arising from the
 * use of this program, however caused.  Your unconditional agreement to
 * these terms and conditions is an express condition to, and shall be deemed
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x50, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_addr,
	.probe = ignore,
	.ignore = ignore,
	.probe_range = ignore,
	.ignore_range = ignore,
	.force = ignore,
	.normal_i2c_range = ignore,
};

/* Size of EEPROM in bits */
#define EEPROM_SIZE 256


struct chip_desc {
	char		name[10];
	u32		byte_len;		/* of 1..8 i2c addrs, total */
	u16		page_size;		/* for writes */
};

static ssize_t
at24c_read(struct kobject *kobj, char *buf, loff_t off, size_t count);

static struct bin_attribute at24c_attr = {
	.attr = {
		.name = "24c02",
		.mode = S_IRUGO,
		.owner = THIS_MODULE,
	},
	.size = EEPROM_SIZE,
	.read = at24c_read
};


/* Each client has this additional data */
struct at24c_data {
	struct i2c_client client;
	struct semaphore update_lock;
	u8 valid;			/* bitfield, bit!=0 if slice is valid */
	unsigned long last_updated[8];	/* In jiffies, 8 slices */
	u8 data[EEPROM_SIZE];		/* Register values */
	struct chip_desc	chip;
	struct bin_attribute    bin;
};


static int at24c_attach_adapter(struct i2c_adapter *adapter);
static int at24c_detect(struct i2c_adapter *adapter, int address, int kind);
static int at24c_detach_client(struct i2c_client *client);

/* This is the driver that will be inserted */
static struct i2c_driver at24c_driver = {
	.owner		= THIS_MODULE,
	.name		= "at24c",
	.id		= I2C_DRIVERID_AT24Cxx,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= at24c_attach_adapter,
	.detach_client	= at24c_detach_client,
};

static ssize_t
at24c_read(struct kobject *kobj, char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	struct at24c_data	*at24c;
	int i = 0, j = 0x0;
	uint32_t offset;
	unsigned char value;
	struct i2c_msg msg;
	ssize_t status;

	offset = (uint32_t)off;

	client = to_i2c_client(container_of(kobj, struct device, kobj));
	at24c = i2c_get_clientdata(client);

	if (unlikely(off >= at24c->bin.size))
		return 0;
	if ((off + count) > at24c->bin.size)
		count = at24c->bin.size - off;
	if (unlikely(!count))
		return count;

        /* read full and manage the offset later */
	msg.addr = client->addr;
	msg.len = 1;
	msg.buf = &value;
	msg.flags = I2C_M_RD;

	for(j = offset; j < (offset + count); j++)  {
		msg.buf[0] = j;
	  	status = i2c_transfer(client->adapter, &msg, 1);
		if(status < 0) {
			printk("\n\rI2c eeprom error in tx\n");
			return status;
		}
		    buf[i++] = value;
  	}

	return i;
}


static int at24c_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, at24c_detect);
}

/* This function is called by i2c_probe */
int at24c_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct at24c_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_EMUL))
		goto exit;

	if (!(data = kmalloc(sizeof(struct at24c_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct at24c_data));
	data->bin = at24c_attr;

	new_client = &data->client;
	memset(data->data, 0xff, EEPROM_SIZE);
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &at24c_driver;
	new_client->flags = 0;
      

	/* Fill in the remaining client fields */
	strlcpy(new_client->name, "at24c", I2C_NAME_SIZE);
	data->valid = 0;
	init_MUTEX(&data->update_lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_kfree;

	/* create the sysfs at24c file */
	sysfs_create_bin_file(&new_client->dev.kobj, &data->bin);

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int at24c_detach_client(struct i2c_client *client)
{
	int err;

	err = i2c_detach_client(client);
	if (err)
		return err;

	kfree(i2c_get_clientdata(client));

	return 0;
}

static int __init at24c_init(void)
{
	return i2c_add_driver(&at24c_driver);
}

static void __exit at24c_exit(void)
{
	i2c_del_driver(&at24c_driver);
}

module_init(at24c_init);
module_exit(at24c_exit);
