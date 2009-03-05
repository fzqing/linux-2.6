/*
 * linux/drivers/i2c/chips/ds1338.c
 *
 * Created by ripping apart ds1337 driver by James Chapman 
 *
 * Copyright (C) 2005 Sekhar Nori
 * Copyright (C) 2005 James Chapman <jchapman@katalix.com>
 *
 * based on linux/drivers/acron/char/pcf8583.c
 * Copyright (C) 2000 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for Dallas Semiconductor DS1338 real time clock chip
 */
	
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-sensor.h>
#include <linux/string.h>
#include <linux/rtc.h>        /* get the user-level API */
#include <linux/bcd.h>
#include <linux/time.h>
#include <asm/time.h>
	
#define DS1338_REG_HOUR 	2
#define DS1338_REG_DAY  	3
#define DS1338_REG_DATE 	4
#define DS1338_REG_MONTH 	5
#define DS1338_REG_CONTROL 	7
#define DS1338_GET_DATE 	0
#define DS1338_SET_DATE 	1
	
#define EPOCH	1970
#define DEBUG
	
/*
 * Functions declaration
 */
static unsigned short normal_i2c[] = { 0x68, I2C_CLIENT_END };
static unsigned int normal_isa[] = { I2C_CLIENT_ISA_END };
	
SENSORS_INSMOD_1(ds1338);
	
static int ds1338_attach_adapter(struct i2c_adapter *adapter);
static int ds1338_detect(struct i2c_adapter *adapter, int address, int kind);
static void ds1338_init_client(struct i2c_client *client);
static int ds1338_detach_client(struct i2c_client *client);
static int ds1338_command(struct i2c_client *client, unsigned int cmd, void *arg);
	
/*
 * Driver data (common to all clients)
 */
static struct i2c_driver ds1338_driver = {
	.owner		= THIS_MODULE,
	.name		= "ds1338",
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= ds1338_attach_adapter,
	.detach_client	= ds1338_detach_client,
	.command	= ds1338_command,
};
	
/*
 * Client data (each client gets its own)
 */
struct ds1338_data {
	struct i2c_client client;
	struct list_head list;
};
	
/*
 * Internal variables
 */
static LIST_HEAD(ds1338_clients);
	
static inline int ds1338_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int err;
	struct i2c_msg msg[1];
	
	if (!client->adapter)
		return -ENODEV;
	
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = &reg;        
	
	err = i2c_transfer(client->adapter, msg, 1);
	
	if(err >= 0) {
		msg->len = 1; 
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);    
	}
	
	*value = msg->buf[0];
	
	return 0;
}
	
static int ds1338_get_datetime(struct i2c_client *client, struct rtc_time *dt)
{
	int result;
	u8 buf[7];
	u8 val;
	struct i2c_msg msg[2];
	u8 offs = 0;
	
	if (!dt) {
		return -EINVAL;
	}
	
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &offs;
	
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = &buf[0];
	
	/* This is ugly. The i2c adapter did not support sending two messages
	 * in the same transfer. Hence this workaround. TODO: Update this code
	 * once that gets fixed.
	 */
	result = i2c_transfer(client->adapter, &msg[0], 1);
	
	if(result >= 0)
		result = i2c_transfer(client->adapter, &msg[1], 1);
	
	if (result >= 0) {
		dt->tm_sec = BCD2BIN(buf[0]);
		dt->tm_min = BCD2BIN(buf[1]);
		val = buf[2] & 0x3f;
		dt->tm_hour = BCD2BIN(val);
		dt->tm_wday = BCD2BIN(buf[3]) - 1; /* chip does 1-7, POSIX needs 0-6 */
		dt->tm_mday = BCD2BIN(buf[4]); 
		val = buf[5] & 0x7f;
		dt->tm_mon = BCD2BIN(val) - 1; /* chip does 1-12, POSIX needs 0-11 */
	
		dt->tm_year = EPOCH + BCD2BIN(buf[6]);
	
		return 0;
	}
	
	return -EIO;
}
	
static int ds1338_set_datetime(struct i2c_client *client, struct rtc_time *dt)
{
	int result;
	u8 buf[8];
	struct i2c_msg msg[1];
	int val;
	
	if (!dt) {
		return -EINVAL;
	}
	
	buf[0] = 0;        /* reg offset */
	dt->tm_sec &= 0x7f; /* make sure CH bit is unset - the clock starts */
	buf[1] = BIN2BCD(dt->tm_sec);
	buf[2] = BIN2BCD(dt->tm_min);
	buf[3] = BIN2BCD(dt->tm_hour);     
	buf[3] &= ~(1 << 6); /* ensure 24 hour mode */    
	dt->tm_wday += 1;
	buf[4] = BIN2BCD(dt->tm_wday);
	buf[5] = BIN2BCD(dt->tm_mday);
	dt->tm_mon += 1;
	buf[6] = BIN2BCD(dt->tm_mon);    
	
	val = dt->tm_year;
	if((val < EPOCH) || (val > EPOCH + 99)) {
		val = (dt->tm_year % 100);
	} else {
		val -= EPOCH;
	}
	
	buf[7] = BIN2BCD(val);
	
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(buf);
	msg[0].buf = &buf[0];
	
	result = i2c_transfer(client->adapter, msg, 1);
	if (result >= 0)
		return 0;
	
	return -EIO;
}
	
static int ds1338_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case DS1338_GET_DATE:
		return ds1338_get_datetime(client, arg);

	case DS1338_SET_DATE:
		return ds1338_set_datetime(client, arg);
	
	default:
		return -EINVAL;
	}
}
	
/*
 * Public API for access to specific device. Useful for low-level
 * RTC access from kernel code.
 */
int ds1338_do_command(int bus, int cmd, void *arg)
{
	struct list_head *walk;
	struct list_head *tmp;
	struct ds1338_data *data;
	
	list_for_each_safe(walk, tmp, &ds1338_clients) {
		data = list_entry(walk, struct ds1338_data, list);
		if (data->client.adapter->nr == bus)
			return ds1338_command(&data->client, cmd, arg);
	}
	
	return -ENODEV;
}
	
static int ds1338_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_detect(adapter, &addr_data, ds1338_detect);
}
	
/*
 * The following function does more than just detection. If detection
 * succeeds, it also registers the new chip.
 */
static int ds1338_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct ds1338_data *data;
	int err = 0;
	const char *name = "";
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_I2C))
		goto exit;
	
	if (!(data = kmalloc(sizeof(struct ds1338_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct ds1338_data));
	INIT_LIST_HEAD(&data->list);
	
	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ds1338_driver;
	new_client->flags = 0;
	
	/*
	 * Now we do the remaining detection. A negative kind means that
	 * the driver was loaded with no force parameter (default), so we
	 * must both detect and identify the chip. A zero kind means that
	 * the driver was loaded with the force parameter, the detection
	 * step shall be skipped. A positive kind means that the driver
	 * was loaded with the force parameter and a given kind of chip is
	 * requested, so both the detection and the identification steps
	 * are skipped.
	 *
	 * For detection, we read registers that are most likely to cause
	 * detection failure, i.e. those that have more bits with fixed
	 * or reserved values.
	 */
	
	/* Default to an DS1338 if forced */
	if (kind == 0)
		kind = ds1338;
	
	if (kind < 0) {        /* detection and identification */
		u8 data;
	
		/* Check for a valid day register value */
		if ((ds1338_read(new_client, DS1338_REG_DAY, &data) < 0) || (data == 0) || (data & 0xf8)) {
			goto exit_free;
		}
	
		/* Check for a valid date register value */
		if ((ds1338_read(new_client, DS1338_REG_DATE, &data) < 0) ||
		   (data == 0) || (data & 0xc0) || ((data & 0x0f) > 9) ||
		   (data >= 0x32)) {
			goto exit_free;
		}
	
		/* Check for a valid month register value */
		if ((ds1338_read(new_client, DS1338_REG_MONTH, &data) < 0) ||
		   (data == 0) || (data & 0x60) || ((data & 0x0f) > 9) ||
		   ((data >= 0x13) && (data <= 0x19))) {
			goto exit_free;
		}
	
		/* Check that control register bits 2,3 and 6 are zero */
		if ((ds1338_read(new_client, DS1338_REG_CONTROL, &data) < 0) || (data & 0x4C)) {
			goto exit_free;
		}
	
		kind = ds1338;
	}
	
	if (kind == ds1338)
		name = "ds1338";
	
	/* We can fill in the remaining client fields */
	strlcpy(new_client->name, name, I2C_NAME_SIZE);
	
	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;
	
	/* Initialize the DS1338 chip */
	ds1338_init_client(new_client);
	
	/* Add client to local list */
	list_add(&data->list, &ds1338_clients);
	
	return 0;
	
exit_free:
	kfree(data);
exit:
	return err;
}
	
static void ds1338_init_client(struct i2c_client *client)
{
}
static int ds1338_detach_client(struct i2c_client *client)
{
	int err;
	struct ds1338_data *data = i2c_get_clientdata(client);
	
	if ((err = i2c_detach_client(client))) {
		return err;
	}
	
	list_del(&data->list);
	kfree(data);
	return 0;
}
	
static int __init ds1338_init(void)
{
	return i2c_add_driver(&ds1338_driver);
}
	
static void __exit ds1338_exit(void)
{
	i2c_del_driver(&ds1338_driver);
}
	
MODULE_AUTHOR("Sekhar Nori <nsekhar@ti.com>");
MODULE_DESCRIPTION("DS1338 RTC driver");
MODULE_LICENSE("GPL");
	
EXPORT_SYMBOL_GPL(ds1338_do_command);
	
module_init(ds1338_init);
module_exit(ds1338_exit);
