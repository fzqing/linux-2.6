/*
 * Copyright © 200-2007 Raza Microelectronics, Inc. (.RMI.)
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

/*
 * linux/drivers/i2c/chips/max6657.c
 *
 * based on drivers/char/x1226-rtc.c
 * Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 * 2002-2003 (c) MontaVista Software, Inc.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/time.h>

#include <asm/rmi/sim.h>

/* #define 	DEBUG_MAX6657 */


#ifdef DEBUG_MAX6657		
#define dbg(fmt, args...) 	printk(KERN_DEBUG "%s: " fmt, __func__, ## args)
#else
#define dbg(fmt, args...)
#endif

#define err(format, arg...) 	printk(KERN_ERR ": " format , ## arg)
#define info(format, arg...) 	printk(KERN_INFO ": " format , ## arg)

#define DEVID_TEMP    	   	0x4c
#define MAX6657_TEMP_BASE    	0x00
#define XLR_I2C_DRIVERID_MAX6657   	0x01

static struct 	i2c_driver max6657_driver;
static struct 	i2c_client *this_client = NULL;
static int 	max6657_use_count = 0;
static int 	temp_read_proc(char *page, char **start, off_t off, int count, 
		int *eof, void *data);

static int
max6657_read(struct i2c_client *client, u16 offset, u8 * buf, int len)
{
	int ret = 0, sz = 0;
	struct i2c_msg msg;

	for(sz = 0; sz < len ; sz++) {
		msg.addr = client->addr;		/* 0x4c */
		msg.flags = I2C_M_RD;			/* I2C Read Command */
		msg.len =  1;			        /* bytes */
		buf[sz] = offset + sz;
		msg.buf =  &buf[sz];			/* Store the result here */

		if ((ret = i2c_transfer(client->adapter, &msg, 1)) < 0) {
			err("i2c_transfer failed, ret=%d\n", ret);
			ret = -ENXIO;
		}
	}
	
	return ret;
}

static int
max6657_get_value(struct i2c_client *client, unsigned char *value, int offset)
{
	int ret;

	if (!client) return -ENXIO;

	/* read TEMP registers */
	if ((ret = max6657_read(client, offset, value, sizeof (char))) < 0) {
		dbg("couldn't read TEMP\n");
		return ret;
	}

	dbg("IN: value=%0x\n", *value);

	return 0;
}

static int max6657_probe(struct i2c_adapter *adap)
{
	int ret;
	unsigned char temp;
	if (this_client != NULL)
		return -EBUSY;

	this_client = kmalloc(sizeof (*this_client), GFP_KERNEL);
	if (this_client == NULL) {
		return -ENOMEM;
	}

	memset(this_client, 0, sizeof(*this_client));

	strcpy(this_client->name, "MAX6657");
/* 	this_client->id 	= max6657_driver.id; */
	this_client->addr 	= DEVID_TEMP;
	this_client->adapter 	= adap;
	this_client->driver 	= &max6657_driver;
	this_client->flags 	= 0;
	/*
	 * use max6657_get_value() to probe for a MAX6657 on this bus.
	 */
	if ((ret = max6657_get_value(this_client, &temp, MAX6657_TEMP_BASE+1)) < 0) {
		kfree(this_client);
		this_client = NULL;
		return ret;
	}

	info("Found MAX6657 on %s\n", adap->name);
	printk("XLR Chip temperature is %d degrees Celsius\n", temp);

	/* attach it. */
	return i2c_attach_client(this_client);
}

static int
max6657_detach(struct i2c_client *client)
{
	i2c_detach_client(client);

	if (this_client != NULL) {
		kfree(this_client);
		this_client = NULL;
	}

	return 0;
}

int
temp_open(struct inode *minode, struct file *mfile)
{
	/*if(MOD_IN_USE) */
	if (max6657_use_count > 0) {
		return -EBUSY;
	}
/* 	MOD_INC_USE_COUNT; */
	++max6657_use_count;
	return 0;
}

int
temp_release(struct inode *minode, struct file *mfile)
{
/* 	MOD_DEC_USE_COUNT; */
	--max6657_use_count;
	return 0;
}

static int
temp_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	return -EINVAL;
}

static struct i2c_driver max6657_driver = {
	.owner		= THIS_MODULE,
	.name   = "max6657",
	.id		= XLR_I2C_DRIVERID_MAX6657,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter = max6657_probe,
	.detach_client	= max6657_detach,
};

static struct file_operations temp_fops = {
	.owner 		= THIS_MODULE,
	.ioctl 		= temp_ioctl,
	.open 		= temp_open,
	.release 	= temp_release,
};

static struct miscdevice max6657_miscdev = {
	TEMP_MINOR,
	"max6657",
	&temp_fops
};

static int __init max6657_init(void)
{
	int ret;

	printk("Registering XLR I2C based Temperature Sensor driver...\n");
	ret = i2c_add_driver(&max6657_driver);
	if (ret) {
		printk("Unable to register xlr temperaturn sensor driver!\n");
		return ret;		
	}	

	ret = misc_register(&max6657_miscdev);
	if (ret) {
		err("Register misc driver failed, errno is %d\n", ret);
		ret = i2c_del_driver(&max6657_driver);
		if (ret) {
			err("Unregister I2C driver failed, errno is %d\n", ret);
		}
		return ret;
	}

	create_proc_read_entry("driver/max6657", 0, 0, temp_read_proc, NULL);

	return 0;
}

static void __exit max6657_exit(void)
{
	remove_proc_entry("driver/max6657", NULL);
	misc_deregister(&max6657_miscdev);
	i2c_del_driver(&max6657_driver);
}


module_init(max6657_init);
module_exit(max6657_exit);

/*
 * Info exported via "/proc/driver/max6657".
 */

static int
temp_proc_output(char *buf)
{
	char *p;
	unsigned char temp;
	int ret;

	if ((ret = max6657_get_value(this_client, &temp, 1)) < 0)
		return ret;

	p = buf;
	/*
	 * There is no way to tell if the luser has the TEMP set for local
	 * time or for Universal Standard Time (GMT). Probably local though.
	 */
	p += sprintf(p, "temperature: %d C\n", temp);
	return p - buf;
}

static int
temp_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = temp_proc_output(page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

MODULE_AUTHOR("RMI");
MODULE_DESCRIPTION("Maxim max6657 Driver");
MODULE_LICENSE("GPL");
