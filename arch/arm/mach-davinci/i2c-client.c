/*
 *  linux/drivers/davinci/i2c-davinci-client.c
 *
 * Copyright (C) 2006 Texas Instruments Inc
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
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/arch/i2c-client.h>
#include <asm/arch/cpu.h>
#include <asm/arch/io.h>
#include <asm/arch/mux.h>

static unsigned long initialized;
static struct semaphore expander_sema;

struct davinci_bus_ops {
	unsigned char version;
	int (*init) (void);
	void (*cleanup) (void);
	int (*read) (u8 size, u8 * val, u16 addr);
	int (*write) (u8 size, u8 * val, u16 addr);
};

struct davinci_i2c_param {
	struct i2c_client client;
	struct i2c_driver driver;
};

static struct davinci_i2c_param davinci_i2c_dev;

static int davinci_i2c_client_init(void);

/* This function is used for internal initialization */
int davinci_i2c_read(u8 size, u8 * val, u16 client_addr)
{
	int err;
	struct i2c_client *client = &davinci_i2c_dev.client;

	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	if (unlikely(!initialized))
		davinci_i2c_client_init();

	msg->addr = client_addr;
	msg->flags = I2C_M_RD;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		return 0;
	}

	return err;
}

EXPORT_SYMBOL(davinci_i2c_read);

/* This function is used for internal initialization */
int davinci_i2c_write(u8 size, u8 * val, u16 client_addr)
{
	int err;
	struct i2c_client *client = &davinci_i2c_dev.client;

	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	if (unlikely(!initialized))
		davinci_i2c_client_init();

	msg->addr = client_addr;
	msg->flags = 0;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	return err;
}

EXPORT_SYMBOL(davinci_i2c_write);

int davinci_i2c_expander_op(u16 client_addr, u35_expander_ops pin, u8 val)
{
	int err = 0;
	char cmd[4] = { 4, 6, 0x00, 0x09 };
	u8 data_to_u35 = 0;

	if (unlikely(!initialized))
		davinci_i2c_client_init();

	if (val > 1)
		return -1;

	down(&expander_sema);

	if (!cpu_is_davinci_dm6467()) {
		err = davinci_i2c_read(1, &data_to_u35, 0x3A);

		if (client_addr == 0x3A) {
			switch (pin) {
			case USB_DRVVBUS:
				if (val)
					data_to_u35 |= val;
				else {
					data_to_u35 &= (val | 0xFE);
				}
				break;
			case VDDIMX_EN:
				if (val)
					data_to_u35 |= (val << 1);
				else {
					data_to_u35 &= (val | 0xFD);
				}
				break;
			case VLYNQ_ON:
				if (val)
					data_to_u35 |= (val << 2);
				else {
					data_to_u35 &= (val | 0xFB);
				}
				break;
			case CF_RESET:
				if (val)
					data_to_u35 |= (val << 3);
				else {
					data_to_u35 &= (val | 0xF7);
				}
				break;
			case WLAN_RESET:
				if (val)
					data_to_u35 |= (val << 5);
				else {
					data_to_u35 &= (val | 0xDF);
				}
				break;
			case ATA_SEL:
				if (val)
					data_to_u35 |= (val << 6);
				else {
					data_to_u35 &= (val | 0xBF);
				}
				break;
			case CF_SEL:
				davinci_i2c_write(4, cmd, 0x23);
				if (val)
					data_to_u35 |= (val << 7);
				else {
					data_to_u35 &= (val | 0x7F);
				}
				break;
			default:
				break;
			}
		} else {
			printk(KERN_WARNING "Only IO Expander at address 0x3A "
			       "is supported\n");
			up(&expander_sema);
			return -1;
		}

		err = davinci_i2c_write(1, &data_to_u35, 0x3A);
	} else {
		err = davinci_i2c_read(1, &data_to_u35, 0x3A);

		if (client_addr == 0x3A) {
			switch (pin) {
			case ATA_SEL_DM646X:
				if (val)
					data_to_u35 |= val;
				else {
					data_to_u35 &= (val | 0xFE);
				}
				break;
			case ATA_PWD_DM646X:
				if (val)
					data_to_u35 |= (val << 1);
				else {
					data_to_u35 &= (val | 0xFD);
				}
				break;
			case VSCALE_ON_DM646X:
				if (val)
					data_to_u35 |= (val << 2);
				else {
					data_to_u35 &= (val | 0xFB);
				}
				break;
			case VLYNQ_RESET_DM646X:
				if (val)
					data_to_u35 |= (val << 3);
				else {
					data_to_u35 &= (val | 0xF7);
				}
				break;
			case I2C_INT_DM646X:
				if (val)
					data_to_u35 |= (val << 6);
				else {
					data_to_u35 &= (val | 0xBF);
				}
				break;
			case USB_FB_DM646X:
				if (val)
					data_to_u35 |= (val << 7);
				else {
					data_to_u35 &= (val | 0x7F);
				}
				break;
			case CIR_MOD_DM646X:
				if (val)
					data_to_u35 |= (val << 5);
				else {
					data_to_u35 &= (val | 0xDF);
				}
				break;
			case CIR_DEMOD_DM646X:
				if (val)
					data_to_u35 |= (val << 4);
				else {
					data_to_u35 &= (val | 0xEF);
				}
				break;
			default:
				break;
			}
		} else {
			printk(KERN_WARNING "Only IO Expander at address 0x3A "
			       "is supported\n");
			up(&expander_sema);
			return -1;
		}

		err = davinci_i2c_write(1, &data_to_u35, 0x3A);
	}
	up(&expander_sema);

	return err;
}

EXPORT_SYMBOL(davinci_i2c_expander_op);

static int davinci_i2c_attach_client(struct i2c_adapter *adap, int addr)
{
	struct davinci_i2c_param *davinci_i2c_if = &davinci_i2c_dev;
	struct i2c_client *client = &davinci_i2c_if->client;
	int err;
	u8 data_to_u35 = 0xf6;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	client->flags = I2C_CLIENT_ALLOW_USE;
	client->driver = &davinci_i2c_if->driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	err = davinci_i2c_write(1, &data_to_u35, 0x3A);

	return 0;
}

static int davinci_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;
	return err;
}

static int davinci_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return davinci_i2c_attach_client(adap, 0x3A);
}

static int davinci_i2c_client_init(void)
{
	int err;
	struct i2c_driver *driver = &davinci_i2c_dev.driver;

	if (likely(initialized))
		return 0;
	initialized = 1;

	init_MUTEX(&expander_sema);

	driver->owner = THIS_MODULE;
	strlcpy(driver->name, "Davinci I2C driver", sizeof(driver->name));
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = davinci_i2c_probe_adapter;
	driver->detach_client = davinci_i2c_detach_client;

	err = i2c_add_driver(driver);
	if (err) {
		printk(KERN_ERR "Failed to register Davinci I2C client.\n");
		return err;
	}

	return 0;
}

static void davinci_i2c_client_cleanup(void)
{
	i2c_detach_client(&davinci_i2c_dev.client);
	davinci_i2c_dev.client.adapter = NULL;

	return;
}

module_init(davinci_i2c_client_init);
module_exit(davinci_i2c_client_cleanup);
