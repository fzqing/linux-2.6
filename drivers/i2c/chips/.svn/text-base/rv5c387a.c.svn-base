/*
 * drivers/char/rv5c387a.c
 *
 * I2C client/driver for the Ricoh RV5C387A Real-Time Clock chip.
 *
 * Author: Wade Farnsworth <wfarnsworth@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>

#include <asm/time.h>

#define RV5C387A_DRV_NAME	"rv5c387a"

static unsigned short ignore [] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x32, I2C_CLIENT_END };

static unsigned short rtc_attached = 0;

static struct i2c_driver rv5c387a_driver;
static struct i2c_client *save_client;

static spinlock_t rv5c387a_lock;

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.normal_i2c_range	= ignore,
	.probe			= ignore,
	.probe_range		= ignore,
	.ignore			= ignore,
	.ignore_range		= ignore,
	.force			= ignore,
};

unsigned long rv5c387a_rtc_get_time(void)
{
	s32 sec, min, hour, wday, mday, mon, year;
	u8 buf[7];
	u8 addr[1] = { 4 }; /* internal addr point 0x0; transmission fmt 0x4 */
	struct i2c_msg msgs[2] = {
		{save_client->addr, 0, 1, addr},
		{save_client->addr, I2C_M_NOSTART | I2C_M_RD, 7, buf},
	};

	spin_lock_irq(&rv5c387a_lock);
	i2c_transfer(save_client->adapter, msgs, 2);
	spin_unlock_irq(&rv5c387a_lock);

	sec = buf[0] & 0x7f;
	min = buf[1] & 0x7f;
	hour = buf[2] & 0x3f;
	wday = buf[3] & 0x7;
	mday = buf[4] & 0x3f;
	mon = buf[5];
	year = buf[6] & 0xff;

	BCD_TO_BIN(sec);
	BCD_TO_BIN(min);
	BCD_TO_BIN(hour);
	BCD_TO_BIN(mday);
	BCD_TO_BIN(year);

	if (hour == 12)
		hour = 0;
	else if (hour == 32)
		hour = 12;
	else if (hour >= 21)
		hour -= 8;

	year += mon & 0x80 ? 2000 : 1900;

	mon = mon & 0x1f;
	BCD_TO_BIN(mon);
	return mktime(year, mon, mday, hour, min, sec);
}
EXPORT_SYMBOL_GPL(rv5c387a_rtc_get_time);


int rv5c387a_rtc_set_time(unsigned long t)
{
	struct rtc_time tm;
	u8 buf[7];
	u8 addr[1] = { 0 }; /* internal addr point 0x0; transmission fmt 0x0 */
	struct i2c_msg msgs[2] = {
		{save_client->addr, 0, 1, addr},
		{save_client->addr, I2C_M_NOSTART, 7, buf},
	};

	to_tm(t, &tm);


	if(buf[2] == 0)
		buf[2] = 12;
	else if(buf[2] == 12)
		buf[2] = 32;
	else if(buf[2] >= 13)
		buf[2] += 8;

	buf[0] = tm.tm_sec & 0x7f;
	buf[1] = tm.tm_min & 0x7f;
	buf[2] = tm.tm_hour & 0x3f;
	buf[3] = tm.tm_wday & 0x7;
	buf[4] = tm.tm_mday & 0x3f;
	buf[5] = (tm.tm_mon + 1) & 0x1f;

	BIN_TO_BCD(buf[0]);
	BIN_TO_BCD(buf[1]);
	BIN_TO_BCD(buf[2]);
	BIN_TO_BCD(buf[3]);
	BIN_TO_BCD(buf[4]);
	BIN_TO_BCD(buf[5]);
	buf[5] |= ((tm.tm_year /100) == 20 ? 0x80: 0);

	buf[6] = (tm.tm_year % 100) & 0xff;
	BIN_TO_BCD(buf[6]);

	spin_lock_irq(&rv5c387a_lock);
	i2c_transfer(save_client->adapter, msgs, 2);
	spin_unlock_irq(&rv5c387a_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(rv5c387a_rtc_set_time);

static int rv5c387a_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int rc;

	client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	memset(client, 0, sizeof(struct i2c_client));
	strncpy(client->name, RV5C387A_DRV_NAME, I2C_NAME_SIZE);
	client->id = rv5c387a_driver.id;
	client->flags = I2C_DF_NOTIFY;
	client->addr = addr;
	client->adapter = adap;
	client->driver = &rv5c387a_driver;

	if ((rc = i2c_attach_client(client)) != 0) {
		kfree(client);
		return rc;
	}

	save_client = client;
	rtc_attached = 1;
	return 0;
}

static int rv5c387a_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, rv5c387a_probe);
}

static int rv5c387a_detach(struct i2c_client *client)
{
	int ret;

	rtc_attached = 0;
	if ((ret = i2c_detach_client(client)) == 0)
		kfree(i2c_get_clientdata(client));
	return ret;
}

static struct i2c_driver rv5c387a_driver = {
	.owner		= THIS_MODULE,
	.name		= RV5C387A_DRV_NAME,
	.id		= I2C_DRIVERID_RV5C387A,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= rv5c387a_attach,
	.detach_client	= rv5c387a_detach,
};

static int __init rv5c387a_init(void)
{
	spin_lock_init(&rv5c387a_lock);
	return i2c_add_driver(&rv5c387a_driver);
}

static void __exit rv5c387a_exit(void)
{
	i2c_del_driver(&rv5c387a_driver);
}

module_init(rv5c387a_init);
module_exit(rv5c387a_exit);

MODULE_AUTHOR("Wade Farnsworth <wfarnsworth@mvista.com>");
MODULE_DESCRIPTION("Ricoh RV5C387A RTC I2C Client Driver");
MODULE_LICENSE("GPL");
