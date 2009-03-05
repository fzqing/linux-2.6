/*
 * drivers/i2c/chips/max6900.c
 *
 * I2C client/driver for the DS-MAXIM MAX6900 Real-Time Clock chip.
 *
 * Author: Vladimir A. Barinov <vbarinov@ru.mvista.com>
 * 
 * Based on the m41t00.c by Mark Greer <mgreer@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
/*
 * This i2c client/driver wedges between the drivers/char/genrtc.c RTC
 * interface and the SMBus interface of the i2c subsystem.
 * It would be more efficient to use i2c msgs/i2c_transfer directly but, as
 * recommened in .../Documentation/i2c/writing-clients section
 * "Sending and receiving", using SMBus level communication is preferred.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>

#include <asm/time.h>
#include <asm/rtc.h>

#define	MAX6900_DRV_NAME		"max6900"

static DECLARE_MUTEX(max6900_mutex);

static struct i2c_driver max6900_driver;
static struct i2c_client *save_client;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x50, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.normal_i2c_range	= ignore,
	.probe			= ignore,
	.probe_range		= ignore,
	.ignore			= ignore,
	.ignore_range		= ignore,
	.force			= ignore,
};

/*
 * While the MAX6900 has a nice Clock Burst Read/Write command,
 * we can't use it, since some I2C controllers do not support
 * anything other than single-byte transfers
 */

#define MAX6900_REG_SECOND              0x80
#define MAX6900_REG_MINUTE              0x82
#define MAX6900_REG_HOUR                0x84
#define MAX6900_REG_DATE                0x86
#define MAX6900_REG_MONTH               0x88
#define MAX6900_REG_DAY                 0x8a
#define MAX6900_REG_YEAR                0x8c
#define MAX6900_REG_CONTROL             0x8e
#define MAX6900_REG_CENTURY             0x92
#define MAX6900_REG_CLK_BURST           0xbe

#define MAX6900_SECOND_MASK             0x7f
#define MAX6900_MINUTE_MASK             0x7f
#define MAX6900_HOUR_MASK             	0x3f
#define MAX6900_DATE_MASK               0x3f
#define MAX6900_MONTH_MASK              0x1f
#define MAX6900_YEAR_MASK		0xff
#define MAX6900_CENTURY_MASK		0xff
#define MAX6900_CONTROL_WP              (1 << 7)

ulong
max6900_get_rtc_time(void)
{
	s32	sec, min, hour, day, mon, year, century;
	s32	sec1, min1, hour1, day1, mon1, year1, century1;
	ulong	limit = 10;

	sec = min = hour = day = mon = year = century = 0;
	sec1 = min1 = hour1 = day1 = mon1 = year1 = century1 = 0;


	down(&max6900_mutex);
	do {
		if (((sec = i2c_smbus_read_byte_data(save_client, MAX6900_REG_SECOND + 1)) >= 0)
			&& ((min = i2c_smbus_read_byte_data(save_client, MAX6900_REG_MINUTE + 1))
				>= 0)
			&& ((hour = i2c_smbus_read_byte_data(save_client, MAX6900_REG_HOUR + 1))
				>= 0)
			&& ((day = i2c_smbus_read_byte_data(save_client, MAX6900_REG_DATE + 1))
				>= 0)
			&& ((mon = i2c_smbus_read_byte_data(save_client, MAX6900_REG_MONTH + 1))
				>= 0)
			&& ((year = i2c_smbus_read_byte_data(save_client, MAX6900_REG_YEAR + 1))
				>= 0)
			&& ((century = i2c_smbus_read_byte_data(save_client, MAX6900_REG_CENTURY + 1))
				>= 0)
			&& ((sec == sec1) && (min == min1) && (hour == hour1)
				&& (day == day1) && (mon == mon1)
				&& (year == year1) && (century == century1)))

				break;

		sec1 = sec;
		min1 = min;
		hour1 = hour;
		day1 = day;
		mon1 = mon;
		year1 = year;
		century1 = century;
	} while (--limit > 0);
	up(&max6900_mutex);

	if (limit == 0) {
		dev_warn(&save_client->dev,
			"max6900: can't read rtc chip\n");
		sec = min = hour = day = mon = year = century = 0;
	}

	sec &= MAX6900_SECOND_MASK;
	min &= MAX6900_MINUTE_MASK;
        hour &= MAX6900_HOUR_MASK;
	day &= MAX6900_DATE_MASK;
	mon &= MAX6900_MONTH_MASK;
	year &= MAX6900_YEAR_MASK;
	century &= MAX6900_CENTURY_MASK;

	BCD_TO_BIN(sec);
	BCD_TO_BIN(min);
	BCD_TO_BIN(hour);
	BCD_TO_BIN(day);
	BCD_TO_BIN(mon);
	BCD_TO_BIN(year);
	BCD_TO_BIN(century);

	year += century * 100;

	return mktime(year, mon, day, hour, min, sec);
}

static void
max6900_set_tlet(ulong arg)
{
	struct rtc_time	tm;
	ulong	nowtime = *(ulong *)arg;
	int tm_century;

	 /* Start by clearing the control register's write-protect bit. */
	if (i2c_smbus_write_byte_data(save_client, MAX6900_REG_CONTROL, 0) < 0)
		dev_warn(&save_client->dev,"max6900: can't write to rtc chip\n");

	to_tm(nowtime, &tm);
	tm_century = (tm.tm_year) / 100;
	tm.tm_year = (tm.tm_year) % 100;

	BIN_TO_BCD(tm.tm_sec);
	BIN_TO_BCD(tm.tm_min);
	BIN_TO_BCD(tm.tm_hour);
	BIN_TO_BCD(tm.tm_mon);
	BIN_TO_BCD(tm.tm_mday);
	BIN_TO_BCD(tm.tm_year);
	BIN_TO_BCD(tm_century);

	down(&max6900_mutex);
	if ((i2c_smbus_write_byte_data(save_client, MAX6900_REG_SECOND, tm.tm_sec) < 0)
		|| (i2c_smbus_write_byte_data(save_client, MAX6900_REG_MINUTE, tm.tm_min)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, MAX6900_REG_HOUR, tm.tm_hour)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, MAX6900_REG_DATE, tm.tm_mday)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, MAX6900_REG_MONTH, tm.tm_mon)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, MAX6900_REG_YEAR, tm.tm_year)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, MAX6900_REG_CENTURY, tm_century)
			< 0))
		dev_warn(&save_client->dev,"max6900: can't write to rtc chip\n");
	up(&max6900_mutex);

	/* Finish by setting the control register's write-protect bit. */
	if (i2c_smbus_write_byte_data(save_client, MAX6900_REG_CONTROL,  MAX6900_CONTROL_WP) < 0)
		dev_warn(&save_client->dev,"max6900: can't write to rtc chip\n");

	return;
}

ulong	new_time;

DECLARE_TASKLET_DISABLED(max6900_tasklet, max6900_set_tlet, (ulong)&new_time);

int
max6900_set_rtc_time(ulong nowtime)
{
	new_time = nowtime;

	if (in_interrupt())
		tasklet_schedule(&max6900_tasklet);
	else
		max6900_set_tlet((ulong)&new_time);

	return 0;
}

/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */
static int
max6900_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int rc;

	client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	memset(client, 0, sizeof(struct i2c_client));
	strncpy(client->name, MAX6900_DRV_NAME, I2C_NAME_SIZE);
	client->id = max6900_driver.id;
	client->flags = I2C_DF_NOTIFY;
	client->addr = addr;
	client->adapter = adap;
	client->driver = &max6900_driver;

	if ((rc = i2c_attach_client(client)) != 0) {
		kfree(client);
		return rc;
	}

	save_client = client;

	return 0;
}

static int
max6900_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, max6900_probe);
}

static int
max6900_detach(struct i2c_client *client)
{
	int	rc;

	if ((rc = i2c_detach_client(client)) == 0) {
		kfree(i2c_get_clientdata(client));
		tasklet_kill(&max6900_tasklet);
	}
	return rc;
}

static struct i2c_driver max6900_driver = {
	.owner		= THIS_MODULE,
	.name		= MAX6900_DRV_NAME,
	.id		= I2C_DRIVERID_MAX6900,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= max6900_attach,
	.detach_client	= max6900_detach,
};

static int __init
max6900_init(void)
{
printk("RTC MAX6900 driver init\n");
	return i2c_add_driver(&max6900_driver);
}

static void __exit
max6900_exit(void)
{
	i2c_del_driver(&max6900_driver);
	return;
}

/* 
 * we use subsys_initcall time for this module since
 * the MAX6900 I2C device address conflicts with 
 * i2c eeprom scan address series
 */
subsys_initcall(max6900_init);
module_exit(max6900_exit);

MODULE_AUTHOR("Vladimir A. Barinov <vbarinov@ru.mvista.com>");
MODULE_DESCRIPTION("DS-MAXIM MAX6900 RTC I2C Client Driver");
MODULE_LICENSE("GPL");
