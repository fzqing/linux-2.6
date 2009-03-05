/*
    2006 (c) MontaVista Software, Inc.
    Aleksey Makarov <amakarov@ru.mvista.com>

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
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/i2c-sensor.h>
#include <linux/spinlock.h>

#include <asm/vr41xx/giu.h>
#include <asm/vr41xx/vr41xx.h>

/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x3c, 0x3d, 0x3e, 0x3f, I2C_CLIENT_END };
static unsigned int normal_isa[] = { I2C_CLIENT_ISA_END };

/* Insmod parameters */
SENSORS_INSMOD_1(bu9929fv);

/* Initial values */
#define BU9929FV_INIT 0
#define BU9929FV_OUTPUT_INIT 0
#define BU9929FV_WATCHING_INIT 0

#define WATCH_IRQ_PIN 5

/* Each client has this additional data */
struct bu9929fv_data {
	struct i2c_client client;

	u16 write;
	u16 output;
	int watch;

	wait_queue_head_t event_wait_queue;
	spinlock_t irq_init_lock;
	int irq_initialized;
};

static int bu9929fv_attach_adapter(struct i2c_adapter *adapter);
static int bu9929fv_detect(struct i2c_adapter *adapter, int address, int kind);
static int bu9929fv_detach_client(struct i2c_client *client);
static void bu9929fv_read(struct bu9929fv_data * data, u16 * value);
static void bu9929fv_write(struct bu9929fv_data * data, u16 value);
static void bu9929fv_output(struct bu9929fv_data * data, u16 value);
static void bu9929fv_watch(struct bu9929fv_data * data, int watch_pins);

/* This is the driver that will be inserted */
static struct i2c_driver bu9929fv_driver = {
	.owner		= THIS_MODULE,
	.name		= "bu9929fv",
	.id		= I2C_DRIVERID_BU9929FV,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= bu9929fv_attach_adapter,
	.detach_client	= bu9929fv_detach_client,
};

static int bu9929fv_id;

/* BEGIN sysfs callback functions */

static void watch_reset (struct bu9929fv_data *data)
{
	bu9929fv_watch(data, data->watch);
	wake_up_all (&data->event_wait_queue);
}

static ssize_t show_read(struct device *dev, char *buf)
{
	u16 read;
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	bu9929fv_read(data, &read);
	return sprintf(buf, "%u\n", read);
}

static DEVICE_ATTR(read, S_IRUGO, show_read, NULL);

static ssize_t show_write(struct device *dev, char *buf)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%u\n", data->write);
}

static ssize_t set_write(struct device *dev, const char *buf,
			 size_t count)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	data->write = simple_strtoul(buf, NULL, 10);
	bu9929fv_write(data, data->write);
	watch_reset (data);
	return count;
}

static DEVICE_ATTR(write, S_IWUSR | S_IRUGO, show_write, set_write);

static ssize_t show_output(struct device *dev, char *buf)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%u\n", data->output);
}

static ssize_t set_output(struct device *dev, const char *buf,
			 size_t count)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	data->output = simple_strtoul(buf, NULL, 10);
	bu9929fv_output(data, data->output);
	watch_reset (data);
	return count;
}

static DEVICE_ATTR(output, S_IWUSR | S_IRUGO, show_output, set_output);

static ssize_t show_watch(struct device *dev, char *buf)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%d\n", data->watch);
}

static ssize_t set_watch(struct device *dev, const char *buf,
			 size_t count)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));
	int v = simple_strtol(buf, NULL, 10);

	if (v & ~1)
		return count;

	data->watch = v;
	bu9929fv_watch(data, data->watch);
	return count;
}

static DEVICE_ATTR(watch, S_IWUSR | S_IRUGO, show_watch, set_watch);

static irqreturn_t handler (int irq, void * dev_id, struct pt_regs * regs)
{
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev_id));
	wake_up_all (&data->event_wait_queue);
	return IRQ_HANDLED;
}

static ssize_t show_event(struct device *dev, char *buf)
{
	int err;
	DEFINE_WAIT(w);
	struct bu9929fv_data *data = i2c_get_clientdata(to_i2c_client(dev));

	spin_lock (&data->irq_init_lock);
	if (!(data->irq_initialized++)) {
		err = request_irq(GIU_IRQ(WATCH_IRQ_PIN), handler, 0, "bu9929fv", dev);
		if (err) {
			printk ("interrupt request error\n");
			spin_unlock (&data->irq_init_lock);
			goto exit;
		}
	}
	spin_unlock (&data->irq_init_lock);

	prepare_to_wait(&data->event_wait_queue, &w, TASK_INTERRUPTIBLE);
	schedule();
	finish_wait(&data->event_wait_queue, &w);

	spin_lock (&data->irq_init_lock);
	if (!--data->irq_initialized)
		free_irq (GIU_IRQ(WATCH_IRQ_PIN), dev);
	spin_unlock (&data->irq_init_lock);
exit:
	strcpy (buf, "0\n");
	return strlen (buf);

}

static DEVICE_ATTR(event, S_IRUGO, show_event, NULL);

/* END sysfs callback functions */

/* BEGIN initialization/finalization */

static int bu9929fv_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_detect(adapter, &addr_data, bu9929fv_detect);
}

static int bu9929fv_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct bu9929fv_data *data;
	int err = 0;

	if (!(data = kmalloc(sizeof(struct bu9929fv_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct bu9929fv_data));

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &bu9929fv_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, "bu9929fv", I2C_NAME_SIZE);
	new_client->id = bu9929fv_id++;

	init_waitqueue_head (&data->event_wait_queue);
	data->irq_init_lock = SPIN_LOCK_UNLOCKED;
	data->irq_initialized = 0;

	data->output = BU9929FV_OUTPUT_INIT;
	data->write = BU9929FV_INIT;
	data->watch = BU9929FV_WATCHING_INIT;

	if ((err = i2c_attach_client(new_client)))
		goto exit_free;

	bu9929fv_output(data, data->output);
	bu9929fv_write(data, data->write);
	bu9929fv_watch(data, data->watch);

	vr41xx_set_irq_trigger(WATCH_IRQ_PIN, IRQ_TRIGGER_EDGE_FALLING, IRQ_SIGNAL_HOLD);

	device_create_file(&new_client->dev, &dev_attr_read);
	device_create_file(&new_client->dev, &dev_attr_write);
	device_create_file(&new_client->dev, &dev_attr_output);
	device_create_file(&new_client->dev, &dev_attr_watch);
	device_create_file(&new_client->dev, &dev_attr_event);
	return 0;

      exit_free:
	kfree(data);
      exit:
	return err;
}

static int bu9929fv_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client))) {
		dev_err(&client->dev,
			"Client deregistration failed, client not detached.\n");
		return err;
	}

	kfree(i2c_get_clientdata(client));
	return 0;
}

/* END initialization/finalization */

/* BEGIN real communication with the chip */

static void bu9929fv_read(struct bu9929fv_data * data, u16 * value)
{
	char c[2];
	i2c_master_recv(&data->client, c, 2);
	*value = c[0] | (c[1] << 8);
}

static void bu9929fv_write(struct bu9929fv_data * data, u16 value)
{
	char c[3] = { 1 << 4, value & 0xff, (value >> 8) & 0xff };
	i2c_master_send(&data->client, c, 3);
}

static void bu9929fv_output(struct bu9929fv_data * data, u16 value)
{
	char c[3] = { 1 << 3, value & 0xff, (value >> 8) & 0xff };
	i2c_master_send(&data->client, c, 3);
}

/*
 * watch pins:
 * 0 - PION0-PION7
 * 1 - PION8-PIO15
 */
static void bu9929fv_watch(struct bu9929fv_data * data, int watch_pins)
{
	char c[3] = { 0x63 | ((watch_pins) << 2), 0, 0 };

	if (watch_pins & ~1)
		return;

	i2c_master_send(&data->client, c, 3);
}

/* END real communication with the chip */

static int __init bu9929fv_init(void)
{
	return i2c_add_driver(&bu9929fv_driver);
}

static void __exit bu9929fv_exit(void)
{
	i2c_del_driver(&bu9929fv_driver);
}

MODULE_AUTHOR("Aleksey Makarov <amakarov@ru.mvista.com>");
MODULE_DESCRIPTION("bu9929fv driver");
MODULE_LICENSE("GPL");

module_init(bu9929fv_init);
module_exit(bu9929fv_exit);
