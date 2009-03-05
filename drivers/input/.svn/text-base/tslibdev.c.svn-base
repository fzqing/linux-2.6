/*
 *  linux/drivers/input/tslibdev.c
 *
 *  Copyright (C) 2002 Russell King
 *
 * From tsdev.c:
 *
 *  Copyright (c) 2001 "Crazy" james Simmons 
 *
 *  Input driver to Touchscreen device driver module.
 *
 *  Sponsored by Transvirtual Technology
 */

/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * 
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <jsimmons@transvirtual.com>.
 */

#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/config.h>
#include <linux/smp_lock.h>
#include <linux/random.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/miscdevice.h>

struct ucb1x00_dev {
	int		exist;
	char		name[16];
	wait_queue_head_t wait;
	struct list_head list;
	struct input_handle handle;
	int		x;
	int		y;
	int		pressure;
};

struct ts_event {
	u16		pressure;
	u16		x;
	u16		y;
	u16		pad;
	struct timeval	stamp;
};

#define TSDEV_BUFFER_SIZE	64

struct ucb1x00_list {
	struct list_head	list;
	struct fasync_struct	*fasync;
	struct ucb1x00_dev	*tsdev;
	unsigned int		head;
	unsigned int		tail;
	struct ts_event		event[TSDEV_BUFFER_SIZE];
};

static struct input_handler ucb1x00_handler;
static struct ucb1x00_dev *ucb1x00_dev;

static void ucb1x00_remove(struct ucb1x00_dev *tsdev);


static int ucb1x00_fasync(int fd, struct file *file, int on)
{
	struct ucb1x00_list *list = file->private_data;
	int retval;

	retval = fasync_helper(fd, file, on, &list->fasync);
	return retval < 0 ? retval : 0;
}

static int ucb1x00_open(struct inode *inode, struct file *file)
{
	struct ucb1x00_list *list;
	int empty;

	if (!ucb1x00_dev || !ucb1x00_dev->exist)
		return -ENODEV;

	printk(KERN_WARNING
		"tslibdev: process %s (%d) uses obsolete tslib device\n",
		current->comm, current->pid);

	list = kmalloc(sizeof(struct ucb1x00_list), GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	memset(list, 0, sizeof(struct ucb1x00_list));

	empty = list_empty(&ucb1x00_dev->list);

	list->tsdev = ucb1x00_dev;
	list_add(&list->list, &list->tsdev->list);

	file->private_data = list;

	if (empty && list->tsdev->exist)
		input_open_device(&list->tsdev->handle);

	return 0;
}

static int ucb1x00_release(struct inode *inode, struct file *file)
{
	struct ucb1x00_list *list = file->private_data;

	ucb1x00_fasync(-1, file, 0);

	list_del(&list->list);

	ucb1x00_remove(list->tsdev);

	kfree(list);

	return 0;
}

static ssize_t
ucb1x00_read(struct file *file, char *buffer, size_t count, loff_t * ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct ucb1x00_list *list = file->private_data;
	int retval = 0;

	if (list->head == list->tail) {
		add_wait_queue(&list->tsdev->wait, &wait);

		while (list->head == list->tail) {
			set_current_state(TASK_INTERRUPTIBLE);

			if (!list->tsdev->exist) {
				retval = -ENODEV;
				break;
			}
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			schedule();
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&list->tsdev->wait, &wait);
	}

	if (retval)
		return retval;

	while (list->head != list->tail && count >= sizeof(struct ts_event)) {
		if (copy_to_user(buffer, list->event + list->tail,
				 sizeof(struct ts_event)))
			return retval ? retval : -EFAULT;
		list->tail = (list->tail + 1) & (TSDEV_BUFFER_SIZE - 1);
		retval += sizeof(struct ts_event);
		buffer += sizeof(struct ts_event);
		count -= sizeof(struct ts_event);
	}
	return retval;
}

/* No kernel lock - fine */
static unsigned int ucb1x00_poll(struct file *file, poll_table * wait)
{
	struct ucb1x00_list *list = file->private_data;

	poll_wait(file, &list->tsdev->wait, wait);
	if (list->head != list->tail || !list->tsdev->exist)
		return POLLIN | POLLRDNORM;
	return 0;
}

static int
ucb1x00_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	    unsigned long arg)
{
	return -EINVAL;
}

struct file_operations ucb1x00_fops = {
	.owner		= THIS_MODULE,
	.open		= ucb1x00_open,
	.release	= ucb1x00_release,
	.read		= ucb1x00_read,
	.poll		= ucb1x00_poll,
	.fasync		= ucb1x00_fasync,
	.ioctl		= ucb1x00_ioctl,
};

/*
 * The official UCB1x00 touchscreen is a miscdevice:
 *   10 char        Non-serial mice, misc features
 *                   14 = /dev/touchscreen/ucb1x00  UCB 1x00 touchscreen
 */
static struct miscdevice ucb1x00_ts_dev = {
	.minor		= 14,
	.name		= "touchscreen/ucb1x00",
	.fops		= &ucb1x00_fops,
	.devfs_name	= "touchscreen/ucb1x00",
};

static void ucb1x00_remove(struct ucb1x00_dev *tsdev)
{
	if (list_empty(&tsdev->list)) {
		if (tsdev->exist) {
			input_close_device(&tsdev->handle);
			wake_up_interruptible(&tsdev->wait);
		} else {
			misc_deregister(&ucb1x00_ts_dev);
			ucb1x00_dev = NULL;
			kfree(tsdev);
		}
	}
}


static void
ucb1x00_event(struct input_handle *handle, unsigned int type, unsigned int code,
	    int value)
{
	struct ucb1x00_dev *tsdev = handle->private;
	struct list_head *l;

	/* sorry, we only handle absolute stuff */
	if (type == EV_ABS) {
		switch (code) {
		case ABS_X:
			tsdev->x = value;
			break;
		case ABS_Y:
			tsdev->y = value;
			break;
		case ABS_PRESSURE:
			tsdev->pressure = value;
			break;
		}
		return;
	}

	if (type != EV_SYN || code != SYN_REPORT)
		return;

	list_for_each(l, &tsdev->list) {
		struct ucb1x00_list *list = list_entry(l, struct ucb1x00_list, list);
		list->event[list->head].pressure = tsdev->pressure;
		if (tsdev->pressure) {
			list->event[list->head].x = tsdev->x;
			list->event[list->head].y = tsdev->y;
		} else {
			list->event[list->head].x = 0;
			list->event[list->head].y = 0;
		}
		do_gettimeofday(&list->event[list->head].stamp);
		list->head = (list->head + 1) & (TSDEV_BUFFER_SIZE - 1);
		kill_fasync(&list->fasync, SIGIO, POLL_IN);
	}
	wake_up_interruptible(&tsdev->wait);
}

static struct input_handle *
ucb1x00_connect(struct input_handler *handler, struct input_dev *dev,
	      struct input_device_id *id)
{
	struct ucb1x00_dev *tsdev;

	if (ucb1x00_dev)
		return NULL;

	tsdev = kmalloc(sizeof(struct ucb1x00_dev), GFP_KERNEL);
	if (!tsdev)
		return NULL;

	memset(tsdev, 0, sizeof(struct ucb1x00_dev));
	init_waitqueue_head(&tsdev->wait);
	INIT_LIST_HEAD(&tsdev->list);

	ucb1x00_dev = tsdev;

	strncpy(tsdev->name, "ucb1x00-ts", sizeof(tsdev->name));

	tsdev->handle.dev     = dev;
	tsdev->handle.name    = tsdev->name;
	tsdev->handle.handler = handler;
	tsdev->handle.private = tsdev;

	misc_register(&ucb1x00_ts_dev);

	tsdev->exist = 1;

	return &tsdev->handle;
}

static void ucb1x00_disconnect(struct input_handle *handle)
{
	struct ucb1x00_dev *tsdev = handle->private;

	tsdev->exist = 0;
	ucb1x00_remove(tsdev);
}

static struct input_device_id ucb1x00_ids[] = {
	{
	      .flags	= INPUT_DEVICE_ID_MATCH_ABSBIT,
	      .evbit	= { BIT(EV_ABS) },
	      .absbit	= { BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE) },
	},

	{},/* Terminating entry */
};

MODULE_DEVICE_TABLE(input, ucb1x00_ids);

static struct input_handler ucb1x00_handler = {
	.event		= ucb1x00_event,
	.connect	= ucb1x00_connect,
	.disconnect	= ucb1x00_disconnect,
	.name		= "touchscreen/ucb1x00",
	.id_table	= ucb1x00_ids,
};

static int __init ucb1x00_init(void)
{
	input_register_handler(&ucb1x00_handler);
	printk(KERN_INFO "ts: UCB1x00 touchscreen protocol output\n");
	return 0;
}

static void __exit ucb1x00_exit(void)
{
	input_unregister_handler(&ucb1x00_handler);
}

module_init(ucb1x00_init);
module_exit(ucb1x00_exit);

MODULE_AUTHOR("Russell King <rmk@arm.linux.org.uk>");
MODULE_DESCRIPTION("Input driver to UCB1x00 touchscreen converter");
