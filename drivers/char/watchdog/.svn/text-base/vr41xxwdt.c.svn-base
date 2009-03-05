/*
 * drivers/char/vr41xxwdt.c
 *
 * Watchdog driver for integrated watchdog in the NEC VR41xx processors.
 *
 * Copyright (C) 2001 Paul Mundt <lethal@chaoticdreams.org>
 * Copyright 2004 (c) MontaVista, Software, Inc.
 *          <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/smp_lock.h>
#include <linux/ioport.h>
#include <linux/fs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/vr41xx/vrblade4133a.h>

#define VR41XX_WDT_CNTREG	 (void *)KSEG1ADDR(0x0f000200)
#define VR41XX_WDT_SETREG	 (void *)KSEG1ADDR(0x0f000202)
#define VR41XX_WDT_CLRREG	 (void *)KSEG1ADDR(0x0f000204)
#define VR41XX_WDT_TIMREG	 (void *)KSEG1ADDR(0x0f000206)
#define VR41XX_WDT_UNUTREG	 (void *)KSEG1ADDR(0x0f00020c)

#define WDT_MARGIN_MIN		1	/* in seconds */
#define WDT_MARGIN_MAX		255
#define WDT_MARGIN_DEFAULT	10

static int vr41xx_wdt_is_open = 0;
static struct watchdog_info vr41xx_wdt_info;

static int timer_margin = WDT_MARGIN_DEFAULT;	/* in seconds */

/**
 * 	vr41xx_wdt_start - Start the Watchdog
 *
 * 	Starts the watchdog.
 */
static void vr41xx_wdt_start(void)
{

	/* Clear the counter */
	writew(0x01, VR41XX_WDT_CLRREG);

	/* Set the overflow period (in seconds) */
	writew(timer_margin, VR41XX_WDT_SETREG);

	/* Turn on the watchdog */
	writew(0x01, VR41XX_WDT_CNTREG);
}

/**
 * 	vr41xx_wdt_stop - Stop the Watchdog
 *
 * 	Stops the watchdog.
 */
static void vr41xx_wdt_stop(void)
{
	/* Clear the counter */
	writew(0x01, VR41XX_WDT_CLRREG);

	/* Turn off the watchdog */
	writew(0x00, VR41XX_WDT_CNTREG);
}

/**
 * 	vr41xx_wdt_ping - Ping the Watchdog
 *
 * 	@data: Unused
 *
 * 	Clears WDT_ counter.
 */
static void vr41xx_wdt_ping(void)
{
	writew(0x01, VR41XX_WDT_CLRREG);
}

/**
 * 	vr41xx_wdt_open - Open the Device
 *
 * 	@inode: inode of device
 * 	@file: file handle of device
 *
 * 	Watchdog device is opened and started.
 */
static int vr41xx_wdt_open(struct inode *inode, struct file *file)
{
	switch (MINOR(inode->i_rdev)) {
	case WATCHDOG_MINOR:
		if (vr41xx_wdt_is_open)
			return -EBUSY;

		vr41xx_wdt_is_open = 1;
		vr41xx_wdt_start();

		break;
	default:
		return -ENODEV;
	}

	return 0;
}

/**
 * 	vr41xx_wdt_close - Close the Device
 *
 * 	@inode: inode of device
 * 	@file: file handle of device
 *
 * 	Watchdog is closed and stopped.
 */
static int vr41xx_wdt_close(struct inode *inode, struct file *file)
{
	lock_kernel();

	if (MINOR(inode->i_rdev) == WATCHDOG_MINOR) {
#ifndef CONFIG_WATCHDOG_NOWAYOUT
		vr41xx_wdt_stop();
#endif
		vr41xx_wdt_is_open = 0;
	}

	unlock_kernel();

	return 0;
}

/**
 * 	vr41xx_wdt_read - Read from Device
 *
 * 	@file: file handle of device
 * 	@buf: buffer to write to
 * 	@count: length of buffer
 * 	@ppos: offset
 *
 * 	Unsupported.
 */
static ssize_t vr41xx_wdt_read(struct file *file, char *buf,
			       size_t count, loff_t * ppos)
{
	return -EINVAL;
}

/**
 * 	vr41xx_wdt_write - Write to Device
 *
 * 	@file: file handle of device
 * 	@buf: buffer to write
 * 	@count: length of buffer
 * 	@ppos: offset
 *
 * 	Updates next heartbeat interval on write.
 */
static ssize_t vr41xx_wdt_write(struct file *file, const char *buf,
				size_t count, loff_t * ppos)
{
	if (count) {
		vr41xx_wdt_ping();
		return 1;
	}

	return 0;
}

/**
 * 	vr41xx_wdt_ioctl - Query Device
 *
 * 	@inode: inode of device
 * 	@file: file handle of device
 * 	@cmd: watchdog command
 * 	@arg: argument
 *
 * 	Query basic information from the device or ping it, as outlined by the
 * 	watchdog API.
 */
static int vr41xx_wdt_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	int new_margin;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *)arg,
				    &vr41xx_wdt_info, sizeof(vr41xx_wdt_info))
		    ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
		return copy_to_user((int *)arg, &vr41xx_wdt_is_open,
				    sizeof(int)) ? -EFAULT : 0;
	case WDIOC_KEEPALIVE:
		vr41xx_wdt_ping();
		break;
	case WDIOC_SETTIMEOUT:

		if (get_user(new_margin, (int *)arg)) {
			printk("return -EFAULT\n");
			return -EFAULT;
		}
		if ((new_margin < WDT_MARGIN_MIN)
		    || (new_margin > WDT_MARGIN_MAX))
			timer_margin = WDT_MARGIN_DEFAULT;  /* default timeout */
		else
			timer_margin = new_margin;

		if (vr41xx_wdt_is_open) {
			vr41xx_wdt_stop();
			writew(timer_margin, VR41XX_WDT_TIMREG);
			vr41xx_wdt_start();
		} else {
			printk("vr41xx_wdt_is_open is not open\n");
		}
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int *)arg);
	default:
		return -ENOTTY;
	}

	return 0;
}

/**
 * 	vr41xx_wdt_notify_sys - Notifier Handler
 *
 * 	@this: notifier block
 * 	@code: notifier event
 * 	@unused: unused.
 *
 * 	Handles specific events, such as turning off the watchdog during a
 * 	shutdown event.
 */
static int vr41xx_wdt_notify_sys(struct notifier_block *this,
				 unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		vr41xx_wdt_stop();

	return NOTIFY_DONE;
}

static struct file_operations vr41xx_wdt_fops = {
	owner:THIS_MODULE,
	read:vr41xx_wdt_read,
	write:vr41xx_wdt_write,
	ioctl:vr41xx_wdt_ioctl,
	open:vr41xx_wdt_open,
	release:vr41xx_wdt_close,
};

static struct watchdog_info vr41xx_wdt_info = {
	WDIOF_KEEPALIVEPING,
	1,
	"VR41xx WDT",
};

static struct notifier_block vr41xx_wdt_notifier = {
	vr41xx_wdt_notify_sys,
	NULL,
	0
};

static struct miscdevice vr41xx_wdt_miscdev = {
	WATCHDOG_MINOR,
	"watchdog",
	&vr41xx_wdt_fops,
};

/**
 * 	vr41xx_wdt_init - Initialize module
 *
 * 	Registers the device, reserves register region, and registers the
 * 	notifier handler. Actual device initialization is handled by
 * 	vr41xx_wdt_open().
 */
static int __init vr41xx_wdt_init(void)
{

	if (misc_register(&vr41xx_wdt_miscdev)) {
		printk(KERN_ERR "vr41xx wdt: Can't register misc device\n");
		return -EINVAL;
	}

	if (register_reboot_notifier(&vr41xx_wdt_notifier)) {
		printk(KERN_ERR "vr41xx wdt: Can't register reboot notifier\n");
		release_region((unsigned long)VR41XX_WDT_CNTREG, 8);
		misc_deregister(&vr41xx_wdt_miscdev);
		return -EINVAL;
	}

	return 0;
}

/**
 * 	vr41xx_wdt_exit - Deinitialize module
 *
 * 	Unregisters the device, releases reserved register region, and
 * 	deregisters the notifier handler. Actual device deinitialization is
 * 	handled by vr41xx_wdt_close().
 */
static void __exit vr41xx_wdt_exit(void)
{
	unregister_reboot_notifier(&vr41xx_wdt_notifier);
	release_region((unsigned long)VR41XX_WDT_CNTREG, 8);
	misc_deregister(&vr41xx_wdt_miscdev);
}

MODULE_AUTHOR("Paul Mundt <lethal@chaoticdreams.org>");
MODULE_DESCRIPTION("NEC VR41xx watchdog driver");
MODULE_PARM(timer_margin, "i");
MODULE_PARM_DESC(timer_margin,
		 "Number of seconds for overflow period. (1 - 255 seconds).");
MODULE_LICENSE("GPL");

module_init(vr41xx_wdt_init);
module_exit(vr41xx_wdt_exit);
