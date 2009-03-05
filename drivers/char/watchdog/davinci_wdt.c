/*
 * linux/drivers/char/watchdog/davinci_wdt.c
 *
 * Watchdog driver for the TI DAVINCI
 *
 * Copyright (c) 2006 Texas Instruments.
 *
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/err.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/bitops.h>
#include <asm/hardware/clock.h>
#include <asm/arch/hardware.h>

#include <linux/moduleparam.h>
#include "davinci_wdt.h"

/* Instance of the private WDT device structure */
static struct wdt_davinci_device wdt_davinci_dev;

static int timer_margin = TIMER_MARGIN_CFG_VAL;	/* in seconds */
static int davinci_wdt_users;
static struct miscdevice davinci_wdt_miscdev;	/* Forward declaration */

static void davinci_wdt_ping(void)
{
	wdt_davinci_dev.regs->wdkey = 0xA5C6;
	wdt_davinci_dev.regs->wdkey = 0xDA7E;
	return;
}

static void davinci_wdt_enable(void)
{
	/* timer runs continuously */
	wdt_davinci_dev.regs->tcr |= 0x80;
	/* put watchdog in pre-active state */
	wdt_davinci_dev.regs->wdkey = 0xA5C6;
	/* put watchdog in active state */
	wdt_davinci_dev.regs->wdkey = 0xDA7E;

	return;
}

/*
 *	Allow only one person to hold it open
 */
static int davinci_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, (unsigned long *)&davinci_wdt_users))
		return -EBUSY;

	davinci_wdt_enable();

	return 0;
}

static int davinci_wdt_release(struct inode *inode, struct file *file)
{
	return 0;
}

static loff_t davinci_wdt_llseek(struct file *file, loff_t offset, int whence)
{
	return -ESPIPE;		/* Not seekable */
}

static ssize_t
davinci_wdt_write(struct file *file, const char *data, size_t len,
		  loff_t * ppos)
{
	/*  Can't seek (pwrite) on this device  */
	if (*ppos != file->f_pos)
		return -ESPIPE;

	/* Refresh LOAD_TIME. */
	if (len) {
		davinci_wdt_ping();
		return 1;
	}
	return 0;
}

static int
davinci_wdt_ioctl(struct inode *inode, struct file *file,
		  unsigned int cmd, unsigned long arg)
{
	static struct watchdog_info ident = {
		.identity = "DaVinci Watchdog",
		.options = WDIOF_CARDRESET,
		.firmware_version = 0,
	};

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *)arg, &ident,
				    sizeof(ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int *)arg);
	case WDIOC_KEEPALIVE:
		davinci_wdt_ping();
		return 0;
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int *)arg);
	}
}

static struct file_operations davinci_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = davinci_wdt_llseek,
	.write = davinci_wdt_write,
	.ioctl = davinci_wdt_ioctl,
	.open = davinci_wdt_open,
	.release = davinci_wdt_release,
};

static struct miscdevice davinci_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &davinci_wdt_fops
};

static int __init davinci_wdt_init(void)
{
	int ret;

	ret = misc_register(&davinci_wdt_miscdev);

	if (ret)
		return ret;

	wdt_davinci_dev.regs = (davinci_wdtregsovly) DAVINCI_WDT_BASE;

	/* disable timer */
	wdt_davinci_dev.regs->tcr = 0x0;
	/* reset timer */
	wdt_davinci_dev.regs->tgcr = 0x0;

	/* configure timer2 as 64-bit */
	wdt_davinci_dev.regs->tgcr = 0x8;
	/* release timer from reset */
	wdt_davinci_dev.regs->tgcr |= 0x3;
	/* enable watchdog timer */
	wdt_davinci_dev.regs->wdtcr |= 0x4000;

	if (timer_margin < TIMER_MARGIN_MIN_VAL || 
	    timer_margin > TIMER_MARGIN_MAX_VAL)
		timer_margin = TIMER_MARGIN_DEF_VAL;

	/* configure the period register */
	wdt_davinci_dev.regs->prd12 = CLOCK_TICK_RATE * timer_margin;

	printk(KERN_INFO "%s: TI DaVinci Watchdog Timer: timer margin %d sec\n",
	       davinci_wdt_miscdev.name, timer_margin);

	return 0;
}

static void __exit davinci_wdt_exit(void)
{
	misc_deregister(&davinci_wdt_miscdev);
}

module_init(davinci_wdt_init);
module_exit(davinci_wdt_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
module_param(timer_margin, int, 0);
