/*
 * drivers/char/cmbvr4133-rtc.c
 *
 * Interface for the Ricoh RV5C387A I2C RTC on the NEC CMB-VR4133 board.
 *
 * Author: Wade Farnsworth <wfarnsworth@mvista.com>
 *
 * 2006(c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/time.h>
#include <asm/vr41xx/giu.h>

static spinlock_t rtc_lock;

static struct fasync_struct *rtc_async_queue;

typedef enum {
	RTC_RELEASE,
	RTC_OPEN,
} rtc_status_t;

static rtc_status_t rtc_status;

extern unsigned long rv5c387a_rtc_get_time(void);
extern int rv5c387a_rtc_set_time(unsigned long);

static inline void vr4133_ricoh_rtc_get_time(struct rtc_time *tm)
{
	to_tm(rv5c387a_rtc_get_time(), tm);
	tm->tm_year -= 1900;
}

static inline void vr4133_ricoh_rtc_set_time(struct rtc_time *tm)
{
	rv5c387a_rtc_set_time(mktime(tm->tm_year + 1900 , tm->tm_mon + 1,
				     tm->tm_mday, tm->tm_hour, tm->tm_min,
				     tm->tm_sec));
}


static int vr4133_ricoh_rtc_do_ioctl(unsigned int cmd, unsigned long arg)
{
	struct rtc_time time;

	switch (cmd) {
	case RTC_RD_TIME:
		memset(&time, 0, sizeof(struct rtc_time));
		vr4133_ricoh_rtc_get_time(&time);
		if (copy_to_user((void __user *)arg, &time,
		    sizeof(struct rtc_time)))
			return -EFAULT;
		break;
	case RTC_SET_TIME:
		if (capable(CAP_SYS_TIME) == 0)
			return -EACCES;

		if (copy_from_user(&time, (struct rtc_time __user *)arg,
		    sizeof(struct rtc_time)))
			return -EFAULT;

		vr4133_ricoh_rtc_set_time(&time);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vr4133_ricoh_rtc_ioctl(struct inode *inode, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	return vr4133_ricoh_rtc_do_ioctl(cmd, arg);
}

static int vr4133_ricoh_rtc_open(struct inode *inode, struct file *file)
{
	spin_lock_irq(&rtc_lock);

	if (rtc_status == RTC_OPEN) {
		spin_unlock_irq(&rtc_lock);
		return -EBUSY;
	}

	rtc_status = RTC_OPEN;

	spin_unlock_irq(&rtc_lock);

	return 0;
}

static int vr4133_ricoh_rtc_release(struct inode *inode, struct file *file)
{
	if (file->f_flags & FASYNC)
		(void)fasync_helper(-1, file, 0, &rtc_async_queue);

	spin_lock_irq(&rtc_lock);
	rtc_status = RTC_RELEASE;
	spin_unlock_irq(&rtc_lock);
											return 0;
}

static struct file_operations vr4133_ricoh_rtc_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= vr4133_ricoh_rtc_ioctl,
	.open		= vr4133_ricoh_rtc_open,
	.release	= vr4133_ricoh_rtc_release,
};

static struct miscdevice vr4133_ricoh_rtc_dev = {
	.minor	= RTC_MINOR,
	.name	= "rtc",
	.fops	= &vr4133_ricoh_rtc_fops,
};

static int __init vr4133_ricoh_rtc_init(void)
{
	int retval;

	printk(KERN_INFO "NEC CMB-VR4133/Ricoh RV5C387A RTC Driver\n");

	retval = misc_register(&vr4133_ricoh_rtc_dev);
	if (retval < 0)
		return retval;

	spin_lock_init(&rtc_lock);

	return 0;
}

static void __exit vr4133_ricoh_rtc_exit(void)
{
	misc_deregister(&vr4133_ricoh_rtc_dev);
}

module_init(vr4133_ricoh_rtc_init);
module_exit(vr4133_ricoh_rtc_exit);

