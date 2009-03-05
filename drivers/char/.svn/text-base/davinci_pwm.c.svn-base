/*
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

/* include Linux files */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>		/* Used for struct cdev */
#include <linux/dma-mapping.h>	/* For class_simple_create */
#include <linux/interrupt.h>	/* For IRQ_HANDLED and irqreturn_t */
#include <asm/uaccess.h>	/* for VERIFY_READ/VERIFY_WRITE/
				   copy_from_user */
#include <linux/wait.h>
#include <linux/devfs_fs_kernel.h>	/* for devfs */
#include <asm/hardware/clock.h>
#include <asm/arch/davinci_pwm.h>
#include <asm/arch/cpu.h>
#include <asm/semaphore.h>
#include <asm/arch/irqs.h>

#define	DRIVER_NAME		"PWM"
#define	DAVINCI_PWM_TIMEOUT	(1*HZ)

struct pwm_davinci_device {
	char name[20];
	int intr_complete;
	dev_t devno;
	davinci_pwmregsovly regs;
	wait_queue_head_t intr_wait;
	struct clk *pwm_clk;
};

char *dm644x_name[] = { "PWM0_CLK", "PWM1_CLK", "PWM2_CLK" };
char *dm646x_name[] = { "PWM0_CLK", "PWM1_CLK" };
char *dm355_name[] = { "PWM0_CLK", "PWM1_CLK", "PWM2_CLK", "PWM3_CLK" };

/* Instance of the private WDT device structure */
static struct pwm_davinci_device *pwm_dev_array[DAVINCI_PWM_MINORS];
static DEFINE_SPINLOCK(pwm_dev_array_lock);

static unsigned int pwm_major = 0;
static unsigned int pwm_minor_start = 0;
static unsigned int pwm_minor_count = DM644X_PWM_MINORS;

static unsigned int pwm_device_count = 1;

/* For registeration of charatcer device*/
static struct cdev c_dev;

struct pwm_davinci_device *pwm_dev_get_by_minor(unsigned index)
{
	struct pwm_davinci_device *pwm_dev;

	spin_lock(&pwm_dev_array_lock);
	pwm_dev = pwm_dev_array[index];
	spin_unlock(&pwm_dev_array_lock);
	return pwm_dev;
}

static loff_t pwm_llseek(struct file *file, loff_t offset, int whence)
{
	return -ESPIPE;		/* Not seekable */
}

static int pwm_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct pwm_davinci_device *pwm_dev;

	pwm_dev = pwm_dev_get_by_minor(minor);

	/* sample configuration */
	pwm_dev->regs->per = 0xf;
	pwm_dev->regs->ph1d = 0xf;
	pwm_dev->regs->rpt = 1;
	pwm_dev->regs->cfg |= 0x1;

	pwm_dev->intr_complete = 0;

	return 0;
}

static int pwm_release(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct pwm_davinci_device *pwm_dev;

	pwm_dev = pwm_dev_get_by_minor(minor);

	pwm_dev->regs->cfg &= 0xFFFFFFFC;
	/* This is called when the reference count goes to zero */
	return 0;
}

int pwm_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	      unsigned long arg)
{
	int mode;
	unsigned int minor = iminor(inode);
	struct pwm_davinci_device *pwm_dev;

	pwm_dev = pwm_dev_get_by_minor(minor);

	switch (cmd) {
	case PWMIOC_SET_MODE:
		if (pwm_dev->regs->cfg & 0x20000)
			return -EBUSY;

		get_user(mode, (int *)arg);
		if (mode == PWM_ONESHOT_MODE) {
			pwm_dev->regs->cfg &= 0xFFFFFFFC;
			pwm_dev->regs->cfg |= 0x1;
		} else if (mode == PWM_CONTINUOUS_MODE) {
			pwm_dev->regs->cfg &= 0xFFFFFFFC;
			pwm_dev->regs->cfg |= 0x2;
		} else
			return -EINVAL;
		break;
	case PWMIOC_SET_PERIOD:
		get_user(mode, (int *)arg);

		if (mode < 0 || mode > 0xffffffff)
			return -EINVAL;

		if (pwm_dev->regs->cfg & 0x2 && pwm_dev->regs->cfg & 0x20000) {
			if (mode < 7)
				return -EINVAL;

			/* Enable PWM interrupts */
			pwm_dev->regs->cfg |= 0x40;

			/* wait for the transaction to complete */
			wait_event_timeout(pwm_dev->intr_wait,
					   pwm_dev->intr_complete,
					   DAVINCI_PWM_TIMEOUT);

			if (pwm_dev->intr_complete)
				pwm_dev->regs->per = mode;
			else
				return -1;
		} else
			pwm_dev->regs->per = mode;
		break;
	case PWMIOC_SET_DURATION:
		get_user(mode, (int *)arg);

		if (mode < 0 || mode > 0xffffffff)
			return -EINVAL;

		if (pwm_dev->regs->cfg & 0x2 && pwm_dev->regs->cfg & 0x20000) {
			/* Enable PWM interrupts */
			pwm_dev->regs->cfg |= 0x40;

			/* wait for the transaction to complete */
			wait_event_timeout(pwm_dev->intr_wait,
					   pwm_dev->intr_complete,
					   DAVINCI_PWM_TIMEOUT);

			if (pwm_dev->intr_complete)
				pwm_dev->regs->ph1d = mode;
			else
				return -1;
		} else
			pwm_dev->regs->ph1d = mode;
		break;
	case PWMIOC_SET_RPT_VAL:
		get_user(mode, (int *)arg);

		if (mode < 0 || mode > 0xff)
			return -EINVAL;

		pwm_dev->regs->rpt = mode;
		break;
	case PWMIOC_SET_FIRST_PHASE_STATE:
		get_user(mode, (int *)arg);

		if (pwm_dev->regs->cfg & 0x20000)
			return -EBUSY;
		if (mode == 1)
			pwm_dev->regs->cfg |= 0x10;
		else if (mode == 0)
			pwm_dev->regs->cfg &= ~0x10;
		else
			return -EINVAL;
		break;
	case PWMIOC_SET_INACT_OUT_STATE:
		get_user(mode, (int *)arg);

		if (pwm_dev->regs->cfg & 0x20000)
			return -EBUSY;
		if (mode == 1)
			pwm_dev->regs->cfg |= 0x20;
		else if (mode == 0)
			pwm_dev->regs->cfg &= ~0x20;
		else
			return -EINVAL;
		break;
	case PWMIOC_START:
		pwm_dev->regs->start = 0x1;
		break;
	case PWMIOC_STOP:
		if (pwm_dev->regs->cfg & 0x1 && pwm_dev->regs->cfg & 0x20000)
			pwm_dev->regs->cfg &= 0xFFFFFFFC;
		if (pwm_dev->regs->cfg & 0x2 && pwm_dev->regs->cfg & 0x20000) {
			unsigned long temp;
			temp = pwm_dev->regs->cfg;
			temp &= 0xFFFFFFFC;
			temp |= 0x1;

			/* Enable PWM interrupts */
			pwm_dev->regs->cfg |= 0x40;

			/* wait for the transaction to complete */
			wait_event_timeout(pwm_dev->intr_wait,
					   pwm_dev->intr_complete,
					   DAVINCI_PWM_TIMEOUT);

			if (pwm_dev->intr_complete)
				pwm_dev->regs->cfg = temp;
			else
				return -1;
		}
		break;
	}

	return 0;
}

static int pwm_remove(struct device *device)
{
	return 0;
}

static void pwm_platform_release(struct device *device)
{
	/* this function does nothing */
}

static struct file_operations pwm_fops = {
	.owner = THIS_MODULE,
	.llseek = pwm_llseek,
	.open = pwm_open,
	.release = pwm_release,
	.ioctl = pwm_ioctl,
};

static struct class_simple *pwm_class = NULL;

static struct platform_device pwm_device[] = {
	[0] = {
		.name = "davinci_pwm0",
		.id = 0,
		.dev = {
			.release = pwm_platform_release,
		}
	},
	[1] = {
		.name = "davinci_pwm1",
		.id = 1,
		.dev = {
			.release = pwm_platform_release,
		}
	},
	[2] = {
		.name = "davinci_pwm2",
		.id = 2,
		.dev = {
			.release = pwm_platform_release,
		}
	},
	[3] = {.name = "davinci_pwm3",
	       .id = 3,
	       .dev =  {
			.release = pwm_platform_release,
		}
	}
};

static struct device_driver pwm_driver[] = {
	[0] = {
		.name = "davinci_pwm0",
		.bus = &platform_bus_type,
		.remove = pwm_remove
	},
	[1] = {
		.name = "davinci_pwm1",
		.bus = &platform_bus_type,
		.remove = pwm_remove
	},
	[2] = {
		.name = "davinci_pwm2",
		.bus = &platform_bus_type,
		.remove = pwm_remove
	},
	[3] = {
		.name = "davinci_pwm3",
		.bus = &platform_bus_type,
		.remove = pwm_remove
	},
};

/*
 * This function marks a transaction as complete.
 */
static inline void pwm_davinci_complete_intr(struct pwm_davinci_device *dev)
{
	dev->intr_complete = 1;
	wake_up(&dev->intr_wait);
}

static irqreturn_t pwm_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct pwm_davinci_device *dev = dev_id;

	/* Disable PWM interrupts */
	dev->regs->cfg &= ~0x40;

	pwm_davinci_complete_intr(dev);
	return IRQ_HANDLED;
}

static int __init pwm_init(void)
{
	int result;
	dev_t devno;
	unsigned int size, i, j;
	char *name[DAVINCI_PWM_MINORS];

	if (cpu_is_davinci_dm6467()) {
		pwm_minor_count = DM646X_PWM_MINORS;
		for (i = 0; i < pwm_minor_count; i++)
			name[i] = dm646x_name[i];
	} else if (cpu_is_davinci_dm355()) {
		pwm_minor_count = DM355_PWM_MINORS;
		for (i = 0; i < pwm_minor_count; i++)
			name[i] = dm355_name[i];
	} else {
		pwm_minor_count = DM644X_PWM_MINORS;
		for (i = 0; i < pwm_minor_count; i++)
			name[i] = dm644x_name[i];
	}

	size = pwm_device_count * pwm_minor_count;
	/* Register the driver in the kernel */
	result = alloc_chrdev_region(&devno, 0, size, DRIVER_NAME);
	if (result < 0) {
		printk("DaVinciPWM: Module intialization failed.\
                        could not register character device\n");
		return -ENODEV;
	}
	pwm_major = MAJOR(devno);

	/* Initialize of character device */
	cdev_init(&c_dev, &pwm_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &pwm_fops;

	/* addding character device */
	result = cdev_add(&c_dev, devno, pwm_minor_count);
	if (result) {
		printk("DaVinciPWM:Error adding DavinciPWM\n");
		unregister_chrdev_region(devno, size);
		return result;
	}

	pwm_class = class_simple_create(THIS_MODULE, "davinci_pwm");
	if (!pwm_class) {
		cdev_del(&c_dev);
		return -EIO;
	}

	for (i = 0; i < pwm_device_count; i++) {
		for (j = 0; j < pwm_minor_count; j++) {
			pwm_dev_array[j] =
			    kmalloc(sizeof(struct pwm_davinci_device),
				    GFP_KERNEL);
			pwm_dev_array[j]->devno = devno;
			init_waitqueue_head(&pwm_dev_array[j]->intr_wait);
			sprintf(pwm_dev_array[j]->name, "davinci_pwm%d", j);

			/* register driver as a platform driver */
			if (driver_register(&pwm_driver[j]) != 0) {
				unregister_chrdev_region(devno, size);
				cdev_del(&c_dev);
				kfree(pwm_dev_array[j]);
				return -EINVAL;
			}

			/* Register the drive as a platform device */
			if (platform_device_register(&pwm_device[j]) != 0) {
				driver_unregister(&pwm_driver[j]);
				unregister_chrdev_region(devno, size);
				cdev_del(&c_dev);
				kfree(pwm_dev_array[j]);
				return -EINVAL;
			}

			devno =
			    MKDEV(pwm_major,
				  pwm_minor_start + i * pwm_minor_count + j);
			class_simple_device_add(pwm_class, devno, NULL,
						"davinci_pwm%d", j);

			/*
			 * DM355 has PWM3 IRQ at #28
			 */
			if (j == 3) {
				result =
					request_irq(IRQ_DM355_PWMINT3, pwm_isr,
						SA_INTERRUPT,
						pwm_dev_array[j]->name,
						pwm_dev_array[j]);
			} else {
				result = request_irq(IRQ_PWMINT0 + j,
						pwm_isr, SA_INTERRUPT,
						pwm_dev_array[j]->name,
						pwm_dev_array[j]);
			}

			if (result < 0) {
				printk("Cannot initialize IRQ \n");
				platform_device_unregister(&pwm_device[j]);
				driver_unregister(&pwm_driver[j]);
				kfree(pwm_dev_array[j]);
				return result;
			}

			pwm_dev_array[j]->pwm_clk = clk_get(NULL, *(name + j));
			if (IS_ERR(pwm_dev_array[j]->pwm_clk)) {
				printk("Cannot get clock\n");
				return -1;
			}
			clk_use(pwm_dev_array[j]->pwm_clk);
			clk_enable(pwm_dev_array[j]->pwm_clk);

			pwm_dev_array[j]->regs =
			    (davinci_pwmregsovly) IO_ADDRESS(DAVINCI_PWM0_BASE +
							     j * 0x400);
		}
	}
	return 0;
}

static void __exit pwm_exit(void)
{
	dev_t devno;
	unsigned int size, i;

	if (pwm_class != NULL) {
		size = pwm_device_count * pwm_minor_count;
		for (i = 0; i < size; i++) {
			platform_device_unregister(&pwm_device[i]);
			driver_unregister(&pwm_driver[i]);
			devno = MKDEV(pwm_major, pwm_minor_start + i);
			class_simple_device_remove(devno);
			if ((i == 3) && (cpu_is_davinci_dm355()))
				free_irq(IRQ_DM355_PWMINT3, pwm_dev_array[i]);
			else
				free_irq(IRQ_PWMINT0 + i, pwm_dev_array[i]);
			clk_unuse(pwm_dev_array[i]->pwm_clk);
			clk_disable(pwm_dev_array[i]->pwm_clk);
			kfree(pwm_dev_array[i]);
		}
		class_simple_destroy(pwm_class);
	}

	cdev_del(&c_dev);

	/* Release major/minor numbers */
	if (pwm_major != 0) {
		devno = MKDEV(pwm_major, pwm_minor_start);
		size = pwm_device_count * pwm_minor_count;
		unregister_chrdev_region(devno, size);
	}
}

module_init(pwm_init);
module_exit(pwm_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
