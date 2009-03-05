/* *
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
/* davinci_aew.c file */

/* Kernel specific header files */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* File Structure fops... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* Types like size_t */
#include <linux/cdev.h>
#include <linux/interrupt.h>	/* For interrupt */
#include <linux/dma-mapping.h>	/* For class_simple_create */
#include <asm/uaccess.h>
#include <linux/wait.h>		/* FILES FOR WAIT QUEUE */
#include <linux/devfs_fs_kernel.h>	/* for devfs */
#include <asm/semaphore.h>
#include <linux/device.h>
/* Driver Header Files */
#include <asm/arch/davinci_aew.h>	/* Local Definitions */
#include <asm/arch/davinci_aew_hw.h>	/* Local Definitions */

/*Global structure*/
struct aew_device *aew_dev_configptr;
struct device *aewdev = NULL;
/* device structure to make entry in device*/
static dev_t dev;
static struct class_simple *aew_class = NULL;

/* For registeration of charatcer device*/
static struct cdev c_dev;

/* Module License*/
MODULE_LICENSE("GPL");

/* inline function to free reserver pages  */
void inline aew_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long tempaddr;
	unsigned long size;
	tempaddr = addr;
	if (!addr)
		return;
	size = PAGE_SIZE << (get_order(bufsize));
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(tempaddr, get_order(bufsize));
}

/* Function to check Window Parmeters */
int aew_check_window(void)
{
	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Check Window Horizontal Count */
	if ((aew_dev_configptr->config->window_config.hz_cnt
	     < AEW_WINDOW_HORIZONTAL_COUNT_MIN)
	    || (aew_dev_configptr->config->window_config.hz_cnt
		> AEW_WINDOW_HORIZONTAL_COUNT_MAX)) {
		dev_err(aewdev, "Error :  Horizontal Count is incorrect\n");
		return -AEW_ERR_HZ_COUNT;
	}

	/* Check Window Vertical Count */
	if ((aew_dev_configptr->config->window_config.vt_cnt
	     < AEW_WINDOW_VERTICAL_COUNT_MIN)
	    || (aew_dev_configptr->config->window_config.vt_cnt
		> AEW_WINDOW_VERTICAL_COUNT_MAX)) {
		dev_err(aewdev, "Error :  Vertical Count is incorrect\n");
		return -AEW_ERR_VT_COUNT;
	}

	/* Check Window Vertical Start */
	if ((aew_dev_configptr->config->window_config.vt_start
	     < AEW_WINDOW_START_MIN)
	    || (aew_dev_configptr->config->window_config.vt_start
		> AEW_WINDOW_START_MAX)) {
		dev_err(aewdev, "Error : Vertical Start is incorrect\n");
		return -AEW_ERR_VT_START;
	}

	/* Check Window Horizontal Start */
	if ((aew_dev_configptr->config->window_config.hz_start
	     < AEW_WINDOW_START_MIN)
	    || (aew_dev_configptr->config->window_config.hz_start
		> AEW_WINDOW_START_MAX)) {
		dev_err(aewdev, "Error :  Horizontal Start is incorrect\n");
		return -AEW_ERR_HZ_START;
	}

	/*Check Horizontal Line Increment */
	if ((aew_dev_configptr->config->window_config.hz_line_incr
	     < AEW_WINDOW_LINE_INCREMENT_MIN)
	    || (aew_dev_configptr->config->window_config.hz_line_incr
		> AEW_WINDOW_LINE_INCREMENT_MAX)) {
		dev_err(aewdev,
			"Error :  Horizontal Line Increment value is incorrect\n");
		return -AEW_ERR_HZ_INCR;
	}

	/* Check Vertical Line Increment */
	if ((aew_dev_configptr->config->window_config.vt_line_incr
	     < AEW_WINDOW_LINE_INCREMENT_MIN)
	    || (aew_dev_configptr->config->window_config.vt_line_incr
		> AEW_WINDOW_LINE_INCREMENT_MAX)) {
		dev_err(aewdev,
			"Error :  Vertical Line Increment is  incorrect\n");
		return -AEW_ERR_VT_INCR;
	}

	/* Check Window Height */
	if ((aew_dev_configptr->config->window_config.height
	     < AEW_WINDOW_HEIGHT_MIN)
	    || (aew_dev_configptr->config->window_config.height
		> AEW_WINDOW_HEIGHT_MAX)) {

		dev_err(aewdev, "Error :  Height is incorrect\n");
		return -AEW_ERR_HEIGHT;
	}

	/* Check Window Width */
	if ((aew_dev_configptr->config->window_config.width
	     < AEW_WINDOW_WIDTH_MIN)
	    || (aew_dev_configptr->config->window_config.width
		> AEW_WINDOW_WIDTH_MAX)) {
		dev_err(aewdev, "Error :  Width is incorrect\n");
		return -AEW_ERR_WIDTH;
	}

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return 0;		/*Success */
}

/* Function to check Black Window Parmeters */
int aew_check_black_window(void)
{
	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Check Black Window Height */
	if ((aew_dev_configptr->config->blackwindow_config.height
	     < AEW_WINDOW_HEIGHT_MIN)
	    || (aew_dev_configptr->config->blackwindow_config.height
		> AEW_WINDOW_HEIGHT_MAX)) {
		dev_err(aewdev, "Error :  Height is incorrect\n");
		return -AEW_ERR_BLKWIN_HEIGHT;
	}

	/* Check Black Window Vertical Start */
	if ((aew_dev_configptr->config->blackwindow_config.vt_start
	     < AEW_WINDOW_START_MIN)
	    || (aew_dev_configptr->config->blackwindow_config.vt_start
		> AEW_WINDOW_START_MAX)) {
		dev_err(aewdev, "Error :  Vertical Start is incorrect\n");
		return -AEW_ERR_BLKWIN_VT_START;
	}

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return 0;		/*Success */
}

/* Function to perform hardware Configuration */
int aew_hardware_setup(void)
{
	int result;
	/*Size for buffer in bytes */
	int buff_size = 0;
	unsigned long adr;
	unsigned long size;
	unsigned int busyaew;
	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Get the value of PCR register */
	busyaew = AEW_GET_PCR;

	/* Mask with BUSYAF bit */
	busyaew = busyaew & AEW_BUSYAEWB;

	/* Shift it 18 times to get value of 1 or 0 */
	busyaew = busyaew >> AEW_BUSYAEWB_SHIFT;

	/* If H3A Engine is busy then return */
	if (busyaew == 1) {
		dev_err(aewdev, "\n Error : AEW Engine is busy");
		return -AEW_ERR_ENGINE_BUSY;
	}

	/*Check Window Parameters */
	result = aew_check_window();
	dev_dbg(aewdev, "Result =  %d\n", result);
	if (result < 0) {
		dev_err(aewdev, "Error : Window Parameter are incorrect \n");
		return result;
	}

	/*Check Black Window Parameters */
	result = aew_check_black_window();
	if (result < 0) {
		dev_err(aewdev,
			"Error : Black Window Parameters are incorrect \n");
		return result;
	}

	/*Check Saturuation limit */
	if (aew_dev_configptr->config->saturation_limit > AEW_SAT_LIMIT_MAX) {
		dev_err(aewdev, "Error : Saturation limit is incorrect\n");
		return -AEW_ERR_LIMIT;
	}

	/* Deallocate the previously allocated buffers */
	if (aew_dev_configptr->buff_old)
		aew_free_pages((unsigned long)aew_dev_configptr->buff_old,
			       aew_dev_configptr->size_window);

	if (aew_dev_configptr->buff_curr)
		aew_free_pages((unsigned long)aew_dev_configptr->
			       buff_curr, aew_dev_configptr->size_window);

	if (aew_dev_configptr->buff_app)
		aew_free_pages((unsigned long)aew_dev_configptr->
			       buff_app, aew_dev_configptr->size_window);

	/*Allocat the buffers as per the new buffer size */
	/*Allocate memory for old buffer */
	buff_size = (aew_dev_configptr->config->window_config.hz_cnt + 1)
	    * (aew_dev_configptr->config->window_config.vt_cnt +
	       1) * AEW_WINDOW_SIZE;

	aew_dev_configptr->buff_old =
	    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				     get_order(buff_size));

	if (aew_dev_configptr->buff_old == NULL)
		return -ENOMEM;

	/*Make pges reserved so that they will be swapped out */
	adr = (unsigned long)aew_dev_configptr->buff_old;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/* make sure the frame buffers
		   are never swapped out of memory */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/*Allocate memory for current buffer */
	aew_dev_configptr->buff_curr =
	    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				     get_order(buff_size));

	if (aew_dev_configptr->buff_curr == NULL) {

		/*Free all  buffer that are allocated */
		if (aew_dev_configptr->buff_old)
			aew_free_pages((unsigned long)aew_dev_configptr->
				       buff_old, buff_size);
		return -ENOMEM;
	}

	/*Make pges reserved so that they will be swapped out */
	adr = (unsigned long)aew_dev_configptr->buff_curr;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/* make sure the frame buffers
		   are never swapped out of memory */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/*Allocate memory for application buffer */
	aew_dev_configptr->buff_app =
	    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				     get_order(buff_size));

	if (aew_dev_configptr->buff_app == NULL) {
		/*Free all  buffer that were allocated previously */
		if (aew_dev_configptr->buff_old)
			aew_free_pages((unsigned long)aew_dev_configptr->
				       buff_old, buff_size);
		if (aew_dev_configptr->buff_curr)
			aew_free_pages((unsigned long)aew_dev_configptr->
				       buff_curr, buff_size);
		return -ENOMEM;
	}

	/*Make pges reserved so that they will be swapped out */
	adr = (unsigned long)aew_dev_configptr->buff_app;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/* make sure the frame buffers
		   are never swapped out of memory */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/* Set the registers */
	aew_set_register(aew_dev_configptr);
	aew_dev_configptr->size_window = buff_size;
	aew_dev_configptr->config->aew_config = H3A_AEW_ENABLE;

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return 0;		/*Success */
}

/* This Function is called when driver is opened */
static int aew_open(struct inode *inode, struct file *filp)
{

	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/*Return if Device is in use (Single Channel Support is provided) */
	if (aew_dev_configptr->in_use == AEW_IN_USE)
		return -EBUSY;

	/* Set the aew_dev_configptr structure */
	aew_dev_configptr->config = NULL;

	/* Allocate memory for configuration  structure of this channel */
	aew_dev_configptr->config = (struct aew_configuration *)
	    kmalloc(sizeof(struct aew_configuration), GFP_KERNEL);

	if (aew_dev_configptr->config == NULL) {
		dev_err(aewdev, "Error : Kmalloc fail\n");
		return -ENOMEM;
	}

	/* Initiaze the wait queue */
	init_waitqueue_head(&(aew_dev_configptr->aew_wait_queue));

	/*Device is in use */
	aew_dev_configptr->in_use = AEW_IN_USE;

	/* No Hardware Set up done */
	aew_dev_configptr->config->aew_config = H3A_AEW_DISABLE;

	/* No statistics are available */
	aew_dev_configptr->buffer_filled = 0;

	/* Set Window Size to 0 */
	aew_dev_configptr->size_window = 0;

	/* Initialize the semaphore */
	init_MUTEX(&(aew_dev_configptr->read_blocked));

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return 0;
}
static void aew_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}
static int __init aew_probe(struct device *device)
{
	aewdev = device;
	return 0;
}

static int aew_remove(struct device *device)
{
	return 0;
}

/* This Function is called when driver is closed */
static int aew_release(struct inode *inode, struct file *filp)
{
	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* The Application has closed device so device is not in use */
	aew_dev_configptr->in_use = AEW_NOT_IN_USE;

	/*Release memory for configuration structure of this channel */
	if (aew_dev_configptr->config)
		kfree(aew_dev_configptr->config);

	/* Free Old Buffer */
	if (aew_dev_configptr->buff_old)
		aew_free_pages((unsigned long)aew_dev_configptr->buff_old,
			       aew_dev_configptr->size_window);

	/* Free Current Buffer */
	if (aew_dev_configptr->buff_curr)
		aew_free_pages((unsigned long)aew_dev_configptr->
			       buff_curr, aew_dev_configptr->size_window);

	/* Free Application Buffer */
	if (aew_dev_configptr->buff_app)
		aew_free_pages((unsigned long)aew_dev_configptr->buff_app,
			       aew_dev_configptr->size_window);

	aew_dev_configptr->buff_old = NULL;
	aew_dev_configptr->buff_curr = NULL;
	aew_dev_configptr->config = NULL;
	aew_dev_configptr->buff_app = NULL;

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return 0;
}

/* This function will process IOCTL commands sent by the application and
 * control the devices IO operations.
 */
static int aew_ioctl(struct inode *inode, struct file *filep,
		     unsigned int cmd, unsigned long arg)
{
	/* Stores Previous Configurations */
	struct aew_configuration aewconfig = *(aew_dev_configptr->config);
	int result = 0;

	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Decrement the semaphore */
	down_interruptible(&aew_dev_configptr->read_blocked);

	/*Extract the type and number bitfields, and don't decode wrong cmds: */
	/*verify the magic number */
	if (_IOC_TYPE(cmd) != AEW_MAGIC_NO) {
		up(&aew_dev_configptr->read_blocked);
		return -ENOTTY;
	}

	/*verify the command number */
	if (_IOC_NR(cmd) > AEW_IOC_MAXNR) {
		/* Release semaphore  in case of fault */
		up(&aew_dev_configptr->read_blocked);
		return -ENOTTY;
	}

	/* check for the permission of the operation */
	if (_IOC_DIR(cmd) & _IOC_READ)
		result =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		result =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (result) {
		/* Release semaphore in case of fault */
		up(&aew_dev_configptr->read_blocked);
		return -EFAULT;
	}

	/* Switch according to IOCTL command */
	switch (cmd) {
		/* This ioctl is used to perform hardware set up 
		   and will set all the regiseters */
		/*for AF engine */
	case AEW_S_PARAM:

		/*Copy config structure passed by user */
		if (copy_from_user(aew_dev_configptr->config,
				   (struct aew_configuration *)arg,
				   sizeof(struct aew_configuration))) {
			*(aew_dev_configptr->config) = aewconfig;
			up(&aew_dev_configptr->read_blocked);
			return -EFAULT;
		}

		/*Call aew_hardware_setup to perform register configuration */
		result = aew_hardware_setup();
		if (!result) {	/* Hardware Set up is successful */
			/*Return the no of bytes required for buffer */
			result = aew_dev_configptr->size_window;
		} else {
			/*Change Configuration Structure to original */
			*(aew_dev_configptr->config) = aewconfig;
			dev_err(aewdev, "Error : AEW_S_PARAM  failed\n");
		}

		break;

		/* This ioctl is used to return parameters in user space */
	case AEW_G_PARAM:
		if (aew_dev_configptr->config->aew_config == H3A_AEW_ENABLE) {
			if (copy_to_user
			    ((struct aew_configuration *)arg,
			     aew_dev_configptr->config,
			     sizeof(struct aew_configuration))) {
				up(&aew_dev_configptr->read_blocked);
				return -EFAULT;
			} else
				result = aew_dev_configptr->size_window;
		} else {
			dev_err(aewdev,
				"Error : AEW Hardware is not configured.\n");
			result = -AEW_ERR_SETUP;
		}
		break;

		/* This ioctl is used to enable AEW Engine */
	case AEW_ENABLE:
		/*Enable AEW Engine if Hardware set up is done */
		if (aew_dev_configptr->config->aew_config == H3A_AEW_DISABLE) {
			dev_err(aewdev,
				"Error : AEW Hardware is not configured.\n");
			result = -AEW_ERR_SETUP;
		} else
			/* Enable AF Engine */
			aew_engine_setup(1);
		break;

		/* This ioctl is used to disable AEW Engine */
	case AEW_DISABLE:
		/* Disable AEW Engine */
		aew_engine_setup(0);
		break;

		/* Invalid Command */
	default:
		dev_err(aewdev, "Error: It should not come here!!\n");
		result = -ENOTTY;
		break;
	}

	/*Release the semaphore */
	up(&aew_dev_configptr->read_blocked);
	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return result;
}

/* This function will return statistics to user */
static ssize_t aew_read(struct file *filep, char *kbuff,
			size_t size, loff_t * offset)
{
	void *buffer_temp;
	int result = 0;
	int ret;
	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Semaphore will return immediately if read call is busy */
	ret = down_trylock(&(aew_dev_configptr->read_blocked));
	if (ret != 0) {
		dev_dbg(aewdev, "Read Call : busy  : %d\n", ret);
		return -EBUSY;
	}

	/* First Check the size given by user */
	if (size < aew_dev_configptr->size_window) {
		/* Return Failure to applicaiton */
		/*if size is less than required size */
		dev_dbg(aewdev, "Error : Invalid size of buffer\n");
		up(&(aew_dev_configptr->read_blocked));
		return -1;
	}

	/* The value of buffer_filled flag determines 
	   the status of statistics */
	if (aew_dev_configptr->buffer_filled == 0) {
		/* Decrement the semaphore */
		dev_dbg(aewdev, "READ CALL IS BLOCKED............\n");
		/* Block the read call */
		wait_event_interruptible_timeout(aew_dev_configptr->
						 aew_wait_queue,
						 aew_dev_configptr->
						 buffer_filled, AEW_TIMEOUT);
		dev_dbg(aewdev, "Read Call is unbloked and waking up.......\n");
		dev_dbg(aewdev, "Buffer Filled.... %d\n",
			aew_dev_configptr->buffer_filled);
	}

	if (aew_dev_configptr->buffer_filled == 1) {
		/* Disable the interrupts and then swap the buffers */
		disable_irq(IRQ_H3AINT);
		dev_dbg(aewdev, "READING............\n");

		/* New Statistics are availaible */
		aew_dev_configptr->buffer_filled = 0;

		/*Swap application buffer and old buffer */
		buffer_temp = aew_dev_configptr->buff_old;
		aew_dev_configptr->buff_old = aew_dev_configptr->buff_app;
		aew_dev_configptr->buff_app = buffer_temp;

		/* Interrupts are enabled */
		enable_irq(IRQ_H3AINT);

		/* Copy the entire statistics located in application 
		   buffer to user space */
		if (copy_to_user(kbuff, aew_dev_configptr->buff_app,
				 aew_dev_configptr->size_window)) {
			dev_err(aewdev, "Error : Read Fault\n");
			up(&(aew_dev_configptr->read_blocked));
			return -EFAULT;
		} else
			result = aew_dev_configptr->size_window;

		dev_dbg(aewdev, "Reading Done........................\n");
	}

	dev_dbg(aewdev, "APP BUFF VALUE %x\n",
		(*((unsigned int *)(aew_dev_configptr->buff_app))));

	/*Increment the semaphore */
	up(&(aew_dev_configptr->read_blocked));

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return result;
}

/* This function will handle interrupt generated by H3A Engine. */
static irqreturn_t aew_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	/* Busy AF Bit */
	unsigned int busyaew;
	/* Temporary Buffer for Swapping */
	void *buffer_temp;

	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Get the value of PCR register */
	busyaew = AEW_GET_PCR;

	/* If AEW engine is not enabled, interrupt is not for AEW */
	if (((busyaew & 0x10000) >> 16) == 0) {
		/*printk("busyaf\n"); */
		return -1;
	}

	/*Interrupt is generated by AEW, so Service the Interrupt */
	/*Swap current buffer and old buffer */
	buffer_temp = aew_dev_configptr->buff_curr;
	aew_dev_configptr->buff_curr = aew_dev_configptr->buff_old;
	aew_dev_configptr->buff_old = buffer_temp;

	/* Set the AEWBUFSTAT REgister to current buffer Address */
	aew_set_address((unsigned
			 long)(virt_to_phys(aew_dev_configptr->buff_curr)));

	/*Set buffer filled flag to indicate statistics are available */
	aew_dev_configptr->buffer_filled = 1;

	/*new statistics are available */
	/* Wake up the read call */
	wake_up(&(aew_dev_configptr->aew_wait_queue));

	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return IRQ_HANDLED;
}

/* file Operation Structure*/
static struct file_operations aew_fops = {
	.owner = THIS_MODULE,
	.open = aew_open,
	.read = aew_read,
	.ioctl = aew_ioctl,
	.release = aew_release,
};
static struct platform_device aewdevice = {
	.name = "davinci_aew",
	.id = 2,
	.dev = {
		.release = aew_platform_release,
		}
};

static struct device_driver aew_driver = {
	.name = "davinci_aew",
	.bus = &platform_bus_type,
	.probe = aew_probe,
	.remove = aew_remove,
};

/* Function to register Character Device driver */
int __init aew_init(void)
{
	int err;
	int result;

	/* Checking CCDC Registers */
	/* Module cannot be inserted if data flow path for 
	   h3a is not enabled */
	result = AEW_GET_CCDC_FMTCFG;
	/* Mask with VPEN Bit to check path */
	result = result & AEW_VPEN_MASK;
	result = result >> AEW_FMTCG_VPEN;
	if (!(result)) {
		printk
		    ("\n Davici AEW Driver cannot be loaded\n VIDEO PORT is not enabled");
		printk("Data Flow path from CCDC is disabled\n");
		return -1;
	}

	/*Register the driver in the kernel. 
	   Get major number dynamically */
	result = alloc_chrdev_region(&dev, AEW_MAJOR_NUMBER,
				     AEW_NR_DEVS, DEVICE_NAME);
	if (result < 0) {
		printk("Error : Could not register character device\n");
		return -ENODEV;

	}

	/*allocate memory for device structure and initialize it with 0 */
	aew_dev_configptr =
	    (struct aew_device *)kmalloc(sizeof(struct aew_device), GFP_KERNEL);
	if (!aew_dev_configptr) {
		printk("Error : kmalloc fail\n");
		unregister_chrdev_region(dev, AEW_NR_DEVS);
		return -ENOMEM;
	}

	/* Initialize of character device */
	cdev_init(&c_dev, &aew_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &aew_fops;

	/* Add character device */
	err = cdev_add(&c_dev, dev, 1);
	if (err) {
		printk("Error : Adding DavinciAEW DeviceFailed\n");

		unregister_chrdev_region(dev, AEW_NR_DEVS);

		if (aew_dev_configptr)
			kfree(aew_dev_configptr);

		return -err;
	}

	/* Register the character device driver */
	register_chrdev(MAJOR(dev), DEVICE_NAME, &aew_fops);
	/* register driver as a platform driver */
	if (driver_register(&aew_driver) != 0) {
		unregister_chrdev_region(dev, 1);
		cdev_del(&c_dev);
		return -EINVAL;
	}

	/* Register the drive as a platform device */
	if (platform_device_register(&aewdevice) != 0) {
		driver_unregister(&aew_driver);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), DEVICE_NAME);
		cdev_del(&c_dev);
		return -EINVAL;
	}
	aew_class = class_simple_create(THIS_MODULE, "davinci_aew");

	if (!aew_class) {

		printk("aew_init: error in creating device class\n");

		unregister_chrdev_region(dev, AEW_NR_DEVS);
		platform_device_unregister(&aewdevice);

		if (aew_dev_configptr)
			kfree(aew_dev_configptr);

		cdev_del(&c_dev);

		unregister_chrdev(MAJOR(dev), DEVICE_NAME);

		return -EIO;
	}

	/* Make entry in the devfs */
	result = devfs_mk_cdev(dev, S_IFCHR | S_IRUGO | S_IWUSR,
			       "%s%d", "davinci_aew", 0);

	if (result < 0) {
		printk("Error : Error in  devfs_register_chrdev\n");

		unregister_chrdev_region(dev, AEW_NR_DEVS);

		if (aew_dev_configptr)
			kfree(aew_dev_configptr);

		class_simple_destroy(aew_class);

		cdev_del(&c_dev);

		driver_unregister(&aew_driver);
		platform_device_unregister(&aewdevice);
		unregister_chrdev(MAJOR(dev), DEVICE_NAME);

		return result;
	}

	/*Register simple device class */
	class_simple_device_add(aew_class, dev, NULL, "davinci_aew");

	/* Set up the Interrupt handler for H3AINT interrupt */
	result =
	    request_irq(IRQ_H3AINT, aew_isr, SA_SHIRQ, "dm644xh3a_aew",
			(void *)aew_dev_configptr);

	if (result < 0) {
		unregister_chrdev_region(dev, AEW_NR_DEVS);

		if (aew_dev_configptr)
			kfree(aew_dev_configptr);

		class_simple_device_remove(dev);

		devfs_remove("%s%d", "davinci_aew", 0);

		class_simple_destroy(aew_class);

		cdev_del(&c_dev);

		driver_unregister(&aew_driver);
		platform_device_unregister(&aewdevice);
		unregister_chrdev(MAJOR(dev), DEVICE_NAME);

		printk("\n Error : Request IRQ Failed\n");
		return result;
	}

	/* Initialize the device structure */
	memset(aew_dev_configptr, 0, sizeof(struct aew_device));

	/* Device is not in use */
	aew_dev_configptr->in_use = AEW_NOT_IN_USE;

	/* No statistics available */
	aew_dev_configptr->buffer_filled = 0;

	return 0;		/*Sucess */
}

/*
=====================aew_cleanup===========================
*/
/* This Function is called by the kernel while unloading the driver
This will unregister the
 Character Device Driver
*/
void __exit aew_cleanup(void)
{
	/* Device is in use */
	if (aew_dev_configptr->in_use == AEW_IN_USE) {
		printk("Error : Driver in use");
		return;
	}

	/*Free device structure */
	if (aew_dev_configptr)
		kfree(aew_dev_configptr);
	aew_dev_configptr = NULL;
	unregister_chrdev_region(dev, AEW_NR_DEVS);

	/* remove simple class device */
	class_simple_device_remove(dev);

	/* remove aew device from devfs */
	devfs_remove("%s%d", "davinci_aew", 0);

	/* destroy simple class */
	class_simple_destroy(aew_class);

	driver_unregister(&aew_driver);
	platform_device_unregister(&aewdevice);
	/* Free the interrupt Handler */
	free_irq(IRQ_H3AINT, aew_dev_configptr);
	cdev_del(&c_dev);

	/*unregistering the driver from the kernel */
	unregister_chrdev(MAJOR(dev), DEVICE_NAME);
}

module_init(aew_init)
    module_exit(aew_cleanup)
