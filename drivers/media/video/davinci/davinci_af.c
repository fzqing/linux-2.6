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
/* davinci_af.c file */

/* Linux specific include files */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk  */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* File Structure... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>
#include <linux/interrupt.h>	/* For interrupt */
#include <linux/dma-mapping.h>	/* For class_simple_create */
#include <asm/uaccess.h>
#include <linux/wait.h>		/* FILES FOR WAIT QUEUE */
#include <linux/devfs_fs_kernel.h>	/* for devfs */
#include <asm/semaphore.h>	/* Semaphore */
#include <linux/device.h>
/* Driver include files */
#include <asm/arch/davinci_af.h>	/*Local Definitions */
#include <asm/arch/davinci_af_hw.h>	/* Local Definitions */

/* Module License */
MODULE_LICENSE("GPL");

/*Global structure for device */
struct af_device *af_dev_configptr;
static struct class_simple *af_class = NULL;

/* For registeration of charatcer device */
static struct cdev c_dev;

/* device structure to make entry in device */
static dev_t dev;
struct device *afdev = NULL;
/* inline function to free reserver pages  */
void inline af_free_pages(unsigned long addr, unsigned long bufsize)
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

/* Function to check paxel parameters */
int af_check_paxel(void)
{
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Check horizontal Count */
	if ((af_dev_configptr->config->paxel_config.hz_cnt
	     < AF_PAXEL_HORIZONTAL_COUNT_MIN)
	    || (af_dev_configptr->config->paxel_config.hz_cnt
		> AF_PAXEL_HORIZONTAL_COUNT_MAX)) {
		dev_err(afdev, "Error : Horizontal Count is incorrect");
		return -AF_ERR_HZ_COUNT;
	}

	/*Check Vertical Count */
	if ((af_dev_configptr->config->paxel_config.vt_cnt
	     < AF_PAXEL_VERTICAL_COUNT_MIN)
	    || (af_dev_configptr->config->paxel_config.vt_cnt
		> AF_PAXEL_VERTICAL_COUNT_MAX)) {
		dev_err(afdev, "Error : Vertical Count is incorrect");
		return -AF_ERR_VT_COUNT;
	}

	/*Check Height */
	if ((af_dev_configptr->config->paxel_config.height
	     < AF_PAXEL_HEIGHT_MIN)
	    || (af_dev_configptr->config->paxel_config.height
		> AF_PAXEL_HEIGHT_MAX)) {
		dev_err(afdev, "Error : Height is incorrect");
		return -AF_ERR_HEIGHT;
	}

	/*Check width */
	if ((af_dev_configptr->config->paxel_config.width < AF_PAXEL_WIDTH_MIN)
	    || (af_dev_configptr->config->paxel_config.width
		> AF_PAXEL_WIDTH_MAX)) {
		dev_err(afdev, "Error : Width is incorrect");
		return -AF_ERR_WIDTH;
	}

	/*Check Line Increment */
	if ((af_dev_configptr->config->paxel_config.line_incr
	     < AF_PAXEL_INCREMENT_MIN)
	    || (af_dev_configptr->config->paxel_config.line_incr
		> AF_PAXEL_INCREMENT_MAX)) {
		dev_err(afdev, "Error : Line Increment is incorrect");
		return -AF_ERR_INCR;
	}

	/*Check Horizontal Start */
	if ((af_dev_configptr->config->paxel_config.hz_start % 2 != 0)
	    || (af_dev_configptr->config->paxel_config.hz_start
		< (af_dev_configptr->config->iir_config.hz_start_pos + 2))
	    || (af_dev_configptr->config->paxel_config.hz_start
		> AF_PAXEL_HZSTART_MAX)
	    || (af_dev_configptr->config->paxel_config.hz_start
		< AF_PAXEL_HZSTART_MIN)) {
		dev_err(afdev, "Error : Horizontal Start is incorrect");
		return -AF_ERR_HZ_START;
	}

	/*Check Vertical Start */
	if ((af_dev_configptr->config->paxel_config.vt_start
	     < AF_PAXEL_VTSTART_MIN)
	    || (af_dev_configptr->config->paxel_config.vt_start
		> AF_PAXEL_VTSTART_MAX)) {
		dev_err(afdev, "Error : Vertical Start is incorrect");
		return -AF_ERR_VT_START;
	}

	dev_dbg(afdev, __FUNCTION__ "L\n");

	return 0;		/*Success */
}

/* Function to check IIR Coefficient */
int af_check_iir(void)
{
	int index;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Check for valid values of IIR coefficients */
	for (index = 0; index < AF_NUMBER_OF_COEF; index++) {
		/*Check Coefficient of set 0 */
		if ((af_dev_configptr->config->iir_config.coeff_set0[index])
		    > AF_COEF_MAX) {
			dev_err(afdev,
				"Error : Coefficient for set 0 is incorrect");
			return -AF_ERR_IIR_COEF;
		}

		/*Check coefficient of set 1 */
		if ((af_dev_configptr->config->iir_config.coeff_set1[index])
		    > AF_COEF_MAX) {
			dev_err(afdev,
				"Error : Coefficient for set 1 is incorrect");
			return -AF_ERR_IIR_COEF;
		}
	}

	/* Check IIRSH Value */
	if ((af_dev_configptr->config->iir_config.hz_start_pos < AF_IIRSH_MIN)
	    || (af_dev_configptr->config->iir_config.hz_start_pos >
		AF_IIRSH_MAX)) {
		dev_err(afdev, "Error : IIRSH is incorrect");
		return -AF_ERR_IIRSH;
	}

	dev_dbg(afdev, __FUNCTION__ "L\n");
	return 0;		/*Success */
}

/* Function to perform hardware set up */
int af_hardware_setup(void)
{
	int result;

	/*Size for buffer in bytes */
	int buff_size;
	unsigned long adr, size;
	unsigned int busyaf;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Get the value of PCR register */
	busyaf = AF_GET_PCR;

	/* Mask with BUSYAF bit */
	busyaf = busyaf & AF_BUSYAF;

	/* Shift it 15 times to get value of 1 or 0 */
	busyaf = busyaf >> 15;

	/*If busy bit is 1 then busy lock registers caanot be configured */
	if (busyaf == 1) {
		/* Hardware cannot be configure while engine is busy */
		dev_err(afdev, "AF_register_setup_ERROR : Engine Bus");
		dev_err(afdev, "\n Configuration cannot be done ");
		return -AF_ERR_ENGINE_BUSY;
	}

	/*Check IIR Coefficient and start Values */
	result = af_check_iir();
	if (result < 0)
		return result;

	/*Check Paxel Values */
	result = af_check_paxel();
	if (result < 0)
		return result;

	/*Check HMF Threshold Values */
	if (af_dev_configptr->config->hmf_config.threshold > AF_THRESHOLD_MAX) {
		dev_err(afdev, "Error : HMF Threshold is incorrect");
		return -AF_ERR_THRESHOLD;
	}

	/* Compute buffer size */
	buff_size =
	    (af_dev_configptr->config->paxel_config.hz_cnt + 1) *
	    (af_dev_configptr->config->paxel_config.vt_cnt + 1) * AF_PAXEL_SIZE;

	/*Deallocate the previosu buffers */
	/* free old buffers */
	if (af_dev_configptr->buff_old)
		af_free_pages((unsigned long)af_dev_configptr->buff_old,
			      af_dev_configptr->size_paxel);

	/* Free current buffer */
	if (af_dev_configptr->buff_curr)
		af_free_pages((unsigned long)af_dev_configptr->buff_curr,
			      af_dev_configptr->size_paxel);

	/* Free application buffers */
	if (af_dev_configptr->buff_app)
		af_free_pages((unsigned long)af_dev_configptr->buff_app,
			      af_dev_configptr->size_paxel);

	/* Reallocate the buffer as per new paxel configurations */
	/*Allocate memory for old buffer */
	af_dev_configptr->buff_old =
	    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				     get_order(buff_size));

	if (af_dev_configptr->buff_old == NULL)
		return -ENOMEM;

	/* allocate the memory for storing old statistics */
	adr = (unsigned long)af_dev_configptr->buff_old;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/* make sure the frame buffers
		   are never swapped out of memory */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/*Allocate memory for current buffer */
	af_dev_configptr->buff_curr =
	    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				     get_order(buff_size));

	/* Free the previously allocated buffer */
	if (af_dev_configptr->buff_curr == NULL) {
		if (af_dev_configptr->buff_old)
			af_free_pages((unsigned long)af_dev_configptr->
				      buff_old, buff_size);
		return -ENOMEM;
	}

	adr = (unsigned long)af_dev_configptr->buff_curr;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/* make sure the frame buffers
		   are never swapped out of memory */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/*Allocate memory for old buffer */
	af_dev_configptr->buff_app =
	    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				     get_order(buff_size));

	if (af_dev_configptr->buff_app == NULL) {

		/*Free the previously allocated buffer */
		if (af_dev_configptr->buff_curr)
			af_free_pages((unsigned long)af_dev_configptr->
				      buff_curr, buff_size);
		/*Free the previously allocated buffer */
		if (af_dev_configptr->buff_old)
			af_free_pages((unsigned long)af_dev_configptr->
				      buff_old, buff_size);
		return -ENOMEM;
	}

	adr = (unsigned long)af_dev_configptr->buff_app;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/* make sure the frame buffers
		   are never swapped out of memory */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	result = af_register_setup(af_dev_configptr);
	if (result < 0)
		return result;
	af_dev_configptr->size_paxel = buff_size;

	/*Set configuration flag to indicate HW setup done */
	af_dev_configptr->config->af_config = H3A_AF_ENABLE;

	dev_dbg(afdev, __FUNCTION__ "L\n");
	/*Success */
	return 0;
}

/* This function called when driver is opened.It creates Channel
 * Configuration Structure
 */
static int af_open(struct inode *inode, struct file *filp)
{
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/*Return if device is in use */
	if (af_dev_configptr->in_use == AF_IN_USE)
		return -EBUSY;
	af_dev_configptr->config = NULL;

	/* Allocate memory for Device Structure */
	af_dev_configptr->config = (struct af_configuration *)
	    kmalloc(sizeof(struct af_configuration)
		    , GFP_KERNEL);
	if (af_dev_configptr->config == NULL) {
		dev_err(afdev, "Error : Kmalloc fail\n");
		return -ENOMEM;
	}

	/* Initialize the wait queue */
	init_waitqueue_head(&(af_dev_configptr->af_wait_queue));

	/* Driver is in use */
	af_dev_configptr->in_use = AF_IN_USE;

	/* Hardware is not set up */
	af_dev_configptr->config->af_config = H3A_AF_DISABLE;
	af_dev_configptr->buffer_filled = 0;

	/* Initialize the semaphore */
	init_MUTEX(&(af_dev_configptr->read_blocked));
	dev_dbg(afdev, __FUNCTION__ "L\n");
	return 0;
}

/* This function called when driver is closed.
 * It will deallocate all the buffers.
 */
static int af_release(struct inode *inode, struct file *filp)
{
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Free all the buffers */
	/* free current buffer */
	if (af_dev_configptr->buff_curr)
		af_free_pages((unsigned long)af_dev_configptr->buff_curr,
			      af_dev_configptr->size_paxel);

	/*Free old buffer */
	if (af_dev_configptr->buff_old)
		af_free_pages((unsigned long)af_dev_configptr->buff_old,
			      af_dev_configptr->size_paxel);

	/* Free application buffer */
	if (af_dev_configptr->buff_app)
		af_free_pages((unsigned long)af_dev_configptr->buff_app,
			      af_dev_configptr->size_paxel);

	/*Release memory for configuration structure of this channel */
	af_dev_configptr->buff_curr = NULL;
	af_dev_configptr->buff_old = NULL;
	af_dev_configptr->buff_app = NULL;
	if (af_dev_configptr->config)
		kfree(af_dev_configptr->config);
	af_dev_configptr->config = NULL;

	/*Device is not in use */
	af_dev_configptr->in_use = AF_NOT_IN_USE;

	dev_dbg(afdev, __FUNCTION__ "L\n");

	return 0;
}
static void af_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}
static int __init af_probe(struct device *device)
{
	afdev = device;
	return 0;
}

static int af_remove(struct device *device)
{
	return 0;
}

/* This function will process IOCTL commands sent by the application and
 * control the device IO operations.
 */
static int af_ioctl(struct inode *inode, struct file *filep,
		    unsigned int cmd, unsigned long arg)
{
	struct af_configuration afconfig = *(af_dev_configptr->config);
	int result = 0;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Block the semaphore while ioctl is called */
	down_interruptible(&af_dev_configptr->read_blocked);

	/*Extract the type and number bitfields, and don't */
	/* decode wrong cmds */
	/*return ENOTTY (inappropriate ioctl) */
	if (_IOC_TYPE(cmd) != AF_MAGIC_NO) {
		/* Release the semaphore */
		up(&af_dev_configptr->read_blocked);
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > AF_IOC_MAXNR) {
		/* Release the semaphore */
		up(&af_dev_configptr->read_blocked);
		return -ENOTTY;
	}

	/*Use 'access_ok' to validate user space pointer */
	if (_IOC_DIR(cmd) & _IOC_READ)
		result =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		result =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (result) {
		/* Release the semaphore */
		up(&af_dev_configptr->read_blocked);
		return -EFAULT;
	}

	switch (cmd) {

		/* This ioctl is used to perform hardware */
		/* set up for AF Engine */
		/* It will configura all the registers. */
	case AF_S_PARAM:
		/*Copy params structure passed by user */
		if (copy_from_user(af_dev_configptr->config,
				   (struct af_configuration *)arg,
				   sizeof(struct af_configuration))) {
			/* Release the semaphore */
			up(&af_dev_configptr->read_blocked);
			return -EFAULT;
		}

		/*Call AF_hardware_setup to perform register configuration */
		result = af_hardware_setup();
		if (!result) {
			result = af_dev_configptr->size_paxel;
		} else {
			dev_err(afdev, "Error : AF_S_PARAM failed");
			*(af_dev_configptr->config) = afconfig;
		}
		break;

		/* This ioctl will get the paramters from application */
	case AF_G_PARAM:
		/*Check if Hardware is configured or not */
		if (af_dev_configptr->config->af_config == H3A_AF_ENABLE) {
			if (copy_to_user((struct af_configuration *)arg,
					 af_dev_configptr->config,
					 sizeof(struct af_configuration))) {
				up(&af_dev_configptr->read_blocked);
				return -EFAULT;
			} else
				result = af_dev_configptr->size_paxel;

		} else {
			dev_dbg(afdev, "Error : AF Hardware not configured.");
			result = -AF_ERR_SETUP;
		}

		break;

		/* This ioctl will enable AF Engine */
		/*if hardware configuration is done */
	case AF_ENABLE:
		/* Check if hardware is configured or not */
		if (af_dev_configptr->config->af_config == H3A_AF_DISABLE) {
			dev_err(afdev, "Error :  AF Hardware not configured.");
			result = -AF_ERR_SETUP;
		} else
			af_engine_setup(1);
		break;

		/* This ioctl will disable AF Engine */
	case AF_DISABLE:
		af_engine_setup(0);
		break;

	default:
		dev_err(afdev, "Error : Invalid IOCTL!");
		result = -ENOTTY;
		break;
	}

	/* Before returning increment semaphore */
	up(&af_dev_configptr->read_blocked);
	dev_dbg(afdev, __FUNCTION__ "L\n");
	return result;
}

/* Function will return the statistics to user */
ssize_t af_read(struct file * filep, char *kbuff, size_t size, loff_t * offset)
{
	void *buff_temp;
	int result = 0;
	int ret;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Semaphore will return immediately if read call is busy */
	ret = down_trylock(&(af_dev_configptr->read_blocked));
	if (ret != 0) {
		dev_err(afdev, "\n Read Call : busy");
		return -EBUSY;
	}

	/*If no of bytes specified by the user is less */
	/* than that of buffer return error */
	if (size < af_dev_configptr->size_paxel) {
		dev_err(afdev, "\n Error : Invalid buffer size");
		up(&(af_dev_configptr->read_blocked));
		return -1;	/* Return error to application */
	}

	/* The value of bufffer_filled flag determines
	   the status of statistics */
	if (af_dev_configptr->buffer_filled == 0) {
		dev_dbg(afdev, "Read call is blocked .......................");
		/* Block the read call until new statistics are available */
		/* or timer expires */
		/* Decrement the semaphore count */
		wait_event_interruptible_timeout(af_dev_configptr->
						 af_wait_queue,
						 af_dev_configptr->
						 buffer_filled, AF_TIMEOUT);
		dev_dbg(afdev,
			"\n Read Call Unblocked..........................");
	}
	if (af_dev_configptr->buffer_filled == 1) {
		/* New Statistics are available */
		/* Disable the interrupts while swapping the buffers */
		disable_irq(IRQ_H3AINT);
		dev_dbg(afdev, "\n Reading.............................");

		af_dev_configptr->buffer_filled = 0;

		/*Swap application buffer and old buffer */
		buff_temp = af_dev_configptr->buff_old;
		af_dev_configptr->buff_old = af_dev_configptr->buff_app;
		af_dev_configptr->buff_app = buff_temp;

		dev_dbg(afdev, "\n Reading Done.............................");

		/* Enable the interrupts  once swapping is done */
		enable_irq(IRQ_H3AINT);

		/* New Statistics are not availaible */
		/* copy the application buffer to user */
		/* Return the entire statistics to user */
		if (copy_to_user(kbuff, af_dev_configptr->buff_app,
				 af_dev_configptr->size_paxel)) {
			/* Release the semaphore in case of fault */
			up(&(af_dev_configptr->read_blocked));
			return -EFAULT;
		} else
			result = af_dev_configptr->size_paxel;
	}

	/* Release the seamphore */
	up(&(af_dev_configptr->read_blocked));
	dev_dbg(afdev, "\n Read APPLICATION  BUFFER %d",
		*((int *)((af_dev_configptr->buff_app))));
	dev_dbg(afdev, __FUNCTION__ "L\n");
	return result;
}

/* This function will handle the H3A interrupt. */
static irqreturn_t af_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	void *buff_temp;	/*Temporary buffer for swapping */
	int busyaf;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Get the value of PCR register */
	busyaf = AF_GET_PCR;

	/* If AF Engine has enabled, interrupt is not for AF */
	if ((busyaf & 0x01) == 0) {
		return -1;
	}

	/*Service  the Interrupt */
	/*Set buffer filled flag to indicate statistics are available */
	/*Swap current buffer and old buffer */
	buff_temp = af_dev_configptr->buff_curr;
	af_dev_configptr->buff_curr = af_dev_configptr->buff_old;
	af_dev_configptr->buff_old = buff_temp;

	/* Set AF Buf st to current register address */
	if (af_dev_configptr->buff_curr)
		af_set_address((unsigned long)
			       virt_to_phys(af_dev_configptr->buff_curr));

	/* Wake up read as new statistics are available */
	af_dev_configptr->buffer_filled = 1;
	wake_up(&(af_dev_configptr->af_wait_queue));
	dev_dbg(afdev, __FUNCTION__ "L\n");
	return IRQ_HANDLED;
}

/* File Operation Structure */
static struct file_operations af_fops = {
	.owner = THIS_MODULE,
	.open = af_open,
	.ioctl = af_ioctl,
	.read = af_read,
	.release = af_release
};
static struct platform_device afdevice = {
	.name = "davinci_af",
	.id = 2,
	.dev = {
		.release = af_platform_release,
		}
};

static struct device_driver af_driver = {
	.name = "davinci_af",
	.bus = &platform_bus_type,
	.probe = af_probe,
	.remove = af_remove,
};

/* Function to register the AF character device driver. */
int __init af_init(void)
{
	int err;
	int result = 0;
	result = AF_GET_CCDC_FMTCFG;
	result = result & AF_VPEN_MASK;
	result = result >> AF_FMTCG_VPEN;
	/* H3A Module cannot be inserted if CCDC
	   path for H3A is not registered */
	if (!(result)) {
		/* Module cannot be inserted if CCDC is not configured */
		printk("\n Davinci AF driver cannot be loaded");
		printk("\n VIDEO PORT is not enabled ");
		printk("\n CCDC needs to be configured");
		return -1;
	}
	/*Register the driver in the kernel. Get major number dynamically */
	result = alloc_chrdev_region(&dev, AF_MAJOR_NUMBER,
				     AF_NR_DEVS, DEVICE_NAME);
	if (result < 0) {
		printk("Error :  Could not register character device");
		return -ENODEV;
	}

	/*allocate memory for device structure and initialize it with 0 */
	af_dev_configptr =
	    (struct af_device *)kmalloc(sizeof(struct af_device), GFP_KERNEL);
	if (!af_dev_configptr) {
		printk("Error : kmalloc fail");
		unregister_chrdev_region(dev, AF_NR_DEVS);
		return -ENOMEM;

	}

	/* Initialize  character device */
	cdev_init(&c_dev, &af_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &af_fops;
	err = cdev_add(&c_dev, dev, 1);
	if (err) {
		printk("Error : Error in  Adding Davinci AF");
		unregister_chrdev_region(dev, AF_NR_DEVS);
		if (af_dev_configptr)
			kfree(af_dev_configptr);
		return -err;
	}

	/* Registe Character device */
	register_chrdev(MAJOR(dev), DEVICE_NAME, &af_fops);
	/* register driver as a platform driver */
	if (driver_register(&af_driver) != 0) {
		unregister_chrdev_region(dev, 1);
		cdev_del(&c_dev);
		return -EINVAL;
	}

	/* Register the drive as a platform device */
	if (platform_device_register(&afdevice) != 0) {
		driver_unregister(&af_driver);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), DEVICE_NAME);
		cdev_del(&c_dev);
		return -EINVAL;
	}
	af_class = class_simple_create(THIS_MODULE, "davinci_af");

	if (!af_class) {
		platform_device_unregister(&afdevice);
		printk("Error : Error in creating device class");
		unregister_chrdev_region(dev, AF_NR_DEVS);
		if (af_dev_configptr)
			kfree(af_dev_configptr);
		class_simple_device_remove(dev);
		class_simple_destroy(af_class);
		cdev_del(&c_dev);
		return -EIO;
	}

	/* make entry in the devfs */
	result =
	    devfs_mk_cdev(dev, S_IFCHR | S_IRUGO | S_IWUSR, "%s%d",
			  "davinci_af", 0);

	if (result < 0) {
		printk("Error : Error in devfs_register_chrdev");
		unregister_chrdev_region(dev, AF_NR_DEVS);
		if (af_dev_configptr)
			kfree(af_dev_configptr);
		class_simple_device_remove(dev);
		cdev_del(&c_dev);
		driver_unregister(&af_driver);
		platform_device_unregister(&afdevice);
		class_simple_destroy(af_class);
		unregister_chrdev(MAJOR(dev), DEVICE_NAME);
		return result;
	}

	/* register simple device class */
	class_simple_device_add(af_class, dev, NULL, "davinci_af");

	/* Set up the Interrupt handler for H3AINT interrupt */
	result =
	    request_irq(IRQ_H3AINT, af_isr, SA_SHIRQ, "dm644xh3a_af",
			(void *)af_dev_configptr);

	if (result != 0) {
		printk("Error : Request IRQ Failed");
		unregister_chrdev_region(dev, AF_NR_DEVS);
		if (af_dev_configptr)
			kfree(af_dev_configptr);
		class_simple_device_remove(dev);
		devfs_remove("%s%d", "davinci_af", 0);
		driver_unregister(&af_driver);
		platform_device_unregister(&afdevice);
		class_simple_destroy(af_class);
		cdev_del(&c_dev);
		unregister_chrdev(MAJOR(dev), DEVICE_NAME);
		return result;
	}

	/* Initialize device structure */
	memset(af_dev_configptr, 0, sizeof(struct af_device));

	af_dev_configptr->in_use = AF_NOT_IN_USE;
	af_dev_configptr->buffer_filled = 0;

	return 0;		/*Sucess */
}

/* This function is called by the kernel while unloading the driver.
 * It will unregister character device driver
 */
void __exit af_cleanup(void)
{

	/* Return if driver is busy */
	if (af_dev_configptr->in_use == AF_IN_USE) {
		printk("Error : Driver in use. Can't remove.");
		return;
	}

	/*Free device structure */
	if (af_dev_configptr)
		kfree(af_dev_configptr);

	unregister_chrdev_region(dev, AF_NR_DEVS);

	/* remove simple class device */
	class_simple_device_remove(dev);

	/* remove prev device from devfs */
	devfs_remove("%s%d", "davinci_af", 0);

	/* destroy simple class */
	class_simple_destroy(af_class);

	driver_unregister(&af_driver);
	platform_device_unregister(&afdevice);
	/*unregistering the driver from the kernel */
	cdev_del(&c_dev);
	/*Free interrupt line */
	free_irq(IRQ_H3AINT, af_dev_configptr);
	/* Unregiser Character device */
	unregister_chrdev(MAJOR(dev), DEVICE_NAME);
}

module_init(af_init)
    module_exit(af_cleanup)
    MODULE_LICENSE("GPL");
