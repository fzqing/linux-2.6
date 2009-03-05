/*
 * adapter.c
 *
 * Xilinx GPIO Adapter component to interface GPIO component to Linux
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2006 (c)MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * This driver is a bit unusual in that it is composed of two logical
 * parts where one part is the OS independent code and the other part is
 * the OS dependent code.  Xilinx provides their drivers split in this
 * fashion.  This file represents the Linux OS dependent part known as
 * the Linux adapter.  The other files in this directory are the OS
 * independent files as provided by Xilinx with no changes made to them.
 * The names exported by those files begin with XGpio_.  All functions
 * in this file that are called by Linux have names that begin with
 * xgpio_.  Any other functions are static helper functions.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/xilinx_devices.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "xgpio.h"
#include "xgpio_ioctl.h"

struct xgpio_instance {
	struct list_head link;
	unsigned long base_phys;	/* GPIO base address - physical */
	unsigned long remap_size;
	u32 device_id;
	/*
	 * The underlying OS independent code needs space as well.  A
	 * pointer to the following XGpio structure will be passed to
	 * any XGpio_ function that requires it.  However, we try to treat the
	 * data as an opaque object in this file (meaning that we never
	 * reference any of the fields inside of the structure).
	 */
	XGpio gpio;
};

/* SAATODO: This function will be moved into the Xilinx code. */
/****************************************************************************/
/**
* Get the input/output direction of all discrete signals.
*
* @param InstancePtr is a pointer to an XGpio instance to be worked on.
*
* @return Current copy of the tristate (direction) register.
*
* @note
*
* None
*
*****************************************************************************/
u32 XGpio_GetDataDirection(XGpio * InstancePtr, unsigned Channel)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	return XGpio_mReadReg(InstancePtr->BaseAddress,
			      (Channel - 1) * XGPIO_CHAN_OFFSET +
			      XGPIO_TRI_OFFSET);
}

inline int XGpio_IsReady(XGpio * InstancePtr)
{
	return InstancePtr->IsReady == XCOMPONENT_IS_READY;
}

static LIST_HEAD(inst_list);
static DECLARE_RWSEM(inst_list_sem);

/*******************
 * The misc device *
 *******************/

static int xgpio_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int xgpio_release(struct inode *inode, struct file *file)
{
	return 0;
}

/*
 * To access the 1st channel of GPIO with id = N set ioctl_data->device to 2*N.
 * ioctl_data->device of 2*N+1 accesses the 2nd channel (for DUAL GPIO IPs).
 * ioctl_setup returns the channel number to work with (1 or 2) or error code
 * (negative).
 */
static int ioctl_setup(unsigned long arg,
		       struct xgpio_ioctl_data *ioctl_data,
		       struct xgpio_instance **match)
{
	struct list_head *entry;
	struct xgpio_instance *inst;
	int dev_id, chan;

	if (copy_from_user(ioctl_data, (void *)arg, sizeof(*ioctl_data)))
		return -EFAULT;

	dev_id = ioctl_data->device / 2;
	chan = ioctl_data->device % 2;

	down_read(&inst_list_sem);

	list_for_each(entry, &inst_list) {
		inst = list_entry(entry, struct xgpio_instance, link);
		if (dev_id == inst->device_id) {
			up_read(&inst_list_sem);
			if (!inst->gpio.IsDual && chan) {
				return -ENODEV;
			} else if (XGpio_IsReady(&inst->gpio)) {
				*match = inst;
				return chan + 1;
			} else {
				return -EAGAIN;
			}
		}
	}

	up_read(&inst_list_sem);
	return -ENODEV;
}

static int xgpio_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct xgpio_ioctl_data ioctl_data;
	struct xgpio_instance *inst;
	int status;
	u32 r;

	switch (cmd) {
	case XGPIO_IN:
		status = ioctl_setup(arg, &ioctl_data, &inst);
		if (status < 0)
			return status;

		/*
		 * Ensure that the GPIO bits in the mask are tristated.
		 * Unlike IBM OCP GPIO, one needs to set the bits in the
		 * Tristate (direction) Register to make the corresponding
		 * GPIOs to be inputs.
		 */
		r = XGpio_GetDataDirection(&inst->gpio, status);
		XGpio_SetDataDirection(&inst->gpio, status,
				       r | ioctl_data.mask);

		ioctl_data.data = (XGpio_DiscreteRead(&inst->gpio, status)
				   & ioctl_data.mask);
		if (copy_to_user((struct xgpio_ioctl_data *)arg,
				 &ioctl_data, sizeof(ioctl_data))) {
			return -EFAULT;
		}
		break;

	case XGPIO_OUT:
		status = ioctl_setup(arg, &ioctl_data, &inst);
		if (status < 0)
			return status;

		/* Get the prior value. */
		r = XGpio_DiscreteRead(&inst->gpio, status);
		/* Clear the bits that we're going to put in. */
		r &= ~ioctl_data.mask;
		/* Set the bits that were provided. */
		r |= (ioctl_data.mask & ioctl_data.data);

		XGpio_DiscreteWrite(&inst->gpio, status, r);

		/*
		 * Ensure that the GPIO bits in the mask are not tristated.
		 * Unlike IBM OCP GPIO, one needs to clear the bits in the
		 * Tristate (direction) Register to make the corresponding
		 * GPIOs to be outputs.
		 */
		r = XGpio_GetDataDirection(&inst->gpio, status);
		XGpio_SetDataDirection(&inst->gpio, status,
				       r & ~ioctl_data.mask);

		break;

	case XGPIO_TRISTATE:
		status = ioctl_setup(arg, &ioctl_data, &inst);
		if (status < 0)
			return status;

		/* Get the prior value. */
		r = XGpio_GetDataDirection(&inst->gpio, status);
		/* Clear the bits that we're going to put in. */
		r &= ~ioctl_data.mask;
		/*
		 * Set the bits that were provided.
		 * Note that "1" makes the corresponding GPIO pin to tristate.
		 * To keep the interface the same as for IBM OCP GPIO
		 * we invert ioctl_data.data before writing them to the
		 * Tristate Register.
		 */
		r |= (ioctl_data.mask & ~ioctl_data.data);

		XGpio_SetDataDirection(&inst->gpio, status, r);
		break;

	case XGPIO_OPEN_DRAIN:
		/* This can be implemented by configuring a pin as
		 * output when it is "0", and tristating a pin when
		 * it is "1". Now just fall trough. */

	default:
		return -ENOIOCTLCMD;

	}
	return 0;
}

/*
 * We get to all of the GPIOs through one minor number.  Here's the
 * miscdevice that gets registered for that minor number.
 */

static struct file_operations xgpio_fops = {
      owner:THIS_MODULE,
      ioctl:xgpio_ioctl,
      open:xgpio_open,
      release:xgpio_release
};

static struct miscdevice miscdev = {
      minor:XGPIO_MINOR,
      name:"xgpio",
      fops:&xgpio_fops
};

/******************************
 * The platform device driver *
 ******************************/

#define DRIVER_NAME "xilinx_gpio"

static int xgpio_probe(struct device *dev)
{
	XGpio_Config xgpio_config;
	struct xgpio_instance *xgpio_inst;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *regs_res;
	void *v_addr;
	int retval;

	if (!dev)
		return -EINVAL;

	memset(&xgpio_config, 0, sizeof(XGpio_Config));
	xgpio_inst = kzalloc(sizeof(struct xgpio_instance), GFP_KERNEL);
	if (!xgpio_inst) {
		printk(KERN_ERR
		       "%s #%d: Couldn't allocate device private record\n",
		       miscdev.name, pdev->id);
		return -ENOMEM;
	}

	/* Map the control registers in */
	regs_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_res || (regs_res->end - regs_res->start + 1 < 8)) {
		printk(KERN_ERR "%s #%d: Couldn't get registers resource\n",
		       miscdev.name, pdev->id);
		retval = -EFAULT;
		goto failed1;
	}

	xgpio_inst->remap_size = regs_res->end - regs_res->start + 1;
	if (!request_mem_region(regs_res->start, xgpio_inst->remap_size,
				DRIVER_NAME)) {
		printk(KERN_ERR "Couldn't lock memory region at 0x%08lX\n",
		       regs_res->start);
		retval = -EBUSY;
		goto failed2;
	}

	v_addr = ioremap(regs_res->start, xgpio_inst->remap_size);
	if (!v_addr) {
		printk(KERN_ERR "Couldn't ioremap memory at 0x%08lX\n",
		       regs_res->start);
		retval = -EFAULT;
		goto failed3;
	}

	xgpio_inst->base_phys = regs_res->start;
	/* The 1st GPIO channel uses */
	xgpio_inst->device_id = pdev->id;
	xgpio_config.DeviceId = pdev->id;
	xgpio_config.IsDual =
	    ((unsigned)(dev->platform_data) & XGPIO_IS_DUAL) ? 1 : 0;

	/* Tell the Xilinx code to bring this GPIO interface up. */
	if (XGpio_CfgInitialize(&xgpio_inst->gpio, &xgpio_config,
				(u32) v_addr) != XST_SUCCESS) {
		printk(KERN_ERR "%s #%d: Could not initialize instance.\n",
		       miscdev.name, pdev->id);
		retval = -ENODEV;
		goto failed3;
	}

	/* Add XGpio instance to the list */
	down_write(&inst_list_sem);
	if (list_empty(&inst_list)) {
		retval = misc_register(&miscdev);
		if (retval != 0) {
			up_write(&inst_list_sem);
			printk(KERN_ERR "%s #%d: Could not register miscdev.\n",
			       miscdev.name, pdev->id);
			goto failed3;
		}
	}
	list_add_tail(&xgpio_inst->link, &inst_list);
	up_write(&inst_list_sem);

	printk(KERN_INFO "%s #%d at 0x%08lX mapped to 0x%08X\n",
	       miscdev.name, xgpio_inst->device_id,
	       xgpio_inst->base_phys, xgpio_inst->gpio.BaseAddress);

	return 0;		/* success */

      failed3:
	iounmap((void *)(xgpio_config.BaseAddress));

      failed2:
	release_mem_region(regs_res->start, xgpio_inst->remap_size);

      failed1:
	kfree(xgpio_inst);

	return retval;
}

static int xgpio_remove(struct device *dev)
{
	struct list_head *entry;
	struct xgpio_instance *xgpio_inst = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	if (!dev)
		return -EINVAL;

	/* Set xgpio_inst based on pdev->id match */

	down_read(&inst_list_sem);
	list_for_each(entry, &inst_list) {
		xgpio_inst = list_entry(entry, struct xgpio_instance, link);
		if (pdev->id == xgpio_inst->device_id) {
			break;
		} else {
			xgpio_inst = NULL;
		}
	}
	up_read(&inst_list_sem);

	if (xgpio_inst == NULL)
		return -ENODEV;

	/* Remove the private data from the list */
	down_write(&inst_list_sem);
	list_del(&xgpio_inst->link);
	if (list_empty(&inst_list)) {
		misc_deregister(&miscdev);
	}
	up_write(&inst_list_sem);

	iounmap((void *)(xgpio_inst->gpio.BaseAddress));

	release_mem_region(xgpio_inst->base_phys, xgpio_inst->remap_size);

	kfree(xgpio_inst);

	return 0;		/* success */
}

static struct device_driver xgpio_driver = {
	.name = DRIVER_NAME,
	.bus = &platform_bus_type,

	.probe = xgpio_probe,
	.remove = xgpio_remove
};

static int __init xgpio_init(void)
{
	/*
	 * No kernel boot options used,
	 * so we just need to register the driver
	 */
	return driver_register(&xgpio_driver);
}

static void __exit xgpio_cleanup(void)
{
	driver_unregister(&xgpio_driver);
}

module_init(xgpio_init);
module_exit(xgpio_cleanup);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");
