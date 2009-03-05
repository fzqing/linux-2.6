/*
 * adapter.c
 *
 * Xilinx Adapter component to interface touchscreen component to Linux
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2006 (c) MontaVista, Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * Parts of this code were written using drivers/misc/ucb1x00-ts.c as a
 * reference.  ucb1x00-ts.c is:
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 */

/*
 * This driver is a bit unusual in that it is composed of two logical
 * parts where one part is the OS independent code and the other part is
 * the OS dependent code.  Xilinx provides their drivers split in this
 * fashion.  This file represents the Linux OS dependent part known as
 * the Linux adapter.  The other files in this directory are the OS
 * independent files as provided by Xilinx with no changes made to them.
 * The names exported by those files begin with XTouchscreen_.  All
 * functions in this file that are called by Linux have names that begin
 * with xts_.  Any other functions are static helper functions.
 */

/*
 * CURRENT STATE OF AFFAIRS:
 * The underlying driver and hardware have some quirks that need to be
 * ironed out.  I've seen good X and Y values with zero Z's.  That is
 * why this driver synthesizes Z as either zero or 255 and uses
 * XTouchscreen_GetPosition_2D instead of XTouchscreen_GetPosition_3D.
 * There seem to be linearity problems and troubles with dead zones
 * around the edges of the screen.  I believe that this could be a
 * result of the issues outlined in the Burr-Brown (now Texas
 * Instruments) "Touch Screen Controller Tips" application note for the
 * ADS7846 touch screen controller which is used by the Xilinx ML300.
 * The Xilinx touchscreen controls the ADS7846, but it doesn't give
 * software enough access to the ADS7846 to implement the strategies
 * outlined in the aforementioned application note.  Xilinx is going to
 * look into improving this.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/moduleparam.h>
#include <linux/xilinx_devices.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <xbasic_types.h>
#include "xtouchscreen.h"
#include "xtouchscreen_i.h"

#define XTS_NAME	"xilinx_ts"	/* Must match the one in virtex.c */
#define XTS_DAEMON	"xilinx_tsd"

#define NR_EVENTS		16	/* Must be a power of two */

#define XTS_DEFAULT_MAJOR	124
#define XTS_DEFAULT_MINOR	0	/* The minors start from this value */
#define XTS_MINORS		4	/* Allocate 4 minors for this driver */

static int xts_major = XTS_DEFAULT_MAJOR;
static int xts_minor = XTS_DEFAULT_MINOR;
static int xts_no_minors = XTS_MINORS;

module_param(xts_major, int, S_IRUGO);

/*
 * The definition of the following structure is copied from ucb1x00-ts.c so the
 * touchscreen will "just work" with tslib, etc.
 */
/*
 * This structure is nonsense - millisecs is not very useful
 * since the field size is too small.  Also, we SHOULD NOT
 * be exposing jiffies to user space directly.
 */
struct ts_event {
	u16 pressure;
	u16 x;
	u16 y;
	u16 pad;
	struct timeval stamp;
};

/* Our private per interface data. */
struct xts_data {
	struct list_head link;
	int index;			/* Which interface is this */
	u32 phys_addr;			/* Saved physical address */
	ulong remap_size;
	unsigned int irq;		/* device IRQ number */
	struct semaphore sem;
	int use_count;			/* How many things have us open */
	wait_queue_head_t read_wait;
	struct fasync_struct *fasync;
	struct cdev cdev;		/* Char device structure */
	struct ts_event events[NR_EVENTS];
	u8 evt_head;
	u8 evt_tail;
	/*
	 * The underlying OS independent code needs space as well.  A
	 * pointer to the following XTouchscreen structure will be
	 * passed to any XTouchscreen_ function that requires it.
	 * However, we treat the data as an opaque object in this file
	 * (meaning that we never reference any of the fields inside of
	 * the structure).
	 */
	XTouchscreen Touchscreen;
	/*
	 * The following bit fields are used to keep track of what
	 * all has been done to initialize the xts_dev to make
	 * error handling out of probe() easier.
	 */
	unsigned int pen_is_down:1;
	unsigned int pen_was_down:1;	/* Used to determine pen-up */
};

/*
 * List of devices we're handling and a lock to give us atomic access in
 * anticipation of someday being able to hot add/remove.  If this is done, the
 * code that shoots the list in xts_thread will need to have protection added.
 */
static LIST_HEAD(inst_list);
static DECLARE_RWSEM(inst_list_sem);

struct task_struct *xts_task = NULL;	/* xts_thread task pointer */
static struct completion task_sync;	/* xts_thread start/stop syncing */
static char task_shutdown = 0;	/* Set to non-zero when task should quit.  */
static wait_queue_head_t irq_wait;	/* xts_thread waiting for interrupt */

#define event_pending(dev)	((volatile u8)(dev)->evt_head != (dev)->evt_tail)
#define event_get(dev)		((dev)->events + (dev)->evt_tail)
#define event_pull(dev)		((dev)->evt_tail = ((dev)->evt_tail + 1) & (NR_EVENTS - 1))
#define event_clear(dev)	((dev)->evt_head = (dev)->evt_tail = 0)

/*
 * Add a new touch event as long as either x, y or z was different
 * from the last touch event.
 */
static inline void event_add(struct xts_data *dev, u16 pressure, u16 x, u16 y)
{
	int next_head, prior_head;

	next_head = (dev->evt_head + 1) & (NR_EVENTS - 1);
	prior_head = (dev->evt_head - 1) & (NR_EVENTS - 1);
	if (next_head != dev->evt_tail
	    && (dev->events[prior_head].pressure != pressure
		|| dev->events[prior_head].x != x
		|| dev->events[prior_head].y != y))
	{
		dev->events[dev->evt_head].pressure = pressure;
		dev->events[dev->evt_head].x = x;
		dev->events[dev->evt_head].y = y;
		do_gettimeofday(&dev->events[dev->evt_head].stamp);
		dev->evt_head = next_head;

		if (dev->fasync)
			kill_fasync(&dev->fasync, SIGIO, POLL_IN);
		wake_up_interruptible(&dev->read_wait);
	}
}

/*******************************************************************************
 * This configuration stuff should become unnecessary after EDK version 8.x is
 * released.
 ******************************************************************************/

static DECLARE_MUTEX(cfg_sem);
static XTouchscreen_Config * p_xts_cfg;

/*
 * Lookup the device configuration based on the unique device ID.
 * Prototyped in xtouchscreen.h.
 * We rely on that p_xts_cfg is set properly right before XTouchscreen_Initialize()
 * calls XTouchscreen_LookupConfig().
 */
XTouchscreen_Config * XTouchscreen_LookupConfig(u16 DeviceId)
{
	return p_xts_cfg;
}

static XStatus XTouchscreen_CfgInitialize(XTouchscreen * InstancePtr,
					  XTouchscreen_Config * CfgPtr)
{
	XStatus retval;

	down(&cfg_sem);
	p_xts_cfg = CfgPtr;
	retval = XTouchscreen_Initialize(InstancePtr, 0);
	up(&cfg_sem);

	return retval;
}

/*
 * This routine is registered with the OS as the function to call when
 * the touchscreen interrupts.  It in turn, calls the Xilinx OS
 * independent interrupt function.  The Xilinx OS independent interrupt
 * function will in turn call any callbacks that we have registered for
 * various conditions.
 */
static irqreturn_t xts_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct xts_data *dev = (struct xts_data *) dev_id;

	XTouchscreen_InterruptHandler(&dev->Touchscreen);
	return IRQ_HANDLED;
}

static void xts_handler(void *CallbackRef, u32 Event, unsigned int EventData)
{
	struct xts_data *dev = (struct xts_data *) CallbackRef;

	switch (Event) {
	case XTOUCHSCREEN_EVENT_PEN_DOWN:
		dev->pen_is_down = 1;
		wake_up(&irq_wait);	/* Schedule our task */
		break;
	case XTOUCHSCREEN_EVENT_PEN_UP:
		dev->pen_is_down = 0;
		break;
	default:
		printk(KERN_ERR "%s #%d: Unknown event %d.\n", XTS_NAME,
		       dev->index, Event);
		break;
	}
}

/*
 * This task waits until at least one touchscreen is touched.  It then loops
 * digitizing and generating events until no touchscreens are being touched.
 */
static int xts_thread(void *arg)
{
	int any_pens_down;
	unsigned long flags;
	struct list_head *entry;
	struct xts_data *inst;
	struct task_struct *tsk = current;
	DECLARE_WAITQUEUE(wait, tsk);
	xts_task = tsk;

	daemonize(XTS_DAEMON, XTS_NAME);
	/* only want to receive SIGKILL */
	disallow_signal(SIGKILL);

	complete(&task_sync);

	add_wait_queue(&irq_wait, &wait);
	any_pens_down = 0;
	for (;;) {
		/*
		 * Block waiting for interrupt or if any pens are down, either
		 * an interrupt or timeout to sample again.
		 */
		set_current_state(TASK_INTERRUPTIBLE);
		if (any_pens_down)
			schedule_timeout(HZ / 100);
		while (signal_pending(tsk)) {
			siginfo_t info;

			/* Only honor the signal if we're cleaning up */
			if (task_shutdown)
				goto exit;
			/*
			 * Someone else sent us a kill (probably the
			 * shutdown scripts "Sending all processes the
			 * KILL signal").  Just dequeue it and ignore
			 * it.
			 */
			spin_lock_irqsave(&current->sighand->siglock, flags);
			dequeue_signal(current, &current->blocked, &info);
			spin_unlock_irqrestore(&current->sighand->siglock, flags);
		}
		schedule();

		down_read(&inst_list_sem);

		any_pens_down = 0;
		list_for_each(entry, &inst_list) {
			inst = list_entry(entry, struct xts_data, link);
			if (inst->pen_is_down) {
				u32 x, y;
				XTouchscreen_GetPosition_2D(&inst->Touchscreen,
							    &x, &y);
				event_add(inst, 255, (u16) x, (u16) y);
				inst->pen_was_down = 1;
				any_pens_down = 1;
			} else if (inst->pen_was_down) {
				event_add(inst, 0, 0, 0);
				inst->pen_was_down = 0;
			}
		}

		up_read(&inst_list_sem);
	}

exit:
	remove_wait_queue(&irq_wait, &wait);

	xts_task = NULL;
	complete_and_exit(&task_sync, 0);
}

static ssize_t xts_read(struct file *filp, char *buffer, size_t count, loff_t * ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct xts_data *dev = filp->private_data;
	char *ptr = buffer;
	int err = 0;

	add_wait_queue(&dev->read_wait, &wait);
	while (count >= sizeof (struct ts_event)) {
		err = -ERESTARTSYS;
		if (signal_pending(current))
			break;

		if (event_pending(dev)) {
			struct ts_event *evt = event_get(dev);
			err = copy_to_user(ptr, evt, sizeof (struct ts_event));
			event_pull(dev);

			if (err)
				break;

			ptr += sizeof (struct ts_event);
			count -= sizeof (struct ts_event);
			continue;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		err = -EAGAIN;
		if (filp->f_flags & O_NONBLOCK)
			break;
		schedule();
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&dev->read_wait, &wait);

	return ptr == buffer ? err : ptr - buffer;
}

static unsigned int xts_poll(struct file *filp, poll_table * wait)
{
	struct xts_data *dev = (struct xts_data *)(filp->private_data);
	int ret = 0;

	poll_wait(filp, &dev->read_wait, wait);
	if (event_pending(dev))
		ret = POLLIN | POLLRDNORM;

	return ret;
}

static int xts_open(struct inode *inode, struct file *filp)
{
	struct xts_data *dev;

	dev = container_of(inode->i_cdev, struct xts_data, cdev);
	filp->private_data = dev; /* for other methods */

	if (down_interruptible(&dev->sem))
		return -EINTR;

	if (dev->use_count++ == 0) {
		/* This was the first opener; we need to get the IRQ. */
		int retval;
		retval = request_irq(dev->irq, xts_interrupt, 0, XTS_NAME, dev);
		if (retval) {
			printk(KERN_ERR
			       "%s #%d: Could not allocate interrupt %d.\n",
			       XTS_NAME, dev->index, dev->irq);
			dev->use_count--;
			up(&dev->sem);
			return retval;
		}
	}

	up(&dev->sem);

	return 0;
}

static int xts_release(struct inode *inode, struct file *filp)
{
	struct xts_data *dev = (struct xts_data *)(filp->private_data);

	if (down_interruptible(&dev->sem))
		return -EINTR;

	if (--dev->use_count == 0) {
		/* This was the last closer; get rid of the IRQ. */
		disable_irq(dev->irq);
		free_irq(dev->irq, dev);
	}

	up(&dev->sem);
	return 0;
}

static int xts_fasync(int fd, struct file *filp, int on)
{
	struct xts_data *dev = (struct xts_data *)(filp->private_data);

	return fasync_helper(fd, filp, on, &dev->fasync);
}

static struct class_simple *xilinx_ts_class;

static struct file_operations xts_fops = {
	.owner   = THIS_MODULE,
	.read    = xts_read,
	.poll    = xts_poll,
	.open    = xts_open,
	.release = xts_release,
	.fasync  = xts_fasync,
};

/******************************
 * The platform device driver *
 ******************************/

static int xts_probe(struct device *dev)
{
	dev_t devt;
	XTouchscreen_Config xts_cfg;
	struct platform_device *pdev = to_platform_device(dev);
	struct xts_data *drvdata;
	struct resource *irq_res, *regs_res;
	unsigned long remap_size;
	struct class_device *class;
	int retval;

	if (!dev)
		return -EINVAL;

	devt = MKDEV(xts_major, xts_minor + pdev->id);

	drvdata = kzalloc(sizeof(struct xts_data), GFP_KERNEL);
	if (!drvdata) {
		printk(KERN_ERR "%s #%d: Could not allocate device.\n",
		       XTS_NAME, pdev->id);
		return -ENOMEM;
	}
	dev_set_drvdata(dev, (void *)drvdata);
	init_MUTEX(&drvdata->sem);
	init_waitqueue_head(&drvdata->read_wait);

	/* Find irq number, map the control registers in */

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	regs_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_res || !irq_res) {
		printk(KERN_ERR "%s #%d: IO resource(s) not found\n",
		       XTS_NAME, pdev->id);
		retval = -EFAULT;
		goto failed1;
	}
	drvdata->irq = irq_res->start;

	remap_size = regs_res->end - regs_res->start + 1;
	if (!request_mem_region(regs_res->start, remap_size, XTS_NAME)) {
		printk(KERN_ERR
		       "%s #%d: Couldn't lock memory region at 0x%08lX\n",
		       XTS_NAME, pdev->id, regs_res->start);
		retval = -EBUSY;
		goto failed1;
	}
	drvdata->remap_size = remap_size;
	drvdata->phys_addr = regs_res->start;
	drvdata->index = pdev->id;
	xts_cfg.DeviceId = pdev->id;
	xts_cfg.BaseAddress = (u32) ioremap(regs_res->start, remap_size);
	if (xts_cfg.BaseAddress == 0) {
		printk(KERN_ERR
		       "%s #%d: Couldn't ioremap memory at 0x%08lX\n",
		       XTS_NAME, pdev->id, regs_res->start);
		retval = -EFAULT;
		goto failed2;
	}

	/* Tell the Xilinx code to bring this touchscreen interface up. */
	if (XTouchscreen_CfgInitialize(&drvdata->Touchscreen,
				       &xts_cfg) != XST_SUCCESS) {
		printk(KERN_ERR "%s #%d: Could not initialize device.\n",
		       XTS_NAME, pdev->id);
		retval = -ENODEV;
		goto failed3;
	}

	/* Set up the interrupt handler. */
	XTouchscreen_SetHandler(&drvdata->Touchscreen, xts_handler, drvdata);

	printk(KERN_INFO "%s #%d at 0x%08X mapped to 0x%08X\n",
	       XTS_NAME, pdev->id,
	       drvdata->phys_addr, drvdata->Touchscreen.BaseAddress);

	cdev_init(&drvdata->cdev, &xts_fops);
	drvdata->cdev.owner = THIS_MODULE;
	drvdata->cdev.ops = &xts_fops;
	retval = cdev_add(&drvdata->cdev, devt, 1);
	if (retval) {
		printk(KERN_ERR "%s #%d: xts_setup_cdev failed\n",
		       XTS_NAME, pdev->id);
		goto failed3;
	}

	retval = devfs_mk_cdev(devt, S_IFCHR | S_IRUGO | S_IWUGO,
			       "touchscreen/xilinx_ts%d", pdev->id);
	if (retval) {
		printk(KERN_ERR "%s #%d: devfs_mk_cdev failed\n",
		       XTS_NAME, pdev->id);
		goto failed4;
	}

	class = class_simple_device_add(xilinx_ts_class, 
					devt, NULL, "xilinx_ts%d", pdev->id);
	if (IS_ERR(class)) {
		printk(KERN_ERR "%s #%d: class_simple_device_add failed\n",
		       XTS_NAME, pdev->id);
		retval = PTR_ERR(class);
		goto failed5;
	}

	/* Add the touchscreen instance to the list */
	down_write(&inst_list_sem);
	if (list_empty(&inst_list)) {
		init_completion(&task_sync);
		init_waitqueue_head(&irq_wait);
		retval = kernel_thread(xts_thread, 0, 0);
		if (retval < 0) {
			up_write(&inst_list_sem);
			goto failed6;
		}
		wait_for_completion(&task_sync);
	}
	list_add_tail(&drvdata->link, &inst_list);
	up_write(&inst_list_sem);

	return 0;	/* success */

failed6:
	class_simple_device_remove(devt);

failed5:
	devfs_remove("touchscreen/xilinx_ts%d", pdev->id);

failed4:
	cdev_del(&drvdata->cdev);

failed3:
	iounmap((void *) (xts_cfg.BaseAddress));

failed2:
	release_mem_region(regs_res->start, remap_size);

failed1:
	kfree(drvdata);

	return retval;
}

static int xts_remove(struct device *dev)
{
	struct xts_data *drvdata;

	if (!dev)
		return -EINVAL;

	drvdata = (struct xts_data *) dev_get_drvdata(dev);

	/* Remove the private data from the list */
	down_write(&inst_list_sem);
	list_del(&drvdata->link);
	if(list_empty(&inst_list)) {
		task_shutdown = 1;
		send_sig(SIGKILL, xts_task, 1);
		wait_for_completion(&task_sync);
	}
	up_write(&inst_list_sem);

	class_simple_device_remove(MKDEV(xts_major, xts_minor+drvdata->index));
	devfs_remove("touchscreen/xilinx_ts%d", drvdata->index);
	cdev_del(&drvdata->cdev);
	iounmap((void *) (drvdata->Touchscreen.BaseAddress));
	release_mem_region(drvdata->phys_addr, drvdata->remap_size);
	kfree(drvdata);
	dev_set_drvdata(dev, NULL);

	return 0;	/* success */
}

static struct device_driver xts_driver = {
	.name		= XTS_NAME,
	.bus		= &platform_bus_type,
	.probe		= xts_probe,
	.remove		= xts_remove
};

static int __init xts_init(void)
{
	int retval;
	dev_t devt;

	if (xts_major) {
		devt = MKDEV(xts_major, xts_minor);
		retval = register_chrdev_region(devt, xts_no_minors, XTS_NAME);
	} else {
		retval = alloc_chrdev_region(&devt, xts_minor, xts_no_minors,
		                                XTS_NAME);
		xts_major = MAJOR(devt);
	}
	if (retval < 0) {
		xts_major = 0;
		return retval;
	}

	xilinx_ts_class = class_simple_create(THIS_MODULE, XTS_NAME);
	if (IS_ERR(xilinx_ts_class)) {
		retval = PTR_ERR(xilinx_ts_class);
		goto failed1;
	}

	devfs_mk_dir("touchscreen");

	retval = driver_register(&xts_driver);
	if (retval) {
		goto failed2;
	}

	return 0;	/* success */

failed2:
	class_simple_destroy(xilinx_ts_class);
	devfs_remove("touchscreen");

failed1:
	unregister_chrdev_region(devt, xts_no_minors);

	return retval;
}

static void __exit xts_cleanup(void)
{
	dev_t devt = MKDEV(xts_major, xts_minor);

	driver_unregister(&xts_driver);

	class_simple_destroy(xilinx_ts_class);
	devfs_remove("touchscreen");
	unregister_chrdev_region(devt, xts_no_minors);
}

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Xilinx touchscreen driver");
MODULE_LICENSE("GPL");

module_init(xts_init);
module_exit(xts_cleanup);
