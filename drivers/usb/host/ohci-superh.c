/*
 * OHCI HCD (Host Controller Driver) for SuperH internal USB.
 *
 * Copyright (C)  Kuniki Nakamura  (Dec 25, 2004)
 *   Modified to support SH7727 USB Host Controller
 * Copyright (C)  Shiho Shigematsu (Dec 28, 2004)
 *   Modified to support SH7727 USB Host Controller
 * Copyright (C)  Takashi Kusuda   (Jan  1, 2005)
 *   Modified to support SH7720 USB Host Controller snd support
 *     Kernel version 2.6.11.6.
 *
 * This file is licenced under the GPL.
 */

#include "ohci-superh.h"

extern int usb_disabled(void);

static void superh_start_hc(struct platform_device *dev)
{
	printk(KERN_DEBUG __FILE__
	       ": starting SuperH internal OHCI USB Controller\n");

#if defined(CONFIG_CPU_SUBTYPE_SH7720)
	writew((readw(PORT_PHCR) & PHCR_MASK), PORT_PHCR);	/* usb*_pwr_en */
	writew(UTRCTL_DATA, UTRCTL_ADDR);
	writeb((readb(STBCR3_ADDR) | USBH_OFF), STBCR3_ADDR);
	writew(UCLKCR_DATA, UCLKCR_ADDR);
	writeb((readb(STBCR3_ADDR) & USBH_ON), STBCR3_ADDR);
	udelay(1000);

#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
	writew(((readw(PORT_PECR) & PECR_MASK1) | PECR_MASK2), PORT_PECR);	/* PECR */
	writew((readw(PORT_PDCR) & PDCR_MASK), PORT_PDCR);	/* PDCR */
	writew(EXPFC_DATA, EXPFC_ADDR);	/* EXPFC */
	writeb((readb(STBCR3) | USBH_OFF), STBCR3);
	writeb(EXCPGCR_DATA, EXCPGCR_ADDR);	/* EXCPGCR */
	writeb((readb(STBCR3) & USBH_ON), STBCR3);
	udelay(1000);

#if defined(CONFIG_SH_SOLUTION_ENGINE_LIGHT)
	/* USB root hub. individual oc sense/ power control */
	writel((5 << 24) | (1 << 11) | (1 << 8), SH_OHCI_DESCRIPTOR_A);	/* HcRhDescA */
	writel(0, SH_OHCI_PORT_STATUS1);	/* port1 clear */
	writel(0, SH_OHCI_PORT_STATUS2);	/* port2 clear */
	writel(((1 << 2) << 16) | (0 << 2), SH_OHCI_DESCRIPTOR_B);	/* HcRhDescB */
#endif
#endif
}

static void superh_stop_hc(struct platform_device *dev)
{
	printk(KERN_DEBUG __FILE__
	       ": stopping SuperH internal OHCI USB Controller\n");
}

static irqreturn_t usb_hcd_superh_hcim_irq(int irq, void *__hcd,
					   struct pt_regs *r)
{
	struct usb_hcd *hcd = __hcd;

	return usb_hcd_irq(irq, hcd, r);
}

void usb_hcd_superh_remove(struct usb_hcd *, struct platform_device *);

static u64 superh_dmamask = 0xffffffffUL;

int usb_hcd_superh_probe(const struct hc_driver *driver,
			 struct usb_hcd **hcd_out, struct platform_device *dev)
{
	int retval;
	struct usb_hcd *hcd = 0;

	if (!request_mem_region
	    (SH_OHCI_REGS_BASE, sizeof(struct ohci_regs), hcd_name)) {
		dbg("controller already in use");
		return -EBUSY;
	}

	superh_start_hc(dev);

	hcd = usb_create_hcd(driver);
	if (hcd == NULL) {
		dbg("hcd_alloc failed");
		retval = -ENOMEM;
		goto err1;
	}
	ohci_hcd_init(hcd_to_ohci(hcd));

	hcd->driver = (struct hc_driver *)driver;
	hcd->irq = SH_OHCI_IRQ;
	hcd->regs = (void *)SH_OHCI_REGS_BASE;
	dev->dev.dma_mask = &superh_dmamask;
	hcd->self.controller = &dev->dev;

	retval = hcd_buffer_create(hcd);
	if (retval != 0) {
		dbg("pool alloc fail");
		goto err1;
	}

	retval = request_irq(hcd->irq, usb_hcd_superh_hcim_irq, SA_INTERRUPT,
			     hcd->driver->description, hcd);
	if (retval != 0) {
		dbg("request_irq failed");
		retval = -EBUSY;
		goto err2;
	}

	info("%s (SuperH USB) at 0x%p, irq %d\n",
	     hcd->driver->description, hcd->regs, hcd->irq);

	hcd->self.bus_name = "superh";

	usb_register_bus(&hcd->self);

	if ((retval = driver->start(hcd)) < 0) {
		usb_hcd_superh_remove(hcd, dev);
		return retval;
	}
	*hcd_out = hcd;
	return 0;

      err2:
	hcd_buffer_destroy(hcd);
      err1:
	kfree(hcd);
	superh_stop_hc(dev);
	release_mem_region((unsigned long)hcd->regs, sizeof(struct ohci_regs));
	return retval;
}

void usb_hcd_superh_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	pr_debug("remove: %s, state %x", hcd->self.bus_name, hcd->state);

	if (in_interrupt())
		BUG();

	hcd->state = USB_STATE_QUIESCING;

	pr_debug("%s: roothub graceful disconnect", hcd->self.bus_name);
	usb_disconnect(&hcd->self.root_hub);

	hcd->driver->stop(hcd);
	hcd->state = USB_STATE_HALT;

	free_irq(hcd->irq, hcd);
	hcd_buffer_destroy(hcd);

	usb_deregister_bus(&hcd->self);

	superh_stop_hc(dev);
	release_mem_region((unsigned long)hcd->regs, sizeof(struct ohci_regs));
}

static int ohci_superh_reset(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);

	ohci->regs = hcd->regs;
	ohci->next_statechange = jiffies;
	return ohci_init(ohci);
}

static int __devinit ohci_superh_start(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	int ret;

	ohci_dbg(ohci, "ohci_superh_start, ohci:%p", ohci);

	ohci->hcca = dma_alloc_coherent(hcd->self.controller,
					sizeof *ohci->hcca, &ohci->hcca_dma, 0);
	if (!ohci->hcca)
		return -ENOMEM;

	memset(ohci->hcca, 0, sizeof(struct ohci_hcca));
	if ((ret = ohci_mem_init(ohci)) < 0) {
		ohci_stop(hcd);
		return ret;
	}
	ohci->regs = hcd->regs;

	if (ohci_run(ohci) < 0) {
		ohci_err(ohci, "can't start\n");
		ohci_stop(hcd);
		return -EBUSY;
	}
	create_debug_files(ohci);

#ifdef	DEBUG
	ohci_dump(ohci, 1);
#endif
	return 0;
}

#ifdef	CONFIG_PM

static int ohci_superh_suspend(struct usb_hcd *hcd, u32 state)
{
	return 0;
}

static int ohci_superh_resume(struct usb_hcd *hcd)
{
	return 0;
}

#endif				/* CONFIG_PM */

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_superh_hc_driver = {
	.description = hcd_name,
	.product_desc = "SuperH OHCI",
	.hcd_priv_size = sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ohci_irq,
	.flags = HCD_USB11,

	/*
	 * basic lifecycle operations
	 */
	.reset = ohci_superh_reset,
	.start = ohci_superh_start,
#ifdef	CONFIG_PM
	.suspend = ohci_superh_suspend,
	.resume = ohci_superh_resume,
#endif
	.stop = ohci_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ohci_urb_enqueue,
	.urb_dequeue = ohci_urb_dequeue,
	.endpoint_disable = ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ohci_hub_status_data,
	.hub_control = ohci_hub_control,
#ifdef	CONFIG_USB_SUSPEND
	.hub_suspend = ohci_hub_suspend,
	.hub_resume = ohci_hub_resume,
#endif
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_superh_drv_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = NULL;
	int ret;

	pr_debug("In ohci_hcd_superh_drv_probe");

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_superh_probe(&ohci_superh_hc_driver, &hcd, pdev);

	if (ret == 0)
		dev_set_drvdata(dev, hcd);

	return ret;
}

static int ohci_hcd_superh_drv_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	usb_hcd_superh_remove(hcd, pdev);
	dev_set_drvdata(dev, NULL);
	return 0;
}

static struct device_driver ohci_hcd_superh_driver = {
	.name = (char *)hcd_name,
	.bus = &platform_bus_type,
	.probe = ohci_hcd_superh_drv_probe,
	.remove = ohci_hcd_superh_drv_remove,
	/*.suspend      = ohci_hcd_superh_drv_suspend, */
	/*.resume       = ohci_hcd_superh_drv_resume, */
};

static struct platform_device superh_usb_device = {
	.name = (char *)hcd_name,
	.id = 0,
};

static int __init ohci_hcd_superh_init(void)
{
	int ret;
	dbg(DRIVER_INFO " (SuperH USB)");

	ret = platform_device_register(&superh_usb_device);
	if (ret)
		return -EINVAL;

	printk(KERN_DEBUG "%s: block sizes: ed %Zd td %Zd\n", hcd_name,
	       sizeof(struct ed), sizeof(struct td));

	return driver_register(&ohci_hcd_superh_driver);
}

static void __exit ohci_hcd_superh_cleanup(void)
{
	driver_unregister(&ohci_hcd_superh_driver);
	platform_device_unregister(&superh_usb_device);
}

module_init(ohci_hcd_superh_init);
module_exit(ohci_hcd_superh_cleanup);
