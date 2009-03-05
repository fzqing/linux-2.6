/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2003-2006 MontaVista Software Inc.
 * (C) Copyright 2006 AMCC
 *
 * Bus Glue for PPC On-Chip EHCI driver
 * AMCC4xxx
 *
 * Modified from ehci-ppc-soc.c
 *
 * This file is licenced under the GPL.
 */

#ifdef CONFIG_USB_440EPX_EHCI_ERRATA

#define SPEC_REGS_OFFSET	0x90

struct ehci_spec_regs {

	/* EHCI INSNREG00: offset 0x90 */
	u32		insnreg00;
	/* EHCI INSNREG01: offset 0x94 */
	u32		insnreg01;
	/* EHCI INSNREG02: offset 0x98 */
	u32		insnreg02;
	/* EHCI INSNREG03: offset 0x9C */
	u32		insnreg03;
#define EBMT		(1<<0)		/* Enable Break Memory Transfer */
	/* EHCI INSNREG04: offset 0xA0 */
	u32		insnreg04; 	/* address bits 63:32 if needed */
	/* EHCI INSNREG05: offset 0xA4 */
	u32		insnreg05; 	/* points to periodic list */

} __attribute__ ((packed));


static inline void ehci_ppc_soc_errata(struct ehci_hcd *ehci)
{
	u32 hcc_params;
	struct ehci_spec_regs __iomem *spec_regs;

	if (!ehci)
		return;

	spec_regs = ehci->hcd.regs + SPEC_REGS_OFFSET;
	/* Enable Break Memory Transfer */
	ehci_writel(ehci, EBMT, &spec_regs->insnreg03);
	/*
	 * When park mode enabled, frame babble error occurs
	 * due to high system latency. Disable park mode.
	 */
	hcc_params = ehci_readl (ehci, &ehci->caps->hcc_params);
	if (HCC_CANPARK(hcc_params)) {
		u32 cmd;

		cmd = ehci_readl (ehci, &ehci->regs->command);
		cmd &= ~CMD_PARK;
		ehci_writel (ehci, cmd, &ehci->regs->command);
	}
}

#else	/* !CONFIG_USB_440EPX_EHCI_ERRATA */

static inline void ehci_ppc_soc_errata(struct ehci_hcd *ehci)
{
}

#endif	/* CONFIG_USB_440EPX_EHCI_ERRATA */

/* configure so an HC device and id are always provided */
/* This driver does not support extended capabilities */
/* always called with process context; sleeping is OK */

/**
 * ehci_ppc_soc_hcd_init - init echi structure
 * Context: !in_interrupt()
 *
 * Init basic fields like regs for this USB host controller.
 *
 */
static int ehci_ppc_soc_hcd_init(struct ehci_hcd *ehci)
{
	u32			temp;

	spin_lock_init(&ehci->lock);

	ehci->caps = ehci->hcd.regs;
	ehci->regs = ehci->hcd.regs +
			HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "init");
	dbg_hcc_params(ehci, "init");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	ehci_reset(ehci);

	ehci_port_power(ehci, 0);

	temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
	temp &= 0x0f;
	if (temp && HCS_N_PORTS(ehci->hcs_params) > temp) {
		ehci_dbg(ehci, "bogus port configuration: "
				"cc=%d x pcc=%d < ports=%d\n",
				HCS_N_CC(ehci->hcs_params),
				HCS_N_PCC(ehci->hcs_params),
				HCS_N_PORTS(ehci->hcs_params));
	}
	/* force HC to halt state */
	return ehci_halt(ehci);
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_ppc_soc_remove(struct platform_device *pdev)
{
	struct usb_hcd	*hcd;
	struct resource	*res;

	hcd = dev_get_drvdata(&pdev->dev);
	if (!hcd)
		return;

	if (in_interrupt())
		BUG();

	if (HCD_IS_RUNNING(hcd->state))
		hcd->state = USB_STATE_QUIESCING;

	usb_disconnect(&hcd->self.root_hub);

	hcd->driver->stop(hcd);
	del_timer_sync(&hcd->rh_timer);
	hcd_buffer_destroy(hcd);
	hcd->state = USB_STATE_HALT;
	dev_set_drvdata(&pdev->dev, NULL);


	free_irq(hcd->irq, hcd);
	iounmap(hcd->regs);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		release_mem_region(res->start, res->end - res->start + 1);
	}

	usb_deregister_bus (&hcd->self);

	pr_debug("Removed PPC-SOC EHCI USB Controller\n");
}


/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int usb_hcd_ppc_soc_probe(struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ehci_hcd	*ehci;
	struct resource *res;
	unsigned long res_len;
	int irq;

	pr_debug("Initializing PPC-SOC EHCI USB Controller\n");

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		pr_debug(__FILE__ ": no irq\n");
		return -ENODEV;
	}
	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_debug(__FILE__ ": no reg addr\n");
		return -ENODEV;
	}
	hcd = driver->hcd_alloc();
	if (!hcd)
		return -ENOMEM;

	ehci = hcd_to_ehci(hcd);
	ehci->big_endian = 1;

	res_len = res->end - res->start + 1;
	if (!request_mem_region(res->start, res_len, hcd_name)) {
		pr_debug(__FILE__ ": request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap64(res->start, res_len);
	if (!hcd->regs) {
		pr_debug(__FILE__ ": ioremap failed\n");
		retval = -ENOMEM;
		goto err2;
	}

	dev_set_drvdata(&pdev->dev, hcd);
	hcd->driver = driver;
	hcd->description = driver->description;
	hcd->self.bus_name = pdev->dev.bus_id;
	hcd->product_desc = "PPC-SOC EHCI USB";
	hcd->self.controller = &pdev->dev;

	if ((retval = hcd_buffer_create(hcd)) != 0) {
		pr_debug(__FILE__ ": hcd_buffer_create failed\n");
		retval = -ENOMEM;
		goto err3;
	}

	hcd->state = USB_STATE_HALT;

	retval = request_irq(irq, usb_hcd_irq, SA_SHIRQ,
				hcd->description, hcd);
	if (retval != 0) {
		pr_debug(__FILE__ ": request_irq failed\n");
		retval = -ENODEV;
		goto err4;
	}

	hcd->irq = irq;
	usb_bus_init(&hcd->self);
	hcd->self.op = &usb_hcd_operations;
	hcd->self.release = &usb_hcd_release;
	hcd->self.hcpriv = (void *) hcd;
	init_timer(&hcd->rh_timer);

	INIT_LIST_HEAD(&hcd->dev_list);

	usb_register_bus(&hcd->self);

	if ((retval = driver->start(hcd)) < 0) {
		dev_err(hcd->self.controller, "init error %d\n", retval);
		usb_hcd_ppc_soc_remove(pdev);
	}
	return retval;

err4:
	hcd_buffer_destroy(hcd);
err3:
	iounmap(hcd->regs);
err2:
	release_mem_region(res->start, res_len);
err1:
	kfree(hcd);
	return retval;
}


static int ehci_ppc_soc_start(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		ret;

	if ((ret = ehci_ppc_soc_hcd_init(ehci)) < 0)
		return ret;

	if ((ret = ehci_run(hcd)) < 0) {
		err("can't start\n");
		ehci_stop(hcd);
		return ret;
	}

	ehci_ppc_soc_errata(ehci);

	return 0;
}


static struct hc_driver ehci_ppc_soc_hc_driver = {
	.description =		hcd_name,

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_USB2 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ehci_ppc_soc_start,
	.stop =			ehci_stop,
	.hcd_alloc =		ehci_hcd_alloc,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
#endif

};

static int ehci_hcd_ppc_soc_drv_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_ppc_soc_probe(&ehci_ppc_soc_hc_driver, pdev);
	return ret;
}

static int ehci_hcd_ppc_soc_drv_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	usb_hcd_ppc_soc_remove(pdev);
	return 0;
}

static struct device_driver ehci_hcd_ppc_soc_driver = {
	.name		= "ppc-soc-ehci",
	.owner		= THIS_MODULE,
	.bus		= &platform_bus_type,
	.probe		= ehci_hcd_ppc_soc_drv_probe,
	.remove		= ehci_hcd_ppc_soc_drv_remove,
#ifdef	CONFIG_PM
	/*.suspend	= ehci_hcd_ppc_soc_drv_suspend,*/
	/*.resume	= ehci_hcd_ppc_soc_drv_resume,*/
#endif
};

static int __init ehci_hcd_ppc_soc_init(void)
{
	pr_debug(DRIVER_INFO " (EHCI PPC SOC)\n");
	pr_debug("%s: block sizes: qh %Zd qtd %Zd itd %Zd sitd %Zd\n",
		  hcd_name,
		  sizeof(struct ehci_qh), sizeof(struct ehci_qtd),
		  sizeof(struct ehci_itd), sizeof(struct ehci_sitd));

	return driver_register(&ehci_hcd_ppc_soc_driver);
}

static void __exit ehci_hcd_ppc_soc_cleanup(void)
{
	driver_unregister(&ehci_hcd_ppc_soc_driver);
}

module_init(ehci_hcd_ppc_soc_init);
module_exit(ehci_hcd_ppc_soc_cleanup);
