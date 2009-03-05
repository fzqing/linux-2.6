/*
 * (C) Copyright David Brownell 2000-2002
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/device.h>
#include <linux/fsl_devices.h>

#include "ehci-fsl.h"

/* FIXME: Power Managment is un-ported so temporarily disable it */
#undef CONFIG_PM

void usb_hcd_fsl_remove (struct platform_device *pdev);

/* PCI-based HCs are normal, but custom bus glue should be ok */


/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_hcd_pci_probe - initialize FSL-based HCDs
 * @dev: USB Host Controller being probed
 * @id: pci hotplug id connecting controller to HCD framework
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
int usb_hcd_fsl_probe(struct hc_driver *driver, struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	struct usb_hcd *hcd;
	struct resource	*res;
	int irq;
	unsigned long		resource, len;
	void __iomem		*base;
	int			retval;
	char			buf [8], *bufp = buf;
	unsigned int temp;

	pr_debug("initializing FSL-SOC USB Controller\n");

	/* Need platform data for setup */
	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", pdev->dev.bus_id);
		return -ENODEV;
	}

	/*
	 * This is a host mode driver, verify that we're supposed to be
	 * in host mode.
	 */
	if (!((pdata->operating_mode == FSL_USB2_DR_HOST) ||
	      (pdata->operating_mode == FSL_USB2_MPH_HOST))) {
		dev_err(&pdev->dev,
			"Non Host Mode configured for %s. Wrong driver linked.\n",
			pdev->dev.bus_id);
		return -ENODEV;
	}

	/* Get the IRQ from platform data */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			pdev->dev.bus_id);
		return -ENODEV;
	}
	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			pdev->dev.bus_id);
		return -ENODEV;
	}
	resource = res->start;
	len = res->end - res->start + 1;
	if (!request_mem_region (resource, len, driver->description)) {
		dev_dbg (&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto done;
	}
	base = ioremap_nocache (resource, len);
	if (base == NULL) {
		dev_dbg (&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
clean_1:
		release_mem_region (resource, len);
		dev_err (&pdev->dev, "init %s fail, %d\n",
				pdev->dev.bus_id, retval);
		goto done;
	}

	hcd = driver->hcd_alloc ();
	if (hcd == NULL){
		dev_dbg (&pdev->dev, "hcd alloc fail\n");
		retval = -ENOMEM;
clean_2:
		iounmap (base);
		goto clean_1;
	}
	// hcd zeroed everything
	hcd->regs = base;

	/* Enable USB controller */
	temp = in_be32(hcd->regs + 0x500);
	out_be32(hcd->regs + 0x500, temp | 0x4);

	/* Set to Host mode */
	temp = in_le32(hcd->regs + 0x1a8);
	out_le32(hcd->regs + 0x1a8, temp | 0x3);


	dev_set_drvdata(&pdev->dev, hcd);
	hcd->driver = driver;
	hcd->description = driver->description;
	hcd->self.bus_name = pdev->dev.bus_id;
	hcd->product_desc = "FSL USB Host Controller";
	hcd->self.controller = &pdev->dev;

	if ((retval = hcd_buffer_create (hcd)) != 0) {
clean_3:
		kfree (hcd);
		goto clean_2;
	}

	dev_info (hcd->self.controller, "%s\n", hcd->product_desc);

	/* till now HC has been in an indeterminate state ... */
	if (driver->reset && (retval = driver->reset (hcd)) < 0) {
		dev_err (hcd->self.controller, "can't reset\n");
		goto clean_3;
	}
	hcd->state = USB_STATE_HALT;

	sprintf (buf, "%d", irq);
	retval = request_irq (irq, usb_hcd_irq, SA_SHIRQ,
				hcd->description, hcd);
	if (retval != 0) {
		dev_err (hcd->self.controller,
				"request interrupt %s failed\n", bufp);
		goto clean_3;
	}
	hcd->irq = irq;

	dev_info (hcd->self.controller, "irq %s, %s 0x%lx\n", bufp,
		(driver->flags & HCD_MEMORY) ? "pci mem" : "io base",
		resource);

	usb_bus_init (&hcd->self);
	hcd->self.op = &usb_hcd_operations;
	hcd->self.release = &usb_hcd_release;
	hcd->self.hcpriv = (void *) hcd;
	init_timer (&hcd->rh_timer);

	INIT_LIST_HEAD (&hcd->dev_list);

	usb_register_bus (&hcd->self);

	if ((retval = driver->start (hcd)) < 0) {
		dev_err (hcd->self.controller, "init error %d\n", retval);
		usb_hcd_fsl_remove (pdev);
	}

done:
	return retval;
}
EXPORT_SYMBOL (usb_hcd_fsl_probe);


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_fsl_remove - shutdown processing for FSL-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_pci_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 * Store this function in the HCD's struct pci_driver as remove().
 */
void usb_hcd_fsl_remove (struct platform_device *pdev)
{
	struct usb_hcd		*hcd;
	struct fsl_usb2_platform_data *pdata;
	struct resource *res;

	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;

	hcd = dev_get_drvdata(&pdev->dev);
	if (!hcd)
		return;
	dev_info (hcd->self.controller, "remove, state %x\n", hcd->state);

	if (in_interrupt ())
		BUG ();

	if (HCD_IS_RUNNING (hcd->state))
		hcd->state = USB_STATE_QUIESCING;

	dev_dbg (hcd->self.controller, "roothub graceful disconnect\n");
	usb_disconnect (&hcd->self.root_hub);

	hcd->driver->stop (hcd);
	hcd_buffer_destroy (hcd);
	hcd->state = USB_STATE_HALT;
	dev_set_drvdata(&pdev->dev, NULL);

	free_irq (hcd->irq, hcd);
	iounmap (hcd->regs);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region (res->start, res->end - res->start + 1);

	usb_deregister_bus (&hcd->self);

}
EXPORT_SYMBOL (usb_hcd_fsl_remove);

static void mpc83xx_setup_phy(struct ehci_hcd *ehci,
			      enum fsl_usb2_phy_modes phy_mode,
			      unsigned int port_offset)
{
	u32 portsc = readl(&ehci->regs->port_status[port_offset]);
	portsc &= ~PORT_PTS_MSK;
	switch (phy_mode) {
	case FSL_USB2_PHY_ULPI:
		portsc |= PORT_PTS_ULPI;
		break;
	case FSL_USB2_PHY_SERIAL:
		portsc |= PORT_PTS_SERIAL;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		portsc |= PORT_PTS_PTW;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		portsc |= PORT_PTS_UTMI;
		break;
	case FSL_USB2_PHY_NONE:
		break;
	}
	writel(portsc, &ehci->regs->port_status[port_offset]);
}

static void mpc83xx_usb_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fsl_usb2_platform_data *pdata;
	void __iomem *non_ehci = hcd->regs;

	pdata =
	    (struct fsl_usb2_platform_data *)hcd->self.controller->
	    platform_data;
	/* Enable PHY interface in the control reg. */
	out_be32(non_ehci + FSL_SOC_USB_CTRL, 0x00000004);
	out_be32(non_ehci + FSL_SOC_USB_SNOOP1, 0x0000001b);

	if (pdata->operating_mode == FSL_USB2_DR_HOST)
		mpc83xx_setup_phy(ehci, pdata->phy_mode, 0);

	if (pdata->operating_mode == FSL_USB2_MPH_HOST) {
		unsigned int chip, rev, svr;

		svr = mfspr(SPRN_SVR);
		chip = svr >> 16;
		rev = (svr >> 4) & 0xf;

		/* Deal with USB Erratum #14 on MPC834x Rev 1.0 & 1.1 chips */
		if ((rev == 1) && (chip >= 0x8050) && (chip <= 0x8055))
			ehci->has_fsl_port_bug = 1;

		if (pdata->port_enables & FSL_USB2_PORT0_ENABLED)
			mpc83xx_setup_phy(ehci, pdata->phy_mode, 0);
		if (pdata->port_enables & FSL_USB2_PORT1_ENABLED)
			mpc83xx_setup_phy(ehci, pdata->phy_mode, 1);
	}

	/* put controller in host mode. */
	writel(0x00000003, non_ehci + FSL_SOC_USB_USBMODE);
	out_be32(non_ehci + FSL_SOC_USB_PRICTRL, 0x0000000c);
	out_be32(non_ehci + FSL_SOC_USB_AGECNTTHRSH, 0x00000040);
	out_be32(non_ehci + FSL_SOC_USB_SICTRL, 0x00000001);
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_fsl_reinit(struct usb_hcd *hcd)
{
	mpc83xx_usb_setup(hcd);
	ehci_port_power(hcd_to_ehci(hcd), 0);

	return 0;
}
/* called by khubd or root hub init threads */
static int ehci_fsl_reset (struct usb_hcd *hcd)
{
	int retval;
	struct ehci_hcd		*ehci = hcd_to_ehci (hcd);

	spin_lock_init (&ehci->lock);

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
	    HC_LENGTH(readl(&ehci->caps->hc_capbase));
	dbg_hcs_params (ehci, "reset");
	dbg_hcc_params (ehci, "reset");

	/* cache this readonly data; minimize PCI reads */
	ehci->hcs_params = readl (&ehci->caps->hcs_params);

	retval =  ehci_halt(ehci);
	if (retval)
		return retval;

	ehci->is_arc_rh_tt = 1;

	if (ehci_is_ARC(ehci))
		ehci_reset(ehci);

	ehci->sbrn = 0x20;

	retval = ehci_fsl_reinit(hcd);

	return retval;
}

static int ehci_fsl_start (struct usb_hcd *hcd)
{

	return ehci_run (hcd);

}
/* always called by thread; normally rmmod */

static void ehci_fsl_stop (struct usb_hcd *hcd)
{
	ehci_stop (hcd);
}

/*-------------------------------------------------------------------------*/

static struct hc_driver ehci_fsl_hc_driver = {
	.description =		hcd_name,

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ehci_fsl_reset,
	.start =		ehci_fsl_start,
#ifdef	CONFIG_PM
	.suspend =		ehci_bus_suspend,
	.resume =		ehci_bus_resume,
#endif
	.stop =			ehci_fsl_stop,

#if 1
	/*
	 * memory lifecycle (except per-request)
	 */
	.hcd_alloc =		ehci_hcd_alloc,
#endif

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
	.hub_suspend =		ehci_hub_suspend,
	.hub_resume =		ehci_hub_resume,
};

/*-------------------------------------------------------------------------*/

static int ehci_fsl_drv_probe(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	if (usb_disabled())
		return -ENODEV;

	return usb_hcd_fsl_probe(&ehci_fsl_hc_driver, pdev);
}

static int ehci_fsl_drv_remove(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	usb_hcd_fsl_remove(pdev);

	return 0;
}

static struct device_driver ehci_fsl_dr_driver = {
	.name = "fsl-usb2-dr",
	.bus = &platform_bus_type,
	.probe = ehci_fsl_drv_probe,
	.remove = ehci_fsl_drv_remove,
};

static struct device_driver ehci_fsl_mph_driver = {
	.name = "fsl-usb2-mph",
	.bus = &platform_bus_type,
	.probe = ehci_fsl_drv_probe,
	.remove = ehci_fsl_drv_remove,
};

static int __init ehci_fsl_init(void)
{
	int retval;

	pr_debug("%s: block sizes: qh %Zd qtd %Zd itd %Zd sitd %Zd\n",
		 hcd_name,
		 sizeof(struct ehci_qh), sizeof(struct ehci_qtd),
		 sizeof(struct ehci_itd), sizeof(struct ehci_sitd));

	retval = driver_register(&ehci_fsl_dr_driver);
	if (retval)
		return retval;

	return driver_register(&ehci_fsl_mph_driver);
}

static void __exit ehci_fsl_cleanup(void)
{
	driver_unregister(&ehci_fsl_mph_driver);
	driver_unregister(&ehci_fsl_dr_driver);
}

module_init(ehci_fsl_init);
module_exit(ehci_fsl_cleanup);
