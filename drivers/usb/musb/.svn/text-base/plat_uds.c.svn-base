/*****************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

/*
 * Inventra (Multipoint) Dual-Role Controller Driver for Linux.
 *
 * This consists of a Host Controller Driver (HCD) and a peripheral
 * controller driver implementing the "Gadget" API; OTG support is
 * in the works.  These are normal Linux-USB controller drivers which
 * use IRQs and have no dedicated thread.
 *
 * This version of the driver has only been used with products from
 * Texas Instruments.  Those products integrate the Inventra logic
 * with other DMA, IRQ, and bus modules, as well as other logic that
 * needs to be reflected in this driver.
 *
 *
 * NOTE:  the original Mentor code here was pretty much a collection
 * of mechanisms that don't seem to have been fully integrated/working
 * for any Linux kernel version.  This version aims at Linux 2.6.10, and
 * plans for integration with more current kernels.  Key open issues
 * include:
 *
 *  - Lack of host-side transaction scheduling, for all transfer types.
 *    The hardware doesn't do it; instead, software must.
 *
 *    This is not an issue for OTG devices that don't support external
 *    hubs, but for more "normal" USB hosts it's a user issue that the
 *    "multipoint" support doesn't scale in the expected ways.  That
 *    includes DaVinci EVM in a common non-OTG mode.
 *
 *      * Control and bulk use dedicated endpoints, and there's as
 *        yet no mechanism to either (a) reclaim the hardware when
 *        peripherals are NAKing, which gets complicated with bulk
 *        endpoints, or (b) use more than a single bulk endpoint in
 *        each direction.
 *
 *        RESULT:  one device may be perceived as blocking another one.
 *
 *      * Interrupt and isochronous will dynamically allocate endpoint
 *        hardware, but (a) there's no record keeping for bandwidth;
 *        (b) in the common case that few endpoints are available, there
 *	  is no mechanism to reuse endpoints to talk to multiple devices.
 *
 *	  RESULT:  At one extreme, bandwidth can be overcommitted in
 *	  some hardware configurations, no faults will be reported.
 *	  At the other extreme, the bandwidth capabilities which do
 *	  exist tend to be severely undercommitted.  You can't yet hook
 *	  up both a keyboard and a mouse to an external USB hub.
 *
 *      * Host side doesn't understand that hardware endpoints have two
 *        directions, so it uses only half the resources available on
 *        chips like DaVinci or TUSB 6010.
 *
 *	  RESULT:  On DaVinci (and TUSB 6010), only one external device may
 *	  use periodic transfers, other than the hub used to connect it.
 *	  (And if it were to understand, there would still be limitations
 *	  because of the lack of periodic endpoint scheduling.)
 *
 *  - Host-side doesn't use the HCD framework, even the older version in
 *    the 2.6.10 kernel, which doesn't provide per-endpoint URB queues.
 *
 *    RESULT:  code bloat, because it provides its own root hub;
 *    correctness issues.
 *
 *  - Provides its own OTG bits.  These are untested, and many of them
 *    seem to be superfluous code bloat given what usbcore does.  (They
 *    have now been partially removed.)
 */

/*
 * This gets many kinds of configuration information:
 *	- Kconfig for everything user-configurable
 *	- <asm/arch/hdrc_cnf.h> for SOC or family details
 *	- platform_device for addressing, irq, and platform_data
 *	- platform_data is mostly for board-specific informarion
 *
 * Most of the conditional compilation will (someday) vanish.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
// #include <linux/platform_device.h>
// #include <linux/clk.h>
#ifdef CONFIG_ARCH_DAVINCI
#include <asm/hardware/clock.h>
#include <asm/arch/hardware.h>
#include <asm/arch/memory.h>
#endif
#include <asm/io.h>

#ifdef	CONFIG_ARM
#include <asm/mach-types.h>
#endif

// #ifdef CONFIG_USB_MUSB_HDRC_HCD
#include <linux/usb.h>
#include "../core/hcd.h"
// #endif

/* HBG 21 SEPT2006 added below to be otg comply */
//----------------------------------------------
#ifdef CONFIG_USB_MUSB_OTG
#include <asm/uaccess.h>	/* FIXME remove procfs writes */
#include "otg.h"
#include <linux/proc_fs.h>
#endif

//----------------------------------------------
#if MUSB_DEBUG > 0
unsigned MGC_DebugLevel = MUSB_DEBUG;
#endif

#include "musbdefs.h"
// #ifdef CONFIG_USB_MUSB_HDRC_HCD
#include "musb_host.h"
// #endif

#ifdef CONFIG_ARCH_DAVINCI
#include "davinci.h"
#endif

#include "tusb_6010.h"

/***************************** CONSTANTS ********************************/

#define DRIVER_AUTHOR "Mentor Graphics Corp. and Texas Instruments"
#define DRIVER_DESC "Inventra Dual-Role USB Controller Driver"

#define MUSB_VERSION_BASE "2.2a/db-0.4.8"

#ifndef MUSB_VERSION_SUFFIX
#define MUSB_VERSION_SUFFIX	 ""
#endif
#define MUSB_VERSION	MUSB_VERSION_BASE MUSB_VERSION_SUFFIX

#define DRIVER_INFO DRIVER_DESC "v" MUSB_VERSION

#ifdef CONFIG_ARCH_DAVINCI_DM355
#define VBUSERR_RETRY_COUNT 4
#endif

static const char longname[] = DRIVER_INFO;
const char musb_driver_name[] = "musb_hdrc";

/* this module is always GPL, the gadget might not... */
MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");

/* time (millseconds) to wait before a restart */
#define MUSB_RESTART_TIME        5000

/* how many babbles to allow before giving up */
#define MUSB_MAX_BABBLE_COUNT    10

extern void MGC_HdrcEnableTXDMA(struct musb *pThis, u8 bEnd);
#ifdef CONFIG_USB_MUSB_OTG
/*HBG 22SEPT2006 */
static int musb_app_inputs(struct file *file, const char __user * buffer,
			   unsigned long count, void *data);
/*************************************************************************
 * HDRC functions
**************************************************************************/
/* HBG 21 SEPT removed from here now OTG will export this interface*/
//==========================================================================

/* Called with controller locked and IRQ locked */
int musb_start_hnp(struct otg_transceiver *otg)
{
	if (!otg)
		return 0;
	else {
		struct otg_machine *pMachine = container_of(otg,
							    struct otg_machine,
							    xceiv);
		u8 devctl =
		    musb_readb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL);
		musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL,
			    devctl | MGC_M_DEVCTL_HR);
		return 1;
	}

}

/* Called with controller locked and IRQ locked */
int musb_start_srp(struct otg_transceiver *otg)
{
	if (!otg)
		return 0;
	else {
		struct otg_machine *pMachine = container_of(otg,
							    struct otg_machine,
							    xceiv);
		u8 devctl =
		    musb_readb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL);
		devctl |= MGC_M_DEVCTL_SESSION;
		musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);
		return 1;
	}
}

/* Called with controller locked and IRQ locked */
int musb_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{

	if (host)
		otg->host = host;
	else
		otg->host = NULL;

	return 1;
}

/* Called with controller locked and IRQ locked */
int musb_set_peripheral(struct otg_transceiver *otg, struct usb_gadget *gadget)
{
	if (gadget)
		otg->gadget = gadget;
	else
		otg->gadget = NULL;

	return 1;
}
void otg_input_changed(struct musb *pThis, u8 devctl, u8 reset,
		       u8 connection, u8 suspend)
{
	struct otg_machine *otgm = &pThis->OtgMachine;
	MGC_OtgMachineInputs Inputs;

	/* reading suspend state from Power register does NOT work */
	memset(&Inputs, 0, sizeof(Inputs));

	Inputs.bSession = (devctl & MGC_M_DEVCTL_SESSION) ? TRUE : FALSE;
	Inputs.bSuspend = suspend;
	Inputs.bConnection = connection;
	Inputs.bReset = reset;
	Inputs.bConnectorId = (devctl & MGC_M_DEVCTL_BDEVICE) ? TRUE : FALSE;
	MGC_OtgMachineInputsChanged(otgm, &Inputs);
}

void otg_input_changed_X(struct musb *pThis, u8 bVbusError, u8 bConnect)
{
	MGC_OtgMachineInputs Inputs;
	void __iomem *pBase = pThis->pRegs;
	u8 devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
	u8 power = musb_readb(pBase, MGC_O_HDRC_POWER);

	DBG(2, "<== power %02x, devctl %02x%s%s\n", power, devctl,
	    bConnect ? ", bcon" : "", bVbusError ? ", vbus_error" : "");

	/* speculative */
	memset(&Inputs, 0, sizeof(Inputs));
	Inputs.bSession = (devctl & MGC_M_DEVCTL_SESSION) ? TRUE : FALSE;
	Inputs.bConnectorId = (devctl & MGC_M_DEVCTL_BDEVICE) ? TRUE : FALSE;
	Inputs.bReset = (power & MGC_M_POWER_RESET) ? TRUE : FALSE;
	Inputs.bConnection = bConnect;
	Inputs.bVbusError = bVbusError;
	Inputs.bSuspend = (power & MGC_M_POWER_SUSPENDM) ? TRUE : FALSE;
	MGC_OtgMachineInputsChanged(&(pThis->OtgMachine), &Inputs);
}
#endif				// CONFIG_USB_MUSB_OTG
//==========================================================================
#ifdef CONFIG_USB_MUSB_HDRC_HCD
/*
 * Timer completion callback
 */
static void musb_timer_done(unsigned long pParam)
{
	struct musb *pThis = (void *)pParam;
	void __iomem *pBase = pThis->pRegs;
	unsigned long flags;
	u8 power;
	spin_lock_irqsave(&pThis->Lock, flags);
#ifdef  CONFIG_USB_MUSB_OTG
	switch (pThis->OtgMachine.xceiv.state) {

	case OTG_STATE_A_SUSPEND:

		DBG(2, "finish RESUME signaling\n");
		power = musb_readb(pBase, MGC_O_HDRC_POWER);
		musb_writeb(pBase, MGC_O_HDRC_POWER,
			    power & ~MGC_M_POWER_RESUME);
		MGC_VirtualHubPortResumed(&pThis->RootHub, 0);

		otg_input_changed_X(pThis, FALSE, FALSE);

		break;
	case OTG_STATE_A_WAIT_VRISE:
		DBG(2, "restart (?)\n");
		musb_start((struct musb *)pParam);
		break;
	default:
		DBG(1, "<== in state %d\n", pThis->OtgMachine.xceiv.state);
		break;
	}
#else
	DBG(2, "finish RESUME signaling\n");
	power = musb_readb(pBase, MGC_O_HDRC_POWER);
	musb_writeb(pBase, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_RESUME);
	MGC_VirtualHubPortResumed(&pThis->RootHub, 0);
#endif
	spin_unlock_irqrestore(&pThis->Lock, flags);
}
#endif

#ifndef CONFIG_USB_TUSB_6010
/*
 * Load an endpoint's FIFO
 */
#if 1

static void musb_fifo_io(u8 * fifo, u8 * buf, u16 len, char is_read)
{
	u16 index = 0;
	char size = ((unsigned long)buf & 0x04) ? 4 :
	    ((unsigned long)buf & 0x02) ? 2 : 1;
#ifndef CONFIG_CPU_LITTLE_ENDIAN
	u32 *pTemp = (u32 *) buf;
	u16 *pTmp16 = (u16 *) buf;
#endif
	size = (len >= size) ? size : (len >= (size >> 1)) ? (size >> 1) : 1;
	if (size == 1) {
		if (is_read)
			readsb(fifo, (void *__iomem)buf, len);
		else
			writesb(fifo, (void *__iomem)buf, len);
		return;
	}
#ifndef CONFIG_CPU_LITTLE_ENDIAN
	while (len >= size) {
		switch (size) {
		case 4:
			if (is_read)
				*pTemp = cpu_to_le32(*(u32 *) fifo);
			else
				*(u32 *) fifo = cpu_to_le32(*pTemp);
			pTemp++;
			break;
		case 2:
			if (is_read)
				*pTmp16 = cpu_to_le16(*(u16 *) fifo);
			else
				*(u16 *) fifo = cpu_to_le16(*pTmp16);
			pTmp16++;
			break;
		}
		len -= size;
		index += size;
	}
#else
	switch (size) {
	case 4:
		if (is_read)
			readsl(fifo, buf, len >> 2);
		else
			writesl(fifo, (void *__iomem)(buf), len >> 2);
		index += len & ~0x03;
		break;
	case 2:
		if (is_read)
			readsw(fifo, buf, len >> 1);
		else
			writesw(fifo, (void *__iomem)(buf), len >> 1);
		index += len & ~0x01;
		break;
	}
#endif
	if (len & 0x02) {
		if (is_read)
			*(u16 *) & buf[index] =
			    cpu_to_le16(musb_readw(fifo, 0));
		else
			musb_writew(fifo, 0,
				    cpu_to_le16(*(u16 *) & buf[index]));
		index += 2;
	}
	if (len & 0x01) {
		if (is_read)
			buf[index] = musb_readb(fifo, 0);
		else
			musb_writeb(fifo, 0, buf[index]);
	}
}

void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 wCount, const u8 * pSource)
{
	void __iomem *fifo = hw_ep->fifo;
	prefetch((u8 *) pSource);
	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
	    'T', hw_ep->bLocalEnd, fifo, wCount, pSource);

	/* we can't assume unaligned reads work */
	if (likely((0x01 & (unsigned long)pSource) == 0)) {
		/* best case is 32bit-aligned source address */
		if ((0x02 & (unsigned long)pSource) == 0) {
			musb_fifo_io(fifo, (u8 *) pSource, wCount, 0);
		} else {
			musb_fifo_io(fifo, (u8 *) pSource, wCount, 0);
		}
	} else {
		musb_fifo_io(fifo, (u8 *) pSource, wCount, 0);
	}
}

void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 wCount, u8 * pDest)
{
	void __iomem *fifo = hw_ep->fifo;
	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
	    'R', hw_ep->bLocalEnd, fifo, wCount, pDest);

	/* we can't assume unaligned writes work */
	if (likely((0x01 & (unsigned long)pDest) == 0)) {
		/* best case is 32bit-aligned destination address */
		if ((0x02 & (unsigned long)pDest) == 0) {
			musb_fifo_io(fifo, pDest, wCount, 1);
		} else {
			musb_fifo_io(fifo, pDest, wCount, 1);
		}
	} else {
		musb_fifo_io(fifo, pDest, wCount, 1);
	}
}

#else
void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 wCount, const u8 * pSource)
{
	void __iomem *fifo = hw_ep->fifo;

	prefetch((u8 *) pSource);

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
	    'T', hw_ep->bLocalEnd, fifo, wCount, pSource);

	/* we can't assume unaligned reads work */
	if (likely((0x01 & (unsigned long)pSource) == 0)) {
		u16 index = 0;

		/* best case is 32bit-aligned source address */
		if ((0x02 & (unsigned long)pSource) == 0) {
			if (wCount >= 4) {
				writesl(fifo, pSource + index, wCount >> 2);
				index += wCount & ~0x03;
			}
			if (wCount & 0x02) {
				musb_writew(fifo, 0, *(u16 *) & pSource[index]);
				index += 2;
			}
		} else {
			if (wCount >= 2) {
				writesw(fifo, pSource + index, wCount >> 1);
				index += wCount & ~0x01;
			}
		}
		if (wCount & 0x01)
			musb_writeb(fifo, 0, pSource[index]);
	} else {
		/* byte aligned */
		writesb(fifo, pSource, wCount);
	}
}

/*
 * Unload an endpoint's FIFO
 */
void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 wCount, u8 * pDest)
{
	void __iomem *fifo = hw_ep->fifo;

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
	    'R', hw_ep->bLocalEnd, fifo, wCount, pDest);

	/* we can't assume unaligned writes work */
	if (likely((0x01 & (unsigned long)pDest) == 0)) {
		u16 index = 0;

		/* best case is 32bit-aligned destination address */
		if ((0x02 & (unsigned long)pDest) == 0) {
			if (wCount >= 4) {
				readsl(fifo, pDest, wCount >> 2);
				index = wCount & ~0x03;
			}
			if (wCount & 0x02) {
				*(u16 *) & pDest[index] = musb_readw(fifo, 0);
				index += 2;
			}
		} else {
			if (wCount >= 2) {
				readsw(fifo, pDest, wCount >> 1);
				index = wCount & ~0x01;
			}
		}
		if (wCount & 0x01)
			pDest[index] = musb_readb(fifo, 0);
	} else {
		/* byte aligned */
		readsb(fifo, pDest, wCount);
	}
}
#endif
#endif				/* normal PIO */

/* Tasklet routine to handle the completion request. Check for Fifo status
 * before completing the request. Avoids false completions when data is still
 * in the fifo
 */
void musb_fifo_check_tasklet (unsigned long data)
{
	struct musb *pThis = (struct musb *)data;
	u8 epnum = 1, sch_tsklt = 0;
	struct musb_hw_ep *pEnd = NULL;
	unsigned long flags;
	u16 csr;

	do {
		pEnd = &(pThis->aLocalEnd[epnum]);
		spin_lock_irqsave(&pThis->Lock, flags);
		if (pEnd->fifo_flush_check) {
			csr = MGC_ReadCsr16(pThis->pRegs, MGC_O_HDRC_TXCSR,
						pEnd->bLocalEnd);
			if ((csr & MGC_M_TXCSR_FIFONOTEMPTY) ||
				(csr & MGC_M_TXCSR_TXPKTRDY) )
				sch_tsklt = 1;
			else {
				pEnd->fifo_flush_check = 0;
				pThis->fifo_check_complete(pEnd);
				DBG(6, "Completed Tasklet %d\n",
						pEnd->bLocalEnd);
			}
		}

		spin_unlock_irqrestore(&pThis->Lock, flags);
		epnum += 2;
	} while (epnum < MUSB_C_NUM_EPS);

	if (sch_tsklt)
		tasklet_schedule(&pThis->fifo_check);
}


/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * the order of the tests is specified in the manual
 *
 * @param pThis instance pointer
 * @param bIntrUSB register contents
 * @param devctl
 * @param power
 */
static irqreturn_t musb_stage0_irq(struct musb *pThis, u8 bIntrUSB,
				   u8 devctl, u8 power)
{
	irqreturn_t handled = IRQ_NONE;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	void __iomem *pBase = pThis->pRegs;
#endif

	DBG(3, "<== Power=%02x, DevCtl=%02x, bIntrUSB=0x%x\n", power, devctl,
	    bIntrUSB);

	/* in host mode when a device resume me (from power save)
	 * in device mode when the host resume me; it shold not change
	 * "identity".
	 */
	if (bIntrUSB & MGC_M_INTR_RESUME) {
		handled = IRQ_HANDLED;
		DBG(3, "RESUME\n");

		if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
			/* REVISIT:  this is where SRP kicks in, yes? */
			MUSB_HST_MODE(pThis);	/* unnecessary */
			power &= ~MGC_M_POWER_SUSPENDM;
			musb_writeb(pBase, MGC_O_HDRC_POWER,
				    power | MGC_M_POWER_RESUME);
#ifdef CONFIG_USB_MUSB_OTG
			otg_input_changed(pThis, devctl, FALSE, FALSE, FALSE);
#endif
			mod_timer(&pThis->Timer, jiffies +
				  msecs_to_jiffies(10));
#endif
		} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			MUSB_DEV_MODE(pThis);	/* unnecessary */
#endif
			musb_g_resume(pThis);
		}
	}
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* see manual for the order of the tests */
	if (bIntrUSB & MGC_M_INTR_SESSREQ) {
		DBG(1, "SESSION_REQUEST \n");

		/* IRQ arrives from ID pin sense or (later, if VBUS power
		 * is removed) SRP.  responses are time critical:
		 *  - turn on VBUS (with silicon-specific mechanism)
		 *  - go through A_WAIT_VRISE
		 *  - ... to A_WAIT_BCON.
		 * a_wait_vrise_tmout triggers VBUS_ERROR transitions
		 */
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
		pThis->bEnd0Stage = MGC_END0_START;

/* HBG 21SEPT2006 removed as part of OTG improvements */
//-----------------------------------------------------
//              pThis->xceiv.state = OTG_STATE_A_IDLE;
//-----------------------------------------------------
		MUSB_HST_MODE(pThis);

		handled = IRQ_HANDLED;

/* HBG 21SEPT2006 removed as part of OTG improvements */
//-----------------------------------------------------

#ifdef CONFIG_USB_MUSB_OTG
		{
			MGC_OtgMachineInputs Inputs;
			memset(&Inputs, 0, sizeof(Inputs));
			Inputs.bSession = TRUE;
			Inputs.bConnectorId = FALSE;
			Inputs.bReset = FALSE;
			Inputs.bConnection = FALSE;
			Inputs.bSuspend = FALSE;
			MGC_OtgMachineInputsChanged(&(pThis->OtgMachine),
						    &Inputs);
		}
#endif

//-----------------------------------------------------
/* HBG 21SEPT2006 OTG implementation */
#ifdef CONFIG_USB_MUSB_OTG
//              otg_input_changed(pThis,bIntrUSB,devctl,power);
#endif
	}

	if (bIntrUSB & MGC_M_INTR_VBUSERROR) {
		printk("VBUS_ERR\n\n");
		// MGC_OtgMachineInputsChanged(otgm, &Inputs);
		// ... may need to abort otg timer ...

		DBG(1, "VBUS_ERROR (%02x)\n", devctl);

#ifdef CONFIG_ARCH_DAVINCI_DM355
	pThis->vbuserr_retry--;

	if (pThis->vbuserr_retry) {
		musb_writel (pThis->ctrl_base,
			DAVINCI_USB_INT_SRC_CLR_REG, 0xffffffff);
		devctl |= MGC_M_DEVCTL_HR;
		devctl |= MGC_M_DEVCTL_SESSION;
			musb_writeb(pBase, MGC_O_HDRC_DEVCTL, devctl);
	}
#endif
/* HBG 21SEPT2006 removed as part of OTG improvements */
//-----------------------------------------------------
//              pThis->xceiv.state = OTG_STATE_A_IDLE;
//-----------------------------------------------------

		/* HBG 13SEPT2006 OTG implementation */
#ifdef CONFIG_USB_MUSB_OTG
		otg_input_changed_X(pThis, TRUE, FALSE);
#endif

		//musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
		return IRQ_HANDLED;
#ifdef CONFIG_ARCH_DAVINCI_DM355
	} else {
		pThis->vbuserr_retry = VBUSERR_RETRY_COUNT;
#endif
	}

	if (bIntrUSB & MGC_M_INTR_CONNECT) {
		u8 speed = USB_SPEED_FULL;
		DBG(1, "CONNECT\n");
		handled = IRQ_HANDLED;

		pThis->bEnd0Stage = MGC_END0_START;

#ifdef CONFIG_USB_MUSB_OTG
		/* flush endpoints when transitioning from Device Mode */
		if (is_peripheral_active(pThis)) {
			// REVISIT HNP; just force disconnect
		}
		pThis->bDelayPortPowerOff = FALSE;
#endif

		/* high vs full speed is just a guess until after reset */
		if (devctl & MGC_M_DEVCTL_LSDEV)
			speed = USB_SPEED_LOW;

		pThis->bRootSpeed = speed;
		MUSB_HST_MODE(pThis);
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		pThis->fifo_check_complete = musb_h_fifo_check_complete;
#endif
		/* indicate new connection to OTG machine */

/* HBG 21SEPT2006 removed as part of OTG improvements */
//-----------------------------------------------------
		/*
		   switch (pThis->xceiv.state) {
		   case OTG_STATE_B_WAIT_ACON:
		   pThis->xceiv.state = OTG_STATE_B_HOST;
		   break;
		   default:
		   DBG(2, "connect in state %d\n", pThis->xceiv.state);

		   case OTG_STATE_A_WAIT_BCON:
		   case OTG_STATE_A_WAIT_VRISE:
		   pThis->xceiv.state = OTG_STATE_A_HOST;
		   break;
		   }
		   DBG(1, "CONNECT (host state %d)\n", pThis->xceiv.state);
		   otg_input_changed(pThis, devctl, FALSE, TRUE, FALSE);
		 */
//-----------------------------------------------------
#ifdef CONFIG_USB_MUSB_OTG
		otg_input_changed(pThis, devctl, FALSE, TRUE, FALSE);
//              otg_input_changed(pThis,bIntrUSB,devctl,power);
#endif

		MGC_VirtualHubPortConnected(&pThis->RootHub, 0, speed);
	}
#endif				/* CONFIG_USB_MUSB_HDRC_HCD */

	/* saved one bit: bus reset and babble share the same bit;
	 * If I am host is a babble! i must be the only one allowed
	 * to reset the bus; when in otg mode it means that I have
	 * to switch to device
	 */
	if (bIntrUSB & MGC_M_INTR_RESET) {
		if (devctl & MGC_M_DEVCTL_HM) {
			DBG(1, "BABBLE\n");

			/* REVISIT it's unclear how to handle this.  Mentor's
			 * code stopped the whole USB host, which is clearly
			 * very wrong.  For now, just expect the hardware is
			 * sane, so babbling devices also trigger a normal
			 * endpoint i/o fault (with automatic recovery).
			 * (A "babble" IRQ seems quite pointless...)
			 */

		} else {
			DBG(1, "BUS RESET\n");
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			pThis->fifo_check_complete = musb_g_fifo_check_complete;
#endif
			musb_g_reset(pThis);
#ifdef CONFIG_USB_MUSB_OTG
			/* HBG 13SEPT2006 removed as part of OTG improvements */
			//-----------------------------------------------------
			//reading state from Power register doesn't works

			otg_input_changed(pThis, devctl, TRUE, FALSE,
					  (power & MGC_M_POWER_SUSPENDM)
					  ? TRUE : FALSE);

			//-----------------------------------------------------

			/* HBG 13SEPT2006 OTG implementation */

			//otg_input_changed(pThis,bIntrUSB,devctl,power);
#endif
		}

		handled = IRQ_HANDLED;
	}

	return handled;
}

/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * the order of the tests is specified in the manual
 *
 * @param pThis instance pointer
 * @param bIntrUSB register contents
 * @param devctl
 * @param power
 */
static irqreturn_t musb_stage2_irq(struct musb *pThis, u8 bIntrUSB,
				   u8 devctl, u8 power)
{
	irqreturn_t handled = IRQ_NONE;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* FIXME peripherals can use SOF IRQs too */

	u8 bEnd;
	u16 wFrame;
	void __iomem *pBase = pThis->pRegs;

	if (bIntrUSB & MGC_M_INTR_SOF) {
		struct musb_hw_ep *ep;

		DBG(6, "START_OF_FRAME\n");
		handled = IRQ_HANDLED;

		/* start any periodic Tx transfers waiting for current frame */
		wFrame = musb_readw(pBase, MGC_O_HDRC_FRAME);
		ep = pThis->aLocalEnd;
		for (bEnd = 1; (bEnd < pThis->bEndCount)
		     && (pThis->wEndMask >= (1 << bEnd)); bEnd++, ep++) {
			// FIXME handle framecounter wraps (12 bits)
			// eliminate duplicated StartUrb logic
			if (ep->dwWaitFrame >= wFrame) {
				ep->dwWaitFrame = 0;
				printk("SOF --> periodic TX%s on %d\n",
				       ep->pDmaChannel ? " DMA" : "", bEnd);
				if (!ep->pDmaChannel)
					MGC_HdrcStartTx(pThis, bEnd);
				else
					MGC_HdrcEnableTXDMA(pThis, bEnd);
			}
		}		/* end of for loop */
	}
#endif

	if ((bIntrUSB & MGC_M_INTR_DISCONNECT) && !pThis->bIgnoreDisconnect) {
		DBG(1, "DISCONNECT as %s, devctl %02x\n",
		    MUSB_MODE(pThis), devctl);
		handled = IRQ_HANDLED;

		/* need to check it against pThis, because devctl is going
		 * to report ID low as soon as the device gets disconnected
		 */
		if (is_host_active(pThis))
			musb_root_disconnect(pThis);
		else
			musb_g_disconnect(pThis);
#ifdef CONFIG_USB_MUSB_OTG
		/* HBG 21SEPT2006 removed as part of OTG improvements */
		//-----------------------------------------------------
		// REVISIT all OTG state machine transitions
		otg_input_changed_X(pThis, FALSE, FALSE);

		//-----------------------------------------------------
		/* HBG 21SEPT2006 OTG implementation */

		//otg_input_changed(pThis,bIntrUSB,devctl,power);
#endif
	}

	if (bIntrUSB & MGC_M_INTR_SUSPEND) {
		DBG(1, "SUSPEND, devctl %02x\n", devctl);
		handled = IRQ_HANDLED;

		/* peripheral suspend, may trigger HNP */
		if (!(devctl & MGC_M_DEVCTL_HM)) {
			musb_g_suspend(pThis);
#ifdef CONFIG_USB_MUSB_OTG
			/* HBG 21SEPT2006 removed as part of OTG improvements */
			//-----------------------------------------------------
			otg_input_changed(pThis, devctl, FALSE, FALSE, TRUE);
			//-----------------------------------------------------

			/* HBG 21SEPT2006 OTG implementation */

			//otg_input_changed(pThis,bIntrUSB,devctl,power);
#endif
		}
	}

	return handled;
}

/*
* Program the HDRC to start (enable interrupts, etc.).
*/
void musb_start(struct musb *pThis)
{
	void __iomem *pBase = pThis->pRegs;
	u8 state;

	DBG(2, "<==\n");

	/* TODO: always set ISOUPDATE in POWER (periph mode) and leave it on! */

	/*  Set INT enable registers, enable interrupts */
	musb_writew(pBase, MGC_O_HDRC_INTRTXE, pThis->wEndMask);
	musb_writew(pBase, MGC_O_HDRC_INTRRXE, pThis->wEndMask & 0xfffe);
	musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0xf7);

	musb_platform_enable(pThis);

	musb_writeb(pBase, MGC_O_HDRC_TESTMODE, 0);

	/* enable high-speed/low-power and start session */
	/* also set MGC_M_POWER_ENSUSPEND, fix the USB attach issue */
	musb_writeb(pBase, MGC_O_HDRC_POWER,
		    MGC_M_POWER_SOFTCONN | MGC_M_POWER_HSENAB |
		    MGC_M_POWER_ENSUSPEND);

	switch (pThis->board_mode) {
	case MUSB_HOST:
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
		break;
	case MUSB_OTG:
		break;
	case MUSB_PERIPHERAL:
		state = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL,
			    state & ~MGC_M_DEVCTL_SESSION);
		break;
	}

	if (musb_in_tusb())
		tusb_enable(pThis);
}

static void musb_generic_disable(struct musb *pThis)
{
	void *__iomem pBase = pThis->pRegs;
	u16 temp;

	/* disable interrupts */
	musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0);
	musb_writew(pBase, MGC_O_HDRC_INTRTX, 0);
	musb_writew(pBase, MGC_O_HDRC_INTRRX, 0);

	/* off */
	musb_writeb(pBase, MGC_O_HDRC_DEVCTL, 0);

	/*  flush pending interrupts */
	temp = musb_readb(pBase, MGC_O_HDRC_INTRUSB);
	temp = musb_readw(pBase, MGC_O_HDRC_INTRTX);
	temp = musb_readw(pBase, MGC_O_HDRC_INTRRX);

}

/*
 * Make the HDRC stop (disable interrupts, etc.);
 * called with controller locked, irqs blocked
 * acts as a NOP unless some role activated the hardware
 */
void musb_stop(struct musb *pThis)
{
	/* stop IRQs, timers, ... */
	musb_platform_disable(pThis);
	musb_generic_disable(pThis);
	DBG(3, "HDRC disabled\n");

#ifdef CONFIG_USB_MUSB_OTG
	if (is_otg_enabled(pThis))
		MGC_OtgMachineDestroy(&pThis->OtgMachine);
#endif

	/* FIXME
	 *  - mark host and/or peripheral drivers unusable/inactive
	 *  - disable DMA (and enable it in HdrcStart)
	 *  - make sure we can musb_start() after musb_stop(); with
	 *    OTG mode, gadget driver module rmmod/modprobe cycles that
	 *  - ...
	 */

	/* flush endpoints */
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if (is_host_enabled(pThis)) {
		u8 bEnd;

		del_timer_sync(&pThis->Timer);
		MGC_VirtualHubStop(&pThis->RootHub);

		// FIXME just force root disconnect, before disable
		// ... that will handle the OTG "rmmod gadget_driver"
		// case more correctly, making the root hub vanish from
		// userspace visibility ... may be awkward ...
		for (bEnd = 0; bEnd < min(16, (int)pThis->bEndCount / 2);
		     bEnd++) {
			MGC_HdrcStopEnd(pThis, bEnd);
		}
	}
#endif
}

static void musb_shutdown(struct device *dev)
{
	struct musb *musb = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&musb->Lock, flags);
	musb_stop(musb);
	MUSB_ERR_MODE(musb, MUSB_ERR_SHUTDOWN);
	spin_unlock_irqrestore(&musb->Lock, flags);
}

#ifdef MUSB_C_DYNFIFO_DEF

/*
 * We don't currently use dynamic fifo setup capability to do anything
 * more than selecting one of a bunch of predefined configurations.
 */
static ushort __initdata fifo_mode = 4 /*2 */ ;

/* "modprobe ... fifo_mode=1" etc */
module_param(fifo_mode, ushort, 0);
MODULE_PARM_DESC(fifo_mode, "initial endpoint configuration");

#define DYN_FIFO_SIZE (1<<(MUSB_C_RAM_BITS+2))

enum fifo_style { FIFO_RXTX, FIFO_TX, FIFO_RX } __attribute__ ((packed));
enum buf_mode { BUF_SINGLE, BUF_DOUBLE } __attribute__ ((packed));

struct fifo_cfg {
	u8 hw_ep_num;
	enum fifo_style style;
	enum buf_mode mode;
	u16 maxpacket;
};

/*
 * tables defining fifo_mode values.  define more if you like.
 */

/* mode 0 - fits in 2KB */
static struct fifo_cfg __initdata mode_0_cfg[] = {
	{.hw_ep_num = 1,.style = FIFO_TX,.maxpacket = 512,},
	{.hw_ep_num = 2,.style = FIFO_RX,.maxpacket = 512,},
	{.hw_ep_num = 3,.style = FIFO_RXTX,.maxpacket = 256,},
	{.hw_ep_num = 4,.style = FIFO_RXTX,.maxpacket = 256,},
};

/* mode 1 - fits in 4KB */
static struct fifo_cfg __initdata mode_1_cfg[] = {
/*{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },*/
	{.hw_ep_num = 1,.style = FIFO_TX,.maxpacket = 512,},
	{.hw_ep_num = 2,.style = FIFO_RX,.maxpacket = 512,},
	{.hw_ep_num = 3,.style = FIFO_RXTX,.maxpacket = 256,},
	{.hw_ep_num = 4,.style = FIFO_RXTX,.maxpacket = 256,},
};

/* mode 2 - fits in 4KB */
static struct fifo_cfg __initdata mode_2_cfg[] = {
	{.hw_ep_num = 1,.style = FIFO_TX,.maxpacket = 512,},
	{.hw_ep_num = 1,.style = FIFO_RX,.maxpacket = 512,},
	{.hw_ep_num = 2,.style = FIFO_TX,.maxpacket = 512,},
	{.hw_ep_num = 2,.style = FIFO_RX,.maxpacket = 512,},
	{.hw_ep_num = 3,.style = FIFO_RXTX,.maxpacket = 256,},
	{.hw_ep_num = 4,.style = FIFO_RXTX,.maxpacket = 256,},
};

/* mode 3 - fits in 4KB */
static struct fifo_cfg __initdata mode_3_cfg[] = {
	{.hw_ep_num = 1,.style = FIFO_TX,.maxpacket = 512,.mode = BUF_DOUBLE,},
	{.hw_ep_num = 1,.style = FIFO_RX,.maxpacket = 512,.mode = BUF_DOUBLE,},
	{.hw_ep_num = 2,.style = FIFO_TX,.maxpacket = 512,},
	{.hw_ep_num = 2,.style = FIFO_RX,.maxpacket = 512,},
	{.hw_ep_num = 3,.style = FIFO_RXTX,.maxpacket = 256,},
	{.hw_ep_num = 4,.style = FIFO_RXTX,.maxpacket = 256,},
};

/* mode 4 - fits in 4KB */
static struct fifo_cfg __initdata mode_4_cfg[] = {
/*{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },*/
	{.hw_ep_num = 1, .style = FIFO_TX, .maxpacket = 512,},
	{.hw_ep_num = 2, .style = FIFO_RX, .maxpacket = 512,},
	{.hw_ep_num = 3, .style = FIFO_TX, .maxpacket = 512,},
	{.hw_ep_num = 4, .style = FIFO_RX, .maxpacket = 1024,},
	{.hw_ep_num = 5, .style = FIFO_TX, .maxpacket = 512,},
	{.hw_ep_num = 6, .style = FIFO_RX, .maxpacket = 512,},
	{.hw_ep_num = 7, .style = FIFO_TX, .maxpacket = 128,},
	{.hw_ep_num = 8, .style = FIFO_RX, .maxpacket = 128,},
};

/*
 * configure a fifo; for non-shared endpoints, this may be called
 * once for a tx fifo and once for an rx fifo.
 *
 * returns negative errno or offset for next fifo.
 */
static int __init
fifo_setup(struct musb *musb, struct musb_hw_ep *hw_ep,
	   const struct fifo_cfg *cfg, u16 offset)
{
	void *__iomem mbase = musb->pRegs;
	int size = 0;
	u16 maxpacket = cfg->maxpacket;
	u16 c_off = offset >> 3;
	u8 c_size;

	/* expect hw_ep has already been zero-initialized */

	size = ffs(max(maxpacket, (u16) 8)) - 1;
	maxpacket = 1 << size;

	c_size = size - 3;
	if (cfg->mode == BUF_DOUBLE) {
		if ((offset + (maxpacket << 1)) > DYN_FIFO_SIZE)
			return -EMSGSIZE;
		c_size |= MGC_M_FIFOSZ_DPB;
	} else {
		if ((offset + maxpacket) > DYN_FIFO_SIZE)
			return -EMSGSIZE;
	}

	/* configure the FIFO */
	musb_writeb(mbase, MGC_O_HDRC_INDEX, hw_ep->bLocalEnd);

	switch (cfg->style) {
	case FIFO_TX:
		musb_writeb(mbase, MGC_O_HDRC_TXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_TXFIFOADD, c_off);
		hw_ep->tx_double_buffered = !!(c_size & MGC_M_FIFOSZ_DPB);
		hw_ep->wMaxPacketSizeTx = maxpacket;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		/* reserve some OUT endpoint for bulk */
		if (!musb->bulk_tx_end
		    && hw_ep != musb->bulk_rx_end && maxpacket >= 512) {
			musb->bulk_tx_end = hw_ep;
			hw_ep->out_traffic_type = PIPE_BULK;
//                      hw_ep->bIsClaimed = 1;
		}
#endif
		break;
	case FIFO_RX:
		musb_writeb(mbase, MGC_O_HDRC_RXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_RXFIFOADD, c_off);
		hw_ep->rx_double_buffered = !!(c_size & MGC_M_FIFOSZ_DPB);
		hw_ep->wMaxPacketSizeRx = maxpacket;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		/* reserve some IN endpoint for bulk */
		if (!musb->bulk_rx_end
		    && hw_ep != musb->bulk_tx_end && maxpacket >= 512) {
			musb->bulk_rx_end = hw_ep;
			hw_ep->in_traffic_type = PIPE_BULK;
//                      hw_ep->bIsClaimed = 1;
		}
#endif
		break;
	case FIFO_RXTX:
		musb_writeb(mbase, MGC_O_HDRC_TXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_TXFIFOADD, c_off);
		hw_ep->rx_double_buffered = !!(c_size & MGC_M_FIFOSZ_DPB);
		hw_ep->wMaxPacketSizeRx = maxpacket;

		musb_writeb(mbase, MGC_O_HDRC_RXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_RXFIFOADD, c_off);
		hw_ep->tx_double_buffered = hw_ep->rx_double_buffered;
		hw_ep->wMaxPacketSizeTx = maxpacket;

		hw_ep->bIsSharedFifo = TRUE;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		/* reserve EP0 endpoint for control */
		if (hw_ep->bLocalEnd == 0) {
			hw_ep->bIsClaimed = 1;
			hw_ep->in_traffic_type = PIPE_CONTROL;
			hw_ep->out_traffic_type = hw_ep->in_traffic_type;
		}
#endif
		break;
	}

	/* NOTE rx and tx endpoint irqs aren't managed separately,
	 * which happens to be ok
	 */
	musb->wEndMask |= (1 << hw_ep->bLocalEnd);

	return offset + (maxpacket << ((c_size & MGC_M_FIFOSZ_DPB) ? 1 : 0));
}

static struct fifo_cfg __initdata ep0_cfg = {
	.style = FIFO_RXTX,.maxpacket = 64,
};

static int __init ep_config_from_table(struct musb *musb)
{
	const struct fifo_cfg *cfg;
	unsigned n;
	int offset;
	struct musb_hw_ep *hw_ep = musb->aLocalEnd;

	switch (fifo_mode) {
	default:
		fifo_mode = 0;
		/* FALLTHROUGH */
	case 0:
		cfg = mode_0_cfg;
		n = ARRAY_SIZE(mode_0_cfg);
		break;
	case 1:
		cfg = mode_1_cfg;
		n = ARRAY_SIZE(mode_1_cfg);
		break;
	case 2:
		cfg = mode_2_cfg;
		n = ARRAY_SIZE(mode_2_cfg);
		break;
	case 3:
		cfg = mode_3_cfg;
		n = ARRAY_SIZE(mode_3_cfg);
		break;
	case 4:
		cfg = mode_4_cfg;
		n = ARRAY_SIZE(mode_4_cfg);
		break;
	}

	printk(KERN_DEBUG "%s: setup fifo_mode %d\n",
	       musb_driver_name, fifo_mode);

	offset = fifo_setup(musb, hw_ep, &ep0_cfg, 0);
	// assert(offset > 0)

	while (n--) {
		u8 epn = cfg->hw_ep_num;

		if (epn >= MUSB_C_NUM_EPS) {
			pr_debug("%s: invalid ep %d\n", musb_driver_name, epn);
			return -EINVAL;
		}
		offset = fifo_setup(musb, hw_ep + epn, cfg++, offset);
		if (offset < 0) {
			pr_debug("%s: mem overrun, ep %d\n",
				 musb_driver_name, epn);
			return -EINVAL;
		}
		epn++;
		musb->bEndCount = max(epn, musb->bEndCount);
	}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* for now, bulk uses two reserved endpoints */
	if (!musb->bulk_rx_end || !musb->bulk_tx_end) {
		pr_debug("%s: missing bulk TX or RX\n", musb_driver_name);
		return -EINVAL;
	}
#endif

	return 0;
}

#else

/*
 * ep_config_from_hw - when MUSB_C_DYNFIFO_DEF is false
 * @param pThis the controller
 */
static int __init ep_config_from_hw(struct musb *pThis)
{
	u8 bEnd = 0, reg;
	struct musb_hw_ep *pEnd;
	void *pBase = pThis->pRegs;
	/* how many of a given size/direction found: */
	u8 b2kTxEndCount = 0;
	u8 b2kRxEndCount = 0;
	u8 b1kTxEndCount = 0;
	u8 b1kRxEndCount = 0;
	/* the smallest 2k or 1k ends in Tx or Rx direction: */
	u8 b2kTxEnd = 0;
	u8 b2kRxEnd = 0;
	u8 b1kTxEnd = 0;
	u8 b1kRxEnd = 0;
	/* for tracking smallest: */
	u16 w2kTxSize = 0;
	u16 w1kTxSize = 0;
	u16 w2kRxSize = 0;
	u16 w1kRxSize = 0;

	DBG(2, "<== static silicon ep config\n");

	for (bEnd = 1; bEnd < MUSB_C_NUM_EPS; bEnd++) {
		MGC_SelectEnd(pBase, bEnd);
		pEnd = &(pThis->aLocalEnd[bEnd]);

		/* read from core */
		reg = MGC_ReadCsr8(pBase, MGC_O_HDRC_FIFOSIZE, bEnd);
		if (!reg) {
			/* 0's returned when no more endpoints */
			break;
		}

		pEnd->wMaxPacketSizeTx = 1 << (reg & 0x0f);
		/* shared TX/RX FIFO? */
		if ((reg & 0xf0) == 0xf0) {
			pEnd->wMaxPacketSizeRx = 1 << (reg & 0x0f);
			pEnd->bIsSharedFifo = TRUE;
		} else {
			pEnd->wMaxPacketSizeRx = 1 << ((reg & 0xf0) >> 4);
			pEnd->bIsSharedFifo = FALSE;
		}

		/* track certain sizes to try to reserve a bulk resource */
		if (pEnd->wMaxPacketSizeTx >= 2048) {
			b2kTxEndCount++;
			if (!b2kTxEnd || (pEnd->wMaxPacketSizeTx < w2kTxSize)) {
				b2kTxEnd = bEnd;
				w2kTxSize = pEnd->wMaxPacketSizeTx;
			}
		}

		if (pEnd->wMaxPacketSizeRx >= 2048) {
			b2kRxEndCount++;
			if (!b2kRxEnd || (pEnd->wMaxPacketSizeRx < w2kRxSize)) {
				b2kRxEnd = bEnd;
				w2kRxSize = pEnd->wMaxPacketSizeRx;
			}
		}

		if (pEnd->wMaxPacketSizeTx >= 1024) {
			b1kTxEndCount++;
			if (!b1kTxEnd || (pEnd->wMaxPacketSizeTx < w1kTxSize)) {
				b1kTxEnd = bEnd;
				w1kTxSize = pEnd->wMaxPacketSizeTx;
			}
		}

		if (pEnd->wMaxPacketSizeRx >= 1024) {
			b1kRxEndCount++;
			if (!b1kRxEnd || (pEnd->wMaxPacketSizeRx < w1kTxSize)) {
				b1kRxEnd = bEnd;
				w1kRxSize = pEnd->wMaxPacketSizeRx;
			}
		}

		pThis->bEndCount++;
		pThis->wEndMask |= (1 << bEnd);
	}			/* init queues etc. etc. etc. */

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* if possible, reserve the smallest 2k-capable Tx end for bulk */
	if (b2kTxEnd && (b2kTxEndCount > 1)) {
		pThis->bulk_tx_end = pThis->aLocalEnd + b2kTxEnd;
		INFO("Reserved end %d for bulk double-buffered Tx\n", b2kTxEnd);
	}
	/* ...or try 1k */
	else if (b1kTxEnd && (b1kTxEndCount > 1)) {
		pThis->bulk_tx_end = pThis->aLocalEnd + b1kTxEnd;
		INFO("Reserved end %d for bulk Tx\n", b1kTxEnd);
	}

	/* if possible, reserve the smallest 2k-capable Rx end for bulk */
	if (b2kRxEnd && (b2kRxEndCount > 1)) {
		pThis->bulk_rx_end = pThis->aLocalEnd + b2kRxEnd;
		INFO("Reserved end %d for bulk double-buffered Rx\n", b2kRxEnd);
	}
	/* ...or try 1k */
	else if (b1kRxEnd && (b1kRxEndCount > 1)) {
		pThis->bulk_rx_end = pThis->aLocalEnd + b1kRxEnd;
		INFO("Reserved end %d for bulk Rx\n", b1kRxEnd);
	}
#endif

	return 0;
}
#endif

enum { MUSB_CONTROLLER_MHDRC, MUSB_CONTROLLER_HDRC, };

/* Initialize MUSB (M)HDRC part of the USB hardware subsystem;
 * configure endpoints, or take their config from silicon
 */
static int __init musb_core_init(u16 wType, struct musb *pThis)
{
#ifdef MUSB_AHB_ID
	u32 dwData;
#endif
	u8 reg;
	char *type;
	u16 wRelease, wRelMajor, wRelMinor;
	char aInfo[78], aRevision[32], aDate[12];
	void __iomem *pBase = pThis->pRegs;
	int status = 0;
	int i;

	/* log core options */
	MGC_SelectEnd(pBase, 0);
	reg = MGC_ReadCsr8(pBase, MGC_O_HDRC_CONFIGDATA, 0);

	strcpy(aInfo, (reg & MGC_M_CONFIGDATA_UTMIDW) ? "UTMI-16" : "UTMI-8");
	if (reg & MGC_M_CONFIGDATA_DYNFIFO) {
		strcat(aInfo, ", dyn FIFOs");
	}
	if (reg & MGC_M_CONFIGDATA_MPRXE) {
		pThis->bBulkCombine = TRUE;
		strcat(aInfo, ", bulk combine");
#ifndef C_MP_RX
		dev_dbg(pThis->controller, "ignoring bulk combine feature\n");
#endif
	}
	if (reg & MGC_M_CONFIGDATA_MPTXE) {
		pThis->bBulkSplit = TRUE;
		strcat(aInfo, ", bulk split");
#ifndef C_MP_TX
		dev_dbg(pThis->controller, "ignoring bulk split feature\n");
#endif
	}
	if (reg & MGC_M_CONFIGDATA_HBRXE) {
		strcat(aInfo, ", HB-ISO Rx");
	}
	if (reg & MGC_M_CONFIGDATA_HBTXE) {
		strcat(aInfo, ", HB-ISO Tx");
	}
	if (reg & MGC_M_CONFIGDATA_SOFTCONE) {
		strcat(aInfo, ", SoftConn");
	}

	pr_info("%s: ConfigData=0x%02x (%s)\n",
	       musb_driver_name, reg, aInfo);

#ifdef MUSB_AHB_ID
	dwData = musb_readl(pBase, 0x404);
	sprintf(aDate, "%04d-%02x-%02x", (dwData & 0xffff),
		(dwData >> 16) & 0xff, (dwData >> 24) & 0xff);
	/* FIXME ID2 and ID3 are unused */
	dwData = musb_readl(pBase, 0x408);
	printk("ID2=%lx\n", (long unsigned)dwData);
	dwData = musb_readl(pBase, 0x40c);
	printk("ID3=%lx\n", (long unsigned)dwData);
	reg = musb_readb(pBase, 0x400);
	wType = ('M' == reg) ? MUSB_CONTROLLER_MHDRC : MUSB_CONTROLLER_HDRC;
#else
	aDate[0] = 0;
#endif
	if (MUSB_CONTROLLER_MHDRC == wType) {
		pThis->bIsMultipoint = 1;
		type = "M";
	} else {
		pThis->bIsMultipoint = 0;
		type = "";
	}

	/* log release info */
	wRelease = musb_readw(pBase, MGC_O_HDRC_HWVERS);
	wRelMajor = (wRelease >> 10) & 0x1f;
	wRelMinor = wRelease & 0x3ff;
	snprintf(aRevision, 32, "%d.%d%s", wRelMajor,
		 wRelMinor, (wRelease & 0x8000) ? "RC" : "");
	pr_info("%s: %sHDRC RTL version %s %s\n",
	       musb_driver_name, type, aRevision, aDate);

	/* configure ep0 */
	pThis->aLocalEnd[0].wMaxPacketSizeTx = MGC_END0_FIFOSIZE;
	pThis->aLocalEnd[0].wMaxPacketSizeRx = MGC_END0_FIFOSIZE;

	/* discover endpoint configuration */
	pThis->bEndCount = 1;
	pThis->wEndMask = 1;

#ifdef MUSB_C_DYNFIFO_DEF
	if (!(reg & MGC_M_CONFIGDATA_DYNFIFO)) {
		ERR("Dynamic FIFOs not detected; reconfigure software\n");
		return -ENODEV;
	} else
		status = ep_config_from_table(pThis);
#else
	if (reg & MGC_M_CONFIGDATA_DYNFIFO) {
		ERR("Dynamic FIFOs detected; reconfigure software\n");
		return -ENODEV;
	} else
		status = ep_config_from_hw(pThis);
#endif
	if (status < 0)
		return status;

	/* finish init, and print endpoint config */
	for (i = 0; i < pThis->bEndCount; i++) {
		struct musb_hw_ep *hw_ep = pThis->aLocalEnd + i;

		hw_ep->fifo = MGC_FIFO_OFFSET(hw_ep->bLocalEnd) + pBase;
		hw_ep->regs = MGC_END_OFFSET(hw_ep->bLocalEnd, 0) + pBase;

		if (hw_ep->wMaxPacketSizeTx) {
			printk(KERN_DEBUG
			       "%s: hw_ep %d%s, %smax %d\n",
			       musb_driver_name, hw_ep->bLocalEnd,
			       hw_ep->bIsSharedFifo ? "shared" : "tx",
			       hw_ep->tx_double_buffered
			       ? "doublebuffer, " : "",
			       hw_ep->wMaxPacketSizeTx);
		}
		if (hw_ep->wMaxPacketSizeRx && !hw_ep->bIsSharedFifo) {
			printk(KERN_DEBUG
			       "%s: hw_ep %d%s, %smax %d\n",
			       musb_driver_name, hw_ep->bLocalEnd,
			       "rx",
			       hw_ep->rx_double_buffered
			       ? "doublebuffer, " : "",
			       hw_ep->wMaxPacketSizeRx);
		}
		if (!(hw_ep->wMaxPacketSizeTx || hw_ep->wMaxPacketSizeRx))
			DBG(1, "hw_ep %d not configured\n", hw_ep->bLocalEnd);
	}

	return 0;
}

/* -------------------------------- MEMORY ----------------------------- */

/* many common platforms have dma-coherent caches, which means that it's
 * safe to use kmalloc() memory for all i/o buffers without using any
 * cache flushing calls.  (unless you're trying to share cache lines
 * between dma and non-dma activities, which is a slow idea in any case.)
 */

#if	defined(CONFIG_X86)
#define USE_KMALLOC	1

#elif	defined(CONFIG_PPC) && !defined(CONFIG_NOT_COHERENT_CACHE)
#define USE_KMALLOC	1

#elif	defined(CONFIG_MIPS) && !defined(CONFIG_DMA_NONCOHERENT)
#define USE_KMALLOC	1

/* NOTE: there are other cases, including an x86-64 one ...  */

#endif

#ifndef	USE_KMALLOC
/* also use kmalloc when DMA is disabled! */
#define USE_KMALLOC	!is_dma_capable()
#endif

/* Allocate a dma-coherent buffer */
void *musb_alloc_buffer(struct musb *musb,
			size_t bytes, gfp_t gfp_flags, dma_addr_t * dma)
{
	void *addr = NULL;

	if (USE_KMALLOC) {
		addr = kmalloc(bytes, gfp_flags);
		if (addr)
			*dma = virt_to_phys(addr);
	} else {
		/* this allocates 2^X pages; the problem is that X is never
		 * negative.  e.g. allocating 32 bytes wastes most of page...
		 * the host side has some private dma pools to get rid of
		 * most of that cost.
		 * ... we explicitly force that allocation here to shut
		 * up messages from the ARM allocator
		 */
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		/* FIXME for dual-role configs, peripheral side should use
		 * the same buffer pools the host side will, when it switches
		 * over to the standard hcd framework ...
		 */
#endif
		addr = dma_alloc_coherent(musb->controller,
					  (bytes <
					   PAGE_SIZE) ? PAGE_SIZE : bytes, dma,
					  gfp_flags);
	}
	if (!addr)
		*dma = DMA_ADDR_INVALID;
	return addr;
}

/* Free memory previously allocated with AllocBufferMemory */
void musb_free_buffer(struct musb *musb,
		      size_t bytes, void *address, dma_addr_t dma)
{
	if (!USE_KMALLOC)
		dma_free_coherent(musb->controller,
				  (bytes < PAGE_SIZE) ? PAGE_SIZE : bytes,
				  address, dma);
	else
		kfree(address);
}

/*************************************************************************
 * Linux driver hooks
**************************************************************************/

#if 0

static irqreturn_t generic_interrupt(int irq, void *__hci, struct pt_regs *r)
{
	unsigned long flags;
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = __hci;

	spin_lock_irqsave(&musb->Lock, flags);

	musb->int_usb = musb_readb(musb->pRegs, MGC_O_HDRC_INTRUSB);
	musb->int_tx = musb_readw(musb->pRegs, MGC_O_HDRC_INTRTX);
	musb->int_rx = musb_readw(musb->pRegs, MGC_O_HDRC_INTRRX);
	musb->int_regs = r;

	if (musb->int_tx || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->Lock, flags);

	return retval;
}

#endif

/*
 * handle all the irqs defined by the HDRC core. for now we expect:  other
 * irq sources (phy, dma, etc) will be handled first, musb->int_* values
 * will be assigned, and the irq will already have been acked.
 *
 * called in irq context with spinlock held, irqs blocked
 */
irqreturn_t musb_interrupt(struct musb * musb)
{
	irqreturn_t retval = IRQ_NONE;
	u8 devctl, power;
	int ep_num;
	u32 reg;

	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
	power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);

	DBG(3, "<== IRQ %s usb%04x tx%04x rx%04x\n",
	    (devctl & MGC_M_DEVCTL_HM) ? "host" : "peripheral",
	    musb->int_usb, musb->int_tx, musb->int_rx);
	/* ignore requests when in error */
	if (MUSB_IS_ERR(musb)) {
		WARN("irq in error\n");
		musb_platform_disable(musb);
		return IRQ_NONE;
	}

	/* the core can interrupt us for multiple reasons; docs have
	 * a generic interrupt flowchart to follow
	 */
	if (musb->int_usb)
		retval |= musb_stage0_irq(musb, musb->int_usb, devctl, power);
#ifdef CONFIG_ARCH_DAVINCI_DM355
	else
		musb->vbuserr_retry = VBUSERR_RETRY_COUNT;
#endif
	/* "stage 1" is handling endpoint irqs */

	/* handle endpoint 0 first */
	if (musb->int_tx & 1) {
		if (devctl & MGC_M_DEVCTL_HM)
			retval |= musb_h_ep0_irq(musb);
		else
			retval |= musb_g_ep0_irq(musb);
	}

	/* RX on endpoints 1-15 */
	reg = musb->int_rx >> 1;
	ep_num = 1;
	while (reg) {
		if (reg & 1) {
			/* REVISIT just retval = ep->rx_irq(...) */
			retval = IRQ_HANDLED;
			if (devctl & MGC_M_DEVCTL_HM)
				musb_host_rx(musb, ep_num);
			else
				musb_g_rx(musb, ep_num);
		}

		reg >>= 1;
		ep_num++;
	}

	/* TX on endpoints 1-15 */
	reg = musb->int_tx >> 1;
	ep_num = 1;
	while (reg) {
		if (reg & 1) {
			/* REVISIT just retval |= ep->tx_irq(...) */
			retval = IRQ_HANDLED;
			if (devctl & MGC_M_DEVCTL_HM)
				musb_host_tx(musb, ep_num);
			else
				musb_g_tx(musb, ep_num, 1);
		}
		reg >>= 1;
		ep_num++;
	}

	/* finish handling "global" interrupts after handling fifos */
	if (musb->int_usb)
		retval |= musb_stage2_irq(musb, musb->int_usb, devctl, power);

	return retval;
}

#ifndef CONFIG_USB_INVENTRA_FIFO
static int __initdata use_dma = is_dma_capable();

/* "modprobe ... use_dma=0" etc */
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");

/*
 * DMA support
 */

static int musb_dma_completion(void *pPrivateData, u8 bLocalEnd, u8 bTransmit)
{
	struct musb *pThis = pPrivateData;
	const void __iomem *pBase = pThis->pRegs;
	u8 devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);

	/* called with controller lock already held */

	if (!bLocalEnd) {
#ifndef CONFIG_USB_TI_CPPI_DMA
		/* endpoint 0 */
		if (devctl & MGC_M_DEVCTL_HM)
			MGC_HdrcServiceDefaultEnd(pThis);
		else
			musb_g_ep0_irq(pThis);
#endif
	} else {
		/* endpoints 1..15 */
		if (bTransmit) {
			if (devctl & MGC_M_DEVCTL_HM)
				musb_host_tx(pThis, bLocalEnd);
			else
				musb_g_tx(pThis, bLocalEnd, 0);
		} else {
			/* receive */
			if (devctl & MGC_M_DEVCTL_HM)
				musb_host_rx(pThis, bLocalEnd);
			else
				musb_g_rx(pThis, bLocalEnd);
		}
	}

	return 0;
}

#else
#define	musb_dma_completion	NULL
#define use_dma			is_dma_capable()
#endif

/* --------------------------------------------------------------------------
 * Init support
 */

static struct musb *__init allocate_instance(void __iomem * mbase)
{
	struct musb *pThis;
	struct musb_hw_ep *ep;
	int epnum, tmp = 0;

	/* allocate */
	pThis = kzalloc(sizeof *pThis, GFP_KERNEL);
	if (!pThis)
		return NULL;

	pThis->pRegs = mbase;
	pThis->ctrl_base = mbase;
	pThis->nIrq = -ENODEV;
	for (epnum = 0, ep = pThis->aLocalEnd;
	     epnum < MUSB_C_NUM_EPS; epnum++, ep++) {

		ep->musb = pThis;
		ep->bLocalEnd = (epnum % 2) ? ++tmp : tmp;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
		ep->musb_mode[0].dma = 0;
		ep->musb_mode[0].ep = ep;
		ep->musb_mode[1].dma = 1;
		ep->musb_mode[1].ep = ep;
		/* busctl regs too? */
		INIT_LIST_HEAD(&(ep->urb_list));
		init_timer(&pThis->RootHub.Timer);
#ifdef CONFIG_ARCH_DAVINCI_DM355
		pThis->vbuserr_retry = VBUSERR_RETRY_COUNT;
#endif
#endif
	}
	return pThis;
}

static void musb_free(struct musb *musb)
{
#ifdef CONFIG_ARCH_DAVINCI_DM646x
	struct platform_device *pdev = to_platform_device(musb->controller);
#endif
	/* this has multiple entry modes. it handles fault cleanup after
	 * probe(), where things may be partially set up, as well as rmmod
	 * cleanup after everything's been de-activated.
	 */

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if (musb->pBus) {
		struct usb_bus *bus = musb->pBus;

		MGC_VirtualHubStop(&musb->RootHub);
		if (bus->root_hub) {
			/* this also disconnects any active children */
			usb_disconnect(&bus->root_hub);
		}
		usb_deregister_bus(bus);
	}
#endif
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	musb_gadget_cleanup(musb);
#endif

	if (musb->nIrq >= 0)
		free_irq(musb->nIrq, musb);
#ifdef CONFIG_ARCH_DAVINCI_DM646x
	free_irq(platform_get_irq(pdev, 1),
		dev_get_drvdata(&pdev->dev));
#endif
	if (is_dma_capable() && musb->pDmaController) {
		musb->pDmaController->pfDmaStopController(musb->pDmaController->
							  pPrivateData);
		dma_controller_factory.pfDestroyDmaController(musb->
							      pDmaController);
	}
	musb_platform_exit(musb);
#ifdef CONFIG_ARCH_DAVINCI
	if (musb->clock) {
		clk_disable(musb->clock);
		clk_put(musb->clock);
	}
#endif
	/* FIXME make sure all the different faces of this driver
	 * coordinate their refcounting, so the same release() is
	 * called when the host or gadget (or whatever) is the last
	 * one released
	 */
	kfree(musb);
}

/*
 * Perform generic per-controller initialization.
 *
 * @pDevice: the controller (already clocked, etc)
 * @nIrq: irq
 * @pRegs: virtual address of controller registers,
 * 	not yet corrected for platform-specific offsets
 */
static int __init
musb_init_controller(struct device *dev, int nIrq, void __iomem *ctrl)
{
	int status;
	struct musb *pThis;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
#ifdef CONFIG_ARCH_DAVINCI_DM646x
	struct platform_device *pdev = to_platform_device(dev);
	int nIrq1 = platform_get_irq(pdev, 1);

#endif

	/* The driver might handle more features than the board; OK.
	 * Fail when the board needs a feature that's not enabled.
	 */

	if (!plat) {
		dev_dbg(dev, "no platform_data?\n");
		return -ENODEV;
	}

	/* DaVinci platforms sets the mode in musb_platform_init  */
	/* so skip the check */
#ifndef CONFIG_ARCH_DAVINCI
	switch (plat->mode) {
	case MUSB_HOST:
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		break;
#else
		goto bad_config;
#endif
	case MUSB_PERIPHERAL:
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		break;
#else
		goto bad_config;
#endif
	case MUSB_OTG:
#ifdef CONFIG_USB_MUSB_OTG
		break;
#else
	      bad_config:
#endif
	default:
		dev_dbg(dev, "incompatible Kconfig role setting\n");
		return -EINVAL;
	}
#endif

	/* allocate */
	pThis = allocate_instance(ctrl);
	if (!pThis)
		return -ENOMEM;

	pThis->controller = dev;
	dev_set_drvdata(dev, pThis);

	spin_lock_init(&pThis->Lock);
	pThis->board_mode = plat->mode;

	/* assume vbus is off */

	/* platform adjusts pThis->pRegs if needed,
	 * and activates clocks
	 */
	status = musb_platform_init(pThis);

	if (status < 0)
		goto fail;

	plat = dev->platform_data;
	if (use_dma && dev->dma_mask) {
		pThis->pDmaController = dma_controller_factory.
		    pfNewDmaController(musb_dma_completion, pThis,
				       pThis->pRegs);
		if (pThis->pDmaController)
			pThis->pDmaController->pfDmaStartController(pThis->
								    pDmaController->
								    pPrivateData);
	}

	/* Hack for now and save the value here
	   so that we can use the use_dma module param */
	pThis->old_dma_mask = dev->dma_mask;

	/* ideally this would be abstracted in platform setup */
	if (!is_dma_capable() || !pThis->pDmaController)
		dev->dma_mask = NULL;

	/* be sure interrupts are disabled before connecting ISR */
	musb_platform_disable(pThis);

	/* setup musb parts of the core (especially endpoints) */
	status = musb_core_init(plat->multipoint
			   ? MUSB_CONTROLLER_MHDRC
			   : MUSB_CONTROLLER_HDRC, pThis);
	if (status < 0)
		goto fail;

	/* attach to the IRQ */
	if (request_irq (nIrq, pThis->isr, 0, dev->bus_id, pThis)) {
		dev_err(dev, "request_irq %d failed!\n", nIrq);
		status = -ENODEV;
		goto fail;
	}

#ifdef CONFIG_ARCH_DAVINCI_DM646x
	/* attach to the IRQ */
	if (request_irq(nIrq1, pThis->isr, 0, dev->bus_id, pThis)) {
		dev_err(dev, "request_irq %d failed!\n", nIrq1);
		status = -ENODEV;
		goto fail;
	}
#endif
	pThis->nIrq = nIrq;

	/* Initialize the tasklet to check for FIFO status on completion of
	 * a request
	 */
	tasklet_init (&pThis->fifo_check, musb_fifo_check_tasklet,
			(unsigned long)pThis);

	pr_info("%s: USB %s mode controller at %p using %s, IRQ %d\n",
		musb_driver_name, ( {
			char *s;
			switch (pThis->board_mode) {
			case MUSB_HOST: s = "Host"; break;
			case MUSB_PERIPHERAL: s = "Peripheral"; break;
			default: s = "OTG"; break;
			}; s; }
			), ctrl, (is_dma_capable() && pThis->pDmaController)
				? "DMA" : "PIO", pThis->nIrq) ;

/* FIXME:
  - convert to the HCD framework
  - if (board_mode == MUSB_OTG) do startup with peripheral
  - ... involves refcounting updates */

/* HBG 21SEPT2006 modified for OTG modifications */
/*===================================================*/
#ifdef CONFIG_USB_MUSB_OTG
	{
		struct proc_dir_entry *pde;
		pde = create_proc_entry("driver/otg", 0, NULL);
		if (pde) {
			pde->write_proc = musb_app_inputs;
			pde->data = pThis;
		}

	}
	MGC_OtgMachineInit(&pThis->OtgMachine, pThis);
#endif
//===================================================

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* host side needs more setup, except for no-host modes */
	if (pThis->board_mode != MUSB_PERIPHERAL) {
		struct usb_bus *bus;

		/* allocate and register bus */
		bus = usb_alloc_bus(&musb_host_bus_ops);
		if (!bus) {
			dev_dbg(dev, "usb_alloc_bus fail\n");
			status = -ENOMEM;
			goto fail;
		}

		pThis->pBus = bus;

		init_timer(&pThis->Timer);
		pThis->Timer.function = musb_timer_done;
		pThis->Timer.data = (unsigned long)pThis;

		/* register the bus */
		bus->controller = dev;
		bus->bus_name = dev->bus_id;
		if (pThis->board_mode == MUSB_OTG)
			bus->otg_port = 1;
		pThis->pBus->hcpriv = (void *)pThis;
		/*HBG 21SEPT2006 removed as part of OTG improvements */
		//-----------------------------------------------------
		//pThis->xceiv.host = bus;
		//-----------------------------------------------------

		/* FIXME root hub setup changed in 2.6.current kernels
		 * even without involving the hcd framework ...
		 */

		/* FIXME:  hcd framework allocates the bus ...
		 * else bus->release(bus) method looks necessary
		 */

		status = usb_register_bus(bus);
		if (status < 0) {
			kfree(bus);
			pThis->pBus = NULL;
			goto fail;
		}

		/* init virtual root hub */
		if (!MGC_VirtualHubInit(&pThis->RootHub, pThis->pBus, pThis)) {
			dev_dbg(dev, "Virtual Hub init failed\n");
			status = -ENODEV;
			goto fail;
		}
	}
#endif				/* CONFIG_USB_MUSB_HDRC_HCD */

#ifdef CONFIG_USB_MUSB_OTG
	/* if present, this gets used even on non-otg boards */
	MGC_OtgMachineInit(&pThis->OtgMachine, pThis);
#endif

	/* For the host-only role, we can activate right away.
	 * Otherwise, wait till the gadget driver hooks up.
	 *
	 * REVISIT switch to compile-time is_role_host() etc
	 * to get rid of #ifdeffery
	 */
	switch (pThis->board_mode) {
#ifdef CONFIG_USB_MUSB_OTG
	case MUSB_OTG:
#endif
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case MUSB_HOST:
		MUSB_HST_MODE(pThis);
		/* HBG 21SEPT2006 removed as part of OTG improvements */
		//-----------------------------------------------------
		//pThis->xceiv.state = OTG_STATE_A_IDLE;
		//-----------------------------------------------------
		status = usb_register_root_hub(pThis->RootHub.pDevice, dev);
#if 0
		/* FIXME 2.6.10 doesn't budget root hub power correctly, AND
		 * can only modify budgets after hub driver binds
		 */
		if (status == 0)
			hub_set_power_budget(pThis->RootHub.pDevice,
					     2 * (plat->power ? : 250));
#endif

		DBG(1, "%s mode, status %d, devctl %02x %c\n",
			"HOST", status,
			musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL),
			(musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL)
			& MGC_M_DEVCTL_BDEVICE ? 'B' : 'A'));

#ifndef CONFIG_USB_MUSB_OTG
		break;
#endif
#endif
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	case MUSB_PERIPHERAL:
		MUSB_DEV_MODE(pThis);
		status = musb_gadget_setup(pThis);

		DBG(1, "%s mode, status %d, dev%02x\n",
		    "PERIPHERAL", status,
		    musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL));
#ifndef CONFIG_USB_MUSB_OTG
		break;
#endif
#endif
	default:
		break;
	}

	if (status == 0)
		musb_debug_create("driver/musb_hdrc", pThis);
	else {
	      fail:
		musb_free(pThis);
	}
	return status;
}

/*-------------------------------------------------------------------------*/

/* all implementations (PCI bridge to FPGA, VLYNQ, etc) should just
 * bridge to a platform device; this driver then suffices.
 */

static int __init musb_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 0);
	struct resource *iomem;
	void __iomem *base;
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem || irq == 0)
		return -ENODEV;

#ifdef CONFIG_ARCH_DAVINCI
	base = ioremap(iomem->start, iomem->end - iomem->start + 1);
	if (!base) {
		dev_err(dev, "ioremap failed\n");
		return -ENOMEM;
	}
#else
	base = (void *__iomem)iomem->start;
#endif
	return musb_init_controller(dev, irq, base);
}

static int __exit musb_remove(struct device *dev)
{
	struct musb *musb = dev_get_drvdata(dev);

	/* this gets called on rmmod.
	 *  - Host mode: host may still be hactive
	 *  - Peripheral mode: peripheral is deactivated (or never-activated)
	 *  - OTG mode: both roles are deactivated (or never-activated)
	 */

	// restore old value of dma_mask
	dev->dma_mask = musb->old_dma_mask;

	musb_shutdown(musb->controller);
	musb_debug_delete("driver/musb_hdrc", musb);
	musb_free(musb);

	dev_set_drvdata(dev, NULL);
	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT when power savings matter on DaVinci, look at turning
 * off its phy clock during system suspend.
 */

static int musb_suspend(struct device *dev, u32 state, u32 level)
{
	struct musb *musb = dev_get_drvdata(dev);
	unsigned long flags;

	if (level != SUSPEND_POWER_DOWN || !musb->clock)
		return 0;

	spin_lock_irqsave(&musb->Lock, flags);

	if (is_peripheral_active(musb)) {
		/* FIXME force disconnect unless we know USB will wake
		 * the system up quickly enough to respond ...
		 */
	} else if (is_host_active(musb)) {
		/* we know all the children are suspended; sometimes
		 * they will even be wakeup-enabled
		 */
	}
#ifdef CONFIG_ARCH_DAVINCI
	clk_disable(musb->clock);
#endif
	spin_unlock_irqrestore(&musb->Lock, flags);
	return 0;
}

static int musb_resume(struct device *dev, u32 level)
{
	struct musb *musb = dev_get_drvdata(dev);
	unsigned long flags;

	if (level != RESUME_POWER_ON || !musb->clock)
		return 0;

	spin_lock_irqsave(&musb->Lock, flags);
#ifdef CONFIG_ARCH_DAVINCI
	clk_enable(musb->clock);
#endif
	/* for static cmos like DaVinci, register values were preserved
	 * unless for some reason the whole soc powered down and we're
	 * not treating that as a whole-system restart (e.g. swsusp)
	 */
	spin_unlock_irqrestore(&musb->Lock, flags);
	return 0;
}

#else
#define	musb_suspend	NULL
#define	musb_resume	NULL
#endif

static struct device_driver musb_driver = {
	.name = (char *)musb_driver_name,
	.bus = &platform_bus_type,
	.owner = THIS_MODULE,
	.probe = musb_probe,
	.remove = __exit_p(musb_remove),
	.shutdown = musb_shutdown,
	.suspend = musb_suspend,
	.resume = musb_resume,
};

/*-------------------------------------------------------------------------*/

static int __init musb_init(void)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if (usb_disabled())
		return 0;
#endif

	pr_info("%s: version " MUSB_VERSION " "
#ifdef CONFIG_USB_INVENTRA_FIFO
		"[pio]"
#elif defined(CONFIG_USB_TI_CPPI_DMA)
		"[cppi-dma]"
#elif defined(CONFIG_USB_INVENTRA_DMA)
		"[musb-dma]"
#else
		"[?]"
#endif
		" "
#ifdef CONFIG_USB_MUSB_OTG
		"[otg: peripheral+host]"
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
		"[peripheral]"
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
		"[host]"
#endif
		" [debug=%d]\n", musb_driver_name, MGC_GetDebugLevel());

	return driver_register(&musb_driver);
}

/* make us init after usbcore and before usb
 * gadget and host-side drivers start to register
 */
subsys_initcall(musb_init);
//device_initcall(musb_init);

static void __exit musb_cleanup(void)
{
#ifdef CONFIG_USB_MUSB_OTG
	remove_proc_entry("driver/otg", NULL);
#endif

	driver_unregister(&musb_driver);
}
module_exit(musb_cleanup);

#ifdef CONFIG_USB_MUSB_OTG
/* HBG Testing 15SEPT2006 */

static int musb_app_inputs(struct file *file, const char __user * buffer,
			unsigned long count, void *data)
{
	u8 busReq;
	struct musb *pThis = data;
	copy_from_user(&busReq, buffer, 1);
	switch (busReq) {
	case 'i':
		busReq = MGC_OTG_REQUEST_START_BUS;
		break;
	case 's':
		busReq = MGC_OTG_REQUEST_SUSPEND_BUS;
		break;
	case 'e':
		busReq = MGC_OTG_REQUEST_DROP_BUS;
		break;
	/*case 'b':
		busReq=MGC_OTG_REQUEST_START_BUS;
		pThis->OtgMachine.bRequest = busReq;
		return 1;
		break;
	case 'H':
	case 'h':
		BusReq=3;
		printk("Start HNP\n");
		break;
	case 'S':
		BusReq=4;
		printk("Suspend the Bus\n");
		break; */
	default:
		return 1;
		break;
	}
	pThis->OtgMachine.bRequest = busReq;
	/* This is user initiated change so consider that
	 * the connection is active by default
	 */
	otg_input_changed_X(pThis, FALSE, TRUE);
	return 1;
}
#endif

/* HBG 22SEPT 2006 */
/* ID pin sensing support to OTG.c */
u8 is_otg_b_device(struct musb *pThis)
{
	u8 Devctl;
	Devctl = musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL);
	return ((Devctl & MGC_M_DEVCTL_BDEVICE) != 0);
}
