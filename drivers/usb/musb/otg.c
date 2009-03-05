/******************************************************************
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

/* OTG state machine status 8-mar:
 *
 *   - on DaVinci
 *        + EVM gamma boards have troublesome C133, preventing
 *          conformant timings for A_WAIT_VFALL transitions
 *        + ID-pin based role initialization and VBUS switching
 *	    seems partly functional ... seems to bypass this code.
 *        + haven't tried HNP or SRP.
 *
 *   - needs updating along the lines of <linux/usb_otg.h>
 *
 *   - doesn't yet use all the linux 2.6.10 usbcore hooks for OTG, but
 *     some of the conversion (and consequent shrinkage) has begun.
 *
 *   - it's not clear if any version of this code ever have passed
 *     the USB-IF OTG tests even at full speed; presumably not.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <linux/usb.h>

#include "musbdefs.h"
#include "otg.h"
#include "musb_host.h"

#include "../core/hcd.h"

static void otg_set_session(struct musb *musb, u8 bSession, u8 bADevice)
{
	void *__iomem mregs = musb->pRegs;
	u8 devctl = musb_readb(mregs, MGC_O_HDRC_DEVCTL);

	if (bSession)
		devctl |= MGC_M_DEVCTL_SESSION;
	else
		devctl &= ~MGC_M_DEVCTL_SESSION;

	musb_writeb(mregs, MGC_O_HDRC_DEVCTL, devctl);
}

/* caller has irqlocked musb */
static void otg_state_changed(struct musb *musb, enum usb_otg_state state)
{
	/* caller should pass the timeout here */
	unsigned long timer = 0;

	if (state == musb->OtgMachine.xceiv.state)
		return;

	DBG(1, "%d --> %d\n", musb->OtgMachine.xceiv.state, state);
	musb->OtgMachine.xceiv.state = state;

	/* OTG timeouts the hardware doesn't handle:
	 *  - ...
	 */

	switch (state) {
	case OTG_STATE_A_HOST:
	case OTG_STATE_B_HOST:
		/* TODO: graceful Gadget shutdown */
		MUSB_HST_MODE(musb);
		break;

	case OTG_STATE_A_PERIPHERAL:
	case OTG_STATE_B_PERIPHERAL:
		/* TODO: graceful host shutdown */
		MUSB_DEV_MODE(musb);
		break;

	default:
		/* TODO: graceful host shutdown */
		/* TODO: graceful Gadget shutdown */
		DBG(1, "state change to %d?\n", state);
		MUSB_OTG_MODE(musb);
		break;
	}

	if (timer)
		mod_timer(&musb->OtgMachine.Timer, jiffies + timer);
	else
		del_timer(&musb->OtgMachine.Timer);

	/* FIXME  the otg state implies MUSB_MODE().  Properly track
	 * xceiv.state, then remove OtgMachine.bState and MUSB_MODE...
	 */
	DBG(2, "==> OTG state %d(%d), mode %s\n",
	    state, musb->OtgMachine.xceiv.state, MUSB_MODE(musb));
}

/**
 * Timer expiration function to complete the interrupt URB on changes
 * @param ptr standard expiration param (hub pointer)
 */
static void otg_timeout(unsigned long ptr)
{
	struct otg_machine *pMachine = (void *)ptr;
	void __iomem *mregs;
	u8 devctl;
	struct musb *musb = pMachine->musb;
	unsigned long flags;

	/* REVISIT:  a few of these cases _require_ (per the OTG spec)
	 * some sort of user notification, such as turning on an LED
	 * or displaying a message on the screen; INFO() not enough.
	 */

	spin_lock_irqsave(&musb->Lock, flags);
	switch (pMachine->xceiv.state) {
	case OTG_STATE_UNDEFINED:
	case OTG_STATE_A_IDLE:
	case OTG_STATE_B_IDLE:
	case OTG_STATE_B_SRP_INIT:
		INFO("SRP failed\n");
		otg_set_session(pMachine->musb, FALSE, FALSE);
		otg_state_changed(pMachine->musb, OTG_STATE_UNDEFINED);
		DBG(1, "OTG_STATE_B_SRP_INIT->OTG_STATE_UNDEFINED\n");
		break;

	case OTG_STATE_B_WAIT_ACON:
		INFO("No response from A-device\n");
		mregs = pMachine->musb->pRegs;
		devctl = musb_readb(mregs, MGC_O_HDRC_DEVCTL);
		musb_writeb(mregs, MGC_O_HDRC_DEVCTL,
			    devctl & ~MGC_M_DEVCTL_HR);
		otg_set_session(pMachine->musb, TRUE, FALSE);
		otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
		DBG(1, "OTG_STATE_B_WAIT_ACON->OTG_STATE_B_IDLE\n");
		break;

	case OTG_STATE_A_WAIT_BCON:
		/* REVISIT we'd like to force the VBUS-off path here... */
		INFO("No response from B-device\n");
		otg_set_session(pMachine->musb, FALSE, FALSE);
		/* transition via OTG_STATE_A_WAIT_VFALL */
		otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
		DBG(1, "OTG_STATE_A_WAIT_BCON->OTG_STATE_A_IDLE\n");
		break;

	case OTG_STATE_A_SUSPEND:
		/* FIXME b-dev HNP is _optional_ so this is no error */
		INFO("No B-device HNP response\n");
		otg_set_session(pMachine->musb, FALSE, FALSE);
		/* transition via OTG_STATE_A_WAIT_VFALL */
		otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
		DBG(1, "OTG_STATE_A_SUSPEND->OTG_STATE_A_IDLE\n");
		break;
	default:
		WARN("timeout in state %d, now what?\n", pMachine->xceiv.state);
	}
	spin_unlock_irqrestore(&musb->Lock, flags);
}

void MGC_OtgMachineInit(struct otg_machine *pMachine, struct musb *musb)
{
	memset(pMachine, 0, sizeof *pMachine);
	spin_lock_init(&pMachine->Lock);
	pMachine->musb = musb;

	init_timer(&pMachine->Timer);
	pMachine->Timer.function = otg_timeout;
	pMachine->Timer.data = (unsigned long)pMachine;
	/* HBG 21 SEPT 2006 OTG */
	pMachine->xceiv.state = OTG_STATE_UNDEFINED;

/* HBG 21SEPT2006 Added as part of OTG implementation*/
//====================================================
	pMachine->xceiv.set_host = musb_set_host;
	pMachine->xceiv.set_peripheral = musb_set_peripheral;
	pMachine->xceiv.start_hnp = musb_start_hnp;
	pMachine->xceiv.start_srp = musb_start_srp;
//====================================================

}

void MGC_OtgMachineDestroy(struct otg_machine *pMachine)
{
	/* stop timer */
	del_timer_sync(&pMachine->Timer);
}

/* caller has irqlocked musb */
void MGC_OtgMachineInputsChanged(struct otg_machine *pMachine,
				 const MGC_OtgMachineInputs * pInputs)
{

	u8 devctl = musb_readb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL);
	u8 power = musb_readb(pMachine->musb->pRegs, MGC_O_HDRC_POWER);
	DBG(1, "Devctl => %02x\t Power =>%02x\n", devctl, power);
	DBG(2, "<== bState %d%s%s%s%s%s%s\n",
	    pMachine->xceiv.state,
	    pInputs->bSession ? ", sess" : "",
	    pInputs->bSuspend ? ", susp" : "",
	    pInputs->bConnection ? ", bcon" : "",
	    pInputs->bReset ? ", reset" : "",
	    pInputs->bConnectorId ? ", B-Dev" : ", A-Dev",
	    pInputs->bVbusError ? ", vbus_error" : "");

	switch (pMachine->xceiv.state) {

	case OTG_STATE_UNDEFINED:
	case OTG_STATE_A_IDLE:
	case OTG_STATE_B_IDLE:
		if (pInputs->bReset &&
		    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb,
					  OTG_STATE_B_PERIPHERAL);
			DBG(1, "OTG_STATE_UNDEFINED->OTG_STATE_B_PERIPHERAL\n");
		} else if (pInputs->bConnection &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb, OTG_STATE_A_HOST);
			DBG(1, "OTG_STATE_UNDEFINED->OTG_STATE_A_HOST\n");
		} else if (pInputs->bSession &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			if (pInputs->bSuspend) {
				break;
			} else {
				otg_state_changed(pMachine->musb,
						  OTG_STATE_A_WAIT_BCON);
				DBG(1,
				    "OTG_STATE_UNDEFINED->OTG_STATE_A_WAIT_BCON\n");
				otg_set_session(pMachine->musb, TRUE, TRUE);

				/* REVISIT This can cause problem to meet OPT compliance
				 * but we need to wait for becoming session to stable
				 */
				mdelay(36);

				mod_timer(&pMachine->Timer, jiffies
					  +
					  msecs_to_jiffies
					  (MGC_OTG_T_A_WAIT_BCON));
			}
		} else if (pMachine->bRequest == MGC_OTG_REQUEST_START_BUS) {
			pMachine->bRequest = 0;
			otg_set_session(pMachine->musb, TRUE, FALSE);
			/*mdelay(36);
			   b_device = is_otg_b_device(pMachine->musb);
			   if(b_device){
			   otg_state_changed(pMachine->musb,
			   OTG_STATE_B_SRP_INIT);
			   otg_set_session(pMachine->musb,TRUE,FALSE); */
			mod_timer(&pMachine->Timer, jiffies
				  + msecs_to_jiffies(MGC_OTG_T_B_SRP_FAIL));
			DBG(1, "OTG_STATE_UNDEFINED->OTG_STATE_B_SRP_INIT\n");
			/*}else{
			   otg_state_changed(pMachine->musb,
			   OTG_STATE_A_WAIT_BCON);
			   DBG(1,"OTG_STATE_UNDEFINED->OTG_STATE_A_WAIT_BCON\n");

			   mod_timer(&pMachine->Timer, jiffies
			   + msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
			   } */
		}
		break;

	case OTG_STATE_B_SRP_INIT:
		if (pInputs->bReset &&
		    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb,
					  OTG_STATE_B_PERIPHERAL);
			DBG(1,
			    "OTG_STATE_B_SRP_INIT->OTG_STATE_B_PERIPHERAL\n");

			/* SRP is detected by A-Device and B-gets reset
			 * Event is compeleted and no more need of MGC_OTG_T_B_SRP_FAIL
			 * timer
			 */
			del_timer(&pMachine->Timer);
		}
		break;

	case OTG_STATE_B_PERIPHERAL:
		if (!pInputs->bSession) {
			/* We got a condition were we coule have
			   requested for a HNP session from application
			   but user disconnected the cable.
			   Ensure that we reset the current request.
			 */
			pMachine->bRequest = 0;
			musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL,
				    devctl & ~MGC_M_DEVCTL_HR);
			otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
			DBG(1, "OTG_STATE_B_PERIPEHRL->OTG_STATE_B_IDLE\n1");
		} else if (pMachine->musb->pBus->b_hnp_enable
			   && pInputs->bSuspend) {
			pMachine->bRequest = 0;
			otg_state_changed(pMachine->musb,
					  OTG_STATE_B_WAIT_ACON);
			DBG(1,
			    "OTG_STATE_B_PERIPHERAL->OTG_STATE_B_WAIT_ACON\n");
			mod_timer(&pMachine->Timer,
				  jiffies +
				  msecs_to_jiffies(MGC_OTG_T_B_ASE0_BRST));
		}
		break;

	case OTG_STATE_B_WAIT_ACON:
		if (pInputs->bConnection &&
		    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb, OTG_STATE_B_HOST);
			DBG(1, "OTG_STATE_B_WAIT_ACON->OTG_STATE_B_HOST\n");
			/*Enumeration must start here */
		} else if (!pInputs->bSession &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
			DBG(1, "OTG_STATE_B_WAIT_ACON->OTG_STATE_B_IDLE\n");
		} else if (!pInputs->bSuspend &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb,
					  OTG_STATE_B_PERIPHERAL);
			DBG(1,
			    "OTG_STATE_B_WAIT_ACON->OTG_STATE_B_PERIPHERAL\n");
		}
		break;

	case OTG_STATE_B_HOST:
		if (pMachine->bRequest == MGC_OTG_REQUEST_SUSPEND_BUS) {
			u8 power =
			    musb_readb(pMachine->musb->pRegs, MGC_O_HDRC_POWER);
			musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_POWER,
				    power | MGC_M_POWER_SUSPENDM);
			mdelay(10);
			pMachine->bRequest = 0;
			DBG(1, "Suspended bus in B_Host mode\n");

		} else if (!pInputs->bConnection &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			DBG(1, "OTG_STATE_B_HOST->OTG_STATE_B_IDLE\n");
			otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
		} else if (pInputs->bConnection && !pInputs->bReset) {
			/* REVISIT seems incomplete */
		}
		break;

	case OTG_STATE_A_WAIT_BCON:
		if (pInputs->bConnection &&
		    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb, OTG_STATE_A_HOST);
			DBG(1, "OTG_STATE_A_WAIT_BCON->OTG_STATE_A_HOST\n");

			/* Delete OTG_STATE_A_WAIT_BCON Timer as Connection is received */
			del_timer(&pMachine->Timer);

		} else if (pInputs->bReset &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			/* FIXME there is no such transition */
			otg_state_changed(pMachine->musb,
					  OTG_STATE_A_PERIPHERAL);
			DBG(1,
			    "OTG_STATE_A_WAIT_BCON->OTG_STATE_A_PERIPHERAL\n");
		}
		break;

	case OTG_STATE_A_HOST:
		if (pMachine->bRequest == MGC_OTG_REQUEST_SUSPEND_BUS ||
		    pInputs->bSuspend) {
			u8 power = musb_readb(pMachine->musb->pRegs,
					      MGC_O_HDRC_POWER);
			pMachine->bRequest = 0;
			musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_POWER,
				    power | MGC_M_POWER_SUSPENDM);
			DBG(1, "OTG_STATE_A_HOST->OTG_STATE_A_SUSPEND\n");
			otg_state_changed(pMachine->musb, OTG_STATE_A_SUSPEND);
			mod_timer(&pMachine->Timer, jiffies
				  + msecs_to_jiffies(MGC_OTG_T_AIDL_BDIS));
		} else if ((!pInputs->bConnection &&
			    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN))
			   || pMachine->bRequest == MGC_OTG_REQUEST_DROP_BUS) {
			pMachine->bRequest = 0;
			otg_state_changed(pMachine->musb,
					  OTG_STATE_A_WAIT_BCON);
			DBG(1, "OTG_STATE_A_HOST->OTG_STATE_A_WAIT_BCON\n");
			mod_timer(&pMachine->Timer, jiffies
				  + msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
		} else if (pInputs->bConnection && !pInputs->bReset) {
			/* REVISIT seems incomplete */
		}
		break;

	case OTG_STATE_A_SUSPEND:
		if (!pInputs->bConnection &&
		    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {

			/* Remove MGC_OTG_T_AIDL_BDIS Timer Disconnect event occur as part of
			 * HNP initialization after suspending the USB BUS */
			del_timer(&pMachine->Timer);

			if (pMachine->musb->pBus->b_hnp_enable) {
				DBG(1,
				    "OTG_STATE_A_SUSPEND->OTG_STATE_A_PERIPHERAL\n");
				otg_state_changed(pMachine->musb,
						  OTG_STATE_A_PERIPHERAL);
			} else {
				otg_state_changed(pMachine->musb,
						  OTG_STATE_A_WAIT_BCON);
				DBG(1,
				    "OTG_STATE_A_SUSPEND->OTG_STATE_A_WAIT_BCON\n");
				mod_timer(&pMachine->Timer,
					  jiffies +
					  msecs_to_jiffies
					  (MGC_OTG_T_A_WAIT_BCON));
			}
		} else if (!pInputs->bSuspend &&
			   (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			otg_state_changed(pMachine->musb, OTG_STATE_A_HOST);
			DBG(1, "OTG_STATE_A_SUSPEND->OTG_STATE_A_HOST\n");
		}
		break;

	case OTG_STATE_A_PERIPHERAL:
		if (!pInputs->bSession &&
		    (pMachine->bRequest == MGC_OTG_REQUEST_UNKNOWN)) {
			/* transition via OTG_STATE_A_WAIT_VFALL */
			musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL,
				    devctl & ~MGC_M_DEVCTL_HR);
			otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
			DBG(1, "OTG_STATE_A_PERIPHERAL->OTG_STATE_A_IDLE\n");
		} else if (pInputs->bSuspend) {
			pMachine->bRequest = 0;
			musb_writeb(pMachine->musb->pRegs, MGC_O_HDRC_DEVCTL,
				    devctl & ~MGC_M_DEVCTL_HR);
			otg_state_changed(pMachine->musb,
					  OTG_STATE_A_WAIT_BCON);
			DBG(1,
			    "OTG_STATE_A_PERIPHERAL->OTG_STATE_A_WAIT_BCON\n");
			mod_timer(&pMachine->Timer,
				  jiffies +
				  msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
		}
		break;

	default:
		WARN("event in state %d, now what?\n", pMachine->xceiv.state);
	}
}
