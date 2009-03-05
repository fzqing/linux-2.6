/*
 * linux/drivers/usb/gadget/pxa3xx_otg.c
 * Intel Monahans usb otg controller
 *
 * Copyright (C) 2005 Intel Corporation
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
 *

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG

#define MHN_OTG_VBUS_PMIC

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/proc_fs.h>

#include <linux/usb.h>
#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>
#include <linux/usb_otg.h>

#include <asm/arch/udc.h>
#include <asm/uaccess.h>
#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>
#ifdef MHN_OTG_VBUS_PMIC
#include <asm/arch/mhn_pmic.h>
#include <asm/arch/arava.h>
#endif
#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>
#include "pxa3xx_otg.h"

#ifdef CONFIG_USB_OTG_PXA3XX_U2D
#include "../gadget/pxa3xx_u2d.h"
#endif

#ifndef CONFIG_PXA310
extern u32 status_reg_UDCOTGISR;
#else
extern u32 status_reg_U2DOTGISR;
#endif

static char *state_string[] = {
	"undefined",
	"b_idle",
	"b_srp_init",
	"b_peripheral",
	"b_wait_acon",
	"b_host",
	"a_idle",
	"a_wait_vrise",
	"a_wait_bcon",
	"a_host",
	"a_suspend",
	"a_peripheral",
	"a_wait_vfall",
	"a_vbus_err"
};

#ifdef DEBUG
#define dmsg(format, args...) printk(KERN_DEBUG "%s: " format "\n", \
			__FUNCTION__, ##args)
#else
#define dmsg(format, args...) do {} while (0)
#endif

#define DRIVER_DESC "USB OTG on Monahans"

extern void pxa3xx_udc_save_irq(void);
extern void pxa3xx_udc_restore_irq(void);
int mhnotg_state_change(void);
void mhnotg_require_bus(int status);
int otgc_dp_srp(struct monahans_otgc *pOtgHandle);
static int mhnotg_start_srp(struct otg_transceiver *otg);
static void otg_timer_basep_brst(unsigned long data);

static struct otg_transceiver *xceiv;

#undef PMIC_SRP_IGNORE_ONCE
#ifdef CONFIG_USB_OTG_PXA3XX_U2D
extern void reset_xcvr(void);
extern void ulpi_rtsm(void);
extern enum u2d_phy_mode ulpi_get_phymode(void);
extern int ulpi_set_phymode(enum u2d_phy_mode mode);
extern int ulpi_reg_read(u8 reg, u8 * value);
extern int ulpi_reg_write(u8 reg, u8 value);
#endif

/******************************************************************
 * Monahans OTG controller functions
 ******************************************************************
 * timer help functions
 */
#define TIMER_NUM	5
static struct timer_list timer_array[TIMER_NUM];
static unsigned timer_state = 0;

/* register a timer
 * parameters
 * pOtgHandle - OTG handle
 * interval - delay of timer
 * timer_callback - callback function when timer expired
 */
int otg_register_timer(struct monahans_otgc *pOtgHandle,
		       int interval, void (*timer_callback) (unsigned long))
{
	unsigned count, temp;
	int ret;
	struct timer_list *timer;

	/* find a free timer */
	ret = 0;
	count = TIMER_NUM;
	temp = timer_state;
	while ((temp & 0x1) && count) {
		temp = (temp >> 1);
		count--;
		ret++;
	}

	if (ret >= TIMER_NUM) {
		printk(KERN_ERR "No free timer\n");
		return -1;
	} else {
		timer = &timer_array[ret];
		timer_state |= (1 << ret);
	}

	init_timer(timer);
	timer->data = (unsigned long)pOtgHandle;
	timer->function = timer_callback;
	timer->expires = jiffies + interval;
	add_timer(timer);

	return ret;
}

int otg_cancel_timer(int timer_id)
{
	if ((timer_id < 0) || (timer_id > 4)
	    || !((1 << timer_id) & timer_state))
		return OTG_INVALID_PARAMETER;

	del_timer(&timer_array[timer_id]);
	timer_state &= ~(1 << timer_id);

	return 0;
}

/* Timer expired, call this functio to indicate the timer free
 */
int otg_timer_expired(int timer_id)
{
	if ((timer_id < 0) || (timer_id > 4)
	    || !((1 << timer_id) & timer_state))
		return OTG_INVALID_PARAMETER;

	timer_state &= ~(1 << timer_id);

	return 0;
}

/* platform independent functions */

#ifndef CONFIG_PXA310
/* Configures USB host port2 to the desired mode */
int otg_config_port2(int mode)
{
	u32 up2ocr = UP2OCR;
	int status = 0;

	up2ocr &= ~(7 << OTG_UP2OCR_SEOS_SHIFT);
	up2ocr &= ~(OTG_UP2OCR_HXS | OTG_UP2OCR_HXOE);
	up2ocr &= ~(OTG_UP2OCR_DPPUE | OTG_UP2OCR_DMPUE);
	up2ocr &= ~(OTG_UP2OCR_DMPDE | OTG_UP2OCR_DPPDE);
	switch (mode) {
		/* Both differential port and single-ended port are turned off */
	case USB_OTG_OFF:
		break;
		/* Differential Port is off, Single-Ended Port2 is Non-OTG USB Client */
	case USB_NON_OTG_CLIENT_SEP:
		up2ocr |= (2 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is off, Single-Ended Port2 is Non-OTG USB Host */
	case USB_NON_OTG_HOST_SEP:
		up2ocr |= (3 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is off, Single-Ended Port2 is interfaced to external OTG transceiver */
		/* and is directed into USB client controller */
	case USB_EXT_OTG_CLIENT_SEP:
		up2ocr |= (4 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is off, Single-Ended Port2 is interfaced to external OTG transceiver */
		/* and is directed into USB host controller */
	case USB_EXT_OTG_HOST_SEP:
		up2ocr |= (5 << OTG_UP2OCR_SEOS_SHIFT);
		break;
		/* Differential Port is Non-OTG USB client, Single-Ended Port 2 is Non-OTG USB host */
	case USB_NON_OTG_HOST_SEP_CLIENT_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= (3 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is Non-OTG USB host, Single-Ended Port 2 is Non-OTG USB client */
	case USB_NON_OTG_CLIENT_SEP_HOST_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= OTG_UP2OCR_HXS;
		up2ocr |= (2 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is USB OTG host, Single-Ended Port 2 is */
		/* interfaced to external charge pump */
	case USB_INT_OTG_HOST_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= OTG_UP2OCR_HXS;
		up2ocr |= (7 << OTG_UP2OCR_SEOS_SHIFT);
		up2ocr |= (OTG_UP2OCR_DPPDE | OTG_UP2OCR_DMPDE);
		break;

		/* Differential Port is USB OTG client, Single-Ended Port 2 is */
		/* interfaced to external charge pump */
	case USB_INT_OTG_CLIENT_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= (6 << OTG_UP2OCR_SEOS_SHIFT);
		up2ocr |= OTG_UP2OCR_DPPUE;
		break;

		/* Differential Port is Non-OTG USB Client, Single-Ended Port2 is off */
	case USB_NON_OTG_CLIENT_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		break;

		/* Differential Port is Non-OTG USB Host, Single-Ended Port2 is off */
	case USB_NON_OTG_HOST_DP:
		up2ocr |= (OTG_UP2OCR_HXOE | OTG_UP2OCR_HXS);
		break;

	default:
		status = OTG_INVALID_PARAMETER;
		break;
	}
	UP2OCR = up2ocr;

	return status;
}
#else

static void mhn_otg_mfp_set(enum mhn_otg_function mode)
{
	switch (mode) {

	case OTG_B_DEVICE:
		dmsg("========config the device to USB device here=========\n");
		break;

	case OTG_A_DEVICE:
		dmsg("========config the device to USB host here=========\n");
		break;

	default:
		printk(KERN_ALERT
		       "========config the device to ?????========\n");
		break;
	}
}

int otg_config_port2(int mode)
{
	int status = 0;

	switch (mode) {

	case USB_INT_OTG_CLIENT_DP:
		mhn_otg_mfp_set(OTG_B_DEVICE);
		ulpi_set_phymode(SYNCH);
		break;

	case USB_INT_OTG_HOST_DP:
		mhn_otg_mfp_set(OTG_A_DEVICE);
		ulpi_set_phymode(SER_6PIN);
		break;

	default:
		status = OTG_INVALID_PARAMETER;
		break;
	}

	return status;
}
#endif

/*
 * Set field b_bus_required to indicate B-Device hope to control bus.
 * parameters
 *	require - indicate require or release bus
 *	srp - indicate whethere srp is initiated
 */
int otg_require_bus(struct monahans_otgc *pOtgHandle, int require, int srp)
{
#ifdef PMIC_SRP_IGNORE_ONCE
	static int srp_ignore = 1;
#endif

	dmsg(" otg state %s, srp %d, require %d, bus_required %d\n",
	     state_string[pOtgHandle->state], srp, require,
	     pOtgHandle->b_bus_required);
	if (require && (pOtgHandle->b_bus_required == 0)) {
		if (!srp) {
			/* VBUS is high, host connected */
			pOtgHandle->b_bus_required = 1;
			if ((pOtgHandle->state == OTG_B_PERIPHERAL) &&
			    (require & USBOTG_VBUS_VALID)) {
				otg_config_port2(USB_INT_OTG_CLIENT_DP);
				usb_gadget_vbus_connect(xceiv->gadget);
			}

			if ((pOtgHandle->state == OTG_A_SUSPEND) &&
			    (require & (USBOTG_SRP_DETECT | USBOTG_A_REQUIRE)))
			{
#ifdef PMIC_SRP_IGNORE_ONCE
				/* for SRP_DETECT interrupt would happen
				   when disable vbus supply, ignore it */
				if (srp_ignore
				    && (USBOTG_SRP_DETECT == require)) {
					srp_ignore = 0;
					pOtgHandle->b_bus_required = 0;
					dmsg("srp_ignore !\n");
				} else {
					srp_ignore = 1;
#else
				{
#endif
					otgc_vbus_enable(pOtgHandle, 1, 0);
					pOtgHandle->state = OTG_A_HOST;
#ifdef CONFIG_USB_OTG_PXA3XX_U2D
					otg_config_port2(USB_INT_OTG_HOST_DP);
#endif
				}
			}

			/* FIXME, when exit D0CS could not set phy to
			 * serial mode for the first time, MUST re-connect
			 * the cable
			 */
			if ((pOtgHandle->state == OTG_A_WAIT_BCON) &&
			    (require & USBOTG_VBUS_VALID)) {
				dmsg("%s, state = a_wait_bcon", __func__);
				otg_config_port2(USB_INT_OTG_HOST_DP);
			}
		}

		/* If state is idle send SRP, tell host to wake up and power VBUS */
		if ((pOtgHandle->state == OTG_B_PERIPHERAL) && srp) {
			mhnotg_start_srp(xceiv);
		}
	}

	if (!require && (pOtgHandle->b_bus_required == 1)) {
		pOtgHandle->b_bus_required = 0;

		if (pOtgHandle->state == OTG_A_HOST) {
			otgc_vbus_enable(pOtgHandle, 0, 0);
			pOtgHandle->state = OTG_A_SUSPEND;
		}

		if (pOtgHandle->state == OTG_B_WAIT_ACON) {
			pOtgHandle->state = OTG_B_PERIPHERAL;
			mhnotg_state_change();

			otg_cancel_timer(pOtgHandle->b_ase0_brst_tmr);
			pOtgHandle->b_ase0_brst_tmr = -1;
		}

		/* VBUS low, set USB to SE0 state */
#ifdef	CONFIG_USB_OTG_PXA3XX
		otg_config_port2(USB_INT_OTG_HOST_DP);
#endif
		usb_gadget_disconnect(xceiv->gadget);
#ifdef CONFIG_USB_OTG_PXA3XX_U2D
		/* set PHY enter low power mode */
		ulpi_set_phymode(LOWPOWER);
		usb_gadget_vbus_disconnect(xceiv->gadget);
#endif
	}

	return 0;
}

/*
 *  Set or clear field a_set_b_hnp_en
 *  mode - the desired mode for USB host port2
 */
int otg_set_b_hnp(struct monahans_otgc *pOtgHandle, int set)
{
	if (set) {
		pOtgHandle->a_set_b_hnp_en = 1;
	} else {
		pOtgHandle->a_set_b_hnp_en = 0;
	}

	return 0;
}

/* called when timer a_aidl_bdis_tmr expires
 */
static void otg_timer_aidle_bdis(unsigned long data)
{
	struct monahans_otgc *pOtgHandle = (struct monahans_otgc *)data;
	if (OTG_A_SUSPEND != pOtgHandle->state) {
		return;
	}
	/* According to OTG specification, Vbus should be turn off
	 * and A device become idle. In our implement, SRP is not feasible
	 * A device switching to a_wait_bcon is better */

	dmsg(" set otg state to a_wait_bcon");
	pOtgHandle->state = OTG_A_WAIT_BCON;
	mhnotg_state_change();

	otg_timer_expired(pOtgHandle->a_aidl_bdis_tmr);
	pOtgHandle->a_aidl_bdis_tmr = -1;
}

/* called after Host suspend USB device
 */
int otg_suspend_device(struct monahans_otgc *pOtgHandle)
{
	int timer_id;

	if (OTG_A_HOST == pOtgHandle->state) {
		pOtgHandle->state = OTG_A_SUSPEND;
		if (pOtgHandle->a_set_b_hnp_en) {
			timer_id = otg_register_timer(pOtgHandle,
						      T_A_AIDL_BDIS,
						      otg_timer_aidle_bdis);
			if (timer_id < 0) {
				return OTG_TIMER_FAILED;
			}

			pOtgHandle->a_aidl_bdis_tmr = timer_id;
		}
	}

	return 0;
}

/* called when timer b_ase0_brst_tmr expires */
static void otg_timer_basep_brst(unsigned long data)
{
	struct monahans_otgc *pOtgHandle = (struct monahans_otgc *)data;

	dmsg(" otg state %s, UDCCR: 0x%x, b_bus_required: %d\n",
	     state_string[pOtgHandle->state], UDCCR,
	     pOtgHandle->b_bus_required);

	if (OTG_B_WAIT_ACON == pOtgHandle->state) {
		if (pOtgHandle->b_bus_required) {
			otg_config_port2(USB_INT_OTG_CLIENT_DP);
#ifdef CONFIG_USB_OTG_PXA3XX_U2D
			usb_gadget_vbus_connect(xceiv->gadget);
#endif
		}
		pOtgHandle->state = OTG_B_PERIPHERAL;
		mhnotg_state_change();
	}
	otg_timer_expired(pOtgHandle->b_ase0_brst_tmr);
	pOtgHandle->b_ase0_brst_tmr = -1;
}

/* Called by USB client driver when host suspends device */
int otg_respond_suspend(struct monahans_otgc *pOtgHandle)
{
	dmsg(" otg state %s, UDCCR %x, b_bus_required %d\n",
	     state_string[pOtgHandle->state], UDCCR,
	     pOtgHandle->b_bus_required);

	if (OTG_B_PERIPHERAL == pOtgHandle->state) {
#ifdef CONFIG_USB_OTG_PXA3XX_U2D
		if (U2DOTGCR_BHNP & U2DOTGCR) {
#else
		if (UDCCR_BHNP & UDCCR) {
#endif
			pOtgHandle->b_hnp_enabled = 1;
		} else
			return 0;

		if (pOtgHandle->b_bus_required) {
			otg_config_port2(USB_INT_OTG_HOST_DP);
			pOtgHandle->state = OTG_B_WAIT_ACON;
			/* Start timer Tb_ase0_brst_tmr */
			pOtgHandle->b_ase0_brst_tmr =
			    otg_register_timer(pOtgHandle, T_B_ASE0_BRST,
					       otg_timer_basep_brst);
		}
	}

	/* B-Device completes using the bus, A Device return to be host */
	if (OTG_A_PERIPHERAL == pOtgHandle->state) {
		otg_config_port2(USB_INT_OTG_HOST_DP);
		pOtgHandle->state = OTG_A_WAIT_BCON;
		pOtgHandle->a_set_b_hnp_en = 0;
	}

	return 0;
}

/* Called by USB host controller driver when a device connects
 */
int otg_respond_connect(struct monahans_otgc *pOtgHandle)
{
	dmsg("%s otg state %s\n", __func__, state_string[pOtgHandle->state]);

	if (OTG_A_WAIT_BCON == pOtgHandle->state) {
		otg_cancel_timer(pOtgHandle->a_wait_bcon_tmr);
		pOtgHandle->a_wait_bcon_tmr = -1;
		pOtgHandle->state = OTG_A_HOST;
	} else if (OTG_B_WAIT_ACON == pOtgHandle->state) {
		otg_cancel_timer(pOtgHandle->b_ase0_brst_tmr);
		pOtgHandle->b_ase0_brst_tmr = -1;
		pOtgHandle->state = OTG_B_HOST;
	} else
		return OTG_WRONG_STATE;

	return 0;
}

/* Called by USB host controller driver when a device disconnects
 */
int otg_respond_disconnect(struct monahans_otgc *pOtgHandle)
{
	dmsg(" otg state: %s, a_set_b_hnp_en: %d\n",
	     state_string[pOtgHandle->state], pOtgHandle->a_set_b_hnp_en);

	if (OTG_A_HOST == pOtgHandle->state) {
		pOtgHandle->state = OTG_A_WAIT_BCON;
		/* Start timer */
	}

	if (OTG_B_HOST == pOtgHandle->state) {
		otg_config_port2(USB_INT_OTG_CLIENT_DP);
		pOtgHandle->state = OTG_B_PERIPHERAL;
#ifdef	CONFIG_USB_OTG_PXA3XX_U2D
		usb_gadget_disconnect(xceiv->gadget);
		usb_gadget_vbus_disconnect(xceiv->gadget);
		usb_gadget_vbus_connect(xceiv->gadget);
		usb_gadget_connect(xceiv->gadget);
#endif
	}

	if ((OTG_A_SUSPEND == pOtgHandle->state) && pOtgHandle->a_set_b_hnp_en) {
		pOtgHandle->a_sus_ignored = 1;
		otg_config_port2(USB_INT_OTG_CLIENT_DP);
		pOtgHandle->state = OTG_A_PERIPHERAL;
		usb_gadget_vbus_connect(xceiv->gadget);
		otg_cancel_timer(pOtgHandle->a_aidl_bdis_tmr);
		pOtgHandle->a_aidl_bdis_tmr = -1;

	}
	return 0;
}

/* platform (zylonite) dependent functions
 *
 * detect IDON pin
 * When I debug, the following code don't work. The status bit only is set
 * after enable system interrupt, then a interrupt come out
 */
#ifndef CONFIG_PXA310
#define USB_OTGID_PIN	MFP_PIN_GPIO104
#else
#define USB_OTGID_PIN	MFP_PIN_GPIO106
#endif
static enum mhn_otg_function otgc_detect_default_func(struct monahans_otgc
						      *pOtgHandle)
{
#ifndef CONFIG_PXA310
	int value;

	/* detect the OTGID level */
	mhn_mfp_set_afds(USB_OTGID_PIN, 0, 0);
	mhn_gpio_set_direction(USB_OTGID_PIN, GPIO_DIR_IN);
	value = mhn_gpio_get_level(USB_OTGID_PIN);

	/* FIXME, set default function to B device */
	dmsg("OTGID level %d\n", value);
	return OTG_B_DEVICE;

	if (value == GPIO_LEVEL_HIGH)
		return OTG_B_DEVICE;
	else
		return OTG_A_DEVICE;
#else
	u8 tmp;

	/* enable OTG ID pullup register */
	if (!(ulpi_get_phymode() & (SYNCH | PRE_SYNCH)))
		ulpi_rtsm();

	if (ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_IDPULLUP))
		goto err;

	if (ulpi_reg_read(ULPI_INT_STATUS, &tmp))
		goto err;

	/* enable ID detect */
	if (tmp & ULPI_INT_IDGND)
		return OTG_B_DEVICE;
	else
		return OTG_A_DEVICE;

      err:
	reset_xcvr();
	return OTG_B_DEVICE;
#endif
}

/* called when initialize fields in handle
 */
static void otgc_init_handle(struct monahans_otgc *pOtgHandle)
{
	pOtgHandle->a_set_b_hnp_en = 0;
	pOtgHandle->b_hnp_enabled = 0;
	pOtgHandle->a_hnp_supported = 0;
	pOtgHandle->a_alt_hnp_supported = 0;
	pOtgHandle->a_sus_ignored = 0;

	/* Initialize timer identifiers */
	pOtgHandle->a_wait_vrise_tmr = -1;
	pOtgHandle->a_wait_bcon_tmr = -1;
	pOtgHandle->a_aidl_bdis_tmr = -1;
	pOtgHandle->b_ase0_brst_tmr = -1;
	pOtgHandle->b_srp_fail_tmr = -1;
	pOtgHandle->a_srp_rspns_tmr = -1;
}

extern void zylonite_enable_otg_pins(void);

int otgc_init(struct monahans_otgc *pOtgHandle)
{
	enum mhn_otg_function function;

#ifndef CONFIG_PXA310
	if (UDCCR & UDCCR_UDE) {
		UDCCR = UDCCR_UDE | UDCCR_OEN;
	} else
		return OTG_UDC_DISABLED;
#else
	u32 u2dotgcr;

	U2DOTGCR = U2DOTGCR_UTMID;
	u2dotgcr = U2DOTGCR;

	u2dotgcr |= U2DOTGCR_ULE;
	U2DOTGCR = u2dotgcr;

	u2dotgcr |= U2DOTGCR_ULAF;
	u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF);
	U2DOTGCR = u2dotgcr;
#endif

	/* Read the OTGID pin to detect whether there is a mini-A plug
	 * is alreadly inserted in receptacle.*/
	function = otgc_detect_default_func(pOtgHandle);
	pOtgHandle->default_function = function;

	zylonite_enable_otg_pins();

	otgc_init_handle(pOtgHandle);

#ifndef CONFIG_PXA310
	UP2OCR |= OTG_UP2OCR_IDON;
	UDCOTGICR |= (OTG_SETFEATURE | UDCOTGISR_IRIDR | UDCOTGISR_IRIDF);
#else
	/* enable the OTG ID rising/falling interrupt */
	U2DOTGICR = U2DOTGINT_DEFAULT;
	U2DOTGCR |= U2DOTGCR_OTGEN;
#endif

#ifdef MHN_OTG_VBUS_PMIC
	mhn_pmic_set_pump(1);
#endif

	if (OTG_A_DEVICE == function) {
		otg_config_port2(USB_INT_OTG_HOST_DP);

#ifdef MHN_OTG_VBUS_PMIC
		mhn_pmic_set_usbotg_a_mask();
#endif

		otgc_vbus_enable(pOtgHandle, 1, 0);

		pOtgHandle->state = OTG_A_WAIT_BCON;
	} else if (OTG_B_DEVICE == function) {
		otg_config_port2(USB_INT_OTG_CLIENT_DP);
		pOtgHandle->state = OTG_B_PERIPHERAL;

#ifdef MHN_OTG_VBUS_PMIC
		mhn_pmic_set_usbotg_b_mask();
#endif
		otgc_vbus_enable(pOtgHandle, 0, 0);
	}

	return 0;
}

/* Enable or disable USB Vbus
 */
int otgc_vbus_enable(struct monahans_otgc *pOtgHandle, int enable, int srp)
{
	int status;

	dmsg("enable %d srp %d\n", enable, srp);
	status = mhn_pmic_set_vbus_supply(enable, srp);
	if (status)
		return OTG_I2C_ERROR;
	return 0;
}

/* Called to start data-line SRP
 */
int otgc_dp_srp(struct monahans_otgc *pOtgHandle)
{
#ifndef CONFIG_USB_OTG_PXA3XX_U2D
	int up2ocr;
	/* Enable D+ pull-up resister to enable Data line SRP */
	up2ocr = UP2OCR;
	up2ocr &= ~OTG_UP2OCR_DPPDE;
	up2ocr |= OTG_UP2OCR_DPPUE;
	UP2OCR = up2ocr;
	mdelay(T_B_DATA_PLS);

	/* Remove D+ pull-up resister to stop data line SRP */
	up2ocr = UP2OCR;
	up2ocr &= ~OTG_UP2OCR_DPPUE;
	up2ocr |= OTG_UP2OCR_DPPDE;
	UP2OCR = up2ocr;
#else
	enum u2d_phy_mode mode = ulpi_get_phymode();
	char func_ctrl, otg_ctrl;

	ulpi_rtsm();
	ulpi_reg_read(ULPI_FUNCTION_CONTROL, &func_ctrl);
	ulpi_reg_read(ULPI_OTG_CONTROL, &otg_ctrl);

	/* Enable D+ pull-up resister to enable Data line SRP */
	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DPPULLDOWN);
	ulpi_reg_write(ULPI_FUNCTION_CONTROL_CLEAR, 0x45);

	mdelay(T_B_DATA_PLS);

	/* restore values to stop Data line SRP */
	ulpi_reg_write(ULPI_OTG_CONTROL, otg_ctrl);
	ulpi_reg_write(ULPI_FUNCTION_CONTROL, func_ctrl);
	ulpi_set_phymode(mode);
#endif
	return 0;
}

/* Called to start SRP
 */
int otgc_vbus_srp(struct monahans_otgc *pOtgHandle)
{
#ifdef MHN_OTG_VBUS_PMIC

	mhn_pmic_set_vbus_supply(1, 1);
	mdelay(T_B_SRP_INIT - T_B_DATA_PLS);
	mhn_pmic_set_vbus_supply(0, 0);
#else
	ulpi_rtsm();
	/* start VBUS pulse SRP */
	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DISCHRGVBUS);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_CHRGVBUS);

	/* data line SRP */
	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DPPULLDOWN);
	mdelay(T_B_DATA_PLS);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);

	/* stop VBUS pulse SRP */
	mdelay(T_B_SRP_INIT - T_B_DATA_PLS);
	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_CHRGVBUS);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DISCHRGVBUS);

#endif
	return 0;
}

#ifdef SRP_TIMER
/* called when timer a_srp_rspns_tmr expires
 */
static void otg_timer_asrp_rspns(unsigned long data)
{
	struct monahans_otgc *pOtgHandle = (struct monahans_otgc *)data;
	if (OTG_A_SUSPEND != pOtgHandle->state) {
		return;
	}
	/* turn on vbus to start a new session if SRP is initializeded
	 *  by B device */
	dmsg(" require_bus ");
	mhnotg_require_bus(1);
	otg_cancel_timer(pOtgHandle->a_srp_rspns_tmr);
	pOtgHandle->a_srp_rspns_tmr = -1;
}
#endif

/* This function is called when USB OTG interrupt happened
 */
int otgc_interrupt_handle(struct monahans_otgc *pOtgHandle)
{
#ifndef CONFIG_PXA310
	u32 otgisr;

	otgisr = status_reg_UDCOTGISR;
	dmsg(" otg state %s, otgisr %x\n", state_string[pOtgHandle->state],
	     otgisr);
	/* OTG SET FEATURE received */
	if (otgisr & OTG_SETFEATURE) {
		if (UDCCR & UDCCR_BHNP)
			pOtgHandle->b_hnp_enabled = 1;

		if (UDCCR & UDCCR_AHNP)
			pOtgHandle->a_hnp_supported = 1;

		if (UDCCR & UDCCR_AALTHNP)
			pOtgHandle->a_alt_hnp_supported = 1;

	}

	/* mini-A is plugged in */
	if (otgisr & UDCOTGISR_IRIDF) {
		if (OTG_B_DEVICE == pOtgHandle->default_function) {
			otgc_init_handle(pOtgHandle);
			pOtgHandle->default_function = OTG_A_DEVICE;
			otg_config_port2(USB_INT_OTG_HOST_DP);

			/* Mask some interrupts we don't care, */
			mhn_pmic_set_usbotg_a_mask();

			/* Enable USB PUMP */
			otgc_vbus_enable(pOtgHandle, 1, 0);
			pOtgHandle->state = OTG_A_WAIT_BCON;
			pOtgHandle->a_sus_ignored = 1;

			/* The following two line code is need to work around
			 * USB ID pin sighting */
			return 0;
		}
	}
	/* mini-A is plugged out */
	if (otgisr & UDCOTGISR_IRIDR) {
		dmsg("mini-a plug out");
		if (OTG_A_DEVICE == pOtgHandle->default_function) {
			/* Disable usb pump */
			dmsg("disable vbus");
			otgc_vbus_enable(pOtgHandle, 0, 0);

			otgc_init_handle(pOtgHandle);
			pOtgHandle->default_function = OTG_B_DEVICE;
			/* pOtgHandle->state = OTG_B_IDLE; */
			pOtgHandle->state = OTG_B_PERIPHERAL;
			otg_config_port2(USB_INT_OTG_CLIENT_DP);

			/* Mask some interrupts we don't care,
			 * we only care session valid interrupt*/
			mhn_pmic_set_usbotg_b_mask();
		}
	}

	return 0;
#else
	u32 otgisr;
	enum u2d_phy_mode state = ulpi_get_phymode();
#ifdef SRP_TIMER
	int timer_id;
#endif

	dmsg("otgicr %x otgisr %x otgusr %x\n",
	     U2DOTGICR, status_reg_U2DOTGISR, U2DOTGUSR);
	otgisr = status_reg_U2DOTGISR;

	if ((state != SYNCH) && (state != PRE_SYNCH)) {
		dmsg("state %d -> synch\n", state);
		ulpi_rtsm();
	}

	dmsg("\tin synch mode: 0x%08X, USR 0x%08X otg state %s",
	     status_reg_U2DOTGISR, U2DOTGUSR, state_string[pOtgHandle->state]);

	/* OTG SET FEATURE received */
	if (otgisr & U2DOTGINT_SF) {
		if (U2DOTGCR & U2DOTGCR_BHNP)
			pOtgHandle->b_hnp_enabled = 1;

		if (U2DOTGCR & U2DOTGCR_AHNP)
			pOtgHandle->a_hnp_supported = 1;

		if (U2DOTGCR & U2DOTGCR_AALTHNP)
			pOtgHandle->a_alt_hnp_supported = 1;
	}
#ifndef	CABLE_DETECT_GPIO
	if (otgisr & (U2DOTGINT_RVV | U2DOTGINT_FVV)) {
		extern void cable_detect_interrupt(void);
		dmsg("ogtisr: 0x%x, otgusr: 0x%x\n", otgisr, U2DOTGUSR);
		cable_detect_interrupt();
		if ((otgisr & U2DOTGINT_RVV) && !(otgisr & U2DOTGINT_FVV) &&
		    (pOtgHandle->default_function != OTG_A_DEVICE))
			state = SYNCH;
	}
#endif

	if (otgisr & U2DOTGINT_RSV) {
		if ((pOtgHandle->state == OTG_A_SUSPEND) &&
		    !(pOtgHandle->a_set_b_hnp_en)) {
			mhnotg_require_bus(USBOTG_SRP_DETECT);
			state = SER_6PIN;
#ifdef SRP_TIMER
			timer_id = otg_register_timer(pOtgHandle,
						      T_A_SRP_RSPNS,
						      otg_timer_asrp_rspns);
			if (timer_id < 0) {
				return OTG_TIMER_FAILED;
			}

			pOtgHandle->a_srp_rspns_tmr = timer_id;
#endif
		} else if ((pOtgHandle->state == OTG_B_PERIPHERAL) ||
			   (pOtgHandle->state == OTG_A_PERIPHERAL)) {
			/* update the state */
			state = PRE_SYNCH;
		} else
			state = SER_6PIN;

		U2DOTGICR &= ~U2DOTGINT_RSV;
	}

	/* mini-A is plugged in */
	if (otgisr & U2DOTGINT_FID) {
		/* check the current of OTG ID from the PHY */
		if (U2DOTGUSR & U2DOTGUSR_ID) {
			goto out1;
		}

		dmsg(" OTGID low, Mini-A plug in, otgusr %08X", U2DOTGUSR);
		if (OTG_B_DEVICE == pOtgHandle->default_function) {
			otgc_init_handle(pOtgHandle);
			pOtgHandle->default_function = OTG_A_DEVICE;
			/* disable the device controller */
			usb_gadget_vbus_disconnect(xceiv->gadget);
			otg_config_port2(USB_INT_OTG_HOST_DP);

#ifdef MHN_OTG_VBUS_PMIC
			mhn_pmic_set_usbotg_a_mask();
#endif

			/* Enable USB PUMP */
			otgc_vbus_enable(pOtgHandle, 1, 0);
			pOtgHandle->state = OTG_A_WAIT_BCON;
			pOtgHandle->a_sus_ignored = 1;

			/* update the state */
			state = SER_6PIN;
		}
	}
      out1:
	/* mini-A is plugged out */
	if (otgisr & U2DOTGINT_RID) {

		/* check the current of OTG ID from the PHY */
		if (!(U2DOTGUSR & U2DOTGUSR_ID)) {
			goto out2;
		}

		dmsg(" OTGID high, Mini-A plug out, otgusr %08X", U2DOTGUSR);
		if (OTG_A_DEVICE == pOtgHandle->default_function) {
			/* Disable usb pump */
			otgc_vbus_enable(pOtgHandle, 0, 0);

			otgc_init_handle(pOtgHandle);
			pOtgHandle->default_function = OTG_B_DEVICE;
			/* pOtgHandle->state = OTG_B_IDLE; */
			pOtgHandle->state = OTG_B_PERIPHERAL;
			otg_config_port2(USB_INT_OTG_CLIENT_DP);

#ifdef MHN_OTG_VBUS_PMIC
			mhn_pmic_set_usbotg_b_mask();
#endif
			/* update the state */
			if (U2DOTGUSR & U2DOTGUSR_VV)
				state = SYNCH;
			else
				state = LOWPOWER;
		}
	}

      out2:

	if ((state != SYNCH) && (state != PRE_SYNCH)) {
		dmsg("synch -> state %d otg state %s\n ", state,
		     state_string[pOtgHandle->state]);
		ulpi_set_phymode(state);
	}

	return 0;
#endif
}

/* end of Monahans OTG controller functions */

struct otg_transceiver *otg_get_transceiver(void)
{
	if (xceiv)
		get_device(xceiv->dev);

	return xceiv;
}

int otg_set_transceiver(struct otg_transceiver *x)
{
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;

	return 0;
}

struct monahans_otg {
	struct otg_transceiver otg;
	struct usb_device *udev;
	struct monahans_otgc *otg_ctrl;
};

#ifdef MHN_OTG_VBUS_PMIC
/* Called from pmic interrupt */
void mhnotg_pmic_interrupt(unsigned char eventb)
{
	struct monahans_otg *mhn_otg;

	dmsg("event b:0x%02x", eventb);
	if (xceiv) {
		mhn_otg = container_of(xceiv, struct monahans_otg, otg);
	}
}
#endif

/*  When USB cable attached, call this function*/
void mhnotg_require_bus(int status)
{
	struct monahans_otg *mhn_otg;

	if (xceiv) {
		mhn_otg = container_of(xceiv, struct monahans_otg, otg);
		otg_require_bus(mhn_otg->otg_ctrl, status, 0);

		dmsg(" from state %s to state %s\n",
		     state_string[mhn_otg->otg.state],
		     state_string[mhn_otg->otg_ctrl->state]);
		mhn_otg->otg.state = mhn_otg->otg_ctrl->state;

	}
}

char *mhnotg_state(struct otg_transceiver *otg)
{
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);

	return (state_string[mhn_otg->otg.state]);
}

/* Called from monahans udc interrupt handler to deal with otg interrupt*/
int mhnotg_otg_interrupt(struct otg_transceiver *otg)
{
	int old_default;
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);

	dmsg("is called");
	old_default = mhn_otg->otg.default_a;
	otgc_interrupt_handle(mhn_otg->otg_ctrl);
	mhn_otg->otg.state = mhn_otg->otg_ctrl->state;
	mhn_otg->otg.default_a = mhn_otg->otg_ctrl->default_function;

	/* This eradicate unnecessary suspend & reset interrupt */
	if (!old_default && mhn_otg->otg.default_a) {
#ifndef CONFIG_USB_GADGET_PXA3XX_U2D
		pxa3xx_udc_save_irq();
#endif
		mhn_otg->udev = NULL;
		mhn_otg->otg.host->is_b_host = 0;
	} else if (old_default && !mhn_otg->otg.default_a) {
#ifndef CONFIG_USB_GADGET_PXA3XX_U2D
		pxa3xx_udc_restore_irq();
#endif
		mhn_otg->udev = NULL;
	}

	return 0;
}

/* Called when otg has changed it status */
int mhnotg_state_change(void)
{
	struct monahans_otg *mhn_otg;

	if (xceiv) {
		mhn_otg = container_of(xceiv, struct monahans_otg, otg);
		mhn_otg->otg.state = mhn_otg->otg_ctrl->state;
#ifndef CONFIG_USB_GADGET_PXA3XX_U2D
		if (OTG_STATE_B_PERIPHERAL == mhn_otg->otg.state)
			pxa3xx_udc_restore_irq();
#endif
	}
	return 0;
}

static int mhnotg_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);

	mhn_otg->otg.host = host;

	return 0;
}

static int mhnotg_set_peripheral(struct otg_transceiver *otg,
				 struct usb_gadget *gadget)
{
	unsigned long flags;
	int ret;
	struct monahans_otgc *otg_ctrl;
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);

	if (gadget == NULL) {
#ifndef CONFIG_PXA310
		UDCOTGICR &=
		    ~(OTG_SETFEATURE | UDCOTGISR_IRIDR | UDCOTGISR_IRIDF);
		UP2OCR &= ~(OTG_UP2OCR_IDON | (0x7 << OTG_UP2OCR_SEOS_SHIFT));
		mhn_otg->otg_ctrl->b_bus_required = 0;
#else
		/* what if no gadget driver exists, OTG function is disabled as above????? */
		U2DOTGICR = 0;
		U2DOTGISR = U2DOTGISR;
		mhn_otg->otg_ctrl->b_bus_required = 0;
#endif
		mhn_otg->otg.gadget = NULL;
	} else {
		if (mhn_otg->otg.gadget) {
			return 0;
		}
		mhn_otg->otg.gadget = gadget;
		otg_ctrl = mhn_otg->otg_ctrl;
		local_irq_save_nort(flags);
		spin_lock_rt(&otg_ctrl->lock);
		if ((ret = otgc_init(mhn_otg->otg_ctrl))) {
			printk(KERN_ERR "%s: failed to call otgc_init:%d\n",
			       __FUNCTION__, ret);
			local_irq_restore_nort(flags);
			spin_unlock_rt(&otg_ctrl->lock);
			return -EFAULT;
		}

		mhn_otg->otg.state = mhn_otg->otg_ctrl->state;
		mhn_otg->otg.default_a = mhn_otg->otg_ctrl->default_function;
		local_irq_restore_nort(flags);
		spin_unlock_rt(&otg_ctrl->lock);
	}
	return 0;
}

static int mhnotg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	return 0;
}

static int mhnotg_start_srp(struct otg_transceiver *otg)
{
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);
	otgc_dp_srp(mhn_otg->otg_ctrl);
	otgc_vbus_srp(mhn_otg->otg_ctrl);

	return 0;
}

static int mhnotg_start_hnp(struct otg_transceiver *otg)
{
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);

	otg_set_b_hnp(mhn_otg->otg_ctrl, 1);
	otg_suspend_device(mhn_otg->otg_ctrl);

	return 0;
}

int mhnotg_host_suspend(struct otg_transceiver *otg)
{
	struct monahans_otg *mhn_otg = container_of(otg,
						    struct monahans_otg, otg);

	dmsg("otg state before:%s, a_sus_ignored %d\n",
	     state_string[mhn_otg->otg.state],
	     mhn_otg->otg_ctrl->a_sus_ignored);

	if (otg->default_a && mhn_otg->otg_ctrl->a_sus_ignored) {
		mhn_otg->otg_ctrl->a_sus_ignored = 0;
		return 0;
	}

	otg_respond_suspend(mhn_otg->otg_ctrl);
	mhn_otg->otg.state = mhn_otg->otg_ctrl->state;
	if (mhn_otg->otg_ctrl->state == OTG_B_WAIT_ACON) {
		mhn_otg->otg.host->is_b_host = 1;
		mhn_otg->otg.gadget->is_a_peripheral = 0;
	}
#ifndef CONFIG_USB_GADGET_PXA3XX_U2D
	if ((OTG_STATE_B_WAIT_ACON == mhn_otg->otg.state) ||
	    (OTG_STATE_A_WAIT_BCON == mhn_otg->otg.state))
		pxa3xx_udc_save_irq();
#endif
	dmsg("otg state after:%s\n", state_string[mhn_otg->otg.state]);
	return 0;
}

#ifdef CONFIG_PROC_FS
static const char *proc_node_name = "driver/otg";

static int otg_proc_read(char *page, char **start, off_t off, int count,
			 int *eof, void *_dev)
{
	struct monahans_otg *mhn_otg = _dev;
	char *next = page;
	unsigned size = count;
	unsigned long flags;
	int t;
	u8 value;

	if (off != 0)
		return 0;

	local_irq_save_nort(flags);
	spin_lock_rt(&mhn_otg->otg_ctrl->lock);

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n");
	size -= t;
	next += t;

	t = scnprintf(next, size, "otg state:%s bus required:%d\n",
		      state_string[mhn_otg->otg.state],
		      mhn_otg->otg_ctrl->b_bus_required);
	size -= t;
	next += t;

#ifdef MHN_OTG_VBUS_PMIC
#ifdef CONFIG_PXA3XX_ARAVA
	mhn_pmic_read(ARAVA_IRQ_MASK_B, &value);
	t = scnprintf(next, size, "Arava regs:\nevent mask B:0x%02x\n", value);
	size -= t;
	next += t;

	mhn_pmic_read(ARAVA_USBPUMP, &value);
	t = scnprintf(next, size, "USB pump:0x%02x\n", value);
	size -= t;
	next += t;

	mhn_pmic_read(ARAVA_MISCB, &value);
	t = scnprintf(next, size, "Misc control reg:0x%02x\n", value);
	size -= t;
	next += t;

	mhn_pmic_read(ARAVA_STATUS, &value);
	t = scnprintf(next, size, "Status reg:0x%02x\n", value);
	size -= t;
	next += t;

	mhn_pmic_read(ARAVA_EVENT_A, &value);
	t = scnprintf(next, size, "event a:0x%02x\n", value);
	size -= t;
	next += t;
	mhn_pmic_read(ARAVA_EVENT_B, &value);
	t = scnprintf(next, size, "event b:0x%02x\n", value);
	size -= t;
	next += t;
	mhn_pmic_read(ARAVA_EVENT_C, &value);
	t = scnprintf(next, size, "event c:0x%02x\n", value);
	size -= t;
	next += t;

#endif
#endif

	local_irq_restore_nort(flags);
	spin_unlock_rt(&mhn_otg->otg_ctrl->lock);
	*eof = 1;
	return count - size;
}

extern void dump_ulpi_regs(void);

static int otg_proc_write(struct file *filp, const char *buffer,
			  unsigned long count, void *data)
{
	char kbuf[8];
	int index;
	struct monahans_otg *mhn_otg = data;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
		/* Device expects to use the bus */
	case 1:
		otg_require_bus(mhn_otg->otg_ctrl, 1, 1);
		break;

		/* After complete use usb device. Suspend the device connected */
	case 2:
		if (mhn_otg->udev) {
			mhn_pmic_set_usbotg_b_mask();
			otg_require_bus(mhn_otg->otg_ctrl, 0, 0);
			mhn_otg->otg.state = mhn_otg->otg_ctrl->state;
			usb_suspend_device(mhn_otg->udev, PM_SUSPEND);
			msleep(500);
			mhn_pmic_set_usbotg_a_mask();
#ifdef CONFIG_USB_OTG_PXA3XX_U2D
			U2DOTGICR |= U2DOTGINT_RSV;
#endif
			printk
			    ("Session end. A/B device could restart the session after 5 seconds\n");
		} else {
			dmsg("udev is NULL");
		}
		break;

	case 3:
		break;
	case 4:
		if (mhn_otg->otg.state == OTG_A_SUSPEND)
			mhnotg_require_bus(USBOTG_A_REQUIRE);
		break;

	default:
		return -EINVAL;
	}
	return count;
}

#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry(proc_node_name, 0, NULL);\
		if (ent) { \
			ent->data = mhn_otg; \
			ent->read_proc = otg_proc_read; \
			ent->write_proc = otg_proc_write; \
		} \
	}while(0);
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)
#else
#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)
#endif

static int mhn_otg_probe(struct device *dev)
{
	struct monahans_otg *mhn_otg;

	mhn_otg = kmalloc(sizeof(struct monahans_otg), GFP_KERNEL);
	if (!mhn_otg) {
		printk(KERN_ERR "%s: failed to allocate memory!\n",
		       __FUNCTION__);
		return -ENOMEM;
	}
	memset(mhn_otg, 0, sizeof(struct monahans_otg));
	/* Initialize filed of otg */
	mhn_otg->otg.dev = dev;
	mhn_otg->otg.label = "pxa3xx-otg";
	mhn_otg->otg.set_host = mhnotg_set_host;
	mhn_otg->otg.set_peripheral = mhnotg_set_peripheral;
	mhn_otg->otg.set_power = mhnotg_set_power;
	mhn_otg->otg.start_srp = mhnotg_start_srp;
	mhn_otg->otg.start_hnp = mhnotg_start_hnp;
	mhn_otg->otg.state = OTG_STATE_UNDEFINED;
	mhn_otg->udev = NULL;

	/* Allocate & initialize field of otg_ctrl */
	mhn_otg->otg_ctrl = kmalloc(sizeof(struct monahans_otgc), GFP_KERNEL);
	if (!mhn_otg->otg_ctrl) {
		printk(KERN_ERR "%s: failed to allocate memory for otg_ctrl!\n",
		       __FUNCTION__);
		kfree(mhn_otg);
		return -ENOMEM;
	}
	memset(mhn_otg->otg_ctrl, 0, sizeof(struct monahans_otgc));

#ifdef CONFIG_PREEMPT_RT
	spin_lock_init(&mhn_otg->otg_ctrl->lock);
#endif
	otg_set_transceiver(&mhn_otg->otg);
	dev_set_drvdata(dev, mhn_otg);

	create_proc_files();
	return 0;
}

static int mhn_otg_remove(struct device *dev)
{
	struct monahans_otg *mhn_otg = dev_get_drvdata(dev);

	remove_proc_files();
	kfree(mhn_otg->otg_ctrl);
	kfree(mhn_otg);

	return 0;
}

#ifdef CONFIG_PM
static int mhn_otg_suspend(struct device *dev, u32 state, u32 level)
{
	return 0;
}

static int mhn_otg_resume(struct device *dev, u32 level)
{
	return 0;
}
#endif

static struct device_driver monahans_otg_driver = {
	.name = "PXA3xx-otg",
	.bus = &platform_bus_type,
	.probe = mhn_otg_probe,
	.remove = mhn_otg_remove,
#ifdef CONFIG_PM
	.suspend = mhn_otg_suspend,
	.resume = mhn_otg_resume,
#endif
};

static int __init monahans_otg_init(void)
{
	return driver_register(&monahans_otg_driver);
}

static void __exit monahans_otg_exit(void)
{
	driver_unregister(&monahans_otg_driver);
}

module_init(monahans_otg_init);
module_exit(monahans_otg_exit);
MODULE_LICENSE("GPL");
