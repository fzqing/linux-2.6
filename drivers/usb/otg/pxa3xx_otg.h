/*
 * linux/drivers/usb/gadget/pxa3xx_otg.h
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
#ifndef	__MONAHANS_USB_OTG_CONTROLLER__
#define	__MONAHANS_USB_OTG_CONTROLLER__

#include <linux/types.h>

enum mhn_otg_function {
	OTG_B_DEVICE = 0,
	OTG_A_DEVICE
};

enum mnh_otg_state {
	OTG_UNDEFINED = 0,
	/* B device state */
	OTG_B_IDLE,
	OTG_B_SRP_INIT,
	OTG_B_PERIPHERAL,
	OTG_B_WAIT_ACON,
	OTG_B_HOST,
	/* A device state */
	OTG_A_IDLE,
	OTG_A_WAIT_VRISE,
	OTG_A_WAIT_BCON,
	OTG_A_HOST,
	OTG_A_SUSPEND,
	OTG_A_PERIPHERAL,
	OTG_A_WAIT_VFALL,
	OTG_A_VBUS_ERR,
};

/* OTG working mode */
#define	USB_OTG_OFF		0
#define	USB_NON_OTG_CLIENT_SEP	1
#define	USB_NON_OTG_HOST_SEP	2
#define	USB_EXT_OTG_CLIENT_SEP	3
#define	USB_EXT_OTG_HOST_SEP	4
#define	USB_NON_OTG_HOST_SEP_CLIENT_DP	5
#define	USB_NON_OTG_CLIENT_SEP_HOST_DP	6
#define	USB_INT_OTG_HOST_DP	7
#define	USB_INT_OTG_CLIENT_DP	8
#define	USB_NON_OTG_CLIENT_DP	9
#define	USB_NON_OTG_HOST_DP	10

/* UDC Handle Structure */
struct monahans_otgc {
	enum mhn_otg_function default_function;
	enum mnh_otg_state state;
	int a_set_b_hnp_en;	/* A-Device set b_hnp_en */
	int b_hnp_enabled;	/* Set feature command with hnp enable received. */
	int a_hnp_supported;
	int a_alt_hnp_supported;
	int b_bus_required;	/* B-Device Require Bus */
	/* In order to avoid an unexpected suspend after A
	 * device become client, ignore the suspend signal.
	 */
	int a_sus_ignored;

	/* Timer identifiers */
	int a_wait_vrise_tmr;	/* Identifier of timer */
	int a_wait_bcon_tmr;
	int a_aidl_bdis_tmr;
	int b_ase0_brst_tmr;
	int b_srp_fail_tmr;
	int a_srp_rspns_tmr;
	spinlock_t lock;
};

/* OTG software error code */
#define	OTG_INVALID_PARAMETER	-1
#define	OTG_UDC_DISABLED	-2
#define	OTG_WRONG_STATE		-3
#define	OTG_WRONG_RESISTER	-4
#define	OTG_I2C_ERROR		-5
#define	OTG_MFP_FAILED		-6
#define	OTG_TIMER_FAILED	-7

/* fields  and bits for UP2OCR */
#define OTG_UP2OCR_CPVEN	( 1u << 0 )
#define OTG_UP2OCR_CPVEP	( 1u << 1 )
#define OTG_UP2OCR_DPPDE	( 1u << 2 )
#define OTG_UP2OCR_DMPDE	( 1u << 3 )
#define OTG_UP2OCR_DPPUE	( 1u << 4 )
#define OTG_UP2OCR_DMPUE	( 1u << 5 )
#define OTG_UP2OCR_DPPUBE	( 1u << 6 )
#define OTG_UP2OCR_DMPUBE	( 1u << 7 )
#define OTG_UP2OCR_EXSP		( 1u << 8 )
#define OTG_UP2OCR_EXSUS	( 1u << 9 )
#define OTG_UP2OCR_IDON		( 1u << 10)
#define OTG_UP2OCR_HXS		( 1u << 16)
#define OTG_UP2OCR_HXOE		( 1u << 17)
#define OTG_UP2OCR_SEOS_SHIFT	( 24)

/* Timer's interval, unit ms */
#define	T_A_WAIT_VRISE		100
#define T_A_WAIT_BCON		2000
#define T_A_AIDL_BDIS		1000
/* In order to debug, we use a longer timer */
#define T_B_ASE0_BRST		400
#define T_B_SRP_FAIL		30000
#define T_B_DATA_PLS		10
#define T_B_SRP_INIT		100
#define T_A_SRP_RSPNS		10

/* USBOTG EVENTS for require bus */
#define USBOTG_VBUS_VALID	(1<<0)
#define USBOTG_SRP_DETECT	(1<<1)
#define USBOTG_A_REQUIRE	(1<<2)

#define USBOTG_STATUS_REQUIRED	(USBOTG_VBUS_VALID | USBOTG_SRP_DETECT)

/* platform independent functions */
int otg_config_port2(int mode);
int otg_require_bus(struct monahans_otgc *pOtgHandle, int require, int srp);
int otg_set_b_hnp(struct monahans_otgc *pOtgHandle, int set);
int otg_suspend_device(struct monahans_otgc *pOtgHandle);
int otg_respond_suspend(struct monahans_otgc *pOtgHandle);
int otg_respond_connect(struct monahans_otgc *pOtgHandle);
int otg_respond_disconnect(struct monahans_otgc *pOtgHandle);

/* platform specific functions */
int otgc_init(struct monahans_otgc *pOtgHandle);
int otgc_interrupt_handle(struct monahans_otgc *pOtgHandle);
int otgc_vbus_enable(struct monahans_otgc *pOtgHandle, int enable, int srp);
int otgc_vbus_srp(struct monahans_otgc *pOtgHandle);

/* helper functions */
int otg_register_timer(struct monahans_otgc *pOtgHandle,
		       int interval, void (*timer_callback) (unsigned long));
int otg_cancel_timer(int timer_id);
int otg_timer_expired(int id);

/* platform dependent definitions */
/* Define bits in arava register event_b */
#define EVENT_B_SRP_DETECT	( 1 << 6 )
#define EVENT_B_SESSION_VALID	( 1 << 5 )
#define EVENT_B_VBUS_4P0	( 1 << 4 )
#define EVENT_B_VBUS_4P4	( 1 << 3 )

/* Define bits in arava register USBPUMP */
#define USBPUMP_EN_USBVEP	( 1 << 7 )
#define USBPUMP_EN_USBVE	( 1 << 6 )
#define USBPUMP_SRP_DETECT	( 1 << 5 )
#define USBPUMP_SESSION_VALID	( 1 << 4 )
#define USBPUMP_VBUS_VALID_4P0	( 1 << 3 )
#define USBPUMP_VBUS_VALID_4P4	( 1 << 2 )
#define USBPUMP_USBVEP		( 1 << 1 )
#define USBPUMP_USBVE		( 1 << 0 )

/* Define bits in arava register MISC B */
#define MISCB_SESSION_VALID_ENABLE	( 1 << 3 )
#define MISCB_USB_INT_RISING		( 1 << 2 )

#endif
