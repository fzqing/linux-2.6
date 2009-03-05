/*
 * linux/drivers/usb/gadget/pxa2xx_udc.h
 * Intel PXA2xx on-chip full speed USB device controller
 *
 * Copyright (C) 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2003 David Brownell
 *
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

#ifndef __LINUX_USB_GADGET_PXA27X_H
#define __LINUX_USB_GADGET_PXA27X_H

#include <linux/types.h>

enum {
	USB_ZERO_GADGET,
	USB_ETH_GADGET,
	USB_GFS_GADGET,
	USB_FSTRG_GADGET,
	USB_SERIAL_GADGET
};

struct pxa27x_udc;

struct pxa27x_ep {
	int phys;
	/*struct list_head conf_list; */
	struct usb_ep ep;
	struct pxa27x_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	unsigned long pio_irqs;
	unsigned long dma_irqs;
	short dma;

	unsigned short fifo_size;
	u8 bEndpointAddress;
	u8 bmAttributes;

	unsigned stopped:1;
	unsigned dma_fixup:1;

	/* UDCCSR = UDC Control/Status for this EP
	 * UDCBCR = UDC Byte Count Remaining (contents of OUT fifo)
	 * UDCDR  = UDC Endpoint Data Register (the fifo)
	 * DRCMR  = DMA Request Channel Map
	 */
	volatile u32 *reg_udccsr;
	volatile u32 *reg_udcbcr;
	volatile u32 *reg_udcdr;
	volatile u8 *reg_pudcdr;
#ifdef CONFIG_USB_PXA27X_DMA
#define DRCMR_EP0 24
	volatile u32 *reg_drcmr;
#endif
};

struct pxa27x_request {
	struct usb_request req;
	struct list_head queue;
	/*unsigned  dma_bytes; */
	unsigned mapped:1;

};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

#define EP0_FIFO_SIZE	((unsigned)16)
#define BULK_FIFO_SIZE	((unsigned)64)
#define ISO_FIFO_SIZE	((unsigned)256)
#define INT_FIFO_SIZE	((unsigned)8)

struct udc_stats {
	struct ep0stats {
		unsigned long ops;
		unsigned long bytes;
	} read, write;
	unsigned long irqs;
};

#define	PXA_UDC_NUM_ENDPOINTS	16	/*now support only 16 endpoints from 24 */
#define PXA_UDC_NUM_CONFIGS	4	/*3+1 for ep0 */

struct pxa27x_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;

#ifdef CONFIG_PREEMPT_RT
	spinlock_t	lock;
#endif
	enum ep0_state ep0state;
	struct udc_stats stats;
	unsigned got_irq:1,
	    got_disc:1, has_cfr:1, req_pending:1, req_std:1, req_config:1;
	unsigned num_cfg;
	unsigned num_ifc;
	unsigned num_aifc;
	unsigned cfg_done;
#define start_watchdog(dev) mod_timer(&dev->timer, jiffies + (HZ/200))
	struct timer_list timer;

	struct device *dev;
	struct pxa2xx_udc_mach_info *mach;
	u64 dma_mask;
	struct pxa27x_ep ep[PXA_UDC_NUM_ENDPOINTS];
};

struct udc_config_desc {
	unsigned first;
	unsigned num;
};

static unsigned num_use_endpoints = 3;	/*endpoints quantity needed for Gadget driver */

static struct pxa27x_udc *the_controller;

static inline int is_usb_connected(void)
{
	if (!the_controller->mach->udc_is_connected)
		return 1;
	return the_controller->mach->udc_is_connected();
}

static inline void make_usb_disappear(void)
{
	if (!the_controller->mach->udc_command)
		return;
	the_controller->mach->udc_command(PXA2XX_UDC_CMD_DISCONNECT);
}

static inline void let_usb_appear(void)
{
	if (!the_controller->mach->udc_command)
		return;
	the_controller->mach->udc_command(PXA2XX_UDC_CMD_CONNECT);
}

/*-------------------------------------------------------------------------*/
/*
 * Debugging support vanishes in non-debug builds.  DBG_NORMAL should be
 * mostly silent during normal use/testing, with no timing side-effects.
 */
#define DBG_NORMAL	1	/* error paths, device state transitions */
#define DBG_VERBOSE	2	/* add some success path trace info */
#define DBG_NOISY	3	/* ... even more: request level */
#define DBG_VERY_NOISY	4	/* ... even more: packet level */

#define PREFIX "udc: "

#ifdef DEBUG

static const char *state_name[] = {
	"EP0_IDLE",
	"EP0_IN_DATA_PHASE", "EP0_OUT_DATA_PHASE",
	"EP0_END_XFER", "EP0_STALL"
};

#ifdef VERBOSE
#    define UDC_DEBUG DBG_VERBOSE
#else
#    define UDC_DEBUG DBG_NORMAL
#endif

static void __attribute__ ((__unused__))
    dump_udccr(const char *label)
{
	u32 udccr = UDCCR;
	pr_debug(PREFIX "%s %02X =%s%s%s%s%s%s%s%s%s%s\n",
	     label, udccr,
	     (udccr & UDCCR_OEN) ? " oen" : "",
	     (udccr & UDCCR_AALTHNP) ? " aalthnp" : "",
	     (udccr & UDCCR_AHNP) ? " ahnp" : "",
	     (udccr & UDCCR_BHNP) ? " bhnp" : "",
	     (udccr & UDCCR_DWRE) ? " dwre" : "",
	     (udccr & UDCCR_SMAC) ? " smac" : "",
	     (udccr & UDCCR_EMCE) ? " emce" : "",
	     (udccr & UDCCR_UDR) ? " udr" : "",
	     (udccr & UDCCR_UDA) ? " uda" : "",
	     (udccr & UDCCR_UDE) ? " ude" : "");

}

static void __attribute__ ((__unused__))
    dump_udccsr0(const char *label)
{
	u32 udccsr0 = UDCCSR0;

	pr_debug(PREFIX "%s %s %02X =%s%s%s%s%s%s%s%s\n",
	     label, state_name[the_controller->ep0state], udccsr0,
	     (udccsr0 & UDCCSR0_SA) ? " sa" : "",
	     (udccsr0 & UDCCSR0_RNE) ? " rne" : "",
	     (udccsr0 & UDCCSR0_FST) ? " fst" : "",
	     (udccsr0 & UDCCSR0_SST) ? " sst" : "",
	     (udccsr0 & UDCCSR0_DME) ? " dme" : "",
	     (udccsr0 & UDCCSR0_FTF) ? " ftf" : "",
	     (udccsr0 & UDCCSR0_IPR) ? " ipr" : "",
	     (udccsr0 & UDCCSR0_OPC) ? " opc" : "");

}

static void __attribute__ ((__unused__))
    dump_state(struct pxa27x_udc *dev)
{

	u32 tmp;
	unsigned i;

	pr_debug(PREFIX "%s %s, udcicr %02X.%02X, udcisr %02X.%02x, ufnr %X\n",
	     is_usb_connected()? "host " : "disconnected",
	     state_name[dev->ep0state],
	     UDCICR1, UDCICR0, UDCISR1, UDCISR0, UDCFNR);
	dump_udccr("udccr");
	if (dev->has_cfr) {
		tmp = UDCCSR0;
		pr_debug(PREFIX "udccsr0 %02X =%s%s\n", tmp,
		     (tmp & (1 << 8)) ? " aren" : "",
		     (tmp & (1 << 9)) ? " acm" : "");
	}

	if (!dev->driver) {
		pr_debug(PREFIX "no gadget driver bound\n");
		return;
	} else
		pr_debug(PREFIX "ep0 driver '%s'\n", dev->driver->driver.name);

	if (!is_usb_connected())
		return;

	dump_udccsr0("udccsr0");
	pr_debug(PREFIX "ep0 IN %lu/%lu, OUT %lu/%lu\n",
	     dev->stats.write.bytes, dev->stats.write.ops,
	     dev->stats.read.bytes, dev->stats.read.ops);

	for (i = 1; i < num_use_endpoints; i++) {
		if (dev->ep[i].desc == 0)
			continue;
		pr_debug(PREFIX "udccsr%d = %02x\n", i, *dev->ep->reg_udccsr);
	}

}

#else

#define	dump_udccr(x)	do{}while(0)
#define	dump_udccs0(x)	do{}while(0)
#define	dump_state(x)	do{}while(0)

#define UDC_DEBUG ((unsigned)0)

#endif

#define DBG(lvl, stuff...) do{if ((lvl) <= UDC_DEBUG) pr_debug(PREFIX stuff);}while(0)

#define WARN(stuff...) printk(KERN_WARNING PREFIX stuff)
#define INFO(stuff...) printk(KERN_INFO PREFIX stuff)

#endif				/* __LINUX_USB_GADGET_PXA27X_H */
