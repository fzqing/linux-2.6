/*
 * linux/drivers/usb/gadget/pxa3xx_udc.h
 * Intel PXA3xx on-chip full speed USB device controller
 *
 * Copyright (C) 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2004 Intel Corporation
 * Copyright (C) 2007 Marvell International Ltd.
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

/*
 * The CONFIG_USB_COMPOSITE macro enables the ability to support several
 * gadget drivers to be bould with this driver at the same time and work
 * together. This implements a device that has multiple interfaces con-
 * trolled independently of each other, which is referred to as a "compo-
 * site" device in the USB 2.0 spec.
 * As the original design of gadget driver doesn't support several ones
 * to work together, here list some limitations in the current version
 * implemented here:
 * 1. Only support gadget's default configuration
 *    the device configuration is not static, will be the gadget config
 *    value first inserted. Other configuration other than the default
 *    in one gadget driver won't be viewed by USB host.
 * 2. Gadget won't keep gadget driver's private data for each one any
 *    more
 *    gadget driver should maintain the driver's private data themselves,
 *    but not in the gadget any more. e.g. "driver_data" in struct usb_ep
 *    for ep0, and driver_data in "dev" for struct usb_gadget.
 * 3. Consider every gadget driver implements only one function
 *    For gadget driver that already has multiple interface and each
 *    implements a different function, we don't know that it's not one
 *    function and will consider it as one function with multiple
 *    interfaces. And software will add the interface association
 *    descriptor to USB host, which is unnecessary.
 * 4. Potentail conflict exists in class/vendor specific requests
 *    through the default pipe
 *    For class/vendor specific requests that also use the default
 *    pipe, it is hard to determine which gadget driver shoule the
 *    requset to be passed to. The current implementation will give the
 *    request to every gadget driver until it doesn't return EOPNOTSUPP.
 *    This will cause a little delay to the request reponse. And we have
 *    the premiss that no class/vendor specific requests are the same
 *    in two gadget drivers.
 * 5. String descriptors not well supported yet
 *    String descriptors are not well supported when multiple gadget
 *    drivers are inserted. It would pass the request to the "active"
 *    (gadget driver module last registered or with the interface that
 *    last SET_INTERFACE command issued.
 * 6. rmmod issue
 *    please remove all the modules simultaneously when change combination
 *    of the function.
 */

#ifndef __LINUX_USB_GADGET_PXA3XX_H
#define __LINUX_USB_GADGET_PXA3XX_H

#include <linux/types.h>
#include <asm/arch/cpu-freq-voltage-mhn.h>
struct pxa3xx_udc;

#define CONFIG_USB_COMPOSITE

#ifdef CONFIG_USB_COMPOSITE
#undef MULTIPLE_CONFIGURATION	/* FIXME, RNDIS + CDC */
	/* couldn't work in one module both with linux and windows machine */
#undef MULTI_P4			/* FIXME, string descriptor for interface support */
#define MULTI_P3		/* add iad descriptor */
#define UDC_DEFAULT_CONFIG 1	/* FIXME for rndis */
#define UDC_VENDOR_NUM 			0x0525
#define UDC_PRODUCT_NUM 		0x0aaa
#endif

#define DMA_BUFFER_SIZE	 PAGE_SIZE
struct pxa3xx_ep {
	struct usb_ep ep;
	struct pxa3xx_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	struct otg_transceiver *trasceiver;
	unsigned long pio_irqs;
	unsigned long dma_irqs;

	int dma;
	unsigned fifo_size;
	unsigned ep_num;
	unsigned ep_type;

	unsigned stopped:1;
	unsigned dma_con:1;
	unsigned dir_in:1;
	unsigned assigned:1;

	void *dma_buffer_virt;
	dma_addr_t dma_buffer_phys;
	unsigned dma_buffer_size;

	unsigned config;
	unsigned interface;
	unsigned aisn;
	/* UDCCSR = UDC Control/Status Register for this EP
	 * UBCR = UDC Byte Count Remaining (contents of OUT fifo)
	 * UDCDR = UDC Endpoint Data Register (the fifo)
	 * UDCCR = UDC Endpoint Configuration Registers
	 * DRCM = DMA Request Channel Map
	 */
	volatile u32 *reg_udccsr;
	volatile u32 *reg_udcbcr;
	volatile u32 *reg_udcdr;
	volatile u32 *reg_udccr;
#ifdef	CONFIG_USB_PXA3XX_DMA
	volatile u32 *reg_drcmr;
#define	drcmr(n)  .reg_drcmr = & DRCMR ## n ,
#else
#define	drcmr(n)
#endif

#ifdef CONFIG_PM
	unsigned udccsr_value;
	unsigned udccr_value;
#endif
#ifdef CONFIG_USB_COMPOSITE
	unsigned assigned_interface; /* actual interface number report to the host */
	struct gadget_driver_info *driver_info;	/* pointer to corresponding
						   gadget_driver_info */
#endif
};

struct pxa3xx_request {
	struct usb_request req;
	struct list_head queue;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_STALL,
	EP0_IN_FAKE,
	EP0_NO_ACTION
};

#define EP0_FIFO_SIZE	((unsigned)16)
#define BULK_FIFO_SIZE	((unsigned)64)
#ifdef PATCH_TEMP
#define ISO_FIFO_SIZE	((unsigned)512)
#else
#define ISO_FIFO_SIZE	((unsigned)256)
#endif
#define INT_FIFO_SIZE	((unsigned)8)

struct udc_stats {
	struct ep0stats {
		unsigned long ops;
		unsigned long bytes;
	} read, write;
	unsigned long irqs;
};

#ifndef	UDC_EP_NUM
#define	UDC_EP_NUM	24
#endif

#define MAX_CONFIG_LENGTH 256

#ifdef CONFIG_USB_COMPOSITE

struct gadget_driver_info {
	struct gadget_driver_info *next;  /* point to next gadget_driver_info */
	unsigned config;           /* configuration number used by the driver */
	unsigned assigned_intf_start;  /* the first assigned interface number */
	unsigned num_intfs;          /* total interface number for the driver */
	unsigned ep_start;         /* the first ep number used in the ep list */
	unsigned num_eps;
#ifdef MULTI_P3
	unsigned char device_desc[18];        	/* device desc for the driver */
#endif
	unsigned char config_desc[MAX_CONFIG_LENGTH];  /* configuration desc for
								   the driver */

	struct usb_gadget_driver *driver;	/* struct usb_gadget_driver ---
					   store the driver pointer of gadget */
	void *driver_data;            	/* pointer to the driver private data */
	unsigned stopped;                  	/* driver disconnected or not */
};

#ifdef MULTI_P4
struct t_str_id {
	int str_id;	                                     /* old string id */
	struct gadget_driver_info *driver_info;	/* pointer to the driver info */
	struct t_str_id *next;
};
#endif
#endif

struct pxa3xx_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	spinlock_t lock;

#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info *first_gadget; /* head of a gadget_driver_info
									queue */
	struct gadget_driver_info *active_gadget;  /* currently active gadget */
	int interface_count;          	/* how many gadget already registered */
	int driver_count;                         /* how many ep already used */
	struct pxa3xx_request ep0_req;	/* include the usb_req to respond to the
							get_desc request, etc */
	struct t_str_id *str_id;
	int rm_flag;
#endif

	struct otg_transceiver *transceiver;
	enum ep0_state ep0state;
	struct udc_stats stats;
	unsigned got_irq:1, got_disc:1, has_cfr:1, req_pending:1,
#ifndef CONFIG_USB_COMPOSITE
	 req_std:1, req_config:1;
#else
	 req_std:1;
	unsigned req_config;
#endif

#define start_watchdog(dev) mod_timer(&dev->timer, jiffies + (HZ/200))
	struct timer_list timer;

	struct device *dev;
	struct pxa3xx_udc_mach_info *mach;
	u64 dma_mask;
	struct pxa3xx_ep ep[UDC_EP_NUM];

	unsigned char configs[MAX_CONFIG_LENGTH];
	unsigned config_length;
	unsigned configuration, interface, alternate;
#ifdef CONFIG_PM
	unsigned udccsr0;
#endif

#ifdef CONFIG_DVFM
	struct mhn_fv_notifier dvfm_notifier;
#endif
	struct workqueue_struct *workqueue;
	struct work_struct *work;

};

/*-------------------------------------------------------------------------*/

/* LEDs are only for debug */
#ifndef HEX_DISPLAY
#define HEX_DISPLAY(n)		do {} while(0)
#endif

#ifndef LED_CONNECTED_ON
#define LED_CONNECTED_ON	do {} while(0)
#define LED_CONNECTED_OFF	do {} while(0)
#endif
#ifndef LED_EP0_ON
#define LED_EP0_ON		do {} while (0)
#define LED_EP0_OFF		do {} while (0)
#endif

static struct pxa3xx_udc *the_controller;

/*-------------------------------------------------------------------------*/

/*
 * Debugging support vanishes in non-debug builds.  DBG_NORMAL should be
 * mostly silent during normal use/testing, with no timing side-effects.
 */
#define DBG_NORMAL	1	/* error paths, device state transitions */
#define DBG_VERBOSE	2	/* add some success path trace info */
#define DBG_NOISY	3	/* ... even more: request level */
#define DBG_VERY_NOISY	4	/* ... even more: packet level */

#ifdef DEBUG

static const char *state_name[] = {
	"EP0_IDLE",
	"EP0_IN_DATA_PHASE", "EP0_OUT_DATA_PHASE",
	"EP0_END_XFER", "EP0_STALL"
};

#define DMSG(stuff...) printk(KERN_DEBUG stuff)

#ifdef VERBOSE
#    define UDC_DEBUG DBG_VERY_NOISY
#else
#    define UDC_DEBUG DBG_NORMAL
#endif

static void __attribute__ ((__unused__))
    dump_udccr(const char *label)
{
	u32 udccr = UDCCR;
	DMSG("%s 0x%08x =%s%s%s%s%s%s%s%s%s%s, con=%d,inter=%d,altinter=%d\n",
	     label, udccr,
	     (udccr & UDCCR_OEN) ? " oen" : "",
	     (udccr & UDCCR_AALTHNP) ? " aalthnp" : "",
	     (udccr & UDCCR_AHNP) ? " rem" : "",
	     (udccr & UDCCR_BHNP) ? " rstir" : "",
	     (udccr & UDCCR_DWRE) ? " dwre" : "",
	     (udccr & UDCCR_SMAC) ? " smac" : "",
	     (udccr & UDCCR_EMCE) ? " emce" : "",
	     (udccr & UDCCR_UDR) ? " udr" : "",
	     (udccr & UDCCR_UDA) ? " uda" : "",
	     (udccr & UDCCR_UDE) ? " ude" : "",
	     (udccr & UDCCR_ACN) >> UDCCR_ACN_S,
	     (udccr & UDCCR_AIN) >> UDCCR_AIN_S,
	     (udccr & UDCCR_AAISN) >> UDCCR_AAISN_S);
}

static void __attribute__ ((__unused__))
    dump_udccsr0(const char *label)
{
	u32 udccsr0 = UDCCSR0;

	DMSG("%s %s 0x%08x =%s%s%s%s%s%s%s\n",
	     label, state_name[the_controller->ep0state], udccsr0,
	     (udccsr0 & UDCCSR0_SA) ? " sa" : "",
	     (udccsr0 & UDCCSR0_RNE) ? " rne" : "",
	     (udccsr0 & UDCCSR0_FST) ? " fst" : "",
	     (udccsr0 & UDCCSR0_SST) ? " sst" : "",
	     (udccsr0 & UDCCSR0_DME) ? " dme" : "",
	     (udccsr0 & UDCCSR0_IPR) ? " ipr" : "",
	     (udccsr0 & UDCCSR0_OPC) ? " opr" : "");
}

static void __attribute__ ((__unused__))
    dump_state(struct pxa3xx_udc *dev)
{
	unsigned i;

	DMSG("%s, udcicr %02X.%02X, udcsir %02X.%02x, udcfnr %02X\n",
	     state_name[dev->ep0state],
	     UDCICR1, UDCICR0, UDCISR1, UDCISR0, UDCFNR);
	dump_udccr("udccr");

	if (!dev->driver) {
		DMSG("no gadget driver bound\n");
		return;
	} else
		DMSG("ep0 driver '%s'\n", dev->driver->driver.name);

	dump_udccsr0("udccsr0");
	DMSG("ep0 IN %lu/%lu, OUT %lu/%lu\n",
	     dev->stats.write.bytes, dev->stats.write.ops,
	     dev->stats.read.bytes, dev->stats.read.ops);

	for (i = 1; i < UDC_EP_NUM; i++) {
		if (dev->ep[i].desc == 0)
			continue;
		DMSG("udccs%d = %02x\n", i, *dev->ep->reg_udccsr);
	}
}

static void dump_regs(u8 ep)
{
	DMSG("EP:%d UDCCSR:0x%08x UDCBCR:0x%08x\n UDCCR:0x%08x\n",
	     ep, UDCCSN(ep), UDCBCN(ep), UDCCN(ep));
}
static void dump_req(struct pxa3xx_request *req)
{
	struct usb_request *r = &req->req;

	DMSG("%s: buf:0x%08x length:%d dma:0x%08x actual:%d\n",
	     __FUNCTION__, (unsigned)r->buf, r->length, r->dma, r->actual);
}

#else

#define DMSG(stuff...)		do{}while(0)

#define	dump_udccr(x)	do{}while(0)
#define	dump_udccsr0(x)	do{}while(0)
#define	dump_state(x)	do{}while(0)

#define UDC_DEBUG ((unsigned)0)

#endif

#define DBG(lvl, stuff...) do{if ((lvl) <= UDC_DEBUG) DMSG(stuff);}while(0)

#define WARN(stuff...) printk(KERN_WARNING "udc: " stuff)
#define INFO(stuff...) printk(KERN_INFO "udc: " stuff)

#endif				/* __LINUX_USB_GADGET_PXA3XX_H */
