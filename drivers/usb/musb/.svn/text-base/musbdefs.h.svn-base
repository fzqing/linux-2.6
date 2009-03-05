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

#ifndef __MUSB_MUSBDEFS_H__
#define __MUSB_MUSBDEFS_H__

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/usb_ch9.h>
//#include <linux/usb_otg.h>

#include <linux/usb_musb.h>

#define	gfp_t	int

#include "debug.h"
#include "dma.h"

#ifdef CONFIG_USB_INVENTRA_STATIC_CONFIG
#include "plat_cnf.h"
#endif

#include "plat_arc.h"
#include "musbhdrc.h"

struct musb;

/* REVISIT tune this */
#define	MIN_DMA_REQUEST		1	/* use PIO below this xfer size */

#ifdef CONFIG_USB_MUSB_OTG
#include "otg.h"

#define	is_peripheral_enabled(musb)	((musb)->board_mode != MUSB_HOST)
#define	is_host_enabled(musb)		((musb)->board_mode != MUSB_PERIPHERAL)
#define	is_otg_enabled(musb)		((musb)->board_mode == MUSB_OTG)

/* NOTE:  otg and peripheral-only state machines start at B_IDLE.
 * OTG or host-only go to A_IDLE when ID is sensed.
 */
#define is_peripheral_active(m)	(is_peripheral_capable() && !(m)->bIsHost)
#define is_host_active(m)	(is_host_capable() && (m)->bIsHost)

#else
#define	is_peripheral_enabled(musb)	is_peripheral_capable()
#define	is_host_enabled(musb)		is_host_capable()
#define	is_otg_enabled(musb)		0

#define	is_peripheral_active(musb)	is_peripheral_capable()
#define	is_host_active(musb)		is_host_capable()
#endif

#ifdef CONFIG_PROC_FS
#include <linux/fs.h>
#define MUSB_CONFIG_PROC_FS
#define MUSB_STATISTICS
#endif

/****************************** PERIPHERAL ROLE ********************************/
#define MUSB_C_NUM_EPT		5
#define MUSB_C_NUM_EPR		5

struct musb_ep;

#ifdef CONFIG_USB_GADGET_MUSB_HDRC

#include <linux/usb_gadget.h>
#include "musb_gadget.h"

#define	is_peripheral_capable()	(1)

extern irqreturn_t musb_g_ep0_irq(struct musb *);
extern void musb_g_tx(struct musb *, u8, u8);
extern void musb_g_rx(struct musb *, u8);
extern void musb_g_reset(struct musb *);
extern void musb_g_suspend(struct musb *);
extern void musb_g_resume(struct musb *);
extern void musb_g_disconnect(struct musb *);

#else

#define	is_peripheral_capable()	(0)

static inline irqreturn_t musb_g_ep0_irq(struct musb *m)
{
	return IRQ_NONE;
}
static inline void musb_g_tx(struct musb *m, u8 e, u8 i)
{
}
static inline void musb_g_rx(struct musb *m, u8 e)
{
}
static inline void musb_g_reset(struct musb *m)
{
}
static inline void musb_g_suspend(struct musb *m)
{
}
static inline void musb_g_resume(struct musb *m)
{
}
static inline void musb_g_disconnect(struct musb *m)
{
}

#endif

/****************************** HOST ROLE **************************************/

struct musb_hw_ep;

#ifdef CONFIG_USB_MUSB_HDRC_HCD

/* for sizeof struct virtual_root */
#include "virthub.h"
// #include "musb_host.h"

#define	is_host_capable()	(1)

extern irqreturn_t musb_h_ep0_irq(struct musb *);
extern void musb_host_tx(struct musb *, u8);
extern void musb_host_rx(struct musb *, u8);

#else

#define	is_host_capable()	(0)

static inline irqreturn_t musb_h_ep0_irq(struct musb *m)
{
	return IRQ_NONE;
}
static inline void musb_host_tx(struct musb *m, u8 e)
{
}
static inline void musb_host_rx(struct musb *m, u8 e)
{
}

#endif

/****************************** OTG ROLE **************************************/

#if CONFIG_USB_MUSB_OTG

extern void otg_input_changed(struct musb *pThis, u8 devctl, u8 reset,
			      u8 connection, u8 suspend);
extern void otg_input_changed_X(struct musb *pThis, u8 bVbusError, u8 bConnect);

extern int musb_start_hnp(struct otg_transceiver *otg);
extern int musb_start_srp(struct otg_transceiver *otg);
extern int musb_set_host(struct otg_transceiver *otg, struct usb_bus *host);

extern int musb_set_peripheral(struct otg_transceiver *otg,
			       struct usb_gadget *gadget);
#endif

/****************************** CONSTANTS ********************************/

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef MUSB_C_NUM_EPS
#define MUSB_C_NUM_EPS ((u8)16)
#endif

#ifndef MUSB_MAX_END0_PACKET
#define MUSB_MAX_END0_PACKET ((u16)MGC_END0_FIFOSIZE)
#endif

/* host side ep0 states */
#define MGC_END0_START  0x0
#define MGC_END0_OUT    0x2
#define MGC_END0_IN     0x4
#define MGC_END0_STATUS 0x8

/* peripheral side ep0 states */
enum musb_g_ep0_state {
	MGC_END0_STAGE_SETUP,	/* idle, waiting for setup */
	MGC_END0_STAGE_TX,	/* IN data */
	MGC_END0_STAGE_RX,	/* OUT data */
	MGC_END0_STAGE_STATUSIN,	/* (after OUT data) */
	MGC_END0_STAGE_STATUSOUT,	/* (after IN data) */
	MGC_END0_STAGE_ACKWAIT,	/* after zlp, before statusin */
} __attribute__ ((packed));

/* failure codes */
#define MUSB_ERR_WAITING	1
#define MUSB_ERR_VBUS		-1
#define MUSB_ERR_BABBLE		-2
#define MUSB_ERR_CORRUPTED	-3
#define MUSB_ERR_IRQ		-4
#define MUSB_ERR_SHUTDOWN	-5
#define MUSB_ERR_RESTART	-6

/****************************** FUNCTIONS ********************************/

#define kzalloc(n,f) kcalloc(1,(n),(f))

/*************************** REGISTER ACCESS ********************************/

/* Endpoint registers (other than dynfifo setup) can be accessed either
 * directly with the "flat" model, or after setting up an index register.
 */

#if defined(CONFIG_ARCH_DAVINCI)
/* REVISIT "flat" takes about 1% more object code space and can't be very
 * noticeable for speed differences.  But for now indexed access seems to
 * misbehave at least for peripheral IN ...
 */
#define	MUSB_FLAT_REG
#endif

#if	defined(CONFIG_USB_TUSB_6010)
#define MGC_SelectEnd(_pBase, _bEnd) \
	musb_writeb((_pBase), MGC_O_HDRC_INDEX, (_bEnd))
#define	MGC_END_OFFSET			MGC_TUSB_OFFSET

#elif	defined(MUSB_FLAT_REG)
#define MGC_SelectEnd(_pBase, _bEnd)	(((void)(_pBase)),((void)(_bEnd)))
#define	MGC_END_OFFSET			MGC_FLAT_OFFSET

#else
#define MGC_SelectEnd(_pBase, _bEnd) \
	musb_writeb((_pBase), MGC_O_HDRC_INDEX, (_bEnd))
#define	MGC_END_OFFSET			MGC_INDEXED_OFFSET
#endif

#define MGC_ReadCsr8(_pBase, _bOffset, _bEnd) \
	musb_readb((_pBase), MGC_END_OFFSET((_bEnd), (_bOffset)))
#define MGC_ReadCsr16(_pBase, _bOffset, _bEnd) \
	musb_readw((_pBase), MGC_END_OFFSET((_bEnd), (_bOffset)))
#define MGC_WriteCsr8(_pBase, _bOffset, _bEnd, _bData) \
	musb_writeb((_pBase), MGC_END_OFFSET((_bEnd), (_bOffset)), (_bData))
#define MGC_WriteCsr16(_pBase, _bOffset, _bEnd, _bData) \
	musb_writew((_pBase), MGC_END_OFFSET((_bEnd), (_bOffset)), (_bData))

/****************************** FUNCTIONS ********************************/

#define MUSB_HST_MODE(_pthis) { (_pthis)->bIsHost=TRUE; (_pthis)->bIsDevice=FALSE; \
	(_pthis)->bFailCode=0; }
#define MUSB_DEV_MODE(_pthis) { (_pthis)->bIsHost=FALSE; (_pthis)->bIsDevice=TRUE; \
	(_pthis)->bFailCode=0; }
#define MUSB_OTG_MODE(_pthis) { (_pthis)->bIsHost=FALSE; (_pthis)->bIsDevice=FALSE; \
	(_pthis)->bFailCode=MUSB_ERR_WAITING; }
#define MUSB_ERR_MODE(_pthis, _cause) { (_pthis)->bIsHost=FALSE; (_pthis)->bIsDevice=FALSE; \
	(_pthis)->bFailCode=_cause; }

#define MUSB_IS_ERR(_x) ( (_x)->bFailCode<0 )
#define MUSB_IS_HST(_x) ( !MUSB_IS_ERR(_x) && (_x)->bIsHost && !(_x)->bIsDevice )
#define MUSB_IS_DEV(_x) ( !MUSB_IS_ERR(_x) && !(_x)->bIsHost && (_x)->bIsDevice )
#define MUSB_IS_OTG(_x) ( !MUSB_IS_ERR(_x) && !(_x)->bIsHost && !(_x)->bIsDevice )

#define test_devctl_hst_mode(_x) (musb_readb((_x)->pRegs, MGC_O_HDRC_DEVCTL)&MGC_M_DEVCTL_HM)

#define MUSB_MODE(_x) ( MUSB_IS_HST(_x)?"HOST" \
		:( MUSB_IS_DEV(_x)?"PERIPHERAL" \
		:(MUSB_IS_OTG(_x)?"UNCONNECTED" \
		:"ERROR")) )

/******************************** DMA TYPES **********************************/

#ifdef CONFIG_USB_INVENTRA_DMA
#include "dma.h"

#ifndef MGC_HSDMA_CHANNELS
#define MGC_HSDMA_CHANNELS 8
#endif

#endif

/************************** Ep Configuration ********************************/

/** The End point descriptor */
struct MUSB_EpFifoDescriptor {
	u8 bType;		/* 0 for autoconfig, CNTR, ISOC, BULK, INTR */
	u8 bDir;		/* 0 for autoconfig, INOUT, IN, OUT */
	int wSize;		/* 0 for autoconfig, or the size */
};

#define MUSB_EPD_AUTOCONFIG	0

#define MUSB_EPD_T_CNTRL	1
#define MUSB_EPD_T_ISOC		2
#define MUSB_EPD_T_BULK		3
#define MUSB_EPD_T_INTR		4

#define MUSB_EPD_D_INOUT	0
#define MUSB_EPD_D_TX		1
#define MUSB_EPD_D_RX		2

/******************************** TYPES *************************************/
struct musb_iso_desc {
	u32	offset;
	u32	length;
	u32	status;
};

#ifdef CONFIG_USB_MUSB_HDRC_HCD
/* Used for mixing USB requests between dma and pio modes */
struct musb_opmode {
	u8			dma;
	struct musb_hw_ep	*ep;
};
#endif

/*
 * struct musb_hw_ep - endpoint hardware (bidirectional)
 *
 * REVISIT the TX and RX sides should be more completely decoupled,
 * each with separate host and peripheral side state structures
 */
struct musb_hw_ep {
	struct musb *musb;
	void __iomem *fifo;
	void __iomem *regs;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* host side */

#if 1
#define in_urb_list		urb_list
#define out_urb_list		urb_list
	struct list_head urb_list;

#define in_traffic_type		bTrafficType
#define out_traffic_type	bTrafficType
	u8 bTrafficType;

#define in_busy			busy
#define out_busy		busy
	u8 busy;

	/* FIXME not all host side endpoint structures reflect the fact
	 * that each of these endpoints could go in either direction,
	 * unless it's using a shared fifo ...
	 */
	u8 bIsClaimed;
	u8 bAddress;
	u8 bEnd;
	u8 bIsReady;
	u16 wPacketSize;
	u16 dwWaitFrame;

	unsigned int dwOffset;
	unsigned int dwRequestSize;
	unsigned int dwIsoPacket;
	struct musb_opmode musb_mode [2];
#else
	struct musb_host_ep in;
	struct musb_host_ep out;

#define in_urb_list		in.urb_list
#define out_urb_list		out.urb_list
#define in_traffic_type		in.type
#define out_traffic_type	out.type
#define in_busy			in.busy
#define out_busy		out.busy

#endif

#endif				/* CONFIG_USB_MUSB_HDRC_HCD */

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	/* peripheral side */
	struct musb_ep ep_in;	/* TX */
	struct musb_ep ep_out;	/* RX */
#endif

	/* FIXME each direction should have its own channel... */
#define	tx_channel	pDmaChannel
#define	rx_channel	pDmaChannel
	struct dma_channel *pDmaChannel;
	//struct dma_channel    *tx_channel;
	//struct dma_channel    *rx_channel;

	/* hardware configuration, possibly dynamic */
	u16 wMaxPacketSizeTx;
	u16 wMaxPacketSizeRx;
	u8 tx_double_buffered;
	u8 rx_double_buffered;
	u8 bIsSharedFifo;

	/* index in musb->aLocalEnd[]  */
	u8 bLocalEnd;
	struct musb_iso_desc *iso_desc;
	u32	num_iso_desc;
	u8 fifo_flush_check;
};

static inline struct usb_request *next_in_request(struct musb_hw_ep *hw_ep)
{
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	return next_request(&hw_ep->ep_in);
#else
	return NULL;
#endif
}

static inline struct usb_request *next_out_request(struct musb_hw_ep *hw_ep)
{
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	return next_request(&hw_ep->ep_out);
#else
	return NULL;
#endif
}
/*
 * struct musb - Driver instance data.
 */
struct musb {
	spinlock_t Lock;
	struct usb_bus *pBus;
	struct clk *clock;
	 irqreturn_t(*isr) (int, void *, struct pt_regs *);

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	struct virtual_root RootHub;
	struct usb_device *pRootDevice;
	struct timer_list Timer;

	u8 bEnd0Stage;		/* end0 stage while in host */

	/* bulk traffic normally dedicates endpoint hardware, and each
	 * direction has its own ring of host side endpoints.
	 * we try to progress the transfer at the head of each endpoint's
	 * queue until it completes or NAKs too much; then we try the next
	 * endpoint.
	 */
	struct musb_hw_ep *bulk_tx_end;
	struct musb_hw_ep *bulk_rx_end;

#ifdef	SCHEDULER
	/* REVISIT implement a schedule, something like this. */
	struct list_head tx_bulk;	/* of musb_sched_node */
	struct list_head rx_bulk;	/* of musb_sched_node */
	struct musb_sched_node *periodic[32];	/* tree of interrupt+iso */
#endif				/* SCHEDULER */

#endif

	struct dma_controller *pDmaController;
	u64 *old_dma_mask;	/* added for use_dma module param */

	struct device *controller;
	void __iomem *ctrl_base;
	void __iomem *pRegs;

	/* passed down from chip/board specific irq handlers */
	u8 int_usb;
	u16 int_rx;
	u16 int_tx;
	struct pt_regs *int_regs;
	/* HBG 21 SEPT 2006 moved it to otg_machine strucure */
	/* struct otg_transceiver       xceiv; */

	int nIrq;

	struct musb_hw_ep aLocalEnd[MUSB_C_NUM_EPS];

	u16 wEndMask;
	u8 bEndCount;
	u8 bRootSpeed;
	u8 board_mode;		/* enum musb_mode */

	s8 bFailCode;		/* one of MUSB_ERR_* failure code */

	unsigned bIsMultipoint:1;
	unsigned bIsDevice:1;
	unsigned bIsHost:1;
	unsigned bIgnoreDisconnect:1;	/* during bus resets, fake disconnects */
	unsigned bBulkSplit:1;
	unsigned bBulkCombine:1;

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	unsigned bIsSelfPowered:1;
	unsigned bMayWakeup:1;
	unsigned bSetAddress:1;
	unsigned bTestMode:1;
	unsigned softconnect:1;

	enum musb_g_ep0_state ep0_state;
	u8 bAddress;
	u8 bTestModeValue;
	u16 ackpend;		/* ep0 */
	struct usb_gadget g;	/* the gadget */
	struct usb_gadget_driver *pGadgetDriver;	/* its driver */
#endif

#ifdef CONFIG_ARCH_DAVINCI
	u16	vbuserr_retry;
#endif

#ifdef CONFIG_USB_MUSB_OTG
	struct otg_machine OtgMachine;
	u8 bDelayPortPowerOff;
#endif
#ifdef MUSB_CONFIG_PROC_FS
	struct proc_dir_entry *pProcEntry;
#endif
	struct tasklet_struct fifo_check;
	void (*fifo_check_complete) (struct musb_hw_ep *ep);
};

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
static inline struct musb *gadget_to_musb(struct usb_gadget *g)
{
	return container_of(g, struct musb, g);
}
#endif

/***************************** Glue it together *****************************/

extern const char musb_driver_name[];

void *musb_alloc_buffer(struct musb *pThis, size_t bytes, gfp_t gfp_flags,
			dma_addr_t * dma);
void musb_free_buffer(struct musb *pThis, size_t bytes, void *address,
		      dma_addr_t dma);

extern void musb_start(struct musb *pThis);
extern void musb_stop(struct musb *pThis);

extern void musb_write_fifo(struct musb_hw_ep *ep,
			    u16 wCount, const u8 * pSource);
extern void musb_read_fifo(struct musb_hw_ep *ep, u16 wCount, u8 * pDest);

extern irqreturn_t musb_interrupt(struct musb *);

extern void musb_platform_enable(struct musb *musb);
extern void musb_platform_disable(struct musb *musb);
/* HBG 25 SEPT 2006 */
//-------------------------
extern void musb_pullup(struct musb *musb, int is_on);
extern u8 is_otg_b_device(struct musb *pThis);
//-------------------------
extern int __init musb_platform_init(struct musb *musb);
extern int musb_platform_exit(struct musb *musb);

/*-------------------------- ProcFS definitions ---------------------*/

struct proc_dir_entry;

#if (MUSB_DEBUG > 0) && defined(MUSB_CONFIG_PROC_FS)
extern struct proc_dir_entry *musb_debug_create(char *name, struct musb *data);
extern void musb_debug_delete(char *name, struct musb *data);

#else
static inline struct proc_dir_entry *musb_debug_create(char *name,
						       struct musb *data)
{
	return NULL;
}
static inline void musb_debug_delete(char *name, struct musb *data)
{
}
#endif

#endif				/* __MUSB_MUSBDEFS_H__ */
