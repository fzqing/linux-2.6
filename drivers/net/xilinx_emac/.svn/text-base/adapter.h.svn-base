/*
 * drivers/net/xilinx_emac/adapter.h
 *
 * Include file for Xilinx Ethernet Adapter component
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2005 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#ifdef __KERNEL__
#ifndef __XEMAC_ADAPTER_H__
#define __XEMAC_ADAPTER_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/list.h>
#include <asm/atomic.h>

#include "xbasic_types.h"
#include "xemac.h"
#include "xemac_i.h"

#define DRIVER_NAME 		"xilinx_emac"
#define DRIVER_VERSION		"1.0"
#define DRIVER_DESCRIPTION	"Xilinx Eth MAC driver"

/*
 * On the OPB, the 10/100 EMAC requires data to be aligned to 4 bytes.
 * On the PLB, the 10/100 EMAC requires data to be aligned to 8 bytes.
 * For simplicity, we always align to 8 bytes.
 */
#define ALIGNMENT	8
/*
 * In the SGDMA case 32-byte alignment is needed.
 */
#define SGDMA_ALIGNMENT	32
/*
 * BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment.
 */
#define BUFFER_ALIGN(adr) ((ALIGNMENT - ((u32) (adr))) % ALIGNMENT)
#define SGDMA_BUF_ALIGN(adr) ((SGDMA_ALIGNMENT - ((u32) (adr))) % SGDMA_ALIGNMENT)

/*
 * Most of our defaults are different from the ones set in xemac.h.
 * Note that XEmac_Reset() sets some options to the xemac.h's defaults.
 */

#define SGDMA_DEFAULT_THRESHOLD	31
#define SGDMA_DEFAULT_WAITBOUND	XEM_SGDMA_DFT_WAITBOUND
#define SGDMA_DEFAULT_RECV_BUFS	256	/* default # of recv buffers */
#define SGDMA_DEFAULT_SEND_BUFS	64	/* default # of send buffers */
#define SGDMA_DEFAULT_RECV_DESC	256	/* default # of recv descriptors */
#define SGDMA_DEFAULT_SEND_DESC	64	/* default # of send descriptors */

/* Common data structures and functions */

typedef enum DUPLEX { UNKNOWN_DUPLEX, HALF_DUPLEX, FULL_DUPLEX } DUPLEX;

struct net_local {
	struct list_head link;
	struct net_device_stats net_stats;	/* Statistics for this device */
	struct timer_list phy_timer;	/* PHY monitoring timer */
	XInterruptHandler Isr;		/* Pointer to the XEmac ISR routine */
	struct sk_buff *saved_skb;	/* skb being transmitted */
	spinlock_t skb_lock;		/* For atomic access to saved_skb */
	u8 mii_addr;			/* The MII address of the PHY */

	/*
	 * The underlying OS independent code needs space as well.  A
	 * pointer to the following XEmac structure will be passed to
	 * any XEmac_ function that requires it.  However, we treat the
	 * data as an opaque object in this file (meaning that we never
	 * reference any of the fields inside of the structure).
	 */
	XEmac Emac;

	/*
	 * SGDMA stuff
	 */

	void *desc_space_v;		/* start of the descriptor space */
	dma_addr_t desc_space_handle;	/* bus addr of the descriptor space */
	int desc_space_size;

	void *tx_bufs_v;	/* virt addr of the array of tx buffers */
	int tx_buffs_size;	/* size of the array (bytes) */
	int tx_bufs_offset;	/* offset to the 1st free buffer (bytes) */

	atomic_t tx_descs_avail;	/* the number of free tx descriptors */
};

/*
 * Helper function to determine if a given XEmac error warrants a reset.
 * Probably there is a better way to handle XST_DMA_SG_LIST_EMPTY error
 * than just resetting the device.
 */
extern inline int status_requires_reset(XStatus s)
{
	return (s == XST_DMA_ERROR || s == XST_FIFO_ERROR
		|| s == XST_RESET_ERROR || s == XST_DMA_SG_LIST_EMPTY);
}

/*
 * Helper function to reset the underlying hardware.  This is called
 * when we get into such deep trouble that we don't know how to handle
 * otherwise.
 */
void xenet_reset(struct net_device *ndev, DUPLEX duplex);

/* Transmission and receiption in FIFO mode */

void FifoSendHandler(void *CallbackRef);
void FifoRecvHandler(void *CallbackRef);
int xenet_start_xmit_fifo(struct sk_buff *skb, struct net_device *ndev);

/* Transmission and receiption in SGDMA mode */

void SgSendHandler(void *CallbackRef, XBufDescriptor * BdPtr, u32 NumBds);
void SgRecvHandler(void *CallbackRef, XBufDescriptor * BdPtr, u32 NumBds);
int xenet_start_xmit_sgdma(struct sk_buff *skb, struct net_device *ndev);
int sgdma_descriptor_init(struct net_device *ndev);

#endif /* __XEMAC_ADAPTER_H__ */
#endif /* __KERNEL__ */
