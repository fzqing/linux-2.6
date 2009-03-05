/*
 * drivers/net/xilinx_emac/adapter_sgdma.c
 *
 * Xilinx Ethernet Adapter component: transmission and receiption in SGDMA mode.
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2005 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>

#include "adapter.h"

/*
 * Physical to virtual pointer conversion - for the given DMA channel's
 * descriptors only.
 */
#define P_TO_V(InstancePtr, p) \
	((p) ? \
	 ((InstancePtr)->VirtPtr + ((u32)(p) - (u32)(InstancePtr)->PhyPtr)) : \
	 0)

void free_rx_skbs(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	XBufDescriptor* bd_ptr;
	struct sk_buff* skb;
	int i;

	bd_ptr = (XBufDescriptor*)lp->Emac.RecvChannel.VirtPtr;
	for (i = 0; i < SGDMA_DEFAULT_RECV_DESC; i++) {
		skb = (struct sk_buff*)XBufDescriptor_GetId(bd_ptr);
		if (skb == NULL) {
			/* one can get here if sgdma_descriptor_init() fails */
			continue;
		}
		/*
		 * The assumption here is that the DestAddress is *not*
		 * written back into the BufferDescriptor after transfer
		 * is completed.
		 * In the contrary, the Length does gets updated to the
		 * actual number of the received bytes. That's why the
		 * size arg of dma_unmap_single() is the same XEM_MAX_FRAME_SIZE
		 * constant that was used with the corresponding
		 * dma_map_single() (see sgdma_descriptor_init() and
		 * SgRecvHandler()).
		 */
		dma_unmap_single(NULL, XBufDescriptor_GetDestAddress(bd_ptr),
				 XEM_MAX_FRAME_SIZE, PCI_DMA_FROMDEVICE);
		dev_kfree_skb(skb);
		bd_ptr = P_TO_V(&lp->Emac.RecvChannel,
			       XBufDescriptor_GetNextPtr(bd_ptr));
	}
}

/*
 * This function is assumed to be called after XEmac_Initialize(),
 * but before XEmac is started with XEmac_Start().
 * Calling XEmac_SgRecv() if XEmac is not started is just fine.
 */
int sgdma_descriptor_init(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	void *p_v, *p_bus;
	u32 align;
	int i;

	/*
	 * Allocate memory for the buffer descriptors.
	 * 8-byte descriptor alignment is needed for PLB EMAC.
	 * "((sizeof (XBufDescriptor) % ALIGNMENT) == 0)" is assumed 
	 * to be true (due to how dma driver works) and 
	 * the "+ ALIGNMENT" is to allow for space to shift the records 
	 * to make them aligned. If it is not the case, XDmaChannel_CreateSgList() 
	 * called by XEmac_SetSg[Recv/Send]Space() would need reimplementing and
	 * level 1 driver for SGDMA engine needs rewrite.
	 */
	lp->desc_space_size = (SGDMA_DEFAULT_RECV_DESC + SGDMA_DEFAULT_SEND_DESC) *
		sizeof (XBufDescriptor) + ALIGNMENT;
	lp->desc_space_v = dma_alloc_coherent(NULL,
		lp->desc_space_size, &lp->desc_space_handle, GFP_KERNEL);
	if (lp->desc_space_v == NULL) {
		return -ENOMEM;
	}
	memset(lp->desc_space_v, 0, lp->desc_space_size);

	/* Allocate memory for the Tx buffers */
	lp->tx_buffs_size =  SGDMA_DEFAULT_SEND_DESC *
		(XEM_MAX_FRAME_SIZE + SGDMA_ALIGNMENT);
	lp->tx_bufs_v = kmalloc(lp->tx_buffs_size, GFP_KERNEL);
	lp->tx_bufs_offset = SGDMA_BUF_ALIGN(lp->tx_bufs_v);
	if (lp->tx_bufs_v == NULL)
		goto error_free_descs;

	atomic_set(&lp->tx_descs_avail, SGDMA_DEFAULT_SEND_DESC);

	/* Add pointers to descriptor space to the driver */
	align = BUFFER_ALIGN(lp->desc_space_handle);
	p_v = (u8 *)(lp->desc_space_v) + align;
	p_bus = (u8 *)(lp->desc_space_handle) + align;
	XEmac_SetSgRecvSpace(&lp->Emac, p_v,
			     SGDMA_DEFAULT_RECV_DESC * sizeof (XBufDescriptor),
			     p_bus);
	p_v = (u8 *)p_v + SGDMA_DEFAULT_RECV_DESC * sizeof (XBufDescriptor);
	p_bus = (u8 *)p_bus + SGDMA_DEFAULT_RECV_DESC * sizeof (XBufDescriptor);
	XEmac_SetSgSendSpace(&lp->Emac, p_v,
			     SGDMA_DEFAULT_SEND_DESC * sizeof (XBufDescriptor),
			     p_bus);

	/* Allocate skb's and give them to the dma engine */
	for (i = 0; i < SGDMA_DEFAULT_RECV_DESC; i++) {
		struct sk_buff *skb;
		XBufDescriptor bd;
		int result;
		u32 skb_bus_addr;

		skb = dev_alloc_skb(XEM_MAX_FRAME_SIZE + SGDMA_ALIGNMENT);
		if (skb == NULL)
			goto error_free_skbs;

		align = SGDMA_BUF_ALIGN(skb->data);
		if (align)
			skb_reserve(skb, align);

		skb_bus_addr = (u32) dma_map_single(NULL, skb->data,
						    XEM_MAX_FRAME_SIZE,
						    DMA_FROM_DEVICE);

		/*
		 * Initialize descriptor, and set buffer address.
		 * Buffer length gets max frame size
		 */
		XBufDescriptor_Initialize(&bd);
/// Not really used IMO		XBufDescriptor_Lock(&bd);
		XBufDescriptor_SetDestAddress(&bd, skb_bus_addr);
		XBufDescriptor_SetLength(&bd, XEM_MAX_FRAME_SIZE);
		XBufDescriptor_SetId(&bd, skb);

		/*
		 * Pass descriptor with the attached buffer to the driver and
		 * let it make it ready for frame reception
		 */
		result = XEmac_SgRecv(&lp->Emac, &bd);
		if (result != XST_SUCCESS)
			goto error_free_skbs;
	}

	return 0;	/* success */

error_free_skbs:
	free_rx_skbs(ndev);
	kfree(lp->tx_bufs_v);
error_free_descs:
	dma_free_coherent(NULL, lp->desc_space_size, lp->desc_space_v,
			  lp->desc_space_handle);

	return -ENOMEM;
}

void SgRecvHandler(void *CallbackRef, XBufDescriptor * BdPtr, u32 NumBds)
{
	struct net_device *ndev = (struct net_device *) CallbackRef;
	struct net_local *lp = netdev_priv(ndev);
	struct sk_buff *skb, *new_skb;
	u32 len;
	XBufDescriptor *cur_bd;
	u32 skb_bus_addr, align;
	int result;

	while (NumBds != 0) {
		--NumBds;

		skb = (struct sk_buff *) XBufDescriptor_GetId(BdPtr);
                len = XBufDescriptor_GetLength(BdPtr);
		len -= 4; /* crop FCS (the last four bytes) */

		cur_bd = BdPtr;
		BdPtr = P_TO_V(&lp->Emac.RecvChannel,
			       XBufDescriptor_GetNextPtr(cur_bd));

		/*
		 * The assumption here is that the DestAddress is *not*
		 * written back into the BufferDescriptor after transfer
		 * is completed.
		 */
		dma_unmap_single(NULL, XBufDescriptor_GetDestAddress(cur_bd),
				 XEM_MAX_FRAME_SIZE, DMA_FROM_DEVICE);

		/* Replace skb with a new one */
		new_skb = dev_alloc_skb(XEM_MAX_FRAME_SIZE + SGDMA_ALIGNMENT);
                if (new_skb == 0) {
                        printk("SgRecvHandler: no mem for new_skb\n");
                        return;
                }

		/* Align the new_skb->data */
		align = SGDMA_BUF_ALIGN(new_skb->data);
		if (align)
			skb_reserve(new_skb, align);

		skb_bus_addr = (u32) dma_map_single(NULL, new_skb->data,
						    XEM_MAX_FRAME_SIZE,
						    DMA_FROM_DEVICE);

		/*
		 * Initialize descriptor, and set buffer address.
		 * Buffer length gets max frame size
		 */
		XBufDescriptor_SetDestAddress(cur_bd, skb_bus_addr);
		XBufDescriptor_SetLength(cur_bd, XEM_MAX_FRAME_SIZE);
		XBufDescriptor_SetId(cur_bd, new_skb);
/// Not really used IMO		XBufDescriptor_Unlock(cur_bd);

                /* Give the descriptor back to the driver */
                result = XEmac_SgRecv(&lp->Emac, cur_bd);
                if (result != XST_SUCCESS) {
                        printk("SgRecvHandler: SgRecv unsuccessful\n");
                        return;
                }

                /* back to the original skb */
		skb_put(skb, len);	/* Tell the skb how much data we got. */
		skb->dev = ndev;	/* Fill out required meta-data. */
		skb->protocol = eth_type_trans(skb, ndev);
        	skb->ip_summed = CHECKSUM_NONE;

		lp->net_stats.rx_packets++;
		lp->net_stats.rx_bytes += len;

		netif_rx(skb);		/* Send the packet upstream. */
	}
}

/*
 * This function is protected from concurrent calls by a spinlock (xmit_lock)
 * in the net_device structure, and this is enough to protect lp->tx_bufs_offset
 * manipulations, and to prevent from concurrent XEmac_SgSend() calls. */
int xenet_start_xmit_sgdma(struct sk_buff *skb, struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	XBufDescriptor bd;
	u32 data_len;
	u32 data_bus_addr;
	u8 *data_v;
	int result;

	data_len = skb->len;
	data_v = lp->tx_bufs_v + lp->tx_bufs_offset;

	lp->tx_bufs_offset += data_len + SGDMA_BUF_ALIGN(data_len);
	if (lp->tx_bufs_offset + XEM_MAX_FRAME_SIZE > lp->tx_buffs_size) {
		lp->tx_bufs_offset = SGDMA_BUF_ALIGN(lp->tx_bufs_v);
	}

	/*
	 * skb->ip_summed is expected to be CHECKSUM_NONE as the driver doesn't
	 * advertize any checksum calculation capabilities.
	 */
	memcpy(data_v, skb->data, data_len);
        dev_kfree_skb(skb);

	data_bus_addr = dma_map_single(NULL, data_v, data_len, DMA_TO_DEVICE);

	/*
	 * lock the buffer descriptor to prevent lower layers from reusing
	 * it before the adapter has a chance to deallocate the buffer
	 * attached to it. The adapter will unlock it in the callback function
	 * that handles confirmation of transmits.
	 */
	XBufDescriptor_Initialize(&bd);
	XBufDescriptor_Lock(&bd);
	XBufDescriptor_SetSrcAddress(&bd, data_bus_addr);
	XBufDescriptor_SetLength(&bd, data_len);
	XBufDescriptor_SetLast(&bd);

	result = XEmac_SgSend(&lp->Emac, &bd, XEM_SGDMA_NODELAY);
	if (result != XST_SUCCESS) {
		lp->net_stats.tx_dropped++;
		printk(KERN_ERR
		       "%s: ERROR, could not send transmit buffer (%d).\n",
		       ndev->name, result);
		/*
		 * We should never get here in the first place, but
		 * for some reason the kernel doesn't like -EBUSY here,
		 * so just return 0 and let the stack handle dropped packets.
		 */
		return 0;
	}

	if (atomic_dec_and_test(&lp->tx_descs_avail)) {
		netif_stop_queue(ndev);
        }

	ndev->trans_start = jiffies;

	return 0;
}

void SgSendHandler(void *CallbackRef, XBufDescriptor * BdPtr, u32 NumBds)
{
	struct net_device *ndev = (struct net_device *) CallbackRef;
	struct net_local *lp = netdev_priv(ndev);
	u32 len;
	XBufDescriptor *cur_bd;

	atomic_add(NumBds, &lp->tx_descs_avail);
	while(NumBds-- != 0) {
		len = XBufDescriptor_GetLength(BdPtr);
                dma_unmap_single(NULL,
				 (u32) XBufDescriptor_GetSrcAddress(BdPtr),
				 len, DMA_TO_DEVICE);

                lp->net_stats.tx_bytes += len;
                lp->net_stats.tx_packets++;

		cur_bd = BdPtr;
		BdPtr = P_TO_V(&lp->Emac.SendChannel,
			       XBufDescriptor_GetNextPtr(BdPtr));
                XBufDescriptor_Unlock(cur_bd);
	}
	netif_wake_queue(ndev);
}
