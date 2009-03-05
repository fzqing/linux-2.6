/*
 * drivers/net/xilinx_emac/adapter_fifo.c
 *
 * Xilinx Ethernet Adapter component: transmission and receiption in FIFO mode.
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

#include "adapter.h"

static struct sk_buff * align_skb_data(struct sk_buff *skb)
{
	struct sk_buff *new_skb;
	unsigned int len, align;

	if (BUFFER_ALIGN(skb->data) == 0)
		return skb;

	len = skb->len;
	/*
	 * The packet FIFO requires the buffers to be 32 or 64 bit aligned.
	 * The sk_buff data is not aligned, so we have to do this copy.
	 * As you probably well know, this is not optimal.
	 */
	if (!(new_skb = dev_alloc_skb(len + ALIGNMENT))) {
		return NULL;
	}

	/*
	 * A new skb should have the data word aligned, but this is
	 * not enough for PLB EMAC. Calculate how many bytes
	 * we should reserve to get the data to start on a proper
	 * boundary.  */
	align = BUFFER_ALIGN(new_skb->data);
	if (align)
		skb_reserve(new_skb, align);

	/* Copy the data from the original skb to the new one. */
	skb_put(new_skb, len);
	memcpy(new_skb->data, skb->data, len);

	return new_skb;
}

/*
 * The callback function for completed frames sent in FIFO mode.
 */
void FifoSendHandler(void *CallbackRef)
{
	struct net_device *ndev = (struct net_device *) CallbackRef;
	struct net_local *lp = netdev_priv(ndev);
	struct sk_buff *tskb;
	unsigned long flags;
	int retval;

	/* Make sure that the timeout handler and we don't both free the skb. */
	spin_lock_irqsave(&lp->skb_lock, flags);
	tskb = lp->saved_skb;
	lp->saved_skb = NULL;
	spin_unlock_irqrestore(&lp->skb_lock, flags);

	if (tskb) {
		retval = XEmac_FifoSend(&lp->Emac, (u8 *) tskb->data,
					tskb->len);
		if (retval != XST_SUCCESS) {
			lp->net_stats.tx_errors++;
			printk(KERN_ERR
			       "%s: Could not transmit buffer (error %d).\n",
			       ndev->name, retval);
		}
		dev_kfree_skb(tskb);
		netif_wake_queue(ndev);
	}
}

int xenet_start_xmit_fifo(struct sk_buff *skb, struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	struct sk_buff *new_skb;
	unsigned int len;
	int retval;

	len = skb->len;

	new_skb = align_skb_data(skb); /* if new_skb != skb, new_skb has been
					* allocated */

	if (!new_skb) {
		/* We couldn't get the new, "aligned" skb. */
		lp->net_stats.tx_dropped++;
		printk(KERN_ERR "%s: Could not allocate transmit buffer.\n",
		       ndev->name);
		netif_wake_queue(ndev);
		return -EBUSY;
	}

	retval = XEmac_FifoSend(&lp->Emac, (u8 *) new_skb->data, len);
	switch (retval) {
	case XST_FIFO_NO_ROOM:
	case XST_PFIFO_NO_ROOM:
		netif_stop_queue(ndev);
		spin_lock_irq(&lp->skb_lock);
		if (unlikely(lp->saved_skb)) {
			spin_unlock_irq(&lp->skb_lock);
			/* get rid of the aligned copy. */
			if (new_skb != skb) dev_kfree_skb(new_skb);
			lp->net_stats.tx_errors++;
			printk(KERN_ERR "%s: Couldn't transmit buffer "
			       "(XST_%sFIFO_NO_ROOM).\n",
			       ndev->name,
			       (retval == XST_FIFO_NO_ROOM) ? "" : "P");
			return -EIO;
		}
		lp->saved_skb = new_skb;
		spin_unlock_irq(&lp->skb_lock);
		/* get rid of the unaligned skb. */
		if (new_skb != skb) dev_kfree_skb(skb);
		break;

	case XST_SUCCESS:
		/* get rid of the new and the original skbs. */
		if (new_skb != skb) dev_kfree_skb(skb);
		dev_kfree_skb(new_skb);
		/*
		 * We have to update the counters early as we can't
		 * track the length of every packet pending transmission.
		 */
		lp->net_stats.tx_bytes += len;
		lp->net_stats.tx_packets++;
		break;

	default:	/* transmission error */
		/* get rid of the aligned copy. */
		if (new_skb != skb) dev_kfree_skb(new_skb);
		lp->net_stats.tx_errors++;
		printk(KERN_ERR "%s: Could not transmit buffer (%d).\n",
		       ndev->name, retval);
		return -EIO;
	}

	/* reset the transmission timeout */
	ndev->trans_start = jiffies;

	return 0;
}

/*
 * The callback function for frames received when in FIFO mode.
 */

void FifoRecvHandler(void *CallbackRef)
{
	struct net_device *ndev = (struct net_device *) CallbackRef;
	struct net_local *lp = netdev_priv(ndev);
	struct sk_buff *skb;
	unsigned int align;
	u32 len;
	XStatus Result;

	/*
	 * The OS independent Xilinx EMAC code does not provide a
	 * function to get the length of an incoming packet and a
	 * separate call to actually get the packet data.  It does this
	 * because they didn't add any code to keep the hardware's
	 * receive length and data FIFOs in sync.  Instead, they require
	 * that you send a maximal length buffer so that they can read
	 * the length and data FIFOs in a single chunk of code so that
	 * they can't get out of sync.  So, we need to allocate an skb
	 * that can hold a maximal sized packet.  The OS independent
	 * code needs to see the data 32- or 64-bit aligned, so we tack on
	 * some extra bytes just in case we need to do an skb_reserve to get
	 * it that way.
	 */
	len = XEM_MAX_FRAME_SIZE; /* jumbo and VLAN packets could be bigger */
	if (!(skb = dev_alloc_skb(len + ALIGNMENT))) {
		/* Couldn't get memory. */
		lp->net_stats.rx_dropped++;
		printk(KERN_ERR "%s: Could not allocate receive buffer.\n",
		       ndev->name);
		return;
	}

	/*
	 * A new skb should have the data word aligned, but this is
	 * not enough for PLB EMAC. Calculate how many bytes
	 * we should reserve to get the data to start on a proper
	 * boundary.  */
	align = BUFFER_ALIGN(skb->data);
	if (align)
		skb_reserve(skb, align);

	Result = XEmac_FifoRecv(&lp->Emac, (u8 *) skb->data, &len);
	if (Result != XST_SUCCESS) {
		int need_reset = status_requires_reset(Result);

		lp->net_stats.rx_errors++;
		dev_kfree_skb(skb);

		printk(KERN_ERR "%s: Could not receive buffer, error=%d%s\n",
		       ndev->name, Result,
		       need_reset ? ", resetting device." : "");
		if (need_reset)
			xenet_reset(ndev, UNKNOWN_DUPLEX);
		return;
	}

	len -= 4;		/* crop FCS (the last four bytes) */
	skb_put(skb, len);	/* Tell the skb how much data we got. */
	skb->dev = ndev;	/* Fill out required meta-data. */
	skb->protocol = eth_type_trans(skb, ndev);
        skb->ip_summed = CHECKSUM_NONE;

	lp->net_stats.rx_packets++;
	lp->net_stats.rx_bytes += len;

	netif_rx(skb);		/* Send the packet upstream. */
}
