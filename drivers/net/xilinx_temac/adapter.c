/*
 * adapter.c
 *
 * Xilinx Ethernet Adapter component to interface XTemac component to Linux
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2006 (c) MontaVista, Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2.1. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * This driver is a bit unusual in that it is composed of two logical
 * parts where one part is the OS independent code and the other part is
 * the OS dependent code.  Xilinx provides their drivers split in this
 * fashion.  This file represents the Linux OS dependent part known as
 * the Linux adapter.  The other files in this directory are the OS
 * independent files as provided by Xilinx with no changes made to them.
 * The names exported by those files begin with XTemac_.  All functions
 * in this file that are called by Linux have names that begin with
 * xtenet_.  The functions in this file that have Handler in their name
 * are registered as callbacks with the underlying Xilinx OS independent
 * layer.  Any other functions are static helper functions.
 *
 * Current Xilinx Trimode EMAC device version this adapter supports does
 * not have MII/GMII interface, and this adapter assumes that the PHY
 * connecting the Trimode EMAC to the ethernet bus runs at 1Gbps, has
 * Full duplex turned on and PHY link is always on.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/xilinx_devices.h>
#include <asm/io.h>
#include <linux/ethtool.h>

#include "xbasic_types.h"
#include "xtemac.h"
#include "xipif_v1_23_b.h"
#include "xpacket_fifo_v2_00_a.h"
#include "xdmav2.h"
#include "xdmabdv2.h"

/*
 * Default SEND and RECV buffer descriptors (BD) numbers.
 * BD Space needed is (XTE_DFT_SEND_DESC+XTE_DFT_RECV_DESC)*Sizeof(XDmaBdV2).
 * Each XDmaBdV2 instance currently takes 40 bytes.
 */
#undef XTE_DFT_SEND_DESC
#define XTE_DFT_SEND_DESC   256
#undef XTE_DFT_RECV_DESC
#define XTE_DFT_RECV_DESC   256

/* Must be shorter than length of ethtool_drvinfo.driver field to fit */
#define DRIVER_NAME		"xilinx_temac"
#define DRIVER_VERSION		"1.0"
#define DRIVER_DESCRIPTION	"Xilinx Tri-Mode Eth MAC driver"

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL");

#define TX_TIMEOUT   (3*HZ)	/* Transmission timeout is 3 seconds. */

/*
 * When Xilinx TEMAC is configured to use the TX Data Realignment Engine (DRE),
 * alignment restrictions are as follows:
 *   - SGDMA transmit buffers can be aligned on any boundary, but receive buffers
 *     must be aligned on a 8-byte boundary.
 *
 * Without TX DRE, buffer alignment restrictions are as follows:
 *   - SGDMA transmit and receive buffers must be aligned on a 8-byte boundary
 *
 * There are no alignment restrictions when using XTemac_FifoRead() and
 * XTemac_FifoWrite().
 *
 * For simplicity, we always align to 8 bytes.
 */
#define ALIGNMENT_SEND  8
#define ALIGNMENT_RECV  8

/* SGDMA buffer descriptors must be aligned on a 8-byte boundary. */
#define ALIGNMENT_BD    8

/* BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment. */
#define BUFFER_ALIGNSEND(adr) ((ALIGNMENT_SEND - ((u32) adr)) % ALIGNMENT_SEND)
#define BUFFER_ALIGNRECV(adr) ((ALIGNMENT_RECV - ((u32) adr)) % ALIGNMENT_RECV)

/* Default TX/RX Threshold and waitbound values for SGDMA mode */
#define DFT_TX_THRESHOLD  1
#define DFT_TX_WAITBOUND  0
#define DFT_RX_THRESHOLD  1
#define DFT_RX_WAITBOUND  0

/* Is Buffer Descriptors in BRAM ? */
#define BD_IN_BRAM        0

/* Driver configuration  */
#define XTEMAC_MAX_DEVICE_ID 4
static XTemac_Config *xtemac_cfgs[XTEMAC_MAX_DEVICE_ID];

/*
 * Lookup the device configuration based on the unique device ID.
 * Prototyped in xtemac.h.
 */
inline XTemac_Config *XTemac_LookupConfig(u16 DeviceId)
{
	return (DeviceId < XTEMAC_MAX_DEVICE_ID) ? xtemac_cfgs[DeviceId] : NULL;
}

/*
 * Our private per device data.  When a net_device is allocated we will
 * ask for enough extra space for this.
 */
struct net_local {
	struct list_head rcv;
	struct list_head xmit;

	struct net_device_stats stats;	/* Statistics for this device */
	struct net_device *ndev;	/* this device */

	u32 index;		/* Which interface is this */
	XInterruptHandler Isr;	/* Pointer to the XTemac ISR routine */

	/* The underlying OS independent code needs space as well.  A
	 * pointer to the following XTemac structure will be passed to
	 * any XTemac_ function that requires it.  However, we treat the
	 * data as an opaque object in this file (meaning that we never
	 * reference any of the fields inside of the structure). */
	XTemac Emac;
	/*
	 * Currently XEmac has a pointer to XEmac_Config structure.
	 * This pointer is used not only by XEmac_Initialize(), but
	 * by XEmac_mIsSgDma() et al as well. For this reason the
	 * XEmac_Config structure must exist as long as the device
	 * is alive. Otherwise we could drop this structure right
	 * after XEmac_Initialize() returns.
	 */
	XTemac_Config Config;

	unsigned int max_frame_size;

	/* Buffer Descriptor space for both TX and RX BD ring */
	void *desc_space;	/* virtual address of BD space */
	dma_addr_t desc_space_handle;	/* physical address of BD space */
	int desc_space_size;	/* size of BD space */

	/* buffer for one skb in case no room is available for transmission */
	struct sk_buff *deferred_skb;

};

/* for exclusion of all program flows (processes, ISRs and BHs) */
DEFINE_SPINLOCK(XTE_spinlock);

/* Helper function to determine if a given XTemac error warrants a reset. */
extern inline int status_requires_reset(XStatus s)
{
	return (s == XST_FIFO_ERROR ||
		s == XST_PFIFO_DEADLOCK ||
		s == XST_DMA_ERROR || s == XST_IPIF_ERROR);
}

/* BH statics */
static LIST_HEAD(receivedQueue);
static DEFINE_SPINLOCK(receivedQueueSpin);
static LIST_HEAD(sentQueue);
static DEFINE_SPINLOCK(sentQueueSpin);

/* SAATODO: This function will be moved into the Xilinx code. */
/*****************************************************************************/
/**
 * Check if the given TEMAC instance is started or not
 *
 * @param InstancePtr is a pointer to the instance to be worked on.
 *
 * @return
 *
 * - TRUE if the device is already started.
 * - FALSE if the device is stopped
 *
 * @note
 *
 * None
 *
 ******************************************************************************/
int XTemac_IsStarted(XTemac * InstancePtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/* Be sure device has been stopped */
	return (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED);
}

/*
 * This function sets up MAC's speed to 1Gbps as this is the only mode
 * supported by the current h/w
 */
void set_mac_speed(struct net_local *lp)
{
	struct net_device *ndev = lp->ndev;

	XTemac_SetMiiLinkSpeed(&lp->Emac, 1000);
	printk(KERN_INFO "%s: XTemac speed set to 1000Mb/s\n", ndev->name);
}

/*
 * Helper function to reset the underlying hardware.  This is called
 * when we get into such deep trouble that we don't know how to handle
 * otherwise.
 */
typedef enum DUPLEX { UNKNOWN_DUPLEX, HALF_DUPLEX, FULL_DUPLEX } DUPLEX;

/*
 * This reset function should handle four different reset request types
 * from other functions. The reset request types include
 *      1. FIFO error: FifoWrite()/FifoSend()/FifoRecv()/FifoRead() fails
 *      2. DMA error: SgAlloc()/SgCommit()/SgFree() fails
 *      3. TX Timeout: Timeout occurs for a TX frame given to this adapter
 *      4. Error Status: Temac Error interrupt occurs and asks for a reset
 */

static void reset(struct net_device *dev, u32 line_num)
{
	struct net_local *lp = (struct net_local *)dev->priv;
	u16 TxThreshold, TxWaitBound, RxThreshold, RxWaitBound;
	u32 Options;
	u8 Ifg;
	static u32 reset_cnt = 0;

	printk(KERN_INFO "%s: XTemac resets (#%u) from adapter code line %d\n",
	       dev->name, ++reset_cnt, line_num);

	/* Shouldn't really be necessary, but shouldn't hurt. */
	netif_stop_queue(dev);

	/* Stop device */
	XTemac_Stop(&lp->Emac);

	/*
	 * XTemac_Reset puts the device back to the default state.  We need
	 * to save all the settings we don't already know, reset, restore
	 * the settings, and then restart the temac.
	 */
	Ifg = XTemac_GetIfg(&lp->Emac);
	Options = XTemac_GetOptions(&lp->Emac);
	if (XTemac_mIsSgDma(&lp->Emac)) {
		/*
		 * The following two functions will return an error if we are
		 * not doing scatter-gather DMA.  We just checked that so we
		 * can safely ignore the return values.
		 */
		XTemac_IntrSgCoalGet(&lp->Emac, XTE_RECV, &RxThreshold,
				     &RxWaitBound);
		XTemac_IntrSgCoalGet(&lp->Emac, XTE_SEND, &TxThreshold,
				     &TxWaitBound);

	}

	/* now we can reset the device */
	XTemac_Reset(&lp->Emac);

	/* Reset on TEMAC also resets PHY. Give it some time to finish
	 * negotiation before we move on */
	mdelay(2000);

	/*
	 * The following four functions will return an error if the
	 * TEMAC is already started.  We just stopped it by calling
	 * XTemac_Reset() so we can safely ignore the return values.
	 */
	XTemac_SetMacAddress(&lp->Emac, dev->dev_addr);
	XTemac_SetIfg(&lp->Emac, Ifg);
	Options &= ~XTE_HALF_DUPLEX_OPTION;
	XTemac_SetOptions(&lp->Emac, Options);
	XTemac_ClearOptions(&lp->Emac, ~Options);
	Options = XTemac_GetOptions(&lp->Emac);
	printk(KERN_INFO "%s: XTemac Options: 0x%x\n", dev->name, Options);

	set_mac_speed(lp);
	netif_carrier_on(dev);

	if (XTemac_mIsSgDma(&lp->Emac)) {	/* SG DMA mode */
		/*
		 * The following 2 functions will return an error if
		 * we are not doing scatter-gather DMA or if the TEMAC is
		 * already started.  We just checked that we are indeed
		 * doing scatter-gather and we just stopped the TEMAC so
		 * we can safely ignore the return values.
		 */
		XTemac_IntrSgCoalSet(&lp->Emac, XTE_RECV, RxThreshold,
				     RxWaitBound);
		XTemac_IntrSgCoalSet(&lp->Emac, XTE_SEND, TxThreshold,
				     TxWaitBound);

		/* Enable both SEND and RECV interrupts */
		XTemac_IntrSgEnable(&lp->Emac, XTE_SEND | XTE_RECV);

		/* TODO: Check if we should check sgdma using XTemac_SgCheck() here */
	} else {		/* FIFO interrupt mode */
		XTemac_IntrFifoEnable(&lp->Emac, XTE_RECV | XTE_SEND);
	}

	if (lp->deferred_skb) {
		dev_kfree_skb_any(lp->deferred_skb);
		lp->deferred_skb = NULL;
		lp->stats.tx_errors++;
	}

	/*
	 * XTemac_Start returns an error when: if configured for
	 * scatter-gather DMA and a descriptor list has not yet been created
	 * for the send or receive channel, or if no receive buffer descriptors
	 * have been initialized. Those are not happening. so ignore the
	 * returned result checking.
	 */
	XTemac_Start(&lp->Emac);

	/* We're all ready to go.  Start the queue in case it was stopped. */
	netif_wake_queue(dev);
}

/*
 * This routine is registered with the OS as the function to call when
 * the TEMAC interrupts.  It in turn, calls the Xilinx OS independent
 * interrupt function.  There are different interrupt functions for FIFO
 * and scatter-gather so we just set a pointer (Isr) into our private
 * data so we don't have to figure it out here.  The Xilinx OS
 * independent interrupt function will in turn call any callbacks that
 * we have registered for various conditions.
 */
static irqreturn_t xtenet_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct net_local *lp = (struct net_local *)dev->priv;

	/* Call it. */
	(*(lp->Isr)) (&lp->Emac);

	return IRQ_HANDLED;
}

static int xtenet_open(struct net_device *dev)
{
	struct net_local *lp;
	u32 Options;
	unsigned long flags;

	/*
	 * Just to be safe, stop TX queue and the device first. If the device is
	 * already stopped, an error will be returned. In this case, we don't
	 * really care,.
	 */
	netif_stop_queue(dev);
	spin_lock_irqsave(&XTE_spinlock, flags);
	lp = (struct net_local *)dev->priv;
	(void)XTemac_Stop(&lp->Emac);

	/* Set the MAC address each time opened. */
	if (XTemac_SetMacAddress(&lp->Emac, dev->dev_addr) != XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac could not set MAC address.\n",
		       dev->name);
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return -EIO;
	}

	/*
	 * If the device is not configured for polled mode, connect to the
	 * interrupt controller and enable interrupts.  Currently, there
	 * isn't any code to set polled mode, so this check is probably
	 * superfluous.
	 */
	Options = XTemac_GetOptions(&lp->Emac);
	Options &= ~XTE_HALF_DUPLEX_OPTION;
	Options &= ~XTE_SGEND_INT_OPTION;
	Options |= XTE_FLOW_CONTROL_OPTION;
	Options &= ~XTE_JUMBO_OPTION;
	(void)XTemac_SetOptions(&lp->Emac, Options);
	(void)XTemac_ClearOptions(&lp->Emac, ~Options);
	Options = XTemac_GetOptions(&lp->Emac);
	printk(KERN_INFO "%s: XTemac Options: 0x%x\n", dev->name, Options);

	/* Register interrupt handler */
	if ((Options & XTE_POLLED_OPTION) == 0) {
		int retval;
		/* Grab the IRQ */
		retval =
		    request_irq(dev->irq, &xtenet_interrupt, 0, dev->name, dev);
		if (retval) {
			printk(KERN_ERR
			       "%s: XTemac could not allocate interrupt %d.\n",
			       dev->name, dev->irq);
			spin_unlock_irqrestore(&XTE_spinlock, flags);
			return retval;
		}
	}

	set_mac_speed(lp);
	netif_carrier_on(dev);

	INIT_LIST_HEAD(&(lp->rcv));
	INIT_LIST_HEAD(&(lp->xmit));

	/* Enable interrupts if not in polled mode */
	if ((Options & XTE_POLLED_OPTION) == 0) {
		if (!XTemac_mIsSgDma(&lp->Emac))	/*fifo direct interrupt driver mode */
			XTemac_IntrFifoEnable(&lp->Emac, XTE_RECV | XTE_SEND);
		else		/* SG DMA mode */
			XTemac_IntrSgEnable(&lp->Emac, XTE_SEND | XTE_RECV);
	}

	/* Start TEMAC device */
	if (XTemac_Start(&lp->Emac) != XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac could not start device.\n",
		       dev->name);
		free_irq(dev->irq, dev);
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	if (XTemac_mIsSgDma(&lp->Emac)) {
		u16 threshold_s, timer_s, threshold_r, timer_r;

		XTemac_IntrSgCoalGet(&lp->Emac, XTE_SEND, &threshold_s,
				     &timer_s);
		XTemac_IntrSgCoalGet(&lp->Emac, XTE_RECV, &threshold_r,
				     &timer_r);
		printk(KERN_INFO
		       "%s: XTemac Send Threshold = %d, Receive Threshold = %d\n",
		       dev->name, threshold_s, threshold_r);
		printk(KERN_INFO
		       "%s: XTemac Send Wait bound = %d, Receive Wait bound = %d\n",
		       dev->name, timer_s, timer_r);
	}

	/* We're ready to go. */
	netif_start_queue(dev);

	return 0;
}

static int xtenet_close(struct net_device *dev)
{
	struct net_local *lp;
	unsigned long flags, flags_reset;

	spin_lock_irqsave(&XTE_spinlock, flags_reset);
	lp = (struct net_local *)dev->priv;

	/* Stop Send queue */
	netif_stop_queue(dev);

	/* Now we could stop the device */
	XTemac_Stop(&lp->Emac);

	/*
	 * If not in polled mode, free the interrupt.  Currently, there
	 * isn't any code to set polled mode, so this check is probably
	 * superfluous.
	 */
	if ((XTemac_GetOptions(&lp->Emac) & XTE_POLLED_OPTION) == 0)
		free_irq(dev->irq, dev);

	spin_unlock_irqrestore(&XTE_spinlock, flags_reset);

	spin_lock_irqsave(&receivedQueueSpin, flags);
	list_del(&(lp->rcv));
	spin_unlock_irqrestore(&receivedQueueSpin, flags);

	spin_lock_irqsave(&sentQueueSpin, flags);
	list_del(&(lp->xmit));
	spin_unlock_irqrestore(&sentQueueSpin, flags);

	return 0;
}

static struct net_device_stats *xtenet_get_stats(struct net_device *dev)
{
	struct net_local *lp = (struct net_local *)dev->priv;
	return &lp->stats;
}

static int xtenet_change_mtu(struct net_device *dev, int new_mtu)
{
	int head_size = XTE_HDR_SIZE;
	struct net_local *lp = (struct net_local *)dev->priv;
	int max_frame = new_mtu + head_size + XTE_TRL_SIZE;
	int min_frame = 1 + head_size + XTE_TRL_SIZE;

	if ((max_frame < min_frame) || (max_frame > lp->max_frame_size))
		return -EINVAL;

	dev->mtu = new_mtu;	/* change mtu in net_device structure */
	return 0;
}

static int xtenet_FifoSend(struct sk_buff *skb, struct net_device *dev)
{
	struct net_local *lp;
	unsigned int len;
	XStatus result;
	unsigned long flags, fifo_free_bytes;

	/* The following lock is used to protect GetFreeBytes, FifoWrite
	 * and FifoSend sequence which could happen from FifoSendHandler
	 * or other processor in SMP case.
	 */
	spin_lock_irqsave(&XTE_spinlock, flags);
	lp = (struct net_local *)dev->priv;
	len = skb->len;

	fifo_free_bytes = XTemac_FifoGetFreeBytes(&lp->Emac, XTE_SEND);
	if (fifo_free_bytes < len) {
		netif_stop_queue(dev);	/* stop send queue */
		lp->deferred_skb = skb;	/* buffer the sk_buffer and will send
					   it in interrupt context */
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return 0;
	}

	/* Write frame data to FIFO */
	result =
	    XTemac_FifoWrite(&lp->Emac, (void *)skb->data, len,
			     XTE_END_OF_PACKET);
	if (result != XST_SUCCESS) {
		reset(dev, __LINE__);
		lp->stats.tx_errors++;
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return -EIO;
	}

	/* Initiate transmit */
	if ((result = XTemac_FifoSend(&lp->Emac, len)) != XST_SUCCESS) {
		reset(dev, __LINE__);
		lp->stats.tx_errors++;
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return -EIO;
	}
	lp->stats.tx_bytes += len;
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	dev_kfree_skb(skb);	/* free skb */
	dev->trans_start = jiffies;
	return 0;
}

/* Callback function for completed frames sent in FIFO interrupt driven mode */
static void FifoSendHandler(void *CallbackRef)
{
	struct net_device *dev;
	struct net_local *lp;
	XStatus result;
	struct sk_buff *skb;

	spin_lock(&XTE_spinlock);
	dev = (struct net_device *)CallbackRef;
	lp = (struct net_local *)dev->priv;
	lp->stats.tx_packets++;

	/*Send out the deferred skb and wake up send queue if a deferred skb exists */
	if (lp->deferred_skb) {

		skb = lp->deferred_skb;
		/* If no room for the deferred packet, return */
		if (XTemac_FifoGetFreeBytes(&lp->Emac, XTE_SEND) < skb->len) {
			spin_unlock(&XTE_spinlock);
			return;
		}

		/* Write frame data to FIFO */
		result = XTemac_FifoWrite(&lp->Emac, (void *)skb->data,
					  skb->len, XTE_END_OF_PACKET);
		if (result != XST_SUCCESS) {
			reset(dev, __LINE__);
			lp->stats.tx_errors++;
			spin_unlock(&XTE_spinlock);
			return;
		}

		/* Initiate transmit */
		if ((result =
		     XTemac_FifoSend(&lp->Emac, skb->len)) != XST_SUCCESS) {
			reset(dev, __LINE__);
			lp->stats.tx_errors++;
			spin_unlock(&XTE_spinlock);
			return;
		}

		dev_kfree_skb_irq(skb);
		lp->deferred_skb = NULL;
		lp->stats.tx_bytes += skb->len;
		dev->trans_start = jiffies;
		netif_wake_queue(dev);	/* wake up send queue */
	}
	spin_unlock(&XTE_spinlock);
}

/* The send function for frames sent in SGDMA mode and TEMAC has TX DRE. */
static int xtenet_SgSend(struct sk_buff *skb, struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	int nr_frags = skb_shinfo(skb)->nr_frags;
	XDmaBdV2 *bd_1st, *bd_cur;
	dma_addr_t frags_phys[nr_frags + 1];
	int frags_len[nr_frags + 1];
	int i, result;

	if (skb->ip_summed == CHECKSUM_HW) {
		skb_checksum_help(skb, 0);
	}

	frags_len[0] = skb->len;
	if (nr_frags != 0) {
		for (i = 0; i < nr_frags; i++) {
			struct skb_frag_struct *frag =
						&skb_shinfo(skb)->frags[i];
			void * p = (char *)page_address(frag->page)
						+ frag->page_offset;

			frags_phys[i+1] = dma_map_single(NULL, p, frag->size,
						       DMA_TO_DEVICE);
			frags_len[i+1] = frag->size;
		}
		frags_len[0] -= skb->data_len;
	}
	frags_phys[0] = dma_map_single(NULL, skb->data, frags_len[0],
				       DMA_TO_DEVICE);

	/*
	 * The following spin_lock protects SgAlloc, SgCommit sequence,
	 * which also exists in SgSendHandlerBH Bottom Half,
	 * or triggered by other processor in SMP case.
	 */
	spin_lock_bh(&XTE_spinlock);

	/* Allocate TxBDs */
	result = XTemac_SgAlloc(&lp->Emac, XTE_SEND, nr_frags + 1, &bd_1st);
	if (result != XST_SUCCESS) {
		netif_stop_queue(ndev); /* stop send queue */
		lp->deferred_skb = skb; /* buffer the skb to send it
					 later from interrupt context */
		goto out;
	}

	/* Setup TxBD */
	for (i = 0, bd_cur = bd_1st; /* always */; i++) {
		XDmaBdV2_mSetSrcAddr(bd_cur, frags_phys[i]);
		XDmaBdV2_mSetLength(bd_cur, frags_len[i]);
		if (i < nr_frags) {
			XDmaBdV2_mClearLast(bd_cur);
			bd_cur = XTemac_mSgSendBdNext(&lp->Emac, bd_cur);
		} else {
			XDmaBdV2_mSetLast(bd_cur);
			XDmaBdV2_mSetId(bd_cur, skb);
			break;
		}
	}

	/* Enqueue to HW */
	result = XTemac_SgCommit(&lp->Emac, XTE_SEND, nr_frags + 1, bd_1st);
	if (result != XST_SUCCESS) {
		netif_stop_queue(ndev);	/* stop send queue */
		dev_kfree_skb(skb);
		XDmaBdV2_mSetId(bd_cur, NULL);
		lp->stats.tx_dropped++;
		printk(KERN_ERR
		       "%s: XTemac could not send commit TX buffer descriptor (%d).\n",
		       ndev->name, result);
		reset(ndev, __LINE__);
		goto out;
	}

	ndev->trans_start = jiffies;
	spin_unlock_bh(&XTE_spinlock);
	return 0;

out:
	for (i = 0; i < nr_frags + 1; i++) {
		dma_unmap_single(NULL, frags_phys[i],
				 frags_len[i], DMA_TO_DEVICE);
	}
	spin_unlock_bh(&XTE_spinlock);
	return 0;
}

/* The send function for frames sent in SGDMA mode (and no TX DRE is in TEMAC). */
static int xtenet_SgSend_NoDRE(struct sk_buff *skb, struct net_device *ndev)
{
	/* we must be long aligned for dma h/w as no TX DRE is in TEMAC core */
	if (BUFFER_ALIGNSEND(skb->data)) {
		printk(KERN_ERR
		       "%s: NO DRE in Xilinx TEMAC core and data unaligned by (%d) bytes\n",
		       ndev->name, BUFFER_ALIGNSEND(skb->data));
		return -EIO;
	}

	return xtenet_SgSend(skb, ndev);
}

/* The callback function for completed frames sent in SGDMA mode. */
static void SgSendHandlerBH(unsigned long p);
static void SgRecvHandlerBH(unsigned long p);

DECLARE_TASKLET(SgSendBH, SgSendHandlerBH, 0);
DECLARE_TASKLET(SgRecvBH, SgRecvHandlerBH, 0);

static void SgSendHandlerBH(unsigned long p)
{
	struct net_device *dev;
	struct net_local *lp;
	XDmaBdV2 *BdPtr, *BdCurPtr;
	u32 len;
	unsigned long flags;
	struct sk_buff *skb;
	dma_addr_t skb_dma_addr;
	XStatus result;
	unsigned int bd_processed, bd_processed_save;

	while (1) {
		spin_lock_irqsave(&sentQueueSpin, flags);
		if (list_empty(&sentQueue)) {
			spin_unlock_irqrestore(&sentQueueSpin, flags);
			break;
		}

		lp = list_entry(sentQueue.next, struct net_local, xmit);
		list_del_init(&(lp->xmit));
		spin_unlock_irqrestore(&sentQueueSpin, flags);

		spin_lock(&XTE_spinlock);
		dev = lp->ndev;
		bd_processed_save = 0;
		while ((bd_processed =
			XTemac_SgGetProcessed(&lp->Emac, XTE_SEND,
					      XTE_DFT_SEND_DESC, &BdPtr)) > 0) {

			bd_processed_save = bd_processed;
			BdCurPtr = BdPtr;
			do {
				len = XDmaBdV2_mGetProcessedLength(BdCurPtr);
				skb_dma_addr =
				    (dma_addr_t) XDmaBdV2_mGetSrcAddr(BdCurPtr);
				dma_unmap_single(NULL, skb_dma_addr, len,
						 DMA_TO_DEVICE);

				/* get ptr to skb */
				skb =
				    (struct sk_buff *)XDmaBdV2_mGetId(BdCurPtr);
				if (skb)
					dev_kfree_skb_any(skb);

				/* reset BD id */
				XDmaBdV2_mSetId(BdCurPtr, NULL);

				result = XDmaBdV2_mGetDeviceStatus(BdCurPtr);
				if (result & XTE_TSR_ERROR_MASK) {
					lp->stats.tx_errors++;
				} else {
					lp->stats.tx_bytes += len;
					if (skb) lp->stats.tx_packets++;
				}

				BdCurPtr =
				    XTemac_mSgSendBdNext(&lp->Emac, BdCurPtr);
				bd_processed--;
			} while (bd_processed > 0);

			result =
			    XTemac_SgFree(&lp->Emac, XTE_SEND,
					  bd_processed_save, BdPtr);
			if (result != XST_SUCCESS) {
				printk(KERN_ERR
				       "%s: XTemac _SgFree() error %d.\n",
				       dev->name, result);
				reset(dev, __LINE__);
				spin_unlock(&XTE_spinlock);
				return;
			}
		}
		XTemac_IntrSgEnable(&lp->Emac, XTE_SEND);

		/* Send out the deferred skb if it exists */
		if (bd_processed_save && (lp->deferred_skb)) {
			XDmaBdV2 *bd_1st, *bd_cur;
			int nr_frags = skb_shinfo(lp->deferred_skb)->nr_frags;
			dma_addr_t frags_phys[nr_frags + 1];
			int frags_len[nr_frags + 1];
			int i;

			skb = lp->deferred_skb;
			lp->deferred_skb = NULL;

			/* Allocate TxBDs */
			result = XTemac_SgAlloc(&lp->Emac, XTE_SEND,
						nr_frags + 1, &bd_1st);
			if (unlikely(result != XST_SUCCESS)) {
				netif_stop_queue(dev); /* stop send queue */
				dev_kfree_skb_any(skb);
				reset(dev, __LINE__);
				spin_unlock(&XTE_spinlock);
				continue;
			}

			frags_len[0] = skb->len;
			if (nr_frags != 0) {
				for (i = 0; i < nr_frags; i++) {
					struct skb_frag_struct *frag =
						&skb_shinfo(skb)->frags[i];
					void * p = (char *)page_address(frag->page)
						+ frag->page_offset;

					frags_phys[i+1] = dma_map_single(NULL,
								 p, frag->size,
							         DMA_TO_DEVICE);
					frags_len[i+1] = frag->size;
				}
				frags_len[0] -= skb->data_len;
			}
			frags_phys[0] = dma_map_single(NULL, skb->data,
						       frags_len[0],
						       DMA_TO_DEVICE);

			/* Setup TxBD */
			for (i = 0, bd_cur = bd_1st; /* always */; i++) {
				XDmaBdV2_mSetSrcAddr(bd_cur, frags_phys[i]);
				XDmaBdV2_mSetLength(bd_cur, frags_len[i]);
				if (i < nr_frags) {
					XDmaBdV2_mClearLast(bd_cur);
					bd_cur = XTemac_mSgSendBdNext(&lp->Emac,
								      bd_cur);
				} else {
					XDmaBdV2_mSetLast(bd_cur);
					XDmaBdV2_mSetId(bd_cur, skb);
					break;
				}
			}

			/* Enqueue to HW */
			result = XTemac_SgCommit(&lp->Emac, XTE_SEND,
						 nr_frags + 1, bd_1st);
			if (unlikely(result != XST_SUCCESS)) {
				netif_stop_queue(dev);	/* stop send queue */
				dev_kfree_skb_any(skb);
				XDmaBdV2_mSetId(bd_cur, NULL);
				lp->stats.tx_dropped++;
				for (i = 0; i < nr_frags + 1; i++) {
					dma_unmap_single(NULL, frags_phys[i],
							 frags_len[i],
							 DMA_TO_DEVICE);
				}
				printk(KERN_ERR
				       "%s: XTemac could not send commit TX buffer descriptor (%d).\n",
				       dev->name, result);
				reset(dev, __LINE__);
				spin_unlock(&XTE_spinlock);
				continue;
			}

			dev->trans_start = jiffies;
		}

		netif_wake_queue(dev);	/* wake up send queue */
		spin_unlock(&XTE_spinlock);
	}
}

static void SgSendHandler(void *CallBackRef)
{
	struct net_local *lp;
	struct list_head *cur_lp;

	spin_lock(&sentQueueSpin);
	lp = (struct net_local *)CallBackRef;
	list_for_each(cur_lp, &sentQueue) {
		if (cur_lp == &(lp->xmit)) {
			break;
		}
	}
	if (cur_lp != &(lp->xmit)) {
		list_add_tail(&lp->xmit, &sentQueue);
		XTemac_IntrSgDisable(&lp->Emac, XTE_SEND);
		tasklet_schedule(&SgSendBH);
	}
	spin_unlock(&sentQueueSpin);
}

static void xtenet_tx_timeout(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	unsigned long flags;

	/*
	 * Make sure that no interrupts come in that could cause reentrancy
	 * problems in reset.
	 */
	spin_lock_irqsave(&XTE_spinlock, flags);

	printk(KERN_ERR
	       "%s: XTemac exceeded transmit timeout of %lu ms.  Resetting emac.\n",
	       ndev->name, TX_TIMEOUT * 1000UL / HZ);
	lp->stats.tx_errors++;

	reset(ndev, __LINE__);

	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

/* The callback function for frames received when in FIFO mode. */

#define XTE_RX_SINK_BUFFER_SIZE 1024

static void FifoRecvHandler(void *CallbackRef)
{
	struct net_device *ndev;
	struct net_local *lp;
	struct sk_buff *skb;
	u32 len;
	XStatus Result;
	static u32 rx_buffer_sink[XTE_RX_SINK_BUFFER_SIZE / sizeof(u32)];

	spin_lock(&XTE_spinlock);
	ndev = (struct net_device *)CallbackRef;
	lp = netdev_priv(ndev);

	Result = XTemac_FifoRecv(&lp->Emac, &len);
	if (Result != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: XTemac could not read received packet length, error=%d.\n",
		       ndev->name, Result);
		lp->stats.rx_errors++;
		reset(ndev, __LINE__);
		spin_unlock(&XTE_spinlock);
		return;
	}

	if (!(skb = dev_alloc_skb(len + ALIGNMENT_RECV))) {
		/* Couldn't get memory. */
		lp->stats.rx_dropped++;
		printk(KERN_ERR
		       "%s: XTemac could not allocate receive buffer.\n",
		       ndev->name);

		/* consume data in Xilinx TEMAC RX data fifo so it is sync with RX length fifo */
		for (; len > XTE_RX_SINK_BUFFER_SIZE;
		     len -= XTE_RX_SINK_BUFFER_SIZE) {
			XTemac_FifoRead(&lp->Emac, rx_buffer_sink,
					XTE_RX_SINK_BUFFER_SIZE,
					XTE_PARTIAL_PACKET);
		}
		XTemac_FifoRead(&lp->Emac, rx_buffer_sink, len,
				XTE_END_OF_PACKET);

		spin_unlock(&XTE_spinlock);
		return;
	}

	/* Read the packet data */
	Result = XTemac_FifoRead(&lp->Emac, skb->data, len, XTE_END_OF_PACKET);
	if (Result != XST_SUCCESS) {
		lp->stats.rx_errors++;
		dev_kfree_skb_irq(skb);
		printk(KERN_ERR
		       "%s: XTemac could not receive buffer, error=%d.\n",
		       ndev->name, Result);
		reset(ndev, __LINE__);
		spin_unlock(&XTE_spinlock);
		return;
	}
	lp->stats.rx_packets++;
	lp->stats.rx_bytes += len;
	spin_unlock(&XTE_spinlock);

	skb_put(skb, len);	/* Tell the skb how much data we got. */
	skb->dev = ndev;	/* Fill out required meta-data. */
	skb->protocol = eth_type_trans(skb, ndev);
	skb->ip_summed = CHECKSUM_NONE;
	netif_rx(skb);		/* Send the packet upstream. */
}

static void SgRecvHandlerBH(unsigned long p)
{
	struct net_device *dev;
	struct net_local *lp;
	struct sk_buff *skb, *new_skb;
	u32 len, skb_baddr, new_skb_baddr;
	u32 align;
	XStatus result;
	unsigned long flags;
	XDmaBdV2 *BdPtr, *BdCurPtr;
	unsigned int bd_processed, bd_processed_saved;

	while (1) {
		spin_lock_irqsave(&receivedQueueSpin, flags);
		if (list_empty(&receivedQueue)) {
			spin_unlock_irqrestore(&receivedQueueSpin, flags);
			break;
		}
		lp = list_entry(receivedQueue.next, struct net_local, rcv);
		list_del_init(&(lp->rcv));
		dev = lp->ndev;
		spin_unlock_irqrestore(&receivedQueueSpin, flags);

		spin_lock(&XTE_spinlock);
		if ((bd_processed =
		     XTemac_SgGetProcessed(&lp->Emac, XTE_RECV,
					   XTE_DFT_RECV_DESC, &BdPtr)) > 0) {

			bd_processed_saved = bd_processed;
			BdCurPtr = BdPtr;
			do {
				len = XDmaBdV2_mGetProcessedLength(BdCurPtr);

				/* get ptr to skb */
				skb =
				    (struct sk_buff *)XDmaBdV2_mGetId(BdCurPtr);

				/* get and free up dma handle used by skb->data */
				skb_baddr = (dma_addr_t)
				    XDmaBdV2_mGetDestAddr(BdCurPtr);
				dma_unmap_single(NULL, skb_baddr,
						 lp->max_frame_size,
						 DMA_FROM_DEVICE);

				/* reset ID */
				XDmaBdV2_mSetId(BdCurPtr, NULL);

				/* setup received skb and send it upstream */
				skb_put(skb, len);	/* Tell the skb how much data we got. */
				skb->dev = dev;
				skb->protocol = eth_type_trans(skb, dev);
#if 0
				/* Handle checksum offloading for incoming packets. */
				if (checksum is OK)
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				else
					skb->ip_summed = CHECKSUM_NONE;
#else
				skb->ip_summed = CHECKSUM_NONE;
#endif
				lp->stats.rx_packets++;
				lp->stats.rx_bytes += len;
				netif_rx(skb);	/* Send the packet upstream. */

				BdCurPtr =
				    XTemac_mSgRecvBdNext(&lp->Emac, BdCurPtr);
				bd_processed--;
			} while (bd_processed > 0);

			/* give the descriptor back to the driver */
			result =
			    XTemac_SgFree(&lp->Emac, XTE_RECV,
					  bd_processed_saved, BdPtr);
			if (result != XST_SUCCESS) {
				printk(KERN_ERR
				       "%s: XTemac _SgFree unsuccessful (%d)\n",
				       dev->name, result);
				reset(dev, __LINE__);
				spin_unlock(&XTE_spinlock);
				return;
			}

			/* Allocate same number of RxBD 1 by 1. We could allocate multiple RxBDs in one
			 * call, but as there is no guarantee we can have enough socket buffers from kernel
			 * and in that case we could not return the unused allocated RxBD(s) to the DMA RX BD
			 * ring, 1 by 1 allocation is safer solution.
			 */
			bd_processed = bd_processed_saved;
			do {
				/* replace skb with a new one so TEMAC can continue receive packets */
				new_skb =
				    alloc_skb(lp->max_frame_size +
					      ALIGNMENT_RECV, GFP_ATOMIC);
				if (new_skb == NULL) {
					printk(KERN_ERR
					       "%s: XTemac SgRecvHandlerBH(): no memory for new_skb\n",
					       dev->name);
					spin_unlock(&XTE_spinlock);
					return;
				}

				result =
				    XTemac_SgAlloc(&lp->Emac, XTE_RECV, 1,
						   &BdPtr);
				if (result != XST_SUCCESS) {
					dev_kfree_skb_any(new_skb);
					printk(KERN_ERR
					       "%s: XTemac _SgAlloc: unsuccessful (%d)\n",
					       dev->name, result);
					reset(dev, __LINE__);
					spin_unlock(&XTE_spinlock);
					return;
				}

				/* make sure we're long-word aligned */
				align = BUFFER_ALIGNRECV(new_skb->data);
				if (align) {
					skb_reserve(new_skb, align);
				}

				/* Get dma handle of skb->data */
				new_skb_baddr =
				    dma_map_single(NULL, new_skb->data,
						   lp->max_frame_size,
						   DMA_FROM_DEVICE);

				XDmaBdV2_mSetDestAddr(BdPtr, new_skb_baddr);
				XDmaBdV2_mSetLength(BdPtr, lp->max_frame_size);
				XDmaBdV2_mSetId(BdPtr, new_skb);

				/* enqueue RxBD with the attached skb buffers such that it is
				 * ready for frame reception */
				result =
				    XTemac_SgCommit(&lp->Emac, XTE_RECV, 1,
						    BdPtr);
				if (result != XST_SUCCESS) {
					printk(KERN_ERR
					       "%s: XTemac SgRecvHandlerBH: XTemac_SgCommit unsuccessful (%d)\n",
					       dev->name, result);
					dev_kfree_skb_any(new_skb);
					XDmaBdV2_mSetId(BdPtr, NULL);
					reset(dev, __LINE__);
					spin_unlock(&XTE_spinlock);
					return;
				}
				bd_processed--;

			} while (bd_processed > 0);

		}
		XTemac_IntrSgEnable(&lp->Emac, XTE_RECV);
		spin_unlock(&XTE_spinlock);
	}
}

static void SgRecvHandler(void *CallBackRef)
{
	struct net_local *lp;
	struct list_head *cur_lp;

	spin_lock(&receivedQueueSpin);
	lp = (struct net_local *)CallBackRef;
	list_for_each(cur_lp, &receivedQueue)
	    if (cur_lp == &(lp->rcv))
		break;

	if (cur_lp != &(lp->rcv)) {
		list_add_tail(&lp->rcv, &receivedQueue);
		XTemac_IntrSgDisable(&lp->Emac, XTE_RECV);
		tasklet_schedule(&SgRecvBH);
	}
	spin_unlock(&receivedQueueSpin);
}

/* The callback function for errors. */
static void
ErrorHandler(void *CallbackRef, XStatus ErrClass, u32 Word1, u32 Word2)
{
	struct net_device *ndev;
	struct net_local *lp;
	int need_reset;

	spin_lock(&XTE_spinlock);
	ndev = (struct net_device *)CallbackRef;
	lp = netdev_priv(ndev);

	need_reset = status_requires_reset(ErrClass);
	/* The following print code could be used for debugging */
	/*
	   printk(KERN_ERR "%s: XTemac device error %d%s\n",
	   dev->name, ErrClass, need_reset ? ", resetting device." : "");
	 */
	if (need_reset)
		reset(ndev, __LINE__);

	spin_unlock(&XTE_spinlock);
}

/*
 * This function is assumed to be called after XEmac_Initialize(),
 * but before XEmac is started with XEmac_Start().
 * Calling XEmac_SgRecv() if XEmac is not started is just fine.
 */
static int sgdma_descriptor_init(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	int i, recvsize, sendsize;
	u32 *recvpool_v, *sendpool_v;
	void *recvpoolphy, *sendpoolphy;
	XStatus result;
	XDmaBdV2 bd_template;

	/* calc size of send and recv descriptor space */
	recvsize = XTE_DFT_RECV_DESC * sizeof(XDmaBdV2);
	sendsize = XTE_DFT_SEND_DESC * sizeof(XDmaBdV2);

	/* Allocate (non-cached) memory for the buffer descriptors */
	lp->desc_space_size = recvsize + sendsize;

#if BD_IN_BRAM == 0
	lp->desc_space = dma_alloc_coherent(NULL, lp->desc_space_size,
					    &lp->desc_space_handle, GFP_KERNEL);
#else
	lp->desc_space_handle = 0xffff8000;
	lp->desc_space = ioremap(lp->desc_space_handle, lp->desc_space_size);
#endif
	if (lp->desc_space == NULL)
		return -ENOMEM;

	memset(lp->desc_space, 0, lp->desc_space_size);

	printk(KERN_INFO
	       "%s: XTemac buffer_descriptor_space: phy: 0x%x, virt: 0x%x, size: 0x%x\n",
	       ndev->name, lp->desc_space_handle, (unsigned int)lp->desc_space,
	       lp->desc_space_size);

	recvpool_v = lp->desc_space;
	sendpool_v = (u32 *) ((u32) lp->desc_space + recvsize);

	recvpoolphy = (void *)lp->desc_space_handle;
	sendpoolphy = (void *)((u32) lp->desc_space_handle + recvsize);

	/* set up descriptor spaces using a template */
	XDmaBdV2_mClear(&bd_template);
	XDmaBdV2_mSetLast(&bd_template);
	if ((result = XTemac_SgSetSpace(&lp->Emac, XTE_RECV, (u32) recvpoolphy,
					(u32) recvpool_v, ALIGNMENT_BD,
					XTE_DFT_RECV_DESC,
					&bd_template)) != XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac _SgSetSpace RECV ERROR %d\n",
		       ndev->name, result);
		return -EIO;
	}

	if ((result = XTemac_SgSetSpace(&lp->Emac, XTE_SEND, (u32) sendpoolphy,
					(u32) sendpool_v, ALIGNMENT_BD,
					XTE_DFT_SEND_DESC,
					&bd_template)) != XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac _SgSetSpace SEND ERROR %d\n",
		       ndev->name, result);
		return -EIO;
	}

	/* Allocate skb's, attach them to BD's and give to the RX dma engine
	 * Have to allocate and commit BD's one by one to make sure valid skb's
	 * are attached.
	 */
	for (i = 0; i < XTE_DFT_RECV_DESC; i++) {
		struct sk_buff *skb;
		XDmaBdV2 *bd_ptr;
		int result;
		u32 skb_dma_addr, align;

		skb = dev_alloc_skb(lp->max_frame_size + ALIGNMENT_RECV);
		if (skb == NULL) {
			printk(KERN_ERR
			       "%s: XTemac alloc_skb fails to allocate socket buffers\r\n",
			       ndev->name);
			goto fail;
		}

		align = BUFFER_ALIGNRECV(skb->data);
		if (align)
			skb_reserve(skb, align);

		skb_dma_addr = (u32) dma_map_single(NULL, skb->data,
						    lp->max_frame_size,
						    DMA_FROM_DEVICE);

		/* BH has similar function call sequence. Lock before go */
		spin_lock_bh(&XTE_spinlock);

		/* Allocate 1 RxBD. Note that TEMAC utilizes an in-place allocation
		 * scheme. The returned Bd Ptr will point to a free BD in the memory
		 * segment setup with the call to XTemac_SgSetSpace() */
		if ((result =
		     XTemac_SgAlloc(&lp->Emac, XTE_RECV, 1,
				    &bd_ptr)) != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: XTemac descriptor_init(): XTemac_SgAlloc ERROR %d\n",
			       ndev->name, result);
			spin_unlock_bh(&XTE_spinlock);
			goto fail;
		}

		/* Setup the BD. The BD template used in the call to XTemac_SgSetSpace()
		 * set the "last" field of all RxBDs. Therefore we are not required to
		 * issue a XDmaBdV2_mSetLast(bd_ptr) here.  BD length gets max frame size,
		 * and address of skb is also stored for later fetching after packet
		 * reception. */
		XDmaBdV2_mSetDestAddr(bd_ptr, skb_dma_addr);
		XDmaBdV2_mSetLength(bd_ptr, lp->max_frame_size);
		XDmaBdV2_mSetId(bd_ptr, skb);

		/* Enqueue buffer descriptor with attached buffer to the hardware such
		 * that it is ready for frame reception */
		if ((result =
		     XTemac_SgCommit(&lp->Emac, XTE_RECV, 1,
				     bd_ptr)) != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: XTemac descriptor_init(): XTemac_SgCommit ERROR %d\n",
			       ndev->name, result);
			spin_unlock_bh(&XTE_spinlock);
			goto fail;
		}
		spin_unlock_bh(&XTE_spinlock);
	}
	return 0;
      fail:
	/* TODO: free all already allocated resources */
	return -1;
}

void free_descriptor_skb(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	XDmaBdV2 *BdPtr;
	struct sk_buff *skb;
	dma_addr_t skb_dma_addr;
	u32 len, i;

	/* Unmap and free skb's allocated and mapped in descriptor_init() */

	/* Get the virtual address of the 1st BD in the DMA RX BD ring */
	BdPtr = (XDmaBdV2 *) lp->Emac.RecvDma.BdRing.BaseAddr;

	for (i = 0; i < XTE_DFT_RECV_DESC; i++) {
		skb = (struct sk_buff *)XDmaBdV2_mGetId(BdPtr);
		if (skb) {
			skb_dma_addr =
			    (dma_addr_t) XDmaBdV2_mGetDestAddr(BdPtr);
			dma_unmap_single(NULL, skb_dma_addr, lp->max_frame_size,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
		}
		/* find the next BD in the DMA RX BD ring */
		BdPtr = XTemac_mSgRecvBdNext(&lp->Emac, BdPtr);
	}

	/* Unmap and free TX skb's that have not had a chance to be freed
	 * in SgSendHandlerBH(). This could happen when TX Threshold is larger
	 * than 1 and TX waitbound is 0
	 */

	/* Get the virtual address of the 1st BD in the DMA TX BD ring */
	BdPtr = (XDmaBdV2 *) lp->Emac.SendDma.BdRing.BaseAddr;

	for (i = 0; i < XTE_DFT_SEND_DESC; i++) {
		skb = (struct sk_buff *)XDmaBdV2_mGetId(BdPtr);
		if (skb) {
			skb_dma_addr = (dma_addr_t) XDmaBdV2_mGetSrcAddr(BdPtr);
			len = XDmaBdV2_mGetProcessedLength(BdPtr);
			dma_unmap_single(NULL, skb_dma_addr, len,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
		}
		/* find the next BD in the DMA TX BD ring */
		BdPtr = XTemac_mSgSendBdNext(&lp->Emac, BdPtr);
	}

#if BD_IN_BRAM == 0
	dma_free_coherent(NULL, lp->desc_space_size, lp->desc_space,
			  lp->desc_space_handle);
#else
	iounmap(lp->desc_space);
#endif
}

static int
xtenet_ethtool_get_settings(struct net_device *dev, struct ethtool_cmd *ecmd)
{
	struct net_local *lp = (struct net_local *)dev->priv;
	u16 threshold, timer;
	XStatus xs;

	memset(ecmd, 0, sizeof(struct ethtool_cmd));

	ecmd->duplex = DUPLEX_FULL;
	ecmd->supported |= SUPPORTED_MII;
	ecmd->port = PORT_MII;
	ecmd->speed = SPEED_1000;
	ecmd->supported |= SUPPORTED_Autoneg;
	ecmd->autoneg = AUTONEG_ENABLE;
	ecmd->phy_address = (lp->Emac.Config)->BaseAddress;
	ecmd->transceiver = XCVR_INTERNAL;

	if (XTemac_mIsSgDma(&lp->Emac)) {
		/* get TX threshold */
		if ((xs =
		     XTemac_IntrSgCoalGet(&lp->Emac, XTE_SEND, &threshold,
					  &timer))
		    == XST_SUCCESS)
			ecmd->maxtxpkt = threshold;
		else
			return -EIO;

		/* get RX threshold */
		if ((xs =
		     XTemac_IntrSgCoalGet(&lp->Emac, XTE_RECV, &threshold,
					  &timer))
		    == XST_SUCCESS)
			ecmd->maxrxpkt = threshold;
		else
			return -EIO;
	}
	return 0;
}

static int
xtenet_ethtool_get_coalesce(struct net_device *dev, struct ethtool_coalesce *ec)
{
	int ret;
	struct net_local *lp = (struct net_local *)dev->priv;
	u16 threshold, waitbound;

	memset(ec, 0, sizeof(struct ethtool_coalesce));

	if ((ret =
	     XTemac_IntrSgCoalGet(&lp->Emac, XTE_RECV, &threshold, &waitbound))
	    != XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac IntrSgCoalGet error %d\n",
		       dev->name, ret);
		return -EIO;
	}
	ec->rx_max_coalesced_frames = threshold;
	ec->rx_coalesce_usecs = waitbound;

	if ((ret =
	     XTemac_IntrSgCoalGet(&lp->Emac, XTE_SEND, &threshold, &waitbound))
	    != XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac IntrSgCoalGet error %d\n",
		       dev->name, ret);
		return -EIO;
	}
	ec->tx_max_coalesced_frames = threshold;
	ec->tx_coalesce_usecs = waitbound;

	return 0;
}

static int
xtenet_ethtool_set_coalesce(struct net_device *dev, struct ethtool_coalesce *ec)
{
	int ret;
	struct net_local *lp;
	unsigned long flags;
	int dev_started;
	int status;

	spin_lock_irqsave(&XTE_spinlock, flags);
	lp = (struct net_local *)dev->priv;

	status = -EIO;
	dev_started = XTemac_IsStarted(&lp->Emac);

	if (dev_started)
		XTemac_Stop(&lp->Emac);

	if ((ret = XTemac_IntrSgCoalSet(&lp->Emac, XTE_RECV,
					(u16) (ec->rx_max_coalesced_frames),
					(u16) (ec->rx_coalesce_usecs))) !=
	    XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac IntrSgCoalSet error %d\n",
		       dev->name, ret);
		goto fail;
	}

	if ((ret = XTemac_IntrSgCoalSet(&lp->Emac, XTE_SEND,
					(u16) (ec->tx_max_coalesced_frames),
					(u16) (ec->tx_coalesce_usecs))) !=
	    XST_SUCCESS) {
		printk(KERN_ERR "%s: XTemac IntrSgCoalSet error %d\n",
		       dev->name, ret);
		goto fail;
	}

	if (dev_started && XTemac_Start(&lp->Emac) != XST_SUCCESS)
		goto fail;
	status = 0;
      fail:
	spin_unlock_irqrestore(&XTE_spinlock, flags);
	return status;

}

static int
xtenet_ethtool_get_ringparam(struct net_device *dev,
			     struct ethtool_ringparam *erp)
{
	memset(erp, 0, sizeof(struct ethtool_ringparam));

	erp->rx_max_pending = XTE_DFT_RECV_DESC;
	erp->tx_max_pending = XTE_DFT_SEND_DESC;
	erp->rx_pending = XTE_DFT_RECV_DESC;
	erp->tx_pending = XTE_DFT_SEND_DESC;
	return 0;
}

static int
xtenet_ethtool_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *ed)
{
	memset(ed, 0, sizeof(struct ethtool_drvinfo));
	strncpy(ed->driver, DRIVER_NAME, sizeof(ed->driver) - 1);
	strncpy(ed->version, DRIVER_VERSION, sizeof(ed->version) - 1);
	/* Also tell how much memory is needed for dumping register values */
	ed->regdump_len = 0;
	return 0;
}

static int xtenet_do_ethtool_ioctl(struct net_device *dev, struct ifreq *rq)
{
	struct net_local *lp = (struct net_local *)dev->priv;
	struct ethtool_cmd ecmd;
	struct ethtool_coalesce eco;
	struct ethtool_drvinfo edrv;
	struct ethtool_ringparam erp;
	struct ethtool_pauseparam epp;
	int ret = -EOPNOTSUPP;
	u32 Options;

	if (copy_from_user(&ecmd, rq->ifr_data, sizeof(ecmd.cmd)))
		return -EFAULT;
	switch (ecmd.cmd) {
	default:
	case ETHTOOL_SSET:	/* Change setting.  "-s" option w/ ethtool      */
	case ETHTOOL_SPAUSEPARAM:	/* Set pause parameter. "-A" w/ ethtool         */
	case ETHTOOL_GREGS:	/* Get register values. "-d" with ethtool       */
		return -EOPNOTSUPP;

	case ETHTOOL_GSET:	/* Get setting. No command option needed w/ ethtool */
		ret = xtenet_ethtool_get_settings(dev, &ecmd);
		if (ret < 0)
			return -EIO;
		if (copy_to_user(rq->ifr_data, &ecmd, sizeof(ecmd)))
			return -EFAULT;
		ret = 0;
		break;

	case ETHTOOL_GPAUSEPARAM:	/* Get pause parameter information. Use "-a" w/ ethtool */
		ret = xtenet_ethtool_get_settings(dev, &ecmd);
		if (ret < 0)
			return ret;
		epp.cmd = ecmd.cmd;
		epp.autoneg = ecmd.autoneg;
		Options = XTemac_GetOptions(&lp->Emac);
		if (Options & XTE_FCS_INSERT_OPTION) {
			epp.rx_pause = 1;
			epp.tx_pause = 1;
		} else {
			epp.rx_pause = 0;
			epp.tx_pause = 0;
		}
		if (copy_to_user
		    (rq->ifr_data, &epp, sizeof(struct ethtool_pauseparam)))
			return -EFAULT;
		ret = 0;
		break;

	case ETHTOOL_GCOALESCE:	/* Get coalescing info. Use "-c" w/ ethtool */
		if (!(XTemac_mIsSgDma(&lp->Emac)))
			break;
		eco.cmd = ecmd.cmd;
		ret = xtenet_ethtool_get_coalesce(dev, &eco);
		if (ret < 0) {
			return -EIO;
		}
		if (copy_to_user
		    (rq->ifr_data, &eco, sizeof(struct ethtool_coalesce))) {
			return -EFAULT;
		}
		ret = 0;
		break;

	case ETHTOOL_SCOALESCE:	/* Set coalescing info. Use "-C" w/ ethtool */
		if (!(XTemac_mIsSgDma(&lp->Emac)))
			break;
		if (copy_from_user
		    (&eco, rq->ifr_data, sizeof(struct ethtool_coalesce)))
			return -EFAULT;
		ret = xtenet_ethtool_set_coalesce(dev, &eco);
		break;

	case ETHTOOL_GDRVINFO:	/* Get driver information. Use "-i" w/ ethtool */
		edrv.cmd = edrv.cmd;
		ret = xtenet_ethtool_get_drvinfo(dev, &edrv);
		if (ret < 0) {
			return -EIO;
		}
		if (copy_to_user
		    (rq->ifr_data, &edrv, sizeof(struct ethtool_drvinfo))) {
			return -EFAULT;
		}
		ret = 0;
		break;
	case ETHTOOL_GRINGPARAM:	/* Get RX/TX ring parameters. Use "-g" w/ ethtool */
		erp.cmd = edrv.cmd;
		ret = xtenet_ethtool_get_ringparam(dev, &(erp));
		if (ret < 0) {
			return ret;
		}
		if (copy_to_user
		    (rq->ifr_data, &erp, sizeof(struct ethtool_ringparam))) {
			return -EFAULT;
		}
		ret = 0;
		break;
	case ETHTOOL_NWAY_RST:	/* Restart auto negotiation if enabled. Use "-r" w/ ethtool */
		reset(dev, __LINE__);	/* Resetting core and thus PHY restarts auto negotiation */
		return 0;
	}
	return ret;
}

static int xtenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct net_local *lp = (struct net_local *)dev->priv;

	struct {
		__u16 threshold;
		__u32 direction;
	} thr_arg;
	struct {
		__u16 waitbound;
		__u32 direction;
	} wbnd_arg;

	XStatus ret;
	unsigned long flags;
	u16 threshold, timer;
	int dev_started;
	int status;

	switch (cmd) {
	case SIOCETHTOOL:
		return xtenet_do_ethtool_ioctl(dev, rq);

	case SIOCGMIIPHY:	/* Get address of GMII PHY in use. */
	case SIOCDEVPRIVATE:	/* for binary compat, remove in 2.5 */
	case SIOCGMIIREG:	/* Read GMII PHY register. */
	case SIOCDEVPRIVATE + 1:	/* for binary compat, remove in 2.5 */
		return -EOPNOTSUPP;	/* TODO: To support in next version */

	case SIOCSMIIREG:	/* Write GMII PHY register. */
	case SIOCDEVPRIVATE + 2:	/* for binary compat, remove in 2.5 */
		return -EOPNOTSUPP;	/* TODO: To support in next version */

	case SIOCDEVPRIVATE + 3:	/* set THRESHOLD */
		if (!(XTemac_mIsSgDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&thr_arg, rq->ifr_data, sizeof(thr_arg)))
			return -EFAULT;

		spin_lock_irqsave(&XTE_spinlock, flags);

		dev_started = XTemac_IsStarted(&lp->Emac);

		if (dev_started)
			XTemac_Stop(&lp->Emac);

		status = -EIO;
		if (XTemac_IntrSgCoalGet(&lp->Emac, thr_arg.direction,
					 &threshold, &timer) != XST_SUCCESS)
			goto out3;
		if (XTemac_IntrSgCoalSet(&lp->Emac, thr_arg.direction,
					 thr_arg.threshold,
					 timer) != XST_SUCCESS)
			goto out3;
		if (dev_started && XTemac_Start(&lp->Emac) != XST_SUCCESS)
			goto out3;
		status = 0;

	      out3:
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return status;

	case SIOCDEVPRIVATE + 4:	/* set WAITBOUND */
		if (!(XTemac_mIsSgDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&wbnd_arg, rq->ifr_data, sizeof(wbnd_arg)))
			return -EFAULT;

		spin_lock_irqsave(&XTE_spinlock, flags);
		dev_started = XTemac_IsStarted(&lp->Emac);
		if (dev_started)
			XTemac_Stop(&lp->Emac);

		status = -EIO;
		if (XTemac_IntrSgCoalGet(&lp->Emac, wbnd_arg.direction,
					 &threshold, &timer) != XST_SUCCESS)
			goto out4;
		if (XTemac_IntrSgCoalSet(&lp->Emac, wbnd_arg.direction,
					 threshold,
					 wbnd_arg.waitbound) != XST_SUCCESS)
			goto out4;
		if (dev_started && XTemac_Start(&lp->Emac) != XST_SUCCESS)
			goto out4;
		status = 0;
	      out4:
		spin_unlock_irqrestore(&XTE_spinlock, flags);
		return status;

	case SIOCDEVPRIVATE + 5:	/* get THRESHOLD */
		if (!(XTemac_mIsSgDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&thr_arg, rq->ifr_data, sizeof(thr_arg)))
			return -EFAULT;

		if ((ret = XTemac_IntrSgCoalGet(&lp->Emac, thr_arg.direction,
						(u16 *) & (thr_arg.threshold),
						&timer)) != XST_SUCCESS) {
			return -EIO;
		}
		if (copy_to_user(rq->ifr_data, &thr_arg, sizeof(thr_arg))) {
			return -EFAULT;
		}
		return 0;

	case SIOCDEVPRIVATE + 6:	/* get WAITBOUND */
		if (!(XTemac_mIsSgDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&wbnd_arg, rq->ifr_data, sizeof(wbnd_arg))) {
			return -EFAULT;
		}
		if ((ret = XTemac_IntrSgCoalGet(&lp->Emac, wbnd_arg.direction,
						&threshold,
						(u16 *) & (wbnd_arg.
							   waitbound))) !=
		    XST_SUCCESS) {
			return -EIO;
		}
		if (copy_to_user(rq->ifr_data, &wbnd_arg, sizeof(wbnd_arg))) {
			return -EFAULT;
		}
		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

/*********************
 * The device driver *
 *********************/

static void xtenet_remove_ndev(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);

	iounmap((void *)lp->Emac.BaseAddress);

	/* Free up the memory. */
	free_netdev(ndev);
}

static int xtenet_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	unregister_netdev(ndev);
	xtenet_remove_ndev(ndev);

	return 0;		/* success */
}

static int xtenet_probe(struct device *dev)
{
	struct net_device *ndev;
	struct net_local *lp;
	struct platform_device *pdev = to_platform_device(dev);
	struct xtemac_platform_data *pdata;
	struct resource *r_irq, *r_mem;
	u32 baseaddr_v;
	int retval;
	u32 id;

	pdata = (struct xtemac_platform_data *)pdev->dev.platform_data;

	if (!pdata) {
		printk(KERN_ERR "xtemac %d: Couldn't find platform data.\n",
		       pdev->id);

		return -ENODEV;
	}
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_irq || !r_mem) {
		printk(KERN_ERR "xtemac %d: IO resource(s) not found.\n",
		       pdev->id);

		return -ENODEV;
	}

	/* Create an ethernet device instance */
	ndev = alloc_etherdev(sizeof(struct net_local));
	if (!ndev) {
		printk(KERN_ERR "xtemac %d: Could not allocate net device.\n",
		       pdev->id);
		return -ENOMEM;
	}
	dev_set_drvdata(dev, ndev);
	ndev->irq = r_irq->start;

	/* Initialize the private data used by XEmac_LookupConfig().
	 * The private data are zeroed out by alloc_etherdev() already.
	 */
	lp = netdev_priv(ndev);
	lp->Config.DeviceId = pdev->id;
	lp->ndev = ndev;

	/* Initialize the rest of the private data */
	lp->Config.BaseAddress = r_mem->start;
	baseaddr_v = (u32) ioremap(r_mem->start, r_mem->end - r_mem->start + 1);
	lp->Config.IpIfDmaConfig = pdata->dma_mode;
	lp->Config.RxPktFifoDepth = pdata->rx_pkt_fifo_depth;
	lp->Config.TxPktFifoDepth = pdata->tx_pkt_fifo_depth;
	lp->Config.MacFifoDepth = pdata->mac_fifo_depth;
	lp->Config.DcrHost = pdata->dcr_host;
	lp->Config.Dre = pdata->dre;

	/* The following line would not be needed if XEmac_Initialize took
	 * ptr to XEmac_Config instead of DeviceId as the second arg.
	 */
	if (lp->Config.DeviceId < XTEMAC_MAX_DEVICE_ID)
		xtemac_cfgs[lp->Config.DeviceId] = &lp->Config;

	if (XTemac_VmInitialize(&lp->Emac, lp->Config.DeviceId,
				baseaddr_v) != XST_SUCCESS) {
		printk(KERN_ERR "%s: Could not initialize device.\n",
		       ndev->name);
		xtenet_remove_ndev(ndev);
		return -ENODEV;
	}

	memcpy(ndev->dev_addr, pdata->mac_addr, 6);
	if (XTemac_SetMacAddress(&lp->Emac, ndev->dev_addr) != XST_SUCCESS) {
		/* should not fail right after an initialize */
		printk(KERN_ERR "%s: XTemac could not set MAC address.\n",
		       ndev->name);
		xtenet_remove_ndev(ndev);
		return -EIO;
	}

	lp->max_frame_size = XTE_MAX_FRAME_SIZE;
	if (ndev->mtu > XTE_MTU)
		ndev->mtu = XTE_MTU;

	if (XTemac_mIsSgDma(&lp->Emac)) {
		int result;
		XStatus xs;

		printk(KERN_ERR "%s: XTemac using sgDMA mode.\n", ndev->name);
		XTemac_SetHandler(&lp->Emac, XTE_HANDLER_SGSEND, SgSendHandler,
				  lp);
		XTemac_SetHandler(&lp->Emac, XTE_HANDLER_SGRECV, SgRecvHandler,
				  lp);
		lp->Isr = XTemac_IntrSgHandler;

		if (XTemac_mIsDre(&lp->Emac) == TRUE)
			ndev->hard_start_xmit = xtenet_SgSend;
		else
			ndev->hard_start_xmit = xtenet_SgSend_NoDRE;

		ndev->features |= NETIF_F_IP_CSUM | NETIF_F_SG;
		result = sgdma_descriptor_init(ndev);
		if (result) {
			xtenet_remove_ndev(ndev);
			return -EIO;
		}

		/* set the packet threshold and wait bound for both TX/RX directions */
		if ((xs =
		     XTemac_IntrSgCoalSet(&lp->Emac, XTE_SEND, DFT_TX_THRESHOLD,
					  DFT_TX_WAITBOUND)) != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: XTemac could not set SEND pkt threshold/waitbound, ERROR %d",
			       ndev->name, xs);
		}
		if ((xs =
		     XTemac_IntrSgCoalSet(&lp->Emac, XTE_RECV, DFT_RX_THRESHOLD,
					  DFT_RX_WAITBOUND)) != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: XTemac Could not set RECV pkt threshold/waitbound ERROR %d",
			       ndev->name, xs);
		}
	} else {
		printk(KERN_INFO
		       "%s: XTemac using fifo direct interrupt driven mode.\n",
		       ndev->name);
		XTemac_SetHandler(&lp->Emac, XTE_HANDLER_FIFORECV,
				  FifoRecvHandler, ndev);
		XTemac_SetHandler(&lp->Emac, XTE_HANDLER_FIFOSEND,
				  FifoSendHandler, ndev);
		ndev->hard_start_xmit = xtenet_FifoSend;
		lp->Isr = XTemac_IntrFifoHandler;
	}
	XTemac_SetHandler(&lp->Emac, XTE_HANDLER_ERROR, ErrorHandler, ndev);

	SET_NETDEV_DEV(ndev, dev);

	ndev->open = xtenet_open;
	ndev->stop = xtenet_close;
	ndev->change_mtu = xtenet_change_mtu;
	ndev->get_stats = xtenet_get_stats;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->do_ioctl = xtenet_ioctl;
	ndev->tx_timeout = xtenet_tx_timeout;
	ndev->watchdog_timeo = TX_TIMEOUT;

	retval = register_netdev(ndev);
	if (retval) {
		printk(KERN_ERR
		       "%s: Cannot register net device, aborting.\n",
		       ndev->name);
		xtenet_remove_ndev(ndev);
		return retval;
	}

	printk(KERN_INFO
	       "%s: Xilinx TEMAC #%d at 0x%08X mapped to 0x%08X, irq=%d\n",
	       ndev->name, lp->Config.DeviceId,
	       lp->Config.BaseAddress, lp->Emac.BaseAddress, ndev->irq);

	/* print h/w id  */
	id = XIo_In32((lp->Emac).BaseAddress + XIIF_V123B_RESETR_OFFSET);

	printk(KERN_INFO
	       "%s: XTemac id %d.%d%c, block id %d, type %d\n",
	       ndev->name, (id >> 28) & 0xf, (id >> 21) & 0x7f,
	       ((id >> 16) & 0x1f) + 'a', (id >> 16) & 0xff, (id >> 0) & 0xff);

	return 0;
}

static struct device_driver xtenet_driver = {
	.name = DRIVER_NAME,
	.bus = &platform_bus_type,

	.probe = xtenet_probe,
	.remove = xtenet_remove
};

static int __init xtenet_init(void)
{
	/*
	 * No kernel boot options used,
	 * so we just need to register the driver
	 */
	return driver_register(&xtenet_driver);
}

static void __exit xtenet_cleanup(void)
{
	driver_unregister(&xtenet_driver);
}

module_init(xtenet_init);
module_exit(xtenet_cleanup);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");
