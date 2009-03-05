/*
 * adapter.c
 *
 * Xilinx Ethernet Adapter component to interface XEmac component to Linux
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2005 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * This driver is a bit unusual in that it is composed of two logical
 * parts where one part is the OS independent code and the other part is
 * the OS dependent code.  Xilinx provides their drivers split in this
 * fashion.  This file represents the Linux OS dependent part known as
 * the Linux adapter.  The other files in this directory are the OS
 * independent files as provided by Xilinx with no changes made to them.
 * The names exported by those files begin with XEmac_.  All functions
 * in this file that are called by Linux have names that begin with
 * xenet_.  The functions in this file that have Handler in their name
 * are registered as callbacks with the underlying Xilinx OS independent
 * layer.  Any other functions are static helper functions.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/xilinx_devices.h>
#include <linux/ethtool.h>

#include <asm/io.h>

#include "adapter.h"
#include "xipif_v1_23_b.h"

#define TX_TIMEOUT (3*HZ)	/* Transmission timeout is 3 seconds. */

DEFINE_SPINLOCK(XE_spinlock);

/*
 * Helper function to reset the underlying hardware.  This is called
 * when we get into such deep trouble that we don't know how to handle
 * otherwise.
 */
void xenet_reset(struct net_device *ndev, DUPLEX duplex)
{
	struct net_local *lp = netdev_priv(ndev);
	struct sk_buff *tskb;
	u32 Options;
	u8 IfgPart1;
	u8 IfgPart2;
	u8 SendThreshold;
	u32 SendWaitBound;
	u8 RecvThreshold;
	u32 RecvWaitBound;
	unsigned long flags;

	/* Shouldn't really be necessary, but shouldn't hurt. */
	netif_stop_queue(ndev);

	/*
	 * XEmac_Reset puts the device back to the default state.  We need
	 * to save all the settings we don't already know, reset, restore
	 * the settings, and then restart the emac.
	 */
	XEmac_GetInterframeGap(&lp->Emac, &IfgPart1, &IfgPart2);
	Options = XEmac_GetOptions(&lp->Emac);
	switch (duplex) {
	case HALF_DUPLEX:
		Options &= ~XEM_FDUPLEX_OPTION;
		break;
	case FULL_DUPLEX:
		Options |= XEM_FDUPLEX_OPTION;
		break;
	case UNKNOWN_DUPLEX:
		break;
	}

	if (XEmac_mIsSgDma(&lp->Emac)) {
		/*
		 * The following four functions will return an error if we are
		 * not doing scatter-gather DMA.  We just checked that so we
		 * can safely ignore the return values.  We cast them to void
		 * to make that explicit.
		 */
		(void)XEmac_GetPktThreshold(&lp->Emac, XEM_SEND,
					    &SendThreshold);
		(void)XEmac_GetPktWaitBound(&lp->Emac, XEM_SEND,
					    &SendWaitBound);
		(void)XEmac_GetPktThreshold(&lp->Emac, XEM_RECV,
					    &RecvThreshold);
		(void)XEmac_GetPktWaitBound(&lp->Emac, XEM_RECV,
					    &RecvWaitBound);
	}

	XEmac_Reset(&lp->Emac);

	/*
	 * The following three functions will return an error if the
	 * EMAC is already started.  We just stopped it by calling
	 * XEmac_Reset() so we can safely ignore the return values.
	 * We cast them to void to make that explicit.
	 */
	(void)XEmac_SetMacAddress(&lp->Emac, ndev->dev_addr);
	(void)XEmac_SetInterframeGap(&lp->Emac, IfgPart1, IfgPart2);
	(void)XEmac_SetOptions(&lp->Emac, Options);
	if (XEmac_mIsSgDma(&lp->Emac)) {
		/*
		 * The following four functions will return an error if
		 * we are not doing scatter-gather DMA or if the EMAC is
		 * already started.  We just checked that we are indeed
		 * doing scatter-gather and we just stopped the EMAC so
		 * we can safely ignore the return values.  We cast them
		 * to void to make that explicit.
		 */
		(void)XEmac_SetPktThreshold(&lp->Emac, XEM_SEND, SendThreshold);
		(void)XEmac_SetPktWaitBound(&lp->Emac, XEM_SEND, SendWaitBound);
		(void)XEmac_SetPktThreshold(&lp->Emac, XEM_RECV, RecvThreshold);
		(void)XEmac_SetPktWaitBound(&lp->Emac, XEM_RECV, RecvWaitBound);
	}

	/*
	 * XEmac_Start returns an error when: it is already started, the send
	 * and receive handlers are not set, or a scatter-gather DMA list is
	 * missing.  None of these can happen at this point, so we cast the
	 * return to void to make that explicit.
	 */
	(void)XEmac_Start(&lp->Emac);

	/* Make sure that the send handler and we don't both free the skb. */
	spin_lock_irqsave(&lp->skb_lock, flags);
	tskb = lp->saved_skb;
	lp->saved_skb = NULL;
	spin_unlock_irqrestore(&lp->skb_lock, flags);
	if (tskb)
		dev_kfree_skb(tskb);

	/* We're all ready to go.  Start the queue in case it was stopped. */
	netif_wake_queue(ndev);
}

static int get_phy_status(struct net_device *ndev, DUPLEX * duplex, int *linkup)
{
	struct net_local *lp = netdev_priv(ndev);
	u16 reg;
	XStatus xs;

	xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr, MII_BMCR, &reg);
	if (xs != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: Could not read PHY control register; error %d\n",
		       ndev->name, xs);
		return -1;
	}

	if (!(reg & BMCR_ANENABLE)) {
		/*
		 * Auto-negotiation is disabled so the full duplex bit in
		 * the control register tells us if the PHY is running
		 * half or full duplex.
		 */
		*duplex = (reg & BMCR_FULLDPLX) ? FULL_DUPLEX : HALF_DUPLEX;
	} else {
		/*
		 * Auto-negotiation is enabled.  Figure out what was
		 * negotiated by looking for the best mode in the union
		 * of what we and our partner advertise.
		 */
		u16 advertise, partner, negotiated;

		xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr,
				   MII_ADVERTISE, &advertise);
		if (xs != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: Could not read PHY advertisement; error %d\n",
			       ndev->name, xs);
			return -1;
		}
		xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr, MII_LPA, &partner);
		if (xs != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: Could not read PHY LPA; error %d\n",
			       ndev->name, xs);
			return -1;
		}

		negotiated = advertise & partner & ADVERTISE_ALL;
		if (negotiated & ADVERTISE_100FULL)
			*duplex = FULL_DUPLEX;
		else if (negotiated & ADVERTISE_100HALF)
			*duplex = HALF_DUPLEX;
		else if (negotiated & ADVERTISE_10FULL)
			*duplex = FULL_DUPLEX;
		else
			*duplex = HALF_DUPLEX;
	}

	xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr, MII_BMSR, &reg);
	if (xs != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: Could not read PHY status register; error %d\n",
		       ndev->name, xs);
		return -1;
	}

	*linkup = (reg & BMSR_LSTATUS) != 0;

	return 0;
}

/*
 * This routine is used for two purposes.  The first is to keep the
 * EMAC's duplex setting in sync with the PHY's.  The second is to keep
 * the system apprised of the state of the link.  Note that this driver
 * does not configure the PHY.  Either the PHY should be configured for
 * auto-negotiation or it should be handled by something like mii-tool.
 */
static void poll_mii(unsigned long data)
{
	struct net_device *ndev = (struct net_device *)data;
	struct net_local *lp = netdev_priv(ndev);
	u32 Options;
	unsigned long flags;
	DUPLEX phy_duplex, mac_duplex;
	int phy_carrier, netif_carrier;

	/* First, find out what's going on with the PHY. */
	if (get_phy_status(ndev, &phy_duplex, &phy_carrier)) {
		printk(KERN_ERR "%s: Terminating link monitoring.\n",
		       ndev->name);
		return;
	}

	/* Second, figure out if we have the EMAC in half or full duplex. */
	Options = XEmac_GetOptions(&lp->Emac);
	mac_duplex = (Options & XEM_FDUPLEX_OPTION) ? FULL_DUPLEX : HALF_DUPLEX;

	/* Now see if there is a mismatch. */
	if (mac_duplex != phy_duplex) {
		/*
		 * Make sure that no interrupts come in that could cause
		 * reentrancy problems in reset.
		 */
		spin_lock_irqsave(&XE_spinlock, flags);
		xenet_reset(ndev, phy_duplex);
		spin_unlock_irqrestore(&XE_spinlock, flags);
	}

	netif_carrier = netif_carrier_ok(ndev) != 0;

	if (phy_carrier != netif_carrier) {
		if (phy_carrier) {
			printk(KERN_INFO "%s: Link carrier restored.\n",
			       ndev->name);
			netif_carrier_on(ndev);
		} else {
			printk(KERN_INFO "%s: Link carrier lost.\n",
			       ndev->name);
			netif_carrier_off(ndev);
		}
	}

	/* Set up the timer so we'll get called again in 2 seconds. */
	lp->phy_timer.expires = jiffies + 2 * HZ;
	add_timer(&lp->phy_timer);
}

static void xenet_tx_timeout(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	unsigned long flags;
	printk("%s: Exceeded transmit timeout of %lu ms.  Resetting emac.\n",
	       ndev->name, TX_TIMEOUT * 1000UL / HZ);

	lp->net_stats.tx_errors++;

	/*
	 * Make sure that no interrupts come in that could cause reentrancy
	 * problems in reset.
	 */
	spin_lock_irqsave(&XE_spinlock, flags);
	xenet_reset(ndev, UNKNOWN_DUPLEX);
	spin_unlock_irqrestore(&XE_spinlock, flags);
}

/*
 * The callback function for errors.
 */
static void ErrorHandler(void *CallbackRef, XStatus Code)
{
	struct net_device *ndev = (struct net_device *)CallbackRef;
	int need_reset;
	need_reset = status_requires_reset(Code);

	printk(KERN_ERR "%s: device error %d%s\n",
	       ndev->name, Code, need_reset ? ", resetting device." : "");
	if (need_reset)
		xenet_reset(ndev, UNKNOWN_DUPLEX);
}

/*
 * This routine is registered with the OS as the function to call when
 * the EMAC interrupts.  It in turn, calls the Xilinx OS independent
 * interrupt function.  There are different interrupt functions for FIFO
 * and scatter-gather so we just set a pointer (Isr) into our private
 * data so we don't have to figure it out here.  The Xilinx OS
 * independent interrupt function will in turn call any callbacks that
 * we have registered for various conditions.
 */
static irqreturn_t xenet_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct net_local *lp = netdev_priv(ndev);

	/* Call it. */
	(*(lp->Isr)) (&lp->Emac);

	return IRQ_HANDLED;
}

/**********************
 * net_device methods *
 **********************/

static int xenet_open(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	u32 Options;
	DUPLEX phy_duplex, mac_duplex;
	int phy_carrier;

	/*
	 * Just to be safe, stop the device first.  If the device is already
	 * stopped, an error will be returned.  In this case, we don't really
	 * care, so cast it to void to make it explicit.
	 */
	(void)XEmac_Stop(&lp->Emac);

	/* Set the MAC address each time opened. */
	if (XEmac_SetMacAddress(&lp->Emac, ndev->dev_addr) != XST_SUCCESS) {
		printk(KERN_ERR "%s: Could not set MAC address.\n", ndev->name);
		return -EIO;
	}

	/*
	 * If the device is not configured for polled mode, connect to the
	 * interrupt controller and enable interrupts.  Currently, there
	 * isn't any code to set polled mode, so this check is probably
	 * superfluous.
	 */
	Options = XEmac_GetOptions(&lp->Emac);
	if ((Options & XEM_POLLED_OPTION) == 0) {
		int retval;
		/* Grab the IRQ */
		retval =
		    request_irq(ndev->irq, &xenet_interrupt, 0, ndev->name,
				ndev);
		if (retval) {
			printk(KERN_ERR
			       "%s: Could not allocate interrupt %d.\n",
			       ndev->name, ndev->irq);
			return retval;
		}
	}

	/* Set the EMAC's duplex setting based upon what the PHY says. */
	if (!get_phy_status(ndev, &phy_duplex, &phy_carrier)) {
		/* We successfully got the PHY status. */
		mac_duplex = ((Options & XEM_FDUPLEX_OPTION)
			      ? FULL_DUPLEX : HALF_DUPLEX);
		if (mac_duplex != phy_duplex) {
			switch (phy_duplex) {
			case HALF_DUPLEX:
				Options &= ~XEM_FDUPLEX_OPTION;
				break;
			case FULL_DUPLEX:
				Options |= XEM_FDUPLEX_OPTION;
				break;
			case UNKNOWN_DUPLEX:
				break;
			}
		}
	}
	/*
	 * The following function will return an error if the EMAC is already
	 * started.  We know it isn't started so we can safely ignore the
	 * return value.  We cast it to void to make that explicit.
	 */
	(void)XEmac_SetOptions(&lp->Emac, Options | XEM_FLOW_CONTROL_OPTION);

	if (XEmac_Start(&lp->Emac) != XST_SUCCESS) {
		printk(KERN_ERR "%s: Could not start device.\n", ndev->name);
		free_irq(ndev->irq, ndev);
		return -EBUSY;
	}

	/* We're ready to go. */
	netif_start_queue(ndev);

	/* Set up the PHY monitoring timer. */
	lp->phy_timer.expires = jiffies + 2 * HZ;
	lp->phy_timer.data = (unsigned long)ndev;
	lp->phy_timer.function = &poll_mii;
	add_timer(&lp->phy_timer);

	return 0;
}

static int xenet_close(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);

	/* Shut down the PHY monitoring timer. */
	del_timer_sync(&lp->phy_timer);

	netif_stop_queue(ndev);

	/*
	 * If not in polled mode, free the interrupt.  Currently, there
	 * isn't any code to set polled mode, so this check is probably
	 * superfluous.
	 */
	if ((XEmac_GetOptions(&lp->Emac) & XEM_POLLED_OPTION) == 0)
		free_irq(ndev->irq, ndev);

	if (XEmac_Stop(&lp->Emac) != XST_SUCCESS) {
		printk(KERN_ERR "%s: Could not stop device.\n", ndev->name);
		return -EBUSY;
	}

	return 0;
}

static struct net_device_stats *xenet_get_stats(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	return &lp->net_stats;
}

static void xenet_set_multicast_list(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);
	u32 Options;

	/*
	 * XEmac_Start, XEmac_Stop and XEmac_SetOptions are supposed to
	 * be protected by a semaphore.  This Linux adapter doesn't have
	 * it as bad as the VxWorks adapter because the sequence of
	 * requests to us is much more sequential.  However, we do have
	 * one area in which this is a problem.
	 *
	 * xenet_set_multicast_list() is called while the link is up and
	 * interrupts are enabled, so at any point in time we could get
	 * an error that causes our reset() to be called.  reset() calls
	 * the aforementioned functions, and we need to call them from
	 * here as well.
	 *
	 * The solution is to make sure that we don't get interrupts or
	 * timers popping while we are in this function.
	 */
	disable_irq(ndev->irq);
	local_bh_disable();

	/*
	 * The dev's set_multicast_list function is only called when
	 * the device is up.  So, without checking, we know we need to
	 * Stop and Start the XEmac because it has already been
	 * started.  XEmac_Stop() will return an error if it is already
	 * stopped, but in this case we don't care so cast it to void
	 * to make it explicit
	 */
	(void)XEmac_Stop(&lp->Emac);

	Options = XEmac_GetOptions(&lp->Emac);

	/* Clear out the bits we may set. */
	Options &= ~(XEM_PROMISC_OPTION | XEM_MULTICAST_OPTION);

	if (ndev->flags & IFF_PROMISC)
		Options |= XEM_PROMISC_OPTION;
#if 0
	else {
		/*
		 * SAATODO: Xilinx is going to add multicast support to their
		 * VxWorks adapter and OS independent layer.  After that is
		 * done, this skeleton code should be fleshed out.  Note that
		 * IFF_MULTICAST is being masked out from dev->flags in probe,
		 * so that will need to be removed to actually do multidrop.
		 */
		if ((ndev->flags & IFF_ALLMULTI)
		    || ndev->mc_count > MAX_MULTICAST ? ? ?) {
			xemac_get_all_multicast ? ? ? ();
			Options |= XEM_MULTICAST_OPTION;
		} else if (ndev->mc_count != 0) {
			struct dev_mc_list *mc;

			XEmac_MulticastClear(&lp->Emac);
			for (mc = ndev->mc_list; mc; mc = mc->next)
				XEmac_MulticastAdd(&lp->Emac, mc->dmi_addr);
			Options |= XEM_MULTICAST_OPTION;
		}
	}
#endif

	/*
	 * The following function will return an error if the EMAC is already
	 * started.  We know it isn't started so we can safely ignore the
	 * return value.  We cast it to void to make that explicit.
	 */
	(void)XEmac_SetOptions(&lp->Emac, Options);

	/*
	 * XEmac_Start returns an error when: it is already started, the send
	 * and receive handlers are not set, or a scatter-gather DMA list is
	 * missing.  None of these can happen at this point, so we cast the
	 * return to void to make that explicit.
	 */
	(void)XEmac_Start(&lp->Emac);

	/* All done, get those interrupts and timers going again. */
	local_bh_enable();
	enable_irq(ndev->irq);
}

static int xenet_ethtool_get_settings(struct net_device *dev,
				      struct ethtool_cmd *ecmd)
{
	int ret;
	struct net_local *lp = (struct net_local *)dev->priv;
	u32 mac_options;
	u8 threshold;
	u16 mii_cmd;
	u16 mii_status;
	u16 mii_advControl;
	XStatus xs;

	memset(ecmd, 0, sizeof(struct ethtool_cmd));
	mac_options = XEmac_GetOptions(&(lp->Emac));
	xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr, MII_BMCR, &mii_cmd);
	if (xs != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: Could not read mii command register; error %d\n",
		       dev->name, xs);
		return -1;
	}
	xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr, MII_BMSR, &mii_status);
	if (xs != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: Could not read mii status register; error %d\n",
		       dev->name, xs);
		return -1;
	}
	xs = XEmac_PhyRead(&lp->Emac, lp->mii_addr, MII_ADVERTISE,
			   &mii_advControl);
	if (xs != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: Could not read mii advertisement control register; error %d\n",
		       dev->name, xs);
		return -1;
	}

	if (mac_options & XEM_FDUPLEX_OPTION)
		ecmd->duplex = DUPLEX_FULL;
	else
		ecmd->duplex = DUPLEX_HALF;
	if (mii_status & BMSR_100FULL)
		ecmd->supported |= SUPPORTED_100baseT_Full;
	if (mii_status & BMSR_100HALF)
		ecmd->supported |= SUPPORTED_100baseT_Half;
	if (mii_status & BMSR_10FULL)
		ecmd->supported |= SUPPORTED_10baseT_Full;
	if (mii_status & BMSR_10HALF)
		ecmd->supported |= SUPPORTED_10baseT_Half;
	ecmd->supported |= SUPPORTED_MII;
	if (mii_status & BMSR_ANEGCAPABLE)
		ecmd->supported |= SUPPORTED_Autoneg;
	if (mii_status & BMSR_ANEGCOMPLETE) {
		ecmd->autoneg = AUTONEG_ENABLE;
		ecmd->advertising |= ADVERTISED_Autoneg;
		if ((mii_advControl & ADVERTISE_100FULL)
		    || (mii_advControl & ADVERTISE_100HALF))
			ecmd->speed = SPEED_100;
		else
			ecmd->speed = SPEED_10;
	} else {
		ecmd->autoneg = AUTONEG_DISABLE;
		if (mii_cmd & BMCR_SPEED100)
			ecmd->speed = SPEED_100;
		else
			ecmd->speed = SPEED_10;
	}
	if (mii_advControl & ADVERTISE_10FULL)
		ecmd->advertising |= ADVERTISED_10baseT_Full;
	if (mii_advControl & ADVERTISE_10HALF)
		ecmd->advertising |= ADVERTISED_10baseT_Half;
	if (mii_advControl & ADVERTISE_100FULL)
		ecmd->advertising |= ADVERTISED_100baseT_Full;
	if (mii_advControl & ADVERTISE_100HALF)
		ecmd->advertising |= ADVERTISED_100baseT_Half;
	ecmd->advertising |= ADVERTISED_MII;
	ecmd->port = PORT_MII;
	ecmd->phy_address = lp->Emac.PhysAddress;
	ecmd->transceiver = XCVR_INTERNAL;
	if (XEmac_mIsSgDma(&lp->Emac)) {
		if ((ret =
		     XEmac_GetPktThreshold(&lp->Emac, XEM_SEND,
					   &threshold)) == XST_SUCCESS) {
			ecmd->maxtxpkt = threshold;
		} else
			return -EIO;
		if ((ret =
		     XEmac_GetPktThreshold(&lp->Emac, XEM_RECV,
					   &threshold)) == XST_SUCCESS) {
			ecmd->maxrxpkt = threshold;
		} else
			return -EIO;
	}
	return 0;
}

static int xenet_ethtool_get_coalesce(struct net_device *dev,
				      struct ethtool_coalesce *ec)
{
	int ret;
	struct net_local *lp = (struct net_local *)dev->priv;
	u8 threshold;

	memset(ec, 0, sizeof(struct ethtool_coalesce));
	if ((ret =
	     XEmac_GetPktThreshold(&lp->Emac, XEM_RECV,
				   &threshold)) != XST_SUCCESS) {
		printk(KERN_INFO "XEmac_GetPktThreshold error %d\n", ret);
		return -EIO;
	}
	ec->rx_max_coalesced_frames = threshold;
	if ((ret =
	     XEmac_GetPktWaitBound(&lp->Emac, XEM_RECV,
				   &(ec->rx_coalesce_usecs))) != XST_SUCCESS) {
		printk(KERN_INFO "XEmac_GetPktWaitBound error %d\n", ret);
		return -EIO;
	}
	if ((ret =
	     XEmac_GetPktThreshold(&lp->Emac, XEM_SEND,
				   &threshold)) != XST_SUCCESS) {
		printk(KERN_INFO "XEmac_GetPktThreshold send error %d\n", ret);
		return -EIO;
	}
	ec->tx_max_coalesced_frames = threshold;
	if ((ret =
	     XEmac_GetPktWaitBound(&lp->Emac, XEM_SEND,
				   &(ec->tx_coalesce_usecs))) != XST_SUCCESS) {
		printk(KERN_INFO "XEmac_GetPktWaitBound send error %d\n", ret);
		return -EIO;
	}
	return 0;
}

static int xenet_ethtool_set_coalesce(struct net_device *dev,
				      struct ethtool_coalesce *ec)
{
	int ret;
	struct net_local *lp = (struct net_local *)dev->priv;
	unsigned long flags;

	spin_lock_irqsave(&XE_spinlock, flags);
	if ((ret = XEmac_Stop(&lp->Emac)) != XST_SUCCESS) {
		spin_unlock_irqrestore(&XE_spinlock, flags);
		return -EIO;
	}
	if ((ret =
	     XEmac_SetPktThreshold(&lp->Emac, XEM_RECV,
				   ec->rx_max_coalesced_frames)) !=
	    XST_SUCCESS) {
		spin_unlock_irqrestore(&XE_spinlock, flags);
		printk(KERN_INFO "XEmac_SetPktThreshold error %d\n", ret);
		return -EIO;
	}
	if ((ret =
	     XEmac_SetPktWaitBound(&lp->Emac, XEM_RECV,
				   ec->rx_coalesce_usecs)) != XST_SUCCESS) {
		spin_unlock_irqrestore(&XE_spinlock, flags);
		printk(KERN_INFO "XEmac_SetPktWaitBound error %d\n", ret);
		return -EIO;
	}
	if ((ret =
	     XEmac_SetPktThreshold(&lp->Emac, XEM_SEND,
				   ec->tx_max_coalesced_frames)) !=
	    XST_SUCCESS) {
		spin_unlock_irqrestore(&XE_spinlock, flags);
		printk(KERN_INFO "XEmac_SetPktThreshold send error %d\n", ret);
		return -EIO;
	}
	if ((ret =
	     XEmac_SetPktWaitBound(&lp->Emac, XEM_SEND,
				   ec->tx_coalesce_usecs)) != XST_SUCCESS) {
		spin_unlock_irqrestore(&XE_spinlock, flags);
		printk(KERN_INFO "XEmac_SetPktWaitBound send error %d\n", ret);
		return -EIO;
	}
	if ((ret = XEmac_Start(&lp->Emac)) != XST_SUCCESS) {
		spin_unlock_irqrestore(&XE_spinlock, flags);
		return -EIO;
	}
	spin_unlock_irqrestore(&XE_spinlock, flags);
	return 0;
}

static int xenet_ethtool_get_drvinfo(struct net_device *dev,
				     struct ethtool_drvinfo *ed)
{
	memset(ed, 0, sizeof(struct ethtool_drvinfo));
	strcpy(ed->driver, DRIVER_NAME);
	strcpy(ed->version, DRIVER_VERSION);
	return 0;
}

static int xenet_ethtool_get_ringparam(struct net_device *dev,
				       struct ethtool_ringparam *erp)
{
	memset(erp, 0, sizeof(struct ethtool_ringparam));
	erp->rx_max_pending = XEM_DFT_RECV_DESC;
	erp->tx_max_pending = XEM_DFT_SEND_DESC;
	erp->rx_pending = XEM_DFT_RECV_DESC;
	erp->tx_pending = XEM_DFT_SEND_DESC;
	return 0;
}

#define EMAG_REGS_N	32
struct mac_regsDump {
	struct ethtool_regs hd;
	u16 data[EMAG_REGS_N];
};

static void xenet_ethtool_get_regs(struct net_device *dev,
				   struct ethtool_regs *regs, void *ret)
{
	struct net_local *lp = (struct net_local *)dev->priv;
	struct mac_regsDump *dump = (struct mac_regsDump *)regs;
	int i;
	XStatus r;

	dump->hd.version = 0;
	dump->hd.len = EMAG_REGS_N * sizeof(dump->data);
	for (i = 0; i < EMAG_REGS_N; i++) {
		if ((r =
		     XEmac_PhyRead(&(lp->Emac), lp->mii_addr, i,
				   &(dump->data[i]))) != XST_SUCCESS) {
			printk(KERN_INFO "PhyRead ERROR %d\n", r);
			*(int *)ret = -EIO;
			return;
		}
	}
	*(int *)ret = 0;
}

static int xenet_do_ethtool_ioctl(struct net_device *dev, struct ifreq *rq)
{
	struct net_local *lp = (struct net_local *)dev->priv;
	struct ethtool_cmd ecmd;
	struct ethtool_coalesce eco;
	struct ethtool_drvinfo edrv;
	struct ethtool_ringparam erp;
	struct ethtool_pauseparam epp;
	struct mac_regsDump regs;
	int ret = -EOPNOTSUPP;
	XStatus result;
	u32 Options;
	u16 mii_reg_sset;
	u16 mii_reg_spause;
	u16 mii_reg_autoneg;
	u32 flags;

	if (copy_from_user(&ecmd, rq->ifr_data, sizeof(ecmd.cmd)))
		return -EFAULT;
	switch (ecmd.cmd) {
	case ETHTOOL_GSET:
		ret = xenet_ethtool_get_settings(dev, &ecmd);
		if (ret >= 0) {
			if (copy_to_user(rq->ifr_data, &ecmd, sizeof(ecmd)))
				ret = -EFAULT;
		}
		break;
	case ETHTOOL_SSET:
		if (copy_from_user
		    (&ecmd, rq->ifr_data, sizeof(struct ethtool_cmd)))
			return -EFAULT;
		mii_reg_sset = 0;
		if (ecmd.speed == SPEED_100)
			mii_reg_sset |= BMCR_SPEED100;
		if (ecmd.duplex == DUPLEX_FULL)
			mii_reg_sset |= BMCR_FULLDPLX;
		if (ecmd.autoneg == AUTONEG_ENABLE) {
			mii_reg_sset |= (BMCR_ANENABLE | BMCR_ANRESTART);
			spin_lock_irqsave(&XE_spinlock, flags);
			result = XEmac_PhyWrite(&lp->Emac, lp->mii_addr,
						MII_BMCR, mii_reg_sset);
			if (result != XST_SUCCESS) {
				spin_unlock_irqrestore(&XE_spinlock, flags);
				ret = -EIO;
				break;
			}
			result =
			    XEmac_PhyRead(&lp->Emac, lp->mii_addr,
					  MII_ADVERTISE, &mii_reg_sset);
			if (result != XST_SUCCESS) {
				spin_unlock_irqrestore(&XE_spinlock, flags);
				ret = -EIO;
				break;
			}
			if (ecmd.speed == SPEED_100) {
				if (ecmd.duplex == DUPLEX_FULL) {
					mii_reg_sset |=
					    (ADVERTISE_10FULL |
					     ADVERTISE_100FULL |
					     ADVERTISE_10HALF |
					     ADVERTISE_100HALF);
				} else {
					mii_reg_sset |=
					    (ADVERTISE_10HALF |
					     ADVERTISE_100HALF);
					mii_reg_sset &=
					    ~(ADVERTISE_10FULL |
					      ADVERTISE_100FULL);
				}
			} else {
				if (ecmd.duplex == DUPLEX_FULL) {
					mii_reg_sset |=
					    (ADVERTISE_10FULL |
					     ADVERTISE_10HALF);
					mii_reg_sset &=
					    ~(ADVERTISE_100FULL |
					      ADVERTISE_100HALF);
				} else {
					mii_reg_sset |= (ADVERTISE_10HALF);
					mii_reg_sset &=
					    ~(ADVERTISE_100FULL |
					      ADVERTISE_100HALF |
					      ADVERTISE_10FULL);
				}
			}
			result =
			    XEmac_PhyWrite(&lp->Emac, lp->mii_addr,
					   MII_ADVERTISE, mii_reg_sset);
			spin_unlock_irqrestore(&XE_spinlock, flags);
			if (result != XST_SUCCESS) {
				ret = -EIO;
				break;
			}
		} else {
			mii_reg_sset &= ~(BMCR_ANENABLE | BMCR_ANRESTART);
			if (ecmd.duplex == DUPLEX_FULL) {
				mii_reg_sset |= BMCR_FULLDPLX;
			} else {
				mii_reg_sset &= ~BMCR_FULLDPLX;
			}
			if (ecmd.speed == SPEED_100) {
				mii_reg_sset |= BMCR_SPEED100;
			} else {
				mii_reg_sset &= ~BMCR_SPEED100;
			}
			spin_lock_irqsave(&XE_spinlock, flags);
			result = XEmac_PhyWrite(&lp->Emac, lp->mii_addr,
						MII_BMCR, mii_reg_sset);
			spin_unlock_irqrestore(&XE_spinlock, flags);
			if (result != XST_SUCCESS) {
				ret = -EIO;
				break;
			}
		}
		ret = 0;
		break;
	case ETHTOOL_GPAUSEPARAM:
		ret = xenet_ethtool_get_settings(dev, &ecmd);
		if (ret < 0) {
			break;
		}
		epp.cmd = ecmd.cmd;
		epp.autoneg = ecmd.autoneg;
		Options = XEmac_GetOptions(&lp->Emac);
		if (Options & XEM_INSERT_PAD_OPTION) {
			epp.rx_pause = 1;
			epp.tx_pause = 1;
		} else {
			epp.rx_pause = 0;
			epp.tx_pause = 0;
		}
		if (copy_to_user
		    (rq->ifr_data, &epp, sizeof(struct ethtool_pauseparam)))
			ret = -EFAULT;
		else
			ret = 0;
		break;
	case ETHTOOL_SPAUSEPARAM:
		if (copy_from_user
		    (&epp, rq->ifr_data, sizeof(struct ethtool_pauseparam)))
			return -EFAULT;
		ret = xenet_ethtool_get_settings(dev, &ecmd);
		if (ret < 0) {
			break;
		}
		epp.cmd = ecmd.cmd;
		mii_reg_spause = 0;
		if (epp.autoneg == AUTONEG_ENABLE) {
			mii_reg_spause |= (BMCR_ANENABLE | BMCR_ANRESTART);
		} else {
			if (ecmd.speed == SPEED_100)
				mii_reg_spause |= BMCR_SPEED100;
			if (ecmd.duplex == DUPLEX_FULL)
				mii_reg_spause |= BMCR_FULLDPLX;
		}
		spin_lock_irqsave(&XE_spinlock, flags);
		result = XEmac_PhyWrite(&lp->Emac, lp->mii_addr,
					MII_BMCR, mii_reg_spause);
		spin_unlock_irqrestore(&XE_spinlock, flags);
		if (result != XST_SUCCESS) {
			ret = -EIO;
			break;
		}
		if (epp.rx_pause != epp.tx_pause) {
			ret = 0;
			break;
		} else {
			spin_lock_irqsave(&XE_spinlock, flags);
			(void)XEmac_Stop(&(lp->Emac));
			Options = XEmac_GetOptions(&lp->Emac);
			if (epp.rx_pause)
				Options |= XEM_INSERT_PAD_OPTION;
			else
				Options &= ~XEM_INSERT_PAD_OPTION;
			(void)XEmac_SetOptions(&lp->Emac, Options);
			(void)XEmac_Start(&(lp->Emac));
			spin_unlock_irqrestore(&XE_spinlock, flags);
		}
		ret = 0;
		break;
	case ETHTOOL_GCOALESCE:
		eco.cmd = ecmd.cmd;
		ret = xenet_ethtool_get_coalesce(dev, &eco);
		if (ret >= 0) {
			if (copy_to_user
			    (rq->ifr_data, &eco,
			     sizeof(struct ethtool_coalesce)))
				ret = -EFAULT;
		}
		break;
	case ETHTOOL_SCOALESCE:
		if (copy_from_user
		    (&eco, rq->ifr_data, sizeof(struct ethtool_coalesce)))
			return -EFAULT;
		ret = xenet_ethtool_set_coalesce(dev, &eco);
		break;
	case ETHTOOL_GDRVINFO:
		edrv.cmd = edrv.cmd;
		ret = xenet_ethtool_get_drvinfo(dev, &edrv);
		if (ret >= 0) {
			if (copy_to_user
			    (rq->ifr_data, &edrv,
			     sizeof(struct ethtool_drvinfo)))
				ret = -EFAULT;
		}
		break;
	case ETHTOOL_GREGS:
		regs.hd.cmd = edrv.cmd;
		xenet_ethtool_get_regs(dev, &(regs.hd), &ret);
		if (ret >= 0) {
			if (copy_to_user
			    (rq->ifr_data, &regs, sizeof(struct mac_regsDump)))
				ret = -EFAULT;
		}
		break;
	case ETHTOOL_GRINGPARAM:
		erp.cmd = edrv.cmd;
		ret = xenet_ethtool_get_ringparam(dev, &(erp));
		if (ret >= 0) {
			if (copy_to_user
			    (rq->ifr_data, &erp,
			     sizeof(struct ethtool_ringparam)))
				ret = -EFAULT;
		}
		break;
	case ETHTOOL_NWAY_RST:
		epp.cmd = ecmd.cmd;
		mii_reg_autoneg = 0;
		mii_reg_autoneg |= (BMCR_ANENABLE | BMCR_ANRESTART);
		spin_lock_irqsave(&XE_spinlock, flags);
		result = XEmac_PhyWrite(&lp->Emac, lp->mii_addr,
					MII_BMCR, mii_reg_autoneg);
		spin_unlock_irqrestore(&XE_spinlock, flags);
		if (result != XST_SUCCESS) {
			ret = -EIO;
			break;
		}
		ret = 0;
		break;
	default:
		break;
	}
	return ret;
}

static int xenet_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct net_local *lp = netdev_priv(ndev);
	/* mii_ioctl_data has 4 u16 fields: phy_id, reg_num, val_in & val_out */
	struct mii_ioctl_data *data = (struct mii_ioctl_data *)&ifr->ifr_data;

	XStatus Result;

	switch (cmd) {
	case SIOCETHTOOL:
		return xenet_do_ethtool_ioctl(ndev, ifr);
	case SIOCGMIIPHY:	/* Get address of MII PHY in use. */
		data->phy_id = lp->mii_addr;
		/* Fall Through */

	case SIOCGMIIREG:	/* Read MII PHY register. */
		if (data->phy_id > 31 || data->reg_num > 31)
			return -ENXIO;

		/* Stop the PHY timer to prevent reentrancy. */
		del_timer_sync(&lp->phy_timer);
		Result = XEmac_PhyRead(&lp->Emac, data->phy_id,
				       data->reg_num, &data->val_out);
		/* Start the PHY timer up again. */
		lp->phy_timer.expires = jiffies + 2 * HZ;
		add_timer(&lp->phy_timer);

		if (Result != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: Could not read from PHY, error=%d.\n",
			       ndev->name, Result);
			return (Result == XST_EMAC_MII_BUSY) ? -EBUSY : -EIO;
		}
		return 0;

	case SIOCSMIIREG:	/* Write MII PHY register. */
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		if (data->phy_id > 31 || data->reg_num > 31)
			return -ENXIO;

		/* Stop the PHY timer to prevent reentrancy. */
		del_timer_sync(&lp->phy_timer);
		Result = XEmac_PhyWrite(&lp->Emac, data->phy_id,
					data->reg_num, data->val_in);
		/* Start the PHY timer up again. */
		lp->phy_timer.expires = jiffies + 2 * HZ;
		add_timer(&lp->phy_timer);

		if (Result != XST_SUCCESS) {
			printk(KERN_ERR
			       "%s: Could not write to PHY, error=%d.\n",
			       ndev->name, Result);
			return (Result == XST_EMAC_MII_BUSY) ? -EBUSY : -EIO;
		}
		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

/*********************
 * The device driver *
 *********************/

static void xenet_remove_ndev(struct net_device *ndev)
{
	struct net_local *lp = netdev_priv(ndev);

	iounmap((void *)lp->Emac.BaseAddress);

	/* Free up the memory. */
	if (lp->saved_skb)
		dev_kfree_skb(lp->saved_skb);
	free_netdev(ndev);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void xenet_netpoll(struct net_device *netdev)
{
	unsigned long flags;
	spin_lock_irqsave(&XE_spinlock, flags);
	xenet_interrupt(netdev->irq, netdev, NULL);
	spin_unlock_irqrestore(&XE_spinlock, flags);
}
#endif

static int xenet_probe(struct device *dev)
{
	struct net_device *ndev;
	struct net_local *lp;
	struct platform_device *pdev = to_platform_device(dev);
	struct xemac_platform_data *pdata;
	struct resource *r_irq, *r_mem;
	XEmac_Config xemac_config;
	u32 maddr;
	int retval;

	pdata = (struct xemac_platform_data *)pdev->dev.platform_data;

	if (pdata == NULL) {
		printk(KERN_ERR "xemac %d: Couldn't find platform data.\n",
		       pdev->id);

		return -ENODEV;
	}
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_irq || !r_mem) {
		printk(KERN_ERR "xemac %d: IO resource(s) not found.\n",
		       pdev->id);

		return -ENODEV;
	}

	/* Create an ethernet device instance */
	ndev = alloc_etherdev(sizeof(struct net_local));
	if (ndev == NULL) {
		printk(KERN_ERR "xemac %d: Could not allocate net device.\n",
		       pdev->id);
		return -ENOMEM;
	}
	dev_set_drvdata(dev, ndev);
	ndev->irq = r_irq->start;

	/* Initialize the private data used by XEmac_LookupConfig().
	 * The private data are zeroed out by alloc_etherdev() already.
	 */
	lp = netdev_priv(ndev);
	xemac_config.DeviceId = pdev->id;
	spin_lock_init(&lp->skb_lock);
	spin_lock_init(&XE_spinlock);
	init_timer(&lp->phy_timer);

	/* Initialize the rest of the private data */
        xemac_config.PhysAddress = r_mem->start;
        xemac_config.BaseAddress = (u32)ioremap(r_mem->start,
                                                r_mem->end - r_mem->start + 1);
        xemac_config.IpIfDmaConfig = pdata->dma_mode;
        xemac_config.HasCounters = (pdata->device_flags & XEMAC_HAS_ERR_COUNT) ?
	    1 : 0;
	xemac_config.HasMii = (pdata->device_flags & XEMAC_HAS_MII) ? 1 : 0;

        if (XEmac_CfgInitialize(&lp->Emac, &xemac_config,
                                xemac_config.BaseAddress) != XST_SUCCESS) {
		printk(KERN_ERR "%s: Could not initialize device.\n",
		       ndev->name);
		xenet_remove_ndev(ndev);
		return -ENODEV;
	}

	memcpy(ndev->dev_addr, pdata->mac_addr, 6);
	if (XEmac_SetMacAddress(&lp->Emac, ndev->dev_addr) != XST_SUCCESS) {
		/* should not fail right after an initialize */
		printk(KERN_ERR "%s: Could not set MAC address.\n", ndev->name);
		xenet_remove_ndev(ndev);
		return -EIO;
	}

	if (XEmac_mIsSgDma(&lp->Emac)) {
		printk(KERN_ERR "%s: using sgDMA mode.\n", ndev->name);
		XEmac_SetSgRecvHandler(&lp->Emac, ndev, SgRecvHandler);
		XEmac_SetSgSendHandler(&lp->Emac, ndev, SgSendHandler);
		ndev->hard_start_xmit = xenet_start_xmit_sgdma;
		lp->Isr = XEmac_IntrHandlerDma;

		retval = sgdma_descriptor_init(ndev);
		if (retval) {
			xenet_remove_ndev(ndev);
			return -EIO;
		}

		/* set the packet threshold and timeout */
		XEmac_SetPktThreshold(&lp->Emac, XEM_SEND,
				      SGDMA_DEFAULT_THRESHOLD);
		XEmac_SetPktThreshold(&lp->Emac, XEM_RECV,
				      SGDMA_DEFAULT_THRESHOLD);
		(void)XEmac_SetPktWaitBound(&lp->Emac, XEM_SEND,
					    SGDMA_DEFAULT_WAITBOUND);
		(void)XEmac_SetPktWaitBound(&lp->Emac, XEM_RECV,
					    SGDMA_DEFAULT_WAITBOUND);

		/* disable SGEND interrupt */
		XEmac_SetOptions(&lp->Emac, XEmac_GetOptions(&lp->Emac) |
				 XEM_NO_SGEND_INT_OPTION);
	} else {
		printk(KERN_ERR "xemac %d: using fifo mode.\n", pdev->id);
		XEmac_SetFifoRecvHandler(&lp->Emac, ndev, FifoRecvHandler);
		XEmac_SetFifoSendHandler(&lp->Emac, ndev, FifoSendHandler);
		ndev->hard_start_xmit = xenet_start_xmit_fifo;
		lp->Isr = XEmac_IntrHandlerFifo;
	}
	XEmac_SetErrorHandler(&lp->Emac, ndev, ErrorHandler);

	/* Scan to find the PHY. */
	lp->mii_addr = 0xFF;
	for (maddr = 0; maddr < 31; maddr++) {
		XStatus Result;
		u16 reg;

		Result = XEmac_PhyRead(&lp->Emac, maddr, MII_BMCR, &reg);
		/*
		 * XEmac_PhyRead is currently returning XST_SUCCESS even
		 * when reading from non-existent addresses.  Work
		 * around this by doing a primitive validation on the
		 * control word we get back.
		 */
		if (Result == XST_SUCCESS && reg != 0xffff && reg != 0) {
			lp->mii_addr = maddr;
			break;
		}
	}
	if (lp->mii_addr == 0xFF) {
		lp->mii_addr = 0;
		printk(KERN_WARNING
		       "%s: No PHY detected.  Assuming a PHY at address %d.\n",
		       ndev->name, lp->mii_addr);
	}

	SET_NETDEV_DEV(ndev, dev);

	ndev->open = xenet_open;
	ndev->stop = xenet_close;
	ndev->get_stats = xenet_get_stats;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->set_multicast_list = xenet_set_multicast_list;
	ndev->do_ioctl = xenet_ioctl;
	ndev->tx_timeout = xenet_tx_timeout;
	ndev->watchdog_timeo = TX_TIMEOUT;

#ifdef CONFIG_NET_POLL_CONTROLLER
	ndev->poll_controller = xenet_netpoll;
#endif

	retval = register_netdev(ndev);
	if (retval) {
		printk(KERN_ERR
		       "%s: Cannot register net device, aborting.\n",
		       ndev->name);
		xenet_remove_ndev(ndev);
		return retval;
	}

	printk(KERN_INFO
	       "%s: Xilinx EMAC #%d at 0x%08X mapped to 0x%08X, irq=%d\n",
               ndev->name, xemac_config.DeviceId,
               xemac_config.PhysAddress, xemac_config.BaseAddress, ndev->irq);
	return 0;
}

static int xenet_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	unregister_netdev(ndev);
	xenet_remove_ndev(ndev);

	return 0;		/* success */
}

static struct device_driver xenet_driver = {
	.name = DRIVER_NAME,
	.bus = &platform_bus_type,

	.probe = xenet_probe,
	.remove = xenet_remove
};

static int __init xenet_init(void)
{
	/*
	 * No kernel boot options used,
	 * so we just need to register the driver
	 */
	return driver_register(&xenet_driver);
}

static void __exit xenet_cleanup(void)
{
	driver_unregister(&xenet_driver);
}

module_init(xenet_init);
module_exit(xenet_cleanup);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");
