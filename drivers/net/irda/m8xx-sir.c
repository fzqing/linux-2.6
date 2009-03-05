/*
 * linux/drivers/net/irda/m8xx_ir.c
 *
 * Infra-red SIR driver for the MPC8xx processors.
 *
 * 2005 (c) MontaVista Software, Inc.
 * Author: Yuri Shpilevsky <source@mvista.com>
 *
 * Based on linux/drivers/net/irda/omap1610-ir.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/8xx_immap.h>
#include <asm/pgtable.h>
#include <asm/mpc8xx.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/commproc.h>

/*
 * Our netdevice.  There is only ever one of these.
 */

#define CPM_IRDA_RX_PAGES       4
#define CPM_IRDA_RX_FRSIZE      2048
#define CPM_IRDA_RX_FRPPG       (PAGE_SIZE / CPM_IRDA_RX_FRSIZE)
#define RX_RING_SIZE            (CPM_IRDA_RX_FRPPG * CPM_IRDA_RX_PAGES)
#define TX_RING_SIZE            8	/* Must be power of two */
#define TX_RING_MOD_MASK        7

struct mpc8xx_irda {
	unsigned char open;

	int speed;
	int newspeed;

	/* The saved address of a sent-in-place packet/buffer, for skfree().
	 */
	struct sk_buff *tx_skbuff[TX_RING_SIZE];
	ushort skb_cur;
	ushort skb_dirty;

	struct net_device_stats stats;
	struct device *dev;
	struct irda_platform_data *pdata;
	struct irlap_cb *irlap;
	struct qos_info qos;

	scc_t *sccp;
	scc_ahdlc_t *ahp;
	int irq;
	cbd_t *rx_bd_base;
	cbd_t *tx_bd_base;
	cbd_t *dirty_tx;
	cbd_t *cur_rx, *cur_tx;	/* The next free ring entry */
	unsigned char *rx_vaddr[RX_RING_SIZE];
	int tx_free;
	spinlock_t lock;
};

#define HPSIR_MAX_RXLEN		2050
#define HPSIR_MAX_TXLEN		2050
#define TXBUFF_MAX_SIZE		HPSIR_MAX_TXLEN

static void mpc8xx_irda_set_speed(struct net_device *dev, int speed);

/************************************************************************************/

/* Low level init/uninstall function IrDA protocol stack registration
 */

static void mpc8xx_irda_rx(struct net_device *dev)
{
	struct mpc8xx_irda *si = dev->priv;
	cbd_t *bdp;
	struct sk_buff *skb;
	int len;
	ushort status;

	bdp = si->cur_rx;

	for (;;) {
		if (bdp->cbd_sc & BD_AHDLC_RX_EMPTY)
			break;
		status = bdp->cbd_sc;

		if (status & BD_AHDLC_RX_STATS) {
			/* process errors
			 */
			if (bdp->cbd_sc & BD_AHDLC_RX_AB)
				si->stats.rx_length_errors++;
			if (bdp->cbd_sc & BD_AHDLC_RX_CR)	/* CRC Error */
				si->stats.rx_crc_errors++;
			if (bdp->cbd_sc & BD_AHDLC_RX_OV)	/* FIFO overrun */
				si->stats.rx_over_errors++;
		} else {
			/* Process the incoming frame.
			 */
			len = bdp->cbd_datlen;

			skb = dev_alloc_skb(len + 1);
			if (skb == NULL) {
				printk(KERN_INFO
				       "%s: Memory squeeze, dropping packet.\n",
				       dev->name);
				si->stats.rx_dropped++;
			} else {
				skb->dev = dev;
				skb_reserve(skb, 1);
				memcpy(skb_put(skb, len),
				       si->rx_vaddr[bdp - si->rx_bd_base], len);
				skb_trim(skb, skb->len - 2);

				si->stats.rx_packets++;
				si->stats.rx_bytes += len;

				skb->mac.raw = skb->data;
				skb->protocol = htons(ETH_P_IRDA);
				netif_rx(skb);
			}
		}

		/* Clear the status flags for this buffer.
		 */
		bdp->cbd_sc &= ~BD_AHDLC_RX_STATS;

		/* Mark the buffer empty.
		 */
		bdp->cbd_sc |= BD_AHDLC_RX_EMPTY;

		/* Update BD pointer to next entry.
		 */
		if (bdp->cbd_sc & BD_AHDLC_RX_WRAP)
			bdp = si->rx_bd_base;
		else
			bdp++;
	}
	si->cur_rx = (cbd_t *) bdp;
}

static irqreturn_t mpc8xx_irda_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct mpc8xx_irda *si = dev->priv;
	scc_t *sccp = si->sccp;
	cbd_t *bdp;
	ushort int_events;
	int must_restart = 0;

	/* Get the interrupt events that caused us to be here.
	 */
	int_events = in_be16(&sccp->scc_scce);
	out_be16(&sccp->scc_scce, int_events);

	/* Handle receive event in its own function.
	 */
	if (int_events & SCC_AHDLC_RXF)
		mpc8xx_irda_rx(dev);

	spin_lock(&si->lock);

	/* Transmit OK, or non-fatal error.  Update the buffer descriptors.
	 */
	if (int_events & (SCC_AHDLC_TXE | SCC_AHDLC_TXB)) {
		bdp = si->dirty_tx;

		while ((bdp->cbd_sc & BD_AHDLC_TX_READY) == 0) {
			if (si->tx_free == TX_RING_SIZE)
				break;

			if (bdp->cbd_sc & BD_AHDLC_TX_CTS)
				must_restart = 1;

			si->stats.tx_packets++;

			/* Free the sk buffer associated with this last transmit.
			 */
			dev_kfree_skb_irq(si->tx_skbuff[si->skb_dirty]);
			si->skb_dirty = (si->skb_dirty + 1) & TX_RING_MOD_MASK;

			/* Update pointer to next buffer descriptor to be transmitted.
			 */
			if (bdp->cbd_sc & BD_AHDLC_TX_WRAP)
				bdp = si->tx_bd_base;
			else
				bdp++;

			/* Since we have freed up a buffer, the ring is no longer full.
			 */
			if (!si->tx_free++) {
				if (netif_queue_stopped(dev))
					netif_wake_queue(dev);
			}

			si->dirty_tx = (cbd_t *) bdp;

			if (si->newspeed) {
				mpc8xx_irda_set_speed(dev, si->newspeed);
				si->speed = si->newspeed;
				si->newspeed = 0;
			}
		}

		if (must_restart) {
			cpm8xx_t *cp = cpmp;
			printk(KERN_INFO "restart TX\n");

			/* Some transmit errors cause the transmitter to shut
			 * down.  We now issue a restart transmit.  Since the
			 * errors close the BD and update the pointers, the restart
			 * _should_ pick up without having to reset any of our
			 * pointers either.
			 */
			out_be16(&cp->cp_cpcr,
			    mk_cr_cmd(CPM_CR_CH_SCC2,
				      CPM_CR_RESTART_TX) | CPM_CR_FLG);
			while (in_be16(&cp->cp_cpcr) & CPM_CR_FLG) ;
		}
	}

	spin_unlock(&si->lock);

	return IRQ_HANDLED;
}

static int mpc8xx_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mpc8xx_irda *si = dev->priv;
	volatile scc_ahdlc_t *ahp = si->ahp;
	int speed = irda_get_next_speed(skb);
	cbd_t *bdp;
	int mtt;
	uint xbofs, flags;

	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != si->speed && speed != -1)
		si->newspeed = speed;

	mtt = irda_get_mtt(skb);

	spin_lock_irqsave(&si->lock, flags);

	/*
	 * If this is an empty frame, we can bypass a lot.
	 */
	if (skb->len == 0) {
		if (si->newspeed) {
			si->newspeed = 0;
			mpc8xx_irda_set_speed(dev, speed);
		}
		spin_unlock_irqrestore(&si->lock, flags);
		dev_kfree_skb(skb);
		return 0;
	}

	/* Get a Tx ring entry */
	bdp = si->cur_tx;

	/* Clear all of the status flags.
	 */
	bdp->cbd_sc &= ~BD_AHDLC_TX_STATS;

	/* Set xbofs
	 */
	{
		struct irda_skb_cb *cb = (struct irda_skb_cb *)skb->cb;
		xbofs =
		    (cb->magic != LAP_MAGIC) ? 10 : cb->xbofs + cb->xbofs_delay;
		xbofs = (xbofs > 163) ? 163 : xbofs;
		out_be16(&ahp->ahdlc_nof, xbofs);
	}

	/* Set buffer length and buffer pointer.
	 */
	bdp->cbd_datlen = skb->len;
	bdp->cbd_bufaddr = __pa(skb->data);

	/* Push the data cache so the CPM does not get stale memory data.
	 */
	flush_dcache_range((unsigned long)(skb->data),
			   (unsigned long)(skb->data + skb->len));

	/* Save skb pointer.
	 */
	si->tx_skbuff[si->skb_cur] = skb;

	si->stats.tx_bytes += skb->len;
	si->skb_cur = (si->skb_cur + 1) & TX_RING_MOD_MASK;

	/* If we have a mean turn-around time, impose the specified
	 * specified delay.  We could shorten this by timing from
	 * the point we received the packet.
	 */
	if (mtt > 0)
		udelay(mtt);

	/* Send it on its way.  Tell CPM its ready, interrupt when done,
	 * its the last BD of the frame, and to put the CRC on the end.
	 */
	bdp->cbd_sc |=
	    (BD_AHDLC_TX_READY | BD_AHDLC_TX_INTR | BD_AHDLC_TX_LAST);

	dev->trans_start = jiffies;

	/* If this was the last BD in the ring, start at the beginning again.
	 */
	if (bdp->cbd_sc & BD_AHDLC_TX_WRAP)
		bdp = si->tx_bd_base;
	else
		bdp++;

	spin_unlock_irqrestore(&si->lock, flags);

	if (!--si->tx_free)
		netif_stop_queue(dev);

	si->cur_tx = (cbd_t *) bdp;

	return 0;
}

static int
mpc8xx_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct mpc8xx_irda *si = dev->priv;
	int ret = -EOPNOTSUPP;

	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN)) {
			/* We are unable to set the speed if the
			 * device is not running.
			 */
			if (si->open)
				mpc8xx_irda_set_speed(dev, rq->ifr_baudrate);
			else
				printk(KERN_INFO
				       "mpc8xx_irda_ioctl: SIOCSBANDWIDTH: !netif_running\n");
			ret = 0;
		}
		break;

	case SIOCSMEDIABUSY:
		ret = -EPERM;
		if (capable(CAP_NET_ADMIN)) {
			irda_device_set_media_busy(dev, TRUE);
			ret = 0;
		}
		break;

	case SIOCGRECEIVING:
		rq->ifr_receiving = 0;
		break;

	default:
		break;
	}

	return ret;
}

static struct net_device_stats *mpc8xx_irda_stats(struct net_device *ndev)
{
	struct mpc8xx_irda *si = ndev->priv;
	return &si->stats;
}

static int mpc8xx_irda_pd_setup(struct mpc8xx_irda *si)
{
	struct platform_device *pdev = to_platform_device(si->dev);
	struct resource *r;

	/* Fill out IRQ field */
	si->irq = platform_get_irq_byname(pdev, "interrupt");

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	si->sccp = (void *)r->start;

	if (si->sccp == NULL)
		return -EINVAL;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pram");
	si->ahp = (void *)r->start;

	if (si->ahp == NULL)
		return -EINVAL;

	return 0;
}

static int mpc8xx_irda_startup(struct net_device *dev)
{
	struct mpc8xx_irda *si = dev->priv;
	cbd_t *bdp;
	volatile cpm8xx_t *cp = cpmp;
	scc_t *sccp;
	volatile scc_ahdlc_t *ahp;
	dma_addr_t mem_addr;
	int i, j, k;
	unsigned char *ba;
	int err = -ENOMEM;

	spin_lock_init(&si->lock);

	if ((err = mpc8xx_irda_pd_setup(si)))
		return err;

	sccp = si->sccp;
	ahp = si->ahp;

	/* Disable receive and transmit.
	 */
	clrbits32(&sccp->scc_gsmrl, SCC_GSMRL_ENR | SCC_GSMRL_ENT);

	/* Allocate space for the buffer descriptors in the DP ram.
	 * These are relative offsets in the DP ram address space.
	 * Initialize base addresses for the buffer descriptors.
	 */
	i = cpm_dpalloc(sizeof(cbd_t) * RX_RING_SIZE, 8);
	out_be16(&ahp->scc_genscc.scc_rbase, i);
	si->rx_bd_base = (cbd_t *) & cp->cp_dpmem[i];
	si->cur_rx = si->rx_bd_base;

	i = cpm_dpalloc(sizeof(cbd_t) * TX_RING_SIZE, 8);
	out_be16(&ahp->scc_genscc.scc_tbase, i);
	si->tx_bd_base = (cbd_t *) & cp->cp_dpmem[i];
	si->dirty_tx = si->cur_tx = si->tx_bd_base;
	si->tx_free = TX_RING_SIZE;

	/* Issue init Tx and Rx BD command for SCC2.
	 */
	out_be16(&cp->cp_cpcr, mk_cr_cmd(CPM_CR_CH_SCC2, CPM_CR_INIT_TRX) | CPM_CR_FLG);
	while (in_be16(&cp->cp_cpcr) & CPM_CR_FLG) ;

	si->skb_cur = si->skb_dirty = 0;

	/* Initialize function code registers for big-endian.
	 */
	out_8(&ahp->scc_genscc.scc_rfcr, SCC_EB);
	out_8(&ahp->scc_genscc.scc_tfcr, SCC_EB);

	/* Set maximum bytes per receive buffer.
	 * This appears to be an Ethernet frame size, not the buffer
	 * fragment size.  It must be a multiple of four.
	 */
	out_be16(&ahp->scc_genscc.scc_mrblr, 14384);

	/* Set CRC preset and mask.
	 */
	out_be32(&ahp->ahdlc_cpres, 0x0000ffff);
	out_be32(&ahp->ahdlc_cmask, 0x0000f0b8);

	/* Clear zero register.
	 */
	out_be16(&ahp->ahdlc_zero, 0);

	/* Program RFTHR to the number of frames to be received
	 * before generating an interrupt.
	 */
	out_be16(&ahp->ahdlc_rfthr, 1);

	/* Program the control character tables, TXCTL_TBL and RXCTL_TBL.
	 */
	out_be32(&ahp->ahdlc_txctl_tbl, 0);	// initialized to zero for IrLAP.
	out_be32(&ahp->ahdlc_rxctl_tbl, 0);	// initialized to zero for IrLAP.

	out_be16(&ahp->ahdlc_bof, 0xC0);	//IRLAP_BOF; /* Begin. of Flag Char */
	out_be16(&ahp->ahdlc_eof, 0xC1);	//IRLAP_EOF; /* End of Flag Char */
	out_be16(&ahp->ahdlc_esc, 0x7D);	//IRLAP_ESC; /* Control Escape Char */
	out_be16(&ahp->ahdlc_nof, 12);

	/* Now allocate the host memory pages and initialize the
	 * buffer descriptors.
	 */
	bdp = si->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		/* Initialize the BD for every fragment in the page.
		 */
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap.
	 */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	bdp = si->rx_bd_base;
	k = 0;
	for (i = 0; i < CPM_IRDA_RX_PAGES; i++) {

		/* Allocate a page.
		 */
		ba = (unsigned char *)dma_alloc_coherent(NULL, PAGE_SIZE,
							 &mem_addr, GFP_KERNEL);

		/* Initialize the BD for every fragment in the page.
		 */
		for (j = 0; j < CPM_IRDA_RX_FRPPG; j++) {
			bdp->cbd_sc = BD_AHDLC_RX_EMPTY | BD_AHDLC_RX_INTR;
			bdp->cbd_bufaddr = mem_addr;
			si->rx_vaddr[k++] = ba;
			mem_addr += CPM_IRDA_RX_FRSIZE;
			ba += CPM_IRDA_RX_FRSIZE;
			bdp++;
		}
	}

	/* Set the last buffer to wrap.
	 */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	out_be16(&sccp->scc_scce, 0xffff);	/* Clear any pending events */
	out_be16(&sccp->scc_sccm, SCC_AHDLC_TXE | SCC_AHDLC_RXF | SCC_AHDLC_TXB);

	err = request_irq(si->irq, mpc8xx_irda_irq, 0, dev->name, dev);
	if (err) {
		kfree(si);
		return err;
	}

	/* Set GSMR_H to enable all normal operating modes.
	 * Set GSMR_L to enable Ethernet to MC68160.
	 */
	out_be32(&sccp->scc_gsmrh, SCC_GSMRH_RFW | SCC_GSMRH_IRP);
	out_be32(&sccp->scc_gsmrl, SCC_GSMRL_SIR | SCC_GSMRL_MODE_AHDLC | SCC_GSMRL_TDCR_16 |
				   SCC_GSMRL_TDCR_16 | SCC_GSMRL_RDCR_16);

	out_be16(&sccp->scc_dsr, 0x7e7e);

	/* Set processing mode.
	 */
	out_be16(&sccp->scc_psmr, SCC_PMSR_CHLN);

	/* Enable the transmit and receive processing.
	 */
	setbits32(&sccp->scc_gsmrl, SCC_GSMRL_ENR | SCC_GSMRL_ENT);

	netif_start_queue(dev);	//benjamin by irport

	return 0;
}

static int mpc8xx_irda_start(struct net_device *dev)
{
	struct mpc8xx_irda *si = dev->priv;
	int err = 0;

	if ((err = mpc8xx_irda_startup(dev)))
		return err;

	mpc8xx_irda_set_speed(dev, si->speed = 9600);

	/* Open new IrLAP layer instance, now that everything should be
	 * initialized properly
	 */
	si->irlap = irlap_open(dev, &si->qos, "MPC8xx SIR");
	err = -ENOMEM;
	if (!si->irlap) {
		si->open = 0;
		return err;
	}

	/* Now start the queue
	 */
	si->open = 1;
	netif_start_queue(dev);

	return 0;
}

static int mpc8xx_irda_stop(struct net_device *dev)
{
	struct mpc8xx_irda *si = dev->priv;

	disable_irq(dev->irq);

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}

	netif_stop_queue(dev);
	si->open = 0;

	/* Free resources
	 */
	free_irq(dev->irq, dev);

	return 0;
}

static void mpc8xx_irda_set_speed(struct net_device *dev, int speed)
{
	cpm_setbrg(2, speed);
}

static int mpc8xx_irda_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev;
	struct mpc8xx_irda *si;
	int err;

	if (!(ndev = alloc_irdadev(sizeof(struct mpc8xx_irda))))
		return -ENOMEM;

	si = ndev->priv;
	si->dev = dev;
	si->pdata = pdev->dev.platform_data;

	ndev->hard_start_xmit = mpc8xx_irda_hard_xmit;
	ndev->open = mpc8xx_irda_start;
	ndev->stop = mpc8xx_irda_stop;
	ndev->do_ioctl = mpc8xx_irda_ioctl;
	ndev->get_stats = mpc8xx_irda_stats;

	irda_init_max_qos_capabilies(&si->qos);

	/* We support original IRDA up to 115k2.
	 * Min Turn Time set to 1ms or greater.
	 */

	si->qos.baud_rate.bits =
	    IR_9600 | IR_19200 | IR_38400 | IR_57600 | IR_115200;
	si->qos.min_turn_time.bits = 7;
	irda_qos_bits_to_value(&si->qos);

	err = register_netdev(ndev);

	if (!err) {
		dev_set_drvdata(&pdev->dev, ndev);
		printk(KERN_INFO "IrDA: Registered device %s\n", ndev->name);
	} else
		free_netdev(ndev);

	return err;
}

static int mpc8xx_irda_remove(struct device *_dev)
{
	struct net_device *dev = dev_get_drvdata(_dev);

	if (dev) {
		rtnl_lock();
		unregister_netdevice(dev);
		free_netdev(dev);
		rtnl_unlock();
	}

	/*
	 * We now know that the netdevice is no longer in use, and all
	 * references to our driver have been removed.  The only structure
	 * which may still be present is the netdevice, which will get
	 * cleaned up by net/core/dev.c
	 */

	return 0;
}

static struct device_driver m8xxir_driver = {
	.name = "fsl-cpm-scc:irda",
	.bus = &platform_bus_type,
	.probe = mpc8xx_irda_probe,
	.remove = mpc8xx_irda_remove,
};

static int mpc8xx_irda_init(void)
{
	return driver_register(&m8xxir_driver);
}

static void __exit mpc8xx_irda_exit(void)
{
	driver_unregister(&m8xxir_driver);
}

module_init(mpc8xx_irda_init);
module_exit(mpc8xx_irda_exit);

MODULE_AUTHOR("source@mvista.com");
MODULE_DESCRIPTION("MPC8XX SIR");
MODULE_LICENSE("GPL");
