/*
 * drivers/net/nec_candy.c
 *
 * NEC candy ethernet driver.
 *
 * Author: Jun Sun <jsun@mvista.com>
 *
 * 2001-2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Changes:
 *  MontaVista Software Inc. <source@mvista.com>
 *  - Added MII interface library support
 *  - Added Linux ethtool package support (based on 8139cp.c driver)
 *  - Added network interface message level support
 *  - Fixed VLTP register initialisation (set to zero)
 *
 *  MontaVista Software Inc. <source@mvista.com>
 *  - Added support for VR4133 (CCR register defines additional bits).
 *
 *  MontaVista Software Inc. <source@mvista.com>
 *  - Added support for NEC VR7701.
 *
 *  MontaVista Software Inc.<asapkov@ru.mvista.com> or <source@mvista.com>
 *  - Added support for NEC CMB-VR4133.
 *
 *  MontaVista Software Inc. <source@mvista.com>
 *  - Ported to 2.6 kernel.
 *  - Added platform device support
 *  - Added NAPI support
 */

#define DRV_NAME		"nec_candy"
#define DRV_VERSION		"0.5"
#define DRV_RELDATE		"January 25, 2007"

/*
 * NEC Changes:
 *  Nov 30, 2006  NEC Informatec Systems,Ltd
 *  - Base version: 0.3 (release date: Nov 13, 2003)
 *  - Added support for NEC VR4133A
 *  - Added support for ring structure of Tx descriptor on VR4133A.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/param.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/nec_candy_pd.h>

#include <asm/addrspace.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>

#include "nec_candy.h"
#include "mv88e6063.h"
/***********************************************************************
 * debug
 ***********************************************************************/

#define CANDY_DEF_MSG_ENABLE	(NETIF_MSG_DRV		| \
				 NETIF_MSG_PROBE 	| \
				 NETIF_MSG_LINK)

static int debug = -1;
// #define	DEBUG_VERBOSE_NEC_CANDY
// #define	SHOW_BUG		/* targeted bug reporting msgs */

#ifdef DEBUG_VERBOSE_NEC_CANDY
#define DEBUG_VERBOSE(x)  do { x; } while (0)
#else
#define DEBUG_VERBOSE(x)
#endif

#if 0
#define MSG_LVL KERN_ERR
#define DBG_MSG_ON(condition,format,arg...)  				   \
	do { if (condition)    		     				   \
	printk(MSG_LVL "%s:%s: " format "\n",__FILE__,__FUNCTION__, ##arg);\
	} while (0)
#else
#define DBG_MSG_ON(condition,format,arg...)
#endif
/***********************************************************************
 * global data
 ***********************************************************************/

/* These identify the driver base version and may not be removed. */
static char version[] __devinitdata =
    DRV_NAME ": 10/100 Ethernet driver v" DRV_VERSION " (" DRV_RELDATE ")";

static struct candy_private *candy_priv_head = NULL;

/***********************************************************************
 * helpers
 ***********************************************************************/
#define	candy_in(x)	readl((void __iomem *) &x)
#define	candy_out(x, y)	writel((unsigned long)y, (void __iomem *)&x)

#define	candy_set_bits(x, mask)  candy_out(x, candy_in(x) | mask)
#define	candy_clear_bits(x, mask)  candy_out(x, candy_in(x) & ~mask)

#define	candy_set_macc1_bits(x, mask) \
		candy_out(x, (candy_in(x) | mask) & MACC1_RESERVED)
#define	candy_clear_macc1_bits(x, mask) \
		candy_out(x, (candy_in(x) & ~mask) & MACC1_RESERVED)

#define	candy_set_macc2_bits(x, mask) \
		candy_out(x, (candy_in(x) | mask) & MACC2_RESERVED)
#define	candy_clear_macc2_bits(x, mask) \
		candy_out(x, (candy_in(x) & ~mask) & MACC2_RESERVED)

#ifdef CONFIG_CANDY_NAPI
#define CANDY_NAPI_MASK_INTS	      ( RCVDN )
#define candy_unmask_napi_ints(p) \
		candy_out((p)->msr, (candy_in((p)->msr) | CANDY_NAPI_MASK_INTS))
#define candy_mask_napi_ints(p) \
		candy_out((p)->msr, (candy_in((p)->msr) & ~CANDY_NAPI_MASK_INTS))
#endif


/***********************************************************************
 * low-level hardware functions
 ***********************************************************************/

static void
mdio_write(struct net_device *dev, int phy_id, int location, int val)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;

	/* wait for busy */
	while (candy_in(p->mind) & BUSY) ;

	/* start the write */

	candy_out(p->madr,
		  ((phy_id << FIAD_SHIFT) & FIAD_MASK) |
		  (location & RGAD_MASK));
	candy_out(p->mwtd, (ulong) val);
}

static int
mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;

	/* wait for busy */
	while (candy_in(p->mind) & BUSY) ;

	/*  mac_reg->mcmd = 0; */
	candy_out(p->madr,
		  ((phy_id << FIAD_SHIFT) & FIAD_MASK) |
		  (location & RGAD_MASK));
	candy_out(p->mcmd, RSTAT);

	/* wait for busy */
	while (candy_in(p->mind) & BUSY) ;

	return (ushort) candy_in(p->mrdd);
}

static void candy_init_rings(struct net_device *dev);

static void
candy_set_media_speed(struct candy_private *pp)
{
	int lpa, advertise, media;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	ulong ccr;
	if PLATFORM(pdata->platform_id,NEC_VRBLADE_VR4133A_PID) {
		ccr = MII_PIN_SELECT;
		if (pdata->platform_id & WORKAROUND_E19_VR4133AR20_BIT)
			ccr |= SPD100;
	} else
		ccr = RMII_MODE | MII_PIN_SELECT;

	lpa = mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_LPA);
	advertise = mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_ADVERTISE);

	if (lpa == 0 || advertise == 0) {
		candy_out(pp->regs->ccr, ccr);
		pp->link_status.speed100 = 1;
		pp->link_status.fullduplex = 1;
		return;
	}

	/* Always set Marvell as 100MBps Full Duplex */
	if (pp->oui == MARVELL_OUI) {
		pp->link_status.speed100 = 1;
		pp->link_status.fullduplex = 1;
	} else	{
		media = mii_nway_result(lpa & advertise);
		switch (media) {
			case LPA_10FULL:
			case LPA_10HALF:
				pp->link_status.speed100 = 0;
				break;
		default:
				pp->link_status.speed100 = 1;
		}
		switch (media) {
			case LPA_10HALF:
			case LPA_100HALF:
				pp->link_status.fullduplex = 0;
				break;
		default:
				pp->link_status.fullduplex = 1;
		}
	}
	candy_out(pp->regs->ccr, ccr);
}

static int
candy_ethtool_ioctl(struct candy_private *pp, void *useraddr)
{
	u32 ethcmd;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	/* dev_ioctl() in ../../net/core/dev.c has already checked
	   capable(CAP_NET_ADMIN), so don't bother with that here.  */

	if (get_user(ethcmd, (u32 *) useraddr))
		return -EFAULT;

	switch (ethcmd) {

	case ETHTOOL_GDRVINFO:{
			struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
			strcpy(info.driver, DRV_NAME);
			strcpy(info.version, DRV_VERSION);
			strcpy(info.bus_info, "");
			info.regdump_len = CANDY_REGS_SIZE;
			info.n_stats = 0;
			if (copy_to_user(useraddr, &info, sizeof(info)))
				return -EFAULT;
			return 0;
		}
		/* get settings */
	case ETHTOOL_GSET:{
			struct ethtool_cmd ecmd = { ETHTOOL_GSET };
			spin_lock_irq(&pp->lock);
			mii_ethtool_gset(&pp->mii_if, &ecmd);
			spin_unlock_irq(&pp->lock);
			if (copy_to_user(useraddr, &ecmd, sizeof(ecmd)))
				return -EFAULT;
			return 0;
		}
		/* set settings */
	case ETHTOOL_SSET:{
			int r;
			struct ethtool_cmd ecmd;
			if (copy_from_user(&ecmd, useraddr, sizeof(ecmd)))
				return -EFAULT;
			spin_lock_irq(&pp->lock);
			r = mii_ethtool_sset(&pp->mii_if, &ecmd);
			spin_unlock_irq(&pp->lock);
			return r;
		}
		/* restart autonegotiation */
	case ETHTOOL_NWAY_RST:{
			return mii_nway_restart(&pp->mii_if);
		}
		/* get link status */
	case ETHTOOL_GLINK:{
			struct ethtool_value edata = { ETHTOOL_GLINK };
			edata.data = mii_link_ok(&pp->mii_if);
			if (copy_to_user(useraddr, &edata, sizeof(edata)))
				return -EFAULT;
			return 0;
		}

		/* get message-level */
	case ETHTOOL_GMSGLVL:{
			struct ethtool_value edata = { ETHTOOL_GMSGLVL };
			edata.data = pp->msg_enable;
			if (copy_to_user(useraddr, &edata, sizeof(edata)))
				return -EFAULT;
			return 0;
		}
		/* set message-level */
	case ETHTOOL_SMSGLVL:{
			struct ethtool_value edata;
			if (copy_from_user(&edata, useraddr, sizeof(edata)))
				return -EFAULT;
			pp->msg_enable = edata.data;
			return 0;
		}

		/* register's dump */
	case ETHTOOL_GREGS:{
			struct ethtool_regs regs;
			u8 *regbuf = kmalloc(CANDY_REGS_SIZE, GFP_KERNEL);
			int rc;

			if (!regbuf)
				return -ENOMEM;
			memset(regbuf, 0, CANDY_REGS_SIZE);

			rc = copy_from_user(&regs, useraddr, sizeof(regs));
			if (rc) {
				rc = -EFAULT;
				goto err_out_gregs;
			}

			if (regs.len > CANDY_REGS_SIZE)
				regs.len = CANDY_REGS_SIZE;
			if (regs.len < CANDY_REGS_SIZE) {
				rc = -EINVAL;
				goto err_out_gregs;
			}
			if ((PLATFORM(pdata->platform_id,NEC_CMB_VR4133_PID)) ||
			(PLATFORM(pdata->platform_id,NEC_VRBLADE_VR4133A_PID)))
			    regs.version = CANDY_REGS_VER2;
			else
			    regs.version = CANDY_REGS_VER1;
			rc = copy_to_user(useraddr, &regs, sizeof(regs));
			if (rc) {
				rc = -EFAULT;
				goto err_out_gregs;
			}

			useraddr += offsetof(struct ethtool_regs, data);

			spin_lock_irq(&pp->lock);
			memcpy_fromio(regbuf, pp->regs, CANDY_REGS_SIZE);
			spin_unlock_irq(&pp->lock);

			if (copy_to_user(useraddr, regbuf, regs.len))
				rc = -EFAULT;

		      err_out_gregs:
			kfree(regbuf);
			return rc;
		}

		/* get/set TX checksumming */
	case ETHTOOL_GTXCSUM:{
			struct ethtool_value edata = { ETHTOOL_GTXCSUM };

			edata.data = (pp->ndev->features &
				      NETIF_F_IP_CSUM) != 0;
			if (copy_to_user(useraddr, &edata, sizeof(edata)))
				return -EFAULT;
			return 0;
		}
	case ETHTOOL_STXCSUM:{
			struct ethtool_value edata;

			if (copy_from_user(&edata, useraddr, sizeof(edata)))
				return -EFAULT;

			if (edata.data)
				pp->ndev->features |= NETIF_F_IP_CSUM;
			else
				pp->ndev->features &= ~NETIF_F_IP_CSUM;

			return 0;
		}

		/* get/set scatter-gather */
	case ETHTOOL_GSG:{
			struct ethtool_value edata = { ETHTOOL_GSG };

			edata.data = (pp->ndev->features & NETIF_F_SG) != 0;
			if (copy_to_user(useraddr, &edata, sizeof(edata)))
				return -EFAULT;
			return 0;
		}
	case ETHTOOL_SSG:{
			struct ethtool_value edata;

			if (copy_from_user(&edata, useraddr, sizeof(edata)))
				return -EFAULT;

			if (edata.data)
				pp->ndev->features |= NETIF_F_SG;
			else
				pp->ndev->features &= ~NETIF_F_SG;

			return 0;
		}

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int
candy_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct candy_private *pp = dev->priv;
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *) &rq->ifr_data;
	int rc;
	struct vlan_ioctl_data *vdata = (struct vlan_ioctl_data *)&rq->ifr_data;

	if (!netif_running(dev))
		return -EINVAL;

	if (cmd == SIOCETHTOOL)
		return candy_ethtool_ioctl(pp, (void *) rq->ifr_data);

	spin_lock_irq(&pp->lock);

	if ((cmd > SIOCDEVPRIVATE + 10) && (cmd < SIOCDEVPRIVATE + 14)) {
		if (vlan_mii_ioctl)
			rc = vlan_mii_ioctl(&pp->mii_if, mii, vdata, cmd);
		else {
			printk(KERN_ERR "vlan_mii_ioctl not defined\n");
			rc =  -EINVAL;
		}
	} else
		rc = generic_mii_ioctl(&pp->mii_if, mii, cmd, NULL);

	spin_unlock_irq(&pp->lock);
	return rc;
}

static void
set_mac_addr(struct net_device *dev, u_char * addr)
{
	candy_regs *p = ((struct candy_private *) dev->priv)->regs;

	candy_out(p->lsa2, (addr[0] << 8 | addr[1]) & LSA2_MASK);
	candy_out(p->lsa1,
		  addr[2] << 24 | addr[3] << 16 | addr[4] << 8 | addr[5]);
}

/*
 * This is called when system boots up and/or after ether chip is reset.
 */
static void
candy_hw_init(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	candy_regs *p = pp->regs;
	unsigned int temp;
	unsigned tx_flth;

	/*
	 * software reset
	 */
	candy_out(p->ccr, SRT);
	candy_in(p->isr);

	/*
	 * MII software reset
	 */
	candy_out(p->miic, MISRT|CLKS33);
	candy_out(p->miic, CLKS33);

	candy_out(p->macc2, APD);
	candy_out(p->ipgt, IPGT);
	candy_out(p->ipgr, IPGR1 | IPGR2);
	candy_out(p->clrt, LCOLW | RETRY);
	candy_out(p->lmax, MAXF);
	candy_out(p->vltp, 0x0);
	candy_out(p->miic, CLKS66);

	/*
	 * initialize DMA / FIFO
	 */
	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E7_AFCE_BIT))
		candy_out(p->txcfg, DTBS8);
	else
		candy_out(p->txcfg, DTBS8 | AFCE);
	if (PLATFORM(pdata->platform_id,NEC_CMB_VR4133_PID))
		candy_out(p->txcfg, DTBS8); /* DTBS is set to 32 bytes for VR4133. */

	/* DTBS is set to 64 bytes for VR4133A. */
	if (PLATFORM(pdata->platform_id,NEC_VRBLADE_VR4133A_PID)) {
	        if ( candy_in(p->erev) == 0x20000) {
			printk("VR4133A: ether block revision 2.0\n");
			pdata->platform_id |= WORKAROUND_E19_VR4133AR20_BIT | WORKAROUND_E20_VR4133AR20_BIT;
		}
		candy_out(p->txcfg, DTBS16);
		tx_flth = TX_FLTH_VR4133A;
	} else
	if (PLATFORM(pdata->platform_id,NEC_CMB_VR4133_PID))
		tx_flth = TX_FLTH_VR4133;
	else
		tx_flth = TX_FLTH;
	candy_out(p->txfc, TPTV | TX_DRTH | tx_flth);
	candy_out(p->rxcfg, DRBS16);
	candy_out(p->rxfc, UWM | LWM | RX_DRTH16W);
	candy_out(p->rxpd, AL);

        /* Set Transmit Ring Buffer Mode */
        if (pdata->platform_options.use_tx_ring_buffer)
		candy_out(p->mode, 0x00000501); /* TABT=0,TFMODE=0, TDMODE=1, TDP=0101 */

       /* Set pins for MII use */
       candy_set_media_speed(pp);

       /*
	* initialize MAC
	*/
	temp = TXFC | RXFC | CRCEN | PADEN;
	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E13_TXFC_BIT))
		temp &= ~TXFC;
	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E21_PAD_BIT))
		temp &= ~PADEN;
	candy_out(p->macc1, temp);

	netif_carrier_off(dev);
	mii_check_media(&pp->mii_if, netif_msg_link(pp), 1);

#define DAVICOM_OUI 0x00606E /*Davicom Semiconductor MII OUI*/
	pp->oui = mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_PHYSID1) <<6;
	pp->oui |= mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_PHYSID2)>>10;
	if (pp->oui == DAVICOM_OUI) {
		if (pp->mii_if.full_duplex)
			candy_set_macc1_bits(pp->regs->macc1, FULLD);
		else
			candy_clear_macc1_bits(pp->regs->macc1, FULLD);
	}
	else {
		/* ETH1 (Marvell switch) is always set to full duplex! */
		candy_set_macc1_bits(pp->regs->macc1, FULLD);
	}
	netif_carrier_on(dev);

	/* MCRST TFRST RFRST software reset */
	candy_set_macc2_bits(pp->regs->macc2, (MCRST | TFRST | RFRST));
	udelay(3);
	candy_clear_macc2_bits(pp->regs->macc2, (MCRST | TFRST | RFRST));

	set_mac_addr(dev, dev->dev_addr);

	candy_out(p->ht1, 0);
	candy_out(p->ht2, 0);

	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E10_PRM_AMC_BIT))
		candy_out(p->afr, ABC);
	else
		candy_out(p->afr, PRM | ABC | AMC);

	/*
	 * disable all interrupts until dev is opened later
	 */
	candy_out(p->msr, 0);
       /*
	* PHY software reset
	*/
       /*
	* enable Davicom PHY device.
	*/
	if (pp->oui == DAVICOM_OUI) {
	       unsigned long start, end;
	       printk(KERN_INFO "%s: Configuring Davicom PHY tranceiver\n", dev->name);
	       mdio_write(pp->mii_if.dev, pp->mii_if.phy_id, MII_BMCR, BMCR_RESET);
	       start = jiffies;
		end = start + (HZ / 2);
		while(time_before(jiffies, end)){
			if( (mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_BMCR) & BMCR_RESET) == 0)
				break;
		}
		mdio_write(pp->mii_if.dev, pp->mii_if.phy_id, MII_BMCR, BMCR_FULLDPLX|BMCR_ANRESTART|BMCR_ANENABLE|BMCR_SPEED100 );
	}

	/*
	 * enable Marvell Switch device.
	 */
	if (pp->oui == MARVELL_OUI) {
		if (!enableMarvellSwitch)
			printk(KERN_ERR "enableMarvellSwitch not defined\n");
		else {
			printk(KERN_INFO "%s: Configuring Marvell Switch\n", dev->name);
			enableMarvellSwitch(&pp->mii_if);
		}
	}
}

static void
candy_down(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;

	DEBUG_VERBOSE(printk("candy_down() invoked.\n"));

	candy_clear_macc1_bits(p->macc1, SRXEN);

	candy_clear_bits(p->txcfg, TXE);
	candy_clear_bits(p->rxcfg, RXE);

	/*
	 * disable all interrupts until dev is opened later
	 */
	candy_out(p->msr, 0);
}

static void
candy_up(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	candy_regs *p = pp->regs;

	DEBUG_VERBOSE(printk("candy_up() invoked.\n"));

	/* candy must be down right now */
	DBG_MSG_ON(((candy_in(p->txcfg) & TXE) != 0),"TX DMA should not be enabled!");
	DBG_MSG_ON(((candy_in(p->rxcfg) & RXE) != 0),"RX DMA should not be enabled!");

	/* set transmit descriptor pointer register */
        if (pdata->platform_options.use_tx_ring_buffer)
		candy_out(p->txdp, CPHYSADDR(&pp->tx_ring[pp->tx_head]));

	/* enable transmit */
	candy_set_bits(p->txcfg, TXE);

	/* set number of descriptors */
	candy_out(p->rxpd, AL | (RX_RING_SIZE & RNOD_MASK));

	/* set the receive descriptor pointer */
	candy_out(p->rxdp, CPHYSADDR(&pp->rx_ring[pp->rx_head]));

	/* enable receive */
	candy_out(p->rxcfg, RXE | DRBS4);

	candy_set_macc1_bits(p->macc1, SRXEN);

	/* turn on interrupts */
	candy_out(p->msr, 0xffffffff & ISR_RESERVED);
	candy_in(p->isr);
}

static unsigned int
hashit(char *addr)
{
	int i;
	int j;
	unsigned char nibblehigh;
	unsigned char nibblelow;
	unsigned long da_crc = 0L;
	unsigned long hash_p = 0L;
	unsigned long hash_byte = 0L;
	unsigned long hash_bit = 0L;
	unsigned long crc32 = 0xffffffffL;
	unsigned long crcpoly2 = 0xedb88320L;

	for (i = 0; i < 6; i++) {
		nibblelow = addr[i] & 0x0f;
		nibblehigh = addr[i] >> 4;
		crc32 ^= nibblelow;
		for (j = 0; j < 4; j++) {
			if (crc32 & 1) {
				crc32 = (crc32 >> 1) ^ crcpoly2;
			} else {
				crc32 >>= 1;
			}
		}
		crc32 ^= nibblehigh;
		for (j = 0; j < 4; j++) {
			if (crc32 & 1) {
				crc32 = (crc32 >> 1) ^ crcpoly2;
			} else {
				crc32 >>= 1;
			}
		}
	}
	da_crc = crc32 & 0x000001f8L;
	for (j = 31; j >= 23; j--) {
		hash_p = hash_p * 2 + da_crc % 2;
		da_crc = da_crc / 2;
	}
	hash_byte = (hash_p & 0x00000038) >> 3;
	hash_bit = (hash_p & 0x00000007);
	return ((hash_byte << 16) | (hash_bit & 0xFFFF));
}

static void
candy_set_filter(struct net_device *dev, int on)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;
	struct dev_mc_list *mclist = dev->mc_list;
	unsigned long ht1 = 0;
	unsigned long ht2 = 0;
	int i;
	union {
		unsigned long fusion;
		unsigned short byte;
		unsigned short bit;
	} val;

	DEBUG_VERBOSE(printk
		      ("candy_set_filter() invoked - %s\n", on ? "on" : "off"));

	for (i = 0, mclist = dev->mc_list;
	     mclist && i < dev->mc_count; i++, mclist = mclist->next) {
		val.fusion = hashit(mclist->dmi_addr);
		if (val.byte > 3)
			ht1 |= (1 << ((val.byte - 4) * 8 + val.bit));
		else
			ht2 |= (1 << (val.byte * 8 + val.bit));
	}

	/* lock ints */
	candy_out(p->ht1, ht1);
	candy_out(p->ht2, ht2);
}

/*
 * Apparently candy tx can stall due to various reasons.
 * This routine will attempt to recover from tx stall.
 */
static void
candy_error_recover(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	candy_regs *p = pp->regs;

	spin_lock(&pp->lock);

	netif_stop_queue(dev);

	candy_down(dev);

	/* free tx skb */
        if (pdata->platform_options.use_tx_ring_buffer)	{
		while (pp->tx_tail != pp->tx_head) {
			if (pp->tx_skb[pp->tx_head]) {
				dev_kfree_skb_irq(pp->tx_skb[pp->tx_head]);
				pp->tx_skb[pp->tx_head] = NULL;
			}
			pp->tx_count--;
			DBG_MSG_ON((pp->tx_count < 0), " tx_count=%d",pp->tx_count);
			if (++pp->tx_head == TX_RING_SIZE)
				pp->tx_head = 0;
		}
		candy_init_rings(dev);
	}
	candy_hw_init(dev);
	candy_up(dev);

	netif_wake_queue(dev);

	/* restart transmitting */
        if (!pdata->platform_options.use_tx_ring_buffer) {
	pp->tx_stop = pp->tx_tail;
	candy_out(p->txdp, CPHYSADDR(&pp->tx_ring[pp->tx_head]));
	}

	spin_unlock(&pp->lock);
}

/*
 * This implements the workaround described for E-8
 */
static void
tx_stall_recover(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;

	/* E-8 bug only happens when receiving is on */
	if ((candy_in(p->macc1) & SRXEN) && (candy_in(p->rxcfg) & RXE)) {

		candy_clear_macc1_bits(p->macc1, SRXEN);
		candy_clear_bits(p->rxcfg, RXE);

		udelay(20);

		candy_out(p->txdp, CPHYSADDR(&pp->tx_ring[pp->tx_head]));

		candy_set_bits(p->rxcfg, RXE);
		candy_set_macc1_bits(p->macc1, SRXEN);

	} else {
		candy_out(p->txdp, CPHYSADDR(&pp->tx_ring[pp->tx_head]));
	}
}

/***********************************************************************
 * hardware-independent helper routine
 ***********************************************************************/
static void
candy_init_rings(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	int i;

	DEBUG_VERBOSE(printk("candy_init_rings() invoked.\n"));

	/* tx rings */
	for (i = 0; i < TX_RING_SIZE; i++) {
		pp->tx_ring[i].size = 0;
		pp->tx_ring[i].pointer = 0;
    		if (pdata->platform_options.use_tx_ring_buffer)
			pp->tx_ring[i].status = OWN;
		else
			pp->tx_ring[i].status = 0;
	}
	/* this may be not necessary, if we reset every txdpr in intr */
	pp->tx_ring[TX_RING_SIZE].status = 0;
	pp->tx_ring[TX_RING_SIZE].size = 0x0;
	pp->tx_ring[TX_RING_SIZE].pointer = CPHYSADDR(pp->tx_ring);

	pp->tx_tail = pp->tx_stop = pp->tx_head = 0;
	pp->tx_count = 0;

	/* rx rings */
	for (i = 0; i < RX_RING_SIZE; i++) {
		pp->rx_ring[i].status = DB_LP;
		pp->rx_ring[i].size = RX_BUF_SIZE;
		pp->rx_ring[i].pointer = CPHYSADDR(pp->rx_skb[i]->data);
	}
	pp->rx_ring[RX_RING_SIZE].status = 0;	/* link back to the beginning */
	pp->rx_ring[RX_RING_SIZE].size = 0xffff;
	pp->rx_ring[RX_RING_SIZE].pointer = CPHYSADDR(pp->rx_ring);

	pp->rx_head = 0;
	pp->rx_disable = 0;
}

static void
candy_check_intr(ulong isr, struct nec_candy_platform_data * pdata)
{
	static const char *intr_name[32] = {
		"carry flag",
		"reserved",
		"reserved",
		"link failed",
		"overflow",
		"receive buffer desc request at zero",
		"receive buffer desc request at alert level",
		"receive done",

		"reserved",
		"reserved",
		"control frame transmit",
		"transmit aborted",
		"underrun",
		"transmit frame length exceed",
		"",
		"transmit done",

		"reserved",
		"reserved",
		"reserved",
		"reserved",	/* 4 */
		"reserved",
		"reserved",
		"reserved",
		"reserved",

		"reserved",
		"reserved",
		"reserved",
		"reserved",	/* 4 */
		"reserved",
		"reserved",
		"reserved",
		"IBUS error"
	};
	ulong i, j;
	if (pdata->platform_options.use_tx_ring_buffer)
	    intr_name[14] = "transmit buffer descriptor request at status OWN set";
	else
	    intr_name[14] = "transmit buffer descriptor request at NULL";
	for (i = 0, j = 1; i < 32; j <<= 1, i++) {
		if (j & isr)
			printk("\t%s\n", intr_name[i]);
	}
}

#ifdef CONFIG_CANDY_NAPI
static void
handle_rx_error(struct net_device *dev, struct candy_desc *dp, ulong isr)
{
	struct candy_private *pp = (struct candy_private *)dev->priv;
	struct sk_buff *newskb;

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	dev_kfree_skb(pp->rx_skb[pp->rx_head]);

	newskb = dev_alloc_skb(RX_BUF_SIZE);    /* allocate a new skb */
	if (newskb == NULL) {
		printk(KERN_ERR "%s: memory squeeze.\n", dev->name);
			return;
	}
	newskb->dev = dev;
	skb_reserve(newskb, 2); /* to align IP/UDP_TCP header to 4-bytes */
	pp->rx_skb[pp->rx_head] = newskb;
	dp->status = DB_LP;     /* 1 stands for buffer vs link ptr */
	dp->size = RX_BUF_SIZE;
	dp->pointer = CPHYSADDR(newskb->data);
	dma_cache_wback_inv(KSEG0ADDR(dp->pointer), RX_BUF_SIZE);

	/* log some errors that hardware doesn't log */
	pp->stats.rx_errors++;

	if (isr & OF)
		pp->stats.rx_fifo_errors++;
	if (isr & RBDRU)
		pp->stats.rx_missed_errors++;
	if (dp->status & OVRN)
		pp->stats.rx_over_errors++;
}

static int candy_napi_rx(struct net_device *dev, int budget)
{
	struct candy_private *pp = (struct candy_private *)dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	candy_regs *p = pp->regs;
	struct candy_desc *dp = &pp->rx_ring[pp->rx_head];
	struct sk_buff *rxskb, *newskb;
	int pkt_len;
	int received = 0;
	int rnod = 0;

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	while (!((dp->status & OWN) == 0) && (received < budget)) {
		if ((dp->status & RXOK) == 0) { /* handle the error case */
			handle_rx_error(dev, dp, 0);
			if (++pp->rx_head == RX_RING_SIZE)
				pp->rx_head = 0;
			dp = &pp->rx_ring[pp->rx_head];
				received++;
			rnod++;
			continue;
		}

		/* oversize? */
		if (dp->size > ETH_FRAME_SIZE) {
			if (netif_msg_rx_status(pp))
				printk(KERN_DEBUG "%s: rx, oversized pkt\n",
				       dev->name);

			handle_rx_error(dev, dp, 0);
			if (++pp->rx_head == RX_RING_SIZE)
				pp->rx_head = 0;
			dp = &pp->rx_ring[pp->rx_head];
			received++;
			rnod++;
			continue;
		} else {
			pkt_len = dp->size;
		}

		/*
		 * we got a good packet, use the new skb to replace the received
		 * one
		 */
		rxskb = pp->rx_skb[pp->rx_head];

		DBG_MSG_ON((rxskb->data != phys_to_virt(dp->pointer)),"wrong rxskb");

		newskb = dev_alloc_skb(RX_BUF_SIZE);    /* allocate a new skb */
		if (newskb == NULL) {
			printk(KERN_ERR "%s: memory squeeze.\n", dev->name);
			break;
		}

		newskb->dev = dev;
		skb_reserve(newskb, 2); /* to align IP/UDP_TCP header to
					   4-bytes */
		pp->rx_skb[pp->rx_head] = newskb;
		dp->status = DB_LP;     /* 1 stands for buffer vs link ptr */
		dp->size = RX_BUF_SIZE;
		dp->pointer = CPHYSADDR(newskb->data);
		dma_cache_wback_inv(KSEG0ADDR(dp->pointer), RX_BUF_SIZE);
		rnod++;

		if (++(pp->rx_head) == RX_RING_SIZE)
			pp->rx_head = 0;

		skb_put(rxskb, pkt_len);
		rxskb->protocol = eth_type_trans(rxskb, dev);
		rxskb->ip_summed = CHECKSUM_UNNECESSARY;
		netif_receive_skb(rxskb);
		received++;
		dev->last_rx = jiffies;

		dp = &pp->rx_ring[pp->rx_head];
	}

	if (netif_msg_rx_status(pp))
		printk(KERN_DEBUG "%s: rx, processed %d frames.\n",
		       dev->name, received);

	candy_out(p->rxpd, AL | rnod);

	if (pp->rx_disable == 1) {
		if (WORKAROUND(pdata->platform_id,WORKAROUND_E20_VR4133AR20)) {
			ulong current_rxpd = 0;
			candy_out(p->rxdp, candy_in(p->rxdp));
			current_rxpd = RNOD_MASK & candy_in(p->rxpd);
			if ( current_rxpd > 4 ) {
				udelay(23);
				candy_set_macc1_bits(p->macc1, SRXEN);
				pp->rx_disable = 0;
			}
		} else {
			candy_out(p->rxdp, candy_in(p->rxdp));
			pp->rx_disable = 0;
		}
	}

	return received;
}

static int candy_napi_poll(struct net_device *dev, int *budget)
{
	struct candy_private *pp = (struct candy_private *)dev->priv;
	int not_done;

	spin_lock(&pp->rxlock);

	not_done = 0;

	if (pp->prev_rpkt != candy_in((pp->regs)->rpkt)) {
		int orig_budget = *budget;
		int received = 0;

		if (orig_budget > dev->quota)
			orig_budget = dev->quota;

		received = candy_napi_rx(dev, orig_budget);

		pp->prev_rpkt += received;
		*budget -= received;
		dev->quota -= received;

		if (received >= orig_budget)
			not_done = 1;
	}


	if (!not_done) {
		netif_rx_complete(dev);
		candy_unmask_napi_ints(pp->regs);
	}

	spin_unlock(&pp->rxlock);

	return not_done;
}

#else
static void
reclaim_one_rx_desc(struct net_device *dev, char *buf)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;
	struct candy_desc *dp = &pp->rx_ring[pp->rx_head];

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	if (buf != NULL) {
		dp->pointer = CPHYSADDR(buf);
	}
	dp->status = DB_LP;	/* 1 stands for buffer vs link ptr */
	dp->size = RX_BUF_SIZE;

	/* we need to clean up cache here.  Otherwise we may have some
	 * dirty cache while ether controller is feeding fresh pkt data
	 * to the physical RAM.  Data corruption could happen.
	 */
	dma_cache_wback_inv(KSEG0ADDR(dp->pointer), RX_BUF_SIZE);

	if (++pp->rx_head == RX_RING_SIZE)
		pp->rx_head = 0;
	DBG_MSG_ON((pp->rx_head >= RX_RING_SIZE),"rx head (%u) exceeds \
	RX_RING_SIZE (%u)", pp->rx_head,RX_RING_SIZE);

	/* tell hardware we have one descriptor to work with */
	candy_out(p->rxpd, AL | 1);

	/* check rx_disable */
	if (pp->rx_disable) {
		pp->rx_disable = 0;
		candy_out(p->rxdp, CPHYSADDR(dp));
		candy_set_macc1_bits(p->macc1, SRXEN);
		DEBUG(printk(KERN_WARNING "%s : re-enable SRXEN\n", dev->name));
	}
}

static void
handle_rx_error(struct net_device *dev, struct candy_desc *dp, ulong isr)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	if (netif_msg_rx_err(pp))
		printk(KERN_DEBUG "%s: rx_err, rx_ring[%d] error,"
		       "status =%04x, size=%d, isr = 0x%08lx.\n",
		       dev->name, pp->rx_head, dp->status, dp->size, isr);

	/* log some errors that hardware doesn't log */
	pp->stats.rx_errors++;
	if (isr & OF)
		pp->stats.rx_fifo_errors++;
	if (isr & RBDRU)
		pp->stats.rx_missed_errors++;
	if (dp->status & OVRN)
		pp->stats.rx_over_errors++;
}

static void
candy_rx(struct net_device *dev, ulong isr)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;
	struct candy_desc *dp = &pp->rx_ring[pp->rx_head];
	int pkt_len;
	struct sk_buff *newskb;
	struct sk_buff *rxskb;
	int i;
	int skb_size;

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	if (netif_msg_rx_status(pp)) {
		printk(KERN_DEBUG "%s: rx, stat=0x%08lx\n",
		       dev->name, (ulong) candy_in(p->rxst));
		printk(KERN_DEBUG "\trx_head = %d, {0x%04x, %u, 0x%08lx}\n",
		       pp->rx_head, dp->status, dp->size, dp->pointer);
	}

	/* check if we still have any empty receive descriptor */
	if (isr & RBDRU) {
		printk(KERN_ERR
		       "%s : no more receive buffers.  Stop receiving.",
		       dev->name);
		candy_clear_bits(p->macc1, SRXEN);
		pp->rx_disable = 1;
	}

	/* FIXME : we are fetching packets.  How do we know where the
	 * end is?  When OWN bit is 0 (in previous linux driver)?
	 *
	 * Checking OWN bit alone is not reliable especially when kgdb
	 * is used.  Hack to work around.  We really need to understand
	 * what entries we can consume here.
	 */
	for (i = 0;; i++) {
		dp = &pp->rx_ring[pp->rx_head];

		if (((dp->status & OWN) == 0) &&
				((pp->rx_disable == 0) || (i != 0))) {
			/* no frame received in this descriptor yet */
			break;
		}

		/* handle the error case */
		if ((dp->status & RXOK) == 0) {
			handle_rx_error(dev, dp, isr);
			reclaim_one_rx_desc(dev, NULL);
			continue;
		}

		/* oversize? */
		if (dp->size > ETH_FRAME_LEN) {
			if (netif_msg_rx_status(pp))
				printk(KERN_DEBUG "%s: rx, oversized pkt\n",
				       dev->name);
			pkt_len = ETH_FRAME_LEN;
		} else {
			pkt_len = dp->size;
		}

		/* we got a good packet */

		/* STRATEGY: ether packet has 14 bytes.  So we will
		 * suffer from emulated unaligned access if we pass
		 * the skb straight to upper layer.  An alternative is
		 * to copy the buffer by offset of 2 and then pass it up.
		 * Then the overhead is copying.
		 *
		 * In general, it is more beneficial to copy if we have smaller
		 * packet.  Also, it is more beneficial to copy if we have
		 * faster machines.
		 *
		 * To keep it flexible, we will leave rx_copybreak flexible.
		 */

		if (pkt_len < rx_copybreak) {
			skb_size = pkt_len + 2;
		} else {
			skb_size = RX_BUF_SIZE;
		}

		/* allocate a new skb */
		newskb = dev_alloc_skb(skb_size);
		if (newskb == NULL) {
			printk(KERN_ERR
			       "%s: Memory squeeze, dropping packet.\n",
			       dev->name);
			reclaim_one_rx_desc(dev, NULL);
			pp->stats.rx_dropped++;
			continue;
		}

		if (pkt_len <= rx_copybreak) {
			/* we will copy */
			rxskb = pp->rx_skb[pp->rx_head];
			DBG_MSG_ON((rxskb->data != (void *)KSEG0ADDR(dp->pointer)),\
			"ERROR: rxskb->data != (void *)KSEG0ADDR(dp->pointer)");

			newskb->dev = dev;
			skb_reserve(newskb, 2);	/* align IP pkt to 16-byte */
			eth_copy_and_sum(newskb, rxskb->data, pkt_len, 0);

			rxskb = newskb;
			reclaim_one_rx_desc(dev, NULL);
		} else {
			/* use the new skb to replace the recived one */
			rxskb = pp->rx_skb[pp->rx_head];

			newskb->dev = dev;
			pp->rx_skb[pp->rx_head] = newskb;
			reclaim_one_rx_desc(dev, newskb->data);

		}

		skb_put(rxskb, pkt_len);
		rxskb->protocol = eth_type_trans(rxskb, dev);
		rxskb->ip_summed = CHECKSUM_UNNECESSARY;
		netif_rx(rxskb);

		dev->last_rx = jiffies;

	}

	/*
	 * when we are out here, should rxdp be the same as
	 * &pp->rx_ring[pp->head]?
	 */
	if (netif_msg_rx_status(pp))
		printk(KERN_DEBUG "%s: rx, processed %d frames.\n",
		       dev->name, i);
#ifdef SHOW_BUG
	if ((candy_in(p->rxdp) != CPHYSADDR(&pp->rx_ring[pp->rx_head])) &&
	    (candy_in(p->rxdp) != CPHYSADDR(&pp->rx_ring[RX_RING_SIZE]))) {
		int i;
		printk
		    ("%s : unexpected out of rx - rx_ring[rx_head] = (%04x, %d)\n",
		     dev->name, dp->status, dp->size);
		for (i = 0; i < RX_RING_SIZE + 1; i++) {
			if (p->rxdp == CPHYSADDR(&pp->rx_ring[i]))
				break;
		}
		DBG_MSG_ON((i == RX_RING_SIZE + 1),"cannot find current rx tail");
		printk("\trx_head = %d, rx_tail = %d\n", pp->rx_head, i);
	}
#endif
	return;
}
#endif

static void
append_one_tx_desc(struct net_device *dev,
		   ushort status,
		   ushort size, ulong pointer, struct sk_buff *skb)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct candy_desc *dp = &pp->tx_ring[pp->tx_tail];
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;

	dp->size = size;
	dp->pointer = pointer;

	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E10_VR4133_BIT)) {
		int i, shift = pointer % 4;
		if (shift && (size == 30 || size == 31)) {
			if (skb_tailroom(skb) >= 4 - shift) {
				shift = 4 - shift;
				skb->data += shift;
				dp->pointer += shift;
				for (i = size + shift - 1; i >= 0; i--)
					skb->data[i] = skb->data[i - shift];
				dma_cache_wback_inv((ulong) (skb->data), skb->len);
			}
			else if (skb_headroom(skb) >=  shift) {
				skb->data -= shift;
				dp->pointer -= shift;
				for (i = 0; i < size; i++)
					skb->data[i] = skb->data[i + shift];
				dma_cache_wback_inv((ulong) (skb->data), skb->len);
			}
			else {
				printk(KERN_ERR "nec_candy.c: Unable to do a workaround for hardware bug (Restriction 10 for VR4133).\n");
			}
		}
	}
	dp->status = status;
	pp->tx_skb[pp->tx_tail] = skb;

	pp->tx_count++;
	if (++pp->tx_tail == TX_RING_SIZE)
		pp->tx_tail = 0;
}

static void
reclaim_one_tx_desc(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct candy_desc *dp = &pp->tx_ring[pp->tx_head];
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;

	dp->size = 0;
	dp->pointer = 0;

	if (pdata->platform_options.use_tx_ring_buffer)
		dp->status = OWN;
	else
		dp->status = 0;

	/* free skb */
	if (pp->tx_skb[pp->tx_head]) {
		dev_kfree_skb_irq(pp->tx_skb[pp->tx_head]);
		pp->tx_skb[pp->tx_head] = NULL;
	}

	pp->tx_count--;
	if (++pp->tx_head == TX_RING_SIZE)
		pp->tx_head = 0;
}

static void
restart_tx_hw(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;
	struct candy_desc *dp = &pp->tx_ring[pp->tx_tail];
	/* the desc should already be set NULL */
	DBG_MSG_ON((dp->status != 0),"ERROR: candy descriptor status !=0");
	DBG_MSG_ON((dp->size != 0),"ERROR: candy descriptor size !=0");
	DBG_MSG_ON((dp->pointer != 0),"ERROR: candy descriptor pointer !=0");
	/* we need at least one desc for null descriptor */
	DBG_MSG_ON((pp->tx_count >= TX_RING_SIZE),"tx_count (%d) exceeds TX_RING_SIZE (%u)",\
	pp->tx_count,TX_RING_SIZE);

	append_one_tx_desc(dev, 0, 0, 0, NULL);

	/* start */
	DBG_MSG_ON((pp->tx_head != pp->tx_stop),"ERROR:tx_head != tx_stop");
	pp->tx_stop = pp->tx_tail;
	candy_out(p->txdp, CPHYSADDR(&pp->tx_ring[pp->tx_head]));
}

static void
handle_tx_error(struct net_device *dev, struct candy_desc *dp, ulong isr)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	if (netif_msg_tx_err(pp))
		printk(KERN_DEBUG "%s: tx err, tx_ring[%d] error, "
		       "status = %04x, isr = 0x%08lx.\n",
		       dev->name, pp->tx_head, dp->status, isr);

	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E8_TX_STALL_BIT))
		tx_stall_recover(dev);

	pp->stats.tx_errors++;
	pp->stats.tx_aborted_errors++;
	if (dp->status & TUDR)
		pp->stats.tx_fifo_errors++;
	if (dp->status & HBF)
		pp->stats.tx_heartbeat_errors++;
}

static void
candy_tx_done(struct net_device *dev, ulong isr)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	struct candy_desc *dp;
	unsigned stop;
	if (netif_msg_tx_done(pp))
		printk(KERN_DEBUG "%s: tx done, process %d frames from %d\n",
		       dev->name, pp->tx_tail > pp->tx_head ?
		       pp->tx_tail - pp->tx_head :
		       pp->tx_tail + TX_RING_SIZE - pp->tx_head, pp->tx_head);

	spin_lock(&pp->lock);	/* sync with xmit() */
	if (pdata->platform_options.use_tx_ring_buffer)
		stop = pp->tx_tail;
	else
		stop = pp->tx_stop;

	while (pp->tx_head != stop) {
		dp = &pp->tx_ring[pp->tx_head];
		DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

		/* deal with null descriptor, the "stop" packet */
		if (!pdata->platform_options.use_tx_ring_buffer) {
			if (dp->status == 0) {
				DBG_MSG_ON((dp->size != 0),"ERROR:candy descriptor size != 0 ");
				DBG_MSG_ON((dp->pointer != 0),"ERROR:candy descriptor pointer != 0" );
				reclaim_one_tx_desc(dev);
				continue;
			}
		}

		/* how about checking OWN bit */
		if (!(dp->status & OWN)) {
			if (netif_msg_tx_done(pp))
				printk(KERN_DEBUG "%s: tx done, "
				       "found pkt being sent. Break the loop\n",
				       dev->name);
			break;
		}

		/* handle error */
		if (!(dp->status & TOK)) {
			handle_tx_error(dev, dp, isr);
		}

		/* reclaim the descriptor */
		if (!pp->tx_skb[pp->tx_head]) {
			printk(KERN_ERR "%s: tx_done but without skb!\n",
			       dev->name);
		}
		reclaim_one_tx_desc(dev);

		/* FIXME: The Japanese version has a tx restart under
		 * certain error conditions.  Don't understand it.
		 */
	}

	/* check if tx has stopped */
	if (!pdata->platform_options.use_tx_ring_buffer) {
		if ((pp->tx_head == pp->tx_stop) && (pp->tx_stop != pp->tx_tail)) {
			restart_tx_hw(dev);
		}
	}
	/* check if queue were stopped */
	if (netif_queue_stopped(dev) && (pp->tx_count < TX_RING_SIZE - 2)) {
		if (netif_msg_tx_done(pp))
			printk(KERN_DEBUG "%s: tx done, queue becomes free,"
			       "wake up net queue\n", dev->name);
		netif_wake_queue(dev);
	}

	spin_unlock(&pp->lock);
}

static void
candy_update_stats(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	candy_regs *p = pp->regs;

	/* some stats we get from hardware, while the rest we do
	 * counting by ourselves
	 */
	pp->stats.rx_packets = candy_in(p->rpkt);
	pp->stats.tx_packets = candy_in(p->tpct);
	pp->stats.rx_bytes = candy_in(p->rbyt);
	pp->stats.tx_bytes = candy_in(p->tbyt);

	/* we count rx_errors, tx_errors, rx_dropped, tx_dropped */

	pp->stats.multicast = candy_in(p->rmca);
	pp->stats.multicast = candy_in(p->tncl);

	pp->stats.rx_length_errors = candy_in(p->rund) +
	    candy_in(p->rovr) + candy_in(p->rfrg) + candy_in(p->rjbr);

	/* we count rx_over_errors */

	pp->stats.rx_crc_errors = candy_in(p->rfcs);
	pp->stats.rx_frame_errors = candy_in(p->raln);

	/* we count rx_fifo_errors and rx_missed_errors */

	/* we count tx_aborted_errors */

	pp->stats.tx_carrier_errors = candy_in(p->tcse);

	/* we count tx_fifo_errors, heartbeat_errors */

	pp->stats.tx_window_errors = candy_in(p->tlcl);

	/* we don't have rx_compressed and tx_compressed */
}

/***********************************************************************
 * high-level linux-related functions
 ***********************************************************************/
static int candy_open(struct net_device *dev);
static int candy_close(struct net_device *dev);
static irqreturn_t
candy_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *) dev_instance;
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	candy_regs *p = pp->regs;
	ulong isr;

	isr = candy_in(p->isr);

	if (netif_msg_intr(pp)) {
		printk(KERN_DEBUG "%s: intr, isr = 0x%08lx\n", dev->name, isr);
		candy_check_intr(isr,pdata);
	}

	if (isr & BUSERR) {
		printk(KERN_ERR "%s: bus error ... resetting\n", dev->name);
		candy_error_recover(dev);
		return IRQ_HANDLED;

	}
#ifdef CONFIG_CANDY_NAPI
	if (isr & RBDRU) {
		if (WORKAROUND(pdata->platform_id,WORKAROUND_E20_VR4133AR20)){
			if ( pp->rx_disable == 0 ) {
				candy_clear_macc1_bits(p->macc1, SRXEN);
				pp->rx_disable = 1;
			}
		}
		else
		pp->rx_disable = 1;
	}

	if (isr & RCVDN) {
		if (netif_rx_schedule_prep(dev)) {
			candy_mask_napi_ints(p);
			__netif_rx_schedule(dev);
		}
	}
#else
	if (isr & INT_ISR_RX_MASK) {
		candy_rx(dev, isr);
	}
#endif

	if (isr & INT_ISR_TX_MASK) {
		candy_tx_done(dev, isr);
	}

	/* we may need to do something with other intrs too in the future */

	return IRQ_HANDLED;
}

static void
candy_get_phy_status(struct net_device *dev, int *duplex, int *linkup,
		     int *speed)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	u16 reg;

	reg = mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_BMCR);

	if (!(reg & BMCR_ANENABLE)) {
		/*
		 * Auto-negotiation is disabled so the full duplex bit in
		 * the control register tells us if the PHY is running
		 * half or full duplex.
		 */
		*duplex = (reg & BMCR_FULLDPLX) ? 1 : 0;
	} else {
		/*
		 * Auto-negotiation is enabled.  Figure out what was
		 * negotiated by looking for the best mode in the union
		 * of what we and our partner advertise.
		 */
		u16 advertise, partner, negotiated;

		advertise = mdio_read(pp->mii_if.dev,
				      pp->mii_if.phy_id, MII_ADVERTISE);
		partner = mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_LPA);

		negotiated = advertise & partner & ADVERTISE_ALL;
		if (negotiated & ADVERTISE_100FULL) {
			*duplex = 1;
			*speed = 1;
		} else if (negotiated & ADVERTISE_100HALF) {
			*duplex = 0;
			*speed = 1;
		} else if (negotiated & ADVERTISE_10FULL) {
			*duplex = 1;
			*speed = 0;
		} else {
			*duplex = 0;
			*speed = 0;
		}
	}

	reg = mdio_read(pp->mii_if.dev, pp->mii_if.phy_id, MII_BMSR);

	*linkup = (reg & BMSR_LSTATUS) != 0;
}

static void
candy_poll_mii(unsigned long data)
{
	struct net_device *dev = (struct net_device *) data;
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	int phy_duplex, mac_duplex;
	int phy_carrier, netif_carrier;
	int phy_speed, mac_speed;

		/* First, find out what's going on with the PHY. */
		candy_get_phy_status(dev, &phy_duplex, &phy_carrier,
				     &phy_speed);

		/* Second, figure out if we have the EMAC in half or full duplex. */
		mac_duplex = pp->link_status.fullduplex;
		mac_speed = pp->link_status.speed100;

		/* Now see if there is a mismatch. */
		if ((mac_duplex != phy_duplex || phy_speed != mac_speed)
		    && phy_carrier) {
			/* reset */
			netif_stop_queue(dev);

			candy_down(dev);
			/* free tx skb */
			if (pdata->platform_options.use_tx_ring_buffer) {
				while (pp->tx_tail != pp->tx_head) {
					if (pp->tx_skb[pp->tx_head]) {
						dev_kfree_skb_irq(pp->tx_skb[pp->tx_head]);
						pp->tx_skb[pp->tx_head] = NULL;
					}
					pp->tx_count--;
					DBG_MSG_ON((pp->tx_count < 0),"ERROR: tx_count (%d) negative",\
					pp->tx_count);
					if (++pp->tx_head == TX_RING_SIZE)
						pp->tx_head = 0;
				}
				candy_init_rings(dev);
			}
			candy_hw_init(dev);
			candy_up(dev);
			netif_wake_queue(dev);
			if (phy_duplex == 1 && phy_speed == 1) {
				candy_set_macc1_bits(pp->regs->macc1, FULLD);
				printk(KERN_INFO
				       "%s: Duplex has been changed: now %s\n",
				       dev->name, "100Mbps Full Duplex");
			} else if (phy_duplex == 1 && phy_speed == 0) {
				candy_set_macc1_bits(pp->regs->macc1, FULLD);
				printk(KERN_INFO
				       "%s: Duplex has been changed: now %s\n",
				       dev->name, "10Mbps Full Duplex");
			} else if (phy_duplex == 0 && phy_speed == 1) {
				candy_clear_macc1_bits(pp->regs->macc1, FULLD);
				printk(KERN_INFO
				       "%s: Duplex has been changed: now %s\n",
				       dev->name, "100Mbps Half Duplex");
			} else if (phy_duplex == 0 && phy_speed == 0) {
				candy_clear_macc1_bits(pp->regs->macc1, FULLD);
				printk(KERN_INFO
				       "%s: Duplex has been changed: now %s\n",
				       dev->name, "10Mbps Half Duplex");
			}

			candy_set_media_speed(pp);
			pp->link_status.fullduplex = phy_duplex;
			pp->mii_if.full_duplex = phy_duplex;
		}
		netif_carrier = netif_carrier_ok(dev) != 0;

		if (phy_carrier != netif_carrier) {
			if (phy_carrier) {
				candy_set_media_speed(pp);
				printk(KERN_INFO "%s: Link carrier restored.\n",
				       dev->name);
				netif_carrier_on(dev);
			} else {
				printk(KERN_INFO "%s: Link carrier lost.\n",
				       dev->name);
				netif_carrier_off(dev);
			}
		}


	/* Set up the timer so we'll get called again in 2 seconds. */
	pp->phy_timer.expires = jiffies + 2 * HZ;
	add_timer(&pp->phy_timer);
}

static void
candy_poll_marvell(unsigned long data)
{
	struct net_device *dev = (struct net_device *) data;
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	uint newStatusPort;
	uint oldCarrierStatus;
	uint newCarrierStatus;
	uint newSpeedPort;
	uint oldSpeedPort;
	uint i;
	uint nports = (pdata->platform_options.n_marvell_ports < MAX_MARVELL_PORTS) ? \
		       pdata->platform_options.n_marvell_ports : MAX_MARVELL_PORTS;
	for (i=0;i<nports;i++){
		/* Port i */
		oldCarrierStatus = pp->carrierPort[i];
		newStatusPort =
		    mdio_read(pp->mii_if.dev, pp->mii_if.phy_id + PORT0 + i,
			      MARVELL_PORTSTAT);

		if (newStatusPort & PORTSTAT_LINKUP) {
			newCarrierStatus = 1;
		} else {
			newCarrierStatus = 0;
		}

		if (newCarrierStatus != oldCarrierStatus) {
			if (newCarrierStatus) {
				printk(KERN_INFO
				       "%s: PORT%d Link carrier restored.\n",
				       dev->name,i);
			} else {
				printk(KERN_INFO
				       "%s: PORT%d Link carrier lost.\n",
				       dev->name,i);
			}
			pp->carrierPort[i] = newCarrierStatus;
		}


		if (pp->carrierPort[i]) {
			oldSpeedPort = pp->speedPort[i];
			newSpeedPort = newStatusPort &
			    (PORTSTAT_FULLDUPLEX | PORTSTAT_SPEED100);

			if (newSpeedPort != oldSpeedPort) {
				if ((newSpeedPort) ==
				    (PORTSTAT_FULLDUPLEX | PORTSTAT_SPEED100)) {
					printk(KERN_INFO
					       "%s: PORT%d has been changed: now %s\n",
					       dev->name,i,
					       "100Mbps Full Duplex");
				} else if ((newSpeedPort) ==
					   (PORTSTAT_FULLDUPLEX)) {
					printk(KERN_INFO
					       "%s: PORT%d has been changed: now %s\n",
					       dev->name,i, "10Mbps Full Duplex");
				} else if ((newSpeedPort) ==
					   (PORTSTAT_SPEED100)) {
					printk(KERN_INFO
					       "%s: PORT%d has been changed: now %s\n",
					       dev->name,i,
					       "100Mbps Half Duplex");
				} else if ((newSpeedPort) == (0x00)) {
					printk(KERN_INFO
					       "%s: PORT%d has been changed: now %s\n",
					       dev->name,i, "10Mbps Half Duplex");
				}

				pp->speedPort[i] = newSpeedPort;
			}
    		    }

		}


	/* Set up the timer so we'll get called again in 2 seconds. */
	pp->phy_timer.expires = jiffies + 2 * HZ;
	add_timer(&pp->phy_timer);
}

static int
candy_open(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	int retval;
	int count = 0;

	if (netif_msg_ifup(pp))
		printk(KERN_DEBUG "%s: enabling interface\n", dev->name);

	candy_init_rings(dev);

	candy_hw_init(dev);
	candy_up(dev);

	dev->flags |= IFF_RUNNING;

	/* request IRQ */
	retval = request_irq(dev->irq, &candy_interrupt, SA_SHIRQ,
			     dev->name, dev);
	if (retval) {
		printk(KERN_ERR "%s: unable to get IRQ %d\n",
		       dev->name, dev->irq);
		return retval;
	}

	init_timer(&pp->phy_timer);

	pp->phy_timer.data = (unsigned long) dev;
	if (pp->oui == MARVELL_OUI) {
		pp->phy_timer.function = &candy_poll_marvell;
		candy_poll_marvell((unsigned long) dev);
	} else {
		pp->phy_timer.function = &candy_poll_mii;
		candy_poll_mii((unsigned long) dev);
	}
	while (!netif_carrier_ok(dev) && ++count < 5) {
		udelay(10);
	}

	netif_start_queue(dev);

	return 0;
}

static int
candy_close(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;

	if (netif_msg_ifdown(pp))
		printk(KERN_DEBUG "%s: disabling interface\n", dev->name);

	del_timer(&pp->phy_timer);

	dev->flags &= ~(IFF_UP | IFF_RUNNING);

	if (netif_device_present(dev)) {
		netif_stop_queue(dev);
		netif_carrier_off(dev);
		candy_down(dev);

		/* free tx skb */
		while (pp->tx_tail != pp->tx_head) {
			if (pp->tx_skb[pp->tx_head]) {
				dev_kfree_skb_irq(pp->tx_skb[pp->tx_head]);
				pp->tx_skb[pp->tx_head] = NULL;
			}

			pp->tx_count--;
			DBG_MSG_ON((pp->tx_count < 0),"ERROR: tx_count (%d) negative",\
			pp->tx_count);

			if (++pp->tx_head == TX_RING_SIZE)
				pp->tx_head = 0;
		}
	}

	free_irq(dev->irq, dev);

	return 0;
}

static int
candy_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	ulong flags;
	struct candy_desc *dp = &pp->tx_ring[pp->tx_tail];
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;

	DBG_MSG_ON((KSEGX(dp) != KSEG1),"candy_desc should be in KSEG1");

	if (netif_msg_tx_queued(pp))
		printk(KERN_DEBUG "%s: tx queued, skblen %d\n",
		       dev->name, skb->len);

	/* check packet size */
	if (skb->len > ETH_FRAME_LEN) {
		printk(KERN_ERR "%s: packet size too big, %d\n", dev->name,
		       skb->len);
		pp->stats.tx_dropped++;
		return 1;
	}

	spin_lock_irqsave(&pp->lock, flags);

	/* check to see if tx_ring is full */
	if (pp->tx_count >= TX_RING_SIZE - 1) {
		printk(KERN_ERR "%s: TX ring full, packet dropped.\n",
		       dev->name);
		pp->stats.tx_dropped++;
		spin_unlock_irqrestore(&pp->lock, flags);
		/* why the queue was not stopped before we get here? */
		netif_stop_queue(dev);
		return 1;
	}

	/* add the descriptor */
	{
		ushort temp = skb->len;
		if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E21_PAD_BIT)) {
			if (temp < 60)
				temp = 60;
		}

		dma_cache_wback_inv((ulong) (skb->data), skb->len);

		if (pdata->platform_options.use_tx_ring_buffer)
			append_one_tx_desc(dev, LAST | DB_LP | IEN , temp,
						CPHYSADDR(skb->data), skb);
		else
			append_one_tx_desc(dev, LAST | DB_LP, temp,
					CPHYSADDR(skb->data), skb);
	}

	/* logistics */
	dev->trans_start = jiffies;

	/* do we need to start sending or just append */
	if (!pdata->platform_options.use_tx_ring_buffer){
		if ((pp->tx_head == pp->tx_stop) && (pp->tx_stop != pp->tx_tail)) {
			restart_tx_hw(dev);
		}
	}

	if (pp->tx_count >= TX_RING_SIZE - 2) {
		netif_stop_queue(dev);
	}

	spin_unlock_irqrestore(&pp->lock, flags);

	return 0;
}

static struct net_device_stats *
candy_get_stats(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	unsigned long flags;

	if (netif_device_present(dev)) {
		spin_lock_irqsave(&pp->lock, flags);
		candy_update_stats(dev);
		spin_unlock_irqrestore(&pp->lock, flags);
	}
	return &pp->stats;
}

static void
candy_set_rx_mode(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;
	struct nec_candy_platform_data *pdata = pp->dev->platform_data;
	candy_regs *p = pp->regs;
	ulong val;

	DEBUG_VERBOSE(printk("candy_set_rx_mode() invoked.\n"));

	/* TODO: need to acquire spinlock and stop receiving */

	val = candy_in(p->afr);
	val &= ~PRO;
	if (dev->flags & IFF_PROMISC) {
		val |= PRO;
	} else if ((dev->flags & IFF_ALLMULTI) || (dev->mc_count > 64)) {
		/* disable promiscuous mode, use normal mode */
		candy_set_filter(dev, 0);
	} else if (dev->mc_count) {
		/* walk the address list, and load the filter */
		candy_set_filter(dev, 1);
	}
	if (COMMON_WORKAROUND(pdata->platform_id,WORKAROUND_E10_PRM_AMC_BIT))
		candy_out(p->afr, val & ABC);
	else
		candy_out(p->afr, val);
}

static void
candy_tx_timeout(struct net_device *dev)
{
	struct candy_private *pp = (struct candy_private *) dev->priv;

	printk(KERN_ERR "%s : tx_timeout.\n", dev->name);

	pp->stats.tx_errors++;

	candy_error_recover(dev);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void candy_napi_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	candy_interrupt(dev->irq, dev, NULL);
	enable_irq(dev->irq);
}
#endif

static int nec_candy_probe(struct device *dev)
{
	int i, ret;
	struct resource *res;
	struct net_device *ndev;
	candy_regs *p;
	struct candy_private *pp;
	struct nec_candy_platform_data *pdata = dev->platform_data;
	struct platform_device *pdev = to_platform_device(dev);

	printk(KERN_INFO "%s\n", version);

	/*
	 * hardware is already initialized.  We just need do some Linux
	 * related initialization.
	 */

	/* create net_device structure */
	ndev = alloc_etherdev(sizeof(struct candy_private));
	if (!ndev) {
		printk(KERN_ERR "ether device alloc failed. aborting\n");
		ret = -EINVAL;
		goto err1;
	}

	/* init some device related data/func ptrs */

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "nec_candy_regs");
	if (!res) {
		ret = -ENODEV;
		goto err2;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1,
				"NEC_CANDY")) {
		ret = -EBUSY;
		goto err2;
	}

	ndev->base_addr = (unsigned long) ioremap(res->start,
						  res->end - res->start + 1);
	if (!ndev->base_addr) {
		ret = -ENOMEM;
		goto err3;
	}

	ndev->irq = platform_get_irq_byname(pdev, "nec_candy_irq");

	for (i = 0; i < 6; i++)
		ndev->dev_addr[i] = pdata->mac_addr[i];

	ndev->open = candy_open;
	ndev->stop = candy_close;
	ndev->do_ioctl = candy_ioctl;
	ndev->hard_start_xmit = candy_xmit;
	ndev->get_stats = candy_get_stats;
	ndev->set_multicast_list = candy_set_rx_mode;
	ndev->tx_timeout = candy_tx_timeout;
	ndev->watchdog_timeo = TX_TIMEOUT;

#ifdef CONFIG_CANDY_NAPI
	ndev->poll = candy_napi_poll;
	ndev->weight = 64;
#ifdef CONFIG_NET_POLL_CONTROLLER
	ndev->poll_controller = candy_napi_poll_controller;
#endif
#endif

	/* init private data */
	pp = (struct candy_private *) ndev->priv;
	p = pp->regs = (candy_regs *) ndev->base_addr;
	pp->dev = dev;

	/* alloc tx/rx rings and rx buffers */

	pp->tx_ring = kmalloc(sizeof(struct candy_desc) * (TX_RING_SIZE + 1), GFP_ATOMIC);
	if (!pp->tx_ring) {
		ret = -ENOMEM;
		goto err4;
	}
	pp->rx_ring = kmalloc(sizeof(struct candy_desc) * (RX_RING_SIZE + 1), GFP_ATOMIC);
	if (!pp->rx_ring) {
		ret = -ENOMEM;
		goto err5;
	}

	dma_cache_inv((ulong) pp->tx_ring,
		      sizeof(struct candy_desc) * (TX_RING_SIZE + 1));
	dma_cache_inv((ulong) pp->rx_ring,
		      sizeof(struct candy_desc) * (RX_RING_SIZE + 1));

	pp->tx_ring = (void *) KSEG1ADDR(pp->tx_ring);
	pp->rx_ring = (void *) KSEG1ADDR(pp->rx_ring);

	/* allocate rx skbs */
	for (i = 0; i < RX_RING_SIZE; i++) {
		pp->rx_skb[i] = dev_alloc_skb(RX_BUF_SIZE);
		if (pp->rx_skb[i] == NULL) {
			int j;
			printk(KERN_ERR "%s: failed to alloc rx skb!", ndev->name);
			for (j=0; j < i; j++)
				dev_kfree_skb(pp->rx_skb[j]);
			kfree((void *)pp->rx_ring);
			ret = -ENOMEM;
			goto err5;
		}
		pp->rx_skb[i]->dev = ndev;
		dma_cache_inv((ulong) pp->rx_skb[i]->data, RX_BUF_SIZE);
	}

	/* set up links */
	pp->ndev = ndev;
	pp->msg_enable = (debug < 0 ? CANDY_DEF_MSG_ENABLE : debug);
	pp->next = candy_priv_head;
	candy_priv_head = pp;
	pp->mii_if.dev = ndev;
	pp->mii_if.mdio_read = mdio_read;
	pp->mii_if.mdio_write = mdio_write;
	pp->mii_if.phy_id = pdata->pmd_addr;
	pp->mii_if.phy_id_mask = 0x1f;
	pp->mii_if.reg_num_mask = 0x1f;

	spin_lock_init(&pp->lock);
	spin_lock_init(&pp->rxlock);

#ifdef CONFIG_CANDY_NAPI
	pp->prev_rpkt = candy_in(p->rpkt);
#endif
	/*==============================================================
	 * hardware initialization
	 *==============================================================
	 */

	/* TODO: maybe we want to make sure the chip is there */

	/*
	 * zero out counters
	 */
	candy_out(p->rbyt, 0);
	candy_out(p->rpkt, 0);
	candy_out(p->rfcs, 0);
	candy_out(p->rmca, 0);
	candy_out(p->rbca, 0);
	candy_out(p->rxcf, 0);
	candy_out(p->rxpf, 0);
	candy_out(p->rxuo, 0);
	candy_out(p->raln, 0);
	candy_out(p->rflr, 0);
	candy_out(p->rcde, 0);
	candy_out(p->rfcr, 0);
	candy_out(p->rund, 0);
	candy_out(p->rovr, 0);
	candy_out(p->rfrg, 0);
	candy_out(p->rjbr, 0);
	candy_out(p->r64, 0);
	candy_out(p->r127, 0);
	candy_out(p->r255, 0);
	candy_out(p->r511, 0);
	candy_out(p->r1k, 0);
	candy_out(p->rmax, 0);
	candy_out(p->rvbt, 0);

	candy_out(p->tbyt, 0);
	candy_out(p->tpct, 0);
	candy_out(p->tfcs, 0);
	candy_out(p->tmca, 0);
	candy_out(p->tbca, 0);
	candy_out(p->tuca, 0);
	candy_out(p->txpf, 0);
	candy_out(p->tdfr, 0);
	candy_out(p->txdf, 0);
	candy_out(p->tscl, 0);
	candy_out(p->tmcl, 0);
	candy_out(p->tlcl, 0);
	candy_out(p->txcl, 0);
	candy_out(p->tncl, 0);
	candy_out(p->tcse, 0);
	candy_out(p->time, 0);

	ret = register_netdev(ndev);
	if (ret != 0)
		goto err6;

	printk (KERN_INFO DRV_NAME ": Probe candy chip at "
		"0x%08lx, irq %d, Phy ID 0x%08x, "
		"MAC Addr %02x:%02x:%02x:%02x:%02x:%02x\n",
		res->start, ndev->irq, pp->mii_if.phy_id,
		ndev->dev_addr[0], ndev->dev_addr[1],
		ndev->dev_addr[2], ndev->dev_addr[3],
		ndev->dev_addr[4], ndev->dev_addr[5]);

	return 0;

err6:
	printk(KERN_ERR "Failed to register ethernet device\n");
	for (i = 0; i < RX_RING_SIZE; i++)
		dev_kfree_skb(pp->rx_skb[i]);
	kfree((void *)pp->rx_ring);
err5:
	kfree((void *)pp->tx_ring);
err4:
	iounmap((void *) ndev->base_addr);
err3:
	release_mem_region(res->start, res->end - res->start + 1);
err2:
	free_netdev(ndev);
err1:
	return ret;
}


/***********************************************************************
 * Module hookup
 ***********************************************************************
 */

static int nec_candy_remove(struct device *dev)
{
	int i;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct candy_private *pp = netdev_priv(ndev);
	struct resource *res =
		platform_get_resource_byname(to_platform_device(dev),
					     IORESOURCE_MEM,
					     "nec_candy_regs");

	for (i = 0; i < RX_RING_SIZE; i++)
		dev_kfree_skb(pp->rx_skb[i]);

	kfree((void *)pp->rx_ring);
	kfree((void *)pp->tx_ring);

	iounmap((void *)ndev->base_addr);
	release_mem_region(res->start, res->end - res->start + 1);

	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
}

static struct device_driver nec_candy_driver = {
	.name		= "nec_candy",
	.bus		= &platform_bus_type,
	.probe		= nec_candy_probe,
	.remove		= nec_candy_remove,
};

static int __init nec_candy_module_init(void)
{
	return driver_register(&nec_candy_driver);
	return 0;
}

static void __exit nec_candy_module_exit(void)
{
	driver_unregister(&nec_candy_driver);
}

module_init(nec_candy_module_init);
module_exit(nec_candy_module_exit);

#if defined(MODULE)
MODULE_AUTHOR("Jun Sun, jsun@mvista.com or jsun@junsun.net");
MODULE_LICENSE("GPL");
#endif
