/*
	mv88e6063.c: 1) Marvell switch initialisation
		     2) VLAN configuration
	Derived from sb500mii.c written and maintained
	     by Edmond dela Cruz <edmondd@ntsp.nec.co.jp>
	Copyright:
		Written 2005 by Edmond dela Cruz.
		Modified: 2007, MontaVista Software Inc. <source@mvista.com>

		This software may be used and distributed according
		to the terms of the GNU General Public License (GPL),
		incorporated herein by reference.  Drivers based on
		or derived from this code fall under the GPL and must
		retain the authorship, copyright and license notice.
		This file is not a complete program and may only be
		used when the entire operating system is licensed
		under the GPL.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include "mv88e6063.h"
#include <linux/delay.h>

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)

static int vlan_config(struct mii_if_info *mii_if,
		       struct vlan_ioctl_data *vdata)
{
	unsigned int value_L, value_H;
	int i, tempPort;

	struct net_device *dev = mii_if->dev;
	int pmd_addr = mii_if->phy_id;

	if (strcmp(vdata->cmd, "enable") == 0) {
		/* Port Control reg. set Egress mode. */
		mii_if->mdio_write(dev, pmd_addr + PORT0, MARVELL_PORTCTRL,
				   0x1053);
		mii_if->mdio_write(dev, pmd_addr + PORT1, MARVELL_PORTCTRL,
				   0x1053);
		mii_if->mdio_write(dev, pmd_addr + PORT2, MARVELL_PORTCTRL,
				   0x1053);
		mii_if->mdio_write(dev, pmd_addr + PORT3, MARVELL_PORTCTRL,
				   0x1053);
		mii_if->mdio_write(dev, pmd_addr + PORT4, MARVELL_PORTCTRL,
				   0x1053);
		mii_if->mdio_write(dev, pmd_addr + PORT5, MARVELL_PORTCTRL,
				   0x2053);

		/* Port Based VLAN Map reg. set Fallback mode */
		mii_if->mdio_write(dev, pmd_addr + PORT0, MARVELL_PORTVLANMAP,
				   0x0400 | (mii_if->
					     mdio_read(dev, pmd_addr + PORT0,
						       MARVELL_PORTVLANMAP) &
					     0x003f));
		mii_if->mdio_write(dev, pmd_addr + PORT1, MARVELL_PORTVLANMAP,
				   0x0400 | (mii_if->
					     mdio_read(dev, pmd_addr + PORT1,
						       MARVELL_PORTVLANMAP) &
					     0x003f));
		mii_if->mdio_write(dev, pmd_addr + PORT2, MARVELL_PORTVLANMAP,
				   0x0400 | (mii_if->
					     mdio_read(dev, pmd_addr + PORT2,
						       MARVELL_PORTVLANMAP) &
					     0x003f));
		mii_if->mdio_write(dev, pmd_addr + PORT3, MARVELL_PORTVLANMAP,
				   0x0400 | (mii_if->
					     mdio_read(dev, pmd_addr + PORT3,
						       MARVELL_PORTVLANMAP) &
					     0x003f));
		mii_if->mdio_write(dev, pmd_addr + PORT4, MARVELL_PORTVLANMAP,
				   0x0400 | (mii_if->
					     mdio_read(dev, pmd_addr + PORT4,
						       MARVELL_PORTVLANMAP) &
					     0x003f));
		mii_if->mdio_write(dev, pmd_addr + PORT5, MARVELL_PORTVLANMAP,
				   0x0400 | (mii_if->
					     mdio_read(dev, pmd_addr + PORT5,
						       MARVELL_PORTVLANMAP) &
					     0x003f));
	} else if (strcmp(vdata->cmd, "set") == 0) {

		/* Set VLAN ID */
		if (vdata->port > 0x001f) {
			return -1;
		}

		if (vdata->port & 0x0001) {
			mii_if->mdio_write(dev, pmd_addr + PORT0,
					   MARVELL_PORTVLANID, vdata->vlan_num);
			tempPort = PORT0;
		}
		if (vdata->port & 0x0002) {
			mii_if->mdio_write(dev, pmd_addr + PORT1,
					   MARVELL_PORTVLANID, vdata->vlan_num);
			tempPort = PORT1;
		}
		if (vdata->port & 0x0004) {
			mii_if->mdio_write(dev, pmd_addr + PORT2,
					   MARVELL_PORTVLANID, vdata->vlan_num);
			tempPort = PORT2;
		}
		if (vdata->port & 0x0008) {
			mii_if->mdio_write(dev, pmd_addr + PORT3,
					   MARVELL_PORTVLANID, vdata->vlan_num);
			tempPort = PORT3;
		}
		if (vdata->port & 0x0010) {
			mii_if->mdio_write(dev, pmd_addr + PORT4,
					   MARVELL_PORTVLANID, vdata->vlan_num);
			tempPort = PORT4;
		}

		for (i = 0; i <= (PORT4 - PORT0); i++) { /* Only PORT0 to PORT4 */
			if ((tempPort & PORT0 + i) == tempPort) {
				/* Set the target port to 0 ! */
				mii_if->mdio_write(dev,
						   pmd_addr + PORT0 + i,
						   MARVELL_PORTVLANMAP,
						   (mii_if->
						    mdio_read(dev,
							      pmd_addr +
							      PORT0 + i,
							      MARVELL_PORTVLANMAP)
						    & ~vdata->port));
			} else if (vdata->vlan_num ==
				   (mii_if->
				    mdio_read(dev, pmd_addr + PORT0 + i,
					      MARVELL_PORTVLANID) & vdata->
				    vlan_num)) {
				int thisPort = 1 << i;

				/* We have the same VID, allow this port to communicate with the target port! */
				mii_if->mdio_write(dev, pmd_addr + PORT0 + i,
						   MARVELL_PORTVLANMAP,
						   (mii_if->
						    mdio_read(dev,
							      pmd_addr + PORT0 +
							      i,
							      MARVELL_PORTVLANMAP)
						    | vdata->port));

				/* Allow the target port to communicate with this port! */
				mii_if->mdio_write(dev, pmd_addr + tempPort,
						   MARVELL_PORTVLANMAP,
						   (mii_if->
						    mdio_read(dev,
							      pmd_addr +
							      tempPort,
							      MARVELL_PORTVLANMAP)
						    | thisPort));
			} else {
				int thisPort = 1 << i;

				/* Do not allow this port to communicate with target port. */
				mii_if->mdio_write(dev, pmd_addr + PORT0 + i,
						   MARVELL_PORTVLANMAP,
						   (mii_if->
						    mdio_read(dev,
							      pmd_addr + PORT0 +
							      i,
							      MARVELL_PORTVLANMAP)
						    & ~vdata->port));

				/* Do not allow the target port to communicate with this port. */
				mii_if->mdio_write(dev, pmd_addr + tempPort,
						   MARVELL_PORTVLANMAP,
						   (mii_if->
						    mdio_read(dev,
							      pmd_addr +
							      tempPort,
							      MARVELL_PORTVLANMAP)
						    & ~thisPort));
			}
		}

		/* Port5 is always open for every ports. */
		mii_if->mdio_write(dev, pmd_addr + PORT5, MARVELL_PORTVLANMAP,
				   0x001f |
				   (mii_if->
				    mdio_read(dev, pmd_addr + PORT5,
					      MARVELL_PORTVLANMAP) & 0xffc0));

		/* Set VTU */
		mii_if->mdio_write(dev, pmd_addr + PORT_GLOBAL, MARVELL_VTU_VID,
				   vdata->vlan_num);

		value_L = 0x1111;
		if (vdata->port & 0x0001)
			value_L = (0x0002 | (0xfff0 & value_L));
		if (vdata->port & 0x0002)
			value_L = (0x0020 | (0xff0f & value_L));
		if (vdata->port & 0x0004)
			value_L = (0x0200 | (0xf0ff & value_L));
		if (vdata->port & 0x0008)
			value_L = (0x2000 | (0x0fff & value_L));

		value_H = 0x0111;
		if (vdata->port & 0x0010)
			value_H = (0x0002 | (0xfff0 & value_H));
		value_H |= 0x0030;	/* Port5 is always tagged. */

		mii_if->mdio_write(dev, pmd_addr + PORT_GLOBAL,
				   MARVELL_VTU_DATAL, value_L);
		mii_if->mdio_write(dev, pmd_addr + PORT_GLOBAL,
				   MARVELL_VTU_DATAH, value_H);

		mii_if->mdio_write(dev, pmd_addr + PORT_GLOBAL, MARVELL_VTU_OPE,
				   0xb000);

	} else if (strcmp(vdata->cmd, "read") == 0) {
		i = vdata->vlan_num;

		/* Read VLAN ID */
		if (i == (mii_if->mdio_read(dev, pmd_addr + PORT0,
					    MARVELL_PORTVLANID) & 0x0fff)) {
			vdata->port |= 0x0001;	/* PORT0 Flag */
		}
		if (i == (mii_if->mdio_read(dev, pmd_addr + PORT1,
					    MARVELL_PORTVLANID) & 0x0fff)) {
			vdata->port |= 0x0002;	/* PORT1 Flag */
		}
		if (i == (mii_if->mdio_read(dev, pmd_addr + PORT2,
					    MARVELL_PORTVLANID) & 0x0fff)) {
			vdata->port |= 0x0004;	/* PORT2 Flag */
		}
		if (i == (mii_if->mdio_read(dev, pmd_addr + PORT3,
					    MARVELL_PORTVLANID) & 0x0fff)) {
			vdata->port |= 0x0008;	/* PORT3 Flag */
		}
		if (i == (mii_if->mdio_read(dev, pmd_addr + PORT4,
					    MARVELL_PORTVLANID) & 0x0fff)) {
			vdata->port |= 0x0010;	/* PORT4 Flag */
		}
	} else if (strcmp(vdata->cmd, "disable") == 0) {
		printk("Disable tagVLAN!\n");
		/* Port Control reg. set Egress mode. */
		mii_if->mdio_write(dev, pmd_addr + PORT0, MARVELL_PORTCTRL,
				   0x0003);
		mii_if->mdio_write(dev, pmd_addr + PORT1, MARVELL_PORTCTRL,
				   0x0003);
		mii_if->mdio_write(dev, pmd_addr + PORT2, MARVELL_PORTCTRL,
				   0x0003);
		mii_if->mdio_write(dev, pmd_addr + PORT3, MARVELL_PORTCTRL,
				   0x0003);
		mii_if->mdio_write(dev, pmd_addr + PORT4, MARVELL_PORTCTRL,
				   0x0003);
		mii_if->mdio_write(dev, pmd_addr + PORT5, MARVELL_PORTCTRL,
				   0x0003);

		/* Port Based VLAN Map reg. set Fallback mode */
		mii_if->mdio_write(dev, pmd_addr + PORT0, MARVELL_PORTVLANMAP,
				   0x007e);
		mii_if->mdio_write(dev, pmd_addr + PORT1, MARVELL_PORTVLANMAP,
				   0x007d);
		mii_if->mdio_write(dev, pmd_addr + PORT2, MARVELL_PORTVLANMAP,
				   0x007b);
		mii_if->mdio_write(dev, pmd_addr + PORT3, MARVELL_PORTVLANMAP,
				   0x0077);
		mii_if->mdio_write(dev, pmd_addr + PORT4, MARVELL_PORTVLANMAP,
				   0x006f);
		mii_if->mdio_write(dev, pmd_addr + PORT5, MARVELL_PORTVLANMAP,
				   0x005f);

		/* VLAN ID clear */
		mii_if->mdio_write(dev, pmd_addr + PORT0, MARVELL_PORTVLANID,
				   1);
		mii_if->mdio_write(dev, pmd_addr + PORT1, MARVELL_PORTVLANID,
				   1);
		mii_if->mdio_write(dev, pmd_addr + PORT2, MARVELL_PORTVLANID,
				   1);
		mii_if->mdio_write(dev, pmd_addr + PORT3, MARVELL_PORTVLANID,
				   1);
		mii_if->mdio_write(dev, pmd_addr + PORT4, MARVELL_PORTVLANID,
				   1);
		mii_if->mdio_write(dev, pmd_addr + PORT5, MARVELL_PORTVLANID,
				   1);
	} else if (strcmp(vdata->cmd, "print") == 0) {
		printk(KERN_INFO "\nVLAN VIDs :\n");
		for (i = 0; i <= (PORT5 - PORT0); i++) {
			printk(KERN_INFO "PORT%d VID is : 0x%04X\n", i,
			       mii_if->mdio_read(dev, pmd_addr + PORT0 + i,
						 MARVELL_PORTVLANID));
		}

		printk(KERN_INFO "\nVLAN VTable :\n");
		for (i = 0; i <= (PORT5 - PORT0); i++) {
			int nPortTable = 0;
			nPortTable =
			    mii_if->mdio_read(dev, pmd_addr + PORT0 + i,
					      MARVELL_PORTVLANMAP);
			printk(KERN_INFO "PORT%d VLANMAP  is : 0x%04X\n", i,
			       nPortTable);
			printk(KERN_INFO "\tPORT%d VLANMAP  is :\n", i);
			if ((nPortTable & 0x001) == 0x0001) {	/* PORT0 */
				printk(KERN_INFO "\t\tPORT0\n");
			}
			if ((nPortTable & 0x0002) == 0x0002) {	/* PORT1 */
				printk(KERN_INFO "\t\tPORT1\n");
			}
			if ((nPortTable & 0x0004) == 0x0004) {	/* PORT2 */
				printk(KERN_INFO "\t\tPORT2\n");
			}
			if ((nPortTable & 0x0008) == 0x0008) {	/* PORT3 */
				printk(KERN_INFO "\t\tPORT3\n");
			}
			if ((nPortTable & 0x0010) == 0x0010) {	/* PORT4 */
				printk(KERN_INFO "\t\tPORT4\n");
			}
			if ((nPortTable & 0x0020) == 0x0020) {	/* PORT5 */
				printk(KERN_INFO "\t\tPORT5\n");
			}
		}

	} else {
		printk("NO tagVLAN command found....\n");
		return -1;
	}

	return 0;
}

#endif

int vlan_mii_ioctl(struct mii_if_info *mii_if, struct mii_ioctl_data *mii_data,
		   struct vlan_ioctl_data *vdata, int cmd)
{
	int rc = 0;

	if (cmd != (SIOCDEVPRIVATE + 13)) {
		mii_data->phy_id &= mii_if->phy_id_mask;
		mii_data->reg_num &= mii_if->reg_num_mask;
	}

	switch (cmd) {
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	case SIOCDEVPRIVATE + 11:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		mii_if->mdio_write(mii_if->dev, mii_if->dev + mii_data->phy_id,
				   mii_data->reg_num, mii_data->val_in);

		return 0;

	case SIOCDEVPRIVATE + 12:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		mii_data->val_out = mii_if->mdio_read(mii_if->dev,
						      mii_if->phy_id +
						      mii_data->phy_id,
						      mii_data->reg_num);
		return 0;

	case SIOCDEVPRIVATE + 13:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		if (vlan_config(mii_if, vdata))
			return -EOPNOTSUPP;

		return 0;
#endif

	default:
		rc = -EOPNOTSUPP;
		break;
	}
	return rc;
}

int enableMarvellSwitch(struct mii_if_info *mii)
{
	/* Enable and Configure 88E6063 Ports */
	int port, phy;
	for (port = 0; port <= (PORT5 - PORT0); port++) {
		int portctrlval;
		/* Set the port to forwarding. */
		mii->mdio_write(mii->dev, (mii->phy_id + PORT0 + port),
				MARVELL_PORTCTRL, PORTCTRL_PORTSTATEFW);
		udelay(500);

		/* Check if the port was configured properly. */
		portctrlval =
		    mii->mdio_read(mii->dev, (mii->phy_id + PORT0 + port),
				   MARVELL_PORTCTRL);
		if ((portctrlval & PORTCTRL_PORTSTATEFW) !=
		    PORTCTRL_PORTSTATEFW) {
			printk(KERN_INFO
			       "%s: Cannot setup PORT%d of Marvell Switch\n",
			       mii->dev->name, port);
		}
	}

	/* Configure MII PHY on all 88E6063 Ports */
	for (phy = 0; phy <= (MARVELL_MIIPHY4 - MARVELL_MIIPHY0); phy++) {

		/* Reset the Basic mode control register on this MII PHY */
		mii->mdio_write(mii->dev, (mii->phy_id + MARVELL_MIIPHY0 + phy),
				MII_BMCR, 0);

		/* Configure this MII PHY */
		mii->mdio_write(mii->dev, (mii->phy_id + MARVELL_MIIPHY0 + phy),
				MII_BMCR,
				BMCR_FULLDPLX | BMCR_ANRESTART | BMCR_ANENABLE |
				BMCR_SPEED100);
	}
	return 1;
}
