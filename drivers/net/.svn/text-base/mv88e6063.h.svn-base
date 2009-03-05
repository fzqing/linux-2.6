/*
 * mv88e6063.h
 * Marvell 88E6063 MII registers.
 * Marvell 88E6063 Ethernet Switch is embedded on the Source Code together
 * with Davicom DM9161 Transceiver.
 *
 * Copyright (C) 2005 Edmond D. dela Cruz (edmondd@ntsp.nec.co.jp)
 */

#ifndef __MV88E6063_H__
#define __MV88E6063_H__

#include <linux/mii.h>
/***********************************************************************
 * Marvel Ethernet Switch
 ***********************************************************************
*/
#define MARVELL_OUI 0x005043	/*Marvell Semiconducter MII OUI */
/**************************
 * MAX POLL TIMES Setting *
 **************************/
#define		MAX_POLL_TIMES_MARVELL	15000
/*************************************
 * Marvell 88E6063 Switch registers. *
 *************************************/
/* Serial Management Interface (SMI) Port Device Address. */
#define PORT0			0x08	/* SMI Device Port 0 */
#define PORT1			0x09	/* SMI Device Port 1 */
#define PORT2			0x0A	/* SMI Device Port 2 */
#define PORT3			0x0B	/* SMI Device Port 3 */
#define PORT4			0x0C	/* SMI Device Port 4 */
#define PORT5			0x0D	/* SMI Device Port 5 */
#define PORT6			0x0E	/* SMI Device Port 6*/
#define PORT_GLOBAL		0x0F	/* SMI Device Global Port */
/* Serial Management Interface (SMI) PHY MII Device Address. */
#define MARVELL_MIIPHY0			0x00	/* SMI MII PHY Device 0 */
#define MARVELL_MIIPHY1			0x01	/* SMI MII PHY Device 0 */
#define MARVELL_MIIPHY2			0x02	/* SMI MII PHY Device 0 */
#define MARVELL_MIIPHY3			0x03	/* SMI MII PHY Device 0 */
#define MARVELL_MIIPHY4			0x04	/* SMI MII PHY Device 0 */
/* Marvell Semiconducter MII OUI */
#define MARVELL_OUI 0x005043
/* PHY Identifier 1 */
#define MARVELL_OUIMSB			0x0141	/* Organizationally Unique Identifier */
/* Switch Port Register. */
#define MARVELL_PORTSTAT			0x00	/* Port Status Register */
#define MARVELL_RESERVED1			0x01	/* Reserved Register */
#define MARVELL_RESERVED2			0x02	/* Reserved Register */
#define MARVELL_SWITCHID			0x03	/* Switch Identifier Register */
#define MARVELL_PORTCTRL			0x04	/* Port Control Register */
#define MARVELL_RESERVED5			0x05	/* Reserved Register */
#define MARVELL_PORTVLANMAP			0x06	/* Port Based VLAN Map */
#define MARVELL_PORTVLANID			0x07	/* Default Port VLAN ID & Priority */
#define MARVELL_RESERVED8			0x08	/* Reserved Register */
#define MARVELL_RESERVED9			0x09	/* Reserved Register */
#define MARVELL_RATECTRL			0x0A	/* Rate Control */
#define MARVELL_PORTASSVECT			0x0B	/* Port Association Vector */
#define MARVELL_RESERVED12			0x0C	/* Reserved Register */
#define MARVELL_RESERVED13			0x0D	/* Reserved Register */
#define MARVELL_RESERVED14			0x0E	/* Reserved Register */
#define MARVELL_RESERVED15			0x0F	/* Reserved Register */
#define MARVELL_RXCOUNTER			0x10	/* Rx Counter */
#define MARVELL_TXCOUNTER			0x11	/* Tx Counter */
/* Port Status Register */
#define PORTSTAT_SPEED100		0x0100	/* Speed mode (100MBps) */
#define PORTSTAT_FULLDUPLEX		0x0200	/* Duplex mode (Full Duplex) */
#define PORTSTAT_MIIPHYMODE		0x0400	/* PHY mode (MII PHY Mode) */
#define PORTSTAT_MIIPORTMODE	0x0800	/* Port mode (MII 10/100 or RMII 100Mbps) */
#define PORTSTAT_LINKUP			0x1000	/* Link Status (Link is UP!) */
#define PORTSTAT_RESOLVED		0x2000	/* Link Mode is Resolved */
#define PORTSTAT_MYPAUSE		0x2000	/* My Pause Bit is implemented */
#define PORTSTAT_LINKPAUSE		0x2000	/* Link Partner's Pause Bit is implemented */
/* Port Control Register */
#define PORTCTRL_PORTSTATEDA	0x0000	/* Port State. Disabled = 00 */
#define PORTCTRL_PORTSTATEBL	0x0001	/* Port State. Blocking/Listening = 01 */
#define PORTCTRL_PORTSTATELE	0x0002	/* Port State. Learning = 10 */
#define PORTCTRL_PORTSTATEFW	0x0003	/* Port State. Forwarding = 11 */
#define PORTCTRL_FORCEFLOW		0x8000	/* Force Flow Control */
/* Unsorted Registers */
#define MARVELL_VTU_OPE   0x0005
#define MARVELL_VTU_VID   0x0006
#define MARVELL_VTU_DATAL 0x0007
#define MARVELL_VTU_DATAH 0x0008
/******************************************************************************/
struct vlan_ioctl_data {
	char *cmd;		/* Should be one of the vlan_ioctl_cmds enum above. */
	int vlan_num;
	int port;
};

extern int enableMarvellSwitch(struct mii_if_info *mii) __attribute__ ((weak));
extern int vlan_mii_ioctl(struct mii_if_info *mii_if,
			  struct mii_ioctl_data *mii_data,
			  struct vlan_ioctl_data *vdata, int cmd) __attribute__ ((weak));
#endif				/* __MV88E6063_H__ */
