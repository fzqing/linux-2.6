/*
 * arch/mips/pci/ops-tx4939.c
 *
 * Define the pci_ops for the Toshiba rbtx4939
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/config.h>

#include <asm/addrspace.h>
#include <asm/tx4939/tx4939.h>
#include <asm/tx4939/rbtx4939.h>
#include <asm/debug.h>

static inline struct tx4939_pcic_reg *pci_dev_to_pcicptr(struct pci_bus *dev)
{
	struct pci_controller *channel = (struct pci_controller *)dev->sysdata;
	return pcicptrs[channel - &tx4939_pci_controller[0]];
}

static int
mkaddr(unsigned char bus, unsigned char dev_fn, unsigned char where,
       int *flagsp, struct tx4939_pcic_reg *pcicptr)
{
	u32 l;
	if (bus > 0) {
		/* Type 1 configuration */
		reg_wr32(&pcicptr->g2pcfgadrs, ((bus & 0xff) << 0x10) |
			  ((dev_fn & 0xff) << 0x08) | (where & 0xfc) | 1);
	} else {
		if (dev_fn >= PCI_DEVFN(TX4939_PCIC_MAX_DEVNU, 0))
			return -1;

		/* Type 0 configuration */
		reg_wr32(&pcicptr->g2pcfgadrs, ((bus & 0xff) << 0x10) |
			  ((dev_fn & 0xff) << 0x08) | (where & 0xfc));
	}
	/* clear M_ABORT and Disable M_ABORT Int. */
	reg_wr32(&pcicptr->pcistatus,
		  (reg_rd32(&pcicptr->pcistatus) & 0x0000ffff) |
		  (PCI_STATUS_REC_MASTER_ABORT << 16));
	l = reg_rd32(&pcicptr->pcimask);
	reg_wr32(&pcicptr->pcimask, l & ~PCI_STATUS_REC_MASTER_ABORT);
	return 0;
}

static int check_abort(int flags, struct tx4939_pcic_reg *pcicptr)
{
	u32 l;
	int code = PCIBIOS_SUCCESSFUL;
	/* wait write cycle completion before checking error status */
	while (reg_rd32(&pcicptr->pcicstatus) & TX4939_PCICSTATUS_IWB) ;
	if (reg_rd32(&pcicptr->pcistatus) &
	    (PCI_STATUS_REC_MASTER_ABORT << 16)) {
		reg_wr32(&pcicptr->pcistatus,
			  (reg_rd32(&pcicptr->pcistatus) & 0x0000ffff) |
			  (PCI_STATUS_REC_MASTER_ABORT << 16));
		code = PCIBIOS_DEVICE_NOT_FOUND;
	}
	l = reg_rd32(&pcicptr->pcimask);
	reg_wr32(&pcicptr->pcimask, l | PCI_STATUS_REC_MASTER_ABORT);
	return code;
}

static int tx4939_pcibios_read_config(struct pci_bus *bus, unsigned int devfn,
				      int where, int size, u32 * val)
{
	int flags, retval, dev, busno, func;
	struct tx4939_pcic_reg *tx4939_pcicptr = pci_dev_to_pcicptr(bus);

	dev = PCI_SLOT(devfn);
	func = PCI_FUNC(devfn);

	/* check if the bus is top-level */
	if (bus->parent != NULL)
		busno = bus->number;
	else {
		busno = 0;
	}

	if (mkaddr(busno, devfn, where, &flags, tx4939_pcicptr))
		return -1;

	switch (size) {
	case 1:
		*val = *(volatile u8 *)((ulong) & tx4939_pcicptr->g2pcfgdata |
#ifdef __BIG_ENDIAN
					((where & 3) ^ 3));
#else
					(where & 3));
#endif
		break;
	case 2:
		*val = *(volatile u16 *)((ulong) & tx4939_pcicptr->g2pcfgdata |
#ifdef __BIG_ENDIAN
					 ((where & 3) ^ 2));
#else
					 (where & 3));
#endif
		break;
	case 4:
		*val = reg_rd32(&tx4939_pcicptr->g2pcfgdata);
		break;
	}

	retval = check_abort(flags, tx4939_pcicptr);
	if (retval == PCIBIOS_DEVICE_NOT_FOUND)
		*val = 0xffffffff;

	return retval;
}

static int tx4939_pcibios_write_config(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	int flags, dev, busno, func;
	struct tx4939_pcic_reg *tx4939_pcicptr = pci_dev_to_pcicptr(bus);
	busno = bus->number;
	dev = PCI_SLOT(devfn);
	func = PCI_FUNC(devfn);

	/* check if the bus is top-level */
	if (bus->parent != NULL) {
		busno = bus->number;
	} else {
		busno = 0;
	}

	if (mkaddr(busno, devfn, where, &flags, tx4939_pcicptr))
		return -1;

	switch (size) {
	case 1:
		*(volatile u8 *)((ulong) & tx4939_pcicptr->g2pcfgdata |
#ifdef __BIG_ENDIAN
				 ((where & 3) ^ 3)) = val;
#else
				 (where & 3)) = val;
#endif
		break;
	case 2:
		*(volatile u16 *)((ulong) & tx4939_pcicptr->g2pcfgdata |
#ifdef __BIG_ENDIAN
				  ((where & 0x3) ^ 0x2)) = val;
#else
				  (where & 3)) = val;
#endif
		break;
	case 4:
		reg_wr32(&tx4939_pcicptr->g2pcfgdata, val);
		break;
	}

	return check_abort(flags, tx4939_pcicptr);
}

struct pci_ops tx4939_pci_ops = {
        .read = tx4939_pcibios_read_config,
	.write = tx4939_pcibios_write_config
};
