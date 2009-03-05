/*
 * arch/mips/pci/pci-vr5701_sg2.c
 *
 * A code for PCI controllers on NEC Electronics Corporation VR5701 SolutionGearII 
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <asm/bootinfo.h>
#include <asm/debug.h>
#include <asm/byteorder.h>
#include <asm/vr5701/vr5701_sg2.h>

static struct resource extpci_io_resource = {
	"ext pci IO space",
	0x00001000,
	0x007FFFFF,
	IORESOURCE_IO
};

static struct resource extpci_mem_resource = {
	"ext pci memory space",
	0x10000000,
	0x17FFFFFF,
	IORESOURCE_MEM
};

static struct resource iopci_io_resource = {
	"io pci IO space",
	0x01000000,
	0x017FFFFF,
	IORESOURCE_IO
};

static struct resource iopci_mem_resource = {
	"io pci memory space",
	0x18800000,
	0x18FFFFFF,
	IORESOURCE_MEM
};

struct pci_controller VR5701_ext_controller = {
	.pci_ops = &VR5701_ext_pci_ops,
	.io_resource = &extpci_io_resource,
	.mem_resource = &extpci_mem_resource
};

struct pci_controller VR5701_io_controller = {
	.pci_ops = &VR5701_io_pci_ops,
	.io_resource = &iopci_io_resource,
	.mem_resource = &iopci_mem_resource
};

int __init pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int slot_num;
	int k = 0;

	slot_num = PCI_SLOT(dev->devfn);

	if (dev->bus->number == 0) {	/* EPCI */
		switch (slot_num) {
		case 24 - 11:   /* INTD# */
			k = NUM_5701_IRQS + 3;
			break;
		case 25 - 11:	/* INTC# */
			k = NUM_5701_IRQS + 2;
			break;
		case 26 - 11:	/* INTB# */
			k = NUM_5701_IRQS + 1;
			break;
		case 27 - 11:	/* INTA# */
			k = NUM_5701_IRQS + 0;
			break;
		}
	} else {		/* IPCI */
		switch (slot_num) {
		case 29 - 11:	/* INTC# */
			k = NUM_5701_IRQS + NUM_5701_EPCI_IRQ + 2;
			break;
		case 30 - 11:	/* INTB# */
			k = NUM_5701_IRQS + NUM_5701_EPCI_IRQ + 1;
			break;
		case 31 - 11:	/* INTA# */
			k = NUM_5701_IRQS + NUM_5701_EPCI_IRQ + 0;
			break;
		}
	}
	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, k);
	dev->irq = k + 8;
	return dev->irq;
}

void ddb_pci_reset_bus(void)
{
	u32 temp;

	temp = ddb_in32(EPCI_CTRLH);
	temp |= 0x80000000;
	ddb_out32(EPCI_CTRLH, temp);
	udelay(100);
	temp &= ~0xc0000000;
	ddb_out32(EPCI_CTRLH, temp);

	temp = ddb_in32(IPCI_CTRLH);
	temp |= 0x80000000;
	ddb_out32(IPCI_CTRLH, temp);
	udelay(100);
	temp &= ~0xc0000000;
	ddb_out32(IPCI_CTRLH, temp);
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
