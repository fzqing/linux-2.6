/*
 * arch/mips/pci/fixup-tx4939.c
 *
 * Toshiba rbtx4939 pci routines
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

#include <linux/config.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/tx4939/rbtx4939.h>

void __init pcibios_fixup_resources(struct pci_dev *dev)
{
	/* nothing to do here */
}

void __init pcibios_fixup(void)
{
	/* nothing to do here */
}

/**
 * pci_get_irq - get IRQ number of pci device
 * @dev: pci_dev structure
 * @pin: pci interrupt pin
 *
 *
 */
static int __init pci_get_irq(struct pci_dev *dev, int pin)
{
	int irq = pin;
	u8 slot = PCI_SLOT(dev->devfn);
	struct pci_controller *controller =
	    (struct pci_controller *)dev->sysdata;
	struct pci_bus *bus;

	pr_debug("pci_get_irq: pin is %d, slot is %d\n", pin, slot);
	if (controller == &tx4939_pci_controller[1]) {
		unsigned int i;
		for (i = 0; i < 2; i++) {
			if (slot ==
			    TX4939_PCIC_IDSEL_AD_TO_SLOT(TX4939_ETHER_IDSEL(i))) {
				if (reg_rd64s(&tx4939_ccfgptr->pcfg) & (TX4939_PCFG_ET0MODE_ETHER << i)) {
					pr_debug("eth%d pci slot: irq from slot %d is %d\n", i, slot, TX4939_IRQ_ETHER(i));
					return TX4939_IRQ_ETHER(i);
				}
			}
		}
		return 0;
	}

	/* IRQ rotation */
	irq--;			/* 0-3 */
	bus = dev->bus;
	if (bus->parent != NULL) {
		/* PCI-PCI Bridge */
		while (bus != NULL && bus->number > 0) {
			irq = (irq + slot) % 4;
			slot = PCI_SLOT(bus->self->devfn);
			bus = bus->parent;
		}
	}
	irq = (irq + 33 - slot) % 4;	/* magic number to calculate IRQ on RBX4939 */
	irq++;			/* 1-4 */

	switch (irq) {
	case 1:
		irq = TX4939_IRQ_INTA;
		break;
	case 2:
		irq = TX4939_IRQ_INTB;
		break;
	case 3:
		irq = TX4939_IRQ_INTC;
		break;
	case 4:
		irq = TX4939_IRQ_INTD;
		break;
	}

	pr_debug("assigned irq %d\n", irq);

	return irq;
}

int __init pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	unsigned char irq = 0;

	irq = pci_get_irq(dev, pin);

	printk(KERN_INFO "PCI: 0x%02x:0x%02x(0x%02x,0x%02x) IRQ=%d\n",
	       dev->bus->number, dev->devfn, PCI_SLOT(dev->devfn),
	       PCI_FUNC(dev->devfn), irq);

	return irq;
}

/*
 * Do platform specific device initialization at pci_enable_device() time
 */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
