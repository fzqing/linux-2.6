/*
 *	Low-Level PCI Support for the SH7780
 *
 *  Copyright (C)  Takashi Kusuda  (Nov. 24, 2004)
 *
 *  Dustin McIntire (dustin@sensoria.com)
 *	Derived from arch/i386/kernel/pci-*.c which bore the message:
 *	(c) 1999--2000 Martin Mares <mj@ucw.cz>
 *
 *  Ported to the new API by Paul Mundt <lethal@linux-sh.org>
 *  With cleanup by Paul van Gool <pvangool@mimotech.com>
 *
 *  May be copied or modified under the terms of the GNU General Public
 *  License.  See linux/COPYING for more information.
 *
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/delay.h>

#include <asm/machvec.h>
#include <asm/io.h>
#include <asm/cpu/pci-sh7780.h>

static unsigned int pci_probe = PCI_PROBE_CONF1;
extern int pci_fixup_pcic(void);

/*
 * Direct access to PCI hardware...
 */

#define CONFIG_CMD(bus, devfn, where) (0x80000000 | (bus->number << 16) | (devfn << 8) | (where & ~3))

/*
 * Functions for accessing PCI configuration space with type 1 accesses
 */
static int sh7780_pci_read(struct pci_bus *bus, unsigned int devfn,
			   int where, int size, u32 *val)
{
	unsigned long flags;
	u32 data;

	/*
	 * PCIPDR may only be accessed as 32 bit words,
	 * so we must do byte alignment by hand
	 */
	local_irq_save(flags);
	outl(CONFIG_CMD(bus,devfn,where), PCI_REG(SH7780_PCI_PAR));
	data = inl(PCI_REG(SH7780_PCI_PDR));
	local_irq_restore(flags);

	switch (size) {
	case 1:
		*val = (data >> ((where & 3) << 3)) & 0xff;
		break;
	case 2:
		*val = (data >> ((where & 2) << 3)) & 0xffff;
		break;
	case 4:
		*val = data;
		break;
	default:
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}

	return PCIBIOS_SUCCESSFUL;
}

/*
 * Since SH7780 only does 32bit access we'll have to do a read,
 * mask,write operation.
 * We'll allow an odd byte offset, though it should be illegal.
 */
static int sh7780_pci_write(struct pci_bus *bus, unsigned int devfn,
			    int where, int size, u32 val)
{
	unsigned long flags;
	int shift;
	u32 data;

	local_irq_save(flags);
	outl(CONFIG_CMD(bus,devfn,where), PCI_REG(SH7780_PCI_PAR));
	data = inl(PCI_REG(SH7780_PCI_PDR));
	local_irq_restore(flags);

	switch (size) {
	case 1:
		shift = (where & 3) << 3;
		data &= ~(0xff << shift);
		data |= ((val & 0xff) << shift);
		break;
	case 2:
		shift = (where & 2) << 3;
		data &= ~(0xffff << shift);
		data |= ((val & 0xffff) << shift);
		break;
	case 4:
		data = val;
		break;
	default:
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}

	outl(data, PCI_REG(SH7780_PCI_PDR));

	return PCIBIOS_SUCCESSFUL;
}

#undef CONFIG_CMD

struct pci_ops sh7780_pci_ops = {
	.read 		= sh7780_pci_read,
	.write		= sh7780_pci_write,
};

static int __init pci_check_direct(void)
{
	unsigned int tmp, id;

	/* check for SH7780 hardware */
        id = ctrl_inl(PCI_REG(SH7780_PCI_VID));
        if ((id != ((SH7780_DEVICE_ID << 16) | SH7780_VENDOR_ID)) &&
	    (id != ((SH7781_DEVICE_ID << 16) | SH7780_VENDOR_ID))){
                PCIDBG(2,"PCI: This is not an SH7780/SH7781\n");
                return -ENODEV;
        }

	/*
	 * Check if configuration works.
	 */
	if (pci_probe & PCI_PROBE_CONF1) {
		tmp = inl (PCI_REG(SH7780_PCI_PAR));
		outl (0x80000000, PCI_REG(SH7780_PCI_PAR));
		if (inl (PCI_REG(SH7780_PCI_PAR)) == 0x80000000) {
			outl (tmp, PCI_REG(SH7780_PCI_PAR));
			printk(KERN_INFO "PCI: Using configuration type 1\n");
			request_region(PCI_REG(SH7780_PCI_PAR), 8, "PCI conf1");
			return 0;
		}
		outl (tmp, PCI_REG(SH7780_PCI_PAR));
	}

	pr_debug("PCI: pci_check_direct failed\n");
	return -EINVAL;
}

/***************************************************************************************/

/*
 *  Handle bus scanning and fixups ....
 */

static void __init pci_fixup_ide_bases(struct pci_dev *d)
{
	int i;

	/*
	 * PCI IDE controllers use non-standard I/O port decoding, respect it.
	 */
	if ((d->class >> 8) != PCI_CLASS_STORAGE_IDE)
		return;
	pr_debug("PCI: IDE base address fixup for %s\n", d->slot_name);
	for(i=0; i<4; i++) {
		struct resource *r = &d->resource[i];
		if ((r->start & ~0x80) == 0x374) {
			r->start |= 2;
			r->end = r->start;
		}
	}
}

/* Add future fixups here... */
DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID, pci_fixup_ide_bases);

/*
 *  Called after each bus is probed, but before its children
 *  are examined.
 */

void __init pcibios_fixup_bus(struct pci_bus *b)
{
	pci_read_bridge_bases(b);
}

/*
 * Initialization. Try all known PCI access methods. Note that we support
 * using both PCI BIOS and direct access: in such cases, we use I/O ports
 * to access config space.
 *
 * Note that the platform specific initialization (BSC registers, and memory
 * space mapping) will be called via the machine vectors (sh_mv.mv_pci_init()) if it
 * exitst and via the platform defined function pcibios_init_platform().
 * See pci_bigsur.c for implementation;
 *
 * The BIOS version of the pci functions is not yet implemented but it is left
 * in for completeness.  Currently an error will be genereated at compile time.
 */

static int __init sh7780_pci_init(void)
{
	int ret;

	pr_debug("PCI: Starting intialization.\n");
	if ((ret = pci_check_direct()) != 0)
		return ret;

	return 1;
}

subsys_initcall(sh7780_pci_init);

int __init sh7780_pcic_init(struct sh7780_pci_address_map *map)
{
	/* SH7780 PCIC enable */
	ctrl_outl (0x00000001, SH7780_PCI_ECR);
	return 1;
}

char * __init pcibios_setup(char *str)
{
	if (!strcmp(str, "off")) {
		pci_probe = 0;
		return NULL;
	}

	return str;
}

/*
 * 	IRQ functions
 */
static u8 __init sh7780_no_swizzle(struct pci_dev *dev, u8 *pin)
{
	/* no swizzling */
	return PCI_SLOT(dev->devfn);
}

static int sh7780_pci_lookup_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq = -1;

	/* now lookup the actual IRQ on a platform specific basis (pci-'platform'.c) */
	irq = pcibios_map_platform_irq(slot,pin);
	if( irq < 0 ) {
		pr_debug("PCI: Error mapping IRQ on device %s\n", dev->slot_name);
		return irq;
	}

	pr_debug("Setting IRQ for slot %s to %d\n", dev->slot_name, irq);

	return irq;
}

void __init pcibios_fixup_irqs(void)
{
	pci_fixup_irqs(sh7780_no_swizzle, sh7780_pci_lookup_irq);
}

