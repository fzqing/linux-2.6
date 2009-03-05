/*
 * This file contains work-arounds for x86 and x86_64 platform bugs.
 */
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/irq.h>

#if defined(CONFIG_X86_IO_APIC) && defined(CONFIG_SMP) && defined(CONFIG_PCI)

void __devinit quirk_intel_irqbalance(struct pci_dev *dev)
{
	u8 config, rev;
	u32 word;

	/* BIOS may enable hardware IRQ balancing for
	 * E7520/E7320/E7525(revision ID 0x9 and below)
	 * based platforms.
	 * Disable SW irqbalance/affinity on those platforms.
	 */
	pci_read_config_byte(dev, PCI_CLASS_REVISION, &rev);
	if (rev > 0x9)
		return;

	printk(KERN_INFO "Intel E7520/7320/7525 detected.");

	/* enable access to config space*/
	pci_read_config_byte(dev, 0xf4, &config);
	config |= 0x2;
	pci_write_config_byte(dev, 0xf4, config);

	/* read xTPR register */
	raw_pci_ops->read(0, 0, 0x40, 0x4c, 2, &word);

	if (!(word & (1 << 13))) {
		printk(KERN_INFO "Disabling irq balancing and affinity\n");
#ifdef CONFIG_IRQBALANCE
		irqbalance_disable("");
#endif
		noirqdebug_setup("");
		no_irq_affinity = 1;
	}

	config &= ~0x2;
	/* disable access to config space*/
	pci_write_config_byte(dev, 0xf4, config);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_E7320_MCH,	quirk_intel_irqbalance);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_E7525_MCH,	quirk_intel_irqbalance);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_E7520_MCH,	quirk_intel_irqbalance);
#endif

#if defined(CONFIG_PCI_MSI)

extern int msi_trigger_mode;

/*
 * In the Intel Vermilion Range chipset, support for legacy interrupts and
 * Message-Signaled Interrupts is mutually exclusive.  The BIOS should always
 * enable legacy interrupts.  If MSI support is enabled in the kernel, then
 * the MSI interrupt controller must be enabled and legacy interrupts disabled.
 */
void __devinit quirk_intel_vr_msi(struct pci_dev *dev)
{
	u32 mchbar;
	void __iomem *base;
	u32 avroot_csr, avroot_csr_new;

	pci_read_config_dword(dev, 0x44, &mchbar);

	/* use bits 31:14, 16 kB aligned */
	base = ioremap_nocache(mchbar & 0xFFFFC000, 0x4000);
	if (!base)
		return;

	avroot_csr = avroot_csr_new = readl(base + 0xA00);

	if (avroot_csr_new & (1 << 11)) {
		printk(KERN_INFO "Disabling Vermilion Range legacy interrupt "
			"to ICH.\n");
		avroot_csr_new &= ~(1 << 11);
	}

	if ((avroot_csr_new & 0x3) != 0x3) {
		printk(KERN_INFO "Enabling Vermilion Range MSI state machine "
			"with round-robin priorities.\n");
		avroot_csr_new |= 0x3;
	}

	if (avroot_csr_new != avroot_csr)
		writel(avroot_csr_new, base + 0xA00);

	if (readl(base + 0xA0C) != 0xFFFFFFFF) {
		printk(KERN_INFO "Unmasking all Vermilion Range MSI root "
			"interrupt requests.\n");
		writel(0xFFFFFFFF, base + 0xA0C);
	}

	iounmap(base);

	/*
	 * The Vermilion Range root interrupt controller requires MSI trigger
	 * mode to be set (level-triggered).
	 */
	msi_trigger_mode = 1;
}

DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x5001, quirk_intel_vr_msi);
#endif
