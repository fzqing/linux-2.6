/*
 * drivers/ide/pci/nec_vr5701_sg2.c
 *
 * NEC Electronics Corporation VR5701 SolutionGearII IDE controller driver
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/ide.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/pci.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <asm/io.h>

static unsigned int __init init_chipset_nec_vr5701(struct pci_dev *dev,
						   const char *name)
{
	return 0;
}

static u8 nec_vr5701_ratemask(ide_drive_t * drive)
{
	u8 mode = 2;
	if (!eighty_ninty_three(drive))
		mode = min(mode, (u8)1);
	return mode;
}

static void udma_set(ide_drive_t * drive, u16 udma_mode)
{

	ide_hwif_t * hwif = HWIF(drive);
	struct pci_dev * dev = hwif->pci_dev;

	u8 reg4b; /* udma control register */
	u16 reg4c; /* udma timing register */

	pci_read_config_word(dev, 0x4c, &reg4c);
	reg4c &= ~(7 << (drive->dn * 4));
	reg4c |= udma_mode << (drive->dn * 4);
	pci_write_config_word(dev, 0x4c, reg4c);

	pci_read_config_byte(dev, 0x4b, &reg4b);
	reg4b |= 1 << drive->dn;
	pci_write_config_byte(dev, 0x4b, reg4b);
}

static void udma_clean(ide_drive_t * drive)
{

	ide_hwif_t * hwif = HWIF(drive);
	struct pci_dev * dev = hwif->pci_dev;

	u8 reg4b; /* udma control register */

	pci_read_config_byte(dev, 0x4b, &reg4b);
	reg4b &= ~(1 << drive->dn);
	pci_write_config_byte(dev, 0x4b, reg4b);
}

static void dma_set(ide_drive_t * drive, u32 dma_mode)
{

	ide_hwif_t * hwif = HWIF(drive);
	struct pci_dev * dev = hwif->pci_dev;

	u32 reg44; /* dma timing register */

	udma_clean(drive);

	pci_read_config_dword(dev, 0x44, &reg44);
	reg44 &= ~(3 << (8 * drive->dn));
	reg44 |= dma_mode << (8 * drive->dn);
	pci_write_config_dword(dev, 0x44, reg44);
}

static int nec_vr5701_tune_chipset (ide_drive_t *drive, u8 xferspeed)
{
	u8 speed = ide_rate_filter(nec_vr5701_ratemask(drive), xferspeed);

	switch(speed) {
		case XFER_UDMA_4:
			udma_set(drive, 4);
			break;
		case XFER_UDMA_3:
			udma_set(drive, 3);
			break;
		case XFER_UDMA_2:
			udma_set(drive, 2);
			break;
		case XFER_UDMA_1:
			udma_set(drive, 1);
			break;
		case XFER_UDMA_0:
			udma_set(drive, 0);
			break;
		case XFER_MW_DMA_2:
			dma_set(drive, 2);
			break;
		case XFER_MW_DMA_1:
			dma_set(drive, 1);
			break;
		case XFER_MW_DMA_0:
			dma_set(drive, 0);
			break;
		case XFER_PIO_4:
		case XFER_PIO_3:
		case XFER_PIO_2:
		case XFER_PIO_1:
		case XFER_PIO_0:
			udma_clean(drive);
			break;
		default:
			return -1;
	}

	return (ide_config_drive_speed(drive, speed));
}

static void __init init_hwif_nec_vr5701(ide_hwif_t * hwif)
{
	if (!(hwif->dma_base))
		return;

	hwif->atapi_dma = 1;
	hwif->ultra_mask = 0x7f;
	hwif->mwdma_mask = 0x07;
	hwif->swdma_mask = 0x07;

	hwif->speedproc = &nec_vr5701_tune_chipset;

	{
		struct pci_dev * pci_dev = hwif->pci_dev;
		hwif->io_ports[IDE_CONTROL_OFFSET]
			= hwif->hw.io_ports[IDE_CONTROL_OFFSET]
			= pci_resource_start(pci_dev, 1) + 0x06;
	}

	if (!noautodma)
		hwif->autodma = 1;
	hwif->drives[0].autodma = hwif->autodma;
	hwif->drives[1].autodma = hwif->autodma;
}

static ide_pci_device_t nec_vr5701_chipset __devinitdata = {
	.name = "NEC Electronics Corporation VR5701 SolutionGearII",
	.init_chipset = init_chipset_nec_vr5701,
	.init_hwif = init_hwif_nec_vr5701,
	.channels = 2,
	.autodma = AUTODMA,
	.bootable = ON_BOARD,
};

static int __devinit nec_vr5701_init_one(struct pci_dev *dev,
					 const struct pci_device_id *id)
{
	ide_pci_device_t *d = &nec_vr5701_chipset;
	u16 command;

	pci_enable_device(dev);

	outb(6, pci_resource_start(dev, 5));

	pci_read_config_word(dev, PCI_COMMAND, &command);
	if (!(command & PCI_COMMAND_IO)) {
		printk(KERN_INFO "Skipping disabled %s IDE controller.\n",
		       d->name);
		return 1;
	}
	ide_setup_pci_device(dev, d);
	pci_write_config_byte(dev, 0x52, 0x00); /* workaround - Restriction no.2 */
	return 0;
}

static struct pci_device_id nec_vr5701_pci_tbl[] = {
	{PCI_VENDOR_ID_NEC, PCI_DEVICE_ID_NEC_USB_AND_IDE, PCI_ANY_ID,
	 PCI_ANY_ID, 0x010185, 0xffffff, 0},
	{0,},
};

MODULE_DEVICE_TABLE(pci, nec_vr5701_pci_tbl);

static struct pci_driver driver = {
	.name = "nec_vr5701_IDE",
	.id_table = nec_vr5701_pci_tbl,
	.probe = nec_vr5701_init_one,
};

static int nec_vr5701_ide_init(void)
{
	return ide_pci_register_driver(&driver);
}

module_init(nec_vr5701_ide_init);

MODULE_AUTHOR("Sergey Podstavin");
MODULE_DESCRIPTION("PCI driver module for NEC Electronics Corporation VR5701 SolutionGearII IDE");
MODULE_LICENSE("GPL");
