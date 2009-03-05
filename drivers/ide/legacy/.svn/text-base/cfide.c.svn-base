/*
 * Compact Flash Memory Mapped True IDE mode driver
 *
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/cfide.h>
#include <asm/io.h>

static struct {
	void __iomem *cfide_mapbase;
	void __iomem *cfide_alt_mapbase;
	ide_hwif_t *hwif;
	int index;
} hwif_prop;

extern void cfide_platform_mmiops (ide_hwif_t *);
extern void cfide_selectproc (ide_drive_t *);

static ide_hwif_t * __devinit cfide_locate_hwif(void __iomem * base, void __iomem * ctrl,
				struct cfide_platform_data *pdata, int irq)
{
	unsigned long port = (unsigned long)base;
	ide_hwif_t *hwif;
	int index, i;

	for (index = 0; index < MAX_HWIFS; ++index) {
		hwif = ide_hwifs + index;
		if (hwif->io_ports[IDE_DATA_OFFSET] == port)
			goto found;
	}

	for (index = 0; index < MAX_HWIFS; ++index) {
		hwif = ide_hwifs + index;
		if (hwif->io_ports[IDE_DATA_OFFSET] == 0)
			goto found;
	}

	return NULL;

 found:

	hwif->hw.io_ports[IDE_DATA_OFFSET] = port;

	port += pdata->regaddr_step + pdata->byte_lanes_swapping;
	for (i = IDE_ERROR_OFFSET; i <= IDE_STATUS_OFFSET; i++, port += pdata->regaddr_step)
		hwif->hw.io_ports[i] = port;

	hwif->hw.io_ports[IDE_CONTROL_OFFSET] = (unsigned long)ctrl +
		6 * pdata->regaddr_step + pdata->byte_lanes_swapping;

	memcpy(hwif->io_ports, hwif->hw.io_ports, sizeof(hwif->hw.io_ports));
	hwif->hw.irq = hwif->irq = irq;
	hwif->selectproc = cfide_selectproc;

	hwif->hw.dma = NO_DMA;
	hwif->hw.chipset = ide_generic;

	hwif->mmio = 2;
	cfide_platform_mmiops(hwif);
	hwif_prop.hwif = hwif;
	hwif_prop.index = index;

	return hwif;
}

static int __devinit cfide_lbus_probe(struct device *dev_raw)
{
	struct platform_device *dev = container_of(dev_raw,
		struct platform_device, dev);
	struct resource *res_base, *res_alt, *res_irq;
	ide_hwif_t *hwif;
	struct cfide_platform_data *pdata;
	int ret = 0;

	pdata = (struct cfide_platform_data*)dev->dev.platform_data;

	/* get a pointer to the register memory */
	res_base = platform_get_resource(dev, IORESOURCE_MEM, 0);
	res_alt = platform_get_resource(dev, IORESOURCE_MEM, 1);
	res_irq = platform_get_resource(dev, IORESOURCE_IRQ, 0);

	if ((!res_base) || (!res_alt) || (!res_irq)) {
		ret = -ENODEV;
		goto out;
	}

	if (!request_mem_region
	    (res_base->start, res_base->end - res_base->start + 1, dev->name)) {
		dev_printk(KERN_DEBUG, dev_raw, "%s: request_mem_region of base failed\n", dev->name);
		ret = -EBUSY;
		goto out;
	}

	if (!request_mem_region
	    (res_alt->start, res_alt->end - res_alt->start + 1, dev->name)) {
		dev_printk(KERN_DEBUG, dev_raw, "%s: request_mem_region of alt failed\n", dev->name);
		ret = -EBUSY;
		goto release_base;
	}

	hwif_prop.cfide_mapbase =
	    ioremap(res_base->start, res_base->end - res_base->start + 1);
	if (!hwif_prop.cfide_mapbase) {
		ret = -ENOMEM;
		goto release_alt;
	}

	hwif_prop.cfide_alt_mapbase =
	    ioremap(res_alt->start, res_alt->end - res_alt->start + 1);

	if (!hwif_prop.cfide_alt_mapbase) {
		ret = -ENOMEM;
		goto unmap_base;
	}

	hwif = cfide_locate_hwif(hwif_prop.cfide_mapbase, hwif_prop.cfide_alt_mapbase,
			 pdata, res_irq->start);

	if (!hwif) {
		ret = -ENODEV;
		goto unmap_alt;
	}
	hwif->gendev.parent = &dev->dev;
	hwif->noprobe = 0;

	probe_hwif_init(hwif);

	dev_set_drvdata(&dev->dev, hwif);
	create_proc_ide_interfaces();

	return 0;

 unmap_alt:
	iounmap(hwif_prop.cfide_alt_mapbase);
 unmap_base:
	iounmap(hwif_prop.cfide_mapbase);
 release_alt:
	release_mem_region(res_alt->start, res_alt->end - res_alt->start + 1);
 release_base:
	release_mem_region(res_base->start, res_base->end - res_base->start + 1);
 out:
	return ret;
}

static int __devexit cfide_lbus_remove(struct device *dev_raw)
{
	struct platform_device *dev = container_of(dev_raw,
		struct platform_device, dev);

	ide_hwif_t *hwif = dev->dev.driver_data;
	struct resource *res_base, *res_alt;

	/* get a pointer to the register memory */
	res_base = platform_get_resource(dev, IORESOURCE_MEM, 0);
	res_alt = platform_get_resource(dev, IORESOURCE_MEM, 1);

	release_mem_region(res_base->start, res_base->end - res_base->start + 1);
	release_mem_region(res_alt->start, res_alt->end - res_alt->start + 1);

	dev_set_drvdata(&dev->dev, NULL);

	if (hwif != hwif_prop.hwif)
		dev_printk(KERN_DEBUG, dev_raw, "%s: hwif value error", dev->name);
	else {
		ide_unregister (hwif_prop.index);
		hwif_prop.index = 0;
		hwif_prop.hwif = NULL;
	}

	iounmap(hwif_prop.cfide_mapbase);
	iounmap(hwif_prop.cfide_alt_mapbase);

	return 0;
}

static struct device_driver cfide_driver = {
	.name = "mmio-cfide",
	.bus = &platform_bus_type,
	.probe = cfide_lbus_probe,
	.remove = __devexit_p(cfide_lbus_remove),
};

static int __init cfide_lbus_init(void)
{
	return driver_register(&cfide_driver);
}

static void __exit cfide_lbus_exit(void)
{
	driver_unregister(&cfide_driver);
}

MODULE_DESCRIPTION("MMIO based True IDE Mode Compact Flash Driver");
MODULE_LICENSE("GPL");

module_init(cfide_lbus_init);
module_exit(cfide_lbus_exit);

