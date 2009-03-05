/*
 * ichxrom.c
 *
 * Normal mappings of chips in physical memory
 * $Id: ichxrom.c,v 1.17 2005/03/18 14:04:35 gleixner Exp $
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/cfi.h>
#include <linux/mtd/flashchip.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/list.h>

#define xstr(s) str(s)
#define str(s) #s
#define MOD_NAME xstr(KBUILD_BASENAME)

#define ADDRESS_NAME_LEN 18

#define ROM_PROBE_STEP_SIZE (64*1024) /* 64KiB */

/*
 * Define a structure for offsets to the LPC registers related to the
 * Firmware Hub.  These registers are at different offsets in different
 * ICH revisions.
 */
struct lpc_fwh_regs {
	unsigned bios_cntl;
	unsigned fwh_dec_en1;
	unsigned fwh_dec_en2;
	unsigned fwh_sel1;
	unsigned fwh_sel2;
};

/*
 * These offsets apply to the ICH2, ICH3, ICH4, ICH5 and 6300ESB
 * I/O Controller Hubs.
 */
static const struct lpc_fwh_regs ich2_fwh_regs = {
	.bios_cntl	= 0x4E,	/*  8-bit register */
	.fwh_dec_en1	= 0xE3,	/*  8-bit register */
	.fwh_dec_en2	= 0xF0,	/*  8-bit register */
	.fwh_sel1	= 0xE8,	/* 32-bit register */
	.fwh_sel2	= 0xEE,	/* 16-bit register */
};

/* These offsets apply to the ICH6 and ICH7 I/O Controller Hubs. */
static const struct lpc_fwh_regs ich6_fwh_regs = {
	.bios_cntl	= 0xDC,	/*  8-bit register */
	.fwh_dec_en1	= 0xD9, /*  8-bit register */
	.fwh_dec_en2	= 0xD8, /*  8-bit register */
	.fwh_sel1	= 0xD0, /* 32-bit register */
	.fwh_sel2	= 0xD4, /* 16-bit register */
};

struct ichxrom_window {
	void __iomem* virt;
	unsigned long phys;
	unsigned long size;
	struct list_head maps;
	struct resource rsrc;
	struct pci_dev *pdev;
	const struct lpc_fwh_regs *regs;
};

struct ichxrom_map_info {
	struct list_head list;
	struct map_info map;
	struct mtd_info *mtd;
	struct resource rsrc;
	char map_name[sizeof(MOD_NAME) + 2 + ADDRESS_NAME_LEN];
};

static struct ichxrom_window ichxrom_window = {
	.maps = LIST_HEAD_INIT(ichxrom_window.maps),
};

static void ichxrom_cleanup(struct ichxrom_window *window)
{
	struct ichxrom_map_info *map, *scratch;
	const struct lpc_fwh_regs *r;
	u8 byte;

	/* retrieve the register offsets for this ICH variant */
	r = window->regs;

	/* Disable writes through the rom window */
	pci_read_config_byte(window->pdev, r->bios_cntl, &byte);
	pci_write_config_byte(window->pdev, r->bios_cntl, byte & ~1);

	/* Free all of the mtd devices */
	list_for_each_entry_safe(map, scratch, &window->maps, list) {
		if (map->rsrc.parent)
			release_resource(&map->rsrc);
		del_mtd_device(map->mtd);
		map_destroy(map->mtd);
		list_del(&map->list);
		kfree(map);
	}
	if (window->rsrc.parent)
		release_resource(&window->rsrc);
	if (window->virt) {
		iounmap(window->virt);
		window->virt = NULL;
		window->phys = 0;
		window->size = 0;
		window->pdev = NULL;
	}
}


static int __devinit ichxrom_init_one (struct pci_dev *pdev,
	const struct pci_device_id *ent)
{
	static char *rom_probe_types[] = { "cfi_probe", "jedec_probe", NULL };
	struct ichxrom_window *window = &ichxrom_window;
	struct ichxrom_map_info *map = NULL;
	const struct lpc_fwh_regs *r;
	unsigned long map_top;
	u8 byte;

	/* retrieve the register offsets for this ICH variant */
	r = (const struct lpc_fwh_regs *) ent->driver_data;

	/* For now I just handle the ichx and I assume there
	 * are not a lot of resources up at the top of the address
	 * space.  It is possible to handle other devices in the
	 * top 16MB but it is very painful.  Also since
	 * you can only really attach a FWH to an ICHX there
	 * a number of simplifications you can make.
	 *
	 * Also you can page firmware hubs if an 8MB window isn't enough 
	 * but don't currently handle that case either.
	 */
	window->pdev = pdev;
	window->regs = (const struct lpc_fwh_regs *)ent->driver_data;

	/* Find a region continuous to the end of the ROM window  */
	window->phys = 0;
	pci_read_config_byte(pdev, r->fwh_dec_en1, &byte);
	if (byte == 0xff) {
		window->phys = 0xffc00000;
		pci_read_config_byte(pdev, r->fwh_dec_en2, &byte);
		if ((byte & 0x0f) == 0x0f) {
			window->phys = 0xff400000;
		}
		else if ((byte & 0x0e) == 0x0e) {
			window->phys = 0xff500000;
		}
		else if ((byte & 0x0c) == 0x0c) {
			window->phys = 0xff600000;
		}
		else if ((byte & 0x08) == 0x08) {
			window->phys = 0xff700000;
		}
	}
	else if ((byte & 0xfe) == 0xfe) {
		window->phys = 0xffc80000;
	}
	else if ((byte & 0xfc) == 0xfc) {
		window->phys = 0xffd00000;
	}
	else if ((byte & 0xf8) == 0xf8) {
		window->phys = 0xffd80000;
	}
	else if ((byte & 0xf0) == 0xf0) {
		window->phys = 0xffe00000;
	}
	else if ((byte & 0xe0) == 0xe0) {
		window->phys = 0xffe80000;
	}
	else if ((byte & 0xc0) == 0xc0) {
		window->phys = 0xfff00000;
	}
	else if ((byte & 0x80) == 0x80) {
		window->phys = 0xfff80000; 
	}

	if (window->phys == 0) {
		printk(KERN_ERR MOD_NAME ": Rom window is closed\n");
		goto out;
	}
	window->phys -= 0x400000UL;
	window->size = (0xffffffffUL - window->phys) + 1UL;

	/* Enable writes through the rom window */
	pci_read_config_byte(pdev, r->bios_cntl, &byte);
	if (!(byte & 1)  && (byte & (1<<1))) {
		/* The BIOS will generate an error if I enable
		 * this device, so don't even try.
		 */
		printk(KERN_ERR MOD_NAME ": firmware access control, I can't enable writes\n");
		goto out;
	}
	pci_write_config_byte(pdev, r->bios_cntl, byte | 1);

	/*
	 * Try to reserve the window mem region.  If this fails then
	 * it is likely due to the window being "reseved" by the BIOS.
	 */
	window->rsrc.name = MOD_NAME;
	window->rsrc.start = window->phys;
	window->rsrc.end   = window->phys + window->size - 1;
	window->rsrc.flags = IORESOURCE_MEM | IORESOURCE_BUSY;
	if (request_resource(&iomem_resource, &window->rsrc)) {
		window->rsrc.parent = NULL;
		printk(KERN_DEBUG MOD_NAME
			": %s(): Unable to register resource"
			" 0x%.08lx-0x%.08lx - kernel bug?\n",
			__func__,
			window->rsrc.start, window->rsrc.end);
	}

	/* Map the firmware hub into my address space. */
	window->virt = ioremap_nocache(window->phys, window->size);
	if (!window->virt) {
		printk(KERN_ERR MOD_NAME ": ioremap(%08lx, %08lx) failed\n",
			window->phys, window->size);
		goto out;
	}

	/* Get the first address to look for an rom chip at */
	map_top = window->phys;
	if ((window->phys & 0x3fffff) != 0) {
		map_top = window->phys + 0x400000;
	}
#if 1
	/* The probe sequence run over the firmware hub lock
	 * registers sets them to 0x7 (no access).
	 * Probe at most the last 4M of the address space.
	 */
	if (map_top < 0xffc00000) {
		map_top = 0xffc00000;
	}
#endif
	/* Loop through and look for rom chips */
	while((map_top - 1) < 0xffffffffUL) {
		struct cfi_private *cfi;
		unsigned long offset;
		int i;

		if (!map) {
			map = kmalloc(sizeof(*map), GFP_KERNEL);
		}
		if (!map) {
			printk(KERN_ERR MOD_NAME ": kmalloc failed");
			goto out;
		}
		memset(map, 0, sizeof(*map));
		INIT_LIST_HEAD(&map->list);
		map->map.name = map->map_name;
		map->map.phys = map_top;
		offset = map_top - window->phys;
		map->map.virt = (void __iomem *)
			(((unsigned long)(window->virt)) + offset);
		map->map.size = 0xffffffffUL - map_top + 1UL;
		/* Set the name of the map to the address I am trying */
		sprintf(map->map_name, "%s @%08lx",
			MOD_NAME, map->map.phys);

		/* Firmware hubs only use vpp when being programmed
		 * in a factory setting.  So in-place programming
		 * needs to use a different method.
		 */
		for(map->map.bankwidth = 32; map->map.bankwidth; 
			map->map.bankwidth >>= 1)
		{
			char **probe_type;
			/* Skip bankwidths that are not supported */
			if (!map_bankwidth_supported(map->map.bankwidth))
				continue;

			/* Setup the map methods */
			simple_map_init(&map->map);

			/* Try all of the probe methods */
			probe_type = rom_probe_types;
			for(; *probe_type; probe_type++) {
				map->mtd = do_map_probe(*probe_type, &map->map);
				if (map->mtd)
					goto found;
			}
		}
		map_top += ROM_PROBE_STEP_SIZE;
		continue;
	found:
		/* Trim the size if we are larger than the map */
		if (map->mtd->size > map->map.size) {
			printk(KERN_WARNING MOD_NAME
				" rom(%u) larger than window(%lu). fixing...\n",
				map->mtd->size, map->map.size);
			map->mtd->size = map->map.size;
		}
		if (window->rsrc.parent) {
			/*
			 * Registering the MTD device in iomem may not be possible
			 * if there is a BIOS "reserved" and BUSY range.  If this
			 * fails then continue anyway.
			 */
			map->rsrc.name  = map->map_name;
			map->rsrc.start = map->map.phys;
			map->rsrc.end   = map->map.phys + map->mtd->size - 1;
			map->rsrc.flags = IORESOURCE_MEM | IORESOURCE_BUSY;
			if (request_resource(&window->rsrc, &map->rsrc)) {
				printk(KERN_ERR MOD_NAME
					": cannot reserve MTD resource\n");
				map->rsrc.parent = NULL;
			}
		}

		/* Make the whole region visible in the map */
		map->map.virt = window->virt;
		map->map.phys = window->phys;
		cfi = map->map.fldrv_priv;
		for(i = 0; i < cfi->numchips; i++) {
			cfi->chips[i].start += offset;
		}
		
		/* Now that the mtd devices is complete claim and export it */
		map->mtd->owner = THIS_MODULE;
		if (add_mtd_device(map->mtd)) {
			map_destroy(map->mtd);
			map->mtd = NULL;
			goto out;
		}


		/* Calculate the new value of map_top */
		map_top += map->mtd->size;

		/* File away the map structure */
		list_add(&map->list, &window->maps);
		map = NULL;
	}

 out:
	/* Free any left over map structures */
	if (map) {
		kfree(map);
	}
	/* See if I have any map structures */
	if (list_empty(&window->maps)) {
		ichxrom_cleanup(window);
		return -ENODEV;
	}
	return 0;
}


static void __devexit ichxrom_remove_one (struct pci_dev *pdev)
{
	struct ichxrom_window *window = &ichxrom_window;
	ichxrom_cleanup(window);
}

#define INTEL_FWH_DEV(dev, data)	\
	.vendor = PCI_VENDOR_ID_INTEL,	\
	.device = (dev),		\
	.subvendor = PCI_ANY_ID,	\
	.subdevice = PCI_ANY_ID,	\
	.driver_data = (unsigned long) (data)

static struct pci_device_id ichxrom_pci_tbl[] __devinitdata = {
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_82801BA_0, &ich2_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_82801CA_0, &ich2_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_82801DB_0, &ich2_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_82801EB_0, &ich2_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ESB_1, &ich2_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH6_0, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH6_1, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH7_0, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH7_1, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH7_31, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH8_0, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH8_2, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ICH8_3, &ich6_fwh_regs) },
	{ INTEL_FWH_DEV(PCI_DEVICE_ID_INTEL_ESB2_0, &ich6_fwh_regs) },
	{ .vendor = 0, },
};

#if 0
MODULE_DEVICE_TABLE(pci, ichxrom_pci_tbl);

static struct pci_driver ichxrom_driver = {
	.name =		MOD_NAME,
	.id_table =	ichxrom_pci_tbl,
	.probe =	ichxrom_init_one,
	.remove =	ichxrom_remove_one,
};
#endif

static int __init init_ichxrom(void)
{
	struct pci_dev *pdev;
	struct pci_device_id *id;

	pdev = NULL;
	for (id = ichxrom_pci_tbl; id->vendor; id++) {
		pdev = pci_find_device(id->vendor, id->device, NULL);
		if (pdev) {
			break;
		}
	}
	if (pdev) {
		return ichxrom_init_one(pdev, id);
	}
	return -ENXIO;
#if 0
	return pci_register_driver(&ichxrom_driver);
#endif
}

static void __exit cleanup_ichxrom(void)
{
	ichxrom_remove_one(ichxrom_window.pdev);
}

module_init(init_ichxrom);
module_exit(cleanup_ichxrom);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eric Biederman <ebiederman@lnxi.com>");
MODULE_DESCRIPTION("MTD map driver for BIOS chips on the ICHX southbridge");
