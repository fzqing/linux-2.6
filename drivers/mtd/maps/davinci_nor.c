/*
 * Flash memory support for various TI DaVinci boards
 *
 * Copyright (C) 2006 Komal Shah <komal_shah802003@yahoo.com>
 *
 * Derived from OMAP NOR flash mapping driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach/flash.h>

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { /* "RedBoot", */ "cmdlinepart", NULL };
#endif

struct davinciflash_info {
	struct mtd_info		*mtd;
	struct map_info		map;
	struct resource         *res;
#ifdef CONFIG_MTD_PARTITIONS
        struct mtd_partition    *parts;
#endif

};

static int  davinciflash_probe(struct device *dev)
{
	int err;
	struct davinciflash_info *info;
	struct platform_device *pdev = to_platform_device(dev);
	struct flash_platform_data *flash = pdev->dev.platform_data;
	struct resource *res = pdev->resource;
	unsigned long size = res->end - res->start + 1;

	info = kzalloc(sizeof(struct davinciflash_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, info);

	info->res = request_mem_region(res->start, size, "flash");
	if (info->res == NULL) {
		err = -EBUSY;
		goto out_free_info;
	}

	info->map.virt = ioremap(res->start, size);
	if (!info->map.virt) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}
	info->map.name		= pdev->dev.bus_id;
	info->map.phys		= res->start;
	info->map.size		= size;
	info->map.bankwidth	= flash->width;

	simple_map_init(&info->map);
	info->mtd = do_map_probe(flash->map_name, &info->map);
	if (!info->mtd) {
		err = -EIO;
		goto out_iounmap;
	}
	/* fixup mtd size to what is defined in plat struct
 	* do_map_probe overrides this in cfi_amdstd_setup()
 	* with what is actual size is, not what we define it
 	* as
 	*/
	info->mtd->size = info->map.size;
	info->mtd->owner = THIS_MODULE;

	/*
	 * Intel P30 flash is locked by default after power-on, so we need
	 * to unlock the entire flash.
	 */
	if (info->mtd->unlock)
		info->mtd->unlock(info->mtd, 0, info->mtd->size);

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(info->mtd, info->parts, err);
	else if (flash->nr_parts)
		add_mtd_partitions(info->mtd, flash->parts, flash->nr_parts);
	else
#endif
	{
		printk(KERN_NOTICE "TI Davinci flash: no partition info "
			"available, registering whole flash\n");
		add_mtd_device(info->mtd);
	}

	return 0;

out_iounmap:
	iounmap(info->map.virt);
out_release_mem_region:
	release_mem_region(res->start, size);
out_free_info:
	kfree(info);

	return err;
}

static int  davinciflash_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct davinciflash_info *info = dev_get_drvdata(&pdev->dev);
  	struct flash_platform_data *flash = pdev->dev.platform_data;

	if (info == NULL)
		return 0;

	dev_set_drvdata(&pdev->dev, NULL);

	if (info) {
#ifdef CONFIG_MTD_PARTITIONS
		if (info->parts) {
			del_mtd_partitions(info->mtd);
			kfree(info->parts);
		} else if (flash->nr_parts > 0)  {
			del_mtd_partitions(info->mtd);
		} else
#endif
		{
			del_mtd_device(info->mtd);
		}

		map_destroy(info->mtd);
	}

	if (info->map.virt != NULL)
		iounmap(info->map.virt);

	if (info->res != NULL) {
		release_resource(info->res);
                kfree(info->res);
	}

	return 0;
}

static struct device_driver davinciflash_driver = {
	.probe	= &davinciflash_probe,
	.remove	= &davinciflash_remove,
	.name	= "davinciflash",
	.bus = &platform_bus_type,
};

static int __init davinciflash_init(void)
{
	return driver_register(&davinciflash_driver);
}

static void __exit davinciflash_exit(void)
{
	driver_unregister(&davinciflash_driver);
}

module_init(davinciflash_init);
module_exit(davinciflash_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MTD NOR map driver for TI DaVinci boards");

