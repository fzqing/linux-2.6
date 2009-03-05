/*
 * drivers/mtd/nand/intel_vr_nand.c
 *
 * A driver for NAND flash on the Intel Vermilion Range Expansion Bus.
 *
 * Author: Andy Lowe <alowe@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>

struct vr_nand_mtd {
	void __iomem *csr_base;
	struct mtd_info *info;
	int nr_parts;
	struct pci_dev *dev;
};

/* Expansion Bus Configuration and Status Registers are in BAR 0 */
#define EXP_CSR_MBAR 0
/* Expansion Bus Memory Window is BAR 1 */
#define EXP_WIN_MBAR 1
/* We only need to map one page for NAND */
#define NAND_SIZE 0x1000
/* Chip Select 0 is at offset 0 in the Memory Window */
#define CS0_START 0x0
/* Chip Select 0 Timing Register is at offset 0 in CSR */
#define EXP_TIMING_CS0 0x00
#define TIMING_CS_EN		(1 << 31)	/* Chip Select Enable */
#define TIMING_BOOT_ACCEL_DIS	(1 <<  8)	/* Boot Acceleration Disable */
#define TIMING_NAND_BOOT	(1 <<  7)	/* NAND Boot Enable */
#define TIMING_WR_EN		(1 <<  1)	/* Write Enable */
#define TIMING_MASK		0x3FFF0000
/* NAND Command register for Chip Select 0 is at offset 0x30 in CSR */
#define NAND_CMD_0	0x30
#define NAND_CMD_BSY		(1 << 31)
#define NAND_CMD_ADDR_MASK(m)	((m) << 12)
#define NAND_CMD_EN		(1 <<  8)
#define NAND_CMD_MASK		0xff
/* NAND Address register is at offset 0x50 in CSR */
#define NAND_ADDR	0x50
#define NAND_ADDR_FORCE		(1 << 31)

#define BYTE_LANE(n)		(1 << (n))
#define ALL_BYTE_LANES \
	(BYTE_LANE(0) | BYTE_LANE(1) | BYTE_LANE(2) | BYTE_LANE(3))

#define DRV_NAME	"vr_nand"

static void vr_nand_hwcontrol(struct mtd_info *info, int cmd)
{
}

static void vr_nand_send_command(struct vr_nand_mtd *p, unsigned cmd,
				 unsigned addr, unsigned byte_lane_disable)
{
	unsigned nand_cmd;

	writel(addr & ~NAND_ADDR_FORCE, p->csr_base + NAND_ADDR);

	nand_cmd = (cmd & NAND_CMD_MASK) | NAND_CMD_EN
	    | NAND_CMD_ADDR_MASK(byte_lane_disable);

	writel(nand_cmd, p->csr_base + NAND_CMD_0);
}

static void vr_nand_command(struct mtd_info *info, unsigned command,
			    int column, int page_addr)
{
	struct nand_chip *this = info->priv;
	struct vr_nand_mtd *p = this->priv;
	unsigned nand_addr = 0;
	unsigned byte_lane_disable = ALL_BYTE_LANES;

	/*
	 * Write out the command to the device.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= info->oobblock) {
			/* OOB area */
			column -= info->oobblock;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		vr_nand_send_command(p, readcmd, 0, ALL_BYTE_LANES);
	}

	if (column != -1 || page_addr != -1) {
		if (column != -1) {
			nand_addr |= (column & 0xff);
			byte_lane_disable &= ~BYTE_LANE(0);
		}
		if (page_addr != -1) {
			if (this->chipsize > (32 << 20)) {
				nand_addr |= ((page_addr & 0x3fffff) << 9);
				byte_lane_disable &= ~(BYTE_LANE(1)
						       | BYTE_LANE(2) |
						       BYTE_LANE(3));
			} else {
				nand_addr |= ((page_addr & 0xffff) << 9);
				byte_lane_disable &= ~(BYTE_LANE(1)
						       | BYTE_LANE(2));
			}
		}
	}

	vr_nand_send_command(p, command, nand_addr, byte_lane_disable);

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in need no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
	default:
		break;
	}
	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	/* wait until command is processed or timeout occurs */
	{
		unsigned long timeo = jiffies + 2;

		do {
			if (this->dev_ready(info))
				return;
		} while (time_before(jiffies, timeo));
	}
}

static int vr_nand_device_ready(struct mtd_info *info)
{
	struct nand_chip *this = info->priv;
	struct vr_nand_mtd *p = this->priv;

	return !(readl(p->csr_base + NAND_CMD_0) & NAND_CMD_BSY);
}

static void __devexit vr_nand_release(struct vr_nand_mtd *p)
{
	struct nand_chip *this = p->info->priv;
	unsigned int exp_timing_cs0;

	nand_release(p->info);

	exp_timing_cs0 = readl(p->csr_base + EXP_TIMING_CS0);
	exp_timing_cs0 &= ~TIMING_WR_EN;
	writel(exp_timing_cs0, p->csr_base + EXP_TIMING_CS0);

	iounmap(this->IO_ADDR_R);

	kfree(p->info);

	iounmap(p->csr_base);
}

static int __devinit vr_nand_init(struct vr_nand_mtd *p)
{
	unsigned long csr_phys, csr_len;
	unsigned long win_phys, win_len;
	unsigned int exp_timing_cs0;
	int err = 0;
	struct nand_chip *this;
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
	struct mtd_partition *parts;
	static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

	csr_phys = pci_resource_start(p->dev, EXP_CSR_MBAR);
	csr_len = pci_resource_len(p->dev, EXP_CSR_MBAR);
	win_phys = pci_resource_start(p->dev, EXP_WIN_MBAR);
	win_len = pci_resource_len(p->dev, EXP_WIN_MBAR);

	if (!csr_phys || !csr_len || !win_phys || !win_len)
		return -ENODEV;

	if (win_len < (CS0_START + NAND_SIZE))
		return -ENXIO;

	p->csr_base = ioremap_nocache(csr_phys, csr_len);
	if (!p->csr_base)
		return -ENOMEM;

	exp_timing_cs0 = readl(p->csr_base + EXP_TIMING_CS0);
	if (!(exp_timing_cs0 & TIMING_CS_EN)) {
		dev_warn(&p->dev->dev, "Expansion Bus Chip Select 0 "
		       "is disabled.\n");
		err = -ENODEV;
		goto release_csr;
	}
	if ((exp_timing_cs0 & TIMING_MASK) == TIMING_MASK) {
		dev_warn(&p->dev->dev, "Expansion Bus Chip Select 0 "
		       "is configured for maximally slow access times.\n");
	}

	/* Allocate memory for MTD device structure and private data */
	p->info = kzalloc(sizeof(struct mtd_info) +
			  sizeof(struct nand_chip), GFP_KERNEL);
	if (!p->info) {
		err = -ENOMEM;
		goto release_csr;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&p->info[1]);

	/* Link the private data with the MTD structure */
	p->info->priv = this;
	this->priv = p;

	/* initialize I/O function pointers */
	this->hwcontrol = vr_nand_hwcontrol;
	this->cmdfunc = vr_nand_command;
	this->dev_ready = vr_nand_device_ready;

	this->options = 0;	/* only 8-bit buswidth is supported */
	this->eccmode = NAND_ECC_SOFT;

	/* map physical address */
	this->IO_ADDR_R = ioremap_nocache(win_phys + CS0_START, NAND_SIZE);
	if (!this->IO_ADDR_R) {
		err = -ENOMEM;
		goto release_mtd;
	}
	this->IO_ADDR_W = this->IO_ADDR_R;

	/* Enable writes to flash bank */
	exp_timing_cs0 &= ~TIMING_NAND_BOOT;
	exp_timing_cs0 |= TIMING_BOOT_ACCEL_DIS | TIMING_WR_EN;
	writel(exp_timing_cs0, p->csr_base + EXP_TIMING_CS0);

	/* Scan to find the device and allocate data_buf and oob_buf */
	if (nand_scan(p->info, 1)) {
		err = -ENODEV;
		goto release_io;
	}

	/* register the flash */
	p->info->name = DRV_NAME;
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
	/* partition the flash bank */
	p->nr_parts = parse_mtd_partitions(p->info, part_probes, &parts, 0);
	if (p->nr_parts > 0)
		err = add_mtd_partitions(p->info, parts, p->nr_parts);
#endif
	if (p->nr_parts <= 0)
		err = add_mtd_device(p->info);
	if (err)
		goto release_nand;

	return 0;

      release_nand:
	nand_release(p->info);

      release_io:
	exp_timing_cs0 &= ~TIMING_WR_EN;
	writel(exp_timing_cs0, p->csr_base + EXP_TIMING_CS0);
	iounmap(this->IO_ADDR_R);

      release_mtd:
	kfree(p->info);

      release_csr:
	iounmap(p->csr_base);
	return err;
}

static struct pci_device_id vr_nand_pci_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x500D)},
	{0,}
};

static void __devexit vr_nand_pci_remove(struct pci_dev *dev)
{
	struct vr_nand_mtd *p = pci_get_drvdata(dev);

	pci_set_drvdata(dev, NULL);
	vr_nand_release(p);
	kfree(p);
	pci_release_regions(dev);
	pci_disable_device(dev);
}

static int __devinit
vr_nand_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct vr_nand_mtd *p = NULL;
	int err;
	int was_enabled;

	was_enabled = dev->is_enabled;
	if (!was_enabled) {
		err = pci_enable_device(dev);
		if (err)
			goto out;
	}

	err = pci_request_regions(dev, DRV_NAME);
	if (err)
		goto disable_dev;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	err = -ENOMEM;
	if (!p)
		goto release;

	p->dev = dev;

	err = vr_nand_init(p);
	if (err)
		goto release;

	pci_set_drvdata(dev, p);

	return 0;

      release:
	kfree(p);
	pci_release_regions(dev);

      disable_dev:
	if (!was_enabled)
		pci_disable_device(dev);

      out:
	return err;
}

static struct pci_driver vr_nand_pci_driver = {
	.name = DRV_NAME,
	.probe = vr_nand_pci_probe,
	.remove = __devexit_p(vr_nand_pci_remove),
	.id_table = vr_nand_pci_ids,
};

static int __init vr_nand_mtd_init(void)
{
	return pci_register_driver(&vr_nand_pci_driver);
}

static void __exit vr_nand_mtd_exit(void)
{
	pci_unregister_driver(&vr_nand_pci_driver);
}

module_init(vr_nand_mtd_init);
module_exit(vr_nand_mtd_exit);

MODULE_AUTHOR("Andy Lowe");
MODULE_DESCRIPTION("NAND controller driver for Intel Vermilion Range");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, vr_nand_pci_ids);
