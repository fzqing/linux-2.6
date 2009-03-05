/*
 * SuperH default IDE host driver
 *
 * Copyright (C) 2004  Paul Mundt
 *
 * Based on the old include/asm-sh/ide.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ide.h>

#include <asm/irq.h>

enum {
	IDE_SH_PRIMARY_IO = 0x1f0,
	IDE_SH_SECONDARY_IO = 0x170,
};

#if defined(CONFIG_CF_ENABLER)
#  if defined(CONFIG_SH_SOLUTION_ENGINE)
#    define IDE_SH_PRIMARY_IRQ	7
#    define IDE_SH_SECONDARY_IRQ	0
#  elif defined(CONFIG_SH_7751_SOLUTION_ENGINE)
#    define IDE_SH_PRIMARY_IRQ		14
#    define IDE_SH_SECONDARY_IRQ	15
#  elif defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
#    if defined(CONFIG_CPU_SUBTYPE_SH7727)
#      define IDE_SH_PRIMARY_IRQ	IRQ4_IRQ
#    elif defined(CONFIG_CPU_SUBTYPE_SH7751)
#      define IDE_SH_PRIMARY_IRQ	14
#    elif defined(CONFIG_CPU_SUBTYPE_SH7760)
#      define IDE_SH_PRIMARY_IRQ	10
#    endif
#    define IDE_SH_SECONDARY_IRQ	0
#  elif defined(CONFIG_SH_SOLUTION_ENGINE_LIGHT)
#    if defined(CONFIG_CPU_SUBTYPE_SH7727)
#      define IDE_SH_PRIMARY_IRQ	IRQ4_IRQ
#    else
#      define IDE_SH_PRIMARY_IRQ	7
#    endif
#    define IDE_SH_SECONDARY_IRQ	0
#  elif defined(CONFIG_SH_SOLUTION_ENGINE_PLUS)
#    define IDE_SH_PRIMARY_IRQ		14
#    define IDE_SH_SECONDARY_IRQ	13
#  else
#    define IDE_SH_PRIMARY_IRQ		14
#    define IDE_SH_SECONDARY_IRQ	15
#  endif
#else
#define IDE_SH_PRIMARY_IRQ	14
#define IDE_SH_SECONDARY_IRQ	15
#endif

struct sh_ide_hwif {
	unsigned long base;
	int irq;
} sh_ide_hwifs[] __initdata = {
	{
	IDE_SH_PRIMARY_IO, IDE_SH_PRIMARY_IRQ}, {
	IDE_SH_SECONDARY_IO, IDE_SH_SECONDARY_IRQ}, {
0,},};

static inline void __init hw_setup(hw_regs_t * hw, int idx)
{
	unsigned long base = sh_ide_hwifs[idx].base;

	if (!request_region(base, 8, "ide-sh"))
		return;
	if (!request_region(base + 0x206, 1, "ide-sh"))
		return;

	memset(hw, 0, sizeof(hw_regs_t));
	ide_std_init_ports(hw, base, base + 0x206);

	hw->irq = sh_ide_hwifs[idx].irq;
	hw->dma = NO_DMA;
	hw->chipset = ide_generic;
}

static inline void __init hwif_setup(ide_hwif_t * hwif)
{
	default_hwif_iops(hwif);

	hwif->mmio = 2;
}

void __init ide_sh_init(void)
{
	int i;

	printk(KERN_INFO "ide: SuperH generic IDE interface\n");

	for (i = 0; i < MAX_HWIFS; i++) {
		ide_hwif_t *hwif;
		hw_regs_t hw;
		int idx;

		if (!sh_ide_hwifs[i].base) {
			printk(KERN_ERR "ide-sh: Attempting to register ide%d "
			       "when only %d interfaces are available.\n",
			       i, i - 1);
			break;
		}

		hw_setup(&hw, i);

		idx = ide_register_hw(&hw, &hwif);
		if (idx == -1) {
			printk(KERN_ERR
			       "ide-sh: IDE interface registration failed\n");
			return;
		}

		hwif_setup(hwif);
	}
}
