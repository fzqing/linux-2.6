/*
 * arch/mips/vr5701/common/nec_vrxxxx.c
 *
 * A code for low-level routines on NEC Electronics Corporation VR5701 SolutionGearII 
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/vr5701/vr5701_sg2.h>

u32
ddb_calc_pdar(u32 phys, u32 size, int width, int on_memory_bus, int pci_visible)
{
	u32 maskbits;
	u32 widthbits;

	switch (size) {
	case 0x80000000:	/* 2 GB */
		maskbits = 5;
		break;
	case 0x40000000:	/* 1 GB */
		maskbits = 6;
		break;
	case 0x20000000:	/* 512 MB */
		maskbits = 7;
		break;
	case 0x10000000:	/* 256 MB */
		maskbits = 8;
		break;
	case 0x08000000:	/* 128 MB */
		maskbits = 9;
		break;
	case 0x04000000:	/* 64 MB */
		maskbits = 10;
		break;
	case 0x02000000:	/* 32 MB */
		maskbits = 11;
		break;
	case 0x01000000:	/* 16 MB */
		maskbits = 12;
		break;
	case 0x00800000:	/* 8 MB */
		maskbits = 13;
		break;
	case 0x00400000:	/* 4 MB */
		maskbits = 14;
		break;
	case 0x00200000:	/* 2 MB */
		maskbits = 15;
		break;
	case 0:		/* OFF */
		maskbits = 0;
		break;
	default:
		panic("VR5701_set_pdar: unsupported size %p", (void *)size);
	}
	switch (width) {
	case 8:
		widthbits = 0;
		break;
	case 16:
		widthbits = 1;
		break;
	case 32:
		widthbits = 2;
		break;
	case 64:
		widthbits = 3;
		break;
	default:
		panic("VR5701_set_pdar: unsupported width %d", width);
	}

	return maskbits | (on_memory_bus ? 0x10 : 0) |
	    (pci_visible ? 0x20 : 0) | (widthbits << 6) | (phys & 0xffe00000);
}

void
ddb_set_pdar(u32 pdar, u32 phys, u32 size, int width,
	     int on_memory_bus, int pci_visible)
{
	u32 temp = ddb_calc_pdar(phys, size, width, on_memory_bus, pci_visible);
	ddb_out32(pdar, temp);
	ddb_out32(pdar + 4, 0);
	ddb_in32(pdar);
	ddb_in32(pdar + 4);
}

/*
 * routines that mess with PCIINITx registers
 */

void ddb_set_pmr(u32 pmr, u32 type, u32 addr, u32 options)
{
	switch (type) {
	case DDB_PCICMD_IACK:	/* PCI Interrupt Acknowledge */
	case DDB_PCICMD_IO:	/* PCI I/O Space */
	case DDB_PCICMD_MEM:	/* PCI Memory Space */
	case DDB_PCICMD_CFG:	/* PCI Configuration Space */
		break;
	default:
		panic("VR5701_set_pmr: invalid type %d", type);
	}
	ddb_out32(pmr, (type << 1) | (addr & 0xffe00000) | options);
	ddb_out32(pmr + 4, 0);
}

void ddb_set_bar(u32 bar, u32 phys, int prefetchable)
{
	ddb_out32(bar, (phys & 0xffe00000) | (!!prefetchable << 3));
	ddb_out32(bar + 4, 0);
}
