/*
 * include/asm-xtensa/platform-xt2000/pci.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

/*
 * This file defines the aperture windows for the V3 device.
 */

#ifndef _XTENSA_XT2000_PCI_H
#define _XTENSA_XT2000_PCI_H

#include <asm/platform/xt2000.h>

#define PCI_V320_BASE	XTBOARD_V3PCI_VADDR

/* 
 * PCI -> Local Memory
 */

#define PCI_LOCAL_MEM_CPU_BASE	0
#define PCI_LOCAL_MEM_PCI_BASE	0
#define PCI_LOCAL_MEM_SIZE	XTBOARD_MEMORY_SIZE
#define PCI_LOCAL_MEM_MAP_REG	\
	(PCI_MEM_PCI_RD_MB_00 |	\
	PCI_MEM_PCI_WR_MB_00 |	\
	PCI_MEM_MAP_REG_EN |	\
	PCI_MEM_MAP_ENABLE)

	 /* PCI_MEM_BYTE_SWAP_8 | \ */
	/* XTFIXME: need something like XTBOARD_MEMORY_SIZE_LOG2 */

/*
 * CPU -> PCI Memory space
 */

#define PCI_MEM_SPACE_CPU_BASE	XTBOARD_PCI_MEM_VADDR
#define PCI_MEM_SPACE_PCI_BASE	XTBOARD_PCI_MEM_VADDR
#define PCI_MEM_SPACE_SIZE	XTBOARD_PCI_MEM_SIZE
#define PCI_MEM_SPACE_SWAP	0		// FIXME: do we need to bswap?
#define PCI_MEM_SPACE_BASE_REG_DEFAULT	\
	(PCI_MEM_SPACE_CPU_BASE | 	\
	(PCI_MEM_SPACE_PCI_BASE >> 8) | \
	LB_PCI_BASEX_SIZE_128MB |	\
	LB_PCI_BASEX_MEMORY | 		\
	PCI_MEM_SPACE_SWAP)


/*
 * CPU -> PCI IO space
 */

#define PCI_IO_SPACE_CPU_BASE	XTBOARD_PCI_IO_VADDR
#define PCI_IO_SPACE_PCI_BASE	0
#define PCI_IO_SPACE_SIZE	XTBOARD_PCI_IO_SIZE
#define PCI_IO_SPACE_SWAP	0		// FIXME: do we need to bswap?
#define PCI_IO_SPACE_BASE_REG_DEFAULT	\
	(PCI_IO_SPACE_CPU_BASE |	\
	(PCI_IO_SPACE_PCI_BASE >> 8) |	\
	LB_PCI_BASEX_IO |		\
	PCI_IO_SPACE_SWAP |		\
	LB_PCI_BASEX_SIZE_16MB |	\
	LB_PCI_BASEX_ERR_EN)

/*
 * CPU -> PCI Config space
 */

#define PCI_CFG_SPACE_CPU_BASE	XTBOARD_PCI_IO_VADDR
#define PCI_CFG_SPACE_PCI_BASE	0
#define PCI_CFG_SPACE_SIZE	XTBOARD_PCI_IO_SIZE
#define PCI_CFG_SPACE_SWAP	0		// FIXME: do we need to bswap?
#define PCI_CFG_SPACE_BASE_REG_DEFAULT	\
	(PCI_CFG_SPACE_CPU_BASE |	\
	PCI_CFG_SPACE_SWAP |		\
	LB_PCI_BASEX_CONFIG |		\
	LB_PCI_BASEX_SIZE_16MB)

//	LB_PCI_BASEX_ERR_EN

#endif /* _XTENSA_XT2000_H */
