/*
 * arch/ppc/platforms/4xx/xilinx_mlxxx.h
 *
 * Include file that defines Xilinx ML300 and ML40x evaluation boards
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2006 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#ifdef __KERNEL__
#ifndef __ASM_XILINX_MLxxx_H__
#define __ASM_XILINX_MLxxx_H__

/*
 * ML300 has Xilinx Virtex-II Pro processor, ML40x has Xilinx Virtex-4 FX
 * processor.
 */

#include <platforms/4xx/virtex.h>

#ifndef __ASSEMBLY__

#include <linux/types.h>

typedef struct board_info {
	unsigned int	 bi_memsize;		/* DRAM installed, in bytes */
	unsigned char	 bi_enetaddr[6];	/* Local Ethernet MAC address */
	unsigned int	 bi_intfreq;		/* Processor speed, in Hz */
	unsigned int	 bi_busfreq;		/* PLB Bus speed, in Hz */
	unsigned int	 bi_pci_busfreq;	/* PCI Bus speed, in Hz */
} bd_t;

/* Some 4xx parts use a different timebase frequency from the internal clock.
*/
#define bi_tbfreq bi_intfreq

#endif /* !__ASSEMBLY__ */

/* We don't need anything mapped.  Size of zero will accomplish that. */
#define PPC4xx_ONB_IO_PADDR	0u
#define PPC4xx_ONB_IO_VADDR	0u
#define PPC4xx_ONB_IO_SIZE	0u

#if defined(CONFIG_XILINX_ML300)
#define PPC4xx_MACHINE_NAME "Xilinx ML300"
#define XILINX_SYS_ID_STR "Xilinx ML300 Reference System (Virtex-II Pro)\n"
#elif defined(CONFIG_XILINX_ML40x)
#define PPC4xx_MACHINE_NAME "Xilinx ML40x"
#define XILINX_SYS_ID_STR "Xilinx ML40x Reference System (Virtex-4 FX)\n"
#endif

#endif /* __ASM_XILINX_MLxxx_H__ */
#endif /* __KERNEL__ */
