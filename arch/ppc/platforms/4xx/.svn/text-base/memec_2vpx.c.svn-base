/*
 * arch/ppc/platforms/4xx/memec_2vpx.c
 *
 * Memec Virtex-II Pro 2VP7/2VP4 development board initialization
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2006 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serialP.h>
#include <linux/kgdb.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ocp.h>

#ifdef CONFIG_MTD
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#endif

#include <platforms/4xx/virtex.h>	/* for NR_SER_PORTS */

#ifdef CONFIG_MTD

#define BOARD_PHYS_ADDR		XPAR_FLASH_2MX32_MEM0_BASEADDR
#define BOARD_FLASH_SIZE \
	(XPAR_FLASH_2MX32_MEM0_HIGHADDR - XPAR_FLASH_2MX32_MEM0_BASEADDR + 1)
#define BOARD_FLASH_WIDTH	4 /* 32-bits */

/* Reserve 512kB for bootinfo (e.g. MAC address) and/or bootloader */
#define FLASH_RESERVED_SIZE 	0x80000

#ifdef CONFIG_MTD_PARTITIONS

static struct mtd_partition memec2vpx_partitions[] = {
	/* 
	 * Some space is reserved for bootinfo 
	 * (e.g. MAC address) and/or bootloader 
	 */
	{
		.name =		"reserved",
		.size =		FLASH_RESERVED_SIZE,
		.offset =	0,
		.mask_flags =	MTD_WRITEABLE	/* force read-only */
	},
	/* the rest can be used for MTD */
	{
		.name =		"flashdisk",
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_NXTBLK,
		.mask_flags =	0
	}
};

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

#endif /* CONFIG_MTD_PARTITIONS */
#endif /* CONFIG_MTD */

/*
 * As an overview of how the following functions (platform_init,
 * memec2vpx_map_io, memec2vpx_setup_arch and memec2vpx_init_IRQ) fit into the
 * kernel startup procedure, here's a call tree:
 *
 * start_here					arch/ppc/kernel/head_4xx.S
 *  early_init					arch/ppc/kernel/setup.c
 *  machine_init				arch/ppc/kernel/setup.c
 *    platform_init				this file
 *      ppc4xx_init				arch/ppc/syslib/ppc4xx_setup.c
 *        parse_bootinfo
 *          find_bootinfo
 *        "setup some default ppc_md pointers"
 *  MMU_init					arch/ppc/mm/init.c
 *    *ppc_md.setup_io_mappings == memec2vpx_map_io	this file
 *      ppc4xx_map_io				arch/ppc/syslib/ppc4xx_setup.c
 *  start_kernel				init/main.c
 *    setup_arch				arch/ppc/kernel/setup.c
 *      *ppc_md.setup_arch == memec2vpx_setup_arch	this file
 *        ppc4xx_setup_arch			arch/ppc/syslib/ppc4xx_setup.c
 *          ppc4xx_find_bridges			arch/ppc/syslib/ppc405_pci.c
 *    init_IRQ					arch/ppc/kernel/irq.c
 *      *ppc_md.init_IRQ == memec2vpx_init_IRQ	this file
 *        ppc4xx_init_IRQ			arch/ppc/syslib/ppc4xx_setup.c
 *          ppc4xx_pic_init			arch/ppc/syslib/xilinx_pic.c
 */

void __init
memec2vpx_map_io(void)
{
	ppc4xx_map_io();
}

static void __init
memec2vpx_early_serial_map(void)
{
	struct serial_state old_ports[] = { SERIAL_PORT_DFNS };
	struct uart_port port;
	int i;

	/* Setup ioremapped serial port access */
	for (i = 0; i < ARRAY_SIZE(old_ports); i++ ) {
		memset(&port, 0, sizeof(port));
		port.membase = ioremap((phys_addr_t)(old_ports[i].iomem_base), 16);
		port.irq = old_ports[i].irq;
		port.uartclk = old_ports[i].baud_base * 16;
		port.regshift = old_ports[i].iomem_reg_shift;
		port.iotype = SERIAL_IO_MEM;
		port.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
		port.line = i;
		port.lock = SPIN_LOCK_UNLOCKED;

#ifdef CONFIG_SERIAL_8250
		if (early_serial_setup(&port) != 0)
			printk("Early serial init of port %d failed\n", i);
#endif
#ifdef CONFIG_KGDB_8250
		kgdb8250_add_port(i, &port);
#endif
	}
}

void __init
memec2vpx_setup_arch(void)
{
	ppc4xx_setup_arch();	/* calls ppc4xx_find_bridges() */

	memec2vpx_early_serial_map();

#ifdef CONFIG_MTD
	/* we use generic physmap mapping driver and we use partitions */
	physmap_configure(BOARD_PHYS_ADDR, BOARD_FLASH_SIZE,
					BOARD_FLASH_WIDTH, NULL);
#ifdef CONFIG_MTD_PARTITIONS
	physmap_set_partitions(memec2vpx_partitions, 
				NB_OF(memec2vpx_partitions));
#endif /* CONFIG_MTD_PARTITIONS */
#endif /* CONFIG_MTD */

	/* Identify the system */
	printk(KERN_INFO XILINX_SYS_ID_STR);
	printk(KERN_INFO "Port by MontaVista Software, Inc. (source@mvista.com)\n");
}

/* Called after board_setup_irq from ppc4xx_init_IRQ(). */
void __init
memec2vpx_init_irq(void)
{
	unsigned int i;

	ppc4xx_init_IRQ();

	/*
	 * For PowerPC 405 cores the default value for NR_IRQS is 32.
	 * See include/asm-ppc/irq.h for details.
	 * This is just fine for Memec 2VPx.
	 */
#if (NR_IRQS != 32)
#error NR_IRQS must be 32 for Memec 2VPx
#endif

	for (i = 0; i < NR_IRQS; i++) {
		if (XPAR_INTC_0_KIND_OF_INTR & (0x80000000 >> i))
			irq_desc[i].status &= ~IRQ_LEVEL;
		else
			irq_desc[i].status |= IRQ_LEVEL;
	}
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	ppc4xx_init(r3, r4, r5, r6, r7);

	ppc_md.setup_arch = memec2vpx_setup_arch;
	ppc_md.setup_io_mappings = memec2vpx_map_io;
	ppc_md.init_IRQ = memec2vpx_init_irq;
}
