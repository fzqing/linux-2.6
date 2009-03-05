/*
 * arch/mips/vr5701/vr5701_sg2/setup.c
 *
 * Setup file for NEC Electronics Corporation VR5701 SolutionGearII
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/console.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/fs.h>		/* for ROOT_DEV */
#include <linux/ioport.h>
#include <linux/param.h>	/* for HZ */
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/time.h>
#include <asm/bcache.h>
#include <asm/irq.h>
#include <asm/reboot.h>
#include <asm/gdb-stub.h>
#include <asm/debug.h>
#include <asm/vr5701/vr5701_sg2.h>

static void vr5701_sg2_machine_restart(char *command)
{
	static void (*back_to_prom) (void) = (void (*)(void))0xbfc00000;
	back_to_prom();
}

static void vr5701_sg2_machine_halt(void)
{
	printk(KERN_CRIT "NEC Electronics Corporation VR5701 SolutionGearII halted.\n");
	while (1) ;
}

static void vr5701_sg2_machine_power_off(void)
{
	printk(KERN_CRIT "NEC Electronics Corporation VR5701 SolutionGearII halted. Please turn off the power.\n");
	while (1) ;
}

static void __init vr5701_sg2_time_init(void)
{
	mips_hpt_frequency = CPU_COUNTER_FREQUENCY;
}

static void __init vr5701_sg2_timer_setup(struct irqaction *irq)
{
	unsigned int count;
	irq->flags |= SA_NODELAY;

	/* we are using the cpu counter for timer interrupts */
	setup_irq(7, irq);

	/* to generate the first timer interrupt */
	count = read_c0_count();
	write_c0_compare(count + 10000);
}

#if defined(CONFIG_BLK_DEV_INITRD)
extern unsigned long __rd_start, __rd_end, initrd_start, initrd_end;
#endif

static void chk_init_5701_reg(unsigned long addr, unsigned long data)
{
	unsigned long a = ddb_in32(addr);

	if (a != data) {
		printk(KERN_INFO
		       "Unexpected 5701 reg : addr = %08lX, expected = %08lX, read = %08lX\n",
		       addr + VR5701_IO_BASE, data, a);
	}
}

static void __init vr5701_sg2_board_init(void)
{
	chk_init_5701_reg(0, 0x1e00008f);
	chk_init_5701_reg(PADR_SDRAM01, 0x000000a8);
	chk_init_5701_reg(PADR_LOCALCS0, 0x1f00004c);
	chk_init_5701_reg(LOCAL_CST0, 0x00088622);
	chk_init_5701_reg(LOCAL_CFG, 0x000f0000);

	/* setup PCI windows - window0 for MEM/config, window1 for IO */
	ddb_set_pdar(PADR_PCIW0, 0x10000000, 0x08000000, 32, 0, 1);
	ddb_set_pdar(PADR_PCIW1, 0x18000000, 0x00800000, 32, 0, 1);
	ddb_set_pdar(PADR_IOPCIW0, 0x18800000, 0x00800000, 32, 0, 1);
	ddb_set_pdar(PADR_IOPCIW1, 0x19000000, 0x00800000, 32, 0, 1);
	/* ------------ reset PCI bus and BARs ----------------- */
	ddb_pci_reset_bus();
	/* Ext. PCI memory space */
	ddb_out32(PCI_BAR_MEM01, 0x00000000); /* workaround - Restriction no.6 */
	ddb_out8(PCI_MLTIM, 0x40);

	ddb_out32(PCI_BAR_LCS0, 0xffffffff);
	ddb_out32(PCI_BAR_LCS1, 0xffffffff);
	ddb_out32(PCI_BAR_LCS2, 0xffffffff);
	ddb_out32(PCI_BAR_LCS3, 0xffffffff);
	/* Int. PCI memory space */
	ddb_out8(IPCI_MLTIM, 0x40);

	ddb_out32(IPCI_BAR_LCS0, 0xffffffff);
	ddb_out32(IPCI_BAR_LCS1, 0xffffffff);
	ddb_out32(IPCI_BAR_LCS2, 0xffffffff);
	ddb_out32(IPCI_BAR_LCS3, 0xffffffff);
	ddb_out32(IPCI_BAR_IREG, 0xffffffff);

	ddb_out32(IPCI_CTRLL, 0x01000000); /* workaround - Restriction no.1 */
	ddb_out32(EPCI_CTRLL, 0x01000000); /* workaround - Restriction no.1 */
	/*
	 * We use pci master register 0  for memory space / config space
	 * And we use register 1 for IO space.
	 * Note that for memory space, we bump up the pci base address
	 * so that we have 1:1 mapping between PCI memory and cpu physical.
	 * For PCI IO space, it starts from 0 in PCI IO space but with
	 * IO_BASE in CPU physical address space.
	 */
	ddb_set_pmr(EPCI_INIT0, DDB_PCICMD_MEM, 0x10000000, DDB_PCI_ACCESS_32);
	ddb_set_pmr(EPCI_INIT1, DDB_PCICMD_IO, 0x00000000, DDB_PCI_ACCESS_32);
	ddb_set_pmr(IPCI_INIT0, DDB_PCICMD_MEM, 0x18800000, DDB_PCI_ACCESS_32);
	ddb_set_pmr(IPCI_INIT1, DDB_PCICMD_IO, 0x01000000, DDB_PCI_ACCESS_32);

	/* PCI cross window should be set properly */
	ddb_set_pdar(PCI_BAR_IPCIW0, 0x18800000, 0x00800000, 32, 0, 1);
	ddb_set_pdar(PCI_BAR_IPCIW1, 0x19000000, 0x00800000, 32, 0, 1);
	ddb_set_pdar(IPCI_BAR_EPCIW0, 0x10000000, 0x08000000, 32, 0, 1);
	ddb_set_pdar(IPCI_BAR_EPCIW1, 0x18000000, 0x00800000, 32, 0, 1);

	/* setup GPIO */
	ddb_out32(GIU_DIR0, 0xf7ebffdf);
	ddb_out32(GIU_DIR1, 0x000007fa);
	ddb_out32(GIU_FUNCSEL0, 0xf1c07fff);
	ddb_out32(GIU_FUNCSEL1, 0x000007f0);
	chk_init_5701_reg(GIU_DIR0, 0xf7ebffdf);
	chk_init_5701_reg(GIU_DIR1, 0x000007fa);
	chk_init_5701_reg(GIU_FUNCSEL0, 0xf1c07fff);
	chk_init_5701_reg(GIU_FUNCSEL1, 0x000007f0);

	/* enable USB input buffers */
	ddb_out32(PIB_MISC, (ddb_in32(PIB_MISC) | 0x00000031));
}

int __init vr5701_sg2_setup(void)
{
	set_io_port_base(0xB8000000);

	board_time_init = vr5701_sg2_time_init;
	board_timer_setup = vr5701_sg2_timer_setup;

	_machine_restart = vr5701_sg2_machine_restart;
	_machine_halt = vr5701_sg2_machine_halt;
	_machine_power_off = vr5701_sg2_machine_power_off;

	/* setup resource limits */
	ioport_resource.end = 0x02000000;
	iomem_resource.end = 0xffffffff;

	/* Reboot on panic */
	panic_timeout = 30;

#ifdef CONFIG_FB
	conswitchp = &dummy_con;
#endif

	vr5701_sg2_board_init();

#if defined(CONFIG_BLK_DEV_INITRD)
	initrd_start = (unsigned long)&__rd_start;
	initrd_end = (unsigned long)&__rd_end;
#endif
	register_pci_controller(&VR5701_ext_controller);
	register_pci_controller(&VR5701_io_controller);
	return 0;
}

void __init bus_error_init(void)
{
	/* do nothing */
}

early_initcall(vr5701_sg2_setup);
