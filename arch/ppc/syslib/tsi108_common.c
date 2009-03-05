/*
 * arch/ppc/syslib/tsi108_common.c
 *
 * Common routines for Tundra Semiconductor TSI108 host bridge.
 *
 * 2004-2005 (c) Tundra Semiconductor Corp.
 * Author: Alex Bounine (alexandreb@tundra.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/open_pic.h>
#include <asm/tsi108.h>

#undef TSI108_PCI_DEBUG

#define tsi_mk_config_addr(bus, dev, func, offset) \
		(((bus)<<16) | ((dev)<<11) | (func<<8) | \
			(offset & 0xfc)| tsi108_pci_cfg_base)
#define tsi_mk_config_addr1(bus, devfunc, offset) \
	(((bus)<<16) | ((devfunc)<<8) | (offset & 0xfc)| tsi108_pci_cfg_base)

u32 tsi108_csr_base;
u32 tsi108_pci_cfg_base;

/*
 * Prosessor Bus Clock (in MHz) defined by CG_PB_SELECT
 * (based on recommended Tsi108 reference clock 33MHz)
 */
static int pb_clk_sel[8] = { 0, 0, 183, 100, 133, 167, 200, 233 };

/*
 * SDRAM Clock (in MHz) defined by CG_SD_SELECT
 * (based on recommended Tsi108 reference clock 33MHz)
 */
static int sd_clk_sel[2][8] = {
	{0, 0, 183, 100, 133, 167, 200, 233},	/* SYNC */
	{0, 0, 0, 0, 133, 160, 200, 0}	/* ASYNC */
};

int __init tsi108_bridge_init(struct pci_controller *hose, uint phys_csr_base)
{
	/* Nothing to do here at this moment */
	return 0;
}

unsigned long __init tsi108_get_mem_size(void)
{
	ulong total;

	total = 512 * 1024 * 1024;
	return total;
}

int
tsi108_direct_write_config(struct pci_bus *bus, unsigned int devfunc,
			   int offset, int len, u32 val)
{
	volatile unsigned char *cfg_addr;

	cfg_addr = (unsigned char *)(tsi_mk_config_addr1(bus->number,
							 devfunc, offset) |
							 (offset & 0x03));

#ifdef TSI108_PCI_DEBUG
	printk("PCI CFG write : ");
	printk("%d:0x%x:0x%x ", bus->number, devfunc, offset);
	printk("%d ADDR=0x%08x ", len, (uint) cfg_addr);
	printk("data = 0x%08x\n", val);
#endif

	switch (len) {
	case 1:
		out_8((u8 *) cfg_addr, val);
		break;
	case 2:
		out_le16((u16 *) cfg_addr, val);
		break;
	default:
		out_le32((u32 *) cfg_addr, val);
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

void tsi108_clear_pci_error(u32 pci_cfg_base)
{
	u32 err_stat, err_addr, pci_stat;

	/*
	 * Quietly clear PB and PCI error flags set as result
	 * of PCI/X configuration read requests.
	 */

	/* Read PB Error Log Registers */
	err_stat = tsi108_read_reg(TSI108_PB_OFFSET + TSI108_PB_ERRCS);
	err_addr = tsi108_read_reg(TSI108_PB_OFFSET + TSI108_PB_AERR);

	if (err_stat & TSI108_PB_ERRCS_ES) {
		/* Clear error flag */
		tsi108_write_reg(TSI108_PB_OFFSET + TSI108_PB_ERRCS,
				 TSI108_PB_ERRCS_ES);

		/* Clear read error reported in PB_ISR */
		tsi108_write_reg(TSI108_PB_OFFSET + TSI108_PB_ISR,
				 TSI108_PB_ISR_PBS_RD_ERR);

		/* Clear PCI/X bus cfg errors if applicable */
		if ((err_addr & 0xFF000000) == pci_cfg_base) {
			pci_stat =
			    tsi108_read_reg(TSI108_PCI_OFFSET + TSI108_PCI_CSR);
			tsi108_write_reg(TSI108_PCI_OFFSET + TSI108_PCI_CSR,
					 pci_stat);
		}
	}

	return;
}

#define __tsi108_read_pci_config(x, addr, op)		\
	__asm__ __volatile__(				\
		"	"op" %0,0,%1\n"		\
		"1:	eieio\n"			\
		"2:\n"					\
		".section .fixup,\"ax\"\n"		\
		"3:	li %0,-1\n"			\
		"	b 2b\n"				\
		".section __ex_table,\"a\"\n"		\
		"	.align 2\n"			\
		"	.long 1b,3b\n"			\
		".text"					\
		: "=r"(x) : "r"(addr))

int
tsi108_direct_read_config(struct pci_bus *bus, unsigned int devfn, int offset,
			  int len, u32 * val)
{
	volatile unsigned char *cfg_addr;
	u32 temp;

	cfg_addr = (unsigned char *)(tsi_mk_config_addr1(bus->number,
							 devfn,
							 offset) | (offset &
								    0x03));

	switch (len) {
	case 1:
		__tsi108_read_pci_config(temp, cfg_addr, "lbzx");
		break;
	case 2:
		__tsi108_read_pci_config(temp, cfg_addr, "lhbrx");
		break;
	default:
		__tsi108_read_pci_config(temp, cfg_addr, "lwbrx");
		break;
	}

	*val = temp;

#ifdef TSI108_PCI_DEBUG
	if ((0xFFFFFFFF != temp) && (0xFFFF != temp) && (0xFF != temp)) {
		printk("PCI CFG read : ");
		printk("%d:0x%x:0x%x ", bus->number, devfn, offset);
		printk("%d ADDR=0x%08x ", len, (uint) cfg_addr);
		printk("data = 0x%x\n", *val);
	}
#endif
	return PCIBIOS_SUCCESSFUL;
}

unsigned long tsi108_get_cpu_clk(void)
{
	/* Detect PB clock freq. */
	u32 i = tsi108_read_reg(TSI108_CLK_OFFSET + TSI108_CG_PWRUP_STATUS);

	i = (i >> 16) & 0x07;	/* Get PB PLL multiplier */
	return (pb_clk_sel[i] * 1000000);
}

unsigned long tsi108_get_sdc_clk(void)
{
	u32 i, k;

	/* Get SDC/PB clock freq. from CG settings */
	i = tsi108_read_reg(TSI108_CLK_OFFSET + TSI108_CG_PWRUP_STATUS);
	k = (i >> 16) & 0x07;	/* Get PB PLL multiplier */
	i = (i >> 20) & 0x07;	/* Get SDC PLL multiplier */
	k = (k == i) ? 0 : 1;	/* sync/async configuration */
	return (sd_clk_sel[k][i] * 1000000);
}
