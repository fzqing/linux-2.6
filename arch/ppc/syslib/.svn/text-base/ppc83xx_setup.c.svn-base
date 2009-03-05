/*
 * arch/ppc/syslib/ppc83xx_setup.c
 *
 * MPC83XX common board code
 *
 * Maintainer: Kumar Gala <kumar.gala@freescale.com>
 *
 * Copyright 2005-2006 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Added PCI support -- Tony Li <tony.li@freescale.com>
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <linux/tty.h>	/* for linux/serial_core.h */
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/time.h>
#include <asm/mpc83xx.h>
#include <asm/mmu.h>
#include <asm/ppc_sys.h>
#include <asm/kgdb.h>
#include <asm/delay.h>

#include <syslib/ppc83xx_setup.h>
#if defined(CONFIG_PCI)
#include <asm/delay.h>
#include <syslib/ppc83xx_pci.h>
#include <syslib/early_i2c.h>
#endif

extern void kgdb8250_add_port(int i, struct uart_port *serial_req);

phys_addr_t immrbar;

/* Return the amount of memory */
unsigned long __init
mpc83xx_find_end_of_memory(void)
{
        bd_t *binfo;

        binfo = (bd_t *) __res;

        return binfo->bi_memsize;
}

long __init
mpc83xx_time_init(void)
{
	bd_t *binfo = (bd_t *)__res;
	u32 *spcr = ioremap(binfo->bi_immr_base + MPC83xx_SPCR, 4);

	*spcr |= MPC83xx_SPCR_TBEN;

	iounmap(spcr);

	return 0;
}

/* The decrementer counts at the system (internal) clock freq divided by 4 */
void __init
mpc83xx_calibrate_decr(void)
{
        bd_t *binfo = (bd_t *) __res;
        unsigned int freq, divisor;

	freq = binfo->bi_busfreq;
	divisor = 4;
	tb_ticks_per_jiffy = freq / HZ / divisor;
	tb_to_us = mulhwu_scale_factor(freq / divisor, 1000000);
	us_to_tb = (freq / divisor) / 1000000;
}

#ifdef CONFIG_SERIAL_8250
void __init
mpc83xx_early_serial_map(void)
{
#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_8250)
	struct uart_port serial_req;
#endif
	struct plat_serial8250_port *pdata;
	bd_t *binfo = (bd_t *) __res;
	pdata = (struct plat_serial8250_port *) ppc_sys_get_pdata(MPC83xx_DUART);

	/* Setup serial port access */
	pdata[0].uartclk = binfo->bi_busfreq;
	pdata[0].mapbase += binfo->bi_immr_base;
	pdata[0].membase = ioremap(pdata[0].mapbase, 0x100);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_8250)
	memset(&serial_req, 0, sizeof (serial_req));
	serial_req.iotype = SERIAL_IO_MEM;
	serial_req.mapbase = pdata[0].mapbase;
	serial_req.membase = pdata[0].membase;
	serial_req.regshift = 0;
	serial_req.uartclk = pdata[0].uartclk;
	serial_req.irq = pdata[0].irq;
	serial_req.line = pdata[0].line;
	serial_req.flags = pdata[0].flags;

	if (early_serial_setup(&serial_req) != 0)
		printk("Early serial init of port 0 failed\n");
#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	gen550_init(0, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(0, &serial_req);
#endif
#endif

	pdata[1].uartclk = binfo->bi_busfreq;
	pdata[1].mapbase += binfo->bi_immr_base;
	pdata[1].membase = ioremap(pdata[1].mapbase, 0x100);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB_8250)
	/* Assume gen550_init() doesn't modify serial_req */
	serial_req.mapbase = pdata[1].mapbase;
	serial_req.membase = pdata[1].membase;
	serial_req.line = pdata[1].line;

	if (early_serial_setup(&serial_req) != 0)
		printk("Early serial init of port 1 failed\n");

#if defined(CONFIG_SERIAL_TEXT_DEBUG)
	gen550_init(1, &serial_req);
#endif
#ifdef CONFIG_KGDB_8250
	kgdb8250_add_port(1, &serial_req);
#endif
#endif
}
#endif

void
mpc83xx_restart(char *cmd)
{
	bd_t *binfo = (bd_t *)__res;

	u32 *rcr = ioremap(binfo->bi_immr_base + MPC83xx_RCR, 4);
	u32 *rpr = ioremap(binfo->bi_immr_base + MPC83xx_RPR, 4);

	/* apply reset protect unlock command to
	 * reset control protection  register */
	*rpr = MPC83xx_RPR_RSTE;

	/* apply software hard reset to
	 * reset control register*/
	*rcr = MPC83xx_RCR_SWHR;

	/* not reached, but... */
	iounmap(rcr);
	iounmap(rpr);
	for (;;) ;
}

void
mpc83xx_power_off(void)
{
	local_irq_disable();
	for(;;);
}

void
mpc83xx_halt(void)
{
	local_irq_disable();
	for(;;);
}

#if defined(CONFIG_PCI)
void __init
mpc83xx_setup_pci1(struct pci_controller *hose)
{
	u16 reg16;
	volatile immr_pcictrl_t * pci_ctrl;
	volatile immr_ios_t * ios;
	bd_t *binfo = (bd_t *) __res;

	pci_ctrl = ioremap(binfo->bi_immr_base + 0x8500, sizeof(immr_pcictrl_t));
	ios = ioremap(binfo->bi_immr_base + 0x8400, sizeof(immr_ios_t));

	/*
	 * Configure PCI Outbound Translation Windows
	 */
	ios->potar0 = (MPC83xx_PCI1_LOWER_MEM >> 12) & POTAR_TA_MASK;
	ios->pobar0 = (MPC83xx_PCI1_LOWER_MEM >> 12) & POBAR_BA_MASK;
	ios->pocmr0 = POCMR_EN |
		(((0xffffffff - (MPC83xx_PCI1_UPPER_MEM -
				MPC83xx_PCI1_LOWER_MEM)) >> 12) & POCMR_CM_MASK);

	/* mapped to PCI1 IO space */
	ios->potar1 = (MPC83xx_PCI1_LOWER_IO >> 12) & POTAR_TA_MASK;
	ios->pobar1 = (MPC83xx_PCI1_IO_BASE >> 12) & POBAR_BA_MASK;
	ios->pocmr1 = POCMR_EN | POCMR_IO |
		(((0xffffffff - (MPC83xx_PCI1_UPPER_IO -
				MPC83xx_PCI1_LOWER_IO)) >> 12) & POCMR_CM_MASK);

	/*
	 * Configure PCI Inbound Translation Windows
	 */
	pci_ctrl->pitar1 = 0x0;
	pci_ctrl->pibar1 = 0x0;
	pci_ctrl->piebar1 = 0x0;
	pci_ctrl->piwar1 = PIWAR_EN | PIWAR_PF | PIWAR_RTT_SNOOP | PIWAR_WTT_SNOOP | PIWAR_IWS_2G;

	/*
	 * Release PCI RST signal
	 */
	pci_ctrl->gcr = 0;
	udelay(2000);
	pci_ctrl->gcr = 1;
	udelay(2000);

	reg16 = 0xff;
	early_read_config_word(hose, hose->first_busno, 0, PCI_COMMAND, &reg16);
	reg16 |= PCI_COMMAND_SERR | PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	early_write_config_word(hose, hose->first_busno, 0, PCI_COMMAND, reg16);

	/*
	 * Clear non-reserved bits in status register.
	 */
	early_write_config_word(hose, hose->first_busno, 0, PCI_STATUS, 0xffff);
	early_write_config_byte(hose, hose->first_busno, 0, PCI_LATENCY_TIMER, 0x80);

	iounmap(pci_ctrl);
	iounmap(ios);
}

void __init
mpc83xx_setup_pci2(struct pci_controller *hose)
{
	u16 reg16;
	volatile immr_pcictrl_t * pci_ctrl;
	volatile immr_ios_t * ios;
	bd_t *binfo = (bd_t *) __res;

	pci_ctrl = ioremap(binfo->bi_immr_base + 0x8600, sizeof(immr_pcictrl_t));
	ios = ioremap(binfo->bi_immr_base + 0x8400, sizeof(immr_ios_t));

	/*
	 * Configure PCI Outbound Translation Windows
	 */
	ios->potar3 = (MPC83xx_PCI2_LOWER_MEM >> 12) & POTAR_TA_MASK;
	ios->pobar3 = (MPC83xx_PCI2_LOWER_MEM >> 12) & POBAR_BA_MASK;
	ios->pocmr3 = POCMR_EN | POCMR_DST |
		(((0xffffffff - (MPC83xx_PCI2_UPPER_MEM -
				MPC83xx_PCI2_LOWER_MEM)) >> 12) & POCMR_CM_MASK);

	/* mapped to PCI2 IO space */
	ios->potar4 = (MPC83xx_PCI2_LOWER_IO >> 12) & POTAR_TA_MASK;
	ios->pobar4 = (MPC83xx_PCI2_IO_BASE >> 12) & POBAR_BA_MASK;
	ios->pocmr4 = POCMR_EN | POCMR_DST | POCMR_IO |
		(((0xffffffff - (MPC83xx_PCI2_UPPER_IO -
				MPC83xx_PCI2_LOWER_IO)) >> 12) & POCMR_CM_MASK);

	/*
	 * Configure PCI Inbound Translation Windows
	 */
	pci_ctrl->pitar1 = 0x0;
	pci_ctrl->pibar1 = 0x0;
	pci_ctrl->piebar1 = 0x0;
	pci_ctrl->piwar1 = PIWAR_EN | PIWAR_PF | PIWAR_RTT_SNOOP | PIWAR_WTT_SNOOP | PIWAR_IWS_2G;

	/*
	 * Release PCI RST signal
	 */
	pci_ctrl->gcr = 0;
	udelay(2000);
	pci_ctrl->gcr = 1;
	udelay(2000);

	reg16 = 0xff;
	early_read_config_word(hose, hose->first_busno, 0, PCI_COMMAND, &reg16);
	reg16 |= PCI_COMMAND_SERR | PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	early_write_config_word(hose, hose->first_busno, 0, PCI_COMMAND, reg16);

	/*
	 * Clear non-reserved bits in status register.
	 */
	early_write_config_word(hose, hose->first_busno, 0, PCI_STATUS, 0xffff);
	early_write_config_byte(hose, hose->first_busno, 0, PCI_LATENCY_TIMER, 0x80);

	iounmap(pci_ctrl);
	iounmap(ios);
}

/*
 * PCI buses can be enabled only if SYS board combinates with PIB
 * (Platform IO Board) board which provide 3 PCI slots. There is 2 PCI buses
 * and 3 PCI slots, so people must configure the routes between them before
 * enable PCI bus. This routes are under the control of PCA9555PW device which
 * can be accessed via I2C bus 2 and are configured by firmware. Refer to
 * Freescale to get more information about firmware configuration.
 */

extern int mpc83xx_exclude_device(u_char bus, u_char devfn);
extern int mpc83xx_map_irq(struct pci_dev *dev, unsigned char idsel,
		unsigned char pin);

#ifdef CONFIG_PPC_INDIRECT_PCI_BE
#define PCI_CFG_OUT out_be32
#else
#define PCI_CFG_OUT out_le32
#endif

static int
mpc83xx_indirect_read_config(struct pci_bus *bus, unsigned int devfn,
				int offset, int len, u32 *val)
{
	struct pci_controller *hose = bus->sysdata;
	volatile unsigned char *cfg_data;
	u8 cfg_type = 0;
	u8 bus_num;

	if (ppc_md.pci_exclude_device)
		if (ppc_md.pci_exclude_device(bus->number, devfn))
			return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == hose->first_busno)
		bus_num = 0;
	else
		bus_num = bus->number;

	PCI_CFG_OUT(hose->cfg_addr,
		 (0x80000000 | (bus_num << 16)
		  | (devfn << 8) | ((offset & 0xfc) | cfg_type)));

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	cfg_data = hose->cfg_data + (offset & 3);
	switch (len) {
	case 1:
		*val = in_8((u8 *)cfg_data);
		break;
	case 2:
		*val = in_le16((u16 *)cfg_data);
		break;
	default:
		*val = in_le32((u32 *)cfg_data);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static int
mpc83xx_indirect_write_config(struct pci_bus *bus, unsigned int devfn,
				  int offset, int len, u32 val)
{
	struct pci_controller *hose = bus->sysdata;
	volatile unsigned char *cfg_data;
	u8 cfg_type = 0;
	u8 bus_num;

	if (ppc_md.pci_exclude_device)
		if (ppc_md.pci_exclude_device(bus->number, devfn))
			return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == hose->first_busno)
		bus_num = 0;
	else
		bus_num = bus->number;

	PCI_CFG_OUT(hose->cfg_addr,
		 (0x80000000 | (bus_num << 16)
		  | (devfn << 8) | ((offset & 0xfc) | cfg_type)));

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	cfg_data = hose->cfg_data + (offset & 3);
	switch (len) {
	case 1:
		out_8((u8 *)cfg_data, val);
		break;
	case 2:
		out_le16((u16 *)cfg_data, val);
		break;
	default:
		out_le32((u32 *)cfg_data, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops mpc83xx_indirect_pci_ops =
{
	mpc83xx_indirect_read_config,
	mpc83xx_indirect_write_config
};

void __init
mpc83xx_setup_hose(void)
{
	u32 val32;
	volatile immr_clk_t * clk;
        volatile law_t * pci_law;
	struct pci_controller * hose1;
#ifdef CONFIG_MPC83xx_PCI2
	struct pci_controller * hose2;
#endif
	bd_t * binfo = (bd_t *)__res;
	u8 val8;

	clk = ioremap(binfo->bi_immr_base + 0xA00, sizeof(immr_clk_t));
        pci_law = ioremap(binfo->bi_immr_base + 0x60, sizeof(law_t));

	 /* Configure PCI clock drivers */
	val32 = clk->occr;
	udelay(2000);
#ifdef CONFIG_CLK_DIV_ENABLE
	clk->occr = 0xffff0003;
#else
	clk->occr = 0xff000000;
#endif
	udelay(2000);

	iounmap(clk);

        /*
         * Configure PCI Local Access Windows
         */
        pci_law[0].bar = MPC83xx_PCI1_LOWER_MEM & LAWBAR_BAR;
        pci_law[0].ar = LAWAR_EN | LAWAR_SIZE_1G;
        pci_law[1].bar = MPC83xx_PCI1_IO_BASE & LAWBAR_BAR;
        pci_law[1].ar = LAWAR_EN | LAWAR_SIZE_32M;

	iounmap(pci_law);

#ifndef CONFIG_MPC834x_ITX
	/*
	 * Configure PMC <-> PCI by I2C bus
	 */
	early_i2c_init(CFG_I2C_SPD,CFG_I2C_SLV);
	val8 = 0;
	early_i2c_write(0x23,0x6,1,&val8,1);
	early_i2c_write(0x23,0x7,1,&val8,1);
	val8 = 0xff;
	early_i2c_write(0x23,0x2,1,&val8,1);
	early_i2c_write(0x23,0x3,1,&val8,1);
	val8 = 0;
	early_i2c_write(0x26,0x6,1,&val8,1);
	val8 = 0x34;
	early_i2c_write(0x26,0x7,1,&val8,1);
#ifdef CONFIG_PCI_64BIT
	val8 = 0xf4;	/* 64bit PMC2<->PCI1 */
#elif defined(CONFIG_PCI_ONE_PCI1) || defined(CONFIG_MPC832XE_MDS)
	val8 = 0xf9;	/* 32bit PMC1<->PCI1,PMC2<->PCI2,PMC3<->PCI2 */
#elif defined(CONFIG_PCI_TWO_PCI1) || defined(CONFIG_MPC8360E_PB)
	val8 = 0xf7;	/* 32bit PMC1<->PCI1,PMC2<->PCI1 PCM3<->PCI2 disabled*/
#else
	val8 = 0xf3;	/* 32bit PMC1<->PCI1,PMC2<->PCI1,PMC3<->PCI1 */
#endif
	early_i2c_write(0x26,0x2,1,&val8,1);
	val8 = 0xff;
	early_i2c_write(0x26,0x3,1,&val8,1);

	val8 = 0;
	early_i2c_write(0x27,0x6,1,&val8,1);
	early_i2c_write(0x27,0x7,1,&val8,1);
	val8 = 0xff;
	early_i2c_write(0x27,0x2,1,&val8,1);
	val8 = 0xef;
	early_i2c_write(0x27,0x3,1,&val8,1);
	asm("eieio");
#endif /* CONFIG_MPC834x_ITX */

	hose1 = pcibios_alloc_controller();
	if(!hose1)
		return;

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mpc83xx_map_irq;

	hose1->bus_offset = 0;
	hose1->first_busno = 0;
	hose1->last_busno = 0xff;

	setup_indirect_pci(hose1, binfo->bi_immr_base + PCI1_CFG_ADDR_OFFSET,
			binfo->bi_immr_base + PCI1_CFG_DATA_OFFSET);
	hose1->ops = &mpc83xx_indirect_pci_ops;

	mpc83xx_setup_pci1(hose1);

	hose1->pci_mem_offset = MPC83xx_PCI1_MEM_OFFSET;
	hose1->mem_space.start = MPC83xx_PCI1_LOWER_MEM;
	hose1->mem_space.end = MPC83xx_PCI1_UPPER_MEM;

	hose1->io_base_phys = MPC83xx_PCI1_IO_BASE;
	hose1->io_space.start = MPC83xx_PCI1_LOWER_IO;
	hose1->io_space.end = MPC83xx_PCI1_UPPER_IO;
#ifdef CONFIG_MPC83xx_PCI2
	isa_io_base = (unsigned long)ioremap(MPC83xx_PCI1_IO_BASE,
			MPC83xx_PCI1_IO_SIZE + MPC83xx_PCI2_IO_SIZE);
#else
	isa_io_base = (unsigned long)ioremap(MPC83xx_PCI1_IO_BASE,
			MPC83xx_PCI1_IO_SIZE);
#endif /* CONFIG_MPC83xx_PCI2 */
	hose1->io_base_virt = (void *)isa_io_base;
	/* setup resources */
	pci_init_resource(&hose1->io_resource,
			MPC83xx_PCI1_LOWER_IO,
			MPC83xx_PCI1_UPPER_IO,
			IORESOURCE_IO, "PCI host bridge 1");
	pci_init_resource(&hose1->mem_resources[0],
			MPC83xx_PCI1_LOWER_MEM,
			MPC83xx_PCI1_UPPER_MEM,
			IORESOURCE_MEM, "PCI host bridge 1");

	ppc_md.pci_exclude_device = mpc83xx_exclude_device;
	hose1->last_busno = pciauto_bus_scan(hose1, hose1->first_busno);

#ifdef CONFIG_MPC83xx_PCI2
	hose2 = pcibios_alloc_controller();
	if(!hose2)
		return;

	hose2->bus_offset = hose1->last_busno + 1;
	hose2->first_busno = hose1->last_busno + 1;
	hose2->last_busno = 0xff;
	setup_indirect_pci(hose2, binfo->bi_immr_base + PCI2_CFG_ADDR_OFFSET,
			binfo->bi_immr_base + PCI2_CFG_DATA_OFFSET);
	hose2->ops = &mpc83xx_indirect_pci_ops;

	mpc83xx_setup_pci2(hose2);

	hose2->pci_mem_offset = MPC83xx_PCI2_MEM_OFFSET;
	hose2->mem_space.start = MPC83xx_PCI2_LOWER_MEM;
	hose2->mem_space.end = MPC83xx_PCI2_UPPER_MEM;

	hose2->io_base_phys = MPC83xx_PCI2_IO_BASE;
	hose2->io_space.start = MPC83xx_PCI2_LOWER_IO;
	hose2->io_space.end = MPC83xx_PCI2_UPPER_IO;
	hose2->io_base_virt = (void *)(isa_io_base + MPC83xx_PCI1_IO_SIZE);
	/* setup resources */
	pci_init_resource(&hose2->io_resource,
			MPC83xx_PCI2_LOWER_IO,
			MPC83xx_PCI2_UPPER_IO,
			IORESOURCE_IO, "PCI host bridge 2");
	pci_init_resource(&hose2->mem_resources[0],
			MPC83xx_PCI2_LOWER_MEM,
			MPC83xx_PCI2_UPPER_MEM,
			IORESOURCE_MEM, "PCI host bridge 2");

	hose2->last_busno = pciauto_bus_scan(hose2, hose2->first_busno);
#endif /* CONFIG_MPC83xx_PCI2 */
}
#endif /*CONFIG_PCI*/
