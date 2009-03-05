/*
 * arch/mips/pci/pci-tx4939.c
 *
 * tx4939 pcic setup routines
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/stddef.h>
#include <asm/tx4939/tx4939.h>

static struct resource pci_io_resource = {
	.name = "pci IO space",
	.start = 0x0,
	.end = 0x0,
	.flags = IORESOURCE_IO,
 };

static struct resource pci_mem_resource = {
	.name = "pci memory space",
	.start = 0x0,
	.end = 0x0,
	.flags = IORESOURCE_MEM,
};

static struct resource tx4939_pcic1_pci_io_resource = {
	.name = "PCI1 IO",
	.start = 0x0,
	.end = 0x0,
	.flags = IORESOURCE_IO,
};
static struct resource tx4939_pcic1_pci_mem_resource = {
	.name = "PCI1 mem",
	.start = 0x0,
	.end = 0x0,
	.flags = IORESOURCE_MEM,
};

struct pci_controller tx4939_pci_controller[] = {
	/* h/w only supports devices 0x00 to 0x14 */
	{
		.pci_ops = &tx4939_pci_ops,
		.io_resource = &pci_io_resource,
		.mem_resource = &pci_mem_resource,
	},
	/* h/w only supports devices 0x00 to 0x14 */
	{
		.pci_ops = &tx4939_pci_ops,
		.io_resource = &tx4939_pcic1_pci_io_resource,
		.mem_resource = &tx4939_pcic1_pci_mem_resource,
	},
};


struct tx4939_pcic_reg *pcicptrs[4] = {
	tx4939_pcicptr(0),
};

/**
 * set_tx4939_pcicptr - set pcicptr
 * @ch: PCI controller channel
 * @pcicptr: setting pointer
 *
 * This function sets pcicptr which use in pci_ops.c.
 */

void __init set_tx4939_pcicptr(int ch, struct tx4939_pcic_reg *pcicptr)
{
	pcicptrs[ch] = pcicptr;
}

/**
 * get_tx4939_pcic_reg - get pcicptr
 * @ch: PCI controller channel
 *
 * This function gets pcicptr which use in pci_ops.c.
 */

struct tx4939_pcic_reg *get_tx4939_pcicptr(int ch)
{
	return pcicptrs[ch];
}

#ifdef CONFIG_TC35815
inline unsigned long tc_readl(volatile __u32 * addr)
{
	return readl(addr);
}

inline void tc_writel(unsigned long data, volatile __u32 * addr)
{
	writel(data, addr);
}
#endif


/**
 * tx4939_pcic_setup - sets up PCI controller
 * @ch: PCIC channel
 * @intarb:
 *
 * This functions sets up PCI controller.
 */
void __init tx4939_pcic_setup(unsigned ch, unsigned long pci_io_base,
			      int intarb)
{
	int i;
	u32 l;
	u64 q;
	struct tx4939_pcic_reg *r = tx4939_pcicptr(ch);
	struct pci_controller *pcic = &tx4939_pci_controller[ch];

	/* Disable All Initiator Space */
	l = reg_rd32(&r->pciccfg);
	reg_wr32(&r->pciccfg, l & ~(TX4939_PCICCFG_G2PM0EN |
				    TX4939_PCICCFG_G2PM1EN |
				    TX4939_PCICCFG_G2PM2EN |
				    TX4939_PCICCFG_G2PIOEN));
	/* GB->PCI IO mappings */
	reg_wr32(&r->g2piomask,
		 (pcic->io_resource->end - pcic->io_resource->start) >> 4);
	reg_wr64s(&r->g2piogbase, pci_io_base |
#if defined(__BIG_ENDIAN)
		 TX4939_G2PIOGBASE_EXFER
#else
		 TX4939_G2PIOGBASE_BSWAP
#endif
		);
	reg_wr64s(&r->g2piopbase, 0);

	/* GB->PCI MEM mappings */
	for (i = 0; i < 3; i++) {
		reg_wr32(&r->g2pmmask[i], 0);
		reg_wr64s(&r->g2pmgbase[i], 0);
		reg_wr64s(&r->g2pmpbase[i], 0);
	}
	if (pcic->mem_resource->end) {
		reg_wr32(&r->g2pmmask[0],
			 (pcic->mem_resource->end - pcic->mem_resource->start) >> 4);
		reg_wr64s(&r->g2pmgbase[0], pcic->mem_resource->start |
#if defined(__BIG_ENDIAN)
			 TX4939_G2PMGBASE_EXFER
#else
			 TX4939_G2PMGBASE_BSWAP
#endif
			);
		reg_wr64s(&r->g2pmpbase[0], pcic->mem_resource->start);
	}

	/* PCI->GB IO mappings */
	reg_wr32(&r->p2giopbase, 0);
	reg_wr64s(&r->p2giogbase, 0 |
#if defined(__BIG_ENDIAN)
		  TX4939_P2GIOGBASE_EXFER
#else
		  TX4939_P2GIOGBASE_BSWAP
#endif
		);
	/* PCI->GB MEM mappings */
	l = reg_rd32(&r->p2gm0cfg);
	reg_wr32(&r->p2gm0cfg, l & ~TX4939_P2GMCFG_MSS_MASK);
	l = reg_rd32(&r->p2gm0cfg);
	reg_wr32(&r->p2gm0cfg, l | TX4939_P2GMCFG_MSS_MB(tx4939_get_mem_size()));
	reg_wr32(&r->p2gm0plbase, 0);
	reg_wr32(&r->p2gm0pubase, 0);
	reg_wr64s(&r->p2gmgbase[0], 0 | TX4939_P2GMGBASE_P2GMEN |
#if defined(__BIG_ENDIAN)
		  TX4939_P2GMGBASE_EXFER
#else
		  TX4939_P2GMGBASE_BSWAP
#endif
		);
	reg_wr64s(&r->p2gmgbase[1], 0);
	l = reg_rd32(&r->p2gm1cfg);
	reg_wr32(&r->p2gm1cfg, l & ~TX4939_P2GMCFG_MSS_MASK);
	l = reg_rd32(&r->p2gm1cfg);
	reg_wr32(&r->p2gm1cfg, l | TX4939_P2GMCFG_MSS_MB(16));	/* 16MB */
	reg_wr32(&r->p2gm1plbase, 0xffffffff);
	reg_wr32(&r->p2gm1pubase, 0xffffffff);
	reg_wr64s(&r->p2gmgbase[2], 0);
	l = reg_rd32(&r->p2gm2cfg);
	reg_wr32(&r->p2gm2cfg, l & ~TX4939_P2GMCFG_MSS_MASK);
	l = reg_rd32(&r->p2gm2cfg);
	reg_wr32(&r->p2gm2cfg, l | TX4939_P2GMCFG_MSS_MB(1));	/* 1MB */
	reg_wr32(&r->p2gm2pbase, 0xffffffff);

	l = reg_rd32(&r->pciccfg);
	reg_wr32(&r->pciccfg,
		 l & TX4939_PCICCFG_GBWC_SET(CONFIG_TOSHIBA_TX4939_PCI_GBWC));
	/* Enable Initiator Memory Space */
	if (pcic->mem_resource->end) {
		l = reg_rd32(&r->pciccfg);
		reg_wr32(&r->pciccfg, l | TX4939_PCICCFG_G2PM0EN);
	}
	/* Enable Initiator I/O Space */
	if (pcic->io_resource->end) {
		l = reg_rd32(&r->pciccfg);
		reg_wr32(&r->pciccfg, l | TX4939_PCICCFG_G2PIOEN);
	}
	/* Enable Initiator Config */
	l = reg_rd32(&r->pciccfg);
	reg_wr32(&r->pciccfg,
		 l | (TX4939_PCICCFG_ICAEN | TX4939_PCICCFG_TCAR));

	/* Do not use MEMMUL, MEMINF: YMFPCI card causes M_ABORT. */
	reg_wr32(&r->pcicfg1, 0);
	/* set RETRYTO=0, TRDYTO=0 */
	reg_wr32(&r->g2ptocnt, (TX4939_G2PTOCNT_RETRYTO_SET(0)
				| TX4939_G2PTOCNT_TRDYTO_SET(0)));
	/* pdmac endian setting */
	q = reg_rd64s(&r->pdmcfg);
	reg_wr64s(&r->pdmcfg, q |
#if defined(__BIG_ENDIAN)
		  TX4939_PDMCFG_BSWAP
#else
		  TX4939_PDMCFG_EXFER
#endif
		);

	/* Clear All Local Bus Status */
	reg_wr32(&r->pcicstatus, TX4939_PCICSTATUS_ALL);
	/* Enable All Local Bus Interrupts */
	reg_wr32(&r->pcicmask, TX4939_PCICSTATUS_ALL);
	/* Clear All Initiator Status */
	reg_wr32(&r->g2pstatus, TX4939_G2PSTATUS_ALL);
	/* Enable All Initiator Interrupts */
	reg_wr32(&r->g2pmask, TX4939_G2PSTATUS_ALL);
	/* Clear All PCI Status Error */
	l = reg_rd32(&r->pcistatus);
	reg_wr32(&r->pcistatus, (l & 0x0000ffff) | (TX4939_PCISTATUS_ALL << 16));
	/* Enable All PCI Status Error Interrupts */
	reg_wr32(&r->pcimask, TX4939_PCISTATUS_ALL);

	if (intarb) {		/* case of built-in PCI bus arbiter */
		/* Reset Bus Arbiter */
		reg_wr32(&r->pbacfg, TX4939_PBACFG_RPBA);
		reg_wr32(&r->pbabm, 0);
		/* Enable Bus Arbiter */
		reg_wr32(&r->pbacfg, TX4939_PBACFG_PBAEN);
	}
	/* set PCIC IRQ number */
	l = reg_rd32(&r->pcicfg2);
	reg_wr32(&r->pcicfg2,
		 (l & ~TX4939_PCICFG2_IL_MASK) | (TX4939_IRQ_PCIC(ch) - TX4939_IRQ_IRC_BEG));

	reg_wr32(&r->pcistatus, PCI_COMMAND_MASTER |
		 PCI_COMMAND_MEMORY | PCI_COMMAND_PARITY | PCI_COMMAND_SERR);
}

static void tx4939_dump_pcic_settings(unsigned ch)
{
	int i;
	u32 *preg = (u32 *) tx4939_pcicptr(ch);

	printk(KERN_INFO "tx4939 pcic(%d) settings:", ch);

	for (i = 0; i < sizeof(struct tx4939_pcic_reg); i += 4) {
		if (i % 32 == 0)
			printk(KERN_INFO "\n%04x:", i);
		if (preg == &tx4939_pcicptr(ch)->g2pintack
		    || preg == &tx4939_pcicptr(ch)->g2pspc) {
			printk(KERN_INFO " XXXXXXXX");
			preg++;
			continue;
		}
		printk(KERN_INFO " %08lx", (unsigned long)*preg++);
		if (preg == &tx4939_pcicptr(ch)->g2pcfgadrs)
			break;
	}
	printk(KERN_INFO "\n");
}

struct tx4939_pcic_stat_tbl {
	unsigned long flag;
	const char *str;
};

static struct tx4939_pcic_stat_tbl pcistat_tbl[] = {
	{ PCI_STATUS_DETECTED_PARITY,  "DetectedParityError"},
	{ PCI_STATUS_SIG_SYSTEM_ERROR, "SignaledSystemError"},
	{ PCI_STATUS_REC_MASTER_ABORT, "ReceivedMasterAbort"},
	{ PCI_STATUS_REC_TARGET_ABORT, "ReceivedTargetAbort"},
	{ PCI_STATUS_SIG_TARGET_ABORT, "SignaledTargetAbort"},
	{ PCI_STATUS_PARITY,           "MasterParityError"  },};

static struct tx4939_pcic_stat_tbl g2pstat_tbl[] = {
	{ TX4939_G2PSTATUS_IDTTOE, "IDTIOE"},
	{ TX4939_G2PSTATUS_IDRTOE, "IDRTOE"},};

static struct tx4939_pcic_stat_tbl pcicstat_tbl[] = {
	{ TX4939_PCICSTATUS_PME,  "PME" },
	{ TX4939_PCICSTATUS_TLB,  "TLB" },
	{ TX4939_PCICSTATUS_NIB,  "NIB" },
	{ TX4939_PCICSTATUS_ZIB,  "ZIB" },
	{ TX4939_PCICSTATUS_PERR, "PERR"},
	{ TX4939_PCICSTATUS_SERR, "SERR"},
	{ TX4939_PCICSTATUS_GBE,  "GBE" },
	{ TX4939_PCICSTATUS_IWB,  "IWB" },};

static inline void tx4939_report_pcic_status1(struct tx4939_pcic_reg *pcicptr)
{
	int i;
	unsigned short pcistatus =
		(unsigned short)(reg_rd32(&pcicptr->pcistatus) >> 16);
	unsigned long g2pstatus = reg_rd32(&pcicptr->g2pstatus);
	unsigned long pcicstatus = reg_rd32(&pcicptr->pcicstatus);

	printk(KERN_INFO "pcistat:%04x(", pcistatus);
	for (i = 0; i < ARRAY_SIZE(pcistat_tbl); i++)
		if (pcistatus & pcistat_tbl[i].flag)
			printk(KERN_INFO "%s ", pcistat_tbl[i].str);
	printk("), g2pstatus:%08lx(", g2pstatus);
	for (i = 0; i < ARRAY_SIZE(g2pstat_tbl); i++)
		if (g2pstatus & g2pstat_tbl[i].flag)
			printk("%s ", g2pstat_tbl[i].str);
	printk("), pcicstatus:%08lx(", pcicstatus);
	for (i = 0; i < ARRAY_SIZE(pcicstat_tbl); i++)
		if (pcicstatus & pcicstat_tbl[i].flag)
			printk("%s ", pcicstat_tbl[i].str);
	printk(")\n");
}

void tx4939_report_pcic_status(void)
{
	int i;
	struct tx4939_pcic_reg *pcicptr;
	for (i = 0; (pcicptr = get_tx4939_pcicptr(i)) != NULL; i++)
		tx4939_report_pcic_status1(pcicptr);
}

/**
 * tx4939_pcierr_interrupt - PCI error interrupt hendler
 * @irq: irq number
 * @dev_id:
 * @regs:
 *
 */

static irqreturn_t tx4939_pcierr_interrupt(int irq, void *dev_id,
					   struct pt_regs *regs)
{
#if defined(CONFIG_IDE)
	/* ignore MasterAbort for ide probing... */
	u32 stat = reg_rd32(&tx4939_pcicptr(0)->pcistatus);
	if (((stat >> 16) & 0xf900) ==  PCI_STATUS_REC_MASTER_ABORT) {
		reg_wr32(&tx4939_pcicptr(0)->pcistatus,
			 (stat & 0x0000ffff) | (PCI_STATUS_REC_MASTER_ABORT << 16));
		goto done;
	}
#endif
	printk(KERN_ERR "PCIERR interrupt (irq 0x%x) at 0x%08lx.\n", irq, regs->cp0_epc);
	printk(KERN_ERR "ccfg:%Lx, tear:%Lx\n",
	       reg_rd64s(&tx4939_ccfgptr->ccfg),
	       reg_rd64s(&tx4939_ccfgptr->toea));
	tx4939_report_pcic_status();
	show_regs(regs);
	tx4939_dump_pcic_settings(0);
	tx4939_dump_pcic_settings(1);
	panic("PCI error.");
 done:
	return IRQ_HANDLED;
}

static struct irqaction tx4939_pcic_action = {
	.handler = tx4939_pcierr_interrupt,
	.flags = SA_INTERRUPT,
	.mask = CPU_MASK_NONE,
	.name = "PCIC",
};

static struct irqaction tx4939_pcierr_action = {
	.handler = tx4939_pcierr_interrupt,
	.flags = SA_INTERRUPT,
	.mask = CPU_MASK_NONE,
	.name = "PCI error",
};

static struct irqaction tx4939_pcic1_action = {
	.handler = tx4939_pcierr_interrupt,
	.flags = SA_INTERRUPT,
	.mask = CPU_MASK_NONE,
	.name = "PCIC1(ether)",
};

void __init tx4939_pci_setup_irq(void)
{
	setup_irq(TX4939_IRQ_PCIC(0), &tx4939_pcic_action);
	setup_irq(TX4939_IRQ_PCI0ERR, &tx4939_pcierr_action);
	setup_irq(TX4939_IRQ_PCIC(1), &tx4939_pcic1_action);
}

static void tx4939_pcic_bootlog(void)
{
	int intarb =
	    (reg_rd64s(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_PCIARB_BUILTIN);
	int pci66 = (reg_rd64s(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_PCI66);

	printk
	    (KERN_INFO "TX4939 PCIC (Internal)-- DID:%04x VID:%04x RID:%02x Arbiter:%s (%2sMHz)\n",
	     (unsigned short)(reg_rd32(&tx4939_pcicptr(0)->pciid) >> 16),
	     (unsigned short)(reg_rd32(&tx4939_pcicptr(0)->pciid) & 0xffff),
	     (unsigned short)(reg_rd32(&tx4939_pcicptr(0)->pciccrev) & 0xff),
	     intarb ? "External" : "Internal", pci66 ? "66" : "33");
	printk(KERN_INFO "TX4939 PCIC1(Ether)   -- DID:%04x VID:%04x RID:%02x\n",
	       (unsigned short)(reg_rd32(&tx4939_pcicptr(1)->pciid) >> 16),
	       (unsigned short)(reg_rd32(&tx4939_pcicptr(1)->pciid) & 0xffff),
	       (unsigned short)(reg_rd32(&tx4939_pcicptr(1)->pciccrev) &
				0xff));
}

/**
 * tx4939_pci_setup - pci setup routine
 *
 * This function resets PCI controller using CCFG.CLKCTR pci reset
 * bit, and sets tx4939_pci_controller and mips_io_port_base.
 */
static int __init tx4939_pcibios_init(void)
{
	u64 q;
	int intarb =
		(reg_rd64s(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_PCIARB_BUILTIN);

	tx4939_pcic_bootlog();

	/* Reset PCIC */
	q = reg_rd64s(&tx4939_ccfgptr->clkctr);
	reg_wr64s(&tx4939_ccfgptr->clkctr, q | (TX4939_CLKCTR_PCICRST |
					       TX4939_CLKCTR_PCI1RST |
						  TX4939_CLKCTR_EPCIRST));
	wbflush();
	udelay(1);		/* wait 128 cpu clock for CLKCTR (2.5ns x 128 = 320ns) */
	/* clear PCIC reset */
	q = reg_rd64s(&tx4939_ccfgptr->clkctr);
	reg_wr64s(&tx4939_ccfgptr->clkctr, q & ~(TX4939_CLKCTR_PCICRST |
						   TX4939_CLKCTR_PCI1RST |
						   TX4939_CLKCTR_EPCIRST));
	wbflush();
	udelay(1);		/* wait 128 cpu clock for CLKCTR (2.5ns x 128 = 320ns) */

#if 0 /* PCI Hard Reset */
	{
		u32 l;
		l = reg_rd32(&tx4939_pcicptr(0)->pciccfg);
		reg_wr32(&tx4939_pcicptr(0)->pciccfg, l | TX4939_PCICCFG_HRST);
		l = reg_rd32(&tx4939_pcicptr(1)->pciccfg);
		reg_wr32(&tx4939_pcicptr(1)->pciccfg, l | TX4939_PCICCFG_HRST);
		wbflush();
		mdelay(1);		/* wait 32 G-Bus clock cycles */
	}
#endif

	/* setup PCI0 area */
	tx4939_pci_controller[0].io_resource->start = ioport_resource.start;
	tx4939_pci_controller[0].io_resource->end =
		(ioport_resource.start + TX4939_PCI0_IO_RESOURCE_SIZE) - 1;

	tx4939_pci_controller[0].mem_resource->start =
		TX4939_PCI0_MEM_RESOURCE_START;
	tx4939_pci_controller[0].mem_resource->end =
		(TX4939_PCI0_MEM_RESOURCE_START + TX4939_PCI0_MEM_RESOURCE_SIZE) - 1;

	tx4939_pci_controller[0].pci_ops = &tx4939_pci_ops;

	set_tx4939_pcicptr(0, tx4939_pcicptr(0));

	register_pci_controller(&tx4939_pci_controller[0]);

	tx4939_pcic_setup(0, TX4939_PCI0_IO_RESOURCE_START, intarb);


	/* setup PCI1(Ether) area */
	tx4939_pci_controller[1].io_resource->start =
		TX4939_PCI1_IO_RESOURCE_START - TX4939_PCI0_IO_RESOURCE_START;
	tx4939_pci_controller[1].io_resource->end =
		(TX4939_PCI1_IO_RESOURCE_START - TX4939_PCI0_IO_RESOURCE_START
		 + TX4939_PCI1_IO_RESOURCE_SIZE) - 1;

	tx4939_pci_controller[1].mem_resource->start =
		TX4939_PCI1_MEM_RESOURCE_START;
	tx4939_pci_controller[1].mem_resource->end =
		(TX4939_PCI1_MEM_RESOURCE_START + TX4939_PCI1_MEM_RESOURCE_SIZE) - 1;

	tx4939_pci_controller[1].pci_ops = &tx4939_pci_ops;

	set_tx4939_pcicptr(1, tx4939_pcicptr(1));

	register_pci_controller(&tx4939_pci_controller[1]);

	tx4939_pcic_setup(1, TX4939_PCI1_IO_RESOURCE_START, 1);

	/* map ioport 0 to PCI I/O space address 0 */
	set_io_port_base(KSEG1 + TX4939_PCI0_IO_RESOURCE_START);

	return 0;
}

arch_initcall(tx4939_pcibios_init);
