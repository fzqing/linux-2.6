/*
 * arch/ppc/platforms/pq2fads_setup.c
 *
 * PQ2FADS Board-specific Platform Device descriptions
 *
 * 2006 (c) MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/fs_enet_pd.h>

#include <asm/io.h>
#include <asm/mpc8260.h>
#include <asm/cpm2.h>
#include <asm/immap_cpm2.h>
#include <asm/irq.h>
#include <asm/ppc_sys.h>
#include <asm/ppcboot.h>
#include <asm/delay.h>

#include "pq2ads_pd.h"

static void init_fcc2_ioports(void);
static void init_fcc3_ioports(void);

static struct fs_mii_bb_platform_info m82xx_mii_bb_pdata = {
	.mdio_port	= fsiop_portc,
	.mdio_bit	= 9,
	.mdc_port	= fsiop_portc,
	.mdc_bit	= 10,
	.delay		= 1,
};

/* First Ethernet Port - FCC2 */
static struct fs_platform_info mpc82xx_fcc2_pdata = {
	.fs_no		= fsid_fcc2,
	.cp_page	= CPM_CR_FCC2_PAGE,
	.cp_block 	= CPM_CR_FCC2_SBLOCK,
	.clk_trx 	= (PC_F2RXCLK | PC_F2TXCLK),
	.clk_route	= CMX2_CLK_ROUTE,
	.clk_mask	= CMX2_CLK_MASK,
	.init_ioports	= init_fcc2_ioports,

	.phy_addr	= 0,
#ifdef PHY_INTERRUPT
	.phy_irq	= PHY_INTERRUPT,
#else
	.phy_irq	= -1;
#endif
	.mem_offset	= FCC2_MEM_OFFSET,
	.rx_ring	= 32,
	.tx_ring	= 32,
	.rx_copybreak	= 240,
	.use_napi	= 0,
	.napi_weight	= 17,
	.bus_id		= "0:00",
};

/* Second Ethernet Port - FCC3 */
static struct fs_platform_info mpc82xx_fcc3_pdata = {
	.fs_no		= fsid_fcc3,
	.cp_page	= CPM_CR_FCC3_PAGE,
	.cp_block 	= CPM_CR_FCC3_SBLOCK,
	.clk_trx 	= (PC_F3RXCLK | PC_F3TXCLK),
	.clk_route	= CMX3_CLK_ROUTE,
	.clk_mask	= CMX3_CLK_MASK,
	.init_ioports 	= init_fcc3_ioports,

	.phy_addr	= 3,
#ifdef PHY_INTERRUPT
	.phy_irq	= PHY_INTERRUPT,
#else
	.phy_irq	= -1;
#endif
	.mem_offset	= FCC3_MEM_OFFSET,
	.rx_ring	= 32,
	.tx_ring	= 32,
	.rx_copybreak	= 240,
	.use_napi	= 0,
	.napi_weight	= 17,
	.bus_id		= "0:03",
};

static void init_fcc2_ioports(void)
{
	struct io_port *io;
	u32 tempval;
	cpm2_map_t* immap = ioremap(CPM_MAP_ADDR, sizeof(cpm2_map_t));
	u32 *bcsr = ioremap(BCSR_ADDR+4, sizeof(u32));

	io = &immap->im_ioport;

	/* Enable the PHY */
	clrbits32(bcsr, BCSR1_FETHIEN);
	setbits32(bcsr, BCSR1_FETH_RST);

	/* FCC2 pins are on port B/C. */
	/* Configure port B and C pins for FCC2 Ethernet. */

	tempval = in_be32(&io->iop_pdirb);
	tempval &= ~PB2_DIRB0;
	tempval |= PB2_DIRB1;
	out_be32(&io->iop_pdirb, tempval);

	tempval = in_be32(&io->iop_psorb);
	tempval &= ~PB2_PSORB0;
	tempval |= PB2_PSORB1;
	out_be32(&io->iop_psorb, tempval);

	setbits32(&io->iop_pparb,PB2_DIRB0 | PB2_DIRB1);

	tempval = PC_F2RXCLK|PC_F2TXCLK;

	/* Alter clocks */
	clrbits32(&io->iop_psorc,tempval);
	clrbits32(&io->iop_pdirc,tempval);
	setbits32(&io->iop_pparc,tempval);

	clrbits32(&immap->im_cpmux.cmx_fcr, CMX2_CLK_MASK);
	setbits32(&immap->im_cpmux.cmx_fcr, CMX2_CLK_ROUTE);

	iounmap(bcsr);
	iounmap(immap);
}

static void init_fcc3_ioports(void)
{
	struct io_port *io;
	u32 tempval;
	cpm2_map_t* immap = ioremap(CPM_MAP_ADDR, sizeof(cpm2_map_t));
	u32 *bcsr = ioremap(BCSR_ADDR+12, sizeof(u32));

	io = &immap->im_ioport;

	/* Enable the PHY */
	clrbits32(bcsr, BCSR3_FETHIEN2);
	setbits32(bcsr, BCSR3_FETH2_RST);

	/* FCC3 pins are on port B/C. */
	/* Configure port B and C pins for FCC3 Ethernet. */

	tempval = in_be32(&io->iop_pdirb);
	tempval &= ~PB3_DIRB0;
	tempval |= PB3_DIRB1;
	out_be32(&io->iop_pdirb, tempval);

	tempval = in_be32(&io->iop_psorb);
	tempval &= ~PB3_PSORB0;
	tempval |= PB3_PSORB1;
	out_be32(&io->iop_psorb, tempval);

	setbits32(&io->iop_pparb,PB3_DIRB0 | PB3_DIRB1);

	/* Alter clocks */
	tempval = PC_F3TXCLK|PC_F3RXCLK;

	clrbits32(&io->iop_psorc, tempval);
	clrbits32(&io->iop_pdirc, tempval);
	setbits32(&io->iop_pparc, tempval);

	clrbits32(&immap->im_cpmux.cmx_fcr, CMX3_CLK_MASK);
	setbits32(&immap->im_cpmux.cmx_fcr, CMX3_CLK_ROUTE);

	iounmap(bcsr);
	iounmap(immap);
}

static void __init pq2fads_fixup_enet_pdata(struct platform_device *pdev,
						int idx)
{
	bd_t* bi = (void*)__res;
	int fs_no = fsid_fcc1+pdev->id-1;

	mpc82xx_fcc3_pdata.dpram_offset = mpc82xx_fcc2_pdata.dpram_offset = (u32)cpm2_immr->im_dprambase;
	mpc82xx_fcc3_pdata.fcc_regs_c = mpc82xx_fcc2_pdata.fcc_regs_c = (u32)cpm2_immr->im_fcc_c;

	switch(fs_no) {
		case fsid_fcc2:
			memcpy(&mpc82xx_fcc2_pdata.macaddr,bi->bi_enetaddr,6);
			pdev->dev.platform_data = &mpc82xx_fcc2_pdata;
		break;
		case fsid_fcc3:
			memcpy(&mpc82xx_fcc3_pdata.macaddr,bi->bi_enetaddr,6);
			mpc82xx_fcc3_pdata.macaddr[5] ^= 1;
			pdev->dev.platform_data = &mpc82xx_fcc3_pdata;
		break;
	}
}

static void __init pq2fads_fixup_mdio_pdata(struct platform_device *pdev,
						int idx)
{
	m82xx_mii_bb_pdata.irq[0] = PHY_INTERRUPT;
	m82xx_mii_bb_pdata.irq[1] = -1;
	m82xx_mii_bb_pdata.irq[2] = -1;
	m82xx_mii_bb_pdata.irq[3] = PHY_INTERRUPT;
	m82xx_mii_bb_pdata.irq[31] = -1;

	pdev->dev.platform_data = &m82xx_mii_bb_pdata;
}

static int pq2fads_platform_notify(struct device *dev)
{
	static const struct platform_notify_dev_map dev_map[] = {
		{
			.bus_id = "fsl-cpm-fcc",
			.rtn = pq2fads_fixup_enet_pdata
		},
		{
			.bus_id = "fsl-bb-mdio",
			.rtn = pq2fads_fixup_mdio_pdata
		},
		{
			.bus_id = NULL
		}
	};
	platform_notify_map(dev_map,dev);

	return 0;

}

int __init pq2fads_init(void)
{
	printk(KERN_NOTICE "pq2fads: Init\n");

	platform_notify = pq2fads_platform_notify;

	ppc_sys_device_initfunc();

	ppc_sys_device_disable_all();
	ppc_sys_device_enable(MPC82xx_CPM_FCC2);
	ppc_sys_device_enable(MPC82xx_CPM_FCC3);
	ppc_sys_device_enable(MPC82xx_MDIO_BB);

	return 0;
}

arch_initcall(pq2fads_init);
