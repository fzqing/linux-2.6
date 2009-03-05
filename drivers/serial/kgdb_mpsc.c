/*
 * drivers/serial/kgdb_mpsc.
 *
 * KGDB driver for the Marvell MultiProtocol Serial Controller (MPCS)
 *	
 * Based on the polled boot loader driver by Ajit Prem (ajit.prem@motorola.com)
 *
 * Author: Randy Vinson <rvinson@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc.
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */



#include <linux/config.h>
#include <linux/kgdb.h>
#include <linux/mv643xx.h>
#include <linux/device.h>
#include <linux/touch_watchdogs.h>
#include <asm/mv64x60.h>
#include <asm/serial.h>
#include <asm/io.h>
#include <asm/delay.h>

#include "mpsc.h"

/* Speed of the UART. */
#if defined(CONFIG_KGDB_9600BAUD)
static int kgdbmpsc_baud = 9600;
#elif defined(CONFIG_KGDB_19200BAUD)
static int kgdbmpsc_baud = 19200;
#elif defined(CONFIG_KGDB_38400BAUD)
static int kgdbmpsc_baud = 38400;
#elif defined(CONFIG_KGDB_57600BAUD)
static int kgdbmpsc_baud = 57600;
#else
static int kgdbmpsc_baud = 115200;	/* Start with this if not given */
#endif

/* Index of the UART, matches ttyMX naming. */
#if defined(CONFIG_KGDB_TTYMM1)
static int kgdbmpsc_ttyMM = 1;
#else
static int kgdbmpsc_ttyMM = 0;		/* Start with this if not given */
#endif

#define MPSC_INTR_REG_SELECT(x)	((x) + (8 * kgdbmpsc_ttyMM))

static int kgdbmpsc_init(void);

static struct platform_device mpsc_dev, shared_dev;

static void __iomem *mpsc_base;
static void __iomem *brg_base;
static void __iomem *routing_base;
static void __iomem *sdma_base;

static unsigned int mpsc_irq;

static void
kgdb_write_debug_char(int c)
{
	u32	data;

	data = readl(mpsc_base + MPSC_MPCR);
	writeb(c, mpsc_base + MPSC_CHR_1);
	mb();
	data = readl(mpsc_base + MPSC_CHR_2);
	data |= MPSC_CHR_2_TTCS;
	writel(data, mpsc_base + MPSC_CHR_2);
	mb();

	while (readl(mpsc_base + MPSC_CHR_2) & MPSC_CHR_2_TTCS);
}

static int
kgdb_get_debug_char(void)
{
	unsigned char 	c;

	do {
		touch_watchdogs();
	} while (!(readl(sdma_base + MPSC_INTR_REG_SELECT(MPSC_INTR_CAUSE)) &
			MPSC_INTR_CAUSE_RCC));

	c = readb(mpsc_base + MPSC_CHR_10 + (1<<1));
	mb();
	writeb(c, mpsc_base + MPSC_CHR_10 + (1<<1));
	mb();
	writel(~MPSC_INTR_CAUSE_RCC, sdma_base +
			MPSC_INTR_REG_SELECT(MPSC_INTR_CAUSE));
	return(c);
}

/*
 * This is the receiver interrupt routine for the GDB stub.
 * All that we need to do is verify that the interrupt happened on the
 * line we're in charge of.  If this is true, schedule a breakpoint and
 * return.
 */
static irqreturn_t
kgdbmpsc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	if (irq != mpsc_irq)
		return IRQ_NONE;
        /*
         * If  there is some other CPU in KGDB then this is a
         * spurious interrupt. so return without even checking a byte
         */
        if (atomic_read(&debugger_active))
               return IRQ_NONE;

	if (readl(sdma_base + MPSC_INTR_REG_SELECT(MPSC_INTR_CAUSE)) &
			MPSC_INTR_CAUSE_RCC)
		breakpoint();

	return IRQ_HANDLED;
}

static int __init
kgdbmpsc_init(void)
{
	struct mpsc_pdata *pdata;
	u32	cdv;

	if (!brg_base || !mpsc_base || !routing_base || !sdma_base)
		return -1;

	/* Set MPSC Routing to enable both ports */
	writel(0x0, routing_base + MPSC_MRR);

	/* MPSC 0/1 Rx & Tx get clocks BRG0/1 */
	writel(0x00000100, routing_base + MPSC_RCRR);
	writel(0x00000100, routing_base + MPSC_TCRR);

	/* Disable all MPSC interrupts and clear any pending interrupts */
	writel(0, sdma_base + MPSC_INTR_REG_SELECT(MPSC_INTR_MASK));
	writel(0, sdma_base + MPSC_INTR_REG_SELECT(MPSC_INTR_CAUSE));

	pdata = (struct mpsc_pdata *)mpsc_dev.dev.platform_data;

	/* cdv = (clock/(2*16*baud rate)) for 16X mode. */
	cdv = ((pdata->brg_clk_freq/(32*kgdbmpsc_baud))-1);
	writel((pdata->brg_clk_src << 18) | (1 << 16) | cdv,
			brg_base + BRG_BCR);

	/* Put MPSC into UART mode, no null modem, 16x clock mode */
	writel(0x000004c4, mpsc_base + MPSC_MMCRL);
	writel(0x04400400, mpsc_base + MPSC_MMCRH);

	writel(0, mpsc_base + MPSC_CHR_1);
	writel(0, mpsc_base + MPSC_CHR_9);
	writel(0, mpsc_base + MPSC_CHR_10);
	writel(4, mpsc_base + MPSC_CHR_3);
	writel(0x20000000, mpsc_base + MPSC_CHR_4);
	writel(0x9000, mpsc_base + MPSC_CHR_5);
	writel(0, mpsc_base + MPSC_CHR_6);
	writel(0, mpsc_base + MPSC_CHR_7);
	writel(0, mpsc_base + MPSC_CHR_8);

	/* 8 data bits, 1 stop bit */
	writel((3 << 12), mpsc_base + MPSC_MPCR);

	/* Enter "hunt" mode */
	writel((1<<31), mpsc_base + MPSC_CHR_2);

	udelay(100);
	return 0;
}

static void __iomem* __init
kgdbmpsc_map_resource(struct platform_device *pd, int type, int num)
{
	void __iomem *base = NULL;
	struct resource *r;
		
	if ((r = platform_get_resource(pd, IORESOURCE_MEM, num)))
		base = ioremap(r->start, r->end - r->start + 1);
	return base;
}

static void __iomem* __init
kgdbmpsc_unmap_resource(struct platform_device *pd, int type, int num, void __iomem* base)
{
	if (base)
		iounmap(base);
	return NULL;
}

static void __init
kgdbmpsc_reserve_resource(struct platform_device *pd, int type, int num)
{
	struct resource *r;
		
	if ((r = platform_get_resource(pd, IORESOURCE_MEM, num)))
		request_mem_region(r->start, r->end - r->start + 1, "kgdb");
}

static int __init
kgdbmpsc_local_init(void)
{
	if (!mpsc_dev.num_resources || !shared_dev.num_resources)
		return 1; /* failure */

	mpsc_base = kgdbmpsc_map_resource(&mpsc_dev, IORESOURCE_MEM,
			MPSC_BASE_ORDER);
	brg_base = kgdbmpsc_map_resource(&mpsc_dev, IORESOURCE_MEM,
			MPSC_BRG_BASE_ORDER);

	/* get the platform data for the shared registers and get them mapped */
	routing_base = kgdbmpsc_map_resource(&shared_dev,
			IORESOURCE_MEM, MPSC_ROUTING_BASE_ORDER);
	sdma_base = kgdbmpsc_map_resource(&shared_dev, IORESOURCE_MEM,
			MPSC_SDMA_INTR_BASE_ORDER);

	mpsc_irq = platform_get_irq(&mpsc_dev, 1);

	if (mpsc_base && brg_base && routing_base && sdma_base)
		return 0; /* success */

	return 1; /* failure */
}

static void __init
kgdbmpsc_local_exit(void)
{
	if (sdma_base)
		sdma_base = kgdbmpsc_unmap_resource(&shared_dev, IORESOURCE_MEM,
					MPSC_SDMA_INTR_BASE_ORDER, sdma_base);
	if (routing_base)
		routing_base = kgdbmpsc_unmap_resource(&shared_dev,
					IORESOURCE_MEM, MPSC_ROUTING_BASE_ORDER,
					routing_base);
	if (brg_base)
		brg_base = kgdbmpsc_unmap_resource(&mpsc_dev, IORESOURCE_MEM,
					MPSC_BRG_BASE_ORDER, brg_base);
	if (mpsc_base)
		mpsc_base = kgdbmpsc_unmap_resource(&mpsc_dev, IORESOURCE_MEM,
					MPSC_BASE_ORDER, mpsc_base);
}

static void __init
kgdbmpsc_update_pdata(struct platform_device *pdev)
{

	snprintf(pdev->dev.bus_id, BUS_ID_SIZE, "%s.%u", pdev->name, pdev->id);
}

static int __init
kgdbmpsc_pdev_init(void)
{
	struct platform_device *pdev;

	/* get the platform data for the specified port. */
	pdev = mv64x60_early_get_pdev_data(MPSC_CTLR_NAME, kgdbmpsc_ttyMM, 1);
	if (pdev) {
		memcpy(&mpsc_dev, pdev, sizeof(struct platform_device));
		if (platform_notify) {
			kgdbmpsc_update_pdata(&mpsc_dev);
			platform_notify(&mpsc_dev.dev);
		}

		/* get the platform data for the shared registers. */
		pdev = mv64x60_early_get_pdev_data(MPSC_SHARED_NAME, 0, 0);
		if (pdev) {
			memcpy(&shared_dev, pdev,
				sizeof(struct platform_device));
			if (platform_notify) {
				kgdbmpsc_update_pdata(&shared_dev);
				platform_notify(&shared_dev.dev);
			}
		}
	}
	return 0;
}

postcore_initcall(kgdbmpsc_pdev_init);

static int __init
kgdbmpsc_init_io(void)
{

	kgdbmpsc_pdev_init();

	if (kgdbmpsc_local_init()) {
		kgdbmpsc_local_exit();
		return -1;
	}

	if (kgdbmpsc_init() == -1)
		return -1;
	return 0;
}

static void __init
kgdbmpsc_hookup_irq(void)
{
	unsigned int msk;
	if (!request_irq(mpsc_irq, kgdbmpsc_interrupt, 0,
				"kgdb mpsc", NULL)) {
		/* Enable interrupt */
		msk = readl(sdma_base + MPSC_INTR_REG_SELECT(MPSC_INTR_MASK));
		msk |= MPSC_INTR_CAUSE_RCC;
		writel(msk, sdma_base + MPSC_INTR_REG_SELECT(MPSC_INTR_MASK));

		kgdbmpsc_reserve_resource(&mpsc_dev, IORESOURCE_MEM,
				MPSC_BASE_ORDER);
		kgdbmpsc_reserve_resource(&mpsc_dev, IORESOURCE_MEM,
				MPSC_BRG_BASE_ORDER);
	}
}

struct kgdb_io kgdb_io_ops = {
	.read_char = kgdb_get_debug_char,
	.write_char = kgdb_write_debug_char,
	.init = kgdbmpsc_init_io,
	.late_init = kgdbmpsc_hookup_irq,
};
