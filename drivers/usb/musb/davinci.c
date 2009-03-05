/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <asm/hardware/clock.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/memory.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include <asm/arch/cpu.h>
#include <linux/usb.h>
#include "../core/hcd.h"

#include "musbdefs.h"
#include "musb_host.h"

#include <asm/arch/i2c-client.h>
#include <asm/arch/mux.h>
#include "davinci.h"

#ifdef CONFIG_USB_TI_CPPI_DMA
#include "cppi_dma.h"
#endif
/*
 * USB
 */
static inline void phy_on(void)
{
#if defined(CONFIG_MACH_DAVINCI_EVM) || defined(CONFIG_ARCH_DAVINCI_DM355)
	/* start the on-chip PHY and its PLL */
	__raw_writel(USBPHY_SESNDEN | USBPHY_VBDTCTEN | USBPHY_PHYPLLON,
		IO_ADDRESS(USBPHY_CTL_PADDR));
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM355
	__raw_writel(USBPHY_SESNDEN | USBPHY_VBDTCTEN | USBPHY_PHYPLLON
		| DM355_USBPHY_DATAPOL, IO_ADDRESS(USBPHY_CTL_PADDR));
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM646x
	__raw_writel(0, IO_ADDRESS(USBPHY_CTL_PADDR));
#endif
	while ((__raw_readl(IO_ADDRESS(USBPHY_CTL_PADDR))
			& USBPHY_PHYCLKGD) == 0)
		cpu_relax();
}

static inline void phy_off(void)
{
	/* powerdown the on-chip PHY and its oscillator */
#if defined(CONFIG_MACH_DAVINCI_EVM) || defined(CONFIG_ARCH_DAVINCI_DM355)
	__raw_writel(USBPHY_OSCPDWN | USBPHY_PHYSPDWN,
		IO_ADDRESS(USBPHY_CTL_PADDR));
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM646x
	__raw_writel(USBPHY_PHYSPDWN, IO_ADDRESS(USBPHY_CTL_PADDR));
#endif
}

void musb_platform_enable(struct musb *musb)
{
	u32 tmp, old, val;

	/* workaround:  setup irqs through both register sets */
	tmp = (musb->wEndMask & DAVINCI_USB_TX_ENDPTS_MASK)
	    << DAVINCI_USB_TXINT_SHIFT;
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_SET_REG, tmp);
	old = tmp;
	tmp = (musb->wEndMask & (0xfffe & DAVINCI_USB_RX_ENDPTS_MASK))
	    << DAVINCI_USB_RXINT_SHIFT;
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_SET_REG, tmp);
	tmp |= old;

	val = ~MGC_M_INTR_SOF;
	tmp |= ((val & 0x01ff) << DAVINCI_USB_USBINT_SHIFT);
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_SET_REG, tmp);
}

/*
 * Disable the HDRC and flush interrupts
 */
void musb_platform_disable(struct musb *musb)
{
	/* because we don't set CTRLR.UINT, "important" to:
	 *  - not read/write INTRUSB/INTRUSBE
	 *  - (except during initial setup, as workaround)
	 *  - use INTSETR/INTCLRR instead
	 */
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_CLR_REG,
		    DAVINCI_USB_USBINT_MASK
		    | DAVINCI_USB_TXINT_MASK | DAVINCI_USB_RXINT_MASK);
	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, 0);
	musb_writel(musb->ctrl_base, DAVINCI_USB_EOI_REG, 0);
}

/* REVISIT this file shouldn't modify the OTG state machine ...
 *
 * The OTG infrastructure needs updating, to include things like
 * offchip DRVVBUS support and replacing MGC_OtgMachineInputs with
 * musb struct members (so e.g. vbus_state vanishes).
 */
#if defined (CONFIG_MACH_DAVINCI_EVM) || (CONFIG_MACH_DAVINCI_HD_EVM)
static int vbus_state = -1;

/* I2C operations are always synchronous, and require a task context.
 * With unloaded systems, using the shared workqueue seems to suffice
 * to satisfy the 100msec A_WAIT_VRISE timeout...
 */
static void evm_deferred_drvvbus(void *_musb)
{
	int is_on = vbus_state;

#ifdef CONFIG_MACH_DAVINCI_EVM
	davinci_i2c_expander_op(0x3a, USB_DRVVBUS, !is_on);
#endif
#ifdef CONFIG_MACH_DAVINCI_HD_EVM
	davinci_i2c_expander_op(0x3a, USB_FB_DM646X, !is_on);
#endif
	vbus_state = is_on;
}

DECLARE_WORK(evm_vbus_work, evm_deferred_drvvbus, 0);

#ifdef CONFIG_USB_MUSB_HDRC_HCD
/* VBUS SWITCHING IS BOARD-SPECIFIC */
static void davinci_vbus_power(struct musb *musb, int is_on)
{
	if (is_on)
		is_on = 1;

	DBG(1, "vbus_state=>%d\n", is_on);

	if (is_on) {
		MUSB_HST_MODE(musb);
	} else {
		MUSB_DEV_MODE(musb);
	}
#if defined (CONFIG_MACH_DAVINCI_EVM) || (CONFIG_MACH_DAVINCI_HD_EVM)
	if (machine_is_davinci_evm()) {
#ifdef CONFIG_USB_MUSB_OTG

		/* modified EVM board switching VBUS with GPIO(6) not I2C
		 * NOTE:  PINMUX0.RGB888 (bit23) must be clear
		 */
		if (!is_on) {
			REG_DVEVM_GPIO45_SET |= DAVINCI_VBUS_OFF;
		} else {
			REG_DVEVM_GPIO45_CLR |= DAVINCI_VBUS_ON;
		}
#else
		vbus_state = is_on;
		DBG(2, "VBUS power %s\n", is_on ? "on" : "off");
		schedule_work(&evm_vbus_work);
#endif
	}
#endif
}
#endif
#endif

static irqreturn_t davinci_interrupt(int irq, void *__hci, struct pt_regs *r)
{
	unsigned long flags;
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = __hci;
	void *__iomem tibase = musb->ctrl_base;
	u32 tmp;

	spin_lock_irqsave(&musb->Lock, flags);

#ifdef CONFIG_USB_TI_CPPI_DMA
	/* CPPI interrupts share the same IRQ line, but have their own
	 * mask, state, and EIO registers.
	 */
	{
		u32 cppi_tx = musb_readl(tibase, DAVINCI_TXCPPI_MASKED_REG);
		u32 cppi_rx = musb_readl(tibase, DAVINCI_RXCPPI_MASKED_REG);

		if (cppi_tx || cppi_rx) {
			DBG(4, "<== CPPI IRQ t%x r%x\n", cppi_tx, cppi_rx);
			cppi_completion(musb, cppi_rx, cppi_tx);
			retval = IRQ_HANDLED;
		}
	}
#endif

	/* NOTE: DaVinci shadows the Mentor IRQs; don't manage them through
	 * the mentor registers (except for setup), use the TI ones and EOI.
	 */

	/* ack and handle non-CPPI interrupts */
	tmp = musb_readl(tibase, DAVINCI_USB_INT_SRC_MASKED_REG);
	musb_writel(tibase, DAVINCI_USB_INT_SRC_CLR_REG, tmp);

	musb->int_rx = (tmp & DAVINCI_USB_RXINT_MASK)
	    >> DAVINCI_USB_RXINT_SHIFT;
	musb->int_tx = (tmp & DAVINCI_USB_TXINT_MASK)
	    >> DAVINCI_USB_TXINT_SHIFT;
	musb->int_usb = (tmp & DAVINCI_USB_USBINT_MASK)
	    >> DAVINCI_USB_USBINT_SHIFT;
	musb->int_regs = r;

	if (tmp & (1 << (8 + DAVINCI_USB_USBINT_SHIFT))) {
		int drvvbus = musb_readl(tibase, DAVINCI_USB_STAT_REG);

		/* NOTE:  this must complete poweron within 100 msec */
		DBG(1, "drvvbus Interrupt\n");

#ifndef CONFIG_ARCH_DAVINCI_DM355
#ifdef CONFIG_USB_MUSB_OTG
		davinci_vbus_power(musb, drvvbus);
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
		/* In host mode manipulate vbus based on core request
		 * but keep the session on.
		 */
		davinci_vbus_power(musb, drvvbus);
#endif
#endif
		drvvbus = 0;
		retval = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)
		retval |= musb_interrupt(musb);

	/* irq stays asserted until EOI is written */
	musb_writel(tibase, DAVINCI_USB_EOI_REG, 0);

	spin_unlock_irqrestore(&musb->Lock, flags);

	/* REVISIT we sometimes get unhandled IRQs with CPPI
	 * (minimally, host TX).  not clear why...
	 */
	if (retval != IRQ_HANDLED)
		DBG(5, "unhandled? %08x\n", tmp);
	return IRQ_HANDLED;
}

int __init musb_platform_init(struct musb *musb)
{
	void *__iomem tibase = musb->ctrl_base;
	u32 revision, phystatus;
	u8 id;
	struct platform_device *pdev = to_platform_device(musb->controller);
	struct musb_hdrc_platform_data *plat;
	struct clk *clkp;

	if (pdev->id == -1)
		id = 0;
	else
		id = pdev->id;
	
	switch (id) {
	case 0 :
		clkp = clk_get (NULL, "USBCLK");
		break;
	default :
		return -ENODEV;
	}
	if (IS_ERR(clkp))
		return -ENODEV;

	musb->clock = clkp;
	clk_use (clkp);
	if (clk_enable (clkp) != 0)
		return -ENODEV;

	plat = musb->controller->platform_data;

	/* overwrite the USB mode */
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	plat->mode = MUSB_HOST;

#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	plat->mode = MUSB_PERIPHERAL;

#elif defined(CONFIG_USB_MUSB_OTG)
	plat->mode = MUSB_OTG;
#else
	dev_dbg(musb->controller, "incompatible Kconfig role setting");
	return -EINVAL;
#endif

	musb->board_mode = plat->mode;
	musb->pRegs += DAVINCI_BASE_OFFSET;

	/* returns zero if e.g. not clocked */
	revision = musb_readl(tibase, DAVINCI_USB_VERSION_REG);
	if (revision == 0)
		return -ENODEV;

	/* note that transceiver issues make us want to charge
	 * VBUS only when the PHY PLL is not active.
	 */
#if defined(CONFIG_MACH_DAVINCI_EVM) || defined (CONFIG_MACH_DAVINCI_HD_EVM)
#ifdef CONFIG_USB_MUSB_OTG
	/* clear EMACEN to enble OTG GPIO 16 for VBus power control */
	/* Set GPIO Direction */
	REG_DVEVM_GPIO45_DIR &= ~(DVEVM_GPIO45_DIR_OUT);
	davinci_cfg_reg(DM644X_GPIO3V);
#endif
	evm_vbus_work.data = musb;
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM355
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	gpio_direction_output(2, 1);
#else
	gpio_direction_output(2, 0);
#endif
	 __raw_writel( __raw_readl (IO_ADDRESS(DM355_DEEPSLEEP_REG)) &
		0xfffffff0, IO_ADDRESS (DM355_DEEPSLEEP_REG));
#endif

	/* reset the controller */
	musb_writel(tibase, DAVINCI_USB_CTRL_REG, 0x1);

	/* start the on-chip PHY and its PLL */
	phy_on();

	msleep(5);
	phystatus = __raw_readl(IO_ADDRESS(USBPHY_CTL_PADDR));
#ifdef CONFIG_ARCH_DAVINCI_DM646x
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	__raw_writel(phystatus | DM646X_USBPHY_SESSION_VBUS |
		DM646X_USBPHY_NDATAPOL, IO_ADDRESS(USBPHY_CTL_PADDR));
#else
	__raw_writel (phystatus | DM646X_USBPHY_SESSION_VBUS |
		DM646X_USBPHY_NDATAPOL | DM646X_USBPHY_PERI_USBID,
		IO_ADDRESS(USBPHY_CTL_PADDR));
#endif
#endif

	/* NOTE:  irqs are in mixed mode, not bypass to pure-musb */
	pr_debug("DaVinci OTG revision %08x phy %03x control %02x\n",
		 revision,
		 musb_readl((void *__iomem)IO_ADDRESS(USBPHY_CTL_PADDR), 0x00),
		 musb_readb(tibase, DAVINCI_USB_CTRL_REG));

	musb->isr = davinci_interrupt;
	return 0;
}

int musb_platform_exit(struct musb *musb)
{
	phy_off();

#ifdef CONFIG_MACH_DAVINCI_EVM
#ifdef CONFIG_USB_MUSB_OTG
	/* Set EMACEN to enable OTG GPIO 16 for Emac control */
	/* Set GPIO Direction */
	davinci_cfg_reg(DM644X_EMACEN);
#endif
#endif

#if defined(CONFIG_MACH_DAVINCI_HD_EVM) && defined(CONFIG_USB_MUSB_HDRC_HCD)
	davinci_vbus_power(musb, 0);
#endif
	if (musb->clock) {
		clk_disable (musb->clock);
		clk_unuse (musb->clock);
	}

	return 0;
}
