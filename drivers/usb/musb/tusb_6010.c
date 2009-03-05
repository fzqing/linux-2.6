/*
 * TUSB6010 USB 2.0 OTG Dual Role controller
 *
 * Copyright (C) 2006 Nokia Corporation
 * Jarkko Nikula <jarkko.nikula@nokia.com>
 * Tony Lindgren <tony@atomide.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 * - Driver assumes that interface to external host (main CPU) is
 *   configured for NOR FLASH interface instead of VLYNQ serial
 *   interface.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/platform_device.h>

#include "musbdefs.h"
#include "tusb_6010.h"

/*
 * TUSB 6010 may use a parallel bus that doesn't support byte ops;
 * so both loading and unloading FIFOs need explicit byte counts.
 */

void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 * buf)
{
	void __iomem *ep_conf = hw_ep->regs;
	void __iomem *fifo = hw_ep->fifo;
	u8 epnum = hw_ep->bLocalEnd;
	int i, remain;
	u32 val;

	u8 *bufp = buf;

	prefetch(buf);

	DBG(3, "%cX ep%d count %d bufp %p\n", 'T', epnum, len, bufp);

	/* Direction and size of FIFO operation. See also comment
	 * in tusb_6010.h for TUSB_EP_OUT_CONF */
	if (epnum)
		musb_writel(ep_conf, TUSB_EP_OUT_OFFSET,
			    TUSB_EP_OUT_CONFIG_XFR_SIZE(len));
	else
		musb_writel(ep_conf, 0, TUSB_EP0_CONFIG_DIR_OUT |
			    TUSB_EP0_CONFIG_XFR_SIZE(len));

	/* Write full 32-bit blocks from buffer to FIFO */
	for (i = 0; i < (len / 4); i++) {
		val = *(u32 *) bufp;
		musb_writel(fifo, 0, val);
		bufp += 4;
	}

	remain = len - (i * 4);
	if (remain) {
		/* Write rest of 1-3 bytes from buffer into FIFO */
		memcpy(&val, bufp, remain);
		musb_writel(fifo, 0, val);
	}
}

void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 * buf)
{
	void __iomem *ep_conf = hw_ep->regs;
	void __iomem *fifo = hw_ep->fifo;
	u8 epnum = hw_ep->bLocalEnd;
	int i, remain;
	u32 val;

	DBG(3, "%cX ep%d count %d buf %p\n", 'T', epnum, len, buf);

	/* Direction and size of FIFO operation. See also comment
	 * in tusb_6010.h for TUSB_EP_IN_CONF */
	if (epnum)
		musb_writel(ep_conf, TUSB_EP_IN_OFFSET,
			    TUSB_EP_IN_CONFIG_XFR_SIZE(len));
	else
		musb_writel(ep_conf, 0, TUSB_EP0_CONFIG_DIR_IN |
			    TUSB_EP0_CONFIG_XFR_SIZE(len));

	/* Read full 32-bit blocks from FIFO to buffer */
	for (i = 0; i < (len / 4); i++) {
		u32 val = musb_readl(fifo, 0);
		*(u32 *) buf = val;

		/* REVISIT: Remove this once things work reliably */
		if (unlikely((u16) val == 0xdead)) {
			printk(KERN_ERR "tusb: FIFO dead: "
			       "ep%d count %d buf %p\n", epnum, len, buf);
		}

		buf += 4;
	}

	remain = len - (i * 4);
	if (remain) {
		/* Read rest of 1-3 bytes from FIFO */
		val = musb_readl(fifo, 0);
		memcpy(buf, &val, remain);
	}
}

irqreturn_t tusb_interrupt(int irq, void *__hci, struct pt_regs *r)
{
	struct musb *musb = __hci;
	void __iomem *base = musb->ctrl_base;
	void __iomem *musb_base = musb->pRegs;
	unsigned long flags;
	u32 dma_src, int_src, otg_stat, musb_src = 0;

	spin_lock_irqsave(&musb->Lock, flags);

	dma_src = musb_readl(base, TUSB_DMA_INT_SRC);
	int_src = musb_readl(base, TUSB_INT_SRC);
	otg_stat = musb_readl(base, TUSB_DEV_OTG_STAT);

	musb->int_usb = 0;
	musb->int_rx = 0;
	musb->int_tx = 0;
	musb->int_regs = r;

	if (otg_stat & TUSB_DEV_OTG_STAT_ID_STATUS) {
		/* ID pin is up. Either A-plug was removed or TUSB6010
		 * is in peripheral mode */

		/* Still in pheripheral mode? */
		if ((int_src & TUSB_INT_SRC_ID_STATUS_CHNG)) {
			DBG(3, "tusb: Status change\n");
			//return IRQ_HANDLED;
		}
	}

	/* Connect and disconnect */
	if (int_src & TUSB_INT_SRC_USB_IP_CONN) {
		DBG(3, "tusb: Connected\n");
	} else if (int_src & TUSB_INT_SRC_USB_IP_DISCON) {
		DBG(3, "tusb: Disconnected\n");
	}

	/* VBUS state change */
	if ((int_src & TUSB_INT_SRC_VBUS_SENSE_CHNG) ||
	    (int_src & TUSB_INT_SRC_USB_IP_VBUS_ERR)) {
		DBG(3, "tusb: VBUS changed. VBUS state %d\n",
		    (otg_stat & TUSB_DEV_OTG_STAT_VBUS_SENSE) ? 1 : 0);
		if (!(otg_stat & TUSB_DEV_OTG_STAT_VBUS_SENSE) &&
		    !(otg_stat & TUSB_DEV_OTG_STAT_ID_STATUS)) {
			/* VBUS went off and ID pin is down */
			DBG(3, "tusb: No VBUS, starting session\n");
			/* Start session again, VBUS will be enabled */
			musb_writeb(musb_base, MGC_O_HDRC_DEVCTL,
				    MGC_M_DEVCTL_SESSION);
		}
	}

	/* ID pin change */
	if (int_src & TUSB_INT_SRC_ID_STATUS_CHNG) {
		DBG(3, "tusb: ID pin changed. State is %d\n",
		    (musb_readl(base, TUSB_DEV_OTG_STAT) &
		     TUSB_DEV_OTG_STAT_ID_STATUS) ? 1 : 0);
	}

	/* OTG timer expiration */
	if (int_src & TUSB_INT_SRC_OTG_TIMEOUT) {
		DBG(3, "tusb: OTG timer expired\n");
		musb_writel(base, TUSB_DEV_OTG_TIMER,
			    musb_readl(base, TUSB_DEV_OTG_TIMER) |
			    TUSB_DEV_OTG_TIMER_ENABLE);
	}

	/* EP interrupts. In OCP mode tusb6010 mirrors the MUSB * interrupts */
	if (int_src & (TUSB_INT_SRC_USB_IP_TX | TUSB_INT_SRC_USB_IP_RX)) {
		musb_src = musb_readl(base, TUSB_USBIP_INT_SRC);
		musb_writel(base, TUSB_USBIP_INT_CLEAR, musb_src);
		musb->int_rx = (((musb_src >> 16) & 0xffff) << 1);
		musb->int_tx = (musb_src & 0xffff);
	}
	musb->int_usb = (int_src & 0xff);
	if (musb->int_usb || musb->int_rx || musb->int_tx)
		musb_interrupt(musb);

	/* Acknowledge TUSB interrupts. Clear only non-reserved bits */
	if (int_src & TUSB_INT_SRC_CLEAR_MASK) {
		musb_writel(base, TUSB_INT_SRC_CLEAR,
			    int_src & TUSB_INT_SRC_CLEAR_MASK);
	}

	spin_unlock_irqrestore(&musb->Lock, flags);

	return IRQ_HANDLED;
}

/*
 * Enables TUSB6010. Caller must take care of locking.
 * REVISIT:
 * - Check what is unnecessary in MGC_HdrcStart()
 * - Interrupt should really be IRQT_FALLING level sensitive
 */
void tusb_enable(struct musb *musb)
{
	void __iomem *base = musb->ctrl_base;

	/* Setup TUSB6010 main interrupt mask. Enable all interrupts except
	 * reserved ones and VLYNQ invalid access */
	musb_writel(base, TUSB_INT_MASK,
		    TUSB_INT_SRC_RESERVED_MASK |
		    TUSB_INT_SRC_INVALID_ACCESS_MASK(0xf));

	/* Setup subsystem interrupt masks */
	musb_writel(base, TUSB_USBIP_INT_MASK, 0);
	musb_writel(base, TUSB_DMA_INT_MASK, 0);
	musb_writel(base, TUSB_GPIO_INT_MASK, 0x1ff);

	/* Clear all subsystem interrups */
	musb_writel(base, TUSB_USBIP_INT_CLEAR, 0x7fffffff);
	musb_writel(base, TUSB_DMA_INT_CLEAR, 0x7fffffff);
	musb_writel(base, TUSB_GPIO_INT_CLEAR, 0x1ff);

	/* Acknowledge pending interrupt(s) */
	musb_writel(base, TUSB_INT_SRC_CLEAR, ~TUSB_INT_SRC_RESERVED_MASK);

#if 0
	/* Set OTG timer for about one second */
	musb_writel(base, TUSB_DEV_OTG_TIMER,
		    TUSB_DEV_OTG_TIMER_ENABLE |
		    TUSB_DEV_OTG_TIMER_VAL(0x3c00000));
#endif

	/* Only 0 clock cycles for minimum interrupt de-assertion time and
	 * interrupt polarity active low seems to work reliably here */
	musb_writel(base, TUSB_INT_CTRL_CONF, TUSB_INT_CTRL_CONF_INT_RELCYC(0));

	set_irq_type(musb->nIrq, __IRQT_LOWLVL);
}

/*
 * Disables TUSB6010. Caller must take care of locking.
 * REVISIT:
 * - Check what is unnecessary in MGC_HdrcDisable()
 * - Really disable the interrupts we want to disable
 * - Deal with wake-up and gpio interrupts
 */
static void tusb_disable(struct musb *musb)
{
	set_irq_type(musb->nIrq, IRQT_NOEDGE);
}

/*
 * Sets up TUSB6010 CPU interface specific signals and registers
 * Note: Settings optimized for OMAP24xx
 */
static void tusb_setup_cpu_interface(struct musb *musb)
{
	void __iomem *base = musb->ctrl_base;

	/* Disable GPIO[7:0] pullups (used as output DMA requests) */
	musb_writel(base, TUSB_PULLUP_1_CTRL, 0x000000FF);
	/* Disable all pullups on NOR IF, DMAREQ0 and DMAREQ1 */
	musb_writel(base, TUSB_PULLUP_2_CTRL, 0x01FFFFFF);

	/* Turn GPIO[5:0] to DMAREQ[5:0] signals */
	musb_writel(base, TUSB_GPIO_CONF, TUSB_GPIO_CONF_DMAREQ(0x3f));

	/* Burst size 16x16 bits, all six DMA requests enabled, DMA request
	 * de-assertion time 2 system clocks */
	musb_writel(base, TUSB_DMA_REQ_CONF,
		    TUSB_DMA_REQ_CONF_BURST_SIZE(2) |
		    TUSB_DMA_REQ_CONF_DMA_REQ_EN(0x2f) |
		    TUSB_DMA_REQ_CONF_DMA_REQ_ASSER(2));

	/* Set 0 wait count for synchronous burst access */
	musb_writel(base, TUSB_WAIT_COUNT, 0);
}

#define TUSB_REV_MAJOR(reg_val)		((reg_val >> 4) & 0xf)
#define TUSB_REV_MINOR(reg_val)		(reg_val & 0xf)

static void tusb_print_revision(struct musb *musb)
{
	void __iomem *base = musb->ctrl_base;

	pr_info("tusb: Revisions: %s%i.%i %s%i.%i %s%i.%i %s%i.%i\n",
		"prcm",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_PRCM_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_PRCM_REV)),
		"int",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_INT_CTRL_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_INT_CTRL_REV)),
		"gpio",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_GPIO_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_GPIO_REV)),
		"dma",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_DMA_CTRL_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_DMA_CTRL_REV)));
}

static int tusb_start(struct musb *musb)
{
	void __iomem *base = musb->ctrl_base;
	int ret;
	unsigned long flags;

	ret = tusb_power(1);
	if (ret != 0) {
		printk(KERN_ERR "tusb: Cannot enable TUSB6010\n");
		goto err;
	}

	spin_lock_irqsave(&musb->Lock, flags);

	if (musb_readl(base, TUSB_PROD_TEST_RESET) != TUSB_PROD_TEST_RESET_VAL) {
		printk(KERN_ERR "tusb: Unable to detect TUSB6010\n");
		goto err;
	}

	tusb_print_revision(musb);

	/* The uint bit for "USB non-PDR interrupt enable" has to be 1 when
	 * NOR FLASH interface is used */
	musb_writel(base, TUSB_VLYNQ_CTRL, 8);

	/* Select PHY free running 60MHz as a system clock */
	musb_writel(base, TUSB_PRCM_CONF, TUSB_PRCM_CONF_SYS_CLKSEL(1));

	/* VBus valid timer 1us, disable DFT/Debug and VLYNQ clocks for
	 * power saving, enable VBus detect and session end comparators,
	 * enable IDpullup */
	musb_writel(base, TUSB_PRCM_MNGMT,
		    TUSB_PRCM_MNGMT_VBUS_VALID_TIMER(0xa) |
		    TUSB_PRCM_MNGMT_VBUS_VALID_FLT_EN |
		    TUSB_PRCM_MNGMT_DFT_CLK_DIS |
		    TUSB_PRCM_MNGMT_VLYNQ_CLK_DIS |
		    TUSB_PRCM_MNGMT_OTG_SESS_END_EN |
		    TUSB_PRCM_MNGMT_OTG_VBUS_DET_EN |
		    TUSB_PRCM_MNGMT_OTG_ID_PULLUP);

	/* Workaround for enabling IDpullup, VBus detect and session end
	 * comparators in case of silicon bug (which is to be fixed) where they
	 * cannot be enabled in Device PRCM Management Register */
	musb_writel(base, TUSB_PHY_OTG_CTRL_ENABLE,
		    musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE) |
		    TUSB_PHY_OTG_CTRL_WRPROTECT |
		    TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP |
		    TUSB_PHY_OTG_CTRL_OTG_VBUS_DET_EN |
		    TUSB_PHY_OTG_CTRL_OTG_SESS_END_EN);
	musb_writel(base, TUSB_PHY_OTG_CTRL,
		    musb_readl(base, TUSB_PHY_OTG_CTRL) |
		    TUSB_PHY_OTG_CTRL_WRPROTECT |
		    TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP |
		    TUSB_PHY_OTG_CTRL_OTG_VBUS_DET_EN |
		    TUSB_PHY_OTG_CTRL_OTG_SESS_END_EN);

	tusb_setup_cpu_interface(musb);

	spin_unlock_irqrestore(&musb->Lock, flags);

	return 0;

      err:
	tusb_power(0);
	return -ENODEV;
}

void tusb_stop(struct musb *musb)
{
	tusb_power(0);
}

void musb_platform_enable(struct musb *musb)
{
}

void musb_platform_disable(struct musb *musb)
{
}

int __init musb_platform_init(struct musb *musb)
{
	int ret;

	/* Offsets from base: VLYNQ at 0x000, MUSB regs at 0x400,
	 * FIFOs at 0x600, TUSB at 0x800
	 */
	musb->pRegs += TUSB_BASE_OFFSET;

	ret = tusb_start(musb);
	if (ret) {
		printk(KERN_ERR "Could not start tusb6010 (%d)\n", ret);
		return -ENODEV;
	}
	musb->isr = tusb_interrupt;
	return ret;
}
