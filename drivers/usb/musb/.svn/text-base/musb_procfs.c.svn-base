/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
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
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

/*
 * Inventra Controller Driver (ICD) for Linux.
 *
 * The code managing debug files (currently in procfs).
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/usb.h>
#include <asm/uaccess.h>	/* FIXME remove procfs writes */

#include "musbdefs.h"
#ifdef CONFIG_ARCH_DAVINCI
#include "davinci.h"
#endif

#ifdef CONFIG_USB_MUSB_OTG

static const char *state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:
		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:
		return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:
		return "a_wait_bcon";
	case OTG_STATE_A_HOST:
		return "a_host";
	case OTG_STATE_A_SUSPEND:
		return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:
		return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:
		return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:
		return "a_vbus_err";
	case OTG_STATE_B_IDLE:
		return "b_idle";
	case OTG_STATE_B_SRP_INIT:
		return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:
		return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:
		return "b_wait_acon";
	case OTG_STATE_B_HOST:
		return "b_host";
	default:
		return "UNDEFINED";
	}
}

#endif

#ifdef CONFIG_USB_MUSB_HDRC_HCD
static int dump_urbs(struct list_head *list, char *buf, unsigned max)
{
	int count = 0;
	int tmp;
	struct urb *urb;

	if (list_empty(list))
		return snprintf(buf, max, "\t(queue empty)\n");

	list_for_each_entry(urb, list, urb_list) {
		const unsigned pipe = urb->pipe;

		/* for non-multipoint, urb->dev never changes */
		tmp = snprintf(buf, max,
			       "\turb %p dev%d ep%d%s-%s %d/%d\n",
			       urb, urb->dev->devnum,
			       usb_pipeendpoint(pipe),
			       usb_pipein(pipe) ? "in" : "out", ( {
								 char *s;
								 switch
								 (usb_pipetype
								  (pipe)) {
case PIPE_BULK:
s = "bulk"; break; case PIPE_INTERRUPT:
s = "int"; break; case PIPE_CONTROL:
s = "control"; break; default:
								 s = "iso";
								 break;}; s;}
			       ),
			       urb->actual_length,
			       urb->transfer_buffer_length) ;
		if (tmp < 0)
			break;
		tmp = min(tmp, (int)max);
		count += tmp;
		buf += tmp;
		max -= tmp;
	}
	return count;
}
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
static int dump_ep(struct musb_ep *ep, char *buffer, unsigned max)
{
	char *buf = buffer;
	int code = 0;
	void __iomem *regs = ep->pThis->pRegs;

	do {
		struct usb_request *req;

		code = snprintf(buf, max,
				"\n%s (hw%d): %scsr %04x maxp %04x\n",
				ep->name, ep->bEndNumber,
				ep->dma ? "dma, " : "",
				MGC_ReadCsr16(regs,
					      (ep->is_in || !ep->bEndNumber)
					      ? MGC_O_HDRC_TXCSR
					      : MGC_O_HDRC_RXCSR,
					      ep->bEndNumber),
				MGC_ReadCsr16(regs,
					      ep->is_in
					      ? MGC_O_HDRC_TXMAXP
					      : MGC_O_HDRC_RXMAXP,
					      ep->bEndNumber)
		    );
		if (code < 0)
			break;
		code = min(code, (int)max);
		buf += code;
		max -= code;

#ifdef	CONFIG_USB_TI_CPPI_DMA
		if (ep->bEndNumber) {
			unsigned cppi = ep->bEndNumber - 1;
			void __iomem *base = ep->pThis->ctrl_base;
			unsigned off1 = cppi << 2;
			void __iomem *ram = base;
			char tmp[16];

			if (ep->is_in) {
				ram += DAVINCI_TXCPPI_STATERAM_OFFSET(cppi);
				tmp[0] = 0;
			} else {
				ram += DAVINCI_RXCPPI_STATERAM_OFFSET(cppi);
				snprintf(tmp, sizeof tmp, "%d left, ",
					 musb_readl(base,
						    DAVINCI_RXCPPI_BUFCNT0_REG +
						    off1));
			}

			code = snprintf(buf, max, "%cX DMA%d: %s"
					"%08x %08x, %08x %08x; "
					"%08x %08x %08x .. %08x\n",
					ep->is_in ? 'T' : 'R',
					ep->bEndNumber - 1, tmp,
					musb_readl(ram, 0 * 4),
					musb_readl(ram, 1 * 4),
					musb_readl(ram, 2 * 4),
					musb_readl(ram, 3 * 4),
					musb_readl(ram, 4 * 4),
					musb_readl(ram, 5 * 4),
					musb_readl(ram, 6 * 4),
					musb_readl(ram, 7 * 4));
			if (code < 0)
				break;
			code = min(code, (int)max);
			buf += code;
			max -= code;
		}
#endif

		if (list_empty(&ep->req_list)) {
			code = snprintf(buf, max, "\t(queue empty)\n");
			if (code < 0)
				break;
			code = min(code, (int)max);
			buf += code;
			max -= code;
			break;
		}
		list_for_each_entry(req, &ep->req_list, list) {
			code = snprintf(buf, max, "\treq %p, %s%s%d/%d\n",
					req,
					req->zero ? "zero, " : "",
					req->short_not_ok ? "!short, " : "",
					req->actual, req->length);
			if (code < 0)
				break;
			code = min(code, (int)max);
			buf += code;
			max -= code;
		}
	} while (0);
	return (buf > buffer) ? (buf - buffer) : code;
}
#endif

static int
dump_end_info(struct musb *pThis, u8 bEnd, char *aBuffer, unsigned max)
{
	int code = 0;
	char *buf = aBuffer;
	struct musb_hw_ep *pEnd = &pThis->aLocalEnd[bEnd];

	bEnd = pEnd->bLocalEnd;
	do {
		MGC_SelectEnd(pThis->pRegs, bEnd);
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		if (is_host_active(pThis)) {
			int dump_rx, dump_tx;

			/* TEMPORARY (!) until host handles both
			 * directions of the hardware
			 */
			if (pEnd->bIsSharedFifo) {
				/* control is shared, uses RX queue
				 * but (mostly) shadowed tx registers
				 */
				if (!bEnd)
					dump_tx = 1;
				else {
					if (!pEnd->bIsClaimed)
						break;

					/* presumably interrupt-IN...
					 * THIS IS A GUESS
					 * (could look at the queue)
					 */
					dump_tx = 0;
				}

				dump_rx = !dump_tx;

			} else if (pEnd == pThis->bulk_tx_end) {
				dump_tx = 1;
				dump_rx = 0;
			} else if (pEnd == pThis->bulk_rx_end) {
				dump_tx = 0;
				dump_rx = 1;
			} else {
				dump_rx = pEnd->wMaxPacketSizeRx;
				dump_tx = pEnd->wMaxPacketSizeTx;
			}
			/* END TEMPORARY */

			/* FIXME for rx and tx dump hardware fifo and
			 * double-buffer flags ... and make register and stat
			 * dumps (mostly) usable on the peripheral side too
			 */
			if (dump_rx) {
				code = snprintf(buf, max,
						"\nEnd-%d:  rxcsr %04x interval %02x "
						"max %04x type %02x; "
						"dev %d hub %d port %d"
						"\n",
						bEnd,
						MGC_ReadCsr16(pThis->pRegs,
							      MGC_O_HDRC_RXCSR,
							      bEnd),
						MGC_ReadCsr8(pThis->pRegs,
							     MGC_O_HDRC_RXINTERVAL,
							     bEnd),
						MGC_ReadCsr16(pThis->pRegs,
							      MGC_O_HDRC_RXMAXP,
							      bEnd),
						MGC_ReadCsr8(pThis->pRegs,
							     MGC_O_HDRC_RXTYPE,
							     bEnd),
						/* FIXME:  assumes multipoint */
						musb_readb(pThis->pRegs,
							   MGC_BUSCTL_OFFSET
							   (bEnd,
							    MGC_O_HDRC_RXFUNCADDR)),
						musb_readb(pThis->pRegs,
							   MGC_BUSCTL_OFFSET
							   (bEnd,
							    MGC_O_HDRC_RXHUBADDR)),
						musb_readb(pThis->pRegs,
							   MGC_BUSCTL_OFFSET
							   (bEnd,
							    MGC_O_HDRC_RXHUBPORT))
				    );
				if (code < 0)
					break;
				code = min(code, (int)max);
				buf += code;
				max -= code;

#ifdef	CONFIG_USB_TI_CPPI_DMA
				if (bEnd && pEnd->pDmaChannel) {
					unsigned cppi = bEnd - 1;
					unsigned off1 = cppi << 2;
					void __iomem *base;
					void __iomem *ram;
					char tmp[16];

					base = pThis->ctrl_base;
					ram = base +
					    DAVINCI_RXCPPI_STATERAM_OFFSET
					    (cppi);
					snprintf(tmp, sizeof tmp, "%d left, ",
						 musb_readl(base,
							    DAVINCI_RXCPPI_BUFCNT0_REG
							    + off1));

					code = snprintf(buf, max,
							"    rx dma%d: %s"
							"%08x %08x, %08x %08x; "
							"%08x %08x %08x .. %08x\n",
							cppi, tmp,
							musb_readl(ram, 0 * 4),
							musb_readl(ram, 1 * 4),
							musb_readl(ram, 2 * 4),
							musb_readl(ram, 3 * 4),
							musb_readl(ram, 4 * 4),
							musb_readl(ram, 5 * 4),
							musb_readl(ram, 6 * 4),
							musb_readl(ram, 7 * 4));
					if (code < 0)
						break;
					code = min(code, (int)max);
					buf += code;
					max -= code;
				}
#endif
				code = dump_urbs(&pEnd->urb_list, buf, max);
				if (code < 0)
					break;
				code = min(code, (int)max);
				buf += code;
				max -= code;
			}

			if (dump_tx) {
				code = snprintf(buf, max,
						"End-%d:  txcsr %04x interval %02x "
						"max %04x type %02x; "
						"dev %d hub %d port %d"
						"\n",
						bEnd,
						MGC_ReadCsr16(pThis->pRegs,
							      MGC_O_HDRC_TXCSR,
							      bEnd),
						MGC_ReadCsr8(pThis->pRegs,
							     MGC_O_HDRC_TXINTERVAL,
							     bEnd),
						MGC_ReadCsr16(pThis->pRegs,
							      MGC_O_HDRC_TXMAXP,
							      bEnd),
						MGC_ReadCsr8(pThis->pRegs,
							     MGC_O_HDRC_TXTYPE,
							     bEnd),
						/* FIXME:  assumes multipoint */
						musb_readb(pThis->pRegs,
							   MGC_BUSCTL_OFFSET
							   (bEnd,
							    MGC_O_HDRC_TXFUNCADDR)),
						musb_readb(pThis->pRegs,
							   MGC_BUSCTL_OFFSET
							   (bEnd,
							    MGC_O_HDRC_TXHUBADDR)),
						musb_readb(pThis->pRegs,
							   MGC_BUSCTL_OFFSET
							   (bEnd,
							    MGC_O_HDRC_TXHUBPORT))
				    );
				if (code < 0)
					break;
				code = min(code, (int)max);
				buf += code;
				max -= code;
#ifdef	CONFIG_USB_TI_CPPI_DMA
				if (bEnd && pEnd->pDmaChannel) {
					unsigned cppi = bEnd - 1;
					void __iomem *base;
					void __iomem *ram;

					base = pThis->ctrl_base;
					ram = base +
					    DAVINCI_TXCPPI_STATERAM_OFFSET
					    (cppi);
					code =
					    snprintf(buf, max,
						     "    tx dma%d: "
						     "%08x %08x, %08x %08x; "
						     "%08x %08x %08x .. %08x\n",
						     cppi, musb_readl(ram,
								      0 * 4),
						     musb_readl(ram, 1 * 4),
						     musb_readl(ram, 2 * 4),
						     musb_readl(ram, 3 * 4),
						     musb_readl(ram, 4 * 4),
						     musb_readl(ram, 5 * 4),
						     musb_readl(ram, 6 * 4),
						     musb_readl(ram, 7 * 4));
					if (code < 0)
						break;
					code = min(code, (int)max);
					buf += code;
					max -= code;
				}
#endif
				code = dump_urbs(&pEnd->urb_list, buf, max);
				if (code < 0)
					break;
				code = min(code, (int)max);
				buf += code;
				max -= code;
			}
		}
#endif
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		if (is_peripheral_active(pThis)) {
			code = 0;

			if (pEnd->ep_in.desc || !bEnd) {
				code = dump_ep(&pEnd->ep_in, buf, max);
				if (code < 0)
					break;
				code = min(code, (int)max);
				buf += code;
				max -= code;
			}
			if (pEnd->ep_out.desc) {
				code = dump_ep(&pEnd->ep_out, buf, max);
				if (code < 0)
					break;
				code = min(code, (int)max);
				buf += code;
				max -= code;
			}
		}
#endif
	} while (0);

	return buf - aBuffer;
}

/** Dump the current status and compile options.
 * @param pThis the device driver instance
 * @param buffer where to dump the status; it must be big enough hold the
 * result otherwise "BAD THINGS HAPPENS(TM)".
 */
static int dump_header_stats(struct musb *pThis, char *buffer)
{
	int code, count = 0;
	const void __iomem *pBase = pThis->pRegs;

	*buffer = 0;
	count = sprintf(buffer, "Status: %sHDRC, Mode=%s "
			"(Power=%02x, DevCtl=%02x)\tPDR interupt Mask=%x\t PDR Interrypt Source=%x\n",
			(pThis->bIsMultipoint ? "M" : ""), MUSB_MODE(pThis),
			musb_readb(pBase, MGC_O_HDRC_POWER),
			musb_readb(pBase, MGC_O_HDRC_DEVCTL),
			musb_readl(pThis->ctrl_base,
				   DAVINCI_USB_INT_MASK_SET_REG),
			musb_readl(pThis->ctrl_base, DAVINCI_USB_INT_SET_REG));
	if (count < 0)
		return count;
	buffer += count;

#ifdef CONFIG_USB_MUSB_OTG
	code = sprintf(buffer, "OTG state: %s \n",
		       state_string(pThis->OtgMachine.xceiv.state));
	if (code < 0)
		return code;
	buffer += code;
	count += code;
#endif

	code = sprintf(buffer, "Options: "
#ifdef CONFIG_USB_INVENTRA_FIFO
		       "[pio]"
#elif defined(CONFIG_USB_TI_CPPI_DMA)
		       "[cppi-dma]"
#elif defined(CONFIG_USB_INVENTRA_DMA)
		       "[musb-dma]"
#else
		       "[?]"
#endif
		       " "
#ifdef CONFIG_USB_MUSB_OTG
		       "[otg: peripheral+host]"
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
		       "[peripheral]"
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
		       "[host]"
#endif
		       " [debug=%d] [eps=%d]\n",
		       MGC_GetDebugLevel(), pThis->bEndCount);
	if (code < 0)
		return code;
	count += code;
	buffer += code;

#if	defined (CONFIG_ARCH_DAVINCI)
	code = sprintf(buffer,
		       "DaVinci: ctrl=%02x stat=%1x phy=%03x\n"
		       "\trndis=%05x auto=%04x intsrc=%08x intmsk=%08x"
		       "\n",
		       musb_readl(pThis->ctrl_base, DAVINCI_USB_CTRL_REG),
		       musb_readl(pThis->ctrl_base, DAVINCI_USB_STAT_REG),
#ifdef CONFIG_ARCH_DAVINCI
		       __raw_readl(IO_ADDRESS(USBPHY_CTL_PADDR)),
#endif
		       musb_readl(pThis->ctrl_base, DAVINCI_RNDIS_REG),
		       musb_readl(pThis->ctrl_base, DAVINCI_AUTOREQ_REG),
		       musb_readl(pThis->ctrl_base,
				  DAVINCI_USB_INT_SOURCE_REG),
		       musb_readl(pThis->ctrl_base, DAVINCI_USB_INT_MASK_REG));
	if (code < 0)
		return count;
	count += code;
	buffer += code;
#endif				/* DAVINCI */

#ifdef	CONFIG_USB_TI_CPPI_DMA
	code = sprintf(buffer,
		       "CPPI: txcr=%d txsrc=%01x txena=%01x; "
		       "rxcr=%d rxsrc=%01x rxena=%01x "
		       "\n",
		       musb_readl(pThis->ctrl_base, DAVINCI_TXCPPI_CTRL_REG),
		       musb_readl(pThis->ctrl_base, DAVINCI_TXCPPI_RAW_REG),
		       musb_readl(pThis->ctrl_base,
				  DAVINCI_TXCPPI_INTENAB_REG),
		       musb_readl(pThis->ctrl_base, DAVINCI_RXCPPI_CTRL_REG),
		       musb_readl(pThis->ctrl_base, DAVINCI_RXCPPI_RAW_REG),
		       musb_readl(pThis->ctrl_base,
				  DAVINCI_RXCPPI_INTENAB_REG));
	if (code < 0)
		return count;
	count += code;
	buffer += code;
#endif				/* CPPI */

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if (is_peripheral_enabled(pThis)) {
		code = sprintf(buffer, "Gadget driver: %s\n",
			       pThis->pGadgetDriver
			       ? pThis->pGadgetDriver->driver.name : "(none)");
		if (code < 0)
			return code;
		count += code;
		buffer += code;
	}
#endif

	return count;
}

/* Write to ProcFS
 *
 * C soft-connect
 * c soft-disconnect
 * I enable HS
 * i disable HS
 * s stop session
 * F force session (OTG-unfriendly)
 * E rElinquish bus (OTG)
 * H request host mode
 * h cancel host request
 * D<num> set/query the debug level
 */
static int musb_proc_write(struct file *file, const char __user * buffer,
			   unsigned long count, void *data)
{
	char cmd;
	u8 bReg;
	void __iomem *pBase = ((struct musb *)data)->pRegs;

	/* MOD_INC_USE_COUNT; */

	copy_from_user(&cmd, buffer, 1);
	switch (cmd) {
	case 'C':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER) |
			    MGC_M_POWER_SOFTCONN;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'c':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER) &
			    ~MGC_M_POWER_SOFTCONN;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'I':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER) |
			    MGC_M_POWER_HSENAB;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'i':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER) &
			    ~MGC_M_POWER_HSENAB;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'F':
		bReg = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
		bReg |= MGC_M_DEVCTL_SESSION;
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL, bReg);
		break;

	case 'H':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
			bReg |= MGC_M_DEVCTL_HR;
			musb_writeb(pBase, MGC_O_HDRC_DEVCTL, bReg);
			//MUSB_HST_MODE( ((struct musb*)data) );
			//WARN("Host Mode\n");
		}
		break;

	case 'h':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
			bReg &= ~MGC_M_DEVCTL_HR;
			musb_writeb(pBase, MGC_O_HDRC_DEVCTL, bReg);
		}
		break;

#if (MUSB_DEBUG>0)
		/* set/read debug level */
	case 'D':{
			if (count > 1) {
				char digits[8], *p = digits;
				int i = 0, level = 0, sign = 1, len =
				    min(count - 1, (unsigned long)8);

				copy_from_user(&digits, &buffer[1], len);

				/* optional sign */
				if (*p == '-') {
					len -= 1;
					sign = -sign;
					p++;
				}

				/* read it */
				while (i++ < len && *p > '0' && *p < '9') {
					level = level * 10 + (*p - '0');
					p++;
				}

				level *= sign;
				DBG(1, "debug level %d\n", level);
				MGC_SetDebugLevel(level);
			}
		}
		break;

	case '?':
		INFO("?: you are seeing it\n");
		INFO("C/c: soft connect enable/disable\n");
		INFO("I/i: hispeed enable/disable\n");
		INFO("F: force session start\n");
		INFO("H: host mode\n");
		INFO("D: set/read dbug level\n");
		break;
#endif

	default:
		ERR("Command %c not implemented\n", cmd);
		break;
	}

	return count;
}

static int musb_proc_read(char *page, char **start,
			  off_t off, int count, int *eof, void *data)
{
	char *buffer = page;
	int code = 0;
	unsigned long flags;
	struct musb *pThis = data;
	unsigned bEnd;

	count -= off;
	count -= 1;		/* for NUL at end */
	if (count < 0)
		return -EINVAL;

	spin_lock_irqsave(&pThis->Lock, flags);

	code = dump_header_stats(pThis, buffer);
	if (code > 0) {
		buffer += code;
		count -= code;
	}

	/* generate the report for the end points */
	// REVISIT ... not unless something's connected!
	for (bEnd = 0; count >= 0 && bEnd < pThis->bEndCount; bEnd++) {
		code = dump_end_info(pThis, bEnd, buffer, count);
		if (code > 0) {
			buffer += code;
			count -= code;
		}
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);
	*eof = 1;
	return (buffer - page) - off;
}

void __exit musb_debug_delete(char *name, struct musb *musb)
{
	if (musb->pProcEntry)
		remove_proc_entry(name, NULL);
}

struct proc_dir_entry *__init musb_debug_create(char *name, struct musb *data)
{
	struct proc_dir_entry *pde;

	/* FIXME convert everything to seq_file; then later, debugfs */

	if (!name)
		return NULL;

	data->pProcEntry = pde = create_proc_entry(name,
						   S_IFREG | S_IRUGO | S_IWUSR,
						   NULL);
	if (pde) {
		pde->data = data;
		// pde->owner = THIS_MODULE;

		pde->read_proc = musb_proc_read;
		pde->write_proc = musb_proc_write;

		pde->size = 0;

		pr_debug("Registered /proc/%s\n", name);
	} else {
		pr_debug("Cannot create a valid proc file entry");
	}

	return pde;
}
