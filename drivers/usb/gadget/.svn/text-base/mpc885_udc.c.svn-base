/*
 * Driver for the MPC885 on-chip peripheral USB device controller.
 * Specs and errata are available from <http://www.freescale.com>.
 *
 * This driver should work well with Serial "gadget" driver.
 *
 * Author: Gennadiy Kurtsman <source@mvista.com>
 *
 * Based on:
 * Montavista Pro 3.1 8xx USB-based tty device driver, by Yuri Shpilevsly
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>

#include <asm/byteorder.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/8xx_immap.h>
#include <asm/commproc.h>
#include "mpc885_udc.h"

static struct _mpc885_udc_dev *the_controller = NULL;
static struct _mpc885_udc_dev *the_controller_pa = NULL;

static const char driver_name[] = "mpc885_udc, 2006, march 27";

#define EP0_NAME "ep0"
#define EP_IN_NAME "ep-in"
#define EP_OUT_NAME "ep-out"

static const char *ep_name[] = {
	EP0_NAME,
	EP_IN_NAME,
	EP_OUT_NAME,
};

/* Parse standard setup request */
static inline int
mpc885_usbc_setup(struct mpc885_usbc_ep *ep, __u8 * data, int len,
		  unsigned long flags)
{
	struct _mpc885_udc_dev *usbc_dev = ep->dev;
	struct usb_ctrlrequest *cmd =
	    (struct usb_ctrlrequest *)(data - ep->rx_pool + ep->rx_pool_va);
	__u16 typeReq, wValue, wIndex, wLength;
	int ret = -EINVAL;
	int handshake;

	if (len != 8)
		return ret;
	typeReq = (cmd->bRequestType << 8) | cmd->bRequest;
	wValue = le16_to_cpu(cmd->wValue);
	wIndex = le16_to_cpu(cmd->wIndex);
	wLength = le16_to_cpu(cmd->wLength);
	handshake = 1;

	data = usbc_dev->ctrl_data_va;

	switch (typeReq) {
	case DeviceRequest | USB_REQ_GET_STATUS:
	case EndpointRequest | USB_REQ_GET_STATUS:
		data[0] = 0;
		data[1] = 0;
		len = 2;
		ret = 0;
		break;

	case DeviceOutRequest | USB_REQ_SET_ADDRESS:
		/* Assign address */
		usbc_dev->address = (__u8) wValue;
		len = 0;
		ret = 0;
		handshake = 0;
		break;

	default:
		spin_unlock_irqrestore(&ep->rxirq_lock, flags);
		if (cmd->bRequest == USB_REQ_SET_CONFIGURATION) {
			/* according to USB Spec */
			usbc_dev->eps[1].pid = BD_USB_PID0;
			usbc_dev->eps[2].pid = BD_USB_RXPID_0;
		}
		ret = usbc_dev->driver->setup(&usbc_dev->gadget, cmd);
		if (ret >= 0)
			len = usbc_dev->ctrl_length;
		spin_lock_irqsave(&ep->rxirq_lock, flags);
	}
	if (ret < 0)
		return ret;
	else {
		if (len > wLength)
			len = wLength;
		/* this for setup packet must start from DATA1 */
		ep->pid = BD_USB_PID1;
		if ((cmd->bRequest == USB_REQ_GET_DESCRIPTOR)
		    && ((wValue >> 8) == USB_DT_DEVICE)) {
			((struct usb_device_descriptor *)data)->
			    bMaxPacketSize0 = UDC_USBC_CTL_PACKET_SIZE;
			if (wLength > 8) {
				((struct usb_device_descriptor *)data)->
				    idVendor = cpu_to_le16(0x6655);
				((struct usb_device_descriptor *)data)->
				    idProduct = cpu_to_le16(0xffff);
			}
		}
		if ((cmd->bRequest == USB_REQ_GET_DESCRIPTOR) &&
		    ((wValue >> 8) == USB_DT_CONFIG) &&
		    (len == sizeof(struct usb_config_descriptor) +
		     sizeof(struct usb_interface_descriptor) +
		     (sizeof(struct usb_endpoint_descriptor) -
		      2) * (UDC_USBC_MAX_EPS - 1))) {
			struct usb_endpoint_descriptor *ed =
			    (struct usb_endpoint_descriptor *)(data +
							       sizeof(struct
								      usb_config_descriptor)
							       +
							       sizeof(struct
								      usb_interface_descriptor));
			int i;

			for (i = 0; i < (UDC_USBC_MAX_EPS - 1); i++) {
				ed->wMaxPacketSize =
				    cpu_to_le16(UDC_USBC_PACKET_SIZE);
				ed = (struct usb_endpoint_descriptor *)((u8 *)
									ed +
									sizeof
									(struct
									 usb_endpoint_descriptor)
									- 2);
			}
		}
		mpc885_usbc_tx_data(ep, usbc_dev->ctrl_data, len);
	}
	return ret;
}

/* Reset Interrupt handler */
static void
mpc885_usbc_reset_isr(struct _mpc885_udc_dev *usbc_dev)
{
	int i;

	for (i = 0; i < UDC_USBC_MAX_EPS; i++) {
		mpc885_usbc_tx_recover(&usbc_dev->eps[i]);
	}
	usbc_dev->usb_prms->frame_n = 0;
	usbc_dev->usb_rgs->usadr = 0;
}

static irqreturn_t
mpc885_usbc_isr(int irq, void *_dev, struct pt_regs *r)
{
	struct _mpc885_udc_dev *usbc_dev = _dev;
	__u16 usb_usber = usbc_dev->usb_rgs->usber;
	int i;

	/* Confirm */
	usbc_dev->usb_rgs->usber = usb_usber;

	if ((usb_usber & USBE_RXB)) {
		/* Handle receive */
		for (i = 0; i < UDC_USBC_MAX_EPS; i++)
			mpc885_usbc_rx_isr(usbc_dev, i);
	}
	if ((usb_usber & USBE_TXB)) {
		/* Handle transmit confirmation */
		for (i = 0; i < UDC_USBC_MAX_EPS; i++)
			mpc885_usbc_tx_isr(usbc_dev, i);
	}
	if ((usb_usber & USBE_RESET)) {
		/* Reset. Clear address and kill all transactions in progress */
		mpc885_usbc_reset_isr(usbc_dev);
	}
	if ((usb_usber & USBE_ERROR)) {
		if ((usb_usber &
		     (USBE_TXE0 | USBE_TXE1 | USBE_TXE2 | USBE_TXE3))) ;
	}

	return IRQ_HANDLED;
}

static void
mpc885_usbc_kick_tx(struct _mpc885_udc_dev *usbc_dev)
{

	struct mpc885_usbc_ep *ep = &usbc_dev->eps[1];	/* bulk-in */
	int len = __mpc885_usbc_buf_len_flat(&ep->dev->write_buf);
	unsigned long flags;

	if (!len)
		return;
	if (len > UDC_USBC_PACKET_SIZE)
		len = UDC_USBC_PACKET_SIZE;
	spin_lock_irqsave(&ep->txbd_lock, flags);
	mpc885_usbc_tx_data(ep,
			    ep->dev->write_buf.buffer_pa +
			    ep->dev->write_buf.read_pos, len);
	spin_unlock_irqrestore(&ep->txbd_lock, flags);
}

/* EP0 0 control receive callback */
static int
mpc885_usbc_rx_ctrl(struct mpc885_usbc_ep *ep, int pid, __u8 * data, __u16 len,
		    int is_error, unsigned long flags)
{
	int ret;

	if (is_error)
		return 0;

	if (!len)
		return 0;	/* It was a setup status. No action required */

	/* Current packet is rejected, new one will be initiated in mpc885_usbc_setup() */
	if (ep->data) {
		mpc885_usbc_tx_recover(ep);
	}

	if (pid != BD_USB_RXPID_SETUP)
		ret = -EINVAL;
	else
		ret = mpc885_usbc_setup(ep, data, len, flags);

	if (ret)
		mpc885_usbc_tx_handshake(ep, USB_PID_STALL);

	return 0;
}

/* EP0 1 - bulk in receive callback */
static int
mpc885_usbc_rx_data_in(struct mpc885_usbc_ep *ep, int pid, __u8 * data,
		       __u16 len, int is_error, unsigned long flags)
{
	/* We don't expect to receive anything, but IN token on this endpoint */
	return 0;
}

/* EP0 2 - bulk out receive callback */
static int
mpc885_usbc_rx_data_out(struct mpc885_usbc_ep *ep, int pid, __u8 * data,
			__u16 len, int is_error, unsigned long flags)
{
	int room_len;
	struct mpc885_request *req;
	int count;
	struct _mpc885_udc_dev *usbc_dev = (struct _mpc885_udc_dev *)ep->dev;

	mpc885_usbc_tx_handshake(ep, USB_PID_NAK);
	if (is_error)
		return 0;

	/* If pid is not what expected - do nothing */
	if (pid != ep->pid) {
		return 0;
	}

	ep->pid = (pid == BD_USB_RXPID_0) ? BD_USB_RXPID_1 : BD_USB_RXPID_0;

	room_len = __mpc885_usbc_buf_room(&ep->dev->read_buf);
	if (room_len < len)
		return 1;	/* no room */
	__mpc885_usbc_buf_write(&ep->dev->read_buf,
				data + (u32) (ep->rx_pool_va) -
				(u32) (ep->rx_pool), len);
	count = __mpc885_usbc_buf_len(&usbc_dev->read_buf);
	if (!count)
		return 0;
	if (list_empty(&ep->rqueue))
		return 0;
	req = list_entry(ep->rqueue.next, struct mpc885_request, queue);
	list_del_init(&req->queue);
	count =
	    __mpc885_usbc_buf_read(&usbc_dev->read_buf, req->req.buf,
				   req->req.length);
	req->req.status = 0;
	req->req.actual = count;
	spin_lock_irqsave(&ep->up_complete_lock, flags);
	req->req.complete(&ep->ep, &req->req);
	spin_unlock_irqrestore(&ep->up_complete_lock, flags);

	return 0;
}

/* EP0 0 control transmit confirmation callback */
static void
mpc885_usbc_txconf_ctrl(struct mpc885_usbc_ep *ep, __u16 len, int is_error)
{
	if (is_error)
		return;
	if (ep->dev->address) {	/* Set address pending */
		ep->dev->usb_rgs->usadr = ep->dev->address;
		ep->dev->address = 0;
	}
	ep->ack_len += len;
	BUG_ON(ep->ack_len > ep->data_len);
	if (ep->ack_len < ep->data_len) {
		ep->pid = (ep->pid == BD_USB_PID1) ? BD_USB_PID0 : BD_USB_PID1;
		mpc885_usbc_tx(ep, &ep->data[ep->ack_len],
			       ep->data_len - ep->ack_len,
			       BD_USB_TC | BD_USB_CNF | BD_SC_INTRPT);
	} else
		/* All done */
		ep->data = NULL;
	return;
}

/* EP0 1 - bulk in transmit confirmation callback */
static void
mpc885_usbc_txconf_data_in(struct mpc885_usbc_ep *ep, __u16 len, int is_error)
{
	struct mpc885_request *req = NULL;
	int count;
	struct _mpc885_udc_dev *usbc_dev = (struct _mpc885_udc_dev *)ep->dev;
	int len_to_resp;
	unsigned long flags;

	if (!is_error) {
		BUG_ON(!len || (len >= MPC885_USBC_BUF_SIZE));
		__mpc885_usbc_buf_inc_read(&ep->dev->write_buf, len);
		for (len_to_resp = len + ep->len_to_req; len_to_resp;) {
			if (list_empty(&ep->squeue)) {
				len_to_resp = 0;
				break;
			}
			req =
			    list_entry(ep->squeue.next, struct mpc885_request,
				       queue);
			if (req->req.length > len_to_resp)
				break;
			list_del_init(&req->queue);
			req->req.status = 0;
			req->req.actual = req->req.length;
			len_to_resp -= req->req.length;
			spin_lock_irqsave(&ep->up_complete_lock, flags);
			req->req.complete(&ep->ep, &(req->req));
			spin_unlock_irqrestore(&ep->up_complete_lock,
						    flags);
		}
		ep->len_to_req = len_to_resp;
		ep->pid = (ep->pid == BD_USB_PID0) ? BD_USB_PID1 : BD_USB_PID0;
		while (!(list_empty(&ep->rqueue))) {
			req =
			    list_entry(ep->rqueue.next, struct mpc885_request,
				       queue);
			count = __mpc885_usbc_buf_room(&usbc_dev->write_buf);
			if (count > req->req.length) {
				list_del_init(&req->queue);
				list_add_tail(&req->queue, &ep->squeue);
				count =
				    __mpc885_usbc_buf_write(&usbc_dev->
							    write_buf,
							    req->req.buf,
							    req->req.length);
				req->req.status = -EINPROGRESS;
				req->req.actual = 0;
			} else
				break;
		}
	}
	mpc885_usbc_kick_tx(ep->dev);
}

/* EP0 2 - bulk out transmit confirmation callback */
static void
mpc885_usbc_txconf_data_out(struct mpc885_usbc_ep *ep, __u16 len, int is_error)
{
	return;
}

static mpc885_usbc_rx_callback rx_cb[] = {
	mpc885_usbc_rx_ctrl,
	mpc885_usbc_rx_data_in,
	mpc885_usbc_rx_data_out,
};

static mpc885_usbc_txconf_callback txconf_cb[] = {
	mpc885_usbc_txconf_ctrl,
	mpc885_usbc_txconf_data_in,
	mpc885_usbc_txconf_data_out,
};

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int
usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct _mpc885_udc_dev *usbc_dev = the_controller;
	int retval;
	if (!driver
	    || !driver->bind
	    || !driver->unbind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!usbc_dev)
		return -ENODEV;
	if (usbc_dev->driver)
		return -EBUSY;

	/* hook up the driver */
	driver->driver.bus = NULL;
	usbc_dev->driver = driver;
	usbc_dev->gadget.dev.driver = &driver->driver;
	retval = driver->bind(&usbc_dev->gadget);
	if (retval) {
		usbc_dev->driver = NULL;
		usbc_dev->gadget.dev.driver = NULL;
		return retval;
	}
	/* then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 */

	return 0;
}

int
usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct _mpc885_udc_dev *dev = the_controller;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	driver->unbind(&dev->gadget);
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;

	return 0;
}

static int
mpc885_disable(struct usb_ep *ep)
{
	return 0;
}

static int
mpc885_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct _mpc885_udc_dev *dev;
	struct mpc885_usbc_ep *ep;
	u16 max;
	unsigned long flags = 0;

	ep = container_of(_ep, struct mpc885_usbc_ep, ep);
	if (!_ep || !desc || ep->desc
	    || desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	dev = ep->dev;
	if (ep == &dev->eps[0])
		return -EINVAL;
	if (!dev->driver)
		return -ESHUTDOWN;
	if (ep->ep_num != (desc->bEndpointAddress & 0x0f))
		return -EINVAL;

	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		break;
	default:
		return -EINVAL;
	}

	/* enabling the no-toggle interrupt mode would need an api hook */

	max = UDC_USBC_PACKET_SIZE;
	if (!max)
		return -EINVAL;

	spin_lock_irqsave(&ep->enable_lock,flags);

	ep->ep.maxpacket = max;
	ep->desc = (struct usb_endpoint_descriptor *)desc;
	spin_unlock_irqrestore(&ep->enable_lock, flags);
	return 0;
};

static int
mpc885_queue(struct usb_ep *_ep, struct usb_request *_req, int gfp_flags)
{
	struct mpc885_request *req;
	struct mpc885_usbc_ep *ep;
	struct _mpc885_udc_dev *usbc_dev;
	unsigned long flags = 0;
	int count;

	/* always require a cpu-view buffer so pio works */
	if (unlikely(!_req || !_req->complete || !_req->buf))
		return -EINVAL;
	if (unlikely(!_ep))
		return -EINVAL;

	req = container_of(_req, struct mpc885_request, req);
	ep = (struct mpc885_usbc_ep *)container_of(_ep, struct mpc885_usbc_ep,
						   ep);
	usbc_dev = ep->dev;
	if (!strcmp(_ep->name, EP0_NAME)) {
		memcpy(usbc_dev->ctrl_data_va, _req->buf, _req->length);
		usbc_dev->ctrl_length = _req->length;
		_req->status = 0;
		_req->actual = _req->length;
		spin_lock_irqsave(&ep->up_complete_lock,flags);
		_req->complete(&ep->ep, _req);
		spin_unlock_irqrestore(&ep->up_complete_lock, flags);
	}
	if (!strcmp(_ep->name, EP_IN_NAME)) {
		spin_lock_irqsave(&ep->txreq_lock, flags);
		req->req.status = -EINPROGRESS;
		req->req.actual = 0;
		list_add_tail(&req->queue, &ep->rqueue);
		req = list_entry(ep->rqueue.next, struct mpc885_request, queue);
		count = __mpc885_usbc_buf_room(&usbc_dev->write_buf);
		if ((count > req->req.length)) {
			count =
			    __mpc885_usbc_buf_write(&usbc_dev->write_buf,
						    req->req.buf,
						    req->req.length);
			list_del_init(&req->queue);
			list_add_tail(&req->queue, &ep->squeue);
			if ((count =
			     __mpc885_usbc_buf_room(&usbc_dev->write_buf)) ==
			    (usbc_dev->write_buf.size - req->req.length) - 1) {
				mpc885_usbc_kick_tx(usbc_dev);
			}
		}
		spin_unlock_irqrestore(&ep->txreq_lock, flags);
	}
	if (!strcmp(_ep->name, EP_OUT_NAME)) {
		spin_lock_irqsave(&ep->rxreq_lock, flags);
		count = __mpc885_usbc_buf_len(&usbc_dev->read_buf);
		if (!count) {
			_req->status = -EINPROGRESS;
			_req->actual = 0;
			list_add_tail(&req->queue, &ep->rqueue);
			spin_unlock_irqrestore(&ep->rxreq_lock, flags);
		} else {
			count =
			    __mpc885_usbc_buf_read(&usbc_dev->read_buf,
						   _req->buf, _req->length);
			spin_unlock_irqrestore(&ep->rxreq_lock, flags);
			_req->status = 0;
			_req->actual = count;
			spin_lock_irqsave(&ep->up_complete_lock, flags);
			_req->complete(&ep->ep, _req);
			spin_unlock_irqrestore(&ep->up_complete_lock,
						    flags);
		}
	}

	return 0;
}

static struct usb_request*
mpc885_alloc_request(struct usb_ep *_ep,
						int gfp_flags)
{
	struct mpc885_request *req;

	if (!_ep)
		return NULL;
	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void
mpc885_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct mpc885_request *req;

	if (!_ep || !_req)
		return;

	req = container_of(_req, struct mpc885_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static struct usb_ep_ops mpc885_ep_ops = {
	.enable = mpc885_enable,
	.disable = mpc885_disable,
	.alloc_request = mpc885_alloc_request,
	.free_request = mpc885_free_request,
	.queue = mpc885_queue,
};

MODULE_DESCRIPTION("MPC885_ADS USB Peripheral Controller");
MODULE_AUTHOR("Gennadiy Kurtsman");
MODULE_LICENSE("GPL");

static int
mpc885_udc_get_frame(struct usb_gadget *_gadget)
{
	return 0;
}

static const struct usb_gadget_ops mpc885_udc_ops = {
	.get_frame = mpc885_udc_get_frame,
};

static int
mpc885_usbc_hw_ep_init(struct _mpc885_udc_dev *usbc_dev, int ep_idx,
		       int ep_num, __u16 ep_type, int max_len)
{
	struct mpc885_usbc_ep *ep;
	usb_ep_parms_t *ep_parms;
	__u16 ep_parms_offset;
	__u16 *p_usep_reg;
	__u16 usep_reg;

	if (ep_idx >= UDC_USBC_MAX_EPS)
		return -EINVAL;

	ep = &(usbc_dev->eps[ep_idx]);
	ep->ep.name = ep_name[ep_idx];
	if (ep_idx)
		list_add_tail(&usbc_dev->eps[ep_idx].ep.ep_list,
			      &usbc_dev->gadget.ep_list);
	ep->ep.ops = &mpc885_ep_ops;
	ep->ep_idx = ep_idx;
	ep->ep_num = ep_num;
	ep->dev = usbc_dev;
	ep->max_len = max_len;
	p_usep_reg = &usbc_dev->usb_rgs->usep0 + ep_idx;

	/* Allocate memory for buffer descriptors */
	ep->tx_base_offset =
	    cpm_dpalloc(UDC_USBC_TX_BDS * sizeof(mpc885_cpm_bd),
			sizeof(mpc885_cpm_bd));
	if (!(ep->tx_base_offset)) {
		BUG_ON(ep->tx_base == NULL);
		return -ENOMEM;
	}
	ep->tx_base = (mpc885_cpm_bd *) (cpm_dpram_addr(ep->tx_base_offset));
	ep->tx_base_va =
	    ioremap((phys_addr_t) ep->tx_base,
		    UDC_USBC_TX_BDS * sizeof(mpc885_cpm_bd));
	ep->rx_base_offset =
	    cpm_dpalloc(UDC_USBC_RX_BDS * sizeof(mpc885_cpm_bd),
			UDC_USBC_RX_BDS * sizeof(mpc885_cpm_bd));
	if (!(ep->rx_base_offset)) {
		cpm_dpfree(ep->tx_base_offset);
		ep->tx_base_offset = 0;
		ep->tx_base = NULL;
		iounmap(ep->tx_base_va);
		ep->tx_base_va = NULL;
		BUG_ON(ep->rx_base == NULL);
		return -ENOMEM;
	}
	ep->rx_base = (mpc885_cpm_bd *) (cpm_dpram_addr(ep->rx_base_offset));
	ep->rx_base_va =
	    ioremap((phys_addr_t) ep->rx_base,
		    UDC_USBC_RX_BDS * sizeof(mpc885_cpm_bd));
	ep->tx_bd = ep->conf_bd = ep->tx_base;
	ep->tx_bd_va = ep->conf_bd_va = ep->tx_base_va;
	ep->rx_bd = ep->rx_base;
	ep->rx_bd_va = ep->rx_base_va;
	ep_parms_offset = cpm_dpalloc(sizeof(usb_ep_parms_t),
				      sizeof(usb_ep_parms_t));
	if (ep_parms_offset == 0) {
		cpm_dpfree(ep->tx_base_offset);
		ep->tx_base_offset = 0;
		ep->tx_base = NULL;
		iounmap(ep->tx_base_va);
		ep->tx_base_va = NULL;
		cpm_dpfree(ep->rx_base_offset);
		ep->rx_base_offset = 0;
		ep->rx_base = NULL;
		iounmap(ep->rx_base_va);
		ep->rx_base_va = NULL;
		BUG_ON(ep->ep_parms == NULL);
		return -ENOMEM;
	}
	ep_parms = (usb_ep_parms_t *) cpm_dpram_addr(ep_parms_offset);
	ep->ep_parms = ep_parms;
	ep->ep_parms_va =
	    ioremap((phys_addr_t) ep_parms, sizeof(usb_ep_parms_t));
	ep->ep_parms_offset = ep_parms_offset;
	memset((void *)ep->ep_parms_va, 0, sizeof(usb_ep_parms_t));

	*(__u16 *) ((unsigned long)&usbc_dev->usb_prms->ep0_ptr +
		    ep_idx * sizeof(__u16)) =
	    mpc885_dpram_offset((void *)ep_parms);
	ep_parms->rbase = ep_parms->rbptr =
	    mpc885_dpram_offset((void *)ep->rx_base);
	ep_parms->tbase = ep_parms->tbptr =
	    mpc885_dpram_offset((void *)ep->tx_base);

	/* for control and bulk pipe, rx buffer is 8 bytes data and 2 bytes CRC */
	ep_parms->mrblr = max_len + 4;
	ep_parms->tfcr = ep_parms->rfcr = USB_FCR_BO_BE;

	/* Init Tx BD ring */
	{
		memset(ep->tx_base_va, 0,
		       UDC_USBC_TX_BDS * sizeof(mpc885_cpm_bd));
		UDC_BD_STATUS_SET(ep->tx_base + UDC_USBC_TX_BDS - 1,
				  BD_SC_WRAP);
	}
	/* Allocate Rx buffer pool and init Rx BD ring */
	ep->rx_pool_va =
	    dma_alloc_coherent(NULL, UDC_USBC_RX_BDS * (max_len + 4),
			       (dma_addr_t *) & ep->rx_pool, GFP_KERNEL);
	if (!ep->rx_pool_va) {
		cpm_dpfree(ep->tx_base_offset);
		ep->tx_base_offset = 0;
		ep->tx_base = NULL;
		iounmap(ep->tx_base_va);
		ep->tx_base_va = NULL;
		cpm_dpfree(ep->rx_base_offset);
		ep->rx_base_offset = 0;
		ep->rx_base = NULL;
		iounmap(ep->rx_base_va);
		ep->rx_base_va = NULL;
		cpm_dpfree(ep_parms_offset);
		ep_parms_offset = 0;
		iounmap(ep->ep_parms_va);
		ep->ep_parms_va = NULL;
		BUG_ON(ep->ep_parms == NULL);
		return -ENOMEM;
	}
	/* Init Rx BD ring */
	{
		mpc885_cpm_bd *_bd = ep->rx_base_va;
		__u8 *_buf = ep->rx_pool;
		int _i;
		for (_i = 0; _i < UDC_USBC_RX_BDS; _i++) {
			UDC_BD_DATA_SET(_bd, (unsigned long)_buf);
			UDC_BD_LENGTH_SET(_bd, 0);
			UDC_BD_STATUS_SET(_bd, BD_SC_INTRPT | BD_SC_EMPTY);
			_bd++;
			_buf += max_len + 4;
		}
		UDC_BD_STATUS_SET((--_bd),
				  BD_SC_INTRPT | BD_SC_WRAP | BD_SC_EMPTY);
	}

	ep->rx_cb = rx_cb[ep_idx];
	ep->txconf_cb = txconf_cb[ep_idx];

	ep->ep_type = ep_type;
	usep_reg = (ep_num << USEP_NUM_SHIFT) | ep_type;
	/* IT: */
	usep_reg |= USEP_RTE;

	*p_usep_reg = usep_reg;

	mpc885_usbc_cmd(ep, CPM_USBCMD_RESTART_TX);

	return 0;
}

/* Board specific configuration
 */
#ifdef CONFIG_MPC885ADS

/* bits in parallel i/o port registers
 */
#define PA_USB_RXD	((__u16)0x0001)
#define PA_USB_OE	((__u16)0x0002)
#define PA_CLK2		((__u16)0x0200)
#define PC_USB_RXP	((__u16)0x0010)
#define PC_USB_RXN	((__u16)0x0020)
#define PC_USB_TXP	((__u16)0x0100)
#define PC_USB_TXN	((__u16)0x0200)

void
mpc885_usb_iface_config(int enable)
{
	void *__iomem bcsr4_va = ioremap(BCSR4, sizeof(__u32));
	__u32 bcsr4_val;

	bcsr4_val = *((__u32 *) bcsr4_va);

	if (enable) {
		bcsr4_val |= BCSR4_USB_LO_SPD;
		bcsr4_val &= ~BCSR4_USB_FULL_SPD;
		bcsr4_val &= ~BCSR4_USB_EN;
	} else
		bcsr4_val |= BCSR4_USB_EN;

	*((__u32 *) bcsr4_va) = bcsr4_val;
	iounmap(bcsr4_va);
}

void
mpc885_usb_clock_config(immap_t * immap)
{
	cpm8xx_t *cp;

	/* get pointer to Communication Processor */
	cp = ioremap((phys_addr_t) cpmp, 48);

	/* we assume a 48Mhz system clock connected to CLK2 via PA6
	 */
	immap->im_ioport.iop_padir &= ~PA_CLK2;
	immap->im_ioport.iop_papar |= PA_CLK2;

	/* configure Serial Interface clock routing
	 */
	cp->cp_sicr &= (__u32) 0xffffff00;
	cp->cp_sicr |= (__u32) 0x00000028;
	iounmap(cp);
}

void
mpc885_usb_io_pin_config(immap_t * immap)
{

	/* select USBRXD & USBOE
	 */
	immap->im_ioport.iop_padir &= ~(PA_USB_RXD | PA_USB_OE);
	immap->im_ioport.iop_papar |= (PA_USB_RXD | PA_USB_OE);
	immap->im_ioport.iop_paodr &= ~PA_USB_OE;

	/* select USBRXP & USBRXN
	 */
	immap->im_ioport.iop_pcdir &= ~(PC_USB_RXP | PC_USB_RXN);
	immap->im_ioport.iop_pcpar &= ~(PC_USB_RXP | PC_USB_RXN);
	immap->im_ioport.iop_pcso |= (PC_USB_RXP | PC_USB_RXN);

	/* select USBTXP and USBTXN
	 */
	immap->im_ioport.iop_pcdir |= (PC_USB_TXP | PC_USB_TXN);
	immap->im_ioport.iop_pcpar |= (PC_USB_TXP | PC_USB_TXN);
}
#endif

static void
mpc885_usbc_hw_deinit(struct _mpc885_udc_dev *usbc_dev)
{
	int i;

	usbc_dev->usb_rgs->usmod &= ~USMOD_ENABLE;
	usbc_dev->intram->im_cpm.cp_sicr &= ~0x0000003f;
	mpc885_usb_iface_config(0);
	free_irq(CPM_IRQ_OFFSET + CPMVEC_SCC1, usbc_dev);
	for (i = 0; i < ARRAY_SIZE(usbc_dev->eps); i++) {
		if (usbc_dev->eps[i].tx_base_offset) {
			cpm_dpfree(usbc_dev->eps[i].tx_base_offset);
			usbc_dev->eps[i].tx_base_offset = 0;
			iounmap(usbc_dev->eps[i].tx_base_va);
		}
		usbc_dev->eps[i].tx_base_va = NULL;
		usbc_dev->eps[i].tx_base = NULL;
		if (usbc_dev->eps[i].rx_base_offset) {
			cpm_dpfree(usbc_dev->eps[i].rx_base_offset);
			usbc_dev->eps[i].rx_base_offset = 0;
			iounmap(usbc_dev->eps[i].rx_base_va);
		}
		usbc_dev->eps[i].rx_base_va = NULL;
		usbc_dev->eps[i].rx_base = NULL;
		if (usbc_dev->eps[i].ep_parms_offset) {
			cpm_dpfree(usbc_dev->eps[i].ep_parms_offset);
			usbc_dev->eps[i].ep_parms_offset = 0;
			iounmap(usbc_dev->eps[i].ep_parms_va);
		}
		usbc_dev->eps[i].ep_parms_va = NULL;
		usbc_dev->eps[i].ep_parms = NULL;
		if (usbc_dev->eps[i].rx_pool_offset) {
			cpm_dpfree(usbc_dev->eps[i].rx_pool_offset);
			usbc_dev->eps[i].rx_pool_offset = 0;
			iounmap(usbc_dev->eps[i].rx_pool_va);
		}
		usbc_dev->eps[i].rx_pool_va = NULL;
		usbc_dev->eps[i].rx_pool = NULL;
	}
	iounmap(usbc_dev->intram);
}

static int
mpc885_usbc_hw_init(struct _mpc885_udc_dev *usbc_dev)
{
	immap_t *immap;
	usb_parms_t *usb_prms;
	usb_regs_t *usb_rgs;
	int ret;

	/* Get IMMR address, USB params */
	immap = usbc_dev->intram = (immap_t *) ioremap(IMAP_ADDR, IMAP_SIZE);
	usb_prms = usbc_dev->usb_prms =
	    (usb_parms_t *) & immap->im_cpm.cp_dparam;
	usb_rgs = usbc_dev->usb_rgs = (usb_regs_t *) & immap->im_cpm.cp_scc[0];

	mpc885_usb_iface_config(1);
	mpc885_usb_clock_config(immap);
	mpc885_usb_io_pin_config(immap);

	/* Initialize USB parameters RAM and registers */
	memset((void *)usb_prms, 0, sizeof(usb_parms_t));

	/* Initialize endpoints */
	INIT_LIST_HEAD(&usbc_dev->gadget.ep_list);
	usbc_dev->gadget.ep0 = &usbc_dev->eps[0].ep;
	ret = mpc885_usbc_hw_ep_init(usbc_dev, 0, 0, USEP_TM_CTRL,
				     UDC_USBC_CTL_PACKET_SIZE);
	ret = ret ? ret : mpc885_usbc_hw_ep_init(usbc_dev, 1, 1, USEP_TM_BULK,
						 UDC_USBC_PACKET_SIZE);
	ret = ret ? ret : mpc885_usbc_hw_ep_init(usbc_dev, 2, 2, USEP_TM_BULK,
						 UDC_USBC_PACKET_SIZE);
	usbc_dev->eps[1].pid = BD_USB_PID0;
	usbc_dev->eps[2].pid = BD_USB_RXPID_0;
	usb_rgs->usber = 0xffff;
	usb_rgs->usbmr = USBE_TXE0 | USBE_BSY | USBE_TXB
	    | USBE_RXB | USBE_RESET;

	ret =
	    ret ? ret : request_irq(CPM_IRQ_OFFSET + CPMVEC_SCC1,
				    mpc885_usbc_isr, SA_RESTART,
				    "usb_gadget_peripheral", usbc_dev);
	if (!ret)
		usbc_dev->usb_rgs->usmod |= USMOD_ENABLE;
	return ret;
}

static int __init
init(void)
{
	int ret = 0;

	if (the_controller) {
		WARN_ON(the_controller);
		return -EBUSY;
	}
	the_controller =
	    dma_alloc_coherent(NULL, sizeof(struct _mpc885_udc_dev),
			       (dma_addr_t *) & the_controller_pa, SLAB_KERNEL);
	the_controller->read_buf.buffer_pa = the_controller_pa->read_buf.buffer;
	if (!the_controller) {
		WARN_ON(!the_controller);
		return -ENOMEM;
	}
	memset(the_controller, 0, sizeof(struct _mpc885_udc_dev));
	the_controller->write_buf.buffer_pa =
	    the_controller_pa->write_buf.buffer;
	the_controller->ctrl_data_va =
	    dma_alloc_coherent(NULL, MPC885_USBC_CTL_DATA_LEN,
			       (dma_addr_t *) & the_controller->ctrl_data,
			       GFP_KERNEL);
	if (!the_controller->ctrl_data_va) {
		WARN_ON(!the_controller->ctrl_data_va);
		kfree(the_controller);
		return -ENOMEM;
	}
	memset(the_controller->ctrl_data_va, 0, MPC885_USBC_CTL_DATA_LEN);
	the_controller->gadget.ops = &mpc885_udc_ops;
	spin_lock_init(&the_controller->eps[0].rxirq_lock);
	spin_lock_init(&the_controller->eps[0].txirq_lock);
	spin_lock_init(&the_controller->eps[0].enable_lock);
	spin_lock_init(&the_controller->eps[0].txreq_lock);
	spin_lock_init(&the_controller->eps[0].rxreq_lock);
	spin_lock_init(&the_controller->eps[0].txbd_lock);
	spin_lock_init(&the_controller->eps[0].up_complete_lock);
	INIT_LIST_HEAD(&the_controller->eps[0].rqueue);
	INIT_LIST_HEAD(&the_controller->eps[0].squeue);
	spin_lock_init(&the_controller->eps[1].rxirq_lock);
	spin_lock_init(&the_controller->eps[1].txirq_lock);
	spin_lock_init(&the_controller->eps[1].enable_lock);
	spin_lock_init(&the_controller->eps[1].txreq_lock);
	spin_lock_init(&the_controller->eps[1].rxreq_lock);
	spin_lock_init(&the_controller->eps[1].txbd_lock);
	spin_lock_init(&the_controller->eps[1].up_complete_lock);
	INIT_LIST_HEAD(&the_controller->eps[1].rqueue);
	INIT_LIST_HEAD(&the_controller->eps[1].squeue);
	spin_lock_init(&the_controller->eps[2].rxirq_lock);
	spin_lock_init(&the_controller->eps[2].txirq_lock);
	spin_lock_init(&the_controller->eps[2].enable_lock);
	spin_lock_init(&the_controller->eps[2].txreq_lock);
	spin_lock_init(&the_controller->eps[2].rxreq_lock);
	spin_lock_init(&the_controller->eps[2].txbd_lock);
	spin_lock_init(&the_controller->eps[2].up_complete_lock);
	INIT_LIST_HEAD(&the_controller->eps[2].rqueue);
	INIT_LIST_HEAD(&the_controller->eps[2].squeue);
	strcpy(the_controller->gadget.dev.bus_id, "gadget");
	the_controller->gadget.name = driver_name;
	__mpc885_usbc_buf_init(&the_controller->read_buf, MPC885_USBC_BUF_SIZE);
	__mpc885_usbc_buf_init(&the_controller->write_buf,
			       MPC885_USBC_BUF_SIZE);
	ret = mpc885_usbc_hw_init(the_controller);
	if (ret) {
		mpc885_usbc_hw_deinit(the_controller);
		kfree(the_controller);
		the_controller = NULL;
		printk("Unable to initialize USB peripheral device\n");
	} else
		printk("MPC885 USB peripheral device\n");
	device_register(&the_controller->gadget.dev);

	return ret;
}

module_init(init);

static void __exit
cleanup(void)
{
	if (!the_controller)
		return;
	mpc885_usbc_hw_deinit(the_controller);
	dma_free_coherent(NULL, sizeof(struct _mpc885_udc_dev),
			  the_controller, (dma_addr_t) the_controller_pa);
	the_controller = NULL;
}

module_exit(cleanup);
