/*
 * drivers/usb/gadget/mpc885_udc.h
 *
 * MPC885ADS USB peripheral device TTY driver necessary defines
 *
 * Author: Gennadiy Kurtsman <source@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __MPC885_UDC_H__
#define __MPC885_UDC_H__

/*--------------------------------*/
/*  BD access macros              */
/*--------------------------------*/
#define UDC_BD_STATUS(_bd)                (((mpc885_cpm_bd *)(_bd))->status)
#define UDC_BD_STATUS_SET(_bd, _val)      (((mpc885_cpm_bd *)(_bd))->status = _val)
#define UDC_BD_LENGTH(_bd)                (((mpc885_cpm_bd *)(_bd))->len)
#define UDC_BD_LENGTH_SET(_bd, _val)      (((mpc885_cpm_bd *)(_bd))->len = _val)
#define UDC_BD_DATA_CLEAR(_bd)            (((mpc885_cpm_bd *)(_bd))->data = 0)
#define UDC_BD_DATA(_bd)                  ((u8*) ((((mpc885_cpm_bd *)(_bd))->data)))
#define UDC_BD_DATA_SET(_bd, _data)       (((mpc885_cpm_bd *)(_bd))->data = (_data))
#define UDC_BD_ADVANCE(_bd,_status,_base) (((_status) & BD_SC_WRAP) ? (_bd)=(_base) : ++((mpc885_cpm_bd *)(_bd)))

/* USB Console stuff*/
#define UDC_USBC_TX_BDS         4	/* Tx BD ring size */
#define UDC_USBC_RX_BDS         8	/* Rx BD ring size */
#define UDC_USBC_PACKET_SIZE    8	/* Max packet size */
#define UDC_USBC_CTL_PACKET_SIZE 16	/* Max packet size for CTL endpoint */
#define UDC_USBC_MAX_EPS        3	/* Max number of endpoints */

#define MPC885_USBC_BUF_SIZE		128
#define MPC885_USBC_MAX_EPS		3	/* Max number of endpoints */
#define MPC885_USBC_CTL_DATA_LEN	80

/*
 * USB Packet IDs (PIDs)
 */
#define USB_PID_NAK			0x5a
#define USB_PID_STALL			0x1e

#include "../core/hcd.h"

/* Struct definitions */

struct mpc885_request {
	struct usb_request req;
	struct list_head queue;
};

typedef struct mpc885_cpm_bd_s {
	u16 status;
	u16 len;
	u32 data;
} mpc885_cpm_bd;

/* IO buf:
   empty: read==write
   full: next_write==read
*/
struct mpc885_usbc_buf {
	int read_pos;
	int write_pos;
	int size;
	__u8 buffer[MPC885_USBC_BUF_SIZE];
	__u8 *buffer_pa;
	struct semaphore sem;
};

struct mpc885_usbc_ep;
typedef int (*mpc885_usbc_rx_callback) (struct mpc885_usbc_ep * ep, int pid,
					__u8 * data, __u16 len, int is_error,
					unsigned long flags);
typedef void (*mpc885_usbc_txconf_callback) (struct mpc885_usbc_ep * ep,
					     __u16 len, int is_error);

/* Endpoint */
struct mpc885_usbc_ep {
	struct usb_ep ep;
	struct _mpc885_udc_dev *dev;
	int ep_idx;		/* Endpoint index */
	int ep_num;		/* Number of the endpoint */
	__u16 ep_type;		/* Endpoint type: bulk, control, interrupt, iso */
	struct usb_endpoint_descriptor *desc;
	usb_ep_parms_t *ep_parms;	/* EP parameters area */
	usb_ep_parms_t *ep_parms_va;	/* result of ioremap */
	__u16 ep_parms_offset;	/* dual-port memory offset */
	mpc885_cpm_bd *tx_base;	/* 1st BD in Tx BD ring */
	mpc885_cpm_bd *tx_base_va;	/* result of ioremap */
	__u16 tx_base_offset;	/* dual-port memory offset */
	mpc885_cpm_bd *rx_base;	/* 1st BD in Rx BD ring */
	mpc885_cpm_bd *rx_base_va;	/* result of ioremap */
	__u16 rx_base_offset;	/* dual-port memory offset */
	mpc885_cpm_bd *conf_bd;	/* Next bd to confirm */
	mpc885_cpm_bd *conf_bd_va;	/* result of ioremap */
	mpc885_cpm_bd *tx_bd;	/* Next tx bd */
	mpc885_cpm_bd *tx_bd_va;	/* result of ioremap */
	mpc885_cpm_bd *rx_bd;	/* Next rx bd */
	mpc885_cpm_bd *rx_bd_va;	/* result of ioremap */
	__u16 rx_pool_offset;
	__u8 *rx_pool;
	__u8 *rx_pool_va;
	mpc885_usbc_rx_callback rx_cb;
	mpc885_usbc_txconf_callback txconf_cb;
/* Spinlocks below are intended to serialize access to the driver resources.
   There are a wrong functioning and a lot of diagnostics when
   CONFIG_DEBUG_PREEMPT and CONFIG_RT_DEADLOCK_DETECT kernel configuration
   definitions are yes in RT preemption kernel. When CONFIG_DEBUG_PREEMPT
   and CONFIG_RT_DEADLOCK_DETECT are n functioning is correct.
*/
	spinlock_t cmd_lock;		/* serialization of access to CPCR register */
	spinlock_t rxirq_lock;		/* serialization of RX interrupt handlers for
					   RT kernel threaded hardirqs */
	spinlock_t txirq_lock;		/* serialization of TX interrupt handlers for
					   RT kernel threaded hardirqs */
	spinlock_t enable_lock;		/* serialization of access to endpoint
					   descriptor structure */
	spinlock_t txreq_lock;		/* serialization of access to serial gadget's
					   TX request structure */
	spinlock_t rxreq_lock;		/* serialization of access to serial gadget's
					   RX request structure */
	spinlock_t txbd_lock;		/* serialization of access to USB TX buffer
					   descriptor ring */
	spinlock_t up_complete_lock;	/* serialization of access to complete()
					   callback of gadget's request structure */
/************************************************************/
	int len_to_req;
	struct list_head rqueue;
	struct list_head squeue;
	/* State variables */
	__u8 *data;
	__u16 pid;
	int max_len;
	int data_len;
	int ack_len;
};

struct _mpc885_udc_dev {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;

	struct mpc885_usbc_ep eps[MPC885_USBC_MAX_EPS];

	/* Useful addresses */
	immap_t *intram;
	usb_parms_t *usb_prms;
	usb_regs_t *usb_rgs;

	/* IO buffers */
	struct mpc885_usbc_buf read_buf;
	struct mpc885_usbc_buf write_buf;

	/* Control variables */
	__u8 *ctrl_data;
	__u8 *ctrl_data_va;
	int ctrl_length;
	int address;
};

static inline void
__mpc885_usbc_buf_init(struct mpc885_usbc_buf *buf, int size)
{
	buf->size = size;
	buf->read_pos = buf->write_pos = 0;
	init_MUTEX(&buf->sem);
}

static inline int
__mpc885_usbc_buf_len(struct mpc885_usbc_buf *buf)
{
	int len;
	if (buf->write_pos >= buf->read_pos)
		len = buf->write_pos - buf->read_pos;
	else
		len = buf->size - buf->read_pos + buf->write_pos;
	return len;
}

static inline int
__mpc885_usbc_buf_len_flat(struct mpc885_usbc_buf *buf)
{
	int len;
	if (buf->write_pos >= buf->read_pos)
		len = buf->write_pos - buf->read_pos;
	else
		len = buf->size - buf->read_pos;
	return len;
}

static inline __u8*
__mpc885_usbc_buf_data(struct mpc885_usbc_buf *buf)
{
	return buf->buffer + buf->read_pos;
}

static inline int
__mpc885_usbc_buf_room(struct mpc885_usbc_buf *buf)
{
	int len;
	if (buf->write_pos >= buf->read_pos)
		len = buf->size - buf->write_pos + buf->read_pos - 1;
	else
		len = buf->read_pos - buf->write_pos - 1;
	return len;
}

static inline int
__mpc885_usbc_buf_inc_read(struct mpc885_usbc_buf *buf, int len)
{
	int pos = buf->read_pos + len;
	buf->read_pos = pos % buf->size;
	return buf->read_pos;
}

static inline int
__mpc885_usbc_buf_inc_write(struct mpc885_usbc_buf *buf, int len)
{
	int pos = buf->write_pos + len;
	buf->write_pos = pos % buf->size;
	return buf->write_pos;
}

/**********************************************************************************
 * IO buffer helpers
 *********************************************************************************/

static inline int
__mpc885_usbc_buf_write(struct mpc885_usbc_buf *buf, const __u8 * data, int len)
{
	int buf_len = __mpc885_usbc_buf_room(buf);
	int tail_len = buf->size - buf->write_pos;

	if (len > buf_len) {
		len = buf_len;
	}
	if (tail_len > len)
		tail_len = len;
	memcpy(&buf->buffer[buf->write_pos], data, tail_len);
	__mpc885_usbc_buf_inc_write(buf, tail_len);
	data += tail_len;
	if (len > tail_len) {
		memcpy(&buf->buffer[0], data, len - tail_len);
		__mpc885_usbc_buf_inc_write(buf, len - tail_len);
	}
	return len;
}

static inline int
__mpc885_usbc_buf_writeuser(struct mpc885_usbc_buf *buf, const __u8 * data,
			    int len)
{
	int buf_len = __mpc885_usbc_buf_room(buf);
	int tail_len = buf->size - buf->write_pos;

	if (len > buf_len)
		len = buf_len;
	if (tail_len > len)
		tail_len = len;
	copy_from_user(&buf->buffer[buf->write_pos], data, tail_len);
	__mpc885_usbc_buf_inc_write(buf, tail_len);
	data += tail_len;
	if (len > tail_len) {
		copy_from_user(&buf->buffer[0], data, len - tail_len);
		__mpc885_usbc_buf_inc_write(buf, len - tail_len);
	}
	return len;
}

static inline int
__mpc885_usbc_buf_read(struct mpc885_usbc_buf *buf, __u8 * data, int len)
{
	int buf_len = __mpc885_usbc_buf_len(buf);
	int tail_len = buf->size - buf->read_pos;

	if (len > buf_len)
		len = buf_len;
	if (tail_len > len)
		tail_len = len;
	memcpy(data, &buf->buffer[buf->read_pos], tail_len);
	__mpc885_usbc_buf_inc_read(buf, tail_len);
	data += tail_len;
	if (len > tail_len) {
		memcpy(data, &buf->buffer[0], len - tail_len);
		__mpc885_usbc_buf_inc_read(buf, len - tail_len);
	}
	return len;
}

static inline void
mpc885_usbc_cmd(struct mpc885_usbc_ep *ep, __u32 cmd)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&ep->cmd_lock, flags);
	ep->dev->intram->im_cpm.cp_cpcr =
	    (cmd | mk_cr_cmd(CPM_CR_CH_USB, CPM_CR_USB) | ep->
	     ep_idx << 2 | CPM_CR_FLG);
	/* Wait for the CR_FLG to clear */
	udelay(2);		/* I can not explain this delay but w/o it driver freezes */
	while (ep->dev->intram->im_cpm.cp_cpcr & CPM_CR_FLG) ;	/* wait for the CR_FLG to clear */
	spin_unlock_irqrestore(&ep->cmd_lock, flags);
}

__u16
mpc885_dpram_offset(void *addr)
{
	int offset = (int)((__u32) addr -
			   (__u32) ((immap_t *) IMAP_ADDR)->im_cpm.cp_dpmem);

	if ((offset < 0) ||
	    (offset >= sizeof(((immap_t *) IMAP_ADDR)->im_cpm.cp_dpmem))) {
		printk(KERN_ERR "%s: invalid dpram address 0x%08x\n",
		       __FUNCTION__, (__u32) addr);
		return 0;
	}

	return (__u16) offset;
}

static inline void
mpc885_usbc_tx_recover(struct mpc885_usbc_ep *ep)
{

	mpc885_cpm_bd *bd = ep->conf_bd_va;
	/* Stop TX (just in case - should have stopped by itself) */
	mpc885_usbc_cmd(ep, CPM_USBCMD_STOP_TX);
	ep->dev->usb_rgs->uscom = USCOM_FLUSH | ep->ep_idx;

	while (bd->data) {
		__u16 status = UDC_BD_STATUS(bd);
		status &= BD_SC_WRAP;
		UDC_BD_STATUS_SET(bd, status);
		UDC_BD_DATA_CLEAR(bd);
		UDC_BD_ADVANCE(bd, status, ep->tx_base_va);
	}

	ep->tx_bd = ep->conf_bd = ep->tx_base;
	ep->tx_bd_va = ep->conf_bd_va = ep->tx_base_va;
	ep->ep_parms_va->tbptr = mpc885_dpram_offset(ep->tx_base);
	ep->ep_parms_va->tstate = 0;
	ep->data = NULL;
	ep->ack_len = ep->data_len = 0;
	mpc885_usbc_cmd(ep, CPM_USBCMD_RESTART_TX);
}

/* Transmit data or handshake */
static inline void
mpc885_usbc_tx(struct mpc885_usbc_ep *ep, __u8 * data, int len, __u16 flags)
{
	mpc885_cpm_bd *tx_bd = ep->tx_bd;
	mpc885_cpm_bd *tx_bd_va = ep->tx_bd_va;
	__u16 status = UDC_BD_STATUS(tx_bd_va);

	if ((status & BD_SC_READY)) {
		return;
	}
	if (len > ep->max_len)
		len = ep->max_len;
	UDC_BD_DATA_SET(tx_bd_va, (u32) data);
	UDC_BD_LENGTH_SET(tx_bd_va, len);
	status &= BD_SC_WRAP;
	status |= flags | BD_SC_READY | BD_SC_LAST;
	status |= ep->pid;
	UDC_BD_STATUS_SET(tx_bd_va, status);
	ep->tx_bd_va = UDC_BD_ADVANCE(tx_bd_va, status, ep->tx_base_va);
	ep->tx_bd = UDC_BD_ADVANCE(tx_bd, status, ep->tx_base);
	ep->dev->usb_rgs->uscom = (USCOM_STR | ep->ep_idx);
}

/* Transmit handshake */
static inline void
mpc885_usbc_tx_handshake(struct mpc885_usbc_ep *ep, __u8 handshake)
{

	ep->dev->ctrl_data_va[0] = handshake;
	mpc885_usbc_tx(ep, ep->dev->ctrl_data, 1, 0);
}

/* Transmit data */
static inline void
mpc885_usbc_tx_data(struct mpc885_usbc_ep *ep, __u8 * data, int len)
{
	ep->data = data;
	ep->data_len = len;
	ep->ack_len = 0;
	mpc885_usbc_tx(ep, data, len,
		       BD_USB_TC | BD_USB_CNF | BD_SC_INTRPT | ep->pid);
}

/* Rx Interrupt handler */
static inline void
mpc885_usbc_rx_isr(struct _mpc885_udc_dev *usbc_dev, int ep_idx)
{
	struct mpc885_usbc_ep *ep = &usbc_dev->eps[ep_idx];
	mpc885_cpm_bd *rx_bd = ep->rx_bd;
	mpc885_cpm_bd *rx_bd_va = ep->rx_bd_va;
	__u16 status = UDC_BD_STATUS(rx_bd_va);
	__u8 *data;
	__u16 len;
	int pid;
	unsigned long flags;

	spin_lock_irqsave(&ep->rxirq_lock, flags);
	while (!(status & BD_SC_EMPTY)) {
		data = (__u8 *) UDC_BD_DATA(rx_bd_va);
		len = UDC_BD_LENGTH(rx_bd_va);
		if (len >= 2)
			len -= 2;	/* Strip CRC16 */
		pid = status & BD_USB_RXPID_MASK;
		if (ep->rx_cb(ep, pid, data,
			      len, (status & BD_USB_RXERR), flags))
			break;
		status &= BD_USB_CLEAN;
		UDC_BD_LENGTH_SET(rx_bd_va, 0);
		UDC_BD_STATUS_SET(rx_bd_va, status | BD_SC_EMPTY);
		UDC_BD_ADVANCE(rx_bd_va, status, ep->rx_base_va);
		UDC_BD_ADVANCE(rx_bd, status, ep->rx_base);
		udelay(1000);	/* I can not explain this delay but w/o it driver freezes */
		status = UDC_BD_STATUS(rx_bd_va);
	}
	ep->rx_bd = rx_bd;
	ep->rx_bd_va = rx_bd_va;
	spin_unlock_irqrestore(&ep->rxirq_lock, flags);
}

/* Tx Interrupt handler */
static inline void
mpc885_usbc_tx_isr(struct _mpc885_udc_dev *usbc_dev, int ep_idx)
{
	struct mpc885_usbc_ep *ep = &usbc_dev->eps[ep_idx];
	mpc885_cpm_bd *tx_bd = ep->conf_bd;
	mpc885_cpm_bd *tx_bd_va = ep->conf_bd_va;
	__u16 status = UDC_BD_STATUS(tx_bd_va);
	int is_error;
	__u16 len;
	unsigned long flags;

	spin_lock_irqsave(&ep->txirq_lock, flags);
	while (!(status & BD_SC_READY)) {
		if (!tx_bd_va->data) {
			break;
		}
		len = UDC_BD_LENGTH(tx_bd_va);
		is_error = (status & BD_USB_TXERR);
		if ((status & (BD_USB_TO | BD_USB_UN)))
			mpc885_usbc_tx_recover(ep);
		else
			ep->txconf_cb(ep, len, is_error);
		UDC_BD_DATA_CLEAR(tx_bd_va);
		UDC_BD_ADVANCE(tx_bd, status, ep->tx_base);
		UDC_BD_ADVANCE(tx_bd_va, status, ep->tx_base_va);
		status = UDC_BD_STATUS(tx_bd_va);
	}
	ep->conf_bd = tx_bd;
	ep->conf_bd_va = tx_bd_va;
	spin_unlock_irqrestore(&ep->txirq_lock, flags);
}

#endif				/* __MPC885_UDC_H__ */
