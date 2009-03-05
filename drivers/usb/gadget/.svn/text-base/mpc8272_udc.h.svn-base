/*
 * drivers/usb/gadget/mpc8272_udc.h
 *
 * MPC8272ADS USB peripheral device TTY driver necessary defines
 *
 * Author: Gennadiy Kurtsman <source@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __MPC8272_UDC_H__
#define __MPC8272_UDC_H__

/* mpc8272ads controls */
#define BCSR3_USB_SUPPLY_VCC5V	0x20000000
#define BCSR3_USB_LOW_SPEED	0x40000000
#define BCSR3_USB_DISABLE	0x80000000

/* USB rx status fields */
#define UDC_BD_RXPID_MASK   0x00c0
#define UDC_BD_RXPID_0      0x0000
#define UDC_BD_RXPID_1      0x0040
#define UDC_BD_RXPID_SETUP  0x0080
#define UDC_BD_NO    0x0010	/* Rx non-octet aligned packet */
#define UDC_BD_AB    0x0008	/* Frame aborted (bit stuff error) */
#define UDC_BD_CRC   0x0004	/* CRC error */
#define UDC_BD_OV    0x0002	/* Overrun */
#define UDC_BD_CLEAN 0x3000
#define UDC_BD_LAST  0x0800

/* USB tx status fields */
#define UDC_BD_TC    0x0400	/* Transmit CRC */
#define UDC_BD_CNF   0x0200	/* Expect for confirmation before sending the next packet */
#define UDC_BD_LSP   0x0100	/* Low-speed transaction */
#define UDC_BD_PID0  0x0080	/* Transmit DATA0 PID */
#define UDC_BD_PID1  0x00c0	/* Transmit DATA1 PID */
#define UDC_BD_NAK   0x0010	/* NAK received */
#define UDC_BD_STAL  0x0008	/* STALL received */
#define UDC_BD_TO    0x0004	/* Timeout */
#define UDC_BD_UN    0x0002	/* Underrun */
#define UDC_BD_USB_TXERR   (UDC_BD_NAK | UDC_BD_STAL | UDC_BD_TO | UDC_BD_UN)
#define UDC_BD_USB_RXERR   (UDC_BD_NO | UDC_BD_AB | UDC_BD_CRC | UDC_BD_OV)

/*--------------------------------*/
/*  BD access macros              */
/*--------------------------------*/
#define UDC_BD_STATUS(_bd)                (((mpc8272_cpm_bd *)(_bd))->status)
#define UDC_BD_STATUS_SET(_bd, _val)      (((mpc8272_cpm_bd *)(_bd))->status = _val)
#define UDC_BD_LENGTH(_bd)                (((mpc8272_cpm_bd *)(_bd))->len)
#define UDC_BD_LENGTH_SET(_bd, _val)      (((mpc8272_cpm_bd *)(_bd))->len = _val)
#define UDC_BD_DATA_CLEAR(_bd)            (((mpc8272_cpm_bd *)(_bd))->data = 0)
#define UDC_BD_IS_DATA(_bd)               (&((mpc8272_cpm_bd *)(_bd))->data)
#define UDC_BD_DATA(_bd)                  ((u8*) (__va(((mpc8272_cpm_bd *)(_bd))->data)))
#define UDC_BD_DATA_SET(_bd, _data)       (((mpc8272_cpm_bd *)(_bd))->data = __pa(_data))
#define UDC_BD_ADVANCE(_bd,_status,_base) (((_status) & BD_SC_WRAP) ? (_bd)=(_base) : ++((mpc8272_cpm_bd *)(_bd)))

/* USB related CPM commands */
#define UDC_CP_CMD_USB_STOP_TX    10
#define UDC_CP_CMD_USB_RESTART_TX 11

/* USB registers values */
#define UDC_USMOD_LSS       0x80	/* Low-speed operation */
#define UDC_USMOD_RESUME    0x40	/* Generate resume condition (function only) */
#define UDC_USMOD_TEST      0x04	/* Test (loopback) mode */
#define UDC_USMOD_HOST      0x02	/* USB host mode */
#define UDC_USMOD_ENABLE    0x01	/* USB enable */

#define UDC_USEP_NUM_MASK   0xf000	/* Endpoint number mask */
#define UDC_USEP_NUM_SHIFT  12
#define UDC_USEP_TM_CTRL    0x0000	/* Transfer mode: control */
#define UDC_USEP_TM_ISO     0x0300	/* Transfer mode: isochronous */
#define UDC_USEP_TM_BULK    0x0200	/* Transfer mode: bulk (function only) */
#define UDC_USEP_TM_INT     0x0100	/* Transfer mode: interrupt (function only) */
#define UDC_USEP_MF         0x0020	/* Multiframe enable */
#define UDC_USEP_RTE        0x0010	/* Retransmit enable (function only) */
#define UDC_USEP_THS_NORMAL 0x0000	/* Tx handshake: 0-normal, 1-ignore IN, 2-force NACK, 11-force STALL */
#define UDC_USEP_RHS_NORMAL 0x0000	/* Rx handshake: 0-normal, 1-ignore OUT, 2-force NACK, 11-force STALL */

#define UDC_USCOM_STR       0x80	/* Start FIFO fill */
#define UDC_USCOM_FLUSH     0x40	/* Flush FIFO */

#define UDC_USBE_RESET      0x0200	/* Reset condition detected */
#define UDC_USBE_IDLE       0x0100	/* Idle status changed */
#define UDC_USBE_TXE3       0x0080	/* Tx error: EP3 */
#define UDC_USBE_TXE2       0x0040	/* Tx error: EP2 */
#define UDC_USBE_TXE1       0x0020	/* Tx error: EP1 */
#define UDC_USBE_TXE0       0x0010	/* Tx error: EP0 */
#define UDC_USBE_SOF        0x0008	/* SOF received */
#define UDC_USBE_BSY        0x0004	/* Busy condition (no rx buffer) */
#define UDC_USBE_TXB        0x0002	/* A buffer has been transmitted */
#define UDC_USBE_RXB        0x0001	/* A buffer has been received */

/* RFCR/TFCR fields */
#define UDC_USB_FCR_BO_PPC   0x08	/* PPC little endian (dword byte swapping) */
#define UDC_USB_FCR_BO_BE    0x18	/* MOT mode: big endian */

#define UDC_VEC_USB   11

/* USB Console stuff*/
#define UDC_USBC_DEVICE_NAME    "usb-tty"
#define UDC_USBC_BUF_SIZE       128
#define UDC_USBC_BUS_CLOCK      mpc8272_xclk8
#define UDC_USBC_TX_BDS         4	/* Tx BD ring size */
#define UDC_USBC_RX_BDS         16	/* Rx BD ring size */
#define UDC_USBC_PACKET_SIZE    8	/* Max packet size */
#define UDC_USBC_CTL_PACKET_SIZE 16	/* Max packet size for CTL endpoint */
#define UDC_USBC_MAX_EPS        3	/* Max number of endpoints */
#define UDC_USBC_PQID           0
#define UDC_USBC_CTL_DATA_LEN   80
#define UDC_USBC_TRACE_SIZE     64
#define UDC_SERIAL_TTY_MAJOR    189
#define UDC_SERIAL_TTY_MINORS   1

#define UDC_USEP_RHS_MASK   0x0003
#define UDC_USEP_RHS_ACK    0x0000
#define UDC_USEP_RHS_NAK    0x0002
#define UDC_USEP_RHS_STALL  0x0003

#define MPC8272_USBC_BUF_SIZE		128
#define MPC8272_USBC_MAX_EPS		3       /* Max number of endpoints */
#define MPC8272_USBC_CTL_DATA_LEN	80

/*
 * USB Packet IDs (PIDs)
 */
#define USB_PID_UNDEF_0			0xf0
#define USB_PID_OUT			0xe1
#define USB_PID_ACK			0xd2
#define USB_PID_DATA0			0xc3
#define USB_PID_PING			0xb4	/* USB 2.0 */
#define USB_PID_SOF			0xa5
#define USB_PID_NYET			0x96	/* USB 2.0 */
#define USB_PID_DATA2			0x87	/* USB 2.0 */
#define USB_PID_SPLIT			0x78	/* USB 2.0 */
#define USB_PID_IN			0x69
#define USB_PID_NAK			0x5a
#define USB_PID_DATA1			0x4b
#define USB_PID_PREAMBLE		0x3c	/* Token mode */
#define USB_PID_ERR			0x3c	/* USB 2.0: handshake mode */
#define USB_PID_SETUP			0x2d
#define USB_PID_STALL			0x1e
#define USB_PID_MDATA			0x0f	/* USB 2.0 */

/* (shifted) direction/type/recipient from the USB 2.0 spec, table 9.2 */
#define DeviceRequest \
	((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_DEVICE)<<8)
#define DeviceOutRequest \
	((USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_DEVICE)<<8)

#define InterfaceRequest \
	((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_INTERFACE)<<8)

#define EndpointRequest \
	((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_ENDPOINT)<<8)
#define EndpointOutRequest \
	((USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_ENDPOINT)<<8)

/* Struct definitions */

struct mpc8272_request {
        struct usb_request              req;
        struct list_head                queue;
};

typedef struct mpc8272_cpm_bd_s {
	u16 status;
	u16 len;
	u32 data;
} mpc8272_cpm_bd;

/* IO buf:
   empty: read==write
   full: next_write==read
*/
struct mpc8272_usbc_buf {
	int read_pos;
	int write_pos;
	int size;
	__u8 buffer[MPC8272_USBC_BUF_SIZE];
	struct semaphore sem;
};

struct mpc8272_usbc_ep;
typedef int (*mpc8272_usbc_rx_callback) (struct mpc8272_usbc_ep * ep, int pid,
			__u8 * data, __u16 len, int is_error);
typedef void (*mpc8272_usbc_txconf_callback) (struct mpc8272_usbc_ep * ep, __u16 len,
			int is_error);

/* Endpoint */
struct mpc8272_usbc_ep {
	struct usb_ep ep;
	struct _mpc8272_udc_dev *dev;
	int ep_idx;		/* Endpoint index */
	int ep_num;		/* Number of the endpoint */
	__u16 ep_type;		/* Endpoint type: bulk, control, interrupt, iso */
	struct usb_endpoint_descriptor* desc;
	usb_ep_parms *ep_parms;	/* EP parameters area */
	__u16 ep_parms_offset;	/* dual-port memory offset */
	mpc8272_cpm_bd *tx_base;	/* 1st BD in Tx BD ring */
	__u16 tx_base_offset;		/* dual-port memory offset */
	mpc8272_cpm_bd *rx_base;	/* 1st BD in Rx BD ring */
	__u16 rx_base_offset;		/* dual-port memory offset */
	mpc8272_cpm_bd *conf_bd;	/* Next bd to confirm */
	mpc8272_cpm_bd *tx_bd;	/* Next tx bd */
	mpc8272_cpm_bd *rx_bd;	/* Next rx bd */
	__u8 *rx_pool;
	mpc8272_usbc_rx_callback rx_cb;
	mpc8272_usbc_txconf_callback txconf_cb;
	spinlock_t lock;
	struct list_head queue;
	/* State variables */
	__u8 *data;
	__u16 pid;
	int max_len;
	int data_len;
	int ack_len;
};

struct _mpc8272_udc_dev {
	struct usb_gadget		gadget;
	struct usb_gadget_driver 	*driver;

	struct mpc8272_usbc_ep eps[MPC8272_USBC_MAX_EPS];

	/* Useful addresses */
	cpm2_map_t *intram;
	usb_parms *usb_prms;
	usb_cpm2_t *usb_rgs;

	/* IO buffers */
	struct mpc8272_usbc_buf read_buf;
	struct mpc8272_usbc_buf write_buf;

	/* Control variables */
	__u8 ctrl_data[MPC8272_USBC_CTL_DATA_LEN];
	int ctrl_length;
	int address;
};

static inline void
__mpc8272_usbc_buf_init(struct mpc8272_usbc_buf *buf, int size)
{
	buf->size = size;
	buf->read_pos = buf->write_pos = 0;
	init_MUTEX(&buf->sem);
}

static inline int
__mpc8272_usbc_buf_len(struct mpc8272_usbc_buf *buf)
{
	int len;
	if (buf->write_pos >= buf->read_pos)
		len = buf->write_pos - buf->read_pos;
	else
		len = buf->size - buf->read_pos + buf->write_pos;
	return len;
}

static inline int
__mpc8272_usbc_buf_len_flat(struct mpc8272_usbc_buf *buf)
{
	int len;
	if (buf->write_pos >= buf->read_pos)
		len = buf->write_pos - buf->read_pos;
	else
		len = buf->size - buf->read_pos;
	return len;
}

static inline __u8 *
__mpc8272_usbc_buf_data(struct mpc8272_usbc_buf *buf)
{
	return buf->buffer + buf->read_pos;
}

static inline int
__mpc8272_usbc_buf_room(struct mpc8272_usbc_buf *buf)
{
	int len;
	if (buf->write_pos >= buf->read_pos)
		len = buf->size - buf->write_pos + buf->read_pos - 1;
	else
		len = buf->read_pos - buf->write_pos - 1;
	return len;
}

static inline int
__mpc8272_usbc_buf_inc_read(struct mpc8272_usbc_buf *buf, int len)
{
	int pos = buf->read_pos + len;
	buf->read_pos = pos % buf->size;
	return buf->read_pos;
}

static inline int
__mpc8272_usbc_buf_inc_write(struct mpc8272_usbc_buf *buf, int len)
{
	int pos = buf->write_pos + len;
	buf->write_pos = pos % buf->size;
	return buf->write_pos;
}

/**********************************************************************************
 * IO buffer helpers
 *********************************************************************************/

static inline int
__mpc8272_usbc_buf_write(struct mpc8272_usbc_buf *buf, const __u8 * data, int len)
{
	int buf_len = __mpc8272_usbc_buf_room(buf);
	int tail_len = buf->size - buf->write_pos;

	if (len > buf_len) {
		len = buf_len;
	}
	if (tail_len > len)
		tail_len = len;
	memcpy(&buf->buffer[buf->write_pos], data, tail_len);
	__mpc8272_usbc_buf_inc_write(buf, tail_len);
	data += tail_len;
	if (len > tail_len) {
		memcpy(&buf->buffer[0], data, len - tail_len);
		__mpc8272_usbc_buf_inc_write(buf, len - tail_len);
	}
	return len;
}

static inline int
__mpc8272_usbc_buf_writeuser(struct mpc8272_usbc_buf *buf, const __u8 * data, int len)
{
	int buf_len = __mpc8272_usbc_buf_room(buf);
	int tail_len = buf->size - buf->write_pos;

	if (len > buf_len)
		len = buf_len;
	if (tail_len > len)
		tail_len = len;
	copy_from_user(&buf->buffer[buf->write_pos], data, tail_len);
	__mpc8272_usbc_buf_inc_write(buf, tail_len);
	data += tail_len;
	if (len > tail_len) {
		copy_from_user(&buf->buffer[0], data, len - tail_len);
		__mpc8272_usbc_buf_inc_write(buf, len - tail_len);
	}
	return len;
}

static inline int
__mpc8272_usbc_buf_read(struct mpc8272_usbc_buf *buf, __u8 * data, int len)
{
	int buf_len = __mpc8272_usbc_buf_len(buf);
	int tail_len = buf->size - buf->read_pos;

	if (len > buf_len)
		len = buf_len;
	if (tail_len > len)
		tail_len = len;
	memcpy(data, &buf->buffer[buf->read_pos], tail_len);
	__mpc8272_usbc_buf_inc_read(buf, tail_len);
	data += tail_len;
	if (len > tail_len) {
		memcpy(data, &buf->buffer[0], len - tail_len);
		__mpc8272_usbc_buf_inc_read(buf, len - tail_len);
	}
	return len;
}

#define UDC_CP_USB_DEV 0x2e600000	/* See UM, 5.3.1 */
static inline void
mpc8272_usbc_cmd(struct mpc8272_usbc_ep *ep, __u32 cmd)
{
	unsigned long flags = 0;

	if (!in_irq()) {
		save_and_cli(flags);
		spin_lock(&ep->lock);
	}
	(volatile unsigned __iomem )ep->dev->intram->im_cpm.cp_cpcr = (cmd | CPM_CR_FLG | UDC_CP_USB_DEV | ep->ep_idx << 4);
	mb();
	/* Wait for the CR_FLG to clear */
	while ((volatile unsigned __iomem)ep->dev->intram->im_cpm.cp_cpcr & CPM_CR_FLG)
		mb();
	if (!in_irq()) {
		spin_unlock(&ep->lock);
		restore_flags(flags);
	}
}

static inline void
mpc8272_usbc_tx_recover(struct mpc8272_usbc_ep *ep)
{
	mpc8272_cpm_bd *bd = ep->conf_bd;
	/* Stop TX (just in case - should have stopped by itself) */
	mpc8272_usbc_cmd(ep, UDC_CP_CMD_USB_STOP_TX);
	ep->dev->usb_rgs->usb_uscom = UDC_USCOM_FLUSH | ep->ep_idx;

	while (bd->data) {
		__u16 status = UDC_BD_STATUS(bd);
		status &= BD_SC_WRAP;
		UDC_BD_STATUS_SET(bd, status);
		UDC_BD_DATA_CLEAR(bd);
		UDC_BD_ADVANCE(bd, status, ep->tx_base);
	}

	ep->tx_bd = ep->conf_bd = ep->tx_base;
	ep->ep_parms->tbptr =
	    (__u16) ((unsigned long) ep->tx_base -
		     (unsigned long) ep->dev->intram);
	ep->data = NULL;
	ep->ack_len = ep->data_len = 0;
	mpc8272_usbc_cmd(ep, UDC_CP_CMD_USB_RESTART_TX);
}

/* Transmit data or handshake */
static inline void
mpc8272_usbc_tx(struct mpc8272_usbc_ep *ep, __u8 * data, int len, __u16 flags)
{
	mpc8272_cpm_bd *tx_bd = ep->tx_bd;
	__u16 status = UDC_BD_STATUS(tx_bd);
	if ((status & BD_SC_READY)) {
		return;
	}
	if (len > ep->max_len)
		len = ep->max_len;
	UDC_BD_DATA_SET(tx_bd, data);
	UDC_BD_LENGTH_SET(tx_bd, len);
	status &= BD_SC_WRAP;
	mb();
	status |= flags | BD_SC_READY | UDC_BD_LAST;
	mb();
	status |= ep->pid;
	mb();
	UDC_BD_STATUS_SET(tx_bd, status);
	mb();
	ep->tx_bd = UDC_BD_ADVANCE(tx_bd, status, ep->tx_base);
	mb();
	ep->dev->usb_rgs->usb_uscom = (UDC_USCOM_STR | ep->ep_idx);
}

/* Transmit handshake */
static inline void
mpc8272_usbc_tx_handshake(struct _mpc8272_udc_dev *usbc_dev, int ep_idx, __u8 handshake)
{
	volatile __u16 *p_usep_reg;
	__u16 usep_reg;

	if (ep_idx >= UDC_USBC_MAX_EPS)
		return;

	p_usep_reg = &usbc_dev->usb_rgs->usb_usep1 + ep_idx;
	usep_reg = *p_usep_reg & (~UDC_USEP_RHS_MASK);

	switch (handshake) {
	case USB_PID_ACK:
		*p_usep_reg = (usep_reg | UDC_USEP_RHS_ACK);
		break;
	case USB_PID_NAK:
		*p_usep_reg = (usep_reg | UDC_USEP_RHS_NAK);
		break;
	case USB_PID_STALL:
		*p_usep_reg = (usep_reg | UDC_USEP_RHS_STALL);
		break;
	default:
		*p_usep_reg = (usep_reg | UDC_USEP_RHS_ACK);
		break;
	}

}

/* Transmit handshake */
static inline void
mpc8272_usbc_tx_data(struct mpc8272_usbc_ep *ep, __u8 * data, int len)
{
	ep->data = data;
	ep->data_len = len;
	ep->ack_len = 0;
	mpc8272_usbc_tx(ep, data, len,
		    UDC_BD_TC | UDC_BD_CNF | BD_SC_INTRPT | ep->pid);
}

/* Rx Interrupt handler */
static inline void
mpc8272_usbc_rx_isr(struct _mpc8272_udc_dev *usbc_dev, int ep_idx)
{
	struct mpc8272_usbc_ep *ep = &usbc_dev->eps[ep_idx];
	mpc8272_cpm_bd *rx_bd = ep->rx_bd;
	__u16 status = UDC_BD_STATUS(rx_bd);
	__u8 *data;
	__u16 len;
	int pid;

	spin_lock(&ep->lock);
	while (!(status & BD_SC_EMPTY)) {
		data = (__u8 *) UDC_BD_DATA(rx_bd);
		len = UDC_BD_LENGTH(rx_bd);
		if (len >= 2)
			len -= 2;	/* Strip CRC16 */
		pid = status & UDC_BD_RXPID_MASK;
		if (ep->rx_cb(ep, pid, data, len, (status & UDC_BD_USB_RXERR)))
			break;
		status &= UDC_BD_CLEAN;
		UDC_BD_LENGTH_SET(rx_bd, 0);
		UDC_BD_STATUS_SET(rx_bd, status | BD_SC_EMPTY);
		UDC_BD_ADVANCE(rx_bd, status, ep->rx_base);
		status = UDC_BD_STATUS(rx_bd);
	}
	ep->rx_bd = rx_bd;
	spin_unlock(&ep->lock);
}

/* Tx Interrupt handler */
static inline void
mpc8272_usbc_tx_isr(struct _mpc8272_udc_dev *usbc_dev, int ep_idx)
{
	struct mpc8272_usbc_ep *ep = &usbc_dev->eps[ep_idx];
	mpc8272_cpm_bd *tx_bd = ep->conf_bd;
	__u16 status = UDC_BD_STATUS(tx_bd);
	int is_error;
	__u16 len;

	spin_lock(&ep->lock);
	while (!(status & BD_SC_READY)) {
		if (!tx_bd->data) {
			break;
		}
		len = UDC_BD_LENGTH(tx_bd);
		is_error = (status & UDC_BD_USB_TXERR);
		if ((status & (UDC_BD_TO | UDC_BD_UN)))
			mpc8272_usbc_tx_recover(ep);
		ep->txconf_cb(ep, len, is_error);
		UDC_BD_DATA_CLEAR(tx_bd);
		UDC_BD_ADVANCE(tx_bd, status, ep->tx_base);
		status = UDC_BD_STATUS(tx_bd);
	}
	ep->conf_bd = tx_bd;
	spin_unlock(&ep->lock);
}

void mpc8272_usb_hw_config(cpm2_map_t * immr);
void mpc8272_board_usb_iface_config(void);
void mpc8272_board_usb_iface_deconfig(void);
unsigned short mpc8272_dpram_offset(void *addr);

#endif /* __MPC8272_UDC_H__ */
