/*
 * Renesas SuperH USB 1.1 device controller (found on SH7705, SH7727...)
 *
 * Copyright (C) 2003 Renesas Technology Europe Limited
 * Copyright (C) 2003 Julian Back (jback@mpc-data.co.uk), MPC Data Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __LINUX_USB_GADGET_SUPERH_UDC_H
#define __LINUX_USB_GADGET_SUPERH_UDC_H

#include <linux/types.h>
#include <asm/irq.h>

struct superh_udc;

struct superh_ep {
	struct usb_ep				ep;
	struct superh_udc			*dev;

	const struct usb_endpoint_descriptor	*desc;
	struct list_head			queue;
	int					dma;

	u8					bEndpointAddress;
	u8					bmAttributes;

	unsigned				stopped : 1;
	unsigned                                halted : 1;

	u8                                      data_present_mask;
	u8                                      stall_mask;
	u8                                      interrupt_mask;
	u8                                      clear_mask;
	u8                                      packet_enable_mask;
	unsigned                                interrupt_reg;
	unsigned                                fifo_reg;
};

struct superh_request {
	struct usb_request			req;
	struct list_head			queue;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

void and_w(unsigned short mask, unsigned long addr)
{
    ctrl_outw(ctrl_inw(addr) & mask, addr);
}

void or_w(unsigned short mask, unsigned long addr)
{
    ctrl_outw(ctrl_inw(addr) | mask, addr);
}

#define USBFI0_IRQ	65
#define USBFI1_IRQ	66

#define EP0_FIFO_SIZE	((unsigned)8)
#define BULK_FIFO_SIZE	((unsigned)64)
#define ISO_FIFO_SIZE	((unsigned)0)
#define INT_FIFO_SIZE	((unsigned)8)

struct udc_stats {
	struct ep0stats {
		unsigned long		ops;
		unsigned long		bytes;
	} read, write;
	unsigned long			irqs;
	unsigned long                   irq0s;
	unsigned long                   irq1s;
};

struct superh_udc {
	struct usb_gadget			gadget;
	struct usb_gadget_driver		*driver;
	atomic_t		                in_interrupt;

	enum ep0_state				ep0state;
	struct udc_stats			stats;
	unsigned int                            vbusmn;
	unsigned long                           vbusf_time;
	unsigned				got_irq0 : 1,
  	                                        got_irq1 : 1,
						fake_config: 1;
	int                                     setup_countdown;
	unsigned long                           reset_time;
	struct timer_list			timer;
	struct superh_ep			ep [4];
};

/* 2.5 changes ... */

#ifndef container_of
#define container_of    list_entry
#endif

#ifndef WARN_ON
#define WARN_ON BUG_ON
#endif

/* one I/O pin should be used to detect disconnect */
#define is_usb_connected        ((ctrl_inb(USBIFR1) & VBUSF) != 0)

/* Register addresses - should really be in include/asm-sh */

#ifdef CONFIG_CPU_SUBTYPE_SH7705

#define USBEPDR0I     0xA4480000
#define USBEPDR0O     0xA4480004
#define USBEPDR0S     0xA4480008
#define USBEPDR1      0xA448000C
#define USBEPDR2      0xA4480010
#define USBEPDR3      0xA4480014
#define USBIFR0       0xA4480018
#define USBIFR1       0xA448001C
#define USBTRG        0xA4480020
#define USBFCLR       0xA4480024
#define USBEPSZ0O     0xA4480028
#define USBDASTS      0xA448002C
#define USBEPSTL      0xA4480030
#define USBIER0       0xA4480034
#define USBIER1       0xA4480038
#define USBEPSZ1      0xA448003C
#define USBDMA        0xA4480040
#define USBISR0       0xA4480044
#define USBISR1       0xA4480048

#define USBXVERCR     0xA4480060

#define STBCR3        0xA40A0000
#define UCLKCR        0xA40A0008

#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
#define USBEPDR0I     0xA4000242
#define USBEPDR0O     0xA4000243
#define USBEPDR0S     0xA4000247
#define USBEPDR1      0xA400024E
#define USBEPDR2      0xA4000249
#define USBEPDR3      0xA4000252
#define USBIFR0       0xA4000240
#define USBIFR1       0xA4000241
#define USBTRG        0xA4000244
#define USBFCLR       0xA4000245
#define USBEPSZ0O     0xA4000246
#define USBDASTS      0xA4000248
#define USBEPSTL      0xA400024B
#define USBIER0       0xA400024C
#define USBIER1       0xA400024D
#define USBEPSZ1      0xA400024F
#define USBDMA        0xA4000251
#define USBISR0       0xA400024A
#define USBISR1       0xA4000250

#define EXCPGCR	      0xA4000236
#define EXPFC	      0xA4000234
#define STBCR3        0xA4000230
#endif

/*
 * Standby Control Register (STBCR3) c.f. 9.2.3
 */

#if defined(CONFIG_CPU_SUBTYPE_SH7705)
#define MSTP37          0x80
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
#define MSTP14          0x10
#endif

/*
 * EXCPG Control Register (EXCPGCR) c.f. Section 11.3.1
 */

#define USBDIVS_EL0	0x00
#define USBDIVS_EL1	0x01
#define USBDIVS_EL2	0x02

#define	USBCKS_EL1	0x04
#define	USBCKS_EL2	0x10
#define	USBCKS_EL3	0x20

#define USBDIV_11	0x00
#define USBDIV_12	0x01
#define USBDIV_13	0x02

#define	USBCKS_PC	0x00
#define	USBCKS_IC	0x20
#define	USBCKS_BC	0x24
#define	USBCKS_EC	0x30


/*
 * Extra Pin Function Controller (EXPFC) c.f. Section 22.2.1
 */

#define	USB_TRANS_TRAN	0x00
#define	USB_TRANS_DIG	0x02

#define	USB_SEL_HOST	0x00
#define	USB_SEL_FUNC	0x01


/*
 * USBDMA Setting Register (USBDMAR) c.f. Section 23.5.19
 */

#define EP1_DMAE        0x01
#define EP2_DMAE        0x02

#if defined(CONFIG_CPU_SUBTYPE_SH7705)
#if defined(CONFIG_SH_EDOSK7705)
#define PULLUP_E        0x01
#elif defined(CONFIG_SH_SOLUTION_ENGINE)
#define PULLUP_E        0x02 /* PTD1 */
#endif
#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
#define PULLUP_E        0x04
#endif

/*
 * USB Interrupt Flag Register 0 (USBIFR0) c.f. Section 23.5.7
 */

#define BRST            0x80
#define EP1_FULL        0x40
#define EP2_TR          0x20
#define EP2_EMPTY       0x10
#define SETUP_TS        0x08
#define EP0o_TS         0x04
#define EP0i_TR         0x02
#define EP0i_TS         0x01


/*
 * USB Interrupt Flag Register 1 (USBIFR1) c.f. Section 23.5.8
 */

#define VBUSMN          0x08
#define EP3_TR          0x04
#define EP3_TS          0x02
#define VBUSF           0x01

/*
 * USB Trigger Register (USBTRG) c.f. Section 23.5.9
 */

#define EP0i_PKTE       0x01
#define EP0o_PKTE       0x02
#define EP0o_RDFN       0x02
#define EP0s_PKTE       0x04
#define EP0s_RDFN       0x04

#define EP2_PKTE        0x10
#define EP1_PKTE        0x20
#define EP1_RDFN        0x20
#define EP3_PKTE        0x40


/*
 * USBFIFO Clear Register (USBFCLR) c.f. Section 23.5.10
 */

#define EP3_CLEAR       0x40
#define EP1_CLEAR       0x20
#define EP2_CLEAR       0x10
#define EP0o_CLEAR      0x02
#define EP0i_CLEAR      0x01


/*
 * USBEPSTL Endpoint Stall Register
 */
#define EP3_STL      0x08
#define EP2_STL      0x04
#define EP1_STL      0x02
#define EP0_STL      0x01

/*
 * USBDASTS Data Status Register
 */
#define EP3_DE       0x20
#define EP2_DE       0x10
#define EP0i_DE      0x01

/*
 * Port Control Registers (PxCR) c.f. Section 26.2
 */
#define PN_PB0_OF       0x0000
#define PN_PB0_PO       0x0001
#define PN_PB0_PI_ON    0x0002
#define PN_PB0_PI_OFF   0x0003
#define PN_PB0_MSK     ~0x0003

#define PN_PB1_OF       0x0000
#define PN_PB1_PO       0x0004
#define PN_PB1_PI_ON    0x0008
#define PN_PB1_PI_OFF   0x000c
#define PN_PB1_MSK     ~0x000c

#define PN_PB2_OF       0x0000
#define PN_PB2_PO       0x0010
#define PN_PB2_PI_ON    0x0020
#define PN_PB2_PI_OFF   0x0030
#define PN_PB2_MSK     ~0x0030

#define PN_PB3_OF       0x0000
#define PN_PB3_PO       0x0040
#define PN_PB3_PI_ON    0x0080
#define PN_PB3_PI_OFF   0x00c0
#define PN_PB3_MSK     ~0x00c0

#define PN_PB4_OF       0x0000
#define PN_PB4_PO       0x0100
#define PN_PB4_PI_ON    0x0200
#define PN_PB4_PI_OFF   0x0300
#define PN_PB4_MSK     ~0x0300

#define PN_PB5_OF       0x0000
#define PN_PB5_PO       0x0400
#define PN_PB5_PI_ON    0x0800
#define PN_PB5_PI_OFF   0x0c00
#define PN_PB5_MSK     ~0x0c00

#define PN_PB6_OF       0x0000
#define PN_PB6_PO       0x1000
#define PN_PB6_PI_ON    0x2000
#define PN_PB6_PI_OFF   0x3000
#define PN_PB6_MSK     ~0x3000

#define PN_PB7_OF       0x0000
#define PN_PB7_PO       0x4000
#define PN_PB7_PI_ON    0x8000
#define PN_PB7_PI_OFF   0xc000
#define PN_PB7_MSK     ~0xc000

/*
 * Debugging support vanishes in non-debug builds.  DBG_NORMAL should be
 * mostly silent during normal use/testing, with no timing side-effects.
 */
#define DBG_NORMAL	1	/* error paths, device state transitions */
#define DBG_VERBOSE	2	/* add some success path trace info */
#define DBG_NOISY	3	/* ... even more: request level */
#define DBG_VERY_NOISY	4	/* ... even more: packet level */

#ifdef DEBUG

#define DMSG(stuff...) printk(KERN_DEBUG "udc: " stuff)

#if defined(VERY_NOISY)
#    define UDC_DEBUG DBG_VERY_NOISY
#elif defined(NOISY)
#    define UDC_DEBUG DBG_NOISY
#elif defined(VERBOSE)
#    define UDC_DEBUG DBG_VERBOSE
#else
#    define UDC_DEBUG DBG_NORMAL
#endif

static void __attribute__ ((__unused__))
dump_state(struct superh_udc *dev)
{
	if (!is_usb_connected)
		return;
}


#else

#define DMSG(stuff...)		do{}while(0)

#define UDC_DEBUG ((unsigned)0)

#define	dump_state(x)	do{}while(0)

#endif

#define DBG(lvl, stuff...) do{if ((lvl) <= UDC_DEBUG) DMSG(stuff);}while(0)

#define WARN(stuff...) printk(KERN_WARNING "udc: " stuff)

#ifndef likely
#define likely(x)       (x)
#define unlikely(x)     (x)
#endif

#ifndef BUG_ON
#define BUG_ON(condition) do { if (unlikely((condition)!=0)) BUG(); } while(0)
#endif

#endif /* __LINUX_USB_GADGET_SUPERH_UDC_H */
