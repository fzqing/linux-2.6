/*******************************************************************
 * mpc8272_usb_hw.c 2004/04/20
 *
 * mpc8272ads reference board usb hardware initialization
 *
 * Author: Vitaly Bordug <source@mvista.com>
 *
 * Copyright (c) 2004 Motorola, Dave Liu (Daveliu@motorola.com)
 *
 * Copyright (c) 2005 MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
  *******************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
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
#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>

#include <asm/byteorder.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/immap_cpm2.h>
#include <asm/cpm2.h>
#include <asm/mpc8260.h>
#include "mpc8272_udc.h"

#define USB_HIGHSPEED            1
#define USB_LOWSPEED             0
#define USB_VCC_SUPPLY           1
#define USB_VCC_NOT_SUPPLY       0
#define USB_ENABLE               1
#define USB_DISABLE              0

static void
mpc8272_usb_clock_cmxscr_set(cpm2_map_t * immr)
{
	unsigned int tmp, cpm_mux_cmxscr;

	cpm_mux_cmxscr = immr->im_cpmux.cmx_scr;
#ifdef CONFIG_PQ2FADS
	/* 111 SCC4 receive clock is CLK8, 111 SCC4 transmit/USB clock is CLK8 */
	tmp = 0x0000003f;
	cpm_mux_cmxscr &= ~0x0000007f;
#else
	/* 111 SCC3 receive clock is CLK8, 111 SCC3 transmit/USB clock is CLK8 */
	tmp = 0x00003f00;
	cpm_mux_cmxscr &= ~0x00ff3f00;
#endif
	cpm_mux_cmxscr |= tmp;
	immr->im_cpmux.cmx_scr = cpm_mux_cmxscr;

}

static void
mpc8272_usb_clock_io_pin_connect(cpm2_map_t * immr)
{
	unsigned int tmp;

	/* USB clock use port c */
	tmp = immr->im_ioport.iop_pdirc;
	immr->im_ioport.iop_pdirc = tmp & ~0x00000080;
	tmp = immr->im_ioport.iop_pparc;
	immr->im_ioport.iop_pparc = tmp | 0x00000080;
	tmp = immr->im_ioport.iop_psorc;
	immr->im_ioport.iop_psorc = tmp & ~0x00000080;

}

static void
mpc8272_usb_clock_config(cpm2_map_t * immr)
{
	mpc8272_usb_clock_cmxscr_set(immr);
	mpc8272_usb_clock_io_pin_connect(immr);
}

static void
mpc8272_usb_io_pin_config(cpm2_map_t * immr)
{
	unsigned int tmp = 0;

#ifdef CONFIG_PQ2FADS
	/*
	   port      signal    par    sor    odr    dir

	   pd[20]      TP        1      0      0      1
	   pd[21]      TN        1      0      0      1
	   pd[22]      RXD       1      0      0      0
	   pc[8]       RN        1      0      1      0
	   pc[9]       RP        1      0      1      0
	   pc[20]      OE        1      0      0      1

	*/

	tmp = immr->im_ioport.iop_pdird;
	immr->im_ioport.iop_pdird = tmp & ~0x00000200;	/* pdird[22] = 0 */
	tmp = immr->im_ioport.iop_ppard;
	immr->im_ioport.iop_ppard = tmp | 0x00000200;	/* ppard[22] = 1 */
	tmp = immr->im_ioport.iop_pdirc;
	immr->im_ioport.iop_pdirc = tmp & ~0x00c00000;	/* pdirc[8,9] = 0 */
	tmp = immr->im_ioport.iop_pparc;
	immr->im_ioport.iop_pparc = tmp | 0x00c00000;	/* pparc[8,9] = 1 */
	tmp = immr->im_ioport.iop_psorc;
	immr->im_ioport.iop_psorc = tmp & ~0x00c00000;	/* psorc[8,9] = 0 */
	tmp = immr->im_ioport.iop_podrc;
	immr->im_ioport.iop_podrc = tmp | 0x00c00000;	/* podrc[8,9] = 1 */
	tmp = immr->im_ioport.iop_pdird;
	immr->im_ioport.iop_pdird = tmp | 0x00000c00;	/* pdird[20,21] = 1 */
	tmp = immr->im_ioport.iop_ppard;
	immr->im_ioport.iop_ppard = tmp | 0x00000c00;	/* ppard[20,21] = 1 */
	tmp = immr->im_ioport.iop_psord;
	immr->im_ioport.iop_psord = tmp & ~0x00000e00;	/* psord[20,21,22] = 0 */
	tmp = immr->im_ioport.iop_pdirc;
	immr->im_ioport.iop_pdirc = tmp | 0x00000800;	/* pdirc[20] = 1 */
	tmp = immr->im_ioport.iop_pparc;
	immr->im_ioport.iop_pparc = tmp | 0x00000800;	/* pparc[20] = 1 */
#else
	/*
	   port      signal    par    sor    odr    dir

	   pd[23]      TP        1      0      0      1
	   pd[24]      TN        1      0      0      1
	   pd[25]      RXD       1      0      0      0
	   pc[10]      RN        1      0      1      0
	   pc[11]      RP        1      0      1      0
	   pc[20]      OE        1      0      0      1

	 */

	tmp = immr->im_ioport.iop_pdird;
	immr->im_ioport.iop_pdird = tmp & ~0x00000040;	/* pdird[25] = 0 */
	tmp = immr->im_ioport.iop_ppard;
	immr->im_ioport.iop_ppard = tmp | 0x00000040;	/* ppard[25] = 1 */
	tmp = immr->im_ioport.iop_pdirc;
	immr->im_ioport.iop_pdirc = tmp & ~0x00300000;	/* pdirc[10,11] = 0 */
	tmp = immr->im_ioport.iop_pparc;
	immr->im_ioport.iop_pparc = tmp | 0x00300000;	/* pparc[10,11] = 1 */
	tmp = immr->im_ioport.iop_psorc;
	immr->im_ioport.iop_psorc = tmp & ~0x00300000;	/* psorc[10,11] = 0 */
	tmp = immr->im_ioport.iop_podrc;
	immr->im_ioport.iop_podrc = tmp | 0x00300000;	/* podrc[10,11] = 1 */
	tmp = immr->im_ioport.iop_pdird;
	immr->im_ioport.iop_pdird = tmp | 0x00000180;	/* pdird[23,24] = 1 */
	tmp = immr->im_ioport.iop_ppard;
	immr->im_ioport.iop_ppard = tmp | 0x00000180;	/* ppard[23,24] = 1 */
	tmp = immr->im_ioport.iop_psord;
	immr->im_ioport.iop_psord = tmp & ~0x000001c0;	/* psord[23,24,25] = 0 */
	tmp = immr->im_ioport.iop_pdirc;
	immr->im_ioport.iop_pdirc = tmp | 0x00000800;	/* pdirc[20] = 1 */
	tmp = immr->im_ioport.iop_pparc;
	immr->im_ioport.iop_pparc = tmp | 0x00000800;	/* pparc[20] = 1 */
#endif

}

void
mpc8272_usb_hw_config(cpm2_map_t * immr)
{
	mpc8272_usb_clock_config(immr);
	mpc8272_usb_io_pin_config(immr);
}

void
mpc8272_board_usb_iface_speed(int speed)
{
	unsigned int *bscr3;
	unsigned int bscr3_val;
	bscr3 = (unsigned int *) (BCSR_ADDR + 0x0c);
	bscr3_val = *bscr3;

	if (speed == USB_HIGHSPEED)
		bscr3_val &= ~BCSR3_USB_LOW_SPEED;
	else
		bscr3_val |= BCSR3_USB_LOW_SPEED;

	*bscr3 = bscr3_val;

}

void
mpc8272_board_usb_iface_vcc_supply(int supply)
{
	unsigned int *bscr3;
	unsigned int bscr3_val;
	bscr3 = (unsigned int *) (BCSR_ADDR + 0x0c);
	bscr3_val = *bscr3;

	if (supply == USB_VCC_SUPPLY)
		bscr3_val |= BCSR3_USB_SUPPLY_VCC5V;
	else
		bscr3_val &= ~BCSR3_USB_SUPPLY_VCC5V;

	*bscr3 = bscr3_val;
}

void
mpc8272_board_usb_iface_enable(int enable)
{
	unsigned int *bscr3;
	unsigned int bscr3_val;
	bscr3 = (unsigned int *) (BCSR_ADDR + 0x0c);
	bscr3_val = *bscr3;

	if (enable == USB_ENABLE)
		bscr3_val &= ~BCSR3_USB_DISABLE;
	else
		bscr3_val |= BCSR3_USB_DISABLE;

	*bscr3 = bscr3_val;
}

void
mpc8272_board_usb_iface_config(void)
{
	/* high speed, vcc supply disable, usb enable */
	mpc8272_board_usb_iface_speed(USB_HIGHSPEED);
	mpc8272_board_usb_iface_vcc_supply(USB_VCC_NOT_SUPPLY);
	mpc8272_board_usb_iface_enable(USB_ENABLE);

}

unsigned short
mpc8272_dpram_offset(void *addr)
{
	unsigned long base = (unsigned long) (CPM_MAP_ADDR);
	unsigned long offset = (unsigned long) ((unsigned long) addr - base);
	return (unsigned short) offset;
}

void
mpc8272_board_usb_iface_deconfig(void)
{
	mpc8272_board_usb_iface_enable(USB_DISABLE);
}
