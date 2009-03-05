/*
 * arch/mips/tx4939/common/ide.c
 *
 * tx4939 ide setup routines
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/ide.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/tx4939/tx4939.h>

/**
 * tx4939_ide_init - initialize TX4939 IDE controller
 *
 * This function initiates TX4939 IDE controller.
 */

void __init tx4939_ide_setup(int ch)
{
	unsigned long status_base;
	u16 s;
	int i;

	status_base = (TX4939_ATA_REG(ch) + offsetof(struct tx4939_ata_reg, alt_devctl)
		       - mips_io_port_base);

	/* Soft Reset */
	s = reg_rd16(&tx4939_ataptr(ch)->sysctl);
	reg_wr16(&tx4939_ataptr(ch)->sysctl, s | TX4939_ATA_SC_SOFT_RESET);
	wbflush();
	udelay(1);	/* wait for soft reset */

	/* FIFO Reset */
	s = reg_rd16(&tx4939_ataptr(ch)->sysctl);
	reg_wr16(&tx4939_ataptr(ch)->sysctl, s | TX4939_ATA_SC_FIFO_RESET);
	wbflush();
	udelay(1);

	/* ATA Hard Reset */
	s = reg_rd16(&tx4939_ataptr(ch)->sysctl);
	reg_wr16(&tx4939_ataptr(ch)->sysctl, s | TX4939_ATA_SC_ATA_HARD_RESET);
	wbflush();
	mdelay(3);	/* wait for device */

	/* Wait for Status(BSY) to be cleared (Timeout: 60s=50us*1200000) */
	for (i = 0; (inb(status_base) & BUSY_STAT) && i < 1200000; i++)
		udelay(50);

	/* Clear Interrupt status */
	s = reg_rd16(&tx4939_ataptr(ch)->int_ctl);
	reg_wr16(&tx4939_ataptr(ch)->int_ctl, s & TX4939_ATA_IC_TIMERINT);

	/* Set Lower Burst Count Register */
	reg_wr16(&tx4939_ataptr(ch)->lo_bcnt, 0x0008);
	reg_wr16(&tx4939_ataptr(ch)->hi_bcnt, 0x0000);
}
