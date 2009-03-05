/*
 * arch/mips/tx4939/common/rtc.c
 *
 * RTC routines for TX4939.
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <asm/time.h>
#include <asm/tx4939/tx4939.h>

static void write_rtc_time(u64 data)
{
	int i;
	u32 l;

	/* Write to RWB. */
	reg_wr32(&tx4939_rtcptr->adr, 0);
	for (i = 0; i < 6; i++) {
		reg_wr32(&tx4939_rtcptr->dat, (u8) data);
		data >>= 8;
	}

	/* RWB -> RTC. */
	l = reg_rd32(&tx4939_rtcptr->ctl);
	reg_wr32(&tx4939_rtcptr->ctl, l & TX4939_RTCCTL_CMD_CLEAR);
	l = reg_rd32(&tx4939_rtcptr->ctl);
	reg_wr32(&tx4939_rtcptr->ctl, l | TX4939_RTCCTL_CMD_SETTIME);

	/* Wait for completion. */
	while (reg_rd32(&tx4939_rtcptr->ctl) & TX4939_RTCCTL_BUSY) ;
}

static u64 read_rtc_time(void)
{
	int i;
	u32 l;

	u64 data = 0;

	/* RWB <- RTC. */
	l = reg_rd32(&tx4939_rtcptr->ctl);
	reg_wr32(&tx4939_rtcptr->ctl, l & TX4939_RTCCTL_CMD_CLEAR);
	l = reg_rd32(&tx4939_rtcptr->ctl);
	reg_wr32(&tx4939_rtcptr->ctl, l | TX4939_RTCCTL_CMD_GETTIME);

	/* Wait for completion. */
	while (reg_rd32(&tx4939_rtcptr->ctl) & TX4939_RTCCTL_BUSY) ;

	/* Read from RWB. */
	for (i = 0; i < 6; i++)
		data |= (u64) (reg_rd32(&tx4939_rtcptr->dat)) << (8 * i);

	return data;
}

/**
 * tx4939_rtc_set_time - set time to TX4939 rtc
 * @t: set time
 */

static int tx4939_rtc_set_time(unsigned long t)
{
	write_rtc_time((u64) t << 16);
	return 0;
}

/**
 * tx4939_rtc_get_time - get time from TX4939 rtc
 */

static unsigned long tx4939_rtc_get_time(void)
{
	return (read_rtc_time() >> 16);
}

/**
 * tx4939_rtc_init - set the rtc function pointers
 */

void __init tx4939_rtc_init(void)
{
	rtc_get_time = tx4939_rtc_get_time;
	rtc_set_time = tx4939_rtc_set_time;
}
