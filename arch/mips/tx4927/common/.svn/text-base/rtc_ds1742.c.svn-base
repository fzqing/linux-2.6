/*
 * arch/mips/tx4927/common/rtc_ds1742.c 
 * 
 * This is a copy of:  arch/mips/jmr3927/common/rtc_ds1742.c
 *
 * Copyright (c) 2001-2005 MontaVista Software Inc.
 * Copyright (c) 2000-2001 Toshiba Corporation 
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/types.h>
#include <linux/ds1742rtc.h>

#include <asm/time.h>
#include <asm/delay.h>
#include <asm/debug.h>

#define	EPOCH		2000

#undef BCD_TO_BIN
#define BCD_TO_BIN(val) (((val)&15) + ((val)>>4)*10)

#undef BIN_TO_BCD
#define BIN_TO_BCD(val) ((((val)/10)<<4) + (val)%10)

unsigned long rtc_base;

/* RTC-dependent code for time.c */

static unsigned long rtc_ds1742_get_time(void)
{
	unsigned int year, month, day, hour, minute, second;
	unsigned int century;
	static unsigned int save = 0;

	CMOS_WRITE(RTC_READ, RTC_CONTROL);
	second 	= BCD_TO_BIN(CMOS_READ(RTC_SECONDS) & RTC_SECONDS_MASK);
	minute 	= BCD_TO_BIN(CMOS_READ(RTC_MINUTES));
	hour 	= BCD_TO_BIN(CMOS_READ(RTC_HOURS));
	day 	= BCD_TO_BIN(CMOS_READ(RTC_DATE));
	month 	= BCD_TO_BIN(CMOS_READ(RTC_MONTH));
	year 	= BCD_TO_BIN(CMOS_READ(RTC_YEAR));
	century = BCD_TO_BIN(CMOS_READ(RTC_CENTURY) & RTC_CENTURY_MASK);
	CMOS_WRITE(0, RTC_CONTROL);

	/* manual -- must wait 500us min between RTC_READ clr and next set */
	if (save != second)
		save = second;
	else
		udelay(500);

	year += EPOCH;

	return mktime(year, month, day, hour, minute, second);
}

static int rtc_ds1742_set_time(unsigned long t)
{
	struct rtc_time tm;
	u8 year, month, day, hour, minute, second;
	u8 cmos_year, cmos_month, cmos_day, cmos_hour, cmos_minute, cmos_second;
	int cmos_century;

	CMOS_WRITE(RTC_READ, RTC_CONTROL);
	cmos_second  = (u8) (CMOS_READ(RTC_SECONDS) & RTC_SECONDS_MASK);
	cmos_minute  = (u8) CMOS_READ(RTC_MINUTES);
	cmos_hour    = (u8) CMOS_READ(RTC_HOURS);
	cmos_day     = (u8) CMOS_READ(RTC_DATE);
	cmos_month   = (u8) CMOS_READ(RTC_MONTH);
	cmos_year    = (u8) CMOS_READ(RTC_YEAR);
	cmos_century = CMOS_READ(RTC_CENTURY) & RTC_CENTURY_MASK;
	CMOS_WRITE(RTC_WRITE, RTC_CONTROL);

	/* convert */
	to_tm(t, &tm);

	/* check each field one by one */
	year = BIN_TO_BCD(tm.tm_year - EPOCH);
	if (year != cmos_year)
		CMOS_WRITE(year, RTC_YEAR);

	month = BIN_TO_BCD(tm.tm_mon + 1);
	if (month != (cmos_month & 0x1f))
		CMOS_WRITE((month & 0x1f) | (cmos_month & ~0x1f), RTC_MONTH);

	day = BIN_TO_BCD(tm.tm_mday);
	if (day != cmos_day)
		CMOS_WRITE(day, RTC_DATE);

	if (cmos_hour & 0x40) {
		/* 12 hour format */
		hour = 0x40;
		if (tm.tm_hour > 12) {
			hour |= 0x20 | (BIN_TO_BCD(hour - 12) & 0x1f);
		} else {
			hour |= BIN_TO_BCD(tm.tm_hour);
		}
	} else {
		/* 24 hour format */
		hour = BIN_TO_BCD(tm.tm_hour) & 0x3f;
	}

	if (hour != cmos_hour)
		CMOS_WRITE(hour, RTC_HOURS);

	minute = BIN_TO_BCD(tm.tm_min);
	if (minute != cmos_minute)
		CMOS_WRITE(minute, RTC_MINUTES);

	second = BIN_TO_BCD(tm.tm_sec);
	if (second != cmos_second)
		CMOS_WRITE(second & RTC_SECONDS_MASK, RTC_SECONDS);

	/* RTC_CENTURY and RTC_CONTROL share same address... */
	CMOS_WRITE(cmos_century, RTC_CONTROL);
	return 0;
}

void rtc_ds1742_init(unsigned long base)
{
	u8 cmos_second;

	/* remember the base */
	rtc_base = base;
	db_assert((rtc_base & 0xe0000000) == KSEG1);

	/* clear oscillator stop bit */
	CMOS_WRITE(RTC_READ, RTC_CONTROL);
	cmos_second = (u8) (CMOS_READ(RTC_SECONDS) & RTC_SECONDS_MASK);
	CMOS_WRITE(RTC_WRITE, RTC_CONTROL);
	CMOS_WRITE(cmos_second, RTC_SECONDS);	/* clear msb */
	CMOS_WRITE(0, RTC_CONTROL);

	/* set the function pointers */
	rtc_get_time = rtc_ds1742_get_time;
	rtc_set_time = rtc_ds1742_set_time;
}
