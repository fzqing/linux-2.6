/*
 * low level RTC hook up functions for Dallas Semiconductor 1338 chip 
 * This depends on the ds1338 driver to be compiled into the kernel.
 * 
 * Author: Sekhar Nori, nsekhar@ti.com 
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
	
#include <linux/time.h>
#include <asm/time.h>
	
#define DS1338_GET_DATE		0
#define DS1338_SET_DATE		1
	
int ds1338_do_command(int id, int cmd, void *arg);
	
static unsigned long rtc_ds1338_get_time(void)
{
	struct rtc_time date;        
	
	if(ds1338_do_command(0, DS1338_GET_DATE, &date) == -ENODEV)
		return mktime(2000, 1, 1, 0, 0, 0);
	
	return mktime(date.tm_year, date.tm_mon + 1, date.tm_mday, date.tm_hour, date.tm_min, date.tm_sec);
}
	
static int rtc_ds1338_set_time(unsigned long t)
{
	struct rtc_time tm;

	to_tm(t, &tm);
	
	if(ds1338_do_command(0, DS1338_SET_DATE, &tm) == -ENODEV)
		return -ENODEV;             
	
	return 0;
}
	
void ds1338_time_init(void)
{
	rtc_get_time = rtc_ds1338_get_time;
	rtc_set_time = rtc_ds1338_set_time;
	rtc_set_mmss = rtc_set_time;
}
