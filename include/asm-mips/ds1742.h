/*
 * Machine dependent access functions for RTC registers.
 *
 * Do not include this file directly. It's included from linux/ds1742rtc.h
 *
 * Author: Vladimir Barinov <vbarinov@ru.mvista.com>
 *
 * (c) 2005 MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __LINUX_DS1742_H
#define __LINUX_DS1742_H

extern unsigned long rtc_base;

static inline unsigned char CMOS_READ(unsigned long addr)
{
	return (*(volatile u8 *)(rtc_base + addr));
}

static inline void CMOS_WRITE(unsigned char data, unsigned long addr)
{
	*(volatile u8 *)(rtc_base + addr) = data;
}

#endif /* __LINUX_DS1742_H */
