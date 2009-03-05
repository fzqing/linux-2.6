/*
 * arch/mips/vr5701/common/rtc_rx5c348.c Version 0.02 April 11, 2005
 *
 * A Real Time Clock interface for Linux on NEC Electronics Corporation VR5701 SolutionGearII
 * (RICOH Co., Ltd., Rx5C348B)
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/bcd.h>

#include <asm/time.h>
#include <asm/addrspace.h>
#include <asm/delay.h>
#include <asm/debug.h>
#include <asm/vr5701/vr5701_sg2.h>

#undef  DEBUG
#undef RTC_DELAY

void static rtc_set_ce(u32 val)
{
	pr_debug("rtc_set_ce(%d)\n", val);
	reg_set32(GIU_PIO0, GPIO_4_CE, val ? SET_32_BIT : 0);
#ifdef RTC_DELAY
	__delay(100000);
#endif
}

void static rtc_write_burst(int adr, unsigned char *data, int dataLen)
{
	int i;
	for (i = 0; i < dataLen; i++)
		pr_debug(" rtc_write_burst : data=%08x\n", data[i]);
	pr_debug(" rtc_write_burst : adr=0x%02x\n", adr);
	csi1_reset();
	while (io_in32(CSI1_MODE) & CSIn_MODE_CSOT) ;
	reg_set32(CSI1_MODE, CSIn_MODE_AUTO, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_TRMD, SET_32_BIT);
	io_out32(CSI1_INT, CSIn_INT_CSIEND);
	rtc_set_ce(1);

	pr_debug(" rtc_write_burst : CSI1_MODE=%08x\n", io_in32(CSI1_MODE));
	pr_debug(" rtc_write_burst : CSI1_CNT=%08x\n", io_in32(CSI1_CNT));
	io_out32(CSI1_SOTBF, ((adr << 4) | 0x00));

	for (i = 0; i < dataLen; i++) {
		io_out32(CSI1_SOTB, data[i]);
		while (!(io_in32(CSI1_INT) & CSIn_INT_CSIEND)) ;
		io_out32(CSI1_INT, CSIn_INT_CSIEND);
	}
	while (io_in32(CSI1_MODE) & CSIn_MODE_CSOT) ;
	rtc_set_ce(0);
}

void static rtc_read_burst(int adr, unsigned char *data, int dataLen)
{
	int i;
	pr_debug(" rtc_read_burst : adr=0x%02x\n", adr);
	while (io_in32(CSI1_MODE) & CSIn_MODE_CSOT) ;
	reg_set32(CSI1_MODE, CSIn_MODE_AUTO, CLR_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_TRMD, SET_32_BIT);
	io_out32(CSI1_INT, CSIn_INT_CSIEND);
	rtc_set_ce(1);

	pr_debug(" rtc_read_burst : CSI1_MODE=%08x\n", io_in32(CSI1_MODE));
	pr_debug(" rtc_read_burst : CSI1_CNT=%08x\n", io_in32(CSI1_CNT));
	io_out32(CSI1_SOTB, (((adr & 0xf) << 4) | 0x04));
	while (!(io_in32(CSI1_INT) & CSIn_INT_CSIEND)) ;

	while (io_in32(CSI1_MODE) & CSIn_MODE_CSOT) ;
	reg_set32(CSI1_MODE, CSIn_MODE_TRMD, CLR_32_BIT);
	io_out32(CSI1_INT, CSIn_INT_CSIEND);

	udelay(50);
	pr_debug(" rtc_read_burst : CSI1_MODE=%08x\n", io_in32(CSI1_MODE));
	pr_debug(" rtc_read_burst : CSI1_CNT=%08x\n", io_in32(CSI1_CNT));
	io_in32(CSI1_SIRB);	/* dummy read */

	for (i = 0; i < dataLen; i++) {
		while (!(io_in32(CSI1_INT) & CSIn_INT_CSIEND)) ;
		io_out32(CSI1_INT, CSIn_INT_CSIEND);
		data[i] = io_in32(CSI1_SIRB);
	}
	while (io_in32(CSI1_MODE) & CSIn_MODE_CSOT) ;
	rtc_set_ce(0);
	for (i = 0; i < dataLen; i++)
		pr_debug(" rtc_read_burst : data=%08x\n", data[i]);
}

static unsigned long rtc_ricoh_rx5c348_get_time(void)
{
	u8 date[7];
	unsigned int year, month, day, hour, minute, second;

	rtc_read_burst(0, date, sizeof(date));

	year = BCD2BIN(date[6]) + (date[5] & 0x80 ? 2000 : 1900);
	month = BCD2BIN(date[5] & 0x1f);
	day = BCD2BIN(date[4]);
	hour = BCD2BIN(date[2]);
	minute = BCD2BIN(date[1]);
	second = BCD2BIN(date[0]);

	pr_debug(KERN_INFO
		 "rtc_ricoh_rx5c348_get_time: %d/%02d/%02d %02d:%02d:%02d\n",
		 year, month, day, hour, minute, second);
	return mktime(year, month, day, hour, minute, second);
}

static int rtc_ricoh_rx5c348_set_time(unsigned long t)
{
	u8 date[7];
	struct rtc_time tm;

	to_tm(t, &tm);
	date[0] = BIN2BCD(tm.tm_sec);
	date[1] = BIN2BCD(tm.tm_min);
	date[2] = BIN2BCD(tm.tm_hour);
	date[4] = BIN2BCD(tm.tm_mday);
	date[5] = BIN2BCD(tm.tm_mon + 1) + (tm.tm_year > 1999 ? 0x80 : 0);
	date[6] =
	    BIN2BCD(tm.tm_year > 1999 ? tm.tm_year - 2000 : tm.tm_year - 1900);

	rtc_write_burst(0, date, 3);
	rtc_write_burst(4, date + 4, 3);

	pr_debug(KERN_INFO
		 "rtc_ricoh_rx5c348_set_time:t=%ld %d/%02d/%02d %02d:%02d:%02d\n",
		 t, tm.tm_year, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
		 tm.tm_min, tm.tm_sec);
	return 0;
}

static int __devinit rtc_ricoh_rx5c348_init(void)
{
	unsigned char data;
	/* CSI1 reset  */
	io_set16(PIB_RESET, 0x40, 0xffff);
	__delay(10000);
	io_set16(PIB_RESET, 0x40, 0x0000);

	/* set GPIO3 , GPIO4 */
	reg_set32(GIU_FUNCSEL0, (GPIO_4_CE | GPIO_3_INTR), SET_32_BIT);
	/* clear GPIO25 , GPIO26 , GPIO27 */
	reg_set32(GIU_FUNCSEL0, GPIO_CSI1_PIN, CLR_32_BIT);
	/* make GPIO4 output */
	reg_set32(GIU_DIR0, GPIO_4_CE, SET_32_BIT);
	/* make GPIO3 input  */
	reg_set32(GIU_DIR0, GPIO_3_INTR, CLR_32_BIT);

	csi1_reset();

	rtc_read_burst(0x0e, &data, 1);
	if ((data & 0x20) == 0) {	/* 24 hour */
		data |= 0x20;
		rtc_write_burst(0x0e, &data, 1);
#ifdef RTC_DELAY
		__delay(10000);
#endif
	}

	/* set the function pointers */
	rtc_get_time = rtc_ricoh_rx5c348_get_time;
	rtc_set_time = rtc_ricoh_rx5c348_set_time;
	return 0;
}

module_init(rtc_ricoh_rx5c348_init);

MODULE_AUTHOR("Sergey Podstavin");
MODULE_DESCRIPTION("Real Time Clock interface for Linux on NEC Electronics Corporation VR5701 SolutionGearII");
MODULE_LICENSE("GPL");
