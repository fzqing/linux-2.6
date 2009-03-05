/*
 * arch/xtensa/platform-xt2000/i2c.c
 *
 * V320 I2C.
 *
 * Copyright (C) 2001 - 2004 Tensilica Inc.
 *
 * Kevin Chea <kchea@tensilica.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/time.h>
#include <linux/delay.h>
#include <asm/platform.h>
#include <asm/platform/xt2000.h>

#if XCHAL_HAVE_BE
#define DEFAULT_BASE XTBOARD_V3PCI_PADDR+(0x73^3)
#else
#define DEFAULT_BASE XTBOARD_V3PCI_PADDR+(0x73)
#endif

#define V3USC_SYSTEM_B          *(volatile unsigned char *)(DEFAULT_BASE)
#define V3USC_SYS_OUT(__byte__) V3USC_SYSTEM_B = (__byte__)
#define V3USC_SYS_IN()          V3USC_SYSTEM_B

#define SYSTEM_B_SPROM_EN       0x01    /* 1 - Software control         */
                                        /* 0 - Hardware control         */
#define SYSTEM_B_SDA_IN         0x02    /* Serial EEPROM data input     */
#define SYSTEM_B_SDA_IN_SHIFT   1
#define SYSTEM_B_SDA_OUT        0x04    /* Serial EEPROM data output    */
                                        /* SPROM_EN must be enabled     */
#define SYSTEM_B_SCL            0x08    /* Serial EEPROM clock output   */
#define SYSTEM_B_SCL_IN_SHIFT   3
#define SYSTEM_B_LOCK		0x40	/* Lock Register Contents set	*/ 
#define SYSTEM_B_UNLOCK_TOKEN   0xa5

void *memcpy(void *, const void *, unsigned);

#define bit_delay() udelay(10)

static void bit_xt2000_setscl(int state)
{
        if (state)
                V3USC_SYS_OUT(V3USC_SYS_IN() | SYSTEM_B_SCL);
        else
                V3USC_SYS_OUT(V3USC_SYS_IN() & ~SYSTEM_B_SCL);
	bit_delay();
}

static void bit_xt2000_setsda(int state)
{
        if (state)
                V3USC_SYS_OUT(V3USC_SYS_IN() | SYSTEM_B_SDA_OUT);
        else
                V3USC_SYS_OUT(V3USC_SYS_IN() & ~SYSTEM_B_SDA_OUT);
	bit_delay();
}

static int bit_xt2000_getsda(void)
{
        return ((V3USC_SYS_IN() >> SYSTEM_B_SDA_IN_SHIFT) & 1);
}

static void i2c_start(void)
{
	bit_xt2000_setsda(1);
	bit_xt2000_setscl(1);
	bit_xt2000_setsda(0);
	bit_xt2000_setscl(0);
}

static void i2c_stop(void)
{
	bit_xt2000_setscl(0);
	bit_xt2000_setsda(0);
	bit_xt2000_setscl(1);
	bit_xt2000_setsda(1);
}

static unsigned i2c_send_byte(unsigned char byte)
{
	int i;
	for (i=0x80; i>0; i>>=1) {
		bit_xt2000_setsda(byte&i);
		bit_xt2000_setscl(1);
		bit_xt2000_setscl(0);
	}
	bit_xt2000_setsda(1);
	bit_xt2000_setscl(1);
	i = (bit_xt2000_getsda()==0);
	bit_xt2000_setscl(0);
	return i;
}

static unsigned char i2c_recv_byte(void)
{
	int i;
	unsigned char byte = 0;
	bit_xt2000_setsda(1);
	for (i = 8; i; i--) {
		bit_xt2000_setscl(1);
		byte = (byte << 1) | bit_xt2000_getsda();
		bit_xt2000_setscl(0);
	}
	return byte;
}

static void i2c_ack(unsigned ack)
{
	bit_xt2000_setsda(ack);
	bit_xt2000_setscl(1);
	bit_xt2000_setscl(0);
}

static void i2c_read_data(unsigned id, unsigned char *buf,
	unsigned addr, unsigned size)
{
	int i;
        char system;

        V3USC_SYS_OUT(SYSTEM_B_UNLOCK_TOKEN);
        V3USC_SYS_OUT(V3USC_SYS_IN() | SYSTEM_B_SPROM_EN);
	i2c_start();
	i2c_send_byte((id << 1) | (0));
	i2c_send_byte(addr);
	i2c_start();
	i2c_send_byte((id << 1) | (1));
	for(i=0; i<size; i++) {
		buf[i] = i2c_recv_byte();
		i2c_ack(i == size-1);
	}
	i2c_stop();
        system = V3USC_SYS_IN() & ~ SYSTEM_B_SPROM_EN;
        V3USC_SYS_OUT(system);
        V3USC_SYS_OUT(system | SYSTEM_B_LOCK);
}

static int i2c_write_data(unsigned id, unsigned char *buf,
	unsigned addr, unsigned size)
{
	int i;
        char system;
        V3USC_SYS_OUT(SYSTEM_B_UNLOCK_TOKEN);
        V3USC_SYS_OUT(V3USC_SYS_IN() | SYSTEM_B_SPROM_EN);
	i2c_start();
	i2c_send_byte((id << 1) | (0));
	i2c_send_byte(addr);
	for(i=0; i<size; i++)
		i2c_send_byte(buf[i]);
	i2c_ack(1);
	i2c_stop();
        system = V3USC_SYS_IN() & ~ SYSTEM_B_SPROM_EN;
        V3USC_SYS_OUT(system);
        V3USC_SYS_OUT(system | SYSTEM_B_LOCK);
	return 0;
}

static const unsigned short crc16_ltab[16] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7
};

static const unsigned short crc16_utab[16] =
{
	0x0000, 0x1081, 0x2102, 0x3183, 0x4204, 0x5285, 0x6306, 0x7387,
	0x8408, 0x9489, 0xa50a, 0xb58b, 0xc60c, 0xd68d, 0xe70e, 0xf78f
};

static unsigned calc_crc16(char* buf, int len)
{
    unsigned short fcs = 0;
    unsigned char c;
    while (len--) {
        c = (fcs ^ *buf++)&0xff;
        fcs = fcs >> 8 ^ crc16_ltab[c&0x0f] ^ crc16_utab[c>>4];
    }
    return fcs;
}

int platform_get_rtc_time(time_t *time)
{
    unsigned char buf[8];
    i2c_read_data(XT2000_I2C_RTC_ID, buf, 0, 6);
    if (buf[4]&0x80)
	return -2;
    *time = (buf[3]<<24)+(buf[2]<<16)+(buf[1]<<8)+buf[0];
    return 0;
}

int platform_set_rtc_time(time_t time)
{
    unsigned char buf[8];
    buf[0] = time&0xff;
    buf[1] = (time>>8)&0xff;
    buf[2] = (time>>16)&0xff;
    buf[3] = (time>>24)&0xff;
    buf[4] = 0;	/* Enable Oscillator */
    buf[5] = 0;	/* Disable trickle charger */
    return i2c_write_data(XT2000_I2C_RTC_ID, buf, 0, 6);
}

static unsigned nvram_data_valid = 0;
static struct xt2000_nvram_binfo nvram_data;

unsigned xtboard_nvram_valid(void)
{
    if (nvram_data_valid)
	return 1;
    i2c_read_data(XT2000_I2C_NVRAM1_ID, (unsigned char *)&nvram_data,
	XT2000_NVRAM_BINFO_START, XT2000_NVRAM_BINFO_SIZE);
    if (calc_crc16((char *)&nvram_data, XT2000_NVRAM_BINFO_SIZE)==0)
	nvram_data_valid = 1;

    return nvram_data_valid;
}

unsigned xtboard_get_nvram_contents(unsigned char *buf)
{
    if (xtboard_nvram_valid());
	memcpy(buf, &nvram_data, XT2000_NVRAM_BINFO_SIZE);
    return nvram_data_valid;
}

#define ETH_ADDR_SIZE  6

void xtboard_get_ether_addr(unsigned char *buf)
{
    if (nvram_data_valid)
	    memcpy(buf, &nvram_data.eth_addr, ETH_ADDR_SIZE);
    else {
	    unsigned eth_offset = (unsigned) &nvram_data.eth_addr;
	    eth_offset -= (unsigned) &nvram_data;
	    i2c_read_data(XT2000_I2C_NVRAM1_ID, buf, eth_offset, ETH_ADDR_SIZE);
    }
}

