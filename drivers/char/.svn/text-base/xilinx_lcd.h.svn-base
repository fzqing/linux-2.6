/*
 * xilinx_lcd.h
 *
 * Virtex2Pro character LCD driver.
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2004-2005 (c) MontaVista, Software, Inc.
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty
 * of any kind, whether express or implied.
 */

/*
 * Memec IP provides interface to a HD44780 compatible device
 * by means of single 10-bit register (big endian notation):
 *   bit 22 - RS (Register Select) - 0: Instruction register
 *                                   1: Data register
 *   bit 23 - R/#W - 0: Write
 *                   1: Read
 *   bit 24..bit 31 - LCD_DATA0..LCD_DATA7
 */

/*
 * Based on:
 *   Simple driver for a memory-mapped 44780-style LCD display.
 *   Configured for CMB-VR7701/Rockhopper
 *   2003 (c) MontaVista Software, Inc.
 * Which is in turn based on:
 *   Copyright 2001 Bradley D. LaRonde <brad@ltc.com>
 */

#ifndef _XILINX_HD44780_LCD_H
#define _XILINX_HD44780_LCD_H

/* Instruction/Data register and R/W selection masks */
#define HD44780_INSTR		0x000	/* WR to Instruction Register */
#define HD44780_STAT		0x100	/* RD Busy Flag / Address Counter */
#define HD44780_WR_DATA		0x200	/* WR to Data Register */
#define HD44780_RD_DATA		0x300	/* RD from Data Register */

/* Command values */
#define HD44780_CLR_DISPLAY	0x01	/* clears entire display and resets addr counter */
#define HD44780_RET_HOME	0x02	/* resets addr counter, cancels the shifts */
#define HD44780_MODE_DEC	0x04
#define HD44780_MODE_DEC_SHIFT	0x05
#define HD44780_MODE_INC	0x06
#define HD44780_MODE_INC_SHIFT	0x07
#define HD44780_DISPLAY_OFF	0x08
#define HD44780_CURSOR_NONE	0x0c
#define HD44780_CURSOR_LINE	0x0e
#define HD44780_CURSOR_BLOCK	0x0f
#define HD44780_LEFT		0x10
#define HD44780_RIGHT		0x14
#define HD44780_LEFT_SCROLL	0x18
#define HD44780_RIGHT_SCROLL	0x1e
#define HD44780_8BIT_1LINE	0x30
#define HD44780_8BIT_2LINE	0x38

/* These commands to be or'ed with address values */
#define HD44780_CG_ADDRESS	0x40
#define HD44780_DD_ADDRESS	0x80

#define LCD_GREETING "MontaVista Linux****************"
#define XILINX_LCD_MINOR 	200
#define XILINX_LCD_MAX_POS	32
#define XILINX_LCD_NAME		"xilinx_char_lcd"

static inline void lcd_wait(void)
{
	udelay(200);		/* assuming Fosc > 50kHz */
}

#endif	/* _XILINX_HD44780_LCD_H */
