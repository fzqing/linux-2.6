/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */
#ifndef I2C_PALM_H
#define I2C_PALM_H 			1

#define I2C_PALM_CFG			0x00 
#define I2C_PALM_CLKDIV			0x04 
#define I2C_PALM_DEVADDR		0x08 
#define I2C_PALM_ADDR			0x0C
#define I2C_PALM_DATAOUT		0x10 
#define I2C_PALM_DATAIN			0x14 
#define I2C_PALM_STATUS			0x18
#define I2C_PALM_STARTXFR		0x1C
#define I2C_PALM_BYTECNT		0x20
#define I2C_PALM_HDSTATIM		0x24

#define I2C_PALM_CFG_DEF		0x000000F8/* 8-Bit Addr + POR Values */
#define I2C_PALM_CLKDIV_DEF		0x00000052	
#define I2C_PALM_HDSTATIM_DEF		0x00000000
#define I2C_PALM_STARTXFR_RD		0x00000001
#define I2C_PALM_STARTXFR_WR		0x00000000

#endif /* I2C_PALM_H */
