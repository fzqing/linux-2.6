/* -------------------------------------------------------------------- */
/* i2c-algo-emma2rh.h: NEC EMMA2RH global defines                       */
/* -------------------------------------------------------------------- */
/*
    Copyright (C) NEC Electronics Corporation 2005-2006

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA*/
/* --------------------------------------------------------------------	*/

#ifndef I2C_EMMA2RH_H
#define I2C_EMMA2RH_H

/*---------------------------------------------------------------------------*/
/* CNT - Control register (00H R/W)                                          */
/*---------------------------------------------------------------------------*/
#define SPT         0x00000001
#define STT         0x00000002
#define ACKE        0x00000004
#define WTIM        0x00000008
#define SPIE        0x00000010
#define WREL        0x00000020
#define LREL        0x00000040
#define IICE        0x00000080
#define CNT_RESERVED    0x000000ff	/* reserved bit 0 */

#define I2C_EMMA_START      (IICE | STT)
#define I2C_EMMA_STOP       (IICE | SPT)
#define I2C_EMMA_REPSTART   I2C_EMMA_START

/*---------------------------------------------------------------------------*/
/* STA - Status register (10H Read)                                          */
/*---------------------------------------------------------------------------*/
#define MSTS        0x00000080
#define ALD         0x00000040
#define EXC         0x00000020
#define COI         0x00000010
#define TRC         0x00000008
#define ACKD        0x00000004
#define STD         0x00000002
#define SPD         0x00000001

/*---------------------------------------------------------------------------*/
/* CSEL - Clock select register (20H R/W)                                    */
/*---------------------------------------------------------------------------*/
#define FCL         0x00000080
#define ND50        0x00000040
#define CLD         0x00000020
#define DAD         0x00000010
#define SMC         0x00000008
#define DFC         0x00000004
#define CL          0x00000003
#define CSEL_RESERVED   0x000000ff	/* reserved bit 0 */

#define FAST397     0x0000008b
#define FAST297     0x0000008a
#define FAST347     0x0000000b
#define FAST260     0x0000000a
#define FAST130     0x00000008
#define STANDARD108 0x00000083
#define STANDARD83  0x00000082
#define STANDARD95  0x00000003
#define STANDARD73  0x00000002
#define STANDARD36  0x00000001
#define STANDARD71  0x00000000

/*---------------------------------------------------------------------------*/
/* SVA - Slave address register (30H R/W)                                    */
/*---------------------------------------------------------------------------*/
#define SVA         0x000000fe

/*---------------------------------------------------------------------------*/
/* SHR - Shift register (40H R/W)                                            */
/*---------------------------------------------------------------------------*/
#define SR          0x000000ff

/*---------------------------------------------------------------------------*/
/* INT - Interrupt register (50H R/W)                                        */
/* INTM - Interrupt mask register (60H R/W)                                  */
/*---------------------------------------------------------------------------*/
#define INTE0       0x00000001

/***********************************************************************
 * I2C registers
 ***********************************************************************
 */
#define I2C_EMMA_CNT            0x00
#define I2C_EMMA_STA            0x10
#define I2C_EMMA_CSEL           0x20
#define I2C_EMMA_SVA            0x30
#define I2C_EMMA_SHR            0x40
#define I2C_EMMA_INT            0x50
#define I2C_EMMA_INTM           0x60
#endif				/* I2C_EMMA2RH_H */
