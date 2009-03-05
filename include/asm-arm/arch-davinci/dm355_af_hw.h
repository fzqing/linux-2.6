/* *
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/* dm355_af.h file */

#ifndef DM355_AF_DRIVER_HW_H
#define DM355_AF_DRIVER_HW_H

/* Include driver header file */
#include "dm355_af.h"

#ifdef __KERNEL__

/* Register Offsets */
#define AFPID               0x0	/*Peripheral Revision */
					/*and Class Information */
#define AFPCR               0x00000004	/*Peripheral Control Register */
#define AFPAX1              0x00000008	/*Setup for the Paxel Configuration */
#define AFPAX2              0x0000000c	/*Setup for the Paxel Configuration */
#define AFPAXSTART          0x00000010	/*Start Position for AF Engine Paxels */
#define AFIIRSH             0x00000014	/*Start Position for IIRSH */
#define AFBUFST             0x00000018	/*SDRAM/DDRAM Start address */
#define AFCOEF010           0x0000001c	/*IIR filter coefficient data 
					   for SET 0 */
#define AFCOEF032           0x00000020	/*IIR filter coefficient data
					   for SET 0 */
#define AFCOEF054           0x00000024	/*IIR filter coefficient data 
					   for SET 0 */
#define AFCOEF076           0x00000028	/*IIR filter coefficient data
					   for SET 0 */
#define AFCOEF098           0x0000002c	/*IIR filter coefficient data 
					   for SET 0 */
#define AFCOEF0010          0x00000030	/*IIR filter coefficient data 
					   for SET 0 */
#define AFCOEF110           0x00000034	/*IIR filter coefficient data 
					   for SET 1 */
#define AFCOEF132           0x00000038	/*IIR filter coefficient 
					   data for SET 1 */
#define AFCOEF154           0x0000003c	/*IIR filter coefficient data 
					   for SET 1 */
#define AFCOEF176           0x00000040	/*IIR filter coefficient data 
					   for SET 1 */
#define AFCOEF198           0x00000044	/*IIR filter coefficient data 
					   for SET 1 */
#define AFCOEF1010          0x00000048	/*IIR filter coefficient data 
					   for SET 1 */
#define AF_RSDR_ADDR	0x00000060	/*SDRAM/DDRAM Read Address */
#define AF_RSDR_OFFSET	0x00000064	/*SDRAM/DDRAM Line Offset */
#define AF_SDR_FRSIZE	0x00000068	/*Frame Size for SDRAM read data */

#define AFCOEF_OFFSET	0x00000004	/* COEFFICIENT BASE ADDRESS */
//#define AF_INTSEL	IO_ADDRESS(0x1c70810)

#define AF_SET_VAL(x)       	(((x)/2)-1)
#define AF_NOT_EVEN		        1
#define AF_CHECK_EVEN(x)	    ((x)%2)
/* Register Fields */
/*
 *    PID fields
 */
#define AF_TID                          (0xFF<<16)
#define AF_CID                          (0xFF<<8)
#define AF_PREV                         0xFF

/*
 *    PCR fields
 */
#define AVE2LMT                         (0x3FF<<22)
#define AF_INP_SRC			(1<<19)
#define AF_BUSYAEWB                     (1<<18)
#define AEW_ALAW_EN                     (1<<17)
#define AEW_EN                          (1<<16)
#define AF_BUSYAF                       (1<<15)
#define FVMODE                          (1<<14)
#define RGBPOS                          (0x7<<11)
#define MED_TH                          (0xFF<<3)
#define AF_MED_EN                       (1<<2)
#define AF_ALAW_EN                      (1<<1)
#define AF_EN                           (1<<0)

#define AF_SETGAMMAWD			outl(0x00000010,IO_ADDRESS(0x01C70680))

/*
 * AFPAX1 fields
 */
#define PAXW                            (0x7F<<16)
#define PAXH                            0x7F

#define AF_CCDC				0
/*
 * AFPAX2 fields
 */
#define  AFINCV                         (0xF<<13)
#define  PAXVC                          (0x7F<<6)
#define  PAXHC                          0x3F

/*
 * AFPAXSTART fields
 */
#define  PAXSH                          (0xFFF<<16)
#define  PAXSV                          0xFFF

/*
 * COEFFICIENT MASK
 */

#define COEF_MASK0                      0xFFF
#define COEF_MASK1                      (0xFFF<<16)


/* SDRAM Frame Size */
#define AF_SDR_FRSIZE_HSIZE             0xFFF
#define AF_SDR_FRSIZE_VSIZE		(0xFFF<<16)
#define AF_SDR_FRSIZE_BITSEL		(0x7<<28)

/* BIT SHIFTS */
#define AF_RGBPOS_SHIFT                 11
#define AF_MED_TH_SHIFT                 3
#define AF_PAXW_SHIFT                   16
#define AF_LINE_INCR_SHIFT              13
#define AF_VT_COUNT_SHIFT               6
#define AF_HZ_START_SHIFT               16
#define AF_COEF_SHIFT                   16
#define AF_INP_SRC_SHIFT                19
#define AF_GETFRAME_SIZE 		(inl(IO_ADDRESS(0x01C70910)))
#define AF_GETINIT_XY 			(inl(IO_ADDRESS(0x01C70910)))
#define AF_INTSTATBASE			(IO_ADDRESS(0x01C7080C))
#define AF_EVNTSELADDR			(IO_ADDRESS(0x01C70814))
#define	AF_CLKCTRL_ADDR			(IO_ADDRESS(0x01C70004))
#define AF_GETCLKCTRL			inl(AF_CLKCTRL_ADDR)
#define AF_SETCLKCTRL(val)		outl(val,AF_CLKCTRL_ADDR)

/* Macros for register read and write */
#define AF_IOBASE_VADDR                 IO_ADDRESS(0x01c70080)
#define regr(reg)                       inl((reg)+AF_IOBASE_VADDR)
#define regw(val,reg)                   outl(val,(reg)+AF_IOBASE_VADDR)
#define AF_GET_PCR                      inl(AF_IOBASE_VADDR + AFPCR)
//#define AF_SELINT(val)			outl(val,AF_INTSEL)
//#define AFSETINT(val)			outl(val,AF_INTSEL)
//#define AF_EVNTSEL(val)			outl(val,AF_EVNTSELADDR)
//#define AF_GETINT			inl(AF_INTSEL)
//#define AF_GETEVNT			inl(AF_EVNTSELADDR)
//#define AF_INTMASK			(0xF << 16)
#define AF_GETINTSTAT			inl(AF_INTSTATBASE)
/* Function declaration */
int af_register_setup(struct af_device *);
void af_engine_setup(int);
void af_set_address(unsigned long);
#endif				/*enf of #ifdef __KERNEL__  */
#endif
