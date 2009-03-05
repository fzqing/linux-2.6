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
/* dm355_aew_hw.h file */


#ifndef DM355_AEW_DRIVER_HW_H
#define DM355_AEW_DRIVER_HW_H

/* Include Files */
#include <asm/io.h>
#include <asm/arch/hardware.h>

#ifdef __KERNEL__

/* Register Offsets */
#define AEWPID          0x0000000	/*Peripheral Revision */
					    /*and Class Information */
#define AEWPCR          0x00000004	/*Peripheral Control Register */
#define AEWWIN1         0x0000004c	/*Configuration for AE/AWB Windows */
#define AEWINSTART      0x00000050	/*Start position for AE/AWB Windows */
#define AEWINBLK        0x00000054	/*Start position and */
					    /*height for black linr */
					/*of AE/AWB Windows */
#define AEWSUBWIN       0x00000058	/*Configuration for subsampled data */
					/* in AE/AWB windows */
#define AEWBUFST        0x0000005c	/*SDRAM/DDRAM Start address */
					    /*for AEW Engine */

#define AEW_RSDR_ADDR	0x00000060	/*SDRAM/DDRAM Read Address */
#define AEW_RSDR_OFFSET	0x00000064	/*SDRAM/DDRAM Line Offset */
#define AEW_SDR_FRSIZE	0x00000068	/*Frame Size for SDRAM read data */
/* PID fields */
#define AEW_TID             (0xFF<<16)
#define AEW_CID             (0xFF<<8)
#define AEW_PREV            0xFF

/* PCR FIELDS */
#define AVE2LMT             	(0x3ff<<22)	/*Saturation Limit */
#define AEW_SDR_FETCH_ENABLE	(1<<21)
#define AEW_INP_WIDTH		(1<<20)
#define AEW_INP_SRC		(1<<19)
#define AEW_ALAW_EN         	(1<<17)	/*Alaw Enable/Disable Bit */
#define AEW_BUSYAF          	(1<<15)	/* Busy Bit for AF */
#define AEW_BUSYAEWB        	(1<<18)	/*Busy bit for AEW */
#define AEW_EN              	(1<<16)	/*AEW Engine Enable/Disable bit */

/* AEWWIN1 fields */
#define WINH                	(0x7F<<24)	/*Window Height */
#define WINW                	(0x7f<<13)	/*Window Width */
#define WINVC               	(0x7f<<6)	/*Window vertical Count */
#define WINHC               	0x3f	/*Window Horizontal Count */

/* AEWWINSTART fields */
#define WINSV               	(0xfff<<16)	/*Window Vertical Start */
#define WINSH               	0xfff	/*Window Horizontal start */

/* AEWWINBLK fields */
#define BLKWINSV            	(0xfff<<16)	/*Black Window Vertical Start */
#define BLKWINH             	0x7f	/* Black Window height */

/* AEWSUBWIN fields */
#define AEWINCV             	(0xf<<8)	/*Vertical Lime Increment */
#define AEWINCH             	0xf	/*Horizontal Line Increment */

/* BIT POSITIONS */
#define AEW_AVE2LMT_SHIFT		22

#define AEW_WINH_SHIFT			24
#define AEW_WINW_SHIFT              	13
#define AEW_VT_COUNT_SHIFT		6
#define AEW_VT_START_SHIFT		16
#define AEW_LINE_INCR_SHIFT		8
#define AEW_BLKWIN_VT_START_SHIFT   	16
#define AEW_EN_SHIFT                	16
#define AEW_BUSYAEWB_SHIFT          	18
#define AEW_INP_SRC_SHIFT               19

//#define AEW_INTSEL			IO_ADDRESS(0x1c70810)


#define AEW_SET_VAL(x)       	    	(((x)/2)-1)
#define AEW_NOT_EVEN		        1
#define AEW_CHECK_EVEN(x)	        ((x)%2)


#define AEW_CCDC			0
#define AEW_INTSTATBASE			(IO_ADDRESS(0x01C7080C))
#define AEW_EVNTSELADDR			(IO_ADDRESS(0x01C70814))


//#define AEW_SELINT(val)			outl(val,AEW_INTSEL)
//#define AEW_EVNTSEL(val)		outl(val,AEW_EVNTSELADDR)
//#define AEW_GETINT			inl(AEW_INTSEL)
//#define AEW_INTMASK			(0xF << 16)
#define AEW_GETINTSTAT			inl(AEW_INTSTATBASE)
#define AEW_SETGAMMAWD			outl(0x00000010,IO_ADDRESS(0x01C70680))
#define	AEW_CLKCTRL_ADDR		(IO_ADDRESS(0x01C70004))
#define AEW_GETCLKCTRL			inl(AEW_CLKCTRL_ADDR)
#define AEW_SETCLKCTRL(val)		outl(val,AEW_CLKCTRL_ADDR)

#define AEW_IOBASE_VADDR 		(IO_ADDRESS(0x01c70080))	/*Base Addr */

#define regw(val,reg)               	outl(val,(reg+AEW_IOBASE_VADDR))
#define regr(reg)                   	inl(reg+AEW_IOBASE_VADDR)

#define AEW_GETEVNT			inl(AEW_EVNTSELADDR)



#define AEW_GET_PCR                 	inl(AEW_IOBASE_VADDR + AEWPCR);

#define isbusy()			(regr(AEWPCR) & 0x40000)

/* Function Declaration */
int aew_register_setup(struct aew_device *);
void aew_engine_setup(int);
void aew_set_address(unsigned long);

#endif				/*end of #ifdef __KERNEL__ */

#endif				/*end of #ifdef __DAVINCI_AEW_HW_H */
