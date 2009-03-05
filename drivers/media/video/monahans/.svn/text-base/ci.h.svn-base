/*
   Copyright (C) 2005, Intel Corporation.
   Copyright (C) 2006, Marvell International Ltd.

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
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Abstract:
 *   Contains all Monahans QCI specific macros, typedefs, and prototypes.
 *   Declares no storage.
 * Notes:
 *   Only valid for processor code named Monahans.
 */


#ifndef __MONAHANS_CI_HEADER__
#define __MONAHANS_CI_HEADER__

#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <linux/pxa_camera_zl.h>

/*
 * Register definitions
 */

#define	CI_CICR0_SIM_SHIFT		24
#define	CI_CICR0_SIM_SMASK		0x7
#define	CI_CICR0_INTERRUPT_MASK		(CICR0_FOM | CICR0_EOFM | \
		CICR0_SOFM | CICR0_CDM | \
		CICR0_QDM | CICR0_PERRM | \
		CICR0_EOLM | CICR0_FEM | \
		CICR0_TOM | CICR0_FUM | \
		CICR0_BSM | CICR0_EOFXM)

#define CI_CICR1_DW_SHIFT		0
#define CI_CICR1_DW_SMASK		0x7
#define CI_CICR1_COLOR_SP_SHIFT		3
#define CI_CICR1_COLOR_SP_SMASK		0x3
#define CI_CICR1_RAW_BPP_SHIFT		5
#define CI_CICR1_RAW_BPP_SMASK		0x3
#define CI_CICR1_RGB_BPP_SHIFT		7
#define CI_CICR1_RGB_BPP_SMASK		0x7
#define CI_CICR1_YCBCR_F		(1UL << 10)
#define CI_CICR1_RBG_F			(1UL << 11)
#define CI_CICR1_RGB_CONV_SHIFT		12
#define CI_CICR1_RGB_CONV_SMASK		0x7
#define CI_CICR1_PPL_SHIFT		15
#define CI_CICR1_PPL_SMASK		0xFFF
#define CI_CICR1_RESERVED		0x1C000000
#define CI_CICR1_RGBT_CONV_SHIFT	9
#define CI_CICR1_RGBT_CONV_SMASK	0x3
#define CI_CICR1_TBIT			(1UL << 31)


#define CI_CICR2_FSW_SHIFT	0
#define CI_CICR2_FSW_SMASK	0x3
#define CI_CICR2_BFPW_SHIFT	3
#define CI_CICR2_BFPW_SMASK	0x3F
#define CI_CICR2_RESERVED	0x00000200
#define CI_CICR2_HSW_SHIFT	10
#define CI_CICR2_HSW_SMASK	0x3F
#define CI_CICR2_ELW_SHIFT	16
#define CI_CICR2_ELW_SMASK	0xFF
#define CI_CICR2_BLW_SHIFT	24
#define CI_CICR2_BLW_SMASK	0xFF

#define CI_CICR3_LPF_SHIFT	0
#define CI_CICR3_LPF_SMASK	0x7FF
#define CI_CICR3_VSW_SHIFT	11
#define CI_CICR3_VSW_SMASK	0x1F
#define CI_CICR3_EFW_SHIFT	16
#define CI_CICR3_EFW_SMASK	0xFF
#define CI_CICR3_BFW_SHIFT	24
#define CI_CICR3_BFW_SMASK	0xFF

#define CI_CICR4_DIV_SHIFT	0
#define CI_CICR4_DIV_SMASK	0xFF
#define CI_CICR4_FR_RATE_SHIFT	8
#define CI_CICR4_FR_RATE_SMASK	0x7
#define CI_CICR4_RESERVED1	0x0007F800
#define CI_CICR4_MCLK_EN	(1UL << 19)
#define CI_CICR4_VSP		(1UL << 20)
#define CI_CICR4_HSP		(1UL << 21)
#define CI_CICR4_PCP		(1UL << 22)
#define CI_CICR4_PCLK_EN	(1UL << 23)
#if defined(CONFIG_PXA310)
#define CI_CICR4_YCBCR_DS	(1UL << 27)
#define CI_CICR4_RESERVED2	0xF7000000
#else
#define CI_CICR4_RESERVED2	0xFF000000
#endif
#define CI_CICR4_RESERVED	(CI_CICR4_RESERVED1|CI_CICR4_RESERVED2)

#define CI_CISR_EOF          (1 << 3)
#define CI_CISR_SOF          (1 << 4)
#define CI_CISR_CDD          (1 << 5)
#define CI_CISR_CQD          (1 << 6)
#define CI_CISR_PAR_ERR      (1 << 7)
#define CI_CISR_EOL          (1 << 8)
#define CI_CISR_HST_INT      (1 << 9)
#define CI_CISR_CGU_INT      (1 << 10)
#define CI_CISR_FTO          (1 << 15)
#define CI_CISR_EOFX         (1 << 30)
#define CI_CISR_SINT         (1 << 31)
#define	CI_CISR_MASK	(CI_CISR_EOF | CI_CISR_SOF | CI_CISR_CDD | \
		CI_CISR_CQD | CI_CISR_PAR_ERR | CI_CISR_EOL | \
		CI_CISR_CGU_INT|CI_CISR_FTO | \
		CI_CISR_EOFX | CI_CISR_SINT)


#define	CI_CIFSR_EOF3	(1 << 10)


#define	CIPSS_PSU_EN	(1 << 31)

#define	CI_CIPBUF_DEADROW_SHIFT	0
#define	CI_CIPBUF_DEADROW_SMASK	0xFFF
#define	CI_CIPBUF_DEADCOL_SHIFT	16
#define	CI_CIPBUF_DEADCOL_SMASK	0xFFF

#define CI_CIHST_COLOR_SEL_SHIFT	0
#define CI_CIHST_COLOR_SEL_SMASK	0xF
#define CI_CIHST_COLOR_RED		0x1
#define CI_CIHST_COLOR_BLUE		0x2
#define CI_CIHST_COLOR_GREEN1		0x4
#define CI_CIHST_COLOR_GREEN2		0x8
#define CI_CIHST_SCALE_SHIFT		4
#define CI_CIHST_SCALE_SMASK		0x3
#define CI_CIHST_SCALE_0_TO_7		0x0
#define CI_CIHST_SCALE_0_TO_8		0x1
#define CI_CIHST_SCALE_1_TO_9		0x2
#define CI_CIHST_CLR_RAM		(1 << 6)

#define CI_CICCR_EN		(1 << 0)
#define CI_CICCR_SCALE_SHIFT	1
#define CI_CICCR_SCALE_SMASK	0x3
#define CI_CICCR_SCALE_0_TO_7	0x0
#define CI_CICCR_SCALE_1_TO_8	0x1
#define CI_CICCR_SCALE_2_TO_9	0x2
#define CI_CICCR_CLUT_SHIFT	3
#define CI_CICCR_CLUT_SMASK	0xFF
#define CI_CICCR_CLUT_RED	0x00
#define CI_CICCR_CLUT_BLUE	0x20
#define CI_CICCR_CLUT_GREEN	0x40
#define CI_CICCR_BLC_SHIFT	11
#define CI_CICCR_BLC_SMASK	0xFF
#define CI_CICCR_LUT_LD		(1 << 19)

#define CI_CISSC_SCALE_SHIFT	0
#define CI_CISSC_SCALE_SMASK	0x3
#define CI_CISSC_SCALE_DISABLE	0x0
#define CI_CISSC_SCALE_2_TO_1	0x1
#define CI_CISSC_SCALE_4_TO_1	0x2

#define CI_CICMR_DMODE_SHIFT	0
#define CI_CICMR_DMODE_SMASK	0x3
#define CI_CICMR_DMODE_DISABLE	0x0
#define CI_CICMR_DMODE_RGB	0x1
#define CI_CICMR_DMODE_YUV	0x2

#define CI_CICMC0_COF02_SHIFT	0
#define CI_CICMC0_COF02_SMASK	0x3FF

#define CI_CICMC0_COF01_SHIFT	10
#define CI_CICMC0_COF01_SMASK	0x3FF

#define CI_CICMC0_COF00_SHIFT	20
#define CI_CICMC0_COF00_SMASK	0x3FF

#define CI_CICMC1_COF12_SHIFT	0
#define CI_CICMC1_COF12_SMASK	0x3FF

#define CI_CICMC1_COF11_SHIFT	10
#define CI_CICMC1_COF11_SMASK	0x3FF

#define CI_CICMC1_COF10_SHIFT	20
#define CI_CICMC1_COF10_SMASK	0x3FF

#define CI_CICMC2_COF22_SHIFT	0
#define CI_CICMC2_COF22_SMASK	0x3FF

#define CI_CICMC2_COF21_SHIFT	10
#define CI_CICMC2_COF21_SMASK	0x3FF

#define CI_CICMC2_COF20_SHIFT	20
#define CI_CICMC2_COF20_SMASK	0x3FF


#define CI_CIFR_THL_0_SHIFT	4
#define CI_CIFR_THL_0_SMASK	0x3

#define CI_CIFR_RESERVED1	0x000000C0

#define CI_CIFR_FLVL0_SHIFT	8
#define CI_CIFR_FLVL0_SMASK	0xFF

#define CI_CIFR_FLVL1_SHIFT	16
#define CI_CIFR_FLVL1_SMASK	0x7F

#define CI_CIFR_FLVL2_SHIFT	23
#define CI_CIFR_FLVL2_SMASK	0x7F

#define CI_CIFR_RESERVED2	0xC0000000
#define CI_CIFR_RESERVED	CI_CIFR_RESERVED1 | CI_CIFR_RESERVED2

#define	CI_CIFR_FEN3		(1 << 0)
#define	CI_CIFR_FLVL3_SHIFT	1
#define	CI_CIFR_FLVL3_SMASK	0xFF

#define	CI_CIDBR_BRA		(1 << 0)
#define	CI_CIDBR_BINT		(1 << 1)
#define	CI_CIDBR_SRCADDR_SMASK	0xFFFFFFF0

/*
 * Parameter Type definitions
 */
enum pxa_image_format {
	CI_RAW8	= 0,		/* RAW */
	CI_RAW9,
	CI_RAW10,
	CI_YCBCR422,		/* YCBCR */
	CI_YCBCR422_PLANAR,	/* YCBCR Planaried */
	CI_RGB444,		/* RGB */
	CI_RGB555,
	CI_RGB565,
	CI_RGB666,
	CI_RGB888,
	CI_RGBT555_0,		/* RGB+Transparent bit 0 */
	CI_RGBT888_0,
	CI_RGBT555_1,		/* RGB+Transparent bit 1 */
	CI_RGBT888_1,
	CI_RGB666_PACKED,	/* RGB Packed */
	CI_RGB888_PACKED,
#if defined(CONFIG_PXA310)
	CI_YCBCR420,
	CI_YCBCR420_PLANAR,
#endif
	CI_INVALID_FORMAT =	0xFF
};

/* Interrupt mask */
#define CI_INT_IFO	(1 << 0)   /* FIFO Overrun Mask */
#define CI_INT_EOF	(1 << 1)   /* QCI End-of-Frame Mask */
#define CI_INT_SOF	(1 << 2)   /* QCI Start-of-Frame Mask */
#define CI_INT_CDD	(1 << 3)   /* QCI Disable Done Mask */
#define CI_INT_CQD	(1 << 4)   /* QCI Quick Disable Mask */
#define CI_INT_PAR_ERR	(1 << 5)   /* Parity Error Mask */
#define CI_INT_EOL	(1 << 6)   /* End-of-Line Mask */
#define CI_INT_FEMPTY	(1 << 7)   /* FIFO Empty Mask */
#define CI_INT_FTO	(1 << 9)   /* Time-Out Mask */
#define CI_INT_FU	(1 << 10)  /* Input FIFO Underrun Mask Channel 3 */
#define CI_INT_BS	(1 << 11)  /* Branch Status Mask */
#define CI_INT_EOFX	(1 << 12)  /* End-of-Frame Transfer to Memory Mask */
#define CI_INT_SC0	(1 << 13)  /* Stop Channel Interrupt Mask Channel 0*/
#define CI_INT_SC1	(1 << 14)  /* Stop Channel Interrupt Mask Channel 1*/
#define CI_INT_SC2	(1 << 15)  /* Stop Channel Interrupt Mask Channel 2*/
#define CI_INT_SC3	(1 << 16)  /* Stop Channel Interrupt Mask Channel 3*/

#define CICR0_VAL(ci_int_mask) \
	((ci_int_mask) & CI_CICR0_INTERRUPT_MASK)

/* convert CI_INT_SC0 to CIDCSR0[StopIrqEn] */
#define CIDCSR0_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 13))) >> 13) << 29)

/* convert CI_INT_SC1 to CIDCSR1[StopIrqEn] */
#define CIDCSR1_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 14))) >> 14) << 29)

/* convert CI_INT_SC2 to CIDCSR2[StopIrqEn] */
#define CIDCSR2_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 15))) >> 15) << 29)

/* convert CI_INT_SC3 to CIDCSR3[StopIrqEn] */
#define CIDCSR3_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 16))) >> 16) << 29)

#define CI_INT_MASK(cicr0_val, cidcsr0_val, cidcsr1_val,\
		cidcsr2_val, cidcsr3_val) \
	(((cicr0_val) & CI_CICR0_INTERRUPT_MASK) | \
	 ((!((cidcsr0_val) & (1UL << 29))) << 13) | \
	 ((!((cidcsr1_val) & (1UL << 29))) << 14) | \
	 ((!((cidcsr2_val) & (1UL << 29))) << 15) | \
	 ((!((cidcsr3_val) & (1UL << 29))) << 16)   \
)

/*
   notes:

   mapping between ci_int_mask and related registers bits:
   ci_int_mask:

  0  CICR0[FOM]
  1  CICR0[EOFM]
  2  CICR0[SOFM]
  3  CICR0[CDM]
  4  CICR0[QDM]
  5  CICR0[PERRM]
  6  CICR0[EOLM]
  7  CICR0[FEM]
  8  N/A
  9  CICR0[TOM]
  10  CICR0[FUM]
  11  CICR0[BSM]
  12  CICR0[EOFM]
  13  CIDCSR0[StopIrqEn]
  14  CIDCSR1[StopIrqEn]
  15  CIDCSR2[StopIrqEn]
  16  CIDCSR3[StopIrqEn]
  17  N/A
  18  N/A
  19  N/A
  20  N/A
  21  N/A
  22  N/A
  23  N/A
  24  N/A
  25  N/A
  26  N/A
  27  N/A
  28  N/A
  29  N/A
  30  N/A
  31  N/A

#define CI_INT_MASK(cicr0_val, cidcsr0_val, cidcsr1_val,	\
			cidcsr2_val, cidcsr3_val) \
	(((cicr0_val) & CI_CICR0_INTERRUPT_MASK) | \
	((!((cidcsr0_val) & (1UL << 29))) << 13) | \
	((!((cidcsr1_val) & (1UL << 29))) << 14) | \
	((!((cidcsr2_val) & (1UL << 29))) << 15) | \
	((!((cidcsr3_val) & (1UL << 29))) << 16)   \
)
 */

/* Interrupt status */

/* End-of-Frame for Channel 3, depending on Channel 0
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_EOF3              (1 << 0)
/* FIFO Underrun for Channel 3, masked by CI_INT_FU */
#define CI_INTSTATUS_FU_3              (1 << 1)
/* End-of-Frame, masked by CI_INT_EOF */
#define CI_INTSTATUS_EOF               (1 << 3)
/* Start-of-Frame, masked by CI_INT_SOF */
#define CI_INTSTATUS_SOF               (1 << 4)
/* Quick Capture Interface Disable Done, masked by CI_INT_CDD */
#define CI_INTSTATUS_CDD               (1 << 5)
/* Quick Capture Interface Quick Disable Status, masked by CI_INT_CQD */
#define CI_INTSTATUS_CQD               (1 << 6)
/* Parity Error, masked by CI_INT_PAR_ERR */
#define CI_INTSTATUS_PAR_ERR           (1 << 7)
/* End of Line, masked by CI_INT_EOL */
#define CI_INTSTATUS_EOL               (1 << 8)
/* Histogram Interrupt, unmaskable */
#define CI_INTSTATUS_HST_INT           (1 << 9)
/* Compander Interrupt, unmaskable */
#define CI_INTSTATUS_CGC_INT           (1 << 10)
/* FIFO Overrun for Channel 0, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_0             (1 << 11)
/* FIFO Overrun for Channel 1, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_1             (1 << 12)
/* FIFO Overrun for Channel 2, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_2             (1 << 13)
/* FIFO Overrun for Channel 3, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_3             (1 << 14)
/* FIFO Time-out, masked by CI_INT_FTO */
#define CI_INTSTATUS_FTO               (1 << 15)
/* Branch Status for Channel 0, masked by CI_INT_BS */
#define CI_INTSTATUS_BS0               (1 << 16)
/* Branch Status for Channel 1, masked by CI_INT_BS */
#define CI_INTSTATUS_BS1               (1 << 17)
/* Branch Status for Channel 2, masked by CI_INT_BS */
#define CI_INTSTATUS_BS2               (1 << 18)
/* Branch Status for Channel 3, masked by CI_INT_BS */
#define CI_INTSTATUS_BS3               (1 << 19)
/* Start-of-Frame for Channel 0, depending on Channel 0
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF0              (1 << 20)
/* Start-of-Frame for Channel 1, depending on Channel 1
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF1              (1 << 21)
/* Start-of-Frame for Channel 2, depending on Channel 2
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF2              (1 << 22)
/* Start-of-Frame for Channel 3, depending on Channel 3
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF3              (1 << 23)
/* Stop interrupt Channel 0, masked by CI_INT_SC0 */
#define CI_INTSTATUS_SC0               (1 << 24)
/* Stop interrupt Channel 1, masked by CI_INT_SC1 */
#define CI_INTSTATUS_SC1               (1 << 25)
/* Stop interrupt Channel 2, masked by CI_INT_SC2 */
#define CI_INTSTATUS_SC2               (1 << 26)
/* Stop interrupt Channel 3, masked by CI_INT_SC3 */
#define CI_INTSTATUS_SC3               (1 << 27)
/* Bus error in One or more DMA channels */
#define CI_INTSTATUS_BUSERR            (1 << 28)
/* Subsequent Interrupt Status, unmaskable */
#define CI_INTSTATUS_SINT              (1 << 31)
/* End-of-Frame Transferred to Memory (Channel 0-2,
 * not Channel 3) masked by CI_INT_EOFX
 */
#define CI_INTSTATUS_EOFX              (1 << 30)

#define CISR_VAL(ci_int_status) \
	((ci_int_status) & CI_CISR_MASK)

#define CIFSR_VAL(ci_int_status) (\
	  ((((ci_int_status) & (0xf << 11)) >> 11) <<  0) | \
	  ((((ci_int_status) & (0x1 <<  1)) >>  1) << 28) | \
	  ((((ci_int_status) & (0xf << 16)) >> 16) << 21) | \
	  ((((ci_int_status) & (0x1 <<  0)) >>  0) << 10) | \
	  ((((ci_int_status) & (0xf << 20)) >> 20) << 14)   \
	)

#define CIDCSR0_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 24)) >> 24) << 3)

#define CIDCSR1_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 25)) >> 25) << 3)

#define CIDCSR2_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 26)) >> 26) << 3)

#define CIDCSR3_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 27)) >> 27) << 3)

#define CI_INT_STATUS(cisr_val, cifsr_val, cidcsr0_val,		\
		 cidcsr1_val, cidcsr2_val, cidcsr3_val)		\
	(((cisr_val) & CI_CISR_MASK) |				\
	 ((((cifsr_val)   & (0xf <<  0)) >>  0) << 11) |	\
	 ((((cifsr_val)   & (0x1 << 10)) >> 10) <<  0) |	\
	 ((((cifsr_val)   & (0xf << 14)) >> 14) << 20) |	\
	 ((((cifsr_val)   & (0xf << 21)) >> 21) << 16) |	\
	 ((((cifsr_val)   & (0x1 << 28)) >> 28) <<  1) |	\
	 ((((cidcsr0_val) & (0x1 <<  3)) >>  3) << 24) |	\
	 ((((cidcsr1_val) & (0x1 <<  3)) >>  3) << 25) |	\
	 ((((cidcsr2_val) & (0x1 <<  3)) >>  3) << 26) |	\
	 ((((cidcsr3_val) & (0x1 <<  3)) >>  3) << 27) |	\
	 ((((cidcsr0_val) & (0x1 <<  0)) >>  0) << 28) |	\
	 ((((cidcsr1_val) & (0x1 <<  0)) >>  0) << 28) |	\
	 ((((cidcsr2_val) & (0x1 <<  0)) >>  0) << 28) |	\
	 ((((cidcsr3_val) & (0x1 <<  0)) >>  0) << 28)		\
	)

/*
   note:

   mapping between ci_int_status and related registers bits:
   ci_int_status:
  0   CIFSR[EOF3]
  1   CIFSR[FU_3]
  2   N/A
  3   CISR[EOF]
  4   CISR[SOF]
  5   CISR[CDD]
  6   CISR[CQD]
  7   CISR[PAR_ERR]
  8   CISR[EOL]
  9   CISR[HST_INT]
  10  CISR[CGC_INT]
  11  CIFSR[IFO_0]
  12  CIFSR[IFO_1]
  13  CIFSR[IFO_2]
  14  CIFSR[IFO_3]
  15  CISR[FTO]
  16  CIFSR[BS0]
  17  CIFSR[BS1]
  18  CIFSR[BS2]
  19  CIFSR[BS3]
  20  CIFSR[SOF0]
  21  CIFSR[SOF1]
  22  CIFSR[SOF2]
  23  CIFSR[SOF3]
  24  CIDCSR0[StopIrq]
  25  CIDCSR1[StopIrq]
  26  CIDCSR2[StopIrq]]
  27  CIDCSR3[StopIrq]
  28  CIDCSR0[BusErrIntr] | CIDCSR1[BusErrIntr] | CIDCSR2[BusErrIntr]| CIDCSR3[BusErrIntr]
  29  N/A
  30  CISR[EOFX]
  31  CISR[SINT]
*/

typedef enum CI_MODE{
	CI_MODE_MP,             /* Master-Parallel */
	CI_MODE_SP,             /* Slave-Parallel */
	CI_MODE_MS,             /* Master-Serial */
	CI_MODE_EP,             /* Embedded-Parallel */
	CI_MODE_ES              /* Embedded-Serial */
} CI_MODE;


typedef enum  {
	CI_FR_ALL = 0,          /* Capture all incoming frames */
	CI_FR_1_2,              /* Capture 1 out of every 2 frames */
	CI_FR_1_3,              /* Capture 1 out of every 3 frames */
	CI_FR_1_4,
	CI_FR_1_5,
	CI_FR_1_6,
	CI_FR_1_7,
	CI_FR_1_8
} CI_FRAME_CAPTURE_RATE;


typedef enum  {
	CI_FIFO_THL_32 = 0,
	CI_FIFO_THL_64,
	CI_FIFO_THL_96
} CI_FIFO_THRESHOLD;

typedef struct {
	unsigned int BFW;
	unsigned int BLW;
} CI_MP_TIMING, CI_MS_TIMING;

typedef struct {
	unsigned int BLW;
	unsigned int ELW;
	unsigned int HSW;
	unsigned int BFPW;
	unsigned int FSW;
	unsigned int BFW;
	unsigned int EFW;
	unsigned int VSW;
} CI_SP_TIMING;

typedef enum {
	CI_DATA_WIDTH4 = 0x0,
	CI_DATA_WIDTH5 = 0x1,
	CI_DATA_WIDTH8 = 0x2,
	CI_DATA_WIDTH9 = 0x3,
	CI_DATA_WIDTH10= 0x4
} CI_DATA_WIDTH;

#if defined(CONFIG_PXA310)
typedef enum {
	CI_NO_DOWN_SAMPLE,
	CI_YUV_420_DOWN_SAMPLE,
} CI_CICR4_YCBCR_DOWN_SAMPLE;
#endif

/*
 * Configuration APIs
 */
void ci_set_frame_rate(CI_FRAME_CAPTURE_RATE frate);
CI_FRAME_CAPTURE_RATE ci_get_frame_rate(void);
void ci_set_image_format(int input_format, int output_format);
void ci_set_mode(CI_MODE mode, CI_DATA_WIDTH data_width);
void ci_configure_mp(unsigned int PPL, unsigned int LPF, CI_MP_TIMING* timing);
void ci_configure_sp(unsigned int PPL, unsigned int LPF, CI_SP_TIMING* timing);
void ci_configure_ms(unsigned int PPL, unsigned int LPF, CI_MS_TIMING* timing);
void ci_configure_ep(int parity_check);
void ci_configure_es(int parity_check);
void ci_set_clock(int pclk_enable, int mclk_enable, unsigned int mclk_mhz);
void ci_set_polarity(int pclk_sample_falling, int hsync_active_low,
	int vsync_active_low);
void ci_set_fifo(unsigned int timeout, CI_FIFO_THRESHOLD threshold,
	int fifo1_enable, int fifo2_enable);
void ci_reset_fifo(void);
void ci_set_interrupt_mask(unsigned int mask);
unsigned int ci_get_interrupt_mask(void);
void ci_clear_interrupt_status(unsigned int status);
unsigned int ci_get_interrupt_status(void);
void ci_disable_complete(void);

#if defined(CONFIG_PXA310)
int ci_set_ycbcr_420_down_sample (CI_CICR4_YCBCR_DOWN_SAMPLE ycbcr_ds);
#endif

/*
 * Control APIs
 */
void ci_init(void);
void ci_deinit(void);
void ci_enable(void);
int  ci_disable(int quick, int wait_for_disable_complete);
void ci_slave_capture_enable(void);
void ci_slave_capture_disable(void);

/*
 * CI RAW data processing chain APIs
 */

/* Histogram Unit(HSU) related functions */
typedef enum {
	CI_HISTO_RED     = CI_CIHST_COLOR_RED,
	CI_HISTO_BLUE    = CI_CIHST_COLOR_BLUE,
	CI_HISTO_GREEN1  = CI_CIHST_COLOR_GREEN1,
	CI_HISTO_GREEN2  = CI_CIHST_COLOR_GREEN2
} CI_HSU_COLOR_TYPE;

typedef enum {
	CI_HSU_MUX_0_TO_7  = CI_CIHST_SCALE_0_TO_7,    /* for 8bit raw data */
	CI_HSU_MUX_0_TO_8  = CI_CIHST_SCALE_0_TO_8,    /* for 9bit raw data */
	CI_HSU_MUX_1_TO_9  = CI_CIHST_SCALE_1_TO_9     /* for 10bit raw data */
} CI_HSU_MUX_SEL_TYPE;

int ci_hsu_get_histgram(
		CI_HSU_COLOR_TYPE  color_type,
		CI_HSU_MUX_SEL_TYPE mux_select,
		unsigned int *lut_buffer_virtual,
		unsigned int  lut_buffer_physical ,
		unsigned int *lut_dma_desc_virtual,
		unsigned int  lut_dma_desc_physical,
		unsigned int  size,
		unsigned int *sum);

/* Pixel Substitute(PSU) related functions */
int ci_psu_tag_bad_pixel(int column, int row);

int ci_psu_enable(int enable);

/* Compand and Gamma Correction (CGU) related functions */
typedef enum {
	CI_CGU_MUX_0_TO_7  = CI_CICCR_SCALE_0_TO_7,    /* for 8bit raw data */
	CI_CGU_MUX_1_TO_8  = CI_CICCR_SCALE_1_TO_8,    /* for 9bit raw data */
	CI_CGU_MUX_2_TO_9  = CI_CICCR_SCALE_2_TO_9     /* for 10bit raw data */
} CI_CGU_MUX_SEL_TYPE;

int ci_cgu_set_addr_mux_select(CI_CGU_MUX_SEL_TYPE mux_select);

typedef enum {
	CI_CGU_LUT_RED      = CI_CICCR_CLUT_RED,
	CI_CGU_LUT_BLUE     = CI_CICCR_CLUT_BLUE,
	CI_CGU_LUT_GREEN    = CI_CICCR_CLUT_GREEN
} CI_CGU_LUT_TYPE;

int ci_cgu_load_lut_ram(
		unsigned int  *histogram_lut_buffer_virtual,
		unsigned int   histogram_lut_buffer_physical,
		unsigned int  *histogram_lut_dma_descriptors_virtual,
		unsigned int   histogram_lut_dma_descriptors_physical,
		unsigned char *lut_ram);

int ci_cgu_set_black_level(unsigned char black_level);

int ci_cgu_enable(int enable);


#define CI_SSU_SCALE_DEFAULT CI_SSU_SCALE_DISABLE

int ci_ssu_set_scale(CI_SSU_SCALE scale);

/* Color Synthesis Unit(CSU) related functions */

/* Color Management Unit(CMU) related functions */
typedef enum {
	CI_CMU_DISABLE    = CI_CICMR_DMODE_DISABLE,
	CI_CMU_OUTPUT_RGB = CI_CICMR_DMODE_RGB,
	CI_CMU_OUTPUT_YUV = CI_CICMR_DMODE_YUV
} CI_CMU_USAGE;

typedef struct {
	signed short k00, k01, k02;
	signed short k10, k11, k12;
	signed short k20, k21, k22;
} CI_CMU_COE_MATRIX;

/* just for debug use, will be removed later */
#define YUV_FLOAT_TO_INT(x) ((signed short)((float)x*(1UL << 7)) & 0x3ff)
/* example:
   static CI_CMU_COE_MATRIX cRGB24_to_YUV422_matrix = {
   YUV_FLOAT_TO_INT(0.257) , YUV_FLOAT_TO_INT(0.504) , YUV_FLOAT_TO_INT(0.098),
   YUV_FLOAT_TO_INT(-0.148),  YUV_FLOAT_TO_INT(0.291), YUV_FLOAT_TO_INT(0.439),
   YUV_FLOAT_TO_INT(0.439) , YUV_FLOAT_TO_INT(0.368) , YUV_FLOAT_TO_INT(0.071)
   };
 */

/* just for debug use, will be removed later */
#define RGB_FLOAT_TO_INT(x) ((signed short)((float)x*(1UL << 7)) & 0x3ff)
/* example:
   static CI_CMU_COE_MATRIX cRGB24_to_sRGB24_matrix = {
   RGB_FLOAT_TO_INT(1.780214),  RGB_FLOAT_TO_INT(-0.96883),
   RGB_FLOAT_TO_INT(0.188617),  RGB_FLOAT_TO_INT(-0.7987),
   RGB_FLOAT_TO_INT(1.790752), RGB_FLOAT_TO_INT(0.007949),
   RGB_FLOAT_TO_INT(-0.67645),  RGB_FLOAT_TO_INT(-1.60901),
   RGB_FLOAT_TO_INT(3.285467),
   };
 */

int ci_cmu_set_color_correction_coe(CI_CMU_COE_MATRIX *coe_matrix);

int ci_cmu_enable(CI_CMU_USAGE cmu_usage);

/*
 *	CI dedicated DMAC APIs
 */
typedef enum {
	CI_DMA_CHANNEL_0 = 0,
	CI_DMA_CHANNEL_1,
	CI_DMA_CHANNEL_2,
	CI_DMA_CHANNEL_3
} CI_DMA_CHANNEL;

struct ci_dma_descriptor {
	u32     ddadr;  /* descriptor address reg */
	u32     dsadr;  /* source address register */
	u32     dtadr;  /* target address register */
	u32     dcmd ;  /* command address register */
};

#define CI_DMAC_DCMD_LEN                  (1 <<  0)
#define CI_DMAC_DCMD_EOF_IRQ_EN           (1 << 21)
#define CI_DMAC_DCMD_SOF_IRQ_EN           (1 << 22)
#define CI_DMAC_DCMD_INC_TRG_ADDR         (1 << 30)


int ci_dma_load_descriptor(unsigned int dma_desc_phy, CI_DMA_CHANNEL channel);

int ci_dma_set_branch(
		unsigned int branch_to_dma_desc_phy,
		int branch_int_enable,
		int branch_after_cur_frame,
		CI_DMA_CHANNEL channel);

#endif


