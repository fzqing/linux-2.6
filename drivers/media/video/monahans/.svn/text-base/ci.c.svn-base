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
 *     contains all primitive functions for Monahans Quick Capture Interface
 *
 * Notes:
 *     Only valid for processor code named Monahans.
 */

#include <linux/config.h>
#include <linux/delay.h>
#include <linux/stddef.h>
#include <asm/errno.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include "ci.h"

/*
 * Configuration APIs
 */
void ci_set_frame_rate(CI_FRAME_CAPTURE_RATE frate)
{
	unsigned int value;

	/* write cicr4 */
	value = CICR4;
	value &= ~(CI_CICR4_FR_RATE_SMASK << CI_CICR4_FR_RATE_SHIFT);
	value |= (unsigned)frate << CI_CICR4_FR_RATE_SHIFT;
	CICR4 = value;
}

CI_FRAME_CAPTURE_RATE ci_get_frame_rate()
{
	unsigned int value;
	value = CICR4;
	return (CI_FRAME_CAPTURE_RATE)((value >> CI_CICR4_FR_RATE_SHIFT) &
		CI_CICR4_FR_RATE_SMASK);
}

void ci_set_image_format(int input_format, int output_format)
{
	unsigned int value, tbit, rgbt_conv, rgb_conv, rgb_f;
	unsigned int ycbcr_f, rgb_bpp, raw_bpp, cspace;

	/* write cicr1: preserve ppl value and data width value */
	value = CICR1;
	value &= ((CI_CICR1_PPL_SMASK << CI_CICR1_PPL_SHIFT) |
		((CI_CICR1_DW_SMASK) << CI_CICR1_DW_SHIFT));

	tbit = rgbt_conv = rgb_conv = rgb_f = ycbcr_f = rgb_bpp	\
		= raw_bpp = cspace = 0;

	switch (input_format) {
		case CI_RAW8:
			cspace = 0;
			raw_bpp = 0;
			break;
		case CI_RAW9:
			cspace = 0;
			raw_bpp = 1;
			break;
		case CI_RAW10:
			cspace = 0;
			raw_bpp = 2;
			break;
		case CI_YCBCR422:
		case CI_YCBCR422_PLANAR:
			cspace = 2;
#if defined(CONFIG_PXA310)
			if (output_format == CI_YCBCR422_PLANAR || output_format == CI_YCBCR420_PLANAR) {
				ycbcr_f = 1;
			}
#else
			if (output_format == CI_YCBCR422_PLANAR) {
				ycbcr_f = 1;
			}
#endif
			break;
		case CI_RGB444:
			cspace = 1;
			rgb_bpp = 0;
			break;
		case CI_RGB555:
			cspace = 1;
			rgb_bpp = 1;
			if (output_format == CI_RGBT555_0) {
				rgbt_conv = 2;
				tbit = 0;
			}
			else if (output_format == CI_RGBT555_1) {
				rgbt_conv = 2;
				tbit = 1;
			}
			break;
		case CI_RGB565:
			cspace = 3;
			rgb_bpp = 2;
			rgb_f = 1;
			break;
		case CI_RGB666:
			cspace = 1;
			rgb_bpp = 3;
			if (output_format == CI_RGB666_PACKED) {
				rgb_f = 1;
			}
			break;
		case CI_RGB888:
		case CI_RGB888_PACKED:
			cspace = 1;
			rgb_bpp = 4;
			break;
		default:
			break;
	}

	switch (input_format) {
		case CI_RAW8:
		case CI_RAW9:
		case CI_RAW10:
#if defined(CONFIG_PXA310)
			if (output_format == CI_YCBCR422_PLANAR || output_format == CI_YCBCR420_PLANAR) {
				ycbcr_f = 1;
			}
#else
			if (output_format == CI_YCBCR422_PLANAR) {
				ycbcr_f = 1;
			}
#endif
		case CI_RGB888:
		case CI_RGB888_PACKED:
			switch (output_format) {
				case CI_RGB888_PACKED:
					rgb_f = 1;
					break;
				case CI_RGBT888_0:
					rgbt_conv = 1;
					tbit = 0;
					break;
				case CI_RGBT888_1:
					rgbt_conv = 1;
					tbit = 1;
					break;
				case CI_RGB666:
					rgb_conv = 1;
					break;
				case CI_RGB565:
					rgb_conv = 2;
					break;
				case CI_RGB555:
					rgb_conv = 3;
					break;
				case CI_RGB444:
					rgb_conv = 4;
					break;
				default:
					break;
			}
		default:
			break;
	}

	value |= (tbit==1) ? CI_CICR1_TBIT : 0;
	value |= rgbt_conv << CI_CICR1_RGBT_CONV_SHIFT;
	value |= rgb_conv << CI_CICR1_RGB_CONV_SHIFT;
	value |= (rgb_f==1) ? CI_CICR1_RBG_F : 0;
	value |= (ycbcr_f==1) ? CI_CICR1_YCBCR_F : 0;
	value |= rgb_bpp << CI_CICR1_RGB_BPP_SHIFT;
	value |= raw_bpp << CI_CICR1_RAW_BPP_SHIFT;
	value |= cspace << CI_CICR1_COLOR_SP_SHIFT;
	CICR1 = value;
}

void ci_set_mode(CI_MODE mode, CI_DATA_WIDTH data_width)
{
	unsigned int value;

	/* write mode field in cicr0 */
	value = CICR0;
	value &= ~(CI_CICR0_SIM_SMASK << CI_CICR0_SIM_SHIFT);
	value |= (unsigned int)mode << CI_CICR0_SIM_SHIFT;
	CICR0 = value;

	/* write data width cicr1 */
	value = CICR1;
	value &= ~(CI_CICR1_DW_SMASK << CI_CICR1_DW_SHIFT);
	value |= ((unsigned)data_width) << CI_CICR1_DW_SHIFT;
	CICR1 = value;
}

void ci_configure_mp(unsigned int PPL, unsigned int LPF, CI_MP_TIMING* timing)
{
	unsigned int value;

	/* write ppl field in cicr1 */
	value = CICR1;
	value &= ~(CI_CICR1_PPL_SMASK << CI_CICR1_PPL_SHIFT);
	value |= (PPL & CI_CICR1_PPL_SMASK) << CI_CICR1_PPL_SHIFT;
	CICR1 = value;

	/* write BLW, ELW in cicr2 */
	value = CICR2;
	value &= ~((unsigned int)CI_CICR2_BLW_SMASK << CI_CICR2_BLW_SHIFT |
		(unsigned int)CI_CICR2_ELW_SMASK << CI_CICR2_ELW_SHIFT);
	value |= (timing->BLW & CI_CICR2_BLW_SMASK) << CI_CICR2_BLW_SHIFT;
	CICR2 = value;

	/* write BFW, LPF in cicr3 */
	value = CICR3;
	value &= ~((unsigned int)CI_CICR3_BFW_SMASK << CI_CICR3_BFW_SHIFT |
		(unsigned int)CI_CICR3_LPF_SMASK << CI_CICR3_LPF_SHIFT);
	value |= (timing->BFW & CI_CICR3_BFW_SMASK) << CI_CICR3_BFW_SHIFT;
	value |= (LPF & CI_CICR3_LPF_SMASK) << CI_CICR3_LPF_SHIFT;
	CICR3 = value;
}

void ci_configure_sp(unsigned int PPL, unsigned int LPF, CI_SP_TIMING* timing)
{
	unsigned int value;

	/* write ppl field in cicr1 */
	value = CICR1;
	value &= ~(CI_CICR1_PPL_SMASK << CI_CICR1_PPL_SHIFT);
	value |= (PPL & CI_CICR1_PPL_SMASK) << CI_CICR1_PPL_SHIFT;
	CICR1 = value;

	/* write cicr2 */
	value |= (timing->BLW & CI_CICR2_BLW_SMASK) << CI_CICR2_BLW_SHIFT;
	value |= (timing->ELW & CI_CICR2_ELW_SMASK) << CI_CICR2_ELW_SHIFT;
	value |= (timing->HSW & CI_CICR2_HSW_SMASK) << CI_CICR2_HSW_SHIFT;
	value |= (timing->BFPW & CI_CICR2_BFPW_SMASK) << CI_CICR2_BFPW_SHIFT;
	value |= (timing->FSW & CI_CICR2_FSW_SMASK) << CI_CICR2_FSW_SHIFT;
	CICR2 = value;

	/* write cicr3 */
	value |= (timing->BFW & CI_CICR3_BFW_SMASK) << CI_CICR3_BFW_SHIFT;
	value |= (timing->EFW & CI_CICR3_EFW_SMASK) << CI_CICR3_EFW_SHIFT;
	value |= (timing->VSW & CI_CICR3_VSW_SMASK) << CI_CICR3_VSW_SHIFT;
	value |= (LPF & CI_CICR3_LPF_SMASK) << CI_CICR3_LPF_SHIFT;
	CICR3 = value;
}

void ci_configure_ms(unsigned int PPL, unsigned int LPF, CI_MS_TIMING* timing)
{
	/* the operation is same as Master-Parallel */
	ci_configure_mp(PPL, LPF, (CI_MP_TIMING*)timing);
}

void ci_configure_ep(int parity_check)
{
	unsigned int value;

	/* write parity_enable field in cicr0 */
	value = CICR0;
	if (parity_check) {
		value |= CICR0_PAR_EN;
	}
	else {
		value &= ~CICR0_PAR_EN;
	}
	CICR0 = value;
}

void ci_configure_es(int parity_check)
{
	/* the operationi is same as Embedded-Parallel */
	ci_configure_ep(parity_check);
}

void ci_set_clock(int pclk_enable, int mclk_enable, unsigned int mclk_mhz)
{
	unsigned int ciclk,  value, div, accr_hss, accr_d0cs, hss;
	int p;
	unsigned int x;

	/* determine the LCLK frequency programmed into the ACCR */
	accr_hss = (ACSR >> 14) & 0x3;
	accr_d0cs = (ACSR >> 26) & 0x1;

	hss = (accr_d0cs == 1)?60*100:
		(accr_hss == 0x0)?104*100:
		(accr_hss == 0x1)?156*100:
		208*100;/*unit: 10KHZ*/

#ifdef CONFIG_PXA310
	ciclk = hss;
#else
	ciclk = hss/2;
#endif

	x = ((ciclk / mclk_mhz) - 2);
	p =  x/2;

	if (x%2)
		div = p + 1;
	else
		div = p;

	/* write cicr4 */
	value = CICR4;
	value &= ~(CI_CICR4_PCLK_EN | CI_CICR4_MCLK_EN |
		CI_CICR4_DIV_SMASK<<CI_CICR4_DIV_SHIFT);
	value |= (pclk_enable) ? CI_CICR4_PCLK_EN : 0;
	value |= (mclk_enable) ? CI_CICR4_MCLK_EN : 0;
	value |= div << CI_CICR4_DIV_SHIFT;
	CICR4 = value;
}

void ci_set_polarity(int pclk_sample_falling,
		int hsync_active_low, int vsync_active_low)
{
	unsigned int value;

	/* write cicr4 */
	value = CICR4;
	value &= ~(CI_CICR4_PCP | CI_CICR4_HSP | CI_CICR4_VSP);
	value |= (pclk_sample_falling)? CI_CICR4_PCP : 0;
	value |= (hsync_active_low) ? CI_CICR4_HSP : 0;
	value |= (vsync_active_low) ? CI_CICR4_VSP : 0;
	CICR4 = value;
}

void ci_set_fifo(unsigned int timeout, CI_FIFO_THRESHOLD threshold,
		int fifo1_enable, int fifo2_enable)
{
	unsigned int value;

	/* write citor */
	CITOR = timeout;

	/* write cifr0: always enable fifo 0! also reset input fifo */
	value = CIFR0;
	value &= ~(CIFR_FEN0 | CIFR_FEN1 | CIFR_FEN2 | CIFR_RESET_F |
			CI_CIFR_THL_0_SMASK<<CI_CIFR_THL_0_SHIFT);
	value |= (unsigned int)threshold << CI_CIFR_THL_0_SHIFT;
	value |= (fifo1_enable) ? CIFR_FEN1 : 0;
	value |= (fifo2_enable) ? CIFR_FEN2 : 0;
	value |= CIFR_RESET_F | CIFR_FEN0;
	CIFR0 = value;
}

void ci_reset_fifo()
{
	unsigned int value;
	value = CIFR0;
	value |= CIFR_RESET_F;
	CIFR0 = value;
}

void ci_set_interrupt_mask(unsigned int mask)
{
	unsigned int value;

	/* write mask in cicr0 */
	value = CICR0;
	value &= ~CI_CICR0_INTERRUPT_MASK;
	value |= (CICR0_VAL(mask) & CI_CICR0_INTERRUPT_MASK);
	CICR0 = value;

	/* write mask in cidcsr0 */
	value = CIDCSR0;
	value |= CIDCSR0_VAL(mask);
	CIDCSR0 = value;

	/* write mask in cidcsr1 */
	value = CIDCSR1;
	value |= CIDCSR1_VAL(mask);
	CIDCSR1 = value;

	/* write mask in cidcsr2 */
	value = CIDCSR2;
	value |= CIDCSR2_VAL(mask);
	CIDCSR2 = value;

	/* write mask in cidcsr3 */
	value = CIDCSR3;
	value |= CIDCSR3_VAL(mask);
	CIDCSR3 = value;
}

unsigned int ci_get_interrupt_mask()
{
	unsigned int cicr0_val, cidcsr0_val;
	unsigned int  cidcsr1_val, cidcsr2_val, cidcsr3_val;

	/* get mask in cicr0 */
	cicr0_val = CICR0;

	/* get mask in cidcsr0 */
	cidcsr0_val = CIDCSR0;

	/* get mask in cidcsr1 */
	cidcsr1_val = CIDCSR1;

	/* get mask in cidcsr2 */
	cidcsr2_val = CIDCSR2;

	/* get mask in cidcsr3 */
	cidcsr3_val = CIDCSR3;

	return CI_INT_MASK(cicr0_val, cidcsr0_val,
		cidcsr1_val, cidcsr2_val, cidcsr3_val);
}

void ci_clear_interrupt_status(unsigned int status)
{
	unsigned int cidcsr0_val, cidcsr1_val, cidcsr2_val, cidcsr3_val;

	/* write 1 to clear cisr interrupt status */
	CISR = CISR_VAL(status);

	/* write 1 to clear cifsr interrupt status */
	CIFSR = CIFSR_VAL(status);

	/* write 1 to clear cidcsr0 interrupt status */
	if (CIDCSR0_STATUS_VAL(status)) {	/* if DMA channel stopped */
		CIDCSR0 = 0;		/* reset DMA channel */
	}

	/* write 1 to clear cidcsr1 interrupt status */
	if (CIDCSR1_STATUS_VAL(status)) {	/* if DMA channel stopped */
		CIDCSR1 = 0;		/* reset DMA channel */
	}

	/* write 1 to clear cidcsr2 interrupt status */
	if (CIDCSR2_STATUS_VAL(status)) {	/* if DMA channe2 stopped */
		CIDCSR2 = 0;		/* reset DMA channel */
	}

	/* write 1 to clear cidcsr3 interrupt status */
	if (CIDCSR3_STATUS_VAL(status)) {	/* if DMA channe3 stopped */
		CIDCSR3 = 0;		/* reset DMA channel */
	}

	/* write 1 to clear Bus error status */
	if (status & (1UL << 28)) {
		cidcsr0_val = CIDCSR0;
		cidcsr0_val |= 0x1;
		CIDCSR0 = cidcsr0_val;

		cidcsr1_val = CIDCSR1;
		cidcsr1_val |= 0x1;
		CIDCSR1 = cidcsr1_val;

		cidcsr2_val = CIDCSR2;
		cidcsr3_val |= 0x1;
		CIDCSR2 = cidcsr2_val;

		cidcsr3_val = CIDCSR3;
		cidcsr3_val |= 0x1;
		CIDCSR3 = cidcsr3_val;
	}
}

unsigned int ci_get_interrupt_status()
{
	unsigned int cisr_val, cifsr_val, cidcsr0_val;
	unsigned int cidcsr1_val, cidcsr2_val, cidcsr3_val;

	/* get interrupt status in cisr */
	cisr_val = CISR;

	/* get interrupt status in cifsr */
	cifsr_val = CIFSR;

	/* get interrupt status in cidcsr0 */
	cidcsr0_val = CIDCSR0;

	/* get interrupt status in cidcsr1 */
	cidcsr1_val = CIDCSR1;

	/* get interrupt status in cidcsr2 */
	cidcsr2_val = CIDCSR2;

	/* get interrupt status in cidcsr3 */
	cidcsr3_val = CIDCSR3;

	return CI_INT_STATUS(cisr_val, cifsr_val, cidcsr0_val,
		cidcsr1_val, cidcsr2_val, cidcsr3_val);
}

#if defined(CONFIG_PXA310)
int ci_set_ycbcr_420_down_sample (CI_CICR4_YCBCR_DOWN_SAMPLE ycbcr_ds)
{
	unsigned int value;

	/* write cicr4 */
	value = CICR4;
	if (ycbcr_ds == CI_NO_DOWN_SAMPLE) {
		value &= ~(CI_CICR4_YCBCR_DS);
	} else {
		value |= CI_CICR4_YCBCR_DS;
	}
	CICR4 = value;

	return 0;
}
#endif

/*
 * Control APIs
 */
void ci_init()
{
	/* clear all CI registers */

	/* disable all interrupts */
	CICR0 = CI_CICR0_INTERRUPT_MASK;
	CICR1 = 0;
	CICR2 = 0;
	CICR3 = 0;
	CICR4 = 0;
	CISR  = ~0;
	CIFR0 = 0;
	CITOR = 0;
	CIRCD = 0xF0000000;
}

void ci_deinit()
{
}

void ci_enable()
{
	unsigned int value;

	/* write mask in cicr0 */
	value = CICR0;
	value |= CICR0_ENB;

	CICR0 = value;
}

void ci_disable_complete()
{
	unsigned int value;

	/* Clear the disable control bit */
	value = CICR0;
	value &= ~CICR0_DIS;
	CICR0 = value;
}

int ci_disable(int quick, int wait_for_disable_complete)
{
	unsigned int value, mask;
	int retry;


	value = CICR0;
	if (!(value&CICR0_ENB)) {
		return 0;
	}

	value = CISR;
	CISR = value;

	/* write control bit in cicr0 */
	value = CICR0;
	if (quick) {
		value &= ~CICR0_ENB;
		mask = CI_CISR_CQD;
	} else {
		value |= CICR0_DIS;
		mask = CI_CISR_CDD;
	}
	CICR0 = value;

	if (wait_for_disable_complete) {
		/* wait shutdown complete */
		retry = 50;
		while (retry-- > 0) {
			value = CISR;
			if (value & mask) {
				CISR = mask;
				return 0;
			}
			mdelay(10);
		}
	} else {
		return 0;
	}

	return -1;
}

void ci_slave_capture_enable()
{
	unsigned int value;

	/* write mask in cicr0 */
	value = CICR0;
	value |= CICR0_SL_CAP_EN;
	CICR0 = value;
}

void ci_slave_capture_disable(void)
{
	unsigned int value;

	/* write mask in cicr0 */
	value = CICR0;
	value &= ~CICR0_SL_CAP_EN;
	CICR0 = value;
}

/*
 * CI RAW data processing chain APIs
 */
int ci_hsu_get_histgram (CI_HSU_COLOR_TYPE  color_type,
		CI_HSU_MUX_SEL_TYPE mux_select,
		unsigned int *histogram_lut_buffer_virtual,
		unsigned int  histogram_lut_buffer_physical,
		unsigned int *histogram_lut_dma_descriptors_virtual,
		unsigned int  histogram_lut_dma_descriptors_physical,
		unsigned int  histogram_size,
		unsigned int *histogram_sum)
{
	int tries;
	volatile struct ci_dma_descriptor *dma_desc;
	unsigned int reg_val;

	if (histogram_lut_buffer_virtual == NULL) {
		return -EINVAL;
	}

	/* enable FIFO3 */
	reg_val = CIFR1;
	reg_val |= CI_CIFR_FEN3;
	CIFR1 = reg_val;

	/* clear the CI sum register */
	reg_val = 0x00000000;
	CISUM = reg_val;

	/* clear the CI Histgram RAM and EOF3 */
	reg_val = CIHST;
	reg_val |= CI_CIHST_CLR_RAM;
	CIHST = reg_val;

	CIFSR = CI_CIFSR_EOF3;

	/* wait for rma clear interrupt */
	for (tries = 5; tries >= 0; tries--) {
		if (CISR & CI_CISR_HST_INT) {
			CISR = CI_CISR_HST_INT;
			break;
		}
		mdelay(10);

		if (tries == 0) {
			return -EBUSY;
		}
	}

	/* configure the DMA for fifo3 */
	dma_desc = (struct ci_dma_descriptor*)histogram_lut_dma_descriptors_virtual;
	dma_desc->ddadr = histogram_lut_dma_descriptors_physical;
	dma_desc->dsadr = __PREG_3(CIBR3);
	dma_desc->dtadr = histogram_lut_buffer_physical;
	dma_desc->dcmd  = CI_DMAC_DCMD_INC_TRG_ADDR | histogram_size;


	ci_dma_load_descriptor(histogram_lut_dma_descriptors_physical,
		CI_DMA_CHANNEL_3);

	/* enable histgram */
	reg_val = CIHST;
	reg_val &= ~(CI_CIHST_COLOR_SEL_SMASK << CI_CIHST_COLOR_SEL_SHIFT);
	reg_val |= (color_type << CI_CIHST_COLOR_SEL_SHIFT);
	reg_val &= ~(CI_CIHST_SCALE_SMASK << CI_CIHST_SCALE_SHIFT);
	reg_val |= (mux_select << CI_CIHST_SCALE_SHIFT);
	CIHST = reg_val;

	/* wait for fifo3 end of frame
	 * WARNING: to judge if the getting histgram process is completed,
	 *	    don't clear CI_INTSTATUS_EOF3 in  the interrupt handler!!!!!
	 */
	for (tries = 100; tries >= 0; tries--) {
		reg_val = CIFSR;
		if (reg_val & CI_CIFSR_EOF3) {
			CIFSR = CI_CIFSR_EOF3;
			break;
		}
		mdelay(10);
		if (tries == 0)
			return -EBUSY;
	}

	/* stop Histgram */
	reg_val = CIHST;
	reg_val &= ~(CI_CIHST_COLOR_SEL_SMASK << CI_CIHST_COLOR_SEL_SHIFT);
	reg_val |= (0 << CI_CIHST_COLOR_SEL_SHIFT);
	CIHST = reg_val;

	/* disable FIFO3 */
	reg_val = CIFR1;
	reg_val &= ~CI_CIFR_FEN3;
	CIFR1 = reg_val;

	*histogram_sum = CISUM;

	return 0;
}

int ci_psu_tag_bad_pixel(int column, int row)
{
	unsigned int reg_val;

	reg_val = CIPBUF;

	reg_val &= ~((CI_CIPBUF_DEADCOL_SMASK << CI_CIPBUF_DEADCOL_SHIFT) |
			(CI_CIPBUF_DEADROW_SMASK << CI_CIPBUF_DEADROW_SHIFT));

	reg_val |= ((column << CI_CIPBUF_DEADCOL_SHIFT) |
			(row << CI_CIPBUF_DEADROW_SHIFT));

	CIPBUF = reg_val;

	return 0;
}

int ci_psu_enable(int enable)
{
	unsigned int reg_val;

	reg_val = CIPSS;

	if (enable) {
		reg_val |= CIPSS_PSU_EN;
	} else {
		reg_val &= ~CIPSS_PSU_EN;
	}

	CIPSS = reg_val;

	return 0;
}

int ci_cgu_set_black_level(unsigned char black_level)
{
	unsigned int reg_val;

	reg_val = CICCR;

	reg_val &= ~(CI_CICCR_BLC_SMASK << CI_CICCR_BLC_SHIFT);
	reg_val &= ~(CI_CICCR_CLUT_SMASK << CI_CICCR_CLUT_SHIFT);
	reg_val |= (black_level << CI_CICCR_BLC_SHIFT);

	CICCR = reg_val;

	return 0;
}

int ci_cgu_set_addr_mux_select(CI_CGU_MUX_SEL_TYPE mux_select)
{
	unsigned int reg_val;

	reg_val = CICCR;

	reg_val &= ~(CI_CICCR_SCALE_SMASK << CI_CICCR_SCALE_SHIFT);
	reg_val |= (mux_select << CI_CICCR_SCALE_SHIFT);

	CICCR = reg_val;

	return 0;
}

int ci_cgu_load_lut_ram(
		unsigned int  *histogram_lut_buffer_virtual,
		unsigned int   histogram_lut_buffer_physical,
		unsigned int  *histogram_lut_dma_descriptors_virtual,
		unsigned int   histogram_lut_dma_descriptors_physical,
		unsigned char *lut_ram)
{
	int tries;
	volatile struct ci_dma_descriptor *dma_desc;
	unsigned int reg_val;
	int i;
	unsigned char *src, *dst;

	/* enable FIFO3 */
	reg_val = CIFR1;
	reg_val |= CI_CIFR_FEN3;
	CIFR1 = reg_val;

	/* clear CIFSR.EOF3 at first */
	CIFSR = CI_CIFSR_EOF3;

	/* memcopy the LUT data to the memory which can be used by DMA */
	src = lut_ram;
	dst = (unsigned char*)histogram_lut_buffer_virtual;
	for (i = 0; i < 192; i++) {
		*dst = *src;
		dst++;
		src++;
	}

	for (i=0; i<3; i++) {
		/* configure the DMA for fifo3 */
		dma_desc = (struct ci_dma_descriptor*)	\
			histogram_lut_dma_descriptors_virtual;
		dma_desc->ddadr = histogram_lut_dma_descriptors_physical;
		dma_desc->dsadr = histogram_lut_buffer_physical + i*64;
		dma_desc->dtadr = __PREG_3(CIBR3);
		dma_desc->dcmd  = 64;
		ci_dma_load_descriptor(histogram_lut_dma_descriptors_physical,
			CI_DMA_CHANNEL_3);

		/* start loading the LUT ram */
		reg_val = CICCR;
		reg_val &= ~(CI_CICCR_CLUT_SMASK << CI_CICCR_CLUT_SHIFT);
		reg_val |= (CI_CICCR_CLUT_RED << CI_CICCR_CLUT_SHIFT);
		reg_val |= CI_CICCR_LUT_LD;
		CICCR = reg_val;

		/* wait for LUT ram loading process completed */
		for (tries = 100; tries >= 0; tries--) {
			if ((CICCR & CI_CICCR_LUT_LD) == 0) {
				break;
			}
			mdelay(10);

			if (tries == 0) {
				return -EBUSY;
			}
		}
	}

	/* disable FIFO3 */
	reg_val = CIFR1;
	reg_val &= ~CI_CIFR_FEN3;
	CIFR1 = reg_val;

	return 0;
}

int ci_cgu_enable(int enable)
{
	unsigned int reg_val;

	reg_val = CICCR;

	if (enable) {
		reg_val |= CI_CICCR_EN;
	} else {
		reg_val &= ~CI_CICCR_EN;
	}

	CICCR = reg_val;

	return 0;
}

int ci_ssu_set_scale(CI_SSU_SCALE scale)
{
	unsigned int reg_val;

	reg_val = CISSC;

	reg_val &= ~(CI_CISSC_SCALE_SMASK << CI_CISSC_SCALE_SHIFT);
	reg_val |= (scale << CI_CISSC_SCALE_SHIFT);

	CISSC = reg_val;

	return 0;
}

int ci_cmu_set_color_correction_coe(CI_CMU_COE_MATRIX *coe_matrix)
{
	unsigned int reg_val;

	/* set k00, k01, k02 to CICMC0 */
	reg_val = CICMC0;
	reg_val &= ~((CI_CICMC0_COF02_SMASK << CI_CICMC0_COF02_SHIFT) |
			(CI_CICMC0_COF01_SMASK << CI_CICMC0_COF01_SHIFT) |
			(CI_CICMC0_COF00_SMASK << CI_CICMC0_COF00_SHIFT));
	reg_val |=  ((coe_matrix->k00 << CI_CICMC0_COF00_SHIFT) |
			(coe_matrix->k01 << CI_CICMC0_COF01_SHIFT) |
			(coe_matrix->k02 << CI_CICMC0_COF02_SHIFT));
	CICMC0 = reg_val;

	/* set k10, k11, k12 to CICMC1 */
	reg_val = CICMC1;
	reg_val &= ~((CI_CICMC1_COF12_SMASK << CI_CICMC1_COF12_SHIFT) |
			(CI_CICMC1_COF11_SMASK << CI_CICMC1_COF11_SHIFT) |
			(CI_CICMC1_COF10_SMASK << CI_CICMC1_COF10_SHIFT));
	reg_val |=  ((coe_matrix->k10 << CI_CICMC1_COF10_SHIFT) |
			(coe_matrix->k11 << CI_CICMC1_COF11_SHIFT) |
			(coe_matrix->k12 << CI_CICMC1_COF12_SHIFT));
	CICMC1 = reg_val;

	/* set k20, k21, k22 to CICMC2 */
	reg_val = CICMC2;
	reg_val &= ~((CI_CICMC2_COF22_SMASK << CI_CICMC2_COF22_SHIFT) |
			(CI_CICMC2_COF21_SMASK << CI_CICMC2_COF21_SHIFT) |
			(CI_CICMC2_COF20_SMASK << CI_CICMC2_COF20_SHIFT));
	reg_val |=  ((coe_matrix->k20 << CI_CICMC2_COF20_SHIFT) |
			(coe_matrix->k21 << CI_CICMC2_COF21_SHIFT) |
			(coe_matrix->k22 << CI_CICMC2_COF22_SHIFT));
	CICMC2 = reg_val;

	return 0;
}

int ci_cmu_enable(CI_CMU_USAGE cmu_usage)
{
	unsigned int reg_val;

	reg_val = CICMR;

	reg_val &= ~(CI_CICMR_DMODE_SMASK << CI_CICMR_DMODE_SHIFT);
	reg_val |= (cmu_usage << CI_CICMR_DMODE_SHIFT);

	CICMR = reg_val;

	return 0;
}

/*
 * CI dedicated DMAC APIs
 */
int ci_dma_load_descriptor(unsigned int dma_desc_phy, CI_DMA_CHANNEL channel)
{
	*(&CIDADR0 + channel*4) = dma_desc_phy;

	return 0;
}

int ci_dma_set_branch (
		unsigned int branch_to_dma_desc_phy,
		int branch_int_enable,
		int branch_after_cur_frame,
		CI_DMA_CHANNEL channel)
{
	unsigned int reg_val;

	/* note: to enable Branch Interrupt, CI_INT_BS should be enabled
	 * as well as branch_int_enable is set as 1
	 */
	reg_val = (branch_to_dma_desc_phy & CI_CIDBR_SRCADDR_SMASK);
	reg_val |= (branch_int_enable == 1)?CI_CIDBR_BINT:0;
	reg_val |= (branch_after_cur_frame == 1)?CI_CIDBR_BRA:0;

	*(&CIDBR0 + channel) = reg_val;

	return 0;
}

