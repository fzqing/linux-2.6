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
/* davinci_aew_hw.c file */

/* include driver header files */
#include <asm/arch-davinci/dm355_aew.h>
#include <asm/arch-davinci/dm355_aew_hw.h>
#include <linux/device.h>
extern struct device *aewdev;
/* Function to set hardware configuration registers */
int aew_register_setup(struct aew_device *aew_dev)
{
	unsigned int pcr = 0, win1 = 0, winstart = 0, blkwin = 0, subwin = 0;
	dev_dbg(aewdev, __FUNCTION__ "E\n");

	/* Set up the registers */
	pcr = regr(AEWPCR);

	/* Enable A Law */
	if (aew_dev->config->alaw_enable == H3A_AEW_ENABLE)
		pcr |= AEW_ALAW_EN;
	else
		pcr &= ~AEW_ALAW_EN;

	/*Configure Saturation limit */
	pcr &= ~AVE2LMT;
	pcr |= aew_dev->config->saturation_limit << AEW_AVE2LMT_SHIFT;
	/* Set Input Source */
	pcr &= ~AEW_INP_SRC;
	pcr |= (AEW_CCDC) << AEW_INP_SRC_SHIFT;

	regw(pcr, AEWPCR);

	/*Window parameter configuration */

	/* Configure Window Width in AEWWIN1 register */
	win1 = 0;
	win1 |=
	    ((AEW_SET_VAL(aew_dev->config->window_config.height)) <<
	     AEW_WINH_SHIFT);

	/* Configure Window height  in AEWWIN1 register */
	win1 |=
	    ((AEW_SET_VAL(aew_dev->config->window_config.width)) <<
	     AEW_WINW_SHIFT);

	/* Configure Window vertical count  in AEWWIN2 register */
	win1 |=
	    ((aew_dev->config->window_config).vt_cnt - 1) << AEW_VT_COUNT_SHIFT;

	/* Configure Window horizontal count  in AEWWIN1 register */
	win1 |= ((aew_dev->config->window_config).hz_cnt - 1);

	/* Configure Window vertical start  in AEWWIN1 register */
	regw(win1, AEWWIN1);

	/*Window Start parameter configuration */

	winstart &= ~WINSV;
	winstart |=
	    (aew_dev->config->window_config).vt_start << AEW_VT_START_SHIFT;

	/* Configure Window horizontal start  in AEWWIN2 register */
	winstart &= ~WINSH;
	winstart |= (aew_dev->config->window_config).hz_start;
	regw(winstart, AEWINSTART);

	/*Window Line Increment configuration */
	/*Configure vertical line increment in AEWSUBWIN */
	subwin &= ~AEWINCV;
	subwin |=
	    (AEW_SET_VAL(aew_dev->config->window_config.
			 vt_line_incr) << AEW_LINE_INCR_SHIFT);

	/* Configuring Horizontal Line increment in AEWSUBWIN */
	subwin &= ~AEWINCH;
	subwin |= (AEW_SET_VAL(aew_dev->config->window_config.hz_line_incr));

	regw(subwin, AEWSUBWIN);

	/* Black Window Configuration */
	/* Configure vertical start and height in AEWWINBLK */
	blkwin &= ~BLKWINSV;
	blkwin |=
	    (aew_dev->config->blackwindow_config).
	    vt_start << AEW_BLKWIN_VT_START_SHIFT;

	/* Configure height in Black window */
	blkwin &= ~BLKWINH;
	blkwin |= (AEW_SET_VAL(aew_dev->config->blackwindow_config.height));
	regw(blkwin, AEWINBLK);

	/* Set AFBUFST to Current buffer Physical Address */
	regw((unsigned int)(virt_to_phys(aew_dev->buff_curr)), AEWBUFST);
	dev_dbg(aewdev, "\n PCR is %x", regr(AEWPCR));
	dev_dbg(aewdev, "\n SUBWIN is %x", regr(AEWSUBWIN));
	dev_dbg(aewdev, "\n WINSTART is %x", regr(AEWINSTART));
	dev_dbg(aewdev, "\n WINBLK is %x", regr(AEWINBLK));
	dev_dbg(aewdev, "\n WIN1  is %x", regr(AEWWIN1));
	dev_dbg(aewdev, "\n AEWBUST %x", regr(AEWBUFST));

	AEW_SETGAMMAWD;
	dev_dbg(aewdev, __FUNCTION__ "L\n");
	return 0;
}

/* Function to enable/ disable AEW Engine */
inline void aew_engine_setup(int value)
{
	unsigned int pcr;
	dev_dbg(aewdev, __FUNCTION__ "E\n");
	dev_dbg(aewdev, "\nAEW_REG(PCR) Before Setting %x", regr(AEWPCR));

	/* Read Pcr Register */
	pcr = regr(AEWPCR);
	pcr &= ~AEW_EN;
	pcr |= (value << AEW_EN_SHIFT);

	/*Set AF_EN bit in PCR Register */
	regw(pcr, AEWPCR);

	dev_dbg(aewdev, "\nAfter Setting %d : PCR VALUE %x", value,
		regr(AEWPCR));
	dev_dbg(aewdev, __FUNCTION__ "L\n");

}

/* Function used to set adddress */
inline void aew_set_address(unsigned long address)
{
	dev_dbg(aewdev, __FUNCTION__ "E\n");
	regw(address, AEWBUFST);
	dev_dbg(aewdev, __FUNCTION__ "L\n");
}
