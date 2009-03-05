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
/* Include driver header file */
#include <asm/arch/davinci_af_hw.h>
#include <linux/device.h>
extern struct device *afdev;
/* Function to set register */
int af_register_setup(struct af_device *af_dev)
{
	unsigned int pcr = 0, pax1 = 0, pax2 = 0, paxstart = 0;
	unsigned int coef = 0;
	unsigned int base_coef_set0 = 0;
	unsigned int base_coef_set1 = 0;
	int index;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	/* Configure Hardware Registers */
	/* Set PCR Register */
	pcr = regr(AFPCR);	/* Read PCR Register */

	/*Set Accumulator Mode */
	if (af_dev->config->mode == ACCUMULATOR_PEAK)
		pcr |= FVMODE;
	else
		pcr &= ~FVMODE;

	/*Set A-law */
	if (af_dev->config->alaw_enable == H3A_AF_ENABLE)
		pcr |= AF_ALAW_EN;
	else
		pcr &= ~AF_ALAW_EN;

	/*Set RGB Position */
	pcr &= ~RGBPOS;
	pcr |= (af_dev->config->rgb_pos) << AF_RGBPOS_SHIFT;

	/*HMF Configurations */
	if (af_dev->config->hmf_config.enable == H3A_AF_ENABLE) {
		pcr &= ~AF_MED_EN;
		/* Enable HMF */
		pcr |= AF_MED_EN;

		/* Set Median Threshold */
		pcr &= ~MED_TH;
		pcr |=
		    (af_dev->config->hmf_config.threshold) << AF_MED_TH_SHIFT;
	} else
		pcr &= ~AF_MED_EN;

	/* Set PCR Register */
	regw(pcr, AFPCR);

	/* Configure AFPAX1 */
	/*Paxel parameter configuration */
	/*Set Width in AFPAX1 Register */
	pax1 &= ~PAXW;
	pax1 |= (af_dev->config->paxel_config.width) << AF_PAXW_SHIFT;

	/* Set height in AFPAX1 */
	pax1 &= ~PAXH;
	pax1 |= af_dev->config->paxel_config.height;

	regw(pax1, AFPAX1);

	/* Configure AFPAX2 Register */
	/* Set Line Increment in AFPAX2 Register */
	pax2 &= ~AFINCV;
	pax2 |= (af_dev->config->paxel_config.line_incr) << AF_LINE_INCR_SHIFT;
	/* Set Vertical Count */
	pax2 &= ~PAXVC;
	pax2 |= (af_dev->config->paxel_config.vt_cnt) << AF_VT_COUNT_SHIFT;
	/* Set Horizontal Count */
	pax2 &= ~PAXHC;
	pax2 |= af_dev->config->paxel_config.hz_cnt;
	regw(pax2, AFPAX2);

	/* Configure PAXSTART Register */
	/*Configure Horizontal Start */
	paxstart &= ~PAXSH;
	paxstart |=
	    (af_dev->config->paxel_config.hz_start) << AF_HZ_START_SHIFT;
	/* Configure Vertical Start */
	paxstart &= ~PAXSV;
	paxstart |= af_dev->config->paxel_config.vt_start;
	regw(paxstart, AFPAXSTART);

	/*SetIIRSH Register */
	regw(af_dev->config->iir_config.hz_start_pos, AFIIRSH);

	/*Set IIR Filter0 Coefficients */
	base_coef_set0 = AFCOEF010;
	for (index = 0; index <= 8; index += 2) {
		coef &= ~COEF_MASK0;
		coef |= af_dev->config->iir_config.coeff_set0[index];
		coef &= ~COEF_MASK1;
		coef |=
		    (af_dev->config->iir_config.
		     coeff_set0[index + 1]) << AF_COEF_SHIFT;
		regw(coef, base_coef_set0);
		dev_dbg(afdev, "\n COEF0 %x", regr(base_coef_set0));
		base_coef_set0 = base_coef_set0 + AFCOEF_OFFSET;
	}

	/* set AFCOEF0010 Register */
	regw(af_dev->config->iir_config.coeff_set0[10], AFCOEF0010);

	/*Set IIR Filter1 Coefficients */

	base_coef_set1 = AFCOEF110;
	for (index = 0; index <= 8; index += 2) {
		coef &= ~COEF_MASK0;
		coef |= af_dev->config->iir_config.coeff_set1[index];
		coef &= ~COEF_MASK1;
		coef |=
		    (af_dev->config->iir_config.
		     coeff_set1[index + 1]) << AF_COEF_SHIFT;
		regw(coef, base_coef_set1);
		dev_dbg(afdev, "\n COEF1 %x", regr(base_coef_set1));
		base_coef_set1 = base_coef_set1 + AFCOEF_OFFSET;
	}
	/* Set AFCOEF0110 */
	regw(af_dev->config->iir_config.coeff_set1[10], AFCOEF1010);

	/*Set AFBUFST to Current buffer Physical Address */
	regw((unsigned int)(virt_to_phys(af_dev->buff_curr)), AFBUFST);

	dev_dbg(afdev, "\n PCR %x", pcr);
	dev_dbg(afdev, "\n AFPAX1 %x", regr(AFPAX1));
	dev_dbg(afdev, "\n PAXSTART %x", paxstart);
	dev_dbg(afdev, "\n PAX2 %x", regr(AFPAX2));
	dev_dbg(afdev, "\n COEF 10  %x", regr(AFCOEF0010));
	dev_dbg(afdev, "\n COEF 10  %x", regr(AFCOEF1010));
	dev_dbg(afdev, "\n AFBUFST %x", regr(AFBUFST));
	dev_dbg(afdev, __FUNCTION__ "L\n");
	return 0;
}

/* Function to Enable/Disable AF Engine */
inline void af_engine_setup(int enable)
{
	unsigned int pcr;
	dev_dbg(afdev, __FUNCTION__ "E\n");

	pcr = regr(AFPCR);
	dev_dbg(afdev, "\n Engine Setup value before PCR : %x", pcr);

	/* Set AF_EN bit in PCR Register */
	if (enable)
		pcr |= AF_EN;
	else
		pcr &= ~AF_EN;

	regw(pcr, AFPCR);

	dev_dbg(afdev, "\n Engine Setup value after PCR : %x", pcr);
	dev_dbg(afdev, __FUNCTION__ "L\n");
}

/* Function to set address */
inline void af_set_address(unsigned long address)
{
	regw(address, AFBUFST);
}
