/*
 * 
 *
 * Copyright (C) 2005 Texas Instruments Inc
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
/* ccdc_davinci.c */
#include <linux/device.h>
#include <linux/delay.h>
#include <asm/arch/mux.h>
#include <media/davinci/ccdc_dm355.h>
/*Object for CCDC raw mode */
ccdc_params_raw ccdc_hw_params_raw = {
	.pix_fmt = CCDC_PIXFMT_RAW,
	.frm_fmt = CCDC_FRMFMT_PROGRESSIVE,
	.win = VPFE_WIN_PAL,
	.fid_pol = CCDC_PINPOL_POSITIVE,
	.vd_pol = CCDC_PINPOL_POSITIVE,
	.hd_pol = CCDC_PINPOL_POSITIVE,
	.image_invert_enable = FALSE,
	.data_sz = _10BITS,
	.med_filt_thres = 0,
	.mfilt1 = NO_MEDIAN_FILTER1,
	.mfilt2 = NO_MEDIAN_FILTER2,
	.ccdc_offset = 0,
	.lpf_enable = FALSE,
	.datasft = 2,
	.alaw = {
		 .b_alaw_enable = FALSE,
		 .gama_wd = 2}
	,
	.blk_clamp = {
		      .b_clamp_enable = FALSE,
		      .sample_pixel = 1,
		      .start_pixel = 0,
		      .dc_sub = 25}
	,
	.blk_comp = {
		     .b_comp = 0,
		     .gb_comp = 0,
		     .gr_comp = 0,
		     .r_comp = 0}
	,
	.vertical_dft = {
			 .ver_dft_en = FALSE}
	,
	.lens_sh_corr = {
			 .lsc_enable = FALSE}
	,
	.data_formatter_r = {
			     .fmt_enable = FALSE}
	,
	.color_space_con = {
			    .csc_enable = FALSE}
};

/*Object for CCDC ycbcr mode */
ccdc_params_ycbcr ccdc_hw_params_ycbcr = {
	.pix_fmt = CCDC_PIXFMT_YCBCR_8BIT,
	.frm_fmt = CCDC_FRMFMT_INTERLACED,
	.win = VPFE_WIN_PAL,
	.fid_pol = CCDC_PINPOL_POSITIVE,
	.vd_pol = CCDC_PINPOL_POSITIVE,
	.hd_pol = CCDC_PINPOL_POSITIVE,
	.bt656_enable = TRUE,
	.pix_order = CCDC_PIXORDER_CBYCRY,
	.buf_type = CCDC_BUFTYPE_FLD_INTERLEAVED
};

extern struct device *vpfe_dev;
void ccdc_readregs(void)
{
	unsigned int val = 0;

	val = regr(SYNCEN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to SYNCEN...\n", val);
	val = regr(MODESET);
	dev_dbg(vpfe_dev, "\nReading 0x%x to MODESET...\n", val);
	val = regr(HDWIDTH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to HDWIDTH...\n", val);
	val = regr(VDWIDTH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to VDWIDTH...\n", val);
	val = regr(PPLN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to PPLN...\n", val);
	val = regr(LPFR);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LPFR...\n", val);
	val = regr(SPH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to SPH...\n", val);
	val = regr(NPH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to NPH...\n", val);
	val = regr(SLV0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to SLV0...\n", val);
	val = regr(SLV1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to SLV1...\n", val);
	val = regr(NLV);
	dev_dbg(vpfe_dev, "\nReading 0x%x to NLV...\n", val);
	val = regr(CULH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CULH...\n", val);
	val = regr(CULV);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CULV...\n", val);
	val = regr(HSIZE);
	dev_dbg(vpfe_dev, "\nReading 0x%x to HSIZE...\n", val);
	val = regr(SDOFST);
	dev_dbg(vpfe_dev, "\nReading 0x%x to SDOFST...\n", val);
	val = regr(STADRH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to STADRH...\n", val);
	val = regr(STADRL);
	dev_dbg(vpfe_dev, "\nReading 0x%x to STADRL...\n", val);
	val = regr(CLAMP);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CLAMP...\n", val);
	val = regr(DCSUB);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DCSUB...\n", val);
	val = regr(COLPTN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to COLPTN...\n", val);

	val = regr(BLKCMP0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to BLKCMP0...\n", val);
	val = regr(BLKCMP1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to BLKCMP1...\n", val);
	val = regr(MEDFILT);
	dev_dbg(vpfe_dev, "\nReading 0x%x to MEDFILT...\n", val);
	val = regr(RYEGAIN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to RYEGAIN...\n", val);
	val = regr(GRCYGAIN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to GRCYGAIN...\n", val);
	val = regr(GBGGAIN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to GBGGAIN...\n", val);
	val = regr(BMGGAIN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to BMGGAIN...\n", val);
	val = regr(OFFSET);
	dev_dbg(vpfe_dev, "\nReading 0x%x to OFFSET...\n", val);
	val = regr(OUTCLIP);
	dev_dbg(vpfe_dev, "\nReading 0x%x to OUTCLIP...\n", val);
	val = regr(VDINT0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to VDINT0...\n", val);
	val = regr(VDINT1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to VDINT1...\n", val);
	val = regr(RSV0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to RSV0...\n", val);
	val = regr(GAMMAWD);
	dev_dbg(vpfe_dev, "\nReading 0x%x to GAMMAWD...\n", val);
	val = regr(REC656IF);
	dev_dbg(vpfe_dev, "\nReading 0x%x to REC656IF...\n", val);
	val = regr(CCDCFG);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CCDCFG...\n", val);
	val = regr(FMTCFG);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTCFG...\n", val);
	val = regr(FMTPLEN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPLEN...\n", val);
	val = regr(FMTSPH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTSPH...\n", val);
	val = regr(FMTLNH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTLNH...\n", val);
	val = regr(FMTSLV);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTSLV...\n", val);
	val = regr(FMTLNV);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTLNV...\n", val);
	val = regr(FMTRLEN);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTRLEN...\n", val);
	val = regr(FMTHCNT);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTHCNT...\n", val);
	val = regr(FMT_ADDR_PTR_B);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMT_ADDR_PTR_B...\n", val);
	val = regr(FMTPGM_VF0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_VF0...\n", val);
	val = regr(FMTPGM_VF1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_VF1...\n", val);
	val = regr(FMTPGM_AP0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP0...\n", val);
	val = regr(FMTPGM_AP1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP1...\n", val);
	val = regr(FMTPGM_AP2);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP2...\n", val);
	val = regr(FMTPGM_AP3);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP3...\n", val);
	val = regr(FMTPGM_AP4);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP4...\n", val);
	val = regr(FMTPGM_AP5);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP5...\n", val);
	val = regr(FMTPGM_AP6);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP6...\n", val);
	val = regr(FMTPGM_AP7);
	dev_dbg(vpfe_dev, "\nReading 0x%x to FMTPGM_AP7...\n", val);

	val = regr(LSCCFG1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCCFG1...\n", val);
	val = regr(LSCCFG2);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCCFG2...\n", val);
	val = regr(LSCH0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCH0...\n", val);
	val = regr(LSCV0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCV0...\n", val);
	val = regr(LSCKH);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCKH...\n", val);
	val = regr(LSCKV);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCKV...\n", val);
	val = regr(LSCMEMCTL);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCMEMCTL...\n", val);
	val = regr(LSCMEMD);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCMEMD...\n", val);
	val = regr(LSCMEMQ);
	dev_dbg(vpfe_dev, "\nReading 0x%x to LSCMEMQ...\n", val);
	val = regr(DFCCTL);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCCTL...\n", val);
	val = regr(DFCVSAT);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCVSAT...\n", val);
	val = regr(DFCMEMCTL);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCMEMCTL...\n", val);
	val = regr(DFCMEM0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCMEM0...\n", val);
	val = regr(DFCMEM1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCMEM1...\n", val);
	val = regr(DFCMEM2);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCMEM2...\n", val);
	val = regr(DFCMEM3);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCMEM3...\n", val);
	val = regr(DFCMEM4);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DFCMEM4...\n", val);
	val = regr(CSCCTL);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCCTL...\n", val);
	val = regr(CSCM0);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM0...\n", val);
	val = regr(CSCM1);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM1...\n", val);

	val = regr(CSCM2);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM2...\n", val);

	val = regr(CSCM3);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM3...\n", val);
	val = regr(CSCM4);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM4...\n", val);
	val = regr(CSCM5);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM5...\n", val);

	val = regr(CSCM6);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM6...\n", val);
	val = regr(CSCM7);
	dev_dbg(vpfe_dev, "\nReading 0x%x to CSCM7...\n", val);
	val = regr(DATAOFST);
	dev_dbg(vpfe_dev, "\nReading 0x%x to DATAOFST...\n", val);
}

int ccdc_update_raw_params(void *arg)
{
	memcpy(&ccdc_hw_params_raw,
	       (ccdc_config_params_raw *) arg, sizeof(ccdc_config_params_raw));
	return 0;
}

int ccdc_update_ycbcr_params(void *arg)
{
	memcpy(&ccdc_hw_params_ycbcr,
	       (ccdc_params_ycbcr *) arg, sizeof(ccdc_params_ycbcr));
	return 0;
}

void ccdc_init(void)
{
	davinci_cfg_reg(DM355_VIN_PCLK);
	davinci_cfg_reg(DM355_VIN_CAM_WEN);
	davinci_cfg_reg(DM355_VIN_CAM_VD);
	davinci_cfg_reg(DM355_VIN_CAM_HD);
	davinci_cfg_reg(DM355_VIN_YIN_EN);
	davinci_cfg_reg(DM355_VIN_CINL_EN);
	davinci_cfg_reg(DM355_VIN_CINH_EN);
}

void ccdc_cleanup(void)
{
	/* Nothing for DM355 */
}

/*                                                       
 * ======== ccdc_reset  ========                        
 */
/*This function will reset all CCDc reg */

void ccdc_reset()
{
	int i, memctrl, clkctrl;
	/* disable CCDC */
	dev_dbg(vpfe_dev, "\nStarting ccdc_reset...");
	mdelay(5);
	ccdc_enable(0);
	/* set all registers to default value */
	for (i = 0; i <= 0x15c; i += 4) {
		mdelay(5);
		regw(0, i);
	}
	mdelay(5);
	/* no culling support */
	regw(0xffff, CULH);
	mdelay(5);
	regw(0xff, CULV);
	mdelay(5);
	/* always set the gain to 1 */
	regw(0x80, RYEGAIN);
	mdelay(5);
	regw(0x80, GRCYGAIN);
	mdelay(5);
	regw(0x80, GBGGAIN);
	mdelay(5);
	regw(0x80, BMGGAIN);
	mdelay(5);
	/* up to 12 bit sensor */
	regw(0x0FFF, OUTCLIP);
	mdelay(5);

	memctrl = regr_bl(MEMCTRL);
	memctrl &= 0xfffffffe;	/*configuring DFC for IPIPE */
	mdelay(5);
	regw_bl(memctrl, MEMCTRL);
	mdelay(5);
	regw_bl(0x00, CCDCMUX);	/*CCDC input Mux select directly from sensor */
	mdelay(5);
	clkctrl = regr_clk(CLKCTRL);
	clkctrl &= 0x3f;
	clkctrl |= 0x40;
	mdelay(5);
	regw_clk(clkctrl, CLKCTRL);

	dev_dbg(vpfe_dev, "\nEnd of ccdc_reset...");
}

/*                                                       
 * ======== ccdc_setwin  ========                        
 */
/*This function will configure the window size to be capture in CCDC reg */
void ccdc_setwin(ccdc_imgwin * image_win, ccdc_frmfmt frm_fmt, int ppc)
{
	int horz_start, horz_nr_pixels;
	int vert_start, vert_nr_lines;
	int mid_img = 0;
	dev_dbg(vpfe_dev, "\nStarting ccdc_setwin...");
	/* configure horizonal and vertical starts and sizes */
	horz_start = image_win->left << (ppc - 1);
	horz_nr_pixels = ((image_win->width) << (ppc - 1)) - 1;

	/*Writing the horizontal info into the registers */
	mdelay(5);
	regw(horz_start & START_PX_HOR_MASK, SPH);
	mdelay(5);
	regw(horz_nr_pixels & NUM_PX_HOR_MASK, NPH);
	vert_start = image_win->top;

	if (frm_fmt == CCDC_FRMFMT_INTERLACED) {
		vert_nr_lines = (image_win->height >> 1) - 1;
		vert_start >>= 1;
		/* configure VDINT0 and VDINT1 */
		regw(vert_start, VDINT0);
	} else {
		vert_nr_lines = image_win->height - 1;
		/* configure VDINT0 and VDINT1 */
		mid_img = vert_start + (image_win->height / 2);
		regw(vert_start, VDINT0);
		regw(mid_img, VDINT1);
	}
	mdelay(5);
	regw(vert_start & START_VER_ONE_MASK, SLV0);
	mdelay(5);
	regw(vert_start & START_VER_TWO_MASK, SLV1);
	mdelay(5);
	regw(vert_nr_lines & NUM_LINES_VER, NLV);
	dev_dbg(vpfe_dev, "\nEnd of ccdc_setwin...");
}

#if 0
void ccdc_config_stillcapture(void *arg)
{
	ccdc_params_raw *params;
	params = (ccdc_params_raw *) arg;
	ccdc_config_raw(params);

	/***/
	regw(0x63, HDWIDTH);
	regw(0x8, VDWIDTH);
	regw(0x11c5, PPLN);
	regw(0x947, LPFR);
	regw(2, LSCCFG2);
	/***/

	regw(0xFFFF, CULH);
	regw(0x159, HSIZE);
	regw(0x100, RYEGAIN);
	regw(0x100, GRCYGAIN);
	regw(0x100, GBGGAIN);
	regw(0x100, BMGGAIN);
	regw(0x64, VDINT0);
	regw(0x1e, VDINT1);
	regw(0x9, GAMMAWD);
	regw(0x20, FMTCFG);
	regw(0x0, CCDCFG);

}
#endif

/* following function is just for the reference, check the registers value , 
not used any where */
void ccdc_config_stillcapture1(void *arg)
{
	__REG(0x01C70600) = 0x00000000;
	__REG(0x01C70604) = 0x00000800;
	__REG(0x01C70608) = 0x00000063;
	__REG(0x01C7060c) = 0x00000008;
	__REG(0x01C70610) = 0x000011c5;
	__REG(0x01C70614) = 0x00000947;
	__REG(0x01C70618) = 0x0000037a;
	__REG(0x01C7061c) = 0x00000e5f;
	__REG(0x01C70620) = 0x00000037;
	__REG(0x01C70624) = 0x00000037;
	__REG(0x01C70628) = 0x00000395;
	__REG(0x01C7062c) = 0x0000ffff;
	__REG(0x01C70630) = 0x000000FF;
	__REG(0x01C70634) = 0x00000159;
	__REG(0x01C70638) = 0x00000000;
	__REG(0x01C70644) = 0x00000000;
	__REG(0x01C70648) = 0x00000000;
	__REG(0x01C7064c) = 0x00004E4E;
	__REG(0x01C70650) = 0x00000000;
	__REG(0x01C70654) = 0x00000000;
	__REG(0x01C70658) = 0x00000000;
	__REG(0x01C7065c) = 0x00000100;
	__REG(0x01C70660) = 0x00000100;
	__REG(0x01C70664) = 0x00000100;
	__REG(0x01C70668) = 0x00000100;
	__REG(0x01C7066c) = 0x00000000;
	__REG(0x01C70670) = 0x00003FFF;
	__REG(0x01C70674) = 0x00000064;
	__REG(0x01C70678) = 0x0000001e;
	__REG(0x01C7067c) = 0x00000000;
	__REG(0x01C70680) = 0x00000009;
	__REG(0x01C70684) = 0x00000000;
	__REG(0x01C70688) = 0x00000000;
	__REG(0x01C7068c) = 0x00000020;
	__REG(0x01C70690) = 0x00007722;
	__REG(0x01C70694) = 0x0000036e;
	__REG(0x01C70698) = 0x00000e57;
	__REG(0x01C7069c) = 0x00000100;
	__REG(0x01C706a0) = 0x00000395;
	__REG(0x01C706a4) = 0x000004c7;
	__REG(0x01C706a8) = 0x000004d2;
	__REG(0x01C706ac) = 0x00000000;
	__REG(0x01C706b0) = 0x00002000;
	__REG(0x01C706b4) = 0x00004000;
	__REG(0x01C706b8) = 0x00000000;
	__REG(0x01C706bc) = 0x00000000;
	__REG(0x01C706c0) = 0x00000000;
	__REG(0x01C706c4) = 0x00000000;
	__REG(0x01C706c8) = 0x00000000;
	__REG(0x01C706cc) = 0x00000007;
	__REG(0x01C706d0) = 0x00000007;
	__REG(0x01C706d4) = 0x00000210;
	__REG(0x01C706d8) = 0x00000000;
	__REG(0x01C706dc) = 0x00000000;
	__REG(0x01C706e0) = 0x00000000;
	__REG(0x01C706e4) = 0x00000210;
	__REG(0x01C706e8) = 0x00000000;
	__REG(0x01C706ec) = 0x00000000;
	__REG(0x01C706f0) = 0x00000000;
	__REG(0x01C706f4) = 0x00000000;
	__REG(0x01C706f8) = 0x00000002;
	__REG(0x01C706fc) = 0x00000000;
	__REG(0x01C70700) = 0x00000000;
	__REG(0x01C70704) = 0x00000000;
	__REG(0x01C70708) = 0x00000000;

}

/*This function will configure CCDC for YCbCr parameters*/
void ccdc_config_ycbcr()
{
	u32 modeset;
	ccdc_params_ycbcr *params = &ccdc_hw_params_ycbcr;

	/* first reset the CCDC                                          */
	/* all registers have default values after reset                 */
	/* This is important since we assume default values to be set in */
	/* a lot of registers that we didn't touch                       */
	dev_dbg(vpfe_dev, "\nStarting ccdc_config_ycbcr...");
	mdelay(5);
	ccdc_reset();

	/* configure pixel format */
	modeset = (params->pix_fmt & 0x3) << 12;

	/* configure video frame format */
	modeset |= (params->frm_fmt & 0x1) << 7;

	/* setup BT.656 sync mode */
	if (params->bt656_enable) {
		regw(3, REC656IF);
		mdelay(5);
		/* configure the FID, VD, HD pin polarity */
		/* fld,hd pol positive, vd negative, 8-bit pack mode */
		modeset |= 0x04;
	} else {		/* y/c external sync mode */
		modeset |= ((params->fid_pol & 0x1) << 4);
		modeset |= ((params->hd_pol & 0x1) << 3);
		modeset |= ((params->vd_pol & 0x1) << 2);
	}

	/* pack the data to 8-bit */
	modeset |= 0x1 << 11;

	regw(modeset, MODESET);
	mdelay(5);

	/* configure video window */
	ccdc_setwin(&params->win, params->frm_fmt, 2);
	mdelay(5);
	/* configure the order of y cb cr in SD-RAM */
	regw((params->pix_order << 11) | 0x8040, CCDCFG);
	mdelay(5);

	/* configure the horizontal line offset */
	/* this is done by rounding up width to a multiple of 16 pixels */
	/* and multiply by two to account for y:cb:cr 4:2:2 data */
	regw(((((params->win.width * 2) + 31) & 0xffffffe0) >> 5), HSIZE);
	mdelay(5);

	/* configure the memory line offset */
	if (params->buf_type == CCDC_BUFTYPE_FLD_INTERLEAVED) {
		/* two fields are interleaved in memory */
		regw(0x00000249, SDOFST);
		mdelay(5);
	}
	/*val = (unsigned int)ccdc_sbl_reset();
	   dev_dbg(vpfe_dev, "\nReading 0x%x from SBL...\n", val); */

	dev_dbg(vpfe_dev, "\nEnd of ccdc_config_ycbcr...\n");
	/*ccdc_readregs(); */
}

/*                                                        
 * ======== ccdc_config_raw  ========                   
 */
/*This function will configure CCDC for Raw mode parameters*/
void ccdc_config_raw()
{
	ccdc_params_raw *params = &ccdc_hw_params_raw;
	unsigned int mode_set = 0;
	unsigned int val = 0, val1 = 0;
	int temp1 = 0, temp2 = 0, i = 0, fmtreg_v = 0, shift_v = 0, flag = 0;
	int temp_gf = 0, temp_lcs = 0;
	dev_dbg(vpfe_dev, "\nStarting ccdc_config_raw...");
	/*      Reset CCDC */
	mdelay(5);
	ccdc_reset();

	/*
	 *      C O N F I G U R I N G  T H E  C C D C F G  R E G I S T E R
	 */

	/*Set CCD Not to swap input since input is RAW data */
	val |= CCDC_YCINSWP_RAW;

	/*set FID detection function to Latch at V-Sync */
	val |= CCDC_CCDCFG_FIDMD_LATCH_VSYNC << CCDC_CCDCFG_FIDMD_SHIFT;

	/*set WENLOG - ccdc valid area */
	val |= CCDC_CCDCFG_WENLOG_AND << CCDC_CCDCFG_WENLOG_SHIFT;

	/*set TRGSEL */
	val |= CCDC_CCDCFG_TRGSEL_WEN << CCDC_CCDCFG_TRGSEL_SHIFT;

	/*set EXTRG */
	val |= CCDC_CCDCFG_EXTRG_DISABLE << CCDC_CCDCFG_EXTRG_SHIFT;

	/* Disable latching function registers on VSYNC-busy writable
	   registers  */
	/*regw(CCDC_LATCH_ON_VSYNC_DISABLE, CCDCFG); */

	/* Enable latching function registers on VSYNC-shadowed registers */
	val |= CCDC_LATCH_ON_VSYNC_DISABLE;
	mdelay(5);
	regw(val, CCDCFG);
	/*
	 *      C O N F I G U R I N G  T H E  M O D E S E T  R E G I S T E R
	 */

	/*Set VDHD direction to input */
	mode_set |=
	    (CCDC_VDHDOUT_INPUT & CCDC_VDHDOUT_MASK) << CCDC_VDHDOUT_SHIFT;

	/*Set input type to raw input */
	mode_set |=
	    (CCDC_RAW_IP_MODE & CCDC_RAW_INPUT_MASK) << CCDC_RAW_INPUT_SHIFT;

	/*      Configure the vertical sync polarity(MODESET.VDPOL) */
	mode_set = (params->vd_pol & CCDC_VD_POL_MASK) << CCDC_VD_POL_SHIFT;

	/*      Configure the horizontal sync polarity (MODESET.HDPOL) */
	mode_set |= (params->hd_pol & CCDC_HD_POL_MASK) << CCDC_HD_POL_SHIFT;

	/*      Configure frame id polarity (MODESET.FLDPOL) */
	mode_set |= (params->fid_pol & CCDC_FID_POL_MASK) << CCDC_FID_POL_SHIFT;

	/*      Configure data polarity */
	mode_set |=
	    (CCDC_DATAPOL_NORMAL & CCDC_DATAPOL_MASK) << CCDC_DATAPOL_SHIFT;

	/*      Configure External WEN Selection */
	mode_set |= (CCDC_EXWEN_DISABLE & CCDC_EXWEN_MASK) << CCDC_EXWEN_SHIFT;

	/* Configure frame format(progressive or interlace) */
	mode_set |= (params->frm_fmt & CCDC_FRM_FMT_MASK) << CCDC_FRM_FMT_SHIFT;

	/* Configure pixel format (Input mode) */
	mode_set |= (params->pix_fmt & CCDC_PIX_FMT_MASK) << CCDC_PIX_FMT_SHIFT;

	if ((params->data_sz == _8BITS) || params->alaw.b_alaw_enable) {
		mode_set |= CCDC_DATA_PACK_ENABLE;
	}

	/*Configure for LPF */
	if (params->lpf_enable) {
		mode_set |=
		    (params->lpf_enable & CCDC_LPF_MASK) << CCDC_LPF_SHIFT;
	}

	/*Configure the data shift */
	mode_set |= (params->datasft & CCDC_DATASFT_MASK) << CCDC_DATASFT_SHIFT;
	mdelay(5);
	regw(mode_set, MODESET);
	dev_dbg(vpfe_dev, "\nWriting 0x%x to MODESET...\n", mode_set);

	/*Configure the Median Filter threshold */
	mdelay(5);
	regw((params->med_filt_thres) & 0x3fff, MEDFILT);

	/*
	 *      C O N F I G U R E   T H E   G A M M A W D   R E G I S T E R
	 */

	val = 8;
	val |=
	    (CCDC_CFA_MOSAIC & CCDC_GAMMAWD_CFA_MASK) << CCDC_GAMMAWD_CFA_SHIFT;

	/* Enable and configure aLaw register if needed */
	if (params->alaw.b_alaw_enable) {
		val |= (params->alaw.gama_wd & CCDC_ALAW_GAMA_WD_MASK) << 2;
		val |= CCDC_ALAW_ENABLE;	/*set enable bit of alaw */
	}

	/*Configure Median filter1 for IPIPE capture */
	val |= params->mfilt1 << CCDC_MFILT1_SHIFT;

	/*Configure Median filter2 for SDRAM capture */
	val |= params->mfilt2 << CCDC_MFILT2_SHIFT;

	mdelay(5);
	regw(val, GAMMAWD);
	dev_dbg(vpfe_dev, "\nWriting 0x%x to GAMMAWD...\n", val);

	/* configure video window */
	mdelay(5);
	ccdc_setwin(&params->win, params->frm_fmt, 1);

	/*
	 *      O P T I C A L   B L A C K   A V E R A G I N G
	 */
	val = 0;
	if (params->blk_clamp.b_clamp_enable) {
		val |= (params->blk_clamp.start_pixel & CCDC_BLK_ST_PXL_MASK);

		val1 |= (params->blk_clamp.sample_ln & CCDC_NUM_LINE_CALC_MASK)
		    << CCDC_NUM_LINE_CALC_SHIFT;	/*No of line
							   to be avg */
		val |=
		    (params->blk_clamp.sample_pixel & CCDC_BLK_SAMPLE_LN_MASK)
		    << CCDC_BLK_SAMPLE_LN_SHIFT;	/*No of pixel/line to be avg */
		val |= CCDC_BLK_CLAMP_ENABLE;	/*Enable the Black clamping */
		mdelay(5);
		regw(val, CLAMP);

		dev_dbg(vpfe_dev, "\nWriting 0x%x to CLAMP...\n", val);
		mdelay(5);
		regw(val1, DCSUB);	/*If Black clamping is enable
					   then make dcsub 0 */
		dev_dbg(vpfe_dev, "\nWriting 0x00000000 to DCSUB...\n");

	} else {
		/* configure DCSub */
		val = (params->blk_clamp.dc_sub) & CCDC_BLK_DC_SUB_MASK;
		mdelay(5);
		regw(val, DCSUB);

		dev_dbg(vpfe_dev, "\nWriting 0x%x to DCSUB...\n", val);
		mdelay(5);
		regw(0x0000, CLAMP);

		dev_dbg(vpfe_dev, "\nWriting 0x0000 to CLAMP...\n");
	}

	/*
	 *  C O N F I G U R E   B L A C K   L E V E L   C O M P E N S A T I O N
	 */
	val = 0;
	val = (params->blk_comp.b_comp & CCDC_BLK_COMP_MASK);
	val |= (params->blk_comp.gb_comp & CCDC_BLK_COMP_MASK)
	    << CCDC_BLK_COMP_GB_COMP_SHIFT;
	mdelay(5);
	regw(val, BLKCMP1);

	val1 = 0;
	val1 |= (params->blk_comp.gr_comp & CCDC_BLK_COMP_MASK)
	    << CCDC_BLK_COMP_GR_COMP_SHIFT;
	val1 |= (params->blk_comp.r_comp & CCDC_BLK_COMP_MASK)
	    << CCDC_BLK_COMP_R_COMP_SHIFT;
	mdelay(5);
	regw(val1, BLKCMP0);

	dev_dbg(vpfe_dev, "\nWriting 0x%x to BLKCMP1...\n", val);
	dev_dbg(vpfe_dev, "\nWriting 0x%x to BLKCMP0...\n", val1);

	/* Configure Vertical Defect Correction if needed */
	if (params->vertical_dft.ver_dft_en) {

		shift_v = 0;
		shift_v = 0 << CCDC_DFCCTL_VDFCEN_SHIFT;
		shift_v |=
		    params->vertical_dft.gen_dft_en & CCDC_DFCCTL_GDFCEN_MASK;
		shift_v |=
		    (params->vertical_dft.dft_corr_ctl.
		     vdfcsl & CCDC_DFCCTL_VDFCSL_MASK) <<
		    CCDC_DFCCTL_VDFCSL_SHIFT;
		shift_v |=
		    (params->vertical_dft.dft_corr_ctl.
		     vdfcuda & CCDC_DFCCTL_VDFCUDA_MASK) <<
		    CCDC_DFCCTL_VDFCUDA_SHIFT;
		shift_v |=
		    (params->vertical_dft.dft_corr_ctl.
		     vdflsft & CCDC_DFCCTL_VDFLSFT_MASK) <<
		    CCDC_DFCCTL_VDFLSFT_SHIFT;
		mdelay(5);
		regw(shift_v, DFCCTL);
		mdelay(5);
		regw(params->vertical_dft.dft_corr_vert[0], DFCMEM0);
		mdelay(5);
		regw(params->vertical_dft.dft_corr_horz[0], DFCMEM1);
		mdelay(5);
		regw(params->vertical_dft.dft_corr_sub1[0], DFCMEM2);
		mdelay(5);
		regw(params->vertical_dft.dft_corr_sub2[0], DFCMEM3);
		mdelay(5);
		regw(params->vertical_dft.dft_corr_sub3[0], DFCMEM4);

		shift_v = 0;
		shift_v = regr(DFCMEMCTL);
		shift_v |= 1 << CCDC_DFCMEMCTL_DFCMARST_SHIFT;
		shift_v |= 1;
		mdelay(5);
		regw(shift_v, DFCMEMCTL);

		while (1) {
			flag = regr(DFCMEMCTL);
			if ((flag & 0x01) == 0x00)
				break;
		}
		flag = 0;
		shift_v = 0;
		shift_v = regr(DFCMEMCTL);
		shift_v |= 0 << CCDC_DFCMEMCTL_DFCMARST_SHIFT;
		mdelay(5);
		regw(shift_v, DFCMEMCTL);

		for (i = 1; i < 16; i++) {
			mdelay(5);
			regw(params->vertical_dft.dft_corr_vert[i], DFCMEM0);
			mdelay(5);
			regw(params->vertical_dft.dft_corr_horz[i], DFCMEM1);
			mdelay(5);
			regw(params->vertical_dft.dft_corr_sub1[i], DFCMEM2);
			mdelay(5);
			regw(params->vertical_dft.dft_corr_sub2[i], DFCMEM3);
			mdelay(5);
			regw(params->vertical_dft.dft_corr_sub3[i], DFCMEM4);

			shift_v = 0;
			shift_v = regr(DFCMEMCTL);
			shift_v |= 1;
			mdelay(5);
			regw(shift_v, DFCMEMCTL);

			while (1) {
				mdelay(5);
				flag = regr(DFCMEMCTL);
				if ((flag & 0x01) == 0x00)
					break;
			}
			flag = 0;
		}
		mdelay(5);
		regw(params->vertical_dft.
		     saturation_ctl & CCDC_VDC_DFCVSAT_MASK, DFCVSAT);

		shift_v = 0;
		shift_v = regr(DFCCTL);
		shift_v |= 1 << CCDC_DFCCTL_VDFCEN_SHIFT;
		mdelay(5);
		regw(shift_v, DFCCTL);
	}

	/* Configure Lens Shading Correction if needed */
	if (params->lens_sh_corr.lsc_enable) {
		dev_dbg(vpfe_dev, "\nlens shading Correction entered....\n");

		/*first disable the LSC */
		mdelay(5);
		regw(CCDC_LSC_DISABLE, LSCCFG1);

		/*UPDATE PROCEDURE FOR GAIN FACTOR TABLE 1 */

		/*select table 1 */
		mdelay(5);
		regw(CCDC_LSC_TABLE1_SLC, LSCMEMCTL);

		/*Reset memory address */
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_MEMADDR_RESET;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		/*Update gainfactor for table 1 - u8q8 */
		temp_gf =
		    ((int)(params->lens_sh_corr.gf_table1[0].frac_no * 256))
		    & CCDC_LSC_FRAC_MASK_T1;
		temp_gf |=
		    (((int)(params->lens_sh_corr.gf_table1[0].frac_no * 256))
		     & CCDC_LSC_FRAC_MASK_T1) << 8;
		mdelay(5);
		regw(temp_gf, LSCMEMD);

		while (1) {
			if ((regr(LSCMEMCTL) & 0x10) == 0)
				break;
		}

		/*set the address to incremental mode */
		temp_lcs = 0;
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_MEMADDR_INCR;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		for (i = 2; i < 255; i += 2) {
			temp_gf = 0;
			temp_gf = ((int)
				   (params->lens_sh_corr.gf_table1[0].frac_no *
				    256))
			    & CCDC_LSC_FRAC_MASK_T1;
			temp_gf |= (((int)
				     (params->lens_sh_corr.gf_table1[0].
				      frac_no * 256))
				    & CCDC_LSC_FRAC_MASK_T1) << 8;
			mdelay(5);
			regw(temp_gf, LSCMEMD);

			while (1) {
				if ((regr(LSCMEMCTL) & 0x10) == 0)
					break;
			}
		}

		/*UPDATE PROCEDURE FOR GAIN FACTOR TABLE 2 */

		/*select table 2 */
		temp_lcs = 0;
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_TABLE2_SLC;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		/*Reset memory address */
		temp_lcs = 0;
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_MEMADDR_RESET;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		/*Update gainfactor for table 2 - u16q14 */
		temp_gf =
		    (params->lens_sh_corr.gf_table2[0].
		     int_no & CCDC_LSC_INT_MASK) << 14;
		temp_gf |=
		    ((int)(params->lens_sh_corr.gf_table2[0].frac_no) * 16384)
		    & CCDC_LSC_FRAC_MASK;
		mdelay(5);
		regw(temp_gf, LSCMEMD);

		while (1) {
			if ((regr(LSCMEMCTL) & 0x10) == 0)
				break;
		}

		/*set the address to incremental mode */
		temp_lcs = 0;
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_MEMADDR_INCR;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		for (i = 1; i < 128; i++) {
			temp_gf = 0;
			temp_gf =
			    (params->lens_sh_corr.gf_table2[i].
			     int_no & CCDC_LSC_INT_MASK) << 14;
			temp_gf |=
			    ((int)(params->lens_sh_corr.gf_table2[0].frac_no) *
			     16384)
			    & CCDC_LSC_FRAC_MASK;
			mdelay(5);
			regw(temp_gf, LSCMEMD);

			while (1) {
				mdelay(5);
				if ((regr(LSCMEMCTL) & 0x10) == 0)
					break;
			}
		}

		/*UPDATE PROCEDURE FOR GAIN FACTOR TABLE 3 */

		/*select table 3 */
		temp_lcs = 0;
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_TABLE3_SLC;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		/*Reset memory address */
		temp_lcs = 0;
		temp_lcs = regr(LSCMEMCTL);
		temp_lcs |= CCDC_LSC_MEMADDR_RESET;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		/*Update gainfactor for table 2 - u16q14 */
		temp_gf =
		    (params->lens_sh_corr.gf_table3[0].
		     int_no & CCDC_LSC_INT_MASK) << 14;
		temp_gf |=
		    ((int)(params->lens_sh_corr.gf_table3[0].frac_no) * 16384)
		    & CCDC_LSC_FRAC_MASK;
		mdelay(5);
		regw(temp_gf, LSCMEMD);

		while (1) {
			mdelay(5);
			if ((regr(LSCMEMCTL) & 0x10) == 0)
				break;
		}

		/*set the address to incremental mode */
		temp_lcs = 0;
		mdelay(5);
		temp_lcs = regr(LSCMEMCTL);
		mdelay(5);
		temp_lcs |= CCDC_LSC_MEMADDR_INCR;
		mdelay(5);
		regw(temp_lcs, LSCMEMCTL);

		for (i = 1; i < 128; i++) {
			temp_gf = 0;
			temp_gf =
			    (params->lens_sh_corr.gf_table3[i].
			     int_no & CCDC_LSC_INT_MASK) << 14;
			temp_gf |=
			    ((int)(params->lens_sh_corr.gf_table3[0].frac_no) *
			     16384)
			    & CCDC_LSC_FRAC_MASK;
			mdelay(5);
			regw(temp_gf, LSCMEMD);

			while (1) {
				mdelay(5);
				if ((regr(LSCMEMCTL) & 0x10) == 0)
					break;
			}
		}
		/*Configuring the optical centre of the lens */
		mdelay(5);
		regw(params->lens_sh_corr.
		     lens_center_horz & CCDC_LSC_CENTRE_MASK, LSCH0);
		mdelay(5);
		regw(params->lens_sh_corr.
		     lens_center_vert & CCDC_LSC_CENTRE_MASK, LSCV0);

		val = 0;
		val =
		    ((int)(params->lens_sh_corr.horz_left_coef.frac_no * 128)) &
		    0x7f;
		val |= (params->lens_sh_corr.horz_left_coef.int_no & 0x01) << 7;
		val |=
		    (((int)(params->lens_sh_corr.horz_right_coef.frac_no * 128))
		     & 0x7f) << 8;
		val |=
		    (params->lens_sh_corr.horz_right_coef.int_no & 0x01) << 15;
		mdelay(5);
		regw(val, LSCKH);

		val = 0;
		val =
		    ((int)(params->lens_sh_corr.ver_up_coef.frac_no * 128)) &
		    0x7f;
		val |= (params->lens_sh_corr.ver_up_coef.int_no & 0x01) << 7;
		val |=
		    (((int)(params->lens_sh_corr.ver_low_coef.frac_no * 128)) &
		     0x7f) << 8;
		val |= (params->lens_sh_corr.ver_low_coef.int_no & 0x01) << 15;
		mdelay(5);
		regw(val, LSCKV);

		/*configuring the lsc configuration register 2 */
		temp_lcs = 0;
		temp_lcs |=
		    (params->lens_sh_corr.lsc_config.
		     gf_table_scaling_fact & CCDC_LSCCFG_GFTSF_MASK) <<
		    CCDC_LSCCFG_GFTSF_SHIFT;
		temp_lcs |=
		    (params->lens_sh_corr.lsc_config.
		     gf_table_interval & CCDC_LSCCFG_GFTINV_MASK) <<
		    CCDC_LSCCFG_GFTINV_SHIFT;
		temp_lcs |=
		    (params->lens_sh_corr.lsc_config.
		     epel & CCDC_LSC_GFTABLE_SEL_MASK) <<
		    CCDC_LSC_GFTABLE_EPEL_SHIFT;
		temp_lcs |=
		    (params->lens_sh_corr.lsc_config.
		     opel & CCDC_LSC_GFTABLE_SEL_MASK) <<
		    CCDC_LSC_GFTABLE_OPEL_SHIFT;
		temp_lcs |=
		    (params->lens_sh_corr.lsc_config.
		     epol & CCDC_LSC_GFTABLE_SEL_MASK) <<
		    CCDC_LSC_GFTABLE_EPOL_SHIFT;
		temp_lcs |=
		    (params->lens_sh_corr.lsc_config.
		     opol & CCDC_LSC_GFTABLE_SEL_MASK) <<
		    CCDC_LSC_GFTABLE_OPOL_SHIFT;
		mdelay(5);
		regw(temp_lcs, LSCCFG2);

		/*configuring the LSC configuration register 1 */
		temp_lcs = 0;
		temp_lcs |= CCDC_LSC_ENABLE;
		temp_lcs |= (params->lens_sh_corr.lsc_config.gfmode &
			     CCDC_LSC_GFMODE_MASK) << CCDC_LSC_GFMODE_SHIFT;
		mdelay(5);
		regw(temp_lcs, LSCCFG1);
	}

	/* Configure data formatter if needed */
	if (params->data_formatter_r.fmt_enable
	    && (!params->color_space_con.csc_enable)) {
		dev_dbg(vpfe_dev,
			"\ndata formatter will be configured now....\n");

		/*Configuring the FMTPLEN */
		fmtreg_v = 0;
		fmtreg_v |=
		    (params->data_formatter_r.fmtplen.
		     plen0 & CCDC_FMTPLEN_P0_MASK);
		fmtreg_v |=
		    ((params->data_formatter_r.fmtplen.
		      plen1 & CCDC_FMTPLEN_P1_MASK)
		     << CCDC_FMTPLEN_P1_SHIFT);
		fmtreg_v |=
		    ((params->data_formatter_r.fmtplen.
		      plen2 & CCDC_FMTPLEN_P2_MASK)
		     << CCDC_FMTPLEN_P2_SHIFT);
		fmtreg_v |=
		    ((params->data_formatter_r.fmtplen.
		      plen3 & CCDC_FMTPLEN_P3_MASK)
		     << CCDC_FMTPLEN_P3_SHIFT);
		mdelay(5);
		regw(fmtreg_v, FMTPLEN);

		/*Configurring the FMTSPH */
		mdelay(5);
		regw((params->data_formatter_r.fmtsph & CCDC_FMTSPH_MASK),
		     FMTSPH);

		/*Configurring the FMTLNH */
		mdelay(5);
		regw((params->data_formatter_r.fmtlnh & CCDC_FMTLNH_MASK),
		     FMTLNH);

		/*Configurring the FMTSLV */
		mdelay(5);
		regw((params->data_formatter_r.fmtslv & CCDC_FMTSLV_MASK),
		     FMTSLV);

		/*Configurring the FMTLNV */
		mdelay(5);
		regw((params->data_formatter_r.fmtlnv & CCDC_FMTLNV_MASK),
		     FMTLNV);

		/*Configurring the FMTRLEN */
		mdelay(5);
		regw((params->data_formatter_r.fmtrlen & CCDC_FMTRLEN_MASK),
		     FMTRLEN);

		/*Configurring the FMTHCNT */
		mdelay(5);
		regw((params->data_formatter_r.fmthcnt & CCDC_FMTHCNT_MASK),
		     FMTHCNT);

		/*Configuring the FMTADDR_PTR */
		for (i = 0; i < 8; i++) {
			fmtreg_v = 0;

			if (params->data_formatter_r.fmtaddr_ptr[i].init >
			    (params->data_formatter_r.fmtrlen - 1)) {
				dev_dbg(vpfe_dev,
					"\nInvalid init parameter for FMTADDR_PTR....\n");
				return;
			}

			fmtreg_v =
			    (params->data_formatter_r.fmtaddr_ptr[i].
			     init & CCDC_ADP_INIT_MASK);
			fmtreg_v |=
			    ((params->data_formatter_r.fmtaddr_ptr[i].
			      line & CCDC_ADP_LINE_MASK) <<
			     CCDC_ADP_LINE_SHIFT);
			mdelay(5);
			regw(fmtreg_v, FMT_ADDR_PTR(i));
		}

		/*Configuring the FMTPGM_VF0 */
		fmtreg_v = 0;
		for (i = 0; i < 16; i++) {
			fmtreg_v |= params->data_formatter_r.pgm_en[i] << i;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_VF0);

		/*Configuring the FMTPGM_VF1 */
		fmtreg_v = 0;
		for (i = 16; i < 32; i++) {
			fmtreg_v |=
			    params->data_formatter_r.pgm_en[i] << (i - 16);
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_VF1);

		/*Configuring the FMTPGM_AP0 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 0; i < 4; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP0);

		/*Configuring the FMTPGM_AP1 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 4; i < 8; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP1);

		/*Configuring the FMTPGM_AP2 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 8; i < 12; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP2);

		/*Configuring the FMTPGM_AP3 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 12; i < 16; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP3);

		/*Configuring the FMTPGM_AP4 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 16; i < 20; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP4);

		/*Configuring the FMTPGM_AP5 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 20; i < 24; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP5);

		/*Configuring the FMTPGM_AP6 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 24; i < 28; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP6);

		/*Configuring the FMTPGM_AP7 */
		fmtreg_v = 0;
		shift_v = 0;
		for (i = 28; i < 32; i++) {
			fmtreg_v |=
			    ((params->data_formatter_r.fmtpgm_ap[i].
			      pgm_aptr & CCDC_FMTPGN_APTR_MASK) << shift_v);
			fmtreg_v |=
			    (params->data_formatter_r.fmtpgm_ap[i].
			     pgmupdt << (shift_v + 3));
			shift_v += 4;
		}
		mdelay(5);
		regw(fmtreg_v, FMTPGM_AP7);

		/*Configuring the FMTCFG register */
		fmtreg_v = 0;
		fmtreg_v = CCDC_DF_ENABLE;
		fmtreg_v |=
		    ((params->data_formatter_r.fmtcfg.
		      fmtmode & CCDC_FMTCFG_FMTMODE_MASK)
		     << CCDC_FMTCFG_FMTMODE_SHIFT);
		fmtreg_v |=
		    ((params->data_formatter_r.fmtcfg.
		      lnum & CCDC_FMTCFG_LNUM_MASK)
		     << CCDC_FMTCFG_LNUM_SHIFT);
		fmtreg_v |=
		    ((params->data_formatter_r.fmtcfg.
		      addrinc & CCDC_FMTCFG_ADDRINC_MASK)
		     << CCDC_FMTCFG_ADDRINC_SHIFT);
		mdelay(5);
		regw(fmtreg_v, FMTCFG);

	} else if (params->data_formatter_r.fmt_enable) {
		dev_dbg(vpfe_dev,
			"\nCSC and Data Formatter Enabled at same time....\n");
	}

	/*
	 *      C O N F I G U R E   C O L O R   S P A C E   C O N V E R T E R
	 */

	if ((params->color_space_con.csc_enable)
	    && (!params->data_formatter_r.fmt_enable)) {
		dev_dbg(vpfe_dev, "\nconfiguring the CSC Now....\n");

		/*Enable the CSC sub-module */
		mdelay(5);
		regw(CCDC_CSC_ENABLE, CSCCTL);

		/*Converting the co-eff as per the format of the register */
		for (i = 0; i < 16; i++) {
			temp1 = params->color_space_con.csc_dec_coeff[i];
			/*Masking the data for 3 bits */
			temp1 &= CCDC_CSC_COEFF_DEC_MASK;
			/*Recovering the fractional part and converting to
			   binary of 5 bits */
			temp2 =
			    (int)(params->color_space_con.csc_frac_coeff[i] *
				  (32 / 10));
			temp2 &= CCDC_CSC_COEFF_FRAC_MASK;
			/*shifting the decimal to the MSB */
			temp1 = temp1 << CCDC_CSC_DEC_SHIFT;
			temp1 |= temp2;	/*appending the fraction at LSB */
			params->color_space_con.csc_dec_coeff[i] = temp1;
		}
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[0], CSCM0);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[1] << CCDC_CSC_COEFF_SHIFT, CSCM0);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[2], CSCM1);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[3] << CCDC_CSC_COEFF_SHIFT, CSCM1);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[4], CSCM2);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[5] << CCDC_CSC_COEFF_SHIFT, CSCM2);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[6], CSCM3);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[7] << CCDC_CSC_COEFF_SHIFT, CSCM3);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[8], CSCM4);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[9] << CCDC_CSC_COEFF_SHIFT, CSCM4);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[10], CSCM5);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[11] << CCDC_CSC_COEFF_SHIFT, CSCM5);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[12], CSCM6);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[13] << CCDC_CSC_COEFF_SHIFT, CSCM6);
		mdelay(5);
		regw(params->color_space_con.csc_dec_coeff[14], CSCM7);
		mdelay(5);
		regw(params->color_space_con.
		     csc_dec_coeff[15] << CCDC_CSC_COEFF_SHIFT, CSCM7);

	} else if (params->color_space_con.csc_enable) {
		dev_err(vpfe_dev,
			"\nCSC and Data Formatter Enabled at same time....\n");
	}

	/*Configure the offset control */
	if (params->ccdc_offset) {
		if (params->ccdc_offset < 1024) {
			mdelay(5);
			regw(params->ccdc_offset & CCDC_OFFSET_MASK, OFFSET);
		} else
			dev_err(vpfe_dev, "\ninvalid offset value....\n");
	}

	/*
	 *      C O N F I G U R E  C O L O R  P A T T E R N  A S
	 *      P E R  N N 1 2 8 6 A  S E N S O R
	 */
	mdelay(5);
	regw(0x4E4E, COLPTN);

	dev_dbg(vpfe_dev, "\nWriting 0x4e4e to COLPTN...\n");

	/*
	 *      C O N F I G U R I N G  T H E  H S I Z E  R E G I S T E R
	 */
	val = 0;
	val |=
	    (params->data_offset_s.
	     horz_offset & CCDC_DATAOFST_MASK) << CCDC_DATAOFST_H_SHIFT;
	val |=
	    (params->data_offset_s.
	     vert_offset & CCDC_DATAOFST_MASK) << CCDC_DATAOFST_V_SHIFT;
	mdelay(5);
	regw(val, DATAOFST);

	/*
	 *      C O N F I G U R I N G  T H E  H S I Z E  R E G I S T E R
	 */
	val = 0;
	val |=
	    (params->
	     horz_flip_enable & CCDC_HSIZE_FLIP_MASK) << CCDC_HSIZE_FLIP_SHIFT;

	/* If pack 8 is enable then 1 pixel will take 1 byte */
	if ((params->data_sz == _8BITS) || params->alaw.b_alaw_enable) {
		val |= (((params->win.width) + 31) >> 5) & 0x0fff;

		dev_dbg(vpfe_dev, "\nWriting 0x%x to HSIZE...\n",
			(((params->win.width) + 31) >> 5) & 0x0fff);
	} else {		/* else one pixel will take 2 byte */

		val |= (((params->win.width * 2) + 31) >> 5) & 0x0fff;

		dev_dbg(vpfe_dev, "\nWriting 0x%x to HSIZE...\n",
			(((params->win.width * 2) + 31) >> 5) & 0x0fff);
	}
	mdelay(5);
	regw(val, HSIZE);

	/*
	 *      C O N F I G U R E   S D O F S T  R E G I S T E R
	 */

	if (params->frm_fmt == CCDC_FRMFMT_INTERLACED) {
		if (params->image_invert_enable) {
			/* For interlace inverse mode */
			mdelay(5);
			regw(0x4B6D, SDOFST);
			dev_dbg(vpfe_dev, "\nWriting 0x4B6D to SDOFST...\n");
		}

		else {
			/* For interlace non inverse mode */
			regw(0x0B6D, SDOFST);
			mdelay(5);
			dev_dbg(vpfe_dev, "\nWriting 0x0B6D to SDOFST...\n");
		}
	} else if (params->frm_fmt == CCDC_FRMFMT_PROGRESSIVE) {
		if (params->image_invert_enable) {
			/* For progessive inverse mode */
			mdelay(5);
			regw(0x4000, SDOFST);
			dev_dbg(vpfe_dev, "\nWriting 0x4000 to SDOFST...\n");
		}

		else {
			/* For progessive non inverse mode */
			mdelay(5);
			regw(0x0000, SDOFST);
			dev_dbg(vpfe_dev, "\nWriting 0x0000 to SDOFST...\n");
		}

	}

	/*
	 *      C O N F I G U R E   I N T E R R U P T   R E G I S T E R S
	 */
	if (params->frm_fmt == CCDC_FRMFMT_PROGRESSIVE) {
		val = params->win.height / 2;
		mdelay(5);
		regw(136, VDINT0);
		regw(149, VDINT0);
		mdelay(5);
		regw(0, VDINT1);
	} else {
		mdelay(5);
		regw(0, VDINT0);
		mdelay(5);
		regw(0, VDINT1);
	}

	dev_dbg(vpfe_dev, "\nend of ccdc_config_raw...");
	/*ccdc_readregs(); */
}

int validate_ccdc_param(ccdc_config_params_raw * ccdcparam)
{
	if (ccdcparam->pix_fmt != 0) {
		dev_err(vpfe_dev,
			"Invalid value of pix_fmt, other than RAW data input is \
 not supported\n");
		return -1;
	}

	if (ccdcparam->frm_fmt != 0) {
		dev_err(vpfe_dev,
			"Other than Progressive fram format is not supported\n");
		return -1;
	}

	if (ccdcparam->fid_pol != CCDC_PINPOL_POSITIVE
	    && ccdcparam->fid_pol != CCDC_PINPOL_NEGATIVE) {
		dev_err(vpfe_dev, "Invalid value of field id polarity\n");
		return -1;
	}

	if (ccdcparam->vd_pol != CCDC_PINPOL_POSITIVE
	    && ccdcparam->vd_pol != CCDC_PINPOL_NEGATIVE) {
		dev_err(vpfe_dev, "Invalid value of VD polarity\n");
		return -1;
	}

	if (ccdcparam->hd_pol != CCDC_PINPOL_POSITIVE
	    && ccdcparam->hd_pol != CCDC_PINPOL_NEGATIVE) {
		dev_err(vpfe_dev, "Invalid value of HD polarity\n");
		return -1;
	}

	if (ccdcparam->datasft < NO_SHIFT || ccdcparam->datasft > _6BIT) {
		dev_err(vpfe_dev, "Invalid value of data shift\n");
		return -1;
	}

	if (ccdcparam->mfilt1 < NO_MEDIAN_FILTER1
	    || ccdcparam->mfilt1 > MEDIAN_FILTER1) {
		dev_err(vpfe_dev, "Invalid value of median filter1\n");
		return -1;
	}

	if (ccdcparam->mfilt2 < NO_MEDIAN_FILTER2
	    || ccdcparam->mfilt2 > MEDIAN_FILTER2) {
		dev_err(vpfe_dev, "Invalid value of median filter2\n");
		return -1;
	}

	if (ccdcparam->ccdc_offset < 0 || ccdcparam->ccdc_offset > 1023) {
		dev_err(vpfe_dev, "Invalid value of offset\n");
		return -1;
	}

	if (ccdcparam->med_filt_thres < 0 || ccdcparam->med_filt_thres > 0x3FFF) {
		dev_err(vpfe_dev, "Invalid value of median filter thresold\n");
		return -1;
	}

	if (ccdcparam->data_sz < _16BITS || ccdcparam->data_sz > _8BITS) {
		dev_err(vpfe_dev, "Invalid value of data size\n");
		return -1;
	}

	if (ccdcparam->alaw.b_alaw_enable) {
		if (ccdcparam->alaw.gama_wd < BITS_13_4
		    || ccdcparam->alaw.gama_wd > BITS_09_0) {
			dev_err(vpfe_dev, "Invalid value of ALAW\n");
			return -1;
		}
	}

	if (ccdcparam->blk_clamp.b_clamp_enable) {
		if (ccdcparam->blk_clamp.sample_pixel < _1PIXELS
		    && ccdcparam->blk_clamp.sample_pixel > _16PIXELS) {
			dev_err(vpfe_dev, "Invalid value of sample pixel\n");
			return -1;
		}
		if (ccdcparam->blk_clamp.sample_ln < _1LINES
		    && ccdcparam->blk_clamp.sample_ln > _16LINES) {
			dev_err(vpfe_dev, "Invalid value of sample lines\n");
			return -1;
		}

	}

	if (ccdcparam->vertical_dft.ver_dft_en) {
		dev_err(vpfe_dev, "Defect correction is not supported\n");
		return -1;
	}

	if (ccdcparam->lens_sh_corr.lsc_enable) {
		dev_err(vpfe_dev,
			"Lens shadding correction is not supported\n");
		return -1;
	}
	if (ccdcparam->color_space_con.csc_enable) {
		dev_err(vpfe_dev, "Color Space converter not supported\n");
		return -1;
	}
	return 0;
}
