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
/* davinci_ipipe_hw.c file */

#include <linux/errno.h>
#include <linux/delay.h>
#include <asm-arm/arch-davinci/dm355_ipipe_hw.h>
#include <asm-arm/arch-davinci/dm355_ipipe.h>

#include <linux/device.h>
#ifdef __KERNEL__

extern struct device *ipipe_dev;

/* ipipe_hw_setup:It is used for Hardware Setup */
int ipipe_hw_setup(struct ipipe_params *config)
{
	u32 utemp = 0;
	u32 data_format;
	ipipeif_decimation decimation_en;
	ipipeif_input_source input_source = config->ipipeif_param.source;
	if (!config)
		return -EINVAL;

	/*Hardware set up of IPIPEIF Module */
	/*Combine all the fields to make CFG register of IPIPEIF */
	utemp = config->ipipeif_param.mode << 0;
	utemp |= config->ipipeif_param.decimation << 1;
	utemp |= config->ipipeif_param.source << 2;
	utemp |= config->ipipeif_param.clk_div << 4;
	utemp |= config->ipipeif_param.avg_filter << 7;
	utemp |= config->ipipeif_param.pack_mode << 8;
	utemp |= config->ipipeif_param.ialaw << 9;
	utemp |= config->ipipeif_param.clock_select << 10;
	utemp |= config->ipipeif_param.data_shift << 11;
	regw_if(utemp, IPIPEIF_GFG);
	switch (input_source) {
	case CCDC:
		regw_if(config->ipipeif_param.gain, IPIPEIF_GAIN);

		break;

	case SDRAM_RAW:

	case CCDC_DARKFM:
		regw_if(config->ipipeif_param.gain, IPIPEIF_GAIN);

	case SDRAM_YUV:
		regw_if(config->ipipeif_param.glob_hor_size, IPIPEIF_PPLN);
		regw_if(config->ipipeif_param.glob_ver_size, IPIPEIF_LPFR);
		regw_if(config->ipipeif_param.hnum, IPIPEIF_HNUM);
		regw_if(config->ipipeif_param.vnum, IPIPEIF_VNUM);
#if 0
		/* Following Register are part of buffer management (ipipe) */
		regw_if(config->ipipeif_param.addru, IPIPEIF_ADDRU);
		regw_if(config->ipipeif_param.addrl, IPIPEIF_ADDRL);
		regw_if(config->ipipeif_param.adofs, IPIPEIF_ADOFS);
#endif
		utemp = regr_vpss(VPSS_PCR);
		RESETBIT(utemp, 4);
		RESETBIT(utemp, 5);
		regw_vpss(utemp, VPSS_PCR);

		break;

	}
	/*check if decimation is enable or not */
	decimation_en = config->ipipeif_param.decimation;
	if (decimation_en) {
		regw_if(config->ipipeif_param.rsz, IPIPEIF_RSZ);
		/* Enable Aneraging filter PRATIK */
	}

	/*Hardware set up of IPIPE Module */
	/*set GCL_ARM reg before writting to ipipe registers */
	regw_ip(1, GCL_ARM);
	/*enable the clock wb,cfa,dfc,d2f,pre modules */
	regw_ip(0x06, GCL_CCD);
	data_format =
	    (config->ipipe_dpaths_fmt | (config->ipipe_dpaths_bypass) << 2);

	utemp = regr_vpss(VPSS_PCR);
	RESETBIT(utemp, 6);
	regw_vpss(utemp, VPSS_PCR);
	/*enable ipipe mode to either one shot or continuous */
	utemp = regr_ip(IPIPE_MODE);
	RESETBIT(utemp, 0); /*first reset mode bit and then set it by
			      config->ipipe_mode */
	utemp |= config->ipipe_mode;
	regw_ip(utemp, IPIPE_MODE);
	regw_ip(data_format, IPIPE_DPATHS);
	/*set size */
	regw_ip(config->ipipe_vst, IPIPE_VST);
	regw_ip(config->ipipe_hst, IPIPE_HST);
	regw_ip(config->ipipe_vsz, IPIPE_VSZ);
	regw_ip(config->ipipe_hsz, IPIPE_HSZ);
	switch (data_format) {
	case RAW2YUV:

		/*Combine all the fields to make COLPAT register of IPIPE */
		utemp = (config->ipipe_colpat_elep << 0);
		utemp |= (config->ipipe_colpat_elop << 2);
		utemp |= (config->ipipe_colpat_olep << 4);
		utemp |= (config->ipipe_colpat_olop << 6);

		regw_ip(utemp, IPIPE_COLPAT);
		/* set_dfc_regs(&(config->def_cor)); */ /*defect correction */
		set_d2f_regs(&(config->prog_nf));	/*noise filter */
		set_pre_regs(&(config->prefilter));	/*prefilter */
		/*histogram configuration may come here */
		set_wb_regs(&(config->wb));
		/*diff data_format value needs diff regs to be
		  configured so passing the value of data_format */
		set_rgb_2_yuv_regs(config->ipipe_dpaths_fmt,
				   &(config->rgb2yuv));

		set_rgb_to_rgb_regs(&(config->rgb2rgb));	/*set RGB_*
								   and GMM reg */
		/*boundary scan calc configuration may come here */

		set_ee_regs(&(config->edge_enhancer));
		set_fcs_regs(&(config->false_color_suppresion));
		set_rsz_regs(config);	/*set RSZ_SEQ registers */
		set_aal_regs(config);	/*set RSZ_AAL registers */
		/*set the registers of either RSZ0 or RSZ1 */
		set_rsz_structs(config);

		break;

	case RAW2RAW:

		/* set_dfc_regs(&(config->def_cor)); */       /*defect correction */
		set_d2f_regs(&(config->prog_nf));	/*noise filter */
		set_pre_regs(&(config->prefilter));	/*prefilter */
		set_wb_regs(&(config->wb));	/* .................CHECK */
		/*diff format value needs diff regs to be configured */
		/*set_rgb_2_yuv_regs(config->ipipe_dpaths_fmt,
		   &(config->rgb2yuv)); */
		set_aal_regs(config);	/*set RSZ_AAL registers */
		/*set the registers of RSZ0 and RSZ1 */
		set_rsz_structs(config);	/*...................CHECK */
		default_for_raw2raw(config);
		break;

	case RAW2BOX:
		printk(KERN_INFO "boxcar mode is not supported by driver\n");

		break;

	case YUV2YUV:
		set_ee_regs(&(config->edge_enhancer));
		set_fcs_regs(&(config->false_color_suppresion));
		set_rsz_regs(config);	/*set RSZ_SEQ registers */
		set_aal_regs(config);	/*set RSZ_AAL registers */
		/*set the registers of either RSZ0 or RSZ1 */
		set_rsz_structs(config);
		break;

	case RAW2RAW_BYPASS:
		/*set_dfc_regs(&(config->def_cor)); */      /*defect correction */
		set_d2f_regs(&(config->prog_nf));	/*noise filter */
		set_pre_regs(&(config->prefilter));	/*prefilter */
		set_wb_regs(&(config->wb));
		/*diff data_format  value needs diff regs to be configured
		   so passing the value of data_format */
		set_rgb_2_yuv_regs(config->ipipe_dpaths_fmt,
				   &(config->rgb2yuv));
		default_for_raw2raw(config);
		default_for_bypass(config);
		set_aal_regs(config);	/*set RSZ_AAL registers */
		/*set the registers of either RSZ0 or RSZ1 */
		set_rsz_structs(config);
		break;

	}
	return 0;

}

/*default configuratins for RAW2RAW mode*/
int default_for_raw2raw(struct ipipe_params *parameter)
{
	u32 utemp;
	u32 bright = 0;
	u32 contrast = 16;

	int seq_tmm = 0;
	utemp = regr_vpss(VPSS_MEMCTL);
	RESETBIT(utemp, 1);
	RESETBIT(utemp, 0);
	SETBIT(utemp, 2);
	regw_vpss(utemp, VPSS_MEMCTL);

	regw_ip(1, GCL_SDR);

	/*set this to 0 for dafault config */
	utemp =
	    (parameter->rsz_seq_seq << 0) | (seq_tmm << 1) | (parameter->
							      rsz_seq_hrv << 2)
	    | (parameter->rsz_seq_vrv << 3) | (parameter->rsz_seq_crv << 4);
	regw_ip(utemp, RSZ_SEQ);
	/*set this to 0 for dafault config */
	regw_ip(0, FCS_EN);
	/*set this to 0 for dafault config */
	regw_ip(0, YEE_EN);
	/*set default brightness and contrast */
	utemp = ((contrast << 0) | (bright << 8));
	regw_ip(utemp, YUV_ADJ);

	/*set default luminance */
	regw_ip(0, YUV_Y_MIN);
	regw_ip(255, YUV_Y_MAX);

	/*set default chrominance */
	regw_ip(0, YUV_C_MIN);
	regw_ip(255, YUV_C_MAX);
	/*default config for resizer 1  registers */
	regw_ip(1, RSZ_EN_0);
	regw_ip(0, RSZ_EN_0 + RSZ_I_HST);
	regw_ip(0, RSZ_EN_0 + RSZ_I_VST);
	regw_ip(0, RSZ_EN_0 + RSZ_O_HST);
	regw_ip(0, RSZ_EN_0 + RSZ_V_PHS);
	regw_ip(256, RSZ_EN_0 + RSZ_V_DIF);
	regw_ip(256, RSZ_EN_0 + RSZ_H_DIF);
	regw_ip(0, RSZ_EN_0 + RSZ_H_PHS);
	regw_ip(0, RSZ_EN_0 + RSZ_H_TYP);
	regw_ip(0, RSZ_EN_0 + RSZ_H_LSE);
	regw_ip(0, RSZ_EN_0 + RSZ_H_LPF);
	regw_ip(0, RSZ_EN_0 + RSZ_RGB_EN);
	/*disable resizer 0 in default mode */
	regw_ip(0, RSZ_EN_1);
/*
for debugging	
*/
	return 0;
}

/*default configuratins for RAW2RAW_bypass mode*/
int default_for_bypass(struct ipipe_params *parameter)
{
	/*disable noise filter in default config */
	regw_ip(0, D2F_EN);
	/*disable defect coorection in default config */
	regw_ip(0, DFC_EN);
	/*disable prefilter filter in default config */
	regw_ip(0, PRE_EN);
	/*set default config for white balance */
	regw_ip(256, WB2_DGN);
	regw_ip(128, WB2_WG_R);
	regw_ip(128, WB2_WG_GR);
	regw_ip(128, WB2_WG_GB);
	regw_ip(128, WB2_WG_B);
	return 0;
}

/*IPIPE Register write function definition */
int set_dfc_regs(struct ipipe_def_cor *dfc)
{
	u32 utemp;
	unsigned int horizontal_pos[MAX_SIZE_DFC];
	unsigned int vertical_pos_method[MAX_SIZE_DFC];
	unsigned int count;
	regw_ip(dfc->dfc_en, DFC_EN);	/*writting to enable register */

	if (1 == dfc->dfc_en) {
		regw_ip(dfc->dfc_sel, DFC_SEL);
		regw_ip(DEF_COR_START_ADDR, DFC_ADR);
		regw_ip(dfc->dfc_siz, DFC_SIZE);
		utemp = regr_vpss(VPSS_MEMCTL);
		RESETBIT(utemp, 0);
		regw_vpss(utemp, VPSS_MEMCTL);
		/*set the auto increment,write only,dfc mode in RAM_MODE */
		regw_ip(0x0034, RAM_MODE);
		regw_ip(0x00, RAM_ADR);
		regw_ip(dfc->dfc_adr, DFC_ADR);
		/*regw_ip(0x00,DFC_ADR);*/
		if (dfc->dfc_table != NULL) {
			for (count = 0; count < dfc->dfc_siz; count++) {
				horizontal_pos[count] =
				    dfc->dfc_table[count] & 0x00000FFF;
				vertical_pos_method[count] =
				    dfc->dfc_table[count] & 0x07FFF000;
			}

			/*write first twelve bit */
			count = 0;

			while (count < dfc->dfc_siz) {
				regw_ip(horizontal_pos[count], RAM_WDT);
				printk(KERN_INFO "###RAM_WDT[%d] = %x\n", count,
				       regr_ip(RAM_WDT));
				/*write next fifteen bit */
				regw_ip(vertical_pos_method[count], RAM_WDT);
				printk(KERN_INFO "RAM_WDT[%d] = %x\n", count,
				       regr_ip(RAM_WDT));

				count++;
			}

		} else {

		}

	}
	return 0;

}

int set_d2f_regs(struct ipipe_prog_nf *noise_filter)
{
	u32 utemp;
	int count = 0;
	regw_ip(noise_filter->noise_fil_en, D2F_EN);
	if (1 == noise_filter->noise_fil_en) {
		/*Combine all the fields to make D2F_CFG register of IPIPE */
		utemp =
		    (noise_filter->d2f_cfg_spr << 0) | (noise_filter->
							d2f_cfg_shf << 2) |
		    (noise_filter->type << 4);
		regw_ip(utemp, D2F_CFG);
		if (noise_filter->d2f_str != NULL) {
			count = 0;
			while (count < 32) {
				regw_ip(noise_filter->d2f_str[count],
					D2F_STR + count * 4);
				count++;
			}
		} else {
		}
		if (noise_filter->d2f_thr != NULL) {
			count = 0;
			while (count < 32) {
				regw_ip(noise_filter->d2f_thr[count],
					DFC_THR + count * 4);
				count++;
			}
		} else {
		}
	}
	return 0;
}

int set_pre_regs(struct ipipe_prefilter *pre_filter)
{

	u32 utemp;
	regw_ip(pre_filter->pre_en, PRE_EN);
	if (1 == pre_filter->pre_en) {
		/*Combine all the fields to make PRE_EN register of IPIPE */
		utemp = ((pre_filter->sel_0 << 0) | (pre_filter->sel_1 << 1) |
			 (pre_filter->typ_adaptive << 2) | (pre_filter->
							    typ_adaptive_dotred)
			 << 3);
		regw_ip(utemp, PRE_TYP);
		regw_ip(pre_filter->pre_shf, PRE_SHF);
		regw_ip(pre_filter->pre_gain, PRE_GAIN);
		regw_ip(pre_filter->pre_thr_g, PRE_THR_G);
		regw_ip(pre_filter->pre_thr_b, PRE_THR_B);
		regw_ip(pre_filter->pre_thr_1, PRE_THR_1);
	}
	return 0;

}
int set_wb_regs(struct ipipe_wb *white_balance)
{
	regw_ip(white_balance->wb2_dgn, WB2_DGN);
	regw_ip(white_balance->wb2_wg_r, WB2_WG_R);
	regw_ip(white_balance->wb2_wg_gr, WB2_WG_GR);
	regw_ip(white_balance->wb2_wg_gb, WB2_WG_GB);
	regw_ip(white_balance->wb2_wg_b, WB2_WG_B);
	return 0;
}

int set_rgb_2_yuv_regs(int data_format, struct ipipe_rgb2yuv *y_cr_cb)
{
	u32 utemp;
	if (data_format < 2) {
		/*combine fields of YUV_ADJ to set brightness and contrast */
		utemp =
		    ((y_cr_cb->yuv_adj_ctr << 0) | (y_cr_cb->yuv_adj_brt << 8));
		regw_ip(utemp, YUV_ADJ);
		regw_ip(y_cr_cb->yuv_y_min, YUV_Y_MIN);
		regw_ip(y_cr_cb->yuv_y_max, YUV_Y_MAX);
		regw_ip(y_cr_cb->yuv_c_min, YUV_C_MIN);
		regw_ip(y_cr_cb->yuv_c_max, YUV_C_MAX);

	}
	if (data_format == 0) {

		regw_ip(y_cr_cb->yuv_mul_ry, YUV_MUL_RY);
		regw_ip(y_cr_cb->yuv_mul_gy, YUV_MUL_GY);
		regw_ip(y_cr_cb->yuv_mul_by, YUV_MUL_BY);
		regw_ip(y_cr_cb->yuv_mul_rcb, YUV_MUL_RCB);
		regw_ip(y_cr_cb->yuv_mul_gcb, YUV_MUL_GCB);
		regw_ip(y_cr_cb->yuv_mul_bcb, YUV_MUL_BCB);
		regw_ip(y_cr_cb->yuv_mul_rcr, YUV_MUL_RCR);
		regw_ip(y_cr_cb->yuv_mul_gcr, YUV_MUL_GCR);
		regw_ip(y_cr_cb->yuv_mul_bcr, YUV_MUL_BCR);
		regw_ip(y_cr_cb->yuv_oft_y, YUV_OFT_Y);
		regw_ip(y_cr_cb->yuv_oft_cb, YUV_OFT_CB);
		regw_ip(y_cr_cb->yuv_oft_cr, YUV_OFT_CR);
		/*Combine all the fields to make YUV_PHS register of IPIPE */
		utemp =
		    ((y_cr_cb->yuv_phs_position << 0) | (y_cr_cb->
							 yuv_phs_lpf << 1));
		regw_ip(utemp, YUV_PHS);

	}
	return 0;
}
int set_rgb_to_rgb_regs(struct ipipe_rgb2rgb *rgb)
{
	u32 utemp;
	int count, table_size = 0;

	regw_ip(rgb->rgb_mul_rr, RGB_MUL_RR);
	regw_ip(rgb->rgb_mul_gr, RGB_MUL_GR);
	regw_ip(rgb->rgb_mul_br, RGB_MUL_BR);
	regw_ip(rgb->rgb_mul_rg, RGB_MUL_RG);
	regw_ip(rgb->rgb_mul_gg, RGB_MUL_GG);
	regw_ip(rgb->rgb_mul_bg, RGB_MUL_BG);
	regw_ip(rgb->rgb_mul_rb, RGB_MUL_RB);
	regw_ip(rgb->rgb_mul_gb, RGB_MUL_GB);
	regw_ip(rgb->rgb_mul_bb, RGB_MUL_BB);
	regw_ip(rgb->rgb_oft_or, RGB_MUL_OR);
	regw_ip(rgb->rgb_oft_og, RGB_MUL_OG);
	regw_ip(rgb->rgb_oft_ob, RGB_MUL_OB);

	utemp =
	    ((rgb->gmm_cfg_bypr << 0) | (rgb->gmm_cfg_bypg << 1) | (rgb->
								    gmm_cfg_bypb
								    << 2)
	     | (rgb->gmm_cfg_tbl << 4) | (rgb->gmm_cfg_siz << 5));

	regw_ip(utemp, GMM_CFG);
	/*testing -- for register write */
	utemp = regr_ip(GMM_CFG);

	if (rgb->gmm_cfg_siz == IPIPE_128) {
		table_size = 128 * 2;
	} else if (rgb->gmm_cfg_siz == IPIPE_256) {
		table_size = 256 * 2;
	} else if (rgb->gmm_cfg_siz == IPIPE_512) {
		table_size = 512 * 2;
	}
	if (!(rgb->gmm_cfg_bypr)) {
		if (rgb->gmm_tbl_r != NULL) {
			/*set the auto increment,write only, gamma
			  red mode in RAM_MODE */
			regw_ip(0x0035, RAM_MODE);
			/*set the starting address of gamma table */
			regw_ip(0x00, RAM_ADR);

			for (count = 0; count < table_size; count++)
				regw_ip(rgb->gmm_tbl_r[count], RAM_WDT);
		}
	}
	if (!(rgb->gmm_cfg_bypb)) {
		if (rgb->gmm_tbl_b != NULL) {
			/*set the auto increment,write only, gamma red mode
			  in RAM_MODE */
			regw_ip(0x0036, RAM_MODE);
			/*set the starting address of gamma table */
			regw_ip(0x00, RAM_ADR);
			for (count = 0; count < table_size; count++)
				regw_ip(rgb->gmm_tbl_b[count], RAM_WDT);
		}
	}
	if (!(rgb->gmm_cfg_bypg)) {
		if (rgb->gmm_tbl_g != NULL) {
			/*set the auto increment,write only, gamma red
			  mode in RAM_MODE */
			regw_ip(0x0037, RAM_MODE);
			/*set the starting address of gamma table */
			regw_ip(0x00, RAM_ADR);
			for (count = 0; count < table_size; count++)
				regw_ip(rgb->gmm_tbl_g[count], RAM_WDT);
		}
	}
	/*set the auto increment,write only, gamma red mode in RAM_MODE */
	regw_ip(0x0038, RAM_MODE);
	/*set the starting address of gamma table */
	regw_ip(0x00, RAM_ADR);
	if (rgb->gmm_tbl_all != NULL) {
		printk(KERN_INFO "gamma table not null\n");
		for (count = 0; count < table_size; count++)
			regw_ip(rgb->gmm_tbl_all[count], RAM_WDT);
	} else {
	}
	regw_ip(0x00, RAM_MODE);
	return 0;
}

int set_ee_regs(struct ipipe_edge_enhancer *edge_enhance)
{
	unsigned int count;
	regw_ip(edge_enhance->yee_en, YEE_EN);
	if (1 == edge_enhance->yee_en) {
		regw_ip(edge_enhance->yee_emf, YEE_EMF);
		regw_ip(edge_enhance->yee_shf, YEE_SHF);
		regw_ip(edge_enhance->yee_mul_00, YEE_MUL_00);
		regw_ip(edge_enhance->yee_mul_01, YEE_MUL_01);
		regw_ip(edge_enhance->yee_mul_02, YEE_MUL_02);
		regw_ip(edge_enhance->yee_mul_10, YEE_MUL_10);
		regw_ip(edge_enhance->yee_mul_11, YEE_MUL_11);
		regw_ip(edge_enhance->yee_mul_12, YEE_MUL_12);
		regw_ip(edge_enhance->yee_mul_20, YEE_MUL_20);
		regw_ip(edge_enhance->yee_mul_21, YEE_MUL_21);
		regw_ip(edge_enhance->yee_mul_22, YEE_MUL_22);
		/*set the auto increment,write only,ee mode in RAM_MODE */
		regw_ip(0x0039, RAM_MODE);
		regw_ip(-512 /*0x1FF */ , RAM_ADR);
		if (edge_enhance->ee_table != NULL) {
			for (count = 0; count < MAX_SIZE_EEC; count++)
				regw_ip(edge_enhance->ee_table[count], RAM_WDT);

			regw_ip(0x0, RAM_MODE);
			regw_ip(0x0019, RAM_MODE);
			regw_ip(-512, RAM_ADR);
			for (count = 0; count < MAX_SIZE_EEC; count++)
				regw_ip(0xFF, RAM_WDT);
		}
	}
	return 0;
}

int set_fcs_regs(struct ipipe_false_color_suppresion *color_supress)
{
	regw_ip(color_supress->fcs_en, FCS_EN);
	if (1 == color_supress->fcs_en) {
		regw_ip(color_supress->fcs_typ_typ, FCS_TYP);
		regw_ip(color_supress->fcs_shf_y, FCS_SHF_Y);
		regw_ip(color_supress->fcs_shf_c, FCS_SHF_C);
		regw_ip(color_supress->fcs_thr, FCS_THR);
		regw_ip(color_supress->fcs_sgn, FCS_SGN);
		regw_ip(color_supress->fcs_lth, FCS_LTH);
	}
	return 0;
}

int set_rsz_regs(struct ipipe_params *param_resize)
{
	u32 utemp;
	/*Combine all the fields to make RSZ_SEQ register of IPIPE */
	utemp =
	    (param_resize->rsz_seq_seq << 0) | (param_resize->
						rsz_seq_tmm << 1) |
	    (param_resize->rsz_seq_hrv << 2)
	    | (param_resize->rsz_seq_vrv << 3) | (param_resize->
						  rsz_seq_crv << 4);
	regw_ip(utemp, RSZ_SEQ);

	return 0;
}

int set_aal_regs(struct ipipe_params *param_resize)
{
	regw_ip(param_resize->rsz_aal, RSZ_AAL);
	return 0;
}

int set_rsz_structs(struct ipipe_params *params)
{				/*set the registers of either RSZ0 or RSZ1 */
	u32 utemp;
	u32 rsz_seq, rsz_tmm;
	utemp = regr_vpss(VPSS_MEMCTL);
	RESETBIT(utemp, 1);
	RESETBIT(utemp, 0);
	SETBIT(utemp, 2);
	regw_vpss(utemp, VPSS_MEMCTL);
	regw_ip(params->rsz_en[0], RSZ_EN_0);
	if (params->rsz_en[0]) {
		/*testing--- for register write */
		utemp = regr_ip(RSZ_EN_0);
		/*enable RSZ clock */
		regw_ip(1, GCL_SDR);
		/*setting rescale parameters */
		regw_ip(params->rsz_rsc_param[0].rsz_mode, RSZ_EN_0 + RSZ_MODE);
		regw_ip(params->rsz_rsc_param[0].rsz_i_vst,
			RSZ_EN_0 + RSZ_I_VST);
		regw_ip(params->rsz_rsc_param[0].rsz_i_hst,
			RSZ_EN_0 + RSZ_I_HST);
		regw_ip(params->rsz_rsc_param[0].rsz_o_vsz,
			RSZ_EN_0 + RSZ_O_VSZ);
		regw_ip(params->rsz_rsc_param[0].rsz_o_hsz,
			RSZ_EN_0 + RSZ_O_HSZ);
		regw_ip(params->rsz_rsc_param[0].rsz_o_hst,
			RSZ_EN_0 + RSZ_O_HST);
		regw_ip(params->rsz_rsc_param[0].rsz_v_phs,
			RSZ_EN_0 + RSZ_V_PHS);
		regw_ip(params->rsz_rsc_param[0].rsz_v_dif,
			RSZ_EN_0 + RSZ_V_DIF);
		regw_ip(params->rsz_rsc_param[0].rsz_h_phs,
			RSZ_EN_0 + RSZ_H_PHS);
		regw_ip(params->rsz_rsc_param[0].rsz_h_dif,
			RSZ_EN_0 + RSZ_H_DIF);
		regw_ip(params->rsz_rsc_param[0].rsz_h_typ,
			RSZ_EN_0 + RSZ_H_TYP);
		regw_ip(params->rsz_rsc_param[0].rsz_h_lse_sel,
			RSZ_EN_0 + RSZ_H_LSE);
		regw_ip(params->rsz_rsc_param[0].rsz_h_lpf,
			RSZ_EN_0 + RSZ_H_LPF);

		/*seting rgb conversion parameters */
		regw_ip(params->rsz2rgb[0].rsz_rgb_en, RSZ_EN_0 + RSZ_RGB_EN);
		regw_ip(params->rsz2rgb[0].rsz_rgb_en, RSZ_EN_0 + RSZ_RGB_EN);
		utemp =
		    ((params->rsz2rgb[0].rsz_rgb_typ << 0) | (params->
							      rsz2rgb[0].
							      rsz_rgb_msk0 << 1)
		     | (params->rsz2rgb[0].rsz_rgb_msk1) << 2);
		regw_ip(utemp, RSZ_RGB_TYP);
		regw_ip(params->rsz2rgb[0].rsz_rgb_alpha_val,
			RSZ_EN_0 + RSZ_RGB_BLD);

		/*setting external memory parameters */
		regw_ip(params->ext_mem_param[0].rsz_sdr_oft,
			RSZ_EN_0 + RSZ_SDR_OFT);
		regw_ip(params->ext_mem_param[0].rsz_sdr_ptr_s,
			RSZ_EN_0 + RSZ_SDR_PTR_S);
		regw_ip(params->ext_mem_param[0].rsz_sdr_ptr_e,
			RSZ_EN_0 + RSZ_SDR_PTR_E);
	}

	regw_ip(params->rsz_en[1], RSZ_EN_1);
	if (params->rsz_en[1]) {
		/*testing---- for register write */
		utemp = regr_ip(RSZ_EN_1);

		/*enable RSZ clock */
		regw_ip(1, GCL_SDR);
		/*setting rescale parameters */
		regw_ip(params->rsz_rsc_param[1].rsz_mode, RSZ_EN_1 + RSZ_MODE);
		regw_ip(params->rsz_rsc_param[1].rsz_i_vst,
			RSZ_EN_1 + RSZ_I_VST);
		/*regw_ip(rez_rescale_para->rsz_i_vsz,
		   RSZ_EN_0 + RSZ_I_VSZ); */
		regw_ip(params->rsz_rsc_param[1].rsz_i_hst,
			RSZ_EN_1 + RSZ_I_HST);
		regw_ip(params->rsz_rsc_param[1].rsz_o_vsz,
			RSZ_EN_1 + RSZ_O_VSZ);
		regw_ip(params->rsz_rsc_param[1].rsz_o_hsz,
			RSZ_EN_1 + RSZ_O_HSZ);
		regw_ip(params->rsz_rsc_param[1].rsz_o_hst,
			RSZ_EN_1 + RSZ_O_HST);
		regw_ip(params->rsz_rsc_param[1].rsz_v_phs,
			RSZ_EN_1 + RSZ_V_PHS);
		regw_ip(params->rsz_rsc_param[1].rsz_v_dif,
			RSZ_EN_1 + RSZ_V_DIF);
		regw_ip(params->rsz_rsc_param[1].rsz_h_phs,
			RSZ_EN_1 + RSZ_H_PHS);
		regw_ip(params->rsz_rsc_param[1].rsz_h_dif,
			RSZ_EN_1 + RSZ_H_DIF);
		regw_ip(params->rsz_rsc_param[1].rsz_h_typ,
			RSZ_EN_1 + RSZ_H_TYP);
		regw_ip(params->rsz_rsc_param[1].rsz_h_lse_sel,
			RSZ_EN_1 + RSZ_H_LSE);
		regw_ip(params->rsz_rsc_param[1].rsz_h_lpf,
			RSZ_EN_1 + RSZ_H_LPF);

		/*seting rgb conversion parameters */
		regw_ip(params->rsz2rgb[1].rsz_rgb_en, RSZ_EN_1 + RSZ_RGB_EN);
		regw_ip(params->rsz2rgb[1].rsz_rgb_en, RSZ_EN_1 + RSZ_RGB_EN);
		utemp =
		    ((params->rsz2rgb[1].rsz_rgb_typ << 0) | (params->
							      rsz2rgb[1].
							      rsz_rgb_msk0 << 1)
		     | (params->rsz2rgb[1].rsz_rgb_msk1) << 2);
		regw_ip(utemp, RSZ_RGB_TYP);
		regw_ip(params->rsz2rgb[1].rsz_rgb_alpha_val,
			RSZ_EN_1 + RSZ_RGB_BLD);

		/*setting external memory parameters */
		regw_ip(params->ext_mem_param[1].rsz_sdr_oft,
			RSZ_EN_1 + RSZ_SDR_OFT);
		regw_ip(params->ext_mem_param[1].rsz_sdr_ptr_s,
			RSZ_EN_1 + RSZ_SDR_PTR_S);
		regw_ip(params->ext_mem_param[1].rsz_sdr_ptr_e,
			RSZ_EN_1 + RSZ_SDR_PTR_E);

	} else {
	}

	if (!params->rsz_en[0] && !params->rsz_en[1]) {	/*resizer bypass mode */
		rsz_tmm = 0;
		rsz_seq = 0;
		utemp =
		    (params->rsz_seq_seq << 0) | (params->
						  rsz_seq_tmm << 1) | (params->
								       rsz_seq_hrv
								       << 2)
		    | (params->rsz_seq_vrv << 3) | (params->rsz_seq_crv << 4);
		regw_ip(0, RSZ_AAL);
		regw_ip(0, RSZ_EN_0 + RSZ_O_HST);
		regw_ip(0, RSZ_EN_0 + RSZ_V_PHS);
		regw_ip(256, RSZ_EN_0 + RSZ_V_DIF);
		regw_ip(256, RSZ_EN_0 + RSZ_H_DIF);
		regw_ip(0, RSZ_EN_0 + RSZ_H_LSE);
		regw_ip(0, RSZ_EN_0 + RSZ_H_PHS);
		regw_ip(0, RSZ_EN_1);
		/*disable resizer clock, necessary to bypass resizer */
		regw_ip(0, GCL_SDR);

	}
	return 0;
}

#endif				/* End of #ifdef __KERNEL__ */
