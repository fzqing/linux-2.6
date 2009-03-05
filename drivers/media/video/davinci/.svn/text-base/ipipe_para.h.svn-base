#define  WIDTH_I 640
#define  HEIGHT_I 480
#define  WIDTH_O 640
#define  HEIGHT_O 480

struct ipipe_params param_def = {
	.ipipeif_param = {
			  /*IPPEIF config register */
			  .data_shift = BITS9_0,
			  .clock_select = SDRAM_CLK,

			  .ialaw = ALAW_OFF,
			  .pack_mode = SIXTEEN_BIT,
			  .avg_filter = AVG_OFF,
			  .clk_div = DIVIDE_SIXTH,
			  .source = SDRAM_RAW,
			  .decimation = DECIMATION_OFF,
			  .mode = ONE_SHOT,

			  .glob_hor_size = WIDTH_I + 8,	/*632,*/
			  .glob_ver_size = HEIGHT_I + 10,	/*466,*/
			  .hnum = WIDTH_I,	/* 624,*/
			  .vnum = HEIGHT_I,	/* 456, */
			  .adofs = WIDTH_I * 2,	/* 1248,  to make it
						   32 alligned */
			  .rsz = 16,	/* resize ratio = 16/rsz:
					   valid range (16-112) */
			  .gain = 0x200	/* (precision is U10Q9) */
			  },
	.ipipe_mode = ONE_SHOT,
	/*input/output datapath register */
	.ipipe_dpaths_fmt = RAW2YUV,
	.ipipe_dpaths_bypass = RAW_MODE_OFF,	/*...check */

	/*color pattern register */
	.ipipe_colpat_olop = RED,
	.ipipe_colpat_olep = GREEN_RED,
	.ipipe_colpat_elop = GREEN_BLUE,
	.ipipe_colpat_elep = BLUE,

	/*horizontal/vertical start, horizontal/vertical size */
	.ipipe_vst = 0,
	.ipipe_vsz = HEIGHT_I - 1,	/*456 - 1,*/
	.ipipe_hst = 0,		/*check*/
	.ipipe_hsz = WIDTH_I - 1,	/*624 - 1,*/
	/*interupt generation after lines */

	.def_cor = {.dfc_en = DISABLE,
		    .dfc_sel = 0,
		    /*.dfc_adr = 0,*/
		    .dfc_siz = 4,
		    .dfc_table = NULL	/*(unsigned int*) yeeTable*/
		    },
	.prog_nf = {
		    .noise_fil_en = DISABLE,
		    .d2f_cfg_spr = 0,
		    .d2f_cfg_shf = 0,
		    .type = 0,
		    .d2f_thr = NULL,	/*(unsigned int *)NoiseFilterTHR ,*/
		    .d2f_str = NULL	/*(unsigned int *)NoiseFilterSTR */
		    },
	.prefilter = {
		      .pre_en = ENABLE,
		      .sel_0 = 1,	/*AVG2MEDPIX,*/
		      .sel_1 = 1,
		      .typ_adaptive = ENABLE,
		      .typ_adaptive_dotred = DISABLE,
		      .pre_shf = 9,
		      .pre_gain = 128,
		      .pre_thr_g = 500,
		      .pre_thr_b = 4096,
		      .pre_thr_1 = 800},
	.wb = {
	       .wb2_dgn = 0x200,	/*512,512,1023,256, */
	       .wb2_wg_r = 0x40,	/*444,*/
	       .wb2_wg_gr = 0x45,	/*256,*/
	       .wb2_wg_gb = 0x60,	/*256,*/
	       .wb2_wg_b = 0x45,	/*568,428*/
	       },
	.rgb2rgb = {
		    .rgb_mul_rr = 0x1a1,	/*0x016c,*/
		    .rgb_mul_gr = 0xf8a,	/*0x0FA4,*/
		    .rgb_mul_br = 0xfd5,	/*0x0FF1,*/
		    .rgb_mul_rg = 0xfa1,	/*0x0FD2,*/
		    .rgb_mul_gg = 0x1c4,	/*0x013D,*/
		    .rgb_mul_bg = 0xf9b,	/*0x0FF1,*/
		    .rgb_mul_rb = 0xfbd,	/*0x0FD2,*/
		    .rgb_mul_gb = 0xfb1,	/*0x0FA4,*/
		    .rgb_mul_bb = 0x192,	/*0x018A,*/
		    .rgb_oft_or = 0x0000,
		    .rgb_oft_og = 0x0000,
		    .rgb_oft_ob = 0x0000,
		    .gmm_cfg_bypr = GC_BYPASS,
		    .gmm_cfg_bypg = GC_BYPASS,
		    .gmm_cfg_bypb = GC_BYPASS,
		    .gmm_cfg_tbl = IPIPE_RAM,
		    .gmm_cfg_siz = IPIPE_512,
		    .gmm_tbl_r = NULL,
		    .gmm_tbl_b = NULL,
		    .gmm_tbl_g = NULL,
		    .gmm_tbl_all = NULL	/*(unsigned int *)GammaTableall */
		    },
	.rgb2yuv = {
		    /* RDRV_IPIPE__SAT_LOW */
		    .yuv_adj_ctr = 0x10,
		    .yuv_adj_brt = 0x00,

		    .yuv_mul_ry = 0x004d,
		    .yuv_mul_gy = 0x0096,
		    .yuv_mul_by = 0x001d,
		    .yuv_mul_rcb = 0x03d4,
		    .yuv_mul_gcb = 0x03ac,
		    .yuv_mul_bcb = 0x0080,
		    .yuv_mul_rcr = 0x0080,
		    .yuv_mul_gcr = 0x0395,
		    .yuv_mul_bcr = 0x03eb,
		    .yuv_oft_y = 0x00,
		    .yuv_oft_cb = 0x80,
		    .yuv_oft_cr = 0x80,
		    .yuv_y_min = 0,
		    .yuv_y_max = 0xFF,
		    .yuv_c_min = 0,
		    .yuv_c_max = 0xFF,
		    .yuv_phs_lpf = DISABLE,
		    .yuv_phs_position = 1,

		    },
	.edge_enhancer = {

			  .yee_en = DISABLE,
			  .yee_emf = ENABLE,
			  .yee_shf = 4,	/* HPF Down Shift
					   Value: valid range (0-15) */
			  .yee_mul_00 = 48,
			  .yee_mul_01 = 12,
			  .yee_mul_02 = 1014,
			  .yee_mul_10 = 12,
			  .yee_mul_11 = 0,
			  .yee_mul_12 = 1018,
			  .yee_mul_20 = 1014,
			  .yee_mul_21 = 1018,
			  .yee_mul_22 = 1022,
			  .ee_table = NULL	/*(unsigned int*) yeeTable*/
			  },
	.false_color_suppresion = {
				   .fcs_en = ENABLE,	/* Uint8 csupEnable*/
				   .fcs_typ_typ = 0,
				   .fcs_shf_y = 0,
				   .fcs_shf_c = 7,
				   .fcs_thr = 235,
				   .fcs_sgn = 0,
				   .fcs_lth = 0},
	.rsz_seq_seq = DISABLE,
	.rsz_seq_tmm = DISABLE,	/* output confined mode (normal mode) */
	.rsz_seq_hrv = DISABLE,
	.rsz_seq_vrv = DISABLE,
	.rsz_seq_crv = DISABLE,

	.rsz_aal = DISABLE,

	.rsz_rsc_param = {
			  {
			   .rsz_mode = ONE_SHOT,
			   .rsz_i_vst = 0,
			   .rsz_i_vsz = 0,
			   .rsz_i_hst = 0,
			   .rsz_o_vsz = HEIGHT_O - 1,
			   .rsz_o_hsz = WIDTH_O - 1,
			   .rsz_o_hst = 0,
			   .rsz_v_phs = 0,
			   /*unsigned int rsz_v_phs_o;*/
			   .rsz_v_dif = 243,
			   /*unsigned int rsz_v_siz_o;*/
			   .rsz_h_phs = 0,
			   .rsz_h_dif = 243,
			   .rsz_h_typ = CUBIC,
			   .rsz_h_lse_sel = INTERNAL_VALUE,
			   .rsz_h_lpf = 0},
			  {
			   ONE_SHOT,
			   0,
			   0,
			   0,
			   239,
			   319,
			   0,
			   0,
			   256,
			   0,
			   256,
			   CUBIC,
			   INTERNAL_VALUE,
			   0}
			  },
	.rsz2rgb = {
		    {
		     .rsz_rgb_en = DISABLE,	/*....check*/
		     /* .rsz_rgb_typ = 0,
			.rsz_rgb_msk0 = 0,
			.rsz_rgb_msk1 = 0,
			.rsz_rgb_alpha_val = 0 */
		     },
		    {
		     DISABLE,
		     }
		    },

	.ext_mem_param = {
			  {
			   .rsz_sdr_bad_h = 0,
			   .rsz_sdr_bad_l = 0,
			   .rsz_sdr_sad_h = 0,
			   .rsz_sdr_sad_l = 0,
			   .rsz_sdr_oft = WIDTH_O * 2,
			   .rsz_sdr_ptr_s = 0,
			   .rsz_sdr_ptr_e = WIDTH_O},
			  {
			   0,
			   0,
			   0,
			   0,
			   WIDTH_O * 2,
			   0,
			   8191}
			  },
	.rsz_en[0] = ENABLE,
	.rsz_en[1] = DISABLE
};
