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
/* dm355_ipipe.h file */


#ifndef DM355_IPIPE_H
#define DM355_IPIPE_H

#include <linux/ioctl.h>

#ifdef __KERNEL__

/* include linux specific header files */
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <asm/semaphore.h>

#endif	/* End of #ifdef __KERNEL__ */


#define MAX_SIZE_DFC			1024
#define MAX_SIZE_EEC			1024
#define MAX_SIZE_GAMMA			512

#define MAX_SIZE_RAW_BY_PASS		4096
#define MAX_SIZE			1344
#define MAX_SIZE_RSZ0			1344
#define MAX_SIZE_RSZ1			640

#define WB_GAIN_MAX     4
#define RGB_MAX         3

#define MAX_BUFFER      8
#define SET_LOW_ADD    	0x0000FFFF
#define SET_HIGH_ADD	0xFFFF0000

#define IPIPE_BUF_IN     0	/* input buffer */
#define IPIPE_BUF_OUT    1	/* output buffer */

#define IPIPE_INWIDTH_8BIT   0	/* pixel width of 8 bitS */
#define IPIPE_INWIDTH_10BIT  1	/* pixel width of 10 bits */

/* 
 * list of enums
 */
typedef enum copy_method {
	FROMTOP    = 0,
	FROMBOTTON = 1
}copy_method_t; 

typedef enum sampling_type {
	BOX = 0,
	DIAMOND = 1
}sampling_type_t;

typedef enum pre_filter_type {
        AVG4PIX = 0,
        AVG2MEDPIX = 1
}pre_filter_type_t;

typedef enum enable_disable {
	DISABLE =  0,
	ENABLE =   1
}enable_disable_t;

typedef enum gamma_cor {
        GC_ENABLE = 0,
        GC_BYPASS = 1
}gamma_cor_t;

typedef enum gamma_tbl {
        IPIPE_RAM = 0,
        IPIPE_ROM = 1
}gamma_tbl_t;

typedef enum gamma_siz {
        IPIPE_128  = 0,
        IPIPE_256  = 1,
        IPIPE_RESV = 2,
        IPIPE_512  = 3
}gamma_siz_t;


typedef enum _ipipe_dpaths_fmt {
	RAW2YUV = 0,
	RAW2RAW = 1,
	RAW2BOX = 2,
	YUV2YUV = 3,
	RAW2RAW_BYPASS = 4
}ipipe_dpaths_fmt_t;

typedef enum _ipipe_dpaths_bypass {
        RAW_MODE_OFF = 0,
        RAW_MODE_ON  = 1    
}ipipe_dpaths_bypass_t;

typedef enum _ipipe_colpat {
	RED = 0, 	
	GREEN_RED  = 1,	
	GREEN_BLUE = 2,
	BLUE = 3 
}ipipe_colpat_t;

typedef enum fcs_typ {
        Y          = 0,
        HPF_HORZ   = 1,
	HPF_VERT   = 2,
	HPF_2D     = 3,
	HPF_2D_YEE = 4	
}fcs_typ_t;

/* Resizer */
typedef enum rsz_h_typ {
	CUBIC =  0,
	LINEAR = 1
}rsz_h_typ_t;

typedef enum rsz_h_lse {
	INTERNAL_VALUE   = 0,
	PROGRAMMED_VALUE = 1 
}rsz_h_lse_t;

typedef enum _ipipe_rsz_rgb_typ {
        OUTPUT_32BIT = 0,
        OUTPUT_16BIT = 1,
}ipipe_rsz_rgb_typ_t;

typedef enum _ipipe_rsz_rgb_msk {
        NOMASK = 0,
        MASKLAST2 = 1,
}ipipe_rsz_rgb_msk_t;

typedef enum _data_shift{
        BITS15_2	   = 0,
        BITS14_1	   = 1,
	BITS13_0	   = 2,
	BITS12_0	   = 3,
	BITS11_0	   = 4,
	BITS10_0	   = 5,
	BITS9_0	   	   = 6
}ipipeif_data_shift;

typedef enum _clk_sel{
	PIXCEL_CLK	= 0,
	SDRAM_CLK	= 1 	
}ipipeif_clock;

typedef enum _ialaw {
	ALAW_OFF	= 0,
	ALAW_ON		= 1
}ipipeif_ialaw;

typedef enum  _pack_mode {
	SIXTEEN_BIT	= 0,
	EIGHT_BIT	= 1
}ipipeif_pack_mode;

typedef enum   _avg_filter{
	AVG_OFF		= 0,
	AVG_ON		= 1
}ipipeif_avg_filter;

typedef enum _clk_div {
	DIVIDE_HALF		= 0,
	DIVIDE_THIRD		= 1,
	DIVIDE_FOURTH		= 2,
	DIVIDE_FIFTH		= 3,
	DIVIDE_SIXTH		= 4,
	DIVIDE_EIGHTH		= 5,
	DIVIDE_SIXTEENTH	= 6,
	DIVIDE_THIRTY		= 7
	
}ipipeif_clkdiv;

typedef enum   _data_selection{
	CCDC		= 0,
	SDRAM_RAW	= 1,
	CCDC_DARKFM	= 2,
	SDRAM_YUV	= 3
}ipipeif_input_source;

typedef enum   _decimation {
	DECIMATION_OFF	= 0,
	DECIMATION_ON	= 1
}ipipeif_decimation;

typedef enum  _mode { 
	CONTINUOUS	= 0,
	ONE_SHOT	= 1
}operation_mode;

typedef enum _RSZ{
	ONE		= 16,
	ONE_HALF	= 32,
	ONE_THIRD	= 48,
	ONE_FOURTH	= 64,
	ONE_FIFTH	= 80,
	ONE_SIXTH	= 96,
	ONE_SEVENTH	= 112
	
}ipipeif_rsz_ratio;



/*struct ipipe_cropsize {
	int hcrop;
        int vcrop;
};
*/
/* Defect Correction */
struct ipipe_def_cor {
	enable_disable_t dfc_en;
	copy_method_t dfc_sel;

        unsigned int dfc_siz;
	unsigned int dfc_adr;	
	unsigned int *dfc_table;
};
/* 
 * list of structures 
 */
/* structure for request buffer */

struct ipipe_reqbufs {
        int buf_type;   /* type of frame buffer */
        unsigned int size;       /* size of the frame buffer to be allocated */
        int count;      /* number of frame buffer to be allocated */
};	
/* structure buffer */
struct ipipe_buffer {
        int index;      /* index number, 0 -> N-1 */
        int buf_type;   /* buffer type, input or output */
        unsigned int offset;     /* address of the buffer used in the mmap()
                           system call */
        unsigned int size;       /* size of the buffer */
};


/* Programmable Noise Filter */
struct ipipe_prog_nf {   
	enable_disable_t noise_fil_en;
        unsigned int d2f_cfg_spr;
	unsigned int d2f_cfg_shf;
	sampling_type_t type; 
        unsigned int *d2f_thr;
        unsigned int *d2f_str;
};

/* Prefilter */
struct ipipe_prefilter {
	enable_disable_t pre_en;
	pre_filter_type_t sel_0;
	pre_filter_type_t sel_1;

	enable_disable_t typ_adaptive; 
	enable_disable_t typ_adaptive_dotred;
	unsigned int pre_shf;
	unsigned int pre_gain;
	unsigned int pre_thr_g;
	unsigned int pre_thr_b;
	unsigned int pre_thr_1;
};

/* White Balance */
struct ipipe_wb {
	unsigned int wb2_dgn;
	unsigned int wb2_wg_r;
	unsigned int wb2_wg_gr;
	unsigned int wb2_wg_gb;
	unsigned int wb2_wg_b;
};

/* RGB to RGB conversion (include GAMMA correction) */
struct ipipe_rgb2rgb {
	unsigned int rgb_mul_rr;
	unsigned int rgb_mul_gr;	
	unsigned int rgb_mul_br;	
        unsigned int rgb_mul_rg;
        unsigned int rgb_mul_gg;
        unsigned int rgb_mul_bg;
	unsigned int rgb_mul_rb;
	unsigned int rgb_mul_gb;
	unsigned int rgb_mul_bb;
	unsigned int rgb_oft_or;
	unsigned int rgb_oft_og;
	unsigned int rgb_oft_ob;
	gamma_cor_t gmm_cfg_bypr;
	gamma_cor_t gmm_cfg_bypg;
	gamma_cor_t gmm_cfg_bypb;
	gamma_tbl_t gmm_cfg_tbl;
	gamma_siz_t gmm_cfg_siz;
	/* 
	unsigned int gmm_tbl_r[1024];
	unsigned int gmm_tbl_b[1024];
	unsigned int gmm_tbl_g[1024];
	unsigned int gmm_tbl_all[1024];
	*/
	unsigned int *gmm_tbl_r;
	unsigned int *gmm_tbl_b;
	unsigned int *gmm_tbl_g;
	unsigned int *gmm_tbl_all;
};

typedef enum yuv_phs_pos {
	COSITING =  0,
	CENTERING = 1
}yuv_phs_pos_t;

/* RGB to YUV(YCbCr) conversion */
struct ipipe_rgb2yuv {
	unsigned int yuv_adj_ctr;
	unsigned int yuv_adj_brt;
	
	unsigned int yuv_mul_ry;
	unsigned int yuv_mul_gy;
	unsigned int yuv_mul_by;
	unsigned int yuv_mul_rcb;
	unsigned int yuv_mul_gcb;
	unsigned int yuv_mul_bcb;
	unsigned int yuv_mul_rcr;
	unsigned int yuv_mul_gcr;
	unsigned int yuv_mul_bcr;
	unsigned int yuv_oft_y;
	unsigned int yuv_oft_cb;
	unsigned int yuv_oft_cr;
	unsigned int yuv_y_min;
	unsigned int yuv_y_max;
	unsigned int yuv_c_min;
	unsigned int yuv_c_max;
	enable_disable_t yuv_phs_lpf;
	yuv_phs_pos_t yuv_phs_position;	
};

/* Edge Enhancer */ 
struct ipipe_edge_enhancer {
	enable_disable_t yee_en;
	enable_disable_t yee_emf;
	unsigned int yee_shf;	
	unsigned int yee_mul_00;
	unsigned int yee_mul_01;
	unsigned int yee_mul_02;
	unsigned int yee_mul_10;
	unsigned int yee_mul_11;
	unsigned int yee_mul_12;
	unsigned int yee_mul_20;	
	unsigned int yee_mul_21;
	unsigned int yee_mul_22;
	unsigned int *ee_table;
};

/* False Color Suppression */
struct ipipe_false_color_suppresion {
	enable_disable_t fcs_en;
	fcs_typ_t fcs_typ_typ;	
	unsigned int fcs_shf_y;
	unsigned int fcs_shf_c;
	unsigned int fcs_thr;
	unsigned int fcs_sgn;
	unsigned int fcs_lth;	
};

/* Resizer Rescale Parameters*/
struct ipipe_resizer_rescale_param {
	unsigned int rsz_mode;
	unsigned int rsz_i_vst;
	unsigned int rsz_i_vsz;
	unsigned int rsz_i_hst;
	unsigned int rsz_o_vsz;
	unsigned int rsz_o_hsz;
        unsigned int rsz_o_hst;
	unsigned int rsz_v_phs;
        //unsigned int rsz_v_phs_o;
	unsigned int rsz_v_dif;
	//unsigned int rsz_v_siz_o;
	unsigned int rsz_h_phs;
	unsigned int rsz_h_dif;
	rsz_h_typ_t rsz_h_typ;
	rsz_h_lse_t rsz_h_lse_sel;
	unsigned int rsz_h_lpf;
}; 

/* Resizer RGB Conversion Parameters */
struct ipipe_resize2rgb {
	enable_disable_t rsz_rgb_en;
	ipipe_rsz_rgb_typ_t rsz_rgb_typ;
	ipipe_rsz_rgb_msk_t rsz_rgb_msk0;
	ipipe_rsz_rgb_msk_t rsz_rgb_msk1;
//	unsigned int rsz_rgb_bld;
	unsigned int rsz_rgb_alpha_val;
};

/* Resizer External Memory Parameters */
struct ipipe_ext_mem_param {
	unsigned int rsz_sdr_bad_h;
	unsigned int rsz_sdr_bad_l;
	unsigned int rsz_sdr_sad_h;
	unsigned int rsz_sdr_sad_l;	
	unsigned int rsz_sdr_oft;
	unsigned int rsz_sdr_ptr_s;
	unsigned int rsz_sdr_ptr_e;
};	
/*ipipeif structures*/
struct ipipeif {
	/*IPPEIF config register*/
	ipipeif_data_shift data_shift;
	ipipeif_clock clock_select;
	
	ipipeif_ialaw ialaw;
	ipipeif_pack_mode pack_mode;
	ipipeif_avg_filter avg_filter;
	ipipeif_clkdiv clk_div;	
	ipipeif_input_source source;
	ipipeif_decimation decimation;
	operation_mode mode;
	
	unsigned int glob_hor_size;
	unsigned int glob_ver_size;
	unsigned int hnum;
	unsigned int vnum;
	unsigned int adofs;
	ipipeif_rsz_ratio rsz;
	unsigned int gain;
};
/* structure for all configurations */ 
struct ipipe_params {
	struct ipipeif ipipeif_param;
	
	operation_mode ipipe_mode;
	/*input/output datapath register*/
	ipipe_dpaths_fmt_t ipipe_dpaths_fmt;
	ipipe_dpaths_bypass_t ipipe_dpaths_bypass;
	
	/*color pattern register*/
	ipipe_colpat_t ipipe_colpat_elep;
        ipipe_colpat_t ipipe_colpat_elop;
        ipipe_colpat_t ipipe_colpat_olep;
        ipipe_colpat_t ipipe_colpat_olop;

	/*horizontal/vertical start, horizontal/vertical size*/
	unsigned int ipipe_vst;
	unsigned int ipipe_vsz;
	unsigned int ipipe_hst;
	unsigned int ipipe_hsz;
	/*interupt generation after lines*/
	
	struct ipipe_def_cor def_cor; 
	struct ipipe_prog_nf prog_nf;
	struct ipipe_prefilter prefilter; 
	struct ipipe_wb wb; 	
	struct ipipe_rgb2rgb rgb2rgb;
	struct ipipe_rgb2yuv rgb2yuv; 
 	struct ipipe_edge_enhancer edge_enhancer; 	
	struct ipipe_false_color_suppresion false_color_suppresion;
	
	enable_disable_t rsz_seq_seq;
	enable_disable_t rsz_seq_tmm;
	enable_disable_t rsz_seq_hrv;
	enable_disable_t rsz_seq_vrv;
	enable_disable_t rsz_seq_crv;
	
	enable_disable_t rsz_aal;

	struct ipipe_resizer_rescale_param rsz_rsc_param[2];	
	struct ipipe_resize2rgb rsz2rgb[2];	
	struct ipipe_ext_mem_param ext_mem_param[2];	

	enable_disable_t rsz_en[2];
};
struct ipipe_convert {
        struct ipipe_buffer in_buff;
        struct ipipe_buffer out_buff;
};
#ifdef __KERNEL__
/* device structure keeps track of global information */
struct ipipe_device {
        struct ipipe_params *params;
        unsigned char opened;		/* state of the device */
        unsigned char in_numbuffers;	/* number of input buffers */
        unsigned char out_numbuffers;	/* number of output buffers */
        struct ipipe_buffer *in_buff[MAX_BUFFER];  /*pointer to input buffers*/
        struct ipipe_buffer *out_buff[MAX_BUFFER]; /*pointer to output buffers */
        struct completion wfc;/*used to wait for frame precessing to be completed*/
        struct semaphore sem;
};

int ipipe_hw_setup(struct ipipe_params *config);
int default_for_raw2raw(struct ipipe_params* parameter);
int default_for_bypass(struct ipipe_params* parameter);
int set_dfc_regs(struct ipipe_def_cor *dfc);
int set_d2f_regs( struct ipipe_prog_nf *noise_filter);
int set_pre_regs(struct ipipe_prefilter *pre_amplifier);
int set_wb_regs(struct ipipe_wb *white_balance);
int set_rgb_2_yuv_regs(int data_format,struct ipipe_rgb2yuv *y_cr_cb);
int set_rgb_to_rgb_regs(struct ipipe_rgb2rgb *rgb);
int set_ee_regs(struct ipipe_edge_enhancer *edge_enhance);
int set_fcs_regs(struct ipipe_false_color_suppresion *color_supress);
int set_rsz_regs(struct ipipe_params *param_resize);
int set_aal_regs(struct ipipe_params *param_resize);
int set_rsz_structs(struct ipipe_params *params );
int write_out_addr(int resize_no,unsigned int address);

int ipipe(struct ipipe_device *device, struct ipipe_convert *);
int request_buffer(struct ipipe_device*, struct ipipe_reqbufs*);
int query_buffer(struct ipipe_device*, struct ipipe_buffer*);
irqreturn_t ipipe_isr(int, void*, struct pt_regs*);
int free_buffers(struct ipipe_device*);
int validate_params(struct ipipe_params*);
#endif	/* End of #ifdef __KERNEL__ */
/* ioctls definition */
#define IPIPE_IOC_BASE   	'P'
#define IPIPE_REQBUF     	_IOW(IPIPE_IOC_BASE, 1, struct ipipe_reqbufs)
#define IPIPE_QUERYBUF   	_IOR(IPIPE_IOC_BASE, 2, struct ipipe_buffer)
#define IPIPE_SET_PARAM  	_IOWR(IPIPE_IOC_BASE, 3, struct ipipe_params*)
#define IPIPE_GET_PARAM  	_IOWR(IPIPE_IOC_BASE, 4, struct ipipe_params*)
#define IPIPE_START	    	_IOWR(IPIPE_IOC_BASE,5, char)
/*
*/
#define IPIPE_IOC_MAXNR 5 
/* End of ioctls */

#ifdef __KERNEL__
struct vm_struct_area;
struct inode;
struct file;
/* function definition for character driver interface functions */
int ipipe_init(void);
void ipipe_cleanup(void);
int ipipe_open(struct inode *inode, struct file *);
int ipipe_release(struct inode *inode, struct file *);
int ipipe_ioctl(struct inode *inode, struct file *, unsigned int,
		    unsigned long);
int ipipe_mmap(struct file *, struct vm_area_struct *);

#endif	/* End of #ifdef __KERNEL__ */

#endif	/* End of DM355_IPIPE_H */
