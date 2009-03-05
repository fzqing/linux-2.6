/*
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
/* ccdc_davinci.h */

#ifndef CCDC_DM355_H
#define CCDC_DM355_H

#ifdef __KERNEL__
#include <asm/io.h>
#endif
#include <linux/videodev.h>

#ifndef TYPES
#define TYPES
typedef unsigned char bool;
#endif

#define TRUE 1
#define FALSE 0

/* Define to enable/disable video port */
#define VIDEO_PORT_ENABLE	(1)

typedef enum ccdc_pixfmt {
	CCDC_PIXFMT_RAW = 0,
	CCDC_PIXFMT_YCBCR_16BIT = 1,
	CCDC_PIXFMT_YCBCR_8BIT = 2
} ccdc_pixfmt;

typedef enum ccdc_frmfmt {
	CCDC_FRMFMT_PROGRESSIVE = 0,
	CCDC_FRMFMT_INTERLACED = 1
} ccdc_frmfmt;

typedef enum ccdc_pinpol {
	CCDC_PINPOL_POSITIVE = 0,
	CCDC_PINPOL_NEGATIVE = 1
} ccdc_pinpol;

/* PIXEL ORDER IN MEMORY from LSB to MSB */
/* only applicable for 8-bit input mode  */
typedef enum ccdc_pixorder {
	CCDC_PIXORDER_CBYCRY = 1,
	CCDC_PIXORDER_YCBYCR = 0
} ccdc_pixorder;

typedef enum ccdc_buftype {
	CCDC_BUFTYPE_FLD_INTERLEAVED,
	CCDC_BUFTYPE_FLD_SEPARATED
} ccdc_buftype;

/* enum for No of pixel per line to be avg. in Black Clamping*/
enum sample_length {
	_1PIXELS = 0,
	_2PIXELS,
	_4PIXELS,
	_8PIXELS,
	_16PIXELS
};

#define FP_NUM_BYTES					(4)
/* Define for extra pixel/line and extra lines/frame */
#define NUM_EXTRAPIXELS	    0
#define NUM_EXTRALINES		0

/* settings for commonly used video formats */
#define VPFE_WIN_NTSC    {0,0,720,480}
#define VPFE_WIN_PAL     {0,0,720,576}
#define VPFE_WIN_NTSC_SP {0,0,640,480}	/* ntsc square pixel */
#define VPFE_WIN_PAL_SP  {0,0,768,576}	/* pal square pixel */
#define VPFE_WIN_CIF     {0,0,352,288}
#define VPFE_WIN_QCIF    {0,0,176,144}
#define VPFE_WIN_QVGA    {0,0,320,240}
#define VPFE_WIN_SIF     {0,0,352,240}

#define VPFE_WIN_VGA	{0,0,(640 + NUM_EXTRAPIXELS),(480 + NUM_EXTRALINES)}
#define VPFE_WIN_SVGA 	{0,0,(800 + NUM_EXTRAPIXELS),(600 + NUM_EXTRALINES)}
#define VPFE_WIN_XGA	{0,0,(1024+ NUM_EXTRAPIXELS),(768 + NUM_EXTRALINES)}
#define VPFE_WIN_480p	{0,0,(720 + NUM_EXTRAPIXELS),(480 + NUM_EXTRALINES)}
#define VPFE_WIN_576p	{0,0,(720 + NUM_EXTRAPIXELS),(576 + NUM_EXTRALINES)}
#define VPFE_WIN_720p 	{0,0,(1280+ NUM_EXTRAPIXELS),(720 + NUM_EXTRALINES)}
#define VPFE_WIN_1080p 	{0,0,(1920),(1080)}

/* enum for No of lines in Black Clamping */
enum sample_line {
	_1LINES = 0,
	_2LINES,
	_4LINES,
	_8LINES,
	_16LINES
};

enum hw_frame {
	CCDC_RAW,
	CCDC_YCBCR
};

/* enum for Alaw gama width */
enum gama_width {
	BITS_13_4 = 0,
	BITS_12_3,
	BITS_11_2,
	BITS_10_1,
	BITS_09_0
};

enum ccdc_datasft {
	NO_SHIFT = 0,
	_1BIT,
	_2BIT,
	_3BIT,
	_4BIT,
	_5BIT,
	_6BIT
};

enum data_size {
	_16BITS = 0,
	_15BITS,
	_14BITS,
	_13BITS,
	_12BITS,
	_11BITS,
	_10BITS,
	_8BITS
};
enum ccdc_mfilt1 {
	NO_MEDIAN_FILTER1 = 0,
	AVERAGE_FILTER1,
	MEDIAN_FILTER1
};

enum ccdc_mfilt2 {
	NO_MEDIAN_FILTER2 = 0,
	AVERAGE_FILTER2,
	MEDIAN_FILTER2
};

typedef struct v4l2_rect ccdc_imgwin;

/* structure for ALaw */
struct a_law {
	bool b_alaw_enable;	/* Enable/disable A-Law */
	enum gama_width gama_wd;	/*Gama Width Input */
};

/* structure for Black Clamping */
struct black_clamp {
	bool b_clamp_enable;
	enum sample_length sample_pixel;	/* only if bClampEnable is TRUE */
	enum sample_line sample_ln;	/* only if bClampEnable is TRUE */
	unsigned short start_pixel;	/* only if bClampEnable is TRUE */
	unsigned short sgain;	/* only if bClampEnable is TRUE */
	unsigned short dc_sub;	/* only if bClampEnable is FALSE */
};

/* structure for Black Level Compensation */
struct black_compensation {
	char r_comp;		/* Constant value to subtract from Red component */
	char gr_comp;		/* Constant value to subtract from Gr component */
	char b_comp;		/* Constant value to subtract from Blue component */
	char gb_comp;		/* Constant value to subtract from Gb component */
};

/*structures for lens shading correction*/

/*gain factor modes*/
typedef enum GfMode {
	u8q8_interpol = 0,
	u16q14_interpol,
	reserved,
	u16q14
} GfMode;

typedef enum gf_table_sel {
	table1 = 0,
	table2,
	table3
} gf_table_sel;

/*LSC configuration structure*/
typedef struct LscCfg {
	GfMode gfmode;
	int gf_table_scaling_fact;
	int gf_table_interval;
	gf_table_sel epel;
	gf_table_sel opel;
	gf_table_sel epol;
	gf_table_sel opol;
} LscCfg;

typedef struct float_ccdc {
	unsigned int int_no;
	unsigned int frac_no;
} float_ccdc;

/*Main structure for lens shading correction*/
struct lens_shading_corr {
	bool lsc_enable;
	LscCfg lsc_config;
	unsigned int lens_center_horz;
	unsigned int lens_center_vert;
	float_ccdc horz_left_coef;
	float_ccdc horz_right_coef;
	float_ccdc ver_low_coef;
	float_ccdc ver_up_coef;
	float_ccdc gf_table1[256];	/*int_no will be always 0 since it is u8q8 */
	float_ccdc gf_table2[128];
	float_ccdc gf_table3[128];
};

/*structure for color space converter*/
struct color_space_converter {
	bool csc_enable;
	int csc_dec_coeff[16];
	int csc_frac_coeff[16];
};

/*supporting structures for data formatter*/

typedef enum FmtMode {
	split = 0,
	combine,
	line_alt_mode
} FmtMode;

typedef enum LNum {
	_1line = 0,
	_2lines,
	_3lines,
	_4lines
} LNum;

typedef enum Line {
	_1stline = 0,
	_2ndline,
	_3rdline,
	_4thline
} Line;

typedef struct FmtpLen {
	unsigned int plen0;
	unsigned int plen1;
	unsigned int plen2;
	unsigned int plen3;
} FmtpLen_s;

typedef struct FmtCfg {
	FmtMode fmtmode;
	LNum lnum;
	unsigned int addrinc;
} FmtCfg_s;

typedef struct FmtAddr_Ptr {
	unsigned int init;
	Line line;
} fmtaddr_ptr_s;

typedef struct FmtPgm_AP {
	unsigned int pgm_aptr;
	bool pgmupdt;
} fmtpgm_ap_s;

/*Main Structure for data formatter*/
struct data_formatter {
	bool fmt_enable;
	FmtCfg_s fmtcfg;
	FmtpLen_s fmtplen;
	unsigned int fmtsph;
	unsigned int fmtlnh;
	unsigned int fmtslv;
	unsigned int fmtlnv;
	unsigned int fmtrlen;
	unsigned int fmthcnt;
	fmtaddr_ptr_s fmtaddr_ptr[8];
	bool pgm_en[32];
	fmtpgm_ap_s fmtpgm_ap[32];
};

/*Structures for Vertical Defect Correction*/
typedef enum VdfCsl {
	normal = 0,
	horz_interpol_sat,
	horz_interpol
} VdfCsl;

typedef enum VdfCuda {
	whole_line_correct = 0,
	upper_disable
} VdfCuda;

typedef enum DfcMwr {
	write_complete = 0,
	write_reg
} DfcMwr;

typedef enum DfcMrd {
	read_complete = 0,
	read_reg
} DfcMrd;

typedef enum DfcMaRst {
	incr_addr = 0,
	clr_addr
} DfcMaRst;

typedef enum DfcmClr {
	clear_complete = 0,
	clear
} DfcmClr;

struct dft_corr_ctl_s {
	VdfCsl vdfcsl;
	VdfCuda vdfcuda;
	unsigned int vdflsft;
};

struct dft_corr_mem_ctl_s {
	DfcMwr dfcmwr;
	DfcMrd dfcmrd;
	DfcMaRst dfcmarst;
	DfcmClr dfcmclr;
};

/*Main Structure for vertical defect correction*/
/*Vertical defect correction can correct upto 16 defects*/
/*if defects less than 16 then pad the rest with 0*/
struct vertical_dft_s {
	bool ver_dft_en;
	bool gen_dft_en;
	unsigned int saturation_ctl;
	struct dft_corr_ctl_s dft_corr_ctl;
	struct dft_corr_mem_ctl_s dft_corr_mem_ctl;
	unsigned int dft_corr_horz[16];
	unsigned int dft_corr_vert[16];
	unsigned int dft_corr_sub1[16];
	unsigned int dft_corr_sub2[16];
	unsigned int dft_corr_sub3[16];
};

struct data_offset {
	unsigned char horz_offset;
	unsigned char vert_offset;
};

/* Structure for CCDC configuration parameters for raw capture mode passed 
 * by application
 */
typedef struct ccdc_config_params_raw {
	ccdc_pixfmt pix_fmt;	/* pixel format */
	ccdc_frmfmt frm_fmt;	/* progressive or interlaced frame */
	ccdc_imgwin win;	/* video window */
	ccdc_pinpol fid_pol;	/* field id polarity */
	ccdc_pinpol vd_pol;	/* vertical sync polarity */
	ccdc_pinpol hd_pol;	/* horizontal sync polarity */
	ccdc_buftype buf_type;	/* interleaved or separated fields */

	enum ccdc_datasft datasft;	/*data shift to be applied before storing */
	enum ccdc_mfilt1 mfilt1;	/*median filter for sdram */
	enum ccdc_mfilt2 mfilt2;	/*median filter for ipipe */
	bool lpf_enable;	/*low pass filter enable/disable */
	bool horz_flip_enable;
	unsigned int ccdc_offset;	/*offset value to be applied to data */
	/*Range is 0 to 1023 */
	int med_filt_thres;	/*Threshold of median filter */
	bool image_invert_enable;	/* enable to store the image in inverse
					   order in memory(bottom to top) */
	enum data_size data_sz;	/* data size value from 8 to 16 bits */
	struct data_offset data_offset_s;	/*horz and vertical data offset */
	struct a_law alaw;	/* Structure for Optional A-Law */
	struct black_clamp blk_clamp;	/* Structure for Optical Black Clamp */
	struct black_compensation blk_comp;	/* Structure for
						   Black Compensation */
	struct vertical_dft_s vertical_dft;	/*struture for vertical Defect Correction
						   Module Configuration */
	struct lens_shading_corr lens_sh_corr;	/*structure for lens shading Correction
						   Module Configuration */
	struct data_formatter data_formatter_r;	/*structure for data formatter
						   Module Configuration */
	struct color_space_converter color_space_con;	/*structure for color space converter
							   Module Configuration */
} ccdc_config_params_raw;

#define VPFE_CMD_CONFIG_CCDC_YCBCR \
			_IOW('V',BASE_VIDIOC_PRIVATE + 1,ccdc_params_ycbcr)
#define VPFE_CMD_CONFIG_CCDC_RAW \
			_IOW('V',BASE_VIDIOC_PRIVATE + 4,ccdc_config_params_raw)

#ifdef __KERNEL__

/* Structure for CCDC configuration parameters for raw capture mode */
typedef struct ccdc_params_raw {

	ccdc_pixfmt pix_fmt;	/* pixel format */
	ccdc_frmfmt frm_fmt;	/* progressive or interlaced frame */
	ccdc_imgwin win;	/* video window */
	ccdc_pinpol fid_pol;	/* field id polarity */
	ccdc_pinpol vd_pol;	/* vertical sync polarity */
	ccdc_pinpol hd_pol;	/* horizontal sync polarity */
	ccdc_buftype buf_type;	/* interleaved or separated fields */

	enum ccdc_datasft datasft;	/*data shift to be applied before storing */
	enum ccdc_mfilt1 mfilt1;	/*median filter for sdram */
	enum ccdc_mfilt2 mfilt2;	/*median filter for ipipe */
	bool lpf_enable;	/*low pass filter enable/disable */
	bool horz_flip_enable;
	unsigned int ccdc_offset;	/*offset value to be applied to data */
	/*Range is 0 to 1023 */
	int med_filt_thres;	/*Threshold of median filter */
	bool image_invert_enable;	/* enable to store the image in inverse 
					   order in memory(bottom to top) */
	enum data_size data_sz;	/* data size value from 8 to 16 bits */
	struct a_law alaw;	/* Structure for Optional A-Law */
	struct data_offset data_offset_s;	/*horz and vertical data offset */
	struct black_clamp blk_clamp;	/* Structure for Optical Black Clamp */
	struct black_compensation blk_comp;	/* Structure for 
						   Black Compensation */
	struct vertical_dft_s vertical_dft;	/*struture for vertical Defect Correction 
						   Module Configuration */
	struct lens_shading_corr lens_sh_corr;	/*structure for lens shading Correction
						   Module Configuration */
	struct data_formatter data_formatter_r;	/*structure for data formatter
						   Module Configuration */
	struct color_space_converter color_space_con;	/*structure for color space converter
							   Module Configuration */

} ccdc_params_raw;

typedef struct ccdc_params_ycbcr {
	ccdc_pixfmt pix_fmt;	/* pixel format */
	ccdc_frmfmt frm_fmt;	/* progressive or interlaced frame */
	ccdc_imgwin win;	/* video window */
	ccdc_pinpol fid_pol;	/* field id polarity */
	ccdc_pinpol vd_pol;	/* vertical sync polarity */
	ccdc_pinpol hd_pol;	/* horizontal sync polarity */
	int bt656_enable;	/* enable BT.656 embedded sync mode */
	ccdc_pixorder pix_order;	/* cb:y:cr:y or y:cb:y:cr in memory */
	ccdc_buftype buf_type;	/* interleaved or separated fields  */

} ccdc_params_ycbcr;

extern ccdc_params_raw ccdc_hw_params_raw;
extern ccdc_params_ycbcr ccdc_hw_params_ycbcr;

static inline enum data_size ccdc_raw_data_size(void)
{
	return ccdc_hw_params_raw.data_sz;
}

static inline bool ccdc_alaw_enable(void)
{
	return ccdc_hw_params_raw.alaw.b_alaw_enable;
}

static inline void ccdc_set_frame_format(enum hw_frame frame_type,
					 ccdc_frmfmt frame_format)
{
	if (frame_type == CCDC_YCBCR)
		ccdc_hw_params_ycbcr.frm_fmt = frame_format;
	else
		ccdc_hw_params_raw.frm_fmt = frame_format;
}

static inline ccdc_frmfmt ccdc_get_frame_format(enum hw_frame frame_type)
{
	if (frame_type == CCDC_YCBCR)
		return ccdc_hw_params_ycbcr.frm_fmt;
	else
		return ccdc_hw_params_raw.frm_fmt;
}

static inline void ccdc_set_buf_type(enum hw_frame frame_type,
				     ccdc_buftype buf_type)
{
	if (frame_type == CCDC_YCBCR)
		ccdc_hw_params_ycbcr.buf_type = buf_type;
	else
		ccdc_hw_params_raw.buf_type = buf_type;
}

static inline ccdc_buftype ccdc_get_buf_type(enum hw_frame frame_type)
{
	if (frame_type == CCDC_YCBCR)
		return ccdc_hw_params_ycbcr.buf_type;
	else
		return ccdc_hw_params_raw.buf_type;
}

static inline void ccdc_get_image_window(enum hw_frame frame_type,
					 ccdc_imgwin * win)
{
	if (frame_type == CCDC_YCBCR)
		*win = ccdc_hw_params_ycbcr.win;
	else
		*win = ccdc_hw_params_raw.win;
}

static inline void ccdc_set_image_window(enum hw_frame frame_type,
					 ccdc_imgwin win)
{
	if (frame_type == CCDC_YCBCR)
		ccdc_hw_params_ycbcr.win = win;
	else
		ccdc_hw_params_raw.win = win;
}

static inline void ccdc_set_pix_order(ccdc_pixorder order)
{
	ccdc_hw_params_ycbcr.pix_order = order;
}
static inline ccdc_pixorder ccdc_get_pix_order(void)
{
	return ccdc_hw_params_ycbcr.pix_order;
}

/**************************************************************************\
* Register OFFSET Definitions
\**************************************************************************/
#define SYNCEN				0x00
#define MODESET				0x04
#define HDWIDTH				0x08
#define VDWIDTH				0x0c
#define PPLN				0x10
#define LPFR				0x14
#define SPH				0x18
#define NPH				0x1c
#define SLV0				0x20
#define SLV1				0x24
#define NLV				0x28
#define CULH				0x2c
#define CULV				0x30
#define HSIZE				0x34
#define SDOFST				0x38
#define STADRH				0x3c
#define STADRL				0x40
#define CLAMP				0x44
#define DCSUB				0x48
#define COLPTN				0x4c
#define BLKCMP0				0x50
#define BLKCMP1				0x54
#define MEDFILT				0x58
#define RYEGAIN				0x5c
#define GRCYGAIN			0x60
#define GBGGAIN				0x64
#define BMGGAIN				0x68
#define OFFSET				0x6c
#define OUTCLIP				0x70
#define VDINT0				0x74
#define VDINT1				0x78
#define RSV0				0x7c
#define GAMMAWD				0x80
#define REC656IF			0x84
#define CCDCFG				0x88
#define FMTCFG				0x8c
#define FMTPLEN				0x90
#define FMTSPH				0x94
#define FMTLNH				0x98
#define FMTSLV				0x9c
#define FMTLNV				0xa0
#define FMTRLEN				0xa4
#define FMTHCNT				0xa8
#define FMT_ADDR_PTR_B			0xac
#define FMT_ADDR_PTR(i)			(FMT_ADDR_PTR_B + (i*4))
#define FMTPGM_VF0			0xcc
#define FMTPGM_VF1			0xd0
#define FMTPGM_AP0			0xd4
#define FMTPGM_AP1			0xd8
#define FMTPGM_AP2			0xdc
#define FMTPGM_AP3                      0xe0
#define FMTPGM_AP4                      0xe4
#define FMTPGM_AP5                      0xe8
#define FMTPGM_AP6                      0xec
#define FMTPGM_AP7                      0xf0
#define LSCCFG1                         0xf4
#define LSCCFG2                         0xf8
#define LSCH0                           0xfc
#define LSCV0                           0x100
#define LSCKH                           0x104
#define LSCKV                           0x108
#define LSCMEMCTL                       0x10c
#define LSCMEMD                         0x110
#define LSCMEMQ                         0x114
#define DFCCTL                          0x118
#define DFCVSAT                         0x11c
#define DFCMEMCTL                       0x120
#define DFCMEM0                         0x124
#define DFCMEM1                         0x128
#define DFCMEM2                         0x12c
#define DFCMEM3                         0x130
#define DFCMEM4                         0x134
#define CSCCTL                          0x138
#define CSCM0                           0x13c
#define CSCM1                           0x140
#define CSCM2                           0x144
#define CSCM3                           0x148
#define CSCM4                           0x14c
#define CSCM5                           0x150
#define CSCM6                           0x154
#define CSCM7                           0x158
#define DATAOFST			0x15c

#define CCDC_IOBASE_VADDR               IO_ADDRESS(0x01c70600)

#define regw(val, reg)    		outl(val, (reg)+CCDC_IOBASE_VADDR)
#define regr(reg)         		inl((reg)+CCDC_IOBASE_VADDR)

#define VPSS_BL_IOBASE_VADDR		IO_ADDRESS(0x01c70000)
#define regw_bl(val, reg)    		outl(val, (reg)+VPSS_BL_IOBASE_VADDR)
#define regr_bl(reg)         		inl((reg)+VPSS_BL_IOBASE_VADDR)

#define VPSS_VPSSCLK_BASE		IO_ADDRESS(0x01c70000)
#define regw_clk(val, reg)		outl(val, (reg)+VPSS_VPSSCLK_BASE)
#define regr_clk(reg)                   inl((reg)+VPSS_VPSSCLK_BASE)

#define CLKCTRL				(0x04)

#define INTSTAT				(0x80C)
#define INTSEL				(0x810)
#define	EVTSEL				(0x814)
#define MEMCTRL				(0x818)
#define CCDCMUX				(0x81C)

/**************************************************************
*	Define for various register bit mask and shifts for CCDC
*
**************************************************************/
#define CCDC_RAW_IP_MODE			(0x00)
#define CCDC_VDHDOUT_INPUT			(0x00)
#define CCDC_YCINSWP_RAW			(0x00 << 4)
#define CCDC_EXWEN_DISABLE 			(0x00)
#define CCDC_DATAPOL_NORMAL			(0x00)
#define CCDC_CCDCFG_FIDMD_LATCH_VSYNC		(0x00)
#define CCDC_CCDCFG_WENLOG_AND			(0x00)
#define CCDC_CCDCFG_TRGSEL_WEN                  (0x00)
#define CCDC_CCDCFG_EXTRG_DISABLE               (0x00)
#define CCDC_CFA_MOSAIC				(0x00)

#define CCDC_VDC_DFCVSAT_MASK			(0x3fff)
#define CCDC_DATAOFST_MASK			(0x0ff)
#define CCDC_DATAOFST_H_SHIFT			(0)
#define CCDC_DATAOFST_V_SHIFT			(8)
#define CCDC_GAMMAWD_CFA_MASK			(0x01)
#define CCDC_GAMMAWD_CFA_SHIFT			(5)
#define CCDC_FID_POL_MASK			(0x01)
#define CCDC_FID_POL_SHIFT			(4)
#define CCDC_HD_POL_MASK			(0x01)
#define CCDC_HD_POL_SHIFT			(3)
#define CCDC_VD_POL_MASK			(0x01)
#define CCDC_VD_POL_SHIFT			(2)
#define CCDC_FRM_FMT_MASK			(0x01)
#define CCDC_FRM_FMT_SHIFT			(7  )
#define CCDC_DATA_SZ_MASK			(0x07)
#define CCDC_DATA_SZ_SHIFT			(8)
#define CCDC_VDHDOUT_MASK                       (0x01)
#define CCDC_VDHDOUT_SHIFT                      (0)
#define CCDC_EXWEN_MASK				(0x01)
#define CCDC_EXWEN_SHIFT			(5)
#define CCDC_RAW_INPUT_MASK                     (0x03)
#define CCDC_RAW_INPUT_SHIFT                    (12)
#define CCDC_PIX_FMT_MASK			(0x03)
#define CCDC_PIX_FMT_SHIFT			(12)
#define CCDC_DATAPOL_MASK			(0x01)
#define CCDC_DATAPOL_SHIFT			(6)
#define CCDC_WEN_ENABLE				(0x01 << 1)
#define CCDC_VDHDEN_ENABLE			(0x01 << 16)
#define CCDC_LPF_ENABLE				(0x01 << 14)
#define CCDC_ALAW_ENABLE			(0x01)
#define CCDC_ALAW_GAMA_WD_MASK			(0x07)

#define CCDC_FMTCFG_FMTMODE_MASK 		(0x03)
#define CCDC_FMTCFG_FMTMODE_SHIFT		(1)
#define CCDC_FMTCFG_LNUM_MASK			(0x03)
#define CCDC_FMTCFG_LNUM_SHIFT			(4)
#define CCDC_FMTCFG_ADDRINC_MASK		(0x07)
#define CCDC_FMTCFG_ADDRINC_SHIFT		(8)

#define CCDC_CCDCFG_FIDMD_SHIFT			(6)
#define	CCDC_CCDCFG_WENLOG_SHIFT		(8)
#define CCDC_CCDCFG_TRGSEL_SHIFT                (9)
#define CCDC_CCDCFG_EXTRG_SHIFT                 (10)
#define CCDC_CCDCFG_MSBINVI_SHIFT               (13)

#define CCDC_HSIZE_FLIP_SHIFT			(12)
#define CCDC_HSIZE_FLIP_MASK			(0x01)

#define START_PX_HOR_MASK			(0x7FFF)
#define NUM_PX_HOR_MASK				(0x7FFF)
#define START_VER_ONE_MASK			(0x7FFF)
#define START_VER_TWO_MASK			(0x7FFF)
#define NUM_LINES_VER				(0x7FFF)

#define CCDC_BLK_CLAMP_ENABLE			(0x01 << 15)
#define CCDC_BLK_SGAIN_MASK			(0x1F )
#define CCDC_BLK_ST_PXL_MASK			(0x1FFF)
#define CCDC_BLK_SAMPLE_LN_MASK			(0x03)
#define CCDC_BLK_SAMPLE_LN_SHIFT		(13)

#define CCDC_NUM_LINE_CALC_MASK			(0x03)
#define CCDC_NUM_LINE_CALC_SHIFT		(14)

#define CCDC_BLK_DC_SUB_MASK			(0x03FFF)
#define CCDC_BLK_COMP_MASK			(0x000000FF)
#define CCDC_BLK_COMP_GB_COMP_SHIFT		(8)
#define CCDC_BLK_COMP_GR_COMP_SHIFT		(0)
#define CCDC_BLK_COMP_R_COMP_SHIFT		(8)
#define CCDC_LATCH_ON_VSYNC_DISABLE		(0x01 << 15)
#define CCDC_LATCH_ON_VSYNC_ENABLE		(0x00 << 15)
#define CCDC_FPC_ENABLE				(0x01 << 15)
#define CCDC_FPC_FPC_NUM_MASK 			(0x7FFF)
#define CCDC_DATA_PACK_ENABLE			(0x01<<11)
#define CCDC_FMT_HORZ_FMTLNH_MASK		(0x1FFF)
#define CCDC_FMT_HORZ_FMTSPH_MASK		(0x1FFF)
#define CCDC_FMT_HORZ_FMTSPH_SHIFT		(16 )
#define CCDC_FMT_VERT_FMTLNV_MASK		(0x1FFF)
#define CCDC_FMT_VERT_FMTSLV_MASK		(0x1FFF)
#define CCDC_FMT_VERT_FMTSLV_SHIFT		(16 )
#define CCDC_VP_OUT_VERT_NUM_MASK		(0x3FFF)
#define CCDC_VP_OUT_VERT_NUM_SHIFT		(17)
#define CCDC_VP_OUT_HORZ_NUM_MASK		(0x1FFF)
#define CCDC_VP_OUT_HORZ_NUM_SHIFT		(4)
#define CCDC_VP_OUT_HORZ_ST_MASK		(0x000F)

#define CCDC_CSC_COEFF_SHIFT			(8)
#define CCDC_CSC_COEFF_DEC_MASK			(0x0007)
#define CCDC_CSC_COEFF_FRAC_MASK		(0x001F)
#define CCDC_CSC_DEC_SHIFT			(5)
#define CCDC_CSC_ENABLE                         (0x01)
#define CCDC_MFILT1_SHIFT			(10)
#define CCDC_MFILT2_SHIFT			(8)
#define CCDC_LPF_MASK				(0x01)
#define CCDC_LPF_SHIFT				(14)
#define CCDC_OFFSET_MASK			(0x3FF)
#define CCDC_DATASFT_MASK			(0x07)
#define CCDC_DATASFT_SHIFT			(8)
#define CCDC_DF_ENABLE				(0x01)

#define CCDC_FMTPLEN_P0_MASK			(0x000F)
#define CCDC_FMTPLEN_P1_MASK			(0x000F)
#define CCDC_FMTPLEN_P2_MASK			(0x0007)
#define CCDC_FMTPLEN_P3_MASK			(0x0007)
#define CCDC_FMTPLEN_P0_SHIFT			(0)
#define CCDC_FMTPLEN_P1_SHIFT			(4)
#define CCDC_FMTPLEN_P2_SHIFT			(8)
#define CCDC_FMTPLEN_P3_SHIFT			(12)

#define CCDC_FMTSPH_MASK			(0x01FFF)
#define CCDC_FMTLNH_MASK                        (0x01FFF)
#define CCDC_FMTSLV_MASK                        (0x01FFF)
#define CCDC_FMTLNV_MASK                        (0x07FFF)
#define CCDC_FMTRLEN_MASK                       (0x01FFF)
#define CCDC_FMTHCNT_MASK                       (0x01FFF)

#define CCDC_ADP_INIT_MASK			(0x01FFF)
#define CCDC_ADP_LINE_SHIFT			(13)
#define CCDC_ADP_LINE_MASK			(0x0003)
#define CCDC_FMTPGN_APTR_MASK			(0x0007)

#define CCDC_DFCCTL_GDFCEN_MASK			(0x01)
#define CCDC_DFCCTL_VDFCEN_MASK			(0x01)
#define CCDC_DFCCTL_VDFCEN_SHIFT		(4)
#define CCDC_DFCCTL_VDFCSL_MASK			(0x03)
#define CCDC_DFCCTL_VDFCSL_SHIFT		(5)
#define CCDC_DFCCTL_VDFCUDA_MASK		(0x01)
#define CCDC_DFCCTL_VDFCUDA_SHIFT		(7)
#define CCDC_DFCCTL_VDFLSFT_MASK                (0x03)
#define CCDC_DFCCTL_VDFLSFT_SHIFT               (8)
#define CCDC_DFCMEMCTL_DFCMARST_MASK		(0x01)
#define CCDC_DFCMEMCTL_DFCMARST_SHIFT		(2)
#define CCDC_DFCMEMCTL_DFCMWR_MASK		(0x01)
#define CCDC_DFCMEMCTL_DFCMWR_SHIFT		(0)

#define CCDC_LSCCFG_GFTSF_MASK			(0x07)
#define CCDC_LSCCFG_GFTSF_SHIFT			(1)
#define CCDC_LSCCFG_GFTINV_MASK                 (0x0f)
#define CCDC_LSCCFG_GFTINV_SHIFT                (4)
#define CCDC_LSC_GFTABLE_SEL_MASK		(0x03)
#define CCDC_LSC_GFTABLE_EPEL_SHIFT		(8)
#define CCDC_LSC_GFTABLE_OPEL_SHIFT		(10)
#define CCDC_LSC_GFTABLE_EPOL_SHIFT		(12)
#define CCDC_LSC_GFTABLE_OPOL_SHIFT		(14)
#define CCDC_LSC_GFMODE_MASK			(0x03)
#define CCDC_LSC_GFMODE_SHIFT			(4)
#define CCDC_LSC_DISABLE			(0)
#define CCDC_LSC_ENABLE				(1)
#define CCDC_LSC_TABLE1_SLC			(0)
#define CCDC_LSC_TABLE2_SLC			(1)
#define CCDC_LSC_TABLE3_SLC			(2)
#define CCDC_LSC_MEMADDR_RESET			(1<<2)
#define CCDC_LSC_MEMADDR_INCR                   (0<<2)
#define CCDC_LSC_FRAC_MASK_T1			(0xFF)
#define CCDC_LSC_INT_MASK			(0x03)
#define CCDC_LSC_FRAC_MASK                      (0x3FFF)
#define CCDC_LSC_CENTRE_MASK			(0x3FFF)
#define CCDC_LSC_COEF_MASK			(0x0ff)
#define  CCDC_LSC_COEFL_SHIFT			(0)
#define CCDC_LSC_COEFU_SHIFT			(8)

extern void ccdc_reset(void);
extern void ccdc_init(void);
extern void ccdc_cleanup(void);
extern void ccdc_setwin(ccdc_imgwin * image_win, ccdc_frmfmt frm_fmt, int ppc);
void ccdc_config_raw(void);
void ccdc_config_ycbcr(void);
int validate_ccdc_param(ccdc_config_params_raw * ccdcparam);
int ccdc_update_ycbcr_params(void *);
int ccdc_update_raw_params(void *);
static inline void ccdc_enable_vport(void)
{
	/* Do nothing for DM355 */
}

static inline int ccdc_sbl_reset(void)
{
	return 0;
}

/* inline functions that must be fast because they are called frequently */
static inline void ccdc_enable(int flag)
{
	if (flag)
		flag = 3;
	else
		flag = 0;
	regw(flag, SYNCEN);
}

/*
static inline void sdramop_enable(int flag){
        unsigned int temp;
	temp = regr(SYNCEN);
	temp |= (flag & 0x01)<<1;
	regw(temp, SYNCEN);
}
*/
static inline void ccdc_setfbaddr(unsigned long paddr)
{
	regw((paddr >> 21) & 0x007f, STADRH);
	regw((paddr >> 5) & 0x0ffff, STADRL);
}

static inline int ccdc_getfid(void)
{
	int fid = (regr(MODESET) >> 15) & 0x1;
	return fid;
}

/*
static inline int ccdc_sbl_reset(void)
{
	int sb_reset;
	sb_reset = regr_sb(4);
	regw_sb((sb_reset & 0xFF7FFFFF), 4);
	return sb_reset;
}*/

#endif

#endif				/* CCDC_DM355_H */
