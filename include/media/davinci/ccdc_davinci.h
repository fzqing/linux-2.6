/*
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
/* ccdc_davinci.h */

#ifndef CCDC_DAVINCI_H
#define CCDC_DAVINCI_H
#include <asm/io.h>
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
#define NUM_EXTRAPIXELS	    8
#define NUM_EXTRALINES		8

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
	BITS_15_6 = 0,
	BITS_14_5,
	BITS_13_4,
	BITS_12_3,
	BITS_11_2,
	BITS_10_1,
	BITS_09_0
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

/* structure for fault pixel correction */
struct fault_pixel {
	bool fpc_enable;	/*Enable or Disable fault pixel correction */
	unsigned short fp_num;	/*Number of fault pixel */
	unsigned int fpc_table_addr;	/*Address of fault pixel table */

};

/* Structure for CCDC configuration parameters for raw capture mode passed 
 * by application
 */
typedef struct ccdc_config_params_raw {

	ccdc_pinpol fid_pol;	/* field id polarity */
	ccdc_pinpol vd_pol;	/* vertical sync polarity */
	ccdc_pinpol hd_pol;	/* horizontal sync polarity */
	bool image_invert_enable;	/* enable to store the image in inverse
					   order in memory(bottom to top) */
	enum data_size data_sz;	/* data size value from 8 to 16 bits */
	struct a_law alaw;	/* Structure for Optional A-Law */
	struct black_clamp blk_clamp;	/* Structure for Optical Black Clamp */
	struct black_compensation blk_comp;	/* Structure for 
						   Black Compensation */
	struct fault_pixel fault_pxl;	/* Structure for Fault Pixel 
					   Module Configuration */
} ccdc_config_params_raw;

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
	bool image_invert_enable;	/* enable to store the image in inverse 
					   order in memory(bottom to top) */
	enum data_size data_sz;	/* data size value from 8 to 16 bits */
	struct a_law alaw;	/* Structure for Optional A-Law */
	struct black_clamp blk_clamp;	/* Structure for Optical Black Clamp */
	struct black_compensation blk_comp;	/* Structure for 
						   Black Compensation */
	struct fault_pixel fault_pxl;	/* Structure for Fault Pixel 
					   Module Configuration */
} ccdc_params_raw;

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
#define PID                             (0x0)
#define PCR                             (0x4)
#define SYN_MODE                        (0x8)
#define HD_VD_WID                       (0xc)
#define PIX_LINES                       (0x10)
#define HORZ_INFO                       (0x14)
#define VERT_START                      (0x18)
#define VERT_LINES                      (0x1c)
#define CULLING                         (0x20)
#define HSIZE_OFF                       (0x24)
#define SDOFST                          (0x28)
#define SDR_ADDR                        (0x2c)
#define CLAMP                           (0x30)
#define DCSUB                           (0x34)
#define COLPTN                          (0x38)
#define BLKCMP                          (0x3c)
#define FPC                             (0x40)
#define FPC_ADDR                        (0x44)
#define VDINT                           (0x48)
#define ALAW                            (0x4c)
#define REC656IF                        (0x50)
#define CCDCFG                          (0x54)
#define FMTCFG                          (0x58)
#define FMT_HORZ                        (0x5c)
#define FMT_VERT                        (0x60)
#define FMT_ADDR0                       (0x64)
#define FMT_ADDR1                       (0x68)
#define FMT_ADDR2                       (0x6c)
#define FMT_ADDR3                       (0x70)
#define FMT_ADDR4                       (0x74)
#define FMT_ADDR5                       (0x78)
#define FMT_ADDR6                       (0x7c)
#define FMT_ADDR7                       (0x80)
#define PRGEVEN_0                       (0x84)
#define PRGEVEN_1                       (0x88)
#define PRGODD_0                        (0x8c)
#define PRGODD_1                        (0x90)
#define VP_OUT                          (0x94)

#define CCDC_IOBASE_VADDR               IO_ADDRESS(0x01c70400)

#define regw(val, reg)    		outl(val, (reg)+CCDC_IOBASE_VADDR)
#define regr(reg)         		inl((reg)+CCDC_IOBASE_VADDR)

#define VPSS_SB_IOBASE_VADDR		IO_ADDRESS(0x01c73400)
#define regw_sb(val, reg)    		outl(val, (reg)+VPSS_SB_IOBASE_VADDR)
#define regr_sb(reg)         		inl((reg)+VPSS_SB_IOBASE_VADDR)

/***************************************************************
*	Define for various register bit mask and shifts for CCDC
****************************************************************/
#define CCDC_FID_POL_MASK			(0x01)
#define CCDC_FID_POL_SHIFT			(4)
#define CCDC_HD_POL_MASK			(0x01)
#define CCDC_HD_POL_SHIFT			(3)
#define CCDC_VD_POL_MASK			(0x01)
#define CCDC_VD_POL_SHIFT			(2)
#define CCDC_HSIZE_OFF_MASK			(0xffffffe0)
#define CCDC_32BYTE_ALIGN_VAL			(31)
#define CCDC_FRM_FMT_MASK			(0x01)
#define CCDC_FRM_FMT_SHIFT			(7  )
#define CCDC_DATA_SZ_MASK			(0x07)
#define CCDC_DATA_SZ_SHIFT			(8)
#define CCDC_PIX_FMT_MASK			(0x03)
#define CCDC_PIX_FMT_SHIFT			(12)
#define CCDC_VP2SDR_DISABLE			(0xFFFBFFFF)
#define CCDC_WEN_ENABLE				(0x01 << 17)
#define CCDC_SDR2RSZ_DISABLE			(0xFFF7FFFF)
#define CCDC_VDHDEN_ENABLE			(0x01 << 16)
#define CCDC_LPF_ENABLE				(0x01 << 14)
#define CCDC_ALAW_ENABLE			(0x01 << 3 )
#define CCDC_ALAW_GAMA_WD_MASK			(0x07)
#define CCDC_BLK_CLAMP_ENABLE			(0x01 << 31)
#define CCDC_BLK_SGAIN_MASK			(0x1F )
#define CCDC_BLK_ST_PXL_MASK			(0x7FFF)
#define CCDC_BLK_ST_PXL_SHIFT			(10)
#define CCDC_BLK_SAMPLE_LN_MASK			(0x07)
#define CCDC_BLK_SAMPLE_LN_SHIFT		(28)
#define CCDC_BLK_SAMPLE_LINE_MASK		(0x07)
#define CCDC_BLK_SAMPLE_LINE_SHIFT		(25)
#define CCDC_BLK_DC_SUB_MASK			(0x03FFF)
#define CCDC_BLK_COMP_MASK			(0x000000FF)
#define CCDC_BLK_COMP_GB_COMP_SHIFT		(8)
#define CCDC_BLK_COMP_GR_COMP_SHIFT		(16)
#define CCDC_BLK_COMP_R_COMP_SHIFT		(24)
#define CCDC_LATCH_ON_VSYNC_DISABLE		(0x01 << 15)
#define CCDC_FPC_ENABLE				(0x01 << 15)
#define CCDC_FPC_DISABLE			(0x0)
#define CCDC_FPC_FPC_NUM_MASK 			(0x7FFF)
#define CCDC_DATA_PACK_ENABLE			(0x01<<11)
#define CCDC_FMTCFG_VPIN_MASK			(0x07)
#define CCDC_FMTCFG_VPIN_SHIFT			(12)
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
#define CCDC_HORZ_INFO_SPH_SHIFT		(16)
#define CCDC_VERT_START_SLV0_SHIFT		(16)
#define CCDC_VDINT_VDINT0_SHIFT			(16)
#define CCDC_VDINT_VDINT1_MASK			(0xFFFF)

/* SBL register and mask defination */
#define SBL_PCR_VPSS				(4)
#define SBL_PCR_CCDC_WBL_O			(0xFF7FFFFF)

#define PPC_RAW					(1)
#define DCSUB_DEFAULT_VAL			(0)
#define CLAMP_DEFAULT_VAL			(0)
#define ENABLE_VIDEO_PORT			(0x00008000)
#define DISABLE_VIDEO_PORT			(0)
#define CCDC_COLPTN_VAL				(0xBB11BB11)
#define TWO_BYTES_PER_PIXEL			(2)
#define INTERLACED_IMAGE_INVERT			(0x4B6D)
#define INTERLACED_NO_IMAGE_INVERT		(0x0249)
#define PROGRESSIVE_IMAGE_INVERT		(0x4000)
#define PROGRESSIVE_NO_IMAGE_INVERT		(0)
#define CCDC_INTERLACED_HEIGHT_SHIFT		(1)

extern void ccdc_reset(void);
extern void ccdc_init(void);
extern void ccdc_cleanup(void);

extern void ccdc_setwin(ccdc_imgwin * image_win, ccdc_frmfmt frm_fmt, int ppc);
void ccdc_config_ycbcr(void);
void ccdc_config_raw(void);
int validate_ccdc_param(ccdc_config_params_raw * ccdcparam);
int ccdc_update_ycbcr_params(void *);
int ccdc_update_raw_params(void *);

/* inline functions that must be fast because they are called frequently */
static inline void ccdc_vdhd_enable(int flag)
{
	unsigned int syn_mode;
	syn_mode = regr(SYN_MODE);
	syn_mode =
	    (flag) ? (syn_mode | CCDC_VDHDEN_ENABLE) : (syn_mode &
							(!CCDC_VDHDEN_ENABLE));

	/* configure internal timing generator */
	regw(syn_mode, SYN_MODE);
}

static inline void ccdc_enable(int flag)
{
	regw(flag, PCR);
}

static inline void ccdc_setfbaddr(unsigned long paddr)
{
	regw(paddr & 0xffffffe0, SDR_ADDR);
}

static inline int ccdc_getfid(void)
{
	int fid = (regr(SYN_MODE) >> 15) & 0x1;
	return fid;
}

static inline int ccdc_sbl_reset(void)
{
	int sb_reset;
	sb_reset = regr_sb(SBL_PCR_VPSS);
	regw_sb((sb_reset & SBL_PCR_CCDC_WBL_O), SBL_PCR_VPSS);
	return sb_reset;
}

static inline void ccdc_enable_vport(void)
{
	/* enable video port */
	regw(ENABLE_VIDEO_PORT, FMTCFG);
}

#endif

#endif				/* CCDC_DAVINCI_H */
