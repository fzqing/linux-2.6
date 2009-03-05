/*
 * Copyright (C) 2003-2004 Marvell International Ltd.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 */

#ifndef CPU_FREQ_VOLTAGE_MHN_H
#define CPU_FREQ_VOLTAGE_MHN_H

#ifdef __KERNEL__
enum {
	FV_NOTIFIER_QUERY_SET = 1,
	FV_NOTIFIER_PRE_SET = 2,
	FV_NOTIFIER_POST_SET = 3,
};
#endif

/*
 * frequency and voltage change mode
 */
typedef unsigned int pm_fv_mode_t;

/* Driver clock from System PLL during Xscale PLL frequency change */
typedef pm_fv_mode_t FV_MODE;
#define FV_MODE_SET_CFS_XSPCLK_OFFSET	0
#define FV_MODE_SET_CFS_XSPCLK_MASK	0x3

/* Drive 156M clock from System PLL when core freq change */
#define	FV_MODE_SET_CFS_XSPCLK_156M	0
/* Drive 208M clock from System PLL when core freq change */
#define	FV_MODE_SET_CFS_XSPCLK_208M	1
/* Drive 312M clock from System PLL when core freq change */
#define	FV_MODE_SET_CFS_XSPCLK_312M	2
/* No clock to core until core freq change */
#define	FV_MODE_SET_CFS_XSPCLK_DIS	3

/* Set SMCFS clock in ACCR */
#define FV_ACCR_SMCFS_78M	0
#define FV_ACCR_SMCFS_104M	2
#define FV_ACCR_SMCFS_208M	5

/* Set SFLFS clock in ACCR */
#define FV_ACCR_SFLFS_104M	0
#define FV_ACCR_SFLFS_156M	1
#define FV_ACCR_SFLFS_208M	2
#define FV_ACCR_SFLFS_312M	3

/* Set HSS clock in ACCR */
#define FV_ACCR_HSS_104M	0
#define FV_ACCR_HSS_156M	1
#define FV_ACCR_HSS_208M	2

/* Set DMSFS in ACCR */
#define FV_ACCR_DMCFS_26M	0
#define FV_ACCR_DMCFS_260M	3

/* Set df_clkdiv in MEMCLKCFG */
#define FV_DF_CLKDIV_1 		1
#define FV_DF_CLKDIV_2 		2
#define FV_DF_CLKDIV_4 		3

/* set empi_clkdiv in MEMCLKCFG  */
#define FV_EMPI_CLKDIV_1 	1
#define FV_EMPI_CLKDIV_2 	2
#define FV_EMPI_CLKDIV_4 	3

#define	DVFM_MAX_OP		20

/* ACCR macros*/
#define ACCR_XL_MASK    	0x0000001f
#define ACCR_XN_MASK    	0x00000700
#define ACCR_DMCFS_MASK      	0x00003000
#define ACCR_HSS_MASK 		0x0000c000
#define ACCR_XSPCLK_MASK   	0x00030000
#define ACCR_SFLFS_MASK		0x000c0000
#define ACCR_13MEND2_MASK       0x00200000
#define ACCR_SMCFS_MASK         0x03800000
#define ACCR_D0CS_MASK          0x04000000
#define ACCR_13MEND1_MASK       0x08000000
#define ACCR_SPDIS_MASK         0x40000000
#define ACCR_XPDIS_MASK         0x80000000
#define ACCR_SPCLK_MASK         0x10000000
#define ACCR_XPCLK_MASK         0x20000000

#define ACCR_G_XL(accr)			((accr) & 0x1F)
#define ACCR_G_XN(accr)			(((accr) >> 8) & 0x7)
#define ACCR_G_SMCFS(accr)		(((accr) >> 23)  & 0x7)
#define ACCR_G_SFLFS(accr)		(((accr) >> 18)  & 0x3)
#define ACCR_G_HSS(accr)		(((accr) >> 14)  & 0x3)
#define ACCR_G_DMCFS(accr)		(((accr) >> 12)  & 0x3)
#define ACCR_G_XSPCLK(accr)		(((accr) >> 16)  & 0x3)
#define ACCR_G_D0CS(accr)          (((accr) >> 26)  & 0x1)
#define ACCR_G_SPDIS(accr)          (((accr) >> 30)  & 0x1)
#define ACCR_G_XPDIS(accr)          (((accr) >> 31)  & 0x1)
#define ACCR_G_D0CS(accr)          (((accr) >> 26)  & 0x1)
#define ACCR_S_XL(accr, xl)		(((accr) & ~0x1F) | xl)
#define ACCR_S_XN(accr, xn)		((((accr) & ~(0x7 << 8)) | ((xn) << 8)))
#define ACCR_S_SMCFS(accr, smcfs)	((((accr) & ~(0x7 << 23)) | ((smcfs) << 23)))
#define ACCR_S_SFLFS(accr, sflfs)	((((accr) & ~(0x3 << 18)) | ((sflfs) << 18)))
#define ACCR_S_HSS(accr, hss)		((((accr) & ~(0x3 << 14)) | ((hss) << 14)))
#define ACCR_S_DMCFS(accr, dmcfs)	((((accr) & ~(0x3 << 12)) | ((dmcfs) << 12)))
#define ACCR_S_XSPCLK(accr, xspclk)	((((accr) & ~(0x3 << 16)) | ((xspclk) << 16)))
#define ACCR_S_D0CS(accr, d0cs)	((((accr) & ~(0x1 << 26)) | ((d0cs) << 26)))
#define ACCR_S_SPDIS(accr, spdis)	((((accr) & ~(0x1 << 30)) | ((spdis) << 30)))
#define ACCR_S_XPDIS(accr, xpdis)	((((accr) & ~(0x1 << 31)) | ((xpdis) << 31)))
#define XCLKCFG_G_T(clkcfg)	((clkcfg) & 0x1)
#define XCLKCFG_SET(f, t)	(((f) << 0x1) | (t))
#define ACSR_ACCR_MASK				0xc78cf71f

#define MEMCLKCFG_G_DFCLKDIV(memclkcfg)   (((memclkcfg) >> 16)  & 0x7)
#define MEMCLKCFG_G_EMPICLKDIV(memclkcfg) (((memclkcfg) >> 0)  & 0x7)
#define MEMCLKCFG_S_DFCLKDIV(memclkcfg, df_clk)  \
						(((memclkcfg) & ~0x00070000) | ((df_clk) << 16 ))
#define MEMCLKCFG_S_EMPICLKDIV(memclkcfg, empi_clk)  \
						(((memclkcfg) & ~0x00000007) | ((empi_clk) << 0 ))

/* flags to mark those frequencies and voltages to be changed  */
#define FV_SET_CFS	0x0001	/* set core frequency */
#define FV_SET_SMCFS	0x0002
#define FV_SET_SFLFS	0x0004
#define FV_SET_HSS	0x0008
#define FV_SET_DMCFS	0x0010
#define FV_SET_DFCLK	0x0020
#define FV_SET_EMPICLK	0x0040
#define FV_SET_D0CS	0x0080
#define FV_SET_CV	0x8000	/* set VccSram and VccApp */

/*
 * operating point definition
 */
struct mhn_fv_info {
	unsigned long xl;
	unsigned long xn;
	unsigned int vcc_core;
	unsigned int vcc_sram;
	unsigned long smcfs;
	unsigned long sflfs;
	unsigned long hss;
	unsigned long dmcfs;
	unsigned long df_clk;
	unsigned long empi_clk;
	unsigned long d0cs;
	/* WARNING: above fields must be consistent with PM_FV_INFO!!! */

	unsigned long lpj;	/* New value for loops_per_jiffy */
};

#ifdef __KERNEL__
struct mhn_fv_notifier_info {
	struct mhn_fv_info cur;
	struct mhn_fv_info next;
	unsigned int mode;
	/* unsigned int flag; */
};

struct mhn_fv_notifier {
	char *name;
	void *client_data;
	int priority;
	struct list_head notifier_list;
	/* struct list_head deny_list; */
	int ret_code;
	int (*notifier_call) (unsigned int, void *, void *);
};

extern int mhn_fv_register_notifier(struct mhn_fv_notifier *);
extern int mhn_fv_unregister_notifier(struct mhn_fv_notifier *);

extern void mhn_set_hss(unsigned int hss);
extern int mhn_fv_set_op(unsigned int op, unsigned int mode);
extern int mhn_fv_get_op(void);
extern int mhn_fv_get_op_count(void);
extern int mhn_fv_get_op_info(unsigned int op, struct mhn_fv_info *info);
extern int mhn_fv_get_def_op(void);
extern int mhn_fv_add_op(struct mhn_fv_info *info);
extern void mhn_fv_restore(unsigned int saved_op);

#endif

#endif
