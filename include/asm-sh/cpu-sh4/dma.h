/*
 * linux/include/asm-sh/cpu-sh4/dma.h
 *
 *  Copyright (C)  Takashi Kusuda & Nobuhiro Iwamatsu (June 18, 2005)
 *
*/

#ifndef __ASM_CPU_SH4_DMA_H
#define __ASM_CPU_SH4_DMA_H


#if defined(CONFIG_CPU_SUBTYPE_SH7780)
#define SH_DMAC_BASE0  0xFC808000
#define SH_DMAC_BASE1  0xFC818000

#define SAR    ((unsigned long[]){SH_DMAC_BASE0 + 0x20, SH_DMAC_BASE0 + 0x30, \
                                  SH_DMAC_BASE0 + 0x40, SH_DMAC_BASE0 + 0x50, \
                                  SH_DMAC_BASE0 + 0x70, SH_DMAC_BASE0 + 0x80, \
                                  SH_DMAC_BASE1 + 0x20, SH_DMAC_BASE1 + 0x30, \
                                  SH_DMAC_BASE1 + 0x40, SH_DMAC_BASE1 + 0x50, \
                                  SH_DMAC_BASE1 + 0x70, SH_DMAC_BASE1 + 0x80})
#define DAR    ((unsigned long[]){SH_DMAC_BASE0 + 0x24, SH_DMAC_BASE0 + 0x34, \
                                  SH_DMAC_BASE0 + 0x44, SH_DMAC_BASE0 + 0x54, \
                                  SH_DMAC_BASE0 + 0x74, SH_DMAC_BASE0 + 0x84, \
                                  SH_DMAC_BASE1 + 0x24, SH_DMAC_BASE1 + 0x34, \
                                  SH_DMAC_BASE1 + 0x44, SH_DMAC_BASE1 + 0x54, \
                                  SH_DMAC_BASE1 + 0x74, SH_DMAC_BASE1 + 0x84})
#define DMATCR ((unsigned long[]){SH_DMAC_BASE0 + 0x28, SH_DMAC_BASE0 + 0x38, \
                                  SH_DMAC_BASE0 + 0x48, SH_DMAC_BASE0 + 0x58, \
                                  SH_DMAC_BASE0 + 0x78, SH_DMAC_BASE0 + 0x88, \
                                  SH_DMAC_BASE1 + 0x28, SH_DMAC_BASE1 + 0x38, \
                                  SH_DMAC_BASE1 + 0x48, SH_DMAC_BASE1 + 0x58, \
                                  SH_DMAC_BASE1 + 0x78, SH_DMAC_BASE1 + 0x88})
#define CHCR   ((unsigned long[]){SH_DMAC_BASE0 + 0x2C, SH_DMAC_BASE0 + 0x3C, \
                                  SH_DMAC_BASE0 + 0x4C, SH_DMAC_BASE0 + 0x5C, \
                                  SH_DMAC_BASE0 + 0x7C, SH_DMAC_BASE0 + 0x8C, \
                                  SH_DMAC_BASE1 + 0x2C, SH_DMAC_BASE1 + 0x3C, \
                                  SH_DMAC_BASE1 + 0x4C, SH_DMAC_BASE2 + 0x5C, \
                                  SH_DMAC_BASE1 + 0x7C, SH_DMAC_BASE1 + 0x8C})
#define DMAOR  ((unsigned long[]){SH_DMAC_BASE0 + 0x60, SH_DMAC_BASE1 + 0x60}

#define SARB_0 ((unsigned long[]){SH_DMAC_BASE0 + 0x120, SH_DMAC_BASE0 + 0x130, \
                                  SH_DMAC_BASE0 + 0x140, SH_DMAC_BASE0 + 0x150})
#define SARB_6 ((unsigned long[]){SH_DMAC_BASE1 + 0x120, SH_DMAC_BASE1 + 0x130, \
                                  SH_DMAC_BASE1 + 0x140, SH_DMAC_BASE1 + 0x150})
#define DARB_0 ((unsigned long[]){SH_DMAC_BASE0 + 0x124, SH_DMAC_BASE0 + 0x134, \
                                  SH_DMAC_BASE0 + 0x144, SH_DMAC_BASE0 + 0x154})
#define DARB_6 ((unsigned long[]){SH_DMAC_BASE1 + 0x124, SH_DMAC_BASE1 + 0x134, \
                                  SH_DMAC_BASE1 + 0x144, SH_DMAC_BASE1 + 0x154})
#define TCRB_0 ((unsigned long[]){SH_DMAC_BASE0 + 0x128, SH_DMAC_BASE0 + 0x138, \
                                  SH_DMAC_BASE0 + 0x148, SH_DMAC_BASE0 + 0x158})
#define TCRB_6 ((unsigned long[]){SH_DMAC_BASE1 + 0x128, SH_DMAC_BASE1 + 0x138, \
                                  SH_DMAC_BASE1 + 0x148, SH_DMAC_BASE1 + 0x158})

/* DMA EXT RSC */
#define DMAC_EXT_RSC_0         0xFC809000UL
#define DMAC_EXT_RSC_1         0xFC809004UL
#define DMAC_EXT_RSC_2         0xFC809008UL


#elif defined(CONFIG_CPU_SUBTYPE_SH73180)
#define SH_DMAC_BASE   0xFE008000

#define SAR    ((unsigned long[]){SH_DMAC_BASE + 0x20, SH_DMAC_BASE + 0x30, \
                                  SH_DMAC_BASE + 0x40, SH_DMAC_BASE + 0x50, \
                                  SH_DMAC_BASE + 0x70, SH_DMAC_BASE + 0x80})
#define DAR    ((unsigned long[]){SH_DMAC_BASE + 0x24, SH_DMAC_BASE + 0x34, \
                                  SH_DMAC_BASE + 0x44, SH_DMAC_BASE + 0x54, \
                                  SH_DMAC_BASE + 0x74, SH_DMAC_BASE + 0x84})
#define DMATCR ((unsigned long[]){SH_DMAC_BASE + 0x28, SH_DMAC_BASE + 0x38, \
                                  SH_DMAC_BASE + 0x48, SH_DMAC_BASE + 0x58, \
                                  SH_DMAC_BASE + 0x78, SH_DMAC_BASE + 0x88})
#define CHCR   ((unsigned long[]){SH_DMAC_BASE + 0x2C, SH_DMAC_BASE + 0x3C, \
                                  SH_DMAC_BASE + 0x4C, SH_DMAC_BASE + 0x5C, \
                                  SH_DMAC_BASE + 0x7C, SH_DMAC_BASE + 0x8C})
#define DMAOR  (SH_DMAC_BASE + 0x60)

#else
#define SH_DMAC_BASE	0xffa00000

#define SAR	((unsigned long[]){SH_DMAC_BASE + 0x00, SH_DMAC_BASE + 0x10, \
                                  SH_DMAC_BASE + 0x20, SH_DMAC_BASE + 0x30, \
                                  SH_DMAC_BASE + 0x50, SH_DMAC_BASE + 0x60, \
                                  SH_DMAC_BASE + 0x70, SH_DMAC_BASE + 0x80})
#define DAR	((unsigned long[]){SH_DMAC_BASE + 0x04, SH_DMAC_BASE + 0x14, \
                                  SH_DMAC_BASE + 0x24, SH_DMAC_BASE + 0x34, \
                                  SH_DMAC_BASE + 0x54, SH_DMAC_BASE + 0x64, \
                                  SH_DMAC_BASE + 0x74, SH_DMAC_BASE + 0x84})
#define DMATCR	((unsigned long[]){SH_DMAC_BASE + 0x08, SH_DMAC_BASE + 0x18, \
                                  SH_DMAC_BASE + 0x28, SH_DMAC_BASE + 0x38, \
                                  SH_DMAC_BASE + 0x58, SH_DMAC_BASE + 0x68, \
                                  SH_DMAC_BASE + 0x78, SH_DMAC_BASE + 0x88})
#define CHCR	((unsigned long[]){SH_DMAC_BASE + 0x0C, SH_DMAC_BASE + 0x1C, \
                                  SH_DMAC_BASE + 0x2C, SH_DMAC_BASE + 0x3C, \
                                  SH_DMAC_BASE + 0x5C, SH_DMAC_BASE + 0x6C, \
                                  SH_DMAC_BASE + 0x7C, SH_DMAC_BASE + 0x8C})
#define DMAOR	(SH_DMAC_BASE + 0x40)
#endif

/* DMAOR regster access method */
#if defined(CONFIG_CPU_SUBTYPE_SH7780)
#define DMAOR_ERR_CLEAR        \
{ \
       unsigned long dmaor0 = ctrl_inl(DMAOR[0]); \
       unsigned long dmaor1 = ctrl_inl(DMAOR[1]); \
\
       printk("DMAE: DMAOR0=%lx\n", dmaor0); \
       printk("DMAE: DMAOR1=%lx\n", dmaor1); \
\
       ctrl_outl(ctrl_inl(DMAOR[0]) & ~DMAOR_NMIF, DMAOR[0]); \
       ctrl_outl(ctrl_inl(DMAOR[0]) & ~DMAOR_AE, DMAOR[0]); \
       ctrl_outl(ctrl_inl(DMAOR[0]) | DMAOR_DME, DMAOR[0]); \
\
       ctrl_outl(ctrl_inl(DMAOR[1]) & ~DMAOR_NMIF, DMAOR[1]); \
       ctrl_outl(ctrl_inl(DMAOR[1]) & ~DMAOR_AE, DMAOR[1]); \
       ctrl_outl(ctrl_inl(DMAOR[1]) | DMAOR_DME, DMAOR[1]); \
}

#define DMAOR_INIT             ctrl_outw(DMAOR_DME, DMAOR)
#else

#define DMAOR_ERR_CLEAR        \
{ \
       unsigned long dmaor = ctrl_inl(DMAOR); \
\
       printk("DMAE: DMAOR=%lx\n", dmaor); \
\
       ctrl_outl(ctrl_inl(DMAOR) & ~DMAOR_NMIF, DMAOR); \
       ctrl_outl(ctrl_inl(DMAOR) & ~DMAOR_AE, DMAOR); \
       ctrl_outl(ctrl_inl(DMAOR) | DMAOR_DME, DMAOR); \
}

#if defined(CONFIG_CPU_SUBTYPE_SH7760)
#define DMAOR_INIT             ctrl_outl(DMAOR_DME, DMAOR)
#else
#define DMAOR_INIT             ctrl_outl((0x8000 | DMAOR_DME), DMAOR)
#endif
#endif

/* Definitions for the SuperH DMAC */
#if defined(CONFIG_CPU_SUBTYPE_SH7780)
/* command */
/* CHCR bit */
#define LCKN           (1 << 30)
#define RPT_NORMAL     (0 << 25)
#define RPT_RP_ALL     (1 << 25)
#define RPT_RP_D_T     (2 << 25)
#define RPT_RP_S_T     (3 << 25)
#define RPT_RL_ALL     (5 << 25)
#define RPT_RL_D_T     (6 << 25)
#define RPT_RL_S_T     (7 << 25)
#define OVRRN_DT0      (0 << 23)
#define OVRRN_DT1      (1 << 23)
#define RACK_L         (0 << 22)
#define RACK_H         (1 << 22)
#define TS_8           0
#define TS_16          (1 << 3)
#define TS_32          (2 << 3)
#define TS_128         (3 << 3) /* 16byte */
#define TS_256         (1 << 20) /* 32byte */
#define DMAC_HE                (1 << 19)
#define DMAC_HIE       (1 << 18)
#define ACK_R          (0 << 17)
#define ACK_W          (1 << 17)
#define ACK_L          (0 << 16)
#define ACK_H          (1 << 16)
#define REQ_L_L                (0 << 6)
#define REQ_D_E                (1 << 6)
#define REQ_H_L                (2 << 6)
#define REQ_U_E                (3 << 6)
#define TM_BURST       (1 << 5)

/* DMAOR bit */
#define CMS            (0 << 12)
#define CMS_16         (2 << 12)
#define CMS_64         (3 << 12)

#else
#define REQ_L_L                (0 << 19)
#define REQ_D_E                (1 << 19)
#define RACK_H         (0 << 18)
#define RACK_L         (1 << 18)
#define ACK_R          (0 << 17)
#define ACK_W          (1 << 17)
#define ACK_H          (0 << 16)
#define ACK_L          (1 << 16)
#define TM_BURST       (1 << 7)
#define TS_64          (0 << 4)
#define TS_8           (1 << 4)
#define TS_16          (2 << 4)
#define TS_32          (3 << 4)
#define TS_BLK         (4 << 4)
#endif

#endif /* __ASM_CPU_SH4_DMA_H */

