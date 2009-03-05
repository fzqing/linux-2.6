/*
 * include/asm-mips/tx4939/tx4939.h
 *
 * Definitions for TX4939
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * Author: source@mvista.com
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#ifndef __ASM_TX4939H
#define __ASM_TX4939H

#include <asm/tx4939/config.h>

#ifndef __ASSEMBLY__

extern struct pci_ops tx4939_pci_ops;
extern struct tx4939_pcic_reg *pcicptrs[4];

extern unsigned int tx4939_get_mem_size(void);

/* TX4939 clocks */
extern unsigned int txx9_cpu_clock;
extern unsigned int txx9_gbus_clock;
extern unsigned int txx9_sys_clock;

/* TX4939 proc entry */
extern struct proc_dir_entry *tx4939_proc_entry;
extern struct pci_controller tx4939_pci_controller[];

#endif                         /* __ASSEMBLY__ */


/* TX4939 REGISTER ADDRESS MAP */

#define TX4939_REG_BASE          0xff1f0000
#define TX4939_REG_SIZE          0x00010000

#define TX4939_ATA_REG(ch)       (TX4939_REG_BASE + 0x3000 + (ch) * 0x1000)
#define TX4939_NDFMC_REG         (TX4939_REG_BASE + 0x5000)
#define TX4939_SRAMC_REG         (TX4939_REG_BASE + 0x6000)
#define TX4939_DDRC_REG          (TX4939_REG_BASE + 0x8000)
#define TX4939_EBUSC_REG         (TX4939_REG_BASE + 0x9000)
#define TX4939_VPC_REG           (TX4939_REG_BASE + 0xa000)
#define TX4939_DMAC_REG(ch)      (TX4939_REG_BASE + 0xb000 + (ch) * 0x800)
#define TX4939_PCIC_REG(ch)      (TX4939_REG_BASE + 0xd000 - (ch) * 0x6000)
#define TX4939_CONFIG_REG        (TX4939_REG_BASE + 0xe000)
#define TX4939_IRC_REG           (TX4939_REG_BASE + 0xe800)
#define TX4939_TMR_REG(ch)       (TX4939_REG_BASE + 0xf000 + (ch) * 0x100 + (((ch)+1) & 0x4) * 0x280)
#define TX4939_SIO_REG(ch)       (TX4939_REG_BASE + 0xf300 + ((ch) & 0x01) * 0x100 + ((ch) & 0x2) * 0x40)
#define TX4939_ACLC_REG          (TX4939_REG_BASE + 0xf700)
#define TX4939_SPIC_REG          (TX4939_REG_BASE + 0xf800)
#define TX4939_I2C_REG           (TX4939_REG_BASE + 0xf900)
#define TX4939_I2S_REG           (TX4939_REG_BASE + 0xfa00)
#define TX4939_RTC_REG           (TX4939_REG_BASE + 0xfb00)
#define TX4939_CIR_REG           (TX4939_REG_BASE + 0xfc00)

/* IRQ NUMBER */
#define TX4939_IRQ_CP0_BEG       0
#define TX4939_IRQ_CP0_END       ( 7 + TX4939_IRQ_CP0_BEG)

#define TX4939_IRQ_USER0         ( 0 + TX4939_IRQ_CP0_BEG)
#define TX4939_IRQ_USER1         ( 1 + TX4939_IRQ_CP0_BEG)
#define TX4939_IRQ_IRC_CP0       ( 2 + TX4939_IRQ_CP0_BEG)
#define TX4939_IRQ_CPU_TIMER     ( 7 + TX4939_IRQ_CP0_BEG)

#define TX4939_IRQ_IRC_BEG       ( 1 + TX4939_IRQ_CP0_END)
#define TX4939_IRQ_IRC_END       (50 + TX4939_IRQ_IRC_BEG)

#define TX4939_IRQ_DDR           ( 1 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_TX49WTOE      ( 2 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_INT(ch)       ( 3 + TX4939_IRQ_IRC_BEG + (ch))
#define TX4939_IRQ_ETHER(ch)     ( 6 + TX4939_IRQ_IRC_BEG + ((ch) * 37))
#define TX4939_IRQ_VIDEO         ( 7 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_CIR           ( 8 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_SIO(ch)       (43 + TX4939_IRQ_IRC_BEG - (((4-ch) & 0x4) >> 2) * 34 + ch)
#define TX4939_IRQ_DMA0(x)       (10 + (x) + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_IRC           (14 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_PDMAC         (15 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_TMR(ch)       (16 + TX4939_IRQ_IRC_BEG + (((ch+1) & 0x4) >> 2) * 29 + ch)
#define TX4939_IRQ_ATA(ch)       (19 + TX4939_IRQ_IRC_BEG + ((ch) & 0x1))
#define TX4939_IRQ_ACLC          (21 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_DMA(ch)       (22 + TX4939_IRQ_IRC_BEG + ((ch) & 0x1))
#define TX4939_IRQ_CIPHER        (26 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_INTA          (27 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_INTB          (28 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_INTC          (29 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_INTD          (30 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_I2C           (33 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_SPI           (34 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_PCIC(ch)      (35 + TX4939_IRQ_IRC_BEG + ((ch) & 0x1))
#define TX4939_IRQ_PCI0ERR       (37 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_PCI0PME       (38 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_NDFMC         (39 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_ACLCPME       (40 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_RTC           (41 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_RSARND        (42 + TX4939_IRQ_IRC_BEG)
#define TX4939_IRQ_I2S           (47 + TX4939_IRQ_IRC_BEG)

/* PCI RESOURCE */
#define TX4939_PCI0_MEM_RESOURCE_START  0x10000000
#define TX4939_PCI0_MEM_RESOURCE_SIZE   0x04000000
#define TX4939_PCI0_IO_RESOURCE_START   0x14000000
#define TX4939_PCI0_IO_RESOURCE_SIZE    0x01000000
#define TX4939_PCI1_MEM_RESOURCE_START  0x15000000
#define TX4939_PCI1_MEM_RESOURCE_SIZE   0x00010000
#define TX4939_PCI1_IO_RESOURCE_START   0x15010000
#define TX4939_PCI1_IO_RESOURCE_SIZE    0x00010000

/* MACRO DEFINITION */

#ifndef __ASSEMBLY__
#include <asm/types.h>

__asm__(".macro\t__sti\n\t"
	".set\tpush\n\t"
	".set\treorder\n\t"
	".set\tnoat\n\t"
	"mfc0\t$1,$12\n\t"
	"ori\t$1,0x1f\n\t"
	"xori\t$1,0x1e\n\t" "mtc0\t$1,$12\n\t" ".set\tpop\n\t" ".endm");

extern __inline__ void asm_wait(void)
{
	__asm__(".set\tmips3\n\t" "wait\n\t" ".set\tmips0");
}

#define reg_rd08(r)    ((u8 )(*((vu8 *)(r))))
#define reg_rd16(r)    ((u16)(*((vu16*)(r))))
#define reg_rd32(r)    ((u32)(*((vu32*)(r))))
#define reg_rd64(r)    ((u64)(*((vu64*)(r))))

#define reg_wr08(r,v)  ((*((vu8 *)(r)))=((u8 )(v)))
#define reg_wr16(r,v)  ((*((vu16*)(r)))=((u16)(v)))
#define reg_wr32(r,v)  ((*((vu32*)(r)))=((u32)(v)))
#define reg_wr64(r,v)  ((*((vu64*)(r)))=((u64)(v)))

#ifdef __BIG_ENDIAN
#include <asm/tx4939/64bit.h>
# define reg_rd64s(r)   tx4939_in64(r)
# define reg_wr64s(r,v) tx4939_out64(v, r)
#else
# define reg_rd64s(r)   ((u64)(*((vu64*)(r))))
# define reg_wr64s(r,v)  ((*((vu64*)(r)))=((u64)(v)))
#endif

typedef volatile __signed char vs8;
typedef volatile unsigned char vu8;

typedef volatile __signed short vs16;
typedef volatile unsigned short vu16;

typedef volatile __signed int vs32;
typedef volatile unsigned int vu32;

typedef s8 s08;
typedef vs8 vs08;

typedef u8 u08;
typedef vu8 vu08;

#if (_MIPS_SZLONG == 64)

typedef volatile __signed__ long vs64;
typedef volatile unsigned long vu64;

#else

typedef volatile __signed__ long long vs64;
typedef volatile unsigned long long vu64;

#endif

#endif				/* __ASSEMBLY__ */

#ifndef _LANGUAGE_ASSEMBLY
#include <asm/byteorder.h>

#define TX4939_MKA(x) ((u32)( ((u32)(TX4939_REG_BASE)) | ((u32)(x)) ))

#define TX4939_RD08( reg      )   (*(vu08*)(reg))
#define TX4939_WR08( reg, val )  ((*(vu08*)(reg))=(val))

#define TX4939_RD16( reg      )   (*(vu16*)(reg))
#define TX4939_WR16( reg, val )  ((*(vu16*)(reg))=(val))

#define TX4939_RD32( reg      )   (*(vu32*)(reg))
#define TX4939_WR32( reg, val )  ((*(vu32*)(reg))=(val))

#define TX4939_RD64( reg      )   (*(vu64*)(reg))
#define TX4939_WR64( reg, val )  ((*(vu64*)(reg))=(val))

#define TX4939_RD( reg      ) TX4939_RD32( reg )
#define TX4939_WR( reg, val ) TX4939_WR32( reg, val )

#endif				/* !__ASSEMBLY__ */

#ifdef __ASSEMBLY__
#define _CONST64(c)     c
#else
#define _CONST64(c)     c##ull

#include <asm/byteorder.h>

#ifdef __BIG_ENDIAN
#define endian_def_l2(e1,e2)    \
        u32 e1,e2
#define endian_def_s2(e1,e2)    \
        u16 e1,e2
#define endian_def_s4(e1,e2,e3,e4)      \
        u16 e1,e2,e3,e4
#define endian_def_sb2(e1,e2,e3)        \
        u16 e1;u8 e2,e3
#define endian_def_b2s(e1,e2,e3)        \
        u8 e1,e2;u16 e3
#define endian_def_b4(e1,e2,e3,e4)      \
        u8 e1,e2,e3,e4
#define endian_def_b8(e1,e2,e3,e4,e5,e6,e7,e8)      \
        u8 e1,e2,e3,e4,e5,e6,e7,e8
#define endian_def_lb4(e1,e2,e3,e4,e5)      \
        u32 e1;u8 e2,e3,e4,e5
#else
#define endian_def_l2(e1,e2)    \
        u32 e2,e1
#define endian_def_s2(e1,e2)    \
        u16 e2,e1
#define endian_def_s4(e1,e2,e3,e4)      \
        u16 e4,e3,e2,e1
#define endian_def_sb2(e1,e2,e3)        \
        u8 e3,e2;u16 e1
#define endian_def_b2s(e1,e2,e3)        \
        u16 e3;u8 e2,e1
#define endian_def_b4(e1,e2,e3,e4)      \
        u8 e4,e3,e2,e1
#define endian_def_b8(e1,e2,e3,e4,e5,e6,e7,e8)      \
        u8 e8,e7,e6,e5,e4,e3,e2,e1
#define endian_def_lb4(e1,e2,e3,e4,e5)      \
        u8 e5,e4,e3,e2;u32 e1
#endif				/* __BIG_ENDIAN */
#endif				/* __ASSEMBLY__ */

/* TX4939 REGISTER STRUCTURE */

#ifndef __ASSEMBLY__

struct tx4939_ata_reg {
	/*
	 * In little endian mode, address of DATA is 0 and Error/Feature
	 * register is 1. So, they don't conflict.
	 *   Gbus Address |Width  |Access Type  |Register Name
	 *   -------------+-------+-------------+--------------
	 *   000          |16     |R/W          |DATA
	 *   001          |8      |R            |Error
	 *   001          |8      |W            |Feature
	 *   ...
	 * In big endian mode, address of DATA and Error/Feature register
	 * is 6. So, they will conflict in __request_region().
	 *   Gbus Address |Width  |Access Type  |Register Name
	 *   -------------+-------+-------------+--------------
	 *   006          |16     |R/W          |DATA
	 *   006          |8      |R            |Error
	 *   006          |8      |W            |Feature
	 *   ...
	 * then in big endian mode, put 7 as address of DATA register in
	 * io_ports[], then mask the address when accessing DATA register.
	 */
	endian_def_b8(status, device, high, mid,
		      low, sector, error, data);
	u64 unused0[127];
	endian_def_b8(unused10, unused11, unused12, unused13,
		      unused14, alt_devctl, unused16, unused17);/* +0x400 */
	u64 unused1[127];
	endian_def_lb4(prd_tbl,
		      unused20, dma_stat, unused21, dma_cmd);	/* +0x800 */
	u64 unused2[127];
	endian_def_s4(unused30, unused31, unused25, sysctl);	/* +0xc00 */
	endian_def_s4(unused32, unused33, xfer_cnt2, xfer_cnt1);
	endian_def_s4(unused34, unused35, unused36, sec_cnt);	/* +0xc10 */
	endian_def_s4(unused37, unused38, unused39, strt_addl);
	endian_def_s4(unused40, unused41, unused42, strt_addu);	/* +0xc20 */
	endian_def_s4(unused43, unused44, unused45, add_ctrl);
	endian_def_s4(unused46, unused47, unused48, lo_bcnt);	/* +0xc30 */
        endian_def_s4(unused49, unused50, unused51, hi_bcnt);
	volatile u64 unused52[9];
        endian_def_s4(unused53, unused54, unused55, pio_acc);	/* +0xc88 */
        endian_def_s4(unused56, unused57, unused58, h_rst_tim);	/* +0xc90 */
        endian_def_s4(unused59, unused60, unused61, int_ctl);
	volatile u64 unused62[3];
        endian_def_s4(unused63, unused64, unused65, pkt_cmd);	/* +0xcb8 */
        endian_def_s4(unused66, unused67, unused68, bxfer_cnth);/* +0xcc0 */
        endian_def_s4(unused69, unused70, unused71, bxfer_cntl);
        endian_def_s4(unused72, unused73, unused74, dev_terr);	/* +0xcd0 */
        endian_def_s4(unused75, unused76, unused77, pkt_xfer_ct);
        endian_def_s4(unused78, unused79, unused80, strt_addr);	/* +0xce0 */
};

struct tx4939_ndfmc_reg {
	endian_def_s4(unused00, unused01, unused02, dtr);
	endian_def_s4(unused03, unused04, unused05, mcr);
	endian_def_s4(unused06, unused07, unused08, sr);
	endian_def_s4(unused09, unused10, unused11, isr);
	endian_def_s4(unused12, unused13, unused14, imr);
	endian_def_s4(unused15, unused16, unused17, spr);
};

struct tx4939_sramc_reg {
	u64 cr;
};

struct tx4939_pcic_reg {
	u32 pciid;
	u32 pcistatus;
	u32 pciccrev;
	u32 pcicfg1;
	u32 p2gm0plbase;	/* +10 */
	u32 p2gm0pubase;
	u32 p2gm1plbase;
	u32 p2gm1pubase;
	u32 p2gm2pbase;		/* +20 */
	u32 p2giopbase;
	u32 unused0;
	u32 pcisid;
	u32 unused1;		/* +30 */
	u32 pcicapptr;
	u32 unused2;
	u32 pcicfg2;
	u32 g2ptocnt;		/* +40 */
	u32 unused3[15];
	u32 g2pstatus;		/* +80 */
	u32 g2pmask;
	u32 pcisstatus;
	u32 pcimask;
	u32 p2gcfg;		/* +90 */
	u32 p2gstatus;
	u32 p2gmask;
	u32 p2gccmd;
	u32 unused4[24];	/* +a0 */
	u32 pbareqport;		/* +100 */
	u32 pbacfg;
	u32 pbastatus;
	u32 pbamask;
	u32 pbabm;		/* +110 */
	u32 pbacreq;
	u32 pbacgnt;
	u32 pbacstate;
	u64 g2pmgbase[3];	/* +120 */
	u64 g2piogbase;
	u32 g2pmmask[3];	/* +140 */
	u32 g2piomask;
	u64 g2pmpbase[3];	/* +150 */
	u64 g2piopbase;
	u32 pciccfg;		/* +170 */
	u32 pcicstatus;
	u32 pcicmask;
	u32 unused5;
	u64 p2gmgbase[3];	/* +180 */
	u64 p2giogbase;
	u32 g2pcfgadrs;		/* +1a0 */
	u32 g2pcfgdata;
	u32 unused6[8];
	u32 g2pintack;
	u32 g2pspc;
	u32 pcidata0;		/* +1d0 */
	u32 pcidata1;
	u32 pcidata2;
	u32 pcidata3;
	u32 p2gm0cfg;		/* +1e0 */
	u32 p2gm1cfg;
	u32 p2gm2cfg;
	u32 unused7[5];
	u64 pdmca;		/* +200 */
	u64 pdmga;
	u64 pdmpa;
	u64 pdmctr;
	u64 pdmcfg;		/* +220 */
	u64 pdmsts;
};

struct tx4939_ddrc_reg {
	struct tx4939_ddrc_ddr_ctl_reg {
		endian_def_s4(unused0, unused1, unused2, ctl);
	} ddr[47];
	u64 unused0[17];
	u64 drwinen;
	u64 drwin[4];
};

struct tx4939_ebusc_reg {
	u64 cr[8];
};

struct tx4939_vpc_reg {
	u32 csr;
	u32 unused0;
	struct tx4939_vpc_ch_reg {
		u32 ctrl_a;
		u32 unused0;
		u32 ctrl_b;
		u32 unused1;
		u32 idesptr;
		u32 unused2;
		u32 cdesptr;
		u32 unused3;
	} ch[3];
	u32 buserr;
};

struct tx4939_dma_reg {
	struct tx4939_dma_ch_reg {
		u64 cha;
		u64 sar;
		u64 dar;
		 endian_def_l2(unused0, cntr);
		 endian_def_l2(unused1, sair);
		 endian_def_l2(unused2, dair);
		 endian_def_l2(unused3, ccr);
		 endian_def_l2(unused4, csr);
	} ch[4];
	u64 unused[9];
	u64 mfdr;
	 endian_def_l2(unused0, mcr);
};

struct tx4939_ccfg_reg {
	u64 ccfg;
	u64 revid;
	u64 pcfg;
	u64 toea;
	u64 clkctr;	/* +20 */
	u64 unused0;
	u64 garbc;
	u64 unused1[2];
	u64 ramp;	/* +48 */
	u64 unused2[2];
	u64 dskwctrl;	/* +60 */
	u64 mclkosc;
	u64 mclkctl;
	u64 unused[17];
	struct tx4939_ccfg_gpio_reg {	/* +100 */
		u64 mr;
		u64 dr;
	} gpio[2];
};

struct tx4939_tmr_reg {
	u32 tcr;
	u32 tisr;
	u32 cpra;
	u32 cprb;
	u32 itmr;
	u32 unused0[3];
	u32 ccdr;
	u32 unused1[3];
	u32 pgmr;
	u32 unused2[3];
	u32 wtmr;
	u32 unused3[43];
	u32 trr;
};

struct tx4939_sio_reg {
	u32 lcr;
	u32 dicr;
	u32 disr;
	u32 cisr;
	u32 fcr;
	u32 flcr;
	u32 bgr;
	u32 tfifo;
	u32 rfifo;
};

struct tx4939_irc_reg {
	u32 den;
	u32 unused0;
	u32 iscipb;
	u32 unused1;
	u32 dm0;
	u32 unused2;
	u32 dm1;
	u32 unused3;
	struct tx4939_irc_irlvl_reg {
		u32 reg;
		u32 unused0;
	} irlvl[16];
	u32 msk;
	u32 unused4;
	u32 edc;
	u32 unused5;
	u32 pnd0;
	u32 unused6;
	u32 cs;
	u32 unused7;
	u32 pnd1;
	u32 unused8;
	u32 dm2;
	u32 unused9;
	u32 dm3;
	u32 unused10;
	u32 dbr0;
	u32 unused11;
	u32 dbr1;
	u32 unused12;
	u32 dben;
	u32 unused13;
	u32 unused14[4];
	u32 flag0;
	u32 unused15;
	u32 flag1;
	u32 unused16;
	u32 pol;
	u32 unused17;
	u32 cnt;
	u32 unused18;
	u32 maskint;
	u32 unused19;
	u32 maskext;
};

struct tx4939_aclc_reg {
	u32 ctlen;
	u32 ctldis;
	u32 regacc;
	u32 unused0;
	u32 intsts;
	u32 intmsts;
	u32 inten;
	u32 intdis;
	u32 semaph;
	u32 unused1[7];
	u32 gpidat;
	u32 gpodat;
	u32 slten;
	u32 sltdis;
	u32 fifosts;
	u32 unused2[11];
	u32 dmasts;
	u32 dmasel;
	u32 unused3[6];
	u32 audodat;
	u32 surrdat;
	u32 centdat;
	u32 lfedat;
	u32 audiat;
	u32 unused4;
	u32 modoat;
	u32 modidat;
	u32 unused5[15];
	u32 revid;
};

struct tx4939_spi_reg {
	u32 mcr;
	u32 cr0;
	u32 cr1;
	u32 fs;
	u32 unused1;
	u32 sr;
	u32 dr;
	u32 unused2;
};

struct tx4939_i2c_reg {
	u32 ictsr;
	u32 txrr;
	u32 cr;
	u32 pre;
	u32 unused0[4];
	u32 ctr;	/* +20 */
	u32 unused1[5];
	u32 btxrr;	/* +34 */
	u32 bcr;
	u32 brbcnt;
};

struct tx4939_i2s_reg {
	u32 mcr;
	u32 ccr;
	u32 ictrl;
	u32 cor;
	u32 unused0[28];
	struct tx4939_i2s_ch_reg {	/* +80 */
		u32 dmaaddr;
		u32 unused[3];
	} ch[3];
};

struct tx4939_rtc_reg {
	u32 ctl;
	u32 adr;
	u32 dat;
};

struct tx4939_cir_reg {
	u32 csr;
};

#define tx4939_ataptr(ch)       \
        ((struct tx4939_ata_reg *)TX4939_ATA_REG(ch))
#define tx4939_ndfmcptr         \
        ((struct tx4939_ndfmc_reg *)TX4939_NDFMC_REG)
#define tx4939_sramcptr         \
        ((struct tx4939_sramc_reg *)TX4939_SRAMC_REG)
#define tx4939_pcicptr(ch)      \
        ((struct tx4939_pcic_reg *)TX4939_PCIC_REG(ch))
#define tx4939_ddrcptr          \
        ((struct tx4939_ddrc_reg *)TX4939_DDRC_REG)
#define tx4939_ebuscptr         \
        ((struct tx4939_ebusc_reg *)TX4939_EBUSC_REG)
#define tx4939_vpcptr           \
        ((struct tx4939_vpc_reg *)TX4939_VPC_REG)
#define tx4939_dmacptr(ch)      \
        ((struct tx4939_dma_reg *)TX4939_DMAC_REG(ch))
#define tx4939_ccfgptr          \
        ((struct tx4939_ccfg_reg *)TX4939_CONFIG_REG)
#define tx4939_ircptr           \
        ((struct tx4939_irc_reg *)TX4939_IRC_REG)
#define tx4939_tmrptr(ch)       \
        ((struct tx4939_tmr_reg *)TX4939_TMR_REG(ch))
#define tx4939_sioptr(ch)       \
        ((struct tx4939_sio_reg *)TX4939_SIO_REG(ch))
#define tx4939_aclcptr          \
        ((struct tx4939_aclc_reg *)TX4939_ACLC_REG)
#define tx4939_spiptr           \
        ((struct tx4939_spi_reg *)TX4939_SPIC_REG)
#define tx4939_i2cptr           \
        ((struct tx4939_i2c_reg *)TX4939_I2C_REG)
#define tx4939_i2sptr           \
        ((struct tx4939_i2s_reg *)TX4939_I2S_REG)
#define tx4939_rtcptr           \
        ((struct tx4939_rtc_reg *)TX4939_RTC_REG)
#define tx4939_cirptr           \
        ((struct tx4939_cir_reg *)TX4939_CIR_REG)

#endif				/* __ASSEMBLY__ */

/*
 * Configuration Registers
 */

/* CCFG : Chip Configuration Register */

#define TX4939_REG_BASE	               0xff1f0000
#define TX4939_REG_SIZE	               0x00010000

#define TX4939_CCFG_PCIBOOT_ON         _CONST64(0x0000040000000000)
#define TX4939_CCFG_PCIBOOT_OFF        _CONST64(0x0000000000000000)
#define TX4939_CCFG_WDRST              _CONST64(0x0000020000000000)
#define TX4939_CCFG_WDREXEN_ASSERT     _CONST64(0x0000010000000000)
#define TX4939_CCFG_BCFG_MASK          _CONST64(0x000000ff00000000)
#define TX4939_CCFG_GTOT_MASK          _CONST64(0x0000000006000000)
#define TX4939_CCFG_GTOT_4096          _CONST64(0x0000000006000000)
#define TX4939_CCFG_GTOT_2048          _CONST64(0x0000000004000000)
#define TX4939_CCFG_GTOT_1028          _CONST64(0x0000000002000000)
#define TX4939_CCFG_GTOT_512           _CONST64(0x0000000000000000)
#define TX4939_CCFG_TINTDIS            _CONST64(0x0000000001000000)
#define TX4939_CCFG_PCI66              _CONST64(0x0000000000800000)
#define TX4939_CCFG_PCIMODE_MASK       _CONST64(0x0000000000400000)
#define TX4939_CCFG_PCIMODE_HOST       _CONST64(0x0000000000400000)
#define TX4939_CCFG_PCIMODE_SATELLITE  _CONST64(0x0000000000000000)
#define TX4939_CCFG_SSCG               _CONST64(0x0000000000100000)
#define TX4939_CCFG_MULCLK_MASK        _CONST64(0x00000000000e0000)
#define TX4939_CCFG_MULCLK_GET(x)      (((x) & TX4939_CCFG_MULCLK_MASK) >> 17)
#define TX4939_CCFG_BEOW               _CONST64(0x0000000000010000)
#define TX4939_CCFG_WR_NMI             _CONST64(0x0000000000000000)
#define TX4939_CCFG_WR_RESET           _CONST64(0x0000000000008000)
#define TX4939_CCFG_TOE                _CONST64(0x0000000000004000)
#define TX4939_CCFG_PCIARB_BUILTIN     _CONST64(0x0000000000002000)
#define TX4939_CCFG_PCIARB_EXTERNAL    _CONST64(0x0000000000000000)
#define TX4939_CCFG_YDIVMODE_MASK      _CONST64(0x0000000000001c00)
#define TX4939_CCFG_YDIVMODE_DIV2      _CONST64(0x0000000000000000)
#define TX4939_CCFG_YDIVMODE_DIV3      _CONST64(0x0000000000000600)
#define TX4939_CCFG_YDIVMODE_DIV5      _CONST64(0x0000000000001f00)
#define TX4939_CCFG_YDIVMODE_DIV6      _CONST64(0x0000000000001c00)
#define TX4939_CCFG_YDIVMODE_GET(x)    (((x) & TX4939_CCFG_YDIVMODE_MASK) >> 10)
#define TX4939_CCFG_PTSEL              _CONST64(0x0000000000000200)
#define TX4939_CCFG_BESEL_BWE          _CONST64(0x0000000000000100)
#define TX4939_CCFG_SYSSP              _CONST64(0x00000000000000c0)
#define TX4939_CCFG_SYSSP_DIV4         _CONST64(0x0000000000000000)
#define TX4939_CCFG_SYSSP_DIV3         _CONST64(0x0000000000000040)
#define TX4939_CCFG_SYSSP_DIV5         _CONST64(0x0000000000000080)
#define TX4939_CCFG_SYSSP_DIV6         _CONST64(0x00000000000000c0)
#define TX4939_CCFG_SYSSP_GET(x)       (((x) & TX4939_CCFG_SYSSP) >> 6)
#define TX4939_CCFG_ACKSEL_MASK        _CONST64(0x0000000000000020)
#define TX4939_CCFG_ACKSEL_ACK         _CONST64(0x0000000000000000)
#define TX4939_CCFG_ACKSEL_NORMAL      _CONST64(0x0000000000000020)
#define TX4939_CCFG_ROMW_MASK          _CONST64(0x0000000000000010)
#define TX4939_CCFG_ROMW_16BIT         _CONST64(0x0000000000000000)
#define TX4939_CCFG_ROMW_8BIT          _CONST64(0x0000000000000010)
#define TX4939_CCFG_ENDIAN_MASK        _CONST64(0x0000000000000004)
#define TX4939_CCFG_ENDIAN_LITTLE      _CONST64(0x0000000000000000)
#define TX4939_CCFG_ENDIAN_BIG         _CONST64(0x0000000000000004)
#define TX4939_CCFG_ARMODE_MASK        _CONST64(0x0000000000000002)
#define TX4939_CCFG_ARMODE_DYNAMIC     _CONST64(0x0000000000000000)
#define TX4939_CCFG_ARMODE_STATIC      _CONST64(0x0000000000000002)
#define TX4939_CCFG_ACEHOLD_MASK       _CONST64(0x0000000000000001)
#define TX4939_CCFG_ACEHOLD_SAME       _CONST64(0x0000000000000000)
#define TX4939_CCFG_ACEHOLD_1CLOCK     _CONST64(0x0000000000000001)

/* REVID : Chip Revision ID Register */

#define TX4939_REVID_PCODE_MASK        _CONST64(0x00000000ffff0000)
#define TX4939_REVID_MJERREV_MASK      _CONST64(0x000000000000f000)
#define TX4939_REVID_MINEREV_MASK      _CONST64(0x0000000000000f00)
#define TX4939_REVID_MJREV_MASK        _CONST64(0x00000000000000f0)
#define TX4939_REVID_MINREV_MASK       _CONST64(0x000000000000000f)

/* PCFG : Pin Configuration Register */
#define TX4939_PCFG_SIO2MODE_MASK      _CONST64(0xc000000000000000)
#define TX4939_PCFG_SIO2MODE_GPIO      _CONST64(0x8000000000000000)
#define TX4939_PCFG_SIO2MODE_SIO2      _CONST64(0x4000000000000000)
#define TX4939_PCFG_SIO2MODE_SIO0      _CONST64(0x0000000000000000)
#define TX4939_PCFG_SPIMODE_MASK       _CONST64(0x2000000000000000)
#define TX4939_PCFG_SPIMODE_SIO_GPIO   _CONST64(0x2000000000000000)
#define TX4939_PCFG_SPIMODE_SPI        _CONST64(0x0000000000000000)
#define TX4939_PCFG_I2CMODE_MASK       _CONST64(0x1000000000000000)
#define TX4939_PCFG_I2CMODE_I2C        _CONST64(0x1000000000000000)
#define TX4939_PCFG_I2CMODE_GPIO       _CONST64(0x0000000000000000)
#define TX4939_PCFG_I2SMODE_MASK       _CONST64(0x0c00000000000000)
#define TX4939_PCFG_I2SMODE_GPIO       _CONST64(0x0c00000000000000)
#define TX4939_PCFG_I2SMODE_I2S        _CONST64(0x0800000000000000)
#define TX4939_PCFG_I2SMODE_ACLC       _CONST64(0x0000000000000000)
#define TX4939_PCFG_SIO3MODE_MASK      _CONST64(0x0200000000000000)
#define TX4939_PCFG_SIO3MODE_GPIO      _CONST64(0x0200000000000000)
#define TX4939_PCFG_SIO3MODE_SIO       _CONST64(0x0000000000000000)
#define TX4939_PCFG_DMASEL3_MASK       _CONST64(0x0004000000000000)
#define TX4939_PCFG_DMASEL3_NDFC       _CONST64(0x0000000000000000)
#define TX4939_PCFG_DMASEL3_SIO0       _CONST64(0x0004000000000000)
#define TX4939_PCFG_VPMODE_MASK        _CONST64(0x0000300000000000)
#define TX4939_PCFG_VPMODE_GPIO        _CONST64(0x0000000000000000)
#define TX4939_PCFG_VPMODE_1P          _CONST64(0x0000100000000000)
#define TX4939_PCFG_VPMODE_3S          _CONST64(0x0000200000000000)
#define TX4939_PCFG_VPMODE_1P1S        _CONST64(0x0000300000000000)
#define TX4939_PCFG_ET1MODE_MASK       _CONST64(0x0000080000000000)
#define TX4939_PCFG_ET1MODE_ETHER      _CONST64(0x0000080000000000)
#define TX4939_PCFG_ET1MODE_OTHER      _CONST64(0x0000000000000000)
#define TX4939_PCFG_ET0MODE_MASK       _CONST64(0x0000040000000000)
#define TX4939_PCFG_ET0MODE_ETHER      _CONST64(0x0000040000000000)
#define TX4939_PCFG_ET0MODE_OTHER      _CONST64(0x0000000000000000)
#define TX4939_PCFG_ETMODE_ALL         _CONST64(0x00000c0000000000)
#define TX4939_PCFG_ATA1MODE_MASK      _CONST64(0x0000020000000000)
#define TX4939_PCFG_ATA1MODE_ATA       _CONST64(0x0000020000000000)
#define TX4939_PCFG_ATA1MODE_OTHER     _CONST64(0x0000000000000000)
#define TX4939_PCFG_ATA0MODE_MASK      _CONST64(0x0000010000000000)
#define TX4939_PCFG_ATA0MODE_ATA       _CONST64(0x0000010000000000)
#define TX4939_PCFG_ATA0MODE_OTHER     _CONST64(0x0000000000000000)
#define TX4939_PCFG_BP_PLL_MASK        _CONST64(0x0000000100000000)
#define TX4939_PCFG_BP_PLL_ON          _CONST64(0x0000000000000000)
#define TX4939_PCFG_BP_PLL_OFF         _CONST64(0x0000000100000000)
#define TX4939_PCFG_SYSCLKEN_MASK      _CONST64(0x0000000008000000)
#define TX4939_PCFG_SYSCLKEN_OUTPUT    _CONST64(0x0000000008000000)
#define TX4939_PCFG_SYSCLKEN_H         _CONST64(0x0000000000000000)
#define TX4939_PCFG_PCICLKEN_MASK      _CONST64(0x00000000000f0000)
#define TX4939_PCFG_PCICLKEN(x)        (TX4939_PCFG_PCICLKEN_MASK & (0x1 << (x+15)))
#define TX4939_PCFG_SPEED1_MASK        _CONST64(0x0000000000002000)
#define TX4939_PCFG_SPEED1_10MBPS      _CONST64(0x0000000000000000)
#define TX4939_PCFG_SPEED1_100MBPS     _CONST64(0x0000000000002000)
#define TX4939_PCFG_SPEED0_MASK        _CONST64(0x0000000000001000)
#define TX4939_PCFG_SPEED0_10MBPS      _CONST64(0x0000000000000000)
#define TX4939_PCFG_SPEED0_100MBPS     _CONST64(0x0000000000001000)
#define TX4939_PCFG_ITMODE_MASK        _CONST64(0x0000000000000180)
#define TX4939_PCFG_DMASEL2_MASK       _CONST64(0x0000000000000004)
#define TX4939_PCFG_DMASEL2_DMAREQ2    _CONST64(0x0000000000000000)
#define TX4939_PCFG_DMASEL2_SIO0       _CONST64(0x0000000000000004)
#define TX4939_PCFG_DMASEL10_MASK      _CONST64(0x0000000000000003)
#define TX4939_PCFG_DMASEL10_DMAREQ1   _CONST64(0x0000000000000000)

/* TOEA : Timeout Error Access Address Register */
#define TX4939_TOEA_TOEA_MASK          _CONST64(0x000000ffffffffff)

/* CLKCTR : Clock Control Register */
#define TX4939_CLKCTR_TM5CKD           _CONST64(0x2000000000000000)
#define TX4939_CLKCTR_TM4CKD           _CONST64(0x1000000000000000)
#define TX4939_CLKCTR_TM3CKD           _CONST64(0x0800000000000000)
#define TX4939_CLKCTR_CIRCKD           _CONST64(0x0400000000000000)
#define TX4939_CLKCTR_SIO3CKD          _CONST64(0x0200000000000000)
#define TX4939_CLKCTR_SIO2CKD          _CONST64(0x0100000000000000)
#define TX4939_CLKCTR_SIO1CKD          _CONST64(0x0080000000000000)
#define TX4939_CLKCTR_VPCCKD           _CONST64(0x0040000000000000)
#define TX4939_CLKCTR_ETH1CKD          _CONST64(0x0008000000000000)
#define TX4939_CLKCTR_ATA1CKD          _CONST64(0x0004000000000000)
#define TX4939_CLKCTR_BROMCKD          _CONST64(0x0002000000000000)
#define TX4939_CLKCTR_NDCCKD           _CONST64(0x0001000000000000)
#define TX4939_CLKCTR_I2CCKD           _CONST64(0x0000800000000000)
#define TX4939_CLKCTR_ETH0CKD          _CONST64(0x0000400000000000)
#define TX4939_CLKCTR_SPICKD           _CONST64(0x0000200000000000)
#define TX4939_CLKCTR_SRAMCKD          _CONST64(0x0000100000000000)
#define TX4939_CLKCTR_PCI1CKD          _CONST64(0x0000080000000000)
#define TX4939_CLKCTR_DMA1CKD          _CONST64(0x0000040000000000)
#define TX4939_CLKCTR_ACLCKD           _CONST64(0x0000020000000000)
#define TX4939_CLKCTR_ATA0CKD          _CONST64(0x0000010000000000)
#define TX4939_CLKCTR_DMAC0CKD         _CONST64(0x0000008000000000)
#define TX4939_CLKCTR_PCICCKD          _CONST64(0x0000004000000000)
#define TX4939_CLKCTR_I2SCKD           _CONST64(0x0000002000000000)
#define TX4939_CLKCTR_TM0CKD           _CONST64(0x0000001000000000)
#define TX4939_CLKCTR_TM1CKD           _CONST64(0x0000000800000000)
#define TX4939_CLKCTR_TM2CKD           _CONST64(0x0000000400000000)
#define TX4939_CLKCTR_SIO0CKD          _CONST64(0x0000000200000000)
#define TX4939_CLKCTR_CYPCKD           _CONST64(0x0000000100000000)
#define TX4939_CLKCTR_IOSRST           _CONST64(0x0000000080000000)
#define TX4939_CLKCTR_SYSRST           _CONST64(0x0000000040000000)
#define TX4939_CLKCTR_TM5RST           _CONST64(0x0000000020000000)
#define TX4939_CLKCTR_TM4RST           _CONST64(0x0000000010000000)
#define TX4939_CLKCTR_TM3RST           _CONST64(0x0000000008000000)
#define TX4939_CLKCTR_CIRRST           _CONST64(0x0000000004000000)
#define TX4939_CLKCTR_SIO3RST          _CONST64(0x0000000002000000)
#define TX4939_CLKCTR_SIO2RST          _CONST64(0x0000000001000000)
#define TX4939_CLKCTR_SIO1RST          _CONST64(0x0000000000800000)
#define TX4939_CLKCTR_VPCRST           _CONST64(0x0000000000400000)
#define TX4939_CLKCTR_EPCIRST          _CONST64(0x0000000000200000)
#define TX4939_CLKCTR_ETH1RST          _CONST64(0x0000000000080000)
#define TX4939_CLKCTR_ATA1RST          _CONST64(0x0000000000040000)
#define TX4939_CLKCTR_BROMRST          _CONST64(0x0000000000020000)
#define TX4939_CLKCTR_NDCRST           _CONST64(0x0000000000010000)
#define TX4939_CLKCTR_I2CRST           _CONST64(0x0000000000008000)
#define TX4939_CLKCTR_ETH0RST          _CONST64(0x0000000000004000)
#define TX4939_CLKCTR_SPIRST           _CONST64(0x0000000000002000)
#define TX4939_CLKCTR_SRAMRST          _CONST64(0x0000000000001000)
#define TX4939_CLKCTR_PCI1RST          _CONST64(0x0000000000000800)
#define TX4939_CLKCTR_DMA1RST          _CONST64(0x0000000000000400)
#define TX4939_CLKCTR_ACLRST           _CONST64(0x0000000000000200)
#define TX4939_CLKCTR_ATA0RST          _CONST64(0x0000000000000100)
#define TX4939_CLKCTR_DMAC0RST         _CONST64(0x0000000000000080)
#define TX4939_CLKCTR_PCICRST          _CONST64(0x0000000000000040)
#define TX4939_CLKCTR_I2SRST           _CONST64(0x0000000000000020)
#define TX4939_CLKCTR_TM0RST           _CONST64(0x0000000000000010)
#define TX4939_CLKCTR_TM1RST           _CONST64(0x0000000000000008)
#define TX4939_CLKCTR_TM2RST           _CONST64(0x0000000000000004)
#define TX4939_CLKCTR_SIO0RST          _CONST64(0x0000000000000002)
#define TX4939_CLKCTR_CYPRST           _CONST64(0x0000000000000001)

/* GARBC : G-Bus Arbiter Control Register */
#define TX4939_GARBC_ARBMD_MASK        _CONST64(0x8000000000000000)
#define TX4939_GARBC_ARBMD_FIX         _CONST64(0x0000000000000000)
#define TX4939_GARBC_ARBMD_RR          _CONST64(0x8000000000000000)
#define TX4939_GARBC_PRIORITY_MASK     _CONST64(0x0000000fffffffff)
#define TX4939_GARBC_PRIORITY_SET(a,b,c,d,e,f,g,h,i) \
             (((0xf & (a)) << 32) | ((0xf & (b)) << 28) | ((0xf & (c)) << 24) | \
              ((0xf & (d)) << 20) | ((0xf & (e)) << 16) | ((0xf & (f)) << 12) | \
              ((0xf & (g)) << 8) | ((0xf & (h)) << 4) | ((0xf & (i))))
#define TX4939_GARBCP_PCIC   0x0000
#define TX4939_GARBCP_PDMAC  0x0001
#define TX4939_GARBCP_DMAC0  0x0010
#define TX4939_GARBCP_DMAC1  0x0011
#define TX4939_GARBCP_PCIC1  0x0100
#define TX4939_GARBCP_ATA0   0x0101
#define TX4939_GARBCP_ATA1   0x0110
#define TX4939_GARBCP_CYP    0x0111
#define TX4939_GARBCP_VPC    0x1000

/* RAMP : Register Address Mapping Register */

/* DSKWCTRL: DLL De-Skew Control Register */

/* MCLKOSC Register */

/* MVLKCTL Register */
#define TX4939_MCLKCTL_BDE             0x00008000
#define TX4939_MCLKCTL_CBE             0x00000080

/* GPIO Mode Register */

/* GPIO Date Register */

/*
 * IRC
 */

/* IRDEN : Interrupt Detection Enable Register */
#define TX4939_IRDEN_IDE_MASK          0x00000001
#define TX4939_IRDEN_IDE_STOP          0x00000000
#define TX4939_IRDEN_IDE_START         0x00000001

/* ISCIPB : Interrupt Source and Cause IP Binding Register */
#define TX4939_ISCIPB_CMM_MASK         0x80000000
#define TX4939_ISCIPB_CMM_ORIGINAL     0x00000000
#define TX4939_ISCIPB_CMM_COMPATIBLE   0x80000000

/* IRDM : Interrupt Detection Mode Register */
#define TX4939_IRDM_LOW_LEVEL          0x0
#define TX4939_IRDM_HIGH_LEVEL         0x1
#define TX4939_IRDM_FALLING_EGDE       0x2
#define TX4939_IRDM_RISING_EGDE        0x3

/* IRMSK : Interrupt Mask Level Register */
#define TX4939_IRMSK_IML_MASK          0x00000007
#define TX4939_IRMSK_IML_LEVEL(x)      (TX4939_IRMSK_IML_MASK & (x))

/* IRLVLxx : Interrupt Level Registers */
#define TX4939_IRLVL_ALL_LEVEL_7       0x07070707

/* IREDC : Interrupt Edge Detection Clear Register */

/* IRPND : Interrupt Pending Register */

/* IRCS : Interrupt Current Status Register */
#define TX4939_IRCS_IF_MASK            0x00010000
#define TX4939_IRCS_IF_INTERRUPT       0x00000000
#define TX4939_IRCS_LVL_MASK           0x00000700
#define TX4939_IRCS_LVL_LEVEL(x)       ((0x7 & (x)) << 8)
#define TX4939_IRCS_CAUSE_MASK         0x0000003f

/* IRFLAG: Interrupt Request Flag Register */

/* IRPOL: Interrupt Request Polarity Control Register */

/* IRRCNT: Interrupt Request Control Register */

/* IRMASKINT: Interrupt Request Internal Interrupt Mask Register */

/* IRMASKEXT: Interrupt Request External Interrupt Mask Register */

/* IRDBR: Interrupt Debug Register */

/*
 * External Bus Interface
 */

#define TX4939_EBCCR_BA_MASK            _CONST64(0xffff000000000000)
#define TX4939_EBCCR_ISA_MASK           _CONST64(0x0000000000400000)
#define TX4939_EBCCR_ISA_ENABLE         _CONST64(0x0000000000400000)
#define TX4939_EBCCR_ISA_DISABLE        _CONST64(0x0000000000000000)
#define TX4939_EBCCR_BSZ_MASK           _CONST64(0x0000000000100000)
#define TX4939_EBCCR_BSZ_16BIT          _CONST64(0x0000000000100000)
#define TX4939_EBCCR_BSZ_8BIT           _CONST64(0x0000000000000000)
#define TX4939_EBCCR_PM_MASK            _CONST64(0x00000000000c0000)
#define TX4939_EBCCR_PM_NORMAL          _CONST64(0x0000000000000000)
#define TX4939_EBCCR_PM_4PAGE           _CONST64(0x0000000000040000)
#define TX4939_EBCCR_PM_8PAGE           _CONST64(0x0000000000080000)
#define TX4939_EBCCR_PM_16PAGE          _CONST64(0x00000000000c0000)
#define TX4939_EBCCR_PWT_MASK           _CONST64(0x0000000000030000)
#define TX4939_EBCCR_PWT_CYCLES(x)      ((0x3 & (x)) << 16)
#define TX4939_EBCCR_WT_MASK            _CONST64(0x000000000000f000)
#define TX4939_EBCCR_WT_EX_ACK_MODE     _CONST64(0x000000000003f000)
#define TX4939_EBCCR_WT_CYCLES(x)       ((0x6 & (x)) << 12)
#define TX4939_EBCCR_CS_MASK            _CONST64(0x0000000000000f00)
#define TX4939_EBCCR_CS_SIZE_1MB        _CONST64(0x0000000000000000)
#define TX4939_EBCCR_CS_SIZE_2MB        _CONST64(0x0000000000000100)
#define TX4939_EBCCR_CS_SIZE_4MB        _CONST64(0x0000000000000200)
#define TX4939_EBCCR_CS_SIZE_8MB        _CONST64(0x0000000000000300)
#define TX4939_EBCCR_CS_SIZE_16MB       _CONST64(0x0000000000000400)
#define TX4939_EBCCR_CS_SIZE_32MB       _CONST64(0x0000000000000500)
#define TX4939_EBCCR_CS_SIZE_64MB       _CONST64(0x0000000000000600)
#define TX4939_EBCCR_CS_SIZE_128MB      _CONST64(0x0000000000000700)
#define TX4939_EBCCR_CS_SIZE_256MB      _CONST64(0x0000000000000800)
#define TX4939_EBCCR_CS_SIZE_512MB      _CONST64(0x0000000000000900)
#define TX4939_EBCCR_BC_MASK            _CONST64(0x0000000000000080)
#define TX4939_EBCCR_BC_BE              _CONST64(0x0000000000000000)
#define TX4939_EBCCR_BC_BWE             _CONST64(0x0000000000000080)
#define TX4939_EBCCR_RDY_MASK           _CONST64(0x0000000000000040)
#define TX4939_EBCCR_RDY_DISABLE        _CONST64(0x0000000000000000)
#define TX4939_EBCCR_RDY_ENABLE         _CONST64(0x0000000000000040)
#define TX4939_EBCCR_SP_MASK            _CONST64(0x0000000000000030)
#define TX4939_EBCCR_SP_1_4             _CONST64(0x0000000000000000)
#define TX4939_EBCCR_SP_1_3             _CONST64(0x0000000000000010)
#define TX4939_EBCCR_SP_1_5             _CONST64(0x0000000000000020)
#define TX4939_EBCCR_SP_1_6             _CONST64(0x0000000000000030)
#define TX4939_EBCCR_ME_MASK            _CONST64(0x0000000000000008)
#define TX4939_EBCCR_ME_DISABLE         _CONST64(0x0000000000000000)
#define TX4939_EBCCR_ME_ENABLE          _CONST64(0x0000000000000008)
#define TX4939_EBCCR_SHWT_MASK          _CONST64(0x0000000000000007)
#define TX4939_EBCCR_SHWT_DISABLE       _CONST64(0x0000000000000000)
#define TX4939_EBCCR_SHWT_CYCLE(X)      ((0x7 & (x)) << 0)

/*
 * NAND Flash Memory Controller
 */

#define tx4939_read_nfmc(addr) (*(volatile unsigned int *)(addr))
#define tx4939_write_nfmc(b,addr) (*(volatile unsigned int *)(addr)) = (b)

/* NDFDTR : NAND Flash Memory Data Transfer Register */

/* NDFMCR : NAND Flash Memory Mode Control Register */
#define TX4939_NDFMCR_X16BUS            0x0400
#define TX4939_NDFMCR_DMAREQ_MASK       0x0300
#define TX4939_NDFMCR_DMAREQ_NODMA      0x0000
#define TX4939_NDFMCR_DMAREQ_128BYTE    0x0100
#define TX4939_NDFMCR_DMAREQ_256BYTE    0x0200
#define TX4939_NDFMCR_DMAREQ_512BYTE    0x0300
#define TX4939_NDFMCR_WE                0x0080
#define TX4939_NDFMCR_ECC_ALL           0x0060
#define TX4939_NDFMCR_ECC_RESET         0x0060
#define TX4939_NDFMCR_ECC_DISABLE       0x0000
#define TX4939_NDFMCR_ECC_ENABLE        0x0020
#define TX4939_NDFMCR_ECC_READ          0x0040
#define TX4939_NDFMCR_CE                0x0010
#define TX4939_NDFMCR_CS                0x00c0
#define TX4939_NDFMCR_BSPRT             0x0004
#define TX4939_NDFMCR_ALE               0x0002
#define TX4939_NDFMCR_ALE_LOW           0x0000
#define TX4939_NDFMCR_ALE_HIGH          0x0002
#define TX4939_NDFMCR_CLE               0x0001
#define TX4939_NDFMCR_CLE_LOW           0x0000
#define TX4939_NDFMCR_CLE_HIGH          0x0001

/* NDFSR : NAND Flash Memory Status Register */
#define TX4939_NDFSR_BUSY               0x80
#define TX4939_NDFSR_DMARUN             0x40

/* NDFISR : NAND Flash Memory Interrupt Status Register */
#define TX4939_NDFISR_DMADONE           0x02
#define TX4939_NDFISR_DMADONE_CLEAR     0x02
#define TX4939_NDFISR_RDY               0x01
#define TX4939_NDFISR_RDY_CLEAR         0x01

/* NDFIMR : NAND Flash Memory Interrupt Mask Register */
#define TX4939_NDFIMR_INTEN             0x80
#define TX4939_NDFIMR_MDMA              0x02
#define TX4939_NDFIMR_MRDY              0x01

/* NDFSPR : NAND Flash Memory Strobe Pulse Width Register */
#define TX4939_NDFSPR_HOLD_MASK         0xf0
#define TX4939_NDFSPR_HOLD_SET(x)       ((0xf & (x)) << 4)
#define TX4939_NDFSPR_HOLD_GET(x)       ((0xf0 & (x)) >> 4)
#define TX4939_NDFSPR_SPW_MASK          0x0f
#define TX4939_NDFSPR_SPW_SET(x)        ((0xf & (x)) << 0)
#define TX4939_NDFSPR_SPW_GET(x)        ((0x0f & (x)) >> 0)

/*
 * RTC
 */

/* RTCCTL : RTC Control and Status Register */
#define TX4939_RTCCTL_ALME              0x80
#define TX4939_RTCCTL_ALMD              0x40
#define TX4939_RTCCTL_BUSY              0x20
#define TX4939_RTCCTL_CMD_CLEAR          0xf8
#define TX4939_RTCCTL_CMD_GETTIME       0x01
#define TX4939_RTCCTL_CMD_SETTIME       0x02
#define TX4939_RTCCTL_CMD_GETALARM      0x03
#define TX4939_RTCCTL_CMD_SETALARM      0x04

/* RTCADR : RTC Address Register */

/* RTCDAT : Data port to access the contents of RTC Register */

/* RTCTBC : RTC Time Base Corrector Register */
#define TX4939_RTCTBC_PM                0x80
#define TX4939_RTCTBC_COMP_MASK         0x7f

/*
 * Video Port
 */

/* CSR : Control and Status Register */
#define TX4939_VPCCSR_GBINT                _CONST64(0x0000000000010000)
#define TX4939_VPCCSR_SWAPO                _CONST64(0x0000000000000020)
#define TX4939_VPCCSR_SWAPI                _CONST64(0x0000000000000010)
#define TX4939_VPCCSR_GINTE                _CONST64(0x0000000000000008)
#define TX4939_VPCCSR_RSTD                 _CONST64(0x0000000000000004)
#define TX4939_VPCCSR_RSTVPC               _CONST64(0x0000000000000002)

/*
 * Timer/Counter
 */

#define TX4939_NR_TMR                      6

/* TMTCRn : Timer Control Register n */
#define TX4939_TMTCR_TCE                   0x00000080
#define TX4939_TMTCR_CCDE                  0x00000040
#define TX4939_TMTCR_CRE                   0x00000020
#define TX4939_TMTCR_CECS                  0x00000008
#define TX4939_TMTCR_CCS                   0x00000004
#define TX4939_TMTCR_TMODE_MASK            0x00000003
#define TX4939_TMTCR_TMODE_WATCHDOG        0x00000002
#define TX4939_TMTCR_TMODE_PULSE           0x00000001
#define TX4939_TMTCR_TMODE_INTERVAL        0x00000000

/* TMTISRn : Timer Interrupt Status Register n */
#define TX4939_TMTISR_TWIS                 0x00000008
#define TX4939_TMTISR_TPIBS                0x00000004
#define TX4939_TMTISR_TPIAS                0x00000002
#define TX4939_TMTISR_TIIS                 0x00000001

/* TMCPRAn : Compare Register An */

/* TMCPRBn : Compare Register Bn */

/* TMITMRn : Interval Timer Mode Register n */
#define TX4939_TMITMR_TIIE                 0x00008000
#define TX4939_TMITMR_TZCE                 0x00000001

/* TMCCDRn : Devide Register n */
#define TX4939_TMCCDR_CCD_MASK             0x00000007
#define TX4939_TMCCDR_CCD_DIV_2            0x00000000
#define TX4939_TMCCDR_CCD_DIV_4            0x00000001
#define TX4939_TMCCDR_CCD_DIV_8            0x00000002
#define TX4939_TMCCDR_CCD_DIV_16           0x00000003
#define TX4939_TMCCDR_CCD_DIV_32           0x00000004
#define TX4939_TMCCDR_CCD_DIV_64           0x00000005
#define TX4939_TMCCDR_CCD_DIV_128          0x00000006
#define TX4939_TMCCDR_CCD_DIV_256          0x00000007

/* TMPGMRn : Pulse Generator Mode Register n */
#define TX4939_TMPGMR_TPIBE                0x00008000
#define TX4939_TMPGMR_TPIAE                0x00004000
#define TX4939_TMPGMR_FFI                  0x00000001

/* TMWTMRn : Watchdog Timer Mode Register n */
#define TX4939_TMWTMR_TWIE                 0x00008000
#define TX4939_TMWTMR_WDIS                 0x00000080
#define TX4939_TMWTMR_TWC                  0x00000001

/* TMTRRn : Timer Read Regiser n */

/*
 * DMAC
 */

#define TX4939_NR_DMA                      2

/* DMmMCR : DMA Master Control Register */
#define TX4939_DMMCR_EIS(n)                (0x1 << (28 + ((n) & 0x3)))
#define TX4939_DMMCR_DIS(n)                (0x1 << (24 + ((n) & 0x3)))
#define TX4939_DMMCR_FIFVC_MASK            _CONST64(0x00000000001fc000)
#define TX4939_DMMCR_FIFVC_GET(n)          (((n) & TX4939_DMMCR_FIFVC_MASK) >> 14)
#define TX4939_DMMCR_FIFWP_MASK            _CONST64(0x0000000000003800)
#define TX4939_DMMCR_FIFWP_GET(n)          (((n) & TX4939_DMMCR_FIFWP_MASK) >> 11)
#define TX4939_DMMCR_FIFRP_MASK            _CONST64(0x0000000000000700)
#define TX4939_DMMCR_FIFRP_GET(n)          (((n) & TX4939_DMMCR_FIFRP_MASK) >> 8)
#define TX4939_DMMCR_RSFIF                 _CONST64(0x0000000000000080)
#define TX4939_DMMCR_FIFUM(n)              (0x1 << (3 + ((n) & 0x3)))
#define TX4939_DMMCR_RRPT                  _CONST64(0x0000000000000002)
#define TX4939_DMMCR_MSTEN                 _CONST64(0x0000000000000001)

/* DMmCCRn : DMA Channel Control Register */
#define TX4939_DMCCR_IMMCHN                _CONST64(0x0000000020000000)
#define TX4939_DMCCR_USEXFSZ               _CONST64(0x0000000010000000)
#define TX4939_DMCCR_LE                    _CONST64(0x0000000008000000)
#define TX4939_DMCCR_DBINH                 _CONST64(0x0000000004000000)
#define TX4939_DMCCR_SBINH                 _CONST64(0x0000000002000000)
#define TX4939_DMCCR_CHRST                 _CONST64(0x0000000001000000)
#define TX4939_DMCCR_REVBYTE               _CONST64(0x0000000000800000)
#define TX4939_DMCCR_ACKPOL                _CONST64(0x0000000000400000)
#define TX4939_DMCCR_REQPL                 _CONST64(0x0000000000200000)
#define TX4939_DMCCR_EGREQ                 _CONST64(0x0000000000100000)
#define TX4939_DMCCR_CHDN                  _CONST64(0x0000000000080000)
#define TX4939_DMCCR_DNCTL_MASK            _CONST64(0x0000000000060000)
#define TX4939_DMCCR_EXTRQ                 _CONST64(0x0000000000010000)
#define TX4939_DMCCR_STLTIME_MASK          _CONST64(0x000000000000e000)
#define TX4939_DMCCR_STLTIME_960           _CONST64(0x0000000000002000)
#define TX4939_DMCCR_STLTIME_4032          _CONST64(0x0000000000004000)
#define TX4939_DMCCR_STLTIME_16320         _CONST64(0x0000000000006000)
#define TX4939_DMCCR_STLTIME_65472         _CONST64(0x0000000000008000)
#define TX4939_DMCCR_STLTIME_262080        _CONST64(0x000000000000a000)
#define TX4939_DMCCR_STLTIME_1048512       _CONST64(0x000000000000c000)
#define TX4939_DMCCR_STLTIME_4194240       _CONST64(0x000000000000e000)
#define TX4939_DMCCR_INTRQD_MASK           _CONST64(0x000000000000e000)
#define TX4939_DMCCR_INTRQD_16             _CONST64(0x0000000000002000)
#define TX4939_DMCCR_INTRQD_32             _CONST64(0x0000000000004000)
#define TX4939_DMCCR_INTRQD_64             _CONST64(0x0000000000006000)
#define TX4939_DMCCR_INTRQD_128            _CONST64(0x0000000000008000)
#define TX4939_DMCCR_INTRQD_256            _CONST64(0x000000000000a000)
#define TX4939_DMCCR_INTRQD_512            _CONST64(0x000000000000c000)
#define TX4939_DMCCR_INTRQD_1024           _CONST64(0x000000000000e000)
#define TX4939_DMCCR_INTENE                _CONST64(0x0000000000001000)
#define TX4939_DMCCR_INTENC                _CONST64(0x0000000000000800)
#define TX4939_DMCCR_INTENT                _CONST64(0x0000000000000400)
#define TX4939_DMCCR_CHNEN                 _CONST64(0x0000000000000200)
#define TX4939_DMCCR_XFACT                 _CONST64(0x0000000000000100)
#define TX4939_DMCCR_SMPCHN                _CONST64(0x0000000000000020)
#define TX4939_DMCCR_XFSZ_MASK             _CONST64(0x000000000000001c)
#define TX4939_DMCCR_XFSZ_1BYTE            _CONST64(0x0000000000000000)
#define TX4939_DMCCR_XFSZ_2BYTE            _CONST64(0x0000000000000004)
#define TX4939_DMCCR_XFSZ_4BYTE            _CONST64(0x0000000000000008)
#define TX4939_DMCCR_XFSZ_8BYTE            _CONST64(0x000000000000000c)
#define TX4939_DMCCR_XFSZ_4DWORD           _CONST64(0x0000000000000010)
#define TX4939_DMCCR_XFSZ_8DWORD           _CONST64(0x0000000000000014)
#define TX4939_DMCCR_XFSZ_16DWORD          _CONST64(0x0000000000000018)
#define TX4939_DMCCR_XFSZ_32DWORD          _CONST64(0x000000000000001c)
#define TX4939_DMCCR_MEMIO                 _CONST64(0x0000000000000002)
#define TX4939_DMCCR_SNGAD                 _CONST64(0x0000000000000001)

/* DMmCSRn : DMA Channel Status Register */
#define TX4939_DMCSR_WAITC_MASK            _CONST64(0x00000000ffff0000)
#define TX4939_DMCSR_WAITC_GET(n)          ((TX4939_DMCSR_WAITC_MASK & (n)) >> 16)
#define TX4939_DMCSR_CHNEN                 _CONST64(0x0000000000000400)
#define TX4939_DMCSR_STLXFER               _CONST64(0x0000000000000200)
#define TX4939_DMCSR_XFACT                 _CONST64(0x0000000000000100)
#define TX4939_DMCSR_ABCHC                 _CONST64(0x0000000000000080)
#define TX4939_DMCSR_NCHNC                 _CONST64(0x0000000000000040)
#define TX4939_DMCSR_NTRNFC                _CONST64(0x0000000000000020)
#define TX4939_DMCSR_EXTDN                 _CONST64(0x0000000000000010)
#define TX4939_DMCSR_CFERR                 _CONST64(0x0000000000000008)
#define TX4939_DMCSR_CHERR                 _CONST64(0x0000000000000004)
#define TX4939_DMCSR_DESERR                _CONST64(0x0000000000000002)
#define TX4939_DMCSR_SORERR                _CONST64(0x0000000000000001)

/* DMmSARn : DMA Source Address Regiser */

/* DMmDARn : DMA Destination Address Register */

/* DMmCHARn : DMA Chain Address Register */

/* DMmSAIRn : DMA Source Address Increment Register */

/* DMmDAIRn : DMA Destination Address Increment Register */

/* DMmCNTRn : DMA Count Register */

/* DM0MFDRn  : DMA Memory Fill Data Register */

/*
 * DDR
 */

/* Interrupt */
#define TX4939_DDR01_INIT_CLEAR            0x0c00

/* DRWINEN : DDR Mapping Window Control Register */
#define TX4939_DRWINEN_DCA_MASK            _CONST64(0x00000000c0000000)
#define TX4939_DRWINEN_EN(n)               (0x1 >> ((n) & 0x3))

/* DRWINn : DDR Mapping Window Register */
#define TX4939_DRWINLO_MASK                _CONST64(0xffff000000000000)
#define TX4939_DRWINLO_GET(n)              (((n) & TX4939_DRWINLO_MASK) >> (48-20))
#define TX4939_DRWINLO_SET(n)              (((n) << (48-20) & TX4939_DRWINLO_MASK))
#define TX4939_DRWINUP_MASK                _CONST64(0x0000ffff00000000)
#define TX4939_DRWINUP_GET(n)              (((n) & TX4939_DRWINUP_MASK) >> (32-20))
#define TX4939_DRWINUP_SET(n)              (((n) << (32-20) & 0x000003ff))
#define TX4939_DRWINOF_MASK                _CONST64(0x0000000003ff0000)
#define TX4939_DRWINOF_GET(n)              (((n) & TX4939_DRWINOF_MASK) << (20-16))
#define TX4939_DRWINOF_SET(n)              (((n) >> (20-16) & TX4939_DRWINOF_MASK))
#define TX4939_DRWIN_CS(n)                 ((n) & 0x3)

/*
 * PCIC
 */

/* PCIID : ID Register */

/* PCISTATUS : PCI Status/Command Regsiter */
#define TX4939_PCISTATUS_ALL               0x0000f900
#define TX4939_PCISTATUS_DPE               0x80000000
#define TX4939_PCISTATUS_SSE               0x40000000
#define TX4939_PCISTATUS_RMA               0x20000000
#define TX4939_PCISTATUS_RTA               0x10000000
#define TX4939_PCISTATUS_STA               0x08000000
#define TX4939_PCISTATUS_DT_MASK           0x06000000
#define TX4939_PCISTATUS_DT_FAST           0x00000000
#define TX4939_PCISTATUS_DT_MEDIUM         0x02000000
#define TX4939_PCISTATUS_DT_SLOW           0x04000000
#define TX4939_PCISTATUS_MDPE              0x01000000
#define TX4939_PCISTATUS_FBBCP             0x00800000
#define TX4939_PCISTATUS_66MCP             0x00200000
#define TX4939_PCISTATUS_CL                0x00100000
#define TX4939_PCISTATUS_FBBEN             0x00000200
#define TX4939_PCISTATUS_SEREN             0x00000100
#define TX4939_PCISTATUS_STPC              0x00000080
#define TX4939_PCISTATUS_PEREN             0x00000040
#define TX4939_PCISTATUS_VPS               0x00000020
#define TX4939_PCISTATUS_MWIEN             0x00000010
#define TX4939_PCISTATUS_SC                0x00000008
#define TX4939_PCISTATUS_BM                0x00000004
#define TX4939_PCISTATUS_MEMSP             0x00000002
#define TX4939_PCISTATUS_IOSP              0x00000001

/* PCICCREV : Class Code / Revision ID Register */

/* PCICFG1 : PCI Configuration 1 Register */
#define TX4939_PCICFG1_BISTC               0x80000000
#define TX4939_PCICFG1_MFUNS               0x00800000
#define TX4939_PCICFG1_HT_MASK             0x007f0000
#define TX4939_PCICFG1_HT_GET(x)           (((x) & TX4939_PCICFG1_HT_MASK) >> 16)
#define TX4939_PCICFG1_HT_SET(x)           (((x) & 0x7f) << 16)
#define TX4939_PCICFG1_LT_MASK             0x0000ff00
#define TX4939_PCICFG1_LT_GET(x)           (((x) & TX4939_PCICFG1_LT_MASK) >> 8)
#define TX4939_PCICFG1_LT_SET(x)           (((x) & 0xff) << 8)
#define TX4939_PCICFG1_CLS_MASK            0x000000ff
#define TX4939_PCICFG1_CLS_GET(x)          (((x) & TX4939_PCICFG1_CLS_MASK) >> 0)
#define TX4939_PCICFG1_CLS_SET(x)          (((x) & 0xff) << 0)

/* P2GMmPLBASE : P2G Memory Space (m) PCI Lower Base Address Register */
#define TX4939_P2GMPLBASE_BA_MASK          0xfff00000
#define TX4939_P2GMPLBASE_BA(x)            ((x) & TX4939_P2GMPLBASE_BA_MASK)
#define TX4939_P2GMPLBASE_PF               0x00000008
#define TX4939_P2GMPLBASE_TYPE_MASK        0x00000006
#define TX4939_P2GMPLBASE_MSI              0x00000001

/* P2GMmCFG : P2G Memory Space (m) Configuration Register */
#define TX4939_P2GMCFG_MSS_MASK            0x0000fff0
#define TX4939_P2GMCFG_MSS_MB(x)           ((0xe000 | (~((x << 4)-1))) & TX4939_P2GMCFG_MSS_MASK)
#define TX4939_P2GMCFG_PFCFG               0x00000008

/* P2GIOPBASE : P2G I/O Space PCI Base Address Register */
#define TX4939_P2GIOPBASE_MASK             0xffffff00
#define TX4939_P2GIOPBASE_BA(x)            ((x) & TX4939_P2GIOPBASE_MASK)
#define TX4939_P2GIOPBASE_IOSI             0x00000001

/* PCISID : Subsystem ID Register */
#define TX4939_PCISID_SSID_MASK            0xffff0000
#define TX4939_PCISID_SSID(x)              ((x) & TX4939_PCISID_SSID_MASK)
#define TX4939_PCISID_SSVID_MASK           0x0000ffff
#define TX4939_PCISID_SSVID(x)             ((x) & TX4939_PCISID_SSVID_MASK)

/* PCICAPPTR : Capabilities Pointer Register */
#define TX4939_PCICAPPTR_CAPPTR_MASK       0x000000ff
#define TX4939_PCICAPPTR_CAPPTR(x)         ((x) & TX4939_PCICAPPTR_CAPPTR_MASK)

/* PCICFG2 : PCI Configuration 2 Register */
#define TX4939_PCICFG2_ML_MASK             0xff000000
#define TX4939_PCICFG2_ML_GET(x)           (((x) & TX4939_PCICFG2_ML_MASK) >> 24)
#define TX4939_PCICFG2_ML_SET(x)           (((x) & 0xff) << 24)
#define TX4939_PCICFG2_MG_MASK             0x00ff0000
#define TX4939_PCICFG2_MG_GET(x)           (((x) & TX4939_PCICFG2_MG_MASK) >> 16)
#define TX4939_PCICFG2_MG_SET(x)           (((x) & 0xff) << 16)
#define TX4939_PCICFG2_IP_MASK             0x0000ff00
#define TX4939_PCICFG2_IP_INTA             0x00000100
#define TX4939_PCICFG2_IP_INTB             0x00000200
#define TX4939_PCICFG2_IP_INTC             0x00000300
#define TX4939_PCICFG2_IP_INTD             0x00000400
#define TX4939_PCICFG2_IL_MASK             0x000000ff
#define TX4939_PCICFG2_IL_GET(x)           (((x) & TX4939_PCICFG2_IL_MASK) >> 0)
#define TX4939_PCICFG2_IL_SET(x)           (((x) & 0xff) << 0)

/* G2PTOCNT : G2P Timeout Count Register */
#define TX4939_G2PTOCNT_RETRYTO_MASK       0x0000ff00
#define TX4939_G2PTOCNT_RETRYTO_GET(x)     (((x) & TX4939_G2PTOCNT_RETRYTO_MASK) >> 8)
#define TX4939_G2PTOCNT_RETRYTO_SET(x)     (((x) & 0xff) << 8)
#define TX4939_G2PTOCNT_TRDYTO_MASK        0x0000ff00
#define TX4939_G2PTOCNT_TRDYTO_GET(x)      (((x) & TX4939_G2PTOCNT_TRDYTO_MASK) >> 0)
#define TX4939_G2PTOCNT_TRDYTO_SET(x)      (((x) & 0xff) << 0)

/* G2PSTATUS : G2P Status Register */
#define TX4939_G2PSTATUS_ALL               0x00000003
#define TX4939_G2PSTATUS_IDTTOE            0x00000002
#define TX4939_G2PSTATUS_IDRTOE            0x00000001

/* G2PMASK : G2P Mask Register */
#define TX4939_G2PMASK_ALL                 0x00000003
#define TX4939_G2PMASK_IDTTOEIE            0x00000002
#define TX4939_G2PMASK_IDRTOEIE            0x00000001

/* PCISSTATUS : Satellite Mode PCI Status Register */
#define TX4939_PCISSTATUS_PS_MASK          0x03000000
#define TX4939_PCISSTATUS_PS_GET(x)        (((x) & TX4939_PCISSTATUS_PS_MASK) >> 24)
#define TX4939_PCISSTATUS_PMEEN            0x00800000
#define TX4939_PCISSTATUS_DPE              0x00008000
#define TX4939_PCISSTATUS_SSE              0x00004000
#define TX4939_PCISSTATUS_RMA              0x00002000
#define TX4939_PCISSTATUS_RTA              0x00001000
#define TX4939_PCISSTATUS_STA              0x00000800
#define TX4939_PCISSTATUS_DT_MASK          0x00000600
#define TX4939_PCISSTATUS_DT_GET(x)        (((x) & TX4939_PCISSTATUS_DT_MASK) >> 9)
#define TX4939_PCISSTATUS_MDPE             0x00000100

/* PCIMASK : PCI Status Interrupt Maks Register */
#define TX4939_PCIMASK_ALL                0x0000f900
#define TX4939_PCIMASK_DPEIE              0x00008000
#define TX4939_PCIMASK_SSEIE              0x00004000
#define TX4939_PCIMASK_RMAIE              0x00002000
#define TX4939_PCIMASK_RTAIE              0x00001000
#define TX4939_PCIMASK_STAIE              0x00000800
#define TX4939_PCIMASK_MDPEIE             0x00000100

/* P2GCFG : P2G Configuration Register */
#define TX4939_P2GCFG_PME                 0x00400000
#define TX4939_P2GCFG_TPRBL_MASK          0x00300000
#define TX4939_P2GCFG_TPRBL_2DWORD        0x00000000
#define TX4939_P2GCFG_TPRBL_4DWORD        0x00100000
#define TX4939_P2GCFG_TPRBL_6DWORD        0x00200000
#define TX4939_P2GCFG_TPRBL_8DWORD        0x00300000
#define TX4939_P2GCFG_FTRD                0x00008000
#define TX4939_P2GCFG_FTA                 0x00004000
#define TX4939_P2GCFG_MEM0PD              0x00001000
#define TX4939_P2GCFG_MEM1PD              0x00000800
#define TX4939_P2GCFG_MEM2PD              0x00000400
#define TX4939_P2GCFG_TOBFR               0x00000200
#define TX4939_P2GCFG_TIBFR               0x00000100

/* P2GSTATUS : P2G Status Register */
#define TX4939_P2GSTATUS_PMSC             0x01000000
#define TX4939_P2GSTATUS_PMEES            0x00800000
#define TX4939_P2GSTATUS_PMECLR           0x00400000
#define TX4939_P2GSTATUS_M66EN            0x00200000
#define TX4939_P2GSTATUS_IOBFE            0x00100000
#define TX4939_P2GSTATUS_IIBFE            0x00080000
#define TX4939_P2GSTATUS_TOBFE            0x00040000
#define TX4939_P2GSTATUS_TIBFE            0x00020000

/* P2GMASK : P2G Interrupt Mask Register */
#define TX4939_P2GMASK_PMSCIE             0x01000000
#define TX4939_P2GMASK_PMEESIE            0x00800000
#define TX4939_P2GMASK_PMECLRIE           0x00400000

/* P2GCCMD : P2G Current Command Register */
#define TX4939_P2GCCMD_TCCMD_MASK         0x0000000f

/* PBAREQPORT : PCI Bus Arbiter Request Port Register */

/* PBACFG : PCI Bus Arbiter Configuration Register */
#define TX4939_PBACFG_FIXPA               0x00000008
#define TX4939_PBACFG_RPBA                0x00000004
#define TX4939_PBACFG_PBAEN               0x00000002
#define TX4939_PBACFG_BMCEN               0x00000001

/* PBASTATUS : PCI Bus Arbiter Status Register */
#define TX4939_PBASTATUS_BM               0x00000001

/* PBAMASK : PCI Bus Arbiter Interrupt Mask Register */
#define TX4939_PBAMASK_BMIE               0x00000000

/* PBABM : PCI Bus Arbiter Broken Master Register */

/* PBACREQ : PCI Bus Arbiter Current Request Register */

/* PBACGNT : PCI Bus Arbiter Current Grant Register */

/* PBACSTATE : PCI Bus Arbiter Current State Register */

/* G2PMnGBASE : G2P Memory Space (m) G-Bus Base Address Register */
#define TX4939_G2PMGBASE_BSWAP            _CONST64(0x0000002000000000)
#define TX4939_G2PMGBASE_EXFER            _CONST64(0x0000001000000000)
#define TX4939_G2PMGBASE_BA_MASK          _CONST64(0x0000000fffffff00)
#define TX4939_G2PMnGBASE_BA(x)           ((x) & TX4939_G2PMnGBASE_BA_MASK)

/* G2PIOGBASE : G2P IO Space G-Bus Base Address Register */
#define TX4939_G2PIOGBASE_BSWAP           _CONST64(0x0000002000000000)
#define TX4939_G2PIOGBASE_EXFER           _CONST64(0x0000001000000000)
#define TX4939_G2PIOGBASE_BA_MASK         _CONST64(0x0000000fffffff00)
#define TX4939_G2PIOGBASE_BA(x)           ((x) & TX4939_G2PIOGBASE_BA_MASK)

/* G2PMnMASK : G2P Memory Space (m) Address Mask Register */
#define TX4939_G2PMMASK_AM_MASK           0xfffffff0
#define TX4939_G2PMMASK_AM_SET(X)         (((x-1) << 24) & TX4939_G2PMMASK_AM_MASK)

/* G2PIOMASK : G2P IO Space Address Mask Register */
#define TX4939_G2PIOMASK_AM_MASK          0xfffffff0
#define TX4939_G2PIOMASK_AM_SET(X)        (((x-1) << 24) & TX4939_G2PIOMASK_AM_MASK)

/* G2PMPBASE : G2P Memory Space (m) PCI Base Address Register */
#define TX4939_G2PMPBASE_BA_MASK          _CONST64(0x000000ffffffff00)

/* G2PIOPBASE : G2P IO Space PCI Base Address Register */
#define TX4939_G2PIOPBASE_BA_MASK         _CONST64(0x000000ffffffff00)

/* PCICCFG : PCI Controller Configuration Register */
#define TX4939_PCICCFG_GBWC_MASK          0x0fff0000
#define TX4939_PCICCFG_GBWC_SET(x)        (((x) << 16) & TX4939_PCICCFG_GBWC_MASK)
#define TX4939_PCICCFG_HRST               0x00000800
#define TX4939_PCICCFG_SRST               0x00000400
#define TX4939_PCICCFG_IRBE               0x00000200
#define TX4939_PCICCFG_G2PM(ch)           (0x00000100>>(ch))
#define TX4939_PCICCFG_G2PM0EN            0x00000100
#define TX4939_PCICCFG_G2PM1EN            0x00000080
#define TX4939_PCICCFG_G2PM2EN            0x00000040
#define TX4939_PCICCFG_G2PIOEN            0x00000020
#define TX4939_PCICCFG_TCAR               0x00000010
#define TX4939_PCICCFG_ICAEN              0x00000008
#define TX4939_PCICCFG_LCFG               0x00000004

/* PCICSTATUS : PCI Controller Status Register */
#define TX4939_PCICSTATUS_ALL             0x000007b8
#define TX4939_PCICSTATUS_PME             0x00000400
#define TX4939_PCICSTATUS_TLB             0x00000200
#define TX4939_PCICSTATUS_NIB             0x00000100
#define TX4939_PCICSTATUS_ZIB             0x00000080
#define TX4939_PCICSTATUS_PERR            0x00000020
#define TX4939_PCICSTATUS_SERR            0x00000010
#define TX4939_PCICSTATUS_GBE             0x00000008
#define TX4939_PCICSTATUS_IWB             0x00000002
#define TX4939_PCICSTATUS_E2PDONE         0x00000001

/* PCICMASK : PCI Controller Mask Register */
#define TX4939_PCICMASK_ALLIE             0x000007b8
#define TX4939_PCICMASK_PMEIE             0x00000400
#define TX4939_PCICMASK_TLBIE             0x00000200
#define TX4939_PCICMASK_NIBIE             0x00000100
#define TX4939_PCICMASK_ZIBIE             0x00000080
#define TX4939_PCICMASK_PERRIE            0x00000020
#define TX4939_PCICMASK_SERRIE            0x00000010
#define TX4939_PCICMASK_GBEIE             0x00000008

/* P2GMGBASE : P2G Memory Space (m) G-Bus Base Address Register */
#define TX4939_P2GMGBASE_P2GMEN           _CONST64(0x0000004000000000)
#define TX4939_P2GMGBASE_BSWAP            _CONST64(0x0000002000000000)
#define TX4939_P2GMGBASE_EXFER            _CONST64(0x0000001000000000)
#define TX4939_P2GMGBASE_BA_MASK          _CONST64(0x0000000fe0000000)
#define TX4939_P2GMGBASE_BA_SET(x)        (((x) << 20) & TX4939_P2GMGBASE_BA_MASK)

/* P2GIOGBASE : P2G IO Space G-Bus Base Address Register */
#define TX4939_P2GIOGBASE_P2GMEN          _CONST64(0x0000004000000000)
#define TX4939_P2GIOGBASE_BSWAP           _CONST64(0x0000002000000000)
#define TX4939_P2GIOGBASE_EXFER           _CONST64(0x0000001000000000)
#define TX4939_P2GIOGBASE_BA_MASK         _CONST64(0x0000000fe0000000)
#define TX4939_P2GIOGBASE_BA_SET(x)       (((x) << 20) & TX4939_P2GIOGBASE_BA_MASK)

/* G2PCFGADRS : G2P Configuration Address Register */
#define TX4939_G2PCFGADRS_BUSNUM_MASK     0x00ff0000
#define TX4939_G2PCFGADRS_BUSNUM_GET(x)   (((x) & TX4939_G2PCFGADRS_BUSNUM_MASK) >> 16)
#define TX4939_G2PCFGADRS_BUSNUM_SET(x)   (((x) << 16) & TX4939_G2PCFGADRS_BUSNUM_MASK)
#define TX4939_G2PCFGADRS_DEVNUM_MASK     0x0000f800
#define TX4939_G2PCFGADRS_DEVNUM_GET(x)   (((x) & TX4939_G2PCFGADRS_DEVNUM_MASK) >> 11)
#define TX4939_G2PCFGADRS_DEVNUM_SET(x)   (((x) << 11) & TX4939_G2PCFGADRS_DEVNUM_MASK)
#define TX4939_G2PCFGADRS_FNNUM_MASK      0x00000700
#define TX4939_G2PCFGADRS_FNNUM_GET(x)    (((x) & TX4939_G2PCFGADRS_FNNUM_MASK) >> 8)
#define TX4939_G2PCFGADRS_FNNUM_SET(x)    (((x) << 8) & TX4939_G2PCFGADRS_FNNUM_MASK)
#define TX4939_G2PCFGADRS_REGNUM_MASK     0x000000fc
#define TX4939_G2PCFGADRS_REGNUM_GET(x)   (((x) & TX4939_G2PCFGADRS_REGNUM_MASK) >> 2)
#define TX4939_G2PCFGADRS_REGNUM_SET(x)   (((x) << 2) & TX4939_G2PCFGADRS_REGNUM_MASK)
#define TX4939_G2PCFGADRS_TYPE_MASK       0x00000002
#define TX4939_G2PCFGADRS_TYPE_GET(x)     (((x) & TX4939_G2PCFGADRS_TYPE_MASK) >> 0)
#define TX4939_G2PCFGADRS_TYPE_SET(x)     (((x) << 0) & TX4939_G2PCFGADRS_TYPE_MASK)

#define TX4939_PCIC_IDSEL_AD_TO_SLOT(ad)  ((ad) - 11)
#define TX4939_PCIC_MAX_DEVNU	          TX4939_PCIC_IDSEL_AD_TO_SLOT(32)

/* G2PCFGDATA : G2P Configuration Data Register */

/* G2PINTACK : G2P Interrupt Acknowledge Data Register */

/* G2PSPC : G2P Special Cycle Data Register */

/* PCICDATA0 : Configuration Data 0 Register */

/* PCICDATA1 : Configuration Data 1 Register */

/* PCICDATA2 : Configuration Data 2 Register */

/* PCICDATA3 : Configuration Data 3 Register */

/* PDMCA : PDMAC Chain Address Register */
#define TX4939_PDMCA_MASK                 _CONST64(0x0000000ffffffff8)
#define TX4939_PDMCA(x)                   ((x) & TX4939_PDMCA_PDMCA_MASK)

/* PDMGA : PDMAC G-Bus Address Register */
#define TX4939_PDMGA_MASK                 _CONST64(0x0000000ffffffffc)
#define TX4939_PDMGA(x)                   ((x) & TX4939_PDMGA_MASK)

/* PDMPA : PDMAC PCI Bus Address Register */
#define TX4939_PDMPA_MASK                 _CONST64(0x0000000ffffffffc)
#define TX4939_PDMPA(x)                   ((x) & TX4939_PDMPA_MASK)

/* PDMCTR : PDMAC Count Register */
#define TX4939_PDMCTR_MASK                _CONST64(0x0000000000fffffc)
#define TX4939_PDMCTR(x)                  ((x) & TX4939_PDMCTR_MASK)

/* PDMCFG : PDMAC Control Register */
#define TX4939_PDMCFG_RSTFIFO             _CONST64(0x0000000000200000)
#define TX4939_PDMCFG_EXFER               _CONST64(0x0000000000100000)
#define TX4939_PDMCFG_REQDLY_MASK         _CONST64(0x0000000000003800)
#define TX4939_PDMCFG_REQDLY_SET(x)       (((x) << 10) & TX4939_PDMCFG_REQDLY_MASK)
#define TX4939_PDMCFG_REQDLY_GET(x)       (((x) & TX4939_PDMCFG_REQDLY_MASK) >> 10)
#define TX4939_PDMCFG_ERRIE               _CONST64(0x0000000000000400)
#define TX4939_PDMCFG_MCCMPIE             _CONST64(0x0000000000000200)
#define TX4939_PDMCFG_NTCMPIE             _CONST64(0x0000000000000100)
#define TX4939_PDMCFG_CHNEN               _CONST64(0x0000000000000080)
#define TX4939_PDMCFG_XFRACT              _CONST64(0x0000000000000040)
#define TX4939_PDMCFG_BSWAP               _CONST64(0x0000000000000010)
#define TX4939_PDMCFG_XFRSIZE_1DWORD      _CONST64(0x0000000000000000)
#define TX4939_PDMCFG_XFRSIZE_1QWORD      _CONST64(0x0000000000000004)
#define TX4939_PDMCFG_XFRSIZE_4QWORD      _CONST64(0x0000000000000008)
#define TX4939_PDMCFG_XFRDIRC             _CONST64(0x0000000000000002)
#define TX4939_PDMCFG_CHRST               _CONST64(0x0000000000000001)

/* PDMSTATUS : PDMAC Status Register */
#define TX4939_PDMSTATUS_REQCNT_MASK      _CONST64(0x000000003f000000)
#define TX4939_PDMSTATUS_REQCNT_GET(x)    (((x) & TX4939_PDMSTATUS_REQCNT_MASK) >> 24)
#define TX4939_PDMSTATUS_FIFOCNT_MASK     _CONST64(0x0000000000f00000)
#define TX4939_PDMSTATUS_FIFOCNT_GET(x)   (((x) & TX4939_PDMSTATUS_FIFOCNT_MASK) >> 20)
#define TX4939_PDMSTATUS_FIFOWP_MASK      _CONST64(0x00000000000c0000)
#define TX4939_PDMSTATUS_FIFOWP_GET(x)    (((x) & TX4939_PDMSTATUS_FIFOWP_MASK) >> 18)
#define TX4939_PDMSTATUS_FIFORP_MASK      _CONST64(0x0000000000030000)
#define TX4939_PDMSTATUS_FIFORP_GET(x)    (((x) & TX4939_PDMSTATUS_FIFORP_MASK) >> 16)
#define TX4939_PDMSTATUS_ERRINT           _CONST64(0x0000000000000800)
#define TX4939_PDMSTATUS_DONEINT          _CONST64(0x0000000000000400)
#define TX4939_PDMSTATUS_CHNEN            _CONST64(0x0000000000000200)
#define TX4939_PDMSTATUS_XFRACT           _CONST64(0x0000000000000100)
#define TX4939_PDMSTATUS_ACCMP            _CONST64(0x0000000000000080)
#define TX4939_PDMSTATUS_NCCMP            _CONST64(0x0000000000000040)
#define TX4939_PDMSTATUS_NTCMP            _CONST64(0x0000000000000020)
#define TX4939_PDMSTATUS_CFGERR           _CONST64(0x0000000000000008)
#define TX4939_PDMSTATUS_PCIERR           _CONST64(0x0000000000000004)
#define TX4939_PDMSTATUS_CHNERR           _CONST64(0x0000000000000002)
#define TX4939_PDMSTATUS_DATAERR          _CONST64(0x0000000000000001)

/*
 * ATA
 */
#define TX4939_ATA_SC_SOFT_RESET          0x8000
#define TX4939_ATA_SC_FIFO_RESET          0x4000
#define TX4939_ATA_SC_PDIAGN              0x2000
#define TX4939_ATA_SC_DASPN               0x1000
#define TX4939_ATA_SC_ATA_HARD_RESET      0x0800
#define TX4939_ATA_SC_MODE_XFER_PIO_4     0x0440
#define TX4939_ATA_SC_MODE_XFER_PIO_3     0x0330
#define TX4939_ATA_SC_MODE_XFER_PIO_2     0x0220
#define TX4939_ATA_SC_MODE_XFER_PIO_1     0x0110
#define TX4939_ATA_SC_MODE_XFER_PIO_0     0x0000
#define TX4939_ATA_SC_MODE_XFER_UDMA_5    0x00d0
#define TX4939_ATA_SC_MODE_XFER_UDMA_4    0x00c0
#define TX4939_ATA_SC_MODE_XFER_UDMA_3    0x00b0
#define TX4939_ATA_SC_MODE_XFER_UDMA_2    0x00a0
#define TX4939_ATA_SC_MODE_XFER_UDMA_1    0x0090
#define TX4939_ATA_SC_MODE_XFER_UDMA_0    0x0080
#define TX4939_ATA_SC_MODE_XFER_MDMA_2    0x0070
#define TX4939_ATA_SC_MODE_XFER_MDMA_1    0x0060
#define TX4939_ATA_SC_MODE_XFER_MDMA_0    0x0050
#define TX4939_ATA_SC_MODE_MASK           0x07f0
#define TX4939_ATA_SC_CMD_MODE_MASK       0x0700
#define TX4939_ATA_SC_DATA_MODE_MASK      0x00f0
#define TX4939_ATA_SC_BREAK_ENABLE        0x0008
#define TX4939_ATA_SC_END_BREAK           0x0004
#define TX4939_ATA_SC_AUTO_DMA_ENABLE     0x0002
#define TX4939_ATA_SC_ACCESS_NOW          0x0001

#define TX4939_ATA_DMA_STARTSTOP          0x0001

#define TX4939_ATA_IC_MASK_ALL            0xff00
#define TX4939_ATA_IC_ADDRESS_ERROR_INT   0x0080
#define TX4939_ATA_IC_REACH_MALTIPLE_INT  0x0040
#define TX4939_ATA_IC_DEV_TIMING_ERROR    0x0020
#define TX4939_ATA_IC_DMA_DEV_TERMINATE   0x0010
#define TX4939_ATA_IC_TIMERINT            0x0008
#define TX4939_ATA_IC_BUS_ERROR           0x0004
#define TX4939_ATA_IC_DATA_TRANSFER_END   0x0002
#define TX4939_ATA_IC_HOSTINT             0x0001

#define TX4939_ATA_PTC_PACKET_START       0x0001

/*
 * Ether
 */

#define TX4939_ETHER_IDSEL(ch)            (31 - (0x1 & (ch)))

/*
 * SIO
 */

#define TX4939_NR_SIO                     4

/*
 * SPI
 */

/*
 * CIR
 */

/*
 * I2C
 */

/*
 * I2S
 */

/*
 * ACLink
 */

/*
 * SRAM
 */

/*
 * Crypt
 */

#endif				/* __ASM_TX4939H */
