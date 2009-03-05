/*
 * Monahans Power Management Routines
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef	__MHN_PM_H__
#define	__MHN_PM_H__

#include <asm/types.h>

/* The io memory map follows the definition in include/asm/arch/hardware.h.
 */
#define RSEG0(a)	(((a) & 0x01ffffff) + 0x40000000)
#define KSEG0(a)	(((a) & 0x01ffffff) + 0xf6000000)
#define RSEG1(a)	(((a) & 0x0fffffff) + 0x42000000)
#define KSEG1(a)	((((a) & 0xfc000000) >> 3) + 0xf8000000)
#define RTCBASE		(0x00900000)
#define OSTBASE		(0x00a00000)
#define CLKBASE		(0x01340000)
#define INTCBASE	(0x00d00000)
#define GPIOBASE	(0x00e00000)
#define PMUBASE		(0x00f40000)

#define ARBBASE		(0x04000000)
#define SMCBASE		(0x08000000)

/* Registers offset with CLKBASE */
#define ACCR_OFF	0x0000
#define ACCR_PCCE	(1 << 11)
#define AICSR_OFF		0x0008
#define AICSR_PCIE	(1 << 4)
#define AICSR_TCIE	(1 << 2)
#define AICSR_FCIE	(1 << 0)
#define CKENA_OFF		0x000C
#define CKENB_OFF		0x0010
#define OSCC_OFF		0x10000
/* Registers offset with OSTBASE */
#define OSSR_OFF		0x0014
#define OIER_OFF		0x001C
#define OSCR4_OFF		0x0040
#define OSMR4_OFF		0x0080
#define OMCR4_OFF		0x00C0
/* Registers offseti with RTCBASE */
#define RTSR_OFF		0x0008
#define RTSR_PICE	(1 << 15)
#define RTSR_PIALE	(1 << 14)
#define PICR_OFF		0x0034
#define PIAR_OFF		0x0038
/* Registers offset with INTCBASE */
#define ICMR_OFF		0x0004
#define ICPR_OFF		0x0010
#define ICCR_OFF		0x0014
#define IPR0_OFF		0x001C
#define ICMR2_OFF		0x00A0
#define IPR32_OFF		0x00B0
/* Registers offset with PMUBASE */
#define ASCR_OFF		0x0000
#define ASCR_RDH		(1 << 31)
#define ASCR_D1S		(1 << 2)
#define ASCR_D2S		(1 << 1)
#define ASCR_D3S		(1 << 0)
#define ASCR_MASK		(ASCR_D1S | ASCR_D2S | ASCR_D3S)
#define ARSR_OFF		0x0004
#define AD3ER_OFF		0x0008
#define AD3SR_OFF		0x000C
#define AD2D0ER_OFF		0x0010
#define AD2D0SR_OFF		0x0014
#define AD2D1ER_OFF		0x0018
#define AD2D1SR_OFF		0x001C
#define AD1D0ER_OFF		0x0020
#define AD1D0SR_OFF		0x0024
#define AD3R_OFF		0x0030
#define AD2R_OFF		0x0034
#define AD1R_OFF		0x0038
#define PMCR_OFF		0x10000
#define PSR_OFF			0x10004
#define PSR_MASK		0x07
#define PSPR_OFF		0x10008
#define PCFR_OFF		0x1000C
#define PWER_OFF		0x10010
#define PWSR_OFF		0x10014
#define PECR_OFF		0x10018
#define PECR_E1IS	(1 << 31)
#define PECR_E1IE	(1 << 30)
#define PECR_E0IS	(1 << 29)
#define PECR_E0IE	(1 << 28)
#define PECR_DIR1	(1 << 5)
#define PECR_DIR0	(1 << 4)
#define PVCR_OFF		0x10100
/* Registers offset with ARBBASE */
#define ARBCTL1_OFF		0xFE00
#define ARBCTL2_OFF		0xFE80
/* Registers offset with SMCBASE */
#define MSC0_OFF		0x0008
#define MSC1_OFF		0x000C
#define MECR_OFF		0x0014
#define SXCNFG_OFF		0x001C
#define MCMEM0_OFF		0x0028
#define MCATT0_OFF		0x0030
#define MCIO0_OFF		0x0038
#define MEMCLKCFG_OFF	0x0068
#define CSADRCFG0_OFF	0x0080
#define CSADRCFG1_OFF	0x0084
#define CSADRCFG2_OFF	0x0088
#define CSADRCFG3_OFF	0x008C
#define CSADRCFG_P_OFF	0x0090
#define CSMSADRCFG_OFF	0x00A0

/* mode save flags */
#define PM_MODE_SAVE_FLAG_SYS	0x1
#define PM_MODE_SAVE_FLAG_IRQ	0x2
#define PM_MODE_SAVE_FLAG_FIQ	0x4
#define PM_MODE_SAVE_FLAG_ABT	0x8
#define PM_MODE_SAVE_FLAG_UND	0x10
#define PM_MODE_SAVE_FLAG_SVC	0x20

/* value for PWRMODE register */
#define	MHN_PM_S2D3C4		0x06
#define	MHN_PM_S0D2C2		0x03
#define	MHN_PM_S3D4C4		0x07
#define	MHN_PM_S0D1C2		0x02
#define	MHN_PM_S0D0C1		0x01

/* CPSR Processor constants */
#define CPSR_Mode_MASK		(0x0000001F)
#define CPSR_Mode_USR		(0x10)
#define CPSR_Mode_FIQ		(0x11)
#define CPSR_Mode_IRQ		(0x12)
#define CPSR_Mode_SVC		(0x13)
#define CPSR_Mode_ABT		(0x17)
#define CPSR_Mode_UND		(0x1B)
#define CPSR_Mode_SYS		(0x1F)
#define CPSR_I_Bit		(0x80)
#define CPSR_F_Bit		(0x40)

/****************************************************************************/
#define MHN_PM_WE_EXTERNAL0 	(0x1UL << 0)
#define MHN_PM_WE_EXTERNAL1 	(0x1UL << 1)
#define	MHN_PM_WE_GENERIC0 	(0x1UL << 2)
#define	MHN_PM_WE_GENERIC1 	(0x1UL << 3)
#define	MHN_PM_WE_GENERIC2	(0x1UL << 4)
#define	MHN_PM_WE_UART1		(0x1UL << 5)
#define	MHN_PM_WE_UART2		(0x1UL << 6)
#define	MHN_PM_WE_UART3		(0x1UL << 7)
#define	MHN_PM_WE_MKEY		(0x1UL << 8)
#define	MHN_PM_WE_GENERIC7	(0x1UL << 9)
#define	MHN_PM_WE_GENERIC8	(0x1UL << 10)
#define MHN_PM_WE_GENERIC9	(0x1UL << 11)
#define	MHN_PM_WE_GENERIC10	(0x1UL << 12)
#define	MHN_PM_WE_GENERIC11	(0x1UL << 13)
#define	MHN_PM_WE_GENERIC12	(0x1UL << 14)
#define	MHN_PM_WE_GENERIC13	(0x1UL << 15)
#define	MHN_PM_WE_OTG		(0x1UL << 16)
#define	MHN_PM_WE_RESERVE0	(0x1UL << 17)
#define	MHN_PM_WE_MLCD  	(0x1UL << 18)
#define	MHN_PM_WE_USIM0		(0x1UL << 19)
#define	MHN_PM_WE_USIM1		(0x1UL << 20)
#define	MHN_PM_WE_DKEY		(0x1UL << 21)
#define	MHN_PM_WE_MUX2		(0x1UL << 22)
#define	MHN_PM_WE_MUX3		(0x1UL << 23)
#define	MHN_PM_WE_MSL0		(0x1UL << 24)
#define	MHN_PM_WE_RESERVE1	(0x1UL << 25)
#define	MHN_PM_WE_USB2		(0x1UL << 26)
#define	MHN_PM_WE_RESERVE2	(0x1UL << 27)
#define	MHN_PM_WE_USBH		(0x1UL << 28)
#define	MHN_PM_WE_TSI		(0x1UL << 29)
#define	MHN_PM_WE_OST		(0x1UL << 30)
#define	MHN_PM_WE_RTC 		(0x1UL << 31)

#define PWSR_EDR0	(0x1 << 0)
#define PWSR_EDR1	(0x1 << 1)
#define PWSR_EDF0	(0x1 << 2)
#define PWSR_EDF1	(0x1 << 3)
#define PWSR_EERTC	(0x1 << 31)

#define PWER_WER0  (0x1 << 0)
#define PWER_WER1  (0x1 << 1)
#define PWER_WEF0  (0x1 << 2)
#define PWER_WEF1  (0x1 << 3)
#define PWER_WERTC (0x1 << 31)

#define AD3ER_MASK  (MHN_PM_WE_EXTERNAL0 | MHN_PM_WE_EXTERNAL1 |\
			MHN_PM_WE_UART1 | MHN_PM_WE_UART2 |	\
			MHN_PM_WE_UART3 | MHN_PM_WE_MKEY |	\
			MHN_PM_WE_OTG | MHN_PM_WE_DKEY |	\
			MHN_PM_WE_USB2 | MHN_PM_WE_USBH |	\
			MHN_PM_WE_TSI | MHN_PM_WE_RTC |		\
			MHN_PM_WE_OST)

#define AD2D1ER_MASK  (MHN_PM_WE_OST | MHN_PM_WE_RTC)

#define AD2D0ER_MASK  (MHN_PM_WE_EXTERNAL0 | MHN_PM_WE_EXTERNAL1 |\
			MHN_PM_WE_UART1 | MHN_PM_WE_UART2 |	\
			MHN_PM_WE_UART3 | MHN_PM_WE_MKEY |	\
			MHN_PM_WE_OTG | MHN_PM_WE_DKEY |	\
			MHN_PM_WE_USB2 | MHN_PM_WE_USBH |	\
			MHN_PM_WE_TSI | MHN_PM_WE_OST |		\
			MHN_PM_WE_RTC)

#define AD1D0ER_MASK  (MHN_PM_WE_EXTERNAL0 | MHN_PM_WE_EXTERNAL1 |\
			MHN_PM_WE_UART1 | MHN_PM_WE_UART2 |	\
			MHN_PM_WE_UART3 | MHN_PM_WE_MKEY |	\
			MHN_PM_WE_OTG | MHN_PM_WE_MLCD |	\
			MHN_PM_WE_DKEY | MHN_PM_WE_USB2 |	\
			MHN_PM_WE_USBH | MHN_PM_WE_TSI |	\
			MHN_PM_WE_OST | MHN_PM_WE_RTC)

#define WORD_SIZE	4

/* the position of each data memeber */
#define	SleepState_begin		0x0
#define SleepState_checksum		0x0
#define SleepState_wordCount		(SleepState_checksum + WORD_SIZE)
#define SleepState_areaAddress		(SleepState_wordCount + WORD_SIZE)
#define SleepState_modeSaveFlags	(SleepState_areaAddress + WORD_SIZE)
/* save ARM registers */
#define	SleepState_ENTRY_REGS		(SleepState_modeSaveFlags + WORD_SIZE)
#define SleepState_ENTRY_CPSR		(SleepState_ENTRY_REGS)
#define SleepState_ENTRY_SPSR		(SleepState_ENTRY_CPSR + WORD_SIZE)
#define SleepState_ENTRY_R0		(SleepState_ENTRY_SPSR + WORD_SIZE)
#define	SleepState_ENTRY_R1		(SleepState_ENTRY_R0 + WORD_SIZE)
#define SleepState_SYS_REGS		(SleepState_ENTRY_REGS + 17*WORD_SIZE)
#define SleepState_FIQ_REGS		(SleepState_SYS_REGS + 2*WORD_SIZE)
#define SleepState_IRQ_REGS		(SleepState_FIQ_REGS + 8*WORD_SIZE)
#define SleepState_ABT_REGS		(SleepState_IRQ_REGS + 3*WORD_SIZE)
#define SleepState_UND_REGS		(SleepState_ABT_REGS + 3*WORD_SIZE)
#define SleepState_SVC_REGS		(SleepState_UND_REGS + 3*WORD_SIZE)
/* save MMU settings */
#define SleepState_Cp15_ACR_MMU		(SleepState_SVC_REGS + 3*WORD_SIZE)
#define SleepState_Cp15_AUXCR_MMU	(SleepState_Cp15_ACR_MMU + WORD_SIZE)
#define SleepState_Cp15_TTBR_MMU	(SleepState_Cp15_AUXCR_MMU + WORD_SIZE)
#define SleepState_Cp15_DACR_MMU	(SleepState_Cp15_TTBR_MMU + WORD_SIZE)
#define SleepState_Cp15_PID_MMU		(SleepState_Cp15_DACR_MMU + WORD_SIZE)
#define SleepState_Cp15_CPAR		(SleepState_Cp15_PID_MMU + WORD_SIZE)

#define SleepState_extendedChecksumByteCount	(SleepState_Cp15_CPAR + WORD_SIZE)
#define SleepState_psprAddress	(SleepState_extendedChecksumByteCount + WORD_SIZE)
#define SleepState_flushFunc	(SleepState_psprAddress + WORD_SIZE)
#define	SleepState_end		(SleepState_flushFunc + WORD_SIZE)

#define	SleepState_size		(SleepState_end - SleepState_begin)

#ifndef __ASSEMBLY__

struct intc_regs {
	unsigned char __iomem *membase;
	unsigned int iccr;
	unsigned int ipr[32];
	unsigned int ipr2[21];
	unsigned int icmr;
	unsigned int icmr2;
	unsigned int iclr;
	unsigned int iclr2;
};

struct clock_regs {
	unsigned char __iomem *membase;
	unsigned int aicsr;
	unsigned int ckena;
	unsigned int ckenb;
	unsigned int oscc;
};

struct ost_regs {
	unsigned char __iomem *membase;
	unsigned int ossr;
	unsigned int oier;
	unsigned int oscr4;
	unsigned int osmr4;
	unsigned int omcr4;
};

struct rtc_regs {
	unsigned char __iomem *membase;
	unsigned int rtsr;
	unsigned int piar;
};

struct smc_regs {
	unsigned char __iomem *membase;
	unsigned int msc0;
	unsigned int msc1;
	unsigned int mecr;
	unsigned int sxcnfg;
	unsigned int mcmem0;
	unsigned int mcatt0;
	unsigned int mcio0;
	unsigned int memclkcfg;
	unsigned int cscfg0;
	unsigned int cscfg1;
	unsigned int cscfg2;
	unsigned int cscfg3;
	unsigned int cscfg_p;
	unsigned int csmscfg;
};

struct arb_regs {
	unsigned char __iomem *membase;
	unsigned int ctl1;
	unsigned int ctl2;
};

struct pmu_regs {
	unsigned char __iomem *membase;
	unsigned int pcfr;
	unsigned int pecr;
	unsigned int pvcr;
};

struct pm_save_data {
	u32 checksum;
	u32 wordCount;
	u32 areaAddress;
	u32 modeSaveFlags;
	/* current mode registers cpsr, sprsr, r0-r12, lr, sp */
	u32 ENTRY_REGS[17];
	/* SYS mode registers:sp, lr */
	u32 SYS_REGS[2];
	/* FIQ mode registers:spsr, r8-r12, sp, lr */
	u32 FIQ_REGS[8];
	/* IRQ mode registers:spsr, sp, lr */
	u32 IRQ_REGS[3];
	/* ABT mode registers:spsr, sp, lr */
	u32 ABT_REGS[3];
	/* UND mode registers:spsr, sp, lr */
	u32 UND_REGS[3];
	/* SVC mode registers:spsr, sp, lr */
	u32 SVC_REGS[3];
	/* MMU registers */
	u32 CP15_ACR_MMU;
	u32 CP15_AUXCR_MMU;
	u32 CP15_TTBR_MMU;
	u32 CP15_DACR_MMU;
	u32 CP15_PID_MMU;
	u32 CP15_CPAR;

	u32 extendedChecksumByteCount;
	/*u32 sramAddress; */
	u32 psprAddress;
	void (*flushFunc) (void);
	/* the parameter is the reserved bytes from 0x5c010000 */
	/* It returns the physical address of initlization code in SRAM */
};

struct mhn_pm_regs {
	/* It's used to save core registers. */
	struct pm_save_data pm_data;
	struct intc_regs intc;
	struct clock_regs clock;
	struct ost_regs ost;
	struct rtc_regs rtc;
	struct smc_regs smc;
	struct arb_regs arb;
	struct pmu_regs pmu;
	/* It's the virtual address of ISRAM that can be accessed by kernel.
	 */
	void *sram_map;
	/* It's used to save ISRAM data. */
	void *sram;
	/* It's used to save OBM that loaded from NAND flash. */
	void *obm;
	/* It's the address of DDR that stores key information.
	 * Two words are used from the address.
	 */
	void *data_pool;
	unsigned int word0;
	unsigned int word1;
};

/*
 * According PM wakeup src table, set proper source.
 */
typedef union {
	unsigned long value;
	struct {
		unsigned ext0:1;
		unsigned ext1:1;
		unsigned rev1:3;
		unsigned uart1:1;
		unsigned uart2:1;
		unsigned uart3:1;
		unsigned mkey:1;
		unsigned rev2:7;
		unsigned usbotg:1;
		unsigned rev3:1;
		unsigned mlcd:1;
		unsigned rev4:2;
		unsigned dkey:1;
		unsigned rev5:4;
		unsigned usb2:1;	/* USB 2.0 client */
		unsigned rev6:1;
		unsigned usbh:1;	/* USB Host Port 1 */
		unsigned tsi:1;
		unsigned ost:1;
		unsigned rtc:1;
	} bits;
} pm_wakeup_src_t;

#endif

#endif
