/* arch/ppc/syslib/mpc832x_pci.h
 *
 * Description: MPC832x PCI controller define
 *
 * Author: Olivia Yin (r63875@freescale.com)
 *
 * Copyright (C) Freescale Semiconductor, Inc. 2005. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __MPC832X_PCI_H__
#define __MPC832X_PCI_H__

/*
 * Local Access Window.
 */
typedef struct law832x {
	u32	bar; /* LBIU local access window base address register */
	u32 	ar;  /* LBIU local access window attribute register */
} law832x_t;

#define	LAWBAR_BAR	0xFFFFF000 /* Identifies the 20 most-significant address bits of the base of local access window n. The specified base address should be aligned to the window size, as defined by LBLAWARn[SIZE]. */
#define	LAWBAR_RES	~(LAWBAR_BAR)
#define	LAWAR_EN	0x80000000 /* 0 The local bus local access window n is disabled. 1 The local bus local access window n is enabled and other LBLAWAR0 and LBLAWBAR0 fields combine to identify an address range for this window. */
#define	LAWAR_SIZE	0x0000003F /* Identifies the size of the window from the starting address. Window size is 2^(SIZE+1) bytes. 000000–001010Reserved. Window is undefined. */
#define	LAWAR_SIZE_4K	0x0000000B
#define	LAWAR_SIZE_8K	0x0000000C
#define	LAWAR_SIZE_16K	0x0000000D
#define	LAWAR_SIZE_32K	0x0000000E
#define	LAWAR_SIZE_64K	0x0000000F
#define	LAWAR_SIZE_128K	0x00000010
#define	LAWAR_SIZE_256K	0x00000011
#define	LAWAR_SIZE_512K	0x00000012
#define	LAWAR_SIZE_1M	0x00000013
#define	LAWAR_SIZE_2M	0x00000014
#define	LAWAR_SIZE_4M	0x00000015
#define	LAWAR_SIZE_8M	0x00000016
#define	LAWAR_SIZE_16M	0x00000017
#define	LAWAR_SIZE_32M	0x00000018
#define	LAWAR_SIZE_64M	0x00000019
#define	LAWAR_SIZE_128M	0x0000001A
#define	LAWAR_SIZE_256M	0x0000001B
#define	LAWAR_SIZE_512M	0x0000001C
#define	LAWAR_SIZE_1G	0x0000001D
#define	LAWAR_SIZE_2G	0x0000001E
#define	LAWAR_RES	~(LAWAR_EN|LAWAR_SIZE)

/*
 * Clock Configuration Register
 */
typedef struct clk832x {
	u32	spmr; /* system PLL mode Register  */
	u32	occr; /* output clock control Register  */
	u32	sccr; /* system clock control Register  */
	u8	res0[0xF4];
} clk832x_t;

#define	OCCR_PCICOE0	0x80000000 /* PCICOE0  */
#define	OCCR_PCICOE1	0x40000000 /* PCICOE1  */
#define	OCCR_PCICOE2	0x20000000 /* PCICOE2  */

/*
 * PCI Outbound Translation Register
 */
typedef struct pci_outbound_window {
	u32	potar;
	u8	res0[4];
	u32	pobar;
	u8	res1[4];
	u32	pocmr;
	u8	res2[4];
} pot832x_t;

#define	POTAR_TA_MASK	0x000fffff
#define	POBAR_BA_MASK	0x000fffff
#define	POCMR_EN	0x80000000
#define	POCMR_IO	0x40000000	/* 0--memory space 1--I/O space */
#define	POCMR_SE	0x20000000	/* streaming enable */
#define	POCMR_DST	0x10000000	/* 0--PCI1 1--PCI2*/
#define	POCMR_CM_MASK	0x000fffff
#define	POCMR_CM_4G	0x00000000
#define	POCMR_CM_2G	0x00080000
#define	POCMR_CM_1G	0x000C0000
#define	POCMR_CM_512M	0x000E0000
#define	POCMR_CM_256M	0x000F0000
#define	POCMR_CM_128M	0x000F8000
#define	POCMR_CM_64M	0x000FC000
#define	POCMR_CM_32M	0x000FE000
#define	POCMR_CM_16M	0x000FF000
#define	POCMR_CM_8M	0x000FF800
#define	POCMR_CM_4M	0x000FFC00
#define	POCMR_CM_2M	0x000FFE00
#define	POCMR_CM_1M	0x000FFF00
#define	POCMR_CM_512K	0x000FFF80
#define	POCMR_CM_256K	0x000FFFC0
#define	POCMR_CM_128K	0x000FFFE0
#define	POCMR_CM_64K	0x000FFFF0
#define	POCMR_CM_32K	0x000FFFF8
#define	POCMR_CM_16K	0x000FFFFC
#define	POCMR_CM_8K	0x000FFFFE
#define	POCMR_CM_4K	0x000FFFFF

/*
 * PCI Controller Control and Status Registers
 */
typedef struct pcictrl832x {
	u32	esr;
	u32	ecdr;
	u32 	eer;
	u32	eatcr;
	u32	eacr;
	u32	eeacr;
	u32	edcr;
	u8	res0[4];
	u32	gcr;
	u32	ecr;
	u32	gsr;
	u8	res1[12];
	u32	pitar2;
	u8	res2[4];
	u32	pibar2;
	u32	piebar2;
	u32	piwar2;
	u8	res3[4];
	u32	pitar1;
	u8	res4[4];
	u32	pibar1;
	u32	piebar1;
	u32	piwar1;
	u8	res5[4];
	u32	pitar0;
	u8	res6[4];
	u32	pibar0;
	u8	res7[4];
	u32	piwar0;
	u8	res8[132];
} pcictrl832x_t;

#define	ESR_MERR	0x80000000
#define	ESR_APAR	0x00000400
#define	ESR_PCISERR	0x00000200
#define	ESR_MPERR	0x00000100
#define	ESR_TPERR	0x00000080
#define	ESR_NORSP	0x00000040
#define	ESR_TABT	0x00000020
#define	ECDR_APAR	0x00000400
#define	ECDR_PCISERR	0x00000200
#define	ECDR_MPERR	0x00000100
#define	ECDR_TPERR	0x00000080
#define	ECDR_NORSP	0x00000040
#define	ECDR_TABT	0x00000020
#define	EER_APAR	0x00000400
#define	EER_PCISERR	0x00000200
#define	EER_MPERR	0x00000100
#define	EER_TPERR	0x00000080
#define	EER_NORSP	0x00000040
#define	EER_TABT	0x00000020
#define	EATCR_ERRTYPR_MASK	0x70000000
#define	EATCR_ERRTYPR_APR	0x00000000	/* address parity error */
#define	EATCR_ERRTYPR_WDPR	0x10000000	/* write data parity error */
#define	EATCR_ERRTYPR_RDPR	0x20000000	/* read data parity error */
#define	EATCR_ERRTYPR_MA	0x30000000	/* master abort */
#define	EATCR_ERRTYPR_TA	0x40000000	/* target abort */
#define	EATCR_ERRTYPR_SE	0x50000000	/* system error indication received */
#define	EATCR_ERRTYPR_PEA	0x60000000	/* parity error indication received on a read */
#define	EATCR_ERRTYPR_PEW	0x70000000	/* parity error indication received on a write */
#define	EATCR_BN_MASK	0x0f000000	/* beat number */
#define	EATCR_BN_1st	0x00000000
#define	EATCR_BN_2ed	0x01000000
#define	EATCR_BN_3rd	0x02000000
#define	EATCR_BN_4th	0x03000000
#define	EATCR_BN_5th	0x0400000
#define	EATCR_BN_6th	0x05000000
#define	EATCR_BN_7th	0x06000000
#define	EATCR_BN_8th	0x07000000
#define	EATCR_BN_9th	0x08000000
#define	EATCR_TS_MASK	0x00300000	/* transaction size */
#define	EATCR_TS_4	0x00000000
#define	EATCR_TS_1	0x00100000
#define	EATCR_TS_2	0x00200000
#define	EATCR_TS_3	0x00300000
#define	EATCR_ES_MASK	0x000f0000	/* error source */
#define	EATCR_ES_EM	0x00000000	/* external master */
#define	EATCR_ES_DMA	0x00050000
#define	EATCR_CMD_MASK	0x0000f000
#define	EATCR_HBE_MASK	0x00000f00	/* PCI high byte enable*/
#define	EATCR_BE_MASK	0x000000f0	/* PCI byte enable */
#define	EATCR_HPB	0x00000004	/* high parity bit */
#define	EATCR_PB	0x00000002	/* parity bit*/
#define	EATCR_VI	0x00000001	/* error information valid */
#define	PITAR_TA_MASK	0x000fffff
#define	PIBAR_MASK	0xffffffff
#define	PIEBAR_EBA_MASK	0x000fffff
#define	PIWAR_EN	0x80000000
#define	PIWAR_PF	0x20000000
#define	PIWAR_RTT_MASK	0x000f0000
#define	PIWAR_RTT_NO_SNOOP	0x00040000
#define	PIWAR_RTT_SNOOP	0x00050000
#define	PIWAR_WTT_MASK	0x0000f000
#define	PIWAR_WTT_NO_SNOOP	0x00004000
#define	PIWAR_WTT_SNOOP	0x00005000
#define	PIWAR_IWS_MASK	0x0000003F
#define	PIWAR_IWS_4K	0x0000000B
#define	PIWAR_IWS_8K	0x0000000C
#define	PIWAR_IWS_16K	0x0000000D
#define	PIWAR_IWS_32K	0x0000000E
#define	PIWAR_IWS_64K	0x0000000F
#define	PIWAR_IWS_128K	0x00000010
#define	PIWAR_IWS_256K	0x00000011
#define	PIWAR_IWS_512K	0x00000012
#define	PIWAR_IWS_1M	0x00000013
#define	PIWAR_IWS_2M	0x00000014
#define	PIWAR_IWS_4M	0x00000015
#define	PIWAR_IWS_8M	0x00000016
#define	PIWAR_IWS_16M	0x00000017
#define	PIWAR_IWS_32M	0x00000018
#define	PIWAR_IWS_64M	0x00000019
#define	PIWAR_IWS_128M	0x0000001A
#define	PIWAR_IWS_256M	0x0000001B
#define	PIWAR_IWS_512M	0x0000001C
#define	PIWAR_IWS_1G	0x0000001D
#define	PIWAR_IWS_2G	0x0000001E

#endif /*__MPC832X_PCI_H__*/
