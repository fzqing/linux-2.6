/*
 * Copyright (c) 2003 Tensilica, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2.1 of the GNU Lesser General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.  Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston MA 02111-1307,
 * USA.
 */

#ifndef _XTENSA_SPECREG_H
#define _XTENSA_SPECREG_H

/*  Special registers:  */
#define LBEG		0
#define LEND		1
#define LCOUNT		2
#define SAR		3
#define WINDOWBASE	72
#define WINDOWSTART	73
#define PTEVADDR	83
#define RASID		90
#define ITLBCFG		91
#define DTLBCFG		92
#define IBREAKENABLE	96
#define DDR		104
#define IBREAKA_0	128
#define IBREAKA_1	129
#define DBREAKA_0	144
#define DBREAKA_1	145
#define DBREAKC_0	160
#define DBREAKC_1	161
#define EPC_1		177
#define EPC_2		178
#define EPC_3		179
#define EPC_4		180
#define DEPC		192
#define EPS_2		194
#define EPS_3		195
#define EPS_4		196
#define EXCSAVE_1	209
#define EXCSAVE_2	210
#define EXCSAVE_3	211
#define EXCSAVE_4	212
#define INTERRUPT	226
#define INTENABLE	228
#define PS		230
#define THREADPTR	231
#define EXCCAUSE	232
#define DEBUGCAUSE	233
#define CCOUNT		234
#define ICOUNT		236
#define ICOUNTLEVEL	237
#define EXCVADDR	238
#define CCOMPARE_0	240
#define CCOMPARE_1	241
#define CCOMPARE_2	242
#define MISC_REG_0	244
#define MISC_REG_1	245

/*  Special cases (bases of special register series):  */
#define IBREAKA		128
#define DBREAKA		144
#define DBREAKC		160
#define EPC		176
#define EPS		192
#define EXCSAVE		208
#define CCOMPARE	240

/*  Special names for read-only and write-only interrupt registers:  */
#define INTREAD		226
#define INTSET		226
#define INTCLEAR	227

/*  EXCCAUSE register fields:  */
#define EXCCAUSE_EXCCAUSE_SHIFT	0
#define EXCCAUSE_EXCCAUSE_MASK	0x3F
/*  Exception causes (mostly incomplete!):  */
#define EXCCAUSE_ILLEGAL_INSTRUCTION		0	/* Illegal Instruction (IllegalInstruction) */
#define EXCCAUSE_SYSTEM_CALL			1	/* System Call (SystemCall) */
#define EXCCAUSE_INSTRUCTION_FETCH_ERROR	2	/* Instruction Fetch Error (InstructionFetchError) */
#define EXCCAUSE_LOAD_STORE_ERROR		3	/* Load Store Error (LoadStoreError) */
#define EXCCAUSE_LEVEL1_INTERRUPT		4	/* Level 1 Interrupt (Level1Interrupt) */
#define EXCCAUSE_ALLOCA				5	/* Stack Extension Assist (Alloca) */
#define EXCCAUSE_INTEGER_DIVIDE_BY_ZERO		6	/* Integer Divide by Zero (IntegerDivideByZero) */
#define EXCCAUSE_SPECULATION			7	/* Speculation (Speculation) */
#define EXCCAUSE_PRIVILEGED			8	/* Privileged Instruction (Privileged) */
#define EXCCAUSE_UNALIGNED			9	/* Unaligned Load Store (Unaligned) */
#define EXCCAUSE_ITLB_MISS			16	/* ITlb Miss Exception (ITlbMiss) */
#define EXCCAUSE_ITLB_MULTIHIT			17	/* ITlb Mutltihit Exception (ITlbMultihit) */
#define EXCCAUSE_ITLB_PRIVILEGE			18	/* ITlb Privilege Exception (ITlbPrivilege) */
#define EXCCAUSE_ITLB_SIZE_RESTRICTION		19	/* ITlb Size Restriction Exception (ITlbSizeRestriction) */
#define EXCCAUSE_FETCH_CACHE_ATTRIBUTE		20	/* Fetch Cache Attribute Exception (FetchCacheAttribute) */
#define EXCCAUSE_DTLB_MISS			24	/* DTlb Miss Exception (DTlbMiss) */
#define EXCCAUSE_DTLB_MULTIHIT			25	/* DTlb Multihit Exception (DTlbMultihit) */
#define EXCCAUSE_DTLB_PRIVILEGE			26	/* DTlb Privilege Exception (DTlbPrivilege) */
#define EXCCAUSE_DTLB_SIZE_RESTRICTION		27	/* DTlb Size Restriction Exception (DTlbSizeRestriction) */
#define EXCCAUSE_LOAD_CACHE_ATTRIBUTE		28	/* Load Cache Attribute Exception (LoadCacheAttribute) */
#define EXCCAUSE_STORE_CACHE_ATTRIBUTE		29	/* Store Cache Attribute Exception (StoreCacheAttribute) */
#define EXCCAUSE_FLOATING_POINT			40	/* Floating Point Exception (FloatingPoint) */

/*  PS register fields:  */
#define PS_WOE_SHIFT		18
#define PS_WOE_MASK		0x00040000
#define PS_WOE			PS_WOE_MASK
#define PS_CALLINC_SHIFT	16
#define PS_CALLINC_MASK		0x00030000
#define PS_CALLINC(n)		(((n)&3)<<PS_CALLINC_SHIFT)	/* n = 0..3 */
#define PS_OWB_SHIFT		8
#define PS_OWB_MASK		0x00000F00
#define PS_OWB(n)		(((n)&15)<<PS_OWB_SHIFT)	/* n = 0..15 (or 0..7) */
#define PS_RING_SHIFT		6
#define PS_RING_MASK		0x000000C0
#define PS_RING(n)		(((n)&3)<<PS_RING_SHIFT)	/* n = 0..3 */
#define PS_UM_SHIFT		5
#define PS_UM_MASK		0x00000020
#define PS_UM			PS_UM_MASK
#define PS_EXCM_SHIFT		4
#define PS_EXCM_MASK		0x00000010
#define PS_EXCM			PS_EXCM_MASK
#define PS_INTLEVEL_SHIFT	0
#define PS_INTLEVEL_MASK	0x0000000F
#define PS_INTLEVEL(n)		((n)&PS_INTLEVEL_MASK)		/* n = 0..15 */
/*  Backward compatibility (deprecated):  */
#define PS_PROGSTACK_SHIFT	PS_UM_SHIFT
#define PS_PROGSTACK_MASK	PS_UM_MASK
#define PS_PROG_SHIFT		PS_UM_SHIFT
#define PS_PROG_MASK		PS_UM_MASK
#define PS_PROG			PS_UM

/*  DBREAKCn register fields:  */
#define DBREAKC_MASK_SHIFT		0
#define DBREAKC_MASK_MASK		0x0000003F
#define DBREAKC_LOADBREAK_SHIFT		30
#define DBREAKC_LOADBREAK_MASK		0x40000000
#define DBREAKC_STOREBREAK_SHIFT	31
#define DBREAKC_STOREBREAK_MASK		0x80000000

/*  DEBUGCAUSE register fields:  */
#define DEBUGCAUSE_DEBUGINT_SHIFT	5
#define DEBUGCAUSE_DEBUGINT_MASK	0x20	/* debug interrupt */
#define DEBUGCAUSE_BREAKN_SHIFT		4
#define DEBUGCAUSE_BREAKN_MASK		0x10	/* BREAK.N instruction */
#define DEBUGCAUSE_BREAK_SHIFT		3
#define DEBUGCAUSE_BREAK_MASK		0x08	/* BREAK instruction */
#define DEBUGCAUSE_DBREAK_SHIFT		2
#define DEBUGCAUSE_DBREAK_MASK		0x04	/* DBREAK match */
#define DEBUGCAUSE_IBREAK_SHIFT		1
#define DEBUGCAUSE_IBREAK_MASK		0x02	/* IBREAK match */
#define DEBUGCAUSE_ICOUNT_SHIFT		0
#define DEBUGCAUSE_ICOUNT_MASK		0x01	/* ICOUNT would increment to zero */


#endif /* _XTENSA_SPECREG_H */

