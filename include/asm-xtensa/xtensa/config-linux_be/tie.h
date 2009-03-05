/*
 * xtensa/config/tie.h -- HAL definitions that are dependent on CORE and TIE configuration
 *
 *  This header file is sometimes referred to as the "compile-time HAL" or CHAL.
 *  It was generated for a specific Xtensa processor configuration,
 *  and furthermore for a specific set of TIE source files that extend
 *  basic core functionality.
 *
 *  Source for configuration-independent binaries (which link in a
 *  configuration-specific HAL library) must NEVER include this file.
 *  It is perfectly normal, however, for the HAL source itself to include this file.
 */

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


#ifndef XTENSA_CONFIG_TIE_H
#define XTENSA_CONFIG_TIE_H

/*----------------------------------------------------------------------
				GENERAL
  ----------------------------------------------------------------------*/

/*
 *  Separators for macros that expand into arrays.
 *  These can be predefined by files that #include this one,
 *  when different separators are required.
 */
/*  Element separator for macros that expand into 1-dimensional arrays:  */
#ifndef XCHAL_SEP
#define XCHAL_SEP			,
#endif
/*  Array separator for macros that expand into 2-dimensional arrays:  */
#ifndef XCHAL_SEP2
#define XCHAL_SEP2			},{
#endif






/*----------------------------------------------------------------------
			COPROCESSORS and EXTRA STATE
  ----------------------------------------------------------------------*/

#define XCHAL_CP_NUM			0	/* number of coprocessors */
#define XCHAL_CP_MAX			0	/* max coprocessor id plus one (0 if none) */
#define XCHAL_CP_MASK			0x00	/* bitmask of coprocessors by id */

/*  Space for coprocessors' state save areas:  */
#define XCHAL_CP0_SA_SIZE		0
#define XCHAL_CP1_SA_SIZE		0
#define XCHAL_CP2_SA_SIZE		0
#define XCHAL_CP3_SA_SIZE		0
#define XCHAL_CP4_SA_SIZE		0
#define XCHAL_CP5_SA_SIZE		0
#define XCHAL_CP6_SA_SIZE		0
#define XCHAL_CP7_SA_SIZE		0
/*  Minimum required alignments of CP state save areas:  */
#define XCHAL_CP0_SA_ALIGN		1
#define XCHAL_CP1_SA_ALIGN		1
#define XCHAL_CP2_SA_ALIGN		1
#define XCHAL_CP3_SA_ALIGN		1
#define XCHAL_CP4_SA_ALIGN		1
#define XCHAL_CP5_SA_ALIGN		1
#define XCHAL_CP6_SA_ALIGN		1
#define XCHAL_CP7_SA_ALIGN		1

/*  Indexing macros:  */
#define _XCHAL_CP_SA_SIZE(n)		XCHAL_CP ## n ## _SA_SIZE
#define XCHAL_CP_SA_SIZE(n)		_XCHAL_CP_SA_SIZE(n)	/* n = 0 .. 7 */
#define _XCHAL_CP_SA_ALIGN(n)		XCHAL_CP ## n ## _SA_ALIGN
#define XCHAL_CP_SA_ALIGN(n)		_XCHAL_CP_SA_ALIGN(n)	/* n = 0 .. 7 */


/*  Space for "extra" state (user special registers and non-cp TIE) save area:  */
#define XCHAL_EXTRA_SA_SIZE		0
#define XCHAL_EXTRA_SA_ALIGN		1

/*  Total save area size (extra + all coprocessors)  */
/*  (not useful until xthal_{save,restore}_all_extra() is implemented,  */
/*   but included for Tor2 beta; doesn't account for alignment!):  */
#define XCHAL_CPEXTRA_SA_SIZE_TOR2	0	/* Tor2Beta temporary definition -- do not use */

/*  Combined required alignment for all CP and EXTRA state save areas  */
/*  (does not include required alignment for any base config registers):  */
#define XCHAL_CPEXTRA_SA_ALIGN		1

/* ... */


#ifdef _ASMLANGUAGE
/*
 *  Assembly-language specific definitions (assembly macros, etc.).
 */

/********************
 *  Macros to save and restore the non-coprocessor TIE portion of EXTRA state.
 */

/* (none) */


/********************
 *  Macros to create functions that save and restore all EXTRA (non-coprocessor) state
 *  (does not include zero-overhead loop registers and non-optional registers).
 */

	/*
	 *  Macro that expands to the body of a function that
	 *  stores the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area in which to save extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_store_funcbody
	.endm


	/*
	 *  Macro that expands to the body of a function that
	 *  loads the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area from which to restore extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_load_funcbody
	.endm


/********************
 *  Macros to save and restore the state of each TIE coprocessor.
 */



/********************
 *  Macros to create functions that save and restore the state of *any* TIE coprocessor.
 */

	/*
	 *  Macro that expands to the body of a function
	 *  that stores the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area in which to save cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_store_funcbody
	.endm


	/*
	 *  Macro that expands to the body of a function
	 *  that loads the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area from which to restore cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_load_funcbody
	.endm

#endif /*_ASMLANGUAGE*/


/*
 *  Contents of save areas in terms of libdb register numbers.
 *  NOTE:  CONTENTS_LIBDB_{UREG,REGF} macros are not defined in this file;
 *  it is up to the user of this header file to define these macros
 *  usefully before each expansion of the CONTENTS_LIBDB macros.
 *  (Fields rsv[123] are reserved for future additions; they are currently
 *   set to zero but may be set to some useful values in the future.)
 *
 *	CONTENTS_LIBDB_SREG(libdbnum, offset, size, align, rsv1, name, sregnum, bitmask, rsv2, rsv3)
 *	CONTENTS_LIBDB_UREG(libdbnum, offset, size, align, rsv1, name, uregnum, bitmask, rsv2, rsv3)
 *	CONTENTS_LIBDB_REGF(libdbnum, offset, size, align, rsv1, name, index, numentries, contentsize, regname_base, regfile_name, rsv2, rsv3)
 */

#define XCHAL_EXTRA_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_EXTRA_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP0_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP0_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP1_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP1_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP2_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP2_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP3_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP3_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP4_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP4_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP5_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP5_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP6_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP6_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP7_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP7_SA_CONTENTS_LIBDB	/* empty */

#endif /*XTENSA_CONFIG_TIE_H*/

