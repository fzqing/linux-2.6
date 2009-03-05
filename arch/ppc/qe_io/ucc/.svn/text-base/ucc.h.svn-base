/*
 * arch/ppc/qe_io/ucc/ucc.h
 *
 * Internal header file for UCC unit routines.
 *
 * Author: Shlomi Gridish <gridish@freescale.com>
 *
 * Copyright 2005 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __UCC_H__
#define __UCC_H__

#include <asm/immap_qe.h>
#include <asm/qe.h>

#define STATISTICS

/* Slow or fast type for UCCs.
*/
typedef enum ucc_speed_type {
	UCC_SPEED_TYPE_FAST, UCC_SPEED_TYPE_SLOW
} ucc_speed_type_e;

/* Initial UCCs Parameter RAM address relative to: MEM_MAP_BASE (IMMR).
*/
typedef enum ucc_pram_initial_offset {
	UCC_PRAM_OFFSET_UCC1 = 0x8400, UCC_PRAM_OFFSET_UCC2 =
	    0x8500, UCC_PRAM_OFFSET_UCC3 = 0x8600, UCC_PRAM_OFFSET_UCC4 =
	    0x9000, UCC_PRAM_OFFSET_UCC5 = 0x8000, UCC_PRAM_OFFSET_UCC6 =
	    0x8100, UCC_PRAM_OFFSET_UCC7 = 0x8200, UCC_PRAM_OFFSET_UCC8 = 0x8300
} ucc_pram_initial_offset_e;

/* ucc_set_type
 * Sets UCC to slow or fast mode.
 *
 * @Param         ucc_num  - (In) number of UCC  (0-7).
 * @Param         regs     - (In) pointer to registers base for the UCC.
 * @Param         speed    - (In) slow or fast mode for UCC.
 */
int ucc_set_type(unsigned int ucc_num, struct ucc_common *regs,
		 enum ucc_speed_type speed);

/* ucc_init_guemr
 * Init the Guemr register.
 *
 * @Param         regs - (In) pointer to registers base for the UCC.
 */
int ucc_init_guemr(struct ucc_common *regs);

int ucc_set_qe_mux_mii_mng(unsigned int ucc_num);

int ucc_set_qe_mux_rxtx(unsigned int ucc_num, qe_clock_e clock,
			comm_dir_e mode);

int ucc_mux_set_grant_tsa_bkpt(unsigned int ucc_num, int set, u32 mask);

/* QE MUX clock routing for UCC
*/
static inline int ucc_set_qe_mux_grant(unsigned int ucc_num, int set)
{
	return ucc_mux_set_grant_tsa_bkpt(ucc_num, set, QE_CMXUCR_GRANT);
}

static inline int ucc_set_qe_mux_tsa(unsigned int ucc_num, int set)
{
	return ucc_mux_set_grant_tsa_bkpt(ucc_num, set, QE_CMXUCR_TSA);
}

static inline int ucc_set_qe_mux_bkpt(unsigned int ucc_num, int set)
{
	return ucc_mux_set_grant_tsa_bkpt(ucc_num, set, QE_CMXUCR_BKPT);
}

#endif				/* __UCC_H__ */
