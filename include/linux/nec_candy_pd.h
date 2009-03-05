/*
 * include/linux/nec_candy_pd.h
 *
 * Platform data for the NEC Candy Ethernet device.
 *
 * Author: Wade Farnsworth <wfarnsworth@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef NEC_CANDY_PD_H
#define NEC_CANDY_PD_H
#define PLATFORM_ID_MASK 0xF8000000
#define WORKAROUND_MASK  0x7FFFFFFF
#define WORKAROUND(pid,wa) ((((pid) & PLATFORM_ID_MASK) == ((wa) & PLATFORM_ID_MASK )) && \
			   (((pid) & WORKAROUND_MASK) & ((wa) & WORKAROUND_MASK)))
#define COMMON_WORKAROUND(pid,wa) (((pid) & WORKAROUND_MASK) & ((wa) & WORKAROUND_MASK))
#define PLATFORM(pid,p) (((pid) & PLATFORM_ID_MASK)==p)
/***********************************************************************
 * platforms supported by nec-candy.
 ***********************************************************************/
#define NEC_CMB_VR4133_PID 0x08000000
#define NEC_CMB_VR7701_PID 0x10000000
#define NEC_VRBLADE_VR4133A_PID 0x18000000
/***********************************************************************
 * hardware bug workarounds shifts and masks
 ***********************************************************************/
#define WORKAROUND_E7_AFCE_SHIFT 0
#define WORKAROUND_E8_TX_STALL_SHIFT 1
#define WORKAROUND_E10_PRM_AMC_SHIFT 2
#define WORKAROUND_E10_VR4133_SHIFT 3
#define WORKAROUND_E13_TXFC_SHIFT 4
#define WORKAROUND_E19_VR4133AR20_SHIFT 5
#define WORKAROUND_E20_VR4133AR20_SHIFT 6
#define WORKAROUND_E21_PAD_SHIFT 7

#define WORKAROUND_E7_AFCE_BIT (1 << WORKAROUND_E7_AFCE_SHIFT)
#define WORKAROUND_E8_TX_STALL_BIT (1 << WORKAROUND_E8_TX_STALL_SHIFT)
#define WORKAROUND_E10_PRM_AMC_BIT (1 << WORKAROUND_E10_PRM_AMC_SHIFT)
#define WORKAROUND_E10_VR4133_BIT (1 << WORKAROUND_E10_VR4133_SHIFT)
#define WORKAROUND_E13_TXFC_BIT (1 << WORKAROUND_E13_TXFC_SHIFT)
/* WORKAROUND_E19_VR4133A and WORKAROUND_E20_VR4133A for restrictions of VR4133A Rev2.0 */
#define WORKAROUND_E19_VR4133AR20_BIT (1 << WORKAROUND_E19_VR4133AR20_SHIFT)
#define WORKAROUND_E20_VR4133AR20_BIT (1 << WORKAROUND_E20_VR4133AR20_SHIFT)
#define WORKAROUND_E21_PAD_BIT (1 << WORKAROUND_E21_PAD_SHIFT)

#define WORKAROUND_E19_VR4133AR20 ( NEC_VRBLADE_VR4133A_PID | WORKAROUND_E19_VR4133AR20_BIT )
#define WORKAROUND_E20_VR4133AR20 ( NEC_VRBLADE_VR4133A_PID | WORKAROUND_E20_VR4133AR20_BIT )

struct platform_options {
	unsigned :27;
	unsigned use_tx_ring_buffer :1;
	unsigned n_marvell_ports :4;
};
struct nec_candy_platform_data {
	unsigned platform_id;
	struct platform_options platform_options;
	unsigned char mac_addr[6];
	unsigned long pmd_addr;
};

#endif
