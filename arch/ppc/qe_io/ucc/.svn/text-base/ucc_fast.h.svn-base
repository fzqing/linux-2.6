/*
 * arch/ppc/qe_io/ucc/ucc_fast.h
 *
 * Internal header file for UCC FAST unit routines.
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
#ifndef __UCC_FAST_H__
#define __UCC_FAST_H__

#include <linux/kernel.h>

#include <asm/immap_qe.h>
#include <asm/qe.h>

#include "ucc.h"

/* Receive BD's status.
*/
#define R_E     0x80000000	/* buffer empty                                   */
#define R_W     0x20000000	/* wrap bit                                       */
#define R_I     0x10000000	/* interrupt on reception                         */
#define R_L     0x08000000	/* last                                           */
#define R_F     0x04000000	/* first                                          */

/* transmit BD's status.
*/
#define T_R     0x80000000	/* ready bit */
#define T_W     0x20000000	/* wrap bit */
#define T_I     0x10000000	/* interrupt on completion */
#define T_L     0x08000000	/* last */

/* Rx Data buffer must be 4 bytes aligned in most cases.*/
#define UCC_FAST_RX_ALIGN                  4
#define UCC_FAST_MRBLR_ALIGNMENT           4
#define UCC_FAST_VIRT_FIFO_REGS_ALIGNMENT  8

/* Sizes
*/
#define UCC_FAST_URFS_MIN_VAL                           0x88
#define UCC_FAST_RECEIVE_VIRTUAL_FIFO_SIZE_FUDGE_FACTOR 8

/* ucc_fast_channel_protocol_mode - UCC FAST mode.
*/
typedef enum ucc_fast_channel_protocol_mode {
	UCC_FAST_PROTOCOL_MODE_HDLC = 0x00000000,
	UCC_FAST_PROTOCOL_MODE_RESERVED01 = 0x00000001,
	UCC_FAST_PROTOCOL_MODE_RESERVED_QMC = 0x00000002,
	UCC_FAST_PROTOCOL_MODE_RESERVED02 = 0x00000003,
	UCC_FAST_PROTOCOL_MODE_RESERVED_UART = 0x00000004,
	UCC_FAST_PROTOCOL_MODE_RESERVED03 = 0x00000005,
	UCC_FAST_PROTOCOL_MODE_RESERVED_EX_MAC_1 = 0x00000006,
	UCC_FAST_PROTOCOL_MODE_RESERVED_EX_MAC_2 = 0x00000007,
	UCC_FAST_PROTOCOL_MODE_RESERVED_BISYNC = 0x00000008,
	UCC_FAST_PROTOCOL_MODE_RESERVED04 = 0x00000009,
	UCC_FAST_PROTOCOL_MODE_ATM = 0x0000000A,
	UCC_FAST_PROTOCOL_MODE_RESERVED05 = 0x0000000B,
	UCC_FAST_PROTOCOL_MODE_ETHERNET = 0x0000000C,
	UCC_FAST_PROTOCOL_MODE_RESERVED06 = 0x0000000D,
	UCC_FAST_PROTOCOL_MODE_POS = 0x0000000E,
	UCC_FAST_PROTOCOL_MODE_RESERVED07 = 0x0000000F
} ucc_fast_channel_protocol_mode_e;

/* ucc_fast_transparent_txrx - UCC Fast Transparent TX & RX
*/
typedef enum ucc_fast_transparent_txrx {
	UCC_FAST_GUMR_TRANSPARENT_TTX_TRX_NORMAL = 0x00000000,
	UCC_FAST_GUMR_TRANSPARENT_TTX_TRX_TRANSPARENT = 0x18000000	/* rx & tx transparent */
} ucc_fast_transparent_txrx_e;

/* UCC fast diagnostic mode
*/
typedef enum ucc_fast_diag_mode {
	UCC_FAST_DIAGNOSTIC_NORMAL = 0x0,	/* normal             */
	UCC_FAST_DIAGNOSTIC_LOCAL_LOOP_BACK = 0x40000000,	/* loop back          */
	UCC_FAST_DIAGNOSTIC_AUTO_ECHO = 0x80000000,	/* echo               */
	UCC_FAST_DIAGNOSTIC_LOOP_BACK_AND_ECHO = 0xC0000000	/* loop back and echo */
} ucc_fast_diag_mode_e;

/* UCC fast Sync length (transparent mode only)
*/
typedef enum ucc_fast_sync_len {
	UCC_FAST_SYNC_LEN_NOT_USED = 0x0,	/* not used  */
	UCC_FAST_SYNC_LEN_AUTOMATIC = 0x00004000,	/* automatic */
	UCC_FAST_SYNC_LEN_8_BIT = 0x00008000,	/* 8 bit     */
	UCC_FAST_SYNC_LEN_16_BIT = 0x0000C000	/* 16 bit    */
} ucc_fast_sync_len_e;

/* UCC fast RTS mode
*/
typedef enum ucc_fast_ready_to_send {
	UCC_FAST_SEND_IDLES_BETWEEN_FRAMES = 0x00000000,	/* idles between frames */
	UCC_FAST_SEND_FLAGS_BETWEEN_FRAMES = 0x00002000	/* flags between frames */
} ucc_fast_ready_to_send_e;

/* UCC fast receiver decoding mode
*/
typedef enum ucc_fast_rx_decoding_method {
	UCC_FAST_RX_ENCODING_NRZ = 0x00000000,	/* nrz  */
	UCC_FAST_RX_ENCODING_NRZI = 0x00000800,	/* nrzi */
	UCC_FAST_RX_ENCODING_RESERVED0 = 0x00001000,	/*      */
	UCC_FAST_RX_ENCODING_RESERVED1 = 0x00001800	/*      */
} ucc_fast_rx_decoding_method_e;

/* UCC fast transmitter encoding mode
*/
typedef enum ucc_fast_tx_encoding_method {
	UCC_FAST_TX_ENCODING_NRZ = 0x00000000,	/* nrz  */
	UCC_FAST_TX_ENCODING_NRZI = 0x00000100,	/* nrzi */
	UCC_FAST_TX_ENCODING_RESERVED0 = 0x00000200,	/*      */
	UCC_FAST_TX_ENCODING_RESERVED1 = 0x00000300	/*      */
} ucc_fast_tx_encoding_method_e;

/* UCC fast CRC length
*/
typedef enum ucc_fast_transparent_tcrc {
	UCC_FAST_16_BIT_CRC = 0x00000000,	/* 16 bit */
	UCC_FAST_CRC_RESERVED0 = 0x00000040,	/*        */
	UCC_FAST_32_BIT_CRC = 0x00000080,	/* 32 bit */
	UCC_FAST_CRC_RESERVED1 = 0x000000C0	/*        */
} ucc_fast_transparent_tcrc_e;

/* Fast UCC initialization structure.
*/
typedef struct ucc_fast_info {
	uint ucc_num;
	qe_clock_e rx_clock;
	qe_clock_e tx_clock;
	u32 regs;
	int irq;
	u32 uccm_mask;
	int bd_mem_part;
	int brkpt_support;
	int grant_support;
	int tsa;
	int cdp;
	int cds;
	int ctsp;
	int ctss;
	int tci;
	int txsy;
	int rtsm;
	int revd;
	int rsyn;
	u16 max_rx_buf_length;
	u16 urfs;
	u16 urfet;
	u16 urfset;
	u16 utfs;
	u16 utfet;
	u16 utftt;
	u16 ufpt;
	ucc_fast_channel_protocol_mode_e mode;
	ucc_fast_transparent_txrx_e ttx_trx;
	ucc_fast_tx_encoding_method_e tenc;
	ucc_fast_rx_decoding_method_e renc;
	ucc_fast_transparent_tcrc_e tcrc;
	ucc_fast_sync_len_e synl;
} ucc_fast_info_t;

typedef struct ucc_fast_private {
	ucc_fast_info_t *uf_info;
	ucc_fast_t *uf_regs;	/* a pointer to memory map of UCC regs.                                                */
	u32 *p_ucce;		/* a pointer to the event register in memory.                                               */
	u32 *p_uccm;		/* a pointer to the mask register in memory.                                                */
	int enabled_tx;		/* Whether channel is enabled for Tx (ENT)                                                  */
	int enabled_rx;		/* Whether channel is enabled for Rx (ENR)                                                  */
	int stopped_tx;		/* Whether channel has been stopped for Tx (STOP_TX, etc.)                                  */
	int stopped_rx;		/* Whether channel has been stopped for Rx                                                  */
	u32 ucc_fast_tx_virtual_fifo_base_offset;	/* pointer to base of Tx virtual fifo                                                       */
	u32 ucc_fast_rx_virtual_fifo_base_offset;	/* pointer to base of Rx virtual fifo                                                       */
#ifdef STATISTICS
	u32 tx_frames;		/* Transmitted frames counter.                                                              */
	u32 rx_frames;		/* Received frames counter (only frames passed to application).                             */
	u32 tx_discarded;	/* Discarded tx frames counter (frames that were discarded by the driver due to errors).    */
	u32 rx_discarded;	/* Discarded rx frames counter (frames that were discarded by the driver due to errors).    */
#endif				/* STATISTICS */
	u16 mrblr;		/* maximum receive buffer length */
} ucc_fast_private_t;

/* ucc_fast_init
 * Initializes Fast UCC according to user provided parameters.
 *
 * @Param         uf_info  - (In) pointer to the fast UCC info structure.
 * @Param         uccf_ret - (Out) pointer to the fast UCC structure.
 */
int ucc_fast_init(ucc_fast_info_t * uf_info, ucc_fast_private_t ** uccf_ret);

/* ucc_fast_free
 * Frees all resources for fast UCC.
 *
 * @Param         uccf - (In) pointer to the fast UCC structure.
 */
void ucc_fast_free(ucc_fast_private_t * uccf);

/* ucc_fast_enable
 * Enables a fast UCC port.
 * This routine enables Tx and/or Rx through the General UCC Mode Register.
 *
 * @Param         uccf - (In) pointer to the fast UCC structure.
 * @Param         mode - (In) TX, RX, or both.
 */
void ucc_fast_enable(ucc_fast_private_t * uccf, comm_dir_e mode);

/* ucc_fast_disable
 * Disables a fast UCC port.
 * This routine disables Tx and/or Rx through the General UCC Mode Register.
 *
 * @Param         uccf - (In) pointer to the fast UCC structure.
 * @Param         mode - (In) TX, RX, or both.
 */
void ucc_fast_disable(ucc_fast_private_t * uccf, comm_dir_e mode);

/* ucc_fast_irq
 * Handles interrupts on fast UCC.
 * Called from the general interrupt routine to handle interrupts on fast UCC.
 *
 * @Param         uccf - (In) pointer to the fast UCC structure.
 */
void ucc_fast_irq(ucc_fast_private_t * uccf);

/* ucc_fast_transmit_on_demand
 * Immediately forces a poll of the transmitter for data to be sent.
 * Typically, the hardware performs a periodic poll for data that the
 * transmit routine has set up to be transmitted. In cases where
 * this polling cycle is not soon enough, this optional routine can
 * be invoked to force a poll right away, instead. Proper use for
 * each transmission for which this functionality is desired is to
 * call the transmit routine and then this routine right after.
 *
 * @Param         uccf - (In) pointer to the fast UCC structure.
 */
void ucc_fast_transmit_on_demand(ucc_fast_private_t * uccf);

u32 ucc_fast_get_qe_cr_subblock(int uccf_num);

void ucc_fast_dump_regs(ucc_fast_private_t * uccf);

#endif				/* __UCC_FAST_H__ */
