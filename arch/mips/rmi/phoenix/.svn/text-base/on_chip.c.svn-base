/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <asm/rmi/msgring.h>
#include <asm/rmi/iomap.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/debug.h>
#include <asm/rmi/phnx_user_mac.h>
#include <asm/rmi/sim.h>

unsigned long phoenix_io_base = (unsigned long)(DEFAULT_PHOENIX_IO_BASE);

#define MSGRNG_CC_INIT_CPU_DEST_0(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_0_REG, cc_table_cpu_##cpu.counters[0][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_1(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_1_REG, cc_table_cpu_##cpu.counters[1][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_2(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_2_REG, cc_table_cpu_##cpu.counters[2][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_3(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_3_REG, cc_table_cpu_##cpu.counters[3][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_4(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_4_REG, cc_table_cpu_##cpu.counters[4][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_5(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_5_REG, cc_table_cpu_##cpu.counters[5][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_6(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_6_REG, cc_table_cpu_##cpu.counters[6][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_7(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_7_REG, cc_table_cpu_##cpu.counters[7][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_8(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_8_REG, cc_table_cpu_##cpu.counters[8][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_9(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_9_REG, cc_table_cpu_##cpu.counters[9][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_10(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_10_REG, cc_table_cpu_##cpu.counters[10][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_11(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_11_REG, cc_table_cpu_##cpu.counters[11][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_12(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_12_REG, cc_table_cpu_##cpu.counters[12][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_13(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_13_REG, cc_table_cpu_##cpu.counters[13][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_14(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_14_REG, cc_table_cpu_##cpu.counters[14][7], 7 ); \
} while(0)

#define MSGRNG_CC_INIT_CPU_DEST_15(cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_15_REG, cc_table_cpu_##cpu.counters[15][7], 7 ); \
} while(0)

/* Initialized CC for cpu 0 to send to all buckets at 0-7 cpus */
#define MSGRNG_CC_INIT_CPU(cpu) \
do { \
  MSGRNG_CC_INIT_CPU_DEST_0(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_1(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_2(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_3(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_4(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_5(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_6(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_7(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_8(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_9(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_10(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_11(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_12(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_13(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_14(cpu); \
  MSGRNG_CC_INIT_CPU_DEST_15(cpu); \
} while (0)

#define MSGRNG_BUCKETSIZE_INIT_CPU(base) \
do { \
  msgrng_write_bucksize(0, bucket_sizes.bucket[base+0]); \
  msgrng_write_bucksize(1, bucket_sizes.bucket[base+1]); \
  msgrng_write_bucksize(2, bucket_sizes.bucket[base+2]); \
  msgrng_write_bucksize(3, bucket_sizes.bucket[base+3]); \
  msgrng_write_bucksize(4, bucket_sizes.bucket[base+4]); \
  msgrng_write_bucksize(5, bucket_sizes.bucket[base+5]); \
  msgrng_write_bucksize(6, bucket_sizes.bucket[base+6]); \
  msgrng_write_bucksize(7, bucket_sizes.bucket[base+7]); \
} while(0);


void phoenix_msgring_cpu_init(void)
{
  int id = cpu_logical_map(smp_processor_id());
	unsigned long flags;

	/* if not thead 0 */
	if ((id & 0x03)!=0) return;

	prom_dbg_msg("Initializing message ring for cpu_%d\n", id);

	msgrng_flags_save(flags);
	          
	/* Message Stations are shared among all threads in a cpu core
	 * Assume, thread 0 on all cores are always active when more than
	 * 1 thread is active in a core
	 */
	if (id == 0) {
		MSGRNG_BUCKETSIZE_INIT_CPU(0);
		MSGRNG_CC_INIT_CPU(0);
	} else if (id == 4) {
		MSGRNG_BUCKETSIZE_INIT_CPU(8);
		MSGRNG_CC_INIT_CPU(1);
	} else if (id == 8) {
		MSGRNG_BUCKETSIZE_INIT_CPU(16);
		MSGRNG_CC_INIT_CPU(2);
	} else if (id == 12) {
		MSGRNG_BUCKETSIZE_INIT_CPU(24);
		MSGRNG_CC_INIT_CPU(3);
	} else if (id == 16) {
		MSGRNG_BUCKETSIZE_INIT_CPU(32);
		MSGRNG_CC_INIT_CPU(4);
	} else if (id == 20) {
		MSGRNG_BUCKETSIZE_INIT_CPU(40);
		MSGRNG_CC_INIT_CPU(5);
	} else if (id == 24) {
		MSGRNG_BUCKETSIZE_INIT_CPU(48);
		MSGRNG_CC_INIT_CPU(6);
	} else if (id == 28) {
		MSGRNG_BUCKETSIZE_INIT_CPU(56);
		MSGRNG_CC_INIT_CPU(7);
	}  

	msgrng_flags_restore(flags);
}


struct tx_stn_handler {
  void (*action)(int, int, int, int, struct msgrng_msg *, void *);
  void *dev_id;
};

struct tx_stn {
  struct tx_stn_handler handler;
};

static struct tx_stn tx_stns[MAX_TX_STNS];

static int rxstn_to_txstn_map[128] = {
  [0 ... 7] = TX_STN_CPU_0,
  [8 ... 15] = TX_STN_CPU_1,
  [16 ... 23] = TX_STN_CPU_2,
  [24 ... 31] = TX_STN_CPU_3,
  [32 ... 39] = TX_STN_CPU_4,
  [40 ... 47] = TX_STN_CPU_5,
  [48 ... 55] = TX_STN_CPU_6,
  [56 ... 63] = TX_STN_CPU_7,
  [64 ... 95] = TX_STN_INVALID,
  [96 ... 103] = TX_STN_GMAC,
  [104 ... 107] = TX_STN_DMA,
  [108 ... 111] = TX_STN_INVALID,
  [112 ... 113] = TX_STN_XGS_0,
  [114 ... 115] = TX_STN_XGS_1,
  [116 ... 119] = TX_STN_INVALID,
  [120 ... 127] = TX_STN_SEC
};

static spinlock_t msgrng_lock;
static rwlock_t phnx_msgrng_reg_lock = RW_LOCK_UNLOCKED;
static phnx_atomic_t msgring_int_enabled;

static int    msgring_pop_num_buckets;
static __u32  msgring_pop_bucket_mask;
static int    msgring_int_type;
static int    msgring_watermark_count;
static __u32  msgring_thread_mask;

struct xlr_user_mac_config xlr_user_mac;
__u32 rmios_user_mac_thr_mask = 0;

void phnx_msgring_config(void)
{
	if (xlr_user_mac_rmios()) {
		msgring_int_type = 0x2;
		msgring_pop_num_buckets = 4;
		msgring_pop_bucket_mask = 0x0f;
	} else {
		msgring_int_type = 0x02;
		msgring_pop_num_buckets = 8;
		msgring_pop_bucket_mask = 0xff;
	}

	msgring_watermark_count = 1;
	msgring_thread_mask = 0x01;
/* 	printk("[%s]: int_type = 0x%x, pop_num_buckets=%d, pop_bucket_mask=%x" */
/* 	       "watermark_count=%d, thread_mask=%x\n", __FUNCTION__, */
/* 	       msgring_int_type, msgring_pop_num_buckets, msgring_pop_bucket_mask, */
/* 	       msgring_watermark_count, msgring_thread_mask); */
}


__u32 msgrng_msg_cycles = 0;
void phnx_msgring_int_handler(unsigned int irq, struct pt_regs *regs)
{
	unsigned long mflags;
	int bucket=0;
	int size=0, code=0, rx_stid=0, tx_stid=0;
	struct msgrng_msg msg;
	struct tx_stn_handler *handler=0;
	unsigned int bucket_empty_bm = 0;
	unsigned int status=0;
	
	if (irq == PIC_TIMER_0_IRQ) {
		phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);

		/* msgring timeout interrupt */
		phnx_inc_counter(MSGRNG_PIC_INT);
		/* ack the pic */
		phoenix_write_reg(mmio, PIC_INT_ACK, 
				(1 << (PIC_TIMER_0_IRQ - PIC_IRQ_BASE) ));
#ifdef CONFIG_PHOENIX_USER_MAC
		if(xlr_hybrid_user_mac())  phoenix_user_mac_update_time();
#endif
	}
	else {
		/* normal message ring interrupt */
		phnx_inc_counter(MSGRNG_INT);
	}

	irq_enter();

	msgrng_flags_save(mflags);

	/* First Drain all the high priority messages */
	for(;;) {
		bucket_empty_bm = (msgrng_read_status() >> 24) & msgring_pop_bucket_mask;
		if (bucket_empty_bm == msgring_pop_bucket_mask) break; /* all buckets empty */

		for(bucket=0;bucket<msgring_pop_num_buckets;bucket++) {      
			if ((bucket_empty_bm & (1 << bucket))/*empty*/) 
				continue;

			status = message_receive(bucket, &size, &code, 
					&rx_stid, &msg);
			if (status) continue;
      
			phnx_inc_counter(MSGRNG_MSG);
			msgrng_msg_cycles = read_c0_count();

			tx_stid = rxstn_to_txstn_map[rx_stid];

			read_lock(&phnx_msgrng_reg_lock);
			handler = &tx_stns[tx_stid].handler;
			if (!handler->action) {
				printk("[%s]: No Handler for message from"
					"stn_id=%d, dropping message\n", 
	       				__FUNCTION__, tx_stid);
			}
			else {
				(handler->action)(bucket, size, code, 
					rx_stid, &msg, handler->dev_id);
			}
			read_unlock(&phnx_msgrng_reg_lock);
		}
	}
  
	phnx_set_counter(MSGRNG_EXIT_STATUS, msgrng_read_status());

	msgrng_flags_restore(mflags);

	irq_exit();
}

#define MSGRING_INT_TYPE          0x02
#define MSGRING_WATERMARK_COUNT   1
#define MSGRING_THREAD_MASK       0x01
static void enable_msgring_int(void *info)
{
	unsigned long flags = 0, mflags=0;

	msgrng_access_save(&msgrng_lock, flags, mflags);
	/* enable the message ring interrupts */
	msgrng_write_config((msgring_watermark_count<<24)|(IRQ_MSGRING<<16)
			    |(msgring_thread_mask<<8)|msgring_int_type);
	msgrng_access_restore(&msgrng_lock, flags, mflags);  
}

void unregister_msgring_handler(int major, void *dev_id)
{
	struct tx_stn_handler *handler = 0;
	unsigned long flags = 0;

	/* Check if the message station is valid, if not return error */
	if (major >= MAX_TX_STNS) return ;

	write_lock_irqsave(&phnx_msgrng_reg_lock, flags);
	handler = &tx_stns[major].handler;

	handler->action = NULL;
	handler->dev_id = NULL;
	write_unlock_irqrestore(&phnx_msgrng_reg_lock, flags);
}
extern spinlock_t phnx_pic_lock;
int register_msgring_handler(int major, 
	     void (*action)(int, int,int,int,struct msgrng_msg *, void *),
	     void *dev_id)
{
	struct tx_stn_handler *handler = 0;
	int ret = 1;
	unsigned long flags = 0;

	if (major >= MAX_TX_STNS) return ret;

	phnx_msgring_config();

	/* Check if the message station is valid, if not return error */
	write_lock_irqsave(&phnx_msgrng_reg_lock, flags);

	handler = &tx_stns[major].handler;

	if (handler->action) goto out;

	handler->action = action;
	handler->dev_id = dev_id;

	ret = 0;
 out:
	write_unlock_irqrestore(&phnx_msgrng_reg_lock, flags);
	
	if (!ret && phnx_test_and_set(&msgring_int_enabled)) {
		uint32_t hard_cpu_online_map = 0;
		phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
		int i=0;
		__u32 msgring_global_thread_mask=0;

		/* configure the msgring interrupt on all cpus */
		on_each_cpu(enable_msgring_int, 0, 1, 1);

		/* Configure PIC to deliver msgring interrupt for timeouts */
		for (i=0;i<32;i++) {
			if (cpu_isset(i, cpu_online_map)) 
				hard_cpu_online_map |= (1<<cpu_logical_map(i));
		}
		for(i=0;i<8;i++) {
			msgring_global_thread_mask |= (msgring_thread_mask << (i<<2));
		}

		printk("[%s]: cpu_online_map = %lx, hard_cpu_online_map=%x,"
			       "msgring_global_thread_mask=%x\n",
	   		__FUNCTION__, (unsigned long)cpu_online_map.bits[0], 
			hard_cpu_online_map, msgring_global_thread_mask);
	  
		/* take an interrupt every 66000 pic cycles(66MHz) i.e 1ms 
		* it is round robin across 8 cpus cores, so each cpu 
		* (assuming all 8 cpu cores are active) takes an interrupt 
		* every 8ms
		*/
		spin_lock_irqsave(&phnx_pic_lock, flags);
		phoenix_write_reg(mmio, PIC_TIMER_0_MAXVAL_0, 66000);
		phoenix_write_reg(mmio, PIC_TIMER_0_MAXVAL_1, 0);
		phoenix_write_reg(mmio, PIC_IRT_0_TIMER_0, 
		      (hard_cpu_online_map & msgring_global_thread_mask));
		phoenix_write_reg(mmio, PIC_IRT_1_TIMER_0, 
				(1<<31)|(0<<30)|(1<<6)|(PIC_TIMER_0_IRQ));
		pic_update_control(1<<(8+0));
		spin_unlock_irqrestore(&phnx_pic_lock, flags);
	}
	
	return ret;
}

static void pic_init(void)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_PIC_OFFSET);
	int i=0;
	int level;

	prom_dbg_msg("Initializing PIC...\n");
	for(i=0; i<PIC_NUM_IRTS; i++) {
		level = PIC_IRQ_IS_EDGE_TRIGGERED(i);

		/* Bind all PIC irqs to cpu 0 */
		phoenix_write_reg(mmio, PIC_IRT_0_BASE + i, 0x01);

		/* Use local scheduling and high polarity for all IRTs 
		* Invalidate all IRTs, by default
		*/
		phoenix_write_reg(mmio, PIC_IRT_1_BASE + i, 
				(level<<30)|(1<<6)|(PIC_IRQ_BASE + i));
	}
	
}

atomic_t phnx_counters[NR_CPUS][PHNX_MAX_COUNTERS] __cacheline_aligned;

void on_chip_init(void)
{
	int i=0, j=0;

	msgring_int_enabled.value = 0;

	phnx_msgring_config();
	pic_init(); 

	phoenix_msgring_cpu_init();

	for(i=0;i<NR_CPUS;i++)
		for(j=0;j<PHNX_MAX_COUNTERS;j++) 
			atomic_set(&phnx_counters[i][j], 0);

	printk("&phnx_counters = 0x%lx, sizeof(phnx_counters) = 0x%x\n", 
	 (unsigned long)phnx_counters, (unsigned int)sizeof(phnx_counters));
}

