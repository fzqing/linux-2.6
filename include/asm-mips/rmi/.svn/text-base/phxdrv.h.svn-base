/*
 **  Raza Microelectronics Incorporated
 **  Phoenix Security Engine driver for Linux
 **
 **  Copyright (C) 2003 Raza Foundries
 **  Author: Dave Koplos;  dkoplos@razafoundries.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * RMI Phoenix Security Engine
 *
 * phxdrv.h: Defintions for the Phoenix Security Engine driver.
 *
 * Revision History:
 *
 * 04/01/2004  DPK   Created
 */

#ifndef _PHXDRV_H_
#define _PHXDRV_H_

#include <asm/rmi/msgring.h>

/*
 *   Message Ring Specifics
 */

#define SEC_MSGRING_WORDSIZE      2

#if 0
#define CTRL_RES0           0
#define CTRL_RES1           1
#define CTRL_REG_FREE       2
#define CTRL_JUMBO_FREE     3
#define CTRL_CONT           4
#define CTRL_EOP            5
#define CTRL_START          6
#define CTRL_SNGL           7
#endif

/*
 *
 * 
 * rwR      31  30 29     27 26    24 23      21 20     18     
 *         |  NA  | RSA0Out | Rsa0In | Pipe3Out | Pipe3In | ... 
 *
 *          17       15 14     12 11      9 8       6 5        3 2       0
 *         |  Pipe2Out | Pipe2In | Pipe1In | Pipe1In | Pipe0Out | Pipe0In |
 *
 * DMA CREDIT REG -
 *   NUMBER OF CREDITS PER PIPE
 */
#define SEC_DMA_CREDIT_RSA0_OUT_FOUR   0x20000000
#define SEC_DMA_CREDIT_RSA0_OUT_TWO    0x10000000
#define SEC_DMA_CREDIT_RSA0_OUT_ONE    0x08000000

#define SEC_DMA_CREDIT_RSA0_IN_FOUR    0x04000000
#define SEC_DMA_CREDIT_RSA0_IN_TWO     0x02000000
#define SEC_DMA_CREDIT_RSA0_IN_ONE     0x01000000

#define SEC_DMA_CREDIT_PIPE3_OUT_FOUR  0x00800000
#define SEC_DMA_CREDIT_PIPE3_OUT_TWO   0x00400000
#define SEC_DMA_CREDIT_PIPE3_OUT_ONE   0x00200000

#define SEC_DMA_CREDIT_PIPE3_IN_FOUR   0x00100000
#define SEC_DMA_CREDIT_PIPE3_IN_TWO    0x00080000
#define SEC_DMA_CREDIT_PIPE3_IN_ONE    0x00040000 

#define SEC_DMA_CREDIT_PIPE2_OUT_FOUR  0x00020000
#define SEC_DMA_CREDIT_PIPE2_OUT_TWO   0x00010000
#define SEC_DMA_CREDIT_PIPE2_OUT_ONE   0x00008000

#define SEC_DMA_CREDIT_PIPE2_IN_FOUR   0x00004000
#define SEC_DMA_CREDIT_PIPE2_IN_TWO    0x00002000
#define SEC_DMA_CREDIT_PIPE2_IN_ONE    0x00001000 

#define SEC_DMA_CREDIT_PIPE1_OUT_FOUR  0x00000800
#define SEC_DMA_CREDIT_PIPE1_OUT_TWO   0x00000400
#define SEC_DMA_CREDIT_PIPE1_OUT_ONE   0x00000200

#define SEC_DMA_CREDIT_PIPE1_IN_FOUR   0x00000100
#define SEC_DMA_CREDIT_PIPE1_IN_TWO    0x00000080
#define SEC_DMA_CREDIT_PIPE1_IN_ONE    0x00000040

#define SEC_DMA_CREDIT_PIPE0_OUT_FOUR  0x00000020
#define SEC_DMA_CREDIT_PIPE0_OUT_TWO   0x00000010
#define SEC_DMA_CREDIT_PIPE0_OUT_ONE   0x00000008

#define SEC_DMA_CREDIT_PIPE0_IN_FOUR   0x00000004
#define SEC_DMA_CREDIT_PIPE0_IN_TWO    0x00000002
#define SEC_DMA_CREDIT_PIPE0_IN_ONE    0x00000001

/*
 *  Currently, FOUR credits per PIPE
 *  0x24924924  
 */
#define SEC_DMA_CREDIT_CONFIG          SEC_DMA_CREDIT_RSA0_OUT_FOUR | \
SEC_DMA_CREDIT_RSA0_IN_FOUR | \
	SEC_DMA_CREDIT_PIPE3_OUT_FOUR | \
	SEC_DMA_CREDIT_PIPE3_IN_FOUR | \
	SEC_DMA_CREDIT_PIPE2_OUT_FOUR | \
	SEC_DMA_CREDIT_PIPE2_IN_FOUR | \
	SEC_DMA_CREDIT_PIPE1_OUT_FOUR | \
	SEC_DMA_CREDIT_PIPE1_IN_FOUR | \
	SEC_DMA_CREDIT_PIPE0_OUT_FOUR | \
	SEC_DMA_CREDIT_PIPE0_IN_FOUR 

	/*
	 * CONFIG2  
	 *    31   5         4                   3            
	 *   |  NA  | PIPE3_DEF_DBL_ISS | PIPE2_DEF_DBL_ISS | ...
	 *
	 *                 2                   1                   0
	 *   ... | PIPE1_DEF_DBL_ISS | PIPE0_DEF_DBL_ISS | ROUND_ROBIN_MODE |
	 *
	 *  DBL_ISS - mode for SECENG and DMA controller which slows down transfers 
	 *             (to be conservativei; 0=Disable,1=Enable).
	 *  ROUND_ROBIN - mode where SECENG dispatches operations to PIPE0-PIPE3 
	 *                and all messages are sent to PIPE0.
	 *
	 */

#define SEC_CFG2_PIPE3_DBL_ISS_ON      0x00000010 
#define SEC_CFG2_PIPE3_DBL_ISS_OFF     0x00000000 
#define SEC_CFG2_PIPE2_DBL_ISS_ON      0x00000008 
#define SEC_CFG2_PIPE2_DBL_ISS_OFF     0x00000000 
#define SEC_CFG2_PIPE1_DBL_ISS_ON      0x00000004 
#define SEC_CFG2_PIPE1_DBL_ISS_OFF     0x00000000 
#define SEC_CFG2_PIPE0_DBL_ISS_ON      0x00000002 
#define SEC_CFG2_PIPE0_DBL_ISS_OFF     0x00000000 
#define SEC_CFG2_ROUND_ROBIN_ON        0x00000001 
#define SEC_CFG2_ROUND_ROBIN_OFF       0x00000000



	enum sec_pipe_config {

		SEC_PIPE_CIPHER_KEY0_L0            = 0x00,
		SEC_PIPE_CIPHER_KEY0_HI,
		SEC_PIPE_CIPHER_KEY1_LO,
		SEC_PIPE_CIPHER_KEY1_HI,
		SEC_PIPE_CIPHER_KEY2_LO,
		SEC_PIPE_CIPHER_KEY2_HI,
		SEC_PIPE_CIPHER_KEY3_LO,
		SEC_PIPE_CIPHER_KEY3_HI,
		SEC_PIPE_HMAC_KEY0_LO,
		SEC_PIPE_HMAC_KEY0_HI,
		SEC_PIPE_HMAC_KEY1_LO,
		SEC_PIPE_HMAC_KEY1_HI,
		SEC_PIPE_HMAC_KEY2_LO,
		SEC_PIPE_HMAC_KEY2_HI,
		SEC_PIPE_HMAC_KEY3_LO,
		SEC_PIPE_HMAC_KEY3_HI,
		SEC_PIPE_HMAC_KEY4_LO,
		SEC_PIPE_HMAC_KEY4_HI,
		SEC_PIPE_HMAC_KEY5_LO,
		SEC_PIPE_HMAC_KEY5_HI,
		SEC_PIPE_HMAC_KEY6_LO,
		SEC_PIPE_HMAC_KEY6_HI,
		SEC_PIPE_HMAC_KEY7_LO,
		SEC_PIPE_HMAC_KEY7_HI,
		SEC_PIPE_NCFBM_LO,
		SEC_PIPE_NCFBM_HI,
		SEC_PIPE_INSTR_LO,
		SEC_PIPE_INSTR_HI,
		SEC_PIPE_RSVD0,
		SEC_PIPE_RSVD1,
		SEC_PIPE_RSVD2,
		SEC_PIPE_RSVD3,

		SEC_PIPE_DF_PTRS0,
		SEC_PIPE_DF_PTRS1,
		SEC_PIPE_DF_PTRS2,
		SEC_PIPE_DF_PTRS3,
		SEC_PIPE_DF_PTRS4,
		SEC_PIPE_DF_PTRS5,
		SEC_PIPE_DF_PTRS6,
		SEC_PIPE_DF_PTRS7,

		SEC_PIPE_DU_DATA_IN_LO,
		SEC_PIPE_DU_DATA_IN_HI,
		SEC_PIPE_DU_DATA_IN_CTRL,
		SEC_PIPE_DU_DATA_OUT_LO,
		SEC_PIPE_DU_DATA_OUT_HI,
		SEC_PIPE_DU_DATA_OUT_CTRL,

		SEC_PIPE_STATE0,
		SEC_PIPE_STATE1,
		SEC_PIPE_STATE2,
		SEC_PIPE_STATE3,
		SEC_PIPE_STATE4,
		SEC_PIPE_INCLUDE_MASK0,
		SEC_PIPE_INCLUDE_MASK1,
		SEC_PIPE_INCLUDE_MASK2,
		SEC_PIPE_INCLUDE_MASK3,
		SEC_PIPE_INCLUDE_MASK4,
		SEC_PIPE_EXCLUDE_MASK0,
		SEC_PIPE_EXCLUDE_MASK1,
		SEC_PIPE_EXCLUDE_MASK2,
		SEC_PIPE_EXCLUDE_MASK3,
		SEC_PIPE_EXCLUDE_MASK4,
	};

enum sec_pipe_base_config {

	SEC_PIPE0_BASE = 0x00,
	SEC_PIPE1_BASE = 0x40,
	SEC_PIPE2_BASE = 0x80,
	SEC_PIPE3_BASE = 0xc0

};

enum sec_rsa_config {

	SEC_RSA_PIPE0_DU_DATA_IN_LO = 0x100,
	SEC_RSA_PIPE0_DU_DATA_IN_HI,
	SEC_RSA_PIPE0_DU_DATA_IN_CTRL,
	SEC_RSA_PIPE0_DU_DATA_OUT_LO,
	SEC_RSA_PIPE0_DU_DATA_OUT_HI,
	SEC_RSA_PIPE0_DU_DATA_OUT_CTRL,
	SEC_RSA_RSVD0,
	SEC_RSA_RSVD1,

	SEC_RSA_PIPE0_STATE0,
	SEC_RSA_PIPE0_STATE1,
	SEC_RSA_PIPE0_STATE2,
	SEC_RSA_PIPE0_INCLUDE_MASK0,
	SEC_RSA_PIPE0_INCLUDE_MASK1,
	SEC_RSA_PIPE0_INCLUDE_MASK2,
	SEC_RSA_PIPE0_EXCLUDE_MASK0,
	SEC_RSA_PIPE0_EXCLUDE_MASK1,
	SEC_RSA_PIPE0_EXCLUDE_MASK2,
	SEC_RSA_PIPE0_EVENT_CTR

};

enum sec_config {

	SEC_DMA_CREDIT = 0x140,
	SEC_CONFIG1,
	SEC_CONFIG2,
	SEC_CONFIG3,  

};

enum sec_debug_config {

	SEC_DW0_DESCRIPTOR0_LO  = 0x180,
	SEC_DW0_DESCRIPTOR0_HI,
	SEC_DW0_DESCRIPTOR1_LO,
	SEC_DW0_DESCRIPTOR1_HI,
	SEC_DW1_DESCRIPTOR0_LO,
	SEC_DW1_DESCRIPTOR0_HI,
	SEC_DW1_DESCRIPTOR1_LO,
	SEC_DW1_DESCRIPTOR1_HI,
	SEC_DW2_DESCRIPTOR0_LO,
	SEC_DW2_DESCRIPTOR0_HI,
	SEC_DW2_DESCRIPTOR1_LO,
	SEC_DW2_DESCRIPTOR1_HI,
	SEC_DW3_DESCRIPTOR0_LO,
	SEC_DW3_DESCRIPTOR0_HI,
	SEC_DW3_DESCRIPTOR1_LO,
	SEC_DW3_DESCRIPTOR1_HI,

	SEC_STATE0,
	SEC_STATE1,  
	SEC_STATE2,
	SEC_INCLUDE_MASK0,
	SEC_INCLUDE_MASK1,
	SEC_INCLUDE_MASK2,
	SEC_EXCLUDE_MASK0,
	SEC_EXCLUDE_MASK1,
	SEC_EXCLUDE_MASK2,
	SEC_EVENT_CTR

};

//enum sec_perf_config {

//  SEC_PERF0  = 0x1c0

//};

enum sec_msgring_bucket_config {

	SEC_BIU_CREDITS = 0x308,

	SEC_MSG_BUCKET0_SIZE = 0x320,
	SEC_MSG_BUCKET1_SIZE,
	SEC_MSG_BUCKET2_SIZE,
	SEC_MSG_BUCKET3_SIZE,
	SEC_MSG_BUCKET4_SIZE,
	SEC_MSG_BUCKET5_SIZE,
	SEC_MSG_BUCKET6_SIZE,
	SEC_MSG_BUCKET7_SIZE,
};

enum sec_msgring_credit_config {

	SEC_CC_CPU0_0                        = 0x380,
	SEC_CC_CPU1_0                        = 0x388,
	SEC_CC_CPU2_0                        = 0x390,
	SEC_CC_CPU3_0                        = 0x398,
	SEC_CC_CPU4_0                        = 0x3a0,
	SEC_CC_CPU5_0                        = 0x3a8,
	SEC_CC_CPU6_0                        = 0x3b0,
	SEC_CC_CPU7_0                        = 0x3b8

};

enum sec_engine_id {
	SEC_PIPE0,
	SEC_PIPE1,
	SEC_PIPE2,
	SEC_PIPE3,
	SEC_RSA
};

enum sec_cipher {
	SEC_AES256_MODE_HMAC,
	SEC_AES256_MODE,
	SEC_AES256_HMAC,
	SEC_AES256,
	SEC_AES192_MODE_HMAC,
	SEC_AES192_MODE,
	SEC_AES192_HMAC,
	SEC_AES192,
	SEC_AES128_MODE_HMAC,
	SEC_AES128_MODE,
	SEC_AES128_HMAC,
	SEC_AES128,
	SEC_DES_HMAC,
	SEC_DES,
	SEC_3DES,
	SEC_3DES_HMAC,
	SEC_HMAC
};

enum sec_msgrng_msg_ctrl_config {
	SEC_EOP=5,
	SEC_SOP=6,
};


/*
 *  STRUCTS
 */

typedef struct phx_sec_stats {
	__u32    operations;
} phx_sec_stats_t;


typedef struct driver_data {

	phx_sec_stats_t                stats;
	spinlock_t                     lock;
	__u32                          pending_ops;

	volatile unsigned long         *mmio;

	unsigned long                  mem_end;

	__u32                          base_address;
	__u16                          irq;

	int                            id;
	int                            type;
	int                            instance;

} driver_data_t, *driver_data_pt;


symkey_desc_pt phxsec_alloc_symkey_desc(void);
void phxsec_free_symkey_desc(symkey_desc_pt desc);
pubkey_desc_pt phxsec_alloc_pubkey_desc(void);
void phxsec_free_pubkey_desc(pubkey_desc_pt desc);
void phxsec_msgring_handler(int bucket, int size, int code, int stid,
		struct msgrng_msg *msg, void *data);
phxsec_error_t phxsec_submit_op_wait(symkey_desc_pt desc);
phxsec_error_t phxsec_submit_pkop_wait(pubkey_desc_pt desc);

phxsec_status_t phxsec_init_device(unsigned short dev_id,
		unsigned long base_address,  unsigned int irq,
		unsigned int num_descriptors,
		dev_info_pt dev_ctx);

phxsec_status_t phxsec_shutdown(dev_info_pt dev_ctx);
phxsec_status_t phxsec_reset(dev_info_pt dev_ctx);

/* Stats-related section */

#define PHXDRV_PROFILE_DES                 0x00000001
#define PHXDRV_PROFILE_3DES                0x00000002
#define PHXDRV_PROFILE_AES                 0x00000004
#ifdef B0
#define PHXDRV_PROFILE_ARC4                 0x00000008
#endif /* B0 */
#define PHXDRV_PROFILE_MD5                 0x00000010
#define PHXDRV_PROFILE_SHA1                0x00000020
#define PHXDRV_PROFILE_SHA256              0x00000040
#define PHXDRV_PROFILE_MODEXP              0x00000080
#define PHXDRV_PROFILE_COMBINED            0x00000100
#define PHXDRV_PROFILE_HMAC_REVERTS        0x00010000
#define PHXDRV_PROFILE_CPHR_REVERTS        0x00020000
#define PHXDRV_PROFILE_UNALIGNED_AUTH_DEST 0x10000000

typedef struct hmac_stats
{
	unsigned long md5_count;
	unsigned long long md5_bytes;
	unsigned long sha1_count;
	unsigned long long sha1_bytes;
	unsigned long sha256_count;
	unsigned long long sha256_bytes;
	unsigned long reverts;
	unsigned long long reverts_bytes;
} hmac_stats_t, *hmac_stats_pt;

typedef struct cipher_stats
{
	unsigned long des_encrypts;
	unsigned long long des_encrypt_bytes;
	unsigned long des_decrypts;
	unsigned long long des_decrypt_bytes;
	unsigned long des3_encrypts;
	unsigned long long des3_encrypt_bytes;
	unsigned long des3_decrypts;
	unsigned long long des3_decrypt_bytes;
	unsigned long aes_encrypts;
	unsigned long long aes_encrypt_bytes;
	unsigned long aes_decrypts;
	unsigned long long aes_decrypt_bytes;
#ifdef B0
	unsigned long arc4_encrypts;
	unsigned long long arc4_encrypt_bytes;
	unsigned long arc4_decrypts;
	unsigned long long arc4_decrypt_bytes;
#endif /* B0 */
	unsigned long reverts;
	unsigned long long reverts_bytes;
} cipher_stats_t, *cipher_stats_pt;

typedef struct modexp_stats
{
	unsigned long modexp_512s;
	unsigned long modexp_1024s;
} modexp_stats_t, *modexp_stats_pt;

typedef struct opt_stats
{
	unsigned long combined;
	unsigned long unaligned_auth_dest;
	unsigned long sym_failed;
	unsigned long modexp_failed;
} opt_stats_t, *opt_stats_pt;

typedef struct rmisec_stats
{
	unsigned int stats_mask;
	unsigned int control_mask;
	rwlock_t rmisec_control_lock;
	rwlock_t rmisec_stats_lock;
	char clear_start[0];
	hmac_stats_t hmac;
	cipher_stats_t cipher;
	modexp_stats_t modexp;
	opt_stats_t opt;
} rmisec_stats_t, *rmisec_stats_pt;

/* Control-related section */

#define PHXDRV_CONTROL_HW_DES          0x00000001
#define PHXDRV_CONTROL_HW_3DES         0x00000002
#define PHXDRV_CONTROL_HW_AES          0x00000004
#ifdef B0
#define PHXDRV_CONTROL_HW_ARC4         0x00000008
#endif /* B0 */
#define PHXDRV_CONTROL_HW_MD5          0x00000010
#define PHXDRV_CONTROL_HW_SHA1         0x00000020
#define PHXDRV_CONTROL_HW_SHA256       0x00000040
#define PHXDRV_CONTROL_HW_COMBINED     0x00008000
#define PHXDRV_CONTROL_SILENT          0x20000000
#define PHXDRV_CONTROL_VERBOSE         0x40000000
#define PHXDRV_CONTROL_VVERBOSE        0x80000000

#define PHXDRV_VERY_VERBOSE            2
#define PHXDRV_VERBOSE                 1
#define PHXDRV_NORMAL                  0
#define PHXDRV_SILENT                 -1

/* stats routines */

static void inline phxdrv_record_des(rmisec_stats_pt stats, int enc,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_DES) {
		write_lock(&stats->rmisec_stats_lock);
		if (enc) {
			stats->cipher.des_encrypts++;
			stats->cipher.des_encrypt_bytes += nbytes;
		}
		else {
			stats->cipher.des_decrypts++;
			stats->cipher.des_decrypt_bytes += nbytes;
		}
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_3des(rmisec_stats_pt stats, int enc,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_3DES) {
		write_lock(&stats->rmisec_stats_lock);
		if (enc) {
			stats->cipher.des3_encrypts++;
			stats->cipher.des3_encrypt_bytes += nbytes;
		}
		else {
			stats->cipher.des3_decrypts++;
			stats->cipher.des3_decrypt_bytes += nbytes;
		}
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_aes(rmisec_stats_pt stats, int enc,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_AES) {
		write_lock(&stats->rmisec_stats_lock);
		if (enc) {
			stats->cipher.aes_encrypts++;
			stats->cipher.aes_encrypt_bytes += nbytes;
		}
		else {
			stats->cipher.aes_decrypts++;
			stats->cipher.aes_decrypt_bytes += nbytes;
		}
		write_unlock(&stats->rmisec_stats_lock);
	}
}

#ifdef B0
static void inline phxdrv_record_arc4(rmisec_stats_pt stats, int enc,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_ARC4) {
		write_lock(&stats->rmisec_stats_lock);
		if (enc) {
			stats->cipher.arc4_encrypts++;
			stats->cipher.arc4_encrypt_bytes += nbytes;
		}
		else {
			stats->cipher.arc4_decrypts++;
			stats->cipher.arc4_decrypt_bytes += nbytes;
		}
		write_unlock(&stats->rmisec_stats_lock);
	}
}
#endif /* B0 */

static void inline phxdrv_record_modexp(rmisec_stats_pt stats, 
		int blksize)
{
	if (stats->stats_mask & PHXDRV_PROFILE_MODEXP) {
		write_lock(&stats->rmisec_stats_lock);
		if (blksize == 512) {
			stats->modexp.modexp_512s++;
		}
		if (blksize == 1024) {
			stats->modexp.modexp_1024s++;
		}
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_cipher_revert(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_CPHR_REVERTS) {
		write_lock(&stats->rmisec_stats_lock);
		stats->cipher.reverts++;
		stats->cipher.reverts_bytes += nbytes;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_hmac_revert(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_HMAC_REVERTS) {
		write_lock(&stats->rmisec_stats_lock);
		stats->hmac.reverts++;
		stats->hmac.reverts_bytes += nbytes;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_md5(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_MD5) {
		write_lock(&stats->rmisec_stats_lock);
		stats->hmac.md5_count++;
		stats->hmac.md5_bytes += nbytes;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_sha1(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_SHA1) {
		write_lock(&stats->rmisec_stats_lock);
		stats->hmac.sha1_count++;
		stats->hmac.sha1_bytes += nbytes;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_sha256(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_SHA256) {
		write_lock(&stats->rmisec_stats_lock);
		stats->hmac.sha256_count++;
		stats->hmac.sha256_bytes += nbytes;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_unaligned_auth_dest(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_UNALIGNED_AUTH_DEST) {
		write_lock(&stats->rmisec_stats_lock);
		stats->opt.unaligned_auth_dest++;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_combined(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_COMBINED) {
		write_lock(&stats->rmisec_stats_lock);
		stats->opt.combined++;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_sym_failed(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_COMBINED) {
		write_lock(&stats->rmisec_stats_lock);
		stats->opt.sym_failed++;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

static void inline phxdrv_record_modexp_failed(rmisec_stats_pt stats,
		int nbytes)
{
	if (stats->stats_mask & PHXDRV_PROFILE_COMBINED) {
		write_lock(&stats->rmisec_stats_lock);
		stats->opt.modexp_failed++;
		write_unlock(&stats->rmisec_stats_lock);
	}
}

#endif /* _PHXDRV_H_ */
