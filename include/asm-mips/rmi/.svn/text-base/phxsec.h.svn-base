/*
 * Copyright (C) 2003 Raza Foundries
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


#ifndef _PHXSEC_H_
#define _PHXSEC_H_


/* 
 *  Wrapper OS types.
 */
#ifndef OS_DeviceInfo_t
#define OS_DeviceInfo_t void *
#endif

#ifndef OS_MemHandle_t
#define OS_MemHandle_t uint64_t 
#endif

#ifdef __KERNEL__

#include <linux/completion.h>

#ifndef OS_ALLOC_KERNEL
#include <linux/gfp.h>
#define OS_ALLOC_KERNEL(size) kmalloc((size), GFP_KERNEL)
#endif

#ifndef OS_FREE
#define OS_FREE(mem) kfree((mem))
#endif

#ifndef OS_SEMAPHORE_INIT
#define OS_SEMAPHORE_INIT(sema) sema_init((sema),0)
#endif

#ifndef OS_SEMAPHORE_GET
#define OS_SEMAPHORE_GET(sema) down_interruptible((sema))
#endif

#ifndef OS_SEMAPHORE_SET
#define OS_SEMAPHORE_SET(sema) up((sema))
#endif


#ifndef OS_WAIT_INIT
#define OS_WAIT_INIT(event) init_completion((event))
#endif

#ifndef OS_WAIT
#define OS_WAIT(event) wait_for_completion((event))
#endif

#ifndef OS_WAIT_END
#define OS_WAIT_END(event) complete((event))
#endif

#ifndef OS_WAIT_US
#include <asm/delay.h>
#define OS_WAIT_US(us) udelay((us))
#endif

#endif /* __KERNEL__ */

/* 
 * Version of the device library
 */
#define PHXSEC_VERSION_MAJOR 0x1
#define PHXSEC_VERSION_MINOR 0x0
/* Single alphanumeric character, start with blank, then a,b,c... */
#define PHXSEC_VERSION_REV   ' '  

/*
 * Cryptographic parameter definitions
 */
#define PHXSEC_DES_KEY_LENGTH        8  /* Bytes */
#define PHXSEC_3DES_KEY_LENGTH       24 /* Bytes */
#define PHXSEC_AES128_KEY_LENGTH     16 /* Bytes */
#define PHXSEC_AES192_KEY_LENGTH     24 /* Bytes */
#define PHXSEC_AES256_KEY_LENGTH     32 /* Bytes */
#define PHXSEC_MAX_CRYPT_KEY_LENGTH  PHXSEC_AES256_KEY_LENGTH

#define PHXSEC_DES_IV_LENGTH         8  /* Bytes */
#define PHXSEC_AES_IV_LENGTH         16 /* Bytes */
#ifdef B0
#define PHXSEC_ARC4_IV_LENGTH        0  /* Bytes */
#endif /* B0 */
#define PHXSEC_MAX_IV_LENGTH         16 /* Bytes */
#define PHXSEC_IV_LENGTH_BYTES       8  /* Bytes */

#define PHXSEC_AES_BLOCK_SIZE        16 /* Bytes */
#define PHXSEC_DES_BLOCK_SIZE        8  /* Bytes */
#define PHXSEC_3DES_BLOCK_SIZE       8  /* Bytes */

#define PHXSEC_MD5_BLOCK_SIZE        64 /* Bytes */
#define PHXSEC_SHA1_BLOCK_SIZE       64 /* Bytes */
#define PHXSEC_SHA256_BLOCK_SIZE     64 /* Bytes */
#define PHXSEC_MAX_BLOCK_SIZE        64 /* Max of MD5/SHA */
#define PHXSEC_MD5_LENGTH            16 /* Bytes */
#define PHXSEC_SHA1_LENGTH           20 /* Bytes */
#define PHXSEC_SHA256_LENGTH         32 /* Bytes */
#define PHXSEC_HMAC_LENGTH           32 /* Max of MD5/SHA */
#ifdef B0
#define PHXSEC_MAX_RC4_STATE_SIZE    264 /* char s[256], int i, int j */
#endif /* B0 */


/* Status code is used by the SRL to indicate status */
typedef long phxsec_status_t;

/*
 * Status codes
 */
#define PHXSEC_STATUS_SUCCESS              0
#define PHXSEC_STATUS_NO_DEVICE           -1
#define PHXSEC_STATUS_TIMEOUT             -2
#define PHXSEC_STATUS_INVALID_PARAMETER   -3
#define PHXSEC_STATUS_DEVICE_FAILED       -4
#define PHXSEC_STATUS_DEVICE_BUSY         -5
#define PHXSEC_STATUS_NO_RESOURCE         -6
#define PHXSEC_STATUS_CANCELLED           -7


/* Error code is used to indicate any errors */
typedef long phxsec_error_t;

/*
 */
#define PHXSEC_ERR_NONE                    0
#define PHXSEC_ERR_CIPHER_OP              -1
#define PHXSEC_ERR_CIPHER_TYPE            -2
#define PHXSEC_ERR_CIPHER_MODE            -3
#define PHXSEC_ERR_CIPHER_INIT            -4
#define PHXSEC_ERR_DIGEST_TYPE            -5
#define PHXSEC_ERR_DIGEST_INIT            -6
#define PHXSEC_ERR_DIGEST_SRC             -7
#define PHXSEC_ERR_CKSUM_TYPE             -8
#define PHXSEC_ERR_CKSUM_SRC              -9
#define PHXSEC_ERR_ALLOC                  -10
#define PHXSEC_ERR_CONTROL_VECTOR         -11
#define PHXSEC_ERR_LOADHMACKEY_MODE       -12
#define PHXSEC_ERR_PADHASH_MODE           -13
#define PHXSEC_ERR_HASHBYTES_MODE         -14
#define PHXSEC_ERR_NEXT_MODE              -15
#define PHXSEC_ERR_PKT_IV_MODE            -16
#define PHXSEC_ERR_LASTWORD_MODE          -17
#define PHXSEC_ERR_PUBKEY_OP              -18
#define PHXSEC_ERR_SYMKEY_MSGSND          -19
#define PHXSEC_ERR_PUBKEY_MSGSND          -20
#define PHXSEC_ERR_SYMKEY_GETSEM          -21
#define PHXSEC_ERR_PUBKEY_GETSEM          -22
#define PHXSEC_ERR_SYMKEY_OP              -23


/*
 * Descriptor Vector quantities 
 *  (helps to identify descriptor type per operation)
 */
#define PHX_VECTOR_CIPHER_DES             0x0001
#define PHX_VECTOR_CIPHER_3DES            0x0002
#define PHX_VECTOR_CIPHER_AES128          0x0004
#define PHX_VECTOR_CIPHER_AES192          0x0008
#define PHX_VECTOR_CIPHER_AES256          0x0010
#ifdef B0
#define PHX_VECTOR_CIPHER_ARC4            0x0020
#endif /* B0 */
#define PHX_VECTOR_CIPHER_AES             (PHX_VECTOR_CIPHER_AES128 | \
		PHX_VECTOR_CIPHER_AES192 | \
		PHX_VECTOR_CIPHER_AES256)
#ifdef B0
#define PHX_VECTOR_CIPHER                 (PHX_VECTOR_CIPHER_DES | \
		PHX_VECTOR_CIPHER_3DES | \
		PHX_VECTOR_CIPHER_AES128 | \
		PHX_VECTOR_CIPHER_AES192 | \
		PHX_VECTOR_CIPHER_AES256 | \
		PHX_VECTOR_CIPHER_ARC4)
#else /* B0 */

#define PHX_VECTOR_CIPHER                 (PHX_VECTOR_CIPHER_DES | \
		PHX_VECTOR_CIPHER_3DES | \
		PHX_VECTOR_CIPHER_AES128 | \
		PHX_VECTOR_CIPHER_AES192 | \
		PHX_VECTOR_CIPHER_AES256)
#endif /* B0 */

#define PHX_VECTOR_HMAC                   0x0040
#define PHX_VECTOR_MAC                    0x0080
#define PHX_VECTOR_MODE_CTR_CFB           0x0100
#define PHX_VECTOR_MODE_ECB_CBC_OFB       0x0200
#define PHX_VECTOR_MODE_ECB_CBC           0x0400
#ifdef B0
#define PHX_VECTOR_STATE                  0x0800
#endif /* B0 */


#ifdef B0
#define PHX_VECTOR_CIPHER_ARC4__HMAC  \
(PHX_VECTOR_CIPHER_ARC4 | PHX_VECTOR_HMAC)
#define PHX_VECTOR_CIPHER_ARC4__STATE  \
(PHX_VECTOR_CIPHER_ARC4 | PHX_VECTOR_STATE)
#define PHX_VECTOR_CIPHER_ARC4__HMAC__STATE  \
(PHX_VECTOR_CIPHER_ARC4 | PHX_VECTOR_HMAC | PHX_VECTOR_STATE)
#endif /* B0 */

#define PHX_VECTOR__CIPHER_DES__HMAC__MODE_ECB_CBC \
(PHX_VECTOR_CIPHER_DES | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_ECB_CBC)

#define PHX_VECTOR__CIPHER_DES__MODE_ECB_CBC \
(PHX_VECTOR_CIPHER_DES | PHX_VECTOR_MODE_ECB_CBC)

#define PHX_VECTOR__CIPHER_3DES__HMAC__MODE_ECB_CBC \
(PHX_VECTOR_CIPHER_3DES | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_ECB_CBC)

#define PHX_VECTOR__CIPHER_3DES__MODE_ECB_CBC \
(PHX_VECTOR_CIPHER_3DES | PHX_VECTOR_MODE_ECB_CBC)

#define PHX_VECTOR__CIPHER_AES128__HMAC__MODE_CTR_CFB \
(PHX_VECTOR_CIPHER_AES128 | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_CTR_CFB)

#define PHX_VECTOR__CIPHER_AES128__MODE_CTR_CFB \
(PHX_VECTOR_CIPHER_AES128 | PHX_VECTOR_MODE_CTR_CFB)

#define PHX_VECTOR__CIPHER_AES128__HMAC__MODE_ECB_CBC_OFB \
(PHX_VECTOR_CIPHER_AES128 | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_ECB_CBC_OFB)

#define PHX_VECTOR__CIPHER_AES128__MODE_ECB_CBC_OFB \
(PHX_VECTOR_CIPHER_AES128 | PHX_VECTOR_MODE_ECB_CBC_OFB)

#define PHX_VECTOR__CIPHER_AES192__HMAC__MODE_CTR_CFB \
(PHX_VECTOR_CIPHER_AES192 | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_CTR_CFB)

#define PHX_VECTOR__CIPHER_AES192__MODE_CTR_CFB \
(PHX_VECTOR_CIPHER_AES192 | PHX_VECTOR_MODE_CTR_CFB)

#define PHX_VECTOR__CIPHER_AES192__HMAC__MODE_ECB_CBC_OFB \
(PHX_VECTOR_CIPHER_AES192 | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_ECB_CBC_OFB)

#define PHX_VECTOR__CIPHER_AES192__MODE_ECB_CBC_OFB \
(PHX_VECTOR_CIPHER_AES192 | PHX_VECTOR_MODE_ECB_CBC_OFB)

#define PHX_VECTOR__CIPHER_AES256__HMAC__MODE_CTR_CFB \
(PHX_VECTOR_CIPHER_AES256 | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_CTR_CFB)

#define PHX_VECTOR__CIPHER_AES256__MODE_CTR_CFB \
(PHX_VECTOR_CIPHER_AES256 | PHX_VECTOR_MODE_CTR_CFB)

#define PHX_VECTOR__CIPHER_AES256__HMAC__MODE_ECB_CBC_OFB \
(PHX_VECTOR_CIPHER_AES256 | PHX_VECTOR_HMAC | PHX_VECTOR_MODE_ECB_CBC_OFB)

#define PHX_VECTOR__CIPHER_AES256__MODE_ECB_CBC_OFB \
(PHX_VECTOR_CIPHER_AES256 | PHX_VECTOR_MODE_ECB_CBC_OFB)





	/*
	 * Cipher Modes
	 */
	typedef enum {
		PHX_CIPHER_MODE_NONE = 0,
		PHX_CIPHER_MODE_PASS = 1,
		PHX_CIPHER_MODE_ECB,
		PHX_CIPHER_MODE_CBC, 
		PHX_CIPHER_MODE_OFB,
		PHX_CIPHER_MODE_CTR,
		PHX_CIPHER_MODE_CFB
	} PHX_CIPHER_MODE;

typedef enum {
	PHX_CIPHER_OP_NONE = 0,
	PHX_CIPHER_OP_ENCRYPT = 1,
	PHX_CIPHER_OP_DECRYPT
} PHX_CIPHER_OP;

typedef enum {
	PHX_CIPHER_TYPE_UNSUPPORTED = -1,
	PHX_CIPHER_TYPE_NONE = 0,
	PHX_CIPHER_TYPE_DES,
	PHX_CIPHER_TYPE_3DES,
	PHX_CIPHER_TYPE_AES128,
	PHX_CIPHER_TYPE_AES192,
	PHX_CIPHER_TYPE_AES256,
#ifdef B0
	PHX_CIPHER_TYPE_ARC4
#endif /* B0 */
} PHX_CIPHER_TYPE;

typedef enum {
#ifdef B0
	PHX_CIPHER_INIT_OK = 1,   /* Preserve old Keys */
	PHX_CIPHER_INIT_NK        /*Load new Keys */
#else /* B0 */
		PHX_CIPHER_INIT_OVKM = 1, /*Preserve old IV/(Keys,NonceCFBMask)*/
	PHX_CIPHER_INIT_NV_OKM,   /*Preserve old IV/(Keys,NonceCFBMask)*/
	PHX_CIPHER_INIT_NKM_OV,   /*Load new Keys,NonceCFBMask use old IV*/
	PHX_CIPHER_INIT_NVKM      /*Load new IV/(Keys,NonceCFBMask)*/
#endif /* B0 */
} PHX_CIPHER_INIT;


/*
 *  Hash Modes
 */
typedef enum {
	PHX_DIGEST_TYPE_UNSUPPORTED = -1,
	PHX_DIGEST_TYPE_NONE = 0,
	PHX_DIGEST_TYPE_MD5,
	PHX_DIGEST_TYPE_SHA1,
	PHX_DIGEST_TYPE_SHA256,
	PHX_DIGEST_TYPE_HMAC_MD5,
	PHX_DIGEST_TYPE_HMAC_SHA1,
	PHX_DIGEST_TYPE_HMAC_SHA256,
	PHX_DIGEST_TYPE_HMAC_AES_CBC,
	PHX_DIGEST_TYPE_HMAC_AES_XCBC
} PHX_DIGEST_TYPE;

typedef enum {
	PHX_DIGEST_INIT_OLDKEY = 1, /* Preserve old key HMAC key stored in ID registers (moot if HASH.HMAC == 0) */
	PHX_DIGEST_INIT_NEWKEY      /*Load new HMAC key from memory ctrl section to ID registers */
} PHX_DIGEST_INIT;

typedef enum {
	PHX_DIGEST_SRC_DMA = 1, /* DMA channel */
	PHX_DIGEST_SRC_CPHR     /*Cipher if word count exceeded Cipher_Offset; else DMA */
} PHX_DIGEST_SRC;

/*
 *  Checksum Modes
 */
typedef enum {
	PHX_CKSUM_TYPE_NOP = 1,
	PHX_CKSUM_TYPE_IP
} PHX_CKSUM_TYPE;

typedef enum {
	PHX_CKSUM_SRC_DMA    = 1,
	PHX_CKSUM_SRC_CIPHER
} PHX_CKSUM_SRC;


/*
 *  Packet Modes
 */
typedef enum {
	PHX_LOADHMACKEY_MODE_OLD = 1,
	PHX_LOADHMACKEY_MODE_LOAD
} PHX_LOADHMACKEY_MODE;

typedef enum {
	PHX_PADHASH_PADDED = 1,
	PHX_PADHASH_PAD
} PHX_PADHASH_MODE;

typedef enum {
	PHX_HASHBYTES_ALL8 = 1,
	PHX_HASHBYTES_MSB,
	PHX_HASHBYTES_MSW
} PHX_HASHBYTES_MODE;

typedef enum {
	PHX_NEXT_FINISH = 1,
	PHX_NEXT_DO
} PHX_NEXT_MODE;

typedef enum {
	PHX_PKT_IV_OLD = 1,
	PHX_PKT_IV_NEW
} PHX_PKT_IV_MODE;

typedef enum {
	PHX_LASTWORD_128 = 1,
	PHX_LASTWORD_96MASK,
	PHX_LASTWORD_64MASK,
	PHX_LASTWORD_32MASK
} PHX_LASTWORD_MODE;


/*
 *  Public Key
 */
typedef enum {
	RMIPK_BLKWIDTH_512 = 1,
	RMIPK_BLKWIDTH_1024 
} RMIPK_BLKWIDTH_MODE;

typedef enum {
	RMIPK_LDCONST_OLD = 1,
	RMIPK_LDCONST_NEW
} RMIPK_LDCONST_MODE;


/*
 * Bulk encryption/decryption ioctl
 */
typedef struct phxsec_io_s {
	uint32_t                  command;
	uint32_t          result_status;
	uint32_t          flags;
	uint32_t          session_num;
	uint32_t          use_callback;
	uint32_t         time_us;
	uint32_t         user_context[2];/*usable for anything by caller*/
	uint32_t         command_context; /* Context (ID) of this command). */

	unsigned char         initial_vector[PHXSEC_MAX_IV_LENGTH];
	unsigned char         crypt_key[PHXSEC_MAX_CRYPT_KEY_LENGTH]; 
	unsigned char         mac_key[PHXSEC_MAX_BLOCK_SIZE]; 

	PHX_CIPHER_OP         cipher_op;
	PHX_CIPHER_MODE       cipher_mode;
	PHX_CIPHER_TYPE       cipher_type;
	PHX_CIPHER_INIT       cipher_init;
	uint32_t          cipher_offset;

	PHX_DIGEST_TYPE       digest_type;
	PHX_DIGEST_INIT       digest_init;
	PHX_DIGEST_SRC        digest_src;
	uint32_t          digest_offset;

	PHX_CKSUM_TYPE        cksum_type;
	PHX_CKSUM_SRC         cksum_src;
	uint32_t          cksum_offset;

	PHX_LOADHMACKEY_MODE  pkt_hmac;
	PHX_PADHASH_MODE      pkt_hash;
	PHX_HASHBYTES_MODE    pkt_hashbytes;
	PHX_NEXT_MODE         pkt_next;
	PHX_PKT_IV_MODE       pkt_iv;
	PHX_LASTWORD_MODE     pkt_lastword;

	uint32_t         nonce;
	uint32_t          cfb_mask;

	uint32_t          iv_offset;
	uint16_t        pad_type;
#ifdef B0
	uint16_t        rc4_key_len;
#endif /* B0 */

	uint32_t          num_packets;
	uint32_t          num_fragments;

	uint64_t        source_buf;
	uint32_t          source_buf_size;
	uint64_t         dest_buf;
	uint32_t          dest_buf_size;

	uint64_t         auth_dest;
	uint64_t        cksum_dest;

#ifdef B0
	uint16_t        rc4_loadstate;
	uint16_t        rc4_savestate;
	uint64_t         rc4_state;
#endif /* B0 */

} phxsec_io_t, *phxsec_io_pt;



/*
 * The long key type is used as a generic type to hold public
 * key information.
 * KeyValue points to an array of 32-bit integers. The convention of these keys
 * is such that element[0] of this array holds the least significant part of
 * the "bignum" (multi-precision integer).
 * Keylength holds the number of significant bits in the key, i.e. the bit
 * position of the most significant "1" bit, plus 1.
 * For example, the multi-precision integer ("key")
 *    0x0102030405060708090A0B0C0D0E0F00
 * has 121 significant bits (KeyLength), and would be arranged in the N-element 
 * array (pointed to by KeyLength) of 32-bit integers as
 *    array[0] = 0x0D0E0F00
 *    array[1] = 0x090A0B0C
 *    array[2] = 0x05060708
 *    array[3] = 0x01020304
 *    array[4] = 0x00000000
 *        ...
 *    array[N-1] = 0x00000000
 */
typedef struct phxsec_LongKey_s {
	unsigned long   KeyLength;	/* length in bits */
	OS_MemHandle_t  KeyValue;	/* pointer to 32-bit integer "key" array */
} phxsec_LongKey_t,*phxsec_LongKey_pt;



/*
 * The bignum type is used as a generic type to hold the multi-precision integer
 * values used in public key calculations.
 * bignum points to an array of 32-bit integers. The convention of these keys
 * is such that element[0] of this array holds the least significant part of
 * the "bignum".
 * bignum_length holds the number of significant bits in the key, i.e. the bit
 * position of the most significant "1" bit, plus 1.
 * For example, the multi-precision integer ("bignum")
 *    0x0102030405060708090A0B0C0D0E0F00
 * has 121 significant bits (bignum_length), 
 *  and would be arranged in the N-element  array 
 *  (pointed to by bignum) of 32-bit integers as
 *    array[0] = 0x0D0E0F00
 *    array[1] = 0x090A0B0C
 *    array[2] = 0x05060708
 *    array[3] = 0x01020304
 *    array[4] = 0x00000000
 *        ...
 *    array[N-1] = 0x00000000
 */
typedef struct phxsec_bignum_s {
	uint32_t   bignum_length;   /* length in bits */
	OS_MemHandle_t  bignum;          /* pointer to 32-bit integer array */
} phxsec_bignum_t,*phxsec_bignum_pt;


/*
 * Diffie-Hellman parameter type definition.
 */
typedef struct phxsec_DH_Params_t {
	phxsec_bignum_t Y;		/* Public value, in (PHXSEC_DH_SHARED), out (PHXSEC_DH_PUBLIC) */
	phxsec_bignum_t X;		/* Secret value, in (PHXSEC_DH_SHARED), out (PHXSEC_DH_PUBLIC) */
	phxsec_bignum_t K;		/* Shared secret value, out (PHXSEC_DH_SHARED) */
	phxsec_bignum_t N;   		/* Modulus, in (PHXSEC_DH_SHARED), out (PHXSEC_DH_PUBLIC) */
	phxsec_bignum_t G;	  	/* Generator, in (PHXSEC_DH_PUBLIC) */
} phxsec_DH_op_t,*phxsec_DH_op_pt;


/*
 * RSA parameter type definition.
 */
typedef struct phxsec_RSA_Params_t {
	uint64_t   data_in;    /* InputKeyInfo - Input data. */
	uint64_t   data_out;   /* OutputKeyInfo - Output data. */
	uint16_t  data_in_len; 
	uint16_t  data_out_len; 
	phxsec_bignum_t constant;   /* RMI: Montgomery Product */
	phxsec_bignum_t modulus;    /* ModN - Modulo N value to be applied */
	phxsec_bignum_t exponent;   /* ExpE - BaseG value to be applied. */
} phxsec_RSA_op_t, *phxsec_RSA_op_pt;


typedef enum {
	PHX_PUBKEY_DH_PUBLIC = 1,
	PHX_PUBKEY_DH_SHARED,
	PHX_PUBKEY_RSA_PUBLIC,
	PHX_PUBKEY_RSA_PRIVATE
} PHX_PUBKEY_CMD;


/* 
 * Generic key command parameters
 */
typedef union phxsec_pubkey_u {
	phxsec_DH_op_t  DH_params;   /* DH parameters  */
	phxsec_RSA_op_t RSA_params;  /* RSA Parameters */
} phxsec_pubkey_t;
typedef  phxsec_pubkey_t *phxsec_pubkey_pt;


/*
 * Key setup ioctl
 */
typedef struct phxsec_key_io_s {
	uint32_t                command;
	uint32_t        result_status;
	uint32_t       command_context; /* Context (ID) of this command). */
	uint32_t      time_us;
	uint32_t       user_context[2]; /* usable for anything by caller */
	phxsec_pubkey_t     key;
} phxsec_key_io_t;
typedef phxsec_key_io_t *phxsec_key_io_pt;


/*
 * PHXDRV Statistics information contains all statistics 
 * maintained by the driver.
 */
/*
   typedef struct phxsec_Statistics_s {
   unsigned long BlocksEncryptedCount;
   unsigned long BlocksDecryptedCount;
   unsigned long BytesEncryptedCount;
   unsigned long BytesDecryptedCount;
   unsigned long CryptoFailedCount;
   unsigned long DHPublicCount;
   unsigned long DHSharedCount;
   unsigned long RSAPublicCount;
   unsigned long RSAPrivateCount;
   unsigned long DMAErrorCount;
   } phxsec_Statistics_t, *phxsec_Statistics_pt;
 */


int phxsec_cipherdigestuserop(void *ctx, phxsec_io_pt op);
int phxsec_publickeyuserop(void *ctx,  phxsec_key_io_pt op);

/* this initializes CryptoAPI interface */
int phxcrypto_init(void *ctx);

/* these functions are used by the CryptoAPI to alloc/dealloc descriptors */
void *phxsec_AllocateDescriptor(void *ctx);
int phxsec_FreeDescriptor(void *ctx, void *desc);

/* this is used by the CryptoAPI to fill in descriptors */
phxsec_error_t phxsec_FillDescriptor(phxsec_io_pt op, void *desc, unsigned int *cfg_vector);

/* this is passed to packet setup to optimize */
#define PHX_SETUP_OP_CIPHER              0x00000001
#define PHX_SETUP_OP_HMAC                0x00000002
#define PHX_SETUP_OP_CIPHER_HMAC         (PHX_SETUP_OP_CIPHER | PHX_SETUP_OP_HMAC)
/* this is passed to control_setup to update w/preserving existing keys */
#define PHX_SETUP_OP_PRESERVE_HMAC_KEY    0x80000000
#define PHX_SETUP_OP_PRESERVE_CIPHER_KEY  0x40000000
#define PHX_SETUP_OP_UPDATE_KEYS          0x00000010
#define PHX_SETUP_OP_FLIP_3DES_KEY        0x00000020

/* this sets up per-packet descriptor */
phxsec_error_t phxsec_SetupDescriptor(phxsec_io_pt op, unsigned int flags,
		void *desc,
		unsigned int vector);

/* this processes the packet */
int phxsec_ProcessPacket(void *ctx, void *desc, unsigned int cfg_vector);

/* debugging function */
void phxsec_DecodeDescriptor (void *desc, unsigned int cfg_vector);

#endif /* _PHXSEC_H_ */
