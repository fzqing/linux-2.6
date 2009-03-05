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
 * phxdesc.h:  Security Engine descriptors for the PHXSEC Software Reference
 *             Library.
 *
 * Revision History:
 *
 * 04/01/2004  DPK   Created
 */

#ifndef _PHXDESC_H_
#define _PHXDESC_H_

#include <asm/semaphore.h>

#define ONE_BIT              0x0000000000000001ULL
#define TWO_BITS             0x0000000000000003ULL
#define THREE_BITS           0x0000000000000007ULL
#define FOUR_BITS            0x000000000000000fULL
#define FIVE_BITS            0x000000000000001fULL
#define SEVEN_BITS           0x000000000000007fULL
#define EIGHT_BITS           0x00000000000000ffULL
#define NINE_BITS            0x00000000000001ffULL
#define ELEVEN_BITS          0x00000000000007ffULL
#define TWELVE_BITS          0x0000000000000fffULL
#define FOURTEEN_BITS        0x0000000000003fffULL
#define TWENTYFOUR_BITS      0x0000000000ffffffULL
#define THIRTY_TWO_BITS      0x00000000ffffffffULL
#define THIRTY_FIVE_BITS     0x00000007ffffffffULL
#define FOURTY_BITS          0x000000ffffffffffULL

#define MSG_IN_CTL_LEN_BASE  40
#define MSG_IN_CTL_ADDR_BASE 0

#define GET_FIELD(word,field) \
(((word) & (field ## _MASK)) >> (field ## _LSB))

#define FIELD_VALUE(field,value) (((value) & (field ## _BITS)) << (field ## _LSB))

	/*
	 * NOTE: this macro expects 'word' to be uninitialized (i.e. zeroed)
	 */
#define SET_FIELD(word,field,value) \
{ (word) |=  (((value) & (field ## _BITS)) << (field ## _LSB)); }

/*
 * This macro clears 'word', then sets the value
 */
#define CLEAR_SET_FIELD(word,field,value) \
{ (word) &= ~((field ## _BITS) << (field ## _LSB)); \
	(word) |=  (((value) & (field ## _BITS)) << (field ## _LSB)); }

	/*
	 * NOTE: May be used to build value specific mask 
	 *        (e.g.  GEN_MASK(CTL_DSC_CPHR_3DES,CTL_DSC_CPHR_LSB)
	 */
#define GEN_MASK(bits,lsb) ((bits) << (lsb))




	/*
	 * Security block data and control exchange
	 *
	 * A 2-word message ring descriptor is used to pass a pointer to the control descriptor data structure
	 * and a pointer to the packet descriptor data structure:
	 *
#ifdef B0
	 *  63  61 60                 54      53      52    49 48            45 44    40
	 *  39                                                     5 4                 0
	 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------
	 * | Ctrl | Resp Dest Id Entry0 | IF_L2ALLOC | UNUSED | Control Length | UNUSED
	 * | 35 MSB of address of control descriptor data structure | Software Scratch0
	 * |
	 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------
	 *    3              7                1          4             4           5
	 *    35                                       5
#else * B0 *
	 *  63  61 60                 54      53      52    49 48            45 44    40 39                                                     5 4                 0
	 *  ---------------------------------------------------------------------------------------------------------------------------------
	 * | Ctrl | UNUSED | IF_L2ALLOC | UNUSED | Control Length | UNUSED | 35 MSB of address of control descriptor data structure | UNUSED |
	 *  ---------------------------------------------------------------------------------------------------------------------------------
	 *    3       7          1          5             4           5                              35                                 5
#endif * B0 *
	 *
	 *  63  61 60    54     53          52             51        50    46      45       44    40 39                                                    5 4      0
	 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------
	 * | Ctrl | UNUSED | WRB_COH | WRB_L2ALLOC | DF_PTR_L2ALLOC | UNUSED | Data Length | UNUSED | 35 MSB of address of packet descriptor data structure | UNUSED |
	 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------
	 *    3       7         1          1               1            5           1          5                                35                              5
	 *
	 * Addresses assumed to be cache-line aligned, i.e., Address[4:0] ignored (using 5'h00 instead)
	 *
#ifdef B0
	 * Control length is the number of control cachelines to be read so user needs
	 * to round up
	 * the control length to closest integer multiple of 32 bytes. Note that at
	 * present (08/12/04)
	 * the longest (sensical) ctrl structure is <= 416 bytes, i.e., 13 cachelines.
#else * B0 *
	 * Control length is the number of control cachelines to be read so user needs to round up
	 * the control length to closest integer multiple of 32 bytes. Note that at present (03/18/04)
	 * the longest (sensical) ctrl structure is <= 128 bytes.
#endif * B0 *
	 *
	 * The packet descriptor data structure size is fixed at 1 cacheline (32 bytes).
	 * This effectively makes "Data Length" a Load/NoLoad bit. NoLoad causes an abort.
	 *
	 *
	 * Upon completion of operation, the security block returns a 2-word free descriptor
	 * in the following format:
	 *
	 *  63  61 60            54 53   52 51       49   48   47               40 39                                                  0
	 *  ----------------------------------------------------------------------------------------------------------------------------
	 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | 1'b0 | Instruction Error |    Address of control descriptor data structure     |
	 *  ----------------------------------------------------------------------------------------------------------------------------
	 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | 1'b0 |     Data Error    |    Address of packet descriptor data structure      |
	 *  ----------------------------------------------------------------------------------------------------------------------------
	 *
	 * The Instruction and Data Error codes are enumerated in the 
	 * controldescriptor and PacketDescriptor sections below
	 *
	 */


	/*
	 * Operating assumptions
	 * =====================
	 *
	 * 
	 *	        -> For all IpSec ops, I assume that all the IP/IPSec/TCP headers
	 *		   and the data are present at the specified source addresses.
	 *		   I also assume that all necessary header data already exists
	 *		   at the destination. Additionally, in AH I assume that all
	 *		   mutable fields (IP.{TOS, Flags, Offset, TTL, Header_Checksum}) 
	 *		   and the AH.Authentication_Data have been zeroed by the client.
	 *
	 *
	 *		-> In principle, the HW can calculate TCP checksums on both
	 *		   incoming and outgoing data; however, since the TCP header
	 *		   contains the TCP checksum of the plain payload and the header
	 *		   is encrypted, two passes would be necessary to do checksum + encryption
	 *                 for outgoing messages;
	 *		   therefore the checksum engine will likely only be used during decryption
	 *                 (incoming).
	 *
	 *
	 *		-> For all operations involving TCP checksum, I assume the client has filled
	 *		   the TCP checksum field with the appropriate value:
	 *
	 *			    - 0 for generation phase
	 *			    - actual value for verification phase (expecting 0 result)
	 *
	 *
	 *		-> For ESP tunnel, the original IP header exists between the end of the
	 *		   ESP header and the beginning of the TCP header; it is assumed that the
	 *		   maximum length of this header is 16 k(32bit)words (used in CkSum_Offset).
	 *
	 *
	 *		-> The authentication data is merely written to the destination address;
	 *		   the client is left with the task of comparing to the data in packet
	 *		   in decrypt.
	 *
	 *              -> packetdescriptor_t.dstLLWMask relevant to AES CTR mode only but it will
	 *                 affect all AES-related operations. It will not affect DES/3DES/bypass ops.
	 *                 The mask is applied to data as it emerges from the AES engine for the sole
	 *                 purpose of providing the authenticator and cksum engines with correct data.
	 *                 CAVEAT: the HW does not mask the incoming data. It is the user's responsibility
	 *                 to set to 0 the corresponding data in memory. If the surplus data is not masked
	 *                 in memory, cksum/auth results will be incorrect if those engines receive data
	 *                 straight from memory (i.e., not from cipher, as it happens while decoding)
	 */

#ifdef B0
	/*
	 * Fragmentation and offset related notes
	 * ======================================
	 *
	 *
	 *      A) Rebuilding packets from fragments on dword boundaries. The discussion
	 *         below is exemplified by tests memcpy_all_off_frags and memcpy_same_off_frags
	 * 
	 *	        1) The Offset before data/iv on first fragment is ALWAYS written back
	 *                 Non-zero dst dword or global offsets may cause more data to be 
	 *                 written than the user-specified length.
	 *
	 *
	 *                 Example:
	 *                 --------
	 *
	 *                 Below is a source (first fragment) packet (@ ADD0 cache-aligned address).
	 *                 Assume we just copy it and relevant data starts on
	 *                 dword 3 so Cipher_Offset = IV_Offset = 3 (dwords).
	 *                 D0X denotes relevant data and G denotes dont care data.
	 *                 Offset data is also copied so Packet_Legth = 9 (dwords) * 8 = 72 (bytes)
	 *                 Segment_src_address = ADD0
	 *
	 *                 If we want to, e.g., copy so that the relevant (i.e., D0X) data
	 *                 starts at (cache-aligned address) ADD1, we need to specify
	 *                 Dst_dword_offset = 1 so D00 is moved from dword position 3 to 0 on next cache-line
	 *                 Cipher_dst_address = ADD1 - 0x20 so D00 is written to ADD1
	 *
	 *                 Note that the security engine always writes full cachelines
	 *                 therefore, data written to dword0 0 of ADD1 (denoted w/ ?) is what the sec pipe
	 *                 write back buffer contained from previous op.
	 *
	 *
	 *                      SOURCE:                                                 DESTINATION:
	 *                      -------                                                 ------------
	 *
	 *                      Segment_src_address = ADD0                              Cipher_dst_address = ADD1 - 0x20
	 *                      Packet_Legth        = 72                                Dst_dword_offset   = 1
	 *                      Cipher_Offset       = 3
	 *                      IV_Offset           = 3
	 *                      Use_IV              = ANY
	 *                      
	 *
	 *
	 *                         3     2     1     0                                  3     2     1     0
	 *                       -----------------------                              -----------------------
	 *                      | D00 | G   | G   | G   | <- ADD0                    | G   | G   | G   | ?   | <- ADD1 - 0x20
	 *                       -----------------------                              -----------------------
	 *                      | D04 | D03 | D02 | D01 |                            | D03 | D02 | D01 | D00 | <- ADD1
	 *                       -----------------------                              -----------------------
	 *                      |     |     |     | D05 |                            |     |     | D05 | D04 |
	 *                       -----------------------                              -----------------------
	 * 
	 *	        2) On fragments following the first, IV_Offset is overloaded to mean data offset
	 *                 (number of dwords to skip from beginning of cacheline before starting processing)
	 *                 and Use_IV is overloaded to mean do writeback the offset (in the clear).
	 *                 These fields in combination with Dst_dword_offset allow packet fragments with
	 *                 arbitrary boundaries/lengthd to be reasembled.
	 *
	 *
	 *                 Example:
	 *                 --------
	 *
	 *                 Assume data above was first fragment of a packet we'd like to merge to
	 *                 (second) fragment below located at ADD2. The written data should follow
	 *                 the previous data without gaps or overwrites. To achieve this, one should
	 *                 assert the "Next" field on the previous fragment and use self-explanatory
	 *                 set of parameters below
	 *
	 *
	 *                      SOURCE:                                                 DESTINATION:
	 *                      -------                                                 ------------
	*
	*                      Segment_src_address = ADD2                              Cipher_dst_address = ADD1 + 0x20
	*                      Packet_Legth        = 104                               Dst_dword_offset   = 1
	*                      IV_Offset           = 1
	*                      Use_IV              = 0
	*                      
	*
	*
	*                         3     2     1     0                                  3     2     1     0
	*                       -----------------------                              -----------------------
	*                      | D12 | D11 | D10 | G   | <- ADD2                    | G   | G   | G   | ?   | <- ADD1 - 0x20
	*                       -----------------------                              -----------------------
	*                      | D16 | D15 | D14 | D13 |                            | D03 | D02 | D01 | D00 | <- ADD1
	*                       -----------------------                              -----------------------
	*                      | D1a | D19 | D18 | D17 |                            | D11 | D10 | D05 | D04 | <- ADD1 + 0x20
	*                       -----------------------                              -----------------------
	*                      |     |     |     | D1b |                            | D15 | D14 | D13 | D12 |
	*                       -----------------------                              -----------------------
	*                                                                           | D19 | D18 | D17 | D16 |
	*                                                                            -----------------------
	*                                                                           |     |     | D1b | D1a |
	*                                                                            -----------------------
	*
	*                 It is note-worthy that the merging can only be achieved if Use_IV is 0. Indeed, the security
	*                 engine always writes full lines, therefore ADD1 + 0x20 will be re-written. Setting Use_IV to 0
	*                 will allow the sec pipe write back buffer to preserve D04, D05 from previous frag and only
	*                 receive D10, D11 thereby preserving the integrity of the previous data.
	* 
	*	        3) On fragments following the first, !UseIV in combination w/ Dst_dword_offset >= (4 - IV_Offset)
*                 will cause a wraparound of the write thus achieving all 16 possible (Initial_Location, Final_Location)
	*                 combinations for the data.
	*
	*
	*                 Example:
	*                 --------
	*
	*                 Contiguously merging 2 data sets above with a third located at ADD3. If this is the last fragment, 
	*                 reset its Next bit.
	*
	*
	*                      SOURCE:                                                 DESTINATION:
	*                      -------                                                 ------------
	*
	*                      Segment_src_address = ADD3                              Cipher_dst_address = ADD1 + 0x80
	*                      Packet_Legth        = 152                               Dst_dword_offset   = 3
	*                      IV_Offset           = 3
	*                      Use_IV              = 0
	*                      
	*
	*
	*                         3     2     1     0                                  3     2     1     0
	*                       -----------------------                              -----------------------
	*                      | D20 | G   | G   | G   | <- ADD2                    | G   | G   | G   | ?   | <- ADD1 - 0x20
	*                       -----------------------                              -----------------------
	*                      | D24 | D23 | D22 | D21 |                            | D03 | D02 | D01 | D00 | <- ADD1
	*                       -----------------------                              -----------------------
	*                      | D28 | D27 | D26 | D25 |                            | D11 | D10 | D05 | D04 | <- ADD1 + 0x20
	*                       -----------------------                              -----------------------
	*                      | D2c | D2b | D2a | D29 |                            | D15 | D14 | D13 | D12 |
	*                       -----------------------                              -----------------------
	*                      |     | D2f | D2e | D2d |                            | D19 | D18 | D17 | D16 |
	*                       -----------------------                              -----------------------
	*                                                                           | D21 | D20 | D1b | D1a | <- ADD1 + 0x80
	*                                                                            -----------------------
	*                                                                           | D25 | D24 | D23 | D22 | 
	*                                                                            -----------------------
	*                                                                           | D29 | D28 | D27 | D26 | 
	*                                                                            -----------------------
	*                                                                           | D2d | D2c | D2b | D2a | 
	*                                                                            -----------------------
	*                                                                           |(D2d)|(D2c)| D2f | D2e | 
	*                                                                            -----------------------
	*
	*                 It is worth noticing that always writing full-lines causes the last 2 dwords in the reconstituted
*                 packet to be unnecessarily written: (D2d) and (D2c)
	*
	*
	*
	*      B) Implications of fragmentation on AES
	* 
	*	        1) AES is a 128 bit block cipher; therefore it requires an even dword total data length
	*                 Data fragments (provided there are more than 1) are allowed to have odd dword
	*                 data lengths provided the total length (cumulated over fragments) is an even dword
	*                 count; an error will be generated otherwise, upon receiving the last fragment descriptor
	*                 (see error conditions below).
	*
	*              2) While using fragments with AES, a fragment (other than first) starting with a != 0 (IV) offset
	*                 while the subsequent total dword count given to AES is odd may not be required to write
	*                 its offset (UseIV). Doing so will cause an error (see error conditions below).
	*
	*
	*                 Example:
	*                 --------
	*
*                 Suppose the first fragment has an odd DATA dword count and USES AES (as seen below)
	*
	*                      SOURCE:                                                 DESTINATION:
	*                      -------                                                 ------------
	*
	*                      Segment_src_address = ADD0                              Cipher_dst_address = ADD1
	*                      Packet_Legth        = 64                                Dst_dword_offset   = 1
	*                      Cipher_Offset       = 3
	*                      IV_Offset           = 1
	*                      Use_IV              = 1
	*                      Cipher              = Any AES
	*                      Next                = 1
	*                      
	*
	* 
	*
	*                         3     2     1     0                                  3     2     1     0
	*                       -----------------------                              -----------------------
	*                      | D00 | IV1 | IV0 | G   | <- ADD0                    | E00 | IV1 | IV0 | G   | <- ADD1
	*                       -----------------------                              -----------------------
	*                      | D04 | D03 | D02 | D01 |                            | X   | E03 | E02 | E01 |
	*                       -----------------------                              -----------------------
	*
	*                 At the end of processing of the previous fragment, the AES engine input buffer has D04
	*                 and waits for next dword, therefore the writeback buffer cannot finish writing the fragment
	*                 to destination (X instead of E04).
	*
	*                 If a second fragment now arrives with a non-0 offset and requires the offset data to be
	*                 written to destination, the previous write (still needing the arrival of the last dword
			*                 required by the AES to complete the previous operation) cannot complete before the present
	*                 should start causing a deadlock.
	*/
#endif /* B0 */

	/*
	 *  Command Control Word for Message Ring Descriptor
	 */

	/* #define MSG_CMD_CTL_CTL       */
#define MSG_CMD_CTL_CTL_LSB   61
#define MSG_CMD_CTL_CTL_BITS  THREE_BITS
#define MSG_CMD_CTL_CTL_MASK  (MSG_CMD_CTL_CTL_BITS << MSG_CMD_CTL_CTL_LSB)

	/* #define MSG_CMD_CTL_ID */
#define MSG_CMD_CTL_ID_LSB    54
#define MSG_CMD_CTL_ID_BITS   SEVEN_BITS
#define MSG_CMD_CTL_ID_MASK   (MSG_CMD_CTL_ID_BITS << MSG_CMD_CTL_ID_LSB)

	/* #define MSG_CMD_CTL_LEN */
#define MSG_CMD_CTL_LEN_LSB   45
#ifdef B0
#define MSG_CMD_CTL_LEN_BITS  FOUR_BITS
#else /* B0 */
#define MSG_CMD_CTL_LEN_BITS  THREE_BITS
#endif /* B0*/
#define MSG_CMD_CTL_LEN_MASK  (MSG_CMD_CTL_LEN_BITS << MSG_CMD_CTL_LEN_LSB)


	/* #define MSG_CMD_CTL_ADDR */
#define MSG_CMD_CTL_ADDR_LSB  0 
#define MSG_CMD_CTL_ADDR_BITS FOURTY_BITS
#define MSG_CMD_CTL_ADDR_MASK (MSG_CMD_CTL_ADDR_BITS << MSG_CMD_CTL_ADDR_LSB)

#define MSG_CMD_CTL_MASK      (MSG_CMD_CTL_CTL_MASK | \
		MSG_CMD_CTL_LEN_MASK | MSG_CMD_CTL_ADDR_MASK)

/*
 *  Command Data Word for Message Ring Descriptor
 */

/* #define MSG_IN_DATA_CTL */
#define MSG_CMD_DATA_CTL_LSB   61
#define MSG_CMD_DATA_CTL_BITS  THREE_BITS
#define MSG_CMD_DATA_CTL_MASK  (MSG_CMD_DATA_CTL_BITS  << MSG_CMD_DATA_CTL_LSB)

/* #define MSG_CMD_DATA_LEN */
#define MSG_CMD_DATA_LEN_LOAD  1
#define MSG_CMD_DATA_LEN_LSB   45
#define MSG_CMD_DATA_LEN_BITS  ONE_BIT
#define MSG_CMD_DATA_LEN_MASK  (MSG_CMD_DATA_LEN_BITS << MSG_CMD_DATA_LEN_LSB)

/* #define MSG_CMD_DATA_ADDR */
#define MSG_CMD_DATA_ADDR_LSB  0 
#define MSG_CMD_DATA_ADDR_BITS FOURTY_BITS 
#define MSG_CMD_DATA_ADDR_MASK (MSG_CMD_DATA_ADDR_BITS << MSG_CMD_DATA_ADDR_LSB)

#define MSG_CMD_DATA_MASK      (MSG_CMD_DATA_CTL_MASK | \
		MSG_CMD_DATA_LEN_MASK | MSG_CMD_DATA_ADDR_MASK)


/*
 * Upon completion of operation, the Sec block returns a 2-word free descriptor
 * in the following format:
 *
 *  63  61 60            54 53   52 51       49  48          40 39             0
 *  ----------------------------------------------------------------------------
 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | Control Error | Source Address |
 *  ----------------------------------------------------------------------------
 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl |   Data Error  | Dest Address   |
 *  ----------------------------------------------------------------------------
 *
 * The Control and Data Error codes are enumerated below
 *
 *                                Error conditions
 *                                ================
 *
 *             Control Error Code                  Control Error Condition
 *             ------------------                  ---------------------------
 *             9'h000                              No Error
 *             9'h001                              Unknown Cipher Op                      ( Cipher == 3'h{6,7})
 *             9'h002                              Unknown or Illegal Mode                ((Mode   == 3'h{2,3,4} & !AES) | (Mode   == 3'h{5,6,7}))
 *             9'h004                              Unsupported CkSum Src                  (CkSum_Src   == 2'h{2,3} & CKSUM)
 *             9'h008                              Forbidden CFB Mask                     (AES & CFBMode & UseNewKeysCFBMask & CFBMask[7] & (| CFBMask[6:0]))
 *             9'h010                              Unknown Ctrl Op                        ((| Ctrl[63:37]) | (| Ctrl[15:14]))
 *             9'h020                              UNUSED
 *             9'h040                              UNUSED
 *             9'h080                              Data Read Error
 *             9'h100                              Descriptor Ctrl Field Error            (D0.Ctrl != SOP || D1.Ctrl != EOP)
 *
 *             Data Error Code                     Data Error Condition
 *             ---------------                     --------------------
 *             9'h000                              No Error
 *             9'h001                              Insufficient Data To Cipher            (Packet_Length <= (Cipher_Offset or IV_Offset))
 *             9'h002                              Illegal IV Location                    ((Cipher_Offset <  IV_Offset) | (Cipher_Offset <= IV_Offset & AES & ~CTR))
 *             9'h004                              Illegal Wordcount To AES               (Packet_Length[3] != Cipher_Offset[0] & AES)
 *             9'h008                              Illegal Pad And ByteCount Spec         (Hash_Byte_Count != 0 & !Pad_Hash)
 *             9'h010                              Insufficient Data To CkSum             ({Packet_Length, 1'b0} <= CkSum_Offset)
 *             9'h020                              Unknown Data Op                        ((| dstLLWMask[63:60]) | (| dstLLWMask[57:40]) | (| authDst[63:40]) | (| ckSumDst[63:40]))
 *             9'h040                              Insufficient Data To Auth              ({Packet_Length} <= Auth_Offset)
 *             9'h080                              Data Read Error
 *             9'h100                              UNUSED
 */

/*
 * Result Control Word for Message Ring Descriptor
 */

/* #define MSG_RSLT_CTL_CTL */
#define MSG_RSLT_CTL_CTL_LSB      61
#define MSG_RSLT_CTL_CTL_BITS     THREE_BITS
#define MSG_RSLT_CTL_CTL_MASK \
(MSG_RSLT_CTL_CTL_BITS << MSG_RSLT_CTL_CTL_LSB)

	/* #define MSG_RSLT_CTL_DST_ID */
#define MSG_RSLT_CTL_DST_ID_LSB   54
#define MSG_RSLT_CTL_DST_ID_BITS  SEVEN_BITS
#define MSG_RSLT_CTL_DST_ID_MASK \
(MSG_RSLT_CTL_DST_ID_BITS << MSG_RSLT_CTL_DST_ID_LSB)

	/* #define MSG_RSLT_CTL_DSC_CTL */
#define MSG_RSLT_CTL_DSC_CTL_LSB  49
#define MSG_RSLT_CTL_DSC_CTL_BITS THREE_BITS
#define MSG_RSLT_CTL_DSC_CTL_MASK \
(MSG_RSLT_CTL_DSC_CTL_BITS << MSG_RSLT_CTL_DSC_CTL_LSB)

	/* #define MSG_RSLT_CTL_INST_ERR */
#define MSG_RSLT_CTL_INST_ERR_LSB      40
#define MSG_RSLT_CTL_INST_ERR_BITS     EIGHT_BITS
#define MSG_RSLT_CTL_INST_ERR_MASK \
(MSG_RSLT_CTL_INST_ERR_BITS << MSG_RSLT_CTL_INST_ERR_LSB)

	/* #define MSG_RSLT_CTL_DSC_ADDR */
#define MSG_RSLT_CTL_DSC_ADDR_LSB      0
#define MSG_RSLT_CTL_DSC_ADDR_BITS     FOURTY_BITS
#define MSG_RSLT_CTL_DSC_ADDR_MASK \
(MSG_RSLT_CTL_DSC_ADDR_BITS << MSG_RSLT_CTL_DSC_ADDR_LSB)

	/* #define MSG_RSLT_CTL_MASK */
#define MSG_RSLT_CTL_MASK \
	(MSG_RSLT_CTL_CTRL_MASK | MSG_RSLT_CTL_DST_ID_MASK | \
	 MSG_RSLT_CTL_DSC_CTL_MASK | MSG_RSLT_CTL_INST_ERR_MASK | \
	 MSG_RSLT_CTL_DSC_ADDR_MASK)

	/*
	 * Result Data Word for Message Ring Descriptor
	 */
	/* #define MSG_RSLT_DATA_CTL */
#define MSG_RSLT_DATA_CTL_LSB     61
#define MSG_RSLT_DATA_CTL_BITS    THREE_BITS
#define MSG_RSLT_DATA_CTL_MASK \
(MSG_RSLT_DATA_CTL_BITS << MSG_RSLT_DATA_CTL_LSB)

	/* #define MSG_RSLT_DATA_DST_ID */
#define MSG_RSLT_DATA_DST_ID_LSB  54
#define MSG_RSLT_DATA_DST_ID_BITS SEVEN_BITS
#define MSG_RSLT_DATA_DST_ID_MASK \
(MSG_RSLT_DATA_DST_ID_BITS << MSG_RSLT_DATA_DST_ID_LSB)

	/* #define MSG_RSLT_DATA_DSC_CTL */
#define MSG_RSLT_DATA_DSC_CTL_LSB       49
#define MSG_RSLT_DATA_DSC_CTL_BITS      THREE_BITS
#define MSG_RSLT_DATA_DSC_CTL_MASK \
(MSG_RSLT_DATA_DSC_CTL_BITS << MSG_RSLT_DATA_DSC_CTL_LSB)

	/* #define MSG_RSLT_DATA_INST_ERR */
#define MSG_RSLT_DATA_INST_ERR_LSB      40
#define MSG_RSLT_DATA_INST_ERR_BITS     EIGHT_BITS
#define MSG_RSLT_DATA_INST_ERR_MASK \
(MSG_RSLT_DATA_INST_ERR_BITS << MSG_RSLT_DATA_INST_ERR_LSB)

	/* #define MSG_RSLT_DATA_DSC_ADDR */
#define MSG_RSLT_DATA_DSC_ADDR_LSB      0
#define MSG_RSLT_DATA_DSC_ADDR_BITS     FOURTY_BITS
#define MSG_RSLT_DATA_DSC_ADDR_MASK     \
(MSG_RSLT_DATA_DSC_ADDR_BITS << MSG_RSLT_DATA_DSC_ADDR_LSB)

#define MSG_RSLT_DATA_MASK   \
	(MSG_RSLT_DATA_CTRL_MASK | MSG_RSLT_DATA_DST_ID_MASK | \
	 MSG_RSLT_DATA_DSC_CTL_MASK | MSG_RSLT_DATA_INST_ERR_MASK | \
	 MSG_RSLT_DATA_DSC_ADDR_MASK)


	/*
	 * Common Message Definitions
	 *
	 */

	/* #define MSG_CTL_OP_ADDR */
#define MSG_CTL_OP_ADDR_LSB      0
#define MSG_CTL_OP_ADDR_BITS     FOURTY_BITS
#define MSG_CTL_OP_ADDR_MASK     (MSG_CTL_OP_ADDR_BITS << MSG_CTL_OP_ADDR_LSB)

#define MSG_CTL_OP_TYPE
#define MSG_CTL_OP_TYPE_LSB             3
#define MSG_CTL_OP_TYPE_BITS            TWO_BITS
#define MSG_CTL_OP_TYPE_MASK            \
(MSG_CTL_OP_TYPE_BITS << MSG_CTL_OP_TYPE_LSB)

#define MSG0_CTL_OP_ENGINE_SYMKEY       0x01
#define MSG0_CTL_OP_ENGINE_PUBKEY       0x02

#define MSG1_CTL_OP_SYMKEY_PIPE0        0x00
#define MSG1_CTL_OP_SYMKEY_PIPE1        0x01
#define MSG1_CTL_OP_SYMKEY_PIPE2        0x02
#define MSG1_CTL_OP_SYMKEY_PIPE3        0x03

#define MSG1_CTL_OP_PUBKEY_PIPE0        0x00
#define MSG1_CTL_OP_PUBKEY_PIPE1        0x01
#define MSG1_CTL_OP_PUBKEY_PIPE2        0x02
#define MSG1_CTL_OP_PUBKEY_PIPE3        0x03


	/*       /----------------------------------------\
	 *       |                                        |
	 *       |   controldescriptor_s datastructure    |
	 *       |                                        |
	 *       \----------------------------------------/
	 *
#ifdef B0
	 *
	 *       controldescriptor_t.Instruction
	 *       -------------------------------
	 *
	 *   63    43         42            41              40         39        35 34    32 31  29     28      27    24    23   22  21     20      19    17    16     15     0
	 *  --------------------------------------------------------------------------------------------------------------------------------------------------------------------
	 * || UNUSED || Arc4Wait4Save | SaveArc4State | LoadArc4State | Arc4KeyLen | Cipher | Mode | InCp_Key || UNUSED || HMAC | Hash | InHs_Key || UNUSED || CkSum || UNUSED ||
	 *  --------------------------------------------------------------------------------------------------------------------------------------------------------------------
	 *      21            1              1               1               5          3       3        1         4        1      2         1          3        1        16
	 *            <-----------------------------------------CIPHER---------------------------------------->          <----------HASH---------->
	 *
	 *  X0  CIPHER.Arc4Wait4Save   =                   If op is Arc4 and it requires state saving, then
	 *                                                 setting this bit will cause the current op to
	 *                                                 delay subsequent op loading until saved state data
	 *                                                 becomes visible.
	 *  X0         SaveArc4State   =                   Save Arc4 state at the end of Arc4 operation
	 *  X0         LoadArc4State   =                   Load Arc4 state at the beginning of an Arc4 operation
	 *                                                 This overriden by the InCp_Key setting for Arc4
	 *             Arc4KeyLen      =                   Length in bytes of Arc4 key (0 is interpreted as 32)
	 *                                                 Ignored for other ciphers
	 *                                                 For ARC4, IFetch/IDecode will always read exactly 4
	 *                                                 consecutive dwords into its CipherKey{0,3} regardless
	 *                                                 of this quantity; it will however only use the specified
	 *                                                 number of bytes.        
	 *             Cipher          =        3'b000     Bypass
	 *                                      3'b001     DES
	 *                                      3'b010     3DES
	 *                                      3'b011     AES 128-bit key
	 *                                      3'b100     AES 192-bit key
	 *                                      3'b101     AES 256-bit key
	 *                                      3'b110     ARC4
	 *                                      Remainder  UNDEFINED
	 *             Mode            =        3'b000     ECB
	 *                                      3'b001     CBC
	 *                                      3'b010     CFB (AES only, otherwise undefined)
	 *                                      3'b011     OFB (AES only, otherwise undefined)
	 *                                      3'b100     CTR (AES only, otherwise undefined)
	 *                                      Remainder  UNDEFINED
	 *             InCp_Key        =        1'b0       Preserve old Cipher Keys
	 *                                      1'b1       Load new Cipher Keys from memory to local registers
	 *                                                 and recalculate the Arc4 Sbox if Arc4 Cipher chosen;
	 *                                                 This overrides LoadArc4State setting.
	 *        HASH.HMAC            =        1'b0       Hash without HMAC
	 *                                      1'b1       Hash with HMAC 
	 *             Hash            =        2'b00      Hash NOP
	 *                                      2'b01      MD5
	 *                                      2'b10      SHA-1
	 *                                      2'b11      SHA-256
	 *             InHs_Key        =        1'b0       Preserve old HMAC Keys
	 *                                      1'b1       Load new HMAC Keys from memory to local registers
	 *                                                 Setting this bit while Cipher=Arc4 and LoadArc4State=1
	 *                                                 causes the decoder to load the Arc4 state from the
	 *                                                 cacheline following the HMAC keys (Whether HASH.HMAC
	 *                                                 is set or not).
	 *    CHECKSUM.CkSum           =        1'b0       CkSum NOP
	 *                                      1'b1       INTERNET_CHECKSUM
	 *
	 *
#else * B0 *
	 *
	 *       controldescriptor_t.Instruction
	 *       -------------------------------
	 *
	 *   63        37   36   35   34    32 31  29 28         27 26           24
	 *  ------------------------------------------------------------------------
	* ||   UNUSED   || V || E/D | Cipher | Mode | Init_Cipher | Cipher_Offset ||   ... CONT ...
	*  ------------------------------------------------------------------------
	*        28        1     1      3       3          2              3        
	*               <CTRL><--------------------CIPHER----------------------->
	*
	*
	*     23   22  21      20     19         18     17        16    15   14 13           2 1         0
	*  -----------------------------------------------------------------------------------------------
	* || HMAC | Hash | Init_Hash | Hash_Offset | Hash_Src || CkSum |  N/U  | CkSum_Offset | CkSum_Src ||
	*  -----------------------------------------------------------------------------------------------
	*     1      2         1            2            1         1       2         12            2
	*   <-----------------------HASH--------------------->  <-----------CHECKSUM-------------------->
	*
	*
	*
	*      CTRL.V                 =        1'b0       Instruction invalid
	*                                      1'b1       Instruction valid
	*      CIPHER.E/D             =        1'b0       Decrypt
	*                                      1'b1       Encrypt
	*             Cipher          =        3'b000     Bypass
	*                                      3'b001     DES
	*                                      3'b010     3DES
	*                                      3'b011     AES 128-bit key
	*                                      3'b100     AES 192-bit key
	*                                      3'b101     AES 256-bit key
	*                                      Remainder  UNDEFINED
	*             Mode            =        3'b000     ECB
	*                                      3'b001     CBC
	*                                      3'b010     CFB (AES only, otherwise undefined)
	*                                      3'b011     OFB (AES only, otherwise undefined)
	*                                      3'b100     CTR (AES only, otherwise undefined)
	*                                      Remainder  UNDEFINED
	*             Init_Cipher     =        2'b00      Preserve old IV/(Keys,NonceCFBMask)
	*                                      2'b01      Load new IV use old Keys,NonceCFBMask
	*                                      2'b10      Load new Keys,NonceCFBMask use old IV (?)
	*                                      2'b11      Load new IV/(Keys,NonceCFBMask)
	*             Cipher_Offset   =                   Nb of words between the first data segment 
	*                                                 and word on which to start cipher operation
*                                                 (64 BIT WORDS !!!)
	*        HASH.HMAC            =        1'b0       Hash without HMAC
	*                                      1'b1       Hash with HMAC 
	*             Hash            =        2'b00      Hash NOP
	*                                      2'b01      MD5
	*                                      2'b10      SHA-1
	*                                      2'b11      SHA-256
	*             Init_Hash       =        1'b0       Preserve old key HMAC key stored in ID registers (moot if HASH.HMAC == 0)
	*                                      1'b1       Load new HMAC key from memory ctrl section to ID registers
	*             Hash_Offset     =                   Nb of words between the first data segment
	*                                                 and word on which to start hashing 
*                                                 (64 bit words)
	*             Hash_Src        =        1'b0       DMA channel
	*                                      1'b1       Cipher if word count exceeded Cipher_Offset; 
	*                                                 DMA channel otherwise
	*    CHECKSUM.CkSum           =        1'b0       CkSum NOP
	*                                      1'b1       INTERNET_CHECKSUM
	*             N/U             =        2'bx       Field not used
	*             CkSum_Offset    =                   Nb of words between the first data segment 
	*                                                 and word on which to start 
*                                                 checksum calculation (32 BIT WORDS !!!)
	*             CkSum_Src       =        2'b00      DMA channel if word count exceeded CkSum_Offset
	*                                      2'b01      Cipher if word count exceeded CkSum_Offset,
	*                                      2'b10      UNDEFINED
	*                                      2'b11      UNDEFINED
	*
	*
	*             OLD !!!
	*             CkSum_Src       =        2'b00      0
	*                                      2'b01      Cipher if word count exceeded CkSum_Offset,
	*                                                 0 otherwise
	*                                      2'b10      DMA channel if word count exceeded 
	*                                                 CkSum_Offset, 0 otherwise
	*                                      2'b11      UNDEFINED
	*
	*
	*       controldescriptor_t.cipherHashInfo.infoAES256ModeHMAC
	*       -----------------------------------------------------
	*  
	*  -----------------------------------------------------------------
	* ||63              AES Key0                                      0||
	*  -----------------------------------------------------------------
	*                   .
	*                   .
	*                   .
	*  -----------------------------------------------------------------
	* ||63              AES Key3                                      0||
	*  -----------------------------------------------------------------
	*                   .
	*                   .
	*                   .
	*  -----------------------------------------------------------------
	* ||63              HMAC Key0                                      0||
	*  -----------------------------------------------------------------
	*                   .
	*                   .
	*                   .
	*  -----------------------------------------------------------------
	* ||63              HMAC Key7                                      0||
	*  -----------------------------------------------------------------
	*
	*   63        40  39                   8  7                       0
	*  -----------------------------------------------------------------
	* ||   UNUSED   || Nonce (AES/CTR only) || CFB_Mask (AES/CFB only) ||
	*  -----------------------------------------------------------------
	*        24                 32                       8
#endif * B0 *
	*
	*/
#ifdef B0

	/* #define CTRL_DSC_ARC4_WAIT4SAVE */
#define CTL_DSC_ARC4_WAIT4SAVE_OFF       0
#define CTL_DSC_ARC4_WAIT4SAVE_ON        1
#define CTL_DSC_ARC4_WAIT4SAVE_LSB       42
#define CTL_DSC_ARC4_WAIT4SAVE_BITS      ONE_BIT
#define CTL_DSC_ARC4_WAIT4SAVE_MASK      (CTL_DSC_ARC4_WAIT4SAVE_BITS << CTL_DSC_ARC4_WAIT4SAVE_LSB)

	/* #define CTRL_DSC_ARC4_SAVESTATE */
#define CTL_DSC_ARC4_SAVESTATE_OFF       0
#define CTL_DSC_ARC4_SAVESTATE_ON        1
#define CTL_DSC_ARC4_SAVESTATE_LSB       41
#define CTL_DSC_ARC4_SAVESTATE_BITS      ONE_BIT
#define CTL_DSC_ARC4_SAVESTATE_MASK      (CTL_DSC_ARC4_SAVESTATE_BITS << CTL_DSC_ARC4_SAVESTATE_LSB)

	/* #define CTRL_DSC_ARC4_LOADSTATE */
#define CTL_DSC_ARC4_LOADSTATE_OFF       0
#define CTL_DSC_ARC4_LOADSTATE_ON        1
#define CTL_DSC_ARC4_LOADSTATE_LSB       40
#define CTL_DSC_ARC4_LOADSTATE_BITS      ONE_BIT
#define CTL_DSC_ARC4_LOADSTATE_MASK      (CTL_DSC_ARC4_LOADSTATE_BITS << CTL_DSC_ARC4_LOADSTATE_LSB)

	/* #define CTRL_DSC_ARC4_KEYLEN */
#define CTL_DSC_ARC4_KEYLEN_LSB          35
#define CTL_DSC_ARC4_KEYLEN_BITS         FIVE_BITS
#define CTL_DSC_ARC4_KEYLEN_MASK         (CTL_DSC_ARC4_KEYLEN_BITS << CTL_DSC_ARC4_KEYLEN_LSB)

	/* #define CTL_DSC_CPHR  (cipher) */
#define CTL_DSC_CPHR_BYPASS       0 /* undefined */
#define CTL_DSC_CPHR_DES          1
#define CTL_DSC_CPHR_3DES         2
#define CTL_DSC_CPHR_AES128       3
#define CTL_DSC_CPHR_AES192       4
#define CTL_DSC_CPHR_AES256       5
#define CTL_DSC_CPHR_ARC4         6
#define CTL_DSC_CPHR_UNDEF3       7 /* undefined */
#define CTL_DSC_CPHR_LSB          32
#define CTL_DSC_CPHR_BITS         THREE_BITS
#define CTL_DSC_CPHR_MASK         (CTL_DSC_CPHR_BITS << CTL_DSC_CPHR_LSB)

	/* #define CTL_DSC_MODE  */
#define CTL_DSC_MODE_ECB          0
#define CTL_DSC_MODE_CBC          1
#define CTL_DSC_MODE_CFB          2
#define CTL_DSC_MODE_OFB          3
#define CTL_DSC_MODE_CTR          4
#define CTL_DSC_MODE_LSB          29
#define CTL_DSC_MODE_BITS         THREE_BITS
#define CTL_DSC_MODE_MASK         (CTL_DSC_MODE_BITS << CTL_DSC_MODE_LSB)

	/* #define CTL_DSC_ICPHR */
#define CTL_DSC_ICPHR_OKY          0 /* Old Keys */
#define CTL_DSC_ICPHR_NKY          1 /* New Keys */
#define CTL_DSC_ICPHR_LSB          28
#define CTL_DSC_ICPHR_BITS         ONE_BIT
#define CTL_DSC_ICPHR_MASK         (CTL_DSC_ICPHR_BITS << CTL_DSC_ICPHR_LSB)

	/* #define CTL_DSC_HMAC */
#define CTL_DSC_HMAC_OFF          0
#define CTL_DSC_HMAC_ON           1
#define CTL_DSC_HMAC_LSB          23
#define CTL_DSC_HMAC_BITS         ONE_BIT
#define CTL_DSC_HMAC_MASK         (CTL_DSC_HMAC_BITS << CTL_DSC_HMAC_LSB)

	/* #define CTL_DSC_HASH */
#define CTL_DSC_HASH_NOP          0
#define CTL_DSC_HASH_MD5          1
#define CTL_DSC_HASH_SHA1         2
#define CTL_DSC_HASH_SHA256       3
#define CTL_DSC_HASH_LSB          21
#define CTL_DSC_HASH_BITS         TWO_BITS
#define CTL_DSC_HASH_MASK         (CTL_DSC_HASH_BITS << CTL_DSC_HASH_LSB)

	/* #define CTL_DSC_IHASH */
#define CTL_DSC_IHASH_OLD         0
#define CTL_DSC_IHASH_NEW         1
#define CTL_DSC_IHASH_LSB         20
#define CTL_DSC_IHASH_BITS        ONE_BIT
#define CTL_DSC_IHASH_MASK        (CTL_DSC_IHASH_BITS << CTL_DSC_IHASH_LSB)

	/* #define CTL_DSC_CKSUM */
#define CTL_DSC_CKSUM_NOP         0
#define CTL_DSC_CKSUM_IP          1
#define CTL_DSC_CKSUM_LSB         16
#define CTL_DSC_CKSUM_BITS        ONE_BIT
#define CTL_DSC_CKSUM_MASK        (CTL_DSC_CKSUM_BITS << CTL_DSC_CKSUM_LSB)

#else /* B0 */


	/* #define CTL_DSC_VALID (valid descriptor flag) */
#define CTL_DSC_CTL_INVALID  0
#define CTL_DSC_CTL_VALID    1
#define CTL_DSC_CTL_LSB      36
#define CTL_DSC_CTL_BITS     ONE_BIT
#define CTL_DSC_CTL_MASK     (CTL_DSC_CTL_BITS << CTL_DSC_CTL_LSB)

	/* #define CTL_DSC_SYM_OP (symmetric key operation) */
#define CTL_DSC_SYM_OP_DECRYPT    0
#define CTL_DSC_SYM_OP_ENCRYPT    1
#define CTL_DSC_SYM_OP_LSB        35
#define CTL_DSC_SYM_OP_BITS       ONE_BIT
#define CTL_DSC_SYM_OP_MASK       (CTL_DSC_SYM_OP_BITS << CTL_DSC_SYM_OP_LSB)


	/* #define CTL_DSC_CPHR  (cipher) */
#define CTL_DSC_CPHR_BYPASS       0 /* undefined */
#define CTL_DSC_CPHR_DES          1
#define CTL_DSC_CPHR_3DES         2
#define CTL_DSC_CPHR_AES128       3
#define CTL_DSC_CPHR_AES192       4
#define CTL_DSC_CPHR_AES256       5
#define CTL_DSC_CPHR_UNDEF2       6 /* undefined */
#define CTL_DSC_CPHR_UNDEF3       7 /* undefined */
#define CTL_DSC_CPHR_LSB          32
#define CTL_DSC_CPHR_BITS         THREE_BITS
#define CTL_DSC_CPHR_MASK         (CTL_DSC_CPHR_BITS << CTL_DSC_CPHR_LSB)

	/* #define CTL_DSC_MODE  */
#define CTL_DSC_MODE_ECB          0
#define CTL_DSC_MODE_CBC          1
#define CTL_DSC_MODE_CFB          2
#define CTL_DSC_MODE_OFB          3
#define CTL_DSC_MODE_CTR          4
#define CTL_DSC_MODE_LSB          29
#define CTL_DSC_MODE_BITS         THREE_BITS
#define CTL_DSC_MODE_MASK         (CTL_DSC_MODE_BITS << CTL_DSC_MODE_LSB)

	/* #define CTL_DSC_ICPHR */
#define CTL_DSC_ICPHR_OIV_OKY_ON   0 /* 2-bits,00=Old IV,Old Keys, Old Nonce */
#define CTL_DSC_ICPHR_NIV_OKY_ON   1 /* 2-bits,01=New IV,Old Keys, Old Nonce */
#define CTL_DSC_ICPHR_OIV_NKY_NN   2 /* 2-bits,10=Old IV,New Keys, New Nonce */
#define CTL_DSC_ICPHR_NIV_NKY_NN   3 /* 2-bits,11=New IV,New Keys, New Nonce */
#define CTL_DSC_ICPHR_LSB          27
#define CTL_DSC_ICPHR_BITS         TWO_BITS
#define CTL_DSC_ICPHR_MASK         (CTL_DSC_ICPHR_BITS << CTL_DSC_ICPHR_LSB)

	/* Cipher_Offset:
	 *   Nb of words between the first data segment and word on which to 
	 *   start cipher operation (64 BIT WORDS !!!)
	 */
	/* #define CTL_DSC_CPHROFF */
#define CTL_DSC_CPHROFF_LSB        24
#define CTL_DSC_CPHROFF_BITS       THREE_BITS
#define CTL_DSC_CPHROFF_MASK       (CTL_DSC_CPHROFF_BITS << CTL_DSC_CPHROFF_LSB)

	/* #define CTL_DSC_HMAC */
#define CTL_DSC_HMAC_OFF          0
#define CTL_DSC_HMAC_ON           1
#define CTL_DSC_HMAC_LSB          23
#define CTL_DSC_HMAC_BITS         ONE_BIT
#define CTL_DSC_HMAC_MASK         (CTL_DSC_HMAC_BITS << CTL_DSC_HMAC_LSB)

	/* #define CTL_DSC_HASH */
#define CTL_DSC_HASH_NOP          0
#define CTL_DSC_HASH_MD5          1
#define CTL_DSC_HASH_SHA1         2
#define CTL_DSC_HASH_SHA256       3
#define CTL_DSC_HASH_LSB          21
#define CTL_DSC_HASH_BITS         TWO_BITS
#define CTL_DSC_HASH_MASK         (CTL_DSC_HASH_BITS << CTL_DSC_HASH_LSB)

	/* #define CTL_DSC_IHASH */
#define CTL_DSC_IHASH_OLD         0
#define CTL_DSC_IHASH_NEW         1
#define CTL_DSC_IHASH_LSB         20 
#define CTL_DSC_IHASH_BITS        ONE_BIT
#define CTL_DSC_IHASH_MASK        (CTL_DSC_IHASH_BITS << CTL_DSC_IHASH_LSB)

	/* #define CTL_DSC_HASHOFF */
#define CTL_DSC_HASHOFF_LSB       18
#define CTL_DSC_HASHOFF_BITS      TWO_BITS
#define CTL_DSC_HASHOFF_MASK      (CTL_DSC_HASHOFF_BITS << CTL_DSC_HASHOFF_LSB)

	/* #define CTL_DSC_HASHSRC */
#define CTL_DSC_HASHSRC_DMA       0
#define CTL_DSC_HASHSRC_CIPHER    1
#define CTL_DSC_HASHSRC_LSB       17
#define CTL_DSC_HASHSRC_BITS      ONE_BIT
#define CTL_DSC_HASHSRC_MASK      (CTL_DSC_HASHSRC_BITS << CTL_DSC_HASHSRC_LSB)

	/* #define CTL_DSC_CKSUM */
#define CTL_DSC_CKSUM_NOP         0
#define CTL_DSC_CKSUM_IP          1
#define CTL_DSC_CKSUM_LSB         16
#define CTL_DSC_CKSUM_BITS        ONE_BIT
#define CTL_DSC_CKSUM_MASK        (CTL_DSC_CKSUM_BITS << CTL_DSC_CKSUM_LSB)

	/* #define CTL_DSC_CKSUMOFF */
#define CTL_DSC_CKSUMOFF_LSB      2
#define CTL_DSC_CKSUMOFF_BITS     TWELVE_BITS
#define CTL_DSC_CKSUMOFF_MASK   (CTL_DSC_CKSUMOFF_BITS << CTL_DSC_CKSUMOFF_LSB)

	/* #define CTL_DSC_CKSUMSRC */
#define CTL_DSC_CKSUMSRC_0        0   /* !!MARK!!  ??? */
#define CTL_DSC_CKSUMSRC_CIPHER   1
#define CTL_DSC_CKSUMSRC_DMA      2
#define CTL_DSC_CKSUMSRC_UNDEF    3
#define CTL_DSC_CKSUMSRC_LSB      0
#define CTL_DSC_CKSUMSRC_BITS     TWO_BITS
#define CTL_DSC_CKSUMSRC_MASK   (CTL_DSC_CKSUMSRC_BITS << CTL_DSC_CKSUMSRC_LSB)

#endif /* B0 */

#ifndef B0
	/* 
	 * **********************************************************************
	 *       controldescriptor_t.cipherHashInfo.infoAES256ModeHMAC
	 * **********************************************************************
	 */

	/* #define CTL_DSC_NONCE */
#define CTL_DSC_NONCE_LSB         8
#define CTL_DSC_NONCE_BITS        THIRTY_TWO_BITS
#define CTL_DSC_NONCE_MASK        (CTL_DSC_NONCE_BITS << CTL_DSC_NONCE_LSB)

	/* #define CTL_DSC_CFB_MASK */
#define CTL_DSC_CFB_MASK_LSB      0
#define CTL_DSC_CFB_MASK_BITS     EIGHT_BITS
#define CTL_DSC_CFB_MASK_MASK     \
(CTL_DSC_CFB_MASK_BITS << CTL_DSC_CFB_MASK_LSB)
#endif /* B0 */


	/*
	 * Component strcts and unions defining CipherHashInfo_u
	 */

	/* All AES256 possibilities */
	/* AES256, (CTR or CFB),    HMAC (MD5, SHA-1, SHA-256)      - 104 bytes */
	typedef struct AES256ModeHMAC_s {
		uint64_t             cipherKey0;
		uint64_t             cipherKey1;
		uint64_t             cipherKey2;
		uint64_t             cipherKey3;
		uint64_t             hmacKey0;
		uint64_t             hmacKey1;
		uint64_t             hmacKey2;
		uint64_t             hmacKey3;
		uint64_t             hmacKey4;
		uint64_t             hmacKey5;
		uint64_t             hmacKey6;
		uint64_t             hmacKey7;
#ifndef B0
		uint64_t             nonceCFBMask;
#endif /* B0 */
	} AES256ModeHMAC_t, *AES256ModeHMAC_pt;

/* AES256, (CTR or CFB),    Non-HMAC (MD5, SHA-1, SHA-256)  - 40  bytes */
typedef struct AES256Mode_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
#ifndef B0
	uint64_t             nonceCFBMask;
#endif /* B0 */
} AES256Mode_t, *AES256Mode_pt;

/* AES256, (ECB, CBC, OFB), HMAC (MD5, SHA-1, SHA-256)      - 96  bytes */
typedef struct AES256HMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} AES256HMAC_t, *AES256HMAC_pt;

/* AES256, (ECB, CBC, OFB), Non-HMAC (MD5, SHA-1, SHA-256)  - 32  bytes */
typedef struct AES256_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
} AES256_t, *AES256_pt;


/* All AES192 possibilities */

/* AES192, (CTR or CFB),    HMAC (MD5, SHA-1, SHA-192)      - 96  bytes */
typedef struct AES192ModeHMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
#ifndef B0
	uint64_t             nonceCFBMask;
#endif /* B0 */
} AES192ModeHMAC_t, *AES192ModeHMAC_pt;

/* AES192, (CTR or CFB),    Non-HMAC (MD5, SHA-1, SHA-192)  - 32  bytes */
typedef struct AES192Mode_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
#ifndef B0
	uint64_t             nonceCFBMask;
#endif /* B0 */
} AES192Mode_t, *AES192Mode_pt;

/* AES192, (ECB, CBC, OFB), HMAC (MD5, SHA-1, SHA-192)      - 88  bytes */
typedef struct AES192HMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} AES192HMAC_t, *AES192HMAC_pt;

/* AES192, (ECB, CBC, OFB), Non-HMAC (MD5, SHA-1, SHA-192)  - 24  bytes */
typedef struct AES192_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
} AES192_t, *AES192_pt;


/* All AES128 possibilities */

/* AES128, (CTR or CFB),    HMAC (MD5, SHA-1, SHA-128)      - 88  bytes */
typedef struct AES128ModeHMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
#ifndef B0
	uint64_t             nonceCFBMask;
#endif /* B0 */
} AES128ModeHMAC_t, *AES128ModeHMAC_pt;

/* AES128, (CTR or CFB),    Non-HMAC (MD5, SHA-1, SHA-128)  - 24  bytes */
typedef struct AES128Mode_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
#ifndef B0
	uint64_t             nonceCFBMask;
#endif /* B0 */
} AES128Mode_t, *AES128Mode_pt;

/* AES128, (ECB, CBC, OFB), HMAC (MD5, SHA-1, SHA-128)      - 80  bytes */
typedef struct AES128HMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} AES128HMAC_t, *AES128HMAC_pt;

/* AES128, (ECB, CBC, OFB), Non-HMAC (MD5, SHA-1, SHA-128)  - 16  bytes */
typedef struct AES128_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
} AES128_t, *AES128_pt;


/* All DES possibilities */

/* DES, (ECB, CBC), HMAC (MD5, SHA-1, SHA-128)              - 72  bytes */
typedef struct DESHMAC_s {
	uint64_t             cipherKey0;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} DESHMAC_t, *DESHMAC_pt;

/* DES, (ECB, CBC), Non-HMAC (MD5, SHA-1, SHA-128)          - 9   bytes */
typedef struct DES_s {
	uint64_t             cipherKey0;
} DES_t, *DES_pt;


/* All 3DES possibilities */

/* 3DES, (ECB, CBC), HMAC (MD5, SHA-1, SHA-128)             - 88  bytes */
typedef struct DES3HMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} DES3HMAC_t, *DES3HMAC_pt;

/* 3DES, (ECB, CBC), Non-HMAC (MD5, SHA-1, SHA-128)         - 24  bytes */
typedef struct DES3_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
} DES3_t, *DES3_pt;


/* HMAC only - no cipher */

/* HMAC (MD5, SHA-1, SHA-128)                               - 64  bytes */
typedef struct HMAC_s {
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} HMAC_t, *HMAC_pt;

#ifdef B0
/* All ARC4 possibilities */
/* ARC4, HMAC (MD5, SHA-1, SHA-256)      - 96 bytes */
typedef struct ARC4ModeHMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
} ARC4ModeHMAC_t, *ARC4ModeHMAC_pt;

/* ARC4, HMAC (MD5, SHA-1, SHA-256)      - 408 bytes (not including 8 bytes from instruction) */
typedef struct ARC4StateModeHMAC_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
	uint64_t             hmacKey0;
	uint64_t             hmacKey1;
	uint64_t             hmacKey2;
	uint64_t             hmacKey3;
	uint64_t             hmacKey4;
	uint64_t             hmacKey5;
	uint64_t             hmacKey6;
	uint64_t             hmacKey7;
	uint64_t             PAD0;
	uint64_t             PAD1;
	uint64_t             PAD2;
	uint64_t             Arc4SboxData0;
	uint64_t             Arc4SboxData1;
	uint64_t             Arc4SboxData2;
	uint64_t             Arc4SboxData3;
	uint64_t             Arc4SboxData4;
	uint64_t             Arc4SboxData5;
	uint64_t             Arc4SboxData6;
	uint64_t             Arc4SboxData7;
	uint64_t             Arc4SboxData8;
	uint64_t             Arc4SboxData9;
	uint64_t             Arc4SboxData10;
	uint64_t             Arc4SboxData11;
	uint64_t             Arc4SboxData12;
	uint64_t             Arc4SboxData13;
	uint64_t             Arc4SboxData14;
	uint64_t             Arc4SboxData15;
	uint64_t             Arc4SboxData16;
	uint64_t             Arc4SboxData17;
	uint64_t             Arc4SboxData18;
	uint64_t             Arc4SboxData19;
	uint64_t             Arc4SboxData20;
	uint64_t             Arc4SboxData21;
	uint64_t             Arc4SboxData22;
	uint64_t             Arc4SboxData23;
	uint64_t             Arc4SboxData24;
	uint64_t             Arc4SboxData25;
	uint64_t             Arc4SboxData26;
	uint64_t             Arc4SboxData27;
	uint64_t             Arc4SboxData28;
	uint64_t             Arc4SboxData29;
	uint64_t             Arc4SboxData30;
	uint64_t             Arc4SboxData31;
	uint64_t             Arc4IJData;
	uint64_t             PAD3;
	uint64_t             PAD4;
	uint64_t             PAD5;
} ARC4StateModeHMAC_t, *ARC4StateModeHMAC_pt;

/* ARC4, Non-HMAC (MD5, SHA-1, SHA-256)  - 32  bytes */
typedef struct ARC4Mode_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
} ARC4Mode_t, *ARC4Mode_pt;

/* ARC4, Non-HMAC (MD5, SHA-1, SHA-256)  - 344  bytes (not including 8 bytes from instruction) */
typedef struct ARC4StateMode_s {
	uint64_t             cipherKey0;
	uint64_t             cipherKey1;
	uint64_t             cipherKey2;
	uint64_t             cipherKey3;
	uint64_t             PAD0;
	uint64_t             PAD1;
	uint64_t             PAD2;
	uint64_t             Arc4SboxData0;
	uint64_t             Arc4SboxData1;
	uint64_t             Arc4SboxData2;
	uint64_t             Arc4SboxData3;
	uint64_t             Arc4SboxData4;
	uint64_t             Arc4SboxData5;
	uint64_t             Arc4SboxData6;
	uint64_t             Arc4SboxData7;
	uint64_t             Arc4SboxData8;
	uint64_t             Arc4SboxData9;
	uint64_t             Arc4SboxData10;
	uint64_t             Arc4SboxData11;
	uint64_t             Arc4SboxData12;
	uint64_t             Arc4SboxData13;
	uint64_t             Arc4SboxData14;
	uint64_t             Arc4SboxData15;
	uint64_t             Arc4SboxData16;
	uint64_t             Arc4SboxData17;
	uint64_t             Arc4SboxData18;
	uint64_t             Arc4SboxData19;
	uint64_t             Arc4SboxData20;
	uint64_t             Arc4SboxData21;
	uint64_t             Arc4SboxData22;
	uint64_t             Arc4SboxData23;
	uint64_t             Arc4SboxData24;
	uint64_t             Arc4SboxData25;
	uint64_t             Arc4SboxData26;
	uint64_t             Arc4SboxData27;
	uint64_t             Arc4SboxData28;
	uint64_t             Arc4SboxData29;
	uint64_t             Arc4SboxData30;
	uint64_t             Arc4SboxData31;
	uint64_t             Arc4IJData;
	uint64_t             PAD3;
	uint64_t             PAD4;
	uint64_t             PAD5;
} ARC4StateMode_t, *ARC4StateMode_pt;
#endif /* B0 */

typedef union CipherHashInfo_u {
	AES256ModeHMAC_t     infoAES256ModeHMAC;
	AES256Mode_t         infoAES256Mode;
	AES256HMAC_t         infoAES256HMAC;
	AES256_t             infoAES256;
	AES192ModeHMAC_t     infoAES192ModeHMAC;
	AES192Mode_t         infoAES192Mode;
	AES192HMAC_t         infoAES192HMAC;
	AES192_t             infoAES192;
	AES128ModeHMAC_t     infoAES128ModeHMAC;
	AES128Mode_t         infoAES128Mode;
	AES128HMAC_t         infoAES128HMAC;
	AES128_t             infoAES128;
	DESHMAC_t            infoDESHMAC;
	DES_t                infoDES;
	DES3HMAC_t           info3DESHMAC;
	DES3_t               info3DES;
	HMAC_t               infoHMAC;
#ifdef B0
	ARC4ModeHMAC_t       infoARC4ModeHMAC;
	ARC4StateModeHMAC_t  infoARC4StateModeHMAC;
	ARC4Mode_t           infoARC4Mode;
	ARC4StateMode_t      infoARC4StateMode;
	// this makes this structure 408 bytes long, so when we add the 8-byte
	// instruction (see just below), the result is 416 (13 cache lines)
	// and we add 3 double words to make it an integer number of cache lines
	uint64_t             infoDwords[51];
#else /* B0 */
	// this makes this structure 120 bytes long, so when we add the 8-byte
	// instruction (see just below), the result is 128 (4 cache lines)
	uint64_t             infoDwords[15];
#endif /* B0 */
} CipherHashInfo_t, *CipherHashInfo_pt;


/* 
 * 
 *    controldescriptor_s datastructure 
 * 
 */

typedef struct controldescriptor_s {
	uint64_t            instruction;
	CipherHashInfo_t    cipherHashInfo;
} controldescriptor_t, *controldescriptor_pt;




/* **********************************************************************
 *       packetdescriptor_t
 * **********************************************************************
 */

/*       /--------------------------------------------\
 *       |                                            |
 *       |    New PacketDescriptor_s datastructure    |
 *       |                                            |
 *       \--------------------------------------------/
 *
 *
#ifdef B0
 *
 *       packetdescriptor_t.srcLengthIVOffUseIVNext
 *       ------------------------------------------
 *
 *           63           62      61             59    58        57    56       54  53           43 
 *  ------------------------------------------------------------------------------------------------
 * || Load HMAC key || Pad Hash || Hash Byte Count || Next || Use IV || IV Offset || Packet length ||   ... CONT ...
 *  ------------------------------------------------------------------------------------------------
 *           1            1           3                1        1          3              11
 *
 *
 *      42        41      40    39                  5  4      3  2                      0
 *  --------------------------------------------------------------------------------------
 * || UNUSED || Break || Wait || Segment src address || UNUSED || Global src data offset ||
 *  --------------------------------------------------------------------------------------
 *      1         1       1             35                2                 3
 *  
 *
 *
 *             Load HMAC key           =        1'b0       Preserve old HMAC key stored in Auth engine (moot if HASH.HMAC == 0)
 *                                              1'b1       Load HMAC key from ID registers at beginning of op
 *             Pad Hash                =        1'b0       HASH will assume the data was padded to be a multiple
 *                                                         of 512 bits in length and that the last 64 bit word
 *                                                         expresses the total datalength in bits seen by HASH engine
 *                                              1'b1       The data was not padded to be a multiple of 512 bits in length;
 *                                                         The Hash engine will do its own padding to generate the correct digest.
 *             Hash Byte Count                             Number of BYTES on last 64-bit data word to use in digest calculation RELEVANT ONLY IF Pad Hash IS SET
 *                                              3'b000     Use all 8
 *                                              3'b001     Use first (MS) byte only (0-out rest), i.e., 0xddXXXXXXXXXXXXXX
 *                                              3'b010     Use first 2 bytes only (0-out rest), i.e., 0xddddXXXXXXXXXXXX     ... etc
 *             Next                    =        1'b0       Finish (return msg descriptor) at end of operation
 *                                              1'b1       Grab the next PacketDescriptor (i.e. next cache-line) when the current is complete.
 *                                                         This allows for fragmentation/defragmentation and processing of large (>16kB) packets.
 *                                                         The sequence of adjacent PacketDescriptor acts as a contiguous linked list of
 *                                                         pointers to the actual packets with Next==0 on the last PacketDescriptor to end processing.
 *             Use IV                  =        1'b0       On first frag:           Use old IV
 *                                                         On subsequent frags:     Do not write out to DST the (dword) offset data
 *                                              1'b1       On first frag:           Use data @ Segment_address + IV_Offset as IV
 *                                                         On subsequent frags:     Do write out to DST the (dword) offset data
 *             IV Offset               =                   On first frag:           Offset IN NB OF 8 BYTE WORDS (dwords) from beginning of packet
 *                                                                                  (i.e. (Potentially byte-shifted) Segment address) to cipher IV
 *                                                         On subsequent frags:     Offset to beginning of data to process; data to offset won't 
 *                                                                                  be given to engines and will be written out to dst in the clear.
 *                                                                                  ON SUBSEQUENT FRAGS, IV_Offset MAY NOT EXCEED 3; LARGER VALUES WILL CAUSE AN ERROR
 *                                                                                  SEE ERROR CONDITIONS BELOW
 *             Packet length           =                   Nb double words to stream in (Including Segment address->CP/IV/Auth/CkSum offsets)
 *                                                         This is the total amount of data (x8 in bytes) read    (+1 dword if "Global src data offset" != 0)
 *                                                         This is the total amount of data (x8 in bytes) written (+1 dword if "Global dst data offset" != 0, if Dst dword offset == 0)
 *                                                         If Packet length == 11'h7ff and (Global src data offset != 0 or Global dst data offset != 0)
 *                                                         the operation is aborted (no mem writes occur)
 *             Break                   =                   Break a wait (see below) state - causes the operation to be flushed and free descriptor to be returned.
 *                                                         Activated if DFetch blocked by Wait and Wait still active.
 *                                                         AS OF 02/10/2005 THIS FEATURE IS EXPERIMENTAL
 *             Wait                    =                   Setting that bit causes the operation to block in DFetch stage.
 *                                                         DFetch will keep polling the memory location until the bit is reset at which time
 *                                                         the pipe resumes normal operation. This feature is convenient for software dealing with fragmented packets.
 *                                                         AS OF 02/10/2005 THIS FEATURE IS EXPERIMENTAL
 *             Segment src address     =                   35 MSB of pointer to src data (i.e., cache-line aligned)
 *             Global src data offset  =                   Nb BYTES to right-shift data by before presenting it to engines
 *                                                         (0-7); allows realignment of byte-aligned, non-double-word aligned data
 *
 *       packetdescriptor_t.dstDataSettings
 *       ----------------------------------
 *
*       63    62           60  59   58           56  55         54     53      52          41     40
*  -------------------------------------------------------------------------------------------------------
* || UNUSED || Arc4ByteCount | E/D | Cipher_Offset || Hash_Offset | Hash_Src || CkSum_Offset | CkSum_Src ||   ... CONT ...
*  -------------------------------------------------------------------------------------------------------
*       1            3          1          3               2           1             12            1
*            <---------------CIPHER----------------><---------HASH-----------><-------CHECKSUM----------->
*
*   39                 5  4                3  2                      0
*  --------------------------------------------------------------------
* || Cipher dst address || Dst dword offset || Global dst data offset ||
*  --------------------------------------------------------------------
*            35                    2                     3
*
*
*  X0         Arc4ByteCount           =                   Number of BYTES on last 64-bit data word to encrypt
*                                              3'b000     Encrypt all 8
*                                              3'b001     Encrypt first (MS) byte only i.e., 0xddXXXXXXXXXXXXXX
*                                              3'b010     Encrypt first 2 bytes only i.e., 0xddddXXXXXXXXXXXX     ... etc
*                                                         In reality, all are encrypted, however, the SBOX
*                                                         is not written past the last byte to encrypt
*             E/D                     =        1'b0       Decrypt
*                                              1'b1       Encrypt
*             Cipher_Offset           =                   Nb of words between the first data segment 
	*                                                         and word on which to start cipher operation
*                                                         (64 BIT WORDS !!!)
	*             Hash_Offset             =                   Nb of words between the first data segment
	*                                                         and word on which to start hashing 
*                                                         (64 bit words)
	*             Hash_Src                =        1'b0       DMA channel
	*                                              1'b1       Cipher if word count exceeded Cipher_Offset; 
	*                                                         DMA channel otherwise
	*             CkSum_Offset            =                   Nb of words between the first data segment 
	*                                                         and word on which to start 
*                                                         checksum calculation (32 BIT WORDS !!!)
	*             CkSum_Src               =        1'b0       DMA channel
	*                                              1'b1       Cipher if word count exceeded Cipher_Offset
	*                                                         DMA channel otherwise
*             Cipher dst address      =                   35 MSB of pointer to dst location (i.e., cache-line aligned)
	*             Dst dword offset        =                   Nb of double-words to left-shift data from spec'ed Cipher dst address before writing it to memory
	*             Global dst data offset  =                   Nb BYTES to left-shift (double-word boundary aligned) data by before writing it to memory
	*
	*
	*       packetdescriptor_t.authDstNonceLow
	*       ----------------------------------
	*
	*   63       40  39               5  4                0   
	*  -----------------------------------------------------
	* || Nonce_Low || Auth_dst_address || Cipher_Offset_Hi ||
	*  -----------------------------------------------------
	*        24             35                    5
	*
	*
	*
	*             Nonce_Low         =                 Nonce[23:0] 24 least significant bits of 32-bit long nonce
	*                                                 Used by AES in counter mode
*             Auth_dst_address  =                 35 MSB of pointer to authentication dst location (i.e., cache-line aligned)
	* X0          Cipher_Offset_Hi  =                 On first frag:           5 MSB of 8-bit Cipher_offset; will be concatenated to
	*                                                                          the top of packetdescriptor_t.dstDataSettings.Cipher_Offset
	*                                                 On subsequent frags:     Ignored
	*
	*
	*       packetdescriptor_t.ckSumDstNonceHiCFBMaskLLWMask
	*       ------------------------------------------------
	*
	*   63    58  57     56  55      48  47      40  39                5  4            0
	*  ----------------------------------------------------------------------------------
	* || UNUSED || LLWMask || CFB_Mask || Nonce_Hi || CkSum_dst_address || IV_Offset_Hi ||
	*  ----------------------------------------------------------------------------------
	*      6          2          8           8                35                 5
	*
	*
	*   LLWMask, aka, Last_long_word_mask =   2'b00      Give last 128 bit word from AES engine to auth/cksum/wrbbufer as is - applicable in AES CTR only
	*                                         2'b11      Mask (zero-out) 32 least significant bits
	*                                         2'b10      Mask 64 LSBs
	*                                         2'b01      Mask 96 LSBs
	*                 CFB_Mask            =              8 bit mask used by AES in CFB mode
	*                 Nonce_Hi            =              Nonce[31:24] 8 most significant bits of 32-bit long nonce
	*                                                    Used by AES in counter mode
*                 CkSum_dst_address   =              35 MSB of pointer to cksum dst location (i.e., cache-line aligned)
	*  X0             IV_Offset_Hi        =              On first frag:           5 MSB of 8-bit IV offset; will be concatenated to
	*                                                                             the top of packetdescriptor_t.srcLengthIVOffUseIVNext.IV_Offset
	*                                                    On subsequent frags:     Ignored
#else * B0 *
	*       packetdescriptor_t.srcLengthIVOffUseIVNext
	*       ------------------------------------------
	*
	*           63           62      61             59    58        57    56       54  53           43  42    40  39                  5  4      3  2                      0
	*  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
	* || Load HMAC key || Pad Hash || Hash Byte Count || Next || Use IV || IV Offset || Packet length || UNUSED || Segment src address || UNUSED || Global src data offset || 
	*  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
	*           1            1           3                1        1          3              11            3             35                 2                 3
	*
	*
	*             Load HMAC key           =        1'b0       Preserve old HMAC key stored in Auth engine (moot if HASH.HMAC == 0)
	*                                              1'b1       Load HMAC key from ID registers at beginning of op
	*             Pad Hash                =        1'b0       HASH will assume the data was padded to be a multiple
	*                                                         of 512 bits in length and that the last 64 bit word
	*                                                         expresses the total datalength in bits seen by HASH engine
	*                                              1'b1       The data was not padded to be a multiple of 512 bits in length;
	*                                                         The Hash engine will do its own padding to generate the correct digest.
	*             Hash Byte Count                             Number of BYTES on last 64-bit data word to use in digest calculation RELEVANT ONLY IF Pad Hash IS SET
	*                                              3'b000     Use all 8
	*                                              3'b001     Use first (MS) byte only (0-out rest), i.e., 0xddXXXXXXXXXXXXXX
	*                                              3'b010     Use first 2 bytes only (0-out rest), i.e., 0xddddXXXXXXXXXXXX     ... etc
	*             Next                    =        1'b0       Finish (return msg descriptor) at end of operation
	*                                              1'b1       Grab the next PacketDescriptor (i.e. next cache-line) - NOT YET IMPLEMENTED !!!
	*             Use IV                  =        1'b0       Use old IV
	*                                              1'b1       Use data @ Segment_address + IV_Offset as IV
	*             IV Offset               =                   Offset IN NB OF 8 BYTE WORDS from beginning of packet
	*                                                         (i.e. (Potentially shifted) Segment address) to cipher IV
*             Packet length           =                   Nb double words to stream in (Including Segment address->CP/IV/Auth/CkSum offsets)
	*                                                         This is the total amount of data (x8 in bytes) read    (+1 dword if "Global src data offset" != 0)
	*                                                         This is the total amount of data (x8 in bytes) written (+1 dword if "Global dst data offset" != 0)
	*                                                         (0-7); allows realignment of byte-aligned, non-double-word aligned data
	*                                                         If Packet length == 11'h7ff and (Global src data offset != 0 or Global dst data offset != 0)
	*                                                         the operation is aborted (no mem writes occur)
*             Segment src address     =                   35 MSB of pointer to src data (i.e., cache-line aligned)
	*             Global src data offset  =                   Nb BYTES to right-shift data by before presenting it to engines
	*                                                         (0-7); allows realignment of byte-aligned, non-double-word aligned data
	*
	*       packetdescriptor_t.dstLLWMask
	*       -----------------------------
	*
	*   63    60  59                 58  57    40  39                 5  4      3  2                      0
	*  -----------------------------------------------------------------------------------------------------
	* || UNUSED || Last long word mask || UNUSED || Cipher dst address || UNUSED || Global dst data offset ||
	*  -----------------------------------------------------------------------------------------------------
	*      4                2               18              35              2                 3
	*
	*
	*             Last long word mask     =   2'b00      Give last 128 bit word to AES/HAMC/CKSUM engines as is
	*                                         2'b11      Mask (zero-out) 32 least significant bits
	*                                         2'b10      Mask 64 LSBs
	*                                         2'b01      Mask 96 LSBs
*             Cipher dst address      =              35 MSB of pointer to dst location (i.e., cache-line aligned)
	*             Global dst data offset  =              Nb BYTES to left-shift (double-word boundary aligned) data by before writing it to memory
#endif * B0 *
	*/

	/* #define PKT_DSC_LOADHMACKEY */
#define PKT_DSC_LOADHMACKEY_OLD   0
#define PKT_DSC_LOADHMACKEY_LOAD  1
#define PKT_DSC_LOADHMACKEY_LSB   63
#define PKT_DSC_LOADHMACKEY_BITS  ONE_BIT
#define PKT_DSC_LOADHMACKEY_MASK  \
(PKT_DSC_LOADHMACKEY_BITS << PKT_DSC_LOADHMACKEY_LSB)

	/* #define PKT_DSC_PADHASH */
#define PKT_DSC_PADHASH_PADDED    0
#define PKT_DSC_PADHASH_PAD       1    /* requires padding */
#define PKT_DSC_PADHASH_LSB       62
#define PKT_DSC_PADHASH_BITS      ONE_BIT
#define PKT_DSC_PADHASH_MASK      (PKT_DSC_PADHASH_BITS << PKT_DSC_PADHASH_LSB)

	/* #define PKT_DSC_HASHBYTES */
#define PKT_DSC_HASHBYTES_ALL8    0
#define PKT_DSC_HASHBYTES_MSB     1
#define PKT_DSC_HASHBYTES_MSW     2
#define PKT_DSC_HASHBYTES_LSB     59
#define PKT_DSC_HASHBYTES_BITS    THREE_BITS
#define PKT_DSC_HASHBYTES_MASK    \
(PKT_DSC_HASHBYTES_BITS << PKT_DSC_HASHBYTES_LSB)

	/* #define PKT_DSC_NEXT */
#define PKT_DSC_NEXT_FINISH       0
#define PKT_DSC_NEXT_DO           1
#define PKT_DSC_NEXT_LSB          58
#define PKT_DSC_NEXT_BITS         ONE_BIT
#define PKT_DSC_NEXT_MASK         (PKT_DSC_NEXT_BITS << PKT_DSC_NEXT_LSB)

	/* #define PKT_DSC_IV */
#define PKT_DSC_IV_OLD            0
#define PKT_DSC_IV_NEW            1
#define PKT_DSC_IV_LSB            57
#define PKT_DSC_IV_BITS           ONE_BIT
#define PKT_DSC_IV_MASK           (PKT_DSC_IV_BITS << PKT_DSC_IV_LSB)

	/* #define PKT_DSC_IVOFF */
#define PKT_DSC_IVOFF_LSB         54
#define PKT_DSC_IVOFF_BITS        THREE_BITS
#define PKT_DSC_IVOFF_MASK        (PKT_DSC_IVOFF_BITS << PKT_DSC_IVOFF_LSB)

	/* #define PKT_DSC_PKTLEN */
#define PKT_DSC_PKTLEN_LSB         43
#define PKT_DSC_PKTLEN_BITS        ELEVEN_BITS
#define PKT_DSC_PKTLEN_MASK        (PKT_DSC_PKTLEN_BITS << PKT_DSC_PKTLEN_LSB)

#ifdef B0
	/* #define PKT_DSC_BREAK */
#define PKT_DSC_BREAK_OLD          0
#define PKT_DSC_BREAK_NEW          1
#define PKT_DSC_BREAK_LSB          41
#define PKT_DSC_BREAK_BITS         ONE_BIT
#define PKT_DSC_BREAK_MASK         (PKT_DSC_BREAK_BITS << PKT_DSC_BREAK_LSB)

	/* #define PKT_DSC_WAIT */
#define PKT_DSC_WAIT_OLD           0
#define PKT_DSC_WAIT_NEW           1
#define PKT_DSC_WAIT_LSB           41
#define PKT_DSC_WAIT_BITS          ONE_BIT
#define PKT_DSC_WAIT_MASK          (PKT_DSC_WAIT_BITS << PKT_DSC_WAIT_LSB)
#endif /* B0 */

	/* #define PKT_DSC_SEGADDR */
#define PKT_DSC_SEGADDR_LSB        0 
#define PKT_DSC_SEGADDR_BITS       FOURTY_BITS
#define PKT_DSC_SEGADDR_MASK       \
(PKT_DSC_SEGADDR_BITS << PKT_DSC_SEGADDR_LSB)

#define PKT_DSC_SEGOFFSET_LSB        0
#define PKT_DSC_SEGOFFSET_BITS       THREE_BITS
#define PKT_DSC_SEGOFFSET_MASK       \
(PKT_DSC_SEGOFFSET_BITS << PKT_DSC_SEGOFFSET_LSB)

#ifdef B0
	/* **********************************************************************
	 *       packetdescriptor_t.dstDataSettings
	 * **********************************************************************
	 */
	/* #define PKT_DSC_ARC4BYTECOUNT */
#define PKT_DSC_ARC4BYTECOUNT_ALL8    0
#define PKT_DSC_ARC4BYTECOUNT_MSB     1
#define PKT_DSC_ARC4BYTECOUNT_MSW     2
#define PKT_DSC_ARC4BYTECOUNT_LSB     60
#define PKT_DSC_ARC4BYTECOUNT_BITS    THREE_BITS
#define PKT_DSC_ARC4BYTECOUNT_MASK    (PKT_DSC_ARC4BYTECOUNT_BITS << PKT_DSC_ARC4BYTECOUNT_LSB)

	/* #define PKT_DSC_SYM_OP (symmetric key operation) */
#define PKT_DSC_SYM_OP_DECRYPT    0
#define PKT_DSC_SYM_OP_ENCRYPT    1
#define PKT_DSC_SYM_OP_LSB        59
#define PKT_DSC_SYM_OP_BITS       ONE_BIT
#define PKT_DSC_SYM_OP_MASK       (PKT_DSC_SYM_OP_BITS << PKT_DSC_SYM_OP_LSB)

	/* #define PKT_DSC_CPHROFF */
#define PKT_DSC_CPHROFF_LSB        56
#define PKT_DSC_CPHROFF_BITS       THREE_BITS
#define PKT_DSC_CPHROFF_MASK       (PKT_DSC_CPHROFF_BITS << PKT_DSC_CPHROFF_LSB)

	/* #define PKT_DSC_HASHOFF */
#define PKT_DSC_HASHOFF_LSB       54
#define PKT_DSC_HASHOFF_BITS      TWO_BITS
#define PKT_DSC_HASHOFF_MASK      (PKT_DSC_HASHOFF_BITS << PKT_DSC_HASHOFF_LSB)

	/* #define PKT_DSC_HASHSRC */
#define PKT_DSC_HASHSRC_DMA       0
#define PKT_DSC_HASHSRC_CIPHER    1
#define PKT_DSC_HASHSRC_LSB       53
#define PKT_DSC_HASHSRC_BITS      ONE_BIT
#define PKT_DSC_HASHSRC_MASK      (PKT_DSC_HASHSRC_BITS << PKT_DSC_HASHSRC_LSB)

	/* #define PKT_DSC_CKSUMOFF */
#define PKT_DSC_CKSUMOFF_LSB      41
#define PKT_DSC_CKSUMOFF_BITS     TWELVE_BITS
#define PKT_DSC_CKSUMOFF_MASK   (PKT_DSC_CKSUMOFF_BITS << PKT_DSC_CKSUMOFF_LSB)

	/* #define PKT_DSC_CKSUMSRC */
#define PKT_DSC_CKSUMSRC_DMA      0
#define PKT_DSC_CKSUMSRC_CIPHER   1
#define PKT_DSC_CKSUMSRC_LSB      40
#define PKT_DSC_CKSUMSRC_BITS     ONE_BIT
#define PKT_DSC_CKSUMSRC_MASK   (PKT_DSC_CKSUMSRC_BITS << PKT_DSC_CKSUMSRC_LSB)

	/* #define PKT_DSC_CPHR_DST_ADDR */
#define PKT_DSC_CPHR_DST_ADDR_LSB  0
#define PKT_DSC_CPHR_DST_ADDR_BITS FOURTY_BITS
#define PKT_DSC_CPHR_DST_ADDR_MASK \
(PKT_DSC_CPHR_DST_ADDR_BITS << PKT_DSC_CPHR_DST_ADDR_LSB)

	/* #define PKT_DSC_CPHR_DST_DWOFFSET */
#define PKT_DSC_CPHR_DST_DWOFFSET_LSB   3
#define PKT_DSC_CPHR_DST_DWOFFSET_BITS  TWO_BITS
#define PKT_DSC_CPHR_DST_DWOFFSET_MASK \
(PKT_DSC_CPHR_DST_DWOFFSET_BITS << PKT_DSC_CPHR_DST_DWOFFSET_LSB)

	/* #define PKT_DSC_CPHR_DST_OFFSET */
#define PKT_DSC_CPHR_DST_OFFSET_LSB   0
#define PKT_DSC_CPHR_DST_OFFSET_BITS  THREE_BITS
#define PKT_DSC_CPHR_DST_OFFSET_MASK \
(PKT_DSC_CPHR_DST_OFFSET_BITS << PKT_DSC_CPHR_DST_OFFSET_LSB)

	/* **********************************************************************
	 *       packetdescriptor_t.authDstNonceLow
	 * **********************************************************************
	 */
	/* #define PKT_DSC_NONCE_LOW */
#define PKT_DSC_NONCE_LOW_LSB  40
#define PKT_DSC_NONCE_LOW_BITS TWENTYFOUR_BITS
#define PKT_DSC_NONCE_LOW_MASK \
(PKT_DSC_NONCE_LOW_BITS << PKT_DSC_NONCE_LOW_LSB)

	/* #define PKT_DSC_AUTH_DST_ADDR */
#define PKT_DSC_AUTH_DST_ADDR_LSB  0
#define PKT_DSC_AUTH_DST_ADDR_BITS FOURTY_BITS
#define PKT_DSC_AUTH_DST_ADDR_MASK \
(PKT_DSC_AUTH_DST_ADDR_BITS << PKT_DSC_AUTH_DST_ADDR_LSB)

	/* #define PKT_DSC_CIPH_OFF_HI */
#define PKT_DSC_CIPH_OFF_HI_LSB      0
#define PKT_DSC_CIPH_OFF_HI_BITS     FIVE_BITS
#define PKT_DSC_CIPH_OFF_HI_MASK   (PKT_DSC_CIPH_OFF_HI_BITS << PKT_DSC_CIPH_OFF_HI_LSB)

	/* **********************************************************************
	 *       packetdescriptor_t.ckSumDstNonceHiCFBMaskLLWMask
	 * **********************************************************************
	 */
	/* #define PKT_DSC_LASTWORD */
#define PKT_DSC_LASTWORD_128       0
#define PKT_DSC_LASTWORD_96MASK    1
#define PKT_DSC_LASTWORD_64MASK    2
#define PKT_DSC_LASTWORD_32MASK    3
#define PKT_DSC_LASTWORD_LSB       56
#define PKT_DSC_LASTWORD_BITS      TWO_BITS
#define PKT_DSC_LASTWORD_MASK      (PKT_DSC_LASTWORD_BITS << PKT_DSC_LASTWORD_LSB)

	/* #define PKT_DSC_CFB_MASK */
#define PKT_DSC_CFB_MASK_LSB      48
#define PKT_DSC_CFB_MASK_BITS     EIGHT_BITS
#define PKT_DSC_CFB_MASK_MASK     (PKT_DSC_CFB_MASK_BITS << PKT_DSC_CFB_MASK_LSB)

	/* #define PKT_DSC_NONCE_HI */
#define PKT_DSC_NONCE_HI_LSB      40
#define PKT_DSC_NONCE_HI_BITS     EIGHT_BITS
#define PKT_DSC_NONCE_HI_MASK (PKT_DSC_NONCE_HI_BITS << PKT_DSC_NONCE_HI_LSB)

	/* #define PKT_DSC_CKSUM_DST_ADDR */
#define PKT_DSC_CKSUM_DST_ADDR_LSB  5
#define PKT_DSC_CKSUM_DST_ADDR_BITS THIRTY_FIVE_BITS
#define PKT_DSC_CKSUM_DST_ADDR_MASK (PKT_DSC_CKSUM_DST_ADDR_BITS << PKT_DSC_CKSUM_DST_ADDR_LSB)

	/* #define PKT_DSC_IV_OFF_HI */
#define PKT_DSC_IV_OFF_HI_LSB      0
#define PKT_DSC_IV_OFF_HI_BITS     FIVE_BITS
#define PKT_DSC_IV_OFF_HI_MASK   (PKT_DSC_IV_OFF_HI_BITS << PKT_DSC_IV_OFF_HI_LSB)

#else /* B0 */

	/* **********************************************************************
	 *       packetdescriptor_t.dstLLWMask
	 * **********************************************************************
	 */

	/* #define PKT_DSC_LASTWORD */
#define PKT_DSC_LASTWORD_128       0
#define PKT_DSC_LASTWORD_96MASK    1
#define PKT_DSC_LASTWORD_64MASK    2
#define PKT_DSC_LASTWORD_32MASK    3
#define PKT_DSC_LASTWORD_LSB       58
#define PKT_DSC_LASTWORD_BITS      TWO_BITS
#define PKT_DSC_LASTWORD_MASK      \
(PKT_DSC_LASTWORD_BITS << PKT_DSC_LASTWORD_LSB)

	/* #define PKT_DSC_CPHR_DST_ADDR */
#define PKT_DSC_CPHR_DST_ADDR_LSB  0
#define PKT_DSC_CPHR_DST_ADDR_BITS FOURTY_BITS
#define PKT_DSC_CPHR_DST_ADDR_MASK \
(PKT_DSC_CPHR_DST_ADDR_BITS << PKT_DSC_CPHR_DST_ADDR_LSB)

	/* #define PKT_DSC_CPHR_DST_OFFSET */
#define PKT_DSC_CPHR_DST_OFFSET_LSB   0
#define PKT_DSC_CPHR_DST_OFFSET_BITS  THREE_BITS
#define PKT_DSC_CPHR_DST_OFFSET_MASK \
(PKT_DSC_CPHR_DST_OFFSET_BITS << PKT_DSC_CPHR_DST_OFFSET_LSB)
#endif /* B0 */


	/* ******************************************************************
	 *             Control Error Code and Conditions
	 * ******************************************************************
	 */
#define CTL_ERR_NONE         0x0000   /* No Error */
#define CTL_ERR_CIPHER_OP    0x0001   /* Unknown Cipher Op */
#define CTL_ERR_MODE         0x0002   /* Unknown or Not Allowed Mode */
#define CTL_ERR_CHKSUM_SRC   0x0004   /* Unknown CkSum Src - B0 UNUSED */
#define CTL_ERR_CFB_MASK     0x0008   /* Forbidden CFB Mask - B0 UNUSED */
#define CTL_ERR_OP           0x0010   /* Unknown Ctrl Op - B0 UNUSED */
#define CTL_ERR_UNDEF1       0x0020   /* UNUSED */
#define CTL_ERR_UNDEF2       0x0040   /* UNUSED */
#define CTL_ERR_DATA_READ    0x0080   /* Data Read Error */
#define CTL_ERR_DESC_CTRL    0x0100   /* Descriptor Ctrl Field Error */

#define CTL_ERR_TIMEOUT      0x1000   /* Message Response Timeout */ 

	/* ******************************************************************
	 *             Data Error Code and Conditions
	 * ******************************************************************
	 */
#define DATA_ERR_NONE        0x0000   /* No Error */
#define DATA_ERR_LEN_CIPHER  0x0001   /* Not Enough Data To Cipher */
#define DATA_ERR_IV_ADDR     0x0002   /* Illegal IV Loacation */
#define DATA_ERR_WD_LEN_AES  0x0004   /* Illegal Nb Words To AES */
#define DATA_ERR_BYTE_COUNT  0x0008   /* Illegal Pad And ByteCount Spec */
#define DATA_ERR_LEN_CKSUM   0x0010   /* Not Enough Data To CkSum */
#define DATA_ERR_OP          0x0020   /* Unknown Data Op */
#define DATA_ERR_UNDEF1      0x0040   /* UNUSED */
#define DATA_ERR_READ        0x0080   /* Data Read Error */
#define DATA_ERR_WRITE       0x0100   /* Data Write Error */


	/*
	 * Common Descriptor 
	 * NOTE:  Size of struct is size of cacheline.
	 */

	typedef struct OperationDescriptor_s {
		uint64_t             phys_self;
		uint32_t             stn_id;
		uint32_t             flags;
		uint32_t             cpu;
		uint32_t             seq_num;
		uint64_t             reserved;
	} operationdescriptor_t, *OperationDescriptor_pt;


/*
 * This defines the security data descriptor format
 */
typedef struct PacketDescriptor_s {
	uint64_t             srcLengthIVOffUseIVNext;
#ifdef B0
	uint64_t             dstDataSettings;
	uint64_t             authDstNonceLow;
	uint64_t             ckSumDstNonceHiCFBMaskLLWMask;
#else /* B0 */
	uint64_t             dstLLWMask;
	uint64_t             authDst;
	uint64_t             ckSumDst;
#endif /* B0 */
} packetdescriptor_t, *PacketDescriptor_pt;

typedef struct {
	__u8 *user_auth;
	__u8 *user_src;
	__u8 *user_dest;
	__u8 *user_state;
	__u8 *kern_auth; 
	__u8 *kern_src; 
	__u8 *kern_dest;
	__u8 *kern_state;
	__u8 *aligned_auth;
	__u8 *aligned_src;
	__u8 *aligned_dest;
	__u8 *aligned_state;
}  phxdrv_user_t, *phxdrv_user_pt;

typedef struct symkey_desc {
	operationdescriptor_t   op_ctl;    /* size is cacheline */
	packetdescriptor_t      pkt_desc;  /* size is cacheline  */
	controldescriptor_t     ctl_desc;  /*  makes this aligned */
	__u64                   control;   /* message word0 */
	__u64                   data;		/* message word1 */
	__u64                   ctl_result;
	__u64                   data_result; 
	struct symkey_desc      *alloc;    /* real allocated addr */
	phxdrv_user_t           user;
	volatile atomic_t       flag_complete;
	struct semaphore        sem_complete;
	wait_queue_t            submit_wait;
} symkey_desc_t, *symkey_desc_pt;


/*
 * **************************************************************************
 *                                 RSA Block
 * **************************************************************************
 */

/*
 *                                 RSA Block
 *                                 =========
 *
 * A 2-word message ring descriptor is used to pass all information
 * pertaining to the RSA operation:
 *
 *  63  61 60    54     53            52            51       50            40 39          5 4      3 2                      0
 *  -------------------------------------------------------------------------------------------------------------------------
 * | Ctrl | UNUSED | Valid Op | Block Width | Load Constant | Exponent Width | Source Addr | UNUSED | Global src data offset |
 *  -------------------------------------------------------------------------------------------------------------------------
 *    3       7         1             1             1              11              35          2                3
 *
 *
 *  63  61 60    54     53         52             51     50           40 39        5 4      3 2                      0
 *  ------------------------------------------------------------------------------------------------------------------
 * | Ctrl | UNUSED | WRB_COH | WRB_L2ALLOC | DF_L2ALLOC | Modulus Width | Dest Addr | UNUSED | Global dst data offset |
 *  ------------------------------------------------------------------------------------------------------------------
 *    3       7         1          1              1            11           35         2                3
 *
 *
 *             Valid Op                =        1'b1       Will cause operation to start; descriptors sent back at end of operation
 *                                              1'b0       No operation performed; descriptors sent back right away
 *             Block Width             =        1'b1       1024 bit op
 *                                     =        1'b0       512  bit op
 *             Load Constant           =        1'b1       Load constant from data structure
 *                                              1'b0       Preserve old constant (this assumes Source Addr points to RSAData_pt->Exponent
 *                                                         or that the length of Constant is 0)
 *             Exponent Width          =                   11-bit expression of exponent width EXPRESSED IN NUMBER OF BITS
 *             Global src data offset  =                   Nb BYTES to right-shift data by before presenting it to engine
 *                                                         (0-7); allows realignment of byte-aligned, non-double-word aligned data
 *             Source Addr             =                   35 MSB of pointer to source address (i.e., cache-line aligned)
 *             Modulus Width           =                   11-bit expression of modulus width EXPRESSED IN NUMBER OF BITS
 *             Global dst data offset  =                   Nb BYTES to left-shift (double-word boundary aligned) data by before writing it to memory
 *             Dest Addr               =                   35 MSB of pointer to destination address (i.e., cache-line aligned)
 */

/* #define PUBKEY_CTL_CTL */
#define PUBKEY_CTL_CTL_LSB         61
#define PUBKEY_CTL_CTL_BITS        THREE_BITS
#define PUBKEY_CTL_CTL_MASK        (PUBKEY_CTL_CTL_BITS << PUBKEY_CTL_CTL_LSB)

/* #define PUBKEY_CTL_VALID */
#define PUBKEY_CTL_VALID_FALSE     0
#define PUBKEY_CTL_VALID_TRUE      1
#define PUBKEY_CTL_VALID_LSB       53
#define PUBKEY_CTL_VALID_BITS      ONE_BIT
#define PUBKEY_CTL_VALID_MASK      \
(PUBKEY_CTL_VALID_BITS << PUBKEY_CTL_VALID_LSB)

	/* #define PUBKEY_CTL_BLKWIDTH */
#define PUBKEY_CTL_BLKWIDTH_512    0
#define PUBKEY_CTL_BLKWIDTH_1024   1
#define PUBKEY_CTL_BLKWIDTH_LSB    52
#define PUBKEY_CTL_BLKWIDTH_BITS   ONE_BIT
#define PUBKEY_CTL_BLKWIDTH_MASK   \
(PUBKEY_CTL_BLKWIDTH_BITS << PUBKEY_CTL_BLKWIDTH_LSB)

	/* #define PUBKEY_CTL_LD_CONST */
#define PUBKEY_CTL_LD_CONST_OLD    0
#define PUBKEY_CTL_LD_CONST_NEW    1
#define PUBKEY_CTL_LD_CONST_LSB    51
#define PUBKEY_CTL_LD_CONST_BITS   ONE_BIT
#define PUBKEY_CTL_LD_CONST_MASK   \
(PUBKEY_CTL_LD_CONST_BITS << PUBKEY_CTL_LD_CONST_LSB)

	/* #define PUBKEY_CTL_EXPWIDTH */
#define PUBKEY_CTL_EXPWIDTH_LSB    40
#define PUBKEY_CTL_EXPWIDTH_BITS   ELEVEN_BITS
#define PUBKEY_CTL_EXPWIDTH_MASK   \
(PUBKEY_CTL_EXPWIDTH_BITS << PUBKEY_CTL_EXPWIDTH_LSB)

	/* #define PUBKEY_CTL_SRCADDR */
#define PUBKEY_CTL_SRCADDR_LSB     0  
#define PUBKEY_CTL_SRCADDR_BITS    FOURTY_BITS
#define PUBKEY_CTL_SRCADDR_MASK    \
(PUBKEY_CTL_SRCADDR_BITS << PUBKEY_CTL_SRCADDR_LSB)

	/* #define PUBKEY_CTL_SRC_OFFSET */
#define PUBKEY_CTL_SRC_OFFSET_LSB  0  
#define PUBKEY_CTL_SRC_OFFSET_BITS THREE_BITS
#define PUBKEY_CTL_SRC_OFFSET_MASK \
(PUBKEY_CTL_SRC_OFFSET_BITS << PUBKEY_CTL_SRC_OFFSET_LSB)


	/* #define PUBKEY_CTL1_CTL */
#define PUBKEY_CTL1_CTL_LSB        61
#define PUBKEY_CTL1_CTL_BITS       THREE_BITS
#define PUBKEY_CTL1_CTL_MASK       (PUBKEY_CTL_CTL_BITS << PUBKEY_CTL_CTL_LSB)

	/* #define PUBKEY_CTL1_MODWIDTH */
#define PUBKEY_CTL1_MODWIDTH_LSB   40     
#define PUBKEY_CTL1_MODWIDTH_BITS  ELEVEN_BITS
#define PUBKEY_CTL1_MODWIDTH_MASK  \
(PUBKEY_CTL1_MODWIDTH_BITS << PUBKEY_CTL1_MODWIDTH_LSB)

	/* #define PUBKEY_CTL1_DSTADDR */
#define PUBKEY_CTL1_DSTADDR_LSB    0
#define PUBKEY_CTL1_DSTADDR_BITS   FOURTY_BITS
#define PUBKEY_CTL1_DSTADDR_MASK   \
(PUBKEY_CTL1_DSTADDR_BITS << PUBKEY_CTL1_DSTADDR_LSB)

	/* #define PUBKEY_CTL1_DST_OFFSET */
#define PUBKEY_CTL1_DST_OFFSET_LSB    0
#define PUBKEY_CTL1_DST_OFFSET_BITS   THREE_BITS
#define PUBKEY_CTL1_DST_OFFSET_MASK   \
(PUBKEY_CTL1_DST_OFFSET_BITS << PUBKEY_CTL1_DST_OFFSET_LSB)

	/*
	 * Upon completion of operation, the RSA block returns a 2-word free descriptor
	 * in the following format:
	 *
	 *  63  61 60            54 53   52 51       49  48          40 39             0
	 *  ----------------------------------------------------------------------------
	 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | Control Error | Source Address |
	 *  ----------------------------------------------------------------------------
	 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl |   Data Error  | Dest Address   |
	 *  ----------------------------------------------------------------------------
	 *
	 * The Control and Data Error codes are enumerated below
	 *
	 *                                Error conditions
	 *                                ================
	 *
	 *             Control Error Code                  Control Error Condition
	 *             ------------------                  -----------------------
	 *             9'h000                              No Error
	 *             9'h001                              UNUSED
	 *             9'h002                              UNUSED
	 *             9'h004                              UNUSED
	 *             9'h008                              UNUSED
	 *             9'h010                              UNUSED
	 *             9'h020                              UNUSED
	 *             9'h040                              UNUSED
	 *             9'h080                              Data Read Error
	 *             9'h100                              Descriptor Ctrl Field Error        (D0.Ctrl != SOP || D1.Ctrl != EOP)
	 *
	 *             Data Error Code                     Data Error Condition
	 *             ---------------                     --------------------
	 *             9'h000                              No Error
	 *             9'h001                              Exponent Width > Block Width
	 *             9'h002                              Modulus Width  > Block Width
	 *             9'h004                              UNUSED
	 *             9'h008                              UNUSED
	 *             9'h010                              UNUSED
	 *             9'h020                              UNUSED
	 *             9'h040                              UNUSED
	 *             9'h080                              Data Read Error
	 *             9'h100                              UNUSED
	 */

	/*
	 * Result Data Word for Message Ring Descriptor
	 */

	/* #define PUBKEY_RSLT_CTL_CTL */
#define PUBKEY_RSLT_CTL_CTL_LSB        61
#define PUBKEY_RSLT_CTL_CTL_BITS       THREE_BITS
#define PUBKEY_RSLT_CTL_CTL_MASK       \
(PUBKEY_RSLT_CTL_CTL_BITS << PUBKEY_RSLT_CTL_CTL_LSB)

	/* #define PUBKEY_RSLT_CTL_DST_ID */
#define PUBKEY_RSLT_CTL_DST_ID_LSB     54
#define PUBKEY_RSLT_CTL_DST_ID_BITS    SEVEN_BITS
#define PUBKEY_RSLT_CTL_DST_ID_MASK    \
(PUBKEY_RSLT_CTL_DST_ID_BITS << PUBKEY_RSLT_CTL_DST_ID_LSB)

	/* #define PUBKEY_RSLT_CTL_DESC_CTL */
#define PUBKEY_RSLT_CTL_DESC_CTL_LSB   49
#define PUBKEY_RSLT_CTL_DESC_CTL_BITS  THREE_BITS
#define PUBKEY_RSLT_CTL_DESC_CTL_MASK  \
(PUBKEY_RSLT_CTL_DESC_CTL_BITS << PUBKEY_RSLT_CTL_DESC_CTL_LSB)


	/* #define PUBKEY_RSLT_CTL_ERROR */
#define PUBKEY_RSLT_CTL_ERROR_LSB      40
#define PUBKEY_RSLT_CTL_ERROR_BITS     NINE_BITS
#define PUBKEY_RSLT_CTL_ERROR_MASK     \
(PUBKEY_RSLT_CTL_ERROR_BITS << PUBKEY_RSLT_CTL_ERROR_LSB)

	/* #define PUBKEY_RSLT_CTL_SRCADDR */
#define PUBKEY_RSLT_CTL_SRCADDR_LSB    0
#define PUBKEY_RSLT_CTL_SRCADDR_BITS   FOURTY_BITS
#define PUBKEY_RSLT_CTL_SRCADDR_MASK   \
(PUBKEY_RSLT_CTL_SRCADDR_BITS << PUBKEY_RSLT_CTL_SRCADDR_LSB)


	/* #define PUBKEY_RSLT_DATA_CTL */
#define PUBKEY_RSLT_DATA_CTL_LSB       61
#define PUBKEY_RSLT_DATA_CTL_BITS      THREE_BITS
#define PUBKEY_RSLT_DATA_CTL_MASK      \
(PUBKEY_RSLT_DATA_CTL_BITS << PUBKEY_RSLT_DATA_CTL_LSB)

	/* #define PUBKEY_RSLT_DATA_DST_ID */
#define PUBKEY_RSLT_DATA_DST_ID_LSB    54
#define PUBKEY_RSLT_DATA_DST_ID_BITS   SEVEN_BITS
#define PUBKEY_RSLT_DATA_DST_ID_MASK   \
(PUBKEY_RSLT_DATA_DST_ID_BITS << PUBKEY_RSLT_DATA_DST_ID_LSB)

	/* #define PUBKEY_RSLT_DATA_DESC_CTL */
#define PUBKEY_RSLT_DATA_DESC_CTL_LSB  49
#define PUBKEY_RSLT_DATA_DESC_CTL_BITS THREE_BITS
#define PUBKEY_RSLT_DATA_DESC_CTL_MASK \
(PUBKEY_RSLT_DATA_DESC_CTL_BITS << PUBKEY_RSLT_DATA_DESC_CTL_LSB)

	/* #define PUBKEY_RSLT_DATA_ERROR */
#define PUBKEY_RSLT_DATA_ERROR_LSB     40
#define PUBKEY_RSLT_DATA_ERROR_BITS    NINE_BITS
#define PUBKEY_RSLT_DATA_ERROR_MASK    \
(PUBKEY_RSLT_DATA_ERROR_BITS << PUBKEY_RSLT_DATA_ERROR_LSB)

	/* #define PUBKEY_RSLT_DATA_DSTADDR */
#define PUBKEY_RSLT_DATA_DSTADDR_LSB   40
#define PUBKEY_RSLT_DATA_DSTADDR_BITS  FOURTY_BITS
#define PUBKEY_RSLT_DATA_DSTADDR_MASK  \
(PUBKEY_RSLT_DATA_DSTADDR_BITS << PUBKEY_RSLT_DATA_DSTADDR_LSB)

	/* 
	 * ******************************************************************
	 *             RSA Block - Data Error Code and Conditions
	 * ******************************************************************
	 */

#define PK_CTL_ERR_NONE        0x0000   /* No Error */
#define PK_CTL_ERR_READ        0x0080   /* Data Read Error */
#define PK_CTL_ERR_DESC        0x0100   /* Descriptor Ctrl Field Error  (D0.Ctrl != SOP || D1.Ctrl != EOP) */
#define PK_CTL_ERR_TIMEOUT     0x1000   /* Message Responce Timeout */ 

#define PK_DATA_ERR_NONE       0x0000   /* No Error */
#define PK_DATA_ERR_EXP_WIDTH  0x0001   /* Exponent Width > Block Width */
#define PK_DATA_ERR_MOD_WIDTH  0x0002   /* Modulus Width  > Block Width */
#define PK_DATA_ERR_READ       0x0080   /* Data Read Error */


	/*
	 * This defines the RSA data format
	 */
	/*
	 * typedef struct RSAData_s {
	 *  uint64_t            Constant;
	 *  uint64_t            Exponent;
	 *  uint64_t            Modulus;
	 * uint64_t            Message;
	 *} RSAData_t, *RSAData_pt;
	 *
	 * typedef RSAData_t DHData_t;
	 * typedef RSAData_pt DHData_pt;
	 */

	typedef struct UserPubData_s {
		uint8_t            *source;
		uint8_t            *user_result;
		uint32_t           result_length;
	} UserPubData_t, *UserPubData_pt;

typedef struct pubkey_desc {
	operationdescriptor_t   op_ctl;    /* size is cacheline */
	uint8_t                 source[1024];
	uint8_t                 dest[256];    /* 1024 makes cacheline-aligned */
	__u64                   control0;
	__u64                   control1;
	__u64                   ctl_result;
	__u64                   data_result; 
	struct pubkey_desc      *alloc;
	UserPubData_t           kern;       /* ptrs for temp buffers */
	volatile atomic_t       flag_complete;
	struct semaphore        sem_complete;
	wait_queue_t            submit_wait;
} pubkey_desc_t, *pubkey_desc_pt;


#endif /* _PHXDESC_H_ */
