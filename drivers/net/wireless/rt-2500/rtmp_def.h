/*************************************************************************** 
 * RT2400/RT2500 SourceForge Project - http://rt2x00.serialmonkey.com      * 
 *                                                                         * 
 *   This program is free software; you can redistribute it and/or modify  * 
 *   it under the terms of the GNU General Public License as published by  * 
 *   the Free Software Foundation; either version 2 of the License, or     * 
 *   (at your option) any later version.                                   * 
 *                                                                         * 
 *   This program is distributed in the hope that it will be useful,       * 
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        * 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         * 
 *   GNU General Public License for more details.                          * 
 *                                                                         * 
 *   You should have received a copy of the GNU General Public License     * 
 *   along with this program; if not, write to the                         * 
 *   Free Software Foundation, Inc.,                                       * 
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             * 
 *                                                                         * 
 *   Licensed under the GNU GPL                                            * 
 *   Original code supplied under license from RaLink Inc, 2004.           * 
 ***************************************************************************/ 

 /*************************************************************************** 
 *      Module Name: rtmp_def.h
 *              
 *      Abstract: Miniport related definition header
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      PaulL           1st  Aug 02     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#ifndef __RTMP_DEF_H__
#define __RTMP_DEF_H__

//
//  Debug information verbosity: lower values indicate higher urgency
//
#define RT_DEBUG_ERROR      KERN_ERR
#define RT_DEBUG_WARN       KERN_WARNING
#define RT_DEBUG_TRACE      KERN_NOTICE
#define RT_DEBUG_INFO       KERN_INFO
#define RT_DEBUG_LOUD       KERN_DEBUG

//
// update the driver version number every time you release a new driver
// The high word is the major version. The low word is the minor version.
//
#define NIC_VENDOR_DRIVER_VERSION   0x00010001

//
// NDIS media type, current is ethernet, change if native wireless supported
//
#define NIC_PCI_HDR_LENGTH      0xe2
#define NIC_MAX_PACKET_SIZE     2304
#define NIC_HEADER_SIZE         14

//
// interface type, we use PCI
//

//
// buffer size passed in NdisMQueryAdapterResources
// We should only need three adapter resources (IO, interrupt and memory),
// Some devices get extra resources, so have room for 10 resources
//

//
// IO space length
//

//
// Entry number for each DMA descriptor ring
//
#define TX_RING_SIZE            48
#define ATIM_RING_SIZE          4
#define PRIO_RING_SIZE          16    // 8
#define RX_RING_SIZE            32
#define BEACON_RING_SIZE        1
#define DESCRIPTOR_REQUIRED     ((TX_RING_SIZE) + (ATIM_RING_SIZE) + (PRIO_RING_SIZE) + (RX_RING_SIZE) + (BEACON_RING_SIZE))
#define OTHER_DESC_REQUIRED     ((ATIM_RING_SIZE) + (PRIO_RING_SIZE) + (RX_RING_SIZE) + (BEACON_RING_SIZE))
#define MGMT_RING_SIZE          32
#define RING_DESCRIPTOR_SIZE    48
#define TX_BUFFER_SIZE          2048
#define PRIO_BUFFER_SIZE        1024  // 2048
#define RX_BUFFER_SIZE          2048
#define ATIM_BUFFER_SIZE        512
#define BEACON_BUFFER_SIZE      2048
#define MAX_FRAME_SIZE          2346                    // Maximum 802.11 frame size
#define ALLOC_RX_PACKET_POOL    (RX_RING_SIZE)
#define ALLOC_RX_BUFFER_POOL    (ALLOC_RX_PACKET_POOL)
#define TX_RING                 0xa
#define ATIM_RING               0xb
#define PRIO_RING               0xc
#define RX_RING                 0xd
#define MAX_TX_PROCESS          4
#define MAX_RX_PROCESS          4
#define MAX_CLIENT              4
#define MAX_MCAST_LIST_SIZE     32

//  RTMP_ADAPTER flags
#define fRTMP_ADAPTER_MAP_REGISTER          0x00000001
#define fRTMP_ADAPTER_INTERRUPT_IN_USE      0x00000002
#define fRTMP_ADAPTER_HARDWARE_ERROR        0x00000004
#define fRTMP_ADAPTER_MLME_INITIALIZED      0x00000008
#define fRTMP_ADAPTER_SEND_PACKET_ERROR     0x00000010
#define fRTMP_ADAPTER_RECEIVE_PACKET_ERROR  0x00000020
#define fRTMP_ADAPTER_HALT_IN_PROGRESS      0x00000040
#define fRTMP_ADAPTER_RESET_IN_PROGRESS     0x00000080
#define fRTMP_ADAPTER_REMOVE_IN_PROGRESS    0x00000100
#define fRTMP_ADAPTER_TX_RING_ALLOCATED     0x00000200
#define fRTMP_ADAPTER_ATIM_RING_ALLOCATED   0x00000400
#define fRTMP_ADAPTER_PRIO_RING_ALLOCATED   0x00000800
#define fRTMP_ADAPTER_RX_RING_ALLOCATED     0x00001000
#define fRTMP_ADAPTER_INTERRUPT_ACTIVE      0x00002000
#define fRTMP_ADAPTER_BSS_SCAN_IN_PROGRESS  0x00004000
#define fRTMP_ADAPTER_MEDIA_STATE_CHANGE    0x00008000
#define fRTMP_ADAPTER_MEDIA_STATE_PENDING   0x00010000
#define fRTMP_ADAPTER_RADIO_OFF             0x00020000
#define	fRTMP_ADAPTER_NIC_NOT_EXIST         0x02000000

// Lock bit for accessing different ring buffers
#define fRTMP_ADAPTER_TX_RING_BUSY          0x80000000
#define fRTMP_ADAPTER_PRIO_RING_BUSY        0x40000000
#define fRTMP_ADAPTER_ATIM_RING_BUSY        0x20000000
#define fRTMP_ADAPTER_RX_RING_BUSY          0x10000000

// Lock bit for accessing different queue
#define fRTMP_ADAPTER_TX_QUEUE_BUSY         0x08000000
#define fRTMP_ADAPTER_PRIO_QUEUE_BUSY       0x04000000

//
// Error code section
//
// NDIS_ERROR_CODE_ADAPTER_NOT_FOUND
#define ERRLOG_READ_PCI_SLOT_FAILED     0x00000101L
#define ERRLOG_WRITE_PCI_SLOT_FAILED    0x00000102L
#define ERRLOG_VENDOR_DEVICE_NOMATCH    0x00000103L

// NDIS_ERROR_CODE_ADAPTER_DISABLED
#define ERRLOG_BUS_MASTER_DISABLED      0x00000201L

// NDIS_ERROR_CODE_UNSUPPORTED_CONFIGURATION
#define ERRLOG_INVALID_SPEED_DUPLEX     0x00000301L
#define ERRLOG_SET_SECONDARY_FAILED     0x00000302L

// NDIS_ERROR_CODE_OUT_OF_RESOURCES
#define ERRLOG_OUT_OF_MEMORY            0x00000401L
#define ERRLOG_OUT_OF_SHARED_MEMORY     0x00000402L
#define ERRLOG_OUT_OF_MAP_REGISTERS     0x00000403L
#define ERRLOG_OUT_OF_BUFFER_POOL       0x00000404L
#define ERRLOG_OUT_OF_NDIS_BUFFER       0x00000405L
#define ERRLOG_OUT_OF_PACKET_POOL       0x00000406L
#define ERRLOG_OUT_OF_NDIS_PACKET       0x00000407L
#define ERRLOG_OUT_OF_LOOKASIDE_MEMORY  0x00000408L

// NDIS_ERROR_CODE_HARDWARE_FAILURE
#define ERRLOG_SELFTEST_FAILED          0x00000501L
#define ERRLOG_INITIALIZE_ADAPTER       0x00000502L
#define ERRLOG_REMOVE_MINIPORT          0x00000503L

// NDIS_ERROR_CODE_RESOURCE_CONFLICT
#define ERRLOG_MAP_IO_SPACE             0x00000601L
#define ERRLOG_QUERY_ADAPTER_RESOURCES  0x00000602L
#define ERRLOG_NO_IO_RESOURCE           0x00000603L
#define ERRLOG_NO_INTERRUPT_RESOURCE    0x00000604L
#define ERRLOG_NO_MEMORY_RESOURCE       0x00000605L



//============================================================
// Length definitions
#define PEER_KEY_NO                       2
#define CRC_LEN                           4
#define TIMESTAMP_LEN                     8
#define MAX_LEN_OF_SUPPORTED_RATES        12    // 1, 2, 5.5, 11, 6, 9, 12, 18, 24, 36, 48, 54
#define MAX_NUM_OF_POWER_LEVEL            8
#define MAX_NUM_OF_DOMAIN                 8
#define MAX_LEN_OF_KEY                    32      // 32 octets == 256 bits, Redefine for WPA
#define MAX_LEN_OF_CHANNELS               42      // 14 channels @2.4G +  12@UNII + 4 @MMAC + 11 @HiperLAN2 + 1 as NULL termination
#define MAX_LEN_OF_PEER_KEY               16
#define MAC_HDR_LEN                       24
#define MAX_LEN_OF_MANUFACTURE_ID         32
#define MAX_LEN_OF_PRODUCT_ID             32
#define MAX_LEN_OF_MAC_TABLE              32
#define MAX_LEN_OF_SSID                   32
#define CIPHER_TEXT_LEN                   128
#define HASH_TABLE_SIZE                   256
#define MAX_LEN_OF_MLME_BUFFER            1024
#define MAX_FRAME_LEN                     2338
#define MAX_VIE_LEN                       128   // New for WPA cipher suite variable IE sizes.
#define MAX_MLME_HANDLER_MEMORY           20    //each them cantains  MAX_LEN_OF_MLME_BUFFER size 
#define MAX_INI_BUFFER_SIZE               1024

#define MAX_TX_POWER_LEVEL                100   /* mW */
#define MAX_RSSI_TRIGGER                 -10    /* dBm */
#define MIN_RSSI_TRIGGER                 -200   /* dBm */
#define MAX_FRAG_THRESHOLD                2346  /* byte count */
#define MIN_FRAG_THRESHOLD                256   /* byte count */
#define MAX_RTS_THRESHOLD                 2347  /* byte count */

// key related definitions
#define SHARE_KEY_NO                      4
#define MAX_LEN_OF_SHARE_KEY              16
#define PAIRWISE_KEY_NO                   4
#define GROUP_KEY_NO                      4

// power status related definitions
#define PWR_ACTIVE                        0
#define PWR_SAVE                          1
#define PWR_UNKNOWN                       2

// Auth and Assoc mode related definitions
#define AUTH_MODE                         0x10
#define ASSOC_MODE                        0x20

#define AUTH_MODE_OPEN                    0x00
#define AUTH_MODE_SHARED                  0x01
#define AUTH_MODE_AUTO_SWITCH             0x03
#define AUTH_MODE_DEAUTH                  0x04
#define AUTH_MODE_UPLAYER                 0x05 // reserved for 802.11i use

#define ASSOC_MODE_DISASSOC               0x04
#define ASSOC_MODE_ASSOC                  0x05

// BSS Type definitions
#define BSS_INDEP                         0  // = Ndis802_11IBSS
#define BSS_INFRA                         1  // = Ndis802_11Infrastructure
#define BSS_ANY                           2  // = Ndis802_11AutoUnknown
#define BSS_MONITOR			  3  // = Ndis802_11Monitor
// #define BSS_UNKNOWN                       0xff


// WEP related definitions
// #define WEP_DISABLE                       0
// #define WEP_ENABLE                        1
// #define WEP_KEY_ABSENT                    2
// #define WEP_NOT_SUPPORTED                 3

// value of FrameDesc.priority
// #define PRIO_CONTENTION                   0
// #define PRIO_CONTENTION_FREE              1

// value of auth_algorithm in Authentication frame body

// Reason code definitions
#define REASON_RESERVED                   0
#define REASON_UNSPECIFY                  1
#define REASON_NO_LONGER_VALID            2
#define REASON_DEAUTH_STA_LEAVING         3
#define REASON_DISASSOC_INACTIVE          4
#define REASON_DISASSPC_AP_UNABLE         5
#define REASON_CLS2ERR                    6
#define REASON_CLS3ERR                    7
#define REASON_DISASSOC_STA_LEAVING       8
#define REASON_STA_REQ_ASSOC_NOT_AUTH     9
#define REASON_INVALID_IE                 13
#define REASON_MIC_FAILURE                14
#define REASON_4_WAY_HANDSHAKE_TIMEOUT    15
#define REASON_GROUP_KEY_UPDATE_TIMEOUT   16

// Status code definitions
#define MLME_SUCCESS                      0
#define MLME_UNSPECIFY_FAIL               1
#define MLME_CANNOT_SUPPORT_CAP           10
#define MLME_REASSOC_DENY_ASSOC_EXIST     11 
#define MLME_ASSOC_DENY_OUT_SCOPE         12
#define MLME_ALG_NOT_SUPPORT              13
#define MLME_SEQ_NR_OUT_OF_SEQUENCE       14
#define MLME_REJ_CHALLENGE_FAILURE        15
#define MLME_REJ_TIMEOUT                  16
#define MLME_ASSOC_REJ_UNABLE_HANDLE_STA  17
#define MLME_ASSOC_REJ_DATA_RATE          18

#define MLME_ASSOC_REJ_NO_EXT_RATE        22
#define MLME_ASSOC_REJ_NO_EXT_RATE_PBCC   23
#define MLME_ASSOC_REJ_NO_CCK_OFDM        24

#define MLME_INVALID_FORMAT               0x51
#define MLME_FAIL_NO_RESOURCE             0x52
#define MLME_STATE_MACHINE_REJECT         0x53
#define MLME_MAC_TABLE_FAIL               0x54

//IE code
#define IE_SSID                           0
#define IE_SUPP_RATES                     1
#define IE_FH_PARM                        2
#define IE_DS_PARM                        3
#define IE_CF_PARM                        4
#define IE_TIM                            5
#define IE_IBSS_PARM                      6
#define IE_COUNTRY                        7     // 802.11d
#define IE_802_11D_REQUEST                10    // 802.11d
#define IE_CHALLENGE_TEXT                 16
#define IE_POWER_CONSTRAINT               32    // 802.11h d3.3
#define IE_POWER_CAPABILITY               33    // 802.11h d3.3
#define IE_TPC_REQUEST                    34    // 802.11h d3.3
#define IE_TPC_REPORT                     35    // 802.11h d3.3
#define IE_SUPP_CHANNELS                  36    // 802.11h d3.3
#define IE_CHANNEL_SWITCH_ANNOUNCEMENT    37    // 802.11h d3.3
#define IE_MEASUREMENT_REQUEST            38    // 802.11h d3.3
#define IE_MEASUREMENT_REPORT             39    // 802.11h d3.3
#define IE_QUIET                          40    // 802.11h d3.3
#define IE_IBSS_DFS                       41    // 802.11h d3.3
#define IE_ERP                            42    // 802.11g
#define IE_EXT_SUPP_RATES                 50    // 802.11g
#define IE_WPA                            221   // WPA
#define IE_RSN                            48    // 802.11i d3.0

#define CNTL_FUNC_SIZE                    1

// Message type for the MLME state machine
// Messages for Associate state machine
#define ASSOC_MACHINE_BASE          0

#define MT2_MLME_ASSOC_REQ          0
#define MT2_MLME_REASSOC_REQ        1
#define MT2_MLME_DISASSOC_REQ       2  
#define MT2_PEER_DISASSOC_REQ       3
#define MT2_PEER_ASSOC_REQ          4
#define MT2_PEER_ASSOC_RSP          5
#define MT2_PEER_REASSOC_REQ        6
#define MT2_PEER_REASSOC_RSP        7
//#define MT2_CLS3ERR                 8
#define MT2_DISASSOC_TIMEOUT        8
#define MT2_ASSOC_TIMEOUT           9
#define MT2_REASSOC_TIMEOUT         10

#define MAX_ASSOC_MSG               11

// Messages for Authentication state machine
#define AUTH_MACHINE_BASE           11

#define MT2_MLME_AUTH_REQ           11
//#define MT2_MLME_DEAUTH_REQ         12
//#define MT2_CLS2ERR                 13
#define MT2_PEER_AUTH_EVEN          14
#define MT2_AUTH_TIMEOUT            15

#define MAX_AUTH_MSG                 5

// Messages for authentication response state machine
#define AUTH_RSP_MACHINE_BASE       16

#define MT2_AUTH_CHALLENGE_TIMEOUT  16
#define MT2_PEER_AUTH_ODD           17
#define MT2_PEER_DEAUTH             18

#define MAX_AUTH_RSP_MSG             3

// Messages for the sync state machine
#define SYNC_MACHINE_BASE           19

#define MT2_MLME_SCAN_REQ           19
#define MT2_MLME_JOIN_REQ           20
#define MT2_MLME_START_REQ          21
#define MT2_PEER_BEACON             22
#define MT2_PEER_PROBE_RSP          23
#define MT2_PEER_ATIM               24
#define MT2_SCAN_TIMEOUT            25
#define MT2_BEACON_TIMEOUT          26
#define MT2_ATIM_TIMEOUT            27
#define MT2_PEER_PROBE_REQ          28

#define MAX_SYNC_MSG                10

// MIB access
#define MT2_GET_REQ                 31
#define MT2_SET_REQ                 32
#define MT2_RESET_REQ               33

// Confirm message
#define MT2_ASSOC_CONF              34
#define MT2_AUTH_CONF               35
#define MT2_DEAUTH_CONF             36
#define MT2_DISASSOC_CONF           37
#define MT2_REASSOC_CONF            38
#define MT2_PWR_MGMT_CONF           39
#define MT2_JOIN_CONF               40
#define MT2_SCAN_CONF               41
#define MT2_START_CONF              42
#define MT2_GET_CONF                43
#define MT2_SET_CONF                44
#define MT2_RESET_CONF              45

// Indication message
#define MT2_DEAUTH_IND              46
#define MT2_ASSOC_IND               47
#define MT2_DISASSOC_IND            48
#define MT2_REASSOC_IND             49
#define MT2_AUTH_IND                50

#define MT2_SCAN_END_CONF           51  // For scan end
#define MT2_MLME_ROAMING_REQ        52

/* #define TXSTATUS_SUCCESS                0 */
/* #define TXSTATUS_FAIL_RETRY_LIMIT       1 */
/* #define TXSTATUS_EXCESSIVE_LENGTH       2 */
/* #define TXSTATUS_NON_NULL_SOURCE_ROUTE  3 */
/* #define TXSTATUS_UNSUPPORTED_PRIORITY   4 */
/* #define TXSTATUS_UNSUPPORTED_SERVICE    5 */
/* #define TXSTATUS_UNAVAILABLE_PRIORITY   6   // CF with no PC available; down-grade to contention */
/* #define TXSTATUS_UNAVAILABLE_SERVICE    7   // strictly-ordered but STA isn't active */
/* #define TXSTATUS_FAIL_LIFE_TIME         8 */
/* #define TXSTATUS_FAIL_NOBSS             9 */
/* #define TXSTATUS_FAIL_NULL_KEY          10 */

// value domain of MacHdr.tyte, which is b3..b2 of the 1st-byte of MAC header
#define BTYPE_MGMT              0   // 00
#define BTYPE_CNTL              1   // 01
#define BTYPE_DATA              2   // 10

// value domain of MacHdr.subtype, which is b7..4 of the 1st-byte of MAC header
// Management frame
#define SUBTYPE_ASSOC_REQ       0
#define SUBTYPE_ASSOC_RSP       1
#define SUBTYPE_REASSOC_REQ     2
#define SUBTYPE_REASSOC_RSP     3
#define SUBTYPE_PROBE_REQ       4
#define SUBTYPE_PROBE_RSP       5
#define SUBTYPE_BEACON          8
#define SUBTYPE_ATIM            9
#define SUBTYPE_DISASSOC        10
#define SUBTYPE_AUTH            11
#define SUBTYPE_DEAUTH          12
#define SUBTYPE_ACTION          13

// Control Frame
#define SUBTYPE_BLOCK_ACK_REQ   8
#define SUBTYPE_BLOCK_ACK       9
#define SUBTYPE_PS_POLL         10
#define SUBTYPE_RTS             11  // 1011
#define SUBTYPE_CTS             12  // 1100
#define SUBTYPE_ACK             13  // 1101
#define SUBTYPE_CFEND           14
#define SUBTYPE_CFEND_CFACK     15

// Data Frame
#define SUBTYPE_DATA                0
#define SUBTYPE_DATA_CFACK          1
#define SUBTYPE_DATA_CFPOLL         2
#define SUBTYPE_DATA_CFACK_CFPOLL   3
#define SUBTYPE_NULL_FUNC           4
#define SUBTYPE_CFACK               5  // 0101
#define SUBTYPE_CFPOLL              6
#define SUBTYPE_CFACK_CFPOLL        7
#define SUBTYPE_QDATA               8
#define SUBTYPE_QDATA_CFACK         9
#define SUBTYPE_QDATA_CFPOLL        10
#define SUBTYPE_QDATA_CFACK_CFPOLL  11
#define SUBTYPE_QOS_NULL            12
#define SUBTYPE_QOS_CFACK           13
#define SUBTYPE_QOS_CFPOLL          14
#define SUBTYPE_QOS_CFACK_CFPOLL    15

#define ASSOC_STATE_MACHINE        1
#define AUTH_STATE_MACHINE         2
#define AUTH_RSP_STATE_MACHINE     3
#define SYNC_STATE_MACHINE         4
#define MLME_CNTL_STATE_MACHINE    5
#define WPA_PSK_STATE_MACHINE       6

//
// rtmp_data.c use these definition
//
#define LENGTH_802_11           24
#define LENGTH_802_11_AND_H     30
#define LENGTH_802_11_CRC_H     34
#define LENGTH_802_11_CRC       28
#define LENGTH_802_3            14
#define LENGTH_802_3_TYPE       2
#define LENGTH_802_1_H          8
#define LENGTH_EAPOL_H          4
#define LENGTH_CRC              4
#define MAX_SEQ_NUMBER          0x0fff

#define SUCCESS_WITHOUT_RETRY   0
#define SUCCESS_WITH_RETRY      1
#define FAIL_RETRY_LIMIT        2
#define FAIL_INVALID            3
#define FAIL_OTHER              4

#define RATE_1                  0
#define RATE_2                  1
#define RATE_5_5                2
#define RATE_11                 3
#define RATE_6                  4   // OFDM
#define RATE_9                  5   // OFDM
#define RATE_12                 6   // OFDM
#define RATE_18                 7   // OFDM
#define RATE_24                 8   // OFDM
#define RATE_36                 9   // OFDM
#define RATE_48                 10  // OFDM
#define RATE_54                 11  // OFDM
#define RATE_72                 12
#define RATE_100                13
#define RATE_FIRST_OFDM_RATE    RATE_6
#define RATE_AUTO_SWITCH        255 // for PortCfg.FixedTxRate only

#define IFS_BACKOFF             0
#define IFS_SIFS                1
#define IFS_NEW_BACKOFF         2
#define IFS_NONE                3

#define LONG_RETRY              1
#define SHORT_RETRY             0

// Country Region definition
#define REGION_MIN              0
#define REGION_FCC              0       // 1-11
#define REGION_IC               1       // 1-11
#define REGION_ETSI             2       // 1-13
#define REGION_SPAIN            3       // 10-11
#define REGION_FRANCE           4       // 10-13
#define REGION_MKK              5       // 14
#define REGION_MKK1             6       // 1-14
#define REGION_ISRAEL           7       // 3-9
#define REGION_MAX              REGION_ISRAEL

#define CIPHER_NONE             0
#define CIPHER_WEP64            1
#define CIPHER_WEP128           2
#define CIPHER_TKIP             3
#define CIPHER_AES              4

// Stall execution time for ndisdpracquires[inlock in miniportReset function
#define WAIT_TIME_FOR_SPINLOCK  10      // usec

// value domain for pAdapter->PortCfg.RfType
#define RFIC_2522               0
#define RFIC_2523               1
#define RFIC_2524               2
#define RFIC_2525               3
#define RFIC_2525E              4
#define RFIC_5222               16

// value domain for pAdapter->PortCfg.LedMode and E2PROM
#define LED_MODE_DEFAULT        0
#define LED_MODE_TXRX_ACTIVITY  1
#define LED_MODE_SINGLE         2       // Single LED mode, driver lid the LED as soon as driver up & enable tx activity right away
#define LED_MODE_ASUS           3       // Two LED modes, bit 16 acts as LED_MODE_SINGLE, bit 17 acts as RADIO status.

// RC4 init value, used fro WEP & TKIP
#define PPPINITFCS32  0xffffffff   /* Initial FCS value */

// 802.1X controlled port definition
#define WPA_802_1X_PORT_SECURED         1
#define WPA_802_1X_PORT_NOT_SECURED     2

#define PAIRWISE_KEY            1
#define GROUP_KEY               2

#ifdef BIG_ENDIAN
#define DIR_READ                    0
#define DIR_WRITE                   1
#define TYPE_TXD                    0
#define TYPE_RXD                    1
#endif

#ifdef RALINK_ATE
#define	ATE_STASTOP		0	// Stop Station
#define	ATE_STASTART	1	// Start Station
#define	ATE_TXCONT   	2	// Continuous Transmit
#define	ATE_TXCARR  	3	// Transmit Carrier
#define	ATE_TXFRAME		4	// Transmit Frames
#define	ATE_RXFRAME		5	// Receive Frames
#endif	//#ifdef RALINK_ATE

#endif  // __RTMP_DEF_H__
