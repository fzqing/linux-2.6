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
 *      Module Name: mlme.h
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      John            28th Aug 03     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#ifndef __MLME_H__
#define __MLME_H__

#include "oid.h"

// maximum supported capability information - 
// ESS, IBSS, Privacy, Short Preamble, Short Slot
#define SUPPORTED_CAPABILITY_INFO 0x0433

#define END_OF_ARGS             -1
#define LFSR_MASK               0x80000057
#define TBTT_PRELOAD_TIME       384        // usec. LomgPreamble + 24-byte at 1Mbps
#define MLME_TASK_EXEC_INTV     1000      // 1 sec

#define BEACON_LOST_TIME        (4*HZ)       // 2048 msec = 2 sec

//#define AUTH_KEY_TIMEOUT        500     // unit: msec
//#define AUTH_OPEN_TIMEOUT       200     // unit: msec
#define AUTH_TIMEOUT            300     // unit: msec
#define ASSOC_TIMEOUT           300     // unit: msec
#define JOIN_TIMEOUT            2000     // unit: msec
#define MIN_CHANNEL_TIME        110        // unit: msec, for dual band scan
#define MAX_CHANNEL_TIME        140       // unit: msec, for single band scan
#define	ACTIVE_SCAN_TIME		30			// Active scan waiting for probe response time
#define CW_MIN_IN_BITS          3         // actual CwMin = 2^CW_MIN_IN_BITS - 1 = 7
#define CW_MAX_IN_BITS          8         // actual CwMax = 2^CW_MAX_IN_BITS - 1 = 255

#define RSSI_TO_DBM_OFFSET          120 // for RT2530 RSSI-115 = dBm
#define RSSI_FOR_MID_TX_POWER       55  // -55 db is considered mid-distance
#define RSSI_FOR_LOW_TX_POWER       45  // -45 db is considered very short distance and 
                                        // eligible to use a lower TX power
#define RSSI_FOR_LOWEST_TX_POWER    30
#define MID_TX_POWER_DELTA          0   // -3 db from full TX power upon mid-distance to AP
#define LOW_TX_POWER_DELTA          3   // -8 db from full TX power upon very short distance
#define LOWEST_TX_POWER_DELTA       6  // -12 db from full TX power upon shortest distance

#define RSSI_TRIGGERED_UPON_BELOW_THRESHOLD     0
#define RSSI_TRIGGERED_UPON_EXCCEED_THRESHOLD   1
#define RSSI_THRESHOLD_FOR_ROAMING              25
#define RSSI_DELTA                              5

// Channel Quality Indication
//#define CQI_GOOD_THRESHOLD      70  // >= this threshold means channel quality GOOD
//#define CQI_FAIR_THRESHOLD      50  // >= this threshold means channel quality FAIR
//#define CQI_POOR_THRESHOLD      30  // >= this threshold means channel quality POOR
                                    // < this threshold means channel quality really BAD, link down
#define CQI_IS_GOOD(cqi)        ((cqi) >= 50)
#define CQI_IS_FAIR(cqi)        (((cqi) >= 20) && ((cqi) < 50)) // (((cqi) >= 50) && ((cqi) < 70))
#define CQI_IS_POOR(cqi)        (((cqi) >= 5) && ((cqi) < 20))  // (((cqi) >= 25) && ((cqi) < 50))
#define CQI_IS_BAD(cqi)         ((cqi) < 5)                     // ((cqi) < 25)

// weighting factor to calculate Channel quality, total should be 100%
#define RSSI_WEIGHTING          40
#define TX_WEIGHTING            40
#define RX_WEIGHTING            20

// prime number closest to 256
//#define HASH_TABLE_SIZE                  191 //191 is another prime
// Only allows 32 entries in the table
#define MAC_TABLE_MAX_CAPACITY           32

#define MAC_ENTRY_NOT_USED               0xff
#define CONTENT_NOT_AVAIL                0xaa

// 10 minute of age out
#define MAC_TABLE_AGE_OUT_TIME           0xffffff

#define MAC_ADDR_HASH_ERROR              0xfffffffe
#define MAC_TABLE_UNKNOWN_INDEX          0xff
#define MAC_TABLE_ADDR_NOT_IN            0xfffffffd

#define PEER_KEY_NOT_USED                0
#define PEER_KEY_64_BIT                  64
#define PEER_KEY_128_BIT                 128

#define PEER_KEY_64BIT_LEN               8
#define PEER_KEY_128BIT_LEN              16

#define MAX_LEN_OF_BSS_TABLE             64
#define BSS_NOT_FOUND                    0xFFFFFFFF

#define MAX_LEN_OF_MLME_QUEUE            10

//! assoc state-machine states
#define ASSOC_IDLE                       0
#define ASSOC_WAIT_RSP                   1
#define REASSOC_WAIT_RSP                 2
#define DISASSOC_WAIT_RSP                3
#define MAX_ASSOC_STATE                  4

#define ASSOC_FUNC_SIZE                  44 // 4-state * 12-event

//authentication state machine
#define AUTH_REQ_IDLE                    0
#define AUTH_WAIT_SEQ2                   1
#define AUTH_WAIT_SEQ4                   2
#define MAX_AUTH_STATE                   3

#define AUTH_FUNC_SIZE                   15 // 3-state * 5-event

#define AUTH_RSP_IDLE                    0
#define AUTH_RSP_WAIT_CHAL               1
#define MAX_AUTH_RSP_STATE               2

#define AUTH_RSP_FUNC_SIZE               6 // 2-state * 3-event

// SYNC state machine
#define SYNC_IDLE                        0 // merge NO_BSS,IBSS_IDLE,IBSS_ACTIVE and BSS in to 1 state
#define JOIN_WAIT_BEACON                 1
#define SCAN_LISTEN                      2
#define MAX_SYNC_STATE                   3

#define SYNC_FUNC_SIZE                   30  // 3-state * 10-event

#define SCAN_PASSIVE                     18
#define SCAN_ACTIVE                      19

//WPA State machine
#define WPA_PSK_IDLE                    0
#define MAX_WPA_PSK_STATE               1
#define WPA_PSK_FUNC_SIZE               5

// Control state machine
#define CNTL_IDLE                        100
#define CNTL_WAIT_DISASSOC               101
#define CNTL_WAIT_JOIN                   102
#define CNTL_WAIT_REASSOC                103
#define CNTL_WAIT_START                  104
#define CNTL_WAIT_AUTH                   105
#define CNTL_WAIT_ASSOC                  106
#define CNTL_WAIT_AUTH2                  107
#define CNTL_WAIT_OID_LIST_SCAN          108
#define CNTL_WAIT_OID_DISASSOC           109

//#define BSS_TABLE_EMPTY(x)               ((x).BssNr == 0)
#define CapabilityInfoGen(Ess,Ibss,Cfp,CfpReq,Priv) ((Ess) ? 0x0001 : 0x0000) | ((Ibss) ? 0x0002 : 0x0000) | ((Cfp) ? 0x0004 : 0x0000) | ((CfpReq) ? 0x0008 : 0x0000) | ((Priv) ? 0x0010: 0x0000)

#define MAC_ADDR_IS_GROUP(Addr)       ((((Addr).Octet[0]) & 0x01) != 0)
#define MAC_ADDR_HASH(Addr)           ((Addr).Octet[0] ^ (Addr).Octet[1] ^ (Addr).Octet[2] ^ (Addr).Octet[3] ^ (Addr).Octet[4] ^ (Addr).Octet[5])
#define MAC_ADDR_HASH_INDEX(Addr)     (MAC_ADDR_HASH(Addr) % HASH_TABLE_SIZE)
#define MAC_ADDR_EQUAL(pAddr1,pAddr2) (memcmp((pAddr1), (pAddr2), ETH_ALEN) == 0)
#define COPY_MAC_ADDR(Addr1, Addr2)   memcpy((Addr1), (Addr2), ETH_ALEN)
//#define MAKE_BROADCAST_ADDR(Addr)     memset(&Addr, 0xff, ETH_ALEN)

// LED Control
// assoiation ON. one LED ON. another blinking when TX, OFF when idle
#define ASIC_LED_ACT_ON(pAdapter)         RTMP_IO_WRITE32(pAdapter, LEDCSR, 0x0003461E)
// no association, both LED off
#define ASIC_LED_ACT_OFF(pAdapter)        RTMP_IO_WRITE32(pAdapter, LEDCSR, 0x0000461E)
//#define ASIC_LED_LINK_UP(pAdapter)        RTMP_IO_WRITE32(pAdapter, LEDCSR, 0x00011E46)
//#define ASIC_LED_LINK_DOWN(pAdapter)      RTMP_IO_WRITE32(pAdapter, LEDCSR, 0x00001E46)

#define CAP_IS_ESS_ON(x)                     (((x) & 0x0001) != 0)
#define CAP_IS_IBSS_ON(x)                    (((x) & 0x0002) != 0)
#define CAP_IS_CF_POLLABLE_ON(x)             (((x) & 0x0004) != 0)
#define CAP_IS_CF_POLL_REQ_ON(x)             (((x) & 0x0008) != 0)
#define CAP_IS_PRIVACY_ON(x)                 (((x) & 0x0010) != 0)

// 802.11G capability features
#define CAP_IS_SHORT_PREAMBLE_ON(x)          (((x) & 0x0020) != 0)
#define CAP_IS_PBCC_ON(x)                    (((x) & 0x0040) != 0)
#define CAP_IS_AGILITY_ON(x)                 (((x) & 0x0080) != 0)
#define CAP_IS_EXT_RATE_PBCC_ON(x)           (((x) & 0x0100) != 0)
//#define CAP_IS_CCK_OFDM_ON(x)                (((x) & 0x0200) != 0)
#define CAP_IS_QOS(x)                        (((x) & 0x0200) != 0)  // defined in 802.11e d4.3
#define CAP_IS_SHORT_SLOT_TIME(x)            (((x) & 0x0400) != 0)
#define CAP_IS_APSD(x)                       (((x) & 0x0800) != 0)  // defined in 802.11e d4.3
#define CAP_IS_Q_ACK(x)                      (((x) & 0x1000) != 0)  // defined in 802.11e d4.3
#define CAP_IS_DSSS_OFDM(x)                  (((x) & 0x2000) != 0)
#define CAP_IS_BLOCK_ACK(x)                  (((x) & 0x4000) != 0)  // defined in 802.11e d4.3

#define CAP_GENERATE(ess,ibss,cfp,cfpreq,priv,prea) ((ess) ? 0x0001 : 0x0000) | ((ibss) ? 0x0002 : 0x0000) | ((cfp) ? 0x0004 : 0x0000) | ((cfpreq) ? 0x0008 : 0x0000) | ((priv) ? 0x0010 : 0x0000) | ((prea) ? 0x0020 : 0x0000)

#define ERP_IS_NON_ERP_PRESENT(x)            (((x) & 0x01) != 0)    // define in 802.11g
#define ERP_IS_USE_PROTECTION(x)             (((x) & 0x02) != 0)    // define in 802.11g
#define ERP_IS_USE_BARKER_PREAMBLE(x)        (((x) & 0x04) != 0)    // define in 802.11g

#define TX_FER_TOO_HIGH(TxFER)          ((TxFER) > 15)   // consider rate down if FER>15%
#define TX_FER_VERY_LOW(TxFER)          ((TxFER) < 7)    // consider rate up if FER<7%
#define FAIR_FER                        10               // any value between TOO_HIGH and VERY_LOW
#define DRS_TX_QUALITY_WORST_BOUND      3 
#define DRS_PENALTY                     8

// Ralink timer control block
typedef struct	_RALINK_TIMER_STRUCT	{
	struct timer_list	TimerObj;		// Ndis Timer object
	ULONG				TimerValue;		// Timer value in milliseconds
	BOOLEAN				State;			// True if timer cancelled
	BOOLEAN				Repeat;			// True if periodic timer
}	RALINK_TIMER_STRUCT, *PRALINK_TIMER_STRUCT;

// Mac Address data structure
typedef struct PACKED _MACADDR {
    UCHAR     Octet[ETH_ALEN];
} MACADDR, *PMACADDR;

// Mac Frame Header
typedef struct PACKED _MACHDR {
    // 2-byte Frame Control. NOTE: bit field assigned from LSB first
#ifdef BIG_ENDIAN
    USHORT    Order:1;
    USHORT    Wep:1;
    USHORT    MoreData:1;
    USHORT    PwrMgmt:1;
    USHORT    Retry:1;
    USHORT    MoreFrag:1;
    USHORT    Frds:1;
    USHORT    Tods:1;
    USHORT    SubType:4;
    USHORT    Type:2;
    USHORT    Ver:2;
#else
    USHORT    Ver:2;
    USHORT    Type:2;
    USHORT    SubType:4;
    USHORT    Tods:1;
    USHORT    Frds:1;
    USHORT    MoreFrag:1;
    USHORT    Retry:1;
    USHORT    PwrMgmt:1;
    USHORT    MoreData:1;
    USHORT    Wep:1;
    USHORT    Order:1;
#endif

    USHORT    Duration;
    MACADDR   Addr1;
    MACADDR   Addr2;
    MACADDR   Addr3;

#ifdef BIG_ENDIAN
    USHORT    Seq:12;
    USHORT    Frag:4;
#else
    USHORT    Frag:4;
    USHORT    Seq:12;
#endif
} MACHDR, *PMACHDR;

typedef struct PACKED _MACFRAME {
    MACHDR    Hdr;
    CHAR      Octet[1];
} MACFRAME, *PMACFRAME;

typedef struct PACKED _PSPOLL_FRAME {
    USHORT    Ver:2;
    USHORT    Type:2;
    USHORT    SubType:4;
    USHORT    Tods:1;
    USHORT    Frds:1;
    USHORT    MoreFrag:1;
    USHORT    Retry:1;
    USHORT    PwrMgmt:1;
    USHORT    MoreData:1;
    USHORT    Wep:1;
    USHORT    Order:1;

    USHORT    Aid;
    MACADDR   Bssid;
    MACADDR   Ta;
} PSPOLL_FRAME;

//
// Contention-free parameter (without ID and Length)
//
typedef struct PACKED _CF_PARM {
    UCHAR        CfpCount;
    UCHAR        CfpPeriod;
    USHORT       CfpMaxDuration;
    USHORT       CfpDurRemaining;
} CF_PARM, *PCF_PARM;


typedef struct PACKED _BSS_ENTRY{
    MACADDR Bssid;
    UCHAR   Channel;
    UCHAR   BssType;
    USHORT  AtimWin;
    USHORT  BeaconPeriod;

    UCHAR   Rates[MAX_LEN_OF_SUPPORTED_RATES];
    UCHAR   RatesLen;
    BOOLEAN ExtendedRateIeExist; // records if this AP use EXTENDED_SUPPORTED_RATES IE
    UCHAR   Rssi;
    UCHAR   Privacy;            // Indicate security function ON/OFF. Don't mess up with auth mode.
    UCHAR   Hidden;

    USHORT  DtimPeriod;
    USHORT  CapabilityInfo;

    USHORT  CfpCount;
    USHORT  CfpPeriod;
    USHORT  CfpMaxDuration;
    USHORT  CfpDurRemaining;
    UCHAR   SsidLen;
    CHAR    Ssid[MAX_LEN_OF_SSID];
    
    ULONG   LastBeaconRxTime; // OS's timestamp

    // New for microsoft WPA support
    NDIS_802_11_FIXED_IEs   FixIEs;
    NDIS_802_11_WEP_STATUS  WepStatus;
    UCHAR                   VarIELen;               // Length of next VIE include EID & Length
    UCHAR                   VarIEs[MAX_VIE_LEN];
} BSS_ENTRY, *PBSS_ENTRY;

typedef struct {
    UCHAR           BssNr;
    BSS_ENTRY       BssEntry[MAX_LEN_OF_BSS_TABLE];
} BSS_TABLE, *PBSS_TABLE;

typedef struct PACKED _MLME_QUEUE_ELEM {
    ULONG             Machine;
    ULONG             MsgType;
    ULONG             MsgLen;
    LARGE_INTEGER     TimeStamp;
    UCHAR             Msg[MAX_LEN_OF_MLME_BUFFER];
    UCHAR             Rssi;
    UCHAR             Channel;
    BOOLEAN           Occupied;
} MLME_QUEUE_ELEM, *PMLME_QUEUE_ELEM;

typedef struct PACKED _MLME_QUEUE {
    ULONG             Num;
    ULONG             Head;
    ULONG             Tail;
    spinlock_t   Lock;
    MLME_QUEUE_ELEM  Entry[MAX_LEN_OF_MLME_QUEUE];
} MLME_QUEUE, *PMLME_QUEUE;

typedef VOID (*STATE_MACHINE_FUNC)(VOID *Adaptor, MLME_QUEUE_ELEM *Elem);

typedef struct PACKED _STATE_MACHINE {
    ULONG                           Base;
    ULONG                           NrState;
    ULONG                           NrMsg;
    ULONG                           CurrState;
    STATE_MACHINE_FUNC             *TransFunc;
} STATE_MACHINE, *PSTATE_MACHINE;

// CNTL State Machine Aux data structure
typedef struct _CNTL_AUX {
    UCHAR               Ssid[MAX_LEN_OF_SSID];
    UCHAR               SsidLen;
    MACADDR             Bssid;
    BSS_TABLE           SsidBssTab;     // AP list for the same SSID
    BSS_TABLE           RoamTab;        // AP list eligible for roaming
    ULONG               BssIdx;
    ULONG               RoamIdx;
    BOOLEAN             CurrReqIsFromNdis; // TRUE - then we should call NdisMSetInformationComplete()
                                           // FALSE - req is from driver itself. 
                                           // no NdisMSetInformationComplete() is required
} CNTL_AUX, *PCNTL_AUX;

// ASSOC State Machine Aux data structure
typedef struct _ASSOC_AUX {
    MACADDR             Addr;
    USHORT              CapabilityInfo;
    USHORT              ListenIntv;
    CHAR                Ssid[MAX_LEN_OF_SSID];
    UCHAR               SsidLen;    
    RALINK_TIMER_STRUCT AssocTimer, ReassocTimer, DisassocTimer;
} ASSOC_AUX, *PASSOC_AUX;

// AUTH State Machine Aux data structure
typedef struct _AUTH_AUX {
    MACADDR             Addr;
    USHORT              Alg;
    RALINK_TIMER_STRUCT AuthTimer;
} AUTH_AUX, *PAUTH_AUX;

// AUTH-RSP State Machine Aux data structure
typedef struct PACKED _AUTH_RSP_AUX {
    MACADDR             Addr;
    USHORT              Alg;
    CHAR                Challenge[CIPHER_TEXT_LEN];
    RALINK_TIMER_STRUCT AuthRspTimer;
} AUTH_RSP_AUX, *PAUTH_RSP_AUX;

// SYNC State Machine Aux data structure
typedef struct _SYNC_AUX {
    MACADDR             Addr;
    MACADDR             Bssid;
    UCHAR               BssType;
    UCHAR               SsidLen;
    CHAR                Ssid[MAX_LEN_OF_SSID];
    UCHAR               ScanType;
    UCHAR               Channel;
    RALINK_TIMER_STRUCT BeaconTimer, ScanTimer;
} SYNC_AUX;

 // assoc struct is equal to reassoc
typedef struct PACKED _MLME_ASSOC_REQ_STRUCT{
    MACADDR   Addr;
    USHORT    CapabilityInfo;
    USHORT    ListenIntv;
    ULONG     Timeout;
} MLME_ASSOC_REQ_STRUCT, *PMLME_ASSOC_REQ_STRUCT, MLME_REASSOC_REQ_STRUCT, *PMLME_REASSOC_REQ_STRUCT;

typedef struct PACKED _MLME_DISASSOC_REQ_STRUCT{
    MACADDR   Addr;
    USHORT    Reason;
} MLME_DISASSOC_REQ_STRUCT, *PMLME_DISASSOC_REQ_STRUCT;

typedef struct PACKED _MLME_AUTH_REQ_STRUCT {
    MACADDR      Addr;
    USHORT       Alg;
    ULONG        Timeout;
} MLME_AUTH_REQ_STRUCT, *PMLME_AUTH_REQ_STRUCT;

typedef struct PACKED _MLME_DEAUTH_REQ_STRUCT {
    MACADDR      Addr;
    USHORT       Reason;
} MLME_DEAUTH_REQ_STRUCT, *PMLME_DEAUTH_REQ_STRUCT;

//typedef struct _MLME_AUTH_IND_STRUCT {
//    MACADDR      Addr;
//    USHORT       Alg;
//} MLME_AUTH_IND_STRUCT, *PMLME_AUTH_IND_STRUCT;

//typedef struct _CLS2ERR_STRUCT {
//    MACADDR      Addr;
//} CLS2ERR_STRUCT, *PCLS2ERR_STRUCT;

typedef struct PACKED _MLME_JOIN_REQ_STRUCT{
    ULONG      BssIdx;
} MLME_JOIN_REQ_STRUCT;

typedef struct PACKED _MLME_SCAN_REQ_STRUCT {
    MACADDR    Bssid;
    UCHAR      BssType;
    UCHAR      ScanType;
    UCHAR      SsidLen;
    CHAR       Ssid[MAX_LEN_OF_SSID];
} MLME_SCAN_REQ_STRUCT, *PMLME_SCAN_REQ_STRUCT;

typedef struct PACKED _MLME_START_REQ_STRUCT {
    CHAR        Ssid[MAX_LEN_OF_SSID];
    UCHAR       SsidLen;
} MLME_START_REQ_STRUCT, *PMLME_START_REQ_STRUCT;

typedef struct PACKED _ARC4_CONTEXT {
    UCHAR x, y, State[256], Key[16];     // 128 bits key
} ARC4_CONTEXT, *PARC4_CONTEXT;

typedef struct PACKED _BEACON_EID_STRUCT {
    UCHAR   Eid;
    UCHAR   Len;
    CHAR   Octet[1];
} BEACON_EID_STRUCT,*PBEACON_EID_STRUCT;

// New for WPA cipher suite 
typedef struct PACKED _RSN_EID_STRUCT {
    UCHAR   Eid;
    UCHAR   Length;
    UCHAR   Oui[4];
    USHORT  Version;
    UCHAR   Multicast[4];
    USHORT  Count;
    struct  {
        UCHAR   Oui[4];
    }   Unicast[1];
}   RSN_EID_STRUCT, *PRSN_EID_STRUCT;

extern UCHAR  RateIdToMbps[];
extern USHORT RateIdTo500Kbps[];

#endif  // MLME_H__
