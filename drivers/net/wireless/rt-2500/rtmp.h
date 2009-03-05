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
 *      Module Name: rt_config.h
 *              
 *      Abstract: Central header file for all includes
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      RoryC           21st Dec 02     Initial code   
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW (rt2400)  8th  Dec 04     Promisc mode support
 *      Flavio (rt2400) 8th  Dec 04     Elegant irqreturn_t handling
 *      RobinC          10th Dec 04     RFMON Support
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0 
 *      MarkW (rt2400)  15th Dec 04     Spinlock fix 
 *      Ivo (rt2400)    15th Dec 04     Debug level switching
 *      GregorG         29th Mar 05     Big endian fixes
 ***************************************************************************/ 

#ifndef __RTMP_H__
#define __RTMP_H__

#include "mlme.h"
#include "oid.h"
#include "wpa.h"

#ifndef IRQ_HANDLED
/* For 2.6.x compatability */
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif

#ifndef pci_name
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
#define pci_name(__pPci_Dev)	(__pPci_Dev)->dev.bus_id
#else /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) */
#define pci_name(__pPci_Dev)	(__pPci_Dev)->slot_name
#endif /*(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) */
#endif /* pci_name */

//
// Defines the state of the LAN media
//
typedef enum _NDIS_MEDIA_STATE
{
    NdisMediaStateConnected,
    NdisMediaStateDisconnected
} NDIS_MEDIA_STATE, *PNDIS_MEDIA_STATE;

//
//  MACRO for debugging information
//
extern int    debug;
#ifdef RT2500_DBG
#define DBGPRINT(Level, fmt, args...) 					\
	if(debug){printk(Level DRV_NAME ": " fmt, ## args);}
#else
#define DBGPRINT(Level, fmt, args...)					\
	while(0){}
#endif

//
//  spin_lock enhanced for Nested spin lock
//

extern unsigned long IrqFlags;

//  Assert MACRO to make sure program running
//
#undef  ASSERT
#define ASSERT(x)                                                               \
{                                                                               \
    if (!(x))                                                         \
    {                                                                           \
        printk(KERN_WARNING __FILE__ ":%d assert " #x "failed\n", __LINE__);    \
    }                                                                           \
}

//
//  Macros for flag and ref count operations
//
#define RTMP_SET_FLAG(_M, _F)       ((_M)->Flags |= (_F))
#define RTMP_CLEAR_FLAG(_M, _F)     ((_M)->Flags &= ~(_F))
#define RTMP_CLEAR_FLAGS(_M)        ((_M)->Flags = 0)
#define RTMP_TEST_FLAG(_M, _F)      (((_M)->Flags & (_F)) != 0)
#define RTMP_TEST_FLAGS(_M, _F)     (((_M)->Flags & (_F)) == (_F))

#define RTMP_INC_RCV_REF(_A)        ((_A)->RcvRefCount++)
#define RTMP_DEC_RCV_REF(_A)        ((_A)->RcvRefCount--)
#define RTMP_GET_RCV_REF(_A)        ((_A)->RcvRefCount)

#define RTMP_INC_SEND_REF(_A)       ((_A)->SendRefCount++)
#define RTMP_DEC_SEND_REF(_A)       ((_A)->SendRefCount--)
#define RTMP_GET_SEND_REF(_A)       ((_A)->SendRefCount)

#define NdisEqualMemory(Source1, Source2, Length)   RTMPEqualMemory(Source1, Source2, Length)

//
// MACRO for 32-bit PCI register read / write
//
// Usage : RTMP_IO_READ32(
//              PRTMP_ADAPTER pAdapter,
//              ULONG Register_Offset,
//              PULONG  pValue)
//
//         RTMP_IO_WRITE32(
//              PRTMP_ADAPTER pAdapter,
//              ULONG Register_Offset,
//              ULONG Value)
//
#ifdef RTMP_EMBEDDED
#define RTMP_IO_READ32(_A, _R, _pV)     (*_pV = PCIMemRead32(__mem_pci(_A->CSRBaseAddress+_R)))
#define RTMP_IO_WRITE32(_A, _R, _V)     (PCIMemWrite32(__mem_pci(_A->CSRBaseAddress+_R),_V))
#else
#define RTMP_IO_READ32(_A, _R, _pV)	(*_pV = readl( (void*) (_A->CSRBaseAddress + _R) ) )
#define RTMP_IO_WRITE32(_A, _R, _V)	(writel(_V, (void*) (_A->CSRBaseAddress + _R) ) )
#endif

//
// BBP & RF are using indirect access. Before write any value into it.
// We have to make sure there is no outstanding command pending via checking busy bit.
//
#define MAX_BUSY_COUNT  10          // Nunber of retry before failing access BBP & RF indirect register
//
#define RTMP_BBP_IO_WRITE32(_A, _V)                 \
{                                                   \
    BBPCSR_STRUC    Value;                          \
    ULONG           BusyCnt = 0;                    \
    do {                                            \
        RTMP_IO_READ32(_A, BBPCSR, &Value.word);    \
        if (Value.field.Busy == IDLE)               \
            break;                                  \
        BusyCnt++;                                  \
    }   while (BusyCnt < MAX_BUSY_COUNT);           \
    if (BusyCnt < MAX_BUSY_COUNT)				    \
	{											    \
		RTMP_IO_WRITE32(_A, BBPCSR, _V);		    \
	}                                               \
}
//
#define RTMP_RF_IO_WRITE32(_A, _V)                  \
{                                                   \
    RFCSR_STRUC     Value;                          \
    ULONG           BusyCnt = 0;                    \
    do {                                            \
        RTMP_IO_READ32(_A, RFCSR, &(Value.word));   \
        if (Value.field.Busy == IDLE)               \
            break;                                  \
        BusyCnt++;                                  \
    }   while (BusyCnt < MAX_BUSY_COUNT);           \
    if (BusyCnt < MAX_BUSY_COUNT)				    \
	{											    \
		RTMP_IO_WRITE32(_A, RFCSR, _V);			    \
	}                                               \
}
//
#define RTMP_BBP_IO_READ32(_A, _pV)             \
{                                               \
    BBPCSR_STRUC    Value;                      \
    ULONG           BusyCnt = 0;                \
    RTMP_BBP_IO_WRITE32(_A, *(_pV));            \
    do {                                        \
        RTMP_IO_READ32(_A, BBPCSR, &Value.word);\
        if (Value.field.Busy == IDLE)           \
            break;                              \
        BusyCnt++;                              \
    }   while (BusyCnt < MAX_BUSY_COUNT);       \
    if (BusyCnt == MAX_BUSY_COUNT)              \
        *(_pV) = 0xff;                          \
    else                                        \
            *(_pV) = Value.field.Value;         \
}
// Read BBP register by register's ID
#define RTMP_BBP_IO_READ32_BY_REG_ID(_A, _I, _pV)   \
{                                                   \
    BBPCSR_STRUC    BbpCsr;                         \
    BbpCsr.word = 0;                                \
    BbpCsr.field.WriteControl = 0;                  \
    BbpCsr.field.Busy = 1;                          \
    BbpCsr.field.RegNum = _I;                       \
    RTMP_BBP_IO_READ32(_A, &BbpCsr.word);           \
    *(_pV) = (UCHAR) BbpCsr.field.Value;            \
}
// Write BBP register by register's ID & value
#define RTMP_BBP_IO_WRITE32_BY_REG_ID(_A, _I, _V)   \
{                                                   \
    BBPCSR_STRUC    BbpCsr;                         \
    BbpCsr.word = 0;                                \
    BbpCsr.field.WriteControl = 1;                  \
    BbpCsr.field.Busy = 1;                          \
    BbpCsr.field.Value = _V;                        \
    BbpCsr.field.RegNum = _I;                       \
    RTMP_BBP_IO_WRITE32(_A, BbpCsr.word);           \
    (_A)->PortCfg.BbpWriteLatch[_I] = _V;           \
}


//
//  Some utility macros
//
#ifndef min
#define min(_a, _b)     (((_a) < (_b)) ? (_a) : (_b))
#endif

#ifndef max
#define max(_a, _b)     (((_a) > (_b)) ? (_a) : (_b))
#endif

#define INC_COUNTER(Val)    (Val.QuadPart++)

#define INFRA_ON(_p)                (((_p)->PortCfg.Massoc) == TRUE)    // Check Massoc
#define ADHOC_ON(_p)                (((_p)->PortCfg.Mibss) == TRUE)    // check Mibss
#define RTMP_SET_PACKET_FRAGMENTS(_p, number)   ((_p)->cb[10] = number)
#define RTMP_GET_PACKET_FRAGMENTS(_p)           ((_p)->cb[10])
#define RTMP_SET_PACKET_RTS(_p, number)         ((_p)->cb[11] = number)
#define RTMP_GET_PACKET_RTS(_p)                 ((_p)->cb[11])

#define MAKE_802_3_HEADER(_p, _pMac1, _pMac2, _pType)                               \
{                                                                                   \
    memcpy(_p, _pMac1, ETH_ALEN);                              \
    memcpy((_p + ETH_ALEN), _pMac2, ETH_ALEN);    \
    memcpy((_p + ETH_ALEN * 2), _pType, LENGTH_802_3_TYPE);    \
}

// if pData has no LLC/SNAP (neither RFC1042 nor Bridge tunnel), keep it that way.
// else if the received frame is LLC/SNAP-encaped IPX or APPLETALK, preserve the LLC/SNAP field in the result Ethernet frame
// else remove the LLC/SNAP field from the result Ethernet frame
// Note:
//     _pData & _DataSize may be altered (remove 8-byte LLC/SNAP) by this MACRO
#define CONVERT_TO_802_3(_p8023hdr, _pDA, _pSA, _pData, _DataSize)      \
{                                                                       \
    char LLC_Len[2];                                                    \
                                                                        \
	if ((!RTMPEqualMemory(SNAP_802_1H, _pData, 6)) &&                   \
	    (!RTMPEqualMemory(SNAP_BRIDGE_TUNNEL, _pData, 6)))              \
	{                                                                   \
		LLC_Len[0] = (UCHAR)(_DataSize / 256);                          \
		LLC_Len[1] = (UCHAR)(_DataSize % 256);                          \
		MAKE_802_3_HEADER(_p8023hdr, _pDA, _pSA, LLC_Len);              \
	}                                                                   \
	else                                                                \
	{                                                                   \
	    PUCHAR pProto = _pData + 6;                                     \
					                                                    \
		if ((RTMPEqualMemory(IPX, pProto, 2) || RTMPEqualMemory(APPLE_TALK, pProto, 2)) &&  \
		    RTMPEqualMemory(SNAP_802_1H, _pData, 6))                    \
		{                                                               \
			LLC_Len[0] = (UCHAR)(_DataSize / 256);                      \
			LLC_Len[1] = (UCHAR)(_DataSize % 256);                      \
			MAKE_802_3_HEADER(_p8023hdr, _pDA, _pSA, LLC_Len);          \
		}                                                               \
		else                                                            \
		{                                                               \
			MAKE_802_3_HEADER(_p8023hdr, _pDA, _pSA, pProto);           \
			_DataSize -= LENGTH_802_1_H;                                \
			_pData += LENGTH_802_1_H;                                   \
		}                                                               \
	}                                                                   \
}

//
// Register set pair for initialzation register set definition
//
typedef struct  _RTMP_REG_PAIR
{
    ULONG   Register;
    ULONG   Value;
}   RTMP_REG_PAIR, *PRTMP_REG_PAIR;

//
// Register set pair for initialzation register set definition
//
typedef struct  _RTMP_RF_REGS
{
    UCHAR   Channel;
    ULONG   R1;
    ULONG   R2;
    ULONG   R3;
    ULONG   R4;
}   RTMP_RF_REGS, *PRTMP_RF_REGS;

//
//  Statistic counter structure
//
typedef struct _COUNTER_802_3
{
    // General Stats
    ULONG       GoodTransmits;
    ULONG       GoodReceives;
    ULONG       TxErrors;
    ULONG       RxErrors;
    ULONG       RxNoBuffer;

    // Ethernet Stats
    ULONG       RcvAlignmentErrors;
    ULONG       OneCollision;
    ULONG       MoreCollisions;

}   COUNTER_802_3, *PCOUNTER_802_3;

typedef struct _COUNTER_802_11 {
    ULONG           Length;
    LARGE_INTEGER   TransmittedFragmentCount;
    LARGE_INTEGER   MulticastTransmittedFrameCount;
    LARGE_INTEGER   FailedCount;
    LARGE_INTEGER   RetryCount;
    LARGE_INTEGER   MultipleRetryCount;
    LARGE_INTEGER   RTSSuccessCount;
    LARGE_INTEGER   RTSFailureCount;
    LARGE_INTEGER   ACKFailureCount;
    LARGE_INTEGER   FrameDuplicateCount;
    LARGE_INTEGER   ReceivedFragmentCount;
    LARGE_INTEGER   MulticastReceivedFrameCount;
    LARGE_INTEGER   FCSErrorCount;
} COUNTER_802_11, *PCOUNTER_802_11;

typedef struct _COUNTER_RALINK {
    ULONG           TransmittedByteCount;   // both successful and failure, used to calculate TX throughput
    ULONG           ReceivedByteCount;      // both CRC okay and CRC error, used to calculate RX throughput
    ULONG           BeenDisassociatedCount;
    ULONG           BadCQIAutoRecoveryCount;
    ULONG           PoorCQIRoamingCount;
    ULONG           MgmtRingFullCount;
    ULONG           RxCount;
    ULONG           DecryptCount;
    ULONG           RxRingErrCount;
    ULONG           EncryptCount;
    ULONG           KickTxCount;
    ULONG           TxRingErrCount; 
    LARGE_INTEGER	RealFcsErrCount;
} COUNTER_RALINK, *PCOUNTER_RALINK;

typedef struct _COUNTER_DRS {
    // to record the each TX rate's quality. 0 is best, the bigger the worse.
    USHORT          TxQuality[MAX_LEN_OF_SUPPORTED_RATES];
    UCHAR           PER[MAX_LEN_OF_SUPPORTED_RATES];
    USHORT          OneSecTxOkCount;
    USHORT          OneSecTxRetryOkCount;
    USHORT          OneSecTxFailCount;
    UCHAR           TxRateUpPenalty;      // extra # of second penalty due to last unstable condition
    ULONG           CurrTxRateStableTime; // # of second in current TX rate
    BOOLEAN         fNoisyEnvironment;
    UCHAR           LastSecTxRateChangeAction; // 0: no change, 1:rate UP, 2:rate down
} COUNTER_DRS, *PCOUNTER_DRS;

//
//  Arcfour Structure Added by PaulWu
//
typedef struct PACKED _ARCFOUR
{
    UINT            X;
    UINT            Y;
    UCHAR           STATE[256];
}   ARCFOURCONTEXT, *PARCFOURCONTEXT;

// Shared key data structure
typedef struct  _WEP_KEY {
    UCHAR   KeyLen;                     // Key length for each key, 0: entry is invalid
    UCHAR   Key[MAX_LEN_OF_KEY];        // right now we implement 4 keys, 128 bits max
}   WEP_KEY, *PWEP_KEY;

// Shared key data structure
typedef struct  _WPA_KEY {
    UCHAR   KeyLen;             // Key length for each key, 0: entry is invalid
    UCHAR   Key[16];            // right now we implement 4 keys, 128 bits max
    UCHAR   RxMic[8];
    UCHAR   TxMic[8];
    NDIS_802_11_MAC_ADDRESS BssId;  // For pairwise key only
    UCHAR   TxTsc[6];           // 48bit TSC value
    UCHAR   RxTsc[6];           // 48bit TSC value
    UCHAR   Type;               // Indicate Pairwise / Group
}   WPA_KEY, *PWPA_KEY;

#if 0
typedef	struct	_IV_CONTROL_
{
	union
	{
		struct 
		{
			UCHAR		rc0;
			UCHAR		rc1;
			UCHAR		rc2;

			union
			{
				struct
				{
#ifdef BIG_ENDIAN
					UCHAR	KeyID:2;
					UCHAR	ExtIV:1;
					UCHAR	Rsvd:5;
#else
					UCHAR	Rsvd:5;
					UCHAR	ExtIV:1;
					UCHAR	KeyID:2;
#endif
				}	field;
				UCHAR		Byte;
			}	CONTROL;
		}	field;
		
		ULONG	word;
	}	IV16;
	
	ULONG	IV32;
}	TKIP_IV, *PTKIP_IV;
#endif

typedef	struct	_IV_CONTROL_
{
		union
		{
			struct
			{
#ifdef BIG_ENDIAN
				ULONG	KeyID:2;
				ULONG	ExtIV:1;
				ULONG	Rsvd:5;
				ULONG	rc2:8;
				ULONG	rc1:8;
				ULONG	rc0:8;
#else
				ULONG	rc0:8;
				ULONG	rc1:8;
				ULONG	rc2:8;
				ULONG	Rsvd:5;
				ULONG	ExtIV:1;
				ULONG	KeyID:2;
#endif
			}field;
			ULONG	word;
		}IV16;

	ULONG	IV32;
}	TKIP_IV, *PTKIP_IV;

// configuration to be used when this STA starts a new ADHOC network
typedef struct _IBSS_CONFIG {
    USHORT    BeaconPeriod;
    USHORT    AtimWin;
    UCHAR     Channel;
    UCHAR     SupportedRates[MAX_LEN_OF_SUPPORTED_RATES];    // Supported rates
    UCHAR     SupportedRatesLen;
}   IBSS_CONFIG, *PIBSS_CONFIG;

typedef struct _LED_CONTROL {
    BOOLEAN             fOdd;
    BOOLEAN             fRxActivity;
    RALINK_TIMER_STRUCT BlinkTimer; // 50 ms periodic timer
    ULONG               LastLedCsr;
}   LED_CONTROL;

typedef struct _BBP_TUNING_STRUCT {
    BOOLEAN     Enable;
    UCHAR       FalseCcaCountUpperBound;  // 100 per sec
    UCHAR       FalseCcaCountLowerBound;  // 10 per sec
    UCHAR       R17LowerBound;            // specified in E2PROM
    UCHAR       R17UpperBound;            // 0x68 according to David Tung
    UCHAR       CurrentR17Value;
} BBP_TUNING, *PBBP_TUNING;

typedef struct _SOFT_RX_ANT_DIVERSITY_STRUCT {
    BOOLEAN   PrimaryInUsed;
    BOOLEAN   FirstPktArrivedWhenEvaluate;     
    UCHAR     PrimaryRxAnt;     // 0:Ant-A, 1:Ant-B
    UCHAR     SecondaryRxAnt;   // 0:Ant-A, 1:Ant-B
    UCHAR     CurrentRxAnt;     // 0:Ant-A, 1:Ant-B
    USHORT    AvgRssi[2];       // AvgRssi[0]:Ant-A, AvgRssi[1]:Ant-B
    ULONG     RcvPktNumWhenEvaluate;
    RALINK_TIMER_STRUCT    RxAntDiversityTimer;
} SOFT_RX_ANT_DIVERSITY, *PSOFT_RX_ANT_DIVERSITY;

typedef struct _STA_WITH_ETHER_BRIDGE_STRUCT {
    BOOLEAN   Enable;
    MACADDR   EtherMacAddr;
} STA_WITH_ETHER_BRIDGE, *PSTA_WITH_ETHER_BRIDGE;

// PortConfig
typedef struct _PORT_CONFIG {

    // MIB:ieee802dot11.dot11smt(1).dot11StationConfigTable(1)
    USHORT    CapabilityInfo;
    USHORT    Psm;                  // power management mode   (PWR_ACTIVE|PWR_SAVE)
    USHORT    BeaconPeriod;         // in units of TU

    USHORT    CfpMaxDuration;
    USHORT    CfpDurRemain;
    USHORT    CfpCount;
    USHORT    CfpPeriod;

    USHORT    DisassocReason;
    MACADDR   DisassocSta;
    USHORT    DeauthReason;
    MACADDR   DeauthSta;
    USHORT    AuthFailReason;
    MACADDR   AuthFailSta;

    NDIS_802_11_AUTHENTICATION_MODE     AuthMode;   // This should match to whatever microsoft defined
    NDIS_802_11_WEP_STATUS              WepStatus;
    
    // MIB:ieee802dot11.dot11smt(1).dot11WEPDefaultKeysTable(3)
    WEP_KEY   SharedKey[SHARE_KEY_NO];      // Keep for backward compatiable
    WPA_KEY   PairwiseKey[PAIRWISE_KEY_NO];
    WPA_KEY   GroupKey[GROUP_KEY_NO];
    WPA_KEY   PskKey;                   // WPA PSK mode PMK
    UCHAR     PTK[64];

    // WPA 802.1x port control, WPA_802_1X_PORT_SECURED, WPA_802_1X_PORT_NOT_SECURED
    UCHAR     PortSecured;

    // For WPA countermeasures
    ULONG       LastMicErrorTime;   // record last MIC error time
    ULONG       MicErrCnt;          // Should be 0, 1, 2, then reset to zero (after disassoiciation).
    BOOLEAN     bBlockAssoc;        // Block associate attempt for 60 seconds after counter measure occurred.
    // For WPA-PSK supplicant state
    WPA_STATE   WpaState;           // Default is SS_NOTUSE and handled by microsoft 802.1x
    UCHAR       ReplayCounter[8];
    UCHAR       ANonce[32];         // ANonce for WPA-PSK from aurhenticator
    UCHAR       SNonce[32];         // SNonce for WPA-PSK

    // MIB:ieee802dot11.dot11smt(1).dot11WEPKeyMappingsTable(4)
    // not implemented yet

    // MIB:ieee802dot11.dot11smt(1).dot11PrivacyTable(5)
    UCHAR     DefaultKeyId;
    NDIS_802_11_PRIVACY_FILTER  PrivacyFilter;      // PrivacyFilter enum for 802.1X

    // MIB:ieee802dot11.dot11mac(2).dot11OperationTable(1)
    USHORT    RtsThreshold;       // in units of BYTE
    USHORT    FragmentThreshold;
    BOOLEAN   bFragmentZeroDisable;     // Microsoft use 0 as disable 
    
    // MIB:ieee802dot11.dot11phy(4).dot11PhyAntennaTable(2)
    UCHAR     CurrentTxAntenna;
    UCHAR     CurrentRxAntenna;
    UCHAR     NumberOfAntenna;

    // MIB:ieee802dot11.dot11phy(4).dot11PhyTxPowerTable(3)
    UCHAR     CurrentTxPowerLevelIndex; //default&other value=MaxPower,1=MinPower,2=1*MaxPower/4,3=2*MaxPower/4,4=3*MaxPower/4,
    UCHAR     TxPower;
    UCHAR     TxRate;       // RATE_1, RATE_2, RATE_5_5, RATE_11, ...
    BOOLEAN   EnableAutoRateSwitching;  // 1 - enable auto rate switching; 0 - disable
    ULONG     TxPowerPercentage;        // 0~100 %
    
    // MIB:ieee802dot11.dot11phy(4).dot11PhyDSSSTable(5)
    UCHAR     Channel;        // current (I)BSS channel used in the station
    UCHAR     CountryRegion;    // Enum of country region, 0:FCC, 1:IC, 2:ETSI, 3:SPAIN, 4:France, 5:MKK, 6:MKK1, 7:Israel
    
    // MIB:ieee802dot11.dot11phy(4).dot11AntennasListTable(8)
    BOOLEAN AntennaSupportTx;
    BOOLEAN AntennaSupportRx;
    BOOLEAN AntennaSupportDiversityRx;

    // Use user changed MAC
    BOOLEAN bLocalAdminMAC;                           
    
    // MIB:ieee802dot11.dot11phy(4).dot11SupportedDataRatesTxTable(9)
    // MIB:ieee802dot11.dot11phy(4).dot11SupportedDataRatesRxTable(10)
    UCHAR     SupportedRates[MAX_LEN_OF_SUPPORTED_RATES];    // Supported rates
    UCHAR     SupportedRatesLen;
    UCHAR     ExpectedACKRate[MAX_LEN_OF_SUPPORTED_RATES];
    // Copy supported rate from desired AP's beacon. We are trying to match
	// AP's supported and extended rate settings.
	UCHAR		SupRate[MAX_LEN_OF_SUPPORTED_RATES];
	UCHAR		ExtRate[MAX_LEN_OF_SUPPORTED_RATES];
	UCHAR		SupRateLen;
	UCHAR		ExtRateLen;

    //
    // other parameters not defined in standard MIB
    //
    UCHAR     DesiredRates[MAX_LEN_OF_SUPPORTED_RATES];      // OID_802_11_DESIRED_RATES
    UCHAR     MaxDesiredRate;
    USHORT    RecvDtim;
    MACADDR   Bssid;
    MACADDR   Broadcast;            // FF:FF:FF:FF:FF:FF
    USHORT    Pss;                  // current power saving status (PWR_SAVE|PWR_ACTIVE)
    UCHAR     RssiTrigger;
    UCHAR     RssiTriggerMode;      // RSSI_TRIGGERED_UPON_BELOW_THRESHOLD or RSSI_TRIGGERED_UPON_EXCCEED_THRESHOLD
    UCHAR     LastRssi;             // last received BEACON's RSSI
    SHORT     LastAvgRssi;          // last
    USHORT    AvgRssi;              // last 8 BEACON's average RSSI
    USHORT    AtimWin;              // in kusec; IBSS parameter set element
    USHORT    Aid;                  // association ID
    UCHAR     RtsRate;         // RATE_xxx
    UCHAR     MlmeRate;               // RATE_xxx, used to send MLME frames
    UCHAR     MaxTxRate;            // RATE_xxx
    USHORT    DefaultListenCount;   // default listen count;
    UCHAR     BssType;              // BSS_INFRA or BSS_INDEP

    UCHAR     SsidLen;               // the actual ssid length in used
    CHAR      Ssid[MAX_LEN_OF_SSID]; // NOT NULL-terminated

    BSS_TABLE BssTab;     // BSS Table

    // global variables mXXXX used in MAC protocol state machines
    BOOLEAN   Mibss;
    BOOLEAN   Massoc;
    BOOLEAN   Mauth;

    // PHY specification
    UCHAR     PhyMode;    // PHY_11A, PHY_11B, PHY_11BG_MIXED, PHY_ABG_MIXED
    USHORT    Dsifs;      // in units of usec

    ULONG     WindowsPowerMode;         // Power mode for AC power
    ULONG     WindowsBatteryPowerMode;  // Power mode for battery if exists
    BOOLEAN   WindowsACCAMEnable;       // Enable CAM power mode when AC on
    ULONG     PacketFilter;             // Packet filter for receiving
    BOOLEAN   AutoReconnect;            // Set to TRUE when setting OID_802_11_SSID with no matching BSSID

    ULONG     WindowsTxPreamble; // Rt802_11PreambleLong, Rt802_11PreambleShort, Rt802_11PreambleAuto

    UCHAR     ChannelTxPower[MAX_LEN_OF_CHANNELS];      // Store Tx power value for all channels.
    UCHAR	  ChannelTssiRef[MAX_LEN_OF_CHANNELS];		// Store Tssi Reference value for all channels.
	UCHAR	  ChannelTssiDelta;							// Store Tx TSSI delta increment / decrement value
	BOOLEAN   bAutoTxAgc;
    UCHAR     ChannelList[MAX_LEN_OF_CHANNELS];         // list all supported channels for site survey
    UCHAR     ChannelListNum;                           // number of channel in ChannelList[]
    BOOLEAN   bShowHiddenSSID;
    
    // configuration to be used when this STA starts a new ADHOC network
    IBSS_CONFIG IbssConfig;

    ULONG     LastBeaconRxTime;     // OS's timestamp of the last BEACON RX time
    ULONG     Last11bBeaconRxTime;  // OS's timestamp of the last 11B BEACON RX time
    ULONG     LastScanTime;     // Record last scan time for issue BSSID_SCAN_LIST
    ULONG     IgnoredScanNumber;    // Ignored BSSID_SCAN_LIST requests
    BOOLEAN   bSwRadio;         // Software controlled Radio On/Off, TRUE: On
    BOOLEAN   bHwRadio;         // Hardware controlled Radio On/Off, TRUE: On
    BOOLEAN   bRadio;           // Radio state, And of Sw & Hw radio state
    BOOLEAN   bHardwareRadio;   // Hardware controlled Radio enabled

    LED_CONTROL             LedCntl;
    UCHAR                   RfType;
    UCHAR                   LedMode;
    RALINK_TIMER_STRUCT       RfTuningTimer;
    STA_WITH_ETHER_BRIDGE               StaWithEtherBridge;
    
    // New for WPA, windows want us to to keep association information and
    // Fixed IEs from last association response
    NDIS_802_11_ASSOCIATION_INFORMATION     AssocInfo;
//  NDIS_802_11_FIXED_IEs                   FixIEs;
    UCHAR                   ReqVarIELen;                // Length of next VIE include EID & Length
    UCHAR                   ReqVarIEs[MAX_VIE_LEN];
    UCHAR                   ResVarIELen;                // Length of next VIE include EID & Length
    UCHAR                   ResVarIEs[MAX_VIE_LEN];

    // the following fields are user setting from UI
    ULONG     EnableTurboRate;      // 0: disable, 1: enable 72/100 Mbps whenever applicable
    ULONG     EnableTxBurst;        // 0: disable, 1: enable TX PACKET BURST
    ULONG     UseBGProtection;      // 0: auto, 1: always use, 2: always not use
    ULONG     UseShortSlotTime;     // 0: disable, 1: enable 9us short slot time if AP supports
    ULONG     AdhocMode; // 0:WIFI mode (11b rates only), 1:allow OFDM rates

    // this flag is the result calculated from UI settings and AP's ERP/Capability
    ULONG     BGProtectionInUsed;   // 0: not in-used, 1: in-used
    ULONG     ShortSlotInUsed;      // 0: not in-used, 1: in-used
    USHORT    TxPreambleInUsed;     // Rt802_11PreambleLong, Rt802_11PreambleShort
    
    // PCI clock adjustment round
    UCHAR       PciAdjustmentRound;

    // latch th latest RF programming value here since RF IC doesn't support READ operation
    RTMP_RF_REGS    LatchRfRegs;

    BOOLEAN                       BbpTuningEnable;
    UCHAR                         VgcLowerBound;
    RT_802_11_RX_AGC_VGC_TUNING   BbpTuning;

    UCHAR                         LastR17Value;

    // New for RSSI to dbm veriable
    UCHAR						  RssiToDbm;	// EEPROM 0x7c


    ULONG                         SystemErrorBitmap;  // b0: E2PROM version error
    
    // This soft Rx Antenna Diversity mechanism is used only when user set 
    // RX Antenna = DIVERSITY ON
    SOFT_RX_ANT_DIVERSITY         RxAnt;

    ULONG                   Rt2560Version;        // MAC/BBP serial interface issue solved after ver.D
    ULONG                   EepromVersion;        // byte 0: version, byte 1: revision, byte 2~3: unused
    UCHAR                   BbpWriteLatch[100];   // record last BBP register value written via BBP_IO_WRITE
//    ULONG                   CurrTxRateStableTime; // # of second in current TX rate
      ULONG     			NumOfAvgRssiSample;//    UCHAR                   TxRateUpPenalty;      // extra # of second penalty due to last unstable condition
} PORT_CONFIG, *PPORT_CONFIG;

typedef struct _MLME_MEMORY_STRUCT {
    PVOID                           AllocVa;    //Pointer to the base virtual address of the allocated memory
    struct _MLME_MEMORY_STRUCT      *Next;      //Pointer to the next virtual address of the allocated memory
} MLME_MEMORY_STRUCT, *PMLME_MEMORY_STRUCT;

typedef struct  _MLME_MEMORY_HANDLER {
    BOOLEAN                 MemRunning;         //The flag of the Mlme memory handler's status
    UINT                    MemoryCount;        //Total nonpaged system-space memory not size
    UINT                    InUseCount;         //Nonpaged system-space memory in used counts
    UINT                    UnUseCount;         //Nonpaged system-space memory available counts
    UINT                    PendingCount;       //Nonpaged system-space memory for free counts
    PMLME_MEMORY_STRUCT     pInUseHead;         //Pointer to the first nonpaed memory not used
    PMLME_MEMORY_STRUCT     pInUseTail;         //Pointer to the last nonpaged memory not used
    PMLME_MEMORY_STRUCT     pUnUseHead;         //Pointer to the first nonpaged memory in used
    PMLME_MEMORY_STRUCT     pUnUseTail;         //Pointer to the last nonpaged memory in used
    PULONG                  MemFreePending[MAX_MLME_HANDLER_MEMORY];   //an array to keep pending free-memory's pointer (32bits)
} MLME_MEMORY_HANDLER, *PMLME_MEMORY_HANDLER;

typedef struct _MLME_STRUCT {
    STATE_MACHINE           CntlMachine, AssocMachine, AuthMachine, AuthRspMachine, SyncMachine, WpaPskMachine;
    STATE_MACHINE_FUNC      CntlFunc[CNTL_FUNC_SIZE], AssocFunc[ASSOC_FUNC_SIZE];
    STATE_MACHINE_FUNC      AuthFunc[AUTH_FUNC_SIZE], AuthRspFunc[AUTH_RSP_FUNC_SIZE];
    STATE_MACHINE_FUNC      SyncFunc[SYNC_FUNC_SIZE], WpaPskFunc[WPA_PSK_FUNC_SIZE];
    
    ASSOC_AUX               AssocAux;
    AUTH_AUX                AuthAux;
    AUTH_RSP_AUX            AuthRspAux;
    SYNC_AUX                SyncAux;
    CNTL_AUX                CntlAux;
    
    COUNTER_802_11          PrevWlanCounters;
    ULONG                   ChannelQuality;  // 0..100, Channel Quality Indication for Roaming

    BOOLEAN                 Running;
    spinlock_t              TaskLock;
    MLME_QUEUE              Queue;

    UINT                    ShiftReg;
    PSPOLL_FRAME            PsFr;
    MACHDR                  NullFr;
    
    RALINK_TIMER_STRUCT     PeriodicTimer;
    ULONG                   PeriodicRound;
    ULONG                   PrevTxCnt;

    MLME_MEMORY_HANDLER     MemHandler;         //The handler of the nonpaged memory inside MLME
} MLME_STRUCT, *PMLME_STRUCT;

//
// Management ring buffer format
//
typedef struct  _MGMT_STRUC {
    BOOLEAN     Valid;
    PUCHAR      pBuffer;
    ULONG       Length;
}   MGMT_STRUC, *PMGMT_STRUC;

//
// P802.11 Frame control field, 16 bit
//
typedef struct PACKED _FRAME_CONTROL  {
#ifdef	BIG_ENDIAN
	USHORT		Order:1;
	USHORT		Wep:1;
	USHORT		MoreData:1;
	USHORT		PwrMgt:1;
	USHORT		Retry:1;
	USHORT		MoreFrag:1;
	USHORT		FrDs:1;
	USHORT		ToDs:1;
	USHORT		Subtype:4;
	USHORT		Type:2;
	USHORT		Ver:2;
#else
    USHORT      Ver:2;              // Protocol version
    USHORT      Type:2;             // MSDU type
    USHORT      Subtype:4;          // MSDU subtype
    USHORT      ToDs:1;             // To DS indication
    USHORT      FrDs:1;             // From DS indication
    USHORT      MoreFrag:1;         // More fragment bit
    USHORT      Retry:1;            // Retry status bit
    USHORT      PwrMgt:1;           // Power management bit
    USHORT      MoreData:1;         // More data bit
    USHORT      Wep:1;              // Wep data
    USHORT      Order:1;            // Strict order expected
#endif
}   FRAME_CONTROL, *PFRAME_CONTROL;

//
// P802.11 intermediate header format
//
typedef struct PACKED _CONTROL_HEADER {
    FRAME_CONTROL   Frame;              // Frame control structure
    USHORT          Duration;           // Duration value
    MACADDR         Addr1;              // Address 1 field
    MACADDR         Addr2;              // Address 2 field
}   CONTROL_HEADER, *PCONTROL_HEADER;

//
// P802.11 header format
//
typedef struct PACKED _HEADER_802_11  {
    CONTROL_HEADER  Controlhead;
    MACADDR         Addr3;              // Address 3 field
#ifdef BIG_ENDIAN
	USHORT			Sequence:12;		// Sequence number
	USHORT			Frag:4;				// Fragment number
#else
    USHORT          Frag:4;             // Fragment number
    USHORT          Sequence:12;        // Sequence number
#endif
}   HEADER_802_11, *PHEADER_802_11;

//
// Receive Tuple Cache Format
//
typedef struct PACKED _TUPLE_CACHE    {
    BOOLEAN         Valid;
    MACADDR         MAC;
    USHORT          Sequence; 
    USHORT          Frag;
}   TUPLE_CACHE, *PTUPLE_CACHE;

//
// Fragment Frame structure
//
typedef struct PACKED _FRAGMENT_FRAME {
    UCHAR       Header802_3[14];
    UCHAR       Header_LLC[8];
    UCHAR       Buffer[LENGTH_802_3 + MAX_FRAME_SIZE];
    ULONG       RxSize;
    USHORT      Sequence;
    USHORT      LastFrag;
    ULONG       Flags;          // Some extra frame information. bit 0: LLC presented
}   FRAGMENT_FRAME, *PFRAGMENT_FRAME;

//
// Tkip Key structure which RC4 key & MIC calculation
//
typedef struct PACKED _TKIP_KEY_INFO  {
    UINT        nBytesInM;  // # bytes in M for MICKEY
    ULONG       IV16;
    ULONG       IV32;   
    ULONG       K0;         // for MICKEY Low
    ULONG       K1;         // for MICKEY Hig
    ULONG       L;          // Current state for MICKEY
    ULONG       R;          // Current state for MICKEY
    ULONG       M;          // Message accumulator for MICKEY
    UCHAR       RC4KEY[16];
    UCHAR       MIC[8];
}   TKIP_KEY_INFO, *PTKIP_KEY_INFO;

//
// Private / Misc data, counters for driver internal use
//
typedef struct  __PRIVATE_STRUC {
    ULONG       SystemResetCnt;         // System reset counter
    ULONG       TxRingFullCnt;          // Tx ring full occurrance number
    ULONG       ResetCountDown;         // Count down before issue reset, patch for RT2430
    ULONG       CCAErrCnt;              // CCA error count, for debug purpose, might move to global counter
    ULONG       PhyRxErrCnt;            // PHY Rx error count, for debug purpose, might move to global counter
    ULONG       PhyTxErrCnt;            // PHY Tx error count, for debug purpose, might move to global counter
    // Variables for WEP encryption / decryption in rtmp_wep.c
    ULONG           FCSCRC32;
    ULONG           RxSetCnt;
    ULONG           DecryptCnt;
    ARCFOURCONTEXT  WEPCONTEXT;
    // Tkip stuff
    TKIP_KEY_INFO   Tx;
    TKIP_KEY_INFO   Rx;
}   PRIVATE_STRUC, *PPRIVATE_STRUC;

//
//  All DMA ring formats
//
struct  ring_desc   {
    // Descriptor size & dma address
    u32         size;
    void        *va_addr;
    dma_addr_t  pa_addr;
    // Dma buffer size and address for real transfer
    u32         data_size;
    void        *va_data_addr;
    dma_addr_t  pa_data_addr;
    UCHAR       FrameType;          // Type of frame in ring buffer
};

#ifdef RALINK_ATE
typedef	struct _ATE_INFO {
	UCHAR	Mode;
	UCHAR	TxPower;
	UCHAR	Addr1[6];
	UCHAR	Addr2[6];
	UCHAR	Addr3[6];
	UCHAR	Channel;
	ULONG	TxLength;
	ULONG	TxCount;
	ULONG	TxDoneCount;
	ULONG	TxRate;
}	ATE_INFO, *PATE_INFO;
#endif	//#ifdef RALINK_ATE

//
//  The miniport adapter structure
//
typedef struct _RTMP_ADAPTER
{
    char nickn[IW_ESSID_MAX_SIZE+1]; // nickname, only used in the iwconfig i/f 
    int chip_id;

    unsigned long           CSRBaseAddress;     // PCI MMIO Base Address, all access will use
                                                // NdisReadRegisterXx or NdisWriteRegisterXx

    // configuration
    UCHAR                   PermanentAddress[ETH_ALEN];    // Factory default MAC address
    UCHAR                   CurrentAddress[ETH_ALEN];      // User changed MAC address

    UCHAR                   EEPROMAddressNum;       // 93c46=6  93c66=8
    USHORT                  EEPROMDefaultValue[NUM_EEPROM_BBP_PARMS];

    // resource for DMA operation
    struct ring_desc        TxRing[TX_RING_SIZE];       // Tx Ring
    struct ring_desc        AtimRing[ATIM_RING_SIZE];   // Atim Ring
    struct ring_desc        PrioRing[PRIO_RING_SIZE];   // Priority Ring
    struct ring_desc        RxRing[RX_RING_SIZE];       // Rx Ring
    struct ring_desc        BeaconRing;                 // Beacon Ring, only one

    MGMT_STRUC              MgmtRing[MGMT_RING_SIZE];   // management ring size
    
    ULONG                   CurRxIndex;                 // Next RxD read pointer
    ULONG                   CurDecryptIndex;            // Next RxD decrypt read pointer
    ULONG                   CurTxIndex;                 // Next TxD write pointer
    ULONG                   CurEncryptIndex;            // Next TxD encrypt write pointer
    ULONG                   CurAtimIndex;               // Next AtimD write pointer
    ULONG                   CurPrioIndex;               // Next PrioD write pointer
    ULONG                   PushMgmtIndex;              // Next SW management ring index
    ULONG                   PopMgmtIndex;               // Next SW management ring index
    ULONG                   MgmtQueueSize;              // Number of Mgmt request stored in MgmtRing
    ULONG                   NextEncryptDoneIndex;
    ULONG                   NextTxDoneIndex;
    ULONG                   NextAtimDoneIndex;
    ULONG                   NextPrioDoneIndex;
    ULONG                   NextDecryptDoneIndex;

    // 802.3 multicast support
    ULONG                   NumberOfMcAddresses;        // Number of mcast entry exists
    UCHAR                   McastTable[MAX_MCAST_LIST_SIZE][ETH_ALEN];     // Mcast list
   //flags
    ULONG                   Flags;                      // Represent current device status

    // Tx software priority queue list, 802.1q priority information mapped as.
    // 0,1 -> queue0, 2,3 -> queue1, 4,5 -> queue2, 6,7 -> queue3
    struct sk_buff_head            TxSwQueue0;                 // Tx software priority queue 0 mapped to 0.1
    struct sk_buff_head            TxSwQueue1;                 // Tx software priority queue 1 mapped to 2.3
    struct sk_buff_head            TxSwQueue2;                 // Tx software priority queue 2 mapped to 4.5
    struct sk_buff_head            TxSwQueue3; 

    USHORT                  Sequence;                   // Current sequence number

    TUPLE_CACHE             TupleCache[MAX_CLIENT];     // Maximum number of tuple caches, only useful in Ad-Hoc
    UCHAR                   TupleCacheLastUpdateIndex;  // 0..MAX_CLIENT-1
    FRAGMENT_FRAME          FragFrame;                  // Frame storage for fragment frame
    
    // For MiniportTransferData
    PUCHAR                  pRxData;                    // Pointer to current RxRing offset / fragment frame offset
    
    // Counters for 802.3 & generic.
    // Add 802.11 specific counters later
    COUNTER_802_3           Counters;                   // 802.3 counters
    COUNTER_802_11          WlanCounters;               // 802.11 MIB counters
    COUNTER_RALINK          RalinkCounters;             // Ralink propriety counters
    COUNTER_DRS             DrsCounters;                // counters for Dynamic Rate Switching

    NDIS_MEDIA_STATE        MediaState;

    PRIVATE_STRUC           PrivateInfo;                // Private information & counters

    // SpinLocks
    spinlock_t              TxRingLock;                 // Tx Ring spinlock
    spinlock_t              PrioRingLock;               // Prio Ring spinlock
    spinlock_t              AtimRingLock;               // Atim Ring spinlock
    spinlock_t              RxRingLock;                 // Rx Ring spinlock
    spinlock_t              TxSwQueueLock;              // SendTxWaitQueue spinlock
    spinlock_t              MemLock;                    // Memory handler spinlock

// Boolean control for packet filter
    BOOLEAN                 bAcceptDirect;
    BOOLEAN                 bAcceptMulticast;
    BOOLEAN                 bAcceptBroadcast;
    BOOLEAN                 bAcceptAllMulticast;
    BOOLEAN                 bAcceptPromiscuous;
    
    // Control to check Tx hang
    BOOLEAN                 bTxBusy;
    //PQUEUE_ENTRY            FirstEntryInQueue;      // The first packet in Tx queue
    
    // Control disconnect / connect event generation
    ULONG                   LinkDownTime;
    ULONG                   LastRxRate;
    UCHAR                   LastSsidLen;               // the actual ssid length in used
    CHAR                    LastSsid[MAX_LEN_OF_SSID]; // NOT NULL-terminated
    MACADDR                 LastBssid;
    BOOLEAN                 bConfigChanged;

    PORT_CONFIG             PortCfg;
    MLME_STRUCT             Mlme;

    struct pci_dev          *pPci_Dev;
    struct net_device       *net_dev;

    RALINK_TIMER_STRUCT     timer;  // Periodic Media monitoring timer.

    BOOLEAN                 bNetDeviceStopQueue;
    BOOLEAN                 NeedSwapToLittleEndian;
    
#if WIRELESS_EXT >= 12
    struct iw_statistics iw_stats;
#endif
    struct net_device_stats stats;

#ifdef RALINK_ATE
	ATE_INFO				ate;
#endif	//#ifdef RALINK_ATE
}   RTMP_ADAPTER, *PRTMP_ADAPTER;

//
// SHA context
//
typedef struct _SHA_CTX
{
    ULONG       H[5];
    ULONG       W[80];
    INT         lenW;
    ULONG       sizeHi, sizeLo;
}   SHA_CTX;

//
// Enable & Disable NIC interrupt via writing interrupt mask register
// Since it use ADAPTER structure, it have to be put after structure definition.
//
static inline  VOID    NICDisableInterrupt(
    IN  PRTMP_ADAPTER   pAd)
{
    RTMP_IO_WRITE32(pAd, CSR8, 0xFFFF);
    RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_ACTIVE);
}

static inline  VOID    NICEnableInterrupt(
    IN  PRTMP_ADAPTER   pAd)
{
    // 0xFF37 : Txdone & Rxdone, 0xFF07: Txdonw, Rxdone, PrioDone, AtimDone,
    RTMP_IO_WRITE32(pAd, CSR8, 0xFE14);     
    RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_ACTIVE);
}

BOOLEAN NICCheckForHang(
    IN  PRTMP_ADAPTER   pAd);


INT     RT2500_close(
    IN  struct net_device *net_dev);

irqreturn_t RTMPIsr(
    IN  INT             irq, 
    IN  VOID            *dev_instance, 
    IN  struct pt_regs  *rgs);

VOID    RT2500_timer(
    IN  unsigned long data);

INT     RT2500_open(
    IN  struct net_device *net_dev);

INT     RTMPSendPackets(
    IN  struct sk_buff *skb, 
    IN  struct net_device *net_dev);

INT     RT2500_probe(
    IN  struct pci_dev              *pPci_Dev, 
    IN  const struct pci_device_id  *ent);

INT     RT2500_ioctl(
    IN  struct net_device   *net_dev, 
    IN  OUT struct ifreq    *rq, 
    IN  INT                 cmd);

VOID    RTMPRingCleanUp(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  UCHAR           RingType);

#if WIRELESS_EXT >= 12
struct iw_statistics *RT2500_get_wireless_stats(
    IN  struct net_device *net_dev);
#endif

struct net_device_stats *RT2500_get_ether_stats(
    IN  struct net_device *net_dev);

VOID    RT2500_set_rx_mode(
    IN  struct net_device *net_dev);

NDIS_STATUS RTMPAllocDMAMemory(
    IN  PRTMP_ADAPTER   pAd);

VOID    RTMPFreeDMAMemory(
    IN  PRTMP_ADAPTER   pAd);

VOID    NICReadEEPROMParameters(
    IN  PRTMP_ADAPTER       pAdapter);

VOID    NICInitAsicFromEEPROM(
    IN  PRTMP_ADAPTER       pAdapter);

VOID    NICInitTransmit(
    IN  PRTMP_ADAPTER   pAdapter);

NDIS_STATUS NICReadAdapterInfo(
    IN  PRTMP_ADAPTER       pAdapter);

VOID    NICInitializeAdapter(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    NICInitializeAsic(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    NICIssueReset(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    PortCfgInit(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    NICResetFromError(
    IN  PRTMP_ADAPTER   pAdapter);

PUCHAR  RTMPFindSection(
    IN  PCHAR   buffer,
    IN  PCHAR   section);

INT RTMPIsFindSection(
    IN  PUCHAR  ptr,
    IN  PUCHAR  buffer);

INT RTMPGetKeyParameter(
    IN  PUCHAR  section,
    IN  PCHAR   key,
    OUT PCHAR   dest,   
    IN  INT     destsize,
    IN  PCHAR   buffer);

VOID    RTMPReadParametersFromFile(
    IN  PRTMP_ADAPTER   pAd);

#define RTMPEqualMemory(p1,p2,n) (memcmp((p1),(p2),(n)) == 0)
    
ULONG   RTMPCompareMemory(
    IN  PVOID   pSrc1,
    IN  PVOID   pSrc2,
    IN  ULONG   Length);

void AtoH(char * src, UCHAR * dest, int destlen);
UCHAR BtoH(char ch);

VOID	RTMPInitTimer(
	IN	PRTMP_ADAPTER			pAdapter,
	IN	PRALINK_TIMER_STRUCT	pTimer,
	IN	PVOID					pTimerFunc);

VOID	RTMPSetTimer(
    IN  PRTMP_ADAPTER           pAdapter,
	IN	PRALINK_TIMER_STRUCT	pTimer,
	IN	ULONG					Value);

VOID	RTMPCancelTimer(
	IN	PRALINK_TIMER_STRUCT	pTimer);

//
// Private routines in rtmp_data.c
//
VOID    RTMPHandleRxDoneInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandleTxRingTxDoneInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandlePrioRingTxDoneInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandleAtimRingTxDoneInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandleTbcnInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandleTwakeupInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandleDecryptionDoneInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHandleEncryptionDoneInterrupt(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPHardTransmitDone(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PTXD_STRUC      pTxD,
    IN  UCHAR           FrameType);

NDIS_STATUS RTMPSendPacket(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct sk_buff *skb);
    
//VOID  RTMPDeQueuePacket(
//    IN    PRTMP_ADAPTER   pAdapter,
//    IN    PQUEUE_HEADER   pQueue);

VOID    RTMPDeQueuePacket(
    IN  PRTMP_ADAPTER   pAdapter);

NDIS_STATUS RTMPHardEncrypt(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct sk_buff  *skb,
    IN  UCHAR           NumberRequired,
    IN  ULONG           EnableTxBurst,
    IN  UCHAR           AccessCategory);

NDIS_STATUS RTMPHardTransmit(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct sk_buff *skb,
    IN  UCHAR           NumberRequired);

NDIS_STATUS RTMPFreeDescriptorRequest(
    IN      PRTMP_ADAPTER   pAdapter,
    IN      UCHAR           RingType,
    IN      UCHAR           NumberRequired);

VOID    MlmeHardTransmit(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuffer,
    IN  ULONG           Length);

USHORT  RTMPCalcDuration(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  UCHAR           Rate,
    IN  ULONG           Size);

VOID    RTMPWriteTxDescriptor(
    IN  PTXD_STRUC  pTxD,
    IN  BOOLEAN     DoEncrypt,
    IN  UCHAR       CipherAlg,
    IN  BOOLEAN     Ack,
    IN  BOOLEAN     Fragment,
    IN  BOOLEAN     InsTimestamp,
    IN  UCHAR       RetryMode,
    IN  UCHAR       Ifs,
    IN  UINT        Rate,
    IN  UCHAR       Service,
    IN  ULONG       Length,
    IN  USHORT      TxPreamble,
    IN  UCHAR       AccessCategory);

BOOLEAN RTMPSearchTupleCache(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PHEADER_802_11  pHeader);

VOID    RTMPUpdateTupleCache(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PHEADER_802_11  pHeader);

VOID    RTMPSuspendMsduTransmission(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    RTMPResumeMsduTransmission(
    IN  PRTMP_ADAPTER   pAdapter);

NDIS_STATUS MiniportMMRequest(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuffer,
    IN  ULONG           Length);

VOID    RTMPSendNullFrame(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuffer,
    IN  ULONG           Length,
    IN  UCHAR           TxRate);

NDIS_STATUS RTMPApplyPacketFilter(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PRXD_STRUC      pRxD, 
    IN  PHEADER_802_11  pHeader);

struct sk_buff_head* RTMPCheckTxSwQueue(
    IN  PRTMP_ADAPTER   pAdapter,
    OUT UCHAR           *AccessCategory);

VOID    RTMPReportMicError(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PWPA_KEY        pWpaKey);
//
// Private routines in rtmp_wep.c
//
VOID    RTMPInitWepEngine(
    IN  PRTMP_ADAPTER   pAdapter,   
    IN  PUCHAR          pKey,
    IN  UCHAR           KeyId,
    IN  UCHAR           KeyLen, 
    IN  PUCHAR          pDest);

VOID    RTMPEncryptData(
    IN  PRTMP_ADAPTER   pAdapter,   
    IN  PUCHAR          pSrc,
    IN  PUCHAR          pDest,
    IN  UINT            Len);

BOOLEAN RTMPDecryptData(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PUCHAR          pSrc,
    IN  UINT            Len);

VOID    RTMPSetICV(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PUCHAR          pDest);

VOID    ARCFOUR_INIT(
    IN  PARCFOURCONTEXT Ctx,
    IN  PUCHAR          pKey,
    IN  UINT            KeyLen);

UCHAR   ARCFOUR_BYTE(
    IN  PARCFOURCONTEXT     Ctx);

VOID    ARCFOUR_DECRYPT(
    IN  PARCFOURCONTEXT Ctx,
    IN  PUCHAR          pDest, 
    IN  PUCHAR          pSrc,
    IN  UINT            Len);

VOID    ARCFOUR_ENCRYPT(
    IN  PARCFOURCONTEXT Ctx,
    IN  PUCHAR          pDest,
    IN  PUCHAR          pSrc,
    IN  UINT            Len);

ULONG   RTMP_CALC_FCS32(
    IN  ULONG   Fcs,
    IN  PUCHAR  Cp,
    IN  INT     Len);

//
// MLME routines
//
//VOID    Arc4Init(ARC4_CONTEXT *Ctx, UCHAR *Key, ULONG KeyLen);
//UCHAR   Arc4Byte(ARC4_CONTEXT *Ctx);
//VOID    Arc4Cipher(ARC4_CONTEXT *Ctx, UCHAR *Dest, UCHAR *Src, ULONG Len);

// Asic/RF/BBP related functions

VOID AsicAdjustTxPower(
    IN PRTMP_ADAPTER pAd);

VOID    AsicSwitchChannel(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN UCHAR Channel);

VOID    AsicLockChannel(
    IN PRTMP_ADAPTER pAd, 
    IN UCHAR Channel) ;

VOID AsicRfTuningExec(
    IN unsigned long data);

VOID    AsicSleepThenAutoWakeup(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  USHORT TbttNumToNextWakeUp);

VOID    AsicForceSleep(
    IN PRTMP_ADAPTER pAdapter);

VOID    AsicForceWakeup(
    IN PRTMP_ADAPTER pAdapter);

VOID    AsicSetBssid(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MACADDR *Bssid);

VOID    AsicDisableSync(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    AsicEnableBssSync(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    AsicEnableIbssSync(
    IN  PRTMP_ADAPTER   pAdapter);

VOID    AsicLedPeriodicExec(
    IN unsigned long data);

VOID AsicSetRxAnt(
    IN PRTMP_ADAPTER pAd);

VOID AsicEvaluateSecondaryRxAnt(
    IN PRTMP_ADAPTER pAd);

VOID AsicRxAntEvalTimeout(
    IN unsigned long data);

VOID AsicSetSlotTime(
    IN PRTMP_ADAPTER pAd,
    IN BOOLEAN UseShortSlotTime);

VOID AsicAdjustUsec(
    IN PRTMP_ADAPTER pAd);

VOID AsicBbpTuning(
    IN PRTMP_ADAPTER pAd);

VOID AsicRestoreBbpSensibility(
    IN PRTMP_ADAPTER pAd);

VOID    MacAddrRandomBssid(
    IN  PRTMP_ADAPTER   pAdapter, 
    OUT PMACADDR Addr);

VOID    MgtMacHeaderInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN OUT PMACHDR Hdr, 
    IN UCHAR Subtype, 
    IN UCHAR ToDs, 
//  IN UCHAR AddrType, 
    IN PMACADDR Ds, 
    IN PMACADDR Bssid);

VOID MlmeRadioOff(
    IN PRTMP_ADAPTER pAd);

VOID MlmeRadioOn(
    IN PRTMP_ADAPTER pAd);

VOID  BssTableInit(
    IN BSS_TABLE *Tab);

ULONG BssTableSearch(
    IN BSS_TABLE *Tab, 
    IN PMACADDR Bssid);

VOID BssTableDeleteEntry(
    IN OUT  BSS_TABLE *Tab, 
    IN      PMACADDR Bssid);

VOID  BssEntrySet(
    IN  PRTMP_ADAPTER   pAdapter, 
    OUT BSS_ENTRY *Bss, 
    IN MACADDR *Bssid, 
    IN CHAR Ssid[], 
    IN UCHAR SsidLen, 
    IN UCHAR BssType, 
    IN USHORT BeaconPeriod,
    IN BOOLEAN CfExist, 
    IN CF_PARM *CfParm, 
    IN USHORT AtimWin, 
    IN USHORT CapabilityInfo, 
    IN UCHAR Rates[], 
    IN UCHAR RatesLen,
    IN BOOLEAN ExtendedRateIeExist,
    IN UCHAR Channel,
    IN UCHAR Rssi,
    IN LARGE_INTEGER TimeStamp,
    IN PNDIS_802_11_VARIABLE_IEs pVIE);

ULONG  BssTableSetEntry(
    IN  PRTMP_ADAPTER   pAdapter, 
    OUT BSS_TABLE *Tab, 
    IN MACADDR *Bssid, 
    IN CHAR Ssid[], 
    IN UCHAR SsidLen, 
    IN UCHAR BssType, 
    IN USHORT BeaconPeriod, 
    IN BOOLEAN CfExist, 
    IN CF_PARM *CfParm, 
    IN USHORT AtimWin, 
    IN USHORT CapabilityInfo, 
    IN UCHAR Rates[], 
    IN UCHAR RatesLen,
    IN BOOLEAN ExtendedRateIeExist,
    IN UCHAR Channel,
    IN UCHAR Rssi,
    IN LARGE_INTEGER TimeStamp,
    IN PNDIS_802_11_VARIABLE_IEs pVIE);

VOID  BssTableSsidSort(
    IN  PRTMP_ADAPTER   pAd, 
    OUT BSS_TABLE *OutTab, 
    IN  CHAR Ssid[], 
    IN  UCHAR SsidLen);

VOID  BssTableSortByRssi(
    IN OUT BSS_TABLE *OutTab);

NDIS_802_11_WEP_STATUS  BssCipherParse(
    IN  PUCHAR  pCipher);

NDIS_STATUS  MlmeQueueInit(
    IN MLME_QUEUE *Queue);

VOID  MlmeQueueDestroy(
    IN MLME_QUEUE *Queue);

BOOLEAN MlmeEnqueue(
    OUT MLME_QUEUE *Queue, 
    IN ULONG Machine, 
    IN ULONG MsgType, 
    IN ULONG MsgLen, 
    IN VOID *Msg);

BOOLEAN MlmeEnqueueForRecv(
    IN  PRTMP_ADAPTER   pAdapter, 
    OUT MLME_QUEUE *Queue, 
    IN ULONG TimeStampHigh, 
    IN ULONG TimeStampLow, 
    IN UCHAR Rssi, 
    IN ULONG MsgLen, 
    IN PVOID Msg);

BOOLEAN MlmeDequeue(
    IN MLME_QUEUE *Queue, 
    OUT MLME_QUEUE_ELEM **Elem);

VOID    MlmeRestartStateMachine(
    IN  PRTMP_ADAPTER   pAd);

BOOLEAN MlmeQueueEmpty(
    IN MLME_QUEUE *Queue);

BOOLEAN MlmeQueueFull(
    IN MLME_QUEUE *Queue);

BOOLEAN  MsgTypeSubst(
    IN MACFRAME *Fr, 
    OUT INT *Machine, 
    OUT INT *MsgType);

VOID StateMachineInit(
    IN STATE_MACHINE *Sm, 
    IN STATE_MACHINE_FUNC Trans[], 
    IN ULONG StNr, 
    IN ULONG MsgNr, 
    IN STATE_MACHINE_FUNC DefFunc, 
    IN ULONG InitState, 
    IN ULONG Base);

VOID StateMachineSetAction(
    IN STATE_MACHINE *S, 
    IN ULONG St, 
    ULONG Msg, 
    IN STATE_MACHINE_FUNC F);

VOID StateMachinePerformAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN STATE_MACHINE *S, 
    IN MLME_QUEUE_ELEM *Elem);

VOID Drop(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN MLME_QUEUE_ELEM *Elem);

VOID StateMachineDestroy(
    IN STATE_MACHINE *Sm);

VOID  AssocStateMachineInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  STATE_MACHINE *Sm, 
    OUT STATE_MACHINE_FUNC Trans[]);

VOID  ReassocTimeout(
    IN  unsigned long data);

VOID  AssocTimeout(
    IN  unsigned long data);

VOID  DisassocTimeout(
    IN  unsigned long data);

//----------------------------------------------
VOID  MlmeDisassocReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  MlmeAssocReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  MlmeReassocReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  MlmeDisassocReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  PeerAssocRspAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  PeerReassocRspAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  PeerDisassocAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  DisassocTimeoutAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  AssocTimeoutAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  ReassocTimeoutAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID  Cls3errAction(
    IN PRTMP_ADAPTER pAdapter,
    IN PMACADDR      pAddr);

VOID  InvalidStateWhenAssoc(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  MLME_QUEUE_ELEM *Elem);

VOID  InvalidStateWhenReassoc(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  MLME_QUEUE_ELEM *Elem);

VOID InvalidStateWhenDisassociate(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  MLME_QUEUE_ELEM *Elem);

VOID  ComposePsPoll(
    IN  PRTMP_ADAPTER   pAdapter);

VOID  ComposeNullFrame(
    IN  PRTMP_ADAPTER   pAdapter);

VOID  AssocPostProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MACADDR *Addr2, 
    IN  USHORT CapabilityInfo, 
    IN  USHORT Aid, 
    IN  UCHAR Rates[], 
    IN  UCHAR RatesLen,
    IN  BOOLEAN ExtendedRateIeExist);

VOID AuthStateMachineInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN PSTATE_MACHINE sm, 
    OUT STATE_MACHINE_FUNC Trans[]);

VOID AuthTimeout(
    IN  unsigned long data);

VOID MlmeAuthReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerAuthRspAtSeq2Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerAuthRspAtSeq4Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID AuthTimeoutAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID Cls2errAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PMACADDR pAddr);

VOID MlmeDeauthReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID InvalidStateWhenAuth(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

//VOID MlmeDeauthReqProc(
//    IN    PRTMP_ADAPTER   pAdapter, 
//    IN  MACADDR *Addr, 
//    IN  USHORT Reason);

//=============================================

VOID AuthRspStateMachineInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PSTATE_MACHINE Sm, 
    IN  STATE_MACHINE_FUNC Trans[]);


VOID AuthRspChallengeTimeout(
    IN  unsigned long data);

VOID AuthRspChallengeTimeoutAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerAuthAtAuthRspIdleAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerAuthAtAuthRspWaitAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerDeauthAction(
    IN  PRTMP_ADAPTER   pAdaptor, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerAuthSimpleRspGenAndSend(
    IN  PRTMP_ADAPTER pAdapter, 
    IN  PMACHDR Hdr, 
    IN  USHORT Alg, 
    IN  USHORT Seq, 
    IN  USHORT Reason, 
    IN  USHORT Status);

//========================================

VOID SyncStateMachineInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  STATE_MACHINE *Sm, 
    OUT STATE_MACHINE_FUNC Trans[]);

VOID BeaconTimeout(
    IN  unsigned long data);

VOID AtimTimeout(
    IN  unsigned long data);

VOID ScanTimeout(
    IN  unsigned long data);

VOID MlmeScanReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID InvalidStateWhenScan(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID InvalidStateWhenJoin(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID InvalidStateWhenStart(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerBeacon(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID EnqueueProbeRequest(
    IN PRTMP_ADAPTER pAd);

//=========================================

VOID MlmeCntlInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  STATE_MACHINE *S, 
    OUT STATE_MACHINE_FUNC Trans[]);

VOID MlmeCntlMachinePerformAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  STATE_MACHINE *S, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlIdleProc(
    IN PRTMP_ADAPTER pAdapter, 
    IN MLME_QUEUE_ELEM *Elem);

VOID CntlOidScanProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlOidSsidProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM * Elem);

VOID CntlOidRTBssidProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlMlmeRoamingProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitDisassocProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitJoinProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitReassocProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitStartProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitAuthProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitAuthProc2(
    IN  PRTMP_ADAPTER pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID CntlWaitAssocProc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID LinkUp(
    IN  PRTMP_ADAPTER pAdapter,
    IN  UCHAR BssType);

VOID LinkDown(
    IN  PRTMP_ADAPTER   pAdapter);

VOID MlmeCntlConfirm(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  ULONG MsgType, 
    IN  USHORT Msg);

VOID IterateOnBssTab(
    IN  PRTMP_ADAPTER   pAdapter);

VOID IterateOnBssTab2(
    IN  PRTMP_ADAPTER   pAdapter);;

VOID JoinParmFill(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  OUT MLME_JOIN_REQ_STRUCT *JoinReq, 
    IN  ULONG BssIdx);

VOID AssocParmFill(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN OUT MLME_ASSOC_REQ_STRUCT *AssocReq, 
    IN  MACADDR *Addr, 
    IN  USHORT CapabilityInfo, 
    IN  ULONG Timeout, 
    IN  USHORT ListenIntv);

VOID ScanParmFill(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  OUT MLME_SCAN_REQ_STRUCT *ScanReq, 
    IN  CHAR Ssid[], 
    IN  UCHAR SsidLen, 
    IN  UCHAR BssType, 
    IN  UCHAR ScanType); 

VOID DisassocParmFill(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  OUT MLME_DISASSOC_REQ_STRUCT *DisassocReq, 
    IN  MACADDR *Addr, 
    IN  USHORT Reason);

VOID StartParmFill(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  OUT MLME_START_REQ_STRUCT *StartReq, 
    IN  CHAR Ssid[], 
    IN  UCHAR SsidLen);

VOID AuthParmFill(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  OUT MLME_AUTH_REQ_STRUCT *AuthReq, 
    IN  MACADDR *Addr, 
    IN  USHORT Alg);

VOID EnqueuePsPoll(
    IN  PRTMP_ADAPTER   pAdapter);

VOID EnqueueBeaconFrame(
    IN  PRTMP_ADAPTER   pAdapter);

VOID EnqueueNullFrame(
    IN  PRTMP_ADAPTER pAdapter,
    IN  UCHAR         TxRate);

VOID MlmeJoinReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID MlmeScanReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID MlmeStartReqAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID ScanTimeoutAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID BeaconTimeoutAtJoinAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerBeaconAtScanAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerBeaconAtJoinAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerBeacon(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID PeerProbeReqAction(
    IN  PRTMP_ADAPTER pAd, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID ScanNextChannel(
    IN  PRTMP_ADAPTER   pAdapter);

ULONG MakeIbssBeacon(
    IN  PRTMP_ADAPTER pAdapter);

BOOLEAN MlmeScanReqSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT UCHAR *BssType, 
    OUT CHAR ssid[], 
    OUT UCHAR *SsidLen, 
    OUT UCHAR *ScanType);

BOOLEAN PeerBeaconAndProbeRspSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT MACADDR *Bssid, 
    OUT CHAR Ssid[], 
    OUT UCHAR *SsidLen, 
    OUT UCHAR *BssType, 
    OUT USHORT *BeaconPeriod, 
    OUT UCHAR *Channel, 
    OUT LARGE_INTEGER *Timestamp, 
    OUT BOOLEAN *CfExist, 
    OUT CF_PARM *Cf, 
    OUT USHORT *AtimWin, 
    OUT USHORT *CapabilityInfo, 
    OUT UCHAR Rate[], 
    OUT UCHAR *RateLen,
    OUT BOOLEAN *ExtendedRateIeExist,
    OUT UCHAR *Erp,
    OUT UCHAR *DtimCount, 
    OUT UCHAR *DtimPeriod, 
    OUT UCHAR *BcastFlag, 
    OUT UCHAR *MessageToMe, 
    OUT UCHAR *Legacy,
    OUT UCHAR SupRate[],
	OUT UCHAR *SupRateLen,
	OUT UCHAR ExtRate[],
	OUT UCHAR *ExtRateLen,
    OUT	PNDIS_802_11_VARIABLE_IEs pVIE);

//BOOLEAN JoinParmSanity(
//    IN    PRTMP_ADAPTER   pAdapter, 
//    IN  VOID *Msg, 
//    IN  ULONG MsgLen, 
//    OUT ULONG *BssIdx,
//    OUT UCHAR SupportedRates[], 
//    OUT UCHAR *SupportedRatesLen);

BOOLEAN MlmeAssocReqSanity(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *ApAddr, 
    OUT USHORT *CapabilityInfo, 
    OUT ULONG *Timeout, 
    OUT USHORT *ListenIntv);

BOOLEAN MlmeAuthReqSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr, 
    OUT ULONG *Timeout, 
    OUT USHORT *Alg);

BOOLEAN MlmeStartReqSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT CHAR Ssid[], 
    OUT UCHAR *Ssidlen);

BOOLEAN PeerAuthSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr, 
    OUT USHORT *Alg, 
    OUT USHORT *Seq, 
    OUT USHORT *Status, 
    OUT CHAR ChlgText[]);

BOOLEAN PeerAssocRspSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT USHORT *CapabilityInfo, 
    OUT USHORT *Status, 
    OUT USHORT *Aid, 
    OUT UCHAR Rates[], 
    OUT UCHAR *RatesLen,
    OUT BOOLEAN *ExtendedRateIeExist);

BOOLEAN PeerDisassocSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT USHORT *Reason);

BOOLEAN PeerDeauthSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT USHORT *Reason);

BOOLEAN PeerProbeReqSanity(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  VOID *Msg, 
    IN  ULONG MsgLen, 
    OUT MACADDR *Addr2,
    OUT CHAR Ssid[], 
    OUT UCHAR *SsidLen); 
//    OUT UCHAR Rates[], 
//    OUT UCHAR *RatesLen);

BOOLEAN GetTimBit(
    IN  CHAR *Ptr, 
    IN  USHORT Aid, 
    OUT UCHAR *TimLen, 
    OUT UCHAR *BcastFlag, 
    OUT UCHAR *DtimCount, 
    OUT UCHAR *DtimPeriod, 
    OUT UCHAR *MessageToMe);

BOOLEAN GetLegacy(
    IN  CHAR *Ptr, 
    OUT UCHAR *Legacy);

ULONG MakeOutgoingFrame(
    OUT CHAR *Buffer, 
    OUT ULONG *Length, ...);

VOID  LfsrInit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  ULONG Seed);

UCHAR RandomByte(
    IN  PRTMP_ADAPTER   pAdapter);

VOID  MlmePeriodicExec(
    IN  unsigned long data);

VOID MlmeAutoScan(
    IN PRTMP_ADAPTER pAdapter);

VOID MlmeAutoRecoverNetwork(
    IN PRTMP_ADAPTER pAdapter);

VOID MlmeAutoReconnectLastSSID(
    IN PRTMP_ADAPTER pAdapter);

VOID MlmeCheckForRoaming(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN ULONG    Now32);

VOID MlmeCheckDynamicTxRateSwitching(
    IN PRTMP_ADAPTER pAd);

VOID MlmeCheckChannelQuality(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN ULONG Now);

VOID MlmeCheckForPsmChange(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN ULONG    Now32);

VOID MlmeSetPsmBit(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN USHORT psm);

VOID MlmeSetTxPreamble(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN USHORT TxPreamble);

VOID MlmeUpdateTxRates(
    IN PRTMP_ADAPTER pAdapter,
    IN BOOLEAN		 bLinkUp);

NDIS_STATUS MlmeInit(
    IN  PRTMP_ADAPTER   pAdapter);

VOID MlmeHandler(
    IN  PRTMP_ADAPTER   pAdapter);

VOID MlmeHalt(
    IN  PRTMP_ADAPTER   pAdapter);

NDIS_STATUS MlmeInitMemoryHandler(
    IN PRTMP_ADAPTER    pAd,
    IN UINT             Number,
    IN UINT             Size);

NDIS_STATUS MlmeAllocateMemory(
    IN PRTMP_ADAPTER    pAd,
    OUT PVOID           *AllocVa);

VOID MlmeFreeMemory(
    IN PRTMP_ADAPTER    pAd,
    IN PVOID            AllocVa);

VOID MlmeFreeMemoryHandler(
    IN PRTMP_ADAPTER    pAd);

VOID BuildChannelList(
    IN PRTMP_ADAPTER pAdapter);

UCHAR FirstChannel(
    IN  PRTMP_ADAPTER   pAdapter);

UCHAR NextChannel(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  UCHAR channel);

VOID RaiseClock(
    IN  PRTMP_ADAPTER   pAd,
    IN  ULONG *x);

VOID LowerClock(
    IN  PRTMP_ADAPTER   pAd,
    IN  ULONG *x);

USHORT ShiftInBits(
    IN  PRTMP_ADAPTER   pAd);

VOID ShiftOutBits(
    IN  PRTMP_ADAPTER   pAd,
    IN  USHORT data,
    IN  USHORT count);

VOID EEpromCleanup(
    IN  PRTMP_ADAPTER   pAd);

VOID EWDS(
    IN  PRTMP_ADAPTER   pAd);

VOID EWEN(
    IN  PRTMP_ADAPTER   pAd);
    
USHORT RTMP_EEPROM_READ16(
    IN  PRTMP_ADAPTER   pAd,
    IN  USHORT Offset);

VOID RTMP_EEPROM_WRITE16(
    IN  PRTMP_ADAPTER   pAd,
    IN  USHORT Offset,
    IN  USHORT Data);
    
UCHAR ChannelSanity(
    IN PRTMP_ADAPTER pAd, 
    IN UCHAR channel);

//
// Prototypes of function definition in rtmp_tkip.c
//
VOID    RTMPInitTkipEngine(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          pTKey,
    IN  UCHAR           KeyId,
    IN  PUCHAR          pTA,
    IN  PUCHAR          pMICKey,
    IN  PUCHAR          pTSC,
    OUT PULONG          pIV16,
    OUT PULONG          pIV32);

VOID    RTMPInitMICEngine(
    IN  PRTMP_ADAPTER   pAdapter,   
    IN  PUCHAR          pKey,
    IN  PUCHAR          pDA,
    IN  PUCHAR          pSA,
    IN  PUCHAR          pMICKey);

BOOLEAN RTMPTkipCompareMICValue(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          pSrc,
    IN  PUCHAR          pDA,
    IN  PUCHAR          pSA,
    IN  PUCHAR          pMICKey,
    IN  UINT            Len);

VOID    RTMPCalculateMICValue(
    IN PRTMP_ADAPTER pAdapter,
    IN  struct sk_buff  *skb,
    IN  PUCHAR          pEncap,
    IN  INT             LenEncap,
    IN  PWPA_KEY        pWpaKey);

BOOLEAN RTMPTkipCompareMICValueWithLLC(
    IN PRTMP_ADAPTER pAdapter,
    IN  PUCHAR          pLLC,
    IN  PUCHAR          pSrc,
    IN  PUCHAR          pDA,
    IN  PUCHAR          pSA,
    IN  PUCHAR          pMICKey,
    IN  UINT            Len);

VOID    RTMPTkipAppend( 
    IN  PTKIP_KEY_INFO  pTkip,  
    IN  PUCHAR          pSrc,
    IN  UINT            nBytes);

VOID    RTMPTkipGetMIC( 
    IN  PTKIP_KEY_INFO  pTkip);

NDIS_STATUS RTMPWPAAddKeyProc(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuf);

NDIS_STATUS RTMPWPARemoveKeyProc(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuf);

VOID    RTMPWPARemoveAllKeys(
    IN PRTMP_ADAPTER pAdapter);

VOID    RTMPSetPhyMode(
    IN PRTMP_ADAPTER pAdapter, 
    IN  ULONG phymode);

VOID    RTMPSetDesiredRates(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  LONG            Rates);

INT RTMPSetInformation(
    IN  PRTMP_ADAPTER pAdapter,
    IN  OUT struct ifreq    *rq,
    IN  INT             cmd);

INT RTMPQueryInformation(
    IN  PRTMP_ADAPTER pAdapter,
    IN  OUT struct ifreq    *rq,
    IN  INT             cmd);

//
// Prototypes of function definition for *iwpriv* in rtmp_info.c
//
INT Set_CountryRegion_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_SSID_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_WirelessMode_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_TxRate_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_AdhocModeRate_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_Channel_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR  
    arg);

#ifdef RT2500_DBG
INT Set_Debug_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);
#endif

INT Set_BGProtection_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_TxPreamble_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_StaWithEtherBridge_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_RTSThreshold_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_FragThreshold_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_TxBurst_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_TurboRate_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_NetworkType_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);
    
INT Set_AuthMode_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_EncrypType_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_DefaultKeyID_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_Key1_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_Key2_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_Key3_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_Key4_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_WPAPSK_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

INT Set_WPANONE_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg);

VOID RTMPIoctlBBP(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct iwreq    *wrq);

VOID RTMPIoctlMAC(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct iwreq    *wrq);

#ifdef RALINK_ATE
VOID RTMPIoctlE2PROM(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct iwreq    *wrq);
#endif

//
// prototype in wpa.c
//
BOOLEAN WpaMsgTypeSubst(
    IN  UCHAR   EAPType,
    OUT ULONG   *MsgType);

VOID WpaPskStateMachineInit(
    IN  PRTMP_ADAPTER       pAd, 
    IN  STATE_MACHINE       *S, 
    OUT STATE_MACHINE_FUNC Trans[]);

VOID WpaEAPOLKeyAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID    WpaPairMsg1Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID    WpaPairMsg3Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem); 

VOID    WpaGroupMsg1Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem);

VOID    WpaMacHeaderInit(
    IN      PRTMP_ADAPTER   pAd, 
    IN OUT  PHEADER_802_11  Hdr, 
    IN      UCHAR           wep, 
    IN      PMACADDR        pAddr1); 

VOID    WpaHardEncrypt(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PUCHAR          pPacket,
    IN  ULONG           Len);

VOID    HMAC_SHA1(
    IN  UCHAR   *text,
    IN  UINT    text_len,
    IN  UCHAR   *key,
    IN  UINT    key_len,
    IN  UCHAR   *digest);

VOID    PRF(
    IN  UCHAR   *key,
    IN  INT     key_len,
    IN  UCHAR   *prefix,
    IN  INT     prefix_len,
    IN  UCHAR   *data,
    IN  INT     data_len,
    OUT UCHAR   *output,
    IN  INT     len);

VOID WpaCountPTK(
    IN  UCHAR   *PMK,
    IN  UCHAR   *ANonce,
    IN  UCHAR   *AA,
    IN  UCHAR   *SNonce,
    IN  UCHAR   *SA,
    OUT UCHAR   *output,
    IN  UINT    len);

VOID    GenRandom(
    IN  PRTMP_ADAPTER   pAd, 
    OUT UCHAR           *random);

VOID    AES_GTK_KEY_UNWRAP( 
    IN  UCHAR   *key,
    OUT UCHAR   *plaintext,
    IN  UCHAR   *ciphertext);

ULONG	RTMPTkipGetUInt32( 	
	IN	PUCHAR	pMICKey);

char * rtstrstr(
	IN	const char * s1,
	IN	const char * s2);

#ifdef RALINK_ATE
INT	Set_ATE_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_DA_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_SA_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_BSSID_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_CHANNEL_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_TX_POWER_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_TX_LENGTH_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_TX_COUNT_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

INT	Set_ATE_TX_RATE_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg);

VOID RTMPStationStop(
    IN  PRTMP_ADAPTER   pAd);

VOID RTMPStationStart(
    IN  PRTMP_ADAPTER   pAd);

#endif	//#ifdef RALINK_ATE

int rt2500_set_mac_address(struct net_device *net_dev, void *addr);

#ifdef BIG_ENDIAN
VOID   RTMPFrameEndianChange(
       IN  PRTMP_ADAPTER   pAdapter, 
       IN  PUCHAR          pData, 
       IN  ULONG           Dir,
       IN  BOOLEAN         FromRxDoneInt);

VOID    RTMPDescriptorEndianChange(
       IN      PUCHAR                  pData,
       IN      ULONG                   DescriptorType);
#endif

#endif  // __RTMP_H__
