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
 *      Module Name: mlme.c
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW           8th  Dec 04     kmalloc ATOMIC fixes
 *      RobinC          10th Dec 04     RFMON Support 
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0 
 *      Ivo (rt2400)    15th Dec 04     Uninitialised timer 
 *      MarkW           17th Dec 04     Monitor mode through iwconfig
 *      BrunoH			3rd  Feb 04     Fix for 802.11b adhoc association
 *      JohnC           19th Mar 04     Fixes for quality reporting     
 * 		MarkW			13th Jun 05		Fix to allow adhoc network creation
 ***************************************************************************/ 

#include "rt_config.h"
#include <stdarg.h>

// e.g. RssiSafeLevelForTxRate[RATE_36]" means if the current RSSI is greater than 
//      this value, then it's quaranteed capable of operating in 36 mbps TX rate in 
//      clean environment.
//                          TxRate: 1   2   5.5   11   6    9    12   18   24   36   48   54   72  100
CHAR RssiSafeLevelForTxRate[] ={  -92, -91, -90, -87, -88, -86, -85, -83, -81, -78, -72, -71, -40, -40 };

                                  //  1      2       5.5      11  
UCHAR Phy11BNextRateDownward[] = {RATE_1, RATE_1,   RATE_2,  RATE_5_5};
UCHAR Phy11BNextRateUpward[]   = {RATE_2, RATE_5_5, RATE_11, RATE_11};

                                  //  1      2       5.5      11        6        9        12      18       24       36       48       54
UCHAR Phy11BGNextRateDownward[]= {RATE_1, RATE_1,   RATE_2,  RATE_5_5,RATE_11,  RATE_6,  RATE_11, RATE_12, RATE_18, RATE_24, RATE_36, RATE_48};
UCHAR Phy11BGNextRateUpward[]  = {RATE_2, RATE_5_5, RATE_11, RATE_12, RATE_9,   RATE_12, RATE_18, RATE_24, RATE_36, RATE_48, RATE_54, RATE_54};

                                  //  1      2       5.5      11        6        9        12      18       24       36       48       54
UCHAR Phy11ANextRateDownward[] = {RATE_6, RATE_6,   RATE_6,  RATE_6,  RATE_6,   RATE_6,  RATE_9,  RATE_12, RATE_18, RATE_24, RATE_36, RATE_48};
UCHAR Phy11ANextRateUpward[]   = {RATE_9, RATE_9,   RATE_9,  RATE_9,  RATE_9,   RATE_12, RATE_18, RATE_24, RATE_36, RATE_48, RATE_54, RATE_54};

// 2560D and after has implemented ASIC-based OFDM rate switching, but not
// 2560C and before. thus software use different PER for rate switching
//                          RATE_1,  2, 5.5, 11,  6,  9, 12, 18, 24, 36, 48, 54
USHORT NewRateUpPER[]   = {    40,  40,  35, 20, 20, 20, 20, 16, 10, 16, 10,  6 }; // in percentage
USHORT NewRateDownPER[] = {    50,  50,  45, 45, 35, 35, 35, 35, 25, 25, 25, 13 }; // in percentage

USHORT OldRateUpPER[]   = {    40,  40,  40, 40, 30, 30, 30, 30, 20, 20, 10, 10 }; // in percentage
USHORT OldRateDownPER[] = {    45,  45,  45, 45, 35, 35, 35, 35, 25, 25, 25, 12 }; // in percentage
    
UCHAR  RateIdToMbps[]    = { 1, 2, 5, 11, 6, 9, 12, 18, 24, 36, 48, 54, 72, 100};
USHORT RateIdTo500Kbps[] = { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108, 144, 200};
    
RTMP_RF_REGS RF2522RegTable[] = {
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94002050, 0x940c1fda, 0x94000101, 0},
        {2,  0x94002050, 0x940c1fee, 0x94000101, 0},
        {3,  0x94002050, 0x940c2002, 0x94000101, 0},
        {4,  0x94002050, 0x940c2016, 0x94000101, 0},
        {5,  0x94002050, 0x940c202a, 0x94000101, 0},
        {6,  0x94002050, 0x940c203e, 0x94000101, 0},
        {7,  0x94002050, 0x940c2052, 0x94000101, 0},
        {8,  0x94002050, 0x940c2066, 0x94000101, 0},
        {9,  0x94002050, 0x940c207a, 0x94000101, 0},
        {10, 0x94002050, 0x940c208e, 0x94000101, 0},
        {11, 0x94002050, 0x940c20a2, 0x94000101, 0},
        {12, 0x94002050, 0x940c20b6, 0x94000101, 0},
        {13, 0x94002050, 0x940c20ca, 0x94000101, 0},
        {14, 0x94002050, 0x940c20fa, 0x94000101, 0}
};
#define	NUM_OF_2522_CHNL	(sizeof(RF2522RegTable) / sizeof(RTMP_RF_REGS))

RTMP_RF_REGS RF2523RegTable[] = {
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94022010, 0x94000c9e, 0x940e0111, 0x94000a1b},
        {2,  0x94022010, 0x94000ca2, 0x940e0111, 0x94000a1b},
        {3,  0x94022010, 0x94000ca6, 0x940e0111, 0x94000a1b},
        {4,  0x94022010, 0x94000caa, 0x940e0111, 0x94000a1b},
        {5,  0x94022010, 0x94000cae, 0x940e0111, 0x94000a1b},
        {6,  0x94022010, 0x94000cb2, 0x940e0111, 0x94000a1b},
        {7,  0x94022010, 0x94000cb6, 0x940e0111, 0x94000a1b},
        {8,  0x94022010, 0x94000cba, 0x940e0111, 0x94000a1b},
        {9,  0x94022010, 0x94000cbe, 0x940e0111, 0x94000a1b},
        {10, 0x94022010, 0x94000d02, 0x940e0111, 0x94000a1b},
        {11, 0x94022010, 0x94000d06, 0x940e0111, 0x94000a1b},
        {12, 0x94022010, 0x94000d0a, 0x940e0111, 0x94000a1b},
        {13, 0x94022010, 0x94000d0e, 0x940e0111, 0x94000a1b},
        {14, 0x94022010, 0x94000d1a, 0x940e0111, 0x94000a03}
#if 0
        {1,  0x94022050, 0x940c1fda, 0x940e8101, 0},
        {2,  0x94022050, 0x940c1fee, 0x940e8101, 0},
        {3,  0x94022050, 0x940c2002, 0x940e8101, 0},
        {4,  0x94022050, 0x940c2016, 0x940e8101, 0},
        {5,  0x94022050, 0x940c202a, 0x940e8101, 0},
        {6,  0x94022050, 0x940c203e, 0x940e8101, 0},
        {7,  0x94022050, 0x940c2052, 0x940e8101, 0},
        {8,  0x94022050, 0x940c2066, 0x940e8101, 0},
        {9,  0x94022050, 0x940c207a, 0x940e8101, 0},
        {10, 0x94022050, 0x940c208e, 0x940e8101, 0},
        {11, 0x94022050, 0x940c20a2, 0x940e8101, 0},
        {12, 0x94022050, 0x940c20b6, 0x940e8101, 0},
        {13, 0x94022050, 0x940c20ca, 0x940e8101, 0},
        {14, 0x94022050, 0x940c20fa, 0x940e8101, 0}
#endif
};
#define	NUM_OF_2523_CHNL	(sizeof(RF2523RegTable) / sizeof(RTMP_RF_REGS))

RTMP_RF_REGS RF2524RegTable[] = {
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94032020, 0x94000c9e, 0x94000101, 0x94000a1b},
        {2,  0x94032020, 0x94000ca2, 0x94000101, 0x94000a1b},
        {3,  0x94032020, 0x94000ca6, 0x94000101, 0x94000a1b},
        {4,  0x94032020, 0x94000caa, 0x94000101, 0x94000a1b},
        {5,  0x94032020, 0x94000cae, 0x94000101, 0x94000a1b},
        {6,  0x94032020, 0x94000cb2, 0x94000101, 0x94000a1b},
        {7,  0x94032020, 0x94000cb6, 0x94000101, 0x94000a1b},
        {8,  0x94032020, 0x94000cba, 0x94000101, 0x94000a1b},
        {9,  0x94032020, 0x94000cbe, 0x94000101, 0x94000a1b},
        {10, 0x94032020, 0x94000d02, 0x94000101, 0x94000a1b},
        {11, 0x94032020, 0x94000d06, 0x94000101, 0x94000a1b},
        {12, 0x94032020, 0x94000d0a, 0x94000101, 0x94000a1b},
        {13, 0x94032020, 0x94000d0e, 0x94000101, 0x94000a1b},
        {14, 0x94032020, 0x94000d1a, 0x94000101, 0x94000a03}
};
#define	NUM_OF_2524_CHNL	(sizeof(RF2524RegTable) / sizeof(RTMP_RF_REGS))
            
RTMP_RF_REGS RF2525RegTable[] = {
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94022020, 0x94080c9e, 0x94060111, 0x94000a1b}, // {1,  0x94022010, 0x9408062e, 0x94060111, 0x94000a23}, 
        {2,  0x94022020, 0x94080ca2, 0x94060111, 0x94000a1b},
        {3,  0x94022020, 0x94080ca6, 0x94060111, 0x94000a1b},
        {4,  0x94022020, 0x94080caa, 0x94060111, 0x94000a1b},
        {5,  0x94022020, 0x94080cae, 0x94060111, 0x94000a1b},
        {6,  0x94022020, 0x94080cb2, 0x94060111, 0x94000a1b},
        {7,  0x94022020, 0x94080cb6, 0x94060111, 0x94000a1b},
        {8,  0x94022020, 0x94080cba, 0x94060111, 0x94000a1b},
        {9,  0x94022020, 0x94080cbe, 0x94060111, 0x94000a1b},
        {10, 0x94022020, 0x94080d02, 0x94060111, 0x94000a1b},
        {11, 0x94022020, 0x94080d06, 0x94060111, 0x94000a1b}, // {11, 0x94022010, 0x94080682, 0x94060111, 0x94000a23}, 
        {12, 0x94022020, 0x94080d0a, 0x94060111, 0x94000a1b},
        {13, 0x94022020, 0x94080d0e, 0x94060111, 0x94000a1b}, // {13, 0x94022010, 0x94080686, 0x94060111, 0x94000a23}, 
        {14, 0x94022020, 0x94080d1a, 0x94060111, 0x94000a03}
};
#define	NUM_OF_2525_CHNL	(sizeof(RF2525RegTable) / sizeof(RTMP_RF_REGS))

RTMP_RF_REGS RF2525HBOffsetRegTable[] = {
        {1,  0x94022020, 0x94080cbe, 0x94060111, 0x94000a1b},  
        {2,  0x94022020, 0x94080d02, 0x94060111, 0x94000a1b},
        {3,  0x94022020, 0x94080d06, 0x94060111, 0x94000a1b},
        {4,  0x94022020, 0x94080d0a, 0x94060111, 0x94000a1b},
        {5,  0x94022020, 0x94080d0e, 0x94060111, 0x94000a1b},
        {6,  0x94022020, 0x94080d12, 0x94060111, 0x94000a1b},
        {7,  0x94022020, 0x94080d16, 0x94060111, 0x94000a1b},
        {8,  0x94022020, 0x94080d1a, 0x94060111, 0x94000a1b},
        {9,  0x94022020, 0x94080d1e, 0x94060111, 0x94000a1b},
        {10, 0x94022020, 0x94080d22, 0x94060111, 0x94000a1b},
        {11, 0x94022020, 0x94080d26, 0x94060111, 0x94000a1b}, 
        {12, 0x94022020, 0x94080d2a, 0x94060111, 0x94000a1b},
        {13, 0x94022020, 0x94080d2e, 0x94060111, 0x94000a1b}, 
        {14, 0x94022020, 0x94080d3a, 0x94060111, 0x94000a03}
};

RTMP_RF_REGS RF2525eRegTable[] = {
#if 1
// using 5 Mhz reference clock
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94022020, 0x94081136, 0x94060111, 0x94000a0b},
        {2,  0x94022020, 0x9408113a, 0x94060111, 0x94000a0b},
        {3,  0x94022020, 0x9408113e, 0x94060111, 0x94000a0b},
        {4,  0x94022020, 0x94081182, 0x94060111, 0x94000a0b},
        {5,  0x94022020, 0x94081186, 0x94060111, 0x94000a0b},
        {6,  0x94022020, 0x9408118a, 0x94060111, 0x94000a0b},
        {7,  0x94022020, 0x9408118e, 0x94060111, 0x94000a0b},
        {8,  0x94022020, 0x94081192, 0x94060111, 0x94000a0b},
        {9,  0x94022020, 0x94081196, 0x94060111, 0x94000a0b},
        {10, 0x94022020, 0x9408119a, 0x94060111, 0x94000a0b},
        {11, 0x94022020, 0x9408119e, 0x94060111, 0x94000a0b}, 
        {12, 0x94022020, 0x940811a2, 0x94060111, 0x94000a0b},
        {13, 0x94022020, 0x940811a6, 0x94060111, 0x94000a0b},
        {14, 0x94022020, 0x940811ae, 0x94060111, 0x94000a1b}
#else
// using 10 Mhz reference clock
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94022010, 0x9408089a, 0x94060111, 0x94000a1b},
        {2,  0x94022010, 0x9408089e, 0x94060111, 0x94000a07},
        {3,  0x94022010, 0x9408089e, 0x94060111, 0x94000a1b},
        {4,  0x94022010, 0x940808a2, 0x94060111, 0x94000a07},
        {5,  0x94022010, 0x940808a2, 0x94060111, 0x94000a1b},
        {6,  0x94022010, 0x940808a6, 0x94060111, 0x94000a07},
        {7,  0x94022010, 0x940808a6, 0x94060111, 0x94000a1b},
        {8,  0x94022010, 0x940808aa, 0x94060111, 0x94000a07},
        {9,  0x94022010, 0x940808aa, 0x94060111, 0x94000a1b},
        {10, 0x94022010, 0x940808ae, 0x94060111, 0x94000a07},
        {11, 0x94022010, 0x940808ae, 0x94060111, 0x94000a1b}, 
        {12, 0x94022010, 0x940808b2, 0x94060111, 0x94000a07},
        {13, 0x94022010, 0x940808b2, 0x94060111, 0x94000a1b},
        {14, 0x94022010, 0x940808b6, 0x94060111, 0x94000a23}
#endif
};
#define	NUM_OF_2525E_CHNL	(sizeof(RF2525eRegTable) / sizeof(RTMP_RF_REGS))

RTMP_RF_REGS RF5222RegTable[] = {
//      ch   R1          R2          R3(TX0~4=0) R4
        {1,  0x94022020, 0x94001136, 0x94000101, 0x94000a0b},
        {2,  0x94022020, 0x9400113a, 0x94000101, 0x94000a0b},
        {3,  0x94022020, 0x9400113e, 0x94000101, 0x94000a0b},
        {4,  0x94022020, 0x94001182, 0x94000101, 0x94000a0b},
        {5,  0x94022020, 0x94001186, 0x94000101, 0x94000a0b},
        {6,  0x94022020, 0x9400118a, 0x94000101, 0x94000a0b},
        {7,  0x94022020, 0x9400118e, 0x94000101, 0x94000a0b},
        {8,  0x94022020, 0x94001192, 0x94000101, 0x94000a0b},
        {9,  0x94022020, 0x94001196, 0x94000101, 0x94000a0b},
        {10, 0x94022020, 0x9400119a, 0x94000101, 0x94000a0b},
        {11, 0x94022020, 0x9400119e, 0x94000101, 0x94000a0b},
        {12, 0x94022020, 0x940011a2, 0x94000101, 0x94000a0b},
        {13, 0x94022020, 0x940011a6, 0x94000101, 0x94000a0b},
        {14, 0x94022020, 0x940011ae, 0x94000101, 0x94000a1b},

        // still lack of MMAC(Japan) ch 34,38,42,46
        
        {36, 0x94022010, 0x94018896, 0x94000101, 0x94000a1f},
        {40, 0x94022010, 0x9401889a, 0x94000101, 0x94000a1f},
        {44, 0x94022010, 0x9401889e, 0x94000101, 0x94000a1f},
        {48, 0x94022010, 0x940188a2, 0x94000101, 0x94000a1f},
        {52, 0x94022010, 0x940188a6, 0x94000101, 0x94000a1f},
        {66, 0x94022010, 0x940188aa, 0x94000101, 0x94000a1f},
        {60, 0x94022010, 0x940188ae, 0x94000101, 0x94000a1f},
        {64, 0x94022010, 0x940188b2, 0x94000101, 0x94000a1f},
        
        {100, 0x94022010, 0x94008802, 0x94000101, 0x94000a0f},
        {104, 0x94022010, 0x94008806, 0x94000101, 0x94000a0f},
        {108, 0x94022010, 0x9400880a, 0x94000101, 0x94000a0f},
        {112, 0x94022010, 0x9400880e, 0x94000101, 0x94000a0f},
        {116, 0x94022010, 0x94008812, 0x94000101, 0x94000a0f},
        {120, 0x94022010, 0x94008816, 0x94000101, 0x94000a0f},
        {124, 0x94022010, 0x9400881a, 0x94000101, 0x94000a0f},
        {128, 0x94022010, 0x9400881e, 0x94000101, 0x94000a0f},
        {132, 0x94022010, 0x94008822, 0x94000101, 0x94000a0f},
        {136, 0x94022010, 0x94008826, 0x94000101, 0x94000a0f},
        {140, 0x94022010, 0x9400882a, 0x94000101, 0x94000a0f},
        
        {149, 0x94022020, 0x940090a6, 0x94000101, 0x94000a07},
        {153, 0x94022020, 0x940090ae, 0x94000101, 0x94000a07},
        {157, 0x94022020, 0x940090b6, 0x94000101, 0x94000a07},
        {161, 0x94022020, 0x940090be, 0x94000101, 0x94000a07}
};
#define	NUM_OF_5222_CHNL	(sizeof(RF5222RegTable) / sizeof(RTMP_RF_REGS))

/*
    ==========================================================================
    Description:
        initialize the MLME task and its data structure (queue, spinlock, 
        timer, state machines).
    Return:
        always return NDIS_STATUS_SUCCESS
    ==========================================================================
*/
NDIS_STATUS MlmeInit(
    IN PRTMP_ADAPTER pAd) 
{
    NDIS_STATUS Status = NDIS_STATUS_SUCCESS;

    if(RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_MLME_INITIALIZED))
    	return Status;

    DBGPRINT(RT_DEBUG_TRACE, "--> MLME Initialize\n");
    
    do 
    {
        Status = MlmeQueueInit(&pAd->Mlme.Queue);
        if(Status != NDIS_STATUS_SUCCESS) 
        {
            break;
        }

        pAd->Mlme.Running = FALSE;
        spin_lock_init(&pAd->Mlme.TaskLock);

        // initialize the two tables
        // MacTableInit(pAd);
        BssTableInit(&pAd->PortCfg.BssTab);

        // init state machines
        ASSERT(ASSOC_FUNC_SIZE == MAX_ASSOC_MSG * MAX_ASSOC_STATE);
        AssocStateMachineInit(pAd, &pAd->Mlme.AssocMachine, pAd->Mlme.AssocFunc);
        
        ASSERT(AUTH_FUNC_SIZE == MAX_AUTH_MSG * MAX_AUTH_STATE);
        AuthStateMachineInit(pAd, &pAd->Mlme.AuthMachine, pAd->Mlme.AuthFunc);
        
        ASSERT(AUTH_RSP_FUNC_SIZE == MAX_AUTH_RSP_MSG * MAX_AUTH_RSP_STATE);
        AuthRspStateMachineInit(pAd, &pAd->Mlme.AuthRspMachine, pAd->Mlme.AuthRspFunc);

        ASSERT(SYNC_FUNC_SIZE == MAX_SYNC_MSG * MAX_SYNC_STATE);
        SyncStateMachineInit(pAd, &pAd->Mlme.SyncMachine, pAd->Mlme.SyncFunc);

		ASSERT(WPA_PSK_FUNC_SIZE == MAX_WPA_PSK_MSG * MAX_WPA_PSK_STATE);
        WpaPskStateMachineInit(pAd,&pAd->Mlme.WpaPskMachine,pAd->Mlme.WpaPskFunc);
		
        // Since we are using switch/case to implement it, the init is different from the above 
        // state machine init
        MlmeCntlInit(pAd, &pAd->Mlme.CntlMachine, NULL);

        // Init mlme periodic timer
        RTMPInitTimer(pAd, &pAd->Mlme.PeriodicTimer, MlmePeriodicExec);
        // Set mlme periodic timer
        RTMPSetTimer(pAd, &pAd->Mlme.PeriodicTimer, MLME_TASK_EXEC_INTV);

        RTMPInitTimer(pAd, &pAd->PortCfg.LedCntl.BlinkTimer, AsicLedPeriodicExec);
        if (pAd->PortCfg.LedMode == LED_MODE_TXRX_ACTIVITY)
        {
            // Set blink timer
            RTMPSetTimer(pAd, &pAd->PortCfg.LedCntl.BlinkTimer, 70);
        }

        // software-based RX Antenna diversity
        RTMPInitTimer(pAd, &pAd->PortCfg.RxAnt.RxAntDiversityTimer, AsicRxAntEvalTimeout);
    } while (FALSE);

    RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_MLME_INITIALIZED);
   
    DBGPRINT(RT_DEBUG_TRACE, "<-- MLME Initialize\n");

    return Status;
}


/*
    ==========================================================================
    Description:
        main loop of the MLME
    Pre:
        Mlme has to be initialized, and there are something inside the queue
    Note:
        This function is invoked from MPSetInformation and MPReceive;
        This task guarantee only one MlmeHandler will run. 
    ==========================================================================
 */
VOID MlmeHandler(
    IN PRTMP_ADAPTER pAd) 
{
    MLME_QUEUE_ELEM        *Elem = NULL;

    // Only accept MLME and Frame from peer side, no other (control/data) frame should
    // get into this state machine

    spin_lock(&pAd->Mlme.TaskLock);
    if(pAd->Mlme.Running) 
    {
        spin_unlock(&pAd->Mlme.TaskLock);
        return;
    } 
    else 
    {
        pAd->Mlme.Running = TRUE;
    }
    spin_unlock(&pAd->Mlme.TaskLock);

    while (!MlmeQueueEmpty(&pAd->Mlme.Queue)) 
    {
        //From message type, determine which state machine I should drive
        if (MlmeDequeue(&pAd->Mlme.Queue, &Elem) && pAd->PortCfg.BssType != BSS_MONITOR) 
        {
            // if dequeue success
            switch (Elem->Machine) 
            {
                case ASSOC_STATE_MACHINE:
                    StateMachinePerformAction(pAd, &pAd->Mlme.AssocMachine, Elem);
                    break;
                case AUTH_STATE_MACHINE:
                    StateMachinePerformAction(pAd, &pAd->Mlme.AuthMachine, Elem);
                    break;
                case AUTH_RSP_STATE_MACHINE:
                    StateMachinePerformAction(pAd, &pAd->Mlme.AuthRspMachine, Elem);
                    break;
                case SYNC_STATE_MACHINE:
                    StateMachinePerformAction(pAd, &pAd->Mlme.SyncMachine, Elem);
                    break;
                case MLME_CNTL_STATE_MACHINE:
                    MlmeCntlMachinePerformAction(pAd, &pAd->Mlme.CntlMachine, Elem);
                    break;
                case WPA_PSK_STATE_MACHINE:
                    StateMachinePerformAction(pAd, &pAd->Mlme.WpaPskMachine, Elem);
                    break;
                default:
                    DBGPRINT(RT_DEBUG_TRACE, "ERROR: Illegal machine in MlmeHandler()\n");
                    break;
            } // end of switch

            // free MLME element
            Elem->Occupied = FALSE;
            Elem->MsgLen = 0;
            
        }
        else
        {
            printk(KERN_ERR DRV_NAME "ERROR: empty Elem in MlmeQueue\n");
        }
    }

    spin_lock(&pAd->Mlme.TaskLock);
    pAd->Mlme.Running = FALSE;
    spin_unlock(&pAd->Mlme.TaskLock);
}

/*
    ==========================================================================
    Description:
        Destructor of MLME (Destroy queue, state machine, spin lock and timer)
    Parameters:
        Adapter - NIC Adapter pointer
    Post:
        The MLME task will no longer work properly
    ==========================================================================
 */
VOID MlmeHalt(
    IN PRTMP_ADAPTER pAd) 
{
    MLME_DISASSOC_REQ_STRUCT DisReq;
    MLME_QUEUE_ELEM          MsgElem;

    if(!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_MLME_INITIALIZED))
      return;
   
    DBGPRINT(RT_DEBUG_TRACE, "==> MlmeHalt\n");
    
    if (INFRA_ON(pAd) && !RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST)) 
    {
        COPY_MAC_ADDR(&DisReq.Addr, &pAd->PortCfg.Bssid);
        DisReq.Reason =  REASON_DISASSOC_STA_LEAVING;

        MsgElem.Machine = ASSOC_STATE_MACHINE;
        MsgElem.MsgType = MT2_MLME_DISASSOC_REQ;
        MsgElem.MsgLen = sizeof(MLME_DISASSOC_REQ_STRUCT);
        memcpy(MsgElem.Msg, &DisReq, sizeof(MLME_DISASSOC_REQ_STRUCT));

        MlmeDisassocReqAction(pAd, &MsgElem);

        udelay(1000);
    }

	if (!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST))
	{
    	// disable BEACON generation and other BEACON related hardware timers
    	AsicDisableSync(pAd);
	}
    
    // Cancel pending timers
    RTMPCancelTimer(&pAd->Mlme.AssocAux.AssocTimer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.ReassocTimer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.DisassocTimer);
    RTMPCancelTimer(&pAd->Mlme.AuthAux.AuthTimer);
    RTMPCancelTimer(&pAd->Mlme.AuthRspAux.AuthRspTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.ScanTimer);
    RTMPCancelTimer(&pAd->Mlme.PeriodicTimer);
    // RTMPCancelTimer(&pAd->PortCfg.MacTab.AgedOutTimer);

	if (!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST))
	{
	RTMPCancelTimer(&pAd->PortCfg.LedCntl.BlinkTimer);
    	ASIC_LED_ACT_OFF(pAd);
	}

    RTMPCancelTimer(&pAd->PortCfg.RxAnt.RxAntDiversityTimer);
    udelay(1000);
    
    MlmeQueueDestroy(&pAd->Mlme.Queue);
    StateMachineDestroy(&pAd->Mlme.AssocMachine);
    StateMachineDestroy(&pAd->Mlme.AuthMachine);
    StateMachineDestroy(&pAd->Mlme.AuthRspMachine);
    StateMachineDestroy(&pAd->Mlme.SyncMachine);
    //    StateMachineDestroy(&pAd->Mlme.CntlMachine);
    //NdisFreeSpinLock(&pAd->Mlme.Queue.Lock);
    //NdisFreeSpinLock(&pAd->Mlme.TaskLock);
    // NdisFreeSpinLock(&pAd->PortCfg.MacTab.Lock);
  
    MlmeFreeMemoryHandler(pAd); //Free MLME memory handler

    RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_MLME_INITIALIZED);
   
	DBGPRINT(RT_DEBUG_TRACE, "<== MlmeHalt\n");
}

/*
    ==========================================================================
    Description:
        This routine is executed periodically to -
        1. Decide if it's a right time to turn on PwrMgmt bit of all 
           outgoiing frames
        2. Calculate ChannelQuality based on statistics of the last
           period, so that TX rate won't toggling very frequently between a 
           successful TX and a failed TX.
        3. If the calculated ChannelQuality indicated current connection not 
           healthy, then a ROAMing attempt is tried here.
    ==========================================================================
 */
#define ADHOC_BEACON_LOST_TIME      (10*HZ)  // 4 sec
VOID MlmePeriodicExec(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    ULONG Now32;
    CSR15_STRUC Csr15;

    if (pAd->PortCfg.BssType == BSS_MONITOR)
    {
        RTMPSetTimer(pAd, &pAd->Mlme.PeriodicTimer, MLME_TASK_EXEC_INTV);
        return;
    }

	if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF))
	{
	    RTMPSetTimer(pAd, &pAd->Mlme.PeriodicTimer, MLME_TASK_EXEC_INTV);
		return;
	}

	if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_RESET_IN_PROGRESS))
	{
	    RTMPSetTimer(pAd, &pAd->Mlme.PeriodicTimer, MLME_TASK_EXEC_INTV);
		return;
	}

	// check every 2 second. If rcv-beacon less than 5 in the past 2 second, then AvgRSSI is no longer a 
    // valid indication of the distance between this AP and its clients.
    if (pAd->MediaState == NdisMediaStateConnected)
    {
        if (pAd->PortCfg.NumOfAvgRssiSample < 3)
        {
        	pAd->PortCfg.RxAnt.AvgRssi[0] = (-95 + 120) << 3;  // reset Ant-A's RSSI history
		    pAd->PortCfg.RxAnt.AvgRssi[1] = (-95 + 120) << 3;  // reset Ant-B's RSSI history
            pAd->PortCfg.AvgRssi = pAd->PortCfg.LastR17Value;
            DBGPRINT(RT_DEBUG_TRACE, "MlmePeriodicExec: no traffic, reset Avg RSSI= %d dbm\n", pAd->PortCfg.AvgRssi);
        }
        else
            pAd->PortCfg.NumOfAvgRssiSample = 0;
    }
	
    Now32 = jiffies;

	if (pAd->RalinkCounters.MgmtRingFullCount >= 2)
	{
		RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_HARDWARE_ERROR);
	}
	else
	{
		pAd->RalinkCounters.MgmtRingFullCount = 0;
	}
    
	if ((pAd->PortCfg.bBlockAssoc == TRUE) && (pAd->PortCfg.LastMicErrorTime + (60 * HZ) < Now32))
	{
		pAd->PortCfg.bBlockAssoc = FALSE;
	}

    // if Rx Antenna is DIVERSITY ON, then perform Software-based diversity evaluation
	if ((pAd->PortCfg.CurrentRxAntenna == 0xff) && (pAd->Mlme.PeriodicRound % 2 == 1))
	{
		SHORT	realavgrssi = (pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.PrimaryRxAnt] >> 3)  - pAd->PortCfg.RssiToDbm;
		DBGPRINT(RT_DEBUG_TRACE, "MlmePeriodicExec:(%d), Primary AvgRssi(%d), LastAvgRssi(%d)\n", pAd->PortCfg.RxAnt.PrimaryRxAnt, realavgrssi, pAd->PortCfg.LastAvgRssi);
		DBGPRINT(RT_DEBUG_TRACE, "Primary AvgRssi(%d), Second AvgRssi(%d)\n", pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.PrimaryRxAnt], pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.SecondaryRxAnt]);
		if ((realavgrssi > pAd->PortCfg.LastAvgRssi + 5) || (realavgrssi < pAd->PortCfg.LastAvgRssi - 5))
		{
			//DBGPRINT(RT_DEBUG_TRACE, ("AsicEvaluateSecondaryRxAnt ===> start evaluate second antenna!!!\n"));
			pAd->PortCfg.LastAvgRssi = realavgrssi;
			AsicEvaluateSecondaryRxAnt(pAd);
		}
	}


#ifndef	WIFI_TEST        
    // danamic tune BBP R17 to find a balance between sensibility and noise isolation
    // 2003-12-05 For 2560C and before, to avoid collision with MAC ASIC, limit 
    //   BBP R17 tuning to be within 20 seconds after LINK UP. 2560D (R0=4) and
    //   after can always enable R17 tuning 
    if (pAd->PortCfg.Rt2560Version >= RT2560_VER_D)
        AsicBbpTuning(pAd);
    else if ((pAd->MediaState == NdisMediaStateConnected) && (pAd->Mlme.PeriodicRound <= 20))
        AsicBbpTuning(pAd);
#endif

    if (pAd->MediaState == NdisMediaStateConnected)
    {
        // update channel quality for Roaming and UI LinkQuality display
      	MlmeCheckChannelQuality(pAd, Now32);
#if 0
        // periodic VCO tuning when there's no traffic.
        // RF guys suspected VCO will shift away upon temperature change along the time
        if (((pAd->Mlme.PeriodicRound % 16) == 2) &&
            ((pAd->DrsCounters.OneSecTxOkCount + pAd->DrsCounters.OneSecTxRetryOkCount)==0))
        {
            DBGPRINT(RT_DEBUG_TRACE,("Periodic VCO tuning...\n"));
            AsicSwitchChannel(pAd, pAd->PortCfg.Channel);
            AsicLockChannel(pAd, pAd->PortCfg.Channel);
        }
#endif
        // perform dynamic tx rate switching based on past TX history
        MlmeCheckDynamicTxRateSwitching(pAd);
    }

    AsicAdjustTxPower(pAd);

    if (INFRA_ON(pAd))
    {
    	// Is PSM bit consistent with user power management policy?
    	// This is the only place that will set PSM bit ON.
      	MlmeCheckForPsmChange(pAd, Now32);

		// Check for EAPOL frame sent after MIC countermeasures
		if (pAd->PortCfg.MicErrCnt >= 3)
		{
			MLME_DISASSOC_REQ_STRUCT	DisassocReq;
			
            // disassoc from current AP first
        	DBGPRINT(RT_DEBUG_TRACE, "MLME - disassociate with current AP after sending second continuous EAPOL frame\n");
            DisassocParmFill(pAd, &DisassocReq, &pAd->PortCfg.Bssid, REASON_MIC_FAILURE);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_DISASSOC_REQ, 
                        sizeof(MLME_DISASSOC_REQ_STRUCT), &DisassocReq);

            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_DISASSOC;
			pAd->PortCfg.bBlockAssoc = TRUE;
		}
		
        else 
        {
            // send out a NULL frame every 10 sec. for what??? inform "PwrMgmt" bit?
            if ((pAd->Mlme.PeriodicRound % 10) == 8)
                EnqueueNullFrame(pAd, pAd->PortCfg.TxRate);
    
       		if (CQI_IS_BAD(pAd->Mlme.ChannelQuality))
       		{
           		pAd->RalinkCounters.BadCQIAutoRecoveryCount ++;
            	DBGPRINT(RT_DEBUG_TRACE, "MMCHK - Bad CQI. Auto Recovery attempt #%d\n", pAd->RalinkCounters.BadCQIAutoRecoveryCount);
   	        	MlmeAutoReconnectLastSSID(pAd);
       		}

       		else if (CQI_IS_FAIR(pAd->Mlme.ChannelQuality) || CQI_IS_POOR(pAd->Mlme.ChannelQuality))
        	{
   	        	// perform aggresive roaming only when SECURITY OFF or WEP64/128;
   	        	// WPA and WPA-PSK has no aggresive roaming because re-negotiation 
   	        	// between 802.1x supplicant and authenticator/AAA server is required
   	        	// but can't be guaranteed.
   	        	if (pAd->PortCfg.AuthMode < Ndis802_11AuthModeWPA)
       	    	    MlmeCheckForRoaming(pAd, Now32);
        	}
		}
    }
    else if (ADHOC_ON(pAd))
    {
        if ((pAd->Mlme.PeriodicRound % 2) == 1)
        {
            // So that even when ASIC's BEACONgen engine been blocked
            // by peer's BEACON due to slower system clock, this STA still can send out
            // minimum BEACON to tell the peer I'm alive.
            // drawback is that this BEACON won't well align at TBTT boundary.
            RTMP_IO_READ32(pAd, CSR15, &Csr15.word);  // read-n-clear "BcnSent" bit
            if (Csr15.field.BeaconSent == 0)  
                EnqueueBeaconFrame(pAd);              // software send BEACON
        }
        else
        {
            // if all 11b peers leave this BSS more than 5 seconds, update Tx rate
            if ((pAd->PortCfg.Channel <= 14)             &&
                (pAd->PortCfg.MaxTxRate <= RATE_11)      &&
                (pAd->PortCfg.MaxDesiredRate > RATE_11)  &&
                ((pAd->PortCfg.Last11bBeaconRxTime + (5 * HZ)) < Now32))
            {
                DBGPRINT(RT_DEBUG_TRACE, "last 11B peer left, update Tx rates\n"); 
                memcpy(pAd->PortCfg.SupportedRates, pAd->PortCfg.IbssConfig.SupportedRates, MAX_LEN_OF_SUPPORTED_RATES);
                pAd->PortCfg.SupportedRatesLen = pAd->PortCfg.IbssConfig.SupportedRatesLen;
                MlmeUpdateTxRates(pAd, FALSE);
                MakeIbssBeacon(pAd);    // supported rates changed
            }
        }
        
#ifndef	SINGLE_ADHOC_LINKUP
        // If all peers leave, and this STA becomes the last one in this IBSS, then change MediaState
        // to DISCONNECTED. But still holding this IBSS (i.e. sending BEACON) so that other STAs can
        // join later.
        if ((pAd->PortCfg.LastBeaconRxTime + ADHOC_BEACON_LOST_TIME < Now32) &&
            (pAd->MediaState == NdisMediaStateConnected))
        {
            DBGPRINT(RT_DEBUG_TRACE, "MMCHK - excessive BEACON lost, last STA in this IBSS, MediaState=Disconnected\n"); 

            pAd->MediaState = NdisMediaStateDisconnected;
			// clean up previous SCAN result, add current BSS back to table if any
			BssTableDeleteEntry(&pAd->PortCfg.BssTab, &(pAd->PortCfg.Bssid));

			pAd->PortCfg.LastScanTime = Now32;
        }
#endif

    }
    else
    {
		DBGPRINT(RT_DEBUG_INFO, "MLME periodic exec, no association so far\n");
		if (pAd->PortCfg.AutoReconnect == TRUE)
		{
			if ((pAd->PortCfg.BssTab.BssNr==0) && (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE))
			{
				MLME_SCAN_REQ_STRUCT	   ScanReq;
			
				if ((pAd->PortCfg.LastScanTime + 10 * HZ) < Now32)
				{
					DBGPRINT(RT_DEBUG_TRACE, "CNTL - No matching BSS, start a new scan\n");
					// BroadSsid[0] = '\0';
					ScanParmFill(pAd, &ScanReq, pAd->Mlme.CntlAux.Ssid, pAd->Mlme.CntlAux.SsidLen, BSS_ANY, SCAN_ACTIVE);
					MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_SCAN_REQ, sizeof(MLME_SCAN_REQ_STRUCT), &ScanReq);
					pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_OID_LIST_SCAN;
					// Reset Missed scan number
					pAd->PortCfg.IgnoredScanNumber = 0;
					pAd->PortCfg.LastScanTime = Now32;
				}
				else if (pAd->PortCfg.BssType == BSS_INDEP)	// Quit the forever scan when in a very clean room
					MlmeAutoRecoverNetwork(pAd);
					//MlmeAutoReconnectLastSSID(pAd);					
			}
			else if (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE)
			{
				if ((pAd->Mlme.PeriodicRound % 10) == 7)
				{
					if ((pAd->PortCfg.LastScanTime + 10 * HZ) < Now32)
					{
						MlmeAutoScan(pAd);
						pAd->PortCfg.LastScanTime = Now32;
					}
				}
				else
					MlmeAutoReconnectLastSSID(pAd);
				
				DBGPRINT(RT_DEBUG_INFO, "pAd->PortCfg.AutoReconnect is TRUE\n");
			}
		}
	}
    pAd->Mlme.PeriodicRound ++;
	MlmeHandler(pAd);

    if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_ACTIVE))
	NICCheckForHang(pAd);

	RTMPSetTimer(pAd, &pAd->Mlme.PeriodicTimer, MLME_TASK_EXEC_INTV);
}
	
VOID MlmeAutoScan(
    IN PRTMP_ADAPTER pAd)
{
    // check CntlMachine.CurrState to avoid collision with NDIS SetOID request
    if (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE)
    {
        DBGPRINT(RT_DEBUG_TRACE, "MMCHK - Driver auto scan\n");

        // tell CNTL state machine NOT to call NdisMSetInformationComplete() after completing
        // this request, because this request is initiated by driver itself.
        pAd->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 
                    
        MlmeEnqueue(&pAd->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_BSSID_LIST_SCAN, 
                    0, 
                    NULL);
        MlmeHandler(pAd);
    }
}
	
VOID MlmeAutoRecoverNetwork(
    IN PRTMP_ADAPTER pAd)
{
    // check CntlMachine.CurrState to avoid collision with NDIS SetOID request
    if (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE)
    {
        NDIS_802_11_SSID OidSsid;
        OidSsid.SsidLength = pAd->PortCfg.SsidLen;
        memcpy(OidSsid.Ssid, pAd->PortCfg.Ssid, pAd->PortCfg.SsidLen);

        DBGPRINT(RT_DEBUG_TRACE, "MMCHK - Driver auto recovering network - %s\n", pAd->PortCfg.Ssid);

        // tell CNTL state machine NOT to call NdisMSetInformationComplete() after completing
        // this request, because this request is initiated by driver itself.
        pAd->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 
                    
        MlmeEnqueue(&pAd->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_SSID, 
                    sizeof(NDIS_802_11_SSID), 
                    &OidSsid);
        MlmeHandler(pAd);
    }

}
    
VOID MlmeAutoReconnectLastSSID(
    IN PRTMP_ADAPTER pAd)
{
    // check CntlMachine.CurrState to avoid collision with NDIS SetOID request
    if (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE)
    {
        NDIS_802_11_SSID OidSsid;
        OidSsid.SsidLength = pAd->Mlme.CntlAux.SsidLen;
        memcpy(OidSsid.Ssid, pAd->Mlme.CntlAux.Ssid, pAd->Mlme.CntlAux.SsidLen);

        DBGPRINT(RT_DEBUG_TRACE, "Driver auto reconnect to last OID_802_11_SSID setting - %s\n", pAd->Mlme.CntlAux.Ssid);

		// We will only try this attemp once, therefore change the AutoReconnect flag afterwards.
        pAd->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 
                    
        MlmeEnqueue(&pAd->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_SSID, 
                    sizeof(NDIS_802_11_SSID), 
                    &OidSsid);
        MlmeHandler(pAd);
    }
}

/*
    ==========================================================================
    Description:
        This routine checks if there're other APs out there capable for
        roaming. Caller should call this routine only when Massoc=TRUE and
        channel quality is below CQI_GOOD_THRESHOLD.
    Output:
    ==========================================================================
 */
VOID MlmeCheckForRoaming(
    IN PRTMP_ADAPTER pAd,
    IN ULONG    Now32)
{
    USHORT     i;
    BSS_TABLE  *pBssTab = &pAd->Mlme.CntlAux.SsidBssTab;
    BSS_TABLE  *pRoamTab = &pAd->Mlme.CntlAux.RoamTab;
    BSS_ENTRY  *pBss;

    // put all roaming candidates into RoamTab, and sort in RSSI order
    BssTableInit(pRoamTab);
    for (i = 0; i < pBssTab->BssNr; i++)
    {
        pBss = &pBssTab->BssEntry[i];
        
        if ((pBssTab->BssEntry[i].LastBeaconRxTime + BEACON_LOST_TIME) < Now32) 
            continue;    // AP disappear
        if (pBss->Rssi <= RSSI_THRESHOLD_FOR_ROAMING)
            continue;    // RSSI too weak. forget it.
        if (MAC_ADDR_EQUAL(&pBssTab->BssEntry[i].Bssid, &pAd->PortCfg.Bssid))
            continue;    // skip current AP
        if (CQI_IS_FAIR(pAd->Mlme.ChannelQuality) && (pAd->PortCfg.LastRssi + RSSI_DELTA > pBss->Rssi)) 
            continue;    // we're still okay, only AP with stronger RSSI is eligible for roaming

        // AP passing all above rules is put into roaming candidate table        
        memcpy(&pRoamTab->BssEntry[pRoamTab->BssNr], pBss, sizeof(BSS_ENTRY));
        pRoamTab->BssNr += 1;
    }

    if (pRoamTab->BssNr > 0)
    {
        // check CntlMachine.CurrState to avoid collision with NDIS SetOID request
        if (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE)
        {
            // tell CNTL state machine NOT to call NdisMSetInformationComplete() after completing
            // this request, because this request is initiated by driver itself, not from NDIS.
            pAd->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 
        
          	pAd->RalinkCounters.PoorCQIRoamingCount ++;
            DBGPRINT(RT_DEBUG_TRACE, "MMCHK - Roaming attempt #%d\n", pAd->RalinkCounters.PoorCQIRoamingCount);
            MlmeEnqueue(&pAd->Mlme.Queue, MLME_CNTL_STATE_MACHINE, MT2_MLME_ROAMING_REQ, 0, NULL);
            MlmeHandler(pAd);
        }
    }
    
}

/*
    ==========================================================================
    Description:
        This routine calculates TxPER, RxPER of the past N-sec period. And 
        according to the calculation result, ChannelQuality is calculated here 
        to decide if current AP is still doing the job. 

        If ChannelQuality is not good, a ROAMing attempt may be tried later.
    Output:
        PortCfg.ChannelQuality - 0..100
    ==========================================================================
 */
VOID MlmeCheckChannelQuality(
    IN PRTMP_ADAPTER pAd,
    IN ULONG Now32)
{
    ULONG TxFailCnt, TxOkCnt, TxRetryCnt, TxCnt;
    ULONG RxFailCnt, RxOkCnt, RxCnt, Cnt0, OldFcsCount;
    static ULONG TxPER = 0, TxPRR = 0, RxPER = 0;

    //
    // monitor TX counters change for the past period
    //
    TxFailCnt     = pAd->WlanCounters.FailedCount.vv.LowPart - 
                    pAd->Mlme.PrevWlanCounters.FailedCount.vv.LowPart;
    TxRetryCnt    = pAd->WlanCounters.RetryCount.vv.LowPart - 
                    pAd->Mlme.PrevWlanCounters.RetryCount.vv.LowPart;
    TxOkCnt       = pAd->WlanCounters.TransmittedFragmentCount.vv.LowPart - 
                    pAd->Mlme.PrevWlanCounters.TransmittedFragmentCount.vv.LowPart;
    TxCnt = TxOkCnt + TxFailCnt;

    if (TxCnt > 5) // if too few TX samples, skip TX related statistics
    {
        TxPER = (TxFailCnt * 100) / TxCnt;
        TxPRR = ((TxRetryCnt + TxFailCnt) * 100) / TxCnt;
    }

    //
    // calculate RX PER
    //

    // Update FCS counters
    RTMP_IO_READ32(pAd, CNT0, &Cnt0);
    OldFcsCount= pAd->WlanCounters.FCSErrorCount.vv.LowPart;
    pAd->WlanCounters.FCSErrorCount.vv.LowPart += ((Cnt0 & 0x0000ffff) >> 7);
    if (pAd->WlanCounters.FCSErrorCount.vv.LowPart < OldFcsCount)
       	pAd->WlanCounters.FCSErrorCount.vv.HighPart++;
            
    // Add FCS error count to private counters
    OldFcsCount = pAd->RalinkCounters.RealFcsErrCount.vv.LowPart;
    pAd->RalinkCounters.RealFcsErrCount.vv.LowPart += Cnt0;
    if (pAd->RalinkCounters.RealFcsErrCount.vv.LowPart < OldFcsCount)
    	pAd->RalinkCounters.RealFcsErrCount.vv.HighPart++;
	
    RxOkCnt   = pAd->WlanCounters.ReceivedFragmentCount.vv.LowPart - 
                pAd->Mlme.PrevWlanCounters.ReceivedFragmentCount.vv.LowPart;
    RxFailCnt = pAd->RalinkCounters.RealFcsErrCount.vv.LowPart - 
                pAd->Mlme.PrevWlanCounters.FCSErrorCount.vv.LowPart;
    RxCnt = RxOkCnt + RxFailCnt;

    if (RxCnt > 5)
        RxPER = (RxFailCnt * 100) / RxCnt;
//printk("!! WiFi: Ok: %d, Fail: %d, PER: %d\n", RxOkCnt, RxFailCnt, RxPER);
    //
    // decide ChannelQuality based on: 1)last BEACON received time, 2)last RSSI, 3)TxPER, and 4)RxPER
    //
    // This value also decides when all roaming fails (or no roaming candidates at 
    // all), should this STA stay with original AP, or a LinkDown signal 
    // is indicated to NDIS
    //
    if (INFRA_ON(pAd) &&
        (pAd->PortCfg.LastBeaconRxTime + BEACON_LOST_TIME < Now32))  // BEACON starving?
    {
    	// Ignore lost beacon when NIC in reset state
    	// Ignore lost beacon if traffic still goes well
    	if (!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_RESET_IN_PROGRESS) && (TxOkCnt < 2))
    	{
        	DBGPRINT(RT_DEBUG_TRACE, "BEACON lost for more than %d sec with TxOkCnt=%d, let CQI = 0\n", BEACON_LOST_TIME/HZ, TxOkCnt); 
        	pAd->Mlme.ChannelQuality = 0;
        	// Lost AP, send disconnect & link down event
			LinkDown(pAd);
    	}
    }
    else
    {
        // ChannelQuality = W1*RSSI + W2*TxPRR + W3*RxPER    (RSSI 0..100), (TxPER 100..0), (RxPER 100..0)
        pAd->Mlme.ChannelQuality = (RSSI_WEIGHTING * pAd->PortCfg.LastRssi + 
                             TX_WEIGHTING * (100 - TxPRR) + 
                             RX_WEIGHTING* (100 - RxPER)) / 100;
        if (pAd->Mlme.ChannelQuality >= 100)
            pAd->Mlme.ChannelQuality = 100;
    }
    
    // latch current WLAN counters for next check-for-roaming usage
    memcpy(&pAd->Mlme.PrevWlanCounters, &pAd->WlanCounters, sizeof(COUNTER_802_11));
	// make sure copy the real FCS counts into previous mlme counter structure.
	pAd->Mlme.PrevWlanCounters.FCSErrorCount = pAd->RalinkCounters.RealFcsErrCount;
	
    DBGPRINT(RT_DEBUG_INFO, "MMCHK - CQI= %d, (Tx Fail=%d/Retry=%d/Total=%d, Rx Fail=%d/Total=%d, RSSI=%d dbm)\n", 
    pAd->Mlme.ChannelQuality, TxFailCnt, TxRetryCnt, TxCnt, RxFailCnt, RxCnt, pAd->PortCfg.LastRssi - pAd->PortCfg.RssiToDbm);

}

/*
    ==========================================================================
    Description:
        This routine calculates the acumulated TxPER of eaxh TxRate. And 
        according to the calculation result, change PortCfg.TxRate which 
        is the stable TX Rate we expect the Radio situation could sustained. 

        PortCfg.TxRate will change dynamically within {RATE_1/RATE_6, MaxTxRate} 
    Output:
        PortCfg.TxRate - 
    NOTE:
        call this routine every second
    ==========================================================================
 */
VOID MlmeCheckDynamicTxRateSwitching(
    IN PRTMP_ADAPTER pAd)
{
    UCHAR UpRate, DownRate, CurrRate;
    USHORT TxTotalCnt   = pAd->DrsCounters.OneSecTxOkCount + pAd->DrsCounters.OneSecTxRetryOkCount + pAd->DrsCounters.OneSecTxFailCount;
    USHORT TxErrorRatio;
    BOOLEAN fUpgradeQuality = FALSE;
    USHORT  *pRateUpPER, *pRateDownPER;

    pAd->DrsCounters.CurrTxRateStableTime ++;
    CurrRate = pAd->PortCfg.TxRate;
    do
    {
        if (pAd->PortCfg.EnableAutoRateSwitching == FALSE)
            break;
            
        // if no traffic in the past 1-sec period, don't change TX rate,
        // but clear all bad history. because the bad history may affect the next 
        // Chariot throughput test
        if (TxTotalCnt == 0)
        {
            pAd->DrsCounters.TxRateUpPenalty = 0;
            memset(pAd->DrsCounters.TxQuality, 0, MAX_LEN_OF_SUPPORTED_RATES);
            memset(pAd->DrsCounters.PER, 0, MAX_LEN_OF_SUPPORTED_RATES);
            break;
        }
        
        // decide the next upgrade rate and downgrade rate, if any
        if (pAd->PortCfg.PhyMode == PHY_11BG_MIXED)
        {
            UpRate = Phy11BGNextRateUpward[CurrRate];
            DownRate = Phy11BGNextRateDownward[CurrRate];
        }
        else if (pAd->PortCfg.PhyMode == PHY_11B)
        {
            UpRate = Phy11BNextRateUpward[CurrRate];
            DownRate = Phy11BNextRateDownward[CurrRate];
        }
        else if (pAd->PortCfg.PhyMode == PHY_11A)
        {
            UpRate = Phy11ANextRateUpward[CurrRate];
            DownRate = Phy11ANextRateDownward[CurrRate];
        }
        else // PHY_11ABG_MIXED
        {
            if (pAd->PortCfg.Channel > 14)  
            {
                UpRate = Phy11ANextRateUpward[CurrRate];
                DownRate = Phy11ANextRateDownward[CurrRate];
            }
            else
            {
                UpRate = Phy11BGNextRateUpward[CurrRate];
                DownRate = Phy11BGNextRateDownward[CurrRate];
            }
        }

        if (UpRate > pAd->PortCfg.MaxTxRate)
            UpRate = pAd->PortCfg.MaxTxRate;

        // decide TX quality based on Tx PER when enough samples are available
        if (TxTotalCnt > 15)
        {
            TxErrorRatio = ((pAd->DrsCounters.OneSecTxRetryOkCount + pAd->DrsCounters.OneSecTxFailCount) *100) / TxTotalCnt;
           
            // 2560D and after has implemented ASIC-based OFDM rate switching,
            // but not 2560C & before. thus software use different PER for rate switching
            if (pAd->PortCfg.Rt2560Version >= RT2560_VER_D)
            {
                pRateUpPER = &NewRateUpPER[0];
                pRateDownPER = &NewRateDownPER[0];
            }
            else
            {
                pRateUpPER = &OldRateUpPER[0];
                pRateDownPER = &OldRateDownPER[0];
            }

            // downgrade TX quality if PER >= Rate-Down threshold
            if (TxErrorRatio >= pRateDownPER[CurrRate])
            {
                pAd->DrsCounters.TxQuality[CurrRate] = DRS_TX_QUALITY_WORST_BOUND;
            }
            // upgrade TX quality if PER <= Rate-Up threshold
            else if (TxErrorRatio <= pRateUpPER[CurrRate])
            {
                fUpgradeQuality = TRUE;
                if (pAd->DrsCounters.TxQuality[CurrRate])
                    pAd->DrsCounters.TxQuality[CurrRate] --;  // quality very good in CurrRate
                    
                if (pAd->DrsCounters.TxRateUpPenalty)
                    pAd->DrsCounters.TxRateUpPenalty --;
                else if (pAd->DrsCounters.TxQuality[UpRate])
                    pAd->DrsCounters.TxQuality[UpRate] --;    // may improve next UP rate's quality
            }
            
        }
        
        // if not enough TX samples, decide by heuristic rules
        else
        {
            TxErrorRatio = 0;
            
            // Downgrade TX quality upon any TX failure in the past second
            if (pAd->DrsCounters.OneSecTxFailCount)
            {
                if ((pAd->DrsCounters.OneSecTxFailCount <= 1) &&
                    (pAd->DrsCounters.OneSecTxOkCount + pAd->DrsCounters.OneSecTxRetryOkCount))
                {
                    pAd->DrsCounters.TxQuality[CurrRate] += 2;   // degrade quality
                    if (pAd->DrsCounters.TxQuality[CurrRate] > DRS_TX_QUALITY_WORST_BOUND)
                        pAd->DrsCounters.TxQuality[CurrRate] = DRS_TX_QUALITY_WORST_BOUND;
                }
                else // more than 2 failure, or no TX ok cases
                {
                    pAd->DrsCounters.TxQuality[CurrRate] = DRS_TX_QUALITY_WORST_BOUND;
                }
            }
            // upgrade TX quality if -
            // 1. no TX failure but do have TX ok case, and
            // 2. there's more one-time-ok cases than retry-ok cases in the past second
            else if ((pAd->DrsCounters.OneSecTxOkCount > pAd->DrsCounters.OneSecTxRetryOkCount))
            {
                fUpgradeQuality = TRUE;
                if (pAd->DrsCounters.TxQuality[CurrRate])
                    pAd->DrsCounters.TxQuality[CurrRate] --;  // quality very good in CurrRate

                if (pAd->DrsCounters.TxRateUpPenalty)
                    pAd->DrsCounters.TxRateUpPenalty --;
                else if (pAd->DrsCounters.TxQuality[UpRate])
                    pAd->DrsCounters.TxQuality[UpRate] --;    // may improve next UP rate's quality
            }
        }

        pAd->DrsCounters.PER[CurrRate] = (UCHAR)TxErrorRatio;

        if (pAd->DrsCounters.fNoisyEnvironment)
        {
            DBGPRINT(RT_DEBUG_TRACE,"DRS(noisy):"); 
        }
        else
        {
            DBGPRINT(RT_DEBUG_TRACE,"DRS:"); 
        }
        DBGPRINT(RT_DEBUG_TRACE, "Qty[%d]=%d PER=%d%% %d-sec, Qty[%d]=%d, Pty=%d\n", 
            RateIdToMbps[CurrRate], pAd->DrsCounters.TxQuality[CurrRate],
            TxErrorRatio,
            pAd->DrsCounters.CurrTxRateStableTime,
            RateIdToMbps[UpRate], pAd->DrsCounters.TxQuality[UpRate],
            pAd->DrsCounters.TxRateUpPenalty);
        
        // 2004-3-13 special case: Claim noisy environment
        //   decide if there was a false "rate down" in the past 2 sec due to noisy 
        //   environment. if so, we would rather switch back to the higher TX rate. 
        //   criteria -
        //     1. there's a higher rate available, AND
        //     2. there was a rate-down happened, AND
        //     3. current rate has 75% > PER > 20%, AND
        //     4. comparing to UpRate, current rate didn't improve PER more than 5 %
        if ((UpRate != CurrRate)                              &&
            (pAd->DrsCounters.LastSecTxRateChangeAction == 2) &&
            (TxTotalCnt > 15) &&  // this line is to prevent the case that not enough TX sample causing PER=0%
            (pAd->DrsCounters.PER[CurrRate] < 75) && 
            ((pAd->DrsCounters.PER[CurrRate] > 20) || (pAd->DrsCounters.fNoisyEnvironment)) && 
            ((pAd->DrsCounters.PER[CurrRate]+5) > pAd->DrsCounters.PER[UpRate]))
        {
            // we believe this is a noisy environment. better stay at UpRate
            DBGPRINT(RT_DEBUG_TRACE,"DRS: #### enter Noisy environment ####\n");
            pAd->DrsCounters.fNoisyEnvironment = TRUE;

            // 2004-3-14 when claiming noisy environment, we're not only switch back
            //   to UpRate, but can be more aggressive to use one more rate up
            UpRate++;
//          if (UpRate>RATE_54) UpRate=RATE_54;
            if ((UpRate==RATE_6) || (UpRate==RATE_9)) UpRate=RATE_12;
            if (UpRate > pAd->PortCfg.MaxTxRate)
                UpRate = pAd->PortCfg.MaxTxRate;
            pAd->PortCfg.TxRate = UpRate;
            break;
        }

        // 2004-3-12 special case: Leave noisy environment
        //   The interference has gone suddenly. reset TX rate to
        //   the theoritical value according to RSSI. Criteria -
        //     1. it's currently in noisy environment
        //     2. PER drops to be below 12%
        if ((pAd->DrsCounters.fNoisyEnvironment == TRUE) &&
            (TxTotalCnt > 15) && (pAd->DrsCounters.PER[CurrRate] <= 12))
        {
            UCHAR JumpUpRate;

            pAd->DrsCounters.fNoisyEnvironment = FALSE;
            for (JumpUpRate = RATE_54; JumpUpRate > RATE_1; JumpUpRate--)
            {
                if (pAd->PortCfg.AvgRssi > (RssiSafeLevelForTxRate[JumpUpRate] + pAd->PortCfg.RssiToDbm))

                    break;
            }

            if (JumpUpRate > pAd->PortCfg.MaxTxRate)
                JumpUpRate = pAd->PortCfg.MaxTxRate;
            
            DBGPRINT(RT_DEBUG_TRACE,"DRS: #### leave Noisy environment ####, RSSI=%d, JumpUpRate=%d\n",

            pAd->PortCfg.AvgRssi - RSSI_TO_DBM_OFFSET, RateIdToMbps[JumpUpRate]);

            
            if (JumpUpRate > CurrRate)
            {
                pAd->PortCfg.TxRate = JumpUpRate;
               	break;
            }
        }

        // we're going to upgrade CurrRate to UpRate at next few seconds, 
        // but before that, we'd better try a NULL frame @ UpRate and 
        // see if UpRate is stable or not. If this NULL frame fails, it will
        // downgrade TxQuality[CurrRate], so that STA won't switch to
        // to UpRate in the next second
        // 2004-04-07 requested by David Tung - sent test frames only in OFDM rates
        if (fUpgradeQuality      && 
            INFRA_ON(pAd)        && 
            (UpRate != CurrRate) && 
            (UpRate > RATE_11)   &&
            (pAd->DrsCounters.TxQuality[CurrRate] <= 1) &&
            (pAd->DrsCounters.TxQuality[UpRate] <= 1))
        {
            DBGPRINT(RT_DEBUG_TRACE,"DRS: 2 NULL frames at UpRate = %d Mbps\n",RateIdToMbps[UpRate]);
            EnqueueNullFrame(pAd, UpRate);
            EnqueueNullFrame(pAd, UpRate);
        }

        // perform DRS - consider TxRate Down first, then rate up.
        //     1. rate down, if current TX rate's quality is not good
        //     2. rate up, if UPRate's quality is very good
        if ((pAd->DrsCounters.TxQuality[CurrRate] >= DRS_TX_QUALITY_WORST_BOUND) &&
            (CurrRate != DownRate))
        {
#ifdef WIFI_TEST
            if (DownRate <= RATE_2) break; // never goes lower than 5.5 Mbps TX rate
#endif
           	pAd->PortCfg.TxRate = DownRate;
        }
        else if ((pAd->DrsCounters.TxQuality[CurrRate] <= 0) && 
            (pAd->DrsCounters.TxQuality[UpRate] <=0)         &&
            (CurrRate != UpRate))
        {
            pAd->PortCfg.TxRate = UpRate;
        }
        
    }while (FALSE);

    
    // if rate-up happen, clear all bad history of all TX rates
    if (pAd->PortCfg.TxRate > CurrRate)
    {
       	DBGPRINT(RT_DEBUG_TRACE,"DRS: ++TX rate from %d to %d Mbps\n", RateIdToMbps[CurrRate],RateIdToMbps[pAd->PortCfg.TxRate]);
        pAd->DrsCounters.CurrTxRateStableTime = 0;
        pAd->DrsCounters.TxRateUpPenalty = 0;
        pAd->DrsCounters.LastSecTxRateChangeAction = 1; // rate UP
        memset(pAd->DrsCounters.TxQuality, 0, MAX_LEN_OF_SUPPORTED_RATES);
        memset(pAd->DrsCounters.PER, 0, MAX_LEN_OF_SUPPORTED_RATES);
    }
    // if rate-down happen, only clear DownRate's bad history
    else if (pAd->PortCfg.TxRate < CurrRate)
    {
       	DBGPRINT(RT_DEBUG_TRACE,"DRS: --TX rate from %d to %d Mbps\n", RateIdToMbps[CurrRate],RateIdToMbps[pAd->PortCfg.TxRate]);
	    // shorter stable time require more penalty in next rate UP criteria
       	if (pAd->DrsCounters.CurrTxRateStableTime < 4)      // less then 4 sec
       	    pAd->DrsCounters.TxRateUpPenalty = DRS_PENALTY; // add 8 sec penalty
       	else if (pAd->DrsCounters.CurrTxRateStableTime < 8) // less then 8 sec
       	    pAd->DrsCounters.TxRateUpPenalty = 2;           // add 2 sec penalty
       	else                                                // >= 8 sec
       	    pAd->DrsCounters.TxRateUpPenalty = 0;           // no penalty
       	    
        pAd->DrsCounters.CurrTxRateStableTime = 0;
        pAd->DrsCounters.LastSecTxRateChangeAction = 2; // rate DOWN
       	pAd->DrsCounters.TxQuality[pAd->PortCfg.TxRate] = 0;
       	pAd->DrsCounters.PER[pAd->PortCfg.TxRate] = 0;
    }
    else
        pAd->DrsCounters.LastSecTxRateChangeAction = 0; // rate no change
    
    // reset all OneSecxxx counters
    pAd->DrsCounters.OneSecTxFailCount = 0;
    pAd->DrsCounters.OneSecTxOkCount = 0;
    pAd->DrsCounters.OneSecTxRetryOkCount = 0;
}

/*
    ==========================================================================
    Description:
        This routine is executed periodically inside MlmePeriodicExec() after 
        association with an AP.
        It checks if PortCfg.Psm is consistent with user policy (recorded in
        PortCfg.WindowsPowerMode). If not, enforce user policy. However, 
        there're some conditions to consider:
        1. we don't support power-saving in ADHOC mode, so Psm=PWR_ACTIVE all
           the time when Mibss==TRUE
        2. When Massoc==TRUE (INFRA mode), Psm should not be switch to PWR_SAVE
           if outgoing traffic available in TxRing or PrioRing.
    Output:
        1. change pAd->PortCfg.Psm to PWR_SAVE or leave it untouched
    ==========================================================================
 */
VOID MlmeCheckForPsmChange(
    IN PRTMP_ADAPTER pAd,
    IN ULONG    Now32)
{
	ULONG	PowerMode;
    // condition -
    // 1. Psm maybe ON only happen in INFRASTRUCTURE mode
    // 2. user wants either MAX_PSP or FAST_PSP
    // 3. but current psm is not in PWR_SAVE
    // 4. CNTL state machine is not doing SCANning
    // 5. no TX SUCCESS event for the past period
    PowerMode = pAd->PortCfg.WindowsPowerMode;
    
    if (INFRA_ON(pAd) &&
        (PowerMode != Ndis802_11PowerModeCAM) &&
        (pAd->PortCfg.Psm == PWR_ACTIVE) &&
        (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE) &&
        (pAd->WlanCounters.TransmittedFragmentCount.vv.LowPart == pAd->Mlme.PrevTxCnt))
    {
        MlmeSetPsmBit(pAd, PWR_SAVE);
        EnqueueNullFrame(pAd, pAd->PortCfg.TxRate);
    }
    
    // latch current count for next-time comparison
    pAd->Mlme.PrevTxCnt = pAd->WlanCounters.TransmittedFragmentCount.vv.LowPart;

}

VOID MlmeSetPsmBit(
    IN PRTMP_ADAPTER pAd, 
    IN USHORT psm)
{
    TXCSR7_STRUC txcsr7;
    
    txcsr7.word = 0;
    pAd->PortCfg.Psm = psm;    
    
    DBGPRINT(RT_DEBUG_TRACE, "MMCHK - change PSM bit to %d <<<\n", psm);
    if (psm == PWR_SAVE)
    {
        txcsr7.field.ARPowerManage = 1;
        RTMP_IO_WRITE32(pAd, TXCSR7, txcsr7.word);
    }
    else
    {
        txcsr7.field.ARPowerManage = 0;
        RTMP_IO_WRITE32(pAd, TXCSR7, txcsr7.word);
    }
}

VOID MlmeSetTxPreamble(
    IN PRTMP_ADAPTER pAd, 
    IN USHORT TxPreamble)
{
    ULONG Plcp1MCsr = 0x00700400;     // 0x13c, ACK/CTS PLCP at 1 Mbps
    ULONG Plcp2MCsr = 0x00380401;     // 0x140, ACK/CTS PLCP at 2 Mbps
    ULONG Plcp5MCsr = 0x00150402;     // 0x144, ACK/CTS PLCP at 5.5 Mbps
    ULONG Plcp11MCsr = 0x000b8403;     // 0x148, ACK/CTS PLCP at 11 Mbps
    
    if (TxPreamble == Rt802_11PreambleShort)
    {
        DBGPRINT(RT_DEBUG_TRACE, "MlmeSetTxPreamble (= SHORT PREAMBLE)\n");
//      Plcp1MCsr |= 0x00000008; // 1Mbps should always use long preamble
        Plcp2MCsr |= 0x00000008;
        Plcp5MCsr |= 0x00000008;
        Plcp11MCsr |= 0x00000008;
        pAd->PortCfg.TxPreambleInUsed = Rt802_11PreambleShort;
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "MlmeSetTxPreamble (= LONG PREAMBLE)\n");
        pAd->PortCfg.TxPreambleInUsed = Rt802_11PreambleLong;
    }

    RTMP_IO_WRITE32(pAd, PLCP1MCSR, Plcp1MCsr);
    RTMP_IO_WRITE32(pAd, PLCP2MCSR, Plcp2MCsr);
    RTMP_IO_WRITE32(pAd, PLCP5MCSR, Plcp5MCsr);
    RTMP_IO_WRITE32(pAd, PLCP11MCSR, Plcp11MCsr);
}
    
VOID MlmeUpdateTxRates(
    IN PRTMP_ADAPTER pAd,
    IN BOOLEAN		 bLinkUp)
{
    int i, num;
    UCHAR Rate, MaxDesire = RATE_1, MaxSupport = RATE_1;
    ULONG BasicRateBitmap = 0;
    UCHAR CurrBasicRate = RATE_1;

    // find max desired rate
    num = 0;
    for (i=0; i<MAX_LEN_OF_SUPPORTED_RATES; i++)
    {
        switch (pAd->PortCfg.DesiredRates[i] & 0x7f)
        {
            case 2:  Rate = RATE_1;   num++;   break;
            case 4:  Rate = RATE_2;   num++;   break;
            case 11: Rate = RATE_5_5; num++;   break;
            case 22: Rate = RATE_11;  num++;   break;
            case 12: Rate = RATE_6;   num++;   break;
            case 18: Rate = RATE_9;   num++;   break;
            case 24: Rate = RATE_12;  num++;   break;
            case 36: Rate = RATE_18;  num++;   break;
            case 48: Rate = RATE_24;  num++;   break;
            case 72: Rate = RATE_36;  num++;   break;
            case 96: Rate = RATE_48;  num++;   break;
            case 108: Rate = RATE_54; num++;   break;
            default: Rate = RATE_1;   break;
        }
        if (MaxDesire < Rate)  MaxDesire = Rate;
    }

    // 2003-12-10 802.11g WIFI spec disallow OFDM rates in 802.11g ADHOC mode
    if ((pAd->PortCfg.BssType == BSS_INDEP)        &&
        (pAd->PortCfg.PhyMode == PHY_11BG_MIXED)   && 
        (pAd->PortCfg.AdhocMode == 0) &&
        (MaxDesire > RATE_11))
        MaxDesire = RATE_11;
    
    pAd->PortCfg.MaxDesiredRate = MaxDesire;
    
    // Auto rate switching is enabled only if more than one DESIRED RATES are 
    // specified; otherwise disabled
    if (num <= 1)
        pAd->PortCfg.EnableAutoRateSwitching = FALSE;
    else
        pAd->PortCfg.EnableAutoRateSwitching = TRUE;

    // find max supported rate
    for (i=0; i<pAd->PortCfg.SupportedRatesLen; i++)
    {
        switch (pAd->PortCfg.SupportedRates[i] & 0x7f)
        {
            case 2: Rate = RATE_1;   
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0001;  
                    break;
            case 4: Rate = RATE_2;   
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0002;  
                    break;
            case 11: 
                    Rate = RATE_5_5; 
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0004;  
                    break;
            case 22: 
                    Rate = RATE_11;  
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0008;  
                    break;
            case 12: 
                    Rate = RATE_6;   
//                  if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0010;  
                    break;
            case 18: 
                    Rate = RATE_9;   
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0020;  
                    break;
            case 24: 
                    Rate = RATE_12;  
//                  if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0040;  
                    break;
            case 36: 
                    Rate = RATE_18;  
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0080;  
                    break;
            case 48: 
                    Rate = RATE_24;  
//                  if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0100;  
                    break;
            case 72: 
                    Rate = RATE_36;  
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0200;  
                    break;
            case 96: 
                    Rate = RATE_48;  
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0400;  
                    break;
            case 108: 
                    Rate = RATE_54; 
                    if (pAd->PortCfg.SupportedRates[i] & 0x80) 
                        BasicRateBitmap |= 0x0800;  
                    break;
            default:  
                    Rate = RATE_1;   
                    break;
        }
        if (MaxSupport < Rate)  MaxSupport = Rate;
    }
	RTMP_IO_WRITE32(pAd, ARCSR1, BasicRateBitmap);

    // calculate the expected ACK rate for each TX rate. This info is used to caculate
    // the DURATION field of outgoing unicast DATA/MGMT frame
    for (i=0; i<MAX_LEN_OF_SUPPORTED_RATES; i++)
    {
        if (BasicRateBitmap & (0x01 << i))
            CurrBasicRate = (UCHAR)i;
        pAd->PortCfg.ExpectedACKRate[i] = CurrBasicRate;
        DBGPRINT(RT_DEBUG_INFO,"Expected ACK rate[%d] = %d Mbps\n", RateIdToMbps[i], RateIdToMbps[CurrBasicRate]);
    }
        
    // max tx rate = min {max desire rate, max supported rate}
    if (MaxSupport < MaxDesire)
        pAd->PortCfg.MaxTxRate = MaxSupport;
    else
        pAd->PortCfg.MaxTxRate = MaxDesire;

    // 2003-07-31 john - 2500 doesn't have good sensitivity at high OFDM rates. to increase the success
    // ratio of initial DHCP packet exchange, TX rate starts from a lower rate depending
    // on average RSSI
    //   1. RSSI >= -70db, start at 54 Mbps (short distance)
    //   2. -70 > RSSI >= -75, start at 24 Mbps (mid distance)
    //   3. -75 > RSSI, start at 11 Mbps (long distance)
    if (pAd->PortCfg.EnableAutoRateSwitching)
    {
        if (pAd->PortCfg.Channel > 14)
            pAd->PortCfg.TxRate = RATE_6; // 802.11a
        else 
        {
            short dbm = pAd->PortCfg.AvgRssi - pAd->PortCfg.RssiToDbm;
			if (bLinkUp == TRUE && pAd->PortCfg.MaxTxRate >= RATE_24)
				pAd->PortCfg.TxRate = RATE_24;
			else
            	pAd->PortCfg.TxRate = pAd->PortCfg.MaxTxRate; 
            if (dbm < -75)
                pAd->PortCfg.TxRate = RATE_11;
            else if ((dbm < -70) && (pAd->PortCfg.TxRate > RATE_24))
                pAd->PortCfg.TxRate = RATE_24;
            DBGPRINT(RT_DEBUG_TRACE, " MlmeUpdateTxRates (Rssi=%d, init TX rate = %d Mbps)\n", dbm, RateIdToMbps[pAd->PortCfg.TxRate]);
        }
    }
    else
        pAd->PortCfg.TxRate = pAd->PortCfg.MaxTxRate;

    switch (pAd->PortCfg.PhyMode) {
        case PHY_11BG_MIXED:
        case PHY_11B:
            pAd->PortCfg.MlmeRate = RATE_2;
#ifdef	WIFI_TEST			
            pAd->PortCfg.RtsRate = RATE_11;
#else
            pAd->PortCfg.RtsRate = RATE_2;
#endif
            break;
        case PHY_11A:
            pAd->PortCfg.MlmeRate = RATE_6;
            pAd->PortCfg.RtsRate = RATE_6;
            break;
        case PHY_11ABG_MIXED:
            if (pAd->PortCfg.Channel <= 14)
            {
                pAd->PortCfg.MlmeRate = RATE_2;
                pAd->PortCfg.RtsRate = RATE_2;
            }
            else
            {
                pAd->PortCfg.MlmeRate = RATE_6;
                pAd->PortCfg.RtsRate = RATE_6;
            }
            break;
        default: // error
            pAd->PortCfg.MlmeRate = RATE_2;
            pAd->PortCfg.RtsRate = RATE_2;
            break;
    }
    
    DBGPRINT(RT_DEBUG_TRACE, " MlmeUpdateTxRates (MaxDesire=%d, MaxSupport=%d, MaxTxRate=%d, Rate Switching =%d)\n", 
             RateIdToMbps[MaxDesire], RateIdToMbps[MaxSupport], RateIdToMbps[pAd->PortCfg.MaxTxRate], pAd->PortCfg.EnableAutoRateSwitching);
    DBGPRINT(RT_DEBUG_TRACE, " MlmeUpdateTxRates (TxRate=%d, RtsRate=%d, BasicRateBitmap=0x%04x)\n", 
             RateIdToMbps[pAd->PortCfg.TxRate], RateIdToMbps[pAd->PortCfg.RtsRate], BasicRateBitmap);
}

VOID MlmeRadioOff(
    IN PRTMP_ADAPTER pAd)
{
	// Set Radio off flag
	RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF);

	// Link down first if any association exists
	if (INFRA_ON(pAd) || ADHOC_ON(pAd))
		LinkDown(pAd);

	// Abort Tx
	RTMP_IO_WRITE32(pAd, TXCSR0, 0x08);
	// Disable Rx
	RTMP_IO_WRITE32(pAd, RXCSR0, 0x01);
	// Turn off radio
	RTMP_IO_WRITE32(pAd, PWRCSR0, 0x00000000);

	if (pAd->PortCfg.LedMode == LED_MODE_ASUS)
	{
		ASIC_LED_ACT_OFF(pAd);
	}
	
	// Clean up old bss table
	BssTableInit(&pAd->PortCfg.BssTab);
}

VOID MlmeRadioOn(
    IN PRTMP_ADAPTER pAd)
{	
	// Turn on radio
	RTMP_IO_WRITE32(pAd, PWRCSR0, 0x3f3b3100);

	// Abort Tx
	RTMP_IO_WRITE32(pAd, TXCSR0, 0x08);
	// Disable Rx
	RTMP_IO_WRITE32(pAd, RXCSR0, 0x01);

	RTMPRingCleanUp(pAd, TX_RING);
	RTMPRingCleanUp(pAd, PRIO_RING);
	RTMPRingCleanUp(pAd, RX_RING);

	NICResetFromError(pAd);
	// Clear Radio off flag
	RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF);

	if (pAd->PortCfg.LedMode == LED_MODE_ASUS)
	{
		RTMP_IO_WRITE32(pAd, LEDCSR, 0x0002461E);
	}
}

// ===========================================================================================
// bss_table.c
// ===========================================================================================


/*! \brief initialize BSS table
 *  \param p_tab pointer to the table
 *  \return none
 *  \pre
 *  \post
 */
VOID BssTableInit(
    IN BSS_TABLE *Tab) 
{
    int i;

    Tab->BssNr = 0;
    for (i = 0; i < MAX_LEN_OF_BSS_TABLE; i++) 
    {
        memset(&Tab->BssEntry[i], 0, sizeof(BSS_ENTRY));
    }
}

/*! \brief search the BSS table by SSID
 *  \param p_tab pointer to the bss table
 *  \param ssid SSID string 
 *  \return index of the table, BSS_NOT_FOUND if not in the table
 *  \pre
 *  \post
 *  \note search by sequential search
 */
ULONG BssTableSearch(
    IN BSS_TABLE *Tab, 
    IN PMACADDR Bssid) 
{
    UCHAR i;
    
    for (i = 0; i < Tab->BssNr; i++) 
    {
        //printf("comparing %s and %s\n", p_tab->bss[i].ssid, ssid);
        if (MAC_ADDR_EQUAL(&(Tab->BssEntry[i].Bssid), Bssid)) 
        { 
            return i;
        }
    }
    return (ULONG)BSS_NOT_FOUND;
}

VOID BssTableDeleteEntry(
    IN OUT	BSS_TABLE *Tab, 
    IN		PMACADDR Bssid) 
{
    UCHAR i, j;
    
    for (i = 0; i < Tab->BssNr; i++) 
    {
        //printf("comparing %s and %s\n", p_tab->bss[i].ssid, ssid);
        if (MAC_ADDR_EQUAL(&(Tab->BssEntry[i].Bssid), Bssid)) 
        {
        	for (j = i; j < Tab->BssNr - 1; j++)
        	{
        		memcpy(&(Tab->BssEntry[j]), &(Tab->BssEntry[j + 1]), sizeof(BSS_ENTRY));
        	}
	        Tab->BssNr -= 1;
            return;
        }
    }
}

UCHAR	ZeroSsid[32] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*! \brief
 *  \param 
 *  \return
 *  \pre
 *  \post
 */
VOID BssEntrySet(
    IN	PRTMP_ADAPTER	pAd, 
    OUT BSS_ENTRY *pBss, 
    IN MACADDR *pBssid, 
    IN CHAR Ssid[], 
    IN UCHAR SsidLen, 
    IN UCHAR BssType, 
    IN USHORT BeaconPeriod, 
    IN BOOLEAN CfExist,
    IN CF_PARM *pCfParm, 
    IN USHORT AtimWin, 
    IN USHORT CapabilityInfo, 
    IN UCHAR Rates[], 
    IN UCHAR RatesLen,
    IN BOOLEAN ExtendedRateIeExist,
    IN UCHAR Channel,
    IN UCHAR Rssi,
    IN LARGE_INTEGER TimeStamp,
    IN PNDIS_802_11_VARIABLE_IEs pVIE) 
{
    COPY_MAC_ADDR(&pBss->Bssid, pBssid);
	// Default Hidden SSID to be TRUE, it will be turned to FALSE after coping SSID
	pBss->Hidden = 1;	
	if (SsidLen > 0)
	{
		// For hidden SSID AP, it might send beacon with SSID len equal to 0
		// Or send beacon /probe response with SSID len matching real SSID length,
		// but SSID is all zero. such as "00-00-00-00" with length 4.
		// We have to prevent this case overwrite correct table
		if (NdisEqualMemory(Ssid, ZeroSsid, SsidLen) == 0)
		{
    		memcpy(pBss->Ssid, Ssid, SsidLen);
    		pBss->SsidLen = SsidLen;
			pBss->Hidden = 0;
		}
	}
    pBss->BssType = BssType;
    pBss->BeaconPeriod = BeaconPeriod;
    if (BssType == BSS_INFRA) 
    {
        if (CfExist) 
        {
            pBss->CfpCount = pCfParm->CfpCount;
            pBss->CfpPeriod = pCfParm->CfpPeriod;
            pBss->CfpMaxDuration = pCfParm->CfpMaxDuration;
            pBss->CfpDurRemaining = pCfParm->CfpDurRemaining;
        }
    } 
    else 
    {
        pBss->AtimWin = AtimWin;
    }

    pBss->CapabilityInfo = CapabilityInfo;
	// The privacy bit indicate security is ON, it maight be WEP, TKIP or AES
	// Combine with AuthMode, they will decide the connection methods.
    pBss->Privacy = CAP_IS_PRIVACY_ON(pBss->CapabilityInfo);
    memcpy(pBss->Rates, Rates, RatesLen);
    pBss->RatesLen = RatesLen;
    pBss->ExtendedRateIeExist = ExtendedRateIeExist;
    pBss->Channel = Channel;
    pBss->Rssi = Rssi;

	// New for microsoft Fixed IEs
	memcpy(pBss->FixIEs.Timestamp, &TimeStamp, 8);
	pBss->FixIEs.BeaconInterval = BeaconPeriod;
	pBss->FixIEs.Capabilities = CapabilityInfo;

	// New for microsoft Variable IEs
	if (pVIE->Length != 0)
	{
		pBss->VarIELen = pVIE->Length + 2;
		memcpy(pBss->VarIEs, pVIE, pBss->VarIELen);
		pBss->WepStatus = BssCipherParse(pBss->VarIEs);
	}
	else
	{
		pBss->VarIELen = 0;
		// No SSN ID, if security is on, this is WEP algorithm
		if  (pBss->Privacy)
			pBss->WepStatus = Ndis802_11WEPEnabled;
		// No SSN ID, security is also off.
		else
			pBss->WepStatus = Ndis802_11WEPDisabled;
	}
}

/*! 
 *  \brief insert an entry into the bss table
 *  \param p_tab The BSS table
 *  \param Bssid BSSID
 *  \param ssid SSID
 *  \param ssid_len Length of SSID
 *  \param bss_type
 *  \param beacon_period
 *  \param timestamp
 *  \param p_cf
 *  \param atim_win
 *  \param cap
 *  \param rates
 *  \param rates_len
 *  \param channel_idx
 *  \return none
 *  \pre
 *  \post
 *  \note If SSID is identical, the old entry will be replaced by the new one
 */
ULONG BssTableSetEntry(
    IN	PRTMP_ADAPTER	pAd, 
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
    IN UCHAR ChannelNo,
    IN UCHAR Rssi,
    IN LARGE_INTEGER TimeStamp,
    IN PNDIS_802_11_VARIABLE_IEs pVIE)
{
    ULONG   Idx;

    Idx = BssTableSearch(Tab, Bssid);
    if (Idx == BSS_NOT_FOUND) 
    {
        if (Tab->BssNr >= MAX_LEN_OF_BSS_TABLE)
            return BSS_NOT_FOUND;
            
        Idx = Tab->BssNr;
        BssEntrySet(pAd, &Tab->BssEntry[Idx], Bssid, Ssid, SsidLen, BssType, BeaconPeriod,
                    CfExist, CfParm, AtimWin, CapabilityInfo, Rates, RatesLen, ExtendedRateIeExist,
                    ChannelNo, Rssi, TimeStamp, pVIE);
        Tab->BssNr++;
    } 
    else
    {
        BssEntrySet(pAd, &Tab->BssEntry[Idx], Bssid, Ssid, SsidLen, BssType, BeaconPeriod,
                    CfExist, CfParm, AtimWin, CapabilityInfo, Rates, RatesLen, ExtendedRateIeExist,
                    ChannelNo, Rssi, TimeStamp, pVIE);
    }
    
    return Idx;
}

VOID BssTableSsidSort(
    IN	PRTMP_ADAPTER	pAd, 
    OUT BSS_TABLE *OutTab, 
    IN  CHAR Ssid[], 
    IN  UCHAR SsidLen) 
{
    INT i;
    BssTableInit(OutTab);

    for (i = 0; i < pAd->PortCfg.BssTab.BssNr; i++) 
    {
        BSS_ENTRY *pInBss = &pAd->PortCfg.BssTab.BssEntry[i];
        
        if ((pInBss->BssType == pAd->PortCfg.BssType) && 
			((pInBss->SsidLen==SsidLen) && RTMPEqualMemory(pInBss->Ssid, Ssid, (ULONG) SsidLen)))
        {
            BSS_ENTRY *pOutBss = &OutTab->BssEntry[OutTab->BssNr];

			// Bss Type matched, SSID matched. 
			// We will check wepstatus for qualification Bss
			if (pAd->PortCfg.WepStatus != pInBss->WepStatus)
					continue;

			// Since the AP is using hidden SSID, and we are trying to connect to ANY
			// It definitely will fail. So, skip it.
			// CCX also require not even try to connect it!!
			if (SsidLen == 0)
				continue;
			
            // copy matching BSS from InTab to OutTab
            memcpy(pOutBss, pInBss, sizeof(BSS_ENTRY));
            
            OutTab->BssNr++;
        }
        else if ((pInBss->BssType == pAd->PortCfg.BssType) && (SsidLen == 0))
        {
            BSS_ENTRY *pOutBss = &OutTab->BssEntry[OutTab->BssNr];

			// Bss Type matched, SSID matched. 
			// We will check wepstatus for qualification Bss
			if (pAd->PortCfg.WepStatus != pInBss->WepStatus)
					continue;
			
            // copy matching BSS from InTab to OutTab
            memcpy(pOutBss, pInBss, sizeof(BSS_ENTRY));
            
            OutTab->BssNr++;
        }
#if 0
		else if ((pInBss->BssType == pAd->PortCfg.BssType) && (pInBss->SsidLen == 0))
		{
			// Add for hidden SSID. But we have to verify the security suite too.
            BSS_ENTRY *pOutBss = &OutTab->BssEntry[OutTab->BssNr];

			// Bss Type matched, SSID matched. 
			// We will check wepstatus for qualification Bss
			if (pAd->PortCfg.WepStatus != pInBss->WepStatus)
					continue;
			
            // copy matching BSS from InTab to OutTab
            memcpy(pOutBss, pInBss, sizeof(BSS_ENTRY));
            
            OutTab->BssNr++;			
		}
#endif		
		if (OutTab->BssNr >= MAX_LEN_OF_BSS_TABLE)
			break;
		
    }
    
    BssTableSortByRssi(OutTab);
}

VOID BssTableSortByRssi(
    IN OUT BSS_TABLE *OutTab) 
{
    INT       i, j;
    BSS_ENTRY TmpBss;

    for (i = 0; i < OutTab->BssNr - 1; i++) 
    {
        for (j = i+1; j < OutTab->BssNr; j++) 
        {
            if (OutTab->BssEntry[j].Rssi > OutTab->BssEntry[i].Rssi) 
            {
                memcpy(&TmpBss, &OutTab->BssEntry[j], sizeof(BSS_ENTRY));
                memcpy(&OutTab->BssEntry[j], &OutTab->BssEntry[i], sizeof(BSS_ENTRY));
                memcpy(&OutTab->BssEntry[i], &TmpBss, sizeof(BSS_ENTRY));
            }
        }
    }
}

NDIS_802_11_WEP_STATUS	BssCipherParse(
	IN	PUCHAR	pCipher)
{
	PBEACON_EID_STRUCT	pEid;
	PUCHAR				pTmp;

	pEid = (PBEACON_EID_STRUCT) pCipher;

	// Double check sanity information, although it should be done at peer beacon sanity check already.
	if (pEid->Eid != IE_WPA)
		return (Ndis802_11WEPDisabled);

	// Double check Var IE length, it must be no less than 0x16
	if (pEid->Len < 0x16)
		return (Ndis802_11WEPDisabled);
	
	// Skip OUI, version, and multicast suite
	// This part should be improved in the future when AP supported multiple cipher suite.
	// For now, it's OK since almost all APs have fixed cipher suite supported.
	pTmp = (PUCHAR) pEid->Octet;
	pTmp += 9;

	if (*pTmp == 4)			// AES
		return (Ndis802_11Encryption3Enabled);
	else if (*pTmp == 2)	// TKIP
		return (Ndis802_11Encryption2Enabled);

	return (Ndis802_11WEPDisabled);
}

// ===========================================================================================
// mac_table.c
// ===========================================================================================

/*! \brief generates a random mac address value for IBSS BSSID
 *  \param Addr the bssid location
 *  \return none
 *  \pre
 *  \post
 */
VOID MacAddrRandomBssid(
    IN PRTMP_ADAPTER pAd, 
    OUT MACADDR *Addr) 
{
    INT i;

    for (i = 0; i < ETH_ALEN; i++) 
    {
        Addr->Octet[i] = RandomByte(pAd);
    }
    
    Addr->Octet[0] = (Addr->Octet[0] & 0xfe) | 0x02;  // the first 2 bits must be 01xxxxxxxx
}

/*! \brief init the management mac frame header
 *  \param p_hdr mac header
 *  \param subtype subtype of the frame
 *  \param p_ds destination address, don't care if it is a broadcast address
 *  \return none
 *  \pre the station has the following information in the pAd->PortCfg
 *   - bssid
 *   - station address
 *  \post
 *  \note this function initializes the following field
 */
VOID MgtMacHeaderInit(
    IN	PRTMP_ADAPTER	pAd, 
    IN OUT PMACHDR Hdr, 
    IN UCHAR Subtype, 
    IN UCHAR ToDs, 
    IN PMACADDR Ds, 
    IN PMACADDR Bssid) 
{
    memset(Hdr, 0, sizeof(MACHDR));
    Hdr->Type = BTYPE_MGMT;
    Hdr->SubType = Subtype;
    Hdr->Tods = ToDs;
    COPY_MAC_ADDR(&Hdr->Addr1, Ds);
    COPY_MAC_ADDR(&Hdr->Addr2, &pAd->CurrentAddress);
    COPY_MAC_ADDR(&Hdr->Addr3, Bssid);
}

// ===========================================================================================
// mem_mgmt.c
// ===========================================================================================

/*!***************************************************************************
 * This routine build an outgoing frame, and fill all information specified 
 * in argument list to the frame body. The actual frame size is the summation 
 * of all arguments.
 * input params:
 *      Buffer - pointer to a pre-allocated memory segment
 *      args - a list of <int arg_size, arg> pairs.
 *      NOTE NOTE NOTE!!!! the last argument must be NULL, otherwise this
 *                         function will FAIL!!!
 * return:
 *      Size of the buffer
 * usage:  
 *      MakeOutgoingFrame(Buffer, output_length, 2, &fc, 2, &dur, 6, p_addr1, 6,p_addr2, END_OF_ARGS);
 ****************************************************************************/
ULONG MakeOutgoingFrame(
    OUT CHAR *Buffer, 
    OUT ULONG *FrameLen, ...) 
{
    CHAR   *p;
    int     leng;
    ULONG   TotLeng;
    va_list Args;

    // calculates the total length
    TotLeng = 0;
    va_start(Args, FrameLen);
    do 
    {
        leng = va_arg(Args, int);
        if (leng == END_OF_ARGS) 
        {
            break;
        }
        p = va_arg(Args, PVOID);
        memcpy(&Buffer[TotLeng], p, leng);
        TotLeng = TotLeng + leng;
    } while(TRUE);

    va_end(Args); /* clean up */
    *FrameLen = TotLeng;
    return TotLeng;
}

// ===========================================================================================
// mlme_queue.c
// ===========================================================================================

/*! \brief  Initialize The MLME Queue, used by MLME Functions
 *  \param  *Queue     The MLME Queue
 *  \return Always     Return NDIS_STATE_SUCCESS in this implementation
 *  \pre
 *  \post
 *  \note   Because this is done only once (at the init stage), no need to be locked
 */
NDIS_STATUS MlmeQueueInit(
    IN MLME_QUEUE *Queue) 
{
    INT i;

    spin_lock_init(&Queue->Lock);

    Queue->Num  = 0;
    Queue->Head = 0;
    Queue->Tail = 0;

    for (i = 0; i < MAX_LEN_OF_MLME_QUEUE; i++) 
    {
        Queue->Entry[i].Occupied = FALSE;
        Queue->Entry[i].MsgLen = 0;
        memset(Queue->Entry[i].Msg, 0, MAX_LEN_OF_MLME_BUFFER);
    }

    return NDIS_STATUS_SUCCESS;
}


/*! \brief   Enqueue a message for other threads, if they want to send messages to MLME thread
 *  \param  *Queue    The MLME Queue
 *  \param   Machine  The State Machine Id
 *  \param   MsgType  The Message Type
 *  \param   MsgLen   The Message length
 *  \param  *Msg      The message pointer
 *  \return  TRUE if enqueue is successful, FALSE if the queue is full
 *  \pre
 *  \post
 *  \note    The message has to be initialized
 */
BOOLEAN MlmeEnqueue(
    OUT MLME_QUEUE *Queue, 
    IN ULONG Machine, 
    IN ULONG MsgType, 
    IN ULONG MsgLen, 
    IN VOID *Msg) 
{
    INT Tail;

	// First check the size, it MUST not exceed the mlme queue size
	if (MsgLen > MAX_LEN_OF_MLME_BUFFER)
	{
        DBGPRINT(RT_DEBUG_ERROR, "MlmeEnqueueForRecv mlme frame too large, size = %d \n", MsgLen);
		return FALSE;
	}
	
    if (MlmeQueueFull(Queue)) 
    {
        printk(KERN_ERR DRV_NAME "MlmeEnqueue full, msg dropped and may corrupt MLME\n");
        return FALSE;
    }

    spin_lock(&(Queue->Lock));
    Tail = Queue->Tail;
    Queue->Tail++;
    Queue->Num++;
    if (Queue->Tail == MAX_LEN_OF_MLME_QUEUE) 
    {
        Queue->Tail = 0;
    }
    DBGPRINT(RT_DEBUG_INFO, "MlmeEnqueue, num=%d\n",Queue->Num);
 
    Queue->Entry[Tail].Occupied = TRUE;
    Queue->Entry[Tail].Machine = Machine;
    Queue->Entry[Tail].MsgType = MsgType;
    Queue->Entry[Tail].MsgLen  = MsgLen;
    memcpy(Queue->Entry[Tail].Msg, Msg, MsgLen);
    spin_unlock(&(Queue->Lock));
    return TRUE;
}

/*! \brief   This function is used when Recv gets a MLME message
 *  \param  *Queue           The MLME Queue
 *  \param   TimeStampHigh   The upper 32 bit of timestamp
 *  \param   TimeStampLow    The lower 32 bit of timestamp
 *  \param   Rssi            The receiving RSSI strength
 *  \param   MsgLen          The length of the message
 *  \param  *Msg             The message pointer
 *  \return  TRUE if everything ok, FALSE otherwise (like Queue Full)
 *  \pre
 *  \post
 */
BOOLEAN MlmeEnqueueForRecv(
    IN	PRTMP_ADAPTER	pAd, 
    OUT MLME_QUEUE *Queue, 
    IN ULONG TimeStampHigh, 
    IN ULONG TimeStampLow,
    IN UCHAR Rssi, 
    IN ULONG MsgLen, 
    IN VOID *Msg) 
{
    INT          Tail, Machine;
    MACFRAME    *Fr = (MACFRAME *)Msg;
    ULONG        MsgType;

	// First check the size, it MUST not exceed the mlme queue size
	if (MsgLen > MAX_LEN_OF_MLME_BUFFER)
	{
        DBGPRINT(RT_DEBUG_ERROR, "MlmeEnqueueForRecv mlme frame too large, size = %d \n", MsgLen);
		return FALSE;
	}

    if (MlmeQueueFull(Queue)) 
    {
        DBGPRINT(RT_DEBUG_ERROR, "MlmeEnqueueForRecv (queue full error) \n");
        return FALSE;
    }

    if (!MsgTypeSubst(Fr, &Machine, &MsgType)) 
    {
        DBGPRINT(RT_DEBUG_ERROR, "MlmeEnqueueForRecv (drop mgmt->subtype=%d)\n",Fr->Hdr.SubType);
        return FALSE;
    }
    
    // OK, we got all the informations, it is time to put things into queue
    spin_lock(&(Queue->Lock));
    Tail = Queue->Tail;
    Queue->Tail++;
    Queue->Num++;
    if (Queue->Tail == MAX_LEN_OF_MLME_QUEUE) 
    {
        Queue->Tail = 0;
    }

    DBGPRINT(RT_DEBUG_INFO, "MlmeEnqueueForRecv, num=%d\n",Queue->Num);
    
    Queue->Entry[Tail].Occupied = TRUE;
    Queue->Entry[Tail].Machine = Machine;
    Queue->Entry[Tail].MsgType = MsgType;
    Queue->Entry[Tail].MsgLen  = MsgLen;
    Queue->Entry[Tail].TimeStamp.vv.LowPart = TimeStampLow;
    Queue->Entry[Tail].TimeStamp.vv.HighPart = TimeStampHigh;
    Queue->Entry[Tail].Rssi = Rssi;
    memcpy(Queue->Entry[Tail].Msg, Msg, MsgLen);
    spin_unlock(&(Queue->Lock));

    MlmeHandler(pAd);

    return TRUE;
}

/*! \brief   Dequeue a message from the MLME Queue
 *  \param  *Queue    The MLME Queue
 *  \param  *Elem     The message dequeued from MLME Queue
 *  \return  TRUE if the Elem contains something, FALSE otherwise
 *  \pre
 *  \post
 */
BOOLEAN MlmeDequeue(
    IN MLME_QUEUE *Queue, 
    OUT MLME_QUEUE_ELEM **Elem) 
{
    spin_lock(&(Queue->Lock));
    *Elem = &(Queue->Entry[Queue->Head]);
    Queue->Num--;
    Queue->Head++;
    if (Queue->Head == MAX_LEN_OF_MLME_QUEUE) 
    {
        Queue->Head = 0;
    }
    spin_unlock(&(Queue->Lock));
    DBGPRINT(RT_DEBUG_INFO, "MlmeDequeue, num=%d\n",Queue->Num);

    return TRUE;
}

VOID	MlmeRestartStateMachine(
    IN	PRTMP_ADAPTER	pAd)
{
    MLME_QUEUE_ELEM		*Elem = NULL;

    if(!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_MLME_INITIALIZED)){
        DBGPRINT(RT_DEBUG_INFO, "MLME not yet initialized...\n");
        if(MlmeInit(pAd))
		DBGPRINT(RT_DEBUG_ERROR, "Failure to initialize mlme.\n");
	// Continue the reset procedure...
    }
   
    spin_lock(&pAd->Mlme.TaskLock);
    if(pAd->Mlme.Running) 
    {
        spin_unlock(&pAd->Mlme.TaskLock);
        return;
    } 
    else 
    {
        pAd->Mlme.Running = TRUE;
    }
    spin_unlock(&pAd->Mlme.TaskLock);

	// Remove all Mlme queues elements
    while (!MlmeQueueEmpty(&pAd->Mlme.Queue)) 
    {
        //From message type, determine which state machine I should drive
        if (MlmeDequeue(&pAd->Mlme.Queue, &Elem)) 
        {
            // free MLME element
            Elem->Occupied = FALSE;
            Elem->MsgLen = 0;
            
        }
        else {
            DBGPRINT(RT_DEBUG_ERROR, "ERROR: empty Elem in MlmeQueue\n");
        }
    }

	// Cancel all timer events
	// Be careful to cancel new added timer
    RTMPCancelTimer(&pAd->Mlme.AssocAux.AssocTimer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.ReassocTimer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.DisassocTimer);
    RTMPCancelTimer(&pAd->Mlme.AuthAux.AuthTimer);
    RTMPCancelTimer(&pAd->Mlme.AuthRspAux.AuthRspTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.ScanTimer);
    RTMPCancelTimer(&pAd->PortCfg.RfTuningTimer);

	// Change back to original channel in case of doing scan
	AsicSwitchChannel(pAd, pAd->PortCfg.Channel);
	AsicLockChannel(pAd, pAd->PortCfg.Channel);

	// Resume MSDU which is turned off durning scan
	RTMPResumeMsduTransmission(pAd);

	// Set all state machines back IDLE
    pAd->Mlme.CntlMachine.CurrState    = CNTL_IDLE;
	pAd->Mlme.AssocMachine.CurrState   = ASSOC_IDLE;
	pAd->Mlme.AuthMachine.CurrState    = AUTH_REQ_IDLE;
	pAd->Mlme.AuthRspMachine.CurrState = AUTH_RSP_IDLE;
	pAd->Mlme.SyncMachine.CurrState    = SYNC_IDLE;
	
	// Remove running state
    spin_lock(&pAd->Mlme.TaskLock);
    pAd->Mlme.Running = FALSE;
    spin_unlock(&pAd->Mlme.TaskLock);
}

/*! \brief  test if the MLME Queue is empty
 *  \param  *Queue    The MLME Queue
 *  \return TRUE if the Queue is empty, FALSE otherwise
 *  \pre
 *  \post
 */
BOOLEAN MlmeQueueEmpty(
    IN MLME_QUEUE *Queue) 
{
    BOOLEAN Ans;

    spin_lock(&(Queue->Lock));
    Ans = (Queue->Num == 0);
    spin_unlock(&(Queue->Lock));

    return Ans;
}

/*! \brief   test if the MLME Queue is full
 *  \param   *Queue      The MLME Queue
 *  \return  TRUE if the Queue is empty, FALSE otherwise
 *  \pre
 *  \post
 */
BOOLEAN MlmeQueueFull(
    IN MLME_QUEUE *Queue) 
{
    BOOLEAN Ans;

    spin_lock(&(Queue->Lock));
    Ans = (Queue->Num == MAX_LEN_OF_MLME_QUEUE);
    spin_unlock(&(Queue->Lock));

    return Ans;
}

/*! \brief   The destructor of MLME Queue
 *  \param 
 *  \return
 *  \pre
 *  \post
 *  \note   Clear Mlme Queue, Set Queue->Num to Zero.
 */
VOID MlmeQueueDestroy(
    IN MLME_QUEUE *Queue) 
{
    spin_lock(&(Queue->Lock));
    Queue->Num  = 0;
    Queue->Head = 0;
    Queue->Tail = 0;
    spin_unlock(&(Queue->Lock));
}

/*! \brief   To substitute the message type if the message is coming from external
 *  \param  *Fr            The frame received
 *  \param  *Machine       The state machine
 *  \param  *MsgType       the message type for the state machine
 *  \return TRUE if the substitution is successful, FALSE otherwise
 *  \pre
 *  \post
 */
BOOLEAN MsgTypeSubst(
    IN MACFRAME *Fr, 
    OUT INT *Machine, 
    OUT INT *MsgType) 
{
    USHORT Seq;
	UCHAR	EAPType;

	// The only data type will pass to this function is EAPOL frame
    if (Fr->Hdr.Type == BTYPE_DATA) 
    {    
       	*Machine = WPA_PSK_STATE_MACHINE;
       	EAPType = *((UCHAR*)Fr + LENGTH_802_11 + LENGTH_802_1_H + 1);
       	return(WpaMsgTypeSubst(EAPType, MsgType));
    }

    switch (Fr->Hdr.SubType) 
    {
        case SUBTYPE_ASSOC_REQ:
            *Machine = ASSOC_STATE_MACHINE;
            *MsgType = MT2_PEER_ASSOC_REQ;
            break;
        case SUBTYPE_ASSOC_RSP:
            *Machine = ASSOC_STATE_MACHINE;
            *MsgType = MT2_PEER_ASSOC_RSP;
            break;
        case SUBTYPE_REASSOC_REQ:
            *Machine = ASSOC_STATE_MACHINE;
            *MsgType = MT2_PEER_REASSOC_REQ;
            break;
        case SUBTYPE_REASSOC_RSP:
            *Machine = ASSOC_STATE_MACHINE;
            *MsgType = MT2_PEER_REASSOC_RSP;
            break;
        case SUBTYPE_PROBE_REQ:
            *Machine = SYNC_STATE_MACHINE;
            *MsgType = MT2_PEER_PROBE_REQ;
            break;
        case SUBTYPE_PROBE_RSP:
            *Machine = SYNC_STATE_MACHINE;
            *MsgType = MT2_PEER_PROBE_RSP;
            break;
        case SUBTYPE_BEACON:
            *Machine = SYNC_STATE_MACHINE;
            *MsgType = MT2_PEER_BEACON;
            break;
        case SUBTYPE_ATIM:
            *Machine = SYNC_STATE_MACHINE;
            *MsgType = MT2_PEER_ATIM;
            break;
        case SUBTYPE_DISASSOC:
            *Machine = ASSOC_STATE_MACHINE;
            *MsgType = MT2_PEER_DISASSOC_REQ;
            break;
        case SUBTYPE_AUTH:
            // get the sequence number from payload 24 Mac Header + 2 bytes algorithm
            memcpy(&Seq, &Fr->Octet[2], sizeof(USHORT));
            if (Seq == 1 || Seq == 3) 
            {
                *Machine = AUTH_RSP_STATE_MACHINE;
                *MsgType = MT2_PEER_AUTH_ODD;
            } 
            else if (Seq == 2 || Seq == 4) 
            {
                *Machine = AUTH_STATE_MACHINE;
                *MsgType = MT2_PEER_AUTH_EVEN;
            } 
            else 
            {
                return FALSE;
            }
            break;
        case SUBTYPE_DEAUTH:
            *Machine = AUTH_RSP_STATE_MACHINE;
            *MsgType = MT2_PEER_DEAUTH;
            break;
        default:
            return FALSE;
            break;
    }

    return TRUE;
}

// ===========================================================================================
// state_machine.c
// ===========================================================================================

/*! \brief Initialize the state machine.
 *  \param *S           pointer to the state machine 
 *  \param  Trans       State machine transition function
 *  \param  StNr        number of states 
 *  \param  MsgNr       number of messages 
 *  \param  DefFunc     default function, when there is invalid state/message combination 
 *  \param  InitState   initial state of the state machine 
 *  \param  Base        StateMachine base, internal use only
 *  \pre p_sm should be a legal pointer
 *  \post
 */

VOID StateMachineInit(
    IN STATE_MACHINE *S, 
    IN STATE_MACHINE_FUNC Trans[], 
    IN ULONG StNr, 
    IN ULONG MsgNr, 
    IN STATE_MACHINE_FUNC DefFunc, 
    IN ULONG InitState, 
    IN ULONG Base) 
{
    ULONG i, j;

    // set number of states and messages
    S->NrState = StNr;
    S->NrMsg   = MsgNr;
    S->Base    = Base;

    S->TransFunc  = Trans;
    
    // init all state transition to default function
    for (i = 0; i < StNr; i++) 
    {
        for (j = 0; j < MsgNr; j++) 
        {
            S->TransFunc[i * MsgNr + j] = DefFunc;
        }
    }
    
    // set the starting state
    S->CurrState = InitState;

}

/*! \brief This function fills in the function pointer into the cell in the state machine 
 *  \param *S   pointer to the state machine
 *  \param St   state
 *  \param Msg  incoming message
 *  \param f    the function to be executed when (state, message) combination occurs at the state machine
 *  \pre *S should be a legal pointer to the state machine, st, msg, should be all within the range, Base should be set in the initial state
 *  \post
 */
VOID StateMachineSetAction(
    IN STATE_MACHINE *S, 
    IN ULONG St, 
    IN ULONG Msg, 
    IN STATE_MACHINE_FUNC Func) 
{
    ULONG MsgIdx;
    
    MsgIdx = Msg - S->Base;

    if (St < S->NrState && MsgIdx < S->NrMsg) 
    {
        // boundary checking before setting the action
        S->TransFunc[St * S->NrMsg + MsgIdx] = Func;
    } 
}

/*! \brief   The destructor of the state machine
 *  \param  *S the statemachine
 *  \note    doing nothing at this moment, may need to do something if the implementation changed
 */
VOID
StateMachineDestroy(IN STATE_MACHINE *S) 
{
}

/*! \brief   This function does the state transition
 *  \param   *Adapter the NIC adapter pointer
 *  \param   *S       the state machine
 *  \param   *Elem    the message to be executed
 *  \return   None
 */
VOID StateMachinePerformAction(
    IN	PRTMP_ADAPTER	pAd, 
    IN STATE_MACHINE *S, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    (*(S->TransFunc[S->CurrState * S->NrMsg + Elem->MsgType - S->Base]))(pAd, Elem);
}

/*
    ==========================================================================
    Description:
        The drop function, when machine executes this, the message is simply 
        ignored. This function does nothing, the message is freed in 
        StateMachinePerformAction()
    ==========================================================================
 */
VOID Drop(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
#if 0
    if ((Elem->MsgType == MT2_PEER_BEACON) ||
        (Elem->MsgType == MT2_PEER_PROBE_REQ) ||
        (Elem->MsgType == MT2_PEER_PROBE_RSP))
        ;
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, ("Warn:>>Drop Msg=%d<<\n",Elem->MsgType));
    }
#endif    
}

// ===========================================================================================
// lfsr.c
// ===========================================================================================

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID LfsrInit(
    IN PRTMP_ADAPTER pAd, 
    IN ULONG Seed) 
{
    if (Seed == 0) 
        pAd->Mlme.ShiftReg = 1;
    else 
        pAd->Mlme.ShiftReg = Seed;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
UCHAR RandomByte(
    IN PRTMP_ADAPTER pAd) 
{
    ULONG i;
    UCHAR R, Result;

    R = 0;

    for (i = 0; i < 8; i++) 
    {
        if (pAd->Mlme.ShiftReg & 0x00000001) 
        {
            pAd->Mlme.ShiftReg = ((pAd->Mlme.ShiftReg ^ LFSR_MASK) >> 1) | 0x80000000;
            Result = 1;
        } 
        else 
        {
            pAd->Mlme.ShiftReg = pAd->Mlme.ShiftReg >> 1;
            Result = 0;
        }
        R = (R << 1) | Result;
    }

    return R;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID AsicSwitchChannel(
    IN PRTMP_ADAPTER pAd, 
    IN UCHAR Channel) 
{
    ULONG R3;
    UCHAR index;

    // TODO: need to update E2PROM format to add 802.11a channel's TX power calibration values
    if (Channel <= 14)    
        R3 = pAd->PortCfg.ChannelTxPower[Channel - 1];
    else 
        R3 = pAd->PortCfg.ChannelTxPower[0];

    if (R3 > 31)  R3 = 31;

    // E2PROM setting is calibrated for maximum TX power (i.e. 100%)
    // We lower TX power here according to the percentage specified from UI
    if (pAd->PortCfg.TxPowerPercentage > 90)       // 91 ~ 100%, treat as 100% in terms of mW
        ;
    else if (pAd->PortCfg.TxPowerPercentage > 60)  // 61 ~ 90%, treat as 75% in terms of mW
        R3 -= 1;
    else if (pAd->PortCfg.TxPowerPercentage > 30)  // 31 ~ 60%, treat as 50% in terms of mW
        R3 -= 3;
    else if (pAd->PortCfg.TxPowerPercentage > 15)  // 16 ~ 30%, treat as 25% in terms of mW
        R3 -= 6;
    else if (pAd->PortCfg.TxPowerPercentage > 9)   // 10 ~ 15%, treat as 12.5% in terms of mW
        R3 -= 9;
    else                                           // 0 ~ 9 %, treat as 6.25% in terms of mW
        R3 -= 12;

    R3 = R3 << 9; // shift TX power control to correct RF R3 bit position

    switch (pAd->PortCfg.RfType)
    {
        case RFIC_2522:
            for (index = 0; index < NUM_OF_2522_CHNL; index++)
            {
                if (Channel == RF2522RegTable[index].Channel)
                {
	                R3 = R3 | RF2522RegTable[index].R3; // set TX power
                    RTMP_RF_IO_WRITE32(pAd, RF2522RegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF2522RegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    pAd->PortCfg.LatchRfRegs.Channel = Channel;
                    pAd->PortCfg.LatchRfRegs.R1 = RF2522RegTable[index].R1;
                    pAd->PortCfg.LatchRfRegs.R2 = RF2522RegTable[index].R2;
                    pAd->PortCfg.LatchRfRegs.R3 = R3;
                    pAd->PortCfg.LatchRfRegs.R4 = RF2522RegTable[index].R4;
                    break;
                }
            }
            break;

        case RFIC_2523:
            for (index = 0; index < NUM_OF_2523_CHNL; index++)
            {
                if (Channel == RF2523RegTable[index].Channel)
                {
	                R3 = R3 | RF2523RegTable[index].R3; // set TX power
                    RTMP_RF_IO_WRITE32(pAd, RF2523RegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF2523RegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    RTMP_RF_IO_WRITE32(pAd, RF2523RegTable[index].R4);
                    pAd->PortCfg.LatchRfRegs.Channel = Channel;
                    pAd->PortCfg.LatchRfRegs.R1 = RF2523RegTable[index].R1;
                    pAd->PortCfg.LatchRfRegs.R2 = RF2523RegTable[index].R2;
                    pAd->PortCfg.LatchRfRegs.R3 = R3;
                    pAd->PortCfg.LatchRfRegs.R4 = RF2523RegTable[index].R4;
                    break;
                }
            }
            break;

        case RFIC_2524:
            for (index = 0; index < NUM_OF_2524_CHNL; index++)
            {
                if (Channel == RF2524RegTable[index].Channel)
                {
	                R3 = R3 | RF2524RegTable[index].R3; // set TX power
                    RTMP_RF_IO_WRITE32(pAd, RF2524RegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF2524RegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    RTMP_RF_IO_WRITE32(pAd, RF2524RegTable[index].R4);
                    pAd->PortCfg.LatchRfRegs.Channel = Channel;
                    pAd->PortCfg.LatchRfRegs.R1 = RF2524RegTable[index].R1;
                    pAd->PortCfg.LatchRfRegs.R2 = RF2524RegTable[index].R2;
                    pAd->PortCfg.LatchRfRegs.R3 = R3;
                    pAd->PortCfg.LatchRfRegs.R4 = RF2524RegTable[index].R4;
                    break;
                }
            }
            break;
            
        case RFIC_2525:
            for (index = 0; index < NUM_OF_2525_CHNL; index++)
            {
                if (Channel == RF2525RegTable[index].Channel)
                {
		    // Tx power should based on the real channel value
	            R3 = R3 | RF2525RegTable[index].R3; // set TX power
		    // Set the channel to half band higher - 8 channels
		    // The addition is based on Gary and Sheng's request
                    RTMP_RF_IO_WRITE32(pAd, RF2525HBOffsetRegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF2525HBOffsetRegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    RTMP_RF_IO_WRITE32(pAd, RF2525HBOffsetRegTable[index].R4);
		    // Chnage to teh connect channel
                    RTMP_RF_IO_WRITE32(pAd, RF2525RegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF2525RegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    RTMP_RF_IO_WRITE32(pAd, RF2525RegTable[index].R4);
                    pAd->PortCfg.LatchRfRegs.Channel = Channel;
                    pAd->PortCfg.LatchRfRegs.R1 = RF2525RegTable[index].R1;
                    pAd->PortCfg.LatchRfRegs.R2 = RF2525RegTable[index].R2;
                    pAd->PortCfg.LatchRfRegs.R3 = R3;
                    pAd->PortCfg.LatchRfRegs.R4 = RF2525RegTable[index].R4;
                    break;
                }
            }
            break;
            
        case RFIC_2525E:
            for (index = 0; index < NUM_OF_2525E_CHNL; index++)
            {
                if (Channel == RF2525eRegTable[index].Channel)
                {
	                R3 = R3 | RF2525eRegTable[index].R3; // set TX power
                    RTMP_RF_IO_WRITE32(pAd, RF2525eRegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF2525eRegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    RTMP_RF_IO_WRITE32(pAd, RF2525eRegTable[index].R4);
                    pAd->PortCfg.LatchRfRegs.Channel = Channel;
                    pAd->PortCfg.LatchRfRegs.R1 = RF2525eRegTable[index].R1;
                    pAd->PortCfg.LatchRfRegs.R2 = RF2525eRegTable[index].R2;
                    pAd->PortCfg.LatchRfRegs.R3 = R3;
                    pAd->PortCfg.LatchRfRegs.R4 = RF2525eRegTable[index].R4;
                    break;
                }
            }
            break;
            
        case RFIC_5222:
            for (index = 0; index < NUM_OF_5222_CHNL; index++)
            {
                if (Channel == RF5222RegTable[index].Channel)
                {
	                R3 = R3 | RF5222RegTable[index].R3; // set TX power
                    RTMP_RF_IO_WRITE32(pAd, RF5222RegTable[index].R1);
                    RTMP_RF_IO_WRITE32(pAd, RF5222RegTable[index].R2);
                    RTMP_RF_IO_WRITE32(pAd, R3);
                    RTMP_RF_IO_WRITE32(pAd, RF5222RegTable[index].R4);
                    pAd->PortCfg.LatchRfRegs.Channel = Channel;
                    pAd->PortCfg.LatchRfRegs.R1 = RF5222RegTable[index].R1;
                    pAd->PortCfg.LatchRfRegs.R2 = RF5222RegTable[index].R2;
                    pAd->PortCfg.LatchRfRegs.R3 = R3;
                    pAd->PortCfg.LatchRfRegs.R4 = RF5222RegTable[index].R4;
                    break;
                }
            }
            break;

        default:
            break;
    }

    DBGPRINT(RT_DEBUG_INFO, "AsicSwitchChannel(RF=%d) to #%d, TXPwr=%d, R1=0x%08x, R2=0x%08x, R3=0x%08x, R4=0x%08x\n",
        pAd->PortCfg.RfType, 
        pAd->PortCfg.LatchRfRegs.Channel, 
        pAd->PortCfg.TxPower,
        pAd->PortCfg.LatchRfRegs.R1, 
        pAd->PortCfg.LatchRfRegs.R2, 
        pAd->PortCfg.LatchRfRegs.R3, 
        pAd->PortCfg.LatchRfRegs.R4);
}

/*
    ==========================================================================
    Description:
        This function is required for 2421 only, and should not be used during
        site survey. It's only required after NIC decided to stay at a channel
        for a longer period.
        When this function is called, it's always after AsicSwitchChannel().
    ==========================================================================
 */
VOID AsicLockChannel(
    IN PRTMP_ADAPTER pAd, 
    IN UCHAR Channel) 
{
    UCHAR   r70;
	ULONG   FcsCnt;

	RTMPCancelTimer(&pAd->PortCfg.RfTuningTimer);
	RTMPSetTimer(pAd, &pAd->PortCfg.RfTuningTimer, 1000/HZ); // 1 msec timer to turn OFF RF auto tuning

	RTMP_BBP_IO_READ32_BY_REG_ID(pAd, 70, &r70);
	if (Channel == 14)
		r70 = 0x4E; 	//set r70 to 0x4E instead of r70 |= 0x08; for turn on Japan filter bit
	else
		r70 = 0x46; 	//set r70 to 0x46 instead of r70 &= 0xf7; for turn off Japan filter bit
	RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 70, r70);

	// Clear false CRC durning switch channel
	RTMP_IO_READ32(pAd, CNT0, &FcsCnt);
}

VOID AsicRfTuningExec(
    IN  unsigned long data)
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;

    switch (pAd->PortCfg.RfType)
    {
        case RFIC_2522:
        case RFIC_2524:
        case RFIC_2525:
        case RFIC_5222:
        case RFIC_2525E:
            pAd->PortCfg.LatchRfRegs.R1 &= 0xfffdffff;  // RF R1.bit17 "tune_en1" OFF
            pAd->PortCfg.LatchRfRegs.R3 &= 0xfffffeff;   // RF R3.bit8 "tune_en2" OFF
            RTMP_RF_IO_WRITE32(pAd, pAd->PortCfg.LatchRfRegs.R1); 
            RTMP_RF_IO_WRITE32(pAd, pAd->PortCfg.LatchRfRegs.R3); 
            DBGPRINT(RT_DEBUG_INFO, "AsicRfTuningExec(R1=0x%x,R3=0x%x)\n",pAd->PortCfg.LatchRfRegs.R1,pAd->PortCfg.LatchRfRegs.R3);
            break;
            
        case RFIC_2523:
            pAd->PortCfg.LatchRfRegs.R3 &= 0xfffffeff;   // RF R3.bit8 "tune_en2" OFF
            RTMP_RF_IO_WRITE32(pAd, pAd->PortCfg.LatchRfRegs.R3); 
            DBGPRINT(RT_DEBUG_INFO, "AsicRfTuningExec(R3=0x%x)\n",pAd->PortCfg.LatchRfRegs.R3);
            break;

        default:
            break;
    }
}

/*
    ==========================================================================
    Description:
        Gives CCK TX rate 2 more dB TX power.
        This routine works only in LINK UP in INFRASTRUCTURE mode.

        calculate desired Tx power in RF R3.Tx0~5,  should consider -
        1. TxPowerPercentage
        2. auto calibration based on TSSI feedback
        3. extra 2 db for CCK
        4. -10 db upon very-short distance (AvgRSSI >= -40db) to AP
    ==========================================================================
 */
VOID AsicAdjustTxPower(
    IN PRTMP_ADAPTER pAd) 
{
    ULONG R3, Channel, CurrTxPwr;

    if ((pAd->PortCfg.Channel >= 1) && (pAd->PortCfg.Channel <= 14))
        Channel = pAd->PortCfg.Channel;
    else 
        Channel = 1;  // don't have calibration info for 11A, temporarily use Channel 1
    
    // get TX Power base from E2PROM
    R3 = pAd->PortCfg.ChannelTxPower[Channel - 1];
    if (R3 > 31)  R3 = 31;

    // E2PROM setting is calibrated for maximum TX power (i.e. 100%)
    // We lower TX power here according to the percentage specified from UI
    if (pAd->PortCfg.TxPowerPercentage == 0xffffffff)       // AUTO TX POWER control
    {
        // only INFRASTRUCTURE mode and 100% TX power need furthur calibration
        if (pAd->MediaState == NdisMediaStateConnected)
        {
            // low TX power upon very-short distance to AP to solve some vendor's AP RX problem
            // in this case, no TSSI compensation is required.

            if ((pAd->DrsCounters.fNoisyEnvironment == FALSE) && 
                (pAd->PortCfg.AvgRssi > (pAd->PortCfg.RssiToDbm - RSSI_FOR_LOWEST_TX_POWER)))
                R3 -= LOWEST_TX_POWER_DELTA;
            else if ((pAd->DrsCounters.fNoisyEnvironment == FALSE) && 
                (pAd->PortCfg.AvgRssi > (pAd->PortCfg.RssiToDbm - RSSI_FOR_LOW_TX_POWER)))
                R3 -= LOW_TX_POWER_DELTA;


            // 2004-03-16 give OFDM rates lower than 48 mbps 2 more DB
            else if ((pAd->PortCfg.TxRate <= RATE_36) && (pAd->PortCfg.TxRate > RATE_11))
            {
                R3 +=2;
                if (R3 > 31) R3 = 31;
            }
            
            // 2 exclusive rules applied on CCK rates only -
            //    1. always plus 2 db for CCK
            //    2. adjust TX Power based on TSSI
            else if (pAd->PortCfg.TxRate <= RATE_11)
    	    {
    	        // if "auto calibration based on TSSI" is not required, then
    	        // always give CCK 2 more db
    	        if (pAd->PortCfg.bAutoTxAgc == FALSE)
    	        {
   	                R3 += 2;  // plus 2 db
   	                if (R3 > 31) R3 = 31;
    	        }
    	        
        	    // Auto calibrate Tx AGC if bAutoTxAgc is TRUE and TX rate is CCK, 
        	    // because E2PROM's TSSI reference is valid only in CCK range.
    	        else  
    	        {
    		        UCHAR	R1,TxPowerRef, TssiRef;

                    R3 = (pAd->PortCfg.LatchRfRegs.R3 >> 9) & 0x0000001f;
	    	        if (pAd->Mlme.PeriodicRound % 4 == 0) // every 4 second
	    	        {
        		        TxPowerRef = pAd->PortCfg.ChannelTxPower[Channel - 1];
	        	        TssiRef    = pAd->PortCfg.ChannelTssiRef[Channel - 1];
		                RTMP_BBP_IO_READ32_BY_REG_ID(pAd, BBP_Tx_Tssi,	&R1);
		                if ((TssiRef >= (R1 + pAd->PortCfg.ChannelTssiDelta)) ||
			                (TssiRef <= (R1 - pAd->PortCfg.ChannelTssiDelta)))
		                {
			                // Need R3 adjustment. However, we have to make sure there is only
			                // plus / minus 5 variation allowed
			                if (TssiRef > R1)
			                {				
				                R3 = (R3 < (ULONG) (TxPowerRef + 5)) ? (R3 + 1) : R3;
				                if (R3 > 31)
				                    R3 = 31;
				                DBGPRINT(RT_DEBUG_INFO,"TSSI(R1)=%d, ++TxPwr=%d\n", R1, R3);
			                }
			                else
			                {
				                R3 = (R3 > (ULONG) (TxPowerRef - 5)) ? (R3 - 1) : R3;
				                DBGPRINT(RT_DEBUG_INFO,"TSSI(R1)=%d, --TxPwr=%d\n", R1, R3);
			                }
		                }
	    	        }
    	        }
    	    }
    	    
        }
    }
    else // fixed AUTO TX power
    {
        if (pAd->PortCfg.TxPowerPercentage > 90)  // 91 ~ 100%, treat as 100% in terms of mW
            ;
        else if (pAd->PortCfg.TxPowerPercentage > 60)  // 61 ~ 90%, treat as 75% in terms of mW
            R3 -= 1;
        else if (pAd->PortCfg.TxPowerPercentage > 30)  // 31 ~ 60%, treat as 50% in terms of mW
            R3 -= 3;
        else if (pAd->PortCfg.TxPowerPercentage > 15)  // 16 ~ 30%, treat as 25% in terms of mW
            R3 -= 6;
        else if (pAd->PortCfg.TxPowerPercentage > 9)   // 10 ~ 15%, treat as 12.5% in terms of mW
            R3 -= 9;
        else                                           // 0 ~ 9 %, treat as MIN(~3%) in terms of mW
            R3 -= 12;
        if (R3 > 31)  R3 = 0;   // negative value, set as minimum 0

        // 2004-03-16 give TX rates <= 36 mbps 2 more DB
        if (pAd->PortCfg.TxRate <= RATE_36)
        {
            R3 +=2;
            if (R3 > 31) R3 = 31;
        }
    }
    
    // compare the desired R3.TxPwr value with current R3, if not equal
    // set new R3.TxPwr
    CurrTxPwr = (pAd->PortCfg.LatchRfRegs.R3 >> 9) & 0x0000001f;
    if (CurrTxPwr != R3)
    {
        CurrTxPwr = R3;
        R3 = (pAd->PortCfg.LatchRfRegs.R3 & 0xffffc1ff) | (R3 << 9);
        RTMP_RF_IO_WRITE32(pAd, R3);
        pAd->PortCfg.LatchRfRegs.R3 = R3;
    }
    DBGPRINT(RT_DEBUG_INFO, "AsicAdjustTxPower = %d, AvgRssi = %d\n",
        CurrTxPwr, pAd->PortCfg.AvgRssi - pAd->PortCfg.RssiToDbm);

}

/*
    ==========================================================================
    Description:
        put PHY to sleep here, and set next wakeup timer
    ==========================================================================
 */
VOID AsicSleepThenAutoWakeup(
    IN PRTMP_ADAPTER pAd, 
    IN USHORT TbttNumToNextWakeUp) 
{
    CSR20_STRUC Csr20;
    PWRCSR1_STRUC Pwrcsr1;

    // we have decided to SLEEP, so at least do it for a BEACON period.
    if (TbttNumToNextWakeUp==0)
        TbttNumToNextWakeUp=1;
    
    // PWRCSR0 remains untouched
    
    // set CSR20 for next wakeup
    Csr20.word = 0;
    Csr20.field.NumBcnBeforeWakeup = TbttNumToNextWakeUp - 1;
    Csr20.field.DelayAfterBcn = (pAd->PortCfg.BeaconPeriod - 20) << 4; // 20 TU ahead of desired TBTT
    Csr20.field.AutoWake = 1;
    RTMP_IO_WRITE32(pAd, CSR20, Csr20.word);

    // set PWRCSR1 to put PHY into SLEEP state
    Pwrcsr1.word = 0;
    Pwrcsr1.field.PutToSleep = 1;
    Pwrcsr1.field.BbpDesireState = 1; // 01:SLEEP
    Pwrcsr1.field.RfDesireState = 1;  // 01:SLEEP
    RTMP_IO_WRITE32(pAd, PWRCSR1, Pwrcsr1.word);
    pAd->PortCfg.Pss = PWR_SAVE;
}

/*
    ==========================================================================
    Description:
        AsicForceWakeup() is used whenever manual wakeup is required
        AsicForceSleep() should only be used when Massoc==FALSE. When
        Massoc==TRUE, we should use AsicSleepThenAutoWakeup() instead.
    ==========================================================================
 */
VOID AsicForceSleep(
    IN PRTMP_ADAPTER pAd)
{
    PWRCSR1_STRUC Pwrcsr1;

    if (pAd->PortCfg.Pss == PWR_ACTIVE)
    {
        DBGPRINT(RT_DEBUG_TRACE, ">>>AsicForceSleep<<<\n");
        Pwrcsr1.word = 0;
        Pwrcsr1.field.RfDesireState = 1; // 01:SLEEP state
        Pwrcsr1.field.BbpDesireState = 1; // 01:SLEEP state
        Pwrcsr1.field.SetState = 1;
        RTMP_IO_WRITE32(pAd, PWRCSR1, Pwrcsr1.word);
        pAd->PortCfg.Pss = PWR_SAVE;
    }
}

VOID AsicForceWakeup(
    IN PRTMP_ADAPTER pAd)
{
    CSR20_STRUC Csr20;
    PWRCSR1_STRUC Pwrcsr1;

    if (pAd->PortCfg.Pss == PWR_SAVE)
    {
        DBGPRINT(RT_DEBUG_TRACE, ">>>AsicForceWakeup<<<\n");

        // 2003-12-19 turn OFF auto wakeup first
        Csr20.word = 0;
        Csr20.field.AutoWake = 0;
        RTMP_IO_WRITE32(pAd, CSR20, Csr20.word);

        Pwrcsr1.word = 0;
        Pwrcsr1.field.RfDesireState = 3; // 11:AWAKE state
        Pwrcsr1.field.BbpDesireState = 3; // 11:AWAKE state
        Pwrcsr1.field.SetState = 1;
        RTMP_IO_WRITE32(pAd, PWRCSR1, Pwrcsr1.word);
        pAd->PortCfg.Pss = PWR_ACTIVE;
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID AsicSetBssid(
    IN PRTMP_ADAPTER pAd, 
    IN MACADDR *Bssid) 
{
    ULONG         Addr4;

    Addr4 = (ULONG)(Bssid->Octet[0]) | 
            (ULONG)(Bssid->Octet[1] << 8) | 
            (ULONG)(Bssid->Octet[2] << 16) |
            (ULONG)(Bssid->Octet[3] << 24);
    RTMP_IO_WRITE32(pAd, CSR5, Addr4);
    
    Addr4 = (ULONG)(Bssid->Octet[4]) | (ULONG)(Bssid->Octet[5] << 8);
    RTMP_IO_WRITE32(pAd, CSR6, Addr4);
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID AsicDisableSync(
    IN PRTMP_ADAPTER pAd) 
{
    // TIMECSR_STRUC TimeCsr;
    DBGPRINT(RT_DEBUG_TRACE, "--->Disable TSF synchronization\n");
#if 1
    // 2003-12-20 disable TSF and Tbcn while NIC in power-saving have side effect
    //            that NIC will never wakes up because TSF stops and no more TBTT interrupts
    RTMP_IO_WRITE32(pAd, CSR14, 0x00000009);
#else
    RTMP_IO_WRITE32(pAd, CSR14, 0x00000000);
#endif

#if 0    
    RTMP_IO_READ32(pAd, TIMECSR, &TimeCsr.word);

    // restore to 33 PCI-tick-per-Usec. for 2560a only where PCI-clock is used as TSF timing source
    if (TimeCsr.field.UsCnt != 0x21)
    {
        TimeCsr.field.UsCnt = 0x21;
        RTMP_IO_WRITE32(pAd, TIMECSR, TimeCsr.word);
    }
#endif
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID AsicEnableBssSync(
    IN PRTMP_ADAPTER pAd) 
{
    CSR12_STRUC Csr12;
    CSR13_STRUC Csr13;
    CSR14_STRUC Csr14;
    BCNCSR1_STRUC Bcncsr1;
    BOOLEAN IsApPc;

    DBGPRINT(RT_DEBUG_TRACE, "--->AsicEnableBssSync(INFRA mode)\n");
    
    RTMP_IO_WRITE32(pAd, CSR14, 0x00000000);
    
    Csr12.word = 0;
    Csr12.field.BeaconInterval = pAd->PortCfg.BeaconPeriod << 4; // ASIC register in units of 1/16 TU
    Csr12.field.CfpMaxDuration = pAd->PortCfg.CfpMaxDuration << 4; // ASIC register in units of 1/16 TU
    RTMP_IO_WRITE32(pAd, CSR12, Csr12.word);
    
    Csr13.word = 0;
    Csr13.field.CfpPeriod = pAd->PortCfg.CfpDurRemain << 4; // ASIC register in units of 1/16 TU
    RTMP_IO_WRITE32(pAd, CSR13, Csr13.word);

    Bcncsr1.word = 0;
    Bcncsr1.field.Preload = TBTT_PRELOAD_TIME; // we guess TBTT is 2 TU ahead of BEACON-RxEnd time
    Bcncsr1.field.BeaconCwMin = 5;
    RTMP_IO_WRITE32(pAd, BCNCSR1, Bcncsr1.word);

    IsApPc = (CAP_IS_CF_POLLABLE_ON(pAd->PortCfg.CapabilityInfo) && 
              CAP_IS_CF_POLL_REQ_ON(pAd->PortCfg.CapabilityInfo));
    IsApPc = FALSE; // TODO: not support so far
    
    Csr14.word = 0;
    Csr14.field.TsfCount = 1;
    Csr14.field.TsfSync = 1; // sync TSF in INFRASTRUCTURE mode
    if (IsApPc) 
    {
        Csr14.field.CfpCntPreload = pAd->PortCfg.CfpCount;
        Csr14.field.Tcfp = 1;
    }
    Csr14.field.BeaconGen = 0;
//  Csr14.field.TbcnPreload = (pAd->PortCfg.BeaconPeriod - 30) << 4; // TODO: ???? 1 TU ???
    Csr14.field.Tbcn = 1;
    RTMP_IO_WRITE32(pAd, CSR14, Csr14.word);
    
}

/*
    ==========================================================================
    Description:
    Note: 
        BEACON frame in shared memory should be built ok before this routine
        can be called. Otherwise, a garbage frame maybe transmitted out every
        Beacon period.
    ==========================================================================
 */
VOID AsicEnableIbssSync(
    IN PRTMP_ADAPTER pAd)
{
    CSR12_STRUC Csr12;
    CSR13_STRUC Csr13;
    CSR14_STRUC Csr14;
    // BCNCSR_STRUC Bcncsr;
    BCNCSR1_STRUC Bcncsr1;
    
    DBGPRINT(RT_DEBUG_TRACE, "--->AsicEnableIbssSync(ADHOC mode)\n");

    RTMP_IO_WRITE32(pAd, CSR14, 0x00000000);

    Csr12.word = 0;
    Csr12.field.BeaconInterval = pAd->PortCfg.BeaconPeriod << 4; // ASIC register in units of 1/16 TU
    RTMP_IO_WRITE32(pAd, CSR12, Csr12.word);

    Csr13.word = 0;
    Csr13.field.AtimwDuration = pAd->PortCfg.AtimWin << 4; // ASIC register in units of 1/16 TU
    RTMP_IO_WRITE32(pAd, CSR13, Csr13.word);

    Bcncsr1.word = 0;
    if ((pAd->PortCfg.PhyMode == PHY_11B) || (pAd->PortCfg.PhyMode == PHY_11BG_MIXED))
    {
        Bcncsr1.field.BeaconCwMin = 5;
        Bcncsr1.field.Preload = 1024;  // 192 + ((MAC_HDR_LEN << 4) / RateIdTo500Kbps[pAd->PortCfg.MlmeRate]);
    }
    else
    {
        Bcncsr1.field.BeaconCwMin = 6;
        Bcncsr1.field.Preload = 700;   // 24 + ((MAC_HDR_LEN << 4) / RateIdTo500Kbps[pAd->PortCfg.MlmeRate]);
    }
    RTMP_IO_WRITE32(pAd, BCNCSR1, Bcncsr1.word);
    
    Csr14.word = 0;
    Csr14.field.TsfCount = 1;
    Csr14.field.TsfSync = 2; // sync TSF in IBSS mode
    Csr14.field.Tbcn = 1;
    Csr14.field.BeaconGen = 1;
    RTMP_IO_WRITE32(pAd, CSR14, Csr14.word);
}

VOID AsicLedPeriodicExec(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    ULONG LedCsr = 0x0000461E; // 0x0000461E;
    
    pAd->PortCfg.LedCntl.fOdd = ! pAd->PortCfg.LedCntl.fOdd;

    if (INFRA_ON(pAd) || ADHOC_ON(pAd))   
        LedCsr |= 0x00010000; // enable hardwired TX activity LED
    if (pAd->PortCfg.LedCntl.fOdd && pAd->PortCfg.LedCntl.fRxActivity) 
        LedCsr |= 0x00020000; // turn on software-based RX activity LED
    pAd->PortCfg.LedCntl.fRxActivity = FALSE;

    if (LedCsr != pAd->PortCfg.LedCntl.LastLedCsr)
    {
//        DBGPRINT(RT_DEBUG_TRACE, ("AsicLedPeriodicExec(%8x)\n",LedCsr));
        pAd->PortCfg.LedCntl.LastLedCsr = LedCsr;
        RTMP_IO_WRITE32(pAd, LEDCSR, LedCsr);
    }

    RTMPSetTimer(pAd, &pAd->PortCfg.LedCntl.BlinkTimer, 70);
}

// pAd->PortCfg.CurrentRxAntenna
// 0xff: diversity, 0:antenna A, 1:antenna B
VOID AsicSetRxAnt(
    IN PRTMP_ADAPTER pAd) 
{
    UCHAR   RxValue, TxValue;
    ULONG   Bbpcsr1;
    
    RTMPCancelTimer(&pAd->PortCfg.RxAnt.RxAntDiversityTimer);
    pAd->PortCfg.RxAnt.AvgRssi[0] = (-95 + 120) << 3;  // reset Ant-A's RSSI history
    pAd->PortCfg.RxAnt.AvgRssi[1] = (-95 + 120) << 3;  // reset Ant-B's RSSI history

   	pAd->PortCfg.RxAnt.PrimaryInUsed  = TRUE;
    
    if (pAd->PortCfg.CurrentRxAntenna == 0xff)     // Diversity
    {
       	pAd->PortCfg.RxAnt.PrimaryRxAnt   = 1;  // assume ant-B
       	pAd->PortCfg.RxAnt.SecondaryRxAnt = 0;  // assume ant-A
    }
    else if (pAd->PortCfg.CurrentRxAntenna == 0)   // ant-A
    {
       	pAd->PortCfg.RxAnt.PrimaryRxAnt   = 0;  // assume ant-A
       	pAd->PortCfg.RxAnt.SecondaryRxAnt = 1;  // assume ant-B
    }
    else                                           // ant-B
    {
       	pAd->PortCfg.RxAnt.PrimaryRxAnt   = 1;  // assume ant-B
       	pAd->PortCfg.RxAnt.SecondaryRxAnt = 0;  // assume ant-A
    }

    DBGPRINT(RT_DEBUG_TRACE,"AntDiv - set RxAnt=%d, primary=%d, second=%d\n",
        pAd->PortCfg.CurrentRxAntenna, pAd->PortCfg.RxAnt.PrimaryRxAnt, pAd->PortCfg.RxAnt.SecondaryRxAnt);
    
    // use primary antenna
    RTMP_IO_READ32(pAd, BBPCSR1, &Bbpcsr1);
    TxValue = pAd->PortCfg.BbpWriteLatch[BBP_Tx_Configure];
    RxValue = pAd->PortCfg.BbpWriteLatch[BBP_Rx_Configure];
    if (pAd->PortCfg.RxAnt.PrimaryRxAnt == 0) // ant-A
    {
        TxValue = (TxValue & 0xFC) | 0x00;
        RxValue = 0x1c; 
        Bbpcsr1 = (Bbpcsr1 & 0xFFFCFFFC) | 0x00000000;
    }
    else                                      // ant-B
    {
		TxValue = (TxValue & 0xFC) | 0x02;
		RxValue = 0x1e; 
        Bbpcsr1 = (Bbpcsr1 & 0xFFFCFFFC) | 0x00020002;
    }
    RTMP_IO_WRITE32(pAd, BBPCSR1, Bbpcsr1);
   	//RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, BBP_Tx_Configure, TxValue);
   	RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, BBP_Rx_Configure, RxValue);
        
}

// switch to secondary RxAnt for a while to collect it's average RSSI
// also set a timeout routine to DO the actual evaluation. If evaluation 
// result shows a much better RSSI using secondary RxAnt, then a official
// RX antenna switch is performed.
VOID AsicEvaluateSecondaryRxAnt(
    IN PRTMP_ADAPTER pAd) 
{
    UCHAR  RxValue, TxValue;
    ULONG  Bbpcsr1;

    if (pAd->PortCfg.CurrentRxAntenna != 0xff)
        return;
    
   	pAd->PortCfg.RxAnt.PrimaryInUsed  = FALSE;
   	pAd->PortCfg.RxAnt.FirstPktArrivedWhenEvaluate = FALSE;
   	pAd->PortCfg.RxAnt.RcvPktNumWhenEvaluate = 0;

//  pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.SecondaryRxAnt] = 0;

    DBGPRINT(RT_DEBUG_TRACE,"AntDiv - evaluate Ant #%d\n", pAd->PortCfg.RxAnt.SecondaryRxAnt);
    
    // temporarily switch to secondary antenna
    RxValue = pAd->PortCfg.BbpWriteLatch[BBP_Rx_Configure];
    TxValue = pAd->PortCfg.BbpWriteLatch[BBP_Tx_Configure];
    RTMP_IO_READ32(pAd, BBPCSR1, &Bbpcsr1);
    
    if (pAd->PortCfg.RxAnt.SecondaryRxAnt == 0) // ant-A
    {
        TxValue = (TxValue & 0xFC) | 0x00;
        RxValue = 0x1c; 
        Bbpcsr1 = (Bbpcsr1 & 0xFFFCFFFC) | 0x00000000;
    }
    else                                        // ant-B
    {
        TxValue = (TxValue & 0xFC) | 0x02;
		RxValue = 0x1e;
        Bbpcsr1 = (Bbpcsr1 & 0xFFFCFFFC) | 0x00020002;
    }
    RTMP_IO_WRITE32(pAd, BBPCSR1, Bbpcsr1);
   	//RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, BBP_Tx_Configure, TxValue);
   	RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, BBP_Rx_Configure, RxValue);

    // a one-shot timer to end the evalution
    if (pAd->MediaState == NdisMediaStateConnected)
        RTMPSetTimer(pAd, &pAd->PortCfg.RxAnt.RxAntDiversityTimer, 150);	
    else
   	RTMPSetTimer(pAd, &pAd->PortCfg.RxAnt.RxAntDiversityTimer, 300);
}

// this timeout routine collect AvgRssi[SecondaryRxAnt] and decide if
// SecondaryRxAnt is much better than PrimaryRxAnt
VOID AsicRxAntEvalTimeout(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;

 
    DBGPRINT(RT_DEBUG_TRACE,"AntDiv - AsicRxAntEvalTimeout, \n");
	// Do nothing if the driver is starting halt state.
	// This might happen when timer already been fired before cancel timer with mlmehalt
	if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_HALT_IN_PROGRESS))
		return;
	
   if (pAd->PortCfg.RxAnt.PrimaryInUsed == TRUE)

        return;

    // 1-db or more will we consider to switch antenna
    if ((pAd->PortCfg.RxAnt.RcvPktNumWhenEvaluate != 0) && (pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.SecondaryRxAnt] >=
        (pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.PrimaryRxAnt])))

    {
        UCHAR temp;
        // secondary antenna is much better than primary, switch RX antenna
        temp = pAd->PortCfg.RxAnt.PrimaryRxAnt;
        pAd->PortCfg.RxAnt.PrimaryRxAnt = pAd->PortCfg.RxAnt.SecondaryRxAnt;
        pAd->PortCfg.RxAnt.SecondaryRxAnt = temp;
        pAd->PortCfg.LastAvgRssi = (pAd->PortCfg.RxAnt.AvgRssi[pAd->PortCfg.RxAnt.SecondaryRxAnt] >> 3) - pAd->PortCfg.RssiToDbm;
        
        DBGPRINT(RT_DEBUG_TRACE,"AntDiv - Switch to Ant #%d, RSSI[0,1]=<%d, %d>\n",
            pAd->PortCfg.RxAnt.PrimaryRxAnt, pAd->PortCfg.RxAnt.AvgRssi[0], pAd->PortCfg.RxAnt.AvgRssi[1]);
    }
    else
    {
        UCHAR RxValue, TxValue;
        ULONG Bbpcsr1;
        
        // end of evaluation, swicth back to primary antenna
        RxValue = pAd->PortCfg.BbpWriteLatch[BBP_Rx_Configure];
        TxValue = pAd->PortCfg.BbpWriteLatch[BBP_Tx_Configure];
        RTMP_IO_READ32(pAd, BBPCSR1, &Bbpcsr1);
        if (pAd->PortCfg.RxAnt.PrimaryRxAnt == 0) // ant-A
        {
            TxValue = (TxValue & 0xFC) | 0x00;
            RxValue = 0x1c; 
            Bbpcsr1 = (Bbpcsr1 & 0xFFFCFFFC) | 0x00000000;
        }
        else                                      // ant-B
        {
		    TxValue = (TxValue & 0xFC) | 0x02;
		    RxValue = 0x1e;
            Bbpcsr1 = (Bbpcsr1 & 0xFFFCFFFC) | 0x00020002;
        }
        RTMP_IO_WRITE32(pAd, BBPCSR1, Bbpcsr1);
   	    //RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, BBP_Tx_Configure, TxValue);
   	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, BBP_Rx_Configure, RxValue);
        DBGPRINT(RT_DEBUG_TRACE,"AntDiv - remain Ant #%d, RSSI[0,1]=<%d, %d>, RcvPktNumWhenEvaluate=%d\n",
            pAd->PortCfg.RxAnt.PrimaryRxAnt, (pAd->PortCfg.RxAnt.AvgRssi[0] >> 3) - pAd->PortCfg.RssiToDbm, (pAd->PortCfg.RxAnt.AvgRssi[1] >> 3) - pAd->PortCfg.RssiToDbm, pAd->PortCfg.RxAnt.RcvPktNumWhenEvaluate);

    }

//  pAd->PortCfg.RxAnt.AvgRssi[0] = 0;  // reset Ant-A's RSSI history
//  pAd->PortCfg.RxAnt.AvgRssi[1] = 0;  // reset Ant-B's RSSI history
   	pAd->PortCfg.RxAnt.PrimaryInUsed  = TRUE;
    pAd->PortCfg.RxAnt.FirstPktArrivedWhenEvaluate = TRUE;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID AsicSetSlotTime(
    IN PRTMP_ADAPTER pAd,
    IN BOOLEAN UseShortSlotTime) 
{
    CSR11_STRUC Csr11;
    CSR18_STRUC Csr18;
    CSR19_STRUC Csr19;
    UCHAR PhyMode;

    pAd->PortCfg.ShortSlotInUsed = UseShortSlotTime;
    
    PhyMode = pAd->PortCfg.PhyMode;
    if (PhyMode == PHY_11ABG_MIXED)
    {
        if (pAd->PortCfg.Channel <=14)
            PhyMode = PHY_11BG_MIXED;
        else
            PhyMode = PHY_11A;
    }

    RTMP_IO_READ32(pAd, CSR11, &Csr11.word);
    if (PhyMode == PHY_11A)
        Csr11.field.SlotTime = 9;
    else
        Csr11.field.SlotTime = (UseShortSlotTime)? 9 : 20;
    RTMP_IO_WRITE32(pAd, CSR11, Csr11.word);

    RTMP_IO_READ32(pAd, CSR18, &Csr18.word);
    Csr18.field.PIFS = Csr18.field.SIFS + Csr11.field.SlotTime;
    RTMP_IO_WRITE32(pAd, CSR18, Csr18.word);

    Csr19.word = 0;
    Csr19.field.DIFS = Csr18.field.PIFS + Csr11.field.SlotTime;
    if (PhyMode == PHY_11B)
        Csr19.field.EIFS = 364;  // SIFS + ACK @1Mbps
    else
        Csr19.field.EIFS = 60;   // roughly = SIFS + ACK @6Mbps
    RTMP_IO_WRITE32(pAd, CSR19, Csr19.word);
    
#if 1
    // force using short SLOT time for FAE to demo performance only
    if (pAd->PortCfg.EnableTxBurst == 1)
        Csr11.field.SlotTime = 9;
    RTMP_IO_WRITE32(pAd, CSR11, Csr11.word);
#endif

    DBGPRINT(RT_DEBUG_TRACE, "AsicSetSlotTime(=%d us, CSR18=0x%08x, CSR19=0x%08x)\n",
        Csr11.field.SlotTime, Csr18.word, Csr19.word);
}

/*
    ==========================================================================
    Description:
       This routine is used for 2560a only where 2560a still use non-accurate
       PCI-clock as TSF 1-usec source. we have to dynamically change tick-per-usec 
       to avoid ADHOC synchronization issue with SYMBOL 11b card
    ==========================================================================
 */
VOID AsicAdjustUsec(
    IN PRTMP_ADAPTER pAd)
{
    TIMECSR_STRUC TimeCsr;
    UCHAR         TickPerUsec = 20;
    pAd->PortCfg.PciAdjustmentRound =  (pAd->PortCfg.PciAdjustmentRound+1) & 0x03;

    RTMP_IO_READ32(pAd, TIMECSR, &TimeCsr.word);
    if (pAd->PortCfg.PciAdjustmentRound == 0)
        TickPerUsec = 0x21;
    else if (pAd->PortCfg.PciAdjustmentRound == 1)
        TickPerUsec = 0x21;
    else if (pAd->PortCfg.PciAdjustmentRound == 2)
        TickPerUsec = 0x20;
    else
        TickPerUsec = 0x21;

    if (TimeCsr.field.UsCnt!= TickPerUsec)
    {
        TimeCsr.field.UsCnt= TickPerUsec;
        RTMP_IO_WRITE32(pAd, TIMECSR, TimeCsr.word);
        DBGPRINT(RT_DEBUG_INFO, "AsicAdjustUsec - change TIMECSR=0x%08x)\n",TimeCsr.word);
    }
}

/*
    ==========================================================================
    Description:
        danamic tune BBP R17 to find a balance between sensibility and 
        noise isolation
    ==========================================================================
 */
VOID AsicBbpTuning(
    IN PRTMP_ADAPTER pAd)
{
    ULONG Value;
    UCHAR R17;
    ULONG FalseCcaUpperThreshold = pAd->PortCfg.BbpTuning.FalseCcaUpperThreshold << 7;
    int dbm = pAd->PortCfg.AvgRssi - pAd->PortCfg.RssiToDbm;
    
    if ((! pAd->PortCfg.BbpTuningEnable) || (pAd->PortCfg.BbpTuning.VgcDelta==0))
        return;
    
    R17 = pAd->PortCfg.BbpWriteLatch[17];

	if ((pAd->PortCfg.Rt2560Version >= RT2560_VER_D) && 
	    (pAd->MediaState == NdisMediaStateConnected))
	{
        // Rule 0.
        // when RSSI is too weak, many signals will become false CCA thus affect R17 tuning.
        // so in this case, just stop R17 tuning (be sure R17 remains in <E2PROM-6, BBP_R17_DYNAMIC_UP_BOUND> range)
        if ((dbm < -80) && (pAd->Mlme.PeriodicRound > 20))
        {
            if (R17 >= BBP_R17_MID_SENSIBILITY)
            {
                R17 = pAd->PortCfg.LastR17Value;
        	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
            }
            DBGPRINT(RT_DEBUG_INFO, "RSSI = %d dbm, stop R17 at 0x%x\n", dbm, R17);
            return;
        }
        // Rule 1. "special big-R17 for short-distance" when not SCANNING
	    else if ((dbm >= RSSI_FOR_LOW_SENSIBILITY) && 
	        (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE))
	    {
	        if (R17 != BBP_R17_LOW_SENSIBILITY)
	        {
	            R17 = BBP_R17_LOW_SENSIBILITY;
    	        RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
	        }
            DBGPRINT(RT_DEBUG_INFO, "RSSI = %d dbm, fixed R17 at 0x%x\n", dbm, R17);
            return;
	    }
        // Rule 2. "special mid-R17 for mid-distance" when not SCANNING
	    else if ((dbm >= RSSI_FOR_MID_SENSIBILITY) && 
	        (pAd->Mlme.CntlMachine.CurrState == CNTL_IDLE))
	    {
	        if (R17 != BBP_R17_MID_SENSIBILITY)
	        {
	            R17 = BBP_R17_MID_SENSIBILITY;
    	        RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
	        }
            DBGPRINT(RT_DEBUG_INFO, "RSSI = %d dbm, fixed R17 at 0x%x\n", dbm, R17);
            return;
	    }
        // Rule 3. leave "short or mid-distance" condition, restore R17 to the 
        //    dynamic tuning range <E2PROM-6, BBP_R17_DYNAMIC_UP_BOUND>
	    else if (R17 >= BBP_R17_MID_SENSIBILITY)
	    {
	        R17 = pAd->PortCfg.LastR17Value;
    	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
            DBGPRINT(RT_DEBUG_INFO, "RSSI = %d dbm, restore R17 to 0x%x\n", dbm, R17);
            return;
	    }
	}
	
    // Rule 3. otherwise, R17 is currenly in dyanmic tuning range: <E2PROM-6, BBP_R17_DYNAMIC_UP_BOUND>.
    //    Keep dynamic tuning based on False CCA conter
    
	RTMP_IO_READ32(pAd, CNT3, &Value);
	pAd->PrivateInfo.CCAErrCnt = (Value & 0x0000ffff);
	DBGPRINT(RT_DEBUG_INFO, "CCA flase alarm = %d, Avg RSSI= %d dbm\n", 
	    pAd->PrivateInfo.CCAErrCnt, dbm);

	if ((pAd->PrivateInfo.CCAErrCnt > FalseCcaUpperThreshold) &&
	    (R17 < pAd->PortCfg.BbpTuning.VgcUpperBound))
	{
	    R17 += pAd->PortCfg.BbpTuning.VgcDelta;
	    pAd->PortCfg.LastR17Value = R17;
	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
        DBGPRINT(RT_DEBUG_INFO, "++R17= 0x%x\n", R17);
	}
	else if ((pAd->PrivateInfo.CCAErrCnt < pAd->PortCfg.BbpTuning.FalseCcaLowerThreshold) &&
	    (R17 > pAd->PortCfg.VgcLowerBound))
	{
	    R17 -= pAd->PortCfg.BbpTuning.VgcDelta;
	    pAd->PortCfg.LastR17Value = R17;
	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
        DBGPRINT(RT_DEBUG_INFO, "--R17= 0x%x\n", R17);
	}
}

// stop and restore R17 value upon SITE-SURVEY and LINK-DOWN
VOID AsicRestoreBbpSensibility(
    IN PRTMP_ADAPTER pAd)
{
    UCHAR R17;

    R17 = pAd->PortCfg.BbpWriteLatch[17];
	if (R17 >= BBP_R17_MID_SENSIBILITY)
	{
        R17 = pAd->PortCfg.LastR17Value;
	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAd, 17, R17);
        DBGPRINT(RT_DEBUG_TRACE, "AsicRestoreBbpSensibility(set R17= 0x%x)\n", R17);
	}
}

/*
    ========================================================================

    Routine Description:
        Mlme free the in-used nonpaged memory,
        move it to the unused memory link list

    Arguments:
        pAd                 Pointer to our adapter
        AllocVa             Pointer to the base virtual address for free

    Return Value:
        None
	
    Note:

    ========================================================================
*/
VOID    MlmeFreeMemory(
    IN PRTMP_ADAPTER pAd,
    IN PVOID         AllocVa)
{
    PMLME_MEMORY_STRUCT             pPrevious = NULL;
    PMLME_MEMORY_STRUCT             pMlmeMemoryStruct = NULL;
    UINT                            Index = 0;
    BOOLEAN                         bIsFound = FALSE;

    DBGPRINT(RT_DEBUG_INFO, "==> MlmeFreeMemory\n");
    spin_lock(&pAd->MemLock);
    if (pAd->Mlme.MemHandler.MemRunning)
    { 
        //Mlme memory handler is busy.
        //Move it to the Pending array for later free
        pAd->Mlme.MemHandler.MemFreePending[pAd->Mlme.MemHandler.PendingCount++] = (PULONG) AllocVa;

        DBGPRINT(RT_DEBUG_INFO, "Mlme memory Handler Busy!! move free memory to pending list [IN:%d][UN:%d][Pending:%d]\n",
                    pAd->Mlme.MemHandler.InUseCount, pAd->Mlme.MemHandler.UnUseCount, pAd->Mlme.MemHandler.PendingCount);
        DBGPRINT(RT_DEBUG_INFO, "<== MlmeFreeMemory\n");
        spin_unlock(&pAd->MemLock);
        return;
    }
    else
    {
        pAd->Mlme.MemHandler.MemRunning = TRUE;
        spin_unlock(&pAd->MemLock);
    }

    //First check is there have to free memory in the  pAd->Mlme.MemHandler.MemFreePending.
    while (pAd->Mlme.MemHandler.PendingCount > 0)
    {
        pPrevious = NULL;
        pMlmeMemoryStruct = pAd->Mlme.MemHandler.pInUseHead;
        while (pMlmeMemoryStruct)
        {
            if (pMlmeMemoryStruct->AllocVa == (PVOID) pAd->Mlme.MemHandler.MemFreePending[Index])
            { 
                //Found virtual address in the in-used link list
                //Remove it from the memory in-used link list, and move it to the unused link list
                if (pPrevious == NULL)
		{
                    pAd->Mlme.MemHandler.pInUseHead = pAd->Mlme.MemHandler.pInUseHead->Next;
                    //
                    // Update pInUseTail pointer, if this is an Empty list
                    //
                    if (pAd->Mlme.MemHandler.pInUseHead == NULL)
                        pAd->Mlme.MemHandler.pInUseTail = NULL;
                }
                else
		{
                    pPrevious->Next = pMlmeMemoryStruct->Next;
                    //
                    // Update pInUseTail pointer, if move the pInUserTail to Unused link list.
                    // move the pInuseTail to his previous.
                    //
                    if (pMlmeMemoryStruct->Next == NULL)
                    {
                        //
                        // This pMlmeMemoryStruct is the one pInUseTail, since it's next pointer is NULL.
                        // Then we need to update pInUseTail.
                        //
                        pAd->Mlme.MemHandler.pInUseTail = pPrevious;
                    }
                }

                if ((pAd->Mlme.MemHandler.pUnUseHead == NULL))
                { //No head, add it as head
                    pMlmeMemoryStruct->Next = NULL;
                    pAd->Mlme.MemHandler.pUnUseHead = pMlmeMemoryStruct;
                    pAd->Mlme.MemHandler.pUnUseTail = pAd->Mlme.MemHandler.pUnUseHead;
                }
                else
                {
                    //Append it to the tail in pAd->Mlme.MemHandler.pUnUseTail
                    pMlmeMemoryStruct->Next = NULL;
                    pAd->Mlme.MemHandler.pUnUseTail->Next = pMlmeMemoryStruct;
                    pAd->Mlme.MemHandler.pUnUseTail = pAd->Mlme.MemHandler.pUnUseTail->Next;
                }
                pAd->Mlme.MemHandler.MemFreePending[Index++] = NULL;
                pAd->Mlme.MemHandler.PendingCount--;
                pAd->Mlme.MemHandler.UnUseCount++;
                pAd->Mlme.MemHandler.InUseCount--;
                bIsFound = TRUE;
                break;
            }
            else
            {
                pPrevious = pMlmeMemoryStruct;
                pMlmeMemoryStruct = pMlmeMemoryStruct->Next;
            }
        }

        if (!bIsFound)
        {
            //This shoult not be happened! Just in case!
            DBGPRINT(RT_DEBUG_INFO, "<Warning>Free memory faild!! memory corruption on [Va:0x%lu] not found in In-Used link list [IN:%d][UN:%d][Pending:%d]\n",
                (unsigned long)pAd->Mlme.MemHandler.MemFreePending[pAd->Mlme.MemHandler.PendingCount],
                pAd->Mlme.MemHandler.InUseCount, pAd->Mlme.MemHandler.UnUseCount, pAd->Mlme.MemHandler.PendingCount);
            //lost a memory
            pAd->Mlme.MemHandler.MemFreePending[Index++] = NULL;
            pAd->Mlme.MemHandler.PendingCount--;
        }
    }

    pPrevious = NULL;
    pMlmeMemoryStruct = pAd->Mlme.MemHandler.pInUseHead;
    while (pMlmeMemoryStruct)
    {
        if (pMlmeMemoryStruct->AllocVa == AllocVa)
        {
            //Found virtual address in the in-used link list
            //Remove it from the memory in-used link list, and move it to the unused link list
            if (pPrevious == NULL)
	    {
                pAd->Mlme.MemHandler.pInUseHead = pAd->Mlme.MemHandler.pInUseHead->Next;
                //
                // Update pInUseTail pointer, if this is an Empty list
                //
                if (pAd->Mlme.MemHandler.pInUseHead == NULL)
                    pAd->Mlme.MemHandler.pInUseTail = NULL;
            }
            else
	    {
                pPrevious->Next = pMlmeMemoryStruct->Next;
                //
                // Update pInUseTail pointer, if move the pInUserTail to Unused link list.
                // move the pInuseTail to his previous.
                //
                if (pMlmeMemoryStruct->Next == NULL)
                {
                    //
                    // This pMlmeMemoryStruct is the one pInUseTail, since it's next pointer is NULL.
                    // Then we need to update pInUseTail.
                    //
                    pAd->Mlme.MemHandler.pInUseTail = pPrevious;
                }
            }

            if (pAd->Mlme.MemHandler.pUnUseHead == NULL)
            {
                pMlmeMemoryStruct->Next = NULL;
                pAd->Mlme.MemHandler.pUnUseHead = pMlmeMemoryStruct;
                pAd->Mlme.MemHandler.pUnUseTail = pMlmeMemoryStruct;
            }
            else
            {
                pMlmeMemoryStruct->Next = NULL;
                pAd->Mlme.MemHandler.pUnUseTail->Next = pMlmeMemoryStruct;
                pAd->Mlme.MemHandler.pUnUseTail = pMlmeMemoryStruct;
            }

            pAd->Mlme.MemHandler.InUseCount--;
            pAd->Mlme.MemHandler.UnUseCount++;
            DBGPRINT(RT_DEBUG_INFO, "MlmeFreeMemory Add it to the Unused memory link List[pMlmeMemoryStruct=0x%lu][VA=0x%lu]\n", (unsigned long)pMlmeMemoryStruct, (unsigned long)pMlmeMemoryStruct->AllocVa);
            break;
        }
        pPrevious = pMlmeMemoryStruct;
        pMlmeMemoryStruct = pMlmeMemoryStruct->Next;
    }

    spin_lock(&pAd->MemLock);
    pAd->Mlme.MemHandler.MemRunning = FALSE;
    spin_unlock(&pAd->MemLock);

    DBGPRINT(RT_DEBUG_INFO, "<== MlmeFreeMemory [IN:%d][UN:%d][Pending:%d]\n", 
                pAd->Mlme.MemHandler.InUseCount, pAd->Mlme.MemHandler.UnUseCount, pAd->Mlme.MemHandler.PendingCount);
}

/*
    ========================================================================

    Routine Description:
        Get an unused nonpaged system-space memory for use

    Arguments:
        pAd                 Pointer to our adapter
        AllocVa             Pointer to the base virtual address for later use

    Return Value:
        NDIS_STATUS_SUCCESS
        NDIS_STATUS_FAILURE
        NDIS_STATUS_RESOURCES
	
    Note:

    ========================================================================
*/
NDIS_STATUS MlmeAllocateMemory(
    IN PRTMP_ADAPTER pAd,
    OUT PVOID        *AllocVa)
{
    NDIS_STATUS                     Status = NDIS_STATUS_SUCCESS;
    PMLME_MEMORY_STRUCT             pMlmeMemoryStruct = NULL;

    DBGPRINT(RT_DEBUG_INFO, "==> MlmeAllocateMemory\n");
    spin_lock(&pAd->MemLock);
    if (pAd->Mlme.MemHandler.MemRunning)
    {
        DBGPRINT(RT_DEBUG_INFO, "Mlme memory Handler Busy!!, MlmeAllocateMemory failed!!\n");
        Status = NDIS_STATUS_FAILURE;
        DBGPRINT(RT_DEBUG_INFO, "<== MlmeAllocateMemory\n");
        spin_unlock(&pAd->MemLock);
        return (Status);
    }
    else
    {
        pAd->Mlme.MemHandler.MemRunning = TRUE;
        spin_unlock(&pAd->MemLock);
    }

    if (pAd->Mlme.MemHandler.pUnUseHead == NULL)
    { //There are no available memory for caller use 
        Status = NDIS_STATUS_RESOURCES;
        spin_lock(&pAd->MemLock);
        pAd->Mlme.MemHandler.MemRunning = FALSE;
        spin_unlock(&pAd->MemLock);
        DBGPRINT(RT_DEBUG_INFO, "MlmeAllocateMemory, failed!! (There are no available memory in list)\n");
        DBGPRINT(RT_DEBUG_INFO, "<== MlmeAllocateMemory\n");
        return (Status);
    }

    pMlmeMemoryStruct = pAd->Mlme.MemHandler.pUnUseHead;
    *AllocVa = pMlmeMemoryStruct->AllocVa;          //Saved porint to Pointer the base virtual address of the nonpaged memory for caller use.
    //Unused memory point to next available
    pAd->Mlme.MemHandler.pUnUseHead = pAd->Mlme.MemHandler.pUnUseHead->Next;
    pAd->Mlme.MemHandler.UnUseCount--;

    //Append the unused memory link list to the in-used link list tail
    if (pAd->Mlme.MemHandler.pInUseHead == NULL)
    {//no head, so current Item assign to In-use Head.
        pAd->Mlme.MemHandler.pInUseHead = pMlmeMemoryStruct;
        pAd->Mlme.MemHandler.pInUseHead->Next = NULL;
        pAd->Mlme.MemHandler.pInUseTail = pAd->Mlme.MemHandler.pInUseHead;
    }
    else
    {
        pMlmeMemoryStruct->Next = NULL;
        pAd->Mlme.MemHandler.pInUseTail->Next = pMlmeMemoryStruct;
        pAd->Mlme.MemHandler.pInUseTail = pAd->Mlme.MemHandler.pInUseTail->Next;
    }
    pAd->Mlme.MemHandler.InUseCount++;
    spin_lock(&pAd->MemLock);
    pAd->Mlme.MemHandler.MemRunning = FALSE;
    spin_unlock(&pAd->MemLock);
    DBGPRINT(RT_DEBUG_INFO, "MlmeAllocateMemory [pMlmeMemoryStruct=0x%lu][VA=0x%lu]\n", (unsigned long)pMlmeMemoryStruct, (unsigned long)pMlmeMemoryStruct->AllocVa);
    DBGPRINT(RT_DEBUG_INFO, "<== MlmeAllocateMemory[IN:%d][UN:%d][Pending:%d]\n",
                pAd->Mlme.MemHandler.InUseCount, pAd->Mlme.MemHandler.UnUseCount, pAd->Mlme.MemHandler.PendingCount);

    return (Status);
}

/*
    ========================================================================

    Routine Description:
        Allocates resident (nonpaged) system-space memory for MLME send frames

    Arguments:
        pAd                 Pointer to our adapter
        Number              Total nonpaged memory for use
        Size                Each nonpaged memory size

    Return Value:
        NDIS_STATUS_SUCCESS
        NDIS_STATUS_RESOURCES
        
    Note:

    ========================================================================
*/
NDIS_STATUS MlmeInitMemoryHandler(
    IN PRTMP_ADAPTER pAd,
    IN UINT  Number,
    IN UINT  Size)
{
    PMLME_MEMORY_STRUCT         Current = NULL;
    NDIS_STATUS                 Status = NDIS_STATUS_SUCCESS;
    UINT                        i;

    DBGPRINT(RT_DEBUG_INFO, "==> MlmeInitMemory\n");
    pAd->Mlme.MemHandler.MemoryCount = 0;
    pAd->Mlme.MemHandler.pInUseHead = NULL;
    pAd->Mlme.MemHandler.pInUseTail = NULL;
    pAd->Mlme.MemHandler.pUnUseHead = NULL;
    pAd->Mlme.MemHandler.pUnUseTail = NULL;
    pAd->Mlme.MemHandler.MemRunning = FALSE;

    //initial the memory free-pending array all to NULL;
    for (i = 0; i < MAX_MLME_HANDLER_MEMORY; i++)
        pAd->Mlme.MemHandler.MemFreePending[i] = NULL;

    //
    // Available nonpaged memory counts MAX_MLME_HANDLER_MEMORY
    //
    if (Number > MAX_MLME_HANDLER_MEMORY)
        Number = MAX_MLME_HANDLER_MEMORY;
        
    for (i = 0; i < Number; i++)
    {
        //Allocate a nonpaged memory for link list use.
        Current = kmalloc(sizeof(MLME_MEMORY_STRUCT), GFP_ATOMIC);
        if (Current == NULL)
        {
            Status = NDIS_STATUS_RESOURCES;
            break;
        }

        //Allocate a nonpaged memory for mlme use, Current->AllocVa is VirtualAddress pointer
        Current->AllocVa = kmalloc(Size, GFP_ATOMIC);
        if (Current->AllocVa == NULL)
        {
            Status = NDIS_STATUS_RESOURCES;
            //Free the nonpaged memory of the current link list
            kfree((VOID *)Current);
            break;
        }
        memset(Current->AllocVa, 0, Size);

        pAd->Mlme.MemHandler.MemoryCount++;

        //build up the link list
        if (pAd->Mlme.MemHandler.pUnUseHead != NULL)
        {
            Current->Next = pAd->Mlme.MemHandler.pUnUseHead;
            pAd->Mlme.MemHandler.pUnUseHead = Current;
        }
        else
        {
            Current->Next = NULL;
            pAd->Mlme.MemHandler.pUnUseHead = Current;
        }

        if (pAd->Mlme.MemHandler.pUnUseTail == NULL)
            pAd->Mlme.MemHandler.pUnUseTail = Current;

    }

    if (pAd->Mlme.MemHandler.MemoryCount < Number)
    {
        Status = NDIS_STATUS_RESOURCES;
        DBGPRINT(RT_DEBUG_TRACE, "MlmeInitMemory Initial failed [Require=%d, available=%d]\n", Number, pAd->Mlme.MemHandler.MemoryCount);
    }

    pAd->Mlme.MemHandler.InUseCount = 0;
    pAd->Mlme.MemHandler.UnUseCount = Number;
    pAd->Mlme.MemHandler.PendingCount = 0;
    DBGPRINT(RT_DEBUG_INFO, "<== MlmeInitMemory\n");
    return (Status);
}

/*
    ========================================================================

    Routine Description:
        Free Mlme memory handler (link list, nonpaged memory, spin lock)

    Arguments:
        pAd                 Pointer to our adapter

    Return Value:
        None
    ========================================================================
*/
VOID MlmeFreeMemoryHandler(
    IN PRTMP_ADAPTER pAd)
{
    PMLME_MEMORY_STRUCT      pMlmeMemoryStruct = NULL;

    //Free nonpaged memory, free it in the *In-used* link list.
    while (pAd->Mlme.MemHandler.pInUseHead != NULL)
    {
        pMlmeMemoryStruct = pAd->Mlme.MemHandler.pInUseHead;
        pAd->Mlme.MemHandler.pInUseHead = pAd->Mlme.MemHandler.pInUseHead->Next;
        //Free the virtual address in AllocVa which size is MAX_LEN_OF_MLME_BUFFER
        kfree(pMlmeMemoryStruct->AllocVa);
        //Free the link list item self
        kfree(pMlmeMemoryStruct);
    }

    //Free nonpaged memory, free it in the *Unused* link list.
    while (pAd->Mlme.MemHandler.pUnUseHead != NULL)
    {
        pMlmeMemoryStruct = pAd->Mlme.MemHandler.pUnUseHead;
        pAd->Mlme.MemHandler.pUnUseHead = pAd->Mlme.MemHandler.pUnUseHead->Next;
        //Free the virtual address in AllocVa which size is MAX_LEN_OF_MLME_BUFFER
        kfree(pMlmeMemoryStruct->AllocVa);
        //Free the link list item self
        kfree(pMlmeMemoryStruct);
    }
}

