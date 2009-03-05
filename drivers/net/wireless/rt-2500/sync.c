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
 *      Module Name: sync.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0
* 		MarkW			5th  Jun 05		Fix no-SSID broadcasting assoc.
 ***************************************************************************/ 

#include "rt_config.h"

// 2.4 Ghz channel plan
UCHAR Ra24Ghz_FCC[] = {1,2,3,4,5,6,7,8,9,10,11};
UCHAR Ra24Ghz_IC[]  = {1,2,3,4,5,6,7,8,9,10,11};
UCHAR Ra24Ghz_ESTI[]= {1,2,3,4,5,6,7,8,9,10,11,12,13};
UCHAR Ra24Ghz_SPAIN[] = {10,11};
UCHAR Ra24Ghz_FRANCE[] = {10,11,12,13};
UCHAR Ra24Ghz_MKK[] =  {14};
UCHAR Ra24Ghz_MKK1[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
UCHAR Ra24Ghz_ISRAEL[]  = {3,4,5,6,7,8,9};

// 5 Ghz channel plan
UCHAR Ra5Ghz_UNII[] = {36,40,44,48,52,56,60,64,  149,153,157,161};
UCHAR Ra5Ghz_MMAC[] = {34,38,42,46};
UCHAR Ra5Ghz_HyperLAN2[] = {36,40,44,48,52,56,60,64,  100,104,108,112,116,120,124,128,132,136,140};

extern UCHAR CipherSuiteWpaNoneTkip[];
extern UCHAR CipherSuiteWpaNoneTkipLen;

extern UCHAR CipherSuiteWpaNoneAes[];
extern UCHAR CipherSuiteWpaNoneAesLen;

/*
    ==========================================================================
    Description:
        The sync state machine, 
    Parameters:
        Sm - pointer to the state machine
    Note:
        the state machine looks like the following

    Column 1-2
                        SYNC_IDLE                       JOIN_WAIT_BEACON
    MT2_MLME_SCAN_REQ   mlme_scan_req_action            invalid_state_when_scan
    MT2_MLME_JOIN_REQ   mlme_join_req_action            invalid_state_when_join
    MT2_MLME_START_REQ  mlme_start_req_action           invalid_state_when_start
    MT2_PEER_BEACON     peer_beacon                     peer_beacon_at_join_wait_beacon_action
    MT2_PEER_PROBE_RSP  peer_beacon                     drop
    MT2_PEER_ATIM       drop                            drop
    MT2_SCAN_TIMEOUT    Drop                            Drop
    MT2_BEACON_TIMEOUT  Drop                            beacon_timeout_at_join_wait_beacon_action
    MT2_ATIM_TIMEOUT    Drop                            Drop
    MT2_PEER_PROBE_REQ  ????                            drop

    column 3
                         SCAN_LISTEN
    MT2_MLME_SCAN_REQ    invalid_state_when_scan
    MT2_MLME_JOIN_REQ    invalid_state_when_join
    MT2_MLME_START_REQ   invalid_state_when_start
    MT2_PEER_BEACON      peer_beacon_at_scan_action
    MT2_PEER_PROBE_RSP   peer_probe_rsp_at_scan_action
    MT2_PEER_ATIM        drop
    MT2_SCAN_TIMEOUT     scan_timeout_action
    MT2_BEACON_TIMEOUT   Drop
    MT2_ATIM_TIMEOUT     Drop
    MT2_PEER_PROBE_REQ   drop
    ==========================================================================
 */
VOID SyncStateMachineInit(
    IN PRTMP_ADAPTER pAd, 
    IN STATE_MACHINE *Sm, 
    OUT STATE_MACHINE_FUNC Trans[]) 
{
    StateMachineInit(Sm, (STATE_MACHINE_FUNC*)Trans, MAX_SYNC_STATE, MAX_SYNC_MSG, (STATE_MACHINE_FUNC)Drop, SYNC_IDLE, SYNC_MACHINE_BASE);

    // column 1
    StateMachineSetAction(Sm, SYNC_IDLE, MT2_MLME_SCAN_REQ, (STATE_MACHINE_FUNC)MlmeScanReqAction);
    StateMachineSetAction(Sm, SYNC_IDLE, MT2_MLME_JOIN_REQ, (STATE_MACHINE_FUNC)MlmeJoinReqAction);
    StateMachineSetAction(Sm, SYNC_IDLE, MT2_MLME_START_REQ, (STATE_MACHINE_FUNC)MlmeStartReqAction);
    StateMachineSetAction(Sm, SYNC_IDLE, MT2_PEER_BEACON, (STATE_MACHINE_FUNC)PeerBeacon);
//  StateMachineSetAction(Sm, SYNC_IDLE, MT2_PEER_PROBE_RSP, (STATE_MACHINE_FUNC)PeerBeacon);
    StateMachineSetAction(Sm, SYNC_IDLE, MT2_PEER_PROBE_REQ, (STATE_MACHINE_FUNC)PeerProbeReqAction); 

    //column 2
    StateMachineSetAction(Sm, JOIN_WAIT_BEACON, MT2_MLME_SCAN_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenScan);
    StateMachineSetAction(Sm, JOIN_WAIT_BEACON, MT2_MLME_JOIN_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenJoin);
    StateMachineSetAction(Sm, JOIN_WAIT_BEACON, MT2_MLME_START_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenStart);
    StateMachineSetAction(Sm, JOIN_WAIT_BEACON, MT2_PEER_BEACON, (STATE_MACHINE_FUNC)PeerBeaconAtJoinAction);
    StateMachineSetAction(Sm, JOIN_WAIT_BEACON, MT2_BEACON_TIMEOUT, (STATE_MACHINE_FUNC)BeaconTimeoutAtJoinAction);

    // column 3
    StateMachineSetAction(Sm, SCAN_LISTEN, MT2_MLME_SCAN_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenScan);
    StateMachineSetAction(Sm, SCAN_LISTEN, MT2_MLME_JOIN_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenJoin);
    StateMachineSetAction(Sm, SCAN_LISTEN, MT2_MLME_START_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenStart);
    StateMachineSetAction(Sm, SCAN_LISTEN, MT2_PEER_BEACON, (STATE_MACHINE_FUNC)PeerBeaconAtScanAction);
    StateMachineSetAction(Sm, SCAN_LISTEN, MT2_PEER_PROBE_RSP, (STATE_MACHINE_FUNC)PeerBeaconAtScanAction);
    StateMachineSetAction(Sm, SCAN_LISTEN, MT2_SCAN_TIMEOUT, (STATE_MACHINE_FUNC)ScanTimeoutAction);

    // timer init
    RTMPInitTimer(pAd, &pAd->Mlme.SyncAux.BeaconTimer, BeaconTimeout);
    RTMPInitTimer(pAd, &pAd->Mlme.SyncAux.ScanTimer, ScanTimeout);
}

/* 
    ==========================================================================
    Description:
        Becaon timeout handler, executed in timer thread
    ==========================================================================
 */
VOID BeaconTimeout(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;

    DBGPRINT(RT_DEBUG_TRACE,"SYNC - BeaconTimeout\n");
    MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_BEACON_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/* 
    ==========================================================================
    Description:
        ATIM timeout handler, executed in timer thread
    ==========================================================================
 */
VOID AtimTimeout(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    
    DBGPRINT(RT_DEBUG_TRACE,"SYNC - AtimTimeout \n");
    MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_ATIM_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/* 
    ==========================================================================
    Description:
        Scan timeout handler, executed in timer thread
    ==========================================================================
 */
VOID ScanTimeout(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;

    DBGPRINT(RT_DEBUG_INFO,"SYNC - Scan Timeout \n");
    MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_SCAN_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/* 
    ==========================================================================
    Description:
        MLME SCAN req state machine procedure
    ==========================================================================
 */
VOID MlmeScanReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    UCHAR          Ssid[MAX_LEN_OF_SSID], SsidLen, ScanType, BssType;
    ULONG          Now;

    // Suspend MSDU transmission here
    RTMPSuspendMsduTransmission(pAd);

    // first check the parameter sanity
    if (MlmeScanReqSanity(pAd, 
                          Elem->Msg, 
                          Elem->MsgLen, 
                          &BssType, 
                          Ssid, 
                          &SsidLen, 
                          &ScanType)) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "SYNC - MlmeScanReqAction\n");
        Now = jiffies;
        pAd->PortCfg.LastScanTime = Now;
        // reset all the timers
        RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);
        RTMPCancelTimer(&pAd->Mlme.SyncAux.ScanTimer);

        // record desired BSS parameters
        pAd->Mlme.SyncAux.BssType = BssType;
        pAd->Mlme.SyncAux.ScanType = ScanType;
        pAd->Mlme.SyncAux.SsidLen = SsidLen;
        memcpy(pAd->Mlme.SyncAux.Ssid, Ssid, SsidLen);
        
        // start from the first channel
        pAd->Mlme.SyncAux.Channel = FirstChannel(pAd);
        ScanNextChannel(pAd);
    } 
    else 
    {
        printk(KERN_ERR DRV_NAME "SYNC - MlmeScanReqAction() sanity check fail. BUG!!!\n");
        pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
        MlmeCntlConfirm(pAd, MT2_SCAN_CONF, MLME_INVALID_FORMAT);
    }
}

/* 
    ==========================================================================
    Description:
        MLME JOIN req state machine procedure
    ==========================================================================
 */
VOID MlmeJoinReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    BSS_ENTRY    *pBss;
    MLME_JOIN_REQ_STRUCT *Info = (MLME_JOIN_REQ_STRUCT *)(Elem->Msg);

    DBGPRINT(RT_DEBUG_TRACE, "SYNC - MlmeJoinReqAction(BSS #%d)\n", Info->BssIdx);

    // reset all the timers
    RTMPCancelTimer(&pAd->Mlme.SyncAux.ScanTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);

    pBss = &pAd->Mlme.CntlAux.SsidBssTab.BssEntry[Info->BssIdx];

    // record the desired SSID & BSSID we're waiting for
    COPY_MAC_ADDR(&pAd->Mlme.SyncAux.Bssid, &pBss->Bssid);
    memcpy(pAd->Mlme.SyncAux.Ssid, pBss->Ssid, pBss->SsidLen);
    pAd->Mlme.SyncAux.SsidLen = pBss->SsidLen;

    // switch channel and waiting for beacon timer
    AsicSwitchChannel(pAd, pBss->Channel);
    AsicLockChannel(pAd, pBss->Channel);
    DBGPRINT(RT_DEBUG_TRACE, "SYNC - Switch to channel %d, SSID %s \n", pBss->Channel, pAd->Mlme.SyncAux.Ssid);
    DBGPRINT(RT_DEBUG_TRACE, "SYNC - Wait BEACON from %02x:%02x:%02x:%02x:%02x:%02x ...\n", 
        pAd->Mlme.SyncAux.Bssid.Octet[0], pAd->Mlme.SyncAux.Bssid.Octet[1],
        pAd->Mlme.SyncAux.Bssid.Octet[2], pAd->Mlme.SyncAux.Bssid.Octet[3],
        pAd->Mlme.SyncAux.Bssid.Octet[4], pAd->Mlme.SyncAux.Bssid.Octet[5]);
    RTMPSetTimer(pAd, &pAd->Mlme.SyncAux.BeaconTimer, JOIN_TIMEOUT);

    pAd->Mlme.SyncMachine.CurrState = JOIN_WAIT_BEACON;
}

/* 
    ==========================================================================
    Description:
        MLME START Request state machine procedure, starting an IBSS
    ==========================================================================
 */
VOID MlmeStartReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    UCHAR         Ssid[MAX_LEN_OF_SSID], SsidLen; 

    // New for WPA security suites
    UCHAR                       VarIE[MAX_VIE_LEN];     // Total VIE length = MAX_VIE_LEN - -5
    NDIS_802_11_VARIABLE_IEs    *pVIE = NULL;
    LARGE_INTEGER               TimeStamp;
    BOOLEAN Privacy;
#ifdef  SINGLE_ADHOC_LINKUP
    ULONG   Bssidx;
    BOOLEAN CfExist = FALSE;
    CF_PARM CfParm;
#endif

    // Init Variable IE structure
    pVIE = (PNDIS_802_11_VARIABLE_IEs) VarIE;
    pVIE->Length = 0;
    TimeStamp.vv.LowPart  = 0;
    TimeStamp.vv.HighPart = 0;

    if (MlmeStartReqSanity(pAd, Elem->Msg, Elem->MsgLen, Ssid, &SsidLen)) 
    {
        // reset all the timers
        RTMPCancelTimer(&pAd->Mlme.SyncAux.ScanTimer);
        RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);

        // PortCfg.PrivacyInvoked should have been set via OID_802_11_WEP_STATUS. 
        // pAd->PortCfg.PrivacyInvoked = FALSE;

        memcpy(pAd->PortCfg.Ssid, Ssid, SsidLen); 
        pAd->PortCfg.SsidLen           = SsidLen;
        pAd->PortCfg.BssType           = BSS_INDEP;
        Privacy = (pAd->PortCfg.WepStatus == Ndis802_11Encryption1Enabled) || 
                  (pAd->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) || 
                  (pAd->PortCfg.WepStatus == Ndis802_11Encryption3Enabled);
        pAd->PortCfg.CapabilityInfo    = CAP_GENERATE(0,1,0,0,Privacy, (pAd->PortCfg.WindowsTxPreamble == Rt802_11PreambleShort));
        pAd->PortCfg.BeaconPeriod      = pAd->PortCfg.IbssConfig.BeaconPeriod;
        pAd->PortCfg.AtimWin           = pAd->PortCfg.IbssConfig.AtimWin;
        pAd->PortCfg.Channel           = pAd->PortCfg.IbssConfig.Channel;
        if ((pAd->PortCfg.PhyMode == PHY_11ABG_MIXED) && (pAd->PortCfg.Channel > 14))
        {
            // no 1,2,5.5,11 Mbps when in 5Ghz band
            pAd->PortCfg.SupportedRatesLen = pAd->PortCfg.IbssConfig.SupportedRatesLen - 4;
            memset(pAd->PortCfg.SupportedRates, 0, MAX_LEN_OF_SUPPORTED_RATES);
            memcpy(pAd->PortCfg.SupportedRates, &pAd->PortCfg.IbssConfig.SupportedRates[4], MAX_LEN_OF_SUPPORTED_RATES - 4);
        }
        else
        {
        pAd->PortCfg.SupportedRatesLen = pAd->PortCfg.IbssConfig.SupportedRatesLen;
        memcpy(pAd->PortCfg.SupportedRates, pAd->PortCfg.IbssConfig.SupportedRates, MAX_LEN_OF_SUPPORTED_RATES);
        }
//      pAd->PortCfg.Pss = PWR_ACTIVE;

        // generate a radom number as BSSID
        MacAddrRandomBssid(pAd, &pAd->PortCfg.Bssid);
        AsicSetBssid(pAd, &pAd->PortCfg.Bssid); 
        AsicSwitchChannel(pAd, pAd->PortCfg.Channel);
        AsicLockChannel(pAd, pAd->PortCfg.Channel);

        DBGPRINT(RT_DEBUG_TRACE, "SYNC - MlmeStartReqAction(ch= %d,supported rate len= %d)\n",
            pAd->PortCfg.Channel, pAd->PortCfg.SupportedRatesLen);

#ifdef  SINGLE_ADHOC_LINKUP
        // Add itself as the entry within BSS table
        Bssidx = BssTableSearch(&pAd->PortCfg.BssTab, &pAd->PortCfg.Bssid);
        if (Bssidx == BSS_NOT_FOUND)
        {
            Bssidx = BssTableSetEntry(pAd, &pAd->PortCfg.BssTab, &pAd->PortCfg.Bssid,
                Ssid, SsidLen, pAd->PortCfg.BssType, pAd->PortCfg.BeaconPeriod, 
                CfExist, &CfParm, pAd->PortCfg.AtimWin, pAd->PortCfg.CapabilityInfo, 
                pAd->PortCfg.SupportedRates, pAd->PortCfg.SupportedRatesLen, TRUE,
                pAd->PortCfg.Channel, Elem->Rssi, TimeStamp, pVIE);
        }
#endif

        pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
        MlmeCntlConfirm(pAd, MT2_START_CONF, (USHORT)MLME_SUCCESS);
    } 
    else 
    {
        printk(KERN_ERR DRV_NAME "SYNC - MlmeStartReqAction() sanity check fail. BUG!!!\n");
        pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
        MlmeCntlConfirm(pAd, MT2_START_CONF, MLME_INVALID_FORMAT);
    }
}

/* 
    ==========================================================================
    Description:
        peer sends beacon back when scanning
    ==========================================================================
 */
VOID PeerBeaconAtScanAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR         Bssid, Addr2;
    UCHAR           Ssid[MAX_LEN_OF_SSID], BssType, Channel, Rates[MAX_LEN_OF_SUPPORTED_RATES], RatesLen, 
                    SsidLen, DtimCount, DtimPeriod, BcastFlag, MessageToMe, Legacy;
    CF_PARM         CfParm;
    USHORT          BeaconPeriod, AtimWin, CapabilityInfo;
    MACFRAME       *Fr;
    LARGE_INTEGER   TimeStamp;
    BOOLEAN         CfExist = FALSE;
    BOOLEAN         ExtendedRateIeExist;
    UCHAR           Erp;
    UCHAR           SupRate[MAX_LEN_OF_SUPPORTED_RATES], ExtRate[MAX_LEN_OF_SUPPORTED_RATES];
	UCHAR           SupRateLen, ExtRateLen;

    // New for WPA security suites
    UCHAR                       VarIE[MAX_VIE_LEN];     // Total VIE length = MAX_VIE_LEN - -5
    NDIS_802_11_VARIABLE_IEs    *pVIE = NULL;

    // NdisFillMemory(Ssid, MAX_LEN_OF_SSID, 0x00);
    Fr = (MACFRAME *) Elem->Msg;
    // Init Variable IE structure
    pVIE = (PNDIS_802_11_VARIABLE_IEs) VarIE;
    pVIE->Length = 0;
    if (PeerBeaconAndProbeRspSanity(pAd, 
                                Elem->Msg, 
                                Elem->MsgLen, 
                                &Addr2, 
                                &Bssid, Ssid, 
                                &SsidLen, 
                                &BssType, 
                                &BeaconPeriod, 
                                &Channel, 
                                &TimeStamp, 
                                &CfExist, 
                                &CfParm, 
                                &AtimWin, 
                                &CapabilityInfo, 
                                Rates, 
                                &RatesLen,
                                &ExtendedRateIeExist,
                                &Erp,
                                &DtimCount, 
                                &DtimPeriod, 
                                &BcastFlag, 
                                &MessageToMe, 
                                &Legacy,
                                SupRate,
                                &SupRateLen,
                                ExtRate,
                                &ExtRateLen,
                                pVIE)) 
    {
        ULONG Idx;
        UCHAR Rssi = 0;

        // This correct im-proper RSSI indication during SITE SURVEY issue.
        // Always report bigger RSSI during SCANNING when receiving multiple BEACONs from the same AP. 
        // This case happens because BEACONs come from adjacent channels, so RSSI become weaker as we 
        // switch to more far away channels.
        Idx = BssTableSearch(&pAd->PortCfg.BssTab, &Bssid);
        if (Idx != BSS_NOT_FOUND) 
            Rssi = pAd->PortCfg.BssTab.BssEntry[Idx].Rssi;
        if (Elem->Rssi > Rssi)
            Rssi = Elem->Rssi;

        DBGPRINT(RT_DEBUG_INFO, "SYNC - PeerBeaconAtScanAction (Subtype=%d, SsidLen=%d, Ssid=%s)\n", Fr->Hdr.SubType, SsidLen,Ssid);

        // Mask out unnecessary capability information
        CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;

        BssTableSetEntry(pAd, &pAd->PortCfg.BssTab, &Bssid, Ssid, SsidLen, BssType, 
                         BeaconPeriod, CfExist, &CfParm, AtimWin, CapabilityInfo, Rates, 
                         RatesLen, ExtendedRateIeExist, Channel, Rssi, TimeStamp, pVIE);
    }
    // sanity check fail, ignored
}

/* 
    ==========================================================================
    Description:
        When waiting joining the (I)BSS, beacon received from external
    ==========================================================================
 */
VOID PeerBeaconAtJoinAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR       Bssid, Addr2;
    UCHAR         Ssid[MAX_LEN_OF_SSID], SsidLen, BssType, Channel, RatesLen, MessageToMe, 
                  Rates[MAX_LEN_OF_SUPPORTED_RATES], DtimCount, DtimPeriod, BcastFlag, Legacy;
    LARGE_INTEGER TimeStamp;
    USHORT        BeaconPeriod, AtimWin, CapabilityInfo;
    CF_PARM       Cf;
    BOOLEAN       CfExist = FALSE, ExtendedRateIeExist;
    UCHAR         Erp;
	UCHAR         SupRate[MAX_LEN_OF_SUPPORTED_RATES], ExtRate[MAX_LEN_OF_SUPPORTED_RATES];
	UCHAR		  SupRateLen, ExtRateLen;

	// New for WPA security suites
	UCHAR						VarIE[MAX_VIE_LEN];		// Total VIE length = MAX_VIE_LEN - -5
	NDIS_802_11_VARIABLE_IEs	*pVIE = NULL;

	// Init Variable IE structure
	pVIE = (PNDIS_802_11_VARIABLE_IEs) VarIE;
	pVIE->Length = 0;
    if (PeerBeaconAndProbeRspSanity(pAd, 
                                Elem->Msg, 
                                Elem->MsgLen, 
                                &Addr2, 
                                &Bssid, 
                                Ssid, 
                                &SsidLen, 
                                &BssType, 
                                &BeaconPeriod, 
                                &Channel, 
                                &TimeStamp, 
                                &CfExist, 
                                &Cf, 
                                &AtimWin, 
                                &CapabilityInfo, 
                                Rates, 
                                &RatesLen,
                                &ExtendedRateIeExist,
                                &Erp,
                                &DtimCount, 
                                &DtimPeriod, 
                                &BcastFlag, 
                                &MessageToMe, 
                                &Legacy,
								SupRate,
								&SupRateLen,
								ExtRate,
								&ExtRateLen,
                                pVIE)) 
    {
		// Disqualify 11b only adhoc when we are in 11g only adhoc mode
		if ((BssType == BSS_INDEP) && (pAd->PortCfg.AdhocMode == 2) && (RatesLen < 12))
			return;
		
		if (MAC_ADDR_EQUAL(&pAd->Mlme.SyncAux.Bssid, &Bssid))
        {
            DBGPRINT(RT_DEBUG_TRACE, "SYNC - receive desired BEACON at JoinWaitBeacon...\n");
            RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);

			// Update RSSI to prevent No signal display when cards first initialized
            pAd->PortCfg.LastRssi = Elem->Rssi;
			pAd->PortCfg.AvgRssi  = Elem->Rssi;
			
            if (pAd->Mlme.SyncAux.SsidLen > 0)
            {
            	memcpy(pAd->PortCfg.Ssid, pAd->Mlme.SyncAux.Ssid, pAd->Mlme.SyncAux.SsidLen);
            	pAd->PortCfg.SsidLen = pAd->Mlme.SyncAux.SsidLen;
            }
			else
			{
            	memcpy(pAd->PortCfg.Ssid, Ssid, SsidLen);
            	pAd->PortCfg.SsidLen = SsidLen;
			}
        
            COPY_MAC_ADDR(&pAd->PortCfg.Bssid, &Bssid);
            AsicSetBssid(pAd, &pAd->PortCfg.Bssid);

            pAd->PortCfg.BssType = BssType;
            pAd->PortCfg.BeaconPeriod = BeaconPeriod;
            pAd->PortCfg.Channel = Channel;

            // filter out non-supported rates
            {
                int i;
                pAd->PortCfg.SupportedRatesLen = 0;
                for (i=0;i<RatesLen;i++)
                {
                    UCHAR Rate = Rates[i] & 0x7f;
                    if ((pAd->PortCfg.PhyMode == PHY_11B) &&
                        (Rate == 2 || Rate == 4 || Rate == 11 || Rate == 22))
                    {
                        ///DBGPRINT(RT_DEBUG_TRACE, ("SYNC - Supported Rate[%d] = 0x%02x\n",pAd->PortCfg.SupportedRatesLen, Rates[i]));
                        pAd->PortCfg.SupportedRates[pAd->PortCfg.SupportedRatesLen] = Rates[i];
                        pAd->PortCfg.SupportedRatesLen ++;
                    }
                    else if ((Rate == 2 || Rate == 4 || Rate == 11 || Rate == 22) ||
                             (Rate == 12 || Rate == 18 || Rate == 24 || Rate == 36) ||
                             (Rate == 48 || Rate == 72 || Rate == 96 || Rate == 108))
                    {
                        // DBGPRINT(RT_DEBUG_TRACE, ("SYNC - Supported Rate[%d] = 0x%02x\n",pAd->PortCfg.SupportedRatesLen, Rates[i]));
                        pAd->PortCfg.SupportedRates[pAd->PortCfg.SupportedRatesLen] = Rates[i];
                        pAd->PortCfg.SupportedRatesLen ++;
                    }
                }
            }

            // Copy AP's supported rate to portcfg for creating assoication request
            // Also filter out not supported rate
            // Supported rate
            {
                int i;
                pAd->PortCfg.SupRateLen = 0;
                for (i = 0; i < SupRateLen; i++)
                {
                    UCHAR Rate = SupRate[i] & 0x7f;
                    if ((pAd->PortCfg.PhyMode == PHY_11B) &&
                        (Rate == 2 || Rate == 4 || Rate == 11 || Rate == 22))
                    {
                        pAd->PortCfg.SupRate[pAd->PortCfg.SupRateLen] = SupRate[i];
                        pAd->PortCfg.SupRateLen ++;
                    }
                    else if ((Rate == 2 || Rate == 4 || Rate == 11 || Rate == 22) ||
                             (Rate == 12 || Rate == 18 || Rate == 24 || Rate == 36) ||
                             (Rate == 48 || Rate == 72 || Rate == 96 || Rate == 108))
                    {
                        pAd->PortCfg.SupRate[pAd->PortCfg.SupRateLen] = SupRate[i];
                        pAd->PortCfg.SupRateLen ++;
                    }
                }
            }

            // Copy AP's supported rate to portcfg for creating assoication request
            // Also filter out not supported rate
            // Extended rate
            if (ExtendedRateIeExist == TRUE)
            {
                int i;
                pAd->PortCfg.ExtRateLen = 0;
                for (i = 0; i < ExtRateLen; i++)
                {
                    UCHAR Rate = ExtRate[i] & 0x7f;
                    if ((pAd->PortCfg.PhyMode == PHY_11B) &&
                        (Rate == 2 || Rate == 4 || Rate == 11 || Rate == 22))
                    {
                        pAd->PortCfg.ExtRate[pAd->PortCfg.ExtRateLen] = ExtRate[i];
                        pAd->PortCfg.ExtRateLen ++;
                    }
                    else if ((Rate == 2 || Rate == 4 || Rate == 11 || Rate == 22) ||
                             (Rate == 12 || Rate == 18 || Rate == 24 || Rate == 36) ||
                             (Rate == 48 || Rate == 72 || Rate == 96 || Rate == 108))
                    {
                        pAd->PortCfg.ExtRate[pAd->PortCfg.ExtRateLen] = ExtRate[i];
                        pAd->PortCfg.ExtRateLen ++;
                    }
                }
            }
			else
			{
                pAd->PortCfg.ExtRateLen = 0;
			}
			
            DBGPRINT(RT_DEBUG_TRACE, "SYNC - AP's SupportedRatesLen=%d, set STA's SupportedRateLen=%d\n", 
                RatesLen, pAd->PortCfg.SupportedRatesLen);
            
			// Mask out unnecessary capability information
			CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;
			
            // Check for 802.11g information, if 802.11 b/g mixed mode.
            // We can't support its short preamble for now.
           	pAd->PortCfg.CapabilityInfo = CapabilityInfo;

            if ((BssType == BSS_INDEP) && (CAP_IS_IBSS_ON(CapabilityInfo))) 
            {
                pAd->PortCfg.AtimWin = AtimWin;
            } 
            else if (BssType == BSS_INFRA) 
            {
                pAd->PortCfg.CfpPeriod = Cf.CfpPeriod;
                pAd->PortCfg.CfpMaxDuration = Cf.CfpMaxDuration;
                pAd->PortCfg.CfpDurRemain = Cf.CfpDurRemaining;
                pAd->PortCfg.CfpCount = Cf.CfpCount;
                pAd->PortCfg.CfpPeriod = Cf.CfpPeriod;

                AsicEnableBssSync(pAd);
            }

            pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
            MlmeCntlConfirm(pAd, MT2_JOIN_CONF, MLME_SUCCESS);
        }
        // not to me BEACON, ignored
    } 
    // sanity check fail, ignore this frame
}

/* 
    ==========================================================================
    Description:
        receive BEACON from peer
    ==========================================================================
 */
VOID PeerBeacon(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR       Bssid, Addr2;
    CHAR          Ssid[MAX_LEN_OF_SSID];
    CF_PARM       CfParm;
    UCHAR         SsidLen, MessageToMe=0, BssType, Channel, Rates[MAX_LEN_OF_SUPPORTED_RATES];
    UCHAR         RatesLen, DtimCount=0, DtimPeriod=0, BcastFlag=0, Legacy;
    USHORT        CapabilityInfo, AtimWin, BeaconPeriod;
    LARGE_INTEGER TimeStamp;
    BOOLEAN       CfExist = FALSE;
    USHORT        TbttNumToNextWakeUp;
    BOOLEAN       ExtendedRateIeExist;
    UCHAR         Erp;
    UCHAR         SupRate[MAX_LEN_OF_SUPPORTED_RATES], ExtRate[MAX_LEN_OF_SUPPORTED_RATES];
	UCHAR		  SupRateLen, ExtRateLen;

    // New for WPA security suites
    UCHAR                       VarIE[MAX_VIE_LEN];     // Total VIE length = MAX_VIE_LEN - -5
    NDIS_802_11_VARIABLE_IEs    *pVIE = NULL;

    if (!INFRA_ON(pAd) && !ADHOC_ON(pAd))
        return;

    // Init Variable IE structure
    pVIE = (PNDIS_802_11_VARIABLE_IEs) VarIE;
    pVIE->Length = 0;
    if (PeerBeaconAndProbeRspSanity(pAd, 
                                Elem->Msg, 
                                Elem->MsgLen, 
                                &Addr2, 
                                &Bssid, 
                                Ssid, 
                                &SsidLen, 
                                &BssType, 
                                &BeaconPeriod, 
                                &Channel, 
                                &TimeStamp, 
                                &CfExist, 
                                &CfParm, 
                                &AtimWin, 
                                &CapabilityInfo, 
                                Rates, 
                                &RatesLen,
                                &ExtendedRateIeExist,
                                &Erp,
                                &DtimCount, 
                                &DtimPeriod, 
                                &BcastFlag, 
                                &MessageToMe, 
                                &Legacy,
                                SupRate,
                                &SupRateLen,
                                ExtRate,
                                &ExtRateLen,
                                pVIE)) 
    {
        BOOLEAN is_my_bssid, is_my_ssid;
        ULONG   Bssidx, Now;
        BSS_ENTRY *pBss;

        is_my_bssid = (MAC_ADDR_EQUAL(&Bssid, &pAd->PortCfg.Bssid) ? TRUE : FALSE);
        is_my_ssid = (((pAd->PortCfg.SsidLen == SsidLen) && RTMPEqualMemory(Ssid, pAd->PortCfg.Ssid, (ULONG) SsidLen)) ? TRUE : FALSE);
        // Mask out unnecessary capability information
        CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;

        // ignore BEACON not for my SSID
        if ((! is_my_ssid) && (! is_my_bssid))
            return;

		//
        // Housekeeping "SsidBssTab" table for later-on ROAMing usage. 
        //
        Bssidx = BssTableSearch(&pAd->Mlme.CntlAux.SsidBssTab, &Bssid);
        if (Bssidx == BSS_NOT_FOUND)
        {
			// Return immediately when in transition process when changing association
			// Found this bug when doing WHQL ad-hoc test case
			if (pAd->PortCfg.SsidLen != pAd->Mlme.CntlAux.SsidLen)
				return;
			if (!RTMPEqualMemory(pAd->PortCfg.Ssid, pAd->Mlme.CntlAux.Ssid, pAd->PortCfg.SsidLen))
				return;
			
            // discover new AP of this network, create BSS entry
            Bssidx = BssTableSetEntry(pAd, &pAd->Mlme.CntlAux.SsidBssTab, &Bssid, Ssid, SsidLen, 
                        BssType, BeaconPeriod, CfExist, &CfParm, AtimWin, CapabilityInfo, 
                        Rates, RatesLen, ExtendedRateIeExist, Channel, Elem->Rssi, TimeStamp, pVIE);

            if (Bssidx == BSS_NOT_FOUND) // return if BSS table full
                return;  

            DBGPRINT(RT_DEBUG_TRACE, "SYNC - New AP added to SsidBssTab[%d], RSSI=%d, MAC=%02x:%02x:%02x:%02x:%02x:%02x\n", 
                Bssidx, Elem->Rssi, Bssid.Octet[0], Bssid.Octet[1], Bssid.Octet[2], 
                Bssid.Octet[3], Bssid.Octet[4], Bssid.Octet[5]);
        }

        // if the ssid matched & bssid unmatched, we should select the bssid with large value.
        // This might happened when two STA start at the same time
        if (is_my_ssid && (! is_my_bssid) && ADHOC_ON(pAd))
        {
            INT i;
			// Add to safe guard adhoc wep status mismatch
			if (pAd->PortCfg.WepStatus != pAd->Mlme.CntlAux.SsidBssTab.BssEntry[Bssidx].WepStatus)
				return;

            // link down the one with smaller BSSID value.
            for (i = 0; i < 6; i++)
            {
                if (Bssid.Octet[i] > pAd->PortCfg.Bssid.Octet[i])
                {
                    AsicDisableSync(pAd);
                    memcpy(&pAd->PortCfg.Bssid, &Bssid, 6);
                    AsicSetBssid(pAd, &pAd->PortCfg.Bssid); 
                    MakeIbssBeacon(pAd);
                    AsicEnableIbssSync(pAd);
                    break;
                }
            }
        }

        DBGPRINT(RT_DEBUG_INFO, "SYNC - PeerBeacon from %02x:%02x:%02x:%02x:%02x:%02x - Dtim=%d/%d, Rssi=%02x\n", 
            Bssid.Octet[0], Bssid.Octet[1], Bssid.Octet[2], 
            Bssid.Octet[3], Bssid.Octet[4], Bssid.Octet[5], 
            DtimCount, DtimPeriod, Elem->Rssi);

        Now = jiffies;
        pBss = &pAd->Mlme.CntlAux.SsidBssTab.BssEntry[Bssidx];
        pBss->Rssi = Elem->Rssi;       // lastest RSSI
        pBss->LastBeaconRxTime = Now;   // last RX timestamp

        //
        // BEACON from my BSSID - either IBSS or INFRA network
        // 
        if (is_my_bssid)
        {
            // 2002/12/06 - patch Abocom AP bug, which forgets to set "Privacy" bit in 
            // AssocRsp even though this bit is ON in Beacon. So we update according 
            // to following Beacon frame.
            // pAd->PortCfg.PrivacyInvoked = CAP_IS_PRIVACY_ON(CapabilityInfo);
            
            pAd->PortCfg.LastBeaconRxTime = Now;
#if 1
            // at least one 11b peer joined. downgrade the MaxTxRate to 11Mbps
            // after last 11b peer left for several seconds, we'll auto switch back to 11G rate
            // in MlmePeriodicExec()
            if (ADHOC_ON(pAd) && (RatesLen <= 4))   
            {
                // this timestamp is for MlmePeriodicExec() to check if all 11B peers have left
                pAd->PortCfg.Last11bBeaconRxTime = Now;
                
                if (pAd->PortCfg.MaxTxRate > RATE_11)
                {
                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - 11b peer joined. down-grade to 11b TX rates \n");
                    memcpy(pAd->PortCfg.SupportedRates, Rates, MAX_LEN_OF_SUPPORTED_RATES);
                    pAd->PortCfg.SupportedRatesLen = RatesLen;
                    MlmeUpdateTxRates(pAd, FALSE);
                    MakeIbssBeacon(pAd);        // supported rates changed
                }
            }
#endif
            // check if RSSI reaches threshold
            pAd->PortCfg.LastRssi = (pAd->PortCfg.LastRssi + Elem->Rssi) / 2;
            pAd->PortCfg.AvgRssi  = (pAd->PortCfg.AvgRssi * 7 + Elem->Rssi) >> 3;
            if ((pAd->PortCfg.RssiTriggerMode == RSSI_TRIGGERED_UPON_BELOW_THRESHOLD) &&
                (pAd->PortCfg.LastRssi < pAd->PortCfg.RssiTrigger))
            {
                // NDIS_802_11_RSSI Dbm = pAd->PortCfg.LastRssi - RSSI_TO_DBM_OFFSET;
                // DBGPRINT(RT_DEBUG_TRACE, "SYNC - NdisMIndicateStatus *** RSSI %d dBm, less than threshold %d dBm\n", 
                //     Dbm, pAd->PortCfg.RssiTrigger - RSSI_TO_DBM_OFFSET);
            }
            else if ((pAd->PortCfg.RssiTriggerMode == RSSI_TRIGGERED_UPON_EXCCEED_THRESHOLD) &&
                (pAd->PortCfg.LastRssi > pAd->PortCfg.RssiTrigger))
            {
                // NDIS_802_11_RSSI Dbm = pAd->PortCfg.LastRssi - RSSI_TO_DBM_OFFSET;
                // DBGPRINT(RT_DEBUG_TRACE, "SYNC - NdisMIndicateStatus *** RSSI %d dBm, greater than threshold %d dBm\n", 
                //     Dbm, pAd->PortCfg.RssiTrigger - RSSI_TO_DBM_OFFSET);
            }

            if (INFRA_ON(pAd)) // && (pAd->PortCfg.PhyMode == PHY_11BG_MIXED))
            {
                BOOLEAN bUseShortSlot, bUseBGProtection;
                
                // decide to use/change to - 
                //      1. long slot (20 us) or short slot (9 us) time
                //      2. turn on/off RTS/CTS and/or CTS-to-self protection
                //      3. short preamble
                bUseShortSlot = (pAd->PortCfg.UseShortSlotTime == TRUE) && CAP_IS_SHORT_SLOT_TIME(CapabilityInfo);
                if (bUseShortSlot != pAd->PortCfg.ShortSlotInUsed)
                    AsicSetSlotTime(pAd, bUseShortSlot);

                bUseBGProtection = (pAd->PortCfg.UseBGProtection == 1) ||    // always use
                                   ((pAd->PortCfg.UseBGProtection == 0) && ERP_IS_USE_PROTECTION(Erp));
                if (bUseBGProtection != pAd->PortCfg.BGProtectionInUsed)
                {
                    pAd->PortCfg.BGProtectionInUsed = bUseBGProtection;
                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - AP changed B/G protection to %d\n", bUseBGProtection);
                }

                if ((pAd->PortCfg.TxPreambleInUsed == Rt802_11PreambleShort) && ERP_IS_USE_BARKER_PREAMBLE(Erp))
                {
                    MlmeSetTxPreamble(pAd, Rt802_11PreambleLong);
                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - AP forced to use LONG preamble\n");
                }
            }

            // only INFRASTRUCTURE mode support power-saving feature
            if (INFRA_ON(pAd) && (pAd->PortCfg.Psm == PWR_SAVE)) 
            {
                //  1. AP has backlogged unicast-to-me frame, stay AWAKE, send PSPOLL
                //  2. AP has backlogged broadcast/multicast frame and we want those frames, stay AWAKE
                //  3. we have outgoing frames in TxRing or PrioRing, better stay AWAKE
                //  4. Psm change to PWR_SAVE, but AP not been informed yet, we better stay AWAKE
                //  5. otherwise, put PHY back to sleep to save battery.
                if (MessageToMe)
                {
                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - AP backlog unicast-to-me, stay AWAKE, send PSPOLL\n");
                    EnqueuePsPoll(pAd);
                }
                else if (BcastFlag && (DtimCount == 0) && pAd->PortCfg.RecvDtim)
                {
                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - AP backlog broadcast/multicast, stay AWAKE\n");
                } 
                else if ((RTMPFreeDescriptorRequest(pAd, TX_RING, TX_RING_SIZE) != NDIS_STATUS_SUCCESS) ||
                    (RTMPFreeDescriptorRequest(pAd, PRIO_RING, PRIO_RING_SIZE) != NDIS_STATUS_SUCCESS))
                {
                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - outgoing frame in TxRing/PrioRing, stay AWAKE\n");
                }
                else 
                {
                    USHORT NextDtim = DtimCount;

                    if (NextDtim == 0) 
                        NextDtim = DtimPeriod;

                    TbttNumToNextWakeUp = pAd->PortCfg.DefaultListenCount;
                    if (pAd->PortCfg.RecvDtim && (TbttNumToNextWakeUp > NextDtim))
                        TbttNumToNextWakeUp = NextDtim;

                    DBGPRINT(RT_DEBUG_TRACE, "SYNC - PHY sleeps for %d Tbcn, Dtim=%d/%d\n", TbttNumToNextWakeUp, DtimCount, DtimPeriod);
                    AsicSleepThenAutoWakeup(pAd, TbttNumToNextWakeUp);
                }
            }

#ifndef SINGLE_ADHOC_LINKUP
            // At least another peer in this IBSS, declare MediaState as CONNECTED
            if (ADHOC_ON(pAd) && (pAd->MediaState == NdisMediaStateDisconnected))
            {
                pAd->MediaState = NdisMediaStateConnected;

                // 2003/03/12 - john
                // Make sure this entry in "PortCfg.BssTab" table, thus complies to Microsoft's policy that
                // "site survey" result should always include the current connected network. 
                //
                Bssidx = BssTableSearch(&pAd->PortCfg.BssTab, &Bssid);
                if (Bssidx == BSS_NOT_FOUND)
                {
                    Bssidx = BssTableSetEntry(pAd, &pAd->PortCfg.BssTab, &Bssid, Ssid, SsidLen, 
                                BssType, BeaconPeriod, CfExist, &CfParm, AtimWin, CapabilityInfo, 
                                Rates, RatesLen, ExtendedRateIeExist, Channel, Elem->Rssi, TimeStamp, pVIE);
                }
            }
#endif
        }
        // not my BSSID, ignore it
    }
    // sanity check fail, ignore this frame
}

/* 
    ==========================================================================
    Description:
        Receive PROBE REQ from remote peer when operating in IBSS mode
    ==========================================================================
 */
VOID PeerProbeReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR       Addr2;
    CHAR          Ssid[MAX_LEN_OF_SSID];
    UCHAR         SsidLen;
    MACHDR        ProbeRspHdr;
    NDIS_STATUS   NStatus;
    UCHAR         *OutBuffer = NULL;
    ULONG         FrameLen = 0;
    LARGE_INTEGER FakeTimestamp;
    UCHAR         SsidIe = IE_SSID, DsIe = IE_DS_PARM, IbssIe = IE_IBSS_PARM, SuppIe = IE_SUPP_RATES, 
                  DsLen = 1, IbssLen = 2;
    UCHAR         SupportedRatesLen;
    UCHAR         SupportedRates[MAX_LEN_OF_SUPPORTED_RATES];
    UCHAR         ExtRateIe = IE_EXT_SUPP_RATES, ExtRatesLen;
    UCHAR         ErpIe[3] = {IE_ERP, 1, 0};
    
    if (! ADHOC_ON(pAd))
        return;

    if (PeerProbeReqSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, Ssid, &SsidLen)) //, Rates, &RatesLen))
    {
        if ((SsidLen == 0) || RTMPEqualMemory(Ssid, pAd->PortCfg.Ssid, (ULONG) SsidLen))
        {
            CSR15_STRUC Csr15;
            
            // we should respond a ProbeRsp only when we're the last BEACON transmitter 
            // in this ADHOC network.
            RTMP_IO_READ32(pAd, CSR15, &Csr15.word);
            if (Csr15.field.BeaconSent == 0)
            {
                DBGPRINT(RT_DEBUG_INFO, "SYNC - NOT last BEACON sender, no PROBE_RSP to %02x:%02x:%02x:%02x:%02x:%02x...\n",
                    Addr2.Octet[0],Addr2.Octet[1],Addr2.Octet[2],Addr2.Octet[3],Addr2.Octet[4],Addr2.Octet[5] );
                return;
            }

            // 2003-12-10 802.11g WIFI spec disallow OFDM rates in 802.11g ADHOC mode
            //            make sure 1,2,5.5,11 are the firt 4 rates in PortCfg.SupportedRates[] array
			if ((pAd->PortCfg.PhyMode == PHY_11BG_MIXED) && (pAd->PortCfg.AdhocMode == 0))
            {
                int i;
                SupportedRatesLen=0;
                for (i=0;i<pAd->PortCfg.SupportedRatesLen;i++)
                {
                    switch (pAd->PortCfg.SupportedRates[i] & 0x7f)
                    {
                        case 2:
                        case 4:
                        case 11:
                        case 22:
                            SupportedRates[SupportedRatesLen] = pAd->PortCfg.SupportedRates[i];
                            SupportedRatesLen ++;
                            break;
                        default:
                            break;
                    }
                }
                // error handling - should never happen
                if (SupportedRatesLen != 4)
                {
                    SupportedRatesLen = 4;
                    SupportedRates[0] = 0x82;
                    SupportedRates[1] = 0x84;
                    SupportedRates[2] = 0x8b;
                    SupportedRates[3] = 0x96;
                }
            }
            else
            {
                SupportedRatesLen = pAd->PortCfg.SupportedRatesLen;
                memcpy(SupportedRates, pAd->PortCfg.SupportedRates, SupportedRatesLen);
            }

            // allocate and send out ProbeRsp frame
            NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
            if (NStatus != NDIS_STATUS_SUCCESS)
                return;

            pAd->PortCfg.AtimWin = 0;  // ??????
            DBGPRINT(RT_DEBUG_TRACE, "SYNC - Send PROBE_RSP to %02x:%02x:%02x:%02x:%02x:%02x...\n", 
                Addr2.Octet[0],Addr2.Octet[1],Addr2.Octet[2],Addr2.Octet[3],Addr2.Octet[4],Addr2.Octet[5] );
            MgtMacHeaderInit(pAd, &ProbeRspHdr, SUBTYPE_PROBE_RSP, 0, &Addr2, &pAd->PortCfg.Bssid);

            if (SupportedRatesLen <= 8)
            {
                MakeOutgoingFrame(OutBuffer,                        &FrameLen, 
                              MAC_HDR_LEN,                      &ProbeRspHdr, 
                              TIMESTAMP_LEN,                    &FakeTimestamp,
                              2,                                &pAd->PortCfg.BeaconPeriod,
                              2,                                &pAd->PortCfg.CapabilityInfo,
                              1,                                &SsidIe, 
                              1,                                &pAd->PortCfg.SsidLen, 
                              pAd->PortCfg.SsidLen,             pAd->PortCfg.Ssid,
                              1,                                &SuppIe, 
                              1,                                &SupportedRatesLen,
                              SupportedRatesLen,                SupportedRates, 
                              1,                                &DsIe, 
                              1,                                &DsLen, 
                              1,                                &pAd->PortCfg.Channel,
                              1,                                &IbssIe, 
                              1,                                &IbssLen, 
                              2,                                &pAd->PortCfg.AtimWin,
                              END_OF_ARGS);
            }
            else
            {
                ExtRatesLen = SupportedRatesLen - 8;
                SupportedRatesLen = 8;
                MakeOutgoingFrame(OutBuffer,                        &FrameLen, 
                              MAC_HDR_LEN,                      &ProbeRspHdr, 
                              TIMESTAMP_LEN,                    &FakeTimestamp,
                              2,                                &pAd->PortCfg.BeaconPeriod,
                              2,                                &pAd->PortCfg.CapabilityInfo,
                              1,                                &SsidIe, 
                              1,                                &pAd->PortCfg.SsidLen, 
                              pAd->PortCfg.SsidLen,             pAd->PortCfg.Ssid,
                              1,                                &SuppIe, 
                              1,                                &SupportedRatesLen,
                              SupportedRatesLen,                SupportedRates, 
                              1,                                &DsIe, 
                              1,                                &DsLen, 
                              1,                                &pAd->PortCfg.Channel,
                              1,                                &IbssIe, 
                              1,                                &IbssLen, 
                              2,                                &pAd->PortCfg.AtimWin,
                              3,                                ErpIe,
                              1,                                &ExtRateIe,
                              1,                                &ExtRatesLen,
                              ExtRatesLen,                      &SupportedRates[SupportedRatesLen],
                              END_OF_ARGS);
            }
			// If adhoc secruity is set for WPA-None, append the cipher suite IE
			if (pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
			{
				ULONG	tmp;
				UCHAR	WpaIe = IE_WPA;
				
				if (pAd->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) 	// Tkip
				{
				MakeOutgoingFrame(OutBuffer + FrameLen,			&tmp,
						1,						   &WpaIe,
						1,						  	&CipherSuiteWpaNoneTkipLen,
						CipherSuiteWpaNoneTkipLen,  &CipherSuiteWpaNoneTkip[0],
						END_OF_ARGS);
				FrameLen += tmp;
			}
				else if (pAd->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)	// Aes
				{
					MakeOutgoingFrame(OutBuffer + FrameLen, 		&tmp,
							1,						   &WpaIe,
							1,							&CipherSuiteWpaNoneAesLen,
							CipherSuiteWpaNoneAesLen,	&CipherSuiteWpaNoneAes[0],
							END_OF_ARGS);
					FrameLen += tmp;
				}
			}                
            MiniportMMRequest(pAd, OutBuffer, FrameLen);
        }
    }
}

VOID BeaconTimeoutAtJoinAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "SYNC - BeaconTimeoutAtJoinAction\n");
    pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
    MlmeCntlConfirm(pAd, MT2_JOIN_CONF, MLME_REJ_TIMEOUT);
}

/* 
    ==========================================================================
    Description:
        Scan timeout procedure. basically add channel index by 1 and rescan
    ==========================================================================
 */
VOID ScanTimeoutAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    pAd->Mlme.SyncAux.Channel = NextChannel(pAd, pAd->Mlme.SyncAux.Channel);
    ScanNextChannel(pAd);
}

/* 
    ==========================================================================
    Description:
        Scan next channel
    ==========================================================================
 */
VOID ScanNextChannel(
    IN PRTMP_ADAPTER pAd) 
{
    MACHDR          Hdr;
    UCHAR           SsidIe = IE_SSID, SuppRateIe = IE_SUPP_RATES;
    VOID           *OutBuffer = NULL;
    VOID           *OutBuffer2 = NULL;
    NDIS_STATUS     NStatus;
    ULONG           FrameLen = 0;
    UCHAR           SsidLen = 0;

    if (pAd->Mlme.SyncAux.Channel == 0) 
    {
        DBGPRINT(RT_DEBUG_INFO, "SYNC - End of SCAN, restore to channel %d\n",pAd->PortCfg.Channel);
        AsicSwitchChannel(pAd, pAd->PortCfg.Channel);
        AsicLockChannel(pAd, pAd->PortCfg.Channel);
        
        pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
        MlmeCntlConfirm(pAd, MT2_SCAN_CONF, MLME_SUCCESS);
    } 
    else 
    {
        AsicSwitchChannel(pAd, pAd->Mlme.SyncAux.Channel);

        // Total SCAN time still limits within 3 sec (DDK constraint). 
        // TODO: We need more intelligent rules here to further improve out-of-service issue.
        // e.g. temporary stop copying NDIS packet to TxRing until SCAN complete
//      if (INFRA_ON(pAd) || ADHOC_ON(pAd))

		// We need to shorten active scan time in order for WZC connect issue
        if (pAd->Mlme.SyncAux.ScanType == SCAN_ACTIVE) 
            RTMPSetTimer(pAd, &pAd->Mlme.SyncAux.ScanTimer, ACTIVE_SCAN_TIME); 
        else if (pAd->PortCfg.PhyMode == PHY_11ABG_MIXED)
            RTMPSetTimer(pAd, &pAd->Mlme.SyncAux.ScanTimer, MIN_CHANNEL_TIME); 
        else
            RTMPSetTimer(pAd, &pAd->Mlme.SyncAux.ScanTimer, MAX_CHANNEL_TIME);

		MgtMacHeaderInit(pAd, &Hdr, SUBTYPE_PROBE_REQ, 0, &pAd->PortCfg.Broadcast, &pAd->PortCfg.Broadcast);
		// There is no need to send broadcast probe request if active scan is in effect.
		// The same rulr should apply to passive scan also.
        if (pAd->Mlme.SyncAux.ScanType == SCAN_PASSIVE) 
        {
			// Send the first probe request with empty SSID
            NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
            if (NStatus != NDIS_STATUS_SUCCESS)
            {
                DBGPRINT(RT_DEBUG_INFO, "SYNC - ScanNextChannel() allocate memory fail\n");
                pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
                MlmeCntlConfirm(pAd, MT2_SCAN_CONF, MLME_FAIL_NO_RESOURCE);
                return;
            }

        	DBGPRINT(RT_DEBUG_INFO, "SYNC - send passive ProbeReq @ channel=%d...\n", pAd->Mlme.SyncAux.Channel);
            SsidLen = 0;
            MakeOutgoingFrame(OutBuffer,        &FrameLen,
                          sizeof(MACHDR),   (UCHAR*)&Hdr,
                              1,                &SsidIe,
                              1,                &SsidLen,  
                              1,                &SuppRateIe,
                              1,                &pAd->PortCfg.SupportedRatesLen,
                              pAd->PortCfg.SupportedRatesLen, pAd->PortCfg.SupportedRates, 
                              END_OF_ARGS);
                              
            MiniportMMRequest(pAd, OutBuffer, FrameLen);
        }
        else if (pAd->Mlme.SyncAux.ScanType == SCAN_ACTIVE) 
        {
            // Allocate another for probe scan with SSID
            NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer2);  //Get an unused nonpaged memory
            if (NStatus != NDIS_STATUS_SUCCESS)
            {
                DBGPRINT(RT_DEBUG_TRACE, "SYNC - ScanNextChannel() allocate memory fail\n");
                pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
                MlmeCntlConfirm(pAd, MT2_SCAN_CONF, MLME_FAIL_NO_RESOURCE);
                return;
            }
            // make another probe scan with SSID from mlme.cntlaux.ssid
			SsidLen = pAd->PortCfg.SsidLen;
            MakeOutgoingFrame(OutBuffer2,       &FrameLen,
                              sizeof(MACHDR),   &Hdr,
                              1,                &SsidIe,
                              1,                &SsidLen,
                              SsidLen,			pAd->PortCfg.Ssid,
                              1,                &SuppRateIe,
                              1,                &pAd->PortCfg.SupportedRatesLen,
                              pAd->PortCfg.SupportedRatesLen, pAd->PortCfg.SupportedRates, 
                              END_OF_ARGS);
                              
            MiniportMMRequest(pAd, OutBuffer2, FrameLen);

 			DBGPRINT(RT_DEBUG_INFO, "SYNC - send active ProbeReq @ channel=%d with essid=%s\n", pAd->Mlme.SyncAux.Channel, pAd->PortCfg.Ssid);
        }

        pAd->Mlme.SyncMachine.CurrState = SCAN_LISTEN;
    }
}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID InvalidStateWhenScan(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "AYNC - InvalidStateWhenScan(state=%d). Reset SYNC machine\n", pAd->Mlme.SyncMachine.CurrState);
    pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
    MlmeCntlConfirm(pAd, MT2_SCAN_CONF, MLME_STATE_MACHINE_REJECT);
}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID InvalidStateWhenJoin(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "InvalidStateWhenJoin(state=%d). Reset SYNC machine\n", pAd->Mlme.SyncMachine.CurrState);
    pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
    MlmeCntlConfirm(pAd, MT2_JOIN_CONF, MLME_STATE_MACHINE_REJECT);
}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID InvalidStateWhenStart(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "InvalidStateWhenStart(state=%d). Reset SYNC machine\n", pAd->Mlme.SyncMachine.CurrState);
    pAd->Mlme.SyncMachine.CurrState = SYNC_IDLE;
    MlmeCntlConfirm(pAd, MT2_START_CONF, MLME_STATE_MACHINE_REJECT);
}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID EnqueuePsPoll(
    IN PRTMP_ADAPTER pAd) 
{
    NDIS_STATUS    NState;
    PSPOLL_FRAME   *PsFr;

    DBGPRINT(RT_DEBUG_TRACE, "SYNC - send PsPoll ...\n");

    NState = MlmeAllocateMemory(pAd, (PVOID)&PsFr);  //Get an unused nonpaged memory
    if (NState == NDIS_STATUS_SUCCESS)
    {
        memcpy((VOID *)PsFr, (VOID *)&pAd->Mlme.PsFr, sizeof(PSPOLL_FRAME));
        MiniportMMRequest(pAd, (VOID *)PsFr, sizeof(PSPOLL_FRAME));
    }
}

// 2003-04-17 john
// driver force send out a BEACON frame to cover ADHOC mode BEACON starving issue
// that is, in ADHOC mode, driver guarantee itself can send out at least a BEACON
// per a specified duration, even the peer's clock is faster than us and win all the
// hardware-based BEACON TX oppertunity. 
// we may remove this software feature once 2560 IC fix this problem in ASIC.
VOID EnqueueBeaconFrame(
    IN PRTMP_ADAPTER pAd) 
{
    NDIS_STATUS    NState;
    PTXD_STRUC     pTxD = (PTXD_STRUC)pAd->BeaconRing.va_addr;
    CHAR           *pBeacon;
    LARGE_INTEGER  Tsf;

    NState = MlmeAllocateMemory(pAd, (PVOID)&pBeacon);  //Get an unused nonpaged memory
    if (NState == NDIS_STATUS_SUCCESS)
    {
        DBGPRINT(RT_DEBUG_TRACE, "SYNC - driver sent BEACON (len=%d)...\n",pTxD->DataByteCnt);
        RTMP_IO_READ32(pAd, CSR17, &Tsf.vv.HighPart);
        RTMP_IO_READ32(pAd, CSR16, &Tsf.vv.LowPart);
        memcpy(pBeacon, pAd->BeaconRing.va_data_addr, pTxD->DataByteCnt);
        memcpy(pBeacon + MAC_HDR_LEN, &Tsf, TIMESTAMP_LEN);
        MiniportMMRequest(pAd, (VOID *)pBeacon, pTxD->DataByteCnt);
    }
}

/* 
    ==========================================================================
    Description:
        Send out a NULL frame to AP. The prpose is to inform AP this client 
        current PSM bit.
    NOTE:
        This routine should only be used in infrastructure mode.
    ==========================================================================
 */
VOID EnqueueNullFrame(
    IN PRTMP_ADAPTER pAd,
    IN UCHAR         TxRate) 
{
    NDIS_STATUS    NState;
    MACHDR         *NullFr;

    // since TxRate may change, we have to change Duration each time
    pAd->Mlme.NullFr.Duration = RTMPCalcDuration(pAd, TxRate, 14);
    NState = MlmeAllocateMemory(pAd, (PVOID)&NullFr);  //Get an unused nonpaged memory
    if (NState == NDIS_STATUS_SUCCESS)
    {
        memcpy((VOID *)NullFr, (VOID *)&pAd->Mlme.NullFr, sizeof(MACHDR));
        RTMPSendNullFrame(pAd, (VOID *)NullFr, sizeof(MACHDR), TxRate);
    }
}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID EnqueueProbeRequest(
    IN PRTMP_ADAPTER pAd)
{
    NDIS_STATUS     NState;
    UCHAR           SsidIe = IE_SSID, SuppRateIe = IE_SUPP_RATES;
    VOID            *OutBuffer;
    ULONG           FrameLen = 0;
    MACHDR          Hdr;

    DBGPRINT(RT_DEBUG_TRACE, "force out a ProbeRequest ...\n");

    NState = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NState == NDIS_STATUS_SUCCESS)
    {
        MgtMacHeaderInit(pAd, &Hdr, SUBTYPE_PROBE_REQ, 0, &pAd->PortCfg.Broadcast, &pAd->PortCfg.Broadcast);
            
        // this ProbeRequest explicitly specify SSID to reduce unwanted ProbeResponse
        MakeOutgoingFrame(OutBuffer,                      &FrameLen,
                          sizeof(MACHDR),                 &Hdr,
                          1,                              &SsidIe,
                          1,                              &pAd->PortCfg.SsidLen,
                          pAd->PortCfg.SsidLen,           pAd->PortCfg.Ssid,
                          1,                              &SuppRateIe,
                          1,                              &pAd->PortCfg.SupportedRatesLen,
                          pAd->PortCfg.SupportedRatesLen, pAd->PortCfg.SupportedRates, 
                          END_OF_ARGS);
        MiniportMMRequest(pAd, OutBuffer, FrameLen);
    }
}

/* 
    ==========================================================================
    Description:
        Update PortCfg->ChannelList[] according to 1) Country Region 2) RF IC type,
        and 3) PHY-mode user selected.
        The outcome is used by driver when doing site survey.
    ==========================================================================
 */
VOID BuildChannelList(
    IN PRTMP_ADAPTER pAd)
{
    UCHAR i,  index = 0;
    memset(pAd->PortCfg.ChannelList, 0, MAX_LEN_OF_CHANNELS);

    // if not 11a-only mode, channel list starts from 2.4Ghz band
    if (pAd->PortCfg.PhyMode != PHY_11A)
{
    switch (pAd->PortCfg.CountryRegion)
    {
        case REGION_FCC:    // 1 - 11
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_FCC, sizeof(Ra24Ghz_FCC));
                index += sizeof(Ra24Ghz_FCC);
            break;
        case REGION_IC:     // 1 -11
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_IC, sizeof(Ra24Ghz_IC));
                index += sizeof(Ra24Ghz_IC);
                break;
            case REGION_ISRAEL:  // 3 - 9
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_ISRAEL, sizeof(Ra24Ghz_ISRAEL));
                index += sizeof(Ra24Ghz_ISRAEL);
            break;
        case REGION_ETSI:   // 1 - 13
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_ESTI, sizeof(Ra24Ghz_ESTI));
                index += sizeof(Ra24Ghz_ESTI);
            break;
        case REGION_SPAIN:  // 10 - 11
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_SPAIN, sizeof(Ra24Ghz_SPAIN));
                index += sizeof(Ra24Ghz_SPAIN);
            break;
        case REGION_FRANCE: // 10 -13
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_FRANCE, sizeof(Ra24Ghz_FRANCE));
                index += sizeof(Ra24Ghz_FRANCE);
            break;
        case REGION_MKK:    // 14
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_MKK, sizeof(Ra24Ghz_MKK));
                index += sizeof(Ra24Ghz_MKK);
            break;
        case REGION_MKK1:   // 1 - 14
                memcpy(&pAd->PortCfg.ChannelList[index], Ra24Ghz_MKK1, sizeof(Ra24Ghz_MKK1));
                index += sizeof(Ra24Ghz_MKK1);
            break;
            default:            // Error. should never happen
                break;
        }
    }

    if ((pAd->PortCfg.PhyMode == PHY_11A) || (pAd->PortCfg.PhyMode == PHY_11ABG_MIXED))
    {
#if 0
        switch (pAd->PortCfg.CountryRegion)
        {
            case REGION_FCC:     // UNII <36,40,44,48,52,56,60,64> <149,153,157,161>
            case REGION_IC:      // UNII <36,40,44,48,52,56,60,64> <149,153,157,161>
            case REGION_ISRAEL:  // UNII <36,40,44,48,52,56,60,64> <149,153,157,161>
                memcpy(&pAd->PortCfg.ChannelList[index], Ra5Ghz_UNII, sizeof(Ra5Ghz_UNII));
                index += sizeof(Ra5Ghz_UNII);
                break;
            case REGION_ETSI:    // HiperLAN2 <36,40,44,48,52,56,60,64> <100,104,108,112,116,120,124,128,132,136,140>
            case REGION_SPAIN:   // HiperLAN2 <36,40,44,48,52,56,60,64> <100,104,108,112,116,120,124,128,132,136,140>
            case REGION_FRANCE:  // HiperLAN2 <36,40,44,48,52,56,60,64> <100,104,108,112,116,120,124,128,132,136,140>
                memcpy(&pAd->PortCfg.ChannelList[index], Ra5Ghz_HyperLAN2, sizeof(Ra5Ghz_HyperLAN2));
                index += sizeof(Ra5Ghz_HyperLAN2);
                break;
            case REGION_MKK:    // Japan MMAC <34,38,42,46>
            case REGION_MKK1:   // Japan MMAC <34,38,42,46>
                memcpy(&pAd->PortCfg.ChannelList[index], Ra5Ghz_MMAC, sizeof(Ra5Ghz_MMAC));
                index += sizeof(Ra5Ghz_MMAC);
                break;
            default:            // Error. should never happen
            break;
        }
#else
        // 2003-10-05 john - use UNII temoparaily for all regulation domains for easy test untill
        // RF guys confirm the supported channel plans
        memcpy(&pAd->PortCfg.ChannelList[index], Ra5Ghz_UNII, sizeof(Ra5Ghz_UNII));
        index += sizeof(Ra5Ghz_UNII);
#endif
    }

    pAd->PortCfg.ChannelListNum = index;
    DBGPRINT(RT_DEBUG_TRACE,"country code=%d, RFIC=%d, PHY mode=%d, support %d channels\n", 
        pAd->PortCfg.CountryRegion, pAd->PortCfg.RfType, pAd->PortCfg.PhyMode, pAd->PortCfg.ChannelListNum);
    for (i=0;i<index;i++)
    {
        DBGPRINT(RT_DEBUG_TRACE,"channel #%d\n", pAd->PortCfg.ChannelList[i]);
    }
}

/* 
    ==========================================================================
    Description:
        This routine return the first channel number according to the country 
        code selection and RF IC selection (signal band or dual band). It is called
        whenever driver need to start a site survey of all supported channels.
    Return:
        ch - the first channel number of current country code setting
    ==========================================================================
 */
UCHAR FirstChannel(
    IN PRTMP_ADAPTER pAd)
{
    return pAd->PortCfg.ChannelList[0];
}

/* 
    ==========================================================================
    Description:
        This routine returns the next channel number. This routine is called
        during driver need to start a site survey of all supported channels.
    Return:
        next_channel - the next channel number valid in current country code setting.
    Note:
        return 0 if no more next channel
    ==========================================================================
 */
UCHAR NextChannel(
    IN PRTMP_ADAPTER pAd, 
    IN UCHAR channel)
{
    int i;
    UCHAR next_channel = 0;
            
    for (i = 0; i < (pAd->PortCfg.ChannelListNum - 1); i++)
        if (channel == pAd->PortCfg.ChannelList[i])
        {
            next_channel = pAd->PortCfg.ChannelList[i+1];
            break;
    }
    return next_channel;
}

