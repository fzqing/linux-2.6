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
 *      Module Name: connect.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 *      Ivo (rt2400)    15th Dec 04     Timing ESSID set 
 ***************************************************************************/ 

#include "rt_config.h"

UCHAR	CipherSuiteWpaNoneTkip[] = {
		0x00, 0x50, 0xf2, 0x01,	// oui
		0x01, 0x00,				// Version
		0x00, 0x50, 0xf2, 0x02,	// Multicast
		0x01, 0x00,				// Number of unicast
		0x00, 0x50, 0xf2, 0x00,	// unicast
		0x01, 0x00,				// number of authentication method
		0x00, 0x50, 0xf2, 0x00	// authentication
		};
UCHAR	CipherSuiteWpaNoneTkipLen = (sizeof(CipherSuiteWpaNoneTkip) / sizeof(UCHAR));

UCHAR	CipherSuiteWpaNoneAes[] = {
		0x00, 0x50, 0xf2, 0x01,	// oui
		0x01, 0x00,				// Version
		0x00, 0x50, 0xf2, 0x04,	// Multicast
		0x01, 0x00,				// Number of unicast
		0x00, 0x50, 0xf2, 0x00,	// unicast
		0x01, 0x00,				// number of authentication method
		0x00, 0x50, 0xf2, 0x00	// authentication
		};
UCHAR	CipherSuiteWpaNoneAesLen = (sizeof(CipherSuiteWpaNoneAes) / sizeof(UCHAR));

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID MlmeCntlInit(
    IN PRTMP_ADAPTER pAd, 
    IN STATE_MACHINE *S, 
    OUT STATE_MACHINE_FUNC Trans[]) 
{
    // Control state machine differs from other state machines, the interface 
    // follows the standard interface
    pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID MlmeCntlMachinePerformAction(
    IN PRTMP_ADAPTER pAd, 
    IN STATE_MACHINE *S, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    switch (Elem->MsgType)
    {
        case OID_802_11_SSID:
            CntlOidSsidProc(pAd, Elem);
            return;
        case OID_802_11_BSSID:
            CntlOidRTBssidProc(pAd, Elem);
            return;
        case OID_802_11_BSSID_LIST_SCAN:
            CntlOidScanProc(pAd, Elem);
            return;
    }

    switch(pAd->Mlme.CntlMachine.CurrState) 
    {
        case CNTL_IDLE:
            CntlIdleProc(pAd, Elem);
            break;
        case CNTL_WAIT_DISASSOC:
            CntlWaitDisassocProc(pAd, Elem);
            break;
        case CNTL_WAIT_JOIN:
            CntlWaitJoinProc(pAd, Elem);
            break;
            
        // CNTL_WAIT_REASSOC is the only state in CNTL machine that does
        // not triggered directly or indirectly by "RTMPSetInformation(OID_xxx)". 
        // Therefore not protected by NDIS's "only one outstanding OID request" 
        // rule. Which means NDIS may SET OID in the middle of ROAMing attempts.
        // Current approach is to block new SET request at RTMPSetInformation()
        // when CntlMachine.CurrState is not CNTL_IDLE
        case CNTL_WAIT_REASSOC:
            CntlWaitReassocProc(pAd, Elem);
            break;
            
        case CNTL_WAIT_START:
            CntlWaitStartProc(pAd, Elem);
            break;
        case CNTL_WAIT_AUTH:
            CntlWaitAuthProc(pAd, Elem);
            break;
        case CNTL_WAIT_AUTH2:
            CntlWaitAuthProc2(pAd, Elem);
            break;
        case CNTL_WAIT_ASSOC:
            CntlWaitAssocProc(pAd, Elem);
            break;

        case CNTL_WAIT_OID_LIST_SCAN:
            if(Elem->MsgType == MT2_SCAN_CONF) 
            {
                // Resume TxRing after SCANING complete. We hope the out-of-service time
                // won't be too long to let upper layer time-out the waiting frames
                RTMPResumeMsduTransmission(pAd);
                if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
                {
                }
                pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
            }
            if (pAd->MediaState == NdisMediaStateDisconnected)
                MlmeAutoReconnectLastSSID(pAd);
            break;
            
        case CNTL_WAIT_OID_DISASSOC:
            if (Elem->MsgType == MT2_DISASSOC_CONF) 
            {
                LinkDown(pAd);

                if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
                {
                }
                pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
            }
            break;

        default:
            printk(KERN_ERR DRV_NAME "CNTL - Illegal message type(=%d)", Elem->MsgType);
            break;
    }
}


/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlIdleProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MLME_DISASSOC_REQ_STRUCT   DisassocReq;
        
    if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF))
    {
        if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
        {
            pAd->Mlme.CntlAux.CurrReqIsFromNdis = FALSE;
        }
        return;
    }

    switch(Elem->MsgType) 
    {
        case OID_802_11_DISASSOCIATE:
            DisassocParmFill(pAd, &DisassocReq, &pAd->PortCfg.Bssid, REASON_DISASSOC_STA_LEAVING);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_DISASSOC_REQ, sizeof(MLME_DISASSOC_REQ_STRUCT), &DisassocReq);
            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_OID_DISASSOC;
            // Set the control aux SSID to prevent it reconnect to old SSID
            // Since calling this indicate user don't want to connect to that SSID anymore.
            pAd->Mlme.CntlAux.SsidLen = 32;
            memset(pAd->Mlme.CntlAux.Ssid, 0x00, pAd->Mlme.CntlAux.SsidLen);
            break;

        case MT2_MLME_ROAMING_REQ:
            CntlMlmeRoamingProc(pAd, Elem);
            break;
            
        default:
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - Illegal message in CntlIdleProc(MsgType=%d)\n",Elem->MsgType);
            break;
    }
}

VOID CntlOidScanProc(
    IN PRTMP_ADAPTER pAd,
    IN MLME_QUEUE_ELEM *Elem)
{
    MLME_SCAN_REQ_STRUCT       ScanReq;
    CHAR                       BroadSsid[MAX_LEN_OF_SSID];
    ULONG                      BssIdx = BSS_NOT_FOUND;
    BSS_ENTRY                  CurrBss;

    DBGPRINT(RT_DEBUG_INFO, "CNTL - SCAN starts\n");

    // temporarily recover BBP from short-distance-low-sensibility mode during SCAN
    // for best SCANNING reult;
    AsicRestoreBbpSensibility(pAd);

    // record current BSS if network is connected. 
    // 2003-2-13 do not include current IBSS if this is the only STA in this IBSS.
    if (pAd->MediaState == NdisMediaStateConnected) //  if (INFRA_ON(pAd) || ADHOC_ON(pAd))
    {
        BssIdx = BssTableSearch(&pAd->PortCfg.BssTab, &pAd->PortCfg.Bssid);
        if (BssIdx != BSS_NOT_FOUND)
        {
            memcpy(&CurrBss, &pAd->PortCfg.BssTab.BssEntry[BssIdx], sizeof(BSS_ENTRY));

            // 2003-2-20 reset this RSSI to a low value but not zero. In normal case, the coming SCAN
            //     should return a correct RSSI to overwrite this. If no BEEACON received after SCAN, 
            //     at least we still report a "greater than 0" RSSI since we claim it's CONNECTED.
            CurrBss.Rssi = 18; // about -82 dB
        }
    }
            
    // clean up previous SCAN result, add current BSS back to table if any
    BssTableInit(&pAd->PortCfg.BssTab); 
    if (BssIdx != BSS_NOT_FOUND)
    {
        // DDK Note: If the NIC is associated with a particular BSSID and SSID 
        //    that are not contained in the list of BSSIDs generated by this scan, the 
        //    BSSID description of the currently associated BSSID and SSID should be 
        //    appended to the list of BSSIDs in the NIC's database.
        // To ensure this, we append this BSS as the first entry in SCAN result
        memcpy(&pAd->PortCfg.BssTab.BssEntry[0], &CurrBss, sizeof(BSS_ENTRY));
        pAd->PortCfg.BssTab.BssNr = 1;
    }

    BroadSsid[0] = '\0';
    ScanParmFill(pAd, &ScanReq, BroadSsid, 0, BSS_ANY, SCAN_PASSIVE);
    MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_SCAN_REQ, 
        sizeof(MLME_SCAN_REQ_STRUCT), &ScanReq);
    pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_OID_LIST_SCAN;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlOidSsidProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM * Elem) 
{
    NDIS_802_11_SSID            *OidSsid = (NDIS_802_11_SSID *)Elem->Msg;
    MLME_DISASSOC_REQ_STRUCT    DisassocReq;
    ULONG                       Now;

    // Step 0. 
    //    record the desired SSID and all matching BSSes into CntlAux.SsidBssTab for 
    //    later-on iteration. Sort by RSSI order
    memcpy(pAd->Mlme.CntlAux.Ssid, OidSsid->Ssid, OidSsid->SsidLength);
    pAd->Mlme.CntlAux.SsidLen = (UCHAR)OidSsid->SsidLength;
    BssTableSsidSort(pAd, &pAd->Mlme.CntlAux.SsidBssTab, pAd->Mlme.CntlAux.Ssid, pAd->Mlme.CntlAux.SsidLen);
    pAd->Mlme.CntlAux.BssIdx = 0;
    DBGPRINT(RT_DEBUG_INFO, "CNTL - %d BSS match the desire SSID %s\n",pAd->Mlme.CntlAux.SsidBssTab.BssNr, pAd->Mlme.CntlAux.Ssid);

    Now = jiffies;

    if ((pAd->MediaState == NdisMediaStateConnected) &&
        MAC_ADDR_EQUAL(&pAd->PortCfg.Bssid, &pAd->Mlme.CntlAux.SsidBssTab.BssEntry[0].Bssid))
    {
        if (((pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPA) || (pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK)) &&
            (pAd->PortCfg.PortSecured == WPA_802_1X_PORT_NOT_SECURED))
        {
            // For WPA, WPA-PSK, if the 1x port is not secured, we have to redo 
            // connection process
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - disassociate with current AP...\n");
            DisassocParmFill(pAd, &DisassocReq, &pAd->PortCfg.Bssid, REASON_DISASSOC_STA_LEAVING);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_DISASSOC_REQ, 
                        sizeof(MLME_DISASSOC_REQ_STRUCT), &DisassocReq);
            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_DISASSOC;
        }
        else if (pAd->bConfigChanged == TRUE)
        {
            // Config has changed, we have to reconnect the same AP
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - disassociate with current AP Because config changed...\n");
            DisassocParmFill(pAd, &DisassocReq, &pAd->PortCfg.Bssid, REASON_DISASSOC_STA_LEAVING);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_DISASSOC_REQ, 
                        sizeof(MLME_DISASSOC_REQ_STRUCT), &DisassocReq);
            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_DISASSOC;
        }
        else
        {
            // We only check if same to the BSSID with highest RSSI.
            // If roaming of same SSID required, we still do the reconnection.
            // same BSSID, go back to idle state directly
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - already with this BSSID. ignore this SET_SSID request\n");
            if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
            {
            }
            pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
        } 
    } 
    else if (INFRA_ON(pAd)) 
    {
        // case 1. active association existent
        //    roaming is done within miniport driver, nothing to do with configuration
        //    utility. so upon a new SET(OID_802_11_SSID) is received, we just 
        //    disassociate with the current (or previous) associated AP, if any, 
        //    then perform a new association with this new SSID, no matter the 
        //    new/old SSID are the same or npt.
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - disassociate with current AP...\n");
        DisassocParmFill(pAd, &DisassocReq, &pAd->PortCfg.Bssid, REASON_DISASSOC_STA_LEAVING);
        MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_DISASSOC_REQ, 
                    sizeof(MLME_DISASSOC_REQ_STRUCT), &DisassocReq);
        pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_DISASSOC;
    }
    else
    {   
        if (ADHOC_ON(pAd))
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - drop current ADHOC\n");
            LinkDown(pAd);
            pAd->MediaState = NdisMediaStateDisconnected;
            DBGPRINT(RT_DEBUG_TRACE, "NDIS_STATUS_MEDIA_DISCONNECT Event C!\n");
        }

        if ((pAd->Mlme.CntlAux.SsidBssTab.BssNr==0) && (pAd->PortCfg.AutoReconnect == TRUE) && (pAd->PortCfg.BssType == BSS_INFRA))
        {
            MLME_SCAN_REQ_STRUCT       ScanReq;

                DBGPRINT(RT_DEBUG_TRACE, "CNTL - No matching BSS, start a new scan\n");
            // BroadSsid[0] = '\0';
            ScanParmFill(pAd, &ScanReq, pAd->Mlme.CntlAux.Ssid, pAd->Mlme.CntlAux.SsidLen, BSS_ANY, SCAN_ACTIVE);
                MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_SCAN_REQ, sizeof(MLME_SCAN_REQ_STRUCT), &ScanReq);
                pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_OID_LIST_SCAN;
                // Reset Missed scan number
                pAd->PortCfg.IgnoredScanNumber = 0;
                pAd->PortCfg.LastScanTime = Now;
            }
        else
        {
            IterateOnBssTab(pAd);
        }
    } 
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlOidRTBssidProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM * Elem) 
{
    ULONG       BssIdx;
    MACADDR     *pOidBssid = (MACADDR *)Elem->Msg;
    MLME_DISASSOC_REQ_STRUCT    DisassocReq;
    MLME_JOIN_REQ_STRUCT        JoinReq;
 
    COPY_MAC_ADDR(&pAd->Mlme.CntlAux.Bssid, pOidBssid);
    BssIdx = BssTableSearch(&pAd->PortCfg.BssTab, pOidBssid);
       
    if (BssIdx == BSS_NOT_FOUND) 
    {
    	DBGPRINT(RT_DEBUG_TRACE, "CNTL - BSSID not found. reply NDIS_STATUS_NOT_ACCEPTED\n");
        if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
        {

        }
        pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
        return;
    }

    // copy the matched BSS entry from PortCfg.BssTab to CntlAux.SsidBssTab
    pAd->Mlme.CntlAux.BssIdx = 0;
    pAd->Mlme.CntlAux.SsidBssTab.BssNr = 1;
    memcpy(&pAd->Mlme.CntlAux.SsidBssTab.BssEntry[0], &pAd->PortCfg.BssTab.BssEntry[BssIdx], sizeof(BSS_ENTRY));

    // Add SSID into Mlme.CntlAux for site surey joining hidden SSID
    pAd->Mlme.CntlAux.SsidLen = pAd->Mlme.CntlAux.SsidBssTab.BssEntry[0].SsidLen;
    memcpy(pAd->Mlme.CntlAux.Ssid, pAd->Mlme.CntlAux.SsidBssTab.BssEntry[0].Ssid, pAd->Mlme.CntlAux.SsidLen);	

    // 2002-11-26 skip the following checking. i.e. if user wants to re-connect to same AP
    // we just follow normal procedure. The reason of user doing this may because he/she changed
    // AP to another channel, but we still received BEACON from it thus don't claim Link Down.
    // Since user knows he's chnged AP channel, he'll re-connect again. By skipping the following
    // checking, we'll disassociate then re-do normal association with this AP at the new channel.
    // 2003-1-6 Re-enable this feature based on microsoft requirement which prefer not to re-do
    // connection when setting the same BSSID.
    if ( (pAd->MediaState == NdisMediaStateConnected) && //(INFRA_ON(pAd) || ADHOC_ON(pAd)) &&
        MAC_ADDR_EQUAL(&pAd->PortCfg.Bssid, pOidBssid))
    {
        // same BSSID, go back to idle state directly
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - already in this BSSID. ignore this SET_BSSID request\n");
        if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
        {
        }
        pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
    } 
    else 
    {
        if (INFRA_ON(pAd))
        {
            // disassoc from current AP first
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - disassociate with current AP ...\n");
            DisassocParmFill(pAd, &DisassocReq, &pAd->PortCfg.Bssid, REASON_DISASSOC_STA_LEAVING);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_DISASSOC_REQ, 
                        sizeof(MLME_DISASSOC_REQ_STRUCT), &DisassocReq);

            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_DISASSOC;
        }
        else
        {
            if (ADHOC_ON(pAd))
            {
                DBGPRINT(RT_DEBUG_TRACE, "CNTL - drop current ADHOC\n");
                LinkDown(pAd);
                pAd->MediaState = NdisMediaStateDisconnected;
                DBGPRINT(RT_DEBUG_TRACE, "NDIS_STATUS_MEDIA_DISCONNECT Event C!\n");
            }
            
            // No active association, join the BSS immediately
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - joining %02x:%02x:%02x:%02x:%02x:%02x ...\n",
                pOidBssid->Octet[0],pOidBssid->Octet[1],pOidBssid->Octet[2],
                pOidBssid->Octet[3],pOidBssid->Octet[4],pOidBssid->Octet[5]);
            JoinParmFill(pAd, &JoinReq, pAd->Mlme.CntlAux.BssIdx);
            MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_JOIN_REQ, sizeof(MLME_JOIN_REQ_STRUCT), &JoinReq);

            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_JOIN;
        }
    } 
}

// Roaming is the only external request triggering CNTL state machine
// despite of other "SET OID" operation. All "SET OID" related oerations 
// happen in sequence, because no other SET OID will be sent to this device
// until the the previous SET operation is complete (successful o failed).
// So, how do we quarantee this ROAMING request won't corrupt other "SET OID"?
// or been corrupted by other "SET OID"?
VOID CntlMlmeRoamingProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    // TODO: 
    // AP in different channel may show lower RSSI than actual value??
    // should we add a weighting factor to compensate it?
    DBGPRINT(RT_DEBUG_TRACE,"CNTL - Roaming in CntlAux.RoamTab...\n");
    BssTableSortByRssi(&pAd->Mlme.CntlAux.RoamTab);
    pAd->Mlme.CntlAux.RoamIdx=0;
    IterateOnBssTab2(pAd);
    
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitDisassocProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MLME_START_REQ_STRUCT     StartReq;
    
    if (Elem->MsgType == MT2_DISASSOC_CONF) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - Dis-associate successful\n");
        LinkDown(pAd);

        // case 1. no matching BSS, and user wants ADHOC, so we just start a new one        
        if ((pAd->Mlme.CntlAux.SsidBssTab.BssNr==0) && (pAd->PortCfg.BssType == BSS_INDEP))
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - No matching BSS, start a new ADHOC (Ssid=%s)...\n",pAd->Mlme.CntlAux.Ssid);
            StartParmFill(pAd, &StartReq, pAd->Mlme.CntlAux.Ssid, pAd->Mlme.CntlAux.SsidLen);
            MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_START_REQ, sizeof(MLME_START_REQ_STRUCT), &StartReq);
            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_START;
        }
        // case 2. try each matched BSS
        else
        {
            IterateOnBssTab(pAd);
        }
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitJoinProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT                      Reason;
    MLME_AUTH_REQ_STRUCT        AuthReq;

    if (Elem->MsgType == MT2_JOIN_CONF) 
    {
        memcpy(&Reason, Elem->Msg, sizeof(USHORT));
        if (Reason == MLME_SUCCESS) 
        {
            // 1. joined an IBSS, we are pretty much done here
            if (pAd->PortCfg.BssType == BSS_INDEP)
            {
                LinkUp(pAd, BSS_INDEP);
                if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
                {
                }
                pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
            } 
            // 2. joined a new INFRA network, start from authentication
            else 
            {
                // either Ndis802_11AuthModeShared or Ndis802_11AuthModeAutoSwitch, try shared key first
                if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeShared) ||
                    (pAd->PortCfg.AuthMode == Ndis802_11AuthModeAutoSwitch))
                {
                    AuthParmFill(pAd, &AuthReq, &pAd->PortCfg.Bssid, Ndis802_11AuthModeShared);
                }
                else
                {
                    AuthParmFill(pAd, &AuthReq, &pAd->PortCfg.Bssid, Ndis802_11AuthModeOpen);
                }
                MlmeEnqueue(&pAd->Mlme.Queue, AUTH_STATE_MACHINE, MT2_MLME_AUTH_REQ, 
                            sizeof(MLME_AUTH_REQ_STRUCT), &AuthReq);

                pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_AUTH;
            }
        }
        else
        {
            // 3. failed, try next BSS
            pAd->Mlme.CntlAux.BssIdx++;
            IterateOnBssTab(pAd);
        } 
    }
}


/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitStartProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT      Result;

    if (Elem->MsgType == MT2_START_CONF) 
    {
        memcpy(&Result, Elem->Msg, sizeof(USHORT));
        if (Result == MLME_SUCCESS) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - We have started a new ADHOC network\n");
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - BSSID %02x:%02x:%02x:%02x:%02x:%02x ...\n", 
                pAd->PortCfg.Bssid.Octet[0],
                pAd->PortCfg.Bssid.Octet[1],
                pAd->PortCfg.Bssid.Octet[2],
                pAd->PortCfg.Bssid.Octet[3],
                pAd->PortCfg.Bssid.Octet[4],
                pAd->PortCfg.Bssid.Octet[5]);
            LinkUp(pAd, BSS_INDEP);
            if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
            {
            }
            pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
        }
        else
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - Start FAIL. BUG!!!!!\n");
            if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
            {
            }
            pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
        }
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitAuthProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT                       Reason;
    MLME_ASSOC_REQ_STRUCT        AssocReq;
    MLME_AUTH_REQ_STRUCT         AuthReq;

    if (Elem->MsgType == MT2_AUTH_CONF) 
    {
        memcpy(&Reason, Elem->Msg, sizeof(USHORT));
        if (Reason == MLME_SUCCESS) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - AUTH OK\n");
            AssocParmFill(pAd, &AssocReq, &pAd->PortCfg.Bssid, pAd->PortCfg.CapabilityInfo, 
                          ASSOC_TIMEOUT, pAd->PortCfg.DefaultListenCount);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_ASSOC_REQ, 
                        sizeof(MLME_ASSOC_REQ_STRUCT), &AssocReq);

            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_ASSOC;
        } 
        else
        {
            // This fail may because of the AP already keep us in its MAC table without 
            // ageing-out. The previous authentication attempt must have let it remove us.
            // so try Authentication again may help. For D-Link DWL-900AP+ compatibility.
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - AUTH FAIL, try again...\n");
            if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeShared) ||
                (pAd->PortCfg.AuthMode == Ndis802_11AuthModeAutoSwitch))
            {
                // either Ndis802_11AuthModeShared or Ndis802_11AuthModeAutoSwitch, try shared key first
                AuthParmFill(pAd, &AuthReq, &pAd->PortCfg.Bssid, Ndis802_11AuthModeShared);
            }
            else
            {
                AuthParmFill(pAd, &AuthReq, &pAd->PortCfg.Bssid, Ndis802_11AuthModeOpen);
            }

            MlmeEnqueue(&pAd->Mlme.Queue, AUTH_STATE_MACHINE, MT2_MLME_AUTH_REQ, 
                        sizeof(MLME_AUTH_REQ_STRUCT), &AuthReq);

            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_AUTH2;
        }
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitAuthProc2(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT                       Reason;
    MLME_ASSOC_REQ_STRUCT        AssocReq;
    MLME_AUTH_REQ_STRUCT         AuthReq;

    if (Elem->MsgType == MT2_AUTH_CONF) 
    {
        memcpy(&Reason, Elem->Msg, sizeof(USHORT));
        if (Reason == MLME_SUCCESS) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - AUTH OK\n");
            AssocParmFill(pAd, &AssocReq, &pAd->PortCfg.Bssid, pAd->PortCfg.CapabilityInfo, 
                          ASSOC_TIMEOUT, pAd->PortCfg.DefaultListenCount);
            MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_ASSOC_REQ, 
                        sizeof(MLME_ASSOC_REQ_STRUCT), &AssocReq);

            pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_ASSOC;
        } 
        else
        {
            if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeAutoSwitch) &&
                 (pAd->Mlme.AuthAux.Alg == Ndis802_11AuthModeShared))
            {
                DBGPRINT(RT_DEBUG_TRACE, "CNTL - AUTH FAIL, try OPEN system...\n");
                AuthParmFill(pAd, &AuthReq, &pAd->PortCfg.Bssid, Ndis802_11AuthModeOpen);
                MlmeEnqueue(&pAd->Mlme.Queue, AUTH_STATE_MACHINE, MT2_MLME_AUTH_REQ, 
                            sizeof(MLME_AUTH_REQ_STRUCT), &AuthReq);

                pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_AUTH2;
            }
            else 
            {
                // not success, try next BSS
                DBGPRINT(RT_DEBUG_TRACE, "CNTL - AUTH FAIL, give up; try next BSS\n");
                pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE; //???????
                pAd->Mlme.CntlAux.BssIdx++;
                IterateOnBssTab(pAd);
            }
        }
    }    
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitAssocProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT      Reason;

    if (Elem->MsgType == MT2_ASSOC_CONF) 
    {
        memcpy(&Reason, Elem->Msg, sizeof(USHORT));
        if (Reason == MLME_SUCCESS) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - Association successful on BSS #%d\n",pAd->Mlme.CntlAux.BssIdx);
            LinkUp(pAd, BSS_INFRA);
            if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
            {
            }
            pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
        } 
        else 
        {
            // not success, try next BSS
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - Association fails on BSS #%d\n",pAd->Mlme.CntlAux.BssIdx);
            pAd->Mlme.CntlAux.BssIdx++;
            IterateOnBssTab(pAd);
        }
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID CntlWaitReassocProc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT      Result;

    if (Elem->MsgType == MT2_REASSOC_CONF) 
    {
        memcpy(&Result, Elem->Msg, sizeof(USHORT));
        if (Result == MLME_SUCCESS) 
        {
            BSS_ENTRY *pBss = &pAd->Mlme.CntlAux.RoamTab.BssEntry[pAd->Mlme.CntlAux.RoamIdx];

            // COPY_MAC_ADDR(&pAd->PortCfg.Bssid, &pBss->Bssid);
            // AsicSetBssid(pAd, &pAd->PortCfg.Bssid);
            
            // The following steps are supposed to be done after JOIN in normal procedure
            // But since this RE-ASSOC skips the JOIN procedure, we have to do it after
            // RE-ASSOC succeeds. If RE-ASSOC fails, then stay at original AP without any change
            pAd->PortCfg.BeaconPeriod = pBss->BeaconPeriod;
            pAd->PortCfg.Channel = pBss->Channel;
            // The security setting should always follow upper layer definition, not from frame
            //pAd->PortCfg.PrivacyInvoked = CAP_IS_PRIVACY_ON(pBss->CapabilityInfo);
            pAd->PortCfg.SupportedRatesLen = pBss->RatesLen;
            memcpy(pAd->PortCfg.SupportedRates, pBss->Rates, pBss->RatesLen);

            // Check for 802.11g information, if 802.11 b /g mixed mode.
            pAd->PortCfg.CapabilityInfo = pBss->CapabilityInfo;

            pAd->PortCfg.CfpPeriod = pBss->CfpPeriod;
            pAd->PortCfg.CfpMaxDuration = pBss->CfpMaxDuration;
            pAd->PortCfg.CfpDurRemain = pBss->CfpDurRemaining;
            pAd->PortCfg.CfpCount = pBss->CfpCount;

            // 
            // NDIS requires a new Link UP indication but no Link Down for RE-ASSOC
            //
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - Re-assocition successful on BSS #%d\n", pAd->Mlme.CntlAux.RoamIdx);
            LinkUp(pAd, BSS_INFRA);
            pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;            
        } 
        else 
        {
            // reassoc failed, try to pick next BSS in the BSS Table
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - Re-assocition fails on BSS #%d\n", pAd->Mlme.CntlAux.RoamIdx);
            pAd->Mlme.CntlAux.RoamIdx++;
            IterateOnBssTab2(pAd);
        }
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID LinkUp(
    IN PRTMP_ADAPTER pAd,
    IN UCHAR BssType) 
{
    ULONG       Now;

    DBGPRINT(RT_DEBUG_TRACE, "CNTL - !!! LINK UP !!!\n");
    MlmeUpdateTxRates(pAd, TRUE);
    memcpy(&pAd->Mlme.PrevWlanCounters, &pAd->WlanCounters, sizeof(COUNTER_802_11));
    memset(&pAd->DrsCounters, 0, sizeof(COUNTER_DRS));

    Now = jiffies;
    pAd->PortCfg.LastBeaconRxTime = Now;   // last RX timestamp

    if ((pAd->PortCfg.WindowsTxPreamble != Rt802_11PreambleLong) &&
        CAP_IS_SHORT_PREAMBLE_ON(pAd->PortCfg.CapabilityInfo))
    {
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - !!! Set to short preamble!!!\n");
        MlmeSetTxPreamble(pAd, Rt802_11PreambleShort);
    }
    
    pAd->PortCfg.BssType = BssType;
    if (BssType == BSS_INDEP)
    {
        pAd->PortCfg.Mibss = TRUE;
        pAd->PortCfg.Massoc = FALSE;
        MakeIbssBeacon(pAd);
        AsicEnableIbssSync(pAd);

#ifdef  SINGLE_ADHOC_LINKUP
        // Although this did not follow microsoft's recommendation.
        //Change based on customer's request
        pAd->MediaState = NdisMediaStateConnected;
#endif

        // Clear Key information when driver change to WPA-None mode
        // which did not have any key set
#if 0
        if (pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
        {
            INT i;

            for (i = 0; i < PAIRWISE_KEY_NO; i++)
            {
                pAd->PortCfg.PairwiseKey[i].KeyLen = 0;
            }

            for (i = 0; i < GROUP_KEY_NO; i++)
            {
                pAd->PortCfg.GroupKey[i].KeyLen = 0;
            }
        }
#endif
    }
    else // BSS_INFRA
    {
        pAd->PortCfg.Massoc = TRUE;
        pAd->PortCfg.Mibss = FALSE;

        // NOTE:
        // the decision of using "short slot time" or not may change dynamically due to
        // new STA association to the AP. so we have to decide that upon parsing BEACON, not here

        // NOTE:
        // the decision to use "RTC/CTS" or "CTS-to-self" protection or not may change dynamically
        // due to new STA association to the AP. so we have to decide that upon parsing BEACON, not here
        
        ComposePsPoll(pAd);
        ComposeNullFrame(pAd);
        AsicEnableBssSync(pAd);
    
        // only INFRASTRUCTURE mode need to indicate connectivity immediately; ADHOC mode
        // should wait until at least 2 active nodes in this BSSID.
        pAd->MediaState = NdisMediaStateConnected;
    }

    DBGPRINT(RT_DEBUG_TRACE, "NDIS_STATUS_MEDIA_CONNECT Event B!\n");

    if (pAd->PortCfg.LedMode != LED_MODE_SINGLE)
    {
        ASIC_LED_ACT_ON(pAd);
    }

    AsicSetSlotTime(pAd, FALSE);
    pAd->Mlme.PeriodicRound = 0;
    // Reset config flag
    pAd->bConfigChanged = FALSE;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID LinkDown(
    IN PRTMP_ADAPTER pAd) 
{
    DBGPRINT(RT_DEBUG_TRACE, "CNTL - !!! LINK DOWN !!!\n");

    if (ADHOC_ON(pAd))      // Adhoc mode link down
    {
        pAd->PortCfg.Mibss = FALSE;

#ifdef  SINGLE_ADHOC_LINKUP
        pAd->MediaState = NdisMediaStateDisconnected;
        // clean up previous SCAN result, add current BSS back to table if any
        BssTableDeleteEntry(&pAd->PortCfg.BssTab, &(pAd->PortCfg.Bssid));
#else
        if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF))
        {
            pAd->MediaState = NdisMediaStateDisconnected;
            // clean up previous SCAN result, add current BSS back to table if any
            BssTableDeleteEntry(&pAd->PortCfg.BssTab, &(pAd->PortCfg.Bssid));
        }
#endif
    }
    else// Infra structure mode
    {
        pAd->PortCfg.Massoc = FALSE;
        pAd->MediaState = NdisMediaStateDisconnected;
        DBGPRINT(RT_DEBUG_TRACE, "NDIS_STATUS_MEDIA_DISCONNECT Event A!\n");
        BssTableDeleteEntry(&pAd->PortCfg.BssTab, &(pAd->PortCfg.Bssid));

        // restore back to - 
        //      1. long slot (20 us) or short slot (9 us) time
        //      2. turn on/off RTS/CTS and/or CTS-to-self protection
        //      3. short preamble
        if (pAd->PortCfg.BGProtectionInUsed == TRUE)
        {
            pAd->PortCfg.BGProtectionInUsed = FALSE;
            DBGPRINT(RT_DEBUG_TRACE, "Link down - turn off B/G protection\n");
        }

    }

    AsicSetSlotTime(pAd, FALSE);
    AsicRestoreBbpSensibility(pAd);

    if (pAd->PortCfg.WindowsTxPreamble == Rt802_11PreambleShort)
        MlmeSetTxPreamble(pAd, Rt802_11PreambleShort);
    else
        MlmeSetTxPreamble(pAd, Rt802_11PreambleLong);

    if ((pAd->PortCfg.LedMode != LED_MODE_SINGLE) && (pAd->PortCfg.LedMode != LED_MODE_ASUS))
    {
        ASIC_LED_ACT_OFF(pAd);
    }
    else if ((pAd->PortCfg.LedMode == LED_MODE_ASUS) && (pAd->PortCfg.bRadio == TRUE))
    {
        RTMP_IO_WRITE32(pAd, LEDCSR, 0x0002461E);
    }
    AsicDisableSync(pAd);
    pAd->Mlme.PeriodicRound = 0;

    // Remove PortCfg Information after link down
        memset(&(pAd->PortCfg.Bssid), 0, ETH_ALEN);
        //memset(pAd->PortCfg.Ssid, 0, MAX_LEN_OF_SSID);
        //pAd->PortCfg.SsidLen = 0;

    // Reset WPA-PSK state. Only reset when supplicant enabled
    if (pAd->PortCfg.WpaState != SS_NOTUSE)
    {
        pAd->PortCfg.WpaState = SS_START;
        // Clear Replay counter
        memset(pAd->PortCfg.ReplayCounter, 0, 8);
    }

    // Remove all WPA keys after link down
    //RTMPWPARemoveAllKeys(pAd);
    // 802.1x port control
    pAd->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;
    pAd->PortCfg.MicErrCnt = 0;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID MlmeCntlConfirm(
    IN PRTMP_ADAPTER pAd, 
    IN ULONG MsgType, 
    IN USHORT Msg) 
{
    MlmeEnqueue(&pAd->Mlme.Queue, MLME_CNTL_STATE_MACHINE, MsgType, sizeof(USHORT), &Msg);
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID IterateOnBssTab(
    IN PRTMP_ADAPTER pAd) 
{
    MLME_START_REQ_STRUCT   StartReq;
    MLME_JOIN_REQ_STRUCT    JoinReq;
    ULONG                   BssIdx;

    BssIdx = pAd->Mlme.CntlAux.BssIdx;
    if (BssIdx < pAd->Mlme.CntlAux.SsidBssTab.BssNr) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - Trying BSSID %02x:%02x:%02x:%02x:%02x:%02x ...\n", 
            pAd->Mlme.CntlAux.SsidBssTab.BssEntry[BssIdx].Bssid.Octet[0],
            pAd->Mlme.CntlAux.SsidBssTab.BssEntry[BssIdx].Bssid.Octet[1],
            pAd->Mlme.CntlAux.SsidBssTab.BssEntry[BssIdx].Bssid.Octet[2],
            pAd->Mlme.CntlAux.SsidBssTab.BssEntry[BssIdx].Bssid.Octet[3],
            pAd->Mlme.CntlAux.SsidBssTab.BssEntry[BssIdx].Bssid.Octet[4],
            pAd->Mlme.CntlAux.SsidBssTab.BssEntry[BssIdx].Bssid.Octet[5]);
        JoinParmFill(pAd, &JoinReq, BssIdx);
        MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_JOIN_REQ, sizeof(MLME_JOIN_REQ_STRUCT),
                    &JoinReq);
        pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_JOIN;
    }
    else if (pAd->PortCfg.BssType == BSS_INDEP)
    {
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - All BSS fail; start a new ADHOC (Ssid=%s)...\n",pAd->Mlme.CntlAux.Ssid);
        StartParmFill(pAd, &StartReq, pAd->Mlme.CntlAux.Ssid, (UCHAR)pAd->Mlme.CntlAux.SsidLen);
        MlmeEnqueue(&pAd->Mlme.Queue, SYNC_STATE_MACHINE, MT2_MLME_START_REQ, sizeof(MLME_START_REQ_STRUCT), &StartReq);
        pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_START;
    }
    else // no more BSS
    {
        if (pAd->Mlme.CntlAux.CurrReqIsFromNdis)
        {
            DBGPRINT(RT_DEBUG_TRACE, "CNTL - All BSS fail; reply NDIS_STATUS_NOT_ACCEPTED\n");
        }
        pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
    } 
}

// for re-association only
VOID IterateOnBssTab2(
    IN PRTMP_ADAPTER pAd) 
{
    MLME_REASSOC_REQ_STRUCT ReassocReq;
    ULONG                   BssIdx;
    BSS_ENTRY               *pBss;

    BssIdx = pAd->Mlme.CntlAux.RoamIdx;
    pBss = &pAd->Mlme.CntlAux.RoamTab.BssEntry[BssIdx];

    if (BssIdx < pAd->Mlme.CntlAux.RoamTab.BssNr)
    {
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - try BSS #%d %02x:%02x:%02x:%02x:%02x:%02x ...\n", 
            BssIdx, pBss->Bssid.Octet[0],pBss->Bssid.Octet[1],pBss->Bssid.Octet[2],
            pBss->Bssid.Octet[3],pBss->Bssid.Octet[4],pBss->Bssid.Octet[5]);

        AsicSwitchChannel(pAd, pBss->Channel);
		AsicLockChannel(pAd, pBss->Channel);
        
        // reassociate message has the same structure as associate message
        AssocParmFill(pAd, &ReassocReq, &pBss->Bssid, pBss->CapabilityInfo, 
                      ASSOC_TIMEOUT, pAd->PortCfg.DefaultListenCount);
        MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_MLME_REASSOC_REQ, 
                    sizeof(MLME_REASSOC_REQ_STRUCT), &ReassocReq);
        
        pAd->Mlme.CntlMachine.CurrState = CNTL_WAIT_REASSOC;
    }
    else // no more BSS
    {
        DBGPRINT(RT_DEBUG_TRACE, "CNTL - All roaming failed, stay with original AP\n");
        AsicSwitchChannel(pAd, pAd->PortCfg.Channel);
        AsicLockChannel(pAd, pAd->PortCfg.Channel);
        pAd->Mlme.CntlMachine.CurrState = CNTL_IDLE;
    } 
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID JoinParmFill(
    IN PRTMP_ADAPTER pAd, 
    IN OUT MLME_JOIN_REQ_STRUCT *JoinReq, 
    IN ULONG BssIdx) 
{
    JoinReq->BssIdx = BssIdx;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID AssocParmFill(
    IN PRTMP_ADAPTER pAd, 
    IN OUT MLME_ASSOC_REQ_STRUCT *AssocReq, 
    IN MACADDR                   *Addr, 
    IN USHORT                     CapabilityInfo, 
    IN ULONG                      Timeout, 
    IN USHORT                     ListenIntv) 
{
    COPY_MAC_ADDR(&AssocReq->Addr, Addr);
    // Add mask to support 802.11b mode only
    AssocReq->CapabilityInfo = CapabilityInfo & 0xfff3; // not cf-pollable, not cf-poll-request
    AssocReq->Timeout = Timeout;
    AssocReq->ListenIntv = ListenIntv;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID ScanParmFill(
    IN PRTMP_ADAPTER pAd, 
    IN OUT MLME_SCAN_REQ_STRUCT *ScanReq, 
    IN CHAR Ssid[], 
    IN UCHAR SsidLen, 
    IN UCHAR BssType, 
    IN UCHAR ScanType) 
{
    ScanReq->SsidLen = SsidLen;
    memcpy(ScanReq->Ssid, Ssid, SsidLen);
    ScanReq->BssType = BssType;
    ScanReq->ScanType = ScanType;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID DisassocParmFill(
    IN PRTMP_ADAPTER pAd, 
    IN OUT MLME_DISASSOC_REQ_STRUCT *DisassocReq, 
    IN MACADDR *Addr, 
    IN USHORT Reason) 
{
    COPY_MAC_ADDR(&DisassocReq->Addr, Addr);
    DisassocReq->Reason = Reason;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID StartParmFill(
    IN PRTMP_ADAPTER pAd, 
    IN OUT MLME_START_REQ_STRUCT *StartReq, 
    IN CHAR Ssid[], 
    IN UCHAR SsidLen) 
{
    memcpy(StartReq->Ssid, Ssid, SsidLen); 
    StartReq->SsidLen = SsidLen;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID AuthParmFill(
    IN PRTMP_ADAPTER pAd, 
    IN OUT MLME_AUTH_REQ_STRUCT *AuthReq, 
    IN MACADDR *Addr, 
    IN USHORT Alg) 
{
    COPY_MAC_ADDR(&AuthReq->Addr, Addr);
    AuthReq->Alg = Alg;
    AuthReq->Timeout = AUTH_TIMEOUT;
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID ComposePsPoll(
    IN PRTMP_ADAPTER pAd)
{
    memset(&pAd->Mlme.PsFr, 0, sizeof(PSPOLL_FRAME));
    pAd->Mlme.PsFr.Type = BTYPE_CNTL;
    pAd->Mlme.PsFr.SubType = SUBTYPE_PS_POLL;
    pAd->Mlme.PsFr.Aid = pAd->PortCfg.Aid | 0xC000;
    COPY_MAC_ADDR(&(pAd->Mlme.PsFr.Bssid), &pAd->PortCfg.Bssid);
    COPY_MAC_ADDR(&(pAd->Mlme.PsFr.Ta), &(pAd->CurrentAddress));
}

VOID ComposeNullFrame(
    IN PRTMP_ADAPTER pAd)
{
    MgtMacHeaderInit(pAd, &pAd->Mlme.NullFr, SUBTYPE_NULL_FUNC, 1, &pAd->PortCfg.Bssid, &pAd->PortCfg.Bssid);
    pAd->Mlme.NullFr.Duration = 0;
    pAd->Mlme.NullFr.Type = BTYPE_DATA;
}

/*
    ==========================================================================
    Description:
        Pre-build a BEACON frame in the shared memory
    ==========================================================================
*/
ULONG MakeIbssBeacon(
    IN PRTMP_ADAPTER pAd) 
{
    UCHAR           SsidIe = IE_SSID, DsIe = IE_DS_PARM, IbssIe = IE_IBSS_PARM, SuppIe = IE_SUPP_RATES, 
                    DsLen = 1, IbssLen = 2;
    UCHAR           ExtRateIe = IE_EXT_SUPP_RATES, ExtRatesLen;
    UCHAR         ErpIe[3] = {IE_ERP, 1, 0x04};
    MACHDR          BcnHdr;
    USHORT          CapabilityInfo;
    LARGE_INTEGER   FakeTimestamp;
    ULONG           FrameLen;
    PTXD_STRUC      pTxD = (PTXD_STRUC)pAd->BeaconRing.va_addr;
    CHAR            *pBeaconFrame = (CHAR *)pAd->BeaconRing.va_data_addr;
    UCHAR           SupportedRatesLen;
    UCHAR           SupportedRates[MAX_LEN_OF_SUPPORTED_RATES];
    BOOLEAN       Privacy;

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

    pAd->PortCfg.AtimWin = 0;  // ??????

    // compose IBSS beacon frame
    MgtMacHeaderInit(pAd, &BcnHdr, SUBTYPE_BEACON, 0, &pAd->PortCfg.Broadcast, &pAd->PortCfg.Bssid);
    Privacy = (pAd->PortCfg.WepStatus == Ndis802_11Encryption1Enabled) || 
              (pAd->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) || 
              (pAd->PortCfg.WepStatus == Ndis802_11Encryption3Enabled);
    CapabilityInfo = CAP_GENERATE(0, 1, 0, 0, Privacy, (pAd->PortCfg.WindowsTxPreamble == Rt802_11PreambleShort));
    if (SupportedRatesLen <= 8)
    {
        MakeOutgoingFrame(pBeaconFrame,                    &FrameLen,
                          MAC_HDR_LEN,                     &BcnHdr, 
                          TIMESTAMP_LEN,                   &FakeTimestamp,
                          2,                               &pAd->PortCfg.BeaconPeriod,
                          2,                               &CapabilityInfo,
                          1,                               &SsidIe, 
                          1,                               &pAd->PortCfg.SsidLen, 
                          pAd->PortCfg.SsidLen,            pAd->PortCfg.Ssid,
                          1,                               &SuppIe, 
                          1,                               &SupportedRatesLen,
                          SupportedRatesLen,               SupportedRates, 
                          1,                               &DsIe, 
                          1,                               &DsLen, 
                          1,                               &pAd->PortCfg.Channel,
                          1,                               &IbssIe, 
                          1,                               &IbssLen, 
                          2,                               &pAd->PortCfg.AtimWin,
                          END_OF_ARGS);
    }
    else
    {
        ExtRatesLen = SupportedRatesLen - 8;
        SupportedRatesLen = 8;
        MakeOutgoingFrame(pBeaconFrame,                    &FrameLen,
                      MAC_HDR_LEN,                     &BcnHdr, 
                      TIMESTAMP_LEN,                   &FakeTimestamp,
                      2,                               &pAd->PortCfg.BeaconPeriod,
                      2,                               &CapabilityInfo,
                      1,                               &SsidIe, 
                      1,                               &pAd->PortCfg.SsidLen, 
                      pAd->PortCfg.SsidLen,             pAd->PortCfg.Ssid,
                      1,                               &SuppIe, 
                      1,                               &SupportedRatesLen,
                      SupportedRatesLen,                SupportedRates, 
                      1,                               &DsIe, 
                      1,                               &DsLen, 
                      1,                               &pAd->PortCfg.Channel,
                      1,                               &IbssIe, 
                      1,                               &IbssLen, 
                      2,                               &pAd->PortCfg.AtimWin,
                      3,                               ErpIe,
                      1,                               &ExtRateIe,
                      1,                               &ExtRatesLen,
                      ExtRatesLen,                     &SupportedRates[SupportedRatesLen],
                      END_OF_ARGS);
    }
	// If adhoc secruity is set for WPA-None, append the cipher suite IE
	if (pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
	{
		ULONG	tmp;
		UCHAR	WpaIe = IE_WPA;
		
		if (pAd->PortCfg.WepStatus == Ndis802_11Encryption2Enabled)		// Tkip
		{
        MakeOutgoingFrame(pBeaconFrame + FrameLen,                    &tmp,
				1,						  &WpaIe,
				1,						  &CipherSuiteWpaNoneTkipLen,
				CipherSuiteWpaNoneTkipLen,	  &CipherSuiteWpaNoneTkip[0],
				END_OF_ARGS);
		FrameLen += tmp;
	}
		else if (pAd->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)	// Aes
		{
        	MakeOutgoingFrame(pBeaconFrame + FrameLen,                    &tmp,
				1,						  &WpaIe,
				1,						  &CipherSuiteWpaNoneAesLen,
				CipherSuiteWpaNoneAesLen,	  &CipherSuiteWpaNoneAes[0],
				END_OF_ARGS);
			FrameLen += tmp;
		}
	}
#ifdef BIG_ENDIAN
    RTMPFrameEndianChange(pAd, pBeaconFrame, DIR_WRITE, FALSE);
#endif

    RTMPWriteTxDescriptor(pTxD, FALSE, CIPHER_NONE, FALSE, FALSE, TRUE, SHORT_RETRY, IFS_NEW_BACKOFF, 
                          pAd->PortCfg.MlmeRate, 4, FrameLen, pAd->PortCfg.TxPreambleInUsed, 0);

    DBGPRINT(RT_DEBUG_TRACE, "MakeIbssBeacon (len=%d)\n", FrameLen);
    return FrameLen;
}


