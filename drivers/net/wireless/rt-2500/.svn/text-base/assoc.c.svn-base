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
 *      Module Name: assoc.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 * 		MarkW			5th  Jun 05		Fix no-SSID broadcasting assoc.
 ***************************************************************************/ 

#include "rt_config.h"

UCHAR	CipherSuiteWpaTkip[] = {
		0x00, 0x50, 0xf2, 0x01,	// oui
		0x01, 0x00,				// Version
		0x00, 0x50, 0xf2, 0x02,	// Multicast
		0x01, 0x00,				// Number of unicast
		0x00, 0x50, 0xf2, 0x02,	// unicast
		0x01, 0x00,				// number of authentication method
		0x00, 0x50, 0xf2, 0x01	// authentication
		};
UCHAR	CipherSuiteWpaTkipLen = (sizeof(CipherSuiteWpaTkip) / sizeof(UCHAR));

UCHAR	CipherSuiteWpaAes[] = {
		0x00, 0x50, 0xf2, 0x01,	// oui
		0x01, 0x00,				// Version
		0x00, 0x50, 0xf2, 0x04,	// Multicast
		0x01, 0x00,				// Number of unicast
		0x00, 0x50, 0xf2, 0x04,	// unicast
		0x01, 0x00,				// number of authentication method
		0x00, 0x50, 0xf2, 0x01	// authentication
		};
UCHAR	CipherSuiteWpaAesLen = (sizeof(CipherSuiteWpaAes) / sizeof(UCHAR));

UCHAR	CipherSuiteWpaPskTkip[] = {
		0x00, 0x50, 0xf2, 0x01,	// oui
		0x01, 0x00,				// Version
		0x00, 0x50, 0xf2, 0x02,	// Multicast
		0x01, 0x00,				// Number of unicast
		0x00, 0x50, 0xf2, 0x02,	// unicast
		0x01, 0x00,				// number of authentication method
		0x00, 0x50, 0xf2, 0x02	// authentication
		};
UCHAR	CipherSuiteWpaPskTkipLen = (sizeof(CipherSuiteWpaPskTkip) / sizeof(UCHAR));

UCHAR	CipherSuiteWpaPskAes[] = {
		0x00, 0x50, 0xf2, 0x01,	// oui
		0x01, 0x00,				// Version
		0x00, 0x50, 0xf2, 0x04,	// Multicast
		0x01, 0x00,				// Number of unicast
		0x00, 0x50, 0xf2, 0x04,	// unicast
		0x01, 0x00,				// number of authentication method
		0x00, 0x50, 0xf2, 0x02	// authentication
		};
UCHAR	CipherSuiteWpaPskAesLen = (sizeof(CipherSuiteWpaPskAes) / sizeof(UCHAR));

/*  
    ==========================================================================
    Description: 
        association state machine init, including state transition and timer init
    Parameters: 
        S - pointer to the association state machine
    Note:
        The state machine looks like the following 
        
                               ASSOC_IDLE               ASSOC_WAIT_RSP             REASSOC_WAIT_RSP             DISASSOC_WAIT_RSP
    MT2_MLME_ASSOC_REQ       mlme_assoc_req_action    invalid_state_when_assoc   invalid_state_when_assoc       invalid_state_when_assoc
    MT2_MLME_REASSOC_REQ     mlme_reassoc_req_action  invalid_state_when_reassoc invalid_state_when_reassoc     invalid_state_when_reassoc
    MT2_MLME_DISASSOC_REQ    mlme_disassoc_req_action mlme_disassoc_req_action   mlme_disassoc_req_action       mlme_disassoc_req_action
    MT2_PEER_DISASSOC_REQ    peer_disassoc_action     peer_disassoc_action       peer_disassoc_action           peer_disassoc_action
    MT2_PEER_ASSOC_REQ       drop                     drop                       drop                           drop
    MT2_PEER_ASSOC_RSP       drop                     peer_assoc_rsp_action      drop                           drop
    MT2_PEER_REASSOC_REQ     drop                     drop                       drop                           drop
    MT2_PEER_REASSOC_RSP     drop                     drop                       peer_reassoc_rsp_action        drop
    MT2_CLS3ERR              cls3err_action           cls3err_action             cls3err_action                 cls3err_action
    MT2_ASSOC_TIMEOUT        timer_nop                assoc_timeout_action       timer_nop                      timer_nop
    MT2_REASSOC_TIMEOUT      timer_nop                timer_nop                  reassoc_timeout_action         timer_nop
    MT2_DISASSOC_TIMEOUT     timer_nop                timer_nop                  timer_nop                      disassoc_timeout_action
    ==========================================================================
 */
VOID AssocStateMachineInit(
    IN	PRTMP_ADAPTER	pAd, 
    IN  STATE_MACHINE *S, 
    OUT STATE_MACHINE_FUNC Trans[]) 
{
    StateMachineInit(S, (STATE_MACHINE_FUNC*)Trans, MAX_ASSOC_STATE, MAX_ASSOC_MSG, (STATE_MACHINE_FUNC)Drop, ASSOC_IDLE, ASSOC_MACHINE_BASE);

    // first column
    StateMachineSetAction(S, ASSOC_IDLE, MT2_MLME_ASSOC_REQ, (STATE_MACHINE_FUNC)MlmeAssocReqAction);
    StateMachineSetAction(S, ASSOC_IDLE, MT2_MLME_REASSOC_REQ, (STATE_MACHINE_FUNC)MlmeReassocReqAction);
    StateMachineSetAction(S, ASSOC_IDLE, MT2_MLME_DISASSOC_REQ, (STATE_MACHINE_FUNC)MlmeDisassocReqAction);
    StateMachineSetAction(S, ASSOC_IDLE, MT2_PEER_DISASSOC_REQ, (STATE_MACHINE_FUNC)PeerDisassocAction);
//  StateMachineSetAction(S, ASSOC_IDLE, MT2_CLS3ERR, (STATE_MACHINE_FUNC)Cls3errAction);
    
    // second column
    StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_MLME_ASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenAssoc);
    StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_MLME_REASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenReassoc);
    StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_MLME_DISASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenDisassociate);
    StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_PEER_DISASSOC_REQ, (STATE_MACHINE_FUNC)PeerDisassocAction);
    StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_PEER_ASSOC_RSP, (STATE_MACHINE_FUNC)PeerAssocRspAction);
//  StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_CLS3ERR, (STATE_MACHINE_FUNC)Cls3errAction);
    StateMachineSetAction(S, ASSOC_WAIT_RSP, MT2_ASSOC_TIMEOUT, (STATE_MACHINE_FUNC)AssocTimeoutAction);

    // third column
    StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_MLME_ASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenAssoc);
    StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_MLME_REASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenReassoc);
    StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_MLME_DISASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenDisassociate);
    StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_PEER_DISASSOC_REQ, (STATE_MACHINE_FUNC)PeerDisassocAction);
    StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_PEER_REASSOC_RSP, (STATE_MACHINE_FUNC)PeerReassocRspAction);
//  StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_CLS3ERR, (STATE_MACHINE_FUNC)Cls3errAction);
    StateMachineSetAction(S, REASSOC_WAIT_RSP, MT2_REASSOC_TIMEOUT, (STATE_MACHINE_FUNC)ReassocTimeoutAction);

    // fourth column
    StateMachineSetAction(S, DISASSOC_WAIT_RSP, MT2_MLME_ASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenAssoc);
    StateMachineSetAction(S, DISASSOC_WAIT_RSP, MT2_MLME_REASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenReassoc);
    StateMachineSetAction(S, DISASSOC_WAIT_RSP, MT2_MLME_DISASSOC_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenDisassociate);
    StateMachineSetAction(S, DISASSOC_WAIT_RSP, MT2_PEER_DISASSOC_REQ, (STATE_MACHINE_FUNC)PeerDisassocAction);
//  StateMachineSetAction(S, DISASSOC_WAIT_RSP, MT2_CLS3ERR, (STATE_MACHINE_FUNC)Cls3errAction);
    StateMachineSetAction(S, DISASSOC_WAIT_RSP, MT2_DISASSOC_TIMEOUT, (STATE_MACHINE_FUNC)DisassocTimeoutAction);

    // initialize the timer
    RTMPInitTimer(pAd, &pAd->Mlme.AssocAux.AssocTimer, AssocTimeout);
    RTMPInitTimer(pAd, &pAd->Mlme.AssocAux.ReassocTimer, ReassocTimeout);
    RTMPInitTimer(pAd, &pAd->Mlme.AssocAux.DisassocTimer, DisassocTimeout);
}

/*
    ==========================================================================
    Description:
        Association timeout procedure. After association timeout, this function 
        will be called and it will put a message into the MLME queue
    Parameters:
        Standard timer parameters
    ==========================================================================
 */
VOID AssocTimeout(
    IN	unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    DBGPRINT(RT_DEBUG_TRACE,"ASSOC - enqueue MT2_ASSOC_TIMEOUT \n");
    MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_ASSOC_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/*
    ==========================================================================
    Description:
        Reassociation timeout procedure. After reassociation timeout, this 
        function will be called and put a message into the MLME queue
    Parameters:
        Standard timer parameters
    ==========================================================================
 */
VOID ReassocTimeout(
    IN	unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    DBGPRINT(RT_DEBUG_TRACE,"ASSOC - enqueue MT2_REASSOC_TIMEOUT \n");
    MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_REASSOC_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/*
    ==========================================================================
    Description:
        Disassociation timeout procedure. After disassociation timeout, this 
        function will be called and put a message into the MLME queue
    Parameters:
        Standard timer parameters
    ==========================================================================
 */
VOID DisassocTimeout(
    IN	unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    DBGPRINT(RT_DEBUG_TRACE,"ASSOC - enqueue MT2_DISASSOC_TIMEOUT \n");
    MlmeEnqueue(&pAd->Mlme.Queue, ASSOC_STATE_MACHINE, MT2_DISASSOC_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/*
    ==========================================================================
    Description:
        mlme assoc req handling procedure
    Parameters:
        Adapter - Adapter pointer
        Elem - MLME Queue Element
    Pre:
        the station has been authenticated and the following information is stored in the config
            -# SSID
            -# supported rates and their length
            -# listen interval (Adapter->PortCfg.default_listen_count)
            -# Transmit power  (Adapter->PortCfg.tx_power)
    Post  :
        -# An association request frame is generated and sent to the air
        -# Association timer starts
        -# Association state -> ASSOC_WAIT_RSP
    ==========================================================================
 */
VOID MlmeAssocReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR          ApAddr;
    MACHDR           AssocHdr;
    UCHAR            SsidIe = IE_SSID, RateIe = IE_SUPP_RATES,WpaIe = IE_WPA, ExtRateIe = IE_EXT_SUPP_RATES;
    USHORT           ListenIntv;
    ULONG            Timeout;
    USHORT           CapabilityInfo;
    UCHAR           *OutBuffer = NULL;
    NDIS_STATUS      NStatus;
    ULONG            FrameLen = 0;
	ULONG			 tmp;
	UCHAR			 VarIesOffset;

	// Block all authentication request durning WPA block period
	if (pAd->PortCfg.bBlockAssoc == TRUE)
	{
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Block Assoc request durning WPA block period!\n");
        pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
        MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, MLME_STATE_MACHINE_REJECT);
	}	
    // check sanity first
    else if (MlmeAssocReqSanity(pAd, Elem->Msg, Elem->MsgLen, &ApAddr, &CapabilityInfo, &Timeout, &ListenIntv)) 
    {
        RTMPCancelTimer(&pAd->Mlme.AssocAux.AssocTimer);
        COPY_MAC_ADDR(&pAd->Mlme.AssocAux.Addr, &ApAddr);
		// Mask out unnecessary capability information
		CapabilityInfo &= SUPPORTED_CAPABILITY_INFO; // pAd->PortCfg.SupportedCapabilityInfo;
        pAd->Mlme.AssocAux.CapabilityInfo = CapabilityInfo;
        pAd->Mlme.AssocAux.ListenIntv = ListenIntv;

        NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
        if (NStatus != NDIS_STATUS_SUCCESS) 
        {
            DBGPRINT(RT_DEBUG_TRACE,"ASSOC - MlmeAssocReqAction() allocate memory failed \n");
            pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
            MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, MLME_FAIL_NO_RESOURCE);
            return;
        }
        
		// Add by James 03/06/27
		pAd->PortCfg.AssocInfo.Length = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION); //+ sizeof(NDIS_802_11_FIXED_IEs); 	// Filled in assoc request
		pAd->PortCfg.AssocInfo.AvailableRequestFixedIEs =
			NDIS_802_11_AI_REQFI_CAPABILITIES | NDIS_802_11_AI_REQFI_LISTENINTERVAL | NDIS_802_11_AI_REQFI_CURRENTAPADDRESS;
		pAd->PortCfg.AssocInfo.RequestFixedIEs.Capabilities = CapabilityInfo;
		pAd->PortCfg.AssocInfo.RequestFixedIEs.ListenInterval = ListenIntv;		
		memcpy(pAd->PortCfg.AssocInfo.RequestFixedIEs.CurrentAPAddress, &AssocHdr, sizeof(NDIS_802_11_MAC_ADDRESS));
		pAd->PortCfg.AssocInfo.OffsetRequestIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION);		// No request Variables IEs
		
		// First add SSID
		VarIesOffset = 0;
		memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &SsidIe, 1);
		VarIesOffset += 1;
		memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &pAd->PortCfg.SsidLen, 1);
		VarIesOffset += 1;
		memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, pAd->PortCfg.Ssid, pAd->PortCfg.SsidLen);
		VarIesOffset += pAd->PortCfg.SsidLen;

		// Second add Supported rates
		memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &RateIe, 1);
		VarIesOffset += 1;
		memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &pAd->PortCfg.SupportedRatesLen, 1);
		VarIesOffset += 1;
		memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, pAd->PortCfg.SupportedRates, pAd->PortCfg.SupportedRatesLen);
		VarIesOffset += pAd->PortCfg.SupportedRatesLen;
		// End Add by James

        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Send ASSOC request...\n");
        MgtMacHeaderInit(pAd, &AssocHdr, SUBTYPE_ASSOC_REQ, 0, &ApAddr, &ApAddr);

		// Build basic frame first
		MakeOutgoingFrame(OutBuffer,				&FrameLen,
						  sizeof(MACHDR),			&AssocHdr,
						  2,						&CapabilityInfo,
						  2,						&ListenIntv,
						  1,						&SsidIe,
						  1,						&pAd->Mlme.SyncAux.SsidLen, 
						  pAd->PortCfg.SsidLen, 	pAd->Mlme.SyncAux.Ssid,
						  1,						&RateIe,
						  1,						&pAd->PortCfg.SupRateLen,
						  pAd->PortCfg.SupRateLen,  pAd->PortCfg.SupRate,
						  END_OF_ARGS);
		if (pAd->PortCfg.ExtRateLen != 0)
		{
			MakeOutgoingFrame(OutBuffer + FrameLen, &tmp,
						1,							&ExtRateIe,
						1,							&pAd->PortCfg.ExtRateLen,
						pAd->PortCfg.ExtRateLen,	pAd->PortCfg.ExtRate,							
						END_OF_ARGS);
			FrameLen += tmp;
		}
		
		if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPA) && (pAd->PortCfg.WepStatus == Ndis802_11Encryption2Enabled))
		{
			MakeOutgoingFrame(OutBuffer + FrameLen, &tmp,
						1,						  	&WpaIe,
						1,						  	&CipherSuiteWpaTkipLen,
						CipherSuiteWpaTkipLen,	  	&CipherSuiteWpaTkip[0],
						END_OF_ARGS);
			FrameLen += tmp;
			
			// Add by James 03/06/27
			// Third add RSN
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &WpaIe, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &CipherSuiteWpaTkipLen, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, CipherSuiteWpaTkip, CipherSuiteWpaTkipLen);
			VarIesOffset += CipherSuiteWpaTkipLen;

			// Set Variable IEs Length
			pAd->PortCfg.ReqVarIELen = VarIesOffset;
			pAd->PortCfg.AssocInfo.RequestIELength = VarIesOffset;

			// OffsetResponseIEs follow ReqVarIE
			pAd->PortCfg.AssocInfo.OffsetResponseIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAd->PortCfg.ReqVarIELen;
			// End Add by James 
		}
		
		else if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPA) && (pAd->PortCfg.WepStatus == Ndis802_11Encryption3Enabled))
		{
			MakeOutgoingFrame(OutBuffer + FrameLen, &tmp,
						1,						  	&WpaIe,
						1,						  	&CipherSuiteWpaAesLen,
						CipherSuiteWpaAesLen,	  	&CipherSuiteWpaAes[0],
						END_OF_ARGS);
			FrameLen += tmp;
			
			// Add by James 03/06/27
			// Third add RSN
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &WpaIe, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &CipherSuiteWpaAesLen, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, CipherSuiteWpaAes, CipherSuiteWpaAesLen);
			VarIesOffset += CipherSuiteWpaAesLen;

			// Set Variable IEs Length
			pAd->PortCfg.ReqVarIELen = VarIesOffset;
			pAd->PortCfg.AssocInfo.RequestIELength = VarIesOffset;

			// OffsetResponseIEs follow ReqVarIE
			pAd->PortCfg.AssocInfo.OffsetResponseIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAd->PortCfg.ReqVarIELen;
			// End Add by James 
		}
		else if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK) && (pAd->PortCfg.WepStatus == Ndis802_11Encryption2Enabled))
		{
			MakeOutgoingFrame(OutBuffer + FrameLen, &tmp,
						1,						  	&WpaIe,
						1,						  	&CipherSuiteWpaPskTkipLen,
						CipherSuiteWpaPskTkipLen, 	&CipherSuiteWpaPskTkip[0],
						END_OF_ARGS);
			FrameLen += tmp;

			// Add by James 03/06/27
			// Third add RSN
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &WpaIe, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &CipherSuiteWpaPskTkipLen, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, CipherSuiteWpaPskTkip, CipherSuiteWpaPskTkipLen);
			VarIesOffset += CipherSuiteWpaPskTkipLen;

			// Set Variable IEs Length
			pAd->PortCfg.ReqVarIELen = VarIesOffset;
			pAd->PortCfg.AssocInfo.RequestIELength = VarIesOffset;

			// OffsetResponseIEs follow ReqVarIE
			pAd->PortCfg.AssocInfo.OffsetResponseIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAd->PortCfg.ReqVarIELen;
			// End Add by James 
		}
		else if ((pAd->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK) && (pAd->PortCfg.WepStatus == Ndis802_11Encryption3Enabled))
		{
			MakeOutgoingFrame(OutBuffer + FrameLen, 	&tmp,
						1,							  	&WpaIe,
						1,							  	&CipherSuiteWpaPskAesLen,
						CipherSuiteWpaPskAesLen,	  	&CipherSuiteWpaPskAes[0],
						END_OF_ARGS);
			FrameLen += tmp;

			// Add by James 03/06/27
			// Third add RSN
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &WpaIe, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, &CipherSuiteWpaPskAesLen, 1);
			VarIesOffset += 1;
			memcpy(pAd->PortCfg.ReqVarIEs + VarIesOffset, CipherSuiteWpaPskAes, CipherSuiteWpaPskAesLen);
			VarIesOffset += CipherSuiteWpaPskAesLen;

			// Set Variable IEs Length
			pAd->PortCfg.ReqVarIELen = VarIesOffset;
			pAd->PortCfg.AssocInfo.RequestIELength = VarIesOffset;

			// OffsetResponseIEs follow ReqVarIE
			pAd->PortCfg.AssocInfo.OffsetResponseIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAd->PortCfg.ReqVarIELen;
			// End Add by James 
		}
		else
		{
			// Add by James 03/06/27
			// Set Variable IEs Length
			pAd->PortCfg.ReqVarIELen = VarIesOffset;
			pAd->PortCfg.AssocInfo.RequestIELength = VarIesOffset;

			// OffsetResponseIEs follow ReqVarIE
			pAd->PortCfg.AssocInfo.OffsetResponseIEs = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION) + pAd->PortCfg.ReqVarIELen;
			// End Add by James 
		}
        MiniportMMRequest(pAd, OutBuffer, FrameLen);
            
		RTMPSetTimer(pAd, &pAd->Mlme.AssocAux.AssocTimer, Timeout);
        pAd->Mlme.AssocMachine.CurrState = ASSOC_WAIT_RSP;
    } 
    else 
    {
        DBGPRINT(RT_DEBUG_TRACE,"ASSOC - MlmeAssocReqAction() sanity check failed. BUG!!!!!! \n");
        pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
        MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, MLME_INVALID_FORMAT);
    }

}

/*
    ==========================================================================
    Description:
        mlme reassoc req handling procedure
    Parameters:
        Elem - 
    Pre:
        -# SSID  (Adapter->PortCfg.ssid[])
        -# BSSID (AP address, Adapter->PortCfg.bssid)
        -# Supported rates (Adapter->PortCfg.supported_rates[])
        -# Supported rates length (Adapter->PortCfg.supported_rates_len)
        -# Tx power (Adapter->PortCfg.tx_power)
    ==========================================================================
 */
VOID MlmeReassocReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR              ApAddr;
    MACHDR               ReassocHdr;
    UCHAR                SsidIe = IE_SSID, RateIe = IE_SUPP_RATES, ExtRateIe = IE_EXT_SUPP_RATES;
    USHORT               CapabilityInfo, ListenIntv;
    ULONG                Timeout;
    ULONG                FrameLen = 0;
    NDIS_STATUS          NStatus;
	ULONG			 	 tmp;
    UCHAR               *OutBuffer = NULL;

	// Block all authentication request durning WPA block period
	if (pAd->PortCfg.bBlockAssoc == TRUE)
	{
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Block ReAssoc request durning WPA block period!\n");
        pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
        MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, MLME_STATE_MACHINE_REJECT);
	}	
    // the parameters are the same as the association
    else if(MlmeAssocReqSanity(pAd, Elem->Msg, Elem->MsgLen, &ApAddr, &CapabilityInfo, &Timeout, &ListenIntv)) 
    {
        RTMPCancelTimer(&pAd->Mlme.AssocAux.ReassocTimer);

        NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
        if(NStatus != NDIS_STATUS_SUCCESS) 
        {
            DBGPRINT(RT_DEBUG_TRACE,"ASSOC - MlmeReassocReqAction() allocate memory failed \n");
            pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
            MlmeCntlConfirm(pAd, MT2_REASSOC_CONF, MLME_FAIL_NO_RESOURCE);
            return;
        }

		// Mask out unnecessary capability information
		CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;  // pAd->PortCfg.SupportedCapabilityInfo;
        pAd->Mlme.AssocAux.CapabilityInfo = CapabilityInfo;
        COPY_MAC_ADDR(&pAd->Mlme.AssocAux.Addr, &ApAddr);
        pAd->Mlme.AssocAux.ListenIntv = ListenIntv;

        // make frame, use bssid as the AP address??
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Send RE-ASSOC request...\n");
        MgtMacHeaderInit(pAd, &ReassocHdr, SUBTYPE_REASSOC_REQ, 0, &ApAddr, &ApAddr);
        MakeOutgoingFrame(OutBuffer,            &FrameLen,
                          sizeof(MACHDR),       &ReassocHdr,
                          2,                    &CapabilityInfo,
                          2,                    &ListenIntv,
                          ETH_ALEN,         &pAd->PortCfg.Bssid,
                          1,                    &SsidIe,
                          1,                    &pAd->PortCfg.SsidLen, 
                          pAd->PortCfg.SsidLen, pAd->PortCfg.Ssid, 
                          1,                    &RateIe,
						  1,						&pAd->PortCfg.SupRateLen,
						  pAd->PortCfg.SupRateLen,  pAd->PortCfg.SupRate,
                          END_OF_ARGS);
		if (pAd->PortCfg.ExtRateLen != 0)
		{
			MakeOutgoingFrame(OutBuffer + FrameLen, &tmp,
						1,							&ExtRateIe,
						1,							&pAd->PortCfg.ExtRateLen,
						pAd->PortCfg.ExtRateLen,	pAd->PortCfg.ExtRate,							
						END_OF_ARGS);
			FrameLen += tmp;
		}
        MiniportMMRequest(pAd, OutBuffer, FrameLen);
            
        RTMPSetTimer(pAd, &pAd->Mlme.AssocAux.ReassocTimer, Timeout); /* in mSec */
        pAd->Mlme.AssocMachine.CurrState = REASSOC_WAIT_RSP;
    } 
    else 
    {
        DBGPRINT(RT_DEBUG_TRACE,"ASSOC - MlmeReassocReqAction() sanity check failed. BUG!!!! \n");
        pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
        MlmeCntlConfirm(pAd, MT2_REASSOC_CONF, MLME_INVALID_FORMAT);
    }
}

/*
    ==========================================================================
    Description:
        Upper layer issues disassoc request
    Parameters:
        Elem -
    ==========================================================================
 */
VOID MlmeDisassocReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MLME_DISASSOC_REQ_STRUCT *DisassocReq;
    MACHDR                DisassocHdr;
    CHAR                 *OutBuffer = NULL;
    ULONG                 FrameLen = 0;
    NDIS_STATUS           NStatus;
    ULONG                 Timeout = 0;

    // skip sanity check
    DisassocReq = (MLME_DISASSOC_REQ_STRUCT *)(Elem->Msg);

    NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - MlmeDisassocReqAction() allocate memory failed\n");
        pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
        MlmeCntlConfirm(pAd, MT2_DISASSOC_CONF, MLME_FAIL_NO_RESOURCE);
        return;
    }
    
    RTMPCancelTimer(&pAd->Mlme.AssocAux.DisassocTimer);
    
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Send DISASSOC request\n");
    MgtMacHeaderInit(pAd, &DisassocHdr, SUBTYPE_DISASSOC, 0, &pAd->PortCfg.Bssid, &pAd->PortCfg.Bssid);
    MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                      sizeof(MACHDR),       &DisassocHdr, 
                      2,                    &DisassocReq->Reason, 
                      END_OF_ARGS);
    MiniportMMRequest(pAd, OutBuffer, FrameLen);
    memset(&(pAd->PortCfg.Bssid), 0, ETH_ALEN);
    
    pAd->PortCfg.DisassocReason = REASON_DISASSOC_STA_LEAVING;
    COPY_MAC_ADDR(&pAd->PortCfg.DisassocSta, &DisassocReq->Addr);

    RTMPSetTimer(pAd, &pAd->Mlme.AssocAux.DisassocTimer, Timeout); /* in mSec */
    pAd->Mlme.AssocMachine.CurrState = DISASSOC_WAIT_RSP;
}

/*
    ==========================================================================
    Description:
        peer sends assoc rsp back
    Parameters:
        Elme - MLME message containing the received frame
    ==========================================================================
 */
VOID PeerAssocRspAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT        CapabilityInfo, Status, Aid;
    UCHAR         Rates[MAX_LEN_OF_SUPPORTED_RATES], RatesLen;
    MACADDR       Addr2;
    BOOLEAN       ExtendedRateIeExist;

    if (PeerAssocRspSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, &CapabilityInfo, &Status, &Aid, Rates, &RatesLen, &ExtendedRateIeExist)) 
    {
        // The frame is for me ?
        if(MAC_ADDR_EQUAL(&Addr2, &pAd->Mlme.AssocAux.Addr)) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "ASSOC - receive ASSOC_RSP to me (status=%d)\n", Status);
            RTMPCancelTimer(&pAd->Mlme.AssocAux.AssocTimer);
            if(Status == MLME_SUCCESS) 
            {
                // go to procedure listed on page 376
				// Mask out unnecessary capability information
				CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;  // pAd->PortCfg.SupportedCapabilityInfo;
                AssocPostProc(pAd, &Addr2, CapabilityInfo, Aid, Rates, RatesLen, ExtendedRateIeExist);
            } 
            pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
            MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, Status);
        } 
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - PeerAssocRspAction() sanity check fail\n");
    }
}

/*
    ==========================================================================
    Description:
        peer sends reassoc rsp
    Parametrs:
        Elem - MLME message cntaining the received frame
    ==========================================================================
 */
VOID PeerReassocRspAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    USHORT      CapabilityInfo;
    USHORT      Status;
    USHORT      Aid;
    UCHAR       Rates[MAX_LEN_OF_SUPPORTED_RATES];
    UCHAR       RatesLen;
    MACADDR     Addr2;
    BOOLEAN     ExtendedRateIeExist;

    if(PeerAssocRspSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, &CapabilityInfo, &Status, &Aid, Rates, &RatesLen, &ExtendedRateIeExist)) 
    {
        if(MAC_ADDR_EQUAL(&Addr2, &pAd->Mlme.AssocAux.Addr)) // The frame is for me ?
        {
            DBGPRINT(RT_DEBUG_TRACE, "ASSOC - receive REASSOC_RSP to me (status=%d)\n", Status);
            RTMPCancelTimer(&pAd->Mlme.AssocAux.ReassocTimer);
            
            if(Status == MLME_SUCCESS) 
            {
				// Mask out unnecessary capability information
				CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;  // pAd->PortCfg.SupportedCapabilityInfo;
                // go to procedure listed on page 376
                AssocPostProc(pAd, &Addr2, CapabilityInfo, Aid, Rates, RatesLen, ExtendedRateIeExist);
            } 

            pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
            MlmeCntlConfirm(pAd, MT2_REASSOC_CONF, Status);
        } 
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - PeerReassocRspAction() sanity check fail\n");
    }
}

/*
    ==========================================================================
    Description:
        procedures on IEEE 802.11/1999 p.376 
    Parametrs:
    ==========================================================================
 */
VOID AssocPostProc(
    IN PRTMP_ADAPTER pAd, 
    IN PMACADDR Addr2, 
    IN USHORT CapabilityInfo, 
    IN USHORT Aid, 
    IN UCHAR Rates[], 
    IN UCHAR RatesLen,
    IN BOOLEAN ExtendedRateIeExist) 
{
	ULONG Idx;
    UCHAR RateIe = IE_SUPP_RATES;
	UCHAR VarIesOffset;

    // 2003/12/11 -  skip the following because experiment show that we can not 
    // trust the "privacy" bit in AssocRsp. We can only trust "Privacy" bit specified in
    // BEACON and ProbeRsp.
    // pAd->PortCfg.PrivacyInvoked = CAP_IS_PRIVACY_ON(CapabilityInfo);
    
    pAd->PortCfg.Aid = Aid;
    memcpy(pAd->PortCfg.SupportedRates, Rates, RatesLen);
    pAd->PortCfg.SupportedRatesLen = RatesLen;
    COPY_MAC_ADDR(&pAd->PortCfg.Bssid, Addr2);
    AsicSetBssid(pAd, &pAd->PortCfg.Bssid);

    // set listen interval
    pAd->PortCfg.DefaultListenCount = pAd->Mlme.AssocAux.ListenIntv;
//  pAd->PortCfg.CurrListenCount = pAd->Mlme.AssocAux.ListenIntv;

	// Set New WPA information
	Idx = BssTableSearch(&pAd->PortCfg.BssTab, Addr2);
	if (Idx == BSS_NOT_FOUND) 
	{
		DBGPRINT(RT_DEBUG_ERROR, "ASSOC - Can't find BSS after receiving Assoc response\n");
	}
	else
	{
		// Mod by James to fix OID_802_11_ASSOCIATION_INFORMATION
		pAd->PortCfg.AssocInfo.Length = sizeof(NDIS_802_11_ASSOCIATION_INFORMATION); //+ sizeof(NDIS_802_11_FIXED_IEs); 	// Filled in assoc request
		pAd->PortCfg.AssocInfo.AvailableResponseFixedIEs =
			NDIS_802_11_AI_RESFI_CAPABILITIES | NDIS_802_11_AI_RESFI_STATUSCODE | NDIS_802_11_AI_RESFI_ASSOCIATIONID;
		pAd->PortCfg.AssocInfo.ResponseFixedIEs.Capabilities  = CapabilityInfo;
		pAd->PortCfg.AssocInfo.ResponseFixedIEs.StatusCode    = MLME_SUCCESS;		// Should be success, add failed later
		pAd->PortCfg.AssocInfo.ResponseFixedIEs.AssociationId = Aid;

		// Copy BSS VarIEs to PortCfg associnfo structure.
		// First add Supported rates
		VarIesOffset = 0;
		memcpy(pAd->PortCfg.ResVarIEs + VarIesOffset, &RateIe, 1);
		VarIesOffset += 1;
		memcpy(pAd->PortCfg.ResVarIEs + VarIesOffset, &RatesLen, 1);
		VarIesOffset += 1;
		memcpy(pAd->PortCfg.ResVarIEs + VarIesOffset, Rates, RatesLen);
		VarIesOffset += RatesLen;

		// Second add RSN
		memcpy(pAd->PortCfg.ResVarIEs + VarIesOffset, pAd->PortCfg.BssTab.BssEntry[Idx].VarIEs, pAd->PortCfg.BssTab.BssEntry[Idx].VarIELen);
		VarIesOffset += pAd->PortCfg.BssTab.BssEntry[Idx].VarIELen;
		
		// Set Variable IEs Length
		pAd->PortCfg.ResVarIELen = VarIesOffset;
		pAd->PortCfg.AssocInfo.ResponseIELength = VarIesOffset;
		// End Mod by James
	}
}

/*
    ==========================================================================
    Description:
        left part of IEEE 802.11/1999 p.374 
    Parameters:
        Elem - MLME message containing the received frame
    ==========================================================================
 */
VOID PeerDisassocAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR       Addr2;
    USHORT        Reason;

    if(PeerDisassocSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, &Reason)) 
    {
        if (INFRA_ON(pAd) && MAC_ADDR_EQUAL(&pAd->PortCfg.Bssid, &Addr2)) 
        {	
            LinkDown(pAd);
            pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;

            pAd->RalinkCounters.BeenDisassociatedCount ++;
            DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Disassociated by AP, Auto Recovery attempt #%d\n", pAd->RalinkCounters.BeenDisassociatedCount);
            MlmeAutoReconnectLastSSID(pAd);
        }
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "ASSOC - PeerDisassocAction() sanity check fail\n");
    }
}

/*
    ==========================================================================
    Description:
        what the state machine will do after assoc timeout
    Parameters:
        Elme -
    ==========================================================================
 */
VOID AssocTimeoutAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - AssocTimeoutAction\n");
    pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
    MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, MLME_REJ_TIMEOUT);
}

/*
    ==========================================================================
    Description:
        what the state machine will do after reassoc timeout
    ==========================================================================
 */
VOID ReassocTimeoutAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - ReassocTimeoutAction\n");
    pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
    MlmeCntlConfirm(pAd, MT2_REASSOC_CONF, MLME_REJ_TIMEOUT);
}

/*
    ==========================================================================
    Description:
        what the state machine will do after disassoc timeout
    ==========================================================================
 */
VOID DisassocTimeoutAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - DisassocTimeoutAction\n");
    pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
    MlmeCntlConfirm(pAd, MT2_DISASSOC_CONF, MLME_SUCCESS);
}

VOID InvalidStateWhenAssoc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - InvalidStateWhenAssoc(state=%d), reset ASSOC state machine\n", 
        pAd->Mlme.AssocMachine.CurrState);
    pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
    MlmeCntlConfirm(pAd, MT2_ASSOC_CONF, MLME_STATE_MACHINE_REJECT);
}

VOID InvalidStateWhenReassoc(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - InvalidStateWhenReassoc(state=%d), reset ASSOC state machine\n", 
        pAd->Mlme.AssocMachine.CurrState);
    pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
    MlmeCntlConfirm(pAd, MT2_REASSOC_CONF, MLME_STATE_MACHINE_REJECT);
}

VOID InvalidStateWhenDisassociate(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - InvalidStateWhenDisassoc(state=%d), reset ASSOC state machine\n", 
        pAd->Mlme.AssocMachine.CurrState);
    pAd->Mlme.AssocMachine.CurrState = ASSOC_IDLE;
    MlmeCntlConfirm(pAd, MT2_DISASSOC_CONF, MLME_STATE_MACHINE_REJECT);
}

/*
    ==========================================================================
    Description:
        right part of IEEE 802.11/1999 page 374
    Note: 
        This event should never cause ASSOC state machine perform state
        transition, and has no relationship with CNTL machine. So we separate
        this routine as a service outside of ASSOC state transition table.
    ==========================================================================
 */
VOID Cls3errAction(
    IN PRTMP_ADAPTER pAd, 
    IN PMACADDR      pAddr) 
{
    MACHDR                DisassocHdr;
    CHAR                 *OutBuffer = NULL;
    ULONG                 FrameLen = 0;
    NDIS_STATUS           NStatus;
    USHORT                Reason = REASON_CLS3ERR;

    NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS) 
        return;
    
    DBGPRINT(RT_DEBUG_TRACE, "ASSOC - Class 3 Error, Send DISASSOC frame\n");
    MgtMacHeaderInit(pAd, &DisassocHdr, SUBTYPE_DISASSOC, 0, pAddr, &pAd->PortCfg.Bssid);
    MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                      sizeof(MACHDR),       &DisassocHdr, 
                      2,                    &Reason, 
                      END_OF_ARGS);
    MiniportMMRequest(pAd, OutBuffer, FrameLen);

    pAd->PortCfg.DisassocReason = REASON_CLS3ERR;
    COPY_MAC_ADDR(&pAd->PortCfg.DisassocSta, pAddr);
}
 

