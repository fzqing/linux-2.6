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
 *      Module Name: auth.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#include "rt_config.h"

/*
    ==========================================================================
    Description:
        authenticate state machine init, including state transition and timer init
    Parameters:
        Sm - pointer to the auth state machine
    Note:
        The state machine looks like this
        
                        AUTH_REQ_IDLE           AUTH_WAIT_SEQ2                   AUTH_WAIT_SEQ4
    MT2_MLME_AUTH_REQ   mlme_auth_req_action    invalid_state_when_auth          invalid_state_when_auth
    MT2_MLME_DEAUTH_REQ mlme_deauth_req_action  mlme_deauth_req_action           mlme_deauth_req_action
    MT2_CLS2ERR         cls2err_action          cls2err_action                   cls2err_action
    MT2_PEER_AUTH_EVEN  drop                    peer_auth_even_at_seq2_action    peer_auth_even_at_seq4_action
    MT2_AUTH_TIMEOUT    Drop                    auth_timeout_action              auth_timeout_action
    ==========================================================================
 */

void AuthStateMachineInit(
    IN PRTMP_ADAPTER pAd, 
    IN STATE_MACHINE *Sm, 
    OUT STATE_MACHINE_FUNC Trans[]) 
{
    StateMachineInit(Sm, (STATE_MACHINE_FUNC*)Trans, MAX_AUTH_STATE, MAX_AUTH_MSG, (STATE_MACHINE_FUNC)Drop, AUTH_REQ_IDLE, AUTH_MACHINE_BASE);
     
    // the first column
    StateMachineSetAction(Sm, AUTH_REQ_IDLE, MT2_MLME_AUTH_REQ, (STATE_MACHINE_FUNC)MlmeAuthReqAction);
//  StateMachineSetAction(Sm, AUTH_REQ_IDLE, MT2_MLME_DEAUTH_REQ, (STATE_MACHINE_FUNC)MlmeDeauthReqAction);
//  StateMachineSetAction(Sm, AUTH_REQ_IDLE, MT2_CLS2ERR, (STATE_MACHINE_FUNC)Cls2errAction);

    // the second column
    StateMachineSetAction(Sm, AUTH_WAIT_SEQ2, MT2_MLME_AUTH_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenAuth);
//  StateMachineSetAction(Sm, AUTH_WAIT_SEQ2, MT2_MLME_DEAUTH_REQ, (STATE_MACHINE_FUNC)MlmeDeauthReqAction);
//  StateMachineSetAction(Sm, AUTH_WAIT_SEQ2, MT2_CLS2ERR, (STATE_MACHINE_FUNC)Cls2errAction);
    StateMachineSetAction(Sm, AUTH_WAIT_SEQ2, MT2_PEER_AUTH_EVEN, (STATE_MACHINE_FUNC)PeerAuthRspAtSeq2Action);
    StateMachineSetAction(Sm, AUTH_WAIT_SEQ2, MT2_AUTH_TIMEOUT, (STATE_MACHINE_FUNC)AuthTimeoutAction);
    
    // the third column
    StateMachineSetAction(Sm, AUTH_WAIT_SEQ4, MT2_MLME_AUTH_REQ, (STATE_MACHINE_FUNC)InvalidStateWhenAuth);
//  StateMachineSetAction(Sm, AUTH_WAIT_SEQ4, MT2_MLME_DEAUTH_REQ, (STATE_MACHINE_FUNC)MlmeDeauthReqAction);
//  StateMachineSetAction(Sm, AUTH_WAIT_SEQ4, MT2_CLS2ERR, (STATE_MACHINE_FUNC)Cls2errAction);
    StateMachineSetAction(Sm, AUTH_WAIT_SEQ4, MT2_PEER_AUTH_EVEN, (STATE_MACHINE_FUNC)PeerAuthRspAtSeq4Action);
    StateMachineSetAction(Sm, AUTH_WAIT_SEQ4, MT2_AUTH_TIMEOUT, (STATE_MACHINE_FUNC)AuthTimeoutAction);
    
    RTMPInitTimer(pAd, &pAd->Mlme.AuthAux.AuthTimer, AuthTimeout);
}

/*
    ==========================================================================
    Description:
        function to be executed at timer thread when auth timer expires
    ==========================================================================
 */
VOID AuthTimeout(
    IN	unsigned long data)
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    
    DBGPRINT(RT_DEBUG_TRACE,"AUTH - AuthTimeout\n");
    MlmeEnqueue(&pAd->Mlme.Queue, AUTH_STATE_MACHINE, MT2_AUTH_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}


/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID MlmeAuthReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR            Addr;
    USHORT             Alg, Seq, Status;
    ULONG              Timeout;
    MACHDR             AuthHdr;
    NDIS_STATUS        NStatus;
    UCHAR             *OutBuffer = NULL;
    ULONG              FrameLen = 0;

    // Block all authentication request durning WPA block period
    if (pAd->PortCfg.bBlockAssoc == TRUE)
    {
        DBGPRINT(RT_DEBUG_TRACE, "AUTH - Block Auth request durning WPA block period!\n");
        pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
        MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_STATE_MACHINE_REJECT);
    }
    else if(MlmeAuthReqSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr, &Timeout, &Alg)) 
    {
        // reset timer
        RTMPCancelTimer(&pAd->Mlme.AuthAux.AuthTimer);
        pAd->Mlme.AuthAux.Addr = Addr;
        pAd->Mlme.AuthAux.Alg  = Alg;
        pAd->PortCfg.Mauth = FALSE;
        Seq = 1;
        Status = MLME_SUCCESS;
        
        NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
        if(NStatus != NDIS_STATUS_SUCCESS)
        {
            DBGPRINT(RT_DEBUG_TRACE, "AUTH - MlmeAuthReqAction() allocate memory failed\n");
            pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
            MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_FAIL_NO_RESOURCE);
            return;
        }

        DBGPRINT(RT_DEBUG_TRACE, "AUTH - Send AUTH request seq#1 (Alg=%d)...\n", Alg);
        MgtMacHeaderInit(pAd, &AuthHdr, SUBTYPE_AUTH, 0, &Addr, &pAd->PortCfg.Bssid);
        MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                          MAC_HDR_LEN,          &AuthHdr, 
                          2,                    &Alg, 
                          2,                    &Seq, 
                          2,                    &Status, 
                          END_OF_ARGS);
        MiniportMMRequest(pAd, OutBuffer, FrameLen);

        RTMPSetTimer(pAd, &pAd->Mlme.AuthAux.AuthTimer, Timeout);
        pAd->Mlme.AuthMachine.CurrState = AUTH_WAIT_SEQ2;
    } 
    else 
    {
        printk(KERN_ERR DRV_NAME "AUTH - MlmeAuthReqAction() sanity check failed. BUG!!!!!\n");
        pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
        MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_INVALID_FORMAT);
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID PeerAuthRspAtSeq2Action(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR       Addr2;
    USHORT        Seq, Status, RemoteStatus, Alg;
    UCHAR         ChlgText[CIPHER_TEXT_LEN];
    UCHAR         CyperChlgText[CIPHER_TEXT_LEN + 8 + 8];
    UCHAR         Element[2];
    MACHDR        AuthHdr;
    UCHAR        *OutBuffer = NULL;
    NDIS_STATUS   NStatus;
    ULONG         FrameLen = 0;

    if (PeerAuthSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, &Alg, &Seq, &Status, ChlgText)) 
    {
        if (MAC_ADDR_EQUAL(&pAd->Mlme.AuthAux.Addr, &Addr2) && Seq == 2) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "AUTH - Receive AUTH_RSP seq#2 to me (Alg=%d, Status=%d)\n", Alg, Status);
            RTMPCancelTimer(&pAd->Mlme.AuthAux.AuthTimer);
            
            if (Status == MLME_SUCCESS) 
            {
                if (pAd->Mlme.AuthAux.Alg == Ndis802_11AuthModeOpen) 
                {
                    pAd->PortCfg.Mauth = TRUE;
                    pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
                    MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_SUCCESS);
                } 
                else
                {
                    // 2. shared key, need to be challenged
                    Seq++;
                    RemoteStatus = MLME_SUCCESS;
                    NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
                    if(NStatus != NDIS_STATUS_SUCCESS)
                    {
                        DBGPRINT(RT_DEBUG_TRACE, "AUTH - PeerAuthRspAtSeq2Action() allocate memory fail\n");
                        pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
                        MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_FAIL_NO_RESOURCE);
                        return;
                    }
                    
                    DBGPRINT(RT_DEBUG_TRACE, "AUTH - Send AUTH request seq#3...\n");
                    MgtMacHeaderInit(pAd, &AuthHdr, SUBTYPE_AUTH, 0, &Addr2, &pAd->PortCfg.Bssid);
                    AuthHdr.Wep = 1;
                    // Encrypt challenge text & auth information
                    RTMPInitWepEngine(
                    	pAd,
                    	pAd->PortCfg.SharedKey[pAd->PortCfg.DefaultKeyId].Key,
                    	pAd->PortCfg.DefaultKeyId,
                    	pAd->PortCfg.SharedKey[pAd->PortCfg.DefaultKeyId].KeyLen,
                    	CyperChlgText);
#ifdef BIG_ENDIAN
                    Alg = SWAP16(*(USHORT *)&Alg);
                    Seq = SWAP16(*(USHORT *)&Seq);
                    RemoteStatus= SWAP16(*(USHORT *)&RemoteStatus);
                    pAd->NeedSwapToLittleEndian = FALSE;
#endif
                    RTMPEncryptData(pAd, (PUCHAR) &Alg, CyperChlgText + 4, 2);
                    RTMPEncryptData(pAd, (PUCHAR) &Seq, CyperChlgText + 6, 2);
                    RTMPEncryptData(pAd, (PUCHAR) &RemoteStatus, CyperChlgText + 8, 2);

                    Element[0] = 16;
                    Element[1] = 128;
                    RTMPEncryptData(pAd, Element, CyperChlgText + 10, 2);
                    RTMPEncryptData(pAd, ChlgText, CyperChlgText + 12, 128);
                    RTMPSetICV(pAd, CyperChlgText + 140);
                    MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                                      MAC_HDR_LEN,          &AuthHdr,  
                                      CIPHER_TEXT_LEN + 16, CyperChlgText, 
                                      END_OF_ARGS);
                    MiniportMMRequest(pAd, OutBuffer, FrameLen);
#ifdef BIG_ENDIAN
                    pAd->NeedSwapToLittleEndian = TRUE;
#endif
                    RTMPSetTimer(pAd, &pAd->Mlme.AuthAux.AuthTimer, AUTH_TIMEOUT);
                    pAd->Mlme.AuthMachine.CurrState = AUTH_WAIT_SEQ4;
                }
            } 
            else 
            {
                pAd->PortCfg.AuthFailReason = Status;
                COPY_MAC_ADDR(&pAd->PortCfg.AuthFailSta, &Addr2);
                pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
                MlmeCntlConfirm(pAd, MT2_AUTH_CONF, Status);
            }
        }
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "AUTH - PeerAuthSanity() sanity check fail\n");
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID PeerAuthRspAtSeq4Action(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MACADDR       Addr2;
    USHORT        Alg, Seq, Status;
    CHAR          ChlgText[CIPHER_TEXT_LEN];

    if(PeerAuthSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, &Alg, &Seq, &Status, ChlgText)) 
    {
        if(MAC_ADDR_EQUAL(&(pAd->Mlme.AuthAux.Addr), &Addr2) && Seq == 4) 
        {
            DBGPRINT(RT_DEBUG_TRACE, "AUTH - Receive AUTH_RSP seq#4 to me\n");
            RTMPCancelTimer(&pAd->Mlme.AuthAux.AuthTimer);
            
            if(Status == MLME_SUCCESS) 
            {
                pAd->PortCfg.Mauth = TRUE;
            } 
            else 
            {
                pAd->PortCfg.AuthFailReason = Status;
                pAd->PortCfg.AuthFailSta = Addr2;
            }                

            pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
            MlmeCntlConfirm(pAd, MT2_AUTH_CONF, Status);
        }
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "AUTH - PeerAuthRspAtSeq4Action() sanity check fail\n");
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID MlmeDeauthReqAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    MLME_DEAUTH_REQ_STRUCT *Info;
    MACHDR        Hdr;
    UCHAR        *OutBuffer = NULL;
    NDIS_STATUS   NStatus;
    ULONG         FrameLen = 0;

    Info = (MLME_DEAUTH_REQ_STRUCT *)Elem->Msg;

    NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS)
    {
        DBGPRINT(RT_DEBUG_TRACE, "AUTH - MlmeDeauthReqAction() allocate memory fail\n");
        pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
        MlmeCntlConfirm(pAd, MT2_DEAUTH_CONF, MLME_FAIL_NO_RESOURCE);
        return;
    }

    DBGPRINT(RT_DEBUG_TRACE, "AUTH - Send DE-AUTH request...\n");
    MgtMacHeaderInit(pAd, &Hdr, SUBTYPE_DEAUTH, 0, &Info->Addr, &pAd->PortCfg.Bssid);
    MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                      sizeof(MACHDR),       &Hdr, 
                      2,                    &Info->Reason, 
                      END_OF_ARGS);
    MiniportMMRequest(pAd, OutBuffer, FrameLen);
    
    pAd->PortCfg.DeauthReason = Info->Reason;
    COPY_MAC_ADDR(&pAd->PortCfg.DeauthSta, &Info->Addr);
    pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
    MlmeCntlConfirm(pAd, MT2_DEAUTH_CONF, MLME_SUCCESS);
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID AuthTimeoutAction(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "AUTH - AuthTimeoutAction\n");
    pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
    MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_REJ_TIMEOUT);
}

/*
    ==========================================================================
    Description:
    ==========================================================================
 */
VOID InvalidStateWhenAuth(
    IN PRTMP_ADAPTER pAd, 
    IN MLME_QUEUE_ELEM *Elem) 
{
    DBGPRINT(RT_DEBUG_TRACE, "AUTH - InvalidStateWhenAuth (state=%d), reset AUTH state machine\n", pAd->Mlme.AuthMachine.CurrState);
    pAd->Mlme.AuthMachine.CurrState = AUTH_REQ_IDLE;
    MlmeCntlConfirm(pAd, MT2_AUTH_CONF, MLME_STATE_MACHINE_REJECT);
}

/*
    ==========================================================================
    Description:
        Some STA/AP
    Note:
        This action should never trigger AUTH state transition, therefore we
        separate it from AUTH state machine, and make it as a standalone service
    ==========================================================================
 */
VOID Cls2errAction(
    IN PRTMP_ADAPTER pAd, 
    IN PMACADDR pAddr) 
{
    MACHDR        Hdr;
    UCHAR        *OutBuffer = NULL;
    NDIS_STATUS   NStatus;
    ULONG         FrameLen = 0;
    USHORT        Reason = REASON_CLS2ERR;
    
    NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS)
        return;

    DBGPRINT(RT_DEBUG_TRACE, "AUTH - Class 2 error, Send DEAUTH frame...\n");
    MgtMacHeaderInit(pAd, &Hdr, SUBTYPE_DEAUTH, 0, pAddr, &pAd->PortCfg.Bssid);
    MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                      sizeof(MACHDR),       &Hdr, 
                      2,                    &Reason, 
                      END_OF_ARGS);
    MiniportMMRequest(pAd, OutBuffer, FrameLen);

    pAd->PortCfg.DeauthReason = Reason;
    COPY_MAC_ADDR(&pAd->PortCfg.DeauthSta, pAddr);
}


