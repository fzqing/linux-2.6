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
 *      Module Name: auth_rsp.c 
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
        authentication state machine init procedure
    Parameters:
        Sm - the state machine
    Note:
        the state machine looks like the following 
        
                                        AUTH_RSP_IDLE                   AUTH_RSP_WAIT_CHAL
    MT2_AUTH_CHALLENGE_TIMEOUT    auth_rsp_challenge_timeout_action    auth_rsp_challenge_timeout_action
    MT2_PEER_AUTH_ODD        peer_auth_at_auth_rsp_idle_action peer_auth_at_auth_rsp_wait_action
    MT2_PEER_DEAUTH                 peer_deauth_action                 peer_deauth_action
    ==========================================================================
 */
VOID AuthRspStateMachineInit(
    IN PRTMP_ADAPTER pAd, 
    IN PSTATE_MACHINE Sm, 
    IN STATE_MACHINE_FUNC Trans[]) 
{
    ULONG        NOW;

    StateMachineInit(Sm, (STATE_MACHINE_FUNC*)Trans, MAX_AUTH_RSP_STATE, MAX_AUTH_RSP_MSG, (STATE_MACHINE_FUNC)Drop, AUTH_RSP_IDLE, AUTH_RSP_MACHINE_BASE);

    // column 1
//  StateMachineSetAction(Sm, AUTH_RSP_IDLE, MT2_AUTH_CHALLENGEG_TIMEOUT, (STATE_MACHINE_FUNC)AuthRspChallengeTimeoutAction);
//  StateMachineSetAction(Sm, AUTH_RSP_IDLE, MT2_PEER_AUTH_ODD, (STATE_MACHINE_FUNC)PeerAuthAtAuthRspIdleAction);
    StateMachineSetAction(Sm, AUTH_RSP_IDLE, MT2_PEER_DEAUTH, (STATE_MACHINE_FUNC)PeerDeauthAction);

    // column 2
//  StateMachineSetAction(Sm, AUTH_RSP_WAIT_CHAL, MT2_PEER_AUTH_ODD, (STATE_MACHINE_FUNC)PeerAuthAtAuthRspWaitAction);
//  StateMachineSetAction(Sm, AUTH_RSP_WAIT_CHAL, MT2_AUTH_CHALLENGE_TIMEOUT, (STATE_MACHINE_FUNC)AuthRspChallengeTimeoutAction);
    StateMachineSetAction(Sm, AUTH_RSP_WAIT_CHAL, MT2_PEER_DEAUTH, (STATE_MACHINE_FUNC)PeerDeauthAction);

    // initialize timer
    RTMPInitTimer(pAd, &pAd->Mlme.AuthRspAux.AuthRspTimer, AuthRspChallengeTimeout);

    // initialize the random number generator
    NOW = jiffies;
    LfsrInit(pAd, NOW);
}


/*
    ==========================================================================
    Description:
        challenge time out, called by timer thread
    ==========================================================================
 */
VOID AuthRspChallengeTimeout(
    IN  unsigned long data) 
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;
    
    DBGPRINT(RT_DEBUG_TRACE,"AUTH_RSP - AuthRspChallengeTimeout \n");
    MlmeEnqueue(&pAd->Mlme.Queue, AUTH_RSP_STATE_MACHINE, MT2_AUTH_CHALLENGE_TIMEOUT, 0, NULL);
    MlmeHandler(pAd);
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID PeerAuthSimpleRspGenAndSend(
    IN PRTMP_ADAPTER pAd, 
    IN PMACHDR Hdr, 
    IN USHORT Alg, 
    IN USHORT Seq, 
    IN USHORT Reason, 
    IN USHORT Status) 
{
    MACHDR            AuthHdr;
    UINT              FrameLen = 0;
    UCHAR            *OutBuffer = NULL;
    NDIS_STATUS       NStatus;

    NStatus = MlmeAllocateMemory(pAd, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS)
        return;

    if (Reason == MLME_SUCCESS)
    {
        DBGPRINT(RT_DEBUG_TRACE, "Send AUTH response (seq#2)...\n");
        MgtMacHeaderInit(pAd, &AuthHdr, SUBTYPE_AUTH, 0, &Hdr->Addr2, &pAd->PortCfg.Bssid);
        MakeOutgoingFrame(OutBuffer,            &FrameLen, 
                          sizeof(MACHDR),       &AuthHdr, 
                          2,                    &Alg, 
                          2,                    &Seq, 
                          2,                    &Reason, 
                          END_OF_ARGS);
        MiniportMMRequest(pAd, OutBuffer, FrameLen);
    }
    else
    {
        MlmeFreeMemory(pAd, OutBuffer);
        DBGPRINT(RT_DEBUG_TRACE, "Peer AUTH fail...\n");
    }
}

/*
    ==========================================================================
    Description:
    ==========================================================================
*/
VOID PeerDeauthAction(
    IN PRTMP_ADAPTER pAd, 
    IN PMLME_QUEUE_ELEM Elem) 
{
    MACADDR     Addr2;
    USHORT      Reason;

    if (PeerDeauthSanity(pAd, Elem->Msg, Elem->MsgLen, &Addr2, &Reason))
    {
        if (INFRA_ON(pAd) && MAC_ADDR_EQUAL(&Addr2, &pAd->PortCfg.Bssid))
        {
            RTMPCancelTimer(&pAd->Mlme.AuthRspAux.AuthRspTimer);
            DBGPRINT(RT_DEBUG_TRACE,"AUTH_RSP - receive DE-AUTH from our AP, reason = %d\n", Reason);
            LinkDown(pAd);
        }
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE,"AUTH_RSP - PeerDeauthAction() sanity check fail\n");
    }
}

