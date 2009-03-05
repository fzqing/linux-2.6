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
 *      Module Name: wpa.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      JanL            22nd Jul 03     Initial code     
 *      PaulL           28th Nov 03     Modify for supplicant
 *      MarkW           8th  Dec 04     Baseline code
 ***************************************************************************/ 

#include "rt_config.h"

UCHAR   CipherWpaPskTkip[] = {
        0xDD, 0x16,             // RSN IE
        0x00, 0x50, 0xf2, 0x01, // oui
        0x01, 0x00,             // Version
        0x00, 0x50, 0xf2, 0x02, // Multicast
        0x01, 0x00,             // Number of unicast
        0x00, 0x50, 0xf2, 0x02, // unicast
        0x01, 0x00,             // number of authentication method
        0x00, 0x50, 0xf2, 0x02  // authentication
        };
UCHAR   CipherWpaPskTkipLen = (sizeof(CipherWpaPskTkip) / sizeof(UCHAR));

UCHAR   CipherWpaPskAes[] = {
        0xDD, 0x16,             // RSN IE
        0x00, 0x50, 0xf2, 0x01, // oui
        0x01, 0x00,             // Version
        0x00, 0x50, 0xf2, 0x04, // Multicast
        0x01, 0x00,             // Number of unicast
        0x00, 0x50, 0xf2, 0x04, // unicast
        0x01, 0x00,             // number of authentication method
        0x00, 0x50, 0xf2, 0x02  // authentication
        };
UCHAR   CipherWpaPskAesLen = (sizeof(CipherWpaPskAes) / sizeof(UCHAR));

/*
    ========================================================================
    
    Routine Description:
        Classify WPA EAP message type

    Arguments:
        EAPType     Value of EAP message type
        MsgType     Internal Message definition for MLME state machine
        
    Return Value:
        TRUE        Found appropriate message type
        FALSE       No appropriate message type

    Note:
        All these constants are defined in wpa.h
        For supplicant, there is only EAPOL Key message avaliable
        
    ========================================================================
*/
BOOLEAN WpaMsgTypeSubst(
    IN  UCHAR   EAPType,
    OUT ULONG   *MsgType)   
{
    switch (EAPType)
    {
        case EAPPacket:
            *MsgType = EAP_MSG_TYPE_EAPPacket;
            break;
        case EAPOLStart:
            *MsgType = EAP_MSG_TYPE_EAPOLStart;
            break;
        case EAPOLLogoff:
            *MsgType = EAP_MSG_TYPE_EAPOLLogoff;
            break;
        case EAPOLKey:
            *MsgType = EAP_MSG_TYPE_EAPOLKey;
            break;
        case EAPOLASFAlert:
            *MsgType = EAP_MSG_TYPE_EAPOLASFAlert;
            break;
        default:
            DBGPRINT(RT_DEBUG_INFO, "WpaMsgTypeSubst : return FALSE; \n");
            return FALSE;       
    }   
    return TRUE;
}

/*  
    ==========================================================================
    Description: 
        association state machine init, including state transition and timer init
    Parameters: 
        S - pointer to the association state machine
    ==========================================================================
 */
VOID WpaPskStateMachineInit(
    IN  PRTMP_ADAPTER   pAd, 
    IN  STATE_MACHINE *S, 
    OUT STATE_MACHINE_FUNC Trans[]) 
{
    StateMachineInit(S, (STATE_MACHINE_FUNC*)Trans, MAX_WPA_PSK_STATE, MAX_WPA_PSK_MSG, (STATE_MACHINE_FUNC)Drop, WPA_PSK_IDLE, WPA_MACHINE_BASE);
    StateMachineSetAction(S, WPA_PSK_IDLE, EAP_MSG_TYPE_EAPOLKey, (STATE_MACHINE_FUNC)WpaEAPOLKeyAction);
}

/*
    ==========================================================================
    Description:
        This is state machine function. 
        When receiving EAPOL packets which is  for 802.1x key management. 
        Use both in WPA, and WPAPSK case. 
        In this function, further dispatch to different functions according to the received packet.  3 categories are : 
          1.  normal 4-way pairwisekey and 2-way groupkey handshake
          2.  MIC error (Countermeasures attack)  report packet from STA.
          3.  Request for pairwise/group key update from STA
    Return:
    ==========================================================================
*/
VOID WpaEAPOLKeyAction(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem) 
{
    INT             MsgType;
    UCHAR           ZeroReplay[LEN_KEY_DESC_REPLAY];
    PKEY_DESCRIPTER pKeyDesc;
    
    DBGPRINT(RT_DEBUG_TRACE, "-----> WpaEAPOLKeyAction\n");
    // Get 802.11 header first
    pKeyDesc = (PKEY_DESCRIPTER) &Elem->Msg[(LENGTH_802_11 + LENGTH_802_1_H + LENGTH_EAPOL_H)];

#ifdef BIG_ENDIAN
	*(USHORT *)((UCHAR *)pKeyDesc+1) = SWAP16(*(USHORT *)((UCHAR *)pKeyDesc+1));
#endif
    // Sanity check, this should only happen in WPA-PSK mode
    if (pAdapter->PortCfg.AuthMode != Ndis802_11AuthModeWPAPSK)
        return;

    // 0. Debug print all bit information
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Key Description Version %d\n", pKeyDesc->KeyInfo.KeyDescVer);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Key Type %d\n", pKeyDesc->KeyInfo.KeyType);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Key Index %d\n", pKeyDesc->KeyInfo.KeyIndex);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Install %d\n", pKeyDesc->KeyInfo.Install);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Key Ack %d\n", pKeyDesc->KeyInfo.KeyAck);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Key MIC %d\n", pKeyDesc->KeyInfo.KeyMic);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Secure %d\n", pKeyDesc->KeyInfo.Secure);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Error %d\n", pKeyDesc->KeyInfo.Error);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo Request %d\n", pKeyDesc->KeyInfo.Request);
    DBGPRINT(RT_DEBUG_INFO, "KeyInfo DL %d\n", pKeyDesc->KeyInfo.DL);
    
    // 1. Check EAPOL frame version and type
    if ((Elem->Msg[LENGTH_802_11+LENGTH_802_1_H] != EAPOL_VER) || (pKeyDesc->Type != RSN_KEY_DESC))
    {
        DBGPRINT(RT_DEBUG_ERROR, "   Key descripter does not match with WPA rule \n");
        return;
    }

    // 2.Check Version for AES & TKIP
    if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled) && (pKeyDesc->KeyInfo.KeyDescVer != DESC_TYPE_AES))
    {
        DBGPRINT(RT_DEBUG_ERROR, "   Key descripter version not match AES \n");
        return;
    }
    else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pKeyDesc->KeyInfo.KeyDescVer != DESC_TYPE_TKIP))
    {
        DBGPRINT(RT_DEBUG_ERROR, "   Key descripter version not match TKIP \n");
        return;
    }

    // First validate replay counter, only accept message with larger replay counter
    // Let equal pass, some AP start with all zero replay counter
    memset(ZeroReplay, 0, LEN_KEY_DESC_REPLAY);
    if ((RTMPCompareMemory(pKeyDesc->ReplayCounter, pAdapter->PortCfg.ReplayCounter, LEN_KEY_DESC_REPLAY) != 1) &&
        (RTMPCompareMemory(pKeyDesc->ReplayCounter, ZeroReplay, LEN_KEY_DESC_REPLAY) != 0))
        return;

    // Classify message Type, either pairwise message 1, 3, or group message 1 for supplicant
    MsgType = EAPOL_MSG_INVALID;
    if ((pKeyDesc->KeyInfo.KeyType == 1) &&
        (pKeyDesc->KeyInfo.KeyIndex == 0) &&
        (pKeyDesc->KeyInfo.KeyAck == 1) &&
        (pKeyDesc->KeyInfo.KeyMic == 0) &&
        (pKeyDesc->KeyInfo.Secure == 0) &&
        (pKeyDesc->KeyInfo.Error == 0) &&
        (pKeyDesc->KeyInfo.Request == 0))
    {
        MsgType = EAPOL_PAIR_MSG_1;
        DBGPRINT(RT_DEBUG_TRACE, "Receive EAPOL Key Pairwise Message 1\n");
    }
    else if ((pKeyDesc->KeyInfo.KeyType == 1) &&
        (pKeyDesc->KeyInfo.KeyIndex == 0) &&
        (pKeyDesc->KeyInfo.KeyAck == 1) &&
        (pKeyDesc->KeyInfo.KeyMic == 1) &&
        (pKeyDesc->KeyInfo.Secure == 0) &&
        (pKeyDesc->KeyInfo.Error == 0) &&
        (pKeyDesc->KeyInfo.Request == 0))
    {
        MsgType = EAPOL_PAIR_MSG_3;
        DBGPRINT(RT_DEBUG_TRACE, "Receive EAPOL Key Pairwise Message 3\n");
    }
    else if ((pKeyDesc->KeyInfo.KeyType == 0) &&
        (pKeyDesc->KeyInfo.KeyIndex != 0) &&
        (pKeyDesc->KeyInfo.KeyAck == 1) &&
        (pKeyDesc->KeyInfo.KeyMic == 1) &&
        (pKeyDesc->KeyInfo.Secure == 1) &&
        (pKeyDesc->KeyInfo.Error == 0) &&
        (pKeyDesc->KeyInfo.Request == 0))
    {
        MsgType = EAPOL_GROUP_MSG_1;
        DBGPRINT(RT_DEBUG_TRACE, "Receive EAPOL Key Group Message 1\n");
    }
    
#ifdef BIG_ENDIAN
	*(USHORT *)((UCHAR *)pKeyDesc+1) = SWAP16(*(USHORT *)((UCHAR *)pKeyDesc+1));
#endif
    
    // We will assume link is up (assoc suceess and port not secured).
    // All state has to be able to process message from previous state
    switch (pAdapter->PortCfg.WpaState)
    {
        case SS_START:
            if (MsgType == EAPOL_PAIR_MSG_1)
            {
                WpaPairMsg1Action(pAdapter, Elem);
                pAdapter->PortCfg.WpaState = SS_WAIT_MSG_3;
            }
            break;
                
        case SS_WAIT_MSG_3:
            if (MsgType == EAPOL_PAIR_MSG_1)
            {
                WpaPairMsg1Action(pAdapter, Elem);
                pAdapter->PortCfg.WpaState = SS_WAIT_MSG_3;
            }
            else if (MsgType == EAPOL_PAIR_MSG_3)
            {
                WpaPairMsg3Action(pAdapter, Elem);
                pAdapter->PortCfg.WpaState = SS_WAIT_GROUP;
            }
            break;
                
        case SS_WAIT_GROUP:     // When doing group key exchange
        case SS_FINISH:         // This happened when update group key
            if (MsgType == EAPOL_PAIR_MSG_1)
            {
                WpaPairMsg1Action(pAdapter, Elem);
                pAdapter->PortCfg.WpaState = SS_WAIT_MSG_3;
                // Reset port secured variable
                pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;
            }
            else if (MsgType == EAPOL_PAIR_MSG_3)
            {
                WpaPairMsg3Action(pAdapter, Elem);
                pAdapter->PortCfg.WpaState = SS_WAIT_GROUP;
                // Reset port secured variable
                pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;
            }
            else if (MsgType == EAPOL_GROUP_MSG_1)
            {
                WpaGroupMsg1Action(pAdapter, Elem);
                pAdapter->PortCfg.WpaState = SS_FINISH;
            }
            break;
                
        default:
            break;              
    }
    
    DBGPRINT(RT_DEBUG_TRACE, "<----- WpaEAPOLKeyAction\n");
}

/*
    ========================================================================
    
    Routine Description:
        Process Pairwise key 4-way handshaking

    Arguments:
        pAdapter    Pointer to our adapter
        Elem        Message body
        
    Return Value:
        None
        
    Note:
        
    ========================================================================
*/
VOID    WpaPairMsg1Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem) 
{
    PHEADER_802_11      pHeader;
    UCHAR               PTK[80];
    UCHAR               *OutBuffer = NULL;
    HEADER_802_11       Header_802_11;
    NDIS_STATUS         NStatus;
    UCHAR               AckRate = RATE_2;
    USHORT              AckDuration = 0;
    ULONG               FrameLen = 0;
    UCHAR               EAPHEAD[8] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00,0x88,0x8e};
    PEAPOL_PACKET       pMsg1;
    EAPOL_PACKET        Packet;
    UCHAR               Mic[16];    
       
    DBGPRINT(RT_DEBUG_TRACE, "WpaPairMsg1Action ----->\n");
    
    pHeader = (PHEADER_802_11) Elem->Msg;
    
    // Save Data Length to pDesc for receiving packet, then put in outgoing frame   Data Len fields.
    pMsg1 = (PEAPOL_PACKET) &Elem->Msg[LENGTH_802_11 + LENGTH_802_1_H];
    
    // Process message 1 from authenticator
    // Key must be Pairwise key, already verified at callee.
    // 1. Save Replay counter, it will use to verify message 3 and construct message 2
    memcpy(pAdapter->PortCfg.ReplayCounter, pMsg1->KeyDesc.ReplayCounter, LEN_KEY_DESC_REPLAY);     

    // 2. Save ANonce
    memcpy(pAdapter->PortCfg.ANonce, pMsg1->KeyDesc.KeyNonce, LEN_KEY_DESC_NONCE);
        
    // TSNonce <--- SNonce
    // Generate random SNonce
    GenRandom(pAdapter, pAdapter->PortCfg.SNonce);  

    // TPTK <--- Calc PTK(ANonce, TSNonce)
    WpaCountPTK(pAdapter->PortCfg.PskKey.Key,   
        pAdapter->PortCfg.ANonce,
        pAdapter->PortCfg.Bssid.Octet, 
        pAdapter->PortCfg.SNonce, 
        pAdapter->CurrentAddress,    
        PTK, 
        LEN_PTK);   

    // Save key to PTK entry
    memcpy(pAdapter->PortCfg.PTK, PTK, LEN_PTK);
    
    // =====================================
    // Use Priority Ring & MiniportMMRequest
    // =====================================
    pAdapter->Sequence = ((pAdapter->Sequence) + 1) & (MAX_SEQ_NUMBER);
    WpaMacHeaderInit(pAdapter, &Header_802_11, 0, &pAdapter->PortCfg.Bssid);

    // ACK size is 14 include CRC, and its rate is based on real time information
    AckRate = pAdapter->PortCfg.ExpectedACKRate[pAdapter->PortCfg.TxRate];
    AckDuration = RTMPCalcDuration(pAdapter, AckRate, 14);
    Header_802_11.Controlhead.Duration = pAdapter->PortCfg.Dsifs + AckDuration;
    
    // Zero message 2 body
    memset(&Packet, 0, sizeof(Packet));
    Packet.Version = EAPOL_VER;
    Packet.Type    = EAPOLKey;
    //
    // Message 2 as  EAPOL-Key(0,1,0,0,0,P,0,SNonce,MIC,RSN IE)
    //
    Packet.KeyDesc.Type = RSN_KEY_DESC;
    // 1. Key descriptor version and appropriate RSN IE
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        Packet.KeyDesc.KeyInfo.KeyDescVer = 2;
        Packet.KeyDesc.KeyDataLen[1] = CipherWpaPskAesLen;
        memcpy(Packet.KeyDesc.KeyData, CipherWpaPskAes, CipherWpaPskAesLen);
    }
    else    // TKIP
    {
        Packet.KeyDesc.KeyInfo.KeyDescVer = 1;
        Packet.KeyDesc.KeyDataLen[1] = CipherWpaPskTkipLen;
        memcpy(Packet.KeyDesc.KeyData, CipherWpaPskTkip, CipherWpaPskTkipLen);
    }
    // Update packet length after decide Key data payload
    Packet.Len[1]  = sizeof(KEY_DESCRIPTER) - MAX_LEN_OF_RSNIE + Packet.KeyDesc.KeyDataLen[1];

    // 2. Key Type PeerKey
    Packet.KeyDesc.KeyInfo.KeyType = 1;

    // 3. KeyMic field presented
    Packet.KeyDesc.KeyInfo.KeyMic  = 1;

    // 4. Fill SNonce
    memcpy(Packet.KeyDesc.KeyNonce, pAdapter->PortCfg.SNonce, LEN_KEY_DESC_NONCE);

    // 5. Key Replay Count
    memcpy(Packet.KeyDesc.ReplayCounter, pAdapter->PortCfg.ReplayCounter, LEN_KEY_DESC_REPLAY);     
    
#ifdef BIG_ENDIAN
	*(USHORT *)(&(Packet.KeyDesc.KeyInfo)) = SWAP16(*(USHORT *)(&(Packet.KeyDesc.KeyInfo)));
#endif
    
    // Send EAPOL(0, 1, 0, 0, 0, K, 0, TSNonce, 0, MIC(TPTK), 0)
    // Out buffer for transmitting message 2        
    NStatus = MlmeAllocateMemory(pAdapter, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS) 
        return;                 

    // Prepare EAPOL frame for MIC calculation
    // Be careful, only EAPOL frame is counted for MIC calculation
    MakeOutgoingFrame(OutBuffer, &FrameLen,
        Packet.Len[1] + 4, &Packet,
        END_OF_ARGS);

    // 5. Prepare and Fill MIC value
    memset(Mic, 0, sizeof(Mic));
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        // AES
        UCHAR digest[80];
            
        HMAC_SHA1(OutBuffer, FrameLen, PTK, LEN_EAP_MICK, digest);
        memcpy(Mic, digest, LEN_KEY_DESC_MIC);
    }
    else
    {
        INT i;
        DBGPRINT(RT_DEBUG_INFO, " PMK = ");
        for (i = 0; i < 16; i++)
            DBGPRINT(RT_DEBUG_INFO, "%2x-", pAdapter->PortCfg.PskKey.Key[i]);
        
        DBGPRINT(RT_DEBUG_INFO, "\n PTK = ");
        for (i = 0; i < 64; i++)
            DBGPRINT(RT_DEBUG_INFO, "%2x-", pAdapter->PortCfg.PTK[i]);
        DBGPRINT(RT_DEBUG_INFO, "\n FrameLen = %d\n", FrameLen);
        
        hmac_md5(PTK,  LEN_EAP_MICK, OutBuffer, FrameLen, Mic);
    }
    memcpy(Packet.KeyDesc.KeyMic, Mic, LEN_KEY_DESC_MIC);

    FrameLen = 0;
    // Make  Transmitting frame
    MakeOutgoingFrame(OutBuffer, &FrameLen, sizeof(MACHDR), &Header_802_11,
        sizeof(EAPHEAD), EAPHEAD, 
        Packet.Len[1] + 4, &Packet,
        END_OF_ARGS);

    // Send using priority queue
    MiniportMMRequest(pAdapter, OutBuffer, FrameLen);
        
    DBGPRINT(RT_DEBUG_TRACE, "WpaPairMsg1Action <-----\n");
}

/*
    ========================================================================
    
    Routine Description:
        Process Pairwise key 4-way handshaking

    Arguments:
        pAdapter    Pointer to our adapter
        Elem        Message body
        
    Return Value:
        None
        
    Note:
        
    ========================================================================
*/
VOID    WpaPairMsg3Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem) 
{
    PHEADER_802_11      pHeader;
    UCHAR               *OutBuffer = NULL;
    HEADER_802_11       Header_802_11;
    NDIS_STATUS         NStatus;
    UCHAR               AckRate = RATE_2;
    USHORT              AckDuration = 0;
    ULONG               FrameLen = 0;
    UCHAR               EAPHEAD[8] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00,0x88,0x8e};
    EAPOL_PACKET        Packet;
    PEAPOL_PACKET       pMsg3;
    PUCHAR              pTmp;
    UCHAR               Mic[16], OldMic[16];    
    NDIS_802_11_KEY     PeerKey;
    
       
    DBGPRINT(RT_DEBUG_TRACE, "WpaPairMsg3Action ----->\n");
    
    pHeader = (PHEADER_802_11) Elem->Msg;
    
    // Process message 3 frame.
    pMsg3 = (PEAPOL_PACKET) &Elem->Msg[LENGTH_802_11 + LENGTH_802_1_H];

#ifdef BIG_ENDIAN
	*(USHORT *)(&(pMsg3->KeyDesc.KeyInfo)) = SWAP16(*(USHORT *)(&(pMsg3->KeyDesc.KeyInfo)));
#endif

    // 1. Verify RSN IE & cipher type match
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        if (pMsg3->KeyDesc.KeyInfo.KeyDescVer != 2)
            return;
        pTmp = (PUCHAR) &CipherWpaPskAes;
    }
    else    // TKIP
    {
        if (pMsg3->KeyDesc.KeyInfo.KeyDescVer != 1)
            return;
        pTmp = (PUCHAR) &CipherWpaPskTkip;
    }

    // Fix compatibility issue, when AP append nonsense data after auth mode with different size.
    // We should qualify this kind of RSN as acceptable
    if (!NdisEqualMemory((PUCHAR) &pMsg3->KeyDesc.KeyData[2], pTmp + 2, CipherWpaPskTkipLen - 2))
    {
        DBGPRINT(RT_DEBUG_ERROR, " RSN IE mismatched msg 3 of 4-way handshake!!!!!!!!!! \n");
        return;
    }
    else
        DBGPRINT(RT_DEBUG_TRACE, " RSN IE matched in msg 3 of 4-way handshake!!!!!!!!!! \n");
    
#ifdef BIG_ENDIAN
	*(USHORT *)(&(pMsg3->KeyDesc.KeyInfo)) = SWAP16(*(USHORT *)(&(pMsg3->KeyDesc.KeyInfo)));
#endif

    // 2. Check MIC value
    // Save the MIC and replace with zero
    memcpy(OldMic, pMsg3->KeyDesc.KeyMic, LEN_KEY_DESC_MIC);
    memset(pMsg3->KeyDesc.KeyMic, 0, LEN_KEY_DESC_MIC);
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        // AES
        UCHAR digest[80];
            
        HMAC_SHA1((PUCHAR) pMsg3, pMsg3->Len[1] + 4, pAdapter->PortCfg.PTK, LEN_EAP_MICK, digest);
        memcpy(Mic, digest, LEN_KEY_DESC_MIC);
    }
    else
    {
        hmac_md5(pAdapter->PortCfg.PTK, LEN_EAP_MICK, (PUCHAR) pMsg3, pMsg3->Len[1] + 4, Mic);
    }
    
    if (!NdisEqualMemory(OldMic, Mic, LEN_KEY_DESC_MIC))
    {
        DBGPRINT(RT_DEBUG_ERROR, " MIC Different in msg 3 of 4-way handshake!!!!!!!!!! \n");
        return;
    }
    else
        DBGPRINT(RT_DEBUG_TRACE, " MIC VALID in msg 3 of 4-way handshake!!!!!!!!!! \n");

    // 3. Check Replay Counter, it has to be larger than last one. No need to be exact one larger
    if (RTMPCompareMemory(pMsg3->KeyDesc.ReplayCounter, pAdapter->PortCfg.ReplayCounter, LEN_KEY_DESC_REPLAY) != 1)
        return;

    // Update new replay counter
    memcpy(pAdapter->PortCfg.ReplayCounter, pMsg3->KeyDesc.ReplayCounter, LEN_KEY_DESC_REPLAY);     

    // 4. Double check ANonce
    if (!NdisEqualMemory(pAdapter->PortCfg.ANonce, pMsg3->KeyDesc.KeyNonce, LEN_KEY_DESC_NONCE))
        return;
    
    // 5. Construct Message 4
    // =====================================
    // Use Priority Ring & MiniportMMRequest
    // =====================================
    pAdapter->Sequence = ((pAdapter->Sequence) + 1) & (MAX_SEQ_NUMBER);
    WpaMacHeaderInit(pAdapter, &Header_802_11, 0, &pAdapter->PortCfg.Bssid);

    // ACK size is 14 include CRC, and its rate is based on real time information
    AckRate = pAdapter->PortCfg.ExpectedACKRate[pAdapter->PortCfg.TxRate];
    AckDuration = RTMPCalcDuration(pAdapter, AckRate, 14);
    Header_802_11.Controlhead.Duration = pAdapter->PortCfg.Dsifs + AckDuration;
    
    // Zero message 4 body
    memset(&Packet, 0, sizeof(Packet));
    Packet.Version = EAPOL_VER;
    Packet.Type    = EAPOLKey;
    Packet.Len[1]  = sizeof(KEY_DESCRIPTER) - MAX_LEN_OF_RSNIE;     // No data field
    
    //
    // Message 4 as  EAPOL-Key(0,1,0,0,0,P,0,0,MIC,0)
    //
    Packet.KeyDesc.Type = RSN_KEY_DESC;
    
#ifdef BIG_ENDIAN
	*(USHORT *)(&(pMsg3->KeyDesc.KeyInfo)) = SWAP16(*(USHORT *)(&(pMsg3->KeyDesc.KeyInfo)));
#endif
    
    // Key descriptor version and appropriate RSN IE
    Packet.KeyDesc.KeyInfo.KeyDescVer = pMsg3->KeyDesc.KeyInfo.KeyDescVer;

    // Key Type PeerKey
    Packet.KeyDesc.KeyInfo.KeyType = 1;

    // KeyMic field presented
    Packet.KeyDesc.KeyInfo.KeyMic  = 1;

    // Key Replay count 
    memcpy(Packet.KeyDesc.ReplayCounter, pMsg3->KeyDesc.ReplayCounter, LEN_KEY_DESC_REPLAY);        
#ifdef BIG_ENDIAN
        *(USHORT *)&Packet.KeyDesc.KeyInfo = SWAP16(*(USHORT *)&Packet.KeyDesc.KeyInfo);
#endif

    // Out buffer for transmitting message 4        
    NStatus = MlmeAllocateMemory(pAdapter, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS)
        return;                 

    // Prepare EAPOL frame for MIC calculation
    // Be careful, only EAPOL frame is counted for MIC calculation
    MakeOutgoingFrame(OutBuffer, &FrameLen,
        Packet.Len[1] + 4, &Packet,
        END_OF_ARGS);

    // Prepare and Fill MIC value
    memset(Mic, 0, sizeof(Mic));
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        // AES
        UCHAR digest[80];
            
        HMAC_SHA1(OutBuffer, FrameLen, pAdapter->PortCfg.PTK, LEN_EAP_MICK, digest);
        memcpy(Mic, digest, LEN_KEY_DESC_MIC);
    }
    else
    {
        hmac_md5(pAdapter->PortCfg.PTK, LEN_EAP_MICK, OutBuffer, FrameLen, Mic);
    }
    memcpy(Packet.KeyDesc.KeyMic, Mic, LEN_KEY_DESC_MIC);

    FrameLen = 0;
    
    // Make  Transmitting frame
    MakeOutgoingFrame(OutBuffer, &FrameLen, sizeof(MACHDR), &Header_802_11,
        sizeof(EAPHEAD), EAPHEAD, 
        Packet.Len[1] + 4, &Packet,
        END_OF_ARGS);

    // 6. Send Message 4 to authenticator
    // Send using priority queue
    MiniportMMRequest(pAdapter, OutBuffer, FrameLen);

    // 7. Update PTK
    memset(&PeerKey, 0, sizeof(PeerKey));
    PeerKey.Length    = sizeof(PeerKey);
    PeerKey.KeyIndex  = 0xe0000000;           
    PeerKey.KeyLength = 16;
    memcpy(PeerKey.BSSID, pAdapter->PortCfg.Bssid.Octet, 6);
    memcpy(&PeerKey.KeyRSC, pMsg3->KeyDesc.KeyRsc, LEN_KEY_DESC_RSC);
    memcpy(PeerKey.KeyMaterial, &pAdapter->PortCfg.PTK[32], 32);
    // Call Add peer key function
    RTMPWPAAddKeyProc(pAdapter, &PeerKey);
    
    DBGPRINT(RT_DEBUG_TRACE, "WpaPairMsg3Action <-----\n");
}


/*
    ========================================================================
    
    Routine Description:
        Process Group key 2-way handshaking

    Arguments:
        pAdapter    Pointer to our adapter
        Elem        Message body
        
    Return Value:
        None
        
    Note:
        
    ========================================================================
*/
VOID    WpaGroupMsg1Action(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  MLME_QUEUE_ELEM *Elem) 
{
    PHEADER_802_11      pHeader;
    UCHAR               *OutBuffer = NULL;
    HEADER_802_11       Header_802_11;
    NDIS_STATUS         NStatus;
    UCHAR               AckRate = RATE_2;
    USHORT              AckDuration = 0;
    ULONG               FrameLen = 0;
    UCHAR               EAPHEAD[8] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00,0x88,0x8e};
    EAPOL_PACKET        Packet;
    PEAPOL_PACKET       pGroup;
    UCHAR               Mic[16], OldMic[16];
    UCHAR               GTK[32], Key[32];
    NDIS_802_11_KEY     GroupKey;
    
       
    DBGPRINT(RT_DEBUG_TRACE, "WpaGroupMsg1Action ----->\n");
    
    pHeader = (PHEADER_802_11) Elem->Msg;
    
    // Process Group message 1 frame.
    pGroup = (PEAPOL_PACKET) &Elem->Msg[LENGTH_802_11 + LENGTH_802_1_H];

    // 1. Verify Replay counter
    //    Check Replay Counter, it has to be larger than last one. No need to be exact one larger
    if (RTMPCompareMemory(pGroup->KeyDesc.ReplayCounter, pAdapter->PortCfg.ReplayCounter, LEN_KEY_DESC_REPLAY) != 1)
        return;

    // Update new replay counter
    memcpy(pAdapter->PortCfg.ReplayCounter, pGroup->KeyDesc.ReplayCounter, LEN_KEY_DESC_REPLAY);        

    // 2. Verify MIC is valid
    // Save the MIC and replace with zero
    memcpy(OldMic, pGroup->KeyDesc.KeyMic, LEN_KEY_DESC_MIC);
    memset(pGroup->KeyDesc.KeyMic, 0, LEN_KEY_DESC_MIC);
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        // AES
        UCHAR digest[80];
            
        HMAC_SHA1((PUCHAR) pGroup, pGroup->Len[1] + 4, pAdapter->PortCfg.PTK, LEN_EAP_MICK, digest);
        memcpy(Mic, digest, LEN_KEY_DESC_MIC);
    }
    else
    {
        hmac_md5(pAdapter->PortCfg.PTK, LEN_EAP_MICK, (PUCHAR) pGroup, pGroup->Len[1] + 4, Mic);
    }
    
    if (!NdisEqualMemory(OldMic, Mic, LEN_KEY_DESC_MIC))
    {
        DBGPRINT(RT_DEBUG_ERROR, " MIC Different in group msg 1 of 2-way handshake!!!!!!!!!! \n");
        return;
    }
    else
        DBGPRINT(RT_DEBUG_TRACE, " MIC VALID in group msg 1 of 2-way handshake!!!!!!!!!! \n");

#ifdef BIG_ENDIAN
	*(USHORT *)(&(pGroup->KeyDesc.KeyInfo)) = SWAP16(*(USHORT *)(&(pGroup->KeyDesc.KeyInfo)));
#endif

    // 3. Decrypt GTK from Key Data
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        if (pGroup->KeyDesc.KeyInfo.KeyDescVer != 2)
            return;
        // Decrypt AES GTK
        AES_GTK_KEY_UNWRAP(&pAdapter->PortCfg.PTK[16], GTK, pGroup->KeyDesc.KeyData);       
    }
    else    // TKIP
    {
        INT i;
        
        if (pGroup->KeyDesc.KeyInfo.KeyDescVer != 1)
            return;
        // Decrypt TKIP GTK
        // Construct 32 bytes RC4 Key
        memcpy(Key, pGroup->KeyDesc.KeyIv, 16);
        memcpy(&Key[16], &pAdapter->PortCfg.PTK[16], 16);
        ARCFOUR_INIT(&pAdapter->PrivateInfo.WEPCONTEXT, Key, 32);
        //discard first 256 bytes
        for (i = 0; i < 256; i++)
            ARCFOUR_BYTE(&pAdapter->PrivateInfo.WEPCONTEXT);
        // Decrypt GTK. Becareful, there is no ICV to check the result is correct or not
        ARCFOUR_DECRYPT(&pAdapter->PrivateInfo.WEPCONTEXT, GTK, pGroup->KeyDesc.KeyData, 32);       
    }
    
    // 4. Construct Group Message 2
    pAdapter->Sequence = ((pAdapter->Sequence) + 1) & (MAX_SEQ_NUMBER);
    WpaMacHeaderInit(pAdapter, &Header_802_11, 1, &pAdapter->PortCfg.Bssid);

    // ACK size is 14 include CRC, and its rate is based on real time information
    AckRate = pAdapter->PortCfg.ExpectedACKRate[pAdapter->PortCfg.TxRate];
    AckDuration = RTMPCalcDuration(pAdapter, AckRate, 14);
    Header_802_11.Controlhead.Duration = pAdapter->PortCfg.Dsifs + AckDuration;
    
    // Zero Group message 1 body
    memset(&Packet, 0, sizeof(Packet));
    Packet.Version = EAPOL_VER;
    Packet.Type    = EAPOLKey;
    Packet.Len[1]  = sizeof(KEY_DESCRIPTER) - MAX_LEN_OF_RSNIE;     // No data field
    
    //
    // Group Message 2 as  EAPOL-Key(1,0,0,0,G,0,0,MIC,0)
    //
    Packet.KeyDesc.Type = RSN_KEY_DESC;
    
    // Key descriptor version and appropriate RSN IE
    Packet.KeyDesc.KeyInfo.KeyDescVer = pGroup->KeyDesc.KeyInfo.KeyDescVer;

    // Key Type Group key
    Packet.KeyDesc.KeyInfo.KeyType = 0;

    // KeyMic field presented
    Packet.KeyDesc.KeyInfo.KeyMic  = 1;

    // Secure bit is 1
    Packet.KeyDesc.KeyInfo.Secure  = 1;
    
    // Key Replay count 
    memcpy(Packet.KeyDesc.ReplayCounter, pGroup->KeyDesc.ReplayCounter, LEN_KEY_DESC_REPLAY);       

#ifdef BIG_ENDIAN
	*(USHORT *)(&(Packet.KeyDesc.KeyInfo)) = SWAP16(*(USHORT *)(&(Packet.KeyDesc.KeyInfo)));
#endif

    // Out buffer for transmitting group message 2      
    NStatus = MlmeAllocateMemory(pAdapter, (PVOID)&OutBuffer);  //Get an unused nonpaged memory
    if (NStatus != NDIS_STATUS_SUCCESS)
        return;                 

    // Prepare EAPOL frame for MIC calculation
    // Be careful, only EAPOL frame is counted for MIC calculation
    MakeOutgoingFrame(OutBuffer, &FrameLen,
        Packet.Len[1] + 4, &Packet,
        END_OF_ARGS);

    // Prepare and Fill MIC value
    memset(Mic, 0, sizeof(Mic));
    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        // AES
        UCHAR digest[80];
            
        HMAC_SHA1(OutBuffer, FrameLen, pAdapter->PortCfg.PTK, LEN_EAP_MICK, digest);
        memcpy(Mic, digest, LEN_KEY_DESC_MIC);
    }
    else
    {
        INT i;
        DBGPRINT(RT_DEBUG_INFO, "PTK = ");
        for (i = 0; i < 64; i++)
            DBGPRINT(RT_DEBUG_INFO, "%2x-", pAdapter->PortCfg.PTK[i]);
        DBGPRINT(RT_DEBUG_INFO, "\n FrameLen = %d\n", FrameLen);
            
        hmac_md5(pAdapter->PortCfg.PTK, LEN_EAP_MICK, OutBuffer, FrameLen, Mic);
    }
    memcpy(Packet.KeyDesc.KeyMic, Mic, LEN_KEY_DESC_MIC);

    FrameLen = 0;   
    // Make Transmitting frame
    MakeOutgoingFrame(OutBuffer, &FrameLen, sizeof(MACHDR), &Header_802_11,
        sizeof(EAPHEAD), EAPHEAD, 
        Packet.Len[1] + 4, &Packet,
        END_OF_ARGS);

    // 5. Copy frame to Tx ring and prepare for encryption
    WpaHardEncrypt(pAdapter, OutBuffer, FrameLen);

    // 6 Free allocated memory
    MlmeFreeMemory(pAdapter, OutBuffer);
    
    // 6. Update GTK
    memset(&GroupKey, 0, sizeof(GroupKey));
    GroupKey.Length    = sizeof(GroupKey);
    GroupKey.KeyIndex  = 0x20000000 | pGroup->KeyDesc.KeyInfo.KeyIndex;           
    GroupKey.KeyLength = 16;
    memcpy(GroupKey.BSSID, pAdapter->PortCfg.Bssid.Octet, 6);
    memcpy(GroupKey.KeyMaterial, GTK, 32);
    // Call Add peer key function
    RTMPWPAAddKeyProc(pAdapter, &GroupKey);
    
    DBGPRINT(RT_DEBUG_TRACE, "WpaGroupMsg1Action <-----\n");
}
/*
    ========================================================================
    
    Routine Description:
        Init WPA MAC header

    Arguments:
        pAdapter    Pointer to our adapter
        
    Return Value:
        None
        
    Note:
        
    ========================================================================
*/
VOID    WpaMacHeaderInit(
    IN      PRTMP_ADAPTER   pAd, 
    IN OUT  PHEADER_802_11  Hdr, 
    IN      UCHAR           wep, 
    IN      PMACADDR        pAddr1) 
{
    memset(Hdr, 0, sizeof(HEADER_802_11));
    Hdr->Controlhead.Frame.Type = BTYPE_DATA;   
    Hdr->Controlhead.Frame.ToDs = 1;
    if (wep == 1)
        Hdr->Controlhead.Frame.Wep = 1;
    
     // Addr1: DA, Addr2: BSSID, Addr3: SA
    COPY_MAC_ADDR(&Hdr->Controlhead.Addr1, pAddr1);
    COPY_MAC_ADDR(&Hdr->Controlhead.Addr2, &pAd->CurrentAddress);
    COPY_MAC_ADDR(&Hdr->Addr3, &pAd->PortCfg.Bssid);
    Hdr->Sequence = pAd->Sequence;      
}

/*
    ========================================================================

    Routine Description:
        Copy frame from waiting queue into relative ring buffer and set 
    appropriate ASIC register to kick hardware encryption before really
    sent out to air.
        
    Arguments:
        pAdapter        Pointer to our adapter
        PNDIS_PACKET    Pointer to outgoing Ndis frame
        NumberOfFrag    Number of fragment required
        
    Return Value:
        None

    Note:
    
    ========================================================================
*/
VOID    WpaHardEncrypt(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PUCHAR          pPacket,
    IN  ULONG           Len)
{
    UCHAR           FrameGap;
    PUCHAR          pDest;
    PUCHAR          pSrc;
    UCHAR           CipherAlg = CIPHER_NONE;
    PTXD_STRUC      pTxD;
#ifdef BIG_ENDIAN
    TXD_STRUC       TxD;
    PTXD_STRUC      pDestTxD;
    PUCHAR          pOriginDest;
#endif
    ULONG           Iv16;
    ULONG           Iv32;
    PWPA_KEY        pWpaKey;
    UCHAR           RetryMode = SHORT_RETRY;
    static UCHAR    Priority[4] = {"\x00\x00\x00\x00"};

    // Make sure Tx ring resource won't be used by other threads
    spin_lock(&pAdapter->TxRingLock);

    FrameGap = IFS_BACKOFF;     // Default frame gap mode
    
    // outgoing frame always wakeup PHY to prevent frame lost and 
    // turn off PSM bit to improve performance
    if (pAdapter->PortCfg.Psm == PWR_SAVE)
    {
        MlmeSetPsmBit(pAdapter, PWR_ACTIVE);
    }
    AsicForceWakeup(pAdapter);
    
    pAdapter->TxRing[pAdapter->CurEncryptIndex].FrameType = BTYPE_DATA;

    pSrc = pPacket; // Point to start of MSDU
    
    pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.PairwiseKey[0];
    pWpaKey->Type = PAIRWISE_KEY;
    if (pWpaKey == NULL)
    {
        // No pairwise key, this should not happen
        spin_unlock(&pAdapter->TxRingLock);
        return;
    }
    
    // Get the Tx Ring descriptor & Dma Buffer address
    pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;              
#ifndef BIG_ENDIAN
    pTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
#else
	pOriginDest = pDest;
	pDestTxD = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
	TxD = *pDestTxD;
	pTxD = &TxD;
	RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif
        
    if ((pTxD->Owner == DESC_OWN_NIC) || (pTxD->CipherOwn == DESC_OWN_NIC))
    {
        // Descriptor owned by NIC. No descriptor avaliable
        // This should not happen since caller guaranteed.
        // Make sure to release Tx ring resource
        pAdapter->RalinkCounters.TxRingErrCount++;
        spin_unlock(&pAdapter->TxRingLock);
        return;
    }
    if (pTxD->Valid == TRUE)
    {
        // Ndis packet of last round did not cleared.
        // This should not happen since caller guaranteed.
        // Make sure to release Tx ring resource
        pTxD->Valid = FALSE;
                
#ifdef BIG_ENDIAN
		RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
		*pDestTxD = TxD;
#endif

        pAdapter->RalinkCounters.TxRingErrCount++;
        spin_unlock(&pAdapter->TxRingLock);
        return;
    }
        
    // Copy whole frame to Tx ring buffer
    memcpy(pDest, pPacket, Len);
    pDest += Len;

    if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled)
    {
        INT     i;

        i = 0;
        // Prepare 8 bytes TKIP encapsulation for MPDU
        {
            TKIP_IV	tkipIv;

            tkipIv.IV16.field.rc0 = *(pWpaKey->TxTsc + 1);
            tkipIv.IV16.field.rc1 = (tkipIv.IV16.field.rc0 | 0x20) & 0x7f;
            tkipIv.IV16.field.rc2 = *pWpaKey->TxTsc;
            tkipIv.IV16.field.Rsvd = 0;
            tkipIv.IV16.field.ExtIV = 1;  // 0: non-extended IV, 1: an extended IV
            tkipIv.IV16.field.KeyID = 0;
            tkipIv.IV32 = *(PULONG)(pWpaKey->TxTsc + 2);

            pTxD->Iv = tkipIv.IV16.word;

            *((PUCHAR) &pTxD->Eiv) = *((PUCHAR) &tkipIv.IV32 + 3);
            *((PUCHAR) &pTxD->Eiv + 1) = *((PUCHAR) &tkipIv.IV32 + 2);
            *((PUCHAR) &pTxD->Eiv + 2) = *((PUCHAR) &tkipIv.IV32 + 1);
            *((PUCHAR) &pTxD->Eiv + 3) = *((PUCHAR) &tkipIv.IV32);
        }
            
        // Increase TxTsc value for next transmission
        while (++pWpaKey->TxTsc[i] == 0x0)
        {
            i++;
            if (i == 6)
                break;
        }
            
        // Set IV offset
        pTxD->IvOffset = LENGTH_802_11;

        // Copy TKey
        memcpy(pTxD->Key, pWpaKey->Key, 16);
            
        // Set Cipher suite
        CipherAlg = CIPHER_TKIP;

        // Calculate MIC value
        // Init MIC value calculation and reset the message
        pAdapter->PrivateInfo.Tx.L = RTMPTkipGetUInt32(pWpaKey->TxMic);
        pAdapter->PrivateInfo.Tx.R = RTMPTkipGetUInt32(pWpaKey->TxMic + 4);
        pAdapter->PrivateInfo.Tx.nBytesInM = 0;
        pAdapter->PrivateInfo.Tx.M = 0;
    	
        // DA & SA field
        RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, pSrc + 4, 12);
        
        // Priority + 3 bytes of 0
        RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, Priority, 4);

        pSrc += LENGTH_802_11;
        RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, pSrc, Len - LENGTH_802_11);
        RTMPTkipGetMIC(&pAdapter->PrivateInfo.Tx);
        // Append MIC
        memcpy(pDest, pAdapter->PrivateInfo.Tx.MIC, 8);
        Len += 8;
        // IV + EIV + ICV which ASIC added after encryption done
        Len += 12;
    }
    else if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
    {
        INT     i;
        PUCHAR  pTmp;

        i = 0;
        pTmp = (PUCHAR) &Iv16;
        *pTmp       = pWpaKey->TxTsc[0];
        *(pTmp + 1) = pWpaKey->TxTsc[1];
        *(pTmp + 2) = 0;
        *(pTmp + 3) = 0x20;
            
        Iv32 = *(PULONG)(&pWpaKey->TxTsc[2]);
            
        // Increase TxTsc value for next transmission
        while (++pWpaKey->TxTsc[i] == 0x0)
        {
            i++;
            if (i == 6)
                break;
        }
            
        // Copy IV
        memcpy(&pTxD->Iv, &Iv16, 4);
            
        // Copy EIV
        memcpy(&pTxD->Eiv, &Iv32, 4);
            
        // Set IV offset
        pTxD->IvOffset = LENGTH_802_11;

        // Copy TKey
        memcpy(pTxD->Key, pWpaKey->Key, 16);

        // Set Cipher suite
        CipherAlg = CIPHER_AES;
            
        // IV + EIV + HW MIC
        Len += 16;
    }               
                
#ifdef BIG_ENDIAN
	RTMPFrameEndianChange(pAdapter, pOriginDest, DIR_WRITE, FALSE);
 	RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
	*pDestTxD = TxD;
	pTxD = pDestTxD;
#endif

    RTMPWriteTxDescriptor(pTxD, TRUE, CipherAlg, TRUE, FALSE, FALSE, RetryMode, FrameGap, 
           pAdapter->PortCfg.TxRate, 4, Len, pAdapter->PortCfg.TxPreambleInUsed, 0);

    // Increase & maintain Tx Ring Index
    pAdapter->CurEncryptIndex++;
    if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
    {
        pAdapter->CurEncryptIndex = 0;
    }       
    pAdapter->RalinkCounters.EncryptCount++;        
    
    // Kick Encrypt Control Register at the end of all ring buffer preparation
    RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
        
    // Make sure to release Tx ring resource
    spin_unlock(&pAdapter->TxRingLock);
}

/*
    ========================================================================
    
    Routine Description:
        SHA1 function 

    Arguments:
        
    Return Value:

    Note:
        
    ========================================================================
*/
VOID    HMAC_SHA1(
    IN  UCHAR   *text,
    IN  UINT    text_len,
    IN  UCHAR   *key,
    IN  UINT    key_len,
    IN  UCHAR   *digest)
{
    SHA_CTX context;
    UCHAR   k_ipad[65]; /* inner padding - key XORd with ipad   */
    UCHAR   k_opad[65]; /* outer padding - key XORd with opad   */
    INT     i;

    // if key is longer than 64 bytes reset it to key=SHA1(key) 
    if (key_len > 64) 
    {
        SHA_CTX      tctx;
        SHAInit(&tctx);
        SHAUpdate(&tctx, key, key_len);
        SHAFinal(&tctx, key);
        key_len = 20;
    }
    memset(k_ipad, 0, sizeof(k_ipad));
    memset(k_opad, 0, sizeof(k_opad));
    memcpy(k_ipad, key, key_len);
    memcpy(k_opad, key, key_len);

    // XOR key with ipad and opad values  
    for (i = 0; i < 64; i++) 
    {   
        k_ipad[i] ^= 0x36;
        k_opad[i] ^= 0x5c;
    }

    // perform inner SHA1 
    SHAInit(&context);                      /* init context for 1st pass */
    SHAUpdate(&context, k_ipad, 64);        /*  start with inner pad */
    SHAUpdate(&context, text, text_len);    /*  then text of datagram */
    SHAFinal(&context, digest);             /* finish up 1st pass */

    //perform outer SHA1  
    SHAInit(&context);                  /* init context for 2nd pass */
    SHAUpdate(&context, k_opad, 64);    /*  start with outer pad */
    SHAUpdate(&context, digest, 20);    /*  then results of 1st hash */
    SHAFinal(&context, digest);         /* finish up 2nd pass */
}

/*
    ========================================================================
    
    Routine Description:
        PRF function 

    Arguments:
        
    Return Value:

    Note:
        802.1i  Annex F.9
        
    ========================================================================
*/
VOID    PRF(
    IN  UCHAR   *key,
    IN  INT     key_len,
    IN  UCHAR   *prefix,
    IN  INT     prefix_len,
    IN  UCHAR   *data,
    IN  INT     data_len,
    OUT UCHAR   *output,
    IN  INT     len)
{
    INT     i;
    UCHAR   input[1024];
    INT     currentindex = 0;
    INT     total_len;
    
    memcpy(input, prefix, prefix_len);
    input[prefix_len] = 0;
    memcpy(&input[prefix_len + 1], data, data_len);
    total_len = prefix_len + 1 + data_len;
    input[total_len] = 0;
    total_len++;
    for (i = 0; i < (len + 19) / 20; i++)
    {
        HMAC_SHA1(input, total_len, key, key_len, &output[currentindex]);
        currentindex += 20;
        input[total_len - 1]++;
    }   
}

/*
    ========================================================================
    
    Routine Description:
        Count TPTK from PMK

    Arguments:
        
    Return Value:
        Output      Store the TPTK

    Note:
        
    ========================================================================
*/
VOID WpaCountPTK(
    IN  UCHAR   *PMK,
    IN  UCHAR   *ANonce,
    IN  UCHAR   *AA,
    IN  UCHAR   *SNonce,
    IN  UCHAR   *SA,
    OUT UCHAR   *output,
    IN  UINT    len)
{   
    UCHAR   concatenation[76];
    UINT    CurrPos = 0;
    UCHAR   temp[32];
    UCHAR   Prefix[] = {'P', 'a', 'i', 'r', 'w', 'i', 's', 'e', ' ', 'k', 'e', 'y', ' ', 
                        'e', 'x', 'p', 'a', 'n', 's', 'i', 'o', 'n'};

    memset(temp, 0, sizeof(temp));

    // Get smaller address
    if (RTMPCompareMemory(SA, AA, 6) == 1)
        memcpy(concatenation, AA, 6);
    else
        memcpy(concatenation, SA, 6);       
    CurrPos += 6;

    // Get larger address
    if (RTMPCompareMemory(SA, AA, 6) == 1)
        memcpy(&concatenation[CurrPos], SA, 6);
    else
        memcpy(&concatenation[CurrPos], AA, 6);     
    CurrPos += 6;

    // Get smaller address
    if (RTMPCompareMemory(ANonce, SNonce, 32) == 1) 
        memcpy(&concatenation[CurrPos], SNonce, 32);
    else        
        memcpy(&concatenation[CurrPos], ANonce, 32);
    CurrPos += 32;

    // Get larger address
    if (RTMPCompareMemory(ANonce, SNonce, 32) == 1) 
        memcpy(&concatenation[CurrPos], ANonce, 32);
    else        
        memcpy(&concatenation[CurrPos], SNonce, 32);
    CurrPos += 32;
        
    PRF(PMK, LEN_MASTER_KEY, Prefix,  22, concatenation, 76 , output, len);
}

/*
    ========================================================================
    
    Routine Description:
        Misc function to Generate random number

    Arguments:
        
    Return Value:

    Note:
        802.1i  Annex F.9
        
    ========================================================================
*/
VOID    GenRandom(
    IN  PRTMP_ADAPTER   pAd, 
    OUT UCHAR           *random)
{   
    INT     i, curr;
    UCHAR   local[80], KeyCounter[32];
    UCHAR   result[80];
    ULONG   CurrentTime;
    UCHAR   prefix[] = {'I', 'n', 'i', 't', ' ', 'C', 'o', 'u', 'n', 't', 'e', 'r'};

    memset(result, 0, 80);
    memset(local, 0, 80);
    memset(KeyCounter, 0, 32);
    memcpy(local, pAd->CurrentAddress, ETH_ALEN);
    
    for (i = 0; i < 32; i++)
    {       
        curr =  ETH_ALEN;
        CurrentTime = jiffies;
        memcpy(local,  pAd->CurrentAddress, ETH_ALEN);
        curr += ETH_ALEN;
        memcpy(&local[curr],  &CurrentTime, sizeof(CurrentTime));
        curr += sizeof(CurrentTime);
        memcpy(&local[curr],  result, 32);
        curr += 32;
        memcpy(&local[curr],  &i,  2);      
        curr += 2;
        PRF(KeyCounter, 32, prefix,12, local,   curr, result, 32); 
    }
    memcpy(random, result,  32);    
}

/*
    ========================================================================
    
    Routine Description:
        Misc function to decrypt AES body
    
    Arguments:
            
    Return Value:
    
    Note:
        This function references to RFC 3394 for aes key unwrap algorithm.
            
    ========================================================================
*/
VOID    AES_GTK_KEY_UNWRAP( 
    IN  UCHAR   *key,
    OUT UCHAR   *plaintext,
    IN  UCHAR   *ciphertext)
{
    UCHAR       A[8],   BIN[16], BOUT[16];
    UCHAR       R1[8],R2[8];
    UCHAR       xor;
    INT         num_blocks = 2;
    INT         j;
    aes_context aesctx;
    
    // Initialize
    // A = C[0]
    memcpy(A, ciphertext, 8);
    // R1 = C1
    memcpy(R1, &ciphertext[8], 8);
    // R2 = C2
    memcpy(R2, &ciphertext[16], 8);

    aes_set_key(&aesctx, key, 128);
    
    for (j = 5; j >= 0; j--)
    {
        xor = num_blocks * j + 2;
        memcpy(BIN, A, 8);
        BIN[7] = A[7] ^ xor;
        memcpy(&BIN[8], R2, 8);
        aes_decrypt(&aesctx, BIN, BOUT);
        memcpy(A, &BOUT[0], 8);
        memcpy(R2, &BOUT[8], 8);
        
        xor = num_blocks * j + 1;
        memcpy(BIN, A, 8);
        BIN[7] = A[7] ^ xor;
        memcpy(&BIN[8], R1, 8);
        aes_decrypt(&aesctx, BIN, BOUT);
        memcpy(A, &BOUT[0], 8);
        memcpy(R1, &BOUT[8], 8);
    }

    // OUTPUT
    memcpy(&plaintext[0], R1, 8);
    memcpy(&plaintext[8], R2, 8);
}
