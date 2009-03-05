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
 *      Module Name: sanity.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#include "rt_config.h"

UCHAR   WPA_OUI[] = {0x00, 0x50, 0xf2, 0x01};

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN MlmeScanReqSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT UCHAR *BssType, 
    OUT CHAR Ssid[], 
    OUT UCHAR *SsidLen, 
    OUT UCHAR *ScanType) 
{
    MLME_SCAN_REQ_STRUCT *Info;

    Info = (MLME_SCAN_REQ_STRUCT *)(Msg);
    *BssType = Info->BssType;
    *SsidLen = Info->SsidLen;
    memcpy(Ssid, Info->Ssid, *SsidLen);
    *ScanType = Info->ScanType;

    if ((*BssType == BSS_INFRA || *BssType == BSS_INDEP || *BssType == BSS_ANY) &&
       (*ScanType == SCAN_ACTIVE || *ScanType == SCAN_PASSIVE)) 
        return TRUE;
    else 
    {
        DBGPRINT(RT_DEBUG_TRACE, "MlmeScanReqSanity fail - wrong BssType or ScanType\n");
        return FALSE;
    }
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN MlmeStartReqSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT CHAR Ssid[], 
    OUT UCHAR *SsidLen) 
{
    MLME_START_REQ_STRUCT *Info;

    Info = (MLME_START_REQ_STRUCT *)(Msg);
    
    if (Info->SsidLen > MAX_LEN_OF_SSID)
    {
        DBGPRINT(RT_DEBUG_TRACE, "MlmeStartReqSanity fail - wrong SSID length\n");
        return FALSE;
    }

    *SsidLen = Info->SsidLen;
    memcpy(Ssid, Info->Ssid, *SsidLen);

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN MlmeAssocReqSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *ApAddr, 
    OUT USHORT *CapabilityInfo, 
    OUT ULONG *Timeout, 
    OUT USHORT *ListenIntv) 
{
    MLME_ASSOC_REQ_STRUCT *Info;

    Info = (MLME_ASSOC_REQ_STRUCT *)Msg;
    *Timeout = Info->Timeout;                             // timeout
    COPY_MAC_ADDR(ApAddr, &Info->Addr);                   // AP address
    *CapabilityInfo = Info->CapabilityInfo;               // capability info
    *ListenIntv = Info->ListenIntv;

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN MlmeAuthReqSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr, 
    OUT ULONG *Timeout, 
    OUT USHORT *Alg) 
{
    MLME_AUTH_REQ_STRUCT *Info;

    Info  = (MLME_AUTH_REQ_STRUCT *)Msg;
    COPY_MAC_ADDR(Addr, &Info->Addr);
    *Timeout = Info->Timeout;
    *Alg = Info->Alg;

    if ((*Alg == Ndis802_11AuthModeShared || *Alg == Ndis802_11AuthModeOpen) && !MAC_ADDR_IS_GROUP(*Addr)) 
    {
        return TRUE;
    } 
    else 
    {
        DBGPRINT(RT_DEBUG_TRACE, "MlmeAuthReqSanity fail - wrong algorithm\n");
        return FALSE;
    }
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN PeerAssocRspSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT USHORT *CapabilityInfo, 
    OUT USHORT *Status, 
    OUT USHORT *Aid, 
    OUT UCHAR Rates[], 
    OUT UCHAR *RatesLen,
    OUT BOOLEAN *ExtendedRateIeExist) 
{
    CHAR          IeType, *Ptr;
    MACFRAME     *Fr = (MACFRAME *)Msg;
    PBEACON_EID_STRUCT  eid_ptr;

    COPY_MAC_ADDR(Addr2, &Fr->Hdr.Addr2);
    Ptr = Fr->Octet;

    memcpy(CapabilityInfo, &Fr->Octet[0], 2);
    memcpy(Status,         &Fr->Octet[2], 2);
    // Mask out unnecessary capability information
    *CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;

    if (*Status == MLME_SUCCESS) 
    {
        memcpy(Aid, &Fr->Octet[4], 2);
        *Aid = (*Aid) & 0x3fff; // AID is low 14-bit

        // -- get supported rates from payload and advance the pointer
        IeType = Fr->Octet[6];
        *RatesLen = Fr->Octet[7];
        if ((IeType != IE_SUPP_RATES) || (*RatesLen > MAX_LEN_OF_SUPPORTED_RATES))
        {
            DBGPRINT(RT_DEBUG_TRACE, "PeerAssocRspSanity fail - wrong SupportedRates IE\n");
            return FALSE;
        } 
        else 
            memcpy(Rates, &Fr->Octet[8], *RatesLen);

        // many AP implement proprietary IEs in non-standard order, we'd better
        // tolerate mis-ordered IEs to get best compatibility
        *ExtendedRateIeExist = FALSE;
        eid_ptr = (PBEACON_EID_STRUCT) &Fr->Octet[8 + (*RatesLen)];

        // get variable fields from payload and advance the pointer
        while (((UCHAR*)eid_ptr + eid_ptr->Len + 1) < ((UCHAR*)Fr + MsgLen))
        {
            switch (eid_ptr->Eid)
            {
                case IE_EXT_SUPP_RATES:
                    *ExtendedRateIeExist = TRUE;
                    if ((*RatesLen + eid_ptr->Len) <= MAX_LEN_OF_SUPPORTED_RATES)
                    {
                        memcpy(&Rates[*RatesLen], eid_ptr->Octet, eid_ptr->Len);
                        *RatesLen = (*RatesLen) + eid_ptr->Len;
                    }
                    else
                    {
                        memcpy(&Rates[*RatesLen], eid_ptr->Octet, MAX_LEN_OF_SUPPORTED_RATES - (*RatesLen));
                        *RatesLen = MAX_LEN_OF_SUPPORTED_RATES;
                    }
                    break;
                default:
                    DBGPRINT(RT_DEBUG_TRACE, "PeerAssocRspSanity - ignore unrecognized EID = %d\n", eid_ptr->Eid);
                    break;
            }

            eid_ptr = (PBEACON_EID_STRUCT)((UCHAR*)eid_ptr + 2 + eid_ptr->Len);
        }

    }

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN PeerDisassocSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT USHORT *Reason) 
{
    MACFRAME *Fr = (MACFRAME *)Msg;

    COPY_MAC_ADDR(Addr2, &Fr->Hdr.Addr2);
    memcpy(Reason, &Fr->Octet[0], 2);

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN PeerDeauthSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT USHORT *Reason) 
{
    MACFRAME *Fr = (MACFRAME *)Msg;

    COPY_MAC_ADDR(Addr2, &Fr->Hdr.Addr2);
    memcpy(Reason, &Fr->Octet[0], 2);

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN PeerAuthSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr, 
    OUT USHORT *Alg, 
    OUT USHORT *Seq, 
    OUT USHORT *Status, 
    CHAR *ChlgText) 
{
    MACFRAME     *Fr = (MACFRAME *)Msg;

    COPY_MAC_ADDR(Addr,    &Fr->Hdr.Addr2);
    memcpy(Alg,    &Fr->Octet[0], 2);
    memcpy(Seq,    &Fr->Octet[2], 2);
    memcpy(Status, &Fr->Octet[4], 2);

    if (*Alg == Ndis802_11AuthModeOpen) 
    {
        if (*Seq == 1 || *Seq == 2) 
        {
            return TRUE;
        } 
        else 
        {
            DBGPRINT(RT_DEBUG_TRACE, "PeerAuthSanity fail - wrong Seg#\n");
            return FALSE;
        }
    } 
    else if (*Alg == Ndis802_11AuthModeShared) 
    {
        if (*Seq == 1 || *Seq == 4) 
        {
            return TRUE;
        } 
        else if (*Seq == 2 || *Seq == 3) 
        {
            memcpy(ChlgText, &Fr->Octet[8], CIPHER_TEXT_LEN);
            return TRUE;
        } 
        else 
        {
            DBGPRINT(RT_DEBUG_TRACE, "PeerAuthSanity fail - wrong Seg#\n");
            return FALSE;
        }
    } 
    else 
    {
        DBGPRINT(RT_DEBUG_TRACE, "PeerAuthSanity fail - wrong algorithm\n");
        return FALSE;
    }
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN PeerProbeReqSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr2,
    OUT CHAR Ssid[], 
    OUT UCHAR *SsidLen) 
//    OUT UCHAR Rates[], 
//    OUT UCHAR *RatesLen) 
{
    UCHAR Idx;
    UCHAR	RateLen;
    CHAR          IeType;
    MACFRAME *Fr = (MACFRAME *)Msg;

    COPY_MAC_ADDR(Addr2, &Fr->Hdr.Addr2);

    if ((Fr->Octet[0] != IE_SSID) || (Fr->Octet[1] > MAX_LEN_OF_SSID)) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "PeerProbeReqSanity fail - wrong SSID IE(Type=%d,Len=%d)\n",Fr->Octet[0],Fr->Octet[1]);
        return FALSE;
    } 
    
    *SsidLen = Fr->Octet[1];
    memcpy(Ssid, &Fr->Octet[2], *SsidLen);

#if 1    
    Idx = *SsidLen + 2;

    // -- get supported rates from payload and advance the pointer
    IeType = Fr->Octet[Idx];
    RateLen = Fr->Octet[Idx + 1];
    if (IeType != IE_SUPP_RATES) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "PeerProbeReqSanity fail - wrong SupportRates IE(Type=%d,Len=%d)\n",Fr->Octet[Idx],Fr->Octet[Idx+1]);
        return FALSE;
    }
    else 
    {
        if ((pAd->PortCfg.AdhocMode == 2) && (RateLen < 8))
            return (FALSE);
    }
#endif
    return TRUE;
}

/* 
    ==========================================================================
    Description:
        MLME message sanity check
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
 */
BOOLEAN PeerBeaconAndProbeRspSanity(
    IN PRTMP_ADAPTER pAd, 
    IN VOID *Msg, 
    IN ULONG MsgLen, 
    OUT MACADDR *Addr2, 
    OUT MACADDR *Bssid, 
    OUT CHAR Ssid[], 
    OUT UCHAR *SsidLen, 
    OUT UCHAR *BssType, 
    OUT USHORT *BeaconPeriod, 
    OUT UCHAR *Channel, 
    OUT LARGE_INTEGER *Timestamp, 
    OUT BOOLEAN *CfExist, 
    OUT CF_PARM *CfParm, 
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
    OUT	PNDIS_802_11_VARIABLE_IEs pVIE) 
{
    CHAR                *Ptr, TimLen;
    MACFRAME            *Fr;
    PBEACON_EID_STRUCT  eid_ptr;
    UCHAR               SubType;
    UCHAR               Sanity;

    // Add for 3 necessary EID field check
    Sanity = 0;

    *ExtendedRateIeExist = FALSE;
    *Erp = 0;

    Fr = (MACFRAME *)Msg;

    // get subtype from header
    SubType = (UCHAR)Fr->Hdr.SubType;

    // get Addr2 and BSSID from header
    COPY_MAC_ADDR(Addr2, &Fr->Hdr.Addr2);
    COPY_MAC_ADDR(Bssid, &Fr->Hdr.Addr3);

    Ptr = Fr->Octet;

    // get timestamp from payload and advance the pointer
    memcpy(Timestamp, Ptr, TIMESTAMP_LEN);
    Ptr += TIMESTAMP_LEN;

    // get beacon interval from payload and advance the pointer
    memcpy(BeaconPeriod, Ptr, 2);
    Ptr += 2;

    // get capability info from payload and advance the pointer
    memcpy(CapabilityInfo, Ptr, 2);
    Ptr += 2;
    if (CAP_IS_ESS_ON(*CapabilityInfo)) 
    {
        *BssType = BSS_INFRA;
    } 
    else 
    {
        *BssType = BSS_INDEP;
    }

    // Mask out unnecessary capability information
    *CapabilityInfo &= SUPPORTED_CAPABILITY_INFO;
    
    eid_ptr = (PBEACON_EID_STRUCT) Ptr;

    // get variable fields from payload and advance the pointer
    while(((UCHAR*)eid_ptr + eid_ptr->Len + 1) < ((UCHAR*)Fr + MsgLen))
    {
        switch(eid_ptr->Eid)
        {
            case IE_SSID:
                // Already has one SSID EID in this beacon, ignore the second one
				if (Sanity & 0x1)
					break;
                if(eid_ptr->Len <= MAX_LEN_OF_SSID)
                {
                    memcpy(Ssid, eid_ptr->Octet, eid_ptr->Len);
		    memset(Ssid + eid_ptr->Len,0,1);
                    *SsidLen = eid_ptr->Len;
                    Sanity |= 0x1;
                    //DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - ESSID=%s Len=%d\n",Ssid,eid_ptr->Len);
                }
                else
                {
                    DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - wrong IE_SSID (len=%d)\n",eid_ptr->Len);
                    return FALSE;
                }
                break;

            case IE_SUPP_RATES:
                if(eid_ptr->Len <= MAX_LEN_OF_SUPPORTED_RATES)
                {
            		int		index;
            		UCHAR	rate, i;
            		PUCHAR	eid_rate;

            		i = 0;
            		eid_rate = eid_ptr->Octet;
            		for (index = 0; index < eid_ptr->Len; index++)
            		{
            			rate = eid_rate[index] & 0x7f; // Mask out basic rate set bit
            			if ((rate == 2) || (rate == 4) || (rate == 11) || (rate == 22) ||
            			    (rate == 12) || (rate == 18) || (rate == 24) || (rate == 36) ||
            			    (rate == 48) || (rate == 72) || (rate == 96) || (rate == 108))
            				Rate[i++] = eid_rate[index];	// Save rate with basic rate set bit if exists
            		}
            		*RateLen = i;
					Sanity |= 0x2;

                    // Copy supported rate from desired AP's beacon. We are trying to match
					// AP's supported and extended rate settings.
					memcpy(SupRate, eid_ptr->Octet, eid_ptr->Len);
					*SupRateLen = eid_ptr->Len;
                }
                else
                {
                    DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - wrong IE_SUPP_RATES (len=%d)\n",eid_ptr->Len);
                    return FALSE;
                }
                break;

            case IE_FH_PARM:
                DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity(IE_FH_PARM) \n");
                break;

            case IE_DS_PARM:
                if(eid_ptr->Len == 1)
                {
                    *Channel = *eid_ptr->Octet;
                    if (ChannelSanity(pAd, *Channel) == 0)
                    {
                        DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - wrong IE_DS_PARM (ch=%d)\n",*Channel);
                        return FALSE;
                    }
                    Sanity |= 0x4;
                }
                else
                {
                    DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - wrong IE_DS_PARM (len=%d)\n",eid_ptr->Len);
                    return FALSE;
                }
                break;

            case IE_CF_PARM:
                if(eid_ptr->Len == 6)
                {
                    *CfExist = TRUE;
                    memcpy(CfParm, eid_ptr->Octet, eid_ptr->Len);
                }
                else
                {
                    DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - wrong IE_CF_PARM\n");
                    return FALSE;
                }
                break;

            case IE_IBSS_PARM:
                if(eid_ptr->Len == 2)
                {
                    memcpy(AtimWin, eid_ptr->Octet, eid_ptr->Len);
                }
                else
                {
                    DBGPRINT(RT_DEBUG_TRACE, "PeerBeaconAndProbeRspSanity - wrong IE_IBSS_PARM\n");
                    return FALSE;
                }
                break;

            case IE_TIM:
                if(INFRA_ON(pAd) && SubType == SUBTYPE_BEACON)
                {
                    GetTimBit((PUCHAR)eid_ptr, pAd->PortCfg.Aid, &TimLen, BcastFlag, DtimCount, DtimPeriod, MessageToMe);
                }
                break;

            // New for WPA
            case IE_WPA:
                // Check the OUI version, filter out non-standard usage
                if (RTMPEqualMemory(eid_ptr->Octet, WPA_OUI, 4))
                {
                    // Copy to pVIE which will report to microsoft bssid list.
                    pVIE->ElementID = eid_ptr->Eid;
                    pVIE->Length = eid_ptr->Len;
                    memcpy(pVIE->data, eid_ptr->Octet, eid_ptr->Len);
                }
                DBGPRINT(RT_DEBUG_INFO, "PeerBeaconAndProbeRspSanity - Receive IE_WPA\n");
                break;

            case IE_EXT_SUPP_RATES:
                // concatenate all extended rates to Rates[] and RateLen
                *ExtendedRateIeExist = TRUE;
                if (eid_ptr->Len <= MAX_LEN_OF_SUPPORTED_RATES)
                {
            		int		index;
            		UCHAR	rate, i;
            		PUCHAR	eid_rate;

            		i = *RateLen;
            		eid_rate = eid_ptr->Octet;
            		for (index = 0; index < eid_ptr->Len; index++)
            		{
            			rate = eid_rate[index] & 0x7f; // Mask out basic rate set bit
            			if ((rate == 2) || (rate == 4) || (rate == 11) || (rate == 22) ||
            			    (rate == 12) || (rate == 18) || (rate == 24) || (rate == 36) ||
            			    (rate == 48) || (rate == 72) || (rate == 96) || (rate == 108))
            				Rate[i++] = eid_rate[index];	// Save rate with basic rate set bit if exists

            		    if (i >= MAX_LEN_OF_SUPPORTED_RATES)
            		        break;
            		}
            		*RateLen = i;
                    // Copy extended rate from desired AP's beacon. We are trying to match
					// AP's supported and extended rate settings.
					memcpy(ExtRate, eid_ptr->Octet, eid_ptr->Len);
					*ExtRateLen = eid_ptr->Len;
                }
                break;

            case IE_ERP:
                if (eid_ptr->Len == 1)
                {
                    *Erp = (UCHAR)eid_ptr->Octet[0];
                }
                break;
                
            default:
                DBGPRINT(RT_DEBUG_INFO, "PeerBeaconAndProbeRspSanity - unrecognized EID = %d\n", eid_ptr->Eid);
                break;
        }
        
        eid_ptr = (PBEACON_EID_STRUCT)((UCHAR*)eid_ptr + 2 + eid_ptr->Len);
    }


    // in 802.11a band, AP may skip this DS IE in their BEACON
    if ((pAd->PortCfg.Channel > 14) && ((Sanity & 0x04)==0))
    {
        *Channel = pAd->PortCfg.Channel;
        Sanity |= 0x04;
    }
    
    if (Sanity != 0x7)
    {
        DBGPRINT(RT_DEBUG_WARN, "PeerBeaconAndProbeRspSanity - missing field, Sanity=0x%02x\n", Sanity);
        return FALSE;
    }
    else
    {
        return TRUE;
    }

}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
BOOLEAN GetTimBit(
    IN CHAR *Ptr, 
    IN USHORT Aid, 
    OUT UCHAR *TimLen, 
    OUT UCHAR *BcastFlag, 
    OUT UCHAR *DtimCount, 
    OUT UCHAR *DtimPeriod,
    OUT UCHAR *MessageToMe) 
{
    UCHAR          BitCntl, N1, N2, MyByte, MyBit;
    CHAR          *IdxPtr;

    IdxPtr = Ptr;

    IdxPtr ++;
    *TimLen = *IdxPtr;

    // get DTIM Count from TIM element
    IdxPtr ++;
    *DtimCount = *IdxPtr;

    // get DTIM Period from TIM element
    IdxPtr++;
    *DtimPeriod = *IdxPtr;

    // get Bitmap Control from TIM element
    IdxPtr++;
    BitCntl = *IdxPtr;

    if ((*DtimCount == 0) && (BitCntl & 0x01)) 
        *BcastFlag = TRUE;
    else 
        *BcastFlag = FALSE;
    
#if 1
    // Parse Partial Virtual Bitmap from TIM element
    N1 = BitCntl & 0xfe;    // N1 is the first bitmap byte#
    N2 = *TimLen - 4 + N1;  // N2 is the last bitmap byte#

    if ((Aid < (N1 << 3)) || (Aid >= ((N2 + 1) << 3)))
        *MessageToMe = FALSE;
    else
    {
        MyByte = (Aid >> 3) - N1;                       // my byte position in the bitmap byte-stream
        MyBit = Aid % 16 - ((MyByte & 0x01)? 8:0);

        IdxPtr += (MyByte + 1);

        //if (*IdxPtr)
        //    DBGPRINT(RT_DEBUG_WARN, ("TIM bitmap = 0x%02x\n", *IdxPtr));
            
        if (*IdxPtr & (0x01 << MyBit))
            *MessageToMe = TRUE;
        else 
            *MessageToMe = FALSE;
    }
#else
    *MessageToMe = FALSE;
#endif

    return TRUE;
}


/*!
 *  \brief Get legacy bit, right now for 11b it is always 0
 *  \param
 *  \return TRUE if the parameters are OK, FALSE otherwise. Always return TRUE
 *  \pre
 *  \post
 */
BOOLEAN GetLegacy(
    IN CHAR *Ptr, 
    OUT UCHAR *Legacy) 
{
    *Legacy = 0;
    return TRUE;
}

UCHAR ChannelSanity(
    IN PRTMP_ADAPTER pAd, 
    IN UCHAR channel)
{
    UCHAR index;

    for (index = 0; index < pAd->PortCfg.ChannelListNum; index ++)
    {
        if (channel == pAd->PortCfg.ChannelList[index])
            return 1;
    }
    return 0;

#if 0    
    switch (pAd->PortCfg.CountryRegion)
    {
        case REGION_FCC:    // 1 - 11
            if ((channel > 0) && (channel < 12))
                return 1;
            break;
            
        case REGION_IC:     // 1 -11
            if ((channel > 0) && (channel < 12))
                return 1;
            break;
            
        case REGION_ETSI:   // 1 - 13
            if ((channel > 0) && (channel < 14))
                return 1;
            break;
            
        case REGION_SPAIN:  // 10 - 11
            if ((channel > 9) && (channel < 12))
                return 1;
            break;
            
        case REGION_FRANCE: // 10 -13
            if ((channel > 9) && (channel < 14))
                return 1;
            break;
            
        case REGION_MKK:    // 14
            if (channel == 14)              
                return 1;
            break;
            
        case REGION_MKK1:   // 1 - 14
            if ((channel > 0) && (channel < 15))
                return 1;
            break;
            
        case REGION_ISRAEL: // 3 - 9
            if ((channel > 2) && (channel < 10))
                return 1;
            break;
            
        default:            // Error
            return 0;           
    }   
    return (0);
#endif    
}
