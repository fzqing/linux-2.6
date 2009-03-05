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
 *      Module Name: rtmp_data.c
 *              
 *      Abstract: Data path subroutines
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      John            25th Feb 03     Modify for rt2560
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW (rt2400)  8th  Dec 04     Promisc mode support
 *      RobinC          10th Dec 04     RFMON Support
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0 
 *      MarkW           17th Dec 04     Monitor mode through iwconfig
 *      MarkW           19th Feb 05     Fixes to incoming byte count
 *      GregorG         29th Mar 05     Big endian fixes
 ***************************************************************************/ 

#include "rt_config.h"

static	UCHAR	SNAP_802_1H[] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00};
static	UCHAR	SNAP_BRIDGE_TUNNEL[] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0xf8};
static	UCHAR	EAPOL[] = {0x88, 0x8e};

static	UCHAR	IPX[] = {0x81, 0x37};
static	UCHAR	APPLE_TALK[] = {0x80, 0xf3};
static  UCHAR   PlcpSignal[12] = { 
	 0, /* RATE_1 */    1, /* RATE_2 */     2, /* RATE_5_5 */   3, /* RATE_11 */    // see BBP spec
	11, /* RATE_6 */   15, /* RATE_9 */    10, /* RATE_12 */   14, /* RATE_18 */    // see IEEE802.11a-1999 p.14
	 9, /* RATE_24 */  13, /* RATE_36 */    8, /* RATE_48 */    12  /* RATE_54 */ }; // see IEEE802.11a-1999 p.14
	 
#define COLLECT_RX_ANTENNA_AVERAGE_RSSI(_pAd, _RxAnt, _rssi)      \
{                                                           \
    USHORT AvgRssi;                                         \
    if (_RxAnt.PrimaryInUsed)                               \
	{                                                       \
	    AvgRssi = _RxAnt.AvgRssi[_RxAnt.PrimaryRxAnt];      \
	    if (AvgRssi > 0)                                    \
	        AvgRssi = AvgRssi - (AvgRssi >> 3) + _rssi;     \
	    else                                                \
	        AvgRssi = _rssi << 3;                           \
	    _RxAnt.AvgRssi[_RxAnt.PrimaryRxAnt] = AvgRssi;      \
	}                                                       \
	else                                                    \
	{                                                       \
            AvgRssi = _RxAnt.AvgRssi[_RxAnt.SecondaryRxAnt];    \
            _RxAnt.RcvPktNumWhenEvaluate++;\
            if ((AvgRssi > 0) && (_RxAnt.FirstPktArrivedWhenEvaluate))	\
                AvgRssi = AvgRssi - (AvgRssi >> 3) + _rssi;     \
            else    \
            {   \
                _RxAnt.FirstPktArrivedWhenEvaluate = TRUE;  \
                AvgRssi = _rssi << 3;   \
                DBGPRINT(RT_DEBUG_TRACE,"Reset RSSI(%d) when first packet is rcved \n",_rssi-_pAd->PortCfg.RssiToDbm);    \
            }   \
        } \
}  \

/*
	========================================================================

	Routine	Description:
		Check Rx descriptor, return NDIS_STATUS_FAILURE if any error dound
		
	Arguments:
		pRxD		Pointer	to the Rx descriptor
		
	Return Value:
		NDIS_STATUS_SUCCESS		No err
		NDIS_STATUS_FAILURE		Error
		
	Note:
	
	========================================================================
*/
inline NDIS_STATUS	RTMPCheckRxDescriptor(
	IN	PRXD_STRUC	pRxD)
{
	// Phy errors
	if (pRxD->PhyErr)
		return(NDIS_STATUS_FAILURE);
	
	// CRC errors
	if (pRxD->Crc)
		return(NDIS_STATUS_FAILURE);

	// Paul 04-03 for OFDM Rx length issue
	if (pRxD->DataByteCnt > 1600)
		return(NDIS_STATUS_FAILURE);

	return(NDIS_STATUS_SUCCESS);
}

#ifdef BIG_ENDIAN
/*
	========================================================================

	Routine	Description:
		Endian conversion of Tx/Rx descriptor .
		
	Arguments:
		pAdapter		Pointer	to our adapter
		pData			Pointer	to Tx/Rx descriptor
		DescriptorType  Direction of the frame
		
	Return Value:
		None
		
	Note:
        Call this function when read or update descriptor
	========================================================================
*/
inline VOID	RTMPDescriptorEndianChange(
	IN	PUCHAR			pData,
	IN	ULONG			DescriptorType)
{
    *((ULONG *)(pData + 40)) = SWAP32(*((ULONG *)(pData + 40)));        // Byte 10
    if(DescriptorType == TYPE_TXD)
        *((ULONG *)(pData + 8)) = SWAP32(*((ULONG *)(pData + 8)));      // Byte 2
    *(ULONG *)pData = SWAP32(*(ULONG *)pData);                          // Byte 0; this must be swapped last
}

/*
	========================================================================

	Routine	Description:
		Endian conversion of all kinds of 802.11 frames .
		
	Arguments:
		pAdapter		Pointer	to our adapter
		pData			Pointer	to the 802.11 frame structure
		Dir				Direction of the frame
		FromRxDoneInt   Caller is from RxDone interrupt

	Return Value:
		None
		
	Note:
	    Call this function when read or update buffer data
	========================================================================
*/
VOID	RTMPFrameEndianChange(
	IN  PRTMP_ADAPTER   pAdapter, 
	IN  PUCHAR          pData, 
	IN  ULONG           Dir,
	IN  BOOLEAN         FromRxDoneInt)
{
    PMACHDR	pFrame;
    PUCHAR  pMacHdr;

    // swab 16 bit fields - Frame Control field
    if(Dir == DIR_READ)
    {
        *(USHORT *)pData = SWAP16(*(USHORT *)pData);
    }

    pFrame = (PMACHDR) pData;
    pMacHdr = (PUCHAR) pFrame;

    // swab 16 bit fields - Duration/ID field
    *(USHORT *)(pMacHdr + 2) = SWAP16(*(USHORT *)(pMacHdr + 2));
    
    // swab 16 bit fields - Sequence Control field
    *(USHORT *)(pMacHdr + 22) = SWAP16(*(USHORT *)(pMacHdr + 22));

    if(pFrame->Type == BTYPE_MGMT)
    {
        switch(pFrame->SubType)
        {
            case SUBTYPE_ASSOC_REQ:
            case SUBTYPE_REASSOC_REQ:
                // swab 16 bit fields - CapabilityInfo field
                pMacHdr += MAC_HDR_LEN;
                *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);

                // swab 16 bit fields - Listen Interval field
                pMacHdr += 2;
                *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);
                break;

            case SUBTYPE_ASSOC_RSP:
            case SUBTYPE_REASSOC_RSP:
                // swab 16 bit fields - CapabilityInfo field
                pMacHdr += MAC_HDR_LEN;
                *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);

                // swab 16 bit fields - Status Code field
                pMacHdr += 2;
                *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);
                
                // swab 16 bit fields - AID field
                pMacHdr += 2;
                *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);
                break;

            case SUBTYPE_AUTH:
                // If from RTMPHandleRxDoneInterrupt routine, it is still a encrypt format.
                // The convertion is delayed to RTMPHandleDecryptionDoneInterrupt.
                if(!FromRxDoneInt &&  pAdapter->NeedSwapToLittleEndian == TRUE)
                {
                    // swab 16 bit fields - Auth Alg No. field
                    pMacHdr += MAC_HDR_LEN;
                    *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);

                    // swab 16 bit fields - Auth Seq No. field
                    pMacHdr += 2;
                    *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);

                    // swab 16 bit fields - Status Code field
                    pMacHdr += 2;
                    *(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);
            	}
            	break;

            case SUBTYPE_BEACON:
            case SUBTYPE_PROBE_RSP:
            	// swab 16 bit fields - BeaconInterval field
            	pMacHdr += MAC_HDR_LEN + TIMESTAMP_LEN;
            	*(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);

            	// swab 16 bit fields - CapabilityInfo field
            	pMacHdr += sizeof(USHORT);
            	*(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);
            	break;

            case SUBTYPE_DEAUTH:
            case SUBTYPE_DISASSOC:
            	// swab 16 bit fields - Reason code field
            	pMacHdr += MAC_HDR_LEN;
            	*(USHORT *)pMacHdr = SWAP16(*(USHORT *)pMacHdr);
            	break;
        }
    }
    else if( pFrame->Type == BTYPE_DATA )
    {
    }
    else if(pFrame->Type == BTYPE_CNTL)
    {
    }
    else
    {
        DBGPRINT(RT_DEBUG_ERROR,"Invalid Frame Type!!!\n");
    }

    // swab 16 bit fields - Frame Control
    if(Dir == DIR_WRITE)
    {
        *(USHORT *)pData = SWAP16(*(USHORT *)pData);
    }
}
#endif

/*
	========================================================================

	Routine	Description:
		Process	RxDone interrupt, running in DPC level

	Arguments:
		pAdapter	Pointer	to our adapter

	Return Value:
		None

	Note:
		This routine has to	maintain Rx	ring read pointer.
	========================================================================
*/
VOID	RTMPHandleRxDoneInterrupt(
	IN	PRTMP_ADAPTER	pAdapter)
{
	PRXD_STRUC		pRxD;
#ifdef BIG_ENDIAN
    PRXD_STRUC      pDestRxD;
    RXD_STRUC       RxD;
#endif
	PHEADER_802_11	pHeader;
	PUCHAR			pData;
	PUCHAR			pDestMac, pSrcMac;
	UCHAR			Count;
	UCHAR			KeyIdx;
	PWPA_KEY		pWpaKey;
	NDIS_STATUS		Status;
	BOOLEAN			bDropFrame;
	ULONG			RegValue;//, Address;
	ULONGLONG		HwDecryptIndex;
        unsigned long           irqflag;

	// Make sure Rx ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->RxRingLock, irqflag);

	// Verify Hardware Decryption pointer with Software Decryption pointer
	RTMP_IO_READ32(pAdapter, SECCSR0, &RegValue);
        HwDecryptIndex = RegValue - pAdapter->RxRing[0].pa_addr;
        do_div(HwDecryptIndex, RING_DESCRIPTOR_SIZE);

#if 0
	Address = pAdapter->RxRing[pAdapter->CurDecryptIndex].pa_addr;
	if (Address != RegValue)
	{
		DBGPRINT(RT_DEBUG_ERROR,"Decrypt pointer not matched SW = 0x%x, HW = 0x%x\n", Address, RegValue);
		DBGPRINT(RT_DEBUG_ERROR,"Sw Decr Ptr = %d, Rx ptr = %d Hw ptr = %d\n",
			pAdapter->CurDecryptIndex, pAdapter->CurRxIndex, HwDecryptIndex);
	}
#endif
	Count = 0;
	do
	{
		// Point to Rx indexed rx ring descriptor
#ifndef BIG_ENDIAN
		pRxD = (PRXD_STRUC) pAdapter->RxRing[pAdapter->CurRxIndex].va_addr;
#else
        pDestRxD = (PRXD_STRUC) pAdapter->RxRing[pAdapter->CurRxIndex].va_addr;
        RxD = *pDestRxD;
        pRxD = &RxD;
        RTMPDescriptorEndianChange((PUCHAR)pRxD, TYPE_RXD);
#endif
		// Initialize drop frame flag
		bDropFrame = FALSE;

		// In case of false alarm or processed at last instance
		if (pRxD->Owner != DESC_OWN_HOST)
		{
			break;
		}

		// Decrypt engine stuck
		if (pRxD->CipherOwner != DESC_OWN_HOST)
		{
			pAdapter->RalinkCounters.RxRingErrCount++;
			break;
		}
		
#ifdef RALINK_ATE
		if(pAdapter->ate.Mode == ATE_RXFRAME)
		{
			bDropFrame = TRUE;
		}
#endif	//#ifdef RALINK_ATE
		
		// Point to Rx ring buffer where stores the real data frame
		pData	= (PUCHAR) (pAdapter->RxRing[pAdapter->CurRxIndex].va_data_addr);
		// Cast to 802.11 header for flags checking
		pHeader	= (PHEADER_802_11) pData;

#ifdef BIG_ENDIAN
        RTMPFrameEndianChange(pAdapter, (PUCHAR)pHeader, DIR_READ, TRUE);
#endif

		// Check for all RxD errors
		Status = RTMPCheckRxDescriptor(pRxD);
		
	    // Apply packet filtering rule based on microsoft requirements.
		if (Status == NDIS_STATUS_SUCCESS)
			Status = RTMPApplyPacketFilter(pAdapter, pRxD, pHeader);
		
		// Add receive counters
		if (Status == NDIS_STATUS_SUCCESS)
		{
            // collect current antenna's average RSSI for software-based RX Antenna diversity
            if (pRxD->U2M || pAdapter->bAcceptPromiscuous == TRUE || ((pHeader->Controlhead.Frame.Subtype == SUBTYPE_BEACON) && (MAC_ADDR_EQUAL(&pAdapter->PortCfg.Bssid, &pHeader->Controlhead.Addr2))))
            {
            	//DBGPRINT(RT_DEBUG_TRACE, "COLLECT_RSSI:(%d)\n", pRxD->BBR1 - pAdapter->PortCfg.RssiToDbm);
            	pAdapter->PortCfg.NumOfAvgRssiSample ++;
                COLLECT_RX_ANTENNA_AVERAGE_RSSI(pAdapter, pAdapter->PortCfg.RxAnt, pRxD->BBR1);

		    }
		}
		else
		{
			// Increase general counters
			pAdapter->Counters.RxErrors++;
		}
		
		// Check for retry bit, if this bit is on, search the cache with SA & sequence
		// as index, if matched, discard this frame, otherwise, update cache
		// This check only apply to unicast data & management frames
		if ((Status == NDIS_STATUS_SUCCESS) && (pRxD->U2M) && (pHeader->Controlhead.Frame.Type != BTYPE_CNTL))
		{
			if (pHeader->Controlhead.Frame.Retry)
			{
				if (RTMPSearchTupleCache(pAdapter, pHeader) == TRUE)
				{
					// Found retry frame in tuple cache, Discard this frame / fragment
					// Increase 802.11 counters
					INC_COUNTER(pAdapter->WlanCounters.FrameDuplicateCount);
					Status = NDIS_STATUS_FAILURE;
				}
				else
					RTMPUpdateTupleCache(pAdapter, pHeader);
			}
			else	// Update Tuple Cache
				RTMPUpdateTupleCache(pAdapter, pHeader);
		}

		//
		// Do RxD release operation	for	all	failure	frames
		//
		pRxD->CipherAlg = CIPHER_NONE;
		if (Status == NDIS_STATUS_SUCCESS && pAdapter->PortCfg.BssType != BSS_MONITOR)
		{
			// pData : Pointer skip	the	first 24 bytes,	802.11 HEADER
			pData += LENGTH_802_11;

			//
			// Start of	main loop to parse receiving frames.
			// The sequence	will be	Type first,	then subtype...
			//
			switch (pHeader->Controlhead.Frame.Type)
			{
				// Frame with data type
				case BTYPE_DATA:
					// Drop not my BSS frame
					if (INFRA_ON(pAdapter))
					{
						// Infrastructure mode, check address 2 for BSSID
						if (!RTMPEqualMemory(&pHeader->Controlhead.Addr2, &pAdapter->PortCfg.Bssid, 6))
						{
							// Receive frame not my BSSID
							bDropFrame = TRUE;
							break;
						}
					}
					else	// Ad-Hoc mode or Not associated
					{
						// Ad-Hoc mode, check address 3 for BSSID
						if (!RTMPEqualMemory(&pHeader->Addr3, &pAdapter->PortCfg.Bssid, 6))
						{
							// Receive frame not my BSSID
							bDropFrame = TRUE;
							break;
						}
						
						// Drop frame from AP while we are in Ad-hoc mode or not associated
						if (pHeader->Controlhead.Frame.FrDs)
						{
							bDropFrame = TRUE;
							break;
						}
					}

					// Drop Null data frame, or CF with NULL data frame
					if ((pHeader->Controlhead.Frame.Subtype == SUBTYPE_NULL_FUNC) ||
						(pHeader->Controlhead.Frame.Subtype == SUBTYPE_CFACK)     ||
						(pHeader->Controlhead.Frame.Subtype == SUBTYPE_CFPOLL)    ||
						(pHeader->Controlhead.Frame.Subtype == SUBTYPE_CFACK_CFPOLL))
					{
						bDropFrame = TRUE;
						break;
					}
				
                                        // Good data frame appears, increase the counters
                                        INC_COUNTER(pAdapter->WlanCounters.ReceivedFragmentCount);
                                        pAdapter->RalinkCounters.ReceivedByteCount +=  pRxD->DataByteCnt;	
	
					// Process Multicast data frame
					if (pRxD->Mcast)
					{
						// Multicast 802.11 Counter
						INC_COUNTER(pAdapter->WlanCounters.MulticastReceivedFrameCount);
						DBGPRINT(RT_DEBUG_INFO,"Receiving multicast frame\n");
					}

					// Init WPA Key to NULL
					pWpaKey = (PWPA_KEY) NULL;
					
					// Find the WPA key, either Group or Pairwise Key
					if ((pAdapter->PortCfg.AuthMode >= Ndis802_11AuthModeWPA) && (pHeader->Controlhead.Frame.Wep))
					{
						INT 	idx;
						
						// First lookup the DA, if it's a group address, use GROUP key
						if (pRxD->Bcast || pRxD->Mcast)
						{
							
							idx = (*(pData + 3) & 0xc0) >> 6;
							if ((pAdapter->PortCfg.GroupKey[idx].KeyLen != 0) && 
								((INFRA_ON(pAdapter) && (NdisEqualMemory(&pHeader->Controlhead.Addr2, &pAdapter->PortCfg.Bssid, 6))) ||
								(ADHOC_ON(pAdapter) && (NdisEqualMemory(&pHeader->Addr3, &pAdapter->PortCfg.Bssid, 6)))))
							{
								pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.GroupKey[idx];
								pWpaKey->Type = GROUP_KEY;
								DBGPRINT(RT_DEBUG_INFO, "Rx Use Group Key %d\n", idx);
							}
						}
						// Try to find the Pairwise Key
						else
						{
							for (idx = 0; idx < PAIRWISE_KEY_NO; idx++)
							{
								if ((NdisEqualMemory(&pHeader->Controlhead.Addr2, pAdapter->PortCfg.PairwiseKey[idx].BssId, 6)) &&
									(pAdapter->PortCfg.PairwiseKey[idx].KeyLen != 0))
								{
									pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.PairwiseKey[idx];
									pWpaKey->Type = PAIRWISE_KEY;
									DBGPRINT(RT_DEBUG_INFO, "Rx Use Pairwise Key\n");
									break;
								}
							}
#if 1							
							// Use default Group Key if there is no Pairwise key present
							if ((pWpaKey == NULL) && (pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0))
							{
								pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId];				
								pWpaKey->Type = GROUP_KEY;
								DBGPRINT(RT_DEBUG_INFO, "Rx Use Group Key\n");
							}
#endif							
						}
					}

					// Process Broadcast & Multicast data frame
					if (pRxD->Bcast || pRxD->Mcast)
					{
						// Drop Mcast / Bcast frame with fragment bit on
						if (pHeader->Controlhead.Frame.MoreFrag)
						{
							DBGPRINT(RT_DEBUG_ERROR,"Receiving multicast frame with fragment bit on\n");
							Status = NDIS_STATUS_FAILURE;
							bDropFrame = TRUE;
							break;
						}	
							
						// Filter out Bcast frame which AP relayed for us
						if (pHeader->Controlhead.Frame.FrDs && RTMPEqualMemory(&pHeader->Addr3, pAdapter->CurrentAddress, 6))
						{
							Status = NDIS_STATUS_FAILURE;
							bDropFrame = TRUE;
							break;
						}	
						
						// WEP encrypted frame
						if (pHeader->Controlhead.Frame.Wep)
						{
							// Check our WEP setting, if no WEP turning on, just drop this frame
							if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption1Enabled)	// WEP
							{
                                KeyIdx = (*(pData + 3) & 0xc0) >> 6;
									memcpy((PUCHAR) &pRxD->Iv, pData, 4);	//Get WEP IV
									memcpy(pRxD->Key, pAdapter->PortCfg.SharedKey[KeyIdx].Key, pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen);									
									if (pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen == 5)
										pRxD->CipherAlg = CIPHER_WEP64;
									else
										pRxD->CipherAlg = CIPHER_WEP128;									
							}
							else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))	// TKIP
							{
								UCHAR	Eiv_Tmp[4];
								
								memcpy((PUCHAR) &pRxD->Iv, pData, 4);	//Get WEP IV
								// Swap EIV byte order, due to ASIC's bug.
								Eiv_Tmp[0] = *(pData + 7);
								Eiv_Tmp[1] = *(pData + 6);
								Eiv_Tmp[2] = *(pData + 5);
								Eiv_Tmp[3] = *(pData + 4);								
								memcpy((PUCHAR) &pRxD->Eiv, Eiv_Tmp, 4);	//Get WEP EIV
								// Copy TA into RxD
								memcpy(pRxD->TA, &pHeader->Controlhead.Addr2, 6);
								KeyIdx = (*(pData + 3) & 0xc0) >> 6;
								memcpy(pRxD->Key, pWpaKey->Key, 16);									
								pRxD->CipherAlg = CIPHER_TKIP;
							}
							else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled) && (pWpaKey != NULL))	// AES
							{
								memcpy((PUCHAR) &pRxD->Iv, pData, 4);			//Get WEP IV
								memcpy((PUCHAR) &pRxD->Eiv, (pData + 4), 4);	//Get WEP EIV
								// Copy TA into RxD
								memcpy(pRxD->TA, &pHeader->Controlhead.Addr2, 6);								
								KeyIdx = (*(pData + 3) & 0xc0) >> 6;
								memcpy(pRxD->Key, pWpaKey->Key, 16);									
								pRxD->CipherAlg = CIPHER_AES;
							}
							else
							{
								// Add error counter
								Status = NDIS_STATUS_FAILURE;
								bDropFrame = TRUE;
								break;
							}
						}
						else	// Not encrypted frames
						{
							pRxD->CipherAlg = CIPHER_NONE;
						}
					}
					
					// Begin process unicast to	me frame
					else if	(pRxD->U2M || pAdapter->bAcceptPromiscuous == TRUE)
					{
						// Send PS-Poll for AP to send next data frame					
						if ((pHeader->Controlhead.Frame.MoreData) && INFRA_ON(pAdapter) && (pAdapter->PortCfg.Psm == PWR_SAVE))
						{
							EnqueuePsPoll(pAdapter);
							DBGPRINT(RT_DEBUG_TRACE, "Sending PS-POLL\n");
						}
						
						//
						// Begin frame processing
						//
						pDestMac = (PUCHAR)	&(pHeader->Controlhead.Addr1); // DA is always	address	1
						if (INFRA_ON(pAdapter))		// For infrastructure, SA is address 3
							pSrcMac	= (PUCHAR) &(pHeader->Addr3);
						else									// For IBSS	mode, SA is	address	2
							pSrcMac	= (PUCHAR) &(pHeader->Controlhead.Addr2);

						// WEP encrypted frame
						if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption1Enabled)	// WEP
						{
							if (pHeader->Controlhead.Frame.Wep)
							{
                                KeyIdx = (*(pData + 3) & 0xc0) >> 6;

									memcpy((PUCHAR) &pRxD->Iv, pData, 4);	//Get WEP IV
									memcpy(pRxD->Key, pAdapter->PortCfg.SharedKey[KeyIdx].Key, pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen);									
									if (pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen == 5)
										pRxD->CipherAlg = CIPHER_WEP64;
									else
										pRxD->CipherAlg = CIPHER_WEP128;									
							}
							else if ((pAdapter->PortCfg.PrivacyFilter == Ndis802_11PrivFilter8021xWEP) &&
								(pHeader->Frag == 0))
							{
								// Check 802.1x frame, if not drop it.
								if (!RTMPEqualMemory(EAPOL, pData + 6, 2))
								{
									// Not 802.1X frames
									// Add error counter
									Status = NDIS_STATUS_FAILURE;
									bDropFrame = TRUE;
									break;
								}
							}
						}
						else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))	// TKIP
						{
							if (pHeader->Controlhead.Frame.Wep)
							{
								UCHAR	Eiv_Tmp[4];
								
								memcpy((PUCHAR) &pRxD->Iv, pData, 4);	//Get WEP IV
								// Swap EIV byte order, due to ASIC's bug.
								Eiv_Tmp[0] = *(pData + 7);
								Eiv_Tmp[1] = *(pData + 6);
								Eiv_Tmp[2] = *(pData + 5);
								Eiv_Tmp[3] = *(pData + 4);								
								memcpy((PUCHAR) &pRxD->Eiv, Eiv_Tmp, 4);	//Get WEP EIV
								KeyIdx = (*(pData + 3) & 0xc0) >> 6;
								// Copy TA into RxD
								memcpy(pRxD->TA, &pHeader->Controlhead.Addr2, 6);
								memcpy(pRxD->Key, pWpaKey->Key, 16);									
								pRxD->CipherAlg = CIPHER_TKIP;
							}
							else if ((pAdapter->PortCfg.PrivacyFilter == Ndis802_11PrivFilter8021xWEP) &&
								(pHeader->Frag == 0))
							{
								// Check 802.1x frame, if not drop it.
								if (!RTMPEqualMemory(EAPOL, pData + 6, 2))
								{
									// Not 802.1X frames
									// Add error counter
									Status = NDIS_STATUS_FAILURE;
									bDropFrame = TRUE;
									break;
								}
							}
						}
						else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled) && (pWpaKey != NULL))	// AES
						{
							if (pHeader->Controlhead.Frame.Wep)
							{
								memcpy((PUCHAR) &pRxD->Iv, pData, 4);			//Get WEP IV
								memcpy((PUCHAR) &pRxD->Eiv, (pData + 4), 4);	//Get WEP EIV
								// Copy TA into RxD
								memcpy(pRxD->TA, &pHeader->Controlhead.Addr2, 6);								
								KeyIdx = (*(pData + 3) & 0xc0) >> 6;
								memcpy(pRxD->Key, pWpaKey->Key, 16);									
								pRxD->CipherAlg = CIPHER_AES;
							}
							else if ((pAdapter->PortCfg.PrivacyFilter == Ndis802_11PrivFilter8021xWEP) &&
								(pHeader->Frag == 0))
							{
								// Check 802.1x frame, if not drop it.
								if (!RTMPEqualMemory(EAPOL, pData + 6, 2))
								{
									// Not 802.1X frames
									// Add error counter
									Status = NDIS_STATUS_FAILURE;
									bDropFrame = TRUE;
									break;
								}
							}
						}
						else if (pHeader->Controlhead.Frame.Wep)
						{
							// Drop WEP frame when PrivacyInvoked is FALSE
							Status = NDIS_STATUS_FAILURE;
							bDropFrame = TRUE;
							break;
						}						
						else	// Not encryptrd frames
						{
							pRxD->CipherAlg = CIPHER_NONE;
						}
					}
					break;

				case BTYPE_MGMT:
					// Always None encrypted
					pRxD->CipherAlg = CIPHER_NONE;
					break;

				case BTYPE_CNTL:
					// Ignore ???
					bDropFrame = TRUE;
					break;

				default	:
					bDropFrame = TRUE;
					break;
			}
		}
		else
			bDropFrame = TRUE;

		// Packet will still do NULL cipher operation and drop afterward
		if (bDropFrame == TRUE)
		{
			pRxD->Drop      = 1;
			pRxD->CipherAlg = CIPHER_NONE;
		}
		else
		{
			pRxD->Drop      = 0;
			pRxD->IvOffset = LENGTH_802_11;
		}
		
		pRxD->CipherOwner = DESC_OWN_NIC;

#ifdef BIG_ENDIAN
        RTMPFrameEndianChange(pAdapter, (PUCHAR)pHeader, DIR_WRITE, TRUE);
        RTMPDescriptorEndianChange((PUCHAR)pRxD, TYPE_RXD);
        *pDestRxD = RxD;
#endif

		pAdapter->CurRxIndex++;
		if (pAdapter->CurRxIndex >= RX_RING_SIZE)
		{
			pAdapter->CurRxIndex = 0;
		}
		Count++;
		
		pAdapter->RalinkCounters.RxCount ++;
		
	}	while (Count < MAX_RX_PROCESS);

	// Kick Decrypt Control Register, based on ASIC's implementation
	// We have to kick decrypt & encrypt every frame.
	RTMP_IO_WRITE32(pAdapter, SECCSR0, 0x1);

	// Make sure to release Rx ring resource
	spin_unlock_irqrestore(&pAdapter->RxRingLock, irqflag);
}

/*
	========================================================================

	Routine	Description:
		Process	TxRing TxDone interrupt, running in	DPC	level

	Arguments:
		Adapter		Pointer	to our adapter

	Return Value:
		None

	Note:

	========================================================================
*/
VOID	RTMPHandleTxRingTxDoneInterrupt(
	IN	PRTMP_ADAPTER	pAdapter)
{
	PTXD_STRUC		pTxD;
#ifdef	BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
#endif
	UCHAR			Count;
        unsigned long           irqflag;
	
	// Make sure Tx ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->TxRingLock, irqflag);
	
	Count = 0;
	do
	{
#ifndef BIG_ENDIAN
		pTxD = (PTXD_STRUC)	(pAdapter->TxRing[pAdapter->NextTxDoneIndex].va_addr);
#else
        pDestTxD = (PTXD_STRUC) (pAdapter->TxRing[pAdapter->NextTxDoneIndex].va_addr);
        TxD = *pDestTxD;
        pTxD = &TxD;
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif

		if ((pTxD->Owner == DESC_OWN_NIC) || (pTxD->CipherOwn == DESC_OWN_NIC) || (pTxD->Valid == FALSE))
		{
			break;
		}

		RTMPHardTransmitDone(
			pAdapter, 
			pTxD, 
			pAdapter->TxRing[pAdapter->NextTxDoneIndex].FrameType);
		
		// It might happend with no Ndis packet to indicate back to upper layer
		// Clear for NdisSendComplete request
		pTxD->Valid = FALSE;
		
		// Increase Total transmit byte counter after real data sent out
		pAdapter->RalinkCounters.TransmittedByteCount +=  pTxD->DataByteCnt;
		
#ifdef BIG_ENDIAN
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
        *pDestTxD = TxD;
#endif
		
		pAdapter->NextTxDoneIndex++;
		if (pAdapter->NextTxDoneIndex >= TX_RING_SIZE)
		{
			pAdapter->NextTxDoneIndex = 0;
		}
	}	while (++Count < MAX_TX_PROCESS);

#ifdef RALINK_ATE
	if((pAdapter->ate.Mode == ATE_TXCONT) || (pAdapter->ate.Mode == ATE_TXCARR) || ((pAdapter->ate.Mode == ATE_TXFRAME)))
	{
    		if (pAdapter->ate.TxDoneCount < pAdapter->ate.TxCount)
		{
			pAdapter->ate.TxDoneCount++;
			DBGPRINT(RT_DEBUG_INFO, "pAdapter->ate.TxDoneCount = %d, Preamble=%d\n", pAdapter->ate.TxDoneCount, pAdapter->PortCfg.TxPreambleInUsed);
			pTxD = (PTXD_STRUC)pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;

			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, FALSE, FALSE, FALSE,
				SHORT_RETRY, IFS_BACKOFF, pAdapter->ate.TxRate, 4,
				pAdapter->ate.TxLength, Rt802_11PreambleLong, 0);

			pAdapter->CurEncryptIndex++;
			if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
			{
				pAdapter->CurEncryptIndex = 0;
			}

			RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
		}
    		else if (pAdapter->ate.Mode == ATE_TXFRAME)
    		{
    		    DBGPRINT(RT_DEBUG_TRACE, "ATE TXFRAME completed!\n");
	    	}
	}
#endif	//#ifdef RALINK_ATE

	// Make sure to release Tx ring resource
	spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
	
	if(pAdapter->bNetDeviceStopQueue)
        {
                DBGPRINT(RT_DEBUG_TRACE, "NetDevice start queue!!!\n\n");
                pAdapter->bNetDeviceStopQueue = FALSE;
                netif_start_queue(pAdapter->net_dev);
        }
	
	// Some Tx ring resource freed, check for pending send frame for hard transmit
	if ((!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_BSS_SCAN_IN_PROGRESS)) && 
		(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF)) &&
		(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RESET_IN_PROGRESS)))
	{
		// RTMPDeQueuePacket(pAdapter, &pAdapter->TxSwQueue0);
		// Call dequeue without selected queue, let the subroutine select the right priority
		// Tx software queue
		RTMPDeQueuePacket(pAdapter);
	}
}

/*
	========================================================================

	Routine	Description:
		Process	Priority ring TxDone interrupt,	running	in DPC level

	Arguments:
		Adapter		Pointer	to our adapter

	Return Value:
		None

	Note:

	========================================================================
*/
VOID	RTMPHandlePrioRingTxDoneInterrupt(
	IN	PRTMP_ADAPTER	pAdapter)
{
	PTXD_STRUC		pTxD;
#ifdef BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
#endif
	UCHAR			Count;
	PMGMT_STRUC		pMgmt;
        unsigned long           irqflag;
	
	// Make sure Prio ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->PrioRingLock, irqflag);	
	
	Count = 0;
	do
	{
#ifndef BIG_ENDIAN
		pTxD = (PTXD_STRUC)	(pAdapter->PrioRing[pAdapter->NextPrioDoneIndex].va_addr);
#else
        pDestTxD = (PTXD_STRUC) (pAdapter->PrioRing[pAdapter->NextPrioDoneIndex].va_addr);
        TxD = *pDestTxD;
        pTxD = &TxD;
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif

		// Check for the descriptor ownership
		if ((pTxD->Owner == DESC_OWN_NIC) || (pTxD->Valid == FALSE))
		{
#ifdef BIG_ENDIAN
            RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
            *pDestTxD = TxD;
#endif
			break;
		}
		
		// No need to put in reply for MLME
		RTMPHardTransmitDone(
			pAdapter, 
			pTxD, 
			pAdapter->PrioRing[pAdapter->NextPrioDoneIndex].FrameType);
		
		// It might happend with no Ndis packet to indicate back to upper layer
		pTxD->Valid = FALSE;		
		
		// Increase Total transmit byte counter after real data sent out
		pAdapter->RalinkCounters.TransmittedByteCount +=  pTxD->DataByteCnt;

#ifdef BIG_ENDIAN
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
        *pDestTxD = TxD;
#endif

		pAdapter->NextPrioDoneIndex++;
		if (pAdapter->NextPrioDoneIndex >= PRIO_RING_SIZE)
		{
			pAdapter->NextPrioDoneIndex = 0;
		}
	}	while (++Count < MAX_TX_PROCESS);

	// Make sure to release Prio ring resource
	spin_unlock_irqrestore(&pAdapter->PrioRingLock, irqflag);	
	
	if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF))
		return;
	
	if (pAdapter->PushMgmtIndex != pAdapter->PopMgmtIndex)
	{
		if (RTMPFreeDescriptorRequest(pAdapter, PRIO_RING, 1) == NDIS_STATUS_SUCCESS)
		{
			pMgmt = (PMGMT_STRUC) &pAdapter->MgmtRing[pAdapter->PopMgmtIndex];
			if (pMgmt->Valid == TRUE)
			{
				MlmeHardTransmit(pAdapter, pMgmt->pBuffer, pMgmt->Length);
				MlmeFreeMemory(pAdapter, pMgmt->pBuffer);
				pMgmt->Valid = FALSE;
				spin_lock(&pAdapter->PrioRingLock);
				pAdapter->PopMgmtIndex++;
				pAdapter->MgmtQueueSize--;
				if (pAdapter->PopMgmtIndex >= MGMT_RING_SIZE)
				{
					pAdapter->PopMgmtIndex = 0;
				}
				spin_unlock(&pAdapter->PrioRingLock);
			}
		}
	}	
}

/*
	========================================================================

	Routine	Description:
		Process	Atim ring TxDone interrupt,	running	in DPC level

	Arguments:
		Adapter		Pointer	to our adapter

	Return Value:
		None

	Note:

	========================================================================
*/
VOID	RTMPHandleAtimRingTxDoneInterrupt(
	IN	PRTMP_ADAPTER	pAdapter)
{
	// PTXD_STRUC		pTxD;
	// UCHAR			Count;
	
	// Make sure Atim ring resource won't be used by other threads
	//spin_lock_irqsave(&pAdapter->AtimRingLock);
	
	// Did not support ATIM, remove everything.
	
	// Make sure to release Atim ring resource
	//spin_unlock_irqrestore(&pAdapter->AtimRingLock);
}

/*
	========================================================================

	Routine	Description:
		Process	Rx ring DecryptionDone interrupt, running in DPC level

	Arguments:
		Adapter		Pointer	to our adapter

	Return Value:
		None

	Note:

	========================================================================
*/
VOID	RTMPHandleDecryptionDoneInterrupt(
	IN	PRTMP_ADAPTER	pAdapter)
{
	PRXD_STRUC		pRxD;
#ifdef BIG_ENDIAN
    PRXD_STRUC      pDestRxD;
    RXD_STRUC       RxD;
#endif
	PHEADER_802_11	pHeader;
	PUCHAR			pData;
	PVOID			pManage;
	PUCHAR			pDestMac, pSrcMac;
	UCHAR			Header802_3[14];
	UCHAR			LLC_Len[2];
	USHORT			PacketSize;
	ULONG			High32TSF, Low32TSF;
	UCHAR			Count;
	PWPA_KEY		pWpaKey;
	NDIS_STATUS 	Status;
	ULONG			RegValue;
	ULONGLONG		HwDecryptIndex;
	ULONG			i;
	struct sk_buff  *skb;
        unsigned long           irqflag;
	
	// Make sure Rx ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->RxRingLock, irqflag);
	
	RTMP_IO_READ32(pAdapter, SECCSR0, &RegValue);
        HwDecryptIndex = RegValue - pAdapter->RxRing[0].pa_addr;
        do_div(HwDecryptIndex, RING_DESCRIPTOR_SIZE);

	Count = 0;
	//do
	while (pAdapter->CurDecryptIndex != HwDecryptIndex)
	{
		// Point to Rx indexed rx ring descriptor
#ifndef BIG_ENDIAN
		pRxD = (PRXD_STRUC) pAdapter->RxRing[pAdapter->CurDecryptIndex].va_addr;
#else
        pDestRxD = (PRXD_STRUC) pAdapter->RxRing[pAdapter->CurDecryptIndex].va_addr;
        RxD = *pDestRxD;
        pRxD = &RxD;
        RTMPDescriptorEndianChange((PUCHAR)pRxD, TYPE_RXD);
#endif
	
		// In case of false alarm or processed at last instance
		if ((pRxD->Owner != DESC_OWN_HOST) || (pRxD->CipherOwner != DESC_OWN_HOST))
			break;
	
		// Point to Rx ring buffer where stores the real data frame
		pData	= (PUCHAR) (pAdapter->RxRing[pAdapter->CurDecryptIndex].va_data_addr);
		// Cast to 802.11 header for flags checking
		pHeader = (PHEADER_802_11) pData;

#ifdef BIG_ENDIAN
        RTMPFrameEndianChange(pAdapter, (PUCHAR)pHeader, DIR_READ, FALSE);
#endif
		// Driver will check the decrypt algorithm and decide whether this ICV is true or not		
		if ((pRxD->IcvError == 1) && (pRxD->CipherAlg == CIPHER_NONE))
				pRxD->IcvError = 0;
		
		// Since we already process header at RxDone interrupt, there is no need to proces
		// header sanity again, the only thing we have to check is icv_err bit
		if (pRxD->IcvError == 1)
		{
   		    DBGPRINT(RT_DEBUG_TRACE,"Rx DecryptDone - ICV error (len %d)\n", pRxD->DataByteCnt);
			pRxD->Drop =1;			// Drop frame with icv error
		}
		// Saved data pointer for management frame which will pass to MLME block
		pManage = (PVOID) pData;

 		if (pAdapter->PortCfg.BssType == BSS_MONITOR)
         	{
 	            struct sk_buff  *skb;
 	
 	            if ((skb = __dev_alloc_skb(2048, GFP_DMA|GFP_ATOMIC)) != NULL)
 	            {
 	                skb->dev = pAdapter->net_dev;
 	                memcpy(skb_put(skb, pRxD->DataByteCnt), pData, pRxD->DataByteCnt);
 	                skb->mac.raw = skb->data;
 	                skb->pkt_type = PACKET_OTHERHOST;
 	                skb->protocol = htons(ETH_P_802_2);
 	                skb->ip_summed = CHECKSUM_NONE;
 	                netif_rx(skb);
 	            }	
                    pRxD->Drop = 1;
         	}

		// pData : Pointer skip the first 24 bytes, 802.11 HEADER
		pData += LENGTH_802_11;

		// The total available payload should exclude 24-byte 802.11 Header
		// If Security is enabled, IV, EIV, ICV size is excluded by ASIC
		PacketSize = (USHORT) pRxD->DataByteCnt - LENGTH_802_11;
	
		// Find the WPA key, either Group or Pairwise Key
		// Although the data has been decrypted by ASIC,
		// driver has to calculate the RxMIC which required the key.
		// The failed case should not happen. If it did, drop it.
		// Init WPA Key
		pWpaKey = (PWPA_KEY) NULL;
        if ((pAdapter->PortCfg.AuthMode >= Ndis802_11AuthModeWPA) && (pHeader->Controlhead.Frame.Wep))
		{
			INT 	idx;
				
			// First lookup the DA, if it's a group address, use GROUP key
			if (pRxD->Bcast || pRxD->Mcast)
			{
				// Get the IV index from RxD descriptor
#ifdef BIG_ENDIAN
				idx = (pRxD->Iv & 0x000000c0) >> 6;
#else
                idx = (pRxD->Iv & 0xc0000000) >> 30;
#endif
				if ((pAdapter->PortCfg.GroupKey[idx].KeyLen != 0) && 
					((INFRA_ON(pAdapter) && (NdisEqualMemory(&pHeader->Controlhead.Addr2, &pAdapter->PortCfg.Bssid, 6))) ||
					(ADHOC_ON(pAdapter) && (NdisEqualMemory(&pHeader->Addr3, &pAdapter->PortCfg.Bssid, 6)))))
				{
					pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.GroupKey[idx];
					pWpaKey->Type = GROUP_KEY;
					DBGPRINT(RT_DEBUG_INFO, "Decrypt Done: Rx Use Group Key %d\n", idx);
				}
			}
			// Try to find the Pairwise Key
			else
			{
				for (idx = 0; idx < PAIRWISE_KEY_NO; idx++)
				{
					if ((NdisEqualMemory(&pHeader->Controlhead.Addr2, pAdapter->PortCfg.PairwiseKey[idx].BssId, 6)) &&
						(pAdapter->PortCfg.PairwiseKey[idx].KeyLen != 0))
					{
						pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.PairwiseKey[idx];
						pWpaKey->Type = PAIRWISE_KEY;
						DBGPRINT(RT_DEBUG_INFO, "Rx Use Pairwise Key\n");
						break;
					}
				}
#if 1				
				// Use default Group Key if there is no Pairwise key present
				if ((pWpaKey == NULL) && (pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0))
				{
					pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId];				
					pWpaKey->Type = GROUP_KEY;
					DBGPRINT(RT_DEBUG_INFO, "Rx Use Group Key\n");
				}
#endif				
			}

			// If there is no WPA key matched, this frame should be dropped
			if (pWpaKey == NULL)
				pRxD->Drop = 1;
		}
			
		//
		// Start of main loop to parse receiving frames.
		// The sequence will be Type first, then subtype...
		//
		if (pRxD->Drop == 0)
		{
			switch (pHeader->Controlhead.Frame.Type)
			{
				// Frame with data type
				case BTYPE_DATA:
					// DA is always address 1
				    // For infrastructure, SA is address 3. For IBSS mode, SA is address 2
					pDestMac = (PUCHAR) &(pHeader->Controlhead.Addr1);
					if (INFRA_ON(pAdapter))
						pSrcMac = (PUCHAR) &(pHeader->Addr3);
					else
						pSrcMac = (PUCHAR) &(pHeader->Controlhead.Addr2);
					
					// Process Broadcast & Multicast data frame
					if (pRxD->Bcast || pRxD->Mcast)
					{							
						// For TKIP frame, calculate the MIC value
						if (pRxD->CipherAlg == CIPHER_TKIP)
						{
							INT	i = 0;

							if (pWpaKey == NULL)
							{
								DBGPRINT(RT_DEBUG_ERROR,"No matched TKIP in decryption done calculate MIC routine!!!\n");
								Status = NDIS_STATUS_FAILURE;
								break;
							}
								
							// Minus MIC length
							PacketSize -= 8;
							if (RTMPTkipCompareMICValue(
								pAdapter,
								pData,
								pDestMac,
								pSrcMac,
								pWpaKey->RxMic,
								PacketSize) == FALSE)
							{
								DBGPRINT(RT_DEBUG_ERROR,"Rx MIC Value error\n");							
								RTMPReportMicError(pAdapter, pWpaKey);
								Status = NDIS_STATUS_FAILURE;
								break;
							}

							// Second, increase RxTsc value for next transmission
							while (++pWpaKey->RxTsc[i] == 0x0)
							{
								i++;
								if (i == 6)
									break;
							}
							// Rx TSC has done one full cycle, since re-key is done by transmitter
							// We did not do anything for Rx path
						}
						
                        // build 802.3 header and decide if remove the 8-byte LLC/SNAP encapsulation
						CONVERT_TO_802_3(Header802_3, pDestMac, pSrcMac, pData, PacketSize);
								
						pAdapter->PortCfg.LedCntl.fRxActivity = TRUE; // for RX ACTIVITY LED

						// For miniportTransferData
						pAdapter->pRxData = pData;
			
						// Acknolwdge upper layer the received frame
#ifdef RTMP_EMBEDDED
                        if ((skb = __dev_alloc_skb(PacketSize + LENGTH_802_3 + 2, GFP_DMA|GFP_ATOMIC)) != NULL)
#else
                        if ((skb = dev_alloc_skb(PacketSize + LENGTH_802_3 + 2)) != NULL)
#endif
                        {
                            skb->dev = pAdapter->net_dev;
                            skb_reserve(skb, 2);    // 16 byte align the IP header
                            memcpy(skb_put(skb, LENGTH_802_3), Header802_3, LENGTH_802_3);
                            memcpy(skb_put(skb, PacketSize), pData, PacketSize);
                            skb->protocol = eth_type_trans(skb, pAdapter->net_dev);
                            netif_rx(skb);
                            pAdapter->net_dev->last_rx = jiffies;
                            pAdapter->stats.rx_packets++;
                        }
		
						DBGPRINT(RT_DEBUG_INFO, "!!! Broadcast Ethenet rx Indicated !!!\n");
					}
						
					// Begin process unicast to me frame
					else if (pRxD->U2M || pAdapter->bAcceptPromiscuous == TRUE)
					{
						// Update Rx data rate first.
						if (pRxD->Ofdm == 1)
						{
							for (i = 4; i < 12; i++)
							{
								if (pRxD->BBR0 == PlcpSignal[i])
									break;
							}
							if (i < 12)
								pAdapter->LastRxRate = i;
						}
						else	// receive CCK encoding
						{
							if (pRxD->BBR0 == 10)
								pAdapter->LastRxRate = 0;
							else if (pRxD->BBR0 == 20)
								pAdapter->LastRxRate = 1;
							else if (pRxD->BBR0 == 55)
								pAdapter->LastRxRate = 2;
							else if (pRxD->BBR0 == 110)
								pAdapter->LastRxRate = 3;
						}
						
						if (pHeader->Frag == 0) 	// First or Only fragment
						{
							// For TKIP frame, calculate the MIC value
							if ((pHeader->Controlhead.Frame.MoreFrag == FALSE) &&
								(pRxD->CipherAlg == CIPHER_TKIP) && 
                                (pHeader->Controlhead.Frame.Wep))
							{
								if (pWpaKey == NULL)
								{
									DBGPRINT(RT_DEBUG_ERROR,"No matched TKIP in decryption done calculate MIC routine!!!\n");
									Status = NDIS_STATUS_FAILURE;
									break;
								}
								// Minus MIC length
								PacketSize -= 8;
								if (RTMPTkipCompareMICValue(
									pAdapter,
									pData,
									pDestMac,
									pSrcMac,
									pWpaKey->RxMic,
									PacketSize) == FALSE)
								{
									DBGPRINT(RT_DEBUG_ERROR,"Rx MIC Value error\n");							
									RTMPReportMicError(pAdapter, pWpaKey);
									Status = NDIS_STATUS_FAILURE;
									break;
								}
							}
								
							pAdapter->FragFrame.Flags &= 0xFFFFFFFE;
								
							// Check for encapsulation other than RFC1042 & Bridge tunnel
							if ((!RTMPEqualMemory(SNAP_802_1H, pData, 6)) && 
							    (!RTMPEqualMemory(SNAP_BRIDGE_TUNNEL, pData, 6)))
							{
								LLC_Len[0] = PacketSize / 256;
								LLC_Len[1] = PacketSize % 256;
								MAKE_802_3_HEADER(Header802_3, pDestMac, pSrcMac, ((PUCHAR) LLC_Len));
							}
							else
							{
							    char *pProto = pData + 6;
							    
								// Remove 802.11 H header & reconstruct 802.3 header
								// pData += (LENGTH_802_1_H - LENGTH_802_3_TYPE);
								// Check for EAPOL frame when driver supplicant enabled
								// TODO: It is not strickly correct. There is no fragment handling. It might damage driver
								// TODO: But for WPAPSK, it's not likely fragment on EAPOL frame will happen
								if (RTMPEqualMemory(EAPOL, pProto, 2) && ((pAdapter->PortCfg.WpaState != SS_NOTUSE))) 
								{
									RTMP_IO_READ32(pAdapter, CSR17, &High32TSF);		// TSF value
									RTMP_IO_READ32(pAdapter, CSR16, &Low32TSF); 		// TSF vlaue
									PacketSize += LENGTH_802_11;
									// Enqueue this frame to MLME engine
									MlmeEnqueueForRecv(
										pAdapter,
										&pAdapter->Mlme.Queue,	
										High32TSF, 
										Low32TSF,
										(UCHAR)pRxD->BBR1, 
										PacketSize, 
										pManage);					
									break;
								}

								if ((RTMPEqualMemory(IPX, pProto, 2) || RTMPEqualMemory(APPLE_TALK, pProto, 2)) && 
								    RTMPEqualMemory(SNAP_802_1H, pData, 6))
								{
								    // preserved the LLC/SNAP filed
									LLC_Len[0] = PacketSize / 256;
									LLC_Len[1] = PacketSize % 256;
									MAKE_802_3_HEADER(Header802_3, pDestMac, pSrcMac, ((PUCHAR) LLC_Len));
								}
								else
								{
								    // remove the LLC/SNAP field
									MAKE_802_3_HEADER(Header802_3, pDestMac, pSrcMac, pProto);
									memcpy(pAdapter->FragFrame.Header_LLC, pData, 8);
									PacketSize -= LENGTH_802_1_H;
									pData += LENGTH_802_1_H;
									pAdapter->FragFrame.Flags |= 0x01;
								}
							}
									
							// One & The only fragment
							if (pHeader->Controlhead.Frame.MoreFrag == FALSE)
							{
								// For miniportTransferData
								pAdapter->pRxData = pData;
								
								pAdapter->PortCfg.LedCntl.fRxActivity = TRUE; // for RX ACTIVITY LED

								// Acknowledge upper layer the received frame
#ifdef RTMP_EMBEDDED
                                if ((skb = __dev_alloc_skb(PacketSize + LENGTH_802_3 + 2, GFP_DMA|GFP_ATOMIC)) != NULL)
#else
                                if ((skb = dev_alloc_skb(PacketSize + LENGTH_802_3 + 2)) != NULL)
#endif
                                {
                                    skb->dev = pAdapter->net_dev;
                                    skb_reserve(skb, 2);    // 16 byte align the IP header
                                    memcpy(skb_put(skb, LENGTH_802_3), Header802_3, LENGTH_802_3);
                                    memcpy(skb_put(skb, PacketSize), pData, PacketSize);
                                    skb->protocol = eth_type_trans(skb, pAdapter->net_dev);
                                    netif_rx(skb);
                                    pAdapter->net_dev->last_rx = jiffies;
                                    pAdapter->stats.rx_packets++;
                                }

								// ZdisZeroMemory(Header802_3, LENGTH_802_3);
								DBGPRINT(RT_DEBUG_INFO, "!!! Frame without Fragment Indicated	!!!\n");

								// Increase general counters
								pAdapter->Counters.GoodReceives++;
	
							}
							// First fragment of fragmented frames
							else
							{
								memcpy(&pAdapter->FragFrame.Buffer[LENGTH_802_3],	pData, PacketSize);
								memcpy(pAdapter->FragFrame.Header802_3, Header802_3, LENGTH_802_3);
								//NdisZeroMemory(Header802_3, LENGTH_802_3);
								pAdapter->FragFrame.RxSize	 = PacketSize;
								pAdapter->FragFrame.Sequence = pHeader->Sequence;
								pAdapter->FragFrame.LastFrag = pHeader->Frag;		// Should be 0
							}
						}
						// Middle & End of fragment burst fragments
						else
						{
							// No LLC-SNAP header in except the first fragment frame
								
							if ((pHeader->Sequence != pAdapter->FragFrame.Sequence) ||
								(pHeader->Frag != (pAdapter->FragFrame.LastFrag + 1)))
							{
								// Fragment is not the same sequence or out of fragment number order
								// Clear Fragment frame contents
								memset(&pAdapter->FragFrame, 0, sizeof(FRAGMENT_FRAME));
								Status = NDIS_STATUS_FAILURE;
								break;
							}	
							else if ((pAdapter->FragFrame.RxSize + PacketSize) > MAX_FRAME_SIZE)
							{
								// Fragment frame is too large, it exeeds the maximum frame size.
								// We have to drop it.
								// Clear Fragment frame contents
								memset(&pAdapter->FragFrame, 0, sizeof(FRAGMENT_FRAME));
								Status = NDIS_STATUS_FAILURE;
								break;
							}
							
                            // concatenate this fragment into the re-assembly buffer
							memcpy(&pAdapter->FragFrame.Buffer[LENGTH_802_3 + pAdapter->FragFrame.RxSize], pData, PacketSize);
							pAdapter->FragFrame.RxSize	+= PacketSize;
							pAdapter->FragFrame.LastFrag = pHeader->Frag;		// Update fragment number
									
							// Last fragment
							if (pHeader->Controlhead.Frame.MoreFrag == FALSE)
							{
								// For TKIP frame, calculate the MIC value
								if ((pRxD->CipherAlg == CIPHER_TKIP) && (pHeader->Controlhead.Frame.Wep))
								{
									if (pWpaKey == NULL)
									{
										DBGPRINT(RT_DEBUG_ERROR,"No matched TKIP in decryption done calculate MIC routine!!!\n");
										Status = NDIS_STATUS_FAILURE;
										break;
									}
									// Minus MIC length
									pAdapter->FragFrame.RxSize -= 8;
											
									if (pAdapter->FragFrame.Flags & 0x00000001)
									{
									    // originally there's an LLC/SNAP field in the first fragment
									    // but been removed in re-assembly buffer. here we have to include
									    // this LLC/SNAP field upon calculating TKIP MIC
										// Copy LLC data to the position in front of real data for MIC calculation
										memcpy(&pAdapter->FragFrame.Buffer[LENGTH_802_3 - LENGTH_802_1_H],
														pAdapter->FragFrame.Header_LLC, 
														LENGTH_802_1_H);
									    pData = (PUCHAR) &pAdapter->FragFrame.Buffer[LENGTH_802_3 - LENGTH_802_1_H];										
									    PacketSize = (USHORT)pAdapter->FragFrame.RxSize + LENGTH_802_1_H;
									    //cketSize = (USHORT)pAdapter->FragFrame.RxSize + 8;
									}
									else
									{
									    pData = (PUCHAR) &pAdapter->FragFrame.Buffer[LENGTH_802_3];
									    PacketSize = (USHORT)pAdapter->FragFrame.RxSize;
									}

									if (RTMPTkipCompareMICValue(
										pAdapter,
    									pData,
										pDestMac,
										pSrcMac,
										pWpaKey->RxMic,
 									    PacketSize) == FALSE)
									{
										DBGPRINT(RT_DEBUG_ERROR,"Rx MIC Value error 2\n");							
										RTMPReportMicError(pAdapter, pWpaKey);
										Status = NDIS_STATUS_FAILURE;
										break;
									}
							
									// TODO:
									// Getting RxTSC from Rx descriptor
								}				

                                // for RX ACTIVITY LED
								pAdapter->PortCfg.LedCntl.fRxActivity = TRUE; 

								// For miniportTransferData
								pAdapter->pRxData = &pAdapter->FragFrame.Buffer[LENGTH_802_3];

								memcpy(pAdapter->FragFrame.Buffer, pAdapter->FragFrame.Header802_3, LENGTH_802_3);
								// Acknowledge upper layer the received frame
#ifdef RTMP_EMBEDDED
                                if ((skb = __dev_alloc_skb(pAdapter->FragFrame.RxSize + LENGTH_802_3 + 2, GFP_DMA|GFP_ATOMIC)) != NULL)
#else
                                if ((skb = dev_alloc_skb(pAdapter->FragFrame.RxSize + LENGTH_802_3 + 2)) != NULL)
#endif
                                {
                                    skb->dev = pAdapter->net_dev;
                                    skb_reserve(skb, 2);    /* 16 byte align the IP header */
                                    memcpy(skb_put(skb, LENGTH_802_3), (PVOID) &pAdapter->FragFrame.Buffer[0], LENGTH_802_3);
                                    memcpy(skb_put(skb, pAdapter->FragFrame.RxSize), (PVOID) &pAdapter->FragFrame.Buffer[LENGTH_802_3], pAdapter->FragFrame.RxSize);
                                    skb->protocol = eth_type_trans(skb, pAdapter->net_dev);
                                    netif_rx(skb);
                                    pAdapter->net_dev->last_rx = jiffies;
                                    pAdapter->stats.rx_packets++;
                                }

								// Increase general counters
								pAdapter->Counters.GoodReceives++;
	
								// Clear Fragment frame contents
								memset(&pAdapter->FragFrame, 0, sizeof(FRAGMENT_FRAME));
								DBGPRINT(RT_DEBUG_INFO, "!!! Frame with Fragment Indicated !!!\n");
							}
						}
					}
					break;
	
				case BTYPE_MGMT:
					// Read required regsiter for MLME engine
					RTMP_IO_READ32(pAdapter, CSR17, &High32TSF);		// TSF value
					RTMP_IO_READ32(pAdapter, CSR16, &Low32TSF); 		// TSF vlaue
				
					// Enqueue this frame to MLME engine
					MlmeEnqueueForRecv(
						pAdapter,
						&pAdapter->Mlme.Queue,	
						High32TSF, 
						Low32TSF,
						(UCHAR)pRxD->BBR1, 
						pRxD->DataByteCnt, 
						pManage);					
					break;
		
				case BTYPE_CNTL:
					// Ignore ???
					break;
	
				default :
					break;
			}
		}
			
		pAdapter->CurDecryptIndex++;
		if (pAdapter->CurDecryptIndex >= RX_RING_SIZE)
		{
			pAdapter->CurDecryptIndex = 0;
		}
		Count++;
			
		pAdapter->RalinkCounters.DecryptCount ++;
			
		// Clear Cipherowner bit & Rx Owner bit for all drop & non-drop frames
		pRxD->CipherOwner = DESC_OWN_HOST;
		pRxD->Owner       = DESC_OWN_NIC;
#ifdef BIG_ENDIAN
                RTMPDescriptorEndianChange((PUCHAR)pRxD, TYPE_RXD);
                *pDestRxD = RxD;
#endif
	}
	//}	while (Count < RX_RING_SIZE);
	//} while (pAdapter->CurDecryptIndex != HwDecryptIndex);
		
	// Make sure to release Rx ring resource
	spin_unlock_irqrestore(&pAdapter->RxRingLock, irqflag);
}

/*
	========================================================================

	Routine	Description:
		Process	Tx ring EncryptionDone interrupt, running in DPC level

	Arguments:
		Adapter		Pointer	to our adapter

	Return Value:
		None

	Note:

	========================================================================
*/
VOID	RTMPHandleEncryptionDoneInterrupt(
	IN	PRTMP_ADAPTER	pAdapter)
{
	PTXD_STRUC		pTxD;
#ifdef BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
#endif
	UCHAR			Count;
	ULONG			RegValue;
	ULONGLONG		HwEncryptIndex;
        unsigned long           irqflag;
	
	// Make sure Prio ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->TxRingLock, irqflag);	
	
	RTMP_IO_READ32(pAdapter, SECCSR1, &RegValue);
        HwEncryptIndex = RegValue - pAdapter->TxRing[0].pa_addr;
        do_div(HwEncryptIndex, RING_DESCRIPTOR_SIZE);

	Count = 0;
	//do
	while (pAdapter->NextEncryptDoneIndex != HwEncryptIndex)
	{
#ifndef BIG_ENDIAN
		pTxD = (PTXD_STRUC)	(pAdapter->TxRing[pAdapter->NextEncryptDoneIndex].va_addr);
#else
        pDestTxD = (PTXD_STRUC) (pAdapter->TxRing[pAdapter->NextEncryptDoneIndex].va_addr);
        TxD = *pDestTxD;
        pTxD = &TxD;
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif

		// Check for the descriptor cipher ownership
		if ((pTxD->CipherOwn == DESC_OWN_NIC) || (pTxD->Owner == DESC_OWN_NIC))
		{
			pAdapter->RalinkCounters.TxRingErrCount++;
#ifdef BIG_ENDIAN
            RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
            *pDestTxD = TxD;
#endif
			break;
		}

		// Alter EIV due to ASIC's bug
		if (pTxD->CipherAlg == CIPHER_TKIP)
		{
			UCHAR	Eiv_Tmp[4];
			PUCHAR	pTmp;

			memcpy(Eiv_Tmp, &pTxD->Eiv, 4);
			pTmp        = (PUCHAR) &pTxD->Eiv;
			*pTmp       = Eiv_Tmp[3];
			*(pTmp + 1) = Eiv_Tmp[2];
			*(pTmp + 2) = Eiv_Tmp[1];
			*(pTmp + 3) = Eiv_Tmp[0];			
		}
		// Sanity Check, CurTxIndex should equal to NextEncryptDoneIndex
		// ASSERT(pAdapter->CurTxIndex == pAdapter->NextEncryptDoneIndex);
		
		pTxD->Valid = TRUE;
		pTxD->Owner = DESC_OWN_NIC;

#ifdef BIG_ENDIAN
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
        *pDestTxD = TxD;
#endif

		pAdapter->NextEncryptDoneIndex++;
		if (pAdapter->NextEncryptDoneIndex >= TX_RING_SIZE)
		{
			pAdapter->NextEncryptDoneIndex = 0;
		}
		pAdapter->CurTxIndex = pAdapter->NextEncryptDoneIndex;
		pAdapter->RalinkCounters.KickTxCount++;

		if (pAdapter->CurTxIndex == pAdapter->CurEncryptIndex)
			break;
	}
	//}	while (++Count < MAX_TX_PROCESS);
	//} while (pAdapter->NextEncryptDoneIndex != HwEncryptIndex);

	// Kick Tx Control Register at the end of all ring buffer preparation
	RTMP_IO_WRITE32(pAdapter, TXCSR0, 0x1);
	
	// Make sure to release Tx ring resource
	spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);	
}

/*
	========================================================================

	Routine	Description:
	Arguments:
		Adapter		Pointer	to our adapter
	========================================================================
*/
void    RTMPHandleTbcnInterrupt(IN PRTMP_ADAPTER pAdapter)
{
	if (ADHOC_ON(pAdapter))
	{
		MACHDR *pBcnHdr = (MACHDR *)pAdapter->BeaconRing.va_data_addr;

		// update BEACON frame's sequence number
		pAdapter->Sequence = ((pAdapter->Sequence) + 1) & (MAX_SEQ_NUMBER);
		pBcnHdr->Seq = pAdapter->Sequence;

#ifdef BIG_ENDIAN
		*(USHORT *)((UCHAR *)pBcnHdr + 22) = SWAP16(*(USHORT *)((UCHAR *)pBcnHdr + 22));
#endif
	}
}

/*
	========================================================================

	Routine	Description:
	Arguments:
		Adapter		Pointer	to our adapter
	========================================================================
*/
void    RTMPHandleTwakeupInterrupt(IN PRTMP_ADAPTER pAdapter)
{
	// DBGPRINT(RT_DEBUG_ERROR, ("Twakeup Expired... !!!\n"));
	pAdapter->PortCfg.Pss = PWR_ACTIVE;
}

/*
	========================================================================

	Routine	Description:
		Process	all transmit ring Tx Done interrupt, running	in DPC level

	Arguments:
		Adapter		Pointer	to our adapter

	Return Value:
		None

	Note:

	========================================================================
*/
VOID	RTMPHardTransmitDone(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PTXD_STRUC		pTxD,
	IN	UCHAR			FrameType)
{

	switch (pTxD->TxResult)
	{
		case SUCCESS_WITHOUT_RETRY:			// Success without any retry
			// Return send complete status
			// DBGPRINT(RT_DEBUG_INFO, "TX Success without retry<<<\n");
			if (pTxD->RTS)
			{
				// Increase 802.11 counters
				INC_COUNTER(pAdapter->WlanCounters.RTSSuccessCount);
				pTxD->RTS = 0;
			}
			
			// Increase general counters
			pAdapter->Counters.GoodTransmits++;
			INC_COUNTER(pAdapter->WlanCounters.TransmittedFragmentCount);

            // update DRS related counters
            if (pTxD->ACK && (FrameType == BTYPE_DATA))
            {
                pAdapter->DrsCounters.OneSecTxOkCount ++;
            }
			break;
			  
		case SUCCESS_WITH_RETRY:			// Success with some retry
			// DBGPRINT(RT_DEBUG_INFO, "TX Success with retry(=%d)<<<\n",pTxD->RetryCount);
			// Increase 802.11 counters
			INC_COUNTER(pAdapter->WlanCounters.RetryCount);
			INC_COUNTER(pAdapter->WlanCounters.ACKFailureCount);
			INC_COUNTER(pAdapter->WlanCounters.TransmittedFragmentCount);
			
			// Increase general counters
			pAdapter->Counters.GoodTransmits++;
			
			if (pTxD->RetryCount > 1)
			{
				// Increase 802.11 counters
				INC_COUNTER(pAdapter->WlanCounters.MultipleRetryCount);
				
				// Increase general counters
				pAdapter->Counters.MoreCollisions++;
			}
			else
			{
				// Increase general counters
				pAdapter->Counters.OneCollision++;
			}
			
			if (pTxD->RTS)
			{
				INC_COUNTER(pAdapter->WlanCounters.RTSSuccessCount);
				pTxD->RTS = 0;
			}

            // update DRS related counters
            if (pTxD->ACK && (FrameType == BTYPE_DATA))
            {
                if (pTxD->TxRate > pAdapter->PortCfg.TxRate)
                {
                    // DRS - must be NULL frame retried @ UpRate; downgrade 
                    //       TxQuality[UpRate] so that not upgrade TX rate
                    pAdapter->DrsCounters.TxQuality[pTxD->TxRate] += 2;
                    if (pAdapter->DrsCounters.TxQuality[pTxD->TxRate] > DRS_TX_QUALITY_WORST_BOUND)
                        pAdapter->DrsCounters.TxQuality[pTxD->TxRate] = DRS_TX_QUALITY_WORST_BOUND;
                }
                else if (pTxD->TxRate == pAdapter->PortCfg.TxRate)
                    pAdapter->DrsCounters.OneSecTxRetryOkCount ++;
            }
			break;

		case FAIL_RETRY_LIMIT:				// Fail on hitting retry count limit
//			DBGPRINT(RT_DEBUG_WARN, ("TX Failed (RETRY LIMIT)<<<\n"));
			// Increase 802.11 counters
			INC_COUNTER(pAdapter->WlanCounters.FailedCount);
			INC_COUNTER(pAdapter->WlanCounters.ACKFailureCount);
			
			// Increase general counters
			pAdapter->Counters.TxErrors++;
			
			if (pTxD->RTS)
			{
				INC_COUNTER(pAdapter->WlanCounters.RTSFailureCount);
				pTxD->RTS = 0;
			}

            // update DRS related counters
            if (pTxD->ACK && (FrameType == BTYPE_DATA))
            {
                if (pTxD->TxRate > pAdapter->PortCfg.TxRate)
                {
                    // DRS - must be NULL frame failed @ UpRate; downgrade 
                    //       TxQuality[UpRate] so that not upgrade TX rate
                    pAdapter->DrsCounters.TxQuality[pTxD->TxRate] = DRS_TX_QUALITY_WORST_BOUND;
                }
                else if (pTxD->TxRate == pAdapter->PortCfg.TxRate)
                {
                    pAdapter->DrsCounters.OneSecTxFailCount ++;
                }
            }
			break;
			
		case FAIL_INVALID:
			// DBGPRINT(RT_DEBUG_WARN, ("TX Failed (INVALID)<<<\n"));
			// Increase general counters
			pAdapter->Counters.TxErrors++;
			
			if (pTxD->RTS)
			{
				INC_COUNTER(pAdapter->WlanCounters.RTSFailureCount);
				pTxD->RTS = 0;
			}
			break;			
			
		case FAIL_OTHER:
		default:
			// DBGPRINT(RT_DEBUG_ERROR, ("TX Failed (other=%d)<<<\n",pTxD->TxResult));
			// Increase 802.11 counters
			INC_COUNTER(pAdapter->WlanCounters.FailedCount);
			INC_COUNTER(pAdapter->WlanCounters.ACKFailureCount);
			
			// Increase general counters
			pAdapter->Counters.TxErrors++;
			
			if (pTxD->RTS)
			{
				INC_COUNTER(pAdapter->WlanCounters.RTSFailureCount);
				pTxD->RTS = 0;
			}
			break;			
	}
}

/*
	========================================================================

	Routine	Description:
		API for MLME to transmit management frame to AP (BSS Mode)
	or station (IBSS Mode)
	
	Arguments:
		pAdapter	Pointer	to our adapter
		Buffer		Pointer to  memory of outgoing frame
		Length		Size of outgoing management frame
		
	Return Value:
		NDIS_STATUS_FAILURE
		NDIS_STATUS_PENDING
		NDIS_STATUS_SUCCESS

	Note:
	
	========================================================================
*/
NDIS_STATUS	MiniportMMRequest(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PVOID			pBuffer,
	IN	ULONG			Length)
{
	PMGMT_STRUC		pMgmt;
	NDIS_STATUS		Status = NDIS_STATUS_SUCCESS;    

	DBGPRINT(RT_DEBUG_INFO, "---> MiniportMMRequest\n");
	// Check management ring free avaliability
	pMgmt = (PMGMT_STRUC) &pAdapter->MgmtRing[pAdapter->PushMgmtIndex];
	
	// This management cell has been occupied
	if (pMgmt->Valid == TRUE)	
	{
		// No Management ring buffer avaliable
		MlmeFreeMemory(pAdapter, pBuffer);
		Status = NDIS_STATUS_FAILURE; 
		DBGPRINT(RT_DEBUG_WARN, "<--- MiniportMMRequest (error:: MgmtRing full)\n");
		pAdapter->RalinkCounters.MgmtRingFullCount++;
		return (Status);
	}
	
	// Insert this request into software managemnet ring
	if (pBuffer)
	{
		pMgmt->pBuffer = pBuffer;		
		pMgmt->Length  = Length;
		pMgmt->Valid   = TRUE;
		pAdapter->PushMgmtIndex++;
		pAdapter->MgmtQueueSize++;
		if (pAdapter->PushMgmtIndex >= MGMT_RING_SIZE)
		{
			pAdapter->PushMgmtIndex = 0;
		}
	}	
	else
	{
		// Null pBuffer, no need to free memory buffer.
		// This should not happen
		DBGPRINT(RT_DEBUG_WARN, "<--- MiniportMMRequest (error:: NULL msg)\n");
		Status = NDIS_STATUS_FAILURE; 
		return (Status);
	}
	
	if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF))
		return (Status);
	
	// Check Free priority queue
	if (RTMPFreeDescriptorRequest(pAdapter, PRIO_RING, 1) == NDIS_STATUS_SUCCESS)
	{
		pMgmt = (PMGMT_STRUC) &pAdapter->MgmtRing[pAdapter->PopMgmtIndex];
		if (pMgmt->Valid == TRUE)
		{
			MlmeHardTransmit(pAdapter, pMgmt->pBuffer, pMgmt->Length);
			MlmeFreeMemory(pAdapter, pMgmt->pBuffer);
			pMgmt->Valid = FALSE;
			spin_lock(&pAdapter->PrioRingLock);
			pAdapter->PopMgmtIndex++;
			pAdapter->MgmtQueueSize--;
			if (pAdapter->PopMgmtIndex >= MGMT_RING_SIZE)
			{
				pAdapter->PopMgmtIndex = 0;
			}
			spin_unlock(&pAdapter->PrioRingLock);
		}
	}
	else
	{
		DBGPRINT(RT_DEBUG_TRACE, "not enough space in PrioRing\n");
	}

	DBGPRINT(RT_DEBUG_INFO, "<--- MiniportMMRequest\n");
	return (Status);
}

/*
	========================================================================

	Routine	Description:
		Copy frame from waiting queue into relative ring buffer and set 
	appropriate ASIC register to kick hardware transmit function
	
	Arguments:
		pAdapter	Pointer	to our adapter
		pBuffer		Pointer to  memory of outgoing frame
		Length		Size of outgoing management frame
		
	Return Value:
		NDIS_STATUS_FAILURE
		NDIS_STATUS_PENDING
		NDIS_STATUS_SUCCESS

	Note:
	
	========================================================================
*/
VOID	MlmeHardTransmit(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PVOID			pBuffer,
	IN	ULONG			Length)
{
	PTXD_STRUC		pTxD;
#ifdef BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
#endif
	PUCHAR			pDest;	
	PHEADER_802_11	pHeader_802_11;
	BOOLEAN         AckRequired, InsertTimestamp;
        unsigned long           irqflag;
	
	DBGPRINT(RT_DEBUG_INFO, "MlmeHardTransmit\n");
	
	// Make sure Prio ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->PrioRingLock, irqflag);
		
	pDest = (PUCHAR) pAdapter->PrioRing[pAdapter->CurPrioIndex].va_data_addr;              
#ifndef BIG_ENDIAN
	pTxD  = (PTXD_STRUC) pAdapter->PrioRing[pAdapter->CurPrioIndex].va_addr;
#else
    pDestTxD  = (PTXD_STRUC) pAdapter->PrioRing[pAdapter->CurPrioIndex].va_addr;
    TxD = *pDestTxD;
    pTxD = &TxD;
    RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif
		
	if (pTxD->Owner == DESC_OWN_NIC)
	{
		// Descriptor owned by NIC. No descriptor avaliable
		// This should not happen since caller guaranteed.
		// Make sure to release Prio ring resource
		spin_unlock_irqrestore(&pAdapter->PrioRingLock, irqflag);
		return;
	}
	if (pTxD->Valid == TRUE)
	{
		// Ndis packet of last round did not cleared.
		// This should not happen since caller guaranteed.
		// Make sure to release Prio ring resource
		spin_unlock_irqrestore(&pAdapter->PrioRingLock, irqflag);
		return;
	}
	if (pBuffer == NULL)
	{
		// The buffer shouldn't be NULL
		spin_unlock_irqrestore(&pAdapter->PrioRingLock, irqflag);
		return;
	}
	
	// outgoing frame always wakeup PHY to prevent frame lost 
	AsicForceWakeup(pAdapter);
	
	pHeader_802_11           = (PHEADER_802_11) pBuffer;
	pHeader_802_11->Controlhead.Frame.PwrMgt = 0; // (pAdapter->PortCfg.Psm == PWR_SAVE);
	InsertTimestamp = FALSE;
	if (pHeader_802_11->Controlhead.Frame.Type == BTYPE_CNTL) // must be PS-POLL
	{
		AckRequired = FALSE;
		pAdapter->PrioRing[pAdapter->CurPrioIndex].FrameType = BTYPE_CNTL;
	}
	else // BTYPE_MGMT or BMGMT_DATA(must be NULL frame)
	{
		pAdapter->PrioRing[pAdapter->CurPrioIndex].FrameType = BTYPE_MGMT;
		pAdapter->Sequence       = ((pAdapter->Sequence) + 1) & (MAX_SEQ_NUMBER);
		pHeader_802_11->Sequence = pAdapter->Sequence;

		if (pHeader_802_11->Controlhead.Addr1.Octet[0] & 0x01) // MULTICAST, BROADCAST
		{
			AckRequired = FALSE;
			pHeader_802_11->Controlhead.Duration = 0;
		}
		else
		{
			AckRequired = TRUE;
			pHeader_802_11->Controlhead.Duration = RTMPCalcDuration(pAdapter, pAdapter->PortCfg.MlmeRate, 14);
			if (pHeader_802_11->Controlhead.Frame.Subtype == SUBTYPE_PROBE_RSP)
			{
				InsertTimestamp = TRUE;
			}
		}
	}
#ifdef BIG_ENDIAN
    RTMPFrameEndianChange(pAdapter, (PUCHAR)pBuffer, DIR_WRITE, FALSE);
#endif
	memcpy(pDest, pBuffer, Length);
   
	// Initialize Priority Descriptor
	// For inter-frame gap, the number is for this frame and next frame
	// For MLME rate, we will fix as 2Mb to match other vendor's implement
#ifdef BIG_ENDIAN
    RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
    *pDestTxD = TxD;
    pTxD = pDestTxD;
#endif
	RTMPWriteTxDescriptor(pTxD, FALSE, CIPHER_NONE, AckRequired, FALSE, InsertTimestamp,
		SHORT_RETRY, IFS_BACKOFF, pAdapter->PortCfg.MlmeRate, 4, Length, pAdapter->PortCfg.TxPreambleInUsed, 0);

	// Increase & maintain Tx Ring Index
	pAdapter->CurPrioIndex++;
	if (pAdapter->CurPrioIndex >= PRIO_RING_SIZE)
	{
		pAdapter->CurPrioIndex = 0;
	}
		
	// Kick priority ring transmit
	RTMP_IO_WRITE32(pAdapter,TXCSR0,0x4);
	
	// Make sure to release Prio ring resource
	spin_unlock_irqrestore(&pAdapter->PrioRingLock, irqflag);
}   
/*
	========================================================================

	Routine	Description:
		This routine is	used to	en-queue outgoing packets when
		there is no	enough shread memory
		
	Arguments:
		pAdapter	Pointer	to our adapter
		pPacket		Pointer to send packet
		
	Return Value:
		None

	Note:
	
	========================================================================
*/
NDIS_STATUS	RTMPSendPacket(
	IN	PRTMP_ADAPTER	pAdapter,
	IN  struct sk_buff  *skb)
{
	PVOID			pVirtualAddress;
	UINT			AllowFragSize;
	UCHAR			NumberOfFrag;
	UCHAR			RTSRequired;
	NDIS_STATUS		Status = NDIS_STATUS_FAILURE;
	UCHAR			PsMode;
	
	struct sk_buff_head	*pTxQueue = NULL;
	ULONG			Priority;
	UCHAR                   AccessCategory;
        unsigned long           irqflag;
	
	DBGPRINT(RT_DEBUG_INFO, "<==== RTMPSendPacket\n");

	// Init priority value
	Priority = 0;
	AccessCategory = 0;
	
    if (skb)
    {
		Priority = skb->priority;
		// 802.11e/d4.4 June, 2003
		if (Priority <=2)
		    AccessCategory = 0;
		else if (Priority == 3)
		    AccessCategory = 1;
		else if (Priority <= 5)
		    AccessCategory = 2;
		else
		    AccessCategory = 3;
		DBGPRINT(RT_DEBUG_INFO, "Priority = %d, AC = %d\n", Priority, AccessCategory);
    }

	// For TKIP, MIC value is treated as payload, it might be fragmented through
	// different MPDUs.
	if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled)
	{
		skb->data_len += 8;
	}

	pVirtualAddress = (PVOID)skb->data;

	// Check for virtual address allocation, it might fail !!!
	if (pVirtualAddress == NULL)
	{
		// Resourece is low, system did not allocation virtual address
		// return NDIS_STATUS_FAILURE directly to upper layer
		return (Status);
	}

	// Store Ethernet MAC address when APClinet mode on
	if ((pAdapter->PortCfg.StaWithEtherBridge.Enable)
	    && ((*((PUCHAR) pVirtualAddress) & 0x01) == 0)
	    && !MAC_ADDR_EQUAL(&pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr, ((PUCHAR) pVirtualAddress) + 6)
	    /*&& !MAC_ADDR_EQUAL(&pAdapter->PermanentAddress, ((PUCHAR) pVirtualAddress) + 6)*/)
	{
        CSR3_STRUC              StaMacReg0;
        CSR4_STRUC              StaMacReg1;

        COPY_MAC_ADDR(&pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr, ((PUCHAR) pVirtualAddress) + 6);

        StaMacReg0.field.Byte0 = pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[0];
        StaMacReg0.field.Byte1 = pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[1];
        StaMacReg0.field.Byte2 = pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[2];
        StaMacReg0.field.Byte3 = pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[3];

        StaMacReg1.field.Byte4 = pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[4];
        StaMacReg1.field.Byte5 = pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[5];

        pAdapter->CurrentAddress[0] = StaMacReg0.field.Byte0;
        pAdapter->CurrentAddress[1] = StaMacReg0.field.Byte1;
        pAdapter->CurrentAddress[2] = StaMacReg0.field.Byte2;
        pAdapter->CurrentAddress[3] = StaMacReg0.field.Byte3;
        pAdapter->CurrentAddress[4] = StaMacReg1.field.Byte4;
        pAdapter->CurrentAddress[5] = StaMacReg1.field.Byte5;
        
        RTMP_IO_WRITE32(pAdapter, CSR3, StaMacReg0.word);
        RTMP_IO_WRITE32(pAdapter, CSR4, StaMacReg1.word);

	    DBGPRINT(RT_DEBUG_TRACE, "StaWithEtherBridge - joining %02x:%02x:%02x:%02x:%02x:%02x ...\n",
	        pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[0],pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[1],pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[2],
	        pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[3],pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[4],pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr.Octet[5]);
	}
	
	//
	// Check for multicast or broadcast (First byte of DA)
	//
	if ((*((PUCHAR) pVirtualAddress) & 0x01) != 0)
	{
		// For multicast & broadcast, there is no fragment allowed
		NumberOfFrag = 1;
	}
	else
	{
		// Check for payload allowed for each fragment
		AllowFragSize = (pAdapter->PortCfg.FragmentThreshold) - LENGTH_802_11 - LENGTH_CRC;

		// Calculate fragments required
		NumberOfFrag = ((skb->data_len - LENGTH_802_3 + LENGTH_802_1_H) / AllowFragSize) + 1;
		// Minus 1 if the size just match to allowable fragment size
		if (((skb->data_len - LENGTH_802_3 + LENGTH_802_1_H) % AllowFragSize) == 0)
		{
			NumberOfFrag--;
		}
	}

	// Check for requirement of RTS 
	if (NumberOfFrag > 1)
	{
		// If multiple fragment required, RTS is required only for the first fragment
		// if the fragment size large than RTS threshold
		RTSRequired = (pAdapter->PortCfg.FragmentThreshold > pAdapter->PortCfg.RtsThreshold) ? 1 : 0;
	}
	else
	{
		RTSRequired = (skb->data_len > pAdapter->PortCfg.RtsThreshold) ? 1 : 0;
	}
	DBGPRINT(RT_DEBUG_INFO, "Number of fragments include RTS :%d\n", NumberOfFrag + RTSRequired);

    // RTS/CTS may also be required in order to protect OFDM frame
    if ((pAdapter->PortCfg.TxRate >= RATE_FIRST_OFDM_RATE) && pAdapter->PortCfg.BGProtectionInUsed)
        RTSRequired = 1;
        
	// Save framnet number to Ndis packet reserved field
	RTMP_SET_PACKET_FRAGMENTS(skb, NumberOfFrag);

	// Save RTS requirement to Ndis packet reserved field
	RTMP_SET_PACKET_RTS(skb, RTSRequired);

	// Make sure SendTxWait queue resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->TxSwQueueLock, irqflag);

	// Select the right priority queue
	// There should be no else statement since it should always fall within 0-3
	if (AccessCategory== 0)
		pTxQueue = &pAdapter->TxSwQueue0;
	else if (AccessCategory== 1)
		pTxQueue = &pAdapter->TxSwQueue1;
	else if (AccessCategory== 2)
		pTxQueue = &pAdapter->TxSwQueue2;
	else if (AccessCategory== 3)
		pTxQueue = &pAdapter->TxSwQueue3;
	
	//
	// For infrastructure mode, enqueue this frame immediately to sendwaitqueue
	// For Ad-hoc mode, check the DA power state, then decide which queue to enqueue
	//
	if (INFRA_ON(pAdapter))
	{
	    // In infrastructure mode, simply enqueue the packet into Tx waiting queue.
	    DBGPRINT(RT_DEBUG_INFO, "Infrastructure -> Enqueue one frame\n");
		
	    // Enqueue Ndis packet to end of Tx wait queue
	    skb_queue_tail(pTxQueue, skb);
	    Status = NDIS_STATUS_SUCCESS;
	}
	else
	{
	    // In IBSS mode, power state of destination should be considered.
	    PsMode = PWR_ACTIVE;		// Faked
	    if (PsMode == PWR_ACTIVE)
	    {
		DBGPRINT(RT_DEBUG_INFO,"Ad-Hoc -> Enqueue one frame\n");

		// Enqueue Ndis packet to end of Tx wait queue
		skb_queue_tail(pTxQueue, skb);
		Status = NDIS_STATUS_SUCCESS;
	    }
	}
	
	spin_unlock_irqrestore(&pAdapter->TxSwQueueLock, irqflag);
	return (Status);
}

/*
	========================================================================

	Routine	Description:
		To do the enqueue operation and extract the first item of waiting 
		list. If a number of available shared memory segments could meet 
		the request of extracted item, the extracted item will be fragmented
		into shared memory segments.
		
	Arguments:
		pAdapter	Pointer	to our adapter
		pQueue		Pointer to Waiting Queue
		
	Return Value:
		None

	Note:
	
	========================================================================
*/
VOID	RTMPDeQueuePacket(
	IN	PRTMP_ADAPTER	pAdapter)
{
	UCHAR			FragmentRequired;
	NDIS_STATUS		Status;
	UCHAR			Count = 0;
	struct sk_buff_head	*pQueue;
	UCHAR           AccessCategory;
	struct sk_buff  *skb;
        unsigned long           irqflag;
	
	// Make sure SendTxWait queue resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->TxSwQueueLock, irqflag);

	while (Count < MAX_TX_PROCESS)
	// Check queue before dequeue
	// while ((pQueue->Head != NULL) && (Count < MAX_TX_PROCESS)) 
	{
		// Reset is in progress, stop immediately
		if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RESET_IN_PROGRESS))
			break;

		pQueue = RTMPCheckTxSwQueue(pAdapter, &AccessCategory);
		if(!pQueue)
                    break;

		// Dequeue the first entry from head of queue list
		skb = skb_dequeue(pQueue);

		if(!skb)
                    break;

		// RTS or CTS-to-self for B/G protection mode has been set already.
		// There is no need to re-do it here. 
		// Total fragment required = number of fragment + RST if required
		FragmentRequired = RTMP_GET_PACKET_FRAGMENTS(skb) + RTMP_GET_PACKET_RTS(skb);
		
		if (RTMPFreeDescriptorRequest(pAdapter, TX_RING, FragmentRequired) == NDIS_STATUS_SUCCESS)
		{
			// Avaliable ring descriptors are enough for this frame
			// Call hard transmit
			Status = RTMPHardEncrypt(pAdapter, skb, FragmentRequired, pAdapter->PortCfg.EnableTxBurst, AccessCategory);

			if (Status == NDIS_STATUS_FAILURE)
			{
                // Packet failed due to various Ndis Packet error
               dev_kfree_skb_irq(skb);
				break;
			}
			else if (Status == NDIS_STATUS_RESOURCES)
			{
				// Not enough free tx ring, it might happen due to free descriptor inquery might be not correct
				// It also might change to NDIS_STATUS_FAILURE to simply drop the frame
				// Put the frame back into head of queue
				skb_queue_head(pQueue, skb);
                break;
			}			
			Count++;
		}	
		else
		{
			skb_queue_head(pQueue, skb);
			pAdapter->PrivateInfo.TxRingFullCnt++;
			DBGPRINT(RT_DEBUG_INFO,"RTMPDequeuePacket --> Not enough free Tx Ring descriptor (CurEncryptIndex=%d,CurTxIndex=%d, NextTxDoneIndex=%d)!!!\n",
			    pAdapter->CurEncryptIndex, pAdapter->CurTxIndex, pAdapter->NextTxDoneIndex);
			break;
		}
	}

	// Release TxSwQueue0 resources
	spin_unlock_irqrestore(&pAdapter->TxSwQueueLock, irqflag);
}    

/*
	========================================================================

	Routine	Description:
		This subroutine will scan through releative ring descriptor to find
		out avaliable free ring descriptor and compare with request size.
		
	Arguments:
		pAdapter	Pointer	to our adapter
		RingType	Selected Ring
		
	Return Value:
		NDIS_STATUS_FAILURE		Not enough free descriptor
		NDIS_STATUS_SUCCESS		Enough free descriptor

	Note:
	
	========================================================================
*/
NDIS_STATUS	RTMPFreeDescriptorRequest(
	IN		PRTMP_ADAPTER	pAdapter,
	IN		UCHAR			RingType,
	IN		UCHAR			NumberRequired)
{
	UCHAR			FreeNumber = 0;
	UINT			Index;
	PTXD_STRUC		pTxD;
#ifdef BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
#endif
        unsigned long           irqflag;
	NDIS_STATUS		Status = NDIS_STATUS_FAILURE;

	switch (RingType)
	{
		case TX_RING:
			spin_lock_irqsave(&pAdapter->TxRingLock, irqflag);
			Index = pAdapter->CurEncryptIndex;
			do
			{
#ifndef BIG_ENDIAN
				pTxD  = (PTXD_STRUC) pAdapter->TxRing[Index].va_addr;
#else
                pDestTxD  = (PTXD_STRUC) pAdapter->TxRing[Index].va_addr;
                TxD = *pDestTxD;
                pTxD = &TxD;
                RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif

				// While Owner bit is NIC, obviously ASIC still need it.
				// If valid bit is TRUE, indicate that TxDone has not process yet
				// We should not use it until TxDone finish cleanup job
				if ((pTxD->Owner == DESC_OWN_HOST) && (pTxD->CipherOwn == DESC_OWN_HOST) && (pTxD->Valid == FALSE))
				{
					// This one is free
					FreeNumber++;
				}
				else
				{
				    DBGPRINT(RT_DEBUG_INFO,"RTMPFreeDescriptorRequest fail - Owner=%d,CipherOwn=%d,Valid=%d\n",pTxD->Owner, pTxD->CipherOwn, pTxD->Valid);
					break;
				}

				Index++;
				if (Index >= TX_RING_SIZE)				// Wrap around issue
				{
					Index = 0;
				}
				
			}	while (FreeNumber < NumberRequired);	// Quit here ! Free number is enough !
			
			if (FreeNumber >= NumberRequired)
			{
				Status = NDIS_STATUS_SUCCESS;
			}
			
			// Make sure to release Tx ring resource
			spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
			break;
			
		case PRIO_RING:
			spin_lock_irqsave(&pAdapter->PrioRingLock, irqflag);
			Index = pAdapter->CurPrioIndex;
			do
			{
#ifndef BIG_ENDIAN
				pTxD  = (PTXD_STRUC) pAdapter->PrioRing[Index].va_addr;
#else
                pDestTxD  = (PTXD_STRUC) pAdapter->PrioRing[Index].va_addr;
                TxD = *pDestTxD;
                pTxD = &TxD;
                RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif
				
				// While Owner bit is NIC, obviously ASIC still need it.
				// If valid bit is TRUE, indicate that TxDone has not process yet
				// We should not use it until TxDone finish cleanup job
				if ((pTxD->Owner == DESC_OWN_HOST) && (pTxD->Valid == FALSE))
				{
					// This one is free
					FreeNumber++;
				}
				else
				{
					break;
				}
					
				Index++;
				if (Index >= PRIO_RING_SIZE)			// Wrap around issue
				{
					Index = 0;
				}
				
			}	while (FreeNumber < NumberRequired);	// Quit here ! Free number is enough !
			
			if (FreeNumber >= NumberRequired)
			{
				Status = NDIS_STATUS_SUCCESS;
			}
			
			// Make sure to release Prio ring resource
			spin_unlock_irqrestore(&pAdapter->PrioRingLock, irqflag);
			break;

		default:
			break;
	}
	
	return (Status);
}

VOID	RTMPSendNullFrame(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PVOID			pBuffer,
	IN	ULONG			Length,
	IN  UCHAR           TxRate)
{
	PUCHAR			pDest;
	PTXD_STRUC		pTxD;
	UCHAR			FrameGap;
#ifdef BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
#endif
        unsigned long           irqflag;
	
	if (pBuffer == NULL)
	{
		return;
	}

	if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RESET_IN_PROGRESS))
	{
		MlmeFreeMemory(pAdapter, pBuffer);
		return;
	}
	
	// WPA 802.1x secured port control
    if (((pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPA) || 
        (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK)) &&
       (pAdapter->PortCfg.PortSecured == WPA_802_1X_PORT_NOT_SECURED)) 
	{
		MlmeFreeMemory(pAdapter, pBuffer);
		return;
	}		
	
	FrameGap = IFS_BACKOFF;		// Default frame gap mode

	// outgoing frame always wakeup PHY to prevent frame lost and 
	// turn off PSM bit to improve performance
	AsicForceWakeup(pAdapter);
#if 0
	if ((pAdapter->TxSwQueue0.Number != 0) || (pAdapter->TxSwQueue1.Number != 0) ||
		(pAdapter->TxSwQueue2.Number != 0) || (pAdapter->TxSwQueue3.Number != 0))
	{
		DBGPRINT(RT_DEBUG_TRACE,("Drop Null frame due to Tx queue not empty!\n"));
	}
	else
#endif	    
	{
		// Make sure Tx ring resource won't be used by other threads
		spin_lock_irqsave(&pAdapter->TxRingLock, irqflag);
	
		// Get the Tx Ring descriptor & Dma Buffer address
		pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;              
#ifndef BIG_ENDIAN
		pTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
#else
        pDestTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
        TxD = *pDestTxD;
        pTxD = &TxD;
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif
		
		if ((pTxD->Owner == DESC_OWN_HOST) && (pTxD->CipherOwn == DESC_OWN_HOST) && (pTxD->Valid == FALSE))
		{
			HEADER_802_11	*pHeader_802_11;
			
			DBGPRINT(RT_DEBUG_TRACE, "SYNC - send NULL Frame @%d Mbps...\n", RateIdToMbps[TxRate]);
#ifdef BIG_ENDIAN
            RTMPFrameEndianChange(pAdapter, (PUCHAR)pBuffer, DIR_WRITE, FALSE);
#endif
			memcpy(pDest, pBuffer, Length);
            pAdapter->TxRing[pAdapter->CurEncryptIndex].FrameType = BTYPE_DATA;

			pHeader_802_11 = (PHEADER_802_11) pDest;
			pHeader_802_11->Controlhead.Frame.PwrMgt = (pAdapter->PortCfg.Psm == PWR_SAVE);
			
#ifdef BIG_ENDIAN
            RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
            *pDestTxD = TxD;
            pTxD = pDestTxD;
#endif

			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, TRUE, FALSE, FALSE, LONG_RETRY, IFS_BACKOFF, 
			    TxRate, 4, Length, pAdapter->PortCfg.TxPreambleInUsed, 0);

			// Increase & maintain Tx Ring Index
			pAdapter->CurEncryptIndex++;
			if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
			{
				pAdapter->CurEncryptIndex = 0;
			}
			
			pAdapter->RalinkCounters.EncryptCount++;

			// Kick Encrypt Control Register at the end of all ring buffer preparation
			RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
			
		}		
		spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
	}
	MlmeFreeMemory(pAdapter, pBuffer);
}

/*
	========================================================================

	Routine	Description:
		Copy frame from waiting queue into relative ring buffer and set 
	appropriate ASIC register to kick hardware encryption before really
	sent out to air.
		
	Arguments:
		pAdapter		Pointer	to our adapter
		PNDIS_PACKET	Pointer to outgoing Ndis frame
		NumberOfFrag	Number of fragment required
		
	Return Value:
		None

	Note:
	
	========================================================================
*/
NDIS_STATUS	RTMPHardEncrypt(
	IN	PRTMP_ADAPTER	pAdapter,
	IN  struct sk_buff  *skb,
	IN	UCHAR			NumberRequired,
	IN	ULONG			EnableTxBurst,
	IN  UCHAR           AccessCategory)
{
	PVOID			pVirtualAddress;
	UINT			NdisBufferLength;
	UINT			BytesCopied;
	UINT			TxSize;
	UINT			FreeFragSize;
	UINT			RemainSize;
	USHORT			Protocol;
	UCHAR			FrameGap;
	HEADER_802_11	Header_802_11;
	PUCHAR			pDest;
	PUCHAR			pSrc;
	PUCHAR			pEncap;
	UCHAR			CipherAlg;
	PTXD_STRUC		pTxD;
	BOOLEAN			StartOfFrame;
	BOOLEAN			EAPOLFrame;
	BOOLEAN			Encapped;
	ULONG			Iv16;
	ULONG			Iv32;
	BOOLEAN			MICFrag;
	PWPA_KEY		pWpaKey = NULL;
    UCHAR           RetryMode = SHORT_RETRY;
    UCHAR           AckRate = RATE_2;
    USHORT          AckDuration = 0;
    USHORT          EncryptionOverhead = 0;
#ifdef BIG_ENDIAN
    PTXD_STRUC      pDestTxD;
    TXD_STRUC       TxD;
    PUCHAR          pOriginDest;
#endif
    unsigned long   irqflag;

	// Make sure Tx ring resource won't be used by other threads
	spin_lock_irqsave(&pAdapter->TxRingLock, irqflag);

    if(skb->data == NULL)
    {
        DBGPRINT(RT_DEBUG_ERROR, "Error, Null skb data buffer!!!\n");

        spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
        return (NDIS_STATUS_FAILURE);
    }

	if (EnableTxBurst == 1)
		FrameGap = IFS_SIFS;
	else
		FrameGap = IFS_BACKOFF;		// Default frame gap mode
	
	// outgoing frame always wakeup PHY to prevent frame lost and 
	// turn off PSM bit to improve performance
	if (pAdapter->PortCfg.Psm == PWR_SAVE)
	{
		MlmeSetPsmBit(pAdapter, PWR_ACTIVE);
	}
	AsicForceWakeup(pAdapter);
	
	// Sequence Number is identical for all fragments belonged to the same frame
	// Sequence is 0 - 4095
	pAdapter->Sequence = ((pAdapter->Sequence) + 1) & (MAX_SEQ_NUMBER);
	
	AckRate = pAdapter->PortCfg.ExpectedACKRate[pAdapter->PortCfg.TxRate];
	AckDuration = RTMPCalcDuration(pAdapter, AckRate, 14);

    pVirtualAddress = skb->data;
    NdisBufferLength = skb->len;
	
	if ((*((PUCHAR) pVirtualAddress) & 0x01) != 0)	// Multicast or Broadcast
	{
		INC_COUNTER(pAdapter->WlanCounters.MulticastTransmittedFrameCount);
	}

	if (NdisBufferLength < 14)
	{
		DBGPRINT(RT_DEBUG_ERROR,"RTMPHardTransmit --> Ndis Packet buffer error !!!\n");
		// Make sure to release Tx ring resource
		spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
		return (NDIS_STATUS_FAILURE);
	}
	
	//
	// Start making 802.11 frame header
	//
	memset(&Header_802_11, 0, sizeof(HEADER_802_11)); // Initialize 802.11 header for each fragment
	if (INFRA_ON(pAdapter))
	{
		// Address 1 - AP, Address 2 - this STA, Address 3 - DA
		memcpy(&Header_802_11.Controlhead.Addr1, &pAdapter->PortCfg.Bssid, ETH_ALEN);
		memcpy(&Header_802_11.Addr3, (PUCHAR) pVirtualAddress, ETH_ALEN);
		Header_802_11.Controlhead.Frame.ToDs = 1;
	}
	else 
	{
		// Address 1 - DA, Address 2 - this STA, Address 3 - BSSID
		memcpy(&Header_802_11.Controlhead.Addr1, (PUCHAR) pVirtualAddress, ETH_ALEN);
		memcpy(&Header_802_11.Addr3, &pAdapter->PortCfg.Bssid, ETH_ALEN);
	}
	memcpy(&Header_802_11.Controlhead.Addr2, pAdapter->CurrentAddress, ETH_ALEN);
	
	Header_802_11.Sequence = pAdapter->Sequence;		// Sequence number
	Header_802_11.Controlhead.Frame.Type = BTYPE_DATA;	// Frame type
	Header_802_11.Controlhead.Frame.PwrMgt = (pAdapter->PortCfg.Psm == PWR_SAVE);

	// Add 802.11x protocol check.
	// For non-WPA network, 802.1x message should not encrypt even
	// the privacy is on.
	if (RTMPEqualMemory(EAPOL, ((PUCHAR) pVirtualAddress) + 12, 2))
	{
		EAPOLFrame = TRUE;
		if (pAdapter->PortCfg.MicErrCnt >= 2)
			pAdapter->PortCfg.MicErrCnt++;
	}
	else
		EAPOLFrame = FALSE;
	
	// WPA 802.1x secured port control
    if (((pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPA) || 
         (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK)) &&
        ((pAdapter->PortCfg.PortSecured == WPA_802_1X_PORT_NOT_SECURED) || (pAdapter->PortCfg.MicErrCnt >= 2)) &&
        (EAPOLFrame == FALSE))
	{
		DBGPRINT(RT_DEBUG_TRACE,"RTMPHardEncrypt --> Drop packet before port secured !!!\n");
		// Make sure to release Tx ring resource
		spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
		return (NDIS_STATUS_FAILURE);
	}		
	
	MICFrag = FALSE;	// Flag to indicate MIC shall spread into two MPDUs
	Encapped = FALSE;
	pEncap = NULL;
	
	pSrc = (PUCHAR) pVirtualAddress;
	Protocol = *(pSrc + 12) * 256 + *(pSrc + 13);
	if (Protocol > 1500)	// CHeck for LLC encaped
	{
		pEncap = SNAP_802_1H;
		Encapped = TRUE;
		if (RTMPEqualMemory(IPX, pSrc + 12, 2) || 
		    RTMPEqualMemory(APPLE_TALK, pSrc + 12, 2))
		{
			pEncap = SNAP_BRIDGE_TUNNEL;
		}
	}

	if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption1Enabled) && 
		(pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0))
		EncryptionOverhead = 8;     // WEP: IV + ICV			
	else if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled)
		EncryptionOverhead = 12;    // TKIP: IV + EIV + ICV, MIC already added to TotalPacketLength
	else if (pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled)
		EncryptionOverhead = 16;    // AES: IV + EIV + Hardware MIC
	else
	    EncryptionOverhead = 0;

	//
	// Make RTS frame if required
	//
	if (RTMP_GET_PACKET_RTS(skb))
	{
		PCONTROL_HEADER		pControlHeader;
		ULONG				NextFragSize;
		
        // RTS-protected frame should use LONG_RETRY (=4), other frames use SHORT_RETRY (=7)
        RetryMode = LONG_RETRY;
        
		pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;              
#ifndef BIG_ENDIAN
		pTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
#else
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
			spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
			return (NDIS_STATUS_RESOURCES);
		}
		if (pTxD->Valid == TRUE)
		{
			// This might happen when HardTransmit process faster than TxDone interrupt.
			// However, Since we did call free descriptor number check before Tx HardTransmit.
			// This case should not happened. We should return to higher caller and requeue this
			// acket until next Tx hardtransmit. Hopefully this situation is corrected then.
			// Ndis packet of last round did not cleared.
			// This should not happen since caller guaranteed.
			// Make sure to release Tx ring resource
			pTxD->Valid = FALSE;
#ifdef BIG_ENDIAN
            RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
            *pDestTxD = TxD;
#endif
				
			spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
			return (NDIS_STATUS_RESOURCES);
		}
		
		pAdapter->TxRing[pAdapter->CurEncryptIndex].FrameType = BTYPE_CNTL;
		pControlHeader = (PCONTROL_HEADER) pDest;
		memset(pControlHeader, 0, sizeof(CONTROL_HEADER));
		pControlHeader->Frame.Type    = BTYPE_CNTL;

		// Calculate duration = 2 SIFS + CTS + Data Frame size
		if (RTMP_GET_PACKET_FRAGMENTS(skb) > 1)
		{
			// If fragment required, size is maximum fragment size
			NextFragSize = pAdapter->PortCfg.FragmentThreshold;
		}
		else
		{
			// Size should be frame with 802.11 header & CRC
			NextFragSize = skb->data_len + LENGTH_802_11 + LENGTH_CRC - LENGTH_802_3;

			if (Encapped)
				NextFragSize += LENGTH_802_1_H;
		}
		pControlHeader->Duration = 2 * (pAdapter->PortCfg.Dsifs)
			+ RTMPCalcDuration(pAdapter, pAdapter->PortCfg.TxRate, NextFragSize + EncryptionOverhead)
			+ AckDuration; 

		// Write Tx descriptor
		// Don't kick tx start until all frames are prepared
		// RTS has to set more fragment bit for fragment burst
		// RTS did not encrypt		
		if (pAdapter->PortCfg.BGProtectionInUsed == 1)
		{
			DBGPRINT(RT_DEBUG_TRACE,"Making CTS-to-self Frame\n");
			pControlHeader->Frame.Subtype = SUBTYPE_CTS;		
			memcpy(&pControlHeader->Addr1, pAdapter->CurrentAddress, ETH_ALEN);

#ifdef BIG_ENDIAN
            RTMPFrameEndianChange(pAdapter, (PUCHAR)pControlHeader, DIR_WRITE, FALSE);
            RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
            *pDestTxD = TxD;
            pTxD = pDestTxD;
#endif


#ifdef	WIFI_TEST			
			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, FALSE, FALSE, FALSE, SHORT_RETRY,
				FrameGap, pAdapter->PortCfg.RtsRate, 4, 10, Rt802_11PreambleShort,
				AccessCategory);
#else
			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, FALSE, FALSE, FALSE, SHORT_RETRY,
				FrameGap, pAdapter->PortCfg.RtsRate, 4, 10, pAdapter->PortCfg.TxPreambleInUsed,
				AccessCategory);
#endif
		}
		else
		{
            DBGPRINT(RT_DEBUG_TRACE,"Making RTS Frame\n");
			pControlHeader->Frame.Subtype = SUBTYPE_RTS;        
		    if (INFRA_ON(pAdapter))
			    memcpy(&pControlHeader->Addr1, &pAdapter->PortCfg.Bssid, ETH_ALEN);
		    else
			    memcpy(&pControlHeader->Addr1, (PUCHAR) pVirtualAddress, ETH_ALEN);
		    memcpy(&pControlHeader->Addr2, pAdapter->CurrentAddress, ETH_ALEN);
#ifdef BIG_ENDIAN
            RTMPFrameEndianChange(pAdapter, (PUCHAR)pControlHeader, DIR_WRITE, FALSE);
            RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
            *pDestTxD = TxD;
            pTxD = pDestTxD;
#endif
			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, TRUE, TRUE, FALSE, SHORT_RETRY,
				FrameGap, pAdapter->PortCfg.RtsRate, 4, sizeof(CONTROL_HEADER),
				pAdapter->PortCfg.TxPreambleInUsed, AccessCategory);
			pTxD->RTS = 1;
		}
		
		FrameGap = IFS_SIFS;		// Init frame gap for coming data after RTS
		NumberRequired--;
		
		// Increase & maintain Tx Ring Index
		pAdapter->CurEncryptIndex++;
		if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
		{
			pAdapter->CurEncryptIndex = 0;
		}
		pAdapter->RalinkCounters.EncryptCount++;		
	}

	// Find the WPA key, either Group or Pairwise Key
	if (pAdapter->PortCfg.AuthMode >= Ndis802_11AuthModeWPA)
	{
		INT 	idx;
			
		pWpaKey = (PWPA_KEY) NULL;
		// First lookup the DA, if it's a group address, use GROUP key
		if (Header_802_11.Controlhead.Addr1.Octet[0] & 0x01)
		{
			if (pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0)
			{
				pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId];
				pWpaKey->Type = GROUP_KEY;
				DBGPRINT(RT_DEBUG_INFO, "Tx Use Group Key\n");
			}
		}
		// Try to find the Pairwise Key
		else
		{
			for (idx = 0; idx < PAIRWISE_KEY_NO; idx++)
			{
				if ((NdisEqualMemory(&Header_802_11.Controlhead.Addr1, pAdapter->PortCfg.PairwiseKey[idx].BssId, 6)) &&
					(pAdapter->PortCfg.PairwiseKey[idx].KeyLen != 0))
				{
					pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.PairwiseKey[idx];
					pWpaKey->Type = PAIRWISE_KEY;
					DBGPRINT(RT_DEBUG_INFO, "Tx Use Pairwise Key\n");
					break;
				}
			}
			// Use default Group Key if there is no Pairwise key present
			if ((pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0) && (pWpaKey == NULL))
			{
				pWpaKey = (PWPA_KEY) &pAdapter->PortCfg.GroupKey[pAdapter->PortCfg.DefaultKeyId];
				pWpaKey->Type = GROUP_KEY;
				DBGPRINT(RT_DEBUG_INFO, "Tx Use Group Key\n");
			}
		}
	}

	// For the purpose to calculate duration for the second last fragment
	RemainSize = skb->data_len - LENGTH_802_3 + LENGTH_CRC;

	StartOfFrame = TRUE;
	// Start Copy Ndis Packet into Ring buffer.
	// For frame required more than one ring buffer (fragment), all ring buffers
	// have to be filled before kicking start tx bit.
	do
	{
		// Get the Tx Ring descriptor & Dma Buffer address
#ifndef BIG_ENDIAN
		pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;              
		pTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
#else
        pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;              
        pOriginDest = pDest;
        pDestTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
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
			spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
			return (NDIS_STATUS_RESOURCES);
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
			spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);
			return (NDIS_STATUS_RESOURCES);
		}
    	pAdapter->TxRing[pAdapter->CurEncryptIndex].FrameType = BTYPE_DATA;

		// Make fragment number & more fragment bit of 802.11 header
		if (StartOfFrame == TRUE)
			Header_802_11.Frag = 0;			// Start of fragment burst / Single Frame
		else
			Header_802_11.Frag++;			// Rest of fragmented frames.
		
		// Maximum allowable payload with one ring buffer, bound by fragment size
		FreeFragSize = pAdapter->PortCfg.FragmentThreshold - LENGTH_CRC;

		//
		// calculate "duration" field of this fragment
		//
		if (NumberRequired > 1)
		{
		    ULONG NextFragSize;
			Header_802_11.Controlhead.Frame.MoreFrag = 1;
			
			if (NumberRequired == 2)
    			NextFragSize = RemainSize - pAdapter->PortCfg.FragmentThreshold + LENGTH_802_11 + LENGTH_802_11 + LENGTH_CRC;
			else
			    NextFragSize = pAdapter->PortCfg.FragmentThreshold;
			
			Header_802_11.Controlhead.Duration = 3 * pAdapter->PortCfg.Dsifs
				+ 2 * AckDuration
				+ RTMPCalcDuration(pAdapter, pAdapter->PortCfg.TxRate, NextFragSize + EncryptionOverhead);
		}
		else // this is the last or only fragment
		{
			Header_802_11.Controlhead.Frame.MoreFrag = 0;
			
			if (Header_802_11.Controlhead.Addr1.Octet[0] & 0x01) // multicast/broadcast
				Header_802_11.Controlhead.Duration = 0;
			else
				Header_802_11.Controlhead.Duration = pAdapter->PortCfg.Dsifs + AckDuration;
		}

		// Check for WEP enable bit and prepare for software WEP
		if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption1Enabled) && (EAPOLFrame == FALSE) &&
			(pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0))
			Header_802_11.Controlhead.Frame.Wep = 1;
		else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))
			Header_802_11.Controlhead.Frame.Wep = 1;
		else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled) && (pWpaKey != NULL))
			Header_802_11.Controlhead.Frame.Wep = 1;
		
		//
		// Copy 802.11 header to Tx ring buffer
		//
		memcpy(pDest, &Header_802_11, sizeof(Header_802_11));
		pDest        += sizeof(Header_802_11);
		FreeFragSize -= sizeof(Header_802_11);

		DBGPRINT(RT_DEBUG_TRACE,"pWpaKey = %s\n", pWpaKey == NULL ? "NULL" : "not NULL");

		if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption1Enabled) && (EAPOLFrame == FALSE) &&
			(pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].KeyLen != 0))
		{
			DBGPRINT(RT_DEBUG_TRACE,"Ndis802_11Encryption1Enabled::DefaultKeyId = %d\n", pAdapter->PortCfg.DefaultKeyId);
                // Prepare IV, IV offset, Key for Hardware encryption
                RTMPInitWepEngine(
                pAdapter,
                pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].Key,
                pAdapter->PortCfg.DefaultKeyId,
                pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].KeyLen,
                (PUCHAR) &pTxD->Iv);

                if (pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].KeyLen == 5)
                    CipherAlg = CIPHER_WEP64;
                else
                    CipherAlg = CIPHER_WEP128;

                // Set Iv offset in TxD
                pTxD->IvOffset = LENGTH_802_11;
                // Copy Encrypt Key to TxD
                memcpy(
					pTxD->Key,
					pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].Key,
					pAdapter->PortCfg.SharedKey[pAdapter->PortCfg.DefaultKeyId].KeyLen);			
		}
		else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))
		{
			INT     i = 0;
			DBGPRINT(RT_DEBUG_TRACE,"Ndis802_11Encryption2Enabled::DefaultKeyId = %d\n", pAdapter->PortCfg.DefaultKeyId);
            // Prepare 8 bytes TKIP encapsulation for MPDU
            {
                TKIP_IV	tkipIv;

                tkipIv.IV16.field.rc0 = *(pWpaKey->TxTsc + 1);
                tkipIv.IV16.field.rc1 = (tkipIv.IV16.field.rc0 | 0x20) & 0x7f;
                tkipIv.IV16.field.rc2 = *pWpaKey->TxTsc;
                tkipIv.IV16.field.ExtIV = 1;  // 0: non-extended IV, 1: an extended IV
                tkipIv.IV16.field.KeyID = pAdapter->PortCfg.DefaultKeyId;
                tkipIv.IV32 = *(PULONG)(pWpaKey->TxTsc + 2);
#if 0	//jett, 2004-1222 ------------------
#if BIG_ENDIAN == TRUE
                pTxD->Iv = (tkipIv.IV16.field.rc0 << 24) | (tkipIv.IV16.field.rc1 << 16) | (tkipIv.IV16.field.rc2 << 8) | (tkipIv.IV16.field.CONTROL.Byte);
#endif

#ifdef RTMP_EMBEDDED
                pTxD->Iv = (tkipIv.IV16.field.CONTROL.Byte << 24) | (tkipIv.IV16.field.rc2 << 16) | (tkipIv.IV16.field.rc1 << 8) | (tkipIv.IV16.field.rc0);
#else
                pTxD->Iv = tkipIv.IV16.word;
#endif
#else	//----------------------------------
#ifdef BIG_ENDIAN
               pTxD->Iv = SWAP32(tkipIv.IV16.word);
#else
                pTxD->Iv = tkipIv.IV16.word;
#endif
#endif	//----------------------------------

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
		}
		else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled) && (pWpaKey != NULL))
		{
			INT		i;
			PUCHAR	pTmp;

			i = 0;
			pTmp = (PUCHAR) &Iv16;
			*pTmp       = pWpaKey->TxTsc[0];
			*(pTmp + 1) = pWpaKey->TxTsc[1];
			*(pTmp + 2) = 0;
			*(pTmp + 3) = (pAdapter->PortCfg.DefaultKeyId << 6) | 0x20;
			
			Iv32 = *(PULONG)(&pWpaKey->TxTsc[2]);
			
			// Increase TxTsc value for next transmission
			while (++pWpaKey->TxTsc[i] == 0x0)
			{
				i++;
				if (i == 6)
					break;
			}
			if (i == 6)
			{
				// TODO: TSC has done one full cycle, do re-keying stuff follow specs
				// Should send a special event microsoft defined to request re-key
			}
			
			memcpy(&pTxD->Iv, &Iv16, 4);            // Copy IV
			memcpy(&pTxD->Eiv, &Iv32, 4);           // Copy EIV
			pTxD->IvOffset = LENGTH_802_11;                 // Set IV offset
			memcpy(pTxD->Key, pWpaKey->Key, 16);    // Copy TKey
			CipherAlg = CIPHER_AES;                         // Set Cipher suite
		}
		else
			CipherAlg = CIPHER_NONE;
		
		//
		// Only the first fragment required LLC-SNAP header !!!
		//
		if ((StartOfFrame == TRUE) && (Encapped == TRUE))
		{
			// For WEP & no encryption required frame, just copy LLC header into buffer,
			// Hardware will do the encryption job.
			// For TKIP, we have to calculate MIC and store it first
			if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))
			{
				// Calculate MSDU MIC Value
				RTMPCalculateMICValue(pAdapter, skb, pEncap, 6, pWpaKey);
			}

				// Copy LLC header
				memcpy(pDest, pEncap, 6);
				pDest += 6;

			// Copy protocol type
			pSrc = (PUCHAR) pVirtualAddress;
			memcpy(pDest, pSrc + 12, 2);
			pDest += 2;
			
			// Exclude 802.3 header size, we will recalculate the size at
			// the end of fragment preparation.
	    	NdisBufferLength -= LENGTH_802_3;
    		pSrc += LENGTH_802_3;
			FreeFragSize -= LENGTH_802_1_H;
		}
		else if ((StartOfFrame == TRUE) && (Encapped == FALSE))
		{
			if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))
			{
				// Calculate MSDU MIC Value
				RTMPCalculateMICValue(pAdapter, skb, pEncap, 0, pWpaKey);
			}
			
   			pSrc = (PUCHAR) pVirtualAddress + LENGTH_802_3;
    		NdisBufferLength -= LENGTH_802_3;
		}
		
		// Start copying payload
		BytesCopied = 0;
		do
		{
		    if (NdisBufferLength >= FreeFragSize)
			{
				// Copy only the free fragment size, and save the pointer
				// of current buffer descriptor for next fragment buffer.
				memcpy(pDest, pSrc, FreeFragSize);
				BytesCopied += FreeFragSize;
				pSrc        += FreeFragSize;
				pDest       += FreeFragSize;
			    NdisBufferLength -= FreeFragSize;
				break;
			}
			else
			{
				// Copy the rest of this buffer descriptor pointed data
				// into ring buffer.
			    memcpy(pDest, pSrc, NdisBufferLength);
			    BytesCopied  += NdisBufferLength;
		    	pDest        += NdisBufferLength;
		    	FreeFragSize -= NdisBufferLength;
			}
		
			// No more buffer descriptor
			// Add MIC value if needed
			if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && 
				(MICFrag == FALSE) &&
				(pWpaKey != NULL))
			{
				INT i;

			    NdisBufferLength = 8;		// Set length to MIC length
				DBGPRINT(RT_DEBUG_INFO, "Calculated TX MIC value =");  
				for (i = 0; i < 8; i++)
				{
					DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PrivateInfo.Tx.MIC[i]);  
				}
				DBGPRINT(RT_DEBUG_INFO, "\n"); 
							
    			if (FreeFragSize >= NdisBufferLength)
				{
					memcpy(pDest, pAdapter->PrivateInfo.Tx.MIC, NdisBufferLength);
						BytesCopied  += NdisBufferLength;
				    pDest		 += NdisBufferLength;
			    	FreeFragSize -= NdisBufferLength;
		    		NdisBufferLength = 0;
					RemainSize   += 8;	// Need to add MIC as payload
				}
				else
				{
					memcpy(pDest, pAdapter->PrivateInfo.Tx.MIC, FreeFragSize);
					BytesCopied  += FreeFragSize;
					pSrc		  = pAdapter->PrivateInfo.Tx.MIC + FreeFragSize;
					pDest		 += FreeFragSize;
				    NdisBufferLength		 -= FreeFragSize;
					MICFrag 	  = TRUE;
					RemainSize   += (8 - FreeFragSize);	// Need to add MIC as payload
				}
			}
		}	while (FALSE);		// End of copying payload
				
		// Real packet size, No 802.1H header for fragments except the first one.
		if ((StartOfFrame == TRUE) && (Encapped == TRUE))
		{
                TxSize = BytesCopied + LENGTH_802_11 + LENGTH_802_1_H;
		}
		else
		{
			TxSize = BytesCopied + LENGTH_802_11;
		}

		RemainSize = RemainSize - BytesCopied;
			
		if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption1Enabled) && (Header_802_11.Controlhead.Frame.Wep == 1))
		{
			// IV + ICV which ASIC added after encryption done
			TxSize += 8;
		}
		else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption2Enabled) && (pWpaKey != NULL))
		{
			// IV + EIV + ICV which ASIC added after encryption done
			TxSize += 12;
		}
		else if ((pAdapter->PortCfg.WepStatus == Ndis802_11Encryption3Enabled) && (pWpaKey != NULL))
		{
			// IV + EIV + HW MIC
			TxSize += 16;
		}
				
		// Prepare Tx descriptors before kicking tx.
		// The BBP register index in Tx descriptor has to be configured too.
#ifdef BIG_ENDIAN
        RTMPFrameEndianChange(pAdapter, pOriginDest, DIR_WRITE, FALSE);
        RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
        *pDestTxD = TxD;
        pTxD = pDestTxD;
#endif
		if (Header_802_11.Controlhead.Addr1.Octet[0] & 0x01)
		{
			// Multicast, retry bit is off
			RTMPWriteTxDescriptor(pTxD, TRUE, CipherAlg, FALSE, FALSE, FALSE, RetryMode, FrameGap, 
                pAdapter->PortCfg.TxRate, 4, TxSize, pAdapter->PortCfg.TxPreambleInUsed, AccessCategory);
		}
		else
		{
			RTMPWriteTxDescriptor(pTxD, TRUE, CipherAlg, TRUE, FALSE, FALSE, RetryMode, FrameGap, 
			    pAdapter->PortCfg.TxRate, 4, TxSize, pAdapter->PortCfg.TxPreambleInUsed, AccessCategory);
		}

		// Set frame gap for the rest of fragment burst.
		// It won't matter if there is only one fragment (single fragment frame).
		StartOfFrame = FALSE;
		FrameGap     = IFS_SIFS;
		NumberRequired--;
		
		// Increase & maintain Tx Ring Index
		pAdapter->CurEncryptIndex++;
		if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
		{
			pAdapter->CurEncryptIndex = 0;
		}
		
		pAdapter->RalinkCounters.EncryptCount++;
		
	}	while (NumberRequired > 0);

	
	// Kick Encrypt Control Register at the end of all ring buffer preparation
	RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
	
    // Acknowledge protocol send complete of pending packet.
	dev_kfree_skb_irq(skb);

	// Make sure to release Tx ring resource
	spin_unlock_irqrestore(&pAdapter->TxRingLock, irqflag);

	return (NDIS_STATUS_SUCCESS);
}

/*
	========================================================================

	Routine	Description:
		Calculates the duration which is required to transmit out frames 
	with given size and specified rate.
		
	Arguments:
		pAdapter		Pointer	to our adapter
		Rate			Transmit rate
		Size			Frame size in units of byte
		
	Return Value:
		Duration number in units of usec

	Note:
	
	========================================================================
*/
USHORT	RTMPCalcDuration(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	UCHAR			Rate,
	IN	ULONG			Size)
{
	ULONG	Duration = 0;

	if (Rate < RATE_FIRST_OFDM_RATE) // CCK
	{
	    if ((Rate > RATE_1) && (pAdapter->PortCfg.TxPreambleInUsed == Rt802_11PreambleShort))
    		Duration = 96;  // 72+24 preamble+plcp
  		else
            Duration = 192; // 144+48 preamble+plcp
    		
		Duration += (USHORT)((Size << 4) / RateIdTo500Kbps[Rate]);
		if ((Size << 4) % RateIdTo500Kbps[Rate])
			Duration ++;
	}
	else // OFDM rates
	{
		Duration = 20 + 6;      // 16+4 preamble+plcp + Signal Extension
		Duration += 4 * (USHORT)((11 + Size * 4) / RateIdTo500Kbps[Rate]);
		if ((11 + Size * 4) % RateIdTo500Kbps[Rate])
			Duration += 4;
	}
	
	return (USHORT)Duration;
	
}

/*
	========================================================================
	
	Routine	Description:
		Calculates the duration which is required to transmit out frames 
	with given size and specified rate.
		
	Arguments:
		pTxD		Pointer to transmit descriptor
		Ack			Setting for Ack requirement bit
		Fragment	Setting for Fragment bit
		RetryMode	Setting for retry mode
		Ifs			Setting for IFS gap
		Rate		Setting for transmit rate
		Service		Setting for service
		Length		Frame length
		TxPreamble  Short or Long preamble when using CCK rates
		AccessCategory - 0-3, according to 802.11e/d4.4 June/2003
		
	Return Value:
		None
		
	========================================================================
*/
VOID	RTMPWriteTxDescriptor(
	IN	PTXD_STRUC	pSourceTxD,
	IN	BOOLEAN		DoEncrypt,
	IN	UCHAR		CipherAlg,
	IN	BOOLEAN		Ack,
	IN	BOOLEAN		Fragment,
	IN  BOOLEAN     InsTimestamp,
	IN	UCHAR		RetryMode,
	IN	UCHAR		Ifs,
	IN	UINT		Rate,
	IN	UCHAR		Service,
	IN	ULONG		Length,
	IN  USHORT      TxPreamble,
	IN  UCHAR       AccessCategory)
{
	UINT	Residual;
	PTXD_STRUC	pTxD;
#ifndef BIG_ENDIAN
	pTxD = pSourceTxD;
#else
	TXD_STRUC	TxD;

	TxD = *pSourceTxD;
	pTxD = &TxD;
	RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
#endif

	pTxD->MoreFrag    = Fragment;
	pTxD->ACK         = Ack;
	pTxD->Timestamp   = InsTimestamp;
	pTxD->RetryMd     = RetryMode;
	pTxD->IFS         = Ifs;
	pTxD->DataByteCnt = Length;
	pTxD->TxRate      = Rate;
	switch (AccessCategory) // 802.11e/d4.4 June/2003
	{
	    case 3: // TC3, <AIFSN, CwMin, CwMax> = <1, aCwMin/4, aCwMin/2>
        	pTxD->CWmin       = CW_MIN_IN_BITS - 2;
        	pTxD->CWmax       = CW_MIN_IN_BITS - 1;
        	pTxD->Aifs        = 1;
        	break;
        case 2: // TC2, <AIFSN, CwMin, CwMax> = <1, aCwMin/2, aCwMin>
        	pTxD->CWmin       = CW_MIN_IN_BITS - 1;
        	pTxD->CWmax       = CW_MIN_IN_BITS;
        	pTxD->Aifs        = 1;
        	break;
        case 1: // TC1, <AIFSN, CwMin, CwMax> = <1, aCwMin, aCwMax>
        	pTxD->CWmin       = CW_MIN_IN_BITS;
        	pTxD->CWmax       = CW_MAX_IN_BITS;
        	pTxD->Aifs        = 1;
        	break;
        case 0: // TC0, <AIFSN, CwMin, CwMax> = <2, aCwMin, aCwMax>
        default:
        	pTxD->CWmin       = CW_MIN_IN_BITS;
        	pTxD->CWmax       = CW_MAX_IN_BITS;
        	pTxD->Aifs        = 2;
        	break;
	}
		
	if (Rate < RATE_FIRST_OFDM_RATE)
		pTxD->Ofdm = 0;
	else
		pTxD->Ofdm = 1;

	// fill up PLCP SIGNAL field
	pTxD->PlcpSignal = PlcpSignal[Rate];
	if (((Rate == RATE_2) || (Rate == RATE_5_5) || (Rate == RATE_11)) && (TxPreamble == Rt802_11PreambleShort)) // no short preamble for RATE_1
	{
		pTxD->PlcpSignal |= 0x0008;
	}

	// fill up PLCP SERVICE field, not used for OFDM rates
	pTxD->PlcpService = Service;

	// file up PLCP LENGTH_LOW and LENGTH_HIGH fields
	Length += 4;
	if (Rate < RATE_FIRST_OFDM_RATE)    // 11b - RATE_1, RATE_2, RATE_5_5, RATE_11
	{
		if ((Rate == RATE_1) || ( Rate == RATE_2))
		{
			Length = Length * 8 / (Rate + 1);
		}
		else
		{
			Residual = ((Length * 16) % (11 * (1 + Rate - RATE_5_5)));
			Length = Length * 16 / (11 * (1 + Rate - RATE_5_5));
			if (Residual != 0)
			{
				Length++;
			}
			if ((Residual <= (3 * (1 + Rate - RATE_5_5))) && (Residual != 0))
			{
				if (Rate == RATE_11)			// Only 11Mbps require length extension bit
					pTxD->PlcpService |= 0x80; // 11b's PLCP Length extension bit
			}
		}

		pTxD->PlcpLengthHigh = Length / 256;
		pTxD->PlcpLengthLow = Length % 256;
	}
	else    // OFDM - RATE_6, RATE_9, RATE_12, RATE_18, RATE_24, RATE_36, RATE_48, RATE_54
	{
		pTxD->PlcpLengthHigh = Length / 64;  // high 6-bit of total byte count
		pTxD->PlcpLengthLow = Length % 64;   // low 6-bit of total byte count
	}
	
	if (DoEncrypt == TRUE)		// Do encryption only
	{
		pTxD->Owner     = DESC_OWN_HOST;
		pTxD->Valid     = FALSE;
		pTxD->CipherAlg = CipherAlg;
		pTxD->CipherOwn = DESC_OWN_NIC;
	}
	else	// Hard transmit
	{
		pTxD->Valid     = TRUE;
		pTxD->CipherAlg = CIPHER_NONE;
		pTxD->CipherOwn = DESC_OWN_HOST;
		pTxD->Owner     = DESC_OWN_NIC;
	}
#ifdef BIG_ENDIAN
    RTMPDescriptorEndianChange((PUCHAR)pTxD, TYPE_TXD);
    *pSourceTxD = *pTxD;
#endif
}

/*
	========================================================================

	Routine	Description:
		Search tuple cache for receive duplicate frame from unicast frames.
		
	Arguments:
		pAdapter		Pointer	to our adapter
		pHeader			802.11 header of receiving frame
		
	Return Value:
		TRUE			found matched tuple cache
		FALSE			no matched found

	Note:
	
	========================================================================
*/
BOOLEAN	RTMPSearchTupleCache(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PHEADER_802_11	pHeader)
{
	INT	Index;

	for (Index = 0; Index < MAX_CLIENT; Index++)
	{
		if (pAdapter->TupleCache[Index].Valid == FALSE)
		    continue;
		
		if (RTMPEqualMemory(&pAdapter->TupleCache[Index].MAC, &pHeader->Controlhead.Addr2, 6) &&
			(pAdapter->TupleCache[Index].Sequence == pHeader->Sequence) &&
			(pAdapter->TupleCache[Index].Frag == pHeader->Frag))
		{
//			DBGPRINT(RT_DEBUG_TRACE,("DUPCHECK - duplicate frame hit entry %d\n", Index)); 
			return (TRUE);
		}
	}
	return (FALSE);
}

/*
	========================================================================

	Routine	Description:
		Update tuple cache for new received unicast frames.
		
	Arguments:
		pAdapter		Pointer	to our adapter
		pHeader			802.11 header of receiving frame
		
	Return Value:
		None
		
	Note:
	
	========================================================================
*/
VOID	RTMPUpdateTupleCache(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PHEADER_802_11	pHeader)
{
	UCHAR	Index;

	for (Index = 0; Index < MAX_CLIENT; Index++)
	{
		if (pAdapter->TupleCache[Index].Valid == FALSE)
		{
			// Add new entry
			memcpy(&pAdapter->TupleCache[Index].MAC, &pHeader->Controlhead.Addr2, 6);
			pAdapter->TupleCache[Index].Sequence = pHeader->Sequence;
			pAdapter->TupleCache[Index].Frag     = pHeader->Frag;
			pAdapter->TupleCache[Index].Valid    = TRUE;
			pAdapter->TupleCacheLastUpdateIndex  = Index;
			DBGPRINT(RT_DEBUG_INFO,"DUPCHECK - Add Entry %d, MAC=%02x:%02x:%02x:%02x:%02x:%02x\n", 
			    Index, pAdapter->TupleCache[Index].MAC.Octet[0], pAdapter->TupleCache[Index].MAC.Octet[1],
			    pAdapter->TupleCache[Index].MAC.Octet[2], pAdapter->TupleCache[Index].MAC.Octet[3],
			    pAdapter->TupleCache[Index].MAC.Octet[4], pAdapter->TupleCache[Index].MAC.Octet[5]);
			return;
		}
		else if (RTMPEqualMemory(&pAdapter->TupleCache[Index].MAC, &pHeader->Controlhead.Addr2, 6))
		{
			// Update old entry
			pAdapter->TupleCache[Index].Sequence = pHeader->Sequence;
			pAdapter->TupleCache[Index].Frag     = pHeader->Frag;
			return;
		}
	}

    // tuple cache full, replace the first inserted one (even though it may not be
    // least referenced one)
	if (Index == MAX_CLIENT)
	{
	    pAdapter->TupleCacheLastUpdateIndex ++;
	    if (pAdapter->TupleCacheLastUpdateIndex >= MAX_CLIENT)
	        pAdapter->TupleCacheLastUpdateIndex = 0;
	    Index = pAdapter->TupleCacheLastUpdateIndex;

		// replace with new entry
		memcpy(&pAdapter->TupleCache[Index].MAC, &pHeader->Controlhead.Addr2, 6);
		pAdapter->TupleCache[Index].Sequence = pHeader->Sequence;
		pAdapter->TupleCache[Index].Frag     = pHeader->Frag;
		pAdapter->TupleCache[Index].Valid    = TRUE;
		DBGPRINT(RT_DEBUG_INFO,"DUPCHECK - replace Entry %d, MAC=%02x:%02x:%02x:%02x:%02x:%02x\n", 
		    Index, pAdapter->TupleCache[Index].MAC.Octet[0], pAdapter->TupleCache[Index].MAC.Octet[1],
		    pAdapter->TupleCache[Index].MAC.Octet[2], pAdapter->TupleCache[Index].MAC.Octet[3],
		    pAdapter->TupleCache[Index].MAC.Octet[4], pAdapter->TupleCache[Index].MAC.Octet[5]);
	}
}

/*
	========================================================================

	Routine	Description:
		Suspend MSDU transmission
		
	Arguments:
		pAdapter		Pointer	to our adapter
		
	Return Value:
		None
		
	Note:
	
	========================================================================
*/
VOID    RTMPSuspendMsduTransmission(
	IN	PRTMP_ADAPTER	pAdapter)
{
	DBGPRINT(RT_DEBUG_TRACE,"SCANNING, suspend MSDU transmission ...\n");
	RTMP_SET_FLAG(pAdapter, fRTMP_ADAPTER_BSS_SCAN_IN_PROGRESS);
}

/*
	========================================================================

	Routine	Description:
		Resume MSDU transmission
		
	Arguments:
		pAdapter		Pointer	to our adapter
		
	Return Value:
		None
		
	Note:
	
	========================================================================
*/
VOID    RTMPResumeMsduTransmission(
	IN	PRTMP_ADAPTER	pAdapter)
{
	DBGPRINT(RT_DEBUG_INFO,"SCAN done, resume MSDU transmission ...\n");
	RTMP_CLEAR_FLAG(pAdapter, fRTMP_ADAPTER_BSS_SCAN_IN_PROGRESS);

	// Dequeue Tx queue if Reset is not in progress
	if ((!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RESET_IN_PROGRESS)) &&
		(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF)))
	{
		//RTMPDeQueuePacket(pAdapter, &pAdapter->TxSwQueue0);
		// Call dequeue without selected queue, let the subroutine select the right priority
		// Tx software queue
		RTMPDeQueuePacket(pAdapter);
	}
}

/*
	========================================================================

	Routine	Description:
		Apply packet filter policy, return NDIS_STATUS_FAILURE if this frame
		should be dropped.
		
	Arguments:
		pAdapter		Pointer	to our adapter
		pRxD			Pointer	to the Rx descriptor
		pHeader			Pointer to the 802.11 frame header
		
	Return Value:
		NDIS_STATUS_SUCCESS		Accept frame
		NDIS_STATUS_FAILURE		Drop Frame
		
	Note:
		Maganement frame should bypass this filtering rule.
	
	========================================================================
*/
NDIS_STATUS	RTMPApplyPacketFilter(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PRXD_STRUC		pRxD, 
	IN	PHEADER_802_11	pHeader)
{
	UCHAR	i;
	
	// 0. Management frame should bypass all these filtering rules.
	if (pHeader->Controlhead.Frame.Type == BTYPE_MGMT)
	{
		return(NDIS_STATUS_SUCCESS);
	}
	
	// 0.1	Drop all Rx frames if MIC countermeasures kicks in
	if (pAdapter->PortCfg.MicErrCnt >= 2)
	{
		return(NDIS_STATUS_FAILURE);
	}
	
	// 1. Drop unicast to me packet if NDIS_PACKET_TYPE_DIRECTED is FALSE
	if (pRxD->U2M)
	{
		if (pAdapter->bAcceptDirect == FALSE)
		{
			return(NDIS_STATUS_FAILURE);
		}
	}
		
	// 2. Drop broadcast packet if NDIS_PACKET_TYPE_BROADCAST is FALSE
	else if (pRxD->Bcast)
	{
		if (pAdapter->bAcceptBroadcast == FALSE)
		{
			return(NDIS_STATUS_FAILURE);
		}
	}
			
	// 3. Drop multicast packet if NDIS_PACKET_TYPE_ALL_MULTICAST is false
	//    and NDIS_PACKET_TYPE_MULTICAST is false.
	//    If NDIS_PACKET_TYPE_MULTICAST is true, but NDIS_PACKET_TYPE_ALL_MULTICAST is false.
	//    We have to deal with multicast table lookup & drop not matched packets.
	else if (pRxD->Mcast)
	{
		if (pAdapter->bAcceptAllMulticast == FALSE)
		{
			if (pAdapter->bAcceptMulticast == FALSE)
			{
				return(NDIS_STATUS_FAILURE);
			}
			else
			{
				// Selected accept multicast packet based on multicast table
				for (i = 0; i < pAdapter->NumberOfMcAddresses; i++)
				{
					if (RTMPEqualMemory(&pHeader->Controlhead.Addr1, pAdapter->McastTable[i], ETH_ALEN))
					{
						break;		// Matched
					}
				}

				// Not matched
				if (i == pAdapter->NumberOfMcAddresses)
				{
					DBGPRINT(RT_DEBUG_INFO,"Drop multicast %02x:%02x:%02x:%02x:%02x:%02x\n",
						pHeader->Controlhead.Addr1.Octet[0], pHeader->Controlhead.Addr1.Octet[1],
						pHeader->Controlhead.Addr1.Octet[2], pHeader->Controlhead.Addr1.Octet[3],
						pHeader->Controlhead.Addr1.Octet[4], pHeader->Controlhead.Addr1.Octet[5]);
					return(NDIS_STATUS_FAILURE);
				}
				else
				{
					DBGPRINT(RT_DEBUG_INFO,"Accept multicast %02x:%02x:%02x:%02x:%02x:%02x\n",
						pHeader->Controlhead.Addr1.Octet[0], pHeader->Controlhead.Addr1.Octet[1],
						pHeader->Controlhead.Addr1.Octet[2], pHeader->Controlhead.Addr1.Octet[3],
						pHeader->Controlhead.Addr1.Octet[4], pHeader->Controlhead.Addr1.Octet[5]);
				}
			}
		}
	}

	// 4. Not U2M, not Mcast, not Bcast, must be unicast to other DA.
	//    Since we did not implement promiscuous mode, just drop this kind of packet for now.
	else if (pAdapter->bAcceptPromiscuous == FALSE)
	{
		return(NDIS_STATUS_FAILURE);
	}
	
	return(NDIS_STATUS_SUCCESS);	
}

/*
	========================================================================

	Routine	Description:
		Check and fine the packet waiting in SW queue with highest priority
		
	Arguments:
		pAdapter	Pointer	to our adapter
		
	Return Value:
		pQueue		Pointer to Waiting Queue

	Note:
	
	========================================================================
*/
struct sk_buff_head* RTMPCheckTxSwQueue(
	IN	PRTMP_ADAPTER	pAdapter,
	OUT UCHAR           *AccessCategory)
{
	if (!skb_queue_empty(&pAdapter->TxSwQueue3))
	{
	    *AccessCategory = 3;
		return (&pAdapter->TxSwQueue3);
	}
	else if (!skb_queue_empty(&pAdapter->TxSwQueue2))
	{
	    *AccessCategory = 2;
		return (&pAdapter->TxSwQueue2);
	}
	else if (!skb_queue_empty(&pAdapter->TxSwQueue1))
	{
	    *AccessCategory = 1;
		return (&pAdapter->TxSwQueue1);
	}
	else if (!skb_queue_empty(&pAdapter->TxSwQueue0))
	{
	    *AccessCategory = 0;
		return (&pAdapter->TxSwQueue0);
	}

	// No packet pending in Tx Sw queue
    *AccessCategory = 0;
	return (NULL);
}

/*
	========================================================================

	Routine	Description:
		Process MIC error indication and record MIC error timer.
		
	Arguments:
		pAdapter		Pointer	to our adapter
		pWpaKey			Pointer	to the WPA key structure
		
	Return Value:
		None
		
	Note:
	
	========================================================================
*/
VOID	RTMPReportMicError(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PWPA_KEY		pWpaKey)
{
	ULONG	Now;
	struct
	{
		NDIS_802_11_STATUS_INDICATION		Status;
		NDIS_802_11_AUTHENTICATION_REQUEST	Request;
	}	Report;

	// 0. Set Status to indicate auth error
	Report.Status.StatusType = Ndis802_11StatusType_Authentication;
	
	// 1. Check for Group or Pairwise MIC error
	if (pWpaKey->Type == PAIRWISE_KEY)
		Report.Request.Flags = NDIS_802_11_AUTH_REQUEST_PAIRWISE_ERROR;
	else
		Report.Request.Flags = NDIS_802_11_AUTH_REQUEST_GROUP_ERROR;

	// 2. Copy AP MAC address
	memcpy(Report.Request.Bssid, pWpaKey->BssId, 6);

	// 3. Calculate length
	Report.Request.Length = sizeof(NDIS_802_11_AUTHENTICATION_REQUEST);

	// 4. Record Last MIC error time and count
	Now = jiffies;
	if (pAdapter->PortCfg.MicErrCnt == 0)
	{
		pAdapter->PortCfg.MicErrCnt++;
		pAdapter->PortCfg.LastMicErrorTime = Now;
	}
	else if (pAdapter->PortCfg.MicErrCnt == 1)
	{
		if ((pAdapter->PortCfg.LastMicErrorTime + (60 * HZ)) < Now)
		{
			// Update Last MIC error time, this did not violate two MIC errors within 60 seconds
			pAdapter->PortCfg.LastMicErrorTime = Now;			
		}
		else
		{
			pAdapter->PortCfg.LastMicErrorTime = Now;			
			// Violate MIC error counts, MIC countermeasures kicks in
			pAdapter->PortCfg.MicErrCnt++;			
			// We shall block all reception
			// We shall clean all Tx ring and disassoicate from AP after next EAPOL frame
			RTMPRingCleanUp(pAdapter, TX_RING);
		}
	}
	else
	{
		// MIC error count >= 2
		// This should not happen
		;
	}
}
