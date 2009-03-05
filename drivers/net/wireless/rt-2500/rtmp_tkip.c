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
 *      Module Name: rtmp_tkip.c
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      PaulW           25th Feb 02     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#include	"rt_config.h"

// Rotation functions on 32 bit values 
#define ROL32( A, n )   ( ((A) << (n)) | ( ((A)>>(32-(n))) ) )
#define ROR32( A, n )   ROL32( (A), 32-(n) ) 

/*
	========================================================================

	Routine	Description:
		Convert from UCHAR[] to ULONG in a portable way 
		
	Arguments:
      pMICKey		pointer to MIC Key
		
	Return Value:
		None

	Note:
		
	========================================================================
*/
ULONG	RTMPTkipGetUInt32( 	
	IN	PUCHAR	pMICKey)
{  	
	ULONG	res = 0; 
	int		i;
	
	for (i = 0; i < 4; i++) 
	{ 
		res |= (*pMICKey++) << (8 * i); 
	}

	return res; 
} 

/*
	========================================================================

	Routine	Description:
		Convert from ULONG to UCHAR[] in a portable way 
		
	Arguments:
      pDst			pointer to destination for convert ULONG to UCHAR[]
      val			the value for convert
		
	Return Value:
		None

	Note:
		
	========================================================================
*/
VOID	RTMPTkipPutUInt32(
	IN OUT	PUCHAR		pDst,
	IN		ULONG		val)					  
{ 	
	int i;
	
	for(i = 0; i < 4; i++) 
	{ 
		*pDst++ = (UCHAR) val; 
		val >>= 8; 
	} 
} 

/*
	========================================================================

	Routine	Description:
		Calculate the MIC Value.
		
	Arguments:
      pAdapter		Pointer to our adapter
      pSrc			Pointer to source data for Calculate MIC Value
      Len			Indicate the length of the source data
		
	Return Value:
		None

	Note:
		
	========================================================================
*/
VOID	RTMPTkipAppend( 
	IN	PTKIP_KEY_INFO	pTkip,	
	IN	PUCHAR			pSrc,
	IN	UINT			nBytes)						  
{
    register ULONG  M, L, R, nBytesInM;

    // load data from memory to register
    L = pTkip->L;
    R = pTkip->R;
    nBytesInM = pTkip->nBytesInM;
    M = pTkip->M;
    
    // Alignment case
    if((nBytesInM == 0) && ((((unsigned long)pSrc) & 0x3) == 0))
    {
        while(nBytes >= 4)
        {
#ifdef BIG_ENDIAN
            M = SWAP32(*(ULONG *)pSrc);
#else
            M = *(ULONG *)pSrc;
#endif
            pSrc += 4;
            nBytes -= 4;
            
            L ^= M;
            R ^= ROL32( L, 17 );
            L += R;
            R ^= ((L & 0xff00ff00) >> 8) | ((L & 0x00ff00ff) << 8);
            L += R;
            R ^= ROL32( L, 3 );
            L += R;
            R ^= ROR32( L, 2 );
            L += R;
        }
        nBytesInM = 0;
        M = 0;
        
        while(nBytes > 0)
        {
            M |= (*pSrc << (8* nBytesInM));

            nBytesInM++;
            pSrc++;
            nBytes--;
            
            if( nBytesInM >= 4 )
            {
                L ^=  M;
                R ^= ROL32( L, 17 );
                L += R;
                R ^= ((L & 0xff00ff00) >> 8) | ((L & 0x00ff00ff) << 8);
                L += R;
                R ^= ROL32( L, 3 );
                L += R;
                R ^= ROR32( L, 2 );
                L += R;
                // Clear the buffer
                M = 0;
                nBytesInM = 0;
            }
        }
    }
    else
    {   // misAlignment case
        while(nBytes > 0)
        {
            M |= (*pSrc << (8* nBytesInM));
            nBytesInM++;
            
            pSrc++;
            nBytes--;
            
            if( nBytesInM >= 4 )
            {
                L ^=  M;
                R ^= ROL32( L, 17 );
                L += R;
                R ^= ((L & 0xff00ff00) >> 8) | ((L & 0x00ff00ff) << 8);
                L += R;
                R ^= ROL32( L, 3 );
                L += R;
                R ^= ROR32( L, 2 );
                L += R;
                // Clear the buffer
                M = 0;
                nBytesInM = 0;
            }
        }
    }
    
    // load data from register to memory
    pTkip->M = M;
    pTkip->nBytesInM = nBytesInM;
    pTkip->L = L;
    pTkip->R = R;
} 

/*
	========================================================================

	Routine	Description:
		Get the MIC Value.
		
	Arguments:
      pAdapter		Pointer to our adapter
		
	Return Value:
		None

	Note:
		the MIC Value is store in pAdapter->PrivateInfo.MIC
	========================================================================
*/
VOID	RTMPTkipGetMIC( 
	IN	PTKIP_KEY_INFO	pTkip)
{
    static unsigned char Last[] = {"\x5a\x00\x00\x00\x00\x00\x00\x00"};

    // Append the minimum padding
    RTMPTkipAppend(pTkip, Last, 8 - pTkip->nBytesInM);

    // The appendByte function has already computed the result.
    RTMPTkipPutUInt32(pTkip->MIC, pTkip->L);
    RTMPTkipPutUInt32(pTkip->MIC + 4, pTkip->R);
} 

/*
	========================================================================

	Routine	Description:
		Compare MIC value of received MSDU
		
	Arguments:
		pAdapter	Pointer to our adapter
		pSrc        Pointer to the received Plain text data
		pDA			Pointer to DA address
		pSA			Pointer to SA address
		pMICKey		pointer to MIC Key
		Len         the length of the received plain text data exclude MIC value
		
	Return Value:
		TRUE        MIC value matched
		FALSE       MIC value mismatched
		
	Note:
	
	========================================================================
*/
BOOLEAN	RTMPTkipCompareMICValue(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PUCHAR			pSrc,
	IN	PUCHAR			pDA,
	IN	PUCHAR			pSA,
	IN	PUCHAR			pMICKey,
	IN	UINT			Len)
{
	static UCHAR    Priority[4] = {"\x00\x00\x00\x00"};

	// Init MIC value calculation and reset the message
    pAdapter->PrivateInfo.Rx.L = RTMPTkipGetUInt32(pMICKey);
    pAdapter->PrivateInfo.Rx.R = RTMPTkipGetUInt32(pMICKey + 4);
	pAdapter->PrivateInfo.Rx.nBytesInM = 0;
	pAdapter->PrivateInfo.Rx.M = 0;

	// DA
	RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pDA, 6);
	// SA
	RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pSA, 6);
	// Priority + 3 bytes of 0
	RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, Priority, 4);
	
	// Calculate MIC value from plain text data
	RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pSrc, Len);

	// Get MIC value from decrypted plain data
	RTMPTkipGetMIC(&pAdapter->PrivateInfo.Rx);
		
	// Move MIC value from MSDU, this steps should move to data path.
	// Since the MIC value might cross MPDUs.
	if(!NdisEqualMemory(pAdapter->PrivateInfo.Rx.MIC, pSrc + Len, 8))
	{
	    INT		i;
	    
		DBGPRINT(RT_DEBUG_ERROR, "! TKIP MIC Error !\n");  //MIC error.
		DBGPRINT(RT_DEBUG_INFO, "Orig MIC value =");  //MIC error.
		for (i = 0; i < 8; i++)
		{
			DBGPRINT(RT_DEBUG_INFO, "%02x:", *(UCHAR*)(pSrc + Len + i));  //MIC error.
		}
		DBGPRINT(RT_DEBUG_INFO, "\n");  //MIC error.
		DBGPRINT(RT_DEBUG_INFO, "Calculated MIC value =");  //MIC error.
		for (i = 0; i < 8; i++)
		{
			DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PrivateInfo.Rx.MIC[i]);  //MIC error.
		}
		DBGPRINT(RT_DEBUG_INFO, "\n");  //MIC error.
		return (FALSE);
	}
	return (TRUE);
}

/*
	========================================================================

	Routine	Description:
		Compare MIC value of received MSDU
		
	Arguments:
		pAdapter	Pointer to our adapter
		pLLC		LLC header
		pSrc        Pointer to the received Plain text data
		pDA			Pointer to DA address
		pSA			Pointer to SA address
		pMICKey		pointer to MIC Key
		Len         the length of the received plain text data exclude MIC value
		
	Return Value:
		TRUE        MIC value matched
		FALSE       MIC value mismatched
		
	Note:
	
	========================================================================
*/
BOOLEAN	RTMPTkipCompareMICValueWithLLC(
	IN	PRTMP_ADAPTER	pAdapter,
	IN	PUCHAR			pLLC,
	IN	PUCHAR			pSrc,
	IN	PUCHAR			pDA,
	IN	PUCHAR			pSA,
	IN	PUCHAR			pMICKey,
	IN	UINT			Len)
{
    static UCHAR    Priority[4] = {"\x00\x00\x00\x00"};
    
    // Init MIC value calculation and reset the message
    pAdapter->PrivateInfo.Rx.L = RTMPTkipGetUInt32(pMICKey);
    pAdapter->PrivateInfo.Rx.R = RTMPTkipGetUInt32(pMICKey + 4);
    pAdapter->PrivateInfo.Rx.nBytesInM = 0;
    pAdapter->PrivateInfo.Rx.M = 0;
	
    // DA
    RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pDA, 6);
    // SA
    RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pSA, 6);
    // Priority + 3 bytes of 0
    RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, Priority, 4);

    // Start with LLC header
    RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pLLC, 8);

    // Calculate MIC value from plain text data
    RTMPTkipAppend(&pAdapter->PrivateInfo.Rx, pSrc, Len);

    // Get MIC value from decrypted plain data
    RTMPTkipGetMIC(&pAdapter->PrivateInfo.Rx);

    // Move MIC value from MSDU, this steps should move to data path.
    // Since the MIC value might cross MPDUs.
    if(!NdisEqualMemory(pAdapter->PrivateInfo.Rx.MIC, pSrc + Len, 8))
    {
        INT		i;

        DBGPRINT(RT_DEBUG_ERROR, "! TKIP MIC Error !\n");  //MIC error.
        DBGPRINT(RT_DEBUG_INFO, "Orig MIC value =");  //MIC error.
        for (i = 0; i < 8; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", *(UCHAR *)(pSrc + Len + i));  //MIC error.
        }

        DBGPRINT(RT_DEBUG_INFO, "\n");  //MIC error.
        DBGPRINT(RT_DEBUG_INFO, "Calculated MIC value =");  //MIC error.
        for (i = 0; i < 8; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PrivateInfo.Rx.MIC[i]);  //MIC error.
        }

        DBGPRINT(RT_DEBUG_INFO, "\n");  //MIC error.
        return (FALSE);
    }
    return (TRUE);
}

/*
	========================================================================

	Routine	Description:
		Copy frame from waiting queue into relative ring buffer and set 
	appropriate ASIC register to kick hardware transmit function
		
	Arguments:
		pAdapter		Pointer	to our adapter
		PNDIS_PACKET	Pointer to Ndis Packet for MIC calculation
		pEncap			Pointer to LLC encap data
		LenEncap		Total encap length, might be 0 which indicates no encap
		
	Return Value:
		None

	Note:
	
	========================================================================
*/
VOID RTMPCalculateMICValue(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct sk_buff  *skb,
    IN  PUCHAR          pEncap,
    IN  INT             LenEncap,
    IN  PWPA_KEY        pWpaKey)
{
    PUCHAR          pSrc;
    static UCHAR    Priority[4] = {"\x00\x00\x00\x00"};
    
    pSrc = (PUCHAR) skb->data;
    
    // Init MIC value calculation and reset the message
    pAdapter->PrivateInfo.Tx.L = RTMPTkipGetUInt32(pWpaKey->TxMic);
    pAdapter->PrivateInfo.Tx.R = RTMPTkipGetUInt32(pWpaKey->TxMic + 4);
    pAdapter->PrivateInfo.Tx.nBytesInM = 0;
    pAdapter->PrivateInfo.Tx.M = 0;
	
    // DA & SA field
    RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, pSrc, 12);
    
    // Priority + 3 bytes of 0
    RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, Priority, 4);
    
    if (LenEncap > 0)
    {
        // LLC encapsulation
        RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, pEncap, LenEncap);
        // Protocol Type
        RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, pSrc + 12, skb->len - 12);
    }
    else
        RTMPTkipAppend(&pAdapter->PrivateInfo.Tx, pSrc + 14, skb->len - 14);
    
    // Compute the final MIC Value
    RTMPTkipGetMIC(&pAdapter->PrivateInfo.Tx);
}

