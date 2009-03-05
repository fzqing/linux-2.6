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
 *      Module Name: rtmp_info.c
 *              
 *      Abstract: IOCTL related subroutines         
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      RoryC           3rd  Jan 03     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 *      RobinC          10th Dec 04     RFMON Support
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0 
 *      MarkW           15th Dec 04     Removed debug iwpriv
 *      RobinC          16th Dec 04     Fix for range values
 *		RobinC			16th Dec 04     support ifpreup scripts
 *      RobinC          17th Dec 04     Link Quality reporting
 *      MarkW           17th Dec 04     iwconfig frequency fix
 *      MarkW           17th Dec 04     Monitor mode through iwconfig 
 *      MarkW           22nd Dec 04     RSSI reporting for iwlist scanning
 *      MarkW           31st Jan 05     if pre-up fix for RaConfig
 *      LuisCorreia     23rd Feb 05     fix unknown IOCTL's
 *      MarkW           9th  Mar 05     Quality reporting in scan for current
 * 		MarkW			9th  Jun 05		Fix channel change for ADHOC mode
 ***************************************************************************/ 

#include    "rt_config.h"
#include <net/iw_handler.h>

#ifndef IW_ESSID_MAX_SIZE
/* Maximum size of the ESSID and NICKN strings */
#define IW_ESSID_MAX_SIZE   32
#endif

#define NR_WEP_KEYS 4
#define WEP_SMALL_KEY_LEN (40/8)
#define WEP_LARGE_KEY_LEN (104/8)

#define     MAP_CHANNEL_ID_TO_KHZ(ch, khz)  {               \
                switch (ch)                                 \
                {                                           \
                    case 1:     khz = 2412000;   break;     \
                    case 2:     khz = 2417000;   break;     \
                    case 3:     khz = 2422000;   break;     \
                    case 4:     khz = 2427000;   break;     \
                    case 5:     khz = 2432000;   break;     \
                    case 6:     khz = 2437000;   break;     \
                    case 7:     khz = 2442000;   break;     \
                    case 8:     khz = 2447000;   break;     \
                    case 9:     khz = 2452000;   break;     \
                    case 10:    khz = 2457000;   break;     \
                    case 11:    khz = 2462000;   break;     \
                    case 12:    khz = 2467000;   break;     \
                    case 13:    khz = 2472000;   break;     \
                    case 14:    khz = 2484000;   break;     \
                    case 36:  /* UNII */  khz = 5180000;   break;     \
                    case 40:  /* UNII */  khz = 5200000;   break;     \
                    case 44:  /* UNII */  khz = 5220000;   break;     \
                    case 48:  /* UNII */  khz = 5240000;   break;     \
                    case 52:  /* UNII */  khz = 5260000;   break;     \
                    case 56:  /* UNII */  khz = 5280000;   break;     \
                    case 60:  /* UNII */  khz = 5300000;   break;     \
                    case 64:  /* UNII */  khz = 5320000;   break;     \
                    case 149: /* UNII */  khz = 5745000;   break;     \
                    case 153: /* UNII */  khz = 5765000;   break;     \
                    case 157: /* UNII */  khz = 5785000;   break;     \
                    case 161: /* UNII */  khz = 5805000;   break;     \
                    case 100: /* HiperLAN2 */  khz = 5500000;   break;     \
                    case 104: /* HiperLAN2 */  khz = 5520000;   break;     \
                    case 108: /* HiperLAN2 */  khz = 5540000;   break;     \
                    case 112: /* HiperLAN2 */  khz = 5560000;   break;     \
                    case 116: /* HiperLAN2 */  khz = 5580000;   break;     \
                    case 120: /* HiperLAN2 */  khz = 5600000;   break;     \
                    case 124: /* HiperLAN2 */  khz = 5620000;   break;     \
                    case 128: /* HiperLAN2 */  khz = 5640000;   break;     \
                    case 132: /* HiperLAN2 */  khz = 5660000;   break;     \
                    case 136: /* HiperLAN2 */  khz = 5680000;   break;     \
                    case 140: /* HiperLAN2 */  khz = 5700000;   break;     \
                    case 34:  /* Japan MMAC */   khz = 5170000;   break;   \
                    case 38:  /* Japan MMAC */   khz = 5190000;   break;   \
                    case 42:  /* Japan MMAC */   khz = 5210000;   break;   \
                    case 46:  /* Japan MMAC */   khz = 5230000;   break;   \
                    default:    khz = 2412000;   break;     \
                }                                           \
            }

#define     MAP_KHZ_TO_CHANNEL_ID(khz, ch)  {               \
                switch (khz)                                \
                {                                           \
                    case 2412000:    ch = 1;     break;     \
                    case 2417000:    ch = 2;     break;     \
                    case 2422000:    ch = 3;     break;     \
                    case 2427000:    ch = 4;     break;     \
                    case 2432000:    ch = 5;     break;     \
                    case 2437000:    ch = 6;     break;     \
                    case 2442000:    ch = 7;     break;     \
                    case 2447000:    ch = 8;     break;     \
                    case 2452000:    ch = 9;     break;     \
                    case 2457000:    ch = 10;    break;     \
                    case 2462000:    ch = 11;    break;     \
                    case 2467000:    ch = 12;    break;     \
                    case 2472000:    ch = 13;    break;     \
                    case 2484000:    ch = 14;    break;     \
                    case 5180000:    ch = 36;  /* UNII */  break;     \
                    case 5200000:    ch = 40;  /* UNII */  break;     \
                    case 5220000:    ch = 44;  /* UNII */  break;     \
                    case 5240000:    ch = 48;  /* UNII */  break;     \
                    case 5260000:    ch = 52;  /* UNII */  break;     \
                    case 5280000:    ch = 56;  /* UNII */  break;     \
                    case 5300000:    ch = 60;  /* UNII */  break;     \
                    case 5320000:    ch = 64;  /* UNII */  break;     \
                    case 5745000:    ch = 149; /* UNII */  break;     \
                    case 5765000:    ch = 153; /* UNII */  break;     \
                    case 5785000:    ch = 157; /* UNII */  break;     \
                    case 5805000:    ch = 161; /* UNII */  break;     \
                    case 5500000:    ch = 100; /* HiperLAN2 */  break;     \
                    case 5520000:    ch = 104; /* HiperLAN2 */  break;     \
                    case 5540000:    ch = 108; /* HiperLAN2 */  break;     \
                    case 5560000:    ch = 112; /* HiperLAN2 */  break;     \
                    case 5580000:    ch = 116; /* HiperLAN2 */  break;     \
                    case 5600000:    ch = 120; /* HiperLAN2 */  break;     \
                    case 5620000:    ch = 124; /* HiperLAN2 */  break;     \
                    case 5640000:    ch = 128; /* HiperLAN2 */  break;     \
                    case 5660000:    ch = 132; /* HiperLAN2 */  break;     \
                    case 5680000:    ch = 136; /* HiperLAN2 */  break;     \
                    case 5700000:    ch = 140; /* HiperLAN2 */  break;     \
                    case 5170000:    ch = 34;  /* Japan MMAC */   break;   \
                    case 5190000:    ch = 38;  /* Japan MMAC */   break;   \
                    case 5210000:    ch = 42;  /* Japan MMAC */   break;   \
                    case 5230000:    ch = 46;  /* Japan MMAC */   break;   \
                    default:         ch = 1;     break;     \
                }                                           \
            }

struct iw_priv_args privtab[] = {
{ RTPRIV_IOCTL_SET, 
  IW_PRIV_TYPE_CHAR | 1024, 0,
  "set"},
{ RTPRIV_IOCTL_BBP,
  IW_PRIV_TYPE_CHAR | 1024, IW_PRIV_TYPE_CHAR | 1024,
  "bbp"},
{ RTPRIV_IOCTL_MAC,
  IW_PRIV_TYPE_CHAR | 1024, IW_PRIV_TYPE_CHAR | 1024,
  "mac"},
{ RTPRIV_IOCTL_E2P,
  IW_PRIV_TYPE_CHAR | 1024, IW_PRIV_TYPE_CHAR | 1024,
  "e2p"}
};

static struct {
    char *name;
    int (*set_proc)(PRTMP_ADAPTER pAdapter, PUCHAR arg);
} *PRTMP_PRIVATE_SET_PROC, RTMP_PRIVATE_SUPPORT_PROC[] = {
    {"CountryRegion", Set_CountryRegion_Proc },
    {"SSID", Set_SSID_Proc},
    {"WirelessMode", Set_WirelessMode_Proc},
    {"TxRate", Set_TxRate_Proc},
    {"AdhocOfdm", Set_AdhocModeRate_Proc},
    {"Channel", Set_Channel_Proc},
    {"BGProtection", Set_BGProtection_Proc},
    {"StaWithEtherBridge", Set_StaWithEtherBridge_Proc},
    {"TxPreamble", Set_TxPreamble_Proc},
    {"RTSThreshold", Set_RTSThreshold_Proc},
    {"FragThreshold", Set_FragThreshold_Proc},
    {"TxBurst", Set_TxBurst_Proc},
    {"TurboRate", Set_TurboRate_Proc},
    {"NetworkType", Set_NetworkType_Proc},
    {"AuthMode", Set_AuthMode_Proc},
    {"EncrypType", Set_EncrypType_Proc},
    {"DefaultKeyID", Set_DefaultKeyID_Proc},
    {"Key1", Set_Key1_Proc},
    {"Key2", Set_Key2_Proc},
    {"Key3", Set_Key3_Proc},
    {"Key4", Set_Key4_Proc},
    {"WPAPSK", Set_WPAPSK_Proc},
    {"WPANONE", Set_WPANONE_Proc},

#ifdef RALINK_ATE
	{"ATE",       Set_ATE_Proc			},	// set ATE Mode to: STOP, TXCONT, TXCARR, TXFRAME, RXFRAME
	{"ATEDA",     Set_ATE_DA_Proc		},	// set ATE TxFrames ADDR1, DA
	{"ATESA",     Set_ATE_SA_Proc		},	// set ATE TxFrames ADDR2, SA
	{"ATEBSSID",  Set_ATE_BSSID_Proc	},	// set ATE TxFrames ADDR3, BSSID
	{"ATECHANNEL",Set_ATE_CHANNEL_Proc	},	// set ATE Channel
	{"ATETXPOW",  Set_ATE_TX_POWER_Proc	},	// set ATE TxPower
	{"ATETXLEN",  Set_ATE_TX_LENGTH_Proc},	// set ATE TxLength
	{"ATETXCNT",  Set_ATE_TX_COUNT_Proc	},	// set ATE TxCount
	{"ATETXRATE", Set_ATE_TX_RATE_Proc	},	// set ATE TxRate
#endif	//#ifdef RALINK_ATE

    {NULL,}
};

char * rtstrchr(const char * s, int c)
{
         for(; *s != (char) c; ++s)
                 if (*s == '\0')
                         return NULL;
         return (char *) s;
 }
/*
This is required for LinEX2004/kernel2.6.7 to provide iwlist scanning function
*/
int rt_ioctl_giwrange(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *data, char *extra)
{
	PRTMP_ADAPTER pAdapter = (PRTMP_ADAPTER) dev->priv;
	struct iw_range *range = (struct iw_range *) extra;
	u16 val;
	int i,chan;

	DBGPRINT(RT_DEBUG_TRACE,"0. rtusb_ioctl_giwrange\n");		
	data->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(struct iw_range));

	range->txpower_capa = IW_TXPOW_DBM;

	if (INFRA_ON(pAdapter)||ADHOC_ON(pAdapter))
	{
		range->min_pmp = 1 * 1024;
		range->max_pmp = 65535 * 1024;
		range->min_pmt = 1 * 1024;
		range->max_pmt = 1000 * 1024;
		range->pmp_flags = IW_POWER_PERIOD;
		range->pmt_flags = IW_POWER_TIMEOUT;
		range->pm_capa = IW_POWER_PERIOD | IW_POWER_TIMEOUT |
			IW_POWER_UNICAST_R | IW_POWER_ALL_R;
	}

	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 14;

	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT;
	range->min_retry = 0;
	range->max_retry = 255;

	val = 0;
	for (i = 0; i < 14; i++) {
		chan = pAdapter->PortCfg.ChannelList[val];
		if (chan != 0)
		{
			range->freq[val].i = chan;
			MAP_CHANNEL_ID_TO_KHZ(range->freq[val].i, range->freq[val].m);
			range->freq[val].m*=100;		
			range->freq[val].e = 1;
			val++;
		}
	}

	range->num_frequency = val;
	range->num_channels = val;

	val = 0;
	for (i = 0; i < pAdapter->PortCfg.SupportedRatesLen; i++) {
		range->bitrate[i]=1000000*(pAdapter->PortCfg.SupportedRates[i] & 0x7f)/2;
		val++;
		if (val == IW_MAX_BITRATES)
			break;
	}
	range->num_bitrates = val;

        range->max_qual.qual = 100; /* % sig quality*/

	range->max_qual.level = 1; /* dB */
	range->max_qual.noise = 152; /* dB */
	range->max_qual.updated = 172; /* Updated all three */

	/* What would be suitable values for "average/typical" qual? */
	range->avg_qual.qual = pAdapter->Mlme.ChannelQuality;
	range->avg_qual.level = pAdapter->PortCfg.LastRssi;
	range->avg_qual.noise = 0;
	range->avg_qual.updated = 7; /* Updated all three */

	range->sensitivity = -30;

	range->max_encoding_tokens = NR_WEP_KEYS;
	range->num_encoding_sizes = 2;
	range->encoding_size[0] = 5;
	range->encoding_size[1] = 13;

#if 0
	over2 = 0;
	len = prism2_get_datarates(dev, rates);
	range->num_bitrates = 0;
	for (i = 0; i < len; i++) {
		if (range->num_bitrates < IW_MAX_BITRATES) {
			range->bitrate[range->num_bitrates] =
				rates[i] * 500000;
			range->num_bitrates++;
		}
		if (rates[i] == 0x0b || rates[i] == 0x16)
			over2 = 1;
	}
	/* estimated maximum TCP throughput values (bps) */
	range->throughput = over2 ? 5500000 : 1500000;
#endif
	range->min_rts = 0;
	range->max_rts = 2347;
	range->min_frag = 256;
	range->max_frag = 2346;

	return 0;
}

static int
rt_ioctl_setparam(struct net_device *dev, struct iw_request_info *info,
			 void *w, char *extra)
{
	PRTMP_ADAPTER pAdapter = (PRTMP_ADAPTER) dev->priv;
	char *this_char;
	char *value;
	int  Status;
  
				while ((this_char = strsep(&extra, ",")) != NULL) 
				{
					if (!*this_char)
						 continue;

					if ((value = rtstrchr(this_char, '=')) != NULL)
						*value++ = 0;

					if (!value || !*value)
						continue;

					for (PRTMP_PRIVATE_SET_PROC = RTMP_PRIVATE_SUPPORT_PROC; PRTMP_PRIVATE_SET_PROC->name; PRTMP_PRIVATE_SET_PROC++)
					{
						if (strcmp(this_char, PRTMP_PRIVATE_SET_PROC->name) == 0) 
						{						
							if(!PRTMP_PRIVATE_SET_PROC->set_proc(pAdapter, value))
							{	//FALSE:Set private failed then return Invalid argument
								Status = -EINVAL;
							}
							break;	//Exit for loop.
						}
					}

					if(PRTMP_PRIVATE_SET_PROC->name == NULL)
					{  //Not found argument
						Status = -EINVAL;
						DBGPRINT(RT_DEBUG_TRACE, "ioctl::(iwpriv) Not Support Set Command [%s=%s]\n", this_char, value);
						break;
					}
				}
				return 0;
}

static const iw_handler rt_priv_handlers[] = {
	(iw_handler) rt_ioctl_setparam,		/* SIOCWFIRSTPRIV+0 */
};

#ifdef SIOCGIWSCAN
int rt_ioctl_siwscan(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	ULONG								Now;
	PRTMP_ADAPTER pAdapter = (PRTMP_ADAPTER) dev->priv;
	int Status = NDIS_STATUS_SUCCESS;
	BOOLEAN 		StateMachineTouched = FALSE;
	if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_BSS_SCAN_IN_PROGRESS))
		return 0;
	if(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_MLME_INITIALIZED))
		return 0;
	do{
		Now = jiffies;

            if ((pAdapter->MediaState == NdisMediaStateConnected) &&
				((pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPA) || 
				(pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK)) &&
                (pAdapter->PortCfg.PortSecured == WPA_802_1X_PORT_NOT_SECURED)
                )
            {
                DBGPRINT(RT_DEBUG_TRACE, "!!! Link UP, Port Not Secured! ignore this set::OID_802_11_BSSID_LIST_SCAN\n");
				Status = NDIS_STATUS_SUCCESS;
				break;
            }

            if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
            {
                MlmeRestartStateMachine(pAdapter);
                DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
            }

            // tell CNTL state machine to call NdisMSetInformationComplete() after completing
            // this request, because this request is initiated by NDIS.
            pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE;
            // Reset Missed scan number
            pAdapter->PortCfg.IgnoredScanNumber = 0;
            pAdapter->PortCfg.LastScanTime = Now;

            MlmeEnqueue(&pAdapter->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_BSSID_LIST_SCAN, 
                    0, 
                    NULL);

		Status = NDIS_STATUS_SUCCESS;
		StateMachineTouched = TRUE;
	}while(0);
	return 0;
}
int
rt_ioctl_giwscan(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{

	PRTMP_ADAPTER pAdapter = (PRTMP_ADAPTER) dev->priv;
	int i=2, j;
	char *current_ev = extra;
	char *end_buf = extra + IW_SCAN_MAX_DATA;
	char *current_val;
	struct iw_event iwe;

	for (i = 0; i < pAdapter->PortCfg.BssTab.BssNr; i++) 
	{
		if (current_ev >= end_buf)
			break;

		//================================
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
				memcpy(iwe.u.ap_addr.sa_data, &pAdapter->PortCfg.BssTab.BssEntry[i].Bssid, ETH_ALEN);
			current_ev = iwe_stream_add_event(current_ev,end_buf, &iwe, IW_EV_ADDR_LEN);
		//================================
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWMODE;
		if (pAdapter->PortCfg.BssTab.BssEntry[i].BssType == Ndis802_11IBSS)
		{
			iwe.u.mode = IW_MODE_ADHOC;
		}
		else if (pAdapter->PortCfg.BssTab.BssEntry[i].BssType == Ndis802_11Infrastructure)
		{
			iwe.u.mode = IW_MODE_INFRA;
		}
		else
		{
			iwe.u.mode = IW_MODE_AUTO;
		}

		iwe.len = IW_EV_UINT_LEN;
		current_ev = iwe_stream_add_event(current_ev, end_buf, &iwe,  IW_EV_UINT_LEN);
		//================================
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.length = pAdapter->PortCfg.BssTab.BssEntry[i].SsidLen;
		iwe.u.data.flags = 1;
		current_ev = iwe_stream_add_point(current_ev,end_buf, &iwe, pAdapter->PortCfg.BssTab.BssEntry[i].Ssid);
		//================================
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWENCODE;
		if (CAP_IS_PRIVACY_ON (pAdapter->PortCfg.BssTab.BssEntry[i].CapabilityInfo ))
			iwe.u.data.flags =IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe.u.data.flags = IW_ENCODE_DISABLED;
		current_ev = iwe_stream_add_point(current_ev, end_buf,&iwe,  pAdapter->PortCfg.BssTab.BssEntry[i].Ssid);

		//================================
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWRATE;
		current_val = current_ev + IW_EV_LCP_LEN;
		//for (j = 0; j < pAdapter->PortCfg.BssTab.BssEntry[i].RatesLen;j++)
		for (j = 0; j < 1;j++)
		{
			iwe.u.bitrate.value = RateIdToMbps[pAdapter->PortCfg.BssTab.BssEntry[i].Rates[i]/2] * 1000000;
			iwe.u.bitrate.disabled = 0;
			current_val = iwe_stream_add_value(current_ev,
				current_val, end_buf, &iwe,
				IW_EV_PARAM_LEN);
		}
		//================================
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWFREQ;
		if (INFRA_ON(pAdapter) || ADHOC_ON(pAdapter))
			iwe.u.freq.m = pAdapter->PortCfg.BssTab.BssEntry[i].Channel;
		else
			iwe.u.freq.m = pAdapter->PortCfg.BssTab.BssEntry[i].Channel;
		iwe.u.freq.e = 0;
		iwe.u.freq.i = 0;
		current_ev = iwe_stream_add_event(current_ev,end_buf, &iwe, IW_EV_FREQ_LEN);
		//================================
		memset(&iwe, 0, sizeof(iwe));
                iwe.cmd = IWEVQUAL;
                if (memcmp(&pAdapter->PortCfg.BssTab.BssEntry[i].Bssid, &pAdapter->PortCfg.Bssid, ETH_ALEN) == 0)
                    iwe.u.qual.qual = pAdapter->Mlme.ChannelQuality;
                else
                    iwe.u.qual.qual = 0;
                iwe.u.qual.level = pAdapter->PortCfg.BssTab.BssEntry[i].Rssi - RSSI_TO_DBM_OFFSET;   // signal level (dBm) 
                iwe.u.qual.noise = (pAdapter->PortCfg.LastR17Value > BBP_R17_DYNAMIC_UP_BOUND) ? BBP_R17_DYNAMIC_UP_BOUND : ((ULONG) pAdapter->PortCfg.LastR17Value);           // // noise level (dBm) 

                current_ev = iwe_stream_add_event(current_ev,end_buf, &iwe, IW_EV_QUAL_LEN);                


                //================================
                memset(&iwe, 0, sizeof(iwe));
	}
	data->length = current_ev - extra;
	DBGPRINT(RT_DEBUG_TRACE,"rtusb_ioctl_giwscan. %d BSS returned\n",pAdapter->PortCfg.BssTab.BssNr);						
	return 0;
}
#endif
static const iw_handler rt_handler[] =
{
	(iw_handler) NULL,				/* SIOCSIWCOMMIT */
	(iw_handler) NULL,			/* SIOCGIWNAME	1 */	 
	(iw_handler) NULL,				/* SIOCSIWNWID */
	(iw_handler) NULL,				/* SIOCGIWNWID */
	(iw_handler) NULL,		/* SIOCSIWFREQ */
	(iw_handler) NULL,		/* SIOCGIWFREQ 5*/
	(iw_handler) NULL,		/* SIOCSIWMODE */
	(iw_handler) NULL,		/* SIOCGIWMODE */
	(iw_handler) NULL,		/* SIOCSIWSENS */
	(iw_handler) NULL,		/* SIOCGIWSENS */
	(iw_handler) NULL /* not used */,		/* SIOCSIWRANGE */
	(iw_handler) rt_ioctl_giwrange,		/* SIOCGIWRANGE 	11 */
	(iw_handler) NULL /* not used */,		/* SIOCSIWPRIV */
	(iw_handler) NULL /* kernel code */,		/* SIOCGIWPRIV */
	(iw_handler) NULL /* not used */,		/* SIOCSIWSTATS */
	(iw_handler) NULL /* kernel code */,		/* SIOCGIWSTATS 	f*/
	(iw_handler) NULL,		/* SIOCSIWSPY */
	(iw_handler) NULL,		/* SIOCGIWSPY */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,		/* SIOCSIWAP */
	(iw_handler) NULL,		/* SIOCGIWAP	0x15*/
	(iw_handler) NULL,				/* -- hole --	0x16 */
	(iw_handler) NULL,		/* SIOCGIWAPLIST */
#ifdef SIOCGIWSCAN
	(iw_handler) rt_ioctl_siwscan,		/* SIOCSIWSCAN		0x18*/
	(iw_handler) rt_ioctl_giwscan,		/* SIOCGIWSCAN */
#else
	(iw_handler) NULL,				/* SIOCSIWSCAN */
	(iw_handler) NULL,				/* SIOCGIWSCAN */
#endif /* SIOCGIWSCAN */
	(iw_handler) NULL,		/* SIOCSIWESSID */
	(iw_handler) NULL,		/* SIOCGIWESSID */
	(iw_handler) NULL,		/* SIOCSIWNICKN */
	(iw_handler) NULL,		/* SIOCGIWNICKN 1d*/
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,		/* SIOCSIWRATE 20*/
	(iw_handler) NULL,		/* SIOCGIWRATE */
	(iw_handler) NULL,		/* SIOCSIWRTS */
	(iw_handler) NULL,		/* SIOCGIWRTS */
	(iw_handler) NULL,		/* SIOCSIWFRAG */
	(iw_handler) NULL,		/* SIOCGIWFRAG 25*/
	(iw_handler) NULL,		/* SIOCSIWTXPOW */
	(iw_handler) NULL,		/* SIOCGIWTXPOW */
	(iw_handler) NULL,		/* SIOCSIWRETRY */
	(iw_handler) NULL,		/* SIOCGIWRETRY 29*/
	(iw_handler) NULL,		/* SIOCSIWENCODE 2a*/
	(iw_handler) NULL,		/* SIOCGIWENCODE 2b*/
	(iw_handler) NULL,		/* SIOCSIWPOWER 2c*/
	(iw_handler) NULL,		/* SIOCGIWPOWER 2d*/
};

const struct iw_handler_def rt2500_iw_handler_def =
{
#define	N(a)	(sizeof (a) / sizeof (a[0]))
	.standard	= (iw_handler *) rt_handler,
	.num_standard	= sizeof(rt_handler) / sizeof(iw_handler),
	.private	= (iw_handler *) rt_priv_handlers,
	.num_private		= N(rt_priv_handlers),
	.private_args	= (struct iw_priv_args *) privtab,
	.num_private_args	= N(privtab),
#if WIRELESS_EXT > 15
//	.spy_offset	= offsetof(struct hostap_interface, spy_data),
#endif /* WIRELESS_EXT > 15 */
};
INT RTMPSetInformation(
    IN  PRTMP_ADAPTER pAdapter,
    IN  OUT struct ifreq    *rq,
    IN  INT                 cmd)
{
    struct iwreq                        *wrq = (struct iwreq *) rq;
    NDIS_802_11_SSID                    Ssid, *pSsid=NULL;
    NDIS_802_11_MAC_ADDRESS             Bssid;
    RT_802_11_PHY_MODE                  PhyMode;
    RT_802_11_STA_CONFIG                StaConfig;
    NDIS_802_11_RATES                   aryRates;
    RT_802_11_PREAMBLE                  Preamble;
    NDIS_802_11_WEP_STATUS              WepStatus;
    NDIS_802_11_AUTHENTICATION_MODE     AuthMode;
    NDIS_802_11_NETWORK_INFRASTRUCTURE  BssType;
    NDIS_802_11_RTS_THRESHOLD           RtsThresh;
    NDIS_802_11_FRAGMENTATION_THRESHOLD FragThresh;
    NDIS_802_11_POWER_MODE              PowerMode;
    NDIS_802_11_TX_POWER_LEVEL          TxPowerLevel;
    PNDIS_802_11_KEY                    pKey = NULL;
    PNDIS_802_11_REMOVE_KEY             pRemoveKey = NULL;
    NDIS_802_11_CONFIGURATION           Config, *pConfig = NULL;
    ULONG                               Now;
    ULONG                               KeyIdx;
    INT                                 Status = NDIS_STATUS_SUCCESS;
    UCHAR                               CountryRegion;
    BOOLEAN                             RadioState;
    BOOLEAN                             StateMachineTouched = FALSE;
    USHORT                              TxTotalCnt;

    switch(cmd & 0x7FFF) {
        case RT_OID_802_11_COUNTRY_REGION:
            if (wrq->u.data.length != sizeof(CountryRegion))
                Status = -EINVAL;
            else
            {
                if(copy_from_user(&CountryRegion, wrq->u.data.pointer, wrq->u.data.length))
			Status = -EINVAL;
                pAdapter->PortCfg.CountryRegion = CountryRegion;
                DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_COUNTRY_REGION (=%d) \n", pAdapter->PortCfg.CountryRegion);
            }
            break;
        case OID_802_11_BSSID_LIST_SCAN:
            Now = jiffies;
			TxTotalCnt = pAdapter->DrsCounters.OneSecTxOkCount + 
						 pAdapter->DrsCounters.OneSecTxRetryOkCount + 
						 pAdapter->DrsCounters.OneSecTxFailCount;
			DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_BSSID_LIST_SCAN, TxCnt = %d \n", TxTotalCnt);
			// For XP WZC, we will allow scan every 10 times, roughly 10 minutes.
			//            if ((Oid == OID_802_11_BSSID_LIST_SCAN) &&
			//                (pAdapter->MediaState == NdisMediaStateConnected) &&
			//                (pAdapter->PortCfg.IgnoredScanNumber < 10))
            if (TxTotalCnt > 100)
            {
                DBGPRINT(RT_DEBUG_TRACE, "!!! Link UP, ignore this set::OID_802_11_BSSID_LIST_SCAN\n");
				Status = NDIS_STATUS_SUCCESS;
				pAdapter->PortCfg.IgnoredScanNumber++;
				break;
            }
            
            if ((pAdapter->MediaState == NdisMediaStateConnected) &&
				((pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPA) || 
				(pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPAPSK)) &&
                (pAdapter->PortCfg.PortSecured == WPA_802_1X_PORT_NOT_SECURED)
                )
            {
                DBGPRINT(RT_DEBUG_TRACE, "!!! Link UP, Port Not Secured! ignore this set::OID_802_11_BSSID_LIST_SCAN\n");
				Status = NDIS_STATUS_SUCCESS;
				break;
            }

            if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
            {
                MlmeRestartStateMachine(pAdapter);
                DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
            }

            // tell CNTL state machine to call NdisMSetInformationComplete() after completing
            // this request, because this request is initiated by NDIS.
            pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE;
            // Reset Missed scan number
            pAdapter->PortCfg.IgnoredScanNumber = 0;
            pAdapter->PortCfg.LastScanTime = Now;

            MlmeEnqueue(&pAdapter->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_BSSID_LIST_SCAN, 
                    0, 
                    NULL);

            Status = NDIS_STATUS_SUCCESS;
            StateMachineTouched = TRUE;
            break;
        case OID_802_11_SSID:
            if (wrq->u.data.length != sizeof(NDIS_802_11_SSID))
                Status = -EINVAL;
            else
            {
                if(copy_from_user(&Ssid, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                pSsid = &Ssid;

                if (pSsid->SsidLength > MAX_LEN_OF_SSID)
                    Status = -EINVAL;
                else
                {
                    if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
                    {
                        MlmeRestartStateMachine(pAdapter);
                        DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
                    } 
                     // tell CNTL state machine to call NdisMSetInformationComplete() after completing
                    // this request, because this request is initiated by NDIS.
                    pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 

                    MlmeEnqueue(&pAdapter->Mlme.Queue, 
                            MLME_CNTL_STATE_MACHINE, 
                            OID_802_11_SSID,
                            sizeof(NDIS_802_11_SSID),
                            (VOID *)pSsid
                            );
                    Status = NDIS_STATUS_SUCCESS;
                    StateMachineTouched = TRUE;

                    DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_SSID (Len=%d,Ssid=%s)\n", pSsid->SsidLength, pSsid->Ssid);
                }
            }
            break;
        case OID_802_11_BSSID:
            if (wrq->u.data.length != sizeof(NDIS_802_11_MAC_ADDRESS))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&Bssid, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}

                if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
                {
                    MlmeRestartStateMachine(pAdapter);
                    DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
                }

                // tell CNTL state machine to call NdisMSetInformationComplete() after completing
                // this request, because this request is initiated by NDIS.
                pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 

                MlmeEnqueue(&pAdapter->Mlme.Queue, 
                            MLME_CNTL_STATE_MACHINE, 
                            OID_802_11_BSSID, 
                            sizeof(NDIS_802_11_MAC_ADDRESS),
                            (VOID *)&Bssid);
                Status = NDIS_STATUS_SUCCESS;
                StateMachineTouched = TRUE;
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_BSSID %02x:%02x:%02x:%02x:%02x:%02x\n",
                                        Bssid[0], Bssid[1], Bssid[2], Bssid[3], Bssid[4], Bssid[5]);
            }
            break;
        case RT_OID_802_11_RADIO:
            if (wrq->u.data.length != sizeof(BOOLEAN))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&RadioState, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_RADIO (=%d)\n", RadioState);
                if (pAdapter->PortCfg.bSwRadio != RadioState)
                {
                    pAdapter->PortCfg.bSwRadio = RadioState;
                    if (pAdapter->PortCfg.bRadio != (pAdapter->PortCfg.bHwRadio && pAdapter->PortCfg.bSwRadio))
                    {
                        pAdapter->PortCfg.bRadio = (pAdapter->PortCfg.bHwRadio && pAdapter->PortCfg.bSwRadio);
                        if (pAdapter->PortCfg.bRadio == TRUE)
                            MlmeRadioOn(pAdapter);
                        else
                            MlmeRadioOff(pAdapter);
                    }
                }
            }
            break;
        case RT_OID_802_11_PHY_MODE:
            if (wrq->u.data.length != sizeof(RT_802_11_PHY_MODE))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&PhyMode, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                RTMPSetPhyMode(pAdapter, PhyMode);
                DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_PHY_MODE (=%d)\n", PhyMode);
            }
            break;
        case RT_OID_802_11_STA_CONFIG:
            if (wrq->u.data.length != sizeof(RT_802_11_STA_CONFIG))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&StaConfig, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                pAdapter->PortCfg.EnableTxBurst = StaConfig.EnableTxBurst;
                pAdapter->PortCfg.EnableTurboRate = StaConfig.EnableTurboRate;
                pAdapter->PortCfg.UseBGProtection = StaConfig.UseBGProtection;
//              pAdapter->PortCfg.UseShortSlotTime = StaConfig.UseShortSlotTime;
                pAdapter->PortCfg.UseShortSlotTime = 1; // 2003-10-30 always SHORT SLOT capable
                if (pAdapter->PortCfg.AdhocMode != StaConfig.AdhocMode)
                {
                    // allow dynamic change of "USE OFDM rate or not" in ADHOC mode
                    // if setting changed, need to reset current TX rate as well as BEACON frame format
                    pAdapter->PortCfg.AdhocMode = StaConfig.AdhocMode;
                    if (pAdapter->PortCfg.BssType == BSS_INDEP)
                    {
                        MlmeUpdateTxRates(pAdapter, FALSE);
                        MakeIbssBeacon(pAdapter);
                    }
                }
                DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_SET_STA_CONFIG (Burst=%d,72/100=%d,Protection=%d,ShortSlot=%d,OFDM in 11g Adhoc=%d\n",
                                        pAdapter->PortCfg.EnableTxBurst,
                                        pAdapter->PortCfg.EnableTurboRate,
                                        pAdapter->PortCfg.UseBGProtection,
                                        pAdapter->PortCfg.UseShortSlotTime,
                                        pAdapter->PortCfg.AdhocMode);
            }
            break;
        case OID_802_11_DESIRED_RATES:
            if (wrq->u.data.length != sizeof(NDIS_802_11_RATES))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&aryRates, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                memset(pAdapter->PortCfg.DesiredRates, 0, MAX_LEN_OF_SUPPORTED_RATES);
                memcpy(pAdapter->PortCfg.DesiredRates, &aryRates, sizeof(NDIS_802_11_RATES));
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_DESIRED_RATES (%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x)\n",
                    pAdapter->PortCfg.DesiredRates[0],pAdapter->PortCfg.DesiredRates[1],
                    pAdapter->PortCfg.DesiredRates[2],pAdapter->PortCfg.DesiredRates[3],
                    pAdapter->PortCfg.DesiredRates[4],pAdapter->PortCfg.DesiredRates[5],
                    pAdapter->PortCfg.DesiredRates[6],pAdapter->PortCfg.DesiredRates[7] );
                // Changing DesiredRate may affect the MAX TX rate we used to TX frames out
                MlmeUpdateTxRates(pAdapter, FALSE);
            }
            break;
        case RT_OID_802_11_PREAMBLE:
            if (wrq->u.data.length != sizeof(RT_802_11_PREAMBLE))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&Preamble, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                if (Preamble == Rt802_11PreambleShort)
                {
                    pAdapter->PortCfg.WindowsTxPreamble = Preamble;
                    MlmeSetTxPreamble(pAdapter, Rt802_11PreambleShort);
                }
                else if ((Preamble == Rt802_11PreambleLong) || (Preamble == Rt802_11PreambleAuto))
                {
                    // if user wants AUTO, initialize to LONG here, then change according to AP's
                    // capability upon association.
                    pAdapter->PortCfg.WindowsTxPreamble = Preamble;
                    MlmeSetTxPreamble(pAdapter, Rt802_11PreambleLong);
                }
                else
                {
                    Status = EINVAL;
                    break;
                }
                DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_SET_PREAMBLE (=%d)\n", Preamble);
            }
            break;
        case OID_802_11_WEP_STATUS:
            if (wrq->u.data.length != sizeof(NDIS_802_11_WEP_STATUS))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&WepStatus, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                // Since TKIP, AES, WEP are all supported. It should not have any invalid setting
                if (WepStatus <= Ndis802_11Encryption3KeyAbsent)
                {
                    if (pAdapter->PortCfg.WepStatus != WepStatus)
                    {
                        // Config has changed
                        pAdapter->bConfigChanged = TRUE;
                    }
                    pAdapter->PortCfg.WepStatus = WepStatus;
                }
                else
                {
                    Status  = -EINVAL;
                    break;
                }
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_WEP_STATUS (=%d)\n",WepStatus);
            }
            break;
        case OID_802_11_AUTHENTICATION_MODE:
            if (wrq->u.data.length != sizeof(NDIS_802_11_AUTHENTICATION_MODE)) 
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&AuthMode, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                if (AuthMode > Ndis802_11AuthModeMax)
                {
                    Status  = -EINVAL;
                    break;
                }
                else
                {
                    if (pAdapter->PortCfg.AuthMode != AuthMode)
                    {
                        // Config has changed
                        pAdapter->bConfigChanged = TRUE;
                    }
                    pAdapter->PortCfg.AuthMode = AuthMode;
                }
                pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_AUTHENTICATION_MODE (=%d) \n",pAdapter->PortCfg.AuthMode);
            }
            break;
        case OID_802_11_INFRASTRUCTURE_MODE:
            if (wrq->u.data.length != sizeof(NDIS_802_11_NETWORK_INFRASTRUCTURE))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&BssType, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                if (BssType == Ndis802_11IBSS) 
                {
                    if (pAdapter->PortCfg.BssType != BSS_INDEP)
                    {
                        // Config has changed
                        pAdapter->bConfigChanged = TRUE;
                    }
                    pAdapter->PortCfg.BssType = BSS_INDEP;
                    DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_INFRASTRUCTURE_MODE (AD-HOC)\n");
                }
                else if (BssType == Ndis802_11Infrastructure) 
                {
                    if (pAdapter->PortCfg.BssType != BSS_INFRA)
                    {
                        // Config has changed
                        pAdapter->bConfigChanged = TRUE;
                    }
                    pAdapter->PortCfg.BssType = BSS_INFRA;
                    DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_INFRASTRUCTURE_MODE (INFRA)\n");
                }
                else
                {
                    Status  = -EINVAL;
                    DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_INFRASTRUCTURE_MODE (unknown)\n");
                }
            }
            // Reset Ralink supplicant to not use, it will be set to start when UI set PMK key
            pAdapter->PortCfg.WpaState = SS_NOTUSE;
            break;
        case RT_OID_802_11_RESET_COUNTERS:
            memset(&pAdapter->WlanCounters, 0, sizeof(COUNTER_802_11));
            memset(&pAdapter->Counters, 0, sizeof(COUNTER_802_3));
            memset(&pAdapter->RalinkCounters, 0, sizeof(COUNTER_RALINK));
            memset(&pAdapter->Mlme.PrevWlanCounters, 0, sizeof(COUNTER_802_11));
            DBGPRINT(RT_DEBUG_INFO, "Set::RT_OID_802_11_RESET_COUNTERS \n");
            break;
        case OID_802_11_RTS_THRESHOLD:
            if (wrq->u.data.length != sizeof(NDIS_802_11_RTS_THRESHOLD))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&RtsThresh, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                if (RtsThresh > MAX_RTS_THRESHOLD)
                    Status  = -EINVAL;
                else
                    pAdapter->PortCfg.RtsThreshold = (USHORT)RtsThresh;
            }
            DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_RTS_THRESHOLD (=%d)\n",RtsThresh);
            break;
        case OID_802_11_FRAGMENTATION_THRESHOLD:
            if (wrq->u.data.length != sizeof(NDIS_802_11_FRAGMENTATION_THRESHOLD))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&FragThresh, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                pAdapter->PortCfg.bFragmentZeroDisable = FALSE;
                if (FragThresh > MAX_FRAG_THRESHOLD || FragThresh < MIN_FRAG_THRESHOLD)
                {
                    if (FragThresh == 0)
                    {
                        pAdapter->PortCfg.FragmentThreshold = MAX_FRAG_THRESHOLD;
                        pAdapter->PortCfg.bFragmentZeroDisable = TRUE;
                    }
                    else
                        Status  = -EINVAL;
                }
                else
                    pAdapter->PortCfg.FragmentThreshold = (USHORT)FragThresh;
            }
            DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_FRAGMENTATION_THRESHOLD (=%d) \n",FragThresh);
            break;
        case OID_802_11_POWER_MODE:
            if (wrq->u.data.length != sizeof(NDIS_802_11_POWER_MODE))
                Status = -EINVAL;
            else
            {
                if(copy_from_user(&PowerMode, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                // save user's policy here, but not change PortCfg.Psm immediately
                if (PowerMode == Ndis802_11PowerModeCAM) 
                {
                    // clear PSM bit immediately
                    MlmeSetPsmBit(pAdapter, PWR_ACTIVE);
                    pAdapter->PortCfg.RecvDtim = TRUE;
                    if (pAdapter->PortCfg.WindowsACCAMEnable == FALSE)
                        pAdapter->PortCfg.WindowsPowerMode = PowerMode;
                    pAdapter->PortCfg.WindowsBatteryPowerMode = PowerMode;
                }
                else if (PowerMode == Ndis802_11PowerModeMAX_PSP) 
                {
                    // do NOT turn on PSM bit here, wait until MlmeCheckForPsmChange()
                    // to exclude certain situations.
                    //     MlmeSetPsmBit(pAdapter, PWR_SAVE);
                    if (pAdapter->PortCfg.WindowsACCAMEnable == FALSE)
                        pAdapter->PortCfg.WindowsPowerMode = PowerMode;
                    pAdapter->PortCfg.WindowsBatteryPowerMode = PowerMode;
                    pAdapter->PortCfg.RecvDtim = TRUE;  // FALSE;
                    pAdapter->PortCfg.DefaultListenCount = 5;
                }
                else if (PowerMode == Ndis802_11PowerModeFast_PSP) 
                {
                    // do NOT turn on PSM bit here, wait until MlmeCheckForPsmChange()
                    // to exclude certain situations.
                    //     MlmeSetPsmBit(pAdapter, PWR_SAVE);
                    pAdapter->PortCfg.RecvDtim = TRUE;
                    if (pAdapter->PortCfg.WindowsACCAMEnable == FALSE)
                        pAdapter->PortCfg.WindowsPowerMode = PowerMode;
                    pAdapter->PortCfg.WindowsBatteryPowerMode = PowerMode;
                    pAdapter->PortCfg.DefaultListenCount = 3;
                }
                else
                    Status = -EINVAL;
            }
            DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_POWER_MODE (=%d)\n",PowerMode);
            break;
        case OID_802_11_TX_POWER_LEVEL:
            if (wrq->u.data.length != sizeof(NDIS_802_11_TX_POWER_LEVEL))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&TxPowerLevel, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                if (TxPowerLevel > MAX_TX_POWER_LEVEL)
                    Status  = -EINVAL;
                else
                    pAdapter->PortCfg.TxPower = (UCHAR)TxPowerLevel;
            }
            DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_TX_POWER_LEVEL (=%d) \n",TxPowerLevel);
            break;
        // For WPA PSK PMK key
        case RT_OID_802_11_ADD_WPA:
            pKey = kmalloc(wrq->u.data.length, GFP_KERNEL);
            if(copy_from_user(pKey, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
            if (pKey->Length != wrq->u.data.length)
            {
                Status  = -EINVAL;
                DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_ADD_WPA, Failed!!\n");
            }
            else
            {
                if (pAdapter->PortCfg.AuthMode != Ndis802_11AuthModeWPAPSK)
                {
                    Status = -EOPNOTSUPP;
                    DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_ADD_WPA, Failed!! [AuthMode != WPAPSK]\n");
                }
                else // Only for WPA PSK mode
                {
                    pAdapter->PortCfg.PskKey.KeyLen = (UCHAR) pKey->KeyLength;
                    memcpy(pAdapter->PortCfg.PskKey.Key, &pKey->KeyMaterial, pKey->KeyLength);
                    // Use RaConfig as PSK agent.
                    // Start STA supplicant state machine
                    pAdapter->PortCfg.WpaState = SS_START;

                    DBGPRINT(RT_DEBUG_TRACE, "Set::RT_OID_802_11_ADD_WPA (id=0x%x, Len=%d-byte)\n", pKey->KeyIndex, pKey->KeyLength);
                }
            }
            kfree(pKey);
            break;
        case OID_802_11_REMOVE_KEY:
            pRemoveKey = kmalloc(wrq->u.data.length, GFP_KERNEL);
            if(copy_from_user(pRemoveKey, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
            if (pRemoveKey->Length != wrq->u.data.length)
            {
                Status  = -EINVAL;
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_REMOVE_KEY, Failed!!\n");
            }
            else
            {
                if (pAdapter->PortCfg.AuthMode >= Ndis802_11AuthModeWPA)
                {
                    RTMPWPARemoveKeyProc(pAdapter, pRemoveKey);
                    DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_REMOVE_KEY, Remove WPA Key!!\n");
                }
                else
                {
                    KeyIdx = pRemoveKey->KeyIndex;

                    if (KeyIdx & 0x80000000)
                    {
                        // Should never set default bit when remove key
                        Status  = -EINVAL;
                        DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_REMOVE_KEY, Failed!!(Should never set default bit when remove key)\n");
                    }
                    else
                    {
                        KeyIdx = KeyIdx & 0x0fffffff;
                        if (KeyIdx > 3)
                        {
                            Status  = -EINVAL;
                            DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_REMOVE_KEY, Failed!!(KeyId[%d] out of range)\n", KeyIdx);
                        }
                        else
                        {
                            pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen = 0;
                            DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_REMOVE_KEY (id=0x%x, Len=%d-byte)\n", pRemoveKey->KeyIndex, pRemoveKey->Length);
                        }
                    }
                }
            }
            kfree(pRemoveKey);
            break;
        case OID_802_11_ADD_KEY:
            pKey = kmalloc(wrq->u.data.length, GFP_KERNEL);
            if(copy_from_user(pKey, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
            if (pKey->Length != wrq->u.data.length)
            {
                Status  = -EINVAL;
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_ADD_KEY, Failed!!\n");
            }
            else
            {
                if (pAdapter->PortCfg.AuthMode >= Ndis802_11AuthModeWPA)
                {
                    RTMPWPAAddKeyProc(pAdapter, pKey);
                }
                else    // Old WEP stuff
                {
                    KeyIdx = pKey->KeyIndex & 0x0fffffff;

                    // it is a shared key
                    if (KeyIdx > 4)
                        Status = -EINVAL;
                    else
                    {
                        pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen = (UCHAR) pKey->KeyLength;
                        memcpy(pAdapter->PortCfg.SharedKey[KeyIdx].Key, &pKey->KeyMaterial, pKey->KeyLength);
                        if (pKey->KeyIndex & 0x80000000)
                        {
                            // Default key for tx (shared key)
                            pAdapter->PortCfg.DefaultKeyId = (UCHAR) KeyIdx;
                        }
                    }
                }
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_ADD_KEY (id=0x%x, Len=%d-byte)\n", pKey->KeyIndex, pKey->KeyLength);
            }
            kfree(pKey);
            break;
        case OID_802_11_CONFIGURATION:
            if (wrq->u.data.length != sizeof(NDIS_802_11_CONFIGURATION))
                Status  = -EINVAL;
            else
            {
                if(copy_from_user(&Config, wrq->u.data.pointer, wrq->u.data.length)){
			Status = -EINVAL;
			break;
		}
                pConfig = &Config;
                pAdapter->PortCfg.IbssConfig.BeaconPeriod = (USHORT) pConfig->BeaconPeriod;
                pAdapter->PortCfg.IbssConfig.AtimWin = (USHORT) pConfig->ATIMWindow;
                MAP_KHZ_TO_CHANNEL_ID(pConfig->DSConfig, pAdapter->PortCfg.IbssConfig.Channel);
                DBGPRINT(RT_DEBUG_TRACE, "Set::OID_802_11_CONFIGURATION (BeacnPeriod=%d,AtimW=%d,Ch=%d)\n",
                    pConfig->BeaconPeriod, pConfig->ATIMWindow, pAdapter->PortCfg.IbssConfig.Channel);
                // Config has changed
                pAdapter->bConfigChanged = TRUE;
            }
            break;
        default:
            DBGPRINT(RT_DEBUG_TRACE, "Set::unknown IOCTL's subcmd = 0x%08x\n", cmd);
            Status = -EOPNOTSUPP;
            break;
    }

    return Status;
}

INT RTMPQueryInformation(
    IN  PRTMP_ADAPTER pAdapter,
    IN  OUT struct ifreq    *rq,
    IN  INT                 cmd)
{
    struct iwreq                        *wrq = (struct iwreq *) rq;
    NDIS_802_11_BSSID_LIST_EX           *pBssidList = NULL;
    PNDIS_WLAN_BSSID_EX                 pBss;
    NDIS_802_11_SSID                    Ssid;
    NDIS_802_11_CONFIGURATION           Configuration;
    RT_802_11_LINK_STATUS               LinkStatus;
    RT_802_11_STA_CONFIG                StaConfig;
    NDIS_802_11_STATISTICS              Statistics;
    NDIS_802_11_RTS_THRESHOLD           RtsThresh;
    NDIS_802_11_FRAGMENTATION_THRESHOLD FragThresh;
    NDIS_802_11_POWER_MODE              PowerMode;
    NDIS_802_11_NETWORK_INFRASTRUCTURE  BssType;
    RT_802_11_PREAMBLE                  PreamType;
    NDIS_802_11_AUTHENTICATION_MODE     AuthMode;
    NDIS_802_11_WEP_STATUS              WepStatus;
    RT_VERSION_INFO                     DriverVersionInfo;
    ULONG                               BssBufSize;
    ULONG                               BssLen;
    ULONG                               ulInfo = 0;
    ULONG                               FcsValue;
    PUCHAR                              pBuf = NULL;
    PUCHAR                              pPtr;
    INT                                 Status = NDIS_STATUS_SUCCESS;
    UCHAR                               Padding;
    UINT                                i;
    BOOLEAN                             RadioState;

    switch(cmd) {
        case RT_OID_DEVICE_NAME:
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_DEVICE_NAME\n");
            if(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_IN_USE))
            {
                // TODO: Any client apps that use this need to be reworked so they are okay when the module is loaded but the interface is down. Until then ....
                // Interface is down, so pretend we don't exist.
                Status = -EFAULT;
            }
            else
            {
                wrq->u.data.length = sizeof(NIC_DEVICE_NAME);
                if(copy_to_user(wrq->u.data.pointer, NIC_DEVICE_NAME, wrq->u.data.length))
	    	    Status = -EFAULT;
            }
            break;
        case RT_OID_VERSION_INFO:
            DBGPRINT(RT_DEBUG_INFO, "Query::RT_OID_VERSION_INFO \n");
            DriverVersionInfo.DriverVersionW = DRV_VERSION_MAJOR;
            DriverVersionInfo.DriverVersionX = DRV_VERSION_MINOR;
            DriverVersionInfo.DriverVersionY = DRV_VERSION_SUB;
            DriverVersionInfo.DriverVersionZ = 0;
            DriverVersionInfo.DriverBuildYear = DRV_BUILD_YEAR;
            DriverVersionInfo.DriverBuildMonth = DRV_BUILD_MONTH;
            DriverVersionInfo.DriverBuildDay = DRV_BUILD_DAY;
            wrq->u.data.length = sizeof(RT_VERSION_INFO);
            if(copy_to_user(wrq->u.data.pointer, &DriverVersionInfo, wrq->u.data.length))
	    	Status = -EFAULT;
            break;
        case OID_802_11_BSSID_LIST:
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_BSSID_LIST (%d BSS returned)\n",pAdapter->PortCfg.BssTab.BssNr);
            // Claculate total buffer size required
            BssBufSize = sizeof(ULONG);
            
            for (i = 0; i < pAdapter->PortCfg.BssTab.BssNr; i++) 
            {
                // Align pointer to 4 bytes boundary.
                Padding = 4 - (pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen & 0x0003);
                if (Padding == 4)
                    Padding = 0;
                BssBufSize += (sizeof(NDIS_WLAN_BSSID_EX) - 4 + sizeof(NDIS_802_11_FIXED_IEs) + pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen + Padding);
            }

            // For safety issue, we add 256 bytes just in case
            BssBufSize += 256;
            // Allocate the same size as passed from higher layer
            pBuf = kmalloc(BssBufSize, GFP_KERNEL);
            if(pBuf == NULL)
            {
                Status = -ENOMEM;
                break;
            }
            // Init 802_11_BSSID_LIST_EX structure
            memset(pBuf, 0, BssBufSize);
            pBssidList = (PNDIS_802_11_BSSID_LIST_EX) pBuf;
            pBssidList->NumberOfItems = pAdapter->PortCfg.BssTab.BssNr;
            
            // Calculate total buffer length
            BssLen = 4; // Consist of NumberOfItems
            // Point to start of NDIS_WLAN_BSSID_EX
            // pPtr = pBuf + sizeof(ULONG);
            pPtr = (PUCHAR) &pBssidList->Bssid[0];
            for (i = 0; i < pAdapter->PortCfg.BssTab.BssNr; i++) 
            {
                pBss = (PNDIS_WLAN_BSSID_EX) pPtr;
                memcpy(&pBss->MacAddress, &pAdapter->PortCfg.BssTab.BssEntry[i].Bssid, ETH_ALEN);
                if ((pAdapter->PortCfg.BssTab.BssEntry[i].Hidden == 1) && (pAdapter->PortCfg.bShowHiddenSSID == FALSE))
                {
                    pBss->Ssid.SsidLength = 0;
                }
                else
                {
                    pBss->Ssid.SsidLength = pAdapter->PortCfg.BssTab.BssEntry[i].SsidLen;
                    memcpy(pBss->Ssid.Ssid, pAdapter->PortCfg.BssTab.BssEntry[i].Ssid, pAdapter->PortCfg.BssTab.BssEntry[i].SsidLen);
                }
                pBss->Privacy = pAdapter->PortCfg.BssTab.BssEntry[i].Privacy;
                pBss->Rssi = pAdapter->PortCfg.BssTab.BssEntry[i].Rssi - pAdapter->PortCfg.RssiToDbm; 
                pBss->NetworkTypeInUse = Ndis802_11DS;
                pBss->Configuration.Length = sizeof(NDIS_802_11_CONFIGURATION);
                pBss->Configuration.BeaconPeriod = pAdapter->PortCfg.BssTab.BssEntry[i].BeaconPeriod;
                pBss->Configuration.ATIMWindow = pAdapter->PortCfg.BssTab.BssEntry[i].AtimWin;

                MAP_CHANNEL_ID_TO_KHZ(pAdapter->PortCfg.BssTab.BssEntry[i].Channel, pBss->Configuration.DSConfig);

                if (pAdapter->PortCfg.BssTab.BssEntry[i].BssType == BSS_INFRA) 
                    pBss->InfrastructureMode = Ndis802_11Infrastructure;
                else
                    pBss->InfrastructureMode = Ndis802_11IBSS;

                memcpy(pBss->SupportedRates, pAdapter->PortCfg.BssTab.BssEntry[i].Rates, pAdapter->PortCfg.BssTab.BssEntry[i].RatesLen);

                DBGPRINT(RT_DEBUG_TRACE, "BSS#%d - %s, Ch %d = %d Khz\n",
                            i,pBss->Ssid.Ssid,pAdapter->PortCfg.BssTab.BssEntry[i].Channel,pBss->Configuration.DSConfig);

                if (pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen == 0)
                {
                    pBss->IELength = sizeof(NDIS_802_11_FIXED_IEs);
                    memcpy(pBss->IEs, &pAdapter->PortCfg.BssTab.BssEntry[i].FixIEs, sizeof(NDIS_802_11_FIXED_IEs));
                    pPtr = pPtr + sizeof(NDIS_WLAN_BSSID_EX) - 4 + sizeof(NDIS_802_11_FIXED_IEs);
                }
                else
                {
                    pBss->IELength = sizeof(NDIS_802_11_FIXED_IEs) + pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen;
                    pPtr = pPtr + sizeof(NDIS_WLAN_BSSID_EX) - 4 + sizeof(NDIS_802_11_FIXED_IEs);
                    memcpy(pBss->IEs, &pAdapter->PortCfg.BssTab.BssEntry[i].FixIEs, sizeof(NDIS_802_11_FIXED_IEs));
                    memcpy(pPtr, pAdapter->PortCfg.BssTab.BssEntry[i].VarIEs, pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen);
                    pPtr += pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen;
                }
                // Align pointer to 4 bytes boundary.
                Padding = 4 - (pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen & 0x0003);
                if (Padding == 4)
                    Padding = 0;
                pPtr += Padding;
                pBss->Length = sizeof(NDIS_WLAN_BSSID_EX) - 4 + sizeof(NDIS_802_11_FIXED_IEs) + pAdapter->PortCfg.BssTab.BssEntry[i].VarIELen + Padding;
                BssLen += pBss->Length;
            }
            wrq->u.data.length = BssLen;
            if(copy_to_user(wrq->u.data.pointer, pBssidList, wrq->u.data.length))
	    	Status = -EFAULT;
            kfree(pBssidList);
            break;
        case OID_802_3_CURRENT_ADDRESS:
            wrq->u.data.length = ETH_ALEN;
            if(copy_to_user(wrq->u.data.pointer, &pAdapter->CurrentAddress, wrq->u.data.length)){
	    	Status = -EFAULT;
		break;
	    }
            DBGPRINT(RT_DEBUG_INFO, "Query::OID_802_3_CURRENT_ADDRESS \n");
            break;
        case OID_GEN_MEDIA_CONNECT_STATUS:
            DBGPRINT(RT_DEBUG_INFO, "Query::OID_GEN_MEDIA_CONNECT_STATUS \n");
            wrq->u.data.length = sizeof(NDIS_MEDIA_STATE);
            if(copy_to_user(wrq->u.data.pointer, &pAdapter->MediaState, wrq->u.data.length))
	    	Status = -EFAULT;
            break;
        case OID_802_11_BSSID:
            if (INFRA_ON(pAdapter) || ADHOC_ON(pAdapter))
            {
                if(copy_to_user(wrq->u.data.pointer, &pAdapter->PortCfg.Bssid, sizeof(MACADDR)))
			Status = -EFAULT;

                DBGPRINT(RT_DEBUG_INFO, "IOCTL::SIOCGIWAP(=%02x:%02x:%02x:%02x:%02x:%02x)\n",
                        pAdapter->PortCfg.Bssid.Octet[0],pAdapter->PortCfg.Bssid.Octet[1],pAdapter->PortCfg.Bssid.Octet[2],
                        pAdapter->PortCfg.Bssid.Octet[3],pAdapter->PortCfg.Bssid.Octet[4],pAdapter->PortCfg.Bssid.Octet[5]);

            }
            else
            {
                DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_BSSID(=EMPTY)\n");
                Status = -ENOTCONN;
            }
            break;
        case OID_802_11_SSID:
            Ssid.SsidLength = pAdapter->PortCfg.SsidLen;
            memset(Ssid.Ssid, 0, MAX_LEN_OF_SSID);
            memcpy(Ssid.Ssid, pAdapter->PortCfg.Ssid, Ssid.SsidLength);
            wrq->u.data.length = sizeof(NDIS_802_11_SSID);
            if(copy_to_user(wrq->u.data.pointer, &Ssid, wrq->u.data.length))
	   	 Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_SSID (Len=%d, ssid=%s)\n", Ssid.SsidLength,Ssid.Ssid);
            break;
        case RT_OID_802_11_LINK_STATUS:
            LinkStatus.CurrTxRate = RateIdTo500Kbps[pAdapter->PortCfg.TxRate];   // unit : 500 kbps
            LinkStatus.ChannelQuality = pAdapter->Mlme.ChannelQuality;
            LinkStatus.RxByteCount = pAdapter->RalinkCounters.ReceivedByteCount;
            LinkStatus.TxByteCount = pAdapter->RalinkCounters.TransmittedByteCount;
            wrq->u.data.length = sizeof(RT_802_11_LINK_STATUS);
            if(copy_to_user(wrq->u.data.pointer, &LinkStatus, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_802_11_LINK_STATUS\n");
            break;
        case OID_802_11_CONFIGURATION:
            Configuration.Length = sizeof(NDIS_802_11_CONFIGURATION);
            Configuration.BeaconPeriod = pAdapter->PortCfg.BeaconPeriod;
            Configuration.ATIMWindow = pAdapter->PortCfg.AtimWin;
            MAP_CHANNEL_ID_TO_KHZ(pAdapter->PortCfg.Channel, Configuration.DSConfig);
            wrq->u.data.length = sizeof(NDIS_802_11_CONFIGURATION);
            if(copy_to_user(wrq->u.data.pointer, &Configuration, wrq->u.data.length))
	   	 Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_CONFIGURATION(BeaconPeriod=%d,AtimW=%d,Channel=%d) \n", 
                                    Configuration.BeaconPeriod, Configuration.ATIMWindow, pAdapter->PortCfg.Channel);
            break;
        case OID_802_11_RSSI:
            ulInfo = pAdapter->PortCfg.LastRssi - pAdapter->PortCfg.RssiToDbm; 
            wrq->u.data.length = sizeof(ulInfo);
            if(copy_to_user(wrq->u.data.pointer, &ulInfo, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_RSSI(=%d)\n", ulInfo);
            break;
        case OID_802_11_STATISTICS:
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_STATISTICS \n");
            // Update FCS counters
            RTMP_IO_READ32(pAdapter, CNT0, &FcsValue);
            pAdapter->WlanCounters.FCSErrorCount.QuadPart += ((FcsValue & 0x0000ffff) >> 7);
            // Add FCS error count to private counters
            pAdapter->RalinkCounters.RealFcsErrCount.QuadPart += FcsValue;

            // Sanity check for calculation of sucessful count
            if (pAdapter->WlanCounters.TransmittedFragmentCount.QuadPart < pAdapter->WlanCounters.RetryCount.QuadPart)
                pAdapter->WlanCounters.TransmittedFragmentCount.QuadPart = pAdapter->WlanCounters.RetryCount.QuadPart;

            Statistics.TransmittedFragmentCount.QuadPart = pAdapter->WlanCounters.TransmittedFragmentCount.QuadPart;
            Statistics.MulticastTransmittedFrameCount.QuadPart = pAdapter->WlanCounters.MulticastTransmittedFrameCount.QuadPart;
            Statistics.FailedCount.QuadPart = pAdapter->WlanCounters.FailedCount.QuadPart;
            Statistics.RetryCount.QuadPart = pAdapter->WlanCounters.RetryCount.QuadPart;
            Statistics.MultipleRetryCount.QuadPart = pAdapter->WlanCounters.MultipleRetryCount.QuadPart;
            Statistics.RTSSuccessCount.QuadPart = pAdapter->WlanCounters.RTSSuccessCount.QuadPart;
            Statistics.RTSFailureCount.QuadPart = pAdapter->WlanCounters.RTSFailureCount.QuadPart;
            Statistics.ACKFailureCount.QuadPart = pAdapter->WlanCounters.ACKFailureCount.QuadPart;
            Statistics.FrameDuplicateCount.QuadPart = pAdapter->WlanCounters.FrameDuplicateCount.QuadPart;
            Statistics.ReceivedFragmentCount.QuadPart = pAdapter->WlanCounters.ReceivedFragmentCount.QuadPart;
            Statistics.MulticastReceivedFrameCount.QuadPart = pAdapter->WlanCounters.MulticastReceivedFrameCount.QuadPart;
#ifdef RT2500_DBG			
            Statistics.FCSErrorCount = pAdapter->RalinkCounters.RealFcsErrCount;
#else
            Statistics.FCSErrorCount.QuadPart = pAdapter->WlanCounters.FCSErrorCount.QuadPart;
            Statistics.FrameDuplicateCount.vv.LowPart = pAdapter->WlanCounters.FrameDuplicateCount.vv.LowPart / 100;
#endif
            wrq->u.data.length = sizeof(NDIS_802_11_STATISTICS);
            if(copy_to_user(wrq->u.data.pointer, &Statistics, wrq->u.data.length))
	   	 Status = -EFAULT;
            break;
        case OID_GEN_RCV_OK:
            DBGPRINT(RT_DEBUG_INFO, "Query::OID_GEN_RCV_OK \n");
            ulInfo = pAdapter->Counters.GoodReceives;
            wrq->u.data.length = sizeof(ulInfo);
            if(copy_to_user(wrq->u.data.pointer, &ulInfo, wrq->u.data.length))
	    	Status = -EFAULT;
            break;
        case OID_GEN_RCV_NO_BUFFER:
            DBGPRINT(RT_DEBUG_INFO, "Query::OID_GEN_RCV_NO_BUFFER \n");
            ulInfo = pAdapter->Counters.RxNoBuffer;
            wrq->u.data.length = sizeof(ulInfo);
            if(copy_to_user(wrq->u.data.pointer, &ulInfo, wrq->u.data.length))
	    	Status = -EFAULT;
            break;
        case RT_OID_802_11_PHY_MODE:
            ulInfo = (ULONG)pAdapter->PortCfg.PhyMode;
            wrq->u.data.length = sizeof(ulInfo);
            if(copy_to_user(wrq->u.data.pointer, &ulInfo, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_802_11_PHY_MODE (=%d)\n", ulInfo);
            break;
        case RT_OID_802_11_STA_CONFIG:
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_802_11_QUERY_STA_CONFIG\n");
            StaConfig.EnableTxBurst = pAdapter->PortCfg.EnableTxBurst;
            StaConfig.EnableTurboRate = pAdapter->PortCfg.EnableTurboRate;
            StaConfig.UseBGProtection = pAdapter->PortCfg.UseBGProtection;
            StaConfig.UseShortSlotTime = pAdapter->PortCfg.UseShortSlotTime;
            StaConfig.AdhocMode = pAdapter->PortCfg.AdhocMode;
            StaConfig.HwRadioStatus = (pAdapter->PortCfg.bHwRadio == TRUE) ? 1 : 0;
            StaConfig.Rsv1 = 0;
            StaConfig.SystemErrorBitmap = pAdapter->PortCfg.SystemErrorBitmap;
            wrq->u.data.length = sizeof(RT_802_11_STA_CONFIG);
            if(copy_to_user(wrq->u.data.pointer, &StaConfig, wrq->u.data.length))
	    	Status = -EFAULT;
            break;
        case OID_802_11_RTS_THRESHOLD:
            RtsThresh = pAdapter->PortCfg.RtsThreshold;
            wrq->u.data.length = sizeof(RtsThresh);
            if(copy_to_user(wrq->u.data.pointer, &RtsThresh, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_RTS_THRESHOLD(=%d)\n", RtsThresh);
            break;
        case OID_802_11_FRAGMENTATION_THRESHOLD:
            FragThresh = pAdapter->PortCfg.FragmentThreshold;
            if (pAdapter->PortCfg.bFragmentZeroDisable == TRUE)
                FragThresh = 0;
            wrq->u.data.length = sizeof(FragThresh);
            if(copy_to_user(wrq->u.data.pointer, &FragThresh, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_FRAGMENTATION_THRESHOLD(=%d)\n", FragThresh);
            break;
        case OID_802_11_POWER_MODE:
            PowerMode = pAdapter->PortCfg.WindowsPowerMode;
            wrq->u.data.length = sizeof(PowerMode);
            if(copy_to_user(wrq->u.data.pointer, &PowerMode, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_POWER_MODE(=%d)\n", PowerMode);
            break;
        case RT_OID_802_11_RADIO:
            RadioState = (BOOLEAN) pAdapter->PortCfg.bSwRadio;
            wrq->u.data.length = sizeof(RadioState);
            if(copy_to_user(wrq->u.data.pointer, &RadioState, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_802_11_QUERY_RADIO (=%d)\n", RadioState);
            break;
        case OID_802_11_INFRASTRUCTURE_MODE:
            if (ADHOC_ON(pAdapter))
                BssType = Ndis802_11IBSS;
            else if (INFRA_ON(pAdapter))
                BssType = Ndis802_11Infrastructure;
            else
                BssType = Ndis802_11AutoUnknown;

            wrq->u.data.length = sizeof(BssType);
            if(copy_to_user(wrq->u.data.pointer, &BssType, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_INFRASTRUCTURE_MODE(=%d)\n", BssType);
            break;
        case RT_OID_802_11_PREAMBLE:
            PreamType = pAdapter->PortCfg.WindowsTxPreamble;
            wrq->u.data.length = sizeof(PreamType);
            if(copy_to_user(wrq->u.data.pointer, &PreamType, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_802_11_QUERY_PREAMBLE(=%d)\n", PreamType);
            break;
        case OID_802_11_AUTHENTICATION_MODE:
            AuthMode = pAdapter->PortCfg.AuthMode;
            wrq->u.data.length = sizeof(AuthMode);
            if(copy_to_user(wrq->u.data.pointer, &AuthMode, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_AUTHENTICATION_MODE(=%d)\n", AuthMode);
            break;
        case OID_802_11_WEP_STATUS:
            WepStatus = pAdapter->PortCfg.WepStatus;
            wrq->u.data.length = sizeof(WepStatus);
            if(copy_to_user(wrq->u.data.pointer, &WepStatus, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::OID_802_11_WEP_STATUS(=%d)\n", WepStatus);
            break;

        case RT_OID_802_11_QUERY_EEPROM_VERSION:
            wrq->u.data.length = sizeof(ULONG);
            if(copy_to_user(wrq->u.data.pointer, &pAdapter->PortCfg.EepromVersion, wrq->u.data.length))
	    	Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "Query::RT_OID_802_11_QUERY_EEPROM_VERSION (=%d)\n", pAdapter->PortCfg.EepromVersion);
            break;

        default:
            DBGPRINT(RT_DEBUG_TRACE, "Query::unknown IOCTL's subcmd = 0x%08x\n", cmd);
            Status = -EOPNOTSUPP;
            break;
    }

    return Status;
}

INT RT2500_ioctl(
    IN  struct net_device   *net_dev, 
    IN  OUT struct ifreq    *rq, 
    IN  INT                 cmd)
{
    PRTMP_ADAPTER                       pAdapter= net_dev->priv;
    struct iwreq                        *wrq = (struct iwreq *) rq;
    struct iw_point                     *erq = NULL;
    struct iw_freq                      *frq = NULL;
    NDIS_802_11_SSID                    Ssid, *pSsid=NULL;
    NDIS_802_11_NETWORK_INFRASTRUCTURE  BssType = Ndis802_11Infrastructure;
    NDIS_802_11_RTS_THRESHOLD           RtsThresh;
    NDIS_802_11_FRAGMENTATION_THRESHOLD FragThresh;
    NDIS_802_11_MAC_ADDRESS             Bssid;
    INT                                 Status = NDIS_STATUS_SUCCESS;   
    USHORT                              subcmd;
    BOOLEAN                             StateMachineTouched = FALSE;
    int                                 i, chan = -1, index = 0, len = 0;


    switch(cmd) {
        case SIOCGIWNAME:
            DBGPRINT(RT_DEBUG_TRACE, "IOCTL::SIOCGIWNAME\n");
            strcpy(wrq->u.name, "RT2500 Wireless");   //Less then 16 bytes. 
            break;
        case SIOCSIWESSID:  //Set ESSID
            erq = &wrq->u.essid;
            memset(&Ssid, 0x00, sizeof(NDIS_802_11_SSID));
            if (erq->flags)
            {
                if (erq->length > IW_ESSID_MAX_SIZE)
                {
                    Status = -E2BIG;
                    break;
                }

		if(RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_IN_USE))
		{
                	if (copy_from_user(Ssid.Ssid, erq->pointer, (erq->length - 1)))
                	{
                   	 Status = -EFAULT;
                   	 break;
                	}
                	Ssid.SsidLength = erq->length - 1;  //minus null character.
		}else{
			// This SEEMS to be needed to actual work RobinC when iface
			// is down
	                if (copy_from_user(pAdapter->PortCfg.Ssid, erq->pointer, (erq->length - 1)))
	                {
	                    Status = -EFAULT;
	                    break;
	                }
	                pAdapter->PortCfg.SsidLen = erq->length - 1;  //minus null character.

			memcpy(pAdapter->Mlme.CntlAux.Ssid, pAdapter->PortCfg.Ssid, pAdapter->PortCfg.SsidLen);	
			pAdapter->Mlme.CntlAux.SsidLen = pAdapter->PortCfg.SsidLen; 
		}
            }
            else
                Ssid.SsidLength = 0;  // ANY ssid 

            pSsid = &Ssid;

	    // if network is down then me MUST not proceed into actualy
	    // running mlme stuff
	    if(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_IN_USE))
		break;

            if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
            {
                MlmeRestartStateMachine(pAdapter);
                DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
            }

             // tell CNTL state machine to call NdisMSetInformationComplete() after completing
            // this request, because this request is initiated by NDIS.
            pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 

            MlmeEnqueue(&pAdapter->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_SSID,
                    sizeof(NDIS_802_11_SSID),
                    (VOID *)pSsid
                    );
            Status = NDIS_STATUS_SUCCESS;
            StateMachineTouched = TRUE;

            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWESSID[cmd=0x%x] (Len=%d,Ssid=%s)\n", SIOCSIWESSID, pSsid->SsidLength, pSsid->Ssid);
            break;
        case SIOCGIWESSID:  //Get ESSID
            erq = &wrq->u.essid;

                erq->flags=1;
                erq->length = pAdapter->PortCfg.SsidLen;
                if(copy_to_user(erq->pointer, pAdapter->PortCfg.Ssid, erq->length))
			Status = -EFAULT;
            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCGIWESSID (Len=%d, ssid=%s...)\n", erq->length, pAdapter->PortCfg.Ssid);
            break;
        case SIOCGIWNWID: // get network id 
            Status = -EOPNOTSUPP;
            break;
        case SIOCSIWNWID: // set network id (the cell)
            Status = -EOPNOTSUPP;
            break;
        case SIOCSIWFREQ: // set channel/frequency (Hz)
            frq = &wrq->u.freq;
            if((frq->e == 0) && (frq->m <= 1000))
                chan = frq->m;  // Setting by channel number 
            else
                MAP_KHZ_TO_CHANNEL_ID( (frq->m /100) , chan); // Setting by frequency - search the table , like 2.412G, 2.422G, 
            pAdapter->PortCfg.IbssConfig.Channel = chan;
            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWFREQ[cmd=0x%x] (Channel=%d)\n", SIOCSIWFREQ, pAdapter->PortCfg.IbssConfig.Channel);
            if(pAdapter->PortCfg.BssType == BSS_MONITOR || pAdapter->PortCfg.BssType == BSS_INDEP)
            {
               pAdapter->PortCfg.Channel = chan; 
               AsicSwitchChannel(pAdapter, pAdapter->PortCfg.Channel);
               AsicLockChannel(pAdapter, pAdapter->PortCfg.Channel);
            }
			break;
        case SIOCGIWFREQ: // get channel/frequency (Hz)
            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCGIWFREQ\n");
            if (INFRA_ON(pAdapter) || ADHOC_ON(pAdapter)){
		MAP_CHANNEL_ID_TO_KHZ(pAdapter->PortCfg.Channel, wrq->u.freq.m);
            }else{
	  	  MAP_CHANNEL_ID_TO_KHZ(pAdapter->PortCfg.IbssConfig.Channel, wrq->u.freq.m);
	    }
            // MW: Alter the multiplier so iwconfig reports GhZ
            wrq->u.freq.e = 3;
            wrq->u.freq.i = 0;
            break;
        case SIOCGIWNICKN: //get node name/nickname
            erq = &wrq->u.data;
            erq->length = strlen(pAdapter->nickn);
            if(copy_to_user(erq->pointer, pAdapter->nickn, erq->length))
	    	Status = -EFAULT;
            break;
        case SIOCSIWNICKN: //set node name/nickname
            erq = &wrq->u.data;
            if (erq->flags)
            {
                if (erq->length <= IW_ESSID_MAX_SIZE){
                    if(copy_from_user(pAdapter->nickn, erq->pointer, erq->length)){
			Status = -EINVAL;
			break;
			}
                }else
                    Status = -E2BIG;
            }
            break;
        case SIOCGIWRATE:  //get default bit rate (bps)
            wrq->u.bitrate.value = RateIdToMbps[pAdapter->PortCfg.TxRate] * 1000000;
            wrq->u.bitrate.disabled = 0;
            break;
        case SIOCSIWRATE:  //set default bit rate (bps)
            RTMPSetDesiredRates(pAdapter, wrq->u.bitrate.value);
            break;
        case SIOCGIWRTS:  // get RTS/CTS threshold (bytes)
            wrq->u.rts.value = (INT) pAdapter->PortCfg.RtsThreshold;
            wrq->u.rts.disabled = (wrq->u.rts.value == MAX_RTS_THRESHOLD);
            wrq->u.rts.fixed = 1;
            break;
        case SIOCSIWRTS:  //set RTS/CTS threshold (bytes)
            RtsThresh = wrq->u.rts.value;
            if (wrq->u.rts.disabled)
                RtsThresh = MAX_RTS_THRESHOLD;

            if((RtsThresh > 0) && (RtsThresh <= MAX_RTS_THRESHOLD))
                pAdapter->PortCfg.RtsThreshold = (USHORT)RtsThresh;
            else if (RtsThresh == 0)
                pAdapter->PortCfg.RtsThreshold = MAX_RTS_THRESHOLD;

            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWRTS (=%d)\n", pAdapter->PortCfg.RtsThreshold);
            break;
        case SIOCGIWFRAG:  //get fragmentation thr (bytes)
            wrq->u.frag.value = (INT) pAdapter->PortCfg.FragmentThreshold;
            wrq->u.frag.disabled = (wrq->u.frag.value >= MAX_FRAG_THRESHOLD);
            wrq->u.frag.fixed = 1;
            break;
        case SIOCSIWFRAG:  //set fragmentation thr (bytes)
            FragThresh = wrq->u.frag.value;
            if (wrq->u.rts.disabled)
                FragThresh = MAX_FRAG_THRESHOLD;

            if ( (FragThresh >= MIN_FRAG_THRESHOLD) && (FragThresh <= MAX_FRAG_THRESHOLD))
                pAdapter->PortCfg.FragmentThreshold = (USHORT)FragThresh;
            else if (FragThresh == 0)
                pAdapter->PortCfg.FragmentThreshold = MAX_FRAG_THRESHOLD;

            if (pAdapter->PortCfg.FragmentThreshold == MAX_FRAG_THRESHOLD)
                pAdapter->PortCfg.bFragmentZeroDisable = TRUE;
            else
                pAdapter->PortCfg.bFragmentZeroDisable = FALSE;

            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWFRAG (=%d)\n", pAdapter->PortCfg.FragmentThreshold);
            break;
        case SIOCGIWENCODE:  //get encoding token & mode
            index = (wrq->u.encoding.flags & IW_ENCODE_INDEX) - 1;
            if ((index < 0) || (index >= NR_WEP_KEYS))
                index = pAdapter->PortCfg.DefaultKeyId; // Default key for tx (shared key)

            if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeOpen)
                wrq->u.encoding.flags = IW_ENCODE_OPEN;
            else if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeShared)
                wrq->u.encoding.flags = IW_ENCODE_RESTRICTED;

            if (pAdapter->PortCfg.WepStatus == Ndis802_11WEPDisabled)
                wrq->u.encoding.flags |= IW_ENCODE_DISABLED;
            else
            {
                if(wrq->u.encoding.pointer)
                {
                    wrq->u.encoding.length = pAdapter->PortCfg.SharedKey[index].KeyLen;
                    if(copy_to_user(wrq->u.encoding.pointer, 
                                pAdapter->PortCfg.SharedKey[index].Key,
                                pAdapter->PortCfg.SharedKey[index].KeyLen))
			Status = -EFAULT;
                    wrq->u.encoding.flags |= (index + 1);
                }
            }
            break;
        case SIOCSIWENCODE:  //set encoding token & mode
            index = (wrq->u.encoding.flags & IW_ENCODE_INDEX) - 1;
            /* take the old default key if index is invalid */
            if((index < 0) || (index >= NR_WEP_KEYS))
                index = pAdapter->PortCfg.DefaultKeyId;     // Default key for tx (shared key)

            if(wrq->u.encoding.pointer)
            {
                len = wrq->u.encoding.length;
                if(len > WEP_LARGE_KEY_LEN)
                    len = WEP_LARGE_KEY_LEN;

                memset(pAdapter->PortCfg.SharedKey[index].Key, 0x00, MAX_LEN_OF_KEY);
                if(copy_from_user(pAdapter->PortCfg.SharedKey[index].Key, 
                                wrq->u.encoding.pointer, len)){
			Status = -EINVAL;
			break;
		}
                pAdapter->PortCfg.SharedKey[index].KeyLen = len <= WEP_SMALL_KEY_LEN ? WEP_SMALL_KEY_LEN : WEP_LARGE_KEY_LEN;
            }
            pAdapter->PortCfg.DefaultKeyId = (UCHAR) index;
            if (wrq->u.encoding.flags & IW_ENCODE_DISABLED)
                pAdapter->PortCfg.WepStatus = Ndis802_11WEPDisabled;
            else
                pAdapter->PortCfg.WepStatus = Ndis802_11WEPEnabled;

            if (wrq->u.encoding.flags & IW_ENCODE_RESTRICTED)
                pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeShared;
            if (wrq->u.encoding.flags & IW_ENCODE_OPEN) 
                pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeOpen;

            if(pAdapter->PortCfg.WepStatus == Ndis802_11WEPDisabled)
                pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeOpen;

//#ifdef RT2500_DBG
            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWENCODE Key[%x] => \n", index);
            for (i = 0; i < len; i++)
            {
                DBGPRINT(RT_DEBUG_TRACE, "%02x:", pAdapter->PortCfg.SharedKey[index].Key[i]);
                if (i%16 == 15)
                    DBGPRINT(RT_DEBUG_TRACE, "\n");
            }
            DBGPRINT(RT_DEBUG_TRACE, "\n");
//#endif
            break;
        case SIOCGIWAP:  //get access point MAC addresses
            if (INFRA_ON(pAdapter) || ADHOC_ON(pAdapter))
            {
                wrq->u.ap_addr.sa_family = ARPHRD_ETHER;
                memcpy(wrq->u.ap_addr.sa_data, &pAdapter->PortCfg.Bssid, ETH_ALEN);

                DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCGIWAP(=%02x:%02x:%02x:%02x:%02x:%02x)\n",
                pAdapter->PortCfg.Bssid.Octet[0], pAdapter->PortCfg.Bssid.Octet[1], pAdapter->PortCfg.Bssid.Octet[2],
                pAdapter->PortCfg.Bssid.Octet[3], pAdapter->PortCfg.Bssid.Octet[4], pAdapter->PortCfg.Bssid.Octet[5]);
            }
            else
            {
                DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCGIWAP(=EMPTY)\n");
                Status = -ENOTCONN;
            }
            break;
        case SIOCSIWAP:  //set access point MAC addresses
            memcpy(&Bssid, &wrq->u.ap_addr.sa_data, sizeof(NDIS_802_11_MAC_ADDRESS));

	    if(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_IN_USE))
		break;

            if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
            {
                MlmeRestartStateMachine(pAdapter);
                DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
            }

            // tell CNTL state machine to call NdisMSetInformationComplete() after completing
            // this request, because this request is initiated by NDIS.
            pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 

            MlmeEnqueue(&pAdapter->Mlme.Queue, 
                        MLME_CNTL_STATE_MACHINE, 
                        OID_802_11_BSSID, 
                        sizeof(NDIS_802_11_MAC_ADDRESS),
                        (VOID *)&Bssid);
            Status = NDIS_STATUS_SUCCESS;
            StateMachineTouched = TRUE;
            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWAP %02x:%02x:%02x:%02x:%02x:%02x\n",
                                    Bssid[0], Bssid[1], Bssid[2], Bssid[3], Bssid[4], Bssid[5]);
            break;
        case SIOCGIWMODE:  //get operation mode
            if (pAdapter->PortCfg.BssType == BSS_INDEP)
            {
                BssType = Ndis802_11IBSS;
                wrq->u.mode = IW_MODE_ADHOC;
            }
            else if (pAdapter->PortCfg.BssType == BSS_INFRA)
            {
                BssType = Ndis802_11Infrastructure;
                wrq->u.mode = IW_MODE_INFRA;
            }
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,4,20)) 
            else if (pAdapter->PortCfg.BssType == BSS_MONITOR)
            {
                BssType = Ndis802_11Monitor;
                wrq->u.mode = IW_MODE_MONITOR;
            }
#endif	    
	    else
            {
                BssType = Ndis802_11AutoUnknown;
                wrq->u.mode = IW_MODE_AUTO;
            }
            DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCGIWMODE(=%d)\n", BssType);
            break;
        case SIOCSIWMODE:  //set operation mode
            if(wrq->u.mode == IW_MODE_ADHOC)
            {
                if (pAdapter->PortCfg.BssType != BSS_INDEP)
                {
                    // Config has changed
                    pAdapter->bConfigChanged = TRUE;
                }
                pAdapter->PortCfg.BssType = BSS_INDEP;
                DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWMODE (AD-HOC)\n");
            }
            else if (wrq->u.mode == IW_MODE_INFRA)
            {
                if (pAdapter->PortCfg.BssType != BSS_INFRA)
                {
                    // Config has changed
                    pAdapter->bConfigChanged = TRUE;
                }
                pAdapter->PortCfg.BssType = BSS_INFRA;
                DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWMODE (INFRA)\n");
            }
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,4,20)) 	    
	    else if (wrq->u.mode == IW_MODE_MONITOR)
            {
                if (pAdapter->PortCfg.BssType != BSS_MONITOR)
                {
                    // COnfig has changed
                    pAdapter->bConfigChanged = TRUE;
                }
                pAdapter->PortCfg.BssType = BSS_MONITOR;
                DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWMODE (MONITOR)\n");
            }
#endif 
  	    else
            {
                Status  = -EINVAL;
                DBGPRINT(RT_DEBUG_TRACE, "ioctl::SIOCSIWMODE (unknown)\n");
            }

            if (pAdapter->bConfigChanged == TRUE)
            {
                if (pAdapter->PortCfg.BssType == BSS_MONITOR)
                {
                    pAdapter->net_dev->type = 801;
                    RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x4e);
                }
                else
                {
                    pAdapter->net_dev->type = 1; 
                    RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x7e);
                }
            }
            // Reset Ralink supplicant to not use, it will be set to start when UI set PMK key
            pAdapter->PortCfg.WpaState = SS_NOTUSE;
            break;
        case SIOCGIWSENS:   //get sensitivity (dBm)
        case SIOCSIWSENS:   //set sensitivity (dBm)
        case SIOCGIWPOWER:  //get Power Management settings
        case SIOCSIWPOWER:  //set Power Management settings
        case SIOCGIWTXPOW:  //get transmit power (dBm)
        case SIOCSIWTXPOW:  //set transmit power (dBm)
        case SIOCGIWRETRY:  //get retry limits and lifetime
        case SIOCSIWRETRY:  //set retry limits and lifetime
	case 0x00008946:    // ethtool specific IOCTL (FIXME!, minimal support should not be difficult)
	case 0x00008947:    // mrtg related IOCTL (ignored for now)
            Status = -EOPNOTSUPP;
            break;
        case RT_PRIV_IOCTL:
            subcmd = wrq->u.data.flags;
            if( subcmd & OID_GET_SET_TOGGLE)
                Status = RTMPSetInformation(pAdapter, rq, subcmd);
            else
                Status = RTMPQueryInformation(pAdapter, rq, subcmd);
            break;
        case SIOCGIWPRIV:
            if (wrq->u.data.pointer) {
                if ( !access_ok(VERIFY_WRITE, wrq->u.data.pointer, sizeof(privtab)) )
                    break;
                wrq->u.data.length = sizeof(privtab) / sizeof(privtab[0]);
                if (copy_to_user(wrq->u.data.pointer, privtab, sizeof(privtab)))
                    Status = -EFAULT;
            }
            break;

        case RTPRIV_IOCTL_SET:
            {               
                char *this_char;
                char *value;

                if( !access_ok(VERIFY_READ, wrq->u.data.pointer, wrq->u.data.length) )
                    break;

                while ((this_char = strsep((char**)&wrq->u.data.pointer, ",")) != NULL) 
                {
                    if (!*this_char)
                         continue;

                    if ((value = rtstrchr(this_char, '=')) != NULL)
                        *value++ = 0;

                    if (!value || !*value)
                        continue;

                    for (PRTMP_PRIVATE_SET_PROC = RTMP_PRIVATE_SUPPORT_PROC; PRTMP_PRIVATE_SET_PROC->name; PRTMP_PRIVATE_SET_PROC++)
                    {
                        if (strcmp(this_char, PRTMP_PRIVATE_SET_PROC->name) == 0) 
                        {                       
                            if(!PRTMP_PRIVATE_SET_PROC->set_proc(pAdapter, value))
                            {   //FALSE:Set private failed then return Invalid argument
                                Status = -EINVAL;
                            }
                            break;  //Exit for loop.
                        }
                    }

                    if(PRTMP_PRIVATE_SET_PROC->name == NULL)
                    {  //Not found argument
                        Status = -EINVAL;
                        DBGPRINT(RT_DEBUG_TRACE, "ioctl::(iwpriv) Not Support Set Command [%s=%s]\n", this_char, value);
                        break;
                    }
                }
            }
            break;

        case RTPRIV_IOCTL_BBP:
            RTMPIoctlBBP(pAdapter, wrq);
            break;

        case RTPRIV_IOCTL_MAC:
            RTMPIoctlMAC(pAdapter, wrq);
            break;

#ifdef RALINK_ATE
        case RTPRIV_IOCTL_E2P:
            RTMPIoctlE2PROM(pAdapter, wrq);
            break;
#endif

        default:
            DBGPRINT(RT_DEBUG_TRACE, "IOCTL::unknown IOCTL's cmd = 0x%08x\n", cmd);
            Status = -EOPNOTSUPP;
            break;
    }

    if(StateMachineTouched) // Upper layer sent a MLME-related operations
        MlmeHandler(pAdapter);

    return Status;
}


UCHAR   BCAST[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
/*
    ========================================================================
    
    Routine Description:
        Add WPA key process

    Arguments:
        pAdapter                        Pointer to our adapter
        pBuf                            Pointer to the where the key stored

    Return Value:
        NDIS_SUCCESS                    Add key successfully

    Note:
        
    ========================================================================
*/
NDIS_STATUS RTMPWPAAddKeyProc(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuf)
{
    PNDIS_802_11_KEY    pKey;
    ULONG               KeyIdx;
    NDIS_STATUS         Status;
    PUCHAR              pTxMic, pRxMic;
    BOOLEAN             bTxKey;         // Set the key as transmit key
    BOOLEAN             bPairwise;      // Indicate the key is pairwise key
    BOOLEAN             bKeyRSC;        // indicate the receive  SC set by KeyRSC value.
                                        // Otherwise, it will set by the NIC.
    BOOLEAN             bAuthenticator; // indicate key is set by authenticator.
    INT                 i, PairwiseIdx;

    pKey = (PNDIS_802_11_KEY) pBuf;
    KeyIdx = pKey->KeyIndex & 0xff;
    // Bit 31 of Add-key, Tx Key
    bTxKey         = (pKey->KeyIndex & 0x80000000) ? TRUE : FALSE;
    // Bit 30 of Add-key PairwiseKey
    bPairwise      = (pKey->KeyIndex & 0x40000000) ? TRUE : FALSE;
    // Bit 29 of Add-key KeyRSC
    bKeyRSC        = (pKey->KeyIndex & 0x20000000) ? TRUE : FALSE;
    // Bit 28 of Add-key Authenticator
    bAuthenticator = (pKey->KeyIndex & 0x10000000) ? TRUE : FALSE;

    // 1. Check Group / Pairwise Key
    if (bPairwise)  // Pairwise Key
    {
        // 1. KeyIdx must be 0, otherwise, return NDIS_STATUS_INVALID_DATA
        if (KeyIdx != 0)
            return(NDIS_STATUS_FAILURE);
        
        // 2. Check bTx, it must be true, otherwise, return NDIS_STATUS_INVALID_DATA
        if (bTxKey == FALSE)
            return(NDIS_STATUS_FAILURE);

        // 3. If BSSID is not all 0xff, return NDIS_STATUS_INVALID_DATA
        if (NdisEqualMemory(pKey->BSSID, BCAST, 6))
            return(NDIS_STATUS_FAILURE);
            
        // 4. Selct RxMic / TxMic based on Supp / Authenticator
        if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
        {
            // for WPA-None Tx, Rx MIC is the same
            pTxMic = (PUCHAR) (&pKey->KeyMaterial) + 16;
            pRxMic = pTxMic;
        }
        else if (bAuthenticator == TRUE)
        {
            pTxMic = (PUCHAR) (&pKey->KeyMaterial) + 16;
            pRxMic = (PUCHAR) (&pKey->KeyMaterial) + 24;
        }
        else
        {
            pRxMic = (PUCHAR) (&pKey->KeyMaterial) + 16;
            pTxMic = (PUCHAR) (&pKey->KeyMaterial) + 24;
        }

        // 5. Find the old entry to overwrite or find an empty entry.
        PairwiseIdx = 0;
        for (i = 0; i < PAIRWISE_KEY_NO; i++)
        {
            if (pAdapter->PortCfg.PairwiseKey[i].KeyLen == 0)
            {
                PairwiseIdx = i;
                break;
            }
            else if (RTMPEqualMemory(pAdapter->PortCfg.PairwiseKey[i].BssId, pKey->BSSID, 6))
            {
                // Found the old entry
                PairwiseIdx = i;
                break;
            }
        }
        // If there is no match and no empty pairwise key, we have to replace an old one
        // which will be index 0 in our case.

        // 6. Check RxTsc
        if (bKeyRSC == TRUE)
        {
            memcpy(&pAdapter->PortCfg.PairwiseKey[PairwiseIdx].RxTsc, &pKey->KeyRSC, 6);            
        }
        else
        {
            memset(pAdapter->PortCfg.PairwiseKey[PairwiseIdx].RxTsc, 0, 6);        
        }

        // 7. Copy information into Pairwise Key structure.
        // pKey->KeyLength will include TxMic and RxMic, therefore, we use 16 bytes hardcoded.
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].KeyLen = 16;     
        memcpy(pAdapter->PortCfg.PairwiseKey[PairwiseIdx].Key, &pKey->KeyMaterial, 16);
        memcpy(pAdapter->PortCfg.PairwiseKey[PairwiseIdx].RxMic, pRxMic, 8);
        memcpy(pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxMic, pTxMic, 8);
        memcpy(pAdapter->PortCfg.PairwiseKey[PairwiseIdx].BssId, pKey->BSSID, 6);
        // Init TxTsc to one based on WiFi WPA specs
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxTsc[0] = 1;
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxTsc[1] = 0;
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxTsc[2] = 0;
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxTsc[3] = 0;
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxTsc[4] = 0;
        pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxTsc[5] = 0;
        Status = NDIS_STATUS_SUCCESS;

        DBGPRINT(RT_DEBUG_INFO, "TKIP Key = ");
        for (i = 0; i < 16; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.PairwiseKey[PairwiseIdx].Key[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "TKIP Rx MIC Key = ");
        for (i = 0; i < 8; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.PairwiseKey[PairwiseIdx].RxMic[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "TKIP Tx MIC Key = ");
        for (i = 0; i < 8; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.PairwiseKey[PairwiseIdx].TxMic[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "TKIP RxTSC = ");
        for (i = 0; i < 6; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.PairwiseKey[PairwiseIdx].RxTsc[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "BSSID:%02x:%02x:%02x:%02x:%02x:%02x \n",
            pKey->BSSID[0],pKey->BSSID[1],pKey->BSSID[2],pKey->BSSID[3],pKey->BSSID[4],pKey->BSSID[5]);

    }
    else    // Group Key
    {
        // 1. Check BSSID, if not current BSSID or Bcast, return NDIS_STATUS_INVALID_DATA
        if ((!NdisEqualMemory(&pKey->BSSID, &BCAST, 6)) &&
            (!NdisEqualMemory(&pKey->BSSID, &pAdapter->PortCfg.Bssid, 6)))
            return(NDIS_STATUS_FAILURE);


        // 2. Check Key index for supported Group Key
        if (KeyIdx >= GROUP_KEY_NO)
            return(NDIS_STATUS_FAILURE);

        // 3. Set as default Tx Key if bTxKey is TRUE
        if (bTxKey == TRUE)
            pAdapter->PortCfg.DefaultKeyId = (UCHAR) KeyIdx;

        // 4. Selct RxMic / TxMic based on Supp / Authenticator
        if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
        {
            // for WPA-None Tx, Rx MIC is the same
            pTxMic = (PUCHAR) (&pKey->KeyMaterial) + 16;
            pRxMic = pTxMic;
        }
        else if (bAuthenticator == TRUE)
        {
            pTxMic = (PUCHAR) (&pKey->KeyMaterial) + 16;
            pRxMic = (PUCHAR) (&pKey->KeyMaterial) + 24;
        }
        else
        {
            pRxMic = (PUCHAR) (&pKey->KeyMaterial) + 16;
            pTxMic = (PUCHAR) (&pKey->KeyMaterial) + 24;
        }

        // 5. Check RxTsc
        if (bKeyRSC == TRUE)
        {
            memcpy(pAdapter->PortCfg.GroupKey[KeyIdx].RxTsc, &pKey->KeyRSC, 6);
        }
        else
        {
            memset(pAdapter->PortCfg.GroupKey[KeyIdx].RxTsc, 0, 6);
        }

        // 6. Copy information into Group Key structure.
        // pKey->KeyLength will include TxMic and RxMic, therefore, we use 16 bytes hardcoded.
        pAdapter->PortCfg.GroupKey[KeyIdx].KeyLen = 16;     
        memcpy(pAdapter->PortCfg.GroupKey[KeyIdx].Key, &pKey->KeyMaterial, 16);
        memcpy(pAdapter->PortCfg.GroupKey[KeyIdx].RxMic, pRxMic, 8);
        memcpy(pAdapter->PortCfg.GroupKey[KeyIdx].TxMic, pTxMic, 8);
        memcpy(pAdapter->PortCfg.GroupKey[KeyIdx].BssId, pKey->BSSID, 6);
        // Init TxTsc to one based on WiFi WPA specs
        pAdapter->PortCfg.GroupKey[KeyIdx].TxTsc[0] = 1;
        pAdapter->PortCfg.GroupKey[KeyIdx].TxTsc[1] = 0;
        pAdapter->PortCfg.GroupKey[KeyIdx].TxTsc[2] = 0;
        pAdapter->PortCfg.GroupKey[KeyIdx].TxTsc[3] = 0;
        pAdapter->PortCfg.GroupKey[KeyIdx].TxTsc[4] = 0;
        pAdapter->PortCfg.GroupKey[KeyIdx].TxTsc[5] = 0;
        // 802.1x port control
        pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_SECURED;
        Status = NDIS_STATUS_SUCCESS;

        // For WEP compatibility, in case it use OID_ADD_KEY, not OID_ADD_WEP
        if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
        {
            pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen = (UCHAR) pKey->KeyLength;
            memcpy(pAdapter->PortCfg.SharedKey[KeyIdx].Key, &pKey->KeyMaterial, pKey->KeyLength);
        }

        DBGPRINT(RT_DEBUG_INFO, "TKIP Key = ");
        for (i = 0; i < 16; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.GroupKey[KeyIdx].Key[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "TKIP Rx MIC Key = ");
        for (i = 0; i < 8; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.GroupKey[KeyIdx].RxMic[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "TKIP Tx MIC Key = ");
        for (i = 0; i < 8; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.GroupKey[KeyIdx].TxMic[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "TKIP RxTSC = ");
        for (i = 0; i < 6; i++)
        {
            DBGPRINT(RT_DEBUG_INFO, "%02x:", pAdapter->PortCfg.GroupKey[KeyIdx].RxTsc[i]);
        }
        DBGPRINT(RT_DEBUG_INFO, "\n");
        DBGPRINT(RT_DEBUG_INFO, "BSSID:%02x:%02x:%02x:%02x:%02x:%02x \n",
            pKey->BSSID[0],pKey->BSSID[1],pKey->BSSID[2],pKey->BSSID[3],pKey->BSSID[4],pKey->BSSID[5]);

    }
    return (Status);
}

/*
    ========================================================================

    Routine Description:
        Remove WPA Key process

    Arguments:
        pAdapter                        Pointer to our adapter
        pBuf                            Pointer to the where the key stored

    Return Value:
        NDIS_SUCCESS                    Add key successfully

    Note:

    ========================================================================
*/
NDIS_STATUS RTMPWPARemoveKeyProc(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  PVOID           pBuf)
{
    PNDIS_802_11_REMOVE_KEY pKey;
    ULONG                   KeyIdx;
    NDIS_STATUS             Status = NDIS_STATUS_FAILURE;
    BOOLEAN                 bTxKey;         // Set the key as transmit key
    BOOLEAN                 bPairwise;      // Indicate the key is pairwise key
    BOOLEAN                 bKeyRSC;        // indicate the receive  SC set by KeyRSC value.
                                            // Otherwise, it will set by the NIC.
    BOOLEAN                 bAuthenticator; // indicate key is set by authenticator.
    INT                     i;

    pKey = (PNDIS_802_11_REMOVE_KEY) pBuf;
    KeyIdx = pKey->KeyIndex & 0xff;
    // Bit 31 of Add-key, Tx Key
    bTxKey         = (pKey->KeyIndex & 0x80000000) ? TRUE : FALSE;
    // Bit 30 of Add-key PairwiseKey
    bPairwise      = (pKey->KeyIndex & 0x40000000) ? TRUE : FALSE;
    // Bit 29 of Add-key KeyRSC
    bKeyRSC        = (pKey->KeyIndex & 0x20000000) ? TRUE : FALSE;
    // Bit 28 of Add-key Authenticator
    bAuthenticator = (pKey->KeyIndex & 0x10000000) ? TRUE : FALSE;

    // 1. If bTx is TRUE, return failure information
    if (bTxKey == TRUE)
        return(NDIS_STATUS_FAILURE);

    // 2. Check Pairwise Key
    if (bPairwise)
    {
        // a. If BSSID is broadcast, remove all pairwise keys.
        if (NdisEqualMemory(&pKey->BSSID, &BCAST, 6))
        {
            for (i = 0; i < PAIRWISE_KEY_NO; i++)
            {
                pAdapter->PortCfg.PairwiseKey[i].KeyLen = 0;
            }
            Status = NDIS_STATUS_SUCCESS;
        }

        // b. If not broadcast, remove the pairwise specified by BSSID
        else
        {
            for (i = 0; i < PAIRWISE_KEY_NO; i++)
            {
                if (NdisEqualMemory(pAdapter->PortCfg.PairwiseKey[i].BssId, pKey->BSSID, 6))
                {
                    pAdapter->PortCfg.PairwiseKey[i].KeyLen = 0;
                    Status = NDIS_STATUS_SUCCESS;
                    break;
                }
            }
            
        }
        // c. If no pairwise supported, delete Group Key 0.
        //    The will be false since we do support pairwise keys.
    }
    // 3. Group Key
    else
    {
        // a. If BSSID is broadcast, remove all group keys indexed
        if (NdisEqualMemory(&pKey->BSSID, &BCAST, 6))
        {
            pAdapter->PortCfg.GroupKey[KeyIdx].KeyLen = 0;
            Status = NDIS_STATUS_SUCCESS;
        }

        // b. If BSSID matched, delte the group key indexed.
        else if (NdisEqualMemory(pAdapter->PortCfg.GroupKey[KeyIdx].BssId, pKey->BSSID, 6))
        {
            pAdapter->PortCfg.GroupKey[KeyIdx].KeyLen = 0;
            Status = NDIS_STATUS_SUCCESS;
        }

        // c. For WEP compatibility
        if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
        {
            pAdapter->PortCfg.SharedKey[KeyIdx].KeyLen = 0;
        }
    }

    return (Status);
}


/*
    ========================================================================

    Routine Description:
        Remove All WPA Keys

    Arguments:
        pAdapter                        Pointer to our adapter

    Return Value:
        None

    Note:
        
    ========================================================================
*/
VOID    RTMPWPARemoveAllKeys(
    IN  PRTMP_ADAPTER   pAdapter)
{
    INT i;

    // For WPA-None, there is no need to remove it, since WinXP won't set it again after
    // Link up. And it will be replaced if user changed it.
    if (pAdapter->PortCfg.AuthMode == Ndis802_11AuthModeWPANone)
        return;

    for (i = 0; i < PAIRWISE_KEY_NO; i++)
    {
        pAdapter->PortCfg.PairwiseKey[i].KeyLen = 0;
    }
    
    for (i = 0; i < GROUP_KEY_NO; i++)
    {
        pAdapter->PortCfg.GroupKey[i].KeyLen = 0;
    }
}

/*
    ========================================================================

    Routine Description:
        Change NIC PHY mode. Re-association may be necessary.

    Arguments:
        pAdapter                        Pointer to our adapter
        phmode
        
    ========================================================================
*/
VOID    RTMPSetPhyMode(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  ULONG phymode)
{
    INT     i;
    
    DBGPRINT(RT_DEBUG_TRACE,"RTMPSetPhyMode(=%d)\n", phymode);

    // the selected phymode must be supported by the RF IC encoded in E2PROM
    if (pAdapter->PortCfg.RfType < RFIC_5222)
    {
        if (phymode == PHY_11A)
            phymode = PHY_11BG_MIXED;
    }

    // if no change, do nothing
    if (pAdapter->PortCfg.PhyMode == phymode)
        return;

    pAdapter->PortCfg.PhyMode = (UCHAR)phymode;
    BuildChannelList(pAdapter);

    for (i = 0; i < pAdapter->PortCfg.ChannelListNum; i++)
	{
		if (pAdapter->PortCfg.IbssConfig.Channel == pAdapter->PortCfg.ChannelList[i])
			break;
	}
	if (i == pAdapter->PortCfg.ChannelListNum)
		pAdapter->PortCfg.IbssConfig.Channel = FirstChannel(pAdapter);
    pAdapter->PortCfg.Channel = pAdapter->PortCfg.IbssConfig.Channel;
	
    AsicSwitchChannel(pAdapter, pAdapter->PortCfg.Channel);
    AsicLockChannel(pAdapter, pAdapter->PortCfg.Channel);

    switch (phymode) {
        case PHY_11B:
            pAdapter->PortCfg.IbssConfig.SupportedRates[0]  = 0x82;    // 1 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[1]  = 0x84;    // 2 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[2]  = 0x8B;    // 5.5 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[3]  = 0x96;    // 11 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRatesLen  = 4;
            pAdapter->PortCfg.SupportedRates[0]  = 0x82;    // 1 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[1]  = 0x84;    // 2 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[2]  = 0x8B;    // 5.5 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[3]  = 0x96;    // 11 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRatesLen  = 4;
            pAdapter->PortCfg.DesiredRates[0]  = 2;     // 1 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[1]  = 4;     // 2 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[2]  = 11;    // 5.5 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[3]  = 22;    // 11 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[4]  = 0;
            pAdapter->PortCfg.DesiredRates[5]  = 0;
            pAdapter->PortCfg.DesiredRates[6]  = 0;
            pAdapter->PortCfg.DesiredRates[7]  = 0;
            pAdapter->PortCfg.DesiredRates[8]  = 0;
            pAdapter->PortCfg.DesiredRates[9]  = 0;
            pAdapter->PortCfg.DesiredRates[10] = 0;
            pAdapter->PortCfg.DesiredRates[11] = 0;
            break;

        case PHY_11BG_MIXED:
        case PHY_11ABG_MIXED:
            pAdapter->PortCfg.IbssConfig.SupportedRates[0]  = 0x82;    // 1 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[1]  = 0x84;    // 2 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[2]  = 0x8B;    // 5.5 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[3]  = 0x96;    // 11 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[4]  = 0x12;    // 9 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[5]  = 0x24;    // 18 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[6]  = 0x48;    // 36 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[7]  = 0x6c;    // 54 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[8]  = 0x8C;    // 6 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[9]  = 0x98;    // 12 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[10] = 0xb0;    // 24 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[11] = 0x60;    // 48 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRatesLen  = 12;
            pAdapter->PortCfg.SupportedRates[0]  = 0x82;    // 1 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[1]  = 0x84;    // 2 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[2]  = 0x8B;    // 5.5 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[3]  = 0x96;    // 11 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[4]  = 0x8C;    // 6 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[5]  = 0x12;    // 9 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[6]  = 0x98;    // 12 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[7]  = 0x24;    // 18 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[8]  = 0xb0;    // 24 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[9]  = 0x48;    // 36 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[10] = 0x60;    // 48 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[11] = 0x6c;    // 54 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRatesLen  = 12;
            pAdapter->PortCfg.DesiredRates[0]  = 2;     // 1 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[1]  = 4;     // 2 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[2]  = 11;    // 5.5 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[3]  = 22;    // 11 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[4]  = 12;    // 6 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[5]  = 18;    // 9 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[6]  = 24;    // 12 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[7]  = 36;    // 18 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[8]  = 48;    // 24 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[9]  = 72;    // 36 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[10] = 96;    // 48 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[11] = 108;   // 54 mbps, in units of 0.5 Mbps
            break;

        case PHY_11A:
            pAdapter->PortCfg.IbssConfig.SupportedRates[0]  = 0x8C;    // 6 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[1]  = 0x12;    // 9 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[2]  = 0x98;    // 12 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[3]  = 0x24;    // 18 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[4]  = 0xb0;    // 24 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.IbssConfig.SupportedRates[5]  = 0x48;    // 36 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[6]  = 0x60;    // 48 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[7]  = 0x6c;    // 54 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.IbssConfig.SupportedRates[8]  = 0;
            pAdapter->PortCfg.IbssConfig.SupportedRates[9]  = 0;
            pAdapter->PortCfg.IbssConfig.SupportedRates[10] = 0;
            pAdapter->PortCfg.IbssConfig.SupportedRates[11] = 0;
            pAdapter->PortCfg.IbssConfig.SupportedRatesLen  = 8;
            pAdapter->PortCfg.SupportedRates[0]  = 0x8C;    // 6 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[1]  = 0x12;    // 9 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[2]  = 0x98;    // 12 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[3]  = 0x24;    // 18 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[4]  = 0xb0;    // 24 mbps, in units of 0.5 Mbps, basic rate
            pAdapter->PortCfg.SupportedRates[5]  = 0x48;    // 36 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[6]  = 0x60;    // 48 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[7]  = 0x6c;    // 54 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.SupportedRates[8]  = 0;
            pAdapter->PortCfg.SupportedRates[9]  = 0;
            pAdapter->PortCfg.SupportedRates[10] = 0;
            pAdapter->PortCfg.SupportedRates[11] = 0;
            pAdapter->PortCfg.SupportedRatesLen  = 8;
            pAdapter->PortCfg.DesiredRates[0]  = 12;    // 6 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[1]  = 18;    // 9 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[2]  = 24;    // 12 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[3]  = 36;    // 18 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[4]  = 48;    // 24 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[5]  = 72;    // 36 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[6]  = 96;    // 48 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[7]  = 108;   // 54 mbps, in units of 0.5 Mbps
            pAdapter->PortCfg.DesiredRates[8]  = 0;
            pAdapter->PortCfg.DesiredRates[9]  = 0;
            pAdapter->PortCfg.DesiredRates[10] = 0;
            pAdapter->PortCfg.DesiredRates[11] = 0;
            break;

        default:
            break;
    }

    MlmeUpdateTxRates(pAdapter, FALSE);
    AsicSetSlotTime(pAdapter, FALSE);
    MakeIbssBeacon(pAdapter);    // supported rates may change
}

VOID    RTMPSetDesiredRates(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  LONG            Rates)
{
    NDIS_802_11_RATES aryRates;

    memset(&aryRates, 0x00, sizeof(NDIS_802_11_RATES));
    switch (pAdapter->PortCfg.PhyMode)
    {
        case PHY_11A: // A only
            switch (Rates)
            {
                case 6000000: //6M
                    aryRates[0] = 0x0c; // 6M
                    break;
                case 9000000: //9M
                    aryRates[0] = 0x12; // 9M
                    break;
                case 12000000: //12M
                    aryRates[0] = 0x18; // 12M
                    break;
                case 18000000: //18M
                    aryRates[0] = 0x24; // 18M
                    break;
                case 24000000: //24M
                    aryRates[0] = 0x30; // 24M
                    break;
                case 36000000: //36M
                    aryRates[0] = 0x48; // 36M
                    break;
                case 48000000: //48M
                    aryRates[0] = 0x60; // 48M
                    break;
                case 54000000: //54M
                    aryRates[0] = 0x6c; // 54M
                    break;
                case -1: //Auto
                default:
                    aryRates[0] = 0x6c; // 54Mbps
                    aryRates[1] = 0x60; // 48Mbps
                    aryRates[2] = 0x48; // 36Mbps
                    aryRates[3] = 0x30; // 24Mbps
                    aryRates[4] = 0x24; // 18M
                    aryRates[5] = 0x18; // 12M
                    aryRates[6] = 0x12; // 9M
                    aryRates[7] = 0x0c; // 6M
                    break;
            }
            break;
        case PHY_11BG_MIXED: // B/G Mixed
        case PHY_11B: // B only
        case PHY_11ABG_MIXED: // A/B/G Mixed
        default:
            switch (Rates)
            {
                case 1000000: //1M
                    aryRates[0] = 0x02;
                    break;
                case 2000000: //2M
                    aryRates[0] = 0x04;
                    break;
                case 5000000: //5.5M
                    aryRates[0] = 0x0b; // 5.5M
                    break;
                case 11000000: //11M
                    aryRates[0] = 0x16; // 11M
                    break;
                case 6000000: //6M
                    aryRates[0] = 0x0c; // 6M
                    break;
                case 9000000: //9M
                    aryRates[0] = 0x12; // 9M
                    break;
                case 12000000: //12M
                    aryRates[0] = 0x18; // 12M
                    break;
                case 18000000: //18M
                    aryRates[0] = 0x24; // 18M
                    break;
                case 24000000: //24M
                    aryRates[0] = 0x30; // 24M
                    break;
                case 36000000: //36M
                    aryRates[0] = 0x48; // 36M
                    break;
                case 48000000: //48M
                    aryRates[0] = 0x60; // 48M
                    break;
                case 54000000: //54M
                    aryRates[0] = 0x6c; // 54M
                    break;
                case -1: //Auto
                default:
                    if (pAdapter->PortCfg.PhyMode == PHY_11B)
                    { //B Only
                        aryRates[0] = 0x16; // 11Mbps
                        aryRates[1] = 0x0b; // 5.5Mbps
                        aryRates[2] = 0x04; // 2Mbps
                        aryRates[3] = 0x02; // 1Mbps
                    }
                    else
                    { //(B/G) Mixed or (A/B/G) Mixed
                        aryRates[0] = 0x6c; // 54Mbps
                        aryRates[1] = 0x60; // 48Mbps
                        aryRates[2] = 0x48; // 36Mbps
                        aryRates[3] = 0x30; // 24Mbps
                        aryRates[4] = 0x16; // 11Mbps
                        aryRates[5] = 0x0b; // 5.5Mbps
                        aryRates[6] = 0x04; // 2Mbps
                        aryRates[7] = 0x02; // 1Mbps
                    }
                    break;
            }
            break;
    }

    memset(pAdapter->PortCfg.DesiredRates, 0, MAX_LEN_OF_SUPPORTED_RATES);
    memcpy(pAdapter->PortCfg.DesiredRates, &aryRates, sizeof(NDIS_802_11_RATES));
    DBGPRINT(RT_DEBUG_TRACE, " RTMPSetDesiredRates (%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x)\n",
        pAdapter->PortCfg.DesiredRates[0],pAdapter->PortCfg.DesiredRates[1],
        pAdapter->PortCfg.DesiredRates[2],pAdapter->PortCfg.DesiredRates[3],
        pAdapter->PortCfg.DesiredRates[4],pAdapter->PortCfg.DesiredRates[5],
        pAdapter->PortCfg.DesiredRates[6],pAdapter->PortCfg.DesiredRates[7] );
    // Changing DesiredRate may affect the MAX TX rate we used to TX frames out
    MlmeUpdateTxRates(pAdapter, FALSE);
}
/* 
    ==========================================================================
    Description:
        Set Country Region
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_CountryRegion_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               region;
    int                                 success = TRUE;

    region = simple_strtol(arg, 0, 10);
    if( (region >= REGION_MIN) && (region <= REGION_MAX) )
    {
        pAdapter->PortCfg.CountryRegion = (UCHAR) region;
        DBGPRINT(RT_DEBUG_TRACE, "Set_CountryRegion_Proc::(CountryRegion=%d)\n", pAdapter->PortCfg.CountryRegion);
        DBGPRINT(RT_DEBUG_TRACE, "Set_CountryRegion_Proc::(CountryRegion=%d)\n", pAdapter->PortCfg.CountryRegion);
    }
    else
        success = FALSE;

    return success;
}
/* 
    ==========================================================================
    Description:
        Set SSID
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_SSID_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    NDIS_802_11_SSID                    Ssid, *pSsid=NULL;
    BOOLEAN                             StateMachineTouched = FALSE;
    int                                 success = TRUE;


    /* Protect against oops if net is down, this will not work with if-preup
     use iwconfig properly */   
    printk("'iwpriv <dev> set essid' is deprecated, please use 'iwconfg <dev> essid' instead\n"); 
    if(!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_IN_USE))
	return FALSE;

    if( strlen(arg) <= MAX_LEN_OF_SSID)
    {
        memset(&Ssid, 0, MAX_LEN_OF_SSID);
        memcpy(Ssid.Ssid, arg, strlen(arg));
        Ssid.SsidLength = strlen(arg);
        pSsid = &Ssid;

	
   	
        if (pAdapter->Mlme.CntlMachine.CurrState != CNTL_IDLE)
        {
            MlmeRestartStateMachine(pAdapter);
            DBGPRINT(RT_DEBUG_TRACE, "!!! MLME busy, reset MLME state machine !!!\n");
        }
         // tell CNTL state machine to call NdisMSetInformationComplete() after completing
        // this request, because this request is initiated by NDIS.
        pAdapter->Mlme.CntlAux.CurrReqIsFromNdis = FALSE; 

        MlmeEnqueue(&pAdapter->Mlme.Queue, 
                    MLME_CNTL_STATE_MACHINE, 
                    OID_802_11_SSID,
                    sizeof(NDIS_802_11_SSID),
                    (VOID *)pSsid);

        StateMachineTouched = TRUE;
        DBGPRINT(RT_DEBUG_TRACE, "Set_SSID_Proc::(Len=%d,Ssid=%s)\n", pAdapter->PortCfg.SsidLen, pAdapter->PortCfg.Ssid);
    }
    else
        success = FALSE;

    if (StateMachineTouched) // Upper layer sent a MLME-related operations
        MlmeHandler(pAdapter);

    return success;
}
/* 
    ==========================================================================
    Description:
        Set Wireless Mode
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_WirelessMode_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               WirelessMode;
    int                                 success = TRUE;

    WirelessMode = simple_strtol(arg, 0, 10);

    if ((WirelessMode == PHY_11BG_MIXED) || (WirelessMode == PHY_11B) ||
        (WirelessMode == PHY_11A) || (WirelessMode == PHY_11ABG_MIXED))
    {
        RTMPSetPhyMode(pAdapter, WirelessMode);
        DBGPRINT(RT_DEBUG_TRACE, "Set_WirelessMode_Proc::(=%d)\n", WirelessMode);
    }
    else
        success = FALSE;

    return success;
}
/* 
    ==========================================================================
    Description:
        Set TxRate
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_TxRate_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               TxRate;
    int                                 success = TRUE;
	ULONG	rate_mapping[12] = {1, 2, 5, 11, 6, 9, 12, 18, 24, 36, 48, 54}; //according to README

    TxRate = simple_strtol(arg, 0, 10);

    if (TxRate == 0)
        RTMPSetDesiredRates(pAdapter, -1);
    else
        RTMPSetDesiredRates(pAdapter, (LONG) (rate_mapping[TxRate-1] * 1000000));
    return success;
}
/* 
    ==========================================================================
    Description:
        Set AdhocMode support Rate can or can not exceed 11Mbps against WiFi spec.
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_AdhocModeRate_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG       AdhocMode;

    AdhocMode = simple_strtol(arg, 0, 10);

    if (AdhocMode == 1)
        pAdapter->PortCfg.AdhocMode = 1;
    else if (AdhocMode == 0)
        pAdapter->PortCfg.AdhocMode = 0;
    else
        return FALSE;  //Invalid argument 

    DBGPRINT(RT_DEBUG_TRACE, "Set_AdhocModeRate_Proc::(AdhocMode=%d)\n", pAdapter->PortCfg.AdhocMode);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set Channel
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_Channel_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    int                                 success = TRUE;
    UCHAR                               Channel;

    Channel = (UCHAR) simple_strtol(arg, 0, 10);

    if (ChannelSanity(pAdapter, Channel) == TRUE)
    {
        pAdapter->PortCfg.Channel = Channel;
	pAdapter->PortCfg.IbssConfig.Channel = Channel;
        DBGPRINT(RT_DEBUG_TRACE, "Set_Channel_Proc::(Channel=%d)\n", Channel);
    }
    else
        success = FALSE;

    return success;
}
/* 
    ==========================================================================
    Description:
        Set 11B/11G Protection
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_BGProtection_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)

{
    switch (simple_strtol(arg, 0, 10))
    {
        case 0: //AUTO
            pAdapter->PortCfg.UseBGProtection = 0;
            break;
        case 1: //Always On
            pAdapter->PortCfg.UseBGProtection = 1;
            break;
        case 2: //Always OFF
            pAdapter->PortCfg.UseBGProtection = 2;
            break;      
        default:  //Invalid argument 
            return FALSE;
    }
    DBGPRINT(RT_DEBUG_TRACE, "Set_BGProtection_Proc::(BGProtection=%d)\n", pAdapter->PortCfg.UseBGProtection);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set StaWithEtherBridge function on/off
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_StaWithEtherBridge_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)

{
    switch (simple_strtol(arg, 0, 10))
    {
        case 0: //Off
            pAdapter->PortCfg.StaWithEtherBridge.Enable = FALSE;
            break;
        case 1: //On
            pAdapter->PortCfg.StaWithEtherBridge.Enable = TRUE;
            break;
        default:  //Invalid argument 
            return FALSE;
    }
    DBGPRINT(RT_DEBUG_TRACE, "Set_StaWithEtherBridge_Proc::(StaWithEtherBridge=%d)\n", pAdapter->PortCfg.StaWithEtherBridge.Enable);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set TxPreamble
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_TxPreamble_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    RT_802_11_PREAMBLE                  Preamble;

    Preamble = simple_strtol(arg, 0, 10);
    switch (Preamble)
    {
        case Rt802_11PreambleShort:
            pAdapter->PortCfg.WindowsTxPreamble = Preamble;
            MlmeSetTxPreamble(pAdapter, Rt802_11PreambleShort);
            break;
        case Rt802_11PreambleLong:
        case Rt802_11PreambleAuto:
            // if user wants AUTO, initialize to LONG here, then change according to AP's
            // capability upon association.
            pAdapter->PortCfg.WindowsTxPreamble = Preamble;
            MlmeSetTxPreamble(pAdapter, Rt802_11PreambleLong);
            break;
        default: //Invalid argument 
            return FALSE;
    }

    DBGPRINT(RT_DEBUG_TRACE, "Set_TxPreamble_Proc::(TxPreamble=%d)\n", Preamble);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set RTS Threshold
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_RTSThreshold_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
    
   
{
    NDIS_802_11_RTS_THRESHOLD           RtsThresh;

   printk("'iwpriv <dev> set RTSThreshold' is deprecated, please use 'iwconfg <dev> rts' instead\n"); 
 
    RtsThresh = simple_strtol(arg, 0, 10);

    if((RtsThresh > 0) && (RtsThresh <= MAX_RTS_THRESHOLD))
        pAdapter->PortCfg.RtsThreshold = (USHORT)RtsThresh;
    else if (RtsThresh == 0)
        pAdapter->PortCfg.RtsThreshold = MAX_RTS_THRESHOLD;
    else
        return FALSE;

    DBGPRINT(RT_DEBUG_TRACE, "Set_RTSThreshold_Proc::(RTSThreshold=%d)\n", pAdapter->PortCfg.RtsThreshold);
    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set Fragment Threshold
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_FragThreshold_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    NDIS_802_11_FRAGMENTATION_THRESHOLD     FragThresh;

    printk("'iwpriv <dev> set FragThreshold' is deprecated, please use 'iwconfg <dev> frag' instead\n"); 
 
    
    FragThresh = simple_strtol(arg, 0, 10);

    if ( (FragThresh >= MIN_FRAG_THRESHOLD) && (FragThresh <= MAX_FRAG_THRESHOLD))
        pAdapter->PortCfg.FragmentThreshold = (USHORT)FragThresh;
    else if (FragThresh == 0)
        pAdapter->PortCfg.FragmentThreshold = MAX_FRAG_THRESHOLD;
    else
        return FALSE; //Invalid argument 

    if (pAdapter->PortCfg.FragmentThreshold == MAX_FRAG_THRESHOLD)
        pAdapter->PortCfg.bFragmentZeroDisable = TRUE;
    else
        pAdapter->PortCfg.bFragmentZeroDisable = FALSE;

    DBGPRINT(RT_DEBUG_TRACE, "Set_FragThreshold_Proc::(FragThreshold=%d)\n", FragThresh);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set TxBurst
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_TxBurst_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               TxBurst;

    TxBurst = simple_strtol(arg, 0, 10);

    if (TxBurst == 1)
        pAdapter->PortCfg.EnableTxBurst = TRUE;
    else if (TxBurst == 0)
        pAdapter->PortCfg.EnableTxBurst = FALSE;
    else
        return FALSE;  //Invalid argument 
    
    DBGPRINT(RT_DEBUG_TRACE, "Set_TxBurst_Proc::(TxBurst=%d)\n", pAdapter->PortCfg.EnableTxBurst);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set TurboRate Enable or Disable
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_TurboRate_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               TurboRate;

    TurboRate = simple_strtol(arg, 0, 10);

    if (TurboRate == 1)
        pAdapter->PortCfg.EnableTurboRate = TRUE;
    else if (TurboRate == 0)
        pAdapter->PortCfg.EnableTurboRate = FALSE;
    else
        return FALSE;  //Invalid argument 
    
    DBGPRINT(RT_DEBUG_TRACE, "Set_TurboRate_Proc::(TurboRate=%d)\n", pAdapter->PortCfg.EnableTurboRate);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set Short Slot Time Enable or Disable
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_ShortSlot_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               ShortSlot;

    ShortSlot = simple_strtol(arg, 0, 10);

    if (ShortSlot == 1)
        pAdapter->PortCfg.UseShortSlotTime = TRUE;
    else if (ShortSlot == 0)
        pAdapter->PortCfg.UseShortSlotTime = FALSE;
    else
        return FALSE;  //Invalid argument 

    DBGPRINT(RT_DEBUG_TRACE, "Set_ShortSlot_Proc::(ShortSlot=%d)\n", pAdapter->PortCfg.UseShortSlotTime);

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set Network Type(Infrastructure/Adhoc mode)
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_NetworkType_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{

    printk("'iwpriv <dev> set NetworkType' is deprecated, please use 'iwconfg <dev> mode' instead\n"); 
    
    if (strcmp(arg, "Adhoc") == 0)
        pAdapter->PortCfg.BssType = BSS_INDEP;
    else //Default Infrastructure mode
        pAdapter->PortCfg.BssType = BSS_INFRA;
    
    // Reset Ralink supplicant to not use, it will be set to start when UI set PMK key
    pAdapter->PortCfg.WpaState = SS_NOTUSE;

    DBGPRINT(RT_DEBUG_TRACE, "Set_NetworkType_Proc::(NetworkType=%d)\n", pAdapter->PortCfg.BssType);

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set Authentication mode
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_AuthMode_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    if ((strcmp(arg, "OPEN") == 0) || (strcmp(arg, "open") == 0))
        pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeOpen;
    else if ((strcmp(arg, "SHARED") == 0) || (strcmp(arg, "shared") == 0))
        pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeShared;
    else if ((strcmp(arg, "AUTO") == 0) || (strcmp(arg, "auto") == 0))
        pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeAutoSwitch;
    else if ((strcmp(arg, "WPAPSK") == 0) || (strcmp(arg, "wpapsk") == 0))
        pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeWPAPSK;
    else if ((strcmp(arg, "WPANONE") == 0) || (strcmp(arg, "wpanone") == 0))
        pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeWPANone;
    else
        return FALSE;  

    pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;

    DBGPRINT(RT_DEBUG_TRACE, "Set_AuthMode_Proc::(AuthMode=%d)\n", pAdapter->PortCfg.AuthMode);

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set Encryption Type
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_EncrypType_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    if ((strcmp(arg, "NONE") == 0) || (strcmp(arg, "none") == 0))
        pAdapter->PortCfg.WepStatus = Ndis802_11WEPDisabled;
    else if ((strcmp(arg, "WEP") == 0) || (strcmp(arg, "wep") == 0))
        pAdapter->PortCfg.WepStatus = Ndis802_11WEPEnabled;
    else if ((strcmp(arg, "TKIP") == 0) || (strcmp(arg, "tkip") == 0))
        pAdapter->PortCfg.WepStatus = Ndis802_11Encryption2Enabled;
    else if ((strcmp(arg, "AES") == 0) || (strcmp(arg, "aes") == 0))
        pAdapter->PortCfg.WepStatus = Ndis802_11Encryption3Enabled;
    else
        return FALSE;

    DBGPRINT(RT_DEBUG_TRACE, "Set_EncrypType_Proc::(EncrypType=%d)\n", pAdapter->PortCfg.WepStatus);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set Default Key ID
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_DefaultKeyID_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    ULONG                               KeyIdx;
    
    printk("'iwpriv <dev> set DefaultKeyID' is deprecated, please use 'iwconfg <dev> key' instead\n"); 
    
    KeyIdx = simple_strtol(arg, 0, 10);
    if((KeyIdx >= 1 ) && (KeyIdx <= 4))
        pAdapter->PortCfg.DefaultKeyId = (UCHAR) (KeyIdx - 1 );
    else
        return FALSE;  //Invalid argument 

    DBGPRINT(RT_DEBUG_TRACE, "Set_DefaultKeyID_Proc::(DefaultKeyID=%d)\n", pAdapter->PortCfg.DefaultKeyId);

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set WEP KEY1
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_Key1_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    int                                 KeyLen;
    int                                 i;

    printk("'iwpriv <dev> set Key1' is deprecated, please use 'iwconfg <dev> key [1] ' instead\n"); 
   
    KeyLen = strlen(arg);

    switch (KeyLen)
    {
        case 5: //wep 40 Ascii type
            pAdapter->PortCfg.SharedKey[0].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[0].Key, arg, KeyLen);    
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key1_Proc::(Key1=%s and type=%s)\n", arg, "Ascii");       
            break;
        case 10: //wep 40 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[0].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[0].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key1_Proc::(Key1=%s and type=%s)\n", arg, "Hex");     
            break;
        case 13: //wep 104 Ascii type
            pAdapter->PortCfg.SharedKey[0].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[0].Key, arg, KeyLen);    
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key1_Proc::(Key1=%s and type=%s)\n", arg, "Ascii");       
            break;
        case 26: //wep 104 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[0].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[0].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key1_Proc::(Key1=%s and type=%s)\n", arg, "Hex");     
            break;
        default: //Invalid argument 
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key1_Proc::Invalid argument (=%s)\n", arg);       
            return FALSE;
    }

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set WEP KEY2
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_Key2_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    int                                 KeyLen;
    int                                 i;

    printk("'iwpriv <dev> set Key2' is deprecated, please use 'iwconfg <dev> key [2] ' instead\n"); 

    
    KeyLen = strlen(arg);

    switch (KeyLen)
    {
        case 5: //wep 40 Ascii type
            pAdapter->PortCfg.SharedKey[1].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[1].Key, arg, KeyLen);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key2_Proc::(Key2=%s and type=%s)\n", arg, "Ascii");
            break;
        case 10: //wep 40 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[1].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[1].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key2_Proc::(Key2=%s and type=%s)\n", arg, "Hex");
            break;
        case 13: //wep 104 Ascii type
            pAdapter->PortCfg.SharedKey[1].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[1].Key, arg, KeyLen);    
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key2_Proc::(Key2=%s and type=%s)\n", arg, "Ascii");
            break;
        case 26: //wep 104 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[1].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[1].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key2_Proc::(Key2=%s and type=%s)\n", arg, "Hex");
            break;
        default: //Invalid argument 
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key2_Proc::Invalid argument (=%s)\n", arg);
            return FALSE;
    }

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set WEP KEY3
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_Key3_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    int                                 KeyLen;
    int                                 i;

     printk("'iwpriv <dev> set Key3' is deprecated, please use 'iwconfg <dev> key [3] ' instead\n"); 

    KeyLen = strlen(arg);

    switch (KeyLen)
    {
        case 5: //wep 40 Ascii type
            pAdapter->PortCfg.SharedKey[2].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[2].Key, arg, KeyLen);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key3_Proc::(Key3=%s and type=%s)\n", arg, "Ascii");
            break;
        case 10: //wep 40 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[2].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[2].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key3_Proc::(Key3=%s and type=%s)\n", arg, "Hex");
            break;
        case 13: //wep 104 Ascii type
            pAdapter->PortCfg.SharedKey[2].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[2].Key, arg, KeyLen);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key3_Proc::(Key3=%s and type=%s)\n", arg, "Ascii");
            break;
        case 26: //wep 104 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[2].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[2].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key3_Proc::(Key3=%s and type=%s)\n", arg, "Hex");
            break;
        default: //Invalid argument 
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key3_Proc::Invalid argument (=%s)\n", arg);
            return FALSE;
    }

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set WEP KEY4
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_Key4_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    int                                 KeyLen;
    int                                 i;

    printk("'iwpriv <dev> set Key4' is deprecated, please use 'iwconfg <dev> key [4] ' instead\n"); 
    
    KeyLen = strlen(arg);

    switch (KeyLen)
    {
        case 5: //wep 40 Ascii type
            pAdapter->PortCfg.SharedKey[3].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[3].Key, arg, KeyLen);    
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key4_Proc::(Key4=%s and type=%s)\n", arg, "Ascii");
            break;
        case 10: //wep 40 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[3].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[3].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key4_Proc::(Key4=%s and type=%s)\n", arg, "Hex");
            break;
        case 13: //wep 104 Ascii type
            pAdapter->PortCfg.SharedKey[3].KeyLen = KeyLen;
            memcpy(pAdapter->PortCfg.SharedKey[3].Key, arg, KeyLen);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key4_Proc::(Key4=%s and type=%s)\n", arg, "Ascii");
            break;
        case 26: //wep 104 Hex type
            for(i=0; i < KeyLen; i++)
            {
                if( !isxdigit(*(arg+i)) )
                    return FALSE;  //Not Hex value;
            }
            pAdapter->PortCfg.SharedKey[3].KeyLen = KeyLen / 2 ;
            AtoH(arg, pAdapter->PortCfg.SharedKey[3].Key, KeyLen / 2);
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key4_Proc::(Key4=%s and type=%s)\n", arg, "Hex");
            break;
        default: //Invalid argument 
            DBGPRINT(RT_DEBUG_TRACE, "Set_Key4_Proc::Invalid argument (=%s)\n", arg);
            return FALSE;
    }

    return TRUE;
}
/* 
    ==========================================================================
    Description:
        Set WPA PSK key
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT Set_WPAPSK_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    UCHAR                               keyMaterial[40];

    DBGPRINT(RT_DEBUG_TRACE, "Set_WPAPSK_Proc::(WPAPSK=%s)\n", arg);
    if ((strlen(arg) < 8) || (strlen(arg) > 64))
    {
        DBGPRINT(RT_DEBUG_TRACE, "Set failed!!(WPAPSK=%s), WPAPSK key-string required 8 ~ 64 characters \n", arg);
        return FALSE;
    }

    if (strlen(arg) == 64)
    {
        AtoH(arg, pAdapter->PortCfg.PskKey.Key, 32);
    }
    else
    {
        PasswordHash((char *)arg, pAdapter->Mlme.CntlAux.Ssid, pAdapter->Mlme.CntlAux.SsidLen, keyMaterial);

        memcpy(&pAdapter->PortCfg.PskKey.Key, &keyMaterial, 32);
    }
 
    // Use RaConfig as PSK agent.
    // Start STA supplicant state machine
    pAdapter->PortCfg.WpaState = SS_START;
 
    return TRUE;
}


/* 
    ==========================================================================
    Description:
        Set WPA NONE key
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/

INT Set_WPANONE_Proc(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  PUCHAR          arg)
{
    UCHAR               keyMaterial[40];

    DBGPRINT(RT_DEBUG_TRACE, "Set_WPANONE_Proc::(WPANONE=%s)\n", arg);
    if ((strlen(arg) < 8) || (strlen(arg) > 64))
    {
        DBGPRINT(RT_DEBUG_TRACE, "Set failed!!(WPANONE=%s), WPANONE key-string required 8 ~ 64 characters \n", arg);
        return FALSE;
    }
 
    if (strlen(arg) == 64)
    {
        AtoH(arg, pAdapter->PortCfg.PskKey.Key, 32);
    }
    else
    {
    	PasswordHash((char *)arg, pAdapter->Mlme.CntlAux.Ssid, pAdapter->Mlme.CntlAux.SsidLen, keyMaterial);

    	memcpy(pAdapter->PortCfg.PskKey.Key, keyMaterial, 32);
    }
    // Use RaConfig as PSK agent.
    // Start STA supplicant state machine
    pAdapter->PortCfg.WpaState = SS_START;

//-----------------------------------------------------------------------------
// pasted from "RTMPWPAAddKeyProc(...)"
// major on Group Key only.
 
    // Group Key
    {
        // 3. Set as default Tx Key if bTxKey is TRUE
        pAdapter->PortCfg.DefaultKeyId = 0;

        // 4. Selct RxMic / TxMic based on Supp / Authenticator
        // for WPA-None Tx, Rx MIC is the same
        //pTxMic = (PUCHAR) (keyMaterial) + 16;
        //pRxMic = pTxMic;

        memset(pAdapter->PortCfg.GroupKey[0].RxTsc, 0x00, 6);

        // 6. Copy information into Group Key structure.
        // pKey->KeyLength will include TxMic and RxMic, therefore, we use 16 bytes hardcoded.
        pAdapter->PortCfg.GroupKey[0].KeyLen = 16;     
        memcpy(pAdapter->PortCfg.GroupKey[0].Key,   (PUCHAR)(keyMaterial) +  0, 16);
        memcpy(pAdapter->PortCfg.GroupKey[0].RxMic, (PUCHAR)(keyMaterial) + 16, 8);
        memcpy(pAdapter->PortCfg.GroupKey[0].TxMic, (PUCHAR)(keyMaterial) + 16, 8);
        memcpy(pAdapter->PortCfg.GroupKey[0].BssId, &pAdapter->PortCfg.Bssid, 6);

        // Init TxTsc to one based on WiFi WPA specs
        pAdapter->PortCfg.GroupKey[0].TxTsc[0] = 1;
        pAdapter->PortCfg.GroupKey[0].TxTsc[1] = 0;
        pAdapter->PortCfg.GroupKey[0].TxTsc[2] = 0;
        pAdapter->PortCfg.GroupKey[0].TxTsc[3] = 0;
        pAdapter->PortCfg.GroupKey[0].TxTsc[4] = 0;
        pAdapter->PortCfg.GroupKey[0].TxTsc[5] = 0;

        // 802.1x port control
        pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_SECURED;
    }

    return TRUE;
}

/* 
    ==========================================================================
    Description:
        Read / Write BBP
Arguments:
    pAdapter                    Pointer to our adapter
    wrq                         Pointer to the ioctl argument

    Return Value:
        None

    Note:
        Usage: 
               1.) iwpriv ra0 bbp               ==> read all BBP
               2.) iwpriv ra0 bbp 1,2,10,32     ==> raed BBP where ID=1,2,10,32
               3.) iwpriv ra0 bbp 1=10,17=3E    ==> write BBP R1=0x10, R17=0x3E
    ==========================================================================
*/
VOID RTMPIoctlBBP(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  struct iwreq    *wrq)
{
    char                *this_char;
    char                *value;
    int                 i=0;
    int                 count = 0;
    UCHAR               regBBP;
    char                msg[1024];
    char                arg[255];
    char                *ptr;
    ULONG               bbpId;
    ULONG               bbpValue;
    BOOLEAN             bIsPrintAllBBP = FALSE;

    DBGPRINT(RT_DEBUG_TRACE, "==>RTMPIoctlBBP\n");
    memset(msg, 0x00, 1024);
    if (wrq->u.data.length > 1) //No parameters.
    {
        memcpy(arg, wrq->u.data.pointer, (wrq->u.data.length > 255) ? 255 : wrq->u.data.length);
        ptr = arg;
        sprintf(msg, "\n");
        //Parsing Read or Write
        while ((this_char = strsep(&ptr, ",")) != NULL)
        {
            i++;
            DBGPRINT(RT_DEBUG_TRACE, "this_char=%s\n", this_char);
            if (!*this_char)
                continue;

            if ((value = rtstrchr(this_char, '=')) != NULL)
                *value++ = 0;

            if (!value || !*value)
            { //Read
                DBGPRINT(RT_DEBUG_TRACE, "this_char=%s, value=%s\n", this_char, value);
                bbpId = simple_strtol(this_char, 0, 10);
                if ((bbpId >=0) && (bbpId <= 63))
                {
                    RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, bbpId, &regBBP);
                    sprintf(msg+strlen(msg), "R%02d[0x%02X]:%02X  ", bbpId, bbpId*2, regBBP);
                    count++;
                    if (count%5 == 4)
                        sprintf(msg+strlen(msg), "\n");
                    DBGPRINT(RT_DEBUG_TRACE, "msg=%s\n", msg);
                }
                else
                {//Invalid parametes, so default print all bbp
                    bIsPrintAllBBP = TRUE;
                    break;
                }
            }
            else
            { //Write
                DBGPRINT(RT_DEBUG_TRACE, "this_char=%s, value=%s\n", this_char, value);
                bbpId = simple_strtol(this_char, 0, 10);
                bbpValue = simple_strtol(value, 0, 10);
                DBGPRINT(RT_DEBUG_TRACE, "bbpID=%02d, value=0x%x\n", bbpId, bbpValue);
                if ((bbpId >=0) && (bbpId <= 63))
                {
                    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, (UCHAR) bbpId, (UCHAR) bbpValue);
                    //Read it back for showing
                    RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, bbpId, &regBBP);
                    sprintf(msg+strlen(msg), "R%02d[0x%02X]:%02X  ", bbpId, bbpId*2, regBBP);
                    count++;
                    if (count%5 == 4)
                        sprintf(msg+strlen(msg), "\n");
                    DBGPRINT(RT_DEBUG_TRACE, "msg=%s\n", msg);
                }
                else
                {//Invalid parametes, so default print all bbp
                    bIsPrintAllBBP = TRUE;
                    break;
                }
            }
        }
    }
    else
        bIsPrintAllBBP = TRUE;

    if (bIsPrintAllBBP)
    {
        memset(msg, 0x00, 1024);
        sprintf(msg, "\n");
        for (i = 0; i <= 63; i++)
        {
            RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, i, &regBBP);
            sprintf(msg+strlen(msg), "R%02d[0x%02X]:%02X  ", i, i*2, regBBP);
            if (i%5 == 4)
                sprintf(msg+strlen(msg), "\n");
        }
        // Copy the information into the user buffer
        wrq->u.data.length = strlen(msg);
        if(copy_to_user(wrq->u.data.pointer, msg, wrq->u.data.length))
		DBGPRINT(RT_DEBUG_ERROR, "RTMPIoctlBBP - copy to user failure\n");
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, "copy to user [msg=%s]\n", msg);
        // Copy the information into the user buffer
        wrq->u.data.length = strlen(msg);
        if(copy_to_user(wrq->u.data.pointer, msg, wrq->u.data.length))
		DBGPRINT(RT_DEBUG_ERROR, "RTMPIoctlBBP - copy to user failure\n");
    }
    DBGPRINT(RT_DEBUG_TRACE, "<==RTMPIoctlBBP\n");
}

/* 
    ==========================================================================
    Description:
        Read / Write MAC
Arguments:
    pAdapter                    Pointer to our adapter
    wrq                         Pointer to the ioctl argument

    Return Value:
        None

    Note:
        Usage: 
               1.) iwpriv ra0 mac 0        ==> read MAC where Addr=0x0
               2.) iwpriv ra0 mac 0=12     ==> write MAC where Addr=0x0, value=12
    ==========================================================================
*/
VOID RTMPIoctlMAC(
    IN  PRTMP_ADAPTER   pAdapter, 
    IN  struct iwreq    *wrq)
{
    char                *this_char;
    char                *value;
    int                 j=0, k=0;
    int                 count = 0;
    char                msg[1024];
    char                arg[255];
    char                *ptr;
    ULONG               macAddr = 0;
    UCHAR               temp[16], temp2[16];
    ULONG               macValue;

    DBGPRINT(RT_DEBUG_TRACE, "==>RTMPIoctlMAC\n");
    memset(msg, 0x00, 1024);
    if (wrq->u.data.length > 1) //No parameters.
    {
        memcpy(arg, wrq->u.data.pointer, (wrq->u.data.length > 255) ? 255 : wrq->u.data.length);
        ptr = arg;
        sprintf(msg, "\n");
        //Parsing Read or Write
        while ((this_char = strsep(&ptr, ",")) != NULL)
        {
            DBGPRINT(RT_DEBUG_TRACE, "this_char=%s\n", this_char);
            if (!*this_char)
                continue;

            if ((value = rtstrchr(this_char, '=')) != NULL)
                *value++ = 0;

            if (!value || !*value)
            { //Read
                // Sanity check
                if(strlen(this_char) > 4)
                    break;

                j = strlen(this_char);
                while(j-- > 0)
                {
                    if(this_char[j] > 'f' || this_char[j] < '0')
                        return;
                }

                // Mac Addr
                k = j = strlen(this_char);
                while(j-- > 0)
                {
                    this_char[4-k+j] = this_char[j];
                }
                
                while(k < 4)
                    this_char[3-k++]='0';
                this_char[4]='\0';

                if(strlen(this_char) == 4)
                {
                    AtoH(this_char, temp, 4);
                    macAddr = *temp*256 + temp[1];
                    if (macAddr < 0xFFFF)
                    {
                        RTMP_IO_READ32(pAdapter, macAddr, &macValue);
                        DBGPRINT(RT_DEBUG_TRACE, "macAddr=%x, regMAC=%x\n", macAddr, macValue);
                        sprintf(msg+strlen(msg), "[0x%08X]:%08X  ", macAddr , macValue);
                        count++;
                        if (count%5 == 4)
                            sprintf(msg+strlen(msg), "\n");
                        DBGPRINT(RT_DEBUG_TRACE, "msg=%s\n", msg);
                    }
                    else
                    {//Invalid parametes, so default print all bbp
                        break;
                    }
                }
            }
            else
            { //Write
                memcpy(&temp2, value, strlen(value));
                temp2[strlen(value)] = '\0';

                // Sanity check
                if((strlen(this_char) > 4) || strlen(temp2) > 8)
                    break;

                j = strlen(this_char);
                while(j-- > 0)
                {
                    if(this_char[j] > 'f' || this_char[j] < '0')
                        return;
                }

                j = strlen(temp2);
                while(j-- > 0)
                {
                    if(temp2[j] > 'f' || temp2[j] < '0')
                        return;
                }

                //MAC Addr
                k = j = strlen(this_char);
                while(j-- > 0)
                {
                    this_char[4-k+j] = this_char[j];
                }

                while(k < 4)
                    this_char[3-k++]='0';
                this_char[4]='\0';

                //MAC value
                k = j = strlen(temp2);
                while(j-- > 0)
                {
                    temp2[8-k+j] = temp2[j];
                }
                
                while(k < 8)
                    temp2[7-k++]='0';
                temp2[8]='\0';

                {
                    AtoH(this_char, temp, 4);
                    macAddr = *temp*256 + temp[1];

                    AtoH(temp2, temp, 8);
                    macValue = *temp*256*256*256 + temp[1]*256*256 + temp[2]*256 + temp[3];

                    DBGPRINT(RT_DEBUG_TRACE, "macAddr=%02x, macValue=0x%x\n", macAddr, macValue);
                    
                    RTMP_IO_WRITE32(pAdapter, macAddr, macValue);
                    sprintf(msg+strlen(msg), "[0x%02X]:%02X  ", macAddr, macValue);
                    count++;
                    if (count%5 == 4)
                        sprintf(msg+strlen(msg), "\n");
                    DBGPRINT(RT_DEBUG_TRACE, "msg=%s\n", msg);
                }
            }
        }
    }

    if(strlen(msg) == 1)
        sprintf(msg+strlen(msg), "===>Error command format!");
    DBGPRINT(RT_DEBUG_TRACE, "copy to user [msg=%s]\n", msg);
    // Copy the information into the user buffer
    wrq->u.data.length = strlen(msg);
    if(copy_to_user(wrq->u.data.pointer, msg, wrq->u.data.length))
    		DBGPRINT(RT_DEBUG_ERROR, "RTMPIoctlMAC - copy to user failure.\n");
    
    DBGPRINT(RT_DEBUG_TRACE, "<==RTMPIoctlMAC\n");
}

#ifdef RALINK_ATE

/* 
    ==========================================================================
    Description:
        Read / Write E2PROM
Arguments:
    pAdapter                    Pointer to our adapter
    wrq                         Pointer to the ioctl argument

    Return Value:
        None

    Note:
        Usage: 
               1.) iwpriv ra0 e2p 0     	==> read E2PROM where Addr=0x0
               2.) iwpriv ra0 e2p 0=1234    ==> write E2PROM where Addr=0x0, value=1234
    ==========================================================================
*/
VOID RTMPIoctlE2PROM(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	struct iwreq	*wrq)
{
	char				*this_char;
	char				*value;
	int					j=0, k=0;
	int					count = 0;
	char				msg[1024];
	char				arg[255];
	char				*ptr;
	USHORT				eepAddr = 0;
	UCHAR				temp[16], temp2[16];
	USHORT				eepValue;

	DBGPRINT(RT_DEBUG_TRACE, "==>RTMPIoctlE2PROM\n");
	memset(msg, 0x00, 1024);
	if (wrq->u.data.length > 1) //No parameters.
	{
		memcpy(arg, wrq->u.data.pointer, (wrq->u.data.length > 255) ? 255 : wrq->u.data.length);
		ptr = arg;
		sprintf(msg, "\n");
		//Parsing Read or Write
		while ((this_char = strsep(&ptr, ",")) != NULL)
		{
			DBGPRINT(RT_DEBUG_TRACE, "this_char=%s\n", this_char);
			if (!*this_char)
				continue;

			if ((value = strchr(this_char, '=')) != NULL)
				*value++ = 0;

			if (!value || !*value)
			{ //Read
				DBGPRINT(RT_DEBUG_TRACE, "Read: this_char=%s, strlen=%d\n", this_char, strlen(this_char));

				// Sanity check
				if(strlen(this_char) > 4)
					break;

				j = strlen(this_char);
				while(j-- > 0)
				{
					if(this_char[j] > 'f' || this_char[j] < '0')
						return;
				}

				// E2PROM addr
				k = j = strlen(this_char);
				while(j-- > 0)
				{
					this_char[4-k+j] = this_char[j];
				}
				
				while(k < 4)
					this_char[3-k++]='0';
				this_char[4]='\0';

				if(strlen(this_char) == 4)
				{
					AtoH(this_char, temp, 4);
					eepAddr = *temp*256 + temp[1];					
					if (eepAddr < 0xFFFF)
					{
						eepValue = RTMP_EEPROM_READ16(pAdapter, eepAddr);
						DBGPRINT(RT_DEBUG_TRACE, "eepAddr=%x, eepValue=%x\n", eepAddr, eepValue);
						sprintf(msg+strlen(msg), "[0x%04X]:%04X  ", eepAddr , eepValue);
						count++;
						if (count%5 == 4)
							sprintf(msg+strlen(msg), "\n");
						DBGPRINT(RT_DEBUG_TRACE, "msg=%s\n", msg);
					}
					else
					{//Invalid parametes, so default printk all bbp
						break;
					}
				}
			}
			else
			{ //Write
				DBGPRINT(RT_DEBUG_TRACE, "Write: this_char=%s, strlen(value)=%d, value=%s\n", this_char, strlen(value), value);
				memcpy(&temp2, value, strlen(value));
				temp2[strlen(value)] = '\0';

				// Sanity check
				if((strlen(this_char) > 4) || strlen(temp2) > 8)
					break;

				j = strlen(this_char);
				while(j-- > 0)
				{
					if(this_char[j] > 'f' || this_char[j] < '0')
						return;
				}
				j = strlen(temp2);
				while(j-- > 0)
				{
					if(temp2[j] > 'f' || temp2[j] < '0')
						return;
				}

				//MAC Addr
				k = j = strlen(this_char);
				while(j-- > 0)
				{
					this_char[4-k+j] = this_char[j];
				}

				while(k < 4)
					this_char[3-k++]='0';
				this_char[4]='\0';

				//MAC value
				k = j = strlen(temp2);
				while(j-- > 0)
				{
					temp2[4-k+j] = temp2[j];
				}
				
				while(k < 4)
					temp2[3-k++]='0';
				temp2[4]='\0';

				AtoH(this_char, temp, 4);
				eepAddr = *temp*256 + temp[1];

				AtoH(temp2, temp, 4);
				eepValue = *temp*256 + temp[1];

				DBGPRINT(RT_DEBUG_TRACE, "eepAddr=%02x, eepValue=0x%x\n", eepAddr, eepValue);
				
				RTMP_EEPROM_WRITE16(pAdapter, eepAddr, eepValue);
				sprintf(msg+strlen(msg), "[0x%02X]:%02X  ", eepAddr, eepValue);
				count++;
				if (count%5 == 4)
					sprintf(msg+strlen(msg), "\n");
				DBGPRINT(RT_DEBUG_TRACE, "msg=%s\n", msg);
			}
		}
	}

	if(strlen(msg) == 1)
		sprintf(msg+strlen(msg), "===>Error command format!");

        // Copy the information into the user buffer	
        DBGPRINT(RT_DEBUG_TRACE, "copy to user [msg=%s]\n", msg);	
	wrq->u.data.length = strlen(msg);
	copy_to_user(wrq->u.data.pointer, msg, wrq->u.data.length);
	
	DBGPRINT(RT_DEBUG_TRACE, "<==RTMPIoctlE2PROM\n");
}

UCHAR TempletFrame[24] = {0x08,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0xAA,0xBB,0x12,0x34,0x56,0x00,0x11,0x22,0xAA,0xBB,0xCC,0x00,0x00};	// 802.11 MAC Header, Type:Data, Length:24bytes 

/*
    ==========================================================================
    Description:
        Set ATE operation mode to
        0. APSTOP  = Stop STA Mode
        1. APSTART = Start STA Mode
        2. TXCONT  = Continuous Transmit
        3. TXCARR  = Transmit Carrier
        4. TXFRAME = Transmit Frames
        5. RXFRAME = Receive Frames
    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	USHORT			BbpData;
	ULONG			MacData;
	PTXD_STRUC		pTxD;
	PUCHAR			pDest;
	UINT			i, j;
	
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_Proc (arg = %s)\n", arg);

	mdelay(5);

	AsicSwitchChannel(pAdapter, pAdapter->ate.Channel);
	AsicLockChannel(pAdapter, pAdapter->ate.Channel);

	mdelay(5);

	RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, 63, &BbpData);
	RTMP_IO_READ32(pAdapter, MACCSR1, &MacData);

	BbpData = 0;
	MacData &= 0xFBFFFFFF;

	if (!strcmp(arg, "STASTOP")) 
	{						
		DBGPRINT(RT_DEBUG_TRACE, "ATE: STASTOP\n");

		RTMP_IO_WRITE32(pAdapter, MACCSR1, MacData);
		RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 63, BbpData);

		pAdapter->ate.Mode = ATE_STASTOP;

        LinkDown(pAdapter);
		AsicEnableBssSync(pAdapter);
		netif_stop_queue(pAdapter->net_dev);
   		RTMPStationStop(pAdapter);		
		RTMP_IO_WRITE32(pAdapter, RXCSR0, 0xffffffff);	// Stop Rx
	}
	else if (!strcmp(arg, "STASTART")) 
	{						
		DBGPRINT(RT_DEBUG_TRACE, "ATE: STASTART\n");

		RTMP_IO_WRITE32(pAdapter, MACCSR1, MacData);
		RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 63, BbpData);

		pAdapter->ate.Mode = ATE_STASTART;

		RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x56);	// Start Rx
		netif_start_queue(pAdapter->net_dev);
		RTMPStationStart(pAdapter);
	}
	else if (!strcmp(arg, "TXCONT")) 		// Continuous Tx
	{						
		DBGPRINT(RT_DEBUG_TRACE, "ATE: TXCONT\n");
		
		pAdapter->ate.Mode = ATE_TXCONT;

		BbpData |= 0x80;
		MacData |= 0x04000000;

		RTMP_IO_WRITE32(pAdapter, MACCSR1, MacData);
		RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 63, BbpData);

		for (i = 0; (i < TX_RING_SIZE) && (i < pAdapter->ate.TxCount); i++)
		{
			pTxD = (PTXD_STRUC)pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
			pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;

			// Prepare frame payload
			memcpy(pDest, &TempletFrame, LENGTH_802_11);
			for(j = LENGTH_802_11; j < pAdapter->ate.TxLength; j++)
				pDest[j] = 0xAA;
			memcpy(&pDest[4], &pAdapter->ate.Addr1, ETH_LENGTH_OF_ADDRESS);
			memcpy(&pDest[10], &pAdapter->ate.Addr2, ETH_LENGTH_OF_ADDRESS);
			memcpy(&pDest[16], &pAdapter->ate.Addr3, ETH_LENGTH_OF_ADDRESS);

			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, FALSE, FALSE, FALSE,
				SHORT_RETRY, IFS_BACKOFF, pAdapter->ate.TxRate, 4,
				pAdapter->ate.TxLength, pAdapter->PortCfg.TxPreambleInUsed, 0);

			pAdapter->CurEncryptIndex++;
	        if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
	        {
	            pAdapter->CurEncryptIndex = 0;
	        }
		}

		RTMP_IO_WRITE32(pAdapter, RXCSR0, 0xffffffff);
		RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
	}
	else if (!strcmp(arg, "TXCARR"))			// Tx Carrier -------------------------------------
	{
		DBGPRINT(RT_DEBUG_TRACE, "ATE: TXCARR\n");
		pAdapter->ate.Mode = ATE_TXCARR;

		BbpData |= 0x40;
		MacData |= 0x04000000;

		RTMP_IO_WRITE32(pAdapter, MACCSR1, MacData);
		RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 63, BbpData);

		for (i = 0; (i < TX_RING_SIZE) && (i < pAdapter->ate.TxCount); i++)
		{
			pTxD = (PTXD_STRUC)pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
			pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;

			// Prepare frame payload
			memcpy(pDest, &TempletFrame, LENGTH_802_11);
			for(j = LENGTH_802_11; j < pAdapter->ate.TxLength; j++)
				pDest[j] = 0xAA;
			memcpy(&pDest[4], &pAdapter->ate.Addr1, ETH_LENGTH_OF_ADDRESS);
			memcpy(&pDest[10], &pAdapter->ate.Addr2, ETH_LENGTH_OF_ADDRESS);
			memcpy(&pDest[16], &pAdapter->ate.Addr3, ETH_LENGTH_OF_ADDRESS);

			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, FALSE, FALSE, FALSE,
				SHORT_RETRY, IFS_BACKOFF, pAdapter->ate.TxRate, 4,
				pAdapter->ate.TxLength, pAdapter->PortCfg.TxPreambleInUsed, 0);

			pAdapter->CurEncryptIndex++;
	        if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
	        {
	            pAdapter->CurEncryptIndex = 0;
	        }
		}

		RTMP_IO_WRITE32(pAdapter, RXCSR0, 0xffffffff);
		RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
	}
	else if (!strcmp(arg, "TXFRAME"))			// Tx Frames --------------------------------------
	{						
		DBGPRINT(RT_DEBUG_TRACE, "ATE: TXFRAME(Count=%d)\n", pAdapter->ate.TxCount);
		pAdapter->ate.Mode = ATE_TXFRAME;

		RTMP_IO_WRITE32(pAdapter, MACCSR1, MacData);
		RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 63, BbpData);

		pAdapter->ate.TxDoneCount = 0;
		
		for (i = 0; (i < TX_RING_SIZE) && (i < pAdapter->ate.TxCount); i++)
		{
			pTxD = (PTXD_STRUC)pAdapter->TxRing[pAdapter->CurEncryptIndex].va_addr;
			pDest = (PUCHAR) pAdapter->TxRing[pAdapter->CurEncryptIndex].va_data_addr;

			// Prepare frame payload
			memcpy(pDest, &TempletFrame, LENGTH_802_11);
			for(j = LENGTH_802_11; j < pAdapter->ate.TxLength; j++)
				pDest[j] = 0xAA;
			memcpy(&pDest[4], &pAdapter->ate.Addr1, ETH_LENGTH_OF_ADDRESS);
			memcpy(&pDest[10], &pAdapter->ate.Addr2, ETH_LENGTH_OF_ADDRESS);
			memcpy(&pDest[16], &pAdapter->ate.Addr3, ETH_LENGTH_OF_ADDRESS);

			RTMPWriteTxDescriptor(pTxD, TRUE, CIPHER_NONE, FALSE, FALSE, FALSE,
				SHORT_RETRY, IFS_BACKOFF, pAdapter->ate.TxRate, 4,
				pAdapter->ate.TxLength, Rt802_11PreambleLong, 0);

			pAdapter->CurEncryptIndex++;
	        if (pAdapter->CurEncryptIndex >= TX_RING_SIZE)
	        {
	            pAdapter->CurEncryptIndex = 0;
	        }
		}
		pAdapter->ate.TxDoneCount += i;
		DBGPRINT(RT_DEBUG_TRACE, "TXFRAME txcount=%d\n", pAdapter->ate.TxCount);
	    DBGPRINT(RT_DEBUG_TRACE, "pAdapter->ate.TxDoneCount = %d\n", pAdapter->ate.TxDoneCount);

		RTMP_IO_WRITE32(pAdapter, RXCSR0, 0xffffffff);
		RTMP_IO_WRITE32(pAdapter, SECCSR1, 0x1);
	}
	else if (!strcmp(arg, "RXFRAME")) 			// Rx Frames --------------------------------------
	{						
		DBGPRINT(RT_DEBUG_TRACE, "ATE: RXFRAME\n");

		RTMP_IO_WRITE32(pAdapter, MACCSR1, MacData);
		RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 63, BbpData);

		pAdapter->ate.Mode = ATE_RXFRAME;
		pAdapter->ate.TxDoneCount = pAdapter->ate.TxCount;
		
		RTMP_IO_WRITE32(pAdapter, TXCSR0, 0x08);		// Abort Tx
		RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x56);		// Start Rx
	}
	else
	{	
		DBGPRINT(RT_DEBUG_TRACE, "ATE:	Invalid arg!\n");
		return FALSE;
	}

	mdelay(5);
	
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_Proc\n");
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE ADDR1=DA for TxFrames    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_DA_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	char				*value;
	int					i;
	
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_DA_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	if(strlen(arg) != 17)  //Mac address acceptable format 01:02:03:04:05:06 length 17
		return FALSE;

    for (i=0, value = strtok(arg,":"); value; value = strtok(NULL,":")) 
	{
		if((strlen(value) != 2) || (!isxdigit(*value)) || (!isxdigit(*(value+1))) ) 
			return FALSE;  //Invalid

		AtoH(value, &pAdapter->ate.Addr1[i++], 2);
	}

	if(i != 6)
		return FALSE;  //Invalid
		
	DBGPRINT(RT_DEBUG_TRACE, "DA=%2X:%2X:%2X:%2X:%2X:%2X\n", pAdapter->ate.Addr1[0], pAdapter->ate.Addr1[1], pAdapter->ate.Addr1[2], pAdapter->ate.Addr1[3], pAdapter->ate.Addr1[4], pAdapter->ate.Addr1[5]);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_DA_Proc\n");
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE ADDR2=SA for TxFrames    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_SA_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	char				*value;
	int					i;
	
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_SA_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	if(strlen(arg) != 17)  //Mac address acceptable format 01:02:03:04:05:06 length 17
		return FALSE;

    for (i=0, value = strtok(arg,":"); value; value = strtok(NULL,":")) 
	{
		if((strlen(value) != 2) || (!isxdigit(*value)) || (!isxdigit(*(value+1))) ) 
			return FALSE;  //Invalid

		AtoH(value, &pAdapter->ate.Addr2[i++], 2);
	}

	if(i != 6)
		return FALSE;  //Invalid

	DBGPRINT(RT_DEBUG_TRACE, "DA=%2X:%2X:%2X:%2X:%2X:%2X\n", pAdapter->ate.Addr2[0], pAdapter->ate.Addr2[1], pAdapter->ate.Addr2[2], pAdapter->ate.Addr2[3], pAdapter->ate.Addr2[4], pAdapter->ate.Addr2[5]);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_SA_Proc\n");
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE ADDR3=BSSID for TxFrames    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_BSSID_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	char				*value;
	int					i;
	
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_BSSID_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	if(strlen(arg) != 17)  //Mac address acceptable format 01:02:03:04:05:06 length 17
		return FALSE;

    for (i=0, value = strtok(arg,":"); value; value = strtok(NULL,":")) 
	{
		if((strlen(value) != 2) || (!isxdigit(*value)) || (!isxdigit(*(value+1))) ) 
			return FALSE;  //Invalid

		AtoH(value, &pAdapter->ate.Addr3[i++], 2);
	}

	if(i != 6)
		return FALSE;  //Invalid

	DBGPRINT(RT_DEBUG_TRACE, "DA=%2X:%2X:%2X:%2X:%2X:%2X\n", pAdapter->ate.Addr3[0], pAdapter->ate.Addr3[1], pAdapter->ate.Addr3[2], pAdapter->ate.Addr3[3], pAdapter->ate.Addr3[4], pAdapter->ate.Addr3[5]);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_BSSID_Proc\n");
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE Channel    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_CHANNEL_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_CHANNEL_Proc (arg = %s)\n", arg);
	
	pAdapter->ate.Channel = simple_strtol(arg, 0, 10);
	if((pAdapter->ate.Channel < 1) || (pAdapter->ate.Channel > 14))
	{
		pAdapter->ate.Channel = 1;
		return FALSE;
	}

	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_CHANNEL_Proc (ATE Channel = %d)\n", pAdapter->ate.Channel);
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE Tx Power    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_TX_POWER_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	ULONG R3;
	
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_TX_POWER_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	pAdapter->ate.TxPower = simple_strtol(arg, 0, 10);

	if(pAdapter->ate.TxPower >= 32)
	{
		pAdapter->ate.TxPower = pAdapter->PortCfg.ChannelTxPower[pAdapter->PortCfg.Channel - 1];;
		return FALSE;
	}

	R3 = pAdapter->ate.TxPower;
    R3 = R3 << 9; // shift TX power control to correct RF register bit position

	R3 |= (pAdapter->PortCfg.LatchRfRegs.R3 & 0xffffc1ff);
	RTMP_RF_IO_WRITE32(pAdapter, R3);

	DBGPRINT(RT_DEBUG_TRACE, "TxPower = %d\n", pAdapter->ate.TxPower);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_TX_POWER_Proc\n");
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE Tx Length    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_TX_LENGTH_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_TX_LENGTH_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	pAdapter->ate.TxLength = simple_strtol(arg, 0, 10);

	if((pAdapter->ate.TxLength < 24) || (pAdapter->ate.TxLength > 1500))
	{
		pAdapter->ate.TxLength = 1500;
		return FALSE;
	}

	DBGPRINT(RT_DEBUG_TRACE, "TxLength = %d\n", pAdapter->ate.TxLength);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_TX_LENGTH_Proc\n");
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE Tx Count    Return:
        TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_TX_COUNT_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_TX_COUNT_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	pAdapter->ate.TxCount = simple_strtol(arg, 0, 10);

	DBGPRINT(RT_DEBUG_TRACE, "TxCount = %d\n", pAdapter->ate.TxCount);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_TX_COUNT_Proc\n");
	
	return TRUE;
}

/* 
    ==========================================================================
    Description:
        Set ATE Tx Rate
        Return:
        	TRUE if all parameters are OK, FALSE otherwise
    ==========================================================================
*/
INT	Set_ATE_TX_RATE_Proc(
	IN	PRTMP_ADAPTER	pAdapter, 
	IN	PUCHAR			arg)
{
	DBGPRINT(RT_DEBUG_TRACE, "==> Set_ATE_TX_RATE_Proc\n");
	DBGPRINT(RT_DEBUG_TRACE, "arg=%s\n", arg);
	
	pAdapter->ate.TxRate = simple_strtol(arg, 0, 10);

	if(pAdapter->ate.TxRate > RATE_54)
	{
		pAdapter->ate.TxRate = RATE_11;
		return FALSE;
	}

	DBGPRINT(RT_DEBUG_TRACE, "TxRate = %d\n", pAdapter->ate.TxRate);
	DBGPRINT(RT_DEBUG_TRACE, "<== Set_ATE_TX_RATE_Proc\n");
	
	return TRUE;
}

VOID RTMPStationStop(
    IN  PRTMP_ADAPTER   pAd)
{
    DBGPRINT(RT_DEBUG_TRACE, "==> RTMPStationStop\n");
    RTMPCancelTimer(&pAd->timer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.AssocTimer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.ReassocTimer);
    RTMPCancelTimer(&pAd->Mlme.AssocAux.DisassocTimer);
    RTMPCancelTimer(&pAd->Mlme.AuthAux.AuthTimer);
    RTMPCancelTimer(&pAd->Mlme.AuthRspAux.AuthRspTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.BeaconTimer);
    RTMPCancelTimer(&pAd->Mlme.SyncAux.ScanTimer);
    RTMPCancelTimer(&pAd->Mlme.PeriodicTimer);
    RTMPCancelTimer(&pAd->PortCfg.RfTuningTimer);
    if (pAd->PortCfg.LedMode == LED_MODE_TXRX_ACTIVITY)
        RTMPCancelTimer(&pAd->PortCfg.LedCntl.BlinkTimer);
    RTMPCancelTimer(&pAd->PortCfg.RxAnt.RxAntDiversityTimer);	
    DBGPRINT(RT_DEBUG_TRACE, "<== RTMPStationStop\n");
}

VOID RTMPStationStart(
    IN  PRTMP_ADAPTER   pAd)
{
    DBGPRINT(RT_DEBUG_TRACE, "==> RTMPStationStart\n");
    RTMPSetTimer(pAd, &pAd->Mlme.PeriodicTimer, MLME_TASK_EXEC_INTV);
    //RTMPSetTimer(pAd, &pAd->timer, DEBUG_TASK_DELAY);	 //not used.
    DBGPRINT(RT_DEBUG_TRACE, "<== RTMPStationStart\n");
}

#endif	//#ifdef RALINK_ATE


