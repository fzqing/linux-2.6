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
 *      Module Name: rtmp_main.c
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      PaulL           25th Nov 02     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW (rt2400)  8th  Dec 04     Promisc mode support
 *      Flavio (rt2400) 8th  Dec 04     Elegant irqreturn_t handling
 *      Flavio (rt2400) 8th  Dec 04     Remove local alloc_netdev
 *      Flavio (rt2400) 8th  Dec 04     Extra debug at init time
 *      Ivo (rt2400)    15th Dec 04     Debug Level Switching
 *	RobinC		16th Dec 04     support ifpreup scripts
 *      RobinC          17th Dec 04     Link quality reporting
 *      MarkW           31st Jan 05     sysfs support for HAL/NetworkMan
 *      Bruno           31st Jan 05     Network device name module param
 *      GertjanW        13th Feb 05     Shared IRQ handling fixes
 *      MeelisR         2nd  Mar 05     PCI management fixes
 *      MichalL         5th  Mar 05     BitKeeper slot_name fix
 *      Tor Petterson   19th Apr 05     Power management: Suspend and Resume
 *      MarkW           15th Jul 05     Disable File Config under 4KSTACK
 *      IvD             15th Jul 05     Support File Config with 4KSTACK
 ***************************************************************************/ 

#include "rt_config.h"

unsigned long IrqFlags;

//  Global static variable, Debug level flag
// Don't hide this behind debug define. There should be as little difference between debug and no-debug as possible.
int debug = 0;	/* Default is off. */
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Enable level: accepted values: 1 to switch debug on, 0 to switch debug off.");

static char *ifname = NULL ;
MODULE_PARM(ifname, "s");
MODULE_PARM_DESC(ifname, "Network device name (default ra%d)");

// Following information will be show when you run 'modinfo'
MODULE_AUTHOR("http://rt2x00.serialmonkey.com");
MODULE_DESCRIPTION("Ralink RT2500 802.11g WLAN driver " DRV_VERSION " " DRV_RELDATE);

MODULE_LICENSE("GPL");

extern	const struct iw_handler_def rt2500_iw_handler_def;

static INT __devinit RT2500_init_one (
    IN  struct pci_dev              *pPci_Dev,
    IN  const struct pci_device_id  *ent)
{
    INT rc;

    // wake up and enable device
    if (pci_enable_device (pPci_Dev))
    {
        rc = -EIO;
    }
    else
    {
        rc = RT2500_probe(pPci_Dev, ent);
        if (rc)
           pci_disable_device(pPci_Dev);
    }
    return rc;
}

//
// PCI device probe & initialization function
//
INT __devinit   RT2500_probe(
    IN  struct pci_dev              *pPci_Dev, 
    IN  const struct pci_device_id  *ent)
{
    struct  net_device      *net_dev;
    RTMP_ADAPTER            *pAd;
    CHAR                    *print_name;
    INT                     chip_id = (int) ent->driver_data;
    unsigned long           csr_addr;
    CSR3_STRUC              StaMacReg0;
    CSR4_STRUC              StaMacReg1;
    INT                     Status = -ENODEV;

    printk("%s %s %s http://rt2x00.serialmonkey.com\n", KERN_INFO DRV_NAME, DRV_VERSION, DRV_RELDATE);

    print_name = pPci_Dev ? pci_name(pPci_Dev) : "rt2500";

    // alloc_etherdev() will set net_dev->name
    net_dev = alloc_etherdev(sizeof(RTMP_ADAPTER));
    if (net_dev == NULL) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "init_ethernet failed\n");
        goto err_out;
    }

    SET_MODULE_OWNER(net_dev);

    #if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
       SET_NETDEV_DEV(net_dev, &(pPci_Dev->dev));
    #endif
        
    if (pci_request_regions(pPci_Dev, print_name))
        goto err_out_free_netdev;

    // Interrupt IRQ number
    net_dev->irq = pPci_Dev->irq;

    // map physical address to virtual address for accessing register
    csr_addr = (unsigned long) ioremap(pci_resource_start(pPci_Dev, 0), pci_resource_len(pPci_Dev, 0));
    if (!csr_addr) 
    {
        DBGPRINT(RT_DEBUG_TRACE, "ioremap failed for device %s, region 0x%X @ 0x%lX\n",
            print_name, (ULONG)pci_resource_len(pPci_Dev, 0), pci_resource_start(pPci_Dev, 0));
        goto err_out_free_res;
    }

    // Save CSR virtual address and irq to device structure
    net_dev->base_addr = (unsigned long)csr_addr;
    pAd = net_dev->priv;
    pAd->CSRBaseAddress = net_dev->base_addr;
    pAd->net_dev = net_dev;

    // Set DMA master
    pci_set_master(pPci_Dev);

    // Read MAC address
    NICReadAdapterInfo(pAd);
    
    RTMP_IO_READ32(pAd, CSR3, &StaMacReg0.word);
    RTMP_IO_READ32(pAd, CSR4, &StaMacReg1.word);
    net_dev->dev_addr[0] = StaMacReg0.field.Byte0;
    net_dev->dev_addr[1] = StaMacReg0.field.Byte1;
    net_dev->dev_addr[2] = StaMacReg0.field.Byte2;
    net_dev->dev_addr[3] = StaMacReg0.field.Byte3;
    net_dev->dev_addr[4] = StaMacReg1.field.Byte4;
    net_dev->dev_addr[5] = StaMacReg1.field.Byte5;

    pAd->chip_id = chip_id;
    pAd->pPci_Dev = pPci_Dev;

    // The chip-specific entries in the device structure.
    net_dev->open = RT2500_open;
    net_dev->hard_start_xmit = RTMPSendPackets;
    net_dev->stop = RT2500_close;
    net_dev->get_stats = RT2500_get_ether_stats;

#if WIRELESS_EXT >= 12
    net_dev->get_wireless_stats = RT2500_get_wireless_stats;
	net_dev->wireless_handlers = (struct iw_handler_def *) &rt2500_iw_handler_def;
#endif

    net_dev->set_multicast_list = RT2500_set_rx_mode;
    net_dev->do_ioctl = RT2500_ioctl;
    net_dev->set_mac_address = rt2500_set_mac_address;
    

    // register_netdev() will call dev_alloc_name() for us
    // TODO: Remove the following line to keep the default eth%d name
    if (ifname == NULL)
       strcpy(net_dev->name, "ra%d");
    else
       strncpy(net_dev->name, ifname, IFNAMSIZ);

    // Register this device
    Status = register_netdev(net_dev);
    if (Status)
        goto err_out_unmap;

    DBGPRINT(RT_DEBUG_TRACE, "%s: at 0x%lx, VA 0x%lx, IRQ %d. \n", 
        net_dev->name, pci_resource_start(pPci_Dev, 0), (unsigned long)csr_addr, pPci_Dev->irq);

    // Set driver data
    pci_set_drvdata(pPci_Dev, net_dev);

    // moved to here by RobinC so if-preup can work
    // When driver now loads it is loaded up with "factory" defaults
    // All this occurs while the net iface is down
    // iwconfig can then be used to configure card BEFORE
    // ifconfig ra0 up is applied.
    // Note the RT2500STA.dat file will still overwrite settings 
    // but it is useful for the settings iwconfig doesn't let you at
    PortCfgInit(pAd); 

    // Build channel list for default physical mode
    BuildChannelList(pAd);
 
    return 0;

err_out_unmap:
    iounmap((void *)csr_addr);
err_out_free_res:
    pci_release_regions(pPci_Dev);
err_out_free_netdev:
    kfree (net_dev);
err_out:
    return Status;
}

INT RT2500_open(
    IN  struct net_device *net_dev)
{
    PRTMP_ADAPTER   pAd = net_dev->priv;
    INT             status = NDIS_STATUS_SUCCESS;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
    if (!try_module_get(THIS_MODULE))
    {
        DBGPRINT(RT_DEBUG_ERROR, "%s: cannot reserve module\n", __FUNCTION__);
        return -1;
    }
#else
    MOD_INC_USE_COUNT;
#endif

    // Enable RF Tuning timer
    RTMPInitTimer(pAd, &pAd->PortCfg.RfTuningTimer, AsicRfTuningExec);
    
    // 1. Allocate DMA descriptors & buffers
    status = RTMPAllocDMAMemory(pAd);
    if (status != NDIS_STATUS_SUCCESS)
        goto out_module_put;

    // 2. request interrupt
    // Disable interrupts here which is as soon as possible
    // This statement should never be true. We might consider to remove it later
    if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_ACTIVE))
    {
        NICDisableInterrupt(pAd);
    }

    status = request_irq(pAd->pPci_Dev->irq, &RTMPIsr, SA_SHIRQ, net_dev->name, net_dev);
    if (status)
    {
        RTMPFreeDMAMemory(pAd);
        goto out_module_put;
    }
    RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_IN_USE);

    // 3. Read MAC address
    //NICReadAdapterInfo(pAd);

    DBGPRINT(RT_DEBUG_TRACE, "%s: RT2500_open() irq %d. MAC = %02x:%02x:%02x:%02x:%02x:%02x \n",
        net_dev->name, pAd->pPci_Dev->irq, pAd->CurrentAddress[0], pAd->CurrentAddress[1], pAd->CurrentAddress[2],
        pAd->CurrentAddress[3], pAd->CurrentAddress[4], pAd->CurrentAddress[5]);

    NICInitTransmit(pAd);

  // load in data from RT2500STA.dat file
  // note this will TRASH any preup settings applied
  // if the parameter exists in the file
   RTMPReadParametersFromFile(pAd);

    // initialize MLME
    status = MlmeInit(pAd);

    // Initialize Mlme Memory Handler
    // Allocate 20 nonpaged memory pool which size are MAX_LEN_OF_MLME_BUFFER for use
    status = MlmeInitMemoryHandler(pAd, 20, MAX_LEN_OF_MLME_BUFFER);

    if(status != NDIS_STATUS_SUCCESS)
    {
        free_irq(net_dev->irq, net_dev);
        RTMPFreeDMAMemory(pAd);
        goto out_module_put;
    }

    // Initialize Asics
    NICInitializeAdapter(pAd);

    NICReadEEPROMParameters(pAd);

    NICInitAsicFromEEPROM(pAd);

    // 2nd stage hardware initialization after all parameters are acquired from
    // Registry or E2PROM
    RTMPSetPhyMode(pAd, PHY_11BG_MIXED);

    // Set the timer to check for link beat.
/*    RTMPInitTimer(pAd, &pAd->timer, RT2500_timer);
    RTMPSetTimer(pAd, &pAd->timer, DEBUG_TASK_DELAY);*/

    // Enable interrupt
    NICEnableInterrupt(pAd);

    // Start net interface tx /rx
    netif_start_queue(net_dev);

    netif_carrier_on(net_dev);
    netif_wake_queue(net_dev);

    return 0;

out_module_put:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
    module_put(THIS_MODULE);
#else
    MOD_DEC_USE_COUNT;
#endif

    return status;
}

VOID RT2500_timer(
    IN  unsigned long data)
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)data;

//  NICCheckForHang(pAd);

    RTMPSetTimer(pAd, &pAd->timer, DEBUG_TASK_DELAY);
}

/*
    ========================================================================

    Routine Description:
        hard_start_xmit handler

    Arguments:
        skb             point to sk_buf which upper layer transmit
        net_dev         point to net_dev
    Return Value:
        None

    Note:

    ========================================================================
*/
INT RTMPSendPackets(
    IN  struct sk_buff *skb,
    IN  struct net_device *net_dev)
{
    NDIS_STATUS     Status = NDIS_STATUS_SUCCESS;
    PRTMP_ADAPTER   pAdapter = net_dev->priv;

    DBGPRINT(RT_DEBUG_INFO, "<==== RTMPSendPackets\n");

    if (pAdapter->PortCfg.BssType == BSS_MONITOR)
    {
       dev_kfree_skb_irq(skb); 
       return 0;
    }

    // Drop packets if no associations
    if (!INFRA_ON(pAdapter) && !ADHOC_ON(pAdapter))
    {
        // Drop send request since there are no physical connection yet
        // Check the association status for infrastructure mode
        // And Mibss for Ad-hoc mode setup
        dev_kfree_skb_irq(skb);
    }
    else
    {
        // This function has to manage NdisSendComplete return call within its routine
        // NdisSendComplete will acknowledge upper layer in two steps.
        // 1. Within Packet Enqueue, set the NDIS_STATUS_PENDING
        // 2. Within TxRingTxDone / PrioRingTxDone call NdisSendComplete with final status          
        // initial skb->data_len=0, we will use this variable to store data size when fragment(in TKIP)
        // and skb->len is actual data len
        skb->data_len = skb->len;
        Status = RTMPSendPacket(pAdapter,skb);

        if (Status != NDIS_STATUS_SUCCESS)
        {
            // Errors before enqueue stage
            dev_kfree_skb_irq(skb);
        }
    }

    // Dequeue one frame from SendTxWait queue and process it
    // There are two place calling dequeue for TX ring.
    // 1. Here, right after queueing the frame.
    // 2. At the end of TxRingTxDone service routine.
    if ((!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_BSS_SCAN_IN_PROGRESS)) && 
        (!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF)) &&
        (!RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RESET_IN_PROGRESS)))
    {
        //RTMPDeQueuePacket(pAdapter, &pAdapter->TxSwQueue0);
        // Call dequeue without selected queue, let the subroutine select the right priority
        // Tx software queue
        RTMPDeQueuePacket(pAdapter);
    }

    return 0;
}

/*
    ========================================================================

    Routine Description:
        Interrupt handler

    Arguments:
        irq                         interrupt line
        dev_instance                Pointer to net_device
        rgs                         store process's context before entering ISR, 
                                    this parameter is just for debug purpose.

    Return Value:
        VOID

    Note:

    ========================================================================
*/
irqreturn_t RTMPIsr(
    IN  INT             irq, 
    IN  VOID            *dev_instance, 
    IN  struct pt_regs  *rgs)
{
    struct net_device   *net_dev = dev_instance;
    PRTMP_ADAPTER       pAdapter = net_dev->priv;
    INTSRC_STRUC        IntSource;
    int         ret = 0;

    DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleInterrupt\n");

    // 1. Disable interrupt
	if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_IN_USE) && RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_INTERRUPT_ACTIVE))
	{
		NICDisableInterrupt(pAdapter);
	}

    //
    // Get the interrupt sources & saved to local variable
    //
    // RTMP_IO_READ32(pAdapter, CSR7, &IntSource);
    RTMP_IO_READ32(pAdapter, CSR7, &IntSource.word);
    RTMP_IO_WRITE32(pAdapter, CSR7, IntSource.word);

    //
    // Handle interrupt, walk through all bits
    // Should start from highest priority interrupt
    // The priority can be adjust by altering processing if statement
    //
    // If required spinlock, each interrupt service routine has to acquire
    // and release itself.
    //
    if (IntSource.field.TbcnExpire)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleTbcnInterrupt\n");
        RTMPHandleTbcnInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.TwakeExpire)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleTwakeupInterrupt\n");
        RTMPHandleTwakeupInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.TatimwExpire)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleTatimInterrupt\n");
        // RTMPHandleTatimInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.EncryptionDone)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleEncryptionDoneInterrupt\n");
        RTMPHandleEncryptionDoneInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.TxRingTxDone)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleTxRingTxDoneInterrupt\n");
        RTMPHandleTxRingTxDoneInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.AtimRingTxDone)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleAtimRingTxDoneInterrupt\n");
        RTMPHandleAtimRingTxDoneInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.PrioRingTxDone)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandlePrioRingTxDoneInterrupt\n");
        RTMPHandlePrioRingTxDoneInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.DecryptionDone)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleDecryptionDoneInterrupt\n");
        RTMPHandleDecryptionDoneInterrupt(pAdapter);
        ret = 1;
    }

    if (IntSource.field.RxDone)
    {
        DBGPRINT(RT_DEBUG_INFO, "====> RTMPHandleRxDoneInterrupt\n");
        RTMPHandleRxDoneInterrupt(pAdapter);
        RTMPHandleEncryptionDoneInterrupt(pAdapter);
        ret = 1;
    }

    // Do nothing if Reset in progress
    if (RTMP_TEST_FLAG(pAdapter, fRTMP_ADAPTER_RESET_IN_PROGRESS))
    {
        ret = 1;
        goto out;
    }

    //
    // Re-enable the interrupt (disabled in RTMPIsr)
    //
    NICEnableInterrupt(pAdapter);

    DBGPRINT(RT_DEBUG_INFO, "<==== RTMPHandleInterrupt\n");
out:
	if(ret)
		return IRQ_RETVAL(IRQ_HANDLED);
	else
		return IRQ_RETVAL(IRQ_NONE);
}

int rt2500_set_mac_address(struct net_device *net_dev, void *addr)
{
	RTMP_ADAPTER		*pAd = net_dev->priv;
	struct sockaddr		*mac = (struct sockaddr*) addr;
	u32			set_mac;

	if(netif_running(net_dev))
		return -EBUSY;

	if(!is_valid_ether_addr(&mac->sa_data[0]))
		return -EINVAL;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,4,20)) 	
	BUG_ON(net_dev->addr_len != ETH_ALEN);
#endif	
	
	memcpy(net_dev->dev_addr, mac->sa_data, ETH_ALEN);
	memcpy(pAd->CurrentAddress, mac->sa_data, ETH_ALEN);
	
	memset(&set_mac, 0x00, sizeof(INT));
	set_mac = (net_dev->dev_addr[0]) |
			(net_dev->dev_addr[1] << 8) |
			(net_dev->dev_addr[2] << 16) |
			(net_dev->dev_addr[3] << 24);
	
	RTMP_IO_WRITE32(pAd, CSR3, set_mac);
	
	memset(&set_mac, 0x00, sizeof(INT));
	set_mac = (net_dev->dev_addr[4]) |
			(net_dev->dev_addr[5] << 8);
	
	RTMP_IO_WRITE32(pAd, CSR4, set_mac);
	
	printk(KERN_INFO "***rt2x00***: Info - Mac address changed to: %02x:%02x:%02x:%02x:%02x:%02x.\n", net_dev->dev_addr[0], net_dev->dev_addr[1], net_dev->dev_addr[2], net_dev->dev_addr[3], net_dev->dev_addr[4], net_dev->dev_addr[5]);
	
	return 0;
}


#if WIRELESS_EXT >= 12
/*
    ========================================================================

    Routine Description:
        get wireless statistics

    Arguments:
        net_dev                     Pointer to net_device

    Return Value:
        struct iw_statistics

    Note:
        This function will be called when query /proc

    ========================================================================
*/
struct iw_statistics *RT2500_get_wireless_stats(
    IN  struct net_device *net_dev)
{
    RTMP_ADAPTER *pAd = net_dev->priv;

    // TODO: All elements are zero before be implemented

    pAd->iw_stats.status = 0;           // Status - device dependent for now

    pAd->iw_stats.qual.qual = pAd->Mlme.ChannelQuality;//pAd->Mlme.RoamCqi;            // link quality (%retries, SNR, %missed beacons or better...)
    pAd->iw_stats.qual.level = pAd->PortCfg.LastRssi - RSSI_TO_DBM_OFFSET;   // signal level (dBm)
        
    pAd->iw_stats.qual.noise = (pAd->PortCfg.LastR17Value > BBP_R17_DYNAMIC_UP_BOUND) ? BBP_R17_DYNAMIC_UP_BOUND : ((ULONG) pAd->PortCfg.LastR17Value);           // // noise level (dBm)
    pAd->iw_stats.qual.updated = 3;     // Flags to know if updated

    pAd->iw_stats.discard.nwid = 0;     // Rx : Wrong nwid/essid
    pAd->iw_stats.miss.beacon = 0;      // Missed beacons/superframe

    // pAd->iw_stats.discard.code, discard.fragment, discard.retries, discard.misc has counted in other place

    return &pAd->iw_stats;
}
#endif

/*
    ========================================================================

    Routine Description:
        return ethernet statistics counter

    Arguments:
        net_dev                     Pointer to net_device

    Return Value:
        net_device_stats*

    Note:

    ========================================================================
*/
struct net_device_stats *RT2500_get_ether_stats(
    IN  struct net_device *net_dev)
{
    RTMP_ADAPTER *pAd = net_dev->priv;

    DBGPRINT(RT_DEBUG_INFO, "RT2500_get_ether_stats --->\n");

    pAd->stats.rx_packets = pAd->WlanCounters.ReceivedFragmentCount.vv.LowPart;        // total packets received
    pAd->stats.tx_packets = pAd->WlanCounters.TransmittedFragmentCount.vv.LowPart;     // total packets transmitted

    pAd->stats.rx_bytes= pAd->RalinkCounters.ReceivedByteCount;             // total bytes received
    pAd->stats.tx_bytes = pAd->RalinkCounters.TransmittedByteCount;         // total bytes transmitted

    pAd->stats.rx_errors = pAd->Counters.RxErrors;                          // bad packets received
    pAd->stats.tx_errors = pAd->Counters.TxErrors;                          // packet transmit problems

    pAd->stats.rx_dropped = pAd->Counters.RxNoBuffer;                       // no space in linux buffers
    pAd->stats.tx_dropped = pAd->WlanCounters.FailedCount.vv.LowPart;                  // no space available in linux

    pAd->stats.multicast = pAd->WlanCounters.MulticastReceivedFrameCount.vv.LowPart;   // multicast packets received
    pAd->stats.collisions = pAd->Counters.OneCollision + pAd->Counters.MoreCollisions;  // Collision packets

    pAd->stats.rx_length_errors = 0;
    pAd->stats.rx_over_errors = pAd->Counters.RxNoBuffer;                   // receiver ring buff overflow
    pAd->stats.rx_crc_errors = 0;//pAd->WlanCounters.FCSErrorCount;     // recved pkt with crc error
    pAd->stats.rx_frame_errors = pAd->Counters.RcvAlignmentErrors;          // recv'd frame alignment error
    pAd->stats.rx_fifo_errors = pAd->Counters.RxNoBuffer;                   // recv'r fifo overrun
    pAd->stats.rx_missed_errors = 0;                                            // receiver missed packet

    // detailed tx_errors
    pAd->stats.tx_aborted_errors = 0;
    pAd->stats.tx_carrier_errors = 0;
    pAd->stats.tx_fifo_errors = 0;
    pAd->stats.tx_heartbeat_errors = 0;
    pAd->stats.tx_window_errors = 0;

    // for cslip etc
    pAd->stats.rx_compressed = 0;
    pAd->stats.tx_compressed = 0;

    return &pAd->stats;
}

/*
    ========================================================================

    Routine Description:
        Set to filter multicast list

    Arguments:
        net_dev                     Pointer to net_device

    Return Value:
        VOID

    Note:

    ========================================================================
*/
VOID RT2500_set_rx_mode(
    IN  struct net_device *net_dev)
{
    RTMP_ADAPTER *pAd;
    pAd = net_dev->priv; 
    if (net_dev->flags&IFF_PROMISC)
    {
        pAd->bAcceptPromiscuous = TRUE;
        RTMP_IO_WRITE32(pAd, RXCSR0, 0x6e);
        DBGPRINT(RT_DEBUG_TRACE,"rt2500 acknowledge PROMISC on\n");
    }
    else
    {
        pAd->bAcceptPromiscuous = FALSE;
        RTMP_IO_WRITE32(pAd, RXCSR0, 0x7e);
        DBGPRINT(RT_DEBUG_TRACE, "rt2500 acknowledge PROMISC off\n");
    }   

}

//
// Close driver function
//
INT RT2500_close(
    IN  struct net_device *net_dev)
{
    RTMP_ADAPTER    *pAd = net_dev->priv;
    // LONG            ioaddr = net_dev->base_addr;

    DBGPRINT(RT_DEBUG_TRACE, "%s: ===> RT2500_close\n", net_dev->name);

    LinkDown(pAd);

    // Stop Mlme state machine
    RTMPCancelTimer(&pAd->PortCfg.RfTuningTimer);
    MlmeHalt(pAd);

    netif_stop_queue(net_dev);
    netif_carrier_off(net_dev);

    // Shut down monitor timer task
    //RTMPCancelTimer(&pAd->timer);

    if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_ACTIVE))
    {
        NICDisableInterrupt(pAd);
    }

    // Disable Rx, register value supposed will remain after reset
    NICIssueReset(pAd);

    // Free IRQ
    if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_IN_USE))
    {
        // Deregister interrupt function
        free_irq(net_dev->irq, net_dev);
        RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_INTERRUPT_IN_USE);
    }

    // Free Ring buffers
    RTMPFreeDMAMemory(pAd);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
    module_put(THIS_MODULE);
#else
    MOD_DEC_USE_COUNT;
#endif

    return 0;
}

//
// Remove driver function
//
static VOID __devexit RT2500_remove_one(
    IN  struct pci_dev  *pPci_Dev)
{
    struct net_device   *net_dev = pci_get_drvdata(pPci_Dev);
    // RTMP_ADAPTER        *pAd = net_dev->priv;

    // Unregister network device
    unregister_netdev(net_dev);

    // Unmap CSR base address
    iounmap((void *)(net_dev->base_addr));

    // release memory regions
    pci_release_regions(pPci_Dev);

    // disable the device
    pci_disable_device(pPci_Dev);

    // Free pre-allocated net_device memory
    kfree(net_dev);
}

//
// prepare for software suspend
//
#ifdef CONFIG_PM
#ifndef pm_message_t
#define pm_message_t u32
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,9))
static u32	suspend_buffer[16];
#define rt2x00_save_state(__pci)	pci_save_state(__pci, suspend_buffer)
#define rt2x00_restore_state(__pci)	pci_restore_state(__pci, suspend_buffer)
#else /* (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,8)) */
#define rt2x00_save_state(__pci)	pci_save_state(__pci)
#define rt2x00_restore_state(__pci)	pci_restore_state(__pci)
#endif /* (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,8)) */

static int rt2500_suspend(struct pci_dev *pdev, pm_message_t state)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    PRTMP_ADAPTER pAdapter = (PRTMP_ADAPTER) dev->priv;

    printk(KERN_NOTICE "%s: got suspend request (state %d)\n",
           dev->name, state);

    rt2x00_save_state(pdev);

    NICDisableInterrupt(pAdapter);
    MlmeHalt(pAdapter);

    netif_stop_queue(dev);
    netif_device_detach(dev);

    return 0;
}

//
// reactivate after software suspend
//
static int rt2500_resume(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    PRTMP_ADAPTER pAdapter = (PRTMP_ADAPTER) dev->priv;
    int status;

    pci_enable_device(pdev);

    printk(KERN_NOTICE "%s: got resume request\n", dev->name);

    rt2x00_restore_state(pdev);

    NICInitTransmit(pAdapter);
    status = MlmeInit(pAdapter);
    status = MlmeInitMemoryHandler(pAdapter, 20, MAX_LEN_OF_MLME_BUFFER);
    NICInitializeAdapter(pAdapter);
    NICEnableInterrupt(pAdapter);

    netif_device_attach(dev);
    netif_start_queue(dev);

    return 0;
}

#endif /* CONFIG_PM */

//
// Ralink PCI device table, include all supported chipsets
//
static struct pci_device_id rt2500_pci_tbl[] __devinitdata =
{
    {0x1814, 0x0201, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RT2560A},
    {0,}                                /* terminate list */
};
MODULE_DEVICE_TABLE(pci, rt2500_pci_tbl);

//
// Our PCI driver structure
//
static struct pci_driver rt2500_driver =
{
    name:       "rt2500",
    id_table:   rt2500_pci_tbl,
    probe:      RT2500_init_one,
#ifdef CONFIG_PM
    suspend:    rt2500_suspend,
    resume:     rt2500_resume,
#endif /* CONFIG_PM */
#if LINUX_VERSION_CODE >= 0x20412 || BIG_ENDIAN == TRUE || RTMP_EMBEDDED == TRUE
    remove:     __devexit_p(RT2500_remove_one),
#else
    remove:     __devexit(RT2500_remove_one),
#endif
};

// =======================================================================
// LOAD / UNLOAD sections
// =======================================================================
//
// Driver module load function
//
static INT __init rt2500_init_module(VOID)
{
    return pci_module_init(&rt2500_driver);
}

//
// Driver module unload function
//
static VOID __exit rt2500_cleanup_module(VOID)
{
    pci_unregister_driver(&rt2500_driver);
}

module_init(rt2500_init_module);
module_exit(rt2500_cleanup_module);
