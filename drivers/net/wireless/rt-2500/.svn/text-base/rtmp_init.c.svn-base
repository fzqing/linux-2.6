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
 *      Module Name: rtmp_init.c
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      PaulL           1st  Aug 02     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW (rt2400)  8th  Dec 04     Promisc mode support
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0
 *      LuisCorreia     15th Feb 05     Added Yann's patch for radio hw
 *      MarkW           12th Jul 05     Disabled all but CAM Power modes
 ***************************************************************************/ 

#include    "rt_config.h"

//
// BBP register initialization set
//
ULONG   BBPRegTable[] = {
    0x00018302,  // R03
    0x00018419,  // R04
    0x00018E1C,  // R14
    0x00018F30,  // R15
    0x000190ac,  // R16
    0x00019148,  // R17
    0x00019218,  // R18
    0x000193ff,  // R19
    0x0001941E,  // R20
    0x00019508,  // R21
    0x00019608,  // R22
    0x00019708,  // R23
    0x00019870,  // R24
    0x00019940,  // R25
    0x00019A08,  // R26
    0x00019B23,  // R27
    0x00019E10,  // R30
    0x00019F2B,  // R31
    0x0001A0B9,  // R32
    0x0001A212,  // R34
    0x0001A350,  // R35
    0x0001A7c4,  // R39
    0x0001A802,  // R40
    0x0001A960,  // R41
    0x0001B510,  // R53
    0x0001B618,  // R54
    0x0001B808,  // R56
    0x0001B910,  // R57
    0x0001BA08,  // R58
    0x0001BD6d,  // R61
    0x0001BE10,  // R62
};

//
// MAC register initialization sets
//
RTMP_REG_PAIR   MACRegTable[] = {
    {PSCSR0,    0x00020002},            // 0xc8
    {PSCSR1,    0x00000002},            // 0xcc
//  {PSCSR2,    0x00023f20},            // 0xd0
    {PSCSR2,    0x00020002},        // 0xd0
    {PSCSR3,    0x00000002},            // 0xd4
    {TIMECSR,   0x00003f21},        // 0xDC, to slower down our 1-us tick 
    {CSR9,      0x00000780},        // 0x24
    {CSR11,     0x07041483},        // 0x2C, lrc=7, src=4, slot=20us, CWmax=2^8, CWmax=2^3  
    {CSR18,     0x00140000},        // SIFS=10us - TR switch time, PIFS=SIFS+20us
    {CSR19,     0x016C0028},        // DIFS=SIFS+2*20us, EIFS=364us
    {CNT3,      0x00000000},        // Backoff_CCA_Th, RX_&_TX_CCA_Th

    {TXCSR1,    0x07614562},        // 0x64, ACK as 1Mb time
    {TXCSR8,    0x8c8d8b8a},        // 0x98, CCK TX BBP register ID
  //{TXCSR9,    0x86870885},        // 0x94, OFDM TX BBP register ID

    {ARCSR1,    0x0000000f},        // 0x9c, Basic rate set bitmap
    {PLCP1MCSR, 0x00700400},        // 0x13c, ACK/CTS PLCP at 1 Mbps
    {PLCP2MCSR, 0x00380401},        // 0x140, ACK/CTS PLCP at 2 Mbps
    {PLCP5MCSR, 0x00150402},        // 0x144, ACK/CTS PLCP at 5.5 Mbps
    {PLCP11MCSR,0x000b8403},        // 0x148, ACK/CTS PLCP at 11 Mbps

    {ARTCSR0,   0x7038140a},        // 0x14c, ACK/CTS payload consumed time for 1/2/5.5/11 mbps
    {ARTCSR1,   0x1d21252d},        // 0x150, alexsu : OFDM ACK/CTS payload consumed time for 18/12/9/6 mbps
    {ARTCSR2,   0x1919191d},        // 0x154, alexsu : OFDM ACK/CTS payload consumed time for 54/48/36/24 mbps

    {RXCSR0,    0xffffffff},        // 0x80 
    {RXCSR3,    0xb3aab3af},        // 0x90. RT2530 BBP 51:RSSI, R42:OFDM rate, R47:CCK SIGNAL
    {PCICSR,    0x000003b8},        // 0x8c, alexsu : PCI control register
    {PWRCSR0,   0x3f3b3100},            // 0xC4
    {GPIOCSR,   0x0000ff00},		// 0x120, GPIO default value
	{TESTCSR,	0x000000f0},		// 0x138, Test CSR, make sure it's running at normal mode
    {PWRCSR1,   0x000001ff},            // 0xd8     
    {MACCSR0,   0x00213223},        // 0xE0, Enable Tx dribble mode - 2003/10/22:Gary
    {MACCSR1,   0x00235518},            // 0xE4, Disable Rx Reset, tx dribble count, 2x30x16 = 960n,
    {MACCSR2,   0x00000040},            // 0x0134, 64*33ns = 2us
    {RALINKCSR, 0x9a009a11},            // 0xE8 
    {CSR7,      0xffffffff},            // 0x1C, Clear all pending interrupt source
    {LEDCSR,    0x00001E46},        // default both LEDs off
    {BBPCSR1,   0x82188200},        // for 2560+2522
    {TXACKCSR0, 0x00000020},        // 0x110, TX ACK timeout in usec
    {SECCSR3,   0x0000e78f},        // AES, mask off more data bit for MIC calculation
};

#define NUM_BBP_REG_PARMS   (sizeof(BBPRegTable) / sizeof(ULONG))
#define NUM_MAC_REG_PARMS   (sizeof(MACRegTable) / sizeof(RTMP_REG_PAIR))

/*
    ========================================================================

    Routine Description:
        Allocate all DMA releated resources

    Arguments:
        Adapter         Pointer to our adapter

    Return Value:
        None

    Note:

    ========================================================================
*/
NDIS_STATUS RTMPAllocDMAMemory(
    IN  PRTMP_ADAPTER   pAd)
{
    INT             index;
    VOID            *ring;          // VA of ring descriptor
    VOID            *ring_data;     // VA of DMA data buffer
    dma_addr_t      ring_dma;       // PA of ring descriptor
    dma_addr_t      ring_data_dma;  // PA of DMA data buffer
    PTXD_STRUC      pTxD;           // Tx type ring descriptor
    PRXD_STRUC      pRxD;           // Rx type ring descriptor

    DBGPRINT(RT_DEBUG_INFO, "--> RTMPAllocDMAMemory\n");

    // 1. Allocate Tx Ring DMA descriptor and buffer memory 
    // Allocate Ring descriptors DMA block
    ring = pci_alloc_consistent(pAd->pPci_Dev, (TX_RING_SIZE * RING_DESCRIPTOR_SIZE), &ring_dma);
    if (!ring) {
        printk(KERN_ERR DRV_NAME "Could not allocate DMA ring descriptor memory.\n");
        goto err_out_allocate_txring;
    }

    // Zero init ring descriptors
    memset(ring, 0, (TX_RING_SIZE * RING_DESCRIPTOR_SIZE));
    
    // Allocate Ring data DMA blocks
    ring_data = pci_alloc_consistent(pAd->pPci_Dev, (TX_RING_SIZE * TX_BUFFER_SIZE), &ring_data_dma);
    
    // If failed, release ring descriptors DMA block & exit
    if (!ring_data) {
        pci_free_consistent(pAd->pPci_Dev, (TX_RING_SIZE * RING_DESCRIPTOR_SIZE), ring, ring_dma);
        printk(KERN_ERR DRV_NAME "Could not allocate DMA ring buffer memory.\n");       
        goto err_out_allocate_txring;
    }

    // Start with Tx ring & DMA buffer
    for (index = 0; index < TX_RING_SIZE; index++)
    {
        // Init Tx Ring Size, Va, Pa variables
        pAd->TxRing[index].size = RING_DESCRIPTOR_SIZE;
        pAd->TxRing[index].va_addr = ring;
        pAd->TxRing[index].pa_addr = ring_dma;
        ring     += RING_DESCRIPTOR_SIZE;
        ring_dma += RING_DESCRIPTOR_SIZE;
        
        // Init Tx DMA buffer
        pAd->TxRing[index].data_size = TX_BUFFER_SIZE;
        pAd->TxRing[index].va_data_addr = ring_data;
        pAd->TxRing[index].pa_data_addr = ring_data_dma;
        ring_data     += TX_BUFFER_SIZE;
        ring_data_dma += TX_BUFFER_SIZE;

        // Write TxD buffer address & allocated buffer length
        pTxD = (PTXD_STRUC) pAd->TxRing[index].va_addr;
#ifndef BIG_ENDIAN
        pTxD->BufferAddressPa = pAd->TxRing[index].pa_data_addr;
#else
        pTxD->BufferAddressPa = SWAP32(pAd->TxRing[index].pa_data_addr);
#endif

        DBGPRINT(RT_DEBUG_INFO, "TxRing[%d] va = 0x%lu, pa = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->TxRing[index].va_addr, (UINT)pAd->TxRing[index].pa_addr, pAd->TxRing[index].size);
        DBGPRINT(RT_DEBUG_INFO, "TxRing[%d] va_data = 0x%lu, pa_data = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->TxRing[index].va_data_addr, (UINT)pAd->TxRing[index].pa_data_addr, pAd->TxRing[index].data_size);
    }

    // 2. Allocate Prio Ring DMA descriptor and buffer memory 
    // Allocate Ring descriptors DMA block
    ring = pci_alloc_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * RING_DESCRIPTOR_SIZE), &ring_dma);
    if (!ring) {
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring descriptor memory.\n");
        goto err_out_allocate_prioring;
    }

    // Zero init ring descriptors
    memset(ring, 0, (PRIO_RING_SIZE * RING_DESCRIPTOR_SIZE));

    // Allocate Ring data DMA blocks
    ring_data = pci_alloc_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * PRIO_BUFFER_SIZE), &ring_data_dma);

    // If failed, release ring descriptors DMA block & exit
    if (!ring_data) {
        pci_free_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * RING_DESCRIPTOR_SIZE), ring, ring_dma);
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring buffer memory.\n");       
        goto err_out_allocate_prioring;
    }

    // Second with Prio ring & DMA buffer
    for (index = 0; index < PRIO_RING_SIZE; index++)
    {
        // Init Prio Ring Size, Va, Pa variables
        pAd->PrioRing[index].size = RING_DESCRIPTOR_SIZE;
        pAd->PrioRing[index].va_addr = ring;
        pAd->PrioRing[index].pa_addr = ring_dma;
        ring     += RING_DESCRIPTOR_SIZE;
        ring_dma += RING_DESCRIPTOR_SIZE;

        // Init Prio DMA buffer
        pAd->PrioRing[index].data_size = PRIO_BUFFER_SIZE;
        pAd->PrioRing[index].va_data_addr = ring_data;
        pAd->PrioRing[index].pa_data_addr = ring_data_dma;
        ring_data     += PRIO_BUFFER_SIZE;
        ring_data_dma += PRIO_BUFFER_SIZE;

        // Write TxD buffer address & allocated buffer length for priority ring
        pTxD = (PTXD_STRUC) pAd->PrioRing[index].va_addr;
#ifndef BIG_ENDIAN
        pTxD->BufferAddressPa = pAd->PrioRing[index].pa_data_addr;
#else
        pTxD->BufferAddressPa = SWAP32(pAd->PrioRing[index].pa_data_addr);
#endif

        DBGPRINT(RT_DEBUG_INFO, "PrioRing[%d] va = 0x%lu, pa = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->PrioRing[index].va_addr, (UINT)pAd->PrioRing[index].pa_addr, pAd->PrioRing[index].size);
        DBGPRINT(RT_DEBUG_INFO, "PrioRing[%d] va_data = 0x%lu, pa_data = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->PrioRing[index].va_data_addr, (UINT)pAd->PrioRing[index].pa_data_addr, pAd->PrioRing[index].data_size);
    }

    // 3. Allocate Atim Ring DMA descriptor and buffer memory 
    // Allocate Ring descriptors DMA block
    ring = pci_alloc_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * RING_DESCRIPTOR_SIZE), &ring_dma);
    if (!ring) {
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring descriptor memory.\n");
        goto err_out_allocate_atimring;
    }

    // Zero init ring descriptors
    memset(ring, 0, (ATIM_RING_SIZE * RING_DESCRIPTOR_SIZE));

    // Allocate Ring data DMA blocks
    ring_data = pci_alloc_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * ATIM_BUFFER_SIZE), &ring_data_dma);

    // If failed, release ring descriptors DMA block & exit
    if (!ring_data) {
        pci_free_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * RING_DESCRIPTOR_SIZE), ring, ring_dma);
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring buffer memory.\n");       
        goto err_out_allocate_atimring;
    }

    // Atim ring & DMA buffer
    for (index = 0; index < ATIM_RING_SIZE; index++)
    {
        // Init Atim Ring Size, Va, Pa variables
        pAd->AtimRing[index].size = RING_DESCRIPTOR_SIZE;
        pAd->AtimRing[index].va_addr = ring;
        pAd->AtimRing[index].pa_addr = ring_dma;
        ring     += RING_DESCRIPTOR_SIZE;
        ring_dma += RING_DESCRIPTOR_SIZE;

        // Init Atim DMA buffer
        pAd->AtimRing[index].data_size = ATIM_BUFFER_SIZE;
        pAd->AtimRing[index].va_data_addr = ring_data;
        pAd->AtimRing[index].pa_data_addr = ring_data_dma;
        ring_data     += ATIM_BUFFER_SIZE;
        ring_data_dma += ATIM_BUFFER_SIZE;

        // Write TxD buffer address & allocated buffer length
        pTxD = (PTXD_STRUC) pAd->AtimRing[index].va_addr;
#ifndef BIG_ENDIAN
        pTxD->BufferAddressPa = pAd->AtimRing[index].pa_data_addr;
#else
        pTxD->BufferAddressPa = SWAP32(pAd->AtimRing[index].pa_data_addr);
#endif

        DBGPRINT(RT_DEBUG_INFO, "AtimRing[%d] va = 0x%lu, pa = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->AtimRing[index].va_addr, (UINT)pAd->AtimRing[index].pa_addr, pAd->AtimRing[index].size);
        DBGPRINT(RT_DEBUG_INFO, "AtimRing[%d] va_data = 0x%lu, pa_data = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->AtimRing[index].va_data_addr, (UINT)pAd->AtimRing[index].pa_data_addr, pAd->AtimRing[index].data_size);
    }

    // 4. Allocate Rx Ring DMA descriptor and buffer memory
    // Allocate Ring descriptors DMA block
    ring = pci_alloc_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RING_DESCRIPTOR_SIZE), &ring_dma);
    if (!ring) {
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring descriptor memory.\n");
        goto err_out_allocate_rxring;
    }

    // Zero init ring descriptors
    memset(ring, 0, (RX_RING_SIZE * RING_DESCRIPTOR_SIZE));

    // Allocate Ring data DMA blocks
    ring_data = pci_alloc_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RX_BUFFER_SIZE), &ring_data_dma);

    // If failed, release ring descriptors DMA block & exit
    if (!ring_data) {
        pci_free_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RING_DESCRIPTOR_SIZE), ring, ring_dma);
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring buffer memory.\n");       
        goto err_out_allocate_rxring;
    }

    // Rx ring & DMA buffer
    for (index = 0; index < RX_RING_SIZE; index++)
    {
        // Init Rx Ring Size, Va, Pa variables
        pAd->RxRing[index].size = RING_DESCRIPTOR_SIZE;
        pAd->RxRing[index].va_addr = ring;
        pAd->RxRing[index].pa_addr = ring_dma;
        ring     += RING_DESCRIPTOR_SIZE;
        ring_dma += RING_DESCRIPTOR_SIZE;

        // Init Rx DMA buffer
        pAd->RxRing[index].data_size = RX_BUFFER_SIZE;
        pAd->RxRing[index].va_data_addr = ring_data;
        pAd->RxRing[index].pa_data_addr = ring_data_dma;
        ring_data     += RX_BUFFER_SIZE;
        ring_data_dma += RX_BUFFER_SIZE;

        // Write RxD buffer address & allocated buffer length
        pRxD = (PRXD_STRUC) pAd->RxRing[index].va_addr;
#ifndef BIG_ENDIAN
        pRxD->BufferAddressPa = pAd->RxRing[index].pa_data_addr;
#else
        pRxD->BufferAddressPa = SWAP32(pAd->RxRing[index].pa_data_addr);
#endif
        // Rx owner bit assign to NIC immediately
        pRxD->Owner = DESC_OWN_NIC;

#ifdef BIG_ENDIAN
		RTMPDescriptorEndianChange((PUCHAR)pRxD, TYPE_RXD);
#endif

        DBGPRINT(RT_DEBUG_INFO, "RxRing[%d] va = 0x%lu, pa = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->RxRing[index].va_addr, (UINT)pAd->RxRing[index].pa_addr, pAd->RxRing[index].size);
        DBGPRINT(RT_DEBUG_INFO, "RxRing[%d] va_data = 0x%lu, pa_data = 0x%x, size = 0x%x\n",
            index, (unsigned long)pAd->RxRing[index].va_data_addr, (UINT)pAd->RxRing[index].pa_data_addr, pAd->RxRing[index].data_size);
    }

    // 5. Allocate Beacon Ring DMA descriptor and buffer memory
    // Init Beacon Ring Size, Va, Pa variables
    ring = pci_alloc_consistent(pAd->pPci_Dev, RING_DESCRIPTOR_SIZE, &ring_dma);
    if (!ring) {
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring descriptor memory.\n");
        goto err_out_allocate_beaconring;
    }

    // Zero init ring descriptors
    memset(ring, 0, (RING_DESCRIPTOR_SIZE));

    // Allocate Ring data DMA blocks
    ring_data = pci_alloc_consistent(pAd->pPci_Dev, BEACON_BUFFER_SIZE, &ring_data_dma);

    // If failed, release ring descriptors DMA block & exit
    if (!ring_data) {
        pci_free_consistent(pAd->pPci_Dev, RING_DESCRIPTOR_SIZE, ring, ring_dma);
        DBGPRINT(RT_DEBUG_ERROR, "Could not allocate DMA ring buffer memory.\n");       
        goto err_out_allocate_beaconring;
    }

    pAd->BeaconRing.size = RING_DESCRIPTOR_SIZE;
    pAd->BeaconRing.va_addr = ring;
    pAd->BeaconRing.pa_addr = ring_dma;

    // Init Beacon DMA buffer
    pAd->BeaconRing.data_size = BEACON_BUFFER_SIZE;
    pAd->BeaconRing.va_data_addr = ring_data;
    pAd->BeaconRing.pa_data_addr = ring_data_dma;

    // Write RxD buffer address & allocated buffer length
    pTxD = (PTXD_STRUC) pAd->BeaconRing.va_addr;
#ifndef BIG_ENDIAN
    pTxD->BufferAddressPa = pAd->BeaconRing.pa_data_addr;
#else
    pTxD->BufferAddressPa = SWAP32(pAd->BeaconRing.pa_data_addr);
#endif

    DBGPRINT(RT_DEBUG_INFO, "BeaconRing va = 0x%lu, pa = 0x%x, size = 0x%x\n",
            (unsigned long)pAd->BeaconRing.va_addr, (UINT)pAd->BeaconRing.pa_addr, pAd->BeaconRing.size);
    DBGPRINT(RT_DEBUG_INFO, "BeaconRing va_data = 0x%lu, pa_data = 0x%x, size = 0x%x\n",
            (unsigned long)pAd->BeaconRing.va_data_addr, (UINT)pAd->BeaconRing.pa_data_addr, pAd->BeaconRing.data_size);

    DBGPRINT(RT_DEBUG_INFO, "<-- RTMPAllocDMAMemory\n");
    return NDIS_STATUS_SUCCESS;


err_out_allocate_beaconring:
    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RX_BUFFER_SIZE), 
        pAd->RxRing[0].va_data_addr, pAd->RxRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->RxRing[0].va_addr, pAd->RxRing[0].pa_addr);
err_out_allocate_rxring:
    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * ATIM_BUFFER_SIZE), 
        pAd->AtimRing[0].va_data_addr, pAd->AtimRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->AtimRing[0].va_addr, pAd->AtimRing[0].pa_addr);
err_out_allocate_atimring:
    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * PRIO_BUFFER_SIZE), 
        pAd->PrioRing[0].va_data_addr, pAd->PrioRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->PrioRing[0].va_addr, pAd->PrioRing[0].pa_addr);
err_out_allocate_prioring:
    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (TX_RING_SIZE * TX_BUFFER_SIZE), 
        pAd->TxRing[0].va_data_addr, pAd->TxRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (TX_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->TxRing[0].va_addr, pAd->TxRing[0].pa_addr);
err_out_allocate_txring:
    DBGPRINT(RT_DEBUG_ERROR, "<-- RTMPAllocDMAMemory (memory not allocate successfully!)\n");

    return -ENOMEM;
}

/*
    ========================================================================

    Routine Description:
        Free all DMA memory.

    Arguments:
        Adapter         Pointer to our adapter

    Return Value:
        None

    Note:

    ========================================================================
*/
VOID    RTMPFreeDMAMemory(
    IN  PRTMP_ADAPTER   pAd)
{
    DBGPRINT(RT_DEBUG_INFO, "--> RTMPFreeDMAMemory\n");

    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (TX_RING_SIZE * TX_BUFFER_SIZE), 
        pAd->TxRing[0].va_data_addr, pAd->TxRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (TX_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->TxRing[0].va_addr, pAd->TxRing[0].pa_addr);

    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * PRIO_BUFFER_SIZE), 
        pAd->PrioRing[0].va_data_addr, pAd->PrioRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (PRIO_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->PrioRing[0].va_addr, pAd->PrioRing[0].pa_addr);

    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * ATIM_BUFFER_SIZE), 
        pAd->AtimRing[0].va_data_addr, pAd->AtimRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (ATIM_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->AtimRing[0].va_addr, pAd->AtimRing[0].pa_addr);
    
    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RX_BUFFER_SIZE), 
        pAd->RxRing[0].va_data_addr, pAd->RxRing[0].pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (RX_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->RxRing[0].va_addr, pAd->RxRing[0].pa_addr);

    // Free data DMA blocks first, the start address is the same as TxRing first DMA data block
    pci_free_consistent(pAd->pPci_Dev, (BEACON_RING_SIZE * BEACON_BUFFER_SIZE), 
        pAd->BeaconRing.va_data_addr, pAd->BeaconRing.pa_data_addr);
    // Free ring descriptor second, the start address is the same as TxRing first elment
    pci_free_consistent(pAd->pPci_Dev, (BEACON_RING_SIZE * RING_DESCRIPTOR_SIZE),
        pAd->BeaconRing.va_addr, pAd->BeaconRing.pa_addr);

}

/*
    ========================================================================

    Routine Description:
        Initialize transmit data structures

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:
        Initialize all transmit releated private buffer, include those define
        in RTMP_ADAPTER structure and all private data structures.

    ========================================================================
*/
VOID    NICInitTransmit(
    IN  PRTMP_ADAPTER   pAdapter)
{
    DBGPRINT(RT_DEBUG_TRACE, "--> NICInitTransmit\n");

    // Initialize all Transmit releated queues
    skb_queue_head_init(&pAdapter->TxSwQueue0);
    skb_queue_head_init(&pAdapter->TxSwQueue1);
    skb_queue_head_init(&pAdapter->TxSwQueue2);
    skb_queue_head_init(&pAdapter->TxSwQueue3);

    // Init Ring index pointer
    pAdapter->CurRxIndex           = 0;
    pAdapter->CurDecryptIndex      = 0;
    pAdapter->CurTxIndex           = 0;
    pAdapter->CurEncryptIndex      = 0;
    pAdapter->CurAtimIndex         = 0;
    pAdapter->CurPrioIndex         = 0;
    pAdapter->NextEncryptDoneIndex = 0;
    pAdapter->NextTxDoneIndex      = 0;
    pAdapter->NextAtimDoneIndex    = 0;
    pAdapter->NextPrioDoneIndex    = 0;
    pAdapter->NextDecryptDoneIndex = 0;
    pAdapter->PushMgmtIndex        = 0;
    pAdapter->PopMgmtIndex         = 0;
    pAdapter->MgmtQueueSize        = 0;

    pAdapter->PrivateInfo.TxRingFullCnt       = 0;

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitTransmit\n");
}

/*
    ========================================================================

    Routine Description:
        Read additional information from EEPROM, such as MAC address

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        NDIS_STATUS_SUCCESS
        NDIS_STATUS_FAILURE

    Note:

    ========================================================================
*/
NDIS_STATUS NICReadAdapterInfo(
    IN  PRTMP_ADAPTER       pAd)
{
    CSR3_STRUC      StaMacReg0;
    CSR4_STRUC      StaMacReg1;
    NDIS_STATUS     Status = NDIS_STATUS_SUCCESS;

    // 
    // Read MAC address from CSR3 & CSR4, these CSRs reflects real value
    // stored with EEPROM.
    //
    RTMP_IO_READ32(pAd, CSR3, &StaMacReg0.word);
    RTMP_IO_READ32(pAd, CSR4, &StaMacReg1.word);
    // Set Current address
    pAd->CurrentAddress[0] = StaMacReg0.field.Byte0;
    pAd->CurrentAddress[1] = StaMacReg0.field.Byte1;
    pAd->CurrentAddress[2] = StaMacReg0.field.Byte2;
    pAd->CurrentAddress[3] = StaMacReg0.field.Byte3;
    pAd->CurrentAddress[4] = StaMacReg1.field.Byte4;
    pAd->CurrentAddress[5] = StaMacReg1.field.Byte5;
    pAd->PermanentAddress[0] = StaMacReg0.field.Byte0;
    pAd->PermanentAddress[1] = StaMacReg0.field.Byte1;
    pAd->PermanentAddress[2] = StaMacReg0.field.Byte2;
    pAd->PermanentAddress[3] = StaMacReg0.field.Byte3;
    pAd->PermanentAddress[4] = StaMacReg1.field.Byte4;
    pAd->PermanentAddress[5] = StaMacReg1.field.Byte5;

    return Status;
}

/*
    ========================================================================

    Routine Description:
        Read initial parameters from EEPROM

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:
        
    ========================================================================
*/
VOID    NICReadEEPROMParameters(
    IN  PRTMP_ADAPTER   pAdapter)
{
    ULONG           data;
    USHORT          i, value;
    UCHAR		TmpPhy;
    EEPROM_TX_PWR_STRUC Power;
    EEPROM_VERSION_STRUC    Version;
    EEPROM_ANTENNA_STRUC	Antenna;

    DBGPRINT(RT_DEBUG_TRACE, "--> NICReadEEPROMParameters\n");

    // Init EEPROM Address Number, before access EEPROM; if 93c46, EEPROMAddressNum=6, else if 93c66, EEPROMAddressNum=8
    RTMP_IO_READ32(pAdapter, CSR21, &data);

    if(data & 0x20)
        pAdapter->EEPROMAddressNum = 6;     
    else
        pAdapter->EEPROMAddressNum = 8;

    // if E2PROM version mismatch with driver's expectation, then skip
    // all subsequent E2RPOM retieval and set a system error bit to notify GUI
    Version.word = RTMP_EEPROM_READ16(pAdapter, EEPROM_VERSION_OFFSET);
    if (Version.field.Version != VALID_EEPROM_VERSION)
    {
        DBGPRINT(RT_DEBUG_ERROR, "WRONG E2PROM VERSION %d, should be %d\n",Version.field.Version, VALID_EEPROM_VERSION);
        pAdapter->PortCfg.SystemErrorBitmap |= 0x00000001;
        return;
    }

    // Read BBP default value from EEPROM and store to array(EEPROMDefaultValue) in pAdapter
    for(i = 0; i < NUM_EEPROM_BBP_PARMS; i++)
    {
        value = RTMP_EEPROM_READ16(pAdapter, EEPROM_BBP_BASE_OFFSET + i*2);
        
        pAdapter->EEPROMDefaultValue[i] = value;
    }

#if 1
    // We have to parse NIC configuration 0 at here.
	// If TSSI did not have preloaded value, it should reset the TxAutoAgc to false
	// Therefore, we have to read TxAutoAgc control beforehand.
	// Read Tx AGC control bit
	Antenna.word = pAdapter->EEPROMDefaultValue[0];
	if (Antenna.field.DynamicTxAgcControl == 1)
		pAdapter->PortCfg.bAutoTxAgc = TRUE;
	else
		pAdapter->PortCfg.bAutoTxAgc = FALSE;

    // Read Tx power value for all 14 channels
    // Value from 1 - 0x7f. Default value is 24.
    for (i = 0; i < NUM_EEPROM_TX_PARMS; i++)
    {
        Power.word = RTMP_EEPROM_READ16(pAdapter, EEPROM_TX_PWR_OFFSET + i*2);
        pAdapter->PortCfg.ChannelTxPower[i * 2]     = ((Power.field.Byte0 > 32) ? 24 : Power.field.Byte0);
        pAdapter->PortCfg.ChannelTxPower[i * 2 + 1] = ((Power.field.Byte1 > 32) ? 24 : Power.field.Byte1);
    }

    // Read Tx TSSI reference value, OK to reuse Power data structure
	for (i = 0; i < NUM_EEPROM_TX_PARMS; i++)
	{
		Power.word = RTMP_EEPROM_READ16(pAdapter, EEPROM_TSSI_REF_OFFSET + i * 2);
		pAdapter->PortCfg.ChannelTssiRef[i * 2]     = Power.field.Byte0;
		pAdapter->PortCfg.ChannelTssiRef[i * 2 + 1] = Power.field.Byte1;
		// Disable TxAgc if the value is not right
		if ((pAdapter->PortCfg.ChannelTssiRef[i * 2] == 0xff) ||
			(pAdapter->PortCfg.ChannelTssiRef[i * 2 + 1] == 0xff))
			pAdapter->PortCfg.bAutoTxAgc = FALSE;					
	}
	
	// Tx Tssi delta offset 0x24
	Power.word = RTMP_EEPROM_READ16(pAdapter, EEPROM_TSSI_DELTA_OFFSET);
	pAdapter->PortCfg.ChannelTssiDelta = Power.field.Byte0;
	
#endif

    //CountryRegion byte offset = 0x35
    value = pAdapter->EEPROMDefaultValue[2] >> 8;
    if ((value <= 7))
    {
         pAdapter->PortCfg.CountryRegion = (UCHAR) value;
         TmpPhy = pAdapter->PortCfg.PhyMode;
         pAdapter->PortCfg.PhyMode = 0xff;
         RTMPSetPhyMode(pAdapter, TmpPhy);
    }

    // Read new Calibrated CV value, reuse the power variable
    Power.word = RTMP_EEPROM_READ16(pAdapter, EEPROM_CALIBRATE_OFFSET);
    if (Power.field.Byte0 == 0xff)
    {
	//pAdapter->PortCfg.R17Dec = 0;
	pAdapter->PortCfg.RssiToDbm = 0x79;
    }
    else
    { 
	//pAdapter->PortCfg.R17Dec = 0x79 - Power.field.Byte0;
	pAdapter->PortCfg.RssiToDbm = Power.field.Byte0;
    }
	

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICReadEEPROMParameters\n");
}

/*
    ========================================================================

    Routine Description:
        Set default value from EEPROM

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:

    ========================================================================
*/
VOID    NICInitAsicFromEEPROM(
    IN  PRTMP_ADAPTER   pAdapter)
{
	ULONG					data, BbpCsr1;
	USHORT					i, value;
	UCHAR					TxValue,RxValue;
	EEPROM_ANTENNA_STRUC	Antenna;
	EEPROM_NIC_CONFIG2_STRUC    NicConfig2;

	DBGPRINT(RT_DEBUG_TRACE, "--> NICInitAsicFromEEPROM\n");
	
	for(i = 3; i < NUM_EEPROM_BBP_PARMS; i++)
	{
		value = pAdapter->EEPROMDefaultValue[i];
		
		if((value != 0xFFFF) && (value != 0))
		{
			data = value | 0x18000;
			RTMP_BBP_IO_WRITE32(pAdapter, data);
		}
	}

	Antenna.word = pAdapter->EEPROMDefaultValue[0];
        DBGPRINT(RT_DEBUG_TRACE, "--> Antenna.word = 0x%x\n",pAdapter->EEPROMDefaultValue[0]);

	if ((Antenna.word == 0xFFFF) || (Antenna.field.TxDefaultAntenna > 2) || (Antenna.field.RxDefaultAntenna > 2))
	{
    	DBGPRINT(RT_DEBUG_TRACE, "E2PROM error(=0x%04x), hard code as 0x0002\n", Antenna.word);
    	Antenna.word = 0x0002;
	}

	pAdapter->PortCfg.NumberOfAntenna = 2;  // (UCHAR)Antenna.field.NumOfAntenna;
	pAdapter->PortCfg.CurrentTxAntenna = (UCHAR)Antenna.field.TxDefaultAntenna;
	pAdapter->PortCfg.CurrentRxAntenna = (UCHAR)Antenna.field.RxDefaultAntenna;
    pAdapter->PortCfg.RfType = (UCHAR) Antenna.field.RfType;

	RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, BBP_Tx_Configure, &TxValue);
	RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, BBP_Rx_Configure, &RxValue);
	RTMP_IO_READ32(pAdapter, BBPCSR1, &BbpCsr1);

	// Tx antenna select
    if(Antenna.field.TxDefaultAntenna == 1)       // Antenna A
    {
		TxValue = (TxValue & 0xFC) | 0x00; 
		BbpCsr1 = (BbpCsr1 & 0xFFFCFFFC) | 0x00000000;
	}
	else if(Antenna.field.TxDefaultAntenna == 2)  // Antenna B
	{
		TxValue = (TxValue & 0xFC) | 0x02; 
		BbpCsr1 = (BbpCsr1 & 0xFFFCFFFC) | 0x00020002;
	}
	else                                          // diverity - start from Antenna B
	{
		TxValue = (TxValue & 0xFC) | 0x02;
		BbpCsr1 = (BbpCsr1 & 0xFFFCFFFC) | 0x00020002;
	}

	// Rx antenna select
	if(Antenna.field.RxDefaultAntenna == 1)       // Antenna A
		RxValue = (RxValue & 0xFC) | 0x00; 
	else if(Antenna.field.RxDefaultAntenna == 2)  // Antenna B
		RxValue = (RxValue & 0xFC) | 0x02; 
	else                                          // Antenna Diversity
		RxValue = (RxValue & 0xFC) | 0x02; 
			
    // RT5222 needs special treatment to swap TX I/Q
    if (pAdapter->PortCfg.RfType == RFIC_5222)
    {
        BbpCsr1 |= 0x00040004;
        TxValue |= 0x04;         // TX I/Q flip
    }
    // RT2525E need to flip TX I/Q but not RX I/Q
    else if (pAdapter->PortCfg.RfType == RFIC_2525E)	
    {
        BbpCsr1 |= 0x00040004;
        TxValue |= 0x04;         // TX I/Q flip
        RxValue &= 0xfb;         // RX I/Q no flip
    }
    
	// Change to match microsoft definition, 0xff: diversity, 0: A, 1: B
	pAdapter->PortCfg.CurrentTxAntenna--;
	pAdapter->PortCfg.CurrentRxAntenna--;

	RTMP_IO_WRITE32(pAdapter, BBPCSR1, BbpCsr1);
	RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, BBP_Tx_Configure, TxValue);
	RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, BBP_Rx_Configure, RxValue);
	
    // 2003-12-16 software-based RX antenna diversity
    // pAdapter->PortCfg.CurrentRxAntenna = 0xff;   // Diversity ON
    AsicSetRxAnt(pAdapter);

	if (Antenna.field.LedMode == LED_MODE_TXRX_ACTIVITY)
	    pAdapter->PortCfg.LedMode = LED_MODE_TXRX_ACTIVITY;
    else if (Antenna.field.LedMode == LED_MODE_SINGLE)
   	{
        pAdapter->PortCfg.LedMode = LED_MODE_SINGLE;
    	ASIC_LED_ACT_ON(pAdapter);
    }
    else if (Antenna.field.LedMode == LED_MODE_ASUS)
   	{
        pAdapter->PortCfg.LedMode = LED_MODE_ASUS;
		RTMP_IO_WRITE32(pAdapter, LEDCSR, 0x0002461E);
    }
	else
	    pAdapter->PortCfg.LedMode = LED_MODE_DEFAULT;

   	// Read Hardware controlled Radio state enable bit
    if (0 && Antenna.field.HardwareRadioControl == 1)
	{
	    pAdapter->PortCfg.bHardwareRadio = TRUE;
		
	    // Read GPIO pin0 as Hardware controlled radio state
	    RTMP_IO_READ32(pAdapter, GPIOCSR, &data);
	    if ((data & 0x01) == 0)
	    {
		    pAdapter->PortCfg.bHwRadio = FALSE;
		    pAdapter->PortCfg.bRadio = FALSE;
		    RTMP_IO_WRITE32(pAdapter, PWRCSR0, 0x00000000);
		    RTMP_SET_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF);
		    if (pAdapter->PortCfg.LedMode == LED_MODE_ASUS)
			{
			    // Turn bit 17 for Radio OFF
			    RTMP_IO_WRITE32(pAdapter, LEDCSR, 0x0000461E);
		    }
	    }
    }
    else
	    pAdapter->PortCfg.bHardwareRadio = FALSE;		
	
	NicConfig2.word = pAdapter->EEPROMDefaultValue[1];
	if (NicConfig2.word == 0xffff)
	    NicConfig2.word = 0;    // empty E2PROM, use default

	// for dynamic BBP R17:RX sensibility tuning
	{
	    UCHAR r17;
	    RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, 17, &r17);
	    pAdapter->PortCfg.BbpTuningEnable = (NicConfig2.field.DynamicBbpTuning==0)? 1:0;
	    pAdapter->PortCfg.VgcLowerBound   = r17;

        // 2004-3-4 per David's request, R7 starts at upper bound
        pAdapter->PortCfg.BbpTuning.VgcUpperBound = BBP_R17_DYNAMIC_UP_BOUND;
	    r17 = pAdapter->PortCfg.BbpTuning.VgcUpperBound;
	    pAdapter->PortCfg.LastR17Value = r17;
	    RTMP_BBP_IO_WRITE32_BY_REG_ID(pAdapter, 17, r17);

        // 2004-2-2 per David's request, lower R17 low-bound for very good quality NIC
	    pAdapter->PortCfg.VgcLowerBound -= 6;  
	    DBGPRINT(RT_DEBUG_TRACE,"R17 tuning enable=%d, R17=0x%02x, range=<0x%02x, 0x%02x>\n",
	        pAdapter->PortCfg.BbpTuningEnable, r17, pAdapter->PortCfg.VgcLowerBound, pAdapter->PortCfg.BbpTuning.VgcUpperBound);
	}

   	DBGPRINT(RT_DEBUG_TRACE, "RF IC=%d, LED mode=%d\n", pAdapter->PortCfg.RfType, pAdapter->PortCfg.LedMode);

	DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitAsicFromEEPROM\n");
}

void NICInitializeAdapter(IN    PRTMP_ADAPTER   pAdapter)
{
    TXCSR2_STRUC    TxCSR2;
    RXCSR1_STRUC    RxCSR1;
    ULONG           Value;

    DBGPRINT(RT_DEBUG_TRACE, "--> NICInitializeAdapter\n");

    // Init spin locks
    spin_lock_init(&pAdapter->TxRingLock);
    spin_lock_init(&pAdapter->PrioRingLock);
    spin_lock_init(&pAdapter->AtimRingLock);
    spin_lock_init(&pAdapter->RxRingLock);
    spin_lock_init(&pAdapter->TxSwQueueLock);
    spin_lock_init(&pAdapter->MemLock);

    // Write TXCSR2 register
    TxCSR2.field.TxDSize = RING_DESCRIPTOR_SIZE;
    TxCSR2.field.NumTxD  = TX_RING_SIZE;
    TxCSR2.field.NumAtimD  = ATIM_RING_SIZE;
    TxCSR2.field.NumPrioD  = PRIO_RING_SIZE;    
    RTMP_IO_WRITE32(pAdapter, TXCSR2, TxCSR2.word);

    // Write TXCSR3 register
    Value = pAdapter->TxRing[0].pa_addr;
    RTMP_IO_WRITE32(pAdapter, TX_RING_BASE_REG, Value);

    // Write TXCSR4 register
    Value = pAdapter->PrioRing[0].pa_addr;
    RTMP_IO_WRITE32(pAdapter, PRIO_RING_BASE_REG, Value);

    // Write TXCSR5 register
    Value = pAdapter->AtimRing[0].pa_addr;
    RTMP_IO_WRITE32(pAdapter, ATIM_RING_BASE_REG, Value);

    // Write TXCSR6 register
    Value = pAdapter->BeaconRing.pa_addr;
    RTMP_IO_WRITE32(pAdapter, BEACON_BASE_REG, Value);

    // Write RXCSR1 register
    RxCSR1.field.RxDSize = RING_DESCRIPTOR_SIZE;
    RxCSR1.field.NumRxD  = RX_RING_SIZE;
    RTMP_IO_WRITE32(pAdapter, RXCSR1, RxCSR1.word);
    
    // Write RXCSR2 register
    Value = pAdapter->RxRing[0].pa_addr;
    RTMP_IO_WRITE32(pAdapter, RX_RING_BASE_REG, Value);

    // Write CSR1 for host ready
    // Move Host reay to end of ASIC initialization 
    // to ensure no Rx will perform before ASIC init
    // RTMP_IO_WRITE32(pAdapter, CSR1, 0x4);

    // Initialze ASIC for TX & Rx operation
    NICInitializeAsic(pAdapter);

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitializeAdapter\n");
}

void    NICInitializeAsic(IN    PRTMP_ADAPTER   pAdapter)
{
    ULONG           Index;
    UCHAR           Value = 0xff;

    DBGPRINT(RT_DEBUG_TRACE, "--> NICInitializeAsic\n");

    // Initialize MAC register to default value
    for (Index = 0; Index < NUM_MAC_REG_PARMS; Index++)
    {
        RTMP_IO_WRITE32(pAdapter, MACRegTable[Index].Register, MACRegTable[Index].Value);
    }

    // Set Host ready before kicking Rx
    RTMP_IO_WRITE32(pAdapter, CSR1, 0x1); // reset MAC state machine, requested by Kevin 2003-2-11
    RTMP_IO_WRITE32(pAdapter, CSR1, 0x4);

	// Read BBP register, make sure BBP is up and running before write new data
	Index = 0;
	while (((Value == 0xff) || (Value == 0x00)) && Index  < 10)
	{
		RTMP_BBP_IO_READ32_BY_REG_ID(pAdapter, BBP_Version, &Value);
		mdelay(1);
		Index++;
	}
	if(Index >= 10){
		DBGPRINT(RT_DEBUG_TRACE, "BBP register was never ready!\n");
		return;
	}

    // Initialize BBP register to default value
    for (Index = 0; Index < NUM_BBP_REG_PARMS; Index++)
    {
        RTMP_BBP_IO_WRITE32(pAdapter, BBPRegTable[Index]);
    }

    // Initialize RF register to default value
    AsicSwitchChannel(pAdapter, pAdapter->PortCfg.Channel);
    AsicLockChannel(pAdapter, pAdapter->PortCfg.Channel);

    // Add radio off control
    if (pAdapter->PortCfg.bRadio == FALSE)
    {
        RTMP_IO_WRITE32(pAdapter, PWRCSR0, 0x00000000);
        RTMP_SET_FLAG(pAdapter, fRTMP_ADAPTER_RADIO_OFF);
    }

    // Kick Rx
    if (pAdapter->bAcceptPromiscuous == TRUE)
    {
        // Register bits with "drop unicast not to me disabled"
        RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x6e); 
    }
    else
    {
        // Standard default register bits.
        RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x7e); 
    }  

    // Clear old FCS jitter before init ASIC
    RTMP_IO_READ32(pAdapter, CNT0, &Index);
    // Clear old Rx FIFO error jitter before init ASIC
    RTMP_IO_READ32(pAdapter, CNT4, &Index);

    pAdapter->MediaState=NdisMediaStateDisconnected;

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitializeAsic\n");
}

/*
    ========================================================================

    Routine Description:
        Reset NIC Asics

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:
        Reset NIC to initial state AS IS system boot up time.

    ========================================================================
*/
VOID    NICIssueReset(
    IN  PRTMP_ADAPTER   pAdapter)
{
    CSR3_STRUC      StaMacReg0;
    CSR4_STRUC      StaMacReg1;

    DBGPRINT(RT_DEBUG_TRACE, "--> NICIssueReset\n");

    // Abort Tx, prevent ASIC from writing to Host memory
    RTMP_IO_WRITE32(pAdapter, TXCSR0, 0x08);

    // Disable Rx, register value supposed will remain after reset
    RTMP_IO_WRITE32(pAdapter, RXCSR0, 0x01);

    if (pAdapter->PortCfg.bLocalAdminMAC)
    {
        // Write Back Permanent MAC address to CSR3 & CSR4
        StaMacReg0.field.Byte0 = pAdapter->PermanentAddress[0];
        StaMacReg0.field.Byte1 = pAdapter->PermanentAddress[1];
        StaMacReg0.field.Byte2 = pAdapter->PermanentAddress[2];
        StaMacReg0.field.Byte3 = pAdapter->PermanentAddress[3];
        StaMacReg1.field.Byte4 = pAdapter->PermanentAddress[4];
        StaMacReg1.field.Byte5 = pAdapter->PermanentAddress[5];
        RTMP_IO_WRITE32(pAdapter, CSR3, StaMacReg0.word);
        RTMP_IO_WRITE32(pAdapter, CSR4, StaMacReg1.word);
    }

    // Issue reset and clear from reset state
    RTMP_IO_WRITE32(pAdapter, CSR1, 0x01);
    RTMP_IO_WRITE32(pAdapter, CSR1, 0x00);

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICIssueReset\n");
}

/*
    ========================================================================

    Routine Description:
        Check ASIC registers and find any reason the system might hang

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:


    ========================================================================
*/
BOOLEAN NICCheckForHang(
    IN  PRTMP_ADAPTER   pAd)
{
    // garbage collection
//    if ((pAd->RalinkCounters.EncryptCount - pAd->RalinkCounters.TxDoneCount) == TX_RING_SIZE)
//    {
//        DBGPRINT(RT_DEBUG_WARNING,"\nNICCheckForHang --- Garbage Collection!!!\n\n");
//    }

    {
        RTMPHandleTxRingTxDoneInterrupt(pAd);
        RTMPHandleEncryptionDoneInterrupt(pAd);
    }

    return (FALSE);
}

/*
    ========================================================================

    Routine Description:
        Reset NIC from error

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:
        Reset NIC from error state

    ========================================================================
*/
VOID    NICResetFromError(
    IN  PRTMP_ADAPTER   pAdapter)
{
    // Reset BBP (according to alex, reset ASIC will force reset BBP
    // Therefore, skip the reset BBP
    // RTMP_IO_WRITE32(pAdapter, CSR1, 0x2);
    // Release BBP reset
    // RTMP_IO_WRITE32(pAdapter, CSR1, 0x0);

    RTMP_IO_WRITE32(pAdapter, CSR1, 0x1);
    // Remove ASIC from reset state
    RTMP_IO_WRITE32(pAdapter, CSR1, 0x0);

    // Init send data structures and related parameters
    NICInitTransmit(pAdapter);

    NICInitializeAdapter(pAdapter); 
    NICInitAsicFromEEPROM(pAdapter);

    // Switch to current channel, since during reset process, the connection should remains on. 
    AsicSwitchChannel(pAdapter, pAdapter->PortCfg.Channel);
    AsicLockChannel(pAdapter, pAdapter->PortCfg.Channel);
}
/*
    ========================================================================

    Routine Description:
        Verify section is valid for Get key parameter.

    Arguments:
        buffer                      Pointer to the buffer to start find the key section
        ptr                         pointer to section

    Return Value:
        TRUE                        Success
        FALSE                       Fail
    ========================================================================
*/
INT RTMPIsFindSection(
    IN  PUCHAR  ptr,
    IN  PUCHAR  buffer)
{
    if(ptr == buffer)
        return TRUE;
    else if (ptr > buffer) 
    {
        while (ptr > buffer)
        {
            ptr--;
            if( *ptr == 0x0a)
                return TRUE;
            else if( (*ptr == ' ') || (*ptr == '\t'))
                continue;
            else
                return FALSE;
        }
        return TRUE;
    }

    return FALSE;
}
/*
    ========================================================================

    Routine Description:
        Find key section for Get key parameter.

    Arguments:
        buffer                      Pointer to the buffer to start find the key section
        section                     the key of the secion to be find

    Return Value:
        NULL                        Fail
        Others                      Success
    ========================================================================
*/
PUCHAR  RTMPFindSection(
    IN  PCHAR   buffer,
    IN  PCHAR   section)
{
    CHAR temp_buf[255];
    PUCHAR  ptr;

    strcpy(temp_buf, "[");                  /*  and the opening bracket [  */
    strcat(temp_buf, section);
    strcat(temp_buf, "]");

    if((ptr = rtstrstr(buffer, temp_buf)) != NULL)
    {
        if(RTMPIsFindSection(ptr, buffer))
            return (ptr+strlen("\n"));
        else
            return NULL;
    }
    else
        return NULL;
}   
 /**
  * strstr - Find the first substring in a %NUL terminated string
  * @s1: The string to be searched
  * @s2: The string to search for
  */
char * rtstrstr(const char * s1,const char * s2)
{
         int l1, l2;
 
         l2 = strlen(s2);
         if (!l2)
                 return (char *) s1;
         l1 = strlen(s1);
         while (l1 >= l2) {
                 l1--;
                 if (!memcmp(s1,s2,l2))
                         return (char *) s1;
                 s1++;
         }
         return NULL;
}
/*
    ========================================================================

    Routine Description:
        Get key parameter.

    Arguments:
        section                     the key of the secion
        key                         Pointer to key string
        dest                        Pointer to destination      
        destsize                    The datasize of the destination
        buffer                      Pointer to the buffer to start find the key

    Return Value:
        TRUE                        Success
        FALSE                       Fail

    Note:
        This routine get the value with the matched key (case case-sensitive)
    ========================================================================
*/
INT RTMPGetKeyParameter(
    IN  PUCHAR  section,
    IN  PCHAR   key,
    OUT PCHAR   dest,   
    IN  INT     destsize,
    IN  PCHAR   buffer)
{
    char temp_buf1[600];
    char temp_buf2[600];
    char *start_ptr;
    char *end_ptr;
    char *ptr;
    char *too_far_ptr;
    char *offset;
    int  len;

    //find section
    if((offset = RTMPFindSection(buffer, section)) == NULL)
        return (FALSE);

    strcpy(temp_buf1, "\n");
    strcat(temp_buf1, key);
    strcat(temp_buf1, "=");

    //search key
    if((start_ptr=rtstrstr(offset, temp_buf1))==NULL)
        return FALSE;

    start_ptr+=strlen("\n");
    if((too_far_ptr=rtstrstr(offset+1, "["))==NULL)
       too_far_ptr=offset+strlen(offset);

    if((end_ptr=rtstrstr(start_ptr, "\n"))==NULL)
       end_ptr=start_ptr+strlen(start_ptr);

    if (too_far_ptr<start_ptr)
        return FALSE;

    memcpy(temp_buf2, start_ptr, end_ptr-start_ptr);
    temp_buf2[end_ptr-start_ptr]='\0';
    len = strlen(temp_buf2);
    strcpy(temp_buf1, temp_buf2);
    if((start_ptr=rtstrstr(temp_buf1, "=")) == NULL)
        return FALSE;

    strcpy(temp_buf2, start_ptr+1);
    ptr = temp_buf2;
    //trim space or tab
    while(*ptr != 0x00)
    {
        if( (*ptr == ' ') || (*ptr == '\t') )
            ptr++;
        else
           break;
    }

    len = strlen(ptr);    
    memset(dest, 0x00, destsize);
    strncpy(dest, ptr, len >= destsize ?  destsize: len);

    return TRUE;
}
/*
    ========================================================================

    Routine Description:
        In kernel mode read parameters from file

    Arguments:
        src                     the location of the file.
        dest                        put the parameters to the destination.
        Length                  size to read.

    Return Value:
        None

    Note:

    ========================================================================
*/

VOID RTMPReadParametersFromFile(
    IN  PRTMP_ADAPTER pAd)
{
    PUCHAR                                  src;
    struct file                             *srcf;
    INT                                     retval, orgfsuid, orgfsgid;
    mm_segment_t                            orgfs;
    CHAR                                    buffer[MAX_INI_BUFFER_SIZE];
    CHAR                                    tmpbuf[255];
    UCHAR                                   keyMaterial[40];
    UCHAR                                   Channel;
    ULONG                                   ulInfo;
    RT_802_11_PREAMBLE                      Preamble;
    int                                     KeyLen;
    int                                     i;
    BOOLEAN                                 bIsHex = TRUE;
	ULONG	rate_mapping[12] = {1, 2, 5, 11, 6, 9, 12, 18, 24, 36, 48, 54}; //according to README

    src = PROFILE_PATH;

    // Save uid and gid used for filesystem access.
    // Set user and group to 0 (root)   
    orgfsuid = current->fsuid;
    orgfsgid = current->fsgid;
    current->fsuid=current->fsgid = 0;
    orgfs = get_fs();
    set_fs(KERNEL_DS);

    if (src && *src)
    {
        srcf = filp_open(src, O_RDONLY, 0);
        if (IS_ERR(srcf))
        {
            DBGPRINT(RT_DEBUG_TRACE, "--> Error %ld opening %s\n", -PTR_ERR(srcf),src);
        }
        else
        {
            /* The object must have a read method */
            if (srcf->f_op && srcf->f_op->read) 
            {
                memset(buffer, 0x00, MAX_INI_BUFFER_SIZE);
                retval=srcf->f_op->read(srcf, buffer, MAX_INI_BUFFER_SIZE, &srcf->f_pos);
                if (retval < 0)
                {
                    DBGPRINT(RT_DEBUG_TRACE, "--> Read %s error %d\n", src, -retval);
                }
                else
                {
                    // set file parameter to portcfg
                    //CountryRegion
                    if (RTMPGetKeyParameter("Default", "CountryRegion", tmpbuf, 255, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);
                        if ((ulInfo >= REGION_MIN) && (ulInfo <= REGION_MAX) )
                        {
                            pAd->PortCfg.CountryRegion = (UCHAR) ulInfo;
                            DBGPRINT(RT_DEBUG_TRACE, "%s::(CountryRegion=%d)\n", __FUNCTION__, pAd->PortCfg.CountryRegion);
                        }
                    }
                    //SSID
                    memset(tmpbuf, 0x00, 255);
                    if (RTMPGetKeyParameter("Default", "SSID", tmpbuf, 32, buffer))
                    {
                        pAd->PortCfg.SsidLen = (UCHAR) strlen(tmpbuf);
                        memcpy(pAd->PortCfg.Ssid, tmpbuf, pAd->PortCfg.SsidLen);

                        pAd->Mlme.CntlAux.SsidLen = pAd->PortCfg.SsidLen;
                        memcpy(pAd->Mlme.CntlAux.Ssid, tmpbuf, pAd->Mlme.CntlAux.SsidLen);

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(SSID=%s Len=%d)\n", __FUNCTION__, tmpbuf, pAd->PortCfg.SsidLen);
                    }
                    //NetworkType
                    if (RTMPGetKeyParameter("Default", "NetworkType", tmpbuf, 255, buffer))
                    {
                        pAd->bConfigChanged = TRUE;
                        if (strcmp(tmpbuf, "Adhoc") == 0)
                            pAd->PortCfg.BssType = BSS_INDEP;
                        else //Default Infrastructure mode
                            pAd->PortCfg.BssType = BSS_INFRA;
                        // Reset Ralink supplicant to not use, it will be set to start when UI set PMK key
                        pAd->PortCfg.WpaState = SS_NOTUSE;
                        DBGPRINT(RT_DEBUG_TRACE, "%s::(NetworkType=%d)\n", __FUNCTION__, pAd->PortCfg.BssType);
                    }
                    //WirelessMode
                    if (RTMPGetKeyParameter("Default", "WirelessMode", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);
                        if ((ulInfo == PHY_11BG_MIXED) || (ulInfo == PHY_11B) ||
                            (ulInfo == PHY_11A) || (ulInfo == PHY_11ABG_MIXED))
                        {
                            RTMPSetPhyMode(pAd, ulInfo);
                            DBGPRINT(RT_DEBUG_TRACE, "%s::(WirelessMode=%d)\n", __FUNCTION__, ulInfo);
                        }
                    }
                    //TxRate
                    if (RTMPGetKeyParameter("Default", "TxRate", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);
                        {
                            if (ulInfo == 0)
                                RTMPSetDesiredRates(pAd, -1);
                            else
                                RTMPSetDesiredRates(pAd, (LONG) (rate_mapping[ulInfo-1] * 1000000));

                            DBGPRINT(RT_DEBUG_TRACE, "%s::(TxRate=%d)\n", __FUNCTION__, ulInfo);
                        }
                    }
                    //Channel
                    if (RTMPGetKeyParameter("Default", "Channel", tmpbuf, 10, buffer))
                    {
                        Channel = (UCHAR) simple_strtol(tmpbuf, 0, 10);
                        if (ChannelSanity(pAd, Channel) == TRUE)
                        {
                            pAd->PortCfg.Channel = Channel;
                            // If default profile in Registry is an ADHOC network, driver should use the specified channel 
                            // number when starting IBSS the first time, because RaConfig is passive and will not set this
                            // via OID_802_11_CONFIGURATION upon driver bootup.
                            pAd->PortCfg.IbssConfig.Channel = pAd->PortCfg.Channel;
                            DBGPRINT(RT_DEBUG_TRACE, "%s::(Channel=%d)\n", __FUNCTION__, Channel);
                        }
                    }
                    //BGProtection
                    if (RTMPGetKeyParameter("Default", "BGProtection", tmpbuf, 10, buffer))
                    {
                        switch (simple_strtol(tmpbuf, 0, 10))
                        {
                            case 1: //Always On
                                pAd->PortCfg.UseBGProtection = 1;
                                break;
                            case 2: //Always OFF
                                pAd->PortCfg.UseBGProtection = 2;
                                break;
                            case 0: //AUTO
                            default:
                                pAd->PortCfg.UseBGProtection = 0;
                                break;
                        }
                        DBGPRINT(RT_DEBUG_TRACE, "%s::(BGProtection=%d)\n", __FUNCTION__, pAd->PortCfg.UseBGProtection);
                    }
                    //StaWithEtherBridge
                    if (RTMPGetKeyParameter("Default", "StaWithEtherBridge", tmpbuf, 10, buffer))
                    {
                        switch (simple_strtol(tmpbuf, 0, 10))
                        {
                            case 0: //Off
                                pAd->PortCfg.StaWithEtherBridge.Enable = FALSE;
                                break;
                            case 1: //On
                                pAd->PortCfg.StaWithEtherBridge.Enable = TRUE;
                                break;
                            default:
                                pAd->PortCfg.StaWithEtherBridge.Enable = FALSE;
                                break;
                        }
                        DBGPRINT(RT_DEBUG_TRACE, "%s::(StaWithEtherBridge=%d)\n", __FUNCTION__, pAd->PortCfg.StaWithEtherBridge.Enable);
                    }
                    //TxPreamble
                    if (RTMPGetKeyParameter("Default", "TxPreamble", tmpbuf, 10, buffer))
                    {
                        Preamble = simple_strtol(tmpbuf, 0, 10);
                        switch (Preamble)
                        {
                            case Rt802_11PreambleShort:
                                pAd->PortCfg.WindowsTxPreamble = Preamble;
                                MlmeSetTxPreamble(pAd, Rt802_11PreambleShort);
                                break;
                            case Rt802_11PreambleLong:
                            case Rt802_11PreambleAuto:
                            default:
                                // if user wants AUTO, initialize to LONG here, then change according to AP's
                                // capability upon association.
                                pAd->PortCfg.WindowsTxPreamble = Preamble;
                                MlmeSetTxPreamble(pAd, Rt802_11PreambleLong);
                        }
                        DBGPRINT(RT_DEBUG_TRACE, "%s::(TxPreamble=%d)\n", __FUNCTION__, Preamble);
                    }
                    //RTSThreshold
                    if (RTMPGetKeyParameter("Default", "RTSThreshold", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);

                        if((ulInfo > 0) && (ulInfo <= MAX_RTS_THRESHOLD))
                            pAd->PortCfg.RtsThreshold = (USHORT)ulInfo;
                        else 
                            pAd->PortCfg.RtsThreshold = MAX_RTS_THRESHOLD;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(RTSThreshold=%d)\n", __FUNCTION__, pAd->PortCfg.RtsThreshold);
                    }
                    //FragThreshold
                    if (RTMPGetKeyParameter("Default", "FragThreshold", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);

                        if ( (ulInfo >= MIN_FRAG_THRESHOLD) && (ulInfo <= MAX_FRAG_THRESHOLD))
                            pAd->PortCfg.FragmentThreshold = (USHORT)ulInfo;
                        else
                            pAd->PortCfg.FragmentThreshold = MAX_FRAG_THRESHOLD;

                        if (pAd->PortCfg.FragmentThreshold == MAX_FRAG_THRESHOLD)
                            pAd->PortCfg.bFragmentZeroDisable = TRUE;
                        else
                            pAd->PortCfg.bFragmentZeroDisable = FALSE;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(FragThreshold=%d)\n", __FUNCTION__, ulInfo);
                    }
                    //AdhocOfdm
                    if (RTMPGetKeyParameter("Default", "AdhocOfdm", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);

                        if (ulInfo == 1)
                            pAd->PortCfg.AdhocMode = 1;
                        else
                            pAd->PortCfg.AdhocMode = 0;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(AdhocOfdm=%d)\n", __FUNCTION__, pAd->PortCfg.AdhocMode);
                    }
                    //TxBurst
                    if (RTMPGetKeyParameter("Default", "TxBurst", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);

                        if (ulInfo == 1)
                            pAd->PortCfg.EnableTxBurst = TRUE;
                        else
                            pAd->PortCfg.EnableTxBurst = FALSE;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(TxBurst=%d)\n", __FUNCTION__, pAd->PortCfg.EnableTxBurst);
                    }
                    //TurboRate
                    if (RTMPGetKeyParameter("Default", "TurboRate", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);

                        if (ulInfo == 1)
                            pAd->PortCfg.EnableTurboRate = TRUE;
                        else
                            pAd->PortCfg.EnableTurboRate = FALSE;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(TurboRate=%d)\n", __FUNCTION__, pAd->PortCfg.EnableTurboRate);
                    }
                    //ShortSlot
                    if (RTMPGetKeyParameter("Default", "ShortSlot", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);

                        if (ulInfo == 1)
                            pAd->PortCfg.UseShortSlotTime = TRUE;
                        else
                            pAd->PortCfg.UseShortSlotTime = FALSE;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(ShortSlot=%d)\n", __FUNCTION__, pAd->PortCfg.UseShortSlotTime);
                    }
                    //POWER_MODE
                    if (RTMPGetKeyParameter("Default", "PSMode", tmpbuf, 10, buffer))
                    {
                        if (pAd->PortCfg.BssType == BSS_INFRA)
                        {
                            if ((strcmp(tmpbuf, "MAX_PSP") == 0) || (strcmp(tmpbuf, "max_psp") == 0))
                            {
				DBGPRINT(RT_DEBUG_INFO, "MAX_PSP power mode not available - defaulting to CAM\n");
                            }
                            else if ((strcmp(tmpbuf, "Fast_PSP") == 0) || (strcmp(tmpbuf, "fast_psp") == 0) 
                                || (strcmp(tmpbuf, "FAST_PSP") == 0))
                            {
				DBGPRINT(RT_DEBUG_INFO, "FAST_PSP power mode not available - defaulting to CAM\n");
                            }

                            //Default Ndis802_11PowerModeCAM
                            // clear PSM bit immediately
                            MlmeSetPsmBit(pAd, PWR_ACTIVE);
                            pAd->PortCfg.RecvDtim = TRUE;
                            if (pAd->PortCfg.WindowsACCAMEnable == FALSE)
                                pAd->PortCfg.WindowsPowerMode = Ndis802_11PowerModeCAM;
                            pAd->PortCfg.WindowsBatteryPowerMode = Ndis802_11PowerModeCAM;
                            DBGPRINT(RT_DEBUG_TRACE, "%s::(PSMode=%d)\n", __FUNCTION__, pAd->PortCfg.WindowsPowerMode);
                        }
                    }
                    //AuthMode
                    if (RTMPGetKeyParameter("Default", "AuthMode", tmpbuf, 10, buffer))
                    {
                        if ((strcmp(tmpbuf, "SHARED") == 0) || (strcmp(tmpbuf, "shared") == 0))
                            pAd->PortCfg.AuthMode = Ndis802_11AuthModeShared;
                        else if ((strcmp(tmpbuf, "WPAPSK") == 0) || (strcmp(tmpbuf, "wpapsk") == 0))
                            pAd->PortCfg.AuthMode = Ndis802_11AuthModeWPAPSK;
			else if ((strcmp(tmpbuf, "AUTO") == 0) || (strcmp(tmpbuf, "auto") == 0))
                            pAd->PortCfg.AuthMode = Ndis802_11AuthModeAutoSwitch;
                        else if ((strcmp(tmpbuf, "WPANONE") == 0) || (strcmp(tmpbuf, "wpanone") == 0))
                            pAd->PortCfg.AuthMode = Ndis802_11AuthModeWPANone;
                        else
                            pAd->PortCfg.AuthMode = Ndis802_11AuthModeOpen;

                        pAd->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;
                        DBGPRINT(RT_DEBUG_TRACE, "%s::(AuthMode=%d)\n", __FUNCTION__, pAd->PortCfg.AuthMode);
                    }
                    //EncrypType
                    if (RTMPGetKeyParameter("Default", "EncrypType", tmpbuf, 10, buffer))
                    {
                        if ((strcmp(tmpbuf, "WEP") == 0) || (strcmp(tmpbuf, "wep") == 0))
                            pAd->PortCfg.WepStatus = Ndis802_11WEPEnabled;
                        else if ((strcmp(tmpbuf, "TKIP") == 0) || (strcmp(tmpbuf, "tkip") == 0))
                            pAd->PortCfg.WepStatus = Ndis802_11Encryption2Enabled;
                        else if ((strcmp(tmpbuf, "AES") == 0) || (strcmp(tmpbuf, "aes") == 0))
                            pAd->PortCfg.WepStatus = Ndis802_11Encryption3Enabled;
                        else
                            pAd->PortCfg.WepStatus = Ndis802_11WEPDisabled;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(EncrypType=%d)\n", __FUNCTION__, pAd->PortCfg.WepStatus);
                    }
                    //WPAPSK_KEY
                    if (RTMPGetKeyParameter("Default", "WPAPSK", tmpbuf, 255, buffer))
                    {
                        if ((strlen(tmpbuf) >= 8) && (strlen(tmpbuf) < 64))
                        {
                            PasswordHash((char *)tmpbuf, pAd->PortCfg.Ssid, pAd->PortCfg.SsidLen, keyMaterial);
                            memcpy(pAd->PortCfg.PskKey.Key, keyMaterial, 32);
                            // Use RaConfig as PSK agent.
                            // Start STA supplicant state machine
                            pAd->PortCfg.WpaState = SS_START;
#if 0
                            DBGPRINT(RT_DEBUG_TRACE, "%s WPAPSK Key => \n", __FUNCTION__);
                            for (i = 0; i < 32; i++)
                            {
                                DBGPRINT(RT_DEBUG_TRACE, "%02x:", pAd->PortCfg.PskKey.Key[i]);
                                if (i%16 == 15)
                                    DBGPRINT(RT_DEBUG_TRACE, "\n");
                            }
                            DBGPRINT(RT_DEBUG_TRACE, "\n");
#endif
                        }
                    }
                    else if (strlen(tmpbuf) == 64)
                    {
                           AtoH(tmpbuf, pAd->PortCfg.PskKey.Key, 32);
                           pAd->PortCfg.WpaState = SS_START;
                    }
                    //DefaultKeyID
                    if (RTMPGetKeyParameter("Default", "DefaultKeyID", tmpbuf, 10, buffer))
                    {
                        ulInfo = simple_strtol(tmpbuf, 0, 10);
                        if((ulInfo >= 1 ) && (ulInfo <= 4))
                            pAd->PortCfg.DefaultKeyId = (UCHAR) (ulInfo - 1 );
                        else
                            pAd->PortCfg.DefaultKeyId = 0;

                        DBGPRINT(RT_DEBUG_TRACE, "%s::(DefaultKeyID=%d)\n", __FUNCTION__, pAd->PortCfg.DefaultKeyId);
                    }
                    //Key1Str
                    if (RTMPGetKeyParameter("Default", "Key1Str", tmpbuf, 26, buffer))
                    {
                        KeyLen = strlen(tmpbuf);
                        switch (KeyLen)
                        {
                            case 0:
                                pAd->PortCfg.SharedKey[0].KeyLen = 0;
                                break;
                            case 5: //wep 40 Ascii type
                                pAd->PortCfg.SharedKey[0].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[0].Key, tmpbuf, KeyLen);
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key1=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                            case 10: //wep 40 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }
                                
                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[0].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[0].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key1=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            case 13: //wep 104 Ascii type
                                pAd->PortCfg.SharedKey[0].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[0].Key, tmpbuf, KeyLen);  
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key1=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                                break;
                            case 26: //wep 104 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }

                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[0].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[0].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key1=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            default:
                                pAd->PortCfg.SharedKey[0].KeyLen = 0;
                                DBGPRINT(RT_DEBUG_TRACE, "%s::Invalid Key (=%s)\n", __FUNCTION__, tmpbuf);
                        }
                    }
                    //Key2Str
                    if (RTMPGetKeyParameter("Default", "Key2Str", tmpbuf, 26, buffer))
                    {
                        KeyLen = strlen(tmpbuf);
                        switch (KeyLen)
                        {
                            case 0:
                                pAd->PortCfg.SharedKey[1].KeyLen = 0;
                                break;
                            case 5: //wep 40 Ascii type
                                pAd->PortCfg.SharedKey[1].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[1].Key, tmpbuf, KeyLen);
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key2=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                            case 10: //wep 40 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }
                                
                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[1].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[1].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key2=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            case 13: //wep 104 Ascii type
                                pAd->PortCfg.SharedKey[1].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[1].Key, tmpbuf, KeyLen);  
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key2=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                                break;
                            case 26: //wep 104 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }

                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[1].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[1].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key2=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            default:
                                pAd->PortCfg.SharedKey[1].KeyLen = 0;
                                DBGPRINT(RT_DEBUG_TRACE, "%s::Invalid argument (=%s)\n", __FUNCTION__, tmpbuf);
                        }
                    }
                    //Key3Str
                    if (RTMPGetKeyParameter("Default", "Key3Str", tmpbuf, 26, buffer))
                    {
                        KeyLen = strlen(tmpbuf);
                        switch (KeyLen)
                        {
                            case 0:
                                pAd->PortCfg.SharedKey[2].KeyLen = 0;
                                break;
                            case 5: //wep 40 Ascii type
                                pAd->PortCfg.SharedKey[2].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[2].Key, tmpbuf, KeyLen);
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key3=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                            case 10: //wep 40 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }

                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[2].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[2].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key3=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            case 13: //wep 104 Ascii type
                                pAd->PortCfg.SharedKey[2].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[2].Key, tmpbuf, KeyLen);  
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key3=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                                break;
                            case 26: //wep 104 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }

                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[2].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[2].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key3=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            default:
                                pAd->PortCfg.SharedKey[2].KeyLen = 0;
                                DBGPRINT(RT_DEBUG_TRACE, "%s::Invalid argument (=%s)\n", __FUNCTION__, tmpbuf);
                        }
                    }
                    //Key4Str
                    if (RTMPGetKeyParameter("Default", "Key4Str", tmpbuf, 26, buffer))
                    {
                        KeyLen = strlen(tmpbuf);
                        switch (KeyLen)
                        {
                            case 0:
                                pAd->PortCfg.SharedKey[3].KeyLen = 0;
                                break;
                            case 5: //wep 40 Ascii type
                                pAd->PortCfg.SharedKey[3].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[3].Key, tmpbuf, KeyLen);
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key4=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                            case 10: //wep 40 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }

                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[3].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[3].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key4=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            case 13: //wep 104 Ascii type
                                pAd->PortCfg.SharedKey[3].KeyLen = KeyLen;
                                memcpy(pAd->PortCfg.SharedKey[3].Key, tmpbuf, KeyLen);  
                                DBGPRINT(RT_DEBUG_TRACE, "%s::(Key4=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Ascii");
                                break;
                            case 26: //wep 104 Hex type
                                for(i=0; i < KeyLen; i++)
                                {
                                    if( !isxdigit(*(tmpbuf+i)) )
                                    {
                                        bIsHex = FALSE;
                                        break;
                                    }
                                }

                                if (bIsHex)
                                {
                                    pAd->PortCfg.SharedKey[3].KeyLen = KeyLen / 2 ;
                                    AtoH(tmpbuf, pAd->PortCfg.SharedKey[3].Key, KeyLen / 2);
                                    DBGPRINT(RT_DEBUG_TRACE, "%s::(Key4=%s and type=%s)\n", __FUNCTION__, tmpbuf, "Hex");
                                }
                                break;
                            default:
                                pAd->PortCfg.SharedKey[3].KeyLen = 0;
                                DBGPRINT(RT_DEBUG_TRACE, "%s::Invalid argument (=%s)\n", __FUNCTION__, tmpbuf);
                        }
                    }
                }
            }
            else
            {
                DBGPRINT(RT_DEBUG_TRACE, "--> %s does not have a write method\n", src);
            }

            retval=filp_close(srcf,NULL);
            if (retval)
            {
                DBGPRINT(RT_DEBUG_TRACE, "--> Error %d closing %s\n", -retval, src);
            }
        }
    } //if (src && *src)

    set_fs(orgfs);
    current->fsuid = orgfsuid;
    current->fsgid = orgfsgid;
}
/*
    ========================================================================

    Routine Description:
        Reset NIC Asics

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:
        Reset NIC to initial state AS IS system boot up time.

    ========================================================================
*/
VOID    RTMPRingCleanUp(
    IN  PRTMP_ADAPTER   pAdapter,
    IN  UCHAR           RingType)
{
    ULONG           Count;
    PTXD_STRUC      pTxD;
    PRXD_STRUC      pRxD;
    PMGMT_STRUC     pMgmt;

    Count = 0;
    switch (RingType)
    {
        case TX_RING:
            // We have to clean all descriptos in case some error happened with reset
            do 
            {
                pTxD  = (PTXD_STRUC) pAdapter->TxRing[pAdapter->NextTxDoneIndex].va_addr;
                        
                pTxD->Owner = DESC_OWN_HOST;
                pTxD->Valid = FALSE;

                pAdapter->NextTxDoneIndex++;
                Count++;
                if (pAdapter->NextTxDoneIndex >= TX_RING_SIZE)
                {
                    pAdapter->NextTxDoneIndex = 0;
                }
        
            }   while (Count < TX_RING_SIZE);   // We have to scan all TX ring

            // Check for packet in send tx wait waiting queue
            skb_queue_purge(&pAdapter->TxSwQueue0);
	    skb_queue_purge(&pAdapter->TxSwQueue1);
	    skb_queue_purge(&pAdapter->TxSwQueue2);
	    skb_queue_purge(&pAdapter->TxSwQueue3);

        case PRIO_RING:
            // We have to clean all descriptos in case some error happened with reset
            do 
            {
                pTxD  = (PTXD_STRUC) pAdapter->PrioRing[pAdapter->NextPrioDoneIndex].va_addr;

                // We just re-claim these ring spaces.
                pTxD->Owner = DESC_OWN_HOST;
                pTxD->Valid = FALSE;

                pAdapter->NextPrioDoneIndex++;
                Count++;
                if (pAdapter->NextPrioDoneIndex >= PRIO_RING_SIZE)
                {
                    pAdapter->NextPrioDoneIndex = 0;
                }

            }   while (Count < PRIO_RING_SIZE);     // We have to scan all Priority Ring

            // Clear managemt buffer ring
            while ((pAdapter->PushMgmtIndex != pAdapter->PopMgmtIndex) || (pAdapter->MgmtQueueSize != 0))
            {
                pMgmt = (PMGMT_STRUC) &pAdapter->MgmtRing[pAdapter->PopMgmtIndex];
                if (pMgmt->Valid == TRUE)
                {
                    MlmeFreeMemory(pAdapter, pMgmt->pBuffer);
                    pMgmt->Valid = FALSE;
                    pAdapter->PopMgmtIndex++;
                    pAdapter->MgmtQueueSize--;
                    if (pAdapter->PopMgmtIndex >= MGMT_RING_SIZE)
                    {
                        pAdapter->PopMgmtIndex = 0;
                    }
                }
            }
            pAdapter->RalinkCounters.MgmtRingFullCount = 0;
            break;

        case RX_RING:
            // We have to clean all descriptos in case some error happened with reset
            do 
            {
                pRxD  = (PRXD_STRUC) pAdapter->RxRing[pAdapter->CurRxIndex].va_addr;

                // Re-initial Rx ring cell to owned by NIC.
                pRxD->Owner = DESC_OWN_NIC;

                pAdapter->CurRxIndex++;
                Count++;
                if (pAdapter->CurRxIndex >= RX_RING_SIZE)
                {
                    pAdapter->CurRxIndex = 0;
                }

            }   while (Count < RX_RING_SIZE);       // We have to scan all Rx Ring
            break;
            
        default:
            break;

    }
}

/*
    ========================================================================

    Routine Description:
        Compare two memory block

    Arguments:
		pSrc1		Pointer to first memory address
		pSrc2		Pointer to second memory addres

    Return Value:
        0:          memory is equal
        1:          pSrc1 memory is larger
        2:          pSrc2 memory is larger

    Note:

    ========================================================================
*/
ULONG   RTMPCompareMemory(
    IN  PVOID   pSrc1,
    IN  PVOID   pSrc2,
    IN  ULONG   Length)
{
    PUCHAR  pMem1;
    PUCHAR  pMem2;
    ULONG   Index = 0;

    pMem1 = (PUCHAR) pSrc1;
    pMem2 = (PUCHAR) pSrc2;

    for (Index = 0; Index < Length; Index++)
    {
        if (pMem1[Index] > pMem2[Index])
            return (1);
        else if (pMem1[Index] < pMem2[Index])
            return (2);
    }

    // Equal
    return (0);
}

/*
    ========================================================================

    Routine Description:
        Initialize port configuration structure

    Arguments:
        Adapter                     Pointer to our adapter

    Return Value:
        None

    Note:

    ========================================================================
*/
VOID    PortCfgInit(
    IN  PRTMP_ADAPTER pAdapter)
{
    UINT i;

    DBGPRINT(RT_DEBUG_TRACE, "--> PortCfgInit\n");    

    pAdapter->PortCfg.UseBGProtection = 0;    // 0: AUTO
    
    pAdapter->PortCfg.CapabilityInfo = 0x0000;
    pAdapter->PortCfg.Psm = PWR_ACTIVE;
    pAdapter->PortCfg.BeaconPeriod = 100;     // in mSec

    pAdapter->PortCfg.CfpMaxDuration = 0;     // never mind, decided by AP later
    pAdapter->PortCfg.CfpDurRemain = 0;       // never mind, decided by AP later
    pAdapter->PortCfg.CfpCount = 0;           // never mind, decided by AP later
    pAdapter->PortCfg.CfpPeriod = 0;          // never mind, decided by AP later
    pAdapter->PortCfg.AuthMode = Ndis802_11AuthModeOpen;

    for(i = 0; i < SHARE_KEY_NO; i++) {
        pAdapter->PortCfg.SharedKey[i].KeyLen = 0;
    }

    for(i = 0; i < PAIRWISE_KEY_NO; i++) {
        pAdapter->PortCfg.PairwiseKey[i].KeyLen = 0;
    }

    for(i = 0; i < GROUP_KEY_NO; i++) {
        pAdapter->PortCfg.GroupKey[i].KeyLen = 0;
    }

    pAdapter->PortCfg.WepStatus = Ndis802_11EncryptionDisabled;
    pAdapter->PortCfg.DefaultKeyId = 0;
    pAdapter->PortCfg.PrivacyFilter = Ndis802_11PrivFilterAcceptAll;

    // 802.1x port control
    pAdapter->PortCfg.PortSecured = WPA_802_1X_PORT_NOT_SECURED;
    pAdapter->PortCfg.LastMicErrorTime = 0;
    pAdapter->PortCfg.MicErrCnt        = 0;
    pAdapter->PortCfg.bBlockAssoc      = FALSE;
    pAdapter->PortCfg.WpaState         = SS_NOTUSE; 

    pAdapter->PortCfg.RtsThreshold = 2347;
    pAdapter->PortCfg.FragmentThreshold = 2346;
    pAdapter->PortCfg.bFragmentZeroDisable = FALSE;

    pAdapter->PortCfg.CurrentTxAntenna = 0xff;  // diversity
    pAdapter->PortCfg.CurrentRxAntenna = 0xff;  // diversity
    pAdapter->PortCfg.NumberOfAntenna = 2;

//  pAdapter->PortCfg.TxPowerLevel[0] = 100;
//  pAdapter->PortCfg.NumOfTxPowerLevel = 1;
    pAdapter->PortCfg.TxPower = 100; //mW
    pAdapter->PortCfg.TxPowerPercentage = 0xffffffff;  // AUTO

    pAdapter->PortCfg.AntennaSupportTx = TRUE;
    pAdapter->PortCfg.AntennaSupportRx = TRUE;
    pAdapter->PortCfg.AntennaSupportDiversityRx = TRUE;

    pAdapter->PortCfg.RecvDtim = TRUE;
    memset(&pAdapter->PortCfg.Bssid, 0, ETH_ALEN);
    memset(&pAdapter->PortCfg.Broadcast, 0xff, ETH_ALEN);
    pAdapter->PortCfg.Pss = PWR_ACTIVE;
    pAdapter->PortCfg.RssiTrigger = 0;
    pAdapter->PortCfg.LastRssi = 0;
    pAdapter->PortCfg.LastAvgRssi  = -95 + RSSI_TO_DBM_OFFSET;// default -95dm
    pAdapter->PortCfg.AvgRssi  = 0;
    pAdapter->PortCfg.RssiTriggerMode = RSSI_TRIGGERED_UPON_BELOW_THRESHOLD;
    pAdapter->PortCfg.AtimWin = 0;
    pAdapter->PortCfg.Channel = 1;

    pAdapter->PortCfg.Aid = 1;

    pAdapter->PortCfg.DefaultListenCount = 3;//default listen count;
    pAdapter->PortCfg.BssType = BSS_INFRA;  // BSS_INFRA or BSS_INDEP

    pAdapter->PortCfg.AdhocMode = 0;

    pAdapter->PortCfg.SsidLen = 0;
    memset(pAdapter->PortCfg.Ssid, 0, MAX_LEN_OF_SSID);  // NOT NULL-terminated

    // global variables mXXXX used in MAC protocol state machines
    pAdapter->PortCfg.Mibss = FALSE;
    pAdapter->PortCfg.Massoc = FALSE;
    pAdapter->PortCfg.Mauth = FALSE;

    // PHY specification
    pAdapter->PortCfg.PhyMode = 0xff;
//  RTMPSetPhyMode(pAdapter, PHY_11BG_MIXED);   // default in 11BG mixed mode
//  pAdapter->PortCfg.Channel = FirstChannel(pAdapter);
    pAdapter->PortCfg.Dsifs = 10;      // in units of usec 
    pAdapter->PortCfg.TxPreambleInUsed = Rt802_11PreambleLong; // use Long preamble on TX by defaut

    // user desired power mode
    pAdapter->PortCfg.WindowsPowerMode = Ndis802_11PowerModeCAM; // Ndis802_11PowerModeFast_PSP;
    pAdapter->PortCfg.WindowsBatteryPowerMode = Ndis802_11PowerModeCAM; // Ndis802_11PowerModeFast_PSP;
    pAdapter->PortCfg.WindowsTxPreamble = Rt802_11PreambleAuto; // use Long preamble on TX by defaut
    pAdapter->PortCfg.WindowsACCAMEnable = FALSE;
//    pAdapter->PortCfg.PacketFilter = NDIS_PACKET_TYPE_ALL_MULTICAST | NDIS_PACKET_TYPE_DIRECTED | NDIS_PACKET_TYPE_BROADCAST;
    pAdapter->bAcceptDirect = TRUE;
    pAdapter->bAcceptMulticast = FALSE;
    pAdapter->bAcceptBroadcast = TRUE;
    pAdapter->bAcceptAllMulticast = TRUE;
    
    // parameters to be used when this STA starts a new ADHOC network
    pAdapter->PortCfg.IbssConfig.BeaconPeriod = 100;
    pAdapter->PortCfg.IbssConfig.AtimWin = 0;
    pAdapter->PortCfg.IbssConfig.Channel = 1;
    pAdapter->PortCfg.RfType = RFIC_2525;
    pAdapter->PortCfg.LedMode = LED_MODE_DEFAULT;

    pAdapter->PortCfg.IgnoredScanNumber = 0;
    pAdapter->bTxBusy = FALSE;

    pAdapter->PortCfg.bHwRadio  = TRUE;
    pAdapter->PortCfg.bSwRadio  = TRUE;
    pAdapter->PortCfg.bRadio    = TRUE;
    pAdapter->PortCfg.bHardwareRadio = FALSE;       // Default is OFF
    pAdapter->PortCfg.bAutoTxAgc = FALSE;			// Default is OFF
	pAdapter->PortCfg.bShowHiddenSSID = FALSE;

    // Nitro mode control
    pAdapter->PortCfg.EnableTxBurst = 0;
    pAdapter->PortCfg.AutoReconnect = TRUE;

    // Save the init time as last scan time, the system should do scan after 2 seconds.
    pAdapter->PortCfg.LastScanTime = 0;

    // Default Config change flag
    pAdapter->bConfigChanged = FALSE;

    pAdapter->PortCfg.bLocalAdminMAC = TRUE;

    pAdapter->NeedSwapToLittleEndian = TRUE;

    // dynamic BBP R17:sensibity tuning to overcome background noise
    pAdapter->PortCfg.BbpTuningEnable  = TRUE;  // overwritten by E2PROM setting
    pAdapter->PortCfg.VgcLowerBound    = 0x38;  // overwritten by E2PROM setting
    pAdapter->PortCfg.BbpTuning.FalseCcaLowerThreshold = 100;
    pAdapter->PortCfg.BbpTuning.FalseCcaUpperThreshold = 4;   // unit 128, 4*128 = 512
    pAdapter->PortCfg.BbpTuning.VgcDelta               = 1;
    pAdapter->PortCfg.BbpTuning.VgcUpperBound          = BBP_R17_DYNAMIC_UP_BOUND;

    pAdapter->PortCfg.StaWithEtherBridge.Enable   = FALSE;
    memset(&pAdapter->PortCfg.StaWithEtherBridge.EtherMacAddr, 0xff, ETH_ALEN);

#ifdef RALINK_ATE
	memset(&pAdapter->ate, 0, sizeof(ATE_INFO));
	pAdapter->ate.Mode = ATE_STASTART;
	pAdapter->ate.TxCount = TX_RING_SIZE;
	pAdapter->ate.TxLength = PRIO_BUFFER_SIZE;
	pAdapter->ate.TxRate = RATE_11;
	pAdapter->ate.Channel = 1;
	memcpy(&pAdapter->ate.Addr1,"001122334455", ETH_ALEN);
	memcpy(&pAdapter->ate.Addr2,"001122334455", ETH_ALEN);
	memcpy(&pAdapter->ate.Addr3,"001122334455", ETH_ALEN);
#endif	//#ifdef RALINK_ATE

    RTMP_IO_READ32(pAdapter, 0, &pAdapter->PortCfg.Rt2560Version);

    DBGPRINT(RT_DEBUG_TRACE, "<-- PortCfgInit\n");
}

UCHAR BtoH(char ch)
{
    if (ch >= '0' && ch <= '9') return (ch - '0');        // Handle numerals
    if (ch >= 'A' && ch <= 'F') return (ch - 'A' + 0xA);  // Handle capitol hex digits
    if (ch >= 'a' && ch <= 'f') return (ch - 'a' + 0xA);  // Handle small hex digits
    return(255);
}

//
//  FUNCTION: AtoH(char *, UCHAR *, int)
//
//  PURPOSE:  Converts ascii string to network order hex
//
//  PARAMETERS:
//    src    - pointer to input ascii string
//    dest   - pointer to output hex
//    destlen - size of dest
//
//  COMMENTS:
//
//    2 ascii bytes make a hex byte so must put 1st ascii byte of pair
//    into upper nibble and 2nd ascii byte of pair into lower nibble.
//

void AtoH(char * src, UCHAR * dest, int destlen)
{
    char *srcptr;
    PUCHAR destTemp;

    srcptr = src;   
    destTemp = (PUCHAR) dest; 

    while(destlen--)
    {
        *destTemp = BtoH(*srcptr++) << 4;    // Put 1st ascii byte in upper nibble.
        *destTemp += BtoH(*srcptr++);      // Add 2nd ascii byte to above.
        destTemp++;
    }
}

/*
	========================================================================
	
	Routine Description:
		Init timer objects

	Arguments:
		pAdapter			Pointer to our adapter
		pTimer				Timer structure
		pTimerFunc			Function to execute when timer expired
		Repeat				Ture for period timer

	Return Value:
		None

	Note:
		
	========================================================================
*/
VOID	RTMPInitTimer(
	IN	PRTMP_ADAPTER			pAdapter,
	IN	PRALINK_TIMER_STRUCT	pTimer,
	IN	PVOID					pTimerFunc)
{
	pTimer->State      = FALSE;
	init_timer(&pTimer->TimerObj);
	pTimer->TimerObj.data = (unsigned long)pAdapter;
	pTimer->TimerObj.function = pTimerFunc;
}

/*
	========================================================================
	
	Routine Description:
		Init timer objects

	Arguments:
		pTimer				Timer structure
		Value				Timer value in milliseconds

	Return Value:
		None

	Note:
		
	========================================================================
*/
VOID	RTMPSetTimer(
    IN  PRTMP_ADAPTER           pAdapter,
	IN	PRALINK_TIMER_STRUCT	pTimer,
	IN	ULONG					Value)
{
	pTimer->TimerValue = Value;
	pTimer->State      = FALSE;
	pTimer->TimerObj.expires = jiffies + (Value * HZ)/1000;
	add_timer(&pTimer->TimerObj);
}

/*
	========================================================================
	
	Routine Description:
		Cancel timer objects

	Arguments:
		Adapter						Pointer to our adapter

	Return Value:
		None

	Note:
		Reset NIC to initial state AS IS system boot up time.
		
	========================================================================
*/
VOID	RTMPCancelTimer(
	IN	PRALINK_TIMER_STRUCT	pTimer)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,4,27))
	del_timer_sync(&pTimer->TimerObj);
#else
	del_timer(&pTimer->TimerObj);
#endif
}

