/******************************************************************************
*
*     Author: Xilinx, Inc.
*
*
*     This program is free software; you can redistribute it and/or modify it
*     under the terms of the GNU General Public License as published by the
*     Free Software Foundation; either version 2 of the License, or (at your
*     option) any later version.
*
*
*     XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
*     COURTESY TO YOU. BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
*     ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE, APPLICATION OR STANDARD,
*     XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION IS FREE
*     FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE FOR OBTAINING
*     ANY THIRD PARTY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
*     XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
*     THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY
*     WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM
*     CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND
*     FITNESS FOR A PARTICULAR PURPOSE.
*
*
*     Xilinx hardware products are not intended for use in life support
*     appliances, devices, or systems. Use in such applications is
*     expressly prohibited.
*
*
*     (c) Copyright 2005 Xilinx Inc.
*     All rights reserved.
*
*
*     You should have received a copy of the GNU General Public License along
*     with this program; if not, write to the Free Software Foundation, Inc.,
*     675 Mass Ave, Cambridge, MA 02139, USA.
*
******************************************************************************/
/*****************************************************************************/
/**
 *
 * @file xtemac.h
 *
 * The Xilinx Tri-Mode Ethernet driver component. This driver supports the
 * Virtex-4(TM) 10/100/1000 MAC (TEMAC).
 *
 * For a full description of TEMAC features, please see the HW spec. This driver
 * supports the following features:
 *   - Memory mapped access to host interface registers
 *   - API for polled frame transfers (FIFO direct polled mode)
 *   - API for interrupt driven frame transfers with or without simple DMA
 *     channels (FIFO direct mode)
 *   - API for interrupt driven frame transfers with scatter-gather DMA (SGDMA
 *     mode)
 *   - Virtual memory support
 *   - Unicast receive address filtering
 *   - Broadcast receive address filtering
 *   - Full duplex operation
 *   - Automatic source address insertion or overwrite (programmable)
 *   - Automatic PAD & FCS insertion and stripping (programmable)
 *   - Pause frame (flow control) detection in full-duplex mode
 *   - Programmable interframe gap
 *   - VLAN frame support
 *   - Pause frame support
 *   - Jumbo frame support
 *   - Data Realignment Engine (DRE)
 *
 * <b>Driver Description</b>
 *
 * The device driver enables higher layer software (e.g., an application) to
 * communicate to the TEMAC. The driver handles transmission and reception of
 * Ethernet frames, as well as configuration and control. No pre or post
 * processing of frame data is performed. The driver does not validate the
 * contents of an incoming frame in addition to what has already occurred in HW.
 * A single device driver can support multiple devices even when those devices
 * have significantly different configurations.
 *
 * <b>Device Configuration</b>
 *
 * The device can be configured in various ways during the FPGA implementation
 * process. Configuration parameters are stored in xtemac_g.c. A table is
 * defined that contains configuration information for each TEMAC in the system.
 * This information includes such things as the base address of the memory-
 * mapped device and whether the device has DMA, counter registers, multicast
 * support, MII support, etc.
 *
 * The driver tries to use the features built into the device. So if the hardware
 * is configured with SG DMA, the driver expects to start the scatter-gather
 * channels and expects that the user has set up the buffer descriptor lists.
 *
 * If the user expects to use the driver in a mode different than how the
 * hardware is configured, the user should modify the configuration table to
 * reflect the mode to be used prior to device initialization. Care must be taken
 * when modifying the table. Adding a feature that is not present in HW will
 * cause system instability.
 *
 * The following example changes a device to FIFO direct mode:
 * <pre>
 *        XTemac_Config *ConfigPtr;
 *
 *        ConfigPtr = XTemac_LookupConfig(DeviceId);
 *        ConfigPtr->IpIfDmaConfig = XEM_CFG_NO_DMA;
 * </pre>
 *
 * <b>Asynchronous Callbacks</b>
 *
 * The driver services interrupts and passes Ethernet frames to the higher layer
 * software through asynchronous callback functions. When using the driver
 * directly (i.e., not with the RTOS protocol stack), the higher layer
 * software must register its callback functions during initialization. The
 * driver requires callback functions for received frames, for confirmation of
 * transmitted frames, and for asynchronous errors.
 *
 * <b>Interrupts</b>
 *
 * The driver has no dependencies on the interrupt controller. The driver
 * provides two interrupt handlers. XTemac_IntrFifoHandler() handles
 * interrupts when the MAC is configured for FIFO direct and simple DMA frame
 * transfer mode. XTemac_IntrSgHandler() handles interrupts when the MAC is
 * configured for SG DMA mode. Either one of these routines can be connected to
 * the system interrupt controller by BSP/OS specific means.
 *
 * SGDMA implements interrupt coalescing features that reduce the number of
 * interrupts the processor must service. A more complete discussion of this
 * feature occurs in the API section below.
 *
 * <b>Device Reset</b>
 *
 * Some errors that can occur in the device require a device reset. These errors
 * are listed in the XTemac_ErrorHandler() function typedef header. The user's
 * error handler is responsible for resetting and re-configuring the device.
 *
 * <b>Virtual Memory</b>
 *
 * This driver has limited support for systems that use virtual memory. For
 * these types of systems, XTemac_VmInitialize() should be used instead of
 * XTemac_Initialize() to setup a device instance.
 *
 * All virtual to physical memory mappings must occur prior to accessing the
 * driver API. The driver does not support multiple virtual memory translations
 * that map to the same physical address.
 *
 * For DMA transactions, user buffers supplied to the driver must be in terms
 * of their physical address.
 *
 * <b>Frame Transfer APIs</b>
 *
 * The next 4 sections discuss the APIs available to transfer frames between user
 * buffers and the Ethernet. These APIs vary considerably in performance and
 * depend on specific HW configuration.
 *
 * Except for SGDMA, these APIs allow the user independent access to the packet
 * FIFOs and the length/status FIFOs. The packet FIFOs contain the frame data
 * while the length/status FIFOs contain receive lengths, transmit lengths,
 * and transmit statuses. It is very important to keep these two sets of
 * FIFOs in sync with each other. When they are out of sync, then user data
 * will become corrupted.
 *
 * On the transmit side, the transmit packet FIFO may contain more than one
 * Ethernet packet. The number of packets it may contain depends on its depth
 * which is controlled at HW build time. For each packet in the FIFO, the user
 * must initiate a transmit by writing into the transmit length FIFO (see
 * XTemac_FifoSend()). The number of bytes specified to transmit must match
 * exactly the lengths of packets in the packet FIFO. For example, if a 76 byte
 * packet was written followed by a 124 byte packet, then the transmit length
 * FIFO must be written with 76 followed by 124. At the completion of the
 * transmission, the transmit status FIFO must be read to obtain the outcome
 * of the operation. The first status will be for the 76 byte packet followed
 * by the 124 byte packet.
 *
 * On the receive side, it is a little easier to keep things in sync because
 * the HW writes to the receive packet FIFO. Just like the transmit packet FIFO,
 * the receive packet FIFO can contain more than one received Ethernet frame.
 * Each time a length is extracted from the receive length FIFO (see
 * XTemac_FifoRecv()), then that many bytes must be read from the receive
 * packet FIFO.
 *
 * The easiest way to keep these FIFOs in sync is to process a single frame at
 * a time. But when performance is an issue, it may be desirable to process
 * multiple or even partial frames from non-contiguous memory regions with the
 * API functions. The examples that accompany this driver illustrate how these
 * advanced frame processing methods can be implemented.
 *
 * <b>API for FIFO Direct Polled Mode</b>
 *
 * This mode is available for all HW configurations. It uses the processor to
 * transfer data between user buffers and the packet FIFOs. This mode may be used
 * when the XTE_POLLED_OPTION has been set (see XTemac_SetOptions()).
 *
 * Relevant functions for this mode are:
 *   - XTemac_FifoGetFreeBytes()
 *   - XTemac_FifoWrite()
 *   - XTemac_FifoSend()
 *   - XTemac_FifoQuerySendStatus()
 *   - XTemac_FifoQueryRecvStatus()
 *   - XTemac_FifoRecv()
 *   - XTemac_FifoRead()
 *
 * To send frames in this mode:
 *   - Call XTemac_FifoGetFreeBytes() to make sure there is room in the transmit
 *     packet FIFO for the frame(s) to be sent.
 *   - Write frame data into the FIFO using XTemac_FifoWrite().
 *   - Initiate transmit by calling XTemac_FifoSend() for each frame to send.
 *   - For each time XTemac_FifoSend() has been called, continue to invoke
 *     XTemac_FifoQuerySendStatus() until it reports that a frame has been sent
 *     or an error occurred.
 *
 * To receive frames in this mode:
 *   - Call XTemac_FifoQueryRecvStatus() until it reports that a frame has been
 *     received.
 *   - Call XTemac_FifoRecv() to get the length of the next pending frame.
 *   - Call XTemac_FifoRead() to read the data into your buffer.
 *
 * <b>API for FIFO Direct Interrupt Driven Mode</b>
 *
 * This mode is available for all HW configurations. It uses the processor to
 * transfer data between user buffers and the packet FIFOs. Interrupts are
 * utilized to notify the user whether frames have arrived, have been sent,
 * or an error has occurred.
 *
 * Relevant functions and callbacks for this mode are:
 *   - XTemac_FifoGetFreeBytes()
 *   - XTemac_FifoWrite()
 *   - XTemac_FifoSend()
 *   - XTemac_FifoRecv()
 *   - XTemac_FifoRead()
 *   - XTemac_IntrFifoEnable()
 *   - XTemac_IntrFifoDisable()
 *   - XTE_HANDLER_FIFOSEND callback
 *   - XTE_HANDLER_FIFORECV callback
 *   - XTE_HANDLER_ERROR callback
 *
 * To send frames in this mode:
 *   - Enable transmit interrupts with XTemac_IntrFifoEnable().
 *   - Call XTemac_FifoGetFreeBytes() to make sure there is room in the transmit
 *     packet FIFO for the frame(s) to be sent.
 *   - Write frame data into the FIFO using XTemac_FifoWrite().
 *   - Initiate transmit by calling XTemac_FifoSend() for each frame to send.
 *   - For each frame successfully transmitted, the FifoSend callback is invoked.
 *   - The user's FifoSend callback should handle post frame transmission
 *     application level work.
 *
 * To receive frames in this mode:
 *   - Enable receive interrupts with XTemac_IntrFifoEnable().
 *   - Upon arrival of a frame, the user's FifoRecv callback is invoked.
 *   - The user's FifoRecv callback must call XTemac_FifoRecv() to get the
 *     length of the waiting frame and then call XTemac_FifoRead() to copy
 *     the frame to a user buffer.
 *   - Alternatively, the user's FifoRecv callback may defer reception of the
 *     frame by disabling receive interrupts with XTemac_IntrFifoDisable().
 *     This can improve interrupt latency for other system devices by moving
 *     the processor intensive XTemac_FifoRead() function out of interrupt
 *     context.
 *
 * <b>API for FIFO Direct with Simple DMA Interrupt Driven Mode</b>
 *
 * This transfer mode utilizes simple DMA channels to move data between user
 * buffers and the FIFOs. This mode is similar to FIFO direct transfer mode
 * except that XTemac_FifoDmaRead() can be used instead of XTemac_FifoRead(),
 * and XTemac_FifoDmaWrite() instead of XTemac_FifoWrite().
 *
 * Relevant functions and callbacks for this mode are:
 *   - XTemac_FifoWrite()
 *   - XTemac_FifoSend()
 *   - XTemac_FifoRecv()
 *   - XTemac_FifoRead()
 *   - XTemac_IntrFifoEnable()
 *   - XTemac_IntrFifoDisable()
 *   - XTE_HANDLER_FIFOSEND callback
 *   - XTE_HANDLER_FIFORECV callback
 *   - XTE_HANDLER_ERROR callback
 *   - XTemac_FifoDmaWrite()
 *   - XTemac_FifoDmaRead()
 *   - XTemac_IntrFifoDmaEnable()
 *   - XTemac_IntrFifoDmaDisable()
 *   - XTE_HANDLER_FIFODMAWRITE callback
 *   - XTE_HANDLER_FIFODMAREAD callback
 *
 * To send frames in this mode:
 *   - Enable transmit interrupts with XTemac_IntrFifoEnable().
 *   - Enable send channel DMA interrupts with XTemac_IntrFifoDmaEnable().
 *   - Call XTemac_FifoGetFreeBytes() to make sure there is room in the transmit
 *     packet FIFO for the frame to be sent.
 *   - Initiate transfer of data from user buffer to FIFO by calling
 *     XTemac_FifoDmaWrite().
 *   - When DMA operation completes, the FifoDmaWrite callback is invoked at
 *     which point XTemac_FifoSend() should be called to begin transmission.
 *   - For each frame successfully transmitted, the FifoSend callback is invoked.
 *   - The user's FifoSend callback should handle post frame transmission
 *     application level work.
 *
 * To receive frames in this mode:
 *   - Enable receive interrupts with XTemac_IntrFifoEnable().
 *   - Enable packet FIFO to buffer DMA interrupts with
 *     XTemac_IntrFifoDmaEnable().
 *   - Upon arrival of a frame, the user's FifoRecv callback is invoked.
 *   - The user's FifoRecv callback must call XTemac_FifoRecv() to get the
 *     length of the waiting frame and then call XTemac_FifoDmaRead() to
 *     initiate a DMA transfer from the FIFO to a user buffer.
 *   - When the DMA operation completes, the FifoDmaRead callback is invoked, at
 *     which point application level work is performed to handle the new frame.
 *
 * In some situations, a DMA operation may not be desirable. The extra overhead
 * of DMA interrupts may decrease throughput of small packets. In this case,
 * the user may elect to use XTemac_FifoRead() or XTemac_FifoWrite() instead of
 * starting a DMA transaction with XTemac_FifoDmaRead() or XTemac_FifoDmaWrite().
 * These two sets of functions can be used interchangeably.
 *
 * <b>API for SG DMA Frame Transfer Mode</b>
 *
 * This API utilizes scatter-gather DMA (SGDMA) channels to transfer frame data
 * between user buffers and the packet FIFOs.
 *
 * The SGDMA engine uses buffer descriptors (BDs) to describe Ethernet frames.
 * These BDs are typically chained together into a list the HW follows when
 * transferring data in and out of the packet FIFOs. Each BD describes a memory
 * region containing either a full or partial Ethernet packet.
 *
 * The frequency of interrupts can be controlled with the interrupt coalescing
 * features of the SG DMA engine. These features can be used to optimize
 * interrupt latency and throughput for the user's network traffic conditions.
 * The packet threshold count will delay processor interrupts until a
 * programmable number of packets have arrived or have been transmitted. The
 * packet wait bound timer can be used to cause a processor interrupt even though
 * the packet threshold has not been reached. The timer begins counting after the
 * last packet is processed. If no other packet is processed as the timer
 * expires, then an interrupt will be generated.
 *
 * Another form of interrupt control is provided with the XTE_SGEND_INT_OPTION
 * option. When enabled, an interrupt will occur when SGDMA engine completes the
 * last BD to be processed and transitions to an idle state. This feature may be
 * useful when a set of BDs have been queued up and the user only wants to be
 * notified when they have all been processed by the HW. To use this feature
 * effectively, interrupt coalescing should be disabled (packet threshold = 0,
 * wait bound timer = 0), or the packet threshold should be set to a number
 * larger than the number of packets queued up.
 *
 * By default, the driver will set the packet threshold = 1, wait bound timer =
 * 0, and disable the XTE_SGEND_INT_OPTION. These settings will cause one
 * interrupt per packet.
 *
 * Relevant functions and callbacks for this mode are:
 *   - XTemac_mSgRecvBdNext()
 *   - XTemac_mSgSendBdNext()
 *   - XTemac_SgAlloc()
 *   - XTemac_SgCommit()
 *   - XTemac_SgGetProcessed()
 *   - XTemac_SgFree()
 *   - XTemac_SgSetSpace()
 *   - XTemac_IntrSgEnable()
 *   - XTemac_IntrSgDisable()
 *   - XTemac_IntrSgCoalSet()
 *   - XTemac_IntrSgCoalGet()
 *   - XTE_HANDLER_SGSEND callback
 *   - XTE_HANDLER_SGRECV callback
 *   - XTE_HANDLER_ERROR callback
 *
 * This API requires the user to understand the how the SGDMA driver operates.
 * The following paragraphs provide some explanation, but the user is encouraged
 * to read documentation in xdmav2.h and xdmabdv2.h as well as study example code
 * that accompanies this driver.
 *
 * The API is designed to get BDs to and from the SGDMA engine in the most
 * efficient means possible. The first step is to establish a  memory region to
 * contain all BDs for a specific channel. This is done with XTemac_SgSetSpace()
 * and assumes the memory region is non-cached. This function sets up a BD ring
 * that HW will follow as BDs are processed. The ring will consist of a user
 * defined number of BDs which will all be partially initialized. For example on
 * the transmit channel, the driver will initialize all BDs' destination address
 * word with the address of the packet FIFO. The more fields that can be
 * permanently setup at initialization, then the fewer accesses will be needed
 * to each BD while the SGDMA engine is in operation resulting in better
 * throughput and CPU utilization. The best case initialization would require
 * the user to set only a frame buffer address and length prior to submitting
 * the BD to the engine.
 *
 * BDs move through the engine with the help of functions XTemac_SgAlloc(),
 * XTemac_SgCommit(), XTemac_SgGetProcessed(), and XTemac_SgFree(). All these
 * functions handle BDs that are in place. That is, there are no copies of BDs
 * kept anywhere and any BD the user interacts with is an actual BD from the
 * same ring HW accesses. Changing fields within BDs is done through an API
 * defined in xdmabdv2.h.
 *
 * BDs in the ring go through a series of states as follows:
 *   1. Idle. The driver controls BDs in this state.
 *   2. The user has data to transfer. XTemac_SgAlloc() is called to reserve
 *      BD(s). Once allocated, the user may setup the BD(s) with frame buffer
 *      address and length attributes. The user controls BDs in this state.
 *   3. The user submits BDs to the SGDMA engine with XTemac_SgCommit. BDs in
 *      this state are either waiting to be processed by HW, are in process, or
 *      have been processed. The SGDMA engine controls BDs in this state.
 *   4. Processed BDs are retrieved with XTemac_SgGetProcessed() by the
 *      user. Once retrieved, the user can examine each BD for the outcome of
 *      the DMA transfer. The user controls BDs in this state. After examining
 *      the BDs the user calls XTemac_SgFree() which places the BDs back into
 *      state 1.
 *
 * Each of the four BD accessor functions operate on a set of BDs. A set is
 * defined as a segment of the BD ring consisting of one or more BDs.  The user
 * views the set as a pointer to the first BD along with the number of BDs for
 * that set. The set can be navigated by using macros XTemac_mSgRecvBdNext() or
 * XTemac_mSgSendBdNext(). The user must exercise extreme caution when changing
 * BDs in a set as there is nothing to prevent doing a mSgRecvBdNext past the
 * end of the set and modifying a BD out of bounds.
 *
 * XTemac_SgAlloc() + XTemac_SgCommit(), as well as XTemac_SgGetProcessed() +
 * XTemac_SgFree() are designed to be used in tandem. The same BD set retrieved
 * with SgAlloc should be the same one provided to HW with SgCommit. Same goes
 * with SgGetProcessed and SgFree.
 *
 * To transmit frames in this mode:
 *   - Setup BD space for the transmit channel with XTemac_SgSetSpace().
 *   - Enable transmit interrupts with XTemac_IntrSgEnable().
 *   - Allocate a set of transmit channel BDs with XTemac_SgAlloc().
 *   - Attach frame data to the BD set.
 *   - Submit BD set to HW with XTemac_SgCommit().
 *   - When the transmit channel completes and interrupt coalescing conditions
 *     are met, the user's SgSend callback is invoked.
 *   - The user's SgSend callback should handle or schedule a time to service
 *     processed BDs as follows:
 *       * Get a processed BD set with XTemac_SgGetProcessed()
 *       * Examine BDs for transmit outcomes
 *       * Free BD set back to the transmit channel with XTemac_SgFree().
 *
 * To receive frames in this mode:
 *   - Setup BD space for the receive channel with XTemac_SgSetSpace().
 *   - Allocate a set of receive channel BDs with XTemac_SgAlloc().
 *   - Attach frame buffers to the BD set.
 *   - Submit BD set to HW with XTemac_SgCommit().
 *   - Enable receive interrupts with XTemac_IntrSgEnable().
 *   - When frames are received and interrupt coalescing conditions
 *     are met, the user's SgRecv callback is invoked.
 *   - The user's SgRecv callback should handle or schedule a time to service
 *     processed BDs as follows:
 *       * Get a processed BD set with XTemac_SgGetProcessed().
 *       * Detach frame buffer from BDs and pass buffer up the communication
 *         stack
 *       * Free BD set back to the receive channel with XTemac_SgFree().
 *
 * <b>SG DMA Troubleshooting</b>
 *
 * The SG DMA design used here is less forgiving than previous ones used in
 * older versions of MAC cores that use version 1.00 of the DMA driver. Since
 * BDs are pre-initialized by this driver to increase performance, there are
 * some operations the user should avoid. For example, setting the destination
 * address for a transmit BD will override the packet FIFO address preset by
 * the XTemac driver. Operations such as this may cause the DMA channel to
 * fail to transfer user data, lockup, or destabilize the system.
 *
 * To verify internal structures of BDs and the BD ring, the function
 * XTemac_SgCheck() is provided. This function should be used as a debugging
 * or diagnostic tool. If it returns a failure, the user should perform more
 * in depth debugging to find the root cause.
 *
 * To avoid problems, do not use the following BD macros for transmit channel
 * BDs (XTE_SEND):
 *
 *   - XDmaBdV2_mClear()
 *   - XDmaBdV2_mSetDestBufIncrement()
 *   - XDmaBdV2_mSetDestBufNoIncrement()
 *   - XDmaBdV2_mSetRxDir()
 *   - XDmaBdV2_mSetDestAddr()
 *
 * and for receive channel BDs (XTE_RECV):
 *
 *   - XDmaBdV2_mClear()
 *   - XDmaBdV2_mSetSrcBufIncrement()
 *   - XDmaBdV2_mSetSrcBufNoIncrement()
 *   - XDmaBdV2_mSetTxDir()
 *   - XDmaBdV2_mSetSrcAddr()
 *
 * <b>Alignment Restrictions</b>
 *
 * When HW is configured to use the Data Realignment Engine (DRE), alignment
 * restrictions are as follows:
 *   - XTemac_FifoDmaRead() & XTemac_FifoDmaWrite(): Transmit and receive buffers
 *     must be aligned on a 8 byte boundary.
 *   - SGDMA buffer descriptors must be aligned on a 8-byte boundary.
 *   - SGDMA transmit buffers can be aligned on any boundary, but receive buffers
 *     must be aligned on a 8-byte boundary.
 *
 * Without DRE, buffer alignment restrictions are as follows:
 *   - XTemac_FifoDmaRead() & XTemac_FifoDmaWrite(): Transmit and receive buffers
 *     must be aligned on a 8-byte boundary.
 *   - SGDMA buffer descriptors must be aligned on a 8-byte boundary.
 *   - SGDMA transmit and receive buffers must be aligned on a 8-byte boundary
 *
 * There are no alignment restrictions when using XTemac_FifoRead() and
 * XTemac_FifoWrite().
 *
 * <b>Buffer Copying</b>
 *
 * The driver is designed for a zero-copy buffer scheme. That is, the driver will
 * not copy buffers. This avoids potential throughput bottlenecks within the
 * driver.
 *
 * The only exception to this is when buffers are passed to XTemac_FifoRead() and
 * XTemac_FifoWrite() on 1, 2, or 3 byte alignments. These buffers will be byte
 * copied into a small holding area on their way to or from the packet FIFOs.
 * For PLB TEMAC this holding area is 8 bytes each way. If byte copying is
 * required, then the transfer will take longer to complete.
 *
 * <b>PHY Communication</b>
 *
 * Prior to PHY access, the MDIO clock must be setup. This driver will set a
 * safe default that should work with PLB bus speeds of up to 150 MHz and keep
 * the MDIO clock below 2.5 MHz. If the user wishes faster access to the PHY
 * then the clock divisor can be set to a different value (see
 * XTemac_PhySetMdioDivisor()).
 *
 * MII register access is performed through the functions XTemac_PhyRead() and
 * XTemac_PhyWrite().
 *
 * <b>Link Sync</b>
 *
 * When the device is used with MII or GMII, the link speed must be explicitly
 * set using XTemac_SetMiiLinkSpeed() and must match the speed the PHY has
 * negotiated. If the speeds are mismatched, then the MAC will not pass traffic.
 *
 * The default duplex is full. On half duplex links, the XTE_HALF_DUPLEX_OPTION
 * must be set.
 *
 * <b>Asserts</b>
 *
 * Asserts are used within all Xilinx drivers to enforce constraints on argument
 * values. Asserts can be turned off on a system-wide basis by defining, at
 * compile time, the NDEBUG identifier. By default, asserts are turned on and it
 * is recommended that users leave asserts on during development.
 *
 * <b>Driver Errata</b>
 *
 * The following features did not make this release due to absense of or
 * problems with HW:
 *   - DCR based access to host interface registers.
 *   - Retrieval of hard statistics counters.
 *   - Multicast receive address filtering.
 *   - Mii access to PHY. API functions always return XST_NO_FEATURE.
 *
 * Other issues:
 *   - Setting XTE_FCS_STRIP_OPTION will cause data corruption due to HW
 *     bugs. By default, this driver will disable the option. Future versions
 *     with operational HW will enable this option by default.
 *   - A dropped receive frame indication may be reported by the driver after
 *     calling XTemac_Stop() followed by XTemac_Start(). This can occur if a
 *     frame is arriving when stop is called.
 *
 * @note
 *
 * Xilinx drivers are typically composed of two components, one is the driver
 * and the other is the adapter.  The driver is independent of OS and processor
 * and is intended to be highly portable.  The adapter is OS-specific and
 * facilitates communication between the driver and an OS.
 * <br><br>
 * This driver is intended to be RTOS and processor independent. Any needs for
 * dynamic memory management, threads or thread mutual exclusion, or cache
 * control must be satisfied by the layer above this driver.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 1.00a rmm  06/01/05 First release
 * </pre>
 *
 *****************************************************************************/

#ifndef XTEMAC_H		/* prevent circular inclusions */
#define XTEMAC_H		/* by using protection macros */

/***************************** Include Files *********************************/

#include <asm/delay.h>
#include "xbasic_types.h"
#include "xstatus.h"
#include "xipif_v1_23_b.h"
#include "xpacket_fifo_v2_00_a.h"
#include "xdmav2.h"
#include "xtemac_l.h"

/************************** Constant Definitions *****************************/

/*
 * Device information
 */
#define XTE_DEVICE_NAME     "xtemac"
#define XTE_DEVICE_DESC     "Xilinx Tri-speed 10/100/1000 MAC"

/** @name Configuration options
 *
 * Device configuration options. See the XTemac_SetOptions(),
 * XTemac_ClearOptions() and XTemac_GetOptions() for information on how to use
 * options.
 *
 * The default state of the options are noted and are what the device and driver
 * will be set to after calling XTemac_Reset() or XTemac_Initialize().
 *
 * @{
 */

#define XTE_PROMISC_OPTION               0x00000001
/**< Accept all incoming packets.
 *   This option defaults to disabled (cleared) */

#define XTE_JUMBO_OPTION                 0x00000002
/**< Jumbo frame support for Tx & Rx.
 *   This option defaults to disabled (cleared) */

#define XTE_VLAN_OPTION                  0x00000004
/**< VLAN Rx & Tx frame support.
 *   This option defaults to disabled (cleared) */

#define XTE_HALF_DUPLEX_OPTION           0x00000008
/**< Half duplex mode for Tx & Rx.
 *   This option defaults to full duplex (cleared) */

#define XTE_FLOW_CONTROL_OPTION          0x00000010
/**< Enable recognition of flow control frames on Rx
 *   This option defaults to enabled (set) */

#define XTE_FCS_STRIP_OPTION             0x00000020
/**< Strip FCS and PAD from incoming frames. Note: PAD from VLAN frames is not
 *   stripped.
 *   This option defaults to disabled (clear) */

#define XTE_FCS_INSERT_OPTION            0x00000040
/**< Generate FCS field and add PAD automatically for outgoing frames.
 *   This option defaults to enabled (set) */

#define XTE_LENTYPE_ERR_OPTION           0x00000080
/**< Enable Length/Type error checking for incoming frames. When this option is
 *   set, the MAC will filter frames that have a mismatched type/length field
 *   and if XTE_REPORT_RXERR_OPTION is set, the user is notified when these
 *   types of frames are encountered. When this option is cleared, the MAC will
 *   allow these types of frames to be received.
 *
 *   This option defaults to enabled (set) */

#define XTE_SGEND_INT_OPTION             0x00000100
/**< Enable the SGEND interrupt with SG DMA. When enabled, an interrupt will
 *   be triggered when the end of the buffer descriptor list is reached. The
 *   interrupt will occur despite interrupt coalescing settings.
 *   This option defaults to disabled (cleared) */

#define XTE_POLLED_OPTION                0x00000200
/**< Polled mode communications. Enables use of XTemac_FifoQuerySendStatus()
 *   and XTemac_FifoQueryRecvStatus(). Users may enter/exit polled mode
 *   from any interrupt driven mode.
 *   This option defaults to disabled (cleared) */

#define XTE_REPORT_RXERR_OPTION          0x00000400
/**< Enable reporting of dropped receive packets due to errors
 *   This option defaults to enabled (set) */

#define XTE_TRANSMITTER_ENABLE_OPTION    0x00000800
/**< Enable the transmitter.
 *   This option defaults to enabled (set) */

#define XTE_RECEIVER_ENABLE_OPTION       0x00001000
/**< Enable the receiver
 *   This option defaults to enabled (set) */

#define XTE_BROADCAST_OPTION             0x00002000
/**< Allow reception of the broadcast address
 *   This option defaults to enabled (set) */

#define XTE_MULTICAST_CAM_OPTION         0x00004000
/**< Allows reception of multicast addresses programmed into CAM
 *   This option defaults to disabled (clear)
 *   Note: this option has no HW support in this release
 */

#define XTE_REPORT_TXSTATUS_OVERRUN_OPTION 0x00008000
/**< Enable reporting the overrun of the Transmit status FIFO. This type of
 *   error is latched by HW and can be cleared only by a reset. SGDMA systems,
 *   this option should be enabled since the DMA engine is responsible for
 *   keeping this from occurring. For simple DMA, or FIFO direct systems,
 *   this error may be a nuisance because a SW system may be able to transmit
 *   frames faster than the interrupt handler can handle retrieving statuses.
 *   This option defaults to enabled (set) */

#define XTE_DEFAULT_OPTIONS                     \
    (XTE_FLOW_CONTROL_OPTION |                  \
     XTE_BROADCAST_OPTION |                     \
     XTE_FCS_INSERT_OPTION |                    \
     XTE_LENTYPE_ERR_OPTION |                   \
     XTE_TRANSMITTER_ENABLE_OPTION |            \
     XTE_REPORT_RXERR_OPTION |                  \
     XTE_REPORT_TXSTATUS_OVERRUN_OPTION |       \
     XTE_RECEIVER_ENABLE_OPTION)
/**< Default options set when device is initialized or reset */

/*@}*/

/** @name Direction identifiers
 *
 *  These are used by several functions and callbacks that need
 *  to specify whether an operation specifies a send or receive channel.
 * @{
 */
#define XTE_SEND    1
#define XTE_RECV    2
/*@}*/

/** @name XTemac_FifoWrite/Read() function arguments
 *
 *  These are used by XTemac_FifoWrite/Read() End Of Packet (Eop)
 *  parameter.
 * @{
 */
#define XTE_END_OF_PACKET   1	/**< The data written is the last for the
                                  *  current packet */
#define XTE_PARTIAL_PACKET  0	/**< There is more data to come for the
                                  *  current packet */
/*@}*/

/** @name Callback identifiers
 *
 * These constants are used as parameters to XTemac_SetHandler()
 * @{
 */
#define XTE_HANDLER_FIFOSEND     1
#define XTE_HANDLER_FIFORECV     2
#define XTE_HANDLER_FIFODMAWRITE 3
#define XTE_HANDLER_FIFODMAREAD  4
#define XTE_HANDLER_SGSEND       5
#define XTE_HANDLER_SGRECV       6
#define XTE_HANDLER_ERROR        7
/*@}*/

/* Constants to determine the configuration of the hardware device. They are
 * used to allow the driver to verify it can operate with the hardware.
 */
#define XTE_CFG_NO_DMA              1	/* No DMA */
#define XTE_CFG_SIMPLE_DMA          2	/* Simple DMA */
#define XTE_CFG_DMA_SG              3	/* DMA scatter gather */

#define XTE_MULTI_CAM_ENTRIES       4	/* Number of storable addresses in
					   the CAM */

#define XTE_MDIO_DIV_DFT            29	/* Default MDIO clock divisor */

/* Some default values for interrupt coalescing within the scatter-gather
 * DMA engine.
 */
#define XTE_SGDMA_DFT_THRESHOLD     1	/* Default pkt threshold */
#define XTE_SGDMA_MAX_THRESHOLD     1023	/* Maximum pkt theshold */
#define XTE_SGDMA_DFT_WAITBOUND     0	/* Default pkt wait bound (msec) */
#define XTE_SGDMA_MAX_WAITBOUND     1023	/* Maximum pkt wait bound (msec) */

/* The next few constants help upper layers determine the size of memory
 * pools used for Ethernet buffers and descriptor lists.
 */
#define XTE_MAC_ADDR_SIZE   6	/* six-byte MAC address */
#define XTE_MTU             1500	/* max MTU size of Ethernet frame */
#define XTE_JUMBO_MTU       8982	/* max MTU size of jumbo Ethernet frame */
#define XTE_HDR_SIZE        14	/* size of Ethernet header */
#define XTE_HDR_VLAN_SIZE   18	/* size of Ethernet header with VLAN */
#define XTE_TRL_SIZE        4	/* size of Ethernet trailer (FCS) */
#define XTE_MAX_FRAME_SIZE       (XTE_MTU + XTE_HDR_SIZE + XTE_TRL_SIZE)
#define XTE_MAX_VLAN_FRAME_SIZE  (XTE_MTU + XTE_HDR_VLAN_SIZE + XTE_TRL_SIZE)
#define XTE_MAX_JUMBO_FRAME_SIZE (XTE_JUMBO_MTU + XTE_HDR_SIZE + XTE_TRL_SIZE)

/* Constant values returned by XTemac_GetMediaInterface() */
#define XTE_MII_TYPE_MII         0
#define XTE_MII_TYPE_GMII        1
#define XTE_MII_TYPE_SGMII       2
#define XTE_MII_TYPE_RGMII       3

/* Constant values returned by XTemac_GetRgmiiStatus() */
#define XTE_RGMII_LINK_DOWN      0
#define XTE_RGMII_LINK_UP        1
#define XTE_RGMII_DUPLEX_HALF    0
#define XTE_RGMII_DUPLEX_FULL    1

/**************************** Type Definitions *******************************/

/**
 * Statistics maintained by the driver
 */
typedef struct {
	u32 TxDmaErrors; /**< Number of Tx DMA errors detected */
	u32 TxPktFifoErrors;
			 /**< Number of Tx packet FIFO errors detected */
	u32 TxStatusErrors;
			 /**< Number of Tx errors derived from XTE_TSR_OFFSET
                                  register */
	u32 RxRejectErrors;
			 /**< Number of frames discarded due to errors */
	u32 RxDmaErrors; /**< Number of Rx DMA errors detected */
	u32 RxPktFifoErrors;
			 /**< Number of Rx packet FIFO errors detected */

	u32 FifoErrors;	 /**< Number of length/status FIFO errors detected */
	u32 IpifErrors;	 /**< Number of IPIF transaction and data phase errors
                                  detected */
	u32 Interrupts;	 /**< Number of interrupts serviced */
} XTemac_SoftStats;

/**
 * This typedef contains configuration information for a device.
 */
typedef struct {
	u16 DeviceId;	/**< Unique ID  of device */
	u32 BaseAddress;/**< Physical base address of IPIF registers */
	u32 RxPktFifoDepth;
			/**< Depth of receive packet FIFO in bits */
	u32 TxPktFifoDepth;
			/**< Depth of transmit packet FIFO in bits */
	u16 MacFifoDepth;
			/**< Depth of the status/length FIFOs in entries */
	u8 IpIfDmaConfig;
			/**< IPIF/DMA hardware configuration */
	u8 DcrHost;	/**< Does device use DCR bus for host access */
	u8 Dre;		/**< Has data realignment engine */
} XTemac_Config;

/* This type encapsulates a packet FIFO channel and support attributes to
 * allow unaligned data transfers.
 */
typedef struct XTemac_PacketFifo {
	u32 Hold[2];		/* Holding register */
	unsigned ByteIndex;	/* Holding register index */
	unsigned Width;		/* Width of packet FIFO's keyhole data port in
				   bytes */
	XPacketFifoV200a Fifo;	/* Packet FIFO channel */
	/* Function used to transfer data between
	   FIFO and a buffer */
	 XStatus(*XferFn) (struct XTemac_PacketFifo * Fptr, void *BufPtr,
			   u32 ByteCount, int Eop);
} XTemac_PacketFifo;

/** @name Typedefs for callback functions
 *
 * These callbacks are invoked in interrupt context.
 * @{
 */

/**
 * Callback invoked when frame(s) have been sent in interrupt driven FIFO
 * direct or simple DMA mode. To set this callback, invoke XTemac_SetHander()
 * with XTE_HANDLER_FIFOSEND in the HandlerType parameter.
 *
 * @param CallBackRef is user data assigned when the callback was set.
 * @param StatusCnt is the number of statuses read from the device indicating
 *        a successful frame transmit.
 *
 */
typedef void (*XTemac_FifoSendHandler) (void *CallBackRef, unsigned StatusCnt);

/**
 * Callback invoked when frame(s) have been received in interrupt driven FIFO
 * direct mode. To set this callback, invoke XTemac_SetHander() with
 * XTE_HANDLER_FIFORECV in the HandlerType parameter.
 *
 * @param CallBackRef is user data assigned when the callback was set.
 *
 */
typedef void (*XTemac_FifoRecvHandler) (void *CallBackRef);

/**
 * Callback invoked when a DMA write operation to the transmit packet FIFO has
 * completed successfully.
 *
 * To set this callback, invoke XTemac_SetHander() with XTE_HANDLER_FIFODMAWRITE
 * in the HandlerType parameter.
 *
 * @param CallBackRef is user data assigned when the callback was set.
 * @param ByteCount is the number of bytes written as specified in the last call
 *        to XTemac_FifoDmaWrite()
 */
typedef void (*XTemac_FifoDmaWriteHandler) (void *CallBackRef, u32 ByteCount);

/**
 * Callback invoked when a DMA read operation from the receive packet FIFO has
 * completed successfully.
 *
 * To set this callback, invoke XTemac_SetHander() with
 * XTE_HANDLER_FIFODMAREAD in the HandlerType parameter.
 *
 * @param CallBackRef is user data assigned when the callback was set.
 * @param ByteCount is the number of bytes read as specified in the last call
 *        to XTemac_FifoDmaRead()
 */
typedef void (*XTemac_FifoDmaReadHandler) (void *CallBackRef, u32 ByteCount);

/**
 * Callback invoked when frame(s) have been sent or received in interrupt
 * driven SGDMA mode. To set the send callback, invoke XTemac_SetHandler()
 * with XTE_HANDLER_SGSEND in the HandlerType parameter. For the receive
 * callback use XTE_HANDLER_SGRECV.
 *
 * @param CallBackRef is user data assigned when the callback was set.
 */
typedef void (*XTemac_SgHandler) (void *CallBackRef);

/**
 * Callback when an asynchronous error occurs. To set this callback, invoke
 * XTemac_SetHandler() with XTE_HANDLER_ERROR in the HandlerType paramter.
 *
 * @param CallBackRef is user data assigned when the callback was set.
 * @param ErrorClass defines what class of error is being reported
 * @param ErrorWord1 definition varies with ErrorClass
 * @param ErrorWord2 definition varies with ErrorClass
 *
 * The following information lists what each ErrorClass is, the source of the
 * ErrorWords, what they mean, and if the device should be reset should it be
 * reported
 *
 * <b>ErrorClass == XST_FIFO_ERROR</b>
 *
 * This error class means there was an overrun or underrun associated
 * with one of the status or length FIFOs. This type of error cannot
 * be cleared. The user should initiate a device reset.
 *
 * ErrorWord1 is defined as a bit mask from XTE_IPXR_FIFO_FATAL_ERROR_MASK
 * that originates from the device's IPISR register.
 *
 * ErrorWord2 is reserved.
 *
 *
 * <b>ErrorClass == XST_PFIFO_DEADLOCK</b>
 *
 * This error class indicates that one of the packet FIFOs is reporting a
 * deadlock condition. This means the FIFO is reporting that it is empty and
 * full at the same time. This condition will occur when data being written
 * exceeds the capacity of the packet FIFO. The device should be reset if this
 * error is reported.
 *
 * If ErrorWord1 = XTE_RECV, then the deadlock occurred in the receive channel.
 * If ErrorWord1 = XTE_SEND, then the deadlock occurred in the send channel.
 *
 * ErrorWord2 is reserved.
 *
 *
 * <b>ErrorClass == XST_IPIF_ERROR</b>
 *
 * This error means that a register read or write caused a bus error within the
 * TEMAC's IPIF. This condition is fatal. The user should initiate a device
 * reset.
 *
 * ErrorWord1 is defined as the contents XTE_DISR_OFFSET register where these
 * errors are reported. Bits XTE_DXR_DPTO_MASK and XTE_DXR_TERR_MASK are
 * relevent in this context.
 *
 * ErrorWord2 is reserved.
 *
 *
 * <b>ErrorClass == XST_DMA_ERROR</b>
 *
 * This error class means there was a problem during a DMA tranfer.
 *
 * ErrorWord1 defines which channel caused the error XTE_RECV or XTE_SEND.
 *
 * ErrorWord2 is set to the DMA status register XDC_DMAS_REG_OFFSET (see
 * xdma_channel_i.h). The relevent bits to test are XDC_DMASR_BUS_ERROR_MASK
 * and XDC_DMASR_BUS_TIMEOUT_MASK (see xdma_channel.h). If either of these bits
 * are set, a reset is recommended.
 *
 *
 * <b>ErrorClass == XST_SEND_ERROR</b>
 *
 * This error class means there was an error during packet transmission.
 *
 * For SGDMA, this class of error is not reported. The status of a transmit is
 * stored in the device status field of BDs (which is in itself a copy of the
 * XTE_TSR_OFFSET register - see ErrorWord1). It is the user's responsibility
 * to examine this status in their XTE_HANDLER_SGSEND callback.
 *
 * ErrorWord1 is set to the transmit status word provided by the hardware
 * (XTE_TSR_OFFSET). Most of the time, none of the bits set in this word
 * should be considered a fatal condition requiring reset. Most errors are
 * due to line conditions, but if XTE_TSR_PFIFOU_MASK is set, then the
 * packet FIFO ran out of data before the packet could get out. This will
 * occur if XTemac_FifoSend() specifies more data that is currenly in the
 * packet FIFO. For SGDMA, this bit being set in the BD's device status field
 * indicates a HW glitch has occurred. In this case, a reset is recommended.
 *
 * ErrorWord2 is reserved.
 *
 *
 * <b>ErrorClass == XST_RECV_ERROR</b>
 *
 * This error class means a packet was dropped.
 *
 * ErrorWord1 is defined as the contents of the device's XTE_IPISR_OFFSET
 * relating to receive errors. If any bit is set in the
 * XTE_IPXR_RECV_DROPPED_MASK then a packet was rejected. Refer to xtemac_l.h
 * for more information on what each bit in this mask means.
 *
 * ErrorWord2 is reserved.
 *
 * No action is typically required when this error occurs.
 *
 * Reporting of this error class can be disabled by clearing the
 * XTE_REPORT_RXERR_OPTION.
 *
 * @note
 * See xtemac_l.h for bitmasks definitions and the device hardware spec for
 * further information on their meaning.
 *
 */
typedef void (*XTemac_ErrorHandler) (void *CallBackRef, XStatus ErrorClass,
				     u32 ErrorWord1, u32 ErrorWord2);
/*@}*/

/**
 * The XTemac driver instance data. The user is required to allocate a
 * structure of this type for every TEMAC device in the system. A pointer
 * to a structure of this type is then passed to the driver API functions.
 */
typedef struct XTemac {
	u32 BaseAddress;	/* Base address of IPIF register set */
	u32 IsStarted;		/* Device is currently started */
	u32 IsReady;		/* Device is initialized and ready */
	u32 IsPolled;		/* Device is in polled mode */
	u32 Options;		/* Current options word */
	u32 Flags;		/* Internal driver flags */
	XTemac_Config *Config;	/* Points to configuration entry
				   in xtemac_g.c */

	/* Packet FIFO channels */
	XTemac_PacketFifo RecvFifo;	/* Receive channel */
	XTemac_PacketFifo SendFifo;	/* Transmit channel */

	/* Simple DMA / SG DMA channels */
	XDmaV2 RecvDma;		/* Receive channel */
	XDmaV2 SendDma;		/* Transmit channel */

	/* Callbacks for FIFO direct modes */
	XTemac_FifoRecvHandler FifoRecvHandler;
	XTemac_FifoSendHandler FifoSendHandler;
	void *FifoRecvRef;
	void *FifoSendRef;

	/* Callbacks for simple DMA mode */
	XTemac_FifoDmaReadHandler FifoDmaReadHandler;
	XTemac_FifoDmaWriteHandler FifoDmaWriteHandler;
	void *FifoDmaReadRef;
	void *FifoDmaWriteRef;
	u32 DmaReadLengthRef;
	u32 DmaWriteLengthRef;

	/* Callbacks for SG DMA mode */
	XTemac_SgHandler SgRecvHandler;
	XTemac_SgHandler SgSendHandler;
	void *SgRecvRef;
	void *SgSendRef;

	/* Error callback */
	XTemac_ErrorHandler ErrorHandler;
	void *ErrorRef;

	/* Driver maintained statistics */
	XTemac_SoftStats Stats;

} XTemac;

/***************** Macros (Inline Functions) Definitions *********************/

/*****************************************************************************/
/**
*
* This macro determines if the device thinks it has received a frame. This
* function is useful if the device is operating in FIFO direct interrupt driven
* mode. For polled mode, use XTemac_FifoQueryRecvStatus().
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
*
* @return
*
* Boolean TRUE if the device interrupt status register reports that a frame
* status and length is available. FALSE otherwise.
*
* @note
*
* Signature: u32 XTemac_mIsRecvFrame(XTemac *InstancePtr)
*
******************************************************************************/
#define XTemac_mIsRecvFrame(InstancePtr)                            \
    ((XTemac_mReadReg((InstancePtr)->BaseAddress, XTE_IPISR_OFFSET) \
      & XTE_IPXR_RECV_DONE_MASK) ? TRUE : FALSE)

/*****************************************************************************/
/**
*
* This macro determines if the device thinks it has dropped a receive frame.
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
*
* @return
*
* Boolean TRUE if the device interrupt status register reports that a frame
* has been dropped. FALSE otherwise.
*
* @note
*
* Signature: u32 XTemac_mIsRecvFrame(XTemac *InstancePtr)
*
******************************************************************************/
#define XTemac_mIsRecvFrameDropped(InstancePtr)                     \
    ((XTemac_mReadReg((InstancePtr)->BaseAddress, XTE_IPISR_OFFSET) \
      & XTE_IPXR_RECV_REJECT_MASK) ? TRUE : FALSE)

/*****************************************************************************/
/**
*
* This macro determines if the device is currently configured for
* scatter-gather DMA.
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
*
* @return
*
* Boolean TRUE if the device is configured for scatter-gather DMA, or FALSE
* if it is not.
*
* @note
*
* Signature: u32 XTemac_mIsSgDma(XTemac *InstancePtr)
*
******************************************************************************/
#define XTemac_mIsSgDma(InstancePtr) \
    (((InstancePtr)->Config->IpIfDmaConfig == XTE_CFG_DMA_SG) ? TRUE : FALSE)

/*****************************************************************************/
/**
*
* This macro determines if the device is currently configured for simple DMA.
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
*
* @return
*
* Boolean TRUE if the device is configured for simple DMA, or FALSE otherwise
*
* @note
*
* Signature: u32 XTemac_mIsSimpleDma(XTemac *InstancePtr)
*
******************************************************************************/
#define XTemac_mIsSimpleDma(InstancePtr) \
    (((InstancePtr)->Config->IpIfDmaConfig == XTE_CFG_SIMPLE_DMA) ? TRUE : FALSE)

/*****************************************************************************/
/**
*
* This macro determines if the device is currently configured with DMA (either
* simple DMA or scatter-gather DMA)
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
*
* @return
*
* Boolean TRUE if the device is configured with DMA, or FALSE otherwise
*
* @note
*
* Signature: u32 XTemac_mIsDma(XTemac *InstancePtr)
*
******************************************************************************/
#define XTemac_mIsDma(InstancePtr)                              \
    ((XTemac_mIsSimpleDma(InstancePtr) == TRUE ||              \
      XTemac_mIsSgDma(InstancePtr) == TRUE) ? TRUE : FALSE)

/*****************************************************************************/
/**
*
* This macro determines if the device is configured with the Data Realignment
* Engine (DRE)
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
*
* @return
*
* Boolean TRUE if the device is configured with DRE, or FALSE otherwise.
* Note that version 1.00a has no DRE capability so this macro always returns
* FALSE.
*
* @note
*
* Signature: u32 XTemac_mIsDre(XTemac *InstancePtr)
*
******************************************************************************/
#define XTemac_mIsDre(InstancePtr) (((InstancePtr)->Config->Dre) ?      \
                                    TRUE : FALSE)

/*****************************************************************************/
/**
*
* Return the next buffer descriptor in the list on the send channel.
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
* @param BdPtr is the source descriptor
*
* @return Next descriptor in the SGDMA transmit ring (i.e. BdPtr->Next)
*
* @note
*
* Signature: XDmaBdV2 XTemac_mSgSendBdNext(XTemac *InstancePtr,
*                                          XDmaBdV2 *BdPtr)
*
******************************************************************************/
#define XTemac_mSgSendBdNext(InstancePtr, BdPtr)        \
    XDmaV2_mSgBdNext(&(InstancePtr)->SendDma, BdPtr)

/*****************************************************************************/
/**
*
* Return the next buffer descriptor in the list on the receive channel.
*
* @param InstancePtr is a pointer to the XTemac instance to be worked on.
* @param BdPtr is the source descriptor
*
* @return Next descriptor in the SGDMA receive ring (i.e. BdPtr->Next)
*
* @note
*
* Signature: XDmaBdV2 XTemac_mSgSendBdNext(XTemac *InstancePtr,
*                                          XDmaBdV2 *BdPtr)
*
******************************************************************************/
#define XTemac_mSgRecvBdNext(InstancePtr, BdPtr)        \
    XDmaV2_mSgBdNext(&(InstancePtr)->RecvDma, BdPtr)

/************************** Function Prototypes ******************************/

/*
 * Initialization functions in xtemac.c
 */
XStatus XTemac_Initialize(XTemac * InstancePtr, u16 DeviceId);
XStatus XTemac_VmInitialize(XTemac * InstancePtr, u16 DeviceId,
			    u32 VirtualAddress);
XStatus XTemac_Start(XTemac * InstancePtr);
void XTemac_Stop(XTemac * InstancePtr);
void XTemac_Reset(XTemac * InstancePtr);
XTemac_Config *XTemac_LookupConfig(u16 DeviceId);

/*
 * Fifo direct mode functions implemented in xtemac_fifo.c
 */
XStatus XTemac_FifoWrite(XTemac * InstancePtr, void *BufPtr, u32 ByteCount,
			 int Eop);
XStatus XTemac_FifoDmaWrite(XTemac * InstancePtr, XDmaBdV2 * BdPtr);
XStatus XTemac_FifoSend(XTemac * InstancePtr, u32 TxByteCount);

XStatus XTemac_FifoRecv(XTemac * InstancePtr, u32 * ByteCountPtr);
XStatus XTemac_FifoRead(XTemac * InstancePtr, void *BufPtr, u32 ByteCount,
			int Eop);
XStatus XTemac_FifoDmaRead(XTemac * InstancePtr, XDmaBdV2 * BdPtr);
u32 XTemac_FifoGetFreeBytes(XTemac * InstancePtr, u32 Direction);

XStatus XTemac_FifoQuerySendStatus(XTemac * InstancePtr, u32 * SendStatusPtr);
XStatus XTemac_FifoQueryRecvStatus(XTemac * InstancePtr);

/*
 * Interrupt management functions for FIFO direct mode implemented in
 * xtemac_intr_fifo.c.
 */
void XTemac_IntrFifoEnable(XTemac * InstancePtr, u32 Direction);
void XTemac_IntrFifoDisable(XTemac * InstancePtr, u32 Direction);
void XTemac_IntrFifoDmaEnable(XTemac * InstancePtr, u32 Direction);
void XTemac_IntrFifoDmaDisable(XTemac * InstancePtr, u32 Direction);
extern void XTemac_IntrFifoHandler(void *InstancePtr);

/*
 * SG DMA mode functions implemented in xtemac_sgdma.c
 */
XStatus XTemac_SgAlloc(XTemac * InstancePtr, u32 Direction,
		       unsigned NumBd, XDmaBdV2 ** BdPtr);
XStatus XTemac_SgCommit(XTemac * InstancePtr, u32 Direction,
			unsigned NumBd, XDmaBdV2 * BdPtr);
unsigned XTemac_SgGetProcessed(XTemac * InstancePtr, u32 Direction,
			       unsigned NumBd, XDmaBdV2 ** BdPtr);
XStatus XTemac_SgFree(XTemac * InstancePtr, u32 Direction,
		      unsigned NumBd, XDmaBdV2 * BdPtr);

XStatus XTemac_SgCheck(XTemac * InstancePtr, u32 Direction);

XStatus XTemac_SgSetSpace(XTemac * InstancePtr, u32 Direction,
			  u32 PhysicalAddr, u32 VirtualAddr,
			  u32 Alignment, unsigned BdCount,
			  XDmaBdV2 * BdTemplate);

/*
 * Interrupt management functions for SG DMA mode implemented in
 * xtemac_intr_sgdma.c
 */
void XTemac_IntrSgEnable(XTemac * InstancePtr, u32 Direction);
void XTemac_IntrSgDisable(XTemac * InstancePtr, u32 Direction);
XStatus XTemac_IntrSgCoalSet(XTemac * InstancePtr, u32 Direction,
			     u16 Threshold, u16 Timer);
XStatus XTemac_IntrSgCoalGet(XTemac * InstancePtr, u32 Direction,
			     u16 * ThresholdPtr, u16 * TimerPtr);

extern void XTemac_IntrSgHandler(void *TemacPtr);

/*
 * General interrupt-related functions in xtemac_intr.c
 */
XStatus XTemac_SetHandler(XTemac * InstancePtr, u32 HandlerType,
			  void *CallbackFunc, void *CallbackRef);

/*
 * MAC configuration/control functions in xtemac_control.c
 */
XStatus XTemac_SetOptions(XTemac * InstancePtr, u32 Options);
XStatus XTemac_ClearOptions(XTemac * InstancePtr, u32 Options);
u32 XTemac_GetOptions(XTemac * InstancePtr);

XStatus XTemac_SetMacAddress(XTemac * InstancePtr, void *AddressPtr);
void XTemac_GetMacAddress(XTemac * InstancePtr, void *AddressPtr);

XStatus XTemac_SetMacPauseAddress(XTemac * InstancePtr, void *AddressPtr);
void XTemac_GetMacPauseAddress(XTemac * InstancePtr, void *AddressPtr);
XStatus XTemac_SendPausePacket(XTemac * InstancePtr, u16 PauseValue);

void XTemac_SetIfg(XTemac * InstancePtr, u8 AdditionalIfg);
u8 XTemac_GetIfg(XTemac * InstancePtr);

void XTemac_GetPhysicalInterface(XTemac * InstancePtr, u8 * MiiType,
				 u32 * Is1000BaseX);
XStatus XTemac_GetRgmiiStatus(XTemac * InstancePtr, u8 * LinkPtr,
			      u8 * DuplexPtr);
u16 XTemac_GetLinkSpeed(XTemac * InstancePtr);
XStatus XTemac_SetMiiLinkSpeed(XTemac * InstancePtr, u16 Speed);

void XTemac_PhySetMdioDivisor(XTemac * InstancePtr, u8 Divisor);
XStatus XTemac_PhyRead(XTemac * InstancePtr, u32 PhyAddress,
		       u32 RegisterNum, u16 * PhyDataPtr);
XStatus XTemac_PhyWrite(XTemac * InstancePtr, u32 PhyAddress,
			u32 RegisterNum, u16 PhyData);

/*
 * Statistics in xtemac_stats.c
 */
void XTemac_GetSoftStats(XTemac * InstancePtr, XTemac_SoftStats * StatsPtr);
void XTemac_ClearSoftStats(XTemac * InstancePtr);

/*
 * Diagnostic functions in xtemac_selftest.c
 */
XStatus XTemac_SelfTest(XTemac * InstancePtr);

#endif				/* end of protection macro */
