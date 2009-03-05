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
 *      Module Name: rt2560.h
 *              
 *      Abstract: RT2560 ASIC related definition & structures
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 *      MarkW           10th Dec 04     Rolled in Ralink 1.4.5.0 
 ***************************************************************************/ 

#ifndef __RT2560_H__
#define __RT2560_H__

//
// Control/Status Registers (CSR)
//
#define CSR0        0x0000      // ASIC revision number
#define CSR1        0x0004      // System control register
#define CSR2        0x0008      // System admin status register (invalid)
#define CSR3        0x000C      // STA MAC address register 0
#define CSR4        0x0010      // STA MAC address register 1
#define CSR5        0x0014      // BSSID register 0
#define CSR6        0x0018      // BSSID register 1
#define CSR7        0x001C      // Interrupt source register
#define CSR8        0x0020      // Interrupt mask register
#define CSR9        0x0024      // Maximum frame length register
#define CSR11       0x002C      // Back-off control register
#define CSR12       0x0030      // Synchronization configuration register 0
#define CSR13       0x0034      // Synchronization configuration register 1
#define CSR14       0x0038      // Synchronization control register
#define CSR15       0x003C      // Synchronization status register
#define CSR16       0x0040      // TSF timer register 0
#define CSR17       0x0044      // TSF timer register 1
#define CSR18       0x0048      // IFS timer register 0
#define CSR19       0x004C      // IFS timer register 1
#define CSR20       0x0050      // WakeUp register
#define CSR21       0x0054      // EEPROM control register
#define CSR22       0x0058      // CFP Control Register

// Security coprocessor registers
#define SECCSR0     0x0028      // WEP control register
#define SECCSR1     0x0158      // WEP control register
#define SECCSR3     0x00fc      // AES  control register

// Transmit related CSRs
#define TXCSR0      0x0060      // TX cintrol register
#define TXCSR1      0x0064      // TX configuration register
#define TXCSR2      0x0068      // TX descriptor configuratioon register
#define TXCSR3      0x006C      // TX Ring Base address register
#define TXCSR4      0x0070      // Atim Ring Base address register
#define TXCSR5      0x0074      // Prio Ring Base address register
#define TXCSR6      0x0078      // Beacon base address
#define TXCSR7      0x007C      // AutoResponder Control Register
#define TXCSR8      0x0098      // CCK TX BBP registers
#define TXCSR9      0x0094      // OFDM TX BBP registers

// Receive related CSRs
#define RXCSR0      0x0080      // RX control register
#define RXCSR1      0x0084      // RX descriptorconfiguration register
#define RXCSR2      0x0088      // RX Ring base address register
#define RXCSR3      0x0090      // BBP ID register 0
//#define   RXCSR4      0x0094      // BBP ID register 1
//#define   ARCSR0      0x0098      // Auto responder PLCP config register 1
#define ARCSR1      0x009C      // Auto responder PLCP config register 1

// PCI control CSRs
#define PCICSR      0x008C      // PCI control register

//
// Alias to all ring base registers. Easier to understand constant definition
// within codes.
//
#define RX_RING_BASE_REG        (RXCSR2)
#define TX_RING_BASE_REG        (TXCSR3)
#define ATIM_RING_BASE_REG      (TXCSR4)
#define PRIO_RING_BASE_REG      (TXCSR5)
#define BEACON_BASE_REG         (TXCSR6)

// Statistic Register
#define CNT0        0x00A0      // Dot11 FCS error count
#define CNT1        0x00AC      // Dot11 PLCP error count
#define CNT2        0x00B0      // Dot11 long error count
#define CNT3        0x00B8      // Dot11 CCA false alarm count
#define CNT4        0x00BC      // Dot11 Rx FIFO overflow count
#define CNT5        0x00C0      // Dot11 Tx FIFO underrun count

// Baseband Control Register
#define PWRCSR0     0x00C4
#define PSCSR0      0x00C8
#define PSCSR1      0x00CC
#define PSCSR2      0x00D0
#define PSCSR3      0x00D4
#define PWRCSR1     0x00D8
#define TIMECSR     0x00DC
#define MACCSR0     0x00E0
#define MACCSR1     0x00E4
#define RALINKCSR   0x00E8      // Ralink Auto-reset register
#define BCNCSR      0x00EC

// BBP/RF/IF Control Register
#define BBPCSR      0x00F0
#define RFCSR       0x00F4
#define LEDCSR      0x00F8

// ASIC pointer information
#define RXPTR       0x0100      // Current RX ring address
#define TXPTR       0x0104      // Current Tx ring address
#define PRIPTR      0x0108      // Current Priority ring address
#define ATIMPTR     0x010c      // Current ATIM ring address

// some others
#define TXACKCSR0   0x0110      // TX ACK timeout
#define ACKCNT0     0x0114      // TX ACK timeout count
#define ACKCNT1     0x0118      // RX ACK timeout count

// GPIO and others
#define GPIOCSR     0x0120      // GPIO direction & in/out
#define FIFOCSR0    0x0128      // TX FIFO pointer
#define FIFOCSR1    0x012C      // RX FIFO pointer
#define BCNCSR1     0x0130      // Tx BEACON offset time, unit: 1 usec
#define MACCSR2     0x0134      // TX_PE to RX_PE delay time, unit: 1 PCI clock cycle
#define TESTCSR     0x0138      // TEST mode selection register

#define PLCP1MCSR   0x013c      // 1 Mbps ACK/CTS PLCP
#define PLCP2MCSR   0x0140      // 2 Mbps ACK/CTS PLCP
#define PLCP5MCSR   0x0144      // 5.5 Mbps ACK/CTS PLCP
#define PLCP11MCSR  0x0148      // 11 Mbps ACK/CTS PLCP

#define ARTCSR0     0x014c      // ACK/CTS payload consumed time for 1/2/5.5/11 mbps
#define ARTCSR1     0x0150      // OFDM ACK/CTS payload consumed time for 6/9/12/18 mbps
#define ARTCSR2     0x0154      // OFDM ACK/CTS payload consumed time for 24/36/48/54 mbps
#define SECCSR1     0x0158      // security control register
#define BBPCSR1     0x015c      // BBP TX configuration

#define DBANDCSR0   0x0160      // Dual band configuration register 0
#define DBANDCSR1   0x0164      // Dual band configuration register 1
#define BBPPCSR     0x0168      // BBP pin control register
#define DBGSEL0     0x016c      // MAC special debug mode selection register 0
#define DBGSEL1     0x0170      // MAC special debug mode selection register 1
#define BISTCSR     0x0174      // BBP BIST register

#define MCAST0      0x0178      // multicast filter register 0
#define MCAST1      0x017c      // multicast filter register 1

#define UARTCSR0    0x0180      // UART1 TX register
#define UARTCSR1    0x0184      // UART1 RX register
#define UARTCSR3    0x0188      // UART1 frame control register
#define UARTCSR4    0x018c      // UART1 buffer control register
#define UART2CSR0   0x0190      // UART2 TX register
#define UART2CSR1   0x0194      // UART2 RX register
#define UART2CSR3   0x0198      // UART2 frame control register
#define UART2CSR4   0x019c      // UART2 buffer control register

#define TIMECSR2    0x00a8
#define TIMECSR3    0x00b4

//
// Tx / Rx / Prio / Atim ring descriptor definition
//
#define DESC_OWN_HOST       0
#define DESC_OWN_NIC        1
#define DESC_VALID_TRUE     1
#define DESC_VALID_FALSE    0

//
// BBP & RF definition
//
#define BUSY        1
#define IDLE        0

#define BBP_Version                 0x00
#define BBP_Tx_Configure            2  // R2
#define	BBP_Tx_Tssi					1  // R1
#define BBP_Rx_Configure            14 // R14

#define PHY_TR_SWITCH_TIME          5  // usec

#define RT2560_VER_B                2
#define RT2560_VER_C                3
#define RT2560_VER_D                4
#define BBP_R17_LOW_SENSIBILITY     0x50
#define BBP_R17_MID_SENSIBILITY     0x41
#define BBP_R17_DYNAMIC_UP_BOUND    0x40
#define RSSI_FOR_LOW_SENSIBILITY    -58
#define RSSI_FOR_MID_SENSIBILITY    -74
//#define RSSI_HIGH_WATERMARK         -53
//#define RSSI_LOW_WATERMARK          -58

//-------------------------------------------------------------------------
// EEPROM definition
//-------------------------------------------------------------------------
#define EEDO        0x10
#define EEDI        0x08
#define EECS        0x04
#define EESK        0x02
#define EERL        0x01

#define EEPROM_WRITE_OPCODE     0x05
#define EEPROM_READ_OPCODE      0x06
#define EEPROM_EWDS_OPCODE      0x10
#define EEPROM_EWEN_OPCODE      0x13

#define NUM_EEPROM_BBP_PARMS        19
#define NUM_EEPROM_TX_PARMS         7
#define EEPROM_BBP_BASE_OFFSET      0x20    // 0x16
#define EEPROM_TX_PWR_OFFSET        0x46    // 0x3c
#define	EEPROM_TSSI_REF_OFFSET		0x54
#define	EEPROM_TSSI_DELTA_OFFSET	0x24
#define EEPROM_CALIBRATE_OFFSET         0x7c
#define EEPROM_VERSION_OFFSET       0x7e
#define VALID_EEPROM_VERSION        1

// =================================================================================
// TX / RX ring descriptor format
// =================================================================================

//
// TX descriptor format, Tx ring, Atim ring & Priority Ring
//
typedef struct  _TXD_STRUC {
    // Word 0
#ifdef BIG_ENDIAN
    ULONG       CipherAlg:3;
    ULONG       Rsv1:1;         //RTS:1;
    ULONG       DataByteCnt:12;
    ULONG		RetryMd:1;
    ULONG		IFS:2;
    ULONG		CipherOwn:1;
    ULONG       Ofdm:1;
    ULONG		Timestamp:1;
    ULONG		ACK:1;
    ULONG		MoreFrag:1;			// More	fragment following this	tx ring
    ULONG		RetryCount:3;		// Retry result
    ULONG		TxResult:3;			// Filled by MAC ASIC
    ULONG		Valid:1;			// Entry valid bit
    ULONG		Owner:1;			// Descriptor owner	bit
#else
    ULONG       Owner:1;            // Descriptor owner bit
    ULONG       Valid:1;            // Entry valid bit
    ULONG       TxResult:3;         // Filled by MAC ASIC
    ULONG       RetryCount:3;       // Retry result
    ULONG       MoreFrag:1;         // More fragment following this tx ring
    ULONG       ACK:1;
    ULONG       Timestamp:1;
    ULONG       Ofdm:1;
    ULONG       CipherOwn:1;
    ULONG       IFS:2;
    ULONG       RetryMd:1;
    ULONG       DataByteCnt:12;
    ULONG       Rsv1:1;         //RTS:1;
    ULONG       CipherAlg:3;
#endif

    // Word 1
    ULONG       BufferAddressPa;

    // Word 2
#ifdef BIG_ENDIAN
    ULONG       Rsv2:16;
    ULONG       CWmax:4;
    ULONG       CWmin:4;
    ULONG       Aifs:2;
    ULONG       IvOffset:6;
#else
    ULONG       IvOffset:6;
    ULONG       Aifs:2;
    ULONG       CWmin:4;
    ULONG       CWmax:4;
    ULONG       Rsv2:16;
#endif

    // Word 3
    ULONG       PlcpSignal:8;
    ULONG       PlcpService:8;
    ULONG       PlcpLengthLow:8;
    ULONG       PlcpLengthHigh:8;

    // Word 4
    ULONG       Iv;

    // Word 5
    ULONG       Eiv;

    // Word 6-9
    UCHAR       Key[16];

    // Word 10 - 11 Reserved,   not necessary to put into the structure.
#ifdef BIG_ENDIAN
    ULONG       Rsv3:24;
    ULONG       TxRate:7;    // for software use to track per-rate TX result, RATE_1, ...
    ULONG       RTS:1;
#else
    ULONG       RTS:1;
    ULONG       TxRate:7;   // software use only. keep record of the Tx rate, RATE_1,...
    ULONG       Rsv3:24;
#endif
}   TXD_STRUC, *PTXD_STRUC;

//
// Rx descriptor format, Rx Ring
//
typedef struct  _RXD_STRUC  {
    // Word 0
#ifdef BIG_ENDIAN
    ULONG       CipherAlg:3;
    ULONG       Rsv1:1;        // Drop:1;			// Drop this frame after NULL cipher operation
    ULONG		DataByteCnt:12;
    ULONG       IvOffset:6;
    ULONG       IcvError:1;
    ULONG		CipherOwner:1;
    ULONG		PhyErr:1;
    ULONG		Ofdm:1;
    ULONG		Crc:1;
    ULONG		MyBss:1;
    ULONG		Bcast:1;
    ULONG		Mcast:1;
    ULONG		U2M:1;
    ULONG		Owner:1;
#else
    ULONG       Owner:1;
    ULONG       U2M:1;
    ULONG       Mcast:1;
    ULONG       Bcast:1;
    ULONG       MyBss:1;
    ULONG       Crc:1;
    ULONG       Ofdm:1;
    ULONG       PhyErr:1;
    ULONG       CipherOwner:1;
    ULONG       IcvError:1;
    ULONG       IvOffset:6;
    ULONG       DataByteCnt:12;
    ULONG       Rsv1:1;        // Drop:1;           // Drop this frame after NULL cipher operation
    ULONG       CipherAlg:3;
#endif

    // Word 1
    ULONG       BufferAddressPa;

    // Word 2 - 3
    UCHAR       BBR0;
    UCHAR       BBR1;   // suppose to read back RSSI
    UCHAR       TA[6];

    // Word 4
    ULONG       Iv;

    // Word 5
    ULONG       Eiv;
    
    // Word 6-9
    UCHAR       Key[16];
    
    // Word 10 - 11 Reserved,   not necessary to put into the structure.
#ifdef BIG_ENDIAN
    ULONG       Rsv2:31;
    ULONG       Drop:1;
#else
    ULONG       Drop:1;
    ULONG       Rsv2:31;
#endif
}   RXD_STRUC, *PRXD_STRUC;

// =================================================================================
// CSR Registers
// =================================================================================

//
// CSR1: System control register
//
typedef union   _CSR1_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd1:29;
        ULONG		HostReady:1;		// Host is ready after initialization, 1: ready
        ULONG		BbpReset:1;			// Hardware reset BBP
		ULONG		SoftReset:1;		// Software reset bit, 1: reset, 0: normal
#else
        ULONG       SoftReset:1;        // Software reset bit, 1: reset, 0: normal
        ULONG       BbpReset:1;         // Hardware reset BBP
        ULONG       HostReady:1;        // Host is ready after initialization, 1: ready
        ULONG       Rsvd1:29;
#endif
    }   field;
    ULONG           word;
}   CSR1_STRUC, *PCSR1_STRUC;

//
// CSR3: STA MAC register 0
//
typedef union   _CSR3_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        UCHAR		Byte3;		// MAC address byte 3
        UCHAR		Byte2;		// MAC address byte 2
        UCHAR		Byte1;		// MAC address byte 1
		UCHAR		Byte0;		// MAC address byte 0
#else
        UCHAR       Byte0;      // MAC address byte 0
        UCHAR       Byte1;      // MAC address byte 1
        UCHAR       Byte2;      // MAC address byte 2
        UCHAR       Byte3;      // MAC address byte 3
#endif
    }   field;
    ULONG           word;
}   CSR3_STRUC, *PCSR3_STRUC;

//
// CSR4: STA MAC register 1
//
typedef union   _CSR4_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        UCHAR		Rsvd1;
        UCHAR		Rsvd0;
        UCHAR		Byte5;		// MAC address byte 5
		UCHAR		Byte4;		// MAC address byte 4
#else
        UCHAR       Byte4;      // MAC address byte 4
        UCHAR       Byte5;      // MAC address byte 5
        UCHAR       Rsvd0;
        UCHAR       Rsvd1;
#endif
    }   field;
    ULONG           word;
}   CSR4_STRUC, *PCSR4_STRUC;

//
// CSR5: BSSID register 0
//
typedef union   _CSR5_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        UCHAR		Byte3;		// BSSID byte 3
        UCHAR		Byte2;		// BSSID byte 2
        UCHAR		Byte1;		// BSSID byte 1
		UCHAR		Byte0;		// BSSID byte 0
#else
        UCHAR       Byte0;      // BSSID byte 0
        UCHAR       Byte1;      // BSSID byte 1
        UCHAR       Byte2;      // BSSID byte 2
        UCHAR       Byte3;      // BSSID byte 3
#endif
    }   field;
    ULONG           word;
}   CSR5_STRUC, *PCSR5_STRUC;

//
// CSR6: BSSID register 1
//
typedef union   _CSR6_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        UCHAR		Rsvd1;
        UCHAR		Rsvd0;
        UCHAR		Byte5;		// BSSID byte 5
		UCHAR		Byte4;		// BSSID byte 4
#else
        UCHAR       Byte4;      // BSSID byte 4
        UCHAR       Byte5;      // BSSID byte 5
        UCHAR       Rsvd0;
        UCHAR       Rsvd1;
#endif
    }   field;
    ULONG           word;
}   CSR6_STRUC, *PCSR6_STRUC;

//
// CSR7: Interrupt source register
// Write one to clear corresponding bit
//
typedef union   _CSR7_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:12;
        ULONG       Timecsr3Expired:1;      // TIMECSR3 hardware timer expired (for 802.1H quiet period)
        ULONG       Uart2RxBufferError:1;   // UART2 RX buffer error
        ULONG       Uart2TxBufferError:1;   // UART2 TX buffer error
        ULONG       Uart2IdleThreshold:1;   // UART2 IDLE over threshold
        ULONG       Uart2RxThreshold:1;     // UART2 RX reaches threshold
        ULONG       Uart2TxThreshold:1;     // UART2 TX reaches threshold
        ULONG       UartRxBufferError:1;    // UART1 RX buffer error
        ULONG       UartTxBufferError:1;    // UART1 TX buffer error
        ULONG       UartIdleThreshold:1;    // UART1 IDLE over threshold
        ULONG       UartRxThreshold:1;      // UART1 RX reaches threshold
        ULONG       UartTxThreshold:1;      // UART1 TX reaches threshold
        ULONG		EncryptionDone:1;	// Encryption done interrupt
        ULONG		DecryptionDone:1;	// Decryption done interrupt
        ULONG		RxDone:1;			// Receive done interrupt
        ULONG		PrioRingTxDone:1;	// Priority ring transmit done interrupt
        ULONG		AtimRingTxDone:1;	// Atim ring transmit done interrupt
        ULONG		TxRingTxDone:1;		// Tx ring transmit done interrupt
        ULONG		TatimwExpire:1;		// Timer of atim window expired interrupt
        ULONG		TwakeExpire:1;		// Wakeup timer expired interrupt
		ULONG		TbcnExpire:1;		// Beacon timer expired interrupt
#else
        ULONG       TbcnExpire:1;       // Beacon timer expired interrupt
        ULONG       TwakeExpire:1;      // Wakeup timer expired interrupt
        ULONG       TatimwExpire:1;     // Timer of atim window expired interrupt
        ULONG       TxRingTxDone:1;     // Tx ring transmit done interrupt
        ULONG       AtimRingTxDone:1;   // Atim ring transmit done interrupt
        ULONG       PrioRingTxDone:1;   // Priority ring transmit done interrupt
        ULONG       RxDone:1;           // Receive done interrupt
        ULONG       DecryptionDone:1;   // Decryption done interrupt
        ULONG       EncryptionDone:1;   // Encryption done interrupt
        ULONG       UartTxThreshold:1;      // UART1 TX reaches threshold
        ULONG       UartRxThreshold:1;      // UART1 RX reaches threshold
        ULONG       UartIdleThreshold:1;    // UART1 IDLE over threshold
        ULONG       UartTxBufferError:1;    // UART1 TX buffer error
        ULONG       UartRxBufferError:1;    // UART1 RX buffer error
        ULONG       Uart2TxThreshold:1;     // UART2 TX reaches threshold
        ULONG       Uart2RxThreshold:1;     // UART2 RX reaches threshold
        ULONG       Uart2IdleThreshold:1;   // UART2 IDLE over threshold
        ULONG       Uart2TxBufferError:1;   // UART2 TX buffer error
        ULONG       Uart2RxBufferError:1;   // UART2 RX buffer error
        ULONG       Timecsr3Expired:1;      // TIMECSR3 hardware timer expired (for 802.1H quiet period)
        ULONG       Rsvd:12;
#endif
    }   field;
    ULONG           word;
}   CSR7_STRUC, *PCSR7_STRUC, INTSRC_STRUC, *PINTSRC_STRUC;

//
// CSR8: Interrupt Mask register
// Write one to mask off interrupt
//
typedef union   _CSR8_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:12;
        ULONG       Timecsr3Expired:1;      // TIMECSR3 hardware timer expired (for 802.1H quiet period)
        ULONG       Uart2RxBufferError:1;   // UART2 RX buffer error
        ULONG       Uart2TxBufferError:1;   // UART2 TX buffer error
        ULONG       Uart2IdleThreshold:1;   // UART2 IDLE over threshold
        ULONG       Uart2RxThreshold:1;     // UART2 RX reaches threshold
        ULONG       Uart2TxThreshold:1;     // UART2 TX reaches threshold
        ULONG       UartRxBufferError:1;    // UART1 RX buffer error
        ULONG       UartTxBufferError:1;    // UART1 TX buffer error
        ULONG       UartIdleThreshold:1;    // UART1 IDLE over threshold
        ULONG       UartRxThreshold:1;      // UART1 RX reaches threshold
        ULONG       UartTxThreshold:1;      // UART1 TX reaches threshold
        ULONG		EncryptionDone:1;	// Encryption done interrupt
        ULONG		DecryptionDone:1;	// Decryption done interrupt
        ULONG		RxDone:1;			// Receive done interrupt mask
        ULONG		PrioRingTxDone:1;	// Priority ring transmit done interrupt mask
        ULONG		AtimRingTxDone:1;	// Atim ring transmit done interrupt mask
        ULONG		TxRingTxDone:1;		// Tx ring transmit done interrupt mask
        ULONG		TatimwExpire:1;		// Timer of atim window expired interrupt mask
        ULONG		TwakeExpire:1;		// Wakeup timer expired interrupt mask
		ULONG		TbcnExpire:1;		// Beacon timer expired interrupt mask
#else
        ULONG       TbcnExpire:1;       // Beacon timer expired interrupt mask
        ULONG       TwakeExpire:1;      // Wakeup timer expired interrupt mask
        ULONG       TatimwExpire:1;     // Timer of atim window expired interrupt mask
        ULONG       TxRingTxDone:1;     // Tx ring transmit done interrupt mask
        ULONG       AtimRingTxDone:1;   // Atim ring transmit done interrupt mask
        ULONG       PrioRingTxDone:1;   // Priority ring transmit done interrupt mask
        ULONG       RxDone:1;           // Receive done interrupt mask
        ULONG       DecryptionDone:1;   // Decryption done interrupt
        ULONG       EncryptionDone:1;   // Encryption done interrupt
        ULONG       UartTxThreshold:1;      // UART1 TX reaches threshold
        ULONG       UartRxThreshold:1;      // UART1 RX reaches threshold
        ULONG       UartIdleThreshold:1;    // UART1 IDLE over threshold
        ULONG       UartTxBufferError:1;    // UART1 TX buffer error
        ULONG       UartRxBufferError:1;    // UART1 RX buffer error
        ULONG       Uart2TxThreshold:1;     // UART2 TX reaches threshold
        ULONG       Uart2RxThreshold:1;     // UART2 RX reaches threshold
        ULONG       Uart2IdleThreshold:1;   // UART2 IDLE over threshold
        ULONG       Uart2TxBufferError:1;   // UART2 TX buffer error
        ULONG       Uart2RxBufferError:1;   // UART2 RX buffer error
        ULONG       Timecsr3Expired:1;      // TIMECSR3 hardware timer expired (for 802.1H quiet period)
        ULONG       Rsvd:12;
#endif
    }   field;
    ULONG           word;
}   CSR8_STRUC, *PCSR8_STRUC, INTMSK_STRUC, *PINTMSK_STRUC;

//
// CSR9: Maximum frame length register
//
typedef union   _CSR9_STRUC {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd1:20;
        ULONG		MaxFrameUnit:5;		// Maximum frame legth in 128B unit, default is 12 = 0xC.
		ULONG		Rsvd0:7;
#else
        ULONG       Rsvd0:7;
        ULONG       MaxFrameUnit:5;     // Maximum frame legth in 128B unit, default is 12 = 0xC.
        ULONG       Rsvd1:20;
#endif
    }   field;
    ULONG           word;
}   CSR9_STRUC, *PCSR9_STRUC;

//
// SECCSR0: WEP control register
//
typedef union   _SECCSR0_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		DescAddress:30;		// Descriptor physical address of frame in one-shot-mode.
        ULONG		OneShotMode:1;		// 1: One shot only mode, 0: ring mode
		ULONG		KickDecypt:1;		// Kick decryption engine, self-clear
#else
        ULONG       KickDecypt:1;       // Kick decryption engine, self-clear
        ULONG       OneShotMode:1;      // 1: One shot only mode, 0: ring mode
        ULONG       DescAddress:30;     // Descriptor physical address of frame in one-shot-mode.
#endif
    }   field;
    ULONG           word;
}   SECCSR0_STRUC, *PSECCSR0_STRUC;

//
// SECCSR1: WEP control register
//
typedef union   _SECCSR1_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		DescAddress:30;		// Descriptor physical address of frame in one-shot-mode.
        ULONG		OneShotMode:1;		// 1: One shot only mode, 0: ring mode
		ULONG		KickEncypt:1;		// Kick encryption engine, self-clear
#else
        ULONG       KickEncypt:1;       // Kick encryption engine, self-clear
        ULONG       OneShotMode:1;      // 1: One shot only mode, 0: ring mode
        ULONG       DescAddress:30;     // Descriptor physical address of frame in one-shot-mode.
#endif
    }   field;
    ULONG           word;
}   SECCSR1_STRUC, *PSECCSR1_STRUC;

//
// CSR11: Back-Off control register
//
typedef union   _CSR11_STRUC    {
    struct {
#ifdef BIG_ENDIAN
        ULONG		ShortRetry:8;	// Short retry count
        ULONG		LongRetry:8;	// Long retry count
        ULONG		Rsvd:2;
        ULONG       CWSelect:1;     // 1: CWmin/CWmax select from register, 0: select from TxD
        ULONG		SlotTime:5;		// Slot time, default is 20us for 802.11B
        ULONG		CWMax:4;		// Bit for Cwmax, default Cwmax is 1023 (2^10 - 1).
		ULONG		CWMin:4;		// Bit for Cwmin. default Cwmin is 31 (2^5 - 1).
#else
        ULONG       CWMin:4;        // Bit for Cwmin. default Cwmin is 31 (2^5 - 1).
        ULONG       CWMax:4;        // Bit for Cwmax, default Cwmax is 1023 (2^10 - 1).
        ULONG       SlotTime:5;     // Slot time, default is 20us for 802.11B
        ULONG       CWSelect:1;     // 1: CWmin/Cwmax select from register, 0:select from TxD
        ULONG       Rsvd:2;
        ULONG       LongRetry:8;    // Long retry count
        ULONG       ShortRetry:8;   // Short retry count
#endif
    }   field;
    ULONG           word;
}   CSR11_STRUC, *PCSR11_STRUC; 

//
// CSR12: Synchronization configuration register 0
// All uint in 1/16 TU
//
typedef union   _CSR12_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		CfpMaxDuration:16;	// Beacon interval, default is 100 TU
		ULONG		BeaconInterval:16;	// CFP maximum duration, default is 100 TU
#else
        ULONG       BeaconInterval:16;  // CFP maximum duration, default is 100 TU
        ULONG       CfpMaxDuration:16;  // Beacon interval, default is 100 TU
#endif
    }   field;
    ULONG           word;
}   CSR12_STRUC, *PCSR12_STRUC;

//
// CSR13: Synchronization configuration register 1
// All uint in 1/16 TU
//
typedef union   _CSR13_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:8;
        ULONG		CfpPeriod:8;		// CFP period, default is 0 TU
		ULONG		AtimwDuration:16;	// ATIM window duration, default is 10 TU
#else
        ULONG       AtimwDuration:16;   // ATIM window duration, default is 10 TU
        ULONG       CfpPeriod:8;        // CFP period, default is 0 TU
        ULONG       Rsvd:8;
#endif
    }   field;
    ULONG           word;
}   CSR13_STRUC, *PCSR13_STRUC;

//
// CSR14: Synchronization control register
//
typedef union   _CSR14_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		TbcnPreload:16;		// Tbcn preload value
        ULONG		CfpCntPreload:8;	// Cfp count preload value
        ULONG		Rsvd:1;
        ULONG		BeaconGen:1;		// Enable beacon generator
        ULONG		Tatimw:1;			// Enable Tatimw & ATIM window switching
        ULONG		Tcfp:1;				// Enable Tcfp & CFP / CP switching
        ULONG		Tbcn:1;				// Enable Tbcn with reload value
        ULONG		TsfSync:2;			// Enable TSF sync, 00: disable, 01: infra mode, 10: ad-hoc mode
		ULONG		TsfCount:1;			// Enable TSF auto counting
#else
        ULONG       TsfCount:1;         // Enable TSF auto counting
        ULONG       TsfSync:2;          // Enable TSF sync, 00: disable, 01: infra mode, 10: ad-hoc mode
        ULONG       Tbcn:1;             // Enable Tbcn with reload value
        ULONG       Tcfp:1;             // Enable Tcfp & CFP / CP switching
        ULONG       Tatimw:1;           // Enable Tatimw & ATIM window switching
        ULONG       BeaconGen:1;        // Enable beacon generator
        ULONG       Rsvd:1;
        ULONG       CfpCntPreload:8;    // Cfp count preload value
        ULONG       TbcnPreload:16;     // Tbcn preload value
#endif
    }   field;
    ULONG           word;
}   CSR14_STRUC, *PCSR14_STRUC;

//
// CSR15: Synchronization status register
//
typedef union   _CSR15_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:29;
        ULONG		BeaconSent:1;	// Beacon sent
        ULONG		Atimw:1;		// Atim window period
		ULONG		Cfp:1;			// CFP period
#else
        ULONG       Cfp:1;          // CFP period
        ULONG       Atimw:1;        // Atim window period
        ULONG       BeaconSent:1;   // Beacon sent
        ULONG       Rsvd:29;
#endif
    }   field;
    ULONG           word;
}   CSR15_STRUC, *PCSR15_STRUC;

//
// CSR18: IFS Timer register 0
//
typedef union   _CSR18_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd1:7;
        ULONG		PIFS:9;	// PIFS, default is 30 TU
        ULONG		Rsvd0:7;
		ULONG		SIFS:9;	// SIFS, default is 10 TU
#else
        ULONG       SIFS:9; // SIFS, default is 10 TU
        ULONG       Rsvd0:7;
        ULONG       PIFS:9; // PIFS, default is 30 TU
        ULONG       Rsvd1:7;
#endif
    }   field;
    ULONG           word;
}   CSR18_STRUC, *PCSR18_STRUC;

//
// CSR19: IFS Timer register 1
//
typedef union   _CSR19_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		EIFS:16;	// EIFS, default is 364 TU
		ULONG		DIFS:16;	// DIFS, default is 50 TU
#else
        ULONG       DIFS:16;    // DIFS, default is 50 TU
        ULONG       EIFS:16;    // EIFS, default is 364 TU
#endif
    }   field;
    ULONG           word;
}   CSR19_STRUC, *PCSR19_STRUC;

//
// CSR20: Wakeup timer register
//
typedef union   _CSR20_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:7;
        ULONG		AutoWake:1;				// Enable auto wakeup / sleep mechanism
        ULONG		NumBcnBeforeWakeup:8;	// Number of beacon before wakeup
		ULONG		DelayAfterBcn:16;		// Delay after Tbcn expired in units of 1/16 TU
#else
        ULONG       DelayAfterBcn:16;       // Delay after Tbcn expired in units of 1/16 TU
        ULONG       NumBcnBeforeWakeup:8;   // Number of beacon before wakeup
        ULONG       AutoWake:1;             // Enable auto wakeup / sleep mechanism
        ULONG       Rsvd:7;
#endif
    }   field;
    ULONG           word;
}   CSR20_STRUC, *PCSR20_STRUC;

//
// CSR21: EEPROM control register
//
typedef union   _CSR21_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:26;
        ULONG		Type:1;			// 1: 93C46, 0:93C66
        ULONG		EepromDO:1;
        ULONG		EepromDI:1;
        ULONG		EepromCS:1;
        ULONG		EepromSK:1;
		ULONG		Reload:1;		// Reload EEPROM content, write one to reload, self-cleared.
#else
        ULONG       Reload:1;       // Reload EEPROM content, write one to reload, self-cleared.
        ULONG       EepromSK:1;
        ULONG       EepromCS:1;
        ULONG       EepromDI:1;
        ULONG       EepromDO:1;
        ULONG       Type:1;         // 1: 93C46, 0:93C66
        ULONG       Rsvd:26;
#endif
    }   field;
    ULONG           word;
}   CSR21_STRUC, *PCSR21_STRUC;

//
// CSR22: CFP control register
//
typedef union   _CSR22_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:15;
        ULONG		ReloadCfpDurRemain:1;	// Reload CFP duration remain, write one to reload, self-cleared
		ULONG		CfpDurRemain:16;		// CFP duration remain, in units of TU
#else
        ULONG       CfpDurRemain:16;        // CFP duration remain, in units of TU
        ULONG       ReloadCfpDurRemain:1;   // Reload CFP duration remain, write one to reload, self-cleared
        ULONG       Rsvd:15;
#endif
    }   field;
    ULONG           word;
}   CSR22_STRUC, *PCSR22_STRUC;

// =================================================================================
// TX / RX Registers
// =================================================================================

//
// TXCSR0 <0x0060> : TX Control Register 
//
typedef union   _TXCSR0_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:28;
        ULONG		Abort:1;		// Abort all transmit related ring operation
        ULONG		KickPrio:1;		// Kick priority ring
        ULONG		KickAtim:1;		// Kick ATIM ring
		ULONG		KickTx:1;		// Kick Tx ring
#else
        ULONG       KickTx:1;       // Kick Tx ring 
        ULONG       KickAtim:1;     // Kick ATIM ring
        ULONG       KickPrio:1;     // Kick priority ring
        ULONG       Abort:1;        // Abort all transmit related ring operation
        ULONG       Rsvd:28;
#endif
    }   field;  
    ULONG           word;
}   TXCSR0_STRUC, *PTXCSR0_STRUC;

//
// TXCSR1 <0x0064> : TX Configuration Register
//
typedef union   _TXCSR1_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Reserved:7;
        ULONG		AutoResponder:1;	// enable auto responder which include ACK & CTS
        ULONG		TsFOffset:6;		// Insert Tsf offset
        ULONG		AckConsumeTime:9;	// ACK consume time, default = SIFS + ACKtime @ 1Mbps
        ULONG		AckTimeOut:9;		// Ack timeout, default = SIFS + 2*SLOT_ACKtime @ 1Mbps
#else
        ULONG       AckTimeOut:9;       // Ack timeout, default = SIFS + 2*SLOT_ACKtime @ 1Mbps
        ULONG       AckConsumeTime:9;   // ACK consume time, default = SIFS + ACKtime @ 1Mbps
        ULONG       TsFOffset:6;        // Insert Tsf offset
        ULONG       AutoResponder:1;    // enable auto responder which include ACK & CTS
        ULONG       Reserved:7;
#endif
    }   field;
    ULONG           word;
}   TXCSR1_STRUC, *PTXCSR1_STRUC;

//
// TXCSR2: Tx descriptor configuration register
//
typedef union   _TXCSR2_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		NumPrioD:8;		// Number of PriorityD in ring
        ULONG		NumAtimD:8;		// Number of AtimD in ring
        ULONG		NumTxD:8;		// Number of TxD in ring
		ULONG		TxDSize:8;		// Tx descriptor size, default is 48
#else
        ULONG       TxDSize:8;      // Tx descriptor size, default is 48
        ULONG       NumTxD:8;       // Number of TxD in ring
        ULONG       NumAtimD:8;     // Number of AtimD in ring
        ULONG       NumPrioD:8;     // Number of PriorityD in ring
#endif
    }   field;
    ULONG           word;
}   TXCSR2_STRUC, *PTXCSR2_STRUC;

//
// TXCSR7: Auto responder control register
//
typedef union   _TXCSR7_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:31;
		ULONG		ARPowerManage:1;	// Auto responder power management bit
#else
        ULONG       ARPowerManage:1;    // Auto responder power management bit
        ULONG       Rsvd:31;
#endif
    }   field;
    ULONG       word;
}   TXCSR7_STRUC, *PTXCSR7_STRUC;

//
// TXCSR8: CCK Tx BBP register
//
typedef union   _TXCSR8_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		CckLenHigh:8;	// BBP length high byte address for CCK
        ULONG		CckLenLow:8;	// BBP length low byte address for CCK
        ULONG		CckService:8;	// BBP service field address for CCK
		ULONG		CckSignal:8;	// BBP signal field address for CCK
#else
        ULONG       CckSignal:8;    // BBP signal field address for CCK
        ULONG       CckService:8;   // BBP service field address for CCK
        ULONG       CckLenLow:8;    // BBP length low byte address for CCK
        ULONG       CckLenHigh:8;   // BBP length high byte address for CCK
#endif
    }   field;
    ULONG       word;
}   TXCSR8_STRUC, *PTXCSR8_STRUC;

//
// TXCSR9: OFDM Tx BBP register
//
typedef union   _TXCSR9_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		OfdmLenHigh:8;	// BBP length high byte address for OFDM
        ULONG		OfdmLenLow:8;	// BBP length low byte address for OFDM
        ULONG		OfdmService:8;	// BBP service field address for OFDM
		ULONG		OfdmRate:8;		// BBP rate field address for OFDM
#else
        ULONG       OfdmRate:8;     // BBP rate field address for OFDM
        ULONG       OfdmService:8;  // BBP service field address for OFDM
        ULONG       OfdmLenLow:8;   // BBP length low byte address for OFDM
        ULONG       OfdmLenHigh:8;  // BBP length high byte address for OFDM
#endif
    }   field;
    ULONG       word;
}   TXCSR9_STRUC, *PTXCSR9_STRUC;

//
// RXCSR0 <0x0080> : RX Control Register
//
typedef union   _RXCSR0_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Reserved:20;
        ULONG       EnableQos:1;        // 1: accept QOS data frame format and parse the QOS field
        ULONG		DropBcast:1;		// Drop broadcast frames
        ULONG		DropMcast:1;		// Drop multicast frames
        ULONG		PassPlcp:1;			// Pass all receive packet with 4 bytes PLCP attached
        ULONG		PassCRC:1;			// Pass all receive packet to host with CRC attached
        ULONG		DropVersionErr:1;	// Drop version error frame
        ULONG		DropToDs:1;			// Drop fram ToDs bit is true
        ULONG		DropNotToMe:1;		// Drop not to me unicast frame
        ULONG		DropControl:1;		// Drop control frame
        ULONG		DropPhysical:1;		// Drop physical error
        ULONG		DropCRC:1;			// Drop CRC error
		ULONG		DisableRx:1;		// Disable Rx engine
#else
        ULONG       DisableRx:1;        // Disable Rx engine
        ULONG       DropCRC:1;          // Drop CRC error
        ULONG       DropPhysical:1;     // Drop physical error
        ULONG       DropControl:1;      // Drop control frame
        ULONG       DropNotToMe:1;      // Drop not to me unicast frame
        ULONG       DropToDs:1;         // Drop fram ToDs bit is true
        ULONG       DropVersionErr:1;   // Drop version error frame
        ULONG       PassCRC:1;          // Pass all receive packet to host with CRC attached
        ULONG       PassPlcp:1;         // Pass all receive packet with 4 bytes PLCP attached
        ULONG       DropMcast:1;        // Drop multicast frames
        ULONG       DropBcast:1;        // Drop broadcast frames
        ULONG       EnableQos:1;        // 1: accept QOS data frame format and parse the QOS field
        ULONG       Reserved:20;
#endif
    }   field;
    ULONG           word;
}   RXCSR0_STRUC, *PRXCSR0_STRUC;

//
// RXCSR1: RX descriptor configuration register
//
typedef union   _RXCSR1_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:16;
        ULONG		NumRxD:8;		// Number of RxD in ring.
		ULONG		RxDSize:8;		// Rx descriptor size, default is 32B.
#else
        ULONG       RxDSize:8;      // Rx descriptor size, default is 32B.
        ULONG       NumRxD:8;       // Number of RxD in ring.
        ULONG       Rsvd:16;
#endif
    }   field;
    ULONG           word;
}   RXCSR1_STRUC, *PRXCSR1_STRUC;

//
// RXCSR3: BBP ID register for Rx operation
//
typedef union   _RXCSR3_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		ValidBbp3:1;		// BBP register 3 ID is valid or not
        ULONG		IdBbp3:7;			// BBP register 3 ID
        ULONG		ValidBbp2:1;		// BBP register 2 ID is valid or not
        ULONG		IdBbp2:7;			// BBP register 2 ID
        ULONG		ValidBbp1:1;		// BBP register 1 ID is valid or not
        ULONG		IdBbp1:7;			// BBP register 1 ID
        ULONG		ValidBbp0:1;		// BBP register 0 ID is valid or not
		ULONG		IdBbp0:7;			// BBP register 0 ID
#else
        ULONG       IdBbp0:7;           // BBP register 0 ID
        ULONG       ValidBbp0:1;        // BBP register 0 ID is valid or not
        ULONG       IdBbp1:7;           // BBP register 1 ID
        ULONG       ValidBbp1:1;        // BBP register 1 ID is valid or not
        ULONG       IdBbp2:7;           // BBP register 2 ID
        ULONG       ValidBbp2:1;        // BBP register 2 ID is valid or not
        ULONG       IdBbp3:7;           // BBP register 3 ID
        ULONG       ValidBbp3:1;        // BBP register 3 ID is valid or not
#endif
    }   field;
    ULONG           word;
}   RXCSR3_STRUC, *PRXCSR3_STRUC;
#if 0
//
// RXCSR4: BBP ID register for Rx operation
//
typedef union   _RXCSR4_STRUC   {
    struct  {
        ULONG       IdBbp4:7;           // BBP register 4 ID
        ULONG       ValidBbp4:1;        // BBP register 4 ID is valid or not
        ULONG       IdBbp5:7;           // BBP register 5 ID
        ULONG       ValidBbp5:1;        // BBP register 5 ID is valid or not
        ULONG       Rsvd:16;
    }   field;
    ULONG           word;
}   RXCSR4_STRUC, *PRXCSR4_STRUC;

//
// ARCSR0: Auto Responder PLCP value register 0
//
typedef union   _ARCSR0_STRUC   {
    struct  {
        ULONG       ArBbpData0:8;       // Auto responder BBP register 0 data
        ULONG       ArBbpId0:8;         // Auto responder BBP register 0 Id
        ULONG       ArBbpData1:8;       // Auto responder BBP register 1 data
        ULONG       ArBbpId1:8;         // Auto responder BBP register 1 Id
    }   field;
    ULONG           word;
}   ARCSR0_STRUC, *PARCSR0_STRUC;

//
// ARCSR0: Auto Responder PLCP value register 1
//
typedef union   _ARCSR1_STRUC   {
    struct  {
        ULONG       ArBbpData2:8;       // Auto responder BBP register 2 data
        ULONG       ArBbpId2:8;         // Auto responder BBP register 2 Id
        ULONG       ArBbpData3:8;       // Auto responder BBP register 3 data
        ULONG       ArBbpId3:8;         // Auto responder BBP register 3 Id
    }   field;
    ULONG           word;
}   ARCSR1_STRUC, *PARCSR1_STRUC;
#endif
// =================================================================================
// Miscellaneous Registers
// =================================================================================

//
// PCISR: PCI control register
//
typedef union   _PCICSR_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:22;
        ULONG		WriteInvalid:1;		// Enable memory write & invalid
        ULONG		ReadMultiple:1;		// Enable memory read multiple
        ULONG		EnableClk:1;		// Enable CLK_RUN, PCI clock can't going down to non-operational
        ULONG		BurstLength:2;		// PCI burst length
										// 01: 8DW, 10: 16DW, 11:32DW, default 00: 4DW
        ULONG		TxThreshold:2;		// Tx threshold in DW to start PCI access
										// 01: 1DW, 10: 4DW, 11: store & forward, default 00: 0DW
        ULONG		RxThreshold:2;		// Rx threshold in DW to start PCI access
										// 01: 8DW, 10: 4DW, 11: 32DW, default 00: 16DW
        ULONG		BigEndian:1;		// 1: big endian, 0: little endian
#else
        ULONG       BigEndian:1;        // 1: big endian, 0: little endian
        ULONG       RxThreshold:2;      // Rx threshold in DW to start PCI access
                                        // 01: 8DW, 10: 4DW, 11: 32DW, default 00: 16DW
        ULONG       TxThreshold:2;      // Tx threshold in DW to start PCI access
                                        // 01: 1DW, 10: 4DW, 11: store & forward, default 00: 0DW
        ULONG       BurstLength:2;      // PCI burst length
                                        // 01: 8DW, 10: 16DW, 11:32DW, default 00: 4DW
        ULONG       EnableClk:1;        // Enable CLK_RUN, PCI clock can't going down to non-operational
        ULONG       ReadMultiple:1;     // Enable memory read multiple
        ULONG       WriteInvalid:1;     // Enable memory write & invalid
        ULONG       Rsvd:22;
#endif
    }   field;
    ULONG           word;
}   PCICSR_STRUC, *PPCICSR_STRUC;

//
// PWRCSR0: Power mode configuration register
// Driver did not control it for now.

//
// PSCSR0: Power saving delay time register 0
// Driver did not control it for now.

//
// PSCSR1: Power saving delay time register 1
// Driver did not control it for now.

//
// PSCSR2: Power saving delay time register 2
// Driver did not control it for now.

//
// PSCSR3: Power saving delay time register 3
// Driver did not control it for now.

//
// PWRCSR1: Manual power control / status register
//
typedef union   _PWRCSR1_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG       Rsvd:22;
        ULONG       PutToSleep:1;
        ULONG       RfCurrState:2;
        ULONG		BbpCurrState:2;
        ULONG		RfDesireState:2;
        ULONG		BbpDesireState:2;
		ULONG		SetState:1;
#else
        ULONG       SetState:1; 
        ULONG       BbpDesireState:2;
        ULONG       RfDesireState:2;
        ULONG       BbpCurrState:2;
        ULONG       RfCurrState:2;
        ULONG       PutToSleep:1;
        ULONG       Rsvd:22;
#endif
    }   field;
    ULONG           word;
}   PWRCSR1_STRUC, *PPWRCSR1_STRUC;

//
// TIMECSR: Timer control register
//
typedef union   _TIMECSR_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG       Rsvd:13;
        ULONG		BeaconExpect:3;	// Beacon expect window
        ULONG		Us64Cnt:8;		// 64 us timer count in units of 1 us timer
		ULONG		UsCnt:8;		// 1 us timer count in units of clock cycles
#else
        ULONG       UsCnt:8;        // 1 us timer count in units of clock cycles
        ULONG       Us64Cnt:8;      // 64 us timer count in units of 1 us timer
        ULONG       BeaconExpect:3; // Beacon expect window
        ULONG       Rsvd:13;
#endif
    }   field;
    ULONG           word;
}   TIMECSR_STRUC, *PTIMECSR_STRUC;

//
// MACCSR0: MAC configuration register 0
//

//
// MACCSR1: MAC configuration register 1
//
typedef union   _MACCSR1_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:24;
        ULONG		IntersilIF:1;		// Intersil IF calibration pin
        ULONG		LoopBack:2;			// Loopback mode. 00: normal, 01: internal, 10: external, 11:rsvd.
        ULONG		AutoRxBbp:1;		// Auto Rx logic access BBP control register
        ULONG		AutoTxBbp:1;		// Auto Tx logic access BBP control register
        ULONG		BbpRxResetMode:1;	// Ralink BBP RX reset mode
        ULONG		OneShotRxMode:1;	// Enable one-shot Rx mode for debugging
		ULONG		KickRx:1;			// Kick one-shot Rx in one-shot Rx mode
#else
        ULONG       KickRx:1;           // Kick one-shot Rx in one-shot Rx mode
        ULONG       OneShotRxMode:1;    // Enable one-shot Rx mode for debugging
        ULONG       BbpRxResetMode:1;   // Ralink BBP RX reset mode
        ULONG       AutoTxBbp:1;        // Auto Tx logic access BBP control register
        ULONG       AutoRxBbp:1;        // Auto Rx logic access BBP control register
        ULONG       LoopBack:2;         // Loopback mode. 00: normal, 01: internal, 10: external, 11:rsvd.
        ULONG       IntersilIF:1;       // Intersil IF calibration pin
        ULONG       Rsvd:24;
#endif
    }   field;
    ULONG           word;
}   MACCSR1_STRUC, *PMACCSR1_STRUC;

//
// RALINKCSR: Ralink Rx auto-reset BBCR
//
typedef union   _RALINKCSR_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		ArBbpValid1:1;		// Auto reset BBP register 1 is valid
        ULONG		ArBbpId1:7;			// Auto reset BBP register 1 Id
        ULONG		ArBbpData1:8;		// Auto reset BBP register 1 data
        ULONG		ArBbpValid0:1;		// Auto reset BBP register 0 is valid
        ULONG		ArBbpId0:7;			// Auto reset BBP register 0 Id
		ULONG		ArBbpData0:8;		// Auto reset BBP register 0 data
#else
        ULONG       ArBbpData0:8;       // Auto reset BBP register 0 data
        ULONG       ArBbpId0:7;         // Auto reset BBP register 0 Id
        ULONG       ArBbpValid0:1;      // Auto reset BBP register 0 is valid
        ULONG       ArBbpData1:8;       // Auto reset BBP register 1 data
        ULONG       ArBbpId1:7;         // Auto reset BBP register 1 Id
        ULONG       ArBbpValid1:1;      // Auto reset BBP register 1 is valid
#endif
    }   field;
    ULONG           word;
}   RALINKCSR_STRUC, *PRALINKCSR_STRUC;

//
// BCNCSR: Beacon interval control register
//
typedef union   _BCNCSR_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:16;
        ULONG		Plus:1;			// plus or minus delta time value
        ULONG		Mode:2;			// please refer to ASIC specs.
        ULONG		NumBcn:8;		// Delta time value or number of beacon according to mode
        ULONG		DeltaTime:4;	// The delta time value
		ULONG		Change:1;		// Write one to change beacon interval
#else
        ULONG       Change:1;       // Write one to change beacon interval
        ULONG       DeltaTime:4;    // The delta time value
        ULONG       NumBcn:8;       // Delta time value or number of beacon according to mode
        ULONG       Mode:2;         // please refer to ASIC specs.
        ULONG       Plus:1;         // plus or minus delta time value
        ULONG       Rsvd:16;
#endif
    }   field;
    ULONG           word;
}   BCNCSR_STRUC, *PBCNCSR_STRUC;

//
// BBPCSR: BBP serial control register
//
typedef union   _BBPCSR_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:15;
        ULONG		WriteControl:1;		// 1: Write	BBP, 0:	Read BBP
        ULONG		Busy:1;				// 1: ASIC is busy execute BBP programming.	
        ULONG		RegNum:7;			// Selected	BBP	register
		ULONG		Value:8;			// Register	value to program into BBP
#else
        ULONG       Value:8;            // Register value to program into BBP
        ULONG       RegNum:7;           // Selected BBP register
        ULONG       Busy:1;             // 1: ASIC is busy execute BBP programming. 
        ULONG       WriteControl:1;     // 1: Write BBP, 0: Read BBP
        ULONG       Rsvd:15;
#endif
    }   field;
    ULONG           word;
}   BBPCSR_STRUC, *PBBPCSR_STRUC;

//
// RFCSR: RF serial control register
//
typedef union   _RFCSR_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Busy:1;				// 1: ASIC is busy execute RF programming.
        ULONG		PLL_LD:1;			// RF PLL_LD status
        ULONG		IFSelect:1;			// 1: select IF	to program,	0: select RF to	program
        ULONG		NumberOfBits:5;		// Number of bits used in RFRegValue (I:20,	RFMD:22)
		ULONG		RFRegValue:24;		// Register	value (include register	id)	serial out to RF/IF	chip.
#else
        ULONG       RFRegValue:24;      // Register value (include register id) serial out to RF/IF chip.
        ULONG       NumberOfBits:5;     // Number of bits used in RFRegValue (I:20, RFMD:22)
        ULONG       IFSelect:1;         // 1: select IF to program, 0: select RF to program
        ULONG       PLL_LD:1;           // RF PLL_LD status
        ULONG       Busy:1;             // 1: ASIC is busy execute RF programming.
#endif
    }   field;
    ULONG           word;
}   RFCSR_STRUC, *PRFCSR_STRUC;

//
// LEDCSR: LED control register
//
typedef union   _LEDCSR_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        ULONG       Rsvd:11;
        ULONG       LedADefault:1;      // LED A default value for "enable" state. 0: LED ON, 1: LED OFF
        ULONG       LedBPolarity:1;     // 0: active low, 1: active high
        ULONG       LedAPolarity:1;     // 0: active low, 1: active high
        ULONG       LedB:1;             // controlled by software, 1: ON, 0: OFF
        ULONG       LedA:1;             // indicate TX activity, 1: enable, 0: disable
        ULONG		OffPeriod:8;		// Off period, default 30ms
		ULONG		OnPeriod:8;			// On period, default 70ms
#else
        ULONG       OnPeriod:8;         // On period, default 70ms
        ULONG       OffPeriod:8;        // Off period, default 30ms
        ULONG       LedA:1;             // indicate TX activity, 1: enable, 0: disable
        ULONG       LedB:1;             // controlled by software, 1: ON, 0: OFF
        ULONG       LedAPolarity:1;     // 0: active low, 1: active high
        ULONG       LedBPolarity:1;     // 0: active low, 1: active high
        ULONG       LedADefault:1;      // LED A default value for "enable" state. 0: LED ON, 1: LED OFF
        ULONG       Rsvd:11;
#endif
    }   field;
    ULONG           word;
}   LEDCSR_STRUC, *PLEDCSR_STRUC;

//
// GPIOCSR: GPIO control register
//
typedef union   _GPIOCSR_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG		Rsvd:16;
        ULONG       Dir7:1;
        ULONG       Dir6:1;
        ULONG       Dir5:1;
        ULONG       Dir4:1;
        ULONG       Dir3:1;
        ULONG       Dir2:1;
        ULONG       Dir1:1;
        ULONG       Dir0:1;
        ULONG		Bit7:1;
        ULONG		Bit6:1;
        ULONG		Bit5:1;
        ULONG		Bit4:1;
        ULONG		Bit3:1;
        ULONG		Bit2:1;
        ULONG		Bit1:1;
		ULONG		Bit0:1;
#else
        ULONG       Bit0:1;
        ULONG       Bit1:1;
        ULONG       Bit2:1;
        ULONG       Bit3:1;
        ULONG       Bit4:1;
        ULONG       Bit5:1;
        ULONG       Bit6:1;
        ULONG       Bit7:1;
        ULONG       Dir0:1;
        ULONG       Dir1:1;
        ULONG       Dir2:1;
        ULONG       Dir3:1;
        ULONG       Dir4:1;
        ULONG       Dir5:1;
        ULONG       Dir6:1;
        ULONG       Dir7:1;
        ULONG       Rsvd:16;
#endif
    }   field;
    ULONG           word;
}   GPIOCSR_STRUC, *PGPIOCSR_STRUC;

//
// BCNCSR1: Tx BEACON offset time control register
//
typedef union   _BCNCSR1_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG  	    Rsvd:12;
        ULONG       BeaconCwMin:4;   // 2^CwMin
		ULONG       Preload:16;      // in units of usec
#else
        ULONG       Preload:16;      // in units of usec
        ULONG       BeaconCwMin:4;   // 2^CwMin
        ULONG       Rsvd:12;
#endif
    }   field;
    ULONG           word;
}   BCNCSR1_STRUC, *PBCNCSR1_STRUC;

//
// MACCSR2: TX_PE to RX_PE turn-around time control register
//
typedef union   _MACCSR2_STRUC  {
    struct  {
#ifdef BIG_ENDIAN
        ULONG       Rsvd:24;
		ULONG       Delay:8;    // in units of PCI clock cycle
#else
        ULONG       Delay:8;    // in units of PCI clock cycle
        ULONG       Rsvd:24;
#endif
    }   field;
    ULONG           word;
}   MACCSR2_STRUC, *PMACCSR2_STRUC;

//
// EEPROM antenna select format
//
typedef union   _EEPROM_ANTENNA_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        USHORT      RfType:5;               // see E2PROM document for RF IC selection
        USHORT		HardwareRadioControl:1;	// 1: Hardware controlled radio enabled, Read GPIO0 required.
        USHORT      DynamicTxAgcControl:1;
        USHORT      LedMode:3;              // 0-default mode, 1:TX/RX activity mode, 2: Single LED (didn't care about link), 3: reserved
        USHORT		RxDefaultAntenna:2;		// default of antenna, 0: diversity, 1:antenna-A, 2:antenna-B reserved (default = 0)
        USHORT		TxDefaultAntenna:2;		// default of antenna, 0: diversity, 1:antenna-A, 2:antenna-B reserved (default = 0)
	USHORT		NumOfAntenna:2;			// Number of antenna
#else
        USHORT      NumOfAntenna:2;         // Number of antenna
        USHORT      TxDefaultAntenna:2;     // default of antenna, 0: diversity, 1:antenna-A, 2:antenna-B reserved (default = 0)
        USHORT      RxDefaultAntenna:2;     // default of antenna, 0: diversity, 1:antenna-A, 2:antenna-B reserved (default = 0)
        USHORT      LedMode:3;              // 0-default mode, 1:TX/RX activity mode, 2: Single LED (didn't care about link), 3: reserved
        USHORT      DynamicTxAgcControl:1;
        USHORT      HardwareRadioControl:1; // 1: Hardware controlled radio enabled, Read GPIO0 required.
	USHORT      RfType:5;               // see E2PROM document for RF IC selection
#endif
    }   field;
    USHORT          word;
}   EEPROM_ANTENNA_STRUC, *PEEPROM_ANTENNA_STRUC;

typedef	union	_EEPROM_NIC_CINFIG2_STRUC	{
	struct	{
#ifdef BIG_ENDIAN
        USHORT      Rsv:12;                 // must be 0
        USHORT	    CckTxPower:2;			// CCK TX power compensation
        USHORT		DynamicBbpTuning:1;		// !!! NOTE: 0 - enable, 1 - disable
	USHORT		CardbusAcceleration:1;	// !!! NOTE: 0 - enable, 1 - disable
#else
		USHORT		CardbusAcceleration:1;	// !!! NOTE: 0 - enable, 1 - disable
		USHORT		DynamicBbpTuning:1;		// !!! NOTE: 0 - enable, 1 - disable
		USHORT		CckTxPower:2;			// CCK TX power compensation
		USHORT      Rsv:12;                 // must be 0
#endif
	}	field;
	USHORT			word;
}	EEPROM_NIC_CONFIG2_STRUC, *PEEPROM_NIC_CONFIG2_STRUC;

typedef union   _EEPROM_TX_PWR_STRUC    {
    struct  {
#ifdef BIG_ENDIAN
        UCHAR	Byte1;				// High Byte
		UCHAR	Byte0;				// Low Byte
#else
        UCHAR   Byte0;              // Low Byte
        UCHAR   Byte1;              // High Byte
#endif
    }   field;
    USHORT  word;
}   EEPROM_TX_PWR_STRUC, *PEEPROM_TX_PWR_STRUC;

typedef union   _EEPROM_VERSION_STRUC   {
    struct  {
#ifdef BIG_ENDIAN
        UCHAR	Version;			// High Byte
		UCHAR	FaeReleaseNumber;	// Low Byte
#else
        UCHAR   FaeReleaseNumber;   // Low Byte
        UCHAR   Version;            // High Byte
#endif
    }   field;
    USHORT  word;
}   EEPROM_VERSION_STRUC, *PEEPROM_VERSION_STRUC;

#endif  // __RT2560_H__
