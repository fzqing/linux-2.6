#ifndef _CPMAC_LX_DDA_H_
#define _CPMAC_LX_DDA_H_
	
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <asm/mips-boards/prom.h>
#include <asm/mach-avalanche/pal.h>
	
#define CPMAC_DDC
typedef struct _CpmacDDCObj_t CpmacDDCObj;
#include <asm/mach-avalanche/ioctl_api.h>
	
#define CSL_IOCTL_SHIFT		(0)
#define CSL_IOCTL_MASK		(0xFF)
#define CSL_IOCTL(ioctl)	((ioctl & CSL_IOCTL_MASK) >> CSL_IOCTL_SHIFT)
	
#define CPMAC_CACHE_WRITEBACK_MODE
#define CPMAC_DDC_MIPS_OPTIMIZED
#define CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
#define CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
#define CPMAC_USE_ENV
	
#define CSL_FMK(PER_REG_FIELD, val) \
	(((val) << CSL_##PER_REG_FIELD##_SHIFT) & CSL_##PER_REG_FIELD##_MASK)
	
#define CSL_FMKT(PER_REG_FIELD, TOKEN) \
	CSL_FMK(PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)
	
#define CSL_FMKR(msb, lsb, val) \
	(((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))
	
#define CSL_FEXT(reg, PER_REG_FIELD) \
	(((reg) & CSL_##PER_REG_FIELD##_MASK) >> CSL_##PER_REG_FIELD##_SHIFT)
	
#define CSL_FEXTR(reg, msb, lsb) \
	(((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))
	
#define CSL_FINS(reg, PER_REG_FIELD, val) \
	((reg) = ((reg) & ~CSL_##PER_REG_FIELD##_MASK)\
	| CSL_FMK(PER_REG_FIELD, val))
	
#define CSL_FINST(reg, PER_REG_FIELD, TOKEN)\
	CSL_FINS((reg), PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)
	
#define CSL_FINSR(reg, msb, lsb, val)\
	((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))\
	| CSL_FMKR(msb, lsb, val))
	
#define CSL_SOK			(1)     /* Success */
#define CSL_ESYS_FAIL		(-1)    /* Generic failure */
#define CSL_ESYS_INUSE 		(-2)    /* Peripheral resource is already in use */
#define CSL_ESYS_XIO		(-3)    /* Encountered a shared I/O(XIO) pin conflict */
#define CSL_ESYS_OVFL		(-4)    /* Encoutered CSL system resource overflow */
#define CSL_ESYS_BADHANDLE 	(-5)    /* Handle passed to CSL was invalid */
#define CSL_ESYS_INVPARAMS	(-6)    /* invalid parameters */
#define CSL_ESYS_INVCMD		(-7)    /* invalid command */
#define CSL_ESYS_INVQUERY	(-8)    /* invalid query */
#define CSL_ESYS_NOTSUPPORTED 	(-9)    /* action not supported */
#define CSL_ESYS_LAST		(-32)   /* Sentinel error code, end of sys range */
	
#if (AVALANCHE_CPMAC_HW_MODULE_REV < 0x000c0200)
	
typedef struct  
{
	CSL_Reg32   Tx_IdVer;
	CSL_Reg32   Tx_Control;
	CSL_Reg32   Tx_Teardown;
	CSL_Reg32   Reserved1;
	CSL_Reg32   Rx_IdVer;
	CSL_Reg32   Rx_Control;
	CSL_Reg32   Rx_Teardown;
	CSL_Reg32   Reserved2[57];      /* Padding for holes in memory map */
	CSL_Reg32   Rx_MBP_Enable;
	CSL_Reg32   Rx_Unicast_Set;
	CSL_Reg32   Rx_Unicast_Clear;
	CSL_Reg32   Rx_Maxlen;
	CSL_Reg32   Rx_Buffer_Offset;
	CSL_Reg32   Rx_FilterLowThresh;
	CSL_Reg32   Reserved3[2];       
	CSL_Reg32   Rx_FlowThresh[8];   
	CSL_Reg32   Rx_FreeBuffer[8];   
	CSL_Reg32   MacControl;
	CSL_Reg32   MacStatus;
	CSL_Reg32   EMControl;
	CSL_Reg32   TxFifoControl;      /* CP(G)MAC added register */
	CSL_Reg32   Tx_IntStat_Raw;
	CSL_Reg32   Tx_IntStat_Masked;
	CSL_Reg32   Tx_IntMask_Set;
	CSL_Reg32   Tx_IntMask_Clear;
	CSL_Reg32   Mac_In_Vector;
	CSL_Reg32   Mac_EOI_Vector;
	CSL_Reg32   Mac_Cfig;           /* CP(G)MAC added register */
	CSL_Reg32   Reserved5;          /* Padding for holes in memory map */
	CSL_Reg32   Rx_IntStat_Raw;
	CSL_Reg32   Rx_IntStat_Masked;
	CSL_Reg32   Rx_IntMask_Set;
	CSL_Reg32   Rx_IntMask_Clear;
	CSL_Reg32   Mac_IntStat_Raw;
	CSL_Reg32   Mac_IntStat_Masked;
	CSL_Reg32   Mac_IntMask_Set;
	CSL_Reg32   Mac_IntMask_Clear;
	CSL_Reg32   Reserved51[8];      /* CP(G)MAC modified register memory map */
	CSL_Reg32   MacSrcAddr_Lo;      /* CP(G)MAC modified register */
	CSL_Reg32   MacSrcAddr_Hi;      /* CP(G)MAC modified register */
	CSL_Reg32   MacHash1;
	CSL_Reg32   MacHash2;
	CSL_Reg32   BoffTest;
	CSL_Reg32   Tpace_Test;         /* CP(G)MAC modified register */
	CSL_Reg32   Rx_Pause;
	CSL_Reg32   Tx_Pause;
	CSL_Reg32   Reserved6[4];       /* Padding for holes in memory map */
	CSL_Reg32   RxGoodFrames;
	CSL_Reg32   RxBroadcastFrames;
	CSL_Reg32   RxMulticastFrames;
	CSL_Reg32   RxPauseFrames;
	CSL_Reg32   RxCRCErrors;
	CSL_Reg32   RxAlignCodeErrors;
	CSL_Reg32   RxOversizedFrames;
	CSL_Reg32   RxJabberFrames;
	CSL_Reg32   RxUndersizedFrames;
	CSL_Reg32   RxFragments;
	CSL_Reg32   RxFilteredFrames;
	CSL_Reg32   RxQosFilteredFrames;
	CSL_Reg32   RxOctets;
	CSL_Reg32   TxGoodFrames;
	CSL_Reg32   TxBroadcastFrames;
	CSL_Reg32   TxMulticastFrames;
	CSL_Reg32   TxPauseFrames;
	CSL_Reg32   TxDeferredFrames;
	CSL_Reg32   TxCollisionFrames;
	CSL_Reg32   TxSingleCollFrames;
	CSL_Reg32   TxMultCollFrames;
	CSL_Reg32   TxExcessiveCollisions;
	CSL_Reg32   TxLateCollisions;
	CSL_Reg32   TxUnderrun;
	CSL_Reg32   TxCarrierSenseErrors;
	CSL_Reg32   TxOctets;
	CSL_Reg32   Reg64octetFrames;
	CSL_Reg32   Reg65t127octetFrames;
	CSL_Reg32   Reg128t255octetFrames;
	CSL_Reg32   Reg256t511octetFrames;
	CSL_Reg32   Reg512t1023octetFrames;
	CSL_Reg32   Reg1024tUPoctetFrames;
	CSL_Reg32   NetOctets;
	CSL_Reg32   RxSofOverruns;
	CSL_Reg32   RxMofOverruns;
	CSL_Reg32   RxDmaOverruns;
	CSL_Reg32   Reserved7[28];      /* CP(G)MAC modified register */
	CSL_Reg32   RX_FIFO_Processor_TestAccess[64];
	CSL_Reg32   TX_FIFO_Processor_TestAccess[64];
	CSL_Reg32   MacAddr_Lo;         /* CP(G)MAC added register */
	CSL_Reg32   MacAddr_Hi;         /* CP(G)MAC added register */
	CSL_Reg32   MacIndex;           /* CP(G)MAC added register */
	CSL_Reg32   Reserved8[61];      /* CP(G)MAC modified register */
	CSL_Reg32   Tx_HDP[8];          /* TX HDP registers 0-7 combined into an array */
	CSL_Reg32   Rx_HDP[8];          /* RX HDP registers 0-7 combined into an array */
	CSL_Reg32   Tx_CP[8];          
	CSL_Reg32   Rx_CP[8];           
	CSL_Reg32   Reserved9[64];      /* Padding for holes in memory map */
	CSL_Reg32   Stateram_Test_Access[32];
	CSL_Reg32   Reserved10[32];     /* Padding for holes in memory map */
} CSL_CpmacRegs;
#else
typedef struct  
{
	CSL_Reg32   Tx_IdVer;
	CSL_Reg32   Tx_Control;
	CSL_Reg32   Tx_Teardown;
	CSL_Reg32   Reserved1;
	CSL_Reg32   Rx_IdVer;
	CSL_Reg32   Rx_Control;
	CSL_Reg32   Rx_Teardown;
	CSL_Reg32   Reserved2[25];      /* Padding for holes in memory map */
	CSL_Reg32   Tx_IntStat_Raw;
	CSL_Reg32   Tx_IntStat_Masked;
	CSL_Reg32   Tx_IntMask_Set;
	CSL_Reg32   Tx_IntMask_Clear;
	CSL_Reg32   Mac_In_Vector;
	CSL_Reg32   Mac_EOI_Vector;
	CSL_Reg32   Reserved21[2];
	CSL_Reg32   Rx_IntStat_Raw;
	CSL_Reg32   Rx_IntStat_Masked;
	CSL_Reg32   Rx_IntMask_Set;
	CSL_Reg32   Rx_IntMask_Clear;
	CSL_Reg32   Mac_IntStat_Raw;
	CSL_Reg32   Mac_IntStat_Masked;
	CSL_Reg32   Mac_IntMask_Set;
	CSL_Reg32   Mac_IntMask_Clear;
	CSL_Reg32   Reserved22[16];
	CSL_Reg32   Rx_MBP_Enable;
	CSL_Reg32   Rx_Unicast_Set;
	CSL_Reg32   Rx_Unicast_Clear;
	CSL_Reg32   Rx_Maxlen;
	CSL_Reg32   Rx_Buffer_Offset;
	CSL_Reg32   Rx_FilterLowThresh;
	CSL_Reg32   Reserved3[2];    
	CSL_Reg32   Rx_FlowThresh[8];   
	CSL_Reg32   Rx_FreeBuffer[8];  
	CSL_Reg32   MacControl;
	CSL_Reg32   MacStatus;
	CSL_Reg32   EMControl;
	CSL_Reg32   TxFifoControl;      /* CP(G)MAC added register */
	CSL_Reg32   Mac_Cfig;           /* CP(G)MAC added register */
	CSL_Reg32   Soft_Reset;         /* CP(G)MAC 2.6 added register */
	CSL_Reg32   Reserved51[22];      /* CP(G)MAC modified register memory map */
	CSL_Reg32   MacSrcAddr_Lo;      /* CP(G)MAC modified register */
	CSL_Reg32   MacSrcAddr_Hi;      /* CP(G)MAC modified register */
	CSL_Reg32   MacHash1;
	CSL_Reg32   MacHash2;
	CSL_Reg32   BoffTest;
	CSL_Reg32   Tpace_Test;         /* CP(G)MAC modified register */
	CSL_Reg32   Rx_Pause;
	CSL_Reg32   Tx_Pause;
	CSL_Reg32   Reserved6[4];       /* Padding for holes in memory map */
	CSL_Reg32   RxGoodFrames;
	CSL_Reg32   RxBroadcastFrames;
	CSL_Reg32   RxMulticastFrames;
	CSL_Reg32   RxPauseFrames;
	CSL_Reg32   RxCRCErrors;
	CSL_Reg32   RxAlignCodeErrors;
	CSL_Reg32   RxOversizedFrames;
	CSL_Reg32   RxJabberFrames;
	CSL_Reg32   RxUndersizedFrames;
	CSL_Reg32   RxFragments;
	CSL_Reg32   RxFilteredFrames;
	CSL_Reg32   RxQosFilteredFrames;
	CSL_Reg32   RxOctets;
	CSL_Reg32   TxGoodFrames;
	CSL_Reg32   TxBroadcastFrames;
	CSL_Reg32   TxMulticastFrames;
	CSL_Reg32   TxPauseFrames;
	CSL_Reg32   TxDeferredFrames;
	CSL_Reg32   TxCollisionFrames;
	CSL_Reg32   TxSingleCollFrames;
	CSL_Reg32   TxMultCollFrames;
	CSL_Reg32   TxExcessiveCollisions;
	CSL_Reg32   TxLateCollisions;
	CSL_Reg32   TxUnderrun;
	CSL_Reg32   TxCarrierSenseErrors;
	CSL_Reg32   TxOctets;
	CSL_Reg32   Reg64octetFrames;
	CSL_Reg32   Reg65t127octetFrames;
	CSL_Reg32   Reg128t255octetFrames;
	CSL_Reg32   Reg256t511octetFrames;
	CSL_Reg32   Reg512t1023octetFrames;
	CSL_Reg32   Reg1024tUPoctetFrames;
	CSL_Reg32   NetOctets;
	CSL_Reg32   RxSofOverruns;
	CSL_Reg32   RxMofOverruns;
	CSL_Reg32   RxDmaOverruns;
	CSL_Reg32   Reserved7[28];      /* CP(G)MAC modified register */
	CSL_Reg32   RX_FIFO_Processor_TestAccess[64];
	CSL_Reg32   TX_FIFO_Processor_TestAccess[64];
	CSL_Reg32   MacAddr_Lo;         /* CP(G)MAC added register */
	CSL_Reg32   MacAddr_Hi;         /* CP(G)MAC added register */
	CSL_Reg32   MacIndex;           /* CP(G)MAC added register */
	CSL_Reg32   Reserved8[61];      /* CP(G)MAC modified register */
	CSL_Reg32   Tx_HDP[8];          /* TX HDP registers 0-7 combined into an array */
	CSL_Reg32   Rx_HDP[8];          /* RX HDP registers 0-7 combined into an array */
	CSL_Reg32   Tx_CP[8];         
	CSL_Reg32   Rx_CP[8];           
	CSL_Reg32   Reserved9[32];      /* Padding for holes in memory map */
	CSL_Reg32   Stateram_Test_Access[32];
	CSL_Reg32   Reserved10[32];     /* Padding for holes in memory map */
} CSL_CpmacRegs;
	
#endif
	
typedef volatile CSL_CpmacRegs *CSL_CpmacRegsOvly;
	
typedef enum  
{
	Tx_IdVer = 0,
	Tx_Control,
	Tx_Teardown,
	Rx_IdVer = 4,
	Rx_Control,
	Rx_Teardown,
	Rx_MBP_Enable = 64,
	Rx_Unicast_Set,
	Rx_Unicast_Clear,
	Rx_Maxlen,
	Rx_Buffer_Offset,
	Rx_FilterLowThresh,
	Rx0_FlowThresh = 72,
	Rx1_FlowThresh,
	Rx2_FlowThresh,
	Rx3_FlowThresh,
	Rx4_FlowThresh,
	Rx5_FlowThresh,
	Rx6_FlowThresh,
	Rx7_FlowThresh,
	Rx0_FreeBuffer,
	Rx1_FreeBuffer,
	Rx2_FreeBuffer,
	Rx3_FreeBuffer,
	Rx4_FreeBuffer,
	Rx5_FreeBuffer,
	Rx6_FreeBuffer,
	Rx7_FreeBuffer,
	MacControl,
	MacStatus,
	EMControl,
	TxFifoControl,
	Tx_IntStat_Raw,
	Tx_IntStat_Masked,
	Tx_IntMask_Set,
	Tx_IntMask_Clear,
	Mac_In_Vector,
	Mac_EOI_Vector,
	Mac_Cfig,
	Rx_IntStat_Raw = 100,
	Rx_IntStat_Masked,
	Rx_IntMask_Set,
	Rx_IntMask_Clear,
	Mac_IntStat_Raw,
	Mac_IntStat_Masked,
	Mac_IntMask_Set,
	Mac_IntMask_Clear,
	MacSrcAddr_Lo = 116,
	MacSrcAddr_Hi,
	MacHash1,
	MacHash2,
	BoffTest,
	Tpace_Test,
	RxPause,
	TxPause,
	RxGoodFrames = 128,
	RxBroadcastFrames,
	RxMulticastFrames,
	RxPauseFrames,
	RxCRCErrors,
	RxAlignCodeErrors,
	RxOversizedFrames,
	RxJabberFrames,
	RxUndersizedFrames,
	RxFragments,
	RxFilteredFrames,
	RxQosFilteredFrames,
	RxOctets,
	TxGoodFrames,
	TxBroadcastFrames,
	TxMulticastFrames,
	TxPauseFrames,
	TxDeferredFrames,
	TxCollisionFrames,
	TxSingleCollFrames,
	TxMultCollFrames,
	TxExcessiveCollisions,
	TxLateCollisions,
	TxUnderrun,
	TxCarrierSenseErrors,
	TxOctets,
	Reg64octetFrames,
	Reg65t127octetFrames,
	Reg128t255octetFrames,
	Reg256t511octetFrames,
	Reg512t1023octetFrames,
	Reg1024tUPoctetFrames,
	NetOctets,
	RxSofOverruns,
	RxMofOverruns,
	RxDmaOverruns,
	RX_FIFO_Processor_TestAccess = 192,     /* First word of RX FIFO */
	TX_FIFO_Processor_TestAccess = 256,     /* First word of TX FIFO */
	MacAddr_Lo = 320,
	MacAddr_Hi,
	MacIndex,
	Tx0_HDP = 384,
	Tx1_HDP,
	Tx2_HDP,
	Tx3_HDP,
	Tx4_HDP,
	Tx5_HDP,
	Tx6_HDP,
	Tx7_HDP,
	Rx0_HDP,
	Rx1_HDP,
	Rx2_HDP,
	Rx3_HDP,
	Rx4_HDP,
	Rx5_HDP,
	Rx6_HDP,
	Rx7_HDP,
	Tx0_CP,
	Tx1_CP,
	Tx2_CP,
	Tx3_CP,
	Tx4_CP,
	Tx5_CP,
	Tx6_CP,
	Tx7_CP,
	Rx0_CP,
	Rx1_CP,
	Rx2_CP,
	Rx3_CP,
	Rx4_CP,
	Rx5_CP,
	Rx6_CP,
	Rx7_CP,
	Stateram_Test_Access = 448     /* First word of State RAM */
} CSL_CpmacRegIds;
	
typedef enum
{   
	RX_ADDR_TYPE0 = 0,      /**< Type 0 addressing - old style used in (CPMAC) */
	RX_ADDR_TYPE1 = 1,      /**< Type 1 addressing - new CPGMAC style */
	RX_ADDR_TYPE2 = 2,      /**< Type 2 addressing - new CPGMAC "filtering" style */
	RX_ADDR_TYPE3 = 3       /**< TODO: Type 3 addressing  - new CPGMAC "filtering" style */
} CpmacRxAddrType;
	
	
#undef REG32_DATA
#define REG32_DATA(addr)                        (*(volatile unsigned int *)(addr))
#define CPMAC_MACADDRLO(baseAddr, chNum)        REG32_DATA((baseAddr) + 0x1B0 + ((chNum) * 4))
#define CPMAC_MACADDRMID(baseAddr)              REG32_DATA((baseAddr) + 0x1D0)
#define CPMAC_MACADDRHI(baseAddr)               REG32_DATA((baseAddr) + 0x1D4)
	
#define CPMAC_NUM_STAT_REGS                     36
#define CPMAC_STAT_CLEAR                        0xFFFFFFFF
#define CPMAC_ALL_MULTI_REG_VALUE               0xFFFFFFFF
#define CPMAC_NUM_MULTICAST_BITS                64
#define CPMAC_TEARDOWN_VALUE                    0xFFFFFFFC
#define CPMAC_TX_CONTROL_TX_ENABLE_VAL          0x1
#define CPMAC_RX_CONTROL_RX_ENABLE_VAL          0x1
#define CPMAC_MAC_HOST_ERR_INTMASK_VAL          0x2
#define CPMAC_MAC_STAT_INT_INTMASK_VAL          0x1
#define CPMAC_RX_UNICAST_CLEAR_ALL              0xFF
	
#define CSL_CPMAC_TYPE_0_MACSRCADDR0_MASK                    (0xFF)
#define CSL_CPMAC_TYPE_0_MACSRCADDR0_SHIFT                   0
#define CSL_CPMAC_TYPE_0_MACSRCADDR1_MASK                    (0xFF)
#define CSL_CPMAC_TYPE_0_MACSRCADDR1_SHIFT                   0
#define CSL_CPMAC_TYPE_0_MACSRCADDR2_MASK                    (0xFF<<24)
#define CSL_CPMAC_TYPE_0_MACSRCADDR2_SHIFT                   24
#define CSL_CPMAC_TYPE_0_MACSRCADDR3_MASK                    (0xFF<<16)
#define CSL_CPMAC_TYPE_0_MACSRCADDR3_SHIFT                   16
#define CSL_CPMAC_TYPE_0_MACSRCADDR4_MASK                    (0xFF<<8)
#define CSL_CPMAC_TYPE_0_MACSRCADDR4_SHIFT                   8
#define CSL_CPMAC_TYPE_0_MACSRCADDR5_MASK                    (0xFF)
#define CSL_CPMAC_TYPE_0_MACSRCADDR5_SHIFT                   0
	
#define CSL_CPMAC_TYPE_1_MACSRCADDR0_MASK                    (0xFF<<8)
#define CSL_CPMAC_TYPE_1_MACSRCADDR0_SHIFT                   8
#define CSL_CPMAC_TYPE_1_MACSRCADDR1_MASK                    (0xFF)
#define CSL_CPMAC_TYPE_1_MACSRCADDR1_SHIFT                   0
	
#define CSL_CPMAC_TYPE_1_MACSRCADDR2_MASK                    (0xFF<<24)
#define CSL_CPMAC_TYPE_1_MACSRCADDR2_SHIFT                   24
#define CSL_CPMAC_TYPE_1_MACSRCADDR3_MASK                    (0xFF<<16)
#define CSL_CPMAC_TYPE_1_MACSRCADDR3_SHIFT                   16
#define CSL_CPMAC_TYPE_1_MACSRCADDR4_MASK                    (0xFF<<8)
#define CSL_CPMAC_TYPE_1_MACSRCADDR4_SHIFT                   8
#define CSL_CPMAC_TYPE_1_MACSRCADDR5_MASK                    (0xFF)
#define CSL_CPMAC_TYPE_1_MACSRCADDR5_SHIFT                   0
	
#define CSL_CPGMAC_VALID_MASK                                   (0x1<<20)
#define CSL_CPGMAC_VALID_SHIFT                                  20
#define CSL_CPGMAC_MATCH_FILTER_MASK                            (0x1<<19)
#define CSL_CPGMAC_MATCH_FILTER_SHIFT                           19
#define CSL_CPGMAC_CHANNEL_MASK                                 (0x7<<16)
#define CSL_CPGMAC_CHANNEL_SHIFT                                16
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR0_MASK                    (0xFF<<8)
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR0_SHIFT                   8
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR1_MASK                    (0xFF)
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR1_SHIFT                   0
	
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR2_MASK                    (0xFF<<24)
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR2_SHIFT                   24
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR3_MASK                    (0xFF<<16)
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR3_SHIFT                   16
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR4_MASK                    (0xFF<<8)
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR4_SHIFT                   8
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR5_MASK                    (0xFF)
#define CSL_CPGMAC_TYPE_2_3_MACSRCADDR5_SHIFT                   0

#define CSL_CPMAC_RXMBP_PASSCRC_SHIFT               30
#define CSL_CPMAC_RXMBP_PASSCRC_MASK                (0x1 << 30)
#define CSL_CPMAC_RXMBP_QOSEN_SHIFT                 29
#define CSL_CPMAC_RXMBP_QOSEN_MASK                  (0x1 << 29)
#define CSL_CPMAC_RXMBP_NOCHAIN_SHIFT               28
#define CSL_CPMAC_RXMBP_NOCHAIN_MASK                (0x1 << 28)
#define CSL_CPMAC_RXMBP_CMFEN_SHIFT                 24
#define CSL_CPMAC_RXMBP_CMFEN_MASK                  (0x1 << 24)
#define CSL_CPMAC_RXMBP_CSFEN_SHIFT                 23
#define CSL_CPMAC_RXMBP_CSFEN_MASK                  (0x1 << 23)
#define CSL_CPMAC_RXMBP_CEFEN_SHIFT                 22
#define CSL_CPMAC_RXMBP_CEFEN_MASK                  (0x1 << 22)
#define CSL_CPMAC_RXMBP_CAFEN_SHIFT                 21
#define CSL_CPMAC_RXMBP_CAFEN_MASK                  (0x1 << 21)
#define CSL_CPMAC_RXMBP_PROMCH_SHIFT                16
#define CSL_CPMAC_RXMBP_PROMCH_MASK                 (0x7 << 16)
#define CSL_CPMAC_RXMBP_BROADEN_SHIFT               13
#define CSL_CPMAC_RXMBP_BROADEN_MASK                (0x1 << 13)
#define CSL_CPMAC_RXMBP_BROADCH_SHIFT               8
#define CSL_CPMAC_RXMBP_BROADCH_MASK                (0x7 << 8)
#define CSL_CPMAC_RXMBP_MULTIEN_SHIFT               5
#define CSL_CPMAC_RXMBP_MULTIEN_MASK                (0x1 << 5)
#define CSL_CPMAC_RXMBP_MULTICH_SHIFT               0
#define CSL_CPMAC_RXMBP_MULTICH_MASK                0x7
	
#ifdef CONFIG_CPGMAC_2_6
	
#define CSL_CPMAC_MACCONTROL_GIGFORCE_SHIFT     17 
#define CSL_CPMAC_MACCONTROL_GIGFORCE_MASK      (0x1 << 17)
#define CSL_CPMAC_MACCONTROL_GPIOB_SHIFT        16 
#define CSL_CPMAC_MACCONTROL_GPIOB_MASK         (0x1 << 16)
#define CSL_CPMAC_MACCONTROL_GPIOA_SHIFT        15 
#define CSL_CPMAC_MACCONTROL_GPIOA_MASK         (0x1 << 15)
#define CSL_CPMAC_MACCONTROL_RXOFFLENBLK_SHIFT  14
#define CSL_CPMAC_MACCONTROL_RXOFFLENBLK_MASK   (0x1 << 14)
#define CSL_CPMAC_MACCONTROL_RXOWNERSHIP_SHIFT  13 
#define CSL_CPMAC_MACCONTROL_RXOWNERSHIP_MASK   (0x1 << 13)
#define CSL_CPMAC_MACCONTROL_RXFIFOFLOWEN_SHIFT 12
#define CSL_CPMAC_MACCONTROL_RXFIFOFLOWEN_MASK  (0x1 << 12)
#define CSL_CPMAC_MACCONTROL_CMDIDLE_SHIFT      11 
#define CSL_CPMAC_MACCONTROL_CMDIDLE_MASK       (0x1 << 11)
	
#endif
	
#define CSL_CPMAC_MACCONTROL_TXSHORTGAPEN_SHIFT     10
#define CSL_CPMAC_MACCONTROL_TXSHORTGAPEN_MASK      (0x1 << 10)
#define CSL_CPMAC_MACCONTROL_TXPTYPE_SHIFT          9
#define CSL_CPMAC_MACCONTROL_TXPTYPE_MASK           (0x1 << 9)
#define CSL_CPMAC_MACCONTROL_GIGABITEN_SHIFT        7
#define CSL_CPMAC_MACCONTROL_GIGABITEN_MASK         (0x1 << 7)
#define CSL_CPMAC_MACCONTROL_TXPACEEN_SHIFT         6
#define CSL_CPMAC_MACCONTROL_TXPACEEN_MASK          (0x1 << 6)
#define CSL_CPMAC_MACCONTROL_MIIEN_SHIFT            5
#define CSL_CPMAC_MACCONTROL_MIIEN_MASK             (0x1 << 5)
#define CSL_CPMAC_MACCONTROL_TXFLOWEN_SHIFT         4
#define CSL_CPMAC_MACCONTROL_TXFLOWEN_MASK          (0x1 << 4)
#define CSL_CPMAC_MACCONTROL_RXFLOWEN_SHIFT         3
#define CSL_CPMAC_MACCONTROL_RXFLOWEN_MASK          (0x1 << 3)
#define CSL_CPMAC_MACCONTROL_LOOPBKEN_SHIFT         1
#define CSL_CPMAC_MACCONTROL_LOOPBKEN_MASK          (0x1 << 1)
#define CSL_CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT     0
#define CSL_CPMAC_MACCONTROL_FULLDUPLEXEN_MASK      (0x1)
	
#define CSL_CPMAC_MACSTATUS_TXERRCODE_MASK          0xF00000
#define CSL_CPMAC_MACSTATUS_TXERRCODE_SHIFT         20
#define CSL_CPMAC_MACSTATUS_TXERRCH_MASK            0x7
#define CSL_CPMAC_MACSTATUS_TXERRCH_SHIFT           16
#define CSL_CPMAC_MACSTATUS_RXERRCODE_MASK          0xF000
#define CSL_CPMAC_MACSTATUS_RXERRCODE_SHIFT         12
#define CSL_CPMAC_MACSTATUS_RXERRCH_MASK            0x7
#define CSL_CPMAC_MACSTATUS_RXERRCH_SHIFT           8
	
#define CSL_CPMAC_RX_MAX_LEN_SHIFT              0
#define CSL_CPMAC_RX_MAX_LEN_MASK               0xFFFF
	
#define CSL_CPMAC_RX_BUFFER_OFFSET_SHIFT        0
#define CSL_CPMAC_RX_BUFFER_OFFSET_MASK         0xFFFF
	
#define CPMAC_MAC_IN_VECTOR_STATUS_INT          (1 << 19)
#define CPMAC_MAC_IN_VECTOR_HOST_INT            (1 << 18)
#define CPMAC_MAC_IN_VECTOR_RX_INT_OR           (1 << 17)
#define CPMAC_MAC_IN_VECTOR_TX_INT_OR           (1 << 16)
#define CPMAC_MAC_IN_VECTOR_RX_INT_VEC          (7 << 8)
#define CPMAC_MAC_IN_VECTOR_TX_INT_VEC          (7)
	
#define CPMAC_CPPI_SOP_BIT                      0x80000000  /*(1 << 31)*/
#define CPMAC_CPPI_EOP_BIT                      0x40000000  /*(1 << 30*/
#define CPMAC_CPPI_OWNERSHIP_BIT                0x20000000  /*(1 << 29)*/
#define CPMAC_CPPI_EOQ_BIT                      0x10000000  /*(1 << 28)*/
#define CPMAC_CPPI_TEARDOWN_COMPLETE_BIT        0x8000000   /*(1 << 27)*/
#define CPMAC_CPPI_PASS_CRC_BIT                 0x4000000   /*(1 << 26)*/
	
#ifdef CONFIG_CPGMAC_2_6
	
#define CPMAC_SOFT_RESET_BIT                    0x00000001
#define CPMAC_SOFT_RESET_COMPLETE               0x00000000
#endif
	
typedef void* DDA_Handle;
typedef void* DDC_Handle;

#define DDC_IOCTL_SHIFT                 8
#define DDC_IOCTL_MASK                  (0xFF00)
#define DDC_IOCTL_GET(ioctl)            ((ioctl) & DDC_IOCTL_MASK)
#define DDC_IOCTL(ioctl, offset)        ( ((ioctl) + (offset)) << DDC_IOCTL_SHIFT)
#define DDC_IOCTL_MIN                   0
#define DDC_IOCTL_GET_NAME_STRING       DDC_IOCTL(DDC_IOCTL_MIN, 0)
#define DDC_IOCTL_GET_VERSION_ID        DDC_IOCTL(DDC_IOCTL_MIN, 1)
#define DDC_IOCTL_GET_VERSION_STRING    DDC_IOCTL(DDC_IOCTL_MIN, 2)
#define DDC_IOCTL_GET_DRV_STATE         DDC_IOCTL(DDC_IOCTL_MIN, 3)
#define DDC_IOCTL_SET_DRV_POWER_DOWN    DDC_IOCTL(DDC_IOCTL_MIN, 4)
#define DDC_IOCTL_SET_DRV_POWER_UP      DDC_IOCTL(DDC_IOCTL_MIN, 5)
#define DDC_IOCTL_MAX                   31
#define DDC_ERROR                       (0x200001F)
#define DDC_MAJOR_ERROR                 (0xB200001F)
#define DDC_ERROR_MIN                   (DDC_ERROR + 1)
#define DDC_ERROR_MAX                   (DDC_ERROR_MIN + 31)
typedef enum
{
	DDC_CREATED,
	DDC_INITIALIZED,
	DDC_OPENED,
	DDC_CLOSED,
	DDC_DEINITIALIZED,
	DDC_POWERED_DOWN,
} DDC_DriverState;
	
typedef struct
{
	unsigned int              instId;     /**< Instance id/number */
} DDC_InitConfig;
	
typedef struct
{
	unsigned int              versionId;  /**< Version Id */
	unsigned int              instId;     /**< Instance id/number */
	DDC_DriverState     state;      /**< DDC State */
	DDA_Handle          hDDA;       /**< DDA Handle */
} DDC_Obj; 
	
typedef int (*DDC_DeleteInstance) (
	DDC_Handle  hDDC,
	void*         param);
	
	
typedef int (*DDC_Init) (
	DDC_Handle  hDDC,
	void*         param);

typedef int (*DDC_DeInit) (
	DDC_Handle  hDDC,
	void*         param);
	
typedef int (*DDC_Open) (
	DDC_Handle  hDDC,
	void*         param);
	
typedef int (*DDC_Close) (
	DDC_Handle  hDDC,
	void*         param);
	
typedef int (*DDC_Control) (
	DDC_Handle  hDDC,
	int         cmd,
	void*         cmdArg,
	void*         param);
	
typedef struct 
{
	DDC_Init        ddcInit;        /**< DDC Instance Initialization */
	DDC_DeInit      ddcDeinit;      /**< DDC Instance Cleanup/De-init */ 
	DDC_Open        ddcOpen;        /**< Opens DDC Instance */
	DDC_Close       ddcClose;       /**< Closes DDC instance */
	DDC_Control     ddcControl;     /**< DDC IoControl operations */
	DDC_DeleteInstance  ddcDelInst; /**< DDC Delete Instance */
} DDC_FuncTable;
	
typedef DDC_FuncTable *DDC_FuncTableHandle;
	
typedef int (*DDA_ControlCb) (
	DDA_Handle  hDDA,
	int         cmd,
	void*         cmdArg,
	void*         param);
typedef struct 
{
	DDA_ControlCb   ddaControlCb ;      /**< DDA Control Callback */
} DDA_FuncTable;
	
typedef DDA_FuncTable *DDA_FuncTableHandle;
	
typedef int (*DDC_CreateInstance) (
int                 instId,
DDA_Handle          hDDA,
DDA_FuncTableHandle hDDACbIf,
DDC_Handle          *hDDC,
DDC_FuncTableHandle *hDDCIf,
void*                 param);
	
typedef enum
{
	CPMAC_MULTICAST_ADD = 0,    
	CPMAC_MULTICAST_DEL         
} CpmacSingleMultiOper;
	
	
typedef enum
{
	CPMAC_ALL_MULTI_SET = 0,
	CPMAC_ALL_MULTI_CLR
} CpmacAllMultiOper;
	
typedef struct
{
	unsigned int      phyNum;             /**< Phy number to be read/written */
	unsigned int      regAddr;            /**< Register to be read/written */
	unsigned int      data;               /**< Data to be read/written */
} CpmacPhyParams;
	
typedef struct
{
	unsigned int      channel;            
	char*      macAddress;         /**< Mac address  */
} CpmacAddressParams;
	
	
typedef struct
{
	unsigned int      channel;           
	char*      macAddress;         /**< Mac address for filtering */
	int         index;              /**< Index of filtering list to update */
	Bool        valid;              /**< Entry Valid */
	int         match;              /**< Entry Matching  */
} CpmacType2_3_AddrFilterParams;
	
	
typedef struct
{
	unsigned int   ifInGoodFrames;
	unsigned int   ifInBroadcasts;
	unsigned int   ifInMulticasts;
	unsigned int   ifInPauseFrames;
	unsigned int   ifInCRCErrors;
	unsigned int   ifInAlignCodeErrors;
	unsigned int   ifInOversizedFrames;
	unsigned int   ifInJabberFrames;
	unsigned int   ifInUndersizedFrames;
	unsigned int   ifInFragments;
	unsigned int   ifInFilteredFrames;
	unsigned int   ifInQosFilteredFrames;
	unsigned int   ifInOctets;
	unsigned int   ifOutGoodFrames;
	unsigned int   ifOutBroadcasts;
	unsigned int   ifOutMulticasts;
	unsigned int   ifOutPauseFrames;
	unsigned int   ifDeferredTransmissions;
	unsigned int   ifCollisionFrames;
	unsigned int   ifSingleCollisionFrames;
	unsigned int   ifMultipleCollisionFrames;
	unsigned int   ifExcessiveCollisionFrames;
	unsigned int   ifLateCollisions;
	unsigned int   ifOutUnderrun;
	unsigned int   ifCarrierSenseErrors;
	unsigned int   ifOutOctets;
	unsigned int   if64OctetFrames;
	unsigned int   if65To127OctetFrames;
	unsigned int   if128To255OctetFrames;
	unsigned int   if256To511OctetFrames;
	unsigned int   if512To1023OctetFrames;
	unsigned int   if1024ToUPOctetFrames;
	unsigned int   ifNetOctets;
	unsigned int   ifRxSofOverruns;
	unsigned int   ifRxMofOverruns;
	unsigned int   ifRxDMAOverruns;
} CpmacHwStatistics;
	
#define DDC_NET_IOCTL_MIN               (DDC_IOCTL_MAX + 1)
#define DDC_NET_IOCTL_GET_NET_STATS     DDC_IOCTL(DDC_NET_IOCTL_MIN, 0)
#define DDC_NET_IOCTL_CLR_NET_STATS     DDC_IOCTL(DDC_NET_IOCTL_MIN, 1)
#define DDC_NET_IOCTL_MAX               (DDC_IOCTL_MAX + 31)
#define DDC_NETDEV_ERROR_MIN            (DDC_ERROR_MAX + 1)
#define DDC_NETDEV_ERROR_MAX            (DDC_NETDEV_ERROR_MIN + 31)
	
typedef void* DDC_NetDataToken;
	
typedef struct
{
	DDC_NetDataToken    bufToken;       /**< Buffer Token. */  
	char                *data;       /**< Pointer to data buffer */
	int                 length;         /**< Buffer Length (number of bytes) */
} DDC_NetBufObj;
	
typedef struct
{
	DDC_NetDataToken    pktToken;       /**< OS Data Token /may hold Tx/Rx chan id */   
	DDC_NetBufObj       *bufList;       /**< Array of Network Buffer objects */
	int                 numBufs;        /**< Number of Network Buffer objects */
	int                 pktLength;      /**< Packet Length (number of bytes) */
} DDC_NetPktObj;
	
typedef enum
{
	DDC_NET_CH_DIR_TX = 0,              /**< Transmit only */
	DDC_NET_CH_DIR_RX,                  /**< Receive only */
	DDC_NET_CH_DIR_BIDIRECTIONAL,       /**< Bidirectonaly - TX/RX  */
	DDC_NET_CH_DIR_UNDEFINED            /**< Not defined */
} DDC_NetChDir;
	
typedef enum
{
	DDC_NET_CH_UNINITIALIZED = 0,       /**< Uninitialized state */
	DDC_NET_CH_INITIALIZED,             
	DDC_NET_CH_OPENED,                  /**< Channel in open state */
	DDC_NET_CH_CLOSE_IN_PROGRESS,       /**< Channel close/teardown in progress */
	DDC_NET_CH_CLOSED                  
} DDC_NetChState;
	
	
typedef struct
{
	int             channel;            /**< Channel number */
	int             direction;         
	DDC_NetChState  state;              /**< Channel State */
} DDC_NetChInfo;
	
typedef struct
{
	DDC_InitConfig  ddcInitConfig;      /**< DDC 'inherited' init config parameters */ 
	int             numTxChannels;      /**< Number of supported TX channels */ 
	int             numRxChannels;      /**< Number of supported TX channels */ 
} DDC_NetInitConfig;
	
	
typedef int (*DDC_NetSend) (
	DDC_Handle      hDDC,
	DDC_NetPktObj   *pkt,
	int             channel,
	void*             sendArgs);
	
typedef int (*DDC_NetSendMultiple) (
	DDC_Handle      hDDC,
	DDC_NetPktObj   *netPktList,
	int             numPkts,
	int             channel,
	void*             sendArgs,
	int             *pktsSent);
	
typedef int (*DDC_NetPollRx) (
	DDC_Handle      hDDC,
	DDC_NetPktObj   **netPktList,
	int             numPkts,
	void*             rxArgs,
	int             *retNumPkts);
	
typedef int (*DDC_NetRxReturn) (
	DDC_Handle      hDDC,
	DDC_NetDataToken    netDataTokens,
	int             numTokens,
	void*             rxRetArgs);
	typedef int (*DDC_NetIsr) (
	DDC_Handle  hDDC,
	void*         isrArgs);
	
typedef int (*DDC_NetChOpen) (
	DDC_Handle      *hDDC,
	DDC_NetChInfo   *chInfo,
	void*             chOpenArgs);
	
typedef int (*DDC_NetChClose) (
	DDC_Handle  hDDC,
	int         channel,
	int         direction,
	void*         chCloseArgs);
typedef struct 
	{
	DDC_FuncTable           ddcFuncTable;       /**< Reference to the DDC layer */
	DDC_NetSend             ddcNetSend;         /**< Send one packet on the network */
	DDC_NetSendMultiple     ddcNetSendMultiple; /**< Send multiple pkts on the network */
	DDC_NetPollRx           ddcNetPollReceive;  /**< Poll packets from the DDC */
	DDC_NetRxReturn         ddcNetRxReturn;     /**< Return packet buffers back to DDC */
	DDC_NetIsr              ddcNetIsr;          /**< interrupt handler */
	DDC_NetChOpen           ddcNetChOpen;       /**< Network Channel Open */
	DDC_NetChClose          ddcNetChClose;      /**< Network Channel Close */   
} DDC_NetFuncTable;
	
typedef DDC_NetFuncTable *DDC_NetFuncTableHandle;
	
typedef int (*DDA_NetRxNotifyCb) (
	DDA_Handle          hDDA,           /**[In]< DDA Handle */
	int                 count,          /**[In]< Number of available Packets */
	int                 chanNum);       /**[In]< Channel number */
	
typedef void* (*DDA_NetAllocRxBufCb) (
	DDA_Handle      hDDA,
	int             bufSize,
	DDC_NetDataToken    *dataToken,
	void*             allocArgs);
	
typedef int (*DDA_NetFreeRxBufCb) (
	DDA_Handle      hDDA,
	void*             buffer,
	DDC_NetDataToken    dataToken,
	void*             freeArgs);
	
typedef int (*DDA_NetRxCb) (
	DDA_Handle      hDDC,
	DDC_NetPktObj   *pkt,
	void*             rxArgs,
	void*             param);
typedef int (*DDA_NetRxMultipleCb) (
	DDA_Handle      hDDC,
	DDC_NetPktObj       *netPktList,
	int             numPkts,
	void*             rxArgs);
	
typedef int (*DDA_NetTxCompleteCb) (
	DDA_Handle      hDDA,
	DDC_NetDataToken    netDataTokens,
	int             numTokens,
	void*             args);
	
typedef struct 
{
	DDA_FuncTable              ddaFuncTable;           /**< Reference to the DDC layer */
	DDA_NetRxNotifyCb          ddaNetRxNotify;        
	DDA_NetAllocRxBufCb        ddaNetAllocRxBufCb;     
	DDA_NetFreeRxBufCb         ddaNetFreeRxBufCb;      
	DDA_NetRxCb                ddaNetrxCb;             /**< Receive  single packet */
	DDA_NetRxMultipleCb        ddaNetrxMultipleCb;     /**< Receive multiple packets */
	DDA_NetTxCompleteCb        ddaNettxCompleteCb;     
} DDA_NetFuncTable;
	
typedef DDA_NetFuncTable *DDA_NetFuncTableHandle;
	
#define CPMAC_INSTANCE_CODE                     0 /*hDDC->initCfg.instId */
#define CPMAC_ERROR_CODE                        ((DDC_ERROR | (CPMAC_INSTANCE_CODE << 16)) + DDC_NETDEV_ERROR_MAX)
#define CPMAC_ERROR_INFO                        (CPMAC_ERROR_CODE)
#define CPMAC_ERROR_WARNING                     (CPMAC_ERROR_CODE | 0x10000000)
#define CPMAC_ERROR_MINOR                       (CPMAC_ERROR_CODE | 0x20000000)
#define CPMAC_ERROR_MAJOR                       (CPMAC_ERROR_CODE | 0x30000000)
#define CPMAC_ERROR_CRITICAL                    (CPMAC_ERROR_CODE | 0x40000000)
	
#define CPMAC_SUCCESS                           PAL_SOK
#define CPMAC_ERR_DEV_ALREADY_INSTANTIATED(instID) (0x30000000 + DDC_ERROR + DDC_NETDEV_ERROR_MAX + ((instId) << 16) )
#define CPMAC_ERR_DEV_NOT_INSTANTIATED          (CPMAC_ERROR_MAJOR + 1)
#define CPMAC_INVALID_PARAM                     (CPMAC_ERROR_MAJOR + 2)
#define CPMAC_ERR_TX_CH_INVALID                 (CPMAC_ERROR_CRITICAL + 3)
#define CPMAC_ERR_TX_CH_ALREADY_INIT            (CPMAC_ERROR_MAJOR + 4)
#define CPMAC_ERR_TX_CH_ALREADY_CLOSED          (CPMAC_ERROR_MAJOR + 5)
#define CPMAC_ERR_TX_CH_NOT_OPEN                (CPMAC_ERROR_MAJOR + 6)
#define CPMAC_ERR_TX_NO_LINK                    (CPMAC_ERROR_MAJOR + 7)
#define CPMAC_ERR_TX_OUT_OF_BD                  (CPMAC_ERROR_MAJOR + 8)
#define CPMAC_ERR_RX_CH_INVALID                 (CPMAC_ERROR_CRITICAL + 9)
#define CPMAC_ERR_RX_CH_ALREADY_INIT            (CPMAC_ERROR_MAJOR + 10)
#define CPMAC_ERR_RX_CH_ALREADY_CLOSED          (CPMAC_ERROR_MAJOR + 11)
#define CPMAC_ERR_RX_CH_NOT_OPEN                (CPMAC_ERROR_MAJOR + 12)
#define CPMAC_ERR_DEV_ALREADY_CREATED           (CPMAC_ERROR_MAJOR + 13)
#define CPMAC_ERR_DEV_NOT_OPEN                  (CPMAC_ERROR_MAJOR + 14)
#define CPMAC_ERR_DEV_ALREADY_CLOSED            (CPMAC_ERROR_MAJOR + 15)
#define CPMAC_ERR_DEV_ALREADY_OPEN              (CPMAC_ERROR_MAJOR + 16)
#define CPMAC_ERR_RX_BUFFER_ALLOC_FAIL          (CPMAC_ERROR_CRITICAL + 17)
#define CPMAC_INTERNAL_FAILURE                  (CPMAC_ERROR_MAJOR + 18)
#define DDC_NET_CPMAC_IOCTL_BASE                0       /* Arbitrary base */
#define CPMAC_DDC_IOCTL_GET_SWVER                DDC_IOCTL(DDC_NET_IOCTL_MIN, 2)
#define CPMAC_DDC_IOCTL_GET_HWVER                DDC_IOCTL(DDC_NET_IOCTL_MIN, 3)
#define CPMAC_DDC_IOCTL_SET_RXCFG                DDC_IOCTL(DDC_NET_IOCTL_MIN, 4)
#define CPMAC_DDC_IOCTL_SET_MACCFG               DDC_IOCTL(DDC_NET_IOCTL_MIN, 5)
#define CPMAC_DDC_IOCTL_GET_STATUS               DDC_IOCTL(DDC_NET_IOCTL_MIN, 6)
#define CPMAC_DDC_IOCTL_READ_PHY_REG            DDC_IOCTL(DDC_NET_IOCTL_MIN, 7)
#define CPMAC_DDC_IOCTL_WRITE_PHY_REG           DDC_IOCTL(DDC_NET_IOCTL_MIN, 8)
#define CPMAC_DDC_IOCTL_GET_STATISTICS          DDC_NET_IOCTL_GET_NET_STATS
#define CPMAC_DDC_IOCTL_CLR_STATISTICS          DDC_NET_IOCTL_CLR_NET_STATS 
#define CPMAC_DDC_IOCTL_MULTICAST_ADDR          DDC_IOCTL(DDC_NET_IOCTL_MIN, 9) 
#define CPMAC_DDC_IOCTL_ALL_MULTI               DDC_IOCTL(DDC_NET_IOCTL_MIN, 10) 
#define CPMAC_DDC_IOCTL_TYPE2_3_FILTERING       DDC_IOCTL(DDC_NET_IOCTL_MIN, 11)
#define CPMAC_DDC_IOCTL_SET_MAC_ADDRESS       DDC_IOCTL(DDC_NET_IOCTL_MIN, 12)
#define CPMAC_DDC_IOCTL_IF_COUNTERS		DDC_IOCTL(DDC_NET_IOCTL_MIN,13)
#define CPMAC_DDC_IOCTL_ETHER_COUNTERS     	DDC_IOCTL(DDC_NET_IOCTL_MIN,14)	
#define CPMAC_DDC_IOCTL_IF_PARAMS_UPDT  	DDC_IOCTL(DDC_NET_IOCTL_MIN,15)    

#define DDA_NET_CPMAC_IOCTL_BASE              0    /* Arbitrary Base */
#define CPMAC_DDA_IOCTL_TIMER_START           (DDA_NET_CPMAC_IOCTL_BASE + 1)
#define CPMAC_DDA_IOCTL_TIMER_STOP            (DDA_NET_CPMAC_IOCTL_BASE + 2)
#define CPMAC_DDA_IOCTL_STATUS_UPDATE         (DDA_NET_CPMAC_IOCTL_BASE + 3)
#define CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START (DDA_NET_CPMAC_IOCTL_BASE + 4)
#define CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP (DDA_NET_CPMAC_IOCTL_BASE + 5)
	
#define SNWAY_AUTOMDIX      (1<<16)   /* Bit 16 and above not used by MII register */
#define SNWAY_FD1000        (1<<13)
#define SNWAY_HD1000        (1<<12)
#define SNWAY_NOPHY         (1<<10)
#define SNWAY_LPBK          (1<<9)
#define SNWAY_FD100         (1<<8)
#define SNWAY_HD100         (1<<7)
#define SNWAY_FD10          (1<<6)
#define SNWAY_HD10          (1<<5)
#define SNWAY_AUTO          (1<<0)
#define SNWAY_AUTOALL       (SNWAY_AUTO|SNWAY_FD100|SNWAY_FD10|SNWAY_HD100|SNWAY_HD10)
	
#define CPMAC_DDC_NO_ERROR          0   /**< Ioctl Success */
#define CPMAC_DDC_TX_HOST_ERROR     0x1 
#define CPMAC_DDC_RX_HOST_ERROR     0x2 
	
typedef struct
{
	unsigned int          hwStatus;     
	unsigned int          hwErrInfo;        
	unsigned int          PhyLinked;          /**< Link status: 1=Linked, 0=No link */
	unsigned int          PhyDuplex;         
	unsigned int          PhySpeed;      
	unsigned int          PhyNum;        
} CpmacDDCStatus;
	
	
typedef struct
{
	int             chNum;              /**< DDC_NetChInfo: Channel number */
	DDC_NetChDir    chDir;              /**< DDC_NetChInfo: Channel direction */
	DDC_NetChState  chState;            /**< DDC_NetChInfo: Channel state */
	int             numBD;              /**< Number of BD (& buffers for RX) */
	int             serviceMax;         /**< Maximum BD's processed in one go */
	int             bufSize;            /**< Buffer Size (applicable for RX only) */
} CpmacChInfo;
	
	
typedef struct
{
	Bool        passCRC;                    /**< Pass CRC bytes to the packet memory */
	Bool        qosEnable;                  /**< Receive QoS enable ? */
	Bool        noBufferChaining;           
	Bool        copyMACControlFramesEnable;
	Bool        copyShortFramesEnable;      /**< Copy Short frames to packet memory */
	Bool        copyErrorFramesEnable;      /**< Copy Errored frames to packet memory */
	Bool        promiscousEnable;           
	unsigned int      promiscousChannel;          /**< Promiscous receive channel */
	Bool        broadcastEnable;            /**< Receive broadcast frames ? */
	unsigned int      broadcastChannel;           /**< Broadcast receive channel */
	Bool        multicastEnable;            /**< Receive multicast frames ? */
	unsigned int      multicastChannel;           /**< Multicast receive channel */
	unsigned int      maxRxPktLength;             /**< Max receive packet length */
	unsigned int      bufferOffset;   
} CpmacRxConfig;
	
typedef enum
{
	CPMAC_TXPRIO_ROUND_ROBIN = 0,   
	CPMAC_TXPRIO_FIXED = 1          /**< Fixed priority mechanism between TX channels */
} CpmacTxQueuePriorityType;
	
typedef struct
{
#if (AVALANCHE_CPMAC_HW_MODULE_REV > 0x000c0100)
	Bool        gigForce;                   /**< Force Gigabit mode */
	Bool        gpioB;                      /**< general purpose output GMII */
	Bool        gpioA;                      /**< general purpose output GMII */
	Bool        rxOfflenBlock;          
	Bool        rxOwnership;               
	Bool        rxFifoFlowEnable;           
	Bool        cmdIdle;                    /**< enable / disable idle command*/
#endif
	CpmacTxQueuePriorityType    pType;      /**< Transmit Queue priority type */
	Bool        txShortGapEnable;           
	Bool        gigaBitEnable;              /**< Gigabit mode -CP(G)MAC only */
	Bool        txPacingEnable;             /**< Transmit pacing enabled ? */
	Bool        rxPacingEnable;             /**< Receive  pacing enabled ? */
	Bool        miiEnable;                  /**< KEPT FOR DEBUGGING ONLY - ALWAYS SET TO 1 */
	Bool        txFlowEnable;               /**< TX Flow Control enabled ? */
	Bool        rxFlowEnable;               /**< TX Flow Control enabled ? */
	Bool        loopbackEnable;             /**< Loopback mode enabled ? */
	Bool        fullDuplexEnable;           /**< KEPT FOR DEBUGGING ONLY - Will be set based upon phyMode */
	Bool        txInterruptDisable;         /**< To allow Disabling of Tx Completion interrupts */
		int			txIntThresholdValue;        /**< Tx threshold value */
} CpmacMacConfig;
	
typedef struct
{
	unsigned int      instId;             /**< DDC Init Cfg: Instance number */
	unsigned int      numTxChannels;      /**< DDCNet Init Cfg: Number of Tx Channels to be supported */
	unsigned int      numRxChannels;      /**< DDCNet Init Cfg: Number of Rx Channels to be supported */ 
	unsigned int      cpmacBusFrequency;  /**< Bus frequency at which this module is operating */
	unsigned int      baseAddress;        /**< CPMAC peripheral device's register overlay address */
	unsigned int      intrLine;           /**< CPMAC Device interrupt Line Number within the system */
	unsigned int      resetLine;          /**< CPMAC Reset Line Number within the system */
	unsigned int      mdioBaseAddress;    /**< MDIO Device base address */
	unsigned int      mdioResetLine;      /**< MDIO Device Reset line number within the system */
	unsigned int      mdioIntrLine;       /**< MDIO Device interrupt line number within the system */
	unsigned int      PhyMask;            /**< Phy Mask for this CPMAC Phy  */
	unsigned int      MLinkMask;          /**< MLink Mask for this CPMAC Phy  */
	unsigned int      MdioBusFrequency;   /**< Bus frequency for the MII module */
	unsigned int      MdioClockFrequency; /**< Clock frequency for MDIO link */
	unsigned int      MdioTickMSec;       /**< DDC MDIO Tick count in milliSeconds */
	unsigned int      Mib64CntMsec; 
	unsigned int      phyMode;            /**< Phy mode settings - Speed,Duplex */
	CpmacRxConfig   rxCfg;          /**< RX common configuration */
	CpmacMacConfig  macCfg;         /**< MAC common configuration */
} CpmacInitConfig;
	
#ifndef CPMAC_DDC
	typedef DDC_Handle CpmacDDCObj;
#endif
	
typedef int (*DDC_CpmacTick) (
	CpmacDDCObj   *hDDC,
	void*             tickArgs);
	
typedef void (*DDC_CpmacGetInterruptCause) (
	CpmacDDCObj     *hDDC,
	Bool             *rxPending,
	Bool             *txPending,
	void*              causeArgs);
	
typedef int (*DDC_CpmacPktProcess) (
	CpmacDDCObj    *hDDC,
	int             *pktsPending,
	void*             pktArgs);
	
typedef int (*DDC_CpmacPktTxCompletionProcess) (
	CpmacDDCObj    *hDDC,
	int             *pktsPending,
	void*             pktArgs);
typedef int (*DDC_CpmacPktRxProcess) (
	CpmacDDCObj    *hDDC,
	int             *pktsPending,
	void*             pktArgs);
	
typedef int (*DDC_CpmacPktProcessEnd) (
	CpmacDDCObj    *hDDC,
	void*             procArgs);
	
typedef int (*DDC_CpmacRecycleBuffer) (
	CpmacDDCObj    *hDDC,
	int            channel,
	void*            param);
	
typedef int (*DDC_CpmacAddRxBd) (
	CpmacDDCObj *hDDC,
	CpmacChInfo *chInfo,
	unsigned int numOfBd2Add);

typedef struct
{
	DDC_NetFuncTable        ddcNetIf;           /* DDC and Net function table */
#ifdef CPMAC_RX_RECYCLE_BUFFER
	DDC_CpmacRecycleBuffer  ddcRecycleBuffer;   /**< DDC CPMAC Buffer recycle function */
#endif
	DDC_CpmacTick              ddctick;            /**< DDC CPMAC Tick function */
	DDC_CpmacPktProcess        pktProcess;         /**< DDC CPMAC packet processing function */
	DDC_CpmacPktProcessEnd     pktProcessEnd;      /**< DDC CPMAC End of packet processing function */
	DDC_CpmacGetInterruptCause getInterruptCause;  /**< DDC CPMAC function returning interrupt cause */
	DDC_CpmacPktTxCompletionProcess pktTxCompletionProcess;       /**< DDC CPMAC Tx packet processing function */
	DDC_CpmacPktRxProcess      pktRxProcess;       /**< DDC CPMAC Rx packet processing function */
		DDC_CpmacAddRxBd           AddRxBd;			   /**< DDC CPMAC add Rx BD function */
	
} CpmacDDCIf;

typedef int (*DDA_Printf)(const char *format, ...);
typedef int (*DDA_ErrLog)(const char *format, ...);
	
typedef struct
{
	DDA_NetFuncTable    ddaNetIf;          
	DDA_Printf          ddaPrintf;          /**< DDA provided debug printing function */
	DDA_ErrLog          ddaErrLog;          /**< DDA provided error logging function */
} CpmacDDACbIf;
	
unsigned char* DDC_cpmacGetVersionInfo(unsigned int *swVer);
int DDC_cpmacCreateInstance (unsigned int instId, DDA_Handle hDDA,  CpmacDDACbIf *hDDACbIf, DDC_Handle **hDDC, CpmacDDCIf **hDDCIf, void* param );
	
void* cpmacGetPhyDev(CpmacDDCObj *hDDC);
	
typedef struct {
	unsigned int rxPkts;		/* Number of rx pkts to be processed */
	unsigned int txPkts;		/* Number of tx pkts to be processed */

	unsigned int retRxPkts;	/* Number of rx pkts processed */
	unsigned int retTxPkts;	/* Number of tx pkts processed */
	
	unsigned int rxMaxService;   /* Number of max rx that can be serviced*/
	unsigned int txMaxService;   /* Number of max tx that can be serviced*/
}RxTxParams;
	
#define CPMAC_DDC_MAX_TX_COMPLETE_PKTS_TO_NOTIFY    64
#define CPMAC_DDC_MAX_RX_COMPLETE_PKTS_TO_NOTIFY    64
#define CPMAC_MAX_INSTANCES                     6       /**< Max CPMAC instances */
#define CPMAC_MIN_ETHERNET_PKT_SIZE             60      /**< Minimum Ethernet packet size */
#define CPMAC_MAX_RX_FRAGMENTS                  24      /**< Maximum RX fragments supported */
#define CPMAC_MAX_TX_FRAGMENTS                  8       /**< Maximum TX fragments supported */
	
#define CPMAC_RESET_CLOCKS_WAIT                 64      /**< Clocks to wait for reset operation */
#define CPMAC_MAX_TX_CHANNELS                   8       /**< Maximum TX Channels supported by the DDC */
#define CPMAC_MAX_RX_CHANNELS                   8       /**< Maximum RX Channels supported by the DDC */
#define CPMAC_MIN_FREQUENCY_FOR_10MBPS          5500000    
#define CPMAC_MIN_FREQUENCY_FOR_100MBPS         55000000
#define CPMAC_MIN_FREQUENCY_FOR_1000MBPS        125000000 
	
#define PAL_CPMAC_VIRT_2_PHYS(addr)             (((unsigned int)(addr)) &~ 0xE0000000)
#define PAL_CPMAC_VIRT_NOCACHE(addr)            ((void*)((PAL_CPMAC_VIRT_2_PHYS(addr)) | 0xA0000000))
	
#define PAL_CPMAC_CACHE_INVALIDATE(addr, size)  dma_cache_sync((void*)(addr), (unsigned int)(size), DMA_FROM_DEVICE)
	
#define PAL_CPMAC_CACHE_WRITEBACK(addr, size)   dma_cache_sync((void*)(addr), (unsigned int)(size), DMA_TO_DEVICE)
	
#define PAL_CPMAC_CACHE_WRITEBACK_INVALIDATE(addr, size)   dma_cache_sync((void*)(addr), (unsigned int)(size), DMA_BIDIRECTIONAL)
	
#define CPMAC_DEBUG_FUNCTION_ENTRY          (0x1 << 1)      /* Almost All functions entry/exit */
#define CPMAC_DEBUG_FUNCTION_EXIT           (0x1 << 2)          
#define CPMAC_DEBUG_BUSY_FUNCTION_ENTRY     (0x1 << 3)      /* Busy functions - frequently called entry/exit */
#define CPMAC_DEBUG_BUSY_FUNCTION_EXIT      (0x1 << 4)
#define CPMAC_DEBUG_TX                      (0x1 << 6)      /* Transmit functionality */
#define CPMAC_DEBUG_RX                      (0x1 << 7)      /* Receive functionality */
#define CPMAC_DEBUG_PORT_UPDATE             (0x1 << 8)      /* Port status updates */ 
#define CPMAC_DEBUG_MII                     (0x1 << 9)      /* MII debug */
#define CPMAC_DEBUG_TEARDOWN                (0x1 << 10)     /* Teardown debug */
	
#define CPMAC_RX_BD_BUF_SIZE                    0xFFFF;
#define CPMAC_BD_LENGTH_FOR_CACHE               16      /* Only CPPI specified bytes to be invalidated */
#define CPMAC_RX_BD_PKT_LENGTH_MASK             0xFFFF
	
typedef struct
{
	unsigned int   hNext;      /**< next (hardware) buffer descriptor pointer */
	unsigned int   buff;    /**< data buffer pointer */
	unsigned int   off_bLen;   /**< (buffer_offset_16)(buffer_length_16) */
	unsigned int   mode;       /**< SOP, EOP, Ownership, EOQ, Teardown, Q Starv, Length */
	void*     next;       /**< Pointer to the next TX buffer descriptor (linked list) */
	DDC_NetDataToken    bufToken;   /**< Data Buffer (OS) Token */
	void*     eopBD;      /**< Pointer to end of packet BD */ 
}CpmacTxBD;    
	
typedef struct _CpmacRxCppiCh_t CpmacRxCppiCh;
typedef volatile struct
{
	unsigned int   hNext;      /**< next (hardware) buffer descriptor pointer */
	unsigned int   buff;    /**< data buffer pointer */
	unsigned int   off_bLen;   /**< (buffer_offset_16)(buffer_length_16) */
	unsigned int   mode;       /**< SOP, EOP, Ownership, EOQ, Teardown, Q Starv, Length */
	void*     next;       /**< Pointer to the next RX buffer in BD queue */
	void*     data;    /**< Datavoid* (virtual address) of the buffer allocated */
	DDC_NetDataToken    bufToken;   /**< Data Buffer (OS) Token */
	CpmacRxCppiCh       *rxCppi;    /**< RX CPPI channel owning this BD */
} CpmacRxBD;
	
	
typedef struct
{
	CpmacChInfo     chInfo;             /**< Channel config/info */
	unsigned int          allocSize;        
	char            *bdMem;             /**< Buffer Descriptor Memory pointer */
	CpmacTxBD       *bdPoolHead;        /**< Free BD Pool Head */
	CpmacTxBD       *activeQueueHead;   /**< Head of active packet queue */
	CpmacTxBD       *activeQueueTail;   /**< Last hardware buffer descriptor written */
	CpmacTxBD       *lastHwBDProcessed; /**< Last hardware buffer descriptor processed */
	Bool            queueActive;        /**< Queue Active ? 1/0 */
#ifdef CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
	unsigned int          *txComplete;        /**< Tx complete notification queue */
#endif
	unsigned int          procCount;        
	unsigned int          misQueuedPackets;   /**< Misqueued packets */
	unsigned int          queueReinit;        /**< Queue reinit - Head ptr reinit */
	unsigned int          endOfQueueAdd;      
	unsigned int          outOfTxBD;          /**< out of tx bd errors */
	unsigned int          noActivePkts;      
	unsigned int          activeQueueCount;   /**< Active tx bd count */
	unsigned int          numMultiFragPkts;
} CpmacTxCppiCh;
	
typedef struct
{
	char *extraAllocMem;
	struct CpmacBdMemChunk *ptr_next;
}CpmacBdMemChunk;
	
typedef struct _CpmacRxCppiCh_t
{
	CpmacDDCObj     *hDDC;
	CpmacChInfo     chInfo;             /**< Channel config/info */
	char            macAddr[6];         /**< Ethernet MAC address */
	unsigned int          allocSize;   
	char            *bdMem;            /**< Buffer Descriptor Memory pointer */
	CpmacBdMemChunk *extraBdMem;      
	CpmacRxBD       *bdPoolHead;        /**< Free BD Pool Head */
	CpmacRxBD       *activeQueueHead;   
	CpmacRxBD       *activeQueueTail;   /**< Active BD Queue Tail */
	Bool            queueActive;        /**< Queue Active ? 1/0 */
	
	DDC_NetPktObj   *pktQueue;                          /**< Packet queue */
	DDC_NetBufObj   *bufQueue;                          /**< Buffer queue */
	
	unsigned int          procCount;         
	unsigned int          processedBD;        /**< Number of BD's processed */
	unsigned int          recycledBD;         /**< Number of recycled BD's */
	unsigned int          outOfRxBD;          /**< NO BD's available */
	unsigned int          outOfRxBuffers;     /**< NO buffers available */
	unsigned int          queueReinit;        /**< Queue re-init condition when recycling buffers */
	unsigned int          endOfQueueAdd;      /**< End of queue condition - when adding BD at end */
	unsigned int          endOfQueue;         /**< End of queue condition */
	unsigned int          misQueuedPackets;   /**< Mis-queued packet condition */
	unsigned int          numMultiFragPkts;  
} _CpmacRxCppiCh;
	
typedef struct _phy_device phy_device;
#include "ti_titan_mdio.h"
	
typedef struct _CpmacDDCObj_t
{
	DDC_Obj             ddcObj;         /* DDC Object containing version, instance id and driver state */
	
	CpmacInitConfig     initCfg;                            /**< Initialization Configuration */
	CpmacTxCppiCh       *txCppi[CPMAC_MAX_TX_CHANNELS];     /**< Tx Control structure pointers */
	CpmacRxCppiCh       *rxCppi[CPMAC_MAX_RX_CHANNELS];     /**< Rx Control structure pointers */
	Bool                txIsCreated[CPMAC_MAX_TX_CHANNELS]; /**< TX Channel created ? */
	Bool                rxIsCreated[CPMAC_MAX_RX_CHANNELS]; /**< RX Channel created ? */
	Bool                txIsOpen[CPMAC_MAX_TX_CHANNELS];    /**< TX channel opened ? */
	Bool                rxIsOpen[CPMAC_MAX_RX_CHANNELS];    /**< RX channel opened ? */
	Bool                txTeardownPending[CPMAC_MAX_TX_CHANNELS];    /**< IS TX Teardown pending ? */
	Bool                rxTeardownPending[CPMAC_MAX_RX_CHANNELS];    /**< IS RX Teardown pending ? */
	int                  txIntThreshold[CPMAC_MAX_TX_CHANNELS];    /**< TX  Completion Threshold count */
	Bool                txInterruptDisable; /* Is Tx completion interrupt disabled? */
	
	unsigned int              Rx_Unicast_Set;     /**< Unicast Set Register */
	unsigned int              Rx_Unicast_Clear;   /**< Unicast Clear Register */
	unsigned int              Rx_MBP_Enable;      /**< RX MBP Register */
	unsigned int              MacHash1;           /**< MAC Hash 1 Register */
	unsigned int              MacHash2;           /**< MAC Hash 2 Register */
	unsigned int              MacControl;         /**< MACControl Register */
	
	CpmacDDCStatus      status;             /**< Structure to capture hardware status */
	unsigned int              multicastHashCnt[CPMAC_NUM_MULTICAST_BITS]; /**< Number of multicast hash bits used in hardware */
	
	unsigned int              RxAddrType;    /**< Address Type: 0 (CPMAC), 1 or 2 (CPGMAC) * MAC Config type */
	
	phy_device          *PhyDev;        /**< MII-MDIO module device structure */
	CSL_CpmacRegsOvly   regs;           /**< Pointer points to CPMAC Base address - Register overlay */
	
	struct mib2_ifHCCounters  Mib2IfHCCounter;
	CpmacDDACbIf       *ddaIf;         /**< DDA provided callback functions */
	CpmacDDCIf         *ddcIf;         /**< DDC implemented functions */
	
} _CpmacDDCObj;
	
int DDC_cpmacSend(CpmacDDCObj *hDDC, DDC_NetPktObj *pkt, int  channel, void* sendArgs);
int cpmacTick (CpmacDDCObj *hDDC, void* tickArgs);
int cpmacPktProcess (CpmacDDCObj *hDDC, int *pktsPending, void* pktArgs);
int cpmacPktProcessEnd (CpmacDDCObj *hDDC, void* procArgs);
int cpmacTxBDProc(CpmacDDCObj *hDDC,  unsigned int channel, unsigned int *morePkts, Bool * isEOQ);
int CpmacRxBDProc(CpmacDDCObj *hDDC,  unsigned int channel, int *morePkts);
#ifdef CPMAC_MULTIFRAGMENT
	void cpmacAddBDToRxQueue(CpmacDDCObj *hDDC, CpmacRxCppiCh *rxCppi, CpmacRxBD *sopBD, CpmacRxBD *eopBD, unsigned int *buffer, DDC_NetDataToken *bufToken, unsigned int numBD);
#else
	void cpmacAddBDToRxQueue(CpmacDDCObj *hDDC, CpmacRxCppiCh *rxCppi, CpmacRxBD *currBD, char *buffer, DDC_NetDataToken bufToken);
#endif
void cpmacAddListToRxQueue(CpmacDDCObj *hDDC, CpmacRxCppiCh *rxCppi, CpmacRxBD *first, CpmacRxBD *last);
int cpmacUpdatePhyStatus(CpmacDDCObj *hDDC);
void cpmacGetInterruptCause(CpmacDDCObj *hDDC,Bool *rxPending,Bool *txPending,Bool *errPending);
int cpmacTxPktCompletionProcess (CpmacDDCObj *hDDC, int *pktsPending, void* pktArgs);
int cpmacRxPktProcess (CpmacDDCObj *hDDC, int *pktsPending, void* pktArgs);
	
#define EGRESS_TRAILOR_LEN                  0
#define CFG_START_LINK_SPEED                (SNWAY_AUTOALL) /* auto nego */
#define CPMAC_DEFAULT_MLINK_MASK                        0
#define CPMAC_DEFAULT_PASS_CRC                          0
#define CPMAC_DEFAULT_QOS_ENABLE                        0
#define CPMAC_DEFAULT_NO_BUFFER_CHAINING                0
#define CPMAC_DEFAULT_COPY_MAC_CONTROL_FRAMES_ENABLE    0
#define CPMAC_DEFAULT_COPY_SHORT_FRAMES_ENABLE          0
#define CPMAC_DEFAULT_COPY_ERROR_FRAMES_ENABLE          0
#define CPMAC_DEFAULT_PROMISCOUS_CHANNEL                0
#define CPMAC_DEFAULT_BROADCAST_CHANNEL                 0
#define CPMAC_DEFAULT_MULTICAST_CHANNEL                 0
#define CPMAC_DEFAULT_BUFFER_OFFSET                     0
#define CPMAC_DEFAULT_TX_PRIO_TYPE                      CPMAC_TXPRIO_FIXED
#define CPMAC_DEFAULT_TX_SHORT_GAP_ENABLE               0

#if (AVALANCHE_CPMAC_HW_MODULE_REV > 0x000c0100)

#define CPMAC_DEFAULT_GIGFORCE                          0
#define CPMAC_DEFAULT_RX_FIFO_FLOW_ENABLE               0

#endif
	
#define LOW_CPMAC_DEFAULT_TX_PACING_ENABLE              0
#define HIGH_CPMAC_DEFAULT_TX_PACING_ENABLE             0
	
#define CPMAC_DEFAULT_TX_PACING_ENABLE(inst)        \
	((inst == 0) ? LOW_CPMAC_DEFAULT_TX_PACING_ENABLE : HIGH_CPMAC_DEFAULT_TX_PACING_ENABLE)
	
#define CPMAC_DEFAULT_MII_ENABLE                        1
#define CPMAC_DEFAULT_TX_FLOW_ENABLE                    0
#define CPMAC_DEFAULT_RX_FLOW_ENABLE                    0
#define CPMAC_DEFAULT_LOOPBACK_ENABLE                   0
#define CPMAC_DEFAULT_FULL_DUPLEX_ENABLE                1
#define CPMAC_DEFAULT_TX_INTERRUPT_DISABLE              1
#define CPMAC_DEFAULT_TX_THRESHOLD_VALUE                20
	
#define CONFIG_CPMAC_MIB_TIMER_TIMEOUT                  5000 /* 5 seconds should be enough */
	
#define CPMAC_DEFAULT_PROMISCOUS_ENABLE                 0
#define CPMAC_DEFAULT_BROADCAST_ENABLE                  1
#define CPMAC_DEFAULT_MULTICAST_ENABLE                  1
#define CPMAC_DDA_CACHE_INVALIDATE(addr, size)      dma_cache_inv((unsigned long)(addr), (size));    
#define CPMAC_DDA_CACHE_WRITEBACK(addr, size)       dma_cache_wback_inv((unsigned long)skb->data, skb->len);
#define CPMAC_DDA_DEFAULT_VLAN_ENABLE       0
	
#define CPMAC_DDA_TICKS_PER_SEC             HZ 
#define CPMAC_DDA_IOCTL_BASE                0
#define CPMAC_DDA_PRIV_FILTERING            (CPMAC_DDA_IOCTL_BASE + 1)
#define CPMAC_DDA_PRIV_MII_READ             (CPMAC_DDA_IOCTL_BASE + 2)
#define CPMAC_DDA_PRIV_MII_WRITE            (CPMAC_DDA_IOCTL_BASE + 3)
#define CPMAC_DDA_PRIV_GET_STATS            (CPMAC_DDA_IOCTL_BASE + 4)
#define CPMAC_DDA_PRIV_CLR_STATS            (CPMAC_DDA_IOCTL_BASE + 5)
#define CPMAC_DDA_EXTERNAL_SWITCH           (CPMAC_DDA_IOCTL_BASE + 6)
#define CPMAC_DDA_SET_ISR_MODE              (CPMAC_DDA_IOCTL_BASE + 7)
#define CPMAC_DDA_SET_TASKLET_MODE           (CPMAC_DDA_IOCTL_BASE + 8)
#define CPMAC_DDA_ADD_RX_BD                  (CPMAC_DDA_IOCTL_BASE + 9)
#define CPMAC_DDA_DEFAULT_MAX_FRAME_SIZE    (1500 + 14 + 4 + 4 + EGRESS_TRAILOR_LEN)
#define CPMAC_DDA_DEFAULT_EXTRA_RXBUF_SIZE  0
#define CPMAC_DDA_DEFAULT_NUM_TX_CHANNELS   1
#define CPMAC_DDA_DEFAULT_TX_CHANNEL        0
#define LOW_CPMAC_DDA_DEFAULT_TX_NUM_BD         64
#define LOW_CPMAC_DDA_DEFAULT_TX_MAX_SERVICE    20
#define HIGH_CPMAC_DDA_DEFAULT_TX_NUM_BD        64
#define HIGH_CPMAC_DDA_DEFAULT_TX_MAX_SERVICE   20
#define CPMAC_DDA_DEFAULT_NUM_RX_CHANNELS   1
#define CPMAC_DDA_DEFAULT_RX_CHANNEL        0
#define HIGH_CPMAC_DDA_DEFAULT_RX_NUM_BD        64
#define LOW_CPMAC_DDA_DEFAULT_RX_NUM_BD         64
#define LOW_CPMAC_DDA_DEFAULT_RX_MAX_SERVICE    8
#define HIGH_CPMAC_DDA_DEFAULT_RX_MAX_SERVICE   8
#define CPMAC_DDA_DEFAULT_CPMAC_SIZE        0x800
#define CPMAC_DDA_MAC_ADDR_A    "maca"
#define CPMAC_DDA_MAC_ADDR_B    "macb"
#define CPMAC_DDA_MAC_ADDR_C    "macc"
#define CPMAC_DDA_MAC_ADDR_D    "macd"
#define CPMAC_DDA_MAC_ADDR_E    "mace"
#define CPMAC_DDA_MAC_ADDR_F    "macf"
#define CPMAC_DDA_CONFIG_A      "MACCFG_A"
#define CPMAC_DDA_CONFIG_B      "MACCFG_B"
#define CPMAC_DDA_CONFIG_C      "MACCFG_C"
#define CPMAC_DDA_CONFIG_D      "MACCFG_D"
#define CPMAC_DDA_CONFIG_E      "MACCFG_E"
#define CPMAC_DDA_CONFIG_F      "MACCFG_F"
#define CPMAC_DDA_DEFAULT_MAX_MULTICAST_ADDRESSES   64

extern int cpmac_debug_mode;

#define dbgPrint if (cpmac_debug_mode) printk
	
#define CPMAC_DDA_INTERNAL_FAILURE  -1
#define CPMAC_LINK_OFF          0
#define CPMAC_LINK_ON           1
#define CPMAC_SPEED_100         2
#define CPMAC_SPEED_10          3
#define CPMAC_FULL_DPLX         4
#define CPMAC_HALF_DPLX         5
#define CPMAC_TX_ACTIVITY       6
#define CPMAC_RX_ACTIVITY       7
	
typedef struct
{
	unsigned int    cmd;    /**< Command */
	void            *data; 
} CpmacDrvPrivIoctl;
	
typedef struct
{
	unsigned long tx_discards;      /**< TX Discards */
	unsigned long rx_discards;      /**< RX Discards */
	unsigned long start_tick;       /**< Start tick */
} CpmacDrvStats;
	
typedef struct _CpmacNetDevice
	{
	void                *owner;             /**< Pointer to the net_device structure */
	unsigned int        instanceNum;        /**< Instance Number of the device */
	struct net_device   *nextDevice;        
	unsigned int        linkSpeed;          /**< Link Speed */
	unsigned int        linkMode;           /**< Link Mode */
	unsigned long       setToClose;         /**< Flag to indicate closing of device */
	void*               ledHandle;          /**< Handle for LED control */
	CpmacDDCObj         *hDDC;              /**< Handle (pointer) to Cpmac DDC object */
	CpmacDDCIf          *ddcIf;             
	CpmacDDCStatus      ddcStatus;          /**< Cpmac DDC data structure */
	
	char                macAddr[6];        /**< Mac (ethernet) address */
	CpmacInitConfig     initCfg;          
	unsigned int        rxBufSize;         
	unsigned int        rxBufOffset;     
	int                vlanEnable;        
	CpmacChInfo         txChInfo[CPMAC_MAX_TX_CHANNELS];    
	CpmacChInfo         rxChInfo[CPMAC_MAX_RX_CHANNELS]; 
	struct timer_list   periodicTimer;      /**< Periodic timer required for DDC (MDIO) polling */
	unsigned int              periodicTicks;      /**< Ticks for this timer */
	int                timerActive;        /**< Periodic timer active ??? */
	
	struct timer_list   mibTimer;      /**< Periodic timer required for 64 bit MIB counter support */
	unsigned int                mibTicks;      /**< Ticks for this timer */
	int                mibTimerActive;        /**< Periodic timer active ??? */
	CpmacHwStatistics       deviceMib;      /**< Device MIB - CPMAC hardware statistics counters */
	CpmacDrvStats           deviceStats;    /**< Device Statstics */
	struct net_device_stats netDevStats;    /**< Linux Network Device statistics */ 
	unsigned int              isrCount;           /**< Number of interrupts */
	unsigned int              Clear_EOI;
	RxTxParams		napiRxTx;
} CpmacNetDevice;
	
int  cpmac_dev_tx( struct sk_buff *skb, struct net_device *p_dev);
irqreturn_t cpmac_hal_isr(int irq, void *dev_id, struct pt_regs *p_cb_param);
void* DDA_cpmac_net_alloc_rx_buf(CpmacNetDevice* hDDA, int bufSize, 
DDC_NetDataToken *dataToken, unsigned int channel, void* allocArgs);
int DDA_cpmac_net_free_rx_buf(CpmacNetDevice* hDDA, void* buffer, 
DDC_NetDataToken dataToken, unsigned int channel, void* freeArgs);
int DDA_cpmac_net_tx_complete(CpmacNetDevice* hDDA, DDC_NetDataToken *netDataTokens,
int numTokens, unsigned int channel);
int DDA_cpmac_net_rx_multiple_cb(CpmacNetDevice* hDDA, DDC_NetPktObj *netPktList, int numPkts, void* rxArgs);
	
#endif /* _CPMAC_LX_DDA_H_ */
