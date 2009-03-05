/*******************************************************************************
*
*   Marvell Serial ATA Linux Driver
*   Copyright 2004
*   Marvell International Ltd.
*
* This software program (the "Program") is distributed by Marvell International
* ltd. under the terms of the GNU General Public License Version 2, June 1991
* (the "License").  You may use, redistribute and/or modify this Program in
* accordance with the terms and conditions of the License, a copy of which is
* available along with the Program in the license.txt file or by writing to the
* Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston,
* MA 02111-1307 or on the worldwide web at http:www.gnu.org/licenses/gpl.txt.
*
* THE PROGRAM IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
* IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE
* EXPRESSLY DISCLAIMED.  The License provides additional details about this
* warranty disclaimer.
*
* For more information about the Program or the License as it applies to the
* Program, please contact Marvell International Ltd. via its affiliate, Marvell
* Semiconductor, Inc., 700 First Avenue, Sunnyvale, CA 94010
*
********************************************************************************
* mvSata - C File for implementation of the core driver for MV88SX50XX.
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*   mvOs.h
*   mvSata.h.
*   mvStorageDev.h
*   mvRegs.h
*
*******************************************************************************/
#include "mvOs.h"
#include "mvSata.h"
#include "mvStorageDev.h"
#include "mvRegs.h"

/* Defines */
#define MV_SATA_PORT_PER_UNIT                   4

#define MV_PHY_DET_STATE_NO_DEVICE              0
#define MV_PHY_DET_STATE_DEVICE_NO_PHY_COM      1
#define MV_PHY_DET_STATE_DEVICE_AND_PHY_COM     3
#define MV_PHY_DET_STATE_PHY_OFFLINE            4
#define MV_PHY_DET_CONTROL_START_NEGOTIATION    1
#define MV_PHY_DET_CONTROL_SHUTDOWN             4
#define MV_NEAR_END_LOOPBACK_TEST_WAIT_TIME     100 /* 100 uSec */
#define MV_FAR_END_LOOPBACK_TEST_WAIT_TIME      5   /* 5 uSec */
#define MV_PHY_COM_SETUP_WAIT                   5000 /* 5 mili seconds */
#define MV_HARD_RESET_WAIT_ASSERT               25    /* 25 uSec */
#define MV_HARD_RESET_WAIT_NEGATE               1000  /* 1  mSec*/
#define MV_HARD_RESET_WAIT_READY                2000 /* ms to wait after HR*/
/* before disk access */
#define MV_HARD_RESET_WAIT_FOR_BUSY_LOOPS       10000
#define MV_HARD_RESET_WAIT_FOR_BUSY_LOOP_DELAY  1000

/* for the command result */
#define MV_EDMA_REQUEST_COMMANDS_NUM            11


/* Fix the watermark to the following default value */
#define MV_WATER_MARK_FIX                       29 /* write 5'b11101 to bits 12:8*/

extern MV_BOOLEAN waitWhileStorageDevIsBusy(MV_SATA_ADAPTER *pAdapter,
                                            MV_BUS_ADDR_T ioBaseAddr,
                                            MV_U32 eDmaRegsOffset, MV_U32 loops,
                                            MV_U32 delay);

extern void dumpAtaDeviceRegisters(MV_SATA_ADAPTER *pAdapter,
                                   MV_U8 channelIndex, MV_BOOLEAN isEXT,
                                   MV_STORAGE_DEVICE_REGISTERS *pRegisters);

extern MV_BOOLEAN _doSoftReset(MV_SATA_CHANNEL *pSataChannel);

MV_BOOLEAN isStorageDevReadyForPIO(MV_SATA_CHANNEL *pSataChannel);

MV_BOOLEAN executeNonUDMACommand(MV_SATA_ADAPTER *pAdapter,
                                 MV_U8 channelIndex,
                                 MV_NON_UDMA_PROTOCOL protocolType,
                                 MV_BOOLEAN  isEXT,
                                 MV_U16_PTR bufPtr, MV_U32 count,
                                 MV_U16 features,
                                 MV_U16 sectorCount,
                                 MV_U16 lbaLow, MV_U16 lbaMid,
                                 MV_U16 lbaHigh, MV_U8 device,
                                 MV_U8 command);

extern MV_BOOLEAN waitForDRQ(MV_SATA_ADAPTER* pAdapter,
                             MV_BUS_ADDR_T ioBaseAddr,
                             MV_U32 eDmaRegsOffset, MV_U32 loops,
                             MV_U32 delay);

static MV_BOOLEAN _checkSStatusAfterHReset(MV_SATA_ADAPTER* pAdapter,
                                           MV_U8 channelIndex);
#ifdef MV_LOGGER
void _dumpPCIRegs(MV_SATA_ADAPTER *pAdapter);

void _dumpEDMARegs(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

void _dumpChannelQueues(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

void _dumpSataRegs(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

void _printATARegs(MV_STORAGE_DEVICE_REGISTERS   *pDeviceRegs);

#else

    #define _dumpPCIRegs(p)
    #define _dumpEDMARegs(p,i)
    #define _dumpChannelQueues(p,i)
    #define _dumpSataRegs(p,i)
    #define _printATARegs(p)
#endif

/* write ATA command register entry in a request entry */
#define WRITE_ATA_COMMAND_REG(addr, data, reg, isLast)              \
do{                                                                 \
    *(addr) = MV_CPU_TO_LE16((((MV_U8)(data)) & 0xff) | ((reg) << 8) | (isLast));   \
} while(0)

#define MV_CHANNEL_INDEX(unit, port)    (((unit) << 2) | (port))


/* Typedefs */

typedef struct mvDmaRequestQueueEntry
{
    /* Fields set by CORE driver */
    volatile MV_U32       prdLowAddr;
    volatile MV_U32       prdHighAddr;
    volatile MV_U16       controlFlags;
    volatile MV_U16       command[MV_EDMA_REQUEST_COMMANDS_NUM];

} MV_DMA_REQUEST_QUEUE_ENTRY;

typedef struct mvDmaResponseQueueEntry
{
    /* Fields set by  hardware */
    volatile MV_U16 commandTag;
    volatile MV_U16 responseFlags;
    volatile MV_U32 timeStamp;
} MV_DMA_RESPONSE_QUEUE_ENTRY;


/* local functions  */

/*static*/ MV_BOOLEAN waitForBusyAfterHReset(MV_SATA_ADAPTER *pAdapter,
                                             MV_U8 channelIndex);

static void unmaskEdmaInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void maskEdmaInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void writeEdmaRequestEntry(MV_DMA_REQUEST_QUEUE_ENTRY *pReqEntry,
                                  MV_SATA_CHANNEL *mvSataChannel,
                                  MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                                  MV_UDMA_COMMAND_PARAMS  *pUdmaParams);

static void handleEdmaFailedCommand(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex, MV_U16 eDmaErrorCause);

static void handleEdmaResponse(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                               MV_DMA_RESPONSE_QUEUE_ENTRY *eDmaResponse);

#ifdef MV_SATA_C2C_COMM
static void handleBmDMAInterrupt(MV_SATA_ADAPTER *pAdapter,
                                 MV_BUS_ADDR_T   ioBaseAddr,
                                 MV_SATA_CHANNEL *pSataChannel,
                                 MV_U8   channelIndex,
                                 MV_U32 edmaError);
#endif

static void handleEdmaInterrupt(MV_SATA_ADAPTER *pAdapter, MV_U8 sataUnit,
                                MV_U8 port, MV_U32 rspInPtr,MV_U32 responseDone,
                                MV_U32 edmaError, MV_U32 unitCause);

static void handleDeviceInterrupt(MV_SATA_ADAPTER *pAdapter, MV_U8 sataUnit,
                                  MV_U8 port);

static void handleEdmaError(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static MV_VOID handleDisconnect(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static MV_VOID handleConnect(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static MV_BOOLEAN handleUnrecoverableError(MV_SATA_ADAPTER *pAdapter,
                                           MV_U8 channelIndex,
                                           MV_U32 eDmaErrorCause);

static MV_BOOLEAN handleRecoverableError(MV_SATA_ADAPTER *pAdapter,
                                         MV_U8 channelIndex,
                                         MV_U32 eDmaErrorCause);

static MV_BOOLEAN handleAsyncNotify(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex,
                                    MV_U32 eDmaErrorCause);

static MV_BOOLEAN handleSelfDisable(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex,
                                    MV_U32 eDmaErrorCause);

static MV_BOOLEAN handleDevErr(MV_SATA_ADAPTER *pAdapter,
                               MV_U8 channelIndex,
                               MV_U32 eDmaErrorCause);

#ifdef MV_SATA_C2C_COMM
static void handleC2CInterrupt(MV_SATA_CHANNEL *pSataChannel);
#endif

static void handlePIOInterrupt(MV_SATA_CHANNEL *pSataChannel,
                               MV_QUEUED_COMMAND_ENTRY *pCommandEntry);

static MV_BOOLEAN transferPIOData(MV_SATA_CHANNEL *pSataChannel,
                                  MV_NONE_UDMA_COMMAND_PARAMS   *pNoneUdmaCommandParams);

static void completePIOCommand(MV_SATA_CHANNEL *pSataChannel,
                               MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                               MV_BOOLEAN failed);

static MV_BOOLEAN _resetEdmaQPointers(MV_SATA_CHANNEL *pSataChannel);

static MV_BOOLEAN resetEdmaChannel(MV_SATA_CHANNEL *pSataChannel);

static void flushDmaQueue(MV_SATA_CHANNEL *pSataChannel,
                          MV_FLUSH_TYPE flushType, MV_COMPLETION_TYPE, MV_U16);

static void _fixPhyParams(MV_SATA_ADAPTER *pMvSataAdapter, MV_U8 channelIndex);

static void _channelHardReset(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void _establishSataComm(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void _establishSataCommAll(MV_SATA_ADAPTER *pAdapter);

void _setActivePMPort(MV_SATA_CHANNEL *pSataChannel, MV_U8 PMPort);

static void revertSataHCRegs (MV_SATA_ADAPTER *pAdapter);

static void revertFlashInterfaceRegs (MV_SATA_ADAPTER *pAdapter);

static void revertPCIInterfaceRegs (MV_SATA_ADAPTER *pAdapter);

static void commandsQueueAddTail(MV_SATA_CHANNEL *pSataChannel,
                                 MV_QUEUED_COMMAND_ENTRY *pCommandEntry);

static void commandsQueueRemove(MV_SATA_CHANNEL *pSataChannel,
                                MV_QUEUED_COMMAND_ENTRY *pCommandEntry);

static void addCommand(MV_SATA_CHANNEL *pSataChannel,
                       MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                       MV_QUEUE_COMMAND_INFO *pCommandInfo);

static void removeCommand(MV_SATA_CHANNEL *pSataChannel,
                          MV_QUEUED_COMMAND_ENTRY *pCommandEntry);

static void enableSaDevInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

void disableSaDevInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void _checkATAStatus(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void activateEdma(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void deactivateEdma(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex);

static void EdmaReqQueueInsert(MV_SATA_CHANNEL *pSataChannel,
                               MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                               MV_UDMA_COMMAND_PARAMS  *pUdmaParams);

static MV_BOOLEAN sendNoneUdmaCommand(MV_SATA_CHANNEL *pSataChannel,
                                      MV_QUEUED_COMMAND_ENTRY *pCommandEntry);

static MV_BOOLEAN isGoodCompletionsExpected(MV_SATA_CHANNEL *pSataChannel);

static MV_VOID updatePortsWithErrors(MV_SATA_CHANNEL *pSataChannel);

static MV_VOID enterRequestSenseState(MV_SATA_CHANNEL *pSataChannel);

static MV_BOOLEAN parseReadLogExtOutPut(MV_SATA_CHANNEL *pSataChannel);

static MV_BOOLEAN
ReadLogExtCompletionCB(MV_SATA_ADAPTER *pSataAdapter,
                       MV_U8 channelNum,
                       MV_COMPLETION_TYPE comp_type,
                       MV_VOID_PTR commandId,
                       MV_U16 responseFlags,
                       MV_U32 timeStamp,
                       MV_STORAGE_DEVICE_REGISTERS *registerStruct);

static MV_VOID setReadLogExtCmndPointers(MV_SATA_CHANNEL *pSataChannel);

static MV_VOID insertReadLogExtCmnd(MV_SATA_CHANNEL *pSataChannel);

static MV_VOID handlePortNCQError(MV_SATA_CHANNEL *pSataChannel);

static MV_VOID _insertQCommandsIntoEdma(MV_SATA_CHANNEL *pSataChannel);

static MV_BOOLEAN _doDevErrorRecovery(MV_SATA_CHANNEL *pSataChannel);

#ifdef MV_SATA_C2C_COMM

/* Channel 2 Channel */
static MV_BOOLEAN sendVendorUniqueFIS(MV_SATA_ADAPTER *pAdapter,
                                      MV_U8 channelIndex,
                                      MV_U32 *vendorUniqueBuffer,
                                      MV_U8 numOfDWords);

static void activateBMDmaMode(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                              MV_U32 prdTableHi, MV_U32 prdTableLow,
                              MV_UDMA_TYPE dmaType);
#endif

#ifdef MV_SATA_IO_GRANULARITY

static void setIoGranularityCount(MV_SATA_ADAPTER *pAdapter,
                                  MV_U8 transId,
                                  MV_U8 counter);
static MV_U8 readIoGranularityCount(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 transId);
static void iogInterrupt(MV_SATA_ADAPTER *pAdapter,
                         MV_BUS_ADDR_T ioBaseAddr,
                         MV_U32 mainCause);
static void checkIogCompletion(MV_SATA_ADAPTER *pAdapter,
                               MV_U32 iogCause, MV_U8 offset);
static void checkIogBit(MV_SATA_ADAPTER *pAdapter,
                        MV_U8 bitOffset,
                        MV_U8 value);

static MV_BOOLEAN iogReset(MV_SATA_ADAPTER *pAdapter);
#endif


/* Calculate the base address of the registers for a SATA channel */
MV_U32 edmaRegOffst[MV_SATA_CHANNELS_NUM] = {0x22000, 0x24000, 0x26000, 0x28000,
    0x32000, 0x34000, 0x36000, 0x38000};

#define getEdmaRegOffset(x) edmaRegOffst[(x)]




MV_BOOLEAN waitForBusyAfterHReset(MV_SATA_ADAPTER *pAdapter,
                                  MV_U8 channelIndex)
{
    MV_U32 i;
    MV_U8       ATAstatus;
    mvMicroSecondsDelay(pAdapter, MV_HARD_RESET_WAIT_READY);
    for (i = MV_HARD_RESET_WAIT_READY; i < 5000000; i+= 10000)
    {
        ATAstatus = MV_REG_READ_BYTE(pAdapter->adapterIoBaseAddress,
                                     getEdmaRegOffset(channelIndex) +
                                     MV_ATA_DEVICE_STATUS_REG_OFFSET);
        if ((ATAstatus & MV_ATA_BUSY_STATUS) == 0)
        {
            return MV_TRUE;
        }
        mvMicroSecondsDelay(pAdapter, 10000);
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: in Channel Hard Rese"
             "t wait for busy, ATA STATUS=0x%02x\n", pAdapter->adapterId,
             channelIndex, ATAstatus);
    return MV_FALSE;
}

static void unmaskEdmaInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       getEdmaRegOffset(channelIndex) +
                       MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET,
                       0);
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        /* Unmask EDMA self disable (bit 8), mask errors that cause self disable */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           getEdmaRegOffset(channelIndex) +
                           MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET,
                           MV_EDMA_GEN_I_ERROR_MASK);
    }
    else
    {
        /* Unmask EDMA self disable (bit 7), mask errors that cause self disable */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           getEdmaRegOffset(channelIndex) +
                           MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET,
                           MV_EDMA_GEN_II_ERROR_MASK);
    }
}

static void maskEdmaInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       getEdmaRegOffset(                           channelIndex)                            +
                       MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET,
                       0);
}

/*******************************************************************************
* writeEdmaRequestEntry  - Write a CRQB (COMMAND REQUEST QUEUE BLOCK)
*
* DESCRIPTION:
*       write one CRQB for an EDMA request queue.
*
* INPUT:
*       pReqEntry     - pointer to the CRQB area on the system memory
*                       (HW reqeust Queue).
*       mvSataChannel - pointer to the channel data structure
*       pCommandEntry - pointer to the command entry data structure
*                       (SW request Queue).
*
* RETURN:
*       None
*
* COMMENTS:
*       None.
*
*******************************************************************************/
static void writeEdmaRequestEntry(MV_DMA_REQUEST_QUEUE_ENTRY *pReqEntry,
                                  MV_SATA_CHANNEL *mvSataChannel,
                                  MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                                  MV_UDMA_COMMAND_PARAMS  *pUdmaParams)
{
    MV_U16           ControlFlags = 0;
    volatile MV_U16  *pCommand = &pReqEntry->command[0];
    MV_U8            ATACommand = 0;

    pReqEntry->prdLowAddr  = MV_CPU_TO_LE32(pUdmaParams->prdLowAddr);
    pReqEntry->prdHighAddr = MV_CPU_TO_LE32(pUdmaParams->prdHighAddr);

    /* Set the direction of the transaction (read/write) */
    if (pUdmaParams->readWrite == MV_UDMA_TYPE_READ)
    {
        ControlFlags |= 0x1; /* Device to system memory */
    }
    ControlFlags |= (pCommandEntry->commandTag << 1);   /* the tag will be used also */
#ifdef MV_SATA_IO_GRANULARITY

    /*If valid IO Granularity transaction Id*/
    if (pUdmaParams->iogCurrentTransId < MV_IOG_INVALID_COMMAND_ID)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG | MV_DEBUG_UDMA_COMMAND, "%d %d: "
                 "Edma request with IO granularity Id = 0x%x\n",
                 mvSataChannel->mvSataAdapter->adapterId,
                 mvSataChannel->channelNumber, pUdmaParams->iogCurrentTransId);
        ControlFlags |= (((MV_U16)pUdmaParams->iogCurrentTransId) << 6);
    }
#endif
    /* in Non-queue EDMA mode    */
    ControlFlags |= (pCommandEntry->commandInfo.PMPort << 12);

    pReqEntry->controlFlags =  MV_CPU_TO_LE16(ControlFlags);

    if ((mvSataChannel->queuedDMA == MV_EDMA_MODE_QUEUED) ||
        (mvSataChannel->queuedDMA == MV_EDMA_MODE_NATIVE_QUEUING))
    {
        if (pUdmaParams->isEXT == MV_TRUE) /* Read/Write DMA QUEUED EXT */
        {
            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->numOfSectors & 0xFF00) >> 8,
                                  MV_EDMA_ATA_FEATURES_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->numOfSectors) & 0xFF,
                                  MV_EDMA_ATA_FEATURES_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pCommandEntry->commandTag << 3) & 0xF8,
                                  MV_EDMA_ATA_SECTOR_COUNT_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF000000)
                                  >> 24, MV_EDMA_ATA_LBA_LOW_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress) & 0xFF,
                                  MV_EDMA_ATA_LBA_LOW_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,(pUdmaParams->highLBAAddress &
                                              0xFF),
                                  MV_EDMA_ATA_LBA_MID_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF00) >> 8,
                                  MV_EDMA_ATA_LBA_MID_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->highLBAAddress & 0xFF00) >> 8,
                                  MV_EDMA_ATA_LBA_HIGH_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF0000) >>
                                  16, MV_EDMA_ATA_LBA_HIGH_ADDR, 0);

            if ((mvSataChannel->queuedDMA == MV_EDMA_MODE_NATIVE_QUEUING) &&
                (pUdmaParams->FUA == MV_TRUE))
            {
                WRITE_ATA_COMMAND_REG(pCommand++, MV_BIT7 | MV_BIT6 ,
                                      MV_EDMA_ATA_DEVICE_ADDR,0);
            }
            else
            {
                WRITE_ATA_COMMAND_REG(pCommand++, MV_BIT6 ,
                                      MV_EDMA_ATA_DEVICE_ADDR,0);
            }

            if (pUdmaParams->readWrite == MV_UDMA_TYPE_READ)
            {
                if (mvSataChannel->queuedDMA == MV_EDMA_MODE_NATIVE_QUEUING)
                {
                    ATACommand = MV_ATA_COMMAND_READ_FPDMA_QUEUED_EXT;
                }
                else
                {
                    ATACommand = MV_ATA_COMMAND_READ_DMA_QUEUED_EXT;
                }
            }
            else
            {
                if (mvSataChannel->queuedDMA == MV_EDMA_MODE_NATIVE_QUEUING)
                {
                    ATACommand = MV_ATA_COMMAND_WRITE_FPDMA_QUEUED_EXT;
                }
                else
                {
                    ATACommand = MV_ATA_COMMAND_WRITE_DMA_QUEUED_EXT;
                }
            }
        }
        else /* Read/Write DMA QUEUED */
        {
            WRITE_ATA_COMMAND_REG(pCommand++, (pUdmaParams->numOfSectors) &
                                  0xFF, MV_EDMA_ATA_FEATURES_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pCommandEntry->commandTag << 3) & 0xF8,
                                  MV_EDMA_ATA_SECTOR_COUNT_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress) & 0xFF,
                                  MV_EDMA_ATA_LBA_LOW_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF00) >> 8,
                                  MV_EDMA_ATA_LBA_MID_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF0000)
                                  >> 16, MV_EDMA_ATA_LBA_HIGH_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++, MV_BIT6 |
                                  (MV_U8)((pUdmaParams->lowLBAAddress & 0xF000000)
                                          >> 24), MV_EDMA_ATA_DEVICE_ADDR, 0);

            if (pUdmaParams->readWrite == MV_UDMA_TYPE_READ)
            {
                ATACommand = MV_ATA_COMMAND_READ_DMA_QUEUED;
            }
            else
            {
                ATACommand = MV_ATA_COMMAND_WRITE_DMA_QUEUED;
            }
        }
    }
    else
    {
        if (pUdmaParams->isEXT == MV_TRUE)
        {   /* READ/WRITE DMA EXT */
            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->numOfSectors & 0xFF00) >> 8,
                                  MV_EDMA_ATA_SECTOR_COUNT_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->numOfSectors) & 0xFF,
                                  MV_EDMA_ATA_SECTOR_COUNT_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF000000)
                                  >> 24,
                                  MV_EDMA_ATA_LBA_LOW_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress) &0xFF,
                                  MV_EDMA_ATA_LBA_LOW_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->highLBAAddress & 0xFF),
                                  MV_EDMA_ATA_LBA_MID_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF00) >> 8,
                                  MV_EDMA_ATA_LBA_MID_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->highLBAAddress & 0xFF00) >> 8,
                                  MV_EDMA_ATA_LBA_HIGH_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF0000)
                                  >> 16,
                                  MV_EDMA_ATA_LBA_HIGH_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++, MV_BIT6, MV_EDMA_ATA_DEVICE_ADDR,
                                  0);

            if (pUdmaParams->readWrite == MV_UDMA_TYPE_READ)
            {
                ATACommand = MV_ATA_COMMAND_READ_DMA_EXT;
            }
            else
            {
                ATACommand = MV_ATA_COMMAND_WRITE_DMA_EXT;
            }
        }
        else /* READ/WRITE DMA */
        {
            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->numOfSectors) & 0xFF,
                                  MV_EDMA_ATA_SECTOR_COUNT_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress) & 0xFF,
                                  MV_EDMA_ATA_LBA_LOW_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF00) >> 8,
                                  MV_EDMA_ATA_LBA_MID_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  (pUdmaParams->lowLBAAddress & 0xFF0000)
                                  >> 16,
                                  MV_EDMA_ATA_LBA_HIGH_ADDR, 0);

            WRITE_ATA_COMMAND_REG(pCommand++,
                                  MV_BIT6 | (MV_U8)((pUdmaParams->lowLBAAddress &
                                                     0xF000000) >> 24),
                                  MV_EDMA_ATA_DEVICE_ADDR, 0);

            if (pUdmaParams->readWrite == MV_UDMA_TYPE_READ)
            {
                ATACommand = MV_ATA_COMMAND_READ_DMA;
            }
            else
            {
                ATACommand = MV_ATA_COMMAND_WRITE_DMA;
            }
        }
    }
    WRITE_ATA_COMMAND_REG(pCommand++, ATACommand, MV_EDMA_ATA_COMMAND_ADDR,
                          MV_BIT15);
}

/*******************************************************************************
* handleEdmaFailedCommand - Handle failed EDMA command which didn't commpleted.
*
* DESCRIPTION:
*       This function handles the completion of failed EDMA command when no
*       response received for that command.
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX50XX adapter data structure.
*       channelIndex - The index of the channel where the response received.
*       eDmaErrorCause - the value of the channel EDMA error cause register.
*
* RETURN:
*       None
*
* COMMENTS:
*       This function assumes that the channel semaphore is locked.
*
*******************************************************************************/
static void handleEdmaFailedCommand(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex, MV_U16 eDmaErrorCause)
{
    MV_QUEUED_COMMAND_ENTRY       *pCommandEntry;
    MV_UDMA_COMMAND_PARAMS        *pUdmaCommandParams;
    MV_SATA_CHANNEL               *pSataChannel = pAdapter->sataChannel[channelIndex];
    MV_STORAGE_DEVICE_REGISTERS   deviceRegs;
    MV_U32      eDmaStatus;
    MV_U32      commandTag;

    eDmaStatus = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   pSataChannel->eDmaRegsOffset +
                                   MV_EDMA_STATUS_REG_OFFSET);

    commandTag = (eDmaStatus & MV_EDMA_STATUS_TAG_MASK) >>
                 MV_EDMA_STATUS_TAG_OFFSET;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Handle failed command,"
             "tag 0x%02x, error cause 0x%02x\n", pAdapter->adapterId,
             channelIndex, commandTag, eDmaErrorCause);

    pCommandEntry = &(pSataChannel->commandsQueue[commandTag]);
    if (pCommandEntry->isFreeEntry == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "%d %d: Received response on a non"
                 "-valid tag (%x)\n", pAdapter->adapterId, channelIndex,
                 commandTag);

        _dumpSataRegs(pAdapter,channelIndex);
        dumpAtaDeviceRegisters(pAdapter, channelIndex, MV_TRUE, &deviceRegs);
        _printATARegs(&deviceRegs);
        return;
    }
    if (pSataChannel->PMSupported == MV_TRUE)
    {
        _setActivePMPort(pSataChannel, pCommandEntry->commandInfo.PMPort);
    }
    pUdmaCommandParams = &pCommandEntry->commandInfo.commandParams.udmaCommand;
    dumpAtaDeviceRegisters(pAdapter, channelIndex, pUdmaCommandParams->isEXT,
                           &deviceRegs);

    pSataChannel->EdmaQueuedCommands--;
    pUdmaCommandParams->callBack(pSataChannel->mvSataAdapter, channelIndex,
                                 MV_COMPLETION_TYPE_ERROR,
                                 pUdmaCommandParams->commandId, eDmaErrorCause,
                                 0, &deviceRegs);
    removeCommand(pSataChannel,pCommandEntry);
}

/*******************************************************************************
* handleEdmaResponse - Handle an EDMA response queue entry.
*
* DESCRIPTION:
*       This function handles the completion of EDMA command when a response
*       entry is received.
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX50XX adapter data structure.
*       channelIndex - The index of the channel where the response received.
*       response     - Pointer to the received EDMA response block structure.
*
* RETURN:
*       None
*
* COMMENTS:
*       This function assumes that the channel semaphore is locked.
*
*******************************************************************************/
static void handleEdmaResponse(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                               MV_DMA_RESPONSE_QUEUE_ENTRY *eDmaResponse)
{
    MV_QUEUED_COMMAND_ENTRY       *pCommandEntry;
    MV_UDMA_COMMAND_PARAMS        *pUdmaCommandParams;
    MV_STORAGE_DEVICE_REGISTERS   deviceRegs;
    MV_COMPLETION_TYPE            compType = MV_COMPLETION_TYPE_NORMAL;
    MV_SATA_CHANNEL         *pSataChannel = pAdapter->sataChannel[channelIndex];
    MV_DMA_RESPONSE_QUEUE_ENTRY   response;
    MV_U16                        eDmaCause = 0;

    response.commandTag    = MV_LE16_TO_CPU(eDmaResponse->commandTag);
    response.responseFlags = MV_LE16_TO_CPU(eDmaResponse->responseFlags);
    response.timeStamp     = MV_LE32_TO_CPU(eDmaResponse->timeStamp);

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, "%d %d: New Response Received. ptr %p, "
             "tag 0x%02x, flags 0x%x ts 0x%08x\n", pAdapter->adapterId,
             channelIndex, eDmaResponse, response.commandTag,
             response.responseFlags, response.timeStamp);

    pCommandEntry = &(pSataChannel->commandsQueue[response.commandTag & 0x1f]);
    if (response.responseFlags & 0xff)  /* response with errors */
    {
        pSataChannel->queueCommandsEnabled = MV_FALSE;
        pSataChannel->EdmaActive = MV_FALSE;
        compType = MV_COMPLETION_TYPE_ERROR;

        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Response with Error. "
                 "outstanding commands %d, response flags 0x%x\n",
                 pAdapter->adapterId, channelIndex,
                 pSataChannel->outstandingCommands, response.responseFlags);

        /* in NCQ mode no responses are completed with error*/
        if (pSataChannel->queuedDMA == MV_EDMA_MODE_NATIVE_QUEUING)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "%d %d: Response "
                     "with Error in NCQ mode !!!!\n",
                     pAdapter->adapterId, channelIndex);
            return;
        }
        /*
         * link & phy layers unrecoverable errors may be the reason for a
         * device errors, so we first check if any unrecoverable errors occured,
         * except PCI/internal parity, if yes then we don't count this response
         * the PCI/internal parity errors excluded since we want to complete
         * the commands with error indication so higher layers can receive it
         */
        {
            MV_U32  edmaErrorCause = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                                       pSataChannel->eDmaRegsOffset +
                                                       MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET);
            MV_U32  unrecoverableErrorMask;
            if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
            {
                unrecoverableErrorMask = MV_EDMA_GEN_I_UNRECOVERABLE_EDMA_ERROR;
            }
            else
            {
                unrecoverableErrorMask = MV_EDMA_GEN_II_UNRECOVERABLE_EDMA_ERROR;
            }
            unrecoverableErrorMask &= ~(MV_BIT1|MV_BIT0);
            if (edmaErrorCause & unrecoverableErrorMask)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Response"
                         " ignored due to unrecoverable error, EDMA Error Cause"
                         ": 0x%08x\n", pAdapter->adapterId, channelIndex,
                         edmaErrorCause);
                return;
            }
        }

        /*
         * responseFlags will hold the low 8 bit of the EDMA error cause
         * regiter. For 88SX50XX set bit 8 sence each error causes to
         * eDmaSelfDisable.
         */
        eDmaCause = (response.responseFlags & 0xff);
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
        {
            eDmaCause |= MV_BIT8;
        }
        else
        {
            eDmaCause |= MV_BIT7;
        }
        if (pSataChannel->PMSupported == MV_TRUE)
        {
            _setActivePMPort(pSataChannel, pCommandEntry->commandInfo.PMPort);
        }
    }
    pUdmaCommandParams = &pCommandEntry->commandInfo.commandParams.udmaCommand;
    if (response.responseFlags & MV_BIT2) /*device error */
    {
        dumpAtaDeviceRegisters(pAdapter, channelIndex, pUdmaCommandParams->isEXT,
                               &deviceRegs);
    }

    if (pCommandEntry->isFreeEntry == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "%d %d: Received response on a non"
                 "-valid tag (%x), ts 0x%08x, address %p\n", pAdapter->adapterId, channelIndex,
                 response.commandTag, response.timeStamp, eDmaResponse);
        _dumpEDMARegs(pAdapter, channelIndex);
        _dumpChannelQueues(pAdapter, channelIndex);
    }
    else
    {
        pSataChannel->EdmaQueuedCommands--;

        pUdmaCommandParams->callBack(pSataChannel->mvSataAdapter, channelIndex,
                                     compType, pUdmaCommandParams->commandId,
                                     eDmaCause, response.timeStamp, &deviceRegs);
        removeCommand(pSataChannel,pCommandEntry);
        if (pSataChannel->queueCommandsEnabled == MV_FALSE)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_UDMA_COMMAND,
                     "%d %d: Commands queuing is disabled\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber);
            return;
        }
        if (pSataChannel->commandsQueueHead == NULL)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_UDMA_COMMAND,
                     "%d %d: Commands queue is empty\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber);
            return;
        }
        if (pSataChannel->NCQErrHandlingInfo.state != MV_NCQ_ERROR_HANDLING_STATE_IDLE)
        {
            if (isGoodCompletionsExpected(pSataChannel) == MV_FALSE)
            {
                enterRequestSenseState(pSataChannel);
                return;
            }
        }
        if (pSataChannel->commandsQueueHead->commandInfo.type ==
            MV_QUEUED_COMMAND_TYPE_NONE_UDMA)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_UDMA_COMMAND,
                     "%d %d: Next Command is PIO\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber);
            deactivateEdma(pAdapter,channelIndex);
            if (pSataChannel->PMSupported == MV_TRUE)
            {
                _setActivePMPort(pSataChannel,
                                 pSataChannel->commandsQueueHead->commandInfo.PMPort);
            }
            if (sendNoneUdmaCommand(pSataChannel,
                                    pSataChannel->commandsQueueHead) == MV_FALSE)
            {
                completePIOCommand(pSataChannel, pSataChannel->commandsQueueHead,
                                   MV_TRUE);
            }

        }
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_UDMA_COMMAND,
                     "%d %d: Next Command is UDMA nothing to do\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber);

        }
    }
}


#ifdef MV_SATA_C2C_COMM
/*******************************************************************************
* handleBmDmaInterrupt - handle DMA interrupt received for a given channel
*
* DESCRIPTION:
*   This function is called when response interrupt is issued when C2C DMA
*   completion event occurs.
*
* INPUT:
*   pAdapter   - pointer to the MV88SX50XX adapter data structure
*   ioBaseAddr  - adapter rbase address
*   pSataChannel  - SATA channel structure
*   channelIndex  - SATA channel index
*   edmaError   - if != zero then EDMA error happened.
*
* RETURN:
*   None.
*
* COMMENTS:
*   None
*
*******************************************************************************/

static void handleBmDMAInterrupt(MV_SATA_ADAPTER *pAdapter,
                                 MV_BUS_ADDR_T   ioBaseAddr,
                                 MV_SATA_CHANNEL *pSataChannel,
                                 MV_U8   channelIndex,
                                 MV_U32 edmaError)
{
    MV_U32  val;
    MV_U32  eDmaErrorCause = 0;
    mvOsSemTake(&pSataChannel->semaphore);
    /*Reset BM dma*/
    val = MV_REG_READ_DWORD (ioBaseAddr,
                             getEdmaRegOffset(channelIndex) +
                             MV_BMDMA_COMMAND_OFFSET);
    val &= ~MV_BIT0;
    MV_REG_WRITE_DWORD (ioBaseAddr,
                        getEdmaRegOffset(channelIndex) +
                        MV_BMDMA_COMMAND_OFFSET, val);

    MV_REG_WRITE_DWORD(ioBaseAddr,
                       getEdmaRegOffset(channelIndex) +
                       MV_SATA_II_IF_CONTROL_REG_OFFSET,
                       0);
    if (edmaError)
    {
        eDmaErrorCause = (MV_U16)MV_REG_READ_DWORD(ioBaseAddr,
                                                   getEdmaRegOffset(channelIndex) +
                                                   MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET);
    }
    mvOsSemRelease(&pSataChannel->semaphore);
    if (pSataChannel->C2CCallback)
    {
        if (!edmaError)
        {
            pSataChannel->C2CCallback(pAdapter,
                                      pSataChannel,
                                      MV_C2C_BM_DMA_DONE,
                                      0,
                                      NULL);
        }
        else
        {
            if (eDmaErrorCause & (MV_BIT17 | MV_BIT26))
            {
                pSataChannel->C2CCallback(pAdapter,
                                          pSataChannel,
                                          MV_C2C_BM_DMA_ERROR,
                                          0,
                                          NULL);
            }
            if (eDmaErrorCause & (MV_BIT13 | MV_BIT21))
            {
                pSataChannel->C2CCallback(pAdapter,
                                          pSataChannel,
                                          MV_C2C_REGISTER_DEVICE_TO_HOST_FIS_ERROR,
                                          0,
                                          NULL);
            }
        }
    }
}
#endif


/*******************************************************************************
* handleEdmaInterrupt - handle EDMA interrupt receivd for a given channel
*
* DESCRIPTION:
*   this function called when response interrupt issuesed for a channel and it
*    handles all EDMA responses.
*
* INPUT:
*   *pAdapter   - pointer to the MV88SX50XX adapter data structure
*   sataUnit    - the SATAHC unit this channel belongs to
*   port        - the port number of the channel
*   rspInPtr    - the value of eRPQIP of the channel
*   responseDone   - if != zero then responses received on this channel
*   edmaError   - if != zero then EDMA error happened.
*
* RETURN:
*   None.
*
* COMMENTS:
*   None.
*
*******************************************************************************/
static void handleEdmaInterrupt(MV_SATA_ADAPTER *pAdapter, MV_U8 sataUnit,
                                MV_U8 port,MV_U32 rspInPtr, MV_U32 responseDone,
                                MV_U32 edmaError, MV_U32 unitCause)
{
    MV_U32  rspOutPtr;
    MV_U8   channelIndex;
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;
    MV_U32  eDmaRegsOffset;
    MV_SATA_CHANNEL *pSataChannel;
    MV_BOOLEAN responseWithErr = MV_FALSE;

    channelIndex = MV_CHANNEL_INDEX(sataUnit, port);
    pSataChannel = pAdapter->sataChannel[channelIndex];


    if (responseDone && (pSataChannel != NULL))/* port Done*/
    {
        mvOsSemTake(&pSataChannel->semaphore);
        pSataChannel->recoveredErrorsCounter = 0;
#ifdef MV_SATA_C2C_COMM
        if (MV_FALSE == pSataChannel->C2CmodeEnabled)
        {
#endif
            eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

            rspOutPtr = pSataChannel->rspOutPtr;
            /* here we should update the response out pointer though we didn't*/
            /* handled the new responses, these response entries will not be  */
            /* accessed again by the EDMA sinse the number of queued commands */
            /* (outstandingCommands) will be updated only after we handle each*/
            /* response entry                                                 */
            MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                               MV_EDMA_RESPONSE_Q_OUTP_REG_OFFSET,
                               pSataChannel->responseQueuePciLowAddress |
                               (rspInPtr << MV_EDMA_RESPONSE_Q_OUTP_OFFSET));
            while (rspOutPtr != rspInPtr)
            {
                handleEdmaResponse(pAdapter, channelIndex,
                                   &(pSataChannel->responseQueue[rspOutPtr]));
                rspOutPtr++;
                rspOutPtr &= MV_EDMA_QUEUE_MASK;
            }
            /*
             * Check if queueCommandsEnabled flag is disabled.
             * If so, then an error has occured and auto flush must be triggered.
             * Basically it is enough to trigger auto flush upon edmaError flag,
             * but since edmaError is set before handleEdmaResponse is called
             * there could be a racing condition between the time edmaError is checked
             * and the response queue is checked.
             * The racing condition is that an error does not occur when setting
             * edmaError to MV_FALSE, but in handlEdmaResponse, the hardware
             * has completed a command with error.
             * The racing condition will complete the error command (through callback)
             * but will prevent the auto flush of all outstanding commands.
             */
            if (pSataChannel->queueCommandsEnabled == MV_FALSE)
            {
                responseWithErr = MV_TRUE;
            }

            pSataChannel->rspOutPtr = rspOutPtr;
            mvOsSemRelease(&pSataChannel->semaphore);
#ifdef MV_SATA_C2C_COMM
        }
        else
        {
            mvOsSemRelease(&pSataChannel->semaphore);
            handleBmDMAInterrupt(pAdapter,
                                 ioBaseAddr,
                                 pSataChannel,
                                 channelIndex,
                                 edmaError);
        }
#endif
    }

    if ((edmaError) ||  (responseWithErr == MV_TRUE))   /* EDMA error interrupt*/
    {
        handleEdmaError(pAdapter,channelIndex );
    }
}

static void handleEdmaError(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_U32 eDmaErrorCause = 0;
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;

    pSataChannel = pAdapter->sataChannel[channelIndex];

    eDmaErrorCause = MV_REG_READ_DWORD(ioBaseAddr,
                                       getEdmaRegOffset(channelIndex) +
                                       MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET);

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%d %d: Edma Error Reg 0x%x\n", pAdapter->adapterId,
             channelIndex, MV_REG_READ_DWORD(ioBaseAddr,
                                             getEdmaRegOffset(channelIndex) +
                                             MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET));
    /* clear the channel's error cause register */
    MV_REG_WRITE_DWORD(ioBaseAddr,
                       getEdmaRegOffset(channelIndex) +
                       MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET,
                       ~eDmaErrorCause);
    /*if PM connected, connect/disconnect interrupts storm could happen*/
    if (MV_REG_READ_DWORD(ioBaseAddr,
                          getEdmaRegOffset(channelIndex) +
                          MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET) &
                          (MV_BIT3 & MV_BIT4))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 "%d %d: Edma Error Reg 0x%x still set!!!!!!!!\n",
                 pAdapter->adapterId, channelIndex,
                 MV_REG_READ_DWORD(ioBaseAddr,
                                   getEdmaRegOffset(channelIndex) +
                                   MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET));
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            /*wait 20ms till diconnect/connect interrupts finish*/
            mvMicroSecondsDelay(pAdapter, 20000);
            eDmaErrorCause |= MV_REG_READ_DWORD(ioBaseAddr,
                                                getEdmaRegOffset(channelIndex) +
                                                MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET);
            MV_REG_WRITE_DWORD(ioBaseAddr,
                               getEdmaRegOffset(channelIndex) +
                               MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET,
                               ~eDmaErrorCause);
        }
    }
    /* dump in case any kind of parity error*/
    if (eDmaErrorCause & (MV_BIT11 | MV_BIT10 | MV_BIT9 | MV_BIT1 | MV_BIT0))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR," PARITY ERROR Detected\n");
        _dumpPCIRegs(pAdapter);
        _dumpEDMARegs(pAdapter,channelIndex);
        _dumpChannelQueues(pAdapter,channelIndex);
    }
    if (eDmaErrorCause & MV_BIT3) /*device disconneted*/
    {
        handleDisconnect(pAdapter, channelIndex);
        /* continue only if we have also a connect interrupt*/
        if ((eDmaErrorCause & MV_BIT4) == 0)
        {
            return;
        }
    }
    if (eDmaErrorCause & MV_BIT4) /*device conneted*/
    {
        handleConnect(pAdapter, channelIndex);
        return;
    }
    /* unrecoverable error*/
    if (handleUnrecoverableError(pAdapter,channelIndex ,eDmaErrorCause ) == MV_TRUE)
    {
        return;
    }

    /*PM hot plug*/
    if (handleAsyncNotify(pAdapter,channelIndex ,eDmaErrorCause ) == MV_TRUE)
    {
        return;
    }
    /* device errors in none NCQ mode generate self disable interrupt*/
    if (handleSelfDisable(pAdapter,channelIndex ,eDmaErrorCause ) == MV_TRUE)
    {
        return;
    }
    /* Neither device error without completion nor self disable must be in NCQ
    *  mode and Port multiplier connected
    */
    if (handleDevErr(pAdapter,channelIndex ,eDmaErrorCause ) == MV_TRUE)
    {
        return;
    }
    /* recoverable error*/
    if (handleRecoverableError(pAdapter,channelIndex ,eDmaErrorCause) == MV_TRUE)
    {
        return;
    }

    /*this point should not be reached*/
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_INTERRUPTS,
             "%d %d: Error in handling EDMA error\n",
             pAdapter->adapterId, channelIndex);


}
static MV_VOID handleDisconnect(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];

    if (pSataChannel)
    {
        mvOsSemTake(&pSataChannel->semaphore);
        pSataChannel->queueCommandsEnabled = MV_FALSE;
        pSataChannel->EdmaActive = MV_FALSE;
    }
    /* If disk is disconnected, then disable the activity LED */
    if (pAdapter->chipIs50XXB2 == MV_TRUE)
    {
        MV_U32 regVal1, regVal2;
        /* First enable flash controller clocks*/
        regVal1 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_PCI_REGS_OFFSET +
                                    MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET);

        regVal1 |= (MV_BIT0);
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_PCI_REGS_OFFSET + MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET,
                           regVal1);
        regVal1 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_PCI_REGS_OFFSET +
                                    MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET);
        /* Disable activity LEDs */
        regVal2 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_FLASH_GPIO_PORT_CONTROL_OFFSET);
        regVal2 |= (MV_BIT8 << channelIndex);
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            MV_FLASH_GPIO_PORT_CONTROL_OFFSET,
                            regVal2);
        regVal2 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_FLASH_GPIO_PORT_CONTROL_OFFSET);
        /* Disable flash controller clocks */
        regVal1 &= ~(MV_BIT0);
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_PCI_REGS_OFFSET + MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET,
                           regVal1);
    }
    /*SHI2*/
    if ((pAdapter->chipIs50XXB0 == MV_TRUE)||
        (pAdapter->chipIs50XXB2 == MV_TRUE))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS,
                 "%d %d: Before Hard RESET Main Cause %x\n",
                 pAdapter->adapterId, channelIndex,
                 MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET));
        /* Hard Reset the channel so we can do re-connect*/
        _channelHardReset(pAdapter, channelIndex);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS,
                 "%d %d: After Hard RESET Main Cause %x\n",
                 pAdapter->adapterId, channelIndex,
                 MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET));
    }
    else
    {
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            _channelHardReset(pAdapter, channelIndex);
            _establishSataComm(pAdapter, channelIndex);
        }
    }
    /* after calling mvSataNotify we can not be sure that the channel*/
    /* data structure is still available so first we release the     */
    /* semaphore, after notifying the upper-layer with the disconnect*/
    /* event, nothing else is done with that channel                 */
    if (pSataChannel)
    {
        mvOsSemRelease(&pSataChannel->semaphore);
    }
    if (mvSataIsStorageDeviceConnected(pAdapter,channelIndex) == MV_FALSE)
    {
        pAdapter->mvSataEventNotify(pAdapter,
                                    MV_EVENT_TYPE_SATA_CABLE,
                                    MV_SATA_CABLE_EVENT_DISCONNECT,
                                    channelIndex);
    }

}
static MV_VOID handleConnect(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    /*SHI2*/
    if ((pAdapter->chipIs50XXB0 == MV_TRUE) ||
        (pAdapter->chipIs50XXB2 == MV_TRUE))
    {
        _fixPhyParams(pAdapter, channelIndex);/*TBD*/
        /* The following link re-establishment is due to non    */
        /* Marvell driven hard drives                           */
        _establishSataComm(pAdapter, channelIndex);
        _establishSataComm(pAdapter, channelIndex);
    }

    /* If disk is connected, then enable the activity LED */
    if (pAdapter->chipIs50XXB2 == MV_TRUE)
    {
        MV_U32 regVal1, regVal2;
        /* First enable flash controller clocks*/
        regVal1 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_PCI_REGS_OFFSET +
                                    MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET);

        regVal1 |= (MV_BIT0);
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_PCI_REGS_OFFSET + MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET,
                           regVal1);
        regVal1 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_PCI_REGS_OFFSET +
                                    MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET);
        /* Enable activity LEDs */
        regVal2 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_FLASH_GPIO_PORT_CONTROL_OFFSET);
        regVal2 &= ~(MV_BIT8 << channelIndex);
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            MV_FLASH_GPIO_PORT_CONTROL_OFFSET,
                            regVal2);
        regVal2 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    MV_FLASH_GPIO_PORT_CONTROL_OFFSET);
        /* Disable flash controller clocks */
        regVal1 &= ~(MV_BIT0);
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_PCI_REGS_OFFSET + MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET,
                           regVal1);
    }
    if (mvSataIsStorageDeviceConnected(pAdapter,channelIndex) == MV_TRUE)
    {
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_CABLE,
                                    MV_SATA_CABLE_EVENT_CONNECT,
                                    channelIndex);
    }
}
static MV_BOOLEAN handleUnrecoverableError(MV_SATA_ADAPTER *pAdapter,
                                           MV_U8 channelIndex,
                                           MV_U32 eDmaErrorCause)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];

    if (((pAdapter->sataAdapterGeneration == MV_SATA_GEN_I) &&
         (eDmaErrorCause & MV_EDMA_GEN_I_UNRECOVERABLE_EDMA_ERROR)) ||
        ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) &&
         (eDmaErrorCause & MV_EDMA_GEN_II_UNRECOVERABLE_EDMA_ERROR)))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Unrecoverable"
                 " HW error detected.\n", pAdapter->adapterId, channelIndex);
#ifdef MV_LOGGER
        if (eDmaErrorCause & MV_BIT0)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: ePrtDataErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT1)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: ePrtPRDErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT3)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: eDevDis"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT9)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: ePrtCRQBErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT10)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: ePrtCRPBErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT11)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: ePrtIntErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT12)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: eIORdyErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT15)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: LinkCtlRxErr[2]"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & (MV_BIT17 | MV_BIT18 | MV_BIT19 | MV_BIT20))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: LinkDataRxErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & (MV_BIT26 | MV_BIT27 | MV_BIT28 | MV_BIT29 | MV_BIT30))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: LinkDataTxErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & MV_BIT31)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: TransProtErr"
                     "UnrecoverableHW error detected.\n", pAdapter->adapterId, channelIndex);
        }
#endif
        if (pSataChannel)
        {
            mvOsSemTake(&pSataChannel->semaphore);
            pSataChannel->queueCommandsEnabled = MV_FALSE;
            pSataChannel->EdmaActive = MV_FALSE;
            deactivateEdma(pAdapter, channelIndex);
            disableSaDevInterrupts(pAdapter, channelIndex);
            flushDmaQueue (pSataChannel, MV_FLUSH_TYPE_CALLBACK,
                           MV_COMPLETION_TYPE_ABORT, (MV_U16)eDmaErrorCause);
            resetEdmaChannel(pSataChannel);
            mvOsSemRelease(&pSataChannel->semaphore);
        }
        _dumpSataRegs(pAdapter, channelIndex);
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_ERROR,
                                    MV_SATA_UNRECOVERABLE_COMMUNICATION_ERROR,
                                    channelIndex);
        return MV_TRUE;
    }
    return MV_FALSE;
}
static MV_BOOLEAN handleRecoverableError(MV_SATA_ADAPTER *pAdapter,
                                         MV_U8 channelIndex,
                                         MV_U32 eDmaErrorCause)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];

    if (((pAdapter->sataAdapterGeneration == MV_SATA_GEN_I) &&
         (eDmaErrorCause & MV_EDMA_GEN_I_RECOVERABLE_EDMA_ERROR)) ||
        ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) &&
         (eDmaErrorCause & MV_EDMA_GEN_II_RECOVERABLE_EDMA_ERROR)))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Recoverable"
                 " HW error detected.\n", pAdapter->adapterId, channelIndex);
#ifdef MV_LOGGER
        if (eDmaErrorCause & MV_BIT5)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: SerrInt"
                     "Recoverable error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & (MV_BIT13 | MV_BIT14 | MV_BIT16))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: LinkCtlRxErr"
                     "Recoverable error detected.\n", pAdapter->adapterId, channelIndex);
        }
        if (eDmaErrorCause & (MV_BIT21 | MV_BIT22 | MV_BIT23 | MV_BIT24 | MV_BIT25))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: LinkCtlTxErr"
                     "Recoverable error detected.\n", pAdapter->adapterId, channelIndex);
        }
#endif
        _dumpSataRegs(pAdapter, channelIndex);
        if ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) &&
            (eDmaErrorCause & MV_BIT5))
        {
            MV_U32  regVal;
            regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                       edmaRegOffst[ channelIndex] +
                                       MV_SATA_II_S_ERROR_REG_OFFSET);
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Clear "
                     " Serror register(0x%02x).\n", pAdapter->adapterId,
                     channelIndex, regVal);
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               edmaRegOffst[ channelIndex] +
                               MV_SATA_II_S_ERROR_REG_OFFSET, regVal);
        }
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_ERROR,
                                    MV_SATA_RECOVERABLE_COMMUNICATION_ERROR,
                                    channelIndex);
        if (pSataChannel)
        {
            mvOsSemTake(&pSataChannel->semaphore);
            if (pSataChannel->recoveredErrorsCounter++ > 10)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         "%d %d: Reached %d Recoverable errors"
                         "notify unrecoverable error\n",
                         pAdapter->adapterId, channelIndex,
                         pSataChannel->recoveredErrorsCounter);
                pSataChannel->queueCommandsEnabled = MV_FALSE;
                pSataChannel->EdmaActive = MV_FALSE;
                deactivateEdma(pAdapter, channelIndex);
                disableSaDevInterrupts(pAdapter, channelIndex);
                flushDmaQueue (pSataChannel, MV_FLUSH_TYPE_CALLBACK,
                               MV_COMPLETION_TYPE_ABORT, (MV_U16)eDmaErrorCause);
                resetEdmaChannel(pSataChannel);
                mvOsSemRelease(&pSataChannel->semaphore);
                pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_ERROR,
                                            MV_SATA_UNRECOVERABLE_COMMUNICATION_ERROR,
                                            channelIndex);
            }
            else
            {
                mvOsSemRelease(&pSataChannel->semaphore);
                return MV_TRUE;
            }
        }
        return MV_TRUE;
    }
    return MV_FALSE;
}

static MV_BOOLEAN handleAsyncNotify(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex,
                                    MV_U32 eDmaErrorCause)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];
    if ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) &&
        (eDmaErrorCause & MV_BIT8))
    {
        MV_U32 regVal1;
        if (pSataChannel != NULL)
        {
            mvOsSemTake(&pSataChannel->semaphore);
        }
        regVal1 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                    getEdmaRegOffset(channelIndex) +
                                    MV_SATA_II_IF_STATUS_REG_OFFSET);
        /*Clear status*/
        if (regVal1 & (MV_BIT31 | MV_BIT30))
        {
            MV_U32 regVal2 = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                               getEdmaRegOffset(channelIndex) +
                                               MV_SATA_II_IF_CONTROL_REG_OFFSET);
            regVal2 |= MV_BIT24;
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               getEdmaRegOffset(channelIndex) +
                               MV_SATA_II_IF_CONTROL_REG_OFFSET, regVal2);
        }
        if (pSataChannel != NULL)
        {
            mvOsSemRelease(&pSataChannel->semaphore);
        }
        if (((regVal1 & MV_BIT30) == 0) &&
            ((regVal1 & MV_BIT31) == MV_BIT31))
        {


            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS,
                     "%d %d: PM asynchronous notification interrupt.\n",
                     pAdapter->adapterId, channelIndex);

            pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_CABLE,
                                        MV_SATA_CABLE_EVENT_PM_HOT_PLUG,
                                        channelIndex);
        }
        return MV_TRUE;
    }
    return MV_FALSE;
}

static MV_BOOLEAN handleSelfDisable(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex,
                                    MV_U32 eDmaErrorCause)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];

    if ((((pAdapter->sataAdapterGeneration == MV_SATA_GEN_I) && (eDmaErrorCause & MV_BIT8)) ||
         ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) && (eDmaErrorCause & MV_BIT7)))
        &&
        (pSataChannel != NULL)) /* edma self disable */
    {
        mvOsSemTake(&pSataChannel->semaphore);
        if (pSataChannel->EdmaActive == MV_TRUE)
        {
            pSataChannel->queueCommandsEnabled = MV_FALSE;
            pSataChannel->EdmaActive = MV_FALSE;

            if (eDmaErrorCause & MV_BIT2)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         "%d %d: Edma Self disabled due to device error"
                         " without completion\n", pAdapter->adapterId,
                         channelIndex);
                switch (pSataChannel->queuedDMA)
                {
                case MV_EDMA_MODE_NOT_QUEUED:
                case MV_EDMA_MODE_QUEUED:
                    handleEdmaFailedCommand(pAdapter, channelIndex,
                                            (MV_U16)eDmaErrorCause);
                    break;
                case MV_EDMA_MODE_NATIVE_QUEUING:
                    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR,
                             "%d %d: Edma Self disabled due to device "
                             "error in NCQ mode!!!!\n",
                             pAdapter->adapterId, channelIndex);

                    break;
                default:
                    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR,
                             "%d %d: Unknown EDMA mode (%d)\n",
                             pAdapter->adapterId, channelIndex,
                             pSataChannel->queuedDMA);
                    break;

                }
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_INTERRUPTS,
                         "%d %d: Edma Self disable received without reason!!!\n",
                         pAdapter->adapterId, channelIndex);

            }
        }
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_ERROR,
                                    MV_SATA_DEVICE_ERROR, channelIndex);
        if (_doDevErrorRecovery(pSataChannel) == MV_FALSE)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_INTERRUPTS,
                     "%d %d: Error Recovery Fails!!!\n",
                     pAdapter->adapterId, channelIndex);
            mvOsSemRelease(&pSataChannel->semaphore);
            return MV_TRUE;
        }
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_TRUE;
    }
    return MV_FALSE;
}
static MV_BOOLEAN handleDevErr(MV_SATA_ADAPTER *pAdapter,
                               MV_U8 channelIndex,
                               MV_U32 eDmaErrorCause)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];
    if ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) &&
        (eDmaErrorCause & MV_BIT2) && (pSataChannel != NULL))
    {
        mvOsSemTake(&pSataChannel->semaphore);
        if (pSataChannel->EdmaActive == MV_TRUE)
        {
            if (pSataChannel->queuedDMA != MV_EDMA_MODE_NATIVE_QUEUING)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_INTERRUPTS,
                         "%d %d: EDMA device error must be handled"
                         "previously!!!\n", pAdapter->adapterId,
                         channelIndex);
            }
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                     "%d %d: EDMA device error in NCQ mode\n",
                     pAdapter->adapterId, channelIndex);
            if (pSataChannel->NCQErrHandlingInfo.state == MV_NCQ_ERROR_HANDLING_STATE_IDLE)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         "%d %d: First NCQ error\n", pAdapter->adapterId,
                         channelIndex);
                pSataChannel->NCQErrHandlingInfo.state = MV_NCQ_ERROR_HANDLING_STATE_WAIT_FOR_COMPLETIONS;
            }
            updatePortsWithErrors(pSataChannel);
            if (isGoodCompletionsExpected(pSataChannel) == MV_FALSE)
            {
                enterRequestSenseState(pSataChannel);
            }
        }
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_INTERRUPTS,
                     "%d %d: EDMA device error while EDMA not active!!!\n",
                     pAdapter->adapterId, channelIndex);
        }
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_ERROR,
                                    MV_SATA_DEVICE_ERROR, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_TRUE;
    }
    return MV_FALSE;
}
#ifdef MV_SATA_C2C_COMM
/*******************************************************************************
* handleC2CInterrupt - channel 2 channel interrupt handler
*
*
* DESCRIPTION:
*   Handles channel 2 channel interrupt (register device 2 host FIS) and
*   convert ATA registers values to user specific 10 bytes message
*
* INPUT:
*   pSataChannel   - pointer to the Sata channel data structure
*
* RETURN:
*   None
*
* COMMENTS:
*   None
*
*******************************************************************************/
static void handleC2CInterrupt(MV_SATA_CHANNEL *pSataChannel)
{
    MV_BUS_ADDR_T   ioBaseAddr =
    pSataChannel->mvSataAdapter->adapterIoBaseAddress;
    MV_U32      eDmaRegsOffset = pSataChannel->eDmaRegsOffset;
    MV_U8       port = pSataChannel->channelNumber & (MV_BIT0 | MV_BIT1);
    MV_U8       sataUnit = (pSataChannel->channelNumber & MV_BIT2) >> 2;
    MV_U8       ATAstatus;
    MV_STORAGE_DEVICE_REGISTERS     deviceRegs;

    mvOsSemTake(&pSataChannel->semaphore);

    MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                     MV_ATA_DEVICE_ALTERNATE_REG_OFFSET);

    ATAstatus = MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                                 MV_ATA_DEVICE_STATUS_REG_OFFSET);
    /* clear DevInterrupt*/
    MV_REG_WRITE_DWORD(ioBaseAddr, MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                       MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET, ~(MV_BIT8 << port));

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
             "%d %d: C2C Interrupt: status 0x%02x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, ATAstatus);

    dumpAtaDeviceRegisters(pSataChannel->mvSataAdapter,
                           pSataChannel->channelNumber,
                           MV_TRUE,
                           &deviceRegs);
    mvOsSemRelease(&pSataChannel->semaphore);
    if (pSataChannel->C2CCallback)
    {
        MV_U8 msg[MV_C2C_MESSAGE_SIZE];
        msg[0] = deviceRegs.errorRegister;
        msg[1] = deviceRegs.lbaLowRegister & 0xFF;
        msg[2] = deviceRegs.lbaMidRegister & 0xFF;
        msg[3] = deviceRegs.lbaHighRegister & 0xFF;
        msg[4] = deviceRegs.deviceRegister;
        msg[5] = deviceRegs.lbaLowRegister >> 8;
        msg[6] = deviceRegs.lbaMidRegister >> 8;
        msg[7] = deviceRegs.lbaHighRegister >> 8;
        msg[8] = deviceRegs.sectorCountRegister & 0xFF;
        msg[9] = deviceRegs.sectorCountRegister >> 8;
        pSataChannel->C2CCallback(pSataChannel->mvSataAdapter,
                                  pSataChannel,
                                  MV_C2C_REGISTER_DEVICE_TO_HOST_FIS_DONE,
                                  MV_C2C_MESSAGE_SIZE,
                                  msg);
    }
}
#endif



static void handleDeviceInterrupt(MV_SATA_ADAPTER *pAdapter, MV_U8 sataUnit,
                                  MV_U8 port)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_QUEUED_COMMAND_ENTRY *pCommandEntry;
    MV_U8   channelIndex;

    channelIndex = MV_CHANNEL_INDEX(sataUnit, port);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
             "%d %d: SaDevInterrupt Received\n", pAdapter->adapterId,
             channelIndex);

    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: SaDevInterrupt Received for disconnected channel\n",
                 pAdapter->adapterId, channelIndex);
        /* disable SaDevInterrupts from this channel */
        disableSaDevInterrupts(pAdapter,channelIndex);
        return;
    }
#ifdef MV_SATA_C2C_COMM
    /*handle channel 2 channel communication mode*/
    if (pSataChannel->C2CmodeEnabled == MV_TRUE)
    {
        handleC2CInterrupt(pSataChannel);
        return;
    }
#endif

    mvOsSemTake(&pSataChannel->semaphore);
    if (pSataChannel->NCQErrHandlingInfo.state == MV_NCQ_ERROR_HANDLING_STATE_WAIT_FOR_BUSY)
    {
        MV_U8       ATAstatus;

        ATAstatus = MV_REG_READ_BYTE(pAdapter->adapterIoBaseAddress,
                                     pSataChannel->eDmaRegsOffset +
                                     MV_ATA_DEVICE_STATUS_REG_OFFSET);
        /* clear DevInterrupt*/
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET,
                           ~(MV_BIT8 << port));

        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: enter NCQ error handling request sense state\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber, ATAstatus);
        pSataChannel->NCQErrHandlingInfo.state = MV_NCQ_ERROR_HANDLING_STATE_REQUEST_SENSE;
        pSataChannel->NCQErrHandlingInfo.CurrPort = 0;
        setReadLogExtCmndPointers(pSataChannel);
        handlePortNCQError(pSataChannel);
        mvOsSemRelease(&pSataChannel->semaphore);
        return;
    }

    /* clear interrupt */

    pCommandEntry = pSataChannel->commandsQueueHead;
    if (pCommandEntry == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: SaDevInterrupt: No command is running!!!\n",
                 pAdapter->adapterId, channelIndex);
        _dumpSataRegs(pAdapter, channelIndex);
        /* disable SaDevInterrupts from this channel */
        disableSaDevInterrupts(pAdapter,channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return;
    }
    if ((pCommandEntry->isFreeEntry == MV_TRUE) ||
        (pCommandEntry->commandInfo.type != MV_QUEUED_COMMAND_TYPE_NONE_UDMA))
    {
        if (pCommandEntry->isFreeEntry == MV_TRUE)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR|MV_DEBUG_NON_UDMA_COMMAND,
                     "%d %d: SaDevInterrupt: current command is free ???\n",
                     pAdapter->adapterId, channelIndex);
        }
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR|MV_DEBUG_NON_UDMA_COMMAND,
                     "%d %d: SaDevInterrupt: current command is Not PIO ???\n",
                     pAdapter->adapterId, channelIndex);
        }
        /* disable SaDevInterrupts from this channel */
        disableSaDevInterrupts(pAdapter,channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return;
    }
    handlePIOInterrupt(pSataChannel, pCommandEntry);
    mvOsSemRelease(&pSataChannel->semaphore);
}

static void handlePIOInterrupt(MV_SATA_CHANNEL *pSataChannel,
                               MV_QUEUED_COMMAND_ENTRY *pCommandEntry)
{
    MV_BUS_ADDR_T   ioBaseAddr = pSataChannel->mvSataAdapter->adapterIoBaseAddress;
    MV_U32      eDmaRegsOffset = pSataChannel->eDmaRegsOffset;
    MV_U8       port = pSataChannel->channelNumber & (MV_BIT0 | MV_BIT1);
    MV_U8       sataUnit = (pSataChannel->channelNumber & MV_BIT2) >> 2;
    MV_U8       ATAstatus;

    MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                     MV_ATA_DEVICE_ALTERNATE_REG_OFFSET);

    ATAstatus = MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                                 MV_ATA_DEVICE_STATUS_REG_OFFSET);
    /* clear DevInterrupt*/
    MV_REG_WRITE_DWORD(ioBaseAddr, MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                       MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET, ~(MV_BIT8 << port));

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
             "%d %d: PIO Interrupt: cmd 0x%02X, type %d. status 0x%02x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber,
             pCommandEntry->commandInfo.commandParams.NoneUdmaCommand.command,
             pCommandEntry->commandInfo.commandParams.NoneUdmaCommand.protocolType,
             ATAstatus);

    if (ATAstatus & MV_ATA_BUSY_STATUS)
    {
        if (pSataChannel->mvSataAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "%d %d: "
                     "PIO Interrupt: drive is BUSY!!!! status 0x%02x\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber, ATAstatus);
        }
        return;
    }
    if (ATAstatus & MV_ATA_ERROR_STATUS)
    {
        completePIOCommand(pSataChannel, pCommandEntry, MV_TRUE);
        return;
    }
    switch (pCommandEntry->commandInfo.commandParams.NoneUdmaCommand.protocolType)
    {
    case MV_NON_UDMA_PROTOCOL_NON_DATA:
        /* command is successfully completed*/
        completePIOCommand(pSataChannel, pCommandEntry, MV_FALSE);
        break;
    case MV_NON_UDMA_PROTOCOL_PIO_DATA_IN:
        if (ATAstatus & MV_ATA_READY_STATUS)
        {
            if (transferPIOData(pSataChannel,
                                &pCommandEntry->commandInfo.commandParams.NoneUdmaCommand) == MV_TRUE)
            {
                if (pCommandEntry->commandInfo.commandParams.NoneUdmaCommand.count == 0)
                {
                    completePIOCommand(pSataChannel, pCommandEntry, MV_FALSE);
                }
#ifdef MV_SATA_SUPPORT_READ_WRITE_LONG

                /* for Read long only*/
                if (pCommandEntry->commandInfo.commandParams.NoneUdmaCommand.count == 4)
                {
                    if (transferPIOData(pSataChannel,
                                        &pCommandEntry->commandInfo.commandParams.NoneUdmaCommand) == MV_TRUE)
                    {
                        completePIOCommand(pSataChannel, pCommandEntry, MV_FALSE);
                    }
                    else
                    {
                        completePIOCommand(pSataChannel, pCommandEntry, MV_TRUE);
                    }
                }
#endif /*MV_SATA_SUPPORT_READ_WRITE_LONG*/

            }
            else
            {
                completePIOCommand(pSataChannel, pCommandEntry, MV_TRUE);
            }
        }
        else    /* when BUSY and DRQ cleared to zero then the device has*/
        {
            /* completed the command with error                     */
            completePIOCommand(pSataChannel, pCommandEntry, MV_TRUE);
            return;
        }
        break;
    case MV_NON_UDMA_PROTOCOL_PIO_DATA_OUT:
        if ((ATAstatus & MV_ATA_READY_STATUS) &&
            !(ATAstatus & MV_ATA_DEVICE_FAULT_STATUS))
        {
            if (pCommandEntry->commandInfo.commandParams.NoneUdmaCommand.count == 0)
            {
                completePIOCommand(pSataChannel, pCommandEntry, MV_FALSE);
            }
            else
            {
                if (transferPIOData(pSataChannel,
                                    &pCommandEntry->commandInfo.commandParams.NoneUdmaCommand) == MV_FALSE)
                {
                    completePIOCommand(pSataChannel, pCommandEntry, MV_TRUE);
                }
            }
        }
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: PIO Interrupt: PIO"
                     " Data Out command failed status 0x%02x\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber, ATAstatus);
            completePIOCommand(pSataChannel, pCommandEntry, MV_TRUE);
        }
        break;
    default: /* never reached */
        break;
    }
}
static MV_BOOLEAN transferPIOData(MV_SATA_CHANNEL *pSataChannel,
                                  MV_NONE_UDMA_COMMAND_PARAMS   *pNoneUdmaCommandParams)
{
    MV_U32  i;
    MV_U32  dataBlockWords = pSataChannel->DRQDataBlockSize * ATA_SECTOR_SIZE_IN_WORDS;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
             "%d %d: xfer data for PIO Data command.count %d\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, pNoneUdmaCommandParams->count);

    switch (pNoneUdmaCommandParams->protocolType)
    {
    case MV_NON_UDMA_PROTOCOL_PIO_DATA_OUT:
        for (i = 0; i < dataBlockWords; i++)
        {
            if (pNoneUdmaCommandParams->count == 0)
            {
                return MV_TRUE;
            }
            pNoneUdmaCommandParams->count--;
            MV_REG_WRITE_WORD(pSataChannel->mvSataAdapter->adapterIoBaseAddress,
                              pSataChannel->eDmaRegsOffset +
                              MV_ATA_DEVICE_PIO_DATA_REG_OFFSET,
                              *pNoneUdmaCommandParams->bufPtr++);
        }
        break;
    case MV_NON_UDMA_PROTOCOL_PIO_DATA_IN:
        for (i = 0; i < dataBlockWords; i++)
        {
            if (pNoneUdmaCommandParams->count == 0)
            {
                return MV_TRUE;
            }
            pNoneUdmaCommandParams->count--;
            *pNoneUdmaCommandParams->bufPtr++ =
            MV_REG_READ_WORD(pSataChannel->mvSataAdapter->adapterIoBaseAddress,
                             pSataChannel->eDmaRegsOffset +
                             MV_ATA_DEVICE_PIO_DATA_REG_OFFSET);
        }
        break;
    case MV_NON_UDMA_PROTOCOL_NON_DATA:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "%d %d: in xfer data "
                 "PIO command is None Data \n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        return MV_FALSE;
    }
    return MV_TRUE;
}

static void completePIOCommand(MV_SATA_CHANNEL *pSataChannel,
                               MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                               MV_BOOLEAN failed)
{
    MV_COMPLETION_TYPE              compType = MV_COMPLETION_TYPE_NORMAL;
    MV_STORAGE_DEVICE_REGISTERS     deviceRegs;
    MV_U8                           commandTag;
    MV_NONE_UDMA_COMMAND_PARAMS     *pParams =
    &pCommandEntry->commandInfo.commandParams.NoneUdmaCommand;

    dumpAtaDeviceRegisters(pSataChannel->mvSataAdapter,
                           pSataChannel->channelNumber, pParams->isEXT,
                           &deviceRegs);
    if (failed == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: PIO Command completed "
                 "with error\n", pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);

        compType = MV_COMPLETION_TYPE_ERROR;
        pSataChannel->queueCommandsEnabled = MV_FALSE;
    }
    else
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: PIO Command completed successfully\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        pSataChannel->recoveredErrorsCounter = 0;
    }
    /* pCommandEntry is invalid after calling the callback function
        so we cache the tag to be used later*/

    commandTag = pCommandEntry->commandTag;
    if (commandTag == 0xFF)/*NCQ Error handling ReadLogExt command*/
    {
        /*sanity check*/
        if (pCommandEntry != pSataChannel->NCQErrHandlingInfo.pReadLogExtEntry)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR,
                     "%d %d: in completePIOCommand, command is ReadLogExt"
                     ", pointers mismatch \n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber);
        }
        commandsQueueRemove(pSataChannel,pCommandEntry);
    }
    else
    {
        removeCommand(pSataChannel,pCommandEntry);
    }

    pParams->callBack(pSataChannel->mvSataAdapter, pSataChannel->channelNumber,
                      compType, pParams->commandId, 0, 0, &deviceRegs);

    if (commandTag != 0xFF)/*if not NCQ Error handling ReadLogExt command*/
    {
        if (failed == MV_TRUE)
        {
            _doDevErrorRecovery(pSataChannel);
        }
        else
        {
            _insertQCommandsIntoEdma(pSataChannel);
        }
    }
}
/*******************************************************************************
* _resetEdmaQPointers - resets EDMA's Queues Pointers
*
*
* DESCRIPTION:
*
* INPUT:
*   *pSataChannel   - pointer to the Sata channel data structure
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
*
* COMMENTS:
*   this function assumes that the channel semaphore is locked
*
*******************************************************************************/
static MV_BOOLEAN _resetEdmaQPointers(MV_SATA_CHANNEL *pSataChannel)
{
    MV_BUS_ADDR_T       ioBaseAddr =
    pSataChannel->mvSataAdapter->adapterIoBaseAddress;
    MV_U32 eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, "%d %d: _resetEdmaQPointers\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber);

    pSataChannel->EdmaQueuedCommands = 0;
    pSataChannel->reqInPtr = 0;
    pSataChannel->rspOutPtr = 0;

    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_EDMA_REQUEST_Q_BAH_REG_OFFSET,
                       pSataChannel->requestQueuePciHiAddress);
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_EDMA_REQUEST_Q_INP_REG_OFFSET,
                       pSataChannel->requestQueuePciLowAddress &
                       MV_EDMA_REQUEST_Q_BA_MASK);
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_EDMA_REQUEST_Q_OUTP_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_EDMA_RESPONSE_Q_BAH_REG_OFFSET,
                       pSataChannel->responseQueuePciHiAddress);
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_EDMA_RESPONSE_Q_INP_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_EDMA_RESPONSE_Q_OUTP_REG_OFFSET,
                       pSataChannel->responseQueuePciLowAddress &
                       MV_EDMA_RESPONSE_Q_BA_MASK);

    return MV_TRUE;
}
/*******************************************************************************
* resetEdmaChannel - resets the channel data stucture and EDMA registers
*
*
* DESCRIPTION:
*   this function resets the low level EDMA fields of Sata channel data
*   structure and initialize the EDMA register accourdingly
*
* INPUT:
*   *pSataChannel   - pointer to the Sata channel data structure
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
*
* COMMENTS:
*   this function assumes that the channel semaphore is locked
*
*******************************************************************************/
static MV_BOOLEAN resetEdmaChannel(MV_SATA_CHANNEL *pSataChannel)
{
    MV_BUS_ADDR_T       ioBaseAddr =
    pSataChannel->mvSataAdapter->adapterIoBaseAddress;
    MV_U32 eDmaRegsOffset = pSataChannel->eDmaRegsOffset;
    int i;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, "%d %d: resetEdmaChannel\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber);

    if (MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                          MV_EDMA_COMMAND_REG_OFFSET) &
        MV_EDMA_COMMAND_HARD_RST_MASK)
    {
        MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                           MV_EDMA_COMMAND_REG_OFFSET,
                           MV_EDMA_COMMAND_DISABLE_MASK);

        MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                          MV_EDMA_COMMAND_REG_OFFSET);
        mvMicroSecondsDelay(pSataChannel->mvSataAdapter,
                            MV_HARD_RESET_WAIT_NEGATE);
        _fixPhyParams(pSataChannel->mvSataAdapter, pSataChannel->channelNumber);
    }
    else
    {
        MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                           MV_EDMA_COMMAND_REG_OFFSET,
                           MV_EDMA_COMMAND_DISABLE_MASK);
    }

    pSataChannel->outstandingCommands = 0;
    for (i = 0; i <= MV_SATA_PM_MAX_PORTS; i++)
    {
        pSataChannel->portQueuedCommands[i] = 0;
    }
    pSataChannel->noneUdmaOutstandingCommands = 0;
    pSataChannel->EdmaActive = MV_FALSE;

    /* init free entries stack*/
    pSataChannel->freeIDsNum = MV_SATA_SW_QUEUE_SIZE;
    for (i = 0; i < MV_SATA_SW_QUEUE_SIZE; i++)
    {
        pSataChannel->freeIDsStack[i] = MV_SATA_SW_QUEUE_SIZE - 1 - i;
        pSataChannel->commandsQueue[i].isFreeEntry = MV_TRUE;
    }

    pSataChannel->commandsQueueHead = NULL;
    pSataChannel->commandsQueueTail = NULL;
    pSataChannel->queueCommandsEnabled = MV_FALSE;
#ifdef MV_SATA_C2C_COMM

    /* C2C */
    pSataChannel->C2CmodeEnabled = MV_FALSE;
#endif
    pSataChannel->NCQErrHandlingInfo.CurrPort = 0;
    pSataChannel->NCQErrHandlingInfo.state = MV_NCQ_ERROR_HANDLING_STATE_IDLE;
    pSataChannel->NCQErrHandlingInfo.PortsWithErrors = 0;
    _resetEdmaQPointers(pSataChannel);
    return MV_TRUE;
}

static void flushDmaQueue(MV_SATA_CHANNEL *pSataChannel,MV_FLUSH_TYPE flushType,
                          MV_COMPLETION_TYPE completionType, MV_U16 eDmaCause)
{
    mvSataCommandCompletionCallBack_t callBackFunc;
    int i;


    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d: Flush DMA, type=%s, commands"
             " %d (on EDMA %d)\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber,
             (flushType==MV_FLUSH_TYPE_CALLBACK)?"CALLBACK":"NONE",
             pSataChannel->outstandingCommands,pSataChannel->EdmaQueuedCommands);

    if (flushType == MV_FLUSH_TYPE_CALLBACK)
    {
        for (i=0;i < MV_SATA_SW_QUEUE_SIZE; i++)
        {
            if (pSataChannel->commandsQueue[i].isFreeEntry == MV_FALSE)
            {
                MV_STORAGE_DEVICE_REGISTERS deviceRegisters;
                MV_BOOLEAN  isEXT;
                MV_VOID_PTR commandId;
                if (pSataChannel->commandsQueue[i].commandInfo.type == MV_QUEUED_COMMAND_TYPE_NONE_UDMA)
                {
                    isEXT = pSataChannel->commandsQueue[i].commandInfo.commandParams.NoneUdmaCommand.isEXT;
                    commandId = pSataChannel->commandsQueue[i].commandInfo.commandParams.NoneUdmaCommand.commandId;
                    callBackFunc = pSataChannel->commandsQueue[i].commandInfo.commandParams.NoneUdmaCommand.callBack;
                }
                else
                {
                    isEXT = pSataChannel->commandsQueue[i].commandInfo.commandParams.udmaCommand.isEXT;
                    commandId = pSataChannel->commandsQueue[i].commandInfo.commandParams.udmaCommand.commandId;
                    callBackFunc = pSataChannel->commandsQueue[i].commandInfo.commandParams.udmaCommand.callBack;
                }

                dumpAtaDeviceRegisters(pSataChannel->mvSataAdapter,
                                       pSataChannel->channelNumber, isEXT,
                                       &deviceRegisters);
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         "%d %d: Calling callBackFunc - command index %d at %p,"
                         " next %p, prev %p\n",
                         pSataChannel->mvSataAdapter->adapterId,
                         pSataChannel->channelNumber,
                         i,
                         &pSataChannel->commandsQueue[i],
                         pSataChannel->commandsQueue[i].next,
                         pSataChannel->commandsQueue[i].prev);
                callBackFunc(pSataChannel->mvSataAdapter,
                             pSataChannel->channelNumber, completionType,
                             commandId, eDmaCause, 0, &deviceRegisters);
            }
        }
    }
}

static void _fixPhyParams(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    /* Set unit 0 or 1 */
    MV_U8       sataUnit = (channelIndex & MV_BIT2) >> 2;
    /* Set port 0-3 */
    MV_U8       port = channelIndex & (MV_BIT0 | MV_BIT1);
    MV_U32      regVal;

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {

        if (pAdapter->chipIs50XXB0 == MV_TRUE)
        {
            /*SHI12*/
            /* Disable auto-power management*/
            regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                       MV_SATA_I_HC_LT_MODES_PORT_REG_OFFSET(port));
            regVal |= MV_BIT19; /* disbale auto-power management*/
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                               MV_SATA_I_HC_LT_MODES_PORT_REG_OFFSET(port),
                               regVal);
            /*SHI9*/
            /*Modify squelch threshold*/
            regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                       MV_SATA_I_HC_PHY_CONTROL_BRIDGE_PORT_OFFSET(port));

            regVal &= ~0x3;
            regVal |= 0x1;
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                               MV_SATA_I_HC_PHY_CONTROL_BRIDGE_PORT_OFFSET(port),
                               regVal);
        }
        /* Revert values of pre-emphasis and signal amps to the saved ones */
        {
            regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                       MV_SATA_I_HC_PHY_MODE_BRIDGE_PORT_REG_OFFSET(port));
            regVal &= ~MV_SATA_I_PHY_MODE_AMP_MASK;
            regVal |= (pAdapter->signalAmps[channelIndex] << MV_SATA_I_PHY_MODE_AMP_OFFSET) &
                      MV_SATA_I_PHY_MODE_AMP_MASK;
            regVal &= ~MV_SATA_I_PHY_MODE_PRE_MASK;
            regVal |= (pAdapter->pre[channelIndex] << MV_SATA_I_PHY_MODE_PRE_OFFSET) &
                      MV_SATA_I_PHY_MODE_PRE_MASK;
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                               MV_SATA_I_HC_PHY_MODE_BRIDGE_PORT_REG_OFFSET(port),
                               regVal);
        }
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        MV_U32 phyMode2Offset = getEdmaRegOffset(channelIndex) +
                                MV_SATA_II_PHY_MODE_2_REG_OFFSET;
        if ((pAdapter->chipIs60X1B2 == MV_TRUE) ||
            (pAdapter->chipIs60X1C0 == MV_TRUE))
        {
            /*SHII23*/
            MV_U32 regVal;
            regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                        getEdmaRegOffset (channelIndex) +
                                        MV_SATA_II_PHY_MODE_2_REG_OFFSET);
            regVal |= MV_BIT31;
            regVal &= ~MV_BIT16;
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                getEdmaRegOffset (channelIndex) +
                                MV_SATA_II_PHY_MODE_2_REG_OFFSET,
                                regVal);
            mvMicroSecondsDelay (pAdapter, 200); /* Wait 200uSec */
            regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                        getEdmaRegOffset (channelIndex) +
                                        MV_SATA_II_PHY_MODE_2_REG_OFFSET);
            regVal &= ~MV_BIT31;
            regVal &= ~MV_BIT16;
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                getEdmaRegOffset (channelIndex) +
                                MV_SATA_II_PHY_MODE_2_REG_OFFSET,
                                regVal);
            mvMicroSecondsDelay (pAdapter, 200); /* Wait 200uSec */
        }


        /* Fix values in phyMode 4 register.*/
        /* SHII10 */
        if ((pAdapter->chipIs60X1B2 == MV_TRUE) ||
            (pAdapter->chipIs60X1C0 == MV_TRUE))
        {
            MV_U32 phyMode4Value;
            MV_U32 tempRegOffset, tempRegValue = 0;
            MV_U32 phyMode4Offset = getEdmaRegOffset (channelIndex) +
                                    MV_SATA_II_PHY_MODE_4_REG_OFFSET;
            tempRegOffset = getEdmaRegOffset (channelIndex) + 0x310;

            mvLogMsg(MV_CORE_DRIVER_LOG_ID,  MV_DEBUG|MV_DEBUG_SATA_LINK,
                     "%d %d: PHY mode4 reg value before fix is %x\n",
                     pAdapter->adapterId, channelIndex,
                     MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                        phyMode4Offset));
            phyMode4Value = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                               phyMode4Offset);
            /* SHII13 */
            if (pAdapter->chipIs60X1B2 == MV_TRUE)
            {
                tempRegValue = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                                  tempRegOffset);
            }

            phyMode4Value |= MV_BIT0;
            phyMode4Value &= ~MV_BIT1;
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                phyMode4Offset, phyMode4Value);
            /* SHII13 */
            if (pAdapter->chipIs60X1B2 == MV_TRUE)
            {
                MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                    tempRegOffset, tempRegValue);
            }

            mvLogMsg(MV_CORE_DRIVER_LOG_ID,  MV_DEBUG|MV_DEBUG_SATA_LINK,
                     "%d %d: PHY mode4 reg value after fix is %x\n",
                     pAdapter->adapterId, channelIndex,
                     MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                        phyMode4Offset));
        }

        /* Revert values of pre-emphasis and signal amps to the saved ones */
        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   phyMode2Offset);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID,  MV_DEBUG, "%d %d: PHY mode2 "
                 "reg = %x (Before AMP/PRE modification)\n",
                 pAdapter->adapterId, channelIndex, regVal);

        regVal &= ~MV_SATA_II_PHY_MODE_2_AMP_MASK;
        regVal |= (pAdapter->signalAmps[channelIndex] << MV_SATA_II_PHY_MODE_2_AMP_OFFSET) &
                  MV_SATA_II_PHY_MODE_2_AMP_MASK;
        regVal &= ~MV_SATA_II_PHY_MODE_2_PRE_MASK;
        regVal |= (pAdapter->pre[channelIndex] << MV_SATA_II_PHY_MODE_2_PRE_OFFSET) &
                  MV_SATA_II_PHY_MODE_2_PRE_MASK;
        regVal &= ~MV_BIT16; /* Should always write 0 to bit 16 in phymode 2 */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           phyMode2Offset, regVal);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID,  MV_DEBUG, "%d %d: PHY mode2 "
                 "reg = %x (After AMP/PRE modification)\n",
                 pAdapter->adapterId, channelIndex,
                 MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   phyMode2Offset));
    }

}

static void _channelHardReset(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_U32 EdmaCommandOffset = getEdmaRegOffset(channelIndex) +
                               MV_EDMA_COMMAND_REG_OFFSET;

    maskEdmaInterrupts(pAdapter, channelIndex);
    /* 1. Set ATA reset bit*/
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress, EdmaCommandOffset,
                       MV_EDMA_COMMAND_HARD_RST_MASK);
    MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, EdmaCommandOffset);

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        MV_U32 regVal;

        regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                    getEdmaRegOffset (channelIndex) +
                                    MV_SATA_II_SATA_CONFIG_REG_OFFSET);
        /* SHII8*/
        regVal |= MV_BIT12;

        if ((pAdapter->limitInterfaceSpeed[channelIndex] == MV_TRUE) &&
            (pAdapter->ifSpeed[channelIndex] == MV_SATA_IF_SPEED_1_5_GBPS))
        {
            regVal &= ~MV_BIT7; /* Disable GEn II */
        }
        else
        {
            regVal |= MV_BIT7;  /* Enable GEn II */

        }
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            getEdmaRegOffset (channelIndex) +
                            MV_SATA_II_SATA_CONFIG_REG_OFFSET,
                            regVal);
        MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                           getEdmaRegOffset (channelIndex) +
                           MV_SATA_II_SATA_CONFIG_REG_OFFSET);


    }
    /* 2. Wait 25uSeconds*/
    mvMicroSecondsDelay(pAdapter, MV_HARD_RESET_WAIT_ASSERT);


    /* 3. Clear ATA reset bit*/
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress, EdmaCommandOffset, 0);
    MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, EdmaCommandOffset);

    /* 4. Change phy params (watermark + squelch) */
    _fixPhyParams(pAdapter, channelIndex);
    /* For Gen 1 devices, time delay is needed after resetingt the SATA bridge*/
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        mvMicroSecondsDelay(pAdapter, MV_HARD_RESET_WAIT_NEGATE);
    }
    unmaskEdmaInterrupts(pAdapter, channelIndex);

}
static void _establishSataComm(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_U32 SControlOffset = getEdmaRegOffset(channelIndex) +
                            MV_SATA_II_S_CONTROL_REG_OFFSET;
    MV_U32 SStatusOffset_88SX60X1 = getEdmaRegOffset(channelIndex) +
                                    MV_SATA_II_S_STATUS_REG_OFFSET;
    MV_U32 SStatus;
    MV_U8   retryCount, commRetryCount = 5;

    maskEdmaInterrupts(pAdapter, channelIndex);

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        /*  Set DET field in SControl register to 1 */
        MV_U8       port = channelIndex & (MV_BIT0 | MV_BIT1);
        MV_U8       sataUnit = (channelIndex & MV_BIT2) >> 2;
        SControlOffset = MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                         MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port);
    }

    while (1)
    {
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset, 0x301);
        MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset);
        MV_CPU_WRITE_BUFFER_FLUSH();
        mvMicroSecondsDelay(pAdapter, MV_SATA_COMM_INIT_DELAY);
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset, 0x300);
        MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset);
        MV_CPU_WRITE_BUFFER_FLUSH();
        mvMicroSecondsDelay(pAdapter, MV_SATA_COMM_INIT_WAIT_DELAY);
        unmaskEdmaInterrupts(pAdapter, channelIndex);
        /*Wait 200 msec for PHY to become ready*/
        for (retryCount = 0; retryCount < 200; retryCount++)
        {
            if (_checkSStatusAfterHReset(pAdapter, channelIndex) == MV_FALSE)
            {
                mvMicroSecondsDelay(pAdapter, 1000);
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_SATA_LINK, "%d %d: SATA PHY ready "
                         "after %d msec\n", pAdapter->adapterId, channelIndex,
                         retryCount);

                break;
            }
        }
        if (retryCount == 200)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_SATA_LINK,
                     "%d %d: SATA PHY not ready after 200 msec\n",
                     pAdapter->adapterId, channelIndex);
        }
        /* For 88SX50XX try communication only once */
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
        {
            break;
        }
        SStatus = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                     SStatusOffset_88SX60X1);
        /* SHII10 - retry SATA communication if failed 5 times */
        if ((SStatus == 0x0) || (SStatus == 0x113) || (SStatus == 0x123))
        {
            break;
        }
        commRetryCount --;
        if (commRetryCount == 0)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_FATAL_ERROR,
                     "%d %d: Failed OOB sequence 5 times !!!",
                     pAdapter->adapterId, channelIndex);
            break;
        }
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 "%d %d: Retrying OOB sequnce",pAdapter->adapterId,
                 channelIndex);
    }
}

static void _establishSataCommAll(MV_SATA_ADAPTER *pAdapter)
{
    MV_U8 channelIndex;
    MV_U32 SControlOffset;
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        for (channelIndex = 0 ; channelIndex < pAdapter->numberOfChannels ;
            channelIndex ++)
        {
            maskEdmaInterrupts(pAdapter, channelIndex);
            SControlOffset = getEdmaRegOffset(channelIndex) +
                             MV_SATA_II_S_CONTROL_REG_OFFSET;
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset,
                                0x301);

        }
        /* Wait for 1mSecond for COMRESET for all drives */
        mvMicroSecondsDelay(pAdapter, MV_SATA_COMM_INIT_DELAY);
        for (channelIndex = 0 ; channelIndex < pAdapter->numberOfChannels ;
            channelIndex ++)
        {
            SControlOffset = getEdmaRegOffset(channelIndex) +
                             MV_SATA_II_S_CONTROL_REG_OFFSET;
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset,
                                0x300);
        }
        mvMicroSecondsDelay(pAdapter, MV_SATA_COMM_INIT_WAIT_DELAY);
        for (channelIndex = 0 ; channelIndex < pAdapter->numberOfChannels ;
            channelIndex ++)
        {
            unmaskEdmaInterrupts(pAdapter, channelIndex);
        }
    }
}





void _setActivePMPort(MV_SATA_CHANNEL *pSataChannel, MV_U8 PMPort)
{
    MV_BUS_ADDR_T   ioBaseAddr = pSataChannel->mvSataAdapter->adapterIoBaseAddress;
    MV_U32          eDmaRegsOffset = pSataChannel->eDmaRegsOffset;
    MV_U32  regVal;

    if (pSataChannel->PMSupported == MV_FALSE)
    {
        return;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG | MV_DEBUG_PM, "%d %d: Set TX PM"
             " Port to %x\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, PMPort);

    regVal = MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                               MV_SATA_II_IF_CONTROL_REG_OFFSET);

    regVal &= ~MV_SATA_II_IF_CONTROL_PMTX_MASK;
    regVal |= (PMPort << MV_SATA_II_IF_CONTROL_PMTX_OFFSET) &
              MV_SATA_II_IF_CONTROL_PMTX_MASK;
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset +
                       MV_SATA_II_IF_CONTROL_REG_OFFSET, regVal);
    MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                      MV_SATA_II_IF_CONTROL_REG_OFFSET);

}
static void revertSataHCRegs (MV_SATA_ADAPTER *pAdapter)
{
    MV_U8 channelIndex;
    MV_U8 temp;
    MV_U32 edmaRegsOffset;
    MV_U32 sataHcRegsOffset;
    MV_U32 regTemp;
    MV_U8  sataUnit;

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        for (sataUnit = 0 ; sataUnit < pAdapter->numberOfUnits ; sataUnit ++)
        {
            for (temp = 0 ; temp < MV_SATA_PORT_PER_UNIT ; temp ++)
            {
                channelIndex = temp + sataUnit * MV_SATA_PORT_PER_UNIT;
                edmaRegsOffset = getEdmaRegOffset(channelIndex);
                /* Disable EDMA */
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_COMMAND_REG_OFFSET,
                                   MV_EDMA_COMMAND_DISABLE_MASK);
                MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                  edmaRegsOffset + MV_EDMA_COMMAND_REG_OFFSET);

                /* Reset SATA bridge */
                _channelHardReset(pAdapter, channelIndex);

                /* Zero EDMA registersr */
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_COMMAND_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_CONFIG_REG_OFFSET, 0x11f);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_TIMER_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_REQUEST_Q_BAH_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_REQUEST_Q_INP_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_REQUEST_Q_OUTP_REG_OFFSET,0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_RESPONSE_Q_BAH_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_RESPONSE_Q_OUTP_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_RESPONSE_Q_INP_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_TEST_CONTROL_REG_OFFSET, 0);
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegsOffset + MV_EDMA_IORDY_TIMEOUT_REG_OFFSET, 0xbc);
            }

            /* Revert values of SATA HC regs (few registers are READ-ONLY ) */
            sataHcRegsOffset = MV_SATAHC_REGS_BASE_OFFSET(sataUnit);
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                sataHcRegsOffset + MV_SATAHC_INT_COAL_THRE_REG_OFFSET, 0);
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                sataHcRegsOffset + MV_SATAHC_INT_TIME_THRE_REG_OFFSET, 0);
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                sataHcRegsOffset + MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET, 0);
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                sataHcRegsOffset + MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET, 0);
            regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                         sataHcRegsOffset +
                                         MV_SATA_I_HC_BRIDGES_PINS_CONFIG_REG_OFFSET);
            /* Keep the SS during power on and the reference clock bits (reset sample )*/
            regTemp &= 0x1c1c1c1c;
            regTemp |= 0x03030303;
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                sataHcRegsOffset + MV_SATA_I_HC_BRIDGES_PINS_CONFIG_REG_OFFSET,
                                regTemp);
        }
    }

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        MV_U32 timeout;
        /* Use global reset feature */
        /* Empty PCI master */

        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET,
                            MV_PCI_MAIN_COMMAND_STOP_MASTER_MASK);
        MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                           MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET);
        timeout = 1000;
        while (timeout)
        {
            if (MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                   MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET) &
                MV_PCI_MAIN_COMMAND_MASTER_EMPTY_MASK)
            {
                break;
            }
            mvMicroSecondsDelay (pAdapter, 1);
            timeout --;
        }
        if (timeout == 0)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d: Global reset timeout when"
                     " trying to flush PCI master - discarding the master flush"
                     , pAdapter->adapterId);
        }
        /* Issue global reset - this will reset both SATAHC */
        regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                     MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET);
        regTemp |= MV_PCI_MAIN_COMMAND_GLOBAL_RESET_MASK;
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET, regTemp);
        regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                     MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET);
        if (!(regTemp & MV_PCI_MAIN_COMMAND_GLOBAL_RESET_MASK))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d: Global reset error while "
                     "writing '1' to the global reset bit",pAdapter->adapterId);
        }
        mvMicroSecondsDelay (pAdapter, 5);
        regTemp &= ~(MV_PCI_MAIN_COMMAND_GLOBAL_RESET_MASK | MV_PCI_MAIN_COMMAND_STOP_MASTER_MASK);
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET, regTemp);
        regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                     MV_PCI_MAIN_COMMAND_STATUS_REG_OFFSET);
        if (regTemp & MV_PCI_MAIN_COMMAND_GLOBAL_RESET_MASK)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d: Global reset error while "
                     "writing '1' to the global reset bit",pAdapter->adapterId);
        }
        mvMicroSecondsDelay (pAdapter, 5);
    }
}

static void revertFlashInterfaceRegs (MV_SATA_ADAPTER *pAdapter)
{
    MV_U32 regTemp;
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_FLASH_PARAMS_REG_OFFSET, 0x0fcfffff);
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                     MV_FLASH_GPIO_PORT_CONTROL_OFFSET);
        regTemp &= 0x3;
        regTemp |= (MV_BIT5 | MV_BIT6);


        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            MV_FLASH_GPIO_PORT_CONTROL_OFFSET, regTemp);
    }
}

static void revertPCIInterfaceRegs (MV_SATA_ADAPTER *pAdapter)
{
    MV_U32 regTemp;
    if ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_I))
    {
        if (!((pAdapter->pciConfigDeviceId == MV_SATA_DEVICE_ID_5080) &&
              (pAdapter->pciConfigRevisionId == 0x0)))
        {
            regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                         MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET);
            regTemp |= MV_BIT0;

            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET, regTemp);
        }
    }

    regTemp = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                 MV_PCI_MODE_REG_OFFSET);
    regTemp &= 0xff00ffff;
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_MODE_REG_OFFSET, regTemp);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_DISCARD_TIMER_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_MSI_TRIGGER_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_XBAR_IF_TIMEOUT_REG_OFFSET, 0x000100ff);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_MAIN_INTERRUPT_MASK_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_SERR_MASK_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_INTERRUPT_CAUSE_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_INTERRUPT_MASK_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_ERROR_LOW_ADDRESS_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_ERROR_HIGH_ADDRESS_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_ERROR_ATTRIBUTE_REG_OFFSET, 0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_PCI_ERROR_COMMAND_REG_OFFSET, 0);
}

static void commandsQueueAddTail(MV_SATA_CHANNEL *pSataChannel,
                                 MV_QUEUED_COMMAND_ENTRY *pCommandEntry)
{
    pCommandEntry->next = NULL;
    pCommandEntry->prev = pSataChannel->commandsQueueTail;
    if (pSataChannel->commandsQueueTail != NULL)
    {
        pSataChannel->commandsQueueTail->next = pCommandEntry;
    }
    pSataChannel->commandsQueueTail = pCommandEntry;

    if (pSataChannel->commandsQueueHead == NULL)
    {
        pSataChannel->commandsQueueHead = pCommandEntry;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: command queued. Head:%p Tail:%p "
             "command :%p\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, pSataChannel->commandsQueueHead,
             pSataChannel->commandsQueueTail, pCommandEntry);
}

static void commandsQueueAddHead(MV_SATA_CHANNEL *pSataChannel,
                                 MV_QUEUED_COMMAND_ENTRY *pCommandEntry)
{
    pCommandEntry->next = pSataChannel->commandsQueueHead;
    pCommandEntry->prev = NULL;
    if (pSataChannel->commandsQueueHead != NULL)
    {
        pSataChannel->commandsQueueHead->prev = pCommandEntry;
    }
    pSataChannel->commandsQueueHead = pCommandEntry;

    if (pSataChannel->commandsQueueTail == NULL)
    {
        pSataChannel->commandsQueueTail = pCommandEntry;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: command queued. Head:%p Tail:%p "
             "command :%p\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, pSataChannel->commandsQueueHead,
             pSataChannel->commandsQueueTail, pCommandEntry);
}

static void commandsQueueRemove(MV_SATA_CHANNEL *pSataChannel,
                                MV_QUEUED_COMMAND_ENTRY *pCommandEntry)
{
    if (pCommandEntry->next == NULL)    /* last */
    {
        pSataChannel->commandsQueueTail = pCommandEntry->prev;
        if (pSataChannel->commandsQueueTail != NULL)
        {
            pSataChannel->commandsQueueTail->next = NULL;
        }
    }
    else
    {
        pCommandEntry->next->prev = pCommandEntry->prev;
    }

    if (pCommandEntry->prev == NULL) /* head*/
    {
        pSataChannel->commandsQueueHead = pCommandEntry->next;
        if (pSataChannel->commandsQueueHead != NULL)
        {
            pSataChannel->commandsQueueHead->prev = NULL;
        }
    }
    else
    {
        pCommandEntry->prev->next = pCommandEntry->next;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: command removed. Head:%p Tail:%p "
             "command :%p\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, pSataChannel->commandsQueueHead,
             pSataChannel->commandsQueueTail, pCommandEntry);
    pCommandEntry->next = NULL;
    pCommandEntry->prev = NULL;
}
static void addCommand(MV_SATA_CHANNEL *pSataChannel,
                       MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                       MV_QUEUE_COMMAND_INFO *pCommandInfo)
{
    if (pCommandInfo->type == MV_QUEUED_COMMAND_TYPE_UDMA)
    {
        memcpy(&pCommandEntry->commandInfo.commandParams.udmaCommand,
               &pCommandInfo->commandParams.udmaCommand,
               sizeof(MV_UDMA_COMMAND_PARAMS));

        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: queue Udma command.\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
    }
    else
    {
        memcpy(&pCommandEntry->commandInfo.commandParams.NoneUdmaCommand,
               &pCommandInfo->commandParams.NoneUdmaCommand,
               sizeof(MV_NONE_UDMA_COMMAND_PARAMS));
        pSataChannel->noneUdmaOutstandingCommands++;
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: queue Non Udma command.[%d]\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber,
                 pSataChannel->noneUdmaOutstandingCommands);
    }
    pCommandEntry->commandInfo.type = pCommandInfo->type;
    pCommandEntry->commandInfo.PMPort = pCommandInfo->PMPort;
    commandsQueueAddTail(pSataChannel, pCommandEntry);

    pCommandEntry->isFreeEntry = MV_FALSE;
    pSataChannel->outstandingCommands++;
    pSataChannel->portQueuedCommands[pCommandInfo->PMPort]++;
}

static void removeCommand(MV_SATA_CHANNEL *pSataChannel,
                          MV_QUEUED_COMMAND_ENTRY *pCommandEntry)
{
    if (pCommandEntry->commandInfo.type == MV_QUEUED_COMMAND_TYPE_UDMA)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: remove Udma command.\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
    }
    else
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: remove Non Udma command.[%d]\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber,
                 pSataChannel->noneUdmaOutstandingCommands);

        pSataChannel->noneUdmaOutstandingCommands--;
    }
    commandsQueueRemove(pSataChannel,pCommandEntry);
    pSataChannel->freeIDsStack[pSataChannel->freeIDsNum++] =
    pCommandEntry->commandTag;
    pCommandEntry->isFreeEntry = MV_TRUE;
    pSataChannel->outstandingCommands--;
    pSataChannel->portQueuedCommands[pCommandEntry->commandInfo.PMPort]--;
}

static MV_U32 SaDevInterrutpBit(MV_U8 channelIndex)
{
    MV_U32      maskBit = 0;

    if (channelIndex >= MV_SATA_PORT_PER_UNIT)
    {
        maskBit = (1 << ((channelIndex << 1) + 2));
    }
    else
    {
        maskBit = (1 << ((channelIndex << 1) + 1));
    }
    return maskBit;
}
static void enableSaDevInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_U32      maskBit = 0;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: enable SaDevInterrupts.\n",
             pAdapter->adapterId, channelIndex);
    maskBit = SaDevInterrutpBit(channelIndex);
    mvOsSemTake(&pAdapter->interruptsMaskSem);

    pAdapter->mainMask |= maskBit;

    /*clear disk interrupt */
    MV_REG_READ_BYTE(pAdapter->adapterIoBaseAddress,
                     getEdmaRegOffset(channelIndex) +
                     MV_ATA_DEVICE_STATUS_REG_OFFSET);
    /* clear DevInterrupt*/
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_SATAHC_REGS_BASE_OFFSET((channelIndex & MV_BIT2) >> 2) +
                       MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET,
                       ~(MV_BIT8 << (channelIndex & (MV_BIT0 | MV_BIT1))));

    /* unmask*/
    if (pAdapter->interruptsAreMasked == MV_FALSE)
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                           pAdapter->mainMask);

        MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                          MV_MAIN_INTERRUPT_MASK_REG_OFFSET);

    }
    mvOsSemRelease(&pAdapter->interruptsMaskSem);
}

void disableSaDevInterrupts(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_U32      maskBit = 0;


    maskBit = SaDevInterrutpBit(channelIndex);
    mvOsSemTake(&pAdapter->interruptsMaskSem);
    pAdapter->mainMask &= ~maskBit;
    if (pAdapter->interruptsAreMasked == MV_FALSE)
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                           pAdapter->mainMask);

        MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                          MV_MAIN_INTERRUPT_MASK_REG_OFFSET);

    }
    mvOsSemRelease(&pAdapter->interruptsMaskSem);
}
static void _checkATAStatus(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL  *pSataChannel = pAdapter->sataChannel[channelIndex];
    MV_U8 ATAstatus = MV_REG_READ_BYTE(pAdapter->adapterIoBaseAddress,
                                       pSataChannel->eDmaRegsOffset +
                                       MV_ATA_DEVICE_STATUS_REG_OFFSET);

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        if ((ATAstatus & (MV_ATA_BUSY_STATUS|MV_ATA_DATA_REQUEST_STATUS|
                          MV_ATA_READY_STATUS|MV_ATA_DEVICE_FAULT_STATUS|
                          MV_ATA_ERROR_STATUS)) == MV_ATA_READY_STATUS)
        {
            return;
        }
    }
    else
    {
        if ((ATAstatus & (MV_ATA_BUSY_STATUS|MV_ATA_DATA_REQUEST_STATUS|
                          MV_ATA_READY_STATUS)) == MV_ATA_READY_STATUS)
        {
            return;
        }
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: _checkATAStatus "
             "EDMA can't be enabled with ATA status (0x%02x), do SW reset\n",
             pAdapter->adapterId, channelIndex, ATAstatus);
    if (pSataChannel->PMSupported == MV_TRUE)
    {
        _setActivePMPort(pSataChannel, MV_SATA_PM_CONTROL_PORT);
    }
    _doSoftReset(pSataChannel);
}
static void activateEdma(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL  *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32      eDmaRegsOffset;
    MV_U8       sataUnit;
    MV_U8       port;

    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: activateEdma\n", pAdapter->adapterId,
             channelIndex);
    pSataChannel->EdmaActive = MV_TRUE;
    sataUnit = (channelIndex & MV_BIT2) >> 2;
    port = channelIndex & (MV_BIT0 | MV_BIT1);
    /* clear Device interrupt */
    MV_REG_WRITE_DWORD(ioBaseAddr, MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                       MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET,
                       ~((MV_BIT8 | MV_BIT0) << port));
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        MV_REG_WRITE_DWORD(ioBaseAddr,
                           pSataChannel->eDmaRegsOffset +
                           MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET, 0);
    }
    else
    {
        MV_REG_WRITE_DWORD(ioBaseAddr,
                           pSataChannel->eDmaRegsOffset +
                           MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET, MV_BIT8);
    }

    /* disable sata device interrupts */
    disableSaDevInterrupts(pAdapter, channelIndex);

    MV_CPU_WRITE_BUFFER_FLUSH();

    _checkATAStatus(pAdapter, channelIndex);

    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset + MV_EDMA_COMMAND_REG_OFFSET,
                       MV_EDMA_COMMAND_ENABLE_MASK);
}

static void deactivateEdma(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL  *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32      eDmaRegsOffset;
    MV_U32      counter = 0;
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: deactivateEdma\n", pAdapter->adapterId,
             channelIndex);
    pSataChannel->EdmaActive = MV_FALSE;

    MV_CPU_WRITE_BUFFER_FLUSH();
    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset + MV_EDMA_COMMAND_REG_OFFSET,
                       MV_EDMA_COMMAND_DISABLE_MASK);
    while (counter < 1000)
    {
        if (MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                              MV_EDMA_COMMAND_REG_OFFSET) & MV_BIT0)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: deactivateEdma: "
                     "Edma still active. elapsed time %d us\n", pAdapter->adapterId,
                     channelIndex, counter * 1000);
            mvMicroSecondsDelay(pAdapter, 1000);
        }
        else
        {
            break;
        }
        counter++;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: deactivateEdma: "
             "Edma status reg 0x%08x\n", pAdapter->adapterId,
             channelIndex, MV_REG_READ_DWORD(ioBaseAddr,
                                             eDmaRegsOffset +
                                             MV_EDMA_STATUS_REG_OFFSET));
    if (counter >= 1000)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR,
                 " %d %d: deactivateEdma: Edma Failed (EDMA status = %x)\n",
                 pAdapter->adapterId, channelIndex,
                 MV_REG_READ_DWORD(ioBaseAddr,
                                   eDmaRegsOffset + MV_EDMA_STATUS_REG_OFFSET));
        pSataChannel->queueCommandsEnabled = MV_FALSE;
        flushDmaQueue (pSataChannel, MV_FLUSH_TYPE_CALLBACK,
                       MV_COMPLETION_TYPE_ABORT, 0);
        resetEdmaChannel(pSataChannel);
        mvOsSemRelease(&pSataChannel->semaphore);
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_SATA_ERROR,
                                    MV_SATA_UNRECOVERABLE_COMMUNICATION_ERROR,
                                    channelIndex);
        mvOsSemTake(&pSataChannel->semaphore);
    }

    /*_dumpSataRegs(pAdapter, channelIndex);*/
    enableSaDevInterrupts(pAdapter, channelIndex);
}

static void EdmaReqQueueInsert(MV_SATA_CHANNEL *pSataChannel,
                               MV_QUEUED_COMMAND_ENTRY *pCommandEntry,
                               MV_UDMA_COMMAND_PARAMS  *pUdmaParams)
{
    MV_BUS_ADDR_T   ioBaseAddr = pSataChannel->mvSataAdapter->adapterIoBaseAddress;


    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_UDMA_COMMAND, " %d %d: Insert Edma "
             "Request. PMPort %x tag = 0x%x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, pCommandEntry->commandInfo.PMPort,
             pCommandEntry->commandTag);

    /* insert the last commmand into the Edma queue */
    writeEdmaRequestEntry (&pSataChannel->requestQueue[pSataChannel->reqInPtr],
                           pSataChannel, pCommandEntry, pUdmaParams);

    pSataChannel->reqInPtr++;
    pSataChannel->reqInPtr &= MV_EDMA_QUEUE_MASK;
    pSataChannel->EdmaQueuedCommands++;
    pCommandEntry->isCommandInEdma = MV_TRUE;

    MV_CPU_WRITE_BUFFER_FLUSH();
    MV_REG_WRITE_DWORD(ioBaseAddr,
                       pSataChannel->eDmaRegsOffset +
                       MV_EDMA_REQUEST_Q_INP_REG_OFFSET,
                       (pSataChannel->requestQueuePciLowAddress &
                        MV_EDMA_REQUEST_Q_BA_MASK) |
                       ((pSataChannel->reqInPtr << MV_EDMA_REQUEST_Q_INP_OFFSET)
                        & MV_EDMA_REQUEST_Q_INP_MASK));

}

static MV_VOID _insertQCommandsIntoEdma(MV_SATA_CHANNEL *pSataChannel)
{
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: _insert"
             "QCommandsIntoEdma\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber);

    if (pSataChannel->commandsQueueHead == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: Commands queue is empty\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        return;
    }
    if (pSataChannel->commandsQueueHead->commandInfo.type ==
        MV_QUEUED_COMMAND_TYPE_NONE_UDMA)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: Next Command is PIO\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        if (pSataChannel->PMSupported == MV_TRUE)
        {
            _setActivePMPort(pSataChannel,
                             pSataChannel->commandsQueueHead->commandInfo.PMPort);
        }
        if (sendNoneUdmaCommand(pSataChannel,
                                pSataChannel->commandsQueueHead) == MV_FALSE)
        {
            completePIOCommand(pSataChannel, pSataChannel->commandsQueueHead,
                               MV_TRUE);
        }

    }
    else
    {
        MV_QUEUED_COMMAND_ENTRY *pEntry;
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
                 "%d %d: Next Command is UDMA\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        activateEdma(pSataChannel->mvSataAdapter,pSataChannel->channelNumber);
        pEntry = pSataChannel->commandsQueueHead;
        while ((pEntry != NULL) &&
               (pEntry->commandInfo.type == MV_QUEUED_COMMAND_TYPE_UDMA))
        {
            EdmaReqQueueInsert(pSataChannel, pEntry,
                               &pEntry->commandInfo.commandParams.udmaCommand);
            pEntry = pEntry->next;
        }

    }

}
/* do device error recovery for PIO, DMA and QUEUED DMA commands (not NCQ)*/
static MV_BOOLEAN _doDevErrorRecovery(MV_SATA_CHANNEL *pSataChannel)
{
    if (pSataChannel->mvSataAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        if (_doSoftReset(pSataChannel) == MV_FALSE)
        {
            return MV_FALSE;
        }
    }
    pSataChannel->queueCommandsEnabled = MV_TRUE;
    /* Enable the storage device interrupts */
    enableSaDevInterrupts(pSataChannel->mvSataAdapter,
                          pSataChannel->channelNumber);
    _resetEdmaQPointers(pSataChannel);
    _insertQCommandsIntoEdma(pSataChannel);
    return MV_TRUE;
}
/* this function used for NCQ error handling, it cheks if further commands
    expected to be completed successfully (from drives without errors in PM)*/
static MV_BOOLEAN isGoodCompletionsExpected(MV_SATA_CHANNEL *pSataChannel)
{
    MV_QUEUED_COMMAND_ENTRY *pEntry;

    /*sanity checks*/
    if ((pSataChannel == NULL) || (pSataChannel->EdmaActive == MV_FALSE) ||
        (pSataChannel->queuedDMA != MV_EDMA_MODE_NATIVE_QUEUING))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID,MV_DEBUG_FATAL_ERROR,
                 "isGoodCompletionsExpected called in wrong context\n");
        return MV_FALSE;
    }
    pEntry = pSataChannel->commandsQueueHead;
    while (pEntry != NULL)
    {
        if (pEntry->isCommandInEdma == MV_TRUE)
        {
            if ((((MV_U16)(1 << pEntry->commandInfo.PMPort)) &
                 pSataChannel->NCQErrHandlingInfo.PortsWithErrors) == 0)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         " %d %d: isGoodCompletionsExpected: command (tag %d)"
                         " expected with good completion from port %d. "
                         "PortsWithErros= 0x%04x \n",
                         pSataChannel->mvSataAdapter->adapterId,
                         pSataChannel->channelNumber, pEntry->commandTag,
                         pEntry->commandInfo.PMPort,
                         pSataChannel->NCQErrHandlingInfo.PortsWithErrors);
                return MV_TRUE;
            }
        }
        else
        {
            /* stop once reached a command that has not been inserted into the
                EDMA since the next commands also must be outside the EDMA
            */
            break;
        }
        pEntry = pEntry->next;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " %d %d: isGoodCompletionsExpected: No commands expected to be "
             "completed. PortrNumDevError 0x%04x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber,
             pSataChannel->NCQErrHandlingInfo.PortsWithErrors);
    return MV_FALSE;
}
/* this function used for NCQ error handling, this function called wheb DevErr
    interrupt receivedm it checks which PM ports repored device error and updates
    PortsWithErrors varibles
*/

static MV_VOID updatePortsWithErrors(MV_SATA_CHANNEL *pSataChannel)
{
    MV_U32  testCtrlReg = MV_REG_READ_DWORD(pSataChannel->mvSataAdapter->adapterIoBaseAddress,
                                            pSataChannel->eDmaRegsOffset +
                                            MV_SATA_II_IF_TEST_CTRL_REG_OFFSET);
    testCtrlReg &= 0xFFFF0000;
    testCtrlReg = testCtrlReg >> 16;
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " %d %d: updatePortsWithErrors: old val 0x%04x, new 0x%04x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber,
             pSataChannel->NCQErrHandlingInfo.PortsWithErrors,
             pSataChannel->NCQErrHandlingInfo.PortsWithErrors | testCtrlReg);

    pSataChannel->NCQErrHandlingInfo.PortsWithErrors |= (MV_U16)testCtrlReg;
}
/* this function used for NCQ error handling, called when device error received
   and no further good complitions expected. it stops the EDMA and starts the
   process if sending ReadLogExt commands to the drives that reported device
   errors
*/
static MV_VOID enterRequestSenseState(MV_SATA_CHANNEL *pSataChannel)
{
    MV_U8 ATAstatus;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " %d %d: enterRequestSenseState: PortsWithErrors 0x%04x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber,
             pSataChannel->NCQErrHandlingInfo.PortsWithErrors);

    deactivateEdma(pSataChannel->mvSataAdapter, pSataChannel->channelNumber);
    /* clear Device errors in EDMA error cause register due to the aborted
     commands*/
    MV_REG_WRITE_DWORD(pSataChannel->mvSataAdapter->adapterIoBaseAddress,
                       pSataChannel->eDmaRegsOffset +
                       MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET,~MV_BIT2);
    /* the EDMA may be disabled after FPDMA commands issued and before */
    /* receiving response from the drive (D2H Fis), in this case the ATA  */
    /* busy bit will be set, so we wait for this bit to be cleared by the */
    /* drive when is sends D2H registers Fis and SaDevInterrupt will be issued*/

    /* clear SaDevInterrupt if already received*/
    {
        MV_U8       port = pSataChannel->channelNumber & (MV_BIT0 | MV_BIT1);
        MV_U8       sataUnit = (pSataChannel->channelNumber & MV_BIT2) >> 2;

        MV_REG_WRITE_DWORD(pSataChannel->mvSataAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET, ~(MV_BIT8 << port));
    }

    ATAstatus = MV_REG_READ_BYTE(pSataChannel->mvSataAdapter->adapterIoBaseAddress,
                                 pSataChannel->eDmaRegsOffset +
                                 MV_ATA_DEVICE_STATUS_REG_OFFSET);

    if (ATAstatus & MV_ATA_BUSY_STATUS)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: ATA Busy"
                 "bit is set after disabling EDMA, wait for SaDevInterrupt\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        pSataChannel->NCQErrHandlingInfo.state = MV_NCQ_ERROR_HANDLING_STATE_WAIT_FOR_BUSY;
        return;
    }
    pSataChannel->NCQErrHandlingInfo.state = MV_NCQ_ERROR_HANDLING_STATE_REQUEST_SENSE;
    pSataChannel->NCQErrHandlingInfo.CurrPort = 0;
    setReadLogExtCmndPointers(pSataChannel);
    handlePortNCQError(pSataChannel);
}



/* this function used for NCQ error handling, called from the ReadLogExt command
    callback function, it makes sanity checks for the command output and
    completes the erring command with the ATA registers values, finally it calls
    handlePortNCQError to handle NCQ errors from the nexr drive if any*/
static MV_BOOLEAN parseReadLogExtOutPut(MV_SATA_CHANNEL *pSataChannel)
{
    MV_U32  count;
    MV_U8   tag;
    MV_STORAGE_DEVICE_REGISTERS registerStruct;
    MV_QUEUED_COMMAND_ENTRY       *pCommandEntry;

    MV_U8_PTR ReadLogExtBuffer = (MV_U8_PTR)pSataChannel->NCQErrHandlingInfo.ReadLogExtBuffer;
    /* chack CRC*/
    {
        MV_U8 crc = 0;
        for (count = 0 ; count < ATA_SECTOR_SIZE ; count ++)
        {
            crc += ReadLogExtBuffer[count];
        }
        if (crc != 0)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: parseReadLogExtOutPut "
                     "ATA Command failed due to wrong CRC checksum (%02x)\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber,crc);
            return MV_FALSE;
        }
    }
    /* Swap to little endianess */
    for (count = 0 ; count < ATA_SECTOR_SIZE_IN_WORDS; count++)
    {
        /* CPU to little*/
        pSataChannel->NCQErrHandlingInfo.ReadLogExtBuffer[count] =
        MV_LE16_TO_CPU(pSataChannel->NCQErrHandlingInfo.ReadLogExtBuffer[count]);
    }

    for (count = 0; count < 3; count++)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " ReadLogExt: 0x%02x  0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                 ReadLogExtBuffer[(count * 6) + 0],
                 ReadLogExtBuffer[(count * 6) + 1],
                 ReadLogExtBuffer[(count * 6) + 2],
                 ReadLogExtBuffer[(count * 6) + 3],
                 ReadLogExtBuffer[(count * 6) + 4],
                 ReadLogExtBuffer[(count * 6) + 5]);
    }
    /* check NQ bit*/
    if (ReadLogExtBuffer[0] & MV_BIT7)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: parseReadLogExtOutPut: "
                 "Error - NQ is set\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        return MV_FALSE;
    }
    tag = 0x1F & ReadLogExtBuffer[0];

    if (tag >= MV_SATA_SW_QUEUE_SIZE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: parseReadLogExtOutPut: "
                 "Error - None Valid tag (0x%02x)\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber, tag);
        return MV_FALSE;
    }
    pCommandEntry = &pSataChannel->commandsQueue[tag];
    if (pCommandEntry->isFreeEntry == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: parseReadLogExtOutPut: "
                 "Error - No command with tag (0x%02x) has been issued\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber, tag);
        return MV_FALSE;
    }
    if (pCommandEntry->commandInfo.PMPort != pSataChannel->NCQErrHandlingInfo.CurrPort)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: parseReadLogExtOutPut: "
                 "Error - command PM Port (0x%02x) and CurrPort (0x%02x) doesn't match\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber, pCommandEntry->commandInfo.PMPort,
                 pSataChannel->NCQErrHandlingInfo.CurrPort);
        return MV_FALSE;
    }
    if (pCommandEntry->commandInfo.type != MV_QUEUED_COMMAND_TYPE_UDMA)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: parseReadLogExtOutPut: "
                 "Error - command with tag (0x%02x) isn't UDMA command\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber, tag);
        return MV_FALSE;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: parseReadLogExtOutPut: "
             " command tag (0x%02x)\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, tag);
    registerStruct.statusRegister = ReadLogExtBuffer[2];
    registerStruct.deviceRegister = ReadLogExtBuffer[7];
    registerStruct.errorRegister = ReadLogExtBuffer[3];
    registerStruct.lbaLowRegister = (ReadLogExtBuffer[8] << 8) | ReadLogExtBuffer[4];
    registerStruct.lbaMidRegister = (ReadLogExtBuffer[9] << 8) | ReadLogExtBuffer[5];
    registerStruct.lbaHighRegister = (ReadLogExtBuffer[10] << 8) | ReadLogExtBuffer[6];
    registerStruct.sectorCountRegister = (ReadLogExtBuffer[13] << 8) | ReadLogExtBuffer[12];

    _printATARegs(&registerStruct);

    pSataChannel->EdmaQueuedCommands--;
    pCommandEntry->commandInfo.commandParams.udmaCommand.callBack(pSataChannel->mvSataAdapter,
                                                                  pSataChannel->channelNumber,
                                                                  MV_COMPLETION_TYPE_ERROR,
                                                                  pCommandEntry->commandInfo.commandParams.udmaCommand.commandId,
                                                                  0x04,
                                                                  0, &registerStruct);
    removeCommand(pSataChannel,pCommandEntry);
    return MV_TRUE;
}
/* this function used for NCQ error handling, it's the callback function of the
    ReadLogExt command with issued by adding command entry to the channel's
    commands queue */
static MV_BOOLEAN
ReadLogExtCompletionCB(MV_SATA_ADAPTER *pSataAdapter,
                       MV_U8 channelNum,
                       MV_COMPLETION_TYPE comp_type,
                       MV_VOID_PTR commandId,
                       MV_U16 responseFlags,
                       MV_U32 timeStamp,
                       MV_STORAGE_DEVICE_REGISTERS *registerStruct)
{
    MV_SATA_CHANNEL *pSataChannel = pSataAdapter->sataChannel[channelNum];

    switch (comp_type)
    {
    case MV_COMPLETION_TYPE_NORMAL:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " %d %d: ReadLogExtCompletionCB: Normal completion. Port 0x%02x\n",
                 pSataAdapter->adapterId, channelNum,
                 pSataChannel->NCQErrHandlingInfo.CurrPort);
        if (parseReadLogExtOutPut(pSataChannel) == MV_TRUE)
        {
            pSataChannel->NCQErrHandlingInfo.CurrPort++;
            handlePortNCQError(pSataChannel);
        }
        break;
    default:
        /* when ReadLogExt fails or parseReaDLogExtOutPut fails do nothing*/
        /* the higher layers will not have the queued commands completed so it*/
        /* should recover this situation by it's timeout error recovery*/
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " %d %d: ReadLogExtCompletionCB: Bad completion. Port 0x%02x\n",
                 pSataAdapter->adapterId, channelNum,
                 pSataChannel->NCQErrHandlingInfo.CurrPort);
        _printATARegs(registerStruct);
        _dumpSataRegs(pSataAdapter, channelNum);
        break;
    }
    return MV_TRUE;
}
/* this function used for NCQ error handling, it "allocates" command entry of the
    ReadLogExt command and data buffer used for that command from the EDMA
    requests queue which is not used meanwhile since the EDMA disabled*/
static MV_VOID setReadLogExtCmndPointers(MV_SATA_CHANNEL *pSataChannel)
{
    struct ReadLogExtBuffers
    {
        MV_QUEUED_COMMAND_ENTRY entry;
        MV_U16                  pioBuffer[ATA_SECTOR_SIZE_IN_WORDS];
    };
    /* EDMA is not active, so we use the request queue buffer for Read Log Ext
        command data */
    struct ReadLogExtBuffers *pReadLogExtBuffers =
    (struct ReadLogExtBuffers *)pSataChannel->requestQueue;
    pSataChannel->NCQErrHandlingInfo.pReadLogExtEntry = &pReadLogExtBuffers->entry;
    pSataChannel->NCQErrHandlingInfo.ReadLogExtBuffer = pReadLogExtBuffers->pioBuffer;
}
/* this function used for NCQ error handling, it sets the ReadLogExt command
    entry, then issues the command to the CuttPort*/
static MV_VOID insertReadLogExtCmnd(MV_SATA_CHANNEL *pSataChannel)
{
    MV_QUEUED_COMMAND_ENTRY *pEntry = pSataChannel->NCQErrHandlingInfo.pReadLogExtEntry;
    MV_NONE_UDMA_COMMAND_PARAMS *pReadLogExtPIOParams  =
    &pEntry->commandInfo.commandParams.NoneUdmaCommand;
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " %d %d: insertReadLogExtCmnd: Port 0x%02x\n",
             pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber,
             pSataChannel->NCQErrHandlingInfo.CurrPort);

    pReadLogExtPIOParams->bufPtr = pSataChannel->NCQErrHandlingInfo.ReadLogExtBuffer;
    pReadLogExtPIOParams->callBack = ReadLogExtCompletionCB;
    pReadLogExtPIOParams->command = MV_ATA_COMMAND_READ_LOG_EXT;
    pReadLogExtPIOParams->commandId = NULL;
    pReadLogExtPIOParams->count = ATA_SECTOR_SIZE_IN_WORDS;
    pReadLogExtPIOParams->device = 0;
    pReadLogExtPIOParams->features = 0;
    pReadLogExtPIOParams->isEXT = MV_TRUE;
    pReadLogExtPIOParams->lbaHigh = 0;
    pReadLogExtPIOParams->lbaLow = 0x10;
    pReadLogExtPIOParams->lbaMid = 0;
    pReadLogExtPIOParams->protocolType = MV_NON_UDMA_PROTOCOL_PIO_DATA_IN;
    pReadLogExtPIOParams->sectorCount = 1;
    pEntry->commandInfo.type = MV_QUEUED_COMMAND_TYPE_NONE_UDMA;
    pEntry->commandInfo.PMPort = pSataChannel->NCQErrHandlingInfo.CurrPort;
    pEntry->isCommandInEdma = MV_FALSE;
    pEntry->isFreeEntry = MV_FALSE;
    pEntry->commandTag = 0xFF;
    commandsQueueAddHead(pSataChannel, pEntry);
    if (pSataChannel->PMSupported == MV_TRUE)
    {
        _setActivePMPort(pSataChannel, pEntry->commandInfo.PMPort);

    }
    if (sendNoneUdmaCommand(pSataChannel, pEntry) == MV_FALSE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: Failed to "
                 "Issue ReadLogExt PIO command\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        completePIOCommand(pSataChannel, pSataChannel->commandsQueueHead,
                           MV_TRUE);
    }
}
/* this function used for NCQ error handling, it checks the comming port
    that experienced NCQ device error starting from CurrPort, if no port found,
    it sets the NCQ error handling state to the Idle state and re-queues the
    outstanding commands*/
static MV_VOID handlePortNCQError(MV_SATA_CHANNEL *pSataChannel)
{
    while (pSataChannel->NCQErrHandlingInfo.CurrPort <= MV_SATA_PM_MAX_PORTS)
    {
        if (((MV_U16)( 1 << pSataChannel->NCQErrHandlingInfo.CurrPort)) & pSataChannel->NCQErrHandlingInfo.PortsWithErrors)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                     " %d %d: handlePortNCQError: NCQ Error found on Port "
                     "0x%02x\n", pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber,
                     pSataChannel->NCQErrHandlingInfo.CurrPort);
            break;
        }
        pSataChannel->NCQErrHandlingInfo.CurrPort++;
    }
    if (pSataChannel->NCQErrHandlingInfo.CurrPort > MV_SATA_PM_MAX_PORTS)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " %d %d: handlePortNCQError: Finished All erring ports\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber);
        pSataChannel->NCQErrHandlingInfo.state = MV_NCQ_ERROR_HANDLING_STATE_IDLE;
        pSataChannel->NCQErrHandlingInfo.PortsWithErrors = 0;
        _resetEdmaQPointers(pSataChannel);
        _insertQCommandsIntoEdma(pSataChannel);
        return;
    }
    insertReadLogExtCmnd(pSataChannel);
}
static MV_BOOLEAN sendNoneUdmaCommand(MV_SATA_CHANNEL *pSataChannel,
                                      MV_QUEUED_COMMAND_ENTRY *pCommandEntry)
{
    MV_NONE_UDMA_COMMAND_PARAMS *pParams =
    &pCommandEntry->commandInfo.commandParams.NoneUdmaCommand;
    MV_BUS_ADDR_T   ioBaseAddr = pSataChannel->mvSataAdapter->adapterIoBaseAddress;
    MV_U32          eDmaRegsOffset;
    MV_U8           ATAstatus;
    unsigned int             i;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG|MV_DEBUG_NON_UDMA_COMMAND,
             " %d %d Issue NON UDMA command: protocol(%d) buff %p , words %x ,"
             " features %x , sector count %x , lba %x.%x.%x device %x "
             "command=%x\n", pSataChannel->mvSataAdapter->adapterId,
             pSataChannel->channelNumber, pParams->protocolType,
             pParams->bufPtr, pParams->count,
             pParams->features, pParams->sectorCount,
             pParams->lbaLow, pParams->lbaMid,
             pParams->lbaHigh, pParams->device,
             pParams->command);

    eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

    ATAstatus = MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                                 MV_ATA_DEVICE_STATUS_REG_OFFSET);
    if ((ATAstatus & (MV_ATA_READY_STATUS | MV_ATA_BUSY_STATUS)) !=
        MV_ATA_READY_STATUS)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: storage drive is not"
                 " ready, ATA STATUS=0x%02x\n",
                 pSataChannel->mvSataAdapter->adapterId,
                 pSataChannel->channelNumber, ATAstatus);
        return MV_FALSE;
    }

    if (pParams->isEXT == MV_TRUE)
    {
        MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                          MV_ATA_DEVICE_FEATURES_REG_OFFSET,
                          (pParams->features & 0xff00) >> 8);
        MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                          MV_ATA_DEVICE_SECTOR_COUNT_REG_OFFSET,
                          (pParams->sectorCount & 0xff00) >> 8);
        MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                          MV_ATA_DEVICE_LBA_LOW_REG_OFFSET,
                          (pParams->lbaLow & 0xff00) >> 8);
        MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                          MV_ATA_DEVICE_LBA_MID_REG_OFFSET,
                          (pParams->lbaMid & 0xff00) >> 8);
        MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                          MV_ATA_DEVICE_LBA_HIGH_REG_OFFSET,
                          (pParams->lbaHigh & 0xff00) >> 8);
    }
    else
    {
        if ((pParams->features & 0xff00) ||
            (pParams->sectorCount & 0xff00) ||
            (pParams->lbaLow & 0xff00) ||
            (pParams->lbaMid & 0xff00) ||
            (pParams->lbaHigh & 0xff00))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_NON_UDMA_COMMAND,
                     " %d %d :in Issue NON UDMA command:"
                     " bits[15:8] of register values should be reserved"
                     " Features 0x%02x, SectorCount 0x%02x, LBA Low 0x%02x,"
                     " LBA Mid 0x%02x, LBA High 0x%02x\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber, pParams->features,
                     pParams->sectorCount, pParams->lbaLow,
                     pParams->lbaMid, pParams->lbaHigh);
            return MV_FALSE;
        }
    }

    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_FEATURES_REG_OFFSET, pParams->features & 0xff);
    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_SECTOR_COUNT_REG_OFFSET, pParams->sectorCount & 0xff);
    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_LBA_LOW_REG_OFFSET, pParams->lbaLow & 0xff);
    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_LBA_MID_REG_OFFSET, pParams->lbaMid & 0xff);
    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_LBA_HIGH_REG_OFFSET,    pParams->lbaHigh    &    0xff);
    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_HEAD_REG_OFFSET, pParams->device);

    MV_CPU_WRITE_BUFFER_FLUSH();

    MV_REG_WRITE_BYTE(ioBaseAddr, eDmaRegsOffset +
                      MV_ATA_DEVICE_COMMAND_REG_OFFSET, pParams->command);

    if (pParams->protocolType == MV_NON_UDMA_PROTOCOL_PIO_DATA_OUT)
    {
        MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                         MV_ATA_DEVICE_ALTERNATE_REG_OFFSET);

        /* Wait for the command to complete */
        if (waitWhileStorageDevIsBusy(pSataChannel->mvSataAdapter,
                                      ioBaseAddr, eDmaRegsOffset, 10, 100) ==
            MV_FALSE)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: in Issue PIO "
                     "DATA-OUT command: disk not ready.\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber);
            return MV_FALSE;
        }

        if (pSataChannel->mvSataAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {

            if (waitForDRQ(pSataChannel->mvSataAdapter, ioBaseAddr, eDmaRegsOffset, 500, 10000)
                == MV_FALSE)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: DRQ bit in ATA STATUS"
                         " register is not set\n", pSataChannel->mvSataAdapter->adapterId, pSataChannel->channelNumber);
                return MV_FALSE;
            }
        }
        /* Check the status register on DATA request commands */
        ATAstatus = MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                                     MV_ATA_DEVICE_STATUS_REG_OFFSET);
        if ((ATAstatus & (MV_ATA_DATA_REQUEST_STATUS | MV_ATA_BUSY_STATUS | MV_ATA_ERROR_STATUS)) !=
            MV_ATA_DATA_REQUEST_STATUS)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: in Issue PIO "
                     "DATA-OUT command: Bad ATA STATUS:0x%02x.\n",
                     pSataChannel->mvSataAdapter->adapterId,
                     pSataChannel->channelNumber, ATAstatus);
            return MV_FALSE;
        }

        if (pSataChannel->mvSataAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            /* Perform a dummy read */
            MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                             MV_ATA_DEVICE_STATUS_REG_OFFSET);
            mvMicroSecondsDelay (pSataChannel->mvSataAdapter, 1);
        }
        for (i = 0; i < ATA_SECTOR_SIZE_IN_WORDS; i++)
        {
            MV_REG_WRITE_WORD(ioBaseAddr, eDmaRegsOffset +
                              MV_ATA_DEVICE_PIO_DATA_REG_OFFSET,
                              *pParams->bufPtr++);
            MV_CPU_WRITE_BUFFER_FLUSH();
        }
        pParams->count -= ATA_SECTOR_SIZE_IN_WORDS;
#ifdef MV_SATA_SUPPORT_READ_WRITE_LONG

        /* for Write long only*/
        if (pParams->count == 4)
        {
            if (pSataChannel->mvSataAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
            {
                /* Perform a dummy read */
                MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                                 MV_ATA_DEVICE_STATUS_REG_OFFSET);
                mvMicroSecondsDelay (pSataChannel->mvSataAdapter, 1);
            }
            if (waitWhileStorageDevIsBusy(pSataChannel->mvSataAdapter,
                                          ioBaseAddr, eDmaRegsOffset,
                                          50000, 100) == MV_FALSE)
            {
                return MV_FALSE;
            }
            if (pSataChannel->mvSataAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
            {
                if (waitForDRQ(pSataChannel->mvSataAdapter, ioBaseAddr, eDmaRegsOffset, 50000, 100)
                    == MV_FALSE)
                {
                    return MV_FALSE;
                }
            }
            for (i = 0; i < 4; i++)
            {
                MV_REG_WRITE_WORD(ioBaseAddr, eDmaRegsOffset +
                                  MV_ATA_DEVICE_PIO_DATA_REG_OFFSET,
                                  *pParams->bufPtr++);
                MV_CPU_WRITE_BUFFER_FLUSH();


                ATAstatus = MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset +
                                             MV_ATA_DEVICE_STATUS_REG_OFFSET);
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: Write Long ECC data"
                         " xfered. ATA STATUS:0x%02x.\n",
                         pSataChannel->mvSataAdapter->adapterId,
                         pSataChannel->channelNumber, ATAstatus);
            }
            pParams->count -= 4;
        }
#endif /*MV_SATA_SUPPORT_READ_WRITE_LONG*/
    }
    return MV_TRUE;
}

/*   SATA Core API functions        */

/*******************************************************************************
* mvSataInitAdapter - initialize MV88SX50XX adapter
*
* DESCRIPTION:
*   this function initializes glabal registers that concerns PCI access
*   and Interrupts.
*
* INPUT:
*   *pAdapter   - pointer to the adapter data structure.
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
*
* COMMENTS:
*   The adapter will not be able yet to generate interrupts
*
*******************************************************************************/
MV_BOOLEAN mvSataInitAdapter (MV_SATA_ADAPTER *pAdapter)
{
    MV_U8    sataUnit;
    MV_U8    channelIndex;
    MV_U32  regVal;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataInitAdapter"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }

    if (mvOsSemInit(&pAdapter->semaphore) == MV_FALSE)
    {
        return MV_FALSE;
    }

    if (mvOsSemInit(&pAdapter->interruptsMaskSem) == MV_FALSE)
    {
        return MV_FALSE;
    }
#ifdef MV_SATA_IO_GRANULARITY
    if (mvOsSemInit(&pAdapter->iogSemaphore) == MV_FALSE)
    {
        return MV_FALSE;
    }
#endif

    if (pAdapter->mvSataEventNotify == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d  : Bad pointer for "
                 "mvSataEventNotify function\n", pAdapter->adapterId);
        return MV_FALSE;
    }
    /* Clear main mask register to prevent adapter from generating interrupts */
    pAdapter->mainMask = 0;
    pAdapter->interruptsAreMasked = MV_TRUE;
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_MAIN_INTERRUPT_MASK_REG_OFFSET, pAdapter->mainMask);


    /* Initialize the hardware information */
    pAdapter->chipIs50XXB0 = MV_FALSE;
    pAdapter->chipIs50XXB2 = MV_FALSE;
    pAdapter->chipIs60X1B2 = MV_FALSE;
    pAdapter->chipIs60X1C0 = MV_FALSE;
    pAdapter->numberOfChannels = MV_SATA_CHANNELS_NUM;
    pAdapter->numberOfUnits = MV_SATA_UNITS_NUM;

    switch (pAdapter->pciConfigDeviceId)
    {
    case MV_SATA_DEVICE_ID_5080 :
        pAdapter->sataAdapterGeneration = MV_SATA_GEN_I;
        switch (pAdapter->pciConfigRevisionId)
        {
        case 0x1:
            pAdapter->chipIs50XXB0 = MV_TRUE;
            break;
        case 0x3:
            pAdapter->chipIs50XXB2 = MV_TRUE;
            break;
        default:
            if (pAdapter->pciConfigRevisionId > 0x3)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         " %d : Warning: Future revision ID %02x for Device "
                         "ID %02x.\n",
                         pAdapter->adapterId, pAdapter->pciConfigRevisionId,
                         pAdapter->pciConfigDeviceId);
                pAdapter->chipIs50XXB2 = MV_TRUE;
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                         " %d : Bad revision ID %02x for Device ID %02x\n",
                         pAdapter->adapterId, pAdapter->pciConfigRevisionId,
                         pAdapter->pciConfigDeviceId);
                return MV_FALSE;
            }
        }
        break;
    case MV_SATA_DEVICE_ID_5041 :
    case MV_SATA_DEVICE_ID_5040 :
        pAdapter->numberOfChannels = MV_SATA_PORT_PER_UNIT;
        pAdapter->numberOfUnits = 1;
    case MV_SATA_DEVICE_ID_5081 :
        pAdapter->sataAdapterGeneration = MV_SATA_GEN_I;
        switch (pAdapter->pciConfigRevisionId)
        {
        case 0x0:
            pAdapter->chipIs50XXB0 = MV_TRUE;
            break;
        case 0x3:
            pAdapter->chipIs50XXB2 = MV_TRUE;
            break;
        default:
            if (pAdapter->pciConfigRevisionId > 0x3)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d : Warning: Future revis"
                         "ion ID %02x.\n",
                         pAdapter->adapterId, pAdapter->pciConfigRevisionId);
                pAdapter->chipIs50XXB2 = MV_TRUE;
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d : Bad revisi"
                         "on ID %02x\n",    pAdapter->adapterId,
                         pAdapter->pciConfigRevisionId);
                return MV_FALSE;
            }
        }
        break;
    case MV_SATA_DEVICE_ID_6041 :
        pAdapter->numberOfChannels = MV_SATA_PORT_PER_UNIT;
        pAdapter->numberOfUnits = 1;
    case MV_SATA_DEVICE_ID_6081 :
        pAdapter->sataAdapterGeneration = MV_SATA_GEN_II;
        switch (pAdapter->pciConfigRevisionId)
        {
        case 0x7:/*B2*/
            pAdapter->chipIs60X1B2 = MV_TRUE;
            break;
        case 0x9:/*C0*/
            pAdapter->chipIs60X1C0 = MV_TRUE;
            break;
        default:
            if (pAdapter->pciConfigRevisionId > 0x9)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d : Warning:"
                         " Future revision ID %02x.\n", pAdapter->adapterId,
                         pAdapter->pciConfigRevisionId);
                pAdapter->chipIs60X1C0 = MV_TRUE;
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d : Bad revisi"
                         "on ID %02x\n", pAdapter->adapterId,
                         pAdapter->pciConfigRevisionId);
                return MV_FALSE;
            }
        }
        break;

    default:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d : Bad device ID"
                 " 0x%04x\n", pAdapter->adapterId,
                 pAdapter->pciConfigDeviceId);
        return MV_FALSE;







    }

    /*
     * Save the PRE and AMP in the adapter. Also set staggared spin up to be
     * disabled on default (60x1 only).
     */
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d : Saving PRE and AMP values\n",
             pAdapter->adapterId);
    for (channelIndex = 0; channelIndex < pAdapter->numberOfChannels; channelIndex++)
    {
        MV_U32  PHYModeRegister;
        pAdapter->staggaredSpinup[channelIndex] = MV_FALSE;
        pAdapter->ifSpeed[channelIndex] = MV_SATA_IF_SPEED_NO_LIMIT;
        pAdapter->limitInterfaceSpeed[channelIndex] = MV_FALSE;
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
        {
            PHYModeRegister = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                                MV_SATAHC_REGS_BASE_OFFSET((channelIndex & MV_BIT2) >> 2) +
                                                MV_SATA_I_HC_PHY_MODE_BRIDGE_PORT_REG_OFFSET(channelIndex & (MV_BIT0 | MV_BIT1)));
            pAdapter->signalAmps[channelIndex] = (MV_U8)((PHYModeRegister & MV_SATA_I_PHY_MODE_AMP_MASK) >> MV_SATA_I_PHY_MODE_AMP_OFFSET);
            pAdapter->pre[channelIndex] = (MV_U8)((PHYModeRegister & MV_SATA_I_PHY_MODE_PRE_MASK) >> MV_SATA_I_PHY_MODE_PRE_OFFSET);
        }
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {

            MV_U32 resetConfigReg =
            MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                               MV_RESET_CONFIG_REG_OFFSET);

            /*
             * Check if TWSI serial ROM initialization was triggered.
             * If so, then PRE/AMP configuration probably are set after
             * reset by serial ROM. If not then override the PRE/AMP
             * values.
             */
            if (resetConfigReg & MV_RESET_CONFIG_TWSI_INIT_MASK)
            {

                MV_U32 phyMode2Offset = getEdmaRegOffset(channelIndex) +
                                        MV_SATA_II_PHY_MODE_2_REG_OFFSET;
                /* Make sure EDMA is off */
                MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                    getEdmaRegOffset (channelIndex) +
                                    MV_EDMA_COMMAND_REG_OFFSET,
                                    MV_EDMA_COMMAND_DISABLE_MASK);
                MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                   getEdmaRegOffset(channelIndex) +
                                   MV_EDMA_COMMAND_REG_OFFSET);
                regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                           phyMode2Offset);
                pAdapter->signalAmps[channelIndex] =
                (MV_U8)((regVal & MV_SATA_II_PHY_MODE_2_AMP_MASK) >>
                        MV_SATA_II_PHY_MODE_2_AMP_OFFSET);
                pAdapter->pre[channelIndex] =
                (MV_U8)((regVal & MV_SATA_II_PHY_MODE_2_PRE_MASK) >>
                        MV_SATA_II_PHY_MODE_2_PRE_OFFSET);
            }
            else
            {
                pAdapter->signalAmps[channelIndex] = 0x7;
                pAdapter->pre[channelIndex] = 0x1;
            }

        }
    }

    /* Revert the registers to it's default value (software reset) */
    revertSataHCRegs (pAdapter);
    revertFlashInterfaceRegs (pAdapter);
    revertPCIInterfaceRegs(pAdapter);

    /* Enable the SATA LEDs if the silicon revision is B0 */
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d : Enabling SATA LEDS\n",
                 pAdapter->adapterId);
        /* Enable the SATA leds */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_FLASH_GPIO_PORT_CONTROL_OFFSET, 0x0);
        if (pAdapter->chipIs50XXB2 == MV_TRUE)
        {
            regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                       MV_FLASH_GPIO_PORT_CONTROL_OFFSET);
            for (channelIndex = 0 ; channelIndex < pAdapter->numberOfChannels ;
                channelIndex ++)
            {
                if (mvSataIsStorageDeviceConnected (pAdapter,channelIndex) == MV_FALSE)
                {
                    regVal |= (MV_BIT8 << channelIndex);
                }
            }
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                MV_FLASH_GPIO_PORT_CONTROL_OFFSET,
                                regVal);
        }
        /* disable Flash controller clock*/
        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   MV_PCI_REGS_OFFSET +
                                   MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET);

        regVal &= ~(MV_BIT0);
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_PCI_REGS_OFFSET + MV_PCI_EXPANSION_ROM_CONTROL_REG_OFFSET,
                           regVal);
    }

    /* Enable the SATA LEDs for 88SX60X1 devices */
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {

        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d : Enabling SATA LEDS\n",
                 pAdapter->adapterId);
        /* Enable the SATA leds */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_FLASH_GPIO_PORT_CONTROL_OFFSET, 0x00000060);

    }

    /* Check if working in PCI-X mode, then disable all conventional PCI */
    /* features */

    regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_PCI_REGS_OFFSET + MV_PCI_MODE_REG_OFFSET);
    if (((regVal & MV_PCI_MODE_MASK) >> MV_PCI_MODE_OFFSET) != 0) /* PCI-X */
    {
        if (pAdapter->pciCommand & MV_PCI_COMMAND_PCI_CONVENTIONAL_ONLY)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d : disable pci conventio"
                     "nal features when working in PCI-X. pciCommand origin"
                     "al value:0x%08x. new value: 0x%08x.\n",
                     pAdapter->adapterId,
                     pAdapter->pciCommand, pAdapter->pciCommand &
                     (~MV_PCI_COMMAND_PCI_CONVENTIONAL_ONLY));
            pAdapter->pciCommand &= ~MV_PCI_COMMAND_PCI_CONVENTIONAL_ONLY;
        }
        if ((pAdapter->chipIs50XXB0 == MV_TRUE) ||
            (pAdapter->chipIs50XXB2 == MV_TRUE) ||
            (pAdapter->chipIs60X1B2 == MV_TRUE))
        {
            /* PHI1*/
            /* PHII7*/
            if (pAdapter->pciCommand & MV_PCI_MWRITE_COMBINE_BIT)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d  : PCI-X Master"
                         " Write combine enable rejected\n",
                         pAdapter->adapterId);
                pAdapter->pciCommand &= ~MV_PCI_MWRITE_COMBINE_BIT;
            }
        }
    }
    else
    {
        if ((pAdapter->chipIs50XXB0 == MV_TRUE) ||
            (pAdapter->chipIs50XXB2 == MV_TRUE))
        {
            /* PHI2*/
            if (pAdapter->pciCommand & MV_PCI_MWRITE_COMBINE_BIT)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d  : PCI Master"
                         " Write combine enable rejected\n",
                         pAdapter->adapterId);
                pAdapter->pciCommand &= ~MV_PCI_MWRITE_COMBINE_BIT;
            }
            if (pAdapter->pciCommand & MV_PCI_MREAD_COMBINE_BIT)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d  : PCI Master"
                         " Read combine enable rejected\n",
                         pAdapter->adapterId);
                pAdapter->pciCommand &= ~MV_PCI_MREAD_COMBINE_BIT;
            }
        }
    }

    /* SHII8*/
    for (channelIndex = 0; channelIndex < pAdapter->numberOfChannels; channelIndex++)
    {
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                        getEdmaRegOffset (channelIndex) +
                                        MV_SATA_II_SATA_CONFIG_REG_OFFSET);
            /* SHII8*/
            regVal |= MV_BIT12;

            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                getEdmaRegOffset (channelIndex) +
                                MV_SATA_II_SATA_CONFIG_REG_OFFSET,
                                regVal);
            /* _channelHardReset(pAdapter, channelIndex);*/
        }

        _fixPhyParams(pAdapter, channelIndex);
    }

    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_PCI_REGS_OFFSET + MV_PCI_COMMAND_REG_OFFSET,
                       pAdapter->pciCommand);
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_PCI_REGS_OFFSET + MV_PCI_SERR_MASK_REG_OFFSET,
                       pAdapter->pciSerrMask);

    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_PCI_REGS_OFFSET + MV_PCI_INTERRUPT_MASK_REG_OFFSET,
                       pAdapter->pciInterruptMask);

    for (sataUnit = 0; sataUnit < pAdapter->numberOfUnits; sataUnit++)
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATAHC_INT_COAL_THRE_REG_OFFSET,
                           pAdapter->intCoalThre[sataUnit]);
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATAHC_INT_TIME_THRE_REG_OFFSET,
                           pAdapter->intTimeThre[sataUnit]);







    }
    pAdapter->mainMask = MV_MAIN_INTERRUPT_MASK_ENABLE_ALL;
    pAdapter->interruptsScheme = MV_SATA_INTERRUPT_HANDLING_IN_ISR;

    for (channelIndex = 0; channelIndex < pAdapter->numberOfChannels; channelIndex++)
    {
        unmaskEdmaInterrupts(pAdapter, channelIndex);
    }
    return MV_TRUE;
}

/*******************************************************************************
* mvSataShutDownAdapter - Shuts down adapter.
*
* DESCRIPTION: Shuts down a specific 88SX50xx adapter.
*
* INPUT:
*   *pAdapter   - pointer to the adapter data structure.
*
* OUTPUT:
*   None.
*
* RETURN:
*   MV_TRUE on success, MV_FALSE on failure
*
* COMMENTS:
*   None.
*
*******************************************************************************/

MV_BOOLEAN mvSataShutdownAdapter(MV_SATA_ADAPTER *pAdapter)
{
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataShutdownAdapter Failed, Bad adapter data structure "
                 "pointer\n");
        return MV_FALSE;
    }
    pAdapter->interruptsAreMasked = MV_TRUE;
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_MAIN_INTERRUPT_MASK_REG_OFFSET, 0);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataReadReg - return the value of register.
*
* DESCRIPTION:
*   return the value of a register, which have the offset regOffset, in a
*   MV88SX50XX adapter.
*   Note that if reading from storage device's internal registers and EDMA
*   is enabled, then the read transaction will never complete and possibly
*   cause system hang.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*       regOffset   - offset of the register
*
* RETURN:
*   the register value in 32 bit.
* COMMENTS:
*   NONE
*
*******************************************************************************/
MV_U32     mvSataReadReg(MV_SATA_ADAPTER *pAdapter, MV_U32 regOffset)
{
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataReadReg Failed,"
                 " Bad adapter data structure pointer\n");
        return 0;
    }

    return MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, regOffset);
}

/*******************************************************************************
* mvSataWriteReg - return the value of register.
*
* DESCRIPTION:
*   write the regValue to a register, which have the offset regOffset, in a
*   MV88SX50XX adapter.
*   Note that if writing to storage device's internal registers and EDMA
*   is enabled, then the write transaction will never complete and possibly
*   cause system hang.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*       regOffset   - offset of the register
*   regValue    - the value to write to the register
*
* RETURN:
*   None.
* COMMENTS:
*   for 8 or 16 bit registers the low bits of the regValue should hold the
*   requested value to be written.
*
*******************************************************************************/
MV_VOID mvSataWriteReg(MV_SATA_ADAPTER *pAdapter, MV_U32 regOffset,
                       MV_U32 regValue)
{
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataWriteReg Failed,"
                 " Bad adapter data structure pointer\n");
        return;
    }

    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress, regOffset, regValue);
}
/*******************************************************************************
* mvSataConfigureChannel - configure Sata channel
*
*
* DESCRIPTION:
*   this function configures SATA channel by resetting the low level fields
*   of the channel data structure and configures EDMA regs accourdingly
*
* INPUT:
*   pAdapter   - pointer to the MV88SX50XX adapter data structure
*   channelIndex    - the index of the channel where the response received
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
*
* COMMENTS:
*   None.
*
*******************************************************************************/
MV_BOOLEAN mvSataConfigureChannel(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL     *pSataChannel;
    MV_BOOLEAN          result;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataConfigureChannel"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataConfigureChann"
                 "el Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    if (pAdapter->numberOfChannels <= channelIndex)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataConfigureChann"
                 "el Failed - requsted to configure a non-valid channel\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    pSataChannel->eDmaRegsOffset = getEdmaRegOffset(channelIndex);
    pSataChannel->mvSataAdapter = pAdapter;

    if (mvOsSemInit(&pSataChannel->semaphore)==MV_FALSE)
    {
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);

    /* Sets the request and response queues base addresses */
    pSataChannel->queueCommandsEnabled = MV_FALSE;
    pSataChannel->EdmaActive = MV_FALSE;
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_UNKNOWN;
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        pSataChannel->PMSupported = MV_TRUE;
    }
    else
    {
        pSataChannel->PMSupported = MV_FALSE;
    }
    pSataChannel->DRQDataBlockSize = 1;
    result = resetEdmaChannel(pSataChannel);
    disableSaDevInterrupts(pAdapter, channelIndex);
    mvOsSemRelease(&pSataChannel->semaphore);
    return result;
}

/*******************************************************************************
* mvSataRemoveChannel -
*
* DESCRIPTION:  Removes data structures and other parameters used for the
*   specific SATA channel that is indicated by pAdapter and channelIndex.
*
* INPUT:
*   pAdapter - A pointer to an MV_SATA_ADAPTER data structure that holds
*                information to access the 88SX50xx device.
*   channelIndex - An index to a specific 88SX50xx channel.
* OUTPUT:
*   None.
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN mvSataRemoveChannel(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL     *pSataChannel;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataRemoveChannel"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    mvOsSemTake(&pAdapter->semaphore);
    pSataChannel = pAdapter->sataChannel[channelIndex];

    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataRemoveChannel"
                 " Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pAdapter->semaphore);
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataRemoveChannel "
                 "failed, DMA is enabled\n", pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        mvOsSemRelease(&pAdapter->semaphore);
        return MV_FALSE;
    }
    mvOsSemRelease(&pSataChannel->semaphore);
    mvOsSemRelease(&pAdapter->semaphore);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataIsStorageDeviceConnected - Check if storage device is connected to a
*                                  SATA channel.
*
* DESCRIPTION:
*       This function reads the DET field from the R00 status bridge register
*       of the corresponding channel.
*
* INPUT:
*       pAdapter     - Pointer to the device data structure.
*       channelIndex - Index of the required channel
*
* RETURN:
*       MV_TRUE when storage device is connected, MV_FALSE otherwise( also when
*       loopback mode is used).
*
* COMMENTS:
*       None
*
*******************************************************************************/
MV_BOOLEAN mvSataIsStorageDeviceConnected(MV_SATA_ADAPTER *pAdapter,
                                          MV_U8 channelIndex)
{
    MV_U32  SStatusOffset = 0;
    MV_U32  det;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataIsStorageDeviceConnected Failed, Bad adapter data"
                 " structure pointer\n");
        return MV_FALSE;
    }

    if (pAdapter->numberOfChannels <= channelIndex)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: "
                 "mvSataIsStorageDeviceConnected Failed, Bad channelIndex "
                 " input on a 0x%x adapterstructure pointer\n",
                 pAdapter->adapterId,channelIndex,pAdapter->pciConfigDeviceId);
        return MV_FALSE;
    }

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        MV_U8   sataUnit = (channelIndex & MV_BIT2) >> 2;
        MV_U8   port = channelIndex & (MV_BIT0 | MV_BIT1);
        SStatusOffset = MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                        MV_SATA_I_HC_R00_STATUS_BRIDGE_PORT_OFFSET(port);
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        SStatusOffset = getEdmaRegOffset(channelIndex) +
                        MV_SATA_II_S_STATUS_REG_OFFSET;
        if (pAdapter->staggaredSpinup[channelIndex] == MV_FALSE)
        {
            return MV_FALSE;
        }
    }

    det = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                             SStatusOffset) & 0xf;

    switch (det)
    {
    case MV_PHY_DET_STATE_NO_DEVICE:
        break;
    case MV_PHY_DET_STATE_DEVICE_NO_PHY_COM:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " %d %d: Detection value is 1\n", pAdapter->adapterId,
                 channelIndex);

        MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                          SStatusOffset);
        mvMicroSecondsDelay(pAdapter, MV_PHY_COM_SETUP_WAIT);
        det = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                SStatusOffset) & 0xf;
        if (det != MV_PHY_DET_STATE_DEVICE_AND_PHY_COM)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: Failed to"
                     " establish SATA PHY communication\n",
                     pAdapter->adapterId, channelIndex);
            break;
        }
    case MV_PHY_DET_STATE_DEVICE_AND_PHY_COM:
        return MV_TRUE;
    case MV_PHY_DET_STATE_PHY_OFFLINE:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INIT, " %d %d: WARNING: SATA PHY is "
                 "offline\n", pAdapter->adapterId, channelIndex);
        break;
    default:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: unknow value for\
                      R00 Status Bridge reg bits[3:0] (0x%02x)\n",
                 pAdapter->adapterId, channelIndex, det);
    }
    return MV_FALSE;
}


static MV_BOOLEAN _checkSStatusAfterHReset(MV_SATA_ADAPTER* pAdapter,
                                           MV_U8 channelIndex)
{
    MV_U32 SStatusOffset;
    MV_U32  SStatus;
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        /*  Get DET field in SControl register to 1 */
        MV_U8       port = channelIndex & (MV_BIT0 | MV_BIT1);
        MV_U8       sataUnit = (channelIndex & MV_BIT2) >> 2;
        SStatusOffset = MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                        MV_SATA_I_HC_R00_STATUS_BRIDGE_PORT_OFFSET(port);
    }
    else
    {
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            SStatusOffset = getEdmaRegOffset(channelIndex) +
                            MV_SATA_II_S_STATUS_REG_OFFSET;
        }
        else
        {
            return MV_TRUE;
        }
    }
    SStatus = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                SStatusOffset);
    SStatus &= (MV_BIT0 | MV_BIT1);
    if ((SStatus == (MV_BIT0 | MV_BIT1)) || (SStatus == 0))
    {
        return MV_TRUE;
    }
    else
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: in Channel Hard "
                 "Reset SATA communication not established.\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
}


/*******************************************************************************
* mvSataChannelHardReset - issue channel SATA HARD reset.
*
* DESCRIPTION:
*   perform HARDWARE RESET to the connected device by asserting the RESET
*   signal, this is done by setting the hardware reset bit of the EDMA
*   command register
*
* INPUT:
*   pAdapter    - pointer to the device data structure.
*   channelIndex    - index of the required channel
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*   NONE
*
*******************************************************************************/
MV_BOOLEAN mvSataChannelHardReset(MV_SATA_ADAPTER *pAdapter,
                                  MV_U8 channelIndex)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32  eDmaRegsOffset;
    MV_U32      count = 0;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataChannelHardReset"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: "
                 "mvSataChannelHardReset Failed, channel data structure not "
                 "allocated\n", pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: Error:\
                 mvSataIChannelHardReset called while EDMA is active\n",
                 pAdapter->adapterId,channelIndex);
        mvOsSemRelease( &pSataChannel->semaphore);
        return MV_FALSE;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: Issue HRST\n", pAdapter->adapterId,
             channelIndex);


    _channelHardReset(pAdapter, channelIndex);
    if ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_II) &&
        (pAdapter->staggaredSpinup[channelIndex] == MV_FALSE))
    {
        mvOsSemRelease( &pSataChannel->semaphore);
        return MV_TRUE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        while (1)
        {
            _establishSataComm(pAdapter, channelIndex);

            /* try the DET fix 3 times */
            if (_checkSStatusAfterHReset(pAdapter, channelIndex) == MV_FALSE)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: in Channel Hard "
                         "Reset storage drive is not ready, try fix #%d\n",
                         pAdapter->adapterId, channelIndex, count);

                count++;
                if (count == 3)
                {
                    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: in Channel H"
                             "ard Reset storage drive is not ready after 3 tries\n",
                             pAdapter->adapterId, channelIndex);
                    mvOsSemRelease( &pSataChannel->semaphore);
                    return MV_FALSE;
                }
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_SATA_LINK, " %d %d: Disk is Ready After"
                         " Hard Reset\n", pAdapter->adapterId, channelIndex);
                break;
            }
        }
    }

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        _establishSataComm(pAdapter, channelIndex);
    }
    mvOsSemRelease( &pSataChannel->semaphore);
    return MV_TRUE;
}
/*******************************************************************************
* mvSataConfigEdmaMode - set EDMA operating mode.
*
* DESCRIPTION:
*   Set the EDMA operating mode - MV_EDMA_MODE_NOT_QUEUED,
*   MV_EDMA_MODE_QUEUED or MV_EDMA_MODE_NATIVE_QUEUING.
*   When set toe MV_EDMA_MODE_QUEUED or MV_EDMA_MODE_NATIVE_QUEUING then
*   the maxQueueDepth should be valud.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*   eDmaMode    - the selected mode
*   maxQueueDepth   - the maximum depth of the queue
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
*
* COMMENTS:
*   None.
*
*******************************************************************************/
MV_BOOLEAN mvSataConfigEdmaMode(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                                MV_EDMA_MODE eDmaMode, MV_U8 maxQueueDepth)
{
    MV_SATA_CHANNEL  *pSataChannel;
    MV_BUS_ADDR_T  ioBaseAddr;
    MV_U32 eDmaRegsOffset;
    MV_U32     val;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataConfigEdmaMode"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataConfigEdmaMode"
                 " Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    mvOsSemTake(&pSataChannel->semaphore);
    eDmaRegsOffset = pSataChannel->eDmaRegsOffset;

    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," %d %d: mvSataConfigEdmaMode failed,"
                 " EDMA is enabled\n", pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }

    val = MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                            MV_EDMA_CONFIG_REG_OFFSET);
    if (eDmaMode == MV_EDMA_MODE_NATIVE_QUEUING)
    {
        if ((maxQueueDepth > MV_EDMA_QUEUE_LENGTH) ||  (maxQueueDepth == 0))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR," %d %d: mvSataConfigEdmaMode "
                     "failed, Bad queue depth(%d)\n", pAdapter->adapterId,
                     channelIndex, maxQueueDepth);
            mvOsSemRelease(&pSataChannel->semaphore);
            return MV_FALSE;
        }
        if (pAdapter->sataAdapterGeneration != MV_SATA_GEN_II)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: Trying to configure "
                     "EDMA to NCQ on a non NCQ enabled EDMA\n",
                     pAdapter->adapterId, channelIndex);
            mvOsSemRelease(&pSataChannel->semaphore);
            return MV_FALSE;
        }
        val &= ~MV_EDMA_CONFIG_Q_DEPTH_MASK; /* clear queue depth */
        /* set the NCQ enable mode bit, and the queue depth bits*/
        val |= MV_EDMA_CONFIG_NATIVE_QUEUING_MASK | (maxQueueDepth - 1);
        val |= MV_EDMA_CONFIG_CONONDEVERR_MASK;

        /* unmask DevErr */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           getEdmaRegOffset(channelIndex) +
                           MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET,
                           MV_EDMA_GEN_II_ERROR_MASK | MV_BIT2);
    }
    else
    {
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               getEdmaRegOffset(channelIndex) +
                               MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET,
                               MV_EDMA_GEN_II_ERROR_MASK);
        }
    }

    if (eDmaMode == MV_EDMA_MODE_QUEUED)
    {
        if ((maxQueueDepth > MV_EDMA_QUEUE_LENGTH) ||  (maxQueueDepth == 0))
        {
            mvOsSemRelease(&pSataChannel->semaphore);
            return MV_FALSE;
        }
        val &= ~MV_EDMA_CONFIG_Q_DEPTH_MASK; /* clear queue depth */
        val &= ~MV_EDMA_CONFIG_NATIVE_QUEUING_MASK; /* clear NCQ mode*/
        val &= ~MV_EDMA_CONFIG_CONONDEVERR_MASK;
        /* set the queue enable mode bit, and the queue depth bits*/
        val |= MV_EDMA_CONFIG_EQUEUE_ENABLED_MASK | (maxQueueDepth - 1);
    }

    if (eDmaMode == MV_EDMA_MODE_NOT_QUEUED)
    {
        val &= ~MV_EDMA_CONFIG_Q_DEPTH_MASK; /* clear queue depth */
        val &= ~MV_EDMA_CONFIG_NATIVE_QUEUING_MASK; /* clear NCQ mode*/
        val &= ~MV_EDMA_CONFIG_EQUEUE_ENABLED_MASK;
        val &= ~MV_EDMA_CONFIG_CONONDEVERR_MASK;
    }
    pSataChannel->queuedDMA = eDmaMode;



    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        val |= MV_EDMA_CONFIG_BURST_SIZE_MASK;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        val |= (MV_EDMA_CONFIG_BURST_SIZE_EXT_MASK | MV_BIT13);
    }

    MV_REG_WRITE_DWORD(ioBaseAddr, eDmaRegsOffset + MV_EDMA_CONFIG_REG_OFFSET,
                       val);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG," %d %d: Edma Configuration Reg Value: 0x%08x\n",
             pAdapter->adapterId, channelIndex,
             MV_REG_READ_DWORD(ioBaseAddr, eDmaRegsOffset +
                               MV_EDMA_CONFIG_REG_OFFSET));

    mvOsSemRelease(&pSataChannel->semaphore);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataEnableChannelDma - enable EDMA engine
*
* DESCRIPTION:
*   enable the EDMA engine for the given channel
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*   NONE
*
*******************************************************************************/
MV_BOOLEAN mvSataEnableChannelDma(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL  *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataEnableChannelDma"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataEnableChannelD"
                 "ma Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex );
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);

    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataEnableChannelDma "
                 "failed, DMA is enabled\n", pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }
#ifdef MV_SATA_C2C_COMM
    /* C2C */
    if (pSataChannel->C2CmodeEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataEnableChannelDma "
                 "failed, C2C mode is enabled\n", pAdapter->adapterId,
                 channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }
#endif
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: mvSataEnableChannelDma\n",
             pAdapter->adapterId, channelIndex);
    pSataChannel->queueCommandsEnabled = MV_TRUE;
    activateEdma(pAdapter,channelIndex);

    mvOsSemRelease(&pSataChannel->semaphore);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataDisableChannelDma - disable EDMA engine
*
* DESCRIPTION:
*   disable the EDMA engine for the given channel
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*   NONE
*
*******************************************************************************/
MV_BOOLEAN mvSataDisableChannelDma(MV_SATA_ADAPTER *pAdapter,
                                   MV_U8 channelIndex)
{
    MV_BUS_ADDR_T   ioBaseAddr;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataDisableChannelDma"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    if (pAdapter->sataChannel[channelIndex] == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataDisableChannel"
                 "Dma Failed, channel data structure is not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: mvSataDisableChannel\n",
             pAdapter->adapterId, channelIndex);
    mvOsSemTake(&pAdapter->sataChannel[channelIndex]->semaphore);
    pAdapter->sataChannel[channelIndex]->queueCommandsEnabled = MV_FALSE;
    deactivateEdma(pAdapter,channelIndex);
    mvOsSemRelease(&pAdapter->sataChannel[channelIndex]->semaphore);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataFlushDmaQueue - flush the outstanding UDMA commands
*
* DESCRIPTION:
*   flush posted UDMA ATA commands on a certain MV88SX50XX SATA channel. if
*   the flush type is MV_FLUSH_TYPE_CALLBACK then all call back functions of
*   the UDMA commands are called with a flush indication.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*   flushType   - indicates wheather to call the callBack function or not.
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*   NONE
*
*******************************************************************************/
MV_BOOLEAN mvSataFlushDmaQueue(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                               MV_FLUSH_TYPE flushType)
{
    MV_SATA_CHANNEL *pSataChannel;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataFlushDmaQueue"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataFlushDmaQueue "
                 "Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataFlushDmaQueue "
                 "Failed, EDMA not disabled\n",
                 pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }
    flushDmaQueue(pSataChannel, flushType, MV_COMPLETION_TYPE_ABORT, 0);
    resetEdmaChannel(pSataChannel);
    mvOsSemRelease(&pSataChannel->semaphore);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataNumOfDmaCommands - get the number of the outstanding commmands for the
*                           given channel
*
* DESCRIPTION:
*   return the number of posted ATA commands on an EDMA engine for a
*   specific SATA channel.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*
* RETURN:
*   num of queue commands.
* COMMENTS:
*   NONE
*
*******************************************************************************/
MV_U8 mvSataNumOfDmaCommands(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_U8           result;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataNumOfDmaCommands"
                 " Failed, Bad adapter data structure pointer\n");
        return 0xFF;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataNumOfDmaComman"
                 "ds Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return 0xFF;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    result = pSataChannel->outstandingCommands;
    mvOsSemRelease(&pSataChannel->semaphore);

    return result;
}
/*******************************************************************************
* mvSataGetNumOfPortQueuedCommands - get the number of the outstanding commmands for
*                               the given port
*
* DESCRIPTION:
*   return the number of posted ATA commands on an EDMA engine for a
*   specific SATA port.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*   PMPort      - port number
*   pCommandsPerChannel - if not null, gets the total number of outstanding
*                   command for the given channel
*
* RETURN:
*   num of queue commands, 0xFF if error detected.
* COMMENTS:
*
*
*******************************************************************************/
MV_U8 mvSataGetNumOfPortQueuedCommands(MV_SATA_ADAPTER *pAdapter,
                                       MV_U8 channelIndex,
                                       MV_U8 PMPort,
                                       MV_U8 *pCommandsPerChannel)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_U8           result;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataGetNumOfPortQueuedCommands"
                 " Failed, Bad adapter data structure pointer\n");
        return 0xFF;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d %d: mvSataGetNumOfPortQueuedCommands"
                 "ds Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex, PMPort);
        return 0xFF;
    }
    if (PMPort > MV_SATA_PM_MAX_PORTS)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d %d: mvSataGetNumOfPortQueuedCommands"
                 "ds Failed, non valid port\n",
                 pAdapter->adapterId, channelIndex, PMPort);
        return 0xFF;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    result = pSataChannel->portQueuedCommands[PMPort];
    if (pCommandsPerChannel)
    {
        *pCommandsPerChannel = pSataChannel->outstandingCommands;
    }
    mvOsSemRelease(&pSataChannel->semaphore);

    return result;
}

/*******************************************************************************
* mvSataSetIntCoalParams - update the interrupt Coalescing registers
*
* DESCRIPTION:  Sets the interrupt coalescing for a specific SATA unit
*               (each SATA unit contains quad SATA channels).
*
* INPUT:
*   pAdapter - pointer to the adapter data structure.
*   sataUnit - which SATA unit to be changed (0xff for all SATA ports)
*   intCoalThre - the value to be written to the Coalescing threshold register
*   intTimeThre - the value to be written to the Time threshold register
*
* OUTPUT:
*   None.
* RETURN:
*   MV_TRUE
* COMMENTS:
*   None.
*
*******************************************************************************/
MV_BOOLEAN mvSataSetIntCoalParams (MV_SATA_ADAPTER *pAdapter, MV_U8 sataUnit,
                                   MV_U32 intCoalThre, MV_U32 intTimeThre)
{
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataSetIntCoalParams"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }

    if ((sataUnit != 0) && (sataUnit != 1) && (sataUnit != 0xff))
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d  : %d Bad"
                 " unit number\n", pAdapter->adapterId, sataUnit);
        return MV_FALSE;
    }

    mvOsSemTake(&pAdapter->semaphore);
    if (sataUnit == 0xff)
    {
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATA_II_ALL_PORTS_INT_COAL_CMND_THR_REG_OFFSET,
                               intCoalThre);

            pAdapter->intCoalThre[0] = intCoalThre;
            pAdapter->intCoalThre[1] = intCoalThre;
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATA_II_ALL_PORTS_INT_COAL_TIME_THR_REG_OFFSET,
                               intTimeThre);
            pAdapter->intTimeThre[0] = intTimeThre;
            pAdapter->intTimeThre[1] = intTimeThre;
            pAdapter->mainMask |= MV_BIT21;
            pAdapter->mainMask &= ~(MV_BIT17 | MV_BIT8);
        }
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
        {
            MV_U8 count;
            for (count = 0 ; count < pAdapter->numberOfUnits ; count ++)
            {
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   MV_SATAHC_REGS_BASE_OFFSET(count) +
                                   MV_SATAHC_INT_COAL_THRE_REG_OFFSET,
                                   intCoalThre);

                pAdapter->intCoalThre[count] = intCoalThre;
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   MV_SATAHC_REGS_BASE_OFFSET(count) +
                                   MV_SATAHC_INT_TIME_THRE_REG_OFFSET,
                                   intTimeThre);
                pAdapter->intTimeThre[count] = intTimeThre;
            }
        }
    }
    else
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATAHC_INT_COAL_THRE_REG_OFFSET,
                           intCoalThre);

        pAdapter->intCoalThre[sataUnit] = intCoalThre;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATAHC_INT_TIME_THRE_REG_OFFSET,
                           intTimeThre);
        pAdapter->intTimeThre[sataUnit] = intTimeThre;
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            pAdapter->mainMask |= (MV_BIT17 | MV_BIT8);
            pAdapter->mainMask &= ~MV_BIT21;
        }
    }
    if (pAdapter->interruptsAreMasked == MV_FALSE)
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                           pAdapter->mainMask);

        MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                          MV_MAIN_INTERRUPT_MASK_REG_OFFSET);







    }
    mvOsSemRelease(&pAdapter->semaphore);

    return MV_TRUE;
}

/*******************************************************************************
* mvSataSetChannelPhyParams - update the channel's Sata Phy params
*
* DESCRIPTION: This functoin changes the Sata Phy params such as the AMP and
*       PRE by updating the PHY Mode register.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex- index of the Edma channel number.
*   signalAmps  - three bits value to be written to the Phy Mode register at
*                   bits[7:5]
*   pre         - two bits value to be written to the Phy Mode register at
*                   bits[12:11]
*
* OUTPUT:
*   None.
*
* RETURN:
*   MV_TRUE on success, MV_FASLE otherwisw
*
*
*******************************************************************************/
MV_BOOLEAN mvSataSetChannelPhyParams(MV_SATA_ADAPTER *pAdapter,
                                     MV_U8 channelIndex,
                                     MV_U8 signalAmps, MV_U8 pre)
{
    MV_U32          regAddr;
    MV_U32          val;
    MV_U8       port = channelIndex & (MV_BIT0 | MV_BIT1);
    MV_U8       unit = (channelIndex & MV_BIT2) >> 2;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "  : mvSataSetChannelPhyParam"
                 "s Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        if ((signalAmps & 0xf8) || (pre & 0xfc))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: "
                     "mvSataSetChannelPhyParams Failed. Bad params\n",
                     pAdapter->adapterId, channelIndex);
            return MV_FALSE;
        }

        regAddr = MV_SATAHC_REGS_BASE_OFFSET(unit) +
                  MV_SATA_I_HC_PHY_MODE_BRIDGE_PORT_REG_OFFSET(port);

        val = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, regAddr);

        val &= ~MV_SATA_I_PHY_MODE_AMP_MASK;
        val |= (signalAmps << MV_SATA_I_PHY_MODE_AMP_OFFSET) &
               MV_SATA_I_PHY_MODE_AMP_MASK;

        val &= ~MV_SATA_I_PHY_MODE_PRE_MASK;
        val |= (pre << MV_SATA_I_PHY_MODE_PRE_OFFSET) &
               MV_SATA_I_PHY_MODE_PRE_MASK;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress, regAddr, val);
        pAdapter->pre[channelIndex] = pre;
        pAdapter->signalAmps[channelIndex] = signalAmps;
    }

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        if ((signalAmps & 0xf8) || (pre & 0xf8))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: "
                     "mvSataSetChannelPhyParams Failed. Bad params\n",
                     pAdapter->adapterId, channelIndex);
            return MV_FALSE;
        }


        pAdapter->pre[channelIndex] = pre;
        pAdapter->signalAmps[channelIndex] = signalAmps;
        regAddr = getEdmaRegOffset(channelIndex) +
                  MV_SATA_II_PHY_MODE_2_REG_OFFSET;
        val = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, regAddr);

        val &= ~MV_SATA_II_PHY_MODE_2_AMP_MASK;
        val |= (signalAmps << MV_SATA_II_PHY_MODE_2_AMP_OFFSET) &
               MV_SATA_II_PHY_MODE_2_AMP_MASK;
        val &= ~MV_SATA_II_PHY_MODE_2_PRE_MASK;
        val |= (pre << MV_SATA_II_PHY_MODE_2_PRE_OFFSET) &
               MV_SATA_II_PHY_MODE_2_PRE_MASK;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress, regAddr, val);
    }

    return MV_TRUE;
}
/*******************************************************************************
* mvSataChannelPhyShutdown -
*
* DESCRIPTION: Shutdown the sata Phy of the given channel.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise
*
* COMMENTS:
*   After shutdown no connect / disconnect indication will be available.
*
*******************************************************************************/

MV_BOOLEAN mvSataChannelPhyShutdown(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex)
{
    MV_U32      regVal;
    MV_U8       port = channelIndex & (MV_BIT0 | MV_BIT1);
    MV_U8       sataUnit = (channelIndex & MV_BIT2) >> 2;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "  : mvSataChannelPhyShutdown"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                    getEdmaRegOffset (channelIndex) +
                                    MV_SATA_II_SATA_CONFIG_REG_OFFSET);
        /* SHII8*/
        regVal |= MV_BIT12;

        regVal |=  MV_BIT9;
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            getEdmaRegOffset (channelIndex) +
                            MV_SATA_II_SATA_CONFIG_REG_OFFSET,
                            regVal);
        return MV_TRUE;
    }
    regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                               MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET);
    regVal |=  MV_SATA_I_TEST_CONTROL_PHY_SHUTDOWN_MASK(port);

    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                       MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET, regVal);
    MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                      MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                      MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET);
    return MV_TRUE;
}
/*******************************************************************************
* mvSataChannelPhyPowerOn -
*
* DESCRIPTION: power on the sata Phy of the given channel.
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise
*
* COMMENTS:
*   None.
*
*******************************************************************************/

MV_BOOLEAN mvSataChannelPhyPowerOn(MV_SATA_ADAPTER *pAdapter,
                                   MV_U8 channelIndex)
{
    MV_U32      regVal;
    MV_U8       port = channelIndex & (MV_BIT0 | MV_BIT1);
    MV_U8       sataUnit = (channelIndex & MV_BIT2) >> 2;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "  : mvSataChannelPhyPowerOn"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                    getEdmaRegOffset (channelIndex) +
                                    MV_SATA_II_SATA_CONFIG_REG_OFFSET);
        /* SHII8*/
        regVal |= MV_BIT12;

        regVal &= ~(MV_BIT9);
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                            getEdmaRegOffset (channelIndex) +
                            MV_SATA_II_SATA_CONFIG_REG_OFFSET,
                            regVal);
        _fixPhyParams (pAdapter,channelIndex);
        return MV_TRUE;
    }
    _fixPhyParams(pAdapter, channelIndex);
    regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                               MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "  : %d %d %d mvSataChannelPhyPowerOn"
             " reg[%x] = 0x%x -> 0x%x\n", pAdapter->adapterId, sataUnit, port,
             MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
             MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET, regVal,
             regVal & ~(MV_SATA_I_TEST_CONTROL_PHY_SHUTDOWN_MASK(port)));
    regVal &=  ~(MV_SATA_I_TEST_CONTROL_PHY_SHUTDOWN_MASK(port));

    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                       MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET, regVal);
    MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                      MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                      MV_SATA_I_HC_BRIDGES_TEST_CONTROL_REG_OFFSET);{
        MV_U32 temp = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                        MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                        MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port));
        temp &= ~0xf;
        temp |= MV_BIT0;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port), temp);
    }
    _fixPhyParams(pAdapter, channelIndex);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataChannelFarLoopbackDiagnostic - do far end loopback
*
* DESCRIPTION: operate the far-end LB mode on the bridge
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - index of the required channel
*
* OUTPUT:
*   None.
* RETURN:
*   MV_TRUE on success, MV_FALSE otherwise
* COMMENTS:
*   None.
*******************************************************************************/

MV_BOOLEAN mvSataChannelFarLoopbackDiagnostic(MV_SATA_ADAPTER *pAdapter,
                                              MV_U8 channelIndex)
{
    MV_U8   sataUnit = (channelIndex & MV_BIT2) >> 2;
    MV_U8   port = channelIndex & (MV_BIT0 | MV_BIT1);
    MV_U32  regVal, temp;
    MV_U32  tryCount, pollCount;
    MV_BOOLEAN  result = MV_TRUE;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "  : mvSataChannelFarLoopback"
                 "Diagnostic Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        /* TODO - Add support for far end loopback */
        return MV_TRUE;
    }

    for (tryCount = 0; tryCount < 5; tryCount++)
    {

        /* Set Far-end loopback */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATA_I_HC_R04_STATUS_BRIDGE_PORT_OFFSET(port),
                           0x00100000);
        /* BIST pattern */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATA_I_HC_R05_STATUS_BRIDGE_PORT_OFFSET(port),
                           0xb5b5b5b5);
        /* BIST pattern */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATA_I_HC_R06_STATUS_BRIDGE_PORT_OFFSET(port),
                           0xb5b5b5b5);
        /* enable BIST */
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                           MV_SATA_I_HC_R0F_STATUS_BRIDGE_PORT_OFFSET(port),
                           0x00200000);

        mvMicroSecondsDelay(pAdapter, MV_FAR_END_LOOPBACK_TEST_WAIT_TIME);

        /* poll bit 20 of register 0F(bist finish) for 50 times*/
        for (pollCount = 0; pollCount < 50; pollCount++)
        {
            regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                       MV_SATA_I_HC_R0F_STATUS_BRIDGE_PORT_OFFSET(port));

            if (regVal & MV_BIT20)
            {
                break;
            }
        }

        if (regVal & MV_BIT20)
        {
            break;
        }/*if bit 20 still 0, then try the bist sequence again for 5 times*/
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: Warning: FarEnd LoopBack "
                     "- bit 20 still not set try again, tryCount %d\n",pAdapter->adapterId,
                     channelIndex, tryCount);
            temp = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                     MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                     MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port));
            temp &= 0xff0;
            temp |= MV_BIT0;
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                               MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port), temp);

            if (waitForBusyAfterHReset(pAdapter, channelIndex) == MV_FALSE)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: in Channel Hard Rese"
                         "t storage drive is not ready\n",pAdapter->adapterId,
                         channelIndex);

                result =  MV_FALSE;
            }
            _fixPhyParams(pAdapter, channelIndex);







        }
    }
    if (tryCount == 5)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: FarEnd LoopBack Failed"
                 "- the test doesn't finish\n",pAdapter->adapterId,
                 channelIndex);
        result = MV_FALSE;
    }
    else
    {
        if (((regVal & MV_BIT20) == 0) && (regVal & MV_BIT19))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: FarEnd LoopBack"
                     " finished with error, 0F regVal= %x\n",pAdapter->adapterId,
                     channelIndex, regVal);
            result = MV_FALSE;
        }
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: FarEnd LoopBack finished "
                     "successfuly, 0F regVal= %x\n",pAdapter->adapterId,
                     channelIndex, regVal);
        }
    }
    /* disable BIST and start phy communication */
    temp = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                             MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                             MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port));
    temp &= 0xff0;
    temp |= MV_BIT0;
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                       MV_SATA_I_HC_R02_STATUS_BRIDGE_PORT_OFFSET(port), temp);

    if (waitForBusyAfterHReset(pAdapter, channelIndex) == MV_FALSE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: in Channel Hard Rese"
                 "t storage drive is not ready\n",pAdapter->adapterId,
                 channelIndex);

        result =  MV_FALSE;
    }
    _fixPhyParams(pAdapter, channelIndex);

    return result;
}

/*******************************************************************************
* mvSataQueueCommand - Execute ATA command (PIO or UDMA)
*
* DESCRIPTION:
*   adds ATA PIO or UDMA request to a MV88SX50XX specific sata channel
*
* INPUT:
*   pAdapter    - pointer to the adapter data structure.
*   channelIndex    - the index of the specific EDMA channel
*   pCommandInfo - Pointer to the PIO or UDMA command
*
* RETURN:
*   MV_DMA_QUEUE_RESULT_OK - Command queuing is successfull
*   MV_QUEUE_COMMAND_RESULT_QUEUED_MODE_DISABLED - when trying to add command
                            while command queuing is disabled
*   MV_QUEUE_COMMAND_RESULT_FULL - Command queue is full
*   MV_QUEUE_COMMAND_RESULT_BAD_LBA_ADDRESS - when the connected device doesn't
*          support 48 bit addressing but the new command need's to use 48bit
*           addressing.
*   MV_QUEUE_RESULT_BAD_PARAMS - When bad parameters are received
*
* COMMENTS:
*   None.
*
*******************************************************************************/
MV_QUEUE_COMMAND_RESULT mvSataQueueCommand(MV_SATA_ADAPTER *pAdapter,
                                           MV_U8 channelIndex,
                                           MV_QUEUE_COMMAND_INFO *pCommandInfo)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];
    MV_QUEUED_COMMAND_ENTRY     *pCommandEntry;
    MV_U32          eDmaRegsOffset;
    MV_U32          nextEntry;

#ifdef MV_SATA_IO_GRANULARITY
    MV_U8          nextTransId;
#endif
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "  : mvSataQueueCommand"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_QUEUE_COMMAND_RESULT_BAD_PARAMS;
    }

    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataQueueCommand"
                 "nd Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return MV_QUEUE_COMMAND_RESULT_BAD_PARAMS;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    eDmaRegsOffset = pSataChannel->eDmaRegsOffset;
    if (pSataChannel->queueCommandsEnabled == MV_FALSE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: queued commands mode"
                 " is disabled\n", pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_QUEUE_COMMAND_RESULT_QUEUED_MODE_DISABLED;
    }
    /* queue up to 31 commands*/
    if (pSataChannel->outstandingCommands == MV_SATA_SW_QUEUE_SIZE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: queue is full\n",
                 pAdapter->adapterId, channelIndex );

        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_QUEUE_COMMAND_RESULT_FULL;
    }
#ifdef MV_SATA_IO_GRANULARITY
    if ((MV_QUEUED_COMMAND_TYPE_UDMA == pCommandInfo->type) &&
        (MV_TRUE ==
         pCommandInfo->commandParams.udmaCommand.ioGranularityEnabled))
    {

        if (MV_FALSE == pAdapter->iogEnabled)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d: IO granularity is "
                     "disabled for the adapter\n",
                     pAdapter->adapterId, channelIndex );
            mvOsSemRelease(&pSataChannel->semaphore);
            return MV_QUEUE_COMMAND_RESULT_BAD_PARAMS;
        }
        else
        {
            if (pAdapter->iogFreeIdsNum <= 0)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d: IO granularity "
                         "queue is full\n",
                         pAdapter->adapterId);
                mvOsSemRelease(&pSataChannel->semaphore);
                return MV_QUEUE_COMMAND_RESULT_FULL;
            }
        }
    }
#endif
    nextEntry = pSataChannel->freeIDsStack[--(pSataChannel->freeIDsNum)];
    pCommandEntry = &(pSataChannel->commandsQueue[nextEntry]);
    pCommandEntry->commandTag = (MV_U8)(nextEntry & 0x1f);
    pCommandEntry->isCommandInEdma = MV_FALSE;
    if (pCommandInfo->type == MV_QUEUED_COMMAND_TYPE_UDMA)
    {
        MV_UDMA_COMMAND_PARAMS  *pUdmaCommand = &pCommandInfo->commandParams.udmaCommand;
        if (pSataChannel->queuedDMA == MV_EDMA_MODE_NATIVE_QUEUING)
        {
            pUdmaCommand->isEXT = MV_TRUE;
        }
#ifdef MV_SATA_IO_GRANULARITY
        pUdmaCommand->iogCurrentTransId = MV_IOG_INVALID_COMMAND_ID;
        if ((MV_TRUE == pAdapter->iogEnabled) &&
            (MV_TRUE == pUdmaCommand->ioGranularityEnabled))
        {
            if (pUdmaCommand->iogCommandType == MV_IOG_COMMAND_TYPE_FIRST)
            {
                mvOsSemTake(&pAdapter->iogSemaphore);
                nextTransId =
                pAdapter->iogFreeIdsStack[--(pAdapter->iogFreeIdsNum)];
                setIoGranularityCount(pAdapter,
                                      nextTransId,
                                      pUdmaCommand->ioGranularityCommandParam.transCount);
                mvOsSemRelease(&pAdapter->iogSemaphore);
            }
            else
            {
                nextTransId =
                pUdmaCommand->ioGranularityCommandParam.transId;
            }
            pUdmaCommand->iogCurrentTransId = nextTransId;
        }
#endif
        if ((pSataChannel->noneUdmaOutstandingCommands == 0) &&
            (pSataChannel->NCQErrHandlingInfo.state == MV_NCQ_ERROR_HANDLING_STATE_IDLE))
        {
            if (pSataChannel->EdmaActive == MV_FALSE)
            {
                disableSaDevInterrupts(pAdapter,channelIndex);
                activateEdma(pAdapter,channelIndex);
            }
            addCommand(pSataChannel, pCommandEntry, pCommandInfo);
            EdmaReqQueueInsert(pSataChannel, pCommandEntry,
                               &pCommandInfo->commandParams.udmaCommand);
        }
        else
        {
            addCommand(pSataChannel, pCommandEntry, pCommandInfo);
        }
    }
    else
    {
        addCommand(pSataChannel, pCommandEntry, pCommandInfo);
        if (pSataChannel->outstandingCommands == 1)
        {
            if (pSataChannel->EdmaActive == MV_TRUE)
            {
                deactivateEdma(pAdapter,channelIndex);
            }
            if (pSataChannel->PMSupported == MV_TRUE)
            {
                _setActivePMPort(pSataChannel, pCommandInfo->PMPort);
            }
            if (sendNoneUdmaCommand(pSataChannel, pCommandEntry) == MV_FALSE)
            {
                removeCommand(pSataChannel,pCommandEntry);
                _doSoftReset(pSataChannel);
                mvOsSemRelease(&pSataChannel->semaphore);
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: Failed to "
                         "Issue PIO command\n", pAdapter->adapterId,
                         channelIndex);
                return MV_QUEUE_COMMAND_RESULT_QUEUED_MODE_DISABLED;
            }
        }
    }

    mvOsSemRelease(&pSataChannel->semaphore);
    return MV_QUEUE_COMMAND_RESULT_OK;







}
/*******************************************************************************
* mvSataSetInterruptsScheme - Modify interrupt scheme
*
* DESCRIPTION:
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*       interruptScheme =  A parameter containing the rquired interrupt scheme
*
* RETURN:
*       MV_TRUE on success, otherwise MV_FALSE.
* COMMENTS: This function doesn't modify the HW main mask register
*
*******************************************************************************/
MV_BOOLEAN mvSataSetInterruptsScheme(MV_SATA_ADAPTER *pAdapter,
                                     MV_SATA_INTERRUPT_SCHEME interruptScheme)
{
    mvOsSemTake(&pAdapter->semaphore);
    switch (interruptScheme)
    {
    case MV_SATA_INTERRUPT_HANDLING_IN_ISR:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS, " %d  : Interrupts"
                 " scheme set to Handling in ISR\n", pAdapter->adapterId);
        break;
    case MV_SATA_INTERRUPT_HANDLING_IN_TASK:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS, " %d  : Interrupts"
                 " scheme set to Handling in TASK\n", pAdapter->adapterId);
        break;
    case MV_SATA_INTERRUPTS_DISABLED:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS, " %d  : Interrupts"
                 " scheme set to interrupts Disabled\n", pAdapter->adapterId);
        break;
    default:
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d  : in mvSataSet"
                 "InterruptsScheme: Invalid Interrup scheme (%d)\n",
                 pAdapter->adapterId, interruptScheme);
        mvOsSemRelease(&pAdapter->semaphore);
        return MV_FALSE;
    }
    pAdapter->interruptsScheme = interruptScheme;
    mvOsSemRelease(&pAdapter->semaphore);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataCheckPendingInterrupt - Check and mask interrupts
*
* DESCRIPTION:
*       Check if an interrupt is pending, If there is  a pending interrupt then
*   this function masks the adapter's interrupts and returns MV_TRUE
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*
* RETURN:
*       MV_TRUE if the interrupt issued by mv adapter, otherwise MV_FALSE.
* COMMENTS:
*       this function must be used only when interrupt scheme is set to
*       MV_SATA_INTERRUPT_IN_TASK
*
*******************************************************************************/
MV_BOOLEAN mvSataCheckPendingInterrupt(MV_SATA_ADAPTER *pAdapter)
{
    MV_U32  mainMask;
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;

    /*mvOsSemTake(&pAdapter->interruptsMaskSem);*/
    mainMask = pAdapter->mainMask;
    /*mvOsSemRelease(&pAdapter->interruptsMaskSem);*/
    /* if the interrupt it ours*/
    if (MV_REG_READ_DWORD(ioBaseAddr,MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET) &
        mainMask)
    {
        /*clear mainMask, the ISR enables the interrupt once served*/
        MV_REG_WRITE_DWORD(ioBaseAddr, MV_MAIN_INTERRUPT_MASK_REG_OFFSET, 0);
        return MV_TRUE;
    }
    /*bogus interrupt*/
    return MV_FALSE;
}
/*******************************************************************************
* mvSataInterruptServiceRoutine - Interrupt service routine
*
* DESCRIPTION:
*       this function is an interrupt service routine that is called upon
*       reception of an interrupt from a MV88SX50XX adapter.
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*
* RETURN:
*       MV_TRUE if the interrupt issued by mv adapter, otherwise MV_FALSE.
* COMMENTS:
*       this function handles all the events that generate interrupts incuding
*       calling the upper layer call back functions.
*
*******************************************************************************/
MV_BOOLEAN mvSataInterruptServiceRoutine(MV_SATA_ADAPTER *pAdapter)
{
    MV_U32  mainCause;
    MV_U32  mainMask;
    MV_U32  unitCause = 0;
    MV_U32  responseDone;
    MV_U32  edmaError;
    MV_U32  deviceInterrupt;
    MV_U8  sataUnit;
    MV_U8  port;
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;
    mvOsSemTake(&pAdapter->semaphore);
    mainCause = MV_REG_READ_DWORD(ioBaseAddr,
                                  MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET);

    /* Check if the interrupt is ours */
    mvOsSemTake(&pAdapter->interruptsMaskSem);
    mainMask = pAdapter->mainMask;
    mvOsSemRelease(&pAdapter->interruptsMaskSem);

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG | MV_DEBUG_INTERRUPTS,
             " %d  : Interrupt. Cause = 0x%08x, mask 0x%08x\n",
             pAdapter->adapterId, mainCause, mainMask);

    /*
     * Check if interrupt is our or interrupts are masked.
     * in interrupts disaled scheme, the main mask register is cleared but the
     * mainMask variable will hold the bits where interrupts expected, this why
     * the sheme is not checked
     */
    if ((0 == (mainCause & mainMask)) ||
        ((pAdapter->interruptsAreMasked == MV_TRUE) &&
         (pAdapter->interruptsScheme != MV_SATA_INTERRUPTS_DISABLED)))
    {
        /* when interrupts handled in task, we expect to find interrupts here*/
        if (pAdapter->interruptsScheme == MV_SATA_INTERRUPT_HANDLING_IN_TASK)
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," %d  : ISR called "
                     "but no interrutps are found in interrupts handle in task"
                     "scheme!\n", pAdapter->adapterId);
            /*anyway unmask interrupts*/
            mvOsSemTake(&pAdapter->interruptsMaskSem);
            if (pAdapter->interruptsAreMasked == MV_FALSE)
            {
                MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                                   MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                                   pAdapter->mainMask);
            }
            mvOsSemRelease(&pAdapter->interruptsMaskSem);
        }
        mvOsSemRelease(&pAdapter->semaphore);
        return MV_FALSE;
    }

    if (mainCause & MV_MAIN_INTERRUPT_MASK_REG_PCIERR_BIT)
    {
        MV_U32  pciCause = MV_REG_READ_DWORD(ioBaseAddr,
                                             MV_PCI_REGS_OFFSET +
                                             MV_PCI_INTERRUPT_CAUSE_REG_OFFSET);

        _dumpPCIRegs(pAdapter);
        {
            MV_U8   i;

            for (i = 0; i < pAdapter->numberOfChannels;i++)
            {
                _dumpEDMARegs(pAdapter, i);
                _dumpChannelQueues(pAdapter, i);
            }
        }
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR | MV_DEBUG_INTERRUPTS,
                 " %d  : PCI error, pci interrupt cause register=%08x\n",
                 pAdapter->adapterId, pciCause);
        /* clear cause register */
        MV_REG_WRITE_DWORD(ioBaseAddr, MV_PCI_REGS_OFFSET +
                           MV_PCI_INTERRUPT_CAUSE_REG_OFFSET,
                           ~pciCause);
        pAdapter->mvSataEventNotify(pAdapter, MV_EVENT_TYPE_ADAPTER_ERROR,
                                    pciCause, 0);
    }
#ifdef MV_SATA_IO_GRANULARITY
    /*IO Granularity interrupt*/
    if (mainCause & MV_IOG_TRANS_INT_MASK)
    {
        mvOsSemTake(&pAdapter->iogSemaphore);
        iogInterrupt(pAdapter, ioBaseAddr, mainCause);
        mvOsSemRelease(&pAdapter->iogSemaphore);
    }
#endif
    for (sataUnit = 0; sataUnit < pAdapter->numberOfUnits; sataUnit++)
    {
        if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
        {
            /* Clear the all ports interrupt coalescing cause register */
            MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                                MV_SATA_II_ALL_PORTS_INT_CAUSE_REG_OFFSET,
                                0);
        }

        if (mainCause & 0x1ff)
        {
            MV_U32  unitCauseAddr = MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                    MV_SATAHC_INTERRUPT_CAUSE_REG_OFFSET;
            MV_U32  unitRspInPtr;

            /* clear the cause bit of the Coalescing interrupt*/
            MV_REG_WRITE_DWORD(ioBaseAddr, unitCauseAddr, ~MV_BIT4);
            unitCause = MV_REG_READ_DWORD(ioBaseAddr, unitCauseAddr);
            /* clear the cause register of the current unit */
            MV_REG_WRITE_DWORD(ioBaseAddr, unitCauseAddr, ~unitCause | MV_BIT4);

            unitRspInPtr = MV_REG_READ_DWORD(ioBaseAddr,
                                             MV_SATAHC_REGS_BASE_OFFSET(sataUnit) +
                                             MV_SATAHC_RESPONSE_Q_IN_POINTER_OFFSET);

            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d unit %d : unitCause = 0x%08x,"
                     " unitRspInPtr 0x%08x\n", pAdapter->adapterId, sataUnit,
                     unitCause, unitRspInPtr);

            for (port = 0; port < MV_SATA_PORT_PER_UNIT; port++)
            {
                deviceInterrupt = unitCause & (MV_BIT8 << port);
                responseDone = unitCause & (1 << port);
                edmaError = (mainCause & MV_BIT0) ;
                if (responseDone || edmaError)
                {
                    handleEdmaInterrupt(pAdapter, sataUnit, port,
                                        unitRspInPtr & 0x1f, responseDone,
                                        edmaError, unitCause);
                }
                if (deviceInterrupt &&
                    (SaDevInterrutpBit((MV_U8)MV_CHANNEL_INDEX(sataUnit, port)) & mainMask))
                {
                    handleDeviceInterrupt(pAdapter, sataUnit, port);
                }


                mainCause >>= 2;
                unitRspInPtr >>= 8;
            }
        }
        else
        {
            mainCause >>=8;
        }
        mainCause >>= 1;            /* this is for the coalescing 0-3 bit*/
    }
    if (pAdapter->interruptsScheme == MV_SATA_INTERRUPT_HANDLING_IN_TASK)
    {
        if (pAdapter->interruptsAreMasked == MV_FALSE)
        {
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                               pAdapter->mainMask);
        }
    }
    mvOsSemRelease(&pAdapter->semaphore);
    return MV_TRUE;
}
/*******************************************************************************
* mvSataMaskAdapterInterrupt - mask any interrupts can be generated from a
*       MV88SX50XX adapter
*
* DESCRIPTION:
*       mask all the interrupts that could occur from the adapter.
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*
* RETURN:
*       MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*       Before masking the interrupts, the value of the interrupt maks register
*       will be stored in the adapter data structure.
*
*******************************************************************************/
MV_BOOLEAN mvSataMaskAdapterInterrupt(MV_SATA_ADAPTER *pAdapter)
{
    pAdapter->interruptsAreMasked = MV_TRUE;
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_MAIN_INTERRUPT_MASK_REG_OFFSET, 0);
    return MV_TRUE;
}

/*******************************************************************************
* mvSataUnmaskAdapterInterrupt - unmask interrupts can be generated from a
*       MV88SX50XX adapter
*
* DESCRIPTION:
*       Restore a previous value in the MV88SX50XX interrupt maks register by
*       writing the previously stored value in the interruptMaskRegister field
*       in the adapter data structure to the MV88SX50XX adapter main interrupt
*       mask register
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*
* RETURN:
*       MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*
*******************************************************************************/
MV_BOOLEAN mvSataUnmaskAdapterInterrupt(MV_SATA_ADAPTER *pAdapter)
{
    mvOsSemTake(&pAdapter->interruptsMaskSem);
    pAdapter->interruptsAreMasked = MV_FALSE;
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                       pAdapter->mainMask);
    mvOsSemRelease(&pAdapter->interruptsMaskSem);
    return MV_TRUE;
}


/*******************************************************************************
* mvSataEnableStaggeredSpinUpAll - Enables staggared spin-up of all SATA channels
*
* DESCRIPTION:
*       Enables staggared spin-up of all SATA II chnannel. This function is not
*       relevant for SATA I.
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*
* RETURN:
*       MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*
*******************************************************************************/
MV_BOOLEAN mvSataEnableStaggeredSpinUpAll (MV_SATA_ADAPTER *pAdapter)
{
    MV_U8 channelIndex;
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataEnableStaggeredSpinUp Failed, Bad adapter data structure"
                 " pointer\n");
        return MV_FALSE;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d  : Staggered spin-up called for all "
             "SATA channels\n",pAdapter->adapterId);
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {

        mvOsSemTake(&pAdapter->semaphore);
        _establishSataCommAll(pAdapter);
        for (channelIndex = 0 ; channelIndex < pAdapter->numberOfChannels ;
            channelIndex ++)
        {
            MV_U32 SStatusReg;
            SStatusReg = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                            getEdmaRegOffset(channelIndex) +
                                            MV_SATA_II_S_STATUS_REG_OFFSET);

            /*
             * SHII10
             */

            if ((SStatusReg != 0x0) && (SStatusReg != 0x113) && (SStatusReg != 0x123))
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: SStatusRegs = %x ; "
                         "retrying communication...",
                         pAdapter->adapterId, channelIndex, SStatusReg);
                _establishSataComm(pAdapter,channelIndex);
                SStatusReg = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                                getEdmaRegOffset(channelIndex) +
                                                MV_SATA_II_S_STATUS_REG_OFFSET);
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: New SStatus is %x\n",
                         pAdapter->adapterId, channelIndex, SStatusReg);

            }
            if ( ((SStatusReg & 0xf) == MV_PHY_DET_STATE_DEVICE_NO_PHY_COM) ||
                 ((SStatusReg & 0xf) == MV_PHY_DET_STATE_PHY_OFFLINE))
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: "
                         "Error - failed to establish SATA link after staggered "
                         "spin up (S Status = %08x)\n", pAdapter->adapterId,
                         channelIndex, SStatusReg);
            }
            pAdapter->staggaredSpinup[channelIndex] = MV_TRUE;
        }
        mvOsSemRelease(&pAdapter->semaphore);
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_SATA_LINK | MV_DEBUG, " %d  : "
             "Finished staggered spin-up\n", pAdapter->adapterId);

    return MV_TRUE;
}

/*******************************************************************************
* mvSataEnableStaggeredSpinUp - Enables staggared spin-up.
*
* DESCRIPTION:
*       Enables staggared spin-up of SATA II chnannel. This function is not
*       relevant for SATA I.
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*   channelIndex    - the index of the specific EDMA channel
*
* RETURN:
*       MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*
*******************************************************************************/
MV_BOOLEAN mvSataEnableStaggeredSpinUp (MV_SATA_ADAPTER *pAdapter,
                                        MV_U8 channelIndex)
{
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataEnableStaggeredSpinUp Failed, Bad adapter data structure"
                 " pointer\n");
        return MV_FALSE;
    }
    if (channelIndex >= pAdapter->numberOfChannels)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataEnableStaggeredSpinUp Failed, bad channel index\n");
        return MV_FALSE;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: Staggered spin-up called\n",
             pAdapter->adapterId, channelIndex);
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        MV_U32 SStatusReg;

        mvOsSemTake(&pAdapter->semaphore);
        _establishSataComm(pAdapter, channelIndex);
        SStatusReg = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,
                                        getEdmaRegOffset(channelIndex) +
                                        MV_SATA_II_S_STATUS_REG_OFFSET);
        if ( ((SStatusReg & 0xf) == MV_PHY_DET_STATE_DEVICE_NO_PHY_COM) ||
             ((SStatusReg & 0xf) == MV_PHY_DET_STATE_PHY_OFFLINE))
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: "
                     "Error - failed to establish SATA link after staggered "
                     "spin up (S Status = %08x)\n", pAdapter->adapterId,
                     channelIndex, SStatusReg);
        }
        pAdapter->staggaredSpinup[channelIndex] = MV_TRUE;
        mvOsSemRelease(&pAdapter->semaphore);
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_SATA_LINK | MV_DEBUG, " %d %d: "
             "Finished staggered spin-up\n", pAdapter->adapterId,
             channelIndex);

    return MV_TRUE;
}

/*******************************************************************************
* mvSataDisableStaggeredSpinUpAll - Disables staggared spin-up on all channels
*
* DESCRIPTION:
*       Disables staggared spin-up of all SATA II chnannel. This function is not
*       relevant for SATA I.
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*
* RETURN:
*       MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*
*******************************************************************************/
MV_BOOLEAN mvSataDisableStaggeredSpinUpAll (MV_SATA_ADAPTER *pAdapter)
{
    MV_U8 channelIndex;
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataDisableStaggeredSpinUp Failed, Bad adapter data structure"
                 " pointer\n");
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        /* OK to use mvSataDisableStaggeredSpinUp since it's fast enough */
        for (channelIndex = 0 ; channelIndex < pAdapter->numberOfChannels ;
            channelIndex ++)
        {
            mvSataDisableStaggeredSpinUp(pAdapter,channelIndex);







        }
    }
    return MV_TRUE;
}

/*******************************************************************************
* mvSataDisableStaggeredSpinUp - Disables staggared spin-up.
*
* DESCRIPTION:
*       Disables staggared spin-up of SATA II chnannel. This function is not
*       relevant for SATA I.
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*   channelIndex    - the index of the specific EDMA channel
*
* RETURN:
*       MV_TRUE on success, MV_FALSE otherwise.
* COMMENTS:
*
*******************************************************************************/
MV_BOOLEAN mvSataDisableStaggeredSpinUp (MV_SATA_ADAPTER *pAdapter,
                                         MV_U8 channelIndex)
{
    MV_U32 SControlOffset, regVal;
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataDisableStaggeredSpinUp Failed, Bad adapter data structure"
                 " pointer\n");
        return MV_FALSE;
    }
    if (channelIndex >= pAdapter->numberOfChannels)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataDisableStaggeredSpinUp Failed, bad channel index\n");
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        mvOsSemTake(&pAdapter->semaphore);
        SControlOffset = getEdmaRegOffset( channelIndex) +
                         MV_SATA_II_S_CONTROL_REG_OFFSET;
        regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset);
        regVal &= ~0xf;
        regVal |= MV_PHY_DET_CONTROL_SHUTDOWN;
        MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress, SControlOffset,
                            regVal);
        pAdapter->staggaredSpinup[channelIndex] = MV_FALSE;
        mvOsSemRelease(&pAdapter->semaphore);
    }
    return MV_TRUE;
}

/*******************************************************************************
* mvSataSetInterfaceSpeed - Sets the interface speed of a specific SATA channel
*
* DESCRIPTION:
*       Sets the interface speed of a specific SATA channel (1.5/3 Gbps)
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*   channelIndex    - the index of the specific EDMA channel
*
* RETURN:
*       MV_SATA_IF_SPEED_1_5 for 1.5 Gbps
*       MV_SATA_IF_SPEED_3   for 3 Gbps
*       MV_SATA_IF_SPEED_INVALID if no speed is negotiated
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN mvSataSetInterfaceSpeed (MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 channelIndex,
                                    MV_SATA_IF_SPEED ifSpeed)
{
    MV_U32  SStatusOffset;
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataSetInterfaceSpeed Failed, Bad adapter data structure"
                 " pointer\n");
        return MV_FALSE;
    }
    if (channelIndex >= pAdapter->numberOfChannels)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataSetInterfaceSpeed Failed, bad channel index\n");
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        MV_SATA_CHANNEL *pSataChannel;

        pSataChannel = pAdapter->sataChannel[channelIndex];
        if (pSataChannel != NULL)
        {
            mvOsSemTake(&pSataChannel->semaphore);
            if (pSataChannel->queueCommandsEnabled == MV_TRUE)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: Error:"
                         "mvSataSetInterfaceSpeed called while EDMA is active\n",
                         pAdapter->adapterId,channelIndex);
                mvOsSemRelease( &pSataChannel->semaphore);
                return MV_FALSE;
            }
            mvOsSemRelease(&pSataChannel->semaphore);
        }

        SStatusOffset = getEdmaRegOffset(channelIndex) +
                        MV_SATA_II_S_STATUS_REG_OFFSET;

        if (ifSpeed == MV_SATA_IF_SPEED_1_5_GBPS)
        {
            pAdapter->limitInterfaceSpeed[channelIndex] = MV_TRUE;
            pAdapter->ifSpeed[channelIndex] = MV_SATA_IF_SPEED_1_5_GBPS;
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_SATA_LINK, " %d %d: Set Sata speed to "
                     "1.5Gbps\n", pAdapter->adapterId,channelIndex);
        }
        else if (ifSpeed == MV_SATA_IF_SPEED_3_GBPS)
        {
            pAdapter->limitInterfaceSpeed[channelIndex] = MV_TRUE;
            pAdapter->ifSpeed[channelIndex] = MV_SATA_IF_SPEED_3_GBPS;
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_SATA_LINK, " %d %d: Set Sata speed to "
                     "3Gbps\n", pAdapter->adapterId,channelIndex);
        }
        else if (ifSpeed == MV_SATA_IF_SPEED_NO_LIMIT)
        {
            pAdapter->limitInterfaceSpeed[channelIndex] = MV_FALSE;
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_SATA_LINK, " %d %d: don't limit Sata"
                     " speed \n", pAdapter->adapterId,channelIndex);
        }
        else
        {
            mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                     "mvSataSetInterfaceSpeed Failed, bad IF speed\n");
            return MV_FALSE;
        }

        mvOsSemTake(&pAdapter->semaphore);
        _channelHardReset(pAdapter, channelIndex);
        /* If interface is already active, then device detection is needed */
        if (pAdapter->staggaredSpinup[channelIndex] == MV_TRUE)
        {
            _establishSataComm(pAdapter, channelIndex);
        }
        mvOsSemRelease(&pAdapter->semaphore);
        return MV_TRUE;
    }
    if ((pAdapter->sataAdapterGeneration == MV_SATA_GEN_I) &&
        ((ifSpeed == MV_SATA_IF_SPEED_NO_LIMIT) ||
         (ifSpeed == MV_SATA_IF_SPEED_1_5_GBPS)))
    {
        return MV_TRUE;
    }
    return MV_FALSE;
}

/*******************************************************************************
* mvSataGetInterfaceSpeed - Gets the interface speed of a specific SATA channel
*
* DESCRIPTION:
*       Gets the interface speed of a specific SATA channel (1.5/3 Gbps)
*
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*   channelIndex    - the index of the specific EDMA channel
*
* RETURN:
*       MV_SATA_IF_SPEED_1_5 for 1.5 Gbps
*       MV_SATA_IF_SPEED_3   for 3 Gbps
*       MV_SATA_IF_SPEED_INVALID if no speed is negotiated
*
* COMMENTS:
*
*******************************************************************************/
MV_SATA_IF_SPEED mvSataGetInterfaceSpeed (MV_SATA_ADAPTER *pAdapter,
                                          MV_U8 channelIndex)
{
    MV_U32 SStatusOffset, regVal;
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataGetInterfaceSpeed Failed, Bad adapter data structure"
                 " pointer\n");
        return MV_SATA_IF_SPEED_INVALID;
    }
    if (channelIndex >= pAdapter->numberOfChannels)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : "
                 "mvSataGetInterfaceSpeed Failed, bad channel index\n");
        return MV_SATA_IF_SPEED_INVALID;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        if (pAdapter->staggaredSpinup[channelIndex] == MV_TRUE)
        {
            SStatusOffset = getEdmaRegOffset(channelIndex) +
                            MV_SATA_II_S_STATUS_REG_OFFSET;
            regVal = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, SStatusOffset);
            if (regVal & MV_BIT4)
            {
                return MV_SATA_IF_SPEED_1_5_GBPS;
            }
            if (regVal & MV_BIT5)
            {
                return MV_SATA_IF_SPEED_3_GBPS;
            }
        }
        return MV_SATA_IF_SPEED_INVALID;
    }
    return MV_SATA_IF_SPEED_1_5_GBPS;/*TBD*/
}

/*================C2C functions==========================================*/
#ifdef MV_SATA_C2C_COMM
static void activateBMDmaMode(MV_SATA_ADAPTER *pAdapter,
                              MV_U8 channelIndex,
                              MV_U32 prdTableHi,
                              MV_U32 prdTableLow,
                              MV_UDMA_TYPE dmaType)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;


    MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                        MV_BMDMA_COMMAND_OFFSET, 0);

    MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                        MV_BMDMA_PRD_TABLE_LOW_ADDRESS_OFFSET, prdTableLow);
    MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                        MV_BMDMA_PRD_TABLE_HIGH_ADDRESS_OFFSET, prdTableHi);

    if (dmaType == MV_UDMA_TYPE_READ)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: Activate BM-DMA in write Mode"
                 " (Read from disk)\n", pAdapter->adapterId, channelIndex);
        MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                            MV_BMDMA_COMMAND_OFFSET, MV_BIT3 | MV_BIT0);
    }
    else
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: Activate BM-DMA in read Mode"
                 " (Writes to disk)\n", pAdapter->adapterId, channelIndex);
        MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                            MV_BMDMA_COMMAND_OFFSET, MV_BIT0);
    }
}


/*******************************************************************************
* sendVendorUniqueFIS - send vendor unique FIS
*
* DESCRIPTION:
*       Performs vendor unique FIS transmission
*
* INPUT:
*       pAdapter            - pointer to the adapter data structure.
*       channelIndex        - the index of the specific SATA channel
*       vendorUniqueBuffer  - data buffer to transmit
*       numOfDWords         - number of double words in the buffer
*
* RETURN:
*       MV_TRUE on success
*       MV_FALSE on error
*
* COMMENTS:
*
*  1. Verify the Transport Layer is in idle, field TransFsmSts in
*     Serial-ATA Interface Status Register is cleared.
*  2. Set Vendor Unique Mode. Write 1 to bit VendorUqMd in register
*     Serial-ATA Interface Control Register.
*  3. Insert data into Vendor Unique Register.
*  4. Repeat steps 3 until all data except last Dword in the vendor unique
*     FIS is transferred. Note that according to Serial-ATA protocol the
*     FIS length is limited to 8 KB.
*  5. Write 1 to bit VendorUqSend in register Serial-ATA Interface Control
*     Register.
*  6. Write last Dword in the FIS to Complete FIS transmission.
*  7. Wait for transmission completion. Bit VendorUqDn or bit VendorUqErr
*     in Serial-ATA Interface Status Register is set to 1.
*  8. Verify successful transmission of the FIS. Bit VendorUqErr in
*     Serial-ATA Interface Status Register is cleared.
*  9. Clear Vendor Unique Mode. Write 0 to bit VendorUqMd in register
*     Serial-ATA Interface Control Register.
*******************************************************************************/
static MV_BOOLEAN sendVendorUniqueFIS(MV_SATA_ADAPTER *pAdapter,
                                      MV_U8 channelIndex,
                                      MV_U32 *vendorUniqueBuffer,
                                      MV_U8 numOfDWords)
{
    MV_SATA_CHANNEL *pSataChannel = pAdapter->sataChannel[channelIndex];
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;
    MV_U32          regVal;
    MV_U8           i;
    MV_BOOLEAN      res = MV_FALSE;

    regVal = MV_REG_READ_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                               MV_SATA_II_IF_STATUS_REG_OFFSET);

    if (regVal & MV_SATA_II_IF_STATUS_FSM_STATUS_MASK)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: ERROR : sendVendorUniqueFIS"
                 " Transport Layer is not in Idle status\n", pAdapter->adapterId,
                 channelIndex);
        return MV_FALSE;
    }

    /* Set Vendor Unique Mode */
    MV_REG_WRITE_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                       MV_SATA_II_IF_CONTROL_REG_OFFSET, MV_BIT8);

    for (i = 1; i < numOfDWords; i++)
    {
        MV_REG_WRITE_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                           MV_SATA_II_VENDOR_UQ_REG_OFFSET,
                           vendorUniqueBuffer[i - 1]);
    }

    /* Write 1 to bit VendorUqSend */
    MV_REG_WRITE_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                       MV_SATA_II_IF_CONTROL_REG_OFFSET,  MV_BIT9|MV_BIT8);

    MV_REG_WRITE_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                       MV_SATA_II_VENDOR_UQ_REG_OFFSET,
                       vendorUniqueBuffer[i - 1]);


    /* polling with timeout*/
    for (i = 0;  i < 200; i++)
    {
        regVal = MV_REG_READ_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                                   MV_SATA_II_IF_STATUS_REG_OFFSET);

        if (regVal & (MV_SATA_II_IF_STATUS_VUQ_DONE_MASK |
                      MV_SATA_II_IF_STATUS_VUQ_ERR_MASK))
        {
            if (regVal & MV_SATA_II_IF_STATUS_VUQ_DONE_MASK)
            {
                res = MV_TRUE;
                break;
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: ERROR"
                         "sendVendorUniqueFIS operation failed. "
                         "regVal = 0x%08x\n",
                         pAdapter->adapterId, channelIndex, regVal);
                break;
            }
        }
        mvMicroSecondsDelay(pAdapter, 1);
    }
    /* Clear Vendor Unique Mode */
    MV_REG_WRITE_DWORD(ioBaseAddr, pSataChannel->eDmaRegsOffset +
                       MV_SATA_II_IF_CONTROL_REG_OFFSET, 0);

    if (res != MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: ERROR in sendVendorUniqueFI"
                 "S operation didn't finish. regVal = 0x%08x\n",
                 pAdapter->adapterId, channelIndex, regVal);
    }
    return res;
}



/*******************************************************************************
* mvSataC2CInit - setup channel-to-channel communication mode on
*                   specific SATA channel
*
* DESCRIPTION:
*       Initializes channel for channel-to-channel communication mode
*
* INPUT:
*       pAdapter        - pointer to the adapter data structure.
*       channelIndex    - the index of the specific SATA channel
*       mvSataC2CMode   - Comunication mode for the channel:
*                           target or initiator
*
*       mvSataC2CCallBack - callback function called on channel 2 channel
*                           communication event
*
* RETURN:
*       MV_TRUE on success
*       MV_FALSE on error
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN mvSataC2CInit (MV_SATA_ADAPTER *pAdapter,
                          MV_U8 channelIndex,
                          MV_SATA_C2C_MODE mvSataC2CMode,
                          C2CCallBack_t mvSataC2CCallBack)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32          regVal;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataC2CInit"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataC2CInit Failed"
                 ", channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex );
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);

    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataC2CInit "
                 "failed, DMA is enabled\n", pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataC2CInit "
                 "failed, Feature not supported by this HW\n",
                 pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }

    regVal = MV_REG_READ_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                                MV_SATA_II_SATA_CONFIG_REG_OFFSET);
    /* Enable communication mode */
    regVal |= MV_BIT11;
    /* SHII8*/
    regVal |= MV_BIT12;

    if (mvSataC2CMode == MV_SATA_C2C_MODE_INITIATOR)
    {
        regVal |= MV_BIT10; /* Initiator */
    }
    else
    {
        regVal &= ~MV_BIT10; /* Target */
    }

    maskEdmaInterrupts(pAdapter, channelIndex);

    MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                        MV_SATA_II_SATA_CONFIG_REG_OFFSET, regVal);
    MV_REG_READ_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                       MV_SATA_II_SATA_CONFIG_REG_OFFSET);

    pSataChannel->C2CmodeEnabled = MV_TRUE;
    pSataChannel->C2CMode = mvSataC2CMode;
    pSataChannel->C2CCallback = mvSataC2CCallBack;

    /*The hard reset is needed to reactivate channel Rx impedance
    auto calibration on the target side.*/
    if (mvSataC2CMode == MV_SATA_C2C_MODE_TARGET)
    {
        _channelHardReset(pAdapter, channelIndex);
    }
    mvSataMaskAdapterInterrupt(pAdapter);
    pAdapter->mainMask |= SaDevInterrutpBit(channelIndex);
    mvSataUnmaskAdapterInterrupt(pAdapter);

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: mvSataC2CInit %s mode\n",
             pAdapter->adapterId, channelIndex,
             (mvSataC2CMode == MV_SATA_C2C_MODE_INITIATOR) ? "Initiator" :
             "Target");

    mvOsSemRelease(&pSataChannel->semaphore);
    return MV_TRUE;
}


/*******************************************************************************
* mvSataC2CStop - Stop channel to channel communication mode
*
* DESCRIPTION:
*       Stop channel to channel communication mode on
*                   specific SATA channel
*
* INPUT:
*       pAdapter        - pointer to the adapter data structure.
*       channelIndex    - the index of the specific SATA channel
*
* RETURN:
*       MV_TRUE on success
*       MV_FALSE on error
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN mvSataC2CStop (MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL  *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32          regVal;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, "    : mvSataC2CStop"
                 " Failed, Bad adapter data structure pointer\n");
        return MV_FALSE;
    }
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d: mvSataC2CStop Failed"
                 ", channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex );
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);

    if (pSataChannel->queueCommandsEnabled == MV_TRUE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataC2CStop "
                 "failed, DMA is enabled\n", pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }

    regVal = MV_REG_READ_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                                MV_SATA_II_SATA_CONFIG_REG_OFFSET);
    regVal &= ~MV_BIT11;    /* Disable communication mode */
    /* SHII8*/
    regVal |= MV_BIT12;

    MV_REG_WRITE_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                        MV_SATA_II_SATA_CONFIG_REG_OFFSET, regVal);
    MV_REG_READ_DWORD (ioBaseAddr, pSataChannel->eDmaRegsOffset +
                       MV_SATA_II_SATA_CONFIG_REG_OFFSET);
    mvSataMaskAdapterInterrupt(pAdapter);
    pAdapter->mainMask &= ~SaDevInterrutpBit(channelIndex);
    mvSataUnmaskAdapterInterrupt(pAdapter);

    _channelHardReset(pAdapter, channelIndex);
    pSataChannel->C2CmodeEnabled = MV_FALSE;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG, " %d %d: mvSataC2CStop\n",
             pAdapter->adapterId, channelIndex);

    mvOsSemRelease(&pSataChannel->semaphore);
    return MV_TRUE;
}



/*******************************************************************************
* mvSataC2CSendRegisterDeviceToHostFIS - sends Register device to host FIS
*
* DESCRIPTION:
*                   Sends Register device to host FIS
*                   used for channel-to-channel communication mode on
*                   specific SATA channel
* INPUT:
*       pAdapter        - pointer to the adapter data structure.
*       channelIndex    - the index of the specific SATA channel
*       pmPort          - port multiplier port
*       bInterrupt      - determine whether the interrupt is being generated on
*                           the receiver side
*       msg             - message containing 10 bytes of user data
*
* RETURN:
*       MV_TRUE on success
*       MV_FALSE on error
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN  mvSataC2CSendRegisterDeviceToHostFIS(
                                                MV_SATA_ADAPTER *pAdapter,
                                                MV_U8 channelIndex,
                                                MV_U8 pmPort,
                                                MV_BOOLEAN bInterrupt,
                                                MV_U8 msg[MV_C2C_MESSAGE_SIZE])

{

    MV_SATA_CHANNEL *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32          buffer[5];
    MV_BOOLEAN      res;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d "
                 "mvSataC2CSendRegisterDeviceToHostFIS failed - "
                 "Bad adapter data structure pointer.\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }

    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d "
                 "mvSataC2CSendRegisterDeviceToHostFIS failed - "
                 "channel data structure not allocated.\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);

    if (pSataChannel->C2CmodeEnabled == MV_FALSE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: mvSataC2CInitiator"
                 "mvSataC2CSendRegisterDeviceToHostFIS failed - "
                 "Bad C2C configuration.\n",
                 pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }

    buffer[0] = MV_SATA_REGISTER_HOST_2_DEVICE_FIS;
    buffer[0] |= ((MV_U32)(pmPort & 0xF)) << 8;
    if (MV_TRUE == bInterrupt)
    {
        buffer[0] |= 1 << 14;
    }
    buffer[0] |= ((MV_U32)msg[0]) << 24;
    buffer[1] = msg[1] |
                ((MV_U32)msg[2]) << 8 |
                ((MV_U32)msg[3]) << 16 |
                ((MV_U32)msg[4]) << 24;
    buffer[2] = msg[5] |
                ((MV_U32)msg[6]) << 8 |
                ((MV_U32)msg[7]) << 16;
    buffer[3] = msg[8] |
                ((MV_U32)msg[9]) << 8;
    buffer[4] = 0;
    res = sendVendorUniqueFIS(pAdapter,
                              channelIndex,
                              (MV_U32*)buffer,
                              5);
    mvOsSemRelease(&pSataChannel->semaphore);
    return res;
}



/*******************************************************************************
* mvSataC2CActivateBmDma - activate B-M DMA
*
* DESCRIPTION:
*       Activates Bus Master DMA for the specific SATA channel
*
* INPUT:
*       pAdapter        - pointer to the adapter data structure.
*       channelIndex    - the index of the specific EDMA channel
*       pmPort          - port multiplier port
*       prdTableHi      - upper 32 bits of PRD table address
*       prdTableHi      - lower 32 bits of PRD table address
*       dmaType         - DMA type (read o write) from the initiator point
                        of view
*
* RETURN:
*       MV_TRUE on success
*       MV_FALSE on error
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN  mvSataC2CActivateBmDma(MV_SATA_ADAPTER *pAdapter,
                                   MV_U8 channelIndex,
                                   MV_U8 pmPort,
                                   MV_U32 prdTableHi,
                                   MV_U32 prdTableLow,
                                   MV_UDMA_TYPE dmaType)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_BOOLEAN      res = MV_TRUE;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d "
                 "mvSataC2CActivateBmDma failed - "
                 "Bad adapter data structure pointer.\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    ioBaseAddr = pAdapter->adapterIoBaseAddress;
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d "
                 "mvSataC2CActivateBmDma failed - "
                 "channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex );
        return MV_FALSE;
    }
    mvOsSemTake(&pSataChannel->semaphore);
    if (pSataChannel->C2CmodeEnabled == MV_FALSE)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d "
                 "mvSataC2CActivateBmDma failed - "
                 "bad C2C configuration.\n",
                 pAdapter->adapterId, channelIndex);
        mvOsSemRelease(&pSataChannel->semaphore);
        return MV_FALSE;
    }
    if (pSataChannel->C2CMode == MV_SATA_C2C_MODE_TARGET)
    {
        activateBMDmaMode(pAdapter, channelIndex,
                          prdTableHi, prdTableLow,
                          (dmaType == MV_UDMA_TYPE_WRITE) ?
                          MV_UDMA_TYPE_READ : MV_UDMA_TYPE_WRITE);
        if (dmaType == MV_UDMA_TYPE_WRITE)
        {
            MV_U32 buffer[1] = {MV_SATA_DMA_ACTIVATE_FIS};

            buffer[0] |= ((MV_U32)(pmPort & 0xF)) << 8;
            res = sendVendorUniqueFIS(pAdapter,
                                      channelIndex,
                                      (MV_U32*)buffer,
                                      1);
        }
        else
        {
            MV_U32 val = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                           pAdapter->sataChannel[channelIndex]->eDmaRegsOffset +
                                           MV_SATA_II_IF_CONTROL_REG_OFFSET);
            val |= MV_BIT16;
            MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                               pAdapter->sataChannel[channelIndex]->eDmaRegsOffset +
                               MV_SATA_II_IF_CONTROL_REG_OFFSET,
                               val);
        }







    }
    else
    {
        activateBMDmaMode(pAdapter, channelIndex, prdTableHi, prdTableLow,
                          dmaType);
    }
    mvOsSemRelease(&pSataChannel->semaphore);
    return res;
}


/*******************************************************************************
* mvSataC2CResetBmDma - reset B-M DMA
*
* DESCRIPTION:
*       Reset Bus Master DMA for the specific SATA channel
*
* INPUT:
*       pAdapter        - pointer to the adapter data structure.
*       channelIndex    - the index of the specific EDMA channel
*
* RETURN:
*       MV_TRUE on success
*       MV_FALSE on error
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN mvSataC2CResetBmDma(MV_SATA_ADAPTER *pAdapter,
                               MV_U8 channelIndex)
{
    MV_BUS_ADDR_T   ioBaseAddr = pAdapter->adapterIoBaseAddress;
    MV_SATA_CHANNEL *pSataChannel;
    MV_U32  val;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d %d "
                 "mvSataC2CResetBmDma failed - "
                 "Bad adapter data structure pointer.\n",
                 pAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel)
    {

        mvOsSemTake(&pSataChannel->semaphore);
    }

    /*Reset bm dma*/
    val = MV_REG_READ_DWORD (ioBaseAddr,
                             getEdmaRegOffset(channelIndex) +
                             MV_BMDMA_COMMAND_OFFSET);
    /*The DMA direction must be preserved*/
    val &= ~MV_BIT0;
    MV_REG_WRITE_DWORD (ioBaseAddr,
                        getEdmaRegOffset(channelIndex) +
                        MV_BMDMA_COMMAND_OFFSET, val);

    MV_REG_WRITE_DWORD(ioBaseAddr,
                       getEdmaRegOffset(channelIndex) +
                       MV_SATA_II_IF_CONTROL_REG_OFFSET,
                       0);

    if (pSataChannel)
    {
        mvOsSemRelease(&pSataChannel->semaphore);
    }
    return MV_TRUE;
}
#endif

#ifdef MV_SATA_IO_GRANULARITY

/*Public functions*/

/*******************************************************************************
* mvSataEnableIoGranularity - Enable/disable I/O granularity for the specific
*                             SATA adapter
*
* DESCRIPTION:
*               Enable/disable I/O granularity for the specific
*               SATA adapter. if IO/granularity is enabled, the function masks
*               all channel's and channel coalescing interrupts and enables
*               I/O granularity coalescing interupts
* INPUT:
*       pAdapter    - pointer to the adapter data structure.
*       enable    -  MV_TRUE to enable I/O granularity
*                    MV_FALSE to disable I/O granularity
*
* RETURN:
*       MV_TRUE if succeed
*       MV_FALSE otherwise
*
* COMMENTS:
*
*******************************************************************************/

MV_BOOLEAN mvSataEnableIoGranularity(MV_SATA_ADAPTER *pAdapter,
                                     MV_BOOLEAN enable)
{
    MV_U32 i;
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_FATAL_ERROR, " %d: "
                 "mvSataEnableIoGranularity Failed, Bad adapter data structure"
                 " pointer\n", pAdapter->adapterId);
        return MV_FALSE;
    }
    if (pAdapter->sataAdapterGeneration != MV_SATA_GEN_II)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d: "
                 "mvSataEnableIoGranularity Failed, Feature is not supported by HW\n",
                 pAdapter->adapterId);
        return MV_FALSE;
    }
    if (enable)
    {
        if (MV_TRUE == pAdapter->iogEnabled)
        {
            return MV_TRUE;
        }
        mvOsSemTake(&pAdapter->iogSemaphore);
        for (i = 0; i < MV_IOG_QUEUE_SIZE; i++)
        {
            pAdapter->iogFreeIdsStack[i] = i;







        }
        pAdapter->iogFreeIdsNum = MV_IOG_QUEUE_SIZE;
        pAdapter->iogEnabled = MV_TRUE;
        mvOsSemRelease(&pAdapter->iogSemaphore);

        mvOsSemTake(&pAdapter->interruptsMaskSem);
        pAdapter->mainMask &= ~(MV_BIT1 | MV_BIT3 |  MV_BIT5 |  MV_BIT7 |
                                MV_BIT10 | MV_BIT12 |  MV_BIT14 |  MV_BIT16 |
                                MV_BIT17 | MV_BIT8);
        pAdapter->mainMask |= MV_IOG_TRANS_INT_MASK;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                           pAdapter->mainMask);
        mvOsSemRelease(&pAdapter->interruptsMaskSem);







    }
    else
    {
        if (MV_FALSE == pAdapter->iogEnabled)
        {
            return MV_TRUE;
        }

        mvOsSemTake(&pAdapter->interruptsMaskSem);
        pAdapter->mainMask &= ~MV_IOG_TRANS_INT_MASK;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                           MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                           pAdapter->mainMask);
        mvOsSemRelease(&pAdapter->interruptsMaskSem);

        mvOsSemTake(&pAdapter->iogSemaphore);
        pAdapter->iogFreeIdsNum = 0;
        pAdapter->iogEnabled = MV_FALSE;
        mvOsSemRelease(&pAdapter->iogSemaphore);
    }
    return MV_TRUE;
}

/*Static functions*/

/*******************************************************************************
* setIoGranularityCount - Set I/O granularity transaction control register.
*
* DESCRIPTION:
*       This function Sets I/O granularity transaction control register for
*       specific transaction ID
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX60XX adapter data structure.
*       transId     -  transaction ID
*       counter     -  I/O granularity counter transaction for current
*                       transaction Id
*
* RETURN:
*       None
*
* COMMENTS:
*       This function assumes that the channel semaphore is locked.
*
*******************************************************************************/

static void setIoGranularityCount(MV_SATA_ADAPTER *pAdapter,
                                  MV_U8 transId,
                                  MV_U8 counter)
{
    MV_U32 offset  = MV_IOG_TRANS_CTRL_REG_OFFSET + transId;
    MV_U8 value  = (counter & 0x1F);
    MV_REG_WRITE_BYTE(pAdapter->adapterIoBaseAddress, offset, value);

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG | MV_DEBUG_UDMA_COMMAND,
             " %d : setIoGranularityCount "
             "writing 0x%X to 0x%X, transId = 0x%X\n",
             pAdapter->adapterId, value, offset, transId);
}

/*******************************************************************************
* readIoGranularityCount - Read I/O granularity transaction control register.
*
* DESCRIPTION:
*       This function reads I/O granularity transaction control register for
*       specific transaction ID
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX60XX adapter data structure.
*       transId     -  transaction ID
*
*
* RETURN:
*        I/O granularity counter for transaction Id
*
* COMMENTS:
*       This function assumes that the channel semaphore is locked.
*
*******************************************************************************/
static MV_U8 readIoGranularityCount(MV_SATA_ADAPTER *pAdapter,
                                    MV_U8 transId)
{
    MV_U32 offset  = MV_IOG_TRANS_CTRL_REG_OFFSET + transId;
    MV_U8 value  = MV_REG_READ_BYTE(pAdapter->adapterIoBaseAddress, offset);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS, " %d : readIoGranularityCount "
             "reading 0x%X from 0x%X, transId = 0x%X\n",
             pAdapter->adapterId, value, offset, transId);
    return value;
}


/*******************************************************************************
* checkIogBit - Check bit of I/O granularity cause register.
*
* DESCRIPTION:
*       Checks bits in I/O granularity cause register for completion
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX60XX adapter data structure.
*       bitOffset    - bit offset in register for current transaction id
*       value        - register value
*
* RETURN:
*        None
*
* COMMENTS:
*       Assume that is function is called while IO granularity semaphore is
*       locked
*******************************************************************************/

static void checkIogBit(MV_SATA_ADAPTER *pAdapter,
                        MV_U8 bitOffset,
                        MV_U8 value)
{
    MV_U32 i;
    MV_U8 id = 0x40;
    for (i = 0; i < 8; i++)
    {
        if ((value >> i) & 0x1)
        {
            MV_U8 iogCount;
            id = bitOffset + i;
            iogCount = readIoGranularityCount(pAdapter, id);
            if (iogCount > 0)
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS,
                         " %d: unexpected IO granularity "
                         "transaction counter = %d > 0\n",
                         pAdapter->adapterId, iogCount);
            }
            else
            {
                mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_INTERRUPTS,
                         " %d: IO granularity transaction Id 0x%X done.\n",
                         pAdapter->adapterId, id);
            }
            pAdapter->iogFreeIdsStack[pAdapter->iogFreeIdsNum++] = id;
        }
    }
}

/*******************************************************************************
* checkIogCompletion - Check bit of I/O granularity cause register.
*
* DESCRIPTION:
*       Checks I/O granularity completion
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX60XX adapter data structure.
*       iogCause     - I/O granularity cause register value
*       Offset      -  0 if transaction Id is 0-31, 32 if transaction Id 32-63
*
* RETURN:
*        None
*
* COMMENTS:
*
*******************************************************************************/

static void checkIogCompletion(MV_SATA_ADAPTER *pAdapter,
                               MV_U32 iogCause, MV_U8 offset)
{
    MV_U32 id;
    MV_U8  byte;

    if (iogCause & 0xFFFF0000)
    {
        byte = (MV_U8)(iogCause >> 24);
        if (byte)
        {
            checkIogBit(pAdapter, (offset) + 24, byte);
        }
        byte  =  (MV_U8)((iogCause >> 16) & 0xFF);
        if (byte)
        {
            checkIogBit(pAdapter, (offset) + 16, byte);
        }
    }
    if (iogCause & 0x0000FFFF)
    {
        byte = (MV_U8)(iogCause >> 8);
        if (byte)
        {
            checkIogBit(pAdapter, (offset) + 8, byte);
        }
        byte  =  (MV_U8)(iogCause & 0xFF);
        if (byte)
        {
            checkIogBit(pAdapter, (offset), byte);
        }
    }
}

/*******************************************************************************
* iogInterrupt - I/O granularity ISR.
*
* DESCRIPTION:
*       Checks bit of I/O granularity cause register for
*       specific transaction ID
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX60XX adapter data structure.
*       ioBaseAddr   - SATA Adapter base address
*       mainCause   - interrupt cause register
*
* RETURN:
*        None
*
* COMMENTS:
*
*******************************************************************************/

static void iogInterrupt(MV_SATA_ADAPTER *pAdapter,
                         MV_BUS_ADDR_T ioBaseAddr,
                         MV_U32 mainCause)
{
    MV_U32 iogCauseRegister;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG | MV_DEBUG_INTERRUPTS,
             " %d: IO Granularity Interrupt."
             "Cause = 0x%08x", pAdapter->adapterId, mainCause);
    if (mainCause & MV_IOG_TRANS_LOW_BIT)
    {

        iogCauseRegister = MV_REG_READ_DWORD(ioBaseAddr,
                                             MV_IOG_TRANS_LOW_REG_OFFSET);
        MV_REG_WRITE_DWORD (ioBaseAddr,
                            MV_IOG_TRANS_LOW_REG_OFFSET,
                            ~iogCauseRegister);
        if (MV_TRUE == pAdapter->iogEnabled)
            checkIogCompletion(pAdapter, iogCauseRegister, 0);
    }
    if (mainCause & MV_IOG_TRANS_HIGH_BIT)
    {
        iogCauseRegister = MV_REG_READ_DWORD(ioBaseAddr,
                                             MV_IOG_TRANS_HIGH_REG_OFFSET);
        MV_REG_WRITE_DWORD (ioBaseAddr,
                            MV_IOG_TRANS_HIGH_REG_OFFSET,
                            ~iogCauseRegister);
        if (MV_TRUE == pAdapter->iogEnabled)
            checkIogCompletion(pAdapter, iogCauseRegister, 32);
    }
}


/*******************************************************************************
* iogReset - reset all settings in HW related to I/O granularity.
*
* DESCRIPTION:
*       The function is executed when the error is occured and IO granularity
*       is enabled for the adapter. The function performs the following
*       operations
*       1. IO granularity interrupts are masked
*       2. Clear IO granularity cause registers
*       3. Reset all IO granularity transcation counters
*
* INPUT:
*       pAdapter     - Pointer to the MV88SX60XX adapter data structure.
*
* RETURN:
*        None
*
* COMMENTS:
*
*******************************************************************************/

static MV_BOOLEAN iogReset(MV_SATA_ADAPTER *pAdapter)
{
    MV_U32 i;

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " %d: IO Granularity error handler is executed.\n.",
             pAdapter->adapterId);
    /*Mask IO Granularity interrupt*/
    mvOsSemTake(&pAdapter->interruptsMaskSem);
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,
                       MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
                       (pAdapter->mainMask & (~MV_IOG_TRANS_INT_MASK)));
    mvOsSemRelease(&pAdapter->interruptsMaskSem);

    mvOsSemTake(&pAdapter->iogSemaphore);
    /*Clear IO Granularity cause registers*/
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_IOG_TRANS_LOW_REG_OFFSET,
                        0);
    MV_REG_WRITE_DWORD (pAdapter->adapterIoBaseAddress,
                        MV_IOG_TRANS_HIGH_REG_OFFSET,
                        0);

    /*Set all transaction counters to zero*/
    for (i = 0; i < MV_IOG_QUEUE_SIZE; i += 4)
    {
        MV_U32 offset  = MV_IOG_TRANS_CTRL_REG_OFFSET + i;
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress, offset, 0);
    }
    mvOsSemRelease(&pAdapter->iogSemaphore);
}



#endif

#ifdef MV_LOGGER
void _dumpPCIRegs(MV_SATA_ADAPTER *pAdapter)
{
    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d "
                 "_dumpPCIRegs failed - "
                 "Bad adapter data structure pointer.\n",
                 pAdapter->adapterId);
        return ;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d :Dump PCI Regs\n",
             pAdapter->adapterId);


    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Main interrupt Cause",MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Main interrupt Mask",MV_MAIN_INTERRUPT_MASK_REG_OFFSET,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                               MV_MAIN_INTERRUPT_MASK_REG_OFFSET));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "SErr Mask",0xC28,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0xc28));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Error Addr Low",0x1d40,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0x1d40));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Error Addr High",0x1d44,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0x1d44));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Error Attr",0x1d48,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0x1d48));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Error Command",0x1d50,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0x1d50));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Intr Cause",0x1d58,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0x1d58));
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%-25s  %04x %08x\n",
             "Intr Mask",0x1d5c,
             MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress, 0x1d5c));
}
static MV_U32 getRegField(MV_U32 regVal, MV_U32 fieldOff, MV_U32 bitsNum)
{
    MV_U32  mask = ((1 << bitsNum) - 1) << fieldOff;
    return(regVal & mask) >> fieldOff;
}
void _dumpEDMARegs(MV_SATA_ADAPTER *pMvSataAdapter, MV_U8 channelIndex)
{
    MV_U32  regVal, regOff;

    if (pMvSataAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d "
                 "_dumpEDMARegs failed - "
                 "Bad adapter data structure pointer.\n",
                 pMvSataAdapter->adapterId, channelIndex);
        return;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d:Dump Edma HW Regs\n",
             pMvSataAdapter->adapterId, channelIndex);

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_CONFIG_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " EMDA CFG   off 0x%08x val 0x%08x: depth 0x%x NCQ %x BURST SIZE %x "
             "eQueue %x Stop On Err %x BURST EXT %x WRITE BURST SIZE %x\n",
             regOff, regVal,
             getRegField(regVal, 0, 5),
             getRegField(regVal, 5, 1),
             getRegField(regVal, 8, 1),
             getRegField(regVal, 9, 1),
             getRegField(regVal, 10, 1),
             getRegField(regVal, 11, 1),
             getRegField(regVal, 13, 1));

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_INTERRUPT_ERROR_CAUSE_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Intr Cause off 0x%08x val 0x%08x\n", regOff, regVal);

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_INTERRUPT_ERROR_MASK_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Intr Mask  off 0x%08x val 0x%08x\n", regOff, regVal);

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_REQUEST_Q_BAH_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Req AddrHi off 0x%08x val 0x%08x\n", regOff, regVal);

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_REQUEST_Q_INP_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Req INP    off 0x%08x val 0x%08x: INP 0x%x BA 0x%x\n",
             regOff, regVal,
             getRegField(regVal, 5, 5),
             getRegField(regVal, 10, 22));

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_REQUEST_Q_OUTP_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Req OUTP   off 0x%08x val 0x%08x: OUT 0x%x\n",
             regOff, regVal,
             getRegField(regVal, 5, 5));

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_RESPONSE_Q_BAH_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Res AddrHi off 0x%08x val 0x%08x\n",
             regOff, regVal);

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_RESPONSE_Q_INP_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Res INP    off 0x%08x val 0x%08x: INP 0x%x\n",
             regOff, regVal,
             getRegField(regVal, 3, 5));

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_RESPONSE_Q_OUTP_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Res OUTP   off 0x%08x val 0x%08x: OUTP 0x%x BA 0x%x\n",
             regOff, regVal,
             getRegField(regVal, 3, 5),
             getRegField(regVal, 8, 24));

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_COMMAND_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Command    off 0x%08x val 0x%08x: EN %x DIS %x HW RESET %x\n",
             regOff, regVal,
             getRegField(regVal, 0, 1),
             getRegField(regVal, 1, 1),
             getRegField(regVal, 2, 1));

    regOff = edmaRegOffst[ channelIndex] + MV_EDMA_STATUS_REG_OFFSET;
    regVal = mvSataReadReg(pMvSataAdapter, regOff);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR," Status     off 0x%08x val 0x%08x: TAG 0x%x\n",
             regOff, regVal,
             getRegField(regVal, 0, 5));







}

void _dumpChannelQueues(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_SATA_CHANNEL *pSataChannel;
    MV_U32  i;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d "
                 "_dumpChannelQueues failed - "
                 "Bad adapter data structure pointer.\n",
                 pAdapter->adapterId, channelIndex);
        return ;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "%d %d:Dump Channel Queues\n",
             pAdapter->adapterId, channelIndex);
    pSataChannel = pAdapter->sataChannel[channelIndex];
    if (pSataChannel == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d: _dumpChannelQueues"
                 "el Failed, channel data structure not allocated\n",
                 pAdapter->adapterId, channelIndex);
        return ;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "Request Qeueu Info:\n");
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " virt addr %p:\n",
             pSataChannel->requestQueue);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " phy low addr %08x:\n",
             pSataChannel->requestQueuePciLowAddress);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " phy high addr %08x:\n",
             pSataChannel->requestQueuePciHiAddress);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " SW IN pointer %x:\n",
             pSataChannel->reqInPtr);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "Request Qeueu Entries:\n");
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "Index [3:0]       [7:4]       [11:8]       [15:12]\n");
    for (i = 0; i < MV_EDMA_QUEUE_LENGTH; i++)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 "[%2d] 0x%08x  0x%08x  0x%08x  0x%08x  0x%08x  0x%08x  0x%08x  0x%08x\n",i,
                 *((MV_U32 *)(pSataChannel->requestQueue + i)),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 1),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 2),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 3),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 4),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 5),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 6),
                 *(((MV_U32 *)(pSataChannel->requestQueue + i)) + 7));
    }

    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "Responset Qeueu Info:\n");
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " virt addr %p:\n",
             pSataChannel->responseQueue);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " phy low addr %08x:\n",
             pSataChannel->responseQueuePciLowAddress);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " phy high addr %08x:\n",
             pSataChannel->responseQueuePciHiAddress);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " SW OUT pointer %x:\n",
             pSataChannel->rspOutPtr);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, "Response Qeueu Entries:\n");
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "Index [1:0]   [3:2]   [7:4]\n");
    for (i = 0; i < MV_EDMA_QUEUE_LENGTH; i++)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 "[%2d] 0x%04x  0x%04x  0x%08x\n",i,
                 *((MV_U16 *)(pSataChannel->responseQueue + i)),
                 *(((MV_U16 *)(pSataChannel->responseQueue + i)) + 1),
                 *(((MV_U32 *)(pSataChannel->responseQueue + i)) + 1));
    }







}
void _dumpSataRegs(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex)
{
    MV_U32  regVal;

    if (pAdapter == NULL)
    {
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR, " %d %d "
                 "_dumpSataRegs failed - "
                 "Bad adapter data structure pointer.\n",
                 pAdapter->adapterId, channelIndex);
        return;
    }

    if (pAdapter->sataAdapterGeneration == MV_SATA_GEN_II)
    {
        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegOffst[ channelIndex]  +
                                   MV_SATA_II_S_STATUS_REG_OFFSET);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " SStatus  0x%08x:\n", regVal);

        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegOffst[ channelIndex]  +
                                   MV_SATA_II_S_CONTROL_REG_OFFSET);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " SControl 0x%08x:\n", regVal);

        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegOffst[ channelIndex]  +
                                   MV_SATA_II_S_ERROR_REG_OFFSET);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " SError    0x%08x:\n", regVal);

        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegOffst[ channelIndex]  +
                                   MV_SATA_II_IF_CONTROL_REG_OFFSET);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " IF Ctrl  0x%08x: TXPort 0x%x\n", regVal,
                 getRegField(regVal, 0, 4));
        regVal = MV_REG_READ_DWORD(pAdapter->adapterIoBaseAddress,
                                   edmaRegOffst[ channelIndex]  +
                                   MV_SATA_II_IF_STATUS_REG_OFFSET);
        mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
                 " IF status 0x%08x: RXFIS 0x%x RXPort 0x%x\n", regVal,
                 getRegField(regVal, 0, 8),
                 getRegField(regVal, 8, 4));
    }
}
void _printATARegs(MV_STORAGE_DEVICE_REGISTERS   *pDeviceRegs)
{
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             " ATA Drive Registers:\n");
    if (pDeviceRegs == NULL)
    {
        return;
    }
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","Error", pDeviceRegs->errorRegister);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","SectorCount", pDeviceRegs->sectorCountRegister);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","LBA Low", pDeviceRegs->lbaLowRegister);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","LBA Mid", pDeviceRegs->lbaMidRegister);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","LBA High", pDeviceRegs->lbaHighRegister);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","Device", pDeviceRegs->deviceRegister);
    mvLogMsg(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
             "%20s : %04x\n","Status", pDeviceRegs->statusRegister);
}

#endif /*MV_LOGGER*/






