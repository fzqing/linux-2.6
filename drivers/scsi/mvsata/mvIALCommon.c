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
* mvIALCommon.c
*
* DESCRIPTION:
*       C implementation for IAL's common functions.
*
* DEPENDENCIES:
*   mvIALCommon.h
*
*******************************************************************************/

/* includes */
#include "mvOs.h"
#include "mvScsiAtaLayer.h"
#include "mvIALCommon.h"
#include "mvIALCommonUtils.h"
#include "mvStorageDev.h"


/* defines */
#undef DISABLE_PM_SCC
/* typedefs */

/*Static functions*/
static MV_BOOLEAN mvGetEDMAAllowedModes(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_SAL_ADAPTER_EXTENSION *pScsiAdapterExt,
                                        MV_U8 channelIndex,
                                        MV_BOOLEAN *TCQ,
                                        MV_BOOLEAN *NCQ,
                                        MV_U8   *queueDepth,
                                        MV_U8   *numOfDrives);

static void mvFlushSCSICommandQueue(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                    MV_U8 channelIndex);

static void mvAddToSCSICommandQueue(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                    MV_U8 channelIndex,
                                    MV_SATA_SCSI_CMD_BLOCK *Scb);

static MV_BOOLEAN mvAdapterStateMachine(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static MV_BOOLEAN mvChannelStateMachine(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_U8 channelIndex,
                                       MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static void mvSetChannelState(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_U8 channelIndex,
                              MV_CHANNEL_STATE state);


/*PM related*/
static MV_BOOLEAN mvPMCommandCompletionCB(MV_SATA_ADAPTER *pSataAdapter,
                                          MV_U8 channelIndex,
                                          MV_COMPLETION_TYPE comp_type,
                                          MV_VOID_PTR commandId,
                                          MV_U16 responseFlags,
                                          MV_U32 timeStamp,
                                          MV_STORAGE_DEVICE_REGISTERS *registerStruct);


static MV_BOOLEAN mvQueuePMAccessRegisterCommand(
                                                MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt,
                                                MV_U8 channelIndex,
                                                MV_U8 PMPort,
                                                MV_U8 PMReg,
                                                MV_U32 Value,
                                                MV_BOOLEAN isRead);




static MV_BOOLEAN mvPMEstablishSataComm(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_U8 channelIndex);

static MV_BOOLEAN mvPMCheckForConnectedDevices(
                                              MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                              MV_U8 channelIndex,
                                              MV_BOOLEAN* bIsChannelReady);


static MV_BOOLEAN mvInitPMDevice(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                 MV_U8 channelIndex,
                                 MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                                 MV_BOOLEAN* bIsChannelReady,
                                 MV_BOOLEAN* isDeviceReconnected,
                                 MV_BOOLEAN* bPMErrorDetected);

static MV_BOOLEAN mvPMEnableCommStatusChangeBits(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                 MV_U8 channelIndex,
                                                 MV_BOOLEAN enable);

static MV_BOOLEAN mvPMEnableAsyncNotify(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_U8 channelIndex);


static void mvCheckPMForError(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_U8 channelIndex);

/*End PM related*/

static MV_BOOLEAN mvStartChannelInit(MV_SATA_ADAPTER *pSataAdapter,
                                     MV_U8 channelIndex,
                                     MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                                     MV_BOOLEAN *isChannelReady);

static MV_BOOLEAN mvChannelSRSTFinished(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_SATA_CHANNEL *pSataChannel,
                                        MV_U8 channelIndex,
                                        MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                                        MV_BOOLEAN* bIsChannelReady,
                                        MV_BOOLEAN* bFatalError);

static MV_BOOLEAN mvConfigChannelQueuingMode(
                                            MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt,
                                            MV_U8 channelIndex,
                                            MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static MV_BOOLEAN mvConfigChannelDMA(
                                    MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt,
                                    MV_U8 channelIndex,
                                    MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static void mvSetChannelTimer(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_U8 channelIndex,
                              MV_U32 timeout);
static void mvDecrementChannelTimer(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                    MV_U8 channelIndex);
static MV_BOOLEAN mvIsChannelTimerExpired(
                                         MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                         MV_U8 channelIndex);


/*Channel state machine*/


static MV_BOOLEAN mvChannelNotConnectedStateHandler(
                                                   MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                   MV_U8 channelIndex,
                                                   MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);


static MV_BOOLEAN mvChannelConnectedStateHandler(
                                                MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                MV_U8 channelIndex,
                                                MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

MV_BOOLEAN mvChannelInSrstStateHandler(
                                      MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                      MV_U8 channelIndex,
                                      MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static MV_BOOLEAN mvChannelPMStaggeredSpinUpStateHandler(
                                                        MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                        MV_U8 channelIndex,
                                                        MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static MV_BOOLEAN mvChannelPMSrstDeviceStateHandler(
                                                   MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                   MV_U8 channelIndex,
                                                   MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

static MV_BOOLEAN mvChannelReadyStateHandler(
                                            MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                            MV_U8 channelIndex);

static MV_BOOLEAN mvChannelPMHotPlugStateHandler(
                                                MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                MV_U8 channelIndex,
                                                MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);



static void mvDrivesInfoSaveAll(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex);

static void mvDrivesInfoFlushAll(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex);

static void mvDrivesInfoFlushSingleDrive(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex, MV_U8 PMPort);

static void mvDrivesInfoSaveSingleDrive(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                      MV_U8 channelIndex,
                                      MV_U8 PMPort,
                                      MV_BOOLEAN  isDriveAdded,
                                      MV_U16_PTR identifyBuffer);

static void mvSetDriveReady(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                            MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                            MV_U8 channelIndex,
                            MV_U8 PMPort,
                            MV_BOOLEAN  isReady,
                            MV_U16_PTR identifyBuffer);

static void mvDrivesInfoGetChannelRescanParams(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                              MV_U8 channelIndex,
                                              MV_U16 *drivesToRemove,
                                              MV_U16 *drivesToAdd);



#ifdef DISABLE_PM_SCC
MV_BOOLEAN mvPMDisableSSC(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex);
#endif

MV_BOOLEAN mvPMEnableLocking(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex);



/*Public functions*/

/*******************************************************************************
* mvAdapterStartInitialization - start adapter initialization
*
* DESCRIPTION:
*  Starts adapter initialization after driver load.
*  State - machine related data structure is initialized for adapter
*  and its channels. Begin staggered spin-up.
*  Adapter state is changed to ADAPTER_READY.
* INPUT:
*    pAdapter    - pointer to the adapter data structure.
*    scsiAdapterExt  - SCSI to ATA layer adapter extension data structure
* OUTPUT:
*    None.
*
* RETURN:
*    MV_TRUE on success
*    MV_FALSE on error
*
*******************************************************************************/

MV_BOOLEAN mvAdapterStartInitialization(MV_SATA_ADAPTER *pSataAdapter,
                                        MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_U8 channelIndex;
    ialExt->pSataAdapter = pSataAdapter;
    ialExt->adapterState = ADAPTER_INITIALIZING;
    for (channelIndex = 0; channelIndex < MV_SATA_CHANNELS_NUM; channelIndex++)
    {
        ialExt->channelState[channelIndex] = CHANNEL_NOT_CONNECTED;
        ialExt->IALChannelExt[channelIndex].SRSTTimerThreshold = 0;
        ialExt->IALChannelExt[channelIndex].SRSTTimerValue = 0;
        ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue = NULL;
        ialExt->IALChannelExt[channelIndex].completionError = MV_FALSE;
        ialExt->IALChannelExt[channelIndex].pmRegAccessInProgress = MV_FALSE;
        ialExt->IALChannelExt[channelIndex].devInSRST =
        MV_SATA_PM_CONTROL_PORT + 1;
        ialExt->IALChannelExt[channelIndex].PMdevsToInit = 0;
        ialExt->IALChannelExt[channelIndex].PMnumberOfPorts = 0;
        ialExt->IALChannelExt[channelIndex].pmAccessType = 0;
        ialExt->IALChannelExt[channelIndex].pmReg = 0;
        ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled = MV_FALSE;
        ialExt->IALChannelExt[channelIndex].bHotPlug = MV_FALSE;
        memset(&ialExt->IALChannelExt[channelIndex].drivesInfo, 0, sizeof(MV_DRIVES_INFO));
    }
    return mvAdapterStateMachine(ialExt, scsiAdapterExt);
}


/*******************************************************************************
* mvRestartChannel - restart specific channel
*
* DESCRIPTION:
*  The function is used in channel hot-plug to restart the channel
*  initialization sequence. The channel stated is changed to
*  CHANNEL_CONNECTED and any pending command in software queue are flushed
* INPUT:
*    pAdapter    - pointer to the adapter data structure.
*    channelIndex  - channel number
*    scsiAdapterExt  - SCSI to ATA layer adapter extension data structure
*    bBusReset       - MV_TRUE if the faunction is called because of bus reset,
*                       MV_FALSE otherwise
* OUTPUT:
*    None.
*
* RETURN:
*    MV_TRUE on success
*    MV_FALSE on error
*
*******************************************************************************/

void mvRestartChannel(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                      MV_U8 channelIndex,
                      MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                      MV_BOOLEAN bBusReset)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_BOOLEAN bBusChangeNotify = MV_FALSE;
    ialExt->IALChannelExt[channelIndex].bHotPlug = MV_TRUE;
    mvSetDriveReady(ialExt,
                    scsiAdapterExt,
                    channelIndex, 0xFF, MV_FALSE, NULL);
    if (pSataAdapter->sataChannel[channelIndex] != NULL)
    {
        if (mvStorageDevGetDeviceType(pSataAdapter,channelIndex)
            == MV_SATA_DEVICE_TYPE_PM)
        {
            bBusChangeNotify = MV_TRUE;
        }
        mvSataDisableChannelDma(pSataAdapter, channelIndex);
        mvSataFlushDmaQueue (pSataAdapter, channelIndex,
                             MV_FLUSH_TYPE_CALLBACK);
    }
    mvFlushSCSICommandQueue(ialExt, channelIndex);
    ialExt->IALChannelExt[channelIndex].SRSTTimerThreshold = 0;
    ialExt->IALChannelExt[channelIndex].SRSTTimerValue = 0;
    ialExt->channelState[channelIndex] = CHANNEL_CONNECTED;
    if (bBusReset == MV_TRUE)
    {
        if (bBusChangeNotify == MV_TRUE)
        {
            /*Notify about bus change*/
            IALBusChangeNotify(pSataAdapter, channelIndex);
        }
    }
    else
    {
        /*Notify about bus change*/
        IALBusChangeNotify(pSataAdapter, channelIndex);
    }
}



/*******************************************************************************
* mvPMHotPlugDetected - restart specific channel
*
* DESCRIPTION:
*  The function is used in PM hot-plug to wait for empty EDMA command queue
*  and then restart the channel initialization sequence.
*  The channel stated is changed to CHANNEL_PM_HOT_PLUG if there are any
*  pending command in EDMA queue
* INPUT:
*    pAdapter    - pointer to the adapter data structure.
*    channelIndex  - channel number

* OUTPUT:
*    None.
*
* RETURN:
*    None
*
*******************************************************************************/

void mvPMHotPlugDetected(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                         MV_U8 channelIndex,
                         MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    if (ialExt->channelState[channelIndex] == CHANNEL_NOT_CONNECTED ||
        ialExt->channelState[channelIndex] == CHANNEL_CONNECTED ||
        ialExt->channelState[channelIndex] == CHANNEL_IN_SRST)
    {
        return;
    }
    mvSataDisableChannelDma(ialExt->pSataAdapter, channelIndex);
    mvSataFlushDmaQueue (ialExt->pSataAdapter,
                         channelIndex, MV_FLUSH_TYPE_CALLBACK);
    mvSataChannelHardReset(ialExt->pSataAdapter, channelIndex);
    mvRestartChannel(ialExt, channelIndex, scsiAdapterExt, MV_FALSE);
}

/*******************************************************************************
* mvStopChannel - stop channel
*
* DESCRIPTION:
*  The function is used when the channel is unplugged.
*  The channel stated is changed to CHANNEL_NOT_CONNECTED
*  until further connection.
* INPUT:
*    pAdapter    - pointer to the adapter data structure.
*    channelIndex  - channel number
* OUTPUT:
*    None.
*
* RETURN:
*    None
*******************************************************************************/
void mvStopChannel(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                   MV_U8 channelIndex,
                   MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_U16 drivesSnapshot =
        ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotSaved;
    mvDrivesInfoFlushAll(ialExt, channelIndex);
    mvSetDriveReady(ialExt, scsiAdapterExt, channelIndex, 0xFF, MV_FALSE, NULL);
    mvSetChannelState(ialExt, channelIndex, CHANNEL_NOT_CONNECTED);
    if (pSataAdapter->sataChannel[channelIndex] != NULL)
    {
        mvSataDisableChannelDma(ialExt->pSataAdapter, channelIndex);
        mvSataFlushDmaQueue (ialExt->pSataAdapter, channelIndex,
                             MV_FLUSH_TYPE_CALLBACK);
    }
    mvFlushSCSICommandQueue(ialExt, channelIndex);
    if (pSataAdapter->sataChannel[channelIndex] != NULL)
    {
        mvSataRemoveChannel(pSataAdapter,channelIndex);
        IALReleaseChannel(pSataAdapter, channelIndex);
    }
    pSataAdapter->sataChannel[channelIndex] = NULL;
    /*Notify about bus change*/
    IALBusChangeNotify(pSataAdapter, channelIndex);
    if (drivesSnapshot != 0)
    {
        IALBusChangeNotifyEx(pSataAdapter, channelIndex, drivesSnapshot, 0);
    }
}


/*******************************************************************************
* mvExecuteScsiCommand - execute SCSI command
*
* DESCRIPTION:
*  IAL common layer wrapper of mvSataExecuteScsiCommand function.
*  If either the adapter state is either other than ADAPTER_READY
*  or the channel is connected but channel state is not CHANNEL_READY,
*  the current SCSI command is queued in channel's SCSI commands
*  software queue until channel initialization sequence completed.
*  If channel is found in CHANNEL ready state the SCSI command is passed to
*  SCSI ATA translation layer.
* INPUT:
*    pScb    - SCSI command block structure.
*
* OUTPUT:
*    None.
*
* RETURN:
*    Return MV_SCSI_COMMAND_STATUS_COMPLETED if the command has been added
*    to channel software queue. Otherwise return the result of
*    mvSataExecuteScsiCommand function call
*******************************************************************************/
MV_SCSI_COMMAND_STATUS_TYPE mvExecuteScsiCommand(MV_SATA_SCSI_CMD_BLOCK *pScb,
                                                 MV_BOOLEAN canQueue)
{
    MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt = pScb->pIalAdapterExtension;
    MV_U8 channelIndex = pScb->bus;

    if ((ialExt->adapterState == ADAPTER_READY) &&
        (ialExt->channelState[channelIndex] == CHANNEL_READY))
    {
        mvCheckPMForError(ialExt, channelIndex);
    }

    if ((ialExt->adapterState == ADAPTER_READY) &&
        ((ialExt->channelState[channelIndex] == CHANNEL_READY) ||
         (ialExt->channelState[channelIndex] == CHANNEL_NOT_CONNECTED)))
    {
        return mvSataExecuteScsiCommand(pScb);
    }
    if (canQueue == MV_FALSE)
    {
        pScb->ScsiStatus = MV_SCSI_STATUS_BUSY;
        pScb->dataTransfered = 0;
        pScb->ScsiCommandCompletion = MV_SCSI_COMPLETION_QUEUE_FULL;
        if (pScb->completionCallBack)
        {
            pScb->completionCallBack(ialExt->pSataAdapter, pScb);
        }
        return MV_SCSI_COMMAND_STATUS_COMPLETED;
    }
    else
    {
        mvAddToSCSICommandQueue(ialExt, channelIndex, pScb);
        return MV_SCSI_COMMAND_STATUS_QUEUED_BY_IAL;
    }
}


/*******************************************************************************
* mvIALTimerCallback - IAL timer callback
*
* DESCRIPTION:
*  The adapter/channel state machine is timer-driven.
*  After being loaded, the IAL must call this callback every 0.5 seconds
* INPUT:
*    pSataAdapter    - pointer to the adapter data structure.
*    scsiAdapterExt  - SCSI to ATA layer adapter extension data structure
* OUTPUT:
*    None.
*
* RETURN:
*
*******************************************************************************/

MV_BOOLEAN mvIALTimerCallback(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{

    return mvAdapterStateMachine(ialExt,
                                 scsiAdapterExt);
}

/*******************************************************************************
* mvCommandCompletionErrorHandler - IAL common command completion error handler
*
* DESCRIPTION:
*  Called by whether SAL completion of SMART completion function. Check whether
*  command is failed because of PM hot plug
*
* INPUT:
*    pSataAdapter    - pointer to the adapter data structure.
*    channelIndex    - channelNumber
* OUTPUT:
*    None.
*
* RETURN:
*
*******************************************************************************/

void mvCommandCompletionErrorHandler(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                     MV_U8 channelIndex)
{
    MV_SATA_ADAPTER* pSataAdapter = ialExt->pSataAdapter;
    if (pSataAdapter->sataChannel[channelIndex] == NULL)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: "
                 "Invalid channel data structure pointer.\n",
                 pSataAdapter->adapterId, channelIndex);
    }

    if ((ialExt->channelState[channelIndex] != CHANNEL_READY) ||
        (mvStorageDevGetDeviceType(pSataAdapter,channelIndex) !=
         MV_SATA_DEVICE_TYPE_PM) ||
        (ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled == MV_TRUE))
    {
        return;
    }
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: "
             "Set completion error to MV_TRUE.\n",
             pSataAdapter->adapterId, channelIndex);
    ialExt->IALChannelExt[channelIndex].completionError = MV_TRUE;
}

/*Static functions*/


static void printAtaDeviceRegisters(
                                   MV_STORAGE_DEVICE_REGISTERS *mvStorageDevRegisters)
{

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "ATA Drive Registers:\n");
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","Error",
             mvStorageDevRegisters->errorRegister);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","SectorCount",
             mvStorageDevRegisters->sectorCountRegister);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","LBA Low",
             mvStorageDevRegisters->lbaLowRegister);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","LBA Mid",
             mvStorageDevRegisters->lbaMidRegister);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","LBA High",
             mvStorageDevRegisters->lbaHighRegister);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","Device",
             mvStorageDevRegisters->deviceRegister);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%20s : %04x\n","Status",
             mvStorageDevRegisters->statusRegister);
}



static void mvDrivesInfoSaveAll(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex)
{
    /*Save disk drives information for channel*/
    ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotSaved =
        ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent;
    memcpy(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialSaved,
        ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent,
        sizeof(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent));
    /*Reset current disk drives information*/
    ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent = 0;
    memset(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent,
            0,
        sizeof(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent));

}

static void mvDrivesInfoFlushAll(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex)
{
    /*Flush drives info*/
    memset(&ialExt->IALChannelExt[channelIndex].drivesInfo, 0,
            sizeof(ialExt->IALChannelExt[channelIndex].drivesInfo));
}

static void mvDrivesInfoFlushSingleDrive(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex, MV_U8 PMPort)
{
    /*Clear bit in disk drive drive snapshot*/
    ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent &=
        ~(1 << PMPort);
    ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotSaved &=
        ~(1 << PMPort);
    /*Clear disk drive serial number string*/
    ialExt->
       IALChannelExt[channelIndex].drivesInfo.driveSerialSaved[PMPort].serial[0] = 0;
    ialExt->
        IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent[PMPort].serial[0] = 0;
}


static void mvDrivesInfoSaveSingleDrive(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                      MV_U8 channelIndex,
                                      MV_U8 PMPort,
                                      MV_BOOLEAN  isDriveAdded,
                                      MV_U16_PTR identifyBuffer)
{
    if (MV_TRUE == isDriveAdded)
    {
        /*Set bit in disk drive snapshot for current disk drive*/
        ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent |=
                1 << PMPort;
        /*Save serial number for current disk drive*/
        memcpy(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent[PMPort].serial,
               &identifyBuffer[IDEN_SERIAL_NUM_OFFSET], IDEN_SERIAL_NUM_SIZE);
    }
    else
    {
        if (0xFF == PMPort)
        {
            ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent
                 = 0;
            memset(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent,
               0,
               sizeof(ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent));
        }
        else
        {
            ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent
                    &= ~(1 << PMPort);
            ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent[PMPort].serial[0] = 0;
        }
    }
}

static void mvSetDriveReady(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                            MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                            MV_U8 channelIndex,
                            MV_U8 PMPort,
                            MV_BOOLEAN  isReady,
                            MV_U16_PTR identifyBuffer)
{
    mvDrivesInfoSaveSingleDrive(ialExt,
                            channelIndex,
                            PMPort,
                            isReady,
                            identifyBuffer);
    mvSataScsiSetDriveReady(scsiAdapterExt,
                            channelIndex, PMPort, isReady);
}


static void mvDrivesInfoGetChannelRescanParams(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                              MV_U8 channelIndex,
                                              MV_U16 *drivesToRemove,
                                              MV_U16 *drivesToAdd)
{
    MV_U8 PMPort;

    *drivesToRemove = 0;
    *drivesToAdd = 0;

    for (PMPort = 0; PMPort < MV_SATA_PM_MAX_PORTS; PMPort++)
    {
        if (ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotCurrent
            & (1 << PMPort))
        {
            if (ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotSaved
                & (1 << PMPort))
            {
                if (memcmp(&ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialCurrent[PMPort].serial,
                    &ialExt->IALChannelExt[channelIndex].drivesInfo.driveSerialSaved[PMPort].serial,
                        IDEN_SERIAL_NUM_SIZE))
                {
                    /*Disk drive connected to port is replaced*/
                    *drivesToAdd |= (1 << PMPort);
                    *drivesToRemove |= (1 << PMPort);
                }
            }
            else
            {
                /*New drive connected to port*/
                *drivesToAdd |= (1 << PMPort);
            }
        }
        else
        {
            /*Drive removed from Port*/
            if (ialExt->IALChannelExt[channelIndex].drivesInfo.drivesSnapshotSaved
                & (1 << PMPort))
            {
                *drivesToRemove |= (1 << PMPort);
            }
        }
    }
}



/*SCSI command queue functions*/
static void mvAddToSCSICommandQueue(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                    MV_U8 channelIndex,
                                    MV_SATA_SCSI_CMD_BLOCK *pScb)
{

    MV_SATA_SCSI_CMD_BLOCK *cmdBlock = (MV_SATA_SCSI_CMD_BLOCK *)
                                       ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue;
    pScb->pNext = NULL;
    if (cmdBlock)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d] Adding next command to SW queue\n",
                 ialExt->pSataAdapter->adapterId, channelIndex);
        while (cmdBlock->pNext)
        {
            cmdBlock = cmdBlock->pNext;
        }
        cmdBlock->pNext = pScb;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d] Adding first command to SW queue\n",
                 ialExt->pSataAdapter->adapterId, channelIndex);
        ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue =
        (MV_VOID_PTR)pScb;
    }
}

MV_BOOLEAN mvRemoveFromSCSICommandQueue(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex,
                                        MV_SATA_SCSI_CMD_BLOCK *pScb)
{

    MV_SATA_SCSI_CMD_BLOCK *cmdBlock = (MV_SATA_SCSI_CMD_BLOCK *)
                                       ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue;
    pScb->pNext = NULL;
    if (cmdBlock)
    {
        if (cmdBlock == pScb)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d] Removing command"
                     " %p from head of SW queue\n",
                     ialExt->pSataAdapter->adapterId,channelIndex, pScb);
            ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue =
            (MV_VOID_PTR) cmdBlock->pNext;
            return MV_TRUE;
        }
        else
        {
            while (cmdBlock->pNext)
            {
                if (cmdBlock->pNext == pScb)
                {
                    break;
                }
                cmdBlock = cmdBlock->pNext;
            }
            if (cmdBlock->pNext == NULL)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d] Removing"
                         " command %p from SW queue failed. command not found\n",
                         ialExt->pSataAdapter->adapterId,channelIndex, pScb);
                return MV_FALSE;
            }
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d] Removing command"
                     " %p from SW queue\n", ialExt->pSataAdapter->adapterId,
                     channelIndex, pScb);
            cmdBlock->pNext = cmdBlock->pNext->pNext;
            return MV_TRUE;
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d] Removing"
                 " command %p from SW queue failed. queue empty\n",
                 ialExt->pSataAdapter->adapterId,channelIndex, pScb);
        return MV_FALSE;
    }
    return MV_FALSE;
}

static void mvFlushSCSICommandQueue(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                    MV_U8 channelIndex)
{
    /*Abort all pending commands in SW queue*/
    MV_SATA_SCSI_CMD_BLOCK *cmdBlock = (MV_SATA_SCSI_CMD_BLOCK *)
                                       ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue;

    while (cmdBlock)
    {
        MV_SATA_SCSI_CMD_BLOCK *nextBlock = cmdBlock->pNext;
        if (cmdBlock->completionCallBack)
        {
            cmdBlock->ScsiStatus = MV_SCSI_STATUS_BUSY;
            cmdBlock->dataTransfered = 0;
            cmdBlock->ScsiCommandCompletion = MV_SCSI_COMPLETION_QUEUE_FULL;
            cmdBlock->completionCallBack(ialExt->pSataAdapter, cmdBlock);
        }
        cmdBlock = nextBlock;
    }
    ialExt->IALChannelExt[channelIndex].IALChannelPendingCmdQueue = NULL;
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: Flush command queue is done\n",
             ialExt->pSataAdapter->adapterId, channelIndex);
}


/*Port Multilier related functions*/
static MV_BOOLEAN mvPMCommandCompletionCB(MV_SATA_ADAPTER *pSataAdapter,
                                          MV_U8 channelIndex,
                                          MV_COMPLETION_TYPE comp_type,
                                          MV_VOID_PTR commandId,
                                          MV_U16 responseFlags,
                                          MV_U32 timeStamp,
                                          MV_STORAGE_DEVICE_REGISTERS *registerStruct)
{
    MV_U32 value;
    MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt =
    (MV_IAL_COMMON_ADAPTER_EXTENSION *)commandId;
    ialExt->IALChannelExt[channelIndex].pmRegAccessInProgress = MV_FALSE;
    switch (comp_type)
    {
    case MV_COMPLETION_TYPE_NORMAL:
        if (ialExt->IALChannelExt[channelIndex].pmAccessType
            == MV_ATA_COMMAND_PM_READ_REG)
        {
            value = registerStruct->sectorCountRegister;
            value |= (registerStruct->lbaLowRegister << 8);
            value |= (registerStruct->lbaMidRegister << 16);
            value |= (registerStruct->lbaMidRegister << 24);
            if (ialExt->IALChannelExt[channelIndex].pmReg
                == MV_SATA_GSCR_ERROR_REG_NUM)
            {
                if (value != 0)
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: PM GSCR[32] = 0x%X\n",
                             pSataAdapter->adapterId, channelIndex, value);
                    ialExt->IALChannelExt[channelIndex].PMdevsToInit =
                    (MV_U16)(value & 0x7FFF);
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: "
                             "PM Hot plug detected "
                             "Bitmask = 0x%X\n",
                             pSataAdapter->adapterId, channelIndex,
                             ialExt->IALChannelExt[channelIndex].PMdevsToInit);
                    mvSetChannelState(ialExt, channelIndex,
                                      CHANNEL_PM_HOT_PLUG);
                }
            }
        }
        break;
    case MV_COMPLETION_TYPE_ABORT:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: read PM register aborted!\n",
                 pSataAdapter->adapterId, channelIndex);

        break;
    case MV_COMPLETION_TYPE_ERROR:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: read PM register error!\n",
                 pSataAdapter->adapterId, channelIndex);
        break;
    default:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Unknown completion type (%d)\n",
                 pSataAdapter->adapterId, channelIndex, comp_type);
        return MV_FALSE;
    }
    return MV_TRUE;
}


static MV_BOOLEAN mvQueuePMAccessRegisterCommand(
                                                MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt,
                                                MV_U8 channelIndex,
                                                MV_U8 PMPort,
                                                MV_U8 PMReg,
                                                MV_U32 Value,
                                                MV_BOOLEAN isRead)
{
    MV_QUEUE_COMMAND_INFO commandParams;
    MV_QUEUE_COMMAND_RESULT result;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    memset(&commandParams, 0, sizeof(commandParams));
    commandParams.type = MV_QUEUED_COMMAND_TYPE_NONE_UDMA;
    commandParams.commandParams.NoneUdmaCommand.protocolType =
    MV_NON_UDMA_PROTOCOL_NON_DATA;
    commandParams.commandParams.NoneUdmaCommand.isEXT = MV_FALSE;
    commandParams.commandParams.NoneUdmaCommand.bufPtr = NULL;
    commandParams.PMPort = MV_SATA_PM_CONTROL_PORT;
    commandParams.commandParams.NoneUdmaCommand.count = 0;
    commandParams.commandParams.NoneUdmaCommand.features = PMReg;
    commandParams.commandParams.NoneUdmaCommand.device = (MV_U8)PMPort;
    commandParams.commandParams.NoneUdmaCommand.callBack =
    mvPMCommandCompletionCB;

    ialExt->IALChannelExt[channelIndex].pmReg = PMReg;

    if (isRead == MV_TRUE)
    {
        ialExt->IALChannelExt[channelIndex].pmAccessType =
        MV_ATA_COMMAND_PM_READ_REG;
        commandParams.commandParams.NoneUdmaCommand.command =
        MV_ATA_COMMAND_PM_READ_REG;
        commandParams.commandParams.NoneUdmaCommand.commandId =
        (MV_VOID_PTR)ialExt;
        commandParams.commandParams.NoneUdmaCommand.sectorCount = 0;
        commandParams.commandParams.NoneUdmaCommand.lbaLow = 0;
        commandParams.commandParams.NoneUdmaCommand.lbaMid = 0;
        commandParams.commandParams.NoneUdmaCommand.lbaHigh = 0;
    }
    else
    {
        ialExt->IALChannelExt[channelIndex].pmAccessType =
        MV_ATA_COMMAND_PM_WRITE_REG;
        commandParams.commandParams.NoneUdmaCommand.command =
        MV_ATA_COMMAND_PM_WRITE_REG;
        commandParams.commandParams.NoneUdmaCommand.commandId =
        (MV_VOID_PTR)ialExt;
        commandParams.commandParams.NoneUdmaCommand.sectorCount =
        (MV_U16)((Value) & 0xff),
        commandParams.commandParams.NoneUdmaCommand.lbaLow =
        (MV_U16)(((Value) & 0xff00) >> 8);
        commandParams.commandParams.NoneUdmaCommand.lbaMid =
        (MV_U16)(((Value) & 0xff0000) >> 16);
        commandParams.commandParams.NoneUdmaCommand.lbaHigh =
        (MV_U16)(((Value) & 0xff000000) >> 24);
    }
    ialExt->IALChannelExt[channelIndex].pmRegAccessInProgress = MV_TRUE;
    result = mvSataQueueCommand(pSataAdapter,
                                channelIndex,
                                &commandParams);
    if (result != MV_QUEUE_COMMAND_RESULT_OK)
    {
        switch (result)
        {
        case MV_QUEUE_COMMAND_RESULT_BAD_LBA_ADDRESS:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, ": Queue PM command failed. Bad LBA "
                     "\n");
            break;
        case MV_QUEUE_COMMAND_RESULT_QUEUED_MODE_DISABLED:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, ": Queue PM command failed. EDMA"
                     " disabled adapter %d channel %d\n",
                     pSataAdapter->adapterId, channelIndex);
            break;
        case MV_QUEUE_COMMAND_RESULT_FULL:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, ": Queue PM command failed. Queue is"
                     " Full adapter %d channel %d\n",
                     pSataAdapter->adapterId, channelIndex);

            break;
        case MV_QUEUE_COMMAND_RESULT_BAD_PARAMS:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, ": Queue PM command failed. (Bad "
                     "Params), pMvSataAdapter: %p, pSataChannel: %p.\n",
                     pSataAdapter, pSataAdapter->sataChannel[channelIndex]);
            break;
        default:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, ": Queue PM command bad result value (%d) "
                     "from queue command\n",
                     result);
        }
        ialExt->IALChannelExt[channelIndex].pmRegAccessInProgress = MV_FALSE;
        return MV_FALSE;
    }
    return MV_TRUE;
}


static MV_BOOLEAN mvPMEnableCommStatusChangeBits(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                 MV_U8 channelIndex,
                                                 MV_BOOLEAN enable)
{
    MV_U32 regVal;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;

    if (enable == MV_TRUE)
    {
        regVal = MV_BIT16 | MV_BIT26;
    }
    else
    {
        regVal = 0;
    }

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: Set PM "
             "GSCR[33] register to 0x%X\n",
             pSataAdapter->adapterId, channelIndex, regVal);
    /*Set N & X bits reflection in PM GSCR*/
    if (mvPMDevWriteReg(pSataAdapter, channelIndex,
                        MV_SATA_PM_CONTROL_PORT,
                        MV_SATA_GSCR_ERROR_ENABLE_REG_NUM,
                        regVal, NULL) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Failed to set "
                 "PortMultiplier Features Enable register\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    return MV_TRUE;
}

static MV_BOOLEAN mvPMEnableAsyncNotify(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_U8 channelIndex)
{
    MV_U32 regVal1, regVal2;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    /*Features register*/
    if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                       MV_SATA_GSCR_FEATURES_REG_NUM,
                       &regVal1, NULL) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Failed to get Port Multiplier Features"
                 " supported register\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: Port Multiplier features supported: 0x%X\n",
             pSataAdapter->adapterId, channelIndex, regVal1);

    /*PM asynchronous notification supported*/
    if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                       MV_SATA_GSCR_FEATURES_ENABLE_REG_NUM,
                       &regVal2 ,NULL) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Failed to get Port Multiplier Features"
                 " register\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;

    }

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: Port Multiplier features enabled "
             "register: 0x%X\n",
             pSataAdapter->adapterId, channelIndex, regVal2);
    if (regVal1 & MV_BIT3)
    {
        regVal2 |= MV_BIT3;
        if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                            MV_SATA_GSCR_FEATURES_ENABLE_REG_NUM,
                            regVal2 ,NULL) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Failed to set "
                     "PortMultiplier Features Enable register\n",
                     pSataAdapter->adapterId, channelIndex);
            return MV_FALSE;

        }
        ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled = MV_TRUE;
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: PM asynchronous notification is "
                 "enabled.\n", pSataAdapter->adapterId, channelIndex);
    }
    else
    {
        regVal2 &= ~MV_BIT3;
        if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                            MV_SATA_GSCR_FEATURES_ENABLE_REG_NUM,
                            regVal2 ,NULL) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Failed to set "
                     "PortMultiplier Features Enable register\n",
                     pSataAdapter->adapterId, channelIndex);
            return MV_FALSE;

        }
        ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled = MV_FALSE;
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: PM asynchronous notification is "
                 "disabled.\n", pSataAdapter->adapterId, channelIndex);
    }
    return MV_TRUE;
}




static MV_BOOLEAN mvConfigurePMDevice(
                                     MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                     MV_U8 channelIndex)
{
    MV_SATA_PM_DEVICE_INFO PMInfo;
    ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled = MV_FALSE;

    if (mvGetPMDeviceInfo(ialExt->pSataAdapter, channelIndex, &PMInfo)
        == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: Failed to get PortMultiplier Info\n",
                 ialExt->pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: PM of %d ports found\n",
             ialExt->pSataAdapter->adapterId, channelIndex,
             PMInfo.numberOfPorts);
    ialExt->IALChannelExt[channelIndex].PMnumberOfPorts = PMInfo.numberOfPorts;
#ifdef DISABLE_PM_SCC
    if (PMInfo.vendorId == 0x11AB)
    {

        if (mvPMDisableSSC(ialExt->pSataAdapter, channelIndex) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: cannot disable SSC for PM.\n"
                     "unknown vendor.\n",
                     ialExt->pSataAdapter->adapterId, channelIndex);
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: cannot disable SSC for PM - "
                 "unknown vendor.\n",
                 ialExt->pSataAdapter->adapterId, channelIndex);
    }
#endif
    if (mvPMEnableCommStatusChangeBits(ialExt,
                                       channelIndex,
                                       MV_FALSE) != MV_TRUE)
    {
        return MV_FALSE;
    }
    if (mvPMEnableAsyncNotify(ialExt, channelIndex) == MV_FALSE)
    {
        return MV_FALSE;
    }
    return MV_TRUE;
}

static MV_BOOLEAN mvPMEstablishSataComm(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_U8 channelIndex)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_IAL_COMMON_CHANNEL_EXTENSION* channelExt =
    &ialExt->IALChannelExt[channelIndex];
    if (mvConfigurePMDevice(ialExt, channelIndex) == MV_FALSE)
    {
        return MV_FALSE;
    }

    if (mvPMDevEnableStaggeredSpinUpAll(pSataAdapter,
                                        channelIndex,
                                        channelExt->PMnumberOfPorts,
                                        &channelExt->PMdevsToInit) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: PM Enable Staggered Spin-Up Failed\n",
                 pSataAdapter->adapterId, channelIndex);
        channelExt->PMdevsToInit = 0;
        return MV_FALSE;
    }
    return MV_TRUE;
}



static MV_BOOLEAN mvPMCheckForConnectedDevices(
                                              MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                              MV_U8 channelIndex,
                                              MV_BOOLEAN* bIsChannelReady)
{
    MV_U8 PMPort;
    MV_U32 SStatus;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_IAL_COMMON_CHANNEL_EXTENSION* channelExt =
    &ialExt->IALChannelExt[channelIndex];
    *bIsChannelReady = MV_FALSE;
    channelExt->devInSRST = MV_SATA_PM_CONTROL_PORT+1;

    for (PMPort = 0; PMPort < channelExt->PMnumberOfPorts; PMPort++)
    {
        if (!(channelExt->PMdevsToInit & (1 << PMPort)))
        {
            continue;
        }
        channelExt->PMdevsToInit &= ~(1 << PMPort);
        if (mvPMDevReadReg(pSataAdapter, channelIndex, PMPort,
                           MV_SATA_PSCR_SSTATUS_REG_NUM, &SStatus, NULL) ==
            MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: mvPMDevReadReg Failed\n",
                     pSataAdapter->adapterId, channelIndex, PMPort);
            if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                               MV_SATA_PM_CONTROL_PORT, NULL) == MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]:failed to Soft Reset PM control "
                         "port\n",
                         pSataAdapter->adapterId, channelIndex);
                return MV_FALSE;
            }
            continue;
        }
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: S-Status: 0x%x\n",
                 pSataAdapter->adapterId,
                 channelIndex, PMPort, SStatus);

        if ((SStatus & 0xf) != 3)
        {
            if (mvPMDevWriteReg(pSataAdapter, channelIndex, PMPort,
                                MV_SATA_PSCR_SERROR_REG_NUM, 0xffffffff, NULL) ==
                MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,
                         "[%d %d %d]: PM Write SERROR Failed\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
            }
            continue;
        }

        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Start reset PM connected device.\n",
                 pSataAdapter->adapterId, channelIndex, PMPort);

        /*Set first connected PM port for further SRST*/
        channelExt->PMdevsToInit |= (1 << PMPort);
        if (channelExt->devInSRST == MV_SATA_PM_CONTROL_PORT+1)
        {
            channelExt->devInSRST = PMPort;
        }
    }

    /*PM is connected but no device is present*/
    if (!channelExt->PMdevsToInit)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: "
                 "No more devices connected to PM.\n",
                 pSataAdapter->adapterId, channelIndex);
        if (mvPMEnableCommStatusChangeBits(ialExt,
                                           channelIndex,
                                           MV_TRUE) != MV_TRUE)
        {
            return MV_FALSE;
        }

        *bIsChannelReady = MV_TRUE;
        return MV_TRUE;
    }


    for (PMPort = channelExt->devInSRST;
        PMPort < channelExt->PMnumberOfPorts;
        PMPort++)
    {
        if (channelExt->PMdevsToInit & (1 << PMPort))
        {
            channelExt->devInSRST = PMPort;
            /*Clear SError on PM port*/
            if (mvPMDevWriteReg(pSataAdapter, channelIndex, PMPort,
                                MV_SATA_PSCR_SERROR_REG_NUM, 0xFFFFFFFF, NULL) ==
                MV_FALSE)
            {
                if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                                   MV_SATA_PM_CONTROL_PORT, NULL) == MV_FALSE)
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: "
                             "failed to Soft Reset PM control port\n",
                             pSataAdapter->adapterId, channelIndex);
                    return MV_FALSE;
                }
            }
            if (mvStorageDevATAStartSoftResetDevice(pSataAdapter,
                                                    channelIndex,
                                                    channelExt->devInSRST)
                == MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: "
                         "failed to Soft Reset PM device port.\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                channelExt->PMdevsToInit &= ~(1 << PMPort);
            }
            else
            {
                break;
            }
        }
    }
    if (PMPort == channelExt->PMnumberOfPorts)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: "
                 "SRST failed for all PM connected devices.\n",
                 pSataAdapter->adapterId, channelIndex);
        if (mvPMEnableCommStatusChangeBits(ialExt,
                                           channelIndex,
                                           MV_TRUE) != MV_TRUE)
        {
            return MV_FALSE;
        }
        *bIsChannelReady = MV_TRUE;
    }
    return MV_TRUE;
}

static MV_BOOLEAN hasSataLinkStatusChanged(MV_SATA_ADAPTER *pSataAdapter,
                                           MV_U8 channelIndex,
                                           MV_U8 PMPort,
                                           MV_U32 *connectStatus)
{
    MV_U32 regVal;
    /*No change in sttatus*/
    *connectStatus = 0;
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: "
             "Before sending PIO to drive: "
             "looking for any change in port status.\n",
             pSataAdapter->adapterId,
             channelIndex,
             PMPort);

    if (mvPMDevReadReg(pSataAdapter, channelIndex, PMPort,
                       MV_SATA_PSCR_SERROR_REG_NUM, &regVal, NULL)
        == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                 "mvPMDevReadReg SCR-ERROR Failed",
                 pSataAdapter->adapterId, channelIndex, PMPort);
        if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                           MV_SATA_PM_CONTROL_PORT,
                                           NULL) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: failed to Soft Reset PM "
                     "control port\n",
                     pSataAdapter->adapterId, channelIndex);

        }
        return MV_FALSE;
    }
    /*clear X bit and N bitin S-Error*/
    if ((regVal != 0) && mvPMDevWriteReg(pSataAdapter, channelIndex, PMPort,
                                         MV_SATA_PSCR_SERROR_REG_NUM, 0xFFFFFFFF, NULL)
        == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                 "mvStorageDevPMWriteReg SCR-ERROR Failed",
                 pSataAdapter->adapterId, channelIndex, PMPort);
        if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                           MV_SATA_PM_CONTROL_PORT,
                                           NULL) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: failed to Soft Reset PM "
                     "control port\n",
                     pSataAdapter->adapterId, channelIndex);
        }
        return MV_FALSE;
    }

    if (!(regVal & MV_BIT16))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: "
                 "N Bit is zero in SError register - no change in status "
                 "of PM ports.\n",
                 pSataAdapter->adapterId,
                 channelIndex,
                 PMPort);

    }
    else
    {
        /*Disconnected*/
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                 "Unexpected device disconnect.\n",
                 pSataAdapter->adapterId,
                 channelIndex,
                 PMPort);
        *connectStatus = 1;
    }

    if (!(regVal & MV_BIT26))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: X Bit is zero in SError register\n",
                 pSataAdapter->adapterId,
                 channelIndex,
                 PMPort);
    }
    else
    {
        /*Reconnected*/
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                 "Device reconnected while before SRST status was probed\n",
                 pSataAdapter->adapterId,
                 channelIndex,
                 PMPort);
        *connectStatus = 2;
    }

    if (*connectStatus < 2)
    {
        return MV_TRUE;
    }

    if (mvPMDevReadReg(pSataAdapter, channelIndex, PMPort,
                       MV_SATA_PSCR_SSTATUS_REG_NUM, &regVal, NULL) ==
        MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: mvPMDevReadReg SStatus "
                 "Failed\n",
                 pSataAdapter->adapterId, channelIndex, PMPort);
        if (mvStorageDevATASoftResetDevice(pSataAdapter,
                                           channelIndex,
                                           MV_SATA_PM_CONTROL_PORT,
                                           NULL) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: failed to Soft Reset PM "
                     "control port\n",
                     pSataAdapter->adapterId, channelIndex);

        }
        return MV_FALSE;
    }
    if ((regVal & 0xf) != 0x3)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: No reconnected device found\n",
                 pSataAdapter->adapterId, channelIndex, PMPort);
        *connectStatus = 1;
    }
    return MV_TRUE;
}

static MV_BOOLEAN mvInitPMDevice(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                 MV_U8 channelIndex,
                                 MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                                 MV_BOOLEAN* bIsChannelReady,
                                 MV_BOOLEAN* isDeviceReconnected,
                                 MV_BOOLEAN* bPMErrorDetected)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_U8 PMPort = ialExt->IALChannelExt[channelIndex].devInSRST;
    MV_SATA_SCSI_DRIVE_DATA *pDriveData;
    MV_STORAGE_DEVICE_REGISTERS mvStorageDevRegisters;
    MV_U32 deviceConnectionStatus = 0;
    MV_IAL_COMMON_CHANNEL_EXTENSION *channelExt =
    &ialExt->IALChannelExt[channelIndex];
    *bIsChannelReady = MV_FALSE;
    *isDeviceReconnected = MV_FALSE;
    *bPMErrorDetected = MV_FALSE;
    if (PMPort == MV_SATA_PM_CONTROL_PORT+1)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Buggy call fo InitPMDevice()\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    PMPort = channelExt->devInSRST;

    if (mvStorageIsDeviceBsyBitOff(pSataAdapter,
                                   channelIndex,
                                   &mvStorageDevRegisters)
        == MV_FALSE)
    {
        if (mvIsChannelTimerExpired(ialExt, channelIndex) !=
            MV_TRUE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d] - disk not ready: wait a "
                     "little more\n",
                     pSataAdapter->adapterId,
                     channelIndex, PMPort);
            printAtaDeviceRegisters(&mvStorageDevRegisters);
            return MV_FALSE;
        }
        /*Skip current PM port*/
        channelExt->PMdevsToInit &= ~(1 << PMPort);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: failed to Soft Reset PM port\n",
                 pSataAdapter->adapterId, channelIndex, PMPort);
        /*Try do do synchronous software reset tp PM*/
        mvStorageDevATASoftResetDevice(pSataAdapter,
                                       channelIndex,
                                       MV_SATA_PM_CONTROL_PORT,
                                       NULL);
    }
    else
    {
        printAtaDeviceRegisters(&mvStorageDevRegisters);
        channelExt->PMdevsToInit &= ~(1 << PMPort);
        if (hasSataLinkStatusChanged(pSataAdapter,
                                     channelIndex,
                                     PMPort,
                                     &deviceConnectionStatus) == MV_TRUE)
        {
            switch (deviceConnectionStatus)
            {
            case 1:
                /*Skip current PM port*/
                channelExt->PMdevsToInit &= ~(1 << PMPort);
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                         "PM port unexpected disconnection detected\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                break;
            case 2:
                *isDeviceReconnected = MV_TRUE;
                return MV_TRUE;
                break;
            }
        }
        else
        {
            *bPMErrorDetected = MV_TRUE;
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: failed to check SATA link "
                     "status\n",
                     pSataAdapter->adapterId, channelIndex, PMPort);
            return MV_FALSE;
        }
        if (deviceConnectionStatus == 0)
        {
            channelExt->PMdevsToInit &= ~(1 << PMPort);
            pDriveData = &scsiAdapterExt->ataDriveData[channelIndex][PMPort];
            if (mvInitSataDisk(pSataAdapter,
                               channelIndex ,
                               PMPort, &pDriveData->identifyInfo,
                               pDriveData->identifyBuffer) == MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: Failed to initialize disk\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                mvDrivesInfoFlushSingleDrive(ialExt, channelIndex, PMPort);
            }
            else
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Disk ready\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                mvSetDriveReady(ialExt,
                                scsiAdapterExt,
                                channelIndex, PMPort,
                                MV_TRUE,
                                pDriveData->identifyBuffer);
                //mvSataScsiNotifyUA(scsiAdapterExt, channelIndex, PMPort);
            }
        }
    }
    if (!channelExt->PMdevsToInit)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: "
                 "No more devices connected to PM.\n",
                 pSataAdapter->adapterId, channelIndex);
        if (mvPMEnableCommStatusChangeBits(ialExt,
                                           channelIndex,
                                           MV_TRUE) != MV_TRUE)
        {
            return MV_FALSE;
        }

        *bIsChannelReady = MV_TRUE;
        return MV_TRUE;
    }
    for (PMPort = channelExt->devInSRST;
        PMPort < channelExt->PMnumberOfPorts;
        PMPort++)
    {
        if (channelExt->PMdevsToInit & (1 << PMPort))
        {
            channelExt->devInSRST = PMPort;
            /*Clear SError on PM port*/
            if (mvPMDevWriteReg(pSataAdapter, channelIndex, PMPort,
                                MV_SATA_PSCR_SERROR_REG_NUM, 0xffffffff, NULL) ==
                MV_FALSE)
            {
                *bPMErrorDetected = MV_TRUE;
                if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                                   MV_SATA_PM_CONTROL_PORT, NULL) == MV_FALSE)
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: failed to Soft Reset PM control"
                             " port\n", pSataAdapter->adapterId, channelIndex);
                }
                return MV_FALSE;
            }

            if (mvStorageDevATAStartSoftResetDevice(pSataAdapter,
                                                    channelIndex,
                                                    PMPort)
                == MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: failed to Soft Reset "
                         "PM port\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                channelExt->PMdevsToInit &= ~(1 << PMPort);
                continue;
            }
            else
            {
                break;
            }
        }
    }
    if (PMPort == channelExt->PMnumberOfPorts)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: "
                 "SRST failed for some PM connected devices.\n",
                 pSataAdapter->adapterId, channelIndex);
        if (mvPMEnableCommStatusChangeBits(ialExt,
                                           channelIndex,
                                           MV_TRUE) != MV_TRUE)
        {
            return MV_FALSE;
        }
        *bIsChannelReady = MV_TRUE;
    }
    return MV_TRUE;
}


static void mvCheckPMForError(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_U8 channelIndex)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    if (ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled == MV_TRUE)
    {
        ialExt->IALChannelExt[channelIndex].completionError = MV_FALSE;
        return;
    }
    if (pSataAdapter->sataChannel[channelIndex] == NULL)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: "
                 "Invalid channel data structure pointer.\n",
                 pSataAdapter->adapterId, channelIndex);
        return;
    }
    if (ialExt->IALChannelExt[channelIndex].completionError == MV_TRUE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: "
                 "Set completion error to MV_FALSE.\n",
                 pSataAdapter->adapterId, channelIndex);
        ialExt->IALChannelExt[channelIndex].completionError = MV_FALSE;

        if (mvStorageDevGetDeviceType(pSataAdapter,channelIndex)
            == MV_SATA_DEVICE_TYPE_PM)
        {
            MV_U32 value;

            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: "
                     "Probe PM GSCR[32] register to detect possible device"
                     " hot plug.\n",
                     pSataAdapter->adapterId, channelIndex);

            mvSataDisableChannelDma(pSataAdapter, channelIndex);
            if (mvPMDevReadReg(pSataAdapter, channelIndex,
                               MV_SATA_PM_CONTROL_PORT,
                               MV_SATA_GSCR_ERROR_REG_NUM,
                               &value ,NULL) == MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: "
                         "Failed to read GSCR register value.\n",
                         pSataAdapter->adapterId, channelIndex);
                mvSataEnableChannelDma(pSataAdapter, channelIndex);
                return;
                /*Do nothing*/
            }
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: "
                     "PM error information GSCR[32] register value = 0x%X\n",
                     pSataAdapter->adapterId, channelIndex, value);

            if (value != 0)
            {
                ialExt->IALChannelExt[channelIndex].PMdevsToInit =
                (MV_U16)(value & 0x7FFF);
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: PM Hot plug detected "
                         "Bitmask = 0x%X\n",
                         pSataAdapter->adapterId, channelIndex,
                         ialExt->IALChannelExt[channelIndex].PMdevsToInit);
                mvSetChannelState(ialExt, channelIndex,
                                  CHANNEL_PM_HOT_PLUG);
            }
            else
            {
                mvSataEnableChannelDma(pSataAdapter, channelIndex);
            }
        }
    }
}

MV_BOOLEAN mvPMEnableLocking(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex)
{
    MV_STORAGE_DEVICE_REGISTERS regs;
    if (mvPMDevWriteReg(pSataAdapter, channelIndex,
                        MV_SATA_PM_CONTROL_PORT,
                        0x89,
                        0x8000003F,
                        &regs) == MV_TRUE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: NCQ lock is enabled for PM.\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_TRUE;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: cannot enable NCQ lock for PM.\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    return MV_TRUE;
}

#ifdef DISABLE_PM_SCC
MV_BOOLEAN mvPMDisableSSC(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex)
{
    MV_STORAGE_DEVICE_REGISTERS regs;
    MV_U32 regVal = 0;

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: "
             "Disable SSC for all PM ports.\n",
             pSataAdapter->adapterId, channelIndex);

    if (mvPMDevReadReg(pSataAdapter,channelIndex, MV_SATA_PM_CONTROL_PORT,
                       MV_SATA_GSCR_FEATURES_ENABLE_REG_NUM,
                       &regVal,
                       &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }

    /*Host SSC disable*/
    regVal &= ~MV_BIT2;
    if (mvPMDevWriteReg(pSataAdapter,channelIndex, MV_SATA_PM_CONTROL_PORT,
                        MV_SATA_GSCR_FEATURES_ENABLE_REG_NUM,
                        regVal,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }

    /* disable ssc for port 0*/
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x8C,
                        0,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x92,
                        0xb02a402a,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    /* disable ssc for port 1*/
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x8C,
                        1,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x92,
                        0xb02a402a,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    /* disable ssc for port 2*/
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x8C,
                        2,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x92,
                        0xb02a402a,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }

    /* disable ssc for port 3*/
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x8C,
                        3,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x92,
                        0xb02a402a,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }

    /* disable ssc for port 15*/
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x8C,
                        MV_SATA_PM_CONTROL_PORT,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    if (mvPMDevWriteReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                        0x92,
                        0xb02a402a,
                        &regs) != MV_TRUE)
    {
        return MV_FALSE;
    }
    return MV_TRUE;
}
#endif

static MV_BOOLEAN mvConfigChannelQueuingMode(
                                            MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt,
                                            MV_U8 channelIndex,
                                            MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_EDMA_MODE mode = MV_EDMA_MODE_NOT_QUEUED;
    MV_BOOLEAN isTCQSupported = MV_FALSE;
    MV_BOOLEAN isNCQSupported = MV_FALSE;
    MV_U8    queueDepth = 0;
    MV_U8    numOfDrives = 0;

    if (mvGetEDMAAllowedModes(ialExt,
                              scsiAdapterExt,
                              channelIndex,
                              &isTCQSupported,
                              &isNCQSupported,
                              &queueDepth,
                              &numOfDrives) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: There is no queing mode supported.\n"
                 ,pSataAdapter->adapterId, channelIndex);
        isTCQSupported = MV_FALSE;
        isNCQSupported = MV_FALSE;
        queueDepth = 0;
    }
    else
    {

        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: Supported queing mode: TCQ = %s, "
                 "NCQ = %s.\n", pSataAdapter->adapterId, channelIndex,
                 (isTCQSupported == MV_TRUE) ? "Yes" : "No",
                 (isNCQSupported == MV_TRUE) ? "Yes" : "No");
    }
    if (isTCQSupported == MV_TRUE)
    {
        mode = MV_EDMA_MODE_QUEUED;
    }
    if (isNCQSupported == MV_TRUE)
    {
        if ((pSataAdapter->sataChannel[channelIndex] != NULL) &&
            (pSataAdapter->sataChannel[channelIndex]->deviceType ==
             MV_SATA_DEVICE_TYPE_PM))
        {

            MV_SATA_PM_DEVICE_INFO PMDeviceInfo;
            if (mvGetPMDeviceInfo(pSataAdapter,
                                  channelIndex,
                                  &PMDeviceInfo) == MV_TRUE)
            {
                if (PMDeviceInfo.vendorId == 0x11AB)
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: Marvell PM found, "
                             "NCQ lock is supported.\n",
                             pSataAdapter->adapterId, channelIndex);
                }
                else
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: NCQ lock not supported by current "
                             "PM. Vendor Id = 0x%X\n",
                             pSataAdapter->adapterId,
                             channelIndex,
                             PMDeviceInfo.vendorId);
                    if (numOfDrives > 1)
                    {
                        isNCQSupported = MV_FALSE;
                    }
                }
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: Enable NCQ locking for PM.\n",
                         pSataAdapter->adapterId, channelIndex);
                if (mvPMEnableLocking(pSataAdapter, channelIndex) == MV_FALSE)
                {
                    isNCQSupported = MV_FALSE;
                }
            }
            else
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: unable to get PM info.\n",
                         pSataAdapter->adapterId, channelIndex);
                isNCQSupported = MV_FALSE;
            }

        }
        if (isNCQSupported == MV_TRUE)
        {
            mode = MV_EDMA_MODE_NATIVE_QUEUING;
        }
        else
        {
            mode = MV_EDMA_MODE_NOT_QUEUED;
            queueDepth = 0;
        }
    }

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: EDMA mode %d, queue depth = %d.\n",
             pSataAdapter->adapterId, channelIndex, mode, queueDepth);
    IALConfigQueuingMode(pSataAdapter,
                         channelIndex,
                         mode,
                         queueDepth);
    return MV_TRUE;
}



/*Channel related functions*/

static void mvSetChannelState(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_U8 channelIndex,
                              MV_CHANNEL_STATE state)
{
    if (ialExt->channelState[channelIndex] != state)
    {
        if ((state == CHANNEL_READY) || (state == CHANNEL_NOT_CONNECTED))
        {
            ialExt->IALChannelExt[channelIndex].SRSTTimerThreshold = 0;
            ialExt->IALChannelExt[channelIndex].SRSTTimerValue = 0;
        }
        if (state == CHANNEL_READY)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: CHANNEL_READY\n",
                     ialExt->pSataAdapter->adapterId,
                     channelIndex);
            ialExt->IALChannelExt[channelIndex].pmRegAccessInProgress
            = MV_FALSE;
            ialExt->IALChannelExt[channelIndex].completionError = MV_FALSE;
            ialExt->channelState[channelIndex] = state;
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] flush pending queue\n",
                     ialExt->pSataAdapter->adapterId, channelIndex);
            /*Abort all pending commands in SW queue*/
            mvFlushSCSICommandQueue(ialExt, channelIndex);
            if (MV_TRUE == ialExt->IALChannelExt[channelIndex].bHotPlug)
            {
                MV_U16 drivesToRemove;
                MV_U16 drivesToAdd;
                ialExt->IALChannelExt[channelIndex].bHotPlug = MV_FALSE;
                mvDrivesInfoGetChannelRescanParams(ialExt,
                                                   channelIndex,
                                                   &drivesToRemove,
                                                   &drivesToAdd);
                if (drivesToRemove != 0 || drivesToAdd != 0)
                {

                    IALBusChangeNotifyEx(ialExt->pSataAdapter,
                                        channelIndex,
                                        drivesToRemove,
                                        drivesToAdd);
                }
            }
            mvDrivesInfoSaveAll(ialExt, channelIndex);
        }
        else
        {
            ialExt->channelState[channelIndex] = state;
        }
    }
}


static MV_BOOLEAN mvStartChannelInit(MV_SATA_ADAPTER *pSataAdapter,
                                     MV_U8 channelIndex,
                                     MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                                     MV_BOOLEAN* bIsChannelReady)
{
    *bIsChannelReady = MV_FALSE;

    if (mvSataConfigureChannel(pSataAdapter, channelIndex) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: configure channel failed\n",
                 pSataAdapter->adapterId,
                 channelIndex);
        return MV_FALSE;
    }

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: start channel\n",
             pSataAdapter->adapterId,
             channelIndex);
    /*Just check SStatus in case of SATA I adapter*/
    if (pSataAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: starting SATA I channel.\n",
                 pSataAdapter->adapterId, channelIndex);
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: starting SATA II channel.\n",
                 pSataAdapter->adapterId, channelIndex);
    }

    return mvStorageDevATAStartSoftResetDevice(pSataAdapter,
                                               channelIndex,
                                               MV_SATA_PM_CONTROL_PORT);
}

static MV_BOOLEAN mvChannelSRSTFinished(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_SATA_CHANNEL *pSataChannel,
                                        MV_U8 channelIndex,
                                        MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                                        MV_BOOLEAN* bIsChannelReady,
                                        MV_BOOLEAN* bFatalError)
{
    MV_SATA_DEVICE_TYPE deviceType;
    MV_STORAGE_DEVICE_REGISTERS mvStorageDevRegisters;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_SATA_SCSI_DRIVE_DATA *pDriveData;
    *bIsChannelReady = MV_FALSE;
    *bFatalError = MV_FALSE;
    if (pSataAdapter->sataAdapterGeneration != MV_SATA_GEN_I)
    {
        if (mvStorageIsDeviceBsyBitOff(pSataAdapter,
                                       channelIndex,
                                       &mvStorageDevRegisters)
            == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: soft Reset PM control port "
                     "in progress\n",
                     pSataAdapter->adapterId, channelIndex);
            printAtaDeviceRegisters(&mvStorageDevRegisters);
            return MV_FALSE;
        }
        deviceType = mvGetSataDeviceType(&mvStorageDevRegisters);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: soft reset SATA II channel - "
                 "device ready.\n",
                 pSataAdapter->adapterId, channelIndex);
    }
    else
    {
        if (mvStorageIsDeviceBsyBitOff(pSataAdapter,
                                       channelIndex,
                                       &mvStorageDevRegisters)
            == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: soft reset of SATA I channel "
                     "in progress\n",
                     pSataAdapter->adapterId, channelIndex);
            printAtaDeviceRegisters(&mvStorageDevRegisters);
            return MV_FALSE;
        }
        deviceType = mvGetSataDeviceType(&mvStorageDevRegisters);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: soft reset SATA I channel - "
                 "device ready.\n",
                 pSataAdapter->adapterId, channelIndex);
        deviceType = mvGetSataDeviceType(&mvStorageDevRegisters);
        if (deviceType != MV_SATA_DEVICE_TYPE_ATA_DISK)
        {
            deviceType = MV_SATA_DEVICE_TYPE_UNKNOWN;
        }

    }
    switch (deviceType)
    {
    case MV_SATA_DEVICE_TYPE_ATA_DISK:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: ATA disk found\n",
                 pSataAdapter->adapterId, channelIndex);
        if (mvStorageDevSetDeviceType(pSataAdapter,channelIndex, deviceType) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Failed to initialize disk\n",
                     pSataAdapter->adapterId, channelIndex);
            *bFatalError = MV_TRUE;
            return MV_FALSE;

        }
        pDriveData = &scsiAdapterExt->ataDriveData[channelIndex][0];
        if (mvInitSataDisk(pSataAdapter,
                           channelIndex,
                           0,
                           &pDriveData->identifyInfo,
                           pDriveData->identifyBuffer) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Failed to initialize disk\n",
                     pSataAdapter->adapterId, channelIndex);
            *bFatalError = MV_TRUE;
            mvDrivesInfoFlushSingleDrive(ialExt,
                                             channelIndex,
                                             0);
            return MV_FALSE;
        }
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: Disk ready\n",
                 pSataAdapter->adapterId, channelIndex);
        mvSetDriveReady(ialExt,
                        scsiAdapterExt,
                        channelIndex,
                        0,
                        MV_TRUE,
                        pDriveData->identifyBuffer);
        //mvSataScsiNotifyUA(scsiAdapterExt, channelIndex, 0);
        *bIsChannelReady = MV_TRUE;
        return MV_TRUE;
        break;
    case MV_SATA_DEVICE_TYPE_PM:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: PortMultiplier device found\n",
                 pSataAdapter->adapterId, channelIndex);
        if (mvStorageDevSetDeviceType(pSataAdapter,channelIndex, deviceType) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Failed to initialize PM\n",
                     pSataAdapter->adapterId, channelIndex);
            *bFatalError = MV_TRUE;
            return MV_FALSE;

        }
        break;
    case MV_SATA_DEVICE_TYPE_ATAPI_DISK:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: ERROR: ATAPI device was found!!! "
                 "(not supported)\n", pSataAdapter->adapterId,
                 channelIndex);
        *bFatalError = MV_TRUE;
        return MV_FALSE;
        break;
    default:
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: ERROR: unknown device type\n",
                 pSataAdapter->adapterId, channelIndex);
        *bFatalError    =    MV_TRUE;
        return MV_FALSE;
    }
    return MV_TRUE;
}



static MV_BOOLEAN mvConfigChannelDMA(
                                    MV_IAL_COMMON_ADAPTER_EXTENSION* ialExt,
                                    MV_U8 channelIndex,
                                    MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] config queueing mode\n",
             pSataAdapter->adapterId, channelIndex);


    if (mvConfigChannelQueuingMode(ialExt,
                                   channelIndex,
                                   scsiAdapterExt)
        == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] Failed to config DMA queuing\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    /* Enable EDMA */
    if (mvSataEnableChannelDma(pSataAdapter, channelIndex) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d] Failed to enable DMA, channel=%d\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d]: channel started successfully\n",
             pSataAdapter->adapterId, channelIndex);
    return MV_TRUE;
}





static void mvSetChannelTimer(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                              MV_U8 channelIndex,
                              MV_U32 timeout)
{
    ialExt->IALChannelExt[channelIndex].SRSTTimerThreshold = timeout;
    ialExt->IALChannelExt[channelIndex].SRSTTimerValue = 1;
}

static void mvDecrementChannelTimer(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                    MV_U8 channelIndex)
{
    if (ialExt->IALChannelExt[channelIndex].SRSTTimerThreshold > 0)
    {
        ialExt->IALChannelExt[channelIndex].SRSTTimerValue +=
        MV_IAL_ASYNC_TIMER_PERIOD;
    }
}

static MV_BOOLEAN mvIsChannelTimerExpired(
                                         MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                         MV_U8 channelIndex)
{
    if (ialExt->IALChannelExt[channelIndex].SRSTTimerValue >
        ialExt->IALChannelExt[channelIndex].SRSTTimerThreshold)
    {
        return MV_TRUE;
    }
    else
    {
        return MV_FALSE;
    }
}

/*******************************************************************************
*State Machine related functions:
*  Return MV_TRUE to proceed to the next channel
*  Return MV_FALSE to proceed to the next state on current channel
*******************************************************************************/

static MV_BOOLEAN mvChannelNotConnectedStateHandler(
                                                   MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                   MV_U8 channelIndex,
                                                   MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    if (pSataAdapter->sataChannel[channelIndex] != NULL)
    {
        mvSataRemoveChannel(pSataAdapter,channelIndex);
        pSataAdapter->sataChannel[channelIndex] = NULL;
        mvSetDriveReady(ialExt,
                        scsiAdapterExt,
                        channelIndex,
                        0xFF, MV_FALSE, NULL);
        mvFlushSCSICommandQueue(ialExt, channelIndex);
    }
    return MV_TRUE;
}


static MV_BOOLEAN mvChannelConnectedStateHandler(
                                                MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                MV_U8 channelIndex,
                                                MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_BOOLEAN res = MV_FALSE;
    MV_BOOLEAN isChannelReady;
    MV_SATA_CHANNEL *pSataChannel;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] CHANNEL_CONNECTED\n",
             ialExt->pSataAdapter->adapterId, channelIndex);
    if (pSataAdapter->sataChannel[channelIndex] == NULL)
    {
        if (IALInitChannel(pSataAdapter, channelIndex) == MV_FALSE)
        {
            IALReleaseChannel(pSataAdapter, channelIndex);
            mvDrivesInfoFlushAll(ialExt, channelIndex);
            mvSetChannelState(ialExt, channelIndex, CHANNEL_NOT_CONNECTED);
            return MV_TRUE;
        }
    }
    pSataChannel = pSataAdapter->sataChannel[channelIndex];
    res = mvStartChannelInit(pSataAdapter,
                             channelIndex,
                             scsiAdapterExt,
                             &isChannelReady);
    if (res == MV_TRUE)
    {
        if (isChannelReady == MV_FALSE)
        {
            /*SRST channel, Set polling timer*/
            mvSetChannelTimer(ialExt, channelIndex,
                              MV_IAL_SRST_TIMEOUT);
            mvSetChannelState(ialExt,
                              channelIndex,
                              CHANNEL_IN_SRST);
        }
        else
        {
            if (mvConfigChannelDMA(ialExt,
                                   channelIndex,
                                   scsiAdapterExt) == MV_TRUE)
            {
                mvSetChannelState(ialExt,
                                  channelIndex,
                                  CHANNEL_READY);
            }
            else
            {
                mvStopChannel(ialExt,
                              channelIndex,
                              scsiAdapterExt);
            }
        }
    }
    else
    {
        mvStopChannel(ialExt,
                      channelIndex,
                      scsiAdapterExt);
    }
    return MV_TRUE;
}


MV_BOOLEAN mvChannelInSrstStateHandler(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_U8 channelIndex,
                                       MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_BOOLEAN bFatalError;
    MV_BOOLEAN res = MV_FALSE;
    MV_BOOLEAN isChannelReady;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    MV_SATA_CHANNEL *pSataChannel = pSataAdapter->sataChannel[channelIndex];
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] CHANNEL_IN_SRST\n",
             pSataAdapter->adapterId, channelIndex);
    mvDecrementChannelTimer(ialExt, channelIndex);
    res = mvChannelSRSTFinished(ialExt,
                                pSataChannel,
                                channelIndex,
                                scsiAdapterExt,
                                &isChannelReady,
                                &bFatalError);
    if (res == MV_TRUE)
    {
        /*Finishing channel initialization*/
        if (isChannelReady == MV_TRUE)
        {
            if (mvConfigChannelDMA(ialExt,
                                   channelIndex,
                                   scsiAdapterExt) == MV_TRUE)
            {
                mvSetChannelState(ialExt,
                                  channelIndex,
                                  CHANNEL_READY);
            }
            else
            {
                mvStopChannel(ialExt,
                              channelIndex,
                              scsiAdapterExt);
            }
        }
        else
        {/*If channel not ready and function call succeed -> PM is found*/
            if (mvPMEstablishSataComm(ialExt, channelIndex) == MV_FALSE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Failed to "
                         "staggered spinup PM\n",
                         pSataAdapter->adapterId,channelIndex);
                mvStopChannel(ialExt,
                              channelIndex,
                              scsiAdapterExt);
            }
            else
            {
                mvSetChannelState(ialExt, channelIndex,
                                  CHANNEL_PM_STAGGERED_SPIN_UP);
                return MV_FALSE;
            }
        }
    }
    else
    {
        if (bFatalError == MV_TRUE)
        {
            mvStopChannel(ialExt,
                          channelIndex,
                          scsiAdapterExt);
        }
        else
        {
            if (mvIsChannelTimerExpired(ialExt, channelIndex) == MV_TRUE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,
                         "[%d %d]: SW reset Failed, timer expired\n",
                         pSataAdapter->adapterId,channelIndex);
                mvStopChannel(ialExt,
                              channelIndex,
                              scsiAdapterExt);
            }
        }
    }
    return MV_TRUE;
}




static MV_BOOLEAN mvChannelPMStaggeredSpinUpStateHandler(
                                                        MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                        MV_U8 channelIndex,
                                                        MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_BOOLEAN isChannelReady;
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] CHANNEL_PM_STAGGERED_SPIN_UP\n",
             ialExt->pSataAdapter->adapterId, channelIndex);

    if (mvPMCheckForConnectedDevices(ialExt,
                                     channelIndex,
                                     &isChannelReady) == MV_TRUE)
    {
        if (isChannelReady == MV_TRUE)
        {
            if (mvConfigChannelDMA(ialExt,
                                   channelIndex,
                                   scsiAdapterExt) == MV_TRUE)
            {
                mvSetChannelState(ialExt, channelIndex, CHANNEL_READY);

            }
            else
            {
                mvStopChannel(ialExt, channelIndex, scsiAdapterExt);
            }
        }
        else
        {
            /*SRST first PM port, Set polling timer*/
            mvSetChannelTimer(ialExt, channelIndex, MV_IAL_SRST_TIMEOUT);
            mvSetChannelState(ialExt, channelIndex, CHANNEL_PM_SRST_DEVICE);
        }
    }
    else
    {
        mvStopChannel(ialExt, channelIndex, scsiAdapterExt);
    }
    return MV_TRUE;
}


static MV_BOOLEAN mvChannelPMSrstDeviceStateHandler(
                                                   MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                   MV_U8 channelIndex,
                                                   MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{

    MV_BOOLEAN isChannelReady;
    MV_BOOLEAN isDeviceReconnected;
    MV_BOOLEAN isPMErrorDetected;
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] CHANNEL_PM_SRST_DEVICE\n",
             ialExt->pSataAdapter->adapterId, channelIndex);
    mvDecrementChannelTimer(ialExt, channelIndex);
    if (mvInitPMDevice(ialExt,
                       channelIndex,
                       scsiAdapterExt,
                       &isChannelReady,
                       &isDeviceReconnected,
                       &isPMErrorDetected) == MV_TRUE)
    {
        if (isDeviceReconnected == MV_TRUE)
        {
            mvRestartChannel(ialExt, channelIndex, scsiAdapterExt, MV_FALSE);
            return MV_TRUE;
        }
        if (isChannelReady == MV_TRUE)
        {
            if (mvConfigChannelDMA(ialExt,
                                   channelIndex,
                                   scsiAdapterExt) == MV_TRUE)
            {
                mvSetChannelState(ialExt, channelIndex, CHANNEL_READY);
            }
            else
            {
                mvStopChannel(ialExt, channelIndex, scsiAdapterExt);
            }
        }
        else
        {
            /*PM port is initialized, SRST next PM Port,
            Set polling timer*/
            mvSetChannelTimer(ialExt, channelIndex, MV_IAL_SRST_TIMEOUT);
        }
    }
    else
    {
        /*SRST timeout is expired*/
        if (mvIsChannelTimerExpired(ialExt, channelIndex) == MV_TRUE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] PM device "
                     "SRST timeout detected.\n",
                     ialExt->pSataAdapter->adapterId, channelIndex);
            if (mvConfigChannelDMA(ialExt,
                                   channelIndex,
                                   scsiAdapterExt) == MV_TRUE)
            {
                mvSetChannelState(ialExt, channelIndex, CHANNEL_READY);
            }
            else
            {
                mvStopChannel(ialExt, channelIndex, scsiAdapterExt);
            }
        }
        else
        {
            if (isPMErrorDetected == MV_TRUE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] unable "
                         "to configure PM\n.", ialExt->pSataAdapter->adapterId,
                         channelIndex);
                mvStopChannel(ialExt, channelIndex, scsiAdapterExt);
            }
        }
    }
    return MV_TRUE;
}


static MV_BOOLEAN mvChannelReadyStateHandler(
                                            MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                            MV_U8 channelIndex)
{

    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    if ((ialExt->IALChannelExt[channelIndex].pmRegAccessInProgress == MV_FALSE) &&
        (mvStorageDevGetDeviceType (pSataAdapter, channelIndex) == MV_SATA_DEVICE_TYPE_PM) &&
        (ialExt->IALChannelExt[channelIndex].pmAsyncNotifyEnabled == MV_FALSE))
    {
        if (mvQueuePMAccessRegisterCommand(ialExt,
                                           channelIndex,
                                           MV_SATA_PM_CONTROL_PORT,
                                           MV_SATA_GSCR_ERROR_REG_NUM,
                                           0,
                                           MV_TRUE) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] error reading "
                     " PM GSCR_ERROR register.\n",
                     pSataAdapter->adapterId, channelIndex);
        }
    }
    return MV_TRUE;
}


static MV_BOOLEAN mvChannelPMHotPlugStateHandler(
                                                MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                                MV_U8 channelIndex,
                                                MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d] CHANNEL_PM_HOT_PLUG\n",
             ialExt->pSataAdapter->adapterId, channelIndex);
    mvSataDisableChannelDma(ialExt->pSataAdapter, channelIndex);
    mvSataFlushDmaQueue (ialExt->pSataAdapter,
                         channelIndex, MV_FLUSH_TYPE_CALLBACK);
    mvSataChannelHardReset(ialExt->pSataAdapter, channelIndex);
    mvRestartChannel(ialExt, channelIndex, scsiAdapterExt, MV_FALSE);
    return MV_TRUE;
}

static MV_BOOLEAN mvChannelStateMachine(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_U8 channelIndex,
                                       MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_BOOLEAN res = MV_FALSE;
    do
    {
        switch (ialExt->channelState[channelIndex])
        {
        case CHANNEL_NOT_CONNECTED:
            res = mvChannelNotConnectedStateHandler(ialExt,
                                                    channelIndex,
                                                    scsiAdapterExt);
            break;
        case CHANNEL_CONNECTED:
            res = mvChannelConnectedStateHandler(ialExt,
                                                 channelIndex,
                                                 scsiAdapterExt);
            break;
        case CHANNEL_IN_SRST:
            res = mvChannelInSrstStateHandler(ialExt,
                                              channelIndex,
                                              scsiAdapterExt);
            break;
        case CHANNEL_PM_STAGGERED_SPIN_UP:
            res = mvChannelPMStaggeredSpinUpStateHandler(ialExt,
                                                         channelIndex,
                                                         scsiAdapterExt);
            break;
        case CHANNEL_PM_SRST_DEVICE:
            res = mvChannelPMSrstDeviceStateHandler(ialExt,
                                                    channelIndex,
                                                    scsiAdapterExt);
            break;
        case CHANNEL_READY:
            res = mvChannelReadyStateHandler(ialExt,
                                             channelIndex);
            break;
        case CHANNEL_PM_HOT_PLUG:
            res = mvChannelPMHotPlugStateHandler(ialExt,
                                                 channelIndex,
                                                 scsiAdapterExt);
            break;
        default:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Unknown channel state.\n",
                     ialExt->pSataAdapter->adapterId, channelIndex);
            return MV_FALSE;
        }
    } while (res == MV_FALSE);

    return MV_TRUE;
}


static MV_BOOLEAN mvAdapterStateMachine(
                                       MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                       MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt)
{
    MV_BOOLEAN res = MV_TRUE;
    MV_U8 channelIndex;
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    switch (ialExt->adapterState)
    {
    case ADAPTER_INITIALIZING:     {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d] ADAPTER_INITIALIZING\n",
                     pSataAdapter->adapterId);

            res = mvSataEnableStaggeredSpinUpAll(pSataAdapter);
            if (res == MV_TRUE)
            {
                if (pSataAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
                {
                    if (mvSataUnmaskAdapterInterrupt(pSataAdapter) == MV_FALSE)
                    {
                        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d]: "
                                 "mvSataUnmaskAdapterInterrupt failed\n",
                                 pSataAdapter->adapterId);
                        ialExt->adapterState = ADAPTER_FATAL_ERROR;
                        return MV_FALSE;
                    }
                    ialExt->adapterState = ADAPTER_READY;
                }
                else
                {
                    if (mvSataUnmaskAdapterInterrupt(pSataAdapter) == MV_FALSE)
                    {
                        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d]:"
                                 "mvSataUnmaskAdapterInterrupt failed\n",
                                 pSataAdapter->adapterId);
                        ialExt->adapterState = ADAPTER_FATAL_ERROR;
                        return MV_FALSE;
                    }
                    ialExt->adapterState = ADAPTER_READY;
                }
            }
        }
        break;
    case ADAPTER_READY:
        for (channelIndex = 0;
            channelIndex < pSataAdapter->numberOfChannels; channelIndex++)
        {

            mvChannelStateMachine(ialExt,
                                  channelIndex,
                                  scsiAdapterExt);
        }
        return MV_TRUE;
        break;
    default:
        break;
    }

    if (ialExt->adapterState != ADAPTER_READY)
    {
        return res;
    }

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d] ADAPTER_READY\n",
             pSataAdapter->adapterId);

    /*Start channel initialization for connected channels*/
    for (channelIndex = 0;
        channelIndex < pSataAdapter->numberOfChannels;
        channelIndex++)
    {
        mvFlushSCSICommandQueue(ialExt, channelIndex);
        if (mvSataIsStorageDeviceConnected(pSataAdapter, channelIndex) ==
            MV_FALSE)
        {
            mvSetChannelState(ialExt,
                              channelIndex,
                              CHANNEL_NOT_CONNECTED);
            continue;
        }

        mvSetChannelState(ialExt,
                          channelIndex,
                          CHANNEL_CONNECTED);
        if (mvChannelStateMachine(ialExt,
                                  channelIndex,
                                  scsiAdapterExt) == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]: Failed to "
                     "start channel.\n", pSataAdapter->adapterId, channelIndex);
            mvSetChannelState(ialExt,
                              channelIndex,
                              CHANNEL_NOT_CONNECTED);
            mvFlushSCSICommandQueue(ialExt, channelIndex);
            mvSataRemoveChannel(pSataAdapter,channelIndex);
            IALReleaseChannel(pSataAdapter, channelIndex);
            pSataAdapter->sataChannel[channelIndex] = NULL;
            mvDrivesInfoFlushAll(ialExt, channelIndex);
            mvSetDriveReady(ialExt,
                            scsiAdapterExt,
                            channelIndex,
                            0xFF, MV_FALSE, NULL);
            continue;
        }
    }
    return res;
}


static MV_BOOLEAN mvGetEDMAAllowedModes(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                        MV_SAL_ADAPTER_EXTENSION *pScsiAdapterExt,
                                        MV_U8 channelIndex,
                                        MV_BOOLEAN *TCQ,
                                        MV_BOOLEAN *NCQ,
                                        MV_U8   *queueDepth,
                                        MV_U8   *numOfDrives)
{
    MV_SATA_ADAPTER *pSataAdapter = ialExt->pSataAdapter;
    *numOfDrives = 0;
    if ((pSataAdapter == NULL) || (pScsiAdapterExt == NULL))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d]"
                 " mvGetEMDAAllowedModes failed, bad pointer\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    if ((ialExt->adapterState != ADAPTER_READY) ||
        (ialExt->channelState[channelIndex] == CHANNEL_NOT_CONNECTED))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] "
                 "mvGetEMDAAllowedModes failed,Bad Adapter or Channel State\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    if (pSataAdapter->sataChannel[channelIndex] == NULL)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] "
                 "mvGetEMDAAllowedModes failed, channel not configured\n",
                 pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    if (mvStorageDevGetDeviceType (pSataAdapter, channelIndex) ==
        MV_SATA_DEVICE_TYPE_ATA_DISK)
    {
        *numOfDrives = 1;
    }

    if (TCQ)
    {
        MV_U8  PMPort = 0;
        MV_SATA_DEVICE_TYPE devType =
        mvStorageDevGetDeviceType (pSataAdapter, channelIndex);
        *TCQ = MV_FALSE;
        switch (devType)
        {
        /*allow TCQ mode only if one drive connected*/
        case MV_SATA_DEVICE_TYPE_PM:{
                MV_U8  i;
                MV_U8  numOFDrives = 0;
                for (i = 0; i < MV_SATA_PM_MAX_PORTS; i++)
                {
                    if (pScsiAdapterExt->
                        ataDriveData[channelIndex][i].driveReady == MV_TRUE)
                    {
                        PMPort = i;
                        numOFDrives++;
                    }
                }
                *numOfDrives = numOFDrives;
                if (numOFDrives != 1)
                {
                    break;
                }
            }
            /*the drive connected to PM on device port number PMPort*/
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: "
                     "mvGetEMDAAllowedModes one drive connected - check if TCQ"
                     " allowed\n", pSataAdapter->adapterId, channelIndex,
                     PMPort);
        case MV_SATA_DEVICE_TYPE_ATA_DISK:

            if (pScsiAdapterExt->ataDriveData[channelIndex][PMPort].driveReady
                != MV_TRUE)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d] "
                         "mvGetEMDAAllowedModes failed, channel not ready\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                return MV_FALSE;
            }
            *TCQ =
            pScsiAdapterExt->ataDriveData[channelIndex][PMPort].
            identifyInfo.DMAQueuedModeSupported;
            if (queueDepth == NULL)
            {
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d] "
                         "mvGetEMDAAllowedModes failed, bad pointer to queueDepth\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);
                return MV_FALSE;
            }
            *queueDepth = pScsiAdapterExt->ataDriveData[channelIndex][PMPort].
                          identifyInfo.DMAQueuedModeDepth;
            break;
        default:
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] "
                     "mvGetEMDAAllowedModes failed, bad device type\n",
                     pSataAdapter->adapterId, channelIndex);
            return MV_FALSE;
        }
    }
    if (NCQ)
    {
        MV_U8  PMPort = 0;
        MV_BOOLEAN allNCQ = MV_TRUE;
        MV_U8 minQueueDepth = 32;
        if (pSataAdapter->sataAdapterGeneration == MV_SATA_GEN_I)
        {
            *NCQ = MV_FALSE;
        }
        else
        {
            MV_SATA_DEVICE_TYPE devType =
            mvStorageDevGetDeviceType (pSataAdapter, channelIndex);


            switch (devType)
            {
            case MV_SATA_DEVICE_TYPE_PM:{
                    MV_U8  i;
                    MV_U8  numOFDrives = 0;
                    for (i = 0; i < MV_SATA_PM_MAX_PORTS; i++)
                    {
                        if (pScsiAdapterExt->
                            ataDriveData[channelIndex][i].driveReady == MV_TRUE)
                        {
                            PMPort = i;
                            numOFDrives++;
                            if (pScsiAdapterExt->
                                ataDriveData[channelIndex][i].identifyInfo.
                                SATACapabilities.NCQSupported == MV_FALSE)
                            {
                                allNCQ = MV_FALSE;
                            }
                            else
                            {
                                if (minQueueDepth > pScsiAdapterExt->
                                    ataDriveData[channelIndex][PMPort].
                                    identifyInfo.DMAQueuedModeDepth)
                                {
                                    minQueueDepth = pScsiAdapterExt->
                                                    ataDriveData[channelIndex][PMPort].
                                                    identifyInfo.DMAQueuedModeDepth;
                                }
                            }
                        }
                    }
                    *numOfDrives = numOFDrives;
                    if ((numOFDrives != 1) && (allNCQ == MV_FALSE))
                    {
                        break;
                    }
                    if (numOFDrives == 0)
                    {
                        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d  ]: "
                                 " No drives connected\n",
                                 pSataAdapter->adapterId, channelIndex);
                        break;
                    }

                    if (numOFDrives > 1 && allNCQ == MV_TRUE)
                    {
                        *queueDepth = minQueueDepth;
                        *NCQ = MV_TRUE;
                        /*the drive connected to PM on device port number PMPort*/
                        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: "
                                 "%d NCQ drives connected. Minimum queue "
                                 "depth = %d\n",
                                 pSataAdapter->adapterId, channelIndex,
                                 PMPort, numOFDrives, *queueDepth);
                        break;
                    }
                }
                /*the drive connected to PM on device port number PMPort*/
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: mvGetEMDAAllowedModes, "
                         "one drive connected - check if NCQ allowed\n",
                         pSataAdapter->adapterId, channelIndex, PMPort);

            case MV_SATA_DEVICE_TYPE_ATA_DISK:

                if (pScsiAdapterExt->
                    ataDriveData[channelIndex][PMPort].driveReady != MV_TRUE)
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d] "
                             "mvGetEMDAAllowedModes failed, "
                             "channel not ready\n",
                             pSataAdapter->adapterId, channelIndex, PMPort);
                    return MV_FALSE;
                }
                *NCQ = pScsiAdapterExt->
                       ataDriveData[channelIndex][PMPort].identifyInfo.
                       SATACapabilities.NCQSupported;
                if (queueDepth == NULL)
                {
                    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d] "
                             "mvGetEMDAAllowedModes failed, "
                             "bad pointer to queueDepth\n",
                             pSataAdapter->adapterId, channelIndex, PMPort);
                    return MV_FALSE;
                }
                *queueDepth = pScsiAdapterExt->
                              ataDriveData[channelIndex][PMPort].
                              identifyInfo.DMAQueuedModeDepth;
                break;
            default:
                mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d] "
                         "mvGetEMDAAllowedModes failed, bad device type\n",
                         pSataAdapter->adapterId, channelIndex);
                return MV_FALSE;
            }
        }
    }
    return MV_TRUE;
}



