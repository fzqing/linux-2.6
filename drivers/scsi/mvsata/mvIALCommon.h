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
* mvIALCommon.h
*
* DESCRIPTION:
*       H implementation for IAL's extension utility functions.
*
* DEPENDENCIES:
*   mvSata.h
*   mvStorageDev.h
*
*******************************************************************************/
#ifndef __INCmvIALCommonh
#define __INCmvIALCommonh

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* includes */
#include "mvSata.h"
#include "mvStorageDev.h"

/* defines  */

/*Timer period in milliseconds*/
#define MV_IAL_ASYNC_TIMER_PERIOD       500
#define MV_IAL_SRST_TIMEOUT             31000

/* typedefs */



    typedef enum mvAdapterState
    {
        ADAPTER_INITIALIZING,
        ADAPTER_READY,
        ADAPTER_FATAL_ERROR
    } MV_ADAPTER_STATE;

    typedef enum mvChannelState
    {
        CHANNEL_NOT_CONNECTED,
        CHANNEL_CONNECTED,
        CHANNEL_IN_SRST,
        CHANNEL_PM_STAGGERED_SPIN_UP,
        CHANNEL_PM_SRST_DEVICE,
        CHANNEL_READY,
        CHANNEL_PM_HOT_PLUG,
    } MV_CHANNEL_STATE;

typedef struct mvDriveSerialNumber
{
    MV_U8 serial[IDEN_SERIAL_NUM_SIZE];
}   MV_DRIVE_SERIAL_NUMBER;


typedef struct mvDrivesInfo
{
    MV_U16                      drivesSnapshotSaved;
    MV_DRIVE_SERIAL_NUMBER      driveSerialSaved[MV_SATA_PM_MAX_PORTS];
    MV_U16                      drivesSnapshotCurrent;
    MV_DRIVE_SERIAL_NUMBER      driveSerialCurrent[MV_SATA_PM_MAX_PORTS];
}   MV_DRIVES_INFO;


    typedef struct mvIALChannelExtension
    {
        MV_U8                       PMnumberOfPorts;
        MV_U16                      PMdevsToInit;
        MV_U8                       devInSRST;
        MV_BOOLEAN                  completionError;
        MV_U8                       pmAccessType;
        MV_U8                       pmReg;
        MV_BOOLEAN                  pmRegAccessInProgress;
        MV_BOOLEAN                  pmAsyncNotifyEnabled;
        MV_U32                      SRSTTimerThreshold;
        MV_U32                      SRSTTimerValue;
        MV_VOID_PTR                 IALChannelPendingCmdQueue;
    MV_BOOLEAN                  bHotPlug;
    MV_DRIVES_INFO              drivesInfo;
} MV_IAL_COMMON_CHANNEL_EXTENSION;


    typedef struct mvIALCommonAdapterExtension
    {
        MV_SATA_ADAPTER   *pSataAdapter;
        MV_ADAPTER_STATE  adapterState;
        MV_CHANNEL_STATE  channelState[MV_SATA_CHANNELS_NUM];
        MV_IAL_COMMON_CHANNEL_EXTENSION IALChannelExt[MV_SATA_CHANNELS_NUM];
    } MV_IAL_COMMON_ADAPTER_EXTENSION;


/*Public functions*/
    MV_BOOLEAN mvAdapterStartInitialization(MV_SATA_ADAPTER* pSataAdapter,
                                            MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                            MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

    void mvRestartChannel(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                          MV_U8 channelIndex,
                          MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt,
                          MV_BOOLEAN    bBusReset);

    void mvStopChannel(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                       MV_U8 channelIndex,
                       MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

    void mvPMHotPlugDetected(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                             MV_U8 channelIndex,
                             MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);


    MV_SCSI_COMMAND_STATUS_TYPE mvExecuteScsiCommand(MV_SATA_SCSI_CMD_BLOCK  *pScb,
                                                     MV_BOOLEAN canQueue);

    MV_BOOLEAN  mvIALTimerCallback(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                   MV_SAL_ADAPTER_EXTENSION *scsiAdapterExt);

    void mvCommandCompletionErrorHandler(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                         MV_U8 channelIndex);

    MV_BOOLEAN mvRemoveFromSCSICommandQueue(MV_IAL_COMMON_ADAPTER_EXTENSION *ialExt,
                                            MV_U8 channelIndex,
                                            MV_SATA_SCSI_CMD_BLOCK *pScb);

/*The following functions which must be implemented in IAL*/

    MV_BOOLEAN IALConfigQueuingMode(MV_SATA_ADAPTER *pSataAdapter,
                                    MV_U8 channelIndex,
                                    MV_EDMA_MODE mode,
                                    MV_U8 queueDepth);


    MV_BOOLEAN IALInitChannel(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex);

    void IALReleaseChannel(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex);
    MV_BOOLEAN IALBusChangeNotify(MV_SATA_ADAPTER *pSataAdapter,
                                  MV_U8 channelIndex);
    MV_BOOLEAN IALBusChangeNotifyEx(MV_SATA_ADAPTER *pSataAdapter,
                              MV_U8 channelIndex,
                              MV_U16 targetsToRemove,
                              MV_U16 targetsToAdd);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCmvIALCommonh */

