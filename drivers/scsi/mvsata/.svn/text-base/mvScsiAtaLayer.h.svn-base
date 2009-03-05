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
* mvScsiAtaLayer.c
*
* DESCRIPTION:
*       C implementation for SCSI to ATA translation layer.
*
* DEPENDENCIES:
*   mvIALCommonUtils.h
*   mvSata.h
*   mvStorageDev.h
*   mvOs.h
*
*******************************************************************************/
#ifndef __INCmvScsiAtaLayer
#define __INCmvScsiAtaLayer

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* includes */
#include "mvOs.h"
#include "mvSata.h"
#include "mvStorageDev.h"
#include "mvIALCommonUtils.h"

/* Defines */
#define MV_SAL_LOG_ID       1


#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef INOUT
#define INOUT
#endif

/* Scsi opcodes*/
/* 6 - bytes commands*/
#define SCSI_OPCODE_TEST_UNIT_READY         0x00
#define SCSI_OPCODE_REQUEST_SENSE6          0x03
#define SCSI_OPCODE_REASSIGN_BLOCKS         0x07
#define SCSI_OPCODE_READ6                   0x08
#define SCSI_OPCODE_WRITE6                  0x0A
#define SCSI_OPCODE_INQUIRY                 0x12
#define SCSI_OPCODE_VERIFY6                 0x13
#define SCSI_OPCODE_MODE_SELECT6            0x15
#define SCSI_OPCODE_MODE_SENSE6             0x1A

/* 10 - bytes commands*/
#define SCSI_OPCODE_READ_CAPACITY10         0x25
#define SCSI_OPCODE_READ10                  0x28
#define SCSI_OPCODE_WRITE10                 0x2A
#define SCSI_OPCODE_VERIFY10                0x2F
#define SCSI_OPCODE_SYNCHRONIZE_CACHE10     0x35
#define SCSI_OPCODE_SEEK10                  0x2B
#define SCSI_OPCODE_WRITE_LONG10            0x3F
#define SCSI_OPCODE_READ_LONG10				0x3E

/* 12 - bytes commands */
/* 16 - bytes commands */



    /* SCSI bus status codes */
#define MV_SCSI_STATUS_GOOD                  0x00
#define MV_SCSI_STATUS_CHECK_CONDITION       0x02
#define MV_SCSI_STATUS_CONDITION_MET         0x04
#define MV_SCSI_STATUS_BUSY                  0x08
#define MV_SCSI_STATUS_INTERMEDIATE          0x10
#define MV_SCSI_STATUS_INTERMEDIATE_COND_MET 0x14
#define MV_SCSI_STATUS_RESERVATION_CONFLICT  0x18
#define MV_SCSI_STATUS_COMMAND_TERMINATED    0x22
#define MV_SCSI_STATUS_QUEUE_FULL            0x28


/* Typedefs */


/* Scsi Sense Data Format */
/* Max length - 18 bytes, the additional sense length will not exceed 10 bytes*/
    typedef struct _mvScsiSenseData
    {
        MV_U8 ResponseCode:7;
        MV_U8 Valid:1;
        MV_U8 Reserved1;
        MV_U8 SenseKey:4;
        MV_U8 Reserved2:1;
        MV_U8 ILI:1; /* Incorrect Length Indicator*/
        MV_U8 EOM:1; /* End Of Media */
        MV_U8 FileMark:1;
        MV_U8 Information[4];
        MV_U8 AdditionalSenseLength;
        MV_U8 CommandSpecificInformation[4];
        MV_U8 AdditionalSenseCode;
        MV_U8 AdditionalSenseCodeQualifier;
        MV_U8 FieldReplaceableUnitCode;
        MV_U8 SenseKeySpecific[3];
    } MV_SCSI_SENSE_DATA;

/* Sense codes */

#define SCSI_SENSE_NO_SENSE         0x00
#define SCSI_SENSE_RECOVERED_ERROR  0x01
#define SCSI_SENSE_NOT_READY        0x02
#define SCSI_SENSE_MEDIUM_ERROR     0x03
#define SCSI_SENSE_HARDWARE_ERROR   0x04
#define SCSI_SENSE_ILLEGAL_REQUEST  0x05
#define SCSI_SENSE_UNIT_ATTENTION   0x06
#define SCSI_SENSE_DATA_PROTECT     0x07
#define SCSI_SENSE_BLANK_CHECK      0x08
#define SCSI_SENSE_UNIQUE           0x09
#define SCSI_SENSE_COPY_ABORTED     0x0A
#define SCSI_SENSE_ABORTED_COMMAND  0x0B
#define SCSI_SENSE_EQUAL            0x0C
#define SCSI_SENSE_VOL_OVERFLOW     0x0D
#define SCSI_SENSE_MISCOMPARE       0x0E
#define SCSI_SENSE_RESERVED         0x0F

/* Additional Sense codes */

#define SCSI_ADSENSE_NO_SENSE       0x00
#define SCSI_ADSENSE_ILLEGAL_COMMAND 0x20
#define SCSI_ADSENSE_ILLEGAL_BLOCK  0x21
#define SCSI_ADSENSE_INVALID_CDB    0x24
#define SCSI_ADSENSE_INVALID_LUN    0x25
#define SCSI_ADSENSE_INVALID_FIELD_IN_PARAMETER_LIST    0x26
#define SCSI_ADSENSE_BUS_RESET      0x29
#define SCSI_ADSENSE_PARAMETERS_CHANGED     0x2A
#define SCSI_ADSENSE_NO_MEDIA_IN_DEVICE 0x3a


#define MV_SCSI_RESPONSE_CODE   0x70
#define MV_SCSI_DIRECT_ACCESS_DEVICE    0x00
#define MV_MAX_MODE_SENSE_RESULT_LENGTH 50


/* Typedefs */
    typedef enum _mvScsiCompletionType
    {
        MV_SCSI_COMPLETION_INVALID_STATUS,
        MV_SCSI_COMPLETION_SUCCESS,
        MV_SCSI_COMPLETION_BAD_SCB,
        MV_SCSI_COMPLETION_BAD_SCSI_COMMAND,
        MV_SCSI_COMPLETION_ATA_FAILED,
        MV_SCSI_COMPLETION_QUEUE_FULL,
        MV_SCSI_COMPLETION_NOT_READY,
        MV_SCSI_COMPLETION_ABORTED,
        MV_SCSI_COMPLETION_OVERRUN,
        MV_SCSI_COMPLETION_UNDERRUN,
        MV_SCSI_COMPLETION_PARITY_ERROR,
        MV_SCSI_COMPLETION_DISCONNECT,
        MV_SCSI_COMPLETION_NO_DEVICE,
        MV_SCSI_COMPLETION_INVALID_BUS,
        MV_SCSI_COMPLETION_BUS_RESET,
        MV_SCSI_COMPLETION_BUSY,
        MV_SCSI_COMPLETION_UA_RESET,
        MV_SCSI_COMPLETION_UA_PARAMS_CHANGED
    }MV_SCSI_COMPLETION_TYPE;

    typedef enum _mvScsiCommandStatus
    {
        MV_SCSI_COMMAND_STATUS_COMPLETED,
        MV_SCSI_COMMAND_STATUS_QUEUED,
        MV_SCSI_COMMAND_STATUS_FAILED,
        MV_SCSI_COMMAND_STATUS_QUEUED_BY_IAL
    }MV_SCSI_COMMAND_STATUS_TYPE;

    struct _mvSataScsiCmdBlock;

    typedef MV_BOOLEAN (* mvScsiCommandCompletionCallBack)(struct mvSataAdapter *,
                                                           struct _mvSataScsiCmdBlock*);

    struct mvSalAdapterExtension;
    struct mvIalCommonAdapterExtension;

    typedef struct _mvSataScsiCmdBlock
    {
        /*  the Scsi command data block buffer*/
        IN MV_U8*       ScsiCdb;

        /* the length in bytes of the CDB (6,10,12,16)*/
        IN MV_U32       ScsiCdbLength;

        /* the scsi bus*/
        IN MV_U8        bus;

        /* the target device id*/
        IN MV_U8        target;

        /* scsi lun number of the device*/
        IN MV_U8        lun;

        /* True when the data located in the buffer pointed by pDataBuffer  */
        /* (virtual address), false when the command is READ/WRITE, in this */
        /* case the data located in a PRD table*/
        /*IN MV_BOOLEAN useSingleBuffer;*/

        /* pointer to the command data buffer*/
        IN MV_U8        *pDataBuffer;

        /* length in bytes of the command data buffer*/
        IN MV_U32       dataBufferLength;

        /* number of entries in the PRD table*/
        /*IN MV_U32     PRDTableEntries; */

        /* low 32 bits of the PRD table physical address*/
        IN MV_U32       PRDTableLowPhyAddress;

        /* high 32 bits of the PRD table physical address*/
        IN MV_U32       PRDTableHighPhyAddress;

        /* the Scsi status will be written to this field*/
        OUT MV_U8       ScsiStatus;

        /* pointer to the Scsi sense buffer*/
        IN MV_U8*       pSenseBuffer;

        /* length in bytes of the Scsi sense buffer*/
        IN MV_U32       senseBufferLength;

        /* length in bytes of the generated sense data*/
        OUT MV_U32      senseDataLength;

        /* length in bytes of the data transferred to the data buffer/s*/
        OUT MV_U32      dataTransfered;

        /* the translation layer status of the completed Scsi command */
        OUT MV_SCSI_COMPLETION_TYPE ScsiCommandCompletion;
        /* call back function called by the translation layer when the Scsi */
        /* completed    */
        IN mvScsiCommandCompletionCallBack completionCallBack;

        IN struct mvSalAdapterExtension * pSalAdapterExtension;
        IN struct mvIALCommonAdapterExtension* pIalAdapterExtension;
        /* field for IAL usage only*/
        MV_VOID_PTR     IALData;
        /* fields for internal usage for the translation layer*/


        MV_UDMA_TYPE            udmaType;
        MV_QUEUED_COMMAND_TYPE  commandType;
        /* used for sense buffer */
        MV_U32                  LowLbaAddress;
        /* Used for non-UDMA and for sense buffer */
        MV_BOOLEAN              isExtended;
        MV_U16                  splitCount;
        MV_U16                  sequenceNumber;
        /* used to create list for comands that need post interrupt service */
        struct _mvSataScsiCmdBlock  *pNext;
#ifdef MV_LOGGER
        MV_STORAGE_DEVICE_REGISTERS ATAregStruct;
#endif
    }MV_SATA_SCSI_CMD_BLOCK;

    typedef struct
    {
        MV_U32              totalIOs;
        MV_U32              totalSectorsTransferred;
    }MV_SATA_SCSI_CHANNEL_STATS;

    typedef struct
    {
        MV_BOOLEAN          driveReady;
        ATA_IDENTIFY_INFO   identifyInfo;
        MV_U16_PTR          identifyBuffer;
        MV_SATA_SCSI_CHANNEL_STATS stats;
        MV_BOOLEAN          UAConditionPending;
        MV_U8               UAEvents;
    }MV_SATA_SCSI_DRIVE_DATA;

    typedef struct mvSalAdapterExtension
    {
        MV_SATA_ADAPTER *pSataAdapter;
        MV_SATA_SCSI_CMD_BLOCK  *pHead;
        MV_U8                   UAMask;/*which UA condictions to report*/
        MV_U32  totalAccumulatedOutstanding[MV_SATA_CHANNELS_NUM];
        MV_SATA_SCSI_DRIVE_DATA     ataDriveData[MV_SATA_CHANNELS_NUM][MV_SATA_PM_MAX_PORTS];
        MV_U16  identifyBuffer[MV_SATA_CHANNELS_NUM][MV_ATA_IDENTIFY_DEV_DATA_LENGTH];

    }MV_SAL_ADAPTER_EXTENSION;


    MV_VOID     mvSataScsiInitAdapterExt(MV_SAL_ADAPTER_EXTENSION *pAdapterExt,
                                         MV_SATA_ADAPTER* pSataAdapter);

    MV_VOID     mvSataScsiPostIntService(MV_SAL_ADAPTER_EXTENSION *pAdapterExt);


    MV_SCSI_COMMAND_STATUS_TYPE mvSataExecuteScsiCommand(MV_SATA_SCSI_CMD_BLOCK *pMvSataScsiCmdBlock);

    MV_VOID     mvSataScsiSetDriveReady(MV_SAL_ADAPTER_EXTENSION *pAdapterExt,
                                        MV_U8   channelIndex, MV_U8 PMPort,
                                        MV_BOOLEAN  isReady);

    MV_VOID mvSataScsiNotifyUA(MV_SAL_ADAPTER_EXTENSION *pAdapterExt,
                               MV_U8    channelIndex, MV_U8 PMPort);

/* Locals */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __IsNCmvScsiAtaLayer */
