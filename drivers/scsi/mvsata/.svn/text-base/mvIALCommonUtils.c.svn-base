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
*******************************************************************************/

/* includes */
#include "mvOs.h"
#include "mvIALCommonUtils.h"


static MV_BOOLEAN mvConfigSataDisk(MV_SATA_ADAPTER *pSataAdapter,
                                   MV_U8 channelIndex,
                                   MV_U8 PMPort,
                                   ATA_IDENTIFY_INFO   *pIdentifyInfo,
                                   MV_U16_PTR identifyBuffer);


static MV_VOID mvAta2HostString(MV_U16 *source, MV_U16 *target,
                                MV_U32 wordsCount)
{
    MV_U32 i;
    for (i=0 ; i < wordsCount; i++)
    {
        /* Big to little*/
        target[i] = (source[i] >> 8) | ((source[i] & 0xff) << 8);
        /* Little to cpu*/
        target[i] = MV_LE16_TO_CPU(target[i]);
    }
}

/******************************************************************************
 *  Name: ParseIdentifyResult
 *
 *  Description:    this functions parses the identify command results, checks
 *                  that the connected deives can be accesed by device EDMA,
 *                  and updates the ATA drive parameters stucture accordingly.
 *
 *  Parameters:     pSataChannel - pointer to the channel data structure.
 *                  pIdentifyInfo- pointer to the ATA parameters structure.
 *
 *  Returns:        MV_TRUE if the ATA drive supported by device.
 *
 ******************************************************************************/
MV_BOOLEAN mvParseIdentifyResult(MV_U16_PTR  iden,
                                 ATA_IDENTIFY_INFO   *pIdentifyInfo)
{
    char    temp[80];
    MV_U8  version;
    MV_BOOLEAN  udmaModeEnabled = MV_FALSE;

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "Parse IDENTIFY data:\n");

    mvAta2HostString( iden + IDEN_MODEL_OFFSET, (MV_U16_PTR)temp, 24);
    temp[25] = '\0';
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n","Model", temp);
    memcpy(&pIdentifyInfo->model[0], iden + IDEN_MODEL_OFFSET, 24);
    memcpy(&pIdentifyInfo->firmware[0], iden + IDEN_FIRMWARE_OFFSET, 4);
    /* ATA version supported*/
    if (iden[IDEN_ATA_VERSION] & MV_BIT7)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %d\n", "ATA version supported", 7);
        version = 7;
    }
    else if (iden[IDEN_ATA_VERSION] & MV_BIT6)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %d\n", "ATA version supported", 6);
        version = 6;
    }
    else if (iden[IDEN_ATA_VERSION] & MV_BIT5)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %d\n", "ATA version supported", 5);
        version = 5;
    }
    else if (iden[IDEN_ATA_VERSION] & MV_BIT4)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %d\n", "ATA version supported", 4);
        version = 4;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, " IDENTIFY info: ATA "
                 "version(%d) not supported\n", iden[IDEN_ATA_VERSION]);
        return MV_FALSE;
    }
    pIdentifyInfo->version = version;
    /*LBA addressing*/
    if ((version >= 6) && (!(iden[IDEN_CAPACITY_1_OFFSET] & MV_BIT9)))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, " IDENTIFY info: LBA not supported\n");
        return MV_FALSE;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "Capabilities",
                 "LBA supported");
    }
    /* 48 bit address */
    if ((version >= 6) && (iden[IDEN_SUPPORTED_COMMANDS2] & MV_BIT10))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "LBA48 addressing", "supported");
        pIdentifyInfo->LBA48Supported = MV_TRUE;

        if ((iden[103]) || (iden[102]))
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "WARNING Disk size more "
                     "than 32 bit sectors - setting to 0xffffffff sectors\n");
            pIdentifyInfo->ATADiskSize = 0xffffffff;
        }
        else
        {
            pIdentifyInfo->ATADiskSize = ((MV_U32)iden[101] << 16) |
                                         ((MV_U32)iden[100]);
        }

        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - 0x%x%04x%04x%04x sectors\n",
                 "Number of sectors", iden[103] , iden[102], iden[101],
                 iden[100]);
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n",
                 "LBA48 addressing", "Not supported");
        pIdentifyInfo->LBA48Supported = MV_FALSE;
        pIdentifyInfo->ATADiskSize = ((MV_U32)iden[IDEN_NUM_OF_ADDRESSABLE_SECTORS + 1] << 16) |
                                     ((MV_U32)iden[IDEN_NUM_OF_ADDRESSABLE_SECTORS]);

        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - 0x%x sectors\n",
                 "Number of sectors",
                 (iden[IDEN_NUM_OF_ADDRESSABLE_SECTORS + 1] << 16) |
                 ((MV_U32)iden[IDEN_NUM_OF_ADDRESSABLE_SECTORS]));


    }
    /*DMA support*/
    if ((version >= 6) && (!(iden[IDEN_CAPACITY_1_OFFSET] & MV_BIT8)))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "IDENTIFY info: DMA not "
                 "supported\n");
        return MV_FALSE;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "Capabilities",
                 "DMA supported");
    }
    /* PIO */
    if ((iden[IDEN_VALID] & MV_BIT1) == 0)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, " IDENTIFY info: not "
                 "able to find PIO mode\n");
        return MV_FALSE;
    }
    else if (iden[IDEN_PIO_MODE_SPPORTED] & MV_BIT0)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "PIO mode 3",
                 "supported");
        pIdentifyInfo->PIOMode = MV_ATA_TRANSFER_PIO_3;
    }
    else if (iden[IDEN_PIO_MODE_SPPORTED] & MV_BIT1)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "PIO mode 4",
                 "supported");
        pIdentifyInfo->PIOMode = MV_ATA_TRANSFER_PIO_4;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "IDENTIFY info: PIO "
                 "modes 3 and 4 not supported\n");
        pIdentifyInfo->PIOMode = MV_ATA_TRANSFER_PIO_SLOW;
        return MV_FALSE;
    }


    /*UDMA*/
    if ((iden[IDEN_VALID] & MV_BIT2) == 0)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, " IDENTIFY info: not "
                 "able to find UDMA mode\n");
        return MV_FALSE;
    }


    if ((version >= 7) && (iden[IDEN_UDMA_MODE] & MV_BIT6))
    {
        pIdentifyInfo->UdmaMode = MV_ATA_TRANSFER_UDMA_6;
        if (iden[IDEN_UDMA_MODE] & MV_BIT14)
        {
            udmaModeEnabled = MV_TRUE;
        }
    }
    else if ((version >= 6) && (iden[IDEN_UDMA_MODE] & MV_BIT5))
    {
        pIdentifyInfo->UdmaMode = MV_ATA_TRANSFER_UDMA_5;
        if (iden[IDEN_UDMA_MODE] & MV_BIT13)
        {
            udmaModeEnabled = MV_TRUE;
        }
    }
    else if ((version >= 5) && (iden[IDEN_UDMA_MODE] & MV_BIT4))
    {
        pIdentifyInfo->UdmaMode = MV_ATA_TRANSFER_UDMA_4;
        if (iden[IDEN_UDMA_MODE] & MV_BIT12)
        {
            udmaModeEnabled = MV_TRUE;
        }
    }
    else if ((version >= 4) && (iden[IDEN_UDMA_MODE] & MV_BIT3))
    {
        pIdentifyInfo->UdmaMode = MV_ATA_TRANSFER_UDMA_3;
        if (iden[IDEN_UDMA_MODE] & MV_BIT11)
        {
            udmaModeEnabled = MV_TRUE;
        }
    }
    else if (iden[IDEN_UDMA_MODE] & MV_BIT2)
    {
        pIdentifyInfo->UdmaMode = MV_ATA_TRANSFER_UDMA_2;
        if (iden[IDEN_UDMA_MODE] & MV_BIT10)
        {
            udmaModeEnabled = MV_TRUE;
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "IDENTIFY info: Ultra"
                 " DMA mode < 2 not supported IDENTIFY[88] 0x%04x\n",
                 iden[IDEN_UDMA_MODE]);
        pIdentifyInfo->UdmaMode = MV_ATA_TRANSFER_UDMA_0;
        return MV_FALSE;
    }
    if (udmaModeEnabled == MV_TRUE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s%d%s\n",
                 "Ultra DMA mode","UDMA mode ",
                 pIdentifyInfo->UdmaMode - MV_ATA_TRANSFER_UDMA_0,
                 " supported and enabled");
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s%d%s\n",
                 "Ultra DMA mode","UDMA mode ",
                 pIdentifyInfo->UdmaMode - MV_ATA_TRANSFER_UDMA_0,
                 " supported but disabled");
    }


    if ((iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT13))
    {
        if (iden[IDEN_ENABLED_COMMANDS1] & MV_BIT13)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "READ BUFFER", "supported and enabled");
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "READ BUFFER", "supported and disabled");
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "READ BUFFER",
                 " Not supported");
    }

    if ((iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT12))
    {
        if (iden[IDEN_ENABLED_COMMANDS1] & MV_BIT12)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "WRITE BUFFER", "supported and enabled");
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "WRITE BUFFER", "supported and disabled");
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "WRITE BUFFER",
                 "Not supported");
    }

    if ((iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT6))
    {
        pIdentifyInfo->readAheadSupported = MV_TRUE;
        if (iden[IDEN_ENABLED_COMMANDS1] & MV_BIT6)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "READ LOOK-AHEAD", "supported and enabled");
            pIdentifyInfo->readAheadEnabled = MV_TRUE;
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "READ LOOK-AHEAD", "supported and disabled");
            pIdentifyInfo->readAheadEnabled = MV_FALSE;
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n",
                 "READ LOOK-AHEAD","Not supported");
        pIdentifyInfo->readAheadSupported = MV_FALSE;
        pIdentifyInfo->readAheadEnabled = MV_FALSE;
    }

    if ((iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT5))
    {
        pIdentifyInfo->writeCacheSupported = MV_TRUE;
        if (iden[IDEN_ENABLED_COMMANDS1] & MV_BIT5)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "WRITE CACHE", "supported and enabled");
            pIdentifyInfo->writeCacheEnabled = MV_TRUE;
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "WRITE CACHE", "supported and disabled");
            pIdentifyInfo->writeCacheEnabled = MV_FALSE;
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "WRITE CACHE",
                 "Not supported");
        pIdentifyInfo->writeCacheSupported = MV_FALSE;
        pIdentifyInfo->writeCacheEnabled = MV_FALSE;
    }

    if ((iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT3))
    {

        if (iden[IDEN_ENABLED_COMMANDS1] & MV_BIT3)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "POWER MANAGMENT", "supported and enabled");
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n",
                     "POWER MANAGMENT", "supported and disabled");
        }

    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n",
                 "POWER MANAGMENT","Not supported");
    }

    if ((iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT0))
    {

        if (iden[IDEN_ENABLED_COMMANDS1] & MV_BIT0)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n", "SMART",
                     "supported and enabled");
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %20s\n", "SMART",
                     "supported and disabled");
        }
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "SMART",
                 "Not supported");
    }


    /* check if REAd/WRITE DMA QUEUE commands supported */
    pIdentifyInfo->DMAQueuedModeDepth = (iden[IDEN_QUEUE_DEPTH] & 0x1f) + 1;
    if ((version >= 5) &&(iden[IDEN_SUPPORTED_COMMANDS2] & MV_BIT1))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %15s Queue Depth %d\n",
                 "READ/WRITE DMA QUEUE","supported",
                 (iden[IDEN_QUEUE_DEPTH] & 0x1f) + 1);
        pIdentifyInfo->DMAQueuedModeSupported = MV_TRUE;
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n",
                 "READ/WRITE DMA QUEUE", "not supported");
        pIdentifyInfo->DMAQueuedModeSupported = MV_FALSE;
    }

    /*check that the non-UDMA ATA commands supported*/

    /*FLUSH CHACHE*/
    if ((version >=6) && ((iden[IDEN_SUPPORTED_COMMANDS2] & MV_BIT12) == 0))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n",
                 "FLUSH CACHE command", "not supported");
    }
    else
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n",
                 "FLUSH CACHE command", "supported");
    }

    pIdentifyInfo->SATACapabilities.SATA_GEN_I_supported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.SATA_GEN_II_supported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.NCQSupported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.RxHostInitiatedPMSupported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.TxDeviceInitiatedPMSupported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.TxDeviceInitiatedPMEnabled = MV_FALSE;
    pIdentifyInfo->SATACapabilities.DMASetupAutoActiveSupported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.DMASetupAutoActiveEnables = MV_FALSE;
    pIdentifyInfo->SATACapabilities.NonZeroBufferOffsetSupported = MV_FALSE;
    pIdentifyInfo->SATACapabilities.NonZeroBufferOffsetEnabled = MV_FALSE;

    if (version >= 6)
    {
        if (iden[IDEN_SATA_CAPABILITIES] & MV_BIT1)
        {
            pIdentifyInfo->SATACapabilities.SATA_GEN_I_supported = MV_TRUE;
        }
        else
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "SATA Gen I",
                     "not supported");
        }
        if (iden[IDEN_SATA_CAPABILITIES] & MV_BIT2)
        {
            pIdentifyInfo->SATACapabilities.SATA_GEN_II_supported = MV_TRUE;
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "SATA Gen II",
                     "supported");
        }

        if (iden[IDEN_SATA_CAPABILITIES] & MV_BIT8)
        {
            pIdentifyInfo->SATACapabilities.NCQSupported = MV_TRUE;
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %s\n", "NCQ",
                     "supported");
        }

        if (iden[IDEN_SATA_CAPABILITIES] & MV_BIT9)
        {
            pIdentifyInfo->SATACapabilities.RxHostInitiatedPMSupported = MV_TRUE;
        }

        if (iden[IDEN_SATA_FEATURES_SUPPORTED] & MV_BIT1)
        {
            pIdentifyInfo->SATACapabilities.NonZeroBufferOffsetSupported = MV_TRUE;
            if (iden[IDEN_SATA_FEATURES_ENABLED] & MV_BIT1)
            {
                pIdentifyInfo->SATACapabilities.NonZeroBufferOffsetEnabled = MV_TRUE;
            }
        }
        if (iden[IDEN_SATA_FEATURES_SUPPORTED] & MV_BIT2)
        {
            pIdentifyInfo->SATACapabilities.DMASetupAutoActiveSupported = MV_TRUE;
            if (iden[IDEN_SATA_FEATURES_ENABLED] & MV_BIT2)
            {
                pIdentifyInfo->SATACapabilities.DMASetupAutoActiveEnables = MV_TRUE;
            }
        }
        if (iden[IDEN_SATA_FEATURES_SUPPORTED] & MV_BIT3)
        {
            pIdentifyInfo->SATACapabilities.TxDeviceInitiatedPMSupported = MV_TRUE;
            if (iden[IDEN_SATA_FEATURES_ENABLED] & MV_BIT3)
            {
                pIdentifyInfo->SATACapabilities.TxDeviceInitiatedPMEnabled = MV_TRUE;
            }
        }
    }

    return MV_TRUE;
}
/*******************************************************************************
* mvGetSataDeviceType - short description
*
* DESCRIPTION:
*       None.
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       None.
*
*******************************************************************************/
MV_SATA_DEVICE_TYPE mvGetSataDeviceType(
                                       MV_STORAGE_DEVICE_REGISTERS *mvStorageDevRegisters)
{
    if (((mvStorageDevRegisters->sectorCountRegister & 0xff) != 1) ||
        ((mvStorageDevRegisters->lbaLowRegister & 0xff) != 1))
    {
        return MV_SATA_DEVICE_TYPE_UNKNOWN;
    }
    if ((((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0) &&
         ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0)) ||
        (((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x3C) &&/*ATA-7*/
         ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0xC3)))
    {
        return MV_SATA_DEVICE_TYPE_ATA_DISK;
    }
    if ((((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x14) &&
         ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0xEB))/* ||
         (((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x69) &&
         ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0x96))*/)
    {
        return MV_SATA_DEVICE_TYPE_ATAPI_DISK;
    }
    if (((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x69) &&
        ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0x96))
    {
        return MV_SATA_DEVICE_TYPE_PM;
    }
    return MV_SATA_DEVICE_TYPE_UNKNOWN;
}

#ifdef MV_LOG_DEBUG
static void printIdentifyBuffer(MV_U16_PTR identifyBuffer)
{
    MV_U8 i,j;
    /*Print Identify buffer*/
    for (i = 0; i < 0x20; i++)
    {
        mvLogMsg(MV_RAW_MSG_ID,  0, "Words [%03d-%03d]: ", i*8, i*8+7);
        for (j = 0; j < 0x8; j++)
        {
            mvLogMsg(MV_RAW_MSG_ID,  0, "0x%04X ", identifyBuffer[i*8+j]);
        }
        mvLogMsg(MV_RAW_MSG_ID,  0, "\n");
    }
}
#endif

/*******************************************************************************
* mvConfigSataDisk - short description
*
* DESCRIPTION:
*       None.
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       None.
*
*******************************************************************************/

MV_BOOLEAN mvConfigSataDisk(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex,
                            MV_U8 PMPort, ATA_IDENTIFY_INFO   *pIdentifyInfo,
                            MV_U16_PTR identifyBuffer)
{
    MV_STORAGE_DEVICE_REGISTERS inATARegs;
    MV_STORAGE_DEVICE_REGISTERS outATARegs;
    /* identify device*/
    memset(&inATARegs, 0, sizeof(inATARegs));
    inATARegs.commandRegister = MV_ATA_COMMAND_IDENTIFY;
    if (mvStorageDevATAIdentifyDevice(pSataAdapter, channelIndex, PMPort,
                                      identifyBuffer)
        == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: failed "
                 "to perform ATA Identify command\n", pSataAdapter->adapterId,
                 channelIndex, PMPort);
        return MV_FALSE;
    }
#ifdef MV_LOG_DEBUG
    mvLogMsg(MV_RAW_MSG_ID, 0, "Drive [%d,%d,%d] Identify Buffer:\n",
             pSataAdapter->adapterId, channelIndex, PMPort);

    printIdentifyBuffer(identifyBuffer);
#endif
    if (mvParseIdentifyResult(identifyBuffer, pIdentifyInfo) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: in "
                 "parsing ATA Identify Data\n", pSataAdapter->adapterId,
                 channelIndex, PMPort);
        return MV_FALSE;
    }
    if ((pIdentifyInfo->writeCacheSupported == MV_TRUE) &&
        (pIdentifyInfo->writeCacheEnabled == MV_FALSE))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG,"[%d %d %d]: Write Cache "
                 "supported but disabled\n", pSataAdapter->adapterId,
                 channelIndex, PMPort);
        memset(&inATARegs, 0, sizeof(inATARegs));
        inATARegs.commandRegister = MV_ATA_COMMAND_SET_FEATURES;
        inATARegs.featuresRegister = MV_ATA_SET_FEATURES_ENABLE_WCACHE;

        if (mvStorageDevExecutePIO(pSataAdapter, channelIndex, PMPort,
                                   MV_NON_UDMA_PROTOCOL_NON_DATA,
                                   MV_FALSE, NULL,0, &inATARegs, &outATARegs)
            == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                     "Set Features failed(ENABLE WCACHE)\n",
                     pSataAdapter->adapterId, channelIndex, PMPort);
            return MV_FALSE;
        }
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Write Cache "
                 "enabled\n", pSataAdapter->adapterId, channelIndex, PMPort);
    }
    if ((pIdentifyInfo->readAheadSupported == MV_TRUE) &&
        (pIdentifyInfo->readAheadEnabled == MV_FALSE))
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Read Look "
                 "Ahead supported but disabled\n", pSataAdapter->adapterId,
                 channelIndex, PMPort);
        memset(&inATARegs, 0, sizeof(inATARegs));
        inATARegs.commandRegister = MV_ATA_COMMAND_SET_FEATURES;
        inATARegs.featuresRegister = MV_ATA_SET_FEATURES_ENABLE_RLA;
        if (mvStorageDevExecutePIO(pSataAdapter, channelIndex, PMPort,
                                   MV_NON_UDMA_PROTOCOL_NON_DATA,
                                   MV_FALSE, NULL,0, &inATARegs, &outATARegs)
            == MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"[%d %d %d]: "
                     "Set Features failed(ENABLE RLA)\n",
                     pSataAdapter->adapterId, channelIndex, PMPort);
            return MV_FALSE;
        }
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Read Look "
                 "Ahead enabled\n", pSataAdapter->adapterId, channelIndex,
                 PMPort);
    }
    /* mvStorageDevATASetFeatures */

    /* Set transfer mode */
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Set transfer mode "
             "XFER_PIO_SLOW\n", pSataAdapter->adapterId, channelIndex, PMPort);
    memset(&inATARegs, 0, sizeof(inATARegs));
    inATARegs.commandRegister = MV_ATA_COMMAND_SET_FEATURES;
    inATARegs.featuresRegister = MV_ATA_SET_FEATURES_TRANSFER;
    inATARegs.sectorCountRegister = MV_ATA_TRANSFER_PIO_SLOW;
    if (mvStorageDevExecutePIO(pSataAdapter, channelIndex, PMPort,
                               MV_NON_UDMA_PROTOCOL_NON_DATA,
                               MV_FALSE, NULL,0, &inATARegs, &outATARegs) ==
        MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: "
                 "Set Features failed to set XFER PIO SLOW\n",
                 pSataAdapter->adapterId, channelIndex, PMPort);

        return MV_FALSE;
    }

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Set transfer mode "
             "XFER_PIO_%d\n", pSataAdapter->adapterId, channelIndex, PMPort,
             pIdentifyInfo->PIOMode - MV_ATA_TRANSFER_PIO_0);

    memset(&inATARegs, 0, sizeof(inATARegs));
    inATARegs.commandRegister = MV_ATA_COMMAND_SET_FEATURES;
    inATARegs.featuresRegister = MV_ATA_SET_FEATURES_TRANSFER;
    inATARegs.sectorCountRegister = pIdentifyInfo->PIOMode;
    if (mvStorageDevExecutePIO(pSataAdapter, channelIndex, PMPort,
                               MV_NON_UDMA_PROTOCOL_NON_DATA,
                               MV_FALSE, NULL,0, &inATARegs, &outATARegs) ==
        MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: "
                 "Set Features failed to set XFER PIO %d\n",
                 pSataAdapter->adapterId, channelIndex, PMPort,
                 pIdentifyInfo->PIOMode - MV_ATA_TRANSFER_PIO_0);
        return MV_FALSE;
    }


    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d %d]: Set transfer mode"
             " XFER_UDMA_%d\n", pSataAdapter->adapterId, channelIndex, PMPort,
             pIdentifyInfo->UdmaMode & 0xf);
    memset(&inATARegs, 0, sizeof(inATARegs));
    inATARegs.commandRegister = MV_ATA_COMMAND_SET_FEATURES;
    inATARegs.featuresRegister = MV_ATA_SET_FEATURES_TRANSFER;
    inATARegs.sectorCountRegister = pIdentifyInfo->UdmaMode;
    if (mvStorageDevExecutePIO(pSataAdapter, channelIndex, PMPort,
                               MV_NON_UDMA_PROTOCOL_NON_DATA,
                               MV_FALSE, NULL,0, &inATARegs, &outATARegs) ==
        MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: "
                 "Set Features failed to set XFER UDMA %d\n",
                 pSataAdapter->adapterId, channelIndex, PMPort,
                 pIdentifyInfo->UdmaMode & 0xf);
        return MV_FALSE;
    }
    return MV_TRUE;
}

/*******************************************************************************
* mvInitSataDisk - short description
*
* DESCRIPTION:
*       None.
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       None.
*
*******************************************************************************/
MV_BOOLEAN mvInitSataDisk(MV_SATA_ADAPTER   *pSataAdapter, MV_U8 channelIndex,
                          MV_U8 PMPort, ATA_IDENTIFY_INFO   *pIdentifyInfo,
                          MV_U16_PTR identifyBuffer
                         )
{
#if 0
    MV_STORAGE_DEVICE_REGISTERS mvStorageDevRegisters;
    MV_SATA_DEVICE_TYPE deviceType;

    /* Software reset channel */
    if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex, PMPort,
                                       &mvStorageDevRegisters)== MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: Software reset "
                 "failed\n", pSataAdapter->adapterId, channelIndex, PMPort);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR," ATA Drive Registers:\n");
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","Error", mvStorageDevRegisters.errorRegister);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","SectorCount", mvStorageDevRegisters.sectorCountRegister);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","LBA Low", mvStorageDevRegisters.lbaLowRegister);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","LBA Mid", mvStorageDevRegisters.lbaMidRegister);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","LBA High", mvStorageDevRegisters.lbaHighRegister);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","Device", mvStorageDevRegisters.deviceRegister);
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR,"%20s : %04x\n","Status", mvStorageDevRegisters.statusRegister);
        /* Software reset PM */
        mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                       MV_SATA_PM_CONTROL_PORT, NULL);
        /*if sw reset of the drive didn't finished, estabisl sama comm  */
        /*again and don't clear s-error so D2H Fis will be blocked      */
        mvPMDevEnableStaggeredSpinUp(pSataAdapter,channelIndex , PMPort);
        return MV_FALSE;
    }

    if (mvStorageDevRegisters.errorRegister != 1)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: Device Diagnostics "
                 "failed\n", pSataAdapter->adapterId, channelIndex, PMPort);
        return MV_FALSE;
    }


    deviceType = mvGetSataDeviceType(&mvStorageDevRegisters);
    if (deviceType != MV_SATA_DEVICE_TYPE_ATA_DISK)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: Bad Device"
                 " Type (%d) failed\n", pSataAdapter->adapterId, channelIndex,
                 PMPort, deviceType);
        return MV_FALSE;
    }
#endif
    if (mvConfigSataDisk(pSataAdapter, channelIndex, PMPort, pIdentifyInfo,
                         identifyBuffer) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: Failed to"
                 " Config Device\n", pSataAdapter->adapterId, channelIndex,
                 PMPort);
        if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex, PMPort,
                                           NULL)== MV_FALSE)
        {
            mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d %d]: "
                     "Software reset failed\n", pSataAdapter->adapterId,
                     channelIndex, PMPort);
            /* Software reset PM */
            mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
                                           MV_SATA_PM_CONTROL_PORT, NULL);

        }
        return MV_FALSE;
    }
    return MV_TRUE;
}
MV_BOOLEAN  mvGetPMDeviceInfo(MV_SATA_ADAPTER   *pSataAdapter,
                              MV_U8 channelIndex,
                              MV_SATA_PM_DEVICE_INFO *pPMDeviceInfo)
{
    MV_U32  regVal;

    if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                       MV_SATA_GSCR_ID_REG_NUM, &regVal, NULL) == MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: "
                 "mvGetPMDeviceInfo Failed", pSataAdapter->adapterId,
                 channelIndex);
        return MV_FALSE;
    }
    pPMDeviceInfo->vendorId = (MV_U16)(regVal & 0xffff);
    pPMDeviceInfo->deviceId = (MV_U16)((regVal & 0xffff0000) >> 16);

    if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                       MV_SATA_GSCR_REVISION_REG_NUM, &regVal, NULL)== MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: "
                 "mvGetPMDeviceInfo Failed", pSataAdapter->adapterId,
                 channelIndex);
        return MV_FALSE;
    }

    pPMDeviceInfo->PMSpecRevision = (MV_U8)(regVal & 0xff);
    pPMDeviceInfo->productRevision = (MV_U8)((regVal & 0xff00) >> 8);

    if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
                       MV_SATA_GSCR_INFO_REG_NUM, &regVal, NULL)== MV_FALSE)
    {
        mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG_ERROR, "[%d %d]: "
                 "mvGetPMDeviceInfo Failed", pSataAdapter->adapterId,
                 channelIndex);
        return MV_FALSE;
    }
    pPMDeviceInfo->numberOfPorts = (MV_U8)(regVal & 0xf);

    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "[%d %d]: PM Information:\n",
             pSataAdapter->adapterId,channelIndex);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %04x\n", "Vendor Id", pPMDeviceInfo->vendorId);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %04x\n", "Device Id", pPMDeviceInfo->deviceId);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %02x\n", "Product Revision", pPMDeviceInfo->productRevision);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %02x\n", "Spec Revision", pPMDeviceInfo->PMSpecRevision);
    mvLogMsg(MV_IAL_COMMON_LOG_ID, MV_DEBUG, "%25s - %02x\n", "Fan-out ports", pPMDeviceInfo->numberOfPorts);
    return MV_TRUE;
}

