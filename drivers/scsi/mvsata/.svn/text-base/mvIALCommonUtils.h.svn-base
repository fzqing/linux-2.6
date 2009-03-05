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
* mvIALCommonUtils.h
*
* DESCRIPTION:
*       H implementation for IAL's extension utility functions.
*
* DEPENDENCIES:
*   mvSata.h
*   mvStorageDev.h
*
*******************************************************************************/
#ifndef __INCmvIALCommonUtilsh
#define __INCmvIALCommonUtilsh

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* includes */
#include "mvSata.h"
#include "mvStorageDev.h"

/* defines */
#define MV_IAL_COMMON_LOG_ID            2

/* typedefs */
    typedef struct serialATACapabilites
    {
        MV_BOOLEAN  SATA_GEN_I_supported:1;
        MV_BOOLEAN  SATA_GEN_II_supported:1;
        MV_BOOLEAN  NCQSupported:1;/*native command queuing*/
        MV_BOOLEAN  RxHostInitiatedPMSupported:1;/* Supports receipt of host-initiated
                                                  interface power management
                                                  requests*/
        MV_BOOLEAN  TxDeviceInitiatedPMSupported:1;/* device supports initiating
                                                    interface power management*/
        MV_BOOLEAN  TxDeviceInitiatedPMEnabled:1;
        MV_BOOLEAN  DMASetupAutoActiveSupported:1;/* supports DMA Setup Auto-Activate
                                                   optimization*/
        MV_BOOLEAN  DMASetupAutoActiveEnables:1;
        MV_BOOLEAN  NonZeroBufferOffsetSupported:1;/* supports non-zero buffer offsets
                                                    in DMA Setup FIS*/
        MV_BOOLEAN  NonZeroBufferOffsetEnabled:1;
    }SERIAL_ATA_CAPABILITIES;

    typedef struct ATAIdentifyInfo
    {
        MV_U8           version;
        MV_U8           model[24];
        MV_U8           firmware[4];
        MV_U8           UdmaMode;
        MV_U8           PIOMode;
        MV_BOOLEAN      LBA48Supported:1;/* used for READ/WRITE commands*/
        MV_BOOLEAN      writeCacheSupported:1;
        MV_BOOLEAN      writeCacheEnabled:1;
        MV_BOOLEAN      readAheadSupported:1;
        MV_BOOLEAN      readAheadEnabled:1;
        MV_BOOLEAN      DMAQueuedModeSupported:1;
        MV_U8           DMAQueuedModeDepth;
        MV_U32          ATADiskSize;
        SERIAL_ATA_CAPABILITIES SATACapabilities;/*valid only for ATA-7 or higher*/
    } ATA_IDENTIFY_INFO;


    typedef struct mvSataPMDeviceInfo
    {
        MV_U16      vendorId;
        MV_U16      deviceId;
        MV_U8       productRevision;
        MV_U8       PMSpecRevision:4;
        MV_U8       numberOfPorts:4;
    } MV_SATA_PM_DEVICE_INFO;


    MV_BOOLEAN mvParseIdentifyResult(MV_U16_PTR  iden,ATA_IDENTIFY_INFO *pIdentifyInfo);

    MV_SATA_DEVICE_TYPE mvGetSataDeviceType(MV_STORAGE_DEVICE_REGISTERS *mvStorageDevRegisters);

    MV_BOOLEAN mvInitSataDisk(MV_SATA_ADAPTER   *pSataAdapter, MV_U8 channelIndex,
                              MV_U8 PMPort, ATA_IDENTIFY_INFO   *pIdentifyInfo,
                              MV_U16_PTR identifyBuffer
                             );

    MV_BOOLEAN  mvGetPMDeviceInfo(MV_SATA_ADAPTER   *pSataAdapter,
                                  MV_U8 channelIndex,
                                  MV_SATA_PM_DEVICE_INFO *pPMDeviceInfo);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCmvIALCommonh */

