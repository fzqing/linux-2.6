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
* mvStorageDev.h - Header File for mvStorageDev.c.
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*       mvSata.h.
*       mvOs.h.
*       ATA/ATAPI-6 standard
*
*******************************************************************************/
#ifndef __INCmvStorageDevh
#define __INCmvStorageDevh
#ifdef __cplusplus

extern "C" {
#endif /* __cplusplus */

/* Includes */
#include "mvOs.h"
#include "mvSata.h"
#include "mvRegs.h"

/* Definitions */

/* ATA register on the ATA drive*/

#define MV_EDMA_ATA_FEATURES_ADDR               0x11
#define MV_EDMA_ATA_SECTOR_COUNT_ADDR           0x12
#define MV_EDMA_ATA_LBA_LOW_ADDR                0x13
#define MV_EDMA_ATA_LBA_MID_ADDR                0x14
#define MV_EDMA_ATA_LBA_HIGH_ADDR               0x15
#define MV_EDMA_ATA_DEVICE_ADDR                 0x16
#define MV_EDMA_ATA_COMMAND_ADDR                0x17

#define MV_ATA_ERROR_STATUS                     MV_BIT0
#define MV_ATA_DATA_REQUEST_STATUS              MV_BIT3
#define MV_ATA_SERVICE_STATUS                   MV_BIT4
#define MV_ATA_DEVICE_FAULT_STATUS              MV_BIT5
#define MV_ATA_READY_STATUS                     MV_BIT6
#define MV_ATA_BUSY_STATUS                      MV_BIT7


#define MV_ATA_COMMAND_READ_SECTORS             0x20
#define MV_ATA_COMMAND_READ_SECTORS_EXT         0x24
#define MV_ATA_COMMAND_READ_LOG_EXT				0x2F
#define MV_ATA_COMMAND_READ_VERIFY_SECTORS      0x40
#define MV_ATA_COMMAND_READ_VERIFY_SECTORS_EXT  0x42
#define MV_ATA_COMMAND_READ_BUFFER              0xE4
#define MV_ATA_COMMAND_WRITE_BUFFER             0xE8
#define MV_ATA_COMMAND_WRITE_SECTORS            0x30
#define MV_ATA_COMMAND_WRITE_SECTORS_EXT        0x34
#define MV_ATA_COMMAND_DIAGNOSTIC               0x90
#define MV_ATA_COMMAND_SMART                    0xb0
#define MV_ATA_COMMAND_READ_MULTIPLE            0xc4
#define MV_ATA_COMMAND_WRITE_MULTIPLE           0xc5
#define MV_ATA_COMMAND_STANDBY_IMMEDIATE        0xe0
#define MV_ATA_COMMAND_IDLE_IMMEDIATE           0xe1
#define MV_ATA_COMMAND_STANDBY                  0xe2
#define MV_ATA_COMMAND_IDLE                     0xe3
#define MV_ATA_COMMAND_SLEEP                    0xe6
#define MV_ATA_COMMAND_IDENTIFY                 0xec
#define MV_ATA_COMMAND_DEVICE_CONFIG            0xb1
#define MV_ATA_COMMAND_SET_FEATURES             0xef
#define MV_ATA_COMMAND_WRITE_DMA                0xca
#define MV_ATA_COMMAND_WRITE_DMA_EXT            0x35
#define MV_ATA_COMMAND_WRITE_DMA_QUEUED         0xcc
#define MV_ATA_COMMAND_WRITE_DMA_QUEUED_EXT     0x36
#define MV_ATA_COMMAND_WRITE_FPDMA_QUEUED_EXT   0x61
#define MV_ATA_COMMAND_READ_DMA                 0xc8
#define MV_ATA_COMMAND_READ_DMA_EXT             0x25
#define MV_ATA_COMMAND_READ_DMA_QUEUED          0xc7
#define MV_ATA_COMMAND_READ_DMA_QUEUED_EXT      0x26
#define MV_ATA_COMMAND_READ_FPDMA_QUEUED_EXT    0x60
#define MV_ATA_COMMAND_FLUSH_CACHE              0xe7
#define MV_ATA_COMMAND_FLUSH_CACHE_EXT          0xea

#define MV_ATA_COMMAND_PM_READ_REG              0xe4
#define MV_ATA_COMMAND_PM_WRITE_REG             0xe8

#define MV_SATA_GSCR_ID_REG_NUM                 0
#define MV_SATA_GSCR_REVISION_REG_NUM           1
#define MV_SATA_GSCR_INFO_REG_NUM               2
#define MV_SATA_GSCR_ERROR_REG_NUM              32
#define MV_SATA_GSCR_ERROR_ENABLE_REG_NUM       33
#define MV_SATA_GSCR_FEATURES_REG_NUM           64
#define MV_SATA_GSCR_FEATURES_ENABLE_REG_NUM    96


#define MV_SATA_PSCR_SSTATUS_REG_NUM            0
#define MV_SATA_PSCR_SERROR_REG_NUM             1
#define MV_SATA_PSCR_SCONTROL_REG_NUM           2
#define MV_SATA_PSCR_SACTIVE_REG_NUM            3



#define MV_ATA_SET_FEATURES_DISABLE_8_BIT_PIO   0x01
#define MV_ATA_SET_FEATURES_ENABLE_WCACHE       0x02  /* Enable write cache */
#define MV_ATA_SET_FEATURES_TRANSFER            0x03  /* Set transfer mode  */
#define MV_ATA_TRANSFER_UDMA_0                  0x40
#define MV_ATA_TRANSFER_UDMA_1                  0x41
#define MV_ATA_TRANSFER_UDMA_2                  0x42
#define MV_ATA_TRANSFER_UDMA_3                  0x43
#define MV_ATA_TRANSFER_UDMA_4                  0x44
#define MV_ATA_TRANSFER_UDMA_5                  0x45
#define MV_ATA_TRANSFER_UDMA_6                  0x46
#define MV_ATA_TRANSFER_UDMA_7                  0x47
#define MV_ATA_TRANSFER_PIO_SLOW                0x00
#define MV_ATA_TRANSFER_PIO_0                   0x08
#define MV_ATA_TRANSFER_PIO_1                   0x09
#define MV_ATA_TRANSFER_PIO_2                   0x0A
#define MV_ATA_TRANSFER_PIO_3                   0x0B
#define MV_ATA_TRANSFER_PIO_4                   0x0C

/* Enable advanced power management */
#define MV_ATA_SET_FEATURES_ENABLE_APM          0x05

/* Disable media status notification*/
#define MV_ATA_SET_FEATURES_DISABLE_MSN         0x31

/* Disable read look-ahead          */
#define MV_ATA_SET_FEATURES_DISABLE_RLA         0x55

/* Enable release interrupt         */
#define MV_ATA_SET_FEATURES_ENABLE_RI           0x5D

/* Enable SERVICE interrupt         */
#define MV_ATA_SET_FEATURES_ENABLE_SI           0x5E

/* Disable revert power-on defaults */
#define MV_ATA_SET_FEATURES_DISABLE_RPOD        0x66

/* Disable write cache              */
#define MV_ATA_SET_FEATURES_DISABLE_WCACHE      0x82

/* Disable advanced power management*/
#define MV_ATA_SET_FEATURES_DISABLE_APM         0x85

/* Enable media status notification */
#define MV_ATA_SET_FEATURES_ENABLE_MSN          0x95

/* Enable read look-ahead           */
#define MV_ATA_SET_FEATURES_ENABLE_RLA          0xAA

/* Enable revert power-on defaults  */
#define MV_ATA_SET_FEATURES_ENABLE_RPOD         0xCC

/* Disable release interrupt        */
#define MV_ATA_SET_FEATURES_DISABLE_RI          0xDD

/* Disable SERVICE interrupt        */
#define MV_ATA_SET_FEATURES_DISABLE_SI          0xDE

/* Defines for parsing the IDENTIFY command results*/
#define IDEN_SERIAL_NUM_OFFSET                  10
#define IDEN_SERIAL_NUM_SIZE                    (20-10)
#define IDEN_FIRMWARE_OFFSET                    23
#define IDEN_FIRMWARE_SIZE                      (27-23)
#define IDEN_MODEL_OFFSET                       27
#define IDEN_MODEL_SIZE                         (47-27)
#define IDEN_CAPACITY_1_OFFSET                  49
#define IDEN_VALID                              53
#define IDEN_NUM_OF_ADDRESSABLE_SECTORS         60
#define IDEN_PIO_MODE_SPPORTED                  64
#define IDEN_QUEUE_DEPTH                        75
#define IDEN_SATA_CAPABILITIES                  76
#define IDEN_SATA_FEATURES_SUPPORTED            78
#define IDEN_SATA_FEATURES_ENABLED              79
#define IDEN_ATA_VERSION                        80
#define IDEN_SUPPORTED_COMMANDS1                82
#define IDEN_SUPPORTED_COMMANDS2                83
#define IDEN_ENABLED_COMMANDS1                  85
#define IDEN_ENABLED_COMMANDS2                  86
#define IDEN_UDMA_MODE                          88

/* Typedefs    */

/* Structures  */

    typedef struct mvStorageDevRegisters
    {
/* Fields set by CORE driver */
        MV_U8    errorRegister;
        MV_U16   featuresRegister;/*input only*/
        MV_U8    commandRegister;/*input only*/
        MV_U16   sectorCountRegister;
        MV_U16   lbaLowRegister;
        MV_U16   lbaMidRegister;
        MV_U16   lbaHighRegister;
        MV_U8    deviceRegister;
        MV_U8    statusRegister;
    } MV_STORAGE_DEVICE_REGISTERS;

/* Function */

    MV_BOOLEAN mvStorageDevATAExecuteNonUDMACommand(MV_SATA_ADAPTER *pAdapter,
                                                    MV_U8 channelIndex,
                                                    MV_U8   PMPort,
                                                    MV_NON_UDMA_PROTOCOL protocolType,
                                                    MV_BOOLEAN  isEXT,
                                                    MV_U16_PTR bufPtr, MV_U32 count,
                                                    MV_U16 features,
                                                    MV_U16 sectorCount,
                                                    MV_U16 lbaLow, MV_U16 lbaMid,
                                                    MV_U16 lbaHigh, MV_U8 device,
                                                    MV_U8 command);

    MV_BOOLEAN mvStorageDevATAIdentifyDevice(MV_SATA_ADAPTER *pAdapter,
                                             MV_U8 channelIndex,
                                             MV_U8 PMPort,
                                             MV_U16_PTR  identifyDeviceResult
                                            );

    MV_BOOLEAN mvStorageDevATASetFeatures(MV_SATA_ADAPTER *pAdapter,
                                          MV_U8 channelIndex,
                                          MV_U8   PMPort,
                                          MV_U8 subCommand,
                                          MV_U8 subCommandSpecific1,
                                          MV_U8 subCommandSpecific2,
                                          MV_U8 subCommandSpecific3,
                                          MV_U8 subCommandSpecific4);

    MV_BOOLEAN mvStorageDevATAIdleImmediate(MV_SATA_ADAPTER *pAdapter,
                                            MV_U8 channelIndex);

    MV_BOOLEAN mvStorageDevATASoftResetDevice(MV_SATA_ADAPTER *pAdapter,
                                              MV_U8 channelIndex,
                                              MV_U8 PMPort,
                                              MV_STORAGE_DEVICE_REGISTERS *registerStruct
                                             );

/*PM*/

    MV_BOOLEAN mvPMDevReadReg(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                              MV_U8 PMPort, MV_U8 PMReg, MV_U32 *pValue,
                              MV_STORAGE_DEVICE_REGISTERS *registerStruct);

    MV_BOOLEAN  mvPMDevWriteReg(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                                MV_U8 PMPort, MV_U8 PMReg, MV_U32 value,
                                MV_STORAGE_DEVICE_REGISTERS *registerStruct);

    MV_BOOLEAN  mvPMDevEnableStaggeredSpinUp(MV_SATA_ADAPTER *pAdapter,
                                             MV_U8 channelIndex, MV_U8 PMPort);

    MV_BOOLEAN  mvPMDevEnableStaggeredSpinUpAll(MV_SATA_ADAPTER *pAdapter,
                                                MV_U8 channelIndex,
                                                MV_U8 PMNumOfPorts,
                                                MV_U16 *bitmask);

    MV_BOOLEAN mvStorageDevExecutePIO(MV_SATA_ADAPTER *pAdapter,
                                      MV_U8 channelIndex,
                                      MV_U8 PMPort,
                                      MV_NON_UDMA_PROTOCOL protocolType,
                                      MV_BOOLEAN  isEXT, MV_U16_PTR bufPtr,
                                      MV_U32 count,
                                      MV_STORAGE_DEVICE_REGISTERS *pInATARegs,
                                      MV_STORAGE_DEVICE_REGISTERS *pOutATARegs);


    MV_BOOLEAN mvStorageDevATAStartSoftResetDevice(MV_SATA_ADAPTER *pAdapter,
                                                   MV_U8 channelIndex,
                                                   MV_U8 PMPort);

    MV_BOOLEAN mvStorageIsDeviceBsyBitOff(MV_SATA_ADAPTER *pAdapter,
                                          MV_U8 channelIndex,
                                          MV_STORAGE_DEVICE_REGISTERS *registerStruct
                                         );

    MV_BOOLEAN  mvStorageDevSetDeviceType(MV_SATA_ADAPTER *pAdapter, MV_U8 channelIndex,
                                          MV_SATA_DEVICE_TYPE deviceType);

    MV_SATA_DEVICE_TYPE mvStorageDevGetDeviceType(MV_SATA_ADAPTER *pAdapter,
                                                  MV_U8 channelIndex);


#ifdef __cplusplus

}
#endif /* __cplusplus */

#endif /* __INCmvStorageDevh */
