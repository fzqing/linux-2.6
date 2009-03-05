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
* file_name - mvLinuxIALSmart.h
*
* DESCRIPTION: C header file for S.M.A.R.T. features - smartmontools app
*
*
* DEPENDENCIES:
*
*
* FILE REVISION NUMBER:
*       $Revision: 1.1 $
*
*******************************************************************************/
#ifndef __INCmvScsiSmart
#define __INCmvScsiSmart

/*Proprietary opcode to support smartmontools app*/
#define SCSI_OPCODE_MVSATA_SMART            0x0C


/*S.M.A.R.T ATA commands used*/
#ifndef WIN_IDENTIFY
#define WIN_IDENTIFY                        0xEC
#endif
#ifndef WIN_SMART
#define WIN_SMART                           0xB0
#endif
#ifndef SMART_READ_VALUES
#define SMART_READ_VALUES                   0xD0
#endif
#ifndef SMART_READ_THRESHOLDS
#define SMART_READ_THRESHOLDS               0xD1
#endif
#ifndef SMART_AUTOSAVE
#define SMART_AUTOSAVE                      0xD2
#endif
#ifndef SMART_SAVE
#define SMART_SAVE                          0xD3
#endif
#ifndef SMART_IMMEDIATE_OFFLINE
#define SMART_IMMEDIATE_OFFLINE             0xD4
#endif
#ifndef SMART_READ_LOG_SECTOR
#define SMART_READ_LOG_SECTOR               0xD5
#endif
#ifndef SMART_WRITE_LOG_SECTOR
#define SMART_WRITE_LOG_SECTOR              0xD6
#endif
/* The following is obsolete -- don't use it!*/
#ifndef SMART_WRITE_THRESHOLDS
#define SMART_WRITE_THRESHOLDS              0xD7
#endif
#ifndef SMART_ENABLE
#define SMART_ENABLE                        0xD8
#endif
#ifndef SMART_DISABLE
#define SMART_DISABLE                       0xD9
#endif
#ifndef SMART_STATUS
#define SMART_STATUS                        0xDA
#endif

/* The following is also marked obsolete in ATA-5*/
#ifndef SMART_AUTO_OFFLINE
#define SMART_AUTO_OFFLINE                  0xDB
#endif

/*Definitions of S.M.A.R.T. command buffer offsets*/
#define SMART_BUF_COMMAND_OFFSET                0
#define SMART_BUF_LBALOW_OFFSET                 1
#define SMART_BUF_FEATURES_OFFSET               2
#define SMART_BUF_SECTORCOUNT_OFFSET            3
#define SMART_BUF_LBAMID_OFFSET                 4
#define SMART_BUF_LBAHIGH_OFFSET                5
#define SMART_BUF_DEVICE_OFFSET                 6
#define SMART_BUF_ERROR_OFFSET                  7


MV_SCSI_COMMAND_STATUS_TYPE  mvScsiAtaSendSmartCommand
                (IN  MV_SATA_ADAPTER* pSataAdapter,
                 IN  MV_SATA_SCSI_CMD_BLOCK *pScb);


#endif
