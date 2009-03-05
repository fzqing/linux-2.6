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

#ifndef __INCmvIALTWSIh
#define __INCmvIALTWSIh

/* includes */
#include "mvRegs.h"


#undef DEBUG

#ifdef DEBUG
    #define PRINT_DBG printf
#else
    #define PRINT_DBG
#endif



/* defines */
#define TWSI_TIMEOUT_VALUE               2000000
#define TWSI_ENABLE                      MV_BIT6
#define TWSI_INT_ENABLE                  MV_BIT7
#define TWSI_ACK                         MV_BIT2
#define TWSI_INT_FLAG                    MV_BIT3
#define TWSI_STOP_BIT                    MV_BIT4
#define TWSI_START_BIT                   MV_BIT5
#define TWSI_READ                        MV_BIT0
#define TWSI_EEPROM_DELAY                10 /* Mili sec */
#define TWSI_10BIT_ADDR                  MV_BIT31

/* Error codes */
#define TWSI_TIME_OUT_ERROR              0xFF
#define TWSI_NO_DEVICE_WITH_SUCH_ADDR    0x01
#define TWSI_GENERAL_ERROR               0x02
#define TWSI_NO_ERROR                    0x03
#define TWSI_INT_FLAG_STUCK_AT_0         0x04

/* TWSI status codes */
/*  ShomvRtcuts-
    RECEIVED    -> REC
    TRANSMITED  -> TRA
    MASTER      -> MAS
    SLAVE       -> SLA
    ACKNOWLEDGE -> ACK
    ARBITRATION -> ARB
    ADDR        -> ADDR
*/

/****************************************/
/* TWSI Registers                        */
/****************************************/

#define TWSI_SLAVE_ADDR                                      0x11000UL
#define TWSI_EXTENDED_SLAVE_ADDR                             0x11010UL
#define TWSI_DATA                                            0x11004UL
#define TWSI_CONTROL                                         0x11008UL
#define TWSI_STATUS_BAUDE_RATE                               0x1100CUL
#define TWSI_SOFT_RESET                                      0x1101cUL


#define TWSI_BUS_ERROR                                                       0X00
#define TWSI_START_CONDITION_TRA                                             0X08
#define TWSI_REPEATED_START_CONDITION_TRA                                    0X10
#define TWSI_ADDR_PLUS_WRITE_BIT_TRA_ACK_REC                                 0X18
#define TWSI_ADDR_PLUS_WRITE_BIT_TRA_ACK_NOT_REC                             0X20
#define TWSI_MAS_TRAN_DATA_BYTE_ACK_REC                                      0X28
#define TWSI_MAS_TRAN_DATA_BYTE_ACK_NOT_REC                                  0X30
#define TWSI_MAS_LOST_ARB_DURING_ADDR_OR_DATA_TRA                            0X38
#define TWSI_ADDR_PLUS_READ_BIT_TRA_ACK_REC                                  0X40
#define TWSI_ADDR_PLUS_READ_BIT_TRA_ACK_NOT_REC                              0X48
#define TWSI_MAS_REC_READ_DATA_ACK_TRA                                       0X50
#define TWSI_MAS_REC_READ_DATA_ACK_NOT_TRA                                   0X58
#define TWSI_SLA_REC_ADDR_PLUS_WRITE_BIT_ACK_TRA                             0X60
#define TWSI_MAS_LOST_ARB_DURING_ADDR_TRA_ADDR_IS_TARGETED_TO_SLA_ACK_TRA_W  0X68
#define TWSI_GENERAL_CALL_REC_ACK_TRA                                        0X70
#define TWSI_MAS_LOST_ARB_DURING_ADDR_TRA_GENERAL_CALL_ADDR_REC_ACK_TRA      0X78
#define TWSI_SLA_REC_WRITE_DATA_AFTER_REC_SLA_ADDR_ACK_TRAN                  0X80
#define TWSI_SLA_REC_WRITE_DATA_AFTER_REC_SLA_ADDR_ACK_NOT_TRAN              0X88
#define TWSI_SLA_REC_WRITE_DATA_AFTER_REC_GENERAL_CALL_ACK_TRAN              0X90
#define TWSI_SLA_REC_WRITE_DATA_AFTER_REC_GENERAL_CALL_ACK_NOT_TRAN          0X98
#define TWSI_SLA_REC_STOP_OR_REPEATED_START_CONDITION                        0XA0
#define TWSI_SLA_REC_ADDR_PLUS_READ_BIT_ACK_TRA                              0XA8
#define TWSI_MAS_LOST_ARB_DURING_ADDR_TRA_ADDR_IS_TARGETED_TO_SLA_ACK_TRA_R  0XB0
#define TWSI_SLA_TRA_READ_DATA_ACK_REC                                       0XB8
#define TWSI_SLA_TRA_READ_DATA_ACK_NOT_REC                                   0XC0
#define TWSI_SLA_TRA_LAST_READ_DATA_ACK_REC                                  0XC8
#define TWSI_SECOND_ADDR_PLUS_WRITE_BIT_TRA_ACK_REC                          0XD0
#define TWSI_SECOND_ADDR_PLUS_WRITE_BIT_TRA_ACK_NOT_REC                      0XD8
#define TWSI_SECOND_ADDR_PLUS_READ_BIT_TRA_ACK_REC                           0XE0
#define TWSI_SECOND_ADDR_PLUS_READ_BIT_TRA_ACK_NOT_REC                       0XE8
#define TWSI_NO_RELEVANT_STATUS_INTERRUPT_FLAG_IS_KEPT_0                     0XF8

/* typedefs */

MV_BOOLEAN mvSataTWSIMasterInit(MV_SATA_ADAPTER *pAdapter);

MV_BOOLEAN mvSataTWSIMasterEEPROMRead(MV_SATA_ADAPTER *pAdapter,
                                      MV_U8 deviceAddress,
                                      MV_U16 address,
                                      MV_U8_PTR data,
                                      MV_BOOLEAN addrRange);

MV_BOOLEAN mvSataTWSIMasterEEPROMWrite(MV_SATA_ADAPTER *pAdapter,
                                       MV_U8    deviceAddress,
                                       MV_U8     data,
                                       MV_U16    address,
                                       MV_BOOLEAN  addrRange);

#endif /* __INCmvIALTWSIh */
