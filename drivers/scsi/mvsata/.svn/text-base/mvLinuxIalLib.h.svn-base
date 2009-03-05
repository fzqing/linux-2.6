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
* mvLinuxIalLib - Header File for Linux IAL Lib.
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*       None.
*
* FILE REVISION NUMBER:
*       $Revision: 1.1 $
*
*******************************************************************************/
#ifndef __INCmvLinuxIalLibh
#define __INCmvLinuxIalLibh

#include "mvLinuxIalHt.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,4,23)
#define irqreturn_t         void
#define IRQ_RETVAL(foo)
#endif
#define MV_LINUX_ASYNC_TIMER_PERIOD       ((MV_IAL_ASYNC_TIMER_PERIOD * HZ) / 1000)

struct pci_dev;
struct IALAdapter;
struct IALHost;


/* Adapter Initialization */
int mv_ial_lib_allocate_edma_queues(struct IALAdapter *pAdapter);

void mv_ial_lib_free_edma_queues(struct IALAdapter *pAdapter);

int mv_ial_lib_init_channel(struct IALAdapter *pAdapter, MV_U8 channelNum);

void mv_ial_lib_free_channel(struct IALAdapter *pAdapter, MV_U8 channelNum);


/* PRD Table Generation */
#define MV_PRD_TABLE_SIZE                   64 /* 64 entries max in PRD table */


int mv_ial_lib_prd_destroy(struct IALHost *pHost);
int mv_ial_lib_prd_init(struct IALHost *);



int mv_ial_lib_generate_prd(MV_SATA_ADAPTER *pMvSataAdapter, struct scsi_cmnd *SCpnt,
                            MV_SATA_EDMA_PRD_ENTRY **ppPRD_table,
                            dma_addr_t *pPRD_dma_address,
                            unsigned int *pPrd_size, dma_addr_t *pBusaddr);


/* Interrupt Service Routine*/
irqreturn_t mv_ial_lib_int_handler (int irq, void *dev_id, struct pt_regs *regs);


/* Event Notification */
MV_BOOLEAN mv_ial_lib_udma_command_completion_call_back(MV_SATA_ADAPTER *pMvSataAdapter,
                                           MV_U8 channelNum,
                                           MV_COMPLETION_TYPE comp_type,
                                           void *commandId,
                                           MV_U16 responseFlags,
                                           MV_U32 timeStamp,
                                           MV_STORAGE_DEVICE_REGISTERS *registerStruct);

MV_BOOLEAN mv_ial_lib_event_notify(MV_SATA_ADAPTER *pMvSataAdapter, MV_EVENT_TYPE eventType,
                             MV_U32 param1, MV_U32 param2);
void asyncStartTimerFunction(unsigned long data);

/* SCSI done queuing and callback */
void mv_ial_lib_add_done_queue (struct IALAdapter *pAdapter,
                                MV_U8 channel,
                                struct scsi_cmnd   *scsi_cmnd);

struct scsi_cmnd * mv_ial_lib_get_first_cmnd (struct IALAdapter *pAdapter,
                                       MV_U8 channel);

void mv_ial_lib_do_done (struct scsi_cmnd *cmnd);

void mv_ial_block_requests(struct IALAdapter *pAdapter, MV_U8 channelIndex);

#endif /* __INCmvLinuxIalLibh */
