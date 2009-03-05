/*
 *
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/rmi/msgring.h>
#include <asm/rmi/iomap.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/debug.h>
#include <asm/rmi/phnx_user_mac.h>
#include <asm/rmi/sim.h>

#define CH0_CONTROL 8
#define MSGRNG_CODE_DMA 8
#define XLR_DMA_RESP_TIMEOUT 500

#define NEXT_SEQ_NUM(x) ((x->sequence_number + 1) & (MAX_DMA_TRANS_PER_CPU - 1))
#define INC_SEQ_NUM(x) x->sequence_number = \
		((x->sequence_number + 1) & (MAX_DMA_TRANS_PER_CPU - 1))

#define DMA_SLOT_BUSY(x) (x->trans[x->sequence_number].pending)

#define DMA_RESP_PENDING(x, seq) (x->trans[seq].pending)

#define DMA_SLOT_GET(x) (x->trans[x->sequence_number].pending = 1); \
				INC_SEQ_NUM(ctrl);

#define DMA_SLOT_PUT(x, seq) (x->trans[seq].pending = 0)

#define DMA_GET_RESP(x, seq) (x->trans[seq].dma_resp)

#define DMA_PUT_RESP(x, seq, msg) x->trans[seq].dma_resp = msg; \
				x->trans[seq].pending = 0;

#define DMA_DONE(x, seq) (x->trans[seq].pending == 0)

enum dma_msgring_bucket_config {

	DMA_MSG_BUCKET0_SIZE = 0x320,
	DMA_MSG_BUCKET1_SIZE,
	DMA_MSG_BUCKET2_SIZE,
	DMA_MSG_BUCKET3_SIZE,
};

enum dma_msgring_credit_config {

	DMA_CC_CPU0_0                        = 0x380,
	DMA_CC_CPU1_0                        = 0x388,
	DMA_CC_CPU2_0                        = 0x390,
	DMA_CC_CPU3_0                        = 0x398,
	DMA_CC_CPU4_0                        = 0x3a0,
	DMA_CC_CPU5_0                        = 0x3a8,
	DMA_CC_CPU6_0                        = 0x3b0,
	DMA_CC_CPU7_0                        = 0x3b8
};

/* We use 10 bit transaction id in the DMA message to uniquely identify a DMA
   response.
   0-7 indicate a sequence number (0 to 255)
   8-9 bits encode the CPU thread id (0 to 3)
   */
#define MAX_DMA_TRANS_PER_CPU 256
#define XLR_MAX_DMA_LEN_PER_DESC ((1 << 20) - 1) /* 1 MB - 1 */
typedef struct dma_trans {
	int pending;
	uint64_t dma_resp;
}dma_trans_t;

typedef struct xlr_dma_ctrl {
	spinlock_t q_lock;
	int sequence_number;
	dma_trans_t trans[MAX_DMA_TRANS_PER_CPU];
}xlr_dma_ctrl_t;


static int xlr_dma_init_done = 0;
DEFINE_PER_CPU(struct xlr_dma_ctrl, xlr_dma_ctrl) ;

/* DMA message handler - Called from interrupt context */
static void xlr_dma_msgring_handler(int bucket, int size, int code, 
		int stid, struct msgrng_msg *msg, void *data/* ignored */)
{
	int cpu, thr_id, tx_id, seq;
	xlr_dma_ctrl_t *ctrl;

	tx_id = (msg->msg0 >> 48) & 0x3ff;
	thr_id = (tx_id >> 8) & 0x3;
	seq = (tx_id & 0xff);

	cpu = (phoenix_cpu_id() * 4) + thr_id;
	ctrl = &per_cpu(xlr_dma_ctrl, cpu);

	spin_lock(&ctrl->q_lock);
	/* Check if there was a pending request. This can happen if the
	   requestor times out and gives up the request. So in that case
	   do not update the response
	NOTE: One corner case that is not handled here is that when seq no
	wraps around and request was pending for the new one and this response
	was for the old request. This ideally must not happen.
	   */
	if(DMA_RESP_PENDING(ctrl, seq)) {
		DMA_PUT_RESP(ctrl, seq, msg->msg0);
		spin_unlock(&ctrl->q_lock);
		return;
	}
	spin_unlock(&ctrl->q_lock);
	printk("ERROR: Stale response from DMA engine for transaction id %d\n",
			seq);
	return;
}

inline void xlr_build_xfer_msg(struct msgrng_msg *msg, uint64_t src, 
				uint64_t dest, uint32_t len, int tx_id,
				int resp_bkt)
{
	msg->msg0 = (1ULL << 63) | ((uint64_t)len << 40) | 
				(src & 0xffffffffffULL);
	msg->msg1 = (1ULL << 62) | (1ULL << 58) | ((uint64_t)tx_id << 48) |
			((uint64_t)resp_bkt << 40) | (dest & 0xffffffffffULL);
}

/* Returns 0 on success, -1 otherwise */
int xlr_request_dma(uint64_t src, uint64_t dest, uint32_t len)
{
	int thr_id, cpu, i;
	xlr_dma_ctrl_t *ctrl;
	int tx_id, resp_bkt, seq, ret, err;
	struct msgrng_msg  msg, r_msg;
	unsigned long flags;

	thr_id = phoenix_thr_id();
	cpu = (phoenix_cpu_id() * 4) + thr_id;

	/* Driver does not support multiple descriptor DMA yet */
	if(len > XLR_MAX_DMA_LEN_PER_DESC) {
		printk("%s: Cannot do DMA for more than %d bytes\n",
				__FUNCTION__, XLR_MAX_DMA_LEN_PER_DESC);
		return -1;
	}
	if(xlr_dma_init_done == 0) {
		printk("%s: XLR DMA engine is not initialized\n", __FUNCTION__);
		return -1;
	}

	ctrl = &per_cpu(xlr_dma_ctrl, cpu);
	spin_lock_irq(&ctrl->q_lock);
	if(DMA_SLOT_BUSY(ctrl)) {
		printk("%s: No space to enqueue this request\n", __FUNCTION__);
		spin_unlock_irq(&ctrl->q_lock);
		return -1;
	}
	tx_id = (thr_id << 8) | ctrl->sequence_number;
	seq = ctrl->sequence_number;
	DMA_SLOT_GET(ctrl);
	spin_unlock_irq(&ctrl->q_lock);

	/* use bucket 0 of each core as the bucket where response will be 
	   received
	   */
	resp_bkt = phoenix_cpu_id() * 8;

	/* Form the DMA simple xfer request and send to Channel 0 */
	xlr_build_xfer_msg(&msg, src, dest, len, tx_id, resp_bkt);
	msgrng_access_enable(flags);
	if(message_send_retry(2, MSGRNG_CODE_DMA, MSGRNG_STNID_DMA_0, &msg)) {
		printk("
		Message_send failed: Cannot submit DMA request to engine\n");
		msgrng_access_disable(flags);
		spin_lock_irq(&ctrl->q_lock);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irq(&ctrl->q_lock);
		return -1;
	}
	msgrng_access_disable(flags);
	/* wait for the response here */
	for(i=0; i < XLR_DMA_RESP_TIMEOUT; i++) {
		if(DMA_DONE(ctrl, seq)) break;

		udelay(50);
	}
	if(i == XLR_DMA_RESP_TIMEOUT) {
		printk("%s:Did not get response from DMA engine\n", 
						__FUNCTION__);
		spin_lock_irq(&ctrl->q_lock);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irq(&ctrl->q_lock);
		return -1;
	}
	/* Do some error checks */

	r_msg.msg0 = DMA_GET_RESP(ctrl, seq);
	ret = (r_msg.msg0 >> 62) & 0x3;
	err = (r_msg.msg0 >> 60) & 0x3;
	if(ret != 0x3) {
		printk("%s: Bad return code %d from DMA engine\n", __FUNCTION__,
					ret);
		return -1;
	}
	if(err & 0x2) {
		printk("%s:DMA engine reported Message format error\n", 
				__FUNCTION__);
		return -1;
	}
	if(err & 0x1) {
		printk("%s:DMA engine reported Bus error\n", __FUNCTION__);
		return -1;
	}
	return 0;
}

static int xlr_init_dma(void)
{
	int i;
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_DMA_OFFSET);
	xlr_dma_ctrl_t *ctrl;

	for(i=0; i < NR_CPUS; i++) {
		ctrl = &per_cpu(xlr_dma_ctrl, i);
		spin_lock_init(&ctrl->q_lock);
		ctrl->sequence_number = 0;
	}
	/* Register for the Message ring handler */
	if (register_msgring_handler(TX_STN_DMA,
				xlr_dma_msgring_handler, NULL)) {
		printk("Couldn't register DMA msgring handler\n");
		return -1;
	}
	
	/* Use channel 0 for all DMA */
	/* Section Size = 4, RMaxCr=4, WMaxCr =4, En=1; */
	mmio[CH0_CONTROL] = (4<<12) | (4<<8) | (4<<5) | (1<<4);

	/* Configure the bucket sizes */
	mmio[DMA_MSG_BUCKET0_SIZE] = bucket_sizes.bucket[MSGRNG_STNID_DMA_0];
	mmio[DMA_MSG_BUCKET1_SIZE] = bucket_sizes.bucket[MSGRNG_STNID_DMA_1];
	mmio[DMA_MSG_BUCKET2_SIZE] = bucket_sizes.bucket[MSGRNG_STNID_DMA_2];
	mmio[DMA_MSG_BUCKET3_SIZE] = bucket_sizes.bucket[MSGRNG_STNID_DMA_3];

	/* Configure the DMA credits */
	for(i=0;i<128;i++) {
		mmio[DMA_CC_CPU0_0 + i] = cc_table_dma.counters[i>>3][i&0x07];
	}
	xlr_dma_init_done = 1;
	printk("Initialized XLR DMA Controller, Channel 0 \n");
	return 0;
}

#ifdef TEST_DMA
void xlr_dma_test(void)
{
	int i;
	uint8_t *ptr1, *ptr2;
	unsigned long s_jiffy, e_jiffy;

	ptr1 = (uint8_t *)kmalloc(0x1000, GFP_KERNEL);
	ptr2 = (uint8_t *)kmalloc(0x1000, GFP_KERNEL);
	if(!ptr1 || !ptr2){
		printk("DMA test buffer alloc failed\n");
		return;
	}
	memset(ptr1, 0xa5, 0x1000);
	s_jiffy = read_c0_count();
	for(i=0; i < 512; i++) {
		xlr_request_dma((uint64_t)virt_to_phys(ptr1),
				(uint64_t)virt_to_phys(ptr2), 0x1000);
	}
	e_jiffy = read_c0_count();
	if(memcmp(ptr1, ptr2, 0x1000)) {
		printk("DMA Data does not match. Test failed\n");
	}else
		printk("DMA Data Matches. Test Successful\n");

	printk("Start time = %lx end time = %lx\n", s_jiffy, e_jiffy);
	kfree(ptr1);
	kfree(ptr2);
}
#endif

int xlr_dma_init_module(void)
{
	int ret;

	ret = xlr_init_dma();
#ifdef TEST_DMA
	xlr_dma_test();
#endif
	return ret;
}
void xlr_dma_exit_module(void)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_DMA_OFFSET);

	mmio[CH0_CONTROL] = 0;
}

module_init(xlr_dma_init_module);
module_exit(xlr_dma_exit_module);

EXPORT_SYMBOL(xlr_request_dma);
