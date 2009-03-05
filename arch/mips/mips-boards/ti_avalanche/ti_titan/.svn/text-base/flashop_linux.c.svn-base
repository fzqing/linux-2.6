/*
 * Copyright (C) 2006 Texas Instruments Inc.
 *
 * Author: Manish Lachwani (mlachwani@mvista.com)
 * Copyright (C) 2006 Montavista Software Inc.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
	
#include "flashop.h"
	
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/mtd/mtd.h>
#include <linux/ctype.h>
#include <linux/module.h>
	
int    FWBopen=0;
	
/* This function returns back a 0 if failure - no valid flash detected */
int FWBGet_flash_type()
{
	return(1);
}
	
int FWBUnLock(unsigned int adr,int size)
{
	struct mtd_info * mtd;
	int ret;
	
	mtd = get_mtd_device(0, 3);
	if(!mtd) {
		printk(KERN_ERR "Failed to access the environment variable region:  mtd3\n");
		return 0;
	}
	ret = mtd->unlock(mtd, 0, mtd->erasesize);
	
	put_mtd_device(mtd);
	
	if (ret)
		printk ("Flash Unlock Block operation Failed: ret=%08x\n", ret);
	
	return(ret==0);
}
	
	
int FWBLock(unsigned int from,int size)
{
	struct mtd_info * mtd;
	int ret;
	
	if (size == 0) 
		return(1);
	
	mtd = get_mtd_device(0, 3);
	
	if(!mtd) {
		printk(KERN_ERR "Failed to access the environment variable region:  mtd3\n");
		return 0;
	}
	ret = mtd->lock(mtd, 0, mtd->erasesize);
	
	put_mtd_device(mtd);
	
	if (ret)
		printk ("Flash Lock Block operation Failed: ret=%08x\n", ret);
	
	return(ret==0);
}

static int  flash_mtd_erase_callback (struct erase_info * instr)
{
	wake_up((wait_queue_head_t *)instr->priv); 
	
	return 1;
}
	
	
int FWBErase(unsigned int base,int size, int verbose)
{
	struct erase_info * instr; 
	struct mtd_info * mtd;
	int ret=0;
	DECLARE_WAITQUEUE(wait,current);
	wait_queue_head_t wait_q;    
	
	base -= GetEnvBaseAndSize(0);
	
	if (!FWBopen)
		return(0);
	
	mtd=get_mtd_device(0, 3);
	if(!mtd) {
		printk(KERN_ERR "Failed to access the environment variable region: mtd3\n");
		return 0;
	}
	
	init_waitqueue_head(&wait_q);
	
	instr=kmalloc(sizeof(struct erase_info), GFP_KERNEL);
	if (!instr) 
		return -ENOMEM;
	memset(instr,0,sizeof(struct erase_info));
	
	instr->mtd=mtd;
	instr->addr=base;
	instr->len=size;
	instr->priv=(u_long)&wait_q;
	instr->callback=flash_mtd_erase_callback;
	
	mtd->unlock(mtd, 0, mtd->erasesize);
	
	ret=mtd->erase(mtd, instr);
	
	mtd->lock(mtd, 0, mtd->erasesize);
	
	if( !ret ) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue( &wait_q, &wait);
		if( instr->state!=MTD_ERASE_DONE &&
			instr->state!=MTD_ERASE_FAILED) {
			schedule();
		}
	
		remove_wait_queue(&wait_q, &wait);
		set_current_state(TASK_RUNNING);
	
		ret=(instr->state==MTD_ERASE_FAILED)?-EIO:0;
	} else {
		printk(KERN_ERR "Failed to erase mtd, region [0x%x,0x%x]\n",
		instr->addr,instr->len);
	}
	
	put_mtd_device(mtd);
	
	if (ret)
		printk ("Flash Erase Block operation Failed: ret=%08x\n", ret);
	
	kfree(instr);
	return(ret==0);
}
	
int FWBOpen(unsigned int base)
{
	FWBopen=1;
	return(1);
}
	
int FWBWriteByte(unsigned int adr, char cVal)
{
	struct mtd_info * mtd;
		int ret,ret1,ret2, num = 1 ;
	off_t   offset;
	char  new;
	
	if (!FWBopen)
	return(0);
	
	mtd=get_mtd_device(0, 3);
	if(!mtd) {
		printk(KERN_ERR "Failed to access the environment variable region: mtd3\n");
		return 0;
	}
	offset = adr - GetEnvBaseAndSize(0);
	mtd->unlock(mtd, 0, mtd->erasesize);
	
	ret1 = MTD_WRITE(mtd, offset, num, &ret, &cVal);
	
	/* Dummy read operation */
	MTD_READ(mtd, offset, num, &ret2, &new);
	
	put_mtd_device(mtd);
	if (num != ret)
		printk ("Flash Write Byte operation Failed: ret=%08x\n", ret);
	
	return(ret1==0);
}
	
int FWBClose(void)
{
	if (!FWBopen) 
		return(0);
	
	FWBopen=0;
	return 1;
}
