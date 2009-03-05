/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/mman.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/bitfield.h>

#include "m2d_drv.h"

//#define DEBUG
//#define DEBUG_CACHE_COHERENCY
//#define CONFIG_M2D_RINGBUF_IN_SRAM

#ifdef CONFIG_M2D_RINGBUF_IN_SRAM
#include <asm/arch/imm.h>
#endif

#define CONFIG_M2D_USE_PROC

#define m2d_warn(fmt,arg...)	printk(KERN_WARNING "m2d: " fmt, ##arg)
#define m2d_debug(fmt,arg...)	printk(KERN_DEBUG   "m2d: " fmt, ##arg)

typedef enum {
	GC_BUFFI_ADDR_SRC0 = 0x0,
	GC_BUFFI_ADDR_SRC1 = 0x1,
	GC_BUFFI_ADDR_DST0 = 0x8,
	GC_BUFFI_ADDR_DST1 = 0x9,
	GC_BUFFI_ADDR_DST2 = 0xa,
} GCU_BUFFI_ADDR;

static int m2d_major = 253;

struct gcu_buff_info;
struct m2d_device;
struct m2d_context;

/***************************************************************************
 * 2 D   G C U   R e g i s t e r   D e f i n i t i o n s
 **************************************************************************/
#define GCU_REGS_BASE		(0x54000000)

#define GCCR_MASK		(0x000003ff)
#define GCCR_STOP		(0x00000010)
#define GCCR_ABORT		(0x00000040)
#define GCCR_BP_RST		(0x00000100)
#define GCCR_SYNC_CLR		(0x00000200)

#define GCIECR_MASK		(0x000001ff)
#define GCIECR_EOB_INTEN	(0x00000001)
#define GCIECR_IN_INTEN		(0x00000002)
#define GCIECR_BF_INTEN		(0x00000004)
#define GCIECR_IOP_INTEN	(0x00000008)
#define GCIECR_IIN_INTEN	(0x00000010)
#define GCIECR_EEOB_INTEN	(0x00000020)
#define GCIECR_PF_INTEN		(0x00000040)
#define GCIECR_STOP_INTEN	(0x00000080)
#define GCIECR_FLAG_INTEN	(0x00000100)

#define GCISCR_MASK		(0x000001ff)
#define GCISCR_EOB_INTST	(0x00000001)
#define GCISCR_IN_INTST		(0x00000002)
#define GCISCR_BF_INTST		(0x00000004)
#define GCISCR_IOP_INTST	(0x00000008)
#define GCISCR_IIN_INTST	(0x00000010)
#define GCISCR_EEOB_INTST	(0x00000020)
#define GCISCR_PF_INTST		(0x00000040)
#define GCISCR_STOP_INTST	(0x00000080)
#define GCISCR_FLAG_INTST	(0x00000100)

typedef struct gcu_regs_s {
	/* Miscellaneous Control and Interrupt Information */
	uint32_t gccr;		/* Configuration Register */
	uint32_t gciscr;	/* Interrupt Status Control Register */
	uint32_t gciecr;	/* Interrupt Enable Control Register */
	uint32_t gcnopid;	/* NOP ID From Instruction Stream Register */
	uint32_t gcalphaset;	/* Default Alpha value Control Register */
	uint32_t gctset;	/* Default Transparecy Value Control Register */
	uint32_t gcflags;	/* ALU Ooperations Flags Status Control Register */

	/* Reserved 0x5400001C */
	uint32_t res0[1];

	/* Ring Buffer Information */
	uint32_t gcrbbr;	/* Ring Buffer Base Address Register */
	uint32_t gcrblr;	/* Ring Buffer Length Register */
	uint32_t gcrbhr;	/* Ring Buffer Head Register */
	uint32_t gcrbtr;	/* Ring Buffer Tail Register */
	uint32_t gcrbexhr;	/* Ring Buffer Execution Head Register */

	/* Reserved 0x54000034-0x5400003C */
	uint32_t res1[3];

	/* Batch Buffer Information */
	uint32_t gcbbbr;	/* Batch Buffer Base Address Register */
	uint32_t gcbbhr;	/* Batch Buffer Head Register */
	uint32_t gcbbexhr;	/* Batch Buffer Execution Head Register */

	/* Reserved 0x5400004C - 0x5400005C */
	uint32_t res2[5];

	/* Destination 0 Information */
	uint32_t gcd0br;	/* Destination 0 Base Address Register */
	uint32_t gcd0stp;	/* Destination 0 Step Size Register */
	uint32_t gcd0str;	/* Destination 0 Stride Size Register */
	uint32_t gcd0pf;	/* Destination 0 Pixel Type Register */

	/* Destination 1 Information */
	uint32_t gcd1br;	/* Destination 1 Base Address Register */
	uint32_t gcd1stp;	/* Destination 1 Step Size Register */
	uint32_t gcd1str;	/* Destination 1 Stride Size Register */
	uint32_t gcd1pf;	/* Destination 1 Pixel Type Register */

	/* Destination 2 Information */
	uint32_t gcd2br;	/* Destination 2 Base Address Register */
	uint32_t gcd2stp;	/* Destination 2 Step Size Register */
	uint32_t gcd2str;	/* Destination 2 Stride Size Register */
	uint32_t gcd2pf;	/* Destination 2 Pixel Type Register */

	/* Reserved 0x54000090-0x540000DC */
	uint32_t res3[20];

	/* Source 0 Information */
	uint32_t gcs0br;	/* Source 0 Base Address Register */
	uint32_t gcs0stp;	/* Source 0 Step Size Register */
	uint32_t gcs0str;	/* Source 0 Stride Size Register */
	uint32_t gcs0pf;	/* Source 0 Pixel Type Register */

	/* Source 1 Information */
	uint32_t gcs1br;	/* Source 1 Base Address Register */
	uint32_t gcs1stp;	/* Source 1 Step Size Register */
	uint32_t gcs1str;	/* Source 1 Stride Size Register */
	uint32_t gcs1pf;	/* Source 1 Pixel Type Register */

	/* Resvered 0x54000100-0x5400015C */
	uint32_t res4[24];

	/* Pixel ALU Scratch Registers */
	uint32_t gcsc0wd0;	/* Pixel ALU Scratch Register 0 Word 0 */
	uint32_t gcsc0wd1;	/* Pixel ALU Scratch Register 0 Word 1 */
	uint32_t gcsc1wd0;	/* Pixel ALU Scratch Register 1 Word 0 */
	uint32_t gcsc1wd1;	/* Pixel ALU Scratch Register 1 Word 1 */
	uint32_t gcsc2wd0;	/* Pixel ALU Scratch Register 2 Word 0 */
	uint32_t gcsc2wd1;	/* Pixel ALU Scratch Register 2 Word 1 */
	uint32_t gcsc3wd0;	/* Pixel ALU Scratch Register 3 Word 0 */
	uint32_t gcsc3wd1;	/* Pixel ALU Scratch Register 3 Word 1 */
	uint32_t gcsc4wd0;	/* Pixel ALU Scratch Register 4 Word 0 */
	uint32_t gcsc4wd1;	/* Pixel ALU Scratch Register 5 Word 1 */
	uint32_t gcsc5wd0;	/* Pixel ALU Scratch Register 5 Word 0 */
	uint32_t gcsc5wd1;	/* Pixel ALU Scratch Register 5 Word 1 */
	uint32_t gcsc6wd0;	/* Pixel ALU Scratch Register 6 Word 0 */
	uint32_t gcsc6wd1;	/* Pixel ALU Scratch Register 6 Word 1 */
	uint32_t gcsc7wd0;	/* Pixel ALU Scratch Register 7 Word 0 */
	uint32_t gcsc7wd1;	/* Pixel ALU Scratch Register 7 Word 1 */

	/* Reserved 0x540001A0~0x540001DC */
	uint32_t res5[16];

	/* Abort Bad Address Storage Registers */
	uint32_t gccabaddr;
	uint32_t gctabaddr;
	uint32_t gcmabaddr;
} gcu_regs_t;

struct gcu_buff_info {
	unsigned long base;
	unsigned long stride;
	unsigned long step;
	unsigned long format;
};

typedef enum {
	GCU_STATE_UNKNOWN = 0,
	GCU_STATE_IDLE = 1,
	GCU_STATE_EXEC = 2,
} GCU_STATE;

struct m2d_device {
	struct device *dev;

#ifdef CONFIG_M2D_RINGBUF_IN_SRAM
	u32 immid;
#endif

	unsigned long primary_fb_base;
	unsigned long primary_fb_len;

	unsigned long gcu_regs_base;
	unsigned long gcu_regs_len;
	gcu_regs_t *gcu_regs;

	wait_queue_head_t wait_eeob;

	struct semaphore submit_sem;
	struct m2d_context *last_context;

	unsigned long *ring_addr;
	unsigned long ring_size;
	unsigned long ring_addr_dma;
	unsigned long ring_tail_dma;

	spinlock_t context_lock;
	spinlock_t regs_lock;
	spinlock_t gcu_lock;


	struct list_head context_list;
	unsigned long context_count;

	unsigned long total_gmem_size;
};

struct m2d_gmem {
	struct list_head gmem_list;
	atomic_t gmem_count;
	struct page *gmem_pages;
	size_t gmem_size;
	unsigned long gmem_virt_addr;
	unsigned long gmem_phys_addr;
	struct m2d_device *gmem_dev;
};

struct m2d_mem_map {
	struct list_head mmap_list;
	struct m2d_gmem *mmap_gmem;
	unsigned long mmap_type;
	unsigned long mmap_addr;
	unsigned long mmap_pgoff;
	unsigned long mmap_size;
};

enum {
	GCU_ERR_NONE = 0,	/* no error */
	GCU_ERR_ILLEGAL_PF,	/* illegal pixel format */
	GCU_ERR_ILLEGAL_INSTRUCTION,	/* illegal instruction */
	GCU_ERR_ILLEGAL_OPERATION,	/* illegal operation */
	GCU_ERR_PIXEL_ALU,	/* event occurs in ALU */
} GCU_ERR_TYPE;

struct m2d_context {
	struct list_head list;
	struct m2d_device *dev;
	struct task_struct *task;

	struct list_head mem_map_list;
	struct m2d_mem_map *mem_map;

	unsigned long total_gmem_size;

	unsigned int err_iin;
	unsigned int err_iop;
	unsigned int err_ipf;

	/* graphics memory allocated for application usage, will be memory
	 * mapped into the user space.  Buffers are allocated within this
	 * area and can be accessed by both application and GCU */

	/* hardware context, will be saved/restored when context switches */
	struct gcu_buff_info srcbuf0;
	struct gcu_buff_info srcbuf1;
	struct gcu_buff_info dstbuf0;
	struct gcu_buff_info dstbuf1;
	struct gcu_buff_info dstbuf2;
	unsigned long alpha_set;
	unsigned long trans_set;
	unsigned long buffer_mode;
};

/****************************************************************************
 * F u n c t i o n   P r o t o t y p e s
 ***************************************************************************/

static struct m2d_context *m2d_create_context(struct m2d_device *dev);
static void m2d_free_context(struct m2d_context *ctx);
static void m2d_switch_context(struct m2d_context *ctx);

static struct m2d_gmem *m2d_alloc_gmem(struct m2d_context *ctx, size_t rsize);
static void m2d_free_gmem(struct m2d_context *ctx, struct m2d_gmem *gmem);

static int m2d_request_mem(struct file *file, struct m2d_context *ctx,
			   struct m2d_mem_req *req);
static void m2d_release_mem(struct m2d_context *ctx, unsigned long mmap_addr);

static int m2d_submit(struct m2d_context *ctx, struct m2d_submit_req *req);
static int m2d_append(struct m2d_device *dev, void *usrbuf, size_t len);
static int m2d_kick(struct m2d_device *dev, int sync);
static int m2d_sync(struct m2d_context *ctx);
static int m2d_wait_for_eeob(struct m2d_device *dev);

static int m2d_release(struct inode *inode, struct file *file);
static int m2d_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);
static int m2d_mmap(struct file *file, struct vm_area_struct *vma);
static int m2d_fsync(struct file *file, struct dentry *dentry, int datasync);

static void m2d_init_proc(void);
static int m2d_read_proc(char *buf, char **start, off_t off,
			 int count, int *eof, void *data);
static void m2d_free_proc(void);

static int m2d_init_device(struct m2d_device *dev);
static void m2d_free_device(struct m2d_device *dev);
static void m2d_reset_device(struct m2d_device *dev);

static int m2d_gcu_irq(int irq, void *dev_id, struct pt_regs *regs);
static int m2d_interrupt_eeob(struct m2d_device *, volatile gcu_regs_t *);
static int m2d_interrupt_err(struct m2d_device *, volatile gcu_regs_t *);

static int m2d_open(struct inode *inode, struct file *file);

/****************************************************************************
 * Global variables
 ***************************************************************************/
static struct m2d_device m2d_dev;
static unsigned int gcu_clock_counter;

/****************************************************************************
 * Clock Management of 2D GCU
 ***************************************************************************/
static void enable_gcu_clock(struct m2d_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->gcu_lock, flags);
	if (!gcu_clock_counter++)
		pxa_set_cken(CKEN_GRAPHICS, 1);
	spin_unlock_irqrestore(&dev->gcu_lock, flags);
}

static void disable_gcu_clock(struct m2d_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->gcu_lock, flags);
	if (!--gcu_clock_counter)
		pxa_set_cken(CKEN_GRAPHICS, 0);
	spin_unlock_irqrestore(&dev->gcu_lock, flags);
}

/****************************************************************************
 * D e b u g   H e l p   F u n c t i o n s
 ***************************************************************************/

#ifdef DEBUG
static void dump_gcu_status(volatile gcu_regs_t * gr)
{
	m2d_debug("GC Status: GCCR=0x%08x, GCISCR=0x%08x, GCFLAGS=0x%08x\n",
		  gr->gccr, gr->gciscr, gr->gcflags);
}

static void dump_gcu_buffi(volatile gcu_regs_t * gr, GCU_BUFFI_ADDR buff_addr)
{
	switch (buff_addr) {
	case GC_BUFFI_ADDR_SRC0:
		printk
		    ("GC_BUFFI_S0: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		     gr->gcs0br, gr->gcs0stp, gr->gcs0str, gr->gcs0pf);
		break;

	case GC_BUFFI_ADDR_SRC1:
		printk
		    ("GC_BUFFI_S1: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		     gr->gcs1br, gr->gcs1stp, gr->gcs1str, gr->gcs1pf);
		break;

	case GC_BUFFI_ADDR_DST0:
		printk
		    ("GC_BUFFI_D0: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		     gr->gcd0br, gr->gcd0stp, gr->gcd0str, gr->gcd0pf);
		break;

	case GC_BUFFI_ADDR_DST1:
		printk
		    ("GC_BUFFI_D1: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		     gr->gcd1br, gr->gcd1stp, gr->gcd1str, gr->gcd1pf);
		break;

	case GC_BUFFI_ADDR_DST2:
		printk
		    ("GC_BUFFI_D2: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		     gr->gcd2br, gr->gcd2stp, gr->gcd2str, gr->gcd2pf);
		break;

	default:
		break;
	}
}

static void dump_content(void *start, int len)
{
	unsigned long *addr = (unsigned long *)start;
	int i;

	for (i = 0; i < (len >> 2); i++) {
		if (i % 4 == 0)
			printk("0x%p: ", addr);
		printk("0x%08lx ", *addr++);
		if (i % 4 == 3)
			printk("\n");
	}
	if (i % 4 != 3)
		printk("\n");
	return;
}
#else
#define dump_gcu_status(a)
#define dump_gcu_buffi(a,b)
#define dump_content(a,b)
#endif

/* dump the GCU ring buffer registers and the instructions nearby the
 * current execution header pointer
 */
static void dump_gcu_ring(struct m2d_device *dev, volatile gcu_regs_t * gr)
{
	unsigned long *addr, len = 0x80;
	int i, offset = (gr->gcrbexhr - gr->gcrbbr);

	if (offset < 0)
		offset = 0;
	addr = (unsigned long *)((unsigned long)(dev->ring_addr) + offset);

	printk("RING: Base=0x%08x, Head=0x%08x, Tail=0x%08x, EH=0x%08x\n",
	       gr->gcrbbr, gr->gcrbhr, gr->gcrbtr, gr->gcrbexhr);

	for (i = 0; i < len; i += 4) {
		if (i % 16 == 0)
			printk("\n0x%lx: ", dev->ring_addr_dma + offset + i);
		printk(" 0x%08lx", *addr++);
	}
	printk("\n");
	return;
}

#ifdef DEBUG_CACHE_COHERENCY
static void dump_page_table_entries(unsigned long start,
				    unsigned long end, struct m2d_gmem *gmem)
{
	unsigned long addr, pfn, size;
	struct page *start_page;
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;

	if (gmem == NULL)
		return;

	start_page = gmem->gmem_pages;
	size = end - start;
	addr = phys_to_virt(page_to_phys(start_page));
	end = addr + size;
	pfn = page_to_pfn(start_page);
	printk("\nPGD for kernel mapping:");
	do {
		pgd = pgd_offset_k(addr);
		printk("pgd[0]=0x%08lx, pgd[1]=0x%08lx\n",
		       (*pgd)[0], (*pgd)[1]);
		pfn += PTRS_PER_PGD;
		addr += (1 << PGDIR_SHIFT);
	} while (addr < end);

	addr = start;
	end = start + size;
	pfn = page_to_pfn(start_page);
	printk("PTE for user mapping:\n");
	do {
		pmd = (pmd_t *) pgd_offset(current->mm, addr);
		pte = (pte_t *) pte_offset_map(pmd, addr);
		printk("page #%06lx pte=0x%08lx, pte_linux=0x%08lx\n",
		       pfn, *(pte - PTRS_PER_PTE), *pte);
		pfn++;
		addr += PAGE_SIZE;
	} while (addr < end);
}
#endif				/* DEBUG_CACHE_COHERENCY */

/****************************************************************************
 * Graphics Context
 ***************************************************************************/

static struct m2d_context *m2d_create_context(struct m2d_device *dev)
{
	struct m2d_context *ctx;
	unsigned long flags;

	if (dev == NULL)
		return NULL;

	ctx = kmalloc(sizeof(struct m2d_context), GFP_KERNEL);
	if (ctx == NULL)
		return NULL;

	memset(ctx, 0, sizeof(struct m2d_context));

	ctx->dev = dev;
	ctx->task = current;
	ctx->alpha_set = 0;
	ctx->trans_set = 0;
	ctx->buffer_mode = 0x08;

	INIT_LIST_HEAD(&ctx->mem_map_list);

	spin_lock_irqsave(&dev->context_lock, flags);
	list_add_tail(&ctx->list, &dev->context_list);
	dev->context_count++;
	spin_unlock_irqrestore(&dev->context_lock, flags);

	return ctx;
}

static void m2d_free_context(struct m2d_context *ctx)
{
	struct m2d_mem_map *mmap, *m;
	struct m2d_device *dev;
	unsigned long flags;

	if (ctx == NULL || (dev = ctx->dev) == NULL)
		return;

	list_for_each_entry_safe(mmap, m, &ctx->mem_map_list, mmap_list) {
		if (mmap->mmap_gmem)
			m2d_free_gmem(ctx, mmap->mmap_gmem);

		list_del(&mmap->mmap_list);
		kfree(mmap);
	}

	spin_lock_irqsave(&dev->context_lock, flags);
	list_del(&ctx->list);
	dev->context_count--;
	spin_unlock_irqrestore(&dev->context_lock, flags);

	kfree(ctx);
}

#define SAVE_BUFFI(br, buffi)			\
	do {					\
		(buffi)->base	= (br)[0];	\
		(buffi)->step	= (br)[1];	\
		(buffi)->stride	= (br)[2];	\
		(buffi)->format	= (br)[3];	\
	} while (0);

static inline void m2d_save_context(struct m2d_context *ctx,
				    volatile gcu_regs_t * gcregs)
{
	SAVE_BUFFI(&gcregs->gcs0br, &ctx->srcbuf0);
	SAVE_BUFFI(&gcregs->gcs1br, &ctx->srcbuf1);
	SAVE_BUFFI(&gcregs->gcd0br, &ctx->dstbuf0);
	SAVE_BUFFI(&gcregs->gcd1br, &ctx->dstbuf1);
	SAVE_BUFFI(&gcregs->gcd2br, &ctx->dstbuf2);

	ctx->buffer_mode = gcregs->gccr | 0x0000000f;
	ctx->alpha_set = gcregs->gcalphaset;
	ctx->trans_set = gcregs->gctset;

	return;
}

#define RESTORE_BUFFI(br, buffi)		\
	do {					\
		(br)[0]	= (buffi)->base;	\
		(br)[1]	= (buffi)->step;	\
		(br)[2]	= (buffi)->stride;	\
		(br)[3]	= (buffi)->format;	\
	} while (0);

static inline void m2d_restore_context(struct m2d_context *ctx,
				       volatile gcu_regs_t * gr)
{
	RESTORE_BUFFI(&gr->gcs0br, &ctx->srcbuf0);
	RESTORE_BUFFI(&gr->gcs1br, &ctx->srcbuf1);
	RESTORE_BUFFI(&gr->gcd0br, &ctx->dstbuf0);
	RESTORE_BUFFI(&gr->gcd1br, &ctx->dstbuf1);
	RESTORE_BUFFI(&gr->gcd2br, &ctx->dstbuf2);

	gr->gccr = (gr->gccr & 0xfffffff0) | ctx->buffer_mode;
	gr->gcalphaset = ctx->alpha_set;
	gr->gctset = ctx->trans_set;
}

/**
 * @brief m2d_switch_context() should be only called when safe.
 * Switching when the GCU is executing is not a good idea.
 */
static void m2d_switch_context(struct m2d_context *ctx)
{
	struct m2d_device *dev = ctx->dev;
	volatile gcu_regs_t *gr = dev->gcu_regs;
	unsigned long flags;

	spin_lock_irqsave(&dev->context_lock, flags);

	if (dev->last_context)
		m2d_save_context(dev->last_context, gr);

	if (ctx)
		m2d_restore_context(ctx, gr);

	dev->last_context = ctx;

	spin_unlock_irqrestore(&dev->context_lock, flags);
}

/****************************************************************************
 * G r a p h i c s   M e m o r y   M a n a g e m e n t
 ***************************************************************************/

/**
 * @brief m2d_alloc_gmem() will allocate contiguous system memory used by
 * 2D Graphics Controller.
 */
static struct m2d_gmem *m2d_alloc_gmem(struct m2d_context *ctx, size_t rsize)
{
	struct m2d_device *dev = ctx->dev;
	unsigned long size, order, kaddr;
	struct page *page, *end_page;
	struct m2d_gmem *gmem;

	if (dev == NULL)
		return NULL;

	if (dev->total_gmem_size + rsize > MAX_DEVICE_GMEM_SIZE)
		return NULL;

	if (ctx->total_gmem_size + rsize > MAX_CONTEXT_GMEM_SIZE)
		return NULL;

	gmem = kmalloc(sizeof(struct m2d_gmem), GFP_KERNEL);
	if (gmem == NULL)
		return NULL;

	size = PAGE_ALIGN(rsize);
	order = get_order(size);

	page = alloc_pages(GFP_KERNEL | GFP_DMA, order);
	if (page == NULL) {
		kfree(gmem);
		return NULL;
	}

	kaddr = (unsigned long)page_address(page);
	end_page = page + (1 << order);

	memset(page_address(page), 0, size);
	__cpuc_flush_kern_all();

	gmem->gmem_size = size;
	gmem->gmem_pages = page;
	gmem->gmem_virt_addr = kaddr;
	gmem->gmem_phys_addr = __pa(kaddr);

	ctx->total_gmem_size += size;
	dev->total_gmem_size += size;

	do {
		set_page_count(page, 1);
		SetPageReserved(page);
		page++;
	} while (size -= PAGE_SIZE);

	/* free the otherwise unused pages. */
	while (page < end_page) {
		set_page_count(page, 1);
		__free_page(page);
		page++;
	}

	atomic_set(&gmem->gmem_count, 1);

	return gmem;
}

/**
 * @brief release the allocated graphics memory to the system, assume that
 * they have been already munmap'ed.
 */
static void m2d_free_gmem(struct m2d_context *ctx, struct m2d_gmem *gmem)
{
	struct m2d_device *dev = ctx->dev;
	struct page *page = gmem->gmem_pages;
	size_t size = gmem->gmem_size;

	if (!atomic_dec_and_test(&gmem->gmem_count))
		return;

	ctx->total_gmem_size -= size;
	dev->total_gmem_size -= size;

	for (; size > 0; size -= PAGE_SIZE, page++) {
		ClearPageReserved(page);
		__free_page(page);
	}

	kfree(gmem);
}

static int m2d_request_mem(struct file *file,
			   struct m2d_context *ctx, struct m2d_mem_req *req)
{
	struct m2d_device *dev = ctx->dev;
	struct mm_struct *mm = current->mm;
	struct m2d_mem_map *mmap = NULL;

	int ret;

	mmap = kmalloc(sizeof(struct m2d_mem_map), GFP_KERNEL);
	if (mmap == NULL)
		return -ENOMEM;

	memset(mmap, 0, sizeof(struct m2d_mem_map));

	mmap->mmap_type = req->req_type;

	switch (M2D_MEM_REQ_TYPE(mmap->mmap_type)) {
	case M2D_GRAPHICS_MEM:
		{
			struct m2d_gmem *gmem;

			if (req->req_size == 0) {
				req->phys_addr = 0l;
				req->mmap_addr = 0l;
				req->mmap_size = 0l;
				return 0;
			}

			gmem = m2d_alloc_gmem(ctx, req->req_size);

			if (gmem == NULL)
				return -ENOMEM;

			mmap->mmap_gmem = gmem;
			mmap->mmap_pgoff = gmem->gmem_phys_addr >> PAGE_SHIFT;
			mmap->mmap_size = gmem->gmem_size;
			break;
		}
	case M2D_FRAME_BUFFER:
		{
			if (dev->primary_fb_base == 0L
			    || dev->primary_fb_len == 0L)
				return -EINVAL;

			mmap->mmap_pgoff = dev->primary_fb_base >> PAGE_SHIFT;
			mmap->mmap_size = dev->primary_fb_len;
			break;
		}
	case M2D_REGISTERS:
		{
			mmap->mmap_pgoff = dev->gcu_regs_base >> PAGE_SHIFT;
			mmap->mmap_size = PAGE_ALIGN(dev->gcu_regs_len);
			break;
		}
	case M2D_RING_BUFFER:
		{
			mmap->mmap_pgoff = dev->ring_addr_dma >> PAGE_SHIFT;
			mmap->mmap_size = PAGE_ALIGN(dev->ring_size);
			break;
		}
	default:
		return -EINVAL;
	}

	list_add(&mmap->mmap_list, &ctx->mem_map_list);

	down_write(&mm->mmap_sem);
	ctx->mem_map = mmap;
	ret = do_mmap_pgoff(file, 0L,
			    mmap->mmap_size,
			    PROT_READ | PROT_WRITE,
			    MAP_SHARED, mmap->mmap_pgoff);
	ctx->mem_map = NULL;
	up_write(&mm->mmap_sem);

	if (ret < 0 && ret > -1024) {
		if (mmap->mmap_gmem)
			m2d_free_gmem(ctx, mmap->mmap_gmem);

		req->mmap_addr = 0l;
		req->mmap_size = 0l;
		kfree(mmap);
		return ret;
	}

	mmap->mmap_addr = (unsigned long)ret;

	req->mmap_addr = (unsigned long)ret;
	req->mmap_size = mmap->mmap_size;
	req->phys_addr = mmap->mmap_pgoff << PAGE_SHIFT;

	return 0;
}

static void m2d_release_mem(struct m2d_context *ctx, unsigned long mmap_addr)
{
	struct mm_struct *mm = current->mm;
	struct m2d_mem_map *mmap, *m;

	list_for_each_entry_safe(mmap, m, &ctx->mem_map_list, mmap_list) {
		if (mmap->mmap_addr == mmap_addr) {
			down_write(&mm->mmap_sem);
			do_munmap(mm, mmap->mmap_addr, mmap->mmap_size);
			up_write(&mm->mmap_sem);

			if (mmap->mmap_gmem)
				m2d_free_gmem(ctx, mmap->mmap_gmem);

			list_del(&mmap->mmap_list);
			kfree(mmap);
		}
	}
}

/****************************************************************************
 * E x e c u t i o n   C o n t r o l
 ***************************************************************************/
/**
 * @brief m2d_submit will try to copy the instructions from a user supplied
 * buffer to the GCU Ring Buffer for execution. This function will return
 * immediately once the copy is done and the GCU is started. Synchronization
 * is done by calling m2d_sync() to ensure that instructions submitted are
 * completely executed or until error occurs.
 *
 * This function will block if the device is current serving another context,
 * unless non-delay mode is set.
 */
static int m2d_submit(struct m2d_context *ctx, struct m2d_submit_req *req)
{
	struct m2d_device *dev;
	int ndelay = (req->mode & M2D_SUBMIT_MODE_NDELAY);
	int sync = (req->mode & M2D_SUBMIT_MODE_SYNC);
	int error = -EINVAL;

	void __user *usrbuf = (void *)req->base;
	size_t len = req->len;

	if (ctx == NULL || (dev = ctx->dev) == NULL)
		return -ENODEV;

	if (len >= GCU_RINGBUF_SIZE || (len % 4))
		return -EINVAL;

	if (!access_ok(VERIFY_READ, usrbuf, len))
		return -EFAULT;

	if (ndelay) {
		if (down_trylock(&dev->submit_sem))
			return -EAGAIN;
	} else {
		if (down_interruptible(&dev->submit_sem))
			return -ERESTARTSYS;
	}

	if (ctx != dev->last_context) {
		if ((error = m2d_wait_for_eeob(dev)) < 0)
			goto out;

		m2d_switch_context(ctx);
	}

	error = m2d_append(dev, usrbuf, len);
	if (error == -ENOSPC) {
		if (ndelay)
			goto out;

		if ((error = m2d_wait_for_eeob(dev)) < 0)
			goto out;

		if ((error = m2d_append(dev, usrbuf, len))) {
			printk("no space no space no space, len=%d\n", len);
			goto out;
		}
	}

	if (error == 0)
		error = m2d_kick(dev, sync);

      out:
	up(&dev->submit_sem);
	return error;
}

//#define FILL_NOP

#ifdef FILL_NOP
/* Copy to the ring buffer and update
 * NOTE: According to the Monahans Spec, we have to set the ring
 * buffer length to zero before touching the base and tail registers
 */
static inline void m2d_fill_nop(void *addr, int length)
{
	int i;
	unsigned long *p = addr;
	for (i = 0; i < (length >> 2); i++)
		*p++ = 0x05000000;
}
#endif

static int m2d_append(struct m2d_device *dev, void *usrbuf, size_t len)
{
	volatile gcu_regs_t *gr = dev->gcu_regs;
	unsigned int tail_room, head_room;
	unsigned long exhead = gr->gcrbexhr;
	unsigned long tail = dev->ring_tail_dma;
	unsigned long base = dev->ring_addr_dma;
	unsigned long size = dev->ring_size;
	unsigned char *ring_addr = (unsigned char *)dev->ring_addr;

	if (tail >= exhead) {
		tail_room = size - (tail - base);
		head_room = exhead - base;
	} else {
		tail_room = exhead - tail;
		head_room = 0;
	}

	if (tail_room >= len) {
		if (copy_from_user(ring_addr + (tail - base), usrbuf, len))
			return -EFAULT;
		tail += len;
#ifdef FILL_NOP
	} else if (head_room >= len) {
		m2d_fill_nop(ring_addr + (tail - base), tail_room);
#else
	} else if (head_room + tail_room >= len) {
		if (copy_from_user(ring_addr + (tail - base),
				   usrbuf, tail_room))
			return -EFAULT;
		usrbuf += tail_room;
		len -= tail_room;
#endif
		if (copy_from_user(ring_addr, usrbuf, len))
			return -EFAULT;
		tail = dev->ring_addr_dma + len;
	} else {
		return -ENOSPC;
	}

	if (tail - base == size)
		tail = base;

	dev->ring_tail_dma = tail;
	gr->gcrbtr = tail;

	return 0;
}

static int m2d_kick(struct m2d_device *dev, int sync)
{
	volatile gcu_regs_t *gr = dev->gcu_regs;
	unsigned long flags;

	/* rock'n roll */
	spin_lock_irqsave(&dev->regs_lock, flags);
	gr->gccr = (gr->gccr & ~GCCR_STOP) & GCCR_MASK;
	spin_unlock_irqrestore(&dev->regs_lock, flags);

	if (sync)
		m2d_wait_for_eeob(dev);

	return 0;
}

static int m2d_wait_for_eeob(struct m2d_device *dev)
{
	volatile gcu_regs_t *gr = dev->gcu_regs;
	int ret = 0;
	unsigned long flags;

	DECLARE_WAITQUEUE(wait, current);
	add_wait_queue(&dev->wait_eeob, &wait);

	spin_lock_irqsave(&dev->regs_lock, flags);
	gr->gciscr |= GCISCR_EEOB_INTST;
	gr->gciecr |= GCIECR_EEOB_INTEN;
	spin_unlock_irqrestore(&dev->regs_lock, flags);

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (gr->gcrbexhr == gr->gcrbtr) {
			ret = 0;
			break;
		}
		schedule();
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&dev->wait_eeob, &wait);
	return ret;
}

/**
 * @brief m2d_sync is used to ensure that instructions submitted last time
 * will be completely finished or an error occurs. Completion is indicated
 * by interrupt handler by waking up.
 */
static int m2d_sync(struct m2d_context *ctx)
{
	struct m2d_device *dev = ctx->dev;

	if (dev->last_context != ctx)
		return 0;

	return m2d_wait_for_eeob(dev);
}

/*****************************************************************************
 * 2D GCU device initialization/destroying functions
 ****************************************************************************/

static int m2d_init_device(struct m2d_device *dev)
{

#ifdef CONFIG_M2D_RINGBUF_IN_SRAM
	if ((dev->immid = imm_register_kernel()) == 0) {
		m2d_warn("error registering with IMM\n");
		return -1;
	}

	dev->ring_addr = imm_malloc(GCU_RINGBUF_SIZE,
				    IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM,
				    dev->immid);

	dev->ring_addr_dma = imm_get_physical(dev->ring_addr, dev->immid);
#else
	dev->ring_addr = dma_alloc_coherent(dev->dev,
					    GCU_RINGBUF_SIZE,
					    (dma_addr_t *) & dev->ring_addr_dma,
					    GFP_KERNEL);
#endif
	if (dev->ring_addr == NULL) {
		m2d_warn("error allocating memory for ring buffer\n");
		return -1;
	}

	dev->ring_size = ALIGN(dev->ring_addr_dma + GCU_RINGBUF_SIZE, 256);
	dev->ring_addr_dma = ALIGN(dev->ring_addr_dma, 256);

	dev->ring_size = dev->ring_size - dev->ring_addr_dma;
	dev->ring_tail_dma = dev->ring_addr_dma;

	dev->gcu_regs_base = GCU_REGS_BASE;
	dev->gcu_regs_len = sizeof(gcu_regs_t);
	dev->gcu_regs = (gcu_regs_t *) & __REG_3(GCU_REGS_BASE);

	m2d_reset_device(dev);

	init_MUTEX(&dev->submit_sem);
	init_waitqueue_head(&dev->wait_eeob);

	INIT_LIST_HEAD(&dev->context_list);
	dev->last_context = NULL;
	dev->context_count = 0UL;
	dev->total_gmem_size = 0UL;

	return 0;
}

static void m2d_reset_device(struct m2d_device *dev)
{
	volatile gcu_regs_t *gr = dev->gcu_regs;
	unsigned long flags;

	dev->ring_tail_dma = dev->ring_addr_dma;

	spin_lock_irqsave(&dev->regs_lock, flags);
	gr->gccr = GCCR_ABORT | (gr->gccr & 0xF);
	udelay(10);
	gr->gccr = GCCR_STOP | (gr->gccr & 0xF);

	gr->gcrblr = 0;
	gr->gcrbbr = dev->ring_addr_dma;
	gr->gcrbtr = dev->ring_tail_dma;
	gr->gcrblr = dev->ring_size;

	gr->gciscr |= GCISCR_MASK;
	gr->gciecr &= ~GCIECR_MASK;
	gr->gciecr |= (GCIECR_PF_INTEN | GCIECR_IIN_INTEN | GCIECR_IOP_INTEN);

	gr->gcnopid = 0;
	gr->gctabaddr = 0;
	gr->gcmabaddr = 0;
	spin_unlock_irqrestore(&dev->regs_lock, flags);

	return;
}

static void m2d_free_device(struct m2d_device *dev)
{
	volatile gcu_regs_t *gr = dev->gcu_regs;
	unsigned long flags;

	spin_lock_irqsave(&dev->regs_lock, flags);

	/* Stop the graphics controller */
	if (!(gr->gccr & GCCR_STOP))
		gr->gccr = (gr->gccr & GCCR_MASK) | GCCR_STOP;

	/* Clear the ring buffer configuration */
	gr->gcrblr = 0;
	gr->gcrbbr = 0;
	gr->gcrbtr = 0;
	spin_unlock_irqrestore(&dev->regs_lock, flags);

	/* Free the ring buffer space */
#ifdef CONFIG_M2D_RINGBUF_IN_SRAM
	imm_free(dev->ring_addr, dev->immid);
#else
	dma_free_coherent(dev->dev,
			  dev->ring_size,
			  dev->ring_addr, (dma_addr_t) dev->ring_addr_dma);
#endif
}

/****************************************************************************
 * I n t e r r u p t   H a n d l e r
 ***************************************************************************/
static int m2d_gcu_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct m2d_device *dev = (struct m2d_device *)dev_id;
	volatile gcu_regs_t *gr = dev->gcu_regs;
	unsigned long status = gr->gciscr & gr->gciecr;

	if (irq != IRQ_GRPHICS)
		return IRQ_NONE;

	/* FIXME: what if this interrupt occurs with no current context
	 * in execution
	 */
	if (status & GCISCR_EEOB_INTST)
		m2d_interrupt_eeob(dev, gr);

	if (status & (GCISCR_PF_INTST | GCISCR_IIN_INTST | GCISCR_IOP_INTST))
		m2d_interrupt_err(dev, gr);

	return IRQ_HANDLED;
}

/**
 * @brief m2d_interrupt_eeob() will be invoked from the interrupt handler
 * m2d_gcu_irq() to handle the Execution End Of Buffer interrupt.
 */
static int m2d_interrupt_eeob(struct m2d_device *dev, volatile gcu_regs_t * gr)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->regs_lock, flags);
	gr->gciecr &= ~GCIECR_EEOB_INTEN;
	gr->gciscr |= GCISCR_EEOB_INTST;

	gr->gciecr = gr->gciecr & GCIECR_MASK;
	spin_unlock_irqrestore(&dev->regs_lock, flags);

	if (gr->gcrbexhr != gr->gcrbtr)
		printk(KERN_ERR "EEOB: exhead=%08x, tail=%08x\n",
		       gr->gcrbexhr, gr->gcrbtr);

	/* wake up the task waiting for completion by m2d_wait_for_eeob() */
	wake_up(&dev->wait_eeob);

	return IRQ_HANDLED;
}

static int m2d_interrupt_err(struct m2d_device *dev, volatile gcu_regs_t * gr)
{
	struct m2d_context *ctx = dev->last_context;
	unsigned long flags;

	if (ctx == NULL) {
		spin_lock_irqsave(&dev->regs_lock, flags);
		gr->gciscr |= (GCISCR_IIN_INTST | GCISCR_IOP_INTST |
			       GCISCR_PF_INTST);
		spin_unlock_irqrestore(&dev->regs_lock, flags);
		return IRQ_HANDLED;
	}

	if (gr->gciscr & GCISCR_PF_INTST) {
		/* Illegal pixel format */
		spin_lock_irqsave(&dev->regs_lock, flags);
		gr->gciscr |= GCISCR_PF_INTST;
		spin_unlock_irqrestore(&dev->regs_lock, flags);
		ctx->err_ipf++;
	}

	if (gr->gciscr & GCISCR_IIN_INTST) {
		/* Illegal Instruction, GCI type of instruction invalid */
		spin_lock_irqsave(&dev->regs_lock, flags);
		gr->gciscr |= GCISCR_IIN_INTST;
		spin_unlock_irqrestore(&dev->regs_lock, flags);
		ctx->err_iin++;
		goto gcu_fault;
	}

	if (gr->gciscr & GCISCR_IOP_INTST) {
		/* Illegal operation, opcode of instruction invalid */
		spin_lock_irqsave(&dev->regs_lock, flags);
		gr->gciscr |= GCISCR_IOP_INTST;
		spin_unlock_irqrestore(&dev->regs_lock, flags);
		ctx->err_iop++;
		goto gcu_fault;
	}

	return IRQ_HANDLED;

gcu_fault:
	/* Illgeal 2D Graphics Instruction, dump the ring buffer content
	 * nearby */
	dump_gcu_ring(dev, gr);

	send_sig(SIGILL, ctx->task, 0);
	m2d_reset_device(dev);

	return IRQ_HANDLED;
}

/****************************************************************************
 * P o w e r   M a n a g e m e n t
 ***************************************************************************/
#ifdef CONFIG_PM
static int m2d_suspend(struct device *dev, u32 state, u32 level)
{
	struct m2d_device *mdev = dev_get_drvdata(dev);
	unsigned long flags;

	if (level == SUSPEND_POWER_DOWN) {
		enable_gcu_clock(mdev);
		m2d_wait_for_eeob(mdev);

		if (mdev->last_context) {
			spin_lock_irqsave(&mdev->context_lock, flags);
			m2d_save_context(mdev->last_context, mdev->gcu_regs);
			spin_unlock_irqrestore(&mdev->context_lock, flags);
		}

		disable_gcu_clock(mdev);

	}
	return 0;
}

static int m2d_resume(struct device *dev, u32 level)
{
	struct m2d_device *mdev = dev_get_drvdata(dev);
	volatile gcu_regs_t *gr = mdev->gcu_regs;
	unsigned long flags;

	if (level == RESUME_ENABLE) {
		enable_gcu_clock(mdev);
		m2d_reset_device(mdev);

		/* Restore the context */
		if (mdev->last_context) {
			spin_lock_irqsave(&mdev->context_lock, flags);
			m2d_restore_context(mdev->last_context, gr);
			spin_unlock_irqrestore(&mdev->context_lock, flags);
		} else
			disable_gcu_clock(mdev);
	}
	return 0;
}
#endif				/* CONFIG_PM */

/****************************************************************************
 * F i l e   S y s t e m   I / O   o p e r a t i o n s
 ***************************************************************************/

static int m2d_open(struct inode *inode, struct file *file)
{
	struct m2d_context *context = file->private_data;
	struct m2d_device *dev = &m2d_dev;

	enable_gcu_clock(dev);
	if (context == NULL) {
		if ((context = m2d_create_context(dev)) == NULL)
			return -ENOMEM;
		file->private_data = context;
	}

	return 0;
}

static int m2d_release(struct inode *inode, struct file *file)
{
	struct m2d_context *context = file->private_data;

	m2d_wait_for_eeob(&m2d_dev);
	m2d_free_context(context);
	disable_gcu_clock(&m2d_dev);

	return 0;
}

static struct page *__m2d_follow_page(struct mm_struct *mm, unsigned long address)
{
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *ptep, pte;
	unsigned long pfn;
	struct page *page;

	pgd = pgd_offset(mm, address);
	if (pgd_none(*pgd) || unlikely(pgd_bad(*pgd)))
		goto out;

	pmd = pmd_offset(pgd, address);
	if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd)))
		goto out;

	ptep = pte_offset_map(pmd, address);
	if (!ptep)
		goto out;

	pte = *ptep;
	pte_unmap(ptep);
	if (pte_present(pte)) {
		pfn = pte_pfn(pte);
		if (pfn_valid(pfn)) {
			page = pfn_to_page(pfn);
			return page;
		}
	}

out:
	return NULL;
}

static int m2d_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	struct m2d_context *ctx = file->private_data;
	struct m2d_device *dev = (struct m2d_device *)ctx->dev;

	if (dev == NULL)
		return -ENODEV;

	switch (cmd) {
	case M2DIO_REQUEST_MEM:
		{
			struct m2d_mem_req mem_req;
			void __user *argp = (void *)arg;
			int ret;

			if (copy_from_user(&mem_req, argp, sizeof(mem_req)))
				return -EFAULT;

			ret = m2d_request_mem(file, ctx, &mem_req);
			if (ret < 0)
				return ret;

			if (copy_to_user(argp, &mem_req, sizeof(mem_req))) {
				m2d_release_mem(ctx, mem_req.mmap_addr);
				return -EFAULT;
			}

			return 0;
		}

	case M2DIO_RELEASE_MEM:
		{
			unsigned long mmap_addr = arg;
			m2d_release_mem(ctx, mmap_addr);
			return 0;
		}

	case M2DIO_FLUSH_MEM:
		{
			struct m2d_mem_map *mmap;
			list_for_each_entry(mmap, &ctx->mem_map_list, mmap_list) {
				if (mmap->mmap_addr == arg)
					__cpuc_flush_user_range(mmap->mmap_addr,
								mmap->mmap_size,
								0);
			}
			return 0;
		}

	case M2DIO_SUBMIT:
		{
			struct m2d_submit_req submit_req;
			void __user *argp = (void *)arg;

			if (copy_from_user
			    (&submit_req, argp, sizeof(submit_req)))
				return -EFAULT;

			return m2d_submit(ctx, &submit_req);
		}

	case M2DIO_SYNC:
		return m2d_sync(ctx);

	case M2DIO_GET_BUS_ADDR:
	{
		/* assume user provided virtual memory are physical
		 * contiguous
		 */
		unsigned long virt_addr;
		unsigned long phys_addr;
		struct page *page;

		if (copy_from_user(&virt_addr, (void *)arg, sizeof(virt_addr)))
			return -EFAULT;

		page = __m2d_follow_page(current->mm, virt_addr);
		if (page == NULL)
			return -EINVAL;
		phys_addr = __pa(page_address(page)) + virt_addr % PAGE_SIZE;

		if (copy_to_user((void *)arg, &phys_addr, sizeof(phys_addr)))
			return -EFAULT;

		return 0;
	}
	default:
		break;
	}

	return 0;
}

static int m2d_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct m2d_context *ctx = file->private_data;
	struct m2d_mem_map *mmap = ctx->mem_map;
	pgprot_t pgprot;

	if (ctx == NULL)
		return -ENODEV;

	/* Within 4GB address space? */
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	/* DO NOT allow arbitrary memory map request */
	if (mmap == NULL)
		return -EPERM;

	if (mmap->mmap_size != (vma->vm_end - vma->vm_start))
		return -EINVAL;

	switch (M2D_MEM_REQ_ATTR(mmap->mmap_type)) {
	case M2D_ATTR_COHERENT:
		pgprot = pgprot_noncached(vma->vm_page_prot);
		break;
	case M2D_ATTR_WRITECOMBINE:
		pgprot = pgprot_writecombine(vma->vm_page_prot);
		break;
	case M2D_ATTR_CACHEABLE:
	default:
		pgprot = vma->vm_page_prot;
		break;
	}

	vma->vm_page_prot = pgprot;
	vma->vm_flags |= (VM_IO | VM_RESERVED);

	if (remap_pfn_range(vma, vma->vm_start,
			    mmap->mmap_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	__cpuc_flush_kern_all();

#ifdef DEBUG_CACHE_COHERENCY
	dump_page_table_entries(vma->vm_start, vma->vm_end, mmap->mmap_gmem);
#endif
	return 0;
}

static int m2d_fsync(struct file *file, struct dentry *dentry, int datasync)
{
	struct m2d_context *ctx;
	int error;

	ctx = (struct m2d_context *)(file->private_data);
	error = m2d_sync(ctx);

	return error;
}

static struct file_operations m2d_fops = {
	.owner = THIS_MODULE,
	.open = m2d_open,
	.release = m2d_release,
	.ioctl = m2d_ioctl,
	.mmap = m2d_mmap,
	.fsync = m2d_fsync,
};

#ifdef CONFIG_M2D_USE_PROC
/****************************************************************************
 * / p r o c    f i l e s y s t e m    i n t e r f a c e
 ***************************************************************************/
static char *mmap_type_desc[] = { "GMEM", "FBUF", "REGS", "RBUF" };
static char *mmap_attr_desc[] = { "coherent", "writecombine", "cacheable" };

static int m2d_read_proc(char *buf, char **start, off_t off,
			 int count, int *eof, void *data)
{
	struct m2d_context *ctx;
	struct m2d_device *dev = &m2d_dev;
	volatile gcu_regs_t *gr = dev->gcu_regs;
	int len = 0, index = 0;

	len += sprintf(buf + len,
		       "device: submit_sem=%d last_context=0x%p gmem_size=%ld\n",
		       atomic_read(&dev->submit_sem.count),
		       dev->last_context, dev->total_gmem_size);

	len += sprintf(buf + len,
		       "GCU : GCCR=0x%08x GCISCR=0x%08x GCFLAGS=0x%08x GCNOPID=0x%08x\n",
		       gr->gccr, gr->gciscr, gr->gcflags, gr->gcnopid);
	len += sprintf(buf + len,
		       "      GCCABADDR=0x%08x GCTABADDR=0x%08x GCMABADDR=0x%08x\n",
		       gr->gccabaddr, gr->gctabaddr, gr->gcmabaddr);

	len += sprintf(buf + len,
		       "RING: @0x%08x (%dkb), head=0x%08x, tail=0x%08x, exhead=0x%08x\n",
		       gr->gcrbbr,
		       (gr->gcrblr >> 10),
		       gr->gcrbhr, gr->gcrbtr, gr->gcrbexhr);
	len += sprintf(buf + len,
		       "GCSC0: %08x%08xL  GCSC1: %08x%08xL\n",
		       gr->gcsc0wd1, gr->gcsc0wd0, gr->gcsc1wd1, gr->gcsc1wd0);
	len += sprintf(buf + len,
		       "GCSC2: %08x%08xL  GCSC3: %08x%08xL\n",
		       gr->gcsc2wd1, gr->gcsc2wd0, gr->gcsc3wd1, gr->gcsc3wd0);
	len += sprintf(buf + len,
		       "GCSC4: %08x%08xL  GCSC5: %08x%08xL\n",
		       gr->gcsc4wd1, gr->gcsc4wd0, gr->gcsc5wd1, gr->gcsc5wd0);
	len += sprintf(buf + len,
		       "GCSC6: %08x%08xL  GCSC7: %08x%08xL\n",
		       gr->gcsc6wd1, gr->gcsc6wd0, gr->gcsc7wd1, gr->gcsc7wd0);

	len += sprintf(buf + len,
		       "SRC0: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		       gr->gcs0br, gr->gcs0stp, gr->gcs0str, gr->gcs0pf);
	len += sprintf(buf + len,
		       "SRC1: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		       gr->gcs1br, gr->gcs1stp, gr->gcs1str, gr->gcs1pf);
	len += sprintf(buf + len,
		       "DST0: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		       gr->gcd0br, gr->gcd0stp, gr->gcd0str, gr->gcd0pf);
	len += sprintf(buf + len,
		       "DST1: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		       gr->gcd1br, gr->gcd1stp, gr->gcd1str, gr->gcd1pf);
	len += sprintf(buf + len,
		       "DST2: Base=0x%08x, Step=0x%x, Stride=%x, PF=%x\n",
		       gr->gcd2br, gr->gcd2stp, gr->gcd2str, gr->gcd2pf);

	list_for_each_entry(ctx, &dev->context_list, list) {

		struct m2d_mem_map *m;

		len += sprintf(buf + len,
			       "%2d[%c] @%p GMEM:%6ld bytes IIN:%2d IOP:%2d IPF:%2d\n",
			       index++,
			       ((ctx == dev->last_context) ? 'A' : '-'), ctx,
			       ctx->total_gmem_size, ctx->err_iin, ctx->err_iop,
			       ctx->err_ipf);

		list_for_each_entry(m, &ctx->mem_map_list, mmap_list) {
			len += sprintf(buf + len,
				       "  %08lx - %08lx [%s] %08lx size=%6ld (%s)\n",
				       (m->mmap_pgoff << PAGE_SHIFT),
				       (m->mmap_pgoff << PAGE_SHIFT) +
				       m->mmap_size,
				       mmap_type_desc[M2D_MEM_REQ_TYPE
						      (m->mmap_type)],
				       (m->mmap_addr), (m->mmap_size),
				       mmap_attr_desc[M2D_MEM_REQ_ATTR
						      (m->mmap_type)]
			    );
		}
	}
	*eof = 1;
	return len;
}

static void m2d_init_proc(void)
{
	struct proc_dir_entry *m2d_proc_entry;

	m2d_proc_entry = create_proc_entry("m2d", 0, NULL);
	if (m2d_proc_entry == NULL) {
		m2d_warn("error creating proc entry\n");
		return;
	}

	m2d_proc_entry->read_proc = m2d_read_proc;
	m2d_proc_entry->write_proc = NULL;
	m2d_proc_entry->data = &m2d_dev;

	return;
}

static void m2d_free_proc(void)
{
	remove_proc_entry("m2d", NULL);
}
#endif				/* CONFIG_M2D_USE_PROC */

/****************************************************************************
 * Initialization / Registeration / Removal
 ***************************************************************************/
static int m2d_probe(struct device *dev)
{
	int result;

	printk(KERN_INFO "2D Graphics Driver for Monahans Linux\n");

	result = register_chrdev(m2d_major, "m2d", &m2d_fops);
	if (result < 0) {
		m2d_warn("unable to register character device /dev/m2d\n");
		return result;
	}

	spin_lock_init(&m2d_dev.context_lock);
	spin_lock_init(&m2d_dev.regs_lock);
	spin_lock_init(&m2d_dev.gcu_lock);

	enable_gcu_clock(&m2d_dev);
	result = m2d_init_device(&m2d_dev);
	if (result < 0) {
		m2d_warn("unable to initialize device\n");
		unregister_chrdev(m2d_major, "m2d");
		disable_gcu_clock(&m2d_dev);
		return result;
	}

	if (request_irq(IRQ_GRPHICS, m2d_gcu_irq,
			0, "GCU", &m2d_dev)) {
		m2d_warn("unable to request the GCU IRQ\n");
		unregister_chrdev(m2d_major, "m2d");
		disable_gcu_clock(&m2d_dev);
		return -EBUSY;
	}

	m2d_dev.dev = dev;
	dev_set_drvdata(dev, &m2d_dev);

#ifdef CONFIG_M2D_USE_PROC
	m2d_init_proc();
#endif

	return 0;
}

static int m2d_remove(struct device *dev)
{
#ifdef CONFIG_M2D_USE_PROC
	m2d_free_proc();
#endif
	disable_gcu_clock(&m2d_dev);
	free_irq(IRQ_GRPHICS, &m2d_dev);
	m2d_free_device(&m2d_dev);

	unregister_chrdev(m2d_major, "m2d");

	return 0;
}

static struct device_driver m2d_driver = {
	.name = "m2d",
	.bus = &platform_bus_type,
	.probe = m2d_probe,
	.remove = m2d_remove,

#ifdef CONFIG_PM
	.suspend = m2d_suspend,
	.resume = m2d_resume,
#endif
};

static int __devinit m2d_init(void)
{
	driver_register(&m2d_driver);

	return 0;
}

static void __exit m2d_exit(void)
{
	driver_unregister(&m2d_driver);
}

module_init(m2d_init);
module_exit(m2d_exit);
MODULE_LICENSE("GPL");
