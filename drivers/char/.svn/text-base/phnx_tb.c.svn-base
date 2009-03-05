#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/rmi/pic.h>
#include <asm/rmi/phnx_tb.h>
#include <linux/phnx_tb.h>

#define PHNX_TB_DEBUG 1
#define TB_INT_FLAGS 0

#define Message(a,b...) //printk("\n"a"\n",##b)
#define ErrorMsg(a,b...) printk("\n"a"\n",##b)


static int			tb_major;
static spinlock_t trace_buf_lock=SPIN_LOCK_UNLOCKED;
static int tb_is_opened = 0;
tb_dev_t    tb_dev; 



struct file_operations tb_fops = {
	.read	    = tb_read,
	.open	    = tb_open,
	.ioctl      = tb_ioctl,
	.release    = tb_release,
};



/* -----------------------------    LAYER 2 Code   ------------------------------ */

static inline void read_tb_entry(unsigned char *tb_entry_ptr)
{
	int j;
	unsigned int *x;

	tb_pop_entry();
	for (j = 0; j < 4; ++j) {
		x = (unsigned int *) (tb_entry_ptr + 4*j);
		*x = tb_read_rddata_reg (j);
	}
}


irqreturn_t tb_int_handler(int irq, void *dev_id, struct pt_regs *regs)
{
  unsigned int  status_reg, ctrl_reg;
	unsigned int  tb_wr_ptr, tb_rd_ptr, tb_first_match_ptr;
	int    m, n;
	unsigned int no_of_junk_entry;
	unsigned int req_read_pos =0;
	status_reg = tb_read_status_reg();
	Message("Interrupt Handler Called.");
	
	if (!TB_IS_STATUS_DONE(status_reg)){
		ErrorMsg("something is wrong as int must not have been generated");
		return IRQ_HANDLED;
	}

	tb_wr_ptr = TB_GET_WRPTR(status_reg);
	tb_rd_ptr = TB_GET_RDPTR(status_reg);
	tb_first_match_ptr = TB_GET_FIRST_MATCH_PTR(status_reg); 

	ctrl_reg = tb_read_ctrl_reg();
	n = TB_GET_CTRL_REQCNT(ctrl_reg);

	Message("tb_wr_ptr %d, tb_rd_ptr %d, tb_first_match_ptr %d, Count %d",
									tb_wr_ptr,tb_rd_ptr,tb_first_match_ptr,n);
	if(tb_wr_ptr >= n)
		req_read_pos = tb_wr_ptr - n;
	else
		req_read_pos = 255 - (n - tb_wr_ptr);

	if(req_read_pos >= tb_rd_ptr){
		no_of_junk_entry = req_read_pos - tb_rd_ptr  ;
		Message("--> 1 No. Of Junk Entry %d",no_of_junk_entry);
	}
	else{
		no_of_junk_entry =(255 - tb_rd_ptr) + req_read_pos ;
		Message("--> 2 No. Of Junk Entry %d",no_of_junk_entry);
	}

	while(no_of_junk_entry--)
	  tb_pop_entry();

	status_reg = tb_read_status_reg();
	tb_rd_ptr = TB_GET_RDPTR(status_reg);
	Message("After Removin Junk Entry ReadPtr %d",tb_rd_ptr);
	
	tb_dev.size = 16 * TB_GET_CTRL_REQCNT(ctrl_reg);
	for (m = 0; m < tb_dev.size; m += 16)
	  read_tb_entry (tb_dev.data + m);

	status_reg = tb_read_status_reg();
	tb_rd_ptr = TB_GET_RDPTR(status_reg);
	Message("After Popin All Valid Entry ReadPtr %d",tb_rd_ptr);
	return 0;
}

/* -----------------------------    LAYER 3 Code   ------------------------------ */

int tb_open (struct inode *inode, struct file *filp)
{
	unsigned long flags=0;

	Message("tb_open() invoked");
	spin_lock_irqsave(&trace_buf_lock,flags);
	if(tb_is_opened){
					spin_unlock_irqrestore(&trace_buf_lock,flags);
					return -1;
	}
	tb_is_opened = 1;
	spin_unlock_irqrestore(&trace_buf_lock,flags);
	tb_dev.size =0;
	return 0;
}

ssize_t tb_read (struct file *filp, char *buf, size_t count, loff_t *offset)
{
	// if current position is past the size of the buffer

	if (*offset >= tb_dev.size)
		return 0;

	if ((*offset + count) > tb_dev.size)
		count = tb_dev.size - *offset;

	if (__copy_to_user (buf, (tb_dev.data+*offset) , count))
		return -EFAULT;

	*offset += count;
	return count;
}

int tb_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
		case TB_IOC_GTBREG: {
			struct tb_register requested_reg;
		
			Message("file: %s, line: %d: ioctl(): TB_IOC_GTBREG\n", __FILE__, __LINE__);
			// copy_from_user(to, from, size)
			if ( copy_from_user (&requested_reg, (struct tb_register *)arg, 
					sizeof(struct tb_register))) {
				ErrorMsg("ioctl(): copy_to_user() failed in file %s at line %d",
					__FILE__, __LINE__);
				return -EFAULT;
			}
			if(requested_reg.type > 0xf || requested_reg.type < 0){
				ErrorMsg("Invalid Register Access");
				return -EINVAL;
			}
			requested_reg.val = tb_read_reg_be32(requested_reg.type);
			if ( copy_to_user ((struct tb_register *) arg, &requested_reg, 
					sizeof(struct tb_register)) ) {
				ErrorMsg("ioctl(): copy_to_user() failed in file %s at line %d",
					__FILE__, __LINE__);
				return -EFAULT;
			}
		}
			break;

		case TB_IOC_STBREG: {
			struct tb_register reg;
		
			Message("file: %s, line: %d: ioctl(): TB_IOC_STBREG\n", __FILE__, __LINE__);
			// copy_from_user(to, from, size)
			if ( copy_from_user (&reg, (struct tb_register *)arg, 
					sizeof(struct tb_register))) {
				ErrorMsg("ioctl(): copy_to_user() failed in file %s at line %d",
					__FILE__, __LINE__);
				return -EFAULT;
			}
			if(reg.type > 0xf || reg.type < 0){
				ErrorMsg("Invalid Register Access");
				return -EINVAL;
			}


			tb_write_reg_be32(reg.type, reg.val);
			if (reg.type == TB_CTRL_REG) {
							unsigned int x;

							Message("ioctl(): the reqcnt in the ctrl reg is(1): %d\n", TB_GET_CTRL_REQCNT(reg.val));
							x = tb_read_ctrl_reg();
							Message("ioctl(): the reqcnt in the ctrl reg is(2): %d\n", TB_GET_CTRL_REQCNT(x));
			}
			
		}
			break;

		case TB_IOC_REINIT: {
			Message("file: %s, line: %d: ioctl(): TB_REINIT\n", __FILE__, __LINE__);
			tb_reinit();
			tb_dev.size =0;
		}
			break;

		default:
			ErrorMsg("Unindentified ioctl() command");
			return -EINVAL;
	}

	return 0;
}

int tb_release (struct inode *inode, struct file *filp)
{
	unsigned long flags;
	spin_lock_irqsave(&trace_buf_lock,flags);
	tb_is_opened = 0;
	spin_unlock_irqrestore(&trace_buf_lock,flags);
	return 0;
}

// tb_init(): invoked as part of the kernel bootup process
static int tb_init(void)	
{
	int err=0;

	Message("tb_init() invoked");
	tb_major = register_chrdev (0, "xlr_tracebuffer", &tb_fops);
	if (tb_major < 0) {
		ErrorMsg("tb_init() register_chrdev() failed");
		return tb_major;
	}
	
	// the handler too gets registered in the following call 
	err = request_irq(PIC_BRIDGE_TB_IRQ, tb_int_handler, TB_INT_FLAGS, "trace buffer", NULL);
	if (err) {
			unregister_chrdev (tb_major, "trace buffer");
			ErrorMsg("tb_init(): request_irq() failed");
			return err;
	}
	tb_dev.data = (unsigned char *)kmalloc(TB_SIZE,GFP_KERNEL);
	Message("tb_init() request_irq() succeeded");
	printk("Registered XLR tracebuffer driver with Major No. [%d]\n",tb_major);
	return 0;
}

static void tb_exit(void)
{
	free_irq(PIC_BRIDGE_TB_IRQ, NULL);
	kfree(tb_dev.data);
	if ( unregister_chrdev (tb_major, "trace buffer") )
		ErrorMsg("tb_exit()- unregister_chrdev() failed");
}
module_init (tb_init);
module_exit (tb_exit);
// Do we need to export any names ?
