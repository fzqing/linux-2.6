/*
 * Monahans ARAVA PMIC Management Routines
 *
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/arava.h>
#include <asm/arch/mhn_pm.h>
#include <asm/arch/mhn_pmic.h>

#define	IRQ_ARAVA_EVENT		(IRQ_WAKEUP1)
#define	PECR_INT_EN		(PECR_E0IE)
#define	PECR_DIR		(PECR_DIR0)

static struct mhn_pmic_regs arava_regs[ARAVA_REG_NUM];
static spinlock_t arava_lock;

/* Make sure that Power I2C has been initialized before invoke this function */
#ifdef CONFIG_USB_PXA3XX
extern int pxa3xx_usb_event_change(unsigned int events);
#endif
int arava_write(u8 reg, u8 val);
int arava_read(u8 reg, u8 * pval);

/* Wrap functions for pmic read/write */
int mhn_pmic_read(u8 reg, u8 * pval)
{
	return arava_read(reg, pval);
}

EXPORT_SYMBOL(mhn_pmic_read);

int mhn_pmic_write(u8 reg, u8 val)
{
	return arava_write(reg, val);
}

EXPORT_SYMBOL(mhn_pmic_write);

static int arava_set_pump(int enable)
{
	int ret;
	u8 val;
	unsigned long flags;

	local_irq_save_nort(flags);
	spin_lock_rt(&arava_lock);
	if (enable) {
		ret = arava_read(ARAVA_MISCB, &val);
		if (ret)
			goto out;
		val |= ARAVA_MISCB_SESSION_VALID_EN;
		val &= ~ARAVA_MISCB_USBINT_BOTHEDGE;
		ret = arava_write(ARAVA_MISCB, val);
		if (ret)
			goto out;

		/* FIXME: We use EXTON as cable detect signal on ZYlonite.
		 * This depends on the cable signal is connected to EXTON
		 * of ARAVA.
		 * If the cable signal is not connected to EXTON, we need
		 * use other signal as cable detect signal.
		 */
		ret = arava_read(ARAVA_IRQ_MASK_A, &val);
		if (ret)
			goto out;
		val &= ~ARAVA_IRQMASK_A_EXTON;
		ret = arava_write(ARAVA_IRQ_MASK_A, val);
		if (ret)
			goto out;
		val = ARAVA_USBPUMP_EN_USBVE | ARAVA_USBPUMP_EN_USBVEP;
		ret = arava_write(ARAVA_USBPUMP, val);
	} else {
		ret = arava_read(ARAVA_MISCB, &val);
		if (ret)
			goto out;
		val &= ~(ARAVA_MISCB_SESSION_VALID_EN |
			 ARAVA_MISCB_USBINT_BOTHEDGE);
		ret = arava_write(ARAVA_MISCB, val);
		if (ret)
			goto out;
		ret = arava_read(ARAVA_IRQ_MASK_A, &val);
		if (ret)
			goto out;
		val |= ARAVA_IRQMASK_A_EXTON;
		ret = arava_write(ARAVA_IRQ_MASK_A, val);
		if (ret)
			goto out;
		ret = arava_write(ARAVA_USBPUMP, 0);
	}
      out:
	local_irq_restore_nort(flags);
	spin_unlock_rt(&arava_lock);
	return ret;
}

static int arava_set_vbus_supply(int enable, int srp)
{
	int ret;
	u8 val;

	ret = arava_read(ARAVA_USBPUMP, &val);
	if (ret)
		goto out;

	if (enable) {
		val |= ARAVA_USBPUMP_USBVE | ARAVA_USBPUMP_EN_USBVE |
		    ARAVA_USBPUMP_EN_USBVEP;
		if (srp)
			val |= ARAVA_USBPUMP_USBVEP;
		else
			val &= ~ARAVA_USBPUMP_USBVEP;
	} else {
		val |= ARAVA_USBPUMP_EN_USBVE | ARAVA_USBPUMP_EN_USBVEP;
		val &= ~(ARAVA_USBPUMP_USBVE | ARAVA_USBPUMP_USBVEP);
	}
	ret = arava_write(ARAVA_USBPUMP, val);
	pr_debug("%s enable %d srp %d val %x\n", __func__, enable, srp, val);

      out:
	if (ret)
		printk(KERN_ALERT "i2c operation error %d\n", ret);
	return ret;
}

/* Set USB A-device events: VBUS/Session valid.
 * For ARAVA, only SRP_DETECT event occurs when peer side B-device send
 * VBUS pulse on the bus. Use SRP_DETECT event as A-device session valid
 * event, which meet the Min value of the threshold.
 */
static int arava_set_usbotg_a_mask(void)
{
	int ret;
	u8 val;

	ret = arava_read(ARAVA_IRQ_MASK_B, &val);
	if (ret)
		return ret;

	/* set interrupts that a device care about */
	val |= (ARAVA_EVENT_B_VBUS_4P0 | ARAVA_EVENT_B_VBUS_4P4 |
		ARAVA_EVENT_B_SESSION_VALID | ARAVA_EVENT_B_SRP_DETECT);
	val &= ~(ARAVA_EVENT_B_SRP_DETECT | ARAVA_EVENT_B_VBUS_4P4);

	ret = arava_write(ARAVA_IRQ_MASK_B, val);
	return ret;
}

/* Set USB B-device events; Session valid/end
 * Current USB driver doesn't care about any B-device evnets.
 */
static int arava_set_usbotg_b_mask(void)
{
	int ret;
	u8 val;

	ret = arava_read(ARAVA_IRQ_MASK_B, &val);
	if (ret)
		return ret;

	/* set interrupts that b device care about */
	val |= (ARAVA_EVENT_B_VBUS_4P0 | ARAVA_EVENT_B_VBUS_4P4 |
		ARAVA_EVENT_B_SESSION_VALID | ARAVA_EVENT_B_SRP_DETECT);

	ret = arava_write(ARAVA_IRQ_MASK_B, val);
	return ret;
}

static int is_arava_vbus_assert(void)
{
	u8 val;

	/* FIXME: We use EXTON as cable detect signal on ZYlonite.
	 * This depends on the cable signal is connected to EXTON
	 * of ARAVA.
	 * If the cable signal is not connected to EXTON, we need
	 * use other signal as cable detect signal.
	 */
	arava_read(ARAVA_STATUS, &val);
	if (val & ARAVA_STATUS_EXTON)
		return 1;
	return 0;
}

static unsigned int arava_event_change(void)
{
	int ret = 0;
	u8 val;

	arava_read(ARAVA_EVENT_A, &val);

	/* FIXME: We use EXTON as cable detect signal on ZYlonite.
	 * This depends on the cable signal is connected to EXTON
	 * of ARAVA.
	 * If the cable signal is not connected to EXTON, we need
	 * use other signal as cable detect signal.
	 */
	if (val & ARAVA_EVENT_A_EXTON)
		ret |= PMIC_EVENT_VBUS;

	arava_read(ARAVA_EVENT_B, &val);
	if (val & ARAVA_EVENT_B_SRP_DETECT) {
		arava_read(ARAVA_IRQ_MASK_B, &val);
		if (!(val & ARAVA_EVENT_B_SRP_DETECT))
			ret |= PMIC_EVENT_VBUS;
	}

	arava_read(ARAVA_EVENT_C, &val);

	return ret;
}

/* FIXME: Because the ARAVA silicon has some issues. So need check whether
 * IRQ is masked or not before check the VBUS state.
 */
static int is_arava_avbusvld(void)
{
	u8 val;

	arava_read(ARAVA_IRQ_MASK_B, &val);
	if (val & ARAVA_EVENT_B_VBUS_4P4)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_VBUS_VALID_4_4)
		return 1;
	else
		return 0;
}

static int is_arava_asessvld(void)
{
	u8 val;

	arava_read(ARAVA_IRQ_MASK_B, &val);

	if (val & ARAVA_EVENT_B_SESSION_VALID)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_SESSION_VALID)
		return 1;
	else
		return 0;
}

static int is_arava_bsessvld(void)
{
	u8 val;

	arava_read(ARAVA_IRQ_MASK_B, &val);

	if (val & ARAVA_EVENT_B_VBUS_4P0)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_VBUS_VALID_4_0)
		return 1;
	else
		return 0;
}

static int is_arava_srp_ready(void)
{
	u8 val;

	/* ARAVA reports unexpected SRP_DETECT event when
	 * VBUS is pulled HIGH/LOW, no matter whether the
	 * event is cared or not. Reture 0 when USB driver
	 * don't want to detect SRP_DETECT event.
	 */
	arava_read(ARAVA_IRQ_MASK_B, &val);

	if (val & ARAVA_EVENT_B_SRP_DETECT)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_SRP_DETECT)
		return 1;
	else
		return 0;
}

static int get_arava_voltage(int cmd, int *pmv)
{
	int ret;
	u8 val;

	*pmv = 0;
	switch (cmd) {
	case VCC_CORE:
		ret = arava_read(ARAVA_BUCK2DVC1, &val);
		break;
	case VCC_SRAM:
		ret = arava_read(ARAVA_LDO1416, &val);
		break;
	case VCC_MVT:
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_MEM:
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_SDIO:
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_CAMERA_ANA:
		ret = arava_read(ARAVA_LDO17_SIMCP0, &val);
		break;
	default:
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}

	if (ret != 0)
		return ret;

	switch (cmd) {
	case VCC_CORE:
		val &= 0x1f;
		*pmv = val * ARAVA_VBUCK2STEP + ARAVA_VBUCK2BASE;
		break;
	case VCC_SRAM:
		val = val >> 3;
		*pmv = val * ARAVA_VLDO16STEP + ARAVA_VLDO16BASE;
		break;
	case VCC_MVT:
		/* LDO19 is similar to LDO10 */
		val = (val >> 4) & 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_MEM:
		val &= 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		/* LDO18 is similar to LDO10 */
		val &= 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_SDIO:
		/* LDO11 is similar to LDO10 */
		val = (val >> 4) & 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_CAMERA_ANA:
		/* LDO17 is similar to LDO10 */
		val &= 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	default:
		break;
	}
	return ret;
}

int set_arava_voltage(int cmd, int mv)
{
	int ret;
	u8 val;

	switch (cmd) {
	case VCC_CORE:
		if (mv < ARAVA_VBUCK2BASE && mv > ARAVA_VBUCK2MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_BUCK2DVC1, &val);
		break;
	case VCC_SRAM:
		if (mv < ARAVA_VLDO16BASE && mv > ARAVA_VLDO16MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1416, &val);
		break;
	case VCC_MVT:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_MEM:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_SDIO:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_CAMERA_ANA:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO17_SIMCP0, &val);
		break;
	default:
		printk(KERN_INFO "error command\n");
		return -EINVAL;
	}

	if (ret != 0)
		return ret;

	switch (cmd) {
	case VCC_CORE:
		val &= 0xe0;
		val |= ((mv - ARAVA_VBUCK2BASE) / ARAVA_VBUCK2STEP) | 0x80;
		ret = arava_write(ARAVA_BUCK2DVC1, val);
		break;
	case VCC_SRAM:
		val &= 0x07;
		val |= ((mv - ARAVA_VLDO16BASE) / ARAVA_VLDO16STEP) << 3;
		ret = arava_write(ARAVA_LDO1416, val);
		break;
	case VCC_MVT:
		val &= 0x0f;
		val |= ((mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP) << 4;
		ret = arava_write(ARAVA_LDO1819, val);
		break;
	case VCC_MEM:
		val &= 0xf0;
		val |= (mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP;
		ret = arava_write(ARAVA_LDO1011, val);
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		val &= 0xf0;
		val |= (mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP;
		ret = arava_write(ARAVA_LDO1819, val);
		break;
	case VCC_SDIO:
		val &= 0x0f;
		val |= ((mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP) << 4;
		ret = arava_write(ARAVA_LDO1011, val);
		break;
	case VCC_CAMERA_ANA:
		val &= 0xf0;
		val |= (mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP;
		ret = arava_write(ARAVA_LDO17_SIMCP0, val);
		break;
	}
	return ret;
}

static int arava_initchip(void)
{
	int i;

	memset(&arava_regs, 0, sizeof(struct mhn_pmic_regs) * ARAVA_REG_NUM);

	/* set these registers as uncacheable */
	for (i = 0; i < 0x10; i++)
		arava_regs[i].mask = 1;
	/* arava_regs[0x15].mask = 1;
	 */
	arava_regs[0x19].mask = 1;
	for (i = 0x1C; i < 0x20; i++)
		arava_regs[i].mask = 1;
	arava_regs[0x24].mask = 1;
	arava_regs[0x27].mask = 1;
	for (i = 0x2C; i < 0x31; i++)
		arava_regs[i].mask = 1;
	for (i = 0x37; i < 0x50; i++)
		arava_regs[i].mask = 1;
	for (i = 0x51; i < 0x80; i++)
		arava_regs[i].mask = 1;
	for (i = 0x82; i < 0x90; i++)
		arava_regs[i].mask = 1;
	for (i = 0x9C; i < 0xA0; i++)
		arava_regs[i].mask = 1;
	for (i = 0xA7; i < ARAVA_REG_NUM; i++)
		arava_regs[i].mask = 1;

	return arava_write(ARAVA_SYSCTRL_A, 0xE8);
}

static DECLARE_WAIT_QUEUE_HEAD(arava_event_wait);
static int arava_event_handler(void *arg)
{
	unsigned int event;

	DECLARE_WAITQUEUE(wait, current);
	current->flags |= PF_MEMALLOC | PF_NOFREEZE;
	daemonize("arava_event");

	while (1) {
#ifdef CONFIG_PREEMPT_RT
		spin_lock_rt(&arava_lock);
#else
		disable_irq(IRQ_ARAVA_EVENT);
#endif

		event = arava_event_change();
#ifdef CONFIG_USB_PXA3XX
		if (event & PMIC_EVENT_USB) {
			pxa3xx_usb_event_change(event);
		}
#endif

		add_wait_queue(&arava_event_wait, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

#ifdef CONFIG_PREEMPT_RT
		spin_unlock_rt(&arava_lock);
#else
		enable_irq(IRQ_ARAVA_EVENT);
#endif
		schedule();
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&arava_event_wait, &wait);
	}
	return 0;
}

/*
 * Arava interrupt service routine.
 * In the ISR we need to check the Status bits in Arava and according to those
 * bits to check which kind of IRQ had happened.
 */
static irqreturn_t arava_irq_handler(int irq, void *dev_id,
				     struct pt_regs *regs)
{
	/* wakeup event */
	PECR |= PECR_INT_EN;

	wake_up_interruptible(&arava_event_wait);

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
/*
 * Suspend the arava interface.
 */
static int arava_suspend(struct device *_dev, u32 state, u32 level)
{
	if (level == SUSPEND_DISABLE) {
		pr_info("arava: arava suspend\n");
		disable_irq(IRQ_ARAVA_EVENT);
	}
#ifdef CONFIG_ZYLONITE_POWER_OPT
	if (level == SUSPEND_POWER_DOWN) {
		arava_write(ARAVA_REGCTRL1, 0x41);
		arava_write(ARAVA_REGCTRL2, 0x03);
		arava_write(ARAVA_LED1_CTRL, 0x00);
		arava_write(ARAVA_LED2_CTRL, 0x00);
		arava_write(ARAVA_LED3_CTRL, 0x80);
		arava_write(ARAVA_LED4_CTRL, 0x80);
		arava_write(ARAVA_LEDPC_CTRL, 0x80);
	}
#endif

	return 0;
}

/*
 * Resume the arava interface.
 */
static int arava_resume(struct device *_dev, u32 level)
{
	int i;

	if (level == RESUME_ENABLE) {
		pr_info("arava: arava resume\n");
		/* all registers need to be read again */
		for (i = 0; i < ARAVA_REG_NUM; i++) {
			arava_regs[i].hit = 0;
		}
		enable_irq(IRQ_ARAVA_EVENT);
	}
#ifdef CONFIG_ZYLONITE_POWER_OPT
	if (level == RESUME_POWER_ON) {
		arava_write(ARAVA_REGCTRL1, 0xBF);
		arava_write(ARAVA_CON1, 0xBF);
		arava_write(ARAVA_CON2, 0x33);
		arava_write(ARAVA_LED4_CTRL, 0x00);
	}
#endif
	return 0;
}

#else
#define	arava_suspend		NULL
#define	arava_resume		NULL
#endif

static struct pmic_ops arava_pmic_ops = {
	.get_voltage = get_arava_voltage,
	.set_voltage = set_arava_voltage,

	.is_vbus_assert = is_arava_vbus_assert,
	.is_avbusvld = is_arava_avbusvld,
	.is_asessvld = is_arava_asessvld,
	.is_bsessvld = is_arava_bsessvld,
	.is_srp_ready = is_arava_srp_ready,

	.set_pump = arava_set_pump,
	.set_vbus_supply = arava_set_vbus_supply,
	.set_usbotg_a_mask = arava_set_usbotg_a_mask,
	.set_usbotg_b_mask = arava_set_usbotg_b_mask,

#ifdef	CONFIG_PM
	.suspend = arava_suspend,
	.resume = arava_resume,
#endif
};

#ifdef CONFIG_PROC_FS
#define	ARAVA_PROC_FILE	"driver/arava"
static struct proc_dir_entry *arava_proc_file;

static int arava_seq_show(struct seq_file *s, void *p)
{
	u8 val;

	mhn_pmic_read(ARAVA_STATUS, &val);
	seq_printf(s, "Arava status regs: 0x%02x\n", val);

	mhn_pmic_read(ARAVA_IRQ_MASK_A, &val);
	seq_printf(s, "Arava event mask A: 0x%02x\n", val);

	mhn_pmic_read(ARAVA_EVENT_A, &val);
	seq_printf(s, "Arava event A:0x%02x\n", val);

	mhn_pmic_read(ARAVA_EVENT_B, &val);
	seq_printf(s, "Arava event B:0x%02x\n", val);

	mhn_pmic_read(ARAVA_EVENT_C, &val);
	seq_printf(s, "Arava event C:0x%02x\n", val);

	mhn_pmic_read(ARAVA_USBPUMP, &val);
	seq_printf(s, "USB pump:0x%02x\n", val);

	mhn_pmic_read(ARAVA_MISCB, &val);
	seq_printf(s, "Misc control reg:0x%02x\n", val);
	return 0;
}

static int arava_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, &arava_seq_show, NULL);
}

static struct file_operations arava_seq_fops = {
	.owner = THIS_MODULE,
	.open = arava_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_arava_proc_file(void)
{
	arava_proc_file = create_proc_entry(ARAVA_PROC_FILE, 0644, NULL);
	if (!arava_proc_file) {
		printk(KERN_INFO "Create proc file for Arava failed\n");
		return;
	}

	arava_proc_file->proc_fops = &arava_seq_fops;
}

static void remove_arava_proc_file(void)
{
	remove_proc_entry(ARAVA_PROC_FILE, &proc_root);
}
#endif

static int arava_probe(struct device *_dev)
{
	int ret;
	u8 value;

	ret = arava_initchip();
	if (ret != 0) {
		printk(KERN_WARNING "Initialize ARAVA failed\n");
	}

	PECR |= PECR_INT_EN;
	PECR &= ~PECR_DIR;
	ret = request_irq(IRQ_ARAVA_EVENT, arava_irq_handler, 0, "Arava", NULL);

	if (ret) {
		printk(KERN_WARNING
		       "Request IRQ for ARAVA failed, return :%d\n", ret);
		return ret;
	}

	/* Mask interrupts that are not needed */
	arava_write(ARAVA_IRQ_MASK_A, 0xFF);
	arava_write(ARAVA_IRQ_MASK_B, 0xFF);
	arava_write(ARAVA_IRQ_MASK_C, 0xFF);

#ifdef CONFIG_ZYLONITE_POWER_OPT
	arava_write(ARAVA_REGCTRL1, 0xBF);
	arava_write(ARAVA_REGCTRL2, 0x43);
#else
	arava_write(ARAVA_REGCTRL1, 0xFF);
	arava_write(ARAVA_REGCTRL2, 0x43);
	/* On old zylonite board, SRAM LDO doesn't work well.
	 * We have to set ARAVA_APPSLEEP_CTRL, to invoid shutdown SRAM LDO.
	 */
	arava_write(ARAVA_APPSLEEP_CTRL, 0x27);
#endif

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	arava_read(ARAVA_EVENT_A, &value);
	arava_read(ARAVA_EVENT_B, &value);
	arava_read(ARAVA_EVENT_C, &value);

	pmic_set_ops(&arava_pmic_ops);

	spin_lock_init(&arava_lock);

	kernel_thread(arava_event_handler, NULL, CLONE_KERNEL);

#ifdef CONFIG_PROC_FS
	create_arava_proc_file();
#endif
	return 0;
}

static int arava_remove(struct device *_dev)
{
#ifdef	CONFIG_PROC_FS
	remove_arava_proc_file();
#endif
	pmic_set_ops(NULL);
	free_irq(IRQ_ARAVA_EVENT, NULL);
	return 0;
}

static struct device_driver arava_driver = {
	.name = "mhn_pmic",
	.bus = &platform_bus_type,
	.probe = arava_probe,
	.remove = arava_remove,
	.suspend = arava_suspend,
	.resume = arava_resume,
};

/******************************************************************************
 *                                                                            *
 *                              ARAVA I2C Client Driver                       *
 *                                                                            *
 ******************************************************************************/
static int i2c_arava_attach_adapter(struct i2c_adapter *adapter);
static int i2c_arava_detect_client(struct i2c_adapter *, int, int);
static int i2c_arava_detach_client(struct i2c_client *client);
#define I2C_DRIVERID_ARAVA   I2C_DRIVERID_EXP1

static struct i2c_driver i2c_arava_driver = {
	.owner = THIS_MODULE,
	.name = "arava i2c client driver",
	.id = I2C_DRIVERID_ARAVA,
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = &i2c_arava_attach_adapter,
	.detach_client = &i2c_arava_detach_client,
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = { ARAVA_ADDRESS, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

int arava_read(u8 reg, u8 * pval)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -EINVAL;

	pr_debug("hit %d\n", arava_regs[reg].hit);
	if (arava_regs[reg].hit) {
		*pval = arava_regs[reg].data;
		return 0;
	}

	ret = i2c_smbus_read_byte_data(g_client, reg);
	pr_debug("i2c read ret:0x%x\n", ret);
	if (ret >= 0) {
		*pval = ret;
		/* If a register is uncacheable, the hit field can't be set */
		arava_regs[reg].hit = ~arava_regs[reg].mask;
		arava_regs[reg].data = ret;
		ret = 0;
	} else
		ret = -EIO;

	return ret;
}

int arava_write(u8 reg, u8 val)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	if (ret == 0) {
		/* If a register is uncacheable, the hit field can't be set */
		arava_regs[reg].hit = ~arava_regs[reg].mask;
		/* arava_regs[reg].hit = 0; */
		arava_regs[reg].data = val;
		ret = 0;
	} else
		ret = -EIO;

	return ret;
}

static int i2c_arava_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &i2c_arava_detect_client);
}

static int i2c_arava_detect_client(struct i2c_adapter *adapter, int address,
				   int kind)
{
	struct i2c_client *new_client;
	int err = 0;
	int chip_id;

	/* Let's see whether this adapter can support what we need.
	   Please substitute the things you need here!  */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_info("byte op is not permited.\n");
		goto ERROR0;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet.
	   But it allows us to access several i2c functions safely */

	/* Note that we reserve some space for arava_data too. If you don't
	   need it, remove it. We do it here to help to lessen memory
	   fragmentation. */

	new_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client) {
		err = -ENOMEM;
		goto ERROR0;
	}

	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &i2c_arava_driver;
	new_client->flags = 0;

	chip_id = i2c_smbus_read_byte_data(new_client, ARAVA_CHIP_ID);
	if (chip_id < 0) {
		printk(KERN_ERR "arava unavailable!\n");
		goto ERROR1;
	} else {
		pr_info("arava(chip id:0x%02x) detected.\n", chip_id);
	}

	g_client = new_client;

	strcpy(new_client->name, "ARAVA");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	return 0;

      ERROR1:
	g_client = NULL;
	kfree(new_client);
      ERROR0:
	return err;
}

static int i2c_arava_detach_client(struct i2c_client *client)
{
	int err;

	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk(KERN_WARNING
		       "arava.o: Client deregistration failed, client not detached.\n");
		return err;
	}

	/* Frees client data too, if allocated at the same time */
	kfree(client);
	g_client = NULL;
	return 0;
}

static int __init arava_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&i2c_arava_driver))) {
		printk(KERN_WARNING
		       "arava: Driver registration failed,module not inserted.\n");
		return ret;
	}
	ret = driver_register(&arava_driver);

	return ret;
}

static void __exit arava_exit(void)
{
	driver_unregister(&arava_driver);

	if (i2c_del_driver(&i2c_arava_driver)) {
		printk(KERN_WARNING
		       "arava: Driver registration failed, module not removed.\n");
	}
}

module_init(arava_init);
module_exit(arava_exit);

MODULE_DESCRIPTION("Arava Driver");
MODULE_LICENSE("GPL");
