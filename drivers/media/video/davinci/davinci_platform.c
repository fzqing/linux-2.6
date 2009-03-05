/*
 * Copyright (C) 2007 Texas Instruments Inc
 *
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

#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/arch/cpu.h>
#include <asm/arch/io.h>
#include <asm/io.h>
#include <media/davinci/davinci_enc.h>
#include <media/davinci/vid_encoder_types.h>
#include <video/davinci_vpbe.h>
#include <video/davinci_osd.h>
#include <media/davinci/davinci_enc_mngr.h>
#include <media/davinci/davinci_platform.h>

extern struct vid_enc_device_mgr enc_dev[];

struct enc_config davinci_enc_default[DAVINCI_ENC_MAX_CHANNELS] = {
	{VID_ENC_OUTPUT_COMPOSITE,
	 VID_ENC_STD_NTSC}
};

EXPORT_SYMBOL(davinci_enc_default);

char *davinci_outputs[] = {
	VID_ENC_OUTPUT_COMPOSITE,
	VID_ENC_OUTPUT_COMPOSITE1,
	VID_ENC_OUTPUT_SVIDEO,
	VID_ENC_OUTPUT_SVIDEO1,
	VID_ENC_OUTPUT_COMPONENT,
	VID_ENC_OUTPUT_COMPONENT1,
	VID_ENC_OUTPUT_LCD,
	VID_ENC_OUTPUT_LCD1,
	""
};

EXPORT_SYMBOL(davinci_outputs);

char *davinci_modes[] = {
	VID_ENC_STD_NTSC,
	"ntsc",
	VID_ENC_STD_NTSC_RGB,
	VID_ENC_STD_PAL,
	"pal",
	VID_ENC_STD_PAL_RGB,
	VID_ENC_STD_720P_25,
	VID_ENC_STD_720P_30,
	VID_ENC_STD_720P_50,
	VID_ENC_STD_720P_60,
	VID_ENC_STD_1080I_25,
	VID_ENC_STD_1080I_30,
	VID_ENC_STD_1080P_25,
	VID_ENC_STD_1080P_30,
	VID_ENC_STD_1080P_50,
	VID_ENC_STD_1080P_60,
	VID_ENC_STD_480P_60,
	VID_ENC_STD_576P_50,
	VID_ENC_STD_640x480,
	VID_ENC_STD_640x400,
	VID_ENC_STD_640x350,
	""
};
EXPORT_SYMBOL(davinci_modes);

static __inline__ u32 dispc_reg_in(u32 offset)
{
	if (cpu_is_davinci_dm355())
		return (davinci_readl(DM355_VENC_REG_BASE + offset));
	else
		return (davinci_readl(DM644X_VENC_REG_BASE + offset));
}

static __inline__ u32 dispc_reg_out(u32 offset, u32 val)
{
	if (cpu_is_davinci_dm355())
		davinci_writel(val, (DM355_VENC_REG_BASE + offset));
	else
		davinci_writel(val, (DM644X_VENC_REG_BASE + offset));
	return (val);
}

static __inline__ u32 dispc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr, new_val;

	if (cpu_is_davinci_dm355())
		addr = DM355_VENC_REG_BASE + offset;
	else
		addr = DM644X_VENC_REG_BASE + offset;

	new_val = (davinci_readl(addr) & ~mask) | (val & mask);
	davinci_writel(new_val, addr);
	return (new_val);
}

#ifdef CONFIG_SYSFS

static spinlock_t reg_access_lock;
static void davinci_enc_set_basep(int channel, unsigned basepx, unsigned basepy)
{
	spin_lock(&reg_access_lock);
	if (cpu_is_davinci_dm355()) {
		davinci_writel((basepx & OSD_BASEPX_BPX),
			       (DM355_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel((basepy & OSD_BASEPY_BPY),
			       (DM355_OSD_REG_BASE + OSD_BASEPY));
	} else if (cpu_is_davinci_dm644x()) {
		davinci_writel((basepx & OSD_BASEPX_BPX),
			       (DM644X_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel((basepy & OSD_BASEPY_BPY),
			       (DM644X_OSD_REG_BASE + OSD_BASEPY));
	} else {
		printk(KERN_WARNING "Unsupported platform\n");
	}
	spin_unlock(&reg_access_lock);
}

static void davinci_enc_get_basep(int channel, unsigned *basepx,
				  unsigned *basepy)
{
	spin_lock(&reg_access_lock);
	if (cpu_is_davinci_dm355()) {
		*basepx =
		    (OSD_BASEPX_BPX &
		     davinci_readl(DM355_OSD_REG_BASE + OSD_BASEPX));
		*basepy =
		    (OSD_BASEPY_BPY &
		     davinci_readl(DM355_OSD_REG_BASE + OSD_BASEPY));
	} else if (cpu_is_davinci_dm644x()) {
		*basepx =
		    (OSD_BASEPX_BPX &
		     davinci_readl(DM644X_OSD_REG_BASE + OSD_BASEPX));
		*basepy =
		    (OSD_BASEPY_BPY &
		     davinci_readl(DM644X_OSD_REG_BASE + OSD_BASEPY));
	} else {
		*basepx = 0;
		*basepy = 0;
		printk(KERN_WARNING "Unsupported platform\n");
	}
	spin_unlock(&reg_access_lock);
}

struct system_device {
	struct module *owner;
	struct class_device class_dev;
};

static struct system_device *davinci_system_device;

#define to_system_dev(cdev)	container_of(cdev, \
 struct system_device, class_dev)

static void davinci_system_class_release(struct class_device *cdev)
{
	struct system_device *dev = to_system_dev(cdev);

	if (dev != NULL)
		kfree(dev);
}

static void __iomem *display_cntl_base;

struct class davinci_system_class = {
	.name = "davinci_system",
	.release = davinci_system_class_release,
};

static ssize_t
reg_store(struct class_device *cdev, const char *buffer, size_t count)
{
	char *str = 0;
	char *bufv = 0;
	int addr = 0;
	int val = 0;
	int len = 0;

	if (!buffer || (count == 0) || (count >= 128))
		return 0;

	str = kmalloc(128, GFP_KERNEL);
	if (0 == str)
		return -ENOMEM;

	strcpy(str, buffer);
	/* overwrite the '\n' */
	strcpy(str + count - 1, "\0");

	/* format: <address> [<value>]
	   if only <address> present, it is a read
	   if <address> <value>, then it is a write */
	len = strcspn(str, " ");
	addr = simple_strtoul(str, NULL, 16);

	if (len != count - 1) {
		bufv = str;
		strsep(&bufv, " ");
		val = simple_strtoul(bufv, NULL, 16);
	}

	kfree(str);

	/* for now, restrict this to access DDR2 controller
	   Peripheral Bust Burst Priority Register PBBPR
	   (addr: 0x20000020) only */
	if (addr != (DM644X_DDR2_CNTL_BASE + 0x20))
		return -EINVAL;

	spin_lock(&reg_access_lock);
	if (bufv != 0)
		writel(val, display_cntl_base + addr - DM644X_DDR2_CNTL_BASE);
	printk(KERN_NOTICE "%05x  %08x\n", addr,
	       readl(display_cntl_base + addr - DM644X_DDR2_CNTL_BASE));
	spin_unlock(&reg_access_lock);

	return count;
}

static ssize_t reg_show(struct class_device *cdev, char *buf)
{
	return 0;
}

static ssize_t osd_basepx_show(struct class_device *cdev, char *buf)
{
	unsigned int basepx, basepy;
	int p;

	davinci_enc_get_basep(0, &basepx, &basepy);
	p = sprintf(buf, "%d\n", basepx);
	return p;
}

static ssize_t osd_basepx_store(struct class_device *cdev, const char *buffer,
				size_t count)
{
	unsigned int basepx, basepy;
	char reg_val[10];

	if (count >= 9) {
		strncpy(reg_val, buffer, 9);
		reg_val[9] = '\0';
	} else {
		/* overwrite the '\n' */
		strcpy(reg_val, buffer);
		strcpy(reg_val + count - 1, "\0");
	}
	davinci_enc_get_basep(0, &basepx, &basepy);
	basepx = simple_strtoul(reg_val, NULL, 10);

	if (basepx > OSD_BASEPX_BPX) {
		printk(KERN_ERR "Invalid value for OSD basepx\n");
		return count;
	}
	davinci_enc_set_basep(0, basepx, basepy);
	return count;
}

static ssize_t osd_basepy_show(struct class_device *cdev, char *buf)
{
	unsigned int basepx, basepy;
	int p;

	davinci_enc_get_basep(0, &basepx, &basepy);
	p = sprintf(buf, "%d\n", basepy);
	return p;
}

static ssize_t osd_basepy_store(struct class_device *cdev, const char *buffer,
				size_t count)
{
	unsigned int basepx, basepy;
	char reg_val[10];

	if (count >= 9) {
		strncpy(reg_val, buffer, 9);
		reg_val[9] = '\0';
	} else {
		/* overwrite the '\n' */
		strcpy(reg_val, buffer);
		strcpy(reg_val + count - 1, "\0");
	}

	davinci_enc_get_basep(0, &basepx, &basepy);
	basepy = simple_strtoul(reg_val, NULL, 10);
	if (basepy > OSD_BASEPY_BPY) {
		printk(KERN_ERR "Invalid value for OSD basepy\n");
		return count;
	}
	davinci_enc_set_basep(0, basepx, basepy);
	return count;
}

#define DECLARE_ATTR(_name, _mode, _show, _store) {		\
	.attr   = { .name = __stringify(_name), .mode = _mode,	\
		    .owner = THIS_MODULE },  			\
	.show   = _show,                                        \
	.store  = _store,}

static struct class_device_attribute system_class_device_attributes[] = {
	DECLARE_ATTR(reg, S_IRWXUGO, reg_show, reg_store),
	DECLARE_ATTR(vpbe_osd_basepx, S_IRWXUGO, osd_basepx_show,
		     osd_basepx_store),
	DECLARE_ATTR(vpbe_osd_basepy, S_IRWXUGO, osd_basepy_show,
		     osd_basepy_store)
};

static void *create_sysfs_files(void)
{
	struct system_device *dev;
	int ret;
	int i;

	dev = kzalloc(sizeof(struct system_device), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->owner = THIS_MODULE;
	dev->class_dev.class = &davinci_system_class;
	snprintf(dev->class_dev.class_id, BUS_ID_SIZE, "system");
	ret = class_device_register(&dev->class_dev);
	if (ret < 0) {
		printk(KERN_ERR "Error in class_device_register\n");
		kfree(dev);
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(system_class_device_attributes); i++) {
		ret = class_device_create_file(&dev->class_dev,
					       &system_class_device_attributes
					       [i]);
		if (ret < 0) {
			while (--i >= 0)
				class_device_remove_file(&dev->class_dev,
					&system_class_device_attributes[i]);
			class_device_unregister(&dev->class_dev);
			printk(KERN_ERR "Error in class_device_create_file\n");
			return NULL;
		}
	}

	return dev;
}

static void remove_sysfs_files(struct system_device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(system_class_device_attributes); i++)
		class_device_remove_file(&dev->class_dev,
					 &system_class_device_attributes[i]);

	class_device_unregister(&dev->class_dev);
}
#endif

/**
 * function davinci_enc_select_venc_clk
 * @clk_source: clock source defined by davinci_enc_clk_source_type
 * Returns: Zero if successful, or non-zero otherwise
 *
 * Description:
 *  Select the venc input clock based on the clk_source_type.
 */
int davinci_enc_select_venc_clock(int clk)
{
	return 0;
}

EXPORT_SYMBOL(davinci_enc_select_venc_clock);

static void enableDigitalOutput(int bEnable)
{
	if (bEnable) {
		dispc_reg_out(VENC_VMOD, 0);
		dispc_reg_out(VENC_CVBS, 0);

		if (!cpu_is_davinci_dm355()) {
			davinci_writel(0, (DM644X_VPBE_REG_BASE + VPBE_PCR));
		}
		dispc_reg_out(VENC_LCDOUT, 0);
		dispc_reg_out(VENC_HSPLS, 0);
		dispc_reg_out(VENC_HSTART, 0);
		dispc_reg_out(VENC_HVALID, 0);
		dispc_reg_out(VENC_HINT, 0);
		dispc_reg_out(VENC_VSPLS, 0);
		dispc_reg_out(VENC_VSTART, 0);
		dispc_reg_out(VENC_VVALID, 0);
		dispc_reg_out(VENC_VINT, 0);
		dispc_reg_out(VENC_YCCCTL, 0);
		dispc_reg_out(VENC_DACSEL, 0);

	} else {
		/* Initialize the VPSS Clock Control register */
		davinci_writel(0x18, SYS_VPSS_CLKCTL);
		davinci_writel(0, (DM644X_VPBE_REG_BASE + VPBE_PCR));

		/* Set PINMUX0 reg to enable LCD (all other settings are kept
		   per boot)
		 */
		if (!cpu_is_davinci_dm355()) {
			davinci_cfg_reg(DM644X_LOEEN);
			davinci_cfg_reg(DM644X_LFLDEN);
		}

		/* disable VCLK output pin enable */
		dispc_reg_out(VENC_VIDCTL, 0x141);

		/* Disable output sync pins */
		dispc_reg_out(VENC_SYNCCTL, 0);

		/* Disable DCLOCK */
		dispc_reg_out(VENC_DCLKCTL, 0);
		dispc_reg_out(VENC_DRGBX1, 0x0000057C);

		/* Disable LCD output control (accepting default polarity) */
		dispc_reg_out(VENC_LCDOUT, 0);
		if (!cpu_is_davinci_dm355()) {
			dispc_reg_out(VENC_CMPNT, 0x100);
		}

		dispc_reg_out(VENC_HSPLS, 0);
		dispc_reg_out(VENC_HINT, 0);
		dispc_reg_out(VENC_HSTART, 0);
		dispc_reg_out(VENC_HVALID, 0);

		dispc_reg_out(VENC_VSPLS, 0);
		dispc_reg_out(VENC_VINT, 0);
		dispc_reg_out(VENC_VSTART, 0);
		dispc_reg_out(VENC_VVALID, 0);

		dispc_reg_out(VENC_HSDLY, 0);
		dispc_reg_out(VENC_VSDLY, 0);

		dispc_reg_out(VENC_YCCCTL, 0);
		dispc_reg_out(VENC_VSTARTA, 0);

		/* Set OSD clock and OSD Sync Adavance registers */
		dispc_reg_out(VENC_OSDCLK0, 1);
		dispc_reg_out(VENC_OSDCLK1, 2);
	}
}

/*
 * setting NTSC mode
 */

static void davinci_enc_set_ntsc(struct vid_enc_mode_info *mode_info)
{
	enableDigitalOutput(0);

	if (cpu_is_davinci_dm355()) {
		dispc_reg_out(VENC_CLKCTL, 0x01);
		dispc_reg_out(VENC_VIDCTL, 0);
		/* DM 350 Configure VDAC_CONFIG , why ?? */
		davinci_writel(0x0E21A6B6, DM355_VDAC_CONFIG);
	} else {
		/* to set VENC CLK DIV to 1 - final clock is 54 MHz */
		dispc_reg_merge(VENC_VIDCTL, 0, 1 << 1);
		/* Set REC656 Mode */
		dispc_reg_out(VENC_YCCCTL, 0x1);
		dispc_reg_merge(VENC_VDPRO, 0, VENC_VDPRO_DAFRQ);
		dispc_reg_merge(VENC_VDPRO, 0, VENC_VDPRO_DAUPS);
	}
	if (cpu_is_davinci_dm355()) {
		davinci_writel(mode_info->left_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPY));
	} else {
		davinci_writel(mode_info->left_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPY));
	}
	dispc_reg_merge(VENC_VMOD, VENC_VMOD_VENC, VENC_VMOD_VENC);
}

/*
 * setting PAL mode
 */
static void davinci_enc_set_pal(struct vid_enc_mode_info *mode_info)
{

	enableDigitalOutput(0);

	if (cpu_is_davinci_dm355()) {
		dispc_reg_out(VENC_CLKCTL, 0x1);
		dispc_reg_out(VENC_VIDCTL, 0);
		/* DM350 Configure VDAC_CONFIG  */
		davinci_writel(0x0E21A6B6, DM355_VDAC_CONFIG);
	} else {
		/* to set VENC CLK DIV to 1 - final clock is 54 MHz */
		dispc_reg_merge(VENC_VIDCTL, 0, 1 << 1);
		/* Set REC656 Mode */
		dispc_reg_out(VENC_YCCCTL, 0x1);
	}

	dispc_reg_merge(VENC_SYNCCTL, 1 << VENC_SYNCCTL_OVD_SHIFT,
			VENC_SYNCCTL_OVD);

	if (cpu_is_davinci_dm355()) {
		davinci_writel(mode_info->left_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPY));
	} else {
		davinci_writel(mode_info->left_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPY));
	}

	dispc_reg_merge(VENC_VMOD, VENC_VMOD_VENC, VENC_VMOD_VENC);
	dispc_reg_out(VENC_DACTST, 0x0);
}

/*
 * davinci_enc_ntsc_pal_rgb
 */
/* This function configures the video encoder to NTSC RGB setting.*/
static void davinci_enc_set_ntsc_pal_rgb(struct vid_enc_mode_info *mode_info)
{

	enableDigitalOutput(0);

	davinci_writel(mode_info->left_margin,
		       (DM644X_OSD_REG_BASE + OSD_BASEPX));
	davinci_writel(mode_info->upper_margin,
		       (DM644X_OSD_REG_BASE + OSD_BASEPY));
}

/*
 * davinci_enc_set_525p
 */
/* This function configures the video encoder to HDTV(525p) component setting.*/
static void davinci_enc_set_525p(struct vid_enc_mode_info *mode_info)
{
	enableDigitalOutput(0);
	davinci_writel(0x19, SYS_VPSS_CLKCTL);

	davinci_writel(mode_info->left_margin,
		       (DM644X_OSD_REG_BASE + OSD_BASEPX));
	davinci_writel(mode_info->upper_margin,
		       (DM644X_OSD_REG_BASE + OSD_BASEPY));

	davinci_writel(VPBE_PCR_VENC_DIV, (DM644X_VPBE_REG_BASE + VPBE_PCR));
	dispc_reg_out(VENC_OSDCLK0, 0);
	dispc_reg_out(VENC_OSDCLK1, 1);

	dispc_reg_merge(VENC_VDPRO, VENC_VDPRO_DAFRQ, VENC_VDPRO_DAFRQ);
	dispc_reg_merge(VENC_VDPRO, VENC_VDPRO_DAUPS, VENC_VDPRO_DAUPS);

	dispc_reg_merge(VENC_VMOD,
			VENC_VMOD_VDMD_YCBCR8 <<
			VENC_VMOD_VDMD_SHIFT, VENC_VMOD_VDMD);

	/* Set REC656 Mode */
	dispc_reg_out(VENC_YCCCTL, 0x1);
	dispc_reg_merge(VENC_VMOD, VENC_VMOD_VENC, VENC_VMOD_VENC);
}

/*
 *  davinci_enc_set_625p
 */
/* This function configures the video encoder to HDTV(625p) component setting.*/
static void davinci_enc_set_625p(struct vid_enc_mode_info *mode_info)
{
	enableDigitalOutput(0);
	davinci_writel(0x19, SYS_VPSS_CLKCTL);

	davinci_writel(mode_info->left_margin,
		       (DM644X_OSD_REG_BASE + OSD_BASEPX));
	davinci_writel(mode_info->upper_margin,
		       (DM644X_OSD_REG_BASE + OSD_BASEPY));

	davinci_writel(VPBE_PCR_VENC_DIV, (DM644X_VPBE_REG_BASE + VPBE_PCR));
	dispc_reg_out(VENC_OSDCLK0, 0);
	dispc_reg_out(VENC_OSDCLK1, 1);

	dispc_reg_merge(VENC_VDPRO, VENC_VDPRO_DAFRQ, VENC_VDPRO_DAFRQ);
	dispc_reg_merge(VENC_VDPRO, VENC_VDPRO_DAUPS, VENC_VDPRO_DAUPS);

	dispc_reg_merge(VENC_VMOD,
			VENC_VMOD_VDMD_YCBCR8 <<
			VENC_VMOD_VDMD_SHIFT, VENC_VMOD_VDMD);

	/* Set REC656 Mode */
	dispc_reg_out(VENC_YCCCTL, 0x1);
	dispc_reg_merge(VENC_VMOD, VENC_VMOD_VENC, VENC_VMOD_VENC);
}

/*
 * davinci_enc_set_display_timing
 */
/* This function sets the display timing from the fb_info structure*/
void davinci_enc_set_display_timing(struct vid_enc_mode_info *mode)
{
	dispc_reg_out(VENC_HSPLS, mode->hsync_len);
	dispc_reg_out(VENC_HSTART, mode->left_margin);
	dispc_reg_out(VENC_HVALID, mode->xres);
	dispc_reg_out(VENC_HINT,
		      mode->xres + mode->left_margin + mode->right_margin - 1);

	dispc_reg_out(VENC_VSPLS, mode->vsync_len);
	dispc_reg_out(VENC_VSTART, mode->upper_margin);
	dispc_reg_out(VENC_VVALID, mode->yres);
	dispc_reg_out(VENC_VINT,
		      mode->yres + mode->upper_margin + mode->lower_margin);
};

EXPORT_SYMBOL(davinci_enc_set_display_timing);

/*
 * setting DLCD 480P PRGB mode
 */
static void davinci_enc_set_prgb(struct vid_enc_mode_info *mode_info)
{

	enableDigitalOutput(1);

	dispc_reg_out(VENC_VIDCTL, 0x141);
	/* set VPSS clock */
	davinci_writel(0x18, SYS_VPSS_CLKCTL);

	dispc_reg_out(VENC_DCLKCTL, 0);
	dispc_reg_out(VENC_DCLKPTN0, 0);

	/* Set the OSD Divisor to 1. */
	dispc_reg_out(VENC_OSDCLK0, 0);
	dispc_reg_out(VENC_OSDCLK1, 1);
	/* Clear composite mode register */
	dispc_reg_out(VENC_CVBS, 0);

	/* Set PINMUX1 to enable all outputs needed to support RGB666 */
	if (cpu_is_davinci_dm355()) {
		/* Enable the venc and dlcd clocks. */
		dispc_reg_out(VENC_CLKCTL, 0x11);
		davinci_cfg_reg(DM355_VOUT_FIELD_G70);
		davinci_cfg_reg(DM355_VOUT_COUTL_EN);
		davinci_cfg_reg(DM355_VOUT_COUTH_EN);
	} else {
		dispc_reg_out(VENC_CMPNT, 0x100);
		davinci_cfg_reg(DM644X_GPIO46_47);
		davinci_cfg_reg(DM644X_GPIO0);
		davinci_cfg_reg(DM644X_RGB666);
		davinci_cfg_reg(DM644X_LOEEN);
		davinci_cfg_reg(DM644X_GPIO3);
	}

	if (cpu_is_davinci_dm355()) {
		davinci_writel(mode_info->left_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPY));
	} else {
		davinci_writel(mode_info->left_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPY));
	}

	/* Set VIDCTL to select VCLKE = 1,
	   VCLKZ =0, SYDIR = 0 (set o/p), DOMD = 0 */
	dispc_reg_merge(VENC_VIDCTL, 1 << VENC_VIDCTL_VCLKE_SHIFT,
			VENC_VIDCTL_VCLKE);
	dispc_reg_merge(VENC_VIDCTL, 0 << VENC_VIDCTL_VCLKZ_SHIFT,
			VENC_VIDCTL_VCLKZ);
	dispc_reg_merge(VENC_VIDCTL, 0 << VENC_VIDCTL_SYDIR_SHIFT,
			VENC_VIDCTL_SYDIR);
	dispc_reg_merge(VENC_VIDCTL, 0 << VENC_VIDCTL_YCDIR_SHIFT,
			VENC_VIDCTL_YCDIR);

	dispc_reg_merge(VENC_DCLKCTL,
			1 << VENC_DCLKCTL_DCKEC_SHIFT, VENC_DCLKCTL_DCKEC);

	dispc_reg_out(VENC_DCLKPTN0, 0x1);

	davinci_enc_set_display_timing(mode_info);
	dispc_reg_out(VENC_SYNCCTL,
		      (VENC_SYNCCTL_SYEV |
		       VENC_SYNCCTL_SYEH | VENC_SYNCCTL_HPL
		       | VENC_SYNCCTL_VPL));

	/* Configure VMOD. No change in VENC bit */
	dispc_reg_out(VENC_VMOD, 0x2011);
	dispc_reg_out(VENC_LCDOUT, 0x1);

}

/*
 *
 */
static void davinci_enc_set_720p(struct vid_enc_mode_info *mode_info)
{
	/* Reset video encoder module */
	dispc_reg_out(VENC_VMOD, 0);

	enableDigitalOutput(1);

	dispc_reg_out(VENC_VIDCTL, (VENC_VIDCTL_VCLKE | VENC_VIDCTL_VCLKP));
	/* Setting DRGB Matrix registers back to default values */
	dispc_reg_out(VENC_DRGBX0, 0x00000400);
	dispc_reg_out(VENC_DRGBX1, 0x00000576);
	dispc_reg_out(VENC_DRGBX2, 0x00000159);
	dispc_reg_out(VENC_DRGBX3, 0x000002cb);
	dispc_reg_out(VENC_DRGBX4, 0x000006ee);

	/* Enable DCLOCK */
	dispc_reg_out(VENC_DCLKCTL, VENC_DCLKCTL_DCKEC);
	/* Set DCLOCK pattern */
	dispc_reg_out(VENC_DCLKPTN0, 1);
	dispc_reg_out(VENC_DCLKPTN1, 0);
	dispc_reg_out(VENC_DCLKPTN2, 0);
	dispc_reg_out(VENC_DCLKPTN3, 0);
	dispc_reg_out(VENC_DCLKPTN0A, 2);
	dispc_reg_out(VENC_DCLKPTN1A, 0);
	dispc_reg_out(VENC_DCLKPTN2A, 0);
	dispc_reg_out(VENC_DCLKPTN3A, 0);
	dispc_reg_out(VENC_DCLKHS, 0);
	dispc_reg_out(VENC_DCLKHSA, 1);
	dispc_reg_out(VENC_DCLKHR, 0);
	dispc_reg_out(VENC_DCLKVS, 0);
	dispc_reg_out(VENC_DCLKVR, 0);
	/* Set brightness start position and pulse width to zero */
	dispc_reg_out(VENC_BRTS, 0);
	dispc_reg_out(VENC_BRTW, 0);
	/* Set LCD AC toggle interval and horizontal position to zero */
	dispc_reg_out(VENC_ACCTL, 0);

	/* Set PWM period and width to zero */
	dispc_reg_out(VENC_PWMP, 0);
	dispc_reg_out(VENC_PWMW, 0);

	dispc_reg_out(VENC_CVBS, 0);
	dispc_reg_out(VENC_CMPNT, 0);
	/* turning on horizontal and vertical syncs */
	dispc_reg_out(VENC_SYNCCTL, (VENC_SYNCCTL_SYEV | VENC_SYNCCTL_SYEH));
	dispc_reg_out(VENC_OSDCLK0, 0);
	dispc_reg_out(VENC_OSDCLK1, 1);
	dispc_reg_out(VENC_OSDHADV, 0);

	dispc_reg_out(VENC_HSDLY, 0);
	dispc_reg_out(VENC_VSDLY, 0);
	dispc_reg_out(VENC_YCCCTL, 0);
	dispc_reg_out(VENC_VSTARTA, 13);

	davinci_writel(0xa, SYS_VPSS_CLKCTL);
	if (cpu_is_davinci_dm355()) {
		dispc_reg_out(VENC_CLKCTL, 0x11);
		davinci_writel(mode_info->left_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPY));
		davinci_cfg_reg(DM355_VOUT_FIELD_G70);
		davinci_cfg_reg(DM355_VOUT_COUTL_EN);
		davinci_cfg_reg(DM355_VOUT_COUTH_EN);
	} else {
		davinci_writel(mode_info->left_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPY));
		davinci_cfg_reg(DM644X_LOEEN);
		davinci_cfg_reg(DM644X_GPIO3);
	}

	/* Set VENC for non-standard timing */
	davinci_enc_set_display_timing(mode_info);

	dispc_reg_out(VENC_HSDLY, 0);
	dispc_reg_out(VENC_VSDLY, 0);
	dispc_reg_out(VENC_YCCCTL, 0);
	dispc_reg_out(VENC_VSTARTA, 0);

	/* Enable all VENC, non-standard timing mode, master timing, HD,
	   progressive
	 */
	if (cpu_is_davinci_dm355()) {
		dispc_reg_out(VENC_VMOD, (VENC_VMOD_VENC | VENC_VMOD_VMD));
	} else {
		dispc_reg_out(VENC_VMOD,
			      (VENC_VMOD_VENC | VENC_VMOD_VMD |
			       VENC_VMOD_HDMD));
	}
	dispc_reg_out(VENC_LCDOUT, 1);
}

/*
 *
 */
static void davinci_enc_set_1080i(struct vid_enc_mode_info *mode_info)
{
	/* Reset video encoder module */
	dispc_reg_out(VENC_VMOD, 0);

	enableDigitalOutput(1);
	dispc_reg_out(VENC_VIDCTL, (VENC_VIDCTL_VCLKE | VENC_VIDCTL_VCLKP));
	/* Setting DRGB Matrix registers back to default values */
	dispc_reg_out(VENC_DRGBX0, 0x00000400);
	dispc_reg_out(VENC_DRGBX1, 0x00000576);
	dispc_reg_out(VENC_DRGBX2, 0x00000159);
	dispc_reg_out(VENC_DRGBX3, 0x000002cb);
	dispc_reg_out(VENC_DRGBX4, 0x000006ee);
	/* Enable DCLOCK */
	dispc_reg_out(VENC_DCLKCTL, VENC_DCLKCTL_DCKEC);
	/* Set DCLOCK pattern */
	dispc_reg_out(VENC_DCLKPTN0, 1);
	dispc_reg_out(VENC_DCLKPTN1, 0);
	dispc_reg_out(VENC_DCLKPTN2, 0);
	dispc_reg_out(VENC_DCLKPTN3, 0);
	dispc_reg_out(VENC_DCLKPTN0A, 2);
	dispc_reg_out(VENC_DCLKPTN1A, 0);
	dispc_reg_out(VENC_DCLKPTN2A, 0);
	dispc_reg_out(VENC_DCLKPTN3A, 0);
	dispc_reg_out(VENC_DCLKHS, 0);
	dispc_reg_out(VENC_DCLKHSA, 1);
	dispc_reg_out(VENC_DCLKHR, 0);
	dispc_reg_out(VENC_DCLKVS, 0);
	dispc_reg_out(VENC_DCLKVR, 0);
	/* Set brightness start position and pulse width to zero */
	dispc_reg_out(VENC_BRTS, 0);
	dispc_reg_out(VENC_BRTW, 0);
	/* Set LCD AC toggle interval and horizontal position to zero */
	dispc_reg_out(VENC_ACCTL, 0);

	/* Set PWM period and width to zero */
	dispc_reg_out(VENC_PWMP, 0);
	dispc_reg_out(VENC_PWMW, 0);

	dispc_reg_out(VENC_CVBS, 0);
	dispc_reg_out(VENC_CMPNT, 0);
	/* turning on horizontal and vertical syncs */
	dispc_reg_out(VENC_SYNCCTL, (VENC_SYNCCTL_SYEV | VENC_SYNCCTL_SYEH));
	dispc_reg_out(VENC_OSDCLK0, 0);
	dispc_reg_out(VENC_OSDCLK1, 1);
	dispc_reg_out(VENC_OSDHADV, 0);

	davinci_writel(0xa, SYS_VPSS_CLKCTL);
	if (cpu_is_davinci_dm355()) {
		dispc_reg_out(VENC_CLKCTL, 0x11);
		davinci_writel(mode_info->left_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM355_OSD_REG_BASE + OSD_BASEPY));
		davinci_cfg_reg(DM355_VOUT_FIELD);
		davinci_cfg_reg(DM355_VOUT_COUTL_EN);
		davinci_cfg_reg(DM355_VOUT_COUTH_EN);
	} else {
		davinci_writel(mode_info->left_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPX));
		davinci_writel(mode_info->upper_margin,
			       (DM644X_OSD_REG_BASE + OSD_BASEPY));
		davinci_cfg_reg(DM644X_LFLDEN);
	}

	/* Set VENC for non-standard timing */
	davinci_enc_set_display_timing(mode_info);

	/* Enable all VENC, non-standard timing mode, master timing,
	   HD, interlaced
	 */
	if (cpu_is_davinci_dm355()) {
		dispc_reg_out(VENC_VMOD,
			      (VENC_VMOD_VENC | VENC_VMOD_VMD |
			       VENC_VMOD_NSIT));
	} else {
		dispc_reg_out(VENC_VMOD,
			      (VENC_VMOD_VENC | VENC_VMOD_VMD | VENC_VMOD_HDMD |
			       VENC_VMOD_NSIT));
	}
	dispc_reg_out(VENC_LCDOUT, 1);
}

void davinci_enc_priv_setmode(struct vid_enc_device_mgr *mgr)
{

	switch (mgr->current_mode.if_type) {
	case VID_ENC_IF_BT656:
		dispc_reg_merge(VENC_VMOD,
				VENC_VMOD_VDMD_YCBCR8 << VENC_VMOD_VDMD_SHIFT,
				VENC_VMOD_VDMD);
		dispc_reg_merge(VENC_YCCCTL, 1, 1);
		break;
	case VID_ENC_IF_BT1120:
		break;
	case VID_ENC_IF_YCC8:
		dispc_reg_merge(VENC_VMOD,
				VENC_VMOD_VDMD_YCBCR8 << VENC_VMOD_VDMD_SHIFT,
				VENC_VMOD_VDMD);
		break;
	case VID_ENC_IF_YCC16:
		dispc_reg_merge(VENC_VMOD,
				VENC_VMOD_VDMD_YCBCR16 << VENC_VMOD_VDMD_SHIFT,
				VENC_VMOD_VDMD);
		break;
	case VID_ENC_IF_SRGB:
		dispc_reg_merge(VENC_VMOD,
				VENC_VMOD_VDMD_RGB8 << VENC_VMOD_VDMD_SHIFT,
				VENC_VMOD_VDMD);
		break;
	case VID_ENC_IF_PRGB:
		dispc_reg_merge(VENC_VMOD,
				VENC_VMOD_VDMD_RGB666 << VENC_VMOD_VDMD_SHIFT,
				VENC_VMOD_VDMD);
		break;
	default:
		break;
	}

	if (strcmp(mgr->current_mode.name, VID_ENC_STD_NTSC) == 0) {
		davinci_enc_set_ntsc(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_NTSC_RGB) == 0) {
		davinci_enc_set_ntsc_pal_rgb(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_PAL) == 0) {
		davinci_enc_set_pal(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_PAL_RGB) == 0) {
		davinci_enc_set_ntsc_pal_rgb(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_480P_60) == 0) {
		davinci_enc_set_525p(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_576P_50) == 0) {
		davinci_enc_set_625p(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_640x480) == 0 ||
		   strcmp(mgr->current_mode.name, VID_ENC_STD_640x400) == 0 ||
		   strcmp(mgr->current_mode.name, VID_ENC_STD_640x350) == 0) {
		davinci_enc_set_prgb(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_720P_60) == 0) {
		davinci_enc_set_720p(&mgr->current_mode);
	} else if (strcmp(mgr->current_mode.name, VID_ENC_STD_1080I_30) == 0) {
		davinci_enc_set_1080i(&mgr->current_mode);

	}

	/* turn off ping-pong buffer and field inversion to fix
	   the image shaking problem in 1080I mode. The problem i.d. by the
	   DM6446 Advisory 1.3.8 is not seen in 1080I mode, but the ping-pong
	   buffer workaround created a shaking problem. */
	if (cpu_is_davinci_dm644x() &&
	    strcmp(mgr->current_mode.name, VID_ENC_STD_1080I_30) == 0)
		davinci_disp_set_field_inversion(0);

	return;
}

void davinci_enc_set_mode_platform(int channel, struct vid_enc_device_mgr *mgr)
{

	if (0 == mgr->current_mode.std) {
		davinci_enc_set_display_timing(&mgr->current_mode);
		return;
	}
	davinci_enc_priv_setmode(mgr);
}

EXPORT_SYMBOL(davinci_enc_set_mode_platform);

static int davinci_platform_init(void)
{
#ifdef CONFIG_SYSFS
	display_cntl_base = ioremap(DM644X_DDR2_CNTL_BASE, 0x24);
	if (!display_cntl_base) {
		printk(KERN_ERR "Could not remap control registers\n");
		return -EINVAL;
	}
	class_register(&davinci_system_class);

	reg_access_lock = SPIN_LOCK_UNLOCKED;
	davinci_system_device = create_sysfs_files();
	if (!davinci_system_device) {
		printk(KERN_ERR "Could not create davinci system sysfs\n");
		iounmap(display_cntl_base);
		return -EINVAL;
	}
#endif
	return 0;
}

static void davinci_platform_cleanup(void)
{
#ifdef CONFIG_SYSFS
	remove_sysfs_files(davinci_system_device);
	class_unregister(&davinci_system_class);
	iounmap(display_cntl_base);
#endif
	return;
}

subsys_initcall(davinci_platform_init);
module_exit(davinci_platform_cleanup);

MODULE_LICENSE("GPL");
