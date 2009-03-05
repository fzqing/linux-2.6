/*
 * TI DaVinci On-Screen Display Manager
 *
 * Andy Lowe (alowe@mvista.com), MontaVista Software
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <video/davinci_vpbe.h>
#include <video/davinci_osd.h>
#include <asm/arch/io.h>
#include <asm/arch/cpu.h>

/* parameters that apply on a per-window (OSD or video) basis */
struct davinci_window_state {
	int is_allocated;
	int is_enabled;
	unsigned long fb_base_phys;
	enum davinci_zoom_factor h_zoom;
	enum davinci_zoom_factor v_zoom;
	struct davinci_layer_config lconfig;
};

/* parameters that apply on a per-OSD-window basis */
struct davinci_osdwin_state {
	enum davinci_clut clut;
	enum davinci_blending_factor blend;
	int colorkey_blending;
	unsigned colorkey;
	int rec601_attenuation;
	unsigned char palette_map[16];	/* index is pixel value */
};

/* hardware rectangular cursor parameters */
struct davinci_cursor_state {
	int is_enabled;
	struct davinci_cursor_config config;
};

struct davinci_osd_state {
	spinlock_t lock;
	unsigned long osdregs;	/* physical base address of OSD registers */
	unsigned long vencregs;	/* physical base address of VENC registers */
	int irq;
	struct davinci_disp_callback *callback;
	int pingpong;		/* 1-->the isr will toggle the VID0 ping-pong buffer */
	int interpolation_filter;
	int field_inversion;
	enum davinci_h_exp_ratio osd_h_exp;
	enum davinci_v_exp_ratio osd_v_exp;
	enum davinci_h_exp_ratio vid_h_exp;
	enum davinci_v_exp_ratio vid_v_exp;
	enum davinci_clut backg_clut;
	unsigned backg_clut_index;
	enum davinci_rom_clut rom_clut;
	int is_blinking;	/* attribute window blinking enabled */
	enum davinci_blink_interval blink;
	enum davinci_pix_format yc_pixfmt;	/* YCbCrI or YCrCbI */
	unsigned char clut_ram[256][3];	/* columns are Y, Cb, Cr */
	struct davinci_cursor_state cursor;
	struct davinci_window_state win[4];	/* OSD0, VID0, OSD1, VID1 */
	struct davinci_osdwin_state osdwin[2];	/* OSD0, OSD1 */
};

static struct davinci_osd_state osd_state;
static struct davinci_osd_state *osd = &osd_state;

/* register access routines */
static __inline__ u32 osd_read(u32 offset)
{
	return davinci_readl(osd->osdregs + offset);
}

static __inline__ u32 osd_write(u32 val, u32 offset)
{
	davinci_writel(val, osd->osdregs + offset);
	return val;
}

static __inline__ u32 osd_set(u32 mask, u32 offset)
{
	u32 addr = osd->osdregs + offset;
	u32 val = davinci_readl(addr) | mask;

	davinci_writel(val, addr);
	return val;
}

static __inline__ u32 osd_clear(u32 mask, u32 offset)
{
	u32 addr = osd->osdregs + offset;
	u32 val = davinci_readl(addr) & ~mask;

	davinci_writel(val, addr);
	return val;
}

static __inline__ u32 osd_merge(u32 mask, u32 val, u32 offset)
{
	u32 addr = osd->osdregs + offset;
	u32 new_val = (davinci_readl(addr) & ~mask) | (val & mask);

	davinci_writel(new_val, addr);
	return new_val;
}

/* define some macros for layer and pixfmt classification */
#define is_osd_win(layer) (((layer) == WIN_OSD0) || ((layer) == WIN_OSD1))
#define is_vid_win(layer) (((layer) == WIN_VID0) || ((layer) == WIN_VID1))
#define is_rgb_pixfmt(pixfmt) \
	(((pixfmt) == PIXFMT_RGB565) || ((pixfmt) == PIXFMT_RGB888))
#define is_yc_pixfmt(pixfmt) \
	(((pixfmt) == PIXFMT_YCbCrI) || ((pixfmt) == PIXFMT_YCrCbI))

#define MAX_WIN_SIZE OSD_VIDWIN0XP_V0X
#define MAX_LINE_LENGTH (OSD_VIDWIN0OFST_V0LO << 5)

/* interrupt service routine */
static irqreturn_t davinci_disp_isr(int irq, void *arg, struct pt_regs *regs)
{
	unsigned event = 0;
	struct davinci_disp_callback *callback = osd->callback;
	static unsigned last_event;

	if (cpu_is_davinci_dm355()) {
		if (!
		    (davinci_readl(DM355_VPSSBL_REG_BASE + VPSSBL_INTSTAT) &
		     VPSSBL_INTSTAT_VENCINT)) {
			return IRQ_NONE;
		}
		davinci_writel(VPSSBL_INTSTAT_VENCINT,
			       DM355_VPSSBL_REG_BASE + VPSSBL_INTSTAT);
	}

	if ((davinci_readl(osd->vencregs + VENC_VSTAT) & VENC_VSTAT_FIDST) ==
	    VENC_VSTAT_FIDST)
		event |= DAVINCI_DISP_SECOND_FIELD;
	else
		event |= DAVINCI_DISP_FIRST_FIELD;

	if (event == (last_event & ~DAVINCI_DISP_END_OF_FRAME)) {
		/*
		 * If the display is non-interlaced, then we need to flag the
		 * end-of-frame event at every interrupt regardless of the
		 * value of the FIDST bit.  We can conclude that the display is
		 * non-interlaced if the value of the FIDST bit is unchanged
		 * from the previous interrupt.
		 */
		event |= DAVINCI_DISP_END_OF_FRAME;
	} else if (event == DAVINCI_DISP_SECOND_FIELD) {
		/* end-of-frame for interlaced display */
		event |= DAVINCI_DISP_END_OF_FRAME;
	}
	last_event = event;

	if (osd->pingpong) {
		/*
		 * Toggle the ping-pong buffers for VID0.  This is part of the
		 * workaround for field signal inversion Advisory 1.3.8 for
		 * the DM6446.
		 */
		if (event & DAVINCI_DISP_SECOND_FIELD)
			osd_set(OSD_MISCCTL_PPSW, OSD_MISCCTL);
		else
			osd_clear(OSD_MISCCTL_PPSW, OSD_MISCCTL);
	}

	while (callback) {
		if (callback->mask & event)
			callback->handler(event, callback->arg);
		callback = callback->next;
	}

	return IRQ_HANDLED;
}

int davinci_disp_unregister_callback(struct davinci_disp_callback *callback)
{
	unsigned long flags;
	struct davinci_disp_callback *prev;
	int err = 0;

	spin_lock_irqsave(&osd->lock, flags);

	prev = osd->callback;
	if (!prev)
		err = -1;
	else if (prev == callback)
		osd->callback = callback->next;
	else {
		while (prev->next && (prev->next != callback))
			prev = prev->next;
		if (!prev->next)
			err = -1;
		else
			prev->next = callback->next;
	}

	spin_unlock_irqrestore(&osd->lock, flags);

	return err;
}

int davinci_disp_register_callback(struct davinci_disp_callback *callback)
{
	unsigned long flags;
	struct davinci_disp_callback *next;

	spin_lock_irqsave(&osd->lock, flags);

	next = osd->callback;
	osd->callback = callback;
	callback->next = next;

	spin_unlock_irqrestore(&osd->lock, flags);

	return 0;
}

/*
 * This routine implements a workaround for the field signal inversion silicon
 * erratum described in Advisory 1.3.8 for the DM6446.  The fb_base_phys and
 * lconfig parameters apply to the vid0 window.  This routine should be called
 * whenever the vid0 layer configuration or start address is modified, or when
 * the OSD field inversion setting is modified.
 * Returns: 1 if the ping-pong buffers need to be toggled in the vsync isr, or
 *          0 otherwise
 */
static int _davinci_disp_dm6446_vid0_pingpong(int field_inversion,
					      unsigned long fb_base_phys,
					      const struct davinci_layer_config
					      *lconfig)
{
	if (!cpu_is_davinci_dm644x())
		return 0;

	if (!field_inversion || !lconfig->interlaced) {
		osd_write(fb_base_phys & ~0x1F, OSD_VIDWIN0ADR);
		osd_write(fb_base_phys & ~0x1F, OSD_PPVWIN0ADR);
		osd_merge(OSD_MISCCTL_PPSW | OSD_MISCCTL_PPRV, 0, OSD_MISCCTL);

		return 0;
	} else {
		unsigned miscctl = OSD_MISCCTL_PPRV;

		osd_write((fb_base_phys & ~0x1F) - lconfig->line_length,
			  OSD_VIDWIN0ADR);
		osd_write((fb_base_phys & ~0x1F) + lconfig->line_length,
			  OSD_PPVWIN0ADR);
		if ((davinci_readl(osd->vencregs + VENC_VSTAT) &
		     VENC_VSTAT_FIDST) == VENC_VSTAT_FIDST) {
			miscctl |= OSD_MISCCTL_PPSW;
		}
		osd_merge(OSD_MISCCTL_PPSW | OSD_MISCCTL_PPRV, miscctl,
			  OSD_MISCCTL);

		return 1;
	}
}

int davinci_disp_get_field_inversion(void)
{
	return osd->field_inversion;
}

static void _davinci_disp_set_field_inversion(int enable)
{
	unsigned fsinv = 0;

	if (enable)
		fsinv = OSD_MODE_FSINV;

	osd_merge(OSD_MODE_FSINV, fsinv, OSD_MODE);
}

void davinci_disp_set_field_inversion(int enable)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->field_inversion = (enable != 0);
	_davinci_disp_set_field_inversion(enable);

	osd->pingpong =
	    _davinci_disp_dm6446_vid0_pingpong(osd->field_inversion,
					       osd->win[WIN_VID0].fb_base_phys,
					       &osd->win[WIN_VID0].lconfig);

	spin_unlock_irqrestore(&osd->lock, flags);
}

void davinci_disp_get_background(enum davinci_clut *clut,
				 unsigned char *clut_index)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*clut = osd->backg_clut;
	*clut_index = osd->backg_clut_index;

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_background(enum davinci_clut clut,
					 unsigned char clut_index)
{
	u32 mode = 0;

	if (clut == RAM_CLUT)
		mode |= OSD_MODE_BCLUT;
	mode |= clut_index;
	osd_merge(OSD_MODE_BCLUT | OSD_MODE_CABG, mode, OSD_MODE);
}

void davinci_disp_set_background(enum davinci_clut clut,
				 unsigned char clut_index)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->backg_clut = clut;
	osd->backg_clut_index = clut_index;
	_davinci_disp_set_background(clut, clut_index);

	spin_unlock_irqrestore(&osd->lock, flags);
}

int davinci_disp_get_interpolation_filter(void)
{
	return osd->interpolation_filter;
}

static void _davinci_disp_set_interpolation_filter(int filter)
{
	if (cpu_is_davinci_dm355())
		osd_clear(OSD_EXTMODE_EXPMDSEL, OSD_EXTMODE);
	osd_merge(OSD_MODE_EF, filter ? OSD_MODE_EF : 0, OSD_MODE);
}

void davinci_disp_set_interpolation_filter(int filter)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->interpolation_filter = (filter != 0);
	_davinci_disp_set_interpolation_filter(filter);

	spin_unlock_irqrestore(&osd->lock, flags);
}

void davinci_disp_get_cursor_config(struct davinci_cursor_config *cursor)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*cursor = osd->cursor.config;

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_cursor_config(const struct davinci_cursor_config
					    *cursor)
{
	unsigned rectcur = 0;

	osd_write(cursor->xsize, OSD_CURXL);
	osd_write(cursor->xpos, OSD_CURXP);

	if (cursor->interlaced) {
		osd_write(cursor->ypos >> 1, OSD_CURYP);
		if (cpu_is_davinci_dm644x()) {
			/* Must add 1 to ysize due to device erratum. */
			osd_write((cursor->ysize >> 1) + 1, OSD_CURYL);
		} else
			osd_write(cursor->ysize >> 1, OSD_CURYL);
	} else {
		osd_write(cursor->ypos, OSD_CURYP);
		if (cpu_is_davinci_dm644x()) {
			/* Must add 1 to ysize due to device erratum. */
			osd_write(cursor->ysize + 1, OSD_CURYL);
		} else
			osd_write(cursor->ysize, OSD_CURYL);
	}

	if (cursor->clut == RAM_CLUT)
		rectcur |= OSD_RECTCUR_CLUTSR;
	rectcur |= (cursor->clut_index << OSD_RECTCUR_RCAD_SHIFT);
	rectcur |= (cursor->h_width << OSD_RECTCUR_RCHW_SHIFT);
	rectcur |= (cursor->v_width << OSD_RECTCUR_RCVW_SHIFT);
	osd_merge(OSD_RECTCUR_RCAD | OSD_RECTCUR_CLUTSR | OSD_RECTCUR_RCHW |
		  OSD_RECTCUR_RCVW, rectcur, OSD_RECTCUR);
}

void davinci_disp_set_cursor_config(struct davinci_cursor_config *cursor)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	cursor->xsize = min(cursor->xsize, (unsigned)OSD_CURXL_RCSW);
	cursor->ysize = min(cursor->ysize, (unsigned)OSD_CURYL_RCSH);
	cursor->xpos = min(cursor->xpos, (unsigned)OSD_CURXP_RCSX);
	cursor->ypos = min(cursor->ypos, (unsigned)OSD_CURYP_RCSY);
	cursor->interlaced = (cursor->interlaced != 0);
	if (cursor->interlaced) {
		cursor->ysize &= ~1;
		cursor->ypos &= ~1;
	}
	cursor->h_width &= (OSD_RECTCUR_RCHW >> OSD_RECTCUR_RCHW_SHIFT);
	cursor->v_width &= (OSD_RECTCUR_RCVW >> OSD_RECTCUR_RCVW_SHIFT);
	cursor->clut = (cursor->clut == RAM_CLUT) ? RAM_CLUT : ROM_CLUT;

	osd->cursor.config = *cursor;
	_davinci_disp_set_cursor_config(cursor);

	spin_unlock_irqrestore(&osd->lock, flags);
}

int davinci_disp_cursor_is_enabled(void)
{
	return osd->cursor.is_enabled;
}

static void _davinci_disp_cursor_disable(void)
{
	osd_clear(OSD_RECTCUR_RCACT, OSD_RECTCUR);
}

void davinci_disp_cursor_disable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->cursor.is_enabled = 0;
	_davinci_disp_cursor_disable();

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_cursor_enable(void)
{
	osd_set(OSD_RECTCUR_RCACT, OSD_RECTCUR);
}

void davinci_disp_cursor_enable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->cursor.is_enabled = 1;
	_davinci_disp_cursor_enable();

	spin_unlock_irqrestore(&osd->lock, flags);
}

void davinci_disp_get_vid_expansion(enum davinci_h_exp_ratio *h_exp,
				    enum davinci_v_exp_ratio *v_exp)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*h_exp = osd->vid_h_exp;
	*v_exp = osd->vid_v_exp;

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_vid_expansion(enum davinci_h_exp_ratio h_exp,
					    enum davinci_v_exp_ratio v_exp)
{
	u32 mode = 0, extmode = 0;

	switch (h_exp) {
	case H_EXP_OFF:
		break;
	case H_EXP_9_OVER_8:
		mode |= OSD_MODE_VHRSZ;
		break;
	case H_EXP_3_OVER_2:
		extmode |= OSD_EXTMODE_VIDHRSZ15;
		break;
	}

	switch (v_exp) {
	case V_EXP_OFF:
		break;
	case V_EXP_6_OVER_5:
		mode |= OSD_MODE_VVRSZ;
		break;
	}

	if (cpu_is_davinci_dm355())
		osd_merge(OSD_EXTMODE_VIDHRSZ15, extmode, OSD_EXTMODE);
	osd_merge(OSD_MODE_VHRSZ | OSD_MODE_VVRSZ, mode, OSD_MODE);
}

int davinci_disp_set_vid_expansion(enum davinci_h_exp_ratio h_exp,
				   enum davinci_v_exp_ratio v_exp)
{
	unsigned long flags;

	if (h_exp == H_EXP_3_OVER_2 && cpu_is_davinci_dm644x())
		return -1;

	spin_lock_irqsave(&osd->lock, flags);

	osd->vid_h_exp = h_exp;
	osd->vid_v_exp = v_exp;
	_davinci_disp_set_vid_expansion(h_exp, v_exp);

	spin_unlock_irqrestore(&osd->lock, flags);
	return 0;
}

void davinci_disp_get_osd_expansion(enum davinci_h_exp_ratio *h_exp,
				    enum davinci_v_exp_ratio *v_exp)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*h_exp = osd->osd_h_exp;
	*v_exp = osd->osd_v_exp;

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_osd_expansion(enum davinci_h_exp_ratio h_exp,
					    enum davinci_v_exp_ratio v_exp)
{
	u32 mode = 0, extmode = 0;

	switch (h_exp) {
	case H_EXP_OFF:
		break;
	case H_EXP_9_OVER_8:
		mode |= OSD_MODE_OHRSZ;
		break;
	case H_EXP_3_OVER_2:
		extmode |= OSD_EXTMODE_OSDHRSZ15;
		break;
	}

	switch (v_exp) {
	case V_EXP_OFF:
		break;
	case V_EXP_6_OVER_5:
		mode |= OSD_MODE_OVRSZ;
		break;
	}

	if (cpu_is_davinci_dm355())
		osd_merge(OSD_EXTMODE_OSDHRSZ15, extmode, OSD_EXTMODE);
	osd_merge(OSD_MODE_OHRSZ | OSD_MODE_OVRSZ, mode, OSD_MODE);
}

int davinci_disp_set_osd_expansion(enum davinci_h_exp_ratio h_exp,
				   enum davinci_v_exp_ratio v_exp)
{
	unsigned long flags;

	if (h_exp == H_EXP_3_OVER_2 && cpu_is_davinci_dm644x())
		return -1;

	spin_lock_irqsave(&osd->lock, flags);

	osd->osd_h_exp = h_exp;
	osd->osd_v_exp = v_exp;
	_davinci_disp_set_osd_expansion(h_exp, v_exp);

	spin_unlock_irqrestore(&osd->lock, flags);
	return 0;
}

void davinci_disp_get_blink_attribute(int *enable,
				      enum davinci_blink_interval *blink)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*enable = osd->is_blinking;
	*blink = osd->blink;

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_blink_attribute(int enable,
					      enum davinci_blink_interval blink)
{
	u32 osdatrmd = 0;

	if (enable) {
		osdatrmd |= OSD_OSDATRMD_BLNK;
		osdatrmd |= blink << OSD_OSDATRMD_BLNKINT_SHIFT;
	}
	/* caller must ensure that OSD1 is configured in attribute mode */
	osd_merge(OSD_OSDATRMD_BLNKINT | OSD_OSDATRMD_BLNK, osdatrmd,
		  OSD_OSDATRMD);
}

void davinci_disp_set_blink_attribute(int enable,
				      enum davinci_blink_interval blink)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->is_blinking = (enable != 0);
	osd->blink = blink;
	if (osd->win[WIN_OSD1].lconfig.pixfmt == PIXFMT_OSD_ATTR)
		_davinci_disp_set_blink_attribute(enable, blink);

	spin_unlock_irqrestore(&osd->lock, flags);
}

enum davinci_rom_clut davinci_disp_get_rom_clut(void)
{
	return osd->rom_clut;
}

static void _davinci_disp_set_rom_clut(enum davinci_rom_clut rom_clut)
{
	if (rom_clut == ROM_CLUT0)
		osd_clear(OSD_MISCCTL_RSEL, OSD_MISCCTL);
	else
		osd_set(OSD_MISCCTL_RSEL, OSD_MISCCTL);
}

void davinci_disp_set_rom_clut(enum davinci_rom_clut rom_clut)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->rom_clut = rom_clut;
	_davinci_disp_set_rom_clut(rom_clut);

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_clut_ycbcr(unsigned char clut_index,
					 unsigned char y, unsigned char cb,
					 unsigned char cr)
{
	/* wait until any previous writes to the CLUT RAM have completed */
	while (osd_read(OSD_MISCCTL) & OSD_MISCCTL_CPBSY)
		cpu_relax();

	osd_write((y << OSD_CLUTRAMYCB_Y_SHIFT) | cb, OSD_CLUTRAMYCB);
	osd_write((cr << OSD_CLUTRAMCR_CR_SHIFT) | clut_index, OSD_CLUTRAMCR);
}

void davinci_disp_set_clut_ycbcr(unsigned char clut_index, unsigned char y,
				 unsigned char cb, unsigned char cr)
{
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osd->clut_ram[clut_index][0] = y;
	osd->clut_ram[clut_index][1] = cb;
	osd->clut_ram[clut_index][2] = cr;
	_davinci_disp_set_clut_ycbcr(clut_index, y, cb, cr);

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_rgb_to_ycbcr(const unsigned char rgb[3],
				       unsigned char ycbcr[3])
{
	int y, cb, cr;
	int r = rgb[0];
	int g = rgb[1];
	int b = rgb[2];
	/*
	 * This conversion matrix corresponds to the conversion matrix used
	 * by the OSD to convert RGB values to YCbCr values.  All coefficients
	 * have been scaled by a factor of 2^22.
	 */
	static const int rgb_to_ycbcr[3][3] = {
		{1250330, 2453618, 490352},
		{-726093, -1424868, 2150957},
		{2099836, -1750086, -349759}
	};

	y = rgb_to_ycbcr[0][0] * r + rgb_to_ycbcr[0][1] * g +
	    rgb_to_ycbcr[0][2] * b;
	cb = rgb_to_ycbcr[1][0] * r + rgb_to_ycbcr[1][1] * g +
	    rgb_to_ycbcr[1][2] * b;
	cr = rgb_to_ycbcr[2][0] * r + rgb_to_ycbcr[2][1] * g +
	    rgb_to_ycbcr[2][2] * b;

	/* round and scale */
	y = ((y + (1 << 21)) >> 22);
	cb = ((cb + (1 << 21)) >> 22) + 128;
	cr = ((cr + (1 << 21)) >> 22) + 128;

	/* clip */
	y = (y < 0) ? 0 : y;
	y = (y > 255) ? 255 : y;
	cb = (cb < 0) ? 0 : cb;
	cb = (cb > 255) ? 255 : cb;
	cr = (cr < 0) ? 0 : cr;
	cr = (cr > 255) ? 255 : cr;

	ycbcr[0] = y;
	ycbcr[1] = cb;
	ycbcr[2] = cr;
}

void davinci_disp_set_clut_rgb(unsigned char clut_index, unsigned char r,
			       unsigned char g, unsigned char b)
{
	unsigned char rgb[3], ycbcr[3];
	unsigned long flags;

	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
	_davinci_disp_rgb_to_ycbcr(rgb, ycbcr);

	spin_lock_irqsave(&osd->lock, flags);

	osd->clut_ram[clut_index][0] = ycbcr[0];
	osd->clut_ram[clut_index][1] = ycbcr[1];
	osd->clut_ram[clut_index][2] = ycbcr[2];
	_davinci_disp_set_clut_ycbcr(clut_index, ycbcr[0], ycbcr[1], ycbcr[2]);

	spin_unlock_irqrestore(&osd->lock, flags);
}

unsigned char davinci_disp_get_palette_map(enum davinci_osd_layer osdwin,
					   unsigned char pixel_value)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned char clut_index;
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	switch (win->lconfig.pixfmt) {
	case PIXFMT_1BPP:
		clut_index = osdwin_state->palette_map[pixel_value & 0x1];
		break;
	case PIXFMT_2BPP:
		clut_index = osdwin_state->palette_map[pixel_value & 0x3];
		break;
	case PIXFMT_4BPP:
		clut_index = osdwin_state->palette_map[pixel_value & 0xf];
		break;
	default:
		clut_index = 0;
		break;
	}

	spin_unlock_irqrestore(&osd->lock, flags);

	return clut_index;
}

static void _davinci_disp_set_palette_map(enum davinci_osd_layer osdwin,
					  unsigned char pixel_value,
					  unsigned char clut_index,
					  enum davinci_pix_format pixfmt)
{
	int bmp_reg, bmp_offset, bmp_mask, bmp_shift;
	static const int map_1bpp[] = { 0, 15 };
	static const int map_2bpp[] = { 0, 5, 10, 15 };

	switch (pixfmt) {
	case PIXFMT_1BPP:
		bmp_reg = map_1bpp[pixel_value & 0x1];
		break;
	case PIXFMT_2BPP:
		bmp_reg = map_2bpp[pixel_value & 0x3];
		break;
	case PIXFMT_4BPP:
		bmp_reg = pixel_value & 0xf;
		break;
	default:
		return;
	}

	switch (osdwin) {
	case OSDWIN_OSD0:
		bmp_offset = OSD_W0BMP01 + (bmp_reg >> 1) * sizeof(u32);
		break;
	case OSDWIN_OSD1:
		bmp_offset = OSD_W1BMP01 + (bmp_reg >> 1) * sizeof(u32);
		break;
	default:
		return;
	}

	if (bmp_reg & 1) {
		bmp_shift = 8;
		bmp_mask = 0xff << 8;
	} else {
		bmp_shift = 0;
		bmp_mask = 0xff;
	}

	osd_merge(bmp_mask, clut_index << bmp_shift, bmp_offset);
}

void davinci_disp_set_palette_map(enum davinci_osd_layer osdwin,
				  unsigned char pixel_value,
				  unsigned char clut_index)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	switch (win->lconfig.pixfmt) {
	case PIXFMT_1BPP:
		osdwin_state->palette_map[pixel_value & 0x1] = clut_index;
		break;
	case PIXFMT_2BPP:
		osdwin_state->palette_map[pixel_value & 0x3] = clut_index;
		break;
	case PIXFMT_4BPP:
		osdwin_state->palette_map[pixel_value & 0xf] = clut_index;
		break;
	default:
		spin_unlock_irqrestore(&osd->lock, flags);
		return;
	}

	_davinci_disp_set_palette_map(osdwin, pixel_value, clut_index,
				      win->lconfig.pixfmt);

	spin_unlock_irqrestore(&osd->lock, flags);
}

int davinci_disp_get_rec601_attenuation(enum davinci_osd_layer osdwin)
{
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];

	return osdwin_state->rec601_attenuation;
}

static void _davinci_disp_set_rec601_attenuation(enum davinci_osd_layer osdwin,
						 int enable)
{
	switch (osdwin) {
	case OSDWIN_OSD0:
		if (cpu_is_davinci_dm644x()) {
			osd_merge(OSD_OSDWIN0MD_ATN0E,
				  enable ? OSD_OSDWIN0MD_ATN0E : 0,
				  OSD_OSDWIN0MD);
		} else if (cpu_is_davinci_dm355()) {
			osd_merge(OSD_EXTMODE_ATNOSD0EN,
				  enable ? OSD_EXTMODE_ATNOSD0EN : 0,
				  OSD_EXTMODE);
		}
		break;
	case OSDWIN_OSD1:
		if (cpu_is_davinci_dm644x()) {
			osd_merge(OSD_OSDWIN1MD_ATN1E,
				  enable ? OSD_OSDWIN1MD_ATN1E : 0,
				  OSD_OSDWIN1MD);
		} else if (cpu_is_davinci_dm355()) {
			osd_merge(OSD_EXTMODE_ATNOSD1EN,
				  enable ? OSD_EXTMODE_ATNOSD1EN : 0,
				  OSD_EXTMODE);
		}
		break;
	}
}

void davinci_disp_set_rec601_attenuation(enum davinci_osd_layer osdwin,
					 int enable)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osdwin_state->rec601_attenuation = (enable != 0);
	if (win->lconfig.pixfmt != PIXFMT_OSD_ATTR)
		_davinci_disp_set_rec601_attenuation(osdwin, enable);

	spin_unlock_irqrestore(&osd->lock, flags);
}

enum davinci_blending_factor davinci_disp_get_blending_factor(enum
							      davinci_osd_layer
							      osdwin)
{
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];

	return osdwin_state->blend;
}

static void _davinci_disp_set_blending_factor(enum davinci_osd_layer osdwin,
					      enum davinci_blending_factor
					      blend)
{
	switch (osdwin) {
	case OSDWIN_OSD0:
		osd_merge(OSD_OSDWIN0MD_BLND0,
			  blend << OSD_OSDWIN0MD_BLND0_SHIFT, OSD_OSDWIN0MD);
		break;
	case OSDWIN_OSD1:
		osd_merge(OSD_OSDWIN1MD_BLND1,
			  blend << OSD_OSDWIN1MD_BLND1_SHIFT, OSD_OSDWIN1MD);
		break;
	}
}

void davinci_disp_set_blending_factor(enum davinci_osd_layer osdwin,
				      enum davinci_blending_factor blend)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osdwin_state->blend = blend;
	if (win->lconfig.pixfmt != PIXFMT_OSD_ATTR)
		_davinci_disp_set_blending_factor(osdwin, blend);

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_disable_color_key(enum davinci_osd_layer osdwin)
{
	switch (osdwin) {
	case OSDWIN_OSD0:
		osd_clear(OSD_OSDWIN0MD_TE0, OSD_OSDWIN0MD);
		break;
	case OSDWIN_OSD1:
		osd_clear(OSD_OSDWIN1MD_TE1, OSD_OSDWIN1MD);
		break;
	}
}

void davinci_disp_disable_color_key(enum davinci_osd_layer osdwin)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osdwin_state->colorkey_blending = 0;
	if (win->lconfig.pixfmt != PIXFMT_OSD_ATTR)
		_davinci_disp_disable_color_key(osdwin);

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_enable_color_key(enum davinci_osd_layer osdwin,
					   unsigned colorkey,
					   enum davinci_pix_format pixfmt)
{
	switch (pixfmt) {
	case PIXFMT_1BPP:
	case PIXFMT_2BPP:
	case PIXFMT_4BPP:
	case PIXFMT_8BPP:
		if (cpu_is_davinci_dm355()) {
			switch (osdwin) {
			case OSDWIN_OSD0:
				osd_merge(OSD_TRANSPBMPIDX_BMP0,
					  colorkey <<
					  OSD_TRANSPBMPIDX_BMP0_SHIFT,
					  OSD_TRANSPBMPIDX);
				break;
			case OSDWIN_OSD1:
				osd_merge(OSD_TRANSPBMPIDX_BMP1,
					  colorkey <<
					  OSD_TRANSPBMPIDX_BMP1_SHIFT,
					  OSD_TRANSPBMPIDX);
				break;
			}
		}
		break;
	case PIXFMT_RGB565:
		if (cpu_is_davinci_dm644x()) {
			osd_write(colorkey & OSD_TRANSPVAL_RGBTRANS,
				  OSD_TRANSPVAL);
		} else if (cpu_is_davinci_dm355()) {
			osd_write(colorkey & OSD_TRANSPVALL_RGBL,
				  OSD_TRANSPVALL);
		}
		break;
	case PIXFMT_YCbCrI:
	case PIXFMT_YCrCbI:
		if (cpu_is_davinci_dm355())
			osd_merge(OSD_TRANSPVALU_Y, colorkey, OSD_TRANSPVALU);
		break;
	case PIXFMT_RGB888:
		if (cpu_is_davinci_dm355()) {
			osd_write(colorkey & OSD_TRANSPVALL_RGBL,
				  OSD_TRANSPVALL);
			osd_merge(OSD_TRANSPVALU_RGBU, colorkey >> 16,
				  OSD_TRANSPVALU);
		}
		break;
	default:
		break;
	}

	switch (osdwin) {
	case OSDWIN_OSD0:
		osd_set(OSD_OSDWIN0MD_TE0, OSD_OSDWIN0MD);
		break;
	case OSDWIN_OSD1:
		osd_set(OSD_OSDWIN1MD_TE1, OSD_OSDWIN1MD);
		break;
	}
}

void davinci_disp_enable_color_key(enum davinci_osd_layer osdwin,
				   unsigned colorkey)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osdwin_state->colorkey_blending = 1;
	osdwin_state->colorkey = colorkey;
	if (win->lconfig.pixfmt != PIXFMT_OSD_ATTR) {
		_davinci_disp_enable_color_key(osdwin, colorkey,
					       win->lconfig.pixfmt);
	}

	spin_unlock_irqrestore(&osd->lock, flags);
}

enum davinci_clut davinci_disp_get_osd_clut(enum davinci_osd_layer osdwin)
{
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];

	return osdwin_state->clut;
}

static void _davinci_disp_set_osd_clut(enum davinci_osd_layer osdwin,
				       enum davinci_clut clut)
{
	u32 winmd = 0;

	switch (osdwin) {
	case OSDWIN_OSD0:
		if (clut == RAM_CLUT)
			winmd |= OSD_OSDWIN0MD_CLUTS0;
		osd_merge(OSD_OSDWIN0MD_CLUTS0, winmd, OSD_OSDWIN0MD);
		break;
	case OSDWIN_OSD1:
		if (clut == RAM_CLUT)
			winmd |= OSD_OSDWIN1MD_CLUTS1;
		osd_merge(OSD_OSDWIN1MD_CLUTS1, winmd, OSD_OSDWIN1MD);
		break;
	}
}

void davinci_disp_set_osd_clut(enum davinci_osd_layer osdwin,
			       enum davinci_clut clut)
{
	enum davinci_disp_layer layer =
	    (osdwin == OSDWIN_OSD0) ? WIN_OSD0 : WIN_OSD1;
	struct davinci_window_state *win = &osd->win[layer];
	struct davinci_osdwin_state *osdwin_state = &osd->osdwin[osdwin];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	osdwin_state->clut = clut;
	if (win->lconfig.pixfmt != PIXFMT_OSD_ATTR)
		_davinci_disp_set_osd_clut(osdwin, clut);

	spin_unlock_irqrestore(&osd->lock, flags);
}

void davinci_disp_get_zoom(enum davinci_disp_layer layer,
			   enum davinci_zoom_factor *h_zoom,
			   enum davinci_zoom_factor *v_zoom)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*h_zoom = win->h_zoom;
	*v_zoom = win->v_zoom;

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_set_zoom(enum davinci_disp_layer layer,
				   enum davinci_zoom_factor h_zoom,
				   enum davinci_zoom_factor v_zoom)
{
	u32 winmd = 0;

	switch (layer) {
	case WIN_OSD0:
		winmd |= (h_zoom << OSD_OSDWIN0MD_OHZ0_SHIFT);
		winmd |= (v_zoom << OSD_OSDWIN0MD_OVZ0_SHIFT);
		osd_merge(OSD_OSDWIN0MD_OHZ0 | OSD_OSDWIN0MD_OVZ0, winmd,
			  OSD_OSDWIN0MD);
		break;
	case WIN_VID0:
		winmd |= (h_zoom << OSD_VIDWINMD_VHZ0_SHIFT);
		winmd |= (v_zoom << OSD_VIDWINMD_VVZ0_SHIFT);
		osd_merge(OSD_VIDWINMD_VHZ0 | OSD_VIDWINMD_VVZ0, winmd,
			  OSD_VIDWINMD);
		break;
	case WIN_OSD1:
		winmd |= (h_zoom << OSD_OSDWIN1MD_OHZ1_SHIFT);
		winmd |= (v_zoom << OSD_OSDWIN1MD_OVZ1_SHIFT);
		osd_merge(OSD_OSDWIN1MD_OHZ1 | OSD_OSDWIN1MD_OVZ1, winmd,
			  OSD_OSDWIN1MD);
		break;
	case WIN_VID1:
		winmd |= (h_zoom << OSD_VIDWINMD_VHZ1_SHIFT);
		winmd |= (v_zoom << OSD_VIDWINMD_VVZ1_SHIFT);
		osd_merge(OSD_VIDWINMD_VHZ1 | OSD_VIDWINMD_VVZ1, winmd,
			  OSD_VIDWINMD);
		break;
	}
}

void davinci_disp_set_zoom(enum davinci_disp_layer layer,
			   enum davinci_zoom_factor h_zoom,
			   enum davinci_zoom_factor v_zoom)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	win->h_zoom = h_zoom;
	win->v_zoom = v_zoom;
	_davinci_disp_set_zoom(layer, h_zoom, v_zoom);

	spin_unlock_irqrestore(&osd->lock, flags);
}

int davinci_disp_layer_is_enabled(enum davinci_disp_layer layer)
{
	struct davinci_window_state *win = &osd->win[layer];
	return win->is_enabled;
}

static void _davinci_disp_disable_layer(enum
					davinci_disp_layer layer)
{
	switch (layer) {
	case WIN_OSD0:
		osd_clear(OSD_OSDWIN0MD_OACT0, OSD_OSDWIN0MD);
		break;
	case WIN_VID0:
		osd_clear(OSD_VIDWINMD_ACT0, OSD_VIDWINMD);
		break;
	case WIN_OSD1:
		/* disable attribute mode as well as disabling the window */
		osd_clear(OSD_OSDWIN1MD_OASW | OSD_OSDWIN1MD_OACT1,
			  OSD_OSDWIN1MD);
		break;
	case WIN_VID1:
		osd_clear(OSD_VIDWINMD_ACT1, OSD_VIDWINMD);
		break;
	}
}

void davinci_disp_disable_layer(enum davinci_disp_layer layer)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	if (!win->is_enabled) {
		spin_unlock_irqrestore(&osd->lock, flags);
		return;
	}
	win->is_enabled = 0;

	_davinci_disp_disable_layer(layer);

	spin_unlock_irqrestore(&osd->lock, flags);
}

static void _davinci_disp_enable_attribute_mode(void)
{
	/* enable attribute mode for OSD1 */
	osd_set(OSD_OSDWIN1MD_OASW, OSD_OSDWIN1MD);
}

static void _davinci_disp_enable_layer(enum
				       davinci_disp_layer layer)
{
	switch (layer) {
	case WIN_OSD0:
		osd_set(OSD_OSDWIN0MD_OACT0, OSD_OSDWIN0MD);
		break;
	case WIN_VID0:
		osd_set(OSD_VIDWINMD_ACT0, OSD_VIDWINMD);
		break;
	case WIN_OSD1:
		/* enable OSD1 and disable attribute mode */
		osd_merge(OSD_OSDWIN1MD_OASW | OSD_OSDWIN1MD_OACT1,
			  OSD_OSDWIN1MD_OACT1, OSD_OSDWIN1MD);
		break;
	case WIN_VID1:
		osd_set(OSD_VIDWINMD_ACT1, OSD_VIDWINMD);
		break;
	}
}

int davinci_disp_enable_layer(enum davinci_disp_layer layer)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	if (!win->is_allocated || !win->fb_base_phys
	    || !win->lconfig.line_length || !win->lconfig.xsize
	    || !win->lconfig.ysize) {
		spin_unlock_irqrestore(&osd->lock, flags);
		return -1;
	}

	if (win->is_enabled) {
		spin_unlock_irqrestore(&osd->lock, flags);
		return 0;
	}
	win->is_enabled = 1;

	if (win->lconfig.pixfmt != PIXFMT_OSD_ATTR)
		_davinci_disp_enable_layer(layer);
	else {
		_davinci_disp_enable_attribute_mode();
		_davinci_disp_set_blink_attribute(osd->is_blinking, osd->blink);
	}

	spin_unlock_irqrestore(&osd->lock, flags);
	return 0;
}

static void _davinci_disp_start_layer(enum davinci_disp_layer layer,
				      unsigned long fb_base_phys)
{
	if (cpu_is_davinci_dm644x()) {
		switch (layer) {
		case WIN_OSD0:
			osd_write(fb_base_phys & ~0x1F, OSD_OSDWIN0ADR);
			break;
		case WIN_VID0:
			osd_write(fb_base_phys & ~0x1F, OSD_VIDWIN0ADR);
			break;
		case WIN_OSD1:
			osd_write(fb_base_phys & ~0x1F, OSD_OSDWIN1ADR);
			break;
		case WIN_VID1:
			osd_write(fb_base_phys & ~0x1F, OSD_VIDWIN1ADR);
			break;
		}
	} else if (cpu_is_davinci_dm355()) {
		unsigned long fb_offset_32 =
		    (fb_base_phys - DAVINCI_DDR_BASE) >> 5;

		switch (layer) {
		case WIN_OSD0:
			osd_merge(OSD_OSDWINADH_O0AH,
				  fb_offset_32 >> (16 -
						   OSD_OSDWINADH_O0AH_SHIFT),
				  OSD_OSDWINADH);
			osd_write(fb_offset_32 & OSD_OSDWIN0ADL_O0AL,
				  OSD_OSDWIN0ADL);
			break;
		case WIN_VID0:
			osd_merge(OSD_VIDWINADH_V0AH,
				  fb_offset_32 >> (16 -
						   OSD_VIDWINADH_V0AH_SHIFT),
				  OSD_VIDWINADH);
			osd_write(fb_offset_32 & OSD_VIDWIN0ADL_V0AL,
				  OSD_VIDWIN0ADL);
			break;
		case WIN_OSD1:
			osd_merge(OSD_OSDWINADH_O1AH,
				  fb_offset_32 >> (16 -
						   OSD_OSDWINADH_O1AH_SHIFT),
				  OSD_OSDWINADH);
			osd_write(fb_offset_32 & OSD_OSDWIN1ADL_O1AL,
				  OSD_OSDWIN1ADL);
			break;
		case WIN_VID1:
			osd_merge(OSD_VIDWINADH_V1AH,
				  fb_offset_32 >> (16 -
						   OSD_VIDWINADH_V1AH_SHIFT),
				  OSD_VIDWINADH);
			osd_write(fb_offset_32 & OSD_VIDWIN1ADL_V1AL,
				  OSD_VIDWIN1ADL);
			break;
		}
	}
}

void davinci_disp_start_layer(enum davinci_disp_layer layer,
			      unsigned long fb_base_phys)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	win->fb_base_phys = fb_base_phys & ~0x1F;
	_davinci_disp_start_layer(layer, fb_base_phys);

	if (layer == WIN_VID0) {
		osd->pingpong =
		    _davinci_disp_dm6446_vid0_pingpong(osd->field_inversion,
						       win->fb_base_phys,
						       &win->lconfig);
	}

	spin_unlock_irqrestore(&osd->lock, flags);
}

void davinci_disp_get_layer_config(enum davinci_disp_layer layer,
				   struct davinci_layer_config *lconfig)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	*lconfig = win->lconfig;

	spin_unlock_irqrestore(&osd->lock, flags);
}

/*
 * If the requested lconfig is completely rejected and the value of lconfig on
 * exit is the current lconfig, then try_layer_config() returns 1.  Otherwise,
 * try_layer_config() returns 0.  A return value of 0 does not necessarily mean
 * that the value of lconfig on exit is identical to the value of lconfig on
 * entry, but merely that it represents a change from the current lconfig.
 */
static int try_layer_config(enum davinci_disp_layer layer,
			    struct davinci_layer_config *lconfig)
{
	struct davinci_window_state *win = &osd->win[layer];
	int bad_config = 0;

	/* verify that the pixel format is compatible with the layer */
	switch (lconfig->pixfmt) {
	case PIXFMT_1BPP:
	case PIXFMT_2BPP:
	case PIXFMT_4BPP:
	case PIXFMT_8BPP:
	case PIXFMT_RGB565:
		bad_config = !is_osd_win(layer);
		break;
	case PIXFMT_YCbCrI:
	case PIXFMT_YCrCbI:
		if (cpu_is_davinci_dm644x())
			bad_config = !is_vid_win(layer);
		break;
	case PIXFMT_RGB888:
		if (cpu_is_davinci_dm644x())
			bad_config = !is_vid_win(layer);
		else if (cpu_is_davinci_dm355())
			bad_config = !is_osd_win(layer);
		break;
	case PIXFMT_OSD_ATTR:
		bad_config = (layer != WIN_OSD1);
		break;
	default:
		bad_config = 1;
		break;
	}
	if (bad_config) {
		/*
		 * The requested pixel format is incompatible with the layer, so
		 * keep the current layer configuration.
		 */
		*lconfig = win->lconfig;
		return bad_config;
	}

	/* only one OSD window at a time can use RGB or YC pixel formats */
	if (is_osd_win(layer)
	    && (is_rgb_pixfmt(lconfig->pixfmt)
		|| is_yc_pixfmt(lconfig->pixfmt))) {
		enum davinci_pix_format pixfmt;

		if (layer == WIN_OSD0)
			pixfmt = osd->win[WIN_OSD1].lconfig.pixfmt;
		else
			pixfmt = osd->win[WIN_OSD0].lconfig.pixfmt;

		if (is_rgb_pixfmt(pixfmt) || is_yc_pixfmt(pixfmt)) {
			/*
			 * The other OSD window is already configured for an RGB
			 * or YC pixel format, so keep the current layer
			 * configuration.
			 */
			*lconfig = win->lconfig;
			return 1;
		}
	}

	/* DM6446: only one video window at a time can use RGB888 */
	if (is_vid_win(layer) && lconfig->pixfmt == PIXFMT_RGB888) {
		enum davinci_pix_format pixfmt;

		if (layer == WIN_VID0)
			pixfmt = osd->win[WIN_VID1].lconfig.pixfmt;
		else
			pixfmt = osd->win[WIN_VID0].lconfig.pixfmt;

		if (pixfmt == PIXFMT_RGB888) {
			/*
			 * The other video window is already configured for
			 * RGB888, so keep the current layer configuration.
			 */
			*lconfig = win->lconfig;
			return 1;
		}
	}

	/* window dimensions must be non-zero */
	if (!lconfig->line_length || !lconfig->xsize || !lconfig->ysize) {
		*lconfig = win->lconfig;
		return 1;
	}

	/* round line_length up to a multiple of 32 */
	lconfig->line_length = ((lconfig->line_length + 31) / 32) * 32;
	lconfig->line_length =
	    min(lconfig->line_length, (unsigned)MAX_LINE_LENGTH);
	lconfig->xsize = min(lconfig->xsize, (unsigned)MAX_WIN_SIZE);
	lconfig->ysize = min(lconfig->ysize, (unsigned)MAX_WIN_SIZE);
	lconfig->xpos = min(lconfig->xpos, (unsigned)MAX_WIN_SIZE);
	lconfig->ypos = min(lconfig->ypos, (unsigned)MAX_WIN_SIZE);
	lconfig->interlaced = (lconfig->interlaced != 0);
	if (lconfig->interlaced) {
		/* ysize and ypos must be even for interlaced displays */
		lconfig->ysize &= ~1;
		lconfig->ypos &= ~1;
	}

	return 0;
}

int davinci_disp_try_layer_config(enum davinci_disp_layer layer,
				  struct davinci_layer_config *lconfig)
{
	int reject_config;
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	reject_config = try_layer_config(layer, lconfig);

	spin_unlock_irqrestore(&osd->lock, flags);

	return reject_config;
}

static void _davinci_disp_disable_vid_rgb888(void)
{
	/*
	 * The DM6446 supports RGB888 pixel format in a single video window.
	 * This routine disables RGB888 pixel format for both video windows.
	 * The caller must ensure that neither video window is currently
	 * configured for RGB888 pixel format.
	 */
	if (cpu_is_davinci_dm644x())
		osd_clear(OSD_MISCCTL_RGBEN, OSD_MISCCTL);
}

static void _davinci_disp_enable_vid_rgb888(enum davinci_disp_layer layer)
{
	/*
	 * The DM6446 supports RGB888 pixel format in a single video window.
	 * This routine enables RGB888 pixel format for the specified video
	 * window.  The caller must ensure that the other video window is not
	 * currently configured for RGB888 pixel format, as this routine will
	 * disable RGB888 pixel format for the other window.
	 */
	if (cpu_is_davinci_dm644x()) {
		if (layer == WIN_VID0) {
			osd_merge(OSD_MISCCTL_RGBEN | OSD_MISCCTL_RGBWIN,
				  OSD_MISCCTL_RGBEN, OSD_MISCCTL);
		} else if (layer == WIN_VID1) {
			osd_merge(OSD_MISCCTL_RGBEN | OSD_MISCCTL_RGBWIN,
				  OSD_MISCCTL_RGBEN | OSD_MISCCTL_RGBWIN,
				  OSD_MISCCTL);
		}
	}
}

static void _davinci_disp_set_cbcr_order(enum davinci_pix_format pixfmt)
{
	/*
	 * The caller must ensure that all windows using YC pixfmt use the same
	 * Cb/Cr order.
	 */
	if (pixfmt == PIXFMT_YCbCrI)
		osd_clear(OSD_MODE_CS, OSD_MODE);
	else if (pixfmt == PIXFMT_YCrCbI)
		osd_set(OSD_MODE_CS, OSD_MODE);
}

static void _davinci_disp_set_layer_config(enum davinci_disp_layer layer, const struct davinci_layer_config
					   *lconfig)
{
	u32 winmd = 0, winmd_mask = 0, bmw = 0;

	_davinci_disp_set_cbcr_order(lconfig->pixfmt);

	switch (layer) {
	case WIN_OSD0:
		if (cpu_is_davinci_dm644x()) {
			winmd_mask |= OSD_OSDWIN0MD_RGB0E;
			if (lconfig->pixfmt == PIXFMT_RGB565)
				winmd |= OSD_OSDWIN0MD_RGB0E;
		} else if (cpu_is_davinci_dm355()) {
			winmd_mask |= OSD_OSDWIN0MD_BMP0MD;
			switch (lconfig->pixfmt) {
			case PIXFMT_RGB565:
				winmd |= (1 << OSD_OSDWIN0MD_BMP0MD_SHIFT);
				break;
			case PIXFMT_RGB888:
				winmd |= (2 << OSD_OSDWIN0MD_BMP0MD_SHIFT);
				break;
			case PIXFMT_YCbCrI:
			case PIXFMT_YCrCbI:
				winmd |= (3 << OSD_OSDWIN0MD_BMP0MD_SHIFT);
				break;
			default:
				break;
			}
		}

		winmd_mask |= OSD_OSDWIN0MD_BMW0 | OSD_OSDWIN0MD_OFF0;

		switch (lconfig->pixfmt) {
		case PIXFMT_1BPP:
			bmw = 0;
			break;
		case PIXFMT_2BPP:
			bmw = 1;
			break;
		case PIXFMT_4BPP:
			bmw = 2;
			break;
		case PIXFMT_8BPP:
			bmw = 3;
			break;
		default:
			break;
		}
		winmd |= (bmw << OSD_OSDWIN0MD_BMW0_SHIFT);

		if (lconfig->interlaced)
			winmd |= OSD_OSDWIN0MD_OFF0;

		osd_merge(winmd_mask, winmd, OSD_OSDWIN0MD);
		osd_write(lconfig->line_length >> 5, OSD_OSDWIN0OFST);
		osd_write(lconfig->xpos, OSD_OSDWIN0XP);
		osd_write(lconfig->xsize, OSD_OSDWIN0XL);
		if (lconfig->interlaced) {
			osd_write(lconfig->ypos >> 1, OSD_OSDWIN0YP);
			osd_write(lconfig->ysize >> 1, OSD_OSDWIN0YL);
		} else {
			osd_write(lconfig->ypos, OSD_OSDWIN0YP);
			osd_write(lconfig->ysize, OSD_OSDWIN0YL);
		}
		break;
	case WIN_VID0:
		winmd_mask |= OSD_VIDWINMD_VFF0;
		if (lconfig->interlaced)
			winmd |= OSD_VIDWINMD_VFF0;

		osd_merge(winmd_mask, winmd, OSD_VIDWINMD);
		osd_write(lconfig->line_length >> 5, OSD_VIDWIN0OFST);
		osd_write(lconfig->xpos, OSD_VIDWIN0XP);
		osd_write(lconfig->xsize, OSD_VIDWIN0XL);
		if (lconfig->interlaced) {
			osd_write(lconfig->ypos >> 1, OSD_VIDWIN0YP);
			osd_write(lconfig->ysize >> 1, OSD_VIDWIN0YL);
		} else {
			osd_write(lconfig->ypos, OSD_VIDWIN0YP);
			osd_write(lconfig->ysize, OSD_VIDWIN0YL);
		}
		break;
	case WIN_OSD1:
		/*
		 * The caller must ensure that OSD1 is disabled prior to
		 * switching from a normal mode to attribute mode or from
		 * attribute mode to a normal mode.
		 */
		if (lconfig->pixfmt == PIXFMT_OSD_ATTR) {
			if (cpu_is_davinci_dm644x()) {
				winmd_mask |=
				    OSD_OSDWIN1MD_ATN1E | OSD_OSDWIN1MD_RGB1E |
				    OSD_OSDWIN1MD_CLUTS1 | OSD_OSDWIN1MD_BLND1 |
				    OSD_OSDWIN1MD_TE1;
			} else {
				winmd_mask |=
				    OSD_OSDWIN1MD_BMP1MD | OSD_OSDWIN1MD_CLUTS1
				    | OSD_OSDWIN1MD_BLND1 | OSD_OSDWIN1MD_TE1;
			}
		} else {
			if (cpu_is_davinci_dm644x()) {
				winmd_mask |= OSD_OSDWIN1MD_RGB1E;
				if (lconfig->pixfmt == PIXFMT_RGB565)
					winmd |= OSD_OSDWIN1MD_RGB1E;
			} else if (cpu_is_davinci_dm355()) {
				winmd_mask |= OSD_OSDWIN1MD_BMP1MD;
				switch (lconfig->pixfmt) {
				case PIXFMT_RGB565:
					winmd |=
					    (1 << OSD_OSDWIN1MD_BMP1MD_SHIFT);
					break;
				case PIXFMT_RGB888:
					winmd |=
					    (2 << OSD_OSDWIN1MD_BMP1MD_SHIFT);
					break;
				case PIXFMT_YCbCrI:
				case PIXFMT_YCrCbI:
					winmd |=
					    (3 << OSD_OSDWIN1MD_BMP1MD_SHIFT);
					break;
				default:
					break;
				}
			}

			winmd_mask |= OSD_OSDWIN1MD_BMW1;
			switch (lconfig->pixfmt) {
			case PIXFMT_1BPP:
				bmw = 0;
				break;
			case PIXFMT_2BPP:
				bmw = 1;
				break;
			case PIXFMT_4BPP:
				bmw = 2;
				break;
			case PIXFMT_8BPP:
				bmw = 3;
				break;
			default:
				break;
			}
			winmd |= (bmw << OSD_OSDWIN1MD_BMW1_SHIFT);
		}

		winmd_mask |= OSD_OSDWIN1MD_OFF1;
		if (lconfig->interlaced)
			winmd |= OSD_OSDWIN1MD_OFF1;

		osd_merge(winmd_mask, winmd, OSD_OSDWIN1MD);
		osd_write(lconfig->line_length >> 5, OSD_OSDWIN1OFST);
		osd_write(lconfig->xpos, OSD_OSDWIN1XP);
		osd_write(lconfig->xsize, OSD_OSDWIN1XL);
		if (lconfig->interlaced) {
			osd_write(lconfig->ypos >> 1, OSD_OSDWIN1YP);
			osd_write(lconfig->ysize >> 1, OSD_OSDWIN1YL);
		} else {
			osd_write(lconfig->ypos, OSD_OSDWIN1YP);
			osd_write(lconfig->ysize, OSD_OSDWIN1YL);
		}
		break;
	case WIN_VID1:
		winmd_mask |= OSD_VIDWINMD_VFF1;
		if (lconfig->interlaced)
			winmd |= OSD_VIDWINMD_VFF1;

		osd_merge(winmd_mask, winmd, OSD_VIDWINMD);
		osd_write(lconfig->line_length >> 5, OSD_VIDWIN1OFST);
		osd_write(lconfig->xpos, OSD_VIDWIN1XP);
		osd_write(lconfig->xsize, OSD_VIDWIN1XL);
		if (lconfig->interlaced) {
			osd_write(lconfig->ypos >> 1, OSD_VIDWIN1YP);
			osd_write(lconfig->ysize >> 1, OSD_VIDWIN1YL);
		} else {
			osd_write(lconfig->ypos, OSD_VIDWIN1YP);
			osd_write(lconfig->ysize, OSD_VIDWIN1YL);
		}
		break;
	}
}

int davinci_disp_set_layer_config(enum davinci_disp_layer layer,
				  struct davinci_layer_config *lconfig)
{
	struct davinci_window_state *win = &osd->win[layer];
	int reject_config;
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	reject_config = try_layer_config(layer, lconfig);
	if (reject_config) {
		spin_unlock_irqrestore(&osd->lock, flags);
		return reject_config;
	}

	/* update the current Cb/Cr order */
	if (is_yc_pixfmt(lconfig->pixfmt))
		osd->yc_pixfmt = lconfig->pixfmt;

	/*
	 * If we are switching OSD1 from normal mode to attribute mode or from
	 * attribute mode to normal mode, then we must disable the window.
	 */
	if (layer == WIN_OSD1) {
		if (((lconfig->pixfmt == PIXFMT_OSD_ATTR)
		     && (win->lconfig.pixfmt != PIXFMT_OSD_ATTR))
		    || ((lconfig->pixfmt != PIXFMT_OSD_ATTR)
			&& (win->lconfig.pixfmt == PIXFMT_OSD_ATTR))) {
			win->is_enabled = 0;
			_davinci_disp_disable_layer(layer);
		}
	}

	_davinci_disp_set_layer_config(layer, lconfig);

	if (layer == WIN_OSD1) {
		struct davinci_osdwin_state *osdwin_state =
		    &osd->osdwin[OSDWIN_OSD1];

		if ((lconfig->pixfmt != PIXFMT_OSD_ATTR)
		    && (win->lconfig.pixfmt == PIXFMT_OSD_ATTR)) {
			/*
			 * We just switched OSD1 from attribute mode to normal
			 * mode, so we must initialize the CLUT select, the
			 * blend factor, transparency colorkey enable, and
			 * attenuation enable (DM6446 only) bits in the
			 * OSDWIN1MD register.
			 */
			_davinci_disp_set_osd_clut(OSDWIN_OSD1,
						   osdwin_state->clut);
			_davinci_disp_set_blending_factor(OSDWIN_OSD1,
							  osdwin_state->blend);
			if (osdwin_state->colorkey_blending) {
				_davinci_disp_enable_color_key(OSDWIN_OSD1,
							       osdwin_state->
							       colorkey,
							       lconfig->pixfmt);
			} else
				_davinci_disp_disable_color_key(OSDWIN_OSD1);
			_davinci_disp_set_rec601_attenuation(OSDWIN_OSD1,
							     osdwin_state->
							     rec601_attenuation);
		} else if ((lconfig->pixfmt == PIXFMT_OSD_ATTR)
			   && (win->lconfig.pixfmt != PIXFMT_OSD_ATTR)) {
			/*
			 * We just switched OSD1 from normal mode to attribute
			 * mode, so we must initialize the blink enable and
			 * blink interval bits in the OSDATRMD register.
			 */
			_davinci_disp_set_blink_attribute(osd->is_blinking,
							  osd->blink);
		}
	}

	/*
	 * If we just switched to a 1-, 2-, or 4-bits-per-pixel bitmap format
	 * then configure a default palette map.
	 */
	if ((lconfig->pixfmt != win->lconfig.pixfmt)
	    && ((lconfig->pixfmt == PIXFMT_1BPP)
		|| (lconfig->pixfmt == PIXFMT_2BPP)
		|| (lconfig->pixfmt == PIXFMT_4BPP))) {
		enum davinci_osd_layer osdwin =
		    ((layer == WIN_OSD0) ? OSDWIN_OSD0 : OSDWIN_OSD1);
		struct davinci_osdwin_state *osdwin_state =
		    &osd->osdwin[osdwin];
		unsigned char clut_index;
		unsigned char clut_entries = 0;

		switch (lconfig->pixfmt) {
		case PIXFMT_1BPP:
			clut_entries = 2;
			break;
		case PIXFMT_2BPP:
			clut_entries = 4;
			break;
		case PIXFMT_4BPP:
			clut_entries = 16;
			break;
		default:
			break;
		}
		/*
		 * The default palette map maps the pixel value to the clut
		 * index, i.e. pixel value 0 maps to clut entry 0, pixel value 1
		 * maps to clut entry 1, etc.
		 */
		for (clut_index = 0; clut_index < 16; clut_index++) {
			osdwin_state->palette_map[clut_index] = clut_index;
			if (clut_index < clut_entries) {
				_davinci_disp_set_palette_map(osdwin,
							      clut_index,
							      clut_index,
							      lconfig->pixfmt);
			}
		}
	}

	win->lconfig = *lconfig;

	/* DM6446: configure the RGB888 enable and window selection */
	if (osd->win[WIN_VID0].lconfig.pixfmt == PIXFMT_RGB888)
		_davinci_disp_enable_vid_rgb888(WIN_VID0);
	else if (osd->win[WIN_VID1].lconfig.pixfmt == PIXFMT_RGB888)
		_davinci_disp_enable_vid_rgb888(WIN_VID1);
	else
		_davinci_disp_disable_vid_rgb888();

	if (layer == WIN_VID0) {
		osd->pingpong =
		    _davinci_disp_dm6446_vid0_pingpong(osd->field_inversion,
						       win->fb_base_phys,
						       &win->lconfig);
	}

	spin_unlock_irqrestore(&osd->lock, flags);

	return 0;
}

void davinci_disp_init_layer(enum davinci_disp_layer layer)
{
	struct davinci_window_state *win = &osd->win[layer];
	enum davinci_osd_layer osdwin;
	struct davinci_osdwin_state *osdwin_state;
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	win->is_enabled = 0;
	_davinci_disp_disable_layer(layer);

	win->h_zoom = ZOOM_X1;
	win->v_zoom = ZOOM_X1;
	_davinci_disp_set_zoom(layer, win->h_zoom, win->v_zoom);

	win->fb_base_phys = 0;
	_davinci_disp_start_layer(layer, win->fb_base_phys);

	win->lconfig.line_length = 0;
	win->lconfig.xsize = 0;
	win->lconfig.ysize = 0;
	win->lconfig.xpos = 0;
	win->lconfig.ypos = 0;
	win->lconfig.interlaced = 0;
	switch (layer) {
	case WIN_OSD0:
	case WIN_OSD1:
		osdwin = (layer == WIN_OSD0) ? OSDWIN_OSD0 : OSDWIN_OSD1;
		osdwin_state = &osd->osdwin[osdwin];
		/*
		 * Other code relies on the fact that OSD windows default to a
		 * bitmap pixel format when they are deallocated, so don't
		 * change this default pixel format.
		 */
		win->lconfig.pixfmt = PIXFMT_8BPP;
		_davinci_disp_set_layer_config(layer, &win->lconfig);
		osdwin_state->clut = RAM_CLUT;
		_davinci_disp_set_osd_clut(osdwin, osdwin_state->clut);
		osdwin_state->colorkey_blending = 0;
		_davinci_disp_disable_color_key(osdwin);
		osdwin_state->blend = OSD_8_VID_0;
		_davinci_disp_set_blending_factor(osdwin, osdwin_state->blend);
		osdwin_state->rec601_attenuation = 0;
		_davinci_disp_set_rec601_attenuation(osdwin,
						     osdwin_state->
						     rec601_attenuation);
		if (osdwin == OSDWIN_OSD1) {
			osd->is_blinking = 0;
			osd->blink = BLINK_X1;
		}
		break;
	case WIN_VID0:
	case WIN_VID1:
		win->lconfig.pixfmt = osd->yc_pixfmt;
		_davinci_disp_set_layer_config(layer, &win->lconfig);
		break;
	}

	spin_unlock_irqrestore(&osd->lock, flags);
}

void davinci_disp_release_layer(enum davinci_disp_layer layer)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	if (!win->is_allocated) {
		spin_unlock_irqrestore(&osd->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&osd->lock, flags);
	davinci_disp_init_layer(layer);
	spin_lock_irqsave(&osd->lock, flags);

	win->is_allocated = 0;

	spin_unlock_irqrestore(&osd->lock, flags);
}

int davinci_disp_request_layer(enum davinci_disp_layer layer)
{
	struct davinci_window_state *win = &osd->win[layer];
	unsigned long flags;

	spin_lock_irqsave(&osd->lock, flags);

	if (win->is_allocated) {
		spin_unlock_irqrestore(&osd->lock, flags);
		return -1;
	}
	win->is_allocated = 1;

	spin_unlock_irqrestore(&osd->lock, flags);
	return 0;
}

static void _davinci_disp_init(void)
{
	osd_write(0, OSD_MODE);
	osd_write(0, OSD_VIDWINMD);
	osd_write(0, OSD_OSDWIN0MD);
	osd_write(0, OSD_OSDWIN1MD);
	osd_write(0, OSD_RECTCUR);
	osd_write(0, OSD_MISCCTL);
	if (cpu_is_davinci_dm355()) {
		osd_write(0, OSD_VBNDRY);
		osd_write(0, OSD_EXTMODE);
		osd_write(OSD_MISCCTL_DMANG, OSD_MISCCTL);
	}
}

int davinci_disp_init(void)
{
	spin_lock_init(&osd->lock);

	if (cpu_is_davinci_dm644x()) {
		osd->osdregs = DM644X_OSD_REG_BASE;
		osd->vencregs = DM644X_VENC_REG_BASE;
	} else if (cpu_is_davinci_dm355()) {
		osd->osdregs = DM355_OSD_REG_BASE;
		osd->vencregs = DM355_VENC_REG_BASE;
	} else
		return -1;

	osd->irq = IRQ_VENCINT;

	_davinci_disp_init();

	/* set default Cb/Cr order */
	osd->yc_pixfmt = PIXFMT_YCbCrI;

	if (cpu_is_davinci_dm644x()) {
		/*
		 * DM6446 silicon advisory 1.3.8 says that when using interlaced
		 * video output, the field signal is improperly inverted for
		 * OSD0, OSD1, and VID1.  The field signal is not inverted for
		 * VID0.  The workaround is to set the field signal inversion
		 * bit so that OSD0, OSD1, and VID1 have the correct field
		 * polarity.  The VID0 ping-pong buffer register will be used
		 * in the encoder ISR to compensate for the reversed field
		 * polarity of VID0.
		 */
		osd->field_inversion = 1;
	}
	if (cpu_is_davinci_dm355()) {
		/*
		 * ROM CLUT1 on the DM355 is similar (identical?) to ROM CLUT0
		 * on the DM6446, so make ROM_CLUT1 the default on the DM355.
		 */
		osd->rom_clut = ROM_CLUT1;
	}
	_davinci_disp_set_field_inversion(osd->field_inversion);
	_davinci_disp_set_rom_clut(osd->rom_clut);

	davinci_disp_init_layer(WIN_OSD0);
	davinci_disp_init_layer(WIN_VID0);
	davinci_disp_init_layer(WIN_OSD1);
	davinci_disp_init_layer(WIN_VID1);

	if (request_irq
	    (osd->irq, davinci_disp_isr, SA_SHIRQ, "davinci_osd", osd))
		return -1;

	return 0;
}

subsys_initcall(davinci_disp_init);

EXPORT_SYMBOL(davinci_disp_request_layer);
EXPORT_SYMBOL(davinci_disp_release_layer);
EXPORT_SYMBOL(davinci_disp_init_layer);
EXPORT_SYMBOL(davinci_disp_enable_layer);
EXPORT_SYMBOL(davinci_disp_disable_layer);
EXPORT_SYMBOL(davinci_disp_layer_is_enabled);
EXPORT_SYMBOL(davinci_disp_set_layer_config);
EXPORT_SYMBOL(davinci_disp_try_layer_config);
EXPORT_SYMBOL(davinci_disp_get_layer_config);
EXPORT_SYMBOL(davinci_disp_start_layer);
EXPORT_SYMBOL(davinci_disp_set_interpolation_filter);
EXPORT_SYMBOL(davinci_disp_get_interpolation_filter);
EXPORT_SYMBOL(davinci_disp_set_osd_expansion);
EXPORT_SYMBOL(davinci_disp_get_osd_expansion);
EXPORT_SYMBOL(davinci_disp_set_vid_expansion);
EXPORT_SYMBOL(davinci_disp_get_vid_expansion);
EXPORT_SYMBOL(davinci_disp_set_zoom);
EXPORT_SYMBOL(davinci_disp_get_zoom);
EXPORT_SYMBOL(davinci_disp_set_background);
EXPORT_SYMBOL(davinci_disp_get_background);
EXPORT_SYMBOL(davinci_disp_set_rom_clut);
EXPORT_SYMBOL(davinci_disp_get_rom_clut);
EXPORT_SYMBOL(davinci_disp_set_clut_ycbcr);
EXPORT_SYMBOL(davinci_disp_set_clut_rgb);
EXPORT_SYMBOL(davinci_disp_set_osd_clut);
EXPORT_SYMBOL(davinci_disp_get_osd_clut);
EXPORT_SYMBOL(davinci_disp_enable_color_key);
EXPORT_SYMBOL(davinci_disp_disable_color_key);
EXPORT_SYMBOL(davinci_disp_set_blending_factor);
EXPORT_SYMBOL(davinci_disp_get_blending_factor);
EXPORT_SYMBOL(davinci_disp_set_rec601_attenuation);
EXPORT_SYMBOL(davinci_disp_get_rec601_attenuation);
EXPORT_SYMBOL(davinci_disp_set_palette_map);
EXPORT_SYMBOL(davinci_disp_get_palette_map);
EXPORT_SYMBOL(davinci_disp_set_blink_attribute);
EXPORT_SYMBOL(davinci_disp_get_blink_attribute);
EXPORT_SYMBOL(davinci_disp_cursor_enable);
EXPORT_SYMBOL(davinci_disp_cursor_disable);
EXPORT_SYMBOL(davinci_disp_cursor_is_enabled);
EXPORT_SYMBOL(davinci_disp_set_cursor_config);
EXPORT_SYMBOL(davinci_disp_get_cursor_config);
EXPORT_SYMBOL(davinci_disp_set_field_inversion);
EXPORT_SYMBOL(davinci_disp_get_field_inversion);
EXPORT_SYMBOL(davinci_disp_register_callback);
EXPORT_SYMBOL(davinci_disp_unregister_callback);
MODULE_LICENSE("GPL");
