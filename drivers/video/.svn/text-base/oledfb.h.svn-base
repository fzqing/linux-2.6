#ifndef _VIDEO_OLEDFB_H
#define _VIDEO_OLEDFB_H

#define LCD_CMD_FRAME_DATA_WRITE	(0x2 << 9)
#define LCD_CMD_WAIT_FOR_VSYNC		(0x3 << 9)
#define LCD_CMD_COMMAND_WRITE		(0x1 << 9)
#define LCD_CMD_DATA_WRITE		(0x1 << 9)

#define LCD_CMD_NOP			(0x4 << 9)

#define LCD_CMD_INT_PROC		(0x5 << 9)
#define LCD_CMD_A0_COMMAND		(0x0 << 8)
#define LCD_CMD_A0_DATA			(0x1 << 8)

#define XLLP_LCCR1_PPL(n)		((n))
#define XLLP_LCCR2_LPP(n)		((n))
#define XLLP_LCCR3_BPP(n)		((((n) & 0x7) << 24) | (((n) & 0x8) << 29))
#define XLLP_LCCR3_PDFOR(n)		((n) << 30)
#define XLLP_LCCR4_PAL_FOR(n)		((n)<<15)

#define XLLP_LCCR1_HSW(n)		((n) << 10)
#define XLLP_LCCR1_ELW(n)		((n) << 16)
#define XLLP_LCCR1_BLW(n)		((n) << 24)

#define XLLP_FDADR_DESCADDR(n)		((n) & 0xFFFFFFF0)
#define XLLP_FSADR_SRCADDR(n)		((n) & 0xFFFFFFF8)
#define XLLP_FIDR_FRAMEID(n)		((n) & 0xFFFFFFF8)
#define XLLP_LDCMD_LEN(n)		((n)&0x1ffffc)

#define XLLP_LCCR6_B_BLUE(n)		((n)<<0)
#define XLLP_LCCR6_B_GREEN(n)		((n)<<8)
#define XLLP_LCCR6_B_RED(n)		((n)<<16)

#define XLLP_LCCR3_ACB(n)		((n) << 8)

#define XLLP_LCCR3_PCD(n)		((n))

#define MAKEUP_CMD(x)			(LCD_CMD_COMMAND_WRITE|LCD_CMD_A0_COMMAND|(x))
#define MAKEUP_DATA(x)			(LCD_CMD_DATA_WRITE|LCD_CMD_A0_DATA|(x))

#define CMDCR_SYNC_CNT(n)		((n))

#define LCSR0_LDD		0x00000001
#define LCSR0_SOF0		0x00000002
#define LCSR0_BER		0x00000004
#define LCSR0_ABC		0x00000008
#define LCSR0_IU0		0x00000010
#define LCSR0_IU1		0x00000020
#define LCSR0_OU		0x00000040
#define LCSR0_QD		0x00000080
#define LCSR0_EOF0		0x00000100
#define LCSR0_BS0		0x00000200
#define LCSR0_SINT		0x00000400
#define LCSR0_RD_ST		0x00000800

#define LCSR_CMD_INT		0x00001000

#define LCSR0_REOF0		(0x1u<<13)
#define LCSR0_REOF1		(0x1u<<14)
#define LCSR0_REOF2		(0x1u<<15)
#define LCSR0_REOF3		(0x1u<<16)
#define LCSR0_REOF4		(0x1u<<17)
#define LCSR0_REOF5		(0x1u<<18)
#define LCSR0_REOF6		(0x1u<<19)

struct smart_timing {
	unsigned int BLW;	/* WR, RD pulse width */
	unsigned int ELW;	/* A0, CS, setup/hold */
	unsigned int HSW;	/* output hold */
	unsigned int PCD;	/* command inhibit */
	unsigned int SYNC_CNT;	/* synchronous count, count after vsync signal */
};

struct smart_panel {

	unsigned int width;
	unsigned int height;
	int bpp;

	struct smart_timing timing;
};

#define OLED_WIDTH	128
#define OLED_HEIGHT	128

struct  LCD_FRAME_DESCRIPTOR {
	unsigned int FDADR;		/* Pointer to next frame descriptor (Physical address) */
	unsigned int FSADR;		/* Pointer to the data (Physical address) */
	unsigned int FIDR;		/* Frame descriptor ID */
	unsigned int LDCMD;		/* DMA command */
};

struct lcd_smart_info {
	struct fb_info fb;

	unsigned int BPP;
	unsigned int PixelDataFormat;

	u_int	max_bpp;
	u_int	max_xres;
	u_int	max_yres;

	/*
	 * These are the addresses we mapped
	 * the framebuffer memory region to.
	 */

	/* raw memory addresses */
	dma_addr_t	map_dma;	/* physical */
	u_char *	map_cpu;	/* virtual */
	u_int		map_size;

	/* addresses of pieces placed in raw buffer */
	u_char *	screen_cpu;	/* virtual address of frame buffer */
	dma_addr_t	screen_dma;	/* physical address of frame buffer */

	/*
	 * Palette is used only in palette color mode.
	 * Ch2_YCbCr_Y is used only for overlays.
	 * Ch3_YCbCr_Cb is used only for overlay 2 YCbCr mode.
	 * Ch4_YCbCr_Cr is used only for overlay 2 YCbCr mode.
	 */
	unsigned int _FRAME_BUFFER_BASE_PHYSICAL;
	unsigned int _PALETTE_BUFFER_BASE_PHYSICAL;
	unsigned int _OVERLAY2_Y_CHANNEL_BASE_PHYSICAL;
	unsigned int _OVERLAY2_Cb_CHANNEL_BASE_PHYSICAL;
	unsigned int _OVERLAY2_Cr_CHANNEL_BASE_PHYSICAL;
	unsigned int _COMMAND_BUFFER_BASE_PHYSICAL;
	unsigned short *cmd_buf;
	unsigned int cmd_max;
	dma_addr_t   cmd_buf_dma;

	/*
	 * The memory for following desc need to be allocated
	 * if the corresponding channel is used, while the content
	 * in the memory need not to be initialized.
	 * _DMA_CHANNEL_0_ALT_FRAME_DESCRIPTOR_BASE_PHYSICAL/frameDescriptorCh0fd2
	 * is used only if a palette load is performed.
	 */

	/* frame descriptors (physical address) */
	unsigned int _DMA_CHANNEL_0_FRAME_DESCRIPTOR_BASE_PHYSICAL;
	unsigned int _DMA_CHANNEL_0_ALT_FRAME_DESCRIPTOR_BASE_PHYSICAL;
	unsigned int _PALETTE_FRAME_DESCRIPTOR_BASE_PHYSICAL;
	unsigned int _DMA_CHANNEL_2_Y_FRAME_DESCRIPTOR_BASE_PHYSICAL;
	unsigned int _DMA_CHANNEL_3_Cb_FRAME_DESCRIPTOR_BASE_PHYSICAL;
	unsigned int _DMA_CHANNEL_4_Cr_FRAME_DESCRIPTOR_BASE_PHYSICAL;
	unsigned int _DMA_CHANNEL_6_COMMAND_DESCRIPTOR_BASE_PHYSICAL;

	/* frame descriptors (virtual address) */
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorCh0fd1;
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorCh0fd2;
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorPalette;
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorCh2_YCbCr_Y;
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorCh3_YCbCr_Cb;
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorCh4_YCbCr_Cr;
	struct LCD_FRAME_DESCRIPTOR *frameDescriptorCh6_command;

	/* Fields for internal use. Need no initialization by user. */
	unsigned int	FrameBufferSize;
	unsigned int	PaletteSize;
	unsigned short *cmd_current;

	struct work_struct	task;
	struct semaphore	ctrlr_sem;
	u_char			state;
	u_char			task_state;

	/*
	 * The following are init by panel init or switch codes.
	 * smart panel or tft panel
	 */
	struct smart_panel *panel;
};

#define PRSR_ST_OK		0x00000200
#define PRSR_CON_ST		0x00000400

#define FBIOSCREENUPDATE	0x461D
#define FBIOSENDCMD		0x461F
#define PXA_SMART_CMD_BUF_MAX_NUM		50

/*
 * Used by ioctl(FBIOSENDCMD).
 * Looks no way to transfer non-fixed size data using ioctl()
 */
#define PXA_SMARTPANEL_CMD_BUF_MAX_NUM		50
struct pxa_smartpanel_cmdbuf {
	unsigned int cmd_num;
	unsigned short cmds[PXA_SMARTPANEL_CMD_BUF_MAX_NUM];
};

#endif
