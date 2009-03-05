/*
 * linux/include/asm-arm/arch-pxa/u2d.h
 *
 * This supports machine-specific differences in how the PXA27x
 * USB 2.0 Device Controller (U2D) is wired.
 *
 * It is set in linux/arch/arm/mach-pxa/<machine>.c and used in
 * the probe routine of linux/drivers/usb/gadget/mhn_u2d.c
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
struct pxa27x_u2d_mach_info {
	int (*u2d_is_connected) (void);	/* do we see host? */
	void (*u2d_command) (int cmd);
#define	PXA2XX_U2D_CMD_CONNECT		0	/* let host see us */
#define	PXA2XX_U2D_CMD_DISCONNECT	1	/* so host won't see us */
};

extern void pxa_set_u2d_info(struct pxa27x_u2d_mach_info *info);
