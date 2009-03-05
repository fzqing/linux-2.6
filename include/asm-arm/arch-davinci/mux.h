/*
 * Table of the DAVINCI register configurations for the PINMUX combinations
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on linux/include/asm-arm/arch-omap/mux.h:
 * Copyright (C) 2003 - 2005 Nokia Corporation
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __ASM_ARCH_MUX_H
#define __ASM_ARCH_MUX_H

#ifdef	CONFIG_DAVINCI_MUX_DEBUG
#define MUX_REG(reg, mode_offset, mode_mask, mux_mode) \
			.mux_reg_name = "PINMUX"#reg, \
			.mux_reg = PINMUX##reg, \
			.mask_offset = mode_offset, \
			.mask = mode_mask, \
			.mode = mux_mode,
#else
#define MUX_REG(reg, mode_offset, mode_mask, mux_mode) \
			.mux_reg = PINMUX##reg, \
			.mask_offset = mode_offset, \
			.mask = mode_mask, \
			.mode = mux_mode,
#endif /* CONFIG_DAVINCI_MUX_DEBUG */

#define MUX_CFG(desc, mux_reg, mode_offset, mode_mask,		\
		mux_mode, dbg)					\
{								\
	.name =	 desc,						\
	.debug = dbg,						\
	MUX_REG(mux_reg, mode_offset, mode_mask, mux_mode)	\
},

struct pin_config {
	char *name;
	unsigned char busy;
	unsigned char debug;

	const char *mux_reg_name;
	const unsigned int mux_reg;
	const unsigned char mask_offset;
	const unsigned char mask;
	const unsigned char mode;
};

enum davinci_dm644x_index {
	/* ATA and HDDIR functions */
	DM644X_HDIREN,
	DM644X_ATAEN,

	/* Memory Stick */
	DM644X_MSTK,

	/* I2C */
	DM644X_I2C,

	/* ASP function */
	DM644X_MCBSP0,

	/* PWM0 */
	DM644X_PWM0,

	/* PWM1 */
	DM644X_PWM1,

	/* PWM2 */
	DM644X_PWM2,

	/* VLINQ function */
	DM644X_VLINQEN,
	DM644X_VLINQWD,

	/* EMAC and MDIO function */
	DM644X_EMACEN,

	/* GPIO3V[0:16] pins */
	DM644X_GPIO3V,

	/* GPIO pins */
	DM644X_GPIO0,
	DM644X_GPIO3,
	DM644X_GPIO43_44,
	DM644X_GPIO46_47,

	/* VPBE */
	DM644X_RGB666,

	/* LCD */
	DM644X_LOEEN,
	DM644X_LFLDEN,
};

enum davinci_dm646x_index {
	/* ATA function */
	DM646X_ATAEN,

	/* USB */
	DM646X_VBUSDIS,
};

enum davinci_dm355_index {
	/* MMC/SD 0 */
	DM355_MMCSD0,

	/* MMC/SD 1 */
	DM355_SD1_CLK,
	DM355_SD1_CMD,
	DM355_SD1_DATA3,
	DM355_SD1_DATA2,
	DM355_SD1_DATA1,
	DM355_SD1_DATA0,

	/* I2C */
	DM355_I2C_SDA,
	DM355_I2C_SCL,

	/* ASP function */
	DM355_MCBSP0_BDX,
	DM355_MCBSP0_X,
	DM355_MCBSP0_BFSX,
	DM355_MCBSP0_BDR,
	DM355_MCBSP0_R,
	DM355_MCBSP0_BFSR,

	/* PWM0 */
	DM355_PWM0,

	/* PWM1 */
	DM355_PWM1,

	/* PWM2 */
	DM355_PWM2_G76,
	DM355_PWM2_G77,
	DM355_PWM2_G78,
	DM355_PWM2_G79,

	/* PWM3 */
	DM355_PWM3_G69,
	DM355_PWM3_G70,
	DM355_PWM3_G74,
	DM355_PWM3_G75,

	/* SPI0 */
	DM355_SPI0_SDI,
	DM355_SPI0_SDENA0,
	DM355_SPI0_SDENA1,

	/* SPI1 */
	DM355_SPI1_SCLK,
	DM355_SPI1_SDO,
	DM355_SPI1_SDENA0,
	DM355_SPI1_SDENA1,

	/* SPI2 */
	DM355_SPI2_SCLK,
	DM355_SPI2_SDO,
	DM355_SPI2_SDENA0,
	DM355_SPI2_SDENA1,

	/* GPIO */
	DM355_GPIO14,
	DM355_GPIO15,
	DM355_GPIO71,

	/* Video Out */
	DM355_VOUT_FIELD,
	DM355_VOUT_FIELD_G70,
	DM355_VOUT_HVSYNC,
	DM355_VOUT_COUTL_EN,
	DM355_VOUT_COUTH_EN,

	/* Video In */
	DM355_VIN_PCLK,
	DM355_VIN_CAM_WEN,
	DM355_VIN_CAM_VD,
	DM355_VIN_CAM_HD,
	DM355_VIN_YIN_EN,
	DM355_VIN_CINL_EN,
	DM355_VIN_CINH_EN,
};

#ifdef	CONFIG_DAVINCI_MUX
/* setup pin muxing in Linux */
extern int davinci_mux_init(void);
extern int davinci_mux_register(struct pin_config *pins, unsigned long size);
extern int davinci_cfg_reg(unsigned long reg_cfg);
#else
/* boot loader does it all (no warnings from CONFIG_DAVINCI_MUX_WARNINGS) */
static inline int davinci_mux_init(void) { return 0; }
static inline int davinci_cfg_reg(unsigned long reg_cfg) { return 0; }
#endif

extern void (*davinci_pinmux_setup)(unsigned int id);

#endif
