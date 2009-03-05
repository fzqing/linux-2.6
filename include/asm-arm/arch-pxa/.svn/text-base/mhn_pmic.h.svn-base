/*
 *  linux/include/asm-arm/arch-pxa/mhn_pmic.h
 *
 * Copyright(C) 2006 Marvell Internaltional Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PMIC_H__
#define __PMIC_H__

#include <linux/i2c.h>
#include <linux/interrupt.h>

enum {
	/* Common command for Zylonite/Littleton */

	/* Set command > 0xFFFF0000 to avoid wrong
	 * parameter is used for pmic_set_voltage.
	 */
	VCC_CORE = 0xFFFF0000,
	VCC_SRAM,
	VCC_MVT,
	VCC_3V_APPS,
	VCC_SDIO,
	VCC_CAMERA_ANA,
	VCC_USB,
	VCC_LCD,
	VCC_TSI,
	VCC_CAMERA_IO,

	/* Command for Littleton */
	VCC_1P8V,
	VCC_MEM,
	HDMI_TX,
	TECH_3V,
	TECH_1P8V,
};

#define	PMIC_EVENT_EXTON	(1 << 0)
#define	PMIC_EVENT_VBUS		(1 << 1)
#define	PMIC_EVENT_USB		(PMIC_EVENT_EXTON | PMIC_EVENT_VBUS)

#define	PMIC_EVENT_TOUCH	(1 << 2)

#define PMIC_EVENT_OTGCP_IOVER	(1 << 3)

struct pmic_ops {
	int (*get_voltage) (int cmd, int *pmv);
	int (*set_voltage) (int cmd, int mv);

	int (*is_vbus_assert) (void);
	int (*is_avbusvld) (void);
	int (*is_asessvld) (void);
	int (*is_bsessvld) (void);
	int (*is_srp_ready) (void);

	int (*set_pump) (int enable);
	int (*set_vbus_supply) (int enable, int srp);
	int (*set_usbotg_a_mask) (void);
	int (*set_usbotg_b_mask) (void);
	int (*is_usbpump_chg) (void);

#ifdef CONFIG_PM
	int (*suspend) (struct device * _dev, u32 state, u32 level);
	int (*resume) (struct device * _dev, u32 level);
#endif
	int (*init) (struct device * dev);
	int (*deinit) (struct device * dev);
};

struct mhn_pmic_regs {
	unsigned int data:8;
	unsigned int hit:1;
	unsigned int mask:1;
};

extern void start_calc_time(void);
extern void end_calc_time(void);

extern int mhn_pmic_write(u8 reg, u8 val);
extern int mhn_pmic_read(u8 reg, u8 * pval);

extern int mhn_pmic_get_voltage(int cmd, int *pval);
extern int mhn_pmic_set_voltage(int cmd, int val);

/* Check whether USB VBUS is asserted */
extern int mhn_pmic_is_vbus_assert(void);
/* Check whether USB VBUS has gone above A-device VBUS valid threshold
 * Min: 4.4V	Max: N/A
 */
extern int mhn_pmic_is_avbusvld(void);
/* Check whether VBUS has gone above A-device Session Valid threshold
 * Min: 0.8V	Max: 2.0V
 */
extern int mhn_pmic_is_asessvld(void);
/* Check whether VBUS has gone above B-device Session Valid threshold
 * Min: 0.8V	Max: 4.0V
 */
extern int mhn_pmic_is_bsessvld(void);
/* Check whether VBUS has gone above B-device Session End threshold
 * Min: 0.2V	Max: 0.8V
 */
extern int mhn_pmic_is_srp_ready(void);
/* Initialize the USB PUMP */
extern int mhn_pmic_set_pump(int enable);
/* Check the events change of PMIC */
extern int mhn_pmic_event_change(void);
/* enable/disable VBUS supply */
extern int mhn_pmic_set_vbus_supply(int enable, int srp);
/* Set events mask for USB A-device
 * A-device Sessino Valid event
 * A-device VBUS Valid event
 */
extern int mhn_pmic_set_usbotg_a_mask(void);
/* Set events mask for USB B-device
 * B-device Session Valid event
 * B-device Session end event
 */
extern int mhn_pmic_set_usbotg_b_mask(void);

extern void pmic_set_ops(struct pmic_ops *ops);

#endif
