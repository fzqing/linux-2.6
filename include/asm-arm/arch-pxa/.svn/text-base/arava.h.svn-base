/*
 * include/asm-arm/arch-pxa/arava.h
 *
 * Copyright (C) 2006, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _ARAVA_H_
#define _ARAVA_H_
#include <linux/device.h>

#define ARAVA_REG_NUM	       (0xBA)

/* SYSMON */
#define ARAVA_CHIP_ID		0x00
#define ARAVA_EVENT_A 		0x01
#define ARAVA_EVENT_B 		0x02
#define ARAVA_EVENT_C 		0x03
#define ARAVA_STATUS  		0x04
#define ARAVA_IRQ_MASK_A 	0x05
#define ARAVA_IRQ_MASK_B 	0x06
#define ARAVA_IRQ_MASK_C 	0x07
#define ARAVA_SYSCTRL_A 	0x08
#define ARAVA_SYSCTRL_B		0x09
#define ARAVA_SYSCTRL_C 	0x80
#define ARAVA_FAULT_LOG		0x0A

/* REG */
#define ARAVA_LDO1011		0x10
#define ARAVA_LDO15		0x11
#define ARAVA_LDO1416		0x12
#define ARAVA_LDO1819		0x13
#define ARAVA_LDO17_SIMCP0	0x14
#define ARAVA_BUCK2DVC1		0x15
#define ARAVA_BUCK2DVC2		0x16
#define ARAVA_REGCTRL1		0x17
#define ARAVA_REGCTRL2		0x18
#define ARAVA_USBPUMP		0x19
#define ARAVA_APPSLEEP_CTRL	0x1A
#define ARAVA_STARTUP_CTRL	0x1B

#define ARAVA_LDO0405		0x92
/* LDO REG */
#define ARAVA_LDO01		0x90
#define ARAVA_LDO0203		0x91
#define ARAVA_LDO0405		0x92
#define ARAVA_LDO06SIMCP	0x93
#define ARAVA_LDO0708		0x94
#define ARAVA_LDO0912		0x95

/* CONTROL REG */
#define ARAVA_CON1		0x97
#define ARAVA_CON2		0x98
#define ARAVA_SLEEP_CON1	0x99
#define ARAVA_SLEEP_CON2	0x9A
#define ARAVA_SLEEP_CON3	0x9B

/* LED - ignored now. Skip */
#define ARAVA_LED1_CTRL		0x20
#define ARAVA_LED2_CTRL		0x21
#define ARAVA_LED3_CTRL		0x22
#define ARAVA_LED4_CTRL		0x23
#define ARAVA_LEDPC_CTRL	0x24
#define ARAVA_WLED_CTRL		0x25

/* MISC */
#define ARAVA_MISCA 		0x26
#define ARAVA_MISCB 		0x27

/* Charge */
#define ARAVA_CHARGE_CTRL	0x28
#define ARAVA_CCTR_CTRL		0x29
#define ARAVA_TCTR_CTRL		0x2A
#define ARAVA_CHARGE_PULSE	0x2B

#define ARAVA_ADDRESS 		0x49

#define ARAVA_VBUCK2BASE	850
#define ARAVA_VBUCK2STEP	25
#define ARAVA_VBUCK2MAX		1625

#define ARAVA_VLDO10BASE	1800
#define ARAVA_VLDO10STEP	100
#define ARAVA_VLDO10MAX		3200

#define ARAVA_VLDO14BASE	2760
#define ARAVA_VLDO14STEP	30
#define ARAVA_VLDO14MAX		2940

#define ARAVA_VLDO16BASE	1100
#define ARAVA_VLDO16STEP	50
#define ARAVA_VLDO16MAX		2650

#define ARAVA_VLDO18BASE	1800
#define ARAVA_VLDO18STEP	100
#define ARAVA_VLDO18MAX		3200

/* The bit definition of ARAVA_EVENT_A (0x01) */
#define ARAVA_EVENT_A_EXTON		(1 << 2)

/* The bit definition of ARAVA_EVENT_B (0x02) */
#define ARAVA_EVENT_B_VBUS_4P4		(1 << 3)
#define ARAVA_EVENT_B_VBUS_4P0		(1 << 4)
#define ARAVA_EVENT_B_SESSION_VALID	(1 << 5)
#define ARAVA_EVENT_B_SRP_DETECT	(1 << 6)

/* The bit definition of ARAVA_STATUS (0x04) */
#define ARAVA_STATUS_EXTON		(1 << 2)

/* The bit definition of ARAVA_IRQMASK_A (0x05) */
#define ARAVA_IRQMASK_A_EXTON		(1 << 2)

/* The bit definition of ARAVA_USBPUMP (0x19) */
#define ARAVA_USBPUMP_USBVE		(1 << 0)
#define ARAVA_USBPUMP_USBVEP		(1 << 1)
#define ARAVA_USBPUMP_VBUS_VALID_4_4	(1 << 2)
#define ARAVA_USBPUMP_VBUS_VALID_4_0	(1 << 3)
#define ARAVA_USBPUMP_SESSION_VALID	(1 << 4)
#define ARAVA_USBPUMP_SRP_DETECT	(1 << 5)
#define ARAVA_USBPUMP_EN_USBVE		(1 << 6)
#define ARAVA_USBPUMP_EN_USBVEP		(1 << 7)

/* The bit definition of ARAVA_MISCB (0x27) */
#define ARAVA_MISCB_USBINT_BOTHEDGE	(1 << 2)
#define ARAVA_MISCB_SESSION_VALID_EN	(1 << 3)

/* USB related definitions */
#define ARAVA_EVENT_VBUS		(1 << 0)
#define ARAVA_EVENT_VBUS_4P4		(1 << 2)
#define ARAVA_EVENT_VBUS_4P0		(1 << 4)
#define ARAVA_EVENT_SESSION		(1 << 6)
#define ARAVA_EVENT_SRP			(1 << 8)

#define ARAVA_USB_EVENTS	(ARAVA_EVENT_EXTON     |       \
				ARAVA_EVENT_VBUS_4P4   |       \
				ARAVA_EVENT_VBUS_4P0   |       \
				ARAVA_EVENT_SESSION    |       \
				ARAVA_EVENT_SRP)

extern int arava_platform_init(struct device *_dev);
extern int arava_platform_deinit(struct device *_dev);

#endif

