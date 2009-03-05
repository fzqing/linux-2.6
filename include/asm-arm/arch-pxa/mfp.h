/*
 * include/asm-arm/arch-pxa/mfp.h
 *
 * Copyright (C) 2006, Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MHN_MFP_H
#define _MHN_MFP_H

#include <linux/config.h>

typedef unsigned int mfp_pin_t;

/*
 * Although the structure below uses bit fields, it is never used
 * to map to register content; instead, macros PIN2REG/REG2PIN are
 * used. So __attribute__(packed) is not necessary here
 */
struct mhn_pin_config {
	mfp_pin_t    mfp_pin;
	unsigned int reserved:16;
	unsigned int af_sel:3;
	unsigned int edge_rise_en:1;
	unsigned int edge_fall_en:1;
	unsigned int edge_clear:1;
	unsigned int sleep_oe_n:1;
	unsigned int sleep_data:1;
	unsigned int sleep_sel:1;
	unsigned int drive:3;
	unsigned int pulldown_en:1;
	unsigned int pullup_en:1;
	unsigned int pull_sel:1;
};

/*
 Table that determines the low power modes outputs, with actual settings
 used in parentheses for don't-care values.  Except for the float output,
 the configured driven and pulled levels match.  So if there is a need
 for non-LPM pulled output, the same configuration could probably be
 used.

 Output value    sleep_oe_n  sleep_data  pullup_en   pulldown_en pull_sel
                  (bit 7)     (bit 8)     (bit 14d)   (bit 13d)

 Drive 0             0          0           0           X (1)      0
 Drive 1             0          1           X (1)       0	   0
 Pull hi (1)         1          X(1)        1           0	   0
 Pull lo (0)         1          X(0)        0           1	   0
 Z (float)           1          X(0)        0           0	   0
 */
#define MFP_LPM_DRIVE_LOW	0x8
#define MFP_LPM_DRIVE_HIGH    	0x6
#define MFP_LPM_PULL_HIGH     	0x7
#define MFP_LPM_PULL_LOW      	0x9
#define MFP_LPM_FLOAT         	0x1
#define MFP_LPM_PULL_NEITHER	0x0

/*
 * The pullup and pulldown state of the MFP pin is by default determined by
 * selected alternate function. In case some buggy devices need to override
 * this default behavior, mhn_mfp_set_pull() can be invoked with one of the
 * following definition as the parameter.
 *
 * Definition       pull_sel  pullup_en  pulldown_en
 * MFP_PULL_HIGH        1         1        0
 * MFP_PULL_LOW         1         0        1
 * MFP_PULL_BOTH        1         1        1
 * MFP_PULL_NONE        1         0        0
 * MFP_PULL_DEFAULT     0         X        X
 *
 * NOTE: the function mhn_mfp_set_pull() will modify the bits of PULLUP_EN
 * and PULLDOWN_EN, which will cause potential conflicts with the low power
 * mode setting. Device drivers should take care of such conflicts, restore
 * the low power mode setting before entering low power mode if possible.
 */
#define MFP_PULL_BOTH		(0x7u)
#define MFP_PULL_HIGH		(0x6u)
#define MFP_PULL_LOW		(0x5u)
#define MFP_PULL_NONE		(0x4u)
#define MFP_PULL_DEFAULT	(0x0u)

#define MFP_AF0			(0)
#define MFP_AF1			(1)
#define MFP_AF2			(2)
#define MFP_AF3			(3)
#define MFP_AF4			(4)
#define MFP_AF5			(5)
#define MFP_AF6			(6)
#define MFP_AF7			(7)
#define MFP_DS01X		(0)
#define MFP_DS02X		(1)
#define MFP_DS03X		(2)
#define MFP_DS04X		(3)
#define MFP_DS06X		(4)
#define MFP_DS08X		(5)
#define MFP_DS10X		(6)
#define MFP_DS12X		(7)

#define MFP_EDGE_BOTH		0x3
#define MFP_EDGE_RISE		0x2
#define MFP_EDGE_FALL		0x1
#define MFP_EDGE_NONE		0x0

#define MFP_AF_MASK		0x0007
#define MFP_DRV_MASK		0x1c00
#define MFP_RDH_MASK		0x0200
#define MFP_LPM_MASK		0xe180
#define MFP_PULL_MASK		0xe000
#define MFP_EDGE_MASK		0x0070

#define MHN_MFP_CFG(desc, pin, af, drv, rdh, lpm, edge) \
{							\
	.mfp_pin = pin,					\
	.af_sel	 = af,					\
	.reserved	= 0,				\
	.drive		= drv,				\
	.sleep_sel	= rdh,				\
	.sleep_oe_n	= ((lpm) & 0x1),		\
	.sleep_data	= (((lpm) & 0x2)  >>1),		\
	.pullup_en	= (((lpm) & 0x4)  >>2),		\
	.pulldown_en	= (((lpm) & 0x8)  >>3),		\
	.pull_sel	= (((lpm) & 0x10) >>4),		\
	.edge_clear	= (!(edge)),			\
	.edge_fall_en	= ((edge) & 0x1),		\
	.edge_rise_en	= (((edge) & 0x2) >>1),		\
}

#define MFP_OFFSET(pin)	(((pin) >> 16) & 0xffffU)
#define MFP_REG(pin)	__REG(0x40E10000 + MFP_OFFSET(pin))

#define MFPR_ALT_OFFSET		0
#define MFPR_ERE_OFFSET		4
#define MFPR_EFE_OFFSET		5
#define MFPR_EC_OFFSET		6
#define MFPR_SON_OFFSET		7
#define MFPR_SD_OFFSET		8
#define MFPR_SS_OFFSET		9
#define MFPR_DRV_OFFSET		10
#define MFPR_PD_OFFSET		13
#define MFPR_PU_OFFSET		14
#define MFPR_PS_OFFSET		15

#define PIN2REG(pin_config)					\
		(pin_config->af_sel << MFPR_ALT_OFFSET) | 	\
                (pin_config->edge_rise_en << MFPR_ERE_OFFSET ) |\
                (pin_config->edge_fall_en << MFPR_EFE_OFFSET ) |\
                (pin_config->edge_clear << MFPR_EC_OFFSET ) |	\
                (pin_config->sleep_oe_n << MFPR_SON_OFFSET ) |	\
                (pin_config->sleep_data << MFPR_SD_OFFSET ) |	\
                (pin_config->sleep_sel << MFPR_SS_OFFSET ) |	\
                (pin_config->drive << MFPR_DRV_OFFSET ) |	\
                (pin_config->pulldown_en << MFPR_PD_OFFSET ) |	\
                (pin_config->pullup_en << MFPR_PU_OFFSET ) |	\
                (pin_config->pull_sel << MFPR_PS_OFFSET );


#if defined(CONFIG_PXA3xx) && !defined(CONFIG_PXA310)
#define	MFP_PIN_GPIO0		((0x00B4 << 16) | (0))
#define	MFP_PIN_GPIO1		((0x00B8 << 16) | (1))
#define	MFP_PIN_GPIO2		((0x00BC << 16) | (2))
#define	MFP_PIN_GPIO3		((0x027C << 16) | (3))
#define	MFP_PIN_GPIO4		((0x0280 << 16) | (4))
#define MFP_PIN_DF_SCLK_E_MFPR	((0x0250 << 16) | (0xff))
#define	MFP_PIN_nBE0		((0x0204 << 16) | (0xff))
#define	MFP_PIN_nBE1		((0x0208 << 16) | (0xff))
#define	MFP_PIN_DF_ALE_nWE	((0x020C << 16) | (0xff))
#define	MFP_PIN_DF_INT_RnB	((0x00C8 << 16) | (0xff))
#define	MFP_PIN_DF_nCS0		((0x0248 << 16) | (0xff))
#define	MFP_PIN_DF_nCS1		((0x0278 << 16) | (0xff))
#define	MFP_PIN_DF_nWE		((0x00CC << 16) | (0xff))
#define	MFP_PIN_DF_nRE		((0x0200 << 16) | (0xff))
#define	MFP_PIN_nLUA		((0x0244 << 16) | (0xff))
#define	MFP_PIN_nLLA		((0x0254 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR0	((0x0210 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR1	((0x0214 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR2	((0x0218 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR3	((0x021C << 16) | (0xff))
#define MFP_PIN_CLE_nOE		((0x0240 << 16) | (0xff))
#define	MFP_PIN_DF_IO0		((0x0220 << 16) | (0xff))
#define	MFP_PIN_DF_IO8		((0x0224 << 16) | (0xff))
#define	MFP_PIN_DF_IO1		((0x0228 << 16) | (0xff))
#define	MFP_PIN_DF_IO9		((0x022C << 16) | (0xff))
#define	MFP_PIN_DF_IO2		((0x0230 << 16) | (0xff))
#define	MFP_PIN_DF_IO10		((0x0234 << 16) | (0xff))
#define	MFP_PIN_DF_IO3		((0x0238 << 16) | (0xff))
#define	MFP_PIN_DF_IO11		((0x023C << 16) | (0xff))
#define	MFP_PIN_DF_IO4		((0x0258 << 16) | (0xff))
#define	MFP_PIN_DF_IO12		((0x025C << 16) | (0xff))
#define	MFP_PIN_DF_IO5		((0x0260 << 16) | (0xff))
#define	MFP_PIN_DF_IO13		((0x0264 << 16) | (0xff))
#define	MFP_PIN_DF_IO6		((0x0268 << 16) | (0xff))
#define	MFP_PIN_DF_IO14		((0x026C << 16) | (0xff))
#define	MFP_PIN_DF_IO7		((0x0270 << 16) | (0xff))
#define	MFP_PIN_DF_IO15		((0x0274 << 16) | (0xff))
#define	MFP_PIN_GPIO5		((0x0284 << 16) | (5))
#define	MFP_PIN_GPIO6		((0x0288 << 16) | (6))
#define	MFP_PIN_GPIO7		((0x028C << 16) | (7))
#define	MFP_PIN_GPIO8		((0x0290 << 16) | (8))
#define	MFP_PIN_GPIO9		((0x0294 << 16) | (9))
#define	MFP_PIN_GPIO10		((0x0298 << 16) | (9))
#define	MFP_PIN_GPIO11		((0x029C << 16) | (11))
#define	MFP_PIN_GPIO12		((0x02A0 << 16) | (12))
#define	MFP_PIN_GPIO13		((0x02A4 << 16) | (13))
#define	MFP_PIN_GPIO14		((0x02A8 << 16) | (14))
#define	MFP_PIN_GPIO15		((0x02AC << 16) | (15))
#define	MFP_PIN_GPIO16		((0x02B0 << 16) | (16))
#define	MFP_PIN_GPIO17		((0x02B4 << 16) | (17))
#define	MFP_PIN_GPIO18		((0x02B8 << 16) | (18))
#define	MFP_PIN_GPIO19		((0x02BC << 16) | (19))
#define	MFP_PIN_GPIO20		((0x02C0 << 16) | (20))
#define	MFP_PIN_GPIO21		((0x02C4 << 16) | (21))
#define	MFP_PIN_GPIO22		((0x02C8 << 16) | (22))
#define	MFP_PIN_GPIO23		((0x02CC << 16) | (23))
#define	MFP_PIN_GPIO24		((0x02D0 << 16) | (24))
#define	MFP_PIN_GPIO25		((0x02D4 << 16) | (25))
#define	MFP_PIN_GPIO26		((0x02D8 << 16) | (26))
#define	MFP_PIN_GPIO27		((0x0400 << 16) | (27))
#define	MFP_PIN_GPIO28		((0x0404 << 16) | (28))
#define	MFP_PIN_GPIO29		((0x0408 << 16) | (29))
#define	MFP_PIN_GPIO30		((0x040C << 16) | (30))
#define	MFP_PIN_GPIO31		((0x0410 << 16) | (31))
#define	MFP_PIN_GPIO32		((0x0414 << 16) | (32))
#define	MFP_PIN_GPIO33		((0x0418 << 16) | (33))
#define	MFP_PIN_GPIO34		((0x041C << 16) | (34))
#define	MFP_PIN_GPIO35		((0x0420 << 16) | (35))
#define	MFP_PIN_GPIO36		((0x0424 << 16) | (36))
#define	MFP_PIN_GPIO37		((0x0428 << 16) | (37))
#define	MFP_PIN_GPIO38		((0x042C << 16) | (38))
#define	MFP_PIN_GPIO39		((0x0430 << 16) | (39))
#define	MFP_PIN_GPIO40		((0x0434 << 16) | (40))
#define	MFP_PIN_GPIO41		((0x0438 << 16) | (41))
#define	MFP_PIN_GPIO42		((0x043C << 16) | (42))
#define	MFP_PIN_GPIO43		((0x0440 << 16) | (43))
#define	MFP_PIN_GPIO44		((0x0444 << 16) | (44))
#define	MFP_PIN_GPIO45		((0x0448 << 16) | (45))
#define	MFP_PIN_GPIO46		((0x044C << 16) | (46))
#define	MFP_PIN_GPIO47		((0x0450 << 16) | (47))
#define	MFP_PIN_GPIO48		((0x0454 << 16) | (48))
#define	MFP_PIN_GPIO49		((0x0458 << 16) | (49))
#define	MFP_PIN_GPIO50		((0x045C << 16) | (50))
#define	MFP_PIN_GPIO51		((0x0460 << 16) | (51))
#define	MFP_PIN_GPIO52		((0x0464 << 16) | (52))
#define	MFP_PIN_GPIO53		((0x0468 << 16) | (53))
#define	MFP_PIN_GPIO54		((0x046C << 16) | (54))
#define	MFP_PIN_GPIO55		((0x0470 << 16) | (55))
#define	MFP_PIN_GPIO56		((0x0474 << 16) | (56))
#define	MFP_PIN_GPIO57		((0x0478 << 16) | (57))
#define	MFP_PIN_GPIO58		((0x047C << 16) | (58))
#define	MFP_PIN_GPIO59		((0x0480 << 16) | (59))
#define	MFP_PIN_GPIO60		((0x0484 << 16) | (60))
#define	MFP_PIN_GPIO61		((0x0488 << 16) | (61))
#define	MFP_PIN_GPIO62		((0x048C << 16) | (62))
#define	MFP_PIN_GPIO63		((0x0490 << 16) | (63))
#define	MFP_PIN_GPIO64		((0x0494 << 16) | (64))
#define	MFP_PIN_GPIO65		((0x0498 << 16) | (65))
#define	MFP_PIN_GPIO66		((0x049C << 16) | (66))
#define	MFP_PIN_GPIO67		((0x04A0 << 16) | (67))
#define	MFP_PIN_GPIO68		((0x04A4 << 16) | (68))
#define	MFP_PIN_GPIO69		((0x04A8 << 16) | (69))
#define	MFP_PIN_GPIO70		((0x04AC << 16) | (70))
#define	MFP_PIN_GPIO71		((0x04B0 << 16) | (71))
#define	MFP_PIN_GPIO72		((0x04B4 << 16) | (72))
#define	MFP_PIN_GPIO73		((0x04B8 << 16) | (73))
#define	MFP_PIN_GPIO74		((0x04BC << 16) | (74))
#define	MFP_PIN_GPIO75		((0x04C0 << 16) | (75))
#define	MFP_PIN_GPIO76		((0x04C4 << 16) | (76))
#define	MFP_PIN_GPIO77		((0x04C8 << 16) | (77))
#define	MFP_PIN_GPIO78		((0x04CC << 16) | (78))
#define	MFP_PIN_GPIO79		((0x04D0 << 16) | (79))
#define	MFP_PIN_GPIO80		((0x04D4 << 16) | (80))
#define	MFP_PIN_GPIO81		((0x04D8 << 16) | (81))
#define	MFP_PIN_GPIO82		((0x04DC << 16) | (82))
#define	MFP_PIN_GPIO83		((0x04E0 << 16) | (83))
#define	MFP_PIN_GPIO84		((0x04E4 << 16) | (84))
#define	MFP_PIN_GPIO85		((0x04E8 << 16) | (85))
#define	MFP_PIN_GPIO86		((0x04EC << 16) | (86))
#define	MFP_PIN_GPIO87		((0x04F0 << 16) | (87))
#define	MFP_PIN_GPIO88		((0x04F4 << 16) | (88))
#define	MFP_PIN_GPIO89		((0x04F8 << 16) | (89))
#define	MFP_PIN_GPIO90		((0x04FC << 16) | (90))
#define	MFP_PIN_GPIO91		((0x0500 << 16) | (91))
#define	MFP_PIN_GPIO92		((0x0504 << 16) | (92))
#define	MFP_PIN_GPIO93		((0x0508 << 16) | (93))
#define	MFP_PIN_GPIO94		((0x050C << 16) | (94))
#define	MFP_PIN_GPIO95		((0x0510 << 16) | (95))
#define	MFP_PIN_GPIO96		((0x0514 << 16) | (96))
#define	MFP_PIN_GPIO97		((0x0518 << 16) | (97))
#define	MFP_PIN_GPIO98		((0x051C << 16) | (98))
#define	MFP_PIN_GPIO99		((0x0600 << 16) | (99))
#define	MFP_PIN_GPIO100		((0x0604 << 16) | (100))
#define	MFP_PIN_GPIO101		((0x0608 << 16) | (101))
#define	MFP_PIN_GPIO102		((0x060C << 16) | (102))
#define	MFP_PIN_GPIO103		((0x0610 << 16) | (103))
#define	MFP_PIN_GPIO104		((0x0614 << 16) | (104))
#define	MFP_PIN_GPIO105		((0x0618 << 16) | (105))
#define	MFP_PIN_GPIO106		((0x061C << 16) | (106))
#define	MFP_PIN_GPIO107		((0x0620 << 16) | (107))
#define	MFP_PIN_GPIO108		((0x0624 << 16) | (108))
#define	MFP_PIN_GPIO109		((0x0628 << 16) | (109))
#define	MFP_PIN_GPIO110		((0x062C << 16) | (110))
#define	MFP_PIN_GPIO111		((0x0630 << 16) | (111))
#define	MFP_PIN_GPIO112		((0x0634 << 16) | (112))
#define	MFP_PIN_GPIO113		((0x0638 << 16) | (113))
#define	MFP_PIN_GPIO114		((0x063C << 16) | (114))
#define	MFP_PIN_GPIO115		((0x0640 << 16) | (115))
#define	MFP_PIN_GPIO116		((0x0644 << 16) | (116))
#define	MFP_PIN_GPIO117		((0x0648 << 16) | (117))
#define	MFP_PIN_GPIO118		((0x064C << 16) | (118))
#define	MFP_PIN_GPIO119		((0x0650 << 16) | (119))
#define	MFP_PIN_GPIO120		((0x0654 << 16) | (120))
#define	MFP_PIN_GPIO121		((0x0658 << 16) | (121))
#define	MFP_PIN_GPIO122		((0x065C << 16) | (122))
#define	MFP_PIN_GPIO123		((0x0660 << 16) | (123))
#define	MFP_PIN_GPIO124		((0x0664 << 16) | (124))
#define	MFP_PIN_GPIO125		((0x0668 << 16) | (125))
#define	MFP_PIN_GPIO126		((0x066C << 16) | (126))
#define	MFP_PIN_GPIO127		((0x0670 << 16) | (127))
#define	MFP_PIN_GPIO0_2		((0x0674 << 16) | (0))
#define	MFP_PIN_GPIO1_2		((0x0678 << 16) | (1))
#define	MFP_PIN_GPIO2_2		((0x02DC << 16) | (2))
#define	MFP_PIN_GPIO3_2		((0x02E0 << 16) | (3))
#define	MFP_PIN_GPIO4_2		((0x02E4 << 16) | (4))
#define	MFP_PIN_GPIO5_2		((0x02E8 << 16) | (5))
#define	MFP_PIN_GPIO6_2		((0x02EC << 16) | (6))

#define MHN_MIN_MFP_OFFSET				(MFP_OFFSET(MFP_PIN_GPIO0))
#define MHN_MAX_MFP_OFFSET				(MFP_OFFSET(MFP_PIN_GPIO1_2))

/* Pin GPIO0 alternate function codes */
#define MFP_PIN_GPIO0_AF_GPIO_0				MFP_AF0
#define	MFP_PIN_GPIO0_AF_DF_RDY				MFP_AF1

/* Pin GPIO1 alternate function codes */
#define MFP_PIN_GPIO1_AF_GPIO_1				MFP_AF0
#define	MFP_PIN_GPIO1_AF_nCS2				MFP_AF1

/* Pin GPIO2 alternate function codes */
#define MFP_PIN_GPIO2_AF_GPIO_2				MFP_AF0
#define MFP_PIN_GPIO2_AF_nCS3				MFP_AF1
#define	MFP_PIN_GPIO2_AF_nXCVREN			MFP_AF2

/* Pin GPIO3 alternate function codes */
#define MFP_PIN_GPIO3_AF_GPIO_3				MFP_AF0
#define	MFP_PIN_GPIO3_AF_uIO_IN				MFP_AF1
#define	MFP_PIN_GPIO3_AF_KP_DKIN_6			MFP_AF2
#define	MFP_PIN_GPIO3_AF_MM1_DAT0			MFP_AF4

/* Pin GPIO4 alternate function codes */
#define MFP_PIN_GPIO4_AF_GPIO_4				MFP_AF0
#define	MFP_PIN_GPIO4_AF_uSIM_CARD_STATE		MFP_AF1
#define	MFP_PIN_GPIO4_AF_KP_DKIN_7			MFP_AF2
#define	MFP_PIN_GPIO4_AF_MM1_DAT1			MFP_AF4

/* Pin GPIO5 alternate function codes */
#define MFP_PIN_GPIO5_AF_GPIO_5				MFP_AF0
#define	MFP_PIN_GPIO5_AF_uSIM_uCLK			MFP_AF1
#define	MFP_PIN_GPIO5_AF_KP_MKIN_0			MFP_AF2
#define	MFP_PIN_GPIO5_AF_MM1_DAT2			MFP_AF4

/* Pin GPIO6 alternate function codes */
#define MFP_PIN_GPIO6_AF_GPIO_6				MFP_AF0
#define	MFP_PIN_GPIO6_AF_uSIM_uRST			MFP_AF1
#define	MFP_PIN_GPIO6_AF_KP_MKIN_1			MFP_AF2
#define	MFP_PIN_GPIO6_AF_MM1_DAT3			MFP_AF4

/* Pin GPIO7 alternate function codes */
#define MFP_PIN_GPIO7_AF_GPIO_7				MFP_AF0
#define	MFP_PIN_GPIO7_AF_KP_MKOUT_5			MFP_AF1
#define	MFP_PIN_GPIO7_AF_UART3_RXD			MFP_AF2
#define	MFP_PIN_GPIO7_AF_MM1_CLK			MFP_AF4
#define	MFP_PIN_GPIO7_AF_UART3_TXD			MFP_AF6
#define	MFP_PIN_GPIO7_AF_CLK_BYPASS_XSC			MFP_AF7

/* Pin GPIO8 alternate function codes */
#define MFP_PIN_GPIO8_AF_GPIO_8				MFP_AF0
#define	MFP_PIN_GPIO8_AF_UART3_TXD			MFP_AF2
#define	MFP_PIN_GPIO8_AF_MM1_CMD			MFP_AF4
#define	MFP_PIN_GPIO8_AF_CIR_OUT			MFP_AF5
#define	MFP_PIN_GPIO8_AF_UART3_RXD			MFP_AF6

/* Pin GPIO9 alternate function codes */
#define MFP_PIN_GPIO9_AF_GPIO_9				MFP_AF0
#define	MFP_PIN_GPIO9_AF_SCIO				MFP_AF1
#define	MFP_PIN_GPIO9_AF_KP_MKIN_6			MFP_AF3
#define	MFP_PIN_GPIO9_AF_MM2_DAT0			MFP_AF4

/* Pin GPIO10 alternate function codes */
#define MFP_PIN_GPIO10_AF_GPIO_10			MFP_AF0
#define	MFP_PIN_GPIO10_AF_SC_CARD_STATE			MFP_AF1
#define	MFP_PIN_GPIO10_AF_KP_MKIN_7			MFP_AF3
#define	MFP_PIN_GPIO10_AF_MM2_DAT1			MFP_AF4

/* Pin GPIO11 alternate function codes */
#define MFP_PIN_GPIO11_AF_GPIO_11			MFP_AF0
#define	MFP_PIN_GPIO11_AF_SC_uCLK			MFP_AF1
#define	MFP_PIN_GPIO11_AF_KP_MKOUT_5			MFP_AF3
#define	MFP_PIN_GPIO11_AF_MM2_DAT2			MFP_AF4

/* Pin GPIO12 alternate function codes */
#define MFP_PIN_GPIO12_AF_GPIO_12			MFP_AF0
#define	MFP_PIN_GPIO12_AF_SC_uRST			MFP_AF1
#define	MFP_PIN_GPIO12_AF_KP_MKOUT_6			MFP_AF3
#define	MFP_PIN_GPIO12_AF_MM2_DAT3			MFP_AF4

/* Pin GPIO13 alternate function codes */
#define MFP_PIN_GPIO13_AF_GPIO_13			MFP_AF0
#define	MFP_PIN_GPIO13_AF_KP_MKOUT_7			MFP_AF3
#define	MFP_PIN_GPIO13_AF_MM2_CLK			MFP_AF4

/* Pin GPIO14 alternate function codes */
#define MFP_PIN_GPIO14_AF_GPIO_14			MFP_AF0
#define	MFP_PIN_GPIO14_AF_MM2_CMD			MFP_AF4
#define	MFP_PIN_GPIO14_AF_MM1_CMD			MFP_AF5

/* Pin GPIO15 alternate function codes */
#define MFP_PIN_GPIO15_AF_GPIO_15			MFP_AF0
#define	MFP_PIN_GPIO15_AF_SC_UVS_0			MFP_AF1
#define	MFP_PIN_GPIO15_AF_LCD_nCS			MFP_AF2
#define	MFP_PIN_GPIO15_AF_UART2_CTS			MFP_AF3
#define	MFP_PIN_GPIO15_AF_UART2_RTS			MFP_AF4
#define	MFP_PIN_GPIO15_AF_MM1_CMD			MFP_AF5
#define	MFP_PIN_GPIO15_AF_SSP1_CLK			MFP_AF6

/* Pin GPIO16 alternate function codes */
#define MFP_PIN_GPIO16_AF_GPIO_16			MFP_AF0
#define	MFP_PIN_GPIO16_AF_uSIM_UVS_0			MFP_AF1
#define	MFP_PIN_GPIO16_AF_SSP1_FRM			MFP_AF2
#define	MFP_PIN_GPIO16_AF_CIR_OUT			MFP_AF3
#define	MFP_PIN_GPIO16_AF_UART2_RTS			MFP_AF4
#define	MFP_PIN_GPIO16_AF_UART2_CTS			MFP_AF5
#define MFP_PIN_GPIO16_AF_KP_DKIN_6			MFP_AF6

/* Pin GPIO17 alternate function codes */
#define MFP_PIN_GPIO17_AF_GPIO_17			MFP_AF0
#define	MFP_PIN_GPIO17_AF_PWM0_OUT			MFP_AF1
#define	MFP_PIN_GPIO17_AF_SSP2_FRM			MFP_AF2
#define	MFP_PIN_GPIO17_AF_AC97_SDATA_IN_2		MFP_AF3
#define	MFP_PIN_GPIO17_AF_EXT_SYNC_MVT_0		MFP_AF6

/* Pin GPIO18 alternate function codes */
#define MFP_PIN_GPIO18_AF_GPIO_18			MFP_AF0
#define	MFP_PIN_GPIO18_AF_PWM1_OUT			MFP_AF1
#define	MFP_PIN_GPIO18_AF_SSP1_RXD			MFP_AF2
#define	MFP_PIN_GPIO18_AF_AC97_SDATA_IN_3		MFP_AF3
#define	MFP_PIN_GPIO18_AF_UART2_TXD			MFP_AF4
#define	MFP_PIN_GPIO18_AF_UART2_RXD			MFP_AF5
#define	MFP_PIN_GPIO18_AF_EXT_SYNC_MVT_1		MFP_AF6
#define	MFP_PIN_GPIO18_AF_SSP1_TXD			MFP_AF7

/* Pin GPIO19 alternate function codes */
#define MFP_PIN_GPIO19_AF_GPIO_19			MFP_AF0
#define	MFP_PIN_GPIO19_AF_PWM2_OUT			MFP_AF1
#define	MFP_PIN_GPIO19_AF_SSP2_TXD			MFP_AF2
#define	MFP_PIN_GPIO19_AF_KP_MKOUT_4			MFP_AF3
#define	MFP_PIN_GPIO19_AF_UART2_RXD			MFP_AF4
#define	MFP_PIN_GPIO19_AF_UART2_TXD			MFP_AF5
#define	MFP_PIN_GPIO19_AF_OST_CHOUT_MVT_0		MFP_AF6
#define	MFP_PIN_GPIO19_AF_SSP2_RXD			MFP_AF7

/* Pin GPIO20 alternate function codes */
#define MFP_PIN_GPIO20_AF_GPIO_20			MFP_AF0
#define	MFP_PIN_GPIO20_AF_PWM3_OUT			MFP_AF1
#define	MFP_PIN_GPIO20_AF_SSP1_TXD			MFP_AF2
#define	MFP_PIN_GPIO20_AF_KP_MKOUT_5			MFP_AF3
#define	MFP_PIN_GPIO20_AF_RTC_MVT			MFP_AF4
#define	MFP_PIN_GPIO20_AF_OW_DQ_IN			MFP_AF5
#define	MFP_PIN_GPIO20_AF_OST_CHOUT_MVT_1		MFP_AF6
#define	MFP_PIN_GPIO20_AF_SSP1_RXD			MFP_AF7

/* Pin GPIO21 alternate function codes */
#define MFP_PIN_GPIO21_AF_GPIO_21			MFP_AF0
#define	MFP_PIN_GPIO21_AF_I2C_SCL			MFP_AF1
#define	MFP_PIN_GPIO21_AF_AC97_SDATA_IN_2		MFP_AF2

/* Pin GPIO22 alternate function codes */
#define MFP_PIN_GPIO22_AF_GPIO_22			MFP_AF0
#define	MFP_PIN_GPIO22_AF_I2C_SDA			MFP_AF1
#define	MFP_PIN_GPIO22_AF_AC97_SDATA_IN_3		MFP_AF2

/* Pin GPIO23 alternate function codes */
#define MFP_PIN_GPIO23_AF_GPIO_23			MFP_AF0
#define	MFP_PIN_GPIO23_AF_AC97_RESET			MFP_AF1
#define	MFP_PIN_GPIO23_AF_SSP2_SCLK			MFP_AF2

/* Pin GPIO24 alternate function codes */
#define MFP_PIN_GPIO24_AF_GPIO_24			MFP_AF0
#define	MFP_PIN_GPIO24_AF_AC97_SYSCLK			MFP_AF1
#define	MFP_PIN_GPIO24_AF_UTM_RXVALID			MFP_AF3
#define	MFP_PIN_GPIO24_AF_SSP2_RXD			MFP_AF4
#define	MFP_PIN_GPIO24_AF_SSP2_TXD			MFP_AF5

/* Pin GPIO25 alternate function codes */
#define MFP_PIN_GPIO25_AF_GPIO_25			MFP_AF0
#define	MFP_PIN_GPIO25_AF_AC97_SDATA_IN_0		MFP_AF1
#define	MFP_PIN_GPIO25_AF_SSP2_SCLK			MFP_AF2
#define	MFP_PIN_GPIO25_AF_UTM_RXACTIVE			MFP_AF3

/* Pin GPIO26 alternate function codes */
#define MFP_PIN_GPIO26_AF_GPIO_26			MFP_AF0
#define	MFP_PIN_GPIO26_AF_AC97_SDATA_IN_1		MFP_AF1
#define	MFP_PIN_GPIO26_AF_SSP2_FRM			MFP_AF2
#define	MFP_PIN_GPIO26_AF_U2D_RXERROR			MFP_AF3

/* Pin GPIO27 alternate function codes */
#define MFP_PIN_GPIO27_AF_GPIO_27			MFP_AF0
#define	MFP_PIN_GPIO27_AF_AC97_SDATA_OUT		MFP_AF1
#define	MFP_PIN_GPIO27_AF_SSP2_TXD			MFP_AF2
#define	MFP_PIN_GPIO27_AF_U2D_OPMODE_0			MFP_AF3
#define	MFP_PIN_GPIO27_AF_U2D_OPMODE_PF_0		MFP_AF4
#define	MFP_PIN_GPIO27_AF_SSP2_RXD			MFP_AF5

/* Pin GPIO28 alternate function codes */
#define MFP_PIN_GPIO28_AF_GPIO_28			MFP_AF0
#define	MFP_PIN_GPIO28_AF_AC97_SYNC			MFP_AF1
#define	MFP_PIN_GPIO28_AF_SSP2_RXD			MFP_AF2
#define	MFP_PIN_GPIO28_AF_U2D_OPMODE_1			MFP_AF3
#define	MFP_PIN_GPIO28_AF_U2D_OPMODE_PF_1		MFP_AF4
#define	MFP_PIN_GPIO28_AF_SSP2_TXD			MFP_AF5

/* Pin GPIO29 alternate function codes */
#define MFP_PIN_GPIO29_AF_GPIO_29			MFP_AF0
#define	MFP_PIN_GPIO29_AF_AC97_BITCLK			MFP_AF1
#define	MFP_PIN_GPIO29_AF_SSP2_EXTCLK			MFP_AF2
#define	MFP_PIN_GPIO29_AF_U2D_TXVALID			MFP_AF3

/* Pin GPIO30 alternate function codes */
#define MFP_PIN_GPIO30_AF_GPIO_30			MFP_AF0
#define	MFP_PIN_GPIO30_AF_U2D_PHYDATA_0			MFP_AF1
#define	MFP_PIN_GPIO30_AF_UART1_RXD			MFP_AF2
#define	MFP_PIN_GPIO30_AF_UTM_PHYDATA_OUT_0		MFP_AF3
#define	MFP_PIN_GPIO30_AF_UART1_TXD			MFP_AF4

/* Pin GPIO31 alternate function codes */
#define MFP_PIN_GPIO31_AF_GPIO_31			MFP_AF0
#define	MFP_PIN_GPIO31_AF_U2D_PHYDATA_1			MFP_AF1
#define	MFP_PIN_GPIO31_AF_UART1_TXD			MFP_AF2
#define	MFP_PIN_GPIO31_AF_UTM_PHYDATA_OUT_1		MFP_AF3
#define	MFP_PIN_GPIO31_AF_UART1_RXD			MFP_AF4

/* Pin GPIO32 alternate function codes */
#define MFP_PIN_GPIO32_AF_GPIO_32			MFP_AF0
#define	MFP_PIN_GPIO32_AF_U2D_PHYDATA_2			MFP_AF1
#define	MFP_PIN_GPIO32_AF_UART1_CTS			MFP_AF2
#define	MFP_PIN_GPIO32_AF_UTM_PHYDATA_OUT_2		MFP_AF3
#define	MFP_PIN_GPIO32_AF_UART1_RTS			MFP_AF4

/* Pin GPIO33 alternate function codes */
#define MFP_PIN_GPIO33_AF_GPIO_33			MFP_AF0
#define	MFP_PIN_GPIO33_AF_U2D_PHYDATA_3			MFP_AF1
#define	MFP_PIN_GPIO33_AF_UART1_DCD			MFP_AF2
#define	MFP_PIN_GPIO33_AF_UTM_PHYDATA_OUT_3		MFP_AF3
#define	MFP_PIN_GPIO33_AF_SSP1_SCLK			MFP_AF5
#define	MFP_PIN_GPIO33_AF_SSP2_SCLK			MFP_AF6

/* Pin GPIO34 alternate function codes */
#define MFP_PIN_GPIO34_AF_GPIO_34			MFP_AF0
#define	MFP_PIN_GPIO34_AF_U2D_PHYDATA_4			MFP_AF1
#define	MFP_PIN_GPIO34_AF_UART1_DSR			MFP_AF2
#define	MFP_PIN_GPIO34_AF_UTM_PHYDATA_OUT_4		MFP_AF3
#define	MFP_PIN_GPIO34_AF_UART1_DTR			MFP_AF4
#define	MFP_PIN_GPIO34_AF_SSP1_FRM			MFP_AF5
#define	MFP_PIN_GPIO34_AF_SSP2_FRM			MFP_AF6

/* Pin GPIO35 alternate function codes */
#define MFP_PIN_GPIO35_AF_GPIO_35			MFP_AF0
#define	MFP_PIN_GPIO35_AF_U2D_PHYDATA_5			MFP_AF1
#define	MFP_PIN_GPIO35_AF_UART1_RI			MFP_AF2
#define	MFP_PIN_GPIO35_AF_UTM_PHYDATA_OUT_5		MFP_AF3
#define	MFP_PIN_GPIO35_AF_SSP1_RXD			MFP_AF4
#define	MFP_PIN_GPIO35_AF_SSP1_TXD			MFP_AF5
#define	MFP_PIN_GPIO35_AF_SSP2_RXD			MFP_AF6
#define	MFP_PIN_GPIO35_AF_SSP2_TXD			MFP_AF7

/* Pin GPIO36 alternate function codes */
#define MFP_PIN_GPIO36_AF_GPIO_36			MFP_AF0
#define	MFP_PIN_GPIO36_AF_U2D_PHYDATA_6			MFP_AF1
#define	MFP_PIN_GPIO36_AF_UART1_DTR			MFP_AF2
#define	MFP_PIN_GPIO36_AF_UTM_PHYDATA_OUT_6		MFP_AF3
#define	MFP_PIN_GPIO36_AF_UART1_DSR			MFP_AF4
#define	MFP_PIN_GPIO36_AF_SSP1_TXD			MFP_AF5
#define	MFP_PIN_GPIO36_AF_SSP1_RXD			MFP_AF6
#define	MFP_PIN_GPIO36_AF_SSP2_TXD			MFP_AF7

/* Pin GPIO37 alternate function codes */
#define MFP_PIN_GPIO37_AF_GPIO_37			MFP_AF0
#define	MFP_PIN_GPIO37_AF_U2D_PHYDATA_7			MFP_AF1
#define	MFP_PIN_GPIO37_AF_UART1_RTS			MFP_AF2
#define	MFP_PIN_GPIO37_AF_UTM_PHYDATA_OUT_6		MFP_AF3
#define	MFP_PIN_GPIO37_AF_UART1_CTS			MFP_AF4

/* Pin GPIO38 alternate function codes */
#define MFP_PIN_GPIO38_AF_GPIO_38			MFP_AF0
#define	MFP_PIN_GPIO38_AF_UTM_CLK			MFP_AF1
#define	MFP_PIN_GPIO38_AF_KP_MKOUT_5			MFP_AF5

/* Pin GPIO39 alternate function codes */
#define MFP_PIN_GPIO39_AF_GPIO_39			MFP_AF0
#define	MFP_PIN_GPIO39_AF_CI_DD_0			MFP_AF1
#define	MFP_PIN_GPIO39_AF_U2D_PHYDATA_0			MFP_AF2
#define	MFP_PIN_GPIO39_AF_UTM_PHYDATA_OUT_0		MFP_AF3

/* Pin GPIO40 alternate function codes */
#define MFP_PIN_GPIO40_AF_GPIO_40			MFP_AF0
#define	MFP_PIN_GPIO40_AF_CI_DD_1			MFP_AF1
#define	MFP_PIN_GPIO40_AF_U2D_PHYDATA_1			MFP_AF2
#define	MFP_PIN_GPIO40_AF_UTM_PHYDATA_OUT_1		MFP_AF3

/* Pin GPIO41 alternate function codes */
#define MFP_PIN_GPIO41_AF_GPIO_41			MFP_AF0
#define	MFP_PIN_GPIO41_AF_CI_DD_2			MFP_AF1
#define	MFP_PIN_GPIO41_AF_U2D_PHYDATA_2			MFP_AF2
#define	MFP_PIN_GPIO41_AF_UTM_PHYDATA_OUT_2		MFP_AF3

/* Pin GPIO42 alternate function codes */
#define MFP_PIN_GPIO42_AF_GPIO_42			MFP_AF0
#define MFP_PIN_GPIO42_AF_CI_DD_3			MFP_AF1
#define MFP_PIN_GPIO42_AF_U2D_PHYDATA_3			MFP_AF2
#define MFP_PIN_GPIO42_AF_UTM_PHYDATA_OUT_3		MFP_AF3

/* Pin GPIO43 alternate function codes */
#define MFP_PIN_GPIO43_AF_GPIO_43			MFP_AF0
#define MFP_PIN_GPIO43_AF_CI_DD_4			MFP_AF1
#define MFP_PIN_GPIO43_AF_U2D_PHYDATA_4			MFP_AF2
#define MFP_PIN_GPIO43_AF_UTM_PHYDATA_OUT_4		MFP_AF3

/* Pin GPIO44 alternate function codes */
#define MFP_PIN_GPIO44_AF_GPIO_44			MFP_AF0
#define MFP_PIN_GPIO44_AF_CI_DD_5			MFP_AF1
#define MFP_PIN_GPIO44_AF_U2D_PHYDATA_5			MFP_AF2
#define MFP_PIN_GPIO44_AF_UTM_PHYDATA_OUT_5		MFP_AF3

/* Pin GPIO45 alternate function codes */
#define MFP_PIN_GPIO45_AF_GPIO_45			MFP_AF0
#define MFP_PIN_GPIO45_AF_CI_DD_6			MFP_AF1
#define MFP_PIN_GPIO45_AF_U2D_PHYDATA_6			MFP_AF2
#define MFP_PIN_GPIO45_AF_UTM_PHYDATA_OUT_6		MFP_AF3

/* Pin GPIO46 alternate function codes */
#define MFP_PIN_GPIO46_AF_CI_DD_7			MFP_AF0
#define MFP_PIN_GPIO46_AF_GPIO_46			MFP_AF1
#define MFP_PIN_GPIO46_AF_U2D_PHYDATA_7			MFP_AF2
#define MFP_PIN_GPIO46_AF_UTM_PHYDATA_OUT_7		MFP_AF3

/* Pin GPIO47 alternate function codes */
#define MFP_PIN_GPIO47_AF_GPIO_47			MFP_AF0
#define MFP_PIN_GPIO47_AF_CI_DD_8			MFP_AF1
#define MFP_PIN_GPIO47_AF_UTM_RXACTIVE			MFP_AF2

/* Pin GPIO48 alternate function codes */
#define MFP_PIN_GPIO48_AF_GPIO_48			MFP_AF0
#define MFP_PIN_GPIO48_AF_CI_DD_9			MFP_AF1
#define MFP_PIN_GPIO48_AF_UTM_RXVALID			MFP_AF2

/* Pin GPIO49 alternate function codes */
#define MFP_PIN_GPIO49_AF_CI_MCLK			MFP_AF0
#define MFP_PIN_GPIO49_AF_UTM_RXACTIVE			MFP_AF1
#define MFP_PIN_GPIO49_AF_48M_CLK			MFP_AF2
#define MFP_PIN_GPIO49_AF_GPIO_49			MFP_AF3

/* Pin GPIO50 alternate function codes */
#define MFP_PIN_GPIO50_AF_CI_PCLK			MFP_AF0
#define MFP_PIN_GPIO50_AF_UTM_RXERROR			MFP_AF1
#define MFP_PIN_GPIO50_AF_GPIO_50			MFP_AF2

/* Pin GPIO51 alternate function codes */
#define MFP_PIN_GPIO51_AF_CI_LV				MFP_AF0
#define MFP_PIN_GPIO51_AF_UTM_OPMODE0			MFP_AF1
#define MFP_PIN_GPIO51_AF_UTM_OPMODE_PF_0		MFP_AF2
#define MFP_PIN_GPIO51_AF_GPIO_51			MFP_AF3

/* Pin GPIO52 alternate function codes */
#define MFP_PIN_GPIO52_AF_CI_FV				MFP_AF0
#define MFP_PIN_GPIO52_AF_UTM_OPMODE1			MFP_AF1
#define MFP_PIN_GPIO52_AF_UTM_OPMODE_PF_1		MFP_AF2
#define MFP_PIN_GPIO52_AF_GPIO_52			MFP_AF3
#define MFP_PIN_GPIO52_AF_TXVALID			MFP_AF4

/* Pin GPIO53 alternate function codes */
#define MFP_PIN_GPIO53_AF_GPIO_53			MFP_AF0
#define MFP_PIN_GPIO53_AF_UTM_TXREADY			MFP_AF1
#define MFP_PIN_GPIO53_AF_KP_MKOUT_6			MFP_AF5

/* Pin GPIO54 alternate function codes */
#define MFP_PIN_GPIO54_AF_GPIO_54			MFP_AF0
#define MFP_PIN_GPIO54_AF_LCD_LDD_0			MFP_AF1
#define MFP_PIN_GPIO54_AF_MSLCD_DATA_MVT_0		MFP_AF7

/* Pin GPIO55 alternate function codes */
#define MFP_PIN_GPIO55_AF_GPIO_55			MFP_AF0
#define MFP_PIN_GPIO55_AF_LCD_LDD_1			MFP_AF1
#define MFP_PIN_GPIO55_AF_MSLCD_DATA_MVT_1		MFP_AF7

/* Pin GPIO56 alternate function codes */
#define MFP_PIN_GPIO56_AF_GPIO_56			MFP_AF0
#define MFP_PIN_GPIO56_AF_LCD_LDD_2			MFP_AF1
#define MFP_PIN_GPIO56_AF_MSLCD_DATA_MVT_2		MFP_AF7

/* Pin GPIO57 alternate function codes */
#define MFP_PIN_GPIO57_AF_GPIO_57			MFP_AF0
#define MFP_PIN_GPIO57_AF_LCD_LDD_3			MFP_AF1
#define MFP_PIN_GPIO57_AF_MSLCD_DATA_MVT_3		MFP_AF7

/* Pin GPIO58 alternate function codes */
#define MFP_PIN_GPIO58_AF_GPIO_58			MFP_AF0
#define MFP_PIN_GPIO58_AF_LCD_LDD_4			MFP_AF1
#define MFP_PIN_GPIO58_AF_MSLCD_DATA_MVT_4		MFP_AF7

/* Pin GPIO59 alternate function codes */
#define MFP_PIN_GPIO59_AF_GPIO_59			MFP_AF0
#define MFP_PIN_GPIO59_AF_LCD_LDD_5			MFP_AF1
#define MFP_PIN_GPIO59_AF_MSLCD_DATA_MVT_5		MFP_AF7

/* Pin GPIO60 alternate function codes */
#define MFP_PIN_GPIO60_AF_GPIO_60			MFP_AF0
#define MFP_PIN_GPIO60_AF_LCD_LDD_6			MFP_AF1
#define MFP_PIN_GPIO60_AF_MSLCD_DATA_MVT_6		MFP_AF7

/* Pin GPIO61 alternate function codes */
#define MFP_PIN_GPIO61_AF_GPIO_61			MFP_AF0
#define MFP_PIN_GPIO61_AF_LCD_LDD_7			MFP_AF1
#define MFP_PIN_GPIO61_AF_MSLCD_DATA_MVT_7		MFP_AF7

/* Pin GPIO62 alternate function codes */
#define MFP_PIN_GPIO62_AF_GPIO_62			MFP_AF0
#define MFP_PIN_GPIO62_AF_LCD_LDD_8			MFP_AF1
#define MFP_PIN_GPIO62_AF_LCD_CS_N			MFP_AF2
#define MFP_PIN_GPIO62_AF_MSLCD_DATA_MVT_8		MFP_AF7

/* Pin GPIO63 alternate function codes */
#define MFP_PIN_GPIO63_AF_GPIO_63			MFP_AF0
#define MFP_PIN_GPIO63_AF_LCD_LDD_9			MFP_AF1
#define MFP_PIN_GPIO63_AF_LCD_VSYNC			MFP_AF2
#define MFP_PIN_GPIO63_AF_MSLCD_DATA_MVT_9		MFP_AF7

/* Pin GPIO64 alternate function codes */
#define MFP_PIN_GPIO64_AF_GPIO_64			MFP_AF0
#define MFP_PIN_GPIO64_AF_LCD_LDD_10			MFP_AF1
#define MFP_PIN_GPIO64_AF_SSP2_SCLK			MFP_AF2
#define MFP_PIN_GPIO64_AF_U2D_XCVR			MFP_AF3
#define MFP_PIN_GPIO64_AF_U2D_XCVR_PF			MFP_AF5
#define MFP_PIN_GPIO64_AF_MSLCD_DATA_MVT_10		MFP_AF7

/* Pin GPIO65 alternate function codes */
#define MFP_PIN_GPIO65_AF_GPIO_65			MFP_AF0
#define MFP_PIN_GPIO65_AF_LCD_LDD_11			MFP_AF1
#define MFP_PIN_GPIO65_AF_SSP2_FRM			MFP_AF2
#define MFP_PIN_GPIO65_AF_U2D_TERM			MFP_AF3
#define MFP_PIN_GPIO65_AF_U2D_TERM_PF			MFP_AF5
#define MFP_PIN_GPIO65_AF_MSLCD_DATA_MVT_11		MFP_AF7

/* Pin GPIO66 alternate function codes */
#define MFP_PIN_GPIO66_AF_GPIO_66			MFP_AF0
#define MFP_PIN_GPIO66_AF_LCD_LDD_12			MFP_AF1
#define MFP_PIN_GPIO66_AF_SSP2_RXD			MFP_AF2
#define MFP_PIN_GPIO66_AF_U2D_SUSPEND			MFP_AF3
#define MFP_PIN_GPIO66_AF_SSP2_TXD			MFP_AF4
#define MFP_PIN_GPIO66_AF_MSLCD_DATA_MVT_12		MFP_AF7

/* Pin GPIO67 alternate function codes */
#define MFP_PIN_GPIO67_AF_GPIO_67			MFP_AF0
#define MFP_PIN_GPIO67_AF_LCD_LDD_13			MFP_AF1
#define MFP_PIN_GPIO67_AF_SSP2_TXD			MFP_AF2
#define MFP_PIN_GPIO67_AF_UTM_LINESTATE_0		MFP_AF3
#define MFP_PIN_GPIO67_AF_SSP2_RXD			MFP_AF4
#define MFP_PIN_GPIO67_AF_MSLCD_DATA_MVT_13		MFP_AF7

/* Pin GPIO68 alternate function codes */
#define MFP_PIN_GPIO68_AF_GPIO_68			MFP_AF0
#define MFP_PIN_GPIO68_AF_LCD_LDD_14			MFP_AF1
#define MFP_PIN_GPIO68_AF_SSP3_SCLK			MFP_AF2
#define MFP_PIN_GPIO68_AF_UTM_LINESTATE_1		MFP_AF3
#define MFP_PIN_GPIO68_AF_MSLCD_DATA_MVT_14		MFP_AF7

/* Pin GPIO69 alternate function codes */
#define MFP_PIN_GPIO69_AF_GPIO_69			MFP_AF0
#define MFP_PIN_GPIO69_AF_LCD_LDD_15			MFP_AF1
#define MFP_PIN_GPIO69_AF_SSP3_FRM			MFP_AF2
#define MFP_PIN_GPIO69_AF_U2D_TXVALID			MFP_AF3
#define MFP_PIN_GPIO69_AF_MSLCD_DATA_MVT_15		MFP_AF7

/* Pin GPIO70 alternate function codes */
#define MFP_PIN_GPIO70_AF_GPIO_70			MFP_AF0
#define MFP_PIN_GPIO70_AF_LCD_LDD_16			MFP_AF1
#define MFP_PIN_GPIO70_AF_SSP3_TXD			MFP_AF2
#define MFP_PIN_GPIO70_KP_MKIN_6			MFP_AF3
#define MFP_PIN_GPIO70_SSP3_RXD				MFP_AF5

/* Pin GPIO71 alternate function codes */
#define MFP_PIN_GPIO71_AF_GPIO_71			MFP_AF0
#define MFP_PIN_GPIO71_AF_LCD_LDD_17			MFP_AF1
#define MFP_PIN_GPIO71_AF_SSP3_RXD			MFP_AF2
#define MFP_PIN_GPIO71_AF_KP_MKIN_7			MFP_AF3
#define MFP_PIN_GPIO71_AF_SSP3_TXD			MFP_AF5
#define MFP_PIN_GPIO71_AF_EXT_MATCH_MVT			MFP_AF6

/* Pin GPIO72 alternate function codes */
#define MFP_PIN_GPIO72_AF_GPIO_72			MFP_AF0
#define MFP_PIN_GPIO72_AF_LCD_L_FCLK			MFP_AF1
#define MFP_PIN_GPIO72_AF_MSLCD_FCLK_MVT		MFP_AF7

/* Pin GPIO73 alternate function codes */
#define MFP_PIN_GPIO73_AF_GPIO_73			MFP_AF0
#define MFP_PIN_GPIO73_AF_LCD_L_LCLK			MFP_AF1
#define MFP_PIN_GPIO73_AF_MSLCD_LCLK_MVT		MFP_AF7

/* Pin GPIO74 alternate function codes */
#define MFP_PIN_GPIO74_AF_GPIO_74			MFP_AF0
#define MFP_PIN_GPIO74_AF_LCD_L_PCLK			MFP_AF1
#define MFP_PIN_GPIO74_AF_MSLCD_PCLK_MVT		MFP_AF7

/* Pin GPIO75 alternate function codes */
#define MFP_PIN_GPIO75_AF_GPIO_75			MFP_AF0
#define MFP_PIN_GPIO75_AF_LCD_L_BIAS			MFP_AF1
#define MFP_PIN_GPIO75_AF_MSLCD_L_BIAS_MVT		MFP_AF2

/* Pin GPIO76 alternate function codes */
#define MFP_PIN_GPIO76_AF_GPIO_76			MFP_AF0
#define MFP_PIN_GPIO76_AF_U2D_RESET			MFP_AF1
#define MFP_PIN_GPIO76_AF_LCD_VSYNC			MFP_AF2

/* Pin GPIO77 alternate function codes */
#define MFP_PIN_GPIO77_AF_GPIO_77			MFP_AF0
#define MFP_PIN_GPIO77_AF_UART1_RXD			MFP_AF1
#define MFP_PIN_GPIO77_AF_USB_P3_1			MFP_AF2
#define MFP_PIN_GPIO77_AF_UART1_TXD			MFP_AF3
#define MFP_PIN_GPIO77_AF_MM2_DAT0			MFP_AF4
#define MFP_PIN_GPIO77_AF_MSL_IB_DAT0			MFP_AF5
#define MFP_PIN_GPIO77_AF_MSL_OB_DAT0			MFP_AF6

/* Pin GPIO78 alternate function codes */
#define MFP_PIN_GPIO78_AF_GPIO_78			MFP_AF0
#define MFP_PIN_GPIO78_AF_UART1_TXD			MFP_AF1
#define MFP_PIN_GPIO78_AF_USB_P3_3			MFP_AF2
#define MFP_PIN_GPIO78_AF_UART1_RXD			MFP_AF3
#define MFP_PIN_GPIO78_AF_MM2_DATA_OUT_1		MFP_AF4
#define MFP_PIN_GPIO78_AF_KP_MKOUT_7			MFP_AF5
#define MFP_PIN_GPIO78_AF_MSL_OB_CLK			MFP_AF6

/* Pin GPIO79 alternate function codes */
#define MFP_PIN_GPIO79_AF_GPIO_79			MFP_AF0
#define MFP_PIN_GPIO79_AF_UART1_CTS			MFP_AF1
#define MFP_PIN_GPIO79_AF_USB_P3_3			MFP_AF2
#define MFP_PIN_GPIO79_AF_UART1_RTS			MFP_AF3
#define MFP_PIN_GPIO79_AF_MM2_DAT2			MFP_AF4
#define MFP_PIN_GPIO79_AF_MSL_IB_STB			MFP_AF5
#define MFP_PIN_GPIO79_AF_MSL_OB_STB			MFP_AF6

/* Pin GPIO80 alternate function codes */
#define MFP_PIN_GPIO80_AF_GPIO_80			MFP_AF0
#define MFP_PIN_GPIO80_AF_UART1_DCD			MFP_AF1
#define MFP_PIN_GPIO80_AF_USB_P3_4			MFP_AF2
#define MFP_PIN_GPIO80_AF_MM2_DAT3			MFP_AF4
#define MFP_PIN_GPIO80_AF_MSL_IB_WAIT			MFP_AF5
#define MFP_PIN_GPIO80_AF_MSL_OB_WAIT			MFP_AF6

/* Pin GPIO81 alternate function codes */
#define MFP_PIN_GPIO81_AF_GPIO_81			MFP_AF0
#define MFP_PIN_GPIO81_AF_UART1_DSR			MFP_AF1
#define MFP_PIN_GPIO81_AF_USB_P3_5			MFP_AF2
#define MFP_PIN_GPIO81_AF_UART1_DTR			MFP_AF3
#define MFP_PIN_GPIO81_AF_MM2_CLK			MFP_AF4
#define MFP_PIN_GPIO81_AF_MSL_OB_DAT0			MFP_AF5
#define MFP_PIN_GPIO81_AF_MSL_IB_DAT0			MFP_AF6

/* Pin GPIO82 alternate function codes */
#define MFP_PIN_GPIO82_AF_GPIO_82			MFP_AF0
#define MFP_PIN_GPIO82_AF_UART1_RI			MFP_AF1
#define MFP_PIN_GPIO82_AF_USB_P3_6			MFP_AF2
#define MFP_PIN_GPIO82_AF_MM2_CMD			MFP_AF4
#define MFP_PIN_GPIO82_AF_MSL_OB_CLK			MFP_AF5
#define MFP_PIN_GPIO82_AF_MSL_IB_CLK			MFP_AF6

/* Pin GPIO83 alternate function codes */
#define MFP_PIN_GPIO83_AF_GPIO_83			MFP_AF0
#define MFP_PIN_GPIO83_AF_UART1_DTR			MFP_AF1
#define MFP_PIN_GPIO83_AF_UART1_DSR			MFP_AF3
#define MFP_PIN_GPIO83_AF_MSL_OB_STB			MFP_AF4
#define MFP_PIN_GPIO83_AF_KP_DKIN_2			MFP_AF5
#define MFP_PIN_GPIO83_AF_MSL_IB_STB			MFP_AF6

/* Pin GPIO84 alternate function codes */
#define MFP_PIN_GPIO84_AF_GPIO_84			MFP_AF0
#define MFP_PIN_GPIO84_AF_UART1_RTS			MFP_AF1
#define MFP_PIN_GPIO84_AF_UART_CTS			MFP_AF3
#define MFP_PIN_GPIO84_AF_MSL_OB_WAIT			MFP_AF4
#define MFP_PIN_GPIO84_AF_KP_DKIN_1			MFP_AF5
#define MFP_PIN_GPIO84_AF_MSL_IB_WAIT			MFP_AF6

/* Pin GPIO85 alternate function codes */
#define MFP_PIN_GPIO85_AF_GPIO_85			MFP_AF0
#define MFP_PIN_GPIO85_AF_SSP1_SCLK			MFP_AF1
#define MFP_PIN_GPIO85_AF_KP_MKOUT_0			MFP_AF2
#define MFP_PIN_GPIO85_AF_KP_DKIN_0			MFP_AF3
#define MFP_PIN_GPIO85_AF_MSL_IB_DAT1			MFP_AF4
#define MFP_PIN_GPIO85_AF_U2D_TXVALID			MFP_AF5
#define MFP_PIN_GPIO85_AF_MSL_OB_DAT1			MFP_AF6

/* Pin GPIO86 alternate function codes */
#define MFP_PIN_GPIO86_AF_GPIO_86			MFP_AF0
#define MFP_PIN_GPIO86_AF_SSP1_FRM			MFP_AF1
#define MFP_PIN_GPIO86_AF_KP_MKOUT_1			MFP_AF2
#define MFP_PIN_GPIO86_AF_KP_DKIN_1			MFP_AF3
#define MFP_PIN_GPIO86_AF_MSL_IB_DAT2			MFP_AF4
#define MFP_PIN_GPIO86_AF_MSL_OB_DAT2			MFP_AF6

/* Pin GPIO87 alternate function codes */
#define MFP_PIN_GPIO87_AF_GPIO_87			MFP_AF0
#define MFP_PIN_GPIO87_AF_SSP1_TXD			MFP_AF1
#define MFP_PIN_GPIO87_AF_KP_MKOUT2			MFP_AF2
#define MFP_PIN_GPIO87_AF_KP_DKIN2			MFP_AF3
#define MFP_PIN_GPIO87_AF_MSL_IB_DATA3			MFP_AF4
#define MFP_PIN_GPIO87_AF_UTM_RXVALID			MFP_AF5
#define MFP_PIN_GPIO87_AF_SSP1_RXD			MFP_AF6
#define MFP_PIN_GPIO87_AF_MSL_OB_DAT3			MFP_AF7

/* Pin GPIO88 alternate function codes */
#define MFP_PIN_GPIO88_AF_GPIO_88			MFP_AF0
#define MFP_PIN_GPIO88_AF_SSP1_RXD			MFP_AF1
#define MFP_PIN_GPIO88_AF_KP_MKOUT_3			MFP_AF2
#define MFP_PIN_GPIO88_AF_KP_DKIN_3			MFP_AF3
#define MFP_PIN_GPIO88_AF_MSL_OB_DAT1			MFP_AF4
#define MFP_PIN_GPIO88_AF_UTM_RXACTIVE			MFP_AF5
#define MFP_PIN_GPIO88_AF_SSP1_TXD			MFP_AF6
#define MFP_PIN_GPIO88_AF_MSL_IB_DAT1			MFP_AF7

/* Pin GPIO89 alternate function codes */
#define MFP_PIN_GPIO89_AF_GPIO_89			MFP_AF0
#define MFP_PIN_GPIO89_AF_SSP1_EXTCLK			MFP_AF1
#define MFP_PIN_GPIO89_AF_SC_UVS1			MFP_AF2
#define MFP_PIN_GPIO89_AF_KP_DKIN_3			MFP_AF3
#define MFP_PIN_GPIO89_AF_MSL_OB_DAT2			MFP_AF4
#define MFP_PIN_GPIO89_AF_U2D_RXERROR			MFP_AF5
#define MFP_PIN_GPIO89_AF_MSL_IB_DAT2			MFP_AF6

/* Pin GPIO90 alternate function codes */
#define MFP_PIN_GPIO90_AF_GPIO_90			MFP_AF0
#define MFP_PIN_GPIO90_AF_SSP1_SYSCLK			MFP_AF1
#define MFP_PIN_GPIO90_AF_SC_UVS2			MFP_AF2
#define MFP_PIN_GPIO90_AF_MSL_IB_DAT3			MFP_AF3
#define MFP_PIN_GPIO90_AF_MSL_OB_DAT3			MFP_AF4
#define MFP_PIN_GPIO90_AF_U2D_OPMODE_0			MFP_AF5
#define MFP_PIN_GPIO90_AF_U2D_OPMODE_PF_0		MFP_AF7

/* Pin GPIO91 alternate function codes */
#define MFP_PIN_GPIO91_AF_GPIO_91			MFP_AF0
#define MFP_PIN_GPIO91_AF_SSP3_SCLK			MFP_AF1
#define MFP_PIN_GPIO91_AF_UART3_CTS			MFP_AF2
#define MFP_PIN_GPIO91_AF_UART3_RTS			MFP_AF4

/* Pin GPIO92 alternate function codes */
#define MFP_PIN_GPIO92_AF_GPIO_92			MFP_AF0
#define MFP_PIN_GPIO92_AF_SSP3_FRM			MFP_AF1
#define MFP_PIN_GPIO92_AF_UART3_RTS			MFP_AF2
#define MFP_PIN_GPIO92_AF_UTM_LINESTATE_0		MFP_AF3
#define MFP_PIN_GPIO92_AF_UART3_CTS			MFP_AF4

/* Pin GPIO93 alternate function codes */
#define MFP_PIN_GPIO93_AF_GPIO_93			MFP_AF0
#define MFP_PIN_GPIO93_AF_SSP3_TXD			MFP_AF1
#define MFP_PIN_GPIO93_AF_UART3_TXD			MFP_AF2
#define MFP_PIN_GPIO93_AF_UTM_LINESTATE_1		MFP_AF3
#define MFP_PIN_GPIO93_AF_UART3_RXD			MFP_AF4
#define MFP_PIN_GPIO93_AF_SSP3_RXD			MFP_AF5

/* Pin GPIO94 alternate function codes */
#define MFP_PIN_GPIO94_AF_GPIO_94			MFP_AF0
#define MFP_PIN_GPIO94_AF_SSP3_RXD			MFP_AF1
#define MFP_PIN_GPIO94_AF_UART3_RXD			MFP_AF2
#define MFP_PIN_GPIO94_AF_UART3_TXD			MFP_AF4
#define MFP_PIN_GPIO94_AF_SSP3_TXD			MFP_AF5

/* Pin GPIO95 alternate function codes */
#define MFP_PIN_GPIO95_AF_GPIO_95			MFP_AF0
#define MFP_PIN_GPIO95_AF_SSP4_SCLK			MFP_AF1
#define MFP_PIN_GPIO95_AF_U2D_RESET			MFP_AF2

/* Pin GPIO96 alternate function codes */
#define MFP_PIN_GPIO96_AF_GPIO_96			MFP_AF0
#define MFP_PIN_GPIO96_AF_SSP4_FRM			MFP_AF1
#define MFP_PIN_GPIO96_AF_U2D_XCVR_SEL			MFP_AF2
#define MFP_PIN_GPIO96_AF_U2D_XCVR_SEL_PF		MFP_AF3

/* Pin GPIO97 alternate function codes */
#define MFP_PIN_GPIO97_AF_GPIO_97			MFP_AF0
#define MFP_PIN_GPIO97_AF_SSP4_TXD			MFP_AF1
#define MFP_PIN_GPIO97_AF_U2D_TERM_SEL			MFP_AF2
#define MFP_PIN_GPIO97_AF_U2D_TERM_SEL_PF		MFP_AF3
#define MFP_PIN_GPIO97_AF_SSP4_RXD			MFP_AF5

/* Pin GPIO98 alternate function codes */
#define MFP_PIN_GPIO98_AF_GPIO_98			MFP_AF0
#define MFP_PIN_GPIO98_AF_SSP4_RXD			MFP_AF1
#define MFP_PIN_GPIO98_AF_U2D_SUSPEND			MFP_AF2
#define MFP_PIN_GPIO98_AF_SSP4_TXD			MFP_AF5

/* Pin GPIO99 alternate function codes */
#define MFP_PIN_GPIO99_AF_GPIO_99			MFP_AF0
#define MFP_PIN_GPIO99_AF_UART1_RXD			MFP_AF1
#define MFP_PIN_GPIO99_AF_USB_P2_2			MFP_AF2
#define MFP_PIN_GPIO99_AF_USB_P2_5			MFP_AF3
#define MFP_PIN_GPIO99_AF_USB_P2_6			MFP_AF4
#define MFP_PIN_GPIO99_AF_U2D_TERM_SEL			MFP_AF5
#define MFP_PIN_GPIO99_AF_UART1_TXD			MFP_AF6

/* Pin GPIO100 alternate function codes */
#define MFP_PIN_GPIO100_AF_GPIO_100			MFP_AF0
#define MFP_PIN_GPIO100_AF_UART1_TXD			MFP_AF1
#define MFP_PIN_GPIO100_AF_USB_P2_6			MFP_AF2
#define MFP_PIN_GPIO100_AF_U2D_RESET			MFP_AF3
#define MFP_PIN_GPIO100_AF_USB_P2_2			MFP_AF4
#define MFP_PIN_GPIO100_AF_USB_P2_5			MFP_AF5
#define MFP_PIN_GPIO100_AF_UART1_RXD			MFP_AF6
#define MFP_PIN_GPIO100_AF_KP_MKIN_6			MFP_AF7

/* Pin GPIO101 alternate function codes */
#define MFP_PIN_GPIO101_AF_GPIO_101			MFP_AF0
#define MFP_PIN_GPIO101_AF_UART1_CTS			MFP_AF1
#define MFP_PIN_GPIO101_AF_USB_P2_1			MFP_AF2
#define MFP_PIN_GPIO101_AF_U2D_XCVR_SELECT		MFP_AF3
#define MFP_PIN_GPIO101_AF_U2D_XCVR_SELECT_PF		MFP_AF5
#define MFP_PIN_GPIO101_AF_UART1_RTS			MFP_AF6
#define MFP_PIN_GPIO101_AF_KP_MKIN_7			MFP_AF7

/* Pin GPIO102 alternate function codes */
#define MFP_PIN_GPIO102_AF_GPIO_102			MFP_AF0
#define MFP_PIN_GPIO102_AF_UART1_DCD			MFP_AF1
#define MFP_PIN_GPIO102_AF_USB_P2_4			MFP_AF2
#define MFP_PIN_GPIO102_AF_U2D_TERM_SELECT		MFP_AF3
#define MFP_PIN_GPIO102_AF_UART1_TXD			MFP_AF4
#define MFP_PIN_GPIO102_AF_U2D_TERM_SELECT_PF		MFP_AF5
#define MFP_PIN_GPIO102_AF_UART1_RXD			MFP_AF6

/* Pin GPIO103 alternate function codes */
#define MFP_PIN_GPIO103_AF_GPIO_103			MFP_AF0
#define MFP_PIN_GPIO103_AF_UART1_DSR			MFP_AF1
#define MFP_PIN_GPIO103_AF_USB_P2_8			MFP_AF2
#define MFP_PIN_GPIO103_AF_U2D_SUSPENDM_X		MFP_AF3
#define MFP_PIN_GPIO103_AF_UART1_DTR			MFP_AF6

/* Pin GPIO104 alternate function codes */
#define MFP_PIN_GPIO104_AF_GPIO_104			MFP_AF0
#define MFP_PIN_GPIO104_AF_UART_RI			MFP_AF1
#define MFP_PIN_GPIO104_AF_USB_P2_3			MFP_AF2
#define MFP_PIN_GPIO104_AF_UTM_LINESTATE_0		MFP_AF3
#define MFP_PIN_GPIO104_AF_UART1_RXD			MFP_AF4
#define MFP_PIN_GPIO104_AF_KP_MKOUT_6			MFP_AF5

/* Pin GPIO105 alternate function codes */
#define MFP_PIN_GPIO105_AF_GPIO_105			MFP_AF0
#define MFP_PIN_GPIO105_AF_UART1_DTR			MFP_AF1
#define MFP_PIN_GPIO105_AF_USB_P2_5			MFP_AF2
#define MFP_PIN_GPIO105_AF_UTM_LINESTATE_1		MFP_AF3
#define MFP_PIN_GPIO105_AF_KP_MKOUT_7			MFP_AF5
#define MFP_PIN_GPIO105_AF_UART1_DSR			MFP_AF6

/* Pin GPIO106 alternate function codes */
#define MFP_PIN_GPIO106_AF_GPIO_106			MFP_AF0
#define MFP_PIN_GPIO106_AF_UART1_RTS			MFP_AF1
#define MFP_PIN_GPIO106_AF_USB_P2_7			MFP_AF2
#define MFP_PIN_GPIO106_AF_UTM_OPMODE_1			MFP_AF3
#define MFP_PIN_GPIO106_AF_UTM_OPMODE_PF_1		MFP_AF3
#define MFP_PIN_GPIO106_AF_UART1_CTS			MFP_AF6

/* Pin GPIO107 alternate function codes */
#define MFP_PIN_GPIO107_AF_GPIO_107			MFP_AF0
#define MFP_PIN_GPIO107_AF_UART3_CTS			MFP_AF1
#define MFP_PIN_GPIO107_AF_KP_DKIN_0			MFP_AF2
#define MFP_PIN_GPIO107_AF_UART3_RTS			MFP_AF3

/* Pin GPIO108 alternate function codes */
#define MFP_PIN_GPIO108_AF_GPIO_108			MFP_AF0
#define MFP_PIN_GPIO108_AF_UART3_RTS			MFP_AF1
#define MFP_PIN_GPIO108_AF_KP_DKIN_1			MFP_AF2
#define MFP_PIN_GPIO108_AF_UART3_CTS			MFP_AF3

/* Pin GPIO109 alternate function codes */
#define MFP_PIN_GPIO109_AF_GPIO_109			MFP_AF0
#define MFP_PIN_GPIO109_AF_UART3_TXD			MFP_AF1
#define MFP_PIN_GPIO109_AF_KP_DKIN_2			MFP_AF2
#define MFP_PIN_GPIO109_AF_UART3_RXD			MFP_AF3
#define MFP_PIN_GPIO109_AF_UTM_LINESTATE_0		MFP_AF4

/* Pin GPIO110 alternate function codes */
#define MFP_PIN_GPIO110_AF_GPIO_110			MFP_AF0
#define MFP_PIN_GPIO110_AF_UART3_RXD			MFP_AF1
#define MFP_PIN_GPIO110_AF_KP_DKIN_3			MFP_AF2
#define MFP_PIN_GPIO110_AF_UART3_TXD			MFP_AF3
#define MFP_PIN_GPIO110_AF_U2D_OPMODE_1			MFP_AF4
#define MFP_PIN_GPIO110_AF_U2D_OPMODE_PF_1		MFP_AF5

/* Pin GPIO111 alternate function codes */
#define MFP_PIN_GPIO111_AF_GPIO_111			MFP_AF0
#define MFP_PIN_GPIO111_AF_UART2_RTS			MFP_AF1
#define MFP_PIN_GPIO111_AF_KP_DKIN_4			MFP_AF2
#define MFP_PIN_GPIO111_AF_UART2_CTS			MFP_AF3

/* Pin GPIO112 alternate function codes */
#define MFP_PIN_GPIO112_AF_GPIO_112			MFP_AF0
#define MFP_PIN_GPIO112_AF_UART2_RXD			MFP_AF1
#define MFP_PIN_GPIO112_AF_KP_DKIN_5			MFP_AF2
#define MFP_PIN_GPIO112_AF_UART2_TXD			MFP_AF3
#define MFP_PIN_GPIO112_AF_KP_MKIN_6			MFP_AF4

/* Pin GPIO113 alternate function codes */
#define MFP_PIN_GPIO113_AF_GPIO_113			MFP_AF0
#define MFP_PIN_GPIO113_AF_UART2_TXD			MFP_AF1
#define MFP_PIN_GPIO113_AF_KP_DKIN_6			MFP_AF2
#define MFP_PIN_GPIO113_AF_UART2_RXD			MFP_AF3
#define MFP_PIN_GPIO113_AF_KP_MKIN_7			MFP_AF4

/* Pin GPIO114 alternate function codes */
#define MFP_PIN_GPIO114_AF_GPIO_114			MFP_AF0
#define MFP_PIN_GPIO114_AF_UART2_CTS			MFP_AF1
#define MFP_PIN_GPIO114_AF_KP_DKIN_7			MFP_AF2
#define MFP_PIN_GPIO114_AF_KP_UART2_RTS			MFP_AF3

/* Pin GPIO115 alternate function codes */
#define MFP_PIN_GPIO115_AF_GPIO_115			MFP_AF0
#define MFP_PIN_GPIO115_AF_KP_MKIN_0			MFP_AF1
#define MFP_PIN_GPIO115_AF_KP_DKIN_0			MFP_AF2

/* Pin GPIO116 alternate function codes */
#define MFP_PIN_GPIO116_AF_GPIO_116			MFP_AF0
#define MFP_PIN_GPIO116_AF_KP_MKIN_1			MFP_AF1
#define MFP_PIN_GPIO116_AF_KP_DKIN_1			MFP_AF2

/* Pin GPIO117 alternate function codes */
#define MFP_PIN_GPIO117_AF_GPIO_117			MFP_AF0
#define MFP_PIN_GPIO117_AF_KP_MKIN_2			MFP_AF1
#define MFP_PIN_GPIO117_AF_KP_DKIN_2			MFP_AF2

/* Pin GPIO118 alternate function codes */
#define MFP_PIN_GPIO118_AF_GPIO_118			MFP_AF0
#define MFP_PIN_GPIO118_AF_KP_MKIN_3			MFP_AF1
#define MFP_PIN_GPIO118_AF_KP_DKIN_3			MFP_AF2

/* Pin GPIO119 alternate function codes */
#define MFP_PIN_GPIO119_AF_GPIO_119			MFP_AF0
#define MFP_PIN_GPIO119_AF_KP_MKIN_4			MFP_AF1
#define MFP_PIN_GPIO119_AF_KP_DKIN_4			MFP_AF2

/* Pin GPIO120 alternate function codes */
#define MFP_PIN_GPIO120_AF_GPIO_120			MFP_AF0
#define MFP_PIN_GPIO120_AF_KP_MKIN_5			MFP_AF1
#define MFP_PIN_GPIO120_AF_KP_DKIN_5			MFP_AF2

/* Pin GPIO121 alternate function codes */
#define MFP_PIN_GPIO121_AF_GPIO_121			MFP_AF0
#define MFP_PIN_GPIO121_AF_KP_MKOUT_0			MFP_AF1
#define MFP_PIN_GPIO121_AF_KP_DKIN_6			MFP_AF2

/* Pin GPIO122 alternate function codes */
#define MFP_PIN_GPIO122_AF_GPIO_122			MFP_AF0
#define MFP_PIN_GPIO122_AF_KP_MKOUT_1			MFP_AF1
#define MFP_PIN_GPIO122_AF_KP_DKIN_5			MFP_AF2

/* Pin GPIO123 alternate function codes */
#define MFP_PIN_GPIO123_AF_GPIO_123			MFP_AF0
#define MFP_PIN_GPIO123_AF_KP_MKOUT_2			MFP_AF1
#define MFP_PIN_GPIO123_AF_KP_DKIN_4			MFP_AF2

/* Pin GPIO124 alternate function codes */
#define MFP_PIN_GPIO124_AF_GPIO_124			MFP_AF0
#define MFP_PIN_GPIO124_AF_KP_MKOUT_3			MFP_AF1
#define MFP_PIN_GPIO124_AF_KP_DKIN_3			MFP_AF2

/* Pin GPIO125 alternate function codes */
#define MFP_PIN_GPIO125_AF_GPIO_125			MFP_AF0
#define MFP_PIN_GPIO125_AF_KP_MKOUT_4			MFP_AF1
#define MFP_PIN_GPIO125_AF_KP_MKIN_2			MFP_AF2

/* Pin GPIO126 alternate function codes */
#define MFP_PIN_GPIO126_AF_GPIO_126			MFP_AF0
#define MFP_PIN_GPIO126_AF_RTC_MVT			MFP_AF1
#define MFP_PIN_GPIO126_AF_OW_DQ			MFP_AF2
#define MFP_PIN_GPIO126_AF_EXT_CLK			MFP_AF3
#define MFP_PIN_GPIO126_AF_KP_MKOUT_7			MFP_AF4

/* Pin GPIO127 alternate function codes */
#define MFP_PIN_GPIO127_AF_GPIO_127			MFP_AF0
#define MFP_PIN_GPIO127_AF_LCD_nCS			MFP_AF1
#define MFP_PIN_GPIO127_AF_KP_DKIN_0			MFP_AF5
#define MFP_PIN_GPIO127_AF_CLK_BYPASS_GB		MFP_AF7

/* Pin GPIO0_2 alternate function codes */
#define MFP_PIN_GPIO0_2_AF_GPIO_0			MFP_AF0
#define MFP_PIN_GPIO0_2_AF_UHC_USBHPEN_MVT		MFP_AF1
#define MFP_PIN_GPIO0_2_AF_KP_DKIN_0			MFP_AF2
/*#define MFP_PIN_GPIO0_2_AF_ONE_WIRE			MFP_AF2 */

/* Pin GPIO1_2 alternate function codes */
#define MFP_PIN_GPIO1_2_AF_GPIO_0			MFP_AF0
#define MFP_PIN_GPIO1_2_AF_UHC_USBHPWR_MVT		MFP_AF1
#define MFP_PIN_GPIO1_2_AF_KP_DKIN_1			MFP_AF2

/* Pin GPIO2_2 alternate function codes */
#define MFP_PIN_GPIO2_2_AF_GPIO_2			MFP_AF0
#define MFP_PIN_GPIO2_2_AF_KP_MKIN_6			MFP_AF1
#define MFP_PIN_GPIO2_2_AF_KP_DKIN_6			MFP_AF2

/* Pin GPIO3_2 alternate function codes */
#define MFP_PIN_GPIO3_2_AF_GPIO_3			MFP_AF0
#define MFP_PIN_GPIO3_2_AF_KP_MKIN_7			MFP_AF1
#define MFP_PIN_GPIO3_2_AF_KP_DKIN_7			MFP_AF2

/* Pin GPIO4_2 alternate function codes */
#define MFP_PIN_GPIO4_2_AF_GPIO_4			MFP_AF0
#define MFP_PIN_GPIO4_2_AF_KP_MK0UT_5			MFP_AF1
#define MFP_PIN_GPIO4_2_AF_KP_DKIN_1			MFP_AF2

/* Pin GPIO5_2 alternate function codes */
#define MFP_PIN_GPIO5_2_AF_GPIO_5			MFP_AF0
#define MFP_PIN_GPIO5_2_AF_KP_MKOUT_6			MFP_AF1
#define MFP_PIN_GPIO5_2_AF_MK_DKIN_0			MFP_AF2

/* Pin GPIO6_2 alternate function codes */
#define MFP_PIN_GPIO6_2_AF_GPIO_6			MFP_AF0
#define MFP_PIN_GPIO6_2_AF_MK_MKOUT_7			MFP_AF1

/* Pin DF_CLE alternate function codes */
#define MFP_PIN_DF_CLE_AF_ND_CLE			MFP_AF0

/* Pin DF_ALE_nWE1 alternate function codes */
#define MFP_PIN_DF_ALE_nWE1_AF_CD_ADV1			MFP_AF0
#define MFP_PIN_DF_ALE_nWE1_AF_ND_ALE			MFP_AF1

/* Pin DF_SCLK_E alternate function codes */
#define MFP_PIN_DF_SCLK_E_AF_DF_SCLK_E			MFP_AF0

/* Pin DF_SCLK_S alternate function codes */
#define MFP_PIN_DF_SCLK_S_AF_DF_SCLK_S			MFP_AF0

/* Pin nBE0 alternate function codes */
#define MFP_PIN_nBE0_AF_DF_nBE0				MFP_AF0

/* Pin nBE1 alternate function codes */
#define MFP_PIN_nBE1_AF_DF_nBE1				MFP_AF0

/* Pin DF_INT_RnB alternate function codes */
#define MFP_PIN_DF_INT_RnB_AF_INT_RnB			MFP_AF0

/* Pin nLUA alternate function codes */
#define MFP_PIN_DF_nLUA_AF_DF_nLUA			MFP_AF0
#define MFP_PIN_DF_nLUA_AF_CD_ADV2			MFP_AF1

/* Pin nLLA alternate function codes */
#define MFP_PIN_DF_nLLA_AF_DF_nLLA			MFP_AF0
#define MFP_PIN_DF_nLLA_AF_CD_ADV1			MFP_AF1

/* Pin DF_nWE alternate function codes */
#define MFP_PIN_DF_nWE_AF_CD_WE				MFP_AF0
#define MFP_PIN_DF_nWE_AF_ND_WE				MFP_AF1

/* Pin DF_nRE alternate function codes */
#define MFP_PIN_DF_nRE_AF_CD_OE				MFP_AF0
#define MFP_PIN_DF_nRE_AF_ND_RE				MFP_AF1

/* Pin DF_ADDR0 alternate functino codes */
#define MFP_PIN_DF_ADDR0_AF_DF_ADDR0			MFP_AF0

/* Pin DF_ADDR1 alternate functino codes */
#define MFP_PIN_DF_ADDR0_AF_DF_ADDR1			MFP_AF0

/* Pin DF_ADDR2 alternate functino codes */
#define MFP_PIN_DF_ADDR0_AF_DF_ADDR2			MFP_AF0

/* Pin DF_ADDR3 alternate functino codes */
#define MFP_PIN_DF_ADDR0_AF_DF_ADDR3			MFP_AF0

/* Pin nCS0 alternate function codes */
#define MFP_PIN_nCS0_AF_nCS0				MFP_AF0
#define MFP_PIN_nCS0_AF_DF_XCVREN			MFP_AF1

/* Pin nCS1 alternate function codes */
#define MFP_PIN_nCS1_AF_nCS				MFP_AF0
#define MFP_PIN_nCS1_AF_DF_UNLOCK			MFP_AF1

/* Pin DF_nCS0 alternate function codes */
#define MFP_PIN_DF_nCS0_AF_DF_nCS0			MFP_AF0
#define MFP_PIN_DF_nCS0_AF_ND_nCS0			MFP_AF1

/* Pin DF_nCS1 alternate function codes */
#define MFP_PIN_DF_nCS1_AF_DF_nCS1			MFP_AF0
#define MFP_PIN_DF_nCS1_AF_ND_nCS1			MFP_AF1


/* Pin DF_IO15..0 alternate function codes */
/*   - Note that, in use, all 16 pins must have the same AF */
/*  Currently, only Alternate Function 1 is actually available */
/*  for these pins */

#define MFP_PIN_DF_IO_0_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_0_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_1_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_1_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_2_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_2_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_3_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_3_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_4_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_4_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_5_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_5_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_6_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_6_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_7_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_7_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_8_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_8_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_9_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_9_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_10_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_10_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_11_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_11_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_12_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_12_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_13_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_13_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_14_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_14_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_15_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_15_AF_ND				MFP_AF1

#else
#define	MFP_PIN_GPIO0		((0x00B4 << 16) | (0))
#define	MFP_PIN_GPIO1		((0x00B8 << 16) | (1))
#define	MFP_PIN_GPIO2		((0x00BC << 16) | (2))
#define	MFP_PIN_GPIO3		((0x027C << 16) | (3))
#define	MFP_PIN_GPIO4		((0x0280 << 16) | (4))
#define MFP_PIN_DF_SCLK_E_MFPR	((0x0250 << 16) | (0xff))
#define	MFP_PIN_nBE0		((0x0204 << 16) | (0xff))
#define	MFP_PIN_nBE1		((0x0208 << 16) | (0xff))
#define	MFP_PIN_DF_ALE_nWE	((0x020C << 16) | (0xff))
#define	MFP_PIN_DF_INT_RnB	((0x00C8 << 16) | (0xff))
#define	MFP_PIN_DF_nCS0		((0x0248 << 16) | (0xff))
#define	MFP_PIN_DF_nCS1		((0x0278 << 16) | (0xff))
#define	MFP_PIN_DF_nWE		((0x00CC << 16) | (0xff))
#define	MFP_PIN_DF_nRE		((0x0200 << 16) | (0xff))
#define	MFP_PIN_nLUA		((0x0244 << 16) | (0xff))
#define	MFP_PIN_nLLA		((0x0254 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR0	((0x0210 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR1	((0x0214 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR2	((0x0218 << 16) | (0xff))
#define	MFP_PIN_DF_ADDR3	((0x021C << 16) | (0xff))
#define MFP_PIN_CLE_nOE		((0x0240 << 16) | (0xff))
#define	MFP_PIN_DF_IO0		((0x0220 << 16) | (0xff))
#define	MFP_PIN_DF_IO8		((0x0224 << 16) | (0xff))
#define	MFP_PIN_DF_IO1		((0x0228 << 16) | (0xff))
#define	MFP_PIN_DF_IO9		((0x022C << 16) | (0xff))
#define	MFP_PIN_DF_IO2		((0x0230 << 16) | (0xff))
#define	MFP_PIN_DF_IO10		((0x0234 << 16) | (0xff))
#define	MFP_PIN_DF_IO3		((0x0238 << 16) | (0xff))
#define	MFP_PIN_DF_IO11		((0x023C << 16) | (0xff))
#define	MFP_PIN_DF_IO4		((0x0258 << 16) | (0xff))
#define	MFP_PIN_DF_IO12		((0x025C << 16) | (0xff))
#define	MFP_PIN_DF_IO5		((0x0260 << 16) | (0xff))
#define	MFP_PIN_DF_IO13		((0x0264 << 16) | (0xff))
#define	MFP_PIN_DF_IO6		((0x0268 << 16) | (0xff))
#define	MFP_PIN_DF_IO14		((0x026C << 16) | (0xff))
#define	MFP_PIN_DF_IO7		((0x0270 << 16) | (0xff))
#define	MFP_PIN_DF_IO15		((0x0274 << 16) | (0xff))
#define	MFP_PIN_GPIO5		((0x0284 << 16) | (5))
#define	MFP_PIN_GPIO6		((0x0288 << 16) | (6))
#define	MFP_PIN_GPIO7		((0x028C << 16) | (7))
#define	MFP_PIN_GPIO8		((0x0290 << 16) | (8))
#define	MFP_PIN_GPIO9		((0x0294 << 16) | (9))
#define	MFP_PIN_GPIO10		((0x0298 << 16) | (9))
#define	MFP_PIN_GPIO11		((0x029C << 16) | (11))
#define	MFP_PIN_GPIO12		((0x02A0 << 16) | (12))
#define	MFP_PIN_GPIO13		((0x02A4 << 16) | (13))
#define	MFP_PIN_GPIO14		((0x02A8 << 16) | (14))
#define	MFP_PIN_GPIO15		((0x02AC << 16) | (15))
#define	MFP_PIN_GPIO16		((0x02B0 << 16) | (16))
#define	MFP_PIN_GPIO17		((0x02B4 << 16) | (17))
#define	MFP_PIN_GPIO18		((0x02B8 << 16) | (18))
#define	MFP_PIN_GPIO19		((0x02BC << 16) | (19))
#define	MFP_PIN_GPIO20		((0x02C0 << 16) | (20))
#define	MFP_PIN_GPIO21		((0x02C4 << 16) | (21))
#define	MFP_PIN_GPIO22		((0x02C8 << 16) | (22))
#define	MFP_PIN_GPIO23		((0x02CC << 16) | (23))
#define	MFP_PIN_GPIO24		((0x02D0 << 16) | (24))
#define	MFP_PIN_GPIO25		((0x02D4 << 16) | (25))
#define	MFP_PIN_GPIO26		((0x02D8 << 16) | (26))
#define	MFP_PIN_GPIO27		((0x0400 << 16) | (27))
#define	MFP_PIN_GPIO28		((0x0404 << 16) | (28))
#define	MFP_PIN_GPIO29		((0x0408 << 16) | (29))
#define MFP_PIN_ULPI_STP        ((0x040C << 16) | (0xff))
#define MFP_PIN_ULPI_NXT        ((0x0410 << 16) | (0xff))
#define MFP_PIN_ULPI_DIR        ((0x0414 << 16) | (0xff))
#define	MFP_PIN_GPIO30		((0x0418 << 16) | (30))
#define	MFP_PIN_GPIO31		((0x041C << 16) | (31))
#define	MFP_PIN_GPIO32		((0x0420 << 16) | (32))
#define	MFP_PIN_GPIO33		((0x0424 << 16) | (33))
#define	MFP_PIN_GPIO34		((0x0428 << 16) | (34))
#define	MFP_PIN_GPIO35		((0x042C << 16) | (35))
#define	MFP_PIN_GPIO36		((0x0430 << 16) | (36))
#define	MFP_PIN_GPIO37		((0x0434 << 16) | (37))
#define	MFP_PIN_GPIO38		((0x0438 << 16) | (38))
#define	MFP_PIN_GPIO39		((0x043C << 16) | (39))
#define	MFP_PIN_GPIO40		((0x0440 << 16) | (40))
#define	MFP_PIN_GPIO41		((0x0444 << 16) | (41))
#define	MFP_PIN_GPIO42		((0x0448 << 16) | (42))
#define	MFP_PIN_GPIO43		((0x044C << 16) | (43))
#define	MFP_PIN_GPIO44		((0x0450 << 16) | (44))
#define	MFP_PIN_GPIO45		((0x0454 << 16) | (45))
#define	MFP_PIN_GPIO46		((0x0458 << 16) | (46))
#define	MFP_PIN_GPIO47		((0x045C << 16) | (47))
#define	MFP_PIN_GPIO48		((0x0460 << 16) | (48))
#define	MFP_PIN_GPIO49		((0x0464 << 16) | (49))
#define	MFP_PIN_GPIO50		((0x0468 << 16) | (50))
#define	MFP_PIN_GPIO51		((0x046C << 16) | (51))
#define	MFP_PIN_GPIO52		((0x0470 << 16) | (52))
#define	MFP_PIN_GPIO53		((0x0474 << 16) | (53))
#define	MFP_PIN_GPIO54		((0x0478 << 16) | (54))
#define	MFP_PIN_GPIO55		((0x047C << 16) | (55))
#define	MFP_PIN_GPIO56		((0x0480 << 16) | (56))
#define	MFP_PIN_GPIO57		((0x0484 << 16) | (57))
#define	MFP_PIN_GPIO58		((0x0488 << 16) | (58))
#define	MFP_PIN_GPIO59		((0x048C << 16) | (59))
#define	MFP_PIN_GPIO60		((0x0490 << 16) | (60))
#define	MFP_PIN_GPIO61		((0x0494 << 16) | (61))
#define	MFP_PIN_GPIO62		((0x0498 << 16) | (62))
#define	MFP_PIN_GPIO63		((0x049C << 16) | (63))
#define	MFP_PIN_GPIO64		((0x04A0 << 16) | (64))
#define	MFP_PIN_GPIO65		((0x04A4 << 16) | (65))
#define	MFP_PIN_GPIO66		((0x04A8 << 16) | (66))
#define	MFP_PIN_GPIO67		((0x04AC << 16) | (67))
#define	MFP_PIN_GPIO68		((0x04B0 << 16) | (68))
#define	MFP_PIN_GPIO69		((0x04B4 << 16) | (69))
#define	MFP_PIN_GPIO70		((0x04B8 << 16) | (70))
#define	MFP_PIN_GPIO71		((0x04BC << 16) | (71))
#define	MFP_PIN_GPIO72		((0x04C0 << 16) | (72))
#define	MFP_PIN_GPIO73		((0x04C4 << 16) | (73))
#define	MFP_PIN_GPIO74		((0x04C8 << 16) | (74))
#define	MFP_PIN_GPIO75		((0x04CC << 16) | (75))
#define	MFP_PIN_GPIO76		((0x04D0 << 16) | (76))
#define	MFP_PIN_GPIO77		((0x04D4 << 16) | (77))
#define	MFP_PIN_GPIO78		((0x04D8 << 16) | (78))
#define	MFP_PIN_GPIO79		((0x04DC << 16) | (79))
#define	MFP_PIN_GPIO80		((0x04E0 << 16) | (80))
#define	MFP_PIN_GPIO81		((0x04E4 << 16) | (81))
#define	MFP_PIN_GPIO82		((0x04E8 << 16) | (82))
#define	MFP_PIN_GPIO83		((0x04EC << 16) | (83))
#define	MFP_PIN_GPIO84		((0x04F0 << 16) | (84))
#define	MFP_PIN_GPIO85		((0x04F4 << 16) | (85))
#define	MFP_PIN_GPIO86		((0x04F8 << 16) | (86))
#define	MFP_PIN_GPIO87		((0x04FC << 16) | (87))
#define	MFP_PIN_GPIO88		((0x0500 << 16) | (88))
#define	MFP_PIN_GPIO89		((0x0504 << 16) | (89))
#define	MFP_PIN_GPIO90		((0x0508 << 16) | (90))
#define	MFP_PIN_GPIO91		((0x050C << 16) | (91))
#define	MFP_PIN_GPIO92		((0x0510 << 16) | (92))
#define	MFP_PIN_GPIO93		((0x0514 << 16) | (93))
#define	MFP_PIN_GPIO94		((0x0518 << 16) | (94))
#define	MFP_PIN_GPIO95		((0x051C << 16) | (95))
#define	MFP_PIN_GPIO96		((0x0520 << 16) | (96))
#define	MFP_PIN_GPIO97		((0x0524 << 16) | (97))
#define	MFP_PIN_GPIO98		((0x0528 << 16) | (98))
#define	MFP_PIN_GPIO99		((0x0600 << 16) | (99))
#define	MFP_PIN_GPIO100		((0x0604 << 16) | (100))
#define	MFP_PIN_GPIO101		((0x0608 << 16) | (101))
#define	MFP_PIN_GPIO102		((0x060C << 16) | (102))
#define	MFP_PIN_GPIO103		((0x0610 << 16) | (103))
#define	MFP_PIN_GPIO104		((0x0614 << 16) | (104))
#define	MFP_PIN_GPIO105		((0x0618 << 16) | (105))
#define	MFP_PIN_GPIO106		((0x061C << 16) | (106))
#define	MFP_PIN_GPIO107		((0x0620 << 16) | (107))
#define	MFP_PIN_GPIO108		((0x0624 << 16) | (108))
#define	MFP_PIN_GPIO109		((0x0628 << 16) | (109))
#define	MFP_PIN_GPIO110		((0x062C << 16) | (110))
#define	MFP_PIN_GPIO111		((0x0630 << 16) | (111))
#define	MFP_PIN_GPIO112		((0x0634 << 16) | (112))
#define	MFP_PIN_GPIO113		((0x0638 << 16) | (113))
#define	MFP_PIN_GPIO114		((0x063C << 16) | (114))
#define	MFP_PIN_GPIO115		((0x0640 << 16) | (115))
#define	MFP_PIN_GPIO116		((0x0644 << 16) | (116))
#define	MFP_PIN_GPIO117		((0x0648 << 16) | (117))
#define	MFP_PIN_GPIO118		((0x064C << 16) | (118))
#define	MFP_PIN_GPIO119		((0x0650 << 16) | (119))
#define	MFP_PIN_GPIO120		((0x0654 << 16) | (120))
#define	MFP_PIN_GPIO121		((0x0658 << 16) | (121))
#define	MFP_PIN_GPIO122		((0x065C << 16) | (122))
#define	MFP_PIN_GPIO123		((0x0660 << 16) | (123))
#define	MFP_PIN_GPIO124		((0x0664 << 16) | (124))
#define	MFP_PIN_GPIO125		((0x0668 << 16) | (125))
#define	MFP_PIN_GPIO126		((0x066C << 16) | (126))
#define	MFP_PIN_GPIO127		((0x0670 << 16) | (127))
#define	MFP_PIN_GPIO0_2		((0x0674 << 16) | (0))
#define	MFP_PIN_GPIO1_2		((0x0678 << 16) | (1))
#define	MFP_PIN_GPIO2_2		((0x02DC << 16) | (2))
#define	MFP_PIN_GPIO3_2		((0x02E0 << 16) | (3))
#define	MFP_PIN_GPIO4_2		((0x02E4 << 16) | (4))
#define	MFP_PIN_GPIO5_2		((0x02E8 << 16) | (5))
#define	MFP_PIN_GPIO6_2		((0x02EC << 16) | (6))
#define MFP_PIN_GPIO7_2		((0x052C << 16) | (7))
#define MFP_PIN_GPIO8_2		((0x0530 << 16) | (8))
#define MFP_PIN_GPIO9_2		((0x0534 << 16) | (9))
#define MFP_PIN_GPIO10_2	((0x0538 << 16) | (10))
#define MFP_PIN_GPIO11_2	((0x053C << 16) | (11))
#define MFP_PIN_GPIO12_2	((0x0540 << 16) | (12))

#define MHN_MIN_MFP_OFFSET				(MFP_OFFSET(MFP_PIN_GPIO0))
#define MHN_MAX_MFP_OFFSET				(MFP_OFFSET(MFP_PIN_GPIO1_2))

/* Pin GPIO0 alternate function codes */
#define MFP_PIN_GPIO0_AF_GPIO_0				MFP_AF0
#define	MFP_PIN_GPIO0_AF_DF_RDY				MFP_AF1

/* Pin GPIO1 alternate function codes */
#define MFP_PIN_GPIO1_AF_GPIO_1				MFP_AF0
#define	MFP_PIN_GPIO1_AF_nCS2				MFP_AF1

/* Pin GPIO2 alternate function codes */
#define MFP_PIN_GPIO2_AF_GPIO_2				MFP_AF0
#define MFP_PIN_GPIO2_AF_nCS3				MFP_AF1
#define	MFP_PIN_GPIO2_AF_nXCVREN			MFP_AF2

/* Pin GPIO3 alternate function codes */
#define MFP_PIN_GPIO3_AF_GPIO_3				MFP_AF0
#define	MFP_PIN_GPIO3_AF_uIO_IN				MFP_AF1
#define	MFP_PIN_GPIO3_AF_KP_DKIN_6			MFP_AF2
#define	MFP_PIN_GPIO3_AF_MM1_DAT0			MFP_AF4

/* Pin GPIO4 alternate function codes */
#define MFP_PIN_GPIO4_AF_GPIO_4				MFP_AF0
#define	MFP_PIN_GPIO4_AF_uSIM_CARD_STATE		MFP_AF1
#define	MFP_PIN_GPIO4_AF_KP_DKIN_7			MFP_AF2
#define	MFP_PIN_GPIO4_AF_MM1_DAT1			MFP_AF4

/* Pin GPIO5 alternate function codes */
#define MFP_PIN_GPIO5_AF_GPIO_5				MFP_AF0
#define	MFP_PIN_GPIO5_AF_uSIM_uCLK			MFP_AF1
#define	MFP_PIN_GPIO5_AF_KP_MKIN_0			MFP_AF2
#define	MFP_PIN_GPIO5_AF_MM1_DAT2			MFP_AF4

/* Pin GPIO6 alternate function codes */
#define MFP_PIN_GPIO6_AF_GPIO_6				MFP_AF0
#define	MFP_PIN_GPIO6_AF_uSIM_uRST			MFP_AF1
#define	MFP_PIN_GPIO6_AF_KP_MKIN_1			MFP_AF2
#define	MFP_PIN_GPIO6_AF_MM1_DAT3			MFP_AF4

/* Pin GPIO7 alternate function codes */
#define MFP_PIN_GPIO7_AF_GPIO_7				MFP_AF0
#define	MFP_PIN_GPIO7_AF_KP_MKOUT_5			MFP_AF1
#define	MFP_PIN_GPIO7_AF_UART3_RXD			MFP_AF2
#define	MFP_PIN_GPIO7_AF_MM1_CLK			MFP_AF4
#define	MFP_PIN_GPIO7_AF_UART3_TXD			MFP_AF6
#define	MFP_PIN_GPIO7_AF_CLK_BYPASS_XSC			MFP_AF7

/* Pin GPIO8 alternate function codes */
#define MFP_PIN_GPIO8_AF_GPIO_8				MFP_AF0
#define	MFP_PIN_GPIO8_AF_UART3_TXD			MFP_AF2
#define	MFP_PIN_GPIO8_AF_MM1_CMD			MFP_AF4
#define	MFP_PIN_GPIO8_AF_CIR_OUT			MFP_AF5
#define	MFP_PIN_GPIO8_AF_UART3_RXD			MFP_AF6

/* Pin GPIO9 alternate function codes */
#define MFP_PIN_GPIO9_AF_GPIO_9				MFP_AF0
#define	MFP_PIN_GPIO9_AF_SCIO				MFP_AF1
#define	MFP_PIN_GPIO9_AF_KP_MKIN_6			MFP_AF3
#define	MFP_PIN_GPIO9_AF_MM2_DAT0			MFP_AF4

/* Pin GPIO10 alternate function codes */
#define MFP_PIN_GPIO10_AF_GPIO_10			MFP_AF0
#define	MFP_PIN_GPIO10_AF_SC_CARD_STATE			MFP_AF1
#define	MFP_PIN_GPIO10_AF_KP_MKIN_7			MFP_AF3
#define	MFP_PIN_GPIO10_AF_MM2_DAT1			MFP_AF4

/* Pin GPIO11 alternate function codes */
#define MFP_PIN_GPIO11_AF_GPIO_11			MFP_AF0
#define	MFP_PIN_GPIO11_AF_SC_uCLK			MFP_AF1
#define	MFP_PIN_GPIO11_AF_KP_MKOUT_5			MFP_AF3
#define	MFP_PIN_GPIO11_AF_MM2_DAT2			MFP_AF4

/* Pin GPIO12 alternate function codes */
#define MFP_PIN_GPIO12_AF_GPIO_12			MFP_AF0
#define	MFP_PIN_GPIO12_AF_SC_uRST			MFP_AF1
#define	MFP_PIN_GPIO12_AF_KP_MKOUT_6			MFP_AF3
#define	MFP_PIN_GPIO12_AF_MM2_DAT3			MFP_AF4

/* Pin GPIO13 alternate function codes */
#define MFP_PIN_GPIO13_AF_GPIO_13			MFP_AF0
#define	MFP_PIN_GPIO13_AF_KP_MKOUT_7			MFP_AF3
#define	MFP_PIN_GPIO13_AF_MM2_CLK			MFP_AF4

/* Pin GPIO14 alternate function codes */
#define MFP_PIN_GPIO14_AF_GPIO_14			MFP_AF0
#define	MFP_PIN_GPIO14_AF_MM2_CMD			MFP_AF4
#define	MFP_PIN_GPIO14_AF_MM1_CMD			MFP_AF5

/* Pin GPIO15 alternate function codes */
#define MFP_PIN_GPIO15_AF_GPIO_15			MFP_AF0
#define	MFP_PIN_GPIO15_AF_SC_UVS_0			MFP_AF1
#define	MFP_PIN_GPIO15_AF_LCD_nCS			MFP_AF2
#define	MFP_PIN_GPIO15_AF_UART2_CTS			MFP_AF3
#define	MFP_PIN_GPIO15_AF_UART2_RTS			MFP_AF4
#define	MFP_PIN_GPIO15_AF_MM1_CMD			MFP_AF5
#define	MFP_PIN_GPIO15_AF_SSP1_CLK			MFP_AF6

/* Pin GPIO16 alternate function codes */
#define MFP_PIN_GPIO16_AF_GPIO_16			MFP_AF0
#define	MFP_PIN_GPIO16_AF_uSIM_UVS_0			MFP_AF1
#define	MFP_PIN_GPIO16_AF_SSP1_FRM			MFP_AF2
#define	MFP_PIN_GPIO16_AF_CIR_OUT			MFP_AF3
#define	MFP_PIN_GPIO16_AF_UART2_RTS			MFP_AF4
#define	MFP_PIN_GPIO16_AF_UART2_CTS			MFP_AF5
#define MFP_PIN_GPIO16_AF_KP_DKIN_6			MFP_AF6

/* Pin GPIO17 alternate function codes */
#define MFP_PIN_GPIO17_AF_GPIO_17			MFP_AF0
#define	MFP_PIN_GPIO17_AF_PWM0_OUT			MFP_AF1
#define	MFP_PIN_GPIO17_AF_SSP2_FRM			MFP_AF2
#define	MFP_PIN_GPIO17_AF_AC97_SDATA_IN_2		MFP_AF3
#define	MFP_PIN_GPIO17_AF_EXT_SYNC_MVT_0		MFP_AF6

/* Pin GPIO18 alternate function codes */
#define MFP_PIN_GPIO18_AF_GPIO_18			MFP_AF0
#define	MFP_PIN_GPIO18_AF_PWM1_OUT			MFP_AF1
#define	MFP_PIN_GPIO18_AF_SSP1_RXD			MFP_AF2
#define	MFP_PIN_GPIO18_AF_AC97_SDATA_IN_3		MFP_AF3
#define	MFP_PIN_GPIO18_AF_UART2_TXD			MFP_AF4
#define	MFP_PIN_GPIO18_AF_UART2_RXD			MFP_AF5
#define	MFP_PIN_GPIO18_AF_EXT_SYNC_MVT_1		MFP_AF6
#define	MFP_PIN_GPIO18_AF_SSP1_TXD			MFP_AF7

/* Pin GPIO19 alternate function codes */
#define MFP_PIN_GPIO19_AF_GPIO_19			MFP_AF0
#define	MFP_PIN_GPIO19_AF_PWM2_OUT			MFP_AF1
#define	MFP_PIN_GPIO19_AF_SSP2_TXD			MFP_AF2
#define	MFP_PIN_GPIO19_AF_KP_MKOUT_4			MFP_AF3
#define	MFP_PIN_GPIO19_AF_UART2_RXD			MFP_AF4
#define	MFP_PIN_GPIO19_AF_UART2_TXD			MFP_AF5
#define	MFP_PIN_GPIO19_AF_OST_CHOUT_MVT_0		MFP_AF6
#define	MFP_PIN_GPIO19_AF_SSP2_RXD			MFP_AF7

/* Pin GPIO20 alternate function codes */
#define MFP_PIN_GPIO20_AF_GPIO_20			MFP_AF0
#define	MFP_PIN_GPIO20_AF_PWM3_OUT			MFP_AF1
#define	MFP_PIN_GPIO20_AF_SSP1_TXD			MFP_AF2
#define	MFP_PIN_GPIO20_AF_KP_MKOUT_5			MFP_AF3
#define	MFP_PIN_GPIO20_AF_RTC_MVT			MFP_AF4
#define	MFP_PIN_GPIO20_AF_OW_DQ_IN			MFP_AF5
#define	MFP_PIN_GPIO20_AF_OST_CHOUT_MVT_1		MFP_AF6
#define	MFP_PIN_GPIO20_AF_SSP1_RXD			MFP_AF7

/* Pin GPIO21 alternate function codes */
#define MFP_PIN_GPIO21_AF_GPIO_21			MFP_AF0
#define	MFP_PIN_GPIO21_AF_I2C_SCL			MFP_AF1
#define	MFP_PIN_GPIO21_AF_AC97_SDATA_IN_2		MFP_AF2

/* Pin GPIO22 alternate function codes */
#define MFP_PIN_GPIO22_AF_GPIO_22			MFP_AF0
#define	MFP_PIN_GPIO22_AF_I2C_SDA			MFP_AF1
#define	MFP_PIN_GPIO22_AF_AC97_SDATA_IN_3		MFP_AF2

/* Pin GPIO23 alternate function codes */
#define MFP_PIN_GPIO23_AF_GPIO_23			MFP_AF0
#define	MFP_PIN_GPIO23_AF_AC97_RESET			MFP_AF1
#define	MFP_PIN_GPIO23_AF_SSP2_SCLK			MFP_AF2

/* Pin GPIO24 alternate function codes */
#define MFP_PIN_GPIO24_AF_GPIO_24			MFP_AF0
#define	MFP_PIN_GPIO24_AF_AC97_SYSCLK			MFP_AF1
#define	MFP_PIN_GPIO24_AF_MM1_CMD			MFP_AF3
#define	MFP_PIN_GPIO24_AF_SSP2_RXD			MFP_AF4
#define	MFP_PIN_GPIO24_AF_SSP2_TXD			MFP_AF5

/* Pin GPIO25 alternate function codes */
#define MFP_PIN_GPIO25_AF_GPIO_25			MFP_AF0
#define	MFP_PIN_GPIO25_AF_AC97_SDATA_IN_0		MFP_AF1
#define	MFP_PIN_GPIO25_AF_SSP2_SCLK			MFP_AF2

/* Pin GPIO26 alternate function codes */
#define MFP_PIN_GPIO26_AF_GPIO_26			MFP_AF0
#define	MFP_PIN_GPIO26_AF_AC97_SDATA_IN_1		MFP_AF1
#define	MFP_PIN_GPIO26_AF_SSP2_FRM			MFP_AF2

/* Pin GPIO27 alternate function codes */
#define MFP_PIN_GPIO27_AF_GPIO_27			MFP_AF0
#define	MFP_PIN_GPIO27_AF_AC97_SDATA_OUT		MFP_AF1
#define	MFP_PIN_GPIO27_AF_SSP2_TXD			MFP_AF2
#define	MFP_PIN_GPIO27_AF_U2D_OPMODE_0			MFP_AF3
#define	MFP_PIN_GPIO27_AF_SSP2_RXD			MFP_AF5

/* Pin GPIO28 alternate function codes */
#define MFP_PIN_GPIO28_AF_GPIO_28			MFP_AF0
#define	MFP_PIN_GPIO28_AF_AC97_SYNC			MFP_AF1
#define	MFP_PIN_GPIO28_AF_SSP2_RXD			MFP_AF2
#define	MFP_PIN_GPIO28_AF_U2D_OPMODE_1			MFP_AF3
#define	MFP_PIN_GPIO28_AF_SSP2_TXD			MFP_AF5

/* Pin GPIO29 alternate function codes */
#define MFP_PIN_GPIO29_AF_GPIO_29			MFP_AF0
#define	MFP_PIN_GPIO29_AF_AC97_BITCLK			MFP_AF1
#define	MFP_PIN_GPIO29_AF_SSP2_EXTCLK			MFP_AF2
#define	MFP_PIN_GPIO29_AF_MM1_DAT0			MFP_AF3

/* Pin MFP_ULPI_STP alternate function codes */
#define MFP_ULPI_STP_AF_STP				MFP_AF0

/* Pin MFP_ULPI_NXT alternate function codes */
#define MFP_ULPI_NXT_AF_NXT				MFP_AF0

/* Pin MFP_ULPI_DIR alternate function codes */
#define MFP_ULPI_DIR_AF_DIR				MFP_AF0

/* Pin GPIO30 alternate function codes */
#define MFP_PIN_GPIO30_AF_GPIO_30			MFP_AF0
#define	MFP_PIN_GPIO30_AF_USB_P2_2			MFP_AF1
#define	MFP_PIN_GPIO30_AF_UART1_RXD			MFP_AF2
#define	MFP_PIN_GPIO30_AF_ULPI_DATA_OUT_0		MFP_AF3
#define	MFP_PIN_GPIO30_AF_UART1_TXD			MFP_AF4

/* Pin GPIO31 alternate function codes */
#define MFP_PIN_GPIO31_AF_GPIO_31			MFP_AF0
#define	MFP_PIN_GPIO31_AF_USB_P2_6			MFP_AF1
#define	MFP_PIN_GPIO31_AF_UART1_TXD			MFP_AF2
#define	MFP_PIN_GPIO31_AF_ULPI_DATA_OUT_1		MFP_AF3
#define	MFP_PIN_GPIO31_AF_UART1_RXD			MFP_AF4

/* Pin GPIO32 alternate function codes */
#define MFP_PIN_GPIO32_AF_GPIO_32			MFP_AF0
#define	MFP_PIN_GPIO32_AF_USB_P2_4			MFP_AF1
#define	MFP_PIN_GPIO32_AF_UART1_CTS			MFP_AF2
#define	MFP_PIN_GPIO32_AF_ULPI_DATA_OUT_2		MFP_AF3
#define	MFP_PIN_GPIO32_AF_UART1_RTS			MFP_AF4

/* Pin GPIO33 alternate function codes */
#define MFP_PIN_GPIO33_AF_GPIO_33			MFP_AF0
#define	MFP_PIN_GPIO33_AF_ULPI_OTG_INTR			MFP_AF1
#define	MFP_PIN_GPIO33_AF_UART1_DCD			MFP_AF2
#define	MFP_PIN_GPIO33_AF_ULPI_DATA_OUT_3		MFP_AF3
#define	MFP_PIN_GPIO33_AF_SSP1_SCLK			MFP_AF5
#define	MFP_PIN_GPIO33_AF_SSP2_SCLK			MFP_AF6

/* Pin GPIO34 alternate function codes */
#define MFP_PIN_GPIO34_AF_GPIO_34			MFP_AF0
#define	MFP_PIN_GPIO34_AF_USB_P2_5			MFP_AF1
#define	MFP_PIN_GPIO34_AF_UART1_DSR			MFP_AF2
#define	MFP_PIN_GPIO34_AF_ULPI_DATA_OUT_4		MFP_AF3
#define	MFP_PIN_GPIO34_AF_UART1_DTR			MFP_AF4
#define	MFP_PIN_GPIO34_AF_SSP1_FRM			MFP_AF5
#define	MFP_PIN_GPIO34_AF_SSP2_FRM			MFP_AF6

/* Pin GPIO35 alternate function codes */
#define MFP_PIN_GPIO35_AF_GPIO_35			MFP_AF0
#define	MFP_PIN_GPIO35_AF_USB_P2_3			MFP_AF1
#define	MFP_PIN_GPIO35_AF_UART1_RI			MFP_AF2
#define	MFP_PIN_GPIO35_AF_ULPI_DATA_OUT_5		MFP_AF3
#define	MFP_PIN_GPIO35_AF_SSP1_RXD			MFP_AF4
#define	MFP_PIN_GPIO35_AF_SSP1_TXD			MFP_AF5
#define	MFP_PIN_GPIO35_AF_SSP2_RXD			MFP_AF6
#define	MFP_PIN_GPIO35_AF_SSP2_TXD			MFP_AF7

/* Pin GPIO36 alternate function codes */
#define MFP_PIN_GPIO36_AF_GPIO_36			MFP_AF0
#define	MFP_PIN_GPIO36_AF_USB_P2_1			MFP_AF1
#define	MFP_PIN_GPIO36_AF_UART1_DTR			MFP_AF2
#define	MFP_PIN_GPIO36_AF_ULPI_DATA_OUT_6		MFP_AF3
#define	MFP_PIN_GPIO36_AF_UART1_DSR			MFP_AF4
#define	MFP_PIN_GPIO36_AF_SSP1_TXD			MFP_AF5
#define	MFP_PIN_GPIO36_AF_SSP1_RXD			MFP_AF6
#define	MFP_PIN_GPIO36_AF_SSP2_TXD			MFP_AF7

/* Pin GPIO37 alternate function codes */
#define MFP_PIN_GPIO37_AF_GPIO_37			MFP_AF0
#define	MFP_PIN_GPIO37_AF_UART1_RTS			MFP_AF2
#define	MFP_PIN_GPIO37_AF_ULPI_DATA_OUT_7		MFP_AF3
#define	MFP_PIN_GPIO37_AF_UART1_CTS			MFP_AF4

/* Pin GPIO38 alternate function codes */
#define MFP_PIN_GPIO38_AF_GPIO_38			MFP_AF0
#define	MFP_PIN_GPIO38_AF_ULPI_CLK			MFP_AF1
#define	MFP_PIN_GPIO38_AF_KP_MKOUT_5			MFP_AF5

/* Pin GPIO39 alternate function codes */
#define MFP_PIN_GPIO39_AF_GPIO_39			MFP_AF0
#define	MFP_PIN_GPIO39_AF_CI_DD_0			MFP_AF1

/* Pin GPIO40 alternate function codes */
#define MFP_PIN_GPIO40_AF_GPIO_40			MFP_AF0
#define	MFP_PIN_GPIO40_AF_CI_DD_1			MFP_AF1

/* Pin GPIO41 alternate function codes */
#define MFP_PIN_GPIO41_AF_GPIO_41			MFP_AF0
#define	MFP_PIN_GPIO41_AF_CI_DD_2			MFP_AF1

/* Pin GPIO42 alternate function codes */
#define MFP_PIN_GPIO42_AF_GPIO_42			MFP_AF0
#define MFP_PIN_GPIO42_AF_CI_DD_3			MFP_AF1

/* Pin GPIO43 alternate function codes */
#define MFP_PIN_GPIO43_AF_GPIO_43			MFP_AF0
#define MFP_PIN_GPIO43_AF_CI_DD_4			MFP_AF1

/* Pin GPIO44 alternate function codes */
#define MFP_PIN_GPIO44_AF_GPIO_44			MFP_AF0
#define MFP_PIN_GPIO44_AF_CI_DD_5			MFP_AF1

/* Pin GPIO45 alternate function codes */
#define MFP_PIN_GPIO45_AF_GPIO_45			MFP_AF0
#define MFP_PIN_GPIO45_AF_CI_DD_6			MFP_AF1

/* Pin GPIO46 alternate function codes */
#define MFP_PIN_GPIO46_AF_CI_DD_7			MFP_AF0
#define MFP_PIN_GPIO46_AF_GPIO_46			MFP_AF1

/* Pin GPIO47 alternate function codes */
#define MFP_PIN_GPIO47_AF_GPIO_47			MFP_AF0
#define MFP_PIN_GPIO47_AF_CI_DD_8			MFP_AF1

/* Pin GPIO48 alternate function codes */
#define MFP_PIN_GPIO48_AF_GPIO_48			MFP_AF0
#define MFP_PIN_GPIO48_AF_CI_DD_9			MFP_AF1

/* Pin GPIO49 alternate function codes */
#define MFP_PIN_GPIO49_AF_CI_MCLK			MFP_AF0
#define MFP_PIN_GPIO49_AF_48M_CLK			MFP_AF2
#define MFP_PIN_GPIO49_AF_GPIO_49			MFP_AF3

/* Pin GPIO50 alternate function codes */
#define MFP_PIN_GPIO50_AF_CI_PCLK			MFP_AF0
#define MFP_PIN_GPIO50_AF_GPIO_50			MFP_AF2

/* Pin GPIO51 alternate function codes */
#define MFP_PIN_GPIO51_AF_CI_LV				MFP_AF0
#define MFP_PIN_GPIO51_AF_UTM_OPMODE0			MFP_AF1
#define MFP_PIN_GPIO51_AF_GPIO_51			MFP_AF3

/* Pin GPIO52 alternate function codes */
#define MFP_PIN_GPIO52_AF_CI_FV				MFP_AF0
#define MFP_PIN_GPIO52_AF_UTM_OPMODE1			MFP_AF1
#define MFP_PIN_GPIO52_AF_GPIO_52			MFP_AF3
#define MFP_PIN_GPIO52_AF_TXVALID			MFP_AF4

/* Pin GPIO53 alternate function codes */
#define MFP_PIN_GPIO53_AF_GPIO_53			MFP_AF0
#define MFP_PIN_GPIO53_AF_KP_MKOUT_6			MFP_AF5

/* Pin GPIO54 alternate function codes */
#define MFP_PIN_GPIO54_AF_GPIO_54			MFP_AF0
#define MFP_PIN_GPIO54_AF_LCD_LDD_0			MFP_AF1
#define MFP_PIN_GPIO54_AF_MSLCD_DATA_MVT_0		MFP_AF7

/* Pin GPIO55 alternate function codes */
#define MFP_PIN_GPIO55_AF_GPIO_55			MFP_AF0
#define MFP_PIN_GPIO55_AF_LCD_LDD_1			MFP_AF1
#define MFP_PIN_GPIO55_AF_MSLCD_DATA_MVT_1		MFP_AF7

/* Pin GPIO56 alternate function codes */
#define MFP_PIN_GPIO56_AF_GPIO_56			MFP_AF0
#define MFP_PIN_GPIO56_AF_LCD_LDD_2			MFP_AF1
#define MFP_PIN_GPIO56_AF_MSLCD_DATA_MVT_2		MFP_AF7

/* Pin GPIO57 alternate function codes */
#define MFP_PIN_GPIO57_AF_GPIO_57			MFP_AF0
#define MFP_PIN_GPIO57_AF_LCD_LDD_3			MFP_AF1
#define MFP_PIN_GPIO57_AF_MSLCD_DATA_MVT_3		MFP_AF7

/* Pin GPIO58 alternate function codes */
#define MFP_PIN_GPIO58_AF_GPIO_58			MFP_AF0
#define MFP_PIN_GPIO58_AF_LCD_LDD_4			MFP_AF1
#define MFP_PIN_GPIO58_AF_MSLCD_DATA_MVT_4		MFP_AF7

/* Pin GPIO59 alternate function codes */
#define MFP_PIN_GPIO59_AF_GPIO_59			MFP_AF0
#define MFP_PIN_GPIO59_AF_LCD_LDD_5			MFP_AF1
#define MFP_PIN_GPIO59_AF_MSLCD_DATA_MVT_5		MFP_AF7

/* Pin GPIO60 alternate function codes */
#define MFP_PIN_GPIO60_AF_GPIO_60			MFP_AF0
#define MFP_PIN_GPIO60_AF_LCD_LDD_6			MFP_AF1
#define MFP_PIN_GPIO60_AF_MSLCD_DATA_MVT_6		MFP_AF7

/* Pin GPIO61 alternate function codes */
#define MFP_PIN_GPIO61_AF_GPIO_61			MFP_AF0
#define MFP_PIN_GPIO61_AF_LCD_LDD_7			MFP_AF1
#define MFP_PIN_GPIO61_AF_MSLCD_DATA_MVT_7		MFP_AF7

/* Pin GPIO62 alternate function codes */
#define MFP_PIN_GPIO62_AF_GPIO_62			MFP_AF0
#define MFP_PIN_GPIO62_AF_LCD_LDD_8			MFP_AF1
#define MFP_PIN_GPIO62_AF_LCD_CS_N			MFP_AF2
#define MFP_PIN_GPIO62_AF_MSLCD_DATA_MVT_8		MFP_AF7

/* Pin GPIO63 alternate function codes */
#define MFP_PIN_GPIO63_AF_GPIO_63			MFP_AF0
#define MFP_PIN_GPIO63_AF_LCD_LDD_9			MFP_AF1
#define MFP_PIN_GPIO63_AF_LCD_VSYNC			MFP_AF2
#define MFP_PIN_GPIO63_AF_MSLCD_DATA_MVT_9		MFP_AF7

/* Pin GPIO64 alternate function codes */
#define MFP_PIN_GPIO64_AF_GPIO_64			MFP_AF0
#define MFP_PIN_GPIO64_AF_LCD_LDD_10			MFP_AF1
#define MFP_PIN_GPIO64_AF_SSP2_SCLK			MFP_AF2
#define MFP_PIN_GPIO64_AF_U2D_XCVR			MFP_AF3
#define MFP_PIN_GPIO64_AF_MSLCD_DATA_MVT_10		MFP_AF7

/* Pin GPIO65 alternate function codes */
#define MFP_PIN_GPIO65_AF_GPIO_65			MFP_AF0
#define MFP_PIN_GPIO65_AF_LCD_LDD_11			MFP_AF1
#define MFP_PIN_GPIO65_AF_SSP2_FRM			MFP_AF2
#define MFP_PIN_GPIO65_AF_U2D_TERM			MFP_AF3
#define MFP_PIN_GPIO65_AF_MSLCD_DATA_MVT_11		MFP_AF7

/* Pin GPIO66 alternate function codes */
#define MFP_PIN_GPIO66_AF_GPIO_66			MFP_AF0
#define MFP_PIN_GPIO66_AF_LCD_LDD_12			MFP_AF1
#define MFP_PIN_GPIO66_AF_SSP2_RXD			MFP_AF2
#define MFP_PIN_GPIO66_AF_SSP2_TXD			MFP_AF4
#define MFP_PIN_GPIO66_AF_MSLCD_DATA_MVT_12		MFP_AF7

/* Pin GPIO67 alternate function codes */
#define MFP_PIN_GPIO67_AF_GPIO_67			MFP_AF0
#define MFP_PIN_GPIO67_AF_LCD_LDD_13			MFP_AF1
#define MFP_PIN_GPIO67_AF_SSP2_TXD			MFP_AF2
#define MFP_PIN_GPIO67_AF_SSP2_RXD			MFP_AF4
#define MFP_PIN_GPIO67_AF_MSLCD_DATA_MVT_13		MFP_AF7

/* Pin GPIO68 alternate function codes */
#define MFP_PIN_GPIO68_AF_GPIO_68			MFP_AF0
#define MFP_PIN_GPIO68_AF_LCD_LDD_14			MFP_AF1
#define MFP_PIN_GPIO68_AF_SSP3_SCLK			MFP_AF2
#define MFP_PIN_GPIO68_AF_MSLCD_DATA_MVT_14		MFP_AF7

/* Pin GPIO69 alternate function codes */
#define MFP_PIN_GPIO69_AF_GPIO_69			MFP_AF0
#define MFP_PIN_GPIO69_AF_LCD_LDD_15			MFP_AF1
#define MFP_PIN_GPIO69_AF_SSP3_FRM			MFP_AF2
#define MFP_PIN_GPIO69_AF_MSLCD_DATA_MVT_15		MFP_AF7

/* Pin GPIO70 alternate function codes */
#define MFP_PIN_GPIO70_AF_GPIO_70			MFP_AF0
#define MFP_PIN_GPIO70_AF_LCD_LDD_16			MFP_AF1
#define MFP_PIN_GPIO70_AF_SSP3_TXD			MFP_AF2
#define MFP_PIN_GPIO70_KP_MKIN_6			MFP_AF3
#define MFP_PIN_GPIO70_SSP3_RXD				MFP_AF5

/* Pin GPIO71 alternate function codes */
#define MFP_PIN_GPIO71_AF_GPIO_71			MFP_AF0
#define MFP_PIN_GPIO71_AF_LCD_LDD_17			MFP_AF1
#define MFP_PIN_GPIO71_AF_SSP3_RXD			MFP_AF2
#define MFP_PIN_GPIO71_AF_KP_MKIN_7			MFP_AF3
#define MFP_PIN_GPIO71_AF_SSP3_TXD			MFP_AF5
#define MFP_PIN_GPIO71_AF_EXT_MATCH_MVT			MFP_AF6

/* Pin GPIO72 alternate function codes */
#define MFP_PIN_GPIO72_AF_GPIO_72			MFP_AF0
#define MFP_PIN_GPIO72_AF_LCD_L_FCLK			MFP_AF1
#define MFP_PIN_GPIO72_AF_MSLCD_FCLK_MVT		MFP_AF7

/* Pin GPIO73 alternate function codes */
#define MFP_PIN_GPIO73_AF_GPIO_73			MFP_AF0
#define MFP_PIN_GPIO73_AF_LCD_L_LCLK			MFP_AF1
#define MFP_PIN_GPIO73_AF_MSLCD_LCLK_MVT		MFP_AF7

/* Pin GPIO74 alternate function codes */
#define MFP_PIN_GPIO74_AF_GPIO_74			MFP_AF0
#define MFP_PIN_GPIO74_AF_LCD_L_PCLK			MFP_AF1
#define MFP_PIN_GPIO74_AF_MSLCD_PCLK_MVT		MFP_AF7

/* Pin GPIO75 alternate function codes */
#define MFP_PIN_GPIO75_AF_GPIO_75			MFP_AF0
#define MFP_PIN_GPIO75_AF_LCD_L_BIAS			MFP_AF1
#define MFP_PIN_GPIO75_AF_MSLCD_L_BIAS_MVT		MFP_AF2

/* Pin GPIO76 alternate function codes */
#define MFP_PIN_GPIO76_AF_GPIO_76			MFP_AF0
#define MFP_PIN_GPIO76_AF_LCD_VSYNC			MFP_AF2

/* Pin GPIO77 alternate function codes */
#define MFP_PIN_GPIO77_AF_GPIO_77			MFP_AF0
#define MFP_PIN_GPIO77_AF_UART1_RXD			MFP_AF1
#define MFP_PIN_GPIO77_AF_USB_P3_1			MFP_AF2
#define MFP_PIN_GPIO77_AF_UART1_TXD			MFP_AF3
#define MFP_PIN_GPIO77_AF_MM2_DAT0			MFP_AF4
#define MFP_PIN_GPIO77_AF_MSL_IB_DAT0			MFP_AF5
#define MFP_PIN_GPIO77_AF_MSL_OB_DAT0			MFP_AF6

/* Pin GPIO78 alternate function codes */
#define MFP_PIN_GPIO78_AF_GPIO_78			MFP_AF0
#define MFP_PIN_GPIO78_AF_UART1_TXD			MFP_AF1
#define MFP_PIN_GPIO78_AF_USB_P3_3			MFP_AF2
#define MFP_PIN_GPIO78_AF_UART1_RXD			MFP_AF3
#define MFP_PIN_GPIO78_AF_MM2_DATA_OUT_1		MFP_AF4
#define MFP_PIN_GPIO78_AF_KP_MKOUT_7			MFP_AF5
#define MFP_PIN_GPIO78_AF_MSL_OB_CLK			MFP_AF6

/* Pin GPIO79 alternate function codes */
#define MFP_PIN_GPIO79_AF_GPIO_79			MFP_AF0
#define MFP_PIN_GPIO79_AF_UART1_CTS			MFP_AF1
#define MFP_PIN_GPIO79_AF_USB_P3_3			MFP_AF2
#define MFP_PIN_GPIO79_AF_UART1_RTS			MFP_AF3
#define MFP_PIN_GPIO79_AF_MM2_DAT2			MFP_AF4
#define MFP_PIN_GPIO79_AF_MSL_IB_STB			MFP_AF5
#define MFP_PIN_GPIO79_AF_MSL_OB_STB			MFP_AF6

/* Pin GPIO80 alternate function codes */
#define MFP_PIN_GPIO80_AF_GPIO_80			MFP_AF0
#define MFP_PIN_GPIO80_AF_UART1_DCD			MFP_AF1
#define MFP_PIN_GPIO80_AF_USB_P3_4			MFP_AF2
#define MFP_PIN_GPIO80_AF_MM2_DAT3			MFP_AF4
#define MFP_PIN_GPIO80_AF_MSL_IB_WAIT			MFP_AF5
#define MFP_PIN_GPIO80_AF_MSL_OB_WAIT			MFP_AF6

/* Pin GPIO81 alternate function codes */
#define MFP_PIN_GPIO81_AF_GPIO_81			MFP_AF0
#define MFP_PIN_GPIO81_AF_UART1_DSR			MFP_AF1
#define MFP_PIN_GPIO81_AF_USB_P3_5			MFP_AF2
#define MFP_PIN_GPIO81_AF_UART1_DTR			MFP_AF3
#define MFP_PIN_GPIO81_AF_MM2_CLK			MFP_AF4
#define MFP_PIN_GPIO81_AF_MSL_OB_DAT0			MFP_AF5
#define MFP_PIN_GPIO81_AF_MSL_IB_DAT0			MFP_AF6

/* Pin GPIO82 alternate function codes */
#define MFP_PIN_GPIO82_AF_GPIO_82			MFP_AF0
#define MFP_PIN_GPIO82_AF_UART1_RI			MFP_AF1
#define MFP_PIN_GPIO82_AF_USB_P3_6			MFP_AF2
#define MFP_PIN_GPIO82_AF_MM2_CMD			MFP_AF4
#define MFP_PIN_GPIO82_AF_MSL_OB_CLK			MFP_AF5
#define MFP_PIN_GPIO82_AF_MSL_IB_CLK			MFP_AF6

/* Pin GPIO83 alternate function codes */
#define MFP_PIN_GPIO83_AF_GPIO_83			MFP_AF0
#define MFP_PIN_GPIO83_AF_UART1_DTR			MFP_AF1
#define MFP_PIN_GPIO83_AF_UART1_DSR			MFP_AF3
#define MFP_PIN_GPIO83_AF_MSL_OB_STB			MFP_AF4
#define MFP_PIN_GPIO83_AF_KP_DKIN_2			MFP_AF5
#define MFP_PIN_GPIO83_AF_MSL_IB_STB			MFP_AF6

/* Pin GPIO84 alternate function codes */
#define MFP_PIN_GPIO84_AF_GPIO_84			MFP_AF0
#define MFP_PIN_GPIO84_AF_UART1_RTS			MFP_AF1
#define MFP_PIN_GPIO84_AF_UART_CTS			MFP_AF3
#define MFP_PIN_GPIO84_AF_MSL_OB_WAIT			MFP_AF4
#define MFP_PIN_GPIO84_AF_KP_DKIN_1			MFP_AF5
#define MFP_PIN_GPIO84_AF_MSL_IB_WAIT			MFP_AF6

/* Pin GPIO85 alternate function codes */
#define MFP_PIN_GPIO85_AF_GPIO_85			MFP_AF0
#define MFP_PIN_GPIO85_AF_SSP1_SCLK			MFP_AF1
#define MFP_PIN_GPIO85_AF_KP_MKOUT_0			MFP_AF2
#define MFP_PIN_GPIO85_AF_KP_DKIN_0			MFP_AF3
#define MFP_PIN_GPIO85_AF_MSL_IB_DAT1			MFP_AF4
#define MFP_PIN_GPIO85_AF_MSL_OB_DAT1			MFP_AF6

/* Pin GPIO86 alternate function codes */
#define MFP_PIN_GPIO86_AF_GPIO_86			MFP_AF0
#define MFP_PIN_GPIO86_AF_SSP1_FRM			MFP_AF1
#define MFP_PIN_GPIO86_AF_KP_MKOUT_1			MFP_AF2
#define MFP_PIN_GPIO86_AF_KP_DKIN_1			MFP_AF3
#define MFP_PIN_GPIO86_AF_MSL_IB_DAT2			MFP_AF4
#define MFP_PIN_GPIO86_AF_MSL_OB_DAT2			MFP_AF6

/* Pin GPIO87 alternate function codes */
#define MFP_PIN_GPIO87_AF_GPIO_87			MFP_AF0
#define MFP_PIN_GPIO87_AF_SSP1_TXD			MFP_AF1
#define MFP_PIN_GPIO87_AF_KP_MKOUT2			MFP_AF2
#define MFP_PIN_GPIO87_AF_KP_DKIN2			MFP_AF3
#define MFP_PIN_GPIO87_AF_MSL_IB_DATA3			MFP_AF4
#define MFP_PIN_GPIO87_AF_SSP1_RXD			MFP_AF6
#define MFP_PIN_GPIO87_AF_MSL_OB_DAT3			MFP_AF7

/* Pin GPIO88 alternate function codes */
#define MFP_PIN_GPIO88_AF_GPIO_88			MFP_AF0
#define MFP_PIN_GPIO88_AF_SSP1_RXD			MFP_AF1
#define MFP_PIN_GPIO88_AF_KP_MKOUT_3			MFP_AF2
#define MFP_PIN_GPIO88_AF_KP_DKIN_3			MFP_AF3
#define MFP_PIN_GPIO88_AF_MSL_OB_DAT1			MFP_AF4
#define MFP_PIN_GPIO88_AF_SSP1_TXD			MFP_AF6
#define MFP_PIN_GPIO88_AF_MSL_IB_DAT1			MFP_AF7

/* Pin GPIO89 alternate function codes */
#define MFP_PIN_GPIO89_AF_GPIO_89			MFP_AF0
#define MFP_PIN_GPIO89_AF_SSP1_EXTCLK			MFP_AF1
#define MFP_PIN_GPIO89_AF_SC_UVS1			MFP_AF2
#define MFP_PIN_GPIO89_AF_KP_DKIN_3			MFP_AF3
#define MFP_PIN_GPIO89_AF_MSL_OB_DAT2			MFP_AF4
#define MFP_PIN_GPIO89_AF_MSL_IB_DAT2			MFP_AF6

/* Pin GPIO90 alternate function codes */
#define MFP_PIN_GPIO90_AF_GPIO_90			MFP_AF0
#define MFP_PIN_GPIO90_AF_SSP1_SYSCLK			MFP_AF1
#define MFP_PIN_GPIO90_AF_SC_UVS2			MFP_AF2
#define MFP_PIN_GPIO90_AF_MSL_IB_DAT3			MFP_AF3
#define MFP_PIN_GPIO90_AF_MSL_OB_DAT3			MFP_AF4
#define MFP_PIN_GPIO90_AF_U2D_OPMODE_0			MFP_AF5

/* Pin GPIO91 alternate function codes */
#define MFP_PIN_GPIO91_AF_GPIO_91			MFP_AF0
#define MFP_PIN_GPIO91_AF_SSP3_SCLK			MFP_AF1
#define MFP_PIN_GPIO91_AF_UART3_CTS			MFP_AF2
#define MFP_PIN_GPIO91_AF_UART3_RTS			MFP_AF4

/* Pin GPIO92 alternate function codes */
#define MFP_PIN_GPIO92_AF_GPIO_92			MFP_AF0
#define MFP_PIN_GPIO92_AF_SSP3_FRM			MFP_AF1
#define MFP_PIN_GPIO92_AF_UART3_RTS			MFP_AF2
#define MFP_PIN_GPIO92_AF_UART3_CTS			MFP_AF4

/* Pin GPIO93 alternate function codes */
#define MFP_PIN_GPIO93_AF_GPIO_93			MFP_AF0
#define MFP_PIN_GPIO93_AF_SSP3_TXD			MFP_AF1
#define MFP_PIN_GPIO93_AF_UART3_TXD			MFP_AF2
#define MFP_PIN_GPIO93_AF_UART3_RXD			MFP_AF4
#define MFP_PIN_GPIO93_AF_SSP3_RXD			MFP_AF5

/* Pin GPIO94 alternate function codes */
#define MFP_PIN_GPIO94_AF_GPIO_94			MFP_AF0
#define MFP_PIN_GPIO94_AF_SSP3_RXD			MFP_AF1
#define MFP_PIN_GPIO94_AF_UART3_RXD			MFP_AF2
#define MFP_PIN_GPIO94_AF_UART3_TXD			MFP_AF4
#define MFP_PIN_GPIO94_AF_SSP3_TXD			MFP_AF5

/* Pin GPIO95 alternate function codes */
#define MFP_PIN_GPIO95_AF_GPIO_95			MFP_AF0
#define MFP_PIN_GPIO95_AF_SSP4_SCLK			MFP_AF1

/* Pin GPIO96 alternate function codes */
#define MFP_PIN_GPIO96_AF_GPIO_96			MFP_AF0
#define MFP_PIN_GPIO96_AF_SSP4_FRM			MFP_AF1
#define MFP_PIN_GPIO96_AF_U2D_XCVR_SEL			MFP_AF2

/* Pin GPIO97 alternate function codes */
#define MFP_PIN_GPIO97_AF_GPIO_97			MFP_AF0
#define MFP_PIN_GPIO97_AF_SSP4_TXD			MFP_AF1
#define MFP_PIN_GPIO97_AF_U2D_TERM_SEL			MFP_AF2
#define MFP_PIN_GPIO97_AF_SSP4_RXD			MFP_AF5

/* Pin GPIO98 alternate function codes */
#define MFP_PIN_GPIO98_AF_GPIO_98			MFP_AF0
#define MFP_PIN_GPIO98_AF_SSP4_RXD			MFP_AF1
#define MFP_PIN_GPIO98_AF_SSP4_TXD			MFP_AF5

/* Pin GPIO99 alternate function codes */
#define MFP_PIN_GPIO99_AF_GPIO_99			MFP_AF0
#define MFP_PIN_GPIO99_AF_UART1_RXD			MFP_AF1
#define MFP_PIN_GPIO99_AF_UART1_TXD			MFP_AF6

/* Pin GPIO100 alternate function codes */
#define MFP_PIN_GPIO100_AF_GPIO_100			MFP_AF0
#define MFP_PIN_GPIO100_AF_UART1_TXD			MFP_AF1
#define MFP_PIN_GPIO100_AF_UART1_RXD			MFP_AF6
#define MFP_PIN_GPIO100_AF_KP_MKIN_6			MFP_AF7

/* Pin GPIO101 alternate function codes */
#define MFP_PIN_GPIO101_AF_GPIO_101			MFP_AF0
#define MFP_PIN_GPIO101_AF_UART1_CTS			MFP_AF1
#define MFP_PIN_GPIO101_AF_U2D_XCVR_SELECT		MFP_AF3
#define MFP_PIN_GPIO101_AF_UART1_RTS			MFP_AF6
#define MFP_PIN_GPIO101_AF_KP_MKIN_7			MFP_AF7

/* Pin GPIO102 alternate function codes */
#define MFP_PIN_GPIO102_AF_GPIO_102			MFP_AF0
#define MFP_PIN_GPIO102_AF_UART1_DCD			MFP_AF1
#define MFP_PIN_GPIO102_AF_U2D_TERM_SELECT		MFP_AF3
#define MFP_PIN_GPIO102_AF_UART1_TXD			MFP_AF4
#define MFP_PIN_GPIO102_AF_UART1_RXD			MFP_AF6

/* Pin GPIO103 alternate function codes */
#define MFP_PIN_GPIO103_AF_GPIO_103			MFP_AF0
#define MFP_PIN_GPIO103_AF_UART1_DSR			MFP_AF1
#define MFP_PIN_GPIO103_AF_MM3_CLK			MFP_AF2
#define MFP_PIN_GPIO103_AF_UART1_DTR			MFP_AF6

/* Pin GPIO104 alternate function codes */
#define MFP_PIN_GPIO104_AF_GPIO_104			MFP_AF0
#define MFP_PIN_GPIO104_AF_UART1_RI			MFP_AF1
#define MFP_PIN_GPIO104_AF_UART1_RXD			MFP_AF4
/*#define MFP_PIN_GPIO104_AF_KP_MKOUT_6			MFP_AF5 */

/* Pin GPIO105 alternate function codes */
#define MFP_PIN_GPIO105_AF_GPIO_105			MFP_AF0
#define MFP_PIN_GPIO105_AF_UART1_DTR			MFP_AF1
#define MFP_PIN_GPIO105_AF_MM3_CMD			MFP_AF2
#define MFP_PIN_GPIO105_AF_KP_MKOUT_7			MFP_AF5
#define MFP_PIN_GPIO105_AF_UART1_DSR			MFP_AF6

/* Pin GPIO106 alternate function codes */
#define MFP_PIN_GPIO106_AF_GPIO_106			MFP_AF0
#define MFP_PIN_GPIO106_AF_UART1_RTS			MFP_AF1
#define MFP_PIN_GPIO106_AF_UTM_OPMODE_1			MFP_AF3
#define MFP_PIN_GPIO106_AF_UART1_CTS			MFP_AF6

/* Pin GPIO107 alternate function codes */
#define MFP_PIN_GPIO107_AF_GPIO_107			MFP_AF0
#define MFP_PIN_GPIO107_AF_UART3_CTS			MFP_AF1
#define MFP_PIN_GPIO107_AF_KP_DKIN_0			MFP_AF2
#define MFP_PIN_GPIO107_AF_UART3_RTS			MFP_AF3

/* Pin GPIO108 alternate function codes */
#define MFP_PIN_GPIO108_AF_GPIO_108			MFP_AF0
#define MFP_PIN_GPIO108_AF_UART3_RTS			MFP_AF1
#define MFP_PIN_GPIO108_AF_KP_DKIN_1			MFP_AF2
#define MFP_PIN_GPIO108_AF_UART3_CTS			MFP_AF3

/* Pin GPIO109 alternate function codes */
#define MFP_PIN_GPIO109_AF_GPIO_109			MFP_AF0
#define MFP_PIN_GPIO109_AF_UART3_TXD			MFP_AF1
#define MFP_PIN_GPIO109_AF_KP_DKIN_2			MFP_AF2
#define MFP_PIN_GPIO109_AF_UART3_RXD			MFP_AF3

/* Pin GPIO110 alternate function codes */
#define MFP_PIN_GPIO110_AF_GPIO_110			MFP_AF0
#define MFP_PIN_GPIO110_AF_UART3_RXD			MFP_AF1
#define MFP_PIN_GPIO110_AF_KP_DKIN_3			MFP_AF2
#define MFP_PIN_GPIO110_AF_UART3_TXD			MFP_AF3
#define MFP_PIN_GPIO110_AF_U2D_OPMODE_1			MFP_AF4

/* Pin GPIO111 alternate function codes */
#define MFP_PIN_GPIO111_AF_GPIO_111			MFP_AF0
#define MFP_PIN_GPIO111_AF_UART2_RTS			MFP_AF1
#define MFP_PIN_GPIO111_AF_KP_DKIN_4			MFP_AF2
#define MFP_PIN_GPIO111_AF_UART2_CTS			MFP_AF3

/* Pin GPIO112 alternate function codes */
#define MFP_PIN_GPIO112_AF_GPIO_112			MFP_AF0
#define MFP_PIN_GPIO112_AF_UART2_RXD			MFP_AF1
#define MFP_PIN_GPIO112_AF_KP_DKIN_5			MFP_AF2
#define MFP_PIN_GPIO112_AF_UART2_TXD			MFP_AF3
#define MFP_PIN_GPIO112_AF_KP_MKIN_6			MFP_AF4

/* Pin GPIO113 alternate function codes */
#define MFP_PIN_GPIO113_AF_GPIO_113			MFP_AF0
#define MFP_PIN_GPIO113_AF_UART2_TXD			MFP_AF1
#define MFP_PIN_GPIO113_AF_KP_DKIN_6			MFP_AF2
#define MFP_PIN_GPIO113_AF_UART2_RXD			MFP_AF3
#define MFP_PIN_GPIO113_AF_KP_MKIN_7			MFP_AF4

/* Pin GPIO114 alternate function codes */
#define MFP_PIN_GPIO114_AF_GPIO_114			MFP_AF0
#define MFP_PIN_GPIO114_AF_UART2_CTS			MFP_AF1
#define MFP_PIN_GPIO114_AF_KP_DKIN_7			MFP_AF2
#define MFP_PIN_GPIO114_AF_KP_UART2_RTS			MFP_AF3

/* Pin GPIO115 alternate function codes */
#define MFP_PIN_GPIO115_AF_GPIO_115			MFP_AF0
#define MFP_PIN_GPIO115_AF_KP_MKIN_0			MFP_AF1
#define MFP_PIN_GPIO115_AF_KP_DKIN_0			MFP_AF2

/* Pin GPIO116 alternate function codes */
#define MFP_PIN_GPIO116_AF_GPIO_116			MFP_AF0
#define MFP_PIN_GPIO116_AF_KP_MKIN_1			MFP_AF1
#define MFP_PIN_GPIO116_AF_KP_DKIN_1			MFP_AF2

/* Pin GPIO117 alternate function codes */
#define MFP_PIN_GPIO117_AF_GPIO_117			MFP_AF0
#define MFP_PIN_GPIO117_AF_KP_MKIN_2			MFP_AF1
#define MFP_PIN_GPIO117_AF_KP_DKIN_2			MFP_AF2

/* Pin GPIO118 alternate function codes */
#define MFP_PIN_GPIO118_AF_GPIO_118			MFP_AF0
#define MFP_PIN_GPIO118_AF_KP_MKIN_3			MFP_AF1
#define MFP_PIN_GPIO118_AF_KP_DKIN_3			MFP_AF2

/* Pin GPIO119 alternate function codes */
#define MFP_PIN_GPIO119_AF_GPIO_119			MFP_AF0
#define MFP_PIN_GPIO119_AF_KP_MKIN_4			MFP_AF1
#define MFP_PIN_GPIO119_AF_KP_DKIN_4			MFP_AF2

/* Pin GPIO120 alternate function codes */
#define MFP_PIN_GPIO120_AF_GPIO_120			MFP_AF0
#define MFP_PIN_GPIO120_AF_KP_MKIN_5			MFP_AF1
#define MFP_PIN_GPIO120_AF_KP_DKIN_5			MFP_AF2

/* Pin GPIO121 alternate function codes */
#define MFP_PIN_GPIO121_AF_GPIO_121			MFP_AF0
#define MFP_PIN_GPIO121_AF_KP_MKOUT_0			MFP_AF1
#define MFP_PIN_GPIO121_AF_KP_DKIN_6			MFP_AF2

/* Pin GPIO122 alternate function codes */
#define MFP_PIN_GPIO122_AF_GPIO_122			MFP_AF0
#define MFP_PIN_GPIO122_AF_KP_MKOUT_1			MFP_AF1
#define MFP_PIN_GPIO122_AF_KP_DKIN_5			MFP_AF2

/* Pin GPIO123 alternate function codes */
#define MFP_PIN_GPIO123_AF_GPIO_123			MFP_AF0
#define MFP_PIN_GPIO123_AF_KP_MKOUT_2			MFP_AF1
#define MFP_PIN_GPIO123_AF_KP_DKIN_4			MFP_AF2

/* Pin GPIO124 alternate function codes */
#define MFP_PIN_GPIO124_AF_GPIO_124			MFP_AF0
#define MFP_PIN_GPIO124_AF_KP_MKOUT_3			MFP_AF1
#define MFP_PIN_GPIO124_AF_KP_DKIN_3			MFP_AF2

/* Pin GPIO125 alternate function codes */
#define MFP_PIN_GPIO125_AF_GPIO_125			MFP_AF0
#define MFP_PIN_GPIO125_AF_KP_MKOUT_4			MFP_AF1
#define MFP_PIN_GPIO125_AF_KP_MKIN_2			MFP_AF2

/* Pin GPIO126 alternate function codes */
#define MFP_PIN_GPIO126_AF_GPIO_126			MFP_AF0
#define MFP_PIN_GPIO126_AF_RTC_MVT			MFP_AF1
#define MFP_PIN_GPIO126_AF_OW_DQ			MFP_AF2
#define MFP_PIN_GPIO126_AF_EXT_CLK			MFP_AF3
#define MFP_PIN_GPIO126_AF_KP_MKOUT_7			MFP_AF4

/* Pin GPIO127 alternate function codes */
#define MFP_PIN_GPIO127_AF_GPIO_127			MFP_AF0
#define MFP_PIN_GPIO127_AF_LCD_nCS			MFP_AF1
#define MFP_PIN_GPIO127_AF_KP_DKIN_0			MFP_AF5
#define MFP_PIN_GPIO127_AF_CLK_BYPASS_GB		MFP_AF7

/* Pin GPIO0_2 alternate function codes */
#define MFP_PIN_GPIO0_2_AF_GPIO_0			MFP_AF0
#define MFP_PIN_GPIO0_2_AF_UHC_USBHPEN_MVT		MFP_AF1
#define MFP_PIN_GPIO0_2_AF_KP_DKIN_0			MFP_AF2
/*#define MFP_PIN_GPIO0_2_AF_ONE_WIRE			MFP_AF2 */

/* Pin GPIO1_2 alternate function codes */
#define MFP_PIN_GPIO1_2_AF_GPIO_0			MFP_AF0
#define MFP_PIN_GPIO1_2_AF_UHC_USBHPWR_MVT		MFP_AF1
#define MFP_PIN_GPIO1_2_AF_KP_DKIN_1			MFP_AF2

/* Pin GPIO2_2 alternate function codes */
#define MFP_PIN_GPIO2_2_AF_GPIO_2			MFP_AF0
#define MFP_PIN_GPIO2_2_AF_KP_MKIN_6			MFP_AF1
#define MFP_PIN_GPIO2_2_AF_KP_DKIN_6			MFP_AF2

/* Pin GPIO3_2 alternate function codes */
#define MFP_PIN_GPIO3_2_AF_GPIO_3			MFP_AF0
#define MFP_PIN_GPIO3_2_AF_KP_MKIN_7			MFP_AF1
#define MFP_PIN_GPIO3_2_AF_KP_DKIN_7			MFP_AF2

/* Pin GPIO4_2 alternate function codes */
#define MFP_PIN_GPIO4_2_AF_GPIO_4			MFP_AF0
#define MFP_PIN_GPIO4_2_AF_KP_MK0UT_5			MFP_AF1
#define MFP_PIN_GPIO4_2_AF_KP_DKIN_1			MFP_AF2

/* Pin GPIO5_2 alternate function codes */
#define MFP_PIN_GPIO5_2_AF_GPIO_5			MFP_AF0
#define MFP_PIN_GPIO5_2_AF_KP_MKOUT_6			MFP_AF1
#define MFP_PIN_GPIO5_2_AF_MK_DKIN_0			MFP_AF2

/* Pin GPIO6_2 alternate function codes */
#define MFP_PIN_GPIO6_2_AF_GPIO_6			MFP_AF0
#define MFP_PIN_GPIO6_2_AF_MK_MKOUT_7			MFP_AF1

/* Pin GPIO7_2 alternate function codes */
#define MFP_PIN_GPIO7_2_AF_GPIO_7			MFP_AF0
#define MFP_PIN_GPIO7_2_AF_MM3_DAT0			MFP_AF1

/* Pin GPIO8_2 alternate function codes */
#define MFP_PIN_GPIO8_2_AF_GPIO_8			MFP_AF0
#define MFP_PIN_GPIO8_2_AF_MM3_DAT1			MFP_AF1

/* Pin GPIO9_2 alternate function codes */
#define MFP_PIN_GPIO9_2_AF_GPIO_9			MFP_AF0
#define MFP_PIN_GPIO9_2_AF_MM3_DAT2			MFP_AF1

/* Pin GPIO10_2 alternate function codes */
#define MFP_PIN_GPIO10_2_AF_GPIO_10			MFP_AF0
#define MFP_PIN_GPIO10_2_AF_MM3_DAT3			MFP_AF1

/* Pin GPIO11_2 alternate function codes */
#define MFP_PIN_GPIO11_2_AF_GPIO_11			MFP_AF0
#define MFP_PIN_GPIO11_2_AF_MM3_CLK			MFP_AF1

/* Pin GPIO12_2 alternate function codes */
#define MFP_PIN_GPIO12_2_AF_GPIO_12			MFP_AF0
#define MFP_PIN_GPIO12_2_AF_MM3_CMD			MFP_AF1

/* Pin DF_CLE alternate function codes */
#define MFP_PIN_DF_CLE_AF_ND_CLE			MFP_AF0

/* Pin DF_ALE_nWE1 alternate function codes */
#define MFP_PIN_DF_ALE_nWE1_AF_CD_ADV1			MFP_AF0
#define MFP_PIN_DF_ALE_nWE1_AF_ND_ALE			MFP_AF1

/* Pin DF_SCLK_E alternate function codes */
#define MFP_PIN_DF_SCLK_E_AF_DF_SCLK_E			MFP_AF0

/* Pin DF_SCLK_S alternate function codes */
#define MFP_PIN_DF_SCLK_S_AF_DF_SCLK_S			MFP_AF0

/* Pin nBE0 alternate function codes */
#define MFP_PIN_nBE0_AF_DF_nBE0				MFP_AF0

/* Pin nBE1 alternate function codes */
#define MFP_PIN_nBE1_AF_DF_nBE1				MFP_AF0

/* Pin DF_INT_RnB alternate function codes */
#define MFP_PIN_DF_INT_RnB_AF_INT_RnB			MFP_AF0

/* Pin nLUA alternate function codes */
#define MFP_PIN_DF_nLUA_AF_DF_nLUA			MFP_AF0
#define MFP_PIN_DF_nLUA_AF_CD_ADV2			MFP_AF1

/* Pin nLLA alternate function codes */
#define MFP_PIN_DF_nLLA_AF_DF_nLLA			MFP_AF0
#define MFP_PIN_DF_nLLA_AF_CD_ADV1			MFP_AF1

/* Pin DF_nWE alternate function codes */
#define MFP_PIN_DF_nWE_AF_CD_WE				MFP_AF0
#define MFP_PIN_DF_nWE_AF_ND_WE				MFP_AF1

/* Pin DF_nRE alternate function codes */
#define MFP_PIN_DF_nRE_AF_CD_OE				MFP_AF0
#define MFP_PIN_DF_nRE_AF_ND_RE				MFP_AF1

/* Pin DF_ADDR0 alternate functino codes */
#define MFP_PIN_DF_ADDR0_AF_DF_ADDR0			MFP_AF0

/* Pin DF_ADDR1 alternate functino codes */
#define MFP_PIN_DF_ADDR1_AF_DF_ADDR1			MFP_AF0

/* Pin DF_ADDR2 alternate functino codes */
#define MFP_PIN_DF_ADDR2_AF_DF_ADDR2			MFP_AF0

/* Pin DF_ADDR3 alternate functino codes */
#define MFP_PIN_DF_ADDR3_AF_DF_ADDR3			MFP_AF0

/* Pin nCS0 alternate function codes */
#define MFP_PIN_nCS0_AF_nCS0				MFP_AF0
#define MFP_PIN_nCS0_AF_DF_XCVREN			MFP_AF1

/* Pin nCS1 alternate function codes */
#define MFP_PIN_nCS1_AF_nCS				MFP_AF0
#define MFP_PIN_nCS1_AF_DF_UNLOCK			MFP_AF1

/* Pin DF_nCS0 alternate function codes */
#define MFP_PIN_DF_nCS0_AF_DF_nCS0			MFP_AF0
#define MFP_PIN_DF_nCS0_AF_ND_nCS0			MFP_AF1

/* Pin DF_nCS1 alternate function codes */
#define MFP_PIN_DF_nCS1_AF_DF_nCS1			MFP_AF0
#define MFP_PIN_DF_nCS1_AF_ND_nCS1			MFP_AF1


/* Pin DF_IO15..0 alternate function codes */
/*   - Note that, in use, all 16 pins must have the same AF */
/*  Currently, only Alternate Function 1 is actually available */
/*  for these pins */

#define MFP_PIN_DF_IO_0_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_0_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_1_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_1_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_2_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_2_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_3_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_3_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_4_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_4_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_5_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_5_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_6_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_6_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_7_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_7_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_8_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_8_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_9_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_9_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_10_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_10_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_11_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_11_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_12_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_12_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_13_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_13_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_14_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_14_AF_ND				MFP_AF1

#define MFP_PIN_DF_IO_15_AF_DF				MFP_AF0
#define MFP_PIN_DF_IO_15_AF_ND				MFP_AF1

#endif

#define IS_GPIO_EXP_PIN(pin)         (MFP_OFFSET((pin)) == 0x0000)
#define MFP_PIN_GPIO128         ((0x0000 << 16) | (128))
#define MFP_PIN_GPIO129         ((0x0000 << 16) | (129))
#define MFP_PIN_GPIO130         ((0x0000 << 16) | (130))
#define MFP_PIN_GPIO131         ((0x0000 << 16) | (131))
#define MFP_PIN_GPIO132         ((0x0000 << 16) | (132))
#define MFP_PIN_GPIO133         ((0x0000 << 16) | (133))
#define MFP_PIN_GPIO134         ((0x0000 << 16) | (134))
#define MFP_PIN_GPIO135         ((0x0000 << 16) | (135))
#define MFP_PIN_GPIO136         ((0x0000 << 16) | (136))
#define MFP_PIN_GPIO137         ((0x0000 << 16) | (137))
#define MFP_PIN_GPIO138         ((0x0000 << 16) | (138))
#define MFP_PIN_GPIO139         ((0x0000 << 16) | (139))
#define MFP_PIN_GPIO140         ((0x0000 << 16) | (140))
#define MFP_PIN_GPIO141         ((0x0000 << 16) | (141))
#define MFP_PIN_GPIO142         ((0x0000 << 16) | (142))
#define MFP_PIN_GPIO143         ((0x0000 << 16) | (143))
#define MFP_PIN_GPIO144         ((0x0000 << 16) | (144))
#define MFP_PIN_GPIO145         ((0x0000 << 16) | (145))
#define MFP_PIN_GPIO146         ((0x0000 << 16) | (146))
#define MFP_PIN_GPIO147         ((0x0000 << 16) | (147))
#define MFP_PIN_GPIO148         ((0x0000 << 16) | (148))
#define MFP_PIN_GPIO149         ((0x0000 << 16) | (149))
#define MFP_PIN_GPIO150         ((0x0000 << 16) | (150))
#define MFP_PIN_GPIO151         ((0x0000 << 16) | (151))
#define MFP_PIN_GPIO152         ((0x0000 << 16) | (152))
#define MFP_PIN_GPIO153         ((0x0000 << 16) | (153))
#define MFP_PIN_GPIO154         ((0x0000 << 16) | (154))
#define MFP_PIN_GPIO155         ((0x0000 << 16) | (155))
#define MFP_PIN_GPIO156         ((0x0000 << 16) | (156))
#define MFP_PIN_GPIO157         ((0x0000 << 16) | (157))
#define MFP_PIN_GPIO158         ((0x0000 << 16) | (158))
#define MFP_PIN_GPIO159         ((0x0000 << 16) | (159))

#define MFP_PIN_GPIO128_GPIO_128			MFP_AF0
#define MFP_PIN_GPIO129_GPIO_129			MFP_AF0
#define MFP_PIN_GPIO130_GPIO_130			MFP_AF0
#define MFP_PIN_GPIO131_GPIO_131			MFP_AF0
#define MFP_PIN_GPIO132_GPIO_132			MFP_AF0
#define MFP_PIN_GPIO133_GPIO_133			MFP_AF0
#define MFP_PIN_GPIO134_GPIO_134			MFP_AF0
#define MFP_PIN_GPIO135_GPIO_135			MFP_AF0
#define MFP_PIN_GPIO136_GPIO_136			MFP_AF0
#define MFP_PIN_GPIO137_GPIO_137			MFP_AF0
#define MFP_PIN_GPIO138_GPIO_138			MFP_AF0
#define MFP_PIN_GPIO139_GPIO_139			MFP_AF0
#define MFP_PIN_GPIO140_GPIO_140			MFP_AF0
#define MFP_PIN_GPIO141_GPIO_141			MFP_AF0
#define MFP_PIN_GPIO142_GPIO_142			MFP_AF0
#define MFP_PIN_GPIO143_GPIO_143			MFP_AF0
#define MFP_PIN_GPIO144_GPIO_144			MFP_AF0
#define MFP_PIN_GPIO145_GPIO_145			MFP_AF0
#define MFP_PIN_GPIO146_GPIO_146			MFP_AF0
#define MFP_PIN_GPIO147_GPIO_147			MFP_AF0
#define MFP_PIN_GPIO148_GPIO_148			MFP_AF0
#define MFP_PIN_GPIO149_GPIO_149			MFP_AF0
#define MFP_PIN_GPIO150_GPIO_150			MFP_AF0
#define MFP_PIN_GPIO151_GPIO_151			MFP_AF0
#define MFP_PIN_GPIO152_GPIO_152			MFP_AF0
#define MFP_PIN_GPIO153_GPIO_153			MFP_AF0
#define MFP_PIN_GPIO154_GPIO_154			MFP_AF0
#define MFP_PIN_GPIO155_GPIO_155			MFP_AF0
#define MFP_PIN_GPIO156_GPIO_156			MFP_AF0
#define MFP_PIN_GPIO157_GPIO_157			MFP_AF0
#define MFP_PIN_GPIO158_GPIO_158			MFP_AF0
#define MFP_PIN_GPIO159_GPIO_159			MFP_AF0

extern int mhn_mfp_set_config (struct mhn_pin_config *pin_config);
extern int mhn_mfp_set_configs(struct mhn_pin_config *pin_configs, int n);

extern int mhn_mfp_set_afds(mfp_pin_t pin, int af, int ds);
extern int mhn_mfp_set_lpm (mfp_pin_t pin, int lpm);
extern int mhn_mfp_set_rdh (mfp_pin_t pin, int rdh);
extern int mhn_mfp_set_edge(mfp_pin_t pin, int edge);
extern int mhn_mfp_set_pull(mfp_pin_t pin, int pull);

extern void mhn_mfp_save(void);
extern void mhn_mfp_restore(void);

#endif /* _MHN_MFP_H */
