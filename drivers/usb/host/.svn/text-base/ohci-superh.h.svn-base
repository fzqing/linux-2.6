/*
 * OHCI HCD (Host Controller Driver) for SuperH internal USB.
 *
 * Copyright (C)  Kuniki Nakamura  (Dec 25, 2004)
 *   Modified to support SH7727 USB Host Controller
 * Copyright (C)  Takashi Kusuda   (Nov  1, 2004)
 *   Modified to support SH7720 USB Host Controller
 *
 *  This file is licenced under the GPL.
 */

#include <asm/irq.h>

#if defined(CONFIG_CPU_SUBTYPE_SH7720)
/* Data & Address for setup register of SH7720 USB */
#define SH_OHCI_REGS_BASE	0xA4428000UL
#define SH_OHCI_IRQ		INTEVT2_TO_IRQ(0xA60) /* 67 */

#define UCLKCR_ADDR		0xA40A0008
#define UCLKCR_DATA		0x5AC0		/* External Clock/Stanby CLK off */

#define UTRCTL_ADDR		0xA405012C
#if 0
#define UTRCTL_DATA		0x0001		/* port1 = Function */
#else
#define UTRCTL_DATA		0x0000		/* port1 = Host */
#endif

#define PHCR_MASK		0xFFF0		/* pwr_en */

#define STBCR3_ADDR		0xA40A0000
#define USBH_OFF		0x02
#define USBH_ON			~(0x02)

#elif defined(CONFIG_CPU_SUBTYPE_SH7727)
/* Data & Address for setup register of SH7727 USB */
#define SH_OHCI_REGS_BASE	0xA4000400UL
#define SH_OHCI_IRQ		INTEVT2_TO_IRQ(0xA00) /* 64 */

#define EXCPGCR_ADDR		0xA4000236
#if defined(CONFIG_SH_SOLUTION_ENGINE_2ND)
#define EXCPGCR_DATA		0x28	/* CKIO(48MHz) */
#else
#define EXCPGCR_DATA		0x30	/* External Clock */
#endif

#define EXPFC_ADDR		0xA4000234
#define EXPFC_DATA		0x0001	/* port1 -> USB Function */

#define PDCR_MASK		0xcfff  /* PD6MD(1,0) = 0 */
#define PECR_MASK1		0xF0C3
#define PECR_MASK2		0x0A00

#define STBCR3			0xA4000230
#define USBH_OFF		0x08
#define USBH_ON			~(0x08)

#define SH_OHCI_DESCRIPTOR_A	(SH_OHCI_REGS_BASE + 0x48)
#define SH_OHCI_DESCRIPTOR_B	(SH_OHCI_REGS_BASE + 0x4C)
#define SH_OHCI_PORT_STATUS1	(SH_OHCI_REGS_BASE + 0x54)
#define SH_OHCI_PORT_STATUS2	(SH_OHCI_REGS_BASE + 0x58)

#endif

