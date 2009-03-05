/*
 * linux/drivers/usb/host/ohci-omap.h
 *
 * OMAP OHCI USB controller specific defines
 */

/* OMAP USB OHCI common defines */
#define OMAP_OHCI_NAME		"omap-ohci"
#define OMAP_OHCI_BASE		0xfffba000
#define OMAP_OHCI_SIZE		4096

#define HMC_CLEAR		(0x3f << 1)
#define APLL_NDPLL_SWITCH	0x0001
#define DPLL_PLL_ENABLE		0x0010
#define DPLL_LOCK		0x0001
#define SOFT_REQ_REG_REQ	0x0001
#define USB_MCLK_EN		0x0010
#define USB_HOST_HHC_UHOST_EN	0x00000200
#define SOFT_USB_OTG_REQ	(1 << 8)
#define SOFT_USB_REQ		(1 << 3)
#define STATUS_REQ_REG		0xfffe0840
#define USB_HOST_DPLL_REQ	(1 << 8)
#define SOFT_DPLL_REQ		(1 << 0)

/* OMAP-1510 USB OHCI defines */
#define OMAP1510_LB_MEMSIZE	32		/* Should be same as SDRAM size */
#define OMAP1510_LB_CLOCK_DIV	0xfffec10c
#define OMAP1510_LB_MMU_CTL	0xfffec208	
#define OMAP1510_LB_MMU_LCK	0xfffec224
#define OMAP1510_LB_MMU_LD_TLB	0xfffec228
#define OMAP1510_LB_MMU_CAM_H	0xfffec22c
#define OMAP1510_LB_MMU_CAM_L	0xfffec230
#define OMAP1510_LB_MMU_RAM_H	0xfffec234
#define OMAP1510_LB_MMU_RAM_L	0xfffec238


// #define	HMC_1510	((MOD_CONF_CTRL_0_REG >> 1) & 0x3f)
#define	HMC_1610	(OTG_SYSCON_2_REG & 0x3f)
#define	HMC		 HMC_1610

