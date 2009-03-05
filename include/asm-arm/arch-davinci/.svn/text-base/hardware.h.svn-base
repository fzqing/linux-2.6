/*
 *  linux/include/asm-arm/arch-davinci/hardware.h
 *
 *  Copyright (C) 2006 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <asm/sizes.h>
#include <asm/memory.h>
#include <asm/arch/io.h>

/*
 * DaVinci (DM644X) Base register addresses
 */
#define DAVINCI_DMA_3PCC_BASE			(0x01C00000)
#define DAVINCI_DMA_3PTC0_BASE			(0x01C10000)
#define DAVINCI_DMA_3PTC1_BASE			(0x01C10400)
#define DAVINCI_UART0_BASE			(0x01C20000)
#define DAVINCI_UART1_BASE			(0x01C20400)
#define DAVINCI_I2C_BASE			(0x01C21000)
#define DAVINCI_TIMER0_BASE			(0x01C21400)
#define DAVINCI_TIMER1_BASE			(0x01C21800)
#define DAVINCI_WDOG_BASE			(0x01C21C00)
#define DAVINCI_PWM0_BASE			(0x01C22000)
#define DAVINCI_PWM1_BASE			(0x01C22400)
#define DAVINCI_PWM2_BASE			(0x01C22800)
#define DAVINCI_SYSTEM_MODULE_BASE		(0x01C40000)
#define DAVINCI_PLL_CNTRL0_BASE			(0x01C40800)
#define DAVINCI_PLL_CNTRL1_BASE			(0x01C40C00)
#define DAVINCI_PWR_SLEEP_CNTRL_BASE		(0x01C41000)
#define DAVINCI_SYSTEM_DFT_BASE			(0x01C42000)
#define DAVINCI_ARM_INTC_BASE			(0x01C48000)
#define DAVINCI_IEEE1394_BASE			(0x01C60000)
#define DAVINCI_USB_OTG_BASE			(0x01C64000)
#define DAVINCI_CFC_ATA_BASE			(0x01C66000)
#define DAVINCI_SPI_BASE			(0x01C66800)
#define DAVINCI_GPIO_BASE			(0x01C67000)
#define DAVINCI_UHPI_BASE			(0x01C67800)
#define DAVINCI_VPSS_REGS_BASE			(0x01C70000)
#define DAVINCI_EMAC_CNTRL_REGS_BASE		(0x01C80000)
#define DAVINCI_EMAC_WRAPPER_CNTRL_REGS_BASE	(0x01C81000)
#define DAVINCI_EMAC_WRAPPER_RAM_BASE		(0x01C82000)
#define DAVINCI_MDIO_CNTRL_REGS_BASE		(0x01C84000)
#define DAVINCI_IMCOP_BASE			(0x01CC0000)
#define DAVINCI_VLYNQ_BASE			(0x01E01000)
#define DAVINCI_MCBSP_BASE			(0x01E02000)
#define DAVINCI_MCBSP_RX_DMA_PORT		DAVINCI_MCBSP_BASE
#define DAVINCI_MCBSP_TX_DMA_PORT		(DAVINCI_MCBSP_BASE + 4)
#define DAVINCI_MCBSP0_BASE			DAVINCI_MCBSP_BASE
#define DAVINCI_MCBSP1_BASE			(0x01E04000)
#define DAVINCI_MMC_SD_BASE			(0x01E10000)
#define DAVINCI_MS_BASE				(0x01E20000)
#define DAVINCI_ASYNC_EMIF_DATA_CE0_BASE	(0x02000000)
#define DAVINCI_ASYNC_EMIF_DATA_CE1_BASE	(0x04000000)
#define DAVINCI_ASYNC_EMIF_DATA_CE2_BASE	(0x06000000)
#define DAVINCI_ASYNC_EMIF_DATA_CE3_BASE	(0x08000000)
#define DAVINCI_VLYNQ_REMOTE_BASE		(0x0C000000)

#define DM644X_ASYNC_EMIF_CNTRL_BASE		(0x01E00000)
#define DM644X_UART2_BASE			(0x01C20800)
#define DM644X_DDR2_CNTL_BASE			(0x20000000)

#define DM355_MMC1_BASE				(0x01E00000)
#define DM355_UART2_BASE			(0x01E06000)
#define DM355_ASYNC_EMIF_CNTRL_BASE		(0x01E10000)
#define DM355_MMC0_BASE				(0x01E11000)


/* Power and Sleep Controller (PSC) Domains */
#define DAVINCI_GPSC_ARMDOMAIN      0
#define DAVINCI_GPSC_DSPDOMAIN      1

/*
 * System module registers
 */
#define PINMUX0			(DAVINCI_SYSTEM_MODULE_BASE + 0x00)
#define PINMUX1			(DAVINCI_SYSTEM_MODULE_BASE + 0x04)
/* dm355 only */
#define PINMUX2			(DAVINCI_SYSTEM_MODULE_BASE + 0x08)
#define PINMUX3			(DAVINCI_SYSTEM_MODULE_BASE + 0x0c)
#define PINMUX4			(DAVINCI_SYSTEM_MODULE_BASE + 0x10)

/*
 * LPSC Assignments
 */
#define DAVINCI_LPSC_VPSSMSTR       0       /* VPSS Master LPSC */
#define DAVINCI_LPSC_VPSSSLV        1       /* VPSS Slave LPSC */
#define DAVINCI_LPSC_TPCC           2       /* TPCC LPSC */
#define DAVINCI_LPSC_TPTC0          3       /* TPTC0 LPSC */
#define DAVINCI_LPSC_TPTC1          4       /* TPTC1 LPSC */
#define DAVINCI_LPSC_EMAC           5       /* EMAC LPSC */
#define DAVINCI_LPSC_EMAC_WRAPPER   6       /* EMAC WRAPPER LPSC */
#define DAVINCI_LPSC_MDIO           7       /* MDIO LPSC */
#define DAVINCI_LPSC_MMC_SD1        7
#define DAVINCI_LPSC_IEEE1394       8       /* IEEE1394 LPSC */
#define DAVINCI_LPSC_McBSP1         8       /* McBSP1 LPSC */
#define DAVINCI_LPSC_USB            9       /* USB LPSC */
#define DAVINCI_LPSC_ATA            10      /* ATA LPSC */
#define DAVINCI_LPSC_PWM3           10      /* PWM3 LPSC */
#define DAVINCI_LPSC_VLYNQ          11      /* VLYNQ LPSC */
#define DAVINCI_LPSC_UHPI           12      /* UHPI LPSC */
#define DAVINCI_LPSC_DDR_EMIF       13      /* DDR_EMIF LPSC */
#define DAVINCI_LPSC_AEMIF          14      /* AEMIF LPSC */
#define DAVINCI_LPSC_MMC_SD0        15      /* MMC_SD0 LPSC */
#define DAVINCI_LPSC_MEMSTICK       16      /* MEMSTICK LPSC */
#define DAVINCI_LPSC_McBSP0         17      /* McBSP0 LPSC */
#define DAVINCI_LPSC_I2C            18      /* I2C LPSC */
#define DAVINCI_LPSC_UART0          19      /* UART0 LPSC */
#define DAVINCI_LPSC_UART1          20      /* UART1 LPSC */
#define DAVINCI_LPSC_UART2          21      /* UART2 LPSC */
#define DAVINCI_LPSC_SPI            22      /* SPI LPSC */
#define DAVINCI_LPSC_PWM0           23      /* PWM0 LPSC */
#define DAVINCI_LPSC_PWM1           24      /* PWM1 LPSC */
#define DAVINCI_LPSC_PWM2           25      /* PWM2 LPSC */
#define DAVINCI_LPSC_GPIO           26      /* GPIO LPSC */
#define DAVINCI_LPSC_TIMER0         27      /* TIMER0 LPSC */
#define DAVINCI_LPSC_TIMER1         28      /* TIMER1 LPSC */
#define DAVINCI_LPSC_TIMER2         29      /* TIMER2 LPSC */
#define DAVINCI_LPSC_SYSTEM_SUBSYS  30      /* SYSTEM SUBSYSTEM LPSC */
#define DAVINCI_LPSC_ARM            31      /* ARM LPSC */
#define DAVINCI_LPSC_SCR2           32      /* SCR2 LPSC */
#define DAVINCI_LPSC_SCR3           33      /* SCR3 LPSC */
#define DAVINCI_LPSC_SCR4           34      /* SCR4 LPSC */
#define DAVINCI_LPSC_CROSSBAR       35      /* CROSSBAR LPSC */
#define DAVINCI_LPSC_CFG27          36      /* CFG27 LPSC */
#define DAVINCI_LPSC_CFG3           37      /* CFG3 LPSC */
#define DAVINCI_LPSC_CFG5           38      /* CFG5 LPSC */
#define DAVINCI_LPSC_GEM            39      /* GEM LPSC */
#define DAVINCI_LPSC_IMCOP          40      /* IMCOP LPSC */

/*
 * DM6467 base register addresses different from origina DaVinci
 */
#define DAVINCI_DM646X_DMA_3PTC2_BASE             (0x01C10800)
#define DAVINCI_DM646X_DMA_3PTC3_BASE             (0x01C10C00)
#define DAVINCI_DM646X_VIDEO_PORT_BASE            (0x01C12000)
#define DAVINCI_DM646X_VDCE_BASE                  (0x01C12800)
#define DAVINCI_DM646X_TSIF0_BASE                 (0x01C13000)
#define DAVINCI_DM646X_TSIF1_BASE                 (0x01C13400)
#define DAVINCI_DM646X_PCI_CTL_BASE               (0x01C1A000)
#define DAVINCI_DM646X_CRGEN0_BASE                (0x01C26000)
#define DAVINCI_DM646X_CRGEN1_BASE                (0x01C26400)
#define DAVINCI_DM646X_SEC_CONTROLLER_BASE        (0x01C40400)
#define DAVINCI_DM646X_MCASP0_REG_BASE            (0x01D01000)
#define DAVINCI_DM646X_MCASP0_DATA_PORT_BASE      (0x01D01400)
#define DAVINCI_DM646X_MCASP1_REG_BASE            (0x01D01800)
#define DAVINCI_DM646X_MCASP1_DATA_PORT_BASE      (0x01D01C00)
#define DAVINCI_DM646X_HDVICP0_BASE               (0x02000000)
#define DAVINCI_DM646X_HDVICP1_BASE               (0x02200000)
#define DAVINCI_DM646X_ASYNC_EMIF_CNTRL_BASE      (0x20008000)
#define DAVINCI_DM646X_VLYNQ_BASE                 (0x20010000)
#define DAVINCI_DM646X_ASYNC_EMIF_DATA_CE0_BASE   (0x42000000)
#define DAVINCI_DM646X_ASYNC_EMIF_DATA_CE1_BASE   (0x44000000)
#define DAVINCI_DM646X_ASYNC_EMIF_DATA_CE2_BASE   (0x46000000)
#define DAVINCI_DM646X_ASYNC_EMIF_DATA_CE3_BASE   (0x48000000)
#define DAVINCI_DM646X_VLYNQ_REMOTE_BASE          (0x4C000000)

/*
 * LPSC Assignments
 */
#define DAVINCI_DM646X_LPSC_RESERVED       0       /* Reserved */
#define DAVINCI_DM646X_LPSC_C64X_CPU       1       /* C64x + CPU */
#define DAVINCI_DM646X_LPSC_HDVICP0        2       /* HDVICP0 */
#define DAVINCI_DM646X_LPSC_HDVICP1        3       /* HDVICP1 */
#define DAVINCI_DM646X_LPSC_TPCC           4       /* TPCC LPSC */
#define DAVINCI_DM646X_LPSC_TPTC0          5       /* TPTC0 LPSC */
#define DAVINCI_DM646X_LPSC_TPTC1          6       /* TPTC1 LPSC */
#define DAVINCI_DM646X_LPSC_TPTC2          7       /* TPTC2 LPSC */
#define DAVINCI_DM646X_LPSC_TPTC3          8       /* TPTC3 LPSC */
#define DAVINCI_DM646X_LPSC_PCI            13      /* PCI LPSC */
#define DAVINCI_DM646X_LPSC_EMAC           14      /* EMAC LPSC */
#define DAVINCI_DM646X_LPSC_VDCE           15      /* VDCE LPSC */
#define DAVINCI_DM646X_LPSC_VPSSMSTR       16      /* VPSS Master LPSC */
#define DAVINCI_DM646X_LPSC_VPSSSLV        17      /* VPSS Slave LPSC */
#define DAVINCI_DM646X_LPSC_TSIF0          18      /* TSIF0 LPSC */
#define DAVINCI_DM646X_LPSC_TSIF1          19      /* TSIF1 LPSC */
#define DAVINCI_DM646X_LPSC_DDR_EMIF       20      /* DDR_EMIF LPSC */
#define DAVINCI_DM646X_LPSC_AEMIF          21      /* AEMIF LPSC */
#define DAVINCI_DM646X_LPSC_McASP0         22      /* McASP0 LPSC */
#define DAVINCI_DM646X_LPSC_McASP1         23      /* McASP1 LPSC */
#define DAVINCI_DM646X_LPSC_CRGEN0         24      /* CRGEN0 LPSC */
#define DAVINCI_DM646X_LPSC_CRGEN1         25      /* CRGEN1 LPSC */
#define DAVINCI_DM646X_LPSC_UART0          26      /* UART0 LPSC */
#define DAVINCI_DM646X_LPSC_UART1          27      /* UART1 LPSC */
#define DAVINCI_DM646X_LPSC_UART2          28      /* UART2 LPSC */
#define DAVINCI_DM646X_LPSC_PWM0           29      /* PWM0 LPSC */
#define DAVINCI_DM646X_LPSC_PWM1           30      /* PWM1 LPSC */
#define DAVINCI_DM646X_LPSC_I2C            31      /* I2C LPSC */
#define DAVINCI_DM646X_LPSC_SPI            32      /* SPI LPSC */
#define DAVINCI_DM646X_LPSC_GPIO           33      /* GPIO LPSC */
#define DAVINCI_DM646X_LPSC_TIMER0         34      /* TIMER0 LPSC */
#define DAVINCI_DM646X_LPSC_TIMER1         35      /* TIMER1 LPSC */
#define DAVINCI_DM646X_LPSC_ARM_INTC       45      /* ARM INTC LPSC */

/*
 * DM355 base register addresses different from origina DaVinci
 */
#define DAVINCI_DM355_TIMER2_BASE                (0x01C20800)
#define DAVINCI_DM355_REALTIME_BASE              (0x01C20C00)
#define DAVINCI_DM355_PWM3_BASE                  (0x01C22C00)
#define DAVINCI_DM355_SPI_BASE                   (0x01C66000)
#define DAVINCI_DM355_SPI0_BASE                  DAVINCI_DM355_SPI_BASE
#define DAVINCI_DM355_SPI1_BASE                  (0x01C66800)
#define DAVINCI_DM355_SPI2_BASE                  (0x01C67800)
#define DAVINCI_DM355_VPSS_CLK_BASE              DAVINCI_VPSS_REGS_BASE
#define DAVINCI_DM355_VPSS_HW3A_BASE             (0x01C70080)
#define DAVINCI_DM355_VPSS_IPIPE0_BASE           (0x01C70100)
#define DAVINCI_DM355_VPSS_OSD_BASE              (0x01C70200)
#define DAVINCI_DM355_VPSS_HSSIF_BASE            (0x01C70300)
#define DAVINCI_DM355_VPSS_VENC_BASE             (0x01C70400)
#define DAVINCI_DM355_VPSS_CCDC_BASE             (0x01C70600)
#define DAVINCI_DM355_VPSS_BL_BASE               (0x01C70800)
#define DAVINCI_DM355_VPSS_CFA_LD_BASE           (0x01C70900)
#define DAVINCI_DM355_VPSS_IPIPE1_BASE           (0x01C71000)
#define DAVINCI_DM355_IMX_BASE                   (0x01CD0000)
#define DAVINCI_DM355_IMX_CTL_BASE               (0x01CD0380)
#define DAVINCI_DM355_IMCOP_CTL_BASE             (0x01CDF400)
#define DAVINCI_DM355_IMCOP_SEQ_BASE             (0x01CDFF00)
#define DAVINCI_DM355_MMC_SD1_BASE               (0x01E00000)
#define DAVINCI_DM355_MCBSP0_BASE                DAVINCI_MCBSP_BASE
#define DAVINCI_DM355_MCBSP1_BASE                (0x01E04000)
#define DAVINCI_DM355_MCBSP1_RX_DMA_PORT         DAVINCI_DM355_MCBSP1_BASE
#define DAVINCI_DM355_MCBSP1_TX_DMA_PORT         (DAVINCI_DM355_MCBSP1_BASE + 4)
#define DAVINCI_DM355_UART2_BASE                 (0x01E06000)
#define DAVINCI_DM355_ASYNC_EMIF_CNTRL_BASE      (0x01E10000)

#define DAVINCI_DM355_MMC_SD_BASE                (0x01E11000)
#define DAVINCI_DM355_MMC_SD0_BASE               DAVINCI_DM355_MMC_SD_BASE
#define DAVINCI_MMC_SD0_BASE                     DAVINCI_DM355_MMC_SD_BASE

/*
 * Macro to access device power control
 */
#define DAVINCI_VDD3P3V_PWDN		__REG(DAVINCI_SYSTEM_MODULE_BASE + 0x48)

/*
 * We can have multiple VLYNQ IPs in our system.
 * Define 'LOW_VLYNQ_CONTROL_BASE' with the VLYNQ
 * IP having lowest base address.
 * Define 'HIGH_VLYNQ_CONTROL_BASE' with the VLYNQ
 * IP having highest base address.
 * In case of only one VLYNQ IP, define only the
 * 'LOW_VLYNQ_CONTROL_BASE'.
 */
#define LOW_VLYNQ_CONTROL_BASE		DAVINCI_VLYNQ_BASE


#endif /* __ASM_ARCH_HARDWARE_H */
