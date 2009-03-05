/*
 * arch/ppc/platforms/4xx/virtex.c
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002-2005 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/xilinx_devices.h>
#include <asm/ocp.h>
#include <platforms/4xx/virtex.h>

/* Have OCP take care of the serial ports. */
struct ocp_def core_ocp[] = {
#ifdef XPAR_UARTNS550_0_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 0,
	  .paddr	= XPAR_UARTNS550_0_BASEADDR,
	  .irq		= XPAR_INTC_0_UARTNS550_0_VEC_ID,
	  .pm		= OCP_CPM_NA
	},
#ifdef XPAR_UARTNS550_1_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 1,
	  .paddr	= XPAR_UARTNS550_1_BASEADDR,
	  .irq		= XPAR_INTC_0_UARTNS550_1_VEC_ID,
	  .pm		= OCP_CPM_NA
	},
#ifdef XPAR_UARTNS550_2_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 2,
	  .paddr	= XPAR_UARTNS550_2_BASEADDR,
	  .irq		= XPAR_INTC_0_UARTNS550_2_VEC_ID,
	  .pm		= OCP_CPM_NA
	},
#ifdef XPAR_UARTNS550_3_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 3,
	  .paddr	= XPAR_UARTNS550_3_BASEADDR,
	  .irq		= XPAR_INTC_0_UARTNS550_3_VEC_ID,
	  .pm		= OCP_CPM_NA
	},
#ifdef XPAR_UARTNS550_4_BASEADDR
#error Edit this file to add more devices.
#endif			/* 4 */
#endif			/* 3 */
#endif			/* 2 */
#endif			/* 1 */
#endif			/* 0 */
	{ .vendor	= OCP_VENDOR_INVALID
	}
};

/* Xilinx Virtex-II Pro device descriptions */

#ifdef XPAR_EMAC_0_BASEADDR

static struct xemac_platform_data xemac_0_pdata = {
	.device_flags = (XPAR_EMAC_0_ERR_COUNT_EXIST ? XEMAC_HAS_ERR_COUNT : 0) |
		(XPAR_EMAC_0_MII_EXIST ? XEMAC_HAS_MII : 0) |
		(XPAR_EMAC_0_CAM_EXIST ? XEMAC_HAS_CAM : 0) |
		(XPAR_EMAC_0_JUMBO_EXIST ? XEMAC_HAS_JUMBO : 0),
	.dma_mode = XPAR_EMAC_0_DMA_PRESENT
};

static struct platform_device xilinx_emac_0_device = {
	.name = "xilinx_emac",
	.id = XPAR_EMAC_0_DEVICE_ID,
	.dev.platform_data = &xemac_0_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_EMAC_0_BASEADDR,
			.end	= XPAR_EMAC_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_EMAC_0_VEC_ID,
			.end	= XPAR_INTC_0_EMAC_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_EMAC_0_BASEADDR */

#ifdef XPAR_EMAC_1_BASEADDR

static struct xemac_platform_data xemac_1_pdata = {
	.device_flags = (XPAR_EMAC_1_ERR_COUNT_EXIST ? XEMAC_HAS_ERR_COUNT : 0) |
		(XPAR_EMAC_1_MII_EXIST ? XEMAC_HAS_MII : 0) |
		(XPAR_EMAC_1_CAM_EXIST ? XEMAC_HAS_CAM : 0) |
		(XPAR_EMAC_1_JUMBO_EXIST ? XEMAC_HAS_JUMBO : 0),
	.dma_mode = XPAR_EMAC_1_DMA_PRESENT
};

static struct platform_device xilinx_emac_1_device = {
	.name = "xilinx_emac",
	.id = XPAR_EMAC_1_DEVICE_ID,
	.dev.platform_data = &xemac_1_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_EMAC_1_BASEADDR,
			.end	= XPAR_EMAC_1_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_EMAC_1_VEC_ID,
			.end	= XPAR_INTC_0_EMAC_1_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_EMAC_1_BASEADDR */

#ifdef XPAR_TEMAC_0_BASEADDR

static struct xtemac_platform_data xtemac_0_pdata = {
	.dma_mode = XPAR_TEMAC_0_DMA_TYPE,
	.rx_pkt_fifo_depth = XPAR_TEMAC_0_IPIF_RDFIFO_DEPTH,
	.tx_pkt_fifo_depth = XPAR_TEMAC_0_IPIF_WRFIFO_DEPTH,
	.mac_fifo_depth = XPAR_TEMAC_0_MAC_FIFO_DEPTH,
	.dcr_host = XPAR_TEMAC_0_TEMAC_DCR_HOST,
	.dre = XPAR_TEMAC_0_INCLUDE_DRE
};

static struct platform_device xilinx_temac_0_device = {
	.name = "xilinx_temac",
	.id = XPAR_TEMAC_0_DEVICE_ID,
	.dev.platform_data = &xtemac_0_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_TEMAC_0_BASEADDR,
			.end	= XPAR_TEMAC_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_TEMAC_0_VEC_ID,
			.end	= XPAR_INTC_0_TEMAC_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_TEMAC_0_BASEADDR */

#ifdef XPAR_TEMAC_1_BASEADDR

static struct xtemac_platform_data xtemac_1_pdata = {
	.dma_mode = XPAR_TEMAC_1_DMA_TYPE,
	.rx_pkt_fifo_depth = XPAR_TEMAC_1_IPIF_RDFIFO_DEPTH,
	.tx_pkt_fifo_depth = XPAR_TEMAC_1_IPIF_WRFIFO_DEPTH,
	.mac_fifo_depth = XPAR_TEMAC_1_MAC_FIFO_DEPTH,
	.dcr_host = XPAR_TEMAC_1_TEMAC_DCR_HOST,
	.dre = XPAR_TEMAC_1_INCLUDE_DRE
};

static struct platform_device xilinx_temac_1_device = {
	.name = "xilinx_temac",
	.id = XPAR_TEMAC_1_DEVICE_ID,
	.dev.platform_data = &xtemac_1_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_TEMAC_1_BASEADDR,
			.end	= XPAR_TEMAC_1_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_TEMAC_1_VEC_ID,
			.end	= XPAR_INTC_0_TEMAC_1_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_TEMAC_1_BASEADDR */

#ifdef XPAR_PS2_0_BASEADDR

static struct platform_device xilinx_ps2_0_device = {
	.name = "xilinx_ps2",
	.id = 0,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_PS2_0_BASEADDR,
			.end	= XPAR_PS2_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_PS2_0_VEC_ID,
			.end	= XPAR_INTC_0_PS2_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_PS2_0_BASEADDR */

#ifdef XPAR_PS2_1_BASEADDR

static struct platform_device xilinx_ps2_1_device = {
	.name = "xilinx_ps2",
	.id = 1,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_PS2_1_BASEADDR,
			.end	= XPAR_PS2_1_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_PS2_1_VEC_ID,
			.end	= XPAR_INTC_0_PS2_1_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_PS2_1_BASEADDR */

#ifdef XPAR_TFT_0_BASEADDR

static struct resource xilinx_lcd_0_resource = {
	.start = XPAR_TFT_0_BASEADDR,
	.end = XPAR_TFT_0_BASEADDR+7,
	.flags = IORESOURCE_MEM
};

static struct platform_device xilinx_lcd_0_device = {
	.name = "xilinx_fb",
	.id = 0,
	.num_resources = 1,
	.resource = &xilinx_lcd_0_resource
};

#endif /* XPAR_TFT_0_BASEADDR */

#ifdef XPAR_SYSACE_0_BASEADDR

static struct platform_device xilinx_sysace_0_device = {
	.name = "xilinx_sysace",
	.id = 0,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_SYSACE_0_BASEADDR,
			.end	= XPAR_SYSACE_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_SYSACE_0_VEC_ID,
			.end	= XPAR_INTC_0_SYSACE_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_SYSACE_0_BASEADDR */

#ifdef XPAR_IIC_0_BASEADDR

static struct platform_device xilinx_iic_0_device = {
	.name = "xilinx_iic",
	.id = 0,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_IIC_0_BASEADDR,
			.end	= XPAR_IIC_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_IIC_0_VEC_ID,
			.end	= XPAR_INTC_0_IIC_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_IIC_0_BASEADDR */

#ifdef XPAR_GPIO_0_BASEADDR

static struct platform_device xilinx_gpio_0_device = {
	.name = "xilinx_gpio",
	.id = XPAR_GPIO_0_DEVICE_ID,
	.dev.platform_data = (XPAR_GPIO_0_IS_DUAL ? XGPIO_IS_DUAL : 0),
	.num_resources = 1,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_GPIO_0_BASEADDR,
			.end	= XPAR_GPIO_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		}
	}
};

#endif /* XPAR_GPIO_0_BASEADDR */

#ifdef XPAR_GPIO_1_BASEADDR

static struct platform_device xilinx_gpio_1_device = {
	.name = "xilinx_gpio",
	.id = XPAR_GPIO_1_DEVICE_ID,
	.dev.platform_data = (XPAR_GPIO_1_IS_DUAL ? XGPIO_IS_DUAL : 0),
	.num_resources = 1,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_GPIO_1_BASEADDR,
			.end	= XPAR_GPIO_1_HIGHADDR,
			.flags	= IORESOURCE_MEM
		}
	}
};

#endif /* XPAR_GPIO_1_BASEADDR */

#ifdef XPAR_GPIO_2_BASEADDR

static struct platform_device xilinx_gpio_2_device = {
	.name = "xilinx_gpio",
	.id = XPAR_GPIO_2_DEVICE_ID,
	.dev.platform_data = (XPAR_GPIO_2_IS_DUAL ? XGPIO_IS_DUAL : 0),
	.num_resources = 1,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_GPIO_2_BASEADDR,
			.end	= XPAR_GPIO_2_HIGHADDR,
			.flags	= IORESOURCE_MEM
		}
	}
};

#endif /* XPAR_GPIO_2_BASEADDR */

#ifdef XPAR_GPIO_3_BASEADDR

static struct platform_device xilinx_gpio_3_device = {
	.name = "xilinx_gpio",
	.id = XPAR_GPIO_3_DEVICE_ID,
	.dev.platform_data = (XPAR_GPIO_3_IS_DUAL ? XGPIO_IS_DUAL : 0),
	.num_resources = 1,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_GPIO_3_BASEADDR,
			.end	= XPAR_GPIO_3_HIGHADDR,
			.flags	= IORESOURCE_MEM
		}
	}
};

#endif /* XPAR_GPIO_3_BASEADDR */

#ifdef XPAR_OPB_LCD_INTERFACE_0_BASEADDR

static struct platform_device xilinx_char_lcd_device = {
	.name = "xilinx_char_lcd",
	.id = 0,
	.num_resources = 1,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_OPB_LCD_INTERFACE_0_BASEADDR,
			.end	= XPAR_OPB_LCD_INTERFACE_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		}
	}
};

#endif /* XPAR_OPB_LCD_INTERFACE_0_BASEADDR */

#ifdef XPAR_TOUCHSCREEN_0_BASEADDR

static struct platform_device xilinx_touchscreen_device = {
	.name = "xilinx_ts",
	.id = 0,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_TOUCHSCREEN_0_BASEADDR,
			.end	= XPAR_TOUCHSCREEN_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_TOUCHSCREEN_0_VEC_ID,
			.end	= XPAR_INTC_0_TOUCHSCREEN_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_TOUCHSCREEN_0_BASEADDR */

#ifdef XPAR_SPI_0_BASEADDR

static struct xspi_platform_data xspi_0_pdata = {
	.device_flags = (XPAR_OPB_SPI_0_FIFO_EXIST ? XSPI_HAS_FIFOS : 0) |
		(XPAR_OPB_SPI_0_SPI_SLAVE_ONLY ? XSPI_SLAVE_ONLY : 0),
	.num_slave_bits = XPAR_OPB_SPI_0_NUM_SS_BITS
};

static struct platform_device xilinx_spi_0_device = {
	.name = "xilinx_spi",
	.id = XPAR_SPI_0_DEVICE_ID,
	.dev.platform_data = &xspi_0_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.start	= XPAR_SPI_0_BASEADDR,
			.end	= XPAR_SPI_0_HIGHADDR,
			.flags	= IORESOURCE_MEM
		},
		{
			.start	= XPAR_INTC_0_SPI_0_VEC_ID,
			.end	= XPAR_INTC_0_SPI_0_VEC_ID,
			.flags	= IORESOURCE_IRQ
		}
	}
};

#endif /* XPAR_SPI_0_BASEADDR */

static int __init xilinx_platform_init(void)
{
#ifdef XPAR_EMAC_0_BASEADDR
	memcpy(xemac_0_pdata.mac_addr, __res.bi_enetaddr, 6);
	platform_device_register(&xilinx_emac_0_device);
#endif /* XPAR_EMAC_0_BASEADDR */
#ifdef XPAR_EMAC_1_BASEADDR
	memcpy(xemac_1_pdata.mac_addr, __res.bi_enetaddr, 6);
	platform_device_register(&xilinx_emac_1_device);
#endif /* XPAR_EMAC_1_BASEADDR */

#ifdef XPAR_TEMAC_0_BASEADDR
	memcpy(xtemac_0_pdata.mac_addr, __res.bi_enetaddr, 6);
	platform_device_register(&xilinx_temac_0_device);
#endif /* XPAR_TEMAC_0_BASEADDR */
#ifdef XPAR_TEMAC_1_BASEADDR
	memcpy(xtemac_1_pdata.mac_addr, __res.bi_enetaddr, 6);
	platform_device_register(&xilinx_temac_1_device);
#endif /* XPAR_TEMAC_1_BASEADDR */

#ifdef XPAR_TFT_0_BASEADDR
	platform_device_register(&xilinx_lcd_0_device);
#endif /* XPAR_TFT_0_BASEADDR */

#ifdef XPAR_SYSACE_0_BASEADDR
	platform_device_register(&xilinx_sysace_0_device);
#endif /* XPAR_SYSACE_0_BASEADDR */

#ifdef XPAR_IIC_0_BASEADDR
	platform_device_register(&xilinx_iic_0_device);
#endif /* XPAR_IIC_0_BASEADDR */

#ifdef XPAR_GPIO_0_BASEADDR
	platform_device_register(&xilinx_gpio_0_device);
#endif /* XPAR_GPIO_0_BASEADDR */
#ifdef XPAR_GPIO_1_BASEADDR
	platform_device_register(&xilinx_gpio_1_device);
#endif /* XPAR_GPIO_1_BASEADDR */
#ifdef XPAR_GPIO_2_BASEADDR
	platform_device_register(&xilinx_gpio_2_device);
#endif /* XPAR_GPIO_2_BASEADDR */
#ifdef XPAR_GPIO_3_BASEADDR
	platform_device_register(&xilinx_gpio_3_device);
#endif /* XPAR_GPIO_3_BASEADDR */
#ifdef XPAR_GPIO_4_BASEADDR
	platform_device_register(&xilinx_gpio_4_device);
#endif /* XPAR_GPIO_4_BASEADDR */

#ifdef XPAR_PS2_0_BASEADDR
	platform_device_register(&xilinx_ps2_0_device);
#endif /* XPAR_PS2_0_BASEADDR */
#ifdef XPAR_PS2_1_BASEADDR
	platform_device_register(&xilinx_ps2_1_device);
#endif /* XPAR_PS2_1_BASEADDR */

#ifdef XPAR_TOUCHSCREEN_0_BASEADDR
	platform_device_register(&xilinx_touchscreen_device);
#endif /* XPAR_TOUCHSCREEN_0_BASEADDR */

#ifdef XPAR_SPI_0_BASEADDR
	platform_device_register(&xilinx_spi_0_device);
#endif /* XPAR_SPI_0_BASEADDR */

#ifdef XPAR_OPB_LCD_INTERFACE_0_BASEADDR
	platform_device_register(&xilinx_char_lcd_device);
#endif /* XPAR_OPB_LCD_INTERFACE_0_BASEADDR */

	return 0;
}

subsys_initcall(xilinx_platform_init);
