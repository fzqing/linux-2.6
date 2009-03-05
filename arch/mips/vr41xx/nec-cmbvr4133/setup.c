/*
 * arch/mips/vr41xx/nec-cmbvr4133/setup.c
 *
 * Setup for the NEC CMB-VR4133.
 *
 * Author: Yoichi Yuasa <yyuasa@mvista.com, or source@mvista.com> and
 *         Alex Sapkov <asapkov@ru.mvista.com>
 *
 * 2001-2004 (c) MontaVista, Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for CMBVR4133 board in 2.6
 * Author: Manish Lachwani (mlachwani@mvista.com)
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/ide.h>
#include <linux/ioport.h>

#include <asm/reboot.h>
#include <asm/time.h>
#include <asm/vr41xx/cmbvr4133.h>
#include <asm/bootinfo.h>

#ifdef CONFIG_MTD
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>

static struct mtd_partition cmbvr4133_mtd_parts[] = {
	{
		.name =		"User FS",
		.size =		0x1be0000,
		.offset =	0,
		.mask_flags = 	0,
	}, 
	{
		.name =		"PMON",
		.size =		0x140000,
		.offset =	MTDPART_OFS_APPEND,
		.mask_flags =	MTD_WRITEABLE,  /* force read-only */
	}, 
	{
		.name =		"User FS2",
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_APPEND,
		.mask_flags = 	0,
	}
};

#define number_partitions (sizeof(cmbvr4133_mtd_parts)/sizeof(struct mtd_partition))
#endif

#ifdef CONFIG_NEC_CANDY
#include <linux/nec_candy_pd.h>

#define VR4133_ETHER0_START	0x0f001400
#define VR4133_ETHER0_END	0x0f00163f

#define VR4133_ETHER1_START	0x0f001700
#define VR4133_ETHER1_END	0x0f00193f

#define CMBVR4133_MAC0_FLASH	0xbdbffe00
#define CMBVR4133_MAC1_FLASH	0xbdbffe06

#define VR4133_WORKAROUND_MASK (WORKAROUND_E10_VR4133_BIT | WORKAROUND_E21_PAD_BIT)
static struct nec_candy_platform_data nec_candy0_pdata = {
	.pmd_addr = 0,
	.platform_id = NEC_CMB_VR4133_PID | VR4133_WORKAROUND_MASK,
	.platform_options.use_tx_ring_buffer = 0,
};

static struct platform_device nec_candy0_device = {
	.name = "nec_candy",
	.id = 0,
	.dev.platform_data = &nec_candy0_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.name	= "nec_candy_regs",
			.start  = VR4133_ETHER0_START,
			.end    = VR4133_ETHER0_END,
			.flags  = IORESOURCE_MEM,
		},
		{
			.name	= "nec_candy_irq",
			.start  = ETHERNET_IRQ,
			.end    = ETHERNET_IRQ,
			.flags  = IORESOURCE_IRQ,
		},
	},
};


static struct nec_candy_platform_data nec_candy1_pdata = {
	.pmd_addr = 0,
	.platform_id = NEC_CMB_VR4133_PID | VR4133_WORKAROUND_MASK,
	.platform_options.use_tx_ring_buffer = 0,
};

static struct platform_device nec_candy1_device = {
	.name = "nec_candy",
	.id = 1,
	.dev.platform_data = &nec_candy1_pdata,
	.num_resources = 2,
	.resource = (struct resource[]) {
		{
			.name	= "nec_candy_regs",
			.start  = VR4133_ETHER1_START,
			.end    = VR4133_ETHER1_END,
			.flags  = IORESOURCE_MEM,
		},
		{
			.name	= "nec_candy_irq",
			.start  = ETHERNET_IRQ,
			.end    = ETHERNET_IRQ,
			.flags  = IORESOURCE_IRQ,
		},
	},
};
#endif

#ifdef CONFIG_I2C_VR41XX
#include <asm/vr41xx/giu.h>

static struct vr41xx_i2c_pins cmbvr4133_i2c_gpio_pins = {
	.sda_pin	= CMBVR4133_SDA_PIN,
	.scl_pin	= CMBVR4133_SCL_PIN,
};

static struct platform_device cmbvr4133_i2c_controller = {
	.name		= "VR41XX-I2C",
	.id		= 0,
	.dev		= {
		.platform_data = &cmbvr4133_i2c_gpio_pins,
	},
	.num_resources	= 0,
};
#endif

static int __init nec_cmbvr4133_arch_setup(void)
{
#ifdef CONFIG_NEC_CANDY
	vr41xx_supply_clock(ETHER0_CLOCK);
	vr41xx_supply_clock(ETHER1_CLOCK);

	vr41xx_enable_macint(MACINT_ALL);

	memcpy(nec_candy0_pdata.mac_addr, (void *) CMBVR4133_MAC0_FLASH, 6);
	platform_device_register(&nec_candy0_device);
	memcpy(nec_candy1_pdata.mac_addr, (void *) CMBVR4133_MAC1_FLASH, 6);
	platform_device_register(&nec_candy1_device);
#endif

#ifdef CONFIG_I2C_VR41XX
	platform_device_register(&cmbvr4133_i2c_controller);
#endif
	return 0;
}
arch_initcall(nec_cmbvr4133_arch_setup);

static void __init vr4133_serial_init(void)
{
	vr41xx_select_siu_interface(SIU_RS232C, IRDA_NONE);
	vr41xx_siu_init();
	vr41xx_dsiu_init();
}

extern void i8259_init(void);

static int __init nec_cmbvr4133_setup(void)
{
#ifdef CONFIG_ROCKHOPPER
	extern void disable_pcnet(void);

	disable_pcnet();
#endif
	set_io_port_base(KSEG1ADDR(0x16000000));

	mips_machgroup = MACH_GROUP_NEC_VR41XX;
	mips_machtype = MACH_NEC_CMBVR4133;

	vr4133_serial_init();

#ifdef CONFIG_PCI
#ifdef CONFIG_ROCKHOPPER
	ali_m5229_preinit();
#endif
#endif

#ifdef CONFIG_MTD
	/* we use generic physmap mapping driver and we use partitions */
	physmap_configure(0x1C000000, 0x02000000, 4, NULL);
	physmap_set_partitions(cmbvr4133_mtd_parts, number_partitions);
#endif

	/* 128 MB memory support */
	add_memory_region(0, 0x08000000, BOOT_MEM_RAM);

#ifdef CONFIG_ROCKHOPPER
	i8259_init();
#endif
	return 0;
}
early_initcall(nec_cmbvr4133_setup);
