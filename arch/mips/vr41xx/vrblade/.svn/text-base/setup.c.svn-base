/*
 * arch/mips/vr41xx/vrblade/setup.c
 *
 * Setup for the NEC VRBlade VR4133 Wildcat.
 *
 * Author: Yoichi Yuasa <yyuasa@mvista.com, or source@mvista.com> and
 *         Alex Sapkov <asapkov@ru.mvista.com>
 *		   Edmond dela Cruz <edmondd@ntsp.nec.co.jp>
 *
 * 2001-2004 (c) MontaVista, Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for NEC VRBlade VR4133 Wildcat board in 2.6
 * Author: Manish Lachwani (mlachwani@mvista.com)
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/ioport.h>
#include <asm/irq.h>

#include <asm/reboot.h>
#include <asm/time.h>
#include <asm/vr41xx/vrblade4133a.h>
#include <asm/bootinfo.h>

#ifdef CONFIG_MTD
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/i2c-algo-bit.h>

static struct mtd_partition vrblade_mtd_parts[] __initdata = {
	{
	 .name = "Kernel 1",
	 .size = 0x01000000,
	 .offset = 0,
	 .mask_flags = 0,
	 }, {
	     .name = "Kernel 0",
	     .size = 0x00a00000,
	     .offset = 0x01000000,
	     .mask_flags = 0,
	     }, {
		 .name = "User FS0",
		 .size = 0x00200000,
		 .offset = 0x01a00000,
		 .mask_flags = 0,
		 }, {
		     .name = "PMON",
		     .size = 0x00200000,
		     .offset = 0x01c00000,
		     .mask_flags = MTD_WRITEABLE,
		     }, {
			 .name = "Config",
			 .size = 0x00200000,
			 .offset = 0x01e00000,
			 .mask_flags = 0,
			 }
};
#define part_num (sizeof(vrblade_mtd_parts)/sizeof(struct mtd_partition))
#endif

#ifdef CONFIG_NEC_CANDY
#include <linux/nec_candy_pd.h>

#define VR4133_ETHER0_START	0x0f001400
#define VR4133_ETHER0_END	0x0f00163f

#define VR4133_ETHER1_START	0x0f001700
#define VR4133_ETHER1_END	0x0f00193f

#define VR4133_SCUARBITSELREG	KSEG1ADDR(0x0F00100A)

#define VR4133A_WORKAROUND_MASK (WORKAROUND_E21_PAD_BIT | WORKAROUND_E10_VR4133_BIT)
static struct nec_candy_platform_data nec_candy0_pdata = {
	.pmd_addr = 0,
	.platform_id = NEC_VRBLADE_VR4133A_PID | VR4133A_WORKAROUND_MASK,
	.platform_options.n_marvell_ports = 0,
	.platform_options.use_tx_ring_buffer = 1,
};

static struct platform_device nec_candy0_device = {
	.name = "nec_candy",
	.id = 0,
	.dev.platform_data = &nec_candy0_pdata,
	.num_resources = 2,
	.resource = (struct resource[]){
	 {
	 .name = "nec_candy_regs",
	 .start = VR4133_ETHER0_START,
	 .end = VR4133_ETHER0_END,
	 .flags = IORESOURCE_MEM,
	 },
	 {
	 .name = "nec_candy_irq",
	 .start = ETHERNET_IRQ,
	 .end = ETHERNET_IRQ,
	 .flags = IORESOURCE_IRQ,
	 },
	},
};

static struct nec_candy_platform_data nec_candy1_pdata = {
	.pmd_addr = 0x10,
	.platform_id = NEC_VRBLADE_VR4133A_PID | VR4133A_WORKAROUND_MASK,
	.platform_options.n_marvell_ports = 5,
	.platform_options.use_tx_ring_buffer = 1,
};

static struct platform_device nec_candy1_device = {
	.name = "nec_candy",
	.id = 1,
	.dev.platform_data = &nec_candy1_pdata,
	.num_resources = 2,
	.resource = (struct resource[]){
					{
					 .name = "nec_candy_regs",
					 .start = VR4133_ETHER1_START,
					 .end = VR4133_ETHER1_END,
					 .flags = IORESOURCE_MEM,
					 },
					{
					 .name = "nec_candy_irq",
					 .start = ETHERNET_IRQ,
					 .end = ETHERNET_IRQ,
					 .flags = IORESOURCE_IRQ,
					 },
					},
};

#ifndef CONFIG_I2C_VR41XX
u_char candy0_mac_addr[6] = { 0x00, 0x00, 0x4c, 0x80, 0x92, 0xa1 };
u_char candy1_mac_addr[6] = { 0x00, 0x00, 0x4c, 0x80, 0x92, 0xa2 };
#endif
#endif

#ifdef CONFIG_I2C_VR41XX
#include <asm/vr41xx/giu.h>

static struct vr41xx_i2c_pins cmbvr4133_i2c_gpio_pins = {
	.sda_pin = CMBVR4133_SDA_PIN,
	.scl_pin = CMBVR4133_SCL_PIN,
};

static struct platform_device cmbvr4133_i2c_controller = {
	.name = "VR41XX-I2C",
	.id = 0,
	.dev = {
		.platform_data = &cmbvr4133_i2c_gpio_pins,
		},
	.num_resources = 0,
};

#define EEPROM_ADDRESS 0x50

struct vr41xx_i2c_data {
	struct vr41xx_i2c_pins *gpio_pins;
	struct i2c_adapter adapter;
	struct i2c_algo_bit_data algo_data;
};

unsigned char i2c_eeprom_read_byte(unsigned char address)
{
	unsigned char retval;
	struct vr41xx_i2c_data *drv_data =
	    dev_get_drvdata(&cmbvr4133_i2c_controller.dev);
	struct i2c_msg msg[2] = {
		{
		 .addr = EEPROM_ADDRESS,
		 .flags = 0,
		 .len = 1,
		 .buf = &address,
		 },
		{
		 .addr = EEPROM_ADDRESS,
		 .flags = I2C_M_RD,
		 .len = 1,
		 .buf = &retval,
		 },
	};

	i2c_transfer(&drv_data->adapter, msg, 2);

	return retval;
}

#endif

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_KGDB_8250)
static void __init vr4133_serial_init(void)
{
	vr41xx_select_siu_interface(SIU_RS232C, IRDA_NONE);
	vr41xx_siu_init();
	vr41xx_dsiu_init();
}
#endif

static void __init vrblade_timer_setup(struct irqaction *irq)
{
	setup_irq(TIMER_IRQ, irq);
}

static int __init ncos_vrblade_setup(void)
{
	set_io_port_base(KSEG1ADDR(0x17000000));

	mips_machgroup = MACH_GROUP_NEC_VR41XX;
	mips_machtype = 0x0f000204;

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_KGDB_8250)
	vr4133_serial_init();
#endif

#ifdef CONFIG_MTD
	/* we use generic physmap mapping driver and we use partitions */
	physmap_configure(0x1E000000, 0x02000000, 2, NULL);
	physmap_set_partitions(vrblade_mtd_parts, part_num);
#endif

	/* 128 MB memory support */
	add_memory_region(0, 0x08000000, BOOT_MEM_RAM);

#ifdef CONFIG_CPU_TIMER
	board_timer_setup = vrblade_timer_setup;
	mips_hpt_frequency = 16618750;
#endif

	return 0;
}

early_initcall(ncos_vrblade_setup);

static int __init ether_vrblade_setup(void)
{
	int i;

#ifdef CONFIG_I2C_VR41XX
	platform_device_register(&cmbvr4133_i2c_controller);
#endif
#ifdef CONFIG_NEC_CANDY
	vr41xx_supply_clock(ETHER0_CLOCK);
	vr41xx_supply_clock(ETHER1_CLOCK);

	vr41xx_enable_macint(MACINT_ALL);
	/* Change SCU BUS Arbiter protocol (fixed -> fair) */
	writew(0x01, (void *)VR4133_SCUARBITSELREG);

	/* Ethernet irq is set to Int3# */
	vr41xx_set_intassign(ETHERNET_IRQ, 3);
	/* PCI irq is set to Int2# */
	vr41xx_set_intassign(PCI_IRQ, 2);

#ifndef CONFIG_I2C_VR41XX
	for (i = 0; i < 6; i++) {
		nec_candy0_pdata.mac_addr[i] = candy0_mac_addr[i];
		nec_candy1_pdata.mac_addr[i] = candy1_mac_addr[i];
	}
#else
	for (i = 0; i < 6; i++) {
		nec_candy0_pdata.mac_addr[i] = i2c_eeprom_read_byte(i);

		/* For MAC Printing */
		if (i != 5) {
			pr_debug("%x:", nec_candy0_pdata.mac_addr[i]);
		} else {
			pr_debug("%x\n", nec_candy0_pdata.mac_addr[i]);
		}
	}
	for (i = 0; i < 6; i++) {
		nec_candy1_pdata.mac_addr[i] = i2c_eeprom_read_byte(i + 6);

		/* For MAC Printing */
		if (i != 5) {
			pr_debug("%x:", nec_candy1_pdata.mac_addr[i]);
		} else {
			pr_debug("%x\n", nec_candy1_pdata.mac_addr[i]);
		}
	}
#endif

	platform_device_register(&nec_candy0_device);
	platform_device_register(&nec_candy1_device);
#endif
	return 0;
}

late_initcall(ether_vrblade_setup);
