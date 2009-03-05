/*
 * linux/arch/arm/mach-davinci/serial_davinci.c
 *
 * TI DaVinci serial driver hookup file
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/serial.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/cpu.h>

#define UART_DAVINCI_PWREMU 0x0c

static inline unsigned int davinci_serial_in(struct plat_serial8250_port *up,
					     int offset)
{
	offset <<= up->regshift;
	return (unsigned int)__raw_readb(up->membase + offset);
}

static inline void davinci_serial_outp(struct plat_serial8250_port *p,
				       int offset, int value)
{
	offset <<= p->regshift;
	__raw_writeb(value, p->membase + offset);
}

static void __init davinci_serial_reset(struct plat_serial8250_port *p)
{
	/* reset both transmitter and receiver: bits 14,13 = UTRST, URRST */
	unsigned int pwremu = 0;

	davinci_serial_outp(p, UART_IER, 0);	/* disable all interrupts */

	davinci_serial_outp(p, UART_DAVINCI_PWREMU, pwremu);
	mdelay(10);

	pwremu |= (0x3 << 13);
	pwremu |= 0x1;
	davinci_serial_outp(p, UART_DAVINCI_PWREMU, pwremu);
}

#define UART_DM6467_SCR		__REG(DAVINCI_UART0_BASE + 0x40)
/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static int __init dm646x_serial_reset(void)
{
	UART_DM6467_SCR = 0x08;

	return 0;
}

void __init davinci_serial_init(struct platform_device *pdev)
{
	struct clk *uart_clk;
	struct device *dev = &pdev->dev;
	struct plat_serial8250_port *p;
	int uart;
	char uart_name[6];

	memset(uart_name, 0, sizeof(uart_name));
	for (p = dev->platform_data; p && p->flags; p++) {
		switch (p->mapbase) {
		case DAVINCI_UART0_BASE:
			uart = 0;
			break;
		case DAVINCI_UART1_BASE:
			uart = 1;
			break;
		case DM644X_UART2_BASE:
		case DM355_UART2_BASE:
			uart = 2;
			break;
		default:
			dev_err(dev,
				"Unknown UART base address 0x%08lx\n",
				p->mapbase);
			continue;
		}
		sprintf(uart_name, "UART%i", uart);
		uart_clk = clk_get(dev, uart_name);
		if (IS_ERR(uart_clk))
			dev_err(dev, "failed to get %s clock\n", uart_name);
		else
			clk_enable(uart_clk);

		if (cpu_is_davinci_dm355())
			davinci_serial_reset(p);
	}
}

late_initcall(dm646x_serial_reset);
