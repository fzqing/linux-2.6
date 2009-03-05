/*
 * drivers/char/xilinx_uartlite/xuartlite_serial.h
 *
 * Xilinx UART Lite driver
 *
 * Author: MontaVista Software, Inc.
 *         <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef xuartlite_serial_h
#define xuartlite_serial_h

#define DEVICE_NAME "xuartlite"

#define XULITE_MINOR_START	0
#define UARTLITE_TTY_NAME	"ttl"
#define UARTLITE_TTY_DEVFS_NAME	"ttl/%d"
#define UARTLITE_CU_NAME	"cul"
#define UARTLITE_CU_DEVFS_NAME	"cul/%d"

/* Use EXPERIMENTAL major to support non-devfs configuration as well */
#define XULITE_MAJOR            121
#define XULITE_AUX_MAJOR        122

/*
 * Debugging macros
 */

#define DEBUG_FLOW   0x0001
#define DEBUG_STAT   0x0002

#define DEBUG_MASK   0x0

#if (DEBUG_MASK != 0)
#define d_printk(str...)  printk(str)
#else
#define d_printk(str...)	/* nothing */
#endif

#if ((DEBUG_MASK & DEBUG_FLOW) != 0)
#define func_enter()      printk("xulite: enter %s\n", __FUNCTION__)
#define func_exit()       printk("xulite: exit  %s\n", __FUNCTION__)
#else
#define func_enter()
#define func_exit()
#endif

/*
 * Macro to get zero based index of the driver instance
 */

#define FIND_PNUM(x) (((struct gs_port*)x)->tty->index)

#endif				/* #ifndef xuartlite_serial_h */
