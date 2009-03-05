#
# Makefile for the Xilinx UART Lite driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common -Iarch/ppc/platforms/4xx/xparameters

# The Linux adapter for the Xilinx driver code.
xilinx_uartlite-objs	:= xuartlite_serial.o

# The Xilinx OS independent code.
xilinx_uartlite-objs	+= xuartlite.o xuartlite_intr.o xuartlite_l.o xuartlite_g.o

obj-$(CONFIG_XILINX_UARTLITE) += xilinx_uartlite.o
