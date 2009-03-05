#
# Makefile for the Xilinx GPIO driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.
xilinx_gpio-objs	:= adapter.o

# The Xilinx OS independent code.
xilinx_gpio-objs	+= xgpio.o

obj-$(CONFIG_XILINX_GPIO) += xilinx_gpio.o
