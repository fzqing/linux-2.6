#
# Makefile for the Xilinx I2C driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.
xilinx_iic-objs	:= i2c-algo-xilinx.o

# The Xilinx OS independent code.
xilinx_iic-objs	+= xiic.o xiic_intr.o xiic_l.o xiic_master.o xiic_options.o

obj-$(CONFIG_I2C_ALGO_XILINX) += xilinx_iic.o
