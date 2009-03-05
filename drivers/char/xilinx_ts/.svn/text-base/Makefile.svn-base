#
# Makefile for the Xilinx touchscreen driver
#

EXTRA_CFLAGS	+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.
xilinx_ts-objs	:= adapter.o

# The Xilinx OS independent code.
xilinx_ts-objs	+= xtouchscreen.o xtouchscreen_intr.o xtouchscreen_l.o

obj-$(CONFIG_XILINX_TS) := xilinx_ts.o

