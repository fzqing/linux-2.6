#
# Makefile for the Xilinx 10/100/1000 Mbit TEMAC driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.
xilinx_temac-objs	:= adapter.o

# The Xilinx OS independent code.
xilinx_temac-objs	+= xtemac.o xtemac_intr.o \
			   xtemac_intr_sgdma.o xtemac_sgdma.o \
			   xtemac_intr_fifo.o xtemac_fifo.o \
			   xtemac_l.o xtemac_control.o

obj-$(CONFIG_XILINX_TEMAC) += xilinx_temac.o
