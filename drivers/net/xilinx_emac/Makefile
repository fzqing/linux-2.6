#
# Makefile for the Xilinx 10/100 Mbit EMAC driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.
xilinx_emac-objs	:= adapter.o adapter_fifo.o adapter_sgdma.o

# The Xilinx OS independent code.
xilinx_emac-objs	+= xemac.o xemac_intr.o xemac_intr_dma.o \
			   xemac_intr_fifo.o xemac_options.o xemac_phy.o

obj-$(CONFIG_XILINX_EMAC) += xilinx_emac.o
