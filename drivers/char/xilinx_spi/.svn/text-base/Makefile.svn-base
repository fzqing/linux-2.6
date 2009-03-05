#
# Makefile for the Xilinx SPI driver
#

EXTRA_CFLAGS	+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.
xilinx_spi-objs	:= xspi_adapter.o

# The Xilinx OS independent code.
xilinx_spi-objs	+= xspi.o xspi_options.o xspi_stats.o

obj-$(CONFIG_XILINX_SPI) := xilinx_spi.o

