# Linux file to EXPORT_SYMBOL all of the Xilinx entries.
obj-$(CONFIG_XILINX_EDK) := xilinx_syms.o

# The Xilinx OS independent code.
obj-$(CONFIG_XILINX_EDK) += xbasic_types.o  xipif_v1_23_b.o \
			 xpacket_fifo_v2_00_a.o xpacket_fifo_l_v2_00_a.o \
			 xdma_channel.o xdma_channel_sg.o \
			 xdmav2.o xdmav2_intr.o xdmav2_sg.o xdmav2_simple.o \
			 xversion.o
