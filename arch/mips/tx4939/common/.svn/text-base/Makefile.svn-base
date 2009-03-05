#
# Makefile for common code for Toshiba TX4939 based systems
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

obj-y	+= prom.o setup.o irq.o irq_handler.o rtc.o puts.o \
		spi_txx9.o tx49wtoe.o
obj-$(CONFIG_BLK_DEV_IDE_TX4939) += ide.o
obj-$(CONFIG_PROC_FS) += proc.o
