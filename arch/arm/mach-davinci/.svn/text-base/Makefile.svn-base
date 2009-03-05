#
# Makefile for the linux kernel.
#
#


# Common objects
obj-y                                 := time.o irq.o dma.o serial.o clock.o  io.o \
                                       id.o gpio.o mux.o mux_cfg.o



# Board specific
obj-$(CONFIG_MACH_DAVINCI_EVM)  += board-evm.o i2c-emac.o
obj-$(CONFIG_MACH_DAVINCI_HD_EVM)  += board-dm6467-evm.o i2c-emac.o \
					video_hdevm.o
obj-$(CONFIG_MACH_DAVINCI_DM355_EVM)	+= board-dm355-evm.o
obj-$(CONFIG_DAVINCI_MCBSP)	+= mcbsp.o
obj-$(CONFIG_DAVINCI_MCASP)	+= mcasp.o
obj-$(CONFIG_I2C_DAVINCI)       += i2c-client.o
obj-$(CONFIG_DAVINCI_SPI)       += davinci_spi_platform.o


ifeq ($(CONFIG_LEDS),y)
obj-$(CONFIG_MACH_DAVINCI_EVM)	+= leds-evm.o
endif
