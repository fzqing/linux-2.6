
obj-y := irq.o irq_handler.o time.o msgring.o on_chip.o

obj-$(CONFIG_SMP)	+= smp.o
obj-$(CONFIG_PHOENIX_DMA) += dma.o

EXTRA_AFLAGS := $(CFLAGS)
