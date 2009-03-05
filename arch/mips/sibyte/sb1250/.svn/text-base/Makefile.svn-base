obj-y := setup.o irq.o irq_handler.o 

ifndef CONFIG_CPU_TIMER
obj-y					+= time.o
endif
obj-$(CONFIG_SMP)			+= smp.o
obj-$(CONFIG_SIBYTE_TBPROF)		+= bcm1250_tbprof.o
obj-$(CONFIG_SIBYTE_STANDALONE)		+= prom.o
obj-$(CONFIG_SIBYTE_BUS_WATCHER)	+= bus_watcher.o
obj-$(CONFIG_KGDB_SIBYTE)		+= kgdb_sibyte.o

EXTRA_AFLAGS := $(CFLAGS)
