#
# Makefile for the Cavium Octeon specific kernel interface routines
# under Linux.
#

obj-y                                 := setup.o serial.o int_handler.o hal.o adel_workaround.o perf_counters.o octeon_info.o 
obj-y				      += i8259.o
obj-$(CONFIG_SMP)                     += smp.o
obj-$(CONFIG_CAVIUM_OCTEON_USER_IO)   += userio.o
obj-$(CONFIG_CAVIUM_OCTEON_EBT3000)   += ebt3000_cf.o
obj-y				      += cvmx-spi4000.o
obj-y				      += cvmx-pko.o
