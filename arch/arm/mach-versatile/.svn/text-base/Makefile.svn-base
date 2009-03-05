#
# Makefile for the linux kernel.
#

obj-y					:= core.o clock.o time.o
obj-$(CONFIG_ARCH_VERSATILE_PB)		+= versatile_pb.o
obj-$(CONFIG_MACH_VERSATILE_AB)		+= versatile_ab.o
obj-$(CONFIG_KGDB_ARM_VERSATILE)	+= kgdb_serial.o
obj-$(CONFIG_HIGH_RES_TIMERS)		+= hrtime.o
