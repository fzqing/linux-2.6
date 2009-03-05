# $Id: Makefile,v 1.2 2002/09/19 07:06:12 sfoehner Exp $
#
# Makefile for the Tensilica XT2000 Emulation Board
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are in the main makefile...

obj-y 			= time.o setup.o i2c.o
obj-$(CONFIG_PCI)	+= pci.o
## obj-$(CONFIG_KGDB) 	+= gdb_hook.o

