#
# Makefile for a ramdisk image
#

obj-y += ramdisk.o


img := $(subst ",,$(CONFIG_EMBEDDED_RAMDISK_IMAGE))
img := $(subst $(src)//,/,$(src)/$(img))

EXTRA_AFLAGS="-DMIPS_EMBEDDED_RAMDISK=\"$(img)\""
