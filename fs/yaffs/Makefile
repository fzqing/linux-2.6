
#
# Makefile for the linux YAFFS1 filesystem routines.
#

obj-$(CONFIG_YAFFS1_FS) += yaffs.o

yaffs-y := yaffs_ecc.o yaffs_fs.o yaffs_guts.o

yaffs-$(CONFIG_YAFFS1_MTD_ENABLED) += yaffs_mtdif.o
yaffs-$(CONFIG_YAFFS1_RAM_ENABLED) += yaffs_ramem.o
