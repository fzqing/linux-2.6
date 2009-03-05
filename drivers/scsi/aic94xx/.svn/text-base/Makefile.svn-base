#
# Makefile for the Linux adp94xx SAS driver.
#
# $Id: //depot/razor-branch-v12/linux/src/Makefile.2_6#3 $
#

EXTRA_CFLAGS += -Idrivers/scsi -UASD_DEBUG 

ifeq ($(CONFIG_SCSI_ADP94XX),)
	CONFIG_SCSI_ADP94XX=m
endif

obj-$(CONFIG_SCSI_ADP94XX)	+= adp94xx.o

# OSM and HWI Specific Files
adp94xx-y			+= adp94xx_osm.o	\
				   adp94xx_hwi.o	\
				   adp94xx_seq.o	\
				   adp94xx_discover.o

# IOCTL Specific File
adp94xx-y			+= adp94xx_ioctl.o

# SATA Specific File
adp94xx-y			+= adp94xx_sata.o



