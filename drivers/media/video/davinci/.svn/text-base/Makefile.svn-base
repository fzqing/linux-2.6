#
# Makefile for the davinci video device drivers.
#

# Capture: DaVinci and DM355
ifdef CONFIG_ARCH_DAVINCI_DM644x
davinci_capture-objs := ccdc_davinci.o davinci_vpfe.o 
endif
ifdef CONFIG_ARCH_DAVINCI_DM355
davinci_capture-objs := ccdc_dm355.o davinci_vpfe.o 
endif
obj-$(CONFIG_VIDEO_TVP5146) += tvp5146.o
obj-$(CONFIG_VIDEO_MT9T001) += mt9t001.o
obj-$(CONFIG_VIDEO_DAVINCI_CAPTURE) += davinci_capture.o
obj-$(CONFIG_DAVINCI_OSD) += davinci_osd.o davinci_platform.o

# Encoder Manager and Encoders
obj-$(CONFIG_DAVINCI_ENC_MNGR)	+= davinci_enc_mngr.o
obj-$(CONFIG_DAVINCI_VPBE_ENCODER)	+= vpbe_encoder.o
obj-$(CONFIG_DAVINCI_LOGICPD_ENCODER) += logicpd_encoder.o
obj-$(CONFIG_DAVINCI_THS8200_ENCODER) += ths8200_encoder.o

obj-$(CONFIG_VIDEO_DAVINCI_DISPLAY) += davinci_display.o

# DaVinci HD
obj-$(CONFIG_VIDEO_DAVINCIHD_VPIF) += vpif.o davincihd_platform.o
obj-$(CONFIG_CAPTURE_DAVINCIHD) += davincihd_capture.o
obj-$(CONFIG_VIDEO_TVP5147) += tvp5147.o
obj-$(CONFIG_VIDEO_TVP7002) += tvp7002.o

obj-$(CONFIG_DISPLAY_DAVINCIHD) += davincihd_display.o
obj-$(CONFIG_VIDEO_ADV7343) += adv7343.o

# H3A and IPIPE
davinci_aew_driver-objs := davinci_aew_hw.o davinci_aew.o
obj-$(CONFIG_AEW) += davinci_aew_driver.o
davinci_af_driver-objs := davinci_af_hw.o davinci_af.o
obj-$(CONFIG_AF) += davinci_af_driver.o

dm355_ipipe_driver-objs := dm355_ipipe.o dm355_ipipe_hw.o
obj-$(CONFIG_DM355_IPIPE) += dm355_ipipe_driver.o
dm355_af_driver-objs := dm355_af_hw.o dm355_af.o
obj-$(CONFIG_DM355_AF) += dm355_af_driver.o
dm355_aew_driver-objs := dm355_aew_hw.o dm355_aew.o
obj-$(CONFIG_DM355_AEW) += dm355_aew_driver.o
