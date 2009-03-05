#
# Makefile for the linux kernel.
#

# Common support (must be linked before board specific support)
obj-y += generic.o irq.o dma.o time.o
obj-$(CONFIG_PXA25x) += pxa25x.o
obj-$(CONFIG_PXA27x) += pxa27x.o
obj-$(CONFIG_PXA3xx) += pxa3xx.o mfp.o

# Specific board support
obj-$(CONFIG_ARCH_LUBBOCK) += lubbock.o
obj-$(CONFIG_MACH_MAINSTONE) += mainstone.o
obj-$(CONFIG_ARCH_PXA_IDP) += idp.o
obj-$(CONFIG_MACH_ZYLONITE)   += zylonite.o

# Support for blinky lights
led-y := leds.o
led-$(CONFIG_ARCH_LUBBOCK) += leds-lubbock.o
led-$(CONFIG_MACH_MAINSTONE) += leds-mainstone.o
led-$(CONFIG_ARCH_PXA_IDP) += leds-idp.o
led-$(CONFIG_MACH_ZYLONITE) += leds-zylonite.o

obj-$(CONFIG_LEDS) += $(led-y)

# Misc features
ifeq ($(CONFIG_PXA3xx), y)
obj-$(CONFIG_PXA3xx) += mhn_gpio.o
obj-$(CONFIG_PXA3xx_PMIC) += mhn_pmic.o
obj-$(CONFIG_PM) += pxa3xx_pm.o pxa3xx_suspend.o sleepwkr.o

else
obj-$(CONFIG_PM) += pm.o sleep.o
endif

ifeq ($(CONFIG_PXA27x),y)
obj-$(CONFIG_PM) += standby.o
endif

obj-$(CONFIG_KGDB_PXA_SERIAL) += kgdb-serial.o
obj-$(CONFIG_HIGH_RES_TIMERS) += hrtime.o
