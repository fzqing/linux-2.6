#
# Makefile for ALSA
#

snd-sa11xx-uda1341-objs := sa11xx-uda1341.o
snd-pxa2xx-pcm-objs := pxa2xx-pcm.o
snd-pxa2xx-ac97-objs := pxa2xx-ac97.o
snd-pxa2xx-snd-objs := pxa2xx-snd.o
snd-pxa3xx-snd-objs := pxa3xx-snd.o pxa3xx-wm9713l.o

# Toplevel Module Dependency
obj-$(CONFIG_SND_SA11XX_UDA1341) += snd-sa11xx-uda1341.o 
obj-$(CONFIG_SND_PXA2xx_PCM) += snd-pxa2xx-pcm.o
obj-$(CONFIG_SND_PXA2xx_AC97) += snd-pxa2xx-ac97.o
obj-$(CONFIG_SND_PXA2xx_SND) += snd-pxa2xx-snd.o
obj-$(CONFIG_SND_PXA3xx_SND) += snd-pxa3xx-snd.o
