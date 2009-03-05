#
# Makefile for x86 timers
#

obj-y := timer.o timer_none.o timer_tsc.o timer_pit.o common.o

obj-$(CONFIG_X86_CYCLONE_TIMER)	+= timer_cyclone.o
obj-$(CONFIG_HPET_TIMER)	+= timer_hpet.o
obj-$(CONFIG_X86_PM_TIMER)	+= timer_pm.o
obj-$(CONFIG_HIGH_RES_TIMER_ACPI_PM) += hrtimer_pm.o
obj-$(CONFIG_HIGH_RES_TIMER_ACPI_PM) += high-res-tbxfroot.o
obj-$(CONFIG_HIGH_RES_TIMER_TSC) += hrtimer_tsc.o