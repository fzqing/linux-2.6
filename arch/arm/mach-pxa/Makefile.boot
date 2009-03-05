ifeq ($(CONFIG_MACH_ZYLONITE),y)
   zreladdr-y	:= 0x80008000
else
   zreladdr-y	:= 0xa0008000
endif


