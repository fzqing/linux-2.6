#EXTRA_CFLAGS                   += -DDEBUG=1 -DMV_LOG_DEBUG -DLINUX -Idrivers/scsi
EXTRA_CFLAGS                   += -DLINUX -Idrivers/scsi
obj-$(CONFIG_SCSI_MVSATA)      += mvsata.o
mvsata-y	               += mvLinuxIalLib.o \
                                  mvLinuxIalHt.o \
                                  mvLinuxIalOs.o \
                                  mvIALCommon.o \
                                  mvIALCommonUtils.o \
                                  mvLinuxIalSmart.o \
                                  mvSata.o \
                                  mvStorageDev.o \
                                  mvLog.o \
                                  mvScsiAtaLayer.o
