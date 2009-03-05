make ARCH=arm CROSS_COMPILE=arm_v5t_le- uImage
make ARCH=arm CROSS_COMPILE=arm_v5t_le- modules
cp ./arch/arm/boot/uImage ./
#/tftpboot/uImage_phm_1_3
#make ARCH=arm CROSS_COMPILE=arm_v5t_le- INSTALL_MOD_PATH=/home/phm/MyProjects/DTV/target_1_3/ modules_install
#make ARCH=arm CROSS_COMPILE=arm_v5t_le- INSTALL_MOD_PATH=/project/DTV/busybox-1.11.0/ modules_install
#cp -av /project/DTV/target_1_3/lib/modules/2.6.10_mvl401/kernel/drivers/usb/gadget/*  /home/phm/MyProjects/DTV/release/rootfs/lib/modules/2.6.10_mvl401/kernel/drivers/usb/gadget/
