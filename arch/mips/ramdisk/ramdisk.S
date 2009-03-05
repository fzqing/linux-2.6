/*
 * For a detailed explanation of this file, see usr/initramfs_data.S in
 * the root of this source tree.
 */

.section .initrd,"a"
.incbin MIPS_EMBEDDED_RAMDISK
