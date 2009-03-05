/*
 * linux/drivers/media/video/mainstone.h
 *
 * Driver for Intel Mainstone Camera
 *
 * Author: Aleskey Makarov <amakarov@ru.mvista.com>
 *
 * 2003 (c) Intel Corporation
 * 2003-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _MAINSTONE_CAMERA_H__
#define _MAINSTONE_CAMERA_H__

/* MCLK, kHz */
#define CI_MCLK_DEFT 13000

/* phys addreses of ci registers CIBR */
#define CIBR0_PHY       (0x50000000 + 0x28)
#define CIBR1_PHY       (0x50000000 + 0x30)
#define CIBR2_PHY       (0x50000000 + 0x38)

#define CI_REG_SIZE             0x40	/* 0x5000_0000 --- 0x5000_0038 * 64K */
#define CI_REGS_PHYS            0x50000000

/* CICR0 */
#define CI_CICR0_FOM            (1 << 0)
#define CI_CICR0_EOFM           (1 << 1)
#define CI_CICR0_SOFM           (1 << 2)
#define CI_CICR0_CDM            (1 << 3)
#define CI_CICR0_QDM            (1 << 4)
#define CI_CICR0_PERRM          (1 << 5)
#define CI_CICR0_EOLM           (1 << 6)
#define CI_CICR0_FEM            (1 << 7)
#define CI_CICR0_RDAVM          (1 << 8)
#define CI_CICR0_TOM            (1 << 9)
#define CI_CICR0_SIM_SHIFT              24
#define CI_CICR0_SIM_SMASK              0x7

#define CI_CICR0_SIM_MODE_MP            0	/* Master-Parallel */
#define CI_CICR0_SIM_MODE_SP            1	/* Slave-Parallel */
#define CI_CICR0_SIM_MODE_MS            2	/* Master-Serial */
#define CI_CICR0_SIM_MODE_EP            3	/* Embedded-Parallel */
#define CI_CICR0_SIM_MODE_ES            4	/* Embedded-Serial */

#define CI_CICR0_DIS                    (1 << 27)
#define CI_CICR0_ENB                    (1 << 28)
#define CI_CICR0_SL_CAP_EN              (1 << 29)
#define CI_CICR0_PAR_EN                 (1 << 30)
#define CI_CICR0_DMA_EN                 (1 << 31)
#define CI_CICR0_INTERRUPT_MASK         0x3FF

/* CICR1 */
#define CI_CICR1_TBIT                   (1 << 31)
#define CI_CICR1_RGBT_CONV_SHIFT        29
#define CI_CICR1_RGBT_CONV_SMASK        0x3
#define CI_CICR1_PPL_SHIFT              15
#define CI_CICR1_PPL_SMASK              0x7FF
#define CI_CICR1_RGB_CONV_SHIFT         12
#define CI_CICR1_RGB_CONV_SMASK         0x7
#define CI_CICR1_RBG_F                  (1 << 11)
#define CI_CICR1_YCBCR_F                (1 << 10)
#define CI_CICR1_RGB_BPP_SHIFT          7
#define CI_CICR1_RGB_BPP_SMASK          0x7
#define CI_CICR1_RAW_BPP_SHIFT          5
#define CI_CICR1_RAW_BPP_SMASK          0x3
#define CI_CICR1_COLOR_SP_SHIFT         3
#define CI_CICR1_COLOR_SP_SMASK         0x3
#define CI_CICR1_DW_SHIFT               0
#define CI_CICR1_DW_SMASK               0x7

#define CI_CICR1_DW4  0x0
#define CI_CICR1_DW5  0x1
#define CI_CICR1_DW8  0x2
#define CI_CICR1_DW9  0x3,
#define CI_CICR1_DW10 0x4

/* CICR2 */
#define CI_CICR2_FSW_SHIFT      0
#define CI_CICR2_FSW_SMASK      0x3
#define CI_CICR2_BFPW_SHIFT     3
#define CI_CICR2_BFPW_SMASK     0x3F
#define CI_CICR2_HSW_SHIFT      10
#define CI_CICR2_HSW_SMASK      0x3F
#define CI_CICR2_ELW_SHIFT      16
#define CI_CICR2_ELW_SMASK      0xFF
#define CI_CICR2_BLW_SHIFT      24
#define CI_CICR2_BLW_SMASK      0xFF

/* CICR3 */
#define CI_CICR3_LPF_SHIFT      0
#define CI_CICR3_LPF_SMASK      0x7FF
#define CI_CICR3_VSW_SHIFT      11
#define CI_CICR3_VSW_SMASK      0x1F
#define CI_CICR3_EFW_SHIFT      16
#define CI_CICR3_EFW_SMASK      0xFF
#define CI_CICR3_BFW_SHIFT      24
#define CI_CICR3_BFW_SMASK      0xFF

/* CICR4 */
#define CI_CICR4_DIV_SHIFT      0
#define CI_CICR4_DIV_SMASK      0xFF
#define CI_CICR4_FR_RATE_SHIFT  8
#define CI_CICR4_FR_RATE_SMASK  0x7
#define CI_CICR4_MCLK_EN        (1 << 19)
#define CI_CICR4_VSP            (1 << 20)
#define CI_CICR4_HSP            (1 << 21)
#define CI_CICR4_PCP            (1 << 22)
#define CI_CICR4_PCLK_EN        (1 << 23)

/* CISR */
#define CI_CISR_IFO_0           (1 << 0)
#define CI_CISR_IFO_1           (1 << 1)
#define CI_CISR_IFO_2           (1 << 2)
#define CI_CISR_EOF             (1 << 3)
#define CI_CISR_SOF             (1 << 4)
#define CI_CISR_CDD             (1 << 5)
#define CI_CISR_CQD             (1 << 6)
#define CI_CISR_PAR_ERR         (1 << 7)
#define CI_CISR_EOL             (1 << 8)
#define CI_CISR_FEMPTY_0        (1 << 9)
#define CI_CISR_FEMPTY_1        (1 << 10)
#define CI_CISR_FEMPTY_2        (1 << 11)
#define CI_CISR_RDAV_0          (1 << 12)
#define CI_CISR_RDAV_1          (1 << 13)
#define CI_CISR_RDAV_2          (1 << 14)
#define CI_CISR_FTO             (1 << 15)

/* CIFR */
#define CI_CIFR_FEN0            (1 << 0)
#define CI_CIFR_FEN1            (1 << 1)
#define CI_CIFR_FEN2            (1 << 2)
#define CI_CIFR_RESETF          (1 << 3)
#define CI_CIFR_THL_SHIFT       4
#define CI_CIFR_THL_SMASK       0x3

#define CI_CIFR_THL_32          0
#define CI_CIFR_THL_64          1
#define CI_CIFR_THL_96          2

#define CI_CIFR_FLVL0_SHIFT     8
#define CI_CIFR_FLVL0_SMASK     0xFF
#define CI_CIFR_FLVL1_SHIFT     16
#define CI_CIFR_FLVL1_SMASK     0x7F
#define CI_CIFR_FLVL2_SHIFT     23
#define CI_CIFR_FLVL2_SMASK     0x7F

/* formats */
#define CI_RAW8                 0	/* RAW */
#define CI_RAW9                 1
#define CI_RAW10                2
#define CI_YCBCR422             3	/* YCBCR */
#define CI_YCBCR422_PLANAR      4	/* YCBCR Planaried */
#define CI_RGB444               5	/* RGB */
#define CI_RGB555               6
#define CI_RGB565               7
#define CI_RGB666               8
#define CI_RGB888               9
#define CI_RGBT555_0            10	/* RGB+Transparent bit 0 */
#define CI_RGBT888_0            11
#define CI_RGBT555_1            12	/* RGB+Transparent bit 1 */
#define CI_RGBT888_1            13
#define CI_RGB666_PACKED        14	/* RGB Packed */
#define CI_RGB888_PACKED        15
#define CI_INVALID_FORMAT       0xFF

#endif
