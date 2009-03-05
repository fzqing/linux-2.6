/*
 * drivers/video/smi/smi_hw.h
 *
 * LynxEM+/EM4+(Silicon Motion Inc.) fb driver	for NEC Electronics Corporation VR5701 SolutionGearII
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __SMI_HW_H__
#define __SMI_HW_H__

#include "smifb.h"

#define DPPORT_BASE_OFFSET		0x400000
#define DP_BASE_OFFSET			0x408000
#define VP_BASE_OFFSET			0x40c000
#define CP_BASE_OFFSET			0x40e000
#define IO_BASE_OFFSET			0x700000

#define DPPORT_REGION_SIZE		(32*1024)
#define DPREG_REGION_SIZE		(16*1024)
#define VPREG_REGION_SIZE		(8*1024)
#define CPREG_REGION_SIZE		(8*1024)
#define MMIO_REGION_SIZE		(1*1024*1024)

#define LYNX3DM_DPPORT_BASE_OFFSET		0x100000
#define LYNX3DM_DP_BASE_OFFSET			0x000000
#define LYNX3DM_VP_BASE_OFFSET			0x000800
#define LYNX3DM_CP_BASE_OFFSET			0x001000
#define LYNX3DM_IO_BASE_OFFSET			0x0c0000
#define LYNX3DM_FB_BASE_OFFSET			0x200000

#define LYNX3DM_DPPORT_REGION_SIZE		(1024*1024)
#define LYNX3DM_DPREG_REGION_SIZE		(2*1024)
#define LYNX3DM_VPREG_REGION_SIZE		(2*1024)
#define LYNX3DM_CPREG_REGION_SIZE		(2*1024)
#define LYNX3DM_MMIO_REGION_SIZE		(256*1024)

extern void smi_set_moderegs(struct smifb_info *sinfo,
			     int bpp, int width, int height,
			     int hDisplaySize,
			     int hDisplay, int hStart, int hEnd, int hTotal,
			     int vDisplay, int vStart, int vEnd, int vTotal,
			     int dotClock, int sync);

#define MMIO_OUT8(p, r, d)	(((volatile u8 *)(p))[r] = (d))
#define MMIO_OUT16(p, r, d)	(((volatile u16 *)(p))[(r)>>1] = (d))
#define MMIO_OUT32(p, r, d)	(((volatile u32 *)(p))[(r)>>2] = (d))
#define MMIO_IN8(p, r)		(((volatile u8 *)(p))[(r)])
#define MMIO_IN16(p, r)		(((volatile u16 *)(p))[(r)>>1])
#define MMIO_IN32(p, r)		(((volatile u32 *)(p))[(r)>>2])

static inline u8 VGA_READ8(u8 * base, uint reg)
{
	return MMIO_IN8(base, reg);
}

static inline void VGA_WRITE8(u8 * base, uint reg, u8 data)
{
	MMIO_OUT8(base, reg, data);
}

static inline u8 VGA_READ8_INDEX(u8 * base, u8 index)
{
	VGA_WRITE8(base, 0x3c4, index);
	return VGA_READ8(base, 0x3c5);
}

static inline void VGA_WRITE8_INDEX(u8 * base, u8 index, u8 data)
{
	VGA_WRITE8(base, 0x3c4, index);
	VGA_WRITE8(base, 0x3c5, data);
}

static inline u8 regSR_read(u8 * base, u8 index)
{
	VGA_WRITE8(base, 0x3c4, index);
	return VGA_READ8(base, 0x3c5);
}

static inline void regSR_write(u8 * base, u8 index, u8 data)
{
	VGA_WRITE8(base, 0x3c4, index);
	VGA_WRITE8(base, 0x3c5, data);
}

static inline u8 regCR_read(u8 * base, u8 index)
{
	VGA_WRITE8(base, 0x3d4, index);
	return VGA_READ8(base, 0x3d5);
}

static inline void regCR_write(u8 * base, u8 index, u8 data)
{
	VGA_WRITE8(base, 0x3d4, index);
	VGA_WRITE8(base, 0x3d5, data);
}

static inline u8 regGR_read(u8 * base, u8 index)
{
	VGA_WRITE8(base, 0x3ce, index);
	return VGA_READ8(base, 0x3cf);
}

static inline void regGR_write(u8 * base, u8 index, u8 data)
{
	VGA_WRITE8(base, 0x3ce, index);
	VGA_WRITE8(base, 0x3cf, data);
}

static inline u8 regAR_read(u8 * base, u8 index)
{
	(void)VGA_READ8(base, 0x3da);	/* reset flip-flop */
	VGA_WRITE8(base, 0x3c0, index);
	return VGA_READ8(base, 0x3c1);
}

static inline void regAR_write(u8 * base, u8 index, u8 data)
{
	(void)VGA_READ8(base, 0x3da);	/* reset flip-flop */
	VGA_WRITE8(base, 0x3c0, index);
	VGA_WRITE8(base, 0x3c0, data);
}

/*
 * LynxEM+ registers
 */

/* Drawing Engine Control Registers */
#define reg_DPR00(x)	*(u16 *)((x)->dpr+0x00)	/* Source Y or K2                                       */
#define reg_DPR02(x)	*(u16 *)((x)->dpr+0x02)	/* Source X or K1                                       */
#define reg_DPR04(x)	*(u16 *)((x)->dpr+0x04)	/* Destination Y or Start Y                             */
#define reg_DPR06(x)	*(u16 *)((x)->dpr+0x06)	/* Destination X or Start X                             */
#define reg_DPR08(x)	*(u16 *)((x)->dpr+0x08)	/* Dimension Y or Error Term                            */
#define reg_DPR0A(x)	*(u16 *)((x)->dpr+0x0A)	/* Dimension X or Vector Length                         */
#define reg_DPR0C(x)	*(u16 *)((x)->dpr+0x0C)	/* ROP and Miscellaneous Control                        */
#define reg_DPR0E(x)	*(u16 *)((x)->dpr+0x0E)	/* Drawing Engine Commands and Control                  */
#define reg_DPR10(x)	*(u16 *)((x)->dpr+0x10)	/* Source Row Pitch                                     */
#define reg_DPR12(x)	*(u16 *)((x)->dpr+0x12)	/* Destination Row Picth                                */
#define reg_DPR14(x)	*(u32 *)((x)->dpr+0x14)	/* Foreground Colors                                    */
#define reg_DPR18(x)	*(u32 *)((x)->dpr+0x18)	/* Background Colors                                    */
#define reg_DPR1C(x)	*(u16 *)((x)->dpr+0x1C)	/* Stretch Source Height Y                              */
#define reg_DPR1E(x)	*(u16 *)((x)->dpr+0x1E)	/* Drawing Engine DataFormat and Location Format Select */
#define reg_DPR20(x)	*(u32 *)((x)->dpr+0x20)	/* Color Compare                                        */
#define reg_DPR24(x)	*(u32 *)((x)->dpr+0x24)	/* Color Compare Mask                                   */
#define reg_DPR28(x)	*(u16 *)((x)->dpr+0x28)	/* Bit Mask                                             */
#define reg_DPR2A(x)	*(u16 *)((x)->dpr+0x2A)	/* Byte Mask Enable                                     */
#define reg_DPR2C(x)	*(u16 *)((x)->dpr+0x2C)	/* Scisors Left and Control                             */
#define reg_DPR2E(x)	*(u16 *)((x)->dpr+0x2E)	/* Scisors Top                                          */
#define reg_DPR30(x)	*(u16 *)((x)->dpr+0x30)	/* Scisors Right                                        */
#define reg_DPR32(x)	*(u16 *)((x)->dpr+0x32)	/* Scisors Bottom                                       */
#define reg_DPR34(x)	*(u32 *)((x)->dpr+0x34)	/* Mono Pattern Low                                     */
#define reg_DPR38(x)	*(u32 *)((x)->dpr+0x38)	/* Mono Pattern High                                    */
#define reg_DPR3C(x)	*(u32 *)((x)->dpr+0x3C)	/* XY Addressing Destination & Source Window Widths     */
#define reg_DPR40(x)	*(u32 *)((x)->dpr+0x40)	/* Source Base Address                                  */
#define reg_DPR44(x)	*(u32 *)((x)->dpr+0x44)	/* Destination Base Address                             */

/* Video Processor Control Registers */
#define reg_VPR00(x)	*(u32 *)((x)->vpr+0x00)	/* Miscellaneous Graphics and Video Control                 */
#define reg_VPR04(x)	*(u32 *)((x)->vpr+0x04)	/* Color Keys                                               */
#define reg_VPR08(x)	*(u32 *)((x)->vpr+0x08)	/* Color Key Masks                                          */
#define reg_VPR0C(x)	*(u32 *)((x)->vpr+0x0C)	/* Data Source Start Address for Extended Graphics Modes    */
#define reg_VPR10(x)	*(u32 *)((x)->vpr+0x10)	/* Data Source Width and Offset for Extended Graphics Modes */
#define reg_VPR14(x)	*(u32 *)((x)->vpr+0x14)	/* Video Window I Left and Top Boundaries                   */
#define reg_VPR18(x)	*(u32 *)((x)->vpr+0x18)	/* Video Window I Right and Bottom Boundaries               */
#define reg_VPR1C(x)	*(u32 *)((x)->vpr+0x1C)	/* Video Window I Source Start Address                      */
#define reg_VPR20(x)	*(u32 *)((x)->vpr+0x20)	/* Video Window I Source Width and Offset                   */
#define reg_VPR24(x)	*(u32 *)((x)->vpr+0x24)	/* Video Window I Stretch Factor                            */
#define reg_VPR28(x)	*(u32 *)((x)->vpr+0x28)	/* Video Window II Left and Top Boundaries              */
#define reg_VPR2C(x)	*(u32 *)((x)->vpr+0x2C)	/* Video Window II Right and Bottom Boundaries              */
#define reg_VPR30(x)	*(u32 *)((x)->vpr+0x30)	/* Video Window II Source Start Address                     */
#define reg_VPR34(x)	*(u32 *)((x)->vpr+0x34)	/* Video Window II Source Width and Offset                  */
#define reg_VPR38(x)	*(u32 *)((x)->vpr+0x38)	/* Video Window II Stretch Factor                           */
#define reg_VPR3C(x)	*(u32 *)((x)->vpr+0x3C)	/* Graphics and Video Controll II                           */
#define reg_VPR40(x)	*(u32 *)((x)->vpr+0x40)	/* Graphic Scale Factor                                     */
#define reg_VPR54(x)	*(u32 *)((x)->vpr+0x54)	/* FIFO Priority Control                                    */
#define reg_VPR58(x)	*(u32 *)((x)->vpr+0x58)	/* FIFO Empty Request level Control                         */
#define reg_VPR5C(x)	*(u32 *)((x)->vpr+0x5C)	/* YUV to RGB Conversion Constant                           */
#define reg_VPR60(x)	*(u32 *)((x)->vpr+0x60)	/* Current Scan Line Position                               */
#define reg_VPR64(x)	*(u32 *)((x)->vpr+0x64)	/* Signature Analyzer Control and Status                    */
#define reg_VPR68(x)	*(u32 *)((x)->vpr+0x68)	/* Video Window I Stretch Factor                            */
#define reg_VPR6C(x)	*(u32 *)((x)->vpr+0x6C)	/* Video Window II Stretch Factor                           */

/* Capture Processor Control Registers */
#define reg_CPR00(x)	*(u32 *)((x)->cpr+0x00)	/* Capture Port Control                        */
#define reg_CPR04(x)	*(u32 *)((x)->cpr+0x04)	/* Video Source Clipping Control               */
#define reg_CPR08(x)	*(u32 *)((x)->cpr+0x08)	/* Video Source Capture Size Control           */
#define reg_CPR0C(x)	*(u32 *)((x)->cpr+0x0C)	/* Capture Port Buffer I Source Start Address  */
#define reg_CPR10(x)	*(u32 *)((x)->cpr+0x10)	/* Capture Port Buffer II Source Start Address */
#define reg_CPR14(x)	*(u32 *)((x)->cpr+0x14)	/* Capture Port Source Offset Address          */
#define reg_CPR18(x)	*(u32 *)((x)->cpr+0x18)	/* Capture FIFO Empty Request level Control    */

#endif				/* __SMI_HW_H__ */
