/*
 *  linux/include/asm-arm/arch-davinci/irqs.h
 *
 *  Copyright (C) 2006 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/**************************************************************************
 * Included Files
 **************************************************************************/

/**************************************************************************
 * Definitions
 **************************************************************************/

/* These interrupt definitions are common for DaVinci and DM355 */
#define IRQ_ASQINT		9
#define IRQ_IMXINT		10
#define IRQ_USBINT		12
#define IRQ_PSCINT		20
#define IRQ_MBXINT		24
#define IRQ_MBRINT		25
#define IRQ_MMCINT		26
#define IRQ_PWMINT2		38

/* These interrupt definitions are common to all DaVinci variants */
#define IRQ_CCINT0		16
#define IRQ_CCERRINT		17
#define IRQ_TCERRINT0		18
#define IRQ_TCERRINT		19
#define IRQ_TCERRINT1		19
#define IRQ_DDRINT		29
#define IRQ_AEMIFINT		30
#define IRQ_TINT0		32
#define IRQ_TINT0_TINT12	32
#define IRQ_TINT1		33
#define IRQ_TINT0_TINT34	33
#define IRQ_TINT2		34
#define IRQ_TINT1_TINT12	34
#define IRQ_TINT3		35
#define IRQ_TINT1_TINT34	35
#define IRQ_PWMINT0		36
#define IRQ_PWMINT1		37
#define IRQ_I2C			39
#define IRQ_UARTINT0		40
#define IRQ_UARTINT1		41
#define IRQ_COMMTX		61
#define IRQ_COMMRX		62
#define IRQ_EMUINT		63

/*
 * These interrupt definitions are not common among all DaVinci variants,
 * but are defined here for backwards compatibility with the original
 * DaVinci DM6446 implementation..
 */
#define IRQ_VDINT0		0
#define IRQ_VDINT1		1
#define IRQ_VDINT2		2
#define IRQ_HISTINT		3
#define IRQ_H3AINT		4
#define IRQ_PRVUINT		5
#define IRQ_RSZINT		6
#define IRQ_VFOCINT		7
#define IRQ_VENCINT		8
#define IRQ_VLCDINT		11
#define IRQ_EMACINT		13
#define IRQ_IEEE1394INT		14
#define IRQ_IEEE1394WK		15
#define IRQ_IDE			22
#define IRQ_HPIINT		23
#define IRQ_SDIOINT		27
#define IRQ_MSINT		28
#define IRQ_VLQINT		31
#define IRQ_UARTINT2		42
#define IRQ_SPINT0		43
#define IRQ_SPINT1		44
#define IRQ_DSP2ARM0		46
#define IRQ_DSP2ARM1		47
#define IRQ_GPIO0		48
#define IRQ_GPIO1		49
#define IRQ_GPIO2		50
#define IRQ_GPIO3		51
#define IRQ_GPIO4		52
#define IRQ_GPIO5		53
#define IRQ_GPIO6		54
#define IRQ_GPIO7		55
#define IRQ_GPIOBNK0		56
#define IRQ_GPIOBNK1		57
#define IRQ_GPIOBNK2		58
#define IRQ_GPIOBNK3		59
#define IRQ_GPIOBNK4		60

/* DaVinci DM644x-specific Interrupts */
#define IRQ_DM644X_VDINT0	0
#define IRQ_DM644X_VDINT1	1
#define IRQ_DM644X_VDINT2	2
#define IRQ_DM644X_HISTINT	3
#define IRQ_DM644X_H3AINT	4
#define IRQ_DM644X_PRVUINT	5
#define IRQ_DM644X_RSZINT	6
#define IRQ_DM644X_VENCINT	8
#define IRQ_DM644X_VLCDINT	11
#define IRQ_DM644X_EMACINT	13
#define IRQ_DM644X_IDE		22
#define IRQ_DM644X_SDIOINT	27
#define IRQ_DM644X_VLQINT	31
#define IRQ_DM644X_UARTINT2	42
#define IRQ_DM644X_SPINT0	43
#define IRQ_DM644X_SPINT1	44
#define IRQ_DM644X_DSP2ARM0	46
#define IRQ_DM644X_DSP2ARM1	47
#define IRQ_DM644X_GPIO0	48
#define IRQ_DM644X_GPIO1	49
#define IRQ_DM644X_GPIO2	50
#define IRQ_DM644X_GPIO3	51
#define IRQ_DM644X_GPIO4	52
#define IRQ_DM644X_GPIO5	53
#define IRQ_DM644X_GPIO6	54
#define IRQ_DM644X_GPIO7	55
#define IRQ_DM644X_GPIOBNK0	56
#define IRQ_DM644X_GPIOBNK1	57
#define IRQ_DM644X_GPIOBNK2	58
#define IRQ_DM644X_GPIOBNK3	59
#define IRQ_DM644X_GPIOBNK4	60

/* DaVinci DM355-specific Interrupts */
#define IRQ_DM355_CCDC_VDINT0	0
#define IRQ_DM355_CCDC_VDINT1	1
#define IRQ_DM355_CCDC_VDINT2	2
#define IRQ_DM355_IPIPE_HST 	3	
#define IRQ_DM355_H3AINT	4
#define IRQ_DM355_IPIPE_SDR 	5	
#define IRQ_DM355_IPIPEIFINT	6
#define IRQ_DM355_OSDINT        7	
#define IRQ_DM355_VENCINT       8	
#define IRQ_DM355_IMCOPINT	11
#define IRQ_DM355_RTOINT	13
#define IRQ_DM355_TINT4		13
#define IRQ_DM355_TINT2_TINT12	13
#define IRQ_DM355_UARTINT2	14
#define IRQ_DM355_TINT5		14
#define IRQ_DM355_TINT2_TINT34	14
#define IRQ_DM355_TINT6		15
#define IRQ_DM355_TINT3_TINT12	15
#define IRQ_DM355_SPINT1_0	17
#define IRQ_DM355_SPINT1_1	18
#define IRQ_DM355_SPINT2_0	19
#define IRQ_DM355_SPINT2_1	21
#define IRQ_DM355_TINT7		22
#define IRQ_DM355_TINT3_TINT34	22
#define IRQ_DM355_SDIOINT0	23
#define IRQ_DM355_MMCINT0	26
#define IRQ_DM355_MSINT		26
#define IRQ_DM355_MMCINT1	27
#define IRQ_DM355_PWMINT3	28
#define IRQ_DM355_SDIOINT1	31
#define IRQ_DM355_SPINT0_0	42
#define IRQ_DM355_SPINT0_1	43
#define IRQ_DM355_GPIO0		44
#define IRQ_DM355_GPIO1		45
#define IRQ_DM355_GPIO2		46
#define IRQ_DM355_GPIO3		47
#define IRQ_DM355_GPIO4		48
#define IRQ_DM355_GPIO5		49
#define IRQ_DM355_GPIO6		50
#define IRQ_DM355_GPIO7		51
#define IRQ_DM355_GPIO8		52
#define IRQ_DM355_GPIO9		53
#define IRQ_DM355_GPIOBNK0	54
#define IRQ_DM355_GPIOBNK1	55
#define IRQ_DM355_GPIOBNK2	56
#define IRQ_DM355_GPIOBNK3	57
#define IRQ_DM355_GPIOBNK4	58
#define IRQ_DM355_GPIOBNK5	59
#define IRQ_DM355_GPIOBNK6	60

/* DaVinci DM6467-specific Interrupts */
#define IRQ_DM646X_VP_VERTINT0	0
#define IRQ_DM646X_VP_VERTINT1	1
#define IRQ_DM646X_VP_VERTINT2	2
#define IRQ_DM646X_VP_VERTINT3	3
#define IRQ_DM646X_VP_ERRINT	4
#define IRQ_DM646X_RESERVED_1	5
#define IRQ_DM646X_RESERVED_2	6
#define IRQ_DM646X_WDINT	7
#define IRQ_DM646X_CRGENINT0	8
#define IRQ_DM646X_CRGENINT1	9
#define IRQ_DM646X_TSIFINT0	10
#define IRQ_DM646X_TSIFINT1	11
#define IRQ_DM646X_VDCEINT	12
#define IRQ_DM646X_USBINT	13
#define IRQ_DM646X_USBDMAINT	14
#define IRQ_DM646X_PCIINT	15
#define IRQ_DM646X_TCERRINT2	20
#define IRQ_DM646X_TCERRINT3	21
#define IRQ_DM646X_IDE		22
#define IRQ_DM646X_HPIINT	23
#define IRQ_DM646X_EMACRXTHINT	24
#define IRQ_DM646X_EMACRXINT	25
#define IRQ_DM646X_EMACTXINT	26
#define IRQ_DM646X_EMACMISCINT	27
#define IRQ_DM646X_MCASP0TXINT	28
#define IRQ_DM646X_MCASP0RXINT	29
#define IRQ_DM646X_RESERVED_3	31
#define IRQ_DM646X_MCASP1TXINT	32
#define IRQ_DM646X_VLQINT	38
#define IRQ_DM646X_UARTINT2	42
#define IRQ_DM646X_SPINT0	43
#define IRQ_DM646X_SPINT1	44
#define IRQ_DM646X_DSP2ARMINT	45
#define IRQ_DM646X_RESERVED_4	46
#define IRQ_DM646X_PSCINT	47
#define IRQ_DM646X_GPIO0	48
#define IRQ_DM646X_GPIO1	49
#define IRQ_DM646X_GPIO2	50
#define IRQ_DM646X_GPIO3	51
#define IRQ_DM646X_GPIO4	52
#define IRQ_DM646X_GPIO5	53
#define IRQ_DM646X_GPIO6	54
#define IRQ_DM646X_GPIO7	55
#define IRQ_DM646X_GPIOBNK0	56
#define IRQ_DM646X_GPIOBNK1	57
#define IRQ_DM646X_GPIOBNK2	58
#define IRQ_DM646X_DDRINT	59
#define IRQ_DM646X_AEMIFINT	60

#define DAVINCI_N_AINTC_IRQ	64
#ifdef CONFIG_ARCH_DAVINCI_DM644x
#define DAVINCI_N_GPIO		71
#elif CONFIG_ARCH_DAVINCI_DM355
#define DAVINCI_N_GPIO		104
#else
#define DAVINCI_N_GPIO		48
#endif

#define NR_IRQS			(DAVINCI_N_AINTC_IRQ + DAVINCI_N_GPIO)

#define ARCH_TIMER_IRQ		IRQ_TINT1_TINT34

#endif /* __ASM_ARCH_IRQS_H */
