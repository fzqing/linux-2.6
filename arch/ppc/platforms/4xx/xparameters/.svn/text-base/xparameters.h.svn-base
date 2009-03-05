/*
 * include/asm-ppc/xparameters.h
 *
 * This file includes the correct xparameters.h for the CONFIG'ed board
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#include <linux/config.h>

#if defined(CONFIG_XILINX_ML300)
#include <platforms/4xx/xparameters/xparameters_ml300.h>
#endif
#if defined(CONFIG_XILINX_ML40x)
#include <platforms/4xx/xparameters/xparameters_ml40x.h>
#endif
#if defined(CONFIG_MEMEC_2VPX)
#include <platforms/4xx/xparameters/xparameters_2vpx.h>
#endif

/*
 * A few reasonable defaults for the #defines which could be missing depending
 * on the IP version or variant (e.g. OPB vs PLB)
 */

#ifndef XPAR_EMAC_0_CAM_EXIST
#define XPAR_EMAC_0_CAM_EXIST 0
#endif
#ifndef XPAR_EMAC_1_CAM_EXIST
#define XPAR_EMAC_1_CAM_EXIST 0
#endif

#ifndef XPAR_EMAC_0_JUMBO_EXIST
#define XPAR_EMAC_0_JUMBO_EXIST 0
#endif
#ifndef XPAR_EMAC_1_JUMBO_EXIST
#define XPAR_EMAC_1_JUMBO_EXIST 0
#endif

#ifndef XPAR_GPIO_0_IS_DUAL
#define XPAR_GPIO_0_IS_DUAL 0
#endif
#ifndef XPAR_GPIO_1_IS_DUAL
#define XPAR_GPIO_1_IS_DUAL 0
#endif
#ifndef XPAR_GPIO_2_IS_DUAL
#define XPAR_GPIO_2_IS_DUAL 0
#endif
#ifndef XPAR_GPIO_3_IS_DUAL
#define XPAR_GPIO_3_IS_DUAL 0
#endif

