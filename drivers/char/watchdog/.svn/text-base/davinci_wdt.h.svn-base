/*
 *  linux/drivers/char/watchdog/davinci_wdt.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DaVinci Watchdog timer register definitions
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
 */

#ifndef _DAVINCI_WATCHDOG_H
#define _DAVINCI_WATCHDOG_H

#define DAVINCI_WDT_BASE		IO_ADDRESS(0x01C21C00)

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {
	u32 pid12;
	u16 emu_clk;
	u8 rsvd0[10];
	u32 tim12;
	u32 tim34;
	u32 prd12;
	u32 prd34;
	u32 tcr;
	u16 tgcr;
	u8 rsvd1[2];
	u16 wdtcr;
	u16 wdkey;
} davinci_wdtregs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile davinci_wdtregs *davinci_wdtregsovly;

struct wdt_davinci_device {
	davinci_wdtregsovly regs;
};

#define TIMER_MARGIN_MAX_VAL		(0xFFFFFFFF / CLOCK_TICK_RATE)
#define TIMER_MARGIN_CFG_VAL		64	/* user configurable value */
#define TIMER_MARGIN_DEF_VAL		64	/* Default is 64 seconds */
#define TIMER_MARGIN_MIN_VAL		1

#endif				/* _DAVINCI_WATCHDOG_H */
