/*
 *  linux/drivers/char/davinci_pwm.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DaVinci PWM register definitions
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

#ifndef _DAVINCI_PWM_H
#define _DAVINCI_PWM_H

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {
	unsigned int pid;
	unsigned int pcr;
	unsigned int cfg;
	unsigned int start;
	unsigned int rpt;
	unsigned int per;
	unsigned int ph1d;
} davinci_pwmregs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile davinci_pwmregs *davinci_pwmregsovly;

#define PWM_MINORS		3
#define DM646X_PWM_MINORS	2
#define DM644X_PWM_MINORS	3
#define DM355_PWM_MINORS	4
#define DAVINCI_PWM_MINORS	DM355_PWM_MINORS /* MAX of all PWM_MINORS */

#define	PWMIOC_SET_MODE			0x01
#define	PWMIOC_SET_PERIOD		0x02
#define	PWMIOC_SET_DURATION		0x03
#define	PWMIOC_SET_RPT_VAL		0x04
#define	PWMIOC_START			0x05
#define	PWMIOC_STOP			0x06
#define	PWMIOC_SET_FIRST_PHASE_STATE	0x07
#define	PWMIOC_SET_INACT_OUT_STATE	0x08

#define	PWM_ONESHOT_MODE	0
#define	PWM_CONTINUOUS_MODE	1

#endif				/* _DAVINCI_PWM_H */
