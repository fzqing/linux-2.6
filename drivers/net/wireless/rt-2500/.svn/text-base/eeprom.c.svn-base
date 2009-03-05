/*************************************************************************** 
 * RT2400/RT2500 SourceForge Project - http://rt2x00.serialmonkey.com      * 
 *                                                                         * 
 *   This program is free software; you can redistribute it and/or modify  * 
 *   it under the terms of the GNU General Public License as published by  * 
 *   the Free Software Foundation; either version 2 of the License, or     * 
 *   (at your option) any later version.                                   * 
 *                                                                         * 
 *   This program is distributed in the hope that it will be useful,       * 
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        * 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         * 
 *   GNU General Public License for more details.                          * 
 *                                                                         * 
 *   You should have received a copy of the GNU General Public License     * 
 *   along with this program; if not, write to the                         * 
 *   Free Software Foundation, Inc.,                                       * 
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             * 
 *                                                                         * 
 *   Licensed under the GNU GPL                                            * 
 *   Original code supplied under license from RaLink Inc, 2004.           * 
 ***************************************************************************/ 

 /*************************************************************************** 
 *      Module Name: eeprom.c 
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#include    "rt_config.h"

VOID RaiseClock(
    IN  PRTMP_ADAPTER   pAd,
    IN  ULONG *x)
{
    *x = *x | EESK;
    RTMP_IO_WRITE32(pAd, CSR21, *x);
    udelay(1);
}

VOID LowerClock(
    IN  PRTMP_ADAPTER   pAd,
    IN  ULONG *x)
{
    *x = *x & ~EESK;
    RTMP_IO_WRITE32(pAd, CSR21, *x);
    udelay(1);
}

USHORT ShiftInBits(
    IN  PRTMP_ADAPTER   pAd)
{
    ULONG       x,i;
    USHORT      data=0;

    RTMP_IO_READ32(pAd, CSR21, &x);

    x &= ~( EEDO | EEDI);

    for(i=0; i<16; i++)
    {
        data = data << 1;
        RaiseClock(pAd, &x);

        RTMP_IO_READ32(pAd, CSR21, &x);

        x &= ~(EEDI);
        if(x & EEDO)
            data |= 1;

        LowerClock(pAd, &x);
    }

    return data;
}

VOID ShiftOutBits(
    IN  PRTMP_ADAPTER   pAd,
    IN  USHORT data,
    IN  USHORT count)
{
    ULONG       x,mask;

    mask = 0x01 << (count - 1);
    RTMP_IO_READ32(pAd, CSR21, &x);

    x &= ~(EEDO | EEDI);

    do
    {
        x &= ~EEDI;
        if(data & mask)     x |= EEDI;

        RTMP_IO_WRITE32(pAd, CSR21, x);

        RaiseClock(pAd, &x);
        LowerClock(pAd, &x);

        mask = mask >> 1;
    } while(mask);

    x &= ~EEDI;
    RTMP_IO_WRITE32(pAd, CSR21, x);
}

VOID EEpromCleanup(
    IN  PRTMP_ADAPTER   pAd)
{
    ULONG x;

    RTMP_IO_READ32(pAd, CSR21, &x);

    x &= ~(EECS | EEDI);
    RTMP_IO_WRITE32(pAd, CSR21, x);

    RaiseClock(pAd, &x);
    LowerClock(pAd, &x);
}

VOID EWEN(
    IN  PRTMP_ADAPTER   pAd)
{
    ULONG   x;

    // reset bits and set EECS
    RTMP_IO_READ32(pAd, CSR21, &x);
    x &= ~(EEDI | EEDO | EESK);
    x |= EECS;
    RTMP_IO_WRITE32(pAd, CSR21, x);

    // kick a pulse
    RaiseClock(pAd, &x);
    LowerClock(pAd, &x);

    // output the read_opcode and six pulse in that order
    ShiftOutBits(pAd, EEPROM_EWEN_OPCODE, 5);
    ShiftOutBits(pAd, 0, 6);

    EEpromCleanup(pAd);
}

VOID EWDS(
    IN  PRTMP_ADAPTER   pAd)
{
    ULONG   x;

    // reset bits and set EECS
    RTMP_IO_READ32(pAd, CSR21, &x);
    x &= ~(EEDI | EEDO | EESK);
    x |= EECS;
    RTMP_IO_WRITE32(pAd, CSR21, x);

    // kick a pulse
    RaiseClock(pAd, &x);
    LowerClock(pAd, &x);

    // output the read_opcode and six pulse in that order
    ShiftOutBits(pAd, EEPROM_EWDS_OPCODE, 5);
    ShiftOutBits(pAd, 0, 6);

    EEpromCleanup(pAd);
}

USHORT RTMP_EEPROM_READ16(
    IN  PRTMP_ADAPTER   pAd,
    IN  USHORT Offset)
{
    ULONG       x;
    USHORT      data;

    Offset /= 2;
    // reset bits and set EECS
    RTMP_IO_READ32(pAd, CSR21, &x);
    x &= ~(EEDI | EEDO | EESK);
    x |= EECS;
    RTMP_IO_WRITE32(pAd, CSR21, x);

    // kick a pulse
    RaiseClock(pAd, &x);
    LowerClock(pAd, &x);

    // output the read_opcode and register number in that order    
    ShiftOutBits(pAd, EEPROM_READ_OPCODE, 3);
    ShiftOutBits(pAd, Offset, pAd->EEPROMAddressNum);

    // Now read the data (16 bits) in from the selected EEPROM word
    data = ShiftInBits(pAd);

    EEpromCleanup(pAd);

    return data;
}   //ReadEEprom

VOID RTMP_EEPROM_WRITE16(
    IN  PRTMP_ADAPTER   pAd,
    IN  USHORT Offset,
    IN  USHORT Data)
{
    ULONG   i, x;

    Offset /= 2;

    EWEN(pAd);

    // reset bits and set EECS
    RTMP_IO_READ32(pAd, CSR21, &x);
    x &= ~(EEDI | EEDO | EESK);
    x |= EECS;
    RTMP_IO_WRITE32(pAd, CSR21, x);

    // kick a pulse
    RaiseClock(pAd, &x);
    LowerClock(pAd, &x);

    // output the read_opcode ,register number and data in that order
    ShiftOutBits(pAd, EEPROM_WRITE_OPCODE, 3);
    ShiftOutBits(pAd, Offset, pAd->EEPROMAddressNum);
    ShiftOutBits(pAd, Data, 16);        // 16-bit access

    // read DO status
    RTMP_IO_READ32(pAd, CSR21, &x);

    EEpromCleanup(pAd);

    for(i=0; i<10; i++)
        udelay(1000);       //delay for twp(MAX)=10ms

    EWDS(pAd);

    EEpromCleanup(pAd);
}


