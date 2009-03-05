/*******************************************************************************
*
*   Marvell Serial ATA Linux Driver
*   Copyright 2004
*   Marvell International Ltd.
*
* This software program (the "Program") is distributed by Marvell International
* ltd. under the terms of the GNU General Public License Version 2, June 1991
* (the "License").  You may use, redistribute and/or modify this Program in
* accordance with the terms and conditions of the License, a copy of which is
* available along with the Program in the license.txt file or by writing to the
* Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston,
* MA 02111-1307 or on the worldwide web at http:www.gnu.org/licenses/gpl.txt.
*
* THE PROGRAM IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
* IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE
* EXPRESSLY DISCLAIMED.  The License provides additional details about this
* warranty disclaimer.
*
* For more information about the Program or the License as it applies to the
* Program, please contact Marvell International Ltd. via its affiliate, Marvell
* Semiconductor, Inc., 700 First Avenue, Sunnyvale, CA 94010
*
********************************************************************************

/*includes*/
#include "mvOs.h"
#include "mvSata.h"
#include "mvIALTWSI.h"

MV_BOOLEAN waitForInterrupt (MV_SATA_ADAPTER *pAdapter)
{
    /* Timeout after 1 second */
    MV_U32 timeout = 10000;
    while ((!(MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET) & MV_BIT24)) &&
           timeout)
    {
        mvMicroSecondsDelay (pAdapter, 100);
        timeout --;
    }
    if (timeout == 0)
    {
        return MV_FALSE;
    }
    return MV_TRUE;
}

MV_BOOLEAN waitForStopBit (MV_SATA_ADAPTER *pAdapter)
{
    /* Timeout after 1 second */
    MV_U32 timeout = 10000;
    while ((MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, TWSI_CONTROL) & MV_BIT4) &&
           timeout)
    {
        mvMicroSecondsDelay (pAdapter, 100);
        timeout --;
    }
    if (timeout == 0)
    {
        return MV_FALSE;
    }
    return MV_TRUE;
}


/*******************************************************************************
* mvSataTWSIMasterInit - Initializes the TWSI mechanism integrated in the MV.
*
* DESCRIPTION:
*       This function initialize the TWSI mechanism and must be called before
*       any attemp to use the TWSI bus ( use this function on both master or
*       slave mode to initialize the TWSI mechanism).The function calculates the
*       parameters needed for the TWSI's freuency registers, resets the TWSI and
*       then enables it.
*
* INPUT:
*       i2cFreq - the desired frequency , values defind in mvSataTWSI.h and can be
*                 either _100KHZ or _400KHZ.
*       tclk    - The system's Tclock.
*
* OUTPUT:
*       TWSI mechanism is enabled.
*       mvSataTWSITclock is set to tclk value.
*
* RETURN:
*       the actual frequancy calculated and assigned to the TWSI baude - rate
*       register (for more details please see the TWSI section in the MV
*       datasheet).
*
*******************************************************************************/
MV_BOOLEAN mvSataTWSIMasterInit(MV_SATA_ADAPTER *pAdapter)
{
    MV_U32    actualN = 0,actualM = 0, val;

    if (pAdapter == NULL)
    {
        return MV_FALSE;
    }

    /* Not a 60X1 adapter ? */
    if (pAdapter->sataAdapterGeneration != MV_SATA_GEN_II)
    {
        return MV_FALSE;
    }

    /* Disable the TWSI and slave */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,0);
    /* Dummy read */
    MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET);
    /* Reset the TWSI logic */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_SOFT_RESET,0);

    /* Dummy read */
    MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress, MV_MAIN_INTERRUPT_CAUSE_REG_OFFSET);
    /* Clear control register */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,0);

    /* Set the baud-rate */
    val = 0;
    actualM = 4;
    actualN = 4;
    val |= ((actualM<<3) | actualN);
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE,val);
    /* Remove me - Why TWSI_ENABLE below ??!! --> this is only for slave */
    val = (TWSI_ENABLE | TWSI_ACK | TWSI_INT_ENABLE);
    /* Enable the TWSI and slave */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,val);
    return MV_TRUE;
}

MV_BOOLEAN twsiSend8bit (MV_SATA_ADAPTER *pAdapter, MV_U8 data, MV_U32 statusToCheck)
{
    MV_U32 regVal;
    /* Now Write the data (8-bit)*/
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_DATA,data);
    regVal = (TWSI_ENABLE | TWSI_ACK | TWSI_INT_ENABLE);
    /* Clear the interrupt */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,regVal);
    /* Poll for interrupt */
    if (waitForInterrupt(pAdapter) == MV_FALSE)
    {
        PRINT_DBG("Error - twsiSend8bit line %d\n",__LINE__);
        return MV_FALSE;
    }

    /* We have got interrupt */
    PRINT_DBG("Interrupt for ack received - status = %lx\n",
              MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE));
    /* Check that status is */
    if (MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE) != statusToCheck)
    {
        PRINT_DBG("Error - twsiSend8bit line %d\n",__LINE__);
        return MV_FALSE;
    }
    return MV_TRUE;
}

MV_BOOLEAN mvSataSendStartBit (MV_SATA_ADAPTER *pAdapter,
                               MV_BOOLEAN repeated)
{
    MV_U32 regVal;
    MV_U32 exitStatus;
    regVal = (TWSI_ENABLE | TWSI_ACK | TWSI_INT_ENABLE | TWSI_START_BIT);

    /* Enable the TWSI and slave and ack and send start bit*/
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,regVal);

    /* Poll for interrupt */
    if (waitForInterrupt(pAdapter) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataSendStartBit line %d\n",__LINE__);
    }

    /* OK - Start bit is out, now lets send the EEPROM Address */
    PRINT_DBG("Interrupt for ack received - status = %lx\n",
              MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE));

    exitStatus = (repeated == MV_TRUE) ? 0x10 : 0x8;

    /* Check that status is equals exit status (0x10 for repeated start bit) */
    if (MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE) != exitStatus)
    {
        PRINT_DBG("Error - mvSataSendStartBit line %d\n",__LINE__);
        return MV_FALSE;
    }
    return MV_TRUE;
}

MV_BOOLEAN mvSataSendEEPromAddr (MV_SATA_ADAPTER *pAdapter,
                                 MV_U8 deviceAddress,
                                 MV_BOOLEAN readOperation,
                                 MV_U32 exitStatus)
{
    MV_U32 regVal;
    /* Now Write the address (7-bit)*/
    if (readOperation == MV_TRUE)
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_DATA,(deviceAddress << 1) | MV_BIT0);
    }
    else
    {
        MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_DATA,deviceAddress << 1);
    }

    regVal = (TWSI_ENABLE | TWSI_ACK | TWSI_INT_ENABLE);
    /* Clear the interrupt */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,regVal);

    /* Poll for interrupt */
    if (waitForInterrupt(pAdapter) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataSendEEPromAddr line %d\n",__LINE__);
        return MV_FALSE;
    }

    /* OK - EEProm address is out, now lets send the EEPROM internal Address */
    PRINT_DBG("Interrupt for ack received - status = %lx\n",MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE));
    /* Check that status is 0x18 */
    if (MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE) != exitStatus)
    {
        PRINT_DBG("Error - mvSataSendEEPromAddr line %d\n",__LINE__);
        return MV_FALSE;
    }
    return MV_TRUE;
}

MV_BOOLEAN mvSataSendStartBitAllAddresses (MV_SATA_ADAPTER *pAdapter,
                                           MV_U32 deviceAddress,
                                           MV_U32 address,
                                           MV_BOOLEAN addrRange)
{
    if (mvSataSendStartBit (pAdapter,MV_FALSE) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataSendStartBitAllAddresses line %d\n",__LINE__);
        return MV_FALSE;
    }
    if (mvSataSendEEPromAddr (pAdapter,(MV_U8)deviceAddress, MV_FALSE, 0x18) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataSendStartBitAllAddresses line %d\n",__LINE__);
        return MV_FALSE;
    }

    if (addrRange == MV_TRUE)
    {
        if (twsiSend8bit (pAdapter, (address >> 8) & 0xff, 0x28) == MV_FALSE)
        {
            PRINT_DBG("Error - mvSataSendStartBitAllAddresses line %d\n",__LINE__);
            return MV_FALSE;
        }
    }
    if (twsiSend8bit (pAdapter, address, 0x28) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataSendStartBitAllAddresses line %d\n",__LINE__);
        return MV_FALSE;
    }
    return MV_TRUE;
}

/*******************************************************************************
* mvSataTWSIMasterEEPROMread - Read data from an EEPROM device
*
* DESCRIPTION:
*       This function reads data ( byte size ) from an EEPROM device and return
*       with the 'TWSI_DATA_STRUCT' structure.The data is valid only if the
*       structure's field 'errorCode' has the value 'TWSI_NO_ERROR'.
*
* INPUT:
*       deviceAddress - The target EEPROM device address(7 bit only) .
*       address - The desired offset to be read from ( can be 8 bit or more
*                 depending on the EEPROM device).
*       addrRange - TWSI_ADDR_RANGE value ( defined in i2c.h ) to indicate the
*                   EEPROM address range.
*
* OUTPUT:
*       None.
*
* RETURN:
*       TWSI_DATA_STRUCT struct containing the read data and the error-code.
*      NOTE:
*       Data is valid only if the errorCode is TWSI_NO_ERROR.
*
*******************************************************************************/
MV_BOOLEAN mvSataTWSIMasterEEPROMRead(MV_SATA_ADAPTER *pAdapter,
                                      MV_U8 deviceAddress,
                                      MV_U16 address,
                                      MV_U8_PTR data,
                                      MV_BOOLEAN addrRange)
{
    MV_U32 regVal;
    if (mvSataSendStartBitAllAddresses(pAdapter,deviceAddress,address,addrRange) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMRead line %d\n",__LINE__);
        return MV_FALSE;
    }

    /* Must wait ~5us before any new transaction - EEPROM restrications */
    mvMicroSecondsDelay(pAdapter, 5);
    /* Send again start bit */
    if (mvSataSendStartBit (pAdapter,MV_TRUE) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMRead line %d\n",__LINE__);
        return MV_FALSE;
    }
    /* Send again eeprom address, but with read indication */
    if (mvSataSendEEPromAddr (pAdapter,deviceAddress, MV_TRUE, 0x40) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMRead line %d\n",__LINE__);
        return MV_FALSE;
    }

    /* Now wait for the data */
    regVal = (TWSI_ENABLE | TWSI_INT_ENABLE);
    /* Clear the interrupt and no ack */
    MV_REG_WRITE_DWORD(pAdapter,TWSI_CONTROL,regVal);
    /* Poll for interrupt */
    if (waitForInterrupt(pAdapter) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMRead line %d\n",__LINE__);
        return MV_FALSE;
    }

    /* OK - EEProm address is out, now lets send the EEPROM internal Address */
    PRINT_DBG("Interrupt for ack received - status = %lx\n",
              MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE));
    /* Check that status is 0x58 */
    if (MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_STATUS_BAUDE_RATE) != 0x58)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMRead line %d\n",__LINE__);
        return MV_FALSE;
    }

    *data = MV_REG_READ_DWORD (pAdapter->adapterIoBaseAddress,TWSI_DATA);
    /* Send stop bit */
    regVal = (TWSI_ENABLE | TWSI_INT_ENABLE | TWSI_STOP_BIT);
    /* Clear the interrupt */
    MV_REG_WRITE_DWORD(pAdapter->adapterIoBaseAddress,TWSI_CONTROL,regVal);


    if (waitForStopBit (pAdapter) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMWrite line %d\n",__LINE__);
        return MV_FALSE;
    }


    /* Must wait ~10ms before any new transaction - EEPROM restrications */
    mvMicroSecondsDelay (pAdapter, 10000);
    return MV_TRUE;

}



/*******************************************************************************
* mvSataTWSIMasterEEPROMwrite - write data to an EEPROM device.
*
* DESCRIPTION:
*       This function writes to an EEPROM device the data ( byte size ) within
*       the i2cData struct .
*
* INPUT:
*       deviceAddress - The target device address to be written.
*       *pTWSIData     - struct containing the data to be written.
*       address       - The desired offset to be written within the EEPROM.
*       addrRange     - The address range of the target EEPROM ( values defined
*                       in mvSataTWSI.h).
*       counterNum    - The counter-timer number (one out of 8 possible values
*                       defined in mvCntmr.h) needed for the delay between each
*                       write transaction.
*
* OUTPUT:
*       None.
*
* RETURN:
*       None.
*
*******************************************************************************/
MV_BOOLEAN mvSataTWSIMasterEEPROMWrite(MV_SATA_ADAPTER *pAdapter,
                                       MV_U8 deviceAddress,
                                       MV_U8 data,
                                       MV_U16 address,
                                       MV_BOOLEAN addrRange)
{
    MV_U32 regVal;

    if (mvSataSendStartBitAllAddresses(pAdapter,deviceAddress,address,addrRange) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMWrite line %d\n",__LINE__);
        return MV_FALSE;
    }

    if (twsiSend8bit (pAdapter, data, 0x28) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMWrite line %d\n",__LINE__);
        return MV_FALSE;
    }

    /* Send stop bit */
    regVal = (TWSI_ENABLE | TWSI_ACK | TWSI_INT_ENABLE | TWSI_STOP_BIT);
    /* Clear the interrupt */
    MV_REG_WRITE_DWORD(pAdapter,TWSI_CONTROL,regVal);


    if (waitForStopBit (pAdapter) == MV_FALSE)
    {
        PRINT_DBG("Error - mvSataTWSIMasterEEPROMWrite line %d\n",__LINE__);
        return MV_FALSE;
    }
    /* Need to wait 10ms before next write (EEPROM restrications)*/
    mvMicroSecondsDelay (pAdapter, 10000);
    return MV_TRUE;
}


