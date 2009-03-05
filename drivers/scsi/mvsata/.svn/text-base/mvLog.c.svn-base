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
* mvLog - C File for implementation of the core driver logger
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*
*******************************************************************************/
#include "mvOs.h"



#if defined (MV_LOG_DEBUG) || defined (MV_LOG_ERROR)

char* mvLogMsgType[MV_MAX_MESSAGE_TYPE] = {
    " (FATAL_ERROR) ",
    " (ERROR) ",
    " (DEBUG INIT) ",
    " (DEBUG_INTERRUPTS) ",
    " (DEBUG SATA LINK) ",
    " (DEBUG UDMA COMMAND) ",
    " (DEBUG NON UDMA COMMAND) ",
    " (DEBUG_PM) ",
    " (DEBUG) ",
};

static MV_LOG_FILTER_HEADER mvLogInstance[MV_MAX_LOG_MODULES] =
{
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
    {MV_FALSE, 0, NULL},
};

static char szMessageBuffer[1024];


MV_BOOLEAN mvLogRegisterModule(MV_U8 moduleId, MV_U32 filterMask, char* name)
{
    if (moduleId >= MV_MAX_LOG_MODULES)
    {
        return MV_FALSE;
    }
    if (mvLogInstance[moduleId].used == MV_TRUE)
    {
        return MV_FALSE;
    }
    if (name == NULL)
    {
        return MV_FALSE;
    }
    mvLogInstance[moduleId].filterMask = filterMask;
    mvLogInstance[moduleId].name = name;
    mvLogInstance[moduleId].used = MV_TRUE;
    return MV_TRUE;
}

MV_BOOLEAN mvLogSetModuleFilter(MV_U8 moduleId, MV_U32 filterMask)
{
    if (moduleId >= MV_MAX_LOG_MODULES)
    {
        return MV_FALSE;
    }
    if (mvLogInstance[moduleId].used == MV_FALSE)
    {
        return MV_FALSE;
    }
    mvLogInstance[moduleId].filterMask = filterMask;
    return MV_TRUE;
}


MV_U32 mvLogGetModuleFilter(MV_U8 moduleId)
{
    if (moduleId >= MV_MAX_LOG_MODULES)
    {
        return 0;
    }
    if (mvLogInstance[moduleId].used == MV_FALSE)
    {
        return 0;
    }
    return mvLogInstance[moduleId].filterMask;
}

void mvLogMsg(MV_U8 moduleId, MV_U32 type, char* format, ...)
{
    int len;
    va_list args;

    if (moduleId >= MV_MAX_LOG_MODULES)
    {
        return;
    }
    if ((moduleId != MV_RAW_MSG_ID) &&
        ((mvLogInstance[moduleId].used == MV_FALSE) ||
         ((mvLogInstance[moduleId].filterMask & type) == 0)))
    {
        return;
    }
    if ((moduleId != MV_RAW_MSG_ID) && (type & 0x1ff))
    {
        MV_U8 msgType = 0;
        /* find least significant 1*/
        while (msgType < MV_MAX_MESSAGE_TYPE)
        {
            if (type & ( 1 << msgType))
            {
                break;
            }
            msgType++;
        }
        len = sprintf(szMessageBuffer, "%s%s",
                      mvLogInstance[moduleId].name, mvLogMsgType[msgType]);
    }
    else
    {
        len = 0;
    }
    va_start(args, format);
    vsprintf(&szMessageBuffer[len], format, args);
    va_end(args);
    MV_LOG_PRINT(szMessageBuffer);
}
#endif
