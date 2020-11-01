/*
 * MIT License
 *
 * Copyright (c) 2020 <Duy Lion Tran>. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file GNSS.c
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "UBX_GNSS.h"
#include "Miscellaneous.h"

#include "lwgps.h"

#define CRC_START_CALCULATING_BYTE  2
#define CRC_SIZE_BYTE               2

#define NUMBER_OF_BYTES_AVAILABLE_ADDRESS   0xFD  /**< High byte address = 0xFD, low byte address = 0xFE */
#define DATA_STREAM_ADDRESS                 0xFF


UBXGNSS_State_t UBXGNSS_Init(UBXGNSS_Def_t *locUBXGNSS_p)
{
    if ((NULL == locUBXGNSS_p) || (NULL == locUBXGNSS_p->commHandle))
    {
        return UBXGNSS_INVALID_PARAM;
    }

    if ((locUBXGNSS_p->interface < 0) && (locUBXGNSS_p->interface > UBXGNSS_SPI_INTERFACE))
    {
        return UBXGNSS_INVALID_INTERFACE;
    }

    /* Check communication */
    switch (locUBXGNSS_p->interface)
    {
        case UBXGNSS_UART_INTERFACE:
        {
            break;
        }

        case UBXGNSS_I2C_INTERFACE:
        {
            break;
        }

        /* Not fully support */
        case (UBXGNSS_UART_INTERFACE | UBXGNSS_I2C_INTERFACE):
        {
            break;
        }

        case UBXGNSS_SPI_INTERFACE:
        {
            break;
        }

        default: break;

    }

    /* Init GPS */
    lwgps_init(&(locUBXGNSS_p->gps));
    return UBXGNSS_OK;
}


void UBXGNSS_DeInit(UBXGNSS_Def_t *locUBXGNSS_p)
{

}


void UBXGNSS_ProcessData(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locCommData_p, uint16_t locCommDataSize_u16)
{
    uint16_t len = strlen(locCommData_p);

    if (('$' == locCommData_p[0]) && ('\r' == locCommData_p[len - 2]) && ('\n' == locCommData_p[len - 1]))
    {
        /* Process all input data */
        lwgps_process(&(locUBXGNSS_p->gps), locCommData_p, locCommDataSize_u16);
    }
}


UBXGNSS_State_t UBXGNSS_GetUBXMessage(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locMsg_p, uint16_t *locReadMsgSize_p)
{
    uint8_t locRegAddress;
    uint8_t locDataSizeAvailableBuff_u16[2];
    uint16_t locDataSizeAvailable_u16;
    UBXGNSS_State_t locRet;

    if ((NULL == locUBXGNSS_p) || (NULL == locUBXGNSS_p->commHandle))
    {
        return UBXGNSS_INVALID_PARAM;
    }

    switch (locUBXGNSS_p->interface)
    {
        case UBXGNSS_I2C_INTERFACE:

        /* Not fully support */
        case (UBXGNSS_UART_INTERFACE | UBXGNSS_I2C_INTERFACE):
        {
            locRegAddress = NUMBER_OF_BYTES_AVAILABLE_ADDRESS;
            /* Get number of available bytes from register 0xFD and 0xFE */
            locRet = locUBXGNSS_p->commHandle(I2C_UBX_EVENT_RECEIVE, locUBXGNSS_p->address, locRegAddress, locDataSizeAvailableBuff_u16, SIZE(locDataSizeAvailableBuff_u16), NULL);

            if (UBXGNSS_OK != locRet)
            {
                return UBXGNSS_I2C_COMM_FAILURE;
            }

            locDataSizeAvailable_u16 = (locDataSizeAvailableBuff_u16[0] << 8) | locDataSizeAvailableBuff_u16[1];

            if ((locDataSizeAvailable_u16 > 0) && (locDataSizeAvailable_u16 <= UBXGNSS_UBX_MAX_LENGHTH))
            {
                locRegAddress = DATA_STREAM_ADDRESS;

                locRet = locUBXGNSS_p->commHandle(I2C_UBX_EVENT_RECEIVE, locUBXGNSS_p->address, locRegAddress, locMsg_p, locDataSizeAvailable_u16, NULL);

                if (UBXGNSS_OK != locRet)
                {
                    return UBXGNSS_I2C_COMM_FAILURE;
                }

                *locReadMsgSize_p = locDataSizeAvailable_u16;
            }
            else
            {
                return UBXGNSS_DATA_NOT_AVAILABLE;
            }

            return locRet;
        }

        case UBXGNSS_SPI_INTERFACE:
        {

            break;
        }

        default: break;
    }
}


static void UBXGNSS_ProcessNav(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locNavMsg_p, uint16_t locNavMsgLen_u16)
{
    uint16_t locPayloadLength_u16 = ((locNavMsg_p[5] << 8) | locNavMsg_p[4]);

    /* Process message ID */
    switch (locNavMsg_p[3])
    {
        case 0x03:
        {
            if (16 == locPayloadLength_u16)
            {
                locUBXGNSS_p->timeToFirstFix = ((locNavMsg_p[UBX_MSG_HEADER_LENGHTH + 11] << 24) |
                                                (locNavMsg_p[UBX_MSG_HEADER_LENGHTH + 10] << 16) |
                                                (locNavMsg_p[UBX_MSG_HEADER_LENGHTH + 9] << 8) |
                                                (locNavMsg_p[UBX_MSG_HEADER_LENGHTH + 8]));
            }
            break;
        }

        default: break;
    }
}


UBXGNSS_State_t UBXGNSS_ProcessUBXMsg(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locMsg_p, uint16_t locMsgLen_u16)
{
    uint8_t locUBXCRC_au8[CRC_SIZE_BYTE];

    /* Message must contents at lease 1 byte data, 6 bytes header and 2 bytes CRC */
    if (locMsgLen_u16 <= 8)
    {
        return UBXGNSS_INVALID_MSG;
    }

    for (uint16_t i = CRC_START_CALCULATING_BYTE; i < (locMsgLen_u16 - 2); i++)
    {
        locUBXCRC_au8[0] += locMsg_p[i];
        locUBXCRC_au8[1] += locUBXCRC_au8[0];
    }

    if (0 != memcmp(&locMsg_p[locMsgLen_u16 - 2], locUBXCRC_au8, SIZE(locUBXCRC_au8)))
    {
        return UBXGNSS_CRC_FAIL;
    }

    /* Process message class */
    switch (locMsg_p[2])
    {
        case UBX_ACK:
        {
            break;
        }

        case UBX_CFG:
        {
            break;
        }

        case UBX_INF:
        {
            break;
        }

        case UBX_LOG:
        {
            break;
        }

        case UBX_MGA:
        {
            break;
        }

        case UBX_MON:
        {
            break;
        }

        case UBX_NAV:
        {
            UBXGNSS_ProcessNav(locUBXGNSS_p, locMsg_p, locMsgLen_u16 - CRC_SIZE_BYTE);
            break;
        }

        case UBX_RXM:
        {
            break;
        }

        case UBX_SEC:
        {
            break;
        }

        case UBX_TIM:
        {
            break;
        }

        case UBX_UPD:
        {
            break;
        }

        default:
        {
            return UBXGNSS_INVALID_MSG;
        }
    }

    return UBXGNSS_OK;
}
