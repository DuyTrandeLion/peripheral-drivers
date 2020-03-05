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

#include "minmea/minmea.h"

struct minmea_sentence_rmc rmc_frame;
struct minmea_sentence_gga gga_frame;
struct minmea_sentence_gsa gsa_frame;
struct minmea_sentence_gll gll_frame;
struct minmea_sentence_gst gst_frame;
struct minmea_sentence_gsv gsv_frame;
struct minmea_sentence_vtg vtg_frame;
struct minmea_sentence_zda zda_frame;


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

        case UBXGNSS_SPI_INTERFACE:
        {
            break;
        }

        default: break;

    }

    return UBXGNSS_OK;
}


void UBXGNSS_DeInit(UBXGNSS_Def_t *locUBXGNSS_p)
{

}

uint16_t test_data;
int32_t lati, longi;
void UBXGNSS_ProcessData(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t locDataSource, uint8_t *locCommData_p, uint16_t locCommDataSize_u16)
{
    uint16_t len = strlen(locCommData_p);

    switch (locUBXGNSS_p->interface)
    {
        case UBXGNSS_UART_INTERFACE:
        {
            if (('$' == locCommData_p[0])         &&
                ('\r' == locCommData_p[len - 2])  &&
                ('\n' == locCommData_p[len - 1])
               )
            {
                /* Process string */
                switch (minmea_sentence_id(locCommData_p, false))
                {
                    case MINMEA_SENTENCE_RMC:
                    {
                        if (true == minmea_parse_rmc(&rmc_frame, locCommData_p))
                        {

                        }
                        break;
                    }
                    
                    case MINMEA_SENTENCE_GGA:
                    {
                        if (true == minmea_parse_gga(&gga_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    case MINMEA_SENTENCE_GSA:
                    {
                        if (true == minmea_parse_gsa(&gsa_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    case MINMEA_SENTENCE_GLL:
                    {
                        if (true == minmea_parse_gll(&gll_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    case MINMEA_SENTENCE_GST:
                    {
                        if (true == minmea_parse_gst(&gst_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    case MINMEA_SENTENCE_GSV:
                    {
                        if (true == minmea_parse_gsv(&gsv_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    case MINMEA_SENTENCE_VTG:
                    {
                        if (true == minmea_parse_vtg(&vtg_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    case MINMEA_SENTENCE_ZDA:
                    {
                        if (true == minmea_parse_zda(&zda_frame, locCommData_p))
                        {

                        }
                        break;
                    }

                    default: break;
                }
            }
            break;
        }

        case UBXGNSS_I2C_INTERFACE:
        {
            break;
        }

        case UBXGNSS_SPI_INTERFACE:
        {
            break;
        }

        default: 
            break;
    }
}
