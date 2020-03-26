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
 * @file TI_ADS_ADC.C
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#include "TI_ADS_ADC.h"

#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 * 				Private variables                           *
 ****************************************************************************/


/****************************************************************************
 * 				Private functions                           *
 ****************************************************************************/

/****************************************************************************
 * 				Public functions                            *
 ****************************************************************************/
ADSADC_State_t ADSADC_Init(ADSADC_Def_t *locADSADC_p)
{
    if ((NULL == locADSADC_p) || (NULL == locADSADC_p->spiHandle) || (NULL == locADSADC_p->controlHandle))
    {
        return ADSADC_NULL_PARAM;
    }

    return ADSADC_OK;
}


void ADSADC_DeInit(ADSADC_Def_t *locADSADC_p)
{

}


ADSADC_State_t ADSADC_Config(ADSADC_Def_t *locADSADC_p, ADSADC_Config_t locADSADC_Config_s)
{
    if (NULL == locADSADC_p)
    {
        return ADSADC_NULL_PARAM;
    }

    if (locADSADC_Config_s.deviceType > ADS9110)
    {
        return ADSADC_INVLD_DEVICE_TYPE;
    }

    locADSADC_p->timeout    = locADSADC_Config_s.timeout;
    locADSADC_p->refVoltage = locADSADC_Config_s.refVoltage;
    locADSADC_p->convTime   = locADSADC_Config_s.convTime;

    if ((locADSADC_Config_s.deviceType < ADS8866) || (ADS9110 == locADSADC_Config_s.deviceType))
    {
        locADSADC_p->valueLSB = 131071;
    }
    else
    {
        locADSADC_p->valueLSB = 32767;
    }

    return ADSADC_OK;
}


ADSADC_State_t ADSADC_Control(ADSADC_Def_t *locADSADC_p, ADSADC_Control_Event_t locControlEvent_en, uint32_t locControlData_u32, void *locContext_p)
{
    if (NULL != locADSADC_p->controlHandle)
    {
        return locADSADC_p->controlHandle(locControlEvent_en, locControlData_u32, locContext_p);
    }
    else
    {
        return ADSADC_NULL_PARAM;
    }
}


ADSADC_State_t ADSADC_StartADConversion(ADSADC_Def_t *locADSADC_p)
{
    uint32_t locConvTimeout_u32;

    if ((NULL == locADSADC_p) || (NULL == locADSADC_p->spiHandle) || (NULL == locADSADC_p->controlHandle))
    {
        return ADSADC_NULL_PARAM;
    }

    locConvTimeout_u32 = locADSADC_p->convTime;

    switch (locADSADC_p->communication)
    {
        case SPI_3WIRE:
        {
            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_3WIRE_CONVST, 1, NULL);

            if (NULL == locADSADC_p->delayHandle)
            {
                while (locConvTimeout_u32 > 0)
                {
                    locConvTimeout_u32--;
                }
            }
            else
            {
                locADSADC_p->delayHandle(locConvTimeout_u32);
            }

            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_3WIRE_CONVST, 0, NULL);
            break;
        }

        case SPI_4WIRE:
        {
            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_4WIRE_CONVST, 1, NULL);
            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_4WIRE_DIN, 1, NULL);

            if (NULL == locADSADC_p->delayHandle)
            {
                while (locConvTimeout_u32 > 0)
                {
                    locConvTimeout_u32--;
                }
            }
            else
            {
                locADSADC_p->delayHandle(locConvTimeout_u32);
            }

            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_4WIRE_DIN, 0, NULL);
            break;
        }

        case SPI_DAISY_CHAIN:
        {
            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_4WIRE_CONVST, 1, NULL);
            break;
        }

        default: break;
    }

    return ADSADC_OK;
}


ADSADC_State_t ADSADC_ReadDistributedData(ADSADC_Def_t *locADSADC_p, uint8_t *locReadData_au8)
{
    ADSADC_State_t locRet = ADSADC_DEVICE_BUSY;

    if ((NULL == locADSADC_p) || (NULL == locADSADC_p->spiHandle) || (NULL == locADSADC_p->controlHandle))
    {
        return ADSADC_NULL_PARAM;
    }

    switch (locADSADC_p->communication)
    {
        case SPI_3WIRE:
        {
            locRet = locADSADC_p->spiHandle(SPI_ADS_EVENT_RECEIVE, locReadData_au8, 2, NULL);
            break;
        }

        case SPI_4WIRE:
        {
            locRet = locADSADC_p->spiHandle(SPI_ADS_EVENT_RECEIVE, locReadData_au8, 2, NULL);
            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_3WIRE_CONVST, 0, NULL);
            break;
        }

        case SPI_DAISY_CHAIN:
        {
            locADSADC_p->controlHandle(CONTROL_ADS_EVENT_3WIRE_CONVST, 0, NULL);
            break;
        }

        default: break;
    }

    return locRet;
}


ADSADC_State_t ADSADC_ReadChainData(ADSADC_Def_t *locADSADC_p, uint16_t locNumDevice, uint8_t *locReadData)
{
    return ADSADC_DEVICE_BUSY;
}


void ADSADC_ConvToVoltage(ADSADC_Def_t *locADSADC_p, uint8_t *locRawData_p, float *locVoltage_p)
{
    int32_t locRawADC_i32 = 0;
    float temp_f = 0;

    CONV_18BITS_TO_INT32_RAW_DATA(locRawData_p, &locRawADC_i32);

    temp_f = (float)((float)locRawADC_i32 / locADSADC_p->valueLSB);

    if (NULL != locVoltage_p)
    {
        *locVoltage_p = temp_f * locADSADC_p->refVoltage;
    }
}


void CONV_18BITS_TO_INT32_RAW_DATA(uint8_t *array, int32_t *dataADC)
{
    *dataADC = ((uint32_t)array[0] << 1) | ((uint32_t)array[1]);
    *dataADC = *dataADC << 9;
    *dataADC = *dataADC | ((uint32_t)array[2] << 1);
    *dataADC = *dataADC | ((uint32_t)array[3]);

    if ((0x01FFFF < *dataADC) && (*dataADC <= 0x03FFFF))
    {
        *dataADC = (-1) * (262144 - *dataADC);
    }
}

#ifdef __cplusplus
}
#endif
