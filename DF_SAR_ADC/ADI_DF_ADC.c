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
 * @file ADI_DF_ADC.c
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#include "ADI_DF_ADC.h"

#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 * 					Private variables                   *
 ****************************************************************************/

/****************************************************************************
 * 					Private functions                   *
 ****************************************************************************/
static bool busyWait(DFADC_Def_t *locDFADC_p)
{
    uint32_t gTimeoutCounter_u32 = locDFADC_p->timeout;

    if ((NULL == locDFADC_p) || (NULL == locDFADC_p->controlHandle))
    {
        return false;
    }

    /* Should change to NOP() */
    while (DFADC_DEVICE_BUSY == locDFADC_p->controlHandle(CONTROL_DF_EVENT_CHECK_BUSY, 0, NULL))
    {
        gTimeoutCounter_u32--;

        if (0 == gTimeoutCounter_u32)
        {
            return true;
        }
    }

    return false;
}

/****************************************************************************
 * 					Public functions                    *
 ****************************************************************************/
DFADC_State_t DFADC_Init(DFADC_Def_t *locDFADC_p)
{
    if ((NULL == locDFADC_p) || (NULL == locDFADC_p->spiHandle) || (NULL == locDFADC_p->controlHandle))
    {
        return DFADC_NULL_PARAM;
    }

    return DFADC_OK;
}


void DFADC_DeInit(DFADC_Def_t *locDFADC_p)
{

}


DFADC_State_t DFADC_Config(DFADC_Def_t *locDFADC_p, DFADC_Config_t locDFADC_Config_s)
{
    /*
     * The various modes of operation of the LTC2500-32 are programmed by 10 bits of a 12-bit control word, C[11:0].
     * The control word is shifted in at SDI on the rising edges of SCKA, MSB first.
     * */
    uint8_t locControlWord[2];

    if (NULL == locDFADC_p)
    {
        return DFADC_NULL_PARAM;
    }

    if (locDFADC_Config_s.deviceType > LTC2380_24)
    {
        return DFADC_INVLD_DEVICE_TYPE;
    }

    switch (locDFADC_Config_s.deviceType)
    {
        case LTC2500_32:
        {
            if (locDFADC_Config_s.gain > (DFADC_GAIN_EXPAND_ENABLE | DFADC_GAIN_COMPRESS_ENABLE))
            {
                return DFADC_INVLD_GAIN_CONF;
            }

            /*
             * LTC2508-32
             * Down-Sampling Factor Select Input 0, Down-Sampling Factor Select Input 1. Selects the down-sampling factor for the digital filter.
             * Down-sampling factors of 256, 1024, 4096 and 16384 are selected for [SEL0 SEL1] combinations of 00, 01, 10 and 11 respectively.
             * Logic levels are determined by OVDD.
             * */
            if ((locDFADC_Config_s.downSamplingFactor < DF_4) || (DF_16384 < locDFADC_Config_s.downSamplingFactor))
            {
                return DFADC_INVLD_DF_CONF;
            }

            if ((locDFADC_Config_s.filterType < SINC1_FILTER) || (AVERAGING_FILTER < locDFADC_Config_s.filterType))
            {
                return DFADC_INVLD_FLTR_CONF;
            }

            locDFADC_p->timeout = locDFADC_Config_s.timeout;

            locDFADC_p->numOfSamples = 1;
            for (uint16_t count = 0; count < locDFADC_Config_s.downSamplingFactor; count++)
            {
                locDFADC_p->numOfSamples *= 2;
            }

            /*
             * LTC2500-32
             * The LTC2500-32 offers a digital gain compression (DGC) feature which defines the full-scale input swing to be between 10% and 90%
             * of the ±VREF analog input range. To enable DGC, set DGC(C[9]) = 1 in the configuration word.
             *
             * The LTC2500-32 offers a digital gain expansion (DGE) feature, allowing the differential full-scale input swing to exceed the ±VREF
             * analog input range by 0.78% before the digital output code saturates. This is useful for system calibration where a full-scale
             * input voltage may need to be measured, causing the digital output code to saturate.
             * To enable DGE, set DGE(C[8]) = 1 in the configuration word.
             * */
            switch (locDFADC_Config_s.gain)
            {
                case DFADC_GAIN_DISABLE:
                {
                    locDFADC_p->offsetRefVoltage = locDFADC_Config_s.refVoltage;
                    locDFADC_p->valueLSB = 2147483647;
                    break;
                }

                case DFADC_GAIN_EXPAND_ENABLE:
                {
                    locDFADC_p->offsetRefVoltage = locDFADC_Config_s.refVoltage;
                    locDFADC_p->valueLSB = 1073741823;
                    break;
                }

                case DFADC_GAIN_COMPRESS_ENABLE:
                {
                    locDFADC_p->offsetRefVoltage = locDFADC_Config_s.refVoltage * 0.8;
                    locDFADC_p->valueLSB = 2147483647;
                    break;
                }

                case (DFADC_GAIN_EXPAND_ENABLE | DFADC_GAIN_COMPRESS_ENABLE):
                {
                    locDFADC_p->offsetRefVoltage = locDFADC_Config_s.refVoltage * 0.8;
                    locDFADC_p->valueLSB = 1073741823;
                    break;
                }

                default: break;
            }

            /*
             * LTC2500-32
             * C[11] and C[10] are used during the programming of the LTC2500-32
             * and do not control the configuration of the digital filter or ADC.
             * Bits C[3:0] select the filter type.
             * Bits C[7:4] select the down-sampling factor (DF).
             * C[8] enables/disables digital gain expansion (DGE)
             * and C[9] enables/disables digital gain compression (DGC).
             * */
            locControlWord[0] =  0x80;
            locControlWord[0] |= locDFADC_Config_s.gain << 4;
            locControlWord[0] |= locDFADC_Config_s.downSamplingFactor;
            locControlWord[1] =  locDFADC_Config_s.filterType;

            /*
             * LTC2500-32
             * A transaction window opens at power-up, at the falling edge of DRL, at the falling edge of a RDLA pulse, or when the filter configuration
             * is reset using a SYNC pulse. A transaction window opening allows the filter configuration of the LTC2500-32 to be programmed.
             * Once the transaction window opens, the state machine controlling the programming of the configuration is in a reset state, waiting for a
             * control word to be shifted in at SDI on the first 12 SCKA clock pulses.
             * The transaction window closes at the start of the next conversion when DRL transitions from low to high as shown in Figure 32, or at the
             * end of the 12th SCKA pulse since the transition window opened. Serial input data at SDI should be avoided when BUSY is high.
             * */
            if (NULL == locDFADC_p->busyHandle)
            {
                if (true == busyWait(locDFADC_p))
                {
                        return DFADC_DEVICE_BUSY;
                }
            }
            else
            {
                if (true == locDFADC_p->busyHandle())
                {
                        return DFADC_DEVICE_BUSY;
                }
            }

            /*
             * LTC2500-32
             * The input control word is used to determine whether or not the configuration
             * is programmed. In many cases, the user will simply need to configure
             * the converter once for their specific application after power-up and then drive
             * the SDI pin to GND. This will force the control word bits to all zeros and
             * the LTC2500-32 will operate with the programmed configuration.
             *
             * A valid input control word is one where C[11:10] = 10 and the remaining lower 0 bits,
             * C[9:0], have been shifted in before the transaction window closes as shown in
             * Figure 33a. When a valid control word is successfully entered on the 12th rising edge
             * of SCKA, the digital filter is reset if the configuration changes and is configured
             * to operate as programmed starting with the next conversion.
             * The configuration of the LTC2500-32 is only programmed by valid input control words and
             * discards control words that are partially written or have C[11:10] ≠ 10.
             * If C[11:10] ≠ 10, the LTC2500-32 closes the input transaction window until
             * the next transaction window as shown in Figure 33b. Figure 34 shows a
             * truncated programming transaction where a partial input control word is discarded
             * and a second complete valid input control word is successfully programmed.
             * */
            if (NULL != locDFADC_p->spiHandle)
            {
                return locDFADC_p->spiHandle(SPI_DF_EVENT_TRANSMIT, locControlWord, 2, NULL);
            }
            else
            {
                return DFADC_NULL_PARAM;
            }
            break;
        }

        case LTC2508_32:
        {
            /*
             * LTC2508-32
             * SEL0, SEL1 (Pins 11, 12): Down-Sampling Factor Select Input 0, Down-Sampling Factor Select Input 1.
             * Selects the down-sampling factor for the digital filter. Down-sampling factors of 256, 1024, 4096 and 16384
             * are selected for [SEL0 SEL1] combinations of 00, 01, 10 and 11 respectively.
             * Logic levels are determined by OVDD.
             * */
            if ((locDFADC_Config_s.downSamplingFactor != DF_256) &&
                (locDFADC_Config_s.downSamplingFactor != DF_1024) &&
                (locDFADC_Config_s.downSamplingFactor != DF_4096) &&
                (locDFADC_Config_s.downSamplingFactor != DF_16384))
            {
                return DFADC_INVLD_DF_CONF;
            }

            locDFADC_p->timeout = locDFADC_Config_s.timeout;

            locDFADC_p->numOfSamples = 1;

            for (uint16_t count = 0; count < locDFADC_Config_s.downSamplingFactor; count++)
            {
                locDFADC_p->numOfSamples *= 2;
            }
            break;
        }

        case LTC2512_24:
        {
            /*
             * LTC2512-24
             * Down-Sampling Factor Select Input 0, Down-Sampling Factor Select Input 1.
             * Selects the down-sampling factor for the digital filter. Down-sampling factors of 4, 8, 16 and 32 are
             * selected for [SEL1 SEL0] combinations of 00, 01, 10 and 11 respectively.
             * Logic levels are determined by OVDD.
             * */
            if ((locDFADC_Config_s.downSamplingFactor != DF_4) &&
                (locDFADC_Config_s.downSamplingFactor != DF_8) &&
                (locDFADC_Config_s.downSamplingFactor != DF_16) &&
                (locDFADC_Config_s.downSamplingFactor != DF_32))
            {
                return DFADC_INVLD_DF_CONF;
            }

            locDFADC_p->timeout = locDFADC_Config_s.timeout;

            locDFADC_p->numOfSamples = 1;

            for (uint16_t count = 0; count < locDFADC_Config_s.downSamplingFactor; count++)
            {
                locDFADC_p->numOfSamples *= 2;
            }
            break;
        }

        case LTC2380_24:
        {
            /*
             * LTC2380-24
             * */
        }

        default: break;
    }

    return DFADC_OK;
}


DFADC_State_t DFADC_Control(DFADC_Def_t *locDFADC_p, DFADC_Control_Event_t locControlEvent_en, uint32_t locControlData_u32, void *locContext_p)
{
    if (NULL != locDFADC_p->controlHandle)
    {
        return locDFADC_p->controlHandle(locControlEvent_en, locControlData_u32, locContext_p);
    }
    else
    {
        return DFADC_NULL_PARAM;
    }
}


DFADC_State_t DFADC_StartADConversion(DFADC_Def_t *locDFADC_p)
{
    /*
     * LTC2500-32
     * The filtered output register contains filtered output codes DOUT(k)
     * provided by the decimation filter. DOUT(k) is updated once in every
     * DF number of conversion cycles. A timing signal DRL indicates when
     * DOUT(k) is updated. DRL goes high at the beginning of every DFth conversion,
     * and it goes low when the conversion completes. The 32-bits of DOUT(k)
     * can be read out before the beginning of the next A/D conversion.
     * */
    for (uint16_t count = 0; count < locDFADC_p->numOfSamples; count++)
    {
        if (NULL != locDFADC_p->controlHandle)
        {
            locDFADC_p->controlHandle(CONTROL_DF_EVENT_SET_CLOCK, 1, NULL);
            locDFADC_p->controlHandle(CONTROL_DF_EVENT_WAIT, 1, NULL);
            locDFADC_p->controlHandle(CONTROL_DF_EVENT_SET_CLOCK, 0, NULL);
            locDFADC_p->controlHandle(CONTROL_DF_EVENT_WAIT, 1, NULL);
        }
        else
        {
            return DFADC_NULL_PARAM;
        }

            if (NULL == locDFADC_p->busyHandle)
            {
                if (true == busyWait(locDFADC_p))
                {
                    return DFADC_DEVICE_BUSY;
                }
            }
            else
            {
                if (true == locDFADC_p->busyHandle())
                {
                    return DFADC_DEVICE_BUSY;
                }
            }
    }

    return DFADC_OK;
}


DFADC_State_t DFADC_ReadDFDistributedData(DFADC_Def_t *locDFADC_p, uint8_t *locReadData_au8)
{
    /*
     * LTC2500-32
     * Read Dout for each of DF consecutive A/D conversions, enabling use of a much
     * slower serial clock (SCKA). Transitions on the digital interface should be
     * avoided during A/D conversion operations (when BUSY is high).
     * */
    if ((NULL == locDFADC_p->controlHandle) || (NULL == locDFADC_p->spiHandle))
    {
        return DFADC_NULL_PARAM;
    }

    if (DFADC_DATA_READY == locDFADC_p->controlHandle(CONTROL_DF_EVENT_CHECK_DATA_READY, 0, NULL))
    {
        return locDFADC_p->spiHandle(SPI_DF_EVENT_RECEIVE, locReadData_au8, 4, NULL);
    }

    return DFADC_DATA_NOT_READY;
}


DFADC_State_t DFADC_ReadDFSyncData(DFADC_Def_t *locDFADC_p, uint16_t locNumDevice, uint8_t *locReadData)
{
    return DFADC_DATA_NOT_READY;
}


void DFADC_ConvToVoltage(DFADC_Def_t *locDFADC_p, uint8_t *locRawData_p, float *locVoltage_p)
{
    int32_t locRawADC_i32 = 0;
    float temp_f = 0;

    CONV_TO_INT32_RAW_DATA(locRawData_p, locRawADC_i32);

    temp_f = (float)((float)locRawADC_i32 / locDFADC_p->valueLSB);

    if (NULL != locVoltage_p)
    {
        *locVoltage_p = temp_f * locDFADC_p->offsetRefVoltage;
    }
}


#ifdef __cplusplus
}
#endif

