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
 * @file LTC26x1.c
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#include "LTC26x1.h"

#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * 							Private variables								*
 ****************************************************************************/

/****************************************************************************
 * 							Private functions								*
 ****************************************************************************/
static bool busyWait(DAC_Def_t *locDAC_p)
{
	uint32_t gTimeoutCounter_u32 = locDAC_p->timeout;

	if ((NULL == locDAC_p) ||
		(NULL == locDAC_p->spiHandle))
	{
		return false;
	}

	while (DAC_DEVICE_BUSY == locDAC_p->spiHandle(SPI_DAC_EVENT_BUSY_WAIT, 0, 0, NULL))
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
 * 							Public functions								*
 ****************************************************************************/
DAC_State_t DAC_Init(DAC_Def_t *locDAC_p)
{
	if ((NULL == locDAC_p) 				||
	    (NULL == locDAC_p->spiHandle)
	   )
	{
		return DAC_NULL_PARAM;
	}

	return DAC_OK;
}


void DAC_DeInit(DAC_Def_t *locDAC_p)
{

}


DAC_State_t DAC_Config(DAC_Def_t *locDAC_p, DAC_Config_t locDAC_Config_s)
{
	if (locDAC_Config_s.deviceType > LTC2621)
	{
		return DAC_INVLD_DEVICE_TYPE;
	}

	locDAC_p->timeout 	 = locDAC_Config_s.timeout;
	locDAC_p->refVoltage = locDAC_Config_s.refVoltage;
	locDAC_p->deviceType = locDAC_Config_s.deviceType;

	switch (locDAC_p->deviceType)
	{
		case LTC2601:
		{
			locDAC_p->valueLSB = 65535;
			break;
		}

		case LTC2611:
		{
			locDAC_p->valueLSB = 16383;
			break;
		}

		case LTC2621:
		{
			locDAC_p->valueLSB = 4095;
			break;
		}

		default: break;
	}

	DAC_ClearInput(locDAC_p);
	return DAC_OK;
}


void DAC_ClearInput(DAC_Def_t *locDAC_p)
{
	if (NULL != locDAC_p->spiHandle)
	{
		locDAC_p->spiHandle(SPI_DAC_EVENT_CLEAR_INPUT, 0, 0, NULL);
	}
}


DAC_State_t DAC_StartDAConversion(DAC_Def_t *locDAC_p,
								  uint8_t  locCommand_u8,
								  uint16_t locDataToConv_u16)
{
	uint8_t locTxBuffer_au8[3] = {0};

	/*
	 * The CS/LD input is level triggered. When this input is taken low, it acts as a chip-select signal,
	 * powering-on the SDI and SCK buffers and enabling the input shift register.
	 * Data (SDI input) is transferred at the next 24 rising SCK edges. The 4-bit command, C3-C0,
	 * is loaded first; then 4 don’t care bits; and finally the 16-bit data word. The data word comprises
	 * the 16-, 14- or 12-bit input code, ordered MSB-to-LSB, followed by 0, 2 or 4 don’t care bits
	 * (LTC2601, LTC2611 and LTC2621 respectively). Data can only be transferred to the device when the CS/LD signal
	 * is low.The rising edge of CS/LD ends the data transfer and causes the device to execute the command
	 * specified in the 24-bit input word. The complete sequence is shown in Figure 2a.
	 *
	 * The command (C3-C0) assignments are shown in Table 1. The first four commands in the table consist of write
	 * and update operations. A write operation loads a 16-bit data word from the 32-bit shift register into
	 * the input register of the DAC. In an update operation, the data word is copied from the input register
	 * to the DAC register and converted to an analog voltage at the DAC output. The update operation also powers up
	 * the DAC if it had been in power-down mode. The data path and registers are shown in the Block Diagram.
	 *
	 * While the minimum input word is 24 bits, it may optionally be extended to 32 bits. To use the 32-bit word width,
	 * 8 don’t-care bits are transferred to the device first, followed by the 24-bit word as just described.
	 * Figure 2b shows the 32-bit sequence. The 32-bit word is required for daisychain operation, and is also available
	 * to accommodate microprocessors which have a minimum word width of 16 bits (2 bytes).
	 * */
	locTxBuffer_au8[0] = locCommand_u8 << 4;
	switch (locDAC_p->deviceType)
	{
		case LTC2601:
		{
			/* Use all 16 bits of uint16_t */
			locTxBuffer_au8[1] = locDataToConv_u16 >> 8;
			locTxBuffer_au8[2] = locDataToConv_u16;
			break;
		}

		case LTC2611:
		{
			/* Use only 14 bits LSB of uint16_t. Shift left 2 bits to avoid data missing.
			 * The LTC2611 ignores 2 LSB. */
			uint16_t temp = locDataToConv_u16 << 2;
			locTxBuffer_au8[1] = temp >> 8;
			locTxBuffer_au8[2] = temp;
			break;
		}

		case LTC2621:
		{
			/* Use only 12 bits LSB of uint16_t. Shift left 4 bits to avoid data missing.
			 * The LTC2621 ignores 4 LSB. */
			uint16_t temp = locDataToConv_u16 << 4;
			locTxBuffer_au8[1] = temp >> 8;
			locTxBuffer_au8[2] = temp;
			break;
		}
		default: break;
	}

	if (NULL != locDAC_p->spiHandle)
	{
		return locDAC_p->spiHandle(SPI_DAC_EVENT_TRANSMIT,
				                   locTxBuffer_au8,
								   SIZE(locTxBuffer_au8),
								   NULL);
	}
	else
	{
		return DAC_NULL_PARAM;
	}
}


#ifdef __cplusplus
}
#endif

