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
 * @file EEPROM.c
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#include "EEPROM.h"

#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_COMMUNICATION_BUFFER_SIZE		160
#define MAX_DATA_BUFFER_SIZE				256

/****************************************************************************
 * 							Private variables								*
 ****************************************************************************/

/****************************************************************************
 * 							Private functions								*
 ****************************************************************************/
static bool busyWait(EEPROM_Def_t *locEEPROM_p)
{
	uint32_t gTimeoutCounter_u32 = locEEPROM_p->config.timeout;

	if ((NULL == locEEPROM_p) ||
		(NULL == locEEPROM_p->commHandle))
	{
		return false;
	}

	while (gTimeoutCounter_u32 > 0)
	{
		gTimeoutCounter_u32--;
	}

	return false;
}

/****************************************************************************
 * 							Public functions								*
 ****************************************************************************/

EEPROM_State_t EEPROM_Init(EEPROM_Def_t *locEEPROM_p)
{
	if ((NULL == locEEPROM_p) ||
		(NULL == locEEPROM_p->commHandle))
	{
		return EEPROM_NULL_PARAM;
	}

	return EEPROM_OK;
}


void EEPROM_DeInit(EEPROM_Def_t *locEEPROM_p)
{

}


EEPROM_State_t EEPROM_ReadMemory(EEPROM_Def_t *locEEPROM_p,
		                         uint16_t locMemoryAddress_u16,
								 uint8_t *locReadData_p8,
								 uint16_t locReadSize_u16)
{
	EEPROM_State_t locRet = EEPROM_OK;
	uint8_t locTxBuff_au8[3];

	if (NULL == locEEPROM_p->commHandle)
	{
		return EEPROM_NULL_PARAM;
	}

	switch (locEEPROM_p->config.communication)
	{
		case SPI_COMM:
		{
			locTxBuff_au8[0] = READ_INST;									/* Read instruction            */
			locTxBuff_au8[1] = (locMemoryAddress_u16 >> 8) & 0xFF;			/* Read address high byte      */
			locTxBuff_au8[2] = locMemoryAddress_u16 & 0xFF;					/* Read address low byte       */

			locEEPROM_p->commHandle(SPI_EVENT_CS_LOW,
					 	 	 	 	locEEPROM_p->config.deviceAddress,
									0,
									NULL,
									0,
									NULL);
			locRet = locEEPROM_p->commHandle(SPI_EEP_EVENT_TRANSMIT,
											 locEEPROM_p->config.deviceAddress,
											 0,
											 locTxBuff_au8,
											 3,
											 NULL);
			if (EEPROM_OK == locRet)
			{
				locRet = locEEPROM_p->commHandle(SPI_EEP_EVENT_RECEIVE,
						                locEEPROM_p->config.deviceAddress,
										0,
										locReadData_p8,
										locReadSize_u16,
										NULL);
			}
			locEEPROM_p->commHandle(SPI_EVENT_CS_HIGH,
					 	 	 	 	locEEPROM_p->config.deviceAddress,
									0,
									NULL,
									0,
									NULL);
			return locRet;
		}

		case I2C_COMM:
		{
			return locEEPROM_p->commHandle(I2C_EEP_EVENT_RECEIVE,
					                       locEEPROM_p->config.deviceAddress,
									       locMemoryAddress_u16,
									       locReadData_p8,
									       locReadSize_u16,
									       NULL);
		}

		case MICROWIRE_COMM:
		{
			break;
		}

		default: break;
	}

	return locRet;
}


EEPROM_State_t EEPROM_WriteMemory(EEPROM_Def_t *locEEPROM_p,
		                          uint16_t locMemoryAddress_u16,
								  uint8_t *locWriteData_p8,
								  uint16_t locWriteSize_u16)
{
	EEPROM_State_t locRet = EEPROM_OK;
	uint8_t locWriteEn_u8;
	uint8_t locTxBuff_au8[MAX_COMMUNICATION_BUFFER_SIZE];
	uint8_t locDataBuff_au8[MAX_DATA_BUFFER_SIZE];

	if (NULL == locEEPROM_p->commHandle)
	{
		return EEPROM_NULL_PARAM;
	}

	switch (locEEPROM_p->config.communication)
	{
		case SPI_COMM:
		{
			locWriteEn_u8    = WREN_INST;									/* Write enable	instruction		*/
			locTxBuff_au8[0] = WRITE_INST;									/* Write instruction            */
			locTxBuff_au8[1] = (locMemoryAddress_u16 >> 8) & 0xFF;			/* Write address high byte      */
			locTxBuff_au8[2] = locMemoryAddress_u16 & 0xFF;					/* Write address low byte       */
			memcpy(&locTxBuff_au8[3], locWriteData_p8, locWriteSize_u16);	/* Write data */

			locEEPROM_p->commHandle(SPI_EVENT_CS_LOW,
					 	 	 	 	locEEPROM_p->config.deviceAddress,
									0,
									NULL,
									0,
									NULL);
			locRet = locEEPROM_p->commHandle(SPI_EEP_EVENT_TRANSMIT,
	                						locEEPROM_p->config.deviceAddress,
											0,
											&locWriteEn_u8,
											1,
											NULL);
			locEEPROM_p->commHandle(SPI_EVENT_CS_HIGH,
					 	 	 	 	locEEPROM_p->config.deviceAddress,
									0,
									NULL,
									0,
									NULL);
			if (EEPROM_OK == locRet)
			{
				locEEPROM_p->commHandle(SPI_EVENT_CS_LOW,
						 	 	 	 	locEEPROM_p->config.deviceAddress,
										0,
										NULL,
										0,
										NULL);
				locRet =  locEEPROM_p->commHandle(SPI_EEP_EVENT_TRANSMIT,
										locEEPROM_p->config.deviceAddress,
										0,
										locTxBuff_au8,
										(locWriteSize_u16 + 3),
										NULL);
				locEEPROM_p->commHandle(SPI_EVENT_CS_HIGH,
						 	 	 	 	locEEPROM_p->config.deviceAddress,
										0,
										NULL,
										0,
										NULL);
			}
			else
			{

				return EEPROM_COMM_ERROR;
			}
			break;
		}

		case I2C_COMM:
		{
			locRet = locEEPROM_p->commHandle(I2C_EEP_EVENT_TRANSMIT,
					                locEEPROM_p->config.deviceAddress,
									locMemoryAddress_u16,
									locWriteData_p8,
									locWriteSize_u16,
									NULL);
			break;
		}

		case MICROWIRE_COMM:
		{
			break;
		}

		default: break;
	}

	if (EEPROM_OK != locRet)
	{
		return EEPROM_COMM_ERROR;
	}
	else
	{
		if (NULL == locEEPROM_p->busyHandle)
		{
			busyWait(locEEPROM_p);
		}
		else
		{
			locEEPROM_p->busyHandle();
		}

		locRet = EEPROM_ReadMemory(locEEPROM_p,
								   locMemoryAddress_u16,
								   locDataBuff_au8,
								   locWriteSize_u16);
		if (EEPROM_OK != locRet)
		{
			return EEPROM_COMM_ERROR;
		}
	}

	if (0 == memcmp(locWriteData_p8, locDataBuff_au8, locWriteSize_u16))
	{
		return EEPROM_OK;
	}

	return EEPROM_WRITE_FAIL;
}


EEPROM_State_t EEPROM_WriteMemoryPage(EEPROM_Def_t *locEEPROM_p,
		                              uint16_t 	  locPageAddress_u16,
								      uint8_t     *locWriteData_p8)
{
	EEPROM_State_t locRet = EEPROM_OK;
	uint8_t locWriteEn_u8;
	uint8_t locTxBuff_au8[MAX_COMMUNICATION_BUFFER_SIZE];
	uint8_t locDataBuff_au8[MAX_DATA_BUFFER_SIZE];

	if (NULL == locEEPROM_p->commHandle)
	{
		return EEPROM_NULL_PARAM;
	}

	if (0 != (locPageAddress_u16 % locEEPROM_p->config.blockSize))
	{
		return EEPROM_INVLD_PAGE_ADDR;
	}

	switch (locEEPROM_p->config.communication)
	{
		case SPI_COMM:
		{
			locWriteEn_u8    = WREN_INST;									/* Write enable	instruction		*/
			locTxBuff_au8[0] = WRITE_INST;									/* Write instruction            */
			locTxBuff_au8[1] = (locPageAddress_u16 >> 8) & 0xFF;			/* Write address high byte      */
			locTxBuff_au8[2] = locPageAddress_u16 & 0xFF;					/* Write address low byte       */
			memcpy(&locTxBuff_au8[3], 										/* Write data */
					locWriteData_p8,
					locEEPROM_p->config.blockSize);

			locEEPROM_p->commHandle(SPI_EVENT_CS_LOW,
					 	 	 	 	locEEPROM_p->config.deviceAddress,
									0,
									NULL,
									0,
									NULL);
			locRet = locEEPROM_p->commHandle(SPI_EEP_EVENT_TRANSMIT,
	                						locEEPROM_p->config.deviceAddress,
											0,
											&locWriteEn_u8,
											1,
											NULL);
			locEEPROM_p->commHandle(SPI_EVENT_CS_HIGH,
					 	 	 	 	locEEPROM_p->config.deviceAddress,
									0,
									NULL,
									0,
									NULL);
			if (EEPROM_OK == locRet)
			{
				locEEPROM_p->commHandle(SPI_EVENT_CS_LOW,
						 	 	 	 	locEEPROM_p->config.deviceAddress,
										0,
										NULL,
										0,
										NULL);
				locRet =  locEEPROM_p->commHandle(SPI_EEP_EVENT_TRANSMIT,
										locEEPROM_p->config.deviceAddress,
										0,
										locTxBuff_au8,
										(locEEPROM_p->config.blockSize + 3),
										NULL);
				locEEPROM_p->commHandle(SPI_EVENT_CS_HIGH,
						 	 	 	 	locEEPROM_p->config.deviceAddress,
										0,
										NULL,
										0,
										NULL);
			}
			else
			{

				return EEPROM_COMM_ERROR;
			}
			break;
		}

		case I2C_COMM:
		{
			locRet =  locEEPROM_p->commHandle(I2C_EEP_EVENT_TRANSMIT,
					                		  locEEPROM_p->config.deviceAddress,
											  locPageAddress_u16,
											  locWriteData_p8,
											  locEEPROM_p->config.blockSize,
											  NULL);
		}

		case MICROWIRE_COMM:
		{
			break;
		}

		default: break;
	}

	if (EEPROM_OK != locRet)
	{
		return EEPROM_COMM_ERROR;
	}
	else
	{
		if (NULL == locEEPROM_p->busyHandle)
		{
			busyWait(locEEPROM_p);
		}
		else
		{
			locEEPROM_p->busyHandle();
		}

		locRet = EEPROM_ReadMemory(locEEPROM_p,
								   locPageAddress_u16,
								   locDataBuff_au8,
								   locEEPROM_p->config.blockSize);
		if (EEPROM_OK != locRet)
		{
			return EEPROM_COMM_ERROR;
		}
	}

	if (0 == memcmp(&locWriteData_p8[0], &locDataBuff_au8[0], locEEPROM_p->config.blockSize))
	{
		return EEPROM_OK;
	}

	return EEPROM_WRITE_FAIL;
}


EEPROM_State_t EEPROM_MemoryPut(EEPROM_Def_t *locEEPROM_p,
		                        uint16_t 	  locMemoryAddress_u16,
								StoreData_t  *locStoreData_sp)
{
	EEPROM_State_t locRet = EEPROM_OK;

	if (NULL == locEEPROM_p->commHandle)
	{
		return EEPROM_NULL_PARAM;
	}

	switch (locEEPROM_p->config.communication)
	{
		case SPI_COMM:
		{
			break;
		}

		case I2C_COMM:
		{
			break;
		}

		case MICROWIRE_COMM:
		{
			break;
		}

		default: break;
	}

	return locRet;
}


#ifdef __cplusplus
}
#endif

