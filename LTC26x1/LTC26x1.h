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
 * @file LTC26x1.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __LTC26X1_H__
#define __LTC26X1_H__


#include <stdint.h>
#include <stdbool.h>
#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif


/** Supported DAC devices. */
typedef enum
{
	LTC2601 = 0x00,
	LTC2611 = 0x01,
	LTC2621 = 0x02
} DAC_Chip_t;

/** DAC return codes. */
typedef enum
{
	DAC_OK 					= 0x00,
	DAC_DEVICE_BUSY			= 0x01,
	DAC_INVLD_DEVICE_TYPE 	= 0x02,
	DAC_NULL_PARAM 			= 0x03,
	DAC_COMM_ERROR			= 0x04,
	DAC_COMM_TIMEOUT		= 0x05,
	DAC_UNKNOWN_ERROR		= 0x06
} DAC_State_t;


/** Commands of execution for the DAC.
 *  The DAC will execute the received command in the next cycle. */
#define WRITE_TO_INPUT_REGISTER		0x00
#define UPDATE_PWR_UP_REGISTER		0x01
#define WRITE_TO_AND_UPDATE_PWR_UP	0x03
#define PWR_DOWN					0x04
#define NO_OPERATION				0xFF

/** SPI event types of communication. */
typedef enum
{
	SPI_DAC_EVENT_TRANSMIT 					= 0x00,
	SPI_DAC_EVENT_RECEIVE  					= 0x01,
	SPI_DAC_EVENT_TRANSMIT_RECEIVE 			= 0x02,
	SPI_DAC_EVENT_ABORT_TRANSMIT			= 0x03,
	SPI_DAC_EVENT_ABORT_RECEIVE				= 0x04,
	SPI_DAC_EVENT_ABORT_TRANSMIT_RECEIVE 	= 0x05,
	SPI_DAC_EVENT_CLEAR_INPUT				= 0x06,
	SPI_DAC_EVENT_BUSY_WAIT					= 0x07
} DAC_SPI_Event_t;


/**
 * SPI event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] DAC_SPI_Event_t 	SPI Event type.
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval DAC_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef DAC_State_t (*DAC_SPI_Handle_t)(DAC_SPI_Event_t, uint8_t *, uint16_t, void *);


/**
 * Busy time counting handle callback.
 *
 * @retval true 	If the device is busy
 */
typedef bool (*DAC_BusyWait_Handle_t)(void);


/** DAC chip configuration structure.  */
typedef struct
{
	DAC_Chip_t		deviceType;
	uint32_t		timeout;
	float			refVoltage;
} DAC_Config_t;

/** DAC definition structure.  */
typedef struct
{
	uint16_t				valueLSB;
	uint32_t				timeout;
	float					refVoltage;
	DAC_SPI_Handle_t 		const spiHandle;
	DAC_BusyWait_Handle_t	const busyHandle;
} DAC_Def_t;


/**@brief Function for initializing the LTC DAC driver.
 *
 * @param[in] 	locDAC_p			Pointer to driver definition.
 *
 * @retval 		DAC_OK 				If the driver was initialized successfully.
 *
 * @note SPI full-duplex master with 8-bit word, MSB first
 * 		 SPI clock below 50MHz
 *       SPI mode 0 (CPOL = 0, CPHA = 0)
 *
 * @note Visit https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
 * 		 and https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
 *       for more knowledge about SPI
 */
DAC_State_t DAC_Init(DAC_Def_t *locDAC_p);


/**@brief Function for de-initializing the LTC DAC driver.
 *
 * @param[in] 	locDAC_p			Pointer to driver definition.
 *
 */
void DAC_DeInit(DAC_Def_t *locDAC_p);


/**@brief Function for initializing the timer module.
 *
 * @param[in] 	locDAC_p			Pointer to driver definition.
 *
 * @retval DAC_OK 	If the conversions finished.
 */
void DAC_ClearInput(DAC_Def_t *locDAC_p);


/**@brief Function LTC DAC driver configuration.
 *
 * @param[in] 	locDAC_p				Pointer to driver definition.
 * @param[in] 	locDAC_Config_s			Structure that contains configuration information.
 *
 * @retval DAC_OK 		If the driver was configured successfully.
 */
DAC_State_t DAC_Config(DAC_Def_t *locDAC_p, DAC_Config_t locDAC_Config_s);


/**@brief Function for starting the digital to analog conversion process.
 *
 * @param[in] 	locDAC_p				Pointer to driver definition.
 * @param[in] 	locCommand_u8			Command to be sent to the DAC.
 * @param[in] 	locDataToConv_u16		Digital data to be converted to analog.
 *
 * @retval DAC_OK 		If the conversions finished.
 */
DAC_State_t DAC_StartDAConversion(DAC_Def_t *locDAC_p,
								  uint8_t  locCommand_u8,
								  uint16_t locDataToConv_u16);

#ifdef __cplusplus
}
#endif


#endif /* __LTC26X1_H__ */
