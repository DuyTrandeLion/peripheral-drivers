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
 * @file TI_ADS_ADC.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __TI_ADS_ADC_H__
#define __TI_ADS_ADC_H__

#include <stdint.h>
#include <stdbool.h>
#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum waiting time in tick. */
#define MAXIMUM_TIMEOUT				100

#define SPI_3WIRE					0x00
#define SPI_4WIRE					0x01
#define SPI_DAISY_CHAIN				0x02


/** Supported SAR ADC with Digital Filter devices. */
typedef enum
{
	ADS8887 = 0x00,
	ADS8885 = 0x02,
	ADS8883 = 0x03,
	ADS8881 = 0x04,
	ADS8866 = 0x05,
	ADS8339 = 0x06,
	ADS8864 = 0x07,
	ADS8319 = 0x08,
	ADS8862 = 0x09,
	ADS8860 = 0x0A,
	ADS8867 = 0x0B,
	ADS8865 = 0x0C,
	ADS8318 = 0x0D,
	ADS8863 = 0x0E,
	ADS8861 = 0x0F,
	ADS9110 = 0x10
} ADSADC_Chip_t;

/** SAR ADC return codes. */
typedef enum
{
	ADSADC_OK 					= 0x00,
	ADSADC_DEVICE_NOT_BUSY 		= 0x00,
	ADSADC_DEVICE_BUSY			= 0x01,
	ADSADC_INVLD_DEVICE_TYPE 	= 0x02,
	ADSADC_NULL_PARAM 			= 0x03,
	ADSADC_COMM_ERROR			= 0x04,
	ADSADC_COMM_TIMEOUT			= 0x05,
	ADSADC_UNKNOWN_ERROR		= 0x06
} ADSADC_State_t;

/** SPI event types of communication. */
typedef enum
{
	SPI_ADS_EVENT_TRANSMIT 					= 0x00,
	SPI_ADS_EVENT_RECEIVE  					= 0x01,
	SPI_ADS_EVENT_TRANSMIT_RECEIVE 			= 0x02,
	SPI_ADS_EVENT_ABORT_TRANSMIT			= 0x03,
	SPI_ADS_EVENT_ABORT_RECEIVE				= 0x04,
	SPI_ADS_EVENT_ABORT_TRANSMIT_RECEIVE 	= 0x05
} ADSADC_SPI_Event_t;

/** Control event types. */
typedef enum
{
	CONTROL_ADS_EVENT_3WIRE_CONVST			= 0x00,
	CONTROL_ADS_EVENT_4WIRE_CONVST			= 0x01,
	CONTROL_ADS_EVENT_4WIRE_DIN				= 0x02
} ADSADC_Control_Event_t;


/**
 * SPI event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] ADSADC_SPI_Event_t 	SPI Event type.
 * @param[in] uint8_t *				Pointer to data buffer.
 * @param[in] uint16_t				Size of data buffer.
 * @param[in] void *				Pointer to parameter of event handler
 *
 * @retval ADSADC_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef ADSADC_State_t (*ADSADC_SPI_Handle_t)(ADSADC_SPI_Event_t, uint8_t *, uint16_t, void *);


/**
 * Control event callback type.
 *
 * This function does triggering, waiting actions to/from ADC.
 *
 * @param[in] ADSADC_Control_Handle_t 	Control Event type.
 * @param[in] uint32_t					Control value.
 * @param[in] void *					Pointer to parameter of event handler
 *
 * @retval ADSADC_State_t
 */
typedef ADSADC_State_t (*ADSADC_Control_Handle_t)(ADSADC_Control_Event_t, uint32_t, void *);


/**
 * Busy time counting handle callback.
 *
 * @param[in] uint32_t	Delay time
 * @retval true 		If the device is busy
 */
typedef bool (*ADSADC_Delay_Handle_t)(uint32_t);


/** ADC chip configuration structure.  */
typedef struct
{
	ADSADC_Chip_t	deviceType;
	uint32_t		convTime;
	uint32_t		timeout;
	float			refVoltage;
} ADSADC_Config_t;

/** ADC definition structure.  */
typedef struct
{
	uint8_t 					communication;
	uint32_t					convTime;
	uint32_t					valueLSB;
	uint32_t					timeout;
	float						refVoltage;
	ADSADC_SPI_Handle_t 		const spiHandle;
	ADSADC_Control_Handle_t 	const controlHandle;
	ADSADC_Delay_Handle_t 		const delayHandle;
} ADSADC_Def_t;


/** Converts a macro argument into a character constant.
 */
void CONV_18BITS_TO_INT32_RAW_DATA(uint8_t *array, int32_t *dataADC);


/**@brief Function for initializing the LTC ADC driver.
 *
 * @param[in] 	locADSADC_p			Pointer to driver definition.
 * @param[out] 	locDeviceHandle_p	Instance ID/handle.
 *
 * @retval ADSADC_OK 	If the driver was initialized successfully.
 *
 * @note SPI full-duplex master with 8-bit word, MSB first
 * 		 SPI clock below or equal to 70MHz
 *       SPI mode 0 (CPOL = 0, CPHA = 0)
 *
 * @note Visit https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
 *       and https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
 */
ADSADC_State_t ADSADC_Init(ADSADC_Def_t *locADSADC_p);


/**@brief Function for de-initializing the LTC ADC driver.
 *
 * @param[in] 	locDeviceHandle_hdl 	Handle (ID) of the driver instance.
 *
 */
void ADSADC_DeInit(ADSADC_Def_t *locADSADC_p);


/**@brief Function LTC ADC driver configuration.
 *
 * @param[in] 	locDeviceHandle_hdl 	Handle (ID) of the driver instance.
 * @param[in] 	locADSADC_Config_s		Structure that contains configuration information.
 *
 * @retval ADSADC_OK 	If the driver was configured successfully.
 */
ADSADC_State_t ADSADC_Config(ADSADC_Def_t *locADSADC_p, ADSADC_Config_t locADSADC_Config_s);


/**@brief Function triggering control signal of the LTC ADC chip.
 *
 * @param[in] locDeviceHandle_hdl 		Handle (ID) of the driver instance.
 * @param[in] ADSADC_Control_Handle_t 	Control Event type.
 * @param[in] uint8_t					Control value.
 * @param[in] void *					Pointer to parameter of event handler
 */
ADSADC_State_t ADSADC_Control(ADSADC_Def_t *locADSADC_p, ADSADC_Control_Event_t locControlEvent_en, uint32_t locControlData_u32, void *locContext_p);


/**@brief Function for starting the sample conversion.
 *
 * @param[in] 	locADSADC_p				Pointer to driver definition.
 *
 * @retval 		DFADC_OK 				If the conversions finished.
 */
ADSADC_State_t ADSADC_StartADConversion(ADSADC_Def_t *locADSADC_p);


/**@brief Function for reading the distributed digital output of the LTC ADC chip.
 *
 * @param[in] 	ADSADC_Control_Handle_t 	Handle (ID) of the driver instance.
 * @param[out] 	locReadData				Digital output of the LTC ADC.
 * 										The number of bytes depends on the device type.
 *
 * @retval ADSADC_OK 	If the result is available.
 *
 * @attention Function ADSADC_StartADConversion should be called first to start the conversion.
 */
ADSADC_State_t ADSADC_ReadDistributedData(ADSADC_Def_t *locADSADC_p, uint8_t *locReadData_au8);


/**@brief Function for reading the synchronized digital output of the LTC ADC chips.
 *
 * @param[in] 	locDeviceHandle_p 	Pointer to the array of driver instance handles.
 * @param[in] 	locNumDevice		Number of devices to get data.
 * @param[out]	locReadData			Pointer to the digital output buffer.
 *
 * @retval ADSADC_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 * @attention Function ADSADC_StartADConversion should be called first to start the conversion.
 */
ADSADC_State_t ADSADC_ReadChainData(ADSADC_Def_t *locADSADC_p, uint16_t locNumDevice, uint8_t *locReadData);


/**@brief Function for converting the digital ADC data to voltage.
 *
 * @param[in] 	locDeviceHandle_hdl 	Handle (ID) of the driver instance.
 * @param[in] 	locRawData_p			ADC digital buffer.
 * @param[out] 	locVoltage_p			Pointer to result voltage.
 *
 */
void ADSADC_ConvToVoltage(ADSADC_Def_t *locADSADC_p, uint8_t *locRawData_p, float *locVoltage_p);


#ifdef __cplusplus
}
#endif

#endif /* __TI_ADS_ADC_H__ */
