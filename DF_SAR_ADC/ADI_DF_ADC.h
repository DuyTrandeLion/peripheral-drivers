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
 * @file ADI_DF_ADC.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __ADI_DF_ADC_H__
#define __ADI_DF_ADC_H__

#include <stdint.h>
#include <stdbool.h>
#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum waiting time in tick. */
#define MAXIMUM_TIMEOUT			100

/** SAR ADC digital gains. */
#define DFADC_GAIN_DISABLE		0x00
/* Available for only LTC2500_32. */
#define DFADC_GAIN_EXPAND_ENABLE	0x01
/* Available for LTC2500_32/LTC2512_24/LTC2380_24. */
#define DFADC_GAIN_COMPRESS_ENABLE	0x02

/** Supported SAR ADC with Digital Filter devices. */
typedef enum
{
    LTC2500_32 = 0x00,
    LTC2508_32 = 0x01,
    LTC2512_24 = 0x02,
    LTC2380_24 = 0x03
} DFADC_Chip_t;

/** SAR ADC return codes. */
typedef enum
{
    DFADC_OK 			= 0x00,
    DFADC_DATA_READY 		= 0x00,
    DFADC_DEVICE_NOT_BUSY 	= 0x00,
    DFADC_DATA_NOT_READY 	= 0x01,
    DFADC_DEVICE_BUSY		= 0x02,
    DFADC_INVLD_GAIN_CONF 	= 0x03,
    DFADC_INVLD_DF_CONF		= 0x04,
    DFADC_INVLD_FLTR_CONF 	= 0x05,
    DFADC_INVLD_DEVICE_TYPE     = 0x06,
    DFADC_NULL_PARAM 		= 0x07,
    DFADC_COMM_ERROR		= 0x08,
    DFADC_COMM_TIMEOUT		= 0x09,
    DFADC_UNKNOWN_ERROR		= 0x0A
} DFADC_State_t;

/** SPI event types of communication. */
typedef enum
{
    SPI_DF_EVENT_TRANSMIT 		= 0x00,
    SPI_DF_EVENT_RECEIVE  		= 0x01,
    SPI_DF_EVENT_TRANSMIT_RECEIVE 	= 0x02,
    SPI_DF_EVENT_ABORT_TRANSMIT		= 0x03,
    SPI_DF_EVENT_ABORT_RECEIVE		= 0x04,
    SPI_DF_EVENT_ABORT_TRANSMIT_RECEIVE = 0x05
} DFADC_SPI_Event_t;

/** Control event types. */
typedef enum
{
    CONTROL_DF_EVENT_CHECK_DATA_READY	= 0x00,
    CONTROL_DF_EVENT_CHECK_BUSY		= 0x01,
    CONTROL_DF_EVENT_SET_CLOCK		= 0x02,
    CONTROL_DF_EVENT_PRESET_MODE	= 0x03,
    CONTROL_DF_EVENT_WAIT		= 0x04,
    CONTROL_DF_EVENT_TIMEOUT_COUNT	= 0x05,
    CONTROL_DF_EVENT_SYNC		= 0x06,
    CONTROL_DF_EVENT_DOWN_SAMPL_FACTOR	= 0x07
} DFADC_Control_Event_t;


/**
 * SPI event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] DFADC_SPI_Event_t         SPI Event type.
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval DFADC_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef DFADC_State_t (*DFADC_SPI_Handle_t)(DFADC_SPI_Event_t locCommEvent_en, uint8_t *locData_p8, uint16_t locDataSize_u16, void *locContext_p);


/**
 * Control event callback type.
 *
 * This function does triggering, waiting actions to/from ADC.
 *
 * @param[in] DFADC_Control_Handle_t 	Control Event type.
 * @param[in] uint32_t			Control value.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval DFADC_State_t
 */
typedef DFADC_State_t (*DFADC_Control_Handle_t)(DFADC_Control_Event_t, uint32_t, void *);


/**
 * Busy time counting handle callback.
 *
 * @retval true 	If the device is busy
 */
typedef bool (*DFADC_BusyWait_Handle_t)(void);


/** Supported types of Digital Filter. */
typedef enum
{
    SINC1_FILTER		= 0x01,
    SINC2_FILTER		= 0x02,
    SINC3_FILTER		= 0x03,
    SINC4_FILTER		= 0x04,
    SSINC_FILTER		= 0x05,
    FLAT_PASSBAND_FILTER	= 0x06,
    AVERAGING_FILTER		= 0x07
} DFADC_Filter_t;

/** Supported Down Sampling rates. */
typedef enum
{
    DF_4	= 0x02,
    DF_8	= 0x03,
    DF_16	= 0x04,
    DF_32	= 0x05,
    DF_64	= 0x06,
    DF_128	= 0x07,
    DF_256	= 0x08,
    DF_512	= 0x09,
    DF_1024	= 0x0A,
    DF_2048	= 0x0B,
    DF_4096	= 0x0C,
    DF_8192	= 0x0D,
    DF_16384	= 0x0E
} DFADC_DF_t;

/** ADC chip configuration structure.  */
typedef struct
{
    DFADC_Chip_t	deviceType;
    DFADC_DF_t		downSamplingFactor;
    DFADC_Filter_t 	filterType;
    uint8_t 		gain;
    uint32_t		timeout;
    float		refVoltage;
} DFADC_Config_t;

/** ADC definition structure.  */
typedef struct
{
    uint16_t                numOfSamples;
    uint32_t                valueLSB;
    uint32_t                timeout;
    float                   offsetRefVoltage;
    DFADC_SPI_Handle_t      spiHandle;
    DFADC_Control_Handle_t  controlHandle;
    DFADC_BusyWait_Handle_t busyHandle;
} DFADC_Def_t;


/** Converts a macro argument into a character constant.
 */
#define CONV_TO_INT32_RAW_DATA	MERGE_4BYTES


/**@brief Function for initializing the LTC ADC driver.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 *
 * @retval 	DFADC_OK 		If the driver was initialized successfully.
 *
 * @note SPI full-duplex master with 8-bit word, MSB first
 * 		 SPI clock below or equal to 100MHz
 *       SPI mode 0 (CPOL = 0, CPHA = 0)
 *
 * @note Visit https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
 *       and https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
 *       for more knowledge about SPI
 */
DFADC_State_t DFADC_Init(DFADC_Def_t *locDFADC_p);


/**@brief Function for de-initializing the LTC ADC driver.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 *
 */
void DFADC_DeInit(DFADC_Def_t *locDFADC_p);


/**@brief Function LTC ADC driver configuration.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 * @param[in] 	locDFADC_Config_s	Structure that contains configuration information.
 *
 * @retval DFADC_OK 	If the driver was configured successfully.
 */
DFADC_State_t DFADC_Config(DFADC_Def_t *locDFADC_p, DFADC_Config_t locDFADC_Config_s);


/**@brief Function triggering control signal of the LTC ADC chip.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 * @param[in] 	DFADC_Control_Handle_t 	Control Event type.
 * @param[in] 	uint8_t			Control value.
 * @param[in] 	void *			Pointer to parameter of event handler
 */
DFADC_State_t DFADC_Control(DFADC_Def_t *locDFADC_p, DFADC_Control_Event_t locControlEvent_en, uint32_t locControlData_u32, void *locContext_p);


/**@brief Function for starting the sample conversion.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 *
 * @retval 	DFADC_OK 		If the conversions finished.
 */
DFADC_State_t DFADC_StartADConversion(DFADC_Def_t *locDFADC_p);


/**@brief Function for reading the distributed digital output of the LTC ADC chip.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 * @param[out] 	locReadData		Digital output of the LTC ADC.
 *                                      The number of bytes depends on the device type.
 *
 * @retval DFADC_OK 	If the result is available.
 *
 * @note This functions only returns the data when DRL is low.
 *
 * @attention Function DFADC_StartADConversion should be called first to start the conversion.
 */
DFADC_State_t DFADC_ReadDFDistributedData(DFADC_Def_t *locDFADC_p, uint8_t *locReadData_au8);


/**@brief Function for reading the synchronized digital output of the LTC ADC chips.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 * @param[in] 	locNumDevice		Number of devices to get data.
 * @param[out]	locReadData		Pointer to the digital output buffer.
 *
 * @retval DFADC_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 * @note This functions only returns the data when DRL is low.
 *
 * @attention Function DFADC_StartADConversion should be called first to start the conversion.
 */
DFADC_State_t DFADC_ReadDFSyncData(DFADC_Def_t *locDFADC_p, uint16_t locNumDevice, uint8_t *locReadData);


/**@brief Function for converting the digital ADC data to voltage.
 *
 * @param[in] 	locDFADC_p		Pointer to driver definition.
 * @param[in] 	locRawData_p		ADC digital buffer.
 * @param[out] 	locVoltage_p		Pointer to result voltage.
 *
 */
void DFADC_ConvToVoltage(DFADC_Def_t *locDFADC_p, uint8_t *locRawData_p, float *locVoltage_p);


#ifdef __cplusplus
}
#endif

#endif /* __ADI_DF_ADC_H__ */
