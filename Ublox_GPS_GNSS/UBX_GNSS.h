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
 * @file GNSS.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __UBX_GNSS_H__
#define __UBX_GNSS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "minmea/minmea.h"

#ifdef __cplusplus
extern "C" {
#endif  

#define UBXGNSS_UART_INTERFACE      (0)
#define UBXGNSS_I2C_INTERFACE       (1)
#define UBXGNSS_SPI_INTERFACE       (2)

#define UBXGNSS_NMEA_MAX_LENGTH     MINMEA_MAX_LENGTH

typedef enum
{
    UBXGNSS_OK = 0,
    UBXGNSS_FAIL,
    UBXGNSS_CRC_FAIL,
    UBXGNSS_TIMEOUT,
    UBXGNSS_COMMAND_UNKNOWN,
    UBXGNSS_OUT_OF_RANGE,
    UBXGNSS_INVALID_PARAM,
    UBXGNSS_INVALID_INTERFACE,
    UBXGNSS_INVALID_ARG,
    UBXGNSS_INVALID_OPERATION,
    UBXGNSS_MEM_ERR,
    UBXGNSS_HW_ERR,
    UBXGNSS_DATA_SENT,
    UBXGNSS_DATA_RECEIVED,
    UBXGNSS_I2C_COMM_FAILURE
} UBXGNSS_State_t;


typedef enum
{
    UART_UBX_EVENT_TRANSMIT          = 0x00,
    UART_UBX_EVENT_ABORT_TRANSMIT    = 0x01,
    UART_UBX_EVENT_ABORT_RECEIVE     = 0x02,
    I2C_UBX_EVENT_TRANSMIT           = 0x03,
    I2C_UBX_EVENT_RECEIVE            = 0x04,
    I2C_UBX_EVENT_ABORT_TRANSMIT     = 0x05,
    I2C_UBX_EVENT_ABORT_RECEIVE      = 0x06
} UBXGNSS_Comm_Event_t;


/**
 * Communication event callback type.
 *
 * This function is called when there is UART/I2C/SPI communication.
 *
 * @param[in] UBXGNSS_Comm_Event_t 	UART/SPI/I2C Event type.
 * @param[in] uint16_t			Device address.
 * @param[in] uint16_t			Register address.
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval UBXGNSS_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef UBXGNSS_State_t (*UBXGNSS_Comm_Handle_t)(UBXGNSS_Comm_Event_t, uint16_t, uint16_t, uint8_t *, uint16_t, void *);


/**
 * Busy time counting handle callback.
 *
 * @param[in] uint32_t    Delay time
 * @retval true           If the device is busy
 */
typedef bool (*UBXGNSS_Delay_Handle_t)(uint32_t);


/** Ublox GNSS definition structure.  */
typedef struct
{
        uint8_t                 interface;
        uint8_t                 address;
        uint8_t                 nmeaDataSource;
	uint32_t		timeout;
	UBXGNSS_Comm_Handle_t 	const commHandle;
	UBXGNSS_Delay_Handle_t 	const delayHandle;
} UBXGNSS_Def_t;


/**@brief Function for initializing the Ublox GNSS driver.
 *
 * @param[in] 	locUBXGNSS_p		Pointer to driver definition.
 *
 * @retval 	UBXGNSS_OK 		If the driver was initialized successfully.
 *
 */
UBXGNSS_State_t UBXGNSS_Init(UBXGNSS_Def_t *locUBXGNSS_p);


/**@brief Function for de-initializing Ublox GNSS driver.
 *
 * @param[in] 	locUBXGNSS_p		Pointer to driver definition.
 *
 */
void UBXGNSS_DeInit(UBXGNSS_Def_t *locUBXGNSS_p);


/**@brief Function for data waiting and processing of Ublox GNSS driver.
 *
 * @param[in] 	locUBXGNSS_p		Pointer to driver definition.
 * @param[in]   locCommData_p           Pointer to communication data.
 *
 * @note        This function should be put in the interrupt handler or in the loop.
 */
void UBXGNSS_ProcessData(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t locDataSource, uint8_t *locCommData_p, uint16_t locCommDataSize_u16);


#ifdef __cplusplus
}
#endif

#endif /* __UBX_GNSS_H__ */