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

#include "lwgps.h"

#ifdef __cplusplus
extern "C" {
#endif


#define UBXGNSS_UART_INTERFACE      (0x01)
#define UBXGNSS_I2C_INTERFACE       (0x01 << 1)
#define UBXGNSS_SPI_INTERFACE       (0x01 << 2)

#define UBXGNSS_NMEA_MAX_LENGTH     (1536)
#define UBXGNSS_UBX_MAX_LENGHTH     (256)

#define UBX_MSG_HEADER_LENGHTH      (6)


typedef enum
{
    UBXGNSS_OK = 0,
    UBXGNSS_FAIL,
    UBXGNSS_CRC_FAIL,
    UBXGNSS_INVALID_MSG,
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
    UBXGNSS_DATA_NOT_AVAILABLE,
    UBXGNSS_I2C_COMM_FAILURE,
    UBXGNSS_SPI_COMM_FAILURE
} UBXGNSS_State_t;


typedef enum
{
    UART_UBX_EVENT_TRANSMIT          = 0x00,
    UART_UBX_EVENT_ABORT_TRANSMIT    = 0x01,
    UART_UBX_EVENT_ABORT_RECEIVE     = 0x02,
    I2C_UBX_EVENT_TRANSMIT           = 0x03,
    I2C_UBX_EVENT_RECEIVE            = 0x04,
    I2C_UBX_EVENT_ABORT_TRANSMIT     = 0x05,
    I2C_UBX_EVENT_ABORT_RECEIVE      = 0x06,
    SPI_UBX_EVENT_TRANSMIT           = 0x07,
    SPI_UBX_EVENT_RECEIVE            = 0x08,
    SPI_UBX_EVENT_ABORT_TRANSMIT     = 0x09,
    SPI_UBX_EVENT_ABORT_RECEIVE      = 0x0A
} UBXGNSS_Comm_Event_t;

typedef enum
{
    UBX_ACK = 0x05,
    UBX_CFG = 0x06,
    UBX_INF = 0x04,
    UBX_LOG = 0x21,
    UBX_MGA = 0x13,
    UBX_MON = 0x0A,
    UBX_NAV = 0x01,
    UBX_RXM = 0x02,
    UBX_SEC = 0x27,
    UBX_TIM = 0x0D,
    UBX_UPD = 0x09
} UBXMESSAGE_t;


#define UBXGNSS_IS_DATA_VALID(UBX_Hdl)    ((UBX_Hdl).gps.is_valid)


#define UBXGNSS_GET_FIX(UBX_Hdl)          ((UBX_Hdl).gps.fix)


#define UBXGNSS_GET_FIX_MODE(UBX_Hdl)     ((UBX_Hdl).gps.fix_mode)


#define UBXGNSS_GET_LATITUDE(UBX_Hdl)     ((UBX_Hdl).gps.latitude)


#define UBXGNSS_GET_LONGITUDE(UBX_Hdl)    ((UBX_Hdl).gps.longitude)


#define UBXGNSS_GET_ALTITUDE(UBX_Hdl)       ((UBX_Hdl).gps.altitude)


#define UBXGNSS_GET_SATS_IN_USE(UBX_Hdl)    ((UBX_Hdl).gps.sats_in_use)


#define UBXGNSS_GET_SATS_IN_VIEW(UBX_Hdl)   ((UBX_Hdl).gps.sats_in_view)


#define UBXGNSS_GET_TIME(UBX_Hdl, h, m, s)  ((h) = (UBX_Hdl).gps.hours);    \
                                            ((m) = (UBX_Hdl).gps.minutes);  \
                                            ((s) = (UBX_Hdl).gps.seconds)


#define UBXGNSS_GET_DATE(UBX_Hdl, y, m, d)  ((y) = (UBX_Hdl).gps.year);   \
                                            ((m) = (UBX_Hdl).gps.month);  \
                                            ((d) = (UBX_Hdl).gps.date)

#define UBXGNSS_GET_TIME_TO_FIRST_FIX(UBX_Hdl)  ((UBX_Hdl).timeToFirstFix)


/**
 * \brief           List of optional speed transformation from GPS values (in knots)
 */
//typedef enum {
//    /* Metric values */
//    gps_speed_kps,                              /*!< Kilometers per second */
//    gps_speed_kph,                              /*!< Kilometers per hour */
//    gps_speed_mps,                              /*!< Meters per second */
//    gps_speed_mpm,                              /*!< Meters per minute */
//
//    /* Imperial values */
//    gps_speed_mips,                             /*!< Miles per second */
//    gps_speed_mph,                              /*!< Miles per hour */
//    gps_speed_fps,                              /*!< Foots per second */
//    gps_speed_fpm,                              /*!< Foots per minute */
//
//    /* Optimized for runners/joggers */
//    gps_speed_mpk,                              /*!< Minutes per kilometer */
//    gps_speed_spk,                              /*!< Seconds per kilometer */
//    gps_speed_sp100m,                           /*!< Seconds per 100 meters */
//    gps_speed_mipm,                             /*!< Minutes per mile */
//    gps_speed_spm,                              /*!< Seconds per mile */
//    gps_speed_sp100y,                           /*!< Seconds per 100 yards */
//
//    /* Nautical values */
//    gps_speed_smph,                             /*!< Sea miles per hour */
//} gps_speed_t;
#define UBXGNSS_GET_INSTANT_SPEED(UBX_Hdl, ts)  (lwgps_to_speed((UBX_Hdl).gps.speed, (ts)))


#define UBXGNSS_GET_H_DILUTION(UBX_Hdl)     ((UBX_Hdl).gps.dop_h)


#define UBXGNSS_GET_P_DILUTION(UBX_Hdl)     ((UBX_Hdl).gps.dop_p)


#define UBXGNSS_GET_V_DILUTION(UBX_Hdl)     ((UBX_Hdl).gps.dop_v)


#define UBXGNSS_GET_COG(UBX_Hdl)            ((UBX_Hdl).gps.coarse)


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
typedef UBXGNSS_State_t (*UBXGNSS_Comm_Handle_t)(UBXGNSS_Comm_Event_t,
                                                 uint16_t locDeviceAddress_u16,
                                                 uint16_t locRegisterAddress_u16,
                                                 uint8_t *locCommData_p8,
                                                 uint16_t locCommDataSize_u16,
                                                 void *locContext_p);


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
        uint32_t                timeToFirstFix;       /* ms */
        lwgps_t                 gps;
	UBXGNSS_Comm_Handle_t 	commHandle;
	UBXGNSS_Delay_Handle_t 	delayHandle;
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
void UBXGNSS_ProcessData(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locCommData_p, uint16_t locCommDataSize_u16);


/**@brief Function for getting UBX message.
 *
 * @param[in] 	locUBXGNSS_p		Pointer to driver definition.
 * @param[in]   locMsg_p                Pointer to data buffer to store the message.
 * @param[out]  locReadMsgSize_p        Pointer to the message length.
 */
UBXGNSS_State_t UBXGNSS_GetUBXMessage(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locMsg_p, uint16_t *locReadMsgSize_p);


/**@brief Function for processing UBX message.
 *
 * @param[in] 	locUBXGNSS_p		Pointer to driver definition.
 * @param[in]   locCommData_p           Pointer to the message to be processed.
 * @param[in]   locMsgLen_u16           Length of the message to be processed.
 *
 * @note        This function should be put in the interrupt handler or in the loop.
 */
UBXGNSS_State_t UBXGNSS_ProcessUBXMsg(UBXGNSS_Def_t *locUBXGNSS_p, uint8_t *locMsg_p, uint16_t locMsgLen_u16);

#ifdef __cplusplus
}
#endif

#endif /* __UBX_GNSS_H__ */