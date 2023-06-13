/*
 * ltc2497.h
 *
 *  Created on: Jun 6, 2022
 *      Author: Duy Lion Tran (Gabriel)
 */

#ifndef _LTC2497_H_
#define _LTC2497_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define MAX_NUMBER_OF_CHANNELS          16

#define CONFIG_CHANNEL_ENABLED          (0x01 << 5)
#define CONFIG_DIFF_CHANNEL_ENABLED     (0x01 << 4)
#define CONFIG_EVEN_CHANNEL_POLARITY    (0x01 << 3)

#define CHANNEL_ENABLED                 (0x01 << 5)
#define DIFFERENTIAL_INPUT_ENABLED      (0x01 << 4)
#define ODD_INPUT_ENABLED               (0x01 << 3)


/** LTC249 Delta-Sigma ADC return codes. */
typedef enum
{
    LTC2497_OK                          = 0x00,
    LTC2497_BUSY                        = 0x01,
    LTC2497_NULL_PARAM                  = 0x02,
    LTC2497_COMM_ERROR                  = 0x03,
    LTC2497_INVALID_MODE                = 0x04,
    LTC2497_INVALID_ADDRESS             = 0x05,
    LTC2497_INVALID_PARAM               = 0x06,
    LTC2497_UNKNOWN_ERROR               = 0x07,
    LTC2497_COMM_TIMEOUT                = 0x08
} LTC2497_State_t;


/** I2C event types of communication. */
typedef enum
{
    LTC2497_I2C_EVENT_TRANSMIT                  = 0x00,
    LTC2497_I2C_EVENT_RECEIVE                   = 0x01,
    LTC2497_I2C_EVENT_TRANSMIT_RECEIVE          = 0x02,
    LTC2497_I2C_EVENT_TRANSMIT_RECEIVE_START    = 0x03
} LTC2497_I2C_Event_t;


/**
 * I2C event callback type.
 *
 * This function is called when there is I2C communication.
 *
 * @param[in] LTC2497_I2C_Event_t 	I2C Event type.
 * @param[in] uint8_t                   I2C address of the peripheral
 * @param[in] uint8_t                   Register address
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval LTC2497_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef LTC2497_State_t (*LTC2497_I2C_Handle_t)(LTC2497_I2C_Event_t locCommEvent_en,
                                                uint8_t locDeviceAddress_u16,
                                                uint8_t locRegisterAddress,
                                                uint8_t *locData_p8,
                                                uint16_t locDataSize_u16,
                                                void *locContext_p);


/**
 * Delay handler.
 *
 */
typedef void (*LTC2497_Delay_Handle_t)(uint32_t locDelayPeriodMS_u32);


/** ADC channel configuration structure.  */
typedef struct
{
    uint8_t channelEnabled;
    uint8_t differentialChannelEnabled;
    uint8_t oddInputEnabled;
} LTC2497_Channel_Config_Def_t;


/** ADC channel definition structure.  */
typedef struct
{
    LTC2497_Channel_Config_Def_t  config;
    int32_t                       rawADCValue;
} LTC2497_Channel_Def_t;


/** ADC definition structure.  */
typedef struct
{
    uint8_t                       deviceAddress;
    float                         refVoltage;
    LTC2497_Channel_Def_t         channel[MAX_NUMBER_OF_CHANNELS];
    LTC2497_I2C_Handle_t          i2cHandle;
    LTC2497_Delay_Handle_t        delayHandle;
} LTC2497_Def_t;


/**
 * @brief Function to initialize and setup the ADC controller.
 *
 * @param[in] locLTC2497_p		Pointer to the instance handles.
 *
 * @retval LTC2497_OK			If the driver was initialized successfully.
 *
 */
LTC2497_State_t LTC2497_Init(LTC2497_Def_t *locLTC2497_p);


/**@brief Function for reading the distributed digital output of the ADC chip.
 *
 * @param[in] 	locLTC2497_p            Pointer to the instance handles.
 * @param[in] 	locChannelConfig_s	Channel configuration structure.
 * @param[in] 	locChannelIndex_u8	Index of the channel to read.
 * @param[out]	locChannelADC_pi32	Pointer to the output raw ADC.
 *
 * @retval LTC2497_OK 	If the result is available.
 *
 */
LTC2497_State_t LTC2497_ReadChannelADC(LTC2497_Def_t *locLTC2497_p, LTC2497_Channel_Config_Def_t locChannelConfig_s, int8_t locChannelIndex_u8, int32_t *locChannelADC_pi32);


/**@brief Function for converting the digital ADC data to voltage.
 *
 * @param[in] 	locLTC2497_p            Pointer to the instance handles.
 * @param[in] 	locRawADC_i32		Raw ADC to be converted.
 * @param[out] 	locVoltage_p		Pointer to result voltage.
 *
 */
LTC2497_State_t LTC2497_ConvertRawADC2Voltage(LTC2497_Def_t *locLTC2497_p, uint32_t locRawADC_i32, float *locVoltage_fp);


#ifdef __cplusplus
}
#endif

#endif /* _LTC2497_H_ */
