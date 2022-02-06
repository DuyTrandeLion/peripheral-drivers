/*
 * pca9685.h
 *
 *  Created on: Jan 31, 2022
 *      Author: Duy Lion Tran (Gabriel)
 */

#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define INTERNAL_OSCILLATOR   25000000U

#define REGISTER_MODE1        0x00
#define REGISTER_MODE2        0x01

#define REGISTER_SUBADR1      0x02
#define REGISTER_SUBADR2      0x03
#define REGISTER_SUBADR3      0x04

#define REGISTER_LED0_ON_L    0x06
#define REGISTER_LED0_ON_H    0x07
#define REGISTER_LED0_OFF_L   0x08
#define REGISTER_LED0_OFF_H   0x09

#define REGISTER_ALLCALLADR   0x05

#define REGISTER_ALL_LED_ON   0xFA
#define REGISTER_ALL_LED_OFF  0xFC

#define REGISTER_PRE_SCALE    0xFE        /* output frequency = clock frequency / (4096 * (presale + 1)) */
#define REGISTER_TestMode     0xFF

#define PCA9685_ALL_LED       REGISTER_ALL_LED_ON

#define SUBADDRESS1           0x01
#define SUBADDRESS2           0x02
#define SUBADDRESS3           0X03


/** MCP4xxx potentiometer return codes. */
typedef enum
{
    PCA9685_OK                          = 0x00,
    PCA9685_BUSY                        = 0x01,
    PCA9685_NULL_PARAM                  = 0x02,
    PCA9685_COMM_ERROR                  = 0x03,
    PCA9685_INVALID_MODE                = 0x04,
    PCA9685_INVALID_ADDRESS             = 0x05,
    PCA9685_INVALID_PARAM               = 0x06,
    PCA9685_UNKNOWN_ERROR               = 0x07,
    PCA9685_COMM_TIMEOUT                = 0x08
} PCA9685_State_t;

/** I2C event types of communication. */
typedef enum
{
    I2C_EVENT_TRANSMIT                  = 0x00,
    I2C_EVENT_RECEIVE                   = 0x01,
    I2C_EVENT_TRANSMIT_RECEIVE          = 0x02,
    I2C_EVENT_TRANSMIT_RECEIVE_START    = 0x03
} PCA9685_I2C_Event_t;


/**
 * I2C event callback type.
 *
 * This function is called when there is SPI communication.
 *
 * @param[in] PCA9685_I2C_Event_t 	I2C Event type.
 * @param[in] uint8_t                   I2C address of the peripheral
 * @param[in] uint8_t                   Register address
 * @param[in] uint8_t *			Pointer to data buffer.
 * @param[in] uint16_t			Size of data buffer.
 * @param[in] void *			Pointer to parameter of event handler
 *
 * @retval PCA9685_OK If the notification was sent successfully. Otherwise, an error code is returned.
 */
typedef PCA9685_State_t (*PCA9685_I2C_Handle_t)(PCA9685_I2C_Event_t locCommEvent_en,
                                                uint8_t locDeviceAddress_u16,
                                                uint8_t locRegisterAddress_u8,
                                                uint8_t *locData_p8,
                                                uint16_t locDataSize_u16,
                                                void *locContext_p);


typedef void (*PCA9685_Delay_Handle_t)(uint32_t locDelayPeriodMS_u32);


/** PWM definition structure.  */
typedef struct
{
    uint8_t                 deviceAddress;        /* default device address in 7-bit mode: 1|a5|a4|a3|a2|a1|a0|r/w */
    uint8_t                 enableAllCall;
    uint8_t                 enableExternalClock;
    uint8_t                 prescale;
    uint8_t                 mode1Register;
    uint8_t                 mode2Register;
    PCA9685_I2C_Handle_t    i2cHandle;
    PCA9685_Delay_Handle_t  delayHandle;
} PCA9685_Def_t;


/**
 * @brief Function to initialize and setup the PWM controller.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 *
 * @retval PCA9685_OK			If the driver was initialized successfully.
 *
 */
PCA9685_State_t PCA9685_Init(PCA9685_Def_t *locPCA9685_p);


/**
 * @brief Function to put the PWM controller to sleep.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 *
 * @retval PCA9685_OK			If the PWM controller can sleep.
 *
 */
PCA9685_State_t PCA9685_Sleep(PCA9685_Def_t *locPCA9685_p);


/**
 * @brief Function to wake the PWM controller .
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 *
 * @retval PCA9685_OK			If the PWM controller can wake.
 *
 */
PCA9685_State_t PCA9685_Wake(PCA9685_Def_t *locPCA9685_p);


/**
 * @brief Function to reset PWM controller by putting it to sleep then wake.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 *
 * @retval PCA9685_OK			If the controller is successfully reseted.
 *
 */
PCA9685_State_t PCA9685_SoftwareReset(PCA9685_Def_t *locPCA9685_p);


/**
 * @brief Function to read a single subaddress of the PWM controller.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 * @param[in] locSubAddressIndex_u8	Index of the desired sub address.
 * @param[out] locSubAddress_p8         Pointer to store the read sub address.
 *
 * @retval PCA9685_OK			If the sub adddress is successfully read.
 *
 */
PCA9685_State_t PCA9685_ReadSubAddress(PCA9685_Def_t *locPCA9685_p, uint8_t locSubAddressIndex_u8, uint8_t *locSubAddress_p8);


/**
 * @brief Function to setup the new subaddress of the PWM controller.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 * @param[in] locSubAddressIndex_u8	Index of the desired sub address.
 * @param[in] locSubAddress_u8		New sub address to be writen.
 *
 * @retval PCA9685_OK			If the sub adddress is successfully writen.
 *
 */
PCA9685_State_t PCA9685_WriteSubAddress(PCA9685_Def_t *locPCA9685_p, uint8_t locSubAddressIndex_u8, uint8_t locSubAddress_u8);


/**
 * @brief Function to read the output presale.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 * @param[in] locPrescale_p8            Pointer to presale to be read.
 *
 * @retval PCA9685_OK			If the output presale is successfully read.
 *
 */
PCA9685_State_t PCA9685_ReadOutputPrescale(PCA9685_Def_t *locPCA9685_p, uint8_t *locPrescale_p8);


/**
 * @brief Function to set the output presale.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 * @param[in] locPrescale_u8            Desired presale to be writen.
 *
 * @retval PCA9685_OK			If the output presale is successfully writen.
 *
 */
PCA9685_State_t PCA9685_SetOutputPrescale(PCA9685_Def_t *locPCA9685_p, uint8_t locPrescale_u8);


/**
 * @brief Function to read active time of a sigle channel.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 * @param[in] locLEDIndex_u8            Channel index to be read.
 * @param[in] locSubAddress_u8		Pointer to store the channel PWM output.
 *
 * @retval PCA9685_OK			If the channel output is successfully read.
 *
 */
PCA9685_State_t PCA9685_ReadLEDActiveTime(PCA9685_Def_t *locPCA9685_p, uint8_t locLEDIndex_u8, uint16_t *locActiveTime_p16);


/**
 * @brief Function to initialize and setup the PWM driver.
 *
 * @param[in] locPCA9685_p		Pointer to driver definition.
 * @param[in] locLEDIndex_u8            Channel index to be writen.
 * @param[in] locActiveTime_u16		New channel output to be updated.
 *
 * @retval PCA9685_OK			If the channel output is successfully writen.
 *
 */
PCA9685_State_t PCA9685_SetLEDActiveTime(PCA9685_Def_t *locPCA9685_p, uint8_t locLEDIndex_u8, uint16_t locActiveTime_u16);


#ifdef __cplusplus
}
#endif

#endif /* _PCA9685_H_ */
