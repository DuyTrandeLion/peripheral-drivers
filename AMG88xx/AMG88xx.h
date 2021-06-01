#ifndef __AMG88XX_H__
#define __AMG88XX_H__


#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define AMG88xx_POWER_CTL_REG         0x00 /*< Setting operating mode of device */
#define AMG88xx_RESET_REG             0x01 /*< Writing to reset software. */
#define AMG88xx_FPSC_REG              0x02 /*< Setting Frame Rate. */
#define AMG88xx_INT_CTL_REG           0x03 /*< Setting Interrupt Function. */
#define AMG88xx_STAT_REG              0x04 /*< Indicate Overflow Flag and Interrupt Flag. */
#define AMG88xx_SCLR_REG              0x05 /*< Status Clear Register. */
#define AMG88xx_AVE_REG               0x07 /*< Setting moving average Output Mode. */
#define AMG88xx_INTHL_REG             0x08 /*< Interrupt upper value£¨Upper level£©. */
#define AMG88xx_INTHH_REG             0x09 /*< Interrupt upper value£¨Upper level£©. */
#define AMG88xx_INTLL_REG             0x0A /*< Interrupt lower value£¨Lower level£©. */
#define AMG88xx_INTLH_REG             0x0B /*< Interrupt lower value£¨upper level£©. */
#define AMG88xx_IHYSL_REG             0x0C /*< Interrupt hysteresis value£¨Lower level£©. */
#define AMG88xx_IHYSH_REG             0x0D /*< Interrupt hysteresis value£¨Lower level£©. */
#define AMG88xx_TTHL_REG              0x0E /*< Thermistor Output Value£¨Lower level£©. */
#define AMG88xx_TTHH_REG              0x0F /*< Thermistor Output Value£¨Upper level£©. */
#define AMG88xx_INT0_REG              0x10 /*< Pixel 1 to 8 Interrupt Result. */
#define AMG88xx_INT1_REG              0x11 /*< Pixel 9 to 16 Interrupt Result. */
#define AMG88xx_INT2_REG              0x12 /*< Pixel 17 to 24 Interrupt Result. */
#define AMG88xx_INT3_REG              0x13 /*< Pixel 25 to32 Interrupt Result. */
#define AMG88xx_INT4_REG              0x14 /*< Pixel 33 to 40 Interrupt Result. */
#define AMG88xx_INT5_REG              0x15 /*< Pixel 41 to 48 Interrupt Result. */
#define AMG88xx_INT6_REG              0x16 /*< Pixel 49 to 56 Interrupt Result. */
#define AMG88xx_INT7_REG              0x17 /*< Pixel 57 to 64 Interrupt Result. */

#define AMG88xx_PIXEL_BASE            0x80 /*< Pixel 1 Output Value (Lower Level). */
#define AMG88xx_PIXEL_ADDR(n)         (AMG88xx_PIXEL_BASE + n)

#define DEVICE_ADDRESS_EVEN           0x68                        /* Connect terminal 5 (AD_SELECT) to GND */
#define DEVICE_ADDRESS_ODD            0x69                        /* Connect terminal 5 (AD_SELECT) to VDD */

#define NORMAL_MODE                   0x00
#define SLEEP_MODE                    0x10

#define FLAG_RESET                    0x30
#define INITIAL_RESET                 0x3F

#define FRAME_RATE_1FPS               0x01
#define FRAME_RATE_10FPS              0x10

#define ABS_INTERRUPT_MODE            0x01
#define DIFF_INTERRUPT_MODE           0x00

#define OVS_INT_CLR                   (0x01 << 2)   /*< Temperature output overflow flag reset */
#define INT_CLR                       (0x01 << 1)   /*< Interrupt flag reset */

#define ENABLE_INTERRUPT              0x01
#define DISABLE_INTERRUPT             0x00

#define MOVING_AVERAGE_ENABLE         0x01
#define MOVING_AVERAGE_DISABLE        0x00

#define TEMPERATURE_OVERFLOW          (0x01 << 2)
#define INTERRUPT_OUTBREAK            (0x01 << 1)


typedef enum
{
    AMG88xx_OK                = 0x00,
    AMG88xx_BUSY              = 0x01,
    AMG88xx_NULL_PARAM        = 0x02,
    AMG88xx_COMM_ERROR        = 0x03,
    AMG88xx_INVALID_MODE      = 0x04,
    AMG88xx_INVALID_ADDRESS   = 0x05,
    AMG88xx_INVALID_PARAM     = 0x06,
    AMG88xx_UNKNOWN_ERROR     = 0x09
} AMG88xx_State_t;


typedef enum
{
    I2C_EVENT_TRANSMIT          = 0x00,
    I2C_EVENT_RECEIVE           = 0x01,
    I2C_EVENT_TRANSMIT_RECEIVE  = 0x02,
    I2C_EVENT_TRANSMIT_REPEATED_START  = 0x03
} AMG88xx_Event_t;


/**
 * Communication event callback type.
 *
 * @brief This function is called when there is I2C communication.
 *
 * @param[in] ICP_Event_t 	SPI/I2C Event type.
 * @param[in] uint16_t		Device address.
 * @param[in] uint8_t *		Pointer to data buffer.
 * @param[in] uint16_t		Size of data buffer.
 * @param[in] void *		Pointer to parameter of event handler
 *
 * @retval AMG88xx_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 */
typedef AMG88xx_State_t (*AMG88xxs_Com_Handle_t)(AMG88xx_Event_t locCommEvent_en,
                                                 uint16_t locDeviceAddress_u16,
                                                 uint8_t locRegisterAddress_u8,
                                                 uint8_t *locData_p8,
                                                 uint16_t locDataSize_u16,
                                                 void *locContext_p);


/**
 * Busy time counting handle callback.
 *
 * @retval true 	If the device is busy
 *
 */
typedef void (*AMG88xx_Delay_Handle_t)(uint32_t);


typedef struct
{
    uint8_t deviceAddress;
    AMG88xxs_Com_Handle_t commHandle;
    AMG88xx_Delay_Handle_t delayHandle;
} AMG88xx_Def_t;


/**
 * @brief Function to initialize and setup the sensor.
 *
 * @param[in] locAMG88xx_p      Pointer to driver definition.
 *
 * @retval AMG88xx_OK           If the driver was initialized successfully.
 *
 * @note The reading from OTP sensor should be performed after power up or after SW reset.
 *
 */
AMG88xx_State_t AMG88xx_Init(AMG88xx_Def_t *locAMG88xx_p);


/**
 * @brief Function to read current sensor operation mode.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[out] locOperationMode_p8      Pointer to target operation mode.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_GetOperationMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locOperationMode_p8);


/**
 * @brief Function set sensor operation mode.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[in] locOperationMode_u8       Disired operation mode.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_SetOperationMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locOperationMode_u8);


/**
 * @brief Function to read current sensor reset mode.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[out] locResetMode_p8          Pointer to target reset mode.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_GetResetMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locResetMode_p8);


/**
 * @brief Function to set sensor reset mode.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[in] locResetMode_u8           Disired reset mode.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_SetResetMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locResetMode_u8);


/**
 * @brief Function to read current frame rate of sensor data (data rate in frames per second).
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[out] locFPSCMode_p8           Pointer to target frame rate.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_GetFPSCMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locFPSCMode_p8);


/**
 * @brief Function to set sensor frame rate.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[in] locFPSCMode_u8            Desired frame rate.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_SetFPSCMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locFPSCMode_u8);


/**
 * @brief Function to read current sensor interrupt mode.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[out] locInterruptMode_p8      Pointer to target interrupt mode.
 * @param[out] locInterruptMode_p8      Pointer to target interrupt status.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_GetInterruptMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locInterruptMode_p8, uint8_t *locInterruptStatus_u8);


/**
 * @brief Function to set sensor interrupt mode and enable/disable interrupt.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[in] locInterruptMode_u8       Desired interrupt mode.
 * @param[in] locInterruptStatus_u8     Desired interrupt output: active or inactive
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_SetInterruptMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locInterruptMode_u8, uint8_t locInterruptStatus_u8);


/**
 * @brief Function to read status.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[OUT] locTemperatureOF_u8      Pointer to temperature output overflow flag.
 * @param[OUT] locInterruptOutbreak_u8  Pointer to interrupt outbreak flag.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_ReadStatus(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locTemperatureOF_u8, uint8_t *locInterruptOutbreak_u8);


/**
 * @brief Function to clear status flag.
 *
 * @param[in] locAMG88xx_p              Pointer to driver definition.
 * @param[in] locStatusFlag_u8          Desired status flag to be cleared.
 *
 * @retval AMG88xx_OK                   If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_ClearStatus(AMG88xx_Def_t *locAMG88xx_p, uint8_t locStatusFlag_u8);


/**
 * @brief Function to set interrupt level: higher limit, lower limit, delay offset (hysteresis).
 *
 * @param[in] locAMG88xx_p        Pointer to driver definition.
 * @param[in] locHigh_f           Desired upper temperature interrupt level.
 * @param[in] locLow_f            Desired lower temperature interrupt level.
 * @param[in] locHysteresis_f     Desired delayed interrupt level.
 *
 * @retval AMG88xx_OK             If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_SetInterruptLevels(AMG88xx_Def_t *locAMG88xx_p, float locHigh_f, float locLow_f, float locHysteresis_f);


/**
 * @brief Function to enable/disable moving average mode.
 *
 * @param[in] locAMG88xx_p                      Pointer to driver definition.
 * @param[in] locMovingAverageModeEnable_u8     Desired avering mode.
 *
 * @retval AMG88xx_OK                           If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_SetMovingAveragetMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locMovingAverageModeEnable_u8);


/**
 * @brief Function to read raw and float converted sensor internal termistor.
 *
 * @param[in] locAMG88xx_p            Pointer to driver definition.
 * @param[out] locTemperature_p8      Pointer to target raw temperature, cannot be null.
 * @param[out] locTemperature_pf      Pointer to converted temperature, can be null.
 *
 * @retval AMG88xx_OK                 If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_ReadThermistor(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locTemperature_p8, float *locTemperature_pf);


/**
 * @brief Function to read raw temperature of all pixels and store the result in a buffer.
 *
 * @param[in] locAMG88xx_p                Pointer to driver definition.
 * @param[out] locPixelTemperature_p16    Pointer to raw pixel temperature buffer, cannot be null.
 * @param[out] locPixelTemperature_pf     Pointer to converted pixel temperature buffer, can be null.
 *
 * @retval AMG88xx_OK                     If the communication succeeded.
 *
 */
AMG88xx_State_t AMG88xx_ReadPixelsTemperature(AMG88xx_Def_t *locAMG88xx_p, int16_t *locPixelTemperature_p16, float *locPixelTemperature_pf);


#ifdef __cplusplus
}
#endif


#endif /* __AMG88XX_H__ */