#ifndef __ICP101XX_H__
#define __ICP101XX_H__

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define ICP_I2C_ADDRESS     0x63


#define ICP_CMD_SOFT_RESET  0x805D
#define ICP_CMD_READ_ID     0xEFC8
#define ICP_CMD_SET_ADDR    0xC595
#define ICP_CMD_READ_OTP    0xC7F7

/*
The ICP-101xx provides the possibility to define the sensor behavior during measurement as well as the transmission sequence of
measurement results. These characteristics are defined by the appropriate measurement command.
Each measurement command triggers both a temperature and a pressure measurement.

|--------------------------------------------------------------|
| OPERATION MODE        | TRANSMIT T FIRST  | TRANSMIT P FIRST |
|--------------------------------------------------------------|
| Low Power (LP)        | 0x609C            | 0x401A           |
| Normal (N)            | 0x6825            | 0x48A3           |
| Low Noise (LN)        | 0x70DF            | 0x5059           |
| Ultra-Low Noise (ULN) | 0x7866            | 0x58E0           |
|--------------------------------------------------------------|
*/
#define ICP_CMD_MEASURE_LP_T_FIRST  0x609C
#define ICP_CMD_MEASURE_N_T_FIRST   0x6825
#define ICP_CMD_MEASURE_LN_T_FIRST  0x70DF
#define ICP_CMD_MEASURE_ULN_T_FIRST 0x7866
#define ICP_CMD_MEASURE_LP_P_FIRST  0x401A
#define ICP_CMD_MEASURE_N_P_FIRST   0x48A3
#define ICP_CMD_MEASURE_LN_P_FIRST  0x5059
#define ICP_CMD_MEASURE_ULN_P_FIRST 0x58E0


typedef enum
{
    ICP_OK                = 0x00,
    ICP_BUSY              = 0x01,
    ICP_NULL_PARAM        = 0x02,
    ICP_COMM_ERROR        = 0x04,
    ICP_DATA_ORDER_ERROR  = 0x05,
    ICP_TEMPERATURE_FIRST = 0x06,
    ICP_PRESSURE_FIRST    = 0x07,
    ICP_INVALID_MODE      = 0x08,
    ICP_UNKNOWN_ERROR     = 0x09
} ICPPress_State_t;


typedef enum
{
    I2C_EVENT_TRANSMIT          = 0x00,
    I2C_EVENT_RECEIVE           = 0x01,
    I2C_EVENT_TRANSMIT_RECEIVE  = 0x02
} ICPPress_Event_t;


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
 * @retval ICP_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 */
typedef ICPPress_State_t (*ICPPress_Com_Handle_t)(ICPPress_Event_t, uint16_t, uint8_t *, uint16_t, void *);


/**
 * Busy time counting handle callback.
 *
 * @retval true 	If the device is busy
 *
 */
typedef uint8_t (*ICPPress_Delay_Handle_t)(uint32_t);


typedef struct
{
  uint16_t                sensorMeasurementMode;
  uint8_t                 sensorDataOutMode;
  float                   sensorConstants[4];
  float                   pPaCalib[3];
  float                   LUT_lower;
  float                   LUT_upper;
  double                  quadrFactor;
  float                   offsetFactor;
  ICPPress_Com_Handle_t   commHandle;
  ICPPress_Delay_Handle_t delayHandle;
} ICPPRess_Def_t;


/**
 * @brief Function sends a command to read calibration data from OTP sensor and other initialization data, which is necessary for calculations.
 *
 * @param[in] locICPPress_p     Object where initialization data and data from OTP sensor to be stored
 *
 * @retval ICP_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 * @note The reading from OTP sensor should be performed after power up or after SW reset.
 *
 */
ICPPress_State_t ICPPress_Init(ICPPRess_Def_t *locICPPress_p);


/**
 * @brief Function sends a command to perform a SW Reset of the device.
 *
 * @param[in] locICPPress_p     Object where initialization data and data from OTP sensor to be stored
 *
 * @note This command triggers the sensor to reset all internal state machines and reload calibration data from the memory.
 *
 */
void ICPPress_SoftReset(ICPPRess_Def_t *locICPPress_p);


/**
 * @brief Read ID function
 *
 * @param[in] locICPPress_p     Object where initialization data and data from OTP sensor to be stored
 * @param[out] locDeviceID_u16  Pointer to memory where device ID to be stored
 *
 * @retval ICP_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 */
ICPPress_State_t ICPPress_ReadID(ICPPRess_Def_t *locICPPress_p, uint16_t *locDeviceID_u16);


/**
 * @brief Configure a measurement mode and data reading order
 *
 * @param[in] locICPPress_p     Object where initialization data and data from OTP sensor to be stored
 * @param[in] locModeCmd_u16    Measuring mode to be set
 *
 * @retval ICP_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 */
ICPPress_State_t ICPPress_SetMeasurementMode(ICPPRess_Def_t *locICPPress_p, uint16_t locModeCmd_u16);


/**
 * @brief Read Raw ADC data including 16-bit temperature and 24-bit pressure
 *
 * @param[in] locICPPress_p           Object where initialization data and data from OTP sensor be stored
 * @param[out] locRawTemperature_p16  Pointer to memory where Temperature ADC to be stored
 * @param[out] locRawPressure_p32     Pointer to memory where Pressure ADC to be stored
 *
 * @retval ICP_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 */
ICPPress_State_t ICPPress_ReadRawData(ICPPRess_Def_t *locICPPress_p, int16_t *locRawTemperature_p16, uint32_t *locRawPressure_p32);


/**
 * @brief Read Raw ADC data then calculate these values to standard units.
 *
 * @param[in] locICPPress_p       Object where initialization data and data from OTP sensor be stored
 * @param[out] locTemperature_pf  Pointer to memory wher Temperature to be stored
 * @param[out] locPressure_pf     Pointer to memory where Pressure to be stored
 * @param[out] locAltitude_pf     Pointer to memory where Altitude to be stored
 *
 * @retval ICP_OK If the notification was sent successfully. Otherwise, an error code is returned.
 *
 */
ICPPress_State_t ICPPress_GetProcessedData(ICPPRess_Def_t *locICPPress_p, float *locTemperature_pf, float *locPressure_pf, float *locAltitude_pf);
 
#ifdef __cplusplus
}
#endif

#endif /* __ICP101XX_H__ */