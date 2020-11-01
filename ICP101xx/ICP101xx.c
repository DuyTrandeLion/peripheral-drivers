#include "ICP101xx.h"
#include <math.h>


#define NULL_CHECK_PARAM(ICPPress_Def)  if ((NULL == ICPPress_Def) || (NULL == ICPPress_Def->commHandle) || (NULL == ICPPress_Def->delayHandle)) { return ICP_NULL_PARAM; }

static void calculate_conversion_constants(float *p_Pa, float *p_LUT, float *out);

static uint8_t gUpdateCheck_u8 = 0;


ICPPress_State_t ICPPress_Init(ICPPRess_Def_t *locICPPress_p)
{
    uint8_t restart_i2c = 1;
    uint8_t locOTP_au8[5];
    uint8_t locWriteData_au8[2];
    uint8_t locReadData_au8[3];
    uint16_t locOut_u16;

    NULL_CHECK_PARAM(locICPPress_p);

    locOTP_au8[0] = 0xC5;
    locOTP_au8[1] = 0x95;
    locOTP_au8[2] = 0x00;
    locOTP_au8[3] = 0x66;
    locOTP_au8[4] = 0x9C;
    
    /* Read OTP begins */
    if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_TRANSMIT, ICP_I2C_ADDRESS, locOTP_au8, 5, NULL))
    {
        return ICP_COMM_ERROR;
    }
    
    locICPPress_p->delayHandle(10);
    
    locWriteData_au8[0] = 0xC7;
    locWriteData_au8[1] = 0xF7;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_TRANSMIT, ICP_I2C_ADDRESS, locWriteData_au8, 2, (uint8_t *)&restart_i2c))
        {
            return ICP_COMM_ERROR;
        }

        if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_RECEIVE, ICP_I2C_ADDRESS, locReadData_au8, 3, NULL))
        {
            return ICP_COMM_ERROR;
        }

        locICPPress_p->delayHandle(1);
        
        locOut_u16 = (locReadData_au8[0] << 8) | locReadData_au8[1];
        locICPPress_p->sensorConstants[i] = (float)locOut_u16;
    }
    /* Read OTP ends */

    locICPPress_p->pPaCalib[0] = 45000.0;
    locICPPress_p->pPaCalib[1] = 80000.0;
    locICPPress_p->pPaCalib[2] = 105000.0;
    locICPPress_p->LUT_lower = 3.5 * (1 << 20);
    locICPPress_p->LUT_upper = 11.5 * (1 << 20);
    locICPPress_p->quadrFactor = 1.0 / 16777216.0;
    locICPPress_p->offsetFactor = 2048.0;

    gUpdateCheck_u8 = 1;

    return ICP_OK;
}


void ICPPress_SoftReset(ICPPRess_Def_t *locICPPress_p)
{
    uint8_t locWriteData_au8[2] = {0x80, 0x5D};

    NULL_CHECK_PARAM(locICPPress_p);

    locICPPress_p->commHandle(I2C_EVENT_TRANSMIT, ICP_I2C_ADDRESS, locWriteData_au8, 2, NULL);

    gUpdateCheck_u8 = 0;
}


ICPPress_State_t ICPPress_ReadID(ICPPRess_Def_t *locICPPress_p, uint16_t *locDeviceID_u16)
{
    uint8_t locWriteData_au8[2] = {0xEF, 0xC8};
    uint8_t locReadData_au8[3];

    NULL_CHECK_PARAM(locICPPress_p);

    if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_TRANSMIT, ICP_I2C_ADDRESS, locWriteData_au8, 2, NULL))
    {
        return ICP_COMM_ERROR;
    }

    if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_RECEIVE, ICP_I2C_ADDRESS, locReadData_au8, 3, NULL))
    {
        return ICP_COMM_ERROR;
    }

    *locDeviceID_u16 = (locReadData_au8[0] << 8) | locReadData_au8[1];

    return ICP_OK;
}


ICPPress_State_t ICPPress_SetMeasurementMode(ICPPRess_Def_t *locICPPress_p, uint16_t locModeCmd_u16)
{
    ICPPress_State_t locRet = ICP_OK;
    uint8_t locWriteData_au8[2];

    NULL_CHECK_PARAM(locICPPress_p);

    if ((ICP_CMD_MEASURE_LP_T_FIRST == locModeCmd_u16) || (ICP_CMD_MEASURE_N_T_FIRST == locModeCmd_u16) || (ICP_CMD_MEASURE_LN_T_FIRST == locModeCmd_u16) || (ICP_CMD_MEASURE_ULN_T_FIRST == locModeCmd_u16))
    {
        locRet = ICP_TEMPERATURE_FIRST;
    }
    else if ((ICP_CMD_MEASURE_LP_P_FIRST == locModeCmd_u16) || (ICP_CMD_MEASURE_N_P_FIRST == locModeCmd_u16) || (ICP_CMD_MEASURE_LN_P_FIRST == locModeCmd_u16) || (ICP_CMD_MEASURE_ULN_P_FIRST == locModeCmd_u16))
    {
        locRet = ICP_PRESSURE_FIRST;
    }
    else 
    {
        locRet = ICP_INVALID_MODE;
        return locRet;
    }

    locICPPress_p->sensorMeasurementMode = locModeCmd_u16;
    locICPPress_p->sensorDataOutMode = locRet;
    locWriteData_au8[0] = locICPPress_p->sensorMeasurementMode >> 8;
    locWriteData_au8[1] = locICPPress_p->sensorMeasurementMode;

    if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_TRANSMIT, ICP_I2C_ADDRESS, locWriteData_au8, 2, NULL))
    {
        locRet = ICP_COMM_ERROR;
    }

    return locRet;
}


ICPPress_State_t ICPPress_ReadRawData(ICPPRess_Def_t *locICPPress_p, int16_t *locRawTemperature_p16, uint32_t *locRawPressure_p32)
{
    /* Temperature data is transmitted in two 8-bit words and pressure data is transmitted in four 8-bit words. 
     * Regarding the pressure data, only the first three words MMSB, MLSB and LMSB contain information about the 
     * ADC pressure value p_dout. Therefore, for retrieving the ADC pressure value, LLSB must be disregarded
     * p_dout = MMSB « 16 | MLSB « 8| LMSB.
     * Two bytes of data are always followed by one byte CRC checksum.
     */
    uint8_t locADCData_au8[9];
    uint8_t locTemperatureIndex_u8;
    uint8_t locPressureIndex_u8;
    uint8_t locWriteData_au8[2];

    NULL_CHECK_PARAM(locICPPress_p);

    locWriteData_au8[0] = locICPPress_p->sensorMeasurementMode >> 8;
    locWriteData_au8[1] = locICPPress_p->sensorMeasurementMode;

    if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_TRANSMIT, ICP_I2C_ADDRESS, locWriteData_au8, 2, NULL))
    {
        return ICP_COMM_ERROR;
    }

    locICPPress_p->delayHandle(6);

    if (ICP_OK != locICPPress_p->commHandle(I2C_EVENT_RECEIVE, ICP_I2C_ADDRESS, locADCData_au8, 9, NULL))
    {
        return ICP_COMM_ERROR;
    }

    if (ICP_TEMPERATURE_FIRST == locICPPress_p->sensorDataOutMode)
    {
        locTemperatureIndex_u8 = 0;
        locPressureIndex_u8 = 3;
    }
    else if (ICP_PRESSURE_FIRST == locICPPress_p->sensorDataOutMode)
    {
        locPressureIndex_u8 = 0;
        locTemperatureIndex_u8 = 6;
    }
    else
    {
        return ICP_INVALID_MODE;
    }

    *locRawTemperature_p16 = (locADCData_au8[locTemperatureIndex_u8] << 8) | locADCData_au8[locTemperatureIndex_u8 + 1];
    *locRawPressure_p32 = (locADCData_au8[locPressureIndex_u8] << 16) | (locADCData_au8[locPressureIndex_u8 + 1] << 8) | locADCData_au8[locPressureIndex_u8 + 3];

    return ICP_OK;
}


ICPPress_State_t ICPPress_GetProcessedData(ICPPRess_Def_t *locICPPress_p, float *locTemperature_pf, float *locPressure_pf, float *locAltitude_pf)
{
    int16_t locTemperature_i16;
    uint32_t locPressure_u32;

    float t;
    float s1, s2, s3;
    float res;
    float in[3];
    float out[3];

    ICPPress_State_t locRet;

    NULL_CHECK_PARAM(locICPPress_p);

    if (0 == gUpdateCheck_u8)
    {
        ICPPress_Init(locICPPress_p);
    }

    locRet = ICPPress_ReadRawData(locICPPress_p, &locTemperature_i16, &locPressure_u32);

    if (ICP_OK != locRet)
    {
        return locRet;
    }

    t = (float)(locTemperature_i16 - 32768);
    s1 = locICPPress_p->LUT_lower + (float)(locICPPress_p->sensorConstants[0] * t * t) * locICPPress_p->quadrFactor;
    s2 = (locICPPress_p->offsetFactor * locICPPress_p->sensorConstants[3]) + (float)(locICPPress_p->sensorConstants[1] * t * t) * locICPPress_p->quadrFactor;
    s3 = locICPPress_p->LUT_upper + (float)(locICPPress_p->sensorConstants[2] * t * t) * locICPPress_p->quadrFactor;
    
    in[0] = s1;
    in[1] = s2;
    in[2] = s3;

    calculate_conversion_constants(locICPPress_p->pPaCalib, in, out);

    *locTemperature_pf = -45.0 + 175.0 / 65536.0 * (float)locTemperature_i16;

    *locPressure_pf = (out[0] + out[1] / (out[2] + (float)locPressure_u32)) / 100.0;

    res = *locPressure_pf / 1013.96;
    res = pow(res, 0.19022);
    res = 1.0 - res;
    *locAltitude_pf = res * ((*locTemperature_pf + 273.15) / 0.0065);

    locRet = ICP_OK;
    return locRet;
}


static void calculate_conversion_constants(float *p_Pa, float *p_LUT, float *out)
{
    float A, B, C;

    C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) + p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) + p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
        (p_LUT[2] * (p_Pa[0] - p_Pa[1]) + p_LUT[0] * (p_Pa[1] - p_Pa[2]) + p_LUT[1] * (p_Pa[2] - p_Pa[0]));

    A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);

    B = (p_Pa[0] - A) * (p_LUT[0] + C);

    out[0] = A;
    out[1] = B;
    out[2] = C;
}

