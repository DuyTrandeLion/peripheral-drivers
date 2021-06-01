#include "AMG88xx.h"
#include <math.h>
#include <string.h>

#include "grideye_api_lv1.h"


#define AMG88xx_PIXEL_ARRAY_SIZE            64
#define AMG88xx_PIXEL_TEMP_CONVERSION       0.25
#define AMG88xx_THERMISTOR_CONVERSION       0.0625


#define NULL_CHECK_PARAM(AMG88xx_Def)  if ((NULL == AMG88xx_Def) || \
                                           (NULL == AMG88xx_Def->commHandle) || \
                                           (NULL == AMG88xx_Def->delayHandle) || \
                                           ((DEVICE_ADDRESS_EVEN != locAMG88xx_p->deviceAddress) && (DEVICE_ADDRESS_ODD != locAMG88xx_p->deviceAddress))) { return AMG88xx_NULL_PARAM; }


AMG88xx_State_t AMG88xx_Init(AMG88xx_Def_t *locAMG88xx_p)
{
    AMG88xx_State_t locRet;

    NULL_CHECK_PARAM(locAMG88xx_p);

    locRet = AMG88xx_SetResetMode(locAMG88xx_p, INITIAL_RESET);
    if (AMG88xx_OK != locRet)
    {
        return AMG88xx_OK;
    }

    locRet = AMG88xx_SetOperationMode(locAMG88xx_p, NORMAL_MODE);
    if (AMG88xx_OK != locRet)
    {
        return AMG88xx_OK;
    }
    
    locRet = AMG88xx_SetFPSCMode(locAMG88xx_p, FRAME_RATE_1FPS);

    return locRet;
}


AMG88xx_State_t AMG88xx_GetOperationMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locOperationMode_p8)
{
    uint8_t locReadData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_POWER_CTL_REG, &locReadData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    *locOperationMode_p8 = locReadData_u8;

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_SetOperationMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locOperationMode_u8)
{
    uint8_t locWriteData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if ((NORMAL_MODE != locOperationMode_u8) && (SLEEP_MODE != locOperationMode_u8))
    {
        return AMG88xx_INVALID_MODE;
    }

    locWriteData_u8 = locOperationMode_u8;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_POWER_CTL_REG, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_GetResetMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locResetMode_p8)
{
    uint8_t locReadData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_RESET_REG, &locReadData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    *locResetMode_p8 = locReadData_u8;

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_SetResetMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locResetMode_u8)
{
    uint8_t locWriteData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if ((FLAG_RESET != locResetMode_u8) && (INITIAL_RESET != locResetMode_u8))
    {
        return AMG88xx_INVALID_MODE;
    }

    locWriteData_u8 = locResetMode_u8;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_RESET_REG, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_GetFPSCMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locFPSCMode_p8)
{
    uint8_t locReadData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_FPSC_REG, &locReadData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    *locFPSCMode_p8 = locReadData_u8;

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_SetFPSCMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locFPSCMode_u8)
{
    uint8_t locWriteData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if ((FRAME_RATE_1FPS != locFPSCMode_u8) && (FRAME_RATE_10FPS != locFPSCMode_u8))
    {
        return AMG88xx_INVALID_MODE;
    }

    locWriteData_u8 = locFPSCMode_u8;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_FPSC_REG, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_GetInterruptMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locInterruptMode_p8, uint8_t *locInterruptStatus_u8)
{
    uint8_t locReadData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_INT_CTL_REG, &locReadData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    *locInterruptMode_p8 = (locReadData_u8 >> 1) & 0x01;
    *locInterruptStatus_u8 = locReadData_u8 & 0x01;

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_SetInterruptMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locInterruptMode_u8, uint8_t locInterruptStatus_u8)
{
    uint8_t locWriteData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if ((ABS_INTERRUPT_MODE != locInterruptMode_u8) && (DIFF_INTERRUPT_MODE != locInterruptMode_u8))
    {
        return AMG88xx_INVALID_MODE;
    }

    locWriteData_u8 = (locInterruptMode_u8 << 1) | locInterruptStatus_u8;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_INT_CTL_REG, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_SetInterruptLevels(AMG88xx_Def_t *locAMG88xx_p, float locHigh_f, float locLow_f, float locHysteresis_f)
{
    int16_t locHighConv_i16;
    int16_t locLowConv_i16;
    int16_t locHysConv_i16;

    uint8_t locWriteData_au8[2];

    NULL_CHECK_PARAM(locAMG88xx_p);

    locHighConv_i16 = locHigh_f / AMG88xx_PIXEL_TEMP_CONVERSION;

    if ((locHighConv_i16 < -4095) && (locHighConv_i16 > 4095))
    {
        return AMG88xx_INVALID_PARAM;
    }

    locWriteData_au8[0] = locHighConv_i16 & 0xFF;
    locWriteData_au8[1] = (locHighConv_i16 & 0x0F) >> 4;

    if ((AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, AMG88xx_INTHL_REG, &locWriteData_au8[0], 1, NULL)) &&
        (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_INTHH_REG, &locWriteData_au8[1], 1, NULL)))
    {
        return AMG88xx_COMM_ERROR;
    }

    locLowConv_i16 = locLow_f / AMG88xx_PIXEL_TEMP_CONVERSION;

    if ((locLowConv_i16 < -4095) && (locLowConv_i16 > 4095))
    {
        return AMG88xx_INVALID_PARAM;
    }

    locWriteData_au8[0] = locLowConv_i16 & 0xFF;
    locWriteData_au8[1] = (locLowConv_i16 & 0x0F) >> 4;

    if ((AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, AMG88xx_INTLL_REG, &locWriteData_au8[0], 1, NULL)) &&
        (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_INTLH_REG, &locWriteData_au8[1], 1, NULL)))
    {
        return AMG88xx_COMM_ERROR;
    }

    locHysConv_i16 = locHysteresis_f / AMG88xx_PIXEL_TEMP_CONVERSION;

    if ((locHysConv_i16 < -4095) && (locHysConv_i16 > 4095))
    {
        return AMG88xx_INVALID_PARAM;
    }

    locWriteData_au8[0] = locHysConv_i16 & 0xFF;
    locWriteData_au8[1] = (locHysConv_i16 & 0x0F) >> 4;

    if ((AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, AMG88xx_IHYSL_REG, &locWriteData_au8[0], 1, NULL)) &&
        (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_IHYSH_REG, &locWriteData_au8[1], 1, NULL)))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_ReadStatus(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locTemperatureOF_u8, uint8_t *locInterruptOutbreak_u8)
{
    uint8_t locReadData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_STAT_REG, &locReadData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    *locTemperatureOF_u8 = locReadData_u8 & TEMPERATURE_OVERFLOW;
    *locInterruptOutbreak_u8 = locReadData_u8 & INTERRUPT_OUTBREAK;

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_ClearStatus(AMG88xx_Def_t *locAMG88xx_p, uint8_t locStatusFlag_u8)
{
    uint8_t locWriteData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    locWriteData_u8 = locStatusFlag_u8;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, AMG88xx_SCLR_REG, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_SetMovingAveragetMode(AMG88xx_Def_t *locAMG88xx_p, uint8_t locMovingAverageModeEnable_u8)
{
    uint8_t locWriteData_u8;

    NULL_CHECK_PARAM(locAMG88xx_p);

    if ((MOVING_AVERAGE_DISABLE != locMovingAverageModeEnable_u8) && (MOVING_AVERAGE_ENABLE != locMovingAverageModeEnable_u8))
    {
        return AMG88xx_INVALID_PARAM;
    }

    /* Write the magic register and numbers */
    locWriteData_u8 = 0x50;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, 0x1F, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    locWriteData_u8 = 0x45;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, 0x1F, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    locWriteData_u8 = 0x57;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, 0x1F, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    locWriteData_u8 = locMovingAverageModeEnable_u8 << 5;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT_REPEATED_START, locAMG88xx_p->deviceAddress, AMG88xx_AVE_REG, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    locWriteData_u8 = 0x00;

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_TRANSMIT, locAMG88xx_p->deviceAddress, 0x1F, &locWriteData_u8, 1, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_ReadThermistor(AMG88xx_Def_t *locAMG88xx_p, uint8_t *locTemperature_p8, float *locTemperature_pf)
{
    int16_t locTemperature_i16 = 0;
    uint8_t locReadData_au8[2];

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_TTHL_REG, locReadData_au8, 2, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }
    
    memcpy(locTemperature_p8, locReadData_au8, sizeof(int16_t));

    if (NULL != locTemperature_pf)
    {
        locTemperature_i16 = locReadData_au8[0] | (locReadData_au8[1] << 8);
    
        /* Mask out all the bits that are not needed. */
        locTemperature_i16 &= 0x07FF;

        /* 12th bit is the sign bit */
        locTemperature_i16 = (locTemperature_i16 & 0x800)? (0 - locTemperature_i16) : locTemperature_i16;        

        *locTemperature_pf = (float)locTemperature_i16 * AMG88xx_THERMISTOR_CONVERSION;
    }

    return AMG88xx_OK;
}


AMG88xx_State_t AMG88xx_ReadPixelsTemperature(AMG88xx_Def_t *locAMG88xx_p, int16_t *locPixelTemperature_p16, float *locPixelTemperature_pf)
{
    uint8_t locReadData_au8[AMG88xx_PIXEL_ARRAY_SIZE * 2];
    int16_t locPixelTemperature_ai16[AMG88xx_PIXEL_ARRAY_SIZE] = {0};

    NULL_CHECK_PARAM(locAMG88xx_p);

    if (AMG88xx_OK != locAMG88xx_p->commHandle(I2C_EVENT_RECEIVE, locAMG88xx_p->deviceAddress, AMG88xx_PIXEL_BASE, locReadData_au8, AMG88xx_PIXEL_ARRAY_SIZE * 2, NULL))
    {
        return AMG88xx_COMM_ERROR;
    }

    for (uint8_t idx = 0; idx < AMG88xx_PIXEL_ARRAY_SIZE; idx++)
    {
        locPixelTemperature_ai16[idx] = locReadData_au8[idx * 2] | (locReadData_au8[(idx * 2) + 1] << 8);
        locPixelTemperature_ai16[idx] &= 0x0FFF;

        /* Shift to left so that sign bit of 12 bit integer number is
         * placed on sign bit of 16 bit signed integer number.
         * Then shift back the signed number */
        locPixelTemperature_ai16[idx] = locPixelTemperature_ai16[idx] << 4;
        locPixelTemperature_ai16[idx] = locPixelTemperature_ai16[idx] >> 4;

        locPixelTemperature_p16[idx] = locPixelTemperature_ai16[idx];

        if (NULL != locPixelTemperature_pf)
        {
            locPixelTemperature_pf[idx] = (float)locPixelTemperature_p16[idx] * AMG88xx_PIXEL_TEMP_CONVERSION;
        }
    }

    //memcpy(locPixelTemperature_p16, locPixelTemperature_ai16, AMG88xx_PIXEL_ARRAY_SIZE * sizeof(uint16_t));

    return AMG88xx_OK;
}