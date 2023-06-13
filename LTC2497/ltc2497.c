#include "ltc2497.h"

#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#define NULL_CHECK_PARAM(LTC2497_Def)   if ((NULL == LTC2497_Def) || (NULL == LTC2497_Def->i2cHandle) || (NULL == LTC2497_Def->delayHandle)) { return LTC2497_NULL_PARAM; }


static uint16_t gTempRandomPWM_u16 = 0;



LTC2497_State_t LTC2497_Init(LTC2497_Def_t *locLTC2497_p)
{
    uint8_t locData_u8;
    LTC2497_State_t locRet;

    NULL_CHECK_PARAM(locLTC2497_p);

    locData_u8 = 0x00;

    locRet = locLTC2497_p->i2cHandle(LTC2497_I2C_EVENT_TRANSMIT, locLTC2497_p->deviceAddress, 0x00, &locData_u8, 1, NULL);
    locLTC2497_p->delayHandle(200);

    return LTC2497_OK;
}


LTC2497_State_t LTC2497_ReadChannelADC(LTC2497_Def_t *locLTC2497_p, LTC2497_Channel_Config_Def_t locChannelConfig_s, int8_t locChannelIndex_u8, int32_t *locChannelADC_pi32)
{
    uint8_t locChannelConfigData_u8;
    uint8_t locChannelADC_au8[3];
    LTC2497_State_t locRet;

    NULL_CHECK_PARAM(locLTC2497_p);

    if (locChannelIndex_u8 > 15)
    {
        return LTC2497_INVALID_PARAM;
    }

    locChannelConfigData_u8 = 0x80;
    locChannelConfigData_u8 |= locChannelIndex_u8;
    if (CHANNEL_ENABLED == locChannelConfig_s.channelEnabled)
    {
        locChannelConfigData_u8 |= locChannelConfig_s.channelEnabled;
    }
    if (DIFFERENTIAL_INPUT_ENABLED == locChannelConfig_s.differentialChannelEnabled)
    {
        locChannelConfigData_u8 |= locChannelConfig_s.differentialChannelEnabled;
    }
    if (ODD_INPUT_ENABLED == locChannelConfig_s.oddInputEnabled)
    {
        locChannelConfigData_u8 |= locChannelConfig_s.oddInputEnabled;
    }

    locRet = locLTC2497_p->i2cHandle(LTC2497_I2C_EVENT_RECEIVE, locLTC2497_p->deviceAddress, locChannelConfigData_u8, locChannelADC_au8, 3, NULL);
    if (LTC2497_OK != locRet)
    {
        return locRet;
    }

    /* Byte 0 = bit 23 - 16  */
    /* Byte 1 = bit 15 - 8   */
    /* Byte 2 = bit 7 - 0    */
    *locChannelADC_pi32 = 0x00;
    *locChannelADC_pi32 |= locChannelADC_au8[2];
    *locChannelADC_pi32 |= locChannelADC_au8[1] << 8;
    *locChannelADC_pi32 |= locChannelADC_au8[0] << 16;
    
    locLTC2497_p->channel[locChannelIndex_u8].rawADCValue = *locChannelADC_pi32;

    return locRet;
}


LTC2497_State_t LTC2497_ConvertRawADC2Voltage(LTC2497_Def_t *locLTC2497_p, uint32_t locRawADC_i32, float *locVoltage_fp)
{
    LTC2497_State_t locRet;
    uint32_t locTemp_i32;

    NULL_CHECK_PARAM(locLTC2497_p);

    locTemp_i32 = (locRawADC_i32 & 0x00FFFFFF) >> 6;


    if ((0x20000 <= locTemp_i32) && (locTemp_i32 <= 0x30000))
    {
        *locVoltage_fp = 0.5 * locLTC2497_p->refVoltage * ((float)(locTemp_i32 & 0x1FFFF) / 65535.0);
    }
    else if ((0x10000 <= locTemp_i32) && (locTemp_i32 <= 0x1FFFF))
    {
        *locVoltage_fp = (-1.0) * (0.5 * locLTC2497_p->refVoltage * ((65536.0 - (float)locTemp_i32) / 65535.0));
    }

    return LTC2497_OK;
}


#ifdef __cplusplus
}
#endif
