#include "pca9685.h"

#define NULL_CHECK_PARAM(PCA9685_Def)   if ((NULL == PCA9685_Def) || (NULL == PCA9685_Def->i2cHandle) || (NULL == PCA9685_Def->delayHandle)) { return PCA9685_NULL_PARAM; }


static uint16_t gTempRandomPWM_u16 = 0;


PCA9685_State_t PCA9685_Init(PCA9685_Def_t *locPCA9685_p)
{
    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);

    locPCA9685_p->mode1Register = 0x00;
    locPCA9685_p->mode2Register = 0x00;
    
    if (0x01 == locPCA9685_p->enableAllCall)
    {
        locPCA9685_p->mode1Register |= (0x01 << 0);             /* PCA9685 responds to LED All Call I2C-bus address */
    }
    else
    {
        locPCA9685_p->mode1Register &= ~(0x01 << 0);             /* PCA9685 responds to LED All Call I2C-bus address */
    }
    locPCA9685_p->mode1Register |= (0x01 << 1);             /* PCA9685 responds to I2C-bus subaddress 3 */
    locPCA9685_p->mode1Register |= (0x01 << 2);             /* PCA9685 responds to I2C-bus subaddress 2 */
    locPCA9685_p->mode1Register |= (0x01 << 3);             /* PCA9685 responds to I2C-bus subaddress 1 */
    locPCA9685_p->mode1Register &= ~(0x01 << 4);            /* Normal mode */
    locPCA9685_p->mode1Register |= (0x01 << 5);             /* Enable register auto-increment */
    if (0x01 == locPCA9685_p->enableExternalClock)
    {
        locPCA9685_p->mode1Register |= (0x01 << 6);         /* Enable external clock source */
    }
    else
    {
        locPCA9685_p->mode1Register &= ~(0x01 << 6);
    }
    locPCA9685_p->mode1Register |= (0x01 << 7);             /* Enable restart */

    locPCA9685_p->delayHandle(1);

    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, REGISTER_MODE1, &(locPCA9685_p->mode1Register), 1, NULL);

    locPCA9685_p->mode2Register |= (0x01 << 2);             /* The 16 LEDn outputs are configured with a totem pole structure */

    locRet |= locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, REGISTER_MODE2, &(locPCA9685_p->mode2Register), 1, NULL);

    return locRet;
}


PCA9685_State_t PCA9685_Sleep(PCA9685_Def_t *locPCA9685_p)
{
    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);
    
    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, REGISTER_MODE1, &(locPCA9685_p->mode1Register), 1, NULL);

    /* A random PWM should be read out then writen back again to wake the PWM controller up. */
    locRet |= locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, REGISTER_LED0_ON_L, (uint8_t *)&gTempRandomPWM_u16, 2, NULL);
    if (PCA9685_OK != locRet)
    {
        return locRet;
    }

    locPCA9685_p->mode1Register |= (0x01 << 4);             /* Sleep mode */

    locRet |= locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, REGISTER_MODE1, &(locPCA9685_p->mode1Register), 1, NULL);

    return locRet;
}


PCA9685_State_t PCA9685_Wake(PCA9685_Def_t *locPCA9685_p)
{
    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);

    /* Write logic 1 to bit 7 of MODE1 register. All PWM channels will restart and the
     * RESTART bit will clear.*/
    locPCA9685_p->mode1Register &= ~(0x01 << 4);            /* Normal mode */
    locPCA9685_p->mode1Register |= (0x01 << 7);             /* Enable restart */

    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, REGISTER_MODE1, &(locPCA9685_p->mode1Register), 1, NULL);

    /* A random PWM should be read out then writen back again to wake the PWM controller up. */
    locPCA9685_p->delayHandle(1);
    locRet |= locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, REGISTER_LED0_ON_L, (uint8_t *)&gTempRandomPWM_u16, 2, NULL);

    return locRet;
}


PCA9685_State_t PCA9685_SoftwareReset(PCA9685_Def_t *locPCA9685_p)
{
    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);

    /* To restart all of the previously active PWM channels with a few I2C-bus cycles do the following steps:
          1. Read MODE1 register.
          2. Check that bit 7 (RESTART) is a logic 1. If it is, clear bit 4 (SLEEP). Allow time for oscillator to stabilize (500 us).
          3. Write logic 1 to bit 7 of MODE1 register. All PWM channels will restart and the RESTART bit will clear.

          Remark: The SLEEP bit must be logic 0 for at least 500 us, before a logic 1 is written into the RESTART bit.

        Other actions that will clear the RESTART bit are:
          1. Power cycle.
          2. I2C Software Reset command.
          3. If the MODE2 OCH bit is logic 0, write to any PWM register then issue an I2C-bus STOP.
          4. If the MODE2 OCH bit is logic 1, write to all four PWM registers in any PWM channel.
    */

    /* 1. Read MODE1 register. */
    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, REGISTER_MODE1, &(locPCA9685_p->mode1Register), 1, NULL);
    if (PCA9685_OK != locRet)
    {
        return locRet;
    }
    
    /* 2. Check that bit 7 (RESTART) is a logic 1. If it is, clear bit 4 (SLEEP). Allow time for
     * oscillator to stabilize (500 us). */
    if (0x00 != (locPCA9685_p->mode1Register & (0x01 << 7)))
    {
        locRet |= PCA9685_Sleep(locPCA9685_p);
        locPCA9685_p->delayHandle(1);

        locRet |= PCA9685_Wake(locPCA9685_p);
    }

    return locRet;
}


PCA9685_State_t PCA9685_ReadSubAddress(PCA9685_Def_t *locPCA9685_p, uint8_t locSubAddressIndex_u8, uint8_t *locSubAddress_p8)
{
    NULL_CHECK_PARAM(locPCA9685_p);

    if ((locSubAddressIndex_u8 < SUBADDRESS1) || (locSubAddressIndex_u8 > SUBADDRESS3))
    {
        return PCA9685_INVALID_PARAM;
    }

    return locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, SUBADDRESS1 + locSubAddressIndex_u8 - 1, locSubAddress_p8, 1, NULL);
}


PCA9685_State_t PCA9685_WriteSubAddress(PCA9685_Def_t *locPCA9685_p, uint8_t locSubAddressIndex_u8, uint8_t locSubAddress_u8)
{
    uint8_t locWritenSubAddress_u8;
    uint8_t locReadbackSubAddress_u8;
    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);

    if ((locSubAddressIndex_u8 < SUBADDRESS1) || (locSubAddressIndex_u8 > SUBADDRESS3))
    {
        return PCA9685_INVALID_PARAM;
    }

    locWritenSubAddress_u8 = locSubAddress_u8 & 0xFE;

    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, SUBADDRESS1 + locSubAddressIndex_u8 - 1, &locWritenSubAddress_u8, 1, NULL);
    if (PCA9685_OK != locRet)
    {
        return locRet;
    }

    locRet |= PCA9685_ReadSubAddress(locPCA9685_p, locSubAddressIndex_u8, &locReadbackSubAddress_u8);
    if (locReadbackSubAddress_u8 != locWritenSubAddress_u8)
    {
        return PCA9685_COMM_ERROR;
    }

    return locRet;
}


PCA9685_State_t PCA9685_ReadOutputPrescale(PCA9685_Def_t *locPCA9685_p, uint8_t *locPrescale_p8)
{
    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);

    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, REGISTER_PRE_SCALE, locPrescale_p8, 1, NULL);
    
    if (PCA9685_OK == locRet)
    {
        locPCA9685_p->prescale = *locPrescale_p8;
    }

    return locRet;
}


PCA9685_State_t PCA9685_SetOutputPrescale(PCA9685_Def_t *locPCA9685_p, uint8_t locPrescale_u8)
{
    uint8_t locReadbackOutputPrescale_u8;

    PCA9685_State_t locRet;

    NULL_CHECK_PARAM(locPCA9685_p);
    
    /* Read MODE1 register. */
    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, REGISTER_MODE1, &(locPCA9685_p->mode1Register), 1, NULL);
    if (PCA9685_OK != locRet)
    {
        return locRet;
    }

    /* prescale value = round(osc_clock / (4096 * update_rate)) - 1 */

    /* The maximum PWM frequency is 1526 Hz if the PRE_SCALE register is set "0x03h".
       The minimum PWM frequency is 24 Hz if the PRE_SCALE register is set "0xFFh".
       The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1. */
    locRet = PCA9685_Sleep(locPCA9685_p);
    locPCA9685_p->delayHandle(1);

    locRet |= locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, REGISTER_PRE_SCALE, &locPrescale_u8, 1, NULL);
    locRet |= PCA9685_ReadOutputPrescale(locPCA9685_p, &locReadbackOutputPrescale_u8);
    
    if (PCA9685_OK == locRet)
    {
        if (locReadbackOutputPrescale_u8 == locPrescale_u8)
        {
            locPCA9685_p->prescale = locPrescale_u8;
        }
        else
        {
            return PCA9685_COMM_ERROR;
        }
    }
    else
    {
        return locRet;
    }

    locRet |= PCA9685_Wake(locPCA9685_p);

    return locRet;
}


PCA9685_State_t PCA9685_ReadLEDActiveTime(PCA9685_Def_t *locPCA9685_p, uint8_t locLEDIndex_u8, uint16_t *locActiveTime_p16)
{
    uint8_t locLEDRegisterOnLow_u8;

    NULL_CHECK_PARAM(locPCA9685_p);
    if (locLEDIndex_u8 > 15)
    {
        return PCA9685_INVALID_ADDRESS;
    }

    locLEDRegisterOnLow_u8 = REGISTER_LED0_ON_L + locLEDIndex_u8 * 4;

    return locPCA9685_p->i2cHandle(I2C_EVENT_RECEIVE, locPCA9685_p->deviceAddress, locLEDRegisterOnLow_u8, (uint8_t *)locActiveTime_p16, 2, NULL);
}


PCA9685_State_t PCA9685_SetLEDActiveTime(PCA9685_Def_t *locPCA9685_p, uint8_t locLEDIndex_u8, uint16_t locActiveTime_u16)
{
    PCA9685_State_t locRet;
    uint8_t locLEDRegisterOnLow_u8;
    uint8_t locLEDRegisterOffLow_u8;
    uint16_t locOnTime_u16 = locActiveTime_u16;
    uint16_t locOffTime_u16 = 4095 - locActiveTime_u16;
    uint16_t locReadBackActiveTime_u16;

    NULL_CHECK_PARAM(locPCA9685_p);
    if ((locLEDIndex_u8 > 15) && (PCA9685_ALL_LED != locLEDIndex_u8))
    {
        return PCA9685_INVALID_ADDRESS;
    }

    if (locLEDIndex_u8 < 16)
    {
        locLEDRegisterOnLow_u8 = REGISTER_LED0_ON_L + locLEDIndex_u8 * 4;
        locLEDRegisterOffLow_u8 = REGISTER_LED0_OFF_L + locLEDIndex_u8 * 4;
    }
    else if (PCA9685_ALL_LED == locLEDIndex_u8)
    {
        locLEDRegisterOnLow_u8 = REGISTER_ALL_LED_ON;
        locLEDRegisterOffLow_u8 = REGISTER_ALL_LED_OFF;
    }

    locRet = locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, locLEDRegisterOnLow_u8, (uint8_t *)&locOnTime_u16, 2, NULL);
    locRet |= locPCA9685_p->i2cHandle(I2C_EVENT_TRANSMIT, locPCA9685_p->deviceAddress, locLEDRegisterOffLow_u8, (uint8_t *)&locOffTime_u16, 2, NULL);

    if (locLEDIndex_u8 < 16)
    {
        locRet |= PCA9685_ReadLEDActiveTime(locPCA9685_p, locLEDIndex_u8, &locReadBackActiveTime_u16);

        if (locReadBackActiveTime_u16 != locActiveTime_u16)
        {
            return PCA9685_COMM_ERROR;
        }
    }

    return locRet;
}

