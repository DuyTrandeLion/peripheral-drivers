/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_systick.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define TIMER_INSTANCE_ID   1

#define GPIO_TOF_EN_PIN     ARDUINO_10_PIN

#define TIMER_PERIOD_MS     1

#define TWI_USE_DMA         1

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = true;

static uint8_t m_calibration_trigger = false;
static volatile uint32_t m_previous_tick_ms;
static volatile uint32_t m_timer_tick_ms;
static volatile uint32_t m_timer_tick_s;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Timer instance */
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_ID);

static VL53L0X_Dev_t m_vl53l0x =
{
  .I2cDevAddr = 0x29
};

static VL53L0X_RangingMeasurementData_t m_measurement_data;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
        {
            m_xfer_done = true;
            break;
        }

        default:
        {
            break;
        }
    }
}


void VL53L0X_OsDelay(uint32_t delay_time)
{
    if (0 == delay_time)
    {
        return;
    }

    nrf_delay_ms(delay_time);
}


int _I2CWrite(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count)
{
    ret_code_t err_code;

#if TWI_USE_DMA
    if (false == m_xfer_done)
    {
        return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;
#endif
    err_code = nrf_drv_twi_tx(&m_twi, Dev->I2cDevAddr, pdata, count, false);
    APP_ERROR_CHECK(err_code);
#if TWI_USE_DMA
    while (false == m_xfer_done) { };
#endif

    return err_code;
}


int _I2CRead(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count)
{
    ret_code_t err_code;

#if TWI_USE_DMA
    if (false == m_xfer_done)
    {
        return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;
#endif
    err_code = nrf_drv_twi_rx(&m_twi, Dev->I2cDevAddr, pdata, count);
    APP_ERROR_CHECK(err_code);
#if TWI_USE_DMA
    while (false == m_xfer_done) { };
#endif

    return err_code;
}


/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        {
            m_timer_tick_ms++;

            if (0 == (m_timer_tick_ms % 1000))
            {
                m_timer_tick_s++;
            }
            break;
        }

        default:
        {
            //Do nothing.
            break;
        }
    }
}


/**
 * @brief System tick initialization.
 */
static void systick_init(void)
{
    nrfx_systick_init();
}


/**
 * @brief GPIO initialization.
 */
static void gpio_init(void)
{
    nrf_gpio_cfg_output(GPIO_TOF_EN_PIN);
}


/**
 * @brief TWI initialization.
 */
static void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_amg8833_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

#if TWI_USE_DMA
    err_code = nrf_drv_twi_init(&m_twi, &twi_amg8833_config, twi_handler, NULL);
#else
    err_code = nrf_drv_twi_init(&m_twi, &twi_amg8833_config, NULL, NULL);
#endif

    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief TWI initialization.
 */
static void timer_init(void)
{
    ret_code_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_timer, TIMER_PERIOD_MS),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

    nrf_drv_timer_enable(&m_timer);
}


static void vl53l0x_init(void)
{
    int32_t err_code;
    uint8_t is_aperture_spads;
    uint8_t vhv_settings;
    uint8_t phase_cal;
    uint8_t pre_range_vcsel_period = 18;
    uint8_t final_range_vcsel_period = 14;
    uint16_t id;
    uint32_t ref_spad_count;
    uint32_t timing_budget = 33000;
    FixPoint1616_t signal_limit = (FixPoint1616_t)(0.1 * 65536);
    FixPoint1616_t sigma_limit = (FixPoint1616_t)(60 * 65536);

    /* Try to read one register using module 0x52 address */
    err_code = VL53L0X_RdWord(&m_vl53l0x, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (err_code)
    {
        NRF_LOG_INFO("Read id failed, err_code = %d\r\n", err_code);
        return;
    }

    if (0xeeaa == id)
    {
        ///* Sensor is found => Change its I2C address to disired address */
        //err_code = VL53L0X_SetDeviceAddress(&m_vl53l0x, 0x52 * 2);
        //if (err_code)
        //{
        //    NRF_LOG_INFO("VL53L0X_SetDeviceAddress failed, err_code = %d\r\n", err_code);
        //    return;
        //}

        //m_vl53l0x.I2cDevAddr = 0x52;

        ///* Check all is OK with the new I2C address and initialize the sensor */
        //err_code = VL53L0X_RdWord(&m_vl53l0x, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
        //if (err_code)
        //{
        //    NRF_LOG_INFO("Read id failed, err_code = %d\r\n", err_code);
        //    return;
        //}

        err_code = VL53L0X_DataInit(&m_vl53l0x);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_DataInit failed, err_code = %d\r\n", err_code);
            return;
        }

        err_code = VL53L0X_StaticInit(&m_vl53l0x);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_StaticInit failed, err_code = %d\r\n", err_code);
            return;
        }

        err_code = VL53L0X_PerformRefSpadManagement(&m_vl53l0x, &ref_spad_count, &is_aperture_spads);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_PerformRefSpadManagement failed, err_code = %d\r\n", err_code);
            return;
        }

        err_code = VL53L0X_PerformRefCalibration(&m_vl53l0x, &vhv_settings, &phase_cal);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_PerformRefCalibration failed, err_code = %d\r\n", err_code);
            return;
        }

        /* Setup in single ranging mode */
        err_code = VL53L0X_SetDeviceMode(&m_vl53l0x, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        if (err_code)
        {
           NRF_LOG_INFO("VL53L0X_SetDeviceMode failed, err_code = %d\r\n", err_code);
        }

        /* Enable Sigma limit.
           Sigma is the time difference (shift) between the reference and return SPAD arrays. As
           Sigma represents time of flight and this translates to distance, this parameter is expressed
           in mm. */
        err_code = VL53L0X_SetLimitCheckEnable(&m_vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        if (err_code)
        {
           NRF_LOG_INFO("VL53L0X_SetLimitCheckEnable failed, err_code = %d\r\n", err_code);
        }

        err_code = VL53L0X_SetLimitCheckValue(&m_vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigma_limit);
        if (err_code)
        {
           NRF_LOG_INFO("VL53L0X_SetLimitCheckValue failed, err_code = %d\r\n", err_code);
        }

        /* Enable Sigma limit.
           Return signal rate measurement, expressed in MCPS. This represents the amplitude of the
           signal reflected from the target and detected by the device. */
        err_code = VL53L0X_SetLimitCheckEnable(&m_vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        if (err_code)
        {
           NRF_LOG_INFO("VL53L0X_SetLimitCheckEnable failed, err_code = %d\r\n", err_code);
        }

        err_code = VL53L0X_SetLimitCheckValue(&m_vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signal_limit);
        if (err_code)
        {
           NRF_LOG_INFO("VL53L0X_SetLimitCheckValue failed, err_code = %d\r\n", err_code);
        }

        err_code = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_vl53l0x, timing_budget);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed, err_code = %d\r\n", err_code);
            return;
        }

        err_code = VL53L0X_SetVcselPulsePeriod(&m_vl53l0x, VL53L0X_VCSEL_PERIOD_PRE_RANGE, pre_range_vcsel_period);
        if (err_code)
        {
           NRF_LOG_INFO("VL53L0X_SetVcselPulsePeriod failed, err_code = %d\r\n", err_code);
        }

        err_code = VL53L0X_SetVcselPulsePeriod(&m_vl53l0x, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, final_range_vcsel_period);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_SetVcselPulsePeriod failed, err_code = %d\r\n", err_code);
            return;
        }

        err_code = VL53L0X_PerformRefCalibration(&m_vl53l0x, &vhv_settings, &phase_cal);
        if (err_code)
        {
            NRF_LOG_INFO("VL53L0X_PerformRefCalibration failed, err_code = %d\r\n", err_code);
            return;
        }
    }
    else
    {
        NRF_LOG_INFO("Unknown id, id = %d\r\n", id);
    }
}


static void vl53l0x_measure(void)
{
    int32_t err_code;
    
     err_code = VL53L0X_PerformSingleRangingMeasurement(&m_vl53l0x, &m_measurement_data);
     if (err_code)
     {
          NRF_LOG_INFO("VL53L0X_PerformSingleRangingMeasurement failed, err_code = %d\r\n", err_code);
     }
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    systick_init();
    timer_init();
    gpio_init();
    twi_init();

    vl53l0x_init();
    NRF_LOG_INFO("TWI sensor example started.\r\n");

    while (true)
    {
        if (((uint32_t)m_timer_tick_ms - m_previous_tick_ms) >= 500)
        {
            vl53l0x_measure();
            m_previous_tick_ms = m_timer_tick_ms;
        }

        NRF_LOG_FLUSH();
    }
}

/** @} */
