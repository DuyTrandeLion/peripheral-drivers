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
#include "nrf_drv_twi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_systick.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "pca9685.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define TIMER_INSTANCE_ID   1

#define TIMER_PERIOD_MS     1

#define TWMI_USE_DMA        1

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = true;

static uint32_t m_previous_tick = 0;
static uint32_t m_timer_tick = 0;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_ID);

static PCA9685_Def_t m_pca9685;

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


/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            m_timer_tick++;
            break;

        default:
            //Do nothing.
            break;
    }
}


static PCA9685_I2C_Handle_t pca9685_comm_handle(PCA9685_I2C_Event_t locCommEvent_en, uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t data_size, void *p_context)
{
    ret_code_t err_code;


    switch (locCommEvent_en)
    {
        case I2C_EVENT_TRANSMIT:
        {
            uint8_t write_buffer[129];

            write_buffer[0] = register_address;
            memcpy(&write_buffer[1], data, data_size);

#if TWMI_USE_DMA            
            if (false == m_xfer_done)
            {
                return (PCA9685_I2C_Handle_t)PCA9685_BUSY;
            }
            m_xfer_done = false;
#endif
            err_code = nrf_drv_twi_tx(&m_twi, device_address, write_buffer, data_size + 1, false);
            APP_ERROR_CHECK(err_code);
#if TWMI_USE_DMA            
            while (false == m_xfer_done) { };
#endif
            break;
        }

        case I2C_EVENT_RECEIVE:
        {
            uint8_t write_data;

            write_data = register_address;
#if TWMI_USE_DMA            
            if (false == m_xfer_done)
            {
                return (PCA9685_I2C_Handle_t)PCA9685_BUSY;
            }
            m_xfer_done = false;
#endif
            err_code = nrf_drv_twi_tx(&m_twi, device_address, &write_data, 1, true);
            APP_ERROR_CHECK(err_code);
#if TWMI_USE_DMA            
            while (false == m_xfer_done) { };
            m_xfer_done = false;
#endif
            err_code = nrf_drv_twi_rx(&m_twi, device_address, data, data_size);
            APP_ERROR_CHECK(err_code);
#if TWMI_USE_DMA            
            while (false == m_xfer_done) { };
#endif
            break;
        }

        default: break;
    }

    return (PCA9685_I2C_Handle_t)err_code;
}


static PCA9685_Delay_Handle_t pca9685_delay_handle(uint32_t locDelayPeriodMS_u32)
{
    nrf_delay_ms(locDelayPeriodMS_u32);
}


/**
 * @brief System tick initialization.
 */
static void systick_init(void)
{
    nrfx_systick_init();
}


/**
 * @brief TWI initialization.
 */
static void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_pca9685_config =
    {
       .scl                = ARDUINO_A5_PIN,
       .sda                = ARDUINO_A4_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

#if TWMI_USE_DMA            
    err_code = nrf_drv_twi_init(&m_twi, &twi_pca9685_config, twi_handler, NULL);
#else
    err_code = nrf_drv_twi_init(&m_twi, &twi_pca9685_config, NULL, NULL);
#endif
    
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief TIMER initialization.
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


static void pca9685_init(void)
{
    nrf_gpio_cfg_output(ARDUINO_10_PIN);
    nrf_gpio_pin_clear(ARDUINO_10_PIN);

    m_pca9685.deviceAddress = 0x42;
    m_pca9685.enableAllCall = 0;
    m_pca9685.enableExternalClock = 0;
    m_pca9685.i2cHandle = (PCA9685_I2C_Handle_t)pca9685_comm_handle;
    m_pca9685.delayHandle = (PCA9685_Delay_Handle_t)pca9685_delay_handle;
    
    /* Testing sleep/wake/reset functions */
    //APP_ERROR_CHECK(PCA9685_SoftwareReset(&m_pca9685));

    //APP_ERROR_CHECK(PCA9685_Sleep(&m_pca9685));
    //nrf_delay_ms(5000);
    //APP_ERROR_CHECK(PCA9685_Wake(&m_pca9685));

    APP_ERROR_CHECK(PCA9685_Init(&m_pca9685));
    APP_ERROR_CHECK(PCA9685_SetOutputPrescale(&m_pca9685, 15));
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint16_t light_value = 0;
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    systick_init();
    timer_init();
    twi_init();
    pca9685_init();

    NRF_LOG_INFO("nRF52 + PCA9685 example started.\r\n");
    NRF_LOG_FLUSH();
    
    while (true)
    {
        if (((uint32_t)m_timer_tick - m_previous_tick) >= 50)
        {
            light_value = (light_value + 35) % 4095;
            APP_ERROR_CHECK(PCA9685_SetLEDActiveTime(&m_pca9685, PCA9685_ALL_LED, light_value));

            m_previous_tick = m_timer_tick;
        }

        NRF_LOG_FLUSH();
    }
}

/** @} */
