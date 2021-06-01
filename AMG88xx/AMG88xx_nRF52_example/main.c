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

#include "AMG88xx.h"
#include "AMG88xx_conf.h"
#include "grideye_api_lv1.h"
#include "grideye_api_lv2.h"
#include "grideye_api_lv3.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define TIMER_INSTANCE_ID   1

#define TIMER_PERIOD_MS     1

#define TWMI_USE_DMA        1

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = true;

static uint8_t m_amg8833_temperature_status;
static uint8_t m_amg8833_interrupt_status;
static uint8_t m_amg8833_thermsistor[2];
static int16_t m_amg8833_raw_pixel_temperatures[64];
static volatile uint32_t m_previous_tick;
static volatile uint32_t m_timer_tick;
static float m_amg8833_pixel_temperatures[64];

static float m_amg8833_thermsistor_temperature;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_ID);

static AMG88xx_Def_t m_amg8833;


static AMG88xx_State_t amg88xx_comm_handle(AMG88xx_Event_t amg88xx_event, uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t data_size, void *p_context)
{
    ret_code_t err_code;

    switch (amg88xx_event)
    {
        case I2C_EVENT_TRANSMIT:
        {
            uint8_t write_buffer[129];
            
            write_buffer[0] = register_address;
            memcpy(&write_buffer[1], data, data_size);

#if TWMI_USE_DMA            
            if (false == m_xfer_done)
            {
                return AMG88xx_BUSY;
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
            uint8_t read_buffer[128];

            write_data = register_address;
            
#if TWMI_USE_DMA            
            if (false == m_xfer_done)
            {
                return AMG88xx_BUSY;
            }
            m_xfer_done = false;
#endif
            err_code = nrf_drv_twi_tx(&m_twi, device_address, &write_data, 1, true);
            APP_ERROR_CHECK(err_code);
#if TWMI_USE_DMA            
            while (false == m_xfer_done) { };
            m_xfer_done = false;
#endif
            err_code = nrf_drv_twi_rx(&m_twi, device_address, read_buffer, data_size);
            APP_ERROR_CHECK(err_code);
#if TWMI_USE_DMA            
            while (false == m_xfer_done) { };
#endif
            memcpy(data, read_buffer, data_size);
            break;
        }

        case I2C_EVENT_TRANSMIT_REPEATED_START:
        {
            uint8_t write_buffer[129];
            
            write_buffer[0] = register_address;
            memcpy(&write_buffer[1], data, data_size);
            
#if TWMI_USE_DMA            
            if (false == m_xfer_done)
            {
                return AMG88xx_BUSY;
            }
            m_xfer_done = false;
#endif
            err_code = nrf_drv_twi_tx(&m_twi, device_address, write_buffer, data_size + 1, true);
            APP_ERROR_CHECK(err_code);
#if TWMI_USE_DMA            
            while (false == m_xfer_done) { };
#endif
            break;
        }

        default:
        {
            break;
        }
    }

    return err_code;
}


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

    const nrf_drv_twi_config_t twi_amg8833_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

#if TWMI_USE_DMA            
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


static void amg8833_init(void)
{
    AMG88xx_State_t amg8833_state;

    m_amg8833.commHandle = amg88xx_comm_handle;
    m_amg8833.delayHandle = nrf_delay_ms;

    m_amg8833.deviceAddress = DEVICE_ADDRESS_EVEN;

    amg8833_state = AMG88xx_Init(&m_amg8833);
    NRF_LOG_INFO("AMG8833 initialized state: %d\r\n", amg8833_state);
    
    amg8833_state = AMG88xx_SetFPSCMode(&m_amg8833, FRAME_RATE_1FPS);
    NRF_LOG_INFO("AMG8833 set frame rate state: %d\r\n", amg8833_state);

    amg8833_state = AMG88xx_SetMovingAveragetMode(&m_amg8833, MOVING_AVERAGE_ENABLE);
    NRF_LOG_INFO("AMG8833 set moving average state: %d\r\n", amg8833_state);
}


BOOL GE_I2CRead(UCHAR ucI2cAddr, UCHAR ucRegAddr, UCHAR *ucDstAddr, UCHAR ucSize)
{
    return !amg88xx_comm_handle(I2C_EVENT_RECEIVE, ucI2cAddr, ucRegAddr, ucDstAddr, ucSize, NULL);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    AMG88xx_State_t amg8833_state;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    systick_init();
    timer_init();
    twi_init();
    amg8833_init();

    NRF_LOG_INFO("TWI sensor example started.\r\n");
    NRF_LOG_FLUSH();

    while (true)
    {
        if (((uint32_t)m_timer_tick - m_previous_tick) >= 100)
        {
            amg8833_state = AMG88xx_ReadStatus(&m_amg8833, &m_amg8833_temperature_status, &m_amg8833_interrupt_status);
            APP_ERROR_CHECK(amg8833_state);

            amg8833_state = AMG88xx_ReadThermistor(&m_amg8833, m_amg8833_thermsistor, &m_amg8833_thermsistor_temperature);
            APP_ERROR_CHECK(amg8833_state);

            amg8833_state = AMG88xx_ReadPixelsTemperature(&m_amg8833, m_amg8833_raw_pixel_temperatures, m_amg8833_pixel_temperatures);
            APP_ERROR_CHECK(amg8833_state);
            
            m_previous_tick = m_timer_tick;
        }

        NRF_LOG_FLUSH();
    }
}

/** @} */
