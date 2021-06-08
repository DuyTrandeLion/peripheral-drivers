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

#include "tof.h"

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
static uint32_t last_measure_count;
static uint32_t measure_count;
static uint32_t measure_distance;
static uint32_t measure_conf;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_ID);


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


int32_t write_i2c_block(uint32_t slave_addr, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    ret_code_t err_code;
    uint8_t write_buffer[129];

    write_buffer[0] = reg;
    memcpy(&write_buffer[1], buf, len);

#if TWI_USE_DMA            
    if (false == m_xfer_done)
    {
        return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;
#endif
    err_code = nrf_drv_twi_tx(&m_twi, slave_addr, write_buffer, len + 1, false);
    APP_ERROR_CHECK(err_code);
#if TWI_USE_DMA            
    while (false == m_xfer_done) { };
#endif

    return err_code;
}


int32_t read_i2c_block(uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint32_t len)
{
    ret_code_t err_code;
    uint8_t write_data;

    write_data = reg;
    
#if TWI_USE_DMA            
    if (false == m_xfer_done)
    {
        return NRF_ERROR_TIMEOUT;
    }
    m_xfer_done = false;
#endif
    err_code = nrf_drv_twi_tx(&m_twi, slave_addr, &write_data, 1, true);
    APP_ERROR_CHECK(err_code);
#if TWI_USE_DMA            
    while (false == m_xfer_done) { };
    m_xfer_done = false;
#endif
    err_code = nrf_drv_twi_rx(&m_twi, slave_addr, buf, len);
    APP_ERROR_CHECK(err_code);
#if TWI_USE_DMA            
    while (false == m_xfer_done) { };
#endif

    return err_code;
}


void read_gpio(uint32_t gpio, uint32_t *value)
{
    *value = nrf_gpio_pin_read(gpio);
}


void write_gpio(uint32_t gpio, uint32_t value)
{
    nrf_gpio_pin_write(gpio, value);
}


void usleep(uint32_t time_us)
{
    nrf_delay_us(time_us);
}


void msleep(uint32_t time_ms)
{
    nrf_delay_ms(time_ms);
}


uint32_t get_system_second(void)
{
    return m_timer_tick_s;
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
       .scl                = ARDUINO_A5_PIN,
       .sda                = ARDUINO_A4_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
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
    
    NRF_LOG_INFO("TWI sensor example started.\r\n");
    tof_init(m_calibration_trigger);
    
    while (true)
    {
        if (((uint32_t)m_timer_tick_ms - m_previous_tick_ms) >= 500)
        {
            tof_measure(&measure_count, &measure_distance, &measure_conf);
            if (last_measure_count != measure_count)
            {
                NRF_LOG_INFO("Result no.: %4u\t Distance: %6u mm\t Confidence: %4u\r\n", measure_count, measure_distance, measure_conf);
                last_measure_count = measure_count;
            }

            m_previous_tick_ms = m_timer_tick_ms;
        }

        NRF_LOG_FLUSH();
    }
}

/** @} */
