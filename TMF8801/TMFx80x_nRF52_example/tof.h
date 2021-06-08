#ifndef _TOF_H_
#define _TOF_H_

#include "mcu_tmf8801_config.h"
#include "tmf8801.h"
#include "tof_hex_interpreter.h"

#include "pca10056.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define GPIO_TOF_EN_PIN     ARDUINO_10_PIN

void tof_init(uint8_t calib_signal);
void tof_measure(uint32_t *num, uint32_t *distance, uint32_t *conf);

#endif /* _TOF_H_ */
