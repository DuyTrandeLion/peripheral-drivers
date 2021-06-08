#ifndef _MCU_TMF8801_CONFIG_H_
#define _MCU_TMF8801_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

// configured options for tmf8801 mcu driver

// Include generated source for built-in firmware option
#ifdef _HAS_BUILTIN_FW
#include "tof_bin_image.h"
#endif

// defines configured by CMake
#define MCU_TMF8801_PROJ_NAME "@PROJECT_NAME@"
#define MCU_TMF8801_VERSION_MAJOR @MCU_TMF8801_VERSION_MAJOR@
#define MCU_TMF8801_VERSION_MINOR @MCU_TMF8801_VERSION_MINOR@


void usleep(uint32_t time_us);

void msleep(uint32_t time_ms);

uint32_t get_system_second(void);

void read_gpio(uint32_t gpio, uint32_t *value);

void write_gpio(uint32_t gpio, uint32_t value);

int32_t write_i2c_block(uint32_t slave_addr, uint8_t reg, const uint8_t *buf, uint32_t len);

int32_t read_i2c_block(uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint32_t len);

#endif /* _MCU_TMF8801_CONFIG_H_ */