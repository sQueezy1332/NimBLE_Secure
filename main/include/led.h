/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

/* Includes */
/* ESP APIs */


#include "sdkconfig.h"
#include <stdint.h>
/* Defines */
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define LUAT_OS
#ifdef __cplusplus
extern "C" {
#endif
/* Public function declarations */
uint8_t get_led_state(void);
void led_on(void);
void led_off(void);
void led_init(void);
#ifdef __cplusplus
}
#endif

