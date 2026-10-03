/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once
//#include "sdkconfig.h"
//#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public function declarations */
bool get_led_state(void);
void led_set(unsigned long BRG = 0xFFFFFF, int timeout_ms = 10);
void led_off(void);
void led_init(void);
#ifdef __cplusplus
}
#endif