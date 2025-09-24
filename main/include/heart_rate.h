/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef HEART_RATE_H
#define HEART_RATE_H

/* Includes */
#include "esp_random.h"
/* Defines */
#define HEART_RATE_TASK_PERIOD (2000 / portTICK_PERIOD_MS)
#ifdef __cplusplus
extern "C" {
#endif
/* Public function declarations */
uint8_t get_heart_rate(void);
void update_heart_rate(void);
#ifdef __cplusplus
}
#endif
#endif // HEART_RATE_H
