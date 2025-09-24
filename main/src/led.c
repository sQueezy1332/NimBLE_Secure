/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "led.h"
#include "common.h"

#ifdef LUAT_OS
#define PIN_LED 13
#define PIN_LED_D4 12
#else
#define PIN_LED CONFIG_BLINK_GPIO
#endif
/* Private variables */
static uint8_t led_state;

/* Public functions */
uint8_t get_led_state(void) { return led_state; }


#if defined CONFIG_IDF_TARGET_ESP32S3 && defined CONFIG_BLINK_LED_STRIP
#include "led_strip.h"
static led_strip_handle_t led_strip;
/* #endif

#ifdef CONFIG_BLINK_LED_STRIP */

void led_on(void) {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, 16, 16, 16);
    led_strip_refresh(led_strip);
    led_state = true;
}

void led_off(void) {
    led_strip_clear(led_strip);
    led_state = false;
}

void led_init(void) {
    ESP_LOGI(TAG, "example configured to blink addressable led!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(
        led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_off();
}

#elif CONFIG_BLINK_LED_GPIO
#include "driver/gpio.h"
void led_on(void) { gpio_set_level(PIN_LED, true); }

void led_off(void) { gpio_set_level(PIN_LED, false); }

void led_init(void) {
    ESP_LOGD(TAG, "example configured to blink gpio led!");
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
#ifdef LUAT_OS
    gpio_reset_pin(PIN_LED_D4);
    gpio_set_direction(PIN_LED_D4, GPIO_MODE_OUTPUT);
#endif
}

#else
#error "unsupported LED type"
#endif
