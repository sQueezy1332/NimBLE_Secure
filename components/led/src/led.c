/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
//#define LUAT_OS
//#include "led.h"
//#include "common.h"
#include "esp_log.h"
//#include "esp_check.h"

#ifdef LUAT_OS
#define PIN_LED 13
#define PIN_LED_D4 12
#define NO_INVERTED 1
#define LED_ON 1
#else
#define NO_INVERTED 0
#endif

#ifdef NO_INVERTED
#define LED_ON_LEVEL 1
#define LED_MODE_SET GPIO_MODE_OUTPUT
#else
#define LED_ON_LEVEL 0
#define LED_MODE_SET GPIO_MODE_OUTPUT_OD
#endif

static const char* TAG = "LED";
/* Private variables */
uint32_t led_state;
/* Public functions */
bool get_led_state(void) { return !!led_state; }

#define CONFIG_BLINK_LED_STRIP

#if defined CONFIG_BLINK_LED_STRIP
#include "driver/rmt_tx.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      8

rmt_channel_handle_t led_chan;
rmt_encoder_handle_t simple_encoder;

void led_set(uint32_t GRB, int timeout_ms) {
	rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
		.flags = {}
    };
	if(timeout_ms) {
		rmt_tx_wait_all_done(led_chan, timeout_ms);
	}	
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(led_chan, simple_encoder, &GRB , 3, &tx_config));
	led_state = GRB;
}

void led_off(void) {
	led_set(0x000000, 10);
}

static const rmt_symbol_word_t ws2812_zero = {
	.duration0 = (int)(0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T0H=0.3us
    .level0 = 1,
    .duration1 = (int)(0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T0L=0.9us
	.level1 = 0,
};

static const rmt_symbol_word_t ws2812_one = {
	.duration0 = (int)(0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T1H=0.9us
    .level0 = 1,
	.duration1 = (int)(0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T1L=0.3us
    .level1 = 0,
};

//reset defaults to 50uS
static const rmt_symbol_word_t ws2812_reset = {
	.duration0 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
    .level0 = 0,
    .duration1 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
	.level1 = 0,
};

static size_t encoder_callback(const void *data, size_t data_size,size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // We need a minimum of 8 symbol spaces to encode a byte. We only
    // need one to encode a reset, but it's simpler to simply demand that
    // there are 8 symbol spaces free to write anything.
    if (symbols_free < 8) {
        return 0;
    }
    // We can calculate where in the data we are from the symbol pos.
    // Alternatively, we could use some counter referenced by the arg
    // parameter to keep track of this.
    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        // Encode a byte
        size_t symbol_pos = 0;
        for (size_t bitmask = 0x80; bitmask; bitmask >>= 1) {
            if (data_bytes[data_pos] & bitmask) {
                symbols[symbol_pos++] = ws2812_one;
            } else {
                symbols[symbol_pos++] = ws2812_zero;
            }
        }
        // We're done; we should have written 8 symbols.
        return symbol_pos;
    } else {
        //All bytes already are encoded. Encode the reset, and we're done.
        symbols[0] = ws2812_reset;
        *done = 1; //Indicate end of the transaction.
        return 1; //we only wrote one symbol
    }
}

static void rmt_init(void)
{
    //ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
		.gpio_num = (gpio_num_t)RMT_LED_STRIP_GPIO_NUM,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
		.intr_priority = 0,
		.flags = {},
	};
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    //ESP_LOGI(TAG, "Create simple callback-based encoder");
    
    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = encoder_callback,
		.arg = NULL,
		.min_chunk_size = 0,
        //Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &simple_encoder));
    //ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}


void led_init(void) {
    ESP_LOGI(TAG, "configured to rmt!");
	rmt_init();
    led_off();
}

#elif CONFIG_BLINK_LED_GPIO
#include "driver/gpio.h"
void led_on(void) { gpio_set_level(PIN_LED, LED_ON_LEVEL); }

void led_off(void) { gpio_set_level(PIN_LED, LED_ON_LEVEL); }

void led_init(void) {
    ESP_LOGI(TAG, "configured gpio!");
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, LED_MODE_SET);
#ifdef LUAT_OS
    gpio_reset_pin(PIN_LED_D4);
    gpio_set_direction(PIN_LED_D4, LED_MODE_SET);
#endif
}

#else
#error "unsupported LED type"
#endif
