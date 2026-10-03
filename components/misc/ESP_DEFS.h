#pragma once
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define FUN __FUNCTION__
#define FUNC_ADDRESS esp_cpu_get_call_addr((intptr_t)__builtin_return_address(0))
#define CHECK_(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x)
#define ERROR_STR(rc) "err 0x%02x at 0x%08x", rc, FUNC_ADDRESS
#define CHECK_RET(x) ESP_RETURN_ON_ERROR(x,"",ERROR_STR(err_rc_))
#define CHECK_VOID(x) ESP_RETURN_VOID_ON_ERROR(x,"",ERROR_STR(err_rc_))
#define CHECK_GOTO(x, goto_tag) ESP_GOTO_ON_ERROR(x, goto_tag ,"", ERROR_STR(err_rc_))
#define uS() esp_timer_get_time()
#define delayUntil(prev, tmr) vTaskDelayUntil((prev),pdMS_TO_TICKS(tmr))
#define SEC (1000000ULL)
#define ENTER_CRITICAL() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux);
#define EXIT_CRITICAL() portEXIT_CRITICAL(&mux);
#define CONCAT(l, r) l##r

#if CONFIG_ARDUINO_ISR_IRAM
#define ARDUINO_ISR_ATTR IRAM_ATTR
#define ARDUINO_ISR_FLAG ESP_INTR_FLAG_IRAM
#else
#define ARDUINO_ISR_ATTR
#define ARDUINO_ISR_FLAG (0)
#endif
#define INPUT				0x01
#define OUTPUT				0x03
#define PULLUP				0x04
#define INPUT_PULLUP		0x05
#define PULLDOWN			0x08
#define INPUT_PULLDOWN		0x09
#define OPEN_DRAIN			0x10
#define OUTPUT_OPEN_DRAIN	0x13
#define ANALOG				0xC0