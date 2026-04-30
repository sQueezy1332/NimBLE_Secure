#include "hal/gpio_hal.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_mac.h"
#include "esp_log.h"

#if CONFIG_ARDUINO_ISR_IRAM
#define ARDUINO_ISR_ATTR IRAM_ATTR
#define ARDUINO_ISR_FLAG ESP_INTR_FLAG_IRAM
#else
#define ARDUINO_ISR_ATTR
#define ARDUINO_ISR_FLAG (0)
#endif
#define INPUT 0x01
#define OUTPUT            0x03
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x13
#define ANALOG            0xC0

#ifdef __cplusplus
extern "C" {
#endif
//unsigned long ARDUINO_ISR_ATTR micros() { return esp_timer_get_time();}
//unsigned long ARDUINO_ISR_ATTR millis() { return esp_timer_get_time() / 1000; }
//void delay(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }
//void delayMicroseconds(uint32_t us) { }

void pinMode(uint8_t pin, uint8_t mode) {
	gpio_config_t conf = {
		.pin_bit_mask = (1ULL << pin),
		.mode = (gpio_mode_t)(mode & GPIO_MODE_INPUT_OUTPUT_OD),
		.pull_up_en = mode & PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
		.pull_down_en = mode & PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
		.intr_type = (gpio_int_type_t)GPIO_LL_GET_HW(GPIO_PORT_0)->pin[pin].int_type,
	};
	gpio_config(&conf); //error logs internal
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
void digitalWrite(uint8_t pin, uint8_t val) {
		if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) return;
		const gpio_hal_context_t gpiohal = { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) };
		gpio_hal_set_level(&gpiohal, pin, val);
}
#pragma GCC diagnostic pop
int digitalRead(uint8_t pin) {
	const gpio_hal_context_t gpiohal = { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) };
	return gpio_hal_get_level(&gpiohal, (gpio_num_t)pin);
}


void digitalToggle(uint8_t pin) { digitalWrite(pin, !digitalRead(pin)); }

void attachInterruptArg(uint8_t pin, void(*userFunc )(void*), void* arg, int intr_type) {
	if (pin >= SOC_GPIO_PIN_COUNT) return;// makes sure that pin -1 (255) will never work -- this follows Arduino standard
	__unused esp_err_t err = gpio_install_isr_service(ARDUINO_ISR_FLAG);
	gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0b111));
	if (intr_type & 0b1000) { gpio_wakeup_enable((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0b111)); }
	gpio_isr_handler_add((gpio_num_t)pin, userFunc, arg);
	gpio_hal_context_t gpiohal = { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; //FIX interrupts on peripherals outputs (eg. LEDC,...)
	gpio_hal_input_enable(&gpiohal, pin); //Enable input in GPIO register
}

void attachInterrupt(uint8_t pin, void(*handler)() , int mode) {
	attachInterruptArg(pin, (void(*)(void*))handler, NULL, mode);
}

void detachInterrupt(uint8_t pin) {
	gpio_intr_disable((gpio_num_t)pin);
	gpio_wakeup_disable((gpio_num_t)pin);
	gpio_isr_handler_remove((gpio_num_t)pin);  //remove handle and disable isr for pin
}

void enableInterrupt(uint8_t pin) { gpio_intr_enable((gpio_num_t)pin); }
void disableInterrupt(uint8_t pin) { gpio_intr_disable((gpio_num_t)pin); }

#ifdef __cplusplus
}
#endif


void nvs_init() {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
		if (partition) {
			err = esp_partition_erase_range(partition, 0, partition->size);
			if (err == ESP_OK) { err = nvs_flash_init(); ESP_LOGI("NVS", "nvs_flash_init()"); }
			else {ESP_LOGE("NVS", "!format ");}
		} else { ESP_LOGE("NVS", "!finding"); }
	}
	if (err) ESP_LOGE("NVS", "NVS: %d", err);
}

#ifdef CONFIG_APP_ROLLBACK_ENABLE
esp_ota_img_states_t img_state(bool valid) {
	const esp_partition_t* cur_part = esp_ota_get_running_partition();
	esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
	esp_err_t ret = esp_ota_get_state_partition(cur_part, &ota_state);
	if(ret) { ESP_LOGE("ota", "get_state 0x%x", ret); }
 	ESP_EARLY_LOGI("ota", "%s state: %lu%s", cur_part->label, ota_state, 
		ota_state == ESP_OTA_IMG_PENDING_VERIFY ? " PENDING_VERIFY" : "");
	if(valid && (ota_state == ESP_OTA_IMG_PENDING_VERIFY)) { 
		ret = esp_ota_mark_app_valid_cancel_rollback(); 
		ESP_EARLY_LOGI("ota", "app_valid 0x%x", ret);
	}
	return ota_state;
}

#endif

esp_timer_handle_t
esp_timer_new(esp_timer_cb_t cb, esp_timer_dispatch_t type, bool skip, void* arg, const char* name) {
		esp_timer_handle_t handle;
		esp_timer_create_args_t config = {
			.callback = cb,
			.arg = arg,
			.dispatch_method = type,
			.name = name,
			.skip_unhandled_events = skip,
		};
		if(esp_timer_create(&config, &handle)) return NULL;
		return handle;
}

esp_err_t esp_timer_start(esp_timer_handle_t handle, uint64_t period) {
	if (esp_timer_is_active(handle)) return esp_timer_restart(handle, period);
	return esp_timer_start_once(handle, period);
}

esp_err_t
gptimer_alarm(gptimer_handle_t handle, uint64_t value, bool reload, uint64_t count) {
	gptimer_alarm_config_t alarm_config  = {
			.alarm_count = value,
			.reload_count = count,
			.flags = {.auto_reload_on_alarm = reload} //.flags.auto_reload_on_alarm = reload,
	};
	return gptimer_set_alarm_action(handle, &alarm_config);
}

gptimer_handle_t
gptimer_init(uint64_t value, gptimer_alarm_cb_t func, bool reload, uint8_t priority) {
		gptimer_handle_t handle; esp_err_t ret;
		gptimer_config_t config = {
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000000,
			.intr_priority = priority,
			.flags = { .intr_shared = 1, .allow_pd = 0,//.backup_before_sleep = 0 
			},
		}; //SOC_TIMER_GROUP_TOTAL_TIMERS //internal logs
		gptimer_event_callbacks_t cbs = { .on_alarm = func };
		if((ret = gptimer_new_timer(&config, &handle))) { return NULL; }
		if((ret = gptimer_register_event_callbacks(handle, &cbs, NULL))) { } 
		else if((ret = gptimer_enable(handle))) { }
		else if((ret = gptimer_alarm(handle, value, reload, 0))) { }
		return handle;
}

esp_err_t gptimer_restart(gptimer_handle_t handle) { return gptimer_set_raw_count(handle, 0); }

uint64_t gptimer_read(gptimer_handle_t handle) {
	uint64_t value;
	__unused esp_err_t ret = gptimer_get_raw_count(handle, &value);
	if(ret) { return 0; }
	return value;
}


uint64_t getEfuseMac() { 
	uint64_t mac = 0LL; 
	esp_efuse_mac_get_default((uint8_t *)(&mac)); 
	return mac; 
}
