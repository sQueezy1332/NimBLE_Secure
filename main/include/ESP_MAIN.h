#pragma once
//#define CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT 1
//#define DEBUG_ENABLE
//#define NO_GLOBAL_INSTANCES
#define NO_GLOBAL_SERIAL
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "soc/rtc.h"
#include "nvs_flash.h"
#include "driver/gptimer.h"
#include "esp_ota_ops.h"
#include "hal/gpio_hal.h"
#if defined CONFIG_AUTOSTART_ARDUINO
#pragma message "CONFIG_AUTOSTART_ARDUINO"
#endif
#if defined(CONFIG_BT_ENABLED) && SOC_BT_SUPPORTED
#include "esp_bt.h" 
#if CONFIG_IDF_TARGET_ESP32
__weak_symbol bool  btInUse() __weak_symbol { return false; } //overwritten in esp32-hal-bt.c
#else
/*extern */__weak_symbol bool btInUse() { return true; }
#endif
#endif

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED) //CONFIG_USJ_ENABLE_USB_SERIAL_JTAG=y
#define ARDUINO_USB_CDC_ON_BOOT 1
#define ARDUINO_USB_MODE 1
#endif

#if (ARDUINO_USB_CDC_ON_BOOT | ARDUINO_USB_MSC_ON_BOOT | ARDUINO_USB_DFU_ON_BOOT) && !ARDUINO_USB_MODE
#include "USB.h"
#if ARDUINO_USB_MSC_ON_BOOT
#include "FirmwareMSC.h"
#endif
#endif
#include "chip-debug-report.h" 

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
#define DEBUG_ENABLE
#endif
#ifdef DEBUG_ENABLE
#pragma message "DEBUG_ENABLE"
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE //Serial used from Native_USB_CDC | HW_CDC_JTAG        
HWCDC HWCDCSerial; // Hardware CDC mode
#define Serial HWCDCSerial // Arduino Serial is the HW JTAG CDC device
#elif ARDUINO_USB_MODE// !ARDUINO_USB_MODE -- Native USB Mode
USBCDC USBSerial(0); // Arduino Serial is the Native USB CDC device
#define Serial USBSerial
#else   // !ARDUINO_USB_CDC_ON_BOOT -- Serial is used from UART0
extern HardwareSerial Serial0;
#define Serial Serial0
#endif  // ARDUINO_USB_CDC_ON_BOOT
#define SerialBegin(x)  Serial.begin(x)
#define DEBUG(x, ...) Serial.print(x, ##__VA_ARGS__)
#define DEBUGLN(x, ...) Serial.println(x, ##__VA_ARGS__)
#define DEBUGF(x, ...) Serial.printf(x , ##__VA_ARGS__)
#define CHECK_(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x);
#define CHECK_RET(x) do {esp_err_t ret = (x);\
        if (unlikely(ret != ESP_OK)) { log_e(" 0x%X\t(%s)", ret,  esp_err_to_name(ret)); return;} }while(0)//ESP_RETURN_VOID_ON_ERROR(%s)", err, 
#else
#define DEBUG(x)
#define DEBUGLN(x) 
#define DEBUGF(x, ...)
#define Serial
#define SerialBegin(x)
#define NDEBUG
#define CHECK_(x) (void)(x);
#define CHECK_RET(x) do {esp_err_t ret = (x); if (unlikely(ret != ESP_OK)) {return;} }while(0)
#endif // DEBUG_ENABLE

#define uS esp_timer_get_time()
#define delayms(x) vTaskDelay((x) / portTICK_PERIOD_MS)
#define delayUntil(prev, tmr) vTaskDelayUntil((prev),pdMS_TO_TICKS(tmr))
#define SEC (1000000)
#define ENTER_CRITICAL() {portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux)
#define EXIT_CRITICAL() portEXIT_CRITICAL(&mux);}
#define timer_restart(x) gptimer_set_raw_count(x, 0)
typedef const char cch; typedef const uint8_t cbyte; typedef uint32_t u32; typedef uint64_t u64;

void nvs_init() {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
		if (partition != NULL) {
			err = esp_partition_erase_range(partition, 0, partition->size);
			if (err != ESP_OK) { err = nvs_flash_init(); log_d("nvs_flash_init"); }
			else log_e("Failed to format the broken NVS partition!");
		}
		else log_e("Could not find NVS partition");
	}
	if (err) log_e("Failed to initialize NVS! Error: %u", err);
}

#ifdef CONFIG_APP_ROLLBACK_ENABLE
esp_ota_img_states_t img_state(bool valid = true) {
	const esp_partition_t* running = esp_ota_get_running_partition();
	esp_ota_img_states_t ota_state;
	esp_ota_get_state_partition(running, &ota_state);
	if (valid && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
		esp_ota_mark_app_valid_cancel_rollback();
	}
	return ota_state;
}
//__weak_symbol bool verifyRollbackLater() { return true; }
#endif 
void main_init() {
	//init proper ref tick value for PLL (uncomment if REF_TICK is different than 1MHz)
//ESP_REG(APB_CTRL_PLL_TICK_CONF_REG) = APB_CLK_FREQ / REF_CLK_FREQ - 1;
#ifdef F_XTAL_MHZ
#if !CONFIG_IDF_TARGET_ESP32S2  // ESP32-S2 does not support rtc_clk_xtal_freq_update
	rtc_clk_xtal_freq_update((rtc_xtal_freq_t)F_XTAL_MHZ);
	rtc_clk_cpu_freq_set_xtal();
#endif
#endif
#ifdef F_CPU
	//setCpuFrequencyMhz(F_CPU / 1000000);
#endif
	SerialBegin(115200); DEBUGLN("Serial begin");
#if ARDUINO_USB_MSC_ON_BOOT && !ARDUINO_USB_MODE
	MSC_Update.begin();
#endif
#if ARDUINO_USB_DFU_ON_BOOT && !ARDUINO_USB_MODE
	USB.enableDFU();
#endif
#if ARDUINO_USB_ON_BOOT && !ARDUINO_USB_MODE
	USB.begin();
#endif
#if CONFIG_SPIRAM_SUPPORT || CONFIG_SPIRAM
#ifndef CONFIG_SPIRAM_BOOT_INIT
	psramAddToHeap();
#endif
#endif
	nvs_init();
	//esp_log_level_set("*", CONFIG_LOG_DEFAULT_LEVEL);
#if defined(CONFIG_BT_ENABLED) && SOC_BT_SUPPORTED
	if (!btInUse()) {
		__unused esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BTDM); log_d("%i", ret);
	}
#endif
#if defined CONFIG_APP_ROLLBACK_ENABLE || CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
	//if (!verifyRollbackLater()) { log_i("app_valid"); esp_ota_mark_app_valid_cancel_rollback(); }
#endif
}

esp_err_t timer_alarm(uint64_t value, gptimer_handle_t& handle, bool reload = 0, uint64_t count = 0) {
		gptimer_alarm_config_t alarm_config{
			.alarm_count = value,
			.reload_count = count,
			.flags = {.auto_reload_on_alarm = reload} //.flags.auto_reload_on_alarm = reload,
		};
		return gptimer_set_alarm_action(handle, &alarm_config);
	}
esp_err_t timer_init(uint64_t value, gptimer_handle_t& handle, gptimer_alarm_cb_t func, bool start = 1, bool reload = 0) {
		esp_err_t ret = ESP_OK;
		gptimer_config_t config{
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000000,
			.intr_priority = 0,
			.flags {/* .intr_shared = 0, .allow_pd = 0,.backup_before_sleep = 0 */},
			//.flags = {.intr_shared = 1, },
		}; //SOC_TIMER_GROUP_TOTAL_TIMERS
		gptimer_event_callbacks_t cbs = { .on_alarm = func };
		if ((ret = gptimer_new_timer(&config, &handle))
			|| (ret = gptimer_register_event_callbacks(handle, &cbs, NULL))
			|| (ret = gptimer_enable(handle))
			|| (ret = timer_alarm(value, handle, reload)))
			goto exit;
		if (start) ret = gptimer_start(handle);
	exit://log_v("ret = %i", ret);
		return ret;
	}

 void pinMode(uint8_t pin, uint8_t mode) {
	if (mode >= 32) { log_d("pinMode(%u, %u)", pin, mode); return; }
	gpio_config_t conf = {
		.pin_bit_mask = (1ULL << pin),						/*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
		.mode = (gpio_mode_t)(mode & GPIO_MODE_INPUT_OUTPUT_OD),	/*!<  GPIO mode: set input/output mode */
		.pull_up_en = mode & PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,				/*!< GPIO pull-up */
		.pull_down_en = mode & PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,			/*!< GPIO pull-down */
		.intr_type = (gpio_int_type_t)GPIO_LL_GET_HW(GPIO_PORT_0)->pin[pin].int_type, /*!< GPIO interrupt type - previously set */
	};
	if (gpio_config(&conf) != ESP_OK) log_e("IO %i config failed", pin);
}
 
__attribute__((always_inline)) inline void digitalWrite(uint8_t pin, uint8_t val) {
    if (!digitalPinCanOutput(pin)) return;
    gpio_hal_context_t gpiohal { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; 
	gpio_hal_set_level(&gpiohal, pin, val);
}

__attribute__((always_inline)) inline int digitalRead(uint8_t pin) {
	gpio_hal_context_t gpiohal { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; //return gpio_ll_get_level(&GPIO, (gpio_num_t)pin);
	return gpio_hal_get_level(&gpiohal, (gpio_num_t)pin);
}

void attachInterruptArg(uint8_t pin, voidFuncPtrArg userFunc, void* arg, int intr_type) {
	if (pin >= SOC_GPIO_PIN_COUNT) return;// makes sure that pin -1 (255) will never work -- this follows Arduino standard
	esp_err_t err = gpio_install_isr_service((int)ARDUINO_ISR_FLAG);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		log_e("IO %i ISR Service Failed To Start", pin); return;
	}
	gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0b111));
	if (intr_type & 0b1000) { gpio_wakeup_enable((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0b111)); }
	gpio_isr_handler_add((gpio_num_t)pin, userFunc, arg);
	//gpio_hal_context_t gpiohal{ .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; //FIX interrupts on peripherals outputs (eg. LEDC,...) 
	//gpio_hal_input_enable(&gpiohal, pin); //Enable input in GPIO register
}

void attachInterrupt(uint8_t pin, voidFuncPtr handler, int mode) {
	attachInterruptArg(pin, (voidFuncPtrArg)handler, NULL, mode);
}

void detachInterrupt(uint8_t pin) {
	gpio_intr_disable((gpio_num_t)pin);
	gpio_wakeup_disable((gpio_num_t)pin);
	gpio_isr_handler_remove((gpio_num_t)pin);  //remove handle and disable isr for pin
}
//xTaskCreateUniversal(loopTask, "loopTask", getArduinoLoopTaskStackSize(), NULL, 1, &loopTaskHandle, ARDUINO_RUNNING_CORE);
//#undef CONFIG_AUTOSTART_ARDUINO
#if CONFIG_AUTOSTART_ARDUINO
TaskHandle_t loopTaskHandle = NULL;
#if not defined ARDUINO_LOOP_STACK_SIZE && !defined CONFIG_ARDUINO_LOOP_STACK_SIZE
#define ARDUINO_LOOP_STACK_SIZE 8192
#else
#define ARDUINO_LOOP_STACK_SIZE CONFIG_ARDUINO_LOOP_STACK_SIZE
#endif

#if CONFIG_FREERTOS_UNICORE
void yieldIfNecessary(void) {
	static uint64_t lastYield = 0;
	uint64_t now = millis();
	if ((now - lastYield) > 2000) {
		lastYield = now;
		vTaskDelay(5);  //delay 1 RTOS tick
	}
}
#endif
bool loopTaskWDTEnabled = false;
__weak_symbol extern TaskHandle_t loopTaskHandle = NULL;

__weak_symbol size_t getArduinoLoopTaskStackSize(void) {
	return ARDUINO_LOOP_STACK_SIZE;
}

__weak_symbol bool shouldPrintChipDebugReport(void) {
	return false;
}
//xTaskCreateUniversal(loopTask, "loopTask", getArduinoLoopTaskStackSize(), NULL, 1, &loopTaskHandle, ARDUINO_RUNNING_CORE);
void loopTask(void* pvParameters) {
#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SERIAL)
	// sets UART0 (default console) RX/TX pins as already configured in boot or as defined in variants/pins_arduino.h
	Serial0.setPins(gpioNumberToDigitalPin(SOC_RX0), gpioNumberToDigitalPin(SOC_TX0));
#endif
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
	printBeforeSetupInfo();
#endif
	setup();
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
	printAfterSetupInfo();
#endif
	for (;;) {
#if CONFIG_FREERTOS_UNICORE
		yieldIfNecessary();
#endif
		if (loopTaskWDTEnabled) {
			esp_task_wdt_reset();
		}
		loop();
		if (serialEventRun) {
			serialEventRun();
		}
	}
}

#endif
