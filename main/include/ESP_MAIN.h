#pragma once
#include "sdkconfig.h"
//#define DEBUG_ENABLE
#if !defined(CONFIG_ESP_CONSOLE_NONE) && (defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED) || defined(CONFIG_ESP_CONSOLE_USB_CDC))// && defined //CONFIG_USJ_ENABLE_USB_SERIAL_JTAG
#define ARDUINO_USB_CDC_ON_BOOT 1
#if (CONFIG_ESP_CONSOLE_USB_CDC) && (CONFIG_TINYUSB_CDC_ENABLED)
#define ARDUINO_USB_MODE 0
#else
#define ARDUINO_USB_MODE 1
#endif
#endif 
#include <Arduino.h>
//#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "hal/gpio_hal.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#if defined CONFIG_AUTOSTART_ARDUINO
#pragma message "CONFIG_AUTOSTART_ARDUINO"
#endif
#if (ARDUINO_USB_CDC_ON_BOOT | ARDUINO_USB_MSC_ON_BOOT | ARDUINO_USB_DFU_ON_BOOT) && !ARDUINO_USB_MODE
#include "USB.h"
#if ARDUINO_USB_MSC_ON_BOOT
#include "FirmwareMSC.h"
#endif
#endif

#if defined(CONFIG_BLUEDROID_ENABLED) || defined(CONFIG_NIMBLE_ENABLED)
#if CONFIG_IDF_TARGET_ESP32
__weak_symbol inline bool btInUse() { return false; } //overwritten in esp32-hal-bt.c
#else
extern bool btInUse(); //from esp32-hal-bt.c
#endif
#endif

 #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
#define DEBUG_ENABLE
#endif
#ifdef DEBUG_ENABLE
#include "chip-debug-report.h" 
#pragma message "DEBUG_ENABLE"
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE  //Serial used from Native_USB_CDC | HW_CDC_JTAG
#pragma message "HWCDC"
inline HWCDC HWCDCSerial; 
#elif ARDUINO_USB_CDC_ON_BOOT// !ARDUINO_USB_MODE -- Native USB Mode
#pragma message "USBCDC"
inline USBCDC USBSerial(0)
#else 
#pragma message "UART0" //definiton in HardwareSerial.cpp
#endif  // !ARDUINO_USB_CDC_ON_BOOT -- Serial is used from UART0
#define SerialBegin(x)  Serial.begin(x); //while(!Serial){}
#define DEBUG(x, ...) Serial.print(x, ##__VA_ARGS__)
#define DEBUGLN(x, ...) Serial.println(x, ##__VA_ARGS__)
#define DEBUGF(x, ...) Serial.printf(x , ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#define DEBUGLN(x, ...) 
#define DEBUGF(x, ...)
#define SerialBegin(x)
//do {esp_err_t ret = (x); if (unlikely(ret != ESP_OK)) {return;} }while(0)
#endif // DEBUG_ENABLE
#define CHECK_(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x)
#define CHECK_RET(x) ESP_RETURN_ON_ERROR(x, "","")
#define CHECK_VOID(x) ESP_RETURN_VOID_ON_ERROR(x,"","")
#define uS esp_timer_get_time()
#define delayUntil(prev, tmr) vTaskDelayUntil((prev),pdMS_TO_TICKS(tmr))
#define SEC (1000000ul)
#define ENTER_CRITICAL() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux);
#define EXIT_CRITICAL() portEXIT_CRITICAL(&mux);
typedef const char cch; typedef const uint8_t cbyte; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;

#ifdef CONFIG_APP_ROLLBACK_ENABLE
inline esp_ota_img_states_t img_state(bool valid = false) {
	const esp_partition_t* running = esp_ota_get_running_partition();
	esp_ota_img_states_t ota_state;
	esp_ota_get_state_partition(running, &ota_state);
	if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
		ESP_LOGI("img_state", "IMG_PENDING_VERIFY %s" , valid ? "valid" : "");
		if(valid) esp_ota_mark_app_valid_cancel_rollback();
	}
	return ota_state;
}
__weak_symbol bool verifyRollbackLater() { return true; };
#endif

inline void nvs_init() {
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

inline void main_init() {
	//init proper ref tick value for PLL (uncomment if REF_TICK is different than 1MHz)
//ESP_REG(APB_CTRL_PLL_TICK_CONF_REG) = APB_CLK_FREQ / REF_CLK_FREQ - 1;
#ifdef F_XTAL_MHZ
#include "soc/rtc.h"
#if !CONFIG_IDF_TARGET_ESP32S2  // ESP32-S2 does not support rtc_clk_xtal_freq_update
	rtc_clk_xtal_freq_update((rtc_xtal_freq_t)F_XTAL_MHZ);
	rtc_clk_cpu_freq_set_xtal();
#endif
#endif
#ifdef F_CPU
	//setCpuFrequencyMhz(F_CPU / 1000000);
#endif
	SerialBegin(115200); DEBUGLN("Serial.begin()");
#if ARDUINO_USB_MSC_ON_BOOT && !ARDUINO_USB_MODE
	MSC_Update.begin();
#endif
#if ARDUINO_USB_DFU_ON_BOOT && !ARDUINO_USB_MODE
	USB.enableDFU();
#endif
#if ARDUINO_USB_ON_BOOT && !ARDUINO_USB_MODE
	USB.begin();
#endif
#if (CONFIG_SPIRAM_SUPPORT || CONFIG_SPIRAM) && not defined CONFIG_SPIRAM_BOOT_INIT
  psramAddToHeap();
#endif
	nvs_init();
	//esp_log_level_set("*", CONFIG_LOG_DEFAULT_LEVEL);
#if SOC_BT_SUPPORTED && __has_include("esp_bt.h")  && (defined(CONFIG_BLUEDROID_ENABLED) || defined(CONFIG_NIMBLE_ENABLED))
	//if (!btInUse()) { //"esp_bt.h"
	//	__unused esp_err_t ret = esp_bt_controller_mem_release(BT_MODE_BTDM); log_d("%i", ret);
	//}
#endif
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
	if (!verifyRollbackLater()) { log_i("app_valid"); esp_ota_mark_app_valid_cancel_rollback(); }
#endif
}

class nvsApi {
private: nvs_handle_t _handle = 0;
public:
	nvsApi() {};
	nvsApi(const nvsApi &obj) = delete;
	nvsApi& operator=(const nvsApi&) = delete;
	nvsApi(nvsApi &obj) : _handle(obj._handle) { obj._handle = 0; ESP_LOGD("NVS", "ctor copy"); };
	nvsApi(nvsApi &&obj) : _handle(obj._handle) { obj._handle = 0; ESP_LOGD("NVS", "ctor move"); };
	nvsApi(const char* space, nvs_open_mode_t mode) { begin(space, mode); };
	esp_err_t begin(const char* space, nvs_open_mode_t mode) {
		if(_handle) return 0xDADADA;
		esp_err_t ret = nvs_open(space, mode, &_handle);
		ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
		return ret;
	};
	void close() { if(_handle) { nvs_close(_handle); } };
	~nvsApi() { close(); ESP_LOGV("NVS", "~_handle = %lu", _handle); };
	operator nvs_handle_t() const { return _handle; };
};



inline esp_timer_handle_t 
esp_timer_init( esp_timer_cb_t cb, esp_timer_dispatch_t type = ESP_TIMER_TASK, bool skip = false, void* arg = NULL, const char* name = NULL) {
    esp_timer_handle_t handle; esp_err_t ret;
    esp_timer_create_args_t config = {
      .callback = cb,
      .arg = arg,
      .dispatch_method = type,
      .name = name,
      .skip_unhandled_events = skip,
    };
    ret = esp_timer_create(&config, &handle);
    if (ret != ESP_OK) { ESP_ERROR_CHECK_WITHOUT_ABORT(ret); return NULL; };
    return handle;
}

inline esp_err_t esp_timer_start(esp_timer_handle_t handle, uint64_t period) {
  if (esp_timer_is_active(handle)) return esp_timer_restart(handle, period);
  return esp_timer_start_once(handle, period); 
}

inline esp_err_t 
timer_alarm(gptimer_handle_t handle, uint64_t value, bool reload = false, uint64_t count = 0) {
	gptimer_alarm_config_t alarm_config{
			.alarm_count = value,
			.reload_count = count,
			.flags = {.auto_reload_on_alarm = reload} //.flags.auto_reload_on_alarm = reload,
	};
	return gptimer_set_alarm_action(handle, &alarm_config);
}

inline gptimer_handle_t 
timer_init(uint64_t value, gptimer_alarm_cb_t func, bool reload = false, uint8_t priority = 3) {
		esp_err_t ret; gptimer_handle_t handle;
		gptimer_config_t config {
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000000,
			.intr_priority = priority,
			.flags { .intr_shared = 1, .allow_pd = 0,.backup_before_sleep = 0 },
		}; //SOC_TIMER_GROUP_TOTAL_TIMERS
		gptimer_event_callbacks_t cbs = { .on_alarm = func };
		if ((ret = gptimer_new_timer(&config, &handle)) 
			|| (ret = gptimer_register_event_callbacks(handle, &cbs, NULL))
			|| (ret = gptimer_enable(handle))
			|| (ret = timer_alarm(handle, value, reload))) 
			{ ESP_ERROR_CHECK_WITHOUT_ABORT(ret); return NULL;}
		return handle;
}

inline esp_err_t 
gptimer_restart(gptimer_handle_t h) { return gptimer_set_raw_count(h, 0);}

inline uint64_t timer_read(gptimer_handle_t handle) {
	uint64_t value = 0;
	esp_err_t ret = gptimer_get_raw_count(handle, &value);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
	return value;
}

void ARDUINO_ISR_ATTR pinMode(uint8_t pin, uint8_t mode) {
	if (mode >= 32) { log_w("%u, %u", pin, mode); return; }
	gpio_config_t conf = {
		.pin_bit_mask = (1ULL << pin),
		.mode = (gpio_mode_t)(mode & GPIO_MODE_INPUT_OUTPUT_OD),
		.pull_up_en = mode & PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,		
		.pull_down_en = mode & PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,		
		.intr_type = (gpio_int_type_t)GPIO_LL_GET_HW(GPIO_PORT_0)->pin[pin].int_type,
	};
	if (gpio_config(&conf) != ESP_OK) log_e("IO %i config failed", pin);
}
 
/*  __attribute__((__always_inline__)) inline */ void digitalWrite(uint8_t pin, uint8_t val) {
    if (!digitalPinCanOutput(pin)) return;
    gpio_hal_context_t gpiohal { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; 
	gpio_hal_set_level(&gpiohal, pin, val);
}

int digitalRead(uint8_t pin) {
	gpio_hal_context_t gpiohal { .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; //return gpio_ll_get_level(&GPIO, (gpio_num_t)pin);
	return gpio_hal_get_level(&gpiohal, (gpio_num_t)pin);
}

inline void digitalToggle(uint8_t pin) { digitalWrite(pin, !digitalRead(pin)); }

void attachInterruptArg(uint8_t pin, void(*userFunc )(void*), void* arg, int intr_type) {
	if (pin >= SOC_GPIO_PIN_COUNT) return;// makes sure that pin -1 (255) will never work -- this follows Arduino standard
	esp_err_t err = gpio_install_isr_service((int)ARDUINO_ISR_FLAG);
	if (err != ESP_OK) { log_e("IO %i ISR Service Failed To Start", pin); return; }
	gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0b111));
	if (intr_type & 0b1000) { gpio_wakeup_enable((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0b111)); }
	gpio_isr_handler_add((gpio_num_t)pin, userFunc, arg);
	gpio_hal_context_t gpiohal{ .dev = GPIO_LL_GET_HW(GPIO_PORT_0) }; //FIX interrupts on peripherals outputs (eg. LEDC,...) 
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
