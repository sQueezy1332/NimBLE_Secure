#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#define DEBUG_ENABLE
#if !defined(CONFIG_ESP_CONSOLE_NONE) && (defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED) || defined(CONFIG_ESP_CONSOLE_USB_CDC))// && defined //CONFIG_USJ_ENABLE_USB_SERIAL_JTAG
#define ARDUINO_USB_CDC_ON_BOOT 1
#if (CONFIG_ESP_CONSOLE_USB_CDC) && (CONFIG_TINYUSB_CDC_ENABLED)
#define ARDUINO_USB_MODE 0
#else
#define ARDUINO_USB_MODE 1
#endif
#endif 
//ESP_ROM_HAS_NEWLIB_NANO_FORMAT //627063
//ESP_ROM_HAS_NEWLIB_NORMAL_FORMAT //643633
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_check.h"
//#include "esp_task_wdt.h"

#if defined (CONFIG_LIBC_NEWLIB_NANO_FORMAT) && (defined CONFIG_LIBC_NEWLIB_NANO_FORMAT)
#pragma message "NEWLIB_NANO_FORMAT" //~16,570 bytes smaller
#endif
#ifdef DEBUG_ENABLE
#pragma message "DEBUG_ENABLE"
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE  //Serial used from Native_USB_CDC | HW_CDC_JTAG
#pragma message "HWCDC" 
#elif ARDUINO_USB_CDC_ON_BOOT// !ARDUINO_USB_MODE -- Native USB Mode
#pragma message "USBCDC"
#else 
#pragma message "UART0" //definiton in HardwareSerial.cpp
#endif  // !ARDUINO_USB_CDC_ON_BOOT -- Serial is used from UART0
#define DEBUG(x, ...) printf(x, ##__VA_ARGS__)
#define DEBUGLN(x, ...) printf("\n");
#define DEBUGF(x, ...) printf(x , ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#define DEBUGLN(x, ...) 
#define DEBUGF(x, ...)
#endif // DEBUG_ENABLE
#define FUN __FUNCTION__
#define FUNC_ADDRESS (esp_cpu_get_call_addr((intptr_t)__builtin_return_address(0)))
#define CHECK_(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x)
#define CHECK_RET(x) ESP_RETURN_ON_ERROR(x,"","0x%08x",FUNC_ADDRESS)
#define CHECK_VOID(x) ESP_RETURN_VOID_ON_ERROR(x,"","0x%08x",FUNC_ADDRESS)
#define uS() esp_timer_get_time()
#define delayUntil(prev, tmr) vTaskDelayUntil((prev),pdMS_TO_TICKS(tmr))
#define SEC (1000000ULL)
#define ENTER_CRITICAL() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux);
#define EXIT_CRITICAL() portEXIT_CRITICAL(&mux);
typedef const char cch; typedef uint8_t byte; typedef const uint8_t cbyte; 
typedef unsigned uint; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;

inline void delay(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }
inline void delayMicroseconds(uint32_t us) { esp_rom_delay_us(us); }
#ifdef __cplusplus
extern "C" {
#endif

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
void digitalToggle(uint8_t pin);

void attachInterrupt(uint8_t pin, void (*)(void), int mode);
void attachInterruptArg(uint8_t pin, void (*)(void *), void *arg, int mode);
void detachInterrupt(uint8_t pin);
void enableInterrupt(uint8_t pin);
void disableInterrupt(uint8_t pin);

esp_timer_handle_t 
esp_timer_new(esp_timer_cb_t cb, esp_timer_dispatch_t type = ESP_TIMER_TASK, bool skip = 0, void* arg = NULL, const char* name = NULL);
esp_err_t esp_timer_start(esp_timer_handle_t handle, uint64_t period);

esp_err_t gptimer_alarm(gptimer_handle_t handle, uint64_t value, bool reload = 0, uint64_t count = 0);
gptimer_handle_t gptimer_init(uint64_t value, gptimer_alarm_cb_t func, bool reload = 0, uint8_t priority = 3);
esp_err_t gptimer_restart(gptimer_handle_t handle);
uint64_t gptimer_read(gptimer_handle_t handle);

uint64_t getEfuseMac();

void nvs_init();
esp_ota_img_states_t img_state(bool valid = false);
#ifdef __cplusplus
}
#endif

inline uint32_t getHeapSize() { return heap_caps_get_total_size(MALLOC_CAP_INTERNAL); }

inline uint32_t getFreeHeap() { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL); }

inline uint32_t getMinFreeHeap() { return heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL); }

inline uint32_t getMaxAllocHeap() { return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL); }

inline void printHeapInfo() { heap_caps_print_heap_info(MALLOC_CAP_INTERNAL); }

class nvsApi {
private: nvs_handle_t _handle = 0;
public:
	nvsApi() {};
	nvsApi(const nvsApi &obj) = delete;
	nvsApi& operator=(const nvsApi&) = delete;
	nvsApi(nvsApi &other) : _handle(other._handle) { other._handle = 0; ESP_LOGV("NVS", "ctor copy"); };
	nvsApi(nvsApi &&other) : _handle(other._handle) { other._handle = 0; ESP_LOGV("NVS", "ctor move"); };
	nvsApi(const char* space, nvs_open_mode_t mode) { begin(space, mode); };
	esp_err_t begin(const char* space, nvs_open_mode_t mode) {
		//if(_handle) return 0xDADADA;
		esp_err_t ret = nvs_open(space, mode, &_handle);
		if(ret) { ESP_LOGE("NVS", "0x%X", ret); }
		return ret;
	};
	void close() { if(_handle) { nvs_close(_handle); } };
	~nvsApi() { close(); ESP_LOGV("NVS", "~_handle = %lu", _handle); };
	operator nvs_handle_t() const { return _handle; };
};
