#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if !defined(CONFIG_ESP_CONSOLE_NONE) && (defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED) || defined(CONFIG_ESP_CONSOLE_USB_CDC))// && defined //CONFIG_USJ_ENABLE_USB_SERIAL_JTAG
#define ARDUINO_USB_CDC_ON_BOOT 1
#if (CONFIG_ESP_CONSOLE_USB_CDC) && (CONFIG_TINYUSB_CDC_ENABLED)
#define ARDUINO_USB_MODE 0
#else
#define ARDUINO_USB_MODE 1
#endif
#endif 

#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "driver/gptimer_types.h"
#include "esp_task_wdt.h"
#include "esp_check.h"
#include "ESP_DEFS.h"
//#include "esp_task_wdt.h"
//ESP_ROM_HAS_NEWLIB_NANO_FORMAT //627063
//ESP_ROM_HAS_NEWLIB_NORMAL_FORMAT //643633
#ifdef CONFIG_LIBC_NEWLIB_NANO_FORMAT //~16,570 bytes smaller
#pragma message "NEWLIB_NANO_FORMAT"
#endif
#if _ESP_LOG_ENABLED(1)
	#define DEBUG_ENABLE
#pragma message "DEBUG_ENABLE"
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE  //Serial used from Native_USB_CDC | HW_CDC_JTAG
#pragma message "HWCDC" 
#else 
#pragma message "UART0"
#endif  // !ARDUINO_USB_CDC_ON_BOOT -- Serial is used from UART0
#define DEBUG(x, ...) printf(x, ##__VA_ARGS__)
#define DEBUGLN() printf("\n")
//#define DEBUGLN(x, ...) printf("%s\n", x, ##__VA_ARGS__)
#define DEBUGF(x, ...) printf(x , ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#define DEBUGLN()
#define DEBUGF(x, ...)
#endif // DEBUG_ENABLE
#ifdef NDEBUG
#error "NDEBUG"
#endif
typedef const char cch; typedef uint8_t byte; typedef const uint8_t cbyte; 
typedef unsigned uint; typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;

#ifdef __cplusplus
extern "C" {
#endif
int64_t micros();
uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
void digitalToggle(uint8_t pin);

void attachInterrupt(uint8_t pin, void (*)(void), int mode);
void attachInterruptArg(uint8_t pin, void (*)(void *), void *arg, int mode);
void detachInterrupt(uint8_t pin);
void enableInterrupt(uint8_t pin);
void disableInterrupt(uint8_t pin);
#if CONFIG_BOOTLOADER_WDT_DISABLE_IN_USER_CODE
void rtc_wdt_disable(void);
void rtc_wdt_feed(void);
#endif
#if CONFIG_ESP_TASK_WDT_EN
void task_wdt_reconfigure(uint32_t ms, uint32_t idle_core_mask = 0);
void task_wdt_init(uint32_t ms, uint32_t idle_core_mask = 0);
#endif

esp_timer_handle_t 
esp_timer_new(esp_timer_cb_t cb, void* arg = NULL, esp_timer_dispatch_t type = ESP_TIMER_TASK, const char* name = NULL, bool skip = 0);
esp_err_t esp_timer_start(esp_timer_handle_t handle, uint64_t period);
uint64_t esp_timer_period(esp_timer_handle_t handle);

esp_err_t gptimer_alarm(gptimer_handle_t handle, uint64_t value, bool reload = 0, uint64_t count = 0);
gptimer_handle_t gptimer_init(uint64_t value, gptimer_alarm_cb_t func, bool reload = 0, uint8_t priority = 3);
esp_err_t gptimer_restart(gptimer_handle_t handle);
uint64_t gptimer_read(gptimer_handle_t handle);

uint64_t getEfuseMac();
size_t heap_caps_sprint_heap_info(void* buf, uint32_t caps = MALLOC_CAP_INTERNAL);

void nvs_init();
esp_ota_img_states_t img_state_get(bool valid = false);
#ifdef __cplusplus
}
#endif

inline uint32_t getHeapSize() { return heap_caps_get_total_size(MALLOC_CAP_INTERNAL);}
inline uint32_t getFreeHeap() { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL); }
inline uint32_t getMinFreeHeap() { return heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL); }
inline uint32_t getMaxAllocHeap() { return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL); }
inline void printHeapInfo() { heap_caps_print_heap_info(MALLOC_CAP_INTERNAL); }

#ifdef __cplusplus
class nvsApi {
private: nvs_handle_t _handle = 0;
public:
	nvsApi() {};
	nvsApi(const nvsApi &obj) = delete;
	nvsApi& operator=(const nvsApi&) = delete;
	nvsApi(nvsApi &other) : _handle(other._handle) { other._handle = 0; ESP_LOGV("NVS", "ctor copy"); };
	nvsApi(nvsApi &&other) : _handle(other._handle) { other._handle = 0; ESP_LOGV("NVS", "ctor move"); };
	nvsApi(const char* space, nvs_open_mode_t mode) { begin(space, mode); };
	esp_err_t begin(const char* space, nvs_open_mode_t mode = NVS_READONLY) {
		esp_err_t ret = nvs_open(space, mode, &_handle);
		if(ret) { ESP_LOGE("NVS", "0x%X", ret); }
		return ret;
	};
	void close() { if(_handle) { nvs_close(_handle); } };
	~nvsApi() { close(); ESP_LOGV("NVS", "~_handle = %u", (int)_handle); };
	operator nvs_handle_t() const { return _handle; };
};

inline __nonnull_all void nvs_log(const char* except = NULL) {
	__unused const char* TAG = "NVS";
	size_t buf_cap = 256; esp_err_t ret; 
	void* buf = malloc(buf_cap); assert(buf);
	nvs_stats_t nvs_stats {}; nvs_entry_info_t entry; nvs_iterator_t it = NULL;
	nvs_get_stats(NULL, &nvs_stats);
	ESP_LOGI(TAG, "Used %u, Free %u, Available %u, All = %u, Namespaces %u entries",
		nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.available_entries, nvs_stats.total_entries, nvs_stats.namespace_count);
	for (ret = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it); ret == ESP_OK; ret = nvs_entry_next(&it)) 
	{
		nvs_entry_info(it, &entry); // Can omit error check if parameters are guaranteed to be non-NULL
		nvsApi nvs; ESP_LOGI(TAG, "space '%s'\tkey '%s' type 0x%u", entry.namespace_name, entry.key, entry.type);
		
		if((ret = nvs.begin(entry.namespace_name)) == ESP_OK) {
			nvs_type_t type = entry.type;
			//nvsGet(nvs, entry.key, type, &buf, &buf_cap); CHECK_(ret);
			size_t size; *(uint32_t*)buf = 0x0; cch *key = entry.key;
			switch (type) {
			case NVS_TYPE_U8: ret = nvs_get_u8(nvs, key, (uint8_t*)buf); break;
			case NVS_TYPE_I8: ret = nvs_get_i8(nvs, key, (int8_t*)buf); break;
			case NVS_TYPE_U16: ret = nvs_get_u16(nvs, key, (uint16_t*)buf); break;
			case NVS_TYPE_I16: ret = nvs_get_i16(nvs, key, (int16_t*)buf); break;
			case NVS_TYPE_U32: ret = nvs_get_u32(nvs, key, (uint32_t*)buf); break;
			case NVS_TYPE_I32: ret = nvs_get_i32(nvs, key, (int32_t*)buf); break;
			case NVS_TYPE_U64: ret = nvs_get_u64(nvs, key, (uint64_t*)buf); break;
			case NVS_TYPE_I64: ret = nvs_get_i64(nvs, key, (int64_t*)buf); break;
			case NVS_TYPE_FLOAT: ret = nvs_get_float(nvs, key, (float*)buf); break;
			case NVS_TYPE_DOUBLE: ret = nvs_get_double(nvs, key, (double*)buf); break;
			case NVS_TYPE_STR:
				if ((ret = nvs_get_str(nvs, key, NULL, &size))) break;
				if(!strcmp(except, entry.key)) continue;
				if (size > buf_cap) {
					void* tmp = realloc(buf, size); 
					if (!tmp) { ESP_LOGE(TAG, "realloc size %u", size); continue; }
					buf = tmp;
					buf_cap = size;
				} 
				nvs_get_str(nvs, key, (char*)buf, &size); break;
			case NVS_TYPE_BLOB:
				if ((ret = nvs_get_blob(nvs, key, NULL, &size))) break;
				if(!strcmp(except, entry.key)) continue;
				if (size > buf_cap) {
					void* tmp = realloc(buf, size); 
					if (!tmp) { ESP_LOGE(TAG, "realloc size %u", size); continue; }
					buf = tmp;
					buf_cap = size;
				} 
				nvs_get_blob(nvs, key, buf, &size); break;
			default: ESP_LOGE(TAG, "NVS_TYPE = 0x%X", type);
			}

			if (ret == ESP_OK) {
				switch (type) {
				case NVS_TYPE_U8:
				case NVS_TYPE_I8:
				case NVS_TYPE_U16:
				case NVS_TYPE_I16:
				case NVS_TYPE_U32:
				case NVS_TYPE_I32:
					DEBUGF("Data = %u", *(int*)buf); 
					if(*(uint32_t*)buf >= 10) { DEBUGF(" (0x%lX)", *(uint32_t*)buf); }
					DEBUGLN(); break;
				case NVS_TYPE_U64:
				case NVS_TYPE_I64:
#ifdef CONFIG_LIBC_NEWLIB_NANO_FORMAT
					DEBUGF("Data = "); if (((uint32_t*)buf)[1]) { DEBUGF("%02X", ((int*)buf)[1]); }
					DEBUGF("%08lX\n", ((uint32_t*)buf)[0]);
#else
					DEBUGF("Data = %llu\n", *(uint64_t*)buf); 
#endif				
					break;
				case NVS_TYPE_FLOAT: continue; //TODO
				case NVS_TYPE_DOUBLE: continue;
				case NVS_TYPE_STR: ESP_LOGI(TAG, "Strlen: %u \"%.*s\"\n", size, size, (char*)buf); break;
				case NVS_TYPE_BLOB: ESP_LOGI(TAG, "Blob size %u:", size);
					for (size_t i = 0; i < size; i++) { DEBUGF("%02X ", ((uint8_t*)buf)[i]); }; 
					DEBUGLN(); break;
				default: break;
				}
			} else { CHECK_(ret); }
			DEBUGLN();
		} else { CHECK_(ret); }
	}
	free(buf);
	nvs_release_iterator(it);
}
#endif

#include "esp_partition.h"
inline void partition_log() {
	__unused const char* TAG = "partition";
	auto i = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
	for (;i; i = esp_partition_next(i)) {
		const esp_partition_t* partArr = esp_partition_get(i);
		ESP_LOGI(TAG, "Label %s, size %u, address 0x%X", 
			partArr->label, (int)partArr->size, (int)partArr->address);
	}
	esp_partition_iterator_release(i);
}