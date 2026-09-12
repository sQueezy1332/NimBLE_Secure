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
#define INPUT				0x01
#define OUTPUT				0x03
#define PULLUP				0x04
#define INPUT_PULLUP		0x05
#define PULLDOWN			0x08
#define INPUT_PULLDOWN		0x09
#define OPEN_DRAIN			0x10
#define OUTPUT_OPEN_DRAIN	0x13
#define ANALOG				0xC0
//ESP_ROM_HAS_NEWLIB_NANO_FORMAT //627063
//ESP_ROM_HAS_NEWLIB_NORMAL_FORMAT //643633
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_check.h"
//#include "esp_task_wdt.h"

#ifdef CONFIG_LIBC_NEWLIB_NANO_FORMAT //~16,570 bytes smaller
#pragma message "NEWLIB_NANO_FORMAT"
#endif
#ifdef DEBUG_ENABLE
#pragma message "DEBUG_ENABLE"
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE  //Serial used from Native_USB_CDC | HW_CDC_JTAG
#pragma message "HWCDC" 
#elif ARDUINO_USB_CDC_ON_BOOT// !ARDUINO_USB_MODE -- Native USB Mode
#pragma message "USBCDC"
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
uint64_t esp_timer_period(esp_timer_handle_t handle);

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

#include "esp_partition.h"
inline void partition_read() {
	auto i = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
	for (;i; i = esp_partition_next(i)) {
		const esp_partition_t* partArr = esp_partition_get(i);
		ESP_LOGI("partition", "Label %s, size %lu, address 0x%lX", 
			partArr->label, partArr->size, partArr->address);
	}
	esp_partition_iterator_release(i);
}

inline bool nvsOpen(const char* name_group, nvs_open_mode_t open_mode, nvs_handle_t* nvs_handle) {
	esp_err_t err = nvs_open(name_group, open_mode, nvs_handle);
	if (err == ESP_OK) return true;
	if (err != ESP_ERR_NVS_NOT_FOUND || open_mode != NVS_READONLY) {
		ESP_LOGE("NVS", "Error opening NVS namespace \"%s\": %d (%s)!", name_group, err, esp_err_to_name(err));
	}
	return false;
}

inline esp_err_t nvsGet(nvs_handle_t handle, cch* key, nvs_type_t type, void* &buf_out, size_t* size_out = nullptr) {
	esp_err_t ret; size_t required_size; void* ptr; *(uint32_t*)buf_out = 0x0;
	switch (type) {
	case NVS_TYPE_U8:ret = nvs_get_u8(handle, key, (uint8_t*)buf_out); break;
	case NVS_TYPE_I8:ret = nvs_get_i8(handle, key, (int8_t*)buf_out); break;
	case NVS_TYPE_U16:ret = nvs_get_u16(handle, key, (uint16_t*)buf_out); break;
	case NVS_TYPE_I16:ret = nvs_get_i16(handle, key, (int16_t*)buf_out); break;
	case NVS_TYPE_U32:ret = nvs_get_u32(handle, key, (uint32_t*)buf_out); break;
	case NVS_TYPE_I32:ret = nvs_get_i32(handle, key, (int32_t*)buf_out); break;
	case NVS_TYPE_U64:ret = nvs_get_u64(handle, key, (uint64_t*)buf_out);
		if(ret) return ret;
		ret = 1; break;
	case NVS_TYPE_I64:ret = nvs_get_i64(handle, key, (int64_t*)buf_out);
		if(ret) return ret;
		return 1;
	case NVS_TYPE_STR:
		if((ret = nvs_get_str(handle, key, NULL, &required_size))) return ret;
		ptr = realloc(buf_out, required_size);
		if (!ptr) return ESP_ERR_NO_MEM; buf_out = ptr;
		nvs_get_str(handle, key, (char*)buf_out, &required_size);
		if (size_out) *size_out = required_size; 
		return 2;
	case NVS_TYPE_BLOB: 
		if((ret = nvs_get_blob(handle, key, NULL, &required_size))) return ret;
		ptr = realloc(buf_out, required_size);
		if (!ptr) return ESP_ERR_NO_MEM; buf_out = ptr;
		nvs_get_blob(handle, key, buf_out, &required_size);
		if (size_out) *size_out = required_size;
		return 3;
		default: return ESP_FAIL;
	}
	return ret;
}

inline void nvs_test(const char* key = nullptr) {
	__unused static const char* TAG = "NVS";
	esp_err_t ret; void* buf = malloc(64); size_t buf_len = 0;
	nvs_stats_t nvs_stats {}; nvs_entry_info_t entry; nvs_iterator_t it = NULL;
	nvs_get_stats(NULL, &nvs_stats);
	ESP_LOGI(TAG, "Used %u, Free %u, Available %u, All = %u, Namespaces %u entries",
		nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.available_entries, nvs_stats.total_entries, nvs_stats.namespace_count);
	for (ret = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it); ret == ESP_OK; ret = nvs_entry_next(&it)) {
		nvs_entry_info(it, &entry); // Can omit error check if parameters are guaranteed to be non-NULL
		nvsApi nvs; ESP_LOGI(TAG, "space '%s'\tkey '%s' type '%u'", entry.namespace_name, entry.key, entry.type);
		if(key && !strcmp(key,entry.key)) continue;
		if(!nvs.begin(entry.namespace_name, NVS_READONLY)) {
			switch (esp_err_t ret = nvsGet(nvs, entry.key, entry.type, buf, &buf_len)) {
			case ESP_OK: 
				DEBUGF("Data = %lu\n", *(uint32_t*)buf); 
				break;
			case 1:
				#ifdef CONFIG_LIBC_NEWLIB_NANO_FORMAT
				DEBUGF("Data = "); if(((uint32_t*)buf)[1]) { DEBUGF("%02lX", ((uint32_t*)buf)[1]); }
				DEBUGF("%08lX", ((uint32_t*)buf)[0]); DEBUGLN();
				#else
				DEBUGF("Data = %llu\n", *(uint64_t*)buf);
				#endif
				break;
			case 2: 
				ESP_LOGI(TAG, "Str: %s\n", (char*)buf); break;
			case 3: 
				ESP_LOGI(TAG, "Blob size %u: ", buf_len);
				for (size_t i = 0; i < buf_len; i++) { DEBUGF("%02X ", ((uint8_t*)buf)[i]); }; DEBUGLN();
				break;
			default: ESP_LOGE(TAG, "nvsGet()" " 0x%X", ret);
			} DEBUGF("\n");
		}
	}
	free(buf);
	nvs_release_iterator(it);
}
