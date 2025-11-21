#pragma once
#pragma GCC diagnostic ignored "-Wmisleading-indentation" //braces
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"  //switch
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //struct
//#pragma GCC diagnostic ignored "-Wunused-variable"
//#define DEBUG_ENABLE
#include <ESP_MAIN.h>
#include "esp_timer_api.h"
#include "driver/uart.h"
#include "esp_random.h"
#include "mbedtls/md.h"
#include "rom/crc.h"
#include "esp_check.h"
#include "gap.h"
#include "gatt_svc.h"
#include "heart_rate.h"
#include "led.h"
#include "common.h"
					#define FACTORY_FIRMWARE
#ifdef FACTORY_FIRMWARE
#include "web_server.h"
#endif
#include "credentials.h"
//#define NO_PARSE_KEY
#define TAG "MAIN"
#define TMR "TMR"
#define NVS "NVS"


#define PIN_LINE 0//9
#define PIN_RELAY 3

typedef struct { byte patch , updated;  uint16_t crc; } sets_t;
static_assert(sizeof(sets_t) == 4);
typedef enum : uint8_t { ok, ADV, OTA, VALID, NOTIFY,  MAIN, RESTART, } action;

StackType_t xMainStack[4*1024] , xHostStack[NIMBLE_HS_STACK_SIZE];
StaticTask_t xMainTaskBuffer , xHostTaskBuffer;
TaskHandle_t main_handle, ble_handle;
 /*sizeof(StaticTimer_t); 40 sizeof(StaticTask_t); 344*/
esp_timer_handle_t timer_patch __unused;
static char password[64] = DEF_BLE_PASS;
byte scan_key[64] = DEF_BLE_SCAN_DATA;
byte password_len = sizeof(DEF_BLE_PASS)-1;  static_assert(sizeof(DEF_BLE_PASS)-1 < 64);
byte scan_key_len = DEF_BLE_SCAN_DATA_LEN;	static_assert(sizeof(DEF_BLE_PASS)-1 < 64);
static const auto wifi_key = DEF_OTA_KEY;
static uint32_t pincode;
static sets_t sets  /* __attribute__((section(".noinit." "1")))  */; 
__unused esp_err_t update_error;

static struct bond_mac_s {
	ble_addr_t arr;
	byte crc;
} bonded_addr __attribute__((section(".noinit." "1")));

__unused static void mainTask(void *);
//__unused static void nimble_host_task(void *);
__unused static void ble_hs_cfg_init();
extern "C" void ble_store_config_init(void);

__unused static void IRAM_ATTR isr_handler();
void patch_func(bool state);
__unused static void rand_device_name();
static uint32_t generate_pin(uint32_t, const char * = password, byte = password_len);
static void get_task_list(String& str);
String get_task_list() { String str; get_task_list(str); return str; }
void print_task_list() { DEBUGLN(get_task_list()); DEBUGLN(esp_timer_dump(stdout)); };
template <bool = false, char = 0> void bytes_to_str(char* ptr, cbyte* buf, byte data_size);
void strtoB(const char* str, byte* buf, size_t& data_size, size_t buf_len);
//void generate_salt();
void ble_delete_all_peers(bond_mac_s* = nullptr);
void set_boot_partition(const esp_partition_subtype_t);
bool read_bonded_mac();
void save_bonded_mac(const ble_addr_t &);
void read_noinit();
void write_noinit(byte val);
void nvs_read_sets();
void nvs_write_sets(nvsApi nvs = nvsApi(NVS_SPACE_SETS, NVS_READWRITE));
esp_err_t save_auth_data(cch* = nullptr);
void read_auth_data();
void update_start_cb() { sets.updated = 0; };
void update_finish_cb() { sets.updated = 1; nvs_write_sets(); };
void set_main_part() { set_boot_partition(ESP_PARTITION_SUBTYPE_APP_OTA_0); }

__unused void print_addr(cbyte* addr) { for (byte i = 5;;i--) { DEBUGF("%02X", addr[i]); if (!i) break; DEBUG(':'); } DEBUGLN(); }

#ifdef DEBUG_ENABLE
void timer_patch_off_cb(__unused void * arg = NULL) { led_off();}
void impl_io_on() { led_on(); }
void impl_io_off() { led_off(); }
uint8_t impl_io_get() { return digitalRead(PIN_LINE);/*get_led_state();*/ }
uint32_t get_pincode() { return 111111; }
#else
void timer_patch_off_cb(void * arg) { enableInterrupt(PIN_LINE); sets.patch = 0; if(!arg) nvs_write_sets(); }
void impl_io_on() { patch_func(true); }
void impl_io_off() { patch_func(false); }
uint8_t impl_io_get() { return digitalRead(PIN_LINE); }
uint32_t get_pincode() { return pincode; }
#endif

void partition_read() {
	std::vector<const esp_partition_t*> partArr;
	auto i = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
	for (;i != NULL; i = esp_partition_next(i)) {
		partArr.push_back(esp_partition_get(i));
	}ESP_LOGI("","partition count: %u", partArr.size());
	for (__unused auto& p : partArr) { DEBUGF("Label %s, size %lu, address 0x%lX\n", p->label, p->size, p->address); }
	esp_partition_iterator_release(i);
}

bool nvsOpen(const char* name_group, nvs_open_mode_t open_mode, nvs_handle_t* nvs_handle) {
	esp_err_t err = nvs_open(name_group, open_mode, nvs_handle);
	if (err == ESP_OK) return true;
	if (err != ESP_ERR_NVS_NOT_FOUND || open_mode != NVS_READONLY) {
		ESP_LOGE("NVS", "Error opening NVS namespace \"%s\": %d (%s)!", name_group, err, esp_err_to_name(err));
	}
	return false;
}

esp_err_t nvsGet(nvs_handle_t handle, cch* key, nvs_type_t type, uint64_t& result, void*& buf, size_t* size = nullptr) {
	esp_err_t ret = ESP_FAIL; size_t required_size; void* ptr;
	switch (type) {
	case NVS_TYPE_U8:ret = nvs_get_u8(handle, key, (uint8_t*)&result); break;
	case NVS_TYPE_I8:ret = nvs_get_i8(handle, key, (int8_t*)&result); break;
	case NVS_TYPE_U16:ret = nvs_get_u16(handle, key, (uint16_t*)&result); break;
	case NVS_TYPE_I16:ret = nvs_get_i16(handle, key, (int16_t*)&result); break;
	case NVS_TYPE_U32:ret = nvs_get_u32(handle, key, (uint32_t*)&result); break;
	case NVS_TYPE_I32:ret = nvs_get_i32(handle, key, (int32_t*)&result); break;
	case NVS_TYPE_U64:ret = nvs_get_u64(handle, key, &result); break;
	case NVS_TYPE_I64:ret = nvs_get_i64(handle, key, (int64_t*)&result); break;
	case NVS_TYPE_STR:ret = -2;
		nvs_get_str(handle, key, NULL, &required_size);
		ptr = realloc(buf, required_size);
		if (ptr == NULL) return -1; buf = ptr;
		nvs_get_str(handle, key, (char*)buf, &required_size);
		if (size) *size = required_size; break;
	case NVS_TYPE_BLOB: ret = -3;
		nvs_get_blob(handle, key, NULL, &required_size);
		ptr = realloc(buf, required_size);
		if (ptr == NULL) return -1; buf = ptr;
		nvs_get_blob(handle, key, buf, &required_size);
		if (size) *size = required_size;break; default: break;
	}
	return ret;
}

void nvs_test() {
	esp_err_t ret; void* buf = NULL; uint64_t result = 0; size_t _size = 0;
	nvs_stats_t nvs_stats{}; nvs_entry_info_t entry; nvs_iterator_t it = NULL; 
	nvs_get_stats(NULL, &nvs_stats);
	DEBUGF("UsedEntries = (%u), FreeEntries = (%u), AvailableEntries = (%u), AllEntries = (%u), Namespaces = (%u)\n",
		nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.available_entries, nvs_stats.total_entries, nvs_stats.namespace_count);
	ret = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
	while (ret == ESP_OK) {
		nvs_entry_info(it, &entry); // Can omit error check if parameters are guaranteed to be non-NULL
		nvsApi nvs; DEBUGF("space '%s'\tkey '%s'\ttype '%d'", entry.namespace_name, entry.key, entry.type);
		if(nvs.begin(entry.namespace_name, NVS_READONLY)) {
			switch (auto ret = nvsGet(nvs, entry.key, entry.type, result, buf, &_size)) {
			case ESP_OK: DEBUGF("\tData = %llu\n", result); break;
			case -2: DEBUGF("\nStr: %s\n", (char*)buf); break;
			case -3: DEBUGF("\nBlob (size %u): ", _size);
				for (size_t i = 0; i < _size; i++) { DEBUGF("%02X ", ((byte*)buf)[i]); }; DEBUGLN(); break;
			default:ESP_LOGE(__FUNCTION__, "%i", ret);
			}
		}
		ret = nvs_entry_next(&it);
	}
	free(buf);
	nvs_release_iterator(it);
}

void uart_cb() {
	static char buf[64];
	int len = uart_read_bytes(UART_NUM_0, buf, sizeof(buf)-1, pdMS_TO_TICKS(0));
    if(len < 0) return;
	//buf[len] = '\0';
	//ESP_LOGI("uart", "%u bytes" ,len);
	switch (*buf) {
	case 'P': print_task_list(); break;
	case 'R': vTaskDelay(1); esp_restart(); break;
	case 'D': ble_delete_all_peers(); break;
	//default: Serial.write(buf, len);//Serial.write('\n');break;
	}
	/* if(!strcmp(buf, "P")) {print_task_list();}
	else if(!strcmp(buf, "R")) {xTaskNotify(main_handle, RESTART, eSetValueWithOverwrite);} */
}

void vApplicationIdleHook(void) {
    static bool prev_state = 0;
    if(!sets.patch) {
        const bool val = digitalRead(PIN_LINE);
        if(prev_state != val) { digitalWrite(PIN_RELAY, prev_state = val); }
    }
}

