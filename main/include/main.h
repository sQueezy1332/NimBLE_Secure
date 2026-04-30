#pragma once
#pragma GCC diagnostic ignored "-Wmisleading-indentation" //braces
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"  //switch
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //struct
				#define DEBUG_ENABLE
#define _WANT_USE_LONG_TIME_T
#include "esp_main.h"
#include "driver/gpio.h"
#include "driver/uart.h"
//#include "esp_random.h"
#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include "mbedtls/md.h"
#include "rom/crc.h"
//#include "nimble/nimble_port_freertos.h" 
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
//#include "host/util/util.h"
#include "gap.h"
#include "gatt_svc.h"
#define HEART_RATE_PERIOD (2000 * 1000)
//#include "led.h"
//#include "common.h"

#include "esp_mac.h"
#include "esp_hmac.h"
				//#define CONFIG_FACTORY_FIRMWARE
				//#define CONFIG_GENERIC_PATCHER
//#ifdef CONFIG_FACTORY_FIRMWARE
#include "wifi_api.h"
#include "http_server.h"
//#endif

//#define NO_PARSE_KEY
//#define TAG "MAIN"
static const char* TAG = "MAIN";
#define TMR "TMR"
#define NVS "NVS"

#define dWrite(x,y) digitalWrite(x, y)
#define dRead(x) digitalRead(x)

#define PIN_RELAY 4
#define PIN_RELAY_GND 3 //unused in code
	//#define PIN_RELAY_2 2
#define PIN_LED 8

#ifdef CONFIG_GENERIC_PATCHER
#define PIN_LINE 1
#define PIN_LED_MASK BIT(PIN_LED)
#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT_OUTPUT)
#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_3)
#define RELAY_DEFAULT_IMPL() dWrite(PIN_RELAY, 0)
#define RELAY_PATCH_IMPL() dWrite(PIN_RELAY, 1)
#define RELAY_UNPATCH_IMPL() dWrite(PIN_RELAY, 0)
#define IO_GET_IMPL() dRead(PIN_RELAY)
#else		//forteza
#define PIN_LINE (PIN_LED)
#define PIN_LED_MASK (0)
#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT_OUTPUT_OD)
#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_0)
#define RELAY_DEFAULT_IMPL() dWrite(PIN_RELAY, 1)
#define RELAY_PATCH_IMPL()
#define RELAY_UNPATCH_IMPL()
#define IO_GET_IMPL() dRead(PIN_LINE)
#endif
#ifdef PIN_RELAY_2
	#define RELAY_2_MASK BIT(PIN_RELAY_2)
	#define RELAY_2_DEFAULT_IMPL() dWrite(PIN_RELAY_2, 0)
	#define RELAY_2_PATCH_IMPL() dWrite(PIN_RELAY_2, 1)
	#define RELAY_2_UNPATCH_IMPL() dWrite(PIN_RELAY_2, 0)
#else
	#define RELAY_2_MASK (0)
	#define RELAY_2_DEFAULT_IMPL()
	#define RELAY_2_PATCH_IMPL()
	#define RELAY_2_UNPATCH_IMPL()
#endif
using String = std::string;
typedef struct { byte patch , updated , valid;  uint8_t crc; } sets_t;
static_assert(sizeof(sets_t) == 4);
typedef enum : uint8_t { ok, ADV, OTA, VALID, NOTIFY_ALARM, NOTIFY_TIME,  MAIN, RESTART, } action;

StackType_t xMainStack[4*1024] , xHostStack[NIMBLE_HS_STACK_SIZE]; //static_assert(configTIMER_TASK_STACK_DEPTH > 3000);
StaticTask_t xMainTaskBuffer , xHostTaskBuffer;
TaskHandle_t h_main_task, h_nimble_task;
 /*sizeof(StaticTimer_t); 40 sizeof(StaticTask_t); 344*/
esp_timer_handle_t timer_patch;
__unused esp_netif_t* h_netif_sta;
__unused esp_netif_t* h_netif_ap;
byte passkey[32] = DEF_BLE_PASS_BASE32;
byte scan_key[32] = DEF_BLE_SCAN_DATA;
byte passkey_len = DEF_BLE_PASS_LEN; 		static_assert(DEF_BLE_PASS_LEN <= sizeof(passkey)); //sizeof(DEF_BLE_PASS)-1;
byte scan_key_len = DEF_BLE_SCAN_DATA_LEN;	static_assert(DEF_BLE_SCAN_DATA_LEN <= sizeof(scan_key));
//static const auto wifi_key = DEF_OTA_KEY;
static uint32_t pincode;
static sets_t sets = {};

static struct bond_mac_s {
	ble_addr_t arr;
	byte crc;
} bonded_addr __attribute__((section(".noinit." "1")));
static_assert(sizeof(bond_mac_s) == 8);

static void wifi_init();
__unused static void mainTask(void *);
//__unused static void nimble_host_task(void *);

//extern void set_cts_unix(time_t now);
static void patch_func(uint64_t = TIMER_PATCH);

__unused static void IRAM_ATTR isr_handler();
__unused static void rand_device_name();
void set_ble_device_name();
//static uint32_t generate_pin(uint32_t, const char * = (char *)passkey, byte = passkey_len);
void strtoB(const char* str, byte* buf, size_t& data_size, size_t buf_len);
template <bool = false, char = 0> void bytes_to_str(char* dest, cbyte* src, size_t data_size);
void bytes_to_str_bigend(char* dest, cbyte* src, size_t data_size) { bytes_to_str<true, ' '>(dest, src, data_size) ; };
//void generate_salt();
void ble_delete_all_peers(bond_mac_s* = nullptr);
bool wifi_sta_wait_conn(); 

bool read_bonded_mac();
//esp_err_t save_bonded_mac(const ble_addr_t &);
void read_noinit();
void write_noinit(byte val);
void nvs_read_sets();
void nvs_write_sets(nvsApi nvs = nvsApi(NVS_SPACE_SETS, NVS_READWRITE));
esp_err_t save_auth_data();
void read_auth_data();

void update_start_cb() { sets.updated = 0; };
void update_finish_cb() { sets.updated = 1; nvs_write_sets(); };
void set_boot_partition(const esp_partition_subtype_t);
void set_main_part() { set_boot_partition(ESP_PARTITION_SUBTYPE_APP_OTA_0); }

int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len);
uint32_t HOTPget(const uint8_t* key, size_t key_len, uint64_t salt);
uint32_t TOTPget(const uint8_t* key, size_t key_len, time_t time = time(NULL));

static void get_task_list(String& str);
String get_task_list() { String str; get_task_list(str); return str; }
void print_task_list() { DEBUGLN(get_task_list().c_str()); /*DEBUGLN(esp_timer_dump(stdout));*/ };

//__unused void print_addr(cbyte* addr) { for (byte i = 5;;i--) { DEBUGF("%02X", addr[i]); if (!i) break; DEBUG(':'); } DEBUGLN(); }

decltype(sets_t::crc) crc_func(const sets_t & buf) {
	return (sizeof(sets_t::crc) == 2) ? crc16_le(0,(byte*)&buf, sizeof(sets_t::crc)) : crc8_le(0,(byte*)&buf, sizeof(sets_t::crc));
}

void patch_func(uint64_t period) { 
	RELAY_PATCH_IMPL(); RELAY_2_PATCH_IMPL();
	if(!sets.patch) { sets.patch = true; nvs_write_sets(); }
	CHECK_(esp_timer_start(timer_patch, period));
}

void unpatch_cb() { if(!sets.patch) { RELAY_2_UNPATCH_IMPL(); } }

void timer_patch_off_cb(void *) { 
    RELAY_UNPATCH_IMPL(); RELAY_2_UNPATCH_IMPL();
    sets.patch = false; nvs_write_sets();
	//ble_gap_terminate();
}

void impl_io_on() { patch_func(); }
void impl_io_off() { patch_func(TIMER_PATCH_OFF); }

uint8_t impl_io_get() { return IO_GET_IMPL(); }

#ifdef DEBUG_ENABLE
uint32_t get_pincode() { return 111111; }
#else
uint32_t get_pincode() { return pincode; }
#endif

#include "esp_partition.h"
void partition_read() {
	auto i = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
	for (;i; i = esp_partition_next(i)) {
		const esp_partition_t* partArr = esp_partition_get(i);
		ESP_LOGI(TAG, "Label %s, size %lu, address 0x%lX", 
			partArr->label, partArr->size, partArr->address);
	}
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

esp_err_t nvsGet(nvs_handle_t handle, cch* key, nvs_type_t type, void* &buf, size_t* size = nullptr) {
	esp_err_t ret; size_t required_size; void* ptr; *(uint32_t*)buf = 0x0;
	switch (type) {
	case NVS_TYPE_U8:ret = nvs_get_u8(handle, key, (uint8_t*)buf); break;
	case NVS_TYPE_I8:ret = nvs_get_i8(handle, key, (int8_t*)buf); break;
	case NVS_TYPE_U16:ret = nvs_get_u16(handle, key, (uint16_t*)buf); break;
	case NVS_TYPE_I16:ret = nvs_get_i16(handle, key, (int16_t*)buf); break;
	case NVS_TYPE_U32:ret = nvs_get_u32(handle, key, (uint32_t*)buf); break;
	case NVS_TYPE_I32:ret = nvs_get_i32(handle, key, (int32_t*)buf); break;
	case NVS_TYPE_U64:ret = nvs_get_u64(handle, key, (uint64_t*)buf);
		if(ret) return ret;
		ret = 1; break;
	case NVS_TYPE_I64:ret = nvs_get_i64(handle, key, (int64_t*)buf);
		if(ret) return ret;
		return 1;
	case NVS_TYPE_STR:
		ret = nvs_get_str(handle, key, NULL, &required_size);
		if(ret) return ret;
		ptr = realloc(buf, required_size);
		if (!ptr) return ESP_ERR_NO_MEM; buf = ptr;
		nvs_get_str(handle, key, (char*)buf, &required_size);
		if (size) *size = required_size; 
		return 2;
	case NVS_TYPE_BLOB: 
		ret = nvs_get_blob(handle, key, NULL, &required_size);
		if(ret) return ret;
		ptr = realloc(buf, required_size);
		if (!ptr) return ESP_ERR_NO_MEM; buf = ptr;
		nvs_get_blob(handle, key, buf, &required_size);
		if (size) *size = required_size;
		return 3;
		default: return ESP_FAIL;
	}
	return ret;
}

void nvs_test() {
	esp_err_t ret; void* buf = malloc(64); size_t buf_len = 0;
	nvs_stats_t nvs_stats{}; nvs_entry_info_t entry; nvs_iterator_t it = NULL; 
	nvs_get_stats(NULL, &nvs_stats);
	ESP_LOGI("NVS", "Used %u, Free %u, Available %u, All = %u, Namespaces %u entries",
		nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.available_entries, nvs_stats.total_entries, nvs_stats.namespace_count);
	ret = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
	while (ret == ESP_OK) {
		nvs_entry_info(it, &entry); // Can omit error check if parameters are guaranteed to be non-NULL
		nvsApi nvs; ESP_LOGI("NVS", "space '%s'\tkey '%s'\ttype '%u'\n", entry.namespace_name, entry.key, entry.type);
		if(!nvs.begin(entry.namespace_name, NVS_READONLY)) {
			switch (esp_err_t ret = nvsGet(nvs, entry.key, entry.type, buf, &buf_len)) {
			case ESP_OK: 
				DEBUGF("Data = %lu\n", *(uint32_t*)buf); break;
			case 1:
				DEBUGF("Data = %llu\n", *(uint64_t*)buf); break;
			case 2: 
				DEBUGF("Str: %s\n", (char*)buf); break;
			case 3: 
				DEBUGF("Blob size %u: ", buf_len);
				for (size_t i = 0; i < buf_len; i++) { DEBUGF("%02X ", ((byte*)buf)[i]); }; DEBUGLN();
				break;
			default: ESP_LOGE("NVS", "nvsGet()" " 0x%X", ret);
			}
		}
		ret = nvs_entry_next(&it);
	}
	free(buf);
	nvs_release_iterator(it);
}

void nvsErase(cch* except) {
	esp_err_t ret; nvs_entry_info_t entry; nvs_iterator_t it = NULL; 
	ret = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
	while (ret == ESP_OK) {
		nvs_entry_info(it, &entry); // Can omit error check if parameters are guaranteed to be non-NULL
		ESP_LOGI(TAG, "space '%s'\tkey '%s'\ttype '%d'\n", entry.namespace_name, entry.key, entry.type);
		nvsApi nvs; //types: blob 66, str 33
		if(nvs.begin(entry.namespace_name, NVS_READWRITE) == ESP_OK) {
			//if(*reinterpret_cast<uint32_t*>(entry.namespace_name) != *reinterpret_cast<const uint32_t*>("phy"))
			if(except && !strcmp(entry.namespace_name, except)) continue; 
			nvs_erase_all(nvs);
			nvs_commit(nvs);
		}
		ret = nvs_entry_next(&it);
	}
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

/*
void vApplicationIdleHook(void) {
    static bool prev_state = 0;
    if(!sets.patch) {
        const bool val = digitalRead(PIN_LINE);
        if(prev_state != val) { digitalWrite(PIN_RELAY, prev_state = val); }
    }
}*/

void totp_test() {
	byte buf[64]; char str[64];
	const int len = base32_decode(DEF_BLE_PASS, buf, sizeof(buf));
	DEBUG("base32_decode()\n");
	for (size_t i = 0; i < len; i++) { DEBUGF("0x%02X, ", buf[i]); }
	DEBUGLN(); ESP_LOGD(TAG,"length %d\n", len);
	if (len > 0) {
		__unused const time_t now = 12345678;
		const uint32_t totp = TOTPget(buf, len, now);
		base32_encode(buf, len, str, sizeof(str));
		ESP_LOGD(TAG, "%s\nTOTP = %lu; time = %ld; TOTP_TIMESTEP = %lu\n",
			str, totp, now, TOTP_TIMESTEP);
	} 
}

bool wifi_sta_wait_conn() {
	EventBits_t bits = xEventGroupWaitBits(h_group_wifi, WIFI_STA_GOT_IP | WIFI_FAIL_BIT, 
		pdFALSE, pdFALSE, portMAX_DELAY);

	if (bits & WIFI_STA_GOT_IP) {
		ESP_LOGI(TAG, "Connected!");
		return true;
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGW(TAG, "Failed to connect");
		xEventGroupClearBits(h_group_wifi, WIFI_FAIL_BIT);
	} else {
		ESP_LOGW(TAG, "? event bits: 0x%X", bits);
	}
	return false;
}
