#pragma once
#pragma GCC diagnostic ignored "-Wmisleading-indentation" //braces
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"  //switch
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //struct
				#define DEBUG_ENABLE
#define _WANT_USE_LONG_TIME_T
#include "ESP_MAIN.h"
#include "driver/gpio.h"
#ifdef DEBUG_ENABLE
#include "driver/usb_serial_jtag.h"
#endif
#include "esp_random.h"
#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include "mbedtls/md.h"
#include "rom/crc.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
//#include "host/util/util.h"
#include "gap.h"
#include "gatt.h"
#define HEART_RATE_PERIOD (2000 * 1000)
//#include "led.h"

#include "esp_mac.h"
#include "esp_hmac.h"
				//#define CONFIG_FACTORY_FIRMWARE
				#define CONFIG_ADC_LINE
				#define CONFIG_DOMOPHONE
				//#define PERSIST_SETTINGS
//#ifdef CONFIG_FACTORY_FIRMWARE
#include "wifi_api.h"
//#endif
#include <memory>
#include "credentials.h"

static const char* TAG = "MAIN";
#define TMR "TMR"

#define dWrite(x,y) digitalWrite(x, y)
#define dRead(x) digitalRead(x)

#define PIN_RELAY 4
#define PIN_RELAY_GND 3 //unused in code
#define PIN_LED 8

#ifdef CONFIG_ADC_LINE
#define PIN_LINE 1
#define PIN_LED_MASK BIT(PIN_LED)
//#define PIN_RELAY_2 2
#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_3)
#ifdef CONFIG_DOMOPHONE
void lock_open_close(uint8_t state) { dWrite(PIN_RELAY, state); gpio_output_enable((gpio_num_t)PIN_RELAY); }
void lock_open_only(uint8_t state) { if(state) { dWrite(PIN_RELAY, 1); gpio_output_enable((gpio_num_t)PIN_RELAY); } else { gpio_output_disable((gpio_num_t)PIN_RELAY); } }
void (*patch_fun)(uint8_t) = lock_open_only;
#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT)
#define RELAY_PATCH_IMPL() (*patch_fun)(1)
#define RELAY_UNPATCH_IMPL() (*patch_fun)(0)
#define RELAY_DEFAULT_IMPL() RELAY_UNPATCH_IMPL()
#define IO_GET_IMPL() dRead(PIN_RELAY)
#define BLE_GAP_APPEARANCE  GENERIC_HID_TAG
#else
#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT_OUTPUT)
#define RELAY_PATCH_IMPL() dWrite(PIN_RELAY, 1)
#define RELAY_UNPATCH_IMPL() dWrite(PIN_RELAY, 0)
#define RELAY_DEFAULT_IMPL() RELAY_UNPATCH_IMPL()
#define IO_GET_IMPL() dRead(PIN_RELAY)
#define BLE_GAP_APPEARANCE  MOTION_SENSOR_TAG
#endif
#else		//forteza
#define PIN_LINE (PIN_LED)
#define PIN_LED_MASK (0)
#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_0)
#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT_OUTPUT_OD)
#define RELAY_PATCH_IMPL()
#define RELAY_UNPATCH_IMPL()
#define RELAY_DEFAULT_IMPL() dWrite(PIN_RELAY, 1)
#define IO_GET_IMPL() dRead(PIN_LINE)
#define BLE_GAP_APPEARANCE  MOTION_SENSOR_TAG
#endif

#ifdef PIN_RELAY_2
	#define RELAY_2_MASK BIT(PIN_RELAY_2)
	#define RELAY_2_PATCH_IMPL() dWrite(PIN_RELAY_2, 1)
	#define RELAY_2_UNPATCH_IMPL() dWrite(PIN_RELAY_2, 0)
	#define RELAY_2_DEFAULT_IMPL() RELAY_2_UNPATCH_IMPL()
#else
	#define RELAY_2_MASK (0)
	#define RELAY_2_DEFAULT_IMPL()
	#define RELAY_2_PATCH_IMPL()
	#define RELAY_2_UNPATCH_IMPL()
#endif

#define UART_BUF_SIZE (256)
#define CDC_BUF_SIZE (128)
#define UART_PORT UART_NUM_1
#define PATTERN_CHR_NUM    (1)

using String = std::string;
typedef struct { uint8_t patch , ota , flag;  uint8_t crc; } settings;
typedef struct { uint8_t patch , ota; uint16_t crc; } sets_noinit;
static_assert(sizeof(settings) == 4); static_assert(sizeof(sets_noinit) == 4);
typedef enum : uint8_t { ok, ADV, OTA, VALID, NOTIFY_ALARM, NOTIFY_TIME, MAIN, RESTART, } action;
//NIMBLE_HS_STACK_SIZE
//StackType_t xMainStack[4*1024]; StaticTask_t xMainTaskBuffer;
StackType_t	xHostStack[1024*8]; StaticTask_t xHostTaskBuffer;
TaskHandle_t h_main_task, h_nimble_task;
 /*sizeof(StaticTimer_t); 40 sizeof(StaticTask_t); 344*/
esp_timer_handle_t h_timer_patch, h_timer_wifi, h_timer_valid;
__unused esp_netif_t* h_netif_sta;
__unused esp_netif_t* h_netif_ap;
uint8_t pass_key[32] = DEF_BLE_PASS_BASE32;
uint8_t scan_key[32] = DEF_BLE_SCAN_DATA;
uint8_t pass_key_len = DEF_BLE_PASS_LEN; 		static_assert(DEF_BLE_PASS_LEN <= sizeof(pass_key)); //sizeof(DEF_BLE_PASS)-1;
uint8_t scan_key_len = DEF_BLE_SCAN_DATA_LEN;	static_assert(DEF_BLE_SCAN_DATA_LEN <= sizeof(scan_key));

__unused static settings sets;
__NOINIT_ATTR static sets_noinit noinit;

static void bluetooth_init();
static void wifi_init();
__unused static void mainTask(void * = NULL);
__unused static void usb_cdc_task(void *arg);
__unused static void IRAM_ATTR isr_handler();
//__unused static void nimble_host_task(void *);
extern void adv_init(uint16_t = HID_SVC, uint32_t = TIME_ADV_UNITS);
extern void set_cts_unix(time_t now);
static void patch_func(uint64_t = TIMER_PATCH);

#ifndef CONFIG_DOMOPHONE
void wifi_timer_stop() { CHECK_(esp_timer_stop(h_timer_wifi)); }
void wifi_timer_start() { CHECK_(esp_timer_start(h_timer_wifi, TIMER_WIFI)); }
esp_err_t wifi_timer_restart(uint32_t ms) { return esp_timer_start(h_timer_wifi, ms*1000); }
#endif
extern esp_err_t http_server_init();
void restart_request() { xTaskNotify(h_main_task, RESTART, eSetValueWithOverwrite); }
void ota_update_start_cb() { ble_gap_ext_adv_stop(0); ble_gap_disc_cancel(); }
void ble_device_name_set();
void wifi_hostname_set();
int ble_delete_all_bonds();

size_t strtoB(const char* str, uint8_t* buf, size_t buf_len);
template <bool = false, char = 0> int bytes_to_str(const uint8_t* src, char* dest, size_t data_size);
int bytes_to_str_bigend(const uint8_t* src, char* dest, size_t data_size) { return bytes_to_str<true, ' '>(src, dest, data_size) ; };

void check_img_state();
void nvs_sets_read();
void nvs_sets_write(nvsApi nvs = nvsApi(NVS_SPACE_SETTINGS, NVS_READWRITE));
esp_err_t auth_data_save();
void auth_data_read();

inline auto crc_impl(const sets_noinit & buf) {
	constexpr int size = sizeof(settings::crc), len = sizeof(settings) - size;
	return (size == 2) ? crc16_le(0,(uint8_t*)&buf, len) : crc8_le(0,(uint8_t*)&buf, len);
}

inline sets_noinit read_noinit() {
	ESP_LOGD(TAG,"sets_noinit: %08X", *reinterpret_cast<uint32_t*>(&noinit));
	if(crc_impl(noinit) == noinit.crc) {
		return noinit;
	} else { noinit = {}; ESP_LOGW(TAG, "!noinit crc"); };
	return {};
}

inline void write_noinit_ota(uint8_t val) {
	noinit.ota = val;
	noinit.crc = crc_impl(noinit); ESP_LOGD(TAG,"sets_noinit: %08X", *reinterpret_cast<uint32_t*>(&noinit));
}

void patch_func(uint64_t period) { 
#ifndef CONFIG_DOMOPHONE
	if(!sets.patch) { sets.patch = true; 
		nvs_sets_write(); 
	}
#endif
	if(period) { RELAY_PATCH_IMPL(); RELAY_2_PATCH_IMPL(); CHECK_(esp_timer_start(h_timer_patch, period)); }
	else { RELAY_UNPATCH_IMPL(); RELAY_2_UNPATCH_IMPL(); esp_timer_stop(h_timer_patch); }
}

void set_boot_partition(esp_partition_subtype_t);
void set_main_part() { set_boot_partition(ESP_PARTITION_SUBTYPE_APP_OTA_0); }
void ota_rollback_revoke();
void set_main_partition() { write_noinit_ota(0); }

void ble_connect_cb(int status) { adv_init(); };
int ble_disconnect_cb(uint16_t handle) { int rc = clear_connection(handle); adv_init(); return rc; };
int ble_conn_encrypted_cb(uint16_t handle) { int rc = set_encryption(handle); io_on_impl(); adv_init(); return rc; }
void ble_scan_complete_cb() { //ble_scan_init(); 
}
void ble_adv_complete_cb() { if(!sets.patch) { RELAY_2_UNPATCH_IMPL(); } //ble_scan_init(); 
}

void io_on_impl() { patch_func(); }
void io_off_impl() { patch_func(TIMER_PATCH_OFF); }
int io_get_impl() { return IO_GET_IMPL(); }

void timer_patch_off_cb(void *) { 
    RELAY_UNPATCH_IMPL(); RELAY_2_UNPATCH_IMPL();
#ifndef CONFIG_DOMOPHONE
    sets.patch = false;
	nvs_sets_write();
#endif
}

static uint32_t heart_rate;
void update_heart_rate(void) { heart_rate = esp_random(); /*heart_rate = 60 + (uint8_t)(esp_random() % 21); */ }
uint8_t get_heart_rate(void) { return heart_rate; }

int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len);
uint32_t HOTPget(const uint8_t* key, size_t key_len, uint64_t salt);
uint32_t TOTPget(const uint8_t* key, size_t key_len, time_t time = time(NULL));

std::unique_ptr<char[]> task_list(size_t* len = nullptr);
void print_task_list() { DEBUG(task_list().get()); /*DEBUGLN(esp_timer_dump(stdout));*/ };
constexpr uint32_t strlen_const(const char* str) { return __builtin_strlen(str); }

//__unused void print_addr(cbyte* addr) { for (byte i = 5;;i--) { DEBUGF("%02X", addr[i]); if (!i) break; DEBUG(':'); } DEBUGLN(); }

inline void nvsEraseAll(const char *except) {
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
/*
inline void uart_cb() {
	static char buf[64];
	int len = uart_read_bytes(UART_NUM_0, buf, sizeof(buf)-1, pdMS_TO_TICKS(0));
    if(len < 0) return;
	//buf[len] = '\0';
	//ESP_LOGI("uart", "%u bytes" ,len);
	switch (*buf) {
	case 'P': print_task_list(); break;
	case 'R': vTaskDelay(1); esp_restart(); break;
	case 'D': ble_delete_all_bonds(); break;
	//default: Serial.write(buf, len);//Serial.write('\n');break;
	}
	//if(!strcmp(buf, "P")) {print_task_list();}
	//else if(!strcmp(buf, "R")) {xTaskNotify(main_handle, RESTART, eSetValueWithOverwrite);}
}
*/
/*
void vApplicationIdleHook(void) {
    static bool prev_state = 0;
    if(!sets.patch) {
        const bool val = digitalRead(PIN_LINE);
        if(prev_state != val) { digitalWrite(PIN_RELAY, prev_state = val); }
    }
}*/

inline void totp_test() {
	uint8_t buf[64]; char str[64];
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
