#pragma once
#pragma GCC diagnostic ignored "-Wmisleading-indentation" //braces
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"  //switch
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //struct
#define _WANT_USE_LONG_TIME_T
				//#define CONFIG_ADC_LINE
				#define CONFIG_DOMOPHONE
#include "ESP_MAIN.h"
#include "driver/gpio.h"
#include "esp_random.h"
#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include "mbedtls/md.h"
#include "rom/crc.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "gap.h"
#include "gatt.h"
#include "esp_mac.h"
#include "esp_hmac.h"
						
#include "wifi_api.h"
#include <memory>
#include "credentials.h"
#include "tlsf_block_functions.h"
#define nvs_sets_t u64
#define nvs_read_sets_impl(...) CONCAT(nvs_get_, u64)(__VA_ARGS__)
#define nvs_write_sets_impl(...) CONCAT(nvs_set_, u64)(__VA_ARGS__)
#ifdef DEBUG_ENABLE
	#include "led.h"
	#define DEBUG_LED(...) led_set(__VA_ARGS__)
	#define DEBUG_LED_GET() get_led_state()
#else 
	#define DEBUG_LED_GET() (0)
	#define DEBUG_LED(...)
	static_assert(!_ESP_LOG_ENABLED(1)); static_assert(CONFIG_BT_NIMBLE_LOG_LEVEL_NONE); 
	static_assert(CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT); static_assert(CONFIG_COMPILER_OPTIMIZATION_PERF);
	//static_assert(!configGENERATE_RUN_TIME_STATS); 
	static_assert(!configUSE_TIME_SLICING);
	static_assert(CONFIG_BOOTLOADER_WDT_DISABLE_IN_USER_CODE); static_assert(CONFIG_BOOTLOADER_WDT_TIME_MS >= 15000);
#endif

static const char* TAG = "MAIN";
#define TMR "TMR"
#define HEART_RATE_PERIOD (2000 * 1000)
#define dWrite(x,y) digitalWrite(x, y)
#define dRead(x) digitalRead(x)

#define PIN_RELAY 4
#define PIN_RELAY_GND 3 //unused in code
#define PIN_LED 8
#define BLE_GAP_APPEARANCE  GENERIC_HID_TAG
#ifdef CONFIG_ADC_LINE
	#define  1
	#define PIN_LEDPIN_LINE_MASK BIT(PIN_LED)
	//#define PIN_RELAY_2 2
	#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_3)
	#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT_OUTPUT)
	#define RELAY_PATCH_IMPL() dWrite(PIN_RELAY, 1)
	#define RELAY_UNPATCH_IMPL() dWrite(PIN_RELAY, 0)
	#define RELAY_DEFAULT_IMPL() RELAY_UNPATCH_IMPL()
	#define IO_GET_IMPL() dRead(PIN_RELAY)
#else
	#define PIN_LINE (PIN_LED)
	#define PIN_LED_MASK (0)
		#ifdef CONFIG_DOMOPHONE
static void lock_open_close(uint8_t state) { dWrite(PIN_RELAY, state); gpio_output_enable((gpio_num_t)PIN_RELAY); }
static void lock_open_only(uint8_t state) { if(state) { dWrite(PIN_RELAY, 1); gpio_output_enable((gpio_num_t)PIN_RELAY); } else { gpio_output_disable((gpio_num_t)PIN_RELAY); } }
static void (*patch_impl)(uint8_t) = lock_open_only;
	#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_3)
	#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT)
	#define RELAY_PATCH_IMPL() (*patch_impl)(1)
	#define RELAY_UNPATCH_IMPL() (*patch_impl)(0)
	#define RELAY_DEFAULT_IMPL() RELAY_UNPATCH_IMPL()
	#define LINE_INTERRUPT_MODE GPIO_INTR_NEGEDGE
	#define IO_GET_IMPL() dRead(PIN_RELAY)
		#else
	#define DRIVE_CAP_IMPL (GPIO_DRIVE_CAP_0)
	#define GPIO_MODE_RELAY_IMPL (GPIO_MODE_INPUT_OUTPUT_OD)
	#define RELAY_PATCH_IMPL()
	#define RELAY_UNPATCH_IMPL()
	#define RELAY_DEFAULT_IMPL() dWrite(PIN_RELAY, 1)
	#define LINE_INTERRUPT_MODE GPIO_INTR_ANYEDGE
	#define IO_GET_IMPL() dRead(PIN_LINE)
		#endif
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

#define CDC_BUF_SIZE (128)

using String = std::string;
typedef struct { uint8_t patch , ota , flag;  uint8_t crc; uint32_t reserved; } settings;
typedef struct { int8_t rssi , ota; uint16_t crc; } sets_noinit;
static_assert(sizeof(sets_noinit) == 4); static_assert(sizeof(settings) == sizeof(nvs_sets_t)); 
typedef enum notify_enum : uint8_t 
{ ok, ADV, OTA, VALID, NOTIFY_IO, NOTIFY_TIME, ACCESS_BLE, EXIT_BUTTION, ADV_RESTART, MAIN, RESTART, DEBUG_LED_OFF} action_t;

StackType_t	xHostStack[1024*8]; StaticTask_t xHostTaskBuffer;
TaskHandle_t h_main_task, h_nimble_task;
 /*sizeof(StaticTimer_t); 40 sizeof(StaticTask_t); 344*/
esp_timer_handle_t h_timer_img_valid;
esp_timer_handle_t h_timer_patch; 
esp_timer_handle_t h_timer_wifi;
__unused esp_timer_handle_t h_timer_rssi;
extern esp_netif_t* h_netif_sta;
extern esp_netif_t* h_netif_ap;
uint8_t pass_key[32] = DEF_BLE_PASS_BASE32;
uint8_t scan_key[32] = DEF_BLE_SCAN_DATA;
uint8_t pass_key_len = DEF_BLE_PASS_LEN; 		static_assert(DEF_BLE_PASS_LEN <= sizeof(pass_key)); //sizeof(DEF_BLE_PASS)-1;
uint8_t scan_key_len = DEF_BLE_SCAN_DATA_LEN;	static_assert(DEF_BLE_SCAN_DATA_LEN <= sizeof(scan_key));

static settings sets;
__unused __NOINIT_ATTR static sets_noinit noinit;

static void bluetooth_init();
static void wifi_init();
__unused static void mainTask(void * = NULL);
__unused static void IRAM_ATTR isr_handler();
__unused void usb_cdc_task(void *arg);
static void patch_timer(uint64_t = TIMER_PATCH);
static void IRAM_ATTR timer_patch_off_cb(void *);
//__unused static void nimble_host_task(void *);
extern void set_cts_unix(time_t now);
extern void adv_init(uint16_t = HID_SVC, uint32_t = TIME_ADV_UNITS, bool legacy = true);
static void adv_restart_ev_cb(struct ble_npl_event *ev) { adv_init(); }
	
#ifndef CONFIG_DOMOPHONE
void wifi_ap_connected_cb(wifi_event_ap_staconnected_t*) { };
void wifi_ap_disconnected_cb(wifi_event_ap_stadisconnected_t*) { };
void wifi_disconnected_cb(wifi_event_sta_disconnected_t*) { CHECK_(esp_timer_start(h_timer_wifi, TIMER_WIFI)); }
void wifi_connected_cb(wifi_event_sta_connected_t*) { CHECK_(esp_timer_stop(h_timer_wifi)); }
void ota_update_start_cb() { ble_gap_ext_adv_stop(0); ble_gap_disc_cancel(); }

void ble_scan_complete_cb() { ble_scan_init(); }
void ble_adv_complete_cb() { if(!sets.patch) { RELAY_2_UNPATCH_IMPL(); } ble_scan_init(); }
#else
void ota_update_start_cb() { /*reconfigure_task_wdt(30);*/ ble_gap_ext_adv_stop(0); portYIELD(); } 
void ota_update_end_cb() { /*reconfigure_task_wdt(CONFIG_ESP_TASK_WDT_TIMEOUT_S);*/ adv_init(); }
void ble_scan_complete_cb() { }
static struct ble_npl_event adv_restart_event;
void ble_adv_complete_cb() { ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &adv_restart_event); }

void sntp_setup();
void check_distance_by_rssi();
handle_mask_t tracked_conn_mask = 0;
#endif

extern esp_err_t http_server_init();
void restart_request() { xTaskNotify(h_main_task, RESTART, eSetValueWithOverwrite); }
void patch_request_cb() { patch_timer(); }
esp_err_t wifi_timer_restart(uint32_t ms) { return esp_timer_start(h_timer_wifi, ms*1000); }

void ble_device_name_set();
void wifi_hostname_set();
int ble_delete_all_bonds();
void ble_connect_cb(int status) { ble_adv_complete_cb(); }
int ble_disconnect_cb(uint16_t handle) { ble_adv_complete_cb(); return clear_connection(handle); }


size_t strtoB(const char* str, uint8_t* buf, size_t buf_len);
template <bool = false, char = 0> int bytes_to_str(const uint8_t* src, char* dest, size_t data_size);
int bytes_to_str_bigend(const uint8_t* src, char* dest, size_t data_size) { return bytes_to_str<true, ' '>(src, dest, data_size) ; };

void nvs_sets_read();
void nvs_sets_write(nvsApi nvs = nvsApi(NVS_SPACE_SETTINGS, NVS_READWRITE));
inline auto crc_impl(const sets_noinit & buf = noinit) {
	constexpr int size = sizeof(sets_noinit::crc), len = sizeof(sets_noinit) - size;
	return (size == 2) ? crc16_le(0,(uint8_t*)&buf, len) : crc8_le(0,(uint8_t*)&buf, len);
}
sets_noinit read_noinit();
void write_noinit_ota(uint8_t val);
esp_err_t auth_data_save();
void auth_data_read();
void nvsEraseAll(const char *except);

void set_boot_partition(esp_partition_subtype_t);
void set_main_part() { set_boot_partition(ESP_PARTITION_SUBTYPE_APP_OTA_0); }
void ota_rollback_revoke();

void io_on_cb() { patch_timer(); }
void io_off_cb() { patch_timer(TIMER_PATCH_OFF); }
int io_get_cb() { return IO_GET_IMPL(); }

static uint32_t heart_rate;
void update_heart_rate(void) { heart_rate = esp_random(); /*heart_rate = 60 + (uint8_t)(esp_random() % 21); */ }
uint8_t get_heart_rate(void) { return heart_rate; }

int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len);
uint32_t HOTPget(const uint8_t* key, size_t key_len, uint64_t salt);
uint32_t TOTPget(const uint8_t* key, size_t key_len, time_t time = time(NULL));

std::unique_ptr<char[]> task_list(size_t* len = nullptr);
void print_task_list() { __unused size_t len = 0; DEBUG("%.*s", len, task_list(&len).get()); /*DEBUGLN(esp_timer_dump(stdout));*/ };
constexpr uint32_t strlen_const(const char* str) { return __builtin_strlen(str); }

#if CONFIG_HEAP_USE_HOOKS
#include "esp_heap_caps.h"
void esp_heap_trace_alloc_hook(void* ptr, size_t size, uint32_t caps) {
	ESP_DRAM_LOGD("alloc_hook", "0x%08x, size %lu, caps 0x%04x", ptr, size, caps);
}
void esp_heap_trace_free_hook(void* ptr) {
	ESP_DRAM_LOGD("free_hook", "0x%08x", ptr);
}
#endif

extern "C" void vApplicationIdleHook(void) {
#if	CONFIG_BOOTLOADER_WDT_DISABLE_IN_USER_CODE
	rtc_wdt_feed();
#endif
}

void set_main_partition() { write_noinit_ota(0); }

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

//__unused void print_addr(cbyte* addr) { for (byte i = 5;;i--) { DEBUGF("%02X", addr[i]); if (!i) break; DEBUG(':'); } DEBUGLN(); }

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