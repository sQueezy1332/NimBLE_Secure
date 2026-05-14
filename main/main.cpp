#include "credentials.h"
#include "main.h"
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
extern "C" void app_main() {
	nvs_init();
	nvs_read_sets();
	//read_noinit();  //0x253D7465 == crc8 //609862 //570586
#ifdef DEBUG_ENABLE
	pinMode(13, OUTPUT);
	delay(3000);
	ESP_LOGI(TAG, "Compiled: " __TIMESTAMP__);
	printHeapInfo();
	esp_log_level_set("*", ESP_LOG_DEBUG);
	esp_log_level_set("nvs", ESP_LOG_INFO);esp_log_level_set("wifi", ESP_LOG_INFO);
	esp_log_level_set("event", ESP_LOG_INFO);esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);
	totp_test();
	nvs_test("cal_data");
	{uint8_t mac[8]; esp_efuse_mac_get_default(mac);ESP_LOGD(TAG, "EfuseMac() " MACSTR, MAC2STR(mac));}
	xTaskCreate(usb_cdc_task, "cdc", 4096, &uartBuffer, 5, nullptr);
										
	//extern void uart_init(); uart_init();
	//auto heart = esp_timer_new([](void*){ digitalToggle(12);}); esp_timer_start_periodic(heart, HEART_RATE_PERIOD);
#else
static_assert(!_ESP_LOG_ENABLED(1)); static_assert(CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT);
static_assert(!configGENERATE_RUN_TIME_STATS);
#endif
	RELAY_DEFAULT_IMPL(); RELAY_2_DEFAULT_IMPL();//pinMode(PIN_RELAY, GPIO_MODE_INPUT_OUTPUT_OD);
   	{const gpio_config_t conf = { (BIT(PIN_RELAY) | RELAY_2_MASK | PIN_LED_MASK), GPIO_MODE_RELAY_IMPL };
	ESP_ERROR_CHECK(gpio_config(&conf));}
	gpio_set_drive_capability((gpio_num_t)PIN_RELAY ,DRIVE_CAP_IMPL);
//#ifdef CONFIG_FACTORY_FIRMWARE
	RELAY_PATCH_IMPL(); RELAY_2_PATCH_IMPL(); //esp_rom_get_reset_reason()
	if(read_noinit()) { wifi_init(); return; }
//#else
#ifdef CONFIG_GENERIC_PATCHER
	conf.pin_bit_mask = BIT(PIN_LINE); conf.mode = GPIO_MODE_INPUT;
	ESP_ERROR_CHECK(gpio_config(&conf));
#else
	gpio_pullup_en((gpio_num_t)PIN_LINE);
	attachInterrupt(PIN_LINE, isr_handler, GPIO_INTR_ANYEDGE); enableInterrupt(PIN_LINE);
#endif
	h_timer_patch = esp_timer_new(timer_patch_off_cb); assert(h_timer_patch);
	read_auth_data(); ESP_LOGD(TAG, "pass_key_len %u, scan_key_len: %u\n", pass_key_len, scan_key_len);//DEBUGLN(pass_key);
	ESP_ERROR_CHECK(nimble_port_init());
	gap_init();
	gatt_svc_init();
	ble_hs_cfg_init();
	h_nimble_task = xTaskCreateStaticPinnedToCore((TaskFunction_t)nimble_port_run,
	"nimble", sizeof(xHostStack), NULL, (configMAX_PRIORITIES - 4), xHostStack, &xHostTaskBuffer, NIMBLE_CORE);
	#if PIN_LED_MASK
	delay(1000); dWrite(PIN_LED, 1);
	#endif
	mainTask();
//#endif
}

static void wifi_init() {
	ESP_LOGI(TAG, "Run factory firmware\n");
	h_timer_wifi = esp_timer_new([](void*){ 
		ESP_LOGW(TAG,"TIMER_WIFI ms %lu",(uint32_t)(esp_timer_period(h_timer_wifi) / 1000));
		ESP_LOGI(TAG, "FreeHeap %lu", getFreeHeap()); esp_restart();});
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(h_timer_wifi, TIMER_WIFI)); 
	ESP_LOGD(TAG,"TIMER_WIFI ms %lu", uint32_t(TIMER_WIFI / 1000));
#ifdef STATION_MODE
	wifi_setup_default(WIFI_MODE_STA, WIFI_STORAGE_RAM); h_netif_sta = wifi_init_sta();
#else
	wifi_setup_default(WIFI_MODE_AP, WIFI_STORAGE_RAM); h_netif_ap = wifi_init_ap();
#endif
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(http_server_init());
	//ap_set_dns_addr(h_netif_ap,h_netif_sta);  
#ifdef DEBUG_ENABLE
	ESP_LOGI(TAG, "WiFi started, waiting for connection...");
	if(!wifi_sta_wait_conn()) return;
	esp_log_level_set("wifi", ESP_LOG_DEBUG);
#endif
}

static void usb_cdc_task(void *arg) {
	static const char* TAG = "CDC";
	static uint8_t Buffer[CDC_BUF_SIZE];
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = {CDC_BUF_SIZE, CDC_BUF_SIZE};
	int ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);ESP_ERROR_CHECK(ret);
	for (uint8_t* data = Buffer;;) {
        int len = usb_serial_jtag_read_bytes(data, (CDC_BUF_SIZE - 1), portMAX_DELAY);
        if (!len) continue;
		if (len <= 3) {
			switch (*data) {
				case 'T': print_task_list(); continue;
				case 'D': ret = ble_store_clear();
					ESP_LOGI(TAG, "ble_store_clear %d"); continue;
				case 'V': ESP_LOGI(TAG, "VALID"); revoke_ota_rollback();
					continue;
			}
		}
		ESP_LOG_BUFFER_HEX(TAG, data, len);
		//usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(20));
		//data[len] = '\0'; ESP_LOGW(TAG, "%s", data);
    }
}

static void mainTask(void * arg) {
	h_main_task = xTaskGetCurrentTaskHandle();
//#ifdef	DEBUG_ENABLE
	printHeapInfo(); //ble_store_clear();
	for(;;) {
		switch (ulTaskNotifyTake(1,portMAX_DELAY)) { 
		//case OTA: vTaskSuspend(ble_handle);
			//set_boot_partition(ESP_PARTITION_SUBTYPE_APP_FACTORY);
			//ESP_LOGI(TAG, "reboot to FACTORY...");//esp_restart(); break;
		//case RESTART: vTaskDelay(1); esp_restart(); break;
		//case NOTIFY: send_alarm_notify(); break;
		//case VALID: esp_ota_mark_app_valid_cancel_rollback(); break;
		case NOTIFY_ALARM: send_alarm_notify(); break;
		case NOTIFY_TIME: break;
		} 
	}
//#endif
}

static void isr_handler() {
	static int prev_state = 1;
	const int now = dRead(PIN_LINE); 
	if(now) { ESP_DRAM_LOGI("ISR", "1"); }
	else{ ESP_DRAM_LOGI("ISR", "0"); }
	if(prev_state != now) {
		prev_state = now;
		if(sets.patch == false) {
			dWrite(PIN_RELAY, now); 
		}
		else if(need_notify()) { xTaskNotifyFromISR(h_main_task, NOTIFY_ALARM, eNoAction, NULL); };
	#if PIN_LED_MASK
		dWrite(PIN_LED, now);
	#endif
	}
}

void ble_delete_all_peers() {
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	ble_addr_t peer_id_addrs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
	int num_peers = 0;
   	ble_store_util_bonded_peers(peer_id_addrs, &num_peers, sizeof(peer_id_addrs));
	ESP_LOGI(TAG, "num_peers %d", num_peers);
	if(num_peers > MYNEWT_VAL(BLE_STORE_MAX_BONDS)) { num_peers = MYNEWT_VAL(BLE_STORE_MAX_BONDS); }
	for(size_t i = 0; i < num_peers; i++) {
		//print_addr((peer_id_addrs[i].val));
		/*if(except && (*reinterpret_cast<uint64_t*>(except) != 0) &&
			!ble_addr_cmp(&except->arr, &peer_id_addrs[num_peers])) continue;*/
		ble_store_util_delete_peer(&peer_id_addrs[i]); 
	}
#endif
}
/*
bool read_bonded_mac() {
	byte crc = crc8_le(0, (byte*)&bonded_addr, 7);
	if(bonded_addr.crc == crc) return true;
	bonded_addr = {};
	//bonded_addr.crc = crc8_le(0, (byte*)&bonded_addr, 7);
	return false;
}*/

void nvs_write_sets(nvsApi nvs) {
	sets.crc = crc_impl(sets); 
	ESP_LOGD("NVS","patch %u, upd %u, val %u, crc %02X", sets.patch, sets.upd, sets.flag, sets.crc);
	CHECK_VOID(nvs_set_u32(nvs, NVS_KEY_OTA, *reinterpret_cast<uint32_t*>(&sets)));
	CHECK_(nvs_commit(nvs));
}

void nvs_read_sets() {
	if (img_state() == ESP_OTA_IMG_PENDING_VERIFY) {
		h_timer_valid = esp_timer_new([](void*){
			ESP_LOGW(TAG, "TIMER_OTA_VALID min %lu", uint32_t(TIMER_OTA_VALID / 1000000)); 
			esp_restart();});
		assert(h_timer_valid); esp_timer_start_once(h_timer_valid, TIMER_OTA_VALID);
	}
	nvsApi nvs(NVS_SPACE_SETS, NVS_READWRITE);
	auto ret = nvs_get_u32(nvs, NVS_KEY_OTA, reinterpret_cast<uint32_t*>(&sets));
	if (ret == ESP_OK) {
		auto crc =  crc_impl(sets);
		if (crc == sets.crc) {
			if(sets.patch) { ESP_LOGI(TAG, "PATCH_ON"); patch_func();  }
			else { /* timer_patch_off_cb((void*)1); */ }; //dont write
			return;
		} else { ESP_LOGW(TAG, "crc %u sets.crc %u", crc, sets.crc); };
	} else { CHECK_(ret); } //alarm_on(false);
	sets = {}; 
__unused ota: nvs_write_sets(nvs);
}

void read_auth_data() {
	nvsApi handle; size_t required_size;
	CHECK_VOID(handle.begin(NVS_SPACE_SETS, NVS_READONLY));
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_BLE_PASS, NULL, &required_size));
	if(required_size < 16 || required_size > sizeof(pass_key)) return;
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_BLE_PASS, pass_key, &required_size));
	pass_key_len = required_size;
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_SCAN_DATA, NULL, &required_size));
	if(required_size < 8 || required_size > sizeof(scan_key)) return;
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_SCAN_DATA, scan_key, &required_size));
	scan_key_len = required_size;
}

esp_err_t save_auth_data() {
	int err = ESP_FAIL;
	if (pass_key_len < 8 || scan_key_len < 8) { ESP_LOGW(TAG, "!Auth"); 
		err = (uint8_t)pass_key_len; ((uint8_t*)&err)[1] = scan_key_len; err |= BIT31;
		return err; 
	}
	nvsApi handle; 
	CHECK_RET(handle.begin(NVS_SPACE_SETS, NVS_READWRITE));
	if(pass_key_len > 0) {
		err = nvs_set_blob(handle, NVS_KEY_BLE_PASS, pass_key, pass_key_len);
		if(err) {ESP_LOGE(TAG, "%d", err);}
	}
	if(pass_key_len > 0) {
		err = nvs_set_blob(handle, NVS_KEY_SCAN_DATA, scan_key, scan_key_len);
		if(err) {ESP_LOGE(TAG, "%d", err);}
	}
	if(err == ESP_OK) 
		return nvs_commit(handle);
	return err;
}

void set_boot_partition(esp_partition_subtype_t type) {
	auto i = esp_partition_find(ESP_PARTITION_TYPE_APP, type, NULL);
	for (;i != NULL; i = esp_partition_next(i)) {
		const esp_partition_t* part = esp_partition_get(i);
		if(part->subtype == type) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_set_boot_partition(part)); 
			break;
		}
	}
	esp_partition_iterator_release(i);
}

void revoke_ota_rollback() {
	esp_ota_mark_app_valid_cancel_rollback();
	if (!h_timer_valid) return;
	esp_timer_stop(h_timer_valid); esp_timer_delete(h_timer_valid); h_timer_valid = NULL;
}

void parse_adv_cb(cbyte* data, byte len) {
	//cbyte len = *data, type = data[1], size = scan_key_len;
	if(len == scan_key_len && !memcmp(data, scan_key, scan_key_len)) {
		//extern int RSSI;NIMLOG("\nRSSI:\t\t%i\n", RSSI);
		ESP_LOGW(TAG, "PASS!");
		RELAY_2_PATCH_IMPL();
		ble_gap_disc_cancel();
		adv_init();
	}
	/* if(type == COMPLETE_NAME  || type == SHORT_NAME ) {
		if ((len - 1 == size) && !memcmp(data +1, scan_key, size)) {
			ESP_LOGW(TAG, "PASS!");
			ble_gap_disc_cancel(); adv_init();
		}
	} *///else if (type == UUID32_DATA && len == 9 && *(uint32_t*)(&data[++i]) == ...) { *(uint32_t*)(&data[i+=4])  }
}

void parse_rx_data(const ble_gap_event* event) {
	//extern ble_gap_conn_desc desc;
	const os_mbuf* buf = event->notify_rx.om;
	enum { 
		OFFSET = DEF_CMD_OFFSET,
		OTA_KEY,
		RESTART_KEY,
		VALID_KEY,
		SAVE_MAC,
		NVS_ERASE_ALL,
		NVS_ERASE_ALL_EXC,
		BLE_STORE_CLEAR
	};
	if(buf->om_len != 5) return;
	if(*reinterpret_cast<uint32_t*>(buf->om_data) != DEF_CMD_PASS) return;
	//auto val = *reinterpret_cast<decltype(wifi_key)*>(buf->om_data);
	uint8_t val = buf->om_data[4];
	switch (val) {
	case OTA_KEY:	//ESP_ERROR_CHECK(nimble_port_stop());
		write_noinit(1);
		//set_boot_partition(ESP_PARTITION_SUBTYPE_APP_FACTORY);
		ESP_LOGI(TAG, "reboot to FACTORY...");
	case RESTART_KEY: ESP_LOGI(TAG, "RESTART"); esp_restart(); 
		break;//xTaskNotify(main_handle, RESTART, eSetValueWithOverwrite); break;
	case VALID_KEY: revoke_ota_rollback();
		break;
	case SAVE_MAC: save_bonding(event->notify_rx.conn_handle);
		break;
	case NVS_ERASE_ALL: nvsEraseAll(nullptr);
		break;
	case NVS_ERASE_ALL_EXC: nvsEraseAll("phy");
		break;
	case BLE_STORE_CLEAR: ble_store_clear();
		break;
	case OFFSET: DEBUG(task_list().get());
		break;
	default: ESP_LOGW(TAG, "os_mbuf 0x%02X", val);
	}
}

std::unique_ptr<char[]> task_list(size_t* len) {
	const size_t num = uxTaskGetNumberOfTasks(), heap = getFreeHeap();
	const size_t buf_size = ALIGN_TO_16(((num * 40)  * (configGENERATE_RUN_TIME_STATS ? 2 : 1)));
	std::unique_ptr<char[]> ptr(new char[buf_size]); 
	char* str = ptr.get(); if(!str) return ptr;//931205
	vTaskList(str); 
	size_t i = strlen(str);
	i += sprintf(&str[i] , "FreeHeap: %u", heap); //i += strlen(str);
	strcpy(&str[i],"\n\n"); i +=2;
#if configGENERATE_RUN_TIME_STATS
	vTaskGetRunTimeStats(&str[i]); i += strlen(&str[i]);
#endif
	ESP_LOGD(TAG,"NumberOfTasks: %u, buf_size: %u, strlen: %u", num, buf_size, i);
	if(len) *len = i;
	return ptr;
}

template <bool big_endian, char separ> int bytes_to_str(const byte* src, char* dest, size_t data_size) {
	if(data_size == 0) return 0; 
	int i; char inc; char* ptr = dest;
	if(big_endian){ i = 0; --data_size; inc = 1;}
	else { i = data_size-1; data_size = 0; inc = -1;}
	for (;;i += inc) {
		for (int shift = 4;; shift = 0) {
			byte nibble = (src[i] >> shift) & 0xF;
			*ptr++ = nibble < 10 ? nibble ^ 0x30 : nibble + ('A' - 10);
			if (shift == 0) break;
		} 
		if (i == data_size) break;
		if(separ) { *ptr++ = separ; }
	}
	*ptr = '\0';
	return ptr - dest;
}

size_t strtoB(const char* src, uint8_t *dest, size_t buf_len) { 
	size_t i = 0;
	if (*src) {
		for (unsigned result = 0,shift = 0;; ++src) {
			char temp = *src;
			switch (temp) {
				case '0'... '9':
					result <<= 4;
					result |= (temp ^ 0x30); break;
				case 'A'... 'F':
					result <<= 4;
					result |= temp - 55; break;
				case 'a' ...'f':
					result <<= 4;
					result |= temp - 87; break;
				case '\0': if(shift) dest[i++] = result; 
					return i;
				default: if(!shift) continue;
						else goto rdy;
			}
			if (shift) {
rdy:            dest[i] = result;
				i++;
				if (i >= buf_len) break;
				result = 0; shift = 0;
			} else { shift = 1; };
		}
	} //
	return i;
}

void set_ble_device_name() {
	static_assert(!MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)); static_assert(MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH) >=16);
	constexpr int name_len = sizeof(MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME))-1; static_assert(name_len >= 7);
	char* name = const_cast<char*>(ble_svc_gap_device_name()); byte mac[8];
	*(name += name_len) = '-';
	CHECK_VOID(esp_read_mac(mac, ESP_MAC_BT));
	//bytes_to_str<true, 0>(name+1, mac + 3, 3); //621263
	sprintf(name + 1,"%02X%02X%02X", mac[3],mac[4],mac[5]); //621165
}

uint32_t generate_salt() {
	static TickType_t last_change = __INT32_MAX__; 
	static time_t salt; //ESP_LOGD(TAG, "change_device_name()");
	uint32_t sec = xTaskGetTickCount();
	if(sec - last_change > pdMS_TO_TICKS(TIME_CHANGE_PIN)) {
#if !_ESP_LOG_ENABLED(3)
		if(last_change != __INT32_MAX__)//{ ble_delete_all_peers(&bonded_addr); } 
		ble_gap_terminate();
#endif
		last_change = sec;
		salt = time(NULL); 
		//if(salt < 1777740000) salt = 0;
		pincode = TOTPget(pass_key, pass_key_len, salt); 
		ESP_LOGI(TAG, "NEW PIN: %lu\tTime: %lu", pincode, salt);
	}
	return salt;
}

uint32_t TOTPget(const uint8_t* key, size_t key_len, time_t time) {
	return HOTPget(key, key_len, time / TOTP_TIMESTEP);
}

uint32_t HOTPget(const uint8_t* key, size_t key_len, uint64_t salt) {
	uint8_t* const pSalt = (uint8_t*)&salt;
	uint8_t hash[20];  uint32_t result;
	swap_in_place(pSalt, sizeof(salt)); //salt = ntohll(salt);
	int ret = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA1),key,key_len,pSalt,sizeof(salt), hash);
	if(ret != 0) { ESP_LOGE(TAG, "HOTPget %d", ret); return 0; }
	for (int i = 0, offset = hash[19] & 0xF; i < 4; ++i) {
		((uint8_t*)&result)[3 - i] = hash[offset + i];
	}
	result = (result & 0x7FFFFFFF) % 1000000;
	return result;
}

/**
 * Base32 decoder
 * From https://github.com/google/google-authenticator-libpam/blob/master/src/base32.c
 * @param encoded Encoded text
 * @param result Bytes output
 * @param buf_len Bytes length
 * @return -1 if failed, or length decoded
 */

int base32_decode(const char* encoded, uint8_t* result, size_t buf_len) {
	if (!encoded || !result) { return -1; }
	// Base32's overhead must be at least 1.4x than the decoded bytes, so the result output must be bigger than this
	/* size_t expect_len = ceil(strlen(encoded) / 1.6);
	if (buf_len < expect_len) {
		ESP_LOGE(TAG, "uartBuffer length is too short, only %u, need %u", buf_len, expect_len);
		return -1;
	} */
	int bits_left = 0, count = 0; 
	for (unsigned buffer = 0; count < buf_len && *encoded; ++encoded) {
		uint8_t ch = *encoded;
		buffer <<= 5;
		// Deal with commonly mistyped characters
		switch (ch) {
			case ' ': case '_': case '\t':case '\r':case '\n': case '-': case '+':
				continue;
			case '=': return count;
			/* case '0': ch = 'O';break;
			case '1': ch = 'L';break;
			case '8': ch = 'B';break; */
		}
			// Look up one base32 digit
		if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z')) {
			ch = (ch & 0b11111) - 1;
		} else if (ch >= '2' && ch <= '7') {
			ch -= '2' - 26;
		} else return -2; //error
		buffer |= ch;
		bits_left += 5;
		if (bits_left >= 8) {
			result[count++] = buffer >> (bits_left - 8);
			bits_left -= 8;
		}
	}
	if (count < buf_len) { result[count] = '\0';} 
	return count;
}

int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len) {
	if (!length ||  length > (1 << 16)) { return -1; }
	unsigned buffer = data[0];
	int count = 0, next = 1, bits_left = 8;
		while (count < encode_len && (bits_left > 0 || next < length)) {
			if (bits_left < 5) {
				if (next < length) {
					buffer <<= 8;
					buffer |= data[next++] & 0xFF;
					bits_left += 8;
				} else {
					unsigned pad = 5 - bits_left;
					buffer <<= pad;
					bits_left += pad;
				}
			}
			uint8_t index = 0x1F & (buffer >> (bits_left - 5));
			bits_left -= 5;
			result[count++] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567"[index];
		}
		if (count < encode_len) { result[count] = '\0'; }
	return count;
}
