#include "credentials.h"
#include "main.h"

extern "C" void app_main() {
	nvs_init();
	ESP_LOGI(TAG, "Compiled: " __TIMESTAMP__);
	//read_noinit();  //0x253D7465 == crc8 //609862 //570586
	//ESP_LOGI(TAG,"%lu", ESP.getFreeHeap());nvs_test();ESP_LOGI(TAG,"%lu", ESP.getFreeHeap());
#ifdef DEBUG_ENABLE
		//printHeapInfo();
	esp_log_level_set("*", ESP_LOG_DEBUG);
	esp_log_level_set("nvs", ESP_LOG_INFO);
	esp_log_level_set("wifi", ESP_LOG_INFO);
	esp_log_level_set("event", ESP_LOG_INFO);
	esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);
	totp_test();
	ESP_LOGI(TAG, "FreeHeap() %lu", getFreeHeap());
	{uint8_t mac[8]; esp_efuse_mac_get_default(mac);ESP_LOGD(TAG, "EfuseMac() " MACSTR, MAC2STR(mac));}
	//vTaskDelay(1); volatile int var = 0; assert(var);
	//auto heart = esp_timer_new([](void*){ digitalToggle(12);}); esp_timer_start_periodic(heart, HEART_RATE_PERIOD);
	//pinMode(12, OUTPUT);//led_init();
	//nvs_test();
	//nvsErase(NULL);
#endif
	
	RELAY_DEFAULT_IMPL(); RELAY_2_DEFAULT_IMPL();//pinMode(PIN_RELAY, GPIO_MODE_INPUT_OUTPUT_OD);
   	const gpio_config_t &&conf = { (BIT(PIN_RELAY) | RELAY_2_MASK | PIN_LED_MASK), GPIO_MODE_RELAY_IMPL };
	ESP_ERROR_CHECK(gpio_config(&conf));
	gpio_set_drive_capability((gpio_num_t)PIN_RELAY,DRIVE_CAP_IMPL);
//#ifdef CONFIG_FACTORY_FIRMWARE
	RELAY_PATCH_IMPL(); RELAY_2_PATCH_IMPL();
	//wifi_init();return;
//#else
#ifdef CONFIG_GENERIC_PATCHER
	conf.pin_bit_mask = BIT(PIN_LINE); conf.mode = GPIO_MODE_INPUT;
	ESP_ERROR_CHECK(gpio_config(&conf));
#else
	//attachInterrupt(PIN_LINE, isr_handler, GPIO_INTR_ANYEDGE);
	//enableInterrupt(PIN_LINE);
	
	//nimble_port_stop();
#endif
	timer_patch = esp_timer_new(timer_patch_off_cb);
	assert(timer_patch);
	nvs_read_sets();
	read_auth_data(); ESP_LOGD(TAG, "passkey_len %u, scan_key_len: %u\n", passkey_len, scan_key_len);//DEBUGLN(passkey);
	read_bonded_mac();
	ESP_ERROR_CHECK(nimble_port_init());
	gap_init();
	gatt_svc_init();
	ble_hs_cfg_init();
	h_nimble_task = xTaskCreateStaticPinnedToCore((TaskFunction_t)nimble_port_run,
	"nimble", sizeof(xHostStack), NULL, (configMAX_PRIORITIES - 4), xHostStack, &xHostTaskBuffer, NIMBLE_CORE);
	int &&ret = ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK);
	CHECK_(ret);
	//ble_delete_all_peers();
	#if PIN_LED_MASK
	delay(1000);
	dWrite(PIN_LED, 1);
	#endif
	mainTask(NULL);
//#endif
}

static void wifi_init() {
	ESP_LOGI(TAG, "Run factory firmware\n");
	timer_wifi = esp_timer_new([](void*){ ESP_LOGW(TAG,"TIMER_WIFI minutes %lu",(uint32_t)(TIMER_WIFI / 1000000 / 60));
		ESP_LOGD(TAG, "FreeHeap() %lu", getFreeHeap()); esp_restart();});
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(timer_wifi, TIMER_WIFI)); 
	ESP_LOGD(TAG,"TIMER_WIFI minutes %lu",(uint32_t)(TIMER_WIFI / 1000000 / 60));
	wifi_setup_default( (wifi_mode_t) ( WIFI_MODE_STA | 0
		//| WIFI_MODE_AP
	), WIFI_STORAGE_RAM);
	h_netif_sta = wifi_init_sta();
	//h_netif_ap = wifi_init_ap();
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_LOGI(TAG, "WiFi started, waiting for connection...");
	if(!wifi_sta_wait_conn()) return;
	//ap_set_dns_addr(h_netif_ap,h_netif_sta);  
	esp_log_level_set("wifi", ESP_LOG_DEBUG);
	//wifi_server_init();
	http_server_init();
		printHeapInfo();
}

void wifi_timer_stop() { ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(timer_wifi)); }

void wifi_timer_restart() { ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start(timer_wifi, TIMER_WIFI)); }

static void mainTask(void * arg = NULL) {
	h_main_task = xTaskGetCurrentTaskHandle();
	printHeapInfo();
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
}

static void isr_handler() {
	static int prev_state = 1;
	const int now = dRead(PIN_LINE); ESP_DRAM_LOGI("ISR", "%u", now);
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

void rand_device_name() {
	constexpr byte name_len = sizeof(DEVICE_NAME) -1;
	char* pName = const_cast<char*>(ble_svc_gap_device_name());
	pName[name_len] = ' ';
	const uint32_t rand_num = esp_random(); ESP_LOGD(TAG,"%lu",rand_num);
	bytes_to_str<false, 0>(&pName[name_len + 1], (byte*)&rand_num, sizeof(rand_num));
}

void ble_delete_all_peers(bond_mac_s* except) {
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
	ble_addr_t peer_id_addrs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
	int num_peers = 0;
   	ble_store_util_bonded_peers(peer_id_addrs, &num_peers, sizeof(peer_id_addrs));
	ESP_LOGI(TAG, "num_peers %d", num_peers);
	if(num_peers > MYNEWT_VAL(BLE_STORE_MAX_BONDS)) { num_peers = MYNEWT_VAL(BLE_STORE_MAX_BONDS); }
	for(size_t i = 0; i < num_peers; i++) {
		//print_addr((peer_id_addrs[i].val));
		if(except && (*reinterpret_cast<uint64_t*>(except) != 0) &&
			!ble_addr_cmp(&except->arr, &peer_id_addrs[num_peers])) continue;
		ble_store_util_delete_peer(&peer_id_addrs[i]); 
	}
#endif
}

bool read_bonded_mac() {
	byte crc = crc8_le(0, (byte*)&bonded_addr, 7);
	if(bonded_addr.crc == crc) return true;
	bonded_addr = {};
	//bonded_addr.crc = crc8_le(0, (byte*)&bonded_addr, 7);
	return false;
}

void nvs_write_sets(nvsApi nvs) {
	sets.crc = crc_func(sets); 
	ESP_LOGD(NVS,"patch %u, upd %u, crc %02X", sets.patch, sets.updated, sets.crc);
	CHECK_VOID(nvs_set_u32(nvs, NVS_KEY_OTA, *reinterpret_cast<uint32_t*>(&sets)));
	CHECK_(nvs_commit(nvs));
}

void nvs_read_sets() {
	nvsApi nvs(NVS_SPACE_SETS, NVS_READWRITE);
	auto ret = nvs_get_u32(nvs, NVS_KEY_OTA, reinterpret_cast<uint32_t*>(&sets)); //reinterpret_cast<uint32_t*>(&sets)
	if (ret == ESP_OK) {
		if (crc_func(sets) == sets.crc) {
			if(sets.patch) { ESP_LOGI(TAG, "PATCH_ON"); patch_func();  }
			else { /* timer_patch_off_cb((void*)1); */ }; //dont write
			if (sets.updated == 0) { img_state(true); return; } //validate partition if it is verify state
			else { sets.updated = 0; //only ones
				sets.valid = 1;
				auto h_timer_valid = gptimer_init(OTA_VALID_TIMER, [](gptimer_handle_t timer, const gptimer_alarm_event_data_t *, void *) -> bool {
					if(!sets.valid) { gptimer_disable(timer); gptimer_del_timer(timer); return false; } 
					esp_restart(); return false;
				});
				assert(h_timer_valid); gptimer_start(h_timer_valid);
				goto ota;
			}
		} else { ESP_LOGW("crc", ""); };
	} else { CHECK_(ret); } //alarm_on(false);
	sets = {}; 
ota: nvs_write_sets(nvs);
}

void read_auth_data() {
	nvsApi handle; size_t required_size;
	CHECK_VOID(handle.begin(NVS_SPACE_SETS, NVS_READONLY));
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_BLE_PASS, NULL, &required_size));
	if(required_size < 16 || required_size > sizeof(passkey)) return;
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_BLE_PASS, passkey, &required_size));
	passkey_len = required_size;
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_SCAN_DATA, NULL, &required_size));
	if(required_size < 8 || required_size > sizeof(scan_key)) return;
	CHECK_VOID(nvs_get_blob(handle, NVS_KEY_SCAN_DATA, scan_key, &required_size));
	scan_key_len = required_size;
}

esp_err_t save_auth_data() {
	//extern auth_t Auth;
	if (!passkey_len && !scan_key_len) { ESP_LOGW(TAG, "!Auth"); return 0xFFFF; }
	nvsApi handle; int err = ESP_FAIL;
	CHECK_RET(handle.begin(NVS_SPACE_SETS, NVS_READWRITE));
	if(passkey_len) {
		err = nvs_set_blob(handle, NVS_KEY_BLE_PASS, passkey, passkey_len);
		if(err != ESP_OK) {ESP_LOGE(TAG, "%d", err);}
	}
	if(scan_key_len) {
		err = nvs_set_blob(handle, NVS_KEY_SCAN_DATA, scan_key, scan_key_len);
		if(err != ESP_OK) {ESP_LOGE(TAG, "%d", err);}
	} //else return 0xFF00;
	if(err == ESP_OK) 
		return nvs_commit(handle);
	return err;
}

void set_boot_partition(const esp_partition_subtype_t type) {
	__unused int ret = ESP_FAIL; auto i = esp_partition_find(ESP_PARTITION_TYPE_APP, type, NULL);
	for (const esp_partition_t* part;i != NULL; i = esp_partition_next(i)) {
		part = esp_partition_get(i);
		if(part->subtype == type) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_set_boot_partition(part)); 
			break;
		}
	}
	esp_partition_iterator_release(i);
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
		DELETE_ALL_PEERS,
		NVS_ERASE_ALL,
		NVS_ERASE_ALL_EXC,
	};
	if(buf->om_len != 5) return;
	if(*reinterpret_cast<uint32_t*>(buf->om_data) != DEF_CMD_PASS) return;
	//auto val = *reinterpret_cast<decltype(wifi_key)*>(buf->om_data);
	uint8_t val = buf->om_data[4];
	switch (val) {
	case OTA_KEY: ESP_LOGI(TAG, "OTA");//ESP_ERROR_CHECK(nimble_port_stop());
		set_boot_partition(ESP_PARTITION_SUBTYPE_APP_FACTORY);
		ESP_LOGI(TAG, "reboot to FACTORY...");
	case RESTART_KEY: ESP_LOGI(TAG, "RESTART"); esp_restart(); 
		break;//xTaskNotify(main_handle, RESTART, eSetValueWithOverwrite); break;
	case VALID_KEY: ESP_LOGI(TAG, "VALID");
		if(!esp_ota_mark_app_valid_cancel_rollback()) { sets.valid = 0; }
		break;
	case SAVE_MAC: save_bonding(event->notify_rx.conn_handle);
		break;
	case DELETE_ALL_PEERS: ble_delete_all_peers(); 
		break;
	case NVS_ERASE_ALL: nvsErase(nullptr);
		break;
	case NVS_ERASE_ALL_EXC: nvsErase("phy");
		break;
	case OFFSET: DEBUG(get_task_list().c_str());
		break;
	default: ESP_LOGW(TAG, "os_mbuf 0x%02X", val);
	}
}

void get_task_list(String& str) {/*
	size_t num = uxTaskGetNumberOfTasks(), heap = ESP.getFreeHeap(); ESP_LOGD(TAG,"NumberOfTasks = %u", num);
	if (!str.reserve((num * 32 + 16) * (configGENERATE_RUN_TIME_STATS ? 2 : 1))) return;
	char* const ptr = str.begin(); 
	vTaskList(ptr);
	num = strlen(ptr); strcpy(&ptr[num], "Heap:\t"); ultoa(heap, &ptr[num+6], DEC); num += strlen(&ptr[num]); strcpy(&ptr[num],"\n\n"); num+=2;
#if configGENERATE_RUN_TIME_STATS
	vTaskGetRunTimeStats(&ptr[num]); num += strlen(&ptr[num]);
#endif
	reinterpret_cast<uint32_t*>(&str)[2] = num;*/
}

template <bool big_endian, char separ> void bytes_to_str(char* dest, cbyte* src, size_t data_size) {
	if(data_size == 0) return; 
	int i; char inc; 
	if(big_endian){ i = 0; --data_size; inc = 1;}
	else { i = data_size-1; data_size = 0; inc = -1;}
	for (;;i += inc) {
		for (int shift = 4;; shift = 0) {
			byte nibble = (src[i] >> shift) & 0xF;
			*dest++ = nibble < 10 ? nibble ^ 0x30 : nibble + ('A' - 10);
			if (shift == 0) break;
		} 
		if (i == data_size) break;
		if(separ) { *dest++ = separ; }
	}
	*dest = '\0';
}

void strtoB(cch* src, byte *dest, size_t & data_size, size_t buf_len) { 
	//{size_t str_len = str.length(), hex_len = (str_len + 1) / 2; ESP_LOGD(TAG,"hex_len = %u", hex_len);
	//if (hex_len < 4 || hex_len > buf_len) return false;}
	byte i = 0;
	//byte* _buf = (byte*)realloc(buf, hex_len); if (_buf == NULL) return false; buf = _buf;
	if (src && *src) {
		for (byte shift = 0, result = 0;; ++src) {
				switch (*src) {
				case '0'... '9':
					result <<= shift;
					result |= (*src ^ 0x30); break;
				case 'A'... 'F':
					result <<= shift;
					result |= *src - 55; break;
				case 'a' ...'f':
					result <<= shift;
					result |= *src - 87; break;
				case '\0': dest[i] = result; data_size = ++i;return;
				default: if(!shift) continue;
				goto rdy;
			}
			if (shift) {
rdy:            dest[i] = result;
				if (++i >= buf_len) break;
				result = 0; shift = 0;
			} else { shift = 4;};
		}
	} //
	data_size = i;
	//return realloc(buf, i);
	//for (size_t j = 0; j < i; j++){ DEBUGF("%02X ", buf[j]); }DEBUGLN();
}

void set_ble_device_name() {
	constexpr size_t name_len = sizeof(MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME))-1;
	static_assert(name_len >= 7); static_assert(!MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC));
	char* name = const_cast<char*>(ble_svc_gap_device_name()); byte mac[8];
	*(name += name_len) = '-';
	CHECK_VOID(esp_read_mac(mac, ESP_MAC_BT));
	//bytes_to_str<true, 0>(name+1, mac + 3, 3); //621263
	sprintf(name + 1,"%02X""%02X""%02X", mac[3],mac[4],mac[5]); //621165
}

uint32_t generate_salt() {
	static TickType_t last_change = __INT32_MAX__; 
	static time_t salt; //ESP_LOGD(TAG, "change_device_name()");
	uint32_t sec = xTaskGetTickCount();
	if(sec - last_change > pdMS_TO_TICKS(TIME_CHANGE_PIN)) {
#if !_ESP_LOG_ENABLED(3)
		if(last_change != __INT32_MAX__)
			{ ble_delete_all_peers(&bonded_addr); }
#endif
		last_change = sec;
		salt = time(NULL); 
		//if(salt < 1766600000) salt = 0;
		pincode = TOTPget(passkey, passkey_len, salt); 
		ESP_LOGI(TAG, "NEW PIN: %lu\tTime: %lu", pincode, salt);
	}
	return salt;
}

uint32_t TOTPget(const uint8_t* key, size_t key_len, time_t time) {
	return HOTPget(key, key_len, time / TOTP_TIMESTEP);
}

uint32_t HOTPget(const uint8_t* key, size_t key_len, uint64_t salt) {
	uint8_t* const pSalt = reinterpret_cast<uint8_t*>(&salt);
	uint8_t hash[20];  uint32_t result;
	swap_in_place(pSalt, sizeof(salt)); //salt = ntohll(salt);
	int ret = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA1),key,key_len,pSalt,sizeof(salt), hash);
	if(ret != 0) { ESP_LOGE(TAG, "HOTPget %d", ret); return 0; }
	for (int i = 0, offset = hash[19] & 0xF; i < 4; ++i) {
		reinterpret_cast<uint8_t*>(&result)[3 - i] = hash[offset + i];
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
		ESP_LOGE(TAG, "Buffer length is too short, only %u, need %u", buf_len, expect_len);
		return -1;
	} */
	int buffer = 0, bits_left = 0, count = 0; 
	for (const char* ptr = encoded; count < buf_len && *ptr; ++ptr) {
		char ch = *ptr;
		buffer <<= 5;
		// Deal with commonly mistyped characters
		switch (ch) {
			case ' ': case '\t':case '\r':case '\n': case '-': continue;
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
		} else { return -2; }
		buffer |= ch;
		bits_left += 5;
		if (bits_left >= 8) {
			result[count++] = buffer >> (bits_left - 8);
			bits_left -= 8;
		}
	}
	if (count < buf_len) { result[count] = '\000';} 
	return count;
}

int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len) {
	if (!length ||  length > (1 << 16)) { return -1; } //28
	int count = 0;
		int buffer = data[0], next = 1, bits_left = 8;
		while (count < encode_len && (bits_left > 0 || next < length)) {
			if (bits_left < 5) {
				if (next < length) {
					buffer <<= 8;
					buffer |= data[next++] & 0xFF;
					bits_left += 8;
				} else {
					int pad = 5 - bits_left;
					buffer <<= pad;
					bits_left += pad;
				}
			}
			int index = 0x1F & (buffer >> (bits_left - 5));
			bits_left -= 5;
			result[count++] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567"[index];
		}
		if (count < encode_len) { result[count] = '\0'; }
	return count;
}

void read_noinit() {
	ESP_LOGD(TAG,"%08X", *reinterpret_cast<uint32_t*>(&sets));
	if(crc16_le(0, (byte*)&sets, 2) == sets.crc) {
		if(!sets.updated) { img_state(true); } //validate partition if it is verify state
	} else { sets = {}; ESP_LOGW(TAG, "noinit crc");}; 
	//++sets.count;
	sets.crc = crc16_le(0, (byte*)&sets, 2);
}

void write_noinit(byte val) {
	sets.updated = val;
	sets.crc = crc16_le(0, (byte*)&sets, 2); ESP_LOGD(TAG,"noinit %u, %u, %04X", sets.patch, sets.updated, sets.crc);
}
