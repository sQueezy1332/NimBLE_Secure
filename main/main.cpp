#include "main.h"
#include "tlsf_block_functions.h"
extern "C" void app_main() { {
#ifdef DEBUG_ENABLE
	led_init();
	delay(2000);
	DEBUG_LED(0x00FF00);
#endif
	if (img_state_get() == ESP_OTA_IMG_PENDING_VERIFY) {
		h_timer_img_valid = esp_timer_new([](void*) IRAM_ATTR { 
			ESP_DRAM_LOGW("TMR", "OTA_VALID EXPIRED"); esp_restart();}, NULL, ESP_TIMER_ISR);
		ESP_ERROR_CHECK(esp_timer_start(h_timer_img_valid, TIMER_OTA_VALID));
	}
	nvs_init();
	__unused auto _noinit = read_noinit();  //0x253D7465 == crc8 //609862 //570586
#ifdef DEBUG_ENABLE
	//pinMode(13, OUTPUT);
	ESP_LOGI(TAG, "Compiled: " __TIMESTAMP__);
	ESP_LOGI(TAG, "getFreeHeap() %lu", getFreeHeap());
	esp_log_level_set("*", ESP_LOG_DEBUG);
	esp_log_level_set("nvs", ESP_LOG_INFO);
	//esp_log_level_set("wifi", ESP_LOG_INFO); 
	esp_log_level_set("efuse", ESP_LOG_INFO);
	esp_log_level_set("event", ESP_LOG_INFO);
	esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);
	esp_log_level_set("esp_netif_lwip", ESP_LOG_INFO);
	esp_log_level_set("httpd_uri", ESP_LOG_INFO);
	esp_log_level_set("wifi", ESP_LOG_INFO);
	totp_test();
	nvs_log("cal_data"); //nvsApi nvs("nvs.net80211", NVS_READWRITE); nvs_erase_all(nvs);
	xTaskCreate(usb_cdc_task, "cdc", 4096, NULL, 5, NULL);
	//vTaskPrioritySet(NULL, 10);
	//auto heart = esp_timer_new([](void*){ digitalToggle(12);}); esp_timer_start_periodic(heart, HEART_RATE_PERIOD);
#endif
	RELAY_DEFAULT_IMPL(); RELAY_2_DEFAULT_IMPL(); 
   	gpio_config_t conf = { (BIT(PIN_RELAY) | RELAY_2_MASK | PIN_LED_MASK), GPIO_MODE_RELAY_IMPL }; //LED ON on c3 super mini
	ESP_ERROR_CHECK(gpio_config(&conf));
	gpio_set_drive_capability((gpio_num_t)PIN_RELAY, DRIVE_CAP_IMPL);
#ifndef CONFIG_DOMOPHONE
	if(_noinit.ota) { wifi_init(); }
#else
  	wifi_init(); sntp_setup();
#endif
	h_timer_patch = esp_timer_new(timer_patch_off_cb, NULL, ESP_TIMER_ISR); assert(h_timer_patch);
	nvs_sets_read(); 
	//gpio_input_enable((gpio_num_t)PIN_LINE);
#if not defined(CONFIG_ADC_LINE)
	#if defined(CONFIG_DOMOPHONE) && defined(DEBUG_ENABLE)
		#pragma message "NO INTERRUPT" //coz gpio8 uses rgb led
	#else
		gpio_pullup_en((gpio_num_t)PIN_LINE);
		attachInterrupt(PIN_LINE, isr_handler, LINE_INTERRUPT_MODE);
		enableInterrupt(PIN_LINE);
	#endif
#endif
	auth_data_read(); ESP_LOGD(TAG, "pass_key_len %u, scan_key_len: %u\n", pass_key_len, scan_key_len);//DEBUGLN(pass_key);
	bluetooth_init();
#if PIN_LED_MASK
	delay(1000);
	conf.pin_bit_mask = BIT(PIN_LED); conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&conf); dWrite(PIN_LED, 1); //led off (open drain)
#endif
	} mainTask(); 
}

void mainTask(void *) {
	h_main_task = xTaskGetCurrentTaskHandle();
#ifdef	DEBUG_ENABLE
	printHeapInfo(); //ble_store_clear();
	print_task_list();
	DEBUG_LED(0);
#endif
	for(__unused int rssi = 0;;) {
		uint32_t notify = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		switch (*(uint8_t*)&notify) { 
		//case OTA: vTaskSuspend(ble_handle);
			//set_boot_partition(ESP_PARTITION_SUBTYPE_APP_FACTORY);
			//ESP_LOGI(TAG, "reboot to FACTORY...");//esp_restart(); break;
		//case VALID: esp_ota_mark_app_valid_cancel_rollback(); break;
#ifdef CONFIG_DOMOPHONE
		case ACCESS_BLE:
			check_distance_by_rssi();
			break;
		case EXIT_BUTTION: ESP_LOGI(TAG, "BUTTION");
		 	patch_timer(TIMER_PATCH_BUTTON);
			delay(200);
			enableInterrupt(PIN_LINE);
			break;
#endif
		case NOTIFY_IO: send_io_notify();
		case DEBUG_LED_OFF:
			if(DEBUG_LED_GET()) { DEBUG_LED(0x0); }
			break;
		case RESTART: ESP_LOGI(TAG, "RESTART"); delay(100); esp_restart(); break;
		case NOTIFY_TIME: break;
		default: //if(wifi_is_connected()) { if(!esp_wifi_sta_get_rssi(&rssi)) { esp_rom_printf("rssi: %d\n", rssi); } }
			//generate_pin(); 
			break;
		} 
	}
}

#ifndef CONFIG_DOMOPHONE
void isr_handler() {
#ifndef CONFIG_ADC_LINE
	const auto now = dRead(PIN_LINE);
	if(now) { ESP_DRAM_LOGD("", "1"); }
	else { ESP_DRAM_LOGD("", "0"); }
	if(sets.patch == false) {
		dWrite(PIN_RELAY, now);
	}
	else if(need_notify_io()) { xTaskNotifyFromISR(h_main_task, NOTIFY_IO, eSetValueWithOverwrite, NULL); };
#if PIN_LED_MASK
	dWrite(PIN_LED, now);
#endif
#endif
}
#else
void isr_handler() {
	xTaskNotifyFromISR(h_main_task, EXIT_BUTTION, eSetValueWithOverwrite, NULL);
	disableInterrupt(PIN_LINE);
}
#endif


void bluetooth_init() {
	ESP_ERROR_CHECK(nimble_port_init());
	ble_svc_gap_device_appearance_set(GENERIC_HID_TAG);
	ble_device_name_set();
	ble_hs_cfg_init();
	ble_svc_gap_init();
	gatt_init();
	h_nimble_task = xTaskCreateStaticPinnedToCore( [](void*){ nimble_port_run(); vTaskDelete(h_nimble_task); assert(0); },
	"nimble", sizeof(xHostStack), NULL, (configMAX_PRIORITIES - 4), xHostStack, &xHostTaskBuffer, NIMBLE_CORE);
	CHECK_VOID(ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK));
}

void wifi_init() {
#ifndef CONFIG_DOMOPHONE
	h_timer_wifi = esp_timer_new([](void*) { ESP_DRAM_LOGI("TMR", "\"WIFI\" FreeHeap %lu", getFreeHeap()); esp_restart(); 
	}, NULL, ESP_TIMER_ISR);
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(h_timer_wifi, TIMER_WIFI)); 
	ESP_LOGD(TAG,"TIMER_WIFI ms %lu", uint32_t(TIMER_WIFI / 1000));	
	//set_scan_periods(BLE_GAP_SCAN_ITVL_MS(120), BLE_GAP_SCAN_WIN_MS(40));
#endif
#ifdef STATION_MODE
	wifi_setup_default(WIFI_STORAGE_RAM);
	ESP_ERROR_CHECK(wifi_init_sta(STA_SSID, STA_PASS)); 
#else
	ESP_ERROR_CHECK(wifi_init_ap(AP_SSID, AP_PASS));
#endif
	wifi_hostname_set();
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(http_server_init());
#if defined DEBUG_ENABLE && defined STATION_MODE
	//ap_set_dns_addr(h_netif_ap,h_netif_sta);
	ESP_LOGI(TAG, "WiFi started, waiting for connection...");
	EventBits_t bits = xEventGroupWaitBits(h_group_wifi, WIFI_STA_GOT_IP,
		pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
	if (bits & WIFI_STA_GOT_IP) {
		ESP_LOGI(TAG, "Connected!");
	} else if (bits & WIFI_STA_CONNECTED) {
		ESP_LOGW(TAG, "No IP");
	} else {
		ESP_LOGW(TAG, "bits: 0x%X", bits);
			//CHECK_(esp_wifi_disconnect());
	}
	//esp_log_level_set("wifi", ESP_LOG_DEBUG);
#endif
}
#ifdef DEBUG_ENABLE
#include "driver/usb_serial_jtag.h"
void usb_cdc_task(void *arg) {
	const char* TAG = "CDC";
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
				case 'V': ESP_LOGI(TAG, "VALID"); ota_rollback_revoke();
				case 'N': CHECK_(esp_wifi_disconnect());  break;
				case 'W': CHECK_(wifi_init_ap(AP_SSID, AP_PASS)); break;
					continue;
			}
		}
		//ESP_LOG_BUFFER_HEX(TAG, data, len);
		//usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(20));
		data[len] = '\0'; ESP_LOGW(TAG, "%s", data);
    }
}
#endif

void nvs_sets_write(nvsApi nvs) {
	//sets.crc = crc_impl(sets); 
	ESP_LOGD(TAG,"patch %u, ota %u, val %u, crc %02X", sets.patch, sets.ota, sets.flag, sets.crc);
#ifndef DEBUG_ENABLE
	CHECK_VOID(nvs_set_u32(nvs, NVS_KEY_SETS, *reinterpret_cast<uint32_t*>(&sets)));
	CHECK_(nvs_commit(nvs));
#endif
}

void nvs_sets_read() {
	nvsApi nvs(NVS_SPACE_SETTINGS, NVS_READWRITE);
	auto ret = nvs_get_u32(nvs, NVS_KEY_SETS, reinterpret_cast<uint32_t*>(&sets));
	if (ret == ESP_OK) {
		//auto crc =  crc_impl(sets);
		//if (crc == sets.crc) {
#ifdef CONFIG_DOMOPHONE
			if(sets.flag) { patch_impl = lock_open_close; }
			else { patch_impl = lock_open_only; }
#endif
			if(sets.patch) { ESP_LOGI(TAG, "PATCH_ON"); patch_timer();  }
			else { RELAY_UNPATCH_IMPL(); RELAY_2_UNPATCH_IMPL(); ESP_LOGI(TAG, "PATCH_OFF"); };
			return;
		//} else { ESP_LOGW(TAG, "crc %u sets.crc %u", crc, sets.crc); };
	} else { CHECK_(ret); } //alarm_on(false);
	sets = {}; 
__unused ota: nvs_sets_write(nvs);
}

sets_noinit read_noinit() {
	ESP_LOGD(TAG,"sets_noinit: %08X", *reinterpret_cast<uint32_t*>(&noinit));
	if(crc_impl(noinit) == noinit.crc) {
		return noinit;
	} else { noinit = {
#ifdef CONFIG_DOMOPHONE
		noinit.rssi = MIN_RSSI_PATCH_DEFAULT
#endif
	}; noinit.crc = crc_impl(noinit);
	ESP_LOGW(TAG, "!noinit crc");
};
	return noinit;
}

void write_noinit_ota(uint8_t val) {
	noinit.ota = val;
	noinit.crc = crc_impl(noinit); ESP_LOGD(TAG,"sets_noinit: %08X", *reinterpret_cast<uint32_t*>(&noinit));
}

void auth_data_read() {
	nvsApi handle; size_t required_size; esp_err_t ret;
	ret = handle.begin(NVS_SPACE_SETTINGS, NVS_READONLY); if (ret) { CHECK_(ret); return; }
	if((ret = nvs_get_blob(handle, NVS_KEY_BLE_PASS, NULL, &required_size)) == ESP_OK) {
		if(required_size >= 16 && required_size <= sizeof(pass_key)) {
			if((ret = nvs_get_blob(handle, NVS_KEY_BLE_PASS, pass_key, &required_size)) == ESP_OK) {
				pass_key_len = required_size;
				ESP_LOGI(TAG, "pass_key %s", "readed");
			}
		}
	} else { ESP_LOGW(TAG, "pass_key %s", "not readed");}
	if((ret = nvs_get_blob(handle, NVS_KEY_SCAN_DATA, NULL, &required_size)) == ESP_OK) {
		if(required_size >= 8 && required_size <= sizeof(scan_key)) {
			if((ret = nvs_get_blob(handle, NVS_KEY_SCAN_DATA, scan_key, &required_size)) == ESP_OK){
				scan_key_len = required_size;
				ESP_LOGI(TAG, "scan_key %s", "readed");
			}
		}
	} else { ESP_LOGW(TAG, "scan_key %s", "not readed");}
}

esp_err_t auth_data_save() {
	int err = ESP_FAIL;
	if (pass_key_len < 8 || scan_key_len < 8) { ESP_LOGW(TAG, "!Auth"); 
		err = (uint8_t)pass_key_len; ((uint8_t*)&err)[1] = scan_key_len; err |= BIT31;
		return err; 
	}
	nvsApi handle; 
	CHECK_RET(handle.begin(NVS_SPACE_SETTINGS, NVS_READWRITE));
	if(pass_key_len > 0) {
		err = nvs_set_blob(handle, NVS_KEY_BLE_PASS, pass_key, pass_key_len);
		if(err) { ESP_LOGE(TAG, "%d", err); }
	}
	if(pass_key_len > 0) {
		err = nvs_set_blob(handle, NVS_KEY_SCAN_DATA, scan_key, scan_key_len);
		if(err) { ESP_LOGE(TAG, "%d", err); }
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

void ota_rollback_revoke() {
	if(h_timer_img_valid) {
		esp_timer_stop(h_timer_img_valid);
		ESP_ERROR_CHECK(esp_timer_delete(h_timer_img_valid));
		h_timer_img_valid = NULL;
	}
	esp_ota_mark_app_valid_cancel_rollback();
}

void parse_adv_cb(const struct ble_gap_ext_disc_desc* event) {
#ifndef CONFIG_DOMOPHONE
	const uint8_t *data = event->data, len = event->length_data;
	if(len == scan_key_len && !memcmp(data, scan_key, scan_key_len)) {
		ESP_LOGW(TAG, "PASS!");
#ifndef CONFIG_DOMOPHONE
		RELAY_2_PATCH_IMPL();
		ble_gap_disc_cancel();
		adv_init();
#else
		RELAY_PATCH_IMPL();
		if(!(event->props & BLE_HCI_ADV_LEGACY_MASK)) {
			adv_init();
		}
#endif
	}
	/* if(type == COMPLETE_NAME  || type == SHORT_NAME ) {
		if ((len - 1 == size) && !memcmp(data +1, scan_key, size)) {
			ESP_LOGW(TAG, "PASS!");
			ble_gap_disc_cancel(); adv_init();
		}
	} *///else if (type == UUID32_DATA && len == 9 && *(uint32_t*)(&data[++i]) == ...) { *(uint32_t*)(&data[i+=4])  }
#endif
}

void host_sync_cb() {
	adv_init();
	//ble_scan_init();//set_random_addr();
}

int parse_rx_data_cb(const struct ble_gap_event* event) {
	const auto *notify = &event->notify_rx;
	const os_mbuf* buf = notify->om;
	enum { 
		OFFSET = DEF_CMD_OFFSET,
		OTA_KEY,
		MAIN_KEY,
		RESTART_KEY,
		VALID_KEY,
		SAVE_BOND,
		NVS_ERASE_ALL,
		NVS_ERASE_ALL_EXC,
		BLE_STORE_CLEAR,
		OPEN_ONLY,
		OPEN_CLOSE,
		RSSI_THERSHOLD
	};
	if(buf->om_len > 6)  { 
		ESP_LOGW(TAG, "!om_len"); return BLE_HS_EMSGSIZE; 
	}
	if(*reinterpret_cast<uint32_t*>(buf->om_data) != DEF_CMD_PASS) { 
		ESP_LOGW(TAG, "!pass"); return BLE_HS_EAUTHOR; 
	}
	//auto val = *reinterpret_cast<decltype(wifi_key)*>(buf->om_data);
	uint8_t val = buf->om_data[4];
	switch (val) {
	case OTA_KEY://ESP_ERROR_CHECK(nimble_port_stop());
		write_noinit_ota(1);
		//set_boot_partition(ESP_PARTITION_SUBTYPE_APP_FACTORY);
		ESP_LOGI(TAG, "reboot to FACTORY...");
	case RESTART_KEY: esp_restart();
		break;
	case MAIN_KEY: write_noinit_ota(0);
		break;
	case VALID_KEY: ota_rollback_revoke();
		break;
	case SAVE_BOND: return save_bonding(event->notify_rx.conn_handle);
		break;
	case NVS_ERASE_ALL: nvsEraseAll(nullptr);
		break;
	case NVS_ERASE_ALL_EXC: nvsEraseAll("phy");
		break;
	case BLE_STORE_CLEAR: ble_delete_all_bonds();
		break;
	case OFFSET: print_task_list(); break;
#ifdef CONFIG_DOMOPHONE
	case OPEN_ONLY: patch_impl = lock_open_only;
		sets.flag = 0; nvs_sets_write();
		break;
	case OPEN_CLOSE: patch_impl = lock_open_close;
		sets.flag = 1; nvs_sets_write();
		break;
	case RSSI_THERSHOLD:
		val = buf->om_data[5];
		if((int8_t)val > -30) return BLE_HS_EBADDATA;
		noinit.rssi = val; noinit.crc = crc_impl();
		break;
#endif	
	default: ESP_LOGW(TAG, "os_mbuf 0x%02X", val);
		return BLE_HS_EINVAL;
	}
	return 0;
}

void patch_timer(uint64_t period) { 
#ifndef CONFIG_DOMOPHONE
	if(!sets.patch) { sets.patch = true; 
		nvs_sets_write(); 
	}
#endif
	if(period) { 
		RELAY_PATCH_IMPL(); RELAY_2_PATCH_IMPL(); 
		CHECK_VOID(esp_timer_start(h_timer_patch, period));
		DEBUG_LED(0x00'00'FF);
	}
	else { 
		RELAY_UNPATCH_IMPL(); RELAY_2_UNPATCH_IMPL(); 
		esp_timer_stop(h_timer_patch); 
		DEBUG_LED(0); 
	}
	if(need_notify_io()) { send_io_notify(); }
}

void timer_patch_off_cb(void *) {
    RELAY_UNPATCH_IMPL(); RELAY_2_UNPATCH_IMPL();
#ifndef CONFIG_DOMOPHONE
    sets.patch = false;
	nvs_sets_write();
#endif
	if(need_notify_io()) {
		xTaskNotifyFromISR(h_main_task, NOTIFY_IO, eSetValueWithoutOverwrite, NULL);
	}
#ifdef DEBUG_ENABLE
	else { xTaskNotifyFromISR(h_main_task, DEBUG_LED_OFF, eSetValueWithoutOverwrite, NULL); }
#endif
}

int ble_delete_all_bonds() {
#if MYNEWT_VAL_BLE_STORE_MAX_BONDS
	nvsApi nimble_nvs(NVS_SPACE_NIMBLE, NVS_READWRITE);
	CHECK_RET(nvs_erase_all(nimble_nvs));
	return 0;
#endif
return ENOTSUP;
}

void check_distance_by_rssi() {
	extern uint16_t MAX_HANDLE; 
	if (tracked_conn_mask & get_conn_encrypted()) {
		int8_t rssi, target_rssi = noinit.rssi;
		for (uint16_t h = MIN_HANDLE_VAL; h < MAX_HANDLE; h++) {
			if (handle_read(tracked_conn_mask, h)) {
				int ret = ble_gap_conn_rssi(h, &rssi);
				if (unlikely(ret)) { 
					ESP_LOGW(TAG, "ret %d handle %u", ret, h);
					handle_write(tracked_conn_mask, h, 0);
				} else if (rssi >= target_rssi) {
					patch_timer();
					ESP_LOGI(TAG, "RSSI %i, handle %u", rssi, h);
					handle_write(tracked_conn_mask, h, 0);
				} else { ESP_LOGD(TAG, "RSSI %i, handle %u", rssi, h); }
			}
		}
	} else { 
		ESP_LOGD(TAG, "tracked_conn 0x%X & conn_enc 0x%X" , tracked_conn_mask, get_conn_encrypted());  
		tracked_conn_mask = 0; 
	}
	if(!tracked_conn_mask) { esp_timer_stop(h_timer_rssi);} 
	else { /*ESP_LOGI(TAG, "tracked_conn_mask %u", tracked_conn_mask);*/ }
}

int ble_conn_encrypted_cb(uint16_t handle) { 
	int ret = set_encryption(handle); 
#ifdef CONFIG_DOMOPHONE
	int8_t rssi;
	CHECK_GOTO(ble_gap_conn_rssi(handle, &rssi), exit); 
	if (rssi >= noinit.rssi) {
		ESP_LOGI(TAG, "RSSI %i, handle %u", rssi, handle);
		patch_timer();
	} else {
		ESP_LOGW(TAG, "RSSI %i, handle %u", rssi, handle);
		handle_write(tracked_conn_mask, handle, 1);
		if(!h_timer_rssi) { h_timer_rssi = esp_timer_new([](void*) IRAM_ATTR { 
				xTaskNotifyFromISR(h_main_task, ACCESS_BLE, eSetValueWithoutOverwrite, NULL);} , 
				NULL, ESP_TIMER_ISR);
		}
		esp_timer_start_periodic(h_timer_rssi, TIMER_CHECK_RSSI);
	}
#else
	patch_timer();
#endif
__unused exit:
	adv_init(); 
	return ret;
}

std::unique_ptr<char[]> task_list(size_t* len) {
	const size_t num = uxTaskGetNumberOfTasks(); //heap = getFreeHeap();
	const size_t buf_size = ALIGN_TO_16(((num * 32)  * (configGENERATE_RUN_TIME_STATS ? 2 : 1)) + 128);
	std::unique_ptr<char[]> ptr(new char[buf_size]); 
	char* str = ptr.get(); if(!str) return ptr;//931205
	vTaskList(str); 
	size_t i = strlen(str);
	strcpy(&str[i],"\n\n"); i += 2;
#if configGENERATE_RUN_TIME_STATS
	vTaskGetRunTimeStats(&str[i]); 
	i += strlen(&str[i]); 
	strcpy(&str[i],"\n"); i += 1;
#endif
	i += sprintf(&str[i] , "Heap: ");
	i += heap_caps_sprint_heap_info(&str[i]);
	uint64_t seconds = micros() / 1000000;
    size_t days = seconds / (60*60*24);
   	size_t rem = seconds % (60*60*24);
    size_t hours = rem / (60*60);
	size_t mins = rem % (60*60) / 60;
	i += sprintf(&str[i] , "Uptime: %ud %uh %um\n", days, hours, mins);
	i += sprintf(&str[i], "Compiled: " __TIMESTAMP__ "\n");
	ESP_LOGD(TAG,"\nNumberOfTasks: %u, buf_size: %u, strlen: %u", num, buf_size, i);
	if(len) *len = i;
	return ptr;
}

void ble_device_name_set() {
	static_assert(!MYNEWT_VAL_BLE_STATIC_TO_DYNAMIC); static_assert(MYNEWT_VAL_BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH >=15);
	constexpr int name_len = sizeof(MYNEWT_VAL_BLE_SVC_GAP_DEVICE_NAME)-1; static_assert(name_len >= 7);
	char* name = const_cast<char*>(ble_svc_gap_device_name());
	*(name + name_len) = '-';
	uint8_t mac[8]; CHECK_VOID(esp_read_mac(mac, ESP_MAC_BT));
	//bytes_to_str<true, 0>(name+1, mac + 3, 3); //621263
	sprintf(name + name_len + 1,"%02X%02X%02X", mac[3],mac[4],mac[5]); //621165
	size_t len = strlen(name);
	name[MYNEWT_VAL_BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH] = len;
	ESP_LOGI(TAG, "gap_device_name: %.*s len: %u", len, name, len);
}

void wifi_hostname_set() {
	const int name_len = sizeof(WIFI_HOSTNAME) - 1;
	char name[32] = WIFI_HOSTNAME; uint8_t mac[8];
	name[name_len] = '-';
	CHECK_VOID(esp_read_mac(mac, ESP_MAC_WIFI_STA));
	sprintf(name + name_len + 1,"%02X%02X%02X", mac[3],mac[4],mac[5]);
	esp_netif_set_hostname(h_netif_sta, name); // LWIP_LOCAL_HOSTNAME
	const char* new_name = NULL; esp_netif_get_hostname(h_netif_sta, &new_name);
	if(new_name) { ESP_LOGI(TAG, "hostname: %s", new_name); }
}

template <bool big_endian, char separ> int bytes_to_str(const uint8_t* src, char* dest, size_t data_size) {
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
#ifdef CONFIG_DOMOPHONE
#include "esp_sntp.h"
#include "services/cts/ble_svc_cts.h"
extern "C" void set_cts_noinit_vals(time_t val, uint8_t reason);
void sntp_setup() {
	esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  	sntp_set_time_sync_notification_cb([](struct timeval *tv) {
		set_cts_noinit_vals(tv->tv_sec, EXTERNAL_REFERENCE_TIME_UPDATE_MASK);
		ESP_LOGI(TAG, "sntp update sec: %lu", tv->tv_sec);
	});
  	esp_sntp_setservername(0, "pool.ntp.org");
  	esp_sntp_setservername(1, "time.nist.gov");
	esp_sntp_setservername(2, "0.ru.pool.ntp.org");
  	esp_sntp_init();
}
#endif

uint32_t generate_pin() {
	static uint32_t prev_time = __UINT32_MAX__;
	static uint32_t pincode;
	time_t now = time(NULL);
	//uint32_t rem = now % TOTP_TIMESTEP;
	if(now - prev_time >= TOTP_TIMESTEP) {
		prev_time = now;
		pincode = TOTPget(pass_key, pass_key_len, now);
		ESP_LOGI(TAG, "NEW PIN: %lu Time: %lu", pincode, now);
	}
#ifndef DEBUG_ENABLE 
	return pincode; 
#else
	return 111111;
#endif

}

uint32_t TOTPget(const uint8_t* key, size_t key_len, time_t time) {
	return HOTPget(key, key_len, time / TOTP_TIMESTEP);
}

uint32_t HOTPget(const uint8_t* key, size_t key_len, uint64_t salt) {
	uint8_t* pSalt = (uint8_t*)&salt;
	uint8_t hash[20];  uint32_t result;
	swap_in_place(pSalt, sizeof(salt)); //salt = htonll(salt);
	int i = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA1),key,key_len,pSalt,sizeof(salt), hash);
	if(i != 0) { ESP_LOGE(TAG, "HOTPget %d", i); return 0; }
	for (size_t offset = hash[19] & 0xF; i < 4; ++i) {
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

void nvsEraseAll(const char *except) {
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