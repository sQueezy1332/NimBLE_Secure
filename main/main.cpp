#include "main.h"

extern "C" void port_start_app_hook() { if(esp_reset_reason() == ESP_RST_POWERON) ets_delay_us(1000'000); }
extern "C" void app_main() {
    main_init(); DEBUGLN(__TIMESTAMP__);
    ESP_LOGI(TAG, "Compiled: " __DATE__ "\t" __TIME__); 
    //read_noinit();  //0x253D7465 == crc8 //609862 //570586
    //ESP_LOGI(TAG,"%lu", ESP.getFreeHeap());nvs_test();ESP_LOGI(TAG,"%lu", ESP.getFreeHeap());
#ifdef DEBUG_ENABLE
    DEBUGLN(ESP.getEfuseMac(), HEX);
    {byte buf[64]; char str[64]; const int len = base32_decode(DEF_BLE_PASS, buf, sizeof(buf));
    for (size_t i = 0; i < len; i++) { DEBUGF("0x%02X, ", buf[i]); };DEBUGF("\ncount = %d\n", len);
    if(len > 0){base32_encode(buf, len, str, sizeof(str));DEBUGF("%s\nTOTP = %lu\n",str,TOTPget(buf, len, 12345678));}} //885100
    //pinMode(12, OUTPUT);
    //auto heart = esp_timer_init([](void*){ digitalToggle(12);}); esp_timer_start_periodic(heart, HEART_RATE_PERIOD);
    //led_init();
    //Serial.onReceive(uart_cb); //Serial.setRxTimeout(2);
#endif
    RELAY_DEFAULT_IMPL(); dWrite(PIN_LED, 1);//pinMode(PIN_RELAY, GPIO_MODE_INPUT_OUTPUT_OD);
   	gpio_config_t conf = { (BIT(PIN_RELAY) | BIT(PIN_LED)), GPIO_MODE_RELAY_IMPL };
    ESP_ERROR_CHECK(gpio_config(&conf));
#ifdef CONFIG_FACTORY_FIRMWARE
    RELAY_PATCH_IMPL(1);
    ESP_LOGI(TAG, "Run factory firmware\n");
    wifi_ap_init();
    wifi_server_init(); return;
#else
#ifdef CONFIG_GENERIC_PATCHER
    conf.pin_bit_mask = BIT(PIN_LINE); conf.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&conf));
#else
    gpio_set_drive_capability((gpio_num_t)PIN_RELAY,GPIO_DRIVE_CAP_0);
    attachInterrupt(PIN_LINE, isr_handler, GPIO_INTR_ANYEDGE);
    enableInterrupt(PIN_LINE);
    main_handle =  xTaskCreateStaticPinnedToCore(mainTask, "main", sizeof(xMainStack), NULL, 1, xMainStack, &xMainTaskBuffer, configNUM_CORES -1);//nimble_port_stop();
#endif
    assert(timer_patch = esp_timer_init(timer_patch_off_cb));
    nvs_read_sets();
    read_auth_data(); log_d("passkey_len %u, scan_key_len: %u\n", passkey_len, scan_key_len);//DEBUGLN(passkey);
    read_bonded_mac();
    ESP_ERROR_CHECK(nimble_port_init());
    gap_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(gatt_svc_init());
    ble_hs_cfg_init();
    ble_handle = xTaskCreateStaticPinnedToCore([](void*) { nimble_port_run();}, "nimble",sizeof(xHostStack), NULL, 21,xHostStack,&xHostTaskBuffer, NIMBLE_CORE);
    //ble_delete_all_peers();
#endif
}

static void mainTask(void *) {
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
        else if(need_notify()) { xTaskNotifyFromISR(main_handle, NOTIFY_ALARM, eNoAction, NULL); };
//#ifdef DEBUG_ENABLE
        dWrite(PIN_LED, now);
//#endif
    }
}

 void ble_hs_cfg_init() {
    ble_hs_cfg.reset_cb = [](int reason) { ESP_LOGI(TAG, "nimble stack reset, reason: %d", reason);}; //on_stack_reset is called when host resets BLE stack due to errors
    ble_hs_cfg.sync_cb = host_sync_cb;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_sc_only = 1;
    ble_hs_cfg.sm_sec_lvl = 4;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_store_config_init();
    ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK);
}

void rand_device_name() {
    constexpr byte name_len = sizeof(DEVICE_NAME) -1;
    char* pName = const_cast<char*>(ble_svc_gap_device_name());
    pName[name_len] = ' ';
    const uint32_t rand_num = esp_random(); log_d("%lu",rand_num);
    bytes_to_str<false, 0>(&pName[name_len + 1], (byte*)&rand_num, sizeof(rand_num));
}

void ble_delete_all_peers(bond_mac_s* except) {
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    ble_addr_t peer_id_addrs[CONFIG_BT_NIMBLE_MAX_BONDS];
    int num_peers;
    CHECK_VOID(ble_store_util_bonded_peers(peer_id_addrs, &num_peers, sizeof(peer_id_addrs)));
    ESP_LOGI(TAG, "num_peers %u", num_peers);
    for(; num_peers > 0;) {
        --num_peers; print_addr((peer_id_addrs[num_peers].val));
        if(except && (*reinterpret_cast<uint64_t*>(except) != 0) &&
            !ble_addr_cmp(&except->arr, &peer_id_addrs[num_peers])) continue;
        CHECK_VOID(ble_store_util_delete_peer(&peer_id_addrs[num_peers])); 
    }
#endif
}

bool read_bonded_mac() {
    byte crc = crc8_le(0, (byte*)&bonded_addr, 7);
    if(bonded_addr.crc == crc)  return true;
    bonded_addr = {};
    //bonded_addr.crc = crc8_le(0, (byte*)&bonded_addr, 7);
    return false;
}

void save_bonded_mac(const ble_addr_t & addr) {
    if(addr.type != BLE_ADDR_PUBLIC /* || addr.type != BLE_ADDR_PUBLIC_ID */) return;
    bonded_addr.arr = addr;
    bonded_addr.crc = crc8_le(0, (byte*)&bonded_addr, 7);
}

void nvs_write_sets(nvsApi nvs) {
	sets.crc = crc_func(sets);  log_d("%u, %u, %04X", sets.patch, sets.updated, sets.crc);
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
            else { sets.updated = 0; 
                timer_valid = esp_timer_init([](void*){ esp_restart();}); 
                esp_timer_start_once(timer_valid, SEC*60*30);
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
    nvsApi handle;
    CHECK_RET(handle.begin(NVS_SPACE_SETS, NVS_READWRITE));
    if(passkey_len) {
        CHECK_RET(nvs_set_blob(handle, NVS_KEY_BLE_PASS, passkey, passkey_len));
    }
    if(scan_key_len) {
        CHECK_RET(nvs_set_blob(handle, NVS_KEY_SCAN_DATA, scan_key, scan_key_len));
    } //else return 0xFF00;
    CHECK_RET(nvs_commit(handle));
    return ESP_OK;
}

void set_boot_partition(const esp_partition_subtype_t type) {
	__unused auto ret = ESP_FAIL; auto i = esp_partition_find(ESP_PARTITION_TYPE_APP, type, NULL);
	for (const esp_partition_t* part;i != NULL; i = esp_partition_next(i)) {
		part = esp_partition_get(i);
		if(part->subtype == type) {
			ret = esp_ota_set_boot_partition(part); break;
		}
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
	esp_partition_iterator_release(i);
}

void parse_adv_cb(cbyte* data, byte len) {
    //cbyte len = *data, type = data[1], size = scan_key_len;
    if(len == scan_key_len && !memcmp(data, scan_key, scan_key_len)) {
        ESP_LOGW(TAG, "PASS!"); ble_gap_disc_cancel(); adv_init();
    }
    /* if(type == COMPLETE_NAME  || type == SHORT_NAME ) {
        if ((len - 1 == size) && !memcmp(data +1, scan_key, size)) {
			ESP_LOGW(TAG, "PASS!");
            ble_gap_disc_cancel(); adv_init();
        }
    } *///else if (type == UUID32_DATA && len == 9 && *(uint32_t*)(&data[++i]) == ...) { *(uint32_t*)(&data[i+=4])  }
}

void parse_rx_data(const os_mbuf *buf) {
    extern ble_gap_conn_desc desc;
    if(buf->om_len != 4) return;
    //else if(buf->om_len == 8){} 
    auto val = *reinterpret_cast<decltype(wifi_key)*>(buf->om_data);
    switch (val) {
	case DEF_OTA_KEY: ESP_LOGI("OTA", "");//ESP_ERROR_CHECK(nimble_port_stop());
        set_boot_partition(ESP_PARTITION_SUBTYPE_APP_FACTORY); ESP_LOGI(TAG, "reboot to FACTORY...");
    case DEF_RESTART_KEY: ESP_LOGI("RESTART", "");
		//xTaskNotify(main_handle, RESTART, eSetValueWithOverwrite); break;
		esp_restart(); break;
    case DEF_VALID_KEY: ESP_LOGI("VALID", "");
        if(timer_valid) { esp_timer_stop(timer_valid); esp_timer_delete(timer_valid);}
		esp_ota_mark_app_valid_cancel_rollback(); break;
    //case 1234: set_cts_unix(val);
    case DEF_DELETE_ALL_PEERS: ble_delete_all_peers(); break;
    case DEF_NVS_ERASE_ALL: nvsErase("phy"); break;
    case DEF_NVS_ERASE_ALL + 1: nvsErase(nullptr); break;
    case DEF_SAVE_MAC:  save_bonded_mac(desc.peer_id_addr); break;
    case DEF_PRINT_KEY: DEBUG(get_task_list()); break;
    default: ESP_LOGW("key", "0x%08X", val);
    }
}

void get_task_list(String& str) {
	size_t num = uxTaskGetNumberOfTasks(), heap = ESP.getFreeHeap(); log_d("NumberOfTasks = %u", num);
	if (!str.reserve((num * 32 + 16) * (configGENERATE_RUN_TIME_STATS ? 2 : 1))) return;
	char* const ptr = str.begin(); 
	vTaskList(ptr);
    num = strlen(ptr); strcpy(&ptr[num], "Heap:\t"); ultoa(heap, &ptr[num+6], DEC); num += strlen(&ptr[num]); strcpy(&ptr[num],"\n\n"); num+=2;
#if configGENERATE_RUN_TIME_STATS
	vTaskGetRunTimeStats(&ptr[num]); num += strlen(&ptr[num]);
#endif
    reinterpret_cast<uint32_t*>(&str)[2] = num;
}

template <bool big_endian, char separ> void bytes_to_str(char* ptr, const byte* buf, byte data_size) {
    if(data_size == 0) return; 
    byte i, num, shift, nibble; char inc; 
    if(big_endian){ i = 0; --data_size; inc = 1;}
    else { i = data_size-1; data_size = 0; inc = -1;}
	for (;;i += inc) {
		for (shift = 4, num = buf[i];; shift = 0) {
			nibble = (num >> shift) & 0xF;
			*ptr++ = nibble < 10 ? nibble ^ 0x30 : nibble + ('A' - 10);
			if (shift == 0) break;
		} 
        if (i == data_size) break;
        if(separ) { *ptr++ = separ; }
	}
    *ptr = '\0';
}

void strtoB(cch* ptr, byte *buf, size_t & data_size, size_t buf_len) { 
	//{size_t str_len = str.length(), hex_len = (str_len + 1) / 2; log_d("hex_len = %u", hex_len);
	//if (hex_len < 4 || hex_len > buf_len) return false;}
	byte i = 0;
	//byte* _buf = (byte*)realloc(buf, hex_len); if (_buf == NULL) return false; buf = _buf;
    if (ptr && *ptr) {
        for (byte shift = 0, result = 0;; ++ptr) {
                switch (*ptr) {
                case '0'... '9':
                    result <<= shift;
                    result |= (*ptr ^ 0x30); break;
                case 'A'... 'F':
                    result <<= shift;
                    result |= *ptr - 55; break;
                case 'a' ...'f':
                    result <<= shift;
                    result |= *ptr - 87; break;
                case '\0': buf[i] = result; data_size = ++i;return;
                default: if(!shift) continue;
                goto rdy;
            }
            if (shift) {
rdy:            buf[i] = result;
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
    const size_t name_len = sizeof(MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME))-1; static_assert(name_len >= 7);
    char* name = const_cast<char*>(ble_svc_gap_device_name()); byte mac[8];
    *(name += name_len) = '-';
    CHECK_VOID(esp_read_mac(mac, ESP_MAC_BT));
    bytes_to_str<true, 0>(name+1, mac + 3, 3);
}

uint32_t generate_salt() {
    static uint32_t last_change = __INT32_MAX__, salt; //ESP_LOGD(TAG, "change_device_name()");
    uint32_t sec = xTaskGetTickCount();
    if(sec - last_change > pdMS_TO_TICKS(TIME_CHANGE_PIN)) { 
        last_change = sec;
#if !_ESP_LOG_ENABLED(3)
        ble_delete_all_peers(&bonded_addr);
#endif
        salt = time(NULL); 
        if(salt < 1766600000) salt = 0;
        pincode = TOTPget(passkey, passkey_len, salt); ESP_LOGI(TAG, "NEW PIN: %lu\nTime: %lu", pincode, salt);
    }
    return salt;
}

uint32_t TOTPget(const byte* key, byte key_len, uint32_t time) {
    return HOTPget(key, key_len, time / TOTP_TIMESTEP);
}

uint32_t HOTPget(const byte* key, byte key_len, uint64_t salt) {
    const byte salt_len = sizeof(salt); byte* const pSalt = reinterpret_cast<byte*>(&salt);
    byte hash[20];  uint32_t result;
    swap_in_place(pSalt, salt_len); //salt = ntohll(salt);
    int ret = hmac_hash(hash, key, key_len, pSalt, salt_len); 
    if(ret != 0) return 0;
	for (byte i = 0, offset = hash[19] & 0xF; i < 4; ++i) {
		reinterpret_cast<byte*>(&result)[3 - i] = hash[offset + i];
	}
	result = (result & 0x7FFFFFFF) % 1000000;
	return result;
}

int hmac_hash(byte* hash, const byte* key, size_t key_len, const byte* data, size_t data_len, mbedtls_md_type_t t) {
    mbedtls_md_context_t ctx {/*init*/};
    CHECK_RET(mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(t), 1));
    CHECK_RET(mbedtls_md_hmac_starts(&ctx, key, key_len));
    CHECK_RET(mbedtls_md_hmac_update(&ctx, data,  data_len));
    CHECK_RET(mbedtls_md_hmac_finish(&ctx, hash));
    mbedtls_md_free(&ctx);
    return 0;
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
    if (!encoded || !result) {
        ESP_LOGE(TAG, "String or buffer are null"); return -1;
    }
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
        } else {
            ESP_LOGE(TAG, "Invalid Base32!");
            return -1;
        }
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
    log_d("%08X", *reinterpret_cast<uint32_t*>(&sets));
    if(crc16_le(0, (byte*)&sets, 2) == sets.crc) {
        if(!sets.updated) { img_state(true); } //validate partition if it is verify state
    } else { sets = {}; ESP_LOGW("crc", "");}; 
    //++sets.count;
    sets.crc = crc16_le(0, (byte*)&sets, 2);
}

void write_noinit(byte val) {
    sets.updated = val;
    sets.crc = crc16_le(0, (byte*)&sets, 2); log_d("%u, %u, %04X", sets.patch, sets.updated, sets.crc);
}
