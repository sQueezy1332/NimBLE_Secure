#include "main.h"

extern "C" void app_main() {
    main_init();
    led_init();
    rand_device_name();
    pincode = generate_pin(ble_svc_gap_device_name(), password);
    //pinMode(PIN_LED_D4, PULLUP | OUTPUT_OPEN_DRAIN);
    ESP_RETURN_VOID_ON_ERROR(nimble_port_init(), TAG, "initialize nimble stack"); /*NimBLE stack initialization */
    ESP_RETURN_VOID_ON_ERROR(gap_init(), TAG, "initialize GAP service"); /* GAP service initialization */
    ESP_RETURN_VOID_ON_ERROR(gatt_svc_init(), TAG, "initialize GATT server"); /* GATT server initialization */
    nimble_host_config_init(); /* NimBLE host configuration initialization */
    /* Start NimBLE host task thread and return */
    timerPatch = xTimerCreateStatic("patch", TIMER_PATCH, pdFALSE, NULL,
		[](TimerHandle_t xTimer) { digitalWrite(PIN_LED_D4, 0); sensor_state = 0;}, &xTimerPatchBuffer);
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
    xTaskCreate(heart_rate_task, "Heart Rate", 2*1024, NULL, 5, NULL);
    //auto res = analogReadMilliVolts(0); ESP_LOGD(TAG, "%lu",res);
}

static void nimble_host_config_init() {
    ble_hs_cfg.reset_cb = [](int reason) { ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);}; //on_stack_reset is called when host resets BLE stack due to errors
    ble_hs_cfg.sync_cb = periodic_sync_scan;//adv_init; //on_stack_sync is called when host has synced with controller
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
}

static void nimble_host_task(void *param) {
    ESP_LOGD(TAG, "nimble host task has been started!");
    nimble_port_run();
    vTaskDelete(NULL);
}

static void heart_rate_task(void *param) {
    ESP_LOGD(TAG, "heart rate task has been started!");
    while (1) {
        update_heart_rate(); //ESP_LOGI(TAG, "heart rate updated to %u", get_heart_rate());
        send_heart_rate_indication();
        vTaskDelay(HEART_RATE_TASK_PERIOD);
    }
}

static void rand_device_name() {
    constexpr byte name_len = sizeof(DEVICE_NAME) -1;
    char* pName = const_cast<char*>(ble_svc_gap_device_name());
    pName[name_len] = ' ';
    const uint32_t rand_num = esp_random();
    create_hex_string(pName + name_len + 1, (byte*)&rand_num, sizeof(rand_num));
}

static uint32_t generate_pin(const char *str, const char *pass) {
    const byte name_len = strlen(str), pass_len = strlen(pass);
    byte shaResult[32], payload[name_len + pass_len];
    mbedtls_md_context_t ctx {};
    memcpy(payload, str, name_len);
    memcpy(&payload[name_len], pass, pass_len);
    DEBUGLN((char*)payload);DEBUGF("name_len = %u; ""payloadLen = %u\n", name_len, pass_len);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, payload,  name_len + pass_len);
    mbedtls_md_finish(&ctx, shaResult);
    mbedtls_md_free(&ctx);
    DEBUG("Hash: "); for (byte i = 0; i < sizeof(shaResult); i++) { DEBUGF("%02x", shaResult[i]);} DEBUGLN();
    uint32_t crcResult = crc32_le(0, shaResult, sizeof(shaResult));
    DEBUGF("crcResult = %lu\n", crcResult);
    crcResult = 1000 + crcResult % 999000;
    DEBUGF("pin = %lu\n", crcResult);// return 100000 + esp_random() % 900000,
    return crcResult;
}

void create_hex_string(char* ptr, cbyte* buf, byte data_size) {
	for (byte i = 0, shift, nibble, num;;) {
		for (shift = 4, num = buf[i];; shift = 0) {
			nibble = (num >> shift) & 0xF;
			*ptr++ = nibble < 10 ? nibble ^ 0x30 : nibble + ('A' - 10);
			if (shift == 0) break;
		} if (++i >= data_size) break;
	}*ptr = '\0';
}

void change_device_name() {
    static uint32_t last_change = 0; ESP_LOGI(GAP, "____set_device_name()____");
    TickType_t tick = xTaskGetTickCount();
    if(tick - last_change > TIMER_CHANGE_NAME) { ESP_LOGI(TAG, "TIMER_CHANGE_NAME");
        last_change = tick;
        ble_delete_all_peers();
        rand_device_name();
        pincode = generate_pin(ble_svc_gap_device_name(), password);
    }
}

int ble_delete_all_peers() {
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    ble_addr_t peer_id_addrs[CONFIG_BT_NIMBLE_MAX_BONDS];
    int num_peers;
    ESP_RETURN_ON_ERROR(ble_store_util_bonded_peers(peer_id_addrs, &num_peers, CONFIG_BT_NIMBLE_MAX_BONDS), TAG, "bonded_peers");
    while(num_peers > 0) { 
        ESP_RETURN_ON_ERROR(ble_store_util_delete_peer(&peer_id_addrs[num_peers-1]), TAG, ""); 
        ESP_LOGD(TAG, "num_peers = %u", num_peers); --num_peers;
    }
#endif
    return 0;
}

void get_task_list(String& str) {
	size_t num = uxTaskGetNumberOfTasks(); log_d("NumberOfTasks = %u", num);
	if (!str.reserve((num * 32 + 16) *2)) return;
	char* const ptr = str.begin(); 
	vTaskList(ptr);
    num = strlen(ptr); strcpy(&ptr[num], "Heap:\t"); ultoa(ESP.getFreeHeap(), &ptr[num+6], DEC); num += strlen(&ptr[num]); strcpy(&ptr[num],"\n\n"); num+=2;
	vTaskGetRunTimeStats(&ptr[num]); num += strlen(&ptr[num]);
    reinterpret_cast<uint32_t*>(&str)[2] = num;
}

void print_addr(const uint8_t* const & addr) { 
#ifdef BLE_DEBUG
    for (byte i = 5;;i--) { DEBUGF("%02X", addr[i]); if (i /* < 5 */) DEBUG(':'); else break; }
#endif
}

void parse_adv_data(const uint8_t* const & data, uint8_t data_len) {
#ifdef BLE_DEBUG
    //DEBUG("Raw Data: "); for (byte i = data_len; i--; ) { DEBUGF("%02X ", data[i]); }
    DEBUGF("Data length: \t%u\n", data_len);
    for (byte i = 0, len, type; i < data_len;) {
        if((len = data[i]) > 2) { DEBUG("Len: "); DEBUG(len); DEBUG('\t');}
        type = data[++i]; DEBUG("Type: "); DEBUGF("%02X", type); 
        if(type == COMPLETE_NAME || type == SHORT_NAME) { DEBUG("\tName: "); for(size_t end = i++ + len;i < end; i++) DEBUG((char)data[i]); } 
        else { DEBUG(" { "); for(size_t end = i++ + len;i < end; i++) { DEBUGF("%02X ", data[i]); } DEBUG('}'); } DEBUG(len > 2 ? '\n' : ' ');  
    }
 #endif
}

void print_event_report(const ble_gap_disc_desc & disc) {
    DEBUG("[DEVICE]: \t"); print_addr(disc.addr.val); //BLE_HCI_ADV_RPT_EVTYPE_ADV_IND;//0
    DEBUGF(" (%u)\nAD Event Type:\t%u\nRSSI:\t\t%i\n", disc.addr.type, disc.event_type, disc.rssi);
    if(disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) { DEBUG("Direct address: \t"); print_addr(disc.direct_addr.val); }
	if (disc.length_data) { parse_adv_data(disc.data, disc.length_data); } DEBUGLN('\n');
}

void print_event_report(const ble_gap_ext_disc_desc & disc) {
    DEBUG("[DEVICE]: \t"); print_addr(disc.addr.val);
    DEBUGF(" (%u)\nAD Event Type:\t%u\nRSSI:\t\t%i\n", disc.addr.type, disc.props, disc.rssi);
    if (disc.props & BLE_HCI_ADV_LEGACY_MASK) { DEBUGF("Legacy event: \t%u\n", disc.legacy_event_type); } //BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND; //3
    else { if(disc.tx_power != 127) {DEBUGF("Tx Pwr: \t%i\n", disc.tx_power);}DEBUGF("Prim PHY: \t%u\nSecn PHY: \t%u\nSID:\t\t%d\n", disc.prim_phy, disc.sec_phy, disc.sid); }
    if(disc.props & BLE_HCI_ADV_DIRECT_MASK) { DEBUG("Direct address: \t"); print_addr(disc.direct_addr.val); }
	if (disc.length_data) { parse_adv_data(disc.data, disc.length_data); } DEBUGLN('\n');
}

void print_event_report(const decltype(ble_gap_event::periodic_report) & rep) {
    log_i("Periodic adv report event: \n");
    DEBUGF("sync_handle : %u\n", rep.sync_handle);
    DEBUGF("tx_power : %d\n", rep.tx_power);
    DEBUGF("rssi : %d\n", rep.rssi);
    DEBUGF("data_status : %u\n", rep.data_status);
    DEBUGF("data_length : %u\n", rep.data_length);
    if (rep.data_length) { parse_adv_data(rep.data, rep.data_length); }
}

void print_event_report(const decltype(ble_gap_event::periodic_sync) & rep) {
    log_i("Periodic sync event : \n");
    DEBUGF("status : %d\nperiodic_sync_handle : %d\nsid : %d\n", rep.status, rep.sync_handle, rep.sid);
    DEBUGF("adv addr : "); for (int i = 0; i < 6; i++) {DEBUGF("%02X:",rep.adv_addr.val[i]);}
    DEBUGF("\nadv_phy : %s\n", rep.adv_phy == 1 ? "1m" : (rep.adv_phy == 2 ? "2m" : "coded"));
    DEBUGF("per_adv_ival : %d\n",rep.per_adv_ival);
    DEBUGF("adv_clk_accuracy : %d\n", rep.adv_clk_accuracy); 
}

void print_event_report(const decltype(ble_gap_event::periodic_sync_lost) & rep) {
    log_i("Periodic sync lost\n");
    DEBUGF("sync_handle : %u\n", rep.sync_handle);
    DEBUGF("reason(%i) : %s\n", rep.reason , rep.reason == 13 ? "timeout" : (rep.reason == 14 ? "terminated locally" : "Unknown reason"));
    #ifdef DEBUG_ENABLE
    synced = false;
    #endif
}

uint32_t Kalman(uint32_t kalman){
    const float k = KALMAN_KOEF;
    static float old = kalman;
    kalman = k * kalman + (1 - k) * old;
    return old = kalman;
}
