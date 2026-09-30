#include "common.h"
#include "gap.h"
#include "gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"
#include "rom/crc.h"

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static const char* TAG = "GAP";
//#define RANDOM_ADDR

/* Private function declarations */
const char* format_addr(const uint8_t addr[]) {
	static char buf[18];
	sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", addr[5], addr[4],addr[3], addr[2], addr[1], addr[0]);
	return buf;
}

//__weak_symbol void host_sync_cb() {}
//__weak_symbol uint32_t generate_pin() { return 111111; }
//__weak_symbol uint32_t generate_uuid32() { return 0; }
//__weak_symbol void parse_adv_cb(const struct ble_gap_ext_disc_desc*) {}
//__weak_symbol void ble_adv_complete_cb() {}
//__weak_symbol void ble_scan_complete_cb() {}
//__weak_symbol void ble_connect_cb(int) { }
//__weak_symbol int disconnect_cb(uint16_t) { return 0; }
//__weak_symbol int conn_encrypted_cb(uint16_t) { return 0; }
//__weak_symbol int parse_rx_data(const struct ble_gap_event*) { return 0; }

void set_random_addr(void);
static void print_conn_desc(struct ble_gap_conn_desc *);
static int gap_event_handler(struct ble_gap_event *, void *);
static void parse_adv_data(const uint8_t*, uint8_t);
static void print_rx_data(const struct os_mbuf *);
static void print_event_report_ext(const struct ble_gap_ext_disc_desc*);
static void print_event_report(const struct ble_gap_disc_desc*);
//static void print_event_report(const decltype(ble_gap_event::periodic_report) & rep);
//static void print_event_report(const decltype(ble_gap_event::periodic_sync) & rep);
//static void print_event_report(const decltype(ble_gap_event::periodic_sync_lost) & rep);

__unused bool synced;
__unused static uint8_t own_addr_val[6] = {};
static uint8_t own_addr_type = BLE_HCI_ADV_OWN_ADDR_PUBLIC;

static uint16_t scan_interval = 0;
static uint16_t scan_window = 0;

__NOINIT_ATTR static my_ble_store_t bonds[CONFIG_BT_NIMBLE_MAX_BONDS];
__NOINIT_ATTR static int num_peers;
__NOINIT_ATTR static uint16_t bonds_count;
__NOINIT_ATTR uint16_t crc_bonds;

 //sizeof(ble_gap_conn_desc); //44

/*
 * NimBLE applies an event-driven model to keep TAG service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a TAG event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
	struct ble_gap_conn_desc desc; int ret;
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		ESP_LOGI(TAG, "connection %s; status %d", event->connect.status ? "failed" : "established" ,event->connect.status);
#ifdef DEBUG_LOG		
		if (event->connect.status == 0) {
			if((ret = ble_gap_conn_find(event->connect.conn_handle, &desc))) return ret;
			print_conn_desc(&desc); //BLE_HCI_CONN_ITVL == 1.25; BLE_HCI_SCAN_ITVL == 0.625
			/*struct ble_gap_upd_params params = { .itvl_min = desc.conn_itvl, .itvl_max = desc.conn_itvl, .latency = desc.conn_latency,
											.supervision_timeout = desc.supervision_timeout, //BLE_GAP_SUPERVISION_TIMEOUT_MS ( /10)
											.min_ce_len = 0 , .max_ce_len = 0 };
			if((ret = ble_gap_update_params(event->connect.conn_handle, &params))) return ret; */
		}
#endif
		ble_connect_cb(event->connect.status);
		break;
	case BLE_GAP_EVENT_DISCONNECT: /* A connection was terminated, print connection descriptor */
		ESP_LOGW(TAG, "DISCONNECT from peer; reason 0x%04x",event->disconnect.reason);
		if(event->disconnect.reason == 0x0216) break; //Terminated By Local Host //0x0213 by remote device
		return ble_disconnect_cb(event->disconnect.conn.conn_handle);
	case BLE_GAP_EVENT_CONN_UPDATE:
#ifdef DEBUG_LOG
		ESP_LOGI(TAG, "CONN_UPDATE status %d",event->conn_update.status);
		if((ret = ble_gap_conn_find(event->conn_update.conn_handle, &desc))) { return ret; } //log internal
		print_conn_desc(&desc);
		return ret;
#endif
		return 0;
	case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
		ESP_LOGI(TAG, "PHY_UPDATE_COMPLETE" " %u; conn_handle %u rx_phy %u tx_phy %u",
			event->phy_updated.status, event->phy_updated.conn_handle, event->phy_updated.rx_phy, event->phy_updated.tx_phy);
		break;
	case BLE_GAP_EVENT_DISC_COMPLETE:
		ESP_LOGI(TAG, "DISC_COMPLETE reason %d",event->disc_complete.reason);
		ble_scan_complete_cb();
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		ESP_LOGI(TAG, "ADV_COMPLETE reason %d",event->adv_complete.reason); //start_advertising();
		ret = event->adv_complete.reason;
		if (ret == BLE_HS_ETIMEOUT || (ret != 0 && ret != BLE_HS_EPREEMPTED)) 
		{ ble_adv_complete_cb(); } //BLE_HS_ETIMEOUT (13) //BLE_HS_EPREEMPTED (29)
		break;
	case BLE_GAP_EVENT_NOTIFY_RX:
		ESP_LOGI(TAG,"NOTIFY_RX conn_handle %u attr_handle %u %s",
				event->notify_rx.conn_handle, event->notify_rx.attr_handle,
				event->notify_rx.indication ? "Indication":  "Notification");
		print_rx_data(event->notify_rx.om); //event->notify_rx.conn_handle
		return parse_rx_data_cb(event);
		break;
	case BLE_GAP_EVENT_NOTIFY_TX:
		if (unlikely((event->notify_tx.status != 0) && (event->notify_tx.status != BLE_HS_EDONE))) {
			ESP_LOGW(TAG,"NOTIFY_TX conn_handle %d attr_handle %d " "status %d %s",
					 event->notify_tx.conn_handle, event->notify_tx.attr_handle,
					 event->notify_tx.status, event->notify_tx.indication ? "Indication":  "Notification");
		} else { ESP_LOGV(TAG,"notify_tx.status %d", event->notify_tx.status); }
		break;
	case BLE_GAP_EVENT_SUBSCRIBE:
		ESP_LOGI(TAG,"SUBSCRIBE"" conn_handle %u attr_handle %u " "reason %u notif %u->%u ind %u->%u",
			event->subscribe.conn_handle, event->subscribe.attr_handle, event->subscribe.reason, event->subscribe.prev_notify,
				 event->subscribe.cur_notify, event->subscribe.prev_indicate, event->subscribe.cur_indicate);
		ret = gatt_svr_subscribe_cb(event);
		if (ret == BLE_ATT_ERR_INSUFFICIENT_AUTHEN) {
			return ble_gap_security_initiate(event->subscribe.conn_handle); } /* Request connection encryption */
		return ret;
	case BLE_GAP_EVENT_MTU:
		ESP_LOGI(TAG, "MTU update; conn_handle %u ch id %u mtu %u",
			event->mtu.conn_handle, event->mtu.channel_id,event->mtu.value);
		break;
	case BLE_GAP_EVENT_ENC_CHANGE:/* Encryption change event */
		/* Encryption has been enabled or disabled for this connection. */
		if (likely(event->enc_change.status == 0)) {//"ble_sm.h"
			ESP_LOGI(TAG, "connection encryption status: %d",event->enc_change.status); //13 BLE_HS_ETIMEOUT
	//1035 (0x40B) DHKey Check Failed; 1288 (0x508) BLE_SM_ERR_PAIR_NOT_SUPP; 1281 (0x0501) Passkey Entry Failed
			return ble_conn_encrypted_cb(event->enc_change.conn_handle);
		} else { ESP_LOGW(TAG, "connection encryption status: %d",event->enc_change.status); }
		goto repeat; repeat: //break; //#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
	case BLE_GAP_EVENT_REPEAT_PAIRING: 
	static_assert(MYNEWT_VAL_BLE_HANDLE_REPEAT_PAIRING_DELETION);// break;
		/* Delete the old bond */
		/*if((ret = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc))) { return ret; }
		ble_store_util_delete_peer(&desc.peer_id_addr);
		ESP_LOGW(TAG, "repairing..."); */
		return BLE_GAP_REPEAT_PAIRING_RETRY; //to indicate that the host should continue with pairing operation 
	case BLE_GAP_EVENT_PASSKEY_ACTION:
		if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
			struct ble_sm_io pkey = {
				.action = event->passkey.params.action,
				.passkey = generate_pin()
			};
			ESP_LOGI(TAG, "enter passkey %lu on the peer side", pkey.passkey);
			if((ret = ble_sm_inject_io(event->passkey.conn_handle, &pkey))) return ret;
		} else { ESP_LOGW(TAG, "passkey.params.action %u",event->passkey.params.action); }
		break;
	case BLE_GAP_EVENT_DISC: print_event_report(&event->disc); //LEGACY
		break;
	case BLE_GAP_EVENT_EXT_DISC:
		if(event->ext_disc.data_status != BLE_GAP_EXT_ADV_DATA_STATUS_COMPLETE) {
			ESP_LOGW(TAG,"data_status: %u",event->ext_disc.data_status); break; }
		print_event_report_ext(&event->ext_disc);
		parse_adv_cb(&event->ext_disc);
		break;
	//case BLE_GAP_EVENT_PERIODIC_SYNC: print_event_report(event->periodic_sync); break;
	//case BLE_GAP_EVENT_PERIODIC_REPORT: print_event_report(event->periodic_report); break;
	//case BLE_GAP_EVENT_PERIODIC_SYNC_LOST: print_event_report(event->periodic_sync_lost); break;
	case BLE_GAP_EVENT_LINK_ESTAB: ESP_LOGI(TAG, "LINK_ESTAB"); break;
	case BLE_GAP_EVENT_DATA_LEN_CHG: ESP_LOGI(TAG, "DATA_LEN_CHG"); break;
	case BLE_GAP_EVENT_CONN_UPDATE_REQ: ESP_LOGI(TAG, "CONN_UPDATE_REQ"); break;
	case BLE_GAP_EVENT_PARING_COMPLETE: ESP_LOGI(TAG, "PARING_COMPLETE"); break;
	case BLE_GAP_EVENT_IDENTITY_RESOLVED: ESP_LOGI(TAG, "IDENTITY_RESOLVED");break;
	case BLE_GAP_EVENT_AUTHORIZE: ESP_LOGI(TAG, "AUTHORIZE"); break;
	default: ESP_LOGW(TAG, "event->type %u",event->type);
	}
	return ESP_OK;
}

void adv_init(uint16_t field, uint32_t duration_units) {
	if(ble_gap_ext_adv_active(0)) return;
	//const ble_uuid32_t uuid32 = BLE_UUID32_INIT(generate_uuid32());
	__unused const ble_uuid16_t uuid16 = BLE_UUID16_INIT(field);
	__unused const uint8_t esp_uri[] = { BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f', '.', 'c', 'o', 'm'};
	__unused struct ble_hs_adv_fields rsp_fields = {
		.adv_itvl = BLE_GAP_ADV_ITVL_MS(500),.adv_itvl_is_present = 1,
		//.device_addr = own_addr_val;
		.device_addr_type = own_addr_type, .device_addr_is_present = 1,
		.uri = esp_uri, .uri_len = sizeof(esp_uri),
	}; const uint8_t *name = (uint8_t *)ble_svc_gap_device_name();
	__unused struct ble_hs_adv_fields adv_fields = {
		.flags = BLE_HS_ADV_F_DISC_LTD  | BLE_HS_ADV_F_BREDR_UNSUP , //Type 0x01 Basic Rate / Enhanced Data Rate (BT Classic)
		.uuids16 =  &uuid16, .num_uuids16 = 1, .uuids16_is_complete = 1, //0x03
		//.uuids32 = &uuid32, .num_uuids32 = 1, .uuids32_is_complete = 1, //0x05
		.name = name,
		.name_len = name[MYNEWT_VAL_BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH], // strlen((char*)name);
		.name_is_complete = 1,       // Type 0x09
		.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
		//.tx_pwr_lvl_is_present = 1, // Type 0x0A
		.appearance = ble_svc_gap_device_appearance(),
		.appearance_is_present = 1, // Type 0x19
		.le_role = BLE_GAP_LE_ROLE_PERIPHERAL,
		.le_role_is_present = 1, // Type 0x1C
	}; __unused int8_t tx_pwr;
#if NIMBLE_BLE_ADVERTISE && MYNEWT_VAL_BLE_EXT_ADV
	struct ble_gap_ext_adv_params ext_adv_cfg = {}; 
	ext_adv_cfg.connectable = 1;
	ext_adv_cfg.scan_req_notif = 1;
	ext_adv_cfg.include_tx_power = 1;
	ext_adv_cfg.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
	ext_adv_cfg.itvl_max = BLE_GAP_ADV_ITVL_MS(510);
	ext_adv_cfg.own_addr_type = own_addr_type; //not random
	ext_adv_cfg.primary_phy = BLE_HCI_LE_PHY_1M;
	ext_adv_cfg.secondary_phy = BLE_HCI_LE_PHY_CODED;
	ext_adv_cfg.tx_power = 21;
	int ret = ble_gap_ext_adv_configure(0, &ext_adv_cfg, &tx_pwr,gap_event_handler, NULL);
	if(unlikely(ret == BLE_HS_ENOMEM)) { return; }
	else if(ret != 0) { assert(0); } 
	/* Default to legacy PDUs size, mbuf chain will be increased if needed */
	struct os_mbuf *data = os_msys_get_pkthdr(BLE_HCI_MAX_ADV_DATA_LEN, 0);// assert(data);
	ESP_ERROR_CHECK(ble_hs_adv_set_fields_mbuf(&adv_fields, data));
	ESP_ERROR_CHECK(ble_gap_ext_adv_set_data(0, data));
	ESP_ERROR_CHECK(ble_gap_ext_adv_start(0, duration_units, 0));
	//ESP_LOGI(TAG, "advertising started! tx_pwr %d", tx_pwr);
#else
	__unused ble_gap_adv_params adv_cfg = {
	/* Set non-connetable and general discoverable mode to be a beacon */
		.conn_mode = BLE_GAP_CONN_MODE_UND,
		.disc_mode = BLE_GAP_DISC_MODE_GEN,
		.itvl_min = BLE_GAP_ADV_ITVL_MS(500),
		.itvl_max = BLE_GAP_ADV_ITVL_MS(515),
	};
	ESP_ERROR_CHECK(ble_gap_adv_set_fields(&adv_fields));
	//ESP_ERROR_CHECK(ble_gap_adv_rsp_set_fields(&rsp_fields));
	ESP_ERROR_CHECK(ble_gap_adv_start(own_addr_type, NULL, TIMER_ADV, &adv_cfg, gap_event_handler,NULL));
	ESP_LOGI(TAG, "advertising started!");
#endif
}

void set_scan_periods(uint16_t interval, uint16_t window) { scan_interval = interval, scan_window = window; }

void ble_scan_init() {
	if(ble_gap_disc_active()) return;
	//__unused struct ble_gap_disc_params disc_params = { .itvl = 0, .window = 0, .filter_policy = 0, .limited = 0, .passive = 1, .filter_duplicates = 1, .disable_observer_mode = 0};
	__unused struct ble_gap_ext_disc_params ext_params = { 
		.itvl = scan_interval,
		.window = scan_window,
		.passive = SCAN_PASSIVE,
		.disable_observer_mode = 0 
	};
	//ESP_RETURN_VOID_ON_ERROR(ble_hs_id_infer_auto(0, &own_addr_type), TAG, "determining address type");
	//ESP_RETURN_VOID_ON_ERROR(ble_gap_disc(BLE_ADDR_PUBLIC,0,&disc_params, gap_event_handler, NULL), TAG, "ble_gap_disc");
	ESP_ERROR_CHECK(ble_gap_ext_disc(own_addr_type, 0, 0, 1, 0, 0,&ext_params,&ext_params, gap_event_handler, NULL));
}

static void ble_stack_reset(int reason) { ESP_LOGW("NimBLE", "nimble stack reset, reason: %d", reason);}; 

static int my_ble_store_comparator(const void *a, const void *b) {
    const struct ble_store_value_sec *sec_a = ((struct ble_store_value_sec*)a);
    const struct ble_store_value_sec *sec_b = ((struct ble_store_value_sec*)b);
    return (signed)sec_a->bond_count - (signed)sec_b->bond_count;
}

static int my_ble_store_find(const struct ble_store_key_sec *key_sec, const my_ble_store_t* value_secs, size_t num_value_secs) {
	if(num_value_secs) {
		size_t i = key_sec->idx;
		((struct ble_store_key_sec *)key_sec)->idx = 0;
		uint64_t key = *(uint64_t*)&key_sec->peer_addr;
		if(key == 0x00) {
			ESP_LOGI(TAG, "finding BLE_ADDR_ANY key idx: %u", i);
			if(i < num_value_secs)
				return i;
			return -1;
		}
		for (i = 0; i < num_value_secs; i++) {
			const struct ble_store_value_sec* cur = &value_secs[i];
			if (key == *(uint64_t*)&(cur->peer_addr)) {
                ESP_LOGI(TAG, "finded addr [%u]: %s",i, format_addr(cur->peer_addr.val));
				return i;
            }
		}
	}
	return -1;
}
//extern int ble_store_config_delete_obj(void *, int, int, int *); 
static int ble_store_config_delete_obj(void *values, size_t value_size, size_t idx, int *count) {
    uint8_t *dst, *src;
    size_t move_count;
    //BLE_HS_DBG_ASSERT(idx >= 0 && idx < *num_values && *num_values > 0);
	assert(idx < *count);
    if (idx < --(*count)) { //if only 1 bond in record simply change count to 0
        dst = values;
        dst += idx * value_size;
        src = dst + value_size;
        move_count = *count - idx;
        memmove(dst, src, move_count * value_size);
    }
    return 0;
}

static int ble_store_config_delete_hook(int obj_type, const union ble_store_key *key) {
	ESP_LOGI(TAG, "delete obj_type %u", obj_type);
	int idx;
	switch (obj_type) {
		case BLE_STORE_OBJ_TYPE_OUR_SEC: 
			break;
		case BLE_STORE_OBJ_TYPE_PEER_SEC: 
			idx = my_ble_store_find(&key->sec, bonds, num_peers);
			if(idx == -1) 
				break;
			return ble_store_config_delete_obj(bonds, sizeof(bonds[0]), idx, &num_peers);
			break;
		case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
			break;
		default: ESP_LOGD(TAG, "\tBLE_HS_EDISABLED");
			break;
	}
	return ble_store_config_delete(obj_type, key);
}

static int ble_store_config_read_hook(int obj_type, const union ble_store_key *key, union ble_store_value *value) {
	extern struct ble_store_value_local_irk ble_store_config_local_irks[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
	extern int ble_store_config_num_local_irks;
	// = ble_store_config_read(obj_type, key, value);
	//ESP_LOGI(TAG, "read obj_type %u %c", obj_type, idx == 0 ? '+' : '-');
	ESP_LOGI(TAG, "read obj_type %u", obj_type);
	switch (obj_type) {
		case BLE_STORE_OBJ_TYPE_OUR_SEC:
		case BLE_STORE_OBJ_TYPE_PEER_SEC:
			break;
		case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
			return ble_store_config_read(obj_type, key, value);
		default:
			ESP_LOGD(TAG, "\tBLE_HS_ENOENT");
			return BLE_HS_ENOENT;
	}
		int idx = my_ble_store_find(&key->sec, bonds, num_peers);
		if (idx == -1) {
        	return BLE_HS_ENOENT;
    	}
		value->sec = bonds[idx];
		if(obj_type == BLE_STORE_OBJ_TYPE_OUR_SEC) {
			if(likely(ble_store_config_num_local_irks)) {
				uint8_t *irk = ble_store_config_local_irks[ble_store_config_num_local_irks].irk;
				memcpy(&value->sec.irk, irk, sizeof(ble_store_config_local_irks->irk));
				value->sec.irk_present = 1;
			} else { value->sec.irk_present = 0; }
		}
	return 0;
}

static int ble_store_config_write_hook(int obj_type, const union ble_store_value *val) {
	ESP_LOGI(TAG, "write obj_type %u", obj_type);
	switch (obj_type) {
		//case BLE_STORE_OBJ_TYPE_OUR_SEC:ptr = &bonds[0];break;
		case BLE_STORE_OBJ_TYPE_PEER_SEC:
			break;
		case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
			return ble_store_config_write(obj_type, val); 
		default: ESP_LOGD(TAG, "\tBLE_HS_EDISABLED");
			return BLE_HS_EDISABLED; 
	}
    int idx = my_ble_store_find((struct ble_store_key_sec*)&val->sec, bonds, num_peers);
    if (idx == -1) {
        if (num_peers >= sizeof(bonds) / sizeof(bonds[0])) {
            ESP_LOGD(TAG, "error persisting peer sec; too many entries ""(%d)\n", num_peers);
            return BLE_HS_ENOMEM; //return BLE_HS_ESTORE_CAP;
        }
        idx = num_peers;
        (num_peers)++;
		ESP_LOGI(TAG, "new peer № %u", num_peers);
    }
    bonds[idx] = val->sec;
    bonds[idx].bond_count = ++bonds_count;
	ESP_LOGD(TAG, "bond_count: %d", bonds_count);
    /* Ensure entries are sorted at all times */
    qsort(bonds, num_peers, sizeof(my_ble_store_t), my_ble_store_comparator);
    /*if (bonds_count > (UINT16_MAX - 5)) { //not need because buffer is temporary
        rc = ble_restore_peer_sec_nvs();
        if (rc != 0) {
            return rc;
        }
    }*/
   	crc_bonds = crc16_le(0, (uint8_t*)&bonds, sizeof(bonds)); //TODO
	return 0;
}

void ble_hs_cfg_init() {
	static_assert(!MYNEWT_VAL_BLE_STATIC_TO_DYNAMIC);
	static_assert(!CONFIG_BT_NIMBLE_MAX_CCCDS);
	static_assert(sizeof(struct ble_store_value_sec) == 88);
	const struct ble_store_value_sec tmp;
	int offset = (((size_t)&tmp.bond_count - (size_t)&tmp.peer_addr));
	assert(offset == 8); //check padding for my_ble_store_find and find by ble_store_value
	static_assert(MYNEWT_VAL_BLE_SM_BONDING);
	static_assert(MYNEWT_VAL_BLE_SM_SC);
	//static_assert(MYNEWT_VAL_BLE_SM_SC_ONLY);
	static_assert(MYNEWT_VAL_BLE_SM_LVL == 4);
	//ble_hs_cfg.sm_sec_lvl = 3;
	ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY,
    //ble_hs_cfg.sm_oob_data_flag = MYNEWT_VAL(BLE_SM_OOB_DATA_FLAG)
    ble_hs_cfg.sm_mitm = 1;
    /** Security manager settings (continued). */
    ble_hs_cfg.sm_our_key_dist = (BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    ble_hs_cfg.sm_their_key_dist = (BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
	ble_hs_cfg.reset_cb = ble_stack_reset;//on_stack_reset is called when host resets BLE stack due to errors
	ble_hs_cfg.sync_cb = host_sync_cb;
	ble_hs_cfg.gatts_register_cb = gatt_register_cb;
	ble_hs_cfg.store_read_cb = ble_store_config_read_hook;
	ble_hs_cfg.store_write_cb = ble_store_config_write_hook;
	//ble_hs_cfg.store_write_cb = ble_store_config_write;
	ble_hs_cfg.store_delete_cb = ble_store_config_delete_hook;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	ble_store_config_conf_init(); //ble_store_config_init(); //STORAGE
	if(crc_bonds != crc16_le(0, (uint8_t*)&bonds, sizeof(bonds))) { //TODO
		ESP_LOGW(TAG, "!crc_bonds");
		memset(bonds, 0, sizeof(bonds));
		num_peers = 0; bonds_count = 0;
		crc_bonds = crc16_le(0, (uint8_t*)&bonds, sizeof(bonds));
	}
}

/**
 *	OUR_SEC (1), PEER_SEC (2)
 *	CCCD - Client Characteristic Configuration Descriptor (3)
 *	RPA rec - Resolvable Private Address record (6)
 *	IRK - Identity Resolving Key (7)
 *	CSFC - Client Supported Features Characteristic 	(8)
 *  CSRK - Connection Signature Resolving Key
 *	LTK - Long Term Key
 *	BLE_HS_ESTORE_CAP if the database is full.
 *	
 */
int save_bonding(uint16_t h_conn) {
	struct ble_gap_conn_desc desc;
	int rc = ble_gap_conn_find(h_conn, &desc);
	if(rc) return rc; //log internal
	if (desc.peer_id_addr.type & BLE_ADDR_RANDOM) {
		ESP_LOGW(TAG,"peer_id_addr.type %u" ,desc.peer_id_addr.type);
		return BLE_HS_ENOADDR;
	}
	if(!desc.sec_state.encrypted) {
		ESP_LOGW(TAG,"sec_state.encrypted %u", desc.sec_state.encrypted);
		return BLE_HS_EENCRYPT;
	}
	print_conn_desc(&desc);
	union ble_store_key key = { .sec.peer_addr = desc.peer_id_addr, .sec.idx = 0 };
	rc = my_ble_store_find(&key.sec, bonds, num_peers);
	if (rc == -1) {
		rc = my_ble_store_find(&key.sec, bonds, num_peers);
		if (rc == -1) return BLE_HS_ENOENT;
	}
	ble_hs_cfg.store_write_cb = ble_store_config_write;
	///rc = ble_store_write(BLE_STORE_OBJ_TYPE_OUR_SEC, (union ble_store_value *)&bonds->peer_secs); //1
		rc = ble_store_write(BLE_STORE_OBJ_TYPE_PEER_SEC, (union ble_store_value *)&bonds[rc]); //2
	ble_hs_cfg.store_write_cb = ble_store_config_write_hook;
	return rc;
}

__unused void set_random_addr(void) {
#ifdef RANDOM_ADDR
	set_random_addr();
	int rc; ble_addr_t addr;
	rc = ble_hs_id_gen_rnd(0, &addr); assert(rc == 0);
	rc = ble_hs_id_set_rnd(addr.val); assert(rc == 0);
	ESP_RETURN_VOID_ON_ERROR(ble_hs_util_ensure_addr(true),TAG,  "device does not have any available bt address!");
	/* Figure out BT address to use while advertising */
	ESP_RETURN_VOID_ON_ERROR(ble_hs_id_infer_auto(0, &own_addr_type), TAG, "infer address type");
	/* Copy device address to own_addr_val */
	ESP_RETURN_VOID_ON_ERROR(ble_hs_id_copy_addr(own_addr_type, own_addr_val, NULL), TAG, "copy device address");
	ESP_RETURN_VOID_ON_ERROR(ble_svc_gap_device_name_set(DEVICE_NAME), TAG, "set device name to %s", DEVICE_NAME);
	ESP_LOGI(TAG, "device address: %s", format_addr(own_addr_val));
	return;
#endif
	assert(0);
}

int is_connection_encrypted(uint16_t h_conn) {
	struct ble_gap_conn_desc desc;
	if(!ble_gap_conn_find(h_conn, &desc)) return desc.sec_state.encrypted; //log internal
	return 0;
}

	//PRINT FUNCTIONS
void print_conn_desc(struct ble_gap_conn_desc *desc) {
	ESP_LOGI(TAG, "connection handle: %u", desc->conn_handle);
	ESP_LOGI(TAG, "local address: %s (%u)", format_addr(desc->our_id_addr.val), desc->our_id_addr.type);
	ESP_LOGI(TAG, "peer address: %s (%u)", format_addr(desc->peer_id_addr.val), desc->peer_id_addr.type);
	ESP_LOGI(TAG, "itvl %ums, latency %d, timeout %ums, encr %u, auth %u, bonded %u, key_size %u",
		(uint32_t)desc->conn_itvl * BLE_HCI_CONN_ITVL / 1000, desc->conn_latency, desc->supervision_timeout * 10,
			 desc->sec_state.encrypted, desc->sec_state.authenticated,desc->sec_state.bonded,
			 desc->sec_state.key_size);
}

void parse_adv_data(const uint8_t* data, uint8_t data_len) {
#ifdef DEBUG_LOG
	//NIMLOG("Raw Data: "); for (uint8_t i = 0; i < data_len; i++) { NIMLOG("%02X ", data[i]); }
	NIMLOG("Data length: \t%u\n", data_len);
	for (size_t i = 0; i < data_len;) {
		uint8_t len = data[i]; if(len > 2) { NIMLOG("Len: %u\t", len);}
		uint8_t type = data[++i]; NIMLOG("Type: "); NIMLOG("%02X", type);
		if(type == COMPLETE_NAME || type == SHORT_NAME ) {
			NIMLOG(" Name: "); ++i;
			if(len < 2 ) continue;
			const char* name = (char*)(&data[i]); i += --len;
			NIMLOG("%.*s", len, name);
		}
		else { NIMLOG(" { "); for(size_t end = i + len;++i < end;) { NIMLOG("%02X ", data[i]); } NIMLOG("}"); }
		NIMLOG(len > 2 ? "\n" : " ");
	}//NIMLOG("\n");
#endif
}

void print_rx_data(const struct os_mbuf *buf) { //notify_rx.om->om_len = %u
#ifdef DEBUG_LOG
	__unused size_t len = buf->om_len; __unused uint8_t * om_data = buf->om_data;
	if(!len) { return; }
	NIMLOG("Data len = %u, Data: ", buf->om_len);
	NIMLOG(" { "); for(size_t i = 0; i < len; i++) {
		NIMLOG("%02X ", om_data[i]); }
		NIMLOG("}\n"); //NIMLOG(len > 2 ? "\n" : " ");
#endif
}

void print_props_mask(uint8_t props) {
	__unused const char* str[] = { "CONN", "SCAN" , "DIR", "RESP" , "LEGA" };
	for (uint8_t m = 16; m; m>>=1) { NIMLOG("%c",props & m ? '1': '0'); } NIMLOG("\t");
	for (size_t i = 0; i < 5; i++) { if(props & (1 << i)) { NIMLOG(str[i]); NIMLOG(", "); }  };
}

void print_legacy_event(uint8_t type) {
	__unused const char* str[] = { "DIR" , "SCAN", "NONCONN" , "RESP" };
	if(type != 0 && type < 5) { NIMLOG(str[type - 1]); }; NIMLOG("\n"); //BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND; //3
}

void print_event_report_ext(const struct ble_gap_ext_disc_desc* disc) {
	NIMLOG("\n%s (%u)",format_addr(disc->addr.val), disc->addr.type);
	NIMLOG("\nAD Event Mask: "); print_props_mask(disc->props);
	NIMLOG("\nRSSI:\t\t%d\n", disc->rssi);
	if (disc->props & BLE_HCI_ADV_LEGACY_MASK) { 
		NIMLOG("Legacy event: \t%u\t", disc->legacy_event_type); print_legacy_event(disc->legacy_event_type);
	}
	else { if(disc->tx_power != 127) { NIMLOG("Tx Power: \t%d\n", disc->tx_power); }
		NIMLOG("Prim PHY: \t%u\nSecn PHY: \t%u\nSID:\t\t%d\n", disc->prim_phy, disc->sec_phy, disc->sid); 
	}
	if(disc->props & BLE_HCI_ADV_DIRECT_MASK) { NIMLOG("Direct address: \t%s", format_addr(disc->direct_addr.val));}
	if (disc->length_data) { parse_adv_data(disc->data, disc->length_data); } NIMLOG("\n");
}

void print_event_report(const struct ble_gap_disc_desc* disc) {
	 //BLE_HCI_ADV_RPT_EVTYPE_ADV_IND;//0
	NIMLOG("\n%s (%u)\nAD Event Type:\t%u\nRSSI:\t\t%d\n", format_addr(disc->addr.val), disc->addr.type, disc->event_type, disc->rssi);
	if(disc->event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) { NIMLOG("Direct address: \t%s", format_addr(disc->direct_addr.val));}
	if (disc->length_data) { parse_adv_data(disc->data, disc->length_data); } NIMLOG("\n");
}

// void disconnect_all_connections(void) {
//     struct ble_gap_conn_desc desc;
//     uint16_t conn_handle;
//     int rc;

//     for (int i = 0; i < BLE_HS_CONN_COUNT; i++) {
//         rc = ble_gap_conn_find_by_idx(i, &desc);ble_hs_sched_reset
//         if (rc == 0) {
//             conn_handle = desc.conn_handle;
//             ESP_LOGI("BLE", "Disconnecting handle: %d", conn_handle);
//             ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
//         }
//     }
// }

/*
void print_event_report(const decltype(ble_gap_event::periodic_report) & rep) {
	ESP_LOGI(TAG, "Periodic adv report event: \n");
	NIMLOG("sync_handle : %u\n", rep.sync_handle);
	NIMLOG("tx_power : %d\n", rep.tx_power);
	NIMLOG("rssi: %d\n", rep.rssi);
	NIMLOG("data_status : %u\n", rep.data_status);
	NIMLOG("data_length : %u\n", rep.data_length);
	if (rep.data_length) { parse_adv_data(rep.data, rep.data_length); }
}

void print_event_report(const decltype(ble_gap_event::periodic_sync) & rep) {
	ESP_LOGI(TAG, "Periodic sync event:");
	NIMLOG("status: %d\nperiodic_sync_handle : %d\nsid : %d\n", rep.status, rep.sync_handle, rep.sid);
	NIMLOG("adv addr: %s", format_addr(rep.adv_addr.val));
	NIMLOG("adv_phy: %s\n", rep.adv_phy == 1 ? "1m" : (rep.adv_phy == 2 ? "2m" : "coded"));
	NIMLOG("per_adv_ival: %d\n",rep.per_adv_ival);
	NIMLOG("adv_clk_accuracy: %d\n", rep.adv_clk_accuracy);
}

void print_event_report(const decltype(ble_gap_event::periodic_sync_lost) & rep) {
#if _ESP_LOG_ENABLED(3)
	ESP_LOGI(TAG, "Periodic sync lost");
	NIMLOG("sync_handle: %u\n", rep.sync_handle);
	NIMLOG("reason (%d): %s\n", rep.reason,
		rep.reason == BLE_HS_ETIMEOUT ? "timeout" : (rep.reason == BLE_HS_EDONE ? "terminated locally" : "Unknown reason"));
	synced = false;
#endif
}*/
