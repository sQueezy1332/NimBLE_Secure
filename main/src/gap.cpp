/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "common.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"
#include "gap.h"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
//#include "gatt_svc.h"
static const char* TAG = "Gap";
//#define RANDOM_ADDR
__unused int RSSI;
/* Private function declarations */
static const char* format_addr(const uint8_t addr[]) {
	static char buf[18];
	sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4],addr[3], addr[2], addr[1], addr[0]);
	return buf;
}

void host_sync_cb() { ble_scan_init();/*set_random_addr(); adv_init();*/}
void set_random_addr(void);
static void print_conn_desc(struct ble_gap_conn_desc *);
static int gap_event_handler(struct ble_gap_event *, void *);
static void parse_adv_data(const uint8_t* data, uint8_t data_len);
static void print_rx_data(const struct os_mbuf *);
static void print_event_report(const ble_gap_disc_desc & disc);
static void print_event_report(const ble_gap_ext_disc_desc & disc);
static void print_event_report(const decltype(ble_gap_event::periodic_report) & rep);
static void print_event_report(const decltype(ble_gap_event::periodic_sync) & rep);
static void print_event_report(const decltype(ble_gap_event::periodic_sync_lost) & rep);
/* Private variables */
__unused static bool synced;
__unused static uint8_t own_addr_type = BLE_HCI_ADV_OWN_ADDR_PUBLIC;
__unused static uint8_t addr_val[6] = {};
__unused static uint8_t esp_uri[] = {BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f', '.', 'c', 'o', 'm'};
ble_gap_conn_desc desc; //sizeof(ble_gap_conn_desc); //44
/*
 * NimBLE applies an event-driven model to keep TAG service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a TAG event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		ESP_LOGI(TAG, "connection %s; status %d", event->connect.status ? "failed" : "established" ,event->connect.status);
		if (event->connect.status == 0) {
			if(int ret = ble_gap_conn_find(event->connect.conn_handle, &desc)) return ret;
			print_conn_desc(&desc);
			ble_gap_upd_params params = {.itvl_min = desc.conn_itvl, .itvl_max = desc.conn_itvl, .latency = desc.conn_latency,
											.supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(1000),
											.min_ce_len = 0 , .max_ce_len = 0 };
			if(int ret = ble_gap_update_params(event->connect.conn_handle, &params)) return ret;
		}
		else { adv_init(); }/* Connection failed, restart advertising */
		break;
	case BLE_GAP_EVENT_DISCONNECT: /* A connection was terminated, print connection descriptor */
		ESP_LOGI(TAG, "DISCONNECT from peer; reason 0x%04x",event->disconnect.reason);
		if(event->disconnect.reason == 0x0216) break; //Terminated By Local Host //0x0213 by remote device
		clear_characteristic();
		adv_init();
		break;
	case BLE_GAP_EVENT_CONN_UPDATE:
		ESP_LOGI(TAG, "CONN_UPDATE status %d",event->conn_update.status);
		if(int ret = ble_gap_conn_find(event->conn_update.conn_handle, &desc)) return ret;
		print_conn_desc(&desc);
		break;
	case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
		ESP_LOGI(TAG, "PHY_UPDATE_COMPLETE" " %u; conn_handle %u rx_phy %u tx_phy %u",
			event->phy_updated.status, event->phy_updated.conn_handle, event->phy_updated.rx_phy, event->phy_updated.tx_phy);
	 break;
	case BLE_GAP_EVENT_DISC_COMPLETE:
		ESP_LOGI(TAG, "DISC_COMPLETE reason %d",event->disc_complete.reason);
		adv_init();
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		ESP_LOGI(TAG, "ADV_COMPLETE reason %d",event->adv_complete.reason); //start_advertising();
		if (event->adv_complete.reason != 0) { ble_scan_init(); unpatch_cb(); }//BLE_HS_ETIMEOUT (13) //BLE_HS_EPREEMPTED (29)
		break;
	case BLE_GAP_EVENT_NOTIFY_RX:
		ESP_LOGI(TAG,"NOTIFY_RX conn_handle %u attr_handle %u %s",
				event->notify_rx.conn_handle, event->notify_rx.attr_handle,
				event->notify_rx.indication ? "Indication":  "Notification");
		print_rx_data(event->notify_rx.om); //event->notify_rx.conn_handle
		parse_rx_data(event);
		break;
	case BLE_GAP_EVENT_NOTIFY_TX:
		if ((event->notify_tx.status != 0) && (event->notify_tx.status != BLE_HS_EDONE)) {
			ESP_LOGW(TAG,"NOTIFY_TX conn_handle %d attr_handle %d " "status %d %s",
					 event->notify_tx.conn_handle, event->notify_tx.attr_handle,
					 event->notify_tx.status, event->notify_tx.indication ? "Indication":  "Notification");
		} else { ESP_LOGV(TAG,"notify_tx.status %d", event->notify_tx.status); }
		break;
	case BLE_GAP_EVENT_SUBSCRIBE:
		ESP_LOGI(TAG,"SUBSCRIBE"" conn_handle %u attr_handle %u " "reason %u prevN %u curN %u prevI %u curI %u",
			event->subscribe.conn_handle, event->subscribe.attr_handle, event->subscribe.reason, event->subscribe.prev_notify,
				 event->subscribe.cur_notify, event->subscribe.prev_indicate, event->subscribe.cur_indicate);
		if (gatt_svr_subscribe_cb(event) == BLE_ATT_ERR_INSUFFICIENT_AUTHEN) {
			return ble_gap_security_initiate(event->subscribe.conn_handle); } /* Request connection encryption */
		break;
	case BLE_GAP_EVENT_MTU:
		ESP_LOGI(TAG, "MTU update; conn_handle %u ch id %u mtu %u",
			event->mtu.conn_handle, event->mtu.channel_id,event->mtu.value);
		break;
	case BLE_GAP_EVENT_ENC_CHANGE:/* Encryption change event */
		/* Encryption has been enabled or disabled for this connection. */
		if (event->enc_change.status == 0) {
			ESP_LOGI(TAG, "connection encrypted!"); //ESP_LOGD(TAG,"enc_change.conn_handle = %u", event->enc_change.conn_handle);
			set_encryption();
			impl_io_on();
		} else { ESP_LOGW(TAG, "connection encryption failed, status: %d",event->enc_change.status); }
		break;
	case BLE_GAP_EVENT_REPEAT_PAIRING:
		/* Delete the old bond */
		if(int ret = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc)) return ret;
		ble_store_util_delete_peer(&desc.peer_id_addr);
		ESP_LOGW(TAG, "repairing..."); //Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should continue with pairing operation
		return BLE_GAP_REPEAT_PAIRING_RETRY;
	case BLE_GAP_EVENT_PASSKEY_ACTION:
		if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
			struct ble_sm_io pkey = {
				.action = event->passkey.params.action,
				.passkey = get_pincode()
			};
			ESP_LOGI(TAG, "enter passkey %lu on the peer side",pkey.passkey);
			if(int ret = ble_sm_inject_io(event->passkey.conn_handle, &pkey)) return ret;
		} else { ESP_LOGW(TAG, "passkey.params.action %u",event->passkey.params.action); }
		break;
	case BLE_GAP_EVENT_DISC: print_event_report(event->disc); //LEGACY
		break;
	case BLE_GAP_EVENT_EXT_DISC:
		if(event->ext_disc.data_status != BLE_GAP_EXT_ADV_DATA_STATUS_COMPLETE) {
			ESP_LOGW(TAG,"data_status: %u",event->ext_disc.data_status); break; }
		print_event_report(event->ext_disc);
		//RSSI = event->ext_disc.rssi;
		parse_adv_cb(event->ext_disc.data, event->ext_disc.length_data);
		break;
	case BLE_GAP_EVENT_PERIODIC_SYNC: print_event_report(event->periodic_sync);
		break;
	case BLE_GAP_EVENT_PERIODIC_REPORT: print_event_report(event->periodic_report);
		break;
	case BLE_GAP_EVENT_PERIODIC_SYNC_LOST: print_event_report(event->periodic_sync_lost);
		break;
	case BLE_GAP_EVENT_LINK_ESTAB: ESP_LOGI(TAG, "LINK_ESTAB"); break;
	case BLE_GAP_EVENT_DATA_LEN_CHG: ESP_LOGI(TAG, "DATA_LEN_CHG"); break;
	case BLE_GAP_EVENT_CONN_UPDATE_REQ: ESP_LOGI(TAG, "CONN_UPDATE_REQ"); break;
	case BLE_GAP_EVENT_PARING_COMPLETE: ESP_LOGI(TAG, "PARING_COMPLETE");break;
	case BLE_GAP_EVENT_IDENTITY_RESOLVED: ESP_LOGI(TAG, "IDENTITY_RESOLVED");break;
	case BLE_GAP_EVENT_AUTHORIZE: ESP_LOGI(TAG, "AUTHORIZE"); break;
	default: ESP_LOGI(TAG, "event->type %u",event->type);
	}
	return ESP_OK;
}

/* Public functions */
void adv_init(void) {
	static const uint8_t device_name_len = strlen(ble_svc_gap_device_name());
	const ble_uuid32_t uuid32= { 32, generate_salt() };
	__unused const ble_uuid16_t uuid16 = { 16,0x1805 };
	__unused struct ble_hs_adv_fields rsp_fields = {
		.adv_itvl = BLE_GAP_ADV_ITVL_MS(500),.adv_itvl_is_present = 1,
		//.device_addr = addr_val;
		.device_addr_type = own_addr_type, .device_addr_is_present = 1,
		.uri = esp_uri, .uri_len = sizeof(esp_uri),
	};
	__unused ble_hs_adv_fields adv_fields = {
		.flags = BLE_HS_ADV_F_DISC_GEN  | BLE_HS_ADV_F_BREDR_UNSUP , // Type 0x01
		//.uuids16 =  &uuid16, .num_uuids16 = 1, .uuids16_is_complete = 1, //0x03
		.uuids32 = &uuid32, .num_uuids32 = 1, .uuids32_is_complete = 1, //0x05
		.name = (uint8_t *)ble_svc_gap_device_name(),
		.name_len = device_name_len, // strlen((char*)name);
		.name_is_complete = 1,       // Type 0x09
		//tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
		//.tx_pwr_lvl_is_present = 1, // Type 0x0A
		.appearance = BLE_GAP_APPEARANCE,
		.appearance_is_present = 1, // Type 0x19
		.le_role = BLE_GAP_LE_ROLE_PERIPHERAL,
		.le_role_is_present = 1, // Type 0x1C
	};
#if NIMBLE_BLE_ADVERTISE && MYNEWT_VAL(BLE_EXT_ADV)
	ble_gap_ext_adv_params ext_adv_cfg = {}; __unused int8_t tx_pwr;
	ext_adv_cfg.connectable = 1;
	ext_adv_cfg.scan_req_notif = 1;
	ext_adv_cfg.include_tx_power = 1;
	ext_adv_cfg.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
	ext_adv_cfg.itvl_max = BLE_GAP_ADV_ITVL_MS(515);
	ext_adv_cfg.own_addr_type = 0; //not random
	ext_adv_cfg.primary_phy = BLE_HCI_LE_PHY_1M;
	ext_adv_cfg.secondary_phy = BLE_HCI_LE_PHY_CODED;
	ext_adv_cfg.tx_power = 21;
	ESP_ERROR_CHECK(ble_gap_ext_adv_configure(0,&ext_adv_cfg, &tx_pwr,gap_event_handler, NULL));
	/* adv_fields.tx_pwr_lvl = 10; */
	/* Default to legacy PDUs size, mbuf chain will be increased if needed */
	os_mbuf *data = os_msys_get_pkthdr(BLE_HCI_MAX_ADV_DATA_LEN, 0); assert(data);
	ESP_ERROR_CHECK(ble_hs_adv_set_fields_mbuf(&adv_fields, data));
	ESP_ERROR_CHECK(ble_gap_ext_adv_set_data(0, data));
	ESP_ERROR_CHECK(ble_gap_ext_adv_start(0,(TIMER_ADV / 10), 0));
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
#endif
	ESP_LOGI(TAG, "advertising started! tx_pwr %d", tx_pwr);
}

void ble_scan_init() {
	__unused uint8_t own_addr_type = BLE_ADDR_PUBLIC;
	__unused ble_gap_disc_params disc_params = { .itvl = 0, .window = 0, .filter_policy = 0, .limited = 0, .passive = 1, .filter_duplicates = 1, .disable_observer_mode = 0};
	__unused ble_gap_ext_disc_params ext_params = { .itvl = 0, .window = 0, .passive = SCAN_PASSIVE, .disable_observer_mode = 0 };
	/* Figure out address to use while advertising (no privacy for now) */
	//ESP_RETURN_VOID_ON_ERROR(ble_hs_id_infer_auto(0, &own_addr_type), TAG, "determining address type");
	//ESP_RETURN_VOID_ON_ERROR(ble_gap_disc(BLE_ADDR_PUBLIC,0,&disc_params, gap_event_handler, NULL), TAG, "ble_gap_ext_disc");
	ESP_ERROR_CHECK(ble_gap_ext_disc(own_addr_type,0, 0, true, 0, 0,&ext_params,&ext_params, gap_event_handler, NULL));
}

int gap_init(void) {
	ble_svc_gap_init(); /* Call NimBLE TAG initialization API */
	//char* name = const_cast<char*>(ble_svc_gap_device_name());
	//memcpy(name + sizeof(MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME))-1,,3*2);
	set_ble_device_name();
	//ble_svc_gap_device_name_set(DEVICE_NAME); /* Set TAG device name */
	ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE);
	return ESP_OK;
}

static int ble_store_config_read_hook(int obj_type, const ble_store_key *key, ble_store_value *value) {
	ESP_LOGI(TAG, "read obj_type %u", obj_type);
	return ble_store_config_read(obj_type, key, value);
	return BLE_HS_ENOENT;
}

static int ble_store_config_write_hook(int obj_type, const union ble_store_value *val) {
	ESP_LOGI(TAG, "write obj_type %u", obj_type);
	//return ble_store_config_write(obj_type, val);
	return 0;
}

void ble_hs_cfg_init() {
	ble_hs_cfg.reset_cb = [](int reason) { ESP_LOGI("NimBLE", "nimble stack reset, reason: %d", reason);}; //on_stack_reset is called when host resets BLE stack due to errors
	ble_hs_cfg.sync_cb = host_sync_cb;
	ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
	ble_hs_cfg.store_read_cb = ble_store_config_read_hook;
	ble_hs_cfg.store_write_cb = ble_store_config_write_hook;
	ble_hs_cfg.store_delete_cb = ble_store_config_delete;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
	ble_hs_cfg.sm_bonding = 1;
	ble_hs_cfg.sm_mitm = 1;
	ble_hs_cfg.sm_sc = 1;
	ble_hs_cfg.sm_sc_only = 1;
	ble_hs_cfg.sm_sec_lvl = 4;
	ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_store_config_conf_init(); //ble_store_config_init();
}

int save_bonding(uint16_t conn_handle) { //No such entry
	//our sec 1; peer sec 2; cccd 3; peer addr(rpa_rec) 6; loc irk 7; csfc 8
	//CCCD Client Characteristic Configuration Descriptor
	//IRK - Identity Resolving Key //RPA rec - Resolvable Private Addresses record
	struct ble_store_key_sec key; key.idx = 0;
	union ble_store_value value;
	int ret = ble_gap_conn_find(conn_handle, &desc);
	if(ret) return ret; //log internal
	if (desc.peer_id_addr.type & BLE_ADDR_RANDOM) {
		ESP_LOGW(TAG,"peer_id_addr.type %u" ,desc.peer_id_addr.type);
		return BLE_HS_ENOADDR;
	}
	static_assert(CONFIG_BT_NIMBLE_MAX_CCCDS == 0);

	auto lambda = [](int obj_type, const ble_store_key_sec *key, ble_store_value *val) {
		if(!ble_store_read(obj_type, (ble_store_key*)key, val)) {
			ble_store_write(BLE_STORE_OBJ_TYPE_PEER_SEC, val); //log internal
		}
		else { ESP_LOGI(TAG,"No such entry %u", obj_type); };
	};

	ble_hs_cfg.store_write_cb = ble_store_config_write;
		key.peer_addr = desc.our_id_addr;
		lambda(BLE_STORE_OBJ_TYPE_OUR_SEC, &key, &value);

		key.peer_addr = desc.peer_id_addr;
		lambda(BLE_STORE_OBJ_TYPE_PEER_SEC, &key, &value);
		lambda(BLE_STORE_OBJ_TYPE_PEER_ADDR, &key, &value);
	ble_hs_cfg.store_write_cb = ble_store_config_write_hook;

	ESP_LOGI(TAG, "%s %d", __FUNCTION__, ret);
	return ret;
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
	/* Copy device address to addr_val */
	ESP_RETURN_VOID_ON_ERROR(ble_hs_id_copy_addr(own_addr_type, addr_val, NULL), TAG, "copy device address");
	ESP_RETURN_VOID_ON_ERROR(ble_svc_gap_device_name_set(DEVICE_NAME), TAG, "set device name to %s", DEVICE_NAME);
	ESP_LOGI(TAG, "device address: %s", format_addr(addr_val));
#endif
}

bool is_connection_encrypted(uint16_t conn_handle) {
	//struct ble_gap_conn_desc desc;
	/* Print connection descriptor */
	int ret = ble_gap_conn_find(conn_handle, &desc);
	if(ret) { ESP_LOGW(TAG, "!find connection by handle: %u",conn_handle, ret); return false; }
	return desc.sec_state.encrypted;
}

void print_conn_desc(ble_gap_conn_desc *desc) {
	ESP_LOGI(TAG, "connection handle: %u", desc->conn_handle);
	ESP_LOGI(TAG, "local address: type (%u): %s", desc->our_id_addr.type, format_addr(desc->our_id_addr.val));
	ESP_LOGI(TAG, "peer address: type (%u): %s", desc->peer_id_addr.type, format_addr(desc->peer_id_addr.val));
	ESP_LOGI(TAG, "itvl %d, latency %d, timeout %u, ""encr %u, auth %u, bonded %u, key_size %u",
			 desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
			 desc->sec_state.encrypted, desc->sec_state.authenticated,desc->sec_state.bonded,
			 desc->sec_state.key_size);
}

void parse_adv_data(const uint8_t* data, uint8_t data_len) {
#if _ESP_LOG_ENABLED(3)
	//NIMLOG("Raw Data: "); for (uint8_t i = 0; i < data_len; i++) { NIMLOG("%02X ", data[i]); }
	NIMLOG("Data length: \t%u\n", data_len);
	for (size_t i = 0; i < data_len;) {
		uint8_t len = data[i]; if(len > 2) { NIMLOG("Len: %u\t", len);}
		uint8_t type = data[++i]; NIMLOG("Type: "); NIMLOG("%02X", type);
		if(type == COMPLETE_NAME || type == SHORT_NAME ) {
			NIMLOG(" Name: "); //for(size_t end = i + len;++i < end; i++) { NIMLOG("%c",data[i]); }
			char* name = (char*)(data+i); i += len;
			name[i] = '\0';
			NIMLOG(name);
		}
		else { NIMLOG(" { "); for(size_t end = i + len;++i < end;) { NIMLOG("%02X ", data[i]); } NIMLOG("}"); }
		NIMLOG(len > 2 ? "\n" : " ");
	}NIMLOG("\n");
#endif
}

void print_rx_data(const struct os_mbuf *buf) { //notify_rx.om->om_len = %u
	__unused auto len = buf->om_len; __unused auto &om_data = buf->om_data;
	if(!len) { return; }
	NIMLOG("Data len = %u, Data: ", buf->om_len);
	NIMLOG(" { "); for(size_t i = 0; i < len; i++) {
		NIMLOG("%02X ", om_data[i]); }
		NIMLOG("}\n"); //NIMLOG(len > 2 ? "\n" : " ");
}

void print_props_mask(uint8_t props) {
	__unused static const char* type[] = { "CONN", "SCAN" , "DIR", "RSP" , "LEGA" };
	for (uint8_t mask = 16; mask; mask>>=1) { NIMLOG("%c",props & mask ? '1': '0'); } NIMLOG("\t");
	for (size_t i = 0;i < 5; i++) { if(props & (1 << i)) { NIMLOG(type[i]); NIMLOG(", "); }  };
}

void print_event_report(const ble_gap_ext_disc_desc & disc) {
	NIMLOG("\n%s (%u)",format_addr(disc.addr.val), disc.addr.type);
	NIMLOG("\nAD Event Mask: "); print_props_mask(disc.props);NIMLOG("\nRSSI:\t\t%d\n", disc.rssi);
	if (disc.props & BLE_HCI_ADV_LEGACY_MASK) { NIMLOG("Legacy event: \t%u\n", disc.legacy_event_type); } //BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND; //3
	else { if(disc.tx_power != 127) { NIMLOG("Tx Power: \t%d\n", disc.tx_power); }
		NIMLOG("Prim PHY: \t%u\nSecn PHY: \t%u\nSID:\t\t%d\n", disc.prim_phy, disc.sec_phy, disc.sid); }
	if(disc.props & BLE_HCI_ADV_DIRECT_MASK) { NIMLOG("Direct address: \t%s", format_addr(disc.direct_addr.val));}
	if (disc.length_data) { parse_adv_data(disc.data, disc.length_data); } NIMLOG("\n");
}

void print_event_report(const ble_gap_disc_desc & disc) {
	 //BLE_HCI_ADV_RPT_EVTYPE_ADV_IND;//0
	NIMLOG("\n%s (%u)\nAD Event Type:\t%u\nRSSI:\t\t%d\n", format_addr(disc.addr.val), disc.addr.type, disc.event_type, disc.rssi);
	if(disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) { NIMLOG("Direct address: \t%s", format_addr(disc.direct_addr.val));}
	if (disc.length_data) { parse_adv_data(disc.data, disc.length_data); } NIMLOG("\n");
}

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
}
