//#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "common.h"
#include "gatt.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

static const char* TAG = "GATT";
/* HEART_RATE Service */
__unused static const ble_uuid16_t SVC_HEART = BLE_UUID16_INIT(0x180D);
__unused static const ble_uuid16_t CHR_HEART = BLE_UUID16_INIT(0x2A37);
/* Automation IO service */
static const ble_uuid16_t SVC_AUTO_IO = BLE_UUID16_INIT(0x1815);
static const ble_uuid128_t CHR_AUTO_IO = BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef,0x12, 0x12, 0x25, 0x15, 0x00, 0x00);
/* Serial Port Profile Service */
__unused static const ble_uuid16_t SVC_SPP = BLE_UUID16_INIT(0xABF0);
__unused static const ble_uuid16_t CHR_SPP = BLE_UUID16_INIT(0xABF1);

//#include "heart_rate.h"
extern void gatt_cts_service_init();
extern char* get_serial_buf();  extern uint8_t get_serial_len();
/* Private function declarations */
static int io_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
static int serial_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
__unused static int heart_rate_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
/* Attribute value handles */
static uint16_t h_io_chr;
static uint16_t h_spp_chr;
__unused static uint16_t h_heart_chr;

typedef union { 
	uint8_t encrypted: 1, io: 1, spp: 1, heart: 1; uint8_t val;
} sub_attr_t; static_assert(sizeof(sub_attr_t) == 1);

static sub_attr_t subs_conn[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

/* TAG services table */
const struct ble_gatt_svc_def gatt_svr_svcs [] = {
	{	// Heart rate service
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &SVC_HEART.u,
		.characteristics = (struct ble_gatt_chr_def []) {
		{	// Heart Rate characteristic 
			.uuid = &CHR_HEART.u,
			.access_cb = heart_rate_chr_access,
			.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_NOTIFY ,//| BLE_GATT_CHR_F_READ_ENC,
			.val_handle = &h_heart_chr
		}, {  }
		}
	},	
	{	// Automation IO service
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &SVC_AUTO_IO.u,
		.characteristics = (struct ble_gatt_chr_def []) {
		{	// LED characteristic
			.uuid = &CHR_AUTO_IO.u,
			.access_cb = io_chr_access,
			.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
			.val_handle = &h_io_chr,
		}, {  }
		},
	},
	{	// Serial Profile Service
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &SVC_SPP.u,
		.characteristics = (struct ble_gatt_chr_def []) {
		{	// SPP characteristic
			.uuid = &CHR_SPP.u,
			.access_cb = serial_chr_access,
			.val_handle = &h_spp_chr,
			.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
		}, {  }
		},
	},
	{ /* No more services. */},
};

/* Private functions */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	static const char *TAG = "HEART";
	if (attr_handle != h_heart_chr) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	/* Note: Heart rate characteristic is read only */
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR: {
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "read", conn_handle, attr_handle);
		}	CHECK_RET(os_mbuf_append(ctxt->om, &(uint8_t [2]) {0, get_heart_rate()}, 2));
			return 0;
	} break;
	default: ESP_LOGW(TAG, "opcode: %u", ctxt->op);
	}
	return BLE_ATT_ERR_UNLIKELY;
}

static int io_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	static const char *TAG = "IO";
	if (attr_handle != h_io_chr) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_WRITE_CHR: /* WRITE characteristic event */
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "write",conn_handle, attr_handle);
		}  
		if (ctxt->om->om_len == 1) { 
			if (ctxt->om->om_data[0]) { impl_io_on(); } 
			else { impl_io_off();}
			ESP_LOGI(TAG, "%u", ctxt->om->om_data[0]);
			return 0;
		} else { ESP_LOGW(TAG, "om_len %u",ctxt->om->om_len);}
		break;
	case BLE_GATT_ACCESS_OP_READ_CHR: { /* READ characteristic event */
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "read", conn_handle, attr_handle);
		}
		CHECK_RET(os_mbuf_append(ctxt->om, &(uint8_t [1]) { impl_io_get() }, 1));
		return 0; 
		}
	break;
	default: ESP_LOGW(TAG, "opcode: %u", ctxt->op);
	}
	return BLE_ATT_ERR_UNLIKELY;
}

static int serial_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	static const char *TAG = "SPP";
	static char spp_buf[64] = {};
	if (attr_handle != h_spp_chr) { ESP_LOGW(TAG, "attr_handle %u", attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_WRITE_CHR: { /* WRITE characteristic event */
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "write", conn_handle, attr_handle);
		}
			uint16_t out_len; //ESP_LOGI(TAG, "om_len %u", ctxt->om->om_len); 
			CHECK_RET(ble_hs_mbuf_to_flat(ctxt->om, spp_buf, sizeof(spp_buf), &out_len));
			spp_buf[out_len] = '\0'; ESP_LOGI(TAG, "%s", spp_buf);
		}
		return 0;
	case BLE_GATT_ACCESS_OP_READ_CHR: { /* READ characteristic event */
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "read", conn_handle, attr_handle);
		}
		CHECK_RET(os_mbuf_append(ctxt->om, get_serial_buf(), get_serial_len()));
		} return 0; 
	default: ESP_LOGW(TAG, "opcode: %u", ctxt->op);
	}
	return BLE_ATT_ERR_UNLIKELY;
}
/* Public functions */

void gatt_svr_init(void) {
	ble_svc_gatt_init();
	CHECK_VOID(ble_gatts_count_cfg(gatt_svr_svcs)); 
	CHECK_VOID(ble_gatts_add_svcs(gatt_svr_svcs));
	gatt_cts_service_init(); //ble_svc_cts_time_updated();
}

int need_notify_io() { return subs_conn[0].io; }

int clear_connection(uint16_t h_conn) {
	if (h_conn >= sizeof(subs_conn)) { CHECK_RET(BLE_ATT_ERR_INVALID_HANDLE); }
	subs_conn[h_conn].val = 0; return 0;
}

int set_encryption(uint16_t h_conn) {
	if (h_conn >= sizeof(subs_conn)) { CHECK_RET(BLE_ATT_ERR_INVALID_HANDLE); }
	subs_conn[h_conn].encrypted = 1; return 0;
}

int get_encryption(uint16_t h_conn) {
	if (h_conn >= sizeof(subs_conn)) { ESP_LOGW(TAG, "!get_encryption"); return 0; }
	return subs_conn[h_conn].encrypted;
}

void send_alarm_notify() {
	for (size_t i = 1; i < sizeof(subs_conn); i++) {
		if (subs_conn[i].io) {
			CHECK_VOID(ble_gatts_notify(i, h_io_chr)); //ESP_LOGI(TAG, "send_alarm_notify %u", i);
		}
	}
}

void send_heart_rate_notify() {
	for (size_t i = 1; i < sizeof(subs_conn); i++) {
		if (subs_conn[i].heart) {
			CHECK_VOID(ble_gatts_notify(i, h_heart_chr)); //ESP_LOGI(TAG, "send_heart_rate_notify %u", i);
		}
	}
}

void send_spp_notify() {
	for (size_t i = 1; i < sizeof(subs_conn); i++) {
		if (subs_conn[i].spp) {
			CHECK_VOID(ble_gatts_notify(i, h_spp_chr)); //ESP_LOGI(TAG, "send_spp_notify %u", i);
		}
	}
}

static int set_sub_chr(size_t h_conn, int num, bool val) {
	if (unlikely(h_conn >= sizeof(subs_conn))) return BLE_ATT_ERR_INVALID_HANDLE;
	switch (num) {
			case 1: subs_conn[h_conn].io = val; ESP_LOGI(TAG, "chr io -> %u", val); 
				return 0;
			case 2: subs_conn[h_conn].spp = val; ESP_LOGI(TAG, "chr spp -> %u", val); 
				return 0;
			case 3: subs_conn[h_conn].heart = val; ESP_LOGI(TAG, "chr heart -> %u", val); 
				return 0;
		default: ESP_LOGW(TAG, "set_sub_chr %d", num); 
	}
	return BLE_ATT_ERR_UNLIKELY;
}

int gatt_svr_subscribe_cb(const struct ble_gap_event *event) {
	const size_t attr_handle = event->subscribe.attr_handle;
	int chr_num;
	if(attr_handle == h_io_chr) chr_num = 1;
	else if(attr_handle == h_spp_chr) chr_num = 2;
	else if (attr_handle == h_heart_chr) chr_num = 3;
	else return BLE_ATT_ERR_ATTR_NOT_FOUND;
	size_t h_conn = event->subscribe.conn_handle;
	//if (!is_connection_encrypted(h_conn)) {
	if (!get_encryption(h_conn)) {
		ESP_LOGW(TAG, "connection not encrypted!");
		return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
	}
	return set_sub_chr(h_conn, chr_num, event->subscribe.cur_notify || event->subscribe.cur_indicate); return 0;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
	__unused char buf[BLE_UUID_STR_LEN];
	/* Handle TAG attributes register events */
	switch (ctxt->op) {
	/* Service register event */
	case BLE_GATT_REGISTER_OP_SVC:
		ESP_LOGI(TAG, "registered service %s with handle=%d",
			ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),ctxt->svc.handle);
		break;
	/* Characteristic register event */
	case BLE_GATT_REGISTER_OP_CHR:
		ESP_LOGI(TAG,"registering characteristic %s with ""def_handle=%d val_handle=%d",
			ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),ctxt->chr.def_handle, ctxt->chr.val_handle);
		break;
	/* Descriptor register event */
	case BLE_GATT_REGISTER_OP_DSC:
		ESP_LOGI(TAG, "registering descriptor %s with handle=%d",
			ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
		break;
	default: ESP_LOGW(TAG, "Unknown event");
	}
}

