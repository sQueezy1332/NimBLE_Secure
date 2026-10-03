//#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "common.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include "gatt.h"
#pragma GCC diagnostic ignored "-Wunused-function"

#define AUTO_IO_CHR 1
//#define SPP_CHR 2
//#define HEART_RATE_CHR 3

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

//__weak_symbol void io_on_cb() {}
//__weak_symbol void io_off_cb() {}
//__weak_symbol int io_get_cb() { return 0; }
//__weak_symbol uint8_t get_heart_rate() { return 0; }

//#include "heart_rate.h"
extern void gatt_cts_service_init();
extern void gatt_hid_service_init();
/* Private function declarations */
static int io_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
static int serial_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
static int heart_rate_chr_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *);
/* Attribute value handles */
static uint16_t h_io_chr;
static uint16_t h_spp_chr;
__unused static uint16_t h_heart_chr;

static handle_mask_t subs_io, __unused subs_spp, __unused subs_heart;
static handle_mask_t conn_encrypted;

const uint16_t MAX_HANDLE = sizeof(handle_mask_t) * 8; //2 + MYNEWT_VAL_BLE_MAX_CONNECTIONS;
static_assert(1 + MYNEWT_VAL_BLE_MAX_CONNECTIONS < sizeof(handle_mask_t) * 8);

/* TAG services table */
static const struct ble_gatt_svc_def gatt_svr_svcs [] = {
#ifdef AUTO_IO_CHR
	{	// Automation IO service
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &SVC_AUTO_IO.u,
		.characteristics = (struct ble_gatt_chr_def []) {
		{	// LED characteristic
			.uuid = &CHR_AUTO_IO.u,
			.access_cb = io_chr_access,
			.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_AUTHEN
			| BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN
			| BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_AUTHEN,
			.val_handle = &h_io_chr,
		}, {  }
		},
	},
#endif
#ifdef SPP_CHR
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
#endif
#ifdef HEART_RATE_CHR
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
#endif	
	{ /* No more services. */},
};
/* Private functions */

#ifdef HEART_RATE_CHR
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	const char *TAG = "HEART";
	//if (attr_handle != h_heart_chr) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
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
#endif
#ifdef AUTO_IO_CHR
static int io_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	const char *TAG = "IO";
	//if (attr_handle != h_io_chr) { ESP_LOGW(TAG, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_WRITE_CHR: /* WRITE characteristic event */
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "write",conn_handle, attr_handle);
		}  
		if (ctxt->om->om_len == 1) { 
			if (ctxt->om->om_data[0]) { io_on_cb(); } 
			else { io_off_cb();}
			ESP_LOGI(TAG, "write %u", ctxt->om->om_data[0]);
			return 0;
		} else { ESP_LOGW(TAG, "om_len %u",ctxt->om->om_len);}
		break;
	case BLE_GATT_ACCESS_OP_READ_CHR: { /* READ characteristic event */
			if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
				ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "read", conn_handle, attr_handle);
			} else { ESP_LOGI(TAG, "read %u", io_get_cb());} 
			CHECK_RET(os_mbuf_append(ctxt->om, &(uint8_t [1]) { io_get_cb() }, 1));
			return 0; 
		}
	break;
	default: ESP_LOGW(TAG, "opcode: %u", ctxt->op);
	}
	return BLE_ATT_ERR_UNLIKELY;
}
#endif
#ifdef SPP_CHR
/*
static int serial_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	extern char* get_serial_buf();  extern uint8_t get_serial_len();
	const char *TAG = "SPP";
	static char spp_buf[64] = {};
	//if (attr_handle != h_spp_chr) { ESP_LOGW(TAG, "attr_handle %u", attr_handle); return BLE_ATT_ERR_UNLIKELY; }
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_WRITE_CHR: {  WRITE characteristic event
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "write", conn_handle, attr_handle);
		}
			uint16_t out_len; //ESP_LOGI(TAG, "om_len %u", ctxt->om->om_len); 
			CHECK_RET(ble_hs_mbuf_to_flat(ctxt->om, spp_buf, sizeof(spp_buf)-1, &out_len));
			spp_buf[out_len] = '\0'; ESP_LOGI(TAG, "%s", spp_buf);
		}
		return 0;
	case BLE_GATT_ACCESS_OP_READ_CHR: {  READ characteristic event
		if(conn_handle != BLE_HS_CONN_HANDLE_NONE) {
			ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "read", conn_handle, attr_handle);
		}
		CHECK_RET(os_mbuf_append(ctxt->om, get_serial_buf(), get_serial_len()));
		} return 0; 
	default: ESP_LOGW(TAG, "opcode: %u", ctxt->op);
	}
	return BLE_ATT_ERR_UNLIKELY;
}*/
#endif
/* Public functions */

int clear_connection(uint16_t h_conn) {
	if (h_conn > MAX_HANDLE || !h_conn) { CHECK_RET(BLE_ATT_ERR_INVALID_HANDLE); }
#ifdef AUTO_IO_CHR
	handle_write(subs_io, h_conn, 0);
#endif
#ifdef SPP_CHR
	handle_write(subs_spp, h_conn, 0);
#endif
#ifdef HEART_RATE_CHR
	handle_write(subs_heart, h_conn, 0);
#endif
	handle_write(conn_encrypted, h_conn, 0);
	return 0;
}

int set_encryption(uint16_t h_conn) {
	if (h_conn > MAX_HANDLE || !h_conn) { CHECK_RET(BLE_ATT_ERR_INVALID_HANDLE); }
	handle_write(conn_encrypted, h_conn, 1); return 0;
}

int is_encrypted(uint16_t h_conn) {
	if (h_conn > MAX_HANDLE || !h_conn) { CHECK_RET(BLE_ATT_ERR_INVALID_HANDLE); }
	return handle_read(conn_encrypted, h_conn);
}

handle_mask_t get_conn_encrypted() { return conn_encrypted; }

handle_mask_t need_notify_io() { return subs_io; }

void send_io_notify() {
	for (uint16_t h = MIN_HANDLE_VAL; h < MAX_HANDLE; h++)
		if (handle_read(subs_io, h)) {
			CHECK_VOID(ble_gatts_notify(h, h_io_chr));
		}
}

void send_spp_notify() {
	if(!subs_spp) return;
	for (uint16_t h = MIN_HANDLE_VAL; h < MAX_HANDLE; h++)
		if (handle_read(subs_spp, h))
			CHECK_VOID(ble_gatts_notify(h, h_spp_chr)); //ESP_LOGI(TAG, "send_spp_notify %u", i);
}

void send_heart_rate_notify() {
	if(!subs_heart) return;
	for (uint16_t h = MIN_HANDLE_VAL; h < MAX_HANDLE; h++)
		if (handle_read(subs_heart, h))
			CHECK_VOID(ble_gatts_notify(h, h_heart_chr)); //ESP_LOGI(TAG, "send_heart_rate_notify %u", i);
}

int gatt_svr_subscribe_cb(const struct ble_gap_event *event) {
	const size_t attr_handle = event->subscribe.attr_handle;
	const size_t h_conn = event->subscribe.conn_handle;
	if (unlikely(h_conn > MAX_HANDLE)) return BLE_ATT_ERR_INVALID_HANDLE;
	if (!is_encrypted(h_conn)) {
		ESP_LOGW(TAG, "connection not encrypted!");
		return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
	} ESP_LOGD(TAG, "conn_encrypted 0x%04x", conn_encrypted);
	uint8_t notify = event->subscribe.cur_notify | event->subscribe.cur_indicate;
#ifdef AUTO_IO_CHR
	if(attr_handle == h_io_chr) { handle_write(subs_io, h_conn, notify); return 0; }
#endif
#ifdef SPP_CHR
	else if(attr_handle == h_spp_chr) { handle_write(subs_spp, h_conn, notify); return 0; }
#endif
#ifdef HEART_RATE_CHR
	else if (attr_handle == h_heart_chr) { handle_write(subs_heart, h_conn, notify); return 0; }
#endif
	return BLE_ATT_ERR_ATTR_NOT_FOUND;
}

void gatt_init(void) {
	ble_svc_gatt_init();
	CHECK_VOID(ble_gatts_count_cfg(gatt_svr_svcs)); 
	CHECK_VOID(ble_gatts_add_svcs(gatt_svr_svcs));
	gatt_cts_service_init();
	gatt_hid_service_init();
	
}

void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
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