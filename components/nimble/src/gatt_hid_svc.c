//#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
//#pragma GCC diagnostic ignored "-Wunused-function"
#include "common.h"
//#include "gatt.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

#define BLE_UUID_HID_INFO_CHR				0x2A4A
#define BLE_UUID_HID_REPORT_MAP_CHR			0x2A4B
#define BLE_UUID_HID_CONTROL_POINT_CHR 		0x2A4C
#define BLE_UUID_HID_REPORT_CHR				0x2A4D
#define BLE_UUID_HID_PROTOCOL_MODE_CHR		0x2A4E

static const char* TAG = "HID";

__unused static const ble_uuid16_t SVC_DEVICE_INFO = BLE_UUID16_INIT(0x180A);
__unused static const ble_uuid16_t CHR_MANUFACTURE_NAME = BLE_UUID16_INIT(0x2A29);

__unused static const ble_uuid16_t SVC_HID = BLE_UUID16_INIT(0x1812);
__unused static const ble_uuid16_t CHR_HID_INFO = BLE_UUID16_INIT(BLE_UUID_HID_INFO_CHR);
__unused static const ble_uuid16_t CHR_HID_REPORT_MAP = BLE_UUID16_INIT(BLE_UUID_HID_REPORT_MAP_CHR);
__unused static const ble_uuid16_t CHR_HID_CONTROL_POINT = BLE_UUID16_INIT(BLE_UUID_HID_CONTROL_POINT_CHR);
__unused static const ble_uuid16_t CHR_HID_REPORT = BLE_UUID16_INIT(BLE_UUID_HID_REPORT_CHR);
__unused static const ble_uuid16_t CHR_HID_PROTOCOL_MODE = BLE_UUID16_INIT(BLE_UUID_HID_PROTOCOL_MODE_CHR);
static const ble_uuid16_t DSC_HID_REPORT_REF = BLE_UUID16_INIT(0x2908);
// 1. HID Report Map (DESCRIPTOR)
static const uint8_t hid_report_map[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xA1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xA1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xC0,                          //   END_COLLECTION
    0xC0                           // END_COLLECTION
};

static const uint8_t hid_info[] = {
    0x11, 0x01, // bcdHID (v1.11)
    0x00,       // bCountryCode
    0x1 | 0x2   // Flags (RemoteWake = false, NormallyConnectable = true)
};

static const uint8_t hid_report_ref[] = { 0x00, 0x01 };

//Report Mode (0x01) Boot mode (0x0)
static uint8_t hid_proto_mode = 0x01;
const char manufacture[] = "natribu.org";

static int hid_manufacture_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "read",conn_handle, attr_handle);
		return os_mbuf_append(ctxt->om, manufacture, sizeof(manufacture)-1);
    }
	ESP_LOGW(TAG, "opcode: %u", ctxt->op); return BLE_ATT_ERR_UNLIKELY;
}

static int hid_report_map_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "report_map",conn_handle, attr_handle);
		return os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
	}
	ESP_LOGW(TAG, "opcode: %u", ctxt->op); return BLE_ATT_ERR_UNLIKELY;
}

static int hid_info_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "info",conn_handle, attr_handle);
		return os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
	}
	ESP_LOGW(TAG, "opcode: %u", ctxt->op); return BLE_ATT_ERR_UNLIKELY;
}
static int hid_protocol_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "protocol READ",conn_handle, attr_handle);
		return os_mbuf_append(ctxt->om, &hid_proto_mode, sizeof(hid_proto_mode));
	}
	else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) { ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "protocol WRITE", conn_handle, attr_handle); }
	ESP_LOGW(TAG, "opcode: %u", ctxt->op); return BLE_ATT_ERR_UNLIKELY;
}
static int hid_report_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		ESP_LOGI(TAG, "chr %s conn_handle %u attr_handle %u", "report",conn_handle, attr_handle);
        return os_mbuf_append(ctxt->om, &(uint8_t[3]) {}, 3);
	}
	ESP_LOGW(TAG, "opcode: %u", ctxt->op); return BLE_ATT_ERR_UNLIKELY;
}

static int hid_report_ref_dsc_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        ESP_LOGI(TAG, "dsc read report reference");
        return os_mbuf_append(ctxt->om, hid_report_ref, sizeof(hid_report_ref));
    }
    return BLE_ATT_ERR_UNLIKELY;
}

const struct ble_gatt_svc_def gatt_hid_svcs [] = {
	{				//DEVICE INFO
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_DEVICE_INFO.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &CHR_MANUFACTURE_NAME.u, // Manufacturer Name
            .access_cb = hid_manufacture_chr_access,
            .flags = BLE_GATT_CHR_F_READ,
        	}, { }
		}
    },
    {				//HID
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_HID.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &CHR_HID_REPORT_MAP.u,
            .access_cb = hid_report_map_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_AUTHEN, 
        }, {
            .uuid = &CHR_HID_INFO.u,
            .access_cb = hid_info_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_AUTHEN,
        }, {
            .uuid = &CHR_HID_PROTOCOL_MODE.u,
            .access_cb = hid_protocol_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_AUTHEN | 
			BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_AUTHEN,
        }, {
            .uuid = &CHR_HID_REPORT.u,
            .access_cb = hid_report_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_AUTHEN 
			| BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_AUTHEN, 
			.descriptors = (struct ble_gatt_dsc_def[]) { {
                .uuid = &DSC_HID_REPORT_REF.u,
                .att_flags = BLE_ATT_F_READ,
                .access_cb = hid_report_ref_dsc_access,
            	}, { }
			}
		}, { /* No more characteristics */ } 
		}
    },
	{ /* No more services */},

};

void gatt_hid_service_init() {
	CHECK_VOID(ble_gatts_count_cfg(gatt_hid_svcs));
    CHECK_VOID(ble_gatts_add_svcs(gatt_hid_svcs));
}