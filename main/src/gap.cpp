/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define CHECK_ERROR(x) ESP_ERROR_CHECK_WITHOUT_ABORT(x)
#define CHECK_RETURN(x) ESP_RETURN_ON_ERROR(x, CTS, "")
#include "gatt_svc.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include "common.h"
#include "gatt_cts_svc.h"
#include "heart_rate.h"
//#include "led.h"
#define GATT "Gatt"
/* Private function declarations */
__unused static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */
/* Heart rate service */
static const ble_uuid16_t heart_rate_svc_uuid = BLE_UUID16_INIT(0x180D);
static const ble_uuid16_t heart_rate_chr_uuid = BLE_UUID16_INIT(0x2A37);

static struct gatt_svc_s {
    uint16_t chr_handle;
    uint16_t chr_conn_handle;
    struct {
        uint8_t encrypted , notify  , indicate  ;
        uint8_t send;
    } ch;
} heart_rate;

static_assert(sizeof(heart_rate) == 8);
//extern TaskHandle_t main_handle;
void clear_characteristic() { heart_rate.ch = {}; }
void set_encryption() {  heart_rate.ch.encrypted = 1; }
bool need_notify() { return heart_rate.ch.send; }
void send_alarm_notify() { ESP_ERROR_CHECK_WITHOUT_ABORT(ble_gatts_notify(heart_rate.chr_conn_handle, heart_rate.chr_handle));}

/* Automation IO service */
static const ble_uuid16_t auto_io_svc_uuid = BLE_UUID16_INIT(0x1815);
static const ble_uuid128_t led_chr_uuid = BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef,0x12, 0x12, 0x25, 0x15, 0x00, 0x00);
static const ble_uuid128_t led_chr_time_uuid = BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef,0x12, 0x12, 0x25, 0x15, 0x01, 0x00); 
//__unused static uint16_t led_chr_handle;
/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Heart rate service */
/*     {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &heart_rate_svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]) {
        {.uuid = &heart_rate_chr_uuid.u,
          .access_cb = heart_rate_chr_access,
          .flags =  BLE_GATT_CHR_F_READ |  BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
          .val_handle = &heart_rate.chr_handle
        },{0}
        }
    }, */
    /* Automation IO service */
    {   .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &auto_io_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {/* LED characteristic */
            {   .uuid = &led_chr_uuid.u,
                .access_cb = led_chr_access, 
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC  | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &heart_rate.chr_handle,
            },
            /* {  .uuid = &led_chr_time_uuid.u,
                .access_cb = led_chr_access, 
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC  | BLE_GATT_CHR_F_READ_ENC,
                .val_handle = &heart_rate.chr_handle,
            }, //time to depatch */
            {/* No more characteristics in this service. */}
        },
    },
    {0, /* No more services. */},
};

/* Private functions */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Note: Heart rate characteristic is read only */
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        ESP_LOGI(GATT, "characteristic %s %s conn_handle %u attr_handle %u", "read",
            conn_handle != BLE_HS_CONN_HANDLE_NONE ? "" : "by nimble stack" ,conn_handle, attr_handle);
        /* Verify attribute handle */
        if (attr_handle == heart_rate.chr_handle) {
            //heart_rate_chr_val[1] = // Update access buffer value
            uint8_t chr_val[] = { 0, get_heart_rate() };
            ESP_RETURN_ON_ERROR(os_mbuf_append(ctxt->om, &chr_val, sizeof(chr_val)), GATT, "");
            return 0;
        } else { ESP_LOGW(GATT, "attr_handle %u",attr_handle); }
        //goto error;
    } break;
    default: ESP_LOGW(GATT,"%s characteristic, opcode: %u", "heart_rate", ctxt->op);
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle != heart_rate.chr_handle) { 
        ESP_LOGW(GATT, "attr_handle %u",attr_handle); return BLE_ATT_ERR_UNLIKELY; }
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR: /* WRITE characteristic event */
        ESP_LOGI(GATT, "characteristic %s %s conn_handle %u attr_handle %u", "write",
        conn_handle != BLE_HS_CONN_HANDLE_NONE ? "" : "by nimble stack" ,conn_handle, attr_handle);      
        if (ctxt->om->om_len == 1) { 
            if (ctxt->om->om_data[0]) { impl_io_on(); } 
            else { impl_io_off();}
            ESP_LOGI(GATT, "%u", ctxt->om->om_data[0]);
            return 0;
        } else { ESP_LOGW(GATT, "om_len %u",ctxt->om->om_len);}
        break;
    case BLE_GATT_ACCESS_OP_READ_CHR: { /* READ characteristic event */
        ESP_LOGI(GATT, "characteristic %s %s conn_handle %u attr_handle %u", "read", 
        conn_handle != BLE_HS_CONN_HANDLE_NONE ? "" : "by nimble stack" ,conn_handle, attr_handle);
        uint8_t chr_val[] = { impl_io_get() };
        ESP_RETURN_ON_ERROR(os_mbuf_append(ctxt->om, &chr_val, sizeof(chr_val)), GATT,"");
        return 0;
        }
    break;
    default: ESP_LOGW(GATT,"%s characteristic, opcode: %u", "led", ctxt->op);
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* Public functions */
int gatt_svc_init(void) {
    ble_svc_gatt_init(); /* 1. GATT service initialization */
    ESP_RETURN_ON_ERROR(ble_gatts_count_cfg(gatt_svr_svcs),GATT,""); /* 2. Update GATT services counter */
    ESP_RETURN_ON_ERROR(ble_gatts_add_svcs(gatt_svr_svcs),GATT,"");/* 3. Add GATT services */
    gatt_cts_service_init(); //ble_svc_cts_time_updated()
    return 0;
}

void send_heart_rate_notify(void) {
    //if (!heart_rate.chr_conn_handle) { return; } //heart_rate_chr_conn_handle_inited
    if (heart_rate.ch.send) {
        if(heart_rate.ch.notify) 
            ESP_ERROR_CHECK_WITHOUT_ABORT(ble_gatts_notify(heart_rate.chr_conn_handle, heart_rate.chr_handle));
        //else if(heart_rate.ch.indicate)
            //ESP_ERROR_CHECK_WITHOUT_ABORT(ble_gatts_indicate(heart_rate.chr_conn_handle, heart_rate.chr_handle));
    }
}

int gatt_svr_subscribe_cb(const ble_gap_event *event) {
    if (event->subscribe.attr_handle == heart_rate.chr_handle) {
        heart_rate.chr_conn_handle = event->subscribe.conn_handle;
        heart_rate.ch.notify = event->subscribe.cur_notify || event->subscribe.cur_indicate;
        //heart_rate.ch.indicate = event->subscribe.cur_indicate;
        heart_rate.ch.encrypted = is_connection_encrypted(event->subscribe.conn_handle);
        if (heart_rate.ch.encrypted == false) {
            ESP_LOGW(GATT, "failed to subscribe, connection not encrypted!");
            return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
        }
        heart_rate.ch.send = heart_rate.ch.encrypted && (heart_rate.ch.notify /* || heart_rate.ch.indicate */);
        ESP_LOGI(GATT, "ch.send = %u", heart_rate.ch.send);
    } else if(0) {}
    else { ESP_LOGD(GATT, "event->subscribe.attr_handle %u, chr_handle = %u", event->subscribe.attr_handle, heart_rate.chr_handle);}

    return 0;
}

void gatt_svr_register_cb(ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];
    /* Handle GATT attributes register events */
    switch (ctxt->op) {
    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(GATT, "registered service %s with handle=%d", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),ctxt->svc.handle);
        break;
    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(GATT,"registering characteristic %s with ""def_handle=%d val_handle=%d", ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(GATT, "registering descriptor %s with handle=%d",ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
        break;
    //default: assert(0);break;/* Unknown event */
    }
}

