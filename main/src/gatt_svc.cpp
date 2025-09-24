/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"
#include "heart_rate.h"
#include "led.h"

/* Private function declarations */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */
/* Heart rate service */
static const ble_uuid16_t heart_rate_svc_uuid = BLE_UUID16_INIT(0x180D);

static uint8_t heart_rate_chr_val[2] = {0};
static uint16_t heart_rate_chr_val_handle;
static const ble_uuid16_t heart_rate_chr_uuid = BLE_UUID16_INIT(0x2A37);

static uint16_t heart_rate_chr_conn_handle = 0;
static bool heart_rate_chr_conn_handle_inited = false;
static bool heart_rate_ind_status = false;
/* Automation IO service */
static const ble_uuid16_t auto_io_svc_uuid = BLE_UUID16_INIT(0x1815);
static const ble_uuid128_t led_chr_uuid = BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef,0x12, 0x12, 0x25, 0x15, 0x00, 0x00); 
//static const ble_uuid16_t led_chr_uuid = BLE_UUID16_INIT(0x07C6);
static uint16_t led_chr_val_handle;

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Heart rate service */
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &heart_rate_svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]) {/* Heart rate characteristic */
        {.uuid = &heart_rate_chr_uuid.u,
          .access_cb = heart_rate_chr_access,
          .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ_ENC,
          .val_handle = &heart_rate_chr_val_handle 
        },{0}/* No more characteristics in this service. */
        }
    },
    /* Automation IO service */
    {   .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &auto_io_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){/* LED characteristic */
            {   .uuid = &led_chr_uuid.u,
                .access_cb = led_chr_access, 
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC, 
                .val_handle = &led_chr_val_handle
            }, {0}
        },
    },
    {0, /* No more services. */},
};

/* Private functions */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Note: Heart rate characteristic is read only */
    switch (ctxt->op) {
    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        /* Verify connection handle */
        ESP_LOGI(GATT, "characteristic read%s; conn_handle %u attr_handle %u", 
            conn_handle != BLE_HS_CONN_HANDLE_NONE ? "" : " by nimble stack" ,conn_handle, attr_handle);
        /* Verify attribute handle */
        if (attr_handle == heart_rate_chr_val_handle) {
            heart_rate_chr_val[1] = get_heart_rate();// Update access buffer value
            int ret = os_mbuf_append(ctxt->om, &heart_rate_chr_val, sizeof(heart_rate_chr_val));
            if(ret) { ESP_LOGE(GATT, "os_mbuf_append ret = %d", ret); return BLE_ATT_ERR_INSUFFICIENT_RES; }
            return 0;
        } else { ESP_LOGW(GATT, "attr_handle %u",attr_handle); }
        goto error;
    }
    default: ESP_LOGW(GATT, "ctxt->op %u",ctxt->op); goto error;/* Unknown event */
    }
error:
    ESP_LOGE(GATT,"unexpected access operation to %s characteristic, opcode: %d", "heart rate", ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

static int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,struct ble_gatt_access_ctxt *ctxt, void *arg) {
    //int rc;/* Note: LED characteristic is write only */
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:/* Write characteristic event */
            ESP_LOGI(GATT, "characteristic write%s conn_handle %u attr_handle %u", 
                conn_handle != BLE_HS_CONN_HANDLE_NONE ? ";" : " by nimble stack;", conn_handle, attr_handle);      
        if (attr_handle == led_chr_val_handle) { /* Verify attribute handle */
            if (ctxt->om->om_len == 1) { /* Verify access buffer length */
                /* Turn the LED on or off according to the operation bit */
                if (ctxt->om->om_data[0]) { led_on(); ESP_LOGI(GATT, "led turned on!");} 
                else { led_off(); ESP_LOGI(GATT, "led turned off!"); }
            } else { goto error;}
            return 0;
        }else { ESP_LOGW(GATT, "attr_handle %u",attr_handle); }
        goto error;
    default: ESP_LOGW(GATT, "ctxt->op %u",ctxt->op); goto error; /* Unknown event */
    }
error:
    ESP_LOGE(GATT,"unexpected access operation to %s characteristic, opcode: %d", "led", ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}
/* Public functions */

void send_heart_rate_indication(void) {
    if (!heart_rate_chr_conn_handle_inited) { return; }
    if (heart_rate_ind_status && is_connection_encrypted(heart_rate_chr_conn_handle)) {
        ble_gatts_indicate(heart_rate_chr_conn_handle, heart_rate_chr_val_handle);
    }
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
    default: assert(0);break;/* Unknown event */
    }
}

int gatt_svr_subscribe_cb(ble_gap_event *event) {
    /* Check attribute handle */
    if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
        /* Update heart rate subscription status */
        heart_rate_chr_conn_handle = event->subscribe.conn_handle;
        heart_rate_chr_conn_handle_inited = true;
        heart_rate_ind_status = event->subscribe.cur_indicate;
        /* Check security status */
        if (!is_connection_encrypted(event->subscribe.conn_handle)) {
            ESP_LOGE(GATT, "failed to subscribe to heart rate measurement, connection not encrypted!");
            return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
        }
    }else { ESP_LOGW(GATT, "event->subscribe.attr_handle %u", event->subscribe.attr_handle);}
    return 0;
}

int gatt_svc_init(void) {
    ble_svc_gatt_init(); /* 1. GATT service initialization */
    int rc = ble_gatts_count_cfg(gatt_svr_svcs); /* 2. Update GATT services counter */
    if (rc) {  ESP_LOGE(GATT,"count_cfg = %d", rc); return rc; } 
    rc = ble_gatts_add_svcs(gatt_svr_svcs);/* 3. Add GATT services */
    if (rc) { ESP_LOGE(GATT,"add_svcs = %d", rc); return rc; }
    return 0;
}
