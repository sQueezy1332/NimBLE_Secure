/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gap.h"
#include "common.h"
//#include "gatt_svc.h"

//#define RANDOM_ADDR

/* Private function declarations */
static const char* format_addr(uint8_t addr[]);
static void print_conn_desc(struct ble_gap_conn_desc *desc);
static void start_advertising(void);
static void set_random_addr(void);
static int gap_event_handler(struct ble_gap_event *event, void *arg);
/* Private variables */
static uint8_t own_addr_type = BLE_HCI_ADV_OWN_ADDR_PUBLIC;
__unused static uint8_t addr_val[6] = {};
__unused static uint8_t esp_uri[] = {BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f', '.', 'c', 'o', 'm'};

/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    ble_gap_conn_desc desc;
    /* Handle different GAP event */
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(GAP, "connection %s; status %d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
            /* Check connection handle */
            ESP_RETURN_ON_ERROR(ble_gap_conn_find(event->connect.conn_handle, &desc), GAP, "find connection by handle" );
            /* Print connection descriptor */
            print_conn_desc(&desc);
            /* Try to update connection parameters */
            ble_gap_upd_params params = {.itvl_min = desc.conn_itvl,.itvl_max = desc.conn_itvl, .latency = 3,
                                            .supervision_timeout = desc.supervision_timeout, .min_ce_len = 0 , .max_ce_len = 0};
            ESP_RETURN_ON_ERROR(ble_gap_update_params(event->connect.conn_handle, &params), GAP, "update connection parameters")   ;                               
        }
        /* Connection failed, restart advertising */
        else { start_advertising(); }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(GAP, "disconnected from peer; reason %d",event->disconnect.reason);
        /* Restart advertising */
        start_advertising();
        break;
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(GAP, "connection updated; status %d",event->conn_update.status);
        /* Print connection descriptor */
        ESP_RETURN_ON_ERROR(ble_gap_conn_find(event->conn_update.conn_handle, &desc), GAP, "find connection by handle");
        print_conn_desc(&desc);
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE: ESP_LOGI(GAP, "discovery complete; reason %d",event->disc_complete.reason);
    break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising completed, restart advertising */
        ESP_LOGI(GAP, "advertise complete; reason %d",event->adv_complete.reason);
        start_advertising();
        break;
    case BLE_GAP_EVENT_NOTIFY_TX:
        if ((event->notify_tx.status != 0) && (event->notify_tx.status != BLE_HS_EDONE)) {
            /* Print notification info on error */
            ESP_LOGW(GAP,"notify event; conn_handle %d attr_handle %d " "status %d is_indication %d",
                     event->notify_tx.conn_handle, event->notify_tx.attr_handle,
                     event->notify_tx.status, event->notify_tx.indication);
        } break;
    case BLE_GAP_EVENT_SUBSCRIBE: {
        ESP_LOGI(GAP,"subscribe event; conn_handle %u attr_handle %u " "reason %u prevn %u curn %u previ %u curi %u",
                 event->subscribe.conn_handle, event->subscribe.attr_handle, event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate, event->subscribe.cur_indicate);
        /* GATT subscribe event callback */
        if (gatt_svr_subscribe_cb(event) == BLE_ATT_ERR_INSUFFICIENT_AUTHEN) {
            return ble_gap_security_initiate(event->subscribe.conn_handle); /* Request connection encryption */
        }
    } break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(GAP, "mtu update event; conn_handle %d cid %d mtu %d",
                 event->mtu.conn_handle, event->mtu.channel_id,
                 event->mtu.value);
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:/* Encryption change event */
        /* Encryption has been enabled or disabled for this connection. */
        if (event->enc_change.status == 0) { ESP_LOGI(GAP, "connection encrypted!"); //ESP_LOGD(GAP,"enc_change.conn_handle = %u", event->enc_change.conn_handle);
            patch_func();
            /* if(!patch_func()) { ESP_LOGE(GAP, "patch_func()");
                ESP_RETURN_ON_ERROR(ble_gap_conn_find(event->enc_change.conn_handle, &desc), GAP," find connection"); 
                ble_store_util_delete_peer(&desc.peer_id_addr);
                return BLE_GAP_REPEAT_PAIRING_RETRY;
            } */
        } else { ESP_LOGW(GAP, "connection encryption failed, status: %d",event->enc_change.status); }
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Delete the old bond */
        ESP_RETURN_ON_ERROR(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc), GAP," find connection");
        ble_store_util_delete_peer(&desc.peer_id_addr);
        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should continue with pairing operation */
        ESP_LOGW(GAP, "repairing...");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            struct ble_sm_io pkey = {
                .action = event->passkey.params.action,
                .passkey = get_pincode()
            };
            ESP_LOGI(GAP, "enter passkey %" PRIu32 " on the peer side",pkey.passkey);
            ESP_RETURN_ON_ERROR(ble_sm_inject_io(event->passkey.conn_handle, &pkey), GAP, "inject security manager io");
        } else { ESP_LOGW(GAP, "passkey.params.action %u",event->passkey.params.action); }
        break; 
    case BLE_GAP_EVENT_DISC: print_event_report(event->disc);
    break;  
    case BLE_GAP_EVENT_EXT_DISC: {
        if(event->ext_disc.data_status != BLE_GAP_EXT_ADV_DATA_STATUS_COMPLETE) ESP_LOGW(GAP,"data_status: %u",event->ext_disc.data_status); //else
        print_event_report(event->ext_disc);
    } break;
    case BLE_GAP_EVENT_PERIODIC_SYNC: print_event_report(event->periodic_sync); break;
    case BLE_GAP_EVENT_PERIODIC_REPORT: print_event_report(event->periodic_report); break;
    case BLE_GAP_EVENT_PERIODIC_SYNC_LOST: print_event_report(event->periodic_sync_lost); break; 
    case BLE_GAP_EVENT_AUTHORIZE: print_task_list(); break;
    default: ESP_LOGI(GAP, "event->type %u",event->type);
    }
    return ESP_OK;
}

static void start_advertising(void) {
    change_device_name();
    const uint8_t *name = (uint8_t *)ble_svc_gap_device_name();
    ble_hs_adv_fields adv_fields = {};
    //__unused struct ble_hs_adv_fields rsp_fields = {0};
    ble_gap_adv_params adv_params = {};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP; //Type 0x01
    adv_fields.name = name;
    adv_fields.name_len = DEVICE_NAME_LEN; //strlen((char*)name);
    adv_fields.name_is_complete = 1; //Type 0x09
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1; //Type 0x0A
    adv_fields.appearance = BLE_GAP_APPEARANCE; 
    adv_fields.appearance_is_present = 1; //Type 0x19
    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1; //Type 0x1C
    ESP_RETURN_VOID_ON_ERROR(ble_gap_adv_set_fields(&adv_fields), GAP, "set advertising data");
    //rsp_fields.device_addr = addr_val;
    //rsp_fields.device_addr_type = own_addr_type;
    //rsp_fields.device_addr_is_present = 1;
    //rsp_fields.uri = esp_uri;
    //rsp_fields.uri_len = sizeof(esp_uri);
    //rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    //rsp_fields.adv_itvl_is_present = 1;
    //ESP_RETURN_VOID_ON_ERROR(ble_gap_adv_rsp_set_fields(&rsp_fields), GAP, "set scan response data");
    /* Set non-connetable and general discoverable mode to be a beacon */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);
    ESP_RETURN_VOID_ON_ERROR(ble_gap_adv_start(own_addr_type, NULL, TIMER_ADV, &adv_params, gap_event_handler,NULL), GAP, "start advertising");
    ESP_LOGI(GAP, "advertising started!");
}

/* Public functions */
void adv_init(void) {
    //change_device_name();
#ifdef RANDOM_ADDR
    set_random_addr();
    ESP_RETURN_VOID_ON_ERROR(ble_hs_util_ensure_addr(true),GAP,  "device does not have any available bt address!");
    /* Figure out BT address to use while advertising */
    ESP_RETURN_VOID_ON_ERROR(ble_hs_id_infer_auto(0, &own_addr_type), GAP, "infer address type");
    /* Copy device address to addr_val */
    ESP_RETURN_VOID_ON_ERROR(ble_hs_id_copy_addr(own_addr_type, addr_val, NULL), GAP, "copy device address");
    ESP_RETURN_ON_ERROR(ble_svc_gap_device_name_set(DEVICE_NAME), GAP, "set device name to %s", DEVICE_NAME);
    ESP_LOGI(GAP, "device address: %s", format_addr(addr_val));
#endif
    start_advertising();
}

void periodic_sync_scan() {
    uint8_t own_addr_type;
    __unused ble_gap_disc_params disc_params = { .itvl = 0, .window = 0, .filter_policy = 0, .limited = 0, .passive = 1, .filter_duplicates = 0, .disable_observer_mode = 0};
    __unused ble_gap_ext_disc_params ext_params = { .itvl = 0, .window = 0, .passive = 0, .disable_observer_mode = 0 };
    /* Figure out address to use while advertising (no privacy for now) */
    ESP_RETURN_VOID_ON_ERROR(ble_hs_id_infer_auto(0, &own_addr_type), TAG, "determining address type");
        
    //ESP_RETURN_VOID_ON_ERROR(ble_gap_disc(BLE_ADDR_PUBLIC,0,&disc_params, gap_event_handler, NULL), TAG, "ble_gap_ext_disc");
    ESP_RETURN_VOID_ON_ERROR(ble_gap_ext_disc(BLE_ADDR_PUBLIC,0, 0, true, 0, 0,&ext_params,&ext_params, gap_event_handler, NULL), TAG, "ble_gap_ext_disc");
};

int gap_init(void) {
    ble_svc_gap_init(); /* Call NimBLE GAP initialization API */
    //ESP_RETURN_ON_ERROR(ble_svc_gap_device_name_set(DEVICE_NAME), GAP, "set device name to %s", DEVICE_NAME); /* Set GAP device name */
    ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE);
    return ESP_OK;
}


bool is_connection_encrypted(uint16_t conn_handle) {
    struct ble_gap_conn_desc desc;
    /* Print connection descriptor */
    int ret = ble_gap_conn_find(conn_handle, &desc);
    if(ret) { ESP_LOGW(GAP, "failed to find connection by handle, ret = %d", ret); return false; }
    return desc.sec_state.encrypted;
}

static const char* format_addr(uint8_t addr[]) {
#if _ESP_LOG_ENABLED(ESP_LOG_INFO)
    static char addr_str[18];
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],addr[2], addr[3], addr[4], addr[5]);
    return addr_str;
#endif
return NULL;
}

static void print_conn_desc(struct ble_gap_conn_desc *desc) {
    ESP_LOGI(GAP, "connection handle: %u", desc->conn_handle);
    ESP_LOGI(GAP, "local address: type %u, value %s", 
        desc->our_id_addr.type, format_addr(desc->our_id_addr.val));
    ESP_LOGI(GAP, "peer address: type %u, value %s", 
        desc->peer_id_addr.type, format_addr(desc->peer_id_addr.val));
    ESP_LOGI(GAP,"conn_itvl=%d, conn_latency=%d, supervision_timeout %u, ""encrypted %u, authenticated %u, bonded %u\n",
             desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
             desc->sec_state.encrypted, desc->sec_state.authenticated,
             desc->sec_state.bonded);
}

__unused static void set_random_addr(void) {
    int rc = 0; ble_addr_t addr;
    rc = ble_hs_id_gen_rnd(0, &addr); assert(rc == 0);
    rc = ble_hs_id_set_rnd(addr.val); assert(rc == 0);
}



