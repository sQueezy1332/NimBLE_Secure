#pragma once
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* NimBLE GAP APIs */
/* #include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h" */

/* Defines */
#define DEVICE_NAME CONFIG_IDF_TARGET //CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1) 
#define TIMER_ADV  (1000 * 60 * 10) //min /* BLE_HS_FOREVER */
#define BLE_GAP_APPEARANCE 0x0541 //0x0200 BLE_GAP_APPEARANCE_GENERIC_TAG
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00
#define UUID16_LIST_INCOM 0x02
#define UUID16_LIST	    0x03
#define UUID16_DATA     0x16
#define UUID32_LIST	    0x05
#define UUID32_DATA	    0x20
#define UUID128_LIST	0x07
#define UUID128_DATA	0x21
#define SHORT_NAME	    0x08
#define COMPLETE_NAME	0x09


#ifdef __cplusplus
extern "C" {
#endif
//static int ble_gap_callback_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);
const char* ble_svc_gap_device_name(void);
int ble_svc_gap_device_name_set(const char *);
void ble_store_config_conf_init();
struct ble_hs_adv_fields; struct ble_gap_conn_desc; struct ble_hs_cfg; 
struct ble_gap_event; struct os_mbuf; struct ble_gatt_register_ctxt;
union ble_store_key; union ble_store_value; 
#ifdef __cplusplus
}
#endif

void ble_hs_cfg_init();
int gap_init();
void host_sync_cb();
void ble_scan_adv();
void adv_init();
int save_bonding(uint16_t conn_handle);

#ifdef __cplusplus
extern "C" {
#endif
bool is_connection_encrypted(uint16_t);
int gatt_svr_subscribe_cb(const ble_gap_event *);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void clear_characteristic();
void set_encryption();
void impl_io_on();
void impl_io_off();
#ifdef __cplusplus
}
#endif

extern uint32_t get_pincode();
extern void parse_adv_cb(const uint8_t*, uint8_t);
extern uint32_t generate_salt();
extern void print_task_list();
extern void set_ble_device_name();
extern void unpatch_cb();
//extern void start_adv();
extern void parse_rx_data(const ble_gap_event* event);
