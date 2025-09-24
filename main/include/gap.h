#pragma once
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


/* Includes */
/* NimBLE GAP APIs */
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

/* Defines */
#define BLE_DEBUG
#define DEVICE_NAME CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME //CONFIG_IDF_TARGET
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) + 4 * 2) //with space
#define TIMER_ADV BLE_HS_FOREVER //(1000 * 60 * 10)
#define TIME_SCAN       10 * 1000
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
#define GENERIC         0x18
#define ACCESS          0x00

#ifdef __cplusplus
extern "C" {
#endif
//static int ble_gap_callback_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);
struct ble_hs_adv_fields;
struct ble_gap_conn_desc;
struct ble_hs_cfg;
union ble_store_value;
union ble_store_key;
#ifdef __cplusplus
}
#endif
void adv_init(void);
int gap_init(void);
bool is_connection_encrypted(uint16_t);
extern uint32_t get_pincode();
extern int gatt_svr_subscribe_cb(struct ble_gap_event *);
extern void change_device_name();
extern void patch_func();
extern void print_task_list();

extern void periodic_sync_scan();
extern void parse_adv_data(const uint8_t* const &, uint8_t);
extern void print_event_report(const ble_gap_disc_desc & disc);
extern void print_event_report(const ble_gap_ext_disc_desc & disc);
extern void print_event_report(const decltype(ble_gap_event::periodic_report) & rep);
extern void print_event_report(const decltype(ble_gap_event::periodic_sync) & rep);
extern void print_event_report(const decltype(ble_gap_event::periodic_sync_lost) & rep);
