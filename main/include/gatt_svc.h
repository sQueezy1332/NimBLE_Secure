/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

/* Includes */
/* NimBLE GATT APIs */
/* #include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h" */

#ifdef __cplusplus
extern "C" {
#endif
struct ble_gap_event; struct ble_gatt_register_ctxt;
#ifdef __cplusplus
}
#endif


/* Public function declarations */

//void clear_characteristic();
//void set_encryption();
bool need_notify();
void send_alarm_notify();

void send_heart_rate_notify(void);
/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(ble_gatt_register_ctxt *, void *);
/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */
int gatt_svr_subscribe_cb(ble_gap_event *event);
/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */

int gatt_svc_init(void);

extern bool is_connection_encrypted(uint16_t conn_handle);
extern void impl_io_on();
extern void impl_io_off();
extern uint8_t impl_io_get();

