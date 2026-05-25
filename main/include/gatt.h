#pragma once
/* Includes */
/* NimBLE GATT APIs */
/* #include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h" */

#ifdef __cplusplus
extern "C" {
#endif
struct ble_gap_event; struct ble_gatt_register_ctxt;
/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *, void *);
/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */
int gatt_svr_subscribe_cb(const struct ble_gap_event *event);
/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
void gatt_svr_init(void);

int need_notify_io();
int clear_connection(uint16_t);
int set_encryption(uint16_t);
int get_encryption(uint16_t);
void send_alarm_notify();
void send_heart_rate_notify();
void send_spp_notify();

//extern int is_connection_encrypted(uint16_t);
extern void impl_io_on();
extern void impl_io_off();
extern int impl_io_get();

extern uint8_t get_heart_rate();
extern void update_heart_rate();
#ifdef __cplusplus
}
#endif
