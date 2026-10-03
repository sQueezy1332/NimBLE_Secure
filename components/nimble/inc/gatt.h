#pragma once
/* #include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h" */
//#include "esp_hid_common.h"

#ifdef __cplusplus
extern "C" {
#endif
struct ble_gap_event; struct ble_gatt_register_ctxt;
typedef uint16_t handle_mask_t;
/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_register_cb(struct ble_gatt_register_ctxt *, void *);
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
void gatt_init(void);

handle_mask_t need_notify_io();
handle_mask_t get_conn_encrypted();
int is_encrypted(uint16_t);
int clear_connection(uint16_t);
int set_encryption(uint16_t);

void send_io_notify();
void send_heart_rate_notify();
void send_spp_notify();

//extern int is_connection_encrypted(uint16_t);
extern void io_on_cb();
extern void io_off_cb();
extern int io_get_cb();
extern uint8_t get_heart_rate();
#ifdef __cplusplus
}
#endif

#define handle_write(dest, num, val) bitWrite(dest, num, val)
#define handle_read(src, num) bitRead(src, num)
#define MIN_HANDLE_VAL (1u)

#define DEVICE_NAME CONFIG_IDF_TARGET //CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1) 
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

#define GENERIC_TAG 0x0200
#define GENERIC_HID_TAG 0x03C0
#define KBD_TAG 0x03C1
#define MOUSE_TAG 0x03C2
#define JOYSTICK_TAG 0x03C3
#define GAMEPAD_TAG 0x03C4
#define MOTION_SENSOR_TAG 0x0541
#define GENERIC_ACCESS_CONTROL 0x0700
#define ACCESS_DOOR_TAG 0x0701
#define DOOR_LOCK_TAG 0x0708

#define CURRENT_TIME_SVC 0x1805
#define HID_SVC 0x1812
#define ESPRESSIF_UUID 0x02E5