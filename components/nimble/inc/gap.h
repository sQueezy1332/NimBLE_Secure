#pragma once
//#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#ifdef __cplusplus
extern "C" {
#endif
extern void ble_store_config_init(void);
const char* ble_svc_gap_device_name(void);
int ble_svc_gap_device_name_set(const char *);
void ble_store_config_conf_init(); //CONFIG_BT_NIMBLE_NVS_PERSIST=y
struct ble_hs_adv_fields; struct ble_gap_conn_desc; struct ble_hs_cfg; 
struct ble_gap_event; struct os_mbuf; struct ble_gatt_register_ctxt;
union ble_store_key; union ble_store_value;

void ble_hs_cfg_init();
void ble_scan_init();
void set_scan_periods(uint16_t interval, uint16_t window);
void adv_init(uint16_t field, uint32_t duration_units);
int save_bonding(uint16_t h_conn);
int is_connection_encrypted(uint16_t h_conn);

void host_sync_cb();
uint32_t generate_pin();
uint32_t generate_uuid32();
void parse_adv_cb(const struct ble_gap_ext_disc_desc*);
void ble_adv_complete_cb();
void ble_scan_complete_cb();
void ble_connect_cb(int);
int ble_disconnect_cb(uint16_t);
int ble_conn_encrypted_cb(uint16_t); 
int parse_rx_data_cb(const struct ble_gap_event*);
