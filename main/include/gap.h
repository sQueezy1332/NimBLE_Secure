#pragma once
/* NimBLE GAP APIs */
//#include "host/ble_gap.h"
//#include "services/gap/ble_svc_gap.h"
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
extern void ble_store_config_init(void);
const char* ble_svc_gap_device_name(void);
int ble_svc_gap_device_name_set(const char *);
void ble_store_config_conf_init(); //CONFIG_BT_NIMBLE_NVS_PERSIST=y
struct ble_hs_adv_fields; struct ble_gap_conn_desc; struct ble_hs_cfg; 
struct ble_gap_event; struct os_mbuf; struct ble_gatt_register_ctxt;
union ble_store_key; union ble_store_value;

int gap_init();
void ble_hs_cfg_init();
void ble_scan_init();
void adv_init();
int save_bonding(uint16_t h_conn);
int is_connection_encrypted(uint16_t h_conn);

extern uint32_t get_pincode();
extern void parse_adv_cb(const struct ble_gap_ext_disc_desc*);
extern uint32_t generate_salt();
extern void print_task_list();
extern void set_ble_device_name();
extern void adv_complete_cb();
extern void disc_complete_cb();
extern void conn_encrypted_cb();
extern void parse_rx_data(const struct ble_gap_event*);

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
extern struct ble_store_value_sec
    ble_store_config_our_secs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
extern int ble_store_config_num_our_secs;

extern uint16_t ble_store_config_our_bond_count;
extern uint16_t ble_store_config_peer_bond_count;

extern struct ble_store_value_sec
    ble_store_config_peer_secs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];

extern int ble_store_config_num_peer_secs;

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
extern struct ble_store_value_cccd
    ble_store_config_cccds[MYNEWT_VAL(BLE_STORE_MAX_CCCDS)];
extern int ble_store_config_num_cccds;
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
extern struct ble_store_value_csfc
     ble_store_config_csfcs[MYNEWT_VAL(BLE_STORE_MAX_CSFCS)];
extern int ble_store_config_num_csfcs;
#endif

extern struct ble_store_value_rpa_rec
    ble_store_config_rpa_recs[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
extern int ble_store_config_num_rpa_recs;

extern struct ble_store_value_local_irk
    ble_store_config_local_irks[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
extern int ble_store_config_num_local_irks;

#endif /* !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

typedef struct {
	struct ble_store_value_sec our_secs;
	struct ble_store_value_sec peer_secs;
	//struct ble_store_value_rpa_rec ble_store_config_rpa_recs;
} my_ble_store_t; //MYNEWT_VAL_BLE_MAX_CONNECTIONS

#endif /* !MYNEWT_VAL(BLE_STORE_MAX_BONDS) */

#ifdef __cplusplus
}
#endif
