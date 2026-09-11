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

void ble_hs_cfg_init();
void ble_scan_init();
void adv_init();
int save_bonding(uint16_t h_conn);
int is_connection_encrypted(uint16_t h_conn);

__weak_symbol void host_sync_cb();
__weak_symbol uint32_t get_pincode();
__weak_symbol uint32_t generate_uuid32();
__weak_symbol void parse_adv(const struct ble_gap_ext_disc_desc*);
__weak_symbol void adv_complete_cb();
__weak_symbol void scan_complete_cb();
__weak_symbol void connect_err_cb(int);
__weak_symbol void disconnect_cb();
__weak_symbol void conn_encrypted_cb(); 
__weak_symbol int parse_rx_data(const struct ble_gap_event*);

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)

typedef struct {
	//struct ble_store_value_sec our_secs;
	struct ble_store_value_sec peer_secs;
} my_ble_store_t; //MYNEWT_VAL_BLE_MAX_CONNECTIONS

struct my_ble_store_nvs {
    ble_addr_t peer_addr;
#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    uint16_t bond_count;
#endif
    uint8_t key_size;
    uint16_t ediv;
    uint64_t rand_num;
    uint8_t ltk[16];
	uint8_t irk[16];
    //uint8_t csrk[16];
	//uint8_t csrk_present:1;
    uint8_t ltk_present:1;
	uint8_t irk_present:1;
	uint8_t authenticated:1;
	uint8_t sc:1;
	uint32_t sign_counter;
};

#endif /* !MYNEWT_VAL(BLE_STORE_MAX_BONDS) */

#ifdef __cplusplus
}
#endif
