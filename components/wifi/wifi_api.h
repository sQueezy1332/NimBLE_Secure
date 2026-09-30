#pragma once
#define _WANT_USE_LONG_TIME_T
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_mac.h"
#include "esp_log.h"

#define ALIGN_TO_16(size) (((size) + 0xF) & ~0xF)

#define DHCPS_OFFER_DNS			0x02

#define WIFI_FAIL_BIT      		BIT0
#define WIFI_STA_CONNECTED  	BIT1
#define WIFI_STA_GOT_IP 		BIT2
#define WIFI_AP_CONNECTED 		BIT3
#define WIFI_AP_IPASSIGNED		BIT4
#define WIFI_REASON_HANDSHAKE_TOUT	BIT5

extern EventGroupHandle_t h_group_wifi;
extern esp_event_handler_instance_t h_event_wifi, h_event_ip;
extern esp_netif_t* h_netif_sta;
extern esp_netif_t* h_netif_ap;

void wifi_setup_default(wifi_storage_t = WIFI_STORAGE_RAM);

esp_err_t wifi_init_sta(const char* ssid, const char* pass, uint8_t channel = 0, wifi_bandwidth_t = WIFI_BW20);

esp_err_t wifi_init_ap(const char* ssid = CONFIG_IDF_TARGET, const char* pass = NULL, 
	uint8_t channel = 0, uint8_t max_conn = 3, bool hidden = 0, int8_t power = 80, wifi_bandwidth_t = WIFI_BW20);

void ap_set_dns_addr(esp_netif_t *ap,esp_netif_t *sta);

const char* print_mac(const uint8_t mac[6]);

const char* print_ip(uint32_t addr);

uint32_t wifi_is_connected();

int wifi_ap_get_sta_num();