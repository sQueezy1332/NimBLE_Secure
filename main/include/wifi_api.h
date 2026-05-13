#pragma once
#define _WANT_USE_LONG_TIME_T
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_mac.h"
#include "esp_log.h"

#define STATION_MODE
#define ACCESS_POINT_MODE

#define ALIGN_TO_16(size) (((size) + 15) & ~15)

#define DHCPS_OFFER_DNS			0x02
#define WIFI_FAIL_BIT      		BIT0
#define WIFI_STA_CONNECTED  	BIT1
#define WIFI_STA_GOT_IP 		BIT2
#define WIFI_AP_STACONNECTED 	BIT3
#define WIFI_AP_STAIPASSIGNED	BIT4
#define WIFI_REASON_HANDSHAKE_TOUT	BIT5


//struct EventGroupHandle_t; //struct esp_netif_t;
extern EventGroupHandle_t h_group_wifi;
extern esp_event_handler_instance_t h_event_wifi, h_event_ip;

extern void wifi_timer_stop();

extern void wifi_timer_start();

void wifi_setup_default(wifi_mode_t, wifi_storage_t = WIFI_STORAGE_FLASH);

esp_netif_t* wifi_init_sta();

esp_netif_t* wifi_init_ap();

void ap_set_dns_addr(esp_netif_t *ap,esp_netif_t *sta);

const char* print_mac(const uint8_t (&mac)[6]);

const char* print_ip(uint32_t addr);

int wifi_is_connected();

int wifi_ap_get_sta_num();
