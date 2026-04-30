#include "wifi_api.h"
#include "credentials.h"

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STATION_MODE
#define ACCESS_POINT_MODE

static const char *TAG = "wifi_init";
static const char *TAG_AP = "SoftAP";
static const char *TAG_STA = "STA";

static uint8_t retry_num = 0;
__unused int8_t sta_count = 0;

EventGroupHandle_t h_group_wifi = NULL;
esp_event_handler_instance_t h_event_wifi, h_event_ip;
esp_timer_handle_t h_timer_wifi;

#define MACSTR_SIZE ALIGN_TO_16(sizeof(MACSTR))
#define IPSTR_SIZE ALIGN_TO_16(sizeof(IPSTR))

void print_mac(char *buf, const uint8_t (&mac)[6]) {
	sprintf(buf, MACSTR, MAC2STR(mac));
}
			 
void print_ip(char *buf, uint32_t addr) {
	const uint8_t* const ip = (uint8_t*)&addr;
	sprintf(buf, IPSTR, ip[0], ip[1], ip[2], ip[3]);
}

int wifi_ap_get_sta_num() {
	wifi_sta_list_t list; 
	esp_err_t ret = esp_wifi_ap_get_sta_list(&list);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
	if(ret) return 0;
	ESP_LOGI(TAG, "sta_list.num %d", list.num);
	return list.num;
}

int wifi_is_connected() {
	EventBits_t bits = xEventGroupGetBits(h_group_wifi);
	return bits & (WIFI_STA_GOT_IP);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	switch (event_id) {
#ifdef STATION_MODE						/* Station*/
	case WIFI_EVENT_STA_START: ESP_LOGD(TAG_STA, "START");
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
		break;
	case WIFI_EVENT_STA_STOP: ESP_LOGD(TAG_STA, "STOP");
		break;
	case WIFI_EVENT_STA_CONNECTED: { //ESP_LOGI(TAG_STA, "STA_CONNECTED");
		retry_num = 0;
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(h_timer_wifi));
		__unused auto e = (wifi_event_sta_connected_t*) event_data;
		char buf[MACSTR_SIZE]; print_mac(buf, e->bssid);
		//e->ssid[e->ssid_len & 31] = '\0'; ESP_LOGI(TAG_STA, "CONNECTED to:%s", (char*)e->ssid);
		ESP_LOGI(TAG_STA, "bssid %s, channel %u, AID %u", buf, e->channel, e->aid);
		xEventGroupSetBits(h_group_wifi, WIFI_STA_CONNECTED);
	}
		break;
	case WIFI_EVENT_STA_DISCONNECTED:{ //wifi_err_reason_t;
		__unused auto e = (wifi_event_sta_disconnected_t*) event_data;
		ESP_LOGW(TAG_STA, "DISCONNECTED" " reason %u retry %u", e->reason, retry_num);
    	// 201,    /**< No AP found */
		if(e->reason == WIFI_REASON_ASSOC_LEAVE //|| e->reason == WIFI_REASON_AUTH_LEAVE /*3*/
		) return; //if esp_wifi_connect called
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start(h_timer_wifi, TIMER_WIFI));
		xEventGroupClearBits(h_group_wifi, WIFI_STA_CONNECTED);  //reset flag
		//201 NO_AP_FOUND //205 CONNECTION_FAIL //15 4WAY_HANDSHAKE_TIMEOUT //2 AUTH_EXPIRE
		if (1 && (retry_num < CONN_MAXIMUM_RETRY)) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
			retry_num++;
		} else {
			//ESP_LOGW(TAG_STA, "DISCONNECTED");
			xEventGroupSetBits(h_group_wifi, WIFI_FAIL_BIT);
		} 
	}
	break; 
	case WIFI_EVENT_STA_AUTHMODE_CHANGE: ESP_LOGI(TAG_STA, "AUTHMODE_CHANGE");
		break;
	case WIFI_EVENT_STA_BEACON_TIMEOUT: ESP_LOGI(TAG_STA, "BEACON_TIMEOUT");
		break;
#endif
#ifdef ACCESS_POINT_MODE				/* Access Point*/
	case WIFI_EVENT_AP_START: ESP_LOGD(TAG_AP, "START");
		break;
	case WIFI_EVENT_AP_STOP: ESP_LOGI(TAG_AP, "STOP");
		break;
	case WIFI_EVENT_AP_STACONNECTED: {
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(h_timer_wifi));
		__unused auto e = (const wifi_event_ap_staconnected_t *)event_data;
		//char buf[MACSTR_SIZE]; print_mac(buf, e->mac);ESP_LOGI(TAG_AP, "Station %s joined, AID %u", buf, e->aid);
		xEventGroupSetBits(h_group_wifi, WIFI_AP_STACONNECTED);
	}
	break;
	case WIFI_EVENT_AP_STADISCONNECTED: {
		auto e = (const wifi_event_ap_stadisconnected_t *)event_data;
		char buf[MACSTR_SIZE]; print_mac(buf, e->mac);
		ESP_LOGI(TAG_AP, "Station %s left, AID %d, reason %d", buf, e->aid, e->reason);
		if(wifi_ap_get_sta_num() > 0) 
			break;
		xEventGroupClearBits(h_group_wifi, WIFI_AP_STACONNECTED | WIFI_AP_STAIPASSIGNED);
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start(h_timer_wifi, TIMER_WIFI));
		}
		break;
	case WIFI_EVENT_AP_WRONG_PASSWORD: ESP_LOGI(TAG_AP, "WRONG_PASSWORD");
		break;
#endif
	default:
		ESP_LOGW(WIFI_EVENT, "event_id %d", event_id); 
		//HOME_CHANNEL_CHANGE 43
	}
}

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	switch (event_id) { 
#ifdef STATION_MODE						/* Station*/
	case IP_EVENT_STA_GOT_IP: {
		__unused auto e = (const ip_event_got_ip_t*) event_data;
		//char buf[IPSTR_SIZE]; print_ip(buf, e->ip_info.ip.addr); ESP_LOGI(TAG_STA, "Got IP: %s", buf);
		xEventGroupSetBits(h_group_wifi, WIFI_STA_GOT_IP);
	}
		break;
	case IP_EVENT_STA_LOST_IP: ESP_LOGI(TAG_STA, "STA_LOST_IP");
		xEventGroupClearBits(h_group_wifi, WIFI_STA_GOT_IP);
		break;
#endif
#ifdef ACCESS_POINT_MODE				/* Access Point*/			
	case IP_EVENT_ASSIGNED_IP_TO_CLIENT: {
		__unused auto e = (const ip_event_assigned_ip_to_client_t*)event_data;
		//char buf[IPSTR_SIZE]; print_ip(buf, e->ip.addr);
		//ESP_LOGI(TAG_AP, "Assigned IP to client: %s, MAC " ", hostname '%s'", buf, e->hostname);
		ESP_LOGI(TAG_AP, "hostname '%s'", e->hostname);
		xEventGroupSetBits(h_group_wifi, WIFI_AP_STAIPASSIGNED);
	}
	break;
#endif
	default: //IP_EVENT_TX_RX = 8 //IP_EVENT_NETIF_UP = 9 //IP_EVENT_NETIF_DOWN = 10
		ESP_LOGW(IP_EVENT, "event_id %d", event_id); 
	}

}

void wifi_setup_default(wifi_mode_t mode, wifi_storage_t storage) {
	if (h_group_wifi) { ESP_LOGE(TAG, "already inited"); return;}
	h_group_wifi = xEventGroupCreate(); assert(h_group_wifi);
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h_event_wifi));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL, &h_event_ip));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(storage));
	ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
}

esp_netif_t* wifi_init_sta() {
	esp_netif_t* esp_netif_sta = esp_netif_create_default_wifi_sta();
	wifi_config_t wifi_config = {//[-Wmissing-field-initializers]
		.sta = {
			.ssid = STA_SSID,
			.password = STA_PASS,
			.threshold { .authmode = WIFI_AUTH_WPA_WPA2_PSK }
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	wifi_bandwidths_t band = {.ghz_2g = WIFI_BW20}; 
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_bandwidths(WIFI_IF_STA, &band));
	ESP_LOGI(TAG_STA, "%s finished.", __FUNCTION__);
	ESP_LOGI(TAG_STA, "SSID '%s' pass '%s'", wifi_config.sta.ssid, wifi_config.sta.password);
	return esp_netif_sta;
}

esp_netif_t* wifi_init_ap() {
	esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();
	wifi_config_t wifi_ap_config = {
		.ap = {
			.ssid = AP_SSID,
			.password = AP_PASS,
			.ssid_len = strlen(AP_SSID),
			.channel = 0,//AP_WIFI_CHANNEL,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.max_connection = AP_MAX_CONN,
			//.pmf_cfg = { .required = false, },
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
	//esp_wifi_set_max_tx_power(84);  
	//wifi_bandwidths_t band = {.ghz_2g = WIFI_BW20}; esp_wifi_set_bandwidths(WIFI_IF_AP, &band);
	ESP_LOGI(TAG_AP, "%s finished.", __FUNCTION__);
	ESP_LOGI(TAG_AP, "SSID '%s' pass '%s'", wifi_ap_config.ap.ssid, wifi_ap_config.ap.password);
	ESP_LOGI(TAG_AP, "channel %u max_conn %u", wifi_ap_config.ap.channel, wifi_ap_config.ap.max_connection);
	return esp_netif_ap;
}

void ap_set_dns_addr(esp_netif_t *ap,esp_netif_t *sta) {
	esp_netif_dns_info_t dns; uint8_t dhcps_offer_option = DHCPS_OFFER_DNS;
	esp_netif_get_dns_info(sta,ESP_NETIF_DNS_MAIN,&dns);
	char buf[IPSTR_SIZE]; print_ip(buf, dns.ip.u_addr.ip4.addr); ESP_LOGI(TAG_STA, "dns %s", buf);
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(ap));
	ESP_ERROR_CHECK(esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_offer_option, sizeof(dhcps_offer_option)));
	ESP_ERROR_CHECK(esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns));
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(ap));
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_set_default_netif(sta));
	int ret = esp_netif_napt_enable(ap);
	if(ret) { ESP_LOGE(TAG, "NAPT err 0x%X",ret); } else{ ESP_LOGI(TAG, "NAPT enabled");}
}
