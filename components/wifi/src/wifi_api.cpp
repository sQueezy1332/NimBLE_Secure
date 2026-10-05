#include "wifi_api.h"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

#define MACSTR_SIZE ALIGN_TO_16(sizeof(MACSTR))
#define IPSTR_SIZE ALIGN_TO_16(sizeof(IPSTR))

static const char *TAG = "wifi";
__unused static const char *TAG_AP = "SoftAP";
__unused static const char *TAG_STA = "STA";

EventGroupHandle_t h_group_wifi;
esp_event_handler_instance_t h_event_wifi, h_event_ip;
__unused esp_netif_t* h_netif_sta;
__unused esp_netif_t* h_netif_ap;


__weak_symbol void wifi_connected_cb(wifi_event_sta_connected_t*) {}
__weak_symbol void wifi_ap_connected_cb(wifi_event_ap_staconnected_t*) {}
__weak_symbol void wifi_disconnected_cb(wifi_event_sta_disconnected_t*) {}
__weak_symbol void wifi_ap_disconnected_cb(wifi_event_ap_stadisconnected_t*) {}
__unused static char BUFF[MACSTR_SIZE];

const char* print_mac(const uint8_t mac[6]) {
	sprintf(BUFF, MACSTR, MAC2STR(mac));
	return BUFF;
}
			 
const char* print_ip(uint32_t addr) {
	const uint8_t* const ip = (uint8_t*)&addr;
	sprintf(BUFF, IPSTR, ip[0], ip[1], ip[2], ip[3]);
	return BUFF;
}

int wifi_ap_get_sta_num() {
	wifi_sta_list_t list; 
	esp_err_t ret = esp_wifi_ap_get_sta_list(&list);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
	if(ret) return 0;
	ESP_LOGI(TAG, "sta_list.num %d", list.num);
	return list.num;
}

uint32_t wifi_is_connected() {
	EventBits_t bits = xEventGroupGetBits(h_group_wifi);
	return bits & (WIFI_STA_GOT_IP);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	__unused int ret;
	switch (event_id) {
					/* Station*/
	case WIFI_EVENT_STA_START: ESP_LOGD(TAG_STA, "START"); 
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());break;
	case WIFI_EVENT_STA_STOP: ESP_LOGD(TAG_STA, "STOP"); break;
	case WIFI_EVENT_STA_CONNECTED: {
		__unused wifi_event_sta_connected_t* e = (wifi_event_sta_connected_t*) event_data;
		wifi_connected_cb(e);
		//e->ssid[e->ssid_len & 31] = '\0'; ESP_LOGI(TAG_STA, "CONNECTED to:%s", (char*)e->ssid);
		ESP_LOGI(TAG_STA, "bssid %s, channel %u, AID %u", print_mac(e->bssid), e->channel, e->aid);
		xEventGroupSetBits(h_group_wifi, WIFI_STA_CONNECTED);
	}
		break;
	case WIFI_EVENT_STA_DISCONNECTED:{
		wifi_event_sta_disconnected_t* e = (wifi_event_sta_disconnected_t*) event_data;
		switch (e->reason) {
			case WIFI_REASON_ASSOC_LEAVE: ESP_LOGW(TAG_STA, "ASSOC_LEAVE"); return; //esp_wifi_stop()
			case WIFI_REASON_STA_LEAVING: ESP_LOGW(TAG_STA, "STA_LEAVING"); break; //esp_wifi_disconnect()
			case WIFI_REASON_AUTH_EXPIRE: ESP_LOGW(TAG_STA, "AUTH_EXPIRE"); break; 
			case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY: ESP_LOGW(TAG_STA, "INACTIVITY"); break;
			case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: ESP_LOGW(TAG_STA, "4WAY_HANDSHAKE_TIMEOUT"); break;
			case WIFI_REASON_NO_AP_FOUND: //ESP_LOGW(TAG_STA, "NO_AP_FOUND"); 
				break;
			case WIFI_REASON_CONNECTION_FAIL: ESP_LOGW(TAG_STA, "CONNECTION_FAIL"); break;
			case WIFI_REASON_AUTH_LEAVE: ESP_LOGW(TAG_STA, "AUTH_LEAVE"); break;
			default: ESP_LOGW(TAG_STA, "DISCONNECTED" " reason %u", e->reason); break;
		}
		xEventGroupClearBits(h_group_wifi, WIFI_STA_CONNECTED);
		wifi_disconnected_cb(e);
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
	}
	break; 
	case WIFI_EVENT_STA_AUTHMODE_CHANGE: ESP_LOGI(TAG_STA, "AUTHMODE_CHANGE"); break;
	case WIFI_EVENT_STA_BEACON_TIMEOUT: ESP_LOGI(TAG_STA, "BEACON_TIMEOUT"); break;
	case WIFI_EVENT_HOME_CHANNEL_CHANGE: ESP_LOGI(TAG_STA, "HOME_CHANNEL_CHANGE"); break;
				/* Access Point*/
	case WIFI_EVENT_AP_START: ESP_LOGD(TAG_AP, "START"); break;
	case WIFI_EVENT_AP_STOP: ESP_LOGI(TAG_AP, "STOP"); break;
	case WIFI_EVENT_AP_STACONNECTED: {
		wifi_event_ap_staconnected_t * e = (wifi_event_ap_staconnected_t *)event_data;
		wifi_ap_connected_cb(e);
		//ESP_LOGI(TAG_AP, "Station %s joined, AID %u", print_mac(e->mac), e->aid);
		xEventGroupSetBits(h_group_wifi, WIFI_AP_CONNECTED);
	}
	break;
	case WIFI_EVENT_AP_STADISCONNECTED: {
			wifi_event_ap_stadisconnected_t * e = (wifi_event_ap_stadisconnected_t *)event_data;
			ESP_LOGI(TAG_AP, "Station %s left, AID %d, reason %d", print_mac(e->mac), e->aid, e->reason);
			wifi_ap_disconnected_cb(e);
			if(wifi_ap_get_sta_num() > 0) break;
			xEventGroupClearBits(h_group_wifi, WIFI_AP_CONNECTED | WIFI_AP_IPASSIGNED);
		} break;
	case WIFI_EVENT_AP_WRONG_PASSWORD: ESP_LOGI(TAG_AP, "WRONG_PASSWORD"); break;

	default: ESP_LOGW(WIFI_EVENT, "event_id %d", event_id); 
	}
}

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	switch (event_id) { 
					/* Station*/
	case IP_EVENT_STA_GOT_IP: {
		__unused ip_event_got_ip_t* e = (ip_event_got_ip_t*) event_data;
		xEventGroupSetBits(h_group_wifi, WIFI_STA_GOT_IP);
	}
		break;
	case IP_EVENT_STA_LOST_IP: ESP_LOGI(TAG_STA, "STA_LOST_IP");
		xEventGroupClearBits(h_group_wifi, WIFI_STA_GOT_IP); 
		break;
	case IP_EVENT_NETIF_UP: ESP_LOGI(TAG_STA, "NETIF_UP"); break;
	case IP_EVENT_NETIF_DOWN: ESP_LOGI(TAG_STA, "NETIF_DOWN"); break;
			/* Access Point*/			
	case IP_EVENT_ASSIGNED_IP_TO_CLIENT: {
		__unused ip_event_assigned_ip_to_client_t* e = (ip_event_assigned_ip_to_client_t*)event_data;
		ESP_LOGI(TAG_AP, "hostname '%s'", e->hostname); //IP log internal?
		xEventGroupSetBits(h_group_wifi, WIFI_AP_IPASSIGNED);
	} break;
	default: ESP_LOGW(IP_EVENT, "event_id %d", event_id); 
	}

}

void wifi_setup_default(wifi_storage_t storage) {
	if (h_group_wifi) { ESP_LOGE(TAG, "already inited"); return; }
	h_group_wifi = xEventGroupCreate(); assert(h_group_wifi);
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h_event_wifi));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL, &h_event_ip));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(storage));
}

esp_err_t wifi_init_sta(const char* ssid, const char* pass, uint8_t channel, wifi_bandwidth_t bw) {
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	h_netif_sta = esp_netif_create_default_wifi_sta();
	size_t ssid_len = strlen(ssid), pass_len = strlen(pass);
		wifi_config_t wifi_config = { };
		wifi_config.sta.channel = channel;
		wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	if(ssid_len < sizeof(wifi_config.sta.ssid)) {
		memcpy(wifi_config.sta.ssid, ssid, ssid_len);
	} else return ESP_ERR_WIFI_SSID;
	if(pass_len <= sizeof(wifi_config.sta.password) && (pass_len >= 8)) {
		memcpy(wifi_config.sta.password, pass, pass_len); 
	} else return ESP_ERR_WIFI_PASSWORD;
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	wifi_bandwidths_t band = {.ghz_2g = bw }; 
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_bandwidths(WIFI_IF_STA, &band));
	ESP_LOGI(TAG_STA, "%s finished.", __FUNCTION__);
	ESP_LOGD(TAG_STA, "SSID '%s' pass '%s'", wifi_config.sta.ssid, wifi_config.sta.password);
	return ESP_OK;
    
}

esp_err_t wifi_init_ap(const char* ssid, const char* pass, uint8_t channel, uint8_t max_conn, bool hidden, int8_t power, wifi_bandwidth_t bw) {
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	h_netif_ap = esp_netif_create_default_wifi_ap();
	size_t ssid_len = strlen(ssid), pass_len = strlen(pass);
	wifi_config_t wifi_ap_config { .ap = {
			//.ssid_len = ssid_len,
			.channel = channel,
			.authmode = pass ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN,
			.ssid_hidden = hidden,
			.max_connection = max_conn,
			//.pmf_cfg = { .required = false, },
		}};
	if(ssid_len < sizeof(wifi_ap_config.ap.ssid)) {
		memcpy(wifi_ap_config.ap.ssid, ssid, ssid_len);
		wifi_ap_config.ap.ssid_len = (uint8_t)ssid_len;
	} else return ESP_ERR_WIFI_SSID;
	if(pass_len <= sizeof(wifi_ap_config.ap.password) && pass_len >= 8) {
		memcpy(wifi_ap_config.ap.password, pass, pass_len); 
	} else return ESP_ERR_WIFI_PASSWORD;
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
	esp_wifi_set_max_tx_power(power); wifi_bandwidths_t band = {.ghz_2g = bw};
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_bandwidths(WIFI_IF_AP, &band));
	ESP_LOGI(TAG_AP, "%s finished.", __FUNCTION__);
	ESP_LOGD(TAG_AP, "SSID '%s' pass '%s'", wifi_ap_config.ap.ssid, wifi_ap_config.ap.password);
	ESP_LOGI(TAG_AP, "channel %u max_conn %u", wifi_ap_config.ap.channel, wifi_ap_config.ap.max_connection);
	return ESP_OK;
}

void ap_set_dns_addr(esp_netif_t *ap, esp_netif_t *sta) {
	__unused const char* TAG = "NAPT";
	esp_netif_dns_info_t dns; uint8_t dhcps_offer_option = DHCPS_OFFER_DNS;
	esp_netif_get_dns_info(sta,ESP_NETIF_DNS_MAIN,&dns);
	ESP_LOGI(TAG, "dns %s", print_ip(dns.ip.u_addr.ip4.addr));
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(ap));
	ESP_ERROR_CHECK(esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_offer_option, sizeof(dhcps_offer_option)));
	ESP_ERROR_CHECK(esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns));
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(ap));
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_set_default_netif(sta));
	int ret = esp_netif_napt_enable(ap);
	if(ret) { ESP_LOGE(TAG, "0x%02X",ret); } else { ESP_LOGI(TAG, "enabled");}
}