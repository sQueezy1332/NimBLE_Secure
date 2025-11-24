#pragma once
//#include <FS.h>
//#include <Arduino.h>
#include "ESP_MAIN.h"
#include <WiFi.h>
#define NO_SPIFFS
#include "OTAserver.h"
#include "credentials.h"
#define STAG "Server"

void wifi_ap_init();
void wifi_server_init();
static void onConfigRequest(class AsyncWebServerRequest*);
static void ota_progress(size_t progress, size_t size) __unused;
static void timer_callback(void *);

extern byte scan_key[64];
extern byte scan_key_len, password_len;

template <bool big_endian, char sep> void bytes_to_str(char* ptr, cbyte* buf, byte data_size);
extern void strtoB(const char* ptr, byte *buf, size_t & data_size, size_t buf_len = sizeof(scan_key));
extern void set_main_part();
extern String get_task_list();
extern esp_err_t save_auth_data(const char* pass);
extern void nvsErase();

static char* pass_buf = nullptr;
static esp_timer_handle_t timer_wifi;

void WiFiCallback(arduino_event_id_t event) {
    switch (event) {
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED: log_d("AP_STACONNECTED");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(timer_wifi)); 
		break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: log_d("AP_STADISCONNECTED");
        if (!WiFi.AP.connected()) { 
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(timer_wifi, TIMER_WIFI)); log_d("timer_wifi reset"); } 
		break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED: log_d("AP_STAIPASSIGNED"); break;
	case ARDUINO_EVENT_WIFI_AP_STOP: log_d("WIFI_AP_STOP"); break;
	default: log_d( "%u", event);
    }
}

void timer_callback(void *) {
	ESP_LOGI(STAG, "Timer");
	//if(timer_wifi) { set_main_part(); }
	vTaskDelay(1); esp_restart();
}

void wifi_ap_init() {
	assert(WiFi.mode(WIFI_MODE_AP));
#if	AP_WIFI_CHANNEL > 11
	CHECK_(esp_wifi_set_country_code("CN", false));
#endif 
	assert(WiFi.softAP(AP_SSID, AP_PASS, AP_WIFI_CHANNEL));
	WiFi.setTxPower(WIFI_POWER_21dBm); DEBUG("tx power = "); DEBUGLN(WiFi.getTxPower()/4);
	WiFi.softAPbandwidth(WIFI_BW_HT20);
	DEBUGLN(AP_SSID); DEBUGLN(AP_PASS); DEBUG("My IP address: "); DEBUGLN(WiFi.softAPIP());DEBUGLN();
	WiFi.onEvent(WiFiCallback);
	timer_wifi = esp_timer_init(timer_callback);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(timer_wifi, TIMER_WIFI)); log_d("TIMER_WIFI %lu", TIMER_WIFI);
}

void wifi_server_init() {
	using pReq = AsyncWebServerRequest*;
    static AsyncWebServer server(80);
    //if(!pServer) pServer = new AsyncWebServer(80); auto & server = *pServer;
    //assert(pServer);
	server.onNotFound([](pReq request) { DEBUG('['); DEBUG(request->client()->remoteIP()); DEBUG("] HTTP GET request of "); DEBUGLN(request->url());
		request->send(404, "text/plain", "Not found");
		});
	server.on("/main", HTTP_GET, [](pReq request ) { request->send(200, "text/plain", "reboot to OTA_0...");
		set_main_part(); esp_timer_start_once(timer_wifi, 1000*100); ESP_LOGI(STAG, "reboot to OTA_0..."); } );
    server.on("/save", HTTP_GET, [](pReq request) { auto err = save_auth_data(pass_buf); 
		request->send(200, "text/plain", String(err ? "0x" + String(err, HEX) : "Saved")); });
    server.on("/task_list", HTTP_GET, [](pReq request) { request->send(200, "text/plain", std::move(get_task_list())); });
	server.on("/restart", HTTP_GET, [](pReq request) { request->send(200, "text/plain", "Esp restarting...");
		esp_timer_start_once(timer_wifi, 1000*100); /* timer_wifi = NULL; */ });
	server.on("/nvs_erase_all", HTTP_GET, [](pReq request) { request->send(200, "text/plain", "nvsErase()"); nvsErase();});
	server.on(CHANGE_AUTH, HTTP_GET, [](pReq request) {
#ifndef NO_SPIFFS
        auto response = request->beginResponse(SPIFFS, "/config.html", "text/html"); if(!response) return;
#else
		extern const byte config_html_start[] asm("_binary_config_html_gz_start");
		extern const byte config_html_end[] asm("_binary_config_html_gz_end");
		const size_t html_len = config_html_end - config_html_start; log_d("%lu  start: %p end: %p", html_len, config_html_start, config_html_end);
		auto response = request->beginResponse(200, _HTML, config_html_start, html_len); if(!response) return;
        response->addHeader(asyncsrv::T_Content_Encoding, "gzip");
#endif
		request->send(response);
		});
	server.on(CHANGE_AUTH, HTTP_POST, onConfigRequest);
	ota::server_init(server
#ifdef DEBUG_ENABLE
		, ota_progress
#endif // DEBUG_ENABLE
	);
	server.begin(); log_d("server.begin()");
}

static void onConfigRequest(AsyncWebServerRequest* request) {
	auto pFIRST = request->getParam(0), pSECOND = request->getParam(1);
	if (!pFIRST || !pSECOND) { log_d("getParam() NULL"); return; }
	size_t FIRST_len = pFIRST->value().length(), SECOND_len = pSECOND->value().length();
	bool wrongFIRST = FIRST_len < 16 || FIRST_len > 64;
	bool wrongSECOND = SECOND_len < 8 || SECOND_len > 64*4;
	if (wrongFIRST && wrongSECOND) {
		String str("WRONG INPUT\n");
		str += FIRST_len; str += '\n'; str += SECOND_len;
		request->send(400, "text/plain", str); return;
		} 
		 log_d("%p", pass_buf);//tlsf_free()
	free(pass_buf); pass_buf = nullptr; scan_key_len = 0; password_len = 0; 
	if (!wrongFIRST) {
		String& temp = const_cast<String&>(pFIRST->value());
		/*static_assert(sizeof(String) == 16);
		if (FIRST_len < sizeof(String) - 1) {
				 pass_buf = (char*)malloc(16);
				memcpy(pass_buf, temp.begin(), FIRST_len+1); 
		} else  */
			{ pass_buf = temp.begin(); *reinterpret_cast<long*>(&temp) = 0; }
		password_len = FIRST_len; //pass_buf[FIRST_len] = '\0';
	}
	String& str_second = const_cast<String&>(pSECOND->value());
	if (!wrongSECOND) {
		size_t hex_size, str_size;
		strtoB(str_second.c_str(), scan_key, hex_size);
		if (hex_size >= 8 && hex_size <= 64 && str_second.reserve(str_size = hex_size * 3)) {
			bytes_to_str<true, ' '>(str_second.begin(), scan_key, hex_size);
			reinterpret_cast<long*>(&str_second)[2] = str_size - 1;
			scan_key_len = hex_size;
		}
		//log_d("%s", str_second.c_str()); log_d("str_len %u", str_second.length());
	}DEBUGLN(pass_buf);  DEBUGLN(str_second);
	String log; log.reserve(255);
	//DEBUGF("POST[%s]: %s\n", pFIRST->name().c_str(), pFIRST->value().c_str(), pSECOND->name().c_str(), pSECOND->value().c_str());
	log = "PASS ["; if (!wrongFIRST) log.concat(pass_buf, password_len); log += "]\n";
	log += "SCAN_KEY ["; if (!wrongSECOND) log += str_second; log += "]\n";
	log += "password_len = "; log += password_len;
	log += "\nscan_key_len = "; log += scan_key_len;
	log += "\ngoto /save.";
	DEBUGLN(log); request->send(200, "text/plain", log); //resumeTask(UPD_AUTH);
}

 static void ota_progress(size_t progress, size_t size) {
	if (progress == 0) { DEBUGF("OTA overall size bytes: %u\n", size);};
	if ((progress & 0x3FFF) == 0) { DEBUGLN(progress); }
}