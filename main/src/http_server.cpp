#define _WANT_USE_LONG_TIME_T
///#include "esp_main.h"
//#include <string.h>
//#include <freertos/FreeRTOS.h>
#include <esp_http_server.h>
//#include <freertos/task.h>
#include <esp_ota_ops.h>
//#include <esp_system.h>
//#include <sys/param.h>
#include "esp_check.h"
#include <memory>
 //struct

#define OTA_BUF_SIZE (0x1000)
#define HTTPD_401      "401 UNAUTHORIZED"
#define PART_FMT "%s 0x%X, %s->size %lu, offset 0x%lX; content_len %u"

static const char* TAG = "HTTP";
static const char* save_uri = "/save_config";
constexpr uint32_t strlen_const(const char* str) { return __builtin_strlen(str); }
__unused static esp_err_t basic_auth_get_handler(httpd_req_t *req);
httpd_handle_t http_server;

extern uint8_t scan_key[32]; extern uint8_t pass_key[32];
extern uint8_t pass_key_len, scan_key_len;
extern int wifi_is_connected();
extern size_t strtoB(const char* src, uint8_t *dest, size_t buf_len);
extern int bytes_to_str_bigend(const uint8_t* src, char* dest, size_t data_size);
//extern void set_main_part();
extern esp_err_t wifi_timer_reset(uint32_t);
extern void revoke_ota_rollback();
extern esp_err_t save_auth_data();
extern void nvsEraseAll(const char* = nullptr);
extern int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
//extern int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
extern int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len);
extern std::unique_ptr<char[]> task_list(size_t* len);

static esp_err_t httpd_resp_send_text(httpd_req_t *req, const char *buf, ssize_t buf_len, esp_err_t ret = 0) {
	httpd_resp_set_status(req, ret == ESP_OK ? HTTPD_200 : HTTPD_500); 
    httpd_resp_set_type(req, "text/plain");
	return httpd_resp_send(req, buf, buf_len);
}

static esp_err_t update_post_handler(httpd_req_t *req) {
	int ret; char buf[128]; esp_ota_handle_t _h;
	auto send_error = [&req, &ret, &buf](const char *usr_msg) {
		sprintf(buf, "%s 0x%X", usr_msg, ret);
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf); //log internal
	};
	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	if(!ota_partition) {
		return send_error("!ota_partition");;
	}
	if((ret = esp_ota_begin(ota_partition, req->content_len, &_h))) {
		sprintf(buf, PART_FMT, "ota_begin()",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf); //log internal
		
	}
	ESP_LOGI(TAG, PART_FMT, "ota_begin()",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
	auto deleter = [](esp_ota_handle_t* p) { if(p) { ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort((esp_ota_handle_t)p)); } };
	std::unique_ptr<esp_ota_handle_t,decltype(deleter)> ota_handle((esp_ota_handle_t*)_h ,deleter);
	std::unique_ptr<char[]>ota_buf(new char[OTA_BUF_SIZE]); if(!ota_buf) return send_error("!ota_buf");
	while (1) { 
		ret = httpd_req_recv(req, ota_buf.get(), OTA_BUF_SIZE); //internal check
		if (ret == 0) break;
		if (ret < 0) {
			if (ret == HTTPD_SOCK_ERR_TIMEOUT) { 
				if(wifi_is_connected()) {
					ESP_LOGW(TAG, "Timeout");
					continue;
				}
				ESP_LOGE(TAG, "Timeout"); return 0;
			}
			return send_error("httpd_req_recv()"); // Serious Error: Abort OTA
		}
		if ((ret = esp_ota_write((esp_ota_handle_t)ota_handle.get(), ota_buf.get(), ret))) {
			return send_error("esp_ota_write()");
		}
	}
	ESP_LOGI(TAG, "%lu bytes recieved", req->content_len);
	if(!(ret = esp_ota_end((esp_ota_handle_t)ota_handle.get()))) {
		if(!(ret = esp_ota_set_boot_partition(ota_partition))) {
			int len = sprintf(buf, PART_FMT, "Success! Can reboot\n",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
			(void)ota_handle.release();
			ESP_LOGI(TAG, "%s", buf);
			return httpd_resp_send_text(req, buf, len);
		}
		sprintf(buf, PART_FMT, "esp_ota_set_boot_partition()\n",
		 ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
	} return send_error("esp_ota_end()");
	return 0;
}

static esp_err_t config_post_handler(httpd_req_t *req) {
	char content[256]; const char* _SSID = "ssid=", * _PASS = "&pass="; constexpr int max_size = sizeof(content) / 4 * 3;
	uint8_t pass_key [sizeof(::pass_key)]; uint8_t scan_key [sizeof(::scan_key)];
	size_t recv_size = req->content_len; 
	if(recv_size >= max_size) {
		return httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, NULL);
	}
    int len = httpd_req_recv(req, content, max_size);
    if (len <= 0) { // Check for timeout or error
        if (len == HTTPD_SOCK_ERR_TIMEOUT) {
            return httpd_resp_send_408(req);
        }
        return len;
    }
	content[len] = '\0';
	ESP_LOGI(TAG, "Received POST data:\n%s", content);
	char* ssid = nullptr, * pass = nullptr, * temp;
	if((temp = strstr(content, _SSID))) {
		ssid = temp + strlen_const(_SSID);
	}
	if((temp = strstr(content, _PASS))) {
		pass = temp + strlen_const(_PASS);
		*temp = '\0';
	}
	std::unique_ptr <char[]> str_pass, str_scan; //931541
	int len_pass, len_key;
	if (ssid) {
		len_pass = base32_decode(ssid, pass_key, sizeof(pass_key)); 
		//pass_key_len = len_pass;
    	if(len_pass > 0) {
			reinterpret_cast<char*&>(str_pass) = new char[sizeof(pass_key)*2];
			if(str_pass) {
				len = base32_encode(pass_key, len_pass, str_pass.get(), sizeof(pass_key)*2);
			}
			
		} else { ESP_LOGE(TAG, "Invalid Base32! %d", len_pass); }
	} else len_pass = 0;
	if (pass) {
		len_key = strtoB(pass, scan_key, sizeof(scan_key));
		//scan_key_len = len_key;
		if (len_key >= 8) {
			reinterpret_cast<char*&>(str_scan) = new char[len_key * 3 + 1]; //931571 //
			if(str_scan) {
				len = 0;
				for (size_t i = 0; i < len_key; i++) { len += sprintf(&str_scan.get()[i * 3], "%02X ", scan_key[i]); }
				str_scan.get()[len - 1] = '\0'; //933945
				//str_scankey_len = bytes_to_str_bigend(scan_key, str_scan.get(), len_key); //933999
			}
		}
	} else len_key = 0;
	char* log = content; int &i = len;
	i = sprintf(log,"PASS [%s]\n" , str_pass.get());
	i += sprintf(log+i,"SCAN_KEY [%s]\n" , str_pass.get());
	i += sprintf(log+i,"pass_key_len(base32) = %d\n", len_pass);
	i += sprintf(log+i,"scan_key_len = %d\n", len_key);
	bool save_p = len_pass >= 8, save_s = len_key >= 8;
	if(save_p || save_s) {
		if(save_p){ memcpy(::pass_key, pass_key, pass_key_len = len_pass); }
		if(save_s){ memcpy(::scan_key, scan_key, scan_key_len = len_key); }
		i += sprintf(log+i,"goto %s", save_uri);
	} else { i += sprintf(log+i,"EMPTY"); }
	ESP_LOGI(TAG, "%s", log);
    return httpd_resp_send_text(req, log, i);
}

esp_err_t http_server_init(void) {
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.stack_size = 8*1024; config.task_priority = 15; config.max_uri_handlers = 10;
	ESP_RETURN_ON_ERROR(httpd_start(&http_server, &config), TAG, "");
	httpd_uri_t uri = {
		.uri = "/update",
		.method = HTTP_GET,
		.handler = [](httpd_req_t *req) { 
			extern const char index_html_start[] asm("_binary_index_html_gz_start");
			extern const char index_html_end[] asm("_binary_index_html_gz_end");
			size_t size = index_html_end - index_html_start;  ESP_LOGD(TAG, "html size %lu", size);
			httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
			return httpd_resp_send(req, index_html_start, size);
		},
		.user_ctx = NULL
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/update/start";
	uri.method = HTTP_POST;
	uri.handler = update_post_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/config";
	uri.method = HTTP_GET;
	uri.handler = [](httpd_req_t *req) {
		extern const char config_html_start[] asm("_binary_config_html_gz_start");
		extern const char config_html_end[] asm("_binary_config_html_gz_end");
		const size_t size = config_html_end - config_html_start; ESP_LOGD(TAG, "html size %lu", size);
		httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
		return httpd_resp_send(req, config_html_start, size);
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/config";
	uri.method = HTTP_POST;
	uri.handler = config_post_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = save_uri;
	uri.method = HTTP_POST;
	uri.handler = [](httpd_req_t *req) {
		char buf[32]; esp_err_t ret = save_auth_data(); 
		int len = sprintf(buf, "%s 0x%X", save_uri, ret);
		return httpd_resp_send_text(req, buf, len, ret);
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/ota_part";
	uri.method = HTTP_GET;
	uri.handler = [](httpd_req_t *req) {
		return 0;
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/restart";
	uri.method = HTTP_GET;
	uri.handler = [](httpd_req_t *req) {
		esp_err_t ret = wifi_timer_reset(500); constexpr char str[] = "/restart";
   		return httpd_resp_send_text(req, str, strlen_const(str), ret);
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/info";
	uri.method = HTTP_GET;
	uri.handler = [](httpd_req_t *req) {
		size_t len; auto str = task_list(&len); if(!str) return -1;
		len += sprintf(str.get() + len, "Compiled: " __TIMESTAMP__);
		return httpd_resp_send_text(req, str.get(), len);
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/valid";
	uri.method = HTTP_GET;
	uri.handler = [](httpd_req_t *req) {
		revoke_ota_rollback(); const char* str = "revoke_ota_rollback()";
   		return httpd_resp_send_text(req, str, strlen_const(str));
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");

	uri.uri = "/nvs_erase_all";
	uri.method = HTTP_GET;
	uri.handler = [](httpd_req_t *req)  {
		nvsEraseAll(); const char* str = "/nvs_erase_all";
		return httpd_resp_send_text(req, str, strlen_const(str));
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");
	//
	/*
    uri.uri       = "/basic_auth";
    uri.method    = HTTP_GET;
    uri.handler   = basic_auth_get_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), TAG, "");
	*/
	return ESP_OK;
}

static esp_err_t basic_auth_get_handler(httpd_req_t *req)
{
    char *buf = NULL;
    size_t buf_len = 0;
    //basic_auth_info_t *basic_auth_info = req->user_ctx;
	
    buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
	ESP_LOGI(TAG, "%s; method %d; buf_len %lu; content_len %lu", req->uri, req->method, buf_len, req->content_len);
    if (buf_len > 1) {
        buf = (char*)calloc(1, buf_len);
        if (!buf) {
            ESP_LOGE(TAG, "No enough memory for basic authorization");
            return ESP_ERR_NO_MEM;
        }

        if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Authorization: %s", buf);
        } else {
            ESP_LOGE(TAG, "No auth value received");
        }

        //char *auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
        //if (!auth_credentials) {
         //   ESP_LOGE(TAG, "No enough memory for basic authorization credentials");
         //   free(buf);
         //   return ESP_ERR_NO_MEM;
        //}

        //if (strncmp(auth_credentials, buf, buf_len)) {
		if (0) {
            ESP_LOGE(TAG, "Not authenticated");
            httpd_resp_set_status(req, HTTPD_401);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
        } else {
            ESP_LOGI(TAG, "Authenticated!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            int rc = asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", "username");
            if (rc < 0) {
                ESP_LOGE(TAG, "asprintf() returned: %d", rc);
                //free(auth_credentials);
                return ESP_FAIL;
            }
            if (!basic_auth_resp) {
                ESP_LOGE(TAG, "No enough memory for basic authorization response");
                //free(auth_credentials);
                free(buf);
                return ESP_ERR_NO_MEM;
            }
            httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
            free(basic_auth_resp);
        }
        //free(auth_credentials);
        free(buf);
    } else {
        ESP_LOGE(TAG, "No auth header received");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
    }

    return ESP_OK;
}
