#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include "esp_check.h"
#include "esp_httpd_priv.h"
#include <memory>

#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //struct

#define OTA_BUF_SIZE (0x1000)
#define HTTPD_401      "401 UNAUTHORIZED"
#define PART_FMT "%s 0x%X, %s->size %lu, offset 0x%lX; content_len %u"

extern const char ota_html_start[] asm("_binary_ota_html_gz_start");
extern const char ota_html_end[] asm("_binary_ota_html_gz_end");
extern const char config_html_start[] asm("_binary_config_html_gz_start");
extern const char config_html_end[] asm("_binary_config_html_gz_end");

static const char* TAG = "HTTP";
static const char* SAVE_URL = "/save_config";

constexpr uint32_t strlen_const(const char* str) { return __builtin_strlen(str); }
__unused static esp_err_t basic_auth_get_handler(httpd_req_t *req);
httpd_handle_t http_server;

//extern int bytes_to_str_bigend(const uint8_t* src, char* dest, size_t data_size);
__weak_symbol void ota_update_start_cb() {}
__weak_symbol void ota_update_end_cb() {}
__weak_symbol void ota_rollback_revoke() { esp_ota_mark_app_valid_cancel_rollback(); }
__weak_symbol void set_main_partition() {}
__weak_symbol void restart_request() { esp_restart(); }
__weak_symbol void patch_request_cb() {}

extern uint8_t scan_key[32]; extern uint8_t pass_key[32];
extern uint8_t pass_key_len, scan_key_len;
extern uint32_t wifi_is_connected();
extern size_t strtoB(const char* src, uint8_t *dest, size_t buf_len);

extern esp_err_t auth_data_save();
extern void nvsEraseAll(const char* = nullptr);
extern int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
extern int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len);
extern std::unique_ptr<char[]> task_list(size_t* len);
extern "C" int64_t micros();
extern "C" size_t heap_caps_sprint_heap_info(void* buf, uint32_t caps = MALLOC_CAP_INTERNAL);

static esp_err_t httpd_resp_send_text(httpd_req_t *req, const char *buf, ssize_t buf_len = HTTPD_RESP_USE_STRLEN, esp_err_t ret = 0) {
	httpd_resp_set_status(req, ret == ESP_OK ? HTTPD_200 : HTTPD_500); 
    httpd_resp_set_type(req, "text/plain");
	return httpd_resp_send(req, buf, buf_len);
}

static esp_err_t update_post_handler(httpd_req_t *req) {
	int ret = -1; char buf[128]; esp_ota_handle_t _h;
	auto send_error = [&req, &ret, &buf](const char *usr_msg) {
		sprintf(buf, "%s 0x%X", usr_msg, ret);
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf); //log internal
	};
	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	if(!ota_partition) {
		return send_error("!ota_partition");
	}
	if((ret = esp_ota_begin(ota_partition, req->content_len, &_h))) {
		sprintf(buf, PART_FMT, "ota_begin()",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf); //log internal
		
	}
	ESP_LOGI(TAG, PART_FMT, "ota_begin()",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
	std::unique_ptr<char[]>ota_buf(new char[OTA_BUF_SIZE]); if(!ota_buf) return send_error("!ota_buf");
	auto deleter = [](esp_ota_handle_t* p) { 
		if(p) { ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort((esp_ota_handle_t)p)); }
		ota_update_end_cb();
	};
	std::unique_ptr<esp_ota_handle_t,decltype(deleter)> ota_handle((esp_ota_handle_t*)_h, deleter);
	ota_update_start_cb(); 
	auto started = micros();
	while (1) { 
		ret = httpd_req_recv(req, ota_buf.get(), OTA_BUF_SIZE); //internal check
		if (ret == 0) break;
		if (ret < 0) {
			if (ret == HTTPD_SOCK_ERR_TIMEOUT) { 
				if(wifi_is_connected()) {
					ESP_LOGW(TAG, "Timeout");
					vTaskDelay(1);
					continue;
				}
				ESP_LOGE(TAG, "Timeout"); 
				return 0;
			}
			return send_error("httpd_req_recv()"); // Serious Error: Abort OTA
		}
		if ((ret = esp_ota_write((esp_ota_handle_t)ota_handle.get(), ota_buf.get(), ret))) {
			return send_error("esp_ota_write()");
		}
	}
	if(!(ret = esp_ota_end((esp_ota_handle_t)ota_handle.get()))) {
		ESP_LOGD(TAG, "esp_ota_end()");
		if(!(ret = esp_ota_set_boot_partition(ota_partition))) {
			int len = sprintf(buf, PART_FMT, "Success! Can /restart\n",
			ret, ota_partition->label, ota_partition->size, ota_partition->address, req->content_len);
			auto timer = micros(); //pdTICKS_TO_MS(xTaskGetTickCount() - started),
			size_t delta = (timer - started) / 1000ul;
			len += sprintf(&buf[len],"\nThe update took %u ms\nSpeed: %u kbyte/s", delta, (size_t)req->content_len / delta);
			(void)ota_handle.release();
			ESP_LOGI(TAG, "%.*s", len, buf);
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
	std::unique_ptr <char[]> str_pass, str_scan;
	int len_pass, len_key;
	if (ssid) {
		len_pass = base32_decode(ssid, pass_key, sizeof(pass_key)); 
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
			reinterpret_cast<char*&>(str_scan) = new char[len_key * 3 + 1];
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
	bool save_p = len_pass >= 16, save_s = len_key >= 8;
	if(save_p || save_s) {
		if(save_p){ memcpy(::pass_key, pass_key, ::pass_key_len = len_pass); }
		if(save_s){ memcpy(::scan_key, scan_key, ::scan_key_len = len_key); }
		i += sprintf(log+i,"goto %s", SAVE_URL);
	} else { i += sprintf(log+i,"EMPTY"); }
	ESP_LOGI(TAG, "%s", log);
    return httpd_resp_send_text(req, log, i);
}

esp_err_t http_server_init(void) {
	httpd_uri_t handlers [] = {
		{
			.uri = "/update",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) { 
				size_t size = ota_html_end - ota_html_start;  ESP_LOGD(TAG, "html size %lu", size);
				httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
				return httpd_resp_send(req, ota_html_start, size);
			},
			.user_ctx = NULL,
		},
		{
			.uri = "/update/start",
			.method = HTTP_POST,
			.handler = update_post_handler,
		},
		{
			.uri = "/config",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				const size_t size = config_html_end - config_html_start; ESP_LOGD(TAG, "html size %lu", size);
				httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
				return httpd_resp_send(req, config_html_start, size);
			},
		},
		{
			.uri = "/config",
			.method = HTTP_POST,
			.handler = config_post_handler,
		},
		{
			.uri = SAVE_URL,
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				char buf[32]; esp_err_t ret = auth_data_save(); 
				int len = sprintf(buf, "%s 0x%X", SAVE_URL, ret);
				return httpd_resp_send_text(req, buf, len, ret);
			},
		},
		{
			.uri = "/main",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				set_main_partition(); return httpd_resp_send_text(req, req->uri);
			},
		},
		{
			.uri = "/restart",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				restart_request(); return httpd_resp_send_text(req, req->uri);
			},
		},
		{
			.uri = "/open",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				patch_request_cb(); return httpd_resp_send_text(req, req->uri);
			},
		},
		{
			.uri = "/info",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				size_t len; auto str = task_list(&len); if(!str) return -1;
				return httpd_resp_send_text(req, str.get(), len);
			},
		},
		{
			.uri = "/valid",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req) {
				ota_rollback_revoke();
   				return httpd_resp_send_text(req, "ota_rollback_revoke()");
			},
		},
		{
			.uri = "/nvs_erase",
			.method = HTTP_GET,
			.handler = [](httpd_req_t *req)  {
				nvsEraseAll();
				return httpd_resp_send_text(req, req->uri);
			},
		},
		{
			.uri = "/help",
			.method = HTTP_GET,
			.handler = [](httpd_req_t* req) {
				struct httpd_data* hd = (struct httpd_data*)req->handle;
				const auto max_handlers = hd->config.max_uri_handlers;
				const auto str_cap = max_handlers * 32; size_t len = 0;
				std::unique_ptr<char[]>buf(new char[str_cap]); if(!buf) return -1;
				for (size_t i = 0; i < max_handlers; i++) {
					httpd_uri_t *handler = hd->hd_calls[i];
					len += sprintf(&buf[len], "#%u  \"%s\"\t", i+1, handler->uri); 
					httpd_method_t method = handler->method;
					if(method == HTTP_GET) {
						len += sprintf(&buf[len], "%s\n", "HTTP_GET");
					} else if(method == HTTP_POST){
						len += sprintf(&buf[len], "%s\n", "HTTP_POST");
					} else {
						len += sprintf(&buf[len], "Method %d\n", handler->method);
					}
				}
				ESP_LOGI(TAG, "%s len = %u, strlen = %u", req->uri, len);
				ESP_LOGD(TAG, "%.*s", len, buf.get());
				return httpd_resp_send_text(req, buf.get(), len);
			}
		}
		/*{
    		.uri       = "/basic_auth";
    		.method    = HTTP_GET;
    		.handler   = basic_auth_get_handler;
		}*/
		
	};
	constexpr size_t handlers_num = sizeof(handlers) / sizeof(handlers[0]);
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.stack_size = 8*1024; 
	config.task_priority = CONFIG_LWIP_TCPIP_TASK_PRIO -1; 
	config.max_uri_handlers = handlers_num;
	ESP_RETURN_ON_ERROR(httpd_start(&http_server, &config), TAG, ""); 
	for (size_t i = 0; i < handlers_num; i++) {
		ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &handlers[i]), TAG, "");
	}
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
            if ((rc < 0) || !basic_auth_resp) {
                ESP_LOGE(TAG, "asprintf() returned: %d", rc);
                //free(auth_credentials);
                return ESP_ERR_NO_MEM;
            }
            httpd_resp_send(req, basic_auth_resp, rc);
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