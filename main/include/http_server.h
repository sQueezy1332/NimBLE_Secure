#include <string.h>
//#include <freertos/FreeRTOS.h>
#include <esp_http_server.h>
//#include <freertos/task.h>
#include <esp_ota_ops.h>
//#include <esp_system.h>
#include <sys/param.h>
#include "esp_check.h"
#include <memory>

#define STAG "HTTP"
#define OTA_BUF_SIZE (0x1000)
#define HTTPD_401      "401 UNAUTHORIZED"

__unused static esp_err_t basic_auth_get_handler(httpd_req_t *req);
httpd_handle_t http_server;
esp_timer_handle_t timer_wifi;
extern byte scan_key[32]; extern byte passkey[32];
extern byte scan_key_len, passkey_len;
extern int wifi_is_connected();
extern void bytes_to_str_bigend(char* ptr, cbyte* buf, size_t data_size);
extern void strtoB(const char* ptr, byte *buf, size_t & data_size, size_t  buf_len = sizeof(scan_key));
extern void set_main_part();
//extern String get_task_list();
extern esp_err_t save_auth_data();
extern void nvsErase(const char* = nullptr);
extern int base32_decode(const char* encoded, uint8_t* result, size_t buf_len);
extern int base32_encode(const uint8_t *data, size_t length, char *result, size_t encode_len);

static esp_err_t config_post_handler(httpd_req_t *req) {
	char content[256];
	size_t recv_size = req->content_len;
	if(recv_size >= sizeof(content)) {
		httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, NULL);
	}
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) { // Check for timeout or error
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ret;
    }
    content[ret] = '\0';
	ESP_LOGI(STAG, "Received POST data:\n%s", content);
	httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
    return httpd_resp_send(req, content, ret);
	return ESP_OK;
}

static esp_err_t upload_post_handler(httpd_req_t *req) {
	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	if(!ota_partition) return 0xBAADF00D;
	size_t remaining = req->content_len;
	auto deleter = [](esp_ota_handle_t* p) { if(p) { ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort((uint32_t)p)); } };
	std::unique_ptr<esp_ota_handle_t,decltype(deleter)> ota_handle(NULL ,deleter);
	int ret = esp_ota_begin(ota_partition, remaining, reinterpret_cast<esp_ota_handle_t*>(&ota_handle));
	char buf[64+16];
	if(ret) {
		sprintf(buf, "ota_begin(), 0x%X, %s->size %lu, offset 0x%lX; content_len(%u)",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, remaining);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
		ESP_LOGE(STAG, "%s", buf);
		return ret;
	}
	ESP_LOGI(STAG, "ota_begin(), 0x%X, %s->size %lu, offset 0x%lX; content_len(%u)",
			 ret, ota_partition->label, ota_partition->size, ota_partition->address, remaining);
	std::unique_ptr<char[]>ota_buf(new char[OTA_BUF_SIZE]); if(!ota_buf){ return 0xBAADF00D; }
	while (1) { 
		ret = httpd_req_recv(req, ota_buf.get(), OTA_BUF_SIZE); //internal check
		if (ret == 0) break;
		if (ret < 0) {
			if (ret == HTTPD_SOCK_ERR_TIMEOUT) { 
				if(wifi_is_connected()) {
					ESP_LOGW(STAG, "Timeout");
					continue;
				}
				ESP_LOGE(STAG, "Timeout");
				return ret;
			}
			sprintf(buf, "httpd_req_recv() err %d", ret);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
			return ret; // Serious Error: Abort OTA
		};
		if ((ret = esp_ota_write((esp_ota_handle_t)ota_handle.get(), ota_buf.get(), ret))) {
			sprintf(buf, "esp_ota_write() 0x%X", ret);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
			return ret;
		};
	}
	ESP_LOGI(STAG, "%lu bytes recieved", req->content_len);
	if(!(ret = esp_ota_end((esp_ota_handle_t)ota_handle.get()))) {
		if(!(ret = esp_ota_set_boot_partition(ota_partition))) {
			static const char str[] = "Success! Can reboot";
			ret = httpd_resp_send(req, str, sizeof(str)-1);
			ESP_LOGI(STAG, "%s 0x%X", str, ret);
			*reinterpret_cast<esp_ota_handle_t*>(&ota_handle) = 0;
			return ret;
		}
		sprintf(buf, "esp_ota_set_boot_partition() 0x%X", ret);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
	} else { 
		sprintf(buf, "esp_ota_end() 0x%X", ret);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
	}
	ESP_LOGE(STAG, "%s", buf);
	return ret;
}

static esp_err_t index_get_handler(httpd_req_t *req) {
	extern const char index_html_start[] asm("_binary_index_html_gz_start");
	extern const char index_html_end[] asm("_binary_index_html_gz_end");
	size_t length = index_html_end - index_html_start; 
	ESP_LOGD(STAG, "html size %lu", length);
	httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
	int ret = httpd_resp_send(req, index_html_start, length);
	if(ret) { ESP_LOGE(STAG, "0x%X", ret); }
	return ret;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
	extern const char config_html_start[] asm("_binary_config_html_gz_start");
	extern const char config_html_end[] asm("_binary_config_html_gz_end");
	const size_t length = config_html_end - config_html_start;
	ESP_LOGD("config.html","%lu  start: %p end: %p", length, config_html_start, config_html_end);
	httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
	int ret = httpd_resp_send(req, config_html_start, length);
	if(ret) { ESP_LOGE(STAG, "0x%X", ret); } 
	return ret;
}

static esp_err_t reset_handler(httpd_req_t *req) {
	ESP_LOGI(STAG, "%s; method %d; content_len %lu", req->uri, req->method, req->content_len);
	static const char content[] = "esp_restart()";
	esp_err_t ret = esp_timer_start(timer_wifi, 1000*100);
	httpd_resp_set_status(req, !ret ? HTTPD_200 : HTTPD_500);
    httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
   	httpd_resp_send(req, content, sizeof(content) - 1);
	return ret;
}

static esp_err_t valid_handler(httpd_req_t *req) {
	ESP_LOGI(STAG, "%s; method %d; content_len %lu", req->uri, req->method, req->content_len);
	static const char content[] = "esp_ota_mark_app_valid_cancel_rollback()";
	esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
	httpd_resp_set_status(req, !ret ? HTTPD_200 : HTTPD_500); 
    httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
   	httpd_resp_send(req, content, sizeof(content) - 1);
	return ret;
}

esp_err_t http_server_init(void) {
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	ESP_RETURN_ON_ERROR(httpd_start(&http_server, &config), STAG, "");

	httpd_uri_t uri = {
		.uri = "/update",
		.method = HTTP_GET,
		.handler = index_get_handler,
		.user_ctx = NULL
	};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");

	uri.uri = "/update/start";
	uri.method = HTTP_POST;
	uri.handler = upload_post_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");
/*
	uri.uri = "/restart";
	uri.method = HTTP_POST;
	uri.handler = reset_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");
*/
	uri.uri = "/restart";
	uri.method = HTTP_GET;
	uri.handler = reset_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");

	uri.uri = "/config";
	uri.method = HTTP_GET;
	uri.handler = config_get_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");

	uri.uri = "/config";
	uri.method = HTTP_POST;
	uri.handler = config_post_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");

	uri.uri = "/valid";
	uri.method = HTTP_GET;
	uri.handler = valid_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");
	/*
    uri.uri       = "/basic_auth";
    uri.method    = HTTP_GET;
    uri.handler   = basic_auth_get_handler;
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");
	*/
	return ESP_OK;
}

static esp_err_t basic_auth_get_handler(httpd_req_t *req)
{
    char *buf = NULL;
    size_t buf_len = 0;
    //basic_auth_info_t *basic_auth_info = req->user_ctx;
	
    buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
	ESP_LOGI(STAG, "%s; method %d; buf_len %lu; content_len %lu", req->uri, req->method, buf_len, req->content_len);
    if (buf_len > 1) {
        buf = (char*)calloc(1, buf_len);
        if (!buf) {
            ESP_LOGE(STAG, "No enough memory for basic authorization");
            return ESP_ERR_NO_MEM;
        }

        if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) {
            ESP_LOGI(STAG, "Found header => Authorization: %s", buf);
        } else {
            ESP_LOGE(STAG, "No auth value received");
        }

        //char *auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
        //if (!auth_credentials) {
         //   ESP_LOGE(STAG, "No enough memory for basic authorization credentials");
         //   free(buf);
         //   return ESP_ERR_NO_MEM;
        //}

        //if (strncmp(auth_credentials, buf, buf_len)) {
		if (0) {
            ESP_LOGE(STAG, "Not authenticated");
            httpd_resp_set_status(req, HTTPD_401);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
        } else {
            ESP_LOGI(STAG, "Authenticated!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            int rc = asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", "username");
            if (rc < 0) {
                ESP_LOGE(STAG, "asprintf() returned: %d", rc);
                //free(auth_credentials);
                return ESP_FAIL;
            }
            if (!basic_auth_resp) {
                ESP_LOGE(STAG, "No enough memory for basic authorization response");
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
        ESP_LOGE(STAG, "No auth header received");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
    }

    return ESP_OK;
}
