#include <string.h>
//#include <freertos/FreeRTOS.h>
#include <esp_http_server.h>
//#include <freertos/task.h>
#include <esp_ota_ops.h>
//#include <esp_system.h>
#include <sys/param.h>
#include "esp_check.h"
#include <memory>

#define STAG "server"
#define OTA_BUF_SIZE (0x1000)

httpd_handle_t http_server;
esp_timer_handle_t timer_wifi;
//char ota_buf[OTA_BUF_SIZE];

static esp_err_t index_get_handler(httpd_req_t *req) {
	extern const char index_html_start[] asm("_binary_index_html_gz_start");
	extern const char index_html_end[] asm("_binary_index_html_gz_end");
	size_t length = index_html_end - index_html_start; 
	ESP_LOGD(STAG, "html size %lu", length);
	httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
	int ret = httpd_resp_send(req, index_html_start, length);
	if(ret) { ESP_LOGE(STAG, "%d", ret); }
	return ret;
}

static void send_error(httpd_req_t *req, const char* msg, int err) {
	char buf[32]; size_t length = strlen(msg);
	memcpy(buf, msg, length); sprintf(buf + length, " 0x%X", err);
	httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, buf);
}

static esp_err_t upload_post_handler(httpd_req_t *req) {
	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	if(!ota_partition) return -2;
	size_t remaining = req->content_len;
	esp_ota_handle_t ota_handle;//esp_ota_abort(ota_handle);
	esp_err_t ret = esp_ota_begin(ota_partition, remaining, &ota_handle);
	if(ret) { ESP_LOGE(STAG, "%s, ret 0x%X", ota_partition->label, ret); return ret; }
	ESP_LOGI(STAG, "Target Partition %s->size %lu, offset 0x%lx; content_len %lu", 
		ota_partition->label, ota_partition->size, ota_partition->address, remaining);
	if(remaining > ota_partition->size) return -3;
	std::unique_ptr<char[]>ota_buf(new char[OTA_BUF_SIZE]); if(!ota_buf) return 0xBAADF00D;
	while (remaining > 0) { 
		int recv_len = httpd_req_recv(req, ota_buf.get(), MIN(remaining, OTA_BUF_SIZE));
		if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
			ESP_LOGW(STAG, "Timeout");
			continue; // Timeout Error: Just retry
		} else if (recv_len <= 0) {
			send_error(req, "Protocol Error", recv_len);
			return recv_len; // Serious Error: Abort OTA
		}

		if ((ret = esp_ota_write(ota_handle, ota_buf.get(), recv_len))) {
			send_error(req, "Flash Error", recv_len);
			return ret; 
		}
		remaining -= recv_len;
	}
	ret = esp_ota_end(ota_handle);
	if(ret == ESP_OK) {
		if((ret = esp_ota_set_boot_partition(ota_partition)) == ESP_OK) {
			httpd_resp_sendstr(req, "Success, can reboot!\n");
			ESP_LOGI(STAG, "Success, can reboot!\n");
			return ESP_OK;
		}
		send_error(req, "Activation", ret);
	} else { send_error(req, "Validation", ret); }
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

	uri.uri = "/restart";
	uri.method = HTTP_POST;
	uri.handler = [](httpd_req *r){ esp_timer_start(timer_wifi, 1000*100);return 0;};
	ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &uri), STAG, "");
	return ESP_OK;
}
