
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "http_parser.h"
#include "sdkconfig.h"
#include <string.h>
#include "WEB_FILES.h"

static esp_err_t index_handler(httpd_req_t *req){
	httpd_resp_send(req, (char*)WEB_FILES_index_html, WEB_FILES_index_html_len);
	return ESP_OK;
}

static esp_err_t script_handler(httpd_req_t *req){
	httpd_resp_send(req, (char*)WEB_FILES_script_js, WEB_FILES_script_js_len);
	return ESP_OK;
}

int start_server(){
	ESP_LOGI(CONFIG_TAG_WIFI_WEB_MANAGER, "Starting WiFi web manager.");
	
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	esp_err_t httpd_error = httpd_start(&server, &config);
	if(httpd_error!=ESP_OK){
		ESP_LOGE(CONFIG_TAG_WIFI_WEB_MANAGER, "Failed creating a HTTP server!");
		ESP_ERROR_CHECK(httpd_error);
	}

	httpd_uri_t index_html_uri = {
		.uri = "/",
		.method = HTTP_GET,
		.handler = index_handler,
		.user_ctx = NULL,
	};

	httpd_uri_t script_js_uri = {
		.uri = "/script.js",
		.method = HTTP_GET,
		.handler = script_handler,
		.user_ctx = NULL,
	};

	httpd_register_uri_handler(server, &index_html_uri);	
	httpd_register_uri_handler(server, &script_js_uri);

	return ESP_OK;
}
