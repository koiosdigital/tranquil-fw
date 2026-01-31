#include "esp_http_client.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "esp_crt_bundle.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define MAX_HTTP_OUTPUT_BUFFER 512
#define CLOUD_API_LOGIN_URL "https://tranquil.api.koiosdigital.net/auth"
#define NVS_NAMESPACE "cloud"
#define NVS_KEY_TOKEN "token"
static const char* TAG = "cloud_login";

// Buffer for HTTP response
static char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = { 0 };

// Result struct for login
typedef struct {
    bool success;
    char* token; // malloc'd if present, must be freed by caller
    char* error; // malloc'd if present, must be freed by caller
} cloud_login_result_t;

static esp_err_t cloud_http_event_handler(esp_http_client_event_t* evt) {
    static int output_len = 0;
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client)) {
            if (evt->user_data) {
                memcpy((char*)evt->user_data + output_len, evt->data, evt->data_len);
            }
            output_len += evt->data_len;
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        output_len = 0;
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        output_len = 0;
        break;
    default:
        break;
    }
    return ESP_OK;
}

void cloud_login_clear_response_buffer() {
    memset(local_response_buffer, 0, sizeof(local_response_buffer));
}

char* cloud_login_get_response_buffer() {
    return local_response_buffer;
}

cloud_login_result_t cloud_login_perform(const char* email, const char* password) {
    cloud_login_result_t result = { 0 };
    cloud_login_clear_response_buffer();
    // Prepare JSON payload
    cJSON* payload = cJSON_CreateObject();
    cJSON_AddItemToObject(payload, "email", cJSON_CreateString(email));
    cJSON_AddItemToObject(payload, "password", cJSON_CreateString(password));
    char* payload_str = cJSON_PrintUnformatted(payload);
    cJSON_Delete(payload);

    ESP_LOGI(TAG, "Logging in with payload: %s", payload_str);

    esp_http_client_config_t config = {
        .url = CLOUD_API_LOGIN_URL,
        .event_handler = cloud_http_event_handler,
        .user_data = local_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, payload_str, strlen(payload_str));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    if (err == ESP_OK && status == 200) {
        cJSON* resp_json = cJSON_Parse(local_response_buffer);
        const cJSON* token = resp_json ? cJSON_GetObjectItem(resp_json, "token") : NULL;
        if (token && cJSON_IsString(token)) {
            // Store token in NVS
            nvs_handle_t nvs;
            if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_set_str(nvs, NVS_KEY_TOKEN, token->valuestring);
                nvs_commit(nvs);
                nvs_close(nvs);
            }
            result.success = true;
            result.token = strdup(token->valuestring);
        }
        else {
            result.success = false;
            result.error = strdup("No token in response");
        }
        if (resp_json) cJSON_Delete(resp_json);
    }
    else {
        result.success = false;
        char errbuf[64];
        snprintf(errbuf, sizeof(errbuf), "Cloud login failed, status: %d", status);
        result.error = strdup(errbuf);
    }
    esp_http_client_cleanup(client);
    free(payload_str);
    return result;
}

// Structure for passing args and result to the login task
struct cloud_login_task_args {
    char email[128];
    char password[128];
    cloud_login_result_t result;
    SemaphoreHandle_t done;
};

static void cloud_login_task(void* pvParameter) {
    cloud_login_task_args* args = (cloud_login_task_args*)pvParameter;
    args->result = cloud_login_perform(args->email, args->password);
    xSemaphoreGive(args->done);
    vTaskDelete(NULL);
}

esp_err_t cloud_login_handler(httpd_req_t* req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        else {
            httpd_resp_send_500(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON* json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    const cJSON* email = cJSON_GetObjectItem(json, "email");
    const cJSON* password = cJSON_GetObjectItem(json, "password");
    if (!cJSON_IsString(email) || !cJSON_IsString(password)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing email or password");
        return ESP_FAIL;
    }

    cloud_login_task_args args = { 0 };
    strncpy(args.email, email->valuestring, sizeof(args.email) - 1);
    strncpy(args.password, password->valuestring, sizeof(args.password) - 1);
    args.done = xSemaphoreCreateBinary();
    cJSON_Delete(json);

    // DISABLED for memory profiling - TODO: Re-enable after optimization
    // if (xTaskCreate(cloud_login_task, "cloud_login_task", 6144, &args, 5, NULL) != pdPASS) {
    //     vSemaphoreDelete(args.done);
    //     httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create login task");
    //     return ESP_FAIL;
    // }
    // // Wait for the task to finish
    // xSemaphoreTake(args.done, portMAX_DELAY);
    vSemaphoreDelete(args.done);
    ESP_LOGW(TAG, "cloud_login_task DISABLED for memory profiling");
    httpd_resp_send_err(req, HTTPD_501_METHOD_NOT_IMPLEMENTED, "Cloud login disabled");
    return ESP_FAIL;

    if (args.result.success) {
        cJSON* resp = cJSON_CreateObject();
        cJSON_AddStringToObject(resp, "token", args.result.token);
        char* resp_str = cJSON_PrintUnformatted(resp);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp_str, strlen(resp_str));
        cJSON_Delete(resp);
        free(resp_str);
        free(args.result.token);
        return ESP_OK;
    }
    else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, args.result.error ? args.result.error : "Unknown error");
        if (args.result.error) free(args.result.error);
        return ESP_FAIL;
    }
}
