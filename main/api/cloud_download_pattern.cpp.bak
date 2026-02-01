#include "cloud_download_pattern.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include <string>
#include "ManifestDatabase.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_crt_bundle.h"

#define CLOUD_API_BASE_URL "https://tranquil.api.koiosdigital.net"
#define PATTERN_DIR "/sd/patterns"
static const char* TAG = "cloud_dl";

// --- Event handler for streaming download to file ---
static esp_err_t cloud_download_event_handler(esp_http_client_event_t* evt) {
    FILE* fp = (FILE*)evt->user_data;
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (fp && evt->data && evt->data_len > 0) {
            size_t written = fwrite(evt->data, 1, evt->data_len, fp);
            if (written != evt->data_len) {
                ESP_LOGE(TAG, "fwrite failed");
                return ESP_FAIL;
            }
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

// --- Manifest event handler for response to buffer ---
static char manifest_response_buffer[2048] = { 0 };
static int manifest_response_len = 0;
static SemaphoreHandle_t manifest_response_sem = NULL;

static esp_err_t cloud_manifest_event_handler(esp_http_client_event_t* evt) {
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (evt->user_data && evt->data && evt->data_len > 0) {
            int copy_len = evt->data_len;
            if (manifest_response_len + copy_len > (int)sizeof(manifest_response_buffer) - 1)
                copy_len = sizeof(manifest_response_buffer) - 1 - manifest_response_len;
            memcpy((char*)evt->user_data + manifest_response_len, evt->data, copy_len);
            manifest_response_len += copy_len;
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        if (manifest_response_sem) xSemaphoreGive(manifest_response_sem);
        break;
    default:
        break;
    }
    return ESP_OK;
}

cloud_download_result_t cloud_download_pattern(const char* uuid, const char* manifest_path, const char* data_path, const char* token) {
    cloud_download_result_t result = { 0 };
    char url[256];
    esp_err_t err;

    // Download manifest
    snprintf(url, sizeof(url), CLOUD_API_BASE_URL "/patterns/%s", uuid);
    FILE* manifest_fp = fopen(manifest_path, "wb");
    if (!manifest_fp) {
        result.success = false;
        result.error = strdup("Failed to open manifest file");
        return result;
    }
    esp_http_client_config_t manifest_cfg = {
        .url = url,
        .event_handler = cloud_download_event_handler,
        .user_data = manifest_fp,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t manifest_client = esp_http_client_init(&manifest_cfg);
    char auth_header[128];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", token);
    esp_http_client_set_header(manifest_client, "Authorization", auth_header);
    err = esp_http_client_perform(manifest_client);
    fclose(manifest_fp);
    esp_http_client_cleanup(manifest_client);
    if (err != ESP_OK) {
        result.success = false;
        result.error = strdup("Failed to download manifest");
        return result;
    }

    // Download pattern data
    snprintf(url, sizeof(url), CLOUD_API_BASE_URL "/patterns/%s/data", uuid);
    FILE* data_fp = fopen(data_path, "wb");
    if (!data_fp) {
        result.success = false;
        result.error = strdup("Failed to open data file");
        return result;
    }
    esp_http_client_config_t data_cfg = {
        .url = url,
        .event_handler = cloud_download_event_handler,
        .user_data = data_fp,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t data_client = esp_http_client_init(&data_cfg);
    esp_http_client_set_header(data_client, "Authorization", auth_header);
    err = esp_http_client_perform(data_client);
    fclose(data_fp);
    esp_http_client_cleanup(data_client);
    if (err != ESP_OK) {
        result.success = false;
        result.error = strdup("Failed to download pattern data");
        return result;
    }

    result.success = true;
    result.error = NULL;
    return result;
}

struct cloud_download_task_args {
    char uuid[64];
    char token[400];
    cloud_download_result_t result;
    SemaphoreHandle_t done;
};

static void cloud_download_task(void* pvParameter) {
    cloud_download_task_args* args = (cloud_download_task_args*)pvParameter;
    ESP_LOGI(TAG, "Starting pattern download for uuid: %s", args->uuid);
    // Download manifest (pattern JSON) to memory using event handler
    char url[256];
    snprintf(url, sizeof(url), CLOUD_API_BASE_URL "/patterns/%s", args->uuid);
    ESP_LOGD(TAG, "Manifest URL: %s", url);
    manifest_response_len = 0;
    memset(manifest_response_buffer, 0, sizeof(manifest_response_buffer));
    manifest_response_sem = xSemaphoreCreateBinary();
    esp_http_client_config_t manifest_cfg = {
        .url = url,
        .event_handler = cloud_manifest_event_handler,
        .user_data = manifest_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t manifest_client = esp_http_client_init(&manifest_cfg);
    char auth_header[512];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", args->token);
    esp_http_client_set_header(manifest_client, "Authorization", auth_header);
    esp_http_client_set_method(manifest_client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_perform(manifest_client);
    int status = esp_http_client_get_status_code(manifest_client);
    ESP_LOGI(TAG, "Manifest HTTP status: %d", status);
    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "Failed to download manifest: %s", esp_err_to_name(err));
        esp_http_client_cleanup(manifest_client);
        vSemaphoreDelete(manifest_response_sem);
        manifest_response_sem = NULL;
        args->result.success = false;
        args->result.error = strdup("Failed to download manifest");
        xSemaphoreGive(args->done);
        vTaskDelete(NULL);
    }
    // Wait for event handler to finish
    xSemaphoreTake(manifest_response_sem, portMAX_DELAY);
    vSemaphoreDelete(manifest_response_sem);
    manifest_response_sem = NULL;
    manifest_response_buffer[manifest_response_len > 0 ? manifest_response_len : 0] = '\0';
    ESP_LOGD(TAG, "Manifest JSON: %s", manifest_response_buffer);
    esp_http_client_cleanup(manifest_client);

    // Parse and add pattern to manifest
    cJSON* manifest_pattern_json = cJSON_Parse(manifest_response_buffer);
    if (!manifest_pattern_json) {
        ESP_LOGE(TAG, "Invalid pattern JSON");
        args->result.success = false;
        args->result.error = strdup("Invalid pattern JSON");
        xSemaphoreGive(args->done);
        vTaskDelete(NULL);
    }
    Pattern pattern = ManifestDatabase::jsonToPattern(manifest_pattern_json);
    cJSON_Delete(manifest_pattern_json);
    if (ManifestDatabase::instance().addPattern(pattern) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add pattern to manifest");
        args->result.success = false;
        args->result.error = strdup("Failed to add pattern to manifest");
        xSemaphoreGive(args->done);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Pattern manifest added to ManifestDatabase");

    // Download pattern data to file
    snprintf(url, sizeof(url), CLOUD_API_BASE_URL "/patterns/%s/data", args->uuid);
    ESP_LOGD(TAG, "Pattern data URL: %s", url);
    char data_path[128];
    snprintf(data_path, sizeof(data_path), PATTERN_DIR "/%s.thr", args->uuid);
    FILE* data_fp = fopen(data_path, "wb");
    if (!data_fp) {
        ESP_LOGE(TAG, "Failed to open data file: %s", data_path);
        args->result.success = false;
        args->result.error = strdup("Failed to open data file");
        xSemaphoreGive(args->done);
        vTaskDelete(NULL);
    }
    esp_http_client_config_t data_cfg = {
        .url = url,
        .event_handler = cloud_download_event_handler,
        .user_data = data_fp,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t data_client = esp_http_client_init(&data_cfg);
    esp_http_client_set_header(data_client, "Authorization", auth_header);
    err = esp_http_client_perform(data_client);
    ESP_LOGI(TAG, "Pattern data HTTP status: %d", esp_http_client_get_status_code(data_client));
    fclose(data_fp);
    esp_http_client_cleanup(data_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to download pattern data: %s", esp_err_to_name(err));
        args->result.success = false;
        args->result.error = strdup("Failed to download pattern data");
        xSemaphoreGive(args->done);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Pattern data saved to %s", data_path);

    args->result.success = true;
    args->result.error = NULL;
    xSemaphoreGive(args->done);
    vTaskDelete(NULL);
}

esp_err_t cloud_download_pattern_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Received pattern download request");
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Request timeout");
            httpd_resp_send_408(req);
        }
        else {
            ESP_LOGE(TAG, "Request recv error");
            httpd_resp_send_500(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    ESP_LOGD(TAG, "Request body: %s", buf);
    cJSON* json = cJSON_Parse(buf);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON body");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    const cJSON* uuid_json = cJSON_GetObjectItem(json, "uuid");
    if (!cJSON_IsString(uuid_json)) {
        ESP_LOGE(TAG, "Missing uuid in request");
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing uuid");
        return ESP_FAIL;
    }
    const char* uuid = uuid_json->valuestring;
    ESP_LOGI(TAG, "Pattern UUID: %s", uuid);

    // Get token from NVS
    nvs_handle_t nvs;
    char token[512] = { 0 };
    if (nvs_open("cloud", NVS_READONLY, &nvs) != ESP_OK ||
        nvs_get_str(nvs, "token", token, (size_t[]) { sizeof(token) }) != ESP_OK) {
        if (nvs) nvs_close(nvs);
        ESP_LOGE(TAG, "No cloud token in NVS");
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "No cloud token");
        return ESP_FAIL;
    }
    nvs_close(nvs);

    cloud_download_task_args args = { 0 };
    strncpy(args.uuid, uuid, sizeof(args.uuid) - 1);
    strncpy(args.token, token, sizeof(args.token) - 1);
    args.done = xSemaphoreCreateBinary();
    cJSON_Delete(json);

    // DISABLED for memory profiling - TODO: Re-enable after optimization
    // if (xTaskCreate(cloud_download_task, "cloud_download_task", 8192, &args, 5, NULL) != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create download task");
    //     vSemaphoreDelete(args.done);
    //     httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create download task");
    //     return ESP_FAIL;
    // }
    // // Wait for the task to finish
    // xSemaphoreTake(args.done, portMAX_DELAY);
    vSemaphoreDelete(args.done);
    ESP_LOGW(TAG, "cloud_download_task DISABLED for memory profiling");
    httpd_resp_send_err(req, HTTPD_501_METHOD_NOT_IMPLEMENTED, "Cloud download disabled");
    return ESP_FAIL;

    if (args.result.success) {
        ESP_LOGI(TAG, "Pattern download successful");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
    }
    else {
        ESP_LOGE(TAG, "Pattern download failed: %s", args.result.error ? args.result.error : "Download failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, args.result.error ? args.result.error : "Download failed");
        if (args.result.error) free(args.result.error);
        return ESP_FAIL;
    }
}
