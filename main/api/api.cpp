#include "api.h"

#include "mdns.h"
#include "esp_http_server.h"
#include "kd_common.h"
#include "cJSON.h"
#include <esp_app_desc.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cloud_api.h"
#include "patterns_api.h"
#include "playlists_api.h"
#include "api_player.h"
#include "led_api.h"

#include "static_files.h"

/* Empty handle to esp_http_server */
httpd_handle_t kd_server = NULL;

httpd_handle_t get_httpd_handle() {
    return kd_server;
}

void init_mdns() {
    mdns_init();
    const char* hostname = kd_common_get_wifi_hostname();
    mdns_hostname_set(hostname);

    //esp_app_desc
    const esp_app_desc_t* app_desc = esp_app_get_description();

    mdns_txt_item_t serviceTxtData[3] = {
        {"model", FIRMWARE_VARIANT},
        {"type", "tranquil"},
        { "version", app_desc->version }
    };

    ESP_ERROR_CHECK(mdns_service_add(NULL, "_koiosdigital", "_tcp", 80, serviceTxtData, 3));
}

void server_init() {
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 60;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_start(&kd_server, &config);
}

esp_err_t root_handler(httpd_req_t* req) {
    const char* response = "Welcome to the KD API!";
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

esp_err_t about_handler(httpd_req_t* req) {
    const esp_app_desc_t* app_desc = esp_app_get_description();

    // Create JSON response
    cJSON* json = cJSON_CreateObject();
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON* model = cJSON_CreateString(FIRMWARE_VARIANT);
    cJSON* type = cJSON_CreateString("tranquil");
    cJSON* version = cJSON_CreateString(app_desc->version);

    cJSON_AddItemToObject(json, "model", model);
    cJSON_AddItemToObject(json, "type", type);
    cJSON_AddItemToObject(json, "version", version);

    char* json_string = cJSON_Print(json);
    if (json_string == NULL) {
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);

    return ESP_OK;
}

esp_err_t system_config_get_handler(httpd_req_t* req) {
    char* wifi_hostname = kd_common_get_wifi_hostname();

    // Create JSON response
    cJSON* json = cJSON_CreateObject();
    if (json == NULL) {
        if (wifi_hostname) free(wifi_hostname);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON_AddBoolToObject(json, "auto_timezone", kd_common_get_auto_timezone());
    cJSON_AddStringToObject(json, "timezone", kd_common_get_timezone());
    cJSON_AddStringToObject(json, "ntp_server", kd_common_get_ntp_server());
    cJSON_AddStringToObject(json, "wifi_hostname", wifi_hostname ? wifi_hostname : "");

    char* json_string = cJSON_Print(json);
    if (json_string == NULL) {
        cJSON_Delete(json);
        if (wifi_hostname) free(wifi_hostname);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);
    if (wifi_hostname) free(wifi_hostname);

    return ESP_OK;
}

esp_err_t system_config_post_handler(httpd_req_t* req) {
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        else {
            httpd_resp_send_500(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';

    cJSON* json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
        return ESP_FAIL;
    }

    cJSON* auto_timezone_json = cJSON_GetObjectItem(json, "auto_timezone");
    cJSON* timezone_json = cJSON_GetObjectItem(json, "timezone");
    cJSON* ntp_server_json = cJSON_GetObjectItem(json, "ntp_server");
    cJSON* wifi_hostname_json = cJSON_GetObjectItem(json, "wifi_hostname");

    if (cJSON_IsBool(auto_timezone_json)) {
        kd_common_set_auto_timezone(cJSON_IsTrue(auto_timezone_json));
    }

    if (cJSON_IsString(timezone_json)) {
        const char* tz_str = cJSON_GetStringValue(timezone_json);
        if (tz_str && strlen(tz_str) < 64) {
            kd_common_set_timezone(tz_str);
        }
    }

    if (cJSON_IsString(ntp_server_json)) {
        const char* ntp_str = cJSON_GetStringValue(ntp_server_json);
        if (ntp_str && strlen(ntp_str) < 64) {
            kd_common_set_ntp_server(ntp_str);
        }
    }

    if (cJSON_IsString(wifi_hostname_json)) {
        const char* hostname_str = cJSON_GetStringValue(wifi_hostname_json);
        if (hostname_str && strlen(hostname_str) > 0 && strlen(hostname_str) <= 63) {
            kd_common_set_wifi_hostname(hostname_str);
        }
    }

    cJSON_Delete(json);

    cJSON* response_json = cJSON_CreateObject();
    cJSON_AddStringToObject(response_json, "status", "success");

    char* response_string = cJSON_Print(response_json);
    if (response_string == NULL) {
        cJSON_Delete(response_json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));

    free(response_string);
    cJSON_Delete(response_json);

    return ESP_OK;
}

esp_err_t time_zones_handler(httpd_req_t* req) {
    const kd_common_tz_entry_t* zones = kd_common_get_all_timezones();
    int total_zones = kd_common_get_timezone_count();

    // Set content type and start chunked response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Transfer-Encoding", "chunked");

    // Send opening bracket for JSON array
    httpd_resp_send_chunk(req, "[", 1);

    // Process zones in chunks to avoid memory issues
    const int CHUNK_SIZE = 20;  // Process 20 zones at a time
    bool first_zone = true;

    for (int chunk_start = 0; chunk_start < total_zones; chunk_start += CHUNK_SIZE) {
        int chunk_end = chunk_start + CHUNK_SIZE;
        if (chunk_end > total_zones) {
            chunk_end = total_zones;
        }

        // Create JSON array for this chunk
        cJSON* chunk_array = cJSON_CreateArray();
        if (chunk_array == NULL) {
            httpd_resp_send_chunk(req, NULL, 0);  // End chunked response
            return ESP_FAIL;
        }

        // Add zones to this chunk
        for (int i = chunk_start; i < chunk_end; i++) {
            cJSON* zone_obj = cJSON_CreateObject();
            if (zone_obj == NULL) continue;

            cJSON_AddStringToObject(zone_obj, "name", zones[i].name);
            cJSON_AddStringToObject(zone_obj, "rule", zones[i].rule);
            cJSON_AddItemToArray(chunk_array, zone_obj);
        }

        // Convert chunk to string
        char* chunk_string = cJSON_PrintUnformatted(chunk_array);
        if (chunk_string == NULL) {
            cJSON_Delete(chunk_array);
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_FAIL;
        }

        // Remove the outer brackets from the chunk JSON array
        // (we'll manually manage the main array brackets)
        size_t chunk_len = strlen(chunk_string);
        if (chunk_len > 2) {  // Remove '[' and ']'
            chunk_string[chunk_len - 1] = '\0';  // Remove ']'
            char* content = chunk_string + 1;     // Skip '['

            // Add comma separator if not first zone
            if (!first_zone) {
                httpd_resp_send_chunk(req, ",", 1);
            }

            // Send the chunk content
            httpd_resp_send_chunk(req, content, strlen(content));
            first_zone = false;
        }

        free(chunk_string);
        cJSON_Delete(chunk_array);

        // Small delay to prevent overwhelming the system
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Send closing bracket for JSON array
    httpd_resp_send_chunk(req, "]", 1);

    // End chunked response
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

// Static file handler function
static esp_err_t static_file_handler(httpd_req_t* req) {
    const static_files::file* f = reinterpret_cast<const static_files::file*>(req->user_ctx);
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Set appropriate headers
    httpd_resp_set_type(req, f->type);
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    // Add caching headers for static assets (except HTML)
    if (strcmp(f->type, "text/html") != 0) {
        httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000"); // 1 year
    }
    else {
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache"); // Don't cache HTML
        httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
        httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
        httpd_resp_set_hdr(req, "X-XSS-Protection", "1; mode=block");
    }

    httpd_resp_send(req, reinterpret_cast<const char*>(f->contents), f->size);
    return ESP_OK;
}

// Wildcard OPTIONS handler for CORS preflight and API discoverability
static esp_err_t wildcard_options_handler(httpd_req_t* req) {
    // Allow all origins (adjust as needed for security)
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,PATCH,DELETE,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type,Authorization");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Credentials", "true");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, NULL, 0); // No body needed for OPTIONS
    return ESP_OK;
}

void api_init() {
    init_mdns();
    server_init();

    httpd_handle_t server = get_httpd_handle();

    httpd_uri_t about_uri = {
        .uri = "/api/about",
        .method = HTTP_GET,
        .handler = about_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &about_uri);

    httpd_uri_t system_config_get_uri = {
        .uri = "/api/system/config",
        .method = HTTP_GET,
        .handler = system_config_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &system_config_get_uri);

    httpd_uri_t system_config_post_uri = {
        .uri = "/api/system/config",
        .method = HTTP_POST,
        .handler = system_config_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &system_config_post_uri);

    httpd_uri_t time_zones_uri = {
        .uri = "/api/time/zonedb",
        .method = HTTP_GET,
        .handler = time_zones_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &time_zones_uri);

    // Register wildcard OPTIONS handler for all API routes
    static httpd_uri_t options_uri = {
        .uri = "/api/*",
        .method = HTTP_OPTIONS,
        .handler = wildcard_options_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &options_uri);

    cloud_api_register_handlers(server);
    patterns_api_register_handlers(server);
    playlists_api_register_handlers(server);
    api_player_register_endpoints(server);
    led_api_register_handlers(server);

    // Create an array of httpd_uri_t to keep them alive after the loop
    static httpd_uri_t static_file_uris[static_files::num_of_files + 1]; // +1 for root '/' handler

    // Register static files
    for (int i = 0; i < static_files::num_of_files; i++) {
        const static_files::file& f = static_files::files[i];

        static_file_uris[i] = {
            .uri = f.path,
            .method = HTTP_GET,
            .handler = static_file_handler,
            .user_ctx = (void*)&static_files::files[i]
        };

        httpd_register_uri_handler(server, &static_file_uris[i]);
    }

    // Find index.html file to serve at root '/'
    const static_files::file* index_file = nullptr;
    for (int i = 0; i < static_files::num_of_files; i++) {
        if (strcmp(static_files::files[i].path, "/index.html") == 0) {
            index_file = &static_files::files[i];
            break;
        }
    }

    if (index_file) {
        // Create and register root URI handler '/' to serve index.html
        static_file_uris[static_files::num_of_files] = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = static_file_handler,
            .user_ctx = (void*)index_file
        };
        httpd_register_uri_handler(server, &static_file_uris[static_files::num_of_files]);
    }
    else {
        // Fallback to simple welcome message if no index.html found
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
    }
}