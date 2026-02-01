#include <stdbool.h>
#include <stddef.h>
#include "esp_http_server.h"

// Result struct for download
typedef struct {
    bool success;
    char* error; // malloc'd if present, must be freed by caller
} cloud_download_result_t;

// Download a pattern manifest and data by UUID, saving data to the given file path.
cloud_download_result_t cloud_download_pattern(const char* uuid, const char* manifest_path, const char* data_path, const char* token);

// HTTP handler for POST /api/cloud/download_pattern
esp_err_t cloud_download_pattern_handler(httpd_req_t* req);
