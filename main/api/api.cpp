#include "api.h"

#include "kd_common.h"
#include "kd_api.h"

#include "patterns_api.h"
#include "playlists_api.h"
#include "api_player.h"
#include "license_api.h"
#include "system_api.h"
#include "config_api.h"
#include "schedule_api.h"
#include "websocket_server.h"
#include "message_dispatcher.h"
#include "player_state_broadcaster.h"
#include "schedule_manager.h"

#include "static_files.h"

// Pre-handler hook: CORS headers go on every response registered through
// kd_common_api_register_uri_handler (and on 404/405 via the err handlers).
static void cors_hook(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin",  "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,PATCH,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type,Authorization");
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

static void register_tranquil_handlers(httpd_handle_t server) {
    // Register app-specific handlers.
    // NOTE: the LED API (kd_pixdriver) is registered from main.cpp when LEDs
    // are enabled — do not also attach it here or every URI double-registers.
    patterns_api_register_handlers(server);
    playlists_api_register_handlers(server);
    api_player_register_endpoints(server);
    license_api_register_handlers(server);
    system_api_register_handlers(server);
    config_api_register_handlers(server);
    schedule_api_register_handlers(server);

    // Initialize websocket server
    websocket_server_init(server);

    // Create an array of httpd_uri_t to keep them alive after the loop
    static httpd_uri_t static_file_uris[static_files::num_of_files + 1]; // +1 for root '/' override

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
        // Override root URI handler '/' to serve index.html instead of welcome message
        static_file_uris[static_files::num_of_files] = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = static_file_handler,
            .user_ctx = (void*)index_file
        };
        httpd_register_uri_handler(server, &static_file_uris[static_files::num_of_files]);
    }
}

void tranquil_api_init() {
    // Set device info before mDNS
    kd_common_set_device_info(FIRMWARE_VARIANT, "tranquil");

    // CORS on every wrapped route (kd_common runs this before each handler)
    kd_common_api_set_pre_handler(cors_hook);

    // Initialize unified message dispatcher with standard handlers
    dispatcher_register_standard_handlers();

    // Initialize player state broadcaster for rate-limited state updates
    PlayerStateBroadcaster::instance().init();

    // Initialize schedule manager for scheduled actions
    ScheduleManager::instance().init();

    // Register handler callback - will be called when httpd starts (on WiFi connect)
    kd_common_api_register_handlers(register_tranquil_handlers);
}
