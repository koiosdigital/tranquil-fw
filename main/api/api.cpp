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

// Pre-handler hook: CORS headers go on every response registered through
// kd_common_api_register_uri_handler (and on 404/405 via the err handlers).
static void cors_hook(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,PATCH,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type,Authorization");
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
