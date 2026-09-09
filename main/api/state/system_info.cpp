// SystemInfo builder implementation
#include "system_info.h"
#include "homing_service.h"
#include "SandTablePlayer.h"

#include <esp_app_desc.h>
#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <kd_common.h>
#include <kd_wifi.h>

#include <cstdio>
#include <cstring>

namespace system_info {

namespace {

// A tiny concrete subclass just to reach HandlerBase::serialize() (protected).
struct Packer : HandlerBase {
    HandleResult handle(const Kd__V1__TranquilMessage*, const MessageContext&, ResponseMessage&) override {
        return HandleResult::notHandled();
    }
    bool canHandle(Kd__V1__TranquilMessage__MessageCase) const override { return false; }
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override { return {}; }
    static ResponseMessage pack(const Kd__V1__TranquilMessage* msg) { return serialize(msg); }
};

void fill_ip(char* out, size_t out_len) {
    out[0] = '\0';
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) netif = esp_netif_get_default_netif();
    if (!netif) return;
    esp_netif_ip_info_t ip = {};
    if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        snprintf(out, out_len, IPSTR, IP2STR(&ip.ip));
    }
}

int32_t read_rssi() {
    wifi_ap_record_t ap = {};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

}  // namespace

void snapshot(Snapshot& out) {
    static char model[] = "tranquil";
    const esp_app_desc_t* app_desc = esp_app_get_description();
    const char* device_id = kd_common_get_device_name();
    const char* hostname = kd_common_get_wifi_hostname();

    out.firmware_version = app_desc->version;
    out.hardware_model = model;
    out.device_id = device_id ? device_id : "";
    out.hostname = hostname ? hostname : "";
    out.is_homed = SandTablePlayer::isHomed();
    out.is_homing = homing_service::is_homing();
    out.free_heap = esp_get_free_heap_size();
    out.free_psram = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    out.uptime_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000LL);
    out.wifi_rssi = read_rssi();
    fill_ip(out.ip_address, sizeof(out.ip_address));
}

ResponseMessage build_message() {
    Snapshot s;
    snapshot(s);

    Kd__V1__SystemInfo info = KD__V1__SYSTEM_INFO__INIT;
    info.firmware_version = const_cast<char*>(s.firmware_version);
    info.hardware_model = const_cast<char*>(s.hardware_model);
    info.device_id = const_cast<char*>(s.device_id);
    info.hostname = const_cast<char*>(s.hostname);
    info.is_homed = s.is_homed;
    info.is_homing = s.is_homing;
    info.free_heap = s.free_heap;
    info.free_psram = s.free_psram;
    info.uptime_s = s.uptime_s;
    info.wifi_rssi = s.wifi_rssi;
    info.ip_address = s.ip_address;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SYSTEM_INFO;
    msg.system_info = &info;
    return Packer::pack(&msg);
}

}  // namespace system_info
