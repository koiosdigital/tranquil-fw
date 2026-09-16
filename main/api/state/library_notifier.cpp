// Library / state change notifier implementation
#include "library_notifier.h"

#include "ops/library_ops.h"
#include "ops/led_ops.h"
#include "schedule_manager.h"
#include "led_state.h"
#include "system_info.h"
#include "response_router.h"
#include "websocket_server.h"
#include "sockets.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <atomic>
#include <vector>

static const char* TAG = "library_notifier";

namespace {

// Debounce window: changes arriving within this many ms are merged into one
// push. Long enough to swallow a bulk import burst, short enough that a
// single edit still feels immediate.
constexpr TickType_t kDebounceTicks = pdMS_TO_TICKS(400);

EventGroupHandle_t s_events = nullptr;
TaskHandle_t s_task = nullptr;

// Requester to leave out of the next schedule push (ARCH-B18). Consumed by
// the push; if two sets race within one debounce window the later setter is
// skipped (it holds the newest schedule) and the earlier one is pushed the
// newer state, which is what it needs.
std::atomic<int> s_sched_skip_fd{ -1 };
std::atomic<bool> s_sched_skip_cloud{ false };

constexpr EventBits_t bit(library_notifier::Event ev) {
    return static_cast<EventBits_t>(1u) << static_cast<unsigned>(ev);
}

constexpr EventBits_t kAllBits =
    (static_cast<EventBits_t>(1u) << static_cast<unsigned>(library_notifier::Event::Count)) - 1;

// Fan a packed message out to LAN clients and (when the session is up) the
// cloud. The cloud copy is deliberately unconditional on `to_cloud`: every
// snapshot here is something the cloud caches for off-LAN control.
void publish(const ResponseMessage& msg, bool to_local, bool to_cloud) {
    if (!msg.valid()) return;
    if (to_local) ResponseRouter::instance().broadcastToLocal(msg);
    if (to_cloud) ResponseRouter::instance().sendToCloud(msg);
}

void push_patterns(bool to_local, bool to_cloud) {
    ResponseMessage msg = library_ops::build_pattern_list_message();
    publish(msg, to_local, to_cloud);
}

void push_playlists(bool to_local, bool to_cloud) {
    ResponseMessage msg = library_ops::build_playlist_list_message();
    publish(msg, to_local, to_cloud);
}

void push_led(bool to_local, bool to_cloud) {
    ResponseMessage msg = led_state::build_config_message();
    publish(msg, to_local, to_cloud);
}

// Static per firmware: only ever pushed as part of the cloud snapshot.
void push_led_effects(bool to_local, bool to_cloud) {
    ResponseMessage msg = led_ops::build_effects_message();
    publish(msg, to_local, to_cloud);
}

void push_schedule(bool to_local, bool to_cloud) {
    const int skip_fd = s_sched_skip_fd.exchange(-1);
    const bool skip_cloud = s_sched_skip_cloud.exchange(false);

    ResponseMessage msg = ScheduleManager::instance().buildScheduleMessage();
    if (!msg.valid()) return;
    if (to_local) websocket_broadcast_except(msg.data(), msg.len(), skip_fd);
    if (to_cloud && !skip_cloud) ResponseRouter::instance().sendToCloud(msg);
}

void push_system_info(bool to_local, bool to_cloud) {
    ResponseMessage msg = system_info::build_message();
    publish(msg, to_local, to_cloud);
}

void worker(void*) {
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(s_events, kAllBits, pdTRUE, pdFALSE, portMAX_DELAY);
        if (!bits) continue;

        // Debounce: keep absorbing events for a short window, then act once.
        for (;;) {
            EventBits_t more = xEventGroupWaitBits(s_events, kAllBits, pdTRUE, pdFALSE, kDebounceTicks);
            if (!more) break;
            bits |= more;
        }

        const bool snapshot = bits & bit(library_notifier::Event::CloudSnapshot);
        const bool cloud_up = cloud_sockets_is_ready();

        if (snapshot && cloud_up) {
            // Cloud-only refresh of everything the Worker caches. LAN clients
            // did not change and would only be woken for nothing.
            push_system_info(false, true);
            push_patterns(false, true);
            push_playlists(false, true);
            push_schedule(false, true);
            push_led(false, true);
            push_led_effects(false, true);
            // Anything that ALSO changed in this window is now covered.
            bits &= ~(bit(library_notifier::Event::PatternsChanged) |
                      bit(library_notifier::Event::PlaylistsChanged) |
                      bit(library_notifier::Event::LedChanged) |
                      bit(library_notifier::Event::ScheduleChanged) |
                      bit(library_notifier::Event::SystemInfoChanged));
            // ...for the cloud. Local listeners still need their copy.
            if (bits & bit(library_notifier::Event::PatternsChanged)) push_patterns(true, false);
            if (bits & bit(library_notifier::Event::PlaylistsChanged)) push_playlists(true, false);
            if (bits & bit(library_notifier::Event::LedChanged)) push_led(true, false);
            if (bits & bit(library_notifier::Event::ScheduleChanged)) push_schedule(true, false);
            if (bits & bit(library_notifier::Event::SystemInfoChanged)) push_system_info(true, false);
            continue;
        }

        if (bits & bit(library_notifier::Event::PatternsChanged)) push_patterns(true, cloud_up);
        if (bits & bit(library_notifier::Event::PlaylistsChanged)) push_playlists(true, cloud_up);
        if (bits & bit(library_notifier::Event::LedChanged)) push_led(true, cloud_up);
        if (bits & bit(library_notifier::Event::ScheduleChanged)) push_schedule(true, cloud_up);
        if (bits & bit(library_notifier::Event::SystemInfoChanged)) push_system_info(true, cloud_up);
    }
}

}  // namespace

namespace library_notifier {

void init() {
    if (s_task) return;
    s_events = xEventGroupCreate();
    if (!s_events) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }
    // 8 KB: builds full pattern/playlist lists (manifest parse + protobuf
    // pack) - too much for the esp_timer task, cheap for a dedicated one.
    if (xTaskCreate(worker, "api_events", 8192, nullptr, 4, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create api_events task");
        s_task = nullptr;
        return;
    }
    ESP_LOGI(TAG, "api_events task started");
}

void post(Event ev) {
    if (!s_events || ev >= Event::Count) return;
    xEventGroupSetBits(s_events, bit(ev));
}

void schedule_changed(int exclude_local_fd, bool exclude_cloud) {
    s_sched_skip_fd.store(exclude_local_fd);
    s_sched_skip_cloud.store(exclude_cloud);
    post(Event::ScheduleChanged);
}

}  // namespace library_notifier
