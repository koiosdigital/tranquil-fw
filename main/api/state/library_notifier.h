// Library / state change notifier.
//
// Owns the "api_events" task: a low-priority worker that rebuilds state
// snapshots (pattern list, playlist list, LED config, schedule, system info)
// and pushes them to local WebSocket clients and to the cloud. Producers on
// hot paths (httpd handlers, the cloudlink task, job workers, esp_timer
// callbacks) only post an event here and return immediately.
//
// Every mutation in the ops core (main/api/ops) posts here, so a change made
// over REST reaches WebSocket clients and the cloud exactly like one made
// over the socket.
//
// Events are debounced (a bulk playlist download completing ten patterns in
// two seconds produces one list push, not ten) and coalesced per kind.
#pragma once

#include <cstdint>

namespace library_notifier {

enum class Event : uint8_t {
    PatternsChanged = 0,
    PlaylistsChanged,
    LedChanged,
    ScheduleChanged,
    SystemInfoChanged,
    // Cloud session came up: push every snapshot the cloud caches so it
    // never has to ask (player state is pushed by PlayerStateBroadcaster).
    // Includes the static LED effects list, once per connect.
    CloudSnapshot,
    Count,
};

// Start the worker task. Safe to call more than once.
void init();

// Post an event. Non-blocking; safe from any task (not from ISRs).
void post(Event ev);

inline void patterns_changed() { post(Event::PatternsChanged); }
inline void playlists_changed() { post(Event::PlaylistsChanged); }
inline void led_changed() { post(Event::LedChanged); }
inline void system_info_changed() { post(Event::SystemInfoChanged); }
inline void cloud_snapshot() { post(Event::CloudSnapshot); }

// Schedule push. The requester of a schedule set already received the
// stored schedule as its reply, so it can be left out of the push
// (exclude_local_fd = its socket, or exclude_cloud when it was the cloud).
// -1 / false exclude nobody.
void schedule_changed(int exclude_local_fd, bool exclude_cloud);
inline void schedule_changed() { schedule_changed(-1, false); }

}  // namespace library_notifier
