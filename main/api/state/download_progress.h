// Download progress broadcaster — implements the PatternDownloadProgressReport
// local WebSocket broadcast the proto promises ("every 5% per job").
//
// Progress scale (coarse pipeline stages, not raw bytes):
//   0       request accepted, job not started
//   10      download job picked up by a worker
//   20-80   HTTP transfer (byte progress mapped into the band)
//   90      thumbnailer started
//   100     thumbnailer done — pattern fully usable
#pragma once

#include <cstdint>
#include <map>
#include <string>

class DownloadProgressBroadcaster {
public:
    static DownloadProgressBroadcaster& instance();

    // Start tracking a download at 0% and broadcast. Called when a local
    // client's RequestPatternDownload is accepted/forwarded.
    void track(const std::string& pattern_uuid);

    // Set progress and broadcast if it changed. Tracks the uuid if it isn't
    // yet (downloads can also be cloud-initiated).
    void report(const std::string& pattern_uuid, uint8_t pct);

    // Like report(), but ignored for untracked uuids. Used by the thumbnail
    // stage, which also runs for local uploads that are not downloads.
    void reportIfTracked(const std::string& pattern_uuid, uint8_t pct);

    // Broadcast a final 100% for a tracked uuid and stop tracking it.
    void complete(const std::string& pattern_uuid);

    // Broadcast a terminal failure (failed=true + error) for a tracked uuid,
    // then stop tracking it. No-op for untracked uuids (e.g. local uploads,
    // whose failures ride the conversion/thumbnail progress reports instead).
    void fail(const std::string& pattern_uuid, const std::string& error);

    // Stop tracking without any broadcast (cancel). Prefer fail() for real
    // failures so clients can clear their UI instead of timing out.
    void drop(const std::string& pattern_uuid);

private:
    DownloadProgressBroadcaster() = default;

    void set(const std::string& pattern_uuid, uint8_t pct, bool only_if_tracked);
    // Pack the current map into a PatternDownloadProgressReport and broadcast
    // to local WS clients. Called with the lock held only to snapshot.
    void broadcast();

    std::map<std::string, uint8_t> active_;
};
