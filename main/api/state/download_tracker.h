// Download tracker - tracks pending pattern downloads for response routing
// Routes download progress back to the client that initiated the request
#pragma once

#include <esp_err.h>
#include <map>
#include <string>
#include <cstdint>

struct PendingDownload {
    int origin_socket_fd;       // Socket FD of the client that requested the download
    int64_t request_time_us;    // Time when request was made (for timeout)
    std::string pattern_uuid;
};

class DownloadTracker {
public:
    static DownloadTracker& instance();

    // Register a pending download request
    // Called when forwarding a RequestPatternDownload to cloud
    void trackDownload(const std::string& pattern_uuid, int origin_socket_fd);

    // Get the origin socket for a download (and remove from tracking)
    // Called when receiving download progress from cloud
    // Returns -1 if not found or expired
    int getOriginSocket(const std::string& pattern_uuid);

    // Peek the origin socket without removing
    int peekOriginSocket(const std::string& pattern_uuid) const;

    // Remove a tracked download (e.g., download complete or cancelled)
    void removeDownload(const std::string& pattern_uuid);

    // Clean up expired entries (called periodically)
    void cleanupExpired();

    // Check if a download is being tracked
    bool isTracked(const std::string& pattern_uuid) const;

private:
    DownloadTracker() = default;

    // Map of pattern_uuid -> PendingDownload
    std::map<std::string, PendingDownload> pending_downloads_;

    // Timeout for pending downloads (5 minutes)
    static constexpr int64_t PENDING_TIMEOUT_US = 5 * 60 * 1000 * 1000LL;
};
