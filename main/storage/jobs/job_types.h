#pragma once

#include <string>
#include <cstdint>
#include "esp_err.h"

namespace jobs {

/**
 * @brief Job type enumeration
 */
enum class JobType : uint8_t {
    Conversion,   // .tmp/.thr -> .dat binary conversion
    Thumbnail,    // Generate preview PNG image
    Download,     // Download pattern from cloud
};

/**
 * @brief Job status enumeration
 */
enum class JobStatus : uint8_t {
    Pending,
    InProgress,
    Completed,
    Failed,
};

/**
 * @brief Base job data structure (from database)
 */
struct Job {
    std::string uuid;
    JobType type;
    std::string pattern_uuid;
    JobStatus status;
    int priority;
    int retry_count;
    int max_retries;
    std::string created_at;
    std::string started_at;
    std::string completed_at;
    std::string error_message;
    std::string job_data;  // JSON blob for job-specific data
};

/**
 * @brief Conversion job specific data
 */
struct ConversionJobData {
    std::string temp_path;      // Input .tmp file path
    std::string name;           // Pattern display name
    bool encrypted;             // Is this an encrypted pattern?

    // JSON serialization
    std::string toJson() const;
    static ConversionJobData fromJson(const std::string& json);
};

/**
 * @brief Thumbnail job specific data
 */
struct ThumbnailJobData {
    bool encrypted;             // Source pattern encrypted?
    std::string output_path;    // Output PNG path

    // JSON serialization
    std::string toJson() const;
    static ThumbnailJobData fromJson(const std::string& json);
};

/**
 * @brief Download job specific data
 */
struct DownloadJobData {
    std::string download_url;
    std::string pattern_name;
    std::string pattern_creator;
    int64_t size_bytes = 0;
    bool encrypted = false;
    bool reversible = false;
    int start_point = 0;
    std::string created_at;

    // Purchase receipt (if pattern is purchased)
    // Stored as base64 to fit in JSON job_data blob
    std::string receipt_payload_b64;    // Base64-encoded PurchaseReceipt protobuf
    std::string receipt_signature_b64;  // Base64-encoded RSA signature

    bool hasPurchaseReceipt() const {
        return !receipt_payload_b64.empty() && !receipt_signature_b64.empty();
    }

    // JSON serialization
    std::string toJson() const;
    static DownloadJobData fromJson(const std::string& json);
};

/**
 * @brief Job result returned by executors
 */
struct JobResult {
    bool success;
    std::string error;

    static JobResult ok() { return {true, ""}; }
    static JobResult fail(const std::string& msg) { return {false, msg}; }
};

// String conversion helpers
const char* jobTypeToString(JobType type);
const char* jobStatusToString(JobStatus status);
JobType jobTypeFromString(const char* str);
JobStatus jobStatusFromString(const char* str);

} // namespace jobs
