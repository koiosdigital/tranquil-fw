#include "PatternReader.h"
#include "EncryptedPatternReader.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

static const char* TAG = "PatternReader";

// Factory function
std::unique_ptr<IPatternReader> createPatternReader(bool is_encrypted) {
    if (is_encrypted) {
        return std::make_unique<EncryptedPatternReader>();
    }
    return std::make_unique<BinaryPatternReader>();
}

// =============================================================================
// BinaryPatternReader Implementation
// =============================================================================

BinaryPatternReader::BinaryPatternReader() {
    memset(&header_, 0, sizeof(header_));
}

BinaryPatternReader::~BinaryPatternReader() {
    close();
}

void BinaryPatternReader::getFilePath(const char* uuid, char* path, size_t path_size) {
    snprintf(path, path_size, "%s/%s.dat", PATTERNS_PATH, uuid);
}

esp_err_t BinaryPatternReader::open(const char* uuid) {
    if (file_) {
        close();
    }

    char path[256];
    getFilePath(uuid, path, sizeof(path));

    file_ = fopen(path, "rb");
    if (!file_) {
        ESP_LOGE(TAG, "Failed to open pattern file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    // Read header
    if (fread(&header_, sizeof(header_), 1, file_) != 1) {
        ESP_LOGE(TAG, "Failed to read pattern header");
        fclose(file_);
        file_ = nullptr;
        return ESP_FAIL;
    }

    // Validate magic
    if (header_.magic != BINARY_PATTERN_MAGIC) {
        ESP_LOGE(TAG, "Invalid pattern magic: 0x%08lX (expected THRB)",
                 (unsigned long)header_.magic);
        fclose(file_);
        file_ = nullptr;
        return ESP_ERR_INVALID_ARG;
    }

    // Validate point_count against actual file size so a truncated file
    // can't wedge playback (hasMore() true forever past EOF)
    long file_size = -1;
    if (fseek(file_, 0, SEEK_END) == 0) {
        file_size = ftell(file_);
    }
    if (file_size < 0 || fseek(file_, sizeof(header_), SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Failed to determine pattern file size");
        fclose(file_);
        file_ = nullptr;
        return ESP_FAIL;
    }
    size_t available_points =
        (static_cast<size_t>(file_size) - sizeof(header_)) / sizeof(BinaryPoint);
    if (header_.point_count > available_points) {
        ESP_LOGW(TAG, "Pattern truncated: header claims %lu points, file holds %zu",
                 (unsigned long)header_.point_count, available_points);
        if (available_points == 0) {
            fclose(file_);
            file_ = nullptr;
            return ESP_ERR_INVALID_SIZE;
        }
        header_.point_count = available_points;
    }

    current_point_ = 0;
    has_peeked_ = false;
    read_failed_ = false;

    ESP_LOGI(TAG, "Opened binary pattern: %s (%lu points)",
             path, (unsigned long)header_.point_count);
    return ESP_OK;
}

void BinaryPatternReader::close() {
    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }
    memset(&header_, 0, sizeof(header_));
    current_point_ = 0;
    has_peeked_ = false;
    read_failed_ = false;
}

bool BinaryPatternReader::isOpen() const {
    return file_ != nullptr;
}

size_t BinaryPatternReader::getTotalLines() {
    return header_.point_count;
}

size_t BinaryPatternReader::getCurrentLine() const {
    return current_point_;
}

PatternPoint BinaryPatternReader::readBinaryPoint() {
    // Consume and account for every record read (current_point_++ per fread),
    // skipping corrupt ones until a valid point, the point-count bound, or a
    // read failure. Advancing current_point_ per record — not per valid point —
    // keeps the file position, the count bound, and the peek/read cache in
    // agreement. Previously a skipped record left current_point_ behind the
    // file, so a peekNext() that hit a bad point advanced the stream without
    // caching, and the following readNext() re-read and dropped the next
    // (valid) point (silently skipping it and mis-counting path length).
    while (file_ && !read_failed_ && current_point_ < header_.point_count) {
        BinaryPoint bp;
        if (fread(&bp, sizeof(bp), 1, file_) != 1) {
            ESP_LOGE(TAG, "Failed to read binary point at index %zu", current_point_);
            read_failed_ = true;  // terminal: end playback cleanly instead of spinning
            return PatternPoint();
        }
        ++current_point_;  // one record consumed, valid or not

        // Reject garbage motion targets: theta must be a finite, sane angle
        // (|theta| < 10000 rad is generous for multi-rotation patterns)
        if (!std::isfinite(bp.theta) || std::fabs(bp.theta) >= 10000.0f) {
            ESP_LOGW(TAG, "Invalid theta at point %zu, skipping", current_point_ - 1);
            continue;  // skip the corrupt record and try the next one
        }

        // Convert uint16 rho to float 0.0-1.0 (in [0,1] by construction)
        double rho = static_cast<double>(bp.rho) / 65535.0;
        return PatternPoint(static_cast<double>(bp.theta), rho);
    }

    return PatternPoint();
}

PatternPoint BinaryPatternReader::readNext() {
    if (!file_) return PatternPoint();

    // Return the peeked value if present. The record was already read and
    // counted (current_point_ advanced) when peekNext() consumed it, so do
    // not advance again here.
    if (has_peeked_) {
        has_peeked_ = false;
        return peeked_point_;
    }

    return readBinaryPoint();
}

PatternPoint BinaryPatternReader::peekNext() {
    if (!file_) return PatternPoint();

    // If we already peeked, return cached value
    if (has_peeked_) {
        return peeked_point_;
    }

    // Read the point and leave the file advanced past it: readNext()'s
    // cached branch returns peeked_point_ WITHOUT re-reading the stream,
    // so rewinding here would desync the file from current_point_ — every
    // peek+read cycle re-reads the same bytes and playback replays one
    // point until the index runs out.
    peeked_point_ = readBinaryPoint();

    if (peeked_point_.valid) {
        has_peeked_ = true;
    }

    return peeked_point_;
}

bool BinaryPatternReader::hasMore() const {
    if (!file_ || read_failed_) return false;

    if (has_peeked_) return peeked_point_.valid;

    return current_point_ < header_.point_count;
}

esp_err_t BinaryPatternReader::rewind() {
    if (!file_) return ESP_ERR_INVALID_STATE;

    // Seek to start of points (after header)
    if (fseek(file_, sizeof(UnencryptedPatternHeader), SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Rewind seek failed");
        read_failed_ = true;
        return ESP_FAIL;
    }
    current_point_ = 0;
    has_peeked_ = false;
    read_failed_ = false;

    return ESP_OK;
}
