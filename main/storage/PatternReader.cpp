#include "PatternReader.h"
#include "EncryptedPatternReader.h"
#include "esp_log.h"
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

    current_point_ = 0;
    has_peeked_ = false;

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
    if (!file_ || current_point_ >= header_.point_count) {
        return PatternPoint();
    }

    BinaryPoint bp;
    if (fread(&bp, sizeof(bp), 1, file_) != 1) {
        ESP_LOGE(TAG, "Failed to read binary point at index %zu", current_point_);
        return PatternPoint();
    }

    // Convert uint16 rho to float 0.0-1.0
    double rho = static_cast<double>(bp.rho) / 65535.0;

    return PatternPoint(static_cast<double>(bp.theta), rho);
}

PatternPoint BinaryPatternReader::readNext() {
    if (!file_) return PatternPoint();

    // If we have a peeked value, return it and clear
    if (has_peeked_) {
        has_peeked_ = false;
        current_point_++;
        return peeked_point_;
    }

    PatternPoint point = readBinaryPoint();
    if (point.valid) {
        current_point_++;
    }
    return point;
}

PatternPoint BinaryPatternReader::peekNext() {
    if (!file_) return PatternPoint();

    // If we already peeked, return cached value
    if (has_peeked_) {
        return peeked_point_;
    }

    // Save position
    long prev_pos = ftell(file_);

    // Read next point
    peeked_point_ = readBinaryPoint();

    // Restore position
    fseek(file_, prev_pos, SEEK_SET);

    if (peeked_point_.valid) {
        has_peeked_ = true;
    }

    return peeked_point_;
}

bool BinaryPatternReader::hasMore() const {
    if (!file_) return false;

    if (has_peeked_) return peeked_point_.valid;

    return current_point_ < header_.point_count;
}

esp_err_t BinaryPatternReader::rewind() {
    if (!file_) return ESP_ERR_INVALID_STATE;

    // Seek to start of points (after header)
    fseek(file_, sizeof(UnencryptedPatternHeader), SEEK_SET);
    current_point_ = 0;
    has_peeked_ = false;

    return ESP_OK;
}
