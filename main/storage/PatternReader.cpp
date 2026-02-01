#include "PatternReader.h"
#include "EncryptedPatternReader.h"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>

static const char* TAG = "PatternReader";

// Factory function
std::unique_ptr<IPatternReader> createPatternReader(bool is_encrypted) {
    if (is_encrypted) {
        return std::make_unique<EncryptedPatternReader>();
    }
    return std::make_unique<PlainTextPatternReader>();
}

// =============================================================================
// PlainTextPatternReader Implementation
// =============================================================================

PlainTextPatternReader::PlainTextPatternReader() {
    memset(line_buffer_, 0, sizeof(line_buffer_));
}

PlainTextPatternReader::~PlainTextPatternReader() {
    close();
}

void PlainTextPatternReader::getFilePath(const char* uuid, char* path, size_t path_size) {
    snprintf(path, path_size, "%s/%s.thr", PATTERNS_PATH, uuid);
}

esp_err_t PlainTextPatternReader::open(const char* uuid) {
    if (file_) {
        close();
    }

    char path[256];
    getFilePath(uuid, path, sizeof(path));

    file_ = fopen(path, "r");
    if (!file_) {
        ESP_LOGE(TAG, "Failed to open pattern file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    total_lines_ = 0;
    current_line_ = 0;
    lines_counted_ = false;
    has_peeked_ = false;

    ESP_LOGI(TAG, "Opened pattern file: %s", path);
    return ESP_OK;
}

void PlainTextPatternReader::close() {
    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }
    total_lines_ = 0;
    current_line_ = 0;
    lines_counted_ = false;
    has_peeked_ = false;
}

bool PlainTextPatternReader::isOpen() const {
    return file_ != nullptr;
}

size_t PlainTextPatternReader::getTotalLines() {
    if (!file_) return 0;

    if (!lines_counted_) {
        // Save current position
        long prev_pos = ftell(file_);

        // Count lines from beginning
        ::rewind(file_);
        total_lines_ = 0;
        while (fgets(line_buffer_, sizeof(line_buffer_), file_)) {
            total_lines_++;
        }

        // Restore position
        fseek(file_, prev_pos, SEEK_SET);
        lines_counted_ = true;

        ESP_LOGI(TAG, "Pattern has %zu lines", total_lines_);
    }

    return total_lines_;
}

size_t PlainTextPatternReader::getCurrentLine() const {
    return current_line_;
}

PatternPoint PlainTextPatternReader::parseLine(const char* line) {
    if (!line || strlen(line) == 0) {
        return PatternPoint();
    }

    // Skip empty lines and comments
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '#' || *line == '\n' || *line == '\r') {
        return PatternPoint();
    }

    // Parse space-separated theta and rho values
    char theta_str[64], rho_str[64];
    int parsed = sscanf(line, "%63s %63s", theta_str, rho_str);

    if (parsed != 2) {
        ESP_LOGW(TAG, "Failed to parse line: %.50s...", line);
        return PatternPoint();
    }

    char* endptr;
    double theta = strtod(theta_str, &endptr);
    if (*endptr != '\0') {
        ESP_LOGW(TAG, "Invalid theta value: %s", theta_str);
        return PatternPoint();
    }

    double rho = strtod(rho_str, &endptr);
    if (*endptr != '\0') {
        ESP_LOGW(TAG, "Invalid rho value: %s", rho_str);
        return PatternPoint();
    }

    return PatternPoint(theta, rho);
}

PatternPoint PlainTextPatternReader::readNext() {
    if (!file_) return PatternPoint();

    // If we have a peeked value, return it and clear
    if (has_peeked_) {
        has_peeked_ = false;
        current_line_++;
        return peeked_point_;
    }

    // Read next line
    if (!fgets(line_buffer_, sizeof(line_buffer_), file_)) {
        return PatternPoint();
    }

    // Remove trailing newline
    size_t len = strlen(line_buffer_);
    while (len > 0 && (line_buffer_[len - 1] == '\n' || line_buffer_[len - 1] == '\r')) {
        line_buffer_[--len] = '\0';
    }

    current_line_++;
    return parseLine(line_buffer_);
}

PatternPoint PlainTextPatternReader::peekNext() {
    if (!file_) return PatternPoint();

    // If we already peeked, return cached value
    if (has_peeked_) {
        return peeked_point_;
    }

    // Save position
    long prev_pos = ftell(file_);

    // Read next line
    if (!fgets(line_buffer_, sizeof(line_buffer_), file_)) {
        fseek(file_, prev_pos, SEEK_SET);
        return PatternPoint();
    }

    // Remove trailing newline
    size_t len = strlen(line_buffer_);
    while (len > 0 && (line_buffer_[len - 1] == '\n' || line_buffer_[len - 1] == '\r')) {
        line_buffer_[--len] = '\0';
    }

    // Parse and cache
    peeked_point_ = parseLine(line_buffer_);
    has_peeked_ = true;

    // Don't restore position - we'll consume this line on readNext
    // Actually, for peek we should restore
    fseek(file_, prev_pos, SEEK_SET);

    return peeked_point_;
}

bool PlainTextPatternReader::hasMore() const {
    if (!file_) return false;

    if (has_peeked_) return peeked_point_.valid;

    // Check if we're at EOF
    int c = fgetc(const_cast<FILE*>(file_));
    if (c == EOF) return false;
    ungetc(c, const_cast<FILE*>(file_));
    return true;
}

esp_err_t PlainTextPatternReader::rewind() {
    if (!file_) return ESP_ERR_INVALID_STATE;

    ::rewind(file_);
    current_line_ = 0;
    has_peeked_ = false;

    return ESP_OK;
}
