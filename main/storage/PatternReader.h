#pragma once

#include "esp_err.h"
#include <cstdio>
#include <memory>

// Represents a single point in a pattern
struct PatternPoint {
    double theta;  // Angle in radians (continuous, can exceed 2*PI)
    double rho;    // Radius normalized 0.0 (center) to 1.0 (edge)
    bool valid = false;

    PatternPoint() : theta(0.0), rho(0.0), valid(false) {}
    PatternPoint(double t, double r) : theta(t), rho(r), valid(true) {}
};

// Abstract interface for reading pattern files
// Supports streaming access without loading entire file into memory
class IPatternReader {
public:
    virtual ~IPatternReader() = default;

    // Open pattern file by UUID
    // Returns ESP_OK on success
    virtual esp_err_t open(const char* uuid) = 0;

    // Close the pattern file
    virtual void close() = 0;

    // Check if reader has an open file
    virtual bool isOpen() const = 0;

    // Get total number of lines/points in the pattern
    // May require scanning the file on first call
    virtual size_t getTotalLines() = 0;

    // Get current line index (0-based)
    virtual size_t getCurrentLine() const = 0;

    // Read the next point and advance position
    // Returns invalid PatternPoint if at end or error
    virtual PatternPoint readNext() = 0;

    // Peek at the next point without advancing position
    virtual PatternPoint peekNext() = 0;

    // Check if more data is available
    virtual bool hasMore() const = 0;

    // Reset to beginning of file
    virtual esp_err_t rewind() = 0;
};

// Factory function to create the appropriate reader based on encryption status
std::unique_ptr<IPatternReader> createPatternReader(bool is_encrypted);

// Plain text pattern reader for .thr files
// Format: "theta rho\n" per line (space-separated, text format)
class PlainTextPatternReader : public IPatternReader {
public:
    PlainTextPatternReader();
    ~PlainTextPatternReader() override;

    // Prevent copying
    PlainTextPatternReader(const PlainTextPatternReader&) = delete;
    PlainTextPatternReader& operator=(const PlainTextPatternReader&) = delete;

    esp_err_t open(const char* uuid) override;
    void close() override;
    bool isOpen() const override;
    size_t getTotalLines() override;
    size_t getCurrentLine() const override;
    PatternPoint readNext() override;
    PatternPoint peekNext() override;
    bool hasMore() const override;
    esp_err_t rewind() override;

private:
    static constexpr size_t LINE_BUFFER_SIZE = 512;
    static constexpr const char* PATTERNS_PATH = "/sd/patterns";

    FILE* file_ = nullptr;
    size_t total_lines_ = 0;
    size_t current_line_ = 0;
    bool lines_counted_ = false;
    char line_buffer_[LINE_BUFFER_SIZE];

    // Cached next line for peek support
    bool has_peeked_ = false;
    PatternPoint peeked_point_;

    PatternPoint parseLine(const char* line);
    void getFilePath(const char* uuid, char* path, size_t path_size);
};
