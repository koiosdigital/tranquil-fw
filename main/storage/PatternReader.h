#pragma once

#include "esp_err.h"
#include <cstdio>
#include <memory>
#include <stdint.h>

/**
 * @brief Unencrypted binary pattern magic "THRB"
 */
#define BINARY_PATTERN_MAGIC 0x42524854

/**
 * @brief Unencrypted binary pattern header (8 bytes)
 */
#pragma pack(push, 1)
struct UnencryptedPatternHeader {
    uint32_t magic;        // "THRB" (0x42524854)
    uint32_t point_count;  // Number of points
};

/**
 * @brief Binary point format (6 bytes per point)
 */
struct BinaryPoint {
    float theta;     // Angle in radians (signed, can exceed ±2π)
    uint16_t rho;    // Radius: 0-65535 maps to 0.0-1.0
};
#pragma pack(pop)

static_assert(sizeof(UnencryptedPatternHeader) == 8,
    "UnencryptedPatternHeader size mismatch");
static_assert(sizeof(BinaryPoint) == 6,
    "BinaryPoint size mismatch");

/**
 * @brief Represents a single point in a pattern
 */
struct PatternPoint {
    double theta;  // Angle in radians (continuous, can exceed 2*PI)
    double rho;    // Radius normalized 0.0 (center) to 1.0 (edge)
    bool valid = false;

    PatternPoint() : theta(0.0), rho(0.0), valid(false) {}
    PatternPoint(double t, double r) : theta(t), rho(r), valid(true) {}
};

/**
 * @brief Abstract interface for reading pattern files
 *
 * Supports streaming access without loading entire file into memory.
 */
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

    // Get total number of points in the pattern (O(1) from header)
    virtual size_t getTotalLines() = 0;

    // Get current point index (0-based)
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

/**
 * @brief Binary pattern reader for unencrypted .dat files with THRB magic
 *
 * Format: 8-byte header + 6-byte packed points (float theta, uint16 rho)
 * Server converts uploaded .thr text files to this binary format.
 */
class BinaryPatternReader : public IPatternReader {
public:
    BinaryPatternReader();
    ~BinaryPatternReader() override;

    // Prevent copying
    BinaryPatternReader(const BinaryPatternReader&) = delete;
    BinaryPatternReader& operator=(const BinaryPatternReader&) = delete;

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
    static constexpr const char* PATTERNS_PATH = "/sd/patterns";

    FILE* file_ = nullptr;
    UnencryptedPatternHeader header_;
    size_t current_point_ = 0;

    // Peek support
    bool has_peeked_ = false;
    PatternPoint peeked_point_;

    void getFilePath(const char* uuid, char* path, size_t path_size);
    PatternPoint readBinaryPoint();
};
