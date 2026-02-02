#pragma once

#include "thumbnail_common.h"
#include <cstdint>
#include <cstddef>

namespace thumbnail {

// Bytes per row (8 pixels per byte, MSB first)
static constexpr size_t ROW_BYTES = CANVAS_SIZE / 8;

/**
 * @brief RAII wrapper for SPIRAM-allocated 1-bit framebuffer
 *
 * Memory layout:
 *   - Row-major order, 1 bit per pixel (8 pixels per byte)
 *   - MSB first (bit 7 = leftmost pixel in byte)
 *   - Total size: 1024 * 1024 / 8 = 131,072 bytes (128KB)
 *
 * Initialized to black (0x00).
 */
class Framebuffer {
public:
    Framebuffer();
    Framebuffer(Framebuffer&& other) noexcept;
    Framebuffer& operator=(Framebuffer&& other) noexcept;
    ~Framebuffer();

    // Disable copy
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    void clear();

    /**
     * Set pixel (1 = white, 0 = black).
     * Out-of-bounds writes are silently ignored.
     */
    void setPixel(int x, int y);

    /**
     * Get pixel value. Returns false for out-of-bounds.
     */
    bool getPixel(int x, int y) const;

    /**
     * Raw buffer access for PNG encoding.
     */
    const uint8_t* data() const { return buffer_; }
    uint8_t* data() { return buffer_; }

    /**
     * Get pointer to specific row (ROW_BYTES per row).
     */
    const uint8_t* row(uint16_t y) const;

    bool isValid() const { return buffer_ != nullptr; }

    /**
     * Buffer size in bytes.
     */
    static constexpr size_t size() {
        return static_cast<size_t>(CANVAS_SIZE) * ROW_BYTES;
    }

private:
    uint8_t* buffer_;
};

} // namespace thumbnail
