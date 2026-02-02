#pragma once

#include "thumbnail_common.h"
#include <cstdint>
#include <cstddef>

namespace thumbnail {

/**
 * @brief RAII wrapper for SPIRAM-allocated framebuffer
 *
 * Memory layout:
 *   - Row-major order (pixels stored left-to-right, top-to-bottom)
 *   - Single byte per pixel (grayscale, 0=black, 255=white)
 *   - Total size: 600 * 600 = 360,000 bytes
 *
 * Initialized to black (0x00).
 */
class Framebuffer {
public:
    /**
     * Allocate framebuffer in SPIRAM.
     * Check isValid() after construction to verify allocation succeeded.
     */
    Framebuffer();

    /**
     * Move constructor (transfers ownership).
     */
    Framebuffer(Framebuffer&& other) noexcept;

    /**
     * Move assignment operator.
     */
    Framebuffer& operator=(Framebuffer&& other) noexcept;

    /**
     * Destructor - frees SPIRAM.
     */
    ~Framebuffer();

    // Disable copy (large buffer should not be copied)
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    /**
     * Clear framebuffer to black (0x00).
     */
    void clear();

    /**
     * Set pixel value with bounds checking.
     * Out-of-bounds writes are silently ignored.
     */
    void setPixel(int x, int y, uint8_t value);

    /**
     * Set pixel with alpha blending (for anti-aliasing).
     * Uses additive blending for accumulating line intensity.
     */
    void setPixelBlend(int x, int y, uint8_t value, float alpha);

    /**
     * Get pixel value. Returns 0 for out-of-bounds.
     */
    uint8_t getPixel(int x, int y) const;

    /**
     * Raw buffer access for PNG encoding.
     */
    const uint8_t* data() const { return buffer_; }
    uint8_t* data() { return buffer_; }

    /**
     * Get pointer to specific row.
     */
    const uint8_t* row(uint16_t y) const;
    uint8_t* row(uint16_t y);

    /**
     * Check if buffer is valid/allocated.
     */
    bool isValid() const { return buffer_ != nullptr; }

    /**
     * Buffer size in bytes.
     */
    static constexpr size_t size() {
        return static_cast<size_t>(CANVAS_SIZE) * CANVAS_SIZE;
    }

private:
    uint8_t* buffer_;
};

} // namespace thumbnail
