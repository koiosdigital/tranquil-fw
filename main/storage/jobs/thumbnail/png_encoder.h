#pragma once

#include "thumbnail_common.h"
#include "framebuffer.h"

namespace thumbnail {

/**
 * @brief PNG encoder using spng (lightweight alternative to libpng)
 *
 * Encodes 1-bit framebuffer to PNG file.
 *
 * Output PNG:
 *   - 1024x1024 pixels
 *   - 1-bit grayscale (black/white)
 *   - No interlacing
 *   - Transparent black background
 */
class PngEncoder {
public:
    /**
     * Encode framebuffer to PNG file.
     * @param fb Framebuffer to encode
     * @param filepath Output path (e.g., "/sd/previews/xxx.png")
     */
    static ThumbnailError encodeToFile(const Framebuffer& fb,
                                       const char* filepath);

    /**
     * Encode framebuffer to memory buffer.
     * @param fb Framebuffer to encode
     * @param output_buffer Pre-allocated buffer
     * @param buffer_size Buffer capacity
     * @param bytes_written Actual bytes written
     *
     * 1-bit PNGs compress well - 32KB should be sufficient.
     */
    static ThumbnailError encodeToMemory(const Framebuffer& fb,
                                         uint8_t* output_buffer,
                                         size_t buffer_size,
                                         size_t* bytes_written);
};

} // namespace thumbnail
