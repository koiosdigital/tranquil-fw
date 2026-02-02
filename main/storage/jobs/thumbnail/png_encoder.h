#pragma once

#include "thumbnail_common.h"
#include "framebuffer.h"

namespace thumbnail {

/**
 * @brief PNG encoder using libpng
 *
 * Encodes grayscale framebuffer to PNG file.
 *
 * Output PNG:
 *   - 600x600 pixels
 *   - 8-bit grayscale
 *   - No interlacing
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
     * Recommend allocating at least 100KB for safety.
     */
    static ThumbnailError encodeToMemory(const Framebuffer& fb,
                                         uint8_t* output_buffer,
                                         size_t buffer_size,
                                         size_t* bytes_written);
};

} // namespace thumbnail
