#include "png_encoder.h"
#include "esp_log.h"
#include <png.h>
#include <cstdio>
#include <csetjmp>
#include <cstring>

static const char* TAG = "PngEncoder";

namespace thumbnail {

// Memory write context for encodeToMemory
struct MemoryWriteContext {
    uint8_t* buffer;
    size_t capacity;
    size_t position;
    bool overflow;
};

static void pngMemoryWriteCallback(png_structp png_ptr, png_bytep data, png_size_t length) {
    auto* ctx = static_cast<MemoryWriteContext*>(png_get_io_ptr(png_ptr));

    if (ctx->position + length > ctx->capacity) {
        ctx->overflow = true;
        return;
    }

    std::memcpy(ctx->buffer + ctx->position, data, length);
    ctx->position += length;
}

static void pngMemoryFlushCallback(png_structp /*png_ptr*/) {
    // No-op for memory writes
}

static ThumbnailError encodeInternal(const Framebuffer& fb,
                                     png_structp png_ptr,
                                     png_infop info_ptr) {
    // Set image attributes
    png_set_IHDR(
        png_ptr, info_ptr,
        CANVAS_SIZE, CANVAS_SIZE,
        8,                      // bit depth
        PNG_COLOR_TYPE_GRAY,    // grayscale
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT
    );

    // Set transparency for grayscale value 0 (black background becomes transparent)
    png_color_16 trans_color;
    trans_color.gray = 0;
    png_set_tRNS(png_ptr, info_ptr, nullptr, 0, &trans_color);

    png_write_info(png_ptr, info_ptr);

    // Write image data row by row
    for (uint16_t y = 0; y < CANVAS_SIZE; ++y) {
        const uint8_t* rowData = fb.row(y);
        if (!rowData) {
            return ThumbnailError::PNG_ENCODE_FAILED;
        }
        png_write_row(png_ptr, const_cast<uint8_t*>(rowData));
    }

    png_write_end(png_ptr, nullptr);

    return ThumbnailError::OK;
}

ThumbnailError PngEncoder::encodeToFile(const Framebuffer& fb,
                                        const char* filepath) {
    FILE* fp = std::fopen(filepath, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        return ThumbnailError::FILE_OPEN_FAILED;
    }

    png_structp png_ptr = png_create_write_struct(
        PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        std::fclose(fp);
        ESP_LOGE(TAG, "Failed to create PNG write struct");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        std::fclose(fp);
        ESP_LOGE(TAG, "Failed to create PNG info struct");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        std::fclose(fp);
        ESP_LOGE(TAG, "PNG encoding error");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    png_init_io(png_ptr, fp);

    ThumbnailError result = encodeInternal(fb, png_ptr, info_ptr);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    std::fclose(fp);

    if (result == ThumbnailError::OK) {
        ESP_LOGI(TAG, "PNG saved to %s", filepath);
    }

    return result;
}

ThumbnailError PngEncoder::encodeToMemory(const Framebuffer& fb,
                                          uint8_t* output_buffer,
                                          size_t buffer_size,
                                          size_t* bytes_written) {
    if (!output_buffer || buffer_size == 0 || !bytes_written) {
        return ThumbnailError::INVALID_INPUT;
    }

    MemoryWriteContext ctx{output_buffer, buffer_size, 0, false};

    png_structp png_ptr = png_create_write_struct(
        PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        ESP_LOGE(TAG, "Failed to create PNG write struct");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        ESP_LOGE(TAG, "Failed to create PNG info struct");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        ESP_LOGE(TAG, "PNG encoding error");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    png_set_write_fn(png_ptr, &ctx, pngMemoryWriteCallback, pngMemoryFlushCallback);

    ThumbnailError result = encodeInternal(fb, png_ptr, info_ptr);

    png_destroy_write_struct(&png_ptr, &info_ptr);

    if (ctx.overflow) {
        ESP_LOGE(TAG, "Buffer overflow during PNG encoding");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    *bytes_written = ctx.position;
    ESP_LOGI(TAG, "PNG encoded to memory: %zu bytes", ctx.position);

    return result;
}

} // namespace thumbnail
