#include "png_encoder.h"
#include "esp_log.h"
#include "spng.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "PngEncoder";

namespace thumbnail {

ThumbnailError PngEncoder::encodeToFile(const Framebuffer& fb,
                                        const char* filepath) {
    FILE* fp = std::fopen(filepath, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        return ThumbnailError::FILE_OPEN_FAILED;
    }

    spng_ctx* ctx = spng_ctx_new(SPNG_CTX_ENCODER);
    if (!ctx) {
        std::fclose(fp);
        ESP_LOGE(TAG, "Failed to create spng context");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    // Set output file
    spng_set_png_file(ctx, fp);

    // Set image header (1-bit grayscale)
    spng_ihdr ihdr = {};
    ihdr.width = CANVAS_SIZE;
    ihdr.height = CANVAS_SIZE;
    ihdr.bit_depth = 1;
    ihdr.color_type = SPNG_COLOR_TYPE_GRAYSCALE;
    ihdr.interlace_method = SPNG_INTERLACE_NONE;

    int ret = spng_set_ihdr(ctx, &ihdr);
    if (ret != SPNG_OK) {
        spng_ctx_free(ctx);
        std::fclose(fp);
        ESP_LOGE(TAG, "spng_set_ihdr failed: %s", spng_strerror(ret));
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    // Set transparency for value 0 (black background becomes transparent)
    spng_trns trns = {};
    trns.gray = 0;
    spng_set_trns(ctx, &trns);

    // Encode image - 1-bit packed format
    ret = spng_encode_image(ctx, fb.data(), Framebuffer::size(), SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE);

    spng_ctx_free(ctx);
    std::fclose(fp);

    if (ret != SPNG_OK) {
        ESP_LOGE(TAG, "spng_encode_image failed: %s", spng_strerror(ret));
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    ESP_LOGD(TAG, "PNG saved to %s", filepath);
    return ThumbnailError::OK;
}

ThumbnailError PngEncoder::encodeToMemory(const Framebuffer& fb,
                                          uint8_t* output_buffer,
                                          size_t buffer_size,
                                          size_t* bytes_written) {
    if (!output_buffer || buffer_size == 0 || !bytes_written) {
        return ThumbnailError::INVALID_INPUT;
    }

    spng_ctx* ctx = spng_ctx_new(SPNG_CTX_ENCODER);
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to create spng context");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    // Enable encoding to internal buffer
    spng_set_option(ctx, SPNG_ENCODE_TO_BUFFER, 1);

    // Set image header (1-bit grayscale)
    spng_ihdr ihdr = {};
    ihdr.width = CANVAS_SIZE;
    ihdr.height = CANVAS_SIZE;
    ihdr.bit_depth = 1;
    ihdr.color_type = SPNG_COLOR_TYPE_GRAYSCALE;
    ihdr.interlace_method = SPNG_INTERLACE_NONE;

    int ret = spng_set_ihdr(ctx, &ihdr);
    if (ret != SPNG_OK) {
        spng_ctx_free(ctx);
        ESP_LOGE(TAG, "spng_set_ihdr failed: %s", spng_strerror(ret));
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    // Set transparency for value 0 (black background becomes transparent)
    spng_trns trns = {};
    trns.gray = 0;
    spng_set_trns(ctx, &trns);

    // Encode image - 1-bit packed format
    ret = spng_encode_image(ctx, fb.data(), Framebuffer::size(), SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE);

    if (ret != SPNG_OK) {
        spng_ctx_free(ctx);
        ESP_LOGE(TAG, "spng_encode_image failed: %s", spng_strerror(ret));
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    // Get encoded buffer
    size_t png_size = 0;
    int error = 0;
    void* png_buf = spng_get_png_buffer(ctx, &png_size, &error);

    if (!png_buf || error != SPNG_OK) {
        spng_ctx_free(ctx);
        ESP_LOGE(TAG, "spng_get_png_buffer failed");
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    if (png_size > buffer_size) {
        spng_ctx_free(ctx);
        ESP_LOGE(TAG, "Buffer too small: need %zu, have %zu", png_size, buffer_size);
        return ThumbnailError::PNG_ENCODE_FAILED;
    }

    std::memcpy(output_buffer, png_buf, png_size);
    *bytes_written = png_size;

    spng_ctx_free(ctx);

    ESP_LOGD(TAG, "PNG encoded to memory: %zu bytes", png_size);
    return ThumbnailError::OK;
}

} // namespace thumbnail
