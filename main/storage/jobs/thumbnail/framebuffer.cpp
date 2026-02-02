#include "framebuffer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstring>
#include <algorithm>

static const char* TAG = "Framebuffer";

namespace thumbnail {

Framebuffer::Framebuffer() : buffer_(nullptr) {
    buffer_ = static_cast<uint8_t*>(
        heap_caps_malloc(size(), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );

    if (buffer_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes in SPIRAM", size());
        return;  // isValid() will return false
    }

    ESP_LOGI(TAG, "Allocated %zu bytes in SPIRAM", size());
    clear();
}

Framebuffer::Framebuffer(Framebuffer&& other) noexcept : buffer_(other.buffer_) {
    other.buffer_ = nullptr;
}

Framebuffer& Framebuffer::operator=(Framebuffer&& other) noexcept {
    if (this != &other) {
        if (buffer_) {
            heap_caps_free(buffer_);
        }
        buffer_ = other.buffer_;
        other.buffer_ = nullptr;
    }
    return *this;
}

Framebuffer::~Framebuffer() {
    if (buffer_) {
        heap_caps_free(buffer_);
        buffer_ = nullptr;
        ESP_LOGI(TAG, "Freed SPIRAM buffer");
    }
}

void Framebuffer::clear() {
    if (buffer_) {
        std::memset(buffer_, 0, size());
    }
}

void Framebuffer::setPixel(int x, int y, uint8_t value) {
    if (x < 0 || x >= CANVAS_SIZE || y < 0 || y >= CANVAS_SIZE) {
        return;
    }
    buffer_[y * CANVAS_SIZE + x] = value;
}

void Framebuffer::setPixelBlend(int x, int y, uint8_t value, float alpha) {
    if (x < 0 || x >= CANVAS_SIZE || y < 0 || y >= CANVAS_SIZE) {
        return;
    }

    uint8_t* pixel = &buffer_[y * CANVAS_SIZE + x];
    // Additive blending (white lines accumulate)
    float existing = static_cast<float>(*pixel) / 255.0f;
    float addition = (static_cast<float>(value) / 255.0f) * alpha;
    float blended = std::min(1.0f, existing + addition);
    *pixel = static_cast<uint8_t>(blended * 255.0f);
}

uint8_t Framebuffer::getPixel(int x, int y) const {
    if (x < 0 || x >= CANVAS_SIZE || y < 0 || y >= CANVAS_SIZE) {
        return 0;
    }
    return buffer_[y * CANVAS_SIZE + x];
}

const uint8_t* Framebuffer::row(uint16_t y) const {
    if (y >= CANVAS_SIZE || !buffer_) {
        return nullptr;
    }
    return &buffer_[y * CANVAS_SIZE];
}

uint8_t* Framebuffer::row(uint16_t y) {
    if (y >= CANVAS_SIZE || !buffer_) {
        return nullptr;
    }
    return &buffer_[y * CANVAS_SIZE];
}

} // namespace thumbnail
