#include "framebuffer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "Framebuffer";

namespace thumbnail {

Framebuffer::Framebuffer() : buffer_(nullptr) {
    buffer_ = static_cast<uint8_t*>(
        heap_caps_malloc(size(), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );

    if (buffer_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SPIRAM");
        return;
    }
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
    }
}

void Framebuffer::clear() {
    if (buffer_) {
        std::memset(buffer_, 0, size());
    }
}

void Framebuffer::setPixel(int x, int y) {
    if (x < 0 || x >= CANVAS_SIZE || y < 0 || y >= CANVAS_SIZE) {
        return;
    }
    // MSB first: bit 7 = leftmost pixel
    size_t byte_idx = y * ROW_BYTES + (x >> 3);
    uint8_t bit_mask = 0x80 >> (x & 7);
    buffer_[byte_idx] |= bit_mask;
}

bool Framebuffer::getPixel(int x, int y) const {
    if (x < 0 || x >= CANVAS_SIZE || y < 0 || y >= CANVAS_SIZE) {
        return false;
    }
    size_t byte_idx = y * ROW_BYTES + (x >> 3);
    uint8_t bit_mask = 0x80 >> (x & 7);
    return (buffer_[byte_idx] & bit_mask) != 0;
}

const uint8_t* Framebuffer::row(uint16_t y) const {
    if (y >= CANVAS_SIZE || !buffer_) {
        return nullptr;
    }
    return &buffer_[y * ROW_BYTES];
}

} // namespace thumbnail
