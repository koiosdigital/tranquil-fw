// PSRAM-backed containers for building large protobuf list messages.
//
// The pattern / playlist list builders used to hold every entry (and every
// std::string behind it) in internal RAM. These keep the protobuf structs,
// the pointer tables and the string storage in PSRAM. The project builds
// without C++ exceptions, so every growth path reports failure instead of
// throwing.
#pragma once

#include <esp_heap_caps.h>

#include <cstddef>
#include <cstring>
#include <string>
#include <type_traits>

// Growable array of trivially-copyable elements (protobuf-c structs,
// pointers). Elements move on growth: take pointers into it only after the
// last push_back, or reserve() the final size up front.
template <typename T>
class PsramArray {
    static_assert(std::is_trivially_copyable<T>::value, "PsramArray holds trivially copyable types only");

public:
    PsramArray() = default;
    ~PsramArray() { heap_caps_free(data_); }
    PsramArray(const PsramArray&) = delete;
    PsramArray& operator=(const PsramArray&) = delete;

    bool reserve(size_t n) {
        if (n <= cap_) return true;
        T* p = static_cast<T*>(heap_caps_realloc(data_, n * sizeof(T), MALLOC_CAP_SPIRAM));
        if (!p) return false;
        data_ = p;
        cap_ = n;
        return true;
    }

    bool push_back(const T& v) {
        if (size_ == cap_ && !reserve(cap_ ? cap_ * 2 : 16)) return false;
        data_[size_++] = v;
        return true;
    }

    // Drop elements past `n` (roll back a partially built entry).
    void truncate(size_t n) {
        if (n < size_) size_ = n;
    }

    T* data() { return data_; }
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    T& operator[](size_t i) { return data_[i]; }

private:
    T* data_ = nullptr;
    size_t size_ = 0;
    size_t cap_ = 0;
};

// Fixed-capacity string arena. intern() returns a stable NUL-terminated
// copy, or nullptr once the arena is full (the caller stops adding entries).
class PsramArena {
public:
    explicit PsramArena(size_t capacity)
        : buf_(static_cast<char*>(heap_caps_malloc(capacity, MALLOC_CAP_SPIRAM))),
          cap_(buf_ ? capacity : 0) {}
    ~PsramArena() { heap_caps_free(buf_); }
    PsramArena(const PsramArena&) = delete;
    PsramArena& operator=(const PsramArena&) = delete;

    bool valid() const { return buf_ != nullptr; }

    const char* intern(const char* s, size_t len) {
        if (!buf_ || len + 1 > cap_ - used_) return nullptr;
        char* out = buf_ + used_;
        if (len) memcpy(out, s, len);
        out[len] = '\0';
        used_ += len + 1;
        return out;
    }

    const char* intern(const std::string& s) { return intern(s.data(), s.size()); }

private:
    char* buf_;
    size_t cap_;
    size_t used_ = 0;
};
