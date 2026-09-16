#pragma once

// Internal-RAM placement for objects the RMT TX-done ISR touches.
//
// With CONFIG_SPIRAM_USE_MALLOC, plain new/malloc may land in PSRAM, and an
// IRAM ISR that dereferences PSRAM while the flash cache is disabled (NVS
// commit, OTA write) crashes. The step sequencer (its stop flag, pending
// counters, per-buffer step credits, completion semaphore handle) and the
// two StepperDriver objects (their position counters, credited from the
// ISR) must therefore live in internal RAM regardless of that option.

#include "esp_heap_caps.h"

#include <memory>
#include <new>
#include <utility>

namespace sand_table {

    template <typename T>
    struct InternalRamDeleter {
        void operator()(T* ptr) const {
            if (ptr) {
                ptr->~T();
                heap_caps_free(ptr);
            }
        }
    };

    template <typename T>
    using internal_ptr = std::unique_ptr<T, InternalRamDeleter<T>>;

    /// Construct T in internal (non-PSRAM) RAM. nullptr on allocation failure.
    template <typename T, typename... Args>
    internal_ptr<T> make_internal(Args&&... args) {
        void* mem = heap_caps_malloc(sizeof(T), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!mem) {
            return internal_ptr<T>();
        }
        return internal_ptr<T>(new (mem) T(std::forward<Args>(args)...));
    }

} // namespace sand_table
