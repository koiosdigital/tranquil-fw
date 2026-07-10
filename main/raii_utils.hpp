#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Scope-based guards for FreeRTOS mutexes/semaphores. Prefer these over raw
// xSemaphoreTake/xSemaphoreGive pairs so the give can never be skipped by an
// early return or thrown exception.
namespace raii {

// Takes the mutex in the constructor and gives it back in the destructor.
// Check operator bool() before touching guarded state when a finite timeout is
// used — with portMAX_DELAY the take always succeeds.
class MutexGuard {
public:
    explicit MutexGuard(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
        : mutex_(mutex)
        , acquired_(mutex ? (xSemaphoreTake(mutex, timeout) == pdTRUE) : false) {}

    ~MutexGuard() {
        if (acquired_) {
            xSemaphoreGive(mutex_);
        }
    }

    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;

    explicit operator bool() const { return acquired_; }
    bool acquired() const { return acquired_; }

    // Release the lock early (before scope exit).
    void release() {
        if (acquired_) {
            xSemaphoreGive(mutex_);
            acquired_ = false;
        }
    }

private:
    SemaphoreHandle_t mutex_;
    bool acquired_;
};

// Gives a semaphore/mutex on scope exit without taking it first. Useful when
// the take already happened (or the semaphore is used as a signal). Call
// cancel() to keep it held past scope exit.
class SemaphoreGiver {
public:
    explicit SemaphoreGiver(SemaphoreHandle_t sem) : sem_(sem) {}

    ~SemaphoreGiver() {
        if (sem_) {
            xSemaphoreGive(sem_);
        }
    }

    SemaphoreGiver(const SemaphoreGiver&) = delete;
    SemaphoreGiver& operator=(const SemaphoreGiver&) = delete;

    void cancel() { sem_ = nullptr; }

private:
    SemaphoreHandle_t sem_;
};

}  // namespace raii
