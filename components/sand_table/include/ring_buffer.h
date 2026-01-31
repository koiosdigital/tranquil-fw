#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace sand_table {

    /// Lock-free single-producer, single-consumer ring buffer
    /// Optimized for real-time motion planning where one task produces
    /// segments and another consumes them for step execution.
    template<typename T, size_t Capacity>
    class RingBuffer {
    public:
        static_assert(Capacity > 0, "Capacity must be positive");
        static_assert((Capacity& (Capacity - 1)) == 0, "Capacity must be a power of 2 for efficient modulo");

        RingBuffer() = default;

        // Non-copyable
        RingBuffer(const RingBuffer&) = delete;
        RingBuffer& operator=(const RingBuffer&) = delete;

        /// Try to push an item. Returns false if buffer is full.
        [[nodiscard]] bool push(const T& item) noexcept {
            const size_t head = head_.load(std::memory_order_relaxed);
            const size_t next_head = (head + 1) & kMask;

            // Check if full
            if (next_head == tail_.load(std::memory_order_acquire)) {
                return false;
            }

            buffer_[head] = item;
            head_.store(next_head, std::memory_order_release);
            return true;
        }

        /// Try to push an item (move version). Returns false if buffer is full.
        [[nodiscard]] bool push(T&& item) noexcept {
            const size_t head = head_.load(std::memory_order_relaxed);
            const size_t next_head = (head + 1) & kMask;

            // Check if full
            if (next_head == tail_.load(std::memory_order_acquire)) {
                return false;
            }

            buffer_[head] = std::move(item);
            head_.store(next_head, std::memory_order_release);
            return true;
        }

        /// Try to pop an item. Returns nullopt if buffer is empty.
        [[nodiscard]] std::optional<T> pop() noexcept {
            const size_t tail = tail_.load(std::memory_order_relaxed);

            // Check if empty
            if (tail == head_.load(std::memory_order_acquire)) {
                return std::nullopt;
            }

            T item = std::move(buffer_[tail]);
            tail_.store((tail + 1) & kMask, std::memory_order_release);
            return item;
        }

        /// Try to pop an item into the provided reference. Returns true on success.
        [[nodiscard]] bool pop(T& out) noexcept {
            const size_t tail = tail_.load(std::memory_order_relaxed);

            // Check if empty
            if (tail == head_.load(std::memory_order_acquire)) {
                return false;
            }

            out = std::move(buffer_[tail]);
            tail_.store((tail + 1) & kMask, std::memory_order_release);
            return true;
        }

        /// Peek at the front item without removing it. Returns nullopt if empty.
        [[nodiscard]] std::optional<T> peek() const noexcept {
            const size_t tail = tail_.load(std::memory_order_relaxed);

            if (tail == head_.load(std::memory_order_acquire)) {
                return std::nullopt;
            }

            return buffer_[tail];
        }

        /// Check if buffer is empty
        [[nodiscard]] bool empty() const noexcept {
            return head_.load(std::memory_order_acquire) ==
                tail_.load(std::memory_order_acquire);
        }

        /// Check if buffer is full
        [[nodiscard]] bool full() const noexcept {
            const size_t head = head_.load(std::memory_order_acquire);
            const size_t tail = tail_.load(std::memory_order_acquire);
            return ((head + 1) & kMask) == tail;
        }

        /// Get current number of items in buffer
        [[nodiscard]] size_t size() const noexcept {
            const size_t head = head_.load(std::memory_order_acquire);
            const size_t tail = tail_.load(std::memory_order_acquire);
            return (head - tail) & kMask;
        }

        /// Get buffer capacity
        [[nodiscard]] static constexpr size_t capacity() noexcept {
            return Capacity - 1;  // One slot reserved for full/empty distinction
        }

        /// Get available space
        [[nodiscard]] size_t available() const noexcept {
            return capacity() - size();
        }

        /// Clear the buffer (only safe from consumer side)
        void clear() noexcept {
            tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
        }

    private:
        static constexpr size_t kMask = Capacity - 1;

        std::array<T, Capacity> buffer_{};
        alignas(64) std::atomic<size_t> head_{ 0 };  // Written by producer
        alignas(64) std::atomic<size_t> tail_{ 0 };  // Written by consumer
    };

} // namespace sand_table
