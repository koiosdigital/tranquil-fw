#include "stepper_driver.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <algorithm>

namespace sand_table {

    static const char* TAG = "StepperDriver";

    // =============================================================================
    // StepperDriver Implementation
    // =============================================================================

    StepperDriver::StepperDriver(const Pins& pins, const char* name)
        : pins_(pins)
        , name_(name)
    {
    }

    StepperDriver::~StepperDriver() {
        if (rmt_channel_) {
            rmt_del_channel(rmt_channel_);
        }
        if (step_encoder_) {
            rmt_del_encoder(step_encoder_);
        }
    }

    Result<void> StepperDriver::init() {
        auto gpio_result = init_gpio();
        if (gpio_result.is_err()) {
            return gpio_result;
        }

        auto rmt_result = init_rmt();
        if (rmt_result.is_err()) {
            return rmt_result;
        }

        initialized_ = true;
        return Result<void>::ok();
    }

    Result<void> StepperDriver::init_gpio() {
        // Configure step pin with maximum drive strength for TMC2209
        gpio_config_t step_config = {
            .pin_bit_mask = (1ULL << pins_.step),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&step_config) != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to configure step pin", name_);
            return Result<void>::err(MotionError::HardwareFault);
        }
        gpio_set_drive_capability(pins_.step, GPIO_DRIVE_CAP_3);  // 40mA max drive
        gpio_set_level(pins_.step, 0);

        // Configure direction pin with maximum drive strength
        gpio_config_t dir_config = {
            .pin_bit_mask = (1ULL << pins_.dir),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&dir_config) != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to configure dir pin", name_);
            return Result<void>::err(MotionError::HardwareFault);
        }
        gpio_set_drive_capability(pins_.dir, GPIO_DRIVE_CAP_3);  // 40mA max drive
        gpio_set_level(pins_.dir, 0);

        // Configure enable pin (active low typically)
        gpio_config_t enable_config = {
            .pin_bit_mask = (1ULL << pins_.enable),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&enable_config) != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to configure enable pin", name_);
            return Result<void>::err(MotionError::HardwareFault);
        }
        gpio_set_level(pins_.enable, 1);  // Disabled by default (active low)

        return Result<void>::ok();
    }

    Result<void> StepperDriver::init_rmt() {
        // For now, we'll use GPIO + timer for step generation
        // RMT can be added later for more precise timing if needed
        return Result<void>::ok();
    }

    void StepperDriver::set_enabled(bool enabled) {
        enabled_ = enabled;
        // Active low enable pin
        gpio_set_level(pins_.enable, enabled ? 0 : 1);
        ESP_LOGD(TAG, "%s: %s", name_, enabled ? "enabled" : "disabled");
    }

    void StepperDriver::set_direction(bool positive) {
        direction_positive_ = positive;
        gpio_set_level(pins_.dir, positive ? 1 : 0);
    }

    void StepperDriver::step_once() {
        gpio_set_level(pins_.step, 1);
        esp_rom_delay_us(HardwareConfig::MIN_STEP_PULSE_US);
        gpio_set_level(pins_.step, 0);

        // Update position
        int32_t pos = position_.load(std::memory_order_relaxed);
        pos += direction_positive_ ? 1 : -1;
        position_.store(pos, std::memory_order_release);
    }

    void StepperDriver::step_fixed_rate(uint32_t steps, uint32_t interval_us) {
        for (uint32_t i = 0; i < steps; ++i) {
            step_once();
            if (interval_us > HardwareConfig::MIN_STEP_PULSE_US) {
                esp_rom_delay_us(interval_us - HardwareConfig::MIN_STEP_PULSE_US);
            }
        }
    }

    // =============================================================================
    // RmtStepSequencer Implementation
    // =============================================================================

    RmtStepSequencer::~RmtStepSequencer() {
        deinit();
    }

    Result<void> RmtStepSequencer::init(gpio_num_t theta_step, gpio_num_t rho_step) {
        if (initialized_) {
            return Result<void>::ok();
        }

        // Configure GPIO with maximum drive strength BEFORE RMT takes over
        // TMC2209 requires clean step edges - use strongest drive capability
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << theta_step) | (1ULL << rho_step),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_drive_capability(theta_step, GPIO_DRIVE_CAP_3);  // 40mA max drive
        gpio_set_drive_capability(rho_step, GPIO_DRIVE_CAP_3);    // 40mA max drive

        // Pair-completion semaphore. Counting: the ISR gives once per
        // completed buffer pair (plus stop()-gives), and execute()'s refill
        // loop may lag behind by a pair - a binary semaphore would merge
        // those signals and stall the loop.
        completion_sem_ = xSemaphoreCreateCounting(8, 0);
        if (!completion_sem_) {
            ESP_LOGE(TAG, "Failed to create completion semaphore");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure RMT TX channel for theta
        rmt_tx_channel_config_t theta_config = {
            .gpio_num = theta_step,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = HardwareConfig::RMT_RESOLUTION_HZ,  // 1 MHz (1 tick = 1us)
            .mem_block_symbols = 64,  // Hardware memory block
            .trans_queue_depth = 8,   // Extra depth for dual-channel queuing
            .intr_priority = 0,
            .flags = {
                .invert_out = false,
                .with_dma = false
            },
        };

        if (rmt_new_tx_channel(&theta_config, &theta_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create theta RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure RMT TX channel for rho
        rmt_tx_channel_config_t rho_config = theta_config;
        rho_config.gpio_num = rho_step;

        if (rmt_new_tx_channel(&rho_config, &rho_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create rho RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Create a simple copy encoder for raw symbols
        rmt_copy_encoder_config_t encoder_config = {};
        if (rmt_new_copy_encoder(&encoder_config, &copy_encoder_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create copy encoder");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Register TX done callback on BOTH channels to track completion of each
        rmt_tx_event_callbacks_t cbs = {
            .on_trans_done = tx_done_callback,
        };

        if (rmt_tx_register_event_callbacks(theta_channel_, &cbs, this) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register theta RMT callback");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        if (rmt_tx_register_event_callbacks(rho_channel_, &cbs, this) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register rho RMT callback");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Enable channels BEFORE creating sync manager
        if (rmt_enable(theta_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable theta RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        if (rmt_enable(rho_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable rho RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Allow RMT hardware to stabilize before first transmission
        // This helps prevent timing issues on the first segment (first move speed bug)
        vTaskDelay(pdMS_TO_TICKS(1));

        // Create sync manager AFTER channels are enabled
        rmt_channel_handle_t channels[] = { theta_channel_, rho_channel_ };
        rmt_sync_manager_config_t sync_config = {
            .tx_channel_array = channels,
            .array_size = 2,
        };

        if (rmt_new_sync_manager(&sync_config, &sync_manager_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT sync manager");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        initialized_ = true;
        return Result<void>::ok();
    }

    void RmtStepSequencer::deinit() {
        if (sync_manager_) {
            rmt_del_sync_manager(sync_manager_);
            sync_manager_ = nullptr;
        }
        if (theta_channel_) {
            rmt_disable(theta_channel_);
            rmt_del_channel(theta_channel_);
            theta_channel_ = nullptr;
        }
        if (rho_channel_) {
            rmt_disable(rho_channel_);
            rmt_del_channel(rho_channel_);
            rho_channel_ = nullptr;
        }
        if (copy_encoder_) {
            rmt_del_encoder(copy_encoder_);
            copy_encoder_ = nullptr;
        }
        if (completion_sem_) {
            vSemaphoreDelete(completion_sem_);
            completion_sem_ = nullptr;
        }
        initialized_ = false;
    }

    rmt_symbol_word_t RmtStepSequencer::make_step_symbol(uint16_t interval_us) {
        // At 1 MHz: 1 tick = 1us
        constexpr uint32_t TICKS_PER_US = HardwareConfig::RMT_RESOLUTION_HZ / 1000000;
        constexpr uint32_t PULSE_TICKS = HardwareConfig::MIN_STEP_PULSE_US * TICKS_PER_US;
        constexpr uint32_t MAX_DURATION = 32767;  // 15-bit max for RMT symbol duration

        // Widen to uint32_t before scaling so the product can't wrap uint16_t.
        uint32_t total_ticks = static_cast<uint32_t>(interval_us) * TICKS_PER_US;

        // Clamp total ticks to max representable value
        if (total_ticks > MAX_DURATION + PULSE_TICKS) {
            total_ticks = MAX_DURATION + PULSE_TICKS;
        }

        uint16_t delay_ticks = (total_ticks > PULSE_TICKS)
            ? static_cast<uint16_t>(total_ticks - PULSE_TICKS)
            : 1;

        rmt_symbol_word_t symbol = {
            .duration0 = static_cast<uint16_t>(PULSE_TICKS),  // HIGH for step pulse
            .level0 = 1,
            .duration1 = delay_ticks,  // LOW for remaining interval
            .level1 = 0,
        };
        return symbol;
    }

    rmt_symbol_word_t RmtStepSequencer::make_idle_symbol(uint16_t interval_us) {
        // No step pulse - just maintain timing (must use same structure as step symbol)
        constexpr uint32_t TICKS_PER_US = HardwareConfig::RMT_RESOLUTION_HZ / 1000000;
        constexpr uint32_t PULSE_TICKS = HardwareConfig::MIN_STEP_PULSE_US * TICKS_PER_US;
        constexpr uint32_t MAX_DURATION = 32767;  // 15-bit max for RMT symbol duration

        // Widen to uint32_t before scaling so the product can't wrap uint16_t.
        uint32_t total_ticks = static_cast<uint32_t>(interval_us) * TICKS_PER_US;

        // Clamp total ticks to max representable value
        if (total_ticks > MAX_DURATION + PULSE_TICKS) {
            total_ticks = MAX_DURATION + PULSE_TICKS;
        }

        uint16_t delay_ticks = (total_ticks > PULSE_TICKS)
            ? static_cast<uint16_t>(total_ticks - PULSE_TICKS)
            : 1;

        // Use same timing structure as step symbol, but keep output LOW
        // (duration0 for "pulse" period, duration1 for delay - both LOW)
        rmt_symbol_word_t symbol = {
            .duration0 = static_cast<uint16_t>(PULSE_TICKS),   // Same timing as step pulse
            .level0 = 0,                // But keep LOW (no pulse)
            .duration1 = delay_ticks,   // Delay period
            .level1 = 0,                // LOW
        };
        return symbol;
    }

    void RmtStepSequencer::bresenham_step(bool& step_theta, bool& step_rho) {
        step_theta = false;
        step_rho = false;

        if (bresenham_->theta_remaining == 0 && bresenham_->rho_remaining == 0) {
            return;
        }

        // Standard Bresenham algorithm for coordinated motion
        // Use TOTALS (fixed values) for error updates, not remaining counts
        if (bresenham_->theta_total >= bresenham_->rho_total) {
            // Theta is major axis - always step theta when remaining > 0
            step_theta = (bresenham_->theta_remaining > 0);
            bresenham_->error -= bresenham_->rho_total;  // Use fixed total
            if (bresenham_->error < 0) {
                bresenham_->error += bresenham_->theta_total;  // Use fixed total
                step_rho = (bresenham_->rho_remaining > 0);
            }
        }
        else {
            // Rho is major axis - always step rho when remaining > 0
            step_rho = (bresenham_->rho_remaining > 0);
            bresenham_->error -= bresenham_->theta_total;  // Use fixed total
            if (bresenham_->error < 0) {
                bresenham_->error += bresenham_->rho_total;  // Use fixed total
                step_theta = (bresenham_->theta_remaining > 0);
            }
        }

        // Update remaining counts
        if (step_theta) {
            bresenham_->theta_remaining--;
        }
        if (step_rho) {
            bresenham_->rho_remaining--;
        }
    }

    void RmtStepSequencer::update_speed_scale() {
        float target = 1.0f;
        if (live_feedrate_ && speed_reference_rpm_ > 0.0f) {
            const float live = live_feedrate_->load(std::memory_order_relaxed);
            if (live > 0.0f) {
                target = live / speed_reference_rpm_;
            }
        }
        // Sanity clamp (feedrate range is 1-20, so at most 20x either way),
        // then the per-segment ceiling (theta rotation-rate safety cap).
        target = std::max(0.05f, std::min({ target, 20.0f, speed_scale_ceiling_ }));

        // Slew toward the target by at most 25% per chunk so a large
        // feedrate change ramps over several chunks instead of stepping
        // the motors instantaneously.
        constexpr float kMaxRatioPerChunk = 1.25f;
        if (target > applied_speed_scale_ * kMaxRatioPerChunk) {
            applied_speed_scale_ *= kMaxRatioPerChunk;
        }
        else if (target < applied_speed_scale_ / kMaxRatioPerChunk) {
            applied_speed_scale_ /= kMaxRatioPerChunk;
        }
        else {
            applied_speed_scale_ = target;
        }
    }

    uint16_t RmtStepSequencer::scale_interval(uint16_t base_interval_us) const {
        if (applied_speed_scale_ == 1.0f) {
            return base_interval_us;
        }
        constexpr float kMinIntervalUs =
            1000000.0f / static_cast<float>(HardwareConfig::MAX_STEP_RATE_HZ);
        constexpr float kMaxIntervalUs = 32000.0f;

        float scaled = static_cast<float>(base_interval_us) / applied_speed_scale_;
        scaled = std::max(kMinIntervalUs, std::min(scaled, kMaxIntervalUs));
        return static_cast<uint16_t>(scaled + 0.5f);
    }

    void RmtStepSequencer::encode_chunk(uint8_t buffer_idx) {
        auto& theta_buf = theta_symbols_[buffer_idx];
        auto& rho_buf = rho_symbols_[buffer_idx];
        size_t count = 0;

        int32_t theta_steps_encoded = 0;
        int32_t rho_steps_encoded = 0;

        // Live feedrate: re-read the override once per chunk, so a speed
        // change lands within ~64 steps even mid-segment.
        update_speed_scale();

        for (size_t i = 0; i < kChunkSize && steps_encoded_ < total_steps_; i++) {
            uint16_t interval = scale_interval(intervals_[steps_encoded_]);
            bool step_theta, step_rho;

            bresenham_step(step_theta, step_rho);

            theta_buf[i] = step_theta ? make_step_symbol(interval) : make_idle_symbol(interval);
            rho_buf[i] = step_rho ? make_step_symbol(interval) : make_idle_symbol(interval);

            if (step_theta) theta_steps_encoded++;
            if (step_rho) rho_steps_encoded++;

            steps_encoded_++;
            count++;
        }

        symbol_counts_[buffer_idx] = count;

        // Record this buffer's signed step deltas. They are credited to the
        // position counters only when the buffer's transmission COMPLETES
        // (see tx_done_callback) — crediting at encode time ran ~2 chunks
        // ahead of the motors and desynced position on every aborted segment.
        buf_theta_delta_[buffer_idx] = theta_steps_encoded * bresenham_->theta_dir;
        buf_rho_delta_[buffer_idx] = rho_steps_encoded * bresenham_->rho_dir;
    }

    bool IRAM_ATTR RmtStepSequencer::tx_done_callback(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t* edata,
        void* user_ctx)
    {
        auto* self = static_cast<RmtStepSequencer*>(user_ctx);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Check for stop request
        // Using __atomic_* builtins instead of std::atomic methods for ISR safety.
        // Only wake execute() - it owns the abort and clears executing_;
        // clearing it here would let another execute() claim the sequencer
        // while this one is still mid-abort.
        if (__atomic_load_n(reinterpret_cast<bool*>(&self->stop_requested_), __ATOMIC_ACQUIRE)) {
            xSemaphoreGiveFromISR(self->completion_sem_, &xHigherPriorityTaskWoken);
            return xHigherPriorityTaskWoken == pdTRUE;
        }

        // Atomically increment channel completion counter
        // fetch_add returns the value BEFORE increment, so:
        // - First callback sees 0, increments to 1, returns early
        // - Second callback sees 1, increments to 2, proceeds
        int prev_count = __atomic_fetch_add(reinterpret_cast<int*>(&self->channels_done_), 1, __ATOMIC_ACQ_REL);
        if (prev_count < 1) {
            // First channel done, wait for the other
            return false;
        }

        // Both channels done - reset counter for next round
        __atomic_store_n(reinterpret_cast<int*>(&self->channels_done_), 0, __ATOMIC_RELEASE);

        // Decrement pending counter - this transmission pair just completed
        __atomic_fetch_sub(reinterpret_cast<int32_t*>(&self->pending_tx_), 1, __ATOMIC_ACQ_REL);

        // Buffers transmit in strict 0,1,0,1... order, so the pair that just
        // completed is completed_pairs_ & 1. Credit its steps to the position
        // counters NOW (transmission finished => the motors physically moved),
        // before encode_chunk below overwrites the buffer's deltas.
        // __atomic_* builtins keep this inlined and ISR-safe.
        const uint8_t done_buffer = static_cast<uint8_t>(self->completed_pairs_ & 1);
        self->completed_pairs_++;
        if (self->theta_pos_ && self->buf_theta_delta_[done_buffer] != 0) {
            __atomic_fetch_add(reinterpret_cast<int32_t*>(self->theta_pos_),
                self->buf_theta_delta_[done_buffer], __ATOMIC_RELEASE);
        }
        if (self->rho_pos_ && self->buf_rho_delta_[done_buffer] != 0) {
            __atomic_fetch_add(reinterpret_cast<int32_t*>(self->rho_pos_),
                self->buf_rho_delta_[done_buffer], __ATOMIC_RELEASE);
        }

        // Wake execute()'s refill loop: it re-encodes the freed buffer and
        // queues the next transmission from task context. rmt_transmit()
        // uses non-ISR FreeRTOS queue APIs internally, so calling it from
        // this callback (the old design) corrupted the driver's transaction
        // queues.
        xSemaphoreGiveFromISR(self->completion_sem_, &xHigherPriorityTaskWoken);

        return xHigherPriorityTaskWoken == pdTRUE;
    }

    Result<void> RmtStepSequencer::execute(
        BresenhamState& bresenham,
        const uint16_t* intervals,
        uint32_t total_steps,
        std::atomic<int32_t>& theta_pos,
        std::atomic<int32_t>& rho_pos,
        uint32_t timeout_ms)
    {
        if (!initialized_) {
            return Result<void>::err(MotionError::InvalidState);
        }

        if (total_steps == 0) {
            return Result<void>::ok();
        }

        // Claim execution atomically. The old load-then-store let two tasks
        // (stepper task segment + homing from a console/API task) both pass
        // the check and drive the same buffers and RMT channels.
        bool expected = false;
        if (!executing_.compare_exchange_strong(expected, true,
                std::memory_order_acq_rel)) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // A stop raised since the caller's clear_stop() aborts this motion
        // too - it may have landed between two chunked execute() calls,
        // where the old per-call flag reset silently discarded it.
        if (stop_requested_.load(std::memory_order_acquire)) {
            executing_.store(false, std::memory_order_release);
            return Result<void>::err(MotionError::EmergencyStop);
        }

        // Setup execution state
        bresenham_ = &bresenham;
        intervals_ = intervals;
        total_steps_ = total_steps;
        theta_pos_ = &theta_pos;
        rho_pos_ = &rho_pos;
        steps_encoded_ = 0;
        encoded_chunks_ = 0;
        completed_pairs_ = 0;
        buf_theta_delta_[0] = buf_theta_delta_[1] = 0;
        buf_rho_delta_[0] = buf_rho_delta_[1] = 0;
        pending_tx_.store(0, std::memory_order_release);
        channels_done_.store(0, std::memory_order_release);

        // Drain stale completion signals from a previous segment
        while (xSemaphoreTake(completion_sem_, 0) == pdTRUE) {}

        // Re-arm the sync manager so this segment's first theta/rho pair
        // starts simultaneously (sync only applies to the first transmission
        // after creation/reset).
        if (sync_manager_) {
            rmt_sync_reset(sync_manager_);
        }

        // Encode first chunk into buffer 0
        encode_chunk(0);
        encoded_chunks_ = 1;

        if (symbol_counts_[0] == 0) {
            executing_.store(false, std::memory_order_release);
            return Result<void>::ok();
        }

        // Pre-encode second chunk for ping-pong buffering
        symbol_counts_[1] = 0;
        if (steps_encoded_ < total_steps_) {
            encode_chunk(1);
            encoded_chunks_ = 2;
        }

        // Start transmission - track pending count
        pending_tx_.fetch_add(1, std::memory_order_release);

        rmt_transmit_config_t tx_config = {
            .loop_count = 0,
            .flags = {
                .eot_level = 0,
                .queue_nonblocking = false,  // Block on first transmit to ensure sync
            },
        };

        esp_err_t err = rmt_transmit(theta_channel_, copy_encoder_,
            theta_symbols_[0],
            symbol_counts_[0] * sizeof(rmt_symbol_word_t),
            &tx_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start theta transmission: %d", err);
            pending_tx_.store(0, std::memory_order_release);
            executing_.store(false, std::memory_order_release);
            return Result<void>::err(MotionError::HardwareFault);
        }

        err = rmt_transmit(rho_channel_, copy_encoder_,
            rho_symbols_[0],
            symbol_counts_[0] * sizeof(rmt_symbol_word_t),
            &tx_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start rho transmission: %d", err);
            pending_tx_.store(0, std::memory_order_release);
            rmt_tx_wait_all_done(theta_channel_, 100);
            executing_.store(false, std::memory_order_release);
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Queue second chunk if available
        if (symbol_counts_[1] > 0) {
            pending_tx_.fetch_add(1, std::memory_order_release);
            tx_config.flags.queue_nonblocking = true;
            rmt_transmit(theta_channel_, copy_encoder_,
                theta_symbols_[1],
                symbol_counts_[1] * sizeof(rmt_symbol_word_t),
                &tx_config);
            rmt_transmit(rho_channel_, copy_encoder_,
                rho_symbols_[1],
                symbol_counts_[1] * sizeof(rmt_symbol_word_t),
                &tx_config);
        }

        ESP_LOGD(TAG, "RMT transmission started: %lu total steps", total_steps);

        // Refill loop: the TX-done ISR gives completion_sem_ once per
        // completed buffer pair (after crediting its steps); THIS task then
        // re-encodes the freed buffer and queues the next chunk.
        // rmt_transmit() must run in task context — it uses non-ISR queue
        // APIs, and calling it from the callback (the old design) corrupted
        // the RMT driver's transaction bookkeeping.
        //
        // Timeout is per pair-completion, not per segment: live feedrate
        // scaling can stretch a segment far past any duration computed at
        // plan time, but a single 64-symbol pair is bounded by
        // 64 x 32.767ms ~= 2.1s even at the slowest interval. No pair
        // completing within the window means the hardware is wedged.
        (void)timeout_ms;  // Superseded by per-pair progress timeout
        constexpr TickType_t kPairTimeout = pdMS_TO_TICKS(6000);
        for (;;) {
            if (xSemaphoreTake(completion_sem_, kPairTimeout) != pdTRUE) {
                ESP_LOGE(TAG, "RMT stalled: no chunk completion within 6s");
                stop_requested_.store(true, std::memory_order_release);
                hard_abort_channels();
                return Result<void>::err(MotionError::Timeout);
            }

            // Stop request (emergency stop / hall / stall ISR). The flag is
            // only raised elsewhere — the RMT hardware may still be draining
            // queued symbols, so abort it HERE, in the executing task's
            // context (RMT channel APIs must not be called from ISRs or
            // raced across tasks). Without this, motors kept stepping up to
            // two chunks (~128 steps) after an "emergency stop" returned.
            if (stop_requested_.load(std::memory_order_acquire)) {
                hard_abort_channels();
                return Result<void>::err(MotionError::EmergencyStop);
            }

            // One pair completed: its buffer is free. Buffers alternate
            // strictly, so chunk N lives in buffer N&1.
            if (steps_encoded_ < total_steps_) {
                const uint8_t buf = static_cast<uint8_t>(encoded_chunks_ & 1);
                encode_chunk(buf);
                if (symbol_counts_[buf] > 0) {
                    encoded_chunks_++;
                    pending_tx_.fetch_add(1, std::memory_order_release);

                    rmt_transmit_config_t refill_config = {
                        .loop_count = 0,
                        .flags = {
                            .eot_level = 0,
                            .queue_nonblocking = true,
                        },
                    };
                    rmt_transmit(theta_channel_, copy_encoder_,
                        theta_symbols_[buf],
                        symbol_counts_[buf] * sizeof(rmt_symbol_word_t),
                        &refill_config);
                    rmt_transmit(rho_channel_, copy_encoder_,
                        rho_symbols_[buf],
                        symbol_counts_[buf] * sizeof(rmt_symbol_word_t),
                        &refill_config);
                }
            }

            // Done when everything encoded has finished transmitting
            if (pending_tx_.load(std::memory_order_acquire) == 0 &&
                steps_encoded_ >= total_steps_) {
                break;
            }
        }

        executing_.store(false, std::memory_order_release);
        return Result<void>::ok();
    }

    void RmtStepSequencer::hard_abort_channels() {
        // disable/enable resets the channel: aborts the in-flight
        // transmission and flushes the queue. Task context only.
        if (theta_channel_) {
            rmt_disable(theta_channel_);
            rmt_enable(theta_channel_);
        }
        if (rho_channel_) {
            rmt_disable(rho_channel_);
            rmt_enable(rho_channel_);
        }
        pending_tx_.store(0, std::memory_order_release);
        executing_.store(false, std::memory_order_release);
        // Position note: steps from the partially-transmitted chunk were
        // emitted but not credited (crediting is completion-based), so the
        // logical position may lag the physical by < kChunkSize steps after
        // an abort. This is the bounded residual error of an abort.
    }

    void RmtStepSequencer::stop() {
        // Raise the flag and wake execute(); the executing task performs the
        // actual hardware abort (see execute()). Safe to call from any task.
        stop_requested_.store(true, std::memory_order_release);

        if (completion_sem_) {
            xSemaphoreGive(completion_sem_);
        }
    }

    void RmtStepSequencer::stop_from_isr() {
        // ISR-safe version - only set flag and signal semaphore
        // No blocking calls allowed
        stop_requested_.store(true, std::memory_order_release);

        if (completion_sem_) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    // =============================================================================
    // CoordinatedStepperController Implementation
    // =============================================================================

    CoordinatedStepperController::CoordinatedStepperController(
        StepperDriver& theta,
        StepperDriver& rho)
        : theta_(theta)
        , rho_(rho)
    {
    }

    CoordinatedStepperController::~CoordinatedStepperController() {
        if (rmt_sequencer_) {
            rmt_sequencer_->deinit();
        }
        if (interval_table_.intervals) {
            heap_caps_free(interval_table_.intervals);
            interval_table_.intervals = nullptr;
        }
    }

    Result<void> CoordinatedStepperController::init() {
        // Allocate interval table from INTERNAL RAM. The RMT TX-done ISR
        // reads this table (encode_chunk); SPIRAM access from an IRAM ISR
        // crashes whenever the flash cache is disabled (NVS commits, OTA).
        // At MAX_SEGMENT_STEPS=8192 this is 16KB.
        interval_table_.intervals = static_cast<uint16_t*>(
            heap_caps_calloc(kMaxIntervalsPerSegment, sizeof(uint16_t),
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
        );
        if (!interval_table_.intervals) {
            ESP_LOGE(TAG, "Failed to allocate interval table from internal RAM");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Allocate RMT sequencer in internal RAM (required for ISR callback safety)
        void* mem = malloc(sizeof(RmtStepSequencer));
        if (!mem) {
            ESP_LOGE(TAG, "Failed to allocate RMT sequencer in internal RAM");
            heap_caps_free(interval_table_.intervals);
            interval_table_.intervals = nullptr;
            return Result<void>::err(MotionError::HardwareFault);
        }
        rmt_sequencer_.reset(new (mem) RmtStepSequencer());

        auto result = rmt_sequencer_->init(theta_.step_pin(), rho_.step_pin());
        if (result.is_err()) {
            ESP_LOGE(TAG, "Failed to initialize RMT step sequencer");
            return result;
        }
        return Result<void>::ok();
    }

    void CoordinatedStepperController::enable() {
        // Already on: nothing to warm up. move_to() calls this per move —
        // an unconditional 250ms sleep here throttled pattern feeding to
        // ~4 lines/sec.
        if (theta_.is_enabled() && rho_.is_enabled()) {
            return;
        }

        theta_.set_enabled(true);
        rho_.set_enabled(true);

        // 250ms warmup delay after enabling motors
        // TMC drivers need time to stabilize after enable
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    void CoordinatedStepperController::disable() {
        theta_.set_enabled(false);
        rho_.set_enabled(false);
    }

    void CoordinatedStepperController::emergency_stop() {
        // Only raise the flag. Clearing bresenham_ here raced the executing
        // task's encode loop (execute() sets it up fresh per motion anyway).
        rmt_sequencer_->stop();
    }

    void CoordinatedStepperController::emergency_stop_from_isr() {
        rmt_sequencer_->stop_from_isr();
        // Don't clear bresenham from ISR - not critical
    }

    VelocityProfile CoordinatedStepperController::calculate_velocity_profile(
        const MotionSegment& segment,
        float steps_per_unit) const
    {
        VelocityProfile profile;

        // Calculate total steps - use max to match Bresenham's major axis count
        const uint32_t total_steps = static_cast<uint32_t>(
            std::max(std::abs(segment.delta_theta_steps), std::abs(segment.delta_rho_steps))
            );

        if (total_steps == 0) {
            return profile;
        }

        // Hardware limits
        const float max_step_rate = static_cast<float>(HardwareConfig::MAX_STEP_RATE_HZ);
        const float min_step_rate = 31.0f;  // Limited by RMT symbol duration at 1 MHz

        // For coordinated motion, use velocity from motors that are actually moving
        // If both move, use minimum (so deceleration happens if either needs to slow)
        // If only one moves, use that motor's velocity
        const bool theta_moves = segment.delta_theta_steps != 0;
        const bool rho_moves = segment.delta_rho_steps != 0;

        float entry_rpm, exit_rpm;
        if (theta_moves && rho_moves) {
            // Both moving: use minimum so either motor can request deceleration
            entry_rpm = std::min(segment.theta_entry_velocity, segment.rho_entry_velocity);
            exit_rpm = std::min(segment.theta_exit_velocity, segment.rho_exit_velocity);
        }
        else if (theta_moves) {
            entry_rpm = segment.theta_entry_velocity;
            exit_rpm = segment.theta_exit_velocity;
        }
        else if (rho_moves) {
            entry_rpm = segment.rho_entry_velocity;
            exit_rpm = segment.rho_exit_velocity;
        }
        else {
            // Neither moves - shouldn't happen, but handle gracefully
            entry_rpm = 0.0f;
            exit_rpm = 0.0f;
        }

        // Safety cap applied AFTER feedrate: bound the physical theta spin
        // rate regardless of what path speed was requested. Path speed v
        // (units/min) over `distance` turns |delta_theta| in distance/v
        // minutes, so the rotation rate is v * rotations / distance — clamp
        // v so it never exceeds THETA_MAX_ROT_PER_MIN. Exact for either
        // major axis (the ratio theta_steps/distance is fixed per segment).
        float nominal_rpm = segment.nominal_velocity;
        if (theta_moves && segment.distance > 0.0f) {
            const float rotations = std::fabs(static_cast<float>(segment.delta_theta_rad)) /
                (2.0f * static_cast<float>(M_PI));
            if (rotations > 1e-6f) {
                const float v_cap = MotionConfig::THETA_MAX_ROT_PER_MIN *
                    segment.distance / rotations;
                entry_rpm = std::min(entry_rpm, v_cap);
                exit_rpm = std::min(exit_rpm, v_cap);
                nominal_rpm = std::min(nominal_rpm, v_cap);
            }
        }

        // Convert RPM to steps/s using the relationship:
        //   motion_time = distance / (rpm / 60)
        //   velocity_steps_s = total_steps / motion_time
        //                    = total_steps * (rpm / 60) / distance
        //                    = rpm * (total_steps / distance) / 60
        //                    = rpm * steps_per_unit / 60
        const float rpm_to_steps_s = steps_per_unit / 60.0f;

        float entry_steps_s = entry_rpm * rpm_to_steps_s;
        float exit_steps_s = exit_rpm * rpm_to_steps_s;
        float nominal_steps_s = nominal_rpm * rpm_to_steps_s;

        // Clamp velocities to hardware limits
        entry_steps_s = std::max(min_step_rate, std::min(entry_steps_s, max_step_rate));
        exit_steps_s = std::max(min_step_rate, std::min(exit_steps_s, max_step_rate));
        nominal_steps_s = std::max(min_step_rate, std::min(nominal_steps_s, max_step_rate));

        // Ensure minimum motion time based on max step rate
        const float min_motion_time = static_cast<float>(total_steps) / max_step_rate;
        const float nominal_motion_time = static_cast<float>(total_steps) / nominal_steps_s;
        if (nominal_motion_time < min_motion_time) {
            nominal_steps_s = max_step_rate;
        }

        // Acceleration in steps/s^2 (single tunable, see config.h)
        const float accel_steps_s2 = MotionConfig::STEP_ACCEL_STEPS_S2;

        // Calculate distances (in steps) for acceleration and deceleration
        // Using kinematic equation: v^2 = v0^2 + 2*a*d  =>  d = (v^2 - v0^2) / (2*a)
        float accel_dist = 0.0f;
        float decel_dist = 0.0f;

        if (nominal_steps_s > entry_steps_s) {
            accel_dist = (nominal_steps_s * nominal_steps_s - entry_steps_s * entry_steps_s) /
                (2.0f * accel_steps_s2);
        }
        if (nominal_steps_s > exit_steps_s) {
            decel_dist = (nominal_steps_s * nominal_steps_s - exit_steps_s * exit_steps_s) /
                (2.0f * accel_steps_s2);
        }

        // Check if we can reach cruise velocity (trapezoidal profile)
        // or if we need a triangle profile
        const float total_dist = static_cast<float>(total_steps);

        if (accel_dist + decel_dist <= total_dist) {
            // Trapezoidal profile: we reach cruise velocity
            profile.accel_steps = static_cast<uint32_t>(accel_dist);
            profile.decel_steps = static_cast<uint32_t>(decel_dist);
            profile.cruise_steps = total_steps - profile.accel_steps - profile.decel_steps;
            profile.cruise_velocity = nominal_steps_s;
        }
        else {
            // Triangle profile: can't reach cruise velocity
            // Find peak velocity: v_peak^2 = (v_entry^2 + v_exit^2 + 2*a*d) / 2
            float v_peak_sq = (entry_steps_s * entry_steps_s +
                exit_steps_s * exit_steps_s +
                2.0f * accel_steps_s2 * total_dist) / 2.0f;

            if (v_peak_sq < min_step_rate * min_step_rate) {
                v_peak_sq = min_step_rate * min_step_rate;
            }

            float v_peak = std::sqrt(v_peak_sq);
            v_peak = std::min(v_peak, max_step_rate);

            // Recalculate accel/decel distances with peak velocity
            accel_dist = (v_peak * v_peak - entry_steps_s * entry_steps_s) / (2.0f * accel_steps_s2);
            decel_dist = total_dist - accel_dist;

            profile.accel_steps = static_cast<uint32_t>(std::max(0.0f, accel_dist));
            profile.decel_steps = total_steps - profile.accel_steps;
            profile.cruise_steps = 0;
            profile.cruise_velocity = v_peak;
        }

        profile.entry_velocity = entry_steps_s;
        profile.exit_velocity = exit_steps_s;
        profile.acceleration = accel_steps_s2;

        return profile;
    }

    Result<void> CoordinatedStepperController::execute_segment(
        const MotionSegment& segment)
    {
        if (rmt_sequencer_->is_executing()) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Calculate total steps for Bresenham (major axis)
        const uint32_t total_steps = static_cast<uint32_t>(
            std::max(std::abs(segment.delta_theta_steps), std::abs(segment.delta_rho_steps)));

        // No steps to execute
        if (total_steps == 0) {
            return Result<void>::ok();
        }

        // Reject oversized segments outright. Truncating silently (the old
        // behavior) executed only part of the delta while the planner
        // believed all of it ran — permanent position drift. The path
        // planner splits moves below this cap; hitting this is a bug there.
        if (total_steps > kMaxIntervalsPerSegment) {
            ESP_LOGE(TAG, "Segment exceeds step cap: %lu > %u (theta=%ld, rho=%ld) - rejected",
                total_steps, static_cast<unsigned>(kMaxIntervalsPerSegment),
                segment.delta_theta_steps, segment.delta_rho_steps);
            return Result<void>::err(MotionError::OutOfBounds);
        }

        // Set directions
        theta_.set_direction(segment.delta_theta_steps >= 0);
        rho_.set_direction(segment.delta_rho_steps >= 0);

        // Setup Bresenham state
        bresenham_.theta_remaining = std::abs(segment.delta_theta_steps);
        bresenham_.rho_remaining = std::abs(segment.delta_rho_steps);
        bresenham_.theta_total = bresenham_.theta_remaining;  // Store original totals
        bresenham_.rho_total = bresenham_.rho_remaining;      // for fixed Bresenham error updates
        bresenham_.theta_dir = (segment.delta_theta_steps >= 0) ? 1 : -1;
        bresenham_.rho_dir = (segment.delta_rho_steps >= 0) ? 1 : -1;

        // Initialize Bresenham error term - consistent initialization based on major axis
        const int32_t major = std::max(bresenham_.theta_total, bresenham_.rho_total);
        bresenham_.error = major / 2;

        // Calculate velocity profile
        const float steps_per_unit = (segment.distance > 0.0f)
            ? static_cast<float>(total_steps) / segment.distance
            : static_cast<float>(total_steps);
        const VelocityProfile profile = calculate_velocity_profile(segment, steps_per_unit);

        // Pre-compute interval table (all FPU operations happen here, in task context)
        prepare_interval_table(profile, total_steps);

        // Debug: Validate first interval (for first move speed issue)
        constexpr uint32_t min_interval_us = 1000000 / HardwareConfig::MAX_STEP_RATE_HZ;  // 20us
        if (interval_table_.intervals[0] < min_interval_us) {
            ESP_LOGW(TAG, "First interval too fast: %u us (min: %lu us)",
                interval_table_.intervals[0], min_interval_us);
        }

        ESP_LOGD(TAG, "RMT segment: theta=%ld, rho=%ld, total=%lu, cruise_v=%.1f, interval[0]=%u us",
            segment.delta_theta_steps, segment.delta_rho_steps,
            total_steps, profile.cruise_velocity, interval_table_.intervals[0]);

        // Timeout: the interval table's summed duration with 2x margin plus a
        // fixed floor. A fixed 30s timeout used to kill long slow segments
        // (a max-length segment at minimum speed legitimately runs minutes).
        const uint32_t expected_ms = static_cast<uint32_t>(interval_table_.total_us / 1000);
        const uint32_t timeout_ms = expected_ms * 2 + 2000;

        // Live feedrate scaling reference: chunk intervals execute at
        // (live_feedrate / base_feedrate) x the planned speed, so feedrate
        // changes reach this segment even while it is running. The scale is
        // ceilinged so a live boost can't push the segment past the theta
        // rotation-rate cap the profile was clamped to (actual speed =
        // planned_nominal x scale, so scale may grow only up to v_cap /
        // planned_nominal; 1.0 when the cap already binds the plan).
        float scale_ceiling = 20.0f;  // matches update_speed_scale's sanity clamp
        if (segment.delta_theta_steps != 0 && segment.distance > 0.0f &&
            segment.nominal_velocity > 0.0f) {
            const float rotations = std::fabs(static_cast<float>(segment.delta_theta_rad)) /
                (2.0f * static_cast<float>(M_PI));
            if (rotations > 1e-6f) {
                const float v_cap = MotionConfig::THETA_MAX_ROT_PER_MIN *
                    segment.distance / rotations;
                scale_ceiling = std::max(1.0f,
                    v_cap / std::min(segment.nominal_velocity, v_cap));
            }
        }
        rmt_sequencer_->set_speed_scale_ceiling(scale_ceiling);
        rmt_sequencer_->set_speed_reference(segment.base_feedrate);

        // Each segment is one logical motion op: consume any stop left over
        // from a previous abort, then execute (a stop raised from here on
        // aborts this segment).
        rmt_sequencer_->clear_stop();

        // Execute via RMT sequencer (blocking)
        auto result = rmt_sequencer_->execute(
            bresenham_,
            interval_table_.intervals,
            interval_table_.total_steps,
            theta_.position_,
            rho_.position_,
            timeout_ms
        );

        return result;
    }

    Result<void> CoordinatedStepperController::execute_constant_speed(
        int32_t theta_steps,
        int32_t rho_steps,
        uint32_t interval_us)
    {
        if (rmt_sequencer_->is_executing()) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Calculate total steps for Bresenham (major axis)
        const uint32_t total_steps = static_cast<uint32_t>(
            std::max(std::abs(theta_steps), std::abs(rho_steps)));

        // No steps to execute
        if (total_steps == 0) {
            return Result<void>::ok();
        }

        // Set directions
        theta_.set_direction(theta_steps >= 0);
        rho_.set_direction(rho_steps >= 0);

        // Setup Bresenham state (persists across chunks below)
        bresenham_.theta_remaining = std::abs(theta_steps);
        bresenham_.rho_remaining = std::abs(rho_steps);
        bresenham_.theta_total = bresenham_.theta_remaining;
        bresenham_.rho_total = bresenham_.rho_remaining;
        bresenham_.theta_dir = (theta_steps >= 0) ? 1 : -1;
        bresenham_.rho_dir = (rho_steps >= 0) ? 1 : -1;

        // Initialize Bresenham error term
        const int32_t major = std::max(bresenham_.theta_total, bresenham_.rho_total);
        bresenham_.error = major / 2;

        // Clamp interval to valid range for RMT
        constexpr uint32_t min_interval_us = 1000000 / HardwareConfig::MAX_STEP_RATE_HZ;  // 20µs
        constexpr uint32_t max_interval_us = 32000;  // ~31 steps/s at 1 MHz RMT clock
        uint32_t clamped_interval = interval_us;
        if (clamped_interval < min_interval_us) clamped_interval = min_interval_us;
        if (clamped_interval > max_interval_us) clamped_interval = max_interval_us;

        ESP_LOGI(TAG, "Constant-speed: theta=%ld, rho=%ld, interval=%lu us (%lu steps/s)",
            theta_steps, rho_steps, clamped_interval, 1000000UL / clamped_interval);

        // Fill interval table once with the constant value (reused per chunk)
        interval_table_.clear();
        const uint32_t table_fill = std::min(total_steps,
            static_cast<uint32_t>(kMaxIntervalsPerSegment));
        for (uint32_t i = 0; i < table_fill; ++i) {
            interval_table_.intervals[i] = static_cast<uint16_t>(clamped_interval);
        }

        // Homing moves (e.g. kMaxHomingSteps=100000) exceed the per-execute
        // table capacity, which used to be SILENTLY truncated to 32768 steps.
        // Execute in chunks instead; the shared Bresenham state carries the
        // axis coordination across chunk boundaries, and an ISR-requested
        // stop (hall/stall) aborts the loop via the EmergencyStop result.
        //
        // Constant-speed moves (homing seeks) are exempt from live feedrate
        // scaling - their speeds are chosen for StallGuard reliability.
        // (The theta-cap scale ceiling is moot with scaling off; reset it so
        // it can't leak from a previous capped segment.)
        rmt_sequencer_->set_speed_scale_ceiling(20.0f);
        rmt_sequencer_->set_speed_reference(0.0f);

        // Consume stops ONCE for the whole seek, not per chunk: the stop
        // flag persists across execute() calls, so a hall/stall ISR that
        // fires in the gap between two chunks aborts the next chunk instead
        // of being silently discarded (which drove the axis into the hard
        // stop for up to another full chunk).
        rmt_sequencer_->clear_stop();

        uint32_t executed = 0;
        while (executed < total_steps) {
            const uint32_t chunk = std::min(total_steps - executed,
                static_cast<uint32_t>(kMaxIntervalsPerSegment));

            const uint32_t expected_ms = static_cast<uint32_t>(
                (static_cast<uint64_t>(chunk) * clamped_interval) / 1000);
            const uint32_t timeout_ms = expected_ms * 2 + 2000;

            auto result = rmt_sequencer_->execute(
                bresenham_,
                interval_table_.intervals,
                chunk,
                theta_.position_,
                rho_.position_,
                timeout_ms
            );
            if (result.is_err()) {
                return result;
            }
            executed += chunk;
        }

        return Result<void>::ok();
    }

    void CoordinatedStepperController::prepare_interval_table(
        const VelocityProfile& profile,
        uint32_t total_steps)
    {
        interval_table_.clear();
        interval_table_.total_steps = std::min(total_steps, static_cast<uint32_t>(kMaxIntervalsPerSegment));

        // RMT symbol max duration: 32767 ticks @ 1MHz = 32767µs
        // Minimum velocity = 1,000,000 / 32767 ≈ 30.5 steps/s
        // Use 31 steps/s for margin
        const float min_velocity = 31.0f;  // steps/s (limited by RMT symbol max duration at 1 MHz)
        const float max_velocity = static_cast<float>(HardwareConfig::MAX_STEP_RATE_HZ);

        for (uint32_t step = 0; step < interval_table_.total_steps; ++step) {
            float velocity;

            if (step < profile.accel_steps) {
                // Acceleration phase: v = sqrt(v_entry^2 + 2*a*s)
                velocity = std::sqrt(
                    profile.entry_velocity * profile.entry_velocity +
                    2.0f * profile.acceleration * static_cast<float>(step)
                );
            }
            else if (step < profile.accel_steps + profile.cruise_steps) {
                // Cruise phase
                velocity = profile.cruise_velocity;
            }
            else {
                // Deceleration phase
                const uint32_t decel_steps_taken = step - profile.accel_steps - profile.cruise_steps;
                const uint32_t decel_steps_remaining = (profile.decel_steps > decel_steps_taken)
                    ? profile.decel_steps - decel_steps_taken
                    : 0;
                velocity = std::sqrt(
                    profile.exit_velocity * profile.exit_velocity +
                    2.0f * profile.acceleration * static_cast<float>(decel_steps_remaining)
                );
            }

            // Clamp velocity to range that RMT can represent accurately
            velocity = std::max(min_velocity, std::min(velocity, max_velocity));

            // Convert velocity to interval in MICROSECONDS (for RMT)
            // interval_us = 1,000,000 / velocity
            uint32_t interval_us = static_cast<uint32_t>(1000000.0f / velocity);

            // Clamp interval to safe range
            // Minimum: 20µs (50kHz max step rate)
            // Maximum: 32000µs (~31 steps/s, fits in RMT symbol at 1 MHz)
            constexpr uint32_t min_interval_us = 1000000 / HardwareConfig::MAX_STEP_RATE_HZ;
            constexpr uint32_t max_interval_us = 32000;  // ~31 steps/s, fits in single RMT symbol at 1 MHz
            if (interval_us < min_interval_us) {
                interval_us = min_interval_us;
            }
            if (interval_us > max_interval_us) {
                interval_us = max_interval_us;
            }

            interval_table_.intervals[step] = static_cast<uint16_t>(interval_us);
            interval_table_.total_us += interval_us;
        }
    }

} // namespace sand_table
