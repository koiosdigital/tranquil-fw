#include "motion_controller.h"
#include "stepper_driver.h"
#include "homing_controller.h"
#include "coordinate_transformer.h"
#include "path_planner.h"
#include "velocity_planner.h"

#include "esp_log.h"

#include <cmath>

namespace sand_table {

    static const char* TAG = "MotionController";

    MotionController::MotionController()
        : path_planner_(std::make_unique<PathPlanner>())
        , velocity_planner_(std::make_unique<VelocityPlanner>())
        , transformer_(std::make_unique<CoordinateTransformer>())
    {
        planner_mutex_ = xSemaphoreCreateMutex();
        motor_power_mutex_ = xSemaphoreCreateMutex();

        // Set up PathPlanner callback to enqueue segments via VelocityPlanner
        path_planner_->set_segment_callback([this](MotionSegment& seg) {
            return this->enqueue_segment(seg);
            });
    }

    MotionController::~MotionController() {
        stop();

        if (inactivity_timer_) {
            xTimerDelete(inactivity_timer_, 0);
        }
        if (planner_mutex_) {
            vSemaphoreDelete(planner_mutex_);
            planner_mutex_ = nullptr;
        }
        if (motor_power_mutex_) {
            vSemaphoreDelete(motor_power_mutex_);
            motor_power_mutex_ = nullptr;
        }
    }

    Result<void> MotionController::init() {
        // Initialize TMC UART bus
        auto tmc_result = init_tmc();
        if (tmc_result.is_err()) {
            return tmc_result;
        }

        // Create stepper drivers
        theta_stepper_ = std::make_unique<StepperDriver>(
            StepperDriver::Pins{
                PinConfig::THETA_STEP,
                PinConfig::THETA_DIR,
                PinConfig::THETA_ENABLE
            },
            "Theta"
        );

        rho_stepper_ = std::make_unique<StepperDriver>(
            StepperDriver::Pins{
                PinConfig::RHO_STEP,
                PinConfig::RHO_DIR,
                PinConfig::RHO_ENABLE
            },
            "Rho"
        );

        // Initialize stepper drivers
        auto theta_result = theta_stepper_->init();
        if (theta_result.is_err()) {
            ESP_LOGE(TAG, "Failed to initialize theta stepper");
            return theta_result;
        }

        auto rho_result = rho_stepper_->init();
        if (rho_result.is_err()) {
            ESP_LOGE(TAG, "Failed to initialize rho stepper");
            return rho_result;
        }

        // Create coordinated stepper controller
        stepper_controller_ = std::make_unique<CoordinatedStepperController>(
            *theta_stepper_, *rho_stepper_
        );

        auto controller_result = stepper_controller_->init();
        if (controller_result.is_err()) {
            ESP_LOGE(TAG, "Failed to initialize stepper controller");
            return controller_result;
        }

        // Feed the live feedrate override down to the RMT chunk encoder so
        // speed changes apply to in-flight motion, not just new segments.
        stepper_controller_->set_live_feedrate_source(&live_feedrate_);

        // Create homing controller
        homing_controller_ = std::make_unique<HomingController>(
            *theta_stepper_, *rho_stepper_, *theta_tmc_, *rho_tmc_
        );

        auto homing_result = homing_controller_->init();
        if (homing_result.is_err()) {
            ESP_LOGE(TAG, "Failed to initialize homing controller");
            return homing_result;
        }

        // Give homing controller access to stepper controller for RMT-based motion
        homing_controller_->set_stepper_controller(stepper_controller_.get());

        // Create inactivity timer
        inactivity_timer_ = xTimerCreate(
            "MotorInactivity",
            kInactivityTimeout,
            pdFALSE,  // One-shot
            this,
            inactivity_timer_callback
        );

        if (!inactivity_timer_) {
            ESP_LOGE(TAG, "Failed to create inactivity timer");
            return Result<void>::err(MotionError::HardwareFault);
        }

        return Result<void>::ok();
    }

    Result<void> MotionController::init_tmc() {
        // Create TMC UART bus
        tmc_bus_ = std::make_unique<tmc::UartBus>();

        esp_err_t err = tmc_bus_->initialize(
            static_cast<uart_port_t>(HardwareConfig::TMC_UART_PORT),
            PinConfig::TMC_TX,
            PinConfig::TMC_RX,
            HardwareConfig::TMC_UART_BAUD
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize TMC UART bus");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Create TMC2209 drivers
        theta_tmc_ = std::make_unique<tmc::TMC2209Stepper>(*tmc_bus_, PinConfig::THETA_TMC_ADDR);
        rho_tmc_ = std::make_unique<tmc::TMC2209Stepper>(*tmc_bus_, PinConfig::RHO_TMC_ADDR);

        // Initialize theta TMC
        if (theta_tmc_->initialize() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize theta TMC2209");
            return Result<void>::err(MotionError::HardwareFault);
        }

        (void)theta_tmc_->set_motor_current(MotionConfig::THETA_IRUN_MA);
        (void)theta_tmc_->set_microstep_resolution(tmc::MicrostepResolution::Sixteenth);
        (void)theta_tmc_->set_stealthchop_enable(true);
        (void)theta_tmc_->set_stealthchop_threshold(0);

        // Initialize rho TMC
        if (rho_tmc_->initialize() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize rho TMC2209");
            return Result<void>::err(MotionError::HardwareFault);
        }

        (void)rho_tmc_->set_motor_current(MotionConfig::RHO_IRUN_MA);
        (void)rho_tmc_->set_microstep_resolution(tmc::MicrostepResolution::Sixteenth);
        (void)rho_tmc_->set_stealthchop_enable(true);
        (void)rho_tmc_->set_stealthchop_threshold(0);

        ESP_LOGI(TAG, "TMC2209 drivers initialized");
        return Result<void>::ok();
    }

    Result<void> MotionController::start() {
        if (running_.load(std::memory_order_acquire)) {
            return Result<void>::ok();  // Already running
        }

        // Create stepper task on Core 1 for real-time performance
        BaseType_t result = xTaskCreatePinnedToCore(
            stepper_task_entry,
            "stepper_task",
            TaskConfig::STEPPER_TASK_STACK,
            this,
            TaskConfig::STEPPER_TASK_PRIORITY,
            &stepper_task_,
            TaskConfig::STEPPER_TASK_CORE
        );

        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create stepper task");
            return Result<void>::err(MotionError::HardwareFault);
        }

        running_.store(true, std::memory_order_release);
        state_.store(SystemState::Idle, std::memory_order_release);

        return Result<void>::ok();
    }

    void MotionController::stop() {
        running_.store(false, std::memory_order_release);

        if (stepper_task_) {
            // Wait for task to exit
            vTaskDelay(pdMS_TO_TICKS(100));
            stepper_task_ = nullptr;
        }

        disable_motors();
        state_.store(SystemState::Idle, std::memory_order_release);
    }

    void MotionController::stepper_task_entry(void* arg) {
        auto* self = static_cast<MotionController*>(arg);
        self->stepper_task_loop();
    }

    void MotionController::stepper_task_loop() {
        while (running_.load(std::memory_order_acquire)) {
            // Drain request (normal stop): this task owns the queue's
            // consumer side, so IT clears everything and acknowledges by
            // clearing the flag (see halt_and_drain()).
            if (drain_requested_.load(std::memory_order_acquire)) {
                segment_queue_.clear();
                if (xSemaphoreTake(planner_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                    velocity_planner_->clear();
                    xSemaphoreGive(planner_mutex_);
                }
                stepper_controller_->emergency_stop();
                // Discarded segments will never execute — replan from where
                // the motors actually are.
                path_planner_->set_current_position(get_position());
                state_.store(SystemState::Idle, std::memory_order_release);
                reset_inactivity_timer();
                drain_requested_.store(false, std::memory_order_release);
                continue;
            }

            // Check for emergency stop
            if (emergency_stop_.load(std::memory_order_acquire)) {
                segment_queue_.clear();
                if (xSemaphoreTake(planner_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                    velocity_planner_->clear();
                    xSemaphoreGive(planner_mutex_);
                }
                stepper_controller_->emergency_stop();
                state_.store(SystemState::EStop, std::memory_order_release);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // Check for pause
            if (paused_.load(std::memory_order_acquire)) {
                state_.store(SystemState::Paused, std::memory_order_release);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Transfer ready segments from velocity planner to execution queue
            transfer_ready_segments();

            // Try to get next segment from queue
            MotionSegment segment;
            if (segment_queue_.pop(segment)) {
                // Motor steps already calculated in enqueue_segment() before velocity planning

                // Running reflects actual execution, set HERE. Setting it
                // from move_to() (the old design) only happened after the
                // whole move finished planning — a long multi-rotation move
                // pumps hundreds of split segments through the planner's
                // backpressure, so move_to() doesn't return (and the state
                // stayed "Idle") while the table was visibly drawing.
                state_.store(SystemState::Running, std::memory_order_release);

                // Execute segment with trapezoidal velocity profile
                auto result = execute_segment(segment);

                // Yield to other tasks to prevent watchdog timeout
                taskYIELD();

                if (result.is_err()) {
                    ESP_LOGE(TAG, "Segment execution failed");
                    state_.store(SystemState::Error, std::memory_order_release);
                    continue;
                }

                // Handle position overflow (wrap theta, adjust rho - like main branch)
                handle_position_overflow();

                // Go idle when all queues are empty
                if (segment_queue_.empty() && velocity_planner_->empty()) {
                    state_.store(SystemState::Idle, std::memory_order_release);
                    reset_inactivity_timer();
                }
            }
            else {
                // Queue empty, check if we should go idle
                if (state_.load(std::memory_order_acquire) == SystemState::Running &&
                    velocity_planner_->empty()) {
                    state_.store(SystemState::Idle, std::memory_order_release);
                    reset_inactivity_timer();
                }
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay when idle
            }
        }

        vTaskDelete(nullptr);
    }

    void MotionController::handle_position_overflow() {
        // Like main branch's handleStepOverflow()
        // When theta exceeds ±1 rotation, wrap it and adjust rho for coupling
        int32_t theta_steps = theta_stepper_->position();
        int32_t rho_steps = rho_stepper_->position();
        const int32_t theta_rot = transformer_->steps_per_theta_rotation();

        if (theta_rot <= 0) {
            return;  // Not calibrated yet
        }

        // Calculate rho adjustment per theta rotation (due to coupling).
        // The coupling counteraction accumulated by the transformer over one
        // theta rotation is theta_rot / gear_ratio steps, so that is exactly
        // what must be unwound per wrap. Using the compile-time
        // EFFECTIVE_STEPS_PER_REV here (the old code) is only correct when
        // the calibrated theta_rot happens to equal the nominal
        // gear_ratio * steps_per_rev — any calibration/config difference
        // injected a rho error on every wrap.
        const int32_t rho_per_theta_rot = static_cast<int32_t>(
            std::lround(static_cast<double>(theta_rot) /
                MechanicalConfig::theta_gear_ratio()));

        bool wrapped = false;

        // Wrap when theta reaches ± full rotation
        // Use >= / <= to ensure exact rotation boundary wraps
        // After wrapping: theta is in range (-theta_rot, theta_rot)
        while (theta_steps >= theta_rot) {
            theta_steps -= theta_rot;
            rho_steps -= rho_per_theta_rot;
            wrapped = true;
        }
        while (theta_steps <= -theta_rot) {
            theta_steps += theta_rot;
            rho_steps += rho_per_theta_rot;
            wrapped = true;
        }

        if (wrapped) {
            ESP_LOGD(TAG, "Position wrapped: theta=%ld, rho=%ld (rho_per_rot=%ld)",
                theta_steps, rho_steps, rho_per_theta_rot);
            theta_stepper_->set_position(theta_steps);
            rho_stepper_->set_position(rho_steps);
        }
    }

    void MotionController::inactivity_timer_callback(TimerHandle_t timer) {
        auto* self = static_cast<MotionController*>(pvTimerGetTimerID(timer));

        // Runs on the timer service task. Only power off if no enable
        // happened since this timer was armed — an expiry dispatched just
        // before enable_motors() must not disable under a starting move.
        // A failed take skips the disable (safe direction); the next idle
        // transition re-arms the timer.
        if (xSemaphoreTake(self->motor_power_mutex_, pdMS_TO_TICKS(500)) != pdTRUE) {
            return;
        }
        if (self->enable_epoch_.load(std::memory_order_acquire) ==
            self->armed_epoch_.load(std::memory_order_acquire)) {
            ESP_LOGI(TAG, "Motors idle for %lums - disabling",
                static_cast<unsigned long>(MotionConfig::MOTOR_INACTIVITY_TIMEOUT_MS));
            self->stepper_controller_->disable();
        }
        xSemaphoreGive(self->motor_power_mutex_);
    }

    void MotionController::reset_inactivity_timer() {
        if (inactivity_timer_) {
            armed_epoch_.store(enable_epoch_.load(std::memory_order_acquire),
                std::memory_order_release);
            xTimerReset(inactivity_timer_, 0);
        }
    }

    void MotionController::enable_motors() {
        xTimerStop(inactivity_timer_, 0);
        if (xSemaphoreTake(motor_power_mutex_, portMAX_DELAY) == pdTRUE) {
            // Invalidate any in-flight inactivity expiry before energizing;
            // the callback compares against armed_epoch_ under this mutex.
            enable_epoch_.fetch_add(1, std::memory_order_acq_rel);
            stepper_controller_->enable();
            xSemaphoreGive(motor_power_mutex_);
        }
    }

    void MotionController::disable_motors() {
        // Explicit disable (console command / shutdown) — unconditional,
        // just serialized against the enable/expiry paths.
        if (xSemaphoreTake(motor_power_mutex_, portMAX_DELAY) == pdTRUE) {
            stepper_controller_->disable();
            xSemaphoreGive(motor_power_mutex_);
        }
    }

    Result<void> MotionController::home(bool force_full) {
        if (homing_controller_->is_homing()) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Homing runs in the CALLER's task while the stepper task keeps
        // consuming the segment queue. Stop playback and drain queued
        // segments first, or a queued move executes concurrently with the
        // homing seeks on the same sequencer. move_to()/move_linear() reject
        // new moves while state is Homing (below), so nothing refills the
        // queue mid-homing.
        halt_and_drain();

        state_.store(SystemState::Homing, std::memory_order_release);
        enable_motors();

        auto result = homing_controller_->home_all(force_full);

        if (result.is_ok()) {
            is_homed_.store(true, std::memory_order_release);

            // Pass calibration values to coordinate transformer
            transformer_->set_calibration(
                homing_controller_->theta_steps_per_rotation(),
                homing_controller_->rho_max_steps()
            );

            // The path planner splits moves against the real step scales
            path_planner_->set_calibration(
                homing_controller_->theta_steps_per_rotation(),
                homing_controller_->rho_max_steps()
            );

            // Reset motor positions to home (0 steps)
            theta_stepper_->set_position(0);
            rho_stepper_->set_position(0);

            // Reset fractional accumulators
            transformer_->reset_accumulators();

            // Reset path planner position
            path_planner_->set_current_position({ 0.0, 0.0 });

            state_.store(SystemState::Idle, std::memory_order_release);
        }
        else {
            state_.store(SystemState::Error, std::memory_order_release);
            ESP_LOGE(TAG, "Homing failed");
        }

        return result;
    }

    void MotionController::_set_homed(int32_t theta_steps_per_rot, int32_t rho_max) {
        homing_controller_->_set_homed(theta_steps_per_rot, rho_max);
        transformer_->set_calibration(theta_steps_per_rot, rho_max);
        path_planner_->set_calibration(theta_steps_per_rot, rho_max);

        theta_stepper_->set_position(0);
        rho_stepper_->set_position(0);

        // Reset fractional accumulators
        transformer_->reset_accumulators();
        path_planner_->set_current_position({ 0.0, 0.0 });

        // Mark as homed
        is_homed_.store(true, std::memory_order_release);
        state_.store(SystemState::Idle, std::memory_order_release);
    }

    Result<void> MotionController::move_to(const PolarPosition& target, float feedrate,
                                           float base_feedrate) {
        if (!is_homed_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::NotHomed);
        }

        if (emergency_stop_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::EmergencyStop);
        }

        // No new moves while homing owns the sequencer (see home())
        if (state_.load(std::memory_order_acquire) == SystemState::Homing) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Check bounds
        if (!is_in_bounds(target)) {
            ESP_LOGW(TAG, "Target out of bounds: theta=%.4f, rho=%.4f", target.theta, target.rho);
            return Result<void>::err(MotionError::OutOfBounds);
        }

        // Get current planning position from PathPlanner
        const PolarPosition& current = path_planner_->current_position();

        // Set default feedrate if not specified
        float actual_feedrate = feedrate;
        if (actual_feedrate <= 0) {
            actual_feedrate = static_cast<float>(MotionConfig::RHO_MAX_SPEED_RPM);
        }

        ESP_LOGD(TAG, "Queueing polar move: (%.4f, %.4f) -> (%.4f, %.4f) @ %.1f RPM",
            current.theta, current.rho, target.theta, target.rho, actual_feedrate);

        // Enable motors BEFORE planning: plan_polar_move blocks on planner
        // backpressure for long moves, and execution starts consuming
        // segments while we are still inside it. (SystemState::Running is
        // set by the stepper task when it actually executes — see
        // stepper_task_loop.)
        enable_motors();

        // Plan direct polar move (arcs in XY space)
        return path_planner_->plan_polar_move(current, target, actual_feedrate,
            (base_feedrate > 0) ? base_feedrate : actual_feedrate);
    }

    Result<void> MotionController::move_linear(const PolarPosition& target, float feedrate,
                                               float base_feedrate) {
        if (!is_homed_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::NotHomed);
        }

        if (emergency_stop_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::EmergencyStop);
        }

        // No new moves while homing owns the sequencer (see home())
        if (state_.load(std::memory_order_acquire) == SystemState::Homing) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Check bounds
        if (!is_in_bounds(target)) {
            ESP_LOGW(TAG, "Target out of bounds: theta=%.4f, rho=%.4f", target.theta, target.rho);
            return Result<void>::err(MotionError::OutOfBounds);
        }

        // Get current planning position from PathPlanner
        const PolarPosition& current = path_planner_->current_position();

        // Set default feedrate if not specified
        float actual_feedrate = feedrate;
        if (actual_feedrate <= 0) {
            actual_feedrate = static_cast<float>(MotionConfig::RHO_MAX_SPEED_RPM);
        }

        ESP_LOGD(TAG, "Queueing linear move: (%.4f, %.4f) -> (%.4f, %.4f) @ %.1f RPM",
            current.theta, current.rho, target.theta, target.rho, actual_feedrate);

        // Enable motors before planning; Running is set by the stepper task
        // on actual execution (see move_to / stepper_task_loop).
        enable_motors();

        // Plan Cartesian-interpolated move (straight lines in XY space)
        return path_planner_->plan_linear_move(current, target, actual_feedrate,
            (base_feedrate > 0) ? base_feedrate : actual_feedrate);
    }

    void MotionController::pause() {
        paused_.store(true, std::memory_order_release);
        ESP_LOGI(TAG, "Motion paused");
    }

    void MotionController::resume() {
        paused_.store(false, std::memory_order_release);
        ESP_LOGI(TAG, "Motion resumed");
    }

    void MotionController::halt_and_drain() {
        // No stepper task -> nothing queued, nothing to acknowledge the
        // drain flag (it would just stall the 5s wait below).
        if (!running_.load(std::memory_order_acquire)) {
            return;
        }

        // Refuse new segments so a task blocked mid-plan unwinds
        abort_planning_.store(true, std::memory_order_release);

        // Ask the stepper task to drain, and interrupt the blocking
        // execute() so it gets there promptly. The sequencer only raises a
        // flag here; the hardware abort happens inside execute() in the
        // stepper task's own context.
        drain_requested_.store(true, std::memory_order_release);
        stepper_controller_->emergency_stop();

        // Wait for the stepper task to acknowledge (it clears the flag).
        // Bounded: the in-flight chunk is at most ~2s even at minimum speed.
        for (int i = 0; i < 500 && drain_requested_.load(std::memory_order_acquire); ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (drain_requested_.load(std::memory_order_acquire)) {
            ESP_LOGE(TAG, "halt_and_drain: stepper task did not acknowledge");
            drain_requested_.store(false, std::memory_order_release);
        }

        abort_planning_.store(false, std::memory_order_release);
        ESP_LOGI(TAG, "Motion halted and queues drained");
    }

    void MotionController::emergency_stop() {
        emergency_stop_.store(true, std::memory_order_release);
        stepper_controller_->emergency_stop();
        state_.store(SystemState::EStop, std::memory_order_release);
        ESP_LOGW(TAG, "Emergency stop activated");
    }

    void MotionController::clear_emergency_stop() {
        emergency_stop_.store(false, std::memory_order_release);
        // Queued segments were discarded — replan from the physical position
        path_planner_->set_current_position(get_position());
        state_.store(SystemState::Idle, std::memory_order_release);
        ESP_LOGI(TAG, "Emergency stop cleared");
    }

    PolarPosition MotionController::get_position() const {
        // Derive polar position from actual motor step positions (for display/API only)
        return transformer_->steps_to_polar(
            theta_stepper_->position(),
            rho_stepper_->position()
        );
    }

    PolarPosition MotionController::get_planning_position() const {
        return path_planner_->current_position();
    }

    bool MotionController::enqueue_segment(MotionSegment& segment) {
        // Calculate motor steps BEFORE velocity planning
        // This allows VelocityPlanner to see actual motor directions after coupling compensation
        transformer_->delta_polar_to_motor_steps(
            segment.delta_theta_rad,
            segment.delta_rho_norm,
            segment.delta_theta_steps,
            segment.delta_rho_steps
        );

        // Backpressure: wait for planner space, but bail out if a stop/drain
        // is in progress — refilling a planner that is being drained would
        // deadlock the busy-wait (the drain empties it, we refill it).
        for (;;) {
            if (abort_planning_.load(std::memory_order_acquire) ||
                emergency_stop_.load(std::memory_order_acquire)) {
                return false;
            }
            if (xSemaphoreTake(planner_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
                continue;
            }
            if (!velocity_planner_->full()) {
                break;  // mutex held, space available
            }
            xSemaphoreGive(planner_mutex_);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        const bool added = velocity_planner_->add_segment(segment);
        xSemaphoreGive(planner_mutex_);

        if (added) {
            // Track total steps queued for progress reporting
            // (major axis determines segment duration)
            uint32_t segment_steps = static_cast<uint32_t>(
                std::max(std::abs(segment.delta_theta_steps), std::abs(segment.delta_rho_steps))
            );
            total_steps_queued_.fetch_add(segment_steps, std::memory_order_relaxed);
        }
        return added;
    }

    void MotionController::transfer_ready_segments() {
        // Transfer all available segments from velocity planner to execution queue
        // Lookahead works naturally: if segments queue faster than execution,
        // velocity planner calculates junction velocities before transfer
        if (xSemaphoreTake(planner_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;  // Retry next loop iteration
        }
        while (!velocity_planner_->empty() && !segment_queue_.full()) {
            auto popped = velocity_planner_->pop_segment();
            if (popped) {
                (void)segment_queue_.push(*popped);  // Already checked !full() above
            }
        }
        xSemaphoreGive(planner_mutex_);
    }

    Result<void> MotionController::execute_segment(const MotionSegment& segment) {
        // Calculate segment step count (same logic as enqueue_segment)
        uint32_t segment_steps = static_cast<uint32_t>(
            std::max(std::abs(segment.delta_theta_steps), std::abs(segment.delta_rho_steps))
        );

        // Segment already has entry/exit velocities from VelocityPlanner
        // Execute via stepper controller (this is blocking per-segment)
        auto result = stepper_controller_->execute_segment(segment);

        if (result.is_ok()) {
            // Track completed steps for progress reporting
            total_steps_completed_.fetch_add(segment_steps, std::memory_order_relaxed);
        }

        return result;
    }

    RobotStatus MotionController::get_status() const {
        RobotStatus status;

        status.position = get_position();
        status.state = get_state();
        status.is_homed = is_homed_.load(std::memory_order_acquire);
        status.queue_depth = segment_queue_.size();
        status.queue_capacity = kSegmentQueueSize - 1;  // One slot reserved

        if (theta_stepper_) {
            status.theta.position_steps = theta_stepper_->position();
            status.theta.is_enabled = theta_stepper_->is_enabled();
            status.theta.is_homed = status.is_homed;
        }

        if (rho_stepper_) {
            status.rho.position_steps = rho_stepper_->position();
            status.rho.is_enabled = rho_stepper_->is_enabled();
            status.rho.is_homed = status.is_homed;
            status.rho.is_stalled = rho_tmc_ ? rho_tmc_->is_stalled() : false;
        }

        return status;
    }

    MotionController::MotionProgress MotionController::get_motion_progress() const {
        MotionProgress progress;

        progress.steps_queued = total_steps_queued_.load(std::memory_order_acquire);
        progress.steps_completed = total_steps_completed_.load(std::memory_order_acquire);
        progress.segments_queued = segment_queue_.size();
        // Advisory single-word read; exactness is not required here and the
        // planner mutex must not block a status getter.
        progress.planner_pending = velocity_planner_->segment_count();

        // Get current segment progress from stepper controller
        if (stepper_controller_) {
            auto seg_progress = stepper_controller_->get_segment_progress();
            progress.current_segment_steps_total = seg_progress.steps_total;
            progress.current_segment_steps_done = seg_progress.steps_done;
            progress.is_executing = seg_progress.is_executing;

            // Add current segment's in-progress steps to completed count
            if (seg_progress.is_executing) {
                progress.steps_completed += seg_progress.steps_done;
            }
        } else {
            progress.current_segment_steps_total = 0;
            progress.current_segment_steps_done = 0;
            progress.is_executing = false;
        }

        return progress;
    }

    void MotionController::reset_progress_counters() {
        total_steps_queued_.store(0, std::memory_order_release);
        total_steps_completed_.store(0, std::memory_order_release);
    }

} // namespace sand_table
