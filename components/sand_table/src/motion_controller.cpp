#include "motion_controller.h"
#include "stepper_driver.h"
#include "homing_controller.h"
#include "coordinate_transformer.h"
#include "path_planner.h"
#include "velocity_planner.h"

#include "esp_log.h"

namespace sand_table {

    static const char* TAG = "MotionController";

    MotionController::MotionController()
        : path_planner_(std::make_unique<PathPlanner>())
        , velocity_planner_(std::make_unique<VelocityPlanner>())
        , transformer_(std::make_unique<CoordinateTransformer>())
    {
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
            // Check for emergency stop
            if (emergency_stop_.load(std::memory_order_acquire)) {
                segment_queue_.clear();
                velocity_planner_->clear();
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

        // Calculate rho adjustment per theta rotation (due to coupling)
        // When theta motor rotates once (EFFECTIVE_STEPS_PER_REV steps), rho moves
        // by EFFECTIVE_STEPS_PER_REV / THETA_GEAR_RATIO steps due to mechanical coupling.

        const int32_t rho_per_theta_rot = static_cast<int32_t>(
            MechanicalConfig::EFFECTIVE_STEPS_PER_REV
            );

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
        self->disable_motors();
    }

    void MotionController::reset_inactivity_timer() {
        if (inactivity_timer_) {
            xTimerReset(inactivity_timer_, 0);
        }
    }

    void MotionController::enable_motors() {
        stepper_controller_->enable();
        xTimerStop(inactivity_timer_, 0);
    }

    void MotionController::disable_motors() {
        stepper_controller_->disable();
    }

    Result<void> MotionController::home() {
        if (homing_controller_->is_homing()) {
            return Result<void>::err(MotionError::InvalidState);
        }

        state_.store(SystemState::Homing, std::memory_order_release);
        enable_motors();

        auto result = homing_controller_->home_all();

        if (result.is_ok()) {
            is_homed_.store(true, std::memory_order_release);

            // Pass calibration values to coordinate transformer
            transformer_->set_calibration(
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

        theta_stepper_->set_position(0);
        rho_stepper_->set_position(0);

        // Reset fractional accumulators
        transformer_->reset_accumulators();
        path_planner_->set_current_position({ 0.0, 0.0 });

        // Mark as homed
        is_homed_.store(true, std::memory_order_release);
        state_.store(SystemState::Idle, std::memory_order_release);
    }

    Result<void> MotionController::move_to(const PolarPosition& target, float feedrate) {
        if (!is_homed_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::NotHomed);
        }

        if (emergency_stop_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::EmergencyStop);
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

        // Plan direct polar move (arcs in XY space)
        auto result = path_planner_->plan_polar_move(current, target, actual_feedrate);

        if (result.is_ok()) {
            enable_motors();
            state_.store(SystemState::Running, std::memory_order_release);
        }

        return result;
    }

    Result<void> MotionController::move_linear(const PolarPosition& target, float feedrate) {
        if (!is_homed_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::NotHomed);
        }

        if (emergency_stop_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::EmergencyStop);
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

        // Plan Cartesian-interpolated move (straight lines in XY space)
        auto result = path_planner_->plan_linear_move(current, target, actual_feedrate);

        if (result.is_ok()) {
            enable_motors();
            state_.store(SystemState::Running, std::memory_order_release);
        }

        return result;
    }

    void MotionController::pause() {
        paused_.store(true, std::memory_order_release);
        ESP_LOGI(TAG, "Motion paused");
    }

    void MotionController::resume() {
        paused_.store(false, std::memory_order_release);
        ESP_LOGI(TAG, "Motion resumed");
    }

    void MotionController::emergency_stop() {
        emergency_stop_.store(true, std::memory_order_release);
        stepper_controller_->emergency_stop();
        state_.store(SystemState::EStop, std::memory_order_release);
        ESP_LOGW(TAG, "Emergency stop activated");
    }

    void MotionController::clear_emergency_stop() {
        emergency_stop_.store(false, std::memory_order_release);
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

    bool MotionController::enqueue_segment(MotionSegment& segment) {
        // Calculate motor steps BEFORE velocity planning
        // This allows VelocityPlanner to see actual motor directions after coupling compensation
        transformer_->delta_polar_to_motor_steps(
            segment.delta_theta_rad,
            segment.delta_rho_norm,
            segment.delta_theta_steps,
            segment.delta_rho_steps
        );

        // Push segment to velocity planner for lookahead processing
        // VelocityPlanner calculates entry/exit velocities based on motor directions
        while (velocity_planner_->full()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        return velocity_planner_->add_segment(segment);
    }

    void MotionController::transfer_ready_segments() {
        // Transfer all available segments from velocity planner to execution queue
        // Lookahead works naturally: if segments queue faster than execution,
        // velocity planner calculates junction velocities before transfer
        while (!velocity_planner_->empty() && !segment_queue_.full()) {
            auto popped = velocity_planner_->pop_segment();
            if (popped) {
                segment_queue_.push(*popped);
            }
        }
    }

    Result<void> MotionController::execute_segment(const MotionSegment& segment) {
        // Segment already has entry/exit velocities from VelocityPlanner
        // Execute via stepper controller (this is blocking per-segment)
        return stepper_controller_->execute_segment(segment);
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

} // namespace sand_table
