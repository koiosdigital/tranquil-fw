#include "motion_controller.h"
#include "stepper_driver.h"
#include "homing_controller.h"
#include "coordinate_transformer.h"
#include "path_planner.h"

#include "esp_log.h"

namespace sand_table {

    static const char* TAG = "MotionController";

    MotionController::MotionController()
        : path_planner_(std::make_unique<PathPlanner>())
        , transformer_(std::make_unique<CoordinateTransformer>())
    {
        position_mutex_ = xSemaphoreCreateMutex();

        // Set up PathPlanner callback to enqueue segments
        path_planner_->set_segment_callback([this](MotionSegment& seg) {
            return this->enqueue_segment(seg);
            });
    }

    MotionController::~MotionController() {
        stop();

        if (position_mutex_) {
            vSemaphoreDelete(position_mutex_);
        }

        if (inactivity_timer_) {
            xTimerDelete(inactivity_timer_, 0);
        }
    }

    Result<void> MotionController::init() {
        ESP_LOGI(TAG, "Initializing motion controller...");

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

        ESP_LOGI(TAG, "Motion controller initialized");
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

        ESP_LOGI(TAG, "Starting motion controller tasks...");

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

        ESP_LOGI(TAG, "Motion controller started");
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

        ESP_LOGI(TAG, "Motion controller stopped");
    }

    void MotionController::stepper_task_entry(void* arg) {
        auto* self = static_cast<MotionController*>(arg);
        self->stepper_task_loop();
    }

    void MotionController::stepper_task_loop() {
        ESP_LOGI(TAG, "Stepper task started on core %d", xPortGetCoreID());

        while (running_.load(std::memory_order_acquire)) {
            // Check for emergency stop
            if (emergency_stop_.load(std::memory_order_acquire)) {
                segment_queue_.clear();
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

            // Try to get next segment from queue
            MotionSegment segment;
            if (segment_queue_.pop(segment)) {
                // Convert target polar to absolute step position (one-time conversion)
                PolarPosition target_polar{ segment.target_theta_rad, segment.target_rho_norm };
                int32_t new_target_theta_steps, new_target_rho_steps;
                transformer_->polar_to_absolute_steps(
                    target_polar,
                    new_target_theta_steps, new_target_rho_steps
                );

                // Get current target steps (protected by mutex)
                int32_t curr_theta_steps, curr_rho_steps;
                if (xSemaphoreTake(position_mutex_, pdMS_TO_TICKS(10))) {
                    curr_theta_steps = target_theta_steps_;
                    curr_rho_steps = target_rho_steps_;
                    xSemaphoreGive(position_mutex_);
                }
                else {
                    ESP_LOGE(TAG, "Failed to acquire position mutex");
                    continue;
                }

                // Calculate motor step deltas using pure integer arithmetic
                // This avoids floating-point accumulation errors
                transformer_->calculate_coupled_delta_steps(
                    curr_theta_steps, curr_rho_steps,
                    new_target_theta_steps, new_target_rho_steps,
                    segment.delta_theta_steps, segment.delta_rho_steps
                );

                // Execute segment at constant velocity
                auto result = execute_segment_constant_velocity(segment);

                // Yield to other tasks to prevent watchdog timeout
                taskYIELD();

                if (result.is_err()) {
                    ESP_LOGE(TAG, "Segment execution failed");
                    state_.store(SystemState::Error, std::memory_order_release);
                    continue;
                }

                // Get actual motor positions - these are the source of truth
                int32_t actual_theta = theta_stepper_->position();
                int32_t actual_rho = rho_stepper_->position();

                // Check for drift between expected and actual
                int32_t expected_theta = curr_theta_steps + segment.delta_theta_steps;
                int32_t theta_drift = actual_theta - expected_theta;
                if (std::abs(theta_drift) > 2) {
                    ESP_LOGW(TAG, "Theta drift detected: expected=%ld, actual=%ld, drift=%ld",
                        expected_theta, actual_theta, theta_drift);
                }

                // Update position tracking
                // Theta accumulates naturally (no normalization/wraparound)
                // Rho is tracked as "raw" steps (coupling compensation applied per-segment)
                if (xSemaphoreTake(position_mutex_, pdMS_TO_TICKS(10))) {
                    target_theta_steps_ = new_target_theta_steps;
                    // Rho tracking must account for coupling: motor position includes
                    // compensation, so we need to track the "raw" equivalent.
                    // raw_rho = motor_rho - accumulated_coupling
                    // But simpler: just track what polar_to_absolute_steps gives us,
                    // since that's consistent with how we compute deltas.
                    target_rho_steps_ = new_target_rho_steps;
                    xSemaphoreGive(position_mutex_);
                }

                // Check if this was the last segment
                if (segment.is_last_segment && segment_queue_.empty()) {
                    state_.store(SystemState::Idle, std::memory_order_release);
                    reset_inactivity_timer();
                }
            }
            else {
                // Queue empty, check if we should go idle
                if (state_.load(std::memory_order_acquire) == SystemState::Running) {
                    state_.store(SystemState::Idle, std::memory_order_release);
                    reset_inactivity_timer();
                }
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay when idle
            }
        }

        ESP_LOGI(TAG, "Stepper task exiting");
        vTaskDelete(nullptr);
    }

    void MotionController::inactivity_timer_callback(TimerHandle_t timer) {
        auto* self = static_cast<MotionController*>(pvTimerGetTimerID(timer));
        ESP_LOGD(TAG, "Motor inactivity timeout - disabling motors");
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

            ESP_LOGI(TAG, "Calibration set: theta=%ld steps/rot, rho=%ld max steps",
                homing_controller_->theta_steps_per_rotation(),
                homing_controller_->rho_max_steps());

            // Reset position tracking to home position (0 steps)
            if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
                target_theta_steps_ = 0;
                target_rho_steps_ = 0;
                xSemaphoreGive(position_mutex_);
            }

            // Reset path planner position
            path_planner_->set_current_position({ 0.0, 0.0 });

            state_.store(SystemState::Idle, std::memory_order_release);
            ESP_LOGI(TAG, "Homing complete");
        }
        else {
            state_.store(SystemState::Error, std::memory_order_release);
            ESP_LOGE(TAG, "Homing failed");
        }

        return result;
    }

    void MotionController::_set_homed(int32_t theta_steps_per_rot, int32_t rho_max) {
        ESP_LOGI(TAG, "Setting homed state: theta=%ld steps/rot, rho=%ld max steps",
            theta_steps_per_rot, rho_max);

        // Set homing controller calibration
        homing_controller_->_set_homed(theta_steps_per_rot, rho_max);

        // Pass calibration to coordinate transformer
        transformer_->set_calibration(theta_steps_per_rot, rho_max);

        // Reset position to home (0 steps)
        if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
            target_theta_steps_ = 0;
            target_rho_steps_ = 0;
            xSemaphoreGive(position_mutex_);
        }

        // Reset path planner position
        path_planner_->set_current_position({ 0.0, 0.0 });

        // Mark as homed
        is_homed_.store(true, std::memory_order_release);
        state_.store(SystemState::Idle, std::memory_order_release);

        ESP_LOGI(TAG, "Homed state set (development mode)");
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
        // This represents where the arm will be after all currently queued moves
        // (not the actual position, which is tracked separately)
        const PolarPosition& current = path_planner_->current_position();

        // Set default feedrate if not specified
        float actual_feedrate = feedrate;
        if (actual_feedrate <= 0) {
            actual_feedrate = static_cast<float>(MotionConfig::RHO_MAX_SPEED_RPM);
        }

        ESP_LOGD(TAG, "Queueing move: (%.4f, %.4f) -> (%.4f, %.4f) @ %.1f RPM",
            current.theta, current.rho, target.theta, target.rho, actual_feedrate);

        // Plan the move (this enqueues segments via callback)
        // PathPlanner updates its internal position to target after planning
        auto result = path_planner_->plan_linear_move(current, target, actual_feedrate);

        if (result.is_ok()) {
            // Don't update current_position_ here - that's the actual executed position
            // which is updated by stepper_task_loop after each segment completes.
            // PathPlanner tracks the planning position internally.

            // Ensure motors are enabled and state is Running
            enable_motors();
            state_.store(SystemState::Running, std::memory_order_release);
        }

        return result;  // Returns immediately after queueing
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
        // Derive polar position from step counts (only for display/API)
        int32_t theta_steps, rho_steps;
        if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
            theta_steps = target_theta_steps_;
            rho_steps = target_rho_steps_;
            xSemaphoreGive(position_mutex_);
        }
        else {
            return { 0.0, 0.0 };
        }
        return transformer_->steps_to_polar(theta_steps, rho_steps);
    }

    void MotionController::update_position(int32_t theta_steps, int32_t rho_steps) {
        // Store step counts directly (no floating-point conversion)
        if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
            target_theta_steps_ = theta_steps;
            target_rho_steps_ = rho_steps;
            xSemaphoreGive(position_mutex_);
        }
    }

    bool MotionController::enqueue_segment(MotionSegment& segment) {
        // Wait for queue space (provides natural backpressure)
        // This blocks the caller until there's room in the queue
        // Using 10ms delay to avoid starving IDLE task (watchdog)
        while (segment_queue_.full()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Motor steps are calculated at execution time, not here
        // The segment already has target_theta_rad and target_rho_norm set by PathPlanner
        return segment_queue_.push(segment);
    }

    Result<void> MotionController::execute_segment_constant_velocity(const MotionSegment& segment) {
        // Create a copy with simplified constant velocity profile
        MotionSegment exec_segment = segment;

        // Set entry/exit velocity equal to nominal (constant velocity)
        exec_segment.entry_velocity = segment.nominal_velocity;
        exec_segment.exit_velocity = segment.nominal_velocity;
        exec_segment.acceleration = 0.0f;

        // Execute via stepper controller (this is blocking per-segment)
        return stepper_controller_->execute_segment(exec_segment);
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
