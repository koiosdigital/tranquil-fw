#include "motion_controller.h"
#include "stepper_driver.h"
#include "homing_controller.h"
#include "coordinate_transformer.h"
#include "velocity_planner.h"
#include "path_planner.h"

#include "esp_log.h"

namespace sand_table {

static const char* TAG = "MotionController";

MotionController::MotionController()
    : transformer_(std::make_unique<CoordinateTransformer>())
    , velocity_planner_(std::make_unique<VelocityPlanner>())
    , path_planner_(std::make_unique<PathPlanner>())
{
    position_mutex_ = xSemaphoreCreateMutex();
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

    // Setup path planner callback to feed velocity planner
    path_planner_->set_segment_callback([this](MotionSegment& segment) -> bool {
        return velocity_planner_->add_segment(segment);
    });

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

    // Initialize rho TMC
    if (rho_tmc_->initialize() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize rho TMC2209");
        return Result<void>::err(MotionError::HardwareFault);
    }

    (void)rho_tmc_->set_motor_current(MotionConfig::RHO_IRUN_MA);
    (void)rho_tmc_->set_microstep_resolution(tmc::MicrostepResolution::Sixteenth);
    (void)rho_tmc_->set_stealthchop_enable(true);
    (void)rho_tmc_->set_stallguard_threshold(MotionConfig::RHO_STALLGUARD_THRESHOLD);

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
    velocity_planner_->clear();
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

        // Try to get next segment
        auto segment = velocity_planner_->pop_segment();
        if (!segment) {
            // No segments available, idle
            if (state_.load(std::memory_order_acquire) == SystemState::Running) {
                state_.store(SystemState::Idle, std::memory_order_release);
                reset_inactivity_timer();
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Execute segment
        state_.store(SystemState::Running, std::memory_order_release);
        xTimerStop(inactivity_timer_, 0);

        auto result = stepper_controller_->execute_segment(*segment);

        if (result.is_err()) {
            if (result.error() == MotionError::EmergencyStop) {
                state_.store(SystemState::EStop, std::memory_order_release);
            } else {
                state_.store(SystemState::Error, std::memory_order_release);
            }
            continue;
        }

        // Update position
        update_position(stepper_controller_->get_position());
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

        // Reset position tracking
        if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
            current_position_ = {0.0f, MechanicalConfig::RHO_MIN_MM};
            xSemaphoreGive(position_mutex_);
        }

        path_planner_->set_current_position({0.0f, MechanicalConfig::RHO_MIN_MM});
        path_planner_->reset_accumulator();

        state_.store(SystemState::Idle, std::memory_order_release);
        ESP_LOGI(TAG, "Homing complete");
    } else {
        state_.store(SystemState::Error, std::memory_order_release);
        ESP_LOGE(TAG, "Homing failed");
    }

    return result;
}

Result<void> MotionController::move_to(const PolarPosition& target, float feedrate) {
    if (!is_homed_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::NotHomed);
    }

    if (emergency_stop_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::EmergencyStop);
    }

    enable_motors();

    PolarPosition current;
    if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
        current = current_position_;
        xSemaphoreGive(position_mutex_);
    }

    return path_planner_->plan_linear_move(current, target, feedrate);
}

Result<void> MotionController::draw_spiral(
    float start_rho,
    float end_rho,
    float rotations,
    float feedrate)
{
    if (!is_homed_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::NotHomed);
    }

    if (emergency_stop_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::EmergencyStop);
    }

    enable_motors();

    PolarPosition current;
    if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
        current = current_position_;
        xSemaphoreGive(position_mutex_);
    }

    return path_planner_->plan_spiral(current, start_rho, end_rho, rotations, feedrate);
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
    velocity_planner_->clear();
    state_.store(SystemState::EStop, std::memory_order_release);
    ESP_LOGW(TAG, "Emergency stop activated");
}

void MotionController::clear_emergency_stop() {
    emergency_stop_.store(false, std::memory_order_release);
    state_.store(SystemState::Idle, std::memory_order_release);
    ESP_LOGI(TAG, "Emergency stop cleared");
}

PolarPosition MotionController::get_position() const {
    PolarPosition pos;
    if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
        pos = current_position_;
        xSemaphoreGive(position_mutex_);
    }
    return pos;
}

void MotionController::update_position(const StepPosition& steps) {
    PolarPosition pos = transformer_->steps_to_polar(steps);

    if (xSemaphoreTake(position_mutex_, portMAX_DELAY)) {
        current_position_ = pos;
        xSemaphoreGive(position_mutex_);
    }

    path_planner_->set_current_position(pos);
}

RobotStatus MotionController::get_status() const {
    RobotStatus status;

    status.position = get_position();
    status.state = get_state();
    status.is_homed = is_homed_.load(std::memory_order_acquire);
    status.queue_depth = velocity_planner_->segment_count();
    status.queue_capacity = VelocityPlanner::kLookaheadDepth;

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
