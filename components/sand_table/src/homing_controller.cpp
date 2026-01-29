#include "homing_controller.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

namespace sand_table {

static const char* TAG = "HomingController";

// Global instance pointer for ISR callbacks
static HomingController* g_homing_instance = nullptr;

HomingController::HomingController(
    StepperDriver& theta,
    StepperDriver& rho,
    tmc::TMC2209Stepper& theta_tmc,
    tmc::TMC2209Stepper& rho_tmc)
    : theta_(theta)
    , rho_(rho)
    , theta_tmc_(theta_tmc)
    , rho_tmc_(rho_tmc)
{
    g_homing_instance = this;
}

HomingController::~HomingController() {
    abort();

    if (step_timer_) {
        gptimer_stop(step_timer_);
        gptimer_disable(step_timer_);
        gptimer_del_timer(step_timer_);
    }

    if (completion_sem_) {
        vSemaphoreDelete(completion_sem_);
    }

    gpio_isr_handler_remove(PinConfig::THETA_HALL);
    gpio_isr_handler_remove(PinConfig::RHO_DIAG);
    g_homing_instance = nullptr;
}

Result<void> HomingController::init() {
    ESP_LOGI(TAG, "Initializing homing controller...");
    ESP_LOGI(TAG, "  Hall sensor pin: GPIO%d", PinConfig::THETA_HALL);
    ESP_LOGI(TAG, "  Rho DIAG pin: GPIO%d", PinConfig::RHO_DIAG);
    ESP_LOGI(TAG, "  StallGuard threshold: %d", MotionConfig::RHO_STALLGUARD_THRESHOLD);

    // Create completion semaphore
    completion_sem_ = xSemaphoreCreateBinary();
    if (!completion_sem_) {
        ESP_LOGE(TAG, "Failed to create completion semaphore");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Configure Hall sensor GPIO with interrupt
    gpio_config_t hall_config = {
        .pin_bit_mask = (1ULL << PinConfig::THETA_HALL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Active low
    };

    if (gpio_config(&hall_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Hall sensor GPIO");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Configure StallGuard DIAG pin with interrupt (no pull resistors - TMC2209 is push-pull)
    gpio_config_t diag_config = {
        .pin_bit_mask = (1ULL << PinConfig::RHO_DIAG),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,  // DIAG goes high on stall
    };

    if (gpio_config(&diag_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure StallGuard DIAG GPIO");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Install GPIO ISR service (ignore if already installed)
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Add Hall sensor ISR
    if (gpio_isr_handler_add(PinConfig::THETA_HALL, hall_isr_handler, this) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Hall sensor ISR");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Add StallGuard DIAG ISR
    if (gpio_isr_handler_add(PinConfig::RHO_DIAG, diag_isr_handler, this) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add StallGuard DIAG ISR");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Create step timer (1 MHz resolution = 1us per tick)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz
        .intr_priority = 0,
        .flags = { .intr_shared = false }
    };

    if (gptimer_new_timer(&timer_config, &step_timer_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create step timer");
        return Result<void>::err(MotionError::HardwareFault);
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_callback
    };

    if (gptimer_register_event_callbacks(step_timer_, &cbs, this) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register timer callback");
        return Result<void>::err(MotionError::HardwareFault);
    }

    if (gptimer_enable(step_timer_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable step timer");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Configure StallGuard for rho axis
    ESP_LOGI(TAG, "Configuring TMC2209 StallGuard...");
    (void)rho_tmc_.set_stallguard_threshold(MotionConfig::RHO_STALLGUARD_THRESHOLD);

    // CRITICAL: Set TCOOLTHRS to enable StallGuard at all speeds
    (void)rho_tmc_.set_stallguard_min_speed(0xFFFFF);

    int diag_level = gpio_get_level(PinConfig::RHO_DIAG);
    ESP_LOGI(TAG, "Initial DIAG pin level: %d", diag_level);

    ESP_LOGI(TAG, "Homing controller initialized");
    return Result<void>::ok();
}

// =============================================================================
// ISR Handlers
// =============================================================================

void IRAM_ATTR HomingController::hall_isr_handler(void* arg) {
    auto* self = static_cast<HomingController*>(arg);
    self->hall_triggered_ = true;
}

void IRAM_ATTR HomingController::diag_isr_handler(void* arg) {
    auto* self = static_cast<HomingController*>(arg);
    self->rho_stall_triggered_ = true;
}

bool IRAM_ATTR HomingController::step_timer_callback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t* edata,
    void* user_ctx)
{
    auto* self = static_cast<HomingController*>(user_ctx);
    self->generate_homing_steps();
    return true;  // Keep alarm active
}

// =============================================================================
// Timer-based Step Generation (runs in ISR context)
// =============================================================================

void IRAM_ATTR HomingController::generate_homing_steps() {
    HomingState current_state = state_.load(std::memory_order_acquire);
    bool step_generated = false;

    // Process state machine transitions first
    switch (current_state) {
    case HomingState::RhoSeekingMax:
        if (rho_stall_triggered_) {
            rho_stall_triggered_ = false;
            gptimer_stop(step_timer_);
            // Transition to seeking minimum
            start_rho_min_isr();
            return;
        }
        if (homing_ctrl_.step_count >= homing_ctrl_.max_steps) {
            homing_ctrl_.error = MotionError::HomingFailed;
            state_.store(HomingState::Error, std::memory_order_release);
            gptimer_stop(step_timer_);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
        break;

    case HomingState::RhoSeekingMin:
        if (rho_stall_triggered_) {
            homing_ctrl_.rho_max_steps = homing_ctrl_.step_count;
            rho_stall_triggered_ = false;
            gptimer_stop(step_timer_);
            // Transition to theta calibration
            start_theta_calibration_isr();
            return;
        }
        if (homing_ctrl_.step_count >= homing_ctrl_.max_steps) {
            homing_ctrl_.error = MotionError::HomingFailed;
            state_.store(HomingState::Error, std::memory_order_release);
            gptimer_stop(step_timer_);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
        break;

    case HomingState::ThetaBackingOff:
        // Check if we've backed off the Hall sensor
        if (!hall_triggered_ && gpio_get_level(PinConfig::THETA_HALL) == 1) {
            // Switched to seeking first edge
            state_.store(HomingState::ThetaSeekingFirstEdge, std::memory_order_release);
            gpio_set_level(PinConfig::THETA_DIR, 1);  // Forward direction
            gpio_set_level(PinConfig::RHO_DIR, 1);
            homing_ctrl_.step_count = 0;
            homing_ctrl_.theta_step_counter = 0;
        }
        break;

    case HomingState::ThetaSeekingFirstEdge:
        if (hall_triggered_) {
            hall_triggered_ = false;
            homing_ctrl_.first_hall_edge_found = true;
            homing_ctrl_.step_count = 0;
            homing_ctrl_.theta_step_counter = 0;
            state_.store(HomingState::ThetaSeekingSecondEdge, std::memory_order_release);
        }
        if (homing_ctrl_.step_count >= homing_ctrl_.max_steps) {
            homing_ctrl_.error = MotionError::HomingFailed;
            state_.store(HomingState::Error, std::memory_order_release);
            gptimer_stop(step_timer_);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
        break;

    case HomingState::ThetaSeekingSecondEdge:
        if (hall_triggered_) {
            homing_ctrl_.theta_steps_per_rot = homing_ctrl_.step_count;
            hall_triggered_ = false;
            state_.store(HomingState::Complete, std::memory_order_release);
            homing_active_.store(false, std::memory_order_release);
            gptimer_stop(step_timer_);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
        if (homing_ctrl_.step_count >= homing_ctrl_.max_steps) {
            homing_ctrl_.error = MotionError::HomingFailed;
            state_.store(HomingState::Error, std::memory_order_release);
            gptimer_stop(step_timer_);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
        break;

    case HomingState::Complete:
    case HomingState::Error:
    case HomingState::Idle:
    default:
        return;
    }

    // Check for abort
    if (abort_requested_.load(std::memory_order_acquire)) {
        homing_ctrl_.error = MotionError::EmergencyStop;
        state_.store(HomingState::Error, std::memory_order_release);
        homing_active_.store(false, std::memory_order_release);
        gptimer_stop(step_timer_);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    // Generate steps based on current state
    switch (current_state) {
    case HomingState::RhoSeekingMax:
    case HomingState::RhoSeekingMin:
        // Generate rho step
        gpio_set_level(PinConfig::RHO_STEP, 1);
        esp_rom_delay_us(2);
        gpio_set_level(PinConfig::RHO_STEP, 0);
        step_generated = true;
        break;

    case HomingState::ThetaBackingOff:
    case HomingState::ThetaSeekingFirstEdge:
    case HomingState::ThetaSeekingSecondEdge:
        // Generate theta step
        gpio_set_level(PinConfig::THETA_STEP, 1);
        esp_rom_delay_us(2);
        gpio_set_level(PinConfig::THETA_STEP, 0);

        // Coupled rho motion (rho follows theta at gear ratio)
        homing_ctrl_.theta_step_counter++;
        if (homing_ctrl_.theta_step_counter >= kGearRatio) {
            homing_ctrl_.theta_step_counter = 0;
            gpio_set_level(PinConfig::RHO_STEP, 1);
            esp_rom_delay_us(2);
            gpio_set_level(PinConfig::RHO_STEP, 0);
        }
        step_generated = true;
        break;

    default:
        break;
    }

    if (step_generated) {
        homing_ctrl_.step_count++;
    }
}

// =============================================================================
// Homing Phase Starters (ISR-safe)
// =============================================================================

void IRAM_ATTR HomingController::start_rho_max_isr() {
    homing_ctrl_.max_steps = kMaxHomingSteps;
    homing_ctrl_.step_count = 0;
    rho_stall_triggered_ = false;

    state_.store(HomingState::RhoSeekingMax, std::memory_order_release);
    gpio_set_level(PinConfig::RHO_DIR, 1);  // Move outward

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = kRhoStepIntervalUs,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };
    gptimer_set_alarm_action(step_timer_, &alarm_config);
    gptimer_set_raw_count(step_timer_, 0);
    gptimer_start(step_timer_);
}

void IRAM_ATTR HomingController::start_rho_min_isr() {
    homing_ctrl_.max_steps = kMaxHomingSteps;
    homing_ctrl_.step_count = 0;
    rho_stall_triggered_ = false;

    state_.store(HomingState::RhoSeekingMin, std::memory_order_release);
    gpio_set_level(PinConfig::RHO_DIR, 0);  // Move inward

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = kRhoStepIntervalUs,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };
    gptimer_set_alarm_action(step_timer_, &alarm_config);
    gptimer_set_raw_count(step_timer_, 0);
    gptimer_start(step_timer_);
}

void IRAM_ATTR HomingController::start_theta_calibration_isr() {
    homing_ctrl_.max_steps = kMaxHomingSteps;
    homing_ctrl_.step_count = 0;
    homing_ctrl_.theta_step_counter = 0;
    homing_ctrl_.first_hall_edge_found = false;
    hall_triggered_ = false;

    // Check if already on Hall sensor
    if (gpio_get_level(PinConfig::THETA_HALL) == 0) {
        state_.store(HomingState::ThetaBackingOff, std::memory_order_release);
        gpio_set_level(PinConfig::THETA_DIR, 0);  // Reverse direction
        gpio_set_level(PinConfig::RHO_DIR, 0);
    } else {
        state_.store(HomingState::ThetaSeekingFirstEdge, std::memory_order_release);
        gpio_set_level(PinConfig::THETA_DIR, 1);  // Forward direction
        gpio_set_level(PinConfig::RHO_DIR, 1);
    }

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = kThetaStepIntervalUs,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };
    gptimer_set_alarm_action(step_timer_, &alarm_config);
    gptimer_set_raw_count(step_timer_, 0);
    gptimer_start(step_timer_);
}

// =============================================================================
// Timer Control
// =============================================================================

void HomingController::start_timer(uint64_t interval_us) {
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = interval_us,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };
    gptimer_set_alarm_action(step_timer_, &alarm_config);
    gptimer_set_raw_count(step_timer_, 0);
    gptimer_start(step_timer_);
}

void HomingController::stop_timer() {
    gptimer_stop(step_timer_);
}

// =============================================================================
// Public Homing Functions
// =============================================================================

bool HomingController::is_hall_triggered() const {
    return gpio_get_level(PinConfig::THETA_HALL) == 0;
}

bool HomingController::is_rho_stalled() const {
    return rho_tmc_.is_stalled();
}

void HomingController::abort() {
    abort_requested_.store(true, std::memory_order_release);
    stop_timer();
    homing_active_.store(false, std::memory_order_release);
    state_.store(HomingState::Idle, std::memory_order_release);
}

HomingController::HomingResult HomingController::wait_for_completion() {
    // Wait for ISR to signal completion
    if (xSemaphoreTake(completion_sem_, pdMS_TO_TICKS(60000)) != pdTRUE) {
        // Timeout
        stop_timer();
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    HomingResult result;
    HomingState final_state = state_.load(std::memory_order_acquire);

    if (final_state == HomingState::Complete) {
        result.success = true;
        result.position_steps = homing_ctrl_.step_count;
    } else {
        result.error = homing_ctrl_.error;
    }

    return result;
}

HomingController::HomingResult HomingController::home_rho() {
    ESP_LOGI(TAG, "Starting rho homing...");

    homing_active_.store(true, std::memory_order_release);
    abort_requested_.store(false, std::memory_order_release);
    homing_ctrl_.error = MotionError::None;

    // Enable motors
    theta_.set_enabled(true);
    rho_.set_enabled(true);

    // Start seeking max (this will chain to min, then signal completion)
    // But we need a modified flow for rho-only homing
    homing_ctrl_.max_steps = kMaxHomingSteps;
    homing_ctrl_.step_count = 0;
    rho_stall_triggered_ = false;

    ESP_LOGI(TAG, "Seeking rho maximum...");
    state_.store(HomingState::RhoSeekingMax, std::memory_order_release);
    gpio_set_level(PinConfig::RHO_DIR, 1);
    start_timer(kRhoStepIntervalUs);

    // Wait for max to complete
    if (xSemaphoreTake(completion_sem_, pdMS_TO_TICKS(30000)) != pdTRUE) {
        stop_timer();
        homing_active_.store(false, std::memory_order_release);
        HomingResult result;
        result.error = MotionError::HomingFailed;
        ESP_LOGE(TAG, "Rho max homing timeout");
        return result;
    }

    // Check if max succeeded (state should be RhoSeekingMin now, started by ISR)
    // Actually for rho-only, we need to handle the chain differently
    // The ISR already started seeking min, so wait again

    vTaskDelay(pdMS_TO_TICKS(100));  // Brief pause between phases

    if (state_.load(std::memory_order_acquire) == HomingState::Error) {
        homing_active_.store(false, std::memory_order_release);
        HomingResult result;
        result.error = homing_ctrl_.error;
        return result;
    }

    // Wait for min to complete (ISR will signal and start theta, but we stop there for rho-only)
    if (xSemaphoreTake(completion_sem_, pdMS_TO_TICKS(30000)) != pdTRUE) {
        stop_timer();
        homing_active_.store(false, std::memory_order_release);
        HomingResult result;
        result.error = MotionError::HomingFailed;
        ESP_LOGE(TAG, "Rho min homing timeout");
        return result;
    }

    // For rho-only homing, stop here
    stop_timer();

    rho_max_steps_ = homing_ctrl_.rho_max_steps;
    rho_.reset_position();

    homing_active_.store(false, std::memory_order_release);
    state_.store(HomingState::Complete, std::memory_order_release);

    ESP_LOGI(TAG, "Rho homing complete: %ld steps travel", rho_max_steps_);

    HomingResult result;
    result.success = true;
    result.position_steps = rho_max_steps_;
    return result;
}

HomingController::HomingResult HomingController::home_theta() {
    ESP_LOGI(TAG, "Starting theta homing...");

    homing_active_.store(true, std::memory_order_release);
    abort_requested_.store(false, std::memory_order_release);
    homing_ctrl_.error = MotionError::None;

    // Enable motors
    theta_.set_enabled(true);
    rho_.set_enabled(true);

    // Start theta calibration directly
    start_theta_calibration_isr();

    auto result = wait_for_completion();

    if (result.success) {
        theta_steps_per_rotation_ = homing_ctrl_.theta_steps_per_rot;
        theta_.reset_position();
        ESP_LOGI(TAG, "Theta homing complete: %ld steps/rotation", theta_steps_per_rotation_);
    } else {
        ESP_LOGE(TAG, "Theta homing failed");
    }

    homing_active_.store(false, std::memory_order_release);
    return result;
}

Result<void> HomingController::home_all() {
    ESP_LOGI(TAG, "Starting full homing sequence...");

    homing_active_.store(true, std::memory_order_release);
    abort_requested_.store(false, std::memory_order_release);
    homing_ctrl_.error = MotionError::None;

    // Enable motors
    theta_.set_enabled(true);
    rho_.set_enabled(true);

    // Start the homing sequence (rho max -> rho min -> theta calibration)
    start_rho_max_isr();

    // Wait for full sequence to complete
    auto result = wait_for_completion();

    if (result.success) {
        rho_max_steps_ = homing_ctrl_.rho_max_steps;
        theta_steps_per_rotation_ = homing_ctrl_.theta_steps_per_rot;

        theta_.reset_position();
        rho_.reset_position();

        ESP_LOGI(TAG, "Homing complete - Rho: %ld steps, Theta: %ld steps/rev",
                 rho_max_steps_, theta_steps_per_rotation_);

        homing_active_.store(false, std::memory_order_release);
        return Result<void>::ok();
    }

    ESP_LOGE(TAG, "Homing failed");
    homing_active_.store(false, std::memory_order_release);
    return Result<void>::err(result.error);
}

} // namespace sand_table
