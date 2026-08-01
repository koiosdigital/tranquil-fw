#include "homing_controller.h"
#include "config_manager.h"
#include "stepper_driver.h"
#include "esp_log.h"
#include <cmath>
#include <ctime>

namespace sand_table {

    static const char* TAG = "HomingController";

    // Global instance pointer for ISR access
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

        // Remove ISR handlers
        gpio_isr_handler_remove(PinConfig::THETA_HALL);
        gpio_isr_handler_remove(PinConfig::RHO_DIAG);

        g_homing_instance = nullptr;
    }

    Result<void> HomingController::init() {
        ESP_LOGI(TAG, "Initializing homing controller...");
        ESP_LOGI(TAG, "  Hall sensor pin: GPIO%d", PinConfig::THETA_HALL);
        ESP_LOGI(TAG, "  Rho DIAG pin: GPIO%d", PinConfig::RHO_DIAG);
        ESP_LOGI(TAG, "  StallGuard threshold: %d", MotionConfig::stallguard_threshold());
        ESP_LOGI(TAG, "  Rho step interval: %lu us", kRhoHomingIntervalUs);
        ESP_LOGI(TAG, "  Theta step interval: %lu us", kThetaHomingIntervalUs);

        // Configure Hall sensor GPIO with interrupt
        gpio_config_t hall_config = {
            .pin_bit_mask = (1ULL << PinConfig::THETA_HALL),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,  // Active low - trigger on falling edge
        };

        if (gpio_config(&hall_config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure Hall sensor GPIO");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure StallGuard DIAG pin with interrupt
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

        // Configure StallGuard for rho axis
        ESP_LOGI(TAG, "Configuring TMC2209 StallGuard...");
        (void)rho_tmc_.set_stallguard_threshold(MotionConfig::stallguard_threshold());
        // CRITICAL: Set TCOOLTHRS to enable StallGuard at all speeds
        (void)rho_tmc_.set_stallguard_min_speed(0xFFFFF);

        // Disable both ISRs initially - they'll be enabled only during the appropriate homing phase
        // This prevents them from interfering with normal motion or each other during homing
        gpio_intr_disable(PinConfig::THETA_HALL);
        gpio_intr_disable(PinConfig::RHO_DIAG);

        int diag_level = gpio_get_level(PinConfig::RHO_DIAG);
        ESP_LOGI(TAG, "Initial DIAG pin level: %d", diag_level);
        ESP_LOGI(TAG, "Homing controller initialized");

        return Result<void>::ok();
    }

    void IRAM_ATTR HomingController::hall_isr_handler(void* arg) {
        auto* self = static_cast<HomingController*>(arg);
        self->hall_triggered_ = true;
        // Immediately stop RMT transmission from ISR context
        if (self->stepper_controller_) {
            self->stepper_controller_->emergency_stop_from_isr();
        }
    }

    void IRAM_ATTR HomingController::diag_isr_handler(void* arg) {
        auto* self = static_cast<HomingController*>(arg);
        self->rho_stall_triggered_ = true;
        // Immediately stop RMT transmission from ISR context
        if (self->stepper_controller_) {
            self->stepper_controller_->emergency_stop_from_isr();
        }
    }

    double HomingController::rho_steps_per_theta_step() const {
        int32_t theta_rot = theta_steps_per_rotation_;
        if (theta_rot <= 0) {
            const auto& calib = ConfigManager::instance().calibration();
            if (calib.is_valid) {
                theta_rot = calib.theta_steps_per_rotation;
            }
        }
        if (theta_rot <= 0) {
            theta_rot = MechanicalConfig::NOMINAL_STEPS_PER_THETA_ROTATION;
        }
        return static_cast<double>(MechanicalConfig::effective_steps_per_rev()) /
            static_cast<double>(theta_rot);
    }

    bool HomingController::is_hall_triggered() const {
        return gpio_get_level(PinConfig::THETA_HALL) == 0;
    }

    bool HomingController::is_rho_stalled() const {
        return gpio_get_level(PinConfig::RHO_DIAG) == 1;
    }

    void HomingController::enable_hall_isr() {
        gpio_intr_enable(PinConfig::THETA_HALL);
    }

    void HomingController::disable_hall_isr() {
        gpio_intr_disable(PinConfig::THETA_HALL);
    }

    void HomingController::enable_diag_isr() {
        gpio_intr_enable(PinConfig::RHO_DIAG);
    }

    void HomingController::disable_diag_isr() {
        gpio_intr_disable(PinConfig::RHO_DIAG);
    }

    void HomingController::abort() {
        abort_requested_.store(true, std::memory_order_release);
        if (stepper_controller_) {
            stepper_controller_->emergency_stop();
        }
        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Idle, std::memory_order_release);
    }

    HomingController::HomingResult HomingController::seek_rho_max() {
        ESP_LOGI(TAG, "Seeking rho max at %lu us/step (%lu steps/sec)...",
            kRhoHomingIntervalUs, 1000000UL / kRhoHomingIntervalUs);
        state_.store(HomingState::RhoSeekingMax, std::memory_order_release);

        if (!stepper_controller_) {
            ESP_LOGE(TAG, "No stepper controller set");
            HomingResult result;
            result.error = MotionError::InvalidState;
            return result;
        }

        // Set direction outward; keep both sensor ISRs off for the blanking
        // move below
        rho_.set_direction(true);
        disable_hall_isr();
        disable_diag_isr();

        // Record starting position (before blanking so it counts as travel)
        int32_t rho_before = rho_.position();

        // Blanking: StallGuard is invalid at standstill/spin-up and DIAG can
        // still be asserted from a previous stall, which fired the ISR
        // instantly and reported a stall after ~0 steps. Move a short
        // distance blind before arming stall detection.
        (void)stepper_controller_->execute_constant_speed(
            0, kStallBlankingSteps, kRhoHomingIntervalUs);

        rho_stall_triggered_ = false;
        enable_diag_isr();

        // Execute a single large move - ISR will stop when stall detected
        (void)stepper_controller_->execute_constant_speed(
            0, static_cast<int32_t>(kMaxHomingSteps), kRhoHomingIntervalUs);

        // Calculate actual steps moved
        int32_t rho_after = rho_.position();
        int32_t steps_moved = std::abs(rho_after - rho_before);

        // Check if aborted
        if (abort_requested_.load(std::memory_order_acquire)) {
            HomingResult result;
            result.error = MotionError::EmergencyStop;
            return result;
        }

        // Check if stall was detected
        if (rho_stall_triggered_) {
            ESP_LOGI(TAG, "Rho max found at step %ld", steps_moved);
            HomingResult result;
            result.success = true;
            result.position_steps = steps_moved;
            return result;
        }

        // If we completed all steps without stall, that's a failure
        ESP_LOGE(TAG, "Rho max not found within %lu steps", kMaxHomingSteps);
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    HomingController::HomingResult HomingController::seek_rho_min() {
        ESP_LOGI(TAG, "Seeking rho min...");
        state_.store(HomingState::RhoSeekingMin, std::memory_order_release);

        if (!stepper_controller_) {
            ESP_LOGE(TAG, "No stepper controller set");
            HomingResult result;
            result.error = MotionError::InvalidState;
            return result;
        }

        // Set direction inward; keep both sensor ISRs off for the blanking
        // move below
        rho_.set_direction(false);
        disable_hall_isr();
        disable_diag_isr();

        // Brief pause to let StallGuard clear from previous stall
        vTaskDelay(pdMS_TO_TICKS(100));

        // Record starting position (before blanking so it counts as travel)
        int32_t rho_before = rho_.position();

        // Blanking: this seek starts pressed against the max hard stop with
        // DIAG potentially still asserted and StallGuard invalid at spin-up.
        // Without it the "stall" fired immediately and rho_max was measured
        // as ~0 steps of travel (and then saved as valid calibration).
        (void)stepper_controller_->execute_constant_speed(
            0, -kStallBlankingSteps, kRhoHomingIntervalUs);

        rho_stall_triggered_ = false;
        enable_diag_isr();

        // Execute a single large move inward - ISR will stop when stall detected
        (void)stepper_controller_->execute_constant_speed(
            0, -static_cast<int32_t>(kMaxHomingSteps), kRhoHomingIntervalUs);

        // Calculate actual steps moved
        int32_t rho_after = rho_.position();
        int32_t steps_moved = std::abs(rho_after - rho_before);

        // Check if aborted
        if (abort_requested_.load(std::memory_order_acquire)) {
            HomingResult result;
            result.error = MotionError::EmergencyStop;
            return result;
        }

        // Check if stall was detected
        if (rho_stall_triggered_) {
            // A stall this early is a false trigger, not the inner hard
            // stop - saving it would calibrate rho_max to ~0 and disable
            // all rho motion.
            if (steps_moved < kMinRhoTravelSteps) {
                ESP_LOGE(TAG, "Rho min stall after only %ld steps (min %ld) - "
                    "false trigger, calibration rejected", steps_moved,
                    kMinRhoTravelSteps);
                HomingResult result;
                result.error = MotionError::HomingFailed;
                return result;
            }
            ESP_LOGI(TAG, "Rho min found - total travel: %ld steps", steps_moved);
            HomingResult result;
            result.success = true;
            result.position_steps = steps_moved;
            return result;
        }

        // If we completed all steps without stall, that's a failure
        ESP_LOGE(TAG, "Rho min not found within %lu steps", kMaxHomingSteps);
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    HomingController::HomingResult HomingController::calibrate_theta() {
        ESP_LOGI(TAG, "Calibrating theta...");
        state_.store(HomingState::ThetaCalibrating, std::memory_order_release);

        if (!stepper_controller_) {
            ESP_LOGE(TAG, "No stepper controller set");
            HomingResult result;
            result.error = MotionError::InvalidState;
            return result;
        }

        // Disable both ISRs during backoff phase
        disable_hall_isr();
        disable_diag_isr();

        // If already on Hall sensor, back off first
        // Hall ISR won't trigger during backoff (triggers on NEGEDGE = entering field)
        // We just need to move backward enough to exit the hall field
        if (is_hall_triggered()) {
            ESP_LOGD(TAG, "Backing off Hall sensor...");
            hall_triggered_ = false;
            theta_.set_direction(false);  // Reverse
            rho_.set_direction(false);    // Rho follows theta due to coupling

            // Move backward a moderate amount (hall field is small)
            // Use ~5000 theta steps with coupled rho compensation
            constexpr int32_t kBackoffSteps = 5000;
            int32_t rho_backoff = static_cast<int32_t>(std::lround(
                kBackoffSteps * rho_steps_per_theta_step()));

            (void)stepper_controller_->execute_constant_speed(
                -kBackoffSteps, -rho_backoff, kThetaHomingIntervalUs);

            if (abort_requested_.load(std::memory_order_acquire)) {
                HomingResult result;
                result.error = MotionError::EmergencyStop;
                return result;
            }

            // Verify we're off the hall sensor
            if (is_hall_triggered()) {
                ESP_LOGE(TAG, "Failed to back off Hall sensor");
                HomingResult result;
                result.error = MotionError::HomingFailed;
                return result;
            }
        }

        // Only enable hall ISR for edge detection
        enable_hall_isr();

        // Seek forward to first Hall edge
        // ISR will stop when we enter the hall field (NEGEDGE)
        ESP_LOGD(TAG, "Seeking first Hall edge...");
        hall_triggered_ = false;
        theta_.set_direction(true);   // Forward
        rho_.set_direction(true);     // Rho follows theta

        int32_t theta_before = theta_.position();
        int32_t rho_steps_for_full_rotation = static_cast<int32_t>(std::lround(
            static_cast<double>(kMaxHomingSteps) * rho_steps_per_theta_step()));

        // Execute large forward move - ISR stops at first edge
        (void)stepper_controller_->execute_constant_speed(
            static_cast<int32_t>(kMaxHomingSteps), rho_steps_for_full_rotation, kThetaHomingIntervalUs);

        if (abort_requested_.load(std::memory_order_acquire)) {
            HomingResult result;
            result.error = MotionError::EmergencyStop;
            return result;
        }

        if (!hall_triggered_) {
            ESP_LOGE(TAG, "First Hall edge not found");
            HomingResult result;
            result.error = MotionError::HomingFailed;
            return result;
        }

        int32_t first_edge_position = theta_.position();
        ESP_LOGD(TAG, "First Hall edge found at step %ld", first_edge_position - theta_before);

        // Seek forward to second Hall edge (full rotation measurement)
        // We're currently ON the hall sensor, need to exit it and re-enter
        hall_triggered_ = false;  // Clear for second edge detection

        theta_before = theta_.position();

        // Execute another large forward move - ISR stops at second edge
        (void)stepper_controller_->execute_constant_speed(
            static_cast<int32_t>(kMaxHomingSteps), rho_steps_for_full_rotation, kThetaHomingIntervalUs);

        if (abort_requested_.load(std::memory_order_acquire)) {
            HomingResult result;
            result.error = MotionError::EmergencyStop;
            return result;
        }

        if (!hall_triggered_) {
            ESP_LOGE(TAG, "Second Hall edge not found");
            HomingResult result;
            result.error = MotionError::HomingFailed;
            return result;
        }

        // Steps between edges = one full rotation
        int32_t steps_per_rotation = theta_.position() - theta_before;

        // Reject implausibly short measurements instead of saving them:
        // a spurious hall edge here would otherwise become the coupling
        // denominator and persist in NVS (mirrors the rho false-trigger
        // rejection above).
        if (steps_per_rotation < kMinThetaStepsPerRotation) {
            ESP_LOGE(TAG, "Theta calibration measured %ld steps/rotation "
                "(min %ld) - spurious hall edge, calibration rejected",
                steps_per_rotation, kMinThetaStepsPerRotation);
            HomingResult result;
            result.error = MotionError::HomingFailed;
            return result;
        }

        ESP_LOGI(TAG, "Theta calibration complete: %ld steps/rotation", steps_per_rotation);

        HomingResult result;
        result.success = true;
        result.position_steps = steps_per_rotation;
        return result;
    }

    HomingController::HomingResult HomingController::home_rho() {
        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        // Seek max first
        auto max_result = seek_rho_max();
        if (!max_result.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return max_result;
        }

        // Brief pause
        vTaskDelay(pdMS_TO_TICKS(100));

        // Seek min to measure travel
        auto min_result = seek_rho_min();
        if (!min_result.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return min_result;
        }

        rho_max_steps_ = min_result.position_steps;
        rho_.reset_position();

        // Disable ISRs after homing complete
        disable_hall_isr();
        disable_diag_isr();

        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Complete, std::memory_order_release);

        ESP_LOGI(TAG, "Rho homing complete: %ld steps travel", rho_max_steps_);
        return min_result;
    }

    HomingController::HomingResult HomingController::home_theta() {
        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        auto result = calibrate_theta();

        // Disable ISRs after homing (success or failure)
        disable_hall_isr();
        disable_diag_isr();

        if (result.success) {
            theta_steps_per_rotation_ = result.position_steps;
            theta_.reset_position();
            ESP_LOGI(TAG, "Theta homing complete: %ld steps/rotation", theta_steps_per_rotation_);
        }
        else {
            ESP_LOGE(TAG, "Theta homing failed");
            state_.store(HomingState::Error, std::memory_order_release);
        }

        homing_active_.store(false, std::memory_order_release);
        return result;
    }

    bool HomingController::calibration_plausible(const CalibrationData& calib) {
        return calib.is_valid &&
            calib.theta_steps_per_rotation >= kMinThetaStepsPerRotation &&
            calib.rho_max_steps >= kMinRhoTravelSteps;
    }

    Result<void> HomingController::home_all(bool force_full) {
        // Check if we can use cached calibration
        const auto& calib = ConfigManager::instance().calibration();

        if (!force_full && calib.is_valid && !calibration_plausible(calib)) {
            // Marked valid but out of band (torn NVS write, legacy garbage):
            // recalibrate rather than home against it.
            ESP_LOGW(TAG, "Cached calibration implausible (theta=%ld, rho_max=%ld)"
                " - forcing full calibration",
                calib.theta_steps_per_rotation, calib.rho_max_steps);
            force_full = true;
        }

        if (!force_full && calib.is_valid) {
            ESP_LOGI(TAG, "Using cached calibration (theta=%ld, rho_max=%ld)",
                calib.theta_steps_per_rotation, calib.rho_max_steps);
            return home_quick();
        }

        ESP_LOGI(TAG, "Performing full calibration homing...");

        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        // Home rho first (for safety - moves to known position)
        auto rho_max = seek_rho_max();
        if (!rho_max.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(rho_max.error);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        auto rho_min = seek_rho_min();
        if (!rho_min.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(rho_min.error);
        }

        rho_max_steps_ = rho_min.position_steps;

        vTaskDelay(pdMS_TO_TICKS(100));

        // Home theta
        auto theta_result = calibrate_theta();
        if (!theta_result.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(theta_result.error);
        }

        theta_steps_per_rotation_ = theta_result.position_steps;

        // Reset positions
        theta_.reset_position();
        rho_.reset_position();

        // Save calibration to NVS
        save_calibration_to_nvs();

        // Disable both ISRs after homing complete
        disable_hall_isr();
        disable_diag_isr();

        ESP_LOGI(TAG, "Full homing complete - Rho: %ld steps, Theta: %ld steps/rev",
            rho_max_steps_, theta_steps_per_rotation_);

        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Complete, std::memory_order_release);

        return Result<void>::ok();
    }

    Result<void> HomingController::home_quick() {
        const auto& calib = ConfigManager::instance().calibration();

        if (!calibration_plausible(calib)) {
            ESP_LOGW(TAG, "No plausible calibration for quick home, falling back to full");
            return home_all(true);
        }

        ESP_LOGI(TAG, "Quick homing with cached calibration...");

        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Load cached values
        theta_steps_per_rotation_ = calib.theta_steps_per_rotation;
        rho_max_steps_ = calib.rho_max_steps;

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        rho_.set_direction(true);  // Outwards
        rho_.step_fixed_rate(50, 500);

        // Home rho to center using known max
        auto rho_result = home_rho_to_center(rho_max_steps_);
        if (!rho_result.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(rho_result.error);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        // Home theta to single Hall edge
        auto theta_result = home_theta_single();
        if (!theta_result.success) {
            disable_hall_isr();
            disable_diag_isr();
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(theta_result.error);
        }

        // Reset positions
        theta_.reset_position();
        rho_.reset_position();

        // Disable both ISRs after homing complete
        disable_hall_isr();
        disable_diag_isr();

        ESP_LOGI(TAG, "Quick homing complete");

        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Complete, std::memory_order_release);

        return Result<void>::ok();
    }

    Result<void> HomingController::force_recalibrate() {
        ESP_LOGI(TAG, "Force recalibrating - clearing cached calibration");

        // Clear calibration in NVS
        ConfigManager::instance().clear_calibration();

        // Also drop this run's in-RAM values: force_recalibrate means the
        // caller distrusts them (regear, corrupt measurement), and
        // rho_steps_per_theta_step() would otherwise keep steering the
        // recalibration's own seek compensation from the value being
        // replaced. The seeks fall back to the nominal estimate instead.
        theta_steps_per_rotation_ = 0;
        rho_max_steps_ = 0;

        // Perform full homing
        return home_all(true);
    }

    HomingController::HomingResult HomingController::home_rho_to_center(int32_t rho_max_steps) {
        ESP_LOGI(TAG, "Homing rho to center (max=%ld)...", rho_max_steps);
        state_.store(HomingState::RhoSeekingMin, std::memory_order_release);

        if (!stepper_controller_) {
            ESP_LOGE(TAG, "No stepper controller set");
            HomingResult result;
            result.error = MotionError::InvalidState;
            return result;
        }

        // Move inward until we stall at center (rho=0.0)
        rho_stall_triggered_ = false;
        rho_.set_direction(false);  // Inward

        // Only enable stallguard ISR for this phase
        disable_hall_isr();
        enable_diag_isr();

        // Record starting position
        int32_t rho_before = rho_.position();

        // Execute a single large move inward - ISR will stop when stall detected
        (void)stepper_controller_->execute_constant_speed(
            0, -static_cast<int32_t>(kMaxHomingSteps), kRhoHomingIntervalUs);

        // Calculate actual steps moved
        int32_t rho_after = rho_.position();
        int32_t steps_moved = std::abs(rho_after - rho_before);

        // Check if aborted
        if (abort_requested_.load(std::memory_order_acquire)) {
            HomingResult result;
            result.error = MotionError::EmergencyStop;
            return result;
        }

        // Check if stall was detected
        if (!rho_stall_triggered_) {
            ESP_LOGE(TAG, "Rho center not found");
            HomingResult result;
            result.error = MotionError::HomingFailed;
            return result;
        }

        ESP_LOGI(TAG, "Rho homed to center after %ld steps", steps_moved);

        HomingResult result;
        result.success = true;
        result.position_steps = 0;
        return result;
    }

    HomingController::HomingResult HomingController::home_theta_single() {
        ESP_LOGI(TAG, "Quick theta homing (single edge)...");
        state_.store(HomingState::ThetaCalibrating, std::memory_order_release);

        if (!stepper_controller_) {
            ESP_LOGE(TAG, "No stepper controller set");
            HomingResult result;
            result.error = MotionError::InvalidState;
            return result;
        }

        // Disable both ISRs during backoff phase
        disable_hall_isr();
        disable_diag_isr();

        // If already on Hall sensor, back off first
        // Hall ISR won't trigger during backoff (triggers on NEGEDGE = entering field)
        if (is_hall_triggered()) {
            ESP_LOGD(TAG, "Backing off Hall sensor...");
            hall_triggered_ = false;
            theta_.set_direction(false);  // Reverse
            rho_.set_direction(false);    // Rho follows theta due to coupling

            // Move backward a moderate amount (hall field is small)
            constexpr int32_t kBackoffSteps = 5000;
            int32_t rho_backoff = static_cast<int32_t>(std::lround(
                kBackoffSteps * rho_steps_per_theta_step()));

            (void)stepper_controller_->execute_constant_speed(
                -kBackoffSteps, -rho_backoff, kThetaHomingIntervalUs);

            if (abort_requested_.load(std::memory_order_acquire)) {
                HomingResult result;
                result.error = MotionError::EmergencyStop;
                return result;
            }

            // Verify we're off the hall sensor
            if (is_hall_triggered()) {
                ESP_LOGE(TAG, "Failed to back off Hall sensor");
                HomingResult result;
                result.error = MotionError::HomingFailed;
                return result;
            }
        }

        // Only enable hall ISR for edge detection
        enable_hall_isr();

        // Seek forward to first Hall edge only (no full rotation measurement)
        // ISR will stop when we enter the hall field (NEGEDGE)
        ESP_LOGD(TAG, "Seeking Hall edge...");
        hall_triggered_ = false;
        theta_.set_direction(true);   // Forward
        rho_.set_direction(true);     // Rho follows theta

        int32_t theta_before = theta_.position();
        int32_t rho_steps_for_full_rotation = static_cast<int32_t>(std::lround(
            static_cast<double>(kMaxHomingSteps) * rho_steps_per_theta_step()));

        // Execute large forward move - ISR stops at hall edge
        (void)stepper_controller_->execute_constant_speed(
            static_cast<int32_t>(kMaxHomingSteps), rho_steps_for_full_rotation, kThetaHomingIntervalUs);

        // Calculate actual steps moved
        int32_t theta_after = theta_.position();
        int32_t steps_moved = std::abs(theta_after - theta_before);

        // Check if aborted
        if (abort_requested_.load(std::memory_order_acquire)) {
            HomingResult result;
            result.error = MotionError::EmergencyStop;
            return result;
        }

        // Check if hall was detected
        if (hall_triggered_) {
            ESP_LOGI(TAG, "Theta home found at step %ld", steps_moved);
            HomingResult result;
            result.success = true;
            result.position_steps = steps_moved;
            return result;
        }

        ESP_LOGE(TAG, "Theta home not found");
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    void HomingController::save_calibration_to_nvs() {
        CalibrationData calib;
        calib.theta_steps_per_rotation = theta_steps_per_rotation_;
        calib.rho_max_steps = rho_max_steps_;
        calib.is_valid = true;
        calib.timestamp = static_cast<uint32_t>(time(nullptr));

        esp_err_t err = ConfigManager::instance().save_calibration(calib);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save calibration to NVS: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(TAG, "Calibration saved to NVS");
        }
    }

} // namespace sand_table
