#include "motion_controller.h"
#include "stepper_driver.h"
#include "homing_controller.h"
#include "coordinate_transformer.h"
#include "path_planner.h"
#include "velocity_planner.h"

#include "esp_log.h"

#include <cmath>
#include <cstdio>

namespace sand_table {

    static const char* TAG = "MotionController";

    namespace {
        // IHOLD as a percentage of IRUN. The arm can't backdrive, so a reduced
        // holding current is enough and keeps the drivers cool (matches the
        // previous IHOLD = IRUN/2 intent).
        constexpr uint8_t kHoldCurrentPercent = 50;

        // RAII increment of the abort-planning depth: while held,
        // enqueue_segment() refuses segments so a task blocked mid-plan
        // unwinds (see MotionController::abort_planning_depth_).
        class AbortPlanningScope {
        public:
            explicit AbortPlanningScope(std::atomic<int>& depth) : depth_(depth) {
                depth_.fetch_add(1, std::memory_order_acq_rel);
            }
            ~AbortPlanningScope() {
                depth_.fetch_sub(1, std::memory_order_acq_rel);
            }
            AbortPlanningScope(const AbortPlanningScope&) = delete;
            AbortPlanningScope& operator=(const AbortPlanningScope&) = delete;
        private:
            std::atomic<int>& depth_;
        };

        // Map a configured microstep count to the TMC2209 MRES setting.
        tmc::MicrostepResolution microsteps_to_mres(uint32_t microsteps) {
            switch (microsteps) {
                case 1:   return tmc::MicrostepResolution::Full;
                case 2:   return tmc::MicrostepResolution::Half;
                case 4:   return tmc::MicrostepResolution::Quarter;
                case 8:   return tmc::MicrostepResolution::Eighth;
                case 16:  return tmc::MicrostepResolution::Sixteenth;
                case 32:  return tmc::MicrostepResolution::ThirtySecond;
                case 64:  return tmc::MicrostepResolution::SixtyFourth;
                case 128: return tmc::MicrostepResolution::OneTwentyEighth;
                case 256: return tmc::MicrostepResolution::TwoFiftySixth;
                default:
                    ESP_LOGW(TAG, "Unsupported microsteps=%lu, using 1/16",
                        static_cast<unsigned long>(microsteps));
                    return tmc::MicrostepResolution::Sixteenth;
            }
        }
    }  // namespace

    MotionController::MotionController()
        : path_planner_(std::make_unique<PathPlanner>())
        , velocity_planner_(std::make_unique<VelocityPlanner>())
        , transformer_(std::make_unique<CoordinateTransformer>())
    {
        planner_mutex_ = xSemaphoreCreateMutex();
        motor_power_mutex_ = xSemaphoreCreateMutex();
        plan_mutex_ = xSemaphoreCreateMutex();

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
        if (plan_mutex_) {
            vSemaphoreDelete(plan_mutex_);
            plan_mutex_ = nullptr;
        }
    }

    Result<void> MotionController::init() {
        // Initialize TMC UART bus
        auto tmc_result = init_tmc();
        if (tmc_result.is_err()) {
            return tmc_result;
        }

        // Create stepper drivers. invert_dir applies the Kconfig
        // ROBOT_*_INVERT_DIRECTION flags at the DIR-pin level so all motion
        // code stays in logical directions (positive rho = outward).
        // Internal RAM: the RMT TX-done ISR credits their position counters.
        theta_stepper_ = make_internal<StepperDriver>(
            StepperDriver::Pins{
                PinConfig::THETA_STEP,
                PinConfig::THETA_DIR,
                PinConfig::THETA_ENABLE,
                PinConfig::THETA_INVERT_DIR
            },
            "Theta"
        );

        rho_stepper_ = make_internal<StepperDriver>(
            StepperDriver::Pins{
                PinConfig::RHO_STEP,
                PinConfig::RHO_DIR,
                PinConfig::RHO_ENABLE,
                PinConfig::RHO_INVERT_DIR
            },
            "Rho"
        );

        if (!theta_stepper_ || !rho_stepper_) {
            ESP_LOGE(TAG, "Failed to allocate stepper drivers in internal RAM");
            return Result<void>::err(MotionError::HardwareFault);
        }

        ESP_LOGI(TAG, "Direction inversion: theta=%d, rho=%d",
            PinConfig::THETA_INVERT_DIR ? 1 : 0, PinConfig::RHO_INVERT_DIR ? 1 : 0);

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

        // Apply the NVS motion tunables now that the timer they program
        // exists. Calling this from init_tmc() (before the timer) was a
        // silent no-op, so the persisted idle timeout never applied at boot
        // and the compile-time default ran until the first settings save.
        apply_motion_tunables();

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

        (void)theta_tmc_->set_stealthchop_enable(true);
        (void)theta_tmc_->set_stealthchop_threshold(0);

        // Initialize rho TMC
        if (rho_tmc_->initialize() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize rho TMC2209");
            return Result<void>::err(MotionError::HardwareFault);
        }

        (void)rho_tmc_->set_stealthchop_enable(true);
        (void)rho_tmc_->set_stealthchop_threshold(0);

        // Microstep resolution from runtime config. Applied here (init) only:
        // the coupling and calibration are expressed in steps at this
        // resolution, so changing it requires a reboot + re-home. The
        // configured `microsteps` therefore must match what the coupling math
        // (effective_steps_per_rev) assumes.
        const tmc::MicrostepResolution mres =
            microsteps_to_mres(MechanicalConfig::microsteps());
        (void)theta_tmc_->set_microstep_resolution(mres);
        (void)rho_tmc_->set_microstep_resolution(mres);

        // Snapshot the step scale the drivers now run at. The transformer,
        // path planner and homing controller read THIS until reboot (see
        // MechanicalConfig::applied_effective_steps_per_rev): a runtime
        // PATCH of steps_per_rev/microsteps changes the NVS value but not
        // the resolution programmed above, so using the live value would
        // desync the coupling compensation from the physical drive.
        applied_steps_per_rev_ = MechanicalConfig::effective_steps_per_rev();
        MechanicalConfig::set_applied_effective_steps_per_rev(applied_steps_per_rev_);

        // Motor run/hold current from runtime config. (The motion tunables
        // are applied from init() once the inactivity timer exists.)
        apply_motor_config();

        ESP_LOGI(TAG, "TMC2209 drivers initialized (microsteps=%lu, %lu steps/motor rev applied)",
            static_cast<unsigned long>(MechanicalConfig::microsteps()),
            static_cast<unsigned long>(applied_steps_per_rev_));
        return Result<void>::ok();
    }

    void MotionController::apply_motor_config() {
        if (!theta_tmc_ || !rho_tmc_) {
            return;  // init_tmc() not run yet
        }

        // set_motor_current() sets IRUN and IHOLD to the same scale;
        // set_hold_current_percentage() then reduces IHOLD.
        const uint16_t theta_ma = MotionConfig::theta_irun_ma();
        const uint16_t rho_ma = MotionConfig::rho_irun_ma();

        (void)theta_tmc_->set_motor_current(theta_ma);
        (void)theta_tmc_->set_hold_current_percentage(kHoldCurrentPercent);
        (void)rho_tmc_->set_motor_current(rho_ma);
        (void)rho_tmc_->set_hold_current_percentage(kHoldCurrentPercent);

        ESP_LOGI(TAG, "Applied motor current: theta=%umA, rho=%umA (hold %u%%)",
            theta_ma, rho_ma, static_cast<unsigned>(kHoldCurrentPercent));
    }

    void MotionController::log_stallguard_diagnostics() {
        if (!rho_tmc_) {
            printf("{\"error\":true,\"message\":\"TMC not initialized\"}\n");
            return;
        }

        const auto ver = rho_tmc_->read_version();       // UART read: proves the link
        const bool comms_ok = ver.has_value();
        const uint8_t sgthrs = MotionConfig::stallguard_threshold();
        const uint16_t sg_result = rho_tmc_->get_stallguard_result();
        const int diag_level = gpio_get_level(PinConfig::RHO_DIAG);
        const bool stalled = rho_tmc_->is_stalled();
        const auto status = rho_tmc_->get_driver_status();

        printf("{\"error\":false,"
               "\"rho_tmc_addr\":%u,"
               "\"comms_ok\":%s,"
               "\"ioin_version\":%d,"
               "\"version_expected\":%d,"
               "\"sgthrs\":%u,"
               "\"diag_trips_at_sg_result\":%u,"
               "\"sg_result\":%u,"
               "\"diag_level\":%d,"
               "\"stalled\":%s,"
               "\"drv_status\":%lu}\n",
            rho_tmc_->address(),
            comms_ok ? "true" : "false",
            comms_ok ? static_cast<int>(*ver) : -1,
            static_cast<int>(tmc::TMC2209Stepper::kExpectedVersion),
            sgthrs,
            static_cast<unsigned>(sgthrs) * 2u,
            sg_result,
            diag_level,
            stalled ? "true" : "false",
            status.has_value() ? static_cast<unsigned long>(status->raw) : 0UL);

        if (!comms_ok) {
            ESP_LOGE(TAG, "SGtest: rho TMC (addr %u) not answering over UART - "
                "StallGuard is UNCONFIGURED (SGTHRS/TCOOLTHRS at reset 0), so no "
                "home can detect a stall. Check TMC UART pins/wiring and "
                "RHO_TMC_ADDR.", rho_tmc_->address());
        }
        else {
            ESP_LOGI(TAG, "SGtest: rho TMC ver=0x%02X SGTHRS=%u (DIAG trips when "
                "SG_RESULT<=%u) live SG_RESULT=%u DIAG=%d stalled=%d",
                *ver, sgthrs, sgthrs * 2, sg_result, diag_level, stalled ? 1 : 0);
            ESP_LOGI(TAG, "SGtest note: SG_RESULT is only valid while the motor "
                "is stepping - 0 at standstill is normal. Run sgtest during a "
                "move (e.g. rotate) to read the live load value.");
        }
    }

    bool MotionController::apply_stallguard_config() {
        if (!homing_controller_) {
            return false;
        }
        return homing_controller_->configure_stallguard();
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
        const bool was_running = running_.exchange(false, std::memory_order_acq_rel);

        if (was_running && stepper_task_) {
            // Join the stepper task: it may be inside a blocking segment
            // execute() and it uses planner_mutex_/plan_mutex_ — the
            // destructor deletes those, so wait for its exit acknowledgement
            // rather than a fixed delay. Wake it out of the sequencer first so
            // the join is bounded by one chunk, not a whole segment.
            if (stepper_controller_) {
                stepper_controller_->emergency_stop();
            }
            for (int i = 0; i < 1000 && !stepper_task_exited_.load(std::memory_order_acquire); ++i) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (!stepper_task_exited_.load(std::memory_order_acquire)) {
                ESP_LOGE(TAG, "Stepper task did not exit within 10s");
            }
            stepper_task_ = nullptr;
        }

        if (stepper_controller_) {
            disable_motors();
        }
        state_.store(SystemState::Idle, std::memory_order_release);
    }

    void MotionController::stepper_task_entry(void* arg) {
        auto* self = static_cast<MotionController*>(arg);
        self->stepper_task_loop();
    }

    void MotionController::discard_queued_motion() {
        segment_queue_.clear();
        if (xSemaphoreTake(planner_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            velocity_planner_->clear();
            xSemaphoreGive(planner_mutex_);
        }
        else {
            ESP_LOGW(TAG, "discard: planner mutex busy, planner not cleared");
        }
    }

    void MotionController::resync_planner_from_physical() {
        // Lock order: plan_mutex_ only (planner_mutex_ must NOT be held by
        // the caller). Bounded wait: a plan_*_move blocked on backpressure
        // returns as soon as abort_planning_depth_ > 0 (callers hold an
        // AbortPlanningScope across this), so this normally succeeds at
        // once; the fallback resyncs anyway rather than leaving the planner
        // on a position the discarded segments never reached.
        const bool locked = (xSemaphoreTake(plan_mutex_, pdMS_TO_TICKS(500)) == pdTRUE);
        if (!locked) {
            ESP_LOGW(TAG, "resync: a planning call did not unwind in time, resyncing anyway");
        }
        path_planner_->set_current_position(get_position());
        // The fractional step remainders belong to the plan just discarded.
        transformer_->reset_accumulators();
        if (locked) {
            xSemaphoreGive(plan_mutex_);
        }
    }

    void MotionController::stepper_task_loop() {
        // Raise the sequencer stop ONCE per e-stop latch. The stop flag
        // persists until the next motion op consumes it (clear_stop), and
        // nothing can start one while the latch is held, so re-raising
        // every 100 ms only spammed the completion semaphore.
        bool estop_raised = false;

        while (running_.load(std::memory_order_acquire)) {
            // Drain request (normal stop): this task owns the queue's
            // consumer side, so IT clears everything and acknowledges by
            // clearing the flag (see halt_and_drain()).
            if (drain_requested_.load(std::memory_order_acquire)) {
                discard_queued_motion();
                stepper_controller_->emergency_stop();
                // Discarded segments will never execute — replan from where
                // the motors actually are (halt_and_drain holds the
                // abort-planning scope, so a mid-plan producer has unwound).
                resync_planner_from_physical();
                last_error_.store(MotionError::None, std::memory_order_release);
                state_.store(SystemState::Idle, std::memory_order_release);
                reset_inactivity_timer();
                drain_requested_.store(false, std::memory_order_release);
                continue;
            }

            // Check for emergency stop
            if (emergency_stop_.load(std::memory_order_acquire)) {
                if (!estop_raised) {
                    discard_queued_motion();
                    stepper_controller_->emergency_stop();
                    estop_raised = true;
                    ESP_LOGW(TAG, "E-stop latched: queued motion discarded");
                }
                state_.store(SystemState::EStop, std::memory_order_release);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            estop_raised = false;
            // The latch was released (clear_emergency_stop) — leave EStop
            // even if that call raced our store above.
            if (state_.load(std::memory_order_acquire) == SystemState::EStop) {
                state_.store(SystemState::Idle, std::memory_order_release);
            }

            // Check for pause
            if (paused_.load(std::memory_order_acquire)) {
                state_.store(SystemState::Paused, std::memory_order_release);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Error hold: a segment failed (below). Everything queued was
            // discarded; do NOT pop more until a drain (the player's stop()
            // -> halt_and_drain, or clear_emergency_stop) resets the state.
            // The player sees state == Error / last_error via get_status().
            if (state_.load(std::memory_order_acquire) == SystemState::Error) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Transfer ready segments from velocity planner to execution queue
            transfer_ready_segments();

            // Try to get next segment from queue
            MotionSegment segment;
            if (segment_queue_.pop(segment)) {
                // Motion is flowing — reset the producer-stall tracker
                last_planner_pending_ = 0;
                planner_stall_ticks_ = 0;

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
                    if (result.error() == MotionError::EmergencyStop &&
                        (drain_requested_.load(std::memory_order_acquire) ||
                         emergency_stop_.load(std::memory_order_acquire))) {
                        // Interrupted by a stop we were asked for; the drain /
                        // e-stop branch at the top of the loop takes it from
                        // here.
                        continue;
                    }
                    // A real failure (RMT timeout, hardware fault, rejected
                    // segment). Discard everything queued in ONE place —
                    // the old loop popped every remaining segment and
                    // failed each with its own log line — resync the
                    // planner from where the motors stopped, and park in
                    // Error so the player stops the pattern cleanly.
                    ESP_LOGE(TAG, "Segment execution failed (error %d): discarding %zu queued segments",
                        static_cast<int>(result.error()), segment_queue_.size());
                    last_error_.store(result.error(), std::memory_order_release);
                    {
                        AbortPlanningScope abort(abort_planning_depth_);
                        discard_queued_motion();
                        resync_planner_from_physical();
                    }
                    state_.store(SystemState::Error, std::memory_order_release);
                    reset_inactivity_timer();
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
                // Queue empty. The planner may still hold gated segments
                // (see transfer_ready_segments): if the producer is
                // actively adding, its count keeps changing — wait. If it
                // has gone quiet (stream ended, network stall), flush the
                // gate so the held tail executes its planned decel-to-zero
                // instead of parking the ball mid-path indefinitely.
                // Advisory single-word read; exactness not required.
                const size_t pending = velocity_planner_->segment_count();
                if (pending > 0) {
                    if (pending == last_planner_pending_) {
                        if (++planner_stall_ticks_ >= kPlannerStallFlushTicks) {
                            transfer_ready_segments(true);
                            planner_stall_ticks_ = 0;
                        }
                    }
                    else {
                        last_planner_pending_ = pending;
                        planner_stall_ticks_ = 0;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                last_planner_pending_ = 0;
                planner_stall_ticks_ = 0;

                // Queue and planner both empty, check if we should go idle
                if (state_.load(std::memory_order_acquire) == SystemState::Running) {
                    state_.store(SystemState::Idle, std::memory_order_release);
                    reset_inactivity_timer();
                }
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay when idle
            }
        }

        // Exit acknowledgement for stop() (it waits on this before the
        // destructor deletes the mutexes used above).
        stepper_task_exited_.store(true, std::memory_order_release);
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

        // Rho adjustment per theta rotation (due to coupling): unwind
        // exactly what the transformer accumulated over one rotation, from
        // the transformer's OWN calibration snapshot. Reading live config
        // here instead (the previous code) desyncs from the snapshot the
        // moment steps_per_rev/microsteps are PATCHed after homing,
        // injecting a rho counter error on every wrap.
        const int32_t rho_per_theta_rot = transformer_->rho_counteract_per_rotation();

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
        if (!inactivity_timer_) return;
        if (MotionConfig::motor_idle_timeout_s() == 0) {
            // "Never de-energize": keep the motors powered.
            if (xTimerStop(inactivity_timer_, 0) != pdPASS) {
                ESP_LOGW(TAG, "Inactivity timer stop not queued (timer queue full)");
            }
            return;
        }
        armed_epoch_.store(enable_epoch_.load(std::memory_order_acquire),
            std::memory_order_release);
        // A failed reset just means the motors stay powered until the next
        // idle transition re-arms it — log it, don't block the stepper task.
        if (xTimerReset(inactivity_timer_, 0) != pdPASS) {
            ESP_LOGW(TAG, "Inactivity timer reset not queued (timer queue full)");
        }
    }

    void MotionController::apply_motion_tunables() {
        if (!inactivity_timer_) return;
        const uint32_t timeout_s = MotionConfig::motor_idle_timeout_s();
        if (timeout_s == 0) {
            if (xTimerStop(inactivity_timer_, 0) != pdPASS) {
                ESP_LOGW(TAG, "Inactivity timer stop not queued (timer queue full)");
            }
            ESP_LOGI(TAG, "Motor idle timeout disabled");
            return;
        }
        // xTimerChangePeriod also (re)starts the timer; that is the desired
        // behaviour when the motors are idle, and harmless otherwise since
        // enable_motors() stops it and the epoch check guards the callback.
        if (xTimerChangePeriod(inactivity_timer_, pdMS_TO_TICKS(timeout_s * 1000), 0) != pdPASS) {
            ESP_LOGW(TAG, "Inactivity timer period change not queued (timer queue full)");
        }
        ESP_LOGI(TAG, "Motor idle timeout: %lus, accel=%lu steps/s2, junction dv=%lu steps/s, theta cap=%.1f rot/min",
            static_cast<unsigned long>(timeout_s),
            static_cast<unsigned long>(MotionConfig::accel_steps_s2()),
            static_cast<unsigned long>(MotionConfig::junction_dv_steps_s()),
            static_cast<double>(MotionConfig::theta_max_rot_per_min()));
    }

    void MotionController::enable_motors() {
        if (inactivity_timer_ && xTimerStop(inactivity_timer_, 0) != pdPASS) {
            // Harmless: the epoch check below makes a late expiry a no-op.
            ESP_LOGW(TAG, "Inactivity timer stop not queued (timer queue full)");
        }
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
        // The single homing slot. Console, REST (homing_service) and boot
        // homing all land here, so the claim lives here — before the drain,
        // so two callers cannot both drain and then run seeks concurrently.
        bool expected = false;
        if (!homing_claim_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            ESP_LOGW(TAG, "home(): homing already in progress");
            return Result<void>::err(MotionError::InvalidState);
        }
        struct ClaimRelease {
            std::atomic<bool>& claim;
            ~ClaimRelease() { claim.store(false, std::memory_order_release); }
        } release_claim{ homing_claim_ };

        if (homing_controller_->is_homing()) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // A latched e-stop must be cleared explicitly (console clear_estop /
        // API) before the table moves again — including for homing.
        if (emergency_stop_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "home(): refused, emergency stop is latched");
            return Result<void>::err(MotionError::EmergencyStop);
        }

        // The drivers keep the microstep resolution programmed at boot; a
        // calibration taken now would be in steps that no longer match the
        // configured scale. Refuse until the reboot that applies the change.
        const uint32_t live_steps = MechanicalConfig::effective_steps_per_rev();
        if (applied_steps_per_rev_ != 0 && live_steps != applied_steps_per_rev_) {
            ESP_LOGE(TAG, "home(): motion config changed (steps/motor rev %lu -> %lu) - reboot required",
                static_cast<unsigned long>(applied_steps_per_rev_),
                static_cast<unsigned long>(live_steps));
            return Result<void>::err(MotionError::RebootRequired);
        }

        // Homing runs in the CALLER's task while the stepper task keeps
        // consuming the segment queue. Stop playback and drain queued
        // segments first, or a queued move executes concurrently with the
        // homing seeks on the same sequencer. move_to()/move_linear() reject
        // new moves while state is Homing (below), so nothing refills the
        // queue mid-homing. (Cannot be refused here: state is not Homing yet.)
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

            // Reset path planner position (nothing can be planning: moves
            // are refused while Homing, and the drain above unwound any
            // in-flight plan).
            if (xSemaphoreTake(plan_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                path_planner_->set_current_position({ 0.0, 0.0 });
                xSemaphoreGive(plan_mutex_);
            }
            else {
                path_planner_->set_current_position({ 0.0, 0.0 });
            }

            last_error_.store(MotionError::None, std::memory_order_release);
            state_.store(SystemState::Idle, std::memory_order_release);
        }
        else {
            // A failed or aborted home leaves the motors wherever the seek
            // stopped and the transformer on the OLD calibration. Mark not
            // homed (moves are refused with NotHomed instead of executing
            // against a stale frame), re-seat the planner on the physical
            // counters, and go back to Idle so the next home() is accepted —
            // parking in Error left the controller wedged until reboot.
            is_homed_.store(false, std::memory_order_release);
            resync_planner_from_physical();
            state_.store(emergency_stop_.load(std::memory_order_acquire)
                ? SystemState::EStop : SystemState::Idle,
                std::memory_order_release);
            ESP_LOGE(TAG, "Homing failed (error %d) - not homed", static_cast<int>(result.error()));
        }

        return result;
    }

    void MotionController::abort_homing() {
        if (homing_controller_ && homing_controller_->is_homing()) {
            ESP_LOGW(TAG, "Aborting homing");
            homing_controller_->abort();
        }
    }

    void MotionController::_set_homed(int32_t theta_steps_per_rot, int32_t rho_max) {
        homing_controller_->_set_homed(theta_steps_per_rot, rho_max);
        transformer_->set_calibration(theta_steps_per_rot, rho_max);
        path_planner_->set_calibration(theta_steps_per_rot, rho_max);

        theta_stepper_->set_position(0);
        rho_stepper_->set_position(0);

        // Reset fractional accumulators
        transformer_->reset_accumulators();
        if (xSemaphoreTake(plan_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            path_planner_->set_current_position({ 0.0, 0.0 });
            xSemaphoreGive(plan_mutex_);
        }
        else {
            path_planner_->set_current_position({ 0.0, 0.0 });
        }

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

        // Set default feedrate if not specified
        float actual_feedrate = feedrate;
        if (actual_feedrate <= 0) {
            actual_feedrate = static_cast<float>(MotionConfig::rho_max_rpm());
        }

        // Enable motors BEFORE planning: plan_polar_move blocks on planner
        // backpressure for long moves, and execution starts consuming
        // segments while we are still inside it. (SystemState::Running is
        // set by the stepper task when it actually executes — see
        // stepper_task_loop.)
        enable_motors();

        // The whole plan runs under plan_mutex_: the planner position is read
        // and advanced inside it, and a drain/e-stop resync must not
        // overwrite it midway (the resync waits for us to unwind, which the
        // abort-planning depth guarantees). Lock order: plan_mutex_ before
        // planner_mutex_ (taken inside enqueue_segment).
        if (xSemaphoreTake(plan_mutex_, portMAX_DELAY) != pdTRUE) {
            return Result<void>::err(MotionError::InvalidState);
        }
        const PolarPosition current = path_planner_->current_position();
        ESP_LOGD(TAG, "Queueing polar move: (%.4f, %.4f) -> (%.4f, %.4f) @ %.1f RPM",
            current.theta, current.rho, target.theta, target.rho, actual_feedrate);

        // Plan direct polar move (arcs in XY space)
        auto result = path_planner_->plan_polar_move(current, target, actual_feedrate,
            (base_feedrate > 0) ? base_feedrate : actual_feedrate);
        xSemaphoreGive(plan_mutex_);
        return result;
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

        // Set default feedrate if not specified
        float actual_feedrate = feedrate;
        if (actual_feedrate <= 0) {
            actual_feedrate = static_cast<float>(MotionConfig::rho_max_rpm());
        }

        // Enable motors before planning; Running is set by the stepper task
        // on actual execution (see move_to / stepper_task_loop).
        enable_motors();

        // Under plan_mutex_ for the whole plan — see move_to().
        if (xSemaphoreTake(plan_mutex_, portMAX_DELAY) != pdTRUE) {
            return Result<void>::err(MotionError::InvalidState);
        }
        const PolarPosition current = path_planner_->current_position();
        ESP_LOGD(TAG, "Queueing linear move: (%.4f, %.4f) -> (%.4f, %.4f) @ %.1f RPM",
            current.theta, current.rho, target.theta, target.rho, actual_feedrate);

        // Plan Cartesian-interpolated move (straight lines in XY space)
        auto result = path_planner_->plan_linear_move(current, target, actual_feedrate,
            (base_feedrate > 0) ? base_feedrate : actual_feedrate);
        xSemaphoreGive(plan_mutex_);
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

    bool MotionController::halt_and_drain() {
        // No stepper task -> nothing queued, nothing to acknowledge the
        // drain flag (it would just stall the 5s wait below).
        if (!running_.load(std::memory_order_acquire)) {
            return true;
        }

        // Homing owns the sequencer. Raising the sequencer stop here would
        // kill the seek in flight and surface as a homing failure ("edge
        // not found"). The player's stop() lands here while a home runs
        // (it was drained by home() itself before the seeks started), so
        // refuse instead; abort_homing() is the explicit way to stop one.
        if (state_.load(std::memory_order_acquire) == SystemState::Homing) {
            ESP_LOGW(TAG, "halt_and_drain: refused, homing in progress (use abort_homing)");
            return false;
        }

        // Refuse new segments so a task blocked mid-plan unwinds; held
        // through the acknowledgement so the stepper task's resync sees the
        // planner position at rest.
        AbortPlanningScope abort(abort_planning_depth_);

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

        ESP_LOGI(TAG, "Motion halted and queues drained");
        return true;
    }

    void MotionController::emergency_stop() {
        emergency_stop_.store(true, std::memory_order_release);
        // One raise; the stepper task's e-stop branch does not re-raise.
        stepper_controller_->emergency_stop();
        // A homing run in progress is aborted too (its seek returns
        // EmergencyStop and home() reports it).
        abort_homing();
        state_.store(SystemState::EStop, std::memory_order_release);
        ESP_LOGW(TAG, "Emergency stop activated");
    }

    void MotionController::clear_emergency_stop() {
        emergency_stop_.store(false, std::memory_order_release);
        // Queued segments were discarded — replan from the physical position
        // (and drop the accumulators that belonged to them).
        {
            AbortPlanningScope abort(abort_planning_depth_);
            resync_planner_from_physical();
        }
        last_error_.store(MotionError::None, std::memory_order_release);
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
        // Short bounded take: a torn read of the two doubles mid-plan is the
        // only risk, and callers are status/rebase paths, never real time.
        if (plan_mutex_ && xSemaphoreTake(plan_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
            const PolarPosition pos = path_planner_->current_position();
            xSemaphoreGive(plan_mutex_);
            return pos;
        }
        return path_planner_->current_position();
    }

    MotionController::KinematicsDebug MotionController::kinematics_debug() const {
        const int32_t theta_steps = theta_stepper_->position();
        const int32_t rho_steps = rho_stepper_->position();
        return {
            transformer_->steps_per_theta_rotation(),
            transformer_->rho_max_steps(),
            transformer_->rho_steps_per_theta_step(),
            transformer_->rho_norm_unclamped(theta_steps, rho_steps),
            transformer_->theta_accumulator(),
            transformer_->rho_accumulator(),
        };
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
            if (abort_planning_depth_.load(std::memory_order_acquire) > 0 ||
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

    void MotionController::transfer_ready_segments(bool flush) {
        // Transfer planned segments to the execution queue — but NOT
        // eagerly. A popped segment's exit velocity is frozen, and the
        // planner's backward pass can only keep junction speeds up while
        // enough successors remain in its buffer (the tail always plans
        // down to zero). Draining the planner as fast as possible (the
        // old behavior) left most junctions planned with <=2 segments of
        // lookahead — i.e. effectively unplanned, with timing-dependent
        // stops. Hold back PLANNER_COMMIT_LOOKAHEAD segments unless
        // flushing (stream ended/stalled): the held tail ends in a
        // planned decel-to-zero, so a flush is a clean controlled stop.
        if (xSemaphoreTake(planner_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;  // Retry next loop iteration
        }
        while (!segment_queue_.full() &&
               (flush ? !velocity_planner_->empty()
                      : velocity_planner_->segment_count() >
                            MotionConfig::PLANNER_COMMIT_LOOKAHEAD)) {
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
        status.last_error = last_error_.load(std::memory_order_acquire);
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
