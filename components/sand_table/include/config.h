#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include <cstdint>
#include <cmath>

namespace sand_table {

    // Forward declaration - include config_manager.h for full definition
    class ConfigManager;

    // =============================================================================
    // Pin Configuration (from sdkconfig - hardware, stays compile-time)
    // =============================================================================

    struct PinConfig {
        // Theta Axis (Stage Rotation)
        static constexpr gpio_num_t THETA_STEP = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_STEP_PIN);
        static constexpr gpio_num_t THETA_DIR = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_DIR_PIN);
        static constexpr gpio_num_t THETA_ENABLE = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_ENABLE_PIN);
        static constexpr gpio_num_t THETA_HALL = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_ENDSTOP_PIN);

        // Rho Axis (Radial Movement)
        static constexpr gpio_num_t RHO_STEP = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_STEP_PIN);
        static constexpr gpio_num_t RHO_DIR = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_DIR_PIN);
        static constexpr gpio_num_t RHO_ENABLE = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_ENABLE_PIN);

        // TMC2209 UART (shared bus)
        static constexpr gpio_num_t TMC_TX = static_cast<gpio_num_t>(CONFIG_ROBOT_TMC_UART_TX_PIN);
        static constexpr gpio_num_t TMC_RX = static_cast<gpio_num_t>(CONFIG_ROBOT_TMC_UART_RX_PIN);

        // TMC2209 UART addresses
        static constexpr uint8_t THETA_TMC_ADDR = CONFIG_ROBOT_THETA_TMC_ADDR;
        static constexpr uint8_t RHO_TMC_ADDR = CONFIG_ROBOT_RHO_TMC_ADDR;

        // StallGuard DIAG pins
        static constexpr gpio_num_t RHO_DIAG = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_STALLGUARD_DIAG_PIN);
        static constexpr gpio_num_t THETA_DIAG = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_STALLGUARD_DIAG_PIN);

        // Per-axis direction inversion (Kconfig bools; the macro is absent
        // when unset). ALL motion code works in LOGICAL directions (positive
        // rho = outward toward the edge, positive theta = forward);
        // StepperDriver::dir_pin_level() flips the physical DIR level per
        // these flags. Nothing outside StepperDriver may touch the DIR pin.
#ifdef CONFIG_ROBOT_THETA_INVERT_DIRECTION
        static constexpr bool THETA_INVERT_DIR = true;
#else
        static constexpr bool THETA_INVERT_DIR = false;
#endif
#ifdef CONFIG_ROBOT_RHO_INVERT_DIRECTION
        static constexpr bool RHO_INVERT_DIR = true;
#else
        static constexpr bool RHO_INVERT_DIR = false;
#endif
    };

    // =============================================================================
    // Mechanical Configuration (runtime from NVS via ConfigManager)
    // =============================================================================

    struct MechanicalConfig {
        // Compile-time defaults (fallback if ConfigManager not initialized)
        static constexpr uint32_t kDefaultStepsPerRev = 200;
        static constexpr uint32_t kDefaultMicrosteps = 16;

        // Runtime accessors (delegate to ConfigManager)
        static uint32_t steps_per_rev();
        static uint32_t microsteps();
        // Steps per motor revolution (steps_per_rev * microsteps). The rho
        // coupling compensation is the only consumer: rho drive dragged per
        // theta drum rev = one rho motor rev = this many steps. The rho radial
        // SCALE comes entirely from the homing-observed rho_max_steps, so no
        // pinion/lead geometry is needed anywhere.
        static uint32_t effective_steps_per_rev();

        // Legacy constexpr for compile-time contexts (uses defaults)
        static constexpr uint32_t STEPS_PER_REV = kDefaultStepsPerRev;
        static constexpr uint32_t MICROSTEPS = kDefaultMicrosteps;
        static constexpr uint32_t EFFECTIVE_STEPS_PER_REV = STEPS_PER_REV * MICROSTEPS;

        // Nominal theta motor steps per drum rotation (8.05:1 tabletop drive
        // at 200 * 16 steps/rev). PRE-CALIBRATION ESTIMATE ONLY — homing
        // observes the real value hall-edge-to-hall-edge, and all kinematics
        // and coupling compensation run off that observation. This constant
        // just bounds the very first homing run and the path planner's
        // uncalibrated fallback.
        static constexpr int32_t NOMINAL_STEPS_PER_THETA_ROTATION = 25760;

        // Sign of the theta->rho coupling compensation in LOGICAL step space.
        // The mechanism couples PHYSICAL rotations (one drum revolution drags
        // the rho drive by one rho motor revolution); the +theta/+rho pairing
        // hardcoded throughout this codebase is the baseline with NEITHER
        // axis inverted. Honoring ROBOT_*_INVERT_DIRECTION flips that axis's
        // logical direction relative to physical, so the logical pairing
        // flips with the XOR of the two flags. Every coupling consumer (the
        // coordinate transformer AND the homing companion moves) must use
        // the SIGNED ratio - a consumer using the unsigned ratio on an
        // inverted axis DOUBLES the drag instead of canceling it.
        static constexpr double COUPLING_SIGN =
            (PinConfig::THETA_INVERT_DIR != PinConfig::RHO_INVERT_DIR) ? -1.0 : 1.0;
    };

    // =============================================================================
    // Motion Configuration (runtime from NVS via ConfigManager)
    // =============================================================================

    struct MotionConfig {
        // Runtime accessors (delegate to ConfigManager). These are the live
        // NVS-backed values and are applied to the hardware:
        //   rho_max_rpm        - default path feedrate (move_to / player)
        //   theta_irun_ma      - theta motor run current (TMC IRUN/IHOLD)
        //   rho_irun_ma        - rho motor run current
        //   stallguard_threshold - rho StallGuard sensitivity (homing)
        // The theta spin rate is bounded by the compile-time
        // THETA_MAX_ROT_PER_MIN safety clamp below, not a runtime knob.
        static int32_t rho_max_rpm();
        static uint16_t theta_irun_ma();
        static uint16_t rho_irun_ma();
        static uint8_t stallguard_threshold();

        // Constants that don't change at runtime
        static constexpr uint32_t LOOKAHEAD_DEPTH = 32;
        // De-energize the motors after this long with no motion (holding
        // current just heats the drivers; the arm can't backdrive). The next
        // move re-enables them (with the TMC warmup delay).
        static constexpr uint32_t MOTOR_INACTIVITY_TIMEOUT_MS = 30000;

        // Hard cap on major-axis steps per motion segment. Sized so the
        // interval table (MAX_SEGMENT_STEPS * sizeof(uint16_t)) fits in
        // internal RAM — the RMT TX-done ISR reads it, and ISR-time access
        // to SPIRAM crashes if flash cache is disabled (e.g. NVS commit).
        // The path planner splits longer moves; the stepper controller
        // REJECTS (never silently truncates) segments that exceed this.
        static constexpr uint32_t MAX_SEGMENT_STEPS = 8192;

        // Per-motor step acceleration used for the per-segment trapezoid
        // profile (steps/s^2). This is the knob that actually shapes ramps.
        static constexpr float STEP_ACCEL_STEPS_S2 = 10000.0f;

        // Hard ceiling on the theta axis rotation rate (rotations/min),
        // applied AFTER feedrate (including transit boosts) as a safety
        // clamp. Feedrate is a PATH speed: near the center a tiny path
        // distance maps to a large theta swing (direction changes pass
        // through the middle), so path-speed planning alone can whip the
        // arm around. Enforced in calculate_velocity_profile and, for live
        // feedrate boosts, via the sequencer's speed-scale ceiling.
        static constexpr float THETA_MAX_ROT_PER_MIN = 12.0f;
    };

    // =============================================================================
    // Timer and Hardware Configuration (from sdkconfig)
    // =============================================================================

    struct HardwareConfig {
        // Timer resolution for step generation
        static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;  // 1 MHz = 1us resolution

        // RMT channel configuration
        static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000;  // 1 MHz for RMT (allows min ~31 steps/s)

        // Minimum step pulse width (microseconds)
        // TMC2209 requires 100ns minimum, but use 5µs for reliable edge detection
        static constexpr uint32_t MIN_STEP_PULSE_US = 5;

        // Maximum step rate (steps per second)
        static constexpr uint32_t MAX_STEP_RATE_HZ = 50000;

        // UART configuration for TMC2209
        static constexpr uint32_t TMC_UART_BAUD = CONFIG_ROBOT_TMC_BAUD_RATE;
        static constexpr int TMC_UART_PORT = CONFIG_ROBOT_TMC_UART_PORT;
    };

    // =============================================================================
    // FreeRTOS Task Configuration
    // =============================================================================

    struct TaskConfig {
        // Stepper task - highest priority, runs on Core 1.
        // 4096 (not 2048): this task runs the full motion pipeline
        // (float velocity profiling, interval-table prep, RMT transmit) plus
        // ESP_LOGx with float args, at priority 24 where an overflow is a hard
        // crash rather than a graceful failure.
        static constexpr uint32_t STEPPER_TASK_STACK = 4096;
        static constexpr int STEPPER_TASK_PRIORITY = 24;  // configMAX_PRIORITIES - 1
        static constexpr int STEPPER_TASK_CORE = 1;
    };

} // namespace sand_table
