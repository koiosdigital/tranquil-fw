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
        static constexpr gpio_num_t THETA_ENABLE = static_cast<gpio_num_t>(CONFIG_ROBOT_COMMON_ENABLE_PIN);
        static constexpr gpio_num_t THETA_HALL = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_ENDSTOP_PIN);

        // Rho Axis (Radial Movement)
        static constexpr gpio_num_t RHO_STEP = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_STEP_PIN);
        static constexpr gpio_num_t RHO_DIR = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_DIR_PIN);
        static constexpr gpio_num_t RHO_ENABLE = static_cast<gpio_num_t>(CONFIG_ROBOT_COMMON_ENABLE_PIN);

        // TMC2209 UART (shared bus)
        static constexpr gpio_num_t TMC_TX = static_cast<gpio_num_t>(CONFIG_ROBOT_TMC_UART_TX_PIN);
        static constexpr gpio_num_t TMC_RX = static_cast<gpio_num_t>(CONFIG_ROBOT_TMC_UART_RX_PIN);

        // TMC2209 UART addresses
        static constexpr uint8_t THETA_TMC_ADDR = CONFIG_ROBOT_THETA_TMC_ADDR;
        static constexpr uint8_t RHO_TMC_ADDR = CONFIG_ROBOT_RHO_TMC_ADDR;

        // StallGuard DIAG pins
        static constexpr gpio_num_t RHO_DIAG = static_cast<gpio_num_t>(CONFIG_ROBOT_RHO_STALLGUARD_DIAG_PIN);
        static constexpr gpio_num_t THETA_DIAG = static_cast<gpio_num_t>(CONFIG_ROBOT_THETA_STALLGUARD_DIAG_PIN);
    };

    // =============================================================================
    // Mechanical Configuration (runtime from NVS via ConfigManager)
    // =============================================================================

    struct MechanicalConfig {
        // Compile-time defaults (fallback if ConfigManager not initialized)
        static constexpr uint32_t kDefaultStepsPerRev = 200;
        static constexpr uint32_t kDefaultMicrosteps = 16;
        static constexpr int32_t kDefaultGearRatioX100 = 805;  // 8.05:1
        static constexpr int32_t kDefaultPinionDiaMm = 12;

        // Runtime accessors (delegate to ConfigManager)
        static uint32_t steps_per_rev();
        static uint32_t microsteps();
        static uint32_t effective_steps_per_rev();
        static int32_t theta_gear_ratio_x100();
        static double theta_gear_ratio();
        static int32_t steps_per_theta_rotation();
        static int32_t pinion_diameter_mm();
        static double pinion_circumference_mm();
        static double rho_steps_per_mm();
        static double rho_steps_per_theta_step();

        // Legacy constexpr for compile-time contexts (uses defaults)
        static constexpr uint32_t STEPS_PER_REV = kDefaultStepsPerRev;
        static constexpr uint32_t MICROSTEPS = kDefaultMicrosteps;
        static constexpr uint32_t EFFECTIVE_STEPS_PER_REV = STEPS_PER_REV * MICROSTEPS;
        static constexpr int32_t THETA_GEAR_RATIO_X100 = kDefaultGearRatioX100;
        static constexpr double THETA_GEAR_RATIO = kDefaultGearRatioX100 / 100.0;
        static constexpr int32_t STEPS_PER_THETA_ROTATION =
            (STEPS_PER_REV * MICROSTEPS * THETA_GEAR_RATIO_X100) / 100;
        static constexpr int32_t PINION_PITCH_DIAMETER_MM = kDefaultPinionDiaMm;
        static constexpr double PINION_CIRCUMFERENCE_MM = PINION_PITCH_DIAMETER_MM * M_PI;
        static constexpr double RHO_STEPS_PER_MM =
            static_cast<double>(EFFECTIVE_STEPS_PER_REV) / PINION_CIRCUMFERENCE_MM;
        static constexpr double RHO_STEPS_PER_THETA_STEP = 100.0 / THETA_GEAR_RATIO_X100;
    };

    // =============================================================================
    // Motion Configuration (runtime from NVS via ConfigManager)
    // =============================================================================

    struct MotionConfig {
        // Compile-time defaults (fallback if ConfigManager not initialized)
        static constexpr int32_t kDefaultThetaMaxRpm = 15;
        static constexpr int32_t kDefaultRhoMaxRpm = 15;
        static constexpr uint16_t kDefaultThetaCurrentMa = 400;
        static constexpr uint16_t kDefaultRhoCurrentMa = 400;
        static constexpr uint8_t kDefaultStallguardThreshold = 30;
        static constexpr float kDefaultAccelMmS2 = 100.0f;
        static constexpr float kDefaultMaxAccelMmS2 = 200.0f;

        // Runtime accessors (delegate to ConfigManager)
        static int32_t theta_max_rpm();
        static int32_t rho_max_rpm();
        static uint16_t theta_irun_ma();
        static uint16_t theta_ihold_ma();
        static uint16_t rho_irun_ma();
        static uint16_t rho_ihold_ma();
        static uint8_t stallguard_threshold();
        static float default_accel();
        static float max_accel();

        // Constants that don't change at runtime
        static constexpr uint32_t LOOKAHEAD_DEPTH = 32;
        static constexpr float JUNCTION_DEVIATION_MM = 0.05f;
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

        // Legacy constexpr for compile-time contexts (uses defaults)
        static constexpr int32_t THETA_MAX_SPEED_RPM = kDefaultThetaMaxRpm;
        static constexpr int32_t RHO_MAX_SPEED_RPM = kDefaultRhoMaxRpm;
        static constexpr float DEFAULT_ACCEL_MM_S2 = kDefaultAccelMmS2;
        static constexpr uint16_t THETA_IRUN_MA = kDefaultThetaCurrentMa;
        static constexpr uint16_t THETA_IHOLD_MA = kDefaultThetaCurrentMa / 2;
        static constexpr uint16_t RHO_IRUN_MA = kDefaultRhoCurrentMa;
        static constexpr uint16_t RHO_IHOLD_MA = kDefaultRhoCurrentMa / 2;
        static constexpr uint8_t RHO_STALLGUARD_THRESHOLD = kDefaultStallguardThreshold;
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
        // Stepper task - highest priority, runs on Core 1
        static constexpr uint32_t STEPPER_TASK_STACK = 2048;
        static constexpr int STEPPER_TASK_PRIORITY = 24;  // configMAX_PRIORITIES - 1
        static constexpr int STEPPER_TASK_CORE = 1;
    };

} // namespace sand_table
