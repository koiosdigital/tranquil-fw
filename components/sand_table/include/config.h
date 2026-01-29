#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include <cstdint>
#include <cmath>

namespace sand_table {

    // =============================================================================
    // Pin Configuration (from sdkconfig)
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
    // Mechanical Configuration (from sdkconfig with derived calculations)
    // =============================================================================

    struct MechanicalConfig {
        // Motor specifications
        static constexpr uint32_t STEPS_PER_REV = CONFIG_ROBOT_THETA_STEPS_PER_ROT;
        static constexpr uint32_t MICROSTEPS = 16;  // TMC2209 microstepping
        static constexpr uint32_t EFFECTIVE_STEPS_PER_REV = STEPS_PER_REV * MICROSTEPS;

        // Theta axis: Belt drive with gear ratio (stored as x100 in config)
        static constexpr float THETA_GEAR_RATIO = CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0f;

        // Rho axis: Rack and pinion
        static constexpr int32_t PINION_PITCH_DIAMETER_MM = CONFIG_ROBOT_PINION_DIAMETER_MM;
        static constexpr float PINION_CIRCUMFERENCE_MM = PINION_PITCH_DIAMETER_MM * static_cast<float>(M_PI);

        // Physical limits (integers from config)
        static constexpr int32_t RHO_MIN_MM = CONFIG_ROBOT_RHO_MIN_MM;
        static constexpr int32_t RHO_MAX_MM = CONFIG_ROBOT_RHO_MAX_MM;

        // Coupling compensation direction (determined empirically)
        // If theta CCW causes rho to increase, use -1. If decrease, use +1.
        static constexpr int8_t COUPLING_DIRECTION = -1;

        // ==========================================================================
        // Derived Constants (calculated at compile time from config values)
        // ==========================================================================

        // Theta: steps per degree of stage rotation
        // (effective_steps_per_rev * gear_ratio) / 360 degrees
        static constexpr float THETA_STEPS_PER_DEG =
            (static_cast<float>(EFFECTIVE_STEPS_PER_REV) * THETA_GEAR_RATIO) / 360.0f;

        // Rho: steps per mm of radial movement
        // effective_steps_per_rev / pinion_circumference
        static constexpr float RHO_STEPS_PER_MM =
            static_cast<float>(EFFECTIVE_STEPS_PER_REV) / PINION_CIRCUMFERENCE_MM;

        // Coupling compensation: rho steps per theta motor step
        // = 1 / gear_ratio (since rho and theta have same motor/microstep config)
        static constexpr float RHO_STEPS_PER_THETA_STEP = 1.0f / THETA_GEAR_RATIO;

        // Validation helper
        [[nodiscard]] static constexpr bool is_rho_in_bounds(float rho_mm) noexcept {
            return rho_mm >= RHO_MIN_MM && rho_mm <= RHO_MAX_MM;
        }
    };

    // =============================================================================
    // Motion Configuration (from sdkconfig)
    // =============================================================================

    struct MotionConfig {
        // Velocity limits (integers from config, cast to float when needed)
        static constexpr int32_t MAX_VELOCITY_MM_S = CONFIG_ROBOT_MAX_VELOCITY_MM_S;
        static constexpr int32_t DEFAULT_VELOCITY_MM_S = CONFIG_ROBOT_DEFAULT_VELOCITY_MM_S;
        static constexpr int32_t MIN_VELOCITY_MM_S = 1;

        // Acceleration limits
        static constexpr int32_t MAX_ACCEL_MM_S2 = CONFIG_ROBOT_MAX_ACCEL_MM_S2;
        static constexpr int32_t DEFAULT_ACCEL_MM_S2 = CONFIG_ROBOT_DEFAULT_ACCEL_MM_S2;

        // Path segmentation
        static constexpr float SEGMENT_LENGTH_MM = 1.0f;

        // Velocity lookahead
        static constexpr uint32_t LOOKAHEAD_DEPTH = 32;
        static constexpr float JUNCTION_DEVIATION_MM = 0.05f;

        // Motor current settings (from config, hold = run / 2)
        static constexpr uint16_t THETA_IRUN_MA = CONFIG_ROBOT_THETA_MOTOR_CURRENT;
        static constexpr uint16_t THETA_IHOLD_MA = CONFIG_ROBOT_THETA_MOTOR_CURRENT / 2;
        static constexpr uint16_t RHO_IRUN_MA = CONFIG_ROBOT_RHO_MOTOR_CURRENT;
        static constexpr uint16_t RHO_IHOLD_MA = CONFIG_ROBOT_RHO_MOTOR_CURRENT / 2;

        // StallGuard threshold for rho homing
        static constexpr uint8_t RHO_STALLGUARD_THRESHOLD = CONFIG_ROBOT_RHO_STALLGUARD_THRESHOLD;

        // Homing speeds (integer mm/s)
        static constexpr int32_t HOMING_FAST_SPEED = 20;
        static constexpr int32_t HOMING_SLOW_SPEED = 5;

        // Motor inactivity timeout (ms)
        static constexpr uint32_t MOTOR_INACTIVITY_TIMEOUT_MS = 5000;
    };

    // =============================================================================
    // Timer and Hardware Configuration (from sdkconfig)
    // =============================================================================

    struct HardwareConfig {
        // Timer resolution for step generation
        static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;  // 1 MHz = 1us resolution

        // RMT channel configuration
        static constexpr uint32_t RMT_RESOLUTION_HZ = 10000000;  // 10 MHz for RMT

        // Minimum step pulse width (microseconds)
        static constexpr uint32_t MIN_STEP_PULSE_US = 2;

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
        static constexpr uint32_t STEPPER_TASK_STACK = 4096;
        static constexpr int STEPPER_TASK_PRIORITY = 24;  // configMAX_PRIORITIES - 1
        static constexpr int STEPPER_TASK_CORE = 1;

        // Planner task - medium priority, runs on Core 0
        static constexpr uint32_t PLANNER_TASK_STACK = 4096;
        static constexpr int PLANNER_TASK_PRIORITY = 5;
        static constexpr int PLANNER_TASK_CORE = 0;
    };

} // namespace sand_table
