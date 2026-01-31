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

        // Theta axis: Belt drive with gear ratio
        // Store as integer x100 to avoid float precision issues in step calculations
        static constexpr int32_t THETA_GEAR_RATIO_X100 = CONFIG_ROBOT_THETA_GEAR_RATIO;
        // For calculations that need the actual ratio as double (coupling compensation)
        static constexpr double THETA_GEAR_RATIO = CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0;

        // Steps per full theta (drive gear) rotation - INTEGER ONLY
        // = motor_steps * microsteps * gear_ratio
        // Using x100 ratio: (200 * 16 * 800) / 100 = 25600
        static constexpr int32_t STEPS_PER_THETA_ROTATION =
            (STEPS_PER_REV * MICROSTEPS * THETA_GEAR_RATIO_X100) / 100;

        // Rho axis: Rack and pinion (only used during homing calibration)
        static constexpr int32_t PINION_PITCH_DIAMETER_MM = CONFIG_ROBOT_PINION_DIAMETER_MM;
        static constexpr double PINION_CIRCUMFERENCE_MM = PINION_PITCH_DIAMETER_MM * M_PI;

        // Rho steps per mm (only used during homing to convert physical movement)
        static constexpr double RHO_STEPS_PER_MM =
            static_cast<double>(EFFECTIVE_STEPS_PER_REV) / PINION_CIRCUMFERENCE_MM;

        // Coupling compensation: rho steps per theta motor step
        // = 1 / gear_ratio (since rho and theta have same motor/microstep config)
        // When theta rotates, rho moves due to rack-and-pinion coupling
        static constexpr double RHO_STEPS_PER_THETA_STEP = 100.0 / THETA_GEAR_RATIO_X100;
    };

    // =============================================================================
    // Motion Configuration (from sdkconfig)
    // =============================================================================

    struct MotionConfig {
        // Speed limits (RPM - matches main branch)
        static constexpr int32_t THETA_MAX_SPEED_RPM = CONFIG_ROBOT_THETA_MAX_SPEED;
        static constexpr int32_t RHO_MAX_SPEED_RPM = CONFIG_ROBOT_RHO_MAX_SPEED;

        // Velocity planning
        static constexpr uint32_t LOOKAHEAD_DEPTH = 32;
        static constexpr float DEFAULT_ACCEL_MM_S2 = 100.0f;    // Default acceleration (mm/s^2)
        static constexpr float JUNCTION_DEVIATION_MM = 0.05f;  // Cornering deviation (mm)

        // Motor current settings (from config, hold = run / 2)
        static constexpr uint16_t THETA_IRUN_MA = CONFIG_ROBOT_THETA_MOTOR_CURRENT;
        static constexpr uint16_t THETA_IHOLD_MA = CONFIG_ROBOT_THETA_MOTOR_CURRENT / 2;
        static constexpr uint16_t RHO_IRUN_MA = CONFIG_ROBOT_RHO_MOTOR_CURRENT;
        static constexpr uint16_t RHO_IHOLD_MA = CONFIG_ROBOT_RHO_MOTOR_CURRENT / 2;

        // StallGuard threshold for rho homing
        static constexpr uint8_t RHO_STALLGUARD_THRESHOLD = CONFIG_ROBOT_RHO_STALLGUARD_THRESHOLD;

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
        static constexpr uint32_t STEPPER_TASK_STACK = 4096;
        static constexpr int STEPPER_TASK_PRIORITY = 24;  // configMAX_PRIORITIES - 1
        static constexpr int STEPPER_TASK_CORE = 1;

        // Planner task - medium priority, runs on Core 0
        static constexpr uint32_t PLANNER_TASK_STACK = 4096;
        static constexpr int PLANNER_TASK_PRIORITY = 5;
        static constexpr int PLANNER_TASK_CORE = 0;
    };

} // namespace sand_table
