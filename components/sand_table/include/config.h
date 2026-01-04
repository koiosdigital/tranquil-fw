#pragma once

#include "driver/gpio.h"
#include <cstdint>
#include <cmath>

namespace sand_table {

// =============================================================================
// Pin Configuration
// =============================================================================

struct PinConfig {
    // Theta Axis (Stage Rotation)
    static constexpr gpio_num_t THETA_STEP = GPIO_NUM_1;
    static constexpr gpio_num_t THETA_DIR = GPIO_NUM_2;
    static constexpr gpio_num_t THETA_ENABLE = GPIO_NUM_3;
    static constexpr gpio_num_t THETA_HALL = GPIO_NUM_4;  // Hall effect sensor (active low)

    // Rho Axis (Radial Movement)
    static constexpr gpio_num_t RHO_STEP = GPIO_NUM_5;
    static constexpr gpio_num_t RHO_DIR = GPIO_NUM_6;
    static constexpr gpio_num_t RHO_ENABLE = GPIO_NUM_7;

    // TMC2209 UART (shared bus)
    static constexpr gpio_num_t TMC_TX = GPIO_NUM_17;
    static constexpr gpio_num_t TMC_RX = GPIO_NUM_18;

    // TMC2209 UART addresses
    static constexpr uint8_t THETA_TMC_ADDR = 0x00;
    static constexpr uint8_t RHO_TMC_ADDR = 0x01;
};

// =============================================================================
// Mechanical Configuration
// =============================================================================

struct MechanicalConfig {
    // Motor specifications
    static constexpr uint32_t STEPS_PER_REV = 200;      // 1.8 degree motor
    static constexpr uint32_t MICROSTEPS = 16;          // TMC2209 microstepping
    static constexpr uint32_t EFFECTIVE_STEPS_PER_REV = STEPS_PER_REV * MICROSTEPS;  // 3200

    // Theta axis: Belt drive with gear ratio
    static constexpr float THETA_GEAR_RATIO = 4.0f;     // 4 motor revolutions = 1 stage revolution

    // Rho axis: Rack and pinion
    static constexpr float PINION_PITCH_DIAMETER_MM = 12.0f;
    static constexpr float PINION_CIRCUMFERENCE_MM = PINION_PITCH_DIAMETER_MM * M_PI;  // ~37.7mm

    // Physical limits
    static constexpr float RHO_MIN_MM = 5.0f;           // Center exclusion zone
    static constexpr float RHO_MAX_MM = 150.0f;         // Edge of sand

    // Coupling compensation direction (determined empirically)
    // If theta CCW causes rho to increase, use -1. If decrease, use +1.
    static constexpr int8_t COUPLING_DIRECTION = -1;

    // ==========================================================================
    // Derived Constants (calculated at compile time)
    // ==========================================================================

    // Theta: steps per degree of stage rotation
    // (effective_steps_per_rev * gear_ratio) / 360 degrees
    static constexpr float THETA_STEPS_PER_DEG =
        (static_cast<float>(EFFECTIVE_STEPS_PER_REV) * THETA_GEAR_RATIO) / 360.0f;
    // = (3200 * 4) / 360 = 35.556 steps/degree

    // Rho: steps per mm of radial movement
    // effective_steps_per_rev / pinion_circumference
    static constexpr float RHO_STEPS_PER_MM =
        static_cast<float>(EFFECTIVE_STEPS_PER_REV) / PINION_CIRCUMFERENCE_MM;
    // = 3200 / 37.7 = 84.88 steps/mm

    // Coupling compensation: rho steps per theta motor step
    // When theta rotates, it causes unintended rho movement
    // For each theta motor step, the stage rotates by:
    //   1 / (steps_per_rev * microsteps * gear_ratio) revolutions
    // This causes the rack to move by:
    //   (1 / (steps_per_rev * microsteps * gear_ratio)) * pinion_circumference mm
    // Which is:
    //   pinion_circumference / (steps_per_rev * microsteps * gear_ratio) mm
    // In steps, this is:
    //   (pinion_circumference * rho_steps_per_mm) / (steps_per_rev * microsteps * gear_ratio)
    // = 1 / gear_ratio (since rho and theta have same motor/microstep config)
    static constexpr float RHO_STEPS_PER_THETA_STEP = 1.0f / THETA_GEAR_RATIO;
    // = 0.25 steps of compensation per theta step

    // Validation helper
    [[nodiscard]] static constexpr bool is_rho_in_bounds(float rho_mm) noexcept {
        return rho_mm >= RHO_MIN_MM && rho_mm <= RHO_MAX_MM;
    }
};

// =============================================================================
// Motion Configuration
// =============================================================================

struct MotionConfig {
    // Velocity limits
    static constexpr float MAX_VELOCITY_MM_S = 50.0f;
    static constexpr float DEFAULT_VELOCITY_MM_S = 30.0f;
    static constexpr float MIN_VELOCITY_MM_S = 1.0f;

    // Acceleration limits
    static constexpr float MAX_ACCEL_MM_S2 = 200.0f;
    static constexpr float DEFAULT_ACCEL_MM_S2 = 100.0f;

    // Path segmentation
    static constexpr float SEGMENT_LENGTH_MM = 1.0f;

    // Velocity lookahead
    static constexpr uint32_t LOOKAHEAD_DEPTH = 32;
    static constexpr float JUNCTION_DEVIATION_MM = 0.05f;

    // Motor current settings (mA)
    static constexpr uint16_t THETA_IRUN_MA = 800;
    static constexpr uint16_t THETA_IHOLD_MA = 400;
    static constexpr uint16_t RHO_IRUN_MA = 600;
    static constexpr uint16_t RHO_IHOLD_MA = 300;

    // StallGuard threshold for rho homing
    static constexpr uint8_t RHO_STALLGUARD_THRESHOLD = 100;

    // Homing speeds (mm/s for rho, deg/s for theta)
    static constexpr float HOMING_FAST_SPEED = 20.0f;
    static constexpr float HOMING_SLOW_SPEED = 5.0f;

    // Motor inactivity timeout (ms)
    static constexpr uint32_t MOTOR_INACTIVITY_TIMEOUT_MS = 5000;
};

// =============================================================================
// Timer and Hardware Configuration
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
    static constexpr uint32_t TMC_UART_BAUD = 115200;
    static constexpr int TMC_UART_PORT = 1;
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
