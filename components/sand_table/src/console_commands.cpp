#include "console_commands.h"

#ifndef KD_COMMON_CONSOLE_DISABLE

#include "kd_common.h"
#include "motion_controller.h"
#include "config_manager.h"
#include "argtable3/argtable3.h"

#include <cstdio>
#include <cinttypes>

namespace sand_table {

// Static reference to motion controller (set during init)
static MotionController* s_controller = nullptr;

// =============================================================================
// Helper: Get state name string
// =============================================================================

static const char* state_to_string(SystemState state) {
    switch (state) {
        case SystemState::Idle:   return "Idle";
        case SystemState::Homing: return "Homing";
        case SystemState::Running: return "Running";
        case SystemState::Paused: return "Paused";
        case SystemState::Error:  return "Error";
        case SystemState::EStop:  return "EStop";
        default: return "Unknown";
    }
}

// =============================================================================
// Core Commands
// =============================================================================

// --- home ---
static struct {
    struct arg_lit* full;
    struct arg_end* end;
} home_args;

static int cmd_home(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&home_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, home_args.end, argv[0]);
        return 1;
    }

    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    bool force_full = (home_args.full->count > 0);
    printf("Starting %s homing...\n", force_full ? "full" : "quick");

    auto result = s_controller->home(force_full);

    if (result.is_err()) {
        printf("{\"error\":true,\"message\":\"Homing failed\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"message\":\"Homing complete\"}\n");
    return 0;
}

static void register_home() {
    home_args.full = arg_lit0(NULL, "full", "Force full recalibration");
    home_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "home",
        "Execute homing sequence (--full for recalibration)",
        &cmd_home,
        &home_args
    );
}

// --- stop ---
static int cmd_stop(int argc, char** argv) {
    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    s_controller->emergency_stop();
    printf("{\"error\":false,\"message\":\"Emergency stop triggered\"}\n");
    return 0;
}

static void register_stop() {
    kd_console_register_cmd("stop", "Emergency stop all motion", &cmd_stop);
}

// --- pos ---
static int cmd_pos(int argc, char** argv) {
    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    auto pos = s_controller->get_position();
    auto status = s_controller->get_status();
    auto kin = s_controller->kinematics_debug();

    // rho_norm_unclamped outside [0,1] means the counters were commanded
    // past the physical rails; a physical-vs-counted divergence (coupling
    // error, skipped steps) is NOT detectable from counters alone — see
    // the rotate command for the drift test.
    printf("{\n");
    printf("  \"theta_rad\": %.4f,\n", pos.theta);
    printf("  \"rho_norm\": %.4f,\n", pos.rho);
    printf("  \"rho_norm_unclamped\": %.4f,\n", kin.rho_norm_unclamped);
    printf("  \"theta_steps\": %" PRId32 ",\n", status.theta.position_steps);
    printf("  \"rho_steps\": %" PRId32 ",\n", status.rho.position_steps);
    printf("  \"steps_per_theta_rot\": %" PRId32 ",\n", kin.steps_per_theta_rot);
    printf("  \"rho_max_steps\": %" PRId32 ",\n", kin.rho_max_steps);
    printf("  \"rho_steps_per_theta_step\": %.6f,\n", kin.rho_steps_per_theta_step);
    printf("  \"theta_accumulator\": %.4f,\n", kin.theta_accumulator);
    printf("  \"rho_accumulator\": %.4f,\n", kin.rho_accumulator);
    printf("  \"error\": false\n");
    printf("}\n");
    return 0;
}

static void register_pos() {
    kd_console_register_cmd("pos", "Print current position (theta, rho, steps)", &cmd_pos);
}

// --- rotate (coupling drift test) ---
static struct {
    struct arg_dbl* revs;
    struct arg_end* end;
} rotate_args;

static int cmd_rotate(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&rotate_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, rotate_args.end, argv[0]);
        return 1;
    }

    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    double revs = rotate_args.revs->dval[0];
    if (revs < -100.0 || revs > 100.0 || revs == 0.0) {
        printf("{\"error\":true,\"message\":\"revs must be nonzero, within +/-100\"}\n");
        return 1;
    }

    // Pure theta rotation at constant rho: the coupling compensation is the
    // ONLY rho motion commanded, so any physical radial drift after N revs
    // measures the compensation error per revolution directly (divide the
    // observed drift by N). Reported rho will not change - that's the point.
    auto current = s_controller->get_planning_position();
    PolarPosition target{ current.theta + revs * 2.0 * M_PI, current.rho };

    auto result = s_controller->move_to(target, 0.0f);
    if (result.is_err()) {
        printf("{\"error\":true,\"message\":\"Move rejected (homed? not estopped?)\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"revs\":%.2f,\"rho\":%.4f,"
        "\"note\":\"mark ball position; physical radial drift / revs = coupling error per rev\"}\n",
        revs, current.rho);
    return 0;
}

static void register_rotate() {
    rotate_args.revs = arg_dbl1(NULL, NULL, "<revs>", "Full theta rotations (+/-, e.g. 20)");
    rotate_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "rotate",
        "Rotate theta N revolutions at constant rho (coupling drift test)",
        &cmd_rotate,
        &rotate_args
    );
}

// --- status ---
static int cmd_status(int argc, char** argv) {
    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    auto status = s_controller->get_status();

    printf("{\n");
    printf("  \"state\": \"%s\",\n", state_to_string(status.state));
    printf("  \"is_homed\": %s,\n", status.is_homed ? "true" : "false");
    printf("  \"queue_depth\": %zu,\n", status.queue_depth);
    printf("  \"queue_capacity\": %zu,\n", status.queue_capacity);
    printf("  \"theta_enabled\": %s,\n", status.theta.is_enabled ? "true" : "false");
    printf("  \"rho_enabled\": %s,\n", status.rho.is_enabled ? "true" : "false");
    printf("  \"error\": false\n");
    printf("}\n");
    return 0;
}

static void register_status() {
    kd_console_register_cmd("status", "Print system state and queue info", &cmd_status);
}

// =============================================================================
// Configuration Commands
// =============================================================================

// --- set_rho_sgthrs ---
static struct {
    struct arg_int* threshold;
    struct arg_end* end;
} set_sgthrs_args;

static int cmd_set_rho_sgthrs(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_sgthrs_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_sgthrs_args.end, argv[0]);
        return 1;
    }

    int val = set_sgthrs_args.threshold->ival[0];
    if (val < 0 || val > 255) {
        printf("{\"error\":true,\"message\":\"Threshold must be 0-255\"}\n");
        return 1;
    }

    auto& cfg = ConfigManager::instance();
    RuntimeMotionConfig motion = cfg.motion_config();
    motion.stallguard_threshold = static_cast<uint8_t>(val);

    esp_err_t err = cfg.set_motion_config(motion);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"stallguard_threshold\":%d}\n", val);
    return 0;
}

static void register_set_rho_sgthrs() {
    set_sgthrs_args.threshold = arg_int1(NULL, NULL, "<0-255>", "StallGuard threshold");
    set_sgthrs_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "set_rho_sgthrs",
        "Set rho StallGuard threshold (0-255, higher = less sensitive)",
        &cmd_set_rho_sgthrs,
        &set_sgthrs_args
    );
}

// --- set_theta_max_rpm ---
static struct {
    struct arg_int* rpm;
    struct arg_end* end;
} set_theta_rpm_args;

static int cmd_set_theta_max_rpm(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_theta_rpm_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_theta_rpm_args.end, argv[0]);
        return 1;
    }

    int val = set_theta_rpm_args.rpm->ival[0];
    if (val < 1 || val > 100) {
        printf("{\"error\":true,\"message\":\"RPM must be 1-100\"}\n");
        return 1;
    }

    auto& cfg = ConfigManager::instance();
    RuntimeMotionConfig motion = cfg.motion_config();
    motion.theta_max_rpm = val;

    esp_err_t err = cfg.set_motion_config(motion);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"theta_max_rpm\":%d}\n", val);
    return 0;
}

static void register_set_theta_max_rpm() {
    set_theta_rpm_args.rpm = arg_int1(NULL, NULL, "<rpm>", "Max theta speed in RPM");
    set_theta_rpm_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "set_theta_max_rpm",
        "Set theta max speed (RPM)",
        &cmd_set_theta_max_rpm,
        &set_theta_rpm_args
    );
}

// --- set_rho_max_rpm ---
static struct {
    struct arg_int* rpm;
    struct arg_end* end;
} set_rho_rpm_args;

static int cmd_set_rho_max_rpm(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_rho_rpm_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_rho_rpm_args.end, argv[0]);
        return 1;
    }

    int val = set_rho_rpm_args.rpm->ival[0];
    if (val < 1 || val > 100) {
        printf("{\"error\":true,\"message\":\"RPM must be 1-100\"}\n");
        return 1;
    }

    auto& cfg = ConfigManager::instance();
    RuntimeMotionConfig motion = cfg.motion_config();
    motion.rho_max_rpm = val;

    esp_err_t err = cfg.set_motion_config(motion);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"rho_max_rpm\":%d}\n", val);
    return 0;
}

static void register_set_rho_max_rpm() {
    set_rho_rpm_args.rpm = arg_int1(NULL, NULL, "<rpm>", "Max rho speed in RPM");
    set_rho_rpm_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "set_rho_max_rpm",
        "Set rho max speed (RPM)",
        &cmd_set_rho_max_rpm,
        &set_rho_rpm_args
    );
}

// --- set_accel ---
static struct {
    struct arg_dbl* accel;
    struct arg_end* end;
} set_accel_args;

static int cmd_set_accel(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_accel_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_accel_args.end, argv[0]);
        return 1;
    }

    double val = set_accel_args.accel->dval[0];
    if (val < 10.0 || val > 1000.0) {
        printf("{\"error\":true,\"message\":\"Acceleration must be 10-1000 mm/s^2\"}\n");
        return 1;
    }

    auto& cfg = ConfigManager::instance();
    RuntimeMotionConfig motion = cfg.motion_config();
    motion.default_accel = static_cast<float>(val);

    esp_err_t err = cfg.set_motion_config(motion);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"default_accel\":%.1f}\n", val);
    return 0;
}

static void register_set_accel() {
    set_accel_args.accel = arg_dbl1(NULL, NULL, "<mm/s^2>", "Default acceleration");
    set_accel_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "set_accel",
        "Set default acceleration (mm/s^2)",
        &cmd_set_accel,
        &set_accel_args
    );
}

// --- get_config ---
static int cmd_get_config(int argc, char** argv) {
    auto& cfg = ConfigManager::instance();
    const auto& m = cfg.motion_config();

    printf("{\n");
    printf("  \"steps_per_rev\": %" PRIu32 ",\n", m.steps_per_rev);
    printf("  \"microsteps\": %u,\n", m.microsteps);
    printf("  \"pinion_diameter_mm\": %" PRId32 ",\n", m.pinion_diameter_mm);
    printf("  \"theta_max_rpm\": %" PRId32 ",\n", m.theta_max_rpm);
    printf("  \"rho_max_rpm\": %" PRId32 ",\n", m.rho_max_rpm);
    printf("  \"theta_current_ma\": %u,\n", m.theta_current_ma);
    printf("  \"rho_current_ma\": %u,\n", m.rho_current_ma);
    printf("  \"stallguard_threshold\": %u,\n", m.stallguard_threshold);
    printf("  \"default_accel\": %.1f,\n", m.default_accel);
    printf("  \"max_accel\": %.1f,\n", m.max_accel);
    printf("  \"error\": false\n");
    printf("}\n");
    return 0;
}

static void register_get_config() {
    kd_console_register_cmd("get_config", "Print all current configuration values", &cmd_get_config);
}

// =============================================================================
// Additional Commands
// =============================================================================

// --- set_theta_current ---
static struct {
    struct arg_int* current;
    struct arg_end* end;
} set_theta_curr_args;

static int cmd_set_theta_current(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_theta_curr_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_theta_curr_args.end, argv[0]);
        return 1;
    }

    int val = set_theta_curr_args.current->ival[0];
    if (val < 100 || val > 2000) {
        printf("{\"error\":true,\"message\":\"Current must be 100-2000 mA\"}\n");
        return 1;
    }

    auto& cfg = ConfigManager::instance();
    RuntimeMotionConfig motion = cfg.motion_config();
    motion.theta_current_ma = static_cast<uint16_t>(val);

    esp_err_t err = cfg.set_motion_config(motion);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"theta_current_ma\":%d,\"note\":\"Restart required\"}\n", val);
    return 0;
}

static void register_set_theta_current() {
    set_theta_curr_args.current = arg_int1(NULL, NULL, "<mA>", "Motor current in mA");
    set_theta_curr_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "set_theta_current",
        "Set theta motor current (mA, restart required)",
        &cmd_set_theta_current,
        &set_theta_curr_args
    );
}

// --- set_rho_current ---
static struct {
    struct arg_int* current;
    struct arg_end* end;
} set_rho_curr_args;

static int cmd_set_rho_current(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&set_rho_curr_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_rho_curr_args.end, argv[0]);
        return 1;
    }

    int val = set_rho_curr_args.current->ival[0];
    if (val < 100 || val > 2000) {
        printf("{\"error\":true,\"message\":\"Current must be 100-2000 mA\"}\n");
        return 1;
    }

    auto& cfg = ConfigManager::instance();
    RuntimeMotionConfig motion = cfg.motion_config();
    motion.rho_current_ma = static_cast<uint16_t>(val);

    esp_err_t err = cfg.set_motion_config(motion);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"rho_current_ma\":%d,\"note\":\"Restart required\"}\n", val);
    return 0;
}

static void register_set_rho_current() {
    set_rho_curr_args.current = arg_int1(NULL, NULL, "<mA>", "Motor current in mA");
    set_rho_curr_args.end = arg_end(1);

    kd_console_register_cmd_with_args(
        "set_rho_current",
        "Set rho motor current (mA, restart required)",
        &cmd_set_rho_current,
        &set_rho_curr_args
    );
}

// --- calibration ---
static int cmd_calibration(int argc, char** argv) {
    auto& cfg = ConfigManager::instance();
    const auto& cal = cfg.calibration();

    printf("{\n");
    printf("  \"is_valid\": %s,\n", cal.is_valid ? "true" : "false");
    printf("  \"theta_steps_per_rotation\": %" PRId32 ",\n", cal.theta_steps_per_rotation);
    printf("  \"rho_max_steps\": %" PRId32 ",\n", cal.rho_max_steps);
    printf("  \"timestamp\": %" PRIu32 ",\n", cal.timestamp);
    printf("  \"error\": false\n");
    printf("}\n");
    return 0;
}

static void register_calibration() {
    kd_console_register_cmd("calibration", "Print current calibration data", &cmd_calibration);
}

// --- clear_calibration ---
static int cmd_clear_calibration(int argc, char** argv) {
    auto& cfg = ConfigManager::instance();
    esp_err_t err = cfg.clear_calibration();

    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to clear calibration\"}\n");
        return 1;
    }

    printf("{\"error\":false,\"message\":\"Calibration cleared, full home required\"}\n");
    return 0;
}

static void register_clear_calibration() {
    kd_console_register_cmd("clear_calibration", "Clear stored calibration data", &cmd_clear_calibration);
}

// --- motor_enable ---
static int cmd_motor_enable(int argc, char** argv) {
    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    s_controller->enable_motors();
    printf("{\"error\":false,\"message\":\"Motors enabled\"}\n");
    return 0;
}

static void register_motor_enable() {
    kd_console_register_cmd("motor_enable", "Enable motors", &cmd_motor_enable);
}

// --- motor_disable ---
static int cmd_motor_disable(int argc, char** argv) {
    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }

    s_controller->disable_motors();
    printf("{\"error\":false,\"message\":\"Motors disabled\"}\n");
    return 0;
}

static void register_motor_disable() {
    kd_console_register_cmd("motor_disable", "Disable motors", &cmd_motor_disable);
}

// =============================================================================
// Initialization
// =============================================================================

void console_init(MotionController* controller) {
    s_controller = controller;

    // Core commands
    register_home();
    register_stop();
    register_pos();
    register_rotate();
    register_status();

    // Configuration commands
    register_set_rho_sgthrs();
    register_set_theta_max_rpm();
    register_set_rho_max_rpm();
    register_set_accel();
    register_get_config();

    // Additional commands
    register_set_theta_current();
    register_set_rho_current();
    register_calibration();
    register_clear_calibration();
    register_motor_enable();
    register_motor_disable();
}

} // namespace sand_table

#endif // KD_COMMON_CONSOLE_DISABLE
