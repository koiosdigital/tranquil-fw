#include "console_commands.h"

#ifndef KD_COMMON_CONSOLE_DISABLE

#include "kd_common.h"
#include "motion_controller.h"
#include "config_manager.h"
#include "argtable3/argtable3.h"

#include <cstdio>
#include <cinttypes>
#include <cstdlib>
#include <strings.h>

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

    // Push the new threshold to the rho driver now so it takes effect without
    // a reboot or re-home (the driver register is otherwise only written at
    // boot and at the start of each home).
    bool applied = s_controller && s_controller->apply_stallguard_config();

    printf("{\"error\":false,\"stallguard_threshold\":%d,\"applied_live\":%s}\n",
        val, applied ? "true" : "false");
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

// --- LED config name <-> code mapping helpers ---
static const char* led_ic_name(uint8_t v) {
    switch (v) { case 0: return "ws2812"; case 1: return "sk6812"; case 2: return "fw1906"; default: return "unknown"; }
}
static const char* led_format_name(uint8_t v) {
    switch (v) { case 3: return "rgb"; case 4: return "rgbw"; case 5: return "rgbcct"; default: return "unknown"; }
}
static const char* led_order_name(uint8_t v) {
    static const char* n[] = { "rgb", "rbg", "grb", "gbr", "brg", "bgr" };
    return v < 6 ? n[v] : "unknown";
}

// --- get_config ---
static int cmd_get_config(int argc, char** argv) {
    auto& cfg = ConfigManager::instance();
    const auto& m = cfg.motion_config();
    const auto& led = cfg.led_config();

    printf("{\n");
    printf("  \"steps_per_rev\": %" PRIu32 ",\n", m.steps_per_rev);
    printf("  \"microsteps\": %u,\n", m.microsteps);
    printf("  \"rho_max_rpm\": %" PRId32 ",\n", m.rho_max_rpm);
    printf("  \"theta_current_ma\": %u,\n", m.theta_current_ma);
    printf("  \"rho_current_ma\": %u,\n", m.rho_current_ma);
    printf("  \"stallguard_threshold\": %u,\n", m.stallguard_threshold);
    printf("  \"led\": {\"has_leds\": %s, \"led_count\": %u, \"ic_type\": \"%s\", "
           "\"format\": \"%s\", \"color_order\": \"%s\", \"white_swap\": %s},\n",
        led.has_leds ? "true" : "false", (unsigned)led.led_count,
        led_ic_name(led.ic_type), led_format_name(led.format),
        led_order_name(led.color_order), led.white_swap ? "true" : "false");
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

    // Push the new current to the TMC drivers immediately.
    if (s_controller) {
        s_controller->apply_motor_config();
    }

    printf("{\"error\":false,\"theta_current_ma\":%d}\n", val);
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

    // Push the new current to the TMC drivers immediately.
    if (s_controller) {
        s_controller->apply_motor_config();
    }

    printf("{\"error\":false,\"rho_current_ma\":%d}\n", val);
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

// --- sgtest ---
// One-shot rho StallGuard diagnostic: proves the TMC UART link, shows the
// configured SGTHRS and its DIAG trip point, and reads the live SG_RESULT +
// DIAG level. Run this first when sensorless homing "does nothing" - if
// comms_ok is false, StallGuard was never configured (bad UART pins/addr)
// and no home can detect a stall.
static int cmd_sgtest(int argc, char** argv) {
    (void)argc;
    (void)argv;
    if (!s_controller) {
        printf("{\"error\":true,\"message\":\"Motion controller not initialized\"}\n");
        return 1;
    }
    s_controller->log_stallguard_diagnostics();
    return 0;
}

static void register_sgtest() {
    kd_console_register_cmd(
        "sgtest",
        "Rho StallGuard diagnostics (UART link, SGTHRS, live SG_RESULT, DIAG)",
        &cmd_sgtest);
}

// =============================================================================
// Initialization
// =============================================================================

// --- led_config ---
static void print_led_config_json(const RuntimeLEDConfig& led) {
    printf("{\"error\":false,\"led\":{\"has_leds\":%s,\"led_count\":%u,"
           "\"ic_type\":\"%s\",\"format\":\"%s\",\"color_order\":\"%s\","
           "\"white_swap\":%s},\"note\":\"reboot to apply strip changes\"}\n",
        led.has_leds ? "true" : "false", (unsigned)led.led_count,
        led_ic_name(led.ic_type), led_format_name(led.format),
        led_order_name(led.color_order), led.white_swap ? "true" : "false");
}

static bool led_parse_bool(const char* s, bool& out) {
    if (!strcasecmp(s, "on") || !strcasecmp(s, "true") || !strcmp(s, "1")) { out = true; return true; }
    if (!strcasecmp(s, "off") || !strcasecmp(s, "false") || !strcmp(s, "0")) { out = false; return true; }
    return false;
}
static int led_ic_code(const char* s) {
    if (!strcasecmp(s, "ws2812")) return 0;
    if (!strcasecmp(s, "sk6812")) return 1;
    if (!strcasecmp(s, "fw1906")) return 2;
    if (s[0] >= '0' && s[0] <= '2' && s[1] == '\0') return s[0] - '0';
    return -1;
}
static int led_format_code(const char* s) {
    if (!strcasecmp(s, "rgb")) return 3;
    if (!strcasecmp(s, "rgbw")) return 4;
    if (!strcasecmp(s, "rgbcct")) return 5;
    if (s[0] >= '3' && s[0] <= '5' && s[1] == '\0') return s[0] - '0';
    return -1;
}
static int led_order_code(const char* s) {
    static const char* n[] = { "rgb", "rbg", "grb", "gbr", "brg", "bgr" };
    for (int i = 0; i < 6; ++i) if (!strcasecmp(s, n[i])) return i;
    return -1;
}

static int cmd_led_config(int argc, char** argv) {
    auto& cfg = ConfigManager::instance();
    RuntimeLEDConfig led = cfg.led_config();

    if (argc < 2) {  // no args -> print current config
        print_led_config_json(led);
        return 0;
    }
    if (argc != 3) {
        printf("{\"error\":true,\"message\":\"Usage: led_config [<enabled|count|ic|format|order|swap> <value>]\"}\n");
        return 1;
    }

    const char* key = argv[1];
    const char* val = argv[2];

    if (!strcasecmp(key, "enabled")) {
        bool b;
        if (!led_parse_bool(val, b)) { printf("{\"error\":true,\"message\":\"enabled: on|off\"}\n"); return 1; }
        led.has_leds = b;
    } else if (!strcasecmp(key, "count")) {
        int c = atoi(val);
        if (c < 1 || c > 2000) { printf("{\"error\":true,\"message\":\"count must be 1-2000\"}\n"); return 1; }
        led.led_count = static_cast<uint16_t>(c);
    } else if (!strcasecmp(key, "ic")) {
        int v = led_ic_code(val);
        if (v < 0) { printf("{\"error\":true,\"message\":\"ic: ws2812|sk6812|fw1906\"}\n"); return 1; }
        led.ic_type = static_cast<uint8_t>(v);
    } else if (!strcasecmp(key, "format")) {
        int v = led_format_code(val);
        if (v < 0) { printf("{\"error\":true,\"message\":\"format: rgb|rgbw|rgbcct\"}\n"); return 1; }
        led.format = static_cast<uint8_t>(v);
    } else if (!strcasecmp(key, "order")) {
        int v = led_order_code(val);
        if (v < 0) { printf("{\"error\":true,\"message\":\"order: rgb|rbg|grb|gbr|brg|bgr\"}\n"); return 1; }
        led.color_order = static_cast<uint8_t>(v);
    } else if (!strcasecmp(key, "swap")) {
        bool b;
        if (!led_parse_bool(val, b)) { printf("{\"error\":true,\"message\":\"swap: on|off\"}\n"); return 1; }
        led.white_swap = b;
    } else {
        printf("{\"error\":true,\"message\":\"Unknown key '%s'\"}\n", key);
        return 1;
    }

    led.is_rgbw = (led.format == 4);  // keep legacy flag consistent

    esp_err_t err = cfg.set_led_config(led);
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Failed to save config\"}\n");
        return 1;
    }
    print_led_config_json(led);
    return 0;
}

static void register_led_config() {
    kd_console_register_cmd(
        "led_config",
        "Get/set LED strip: led_config [<enabled|count|ic|format|order|swap> <value>]",
        &cmd_led_config);
}

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
    register_set_rho_max_rpm();
    register_get_config();
    register_led_config();

    // Additional commands
    register_set_theta_current();
    register_set_rho_current();
    register_calibration();
    register_clear_calibration();
    register_motor_enable();
    register_motor_disable();
    register_sgtest();
}

} // namespace sand_table

#endif // KD_COMMON_CONSOLE_DISABLE
