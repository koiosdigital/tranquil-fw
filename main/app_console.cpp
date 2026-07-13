/**
 * @file app_console.cpp
 * @brief Tranquil app-level console commands
 */

#include "app_console.h"

#include "kd_console.h"
#include "kdc_heap_tracing.h"
#include "sd.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

#include <cstdio>
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
// format_sd — wipe the SD card and reboot
// ─────────────────────────────────────────────────────────────────────────────

static int cmd_format_sd(int argc, char** argv) {
    printf("Formatting SD card — device reboots when done...\n");

    // Quiesce SD users so nothing writes while the filesystem is recreated
    SandTablePlayer::stop();
    ManifestDatabase::instance().shutdown();

    // Stop the task watchdog: formatting a large card blocks this task far
    // longer than the TWDT timeout. deinit unsubscribes the idle tasks itself
    // (do NOT esp_task_wdt_delete them first — its internal ESP_ERROR_CHECK
    // aborts on the missing entries).
    esp_err_t wdt_err = esp_task_wdt_deinit();
    if (wdt_err != ESP_OK) {
        printf("warning: task watchdog deinit failed (%s), format may trip it\n",
            esp_err_to_name(wdt_err));
    }

    esp_err_t err = format_sd();
    if (err != ESP_OK) {
        printf("{\"error\":true,\"message\":\"Format failed: %s\"}\n", esp_err_to_name(err));
    }
    else {
        printf("{\"error\":false,\"message\":\"Format complete\"}\n");
    }

    printf("Rebooting...\n");
    fflush(stdout);
    esp_restart();
    return 0;  // not reached
}

// ─────────────────────────────────────────────────────────────────────────────
// Heap corruption hunting (kdc_heap_tracing controls)
// ─────────────────────────────────────────────────────────────────────────────

static bool parse_on_off(int argc, char** argv, bool* out) {
    if (argc != 2) return false;
    if (strcmp(argv[1], "on") == 0) { *out = true; return true; }
    if (strcmp(argv[1], "off") == 0) { *out = false; return true; }
    return false;
}

// Integrity-check the whole heap on EVERY alloc/free. Severe slowdown, but a
// poisoning violation then asserts on the first alloc/free after the rogue
// write — the panic backtrace lands next to the writer instead of minutes
// later in an innocent task.
static int cmd_heap_check(int argc, char** argv) {
    bool on;
    if (!parse_on_off(argc, argv, &on)) {
        printf("usage: heap_check <on|off> (currently %s)\n",
            kdc_heap_get_check_on_alloc() ? "on" : "off");
        return 1;
    }
    kdc_heap_set_check_on_alloc(on);
    printf("heap integrity check on alloc/free: %s\n", on ? "on" : "off");
    return 0;
}

static int cmd_heap_log(int argc, char** argv) {
    bool on;
    if (!parse_on_off(argc, argv, &on)) {
        printf("usage: heap_log <on|off> (currently %s)\n",
            kdc_heap_get_log_allocs() ? "on" : "off");
        return 1;
    }
    kdc_heap_set_log_allocs(on);
    printf("heap alloc/free logging: %s\n", on ? "on" : "off");
    return 0;
}

static int cmd_heap_status(int argc, char** argv) {
    kdc_heap_log_status("console");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Registration
// ─────────────────────────────────────────────────────────────────────────────

void app_console_register_commands() {
    kd_console_register_cmd(
        "format_sd",
        "Format the SD card (erases all patterns/playlists) and reboot",
        &cmd_format_sd);
    kd_console_register_cmd(
        "heap_check",
        "Toggle heap integrity check on every alloc/free (heap_check on|off)",
        &cmd_heap_check);
    kd_console_register_cmd(
        "heap_log",
        "Toggle logging of every heap alloc/free (heap_log on|off)",
        &cmd_heap_log);
    kd_console_register_cmd(
        "heap_status",
        "Log heap status per capability and run an integrity check",
        &cmd_heap_status);
}
