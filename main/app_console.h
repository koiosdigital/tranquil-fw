#pragma once

/**
 * @file app_console.h
 * @brief Tranquil app-level console commands (registered on the kd_common REPL)
 */

// Register all app-level console commands. Call once after kd_common_init().
void app_console_register_commands();
