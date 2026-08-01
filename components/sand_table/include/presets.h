#pragma once

#include "config_manager.h"
#include <cstddef>

namespace sand_table {

// =============================================================================
// Preset Definition
// =============================================================================

struct Preset {
    const char* id;
    const char* name;
    const char* description;
    RuntimeMotionConfig motion;
    RuntimeLEDConfig led;
};

// =============================================================================
// Built-in Presets (compile-time, read-only)
// =============================================================================

// Tranquil 4 Tabletop - Standard desktop-sized sand table
inline constexpr Preset kPresetTranquil4Tabletop = {
    .id = "tranquil4_tabletop",
    .name = "Tranquil 4 Tabletop",
    .description = "Standard tabletop configuration (12in)",
    .motion = {
        .steps_per_rev = 200,
        .microsteps = 16,
        .theta_max_rpm = 15,
        .rho_max_rpm = 15,
        .theta_current_ma = 400,
        .rho_current_ma = 400,
        .stallguard_threshold = 30,
    },
    .led = {
        .has_leds = true,
        .led_count = 143,
        .is_rgbw = true,
    },
};

// Tranquil 4 Coffee Table - Larger coffee table configuration
inline constexpr Preset kPresetTranquil4Coffee = {
    .id = "tranquil4_coffee",
    .name = "Tranquil 4 Coffee Table",
    .description = "Large coffee table configuration (24in)",
    .motion = {
        .steps_per_rev = 200,
        .microsteps = 16,
        .theta_max_rpm = 12,
        .rho_max_rpm = 12,
        .theta_current_ma = 500,
        .rho_current_ma = 500,
        .stallguard_threshold = 25,
    },
    .led = {
        .has_leds = true,
        .led_count = 200,
        .is_rgbw = true,
    },
};

// Array of all presets for iteration
inline constexpr const Preset* kPresets[] = {
    &kPresetTranquil4Tabletop,
    &kPresetTranquil4Coffee,
};

inline constexpr size_t kPresetCount = sizeof(kPresets) / sizeof(kPresets[0]);

// Find preset by ID (returns nullptr if not found)
inline const Preset* find_preset(const char* id) {
    for (size_t i = 0; i < kPresetCount; ++i) {
        // Simple string comparison
        const char* a = kPresets[i]->id;
        const char* b = id;
        while (*a && *b && *a == *b) { ++a; ++b; }
        if (*a == '\0' && *b == '\0') {
            return kPresets[i];
        }
    }
    return nullptr;
}

} // namespace sand_table
