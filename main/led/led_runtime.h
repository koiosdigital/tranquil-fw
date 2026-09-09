// LED strip runtime: owns the PixelDriver channel for the configured strip
// and can rebuild it live when the hardware config (count / IC / format /
// colour order / white swap) changes, so a settings save applies without a
// reboot.
#pragma once

#include "config_manager.h"

namespace led_runtime {

// Boot: initialise the driver and, when LEDs are enabled, bring the strip up
// from the NVS config. The driver (effect engine + REST API) is always
// initialised so LEDs can be enabled later at runtime.
void init();

// Apply a new hardware config: tears the current channel down and rebuilds
// it. Safe to call from httpd. No-op when nothing hardware-relevant changed.
void apply(const sand_table::RuntimeLEDConfig& cfg);

}  // namespace led_runtime
