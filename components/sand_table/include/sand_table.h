#pragma once

/// Sand Table Motion Controller
/// A complete motion control system for Sisyphus-style sand tables
/// using polar coordinates (theta/rho) with velocity lookahead planning.

// Core types and error handling
#include "types.h"

// Configuration constants
#include "config.h"

// Top-level controller (public API)
#include "motion_controller.h"

namespace sand_table {

    /// Library version
    constexpr const char* VERSION = "1.0.0";

} // namespace sand_table
