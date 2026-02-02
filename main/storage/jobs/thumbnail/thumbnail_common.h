#pragma once

#include <cstdint>
#include <cmath>

namespace thumbnail {

// Thumbnail canvas size
static constexpr uint16_t CANVAS_SIZE = 600;

// Line width for rendering
static constexpr float LINE_WIDTH = 0.75f;

// Cartesian point
struct CartesianPoint {
    float x = 0.0f;
    float y = 0.0f;
};

// Polar point (matches pattern format)
struct PolarPoint {
    float theta = 0.0f;  // Angle in radians
    float rho = 0.0f;    // Radius 0.0-1.0
};

// Error codes
enum class ThumbnailError {
    OK = 0,
    MEMORY_ALLOCATION_FAILED,
    FILE_OPEN_FAILED,
    FILE_WRITE_FAILED,
    PNG_ENCODE_FAILED,
    INVALID_INPUT,
};

// Polar to cartesian conversion
// Maps: rho 0=center, 1=edge; theta 0=right, positive=counterclockwise
// Note: True center for pixel array [0, CANVAS_SIZE-1] is (CANVAS_SIZE-1)/2 = 299.5
inline CartesianPoint toCartesian(const PolarPoint& polar) {
    constexpr float center = (CANVAS_SIZE - 1) / 2.0f;  // 299.5 for 600x600
    float r = polar.rho * center;  // Scale factor = distance from center to edge
    return CartesianPoint{
        center + r * std::cos(polar.theta),
        center + r * std::sin(polar.theta)
    };
}

} // namespace thumbnail
