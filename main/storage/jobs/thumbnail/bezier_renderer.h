#pragma once

#include "thumbnail_common.h"
#include "framebuffer.h"
#include <cstddef>

namespace thumbnail {

/**
 * @brief Cubic B-Spline renderer matching D3.js curveBasis behavior
 *
 * D3's curveBasis uses a sliding window to generate smooth cubic Bezier curves.
 * Bezier control points are calculated as:
 *   CP1 = (2*P0 + P1) / 3
 *   CP2 = (P0 + 2*P1) / 3
 *   End = (P0 + 4*P1 + P2) / 6
 */
class BezierRenderer {
public:
    /**
     * Construct renderer with target framebuffer.
     */
    explicit BezierRenderer(Framebuffer& fb);

    /**
     * Reset state for new curve.
     */
    void beginPath();

    /**
     * Add a cartesian point. Automatically generates curve segments.
     */
    void addPoint(const CartesianPoint& point);

    /**
     * Add a polar point (converts internally).
     */
    void addPolarPoint(const PolarPoint& point);

    /**
     * Finalize the curve. Call after all points added.
     */
    void endPath();

    /**
     * Get number of points processed.
     */
    size_t pointCount() const { return point_count_; }

private:
    /**
     * Draw line using Bresenham's algorithm.
     */
    void drawLine(const CartesianPoint& from, const CartesianPoint& to);

    /**
     * Draw cubic Bezier curve using de Casteljau subdivision.
     */
    void drawBezier(const CartesianPoint& p0,
                    const CartesianPoint& cp1,
                    const CartesianPoint& cp2,
                    const CartesianPoint& p1);

    /**
     * Recursive subdivision using de Casteljau's algorithm.
     */
    void subdivide(const CartesianPoint& p0,
                   const CartesianPoint& cp1,
                   const CartesianPoint& cp2,
                   const CartesianPoint& p1,
                   int depth);

    /**
     * Calculate curve flatness for subdivision decision.
     */
    static float flatness(const CartesianPoint& p0,
                          const CartesianPoint& cp1,
                          const CartesianPoint& cp2,
                          const CartesianPoint& p1);

    /**
     * Generate D3 curveBasis Bezier segment from current window.
     */
    void emitBasisCurve(const CartesianPoint& newPoint);

private:
    Framebuffer& framebuffer_;

    // Sliding window for B-spline (matches D3's _x0, _x1, x pattern)
    CartesianPoint p0_;  // Two points back
    CartesianPoint p1_;  // Previous point

    // Current drawing position
    CartesianPoint current_;

    // State machine state (matches D3's _point)
    uint8_t state_;

    // Total points added
    size_t point_count_;

    // Rendering constants
    static constexpr int MAX_BEZIER_DEPTH = 8;
    static constexpr float FLATNESS_THRESHOLD = 0.5f;
};

} // namespace thumbnail
