#include "bezier_renderer.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace thumbnail {

BezierRenderer::BezierRenderer(Framebuffer& fb)
    : framebuffer_(fb)
    , p0_()
    , p1_()
    , current_()
    , state_(0)
    , point_count_(0) {
}

void BezierRenderer::beginPath() {
    state_ = 0;
    point_count_ = 0;
    p0_ = CartesianPoint();
    p1_ = CartesianPoint();
    current_ = CartesianPoint();
}

void BezierRenderer::addPolarPoint(const PolarPoint& point) {
    addPoint(toCartesian(point));
}

void BezierRenderer::addPoint(const CartesianPoint& point) {
    // D3 curveBasis state machine
    switch (state_) {
        case 0:
            // First point: moveTo
            state_ = 1;
            current_ = point;
            break;

        case 1:
            // Second point: just store
            state_ = 2;
            break;

        case 2:
            // Third point: lineTo weighted start, begin curves
            state_ = 3;
            {
                // Start point formula from D3
                CartesianPoint startPt{
                    (5.0f * p0_.x + p1_.x) / 6.0f,
                    (5.0f * p0_.y + p1_.y) / 6.0f
                };
                drawLine(current_, startPt);
                current_ = startPt;
            }
            // Fall through to emit curve
            [[fallthrough]];

        default:
            // Generate Bezier segment
            emitBasisCurve(point);
            break;
    }

    // Shift window
    p0_ = p1_;
    p1_ = point;
    point_count_++;
}

void BezierRenderer::endPath() {
    if (state_ == 3) {
        // End point formula from D3
        CartesianPoint endPt1{
            (p0_.x + 2.0f * p1_.x) / 3.0f,
            (p0_.y + 2.0f * p1_.y) / 3.0f
        };
        CartesianPoint endPt2 = p1_;

        // Draw final segments
        drawLine(current_, endPt1);
        drawLine(endPt1, endPt2);
    }
}

void BezierRenderer::emitBasisCurve(const CartesianPoint& newPoint) {
    // Calculate Bezier control points using D3's formula
    CartesianPoint cp1{
        (2.0f * p0_.x + p1_.x) / 3.0f,
        (2.0f * p0_.y + p1_.y) / 3.0f
    };

    CartesianPoint cp2{
        (p0_.x + 2.0f * p1_.x) / 3.0f,
        (p0_.y + 2.0f * p1_.y) / 3.0f
    };

    CartesianPoint endPoint{
        (p0_.x + 4.0f * p1_.x + newPoint.x) / 6.0f,
        (p0_.y + 4.0f * p1_.y + newPoint.y) / 6.0f
    };

    // Draw the Bezier curve
    drawBezier(current_, cp1, cp2, endPoint);
    current_ = endPoint;
}

void BezierRenderer::drawBezier(const CartesianPoint& p0,
                                const CartesianPoint& cp1,
                                const CartesianPoint& cp2,
                                const CartesianPoint& p1) {
    subdivide(p0, cp1, cp2, p1, 0);
}

void BezierRenderer::subdivide(const CartesianPoint& p0,
                               const CartesianPoint& cp1,
                               const CartesianPoint& cp2,
                               const CartesianPoint& p1,
                               int depth) {
    // Check if curve is flat enough to draw as line
    if (depth >= MAX_BEZIER_DEPTH || flatness(p0, cp1, cp2, p1) < FLATNESS_THRESHOLD) {
        drawLine(p0, p1);
        return;
    }

    // De Casteljau subdivision at t=0.5
    CartesianPoint l1{(p0.x + cp1.x) * 0.5f, (p0.y + cp1.y) * 0.5f};
    CartesianPoint h{(cp1.x + cp2.x) * 0.5f, (cp1.y + cp2.y) * 0.5f};
    CartesianPoint r2{(cp2.x + p1.x) * 0.5f, (cp2.y + p1.y) * 0.5f};

    CartesianPoint l2{(l1.x + h.x) * 0.5f, (l1.y + h.y) * 0.5f};
    CartesianPoint r1{(h.x + r2.x) * 0.5f, (h.y + r2.y) * 0.5f};

    CartesianPoint mid{(l2.x + r1.x) * 0.5f, (l2.y + r1.y) * 0.5f};

    // Recurse on both halves
    subdivide(p0, l1, l2, mid, depth + 1);
    subdivide(mid, r1, r2, p1, depth + 1);
}

float BezierRenderer::flatness(const CartesianPoint& p0,
                               const CartesianPoint& cp1,
                               const CartesianPoint& cp2,
                               const CartesianPoint& p1) {
    // Maximum distance of control points from line p0->p1
    float dx = p1.x - p0.x;
    float dy = p1.y - p0.y;
    float d = std::sqrt(dx * dx + dy * dy);

    if (d < 0.0001f) {
        return 0.0f;  // Degenerate case
    }

    // Distance from cp1 and cp2 to line
    float d1 = std::abs((cp1.y - p0.y) * dx - (cp1.x - p0.x) * dy) / d;
    float d2 = std::abs((cp2.y - p0.y) * dx - (cp2.x - p0.x) * dy) / d;

    return std::max(d1, d2);
}

void BezierRenderer::drawLine(const CartesianPoint& from, const CartesianPoint& to) {
    // Bresenham's line algorithm
    int x0 = static_cast<int>(from.x + 0.5f);
    int y0 = static_cast<int>(from.y + 0.5f);
    int x1 = static_cast<int>(to.x + 0.5f);
    int y1 = static_cast<int>(to.y + 0.5f);

    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        framebuffer_.setPixel(x0, y0);

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

} // namespace thumbnail
