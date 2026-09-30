// Exact distances between segments, points, boxes and planes.
//
// A line-by-line transcription of aris/kernel/geometry.py: the same candidates, the same
// operations in the same order, so the two agree to rounding.  The method notes live there.
#pragma once

#include <algorithm>
#include <cmath>

namespace acol {

inline double dot(const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline double clip01(double x) { return std::min(std::max(x, 0.0), 1.0); }

// R^T v, R row-major 3x3
inline void to_local(const double* v, const double* R, double* out) {
    for (int j = 0; j < 3; ++j) out[j] = v[0] * R[j] + v[1] * R[3 + j] + v[2] * R[6 + j];
}

inline double point_segment_d2(const double* p, const double* a, const double* d, double dd) {
    double pa[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
    double t = clip01(dot(pa, d) / (dd > 0.0 ? dd : 1.0));
    double w[3];
    for (int i = 0; i < 3; ++i) w[i] = a[i] + t * d[i] - p[i];
    return dot(w, w);
}

inline double point_segment_distance(const double* p, const double* a, const double* b) {
    double d[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    return std::sqrt(point_segment_d2(p, a, d, dot(d, d)));
}

inline double segment_segment_distance(const double* p0, const double* p1, const double* q0,
                                       const double* q1) {
    double d1[3], d2[3], r[3];
    for (int i = 0; i < 3; ++i) {
        d1[i] = p1[i] - p0[i];
        d2[i] = q1[i] - q0[i];
        r[i] = p0[i] - q0[i];
    }
    const double a = dot(d1, d1), e = dot(d2, d2), b = dot(d1, d2);
    const double c = dot(d1, r), f = dot(d2, r);
    const double den = a * e - b * b;
    double s = den > 0.0 ? clip01((b * f - c * e) / den) : 0.0;
    const double t = clip01((b * s + f) / (e > 0.0 ? e : 1.0));
    s = clip01((b * t - c) / (a > 0.0 ? a : 1.0));
    double w[3];
    for (int i = 0; i < 3; ++i) w[i] = r[i] + s * d1[i] - t * d2[i];
    double best = dot(w, w);
    best = std::min(best, point_segment_d2(p0, q0, d2, e));
    best = std::min(best, point_segment_d2(p1, q0, d2, e));
    best = std::min(best, point_segment_d2(q0, p0, d1, a));
    best = std::min(best, point_segment_d2(q1, p0, d1, a));
    return std::sqrt(best);
}

inline double segment_plane_distance(const double* p0, const double* p1, const double* n, double off) {
    return std::min(dot(n, p0), dot(n, p1)) - off;
}

inline double point_box_distance(const double* p, const double* R, const double* c, const double* h) {
    double v[3] = {p[0] - c[0], p[1] - c[1], p[2] - c[2]}, x[3];
    to_local(v, R, x);
    for (int i = 0; i < 3; ++i) x[i] = std::max(std::fabs(x[i]) - h[i], 0.0);
    return std::sqrt(dot(x, x));
}

inline double box_slope(const double* a, const double* d, const double* h, double t) {
    double g = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double x = a[i] + t * d[i];
        g = g + d[i] * (std::max(x - h[i], 0.0) + std::min(x + h[i], 0.0));
    }
    return g;
}

inline double segment_box_distance(const double* p0, const double* p1, const double* R,
                                   const double* c, const double* h) {
    double v[3] = {p0[0] - c[0], p0[1] - c[1], p0[2] - c[2]};
    double u[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    double a[3], d[3];
    to_local(v, R, a);
    to_local(u, R, d);
    double t[8];
    t[0] = 0.0;
    t[1] = 1.0;
    for (int i = 0; i < 3; ++i) {
        const bool moving = d[i] != 0.0;
        const double inv = 1.0 / (moving ? d[i] : 1.0);
        t[2 + i] = moving ? std::min(std::max((h[i] - a[i]) * inv, 0.0), 1.0) : 0.0;
        t[5 + i] = moving ? std::min(std::max((-h[i] - a[i]) * inv, 0.0), 1.0) : 0.0;
    }
    double t_lo = -1.0, t_hi = 2.0;
    for (int j = 0; j < 8; ++j) {
        if (box_slope(a, d, h, t[j]) <= 0.0) t_lo = std::max(t_lo, t[j]);
        else t_hi = std::min(t_hi, t[j]);
    }
    const double g_lo = box_slope(a, d, h, t_lo), g_hi = box_slope(a, d, h, t_hi);
    const bool has_lo = t_lo >= 0.0, has_hi = t_hi <= 1.0;
    double ts;
    if (has_lo && has_hi) ts = t_lo - g_lo * (t_hi - t_lo) / (g_hi - g_lo);
    else ts = has_lo ? t_lo : 0.0;
    double f = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double x = a[i] + ts * d[i];
        const double e = std::max(x - h[i], 0.0) + std::min(x + h[i], 0.0);
        f += e * e;
    }
    return std::sqrt(f);
}

}  // namespace acol
