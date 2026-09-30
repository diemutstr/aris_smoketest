// Distance fields (types.Field): the same arithmetic as geometry.field_lookup and
// geometry.segment_field_distance, and the far test of collide_native.pack_fields.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "scene.hpp"

namespace acol {

inline double field_lookup(const Scene& S, int f, const double* p) {
    const double* o = S.fl_origin + 3 * f;
    const int64_t* n = S.fl_dims + 3 * f;
    const double c = S.fl_cell[f];
    const float* d = S.fl_data + S.fl_off[f];
    double q[3];
    int64_t i[3];
    for (int k = 0; k < 3; ++k) {
        const double hi = o[k] + double(n[k] - 1) * c;
        q[k] = std::min(std::max(p[k], o[k]), hi);
        int64_t ik = int64_t(std::floor((q[k] - o[k]) / c));
        i[k] = std::min(std::max(ik, int64_t(0)), std::max(n[k] - 2, int64_t(0)));
    }
    double best = -INF;
    for (int ax = 0; ax < 2; ++ax) {
        const int64_t ix = std::min(i[0] + ax, n[0] - 1);
        const double dx = q[0] - (o[0] + double(ix) * c);
        for (int ay = 0; ay < 2; ++ay) {
            const int64_t iy = std::min(i[1] + ay, n[1] - 1);
            const double dy = q[1] - (o[1] + double(iy) * c);
            for (int az = 0; az < 2; ++az) {
                const int64_t iz = std::min(i[2] + az, n[2] - 1);
                const double dz = q[2] - (o[2] + double(iz) * c);
                const double v = double(d[(ix * n[1] + iy) * n[2] + iz]) -
                                 std::sqrt(dx * dx + dy * dy + dz * dz);
                best = std::max(best, v);
            }
        }
    }
    double e[3] = {p[0] - q[0], p[1] - q[1], p[2] - q[2]};
    const double e2 = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    const double pos = std::max(best, 0.0);
    return e2 > 0.0 ? std::sqrt(pos * pos + e2) : best;
}

inline double segment_field_distance(const Scene& S, int f, const double* a, const double* b) {
    const double c = S.fl_cell[f];
    double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double L = std::sqrt(dot(u, u));
    const int64_t n = L > 0.0 ? int64_t(std::ceil(L / c)) + 1 : 1;
    double v = INF;
    for (int64_t j = 0; j < n; ++j) {
        const double t = double(std::min(j, n - 1)) / double(std::max(n - 1, int64_t(1)));
        double p[3];
        for (int k = 0; k < 3; ++k) p[k] = a[k] + t * u[k];
        v = std::min(v, field_lookup(S, f, p));
    }
    const double s = n > 1 ? L / double(std::max(n - 1, int64_t(1))) : 0.0;
    return v - 0.5 * s;
}

// Lower bound on the value of every pair between a body group's sphere (centre, radius R)
// and field f: see collide_native.pack_fields.
inline double field_group_bound(const Scene& S, int f, const double* sp) {
    const double* o = S.fl_origin + 3 * f;
    const int64_t* n = S.fl_dims + 3 * f;
    const double c = S.fl_cell[f], h = 0.5 * std::sqrt(3.0) * c, R = sp[3];
    double q[3];
    for (int k = 0; k < 3; ++k) q[k] = std::min(std::max(sp[k], o[k]), o[k] + double(n[k] - 1) * c);
    double tau = S.fl_min[f];
    for (int lv = 0; lv < S.NL; ++lv) {
        const double* bx = S.fl_box + (size_t(f) * S.NL + lv) * 6;
        double e2 = 0.0;
        bool empty = false;
        for (int k = 0; k < 3; ++k) {
            if (bx[k] > bx[3 + k]) { empty = true; break; }
            const double e = std::max(std::max(bx[k] - q[k], q[k] - bx[3 + k]), 0.0);
            e2 += e * e;
        }
        if (empty || std::sqrt(e2) > R + h) {
            tau = std::max(tau, S.fl_tau[lv]);
            break;
        }
    }
    return tau - h - 0.5 * c - R - S.fl_margin[f];
}

}  // namespace acol
