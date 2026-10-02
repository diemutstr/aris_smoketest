// The packed obstacles as the compiled engine sees them (built in bindings.cpp).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "geometry.hpp"

namespace acol {

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double BIG = 1e6;  // "nothing to hit", keeps the interval arithmetic finite

struct Scene {  // the packed obstacles
    int Mb = 0, Mp = 0, Mc = 0;
    const double *box_R, *box_c, *box_h, *box_m;               // (Mb,3,3) (Mb,3) (Mb,3) (Mb)
    const double *pl_n, *pl_off, *pl_m, *pl_pen_m;             // (Mp,3) (Mp) (Mp) (Mp)
    const uint8_t* pl_paper;                                   // (Mp)
    const double *cap_a, *cap_b, *cap_rm;                      // (Mc,3) (Mc,3) (Mc)
    const double* pl_tool_m;                                   // (Mp) margin for tool capsules
    // The same boxes and capsules one coordinate per row (see make_soa), so that the cheap
    // first pass runs as straight loops over obstacles that the compiler vectorises.
    const double* box_soa;   // (15, Mb): R00 R01 R02 R10 .. R22, cx cy cz, hx hy hz
    const double* cap_soa;   // (10, Mc): ax ay az, dx dy dz (d = b - a), dd = d.d, (3 unused)
    // Obstacle groups (boxes and capsules; planes are always checked one by one).  Group h
    // holds obstacles og_members[og_start[h] .. og_start[h+1]) (global obstacle indices), all
    // inside one fat capsule og_a/og_b/og_r, and demands at most og_margin[h].
    int G = 0;
    const int64_t *og_start, *og_members;
    const double *og_a, *og_b, *og_r, *og_margin;
    const double* og_soa;    // (8, G): ax ay az, dx dy dz, dd, og_r + og_margin
};

// Fill the row-per-coordinate copies of a scene's boxes and capsules.
inline void make_soa(const Scene& S, std::vector<double>& box, std::vector<double>& cap,
                     std::vector<double>& og) {
    og.assign(size_t(8) * S.G, 0.0);
    for (int h = 0; h < S.G; ++h) {
        double d[3];
        for (int i = 0; i < 3; ++i) {
            d[i] = S.og_b[3 * h + i] - S.og_a[3 * h + i];
            og[size_t(i) * S.G + h] = S.og_a[3 * h + i];
            og[size_t(3 + i) * S.G + h] = d[i];
        }
        og[size_t(6) * S.G + h] = dot(d, d);
        og[size_t(7) * S.G + h] = S.og_r[h] + S.og_margin[h];
    }
    box.assign(size_t(15) * S.Mb, 0.0);
    cap.assign(size_t(10) * S.Mc, 0.0);
    for (int m = 0; m < S.Mb; ++m) {
        for (int i = 0; i < 9; ++i) box[size_t(i) * S.Mb + m] = S.box_R[9 * m + i];
        for (int i = 0; i < 3; ++i) {
            box[size_t(9 + i) * S.Mb + m] = S.box_c[3 * m + i];
            box[size_t(12 + i) * S.Mb + m] = S.box_h[3 * m + i];
        }
    }
    for (int m = 0; m < S.Mc; ++m) {
        double d[3];
        for (int i = 0; i < 3; ++i) {
            d[i] = S.cap_b[3 * m + i] - S.cap_a[3 * m + i];
            cap[size_t(i) * S.Mc + m] = S.cap_a[3 * m + i];
            cap[size_t(3 + i) * S.Mc + m] = d[i];
        }
        cap[size_t(6) * S.Mc + m] = dot(d, d);
    }
}

}  // namespace acol
