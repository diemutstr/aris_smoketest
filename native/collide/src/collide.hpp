// The collision check over a batch: obstacles, the arm's capsules, the arm's kinematic chain.
//
// Mirrors aris/kernel/collide.py: the same clearance per pair, the same definition of each
// capsule's value, the same "first smallest" choice of the closest obstacle, the same path bound.  No robot numbers
// live here: the chain and the capsules are tables handed in by the caller.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>
#include <vector>

#include "chain.hpp"
#include "geometry.hpp"
#include "scene.hpp"

namespace acol {

// Value-returning max and clip: no reference juggling, so the loops below stay branch-free
// and the compiler vectorises them.
inline double vmax(double a, double b) { return a < b ? b : a; }
inline double vclip01(double x) { x = x < 0.0 ? 0.0 : x; return x > 1.0 ? 1.0 : x; }

// ub[m] = distance from point c to box m minus r and the box margin, for all boxes.
inline void point_boxes(const Scene& S, const double* c, double r, double* ub) {
    const int Mb = S.Mb;
    const double* B = S.box_soa;
    const double c0 = c[0], c1 = c[1], c2 = c[2];
    for (int m = 0; m < Mb; ++m) {
        const double vx = c0 - B[9 * Mb + m], vy = c1 - B[10 * Mb + m], vz = c2 - B[11 * Mb + m];
        const double x0 = vx * B[m] + vy * B[3 * Mb + m] + vz * B[6 * Mb + m];
        const double x1 = vx * B[Mb + m] + vy * B[4 * Mb + m] + vz * B[7 * Mb + m];
        const double x2 = vx * B[2 * Mb + m] + vy * B[5 * Mb + m] + vz * B[8 * Mb + m];
        const double e0 = vmax(std::fabs(x0) - B[12 * Mb + m], 0.0);
        const double e1 = vmax(std::fabs(x1) - B[13 * Mb + m], 0.0);
        const double e2 = vmax(std::fabs(x2) - B[14 * Mb + m], 0.0);
        ub[m] = std::sqrt(e0 * e0 + e1 * e1 + e2 * e2) - r - S.box_m[m];
    }
}

// ub[m] = distance from point c to capsule m's axis minus r and its radius + margin.
inline void point_capsules(const Scene& S, const double* c, double r, double* ub) {
    const int Mc = S.Mc;
    const double* A = S.cap_soa;
    for (int m = 0; m < Mc; ++m) {
        const double ax = A[m], ay = A[Mc + m], az = A[2 * Mc + m];
        const double dx = A[3 * Mc + m], dy = A[4 * Mc + m], dz = A[5 * Mc + m], dd = A[6 * Mc + m];
        const double px = c[0] - ax, py = c[1] - ay, pz = c[2] - az;
        const double t = vclip01((px * dx + py * dy + pz * dz) / (dd > 0.0 ? dd : 1.0));
        const double wx = ax + t * dx - c[0], wy = ay + t * dy - c[1], wz = az + t * dz - c[2];
        ub[m] = std::sqrt(wx * wx + wy * wy + wz * wz) - r - S.cap_rm[m];
    }
}

struct Caps {  // the arm's capsules, per capsule
    int K = 0;
    const double* r;
    const uint8_t *is_pen, *is_fixed, *is_tool;
    // (K, Mb) 1 where capsule k is never checked against box b (Box.exempt); null: none
    const uint8_t* exempt = nullptr;
    bool exempt_from(int k, int64_t o, int Mb) const {
        return exempt && o < Mb && exempt[size_t(k) * Mb + o];
    }
    // body groups: group g holds capsules bg_members[bg_start[g] .. bg_start[g+1])
    int NG = 0;
    const int64_t *bg_start, *bg_members;
};

// How one evaluation may save work.  The answer does not depend on it (see eval_config).
struct Mode {
    bool prune = true;    // skip pairs by the midpoint bound
    bool groups = true;   // skip whole groups first (needs prune)
    double span = 0.0;    // per-capsule values are exact up to (configuration minimum + span)
    int64_t* count = nullptr;  // if set, adds the number of exact pair distances taken
};

struct Scratch {
    std::vector<double> pl, ub, R, p, p0, p1, mid, half, sph, gl;
    std::vector<int> order;
};

// A pair's clearance, given the obstacle's global index o (a box or a capsule).
inline double pair_exact(const Scene& S, const double* a, const double* b, double r, int64_t o) {
    if (o < S.Mb)
        return segment_box_distance(a, b, S.box_R + 9 * o, S.box_c + 3 * o, S.box_h + 3 * o) - r -
               S.box_m[o];
    const int64_t m = o - S.Mb - S.Mp;
    return segment_segment_distance(a, b, S.cap_a + 3 * m, S.cap_b + 3 * m) - r - S.cap_rm[m];
}

// The same pair measured from the capsule's midpoint c (an upper bound on the pair's value
// plus half the capsule's length; minus that half length, a lower bound).
inline double pair_mid(const Scene& S, const double* c, double r, int64_t o) {
    if (o < S.Mb)
        return point_box_distance(c, S.box_R + 9 * o, S.box_c + 3 * o, S.box_h + 3 * o) - r - S.box_m[o];
    const int64_t m = o - S.Mb - S.Mp;
    return point_segment_distance(c, S.cap_a + 3 * m, S.cap_b + 3 * m) - r - S.cap_rm[m];
}

// Keep the smaller value; on a tie the smaller obstacle index, so the order of work is moot.
inline void keep(double v, int64_t o, double& val, int64_t& arg) {
    if (v < val || (v == val && o < arg && std::isfinite(v))) {
        val = v;
        arg = o;
    }
}

// Clip every capsule's value at (configuration minimum + span): the answer is then the same
// whichever pairs the work-saving skipped, because a skipped pair is proven above that.
inline void finish(const Caps& C, double span, double* val, int64_t* arg) {
    double m = INF;
    for (int k = 0; k < C.K; ++k)
        if (!C.is_fixed[k]) m = std::min(m, val[k]);
    if (!std::isfinite(m)) return;
    for (int k = 0; k < C.K; ++k)
        if (!C.is_fixed[k] && val[k] > m + span) {
            val[k] = m + span;
            arg[k] = -1;
        }
}

// Bounding sphere (centre, radius) of each body group's live capsules, into w.sph (NG,4);
// radius -1 for a group with nothing live.
inline void group_spheres(const Caps& C, const double* p0, const double* p1, bool live_only,
                          Scratch& w) {
    w.sph.assign(size_t(C.NG) * 4, 0.0);
    for (int g = 0; g < C.NG; ++g) {
        double lo[3] = {INF, INF, INF}, hi[3] = {-INF, -INF, -INF};
        bool any = false;
        for (int64_t n = C.bg_start[g]; n < C.bg_start[g + 1]; ++n) {
            const int64_t k = C.bg_members[n];
            if (live_only && C.is_fixed[k]) continue;
            any = true;
            for (int i = 0; i < 3; ++i) {
                lo[i] = std::min(lo[i], std::min(p0[3 * k + i], p1[3 * k + i]));
                hi[i] = std::max(hi[i], std::max(p0[3 * k + i], p1[3 * k + i]));
            }
        }
        double* sp = w.sph.data() + 4 * g;
        if (!any) { sp[3] = -1.0; continue; }
        for (int i = 0; i < 3; ++i) sp[i] = 0.5 * (lo[i] + hi[i]);
        double R = 0.0;
        for (int64_t n = C.bg_start[g]; n < C.bg_start[g + 1]; ++n) {
            const int64_t k = C.bg_members[n];
            if (live_only && C.is_fixed[k]) continue;
            double u[3], v[3];
            for (int i = 0; i < 3; ++i) {
                u[i] = p0[3 * k + i] - sp[i];
                v[i] = p1[3 * k + i] - sp[i];
            }
            R = std::max(R, std::sqrt(std::max(dot(u, u), dot(v, v))) + C.r[k]);
        }
        sp[3] = R;
    }
}

// Lower bound gl (NG, G) for every body group g against every obstacle group h, into w.gl
// (+inf for a body group with nothing live); returns the index g * G + h of the smallest.
inline int group_bounds(const Scene& S, const Caps& C, Scratch& w) {
    w.gl.assign(size_t(C.NG) * S.G, INF);
    int arg = -1;
    double lo = INF;
    for (int g = 0; g < C.NG; ++g) {
        const double* sp = w.sph.data() + 4 * g;
        if (sp[3] < 0.0) continue;
        double* gl = w.gl.data() + size_t(g) * S.G;
        const int G = S.G;
        const double* A = S.og_soa;
        const double cx = sp[0], cy = sp[1], cz = sp[2], R = sp[3];
        for (int h = 0; h < G; ++h) {  // point to fat capsule, written out so it vectorises
            const double ax = A[h], ay = A[G + h], az = A[2 * G + h];
            const double dx = A[3 * G + h], dy = A[4 * G + h], dz = A[5 * G + h], dd = A[6 * G + h];
            const double px = cx - ax, py = cy - ay, pz = cz - az;
            const double t = vclip01((px * dx + py * dy + pz * dz) / (dd > 0.0 ? dd : 1.0));
            const double wx = ax + t * dx - cx, wy = ay + t * dy - cy, wz = az + t * dz - cz;
            gl[h] = std::sqrt(wx * wx + wy * wy + wz * wz) - A[7 * G + h] - R;
        }
        for (int h = 0; h < S.G; ++h)
            if (gl[h] < lo) { lo = gl[h]; arg = g * S.G + h; }
    }
    return arg;
}

// Clearance of each capsule of one configuration against the obstacles.
//
// val[k] = min(true clearance of capsule k, m + span), m being the configuration's minimum;
// arg[k] = the obstacle attaining val[k] (smallest index on a tie), -1 if clipped or none.
// Fixed capsules: +inf, -1.  These are exact definitions, so every Mode gives the same
// numbers: a pair is only skipped when a lower bound proves it above (best + span), and best
// never falls below m.  With span 0 only the closest capsule's value is exact; the others are
// lower bounds; that is all a configuration's clearance needs.
inline void eval_config(const Scene& S, const Caps& C, const double* p0, const double* p1,
                        bool drawing, const Mode& md, double* val, int64_t* arg, Scratch& w) {
    const int K = C.K, Mb = S.Mb, Mp = S.Mp, Mc = S.Mc;
    const double span = md.span;
    int64_t n_exact = 0;
    double best = INF;
    w.mid.resize(3 * size_t(K));
    w.half.resize(K);
    for (int k = 0; k < K; ++k) {
        val[k] = INF;
        arg[k] = -1;
        if (C.is_fixed[k]) continue;
        const double *a = p0 + 3 * k, *b = p1 + 3 * k;
        for (int m = 0; m < Mp; ++m) {
            double v = segment_plane_distance(a, b, S.pl_n + 3 * m, S.pl_off[m]) - C.r[k] -
                       (C.is_pen[k] ? S.pl_pen_m[m] : (C.is_tool[k] ? S.pl_tool_m[m] : S.pl_m[m]));
            if (drawing && C.is_pen[k] && S.pl_paper[m]) v = INF;
            keep(v, Mb + m, val[k], arg[k]);
            best = std::min(best, v);
        }
        double u[3];
        for (int i = 0; i < 3; ++i) {
            w.mid[3 * k + i] = 0.5 * (a[i] + b[i]);
            u[i] = b[i] - a[i];
        }
        w.half[k] = 0.5 * std::sqrt(dot(u, u));
    }
    auto exact = [&](int k, int64_t o) {
        const double v = pair_exact(S, p0 + 3 * k, p1 + 3 * k, C.r[k], o);
        ++n_exact;
        best = std::min(best, v);
        keep(v, o, val[k], arg[k]);
    };
    if (!md.prune) {
        for (int k = 0; k < K; ++k)
            if (!C.is_fixed[k]) {
                for (int64_t o = 0; o < Mb; ++o)
                    if (!C.exempt_from(k, o, Mb)) exact(k, o);
                for (int64_t o = Mb + Mp; o < Mb + Mp + Mc; ++o) exact(k, o);
            }
    } else if (!md.groups) {
        const int Mo = Mb + Mc;
        w.ub.resize(size_t(K) * Mo);
        for (int k = 0; k < K; ++k) {
            if (C.is_fixed[k]) continue;
            double* ub = w.ub.data() + size_t(k) * Mo;
            point_boxes(S, w.mid.data() + 3 * k, C.r[k], ub);
            for (int m = 0; m < Mb; ++m)  // an exempt pair is neither checked nor a bound
                if (C.exempt_from(k, m, Mb)) ub[m] = INF;
            point_capsules(S, w.mid.data() + 3 * k, C.r[k], ub + Mb);
            for (int m = 0; m < Mo; ++m) best = std::min(best, ub[m]);
        }
        for (int k = 0; k < K; ++k) {
            if (C.is_fixed[k]) continue;
            const double* ub = w.ub.data() + size_t(k) * Mo;
            for (int m = 0; m < Mo; ++m)
                if (!(ub[m] - w.half[k] > best + span)) exact(k, m < Mb ? m : m + Mp);
        }
    } else {
        group_spheres(C, p0, p1, true, w);
        const int first = group_bounds(S, C, w);
        // the closest group pair first, for a good `best`; then every other that may matter
        const int n_gh = C.NG * S.G;
        for (int t = -1; t < n_gh && first >= 0; ++t) {
            const int gh = t < 0 ? first : t;
            if ((t >= 0 && gh == first) || w.gl[gh] > best + span) continue;
            const int g = gh / S.G, h = gh % S.G;
            const int64_t o0 = S.og_start[h], o1 = S.og_start[h + 1];
            const int64_t no = o1 - o0;
            w.ub.resize(size_t(C.bg_start[g + 1] - C.bg_start[g]) * no);
            double* ub = w.ub.data();
            for (int64_t n = C.bg_start[g]; n < C.bg_start[g + 1]; ++n, ub += no) {
                const int64_t k = C.bg_members[n];
                if (C.is_fixed[k]) continue;
                for (int64_t j = 0; j < no; ++j) {
                    const int64_t o = S.og_members[o0 + j];
                    ub[j] = C.exempt_from(int(k), o, Mb) ? INF : pair_mid(S, w.mid.data() + 3 * k, C.r[k], o);
                    best = std::min(best, ub[j]);
                }
            }
            ub = w.ub.data();
            for (int64_t n = C.bg_start[g]; n < C.bg_start[g + 1]; ++n, ub += no) {
                const int64_t k = C.bg_members[n];
                if (C.is_fixed[k]) continue;
                for (int64_t j = 0; j < no; ++j)
                    if (!(ub[j] - w.half[k] > best + span)) exact(int(k), S.og_members[o0 + j]);
            }
        }
    }
    finish(C, span, val, arg);
    if (md.count) *md.count += n_exact;
}

inline void count_add(int64_t* total, int64_t n) {  // only for the optional statistics
    __atomic_fetch_add(total, n, __ATOMIC_RELAXED);
}

// Run fn(i0, i1, scratch) over [0, n) in `threads` contiguous blocks.  Each configuration is
// computed alone, so the result does not depend on the thread count.
template <class Fn>
void parallel_for(int64_t n, int threads, Fn fn) {
    if (threads <= 1 || n < 2 * threads) {
        Scratch w;
        fn(int64_t(0), n, w);
        return;
    }
    std::vector<std::thread> pool;
    const int64_t step = (n + threads - 1) / threads;
    for (int64_t i0 = 0; i0 < n; i0 += step)
        pool.emplace_back([&, i0] {
            Scratch w;
            fn(i0, std::min(n, i0 + step), w);
        });
    for (auto& th : pool) th.join();
}

// Per-capsule clearance for configurations Q (N,J): val, arg (N,K).
inline void eval_q(const Chain& H, const Caps& C, const Scene& S, const double* Q, int64_t N,
                   bool drawing, const Mode& md, int threads, double* val, int64_t* arg) {
    parallel_for(N, threads, [&](int64_t i0, int64_t i1, Scratch& w) {
        std::vector<double> p0(size_t(C.K) * 3), p1(size_t(C.K) * 3);
        Mode mt = md;
        int64_t cnt = 0;
        mt.count = md.count ? &cnt : nullptr;
        for (int64_t i = i0; i < i1; ++i) {
            chain_frames(H, Q + i * H.J, w);
            chain_caps(H, C.K, p0.data(), p1.data(), w);
            eval_config(S, C, p0.data(), p1.data(), drawing, mt, val + i * C.K, arg + i * C.K, w);
        }
        if (md.count) count_add(md.count, cnt);
    });
}

}  // namespace acol
