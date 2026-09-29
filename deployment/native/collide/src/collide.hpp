// The collision check over a batch: obstacles, the arm's capsules, the arm's kinematic chain.
//
// Mirrors aris/kernel/collide.py: the same clearance per pair, the same midpoint pruning, the
// same "first smallest" choice of the closest obstacle, the same path bound.  No robot numbers
// live here: the chain and the capsules are tables handed in by the caller.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>
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
    // The same boxes and capsules one coordinate per row (see make_soa), so that the cheap
    // first pass runs as straight loops over obstacles that the compiler vectorises.
    const double* box_soa;   // (15, Mb): R00 R01 R02 R10 .. R22, cx cy cz, hx hy hz
    const double* cap_soa;   // (10, Mc): ax ay az, dx dy dz (d = b - a), dd = d.d, (3 unused)
};

// Fill the row-per-coordinate copies of a scene's boxes and capsules.
inline void make_soa(const Scene& S, std::vector<double>& box, std::vector<double>& cap) {
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
    const uint8_t *is_pen, *is_fixed;
};

struct Chain {  // serial arm: modified DH rows, then fixed extra frames, then capsules on frames
    int J = 0, E = 0;
    const double* dh;          // (J,3) alpha, a, d; frame i+1 from frame i, joint i about its z
    const int64_t* ex_parent;  // (E) frame index each extra frame hangs from
    const double *ex_R, *ex_t; // (E,3,3) (E,3) the extra frame in its parent
    const int64_t* cap_frame;  // (K) frame each capsule rides on
    const double *cap_a, *cap_b;  // (K,3) capsule ends in that frame
    int frames() const { return 1 + J + E; }
};

struct Scratch {
    std::vector<double> pl, ub, R, p, p0, p1;
};

// Clearance of each capsule of one configuration: val[k] (smallest over obstacles), arg[k]
// (which obstacle: boxes, then planes, then capsules; -1 none).  With `prune`, pairs that
// cannot be the configuration's minimum keep their midpoint lower bound, as in collide.py.
inline void eval_config(const Scene& S, const Caps& C, const double* p0, const double* p1,
                        bool drawing, bool prune, double* val, int64_t* arg, Scratch& w) {
    const int K = C.K, Mb = S.Mb, Mp = S.Mp, Mc = S.Mc, Mo = Mb + Mc;
    w.pl.resize(size_t(K) * Mp);
    w.ub.resize(size_t(K) * Mo + K);
    double* half = w.ub.data() + size_t(K) * Mo;
    double best = INF;
    for (int k = 0; k < K; ++k) {
        if (C.is_fixed[k]) continue;
        const double *a = p0 + 3 * k, *b = p1 + 3 * k;
        for (int m = 0; m < Mp; ++m) {
            double v = segment_plane_distance(a, b, S.pl_n + 3 * m, S.pl_off[m]) - C.r[k] -
                       (C.is_pen[k] ? S.pl_pen_m[m] : S.pl_m[m]);
            if (drawing && C.is_pen[k] && S.pl_paper[m]) v = INF;
            w.pl[size_t(k) * Mp + m] = v;
            best = std::min(best, v);
        }
        if (!prune) continue;
        double c[3], u[3];
        for (int i = 0; i < 3; ++i) {
            c[i] = 0.5 * (a[i] + b[i]);
            u[i] = b[i] - a[i];
        }
        half[k] = 0.5 * std::sqrt(dot(u, u));
        double* ub = w.ub.data() + size_t(k) * Mo;
        point_boxes(S, c, C.r[k], ub);
        point_capsules(S, c, C.r[k], ub + Mb);
        for (int m = 0; m < Mo; ++m) best = std::min(best, ub[m]);
    }
    for (int k = 0; k < K; ++k) {
        double bv = INF;
        int64_t bi = -1;
        if (!C.is_fixed[k]) {
            const double *a = p0 + 3 * k, *b = p1 + 3 * k;
            const double* ub = w.ub.data() + size_t(k) * Mo;
            for (int m = 0; m < Mb; ++m) {
                double v;
                if (prune && ub[m] - half[k] > best) v = ub[m] - half[k];
                else v = segment_box_distance(a, b, S.box_R + 9 * m, S.box_c + 3 * m,
                                              S.box_h + 3 * m) - C.r[k] - S.box_m[m];
                if (v < bv) { bv = v; bi = m; }
            }
            for (int m = 0; m < Mp; ++m) {
                const double v = w.pl[size_t(k) * Mp + m];
                if (v < bv) { bv = v; bi = Mb + m; }
            }
            for (int m = 0; m < Mc; ++m) {
                double v;
                if (prune && ub[Mb + m] - half[k] > best) v = ub[Mb + m] - half[k];
                else v = segment_segment_distance(a, b, S.cap_a + 3 * m, S.cap_b + 3 * m) -
                         C.r[k] - S.cap_rm[m];
                if (v < bv) { bv = v; bi = Mb + Mp + m; }
            }
        }
        val[k] = bv;
        arg[k] = std::isfinite(bv) ? bi : -1;
    }
}

// Frames of one configuration q (J joints) into w.R (F,3,3), w.p (F,3).
inline void chain_frames(const Chain& H, const double* q, Scratch& w) {
    const int F = H.frames();
    w.R.assign(size_t(F) * 9, 0.0);
    w.p.assign(size_t(F) * 3, 0.0);
    double *R = w.R.data(), *p = w.p.data();
    R[0] = R[4] = R[8] = 1.0;
    auto compose = [&](int par, int out, const double* A, const double* t) {
        const double *Rp = R + 9 * par, *pp = p + 3 * par;
        double* Ro = R + 9 * out;
        double* po = p + 3 * out;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c)
                Ro[3 * r + c] = Rp[3 * r] * A[c] + Rp[3 * r + 1] * A[3 + c] + Rp[3 * r + 2] * A[6 + c];
            po[r] = pp[r] + (Rp[3 * r] * t[0] + Rp[3 * r + 1] * t[1] + Rp[3 * r + 2] * t[2]);
        }
    };
    for (int i = 0; i < H.J; ++i) {
        const double al = H.dh[3 * i], a = H.dh[3 * i + 1], d = H.dh[3 * i + 2];
        const double ca = std::cos(al), sa = std::sin(al), ct = std::cos(q[i]), st = std::sin(q[i]);
        const double A[9] = {ct, -st, 0.0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca};
        const double t[3] = {a, -sa * d, ca * d};
        compose(i, i + 1, A, t);
    }
    for (int e = 0; e < H.E; ++e) compose(int(H.ex_parent[e]), 1 + H.J + e, H.ex_R + 9 * e, H.ex_t + 3 * e);
}

// Capsule ends (K,3) from the frames in w.
inline void chain_caps(const Chain& H, int K, double* p0, double* p1, const Scratch& w) {
    for (int k = 0; k < K; ++k) {
        const double *Rf = w.R.data() + 9 * H.cap_frame[k], *pf = w.p.data() + 3 * H.cap_frame[k];
        const double *a = H.cap_a + 3 * k, *b = H.cap_b + 3 * k;
        for (int r = 0; r < 3; ++r) {
            p0[3 * k + r] = (Rf[3 * r] * a[0] + Rf[3 * r + 1] * a[1] + Rf[3 * r + 2] * a[2]) + pf[r];
            p1[3 * k + r] = (Rf[3 * r] * b[0] + Rf[3 * r + 1] * b[1] + Rf[3 * r + 2] * b[2]) + pf[r];
        }
    }
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
                   bool drawing, bool prune, int threads, double* val, int64_t* arg) {
    parallel_for(N, threads, [&](int64_t i0, int64_t i1, Scratch& w) {
        std::vector<double> p0(size_t(C.K) * 3), p1(size_t(C.K) * 3);
        for (int64_t i = i0; i < i1; ++i) {
            chain_frames(H, Q + i * H.J, w);
            chain_caps(H, C.K, p0.data(), p1.data(), w);
            eval_config(S, C, p0.data(), p1.data(), drawing, prune, val + i * C.K, arg + i * C.K, w);
        }
    });
}

}  // namespace acol
