// The arm's kinematic chain as a table: joint angles -> frames -> capsule ends.
// No robot numbers here; the table comes from aris.kernel.collide.arm_tables.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace acol {

struct Chain {  // serial arm: modified DH rows, then fixed extra frames, then capsules on frames
    int J = 0, E = 0;
    const double* dh;          // (J,3) alpha, a, d; frame i+1 from frame i, joint i about its z
    const int64_t* ex_parent;  // (E) frame index each extra frame hangs from
    const double *ex_R, *ex_t; // (E,3,3) (E,3) the extra frame in its parent
    const int64_t* cap_frame;  // (K) frame each capsule rides on
    const double *cap_a, *cap_b;  // (K,3) capsule ends in that frame
    int frames() const { return 1 + J + E; }
};

// Frames of one configuration q (J joints) into w.R (F,3,3), w.p (F,3) (w: any scratch with
// vectors R and p).
template <class W>
inline void chain_frames(const Chain& H, const double* q, W& w) {
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
template <class W>
inline void chain_caps(const Chain& H, int K, double* p0, double* p1, const W& w) {
    for (int k = 0; k < K; ++k) {
        const double *Rf = w.R.data() + 9 * H.cap_frame[k], *pf = w.p.data() + 3 * H.cap_frame[k];
        const double *a = H.cap_a + 3 * k, *b = H.cap_b + 3 * k;
        for (int r = 0; r < 3; ++r) {
            p0[3 * k + r] = (Rf[3 * r] * a[0] + Rf[3 * r + 1] * a[1] + Rf[3 * r + 2] * a[2]) + pf[r];
            p1[3 * k + r] = (Rf[3 * r] * b[0] + Rf[3 * r + 1] * b[1] + Rf[3 * r + 2] * b[2]) + pf[r];
        }
    }
}

}  // namespace acol
