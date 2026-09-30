// Analytic inverse kinematics of the Franka FR3 for a given joint-7 angle, all roots.
//
// Input: the pose of the HAND frame (the flange turned -45 deg about z, the stock Franka hand
// mounting) in the base frame, the joint-7 angle, and the joint limits.  Output: up to 8
// configurations.  No limit is built in.
//
// The geometry.  With q7 fixed, the hand pose fixes the frame of link 6, and so the wrist
// centre W (joints 5 and 6 share an origin).  Joints 1-4 place W: the shoulder point p2, the
// elbow and W lie in one plane (the elbow plane), and |W - p2| depends on q4 alone.
//   1. q4: |W - p2| = |P + R(q4) D| in the elbow plane; two roots, +-acos.
//   2. The forearm direction z5 (joint 5's axis) lies in the elbow plane at a fixed angle to
//      W - p2 once q4 is known, so it lies on a cone about W - p2; and it must be square to
//      joint 6's axis z6, which is known.  Cone meets great circle: two roots.
//   3. z5 and W - p2 fix the elbow plane, hence link 3's frame; the direction of link 3's
//      axis gives (q1, q2) in two ways, (q1, q2) and (q1 + pi, -q2); then q3.
//   4. q5 and q6 from link 4's frame and link 6's frame.
// Slot = 4 * (q4 root) + 2 * (cone root) + (shoulder branch).
//
// Degenerate cases are not dropped silently; each answer carries flags:
//   SHOULDER (1): q2 is 0 to within 1e-10 rad; q1 and q3 then turn about the same axis and only
//                 their combination is fixed.  One answer is returned with q1 = 0 (or the
//                 nearest value that keeps q3 inside its limits); the second branch is empty.
//   PLANE    (2): the forearm points along W - p2, so the elbow plane may turn freely about
//                 that line; one plane is picked.
//   CONE     (4): joint 6's axis points along W - p2 and the forearm is square to it, so every
//                 forearm direction on the circle works; two are picked.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace fr3ik {

constexpr double D1 = 0.333, D3 = 0.316, D5 = 0.384, A4 = 0.0825, A7 = 0.088;
constexpr double D_FLANGE = 0.107;
constexpr int N_SOL = 8;
constexpr uint8_t SHOULDER = 1, PLANE = 2, CONE = 4;
constexpr double EPS_SING = 1e-10;

struct V3 {
    double x, y, z;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(double s, V3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 unit(V3 a) { return (1.0 / norm(a)) * a; }

// Bring q into [lo, hi] by a whole turn if that is possible; NaN if it is not.
inline double wrap_into(double q, double lo, double hi) {
    const double two_pi = 2.0 * M_PI;
    while (q > hi) q -= two_pi;
    while (q < lo) q += two_pi;
    return (q <= hi) ? q : NAN;
}

// One function on purpose (longer than the 60-line rule): it is one derivation, steps 1-4
// above, and splitting it would pass a dozen intermediate vectors between the pieces.
// T: hand pose, 16 values ROW-major.  out: N_SOL x 7 (NaN = no answer), flags: N_SOL.
inline void solve(const double* T, double q7, const double* lo, const double* hi,
                  double* out, uint8_t* flags) {
    for (int k = 0; k < N_SOL * 7; ++k) out[k] = NAN;
    for (int k = 0; k < N_SOL; ++k) flags[k] = 0;
    if (!(q7 >= lo[6] && q7 <= hi[6])) return;

    const V3 xh{T[0], T[4], T[8]}, yh{T[1], T[5], T[9]}, zh{T[2], T[6], T[10]};
    const V3 ph{T[3], T[7], T[11]};
    // link 7 = hand turned back +45 deg; link 6 from link 7 and q7
    const double c = std::cos(M_PI / 4), s = std::sin(M_PI / 4);
    const V3 x7 = c * xh + s * yh, y7 = c * yh - s * xh, z7 = zh;
    const V3 p7 = ph - D_FLANGE * z7;
    const double c7 = std::cos(q7), s7 = std::sin(q7);
    const V3 x6 = c7 * x7 - s7 * y7;
    const V3 z6 = s7 * x7 + c7 * y7;
    const V3 W = p7 - A7 * x6;
    const V3 p2{0.0, 0.0, D1};
    const V3 V = W - p2;
    const double L = norm(V);
    if (L < 1e-12) return;
    const V3 a = (1.0 / L) * V;

    // 1. q4.  In the elbow plane (u along x3, v along z3, origin p2): W = P + R(q4) D.
    const double Pu = A4, Pv = D3, Du = -A4, Dv = D5;
    const double nP = std::hypot(Pu, Pv), nD = std::hypot(Du, Dv);
    double k = (L * L - nP * nP - nD * nD) / (2.0 * nP * nD);
    if (std::fabs(k) > 1.0 + 1e-12) return;
    k = std::fmax(-1.0, std::fmin(1.0, k));
    const double delta = std::atan2(Dv, Du) - std::atan2(Pv, Pu);
    const double q4_roots[2] = {-delta + std::acos(k), -delta - std::acos(k)};

    // a basis square to a, for the cone
    const V3 ex{1, 0, 0}, ey{0, 1, 0}, ez{0, 0, 1};
    const double ax = std::fabs(a.x), ay = std::fabs(a.y), az = std::fabs(a.z);
    const V3 e = (ax <= ay && ax <= az) ? ex : ((ay <= az) ? ey : ez);
    const V3 b1 = unit(cross(a, e));
    const V3 b2 = cross(a, b1);

    for (int i4 = 0; i4 < 2; ++i4) {
        const double q4 = wrap_into(q4_roots[i4], lo[3], hi[3]);
        if (std::isnan(q4)) continue;
        const double c4 = std::cos(q4), s4 = std::sin(q4);
        const double Wu = Pu + c4 * Du - s4 * Dv, Wv = Pv + s4 * Du + c4 * Dv;
        const double beta = std::atan2(Wv, Wu);
        const double alpha_s = std::remainder(q4 + M_PI / 2 - beta, 2.0 * M_PI);
        const double ca = std::cos(alpha_s), sa = std::fabs(std::sin(alpha_s));

        // 2. forearm direction: z5 = ca a + sa (cos psi b1 + sin psi b2), z5 . z6 = 0
        const double A = sa * dot(b1, z6), B = sa * dot(b2, z6), C = ca * dot(a, z6);
        const double R = std::hypot(A, B);
        double psi[2];
        uint8_t flag_cone = 0;
        if (R < 1e-12) {
            if (std::fabs(C) > 1e-12) continue;
            psi[0] = 0.0;
            psi[1] = M_PI;
            flag_cone = CONE;
        } else {
            double r = -C / R;
            if (std::fabs(r) > 1.0 + 1e-12) continue;
            r = std::fmax(-1.0, std::fmin(1.0, r));
            const double base = std::atan2(B, A), half = std::acos(r);
            psi[0] = base + half;
            psi[1] = base - half;
        }

        for (int ip = 0; ip < 2; ++ip) {
            const V3 z5 = ca * a + sa * (std::cos(psi[ip]) * b1 + std::sin(psi[ip]) * b2);
            // 3. the elbow plane's normal n = z4 = -y3, then link 3's frame
            uint8_t flag = flag_cone;
            V3 n;
            if (sa < EPS_SING) {
                n = b1;
                flag |= PLANE;
            } else {
                n = unit(cross(a, z5));
                if (std::sin(alpha_s) < 0) n = -1.0 * n;
            }
            const double cb = std::cos(beta), sb = std::sin(beta);
            const V3 x3 = cb * a - sb * cross(n, a);
            const V3 z3 = cross(n, x3);
            const double s2 = std::hypot(z3.x, z3.y), c2 = z3.z;
            const V3 x4 = c4 * x3 + s4 * z3, z4 = n;
            // 4. q5, q6 do not depend on the shoulder branch
            const double q5 = std::atan2(dot(x4, z6), dot(z4, z6));
            const double c5 = std::cos(q5), s5 = std::sin(q5);
            const V3 x5 = c5 * x4 - s5 * z4;
            const double q6 = std::atan2(dot(z5, x6), dot(x5, x6));

            for (int ib = 0; ib < 2; ++ib) {
                const int slot = 4 * i4 + 2 * ip + ib;
                double q1, q2;
                uint8_t f = flag;
                if (s2 < EPS_SING) {
                    if (ib == 1) continue;
                    q1 = 0.0;
                    q2 = std::atan2(s2, c2);
                    f |= SHOULDER;
                } else {
                    q1 = std::atan2(z3.y, z3.x) + (ib ? M_PI : 0.0);
                    q2 = ib ? -std::atan2(s2, c2) : std::atan2(s2, c2);
                }
                // q3 from link 3's x axis seen in link 2's frame
                auto q3_of = [&](double q1v) {
                    const double c1 = std::cos(q1v), s1 = std::sin(q1v);
                    const double g0 = c1 * x3.x + s1 * x3.y, g1 = -s1 * x3.x + c1 * x3.y;
                    const double g2 = x3.z;
                    const double cq2 = std::cos(q2), sq2 = std::sin(q2);
                    return std::atan2(g1, cq2 * g0 - sq2 * g2);
                };
                double q3 = q3_of(q1);
                if ((f & SHOULDER) && std::isnan(wrap_into(q3, lo[2], hi[2]))) {
                    // only q1 + q3 is fixed (q2 = 0): move the excess from q3 to q1
                    const double q3c = std::fmax(lo[2], std::fmin(hi[2], q3));
                    q1 = q1 + (q3 - q3c);
                    q3 = q3_of(q1);
                }
                const double q[7] = {wrap_into(q1, lo[0], hi[0]), wrap_into(q2, lo[1], hi[1]),
                                     wrap_into(q3, lo[2], hi[2]), q4,
                                     wrap_into(q5, lo[4], hi[4]), wrap_into(q6, lo[5], hi[5]),
                                     q7};
                bool ok = true;
                for (int j = 0; j < 7; ++j) ok = ok && !std::isnan(q[j]);
                if (!ok) continue;
                for (int j = 0; j < 7; ++j) out[slot * 7 + j] = q[j];
                flags[slot] = f;
            }
        }
    }
}

}  // namespace fr3ik
