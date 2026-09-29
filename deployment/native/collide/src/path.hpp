// Lower bound on the clearance along a piecewise-linear joint path, the arm against itself,
// and batches of straight edges.  The same method as aris/kernel/collide.py (notes there).
#pragma once

#include "collide.hpp"

namespace acol {

inline double interval_bound(double ca, double cb, double delta) {
    ca = std::min(ca, BIG);
    cb = std::min(cb, BIG);
    const double s = delta > 0.0 ? clip01((ca - cb + delta) / (2.0 * delta)) : 0.5;
    return std::max(ca - delta * s, cb - delta * (1.0 - s));
}

struct Refine {
    double tol = 5e-4;
    int max_depth = 30;
    int64_t max_evals = 200000;
    bool use_floor = false;  // stop once the bound is proven >= floor, or a sample is below 0
    double floor = 0.0;
    double cap = 0.25;       // clearance above this is never refined for
    double span = 0.05;      // per-item values exact up to (sample minimum + span); see Mode
    bool prune = true, groups = true;
    Mode mode() const {
        Mode m;
        m.prune = prune;
        m.groups = groups;
        m.span = span;
        return m;
    }
};

// The halving loop over a path q (N,J).  `eval(Q, n, vals)` fills vals (n, I) with the
// clearance of each of I items (capsules, or capsule pairs); `reach` (J, I) bounds how fast
// each item's clearance can change per radian of each joint.
template <class Eval>
double refine(int J, int I, const double* reach, const double* q, int64_t N, Eval&& eval,
              const Refine& o) {
    std::vector<double> c(size_t(N) * I);
    eval(q, N, c.data());
    double m = INF;
    for (double v : c) m = std::min(m, v);
    if (N <= 1 || m == INF || (o.use_floor && m < 0.0)) return m;  // INF: nothing to hit
    std::vector<double> qa(q, q + (N - 1) * J), qb(q + J, q + N * J);
    std::vector<double> ca(c.begin(), c.end() - I), cb(c.begin() + I, c.end());
    double done = INF;
    int64_t evals = 0;
    for (int depth = 0; depth <= o.max_depth; ++depth) {
        const int64_t n_iv = int64_t(qa.size()) / J;
        std::vector<double> lb(n_iv, INF);
        std::vector<char> split(n_iv, 0);
        int64_t n_split = 0;
        double now = done;
        for (int64_t i = 0; i < n_iv; ++i) {
            for (int k = 0; k < I; ++k) {
                double delta = 0.0;
                for (int j = 0; j < J; ++j) delta += std::fabs(qb[i * J + j] - qa[i * J + j]) * reach[j * I + k];
                lb[i] = std::min(lb[i], interval_bound(ca[i * I + k], cb[i * I + k], delta));
            }
            split[i] = lb[i] < std::min(m, o.cap) - o.tol;
            n_split += split[i];
            now = std::min(now, lb[i]);
        }
        if (o.use_floor && now >= o.floor) return now;
        if (depth == o.max_depth || evals + n_split > o.max_evals) {
            std::fill(split.begin(), split.end(), 0);
            n_split = 0;
        }
        for (int64_t i = 0; i < n_iv; ++i)
            if (!split[i]) done = std::min(done, lb[i]);
        if (n_split == 0) break;
        std::vector<double> na, nb, nm, nca, ncb;
        for (int64_t i = 0; i < n_iv; ++i) {
            if (!split[i]) continue;
            for (int j = 0; j < J; ++j) {
                na.push_back(qa[i * J + j]);
                nb.push_back(qb[i * J + j]);
                nm.push_back(0.5 * (qa[i * J + j] + qb[i * J + j]));
            }
            nca.insert(nca.end(), ca.begin() + i * I, ca.begin() + (i + 1) * I);
            ncb.insert(ncb.end(), cb.begin() + i * I, cb.begin() + (i + 1) * I);
        }
        std::vector<double> cm(size_t(n_split) * I);
        eval(nm.data(), n_split, cm.data());
        evals += n_split;
        for (double v : cm) m = std::min(m, v);
        if (o.use_floor && m < 0.0) return m;
        // first halves [a, mid], then second halves [mid, b]
        qa = na;
        qa.insert(qa.end(), nm.begin(), nm.end());
        qb = nm;
        qb.insert(qb.end(), nb.begin(), nb.end());
        ca = nca;
        ca.insert(ca.end(), cm.begin(), cm.end());
        cb = cm;
        cb.insert(cb.end(), ncb.begin(), ncb.end());
    }
    return std::min(done, m);
}

// The arm against itself: the pairs (P,2) grouped by the body groups of their two capsules.
struct SelfPlan {
    std::vector<int64_t> start, idx;  // group pair n holds pairs idx[start[n] .. start[n+1])
    std::vector<int> g, h;
    SelfPlan(const Caps& C, const int64_t* pairs, int64_t P) {
        std::vector<int> grp(C.K, 0);
        for (int gg = 0; gg < C.NG; ++gg)
            for (int64_t n = C.bg_start[gg]; n < C.bg_start[gg + 1]; ++n) grp[C.bg_members[n]] = gg;
        std::vector<int64_t> key(P);
        for (int64_t n = 0; n < P; ++n) {
            int a = grp[pairs[2 * n]], b = grp[pairs[2 * n + 1]];
            key[n] = int64_t(std::min(a, b)) * C.NG + std::max(a, b);
        }
        idx.resize(P);
        for (int64_t n = 0; n < P; ++n) idx[n] = n;
        std::stable_sort(idx.begin(), idx.end(), [&](int64_t x, int64_t y) { return key[x] < key[y]; });
        for (int64_t n = 0; n < P; ++n)
            if (n == 0 || key[idx[n]] != key[idx[n - 1]]) {
                start.push_back(n);
                g.push_back(int(key[idx[n]] / C.NG));
                h.push_back(int(key[idx[n]] % C.NG));
            }
        start.push_back(P);
    }
};

// Clearance of each capsule pair of one configuration, into out (P): min(true, m + span) with m
// the smallest over the pairs (exact definitions, as in eval_config).  Returns m.
inline double self_eval(const Caps& C, const double* p0, const double* p1, const int64_t* pairs,
                        int64_t P, double margin, const SelfPlan& plan, const Mode& md,
                        double* out, Scratch& w) {
    const double span = md.span;
    double best = INF;
    int64_t n_exact = 0;
    const double* r = C.r;
    auto exact = [&](int64_t n) {
        const int64_t i = pairs[2 * n], j = pairs[2 * n + 1];
        out[n] = segment_segment_distance(p0 + 3 * i, p1 + 3 * i, p0 + 3 * j, p1 + 3 * j) - r[i] -
                 r[j] - margin;
        ++n_exact;
        best = std::min(best, out[n]);
    };
    for (int64_t n = 0; n < P; ++n) out[n] = INF;
    w.mid.resize(3 * size_t(C.K));
    w.half.resize(C.K);
    for (int k = 0; k < C.K; ++k) {
        double u[3];
        for (int i = 0; i < 3; ++i) {
            w.mid[3 * k + i] = 0.5 * (p0[3 * k + i] + p1[3 * k + i]);
            u[i] = p1[3 * k + i] - p0[3 * k + i];
        }
        w.half[k] = 0.5 * std::sqrt(dot(u, u));
    }
    // midpoint-to-midpoint: an upper bound on the pair (plus both half lengths)
    auto mid_ub = [&](int64_t n) {
        const int64_t i = pairs[2 * n], j = pairs[2 * n + 1];
        double d[3];
        for (int q = 0; q < 3; ++q) d[q] = w.mid[3 * i + q] - w.mid[3 * j + q];
        return std::sqrt(dot(d, d)) - r[i] - r[j] - margin;
    };
    auto slack = [&](int64_t n) { return w.half[pairs[2 * n]] + w.half[pairs[2 * n + 1]]; };
    if (!md.prune) {
        for (int64_t n = 0; n < P; ++n) exact(n);
    } else {
        const int NGP = int(plan.g.size());
        std::vector<double>& gl = w.gl;
        gl.assign(NGP, -INF);
        std::vector<int>& order = w.order;
        order.clear();
        if (md.groups) {
            // group pairs within one group first (always checked), then the others by bound
            group_spheres(C, p0, p1, false, w);
            for (int q = 0; q < NGP; ++q) {
                if (plan.g[q] == plan.h[q]) { order.push_back(q); continue; }
                const double *a = w.sph.data() + 4 * plan.g[q], *b = w.sph.data() + 4 * plan.h[q];
                double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
                gl[q] = std::sqrt(dot(d, d)) - a[3] - b[3] - margin;
            }
            for (int q = 0; q < NGP; ++q)
                if (plan.g[q] != plan.h[q]) order.push_back(q);
        } else {
            for (int q = 0; q < NGP; ++q) order.push_back(q);
        }
        std::vector<double> ub;
        if (!md.groups) {  // one pass for best, as without groups for the obstacles
            ub.resize(P);
            for (int64_t n = 0; n < P; ++n) best = std::min(best, ub[n] = mid_ub(n));
        }
        for (int q : order) {
            if (gl[q] > best + span) continue;
            if (md.groups)
                for (int64_t t = plan.start[q]; t < plan.start[q + 1]; ++t)
                    best = std::min(best, mid_ub(plan.idx[t]));
            for (int64_t t = plan.start[q]; t < plan.start[q + 1]; ++t) {
                const int64_t n = plan.idx[t];
                const double u = md.groups ? mid_ub(n) : ub[n];
                if (!(u - slack(n) > best + span)) exact(n);
            }
        }
    }
    double m = INF;
    for (int64_t n = 0; n < P; ++n) m = std::min(m, out[n]);
    if (std::isfinite(m))
        for (int64_t n = 0; n < P; ++n) out[n] = std::min(out[n], m + span);
    if (md.count) count_add(md.count, n_exact);
    return m;
}

// Bound against the obstacles along the path q (N,J); reach is (J,K).
inline double path_obstacles(const Chain& H, const Caps& C, const Scene& S, const double* reach,
                             const double* q, int64_t N, bool drawing, const Refine& o, int threads) {
    if (S.Mb + S.Mp + S.Mc == 0) return INF;
    std::vector<int64_t> arg;
    const Mode md = o.mode();
    auto eval = [&](const double* Q, int64_t n, double* vals) {
        arg.resize(size_t(n) * C.K);
        eval_q(H, C, S, Q, n, drawing, md, threads, vals, arg.data());
    };
    return refine(H.J, C.K, reach, q, N, eval, o);
}

// Bound of the arm against itself along the path q; pair reach = reach of both capsules.
inline double path_self(const Chain& H, const Caps& C, const double* reach, const int64_t* pairs,
                        int64_t P, double margin, const SelfPlan& plan, const double* q, int64_t N,
                        const Refine& o) {
    std::vector<double> rp(size_t(H.J) * P);
    for (int j = 0; j < H.J; ++j)
        for (int64_t n = 0; n < P; ++n)
            rp[j * P + n] = reach[j * C.K + pairs[2 * n]] + reach[j * C.K + pairs[2 * n + 1]];
    Scratch w;
    std::vector<double> p0(3 * C.K), p1(3 * C.K);
    const Mode md = o.mode();
    auto eval = [&](const double* Q, int64_t n, double* vals) {
        for (int64_t i = 0; i < n; ++i) {
            chain_frames(H, Q + i * H.J, w);
            chain_caps(H, C.K, p0.data(), p1.data(), w);
            self_eval(C, p0.data(), p1.data(), pairs, P, margin, plan, md, vals + i * P, w);
        }
    };
    return refine(H.J, int(P), rp.data(), q, N, eval, o);
}

// One bound per straight edge Qa[e] -> Qb[e]: obstacles, and the arm itself if P > 0.
inline void edges(const Chain& H, const Caps& C, const Scene& S, const double* reach,
                  const double* Qa, const double* Qb, int64_t E, bool drawing, const int64_t* pairs,
                  int64_t P, double margin, const Refine& o, int threads, double* out) {
    const SelfPlan plan(C, pairs, P);
    parallel_for(E, threads, [&](int64_t e0, int64_t e1, Scratch&) {
        std::vector<double> q(2 * size_t(H.J));
        for (int64_t e = e0; e < e1; ++e) {
            std::copy(Qa + e * H.J, Qa + (e + 1) * H.J, q.begin());
            std::copy(Qb + e * H.J, Qb + (e + 1) * H.J, q.begin() + H.J);
            double b = path_obstacles(H, C, S, reach, q.data(), 2, drawing, o, 1);
            if (P > 0 && !(o.use_floor && b < 0.0))
                b = std::min(b, path_self(H, C, reach, pairs, P, margin, plan, q.data(), 2, o));
            out[e] = b;
        }
    });
}

}  // namespace acol
