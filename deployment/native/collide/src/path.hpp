// Lower bound on the clearance along a piecewise-linear joint path, and the arm against itself.
// The same method as path_clearance in aris/kernel/collide.py (the notes are there).
#pragma once

#include "collide.hpp"

namespace acol {

inline double interval_bound(double ca, double cb, double delta) {
    ca = std::min(ca, BIG);
    cb = std::min(cb, BIG);
    const double s = delta > 0.0 ? clip01((ca - cb + delta) / (2.0 * delta)) : 0.5;
    return std::max(ca - delta * s, cb - delta * (1.0 - s));
}

inline double path_clearance(const Chain& H, const Caps& C, const Scene& S, const double* reach,
                             const double* q, int64_t N, bool drawing, double tol, int max_depth,
                             int64_t max_evals, int threads) {
    const int J = H.J, K = C.K;
    std::vector<double> c(size_t(N) * K);
    std::vector<int64_t> arg(size_t(N) * K);
    eval_q(H, C, S, q, N, drawing, true, threads, c.data(), arg.data());
    double m = INF;
    for (double v : c) m = std::min(m, v);
    if (N <= 1) return m;
    std::vector<double> qa(q, q + (N - 1) * J), qb(q + J, q + N * J);
    std::vector<double> ca(c.begin(), c.end() - K), cb(c.begin() + K, c.end());
    double done = INF;
    int64_t evals = 0;
    for (int depth = 0; depth <= max_depth; ++depth) {
        const int64_t I = int64_t(qa.size()) / J;
        std::vector<double> lb(I, INF);
        std::vector<char> split(I, 0);
        int64_t n_split = 0;
        for (int64_t i = 0; i < I; ++i) {
            for (int k = 0; k < K; ++k) {
                double delta = 0.0;
                for (int j = 0; j < J; ++j) delta += std::fabs(qb[i * J + j] - qa[i * J + j]) * reach[j * K + k];
                lb[i] = std::min(lb[i], interval_bound(ca[i * K + k], cb[i * K + k], delta));
            }
            split[i] = lb[i] < m - tol;
            n_split += split[i];
        }
        if (depth == max_depth || evals + n_split > max_evals) {
            std::fill(split.begin(), split.end(), 0);
            n_split = 0;
        }
        for (int64_t i = 0; i < I; ++i)
            if (!split[i]) done = std::min(done, lb[i]);
        if (n_split == 0) break;
        std::vector<double> na, nb, nm, nca, ncb;
        for (int64_t i = 0; i < I; ++i) {
            if (!split[i]) continue;
            for (int j = 0; j < J; ++j) {
                na.push_back(qa[i * J + j]);
                nb.push_back(qb[i * J + j]);
                nm.push_back(0.5 * (qa[i * J + j] + qb[i * J + j]));
            }
            nca.insert(nca.end(), ca.begin() + i * K, ca.begin() + (i + 1) * K);
            ncb.insert(ncb.end(), cb.begin() + i * K, cb.begin() + (i + 1) * K);
        }
        std::vector<double> cm(size_t(n_split) * K);
        std::vector<int64_t> am(size_t(n_split) * K);
        eval_q(H, C, S, nm.data(), n_split, drawing, true, threads, cm.data(), am.data());
        evals += n_split;
        for (double v : cm) m = std::min(m, v);
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

// Smallest clearance over the capsule pairs (P,2) of one configuration's ends p0, p1 (K,3).
inline double self_config(const double* p0, const double* p1, const double* r, const int64_t* pairs,
                          int64_t P, double margin) {
    double best = INF;
    for (int64_t n = 0; n < P; ++n) {
        const int64_t i = pairs[2 * n], j = pairs[2 * n + 1];
        const double d = segment_segment_distance(p0 + 3 * i, p1 + 3 * i, p0 + 3 * j, p1 + 3 * j);
        best = std::min(best, d - r[i] - r[j] - margin);
    }
    return best;
}

}  // namespace acol
