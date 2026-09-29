// Python binding: arrays in, arrays out, loops in C++ with the GIL released.
//
// scene = (box_R, box_c, box_h, box_m, pl_n, pl_off, pl_m, pl_pen_m, pl_paper, cap_a, cap_b, cap_rm,
//          pl_tool_m)
// caps  = (radius, is_pen, is_fixed, is_tool)
// chain = (dh, ex_parent, ex_R, ex_t, cap_frame, cap_a, cap_b)
// The tuples are built by aris.kernel.collide (pack, arm_tables); see that module for meaning.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <stdexcept>

#include "path.hpp"

namespace py = pybind11;
using Arr = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IArr = py::array_t<int64_t, py::array::c_style | py::array::forcecast>;
using BArr = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;

namespace {

struct SceneIn {  // keeps the converted arrays alive while the pointers are in use
    Arr bR, bc, bh, bm, pn, po, pm, ppm, ca, cb, crm, ptm;
    BArr pp;
    std::vector<double> box_soa, cap_soa;
    acol::Scene s;
    explicit SceneIn(const py::tuple& t)
        : bR(t[0].cast<Arr>()), bc(t[1].cast<Arr>()), bh(t[2].cast<Arr>()), bm(t[3].cast<Arr>()),
          pn(t[4].cast<Arr>()), po(t[5].cast<Arr>()), pm(t[6].cast<Arr>()), ppm(t[7].cast<Arr>()),
          ca(t[9].cast<Arr>()), cb(t[10].cast<Arr>()), crm(t[11].cast<Arr>()),
          ptm(t[12].cast<Arr>()), pp(t[8].cast<BArr>()) {
        if (t.size() != 13) throw std::invalid_argument("scene must have 13 arrays");
        s.Mb = int(bm.size());
        s.Mp = int(po.size());
        s.Mc = int(crm.size());
        if (bR.size() != 9 * s.Mb || bc.size() != 3 * s.Mb || bh.size() != 3 * s.Mb ||
            pn.size() != 3 * s.Mp || pm.size() != s.Mp || ppm.size() != s.Mp || pp.size() != s.Mp || ptm.size() != s.Mp ||
            ca.size() != 3 * s.Mc || cb.size() != 3 * s.Mc)
            throw std::invalid_argument("scene arrays have inconsistent sizes");
        s.box_R = bR.data(); s.box_c = bc.data(); s.box_h = bh.data(); s.box_m = bm.data();
        s.pl_n = pn.data(); s.pl_off = po.data(); s.pl_m = pm.data(); s.pl_pen_m = ppm.data();
        s.pl_paper = pp.data();
        s.cap_a = ca.data(); s.cap_b = cb.data(); s.cap_rm = crm.data(); s.pl_tool_m = ptm.data();
        acol::make_soa(s, box_soa, cap_soa);
        s.box_soa = box_soa.data();
        s.cap_soa = cap_soa.data();
    }
};

struct CapsIn {
    Arr r;
    BArr pen, fixed, tool;
    acol::Caps c;
    explicit CapsIn(const py::tuple& t)
        : r(t[0].cast<Arr>()), pen(t[1].cast<BArr>()), fixed(t[2].cast<BArr>()),
          tool(t[3].cast<BArr>()) {
        c.K = int(r.size());
        if (pen.size() != c.K || fixed.size() != c.K || tool.size() != c.K)
            throw std::invalid_argument("caps sizes differ");
        c.r = r.data(); c.is_pen = pen.data(); c.is_fixed = fixed.data(); c.is_tool = tool.data();
    }
};

struct ChainIn {
    Arr dh, eR, et, a, b;
    IArr ep, cf;
    acol::Chain h;
    explicit ChainIn(const py::tuple& t)
        : dh(t[0].cast<Arr>()), eR(t[2].cast<Arr>()), et(t[3].cast<Arr>()), a(t[5].cast<Arr>()),
          b(t[6].cast<Arr>()), ep(t[1].cast<IArr>()), cf(t[4].cast<IArr>()) {
        h.J = int(dh.size() / 3);
        h.E = int(ep.size());
        const py::ssize_t K = cf.size();
        if (eR.size() != 9 * h.E || et.size() != 3 * h.E || a.size() != 3 * K || b.size() != 3 * K)
            throw std::invalid_argument("chain arrays have inconsistent sizes");
        for (py::ssize_t e = 0; e < h.E; ++e)
            if (ep.data()[e] < 0 || ep.data()[e] >= 1 + h.J + e) throw std::invalid_argument("bad ex_parent");
        for (py::ssize_t k = 0; k < K; ++k)
            if (cf.data()[k] < 0 || cf.data()[k] >= h.frames()) throw std::invalid_argument("bad cap_frame");
        h.dh = dh.data(); h.ex_parent = ep.data(); h.ex_R = eR.data(); h.ex_t = et.data();
        h.cap_frame = cf.data(); h.cap_a = a.data(); h.cap_b = b.data();
    }
    int K() const { return int(cf.size()); }
};

py::ssize_t check_q(const Arr& Q, int J) {
    if (Q.ndim() != 2 || Q.shape(1) != J) throw std::invalid_argument("Q must be (N, joints)");
    return Q.shape(0);
}

py::tuple capsule_values(Arr p0, Arr p1, py::tuple caps, py::tuple scene, bool drawing, bool prune,
                         int threads) {
    CapsIn C(caps);
    SceneIn S(scene);
    if (p0.ndim() != 3 || p0.shape(1) != C.c.K || p0.shape(2) != 3 || p1.size() != p0.size())
        throw std::invalid_argument("p0, p1 must be (N, K, 3)");
    const py::ssize_t N = p0.shape(0), K = C.c.K;
    py::array_t<double> val({N, K});
    py::array_t<int64_t> arg({N, K});
    const double *a = p0.data(), *b = p1.data();
    double* v = val.mutable_data();
    int64_t* g = arg.mutable_data();
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            for (int64_t i = i0; i < i1; ++i)
                acol::eval_config(S.s, C.c, a + i * K * 3, b + i * K * 3, drawing, prune, v + i * K,
                                  g + i * K, w);
        });
    }
    return py::make_tuple(val, arg);
}

py::tuple capsule_values_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr Q, bool drawing,
                           bool prune, int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    const py::ssize_t N = check_q(Q, H.h.J), K = C.c.K;
    py::array_t<double> val({N, K});
    py::array_t<int64_t> arg({N, K});
    {
        py::gil_scoped_release release;
        acol::eval_q(H.h, C.c, S.s, Q.data(), N, drawing, prune, threads, val.mutable_data(),
                     arg.mutable_data());
    }
    return py::make_tuple(val, arg);
}

py::array_t<double> clearance_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr Q, bool drawing,
                                int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    const py::ssize_t N = check_q(Q, H.h.J);
    const int K = C.c.K;
    py::array_t<double> out(N);
    double* o = out.mutable_data();
    const double* q = Q.data();
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            std::vector<double> p0(3 * K), p1(3 * K), v(K);
            std::vector<int64_t> g(K);
            for (int64_t i = i0; i < i1; ++i) {
                acol::chain_frames(H.h, q + i * H.h.J, w);
                acol::chain_caps(H.h, K, p0.data(), p1.data(), w);
                acol::eval_config(S.s, C.c, p0.data(), p1.data(), drawing, true, v.data(), g.data(), w);
                double m = acol::INF;
                for (double x : v) m = std::min(m, x);
                o[i] = m;
            }
        });
    }
    return out;
}

py::tuple body_q(py::tuple chain, Arr Q) {
    ChainIn H(chain);
    const py::ssize_t N = check_q(Q, H.h.J), K = H.K();
    py::array_t<double> p0({N, K, py::ssize_t(3)}), p1({N, K, py::ssize_t(3)});
    double *a = p0.mutable_data(), *b = p1.mutable_data();
    acol::Scratch w;
    for (py::ssize_t i = 0; i < N; ++i) {
        acol::chain_frames(H.h, Q.data() + i * H.h.J, w);
        acol::chain_caps(H.h, int(K), a + i * K * 3, b + i * K * 3, w);
    }
    return py::make_tuple(p0, p1);
}

py::array_t<double> self_clearance(Arr p0, Arr p1, Arr r, IArr pairs, double margin, int threads) {
    const py::ssize_t K = r.size();
    if (p0.ndim() != 3 || p0.shape(1) != K || p0.shape(2) != 3 || p1.size() != p0.size())
        throw std::invalid_argument("p0, p1 must be (N, K, 3)");
    const py::ssize_t N = p0.shape(0), P = pairs.size() / 2;
    for (py::ssize_t n = 0; n < 2 * P; ++n)
        if (pairs.data()[n] < 0 || pairs.data()[n] >= K) throw std::invalid_argument("bad pair index");
    py::array_t<double> out(N);
    double* o = out.mutable_data();
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch&) {
            for (int64_t i = i0; i < i1; ++i)
                o[i] = acol::self_config(p0.data() + i * K * 3, p1.data() + i * K * 3, r.data(),
                                         pairs.data(), P, margin);
        });
    }
    return out;
}

py::array_t<double> self_clearance_q(py::tuple chain, Arr r, Arr Q, IArr pairs, double margin, int threads) {
    ChainIn H(chain);
    const int K = H.K();
    if (r.size() != K) throw std::invalid_argument("radius must be (K,)");
    const py::ssize_t N = check_q(Q, H.h.J), P = pairs.size() / 2;
    for (py::ssize_t n = 0; n < 2 * P; ++n)
        if (pairs.data()[n] < 0 || pairs.data()[n] >= K) throw std::invalid_argument("bad pair index");
    py::array_t<double> out(N);
    double* o = out.mutable_data();
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            std::vector<double> p0(3 * K), p1(3 * K);
            for (int64_t i = i0; i < i1; ++i) {
                acol::chain_frames(H.h, Q.data() + i * H.h.J, w);
                acol::chain_caps(H.h, K, p0.data(), p1.data(), w);
                o[i] = acol::self_config(p0.data(), p1.data(), r.data(), pairs.data(), P, margin);
            }
        });
    }
    return out;
}

acol::Refine refine_opts(double tol, int max_depth, int64_t max_evals, bool use_floor, double floor) {
    acol::Refine o;
    o.tol = tol;
    o.max_depth = max_depth;
    o.max_evals = max_evals;
    o.use_floor = use_floor;
    o.floor = floor;
    return o;
}

void check_pairs(const IArr& pairs, int K) {
    for (py::ssize_t n = 0; n < pairs.size(); ++n)
        if (pairs.data()[n] < 0 || pairs.data()[n] >= K) throw std::invalid_argument("bad pair index");
}

double path_clearance_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr reach, Arr q, bool drawing,
                        double tol, int max_depth, int64_t max_evals, int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    if (reach.size() != H.h.J * C.c.K) throw std::invalid_argument("reach must be (joints, K)");
    const py::ssize_t N = check_q(q, H.h.J);
    const acol::Refine o = refine_opts(tol, max_depth, max_evals, false, 0.0);
    py::gil_scoped_release release;
    return acol::path_obstacles(H.h, C.c, S.s, reach.data(), q.data(), N, drawing, o, threads);
}

double path_self_q(py::tuple chain, py::tuple caps, Arr reach, Arr q, IArr pairs, double margin,
                   double tol, int max_depth, int64_t max_evals) {
    ChainIn H(chain);
    CapsIn C(caps);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    if (reach.size() != H.h.J * C.c.K) throw std::invalid_argument("reach must be (joints, K)");
    check_pairs(pairs, C.c.K);
    const py::ssize_t N = check_q(q, H.h.J);
    const acol::Refine o = refine_opts(tol, max_depth, max_evals, false, 0.0);
    py::gil_scoped_release release;
    return acol::path_self(H.h, C.c, reach.data(), pairs.data(), pairs.size() / 2, margin, q.data(),
                           N, o);
}

py::array_t<double> edges_clearance_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr reach,
                                      Arr Qa, Arr Qb, bool drawing, IArr pairs, double margin,
                                      double tol, int max_depth, int64_t max_evals, bool use_floor,
                                      double floor, int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    if (reach.size() != H.h.J * C.c.K) throw std::invalid_argument("reach must be (joints, K)");
    check_pairs(pairs, C.c.K);
    const py::ssize_t E = check_q(Qa, H.h.J);
    if (check_q(Qb, H.h.J) != E) throw std::invalid_argument("Qa and Qb must have the same shape");
    const acol::Refine o = refine_opts(tol, max_depth, max_evals, use_floor, floor);
    py::array_t<double> out(E);
    double* op = out.mutable_data();
    {
        py::gil_scoped_release release;
        acol::edges(H.h, C.c, S.s, reach.data(), Qa.data(), Qb.data(), E, drawing, pairs.data(),
                    pairs.size() / 2, margin, o, threads, op);
    }
    return out;
}

}  // namespace

PYBIND11_MODULE(_collide, m) {
    m.doc() = "Compiled collision check; see aris.kernel.collide.";
    m.def("capsule_values", &capsule_values);
    m.def("capsule_values_q", &capsule_values_q);
    m.def("clearance_q", &clearance_q);
    m.def("body_q", &body_q);
    m.def("self_clearance", &self_clearance);
    m.def("self_clearance_q", &self_clearance_q);
    m.def("path_clearance_q", &path_clearance_q);
    m.def("path_self_q", &path_self_q);
    m.def("edges_clearance_q", &edges_clearance_q);
}
