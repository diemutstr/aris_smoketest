// Python binding: arrays in, arrays out, loops in C++ with the GIL released.
//
// scene = (box_R, box_c, box_h, box_m, pl_n, pl_off, pl_m, pl_pen_m, pl_paper, cap_a, cap_b, cap_rm,
//          pl_tool_m, og_start, og_members, og_a, og_b, og_r, og_margin)
// caps  = (radius, is_pen, is_fixed, is_tool, bg_start, bg_members)
// mode  = (prune, groups, span)
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
    Arr bR, bc, bh, bm, pn, po, pm, ppm, ca, cb, crm, ptm, ga, gb, gr, gm;
    IArr gs, gmem;
    Arr fo, fc, fm, ft, fb, fmin;
    IArr fd, foff;
    py::array_t<float, py::array::c_style | py::array::forcecast> fdata;
    BArr pp;
    std::vector<double> box_soa, cap_soa, og_soa;
    acol::Scene s;
    explicit SceneIn(const py::tuple& t)
        : bR(t[0].cast<Arr>()), bc(t[1].cast<Arr>()), bh(t[2].cast<Arr>()), bm(t[3].cast<Arr>()),
          pn(t[4].cast<Arr>()), po(t[5].cast<Arr>()), pm(t[6].cast<Arr>()), ppm(t[7].cast<Arr>()),
          ca(t[9].cast<Arr>()), cb(t[10].cast<Arr>()), crm(t[11].cast<Arr>()),
          ptm(t[12].cast<Arr>()), ga(t[15].cast<Arr>()), gb(t[16].cast<Arr>()),
          gr(t[17].cast<Arr>()), gm(t[18].cast<Arr>()), gs(t[13].cast<IArr>()),
          gmem(t[14].cast<IArr>()), pp(t[8].cast<BArr>()) {
        if (t.size() != 28) throw std::invalid_argument("scene must have 28 arrays");
        fo = t[19].cast<Arr>(); fc = t[20].cast<Arr>(); fd = t[21].cast<IArr>();
        fm = t[22].cast<Arr>(); foff = t[23].cast<IArr>();
        fdata = t[24].cast<py::array_t<float, py::array::c_style | py::array::forcecast>>();
        ft = t[25].cast<Arr>(); fb = t[26].cast<Arr>(); fmin = t[27].cast<Arr>();
        s.F = int(fc.size());
        s.NL = int(ft.size());
        if (fo.size() != 3 * s.F || fd.size() != 3 * s.F || fm.size() != s.F ||
            foff.size() != s.F + 1 || fb.size() != 6 * s.F * s.NL || fmin.size() != s.F ||
            foff.data()[s.F] != fdata.size())
            throw std::invalid_argument("field arrays have inconsistent sizes");
        for (int f = 0; f < s.F; ++f) {
            const int64_t* n = fd.data() + 3 * f;
            if (n[0] < 1 || n[1] < 1 || n[2] < 1 || fc.data()[f] <= 0.0 ||
                foff.data()[f + 1] - foff.data()[f] != n[0] * n[1] * n[2])
                throw std::invalid_argument("bad field grid");
        }
        s.fl_origin = fo.data(); s.fl_cell = fc.data(); s.fl_dims = fd.data(); s.fl_margin = fm.data();
        s.fl_off = foff.data(); s.fl_data = fdata.data(); s.fl_tau = ft.data(); s.fl_box = fb.data();
        s.fl_min = fmin.data();
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
        s.G = int(gr.size());
        if (gs.size() != s.G + 1 || ga.size() != 3 * s.G || gb.size() != 3 * s.G || gm.size() != s.G ||
            gmem.size() != s.Mb + s.Mc || gs.data()[s.G] != s.Mb + s.Mc)
            throw std::invalid_argument("obstacle groups have inconsistent sizes");
        for (py::ssize_t n = 0; n < gmem.size(); ++n) {
            const int64_t o = gmem.data()[n];
            if (o < 0 || o >= s.Mb + s.Mp + s.Mc || (o >= s.Mb && o < s.Mb + s.Mp))
                throw std::invalid_argument("bad obstacle group member");
        }
        s.og_start = gs.data(); s.og_members = gmem.data();
        s.og_a = ga.data(); s.og_b = gb.data(); s.og_r = gr.data(); s.og_margin = gm.data();
        acol::make_soa(s, box_soa, cap_soa, og_soa);
        s.og_soa = og_soa.data();
        s.box_soa = box_soa.data();
        s.cap_soa = cap_soa.data();
    }
};

struct CapsIn {
    Arr r;
    BArr pen, fixed, tool, ex;
    IArr bs, bm;
    acol::Caps c;
    explicit CapsIn(const py::tuple& t)
        : r(t[0].cast<Arr>()), pen(t[1].cast<BArr>()), fixed(t[2].cast<BArr>()),
          tool(t[3].cast<BArr>()), bs(t[4].cast<IArr>()), bm(t[5].cast<IArr>()) {
        c.K = int(r.size());
        if (pen.size() != c.K || fixed.size() != c.K || tool.size() != c.K)
            throw std::invalid_argument("caps sizes differ");
        c.NG = int(bs.size()) - 1;
        if (c.NG < 0 || bm.size() != c.K || bs.data()[c.NG] != c.K)
            throw std::invalid_argument("body groups must cover every capsule once");
        std::vector<char> seen(c.K, 0);
        for (py::ssize_t n = 0; n < bm.size(); ++n) {
            const int64_t k = bm.data()[n];
            if (k < 0 || k >= c.K || seen[k]) throw std::invalid_argument("bad body group member");
            seen[k] = 1;
        }
        c.r = r.data(); c.is_pen = pen.data(); c.is_fixed = fixed.data(); c.is_tool = tool.data();
        c.bg_start = bs.data(); c.bg_members = bm.data();
        if (t.size() > 6) {  // the boxes' exemptions, (K, Mb); checked against the scene later
            ex = t[6].cast<BArr>();
            c.exempt = ex.data();
        }
    }
};

// The exemption mask must be (K, Mb) for this scene.
void check_exempt(const CapsIn& C, const SceneIn& S) {
    if (C.c.exempt && C.ex.size() != py::ssize_t(C.c.K) * S.s.Mb)
        throw std::invalid_argument("the exemption mask must be (K, number of boxes)");
}

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

acol::Mode to_mode(const py::tuple& t) {
    acol::Mode m;
    m.prune = t[0].cast<bool>();
    m.groups = t[1].cast<bool>();
    m.span = t[2].cast<double>();
    return m;
}

acol::Refine refine_opts(double tol, int max_depth, int64_t max_evals, bool use_floor, double floor,
                         double cap, const py::tuple& mode) {
    acol::Refine o;
    o.tol = tol;
    o.max_depth = max_depth;
    o.max_evals = max_evals;
    o.use_floor = use_floor;
    o.floor = floor;
    o.cap = cap;
    const acol::Mode m = to_mode(mode);
    o.prune = m.prune;
    o.groups = m.groups;
    o.span = m.span;
    return o;
}

void check_pairs(const IArr& pairs, int K) {
    for (py::ssize_t n = 0; n < pairs.size(); ++n)
        if (pairs.data()[n] < 0 || pairs.data()[n] >= K) throw std::invalid_argument("bad pair index");
}

void check_body(const Arr& p0, const Arr& p1, int K) {
    if (p0.ndim() != 3 || p0.shape(1) != K || p0.shape(2) != 3 || p1.size() != p0.size())
        throw std::invalid_argument("p0, p1 must be (N, K, 3)");
}

// Per-capsule values of given capsule ends (N,K,3); returns (val, arg, exact pair count).
py::tuple capsule_values(Arr p0, Arr p1, py::tuple caps, py::tuple scene, bool drawing, py::tuple mode,
                         int threads) {
    CapsIn C(caps);
    SceneIn S(scene);
    check_exempt(C, S);
    check_body(p0, p1, C.c.K);
    const py::ssize_t N = p0.shape(0), K = C.c.K;
    py::array_t<double> val({N, K});
    py::array_t<int64_t> arg({N, K});
    const double *a = p0.data(), *b = p1.data();
    double* v = val.mutable_data();
    int64_t* g = arg.mutable_data();
    int64_t count = 0;
    acol::Mode md = to_mode(mode);
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            acol::Mode mt = md;
            int64_t cnt = 0;
            mt.count = &cnt;
            for (int64_t i = i0; i < i1; ++i)
                acol::eval_config(S.s, C.c, a + i * K * 3, b + i * K * 3, drawing, mt, v + i * K,
                                  g + i * K, w);
            acol::count_add(&count, cnt);
        });
    }
    return py::make_tuple(val, arg, count);
}

py::tuple capsule_values_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr Q, bool drawing,
                           py::tuple mode, int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    check_exempt(C, S);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    const py::ssize_t N = check_q(Q, H.h.J), K = C.c.K;
    py::array_t<double> val({N, K});
    py::array_t<int64_t> arg({N, K});
    int64_t count = 0;
    acol::Mode md = to_mode(mode);
    md.count = &count;
    {
        py::gil_scoped_release release;
        acol::eval_q(H.h, C.c, S.s, Q.data(), N, drawing, md, threads, val.mutable_data(),
                     arg.mutable_data());
    }
    return py::make_tuple(val, arg, count);
}

py::array_t<double> clearance_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr Q, bool drawing,
                                py::tuple mode, int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    check_exempt(C, S);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    const py::ssize_t N = check_q(Q, H.h.J);
    const int K = C.c.K;
    py::array_t<double> out(N);
    double* o = out.mutable_data();
    const double* q = Q.data();
    const acol::Mode md = to_mode(mode);
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            std::vector<double> p0(3 * K), p1(3 * K), v(K);
            std::vector<int64_t> g(K);
            for (int64_t i = i0; i < i1; ++i) {
                acol::chain_frames(H.h, q + i * H.h.J, w);
                acol::chain_caps(H.h, K, p0.data(), p1.data(), w);
                acol::eval_config(S.s, C.c, p0.data(), p1.data(), drawing, md, v.data(), g.data(), w);
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

// The arm against itself.  With values=True returns (N,P) per-pair values, else (N,) minima;
// and the exact pair count.
py::tuple self_values(Arr p0, Arr p1, py::tuple caps, IArr pairs, double margin, py::tuple mode,
                      bool values, int threads) {
    CapsIn C(caps);
    check_body(p0, p1, C.c.K);
    check_pairs(pairs, C.c.K);
    const py::ssize_t N = p0.shape(0), K = C.c.K, P = pairs.size() / 2;
    py::array_t<double> out = values ? py::array_t<double>({N, P}) : py::array_t<double>(N);
    double* o = out.mutable_data();
    int64_t count = 0;
    const acol::Mode md = to_mode(mode);
    const acol::SelfPlan plan(C.c, pairs.data(), P);
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            std::vector<double> buf(P);
            acol::Mode mt = md;
            int64_t cnt = 0;
            mt.count = &cnt;
            for (int64_t i = i0; i < i1; ++i) {
                double* dst = values ? o + i * P : buf.data();
                const double m = acol::self_eval(C.c, p0.data() + i * K * 3, p1.data() + i * K * 3,
                                                 pairs.data(), P, margin, plan, mt, dst, w);
                if (!values) o[i] = m;
            }
            acol::count_add(&count, cnt);
        });
    }
    return py::make_tuple(out, count);
}

py::array_t<double> self_clearance_q(py::tuple chain, py::tuple caps, Arr Q, IArr pairs, double margin,
                                     py::tuple mode, int threads) {
    ChainIn H(chain);
    CapsIn C(caps);
    const int K = H.K();
    if (C.c.K != K) throw std::invalid_argument("chain and caps disagree on K");
    check_pairs(pairs, K);
    const py::ssize_t N = check_q(Q, H.h.J), P = pairs.size() / 2;
    py::array_t<double> out(N);
    double* o = out.mutable_data();
    const acol::Mode md = to_mode(mode);
    const acol::SelfPlan plan(C.c, pairs.data(), P);
    {
        py::gil_scoped_release release;
        acol::parallel_for(N, threads, [&](int64_t i0, int64_t i1, acol::Scratch& w) {
            std::vector<double> p0(3 * K), p1(3 * K), buf(P);
            for (int64_t i = i0; i < i1; ++i) {
                acol::chain_frames(H.h, Q.data() + i * H.h.J, w);
                acol::chain_caps(H.h, K, p0.data(), p1.data(), w);
                o[i] = acol::self_eval(C.c, p0.data(), p1.data(), pairs.data(), P, margin, plan, md,
                                       buf.data(), w);
            }
        });
    }
    return out;
}

double path_clearance_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr reach, Arr q, bool drawing,
                        double tol, int max_depth, int64_t max_evals, int threads, double cap,
                        py::tuple mode) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    check_exempt(C, S);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    if (reach.size() != H.h.J * C.c.K) throw std::invalid_argument("reach must be (joints, K)");
    const py::ssize_t N = check_q(q, H.h.J);
    const acol::Refine o = refine_opts(tol, max_depth, max_evals, false, 0.0, cap, mode);
    py::gil_scoped_release release;
    return acol::path_obstacles(H.h, C.c, S.s, reach.data(), q.data(), N, drawing, o, threads);
}

double path_self_q(py::tuple chain, py::tuple caps, Arr reach, Arr q, IArr pairs, double margin,
                   double tol, int max_depth, int64_t max_evals, double cap, py::tuple mode) {
    ChainIn H(chain);
    CapsIn C(caps);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    if (reach.size() != H.h.J * C.c.K) throw std::invalid_argument("reach must be (joints, K)");
    check_pairs(pairs, C.c.K);
    const py::ssize_t N = check_q(q, H.h.J);
    const acol::Refine o = refine_opts(tol, max_depth, max_evals, false, 0.0, cap, mode);
    py::gil_scoped_release release;
    const acol::SelfPlan plan(C.c, pairs.data(), pairs.size() / 2);
    return acol::path_self(H.h, C.c, reach.data(), pairs.data(), pairs.size() / 2, margin, plan,
                           q.data(), N, o);
}

py::array_t<double> edges_clearance_q(py::tuple chain, py::tuple caps, py::tuple scene, Arr reach,
                                      Arr Qa, Arr Qb, bool drawing, IArr pairs, double margin,
                                      double tol, int max_depth, int64_t max_evals, bool use_floor,
                                      double floor, int threads, double cap, py::tuple mode) {
    ChainIn H(chain);
    CapsIn C(caps);
    SceneIn S(scene);
    check_exempt(C, S);
    if (H.K() != C.c.K) throw std::invalid_argument("chain and caps disagree on K");
    if (reach.size() != H.h.J * C.c.K) throw std::invalid_argument("reach must be (joints, K)");
    check_pairs(pairs, C.c.K);
    const py::ssize_t E = check_q(Qa, H.h.J);
    if (check_q(Qb, H.h.J) != E) throw std::invalid_argument("Qa and Qb must have the same shape");
    const acol::Refine o = refine_opts(tol, max_depth, max_evals, use_floor, floor, cap, mode);
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
    m.def("self_values", &self_values);
    m.def("self_clearance_q", &self_clearance_q);
    m.def("path_clearance_q", &path_clearance_q);
    m.def("path_self_q", &path_self_q);
    m.def("edges_clearance_q", &edges_clearance_q);
}
