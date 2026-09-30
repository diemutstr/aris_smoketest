// The forward and backward speed sweeps of aris.kernel.retime (see sweeps_numpy there, which
// this reproduces operation for operation: same products, same sums, same order).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <stdexcept>

namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;

// x_ceiling (n+1,), p and q (n, m): x[i+1] <= p[i,j] * x[i] + q[i,j] for every j, and backward.
static Array sweeps(Array x_ceiling, Array p, Array q) {
    if (x_ceiling.ndim() != 1 || p.ndim() != 2 || q.ndim() != 2 ||
        p.shape(0) != x_ceiling.shape(0) - 1 || q.shape(0) != p.shape(0) || q.shape(1) != p.shape(1))
        throw std::invalid_argument("sweeps: need x (n+1,), p and q (n, m)");
    const py::ssize_t n = p.shape(0), m = p.shape(1);
    Array out(x_ceiling.shape(0));
    auto x = out.mutable_unchecked<1>();
    auto c = x_ceiling.unchecked<1>();
    auto P = p.unchecked<2>();
    auto Q = q.unchecked<2>();
    for (py::ssize_t i = 0; i <= n; ++i) x(i) = c(i);
    x(0) = 0.0;
    x(n) = 0.0;
    for (py::ssize_t i = 0; i < n; ++i) {
        double best = x(i + 1);
        for (py::ssize_t j = 0; j < m; ++j) {
            double v = P(i, j) * x(i);
            v = v + Q(i, j);
            if (v < best) best = v;
        }
        x(i + 1) = best;
    }
    for (py::ssize_t i = n - 1; i >= 0; --i) {
        double best = x(i);
        for (py::ssize_t j = 0; j < m; ++j) {
            double v = P(i, j) * x(i + 1);
            v = v + Q(i, j);
            if (v < best) best = v;
        }
        x(i) = best;
    }
    return out;
}

PYBIND11_MODULE(_retime, mod) {
    mod.doc() = "Speed sweeps of the timing step.";
    mod.def("sweeps", &sweeps, py::arg("x_ceiling"), py::arg("p"), py::arg("q"));
}
