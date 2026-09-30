// Python binding: arrays in, arrays out, one C++ loop over the batch with the GIL released.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <stdexcept>

#include "fr3_ik.hpp"

namespace py = pybind11;
using Arr = py::array_t<double, py::array::c_style | py::array::forcecast>;

static py::tuple solve(Arr T, Arr q7, Arr q_min, Arr q_max) {
    if (T.ndim() != 3 || T.shape(1) != 4 || T.shape(2) != 4)
        throw std::invalid_argument("T_base_hand must be (M,4,4)");
    const py::ssize_t M = T.shape(0);
    if (q7.ndim() != 1 || q7.shape(0) != M) throw std::invalid_argument("q7 must be (M,)");
    if (q_min.size() != 7 || q_max.size() != 7)
        throw std::invalid_argument("q_min and q_max must have 7 entries");
    py::array_t<double> Q({M, py::ssize_t(fr3ik::N_SOL), py::ssize_t(7)});
    py::array_t<uint8_t> F({M, py::ssize_t(fr3ik::N_SOL)});
    const double *Tp = T.data(), *q7p = q7.data(), *lo = q_min.data(), *hi = q_max.data();
    double* Qp = Q.mutable_data();
    uint8_t* Fp = F.mutable_data();
    {
        py::gil_scoped_release release;
        for (py::ssize_t i = 0; i < M; ++i)
            fr3ik::solve(Tp + 16 * i, q7p[i], lo, hi, Qp + i * fr3ik::N_SOL * 7,
                         Fp + i * fr3ik::N_SOL);
    }
    return py::make_tuple(Q, F);
}

PYBIND11_MODULE(_fr3_ik, m) {
    m.doc() = "Analytic FR3 inverse kinematics for a given q7, all roots (see fr3_ik.hpp)";
    m.def("solve", &solve, py::arg("T_base_hand"), py::arg("q7"), py::arg("q_min"),
          py::arg("q_max"),
          "(M,4,4) hand poses, (M,) q7, (7,) limits -> (Q (M,8,7) NaN where empty, "
          "flags (M,8) uint8: 1 shoulder, 2 plane, 4 cone)");
    m.attr("N_SOL") = fr3ik::N_SOL;
    m.attr("SHOULDER") = fr3ik::SHOULDER;
    m.attr("PLANE") = fr3ik::PLANE;
    m.attr("CONE") = fr3ik::CONE;
}
