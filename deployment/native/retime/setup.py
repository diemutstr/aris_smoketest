"""Build of the compiled speed sweeps of the timing step.

    ../.venv/bin/pip install ./native/retime        (from deployment/)

Needs a C++17 compiler; pybind11 is fetched as a build requirement only.  Same pattern as
native/fr3_ik.  `aris.kernel.retime` uses this module when it is importable and otherwise its
numpy fallback, which gives bit-identical numbers (no fused multiply-add here, on purpose).
"""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

setup(
    name="aris_retime_native",
    version="1.0.0",
    packages=["aris_retime_native"],
    ext_modules=[Pybind11Extension("aris_retime_native._retime", ["src/bindings.cpp"],
                                   cxx_std=17, extra_compile_args=["-O2", "-ffp-contract=off"])],
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
