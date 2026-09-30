"""Build of the compiled collision check.

    ../.venv/bin/pip install ./native/collide        (from deployment/)

Needs a C++17 compiler; pybind11 is fetched as a build requirement only.  Same pattern as
native/fr3_ik.  `aris.kernel.collide` uses this module when it is importable.
"""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

setup(
    name="aris_collide_native",
    version="1.0.0",
    packages=["aris_collide_native"],
    ext_modules=[Pybind11Extension("aris_collide_native._collide", ["src/bindings.cpp"],
                                   include_dirs=["src"], cxx_std=17,
                                   extra_compile_args=["-O3", "-fno-math-errno", "-fno-trapping-math"])],
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
