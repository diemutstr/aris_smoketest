"""Build of the FR3 analytic IK extension.

    ../.venv/bin/pip install ./native/fr3_ik        (from deployment/)

Needs a C++17 compiler; pybind11 is fetched as a build requirement only.
"""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

setup(
    name="aris_fr3_ik",
    version="1.0.0",
    packages=["aris_fr3_ik"],
    ext_modules=[Pybind11Extension("aris_fr3_ik._fr3_ik", ["src/bindings.cpp"],
                                   include_dirs=["src"], cxx_std=17,
                                   extra_compile_args=["-O3"])],
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
