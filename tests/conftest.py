"""Test settings shared by every test file.

One thread for the linear algebra library.  On a busy machine its thread pool can stall a
simple array operation for minutes (measured 2026-09-29: over 2 minutes against 0.27 s), and
none of the code under test needs it.  This applies to test runs only; the package itself sets
no environment variable.
"""
import os

for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_name, "1")
