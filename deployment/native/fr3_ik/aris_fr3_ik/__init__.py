"""Analytic inverse kinematics of the Franka FR3 for a given joint-7 angle, all roots.

`solve(T_base_hand (M,4,4), q7 (M,), q_min (7,), q_max (7,))` -> `(Q (M,8,7), flags (M,8))`.
Poses are of the hand frame (flange turned -45 deg).  Q is NaN where a slot has no answer
inside the limits.  flags: SHOULDER (q2 = 0, only q1 + q3 is fixed), PLANE, CONE (see
src/fr3_ik.hpp).  The answers are not checked by forward kinematics here; `aris.kernel.arm`
does that.
"""
from ._fr3_ik import CONE, N_SOL, PLANE, SHOULDER, solve

__all__ = ["solve", "N_SOL", "SHOULDER", "PLANE", "CONE"]
