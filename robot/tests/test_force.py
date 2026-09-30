"""The pen force on synthetic traces (no recorded force log exists on this machine)."""
import numpy as np
import pytest

from aris.types import Refusal
from aris_robot.force import (Contact, ForceSettings, Guard, Servo, Tare, arc_length,
                              intensity_to_force, normal_force, profile)

S = ForceSettings()
RATE = 250.0                                  # the controller's status rate


def test_intensity_to_force_in_nine_levels():
    assert intensity_to_force(0.0, S) == pytest.approx(0.7)
    assert intensity_to_force(1.0, S) == pytest.approx(1.0)
    assert intensity_to_force(2.0, S) == pytest.approx(1.0)      # clamped
    levels = {round(intensity_to_force(x, S), 9) for x in np.linspace(0, 1, 101)}
    assert len(levels) == 9
    assert intensity_to_force(0.5, S) == pytest.approx(0.85)


def test_site_block_is_read_and_checked():
    s = ForceSettings.from_site({"band_n": [0.6, 1.0], "cap_n": 2.2, "unknown": 1})
    assert s.band_n == (0.6, 1.0) and s.cap_n == 2.2
    with pytest.raises(ValueError):
        ForceSettings.from_site({"band_n": [0.7, 1.0], "cap_n": 0.9})


def test_normal_force_sign():
    n = np.array([0.0, 0.0, -1.0])            # an inverted arm: paper normal is base -z
    assert normal_force([0, 0, -2.0], n) == pytest.approx(2.0)   # paper pushes the pen up
    assert normal_force([0, 0, -2.0], n, sign=-1.0) == pytest.approx(-2.0)


def test_draw_ramps_in_over_the_first_millimetres():
    t = np.linspace(0.0, 1.0, 11)
    tips = np.column_stack([0.02 * t, np.zeros(11), np.zeros(11)])  # 20 mm/s
    f = profile("draw", t, arc_length(tips), 1.0, S)
    assert f(0.0) == 0.0
    assert f(0.05) == pytest.approx(0.5)      # 1 mm of 2
    assert f(0.1) == pytest.approx(1.0) and f(0.9) == pytest.approx(1.0)


def test_lower_is_zero_and_lift_ramps_out():
    t = np.array([0.0, 1.0])
    assert np.all(profile("lower", t, None, 1.0, S)(np.linspace(0, 1, 5)) == 0.0)
    lift = profile("lift", t, None, 1.0, S, f_start=0.9)
    assert lift(0.0) == pytest.approx(0.9) and lift(0.1) == pytest.approx(0.45)
    assert lift(0.2) == 0.0 and lift(1.0) == 0.0


def test_tare_takes_the_air_zero_and_refuses_nonsense():
    rng = np.random.default_rng(1)
    t = Tare(S)
    for x in 2.3 + 0.05 * rng.standard_normal(50):   # a 2.3 N air reading, as on arm 17
        t.add(x)
    assert t.result() == pytest.approx(2.3, abs=0.03)
    big = Tare(S)
    for _ in range(10):
        big.add(9.0)
    assert isinstance(big.result(), Refusal) and big.result().reason == "tare_too_large"
    moving = Tare(S)
    for x in np.linspace(0.0, 1.5, 20):
        moving.add(x)
    assert moving.result().reason == "tare_unsteady"
    assert Tare(S).result().reason == "no_tare"


def _landing(t_touch=1.2, k=600.0, speed=0.004, bias=2.3, drift=0.3, seed=0):
    """Force estimate while lowering at `speed` onto paper of stiffness k (N/m) at t_touch,
    with an air bias that drifts and sensor noise."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, 2.0, 1.0 / RATE)
    f = bias + drift * t / 2.0 + 0.04 * rng.standard_normal(len(t))
    f += np.where(t > t_touch, k * speed * (t - t_touch), 0.0)
    return t, f


def test_contact_is_the_force_lifting_off_the_air_zero():
    t, f = _landing()
    zero = np.mean(f[:int(0.2 * RATE)])             # tared over the first 0.2 s
    c = Contact(S)
    hit = next(ti for ti, fi in zip(t, f) if c.update(fi - zero, ti))
    # 0.25 N over the zero at 2.4 N/s, plus the drift: about 0.1 s after the paper
    assert 1.2 < c.at < 1.35 and hit - c.at == pytest.approx(2 / RATE)


def test_contact_ignores_a_single_spike():
    c = Contact(S)
    for i, x in enumerate([0.0, 0.0, 3.0, 0.0, 0.0, 0.0]):
        assert not c.update(x, i / RATE)


def test_guard_holds_on_a_sustained_over_force_only():
    g = Guard(S)
    assert not any(g.update(x) for x in [5.0] * 11 + [0.9] + [5.0] * 11)
    g = Guard(S)
    out = [g.update(x) for x in [5.0] * 12]
    assert out[-1] and not any(out[:-1])


def test_servo_trims_toward_the_setpoint_and_is_bounded():
    s = ForceSettings(servo_ki=2.0, trim_max_n=1.0)
    servo, plant_gain = Servo(s), 0.7          # the paper feels only 70 % of what is fed
    f_meas = 0.0
    for _ in range(int(5 * RATE)):
        trim = servo.update(0.85, f_meas, 1 / RATE, in_contact=True)
        f_meas = plant_gain * (0.85 + trim)
    assert f_meas == pytest.approx(0.85, abs=0.01)
    stuck = Servo(s)
    for _ in range(int(5 * RATE)):
        stuck.update(0.85, 0.0, 1 / RATE, in_contact=True)        # never feels anything
    assert stuck.trim == pytest.approx(1.0)
    off = Servo(ForceSettings())                                  # ki 0: feed forward only
    assert off.update(0.85, 0.0, 1.0, True) == 0.0
