"""The Franka FR3: numbers only.  Kinematics, joint limits, collision capsules of the arm.

Everything is in the arm's own frames.  Frame index, as `Arm.link_frames` returns them:
  0..7   link0 (the base) .. link7, the URDF link frames (Craig modified DH)
  8      the flange: link7 moved 0.107 m along its z
  9      the hand: the flange turned -45 deg about z (the stock Franka hand mounting)
The hand frame is the frame of `T_base_hand` everywhere in the new code.  The old code's
"hand TCP" is this frame moved D_HAND_TCP = 0.1034 m along z; the IK solver works in that one.
"""
import numpy as np

# ------------------------------------------------------------------ kinematics
# Craig modified DH: (alpha_{i-1}, a_{i-1}, d_i), from the old aris_sixarm/frames.py.
DH = ((0.0, 0.0, 0.333),
      (-np.pi / 2, 0.0, 0.0),
      (np.pi / 2, 0.0, 0.316),
      (np.pi / 2, 0.0825, 0.0),
      (-np.pi / 2, -0.0825, 0.384),
      (np.pi / 2, 0.0, 0.0),
      (np.pi / 2, 0.088, 0.0))
D_FLANGE = 0.107          # link7 -> flange, along z
HAND_TWIST = -np.pi / 4   # flange -> hand, about z
D_HAND_TCP = 0.1034       # hand -> the IK solver's end-effector point, along z

# ------------------------------------------------------------------ limits
# Positions: FR3 datasheet, as in the old frames.py (FR3_MIN / FR3_MAX).
Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
Q_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
# Velocities: the FR3 URDF <limit velocity>, confirmed by libfranka's rate limiter (old
# frames.py QD_MAX).
QD_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
# Acceleration and jerk: libfranka rate_limiting.h (kMaxJointAcceleration, kMaxJointJerk).
# 10 rad/s^2 is also the fr3drivers gate (--fr3_max_joint_acceleration, HARDWARE_DAY1.md).
QDD_MAX = np.full(7, 10.0)
QDDD_MAX = np.full(7, 5000.0)

# ------------------------------------------------------------------ the arm's collision body
# (name, frame index, a, b, radius): a capsule from a to b, both in that frame's coordinates.
#
# link0..link7: taken unchanged from the old aris_sixarm/selfcoll.py BODY_CAPSULES, the table
# the 2026-08-26 self-collision audit measured (scripts/self_collision_audit.py --part
# capsules).  Each link is split into three bands along its own fitted axis and every radius is
# the exact maximum distance of any vertex of the manufacturer's collision mesh UNION the
# high-quality visual mesh, rounded up to the millimetre.  link0 is seven bands about the base
# axis instead (a chord across the base disc is mostly air); its lowest band holds the base
# connector's cable stub, which reaches 0.231 m behind the mounting face.
# Chosen over the other old models because the audit showed them either not containing the
# metal (the vendored sphere sets, worst +235 mm on link0) or being fat sausages drawn about the
# lines between joint origins (coordination.CAPSULES, radii 0.09-0.13, offsets up to 131 mm).
# Containment is re-checked against the meshes by tests/test_kernel_arm.py.
LINK_CAPSULES = (
    ("link0.0", 0, (0.0000, 0.0000, -0.2380), (0.0000, 0.0000, -0.0750), 0.177),
    ("link0.1", 0, (0.0000, 0.0000, -0.0750), (0.0000, 0.0000, 0.0000), 0.176),
    ("link0.2", 0, (0.0000, 0.0000, 0.0000), (0.0000, 0.0000, 0.0350), 0.171),
    ("link0.3", 0, (0.0000, 0.0000, 0.0350), (0.0000, 0.0000, 0.0700), 0.160),
    ("link0.4", 0, (0.0000, 0.0000, 0.0700), (0.0000, 0.0000, 0.1000), 0.112),
    ("link0.5", 0, (0.0000, 0.0000, 0.1000), (0.0000, 0.0000, 0.1200), 0.076),
    ("link0.6", 0, (0.0000, 0.0000, 0.1200), (0.0000, 0.0000, 0.1440), 0.078),
    ("link1.0", 1, (0.0011, 0.0018, -0.1069), (-0.0018, 0.0070, -0.1962), 0.063),
    ("link1.1", 1, (-0.0029, -0.0576, -0.0136), (0.0148, 0.0074, -0.1251), 0.068),
    ("link1.2", 1, (-0.0127, -0.0145, -0.0386), (0.0121, -0.0976, 0.0535), 0.076),
    ("link2.0", 2, (-0.0008, -0.1986, -0.0081), (0.0014, -0.1105, 0.0001), 0.063),
    ("link2.1", 2, (0.0103, -0.1237, 0.0012), (0.0027, -0.0163, 0.0476), 0.069),
    ("link2.2", 2, (0.0093, 0.0542, 0.0977), (-0.0072, -0.0389, 0.0146), 0.075),
    ("link3.0", 3, (0.0732, -0.0047, -0.0006), (0.1003, 0.1079, 0.0090), 0.062),
    ("link3.1", 3, (0.0361, -0.0447, -0.0482), (0.0396, 0.0952, -0.0350), 0.075),
    ("link3.2", 3, (0.0184, 0.0076, -0.1417), (-0.0404, -0.0208, -0.0299), 0.059),
    ("link4.0", 4, (0.0107, -0.0025, 0.1107), (0.0043, -0.0086, -0.0034), 0.062),
    ("link4.1", 4, (-0.0991, -0.0014, 0.0151), (-0.0049, 0.0772, 0.0528), 0.077),
    ("link4.2", 4, (-0.0675, 0.1436, 0.0275), (-0.1157, 0.0410, -0.0440), 0.064),
    ("link5.0", 5, (-0.0131, 0.0089, -0.0072), (0.0096, 0.1292, -0.0110), 0.063),
    ("link5.1", 5, (0.0041, 0.1051, -0.1199), (-0.0022, 0.0072, -0.1083), 0.067),
    ("link5.2", 5, (0.0141, 0.0790, -0.1657), (-0.0073, -0.0417, -0.2735), 0.061),
    ("link6.0", 6, (-0.0196, -0.0460, 0.0285), (-0.0129, 0.0561, 0.0223), 0.051),
    ("link6.1", 6, (0.0658, 0.0834, 0.0020), (0.0341, -0.0526, 0.0140), 0.056),
    ("link6.2", 6, (0.0839, -0.0541, 0.0004), (0.1083, 0.0748, -0.0036), 0.049),
    ("link7.0", 7, (0.0076, 0.0044, 0.0522), (-0.0360, -0.0316, 0.0967), 0.047),
    ("link7.1", 7, (-0.0169, 0.0449, 0.0861), (0.0444, -0.0183, 0.0861), 0.044),
    ("link7.2", 7, (0.0233, 0.0236, 0.0810), (0.0641, 0.0603, 0.0886), 0.038),
)

# The hand and the two Fat Franka Finger blades, in the hand frame (index 9).  NEW, measured by
# tests/oracle/fit_arm_capsules.py with the same method: the old table's three hand bands were
# fitted to the stock fingers, and the Fat finger blades reach 80 mm along hand x, past them.
# The comment is the exact mesh maximum each radius rounds up from.
HAND_CAPSULES = (
    ("hand.0", 9, (-0.0083, 0.0998, 0.0619), (-0.0017, 0.0501, -0.0209), 0.039),          # 0.0385
    ("hand.1", 9, (-0.0018, 0.0328, 0.0168), (0.0016, -0.0345, 0.0166), 0.036),           # 0.0352
    ("hand.2", 9, (-0.0047, -0.0684, -0.0319), (0.0031, -0.0895, 0.0671), 0.031),         # 0.0300
    ("finger_left.0", 9, (0.0462, 0.0270, 0.1177), (0.0808, 0.0415, 0.0631), 0.024),      # 0.0238
    ("finger_left.1", 9, (-0.0035, 0.0406, 0.0606), (0.0076, 0.0306, 0.1152), 0.016),     # 0.0155
    ("finger_right.0", 9, (0.0728, -0.0412, 0.0607), (0.0610, -0.0293, 0.1150), 0.016),   # 0.0156
    ("finger_right.1", 9, (0.0205, -0.0242, 0.1172), (-0.0109, -0.0430, 0.0630), 0.024),  # 0.0232
)

# Where each body sits along the chain, for the self-collision pair rule in arm.py.  The hand,
# the fingers and the tool are one rigid body on the hand frame.
CHAIN_POS = dict(link0=0, link1=1, link2=2, link3=3, link4=4, link5=5, link6=6, link7=7,
                 hand=8, finger_left=8, finger_right=8, tool=8)
# Two bodies fewer than this many joints apart are held apart by the mechanism (they meet at
# a joint and their capsules overlap in every configuration); old selfcoll.WATCH_CHAIN_D.
SELF_CHAIN_GAP = 4
# Clearance the arm owes itself when planning: 20 mm of metal plus 3 mm the old checker's
# sampling needed (old selfcoll.SELF_PLAN_MARGIN; same number as types.Gates.self_margin).
SELF_MARGIN = 0.023
