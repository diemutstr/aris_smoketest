"""The shared data types.  Every module talks to every other module in these, and nothing else.

Units: metres, radians, seconds.
Frames: every position says which frame it is in, either in the variable name (`p_base`,
`T_table_base`) or in a `frame` field.  Two frames exist:
  "table"  origin at the table centre on the paper surface, x across, y along, z up
  "base"   one arm's own base frame (the arm model's origin)
The planners below the system planner only ever see the "base" frame.

This file holds data only.  No logic, no I/O, no imports from the rest of the package.
Changing it changes every module, so it is changed by the orchestrator only.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

N_JOINTS = 7

# An arm is named by its SLOT: the place on the frame it hangs from, "1L" .. "3R" (row 1 to 3
# along the table's length, L/R by x).  Which robot (serial, address) hangs in a slot is a
# fact of the site (site/*.json), never of the planning code.  Decided 2026-10-02.
Slot = str

# --------------------------------------------------------------------------- drawing input


@dataclass(frozen=True)
class Line:
    """One line of the drawing: where the pen tip should go."""
    id: str
    points: np.ndarray                 # (N, 3) tip positions
    frame: Literal["table", "base"]
    intensity: float = 1.0             # 0..1, how hard to press


# --------------------------------------------------------------------------- obstacles
# An obstacle carries the clearance it demands (`margin`).  "Free" means: distance to every
# obstacle is at least that obstacle's margin.  A planner never needs to know what an
# obstacle is; a wall, a parked arm and a strut of the frame are all just geometry.


@dataclass(frozen=True)
class Box:
    name: str
    T_base_box: np.ndarray             # (4, 4) pose of the box centre
    half: np.ndarray                   # (3,) half extents
    margin: float
    # Body groups (the part of a capsule's name before the first ".", e.g. "link1") that are
    # not checked against this box per pose, because their clearance to it is a fact about
    # the rig settled once: an arm's own link 1 against the steel it hangs from.
    exempt: tuple[str, ...] = ()


@dataclass(frozen=True)
class Plane:
    """A flat boundary.  Free space is the side the normal points to: normal . p >= offset."""
    name: str
    normal: np.ndarray                 # (3,) unit vector, pointing into free space
    offset: float
    margin: float
    kind: Literal["paper", "wall", "other"] = "other"
    # The pen touches the paper when drawing and hovers close to it when lifted, so the pen
    # gets its own, smaller clearance against a plane.  None means: same as `margin`.
    pen_margin: float | None = None
    # The pen holder and the gripper that holds it work a few millimetres above the paper, so
    # the tool gets its own clearance too.  None means: same as `margin`.
    tool_margin: float | None = None


@dataclass(frozen=True)
class Capsule:
    """A segment with a radius, e.g. one link of a parked arm."""
    name: str
    p0: np.ndarray                     # (3,)
    p1: np.ndarray                     # (3,)
    radius: float
    margin: float


@dataclass(frozen=True)
class Obstacles:
    """Everything one arm has to stay clear of, in that arm's base frame."""
    boxes: tuple[Box, ...] = ()
    planes: tuple[Plane, ...] = ()
    capsules: tuple[Capsule, ...] = ()


# --------------------------------------------------------------------------- the arm's body


@dataclass(frozen=True)
class Body:
    """The arm's collision body at N configurations: K capsules each."""
    p0: np.ndarray                     # (N, K, 3)
    p1: np.ndarray                     # (N, K, 3)
    radius: np.ndarray                 # (K,)
    names: tuple[str, ...]             # K names, e.g. "forearm", "hand", "pen"
    is_pen: np.ndarray                 # (K,) bool
    # Capsules that do not move with the joints: the base and what is bolted to it.  They
    # hang inside the arm's own mount, so they are never checked against obstacles; whether
    # the mount fits is a fact about the rig, settled once.  They still count for the arm
    # against itself, and they are part of what a neighbour sees of this arm.  (Link 1, which
    # turns about the base axis, is NOT fixed: it is checked per pose like every other link,
    # except against the arm's own mount steel, see `Box.exempt`.)
    is_fixed: np.ndarray | None = None # (K,) bool; None means no capsule is fixed
    # The tool: what is bolted to the flange (gripper, blades, pen holder), the pen excluded.
    is_tool: np.ndarray | None = None  # (K,) bool; None means no capsule is a tool capsule


@dataclass(frozen=True)
class Tool:
    """What is bolted to the hand: the pen holder and the pen."""
    tip_hand: np.ndarray               # (3,) pen tip in the hand frame
    pen_axis_hand: np.ndarray          # (3,) unit vector along the pen, pointing out of the tip
    capsules_hand: tuple[Capsule, ...] # holder and pen, in the hand frame (margin unused)
    pen_names: tuple[str, ...]         # which of those capsules are the pen itself


# --------------------------------------------------------------------------- limits


@dataclass(frozen=True)
class Limits:
    """What the arm's joints may do.  All arrays are (7,)."""
    q_min: np.ndarray                  # rad
    q_max: np.ndarray                  # rad
    qd_max: np.ndarray                 # rad/s
    qdd_max: np.ndarray                # rad/s^2
    qddd_max: np.ndarray               # rad/s^3


# --------------------------------------------------------------------------- paths and motions


@dataclass(frozen=True)
class JointPath:
    """A path without timing."""
    q: np.ndarray                      # (N, 7)


@dataclass(frozen=True)
class Trajectory:
    """A path with timing, inside the arm's limits.

    The motion between two samples is the cubic that matches q and qd at both of them, so a
    trajectory can be sampled at any rate without changing what it means.
    """
    t: np.ndarray                      # (N,) seconds from the start, increasing
    q: np.ndarray                      # (N, 7)
    qd: np.ndarray                     # (N, 7)


@dataclass(frozen=True)
class Piece:
    """A stretch of one line, by arc length along it."""
    line_id: str
    s0: float                          # metres from the start of the line
    s1: float


@dataclass(frozen=True)
class DrawPlan:
    """One way of drawing a piece.  Can be run in either direction."""
    piece: Piece
    q: np.ndarray                      # (N, 7) joint path, pen on the paper
    s: np.ndarray                      # (N,) arc length along the line at each sample
    tip_base: np.ndarray               # (N, 3)
    score: float                       # worst margin along the path; higher is better
    joint_travel: float                # radians summed over joints and samples
    draw_time: float = 0.0             # seconds the timed motion takes; 0 if not timed yet
    spin: np.ndarray | None = None     # (N,) hand spin about the paper normal, for inspection
    lean: np.ndarray | None = None     # (N, 2) pen lean, for inspection

    @property
    def q_start(self) -> np.ndarray:
        return self.q[0]

    @property
    def q_end(self) -> np.ndarray:
        return self.q[-1]


@dataclass(frozen=True)
class Bunch:
    """The alternatives for one piece.  The sequencer picks one."""
    piece: Piece
    plans: tuple[DrawPlan, ...]


Reason = Literal[
    "unreachable",        # no arm configuration puts the tip there inside the gates
    "blocked",            # reachable, but an obstacle is in the way
    "too_short",          # shorter than the smallest stretch worth a pen-down
    "no_free_path",       # could be drawn, but the arm cannot fly to it
    "failed_check",       # the independent checker refused the motion
    "stopped",            # the job was stopped before this was drawn
    "failed",             # an arm failed before this was drawn
    "unaccounted",        # nothing above applies; a bug, reported rather than hidden
]


@dataclass(frozen=True)
class Leftover:
    """Something that was asked for and not planned, and why."""
    piece: Piece
    reason: Reason
    detail: str = ""


@dataclass(frozen=True)
class Motion:
    """The only thing that crosses from planning to execution.

    Starts where the previous motion of the same arm ended.  Ends in a configuration the arm
    can hold for as long as it likes.
    """
    # draw: pen on the paper.  free: pen up, anywhere.  lower: the short move that puts the
    # pen down at the start of a piece; lift: the short move that takes it up at the end.
    # For lower and lift, the pen may be at the paper at one end.
    # touch (calibration): the pen descends from a hover pose onto the nominal paper and comes
    # back up the same way; `traj` is that down-and-up, so the motion starts and ends at the
    # hover.  The arm flies the descent slowly under position control and stops at the first
    # contact, where it reads its joints; finding none by the planned end, it may keep going
    # straight on for `extra_depth` before giving up.  The planned path is what is checked;
    # the extra depth is the declared uncertainty of the paper's height.
    # guide (calibration): the arm stands at a hover above a meeting spot; the person puts the
    # arms in programming mode, brings the two pen tips together and puts them back in
    # execution mode; the driver reads the joints standing still, retreats and returns to
    # the hover.  `traj` is the hover (one sample, where the arm is); `piece.line_id` names
    # the spot; `tip_base` its nominal position.  The registered joints come back as a row.
    # retreat: a short motion that only moves the arm AWAY from the other arms (straight up,
    # then away from the nearest one) — flown from a pose that is already closer than the
    # planners' arm-to-arm clearance allows (after a meeting that was interrupted).  The
    # checker judges it by the distance to every other arm never decreasing, instead of the
    # fixed clearance; everything else (paper, steel, fences, limits) applies as to a free move.
    kind: Literal["draw", "free", "lower", "lift", "touch", "guide", "retreat"]
    traj: Trajectory
    piece: Piece | None = None         # draw motions: what is being drawn
    tip_base: np.ndarray | None = None # draw and touch motions: (N, 3) tips, same samples as traj
    intensity: float = 1.0
    extra_depth: float = 0.0           # touch motions: m past the planned end, see above
    # The independent checker's word on this motion, when it was checked before being handed
    # on: at least {"passed": bool, "tightest": str}, plus the numbers the checker keeps.  The
    # planners do not read it; the execution side queues nothing without it.  None: unchecked.
    checked: dict | None = None

    @property
    def q_start(self) -> np.ndarray:
        return self.traj.q[0]

    @property
    def q_end(self) -> np.ndarray:
        return self.traj.q[-1]


# --------------------------------------------------------------------------- refusals


@dataclass(frozen=True)
class Refusal:
    """A call that could not do what was asked, and why.  Returned, never raised."""
    reason: str
    detail: str = ""


# --------------------------------------------------------------------------- the rig at one moment


@dataclass(frozen=True)
class Wall:
    """A vertical plane in the table frame, between two arms."""
    name: str
    arms: tuple[Slot, Slot]
    point_table: np.ndarray            # (3,) a point on the wall, on the paper
    normal_table: np.ndarray           # (3,) unit, horizontal, pointing from arms[0] to arms[1]


@dataclass(frozen=True)
class Phase:
    """Who moves, who stands parked, and the walls between those who move.  `contact`: the
    phase's active arms are meant to end touching each other (a pen-tip meeting), so the
    phase-end check does not demand the arm-to-arm clearance between them."""
    name: str
    active: tuple[Slot, ...]
    parked: tuple[Slot, ...]
    walls: tuple[Wall, ...]
    contact: bool = False


# --------------------------------------------------------------------------- rules


@dataclass(frozen=True)
class Gates:
    """What a configuration must satisfy to count as usable."""
    limit_margin: float = 0.15         # rad, distance to the nearest joint limit
    sigma_min: float = 0.08            # smallest singular value of the tip Jacobian
    self_margin: float = 0.023         # m, the arm against itself


@dataclass(frozen=True)
class DrawRules:
    """How drawing motions are timed and shaped."""
    draw_speed: float = 0.02           # m/s along the line
    landing_speed: float = 0.010       # m/s of the pen tip on a lower, as it meets the paper
    # The drawing surface lies this far below the paper: with position control the pen is
    # pressed into the paper by planning it there (3.5 mm for 2 mm 4H graphite on the rig,
    # 2026-10-01).  The system planner gives the drawing's points this z; the planners below
    # it never know.  The real paper stays the plane the holder and links must clear.
    press: float = 0.0                 # m
    # A pen that skids when pushed (a gel pen in the lateral holder, found on the rig
    # 2026-10-06): pieces are drawn in the direction that pulls the tip (the tip trails the
    # hand's lean).  The rig's pen table says so per pen.
    drag_only: bool = False
    lean_max: float = np.deg2rad(15.0) # rad, how far the pen may lean off its nominal direction
    min_piece: float = 0.010           # m, shortest stretch worth a pen-down
    speed_fraction: float = 0.30       # fraction of the joint speed limits that may be used
    gates: Gates = field(default_factory=Gates)
