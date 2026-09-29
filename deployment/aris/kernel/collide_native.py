"""The compiled engine's side of the collision check: obstacles and the arm as flat tables.

`pack` turns `Obstacles` into flat arrays (once; reuse the result).  `arm_tables` turns an `Arm`
into its chain and capsule tables, from which `body_q` computes the body without the `Arm`.
`native_on` decides per call whether the compiled module `aris_collide_native` (built from
native/collide) runs.  Everything public here is also importable from `aris.kernel.collide`.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.kernel.collide_groups import body_groups, group_margins, obstacle_groups
from aris.types import Body, Obstacles

try:
    import aris_collide_native as _native
except ImportError:          # not built: numpy does the same job, slower
    _native = None


def native_on(backend: str | None) -> bool:
    """Whether a call with this `backend` argument runs compiled (None: whenever installed)."""
    if backend is None:
        return _native is not None
    if backend == "native" and _native is None:
        raise ValueError("the compiled module aris_collide_native is not installed")
    if backend not in ("native", "numpy"):
        raise ValueError(f"unknown backend {backend!r}")
    return backend == "native"


def backend() -> str:
    """Which engine the calls use by default: "native" (compiled) or "numpy"."""
    return "native" if _native is not None else "numpy"



# --------------------------------------------------------------------------- obstacles as arrays


@dataclass(frozen=True)
class Packed:
    """Obstacles as flat arrays.  Build once with `pack` and reuse; every call also accepts
    plain `Obstacles` and packs them itself.

    Margins may be changed with `dataclasses.replace` (the groups follow them); moving an
    obstacle needs a fresh `pack`, because the groups' fat capsules are built from the geometry.
    """
    box_R: np.ndarray        # (Mb, 3, 3)
    box_c: np.ndarray        # (Mb, 3)
    box_h: np.ndarray        # (Mb, 3)
    box_m: np.ndarray        # (Mb,)
    pl_n: np.ndarray         # (Mp, 3)
    pl_off: np.ndarray       # (Mp,)
    pl_m: np.ndarray         # (Mp,)
    pl_pen_m: np.ndarray     # (Mp,) margin for pen capsules
    pl_paper: np.ndarray     # (Mp,) bool
    cap_a: np.ndarray        # (Mc, 3)
    cap_b: np.ndarray        # (Mc, 3)
    cap_rm: np.ndarray       # (Mc,) obstacle radius + margin
    names: tuple[str, ...]   # boxes, then planes, then capsules: the obstacle index order
    pl_tool_m: np.ndarray | None = None   # (Mp,) margin for tool capsules; None: pl_m
    cap_r: np.ndarray | None = None       # (Mc,) obstacle radius; None: treated as 0
    # obstacle groups (collide_groups.py); None: built from positions alone when needed
    og_start: np.ndarray | None = None
    og_members: np.ndarray | None = None
    og_a: np.ndarray | None = None
    og_b: np.ndarray | None = None
    og_r: np.ndarray | None = None

    @property
    def tool_m(self) -> np.ndarray:
        return self.pl_m if self.pl_tool_m is None else self.pl_tool_m

    @property
    def radius(self) -> np.ndarray:
        return np.zeros(len(self.cap_rm)) if self.cap_r is None else self.cap_r

    @property
    def groups(self) -> tuple:
        """(start, members, a, b, r, margin) of the obstacle groups."""
        Mb, Mp = len(self.box_m), len(self.pl_m)
        g = (self.og_start, self.og_members, self.og_a, self.og_b, self.og_r)
        if g[0] is None:
            g = obstacle_groups(self.box_R, self.box_c, self.box_h, self.cap_a, self.cap_b,
                                self.radius, [""] * len(self.cap_rm), Mp)
        marg = group_margins(g[0], g[1], self.box_m, self.cap_rm - self.radius, Mb, Mp)
        return g + (marg,)

    @property
    def scene(self) -> tuple:
        """The arrays in the order the compiled module takes them."""
        c = np.ascontiguousarray
        return (self.box_R, self.box_c, self.box_h, self.box_m, self.pl_n, self.pl_off, self.pl_m,
                self.pl_pen_m, self.pl_paper.astype(np.uint8), self.cap_a, self.cap_b, self.cap_rm,
                c(self.tool_m, float)) + tuple(
                    c(x, np.int64) if i < 2 else c(x, float) for i, x in enumerate(self.groups))


def pack(obs: Obstacles | Packed) -> Packed:
    """The obstacles as flat arrays, with their groups for the first pass (collide_groups.py)."""
    if isinstance(obs, Packed):
        return obs
    f = lambda xs, shape: np.ascontiguousarray(np.asarray(xs, float).reshape(shape))
    b, p, c = obs.boxes, obs.planes, obs.capsules
    P = Packed(
        box_R=f([x.T_base_box[:3, :3] for x in b], (len(b), 3, 3)),
        box_c=f([x.T_base_box[:3, 3] for x in b], (len(b), 3)),
        box_h=f([x.half for x in b], (len(b), 3)),
        box_m=f([x.margin for x in b], (len(b),)),
        pl_n=f([x.normal for x in p], (len(p), 3)),
        pl_off=f([x.offset for x in p], (len(p),)),
        pl_m=f([x.margin for x in p], (len(p),)),
        pl_pen_m=f([x.margin if x.pen_margin is None else x.pen_margin for x in p], (len(p),)),
        pl_paper=np.array([x.kind == "paper" for x in p], bool).reshape(len(p)),
        cap_a=f([x.p0 for x in c], (len(c), 3)),
        cap_b=f([x.p1 for x in c], (len(c), 3)),
        cap_rm=f([x.radius + x.margin for x in c], (len(c),)),
        names=tuple(x.name for x in (*b, *p, *c)),
        pl_tool_m=f([x.margin if getattr(x, "tool_margin", None) is None else x.tool_margin
                     for x in p], (len(p),)),
        cap_r=f([x.radius for x in c], (len(c),)),
    )
    g = obstacle_groups(P.box_R, P.box_c, P.box_h, P.cap_a, P.cap_b, P.cap_r,
                        [x.name for x in c], len(p))
    return replace(P, og_start=g[0], og_members=g[1], og_a=g[2], og_b=g[3], og_r=g[4])


# --------------------------------------------------------------------------- from joint angles


@dataclass(frozen=True)
class ArmTables:
    """A serial arm as tables: everything the `_q` calls need to go from joints to capsules.

    Frame 0 is the base.  Frame i+1 follows from frame i by modified DH row i (alpha, a, d)
    and joint i turns about its z axis.  Then come fixed extra frames, each given in a parent
    frame.  Each capsule rides on one frame, its two ends given in that frame.
    """
    dh: np.ndarray               # (J, 3) alpha, a, d
    ex_parent: np.ndarray        # (E,) int
    ex_R: np.ndarray             # (E, 3, 3)
    ex_t: np.ndarray             # (E, 3)
    cap_frame: np.ndarray        # (K,) int
    cap_a: np.ndarray            # (K, 3)
    cap_b: np.ndarray            # (K, 3)
    radius: np.ndarray           # (K,)
    is_pen: np.ndarray           # (K,) bool
    is_fixed: np.ndarray         # (K,) bool
    names: tuple[str, ...]
    is_tool: np.ndarray | None = None   # (K,) bool; None: no tool capsules

    @property
    def tool(self) -> np.ndarray:
        return np.zeros(len(self.radius), bool) if self.is_tool is None else self.is_tool

    @property
    def chain(self) -> tuple:
        c = np.ascontiguousarray
        return (c(self.dh, float), c(self.ex_parent, np.int64), c(self.ex_R, float),
                c(self.ex_t, float), c(self.cap_frame, np.int64), c(self.cap_a, float),
                c(self.cap_b, float))

    @property
    def caps(self) -> tuple:
        return (self.radius, self.is_pen.astype(np.uint8), self.is_fixed.astype(np.uint8),
                self.tool.astype(np.uint8)) + body_groups(self.names)


def arm_tables(arm) -> ArmTables:
    """The tables of an `aris.kernel.arm.Arm`, from its `chain_table()` and `capsule_table()`:
    link0..link7 by DH, then its fixed frames (flange, hand), then its capsules."""
    ch, cp = arm.chain_table(), arm.capsule_table()
    return ArmTables(
        dh=np.asarray(ch.dh, float), ex_parent=np.asarray(ch.parent), ex_R=np.asarray(ch.R, float),
        ex_t=np.asarray(ch.t, float), cap_frame=np.asarray(cp.frame), cap_a=np.asarray(cp.a, float),
        cap_b=np.asarray(cp.b, float), radius=np.asarray(cp.radius, float),
        is_pen=np.asarray(cp.is_pen, bool), is_fixed=np.asarray(cp.is_fixed, bool),
        names=tuple(cp.names),
        is_tool=np.asarray(getattr(cp, "is_tool", np.zeros(len(cp.radius), bool)), bool))


def body_q(tables: ArmTables, Q, backend: str | None = None) -> Body:
    """(N, J) joint angles -> the arm's body, from the tables."""
    Q = np.ascontiguousarray(np.asarray(Q, float).reshape(-1, len(tables.dh)))
    if native_on(backend):
        p0, p1 = _native.body_q(tables.chain, Q)
    else:
        p0, p1 = _body_np(tables, Q)
    return Body(p0, p1, tables.radius, tables.names, tables.is_pen, tables.is_fixed, tables.tool)


def _body_np(t: ArmTables, Q):
    n, J = Q.shape
    F = 1 + J + len(t.ex_parent)
    R, p = np.empty((n, F, 3, 3)), np.zeros((n, F, 3))
    R[:, 0] = np.eye(3)
    for i, (al, a, d) in enumerate(t.dh):
        ca, sa, ct, st = np.cos(al), np.sin(al), np.cos(Q[:, i]), np.sin(Q[:, i])
        A = np.zeros((n, 3, 3))
        A[:, 0, 0], A[:, 0, 1] = ct, -st
        A[:, 1, 0], A[:, 1, 1], A[:, 1, 2] = st * ca, ct * ca, -sa
        A[:, 2, 0], A[:, 2, 1], A[:, 2, 2] = st * sa, ct * sa, ca
        R[:, i + 1] = R[:, i] @ A
        p[:, i + 1] = p[:, i] + R[:, i] @ np.array([a, -sa * d, ca * d])
    for e, par in enumerate(t.ex_parent):
        R[:, 1 + J + e] = R[:, par] @ t.ex_R[e]
        p[:, 1 + J + e] = p[:, par] + R[:, par] @ t.ex_t[e]
    Rk, pk = R[:, t.cap_frame], p[:, t.cap_frame]
    return (np.einsum("nkij,kj->nki", Rk, t.cap_a) + pk, np.einsum("nkij,kj->nki", Rk, t.cap_b) + pk)
