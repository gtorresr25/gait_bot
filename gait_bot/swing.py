"""Swing foot arc and hip path for one step: rear foothold S swings over stance M to target T.

Spatial only, parameterized by phase s in [0, 1] (no timing):
  ankle x(s) = S.x + (T.x - S.x) * σ(s)              σ(s) = 3s² - 2s³ (smoothstep)
  ankle y(s) = S.y + (T.y - S.y) * σ(s) + h * sin(πs)
  foot angle = S.angle → T.angle along σ(s)
(σ on both x and y keeps the baseline a straight line in space; the sine bump lifts it.)
h is the smallest bump that keeps the swing foot c·sin(πs) above the terrain, then raised in
2 cm increments if a leg can't reach or a shank hits the terrain (up to h_max).

The hip path and leg angles are closed-form in s too, so `Swing.state_at(s)` gives an exact
pose anywhere in the swing (the stance ankle stays exactly on M); `samples` are the
N_SAMPLES+1 states the planner verified.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import PlannerConfig
from .footholds import Foothold
from .robot import LegAngles, Pose, Robot
from .terrain import Terrain

N_SAMPLES = 40
H_STEP = 0.02


@dataclass(frozen=True)
class SwingSample:
    s: float
    hip: tuple[float, float]
    ankle: tuple[float, float]        # swing ankle
    stance: LegAngles
    swing: LegAngles


def _smooth(s: float) -> float:
    return s * s * (3.0 - 2.0 * s)


def _ankle(S: Foothold, T: Foothold, h: float, s: float) -> tuple[tuple[float, float], float]:
    """Swing ankle position and sole angle at phase s."""
    g = _smooth(s)
    return ((S.x + (T.x - S.x) * g, S.y + (T.y - S.y) * g + h * math.sin(math.pi * s)),
            S.angle + (T.angle - S.angle) * g)


def _state(robot: Robot, S: Foothold, M: Foothold, T: Foothold, h: float, s: float
           ) -> tuple[SwingSample | None, str]:
    """Hip and leg angles at phase s, or (None, reason) if the legs can't do it."""
    R, r = robot.max_reach, robot.min_reach
    nom = robot.cfg.nominal_hip_height
    ankle, angle = _ankle(S, T, h, s)

    # hip: x from mid(S,M) to mid(M,T); y at nominal height, lowered to stay within max reach
    # of both ankles, raised to stay beyond min reach (knee limit)
    hx = (S.x + M.x) / 2 + ((T.x - S.x) / 2) * s
    upper = (S.y + M.y) / 2 + nom + ((T.y - S.y) / 2) * s
    lower = -math.inf
    for a in ((M.x, M.y), ankle):
        dx = hx - a[0]
        if abs(dx) > R:
            return None, f"reach at s={s:.2f}"
        upper = min(upper, a[1] + math.sqrt(R * R - dx * dx))
        if abs(dx) < r:
            lower = max(lower, a[1] + math.sqrt(r * r - dx * dx))
    if lower > upper + 1e-9:
        return None, f"hip squeezed at s={s:.2f}"
    hip = (hx, max(upper, lower))

    st = robot.ik(hip, (M.x, M.y), M.angle)
    if not st.ok:
        return None, f"stance {st.reason} at s={s:.2f}"
    sw = robot.ik(hip, ankle, angle)
    if not sw.ok:
        return None, f"swing {sw.reason} at s={s:.2f}"
    return SwingSample(s, hip, ankle, st.angles, sw.angles), ""


@dataclass(frozen=True)
class Swing:
    ok: bool
    reason: str
    h: float                           # apex bump height above the straight S→T line
    S: Foothold
    M: Foothold
    T: Foothold
    robot: Robot = field(compare=False, repr=False)
    samples: tuple[SwingSample, ...] = ()

    def state_at(self, s: float) -> SwingSample:
        """Exact state anywhere in a feasible swing (s is clamped to [0, 1])."""
        st, reason = _state(self.robot, self.S, self.M, self.T, self.h, min(1.0, max(0.0, s)))
        if st is None:   # can only happen between verified samples; fall back to the nearest one
            return self.samples[round(min(1.0, max(0.0, s)) * N_SAMPLES)]
        return st

    def pose_at(self, s: float, swing_side: str) -> Pose:
        """Robot pose at phase s; swing_side is the side of the moving foot."""
        p = self.state_at(s)
        if swing_side == "L":
            return Pose(p.hip, p.swing, p.stance)
        return Pose(p.hip, p.stance, p.swing)

    def pose(self, k: int, swing_side: str) -> Pose:
        return self.pose_at(self.samples[k].s, swing_side)

    def foot_travel(self, n: int = 20) -> float:
        """Length of the swing ankle's path in space (used for playback timing)."""
        pts = [_ankle(self.S, self.T, self.h, k / n)[0] for k in range(n + 1)]
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _foot_ends(robot: Robot, ankle, angle: float):
    c = robot.cfg
    ux, uy = math.cos(angle), math.sin(angle)
    toe = c.foot_len - c.heel_offset
    return ((ankle[0] - c.heel_offset * ux, ankle[1] - c.heel_offset * uy),
            (ankle[0] + toe * ux, ankle[1] + toe * uy))


def _corner_crossings(terrain: Terrain, robot: Robot, S: Foothold, T: Foothold) -> list[float]:
    """Phases where the heel or toe passes over a terrain vertex. The ground under the foot can
    jump there (stair corners), so fixed samples alone can miss the closest approach."""
    c = robot.cfg
    ends = (-c.heel_offset, c.foot_len - c.heel_offset)    # heel, toe offsets along the sole

    def end_x(s: float, off: float) -> float:
        (ax, _), ang = _ankle(S, T, 0.0, s)
        return ax + off * math.cos(ang)

    lo_x = min(S.x, T.x) - c.heel_offset
    hi_x = max(S.x, T.x) + c.foot_len
    out = []
    for xv in sorted({x for x, _ in terrain.profile if lo_x < x < hi_x}):
        for off in ends:
            a, b = 0.0, 1.0
            fa, fb = end_x(a, off) - xv, end_x(b, off) - xv
            if fa * fb > 0:
                continue
            for _ in range(30):      # bisection; x(s) is monotone (smoothstep, small angles)
                m = (a + b) / 2
                fm = end_x(m, off) - xv
                if (fm > 0) == (fa > 0):
                    a, fa = m, fm
                else:
                    b = m
            for sc in (a, b):
                if 0.0 < sc < 1.0:
                    out.append(sc)
    return out


def plan_swing(terrain: Terrain, robot: Robot, cfg: PlannerConfig,
               S: Foothold, M: Foothold, T: Foothold) -> Swing:
    c = cfg.swing_clearance
    ss = [k / N_SAMPLES for k in range(N_SAMPLES + 1)]

    # 1. minimal bump: foot gap(h) = gap0 + h·sin(πs) must be ≥ c·sin(πs) at every interior sample
    #    and at every phase where the foot passes over a terrain corner
    h = 0.0
    for s in ss[1:-1] + _corner_crossings(terrain, robot, S, T):
        ankle0, angle = _ankle(S, T, 0.0, s)
        gap0 = terrain.vertical_clearance(*_foot_ends(robot, ankle0, angle))
        b = math.sin(math.pi * s)
        h = max(h, (c * b - gap0) / b)
    if h > cfg.swing_h_max:
        return Swing(False, f"arc too high ({h:.2f} m)", h, S, M, T, robot)

    # 2. hip path + IK + shank checks at every sample; raise the arc if needed
    reason = ""
    while h <= cfg.swing_h_max + 1e-9:
        samples, reason = _track(terrain, robot, S, M, T, h, ss)
        if samples:
            return Swing(True, "", h, S, M, T, robot, samples)
        h += H_STEP
    return Swing(False, reason, h, S, M, T, robot)


def _track(terrain, robot, S, M, T, h, ss):
    out = []
    for s in ss:
        st, reason = _state(robot, S, M, T, h, s)
        if st is None:
            return None, reason
        for name, q in (("swing", st.swing), ("stance", st.stance)):
            p = robot.leg_points(st.hip, q)
            if terrain.vertical_clearance(p["knee"], p["ankle"]) < -1e-6:
                return None, f"{name} shank hits terrain at s={s:.2f}"
        out.append(st)
    return tuple(out), ""
