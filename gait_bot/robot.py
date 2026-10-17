"""Planar biped kinematics: FK, analytic IK (knee forward), center of mass.

Angle conventions (radians internally):
  q_hip   thigh angle measured from straight down, + = forward (thigh swings toward +x)
  q_knee  knee flexion, 0 = straight leg, + = shank folds backward
  shank absolute angle from straight down = q_hip - q_knee
  foot_angle  absolute pitch of the sole (0 = horizontal, + = toe up)
The ankle lies on the sole line; heel is heel_offset behind it, toe is the rest ahead.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .config import RobotConfig
from .terrain import Terrain

Point = tuple[float, float]


def _down(angle: float, length: float) -> Point:
    """Vector of `length` at `angle` from straight down (+ = toward +x)."""
    return length * math.sin(angle), -length * math.cos(angle)


@dataclass(frozen=True)
class LegAngles:
    q_hip: float
    q_knee: float
    foot_angle: float = 0.0


@dataclass(frozen=True)
class IKResult:
    ok: bool
    angles: LegAngles | None
    reason: str = ""


@dataclass(frozen=True)
class Segment:
    name: str
    p0: Point
    p1: Point
    mass: float

    @property
    def mid(self) -> Point:
        return (self.p0[0] + self.p1[0]) / 2, (self.p0[1] + self.p1[1]) / 2


@dataclass(frozen=True)
class Pose:
    hip: Point
    left: LegAngles
    right: LegAngles


class Robot:
    def __init__(self, cfg: RobotConfig | None = None):
        self.cfg = cfg or RobotConfig()
        c = self.cfg
        self.max_reach = c.reach_margin * (c.thigh_len + c.shank_len)
        k = math.radians(c.knee_max_deg)
        self.min_reach = math.sqrt(c.thigh_len**2 + c.shank_len**2
                                   + 2 * c.thigh_len * c.shank_len * math.cos(k))

    @property
    def total_mass(self) -> float:
        c = self.cfg
        return c.torso_mass + 2 * (c.thigh_mass + c.shank_mass + c.foot_mass)

    # ---------- forward kinematics ----------
    def leg_points(self, hip: Point, q: LegAngles) -> dict[str, Point]:
        c = self.cfg
        dx, dy = _down(q.q_hip, c.thigh_len)
        knee = (hip[0] + dx, hip[1] + dy)
        dx, dy = _down(q.q_hip - q.q_knee, c.shank_len)
        ankle = (knee[0] + dx, knee[1] + dy)
        ux, uy = math.cos(q.foot_angle), math.sin(q.foot_angle)
        toe_len = c.foot_len - c.heel_offset
        heel = (ankle[0] - c.heel_offset * ux, ankle[1] - c.heel_offset * uy)
        toe = (ankle[0] + toe_len * ux, ankle[1] + toe_len * uy)
        return {"hip": hip, "knee": knee, "ankle": ankle, "heel": heel, "toe": toe}

    def segments(self, pose: Pose) -> list[Segment]:
        c = self.cfg
        hip = pose.hip
        segs = [Segment("torso", hip, (hip[0], hip[1] + c.torso_len), c.torso_mass)]
        for side, q in (("L", pose.left), ("R", pose.right)):
            p = self.leg_points(hip, q)
            segs += [
                Segment(f"thigh_{side}", p["hip"], p["knee"], c.thigh_mass),
                Segment(f"shank_{side}", p["knee"], p["ankle"], c.shank_mass),
                Segment(f"foot_{side}", p["heel"], p["toe"], c.foot_mass),
            ]
        return segs

    def com(self, pose: Pose) -> Point:
        segs = self.segments(pose)
        m = sum(s.mass for s in segs)
        return (sum(s.mass * s.mid[0] for s in segs) / m,
                sum(s.mass * s.mid[1] for s in segs) / m)

    # ---------- inverse kinematics ----------
    def ik(self, hip: Point, ankle: Point, foot_angle: float = 0.0) -> IKResult:
        """Hip and ankle positions → leg angles, knee forward. Checks reach and joint limits."""
        c = self.cfg
        L1, L2 = c.thigh_len, c.shank_len
        dx, dy = ankle[0] - hip[0], ankle[1] - hip[1]
        d = math.hypot(dx, dy)
        if d > self.max_reach + 1e-12:
            return IKResult(False, None, f"too far ({d:.3f} > {self.max_reach:.3f})")
        if d < self.min_reach - 1e-12:
            return IKResult(False, None, f"too close ({d:.3f} < {self.min_reach:.3f})")

        alpha = math.atan2(dx, -dy)  # direction hip→ankle, from straight down
        cos_beta = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
        beta = math.acos(max(-1.0, min(1.0, cos_beta)))  # thigh rotates forward of that line
        cos_inner = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
        q_knee = math.pi - math.acos(max(-1.0, min(1.0, cos_inner)))
        q_hip = alpha + beta

        angles = LegAngles(q_hip, q_knee, foot_angle)
        hip_deg, knee_deg = math.degrees(q_hip), math.degrees(q_knee)
        if not c.hip_min_deg <= hip_deg <= c.hip_max_deg:
            return IKResult(False, angles, f"hip limit ({hip_deg:.0f}°)")
        if not c.knee_min_deg <= knee_deg <= c.knee_max_deg + 1e-9:
            return IKResult(False, angles, f"knee limit ({knee_deg:.0f}°)")
        return IKResult(True, angles)

    # ---------- convenience ----------
    def foothold(self, terrain: Terrain, x: float) -> tuple[Point, float]:
        """Ankle point on the terrain at x, and the sole angle (parallel to the surface)."""
        y = terrain.height_at(x)
        c = self.cfg
        x0, x1 = x - c.heel_offset, x + c.foot_len - c.heel_offset
        slope = math.atan2(terrain.height_at(x1) - terrain.height_at(x0), x1 - x0)
        return (x, y), slope

    def stand(self, terrain: Terrain, x_rear: float, x_front: float, hip_dy: float = 0.0
              ) -> tuple[Pose | None, list[IKResult], Point]:
        """Double-support pose: left foot at x_rear, right at x_front, upright torso.
        Hip x is centered between ankles; hip y is nominal height (+ hip_dy) above the feet,
        lowered if needed so both ankles are reachable. Returns (pose or None, IK results, hip)."""
        a_l, ang_l = self.foothold(terrain, x_rear)
        a_r, ang_r = self.foothold(terrain, x_front)
        hx = (a_l[0] + a_r[0]) / 2
        y = (a_l[1] + a_r[1]) / 2 + self.cfg.nominal_hip_height + hip_dy
        for a in (a_l, a_r):  # highest y keeping this ankle within reach
            h = self.max_reach**2 - (hx - a[0]) ** 2
            if h > 0:
                y = min(y, a[1] + math.sqrt(h))
        hip = (hx, y)
        res = [self.ik(hip, a_l, ang_l), self.ik(hip, a_r, ang_r)]
        if not all(r.ok for r in res):
            return None, res, hip
        return Pose(hip, res[0].angles, res[1].angles), res, hip
