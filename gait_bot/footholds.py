"""Candidate footholds: places where the whole foot fits on one surface.

A surface is a non-vertical segment of the terrain profile. A foothold is valid if the
foot footprint [x - heel, x + toe] lies on a single surface and keeps `edge_margin` from
any riser corner. Stair treads (short horizontal surfaces between two risers) get exactly
one candidate: the foot centered on the tread.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .config import PlannerConfig, RobotConfig
from .terrain import Terrain


@dataclass(frozen=True)
class Surface:
    x0: float
    y0: float
    x1: float
    y1: float
    edge_left: bool    # riser attached at x0
    edge_right: bool   # riser attached at x1
    tread: bool

    @property
    def angle(self) -> float:
        return math.atan2(self.y1 - self.y0, self.x1 - self.x0)

    def y_at(self, x: float) -> float:
        return self.y0 + (self.y1 - self.y0) * (x - self.x0) / (self.x1 - self.x0)


@dataclass(frozen=True)
class Foothold:
    x: float          # ankle x
    y: float          # ankle y (on the surface)
    angle: float      # sole angle = surface slope
    surface: int      # index into surfaces()
    tread: bool
    margin: float     # distance from footprint to nearest riser edge (inf if none)


def surfaces(terrain: Terrain, tread_len: float) -> list[Surface]:
    prof = terrain.profile
    segs = [(a, b) for a, b in zip(prof, prof[1:]) if a != b]
    out = []
    for i, ((x0, y0), (x1, y1)) in enumerate(segs):
        if x0 == x1:
            continue  # riser
        left = i > 0 and segs[i - 1][0][0] == segs[i - 1][1][0]
        right = i + 1 < len(segs) and segs[i + 1][0][0] == segs[i + 1][1][0]
        tread = left and right and y0 == y1 and (x1 - x0) <= tread_len + 1e-9
        out.append(Surface(x0, y0, x1, y1, left, right, tread))
    return out


def candidates(terrain: Terrain, rcfg: RobotConfig, pcfg: PlannerConfig) -> list[Foothold]:
    """All valid footholds, sorted by x."""
    heel, toe = rcfg.heel_offset, rcfg.foot_len - rcfg.heel_offset
    m, g = pcfg.edge_margin, pcfg.grid
    out: list[Foothold] = []
    for k, s in enumerate(surfaces(terrain, terrain.config.stair_tread)):

        def margin(x: float) -> float:
            d = math.inf
            if s.edge_left:
                d = min(d, x - heel - s.x0)
            if s.edge_right:
                d = min(d, s.x1 - (x + toe))
            return d

        if s.tread:
            x = (s.x0 + s.x1) / 2 - (toe - heel) / 2       # foot centered on the tread
            if margin(x) >= m - 1e-9:
                out.append(Foothold(x, s.y_at(x), s.angle, k, True, margin(x)))
            continue
        lo = s.x0 + heel + (m if s.edge_left else 0.0)
        hi = s.x1 - toe - (m if s.edge_right else 0.0)
        i = math.ceil(lo / g - 1e-9)
        while i * g <= hi + 1e-9:
            x = round(i * g, 6)
            out.append(Foothold(x, s.y_at(x), s.angle, k, False, margin(x)))
            i += 1
    out.sort(key=lambda f: f.x)
    return out
