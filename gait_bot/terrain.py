"""2D terrain: a ground profile polyline from x=0 to x=world_length.

The profile is a list of (x, y) vertices, x non-decreasing. Stair risers are
vertical segments (two consecutive vertices with the same x). Base ground is y=0.
"""
from __future__ import annotations

import bisect
import math
import random
from dataclasses import dataclass, field
from functools import cached_property

from .config import TerrainConfig


@dataclass(frozen=True)
class Feature:
    kind: str          # "stairs" or "ramp"
    x_start: float
    x_end: float
    height: float      # height of the flat top above base ground
    points: tuple[tuple[float, float], ...]


@dataclass
class Terrain:
    seed: int
    config: TerrainConfig
    features: list[Feature] = field(default_factory=list)
    profile: list[tuple[float, float]] = field(default_factory=list)

    @property
    def length(self) -> float:
        return self.config.world_length

    @cached_property
    def _xs(self) -> list[float]:
        return [x for x, _ in self.profile]

    def height_at(self, x: float) -> float:
        """Ground height at x. On a stair riser, returns the height just to the right."""
        if x <= 0.0:
            return self.profile[0][1]
        if x >= self.length:
            return self.profile[-1][1]
        i = bisect.bisect_right(self._xs, x) - 1      # last vertex with x_i <= x
        (x0, y0), (x1, y1) = self.profile[i], self.profile[i + 1]
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

    def ground_max(self, x: float) -> float:
        """Highest ground at x: on a riser this is the top corner."""
        return max(self.height_at(x), self.height_at(x - 1e-9))

    def vertical_clearance(self, p0: tuple[float, float], p1: tuple[float, float]) -> float:
        """Min vertical gap between segment p0-p1 and the ground under it (negative = penetrating).
        Exact for a polyline terrain: the gap is piecewise linear in x, so its minimum is at a
        segment endpoint or at a terrain vertex inside the segment's x-range."""
        (xa, ya), (xb, yb) = sorted((p0, p1))
        if xb - xa < 1e-12:   # vertical segment: its lowest point
            return min(ya, yb) - self.ground_max(xa)
        xs = self._xs
        checks = [xa, xb] + xs[bisect.bisect_right(xs, xa):bisect.bisect_left(xs, xb)]
        return min(ya + (yb - ya) * (x - xa) / (xb - xa) - self.ground_max(x) for x in checks)

    def feature_at(self, x: float) -> Feature | None:
        for f in self.features:
            if f.x_start <= x <= f.x_end:
                return f
        return None


def _stairs(x: float, steps: int, top: float, cfg: TerrainConfig) -> list[tuple[float, float]]:
    r, t = cfg.stair_riser, cfg.stair_tread
    pts = [(x, 0.0)]
    y = 0.0
    for i in range(steps):              # up
        y += r
        pts.append((x, y))
        x += top if i == steps - 1 else t
        pts.append((x, y))
    for i in range(steps):              # down
        y -= r
        pts.append((x, y))
        if i < steps - 1:
            x += t
            pts.append((x, y))
    return pts


def _ramp(x: float, rise: float, top: float, cfg: TerrainConfig) -> list[tuple[float, float]]:
    run = rise / math.tan(math.radians(cfg.ramp_angle_deg))
    return [(x, 0.0), (x + run, rise), (x + run + top, rise), (x + 2 * run + top, 0.0)]


def generate(seed: int, cfg: TerrainConfig | None = None) -> Terrain:
    """Place random up-and-down stairs/ramps separated by flat ground."""
    cfg = cfg or TerrainConfig()
    rng = random.Random(seed)
    terrain = Terrain(seed=seed, config=cfg)
    limit = cfg.world_length - cfg.flat_end

    x = cfg.flat_start
    misses = 0
    while misses < 20:   # keep trying smaller features before giving up
        top = round(rng.uniform(cfg.top_min, cfg.top_max), 2)
        if rng.random() < cfg.stair_probability:
            steps = rng.randint(cfg.stair_steps_min, cfg.stair_steps_max)
            pts = _stairs(x, steps, top, cfg)
            kind, height = "stairs", steps * cfg.stair_riser
        else:
            # snap rise to 5 cm so heights stay readable
            rise = round(rng.uniform(cfg.ramp_rise_min, cfg.ramp_rise_max) / 0.05) * 0.05
            pts = _ramp(x, rise, top, cfg)
            kind, height = "ramp", rise
        x_end = pts[-1][0]
        if x_end > limit:
            misses += 1
            continue
        terrain.features.append(Feature(kind, x, x_end, height, tuple(pts)))
        x = x_end + rng.uniform(cfg.gap_min, cfg.gap_max)

    profile = [(0.0, 0.0)]
    for f in terrain.features:
        profile.extend(f.points)
    profile.append((cfg.world_length, 0.0))
    terrain.profile = profile
    return terrain
