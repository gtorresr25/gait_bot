"""Footstep planner: 4-step dynamic program, receding horizon (plan 4, commit 1).

The foothold sequence alternates L, R, L, ... Each new foothold is where the rear
(swing) foot lands, ahead of the stance foot. Transition costs depend only on the
pair (stance foothold → new foothold), so DP over (step index, foothold) is exact.

Swing feasibility depends on three footholds (S → over M → T), which a pairwise DP
can't express. So after each DP solve, every swing along the chosen path is checked;
a failing pair (M, T) is banned for this re-plan and the DP runs again. The first
swing is checked with the real rear foot, so committed steps are always feasible.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

from .config import PlannerConfig
from .footholds import Foothold, candidates
from .robot import Robot
from .swing import Swing, plan_swing
from .terrain import Terrain

INF = math.inf


@dataclass
class PlanResult:
    footholds: list[Foothold]                 # committed sequence, starts with the 2 initial feet
    horizons: list[list[Foothold]] = field(default_factory=list)
    # horizons[i] = the 4-step plan made while standing on footholds[i], footholds[i+1]
    swings: list[Swing] = field(default_factory=list)
    # swings[i] = footholds[i] swings over footholds[i+1] to footholds[i+2]
    horizon_swings: list[list[Swing]] = field(default_factory=list)
    reached_goal: bool = False
    reason: str = ""

    def side(self, i: int) -> str:
        return "L" if i % 2 == 0 else "R"


class FootstepPlanner:
    def __init__(self, terrain: Terrain, robot: Robot, cfg: PlannerConfig | None = None):
        self.terrain = terrain
        self.robot = robot
        self.cfg = cfg or PlannerConfig()
        self.goal_x = terrain.length - self.cfg.goal_margin
        self.cands = [c for c in candidates(terrain, robot.cfg, self.cfg) if c.x <= self.goal_x]
        self.xs = [c.x for c in self.cands]
        self.tread_xs = [c.x for c in self.cands if c.tread]
        self._cost: dict[tuple[int, int], float] = {}
        self._swing: dict[tuple[int, int, int], Swing] = {}
        self.max_replans = 30    # DP re-solves per step before the single-step fallback

    # ---------- transitions ----------
    def successors(self, i: int) -> range:
        x = self.xs[i]
        lo = bisect.bisect_right(self.xs, x + self.cfg.step_min - 1e-9)
        hi = bisect.bisect_right(self.xs, x + self.cfg.step_max + 1e-9)
        return range(lo, hi)

    def feasible(self, a: Foothold, b: Foothold) -> bool:
        # one foot per stair: no tread may be skipped between a and b
        k = bisect.bisect_right(self.tread_xs, a.x + 1e-9)
        if k < len(self.tread_xs) and self.tread_xs[k] < b.x - 1e-9:
            return False
        # both feet reachable from one upright hip (double support)
        pose, _, _ = self.robot.stand(self.terrain, a.x, b.x)
        return pose is not None

    def cost(self, i: int, j: int) -> float:
        key = (i, j)
        if key not in self._cost:
            a, b = self.cands[i], self.cands[j]
            c = self.cfg
            if not self.feasible(a, b):
                v = INF
            else:
                edge = max(0.0, (c.edge_soft - b.margin) / c.edge_soft) if b.margin < INF else 0.0
                v = (c.w_len * (b.x - a.x - c.step_nom) ** 2
                     + c.w_height * abs(b.y - a.y)
                     + c.w_edge * edge)
            self._cost[key] = v
        return self._cost[key]

    def swing(self, a: int, b: int, c: int) -> Swing:
        key = (a, b, c)
        if key not in self._swing:
            self._swing[key] = plan_swing(self.terrain, self.robot, self.cfg,
                                          self.cands[a], self.cands[b], self.cands[c])
        return self._swing[key]

    def terminal(self, j: int) -> float:
        return self.cfg.w_goal * max(0.0, self.goal_x - self.xs[j])

    # ---------- search ----------
    def plan_horizon(self, rear: int, front: int) -> list[int]:
        """Best sequence of up to `horizon` new footholds from stance (rear, front) whose swings
        are all feasible. Empty if none."""
        banned: set[tuple[int, int]] = set()
        for _ in range(self.max_replans):
            path = self._dp(front, banned)
            chain = [rear, front] + path
            bad = next((k for k in range(len(path))
                        if not self.swing(chain[k], chain[k + 1], chain[k + 2]).ok), None)
            if bad is None:
                return path
            banned.add((chain[bad + 1], chain[bad + 2]))
        # Fallback: best single step whose swing works (exact). Only "stuck" if there is none.
        ok = [j for j in self.successors(front)
              if self.cost(front, j) < INF and self.swing(rear, front, j).ok]
        if not ok:
            return []
        return [min(ok, key=lambda j: self.cost(front, j) + self.terminal(j))]

    def _dp(self, front: int, banned: set[tuple[int, int]]) -> list[int]:
        layers: list[dict[int, tuple[float, int | None]]] = [{front: (0.0, None)}]
        for _ in range(self.cfg.horizon):
            nxt: dict[int, tuple[float, int | None]] = {}
            for i, (ci, _) in layers[-1].items():
                if self.at_goal(i):   # done: stand still for free instead of shuffling
                    if i not in nxt or ci < nxt[i][0]:
                        nxt[i] = (ci, i)
                    continue
                for j in self.successors(i):
                    if (i, j) in banned:
                        continue
                    c = ci + self.cost(i, j)
                    if c < INF and (j not in nxt or c < nxt[j][0]):
                        nxt[j] = (c, i)
            if not nxt:
                break
            layers.append(nxt)
        if len(layers) == 1:
            return []
        last = layers[-1]
        j = min(last, key=lambda k: last[k][0] + self.terminal(k))
        path = []
        for layer in reversed(layers[1:]):
            path.append(j)
            j = layer[j][1]
        path = path[::-1]
        # drop the "stand still" repeats after reaching the goal
        return [k for n, k in enumerate(path) if n == 0 or k != path[n - 1]]

    def at_goal(self, i: int) -> bool:
        return self.xs[i] >= self.goal_x - self.cfg.step_min

    def nearest(self, x: float) -> int:
        return min(range(len(self.cands)), key=lambda i: abs(self.xs[i] - x))

    def plan(self, x_start: float | None = None, max_steps: int = 500) -> PlanResult:
        """Receding horizon from a double-support stance centered at x_start (default: spawn)."""
        x0 = self.terrain.config.spawn_x if x_start is None else x_start
        half = self.cfg.step_nom / 2
        idx = [self.nearest(x0 - half), self.nearest(x0 + half)]
        res = PlanResult([self.cands[i] for i in idx])
        for _ in range(max_steps):
            if self.at_goal(idx[-1]):
                res.reached_goal = True
                return res
            path = self.plan_horizon(idx[-2], idx[-1])
            if not path:
                res.reason = f"stuck at x={self.xs[idx[-1]]:.2f}: no feasible next foothold"
                return res
            chain = idx[-2:] + path
            res.horizons.append([self.cands[j] for j in path])
            res.horizon_swings.append([self.swing(*chain[k:k + 3]) for k in range(len(path))])
            res.swings.append(res.horizon_swings[-1][0])
            idx.append(path[0])
            res.footholds.append(self.cands[path[0]])
        res.reason = "max steps exceeded"
        return res
