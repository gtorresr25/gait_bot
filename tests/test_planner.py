import math

import pytest

from gait_bot.config import PlannerConfig
from gait_bot.footholds import candidates, surfaces
from gait_bot.planner import FootstepPlanner
from gait_bot.robot import Robot
from gait_bot.terrain import generate

R = Robot()
P = PlannerConfig()
SEEDS = range(20)
HEEL, TOE = R.cfg.heel_offset, R.cfg.foot_len - R.cfg.heel_offset


@pytest.fixture(scope="module")
def plans():
    out = {}
    for s in SEEDS:
        t = generate(s)
        out[s] = (t, FootstepPlanner(t, R).plan())
    return out


def test_candidates_fit_on_one_surface():
    t = generate(1)
    surf = surfaces(t, t.config.stair_tread)
    for c in candidates(t, R.cfg, P):
        s = surf[c.surface]
        assert s.x0 - 1e-9 <= c.x - HEEL and c.x + TOE <= s.x1 + 1e-9
        assert c.margin >= P.edge_margin - 1e-9
        assert math.isclose(c.y, t.height_at(c.x), abs_tol=1e-9)


def test_tread_candidates_centered():
    t = generate(0)
    surf = surfaces(t, t.config.stair_tread)
    treads = [c for c in candidates(t, R.cfg, P) if c.tread]
    assert treads
    for c in treads:
        s = surf[c.surface]
        foot_center = c.x + (TOE - HEEL) / 2
        assert math.isclose(foot_center, (s.x0 + s.x1) / 2)


@pytest.mark.parametrize("seed", SEEDS)
def test_reaches_goal(plans, seed):
    t, res = plans[seed]
    assert res.reached_goal, res.reason


@pytest.mark.parametrize("seed", SEEDS)
def test_step_lengths(plans, seed):
    _, res = plans[seed]
    for a, b in zip(res.footholds, res.footholds[1:]):
        assert P.step_min - 1e-9 < b.x - a.x <= P.step_max + 1e-9


@pytest.mark.parametrize("seed", SEEDS)
def test_every_tread_used_once(plans, seed):
    t, res = plans[seed]
    goal = t.length - P.goal_margin
    tread_xs = [c.x for c in candidates(t, R.cfg, P) if c.tread and c.x <= goal]
    used = [f.x for f in res.footholds if f.tread]
    assert used == tread_xs


@pytest.mark.parametrize("seed", SEEDS)
def test_double_support_reachable(plans, seed):
    t, res = plans[seed]
    for a, b in zip(res.footholds, res.footholds[1:]):
        pose, results, _ = R.stand(t, a.x, b.x)
        assert pose is not None, [r.reason for r in results]


@pytest.mark.parametrize("seed", SEEDS)
def test_horizons(plans, seed):
    _, res = plans[seed]
    assert len(res.horizons) == len(res.footholds) - 2
    for i, h in enumerate(res.horizons):
        assert 1 <= len(h) <= P.horizon
        assert h[0] == res.footholds[i + 2]     # commit the first planned step
