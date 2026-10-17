import math

import pytest

from gait_bot.config import PlannerConfig
from gait_bot.footholds import Foothold
from gait_bot.planner import FootstepPlanner
from gait_bot.robot import Robot
from gait_bot.swing import plan_swing
from gait_bot.terrain import generate

R = Robot()
P = PlannerConfig()
T0 = generate(0)   # seed 0: stairs with treads starting at x=8.15 (3 steps up)


def fh(t, x, tread=False):
    return Foothold(x, t.height_at(x), 0.0, 0, tread, math.inf)


def foot_gap(t, ankle, angle):
    c = R.cfg
    ux, uy = math.cos(angle), math.sin(angle)
    heel = (ankle[0] - c.heel_offset * ux, ankle[1] - c.heel_offset * uy)
    toe = (ankle[0] + (c.foot_len - c.heel_offset) * ux, ankle[1] + (c.foot_len - c.heel_offset) * uy)
    return t.vertical_clearance(heel, toe)


# ---------- terrain clearance helper ----------
def test_vertical_clearance_flat_and_riser():
    assert math.isclose(T0.vertical_clearance((0.5, 0.1), (1.0, 0.1)), 0.1)
    # segment crossing the first riser (x=8.15, top 0.2) at height 0.1 → penetrates by 0.1
    assert math.isclose(T0.vertical_clearance((8.0, 0.1), (8.3, 0.1)), -0.1)
    # vertical segment above the first tread
    assert math.isclose(T0.vertical_clearance((8.2, 0.5), (8.2, 0.3)), 0.1)


# ---------- single swings ----------
def test_flat_swing_uses_minimum_clearance():
    sw = plan_swing(T0, R, P, fh(T0, 0.5), fh(T0, 0.75), fh(T0, 1.0))
    assert sw.ok, sw.reason
    assert math.isclose(sw.h, P.swing_clearance, abs_tol=1e-9)
    assert math.dist(sw.samples[0].ankle, (0.5, 0.0)) < 1e-9
    assert math.dist(sw.samples[-1].ankle, (1.0, 0.0)) < 1e-9


def test_stair_swing():
    # tread k-1 (first tread) → over tread k → tread k+1, feet centered on treads
    a = 8.15 + 0.125 - 0.025
    S, M, T = (Foothold(a + 0.25 * k, 0.2 * (k + 1), 0.0, 0, True, 0.075) for k in range(3))
    sw = plan_swing(T0, R, P, S, M, T)
    assert sw.ok, sw.reason
    assert 0.1 < sw.h <= P.swing_h_max
    for p in sw.samples[1:-1]:
        ang = sw.samples[0].swing.foot_angle
        assert foot_gap(T0, p.ankle, ang) >= P.swing_clearance * math.sin(math.pi * p.s) - 1e-6


def test_impossible_cap_fails():
    a = 8.15 + 0.125 - 0.025
    S, M, T = (Foothold(a + 0.25 * k, 0.2 * (k + 1), 0.0, 0, True, 0.075) for k in range(3))
    sw = plan_swing(T0, R, PlannerConfig(swing_h_max=0.05), S, M, T)
    assert not sw.ok and sw.reason


# ---------- whole plans ----------
@pytest.fixture(scope="module")
def plans():
    out = {}
    for s in range(10):
        t = generate(s)
        out[s] = (t, FootstepPlanner(t, R).plan())
    return out


@pytest.mark.parametrize("seed", range(10))
def test_plan_swings(plans, seed):
    t, res = plans[seed]
    assert res.reached_goal, res.reason
    assert len(res.swings) == len(res.footholds) - 2
    for i, sw in enumerate(res.swings):
        assert sw.ok, sw.reason
        S, T = res.footholds[i], res.footholds[i + 2]
        assert math.isclose(sw.samples[0].ankle[0], S.x) and math.isclose(sw.samples[-1].ankle[0], T.x)
        for p in sw.samples:
            # FK of the solved angles reproduces both ankles
            M = res.footholds[i + 1]
            st = R.leg_points(p.hip, p.stance)["ankle"]
            sv = R.leg_points(p.hip, p.swing)
            assert math.isclose(st[0], M.x, abs_tol=1e-9) and math.isclose(st[1], M.y, abs_tol=1e-9)
            assert math.isclose(sv["ankle"][0], p.ankle[0], abs_tol=1e-9)
            # foot and shank never below ground
            assert foot_gap(t, p.ankle, p.swing.foot_angle) >= -1e-6
            assert t.vertical_clearance(sv["knee"], sv["ankle"]) >= -1e-6


@pytest.mark.parametrize("seed", range(10))
def test_hip_path_continuous(plans, seed):
    _, res = plans[seed]
    for a, b in zip(res.swings, res.swings[1:]):
        assert math.dist(a.samples[-1].hip, b.samples[0].hip) < 1e-6


def test_tight_cap_stops_at_stairs():
    res = FootstepPlanner(T0, R, PlannerConfig(swing_h_max=0.15)).plan()
    assert not res.reached_goal and "stuck" in res.reason
    assert all(sw.ok for sw in res.swings)
    assert res.footholds[-1].x < 8.5     # stopped at the stairs
