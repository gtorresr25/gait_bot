import math

import pytest

from gait_bot.planner import FootstepPlanner
from gait_bot.playback import Playback
from gait_bot.robot import Robot
from gait_bot.swing import N_SAMPLES, _foot_ends
from gait_bot.terrain import generate

R = Robot()
SEEDS = range(5)


@pytest.fixture(scope="module")
def plans():
    return {s: (generate(s), FootstepPlanner(generate(s), R).plan()) for s in SEEDS}


def angles(pose):
    return [pose.left.q_hip, pose.left.q_knee, pose.left.foot_angle,
            pose.right.q_hip, pose.right.q_knee, pose.right.foot_angle]


@pytest.mark.parametrize("seed", SEEDS)
def test_stance_foot_planted_and_no_clipping(plans, seed):
    t, res = plans[seed]
    for sw in res.swings:
        for k in range(201):
            p = sw.state_at(k / 200)
            ankle = R.leg_points(p.hip, p.stance)["ankle"]
            assert math.dist(ankle, (sw.M.x, sw.M.y)) < 1e-9
            assert t.vertical_clearance(*_foot_ends(R, p.ankle, p.swing.foot_angle)) >= -1e-6
            for q in (p.swing, p.stance):
                lp = R.leg_points(p.hip, q)
                assert t.vertical_clearance(lp["knee"], lp["ankle"]) >= -1e-6


@pytest.mark.parametrize("seed", SEEDS)
def test_continuous_across_steps(plans, seed):
    _, res = plans[seed]
    for i in range(len(res.swings) - 1):
        end = res.swings[i].pose_at(1.0, res.side(i))
        start = res.swings[i + 1].pose_at(0.0, res.side(i + 1))
        assert math.dist(end.hip, start.hip) < 1e-9
        assert max(abs(a - b) for a, b in zip(angles(end), angles(start))) < 1e-9


@pytest.mark.parametrize("seed", SEEDS)
def test_pose_at_matches_verified_samples(plans, seed):
    _, res = plans[seed]
    for sw in res.swings:
        for smp in sw.samples:
            p = sw.state_at(smp.s)
            assert p == smp


def test_playback_timing_and_end(plans):
    _, res = plans[0]
    pb = Playback(res)
    total = sum(pb.durations) + pb.dwell * (len(pb.durations) - 1)
    pb.toggle()
    pb.advance(total - 1e-6)
    assert not pb.at_end and pb.step_idx == pb.last_idx - 1
    pb.advance(1e-3)
    assert pb.at_end and not pb.playing
    assert pb.pose(R, generate(0)) is not None


def test_playback_frame_rate_independent(plans):
    _, res = plans[0]
    a, b = Playback(res), Playback(res)
    a.toggle(), b.toggle()
    a.advance(3.0)
    for _ in range(180):
        b.advance(3.0 / 180)
    assert a.step_idx == b.step_idx
    assert math.isclose(a.s, b.s, abs_tol=1e-9) and math.isclose(a.dwell_left, b.dwell_left, abs_tol=1e-9)


def test_scrub_crosses_boundaries(plans):
    _, res = plans[0]
    pb = Playback(res)
    pb.scrub(N_SAMPLES + 3)
    assert pb.step_idx == 1 and math.isclose(pb.s, 3 / N_SAMPLES)
    pb.scrub(-5)
    assert pb.step_idx == 0 and math.isclose(pb.s, 1 - 2 / N_SAMPLES)
    pb.scrub(-1000)
    assert pb.step_idx == 0 and pb.s == 0.0
    pb.scrub(10 ** 6)
    assert pb.at_end


def test_toggle_at_end_restarts(plans):
    _, res = plans[0]
    pb = Playback(res)
    pb.jump_to(pb.last_idx)
    pb.toggle()
    assert pb.playing and pb.step_idx == 0
