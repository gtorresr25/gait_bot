import math

import pytest

from gait_bot.robot import LegAngles, Pose, Robot
from gait_bot.terrain import generate

R = Robot()


def test_total_mass():
    assert math.isclose(R.total_mass, 8.6)


def test_reach_limits():
    assert math.isclose(R.max_reach, 0.588)
    assert math.isclose(R.min_reach, 0.205, abs_tol=1e-3)   # knee at 140°


@pytest.mark.parametrize("ankle", [(0.0, -0.55), (0.2, -0.5), (-0.1, -0.3), (0.4, -0.2), (0.0, -0.25)])
def test_ik_fk_roundtrip(ankle):
    hip = (0.0, 0.0)
    res = R.ik(hip, ankle)
    assert res.ok, res.reason
    p = R.leg_points(hip, res.angles)
    assert math.isclose(p["ankle"][0], ankle[0], abs_tol=1e-9)
    assert math.isclose(p["ankle"][1], ankle[1], abs_tol=1e-9)


def test_knee_forward():
    hip, ankle = (0.0, 0.0), (0.0, -0.4)
    p = R.leg_points(hip, R.ik(hip, ankle).angles)
    assert p["knee"][0] > 0.0


def test_ik_failures():
    assert "too far" in R.ik((0, 0), (0, -0.7)).reason
    assert "too close" in R.ik((0, 0), (0, -0.1)).reason
    assert "hip limit" in R.ik((0, 0), (-0.5, -0.1)).reason    # leg far behind


def test_foot_geometry():
    p = R.leg_points((0, 0.54), LegAngles(0.0, 0.0, 0.0))
    assert math.isclose(p["ankle"][0] - p["heel"][0], 0.025)
    assert math.isclose(p["toe"][0] - p["ankle"][0], 0.075)


def test_com_symmetric_straight_legs():
    q = LegAngles(0.0, 0.0)
    cx, cy = R.com(Pose((0.0, 0.54), q, q))
    # foot is asymmetric about the ankle (toe sticks forward) → COM slightly ahead
    assert 0.0 < cx < 0.01
    assert 0.54 < cy < 0.54 + 0.30


def test_stand_at_spawn():
    t = generate(0)
    pose, res, hip = R.stand(t, 0.875, 1.125)
    assert pose is not None, [r.reason for r in res]
    assert math.isclose(hip[0], 1.0)
    for q in (pose.left, pose.right):
        p = R.leg_points(pose.hip, q)
        assert math.isclose(p["ankle"][1], 0.0, abs_tol=1e-9)
