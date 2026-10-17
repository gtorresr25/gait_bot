import math

import pytest

from gait_bot.config import TerrainConfig
from gait_bot.terrain import generate

CFG = TerrainConfig()
SEEDS = range(200)


@pytest.mark.parametrize("seed", SEEDS)
def test_profile_bounds(seed):
    t = generate(seed)
    xs = [x for x, _ in t.profile]
    assert t.profile[0] == (0.0, 0.0)
    assert t.profile[-1] == (CFG.world_length, 0.0)
    assert all(a <= b + 1e-9 for a, b in zip(xs, xs[1:])), "x must be non-decreasing"
    assert all(y >= -1e-9 for _, y in t.profile)


@pytest.mark.parametrize("seed", SEEDS)
def test_flat_start_and_end(seed):
    t = generate(seed)
    for f in t.features:
        assert f.x_start >= CFG.flat_start
        assert f.x_end <= CFG.world_length - CFG.flat_end
    assert t.height_at(CFG.spawn_x) == 0.0


@pytest.mark.parametrize("seed", SEEDS)
def test_gaps_between_features(seed):
    t = generate(seed)
    for a, b in zip(t.features, t.features[1:]):
        assert b.x_start - a.x_end >= CFG.gap_min - 1e-9


@pytest.mark.parametrize("seed", SEEDS)
def test_feature_geometry(seed):
    t = generate(seed)
    for f in t.features:
        pts = f.points
        assert pts[0][1] == 0.0 and abs(pts[-1][1]) < 1e-9, "features are net-zero"
        segs = list(zip(pts, pts[1:]))
        if f.kind == "stairs":
            for (x0, y0), (x1, y1) in segs:
                dx, dy = x1 - x0, y1 - y0
                if dx == 0:
                    assert abs(abs(dy) - CFG.stair_riser) < 1e-9
                else:
                    assert dy == 0
                    assert abs(dx - CFG.stair_tread) < 1e-9 or CFG.top_min - 1e-9 <= dx <= CFG.top_max + 1e-9
        else:
            (x0, y0), (x1, y1) = segs[0]
            assert math.isclose(math.degrees(math.atan2(y1 - y0, x1 - x0)), CFG.ramp_angle_deg)
            (x0, y0), (x1, y1) = segs[-1]
            assert math.isclose(math.degrees(math.atan2(y0 - y1, x1 - x0)), CFG.ramp_angle_deg)


def test_deterministic():
    assert generate(42).profile == generate(42).profile


def test_height_at_ramp():
    t = generate(0)
    for f in t.features:
        if f.kind == "ramp":
            (x0, _), (x1, y1) = f.points[0], f.points[1]
            mid = (x0 + x1) / 2
            assert math.isclose(t.height_at(mid), y1 / 2)
            return
    pytest.skip("seed 0 has no ramp")
