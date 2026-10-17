"""Kinematic playback of a plan: a clock over (step index, swing phase s).

Timing is a viewer choice (the planner is purely spatial): each swing lasts
foot_travel / foot_speed seconds, followed by a short double-support dwell.
"""
from __future__ import annotations

from .planner import PlanResult
from .robot import Pose, Robot
from .swing import N_SAMPLES
from .terrain import Terrain

FOOT_SPEED = 0.8       # m/s along the swing foot's path
DWELL_S = 0.1          # both feet down between swings
MIN_SWING_S = 0.2


def stance_pose(robot: Robot, terrain: Terrain, plan: PlanResult, i: int):
    """Double-support pose on footholds i (rear) and i+1 (front), legs assigned by side."""
    rear, front = plan.footholds[i], plan.footholds[i + 1]
    pose, results, hip = robot.stand(terrain, rear.x, front.x)
    if pose and plan.side(i) == "R":          # stand() puts the left leg on the rear foot
        pose = Pose(pose.hip, pose.right, pose.left)
    return pose, results, hip


class Playback:
    def __init__(self, plan: PlanResult, foot_speed: float = FOOT_SPEED, dwell: float = DWELL_S):
        self.plan = plan
        self.dwell = dwell
        self.durations = [max(MIN_SWING_S, sw.foot_travel() / foot_speed) for sw in plan.swings]
        self.step_idx = 0         # swinging footholds[i] over [i+1] to [i+2]
        self.s = 0.0              # phase within that swing
        self.dwell_left = 0.0
        self.speed = 1.0          # playback multiplier
        self.playing = False

    @property
    def last_idx(self) -> int:
        return len(self.plan.swings)

    @property
    def at_end(self) -> bool:
        return self.step_idx >= self.last_idx

    @property
    def in_dwell(self) -> bool:
        return self.dwell_left > 0.0

    def toggle(self) -> None:
        if self.at_end:
            self.jump_to(0)
        self.playing = not self.playing

    def jump_to(self, i: int) -> None:
        self.step_idx = min(max(i, 0), self.last_idx)
        self.s, self.dwell_left = 0.0, 0.0

    def scrub(self, frames: int) -> None:
        """Move by whole sample frames (1/N_SAMPLES of a swing), crossing step boundaries."""
        self.dwell_left = 0.0
        pos = self.step_idx + self.s + frames / N_SAMPLES
        pos = min(max(pos, 0.0), float(self.last_idx))
        self.step_idx = min(int(pos + 1e-9), self.last_idx)
        self.s = 0.0 if self.at_end else max(0.0, pos - self.step_idx)

    def advance(self, dt: float) -> None:
        if not self.playing:
            return
        t = dt * self.speed
        while t > 0.0:
            if self.at_end:
                self.playing = False
                return
            if self.dwell_left > 0.0:
                use = min(t, self.dwell_left)
                self.dwell_left -= use
                t -= use
                continue
            dur = self.durations[self.step_idx]
            remaining = (1.0 - self.s) * dur
            if t < remaining:
                self.s += t / dur
                return
            t -= remaining
            self.step_idx += 1
            self.s = 0.0
            self.dwell_left = self.dwell

    def pose(self, robot: Robot, terrain: Terrain) -> Pose | None:
        if self.at_end:
            pose, _, _ = stance_pose(robot, terrain, self.plan, self.step_idx)
            return pose
        return self.plan.swings[self.step_idx].pose_at(self.s, self.plan.side(self.step_idx))
