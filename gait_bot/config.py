"""World and terrain parameters. All lengths in meters, angles in degrees."""
from dataclasses import dataclass


@dataclass(frozen=True)
class TerrainConfig:
    world_length: float = 20.0
    spawn_x: float = 1.0           # robot starts here
    flat_start: float = 2.0        # ground is guaranteed flat on [0, flat_start]
    flat_end: float = 1.0          # ground is guaranteed flat on the last flat_end meters

    gap_min: float = 1.0           # flat ground between features
    gap_max: float = 2.0

    # Stairs: go up n steps, flat top, come down n steps
    stair_riser: float = 0.20
    stair_tread: float = 0.25
    stair_steps_min: int = 2
    stair_steps_max: int = 4

    # Ramps: slope up, flat top, slope down
    ramp_angle_deg: float = 10.0
    ramp_rise_min: float = 0.2
    ramp_rise_max: float = 0.6

    top_min: float = 0.5           # flat top of every feature
    top_max: float = 1.0

    stair_probability: float = 0.5  # otherwise ramp


@dataclass(frozen=True)
class RobotConfig:
    """Planar biped: every part is a uniform rod (mass at its midpoint)."""
    torso_len: float = 0.60
    thigh_len: float = 0.30
    shank_len: float = 0.30
    foot_len: float = 0.10
    heel_offset: float = 0.025     # ankle sits 1/4 of the foot from the heel
                                   # → heel 0.025 behind ankle, toe 0.075 ahead

    torso_mass: float = 5.0
    thigh_mass: float = 1.0
    shank_mass: float = 0.6
    foot_mass: float = 0.2

    # Joint limits (degrees). Hip: thigh angle from straight down, + = forward.
    # Knee: flexion, 0 = straight, + = shank folds backward (human knee).
    hip_min_deg: float = -30.0
    hip_max_deg: float = 110.0
    knee_min_deg: float = 0.0
    knee_max_deg: float = 140.0

    reach_margin: float = 0.98     # never fully straighten the knee (singular)
    nominal_hip_height: float = 0.54


@dataclass(frozen=True)
class PlannerConfig:
    step_nom: float = 0.25         # sweet-spot step (foothold to next foothold)
    step_max: float = 0.40
    step_min: float = 0.05
    horizon: int = 4               # plan 4 footholds, commit 1
    goal_margin: float = 0.5       # goal x = world_length - goal_margin

    grid: float = 0.01             # candidate spacing on flat/ramp surfaces
    edge_margin: float = 0.02      # min distance from foot to a riser edge
    edge_soft: float = 0.05        # below this distance, edge cost ramps up

    # cost weights (w_len / w_goal sets the bias: optimum step ≈ step_nom + w_goal / (2 w_len))
    w_len: float = 200.0
    w_height: float = 1.0
    w_edge: float = 1.0
    w_goal: float = 4.0            # per meter left to the goal after the horizon

    swing_clearance: float = 0.03  # used from milestone 3
    swing_h_max: float = 0.40
