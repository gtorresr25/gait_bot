# Biped robot

## Decisions (2026-10-17)
- Planar biped. Every part is a **line segment with its own mass**, modeled as a uniform rod (mass at the midpoint).

| part  | length (m) | mass (kg) | count |
|-------|-----------:|----------:|------:|
| torso | 0.60       | 5.0       | 1     |
| thigh | 0.30       | 1.0       | 2     |
| shank | 0.30       | 0.6       | 2     |
| foot  | 0.10       | 0.2       | 2     |

Scaled up 20% from the original 0.50/0.25/0.25/0.08, rounded to whole cm (2026-10-17). Masses are unchanged.
Total mass is 8.6 kg.

- **Human-like leg:** the knee bends forward. The ankle sits 1/4 of the foot length from the heel, so the heel is 0.025 m behind the ankle and the toe is 0.075 m ahead.
- The foot lies flat on the surface during stance (parallel to the ramp slope) and stays horizontal during swing.
- The torso stays upright for now. It rises 0.6 m straight up from the hip, and both legs attach at the hip.
- Simulation (kinematic vs physics) is **deferred**. For now we only need FK/IK and the center of mass.
- The camera **follows the robot** (smoothed). Arrow keys temporarily override it, and F toggles follow mode.

## Kinematics
- Max reach: |hip − ankle| ≤ L_max = 0.98 · 0.6 = 0.588 m. Keep a small margin, because a fully straight knee is a singularity.
- Min reach: from the knee flexion limit (proposed 0–140°), so |hip − ankle| ≥ 2·0.30·sin(20°) ≈ 0.205 m.
- IK: analytic two-link solution (law of cosines), choosing the solution with the knee forward (+x).
- Nominal standing hip height is 0.54 m.

## Reach facts that shape the planner
- On flat ground with the hip at 0.54 m, the farthest step is about 0.46 m (above the 0.40 max step).
- On stairs (one foot per tread), the swing foot travels from tread k−1 to tread k+1: **0.50 m forward and 0.40 m up**. The hip has to move with the feet. A fixed hip height can't reach both.
- The 0.4 m arc-height cap rarely binds. Leg reach usually limits swing height first.

- **Joint limits (decided):** hip −30°…+110°, knee 0°…140°. Collision checks cover the foot and shank.
- Drawing: left leg blue (drawn behind), right leg red, black torso, green COM ring with a drop line to the ground.

## Implementation (milestone 1 done, 2026-10-17)
- `gait_bot/robot.py`: `Robot.ik(hip, ankle)` → `IKResult(ok, angles, reason)`, `leg_points`, `segments`, `com`,
  `foothold(terrain, x)` (ankle and sole angle), `stand(terrain, x_rear, x_front, hip_dy)` (double-support pose).
- Viewer sandbox: click to place the rear foot, `[`/`]` change the step, `W`/`S` move the hip. IK failures show in the HUD.
- Observed: standing on stairs (feet on adjacent treads) gives a rear knee of about 23° and a front knee of about 106°.
  (original size). After the 20% scale-up: rear 23°, front 98°, hip 0.77 m. The front knee has 42° of margin left.
- Flat ground with a 0.25 m step: about 45° knee bend, which is fairly crouched. Not a priority to tune.
