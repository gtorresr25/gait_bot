# Step planner

Output: **foot positions and spatial trajectories** for the swing foot and hip. No timing.
Swing trajectories are parameterized by a phase s ∈ [0, 1], not by time.

## Decisions (2026-10-17)
- One foot per stair, alternating L/R. In 2D both feet share the same plane, so L/R is only a label.
- **Receding horizon:** plan 4 footholds, commit 1, re-plan from the new stance.
- Swing path is an **arc** with a maximum apex height of **0.4 m**. It must not collide with the terrain.

## Pipeline per re-plan
```
stance footholds (rear, front)
   └─ 1. candidate footholds ahead (valid surfaces)
   └─ 2. DP over 4 steps → best foothold sequence (pairwise costs)
          each transition is checked by 3. swing arc and 4. hip path feasibility
   └─ commit step 1 → emit (foothold, swing curve, hip curve)
```

### 1. Candidate footholds
- Sample ankle x on a 1 cm grid from the stance foot out to max reach (about 0.5 m).
- Valid if the whole foot [x−0.025, x+0.075] lies on **one** non-vertical segment (flat, ramp, or tread) and is at least 2 cm from any riser or edge.
- **Stairs:** only one candidate per tread, with the foot centered on it (the ankle 0.100 m from the tread start). A transition onto stairs must land on the next tread, with no skipping and no doubling up.
- Foot angle = surface slope (0° or ±10°).

### 2. Footstep search (dynamic programming)
- Costs depend only on consecutive pairs, so DP over (step index, foothold) is exact: O(4 · K²) with K ≈ 50 candidates.
- Transition cost for a → b:
  `w_len·(Δx − ℓ_nom)² + w_h·|Δy| + w_edge/edge_margin(b) + w_swing·apex_h(a→b)`
- Terminal cost: `(x_goal − x_4) / ℓ_nom`. Without it the planner would happily take short steps.
- Infeasible transitions (reach, swing collision, or swing height > 0.4) cost infinity.

### 3. Swing arc (from rear foothold S over stance to target T)
Parametric curve in phase s ∈ [0, 1], with one free parameter h:
```
x(s) = S.x + (T.x − S.x) · σ(s)        σ(s) = 3s² − 2s³   (smoothstep)
y(s) = S.y + (T.y − S.y) · σ(s) + h · sin(πs)
```
- **Fix (milestone 3):** the baseline's y also uses σ(s), not s. With s, the baseline sagged below the straight S→T line
  near touch-down when going uphill (x was almost at T while y lagged), so ramp entries needed h ≈ 0.05 instead of 0.03.
  The trade-off is that stair swings now need slightly more bump (up to 0.27 m vs 0.24 m).
- The baseline is the straight line S→T. A sine "bump" of height h is added on top; sin(πs) is 0 at both ends and peaks at s=½.
  (A parabola 4s(1−s) would be nearly the same; sine has slightly steeper ends.)
- smoothstep has σ'(0) = σ'(1) = 0, so x barely moves at lift-off and touch-down while y moves.
  The foot rises and lands almost **vertically**, which keeps it off stair nosings.
- Ground under the foot: G(x) = max terrain height over the footprint [x−0.025, x+0.075].
- Collision-free condition: y(s) ≥ G(x(s)) + c·sin(πs). The clearance c (3 cm) fades to 0 at the ends,
  where the foot touches down. (Applying c at the ends makes h blow up to about 2 m. Bug found 2026-10-17.)
- **Minimal h in closed form:** h = max over s of [ (G(x(s)) − base(s)) / sin(πs) ] + c, sampled at about 100 values of s.
  If h > 0.4 (cap) or IK fails along the path, the transition is invalid.
- Worked example, stair swing from tread k−1 to k+1 (0.5 m forward, 0.4 m up), c = 3 cm:
  smoothstep x needs **h = 0.20 m**, an apex 0.24 m above the stance tread. A linear x would need h ≈ 0.28 m.
  At that apex the swing knee is near full flexion (min reach is 0.205 m), so the hip path must allow for it.

### 4. Hip path (needed for IK feasibility)
- Torso upright. The hip x moves from the midpoint of (S, stance) to the midpoint of (stance, T).
- Hip y = min(nominal, highest y that keeps both ankles within L_max), and it must stay ≥ the min-reach bound.
- Check IK for the stance and swing leg at every sampled s (joint limits, plus shank collision if enabled).

## Implementation status (milestone 2 done, 2026-10-17)
- `footholds.py`: `surfaces()` and `candidates()`. Treads are horizontal segments between two risers with length ≤ tread; everything else (including stair-top platforms) uses a 1 cm grid.
- `planner.py`: `FootstepPlanner(terrain, robot).plan()` → `PlanResult(footholds, horizons, reached_goal, reason)`.
  - Search over (step index, foothold). Transitions are cached per (i, j).
    Feasibility for now means: step in (0.05, 0.40], no skipped tread, and a double-support pose exists (`Robot.stand`).
  - Once a plan reaches the goal (x ≥ 19.5 − 0.05) it may stand still for free. Without this, the fixed 4-step horizon caused tiny 5 cm shuffling steps at the end.
  - Weights: w_len 200, w_goal 4/m, so the best step ≈ 0.25 + 4/400 = 0.26 m. Observed steps are 0.21–0.30 m across seeds.
  - About 0.5 s to plan a whole 20 m world (≈ 72 steps, ≈ 1600 candidates).
- Decided by default (the user didn't object): going down stairs is also one foot per tread; ramps use 0.25 m steps both up and down.

### Note for milestone 3 (resolved below)
Swing feasibility depends on three footholds (rear S, stance, target T), not a pair. Options:
(a) a search over pair states, K² states × K successors (could be slow in Python; use caching or a 2 cm grid), or
(b) keep the pairwise search and reject or re-plan when the swing check of the committed triple fails.
Start with (a) plus caching, and measure.

## Implementation status (milestone 3 done, 2026-10-17)
- `swing.py`: `plan_swing(terrain, robot, cfg, S, M, T)` → `Swing(ok, reason, h, samples)`, with 41 samples s = 0..1.
  Each sample holds the hip, swing ankle, and stance/swing `LegAngles`. `Swing.pose(k, side)` gives a full `Pose` (used for playback).
  1. Closed-form minimal h from the exact foot-segment clearance (`Terrain.vertical_clearance`).
  2. Hip path: x moves linearly from mid(S,M) to mid(M,T). y = nominal height, lowered to stay within max reach of both
     ankles and raised to stay above min reach (knee 140°). At s=0 and s=1 this equals `Robot.stand`, so the hip is continuous between steps (tested).
  3. IK for both legs at every sample, and both shanks checked against the terrain. On failure, raise h by 2 cm up to the cap.
- Planner: pairwise search, then verify every swing in the 4-step plan. A failing (M, T) pair is banned and the search re-runs,
  up to 30 times. After that it falls back to the best single step whose swing works, and reports "stuck" only if none exists.
  - Why not search over pairs of footholds: about 140k swing checks per re-plan at 0.46 ms each, i.e. minutes. Lazy checking costs nothing in the default setup:
    over 100 seeds, no swing ever failed, and planning averaged 0.42 s per world.
  - Stress test with a 5 cm arc cap: the robot gets over ramp crests and stops at stairs (correctly impossible).
- Typical bump h: flat 0.03 (the minimum), ramps 0.03, ramp crest about 0.06, stairs 0.10–0.27.
- Not modeled: the swing foot passing the stance foot (in 2D both legs share a plane; a real robot's legs are side by side).
- Not yet in the cost: arc height. That would need triple-aware costs. Add if gaits look too high.

## Implementation status (milestone 4 done, 2026-10-17)
- `Swing.state_at(s)` / `pose_at(s, side)` evaluate the closed-form arc, hip path and IK at any s. The stance ankle stays exactly
  on M (tested at 200 phases per swing). A swing stores S, M, T, h; `samples` are the 41 states the planner verified.
- **Clipping fix:** a dense check (400 phases × 7157 swings over 100 seeds) found the foot 3.9 mm inside a stair corner
  between samples. The ground under the foot jumps at corners, so fixed samples can miss it. `_corner_crossings()` now finds
  (by bisection) the phases where the heel or toe passes over a terrain vertex, and the h solve checks them too. After the fix: min gap 0.00 mm (touch-down only).
- `playback.py`: `Playback` clock over (step_idx, s). Swing duration = foot path length / 0.8 m/s (min 0.2 s), plus a 0.1 s
  double-support dwell. Frame-rate independent (tested). A 20 m world takes about 60 s at 1× (hip ≈ 0.33 m/s).
- Viewer: Space play/pause, N/B step boundaries, `,`/`.` frame scrub, ↑/↓ speed, camera follows the live hip.
  Ghosts show only while paused.
- Export of the motion is deferred to milestone 5 (motion controller), per the user.

## Module plan
- `gait_bot/robot.py` – RobotModel (lengths, masses), FK, IK, COM
- `gait_bot/footholds.py` – candidate generation and validity
- `gait_bot/swing.py` – swing arc and hip path, collision and reach checks
- `gait_bot/planner.py` – DP plus the receding-horizon loop → list of Steps
- `gait_bot/viewer.py` – draw footholds, the 4-step horizon, arcs, the robot, and camera follow

## Milestones
1. robot.py, and draw the robot standing at x=1 to verify the geometry visually.
2. Footholds + DP, and run the receding loop across the whole world to draw every foothold.
3. Swing arcs + hip path, drawing the arcs and flagging infeasible transitions.
4. Kinematic playback (step s through each swing) with camera follow.

## Step length (decided 2026-10-17)
- Step = horizontal distance between consecutive footholds (left to right).
- **ℓ_nom = 0.25 m** (sweet spot), **max 0.40 m**. On stairs the step is exactly 0.25 m, one tread.
- The swing foot itself travels 2 steps per swing (rear to front): 0.5 m nominal, up to 0.8 m.
- Collision checks cover the foot **and shank** against terrain (decided 2026-10-17).

- **Swing clearance c = 0.03 m** (decided).
- Stair tops and ramp platforms use the **normal flat gait** (0.25 m steps) (decided).

## Deferred (not a priority)
- Ending behavior: stopping with feet together near x ≈ 19.5. For now the planner just stops when it runs out of world.
- Tuning the standing hip height to reduce the crouch.
