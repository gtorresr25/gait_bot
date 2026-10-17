# gait_bot

A 2D side-scroller world with a footstep planner and a biped robot that follows it.

## Roadmap
1. [x] Terrain + viewer (flat start, random stairs/ramps, keyboard scrolling)
2. [x] Step planner: footholds + 4-step search (M2), swing arcs + hip path + IK/collision checks (M3) → see [notes/step_planner.md](notes/step_planner.md)
3. [~] Biped robot: kinematics, COM and drawing done; simulation TBD → see [notes/robot.md](notes/robot.md)
4. [x] Milestone 4: continuous kinematic playback (exact `pose_at(s)`), camera follow, timing by foot path length
5. [ ] Milestone 5: motion controller (export of the reference motion lands here)

## Conventions
- **Units:** meters, seconds, degrees in config (radians in math). 1.0 = 1 m.
- **Frame:** x is to the right, y is up. Base ground is y=0.
- **World:** x ∈ [0, 20]. The robot spawns at x=1. Ground is flat on [0, 2] and on the last 1 m.
- **Terrain** is a polyline `Terrain.profile` of (x, y) points with x never decreasing. Stair risers are vertical segments (repeated x).
- **Determinism:** each world comes from an integer seed, and the same seed gives the same world.
- **Visuals:** white background, black ground line, grey fill for features.

## Setup / run
```bash
conda env create -f environment.yml     # once
conda activate gait_bot
python -m gait_bot.viewer [--seed N]    # viewer
python -m pytest -q                     # tests
```
Viewer keys: Space play/pause the walk, N/B step boundaries, ,/. frame scrub, ↑/↓ speed, F camera follow, P footholds, A arcs, O ghost frames, ←/→ scroll (Shift fast),
+/- zoom, Home/End, R new seed, G grid, click = manual placement ([/] step, W/S hip), Esc quit.

## Layout
- `gait_bot/config.py` – `TerrainConfig`, `RobotConfig`, `PlannerConfig` (all tunable numbers)
- `gait_bot/terrain.py` – `generate(seed)` → `Terrain` (profile, features, `height_at`, `feature_at`)
- `gait_bot/robot.py` – `Robot`: FK, analytic IK (knee forward, joint limits), COM, `stand()`
- `gait_bot/footholds.py` – walkable surfaces and candidate footholds (one per stair tread)
- `gait_bot/swing.py` – `plan_swing(S, M, T)`: smoothstep+sine ankle arc, hip path, IK/shank checks → `Swing`
- `gait_bot/planner.py` – `FootstepPlanner`: 4-step search + swing verification, receding horizon → `PlanResult`
- `gait_bot/playback.py` – `Playback`: clock over (step, phase s), timing + dwell, scrub/jump; `stance_pose()`
- `gait_bot/viewer.py` – pygame renderer, camera follow, playback controls, manual placement sandbox
- `tests/` – terrain invariants (200 seeds); robot IK/FK, limits, COM; planner over 20 seeds (goal, step length, every tread once, reachability)
- `notes/` – one file per idea or subsystem: decisions, open questions, scratch thinking

## Dev tips
- `conda run` does not forward heredoc stdin. Put scripts in a file. For headless renders use
  `PYTHONPATH=$PWD SDL_VIDEODRIVER=dummy conda run --no-capture-output -n gait_bot python script.py`.

## Working agreements
- Ask clarifying questions when a choice changes the design. Record each answer in the matching `notes/*.md`.
- Keep the world, planner, and robot as separate modules. The planner uses `Terrain` + `Robot` (for reach checks).
