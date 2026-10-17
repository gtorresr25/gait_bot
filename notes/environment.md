# Environment / terrain

## Decisions (2026-10-17)
- World is 20 m long, x ∈ [0, 20]. The robot spawns at x=1, and [0, 2] is always flat.
- Features are placed at random from a seed. Each one is **up-and-down with net zero height**: it climbs, has a flat top (0.5–1.0 m), then comes back down.
- Flat gaps between features are 1–2 m. The last 1 m of the world is flat.
- **Stairs:** riser 0.20 m, tread 0.25 m, 2–4 steps up and the same number down.
- **Ramps:** 10° slope, rise 0.2–0.6 m (snapped to 5 cm).
- Rendering: white background, black ground line, grey feature fill.

## Observations
- A 10° ramp is long: 0.6 m of rise needs about 3.4 m of run on each side, so a large ramp takes up about 7.8 m of a 20 m world.
  Worlds average about 3.8 features (min 2, max 6 over 500 seeds).
- If a feature doesn't fit, the generator re-rolls up to 20 times before stopping.

## Ideas / open questions
- Should ramps and stairs combine? For example, stairs up and a ramp down.
- Add gaps or holes (no ground) later? These would test planner foothold selection.
- Vary stair riser and tread per world to test robustness.
- Collision: a stair riser is a vertical wall, so the robot's swing foot must clear it.
