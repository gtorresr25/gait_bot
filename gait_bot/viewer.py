"""Pygame side-scroller view of the terrain and robot.

Controls:
  Left/Right   scroll (hold Shift for fast); turns camera follow off
  + / -        zoom in / out
  Home / End   jump to start / end of world
  R            regenerate with a new random seed (and re-plan)
  G            toggle 1 m grid
  Plan mode (default): robot stands on the planned footholds
    Space      play / pause the walk (continuous swings)
    N / B      jump to next / previous step boundary
    , / .      scrub one frame (1/40 of a swing) back / forward
    Up / Down  playback speed x1.25 / ÷1.25
    F          toggle camera follow
    P          cycle foothold display: 4-step plan only → + past steps → all → none
    A          toggle swing arcs + hip path for the 4-step plan
    O          toggle ghost frames of the current swing (s = 0.25, 0.5, 0.75; only while paused)
  Manual mode (entered by clicking):
    Click      place robot: rear (left) foot at cursor, front (right) foot one step ahead
    [ / ]      shorter / longer step between feet (5 cm)
    W / S      raise / lower hip (1 cm)
  Esc / Q      quit
"""
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass

import pygame

from .footholds import Foothold
from .planner import FootstepPlanner, PlanResult
from .playback import Playback, stance_pose
from .robot import Pose, Robot
from .swing import Swing
from .terrain import Terrain, generate

WIDTH, HEIGHT = 1280, 600
GROUND_SCREEN_Y = int(HEIGHT * 0.82)   # where y=0 sits on screen

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (170, 170, 170)
LIGHT = (225, 225, 225)
SPAWN = (40, 110, 220)
LEFT_LEG = (40, 90, 220)     # blue
RIGHT_LEG = (215, 50, 50)    # red
LEFT_FAINT = (170, 190, 240)
RIGHT_FAINT = (240, 175, 175)
COM = (20, 150, 70)
WARN = (200, 110, 0)
HIP_PATH = (90, 90, 90)
GHOST_LEFT = (195, 210, 245)
GHOST_RIGHT = (245, 200, 200)
GHOST_TORSO = (200, 200, 200)

SCROLL_SPEED = 3.0       # m/s
FAST_MULT = 4.0
ZOOM_MIN, ZOOM_MAX = 20.0, 400.0   # pixels per meter


class Camera:
    def __init__(self, ppm: float = 100.0):
        self.x = 0.0      # world x at left edge of screen
        self.ppm = ppm

    def to_screen(self, x: float, y: float) -> tuple[int, int]:
        return round((x - self.x) * self.ppm), round(GROUND_SCREEN_Y - y * self.ppm)

    def to_world_x(self, sx: float) -> float:
        return self.x + sx / self.ppm

    def view_width(self) -> float:
        return WIDTH / self.ppm

    def clamp(self, world_len: float) -> None:
        margin = 0.5
        hi = max(-margin, world_len + margin - self.view_width())
        self.x = min(max(self.x, -margin), hi)

    def zoom(self, factor: float, world_len: float) -> None:
        center = self.x + self.view_width() / 2
        self.ppm = min(max(self.ppm * factor, ZOOM_MIN), ZOOM_MAX)
        self.x = center - self.view_width() / 2
        self.clamp(world_len)


@dataclass
class ViewState:
    mode: str = "plan"              # "plan": robot stands on the planned footholds; "manual": click-placed
    follow: bool = True             # camera follows the hip
    footholds: str = "plan"         # "plan" | "trail" | "all" | "none"
    show_arcs: bool = True
    show_ghosts: bool = True
    # manual mode
    x_rear: float = 1.0 - 0.125     # centered on spawn x=1 with the nominal step
    step: float = 0.25
    hip_dy: float = 0.0


FOOTHOLD_MODES = ["plan", "trail", "all", "none"]


def draw_terrain(screen, terrain: Terrain, cam: Camera, show_grid: bool, font) -> None:
    L = terrain.length
    if show_grid:
        for m in range(int(L) + 1):
            sx, _ = cam.to_screen(m, 0)
            pygame.draw.line(screen, LIGHT, (sx, 0), (sx, HEIGHT))
            screen.blit(font.render(f"{m}", True, GREY), (sx + 3, GROUND_SCREEN_Y + 6))

    # grey fill for each feature (between its outline and y=0)
    for f in terrain.features:
        pygame.draw.polygon(screen, GREY, [cam.to_screen(x, y) for x, y in f.points])

    # black ground line
    pygame.draw.lines(screen, BLACK, False, [cam.to_screen(x, y) for x, y in terrain.profile], 2)

    # world bounds and spawn marker
    for bx in (0.0, L):
        sx, _ = cam.to_screen(bx, 0)
        pygame.draw.line(screen, BLACK, (sx, GROUND_SCREEN_Y - 40), (sx, GROUND_SCREEN_Y + 40), 1)
    sx, sy = cam.to_screen(terrain.config.spawn_x, terrain.height_at(terrain.config.spawn_x))
    pygame.draw.polygon(screen, SPAWN, [(sx, sy + 4), (sx - 6, sy + 14), (sx + 6, sy + 14)])


def draw_robot(screen, robot: Robot, pose: Pose, cam: Camera, terrain: Terrain,
               ghost: bool = False) -> None:
    w = max(2, round(cam.ppm * 0.02))   # line width ≈ 2 cm
    if ghost:
        w = max(1, w // 2)
    joint_r = max(2, w // 2 + 1)
    colors = (GHOST_LEFT, GHOST_RIGHT) if ghost else (LEFT_LEG, RIGHT_LEG)
    # draw the far (left) leg first so the right leg sits on top
    for side, q, color in (("L", pose.left, colors[0]), ("R", pose.right, colors[1])):
        p = {k: cam.to_screen(*v) for k, v in robot.leg_points(pose.hip, q).items()}
        pygame.draw.line(screen, color, p["hip"], p["knee"], w)
        pygame.draw.line(screen, color, p["knee"], p["ankle"], w)
        pygame.draw.line(screen, color, p["heel"], p["toe"], w)
        for j in ("knee", "ankle"):
            pygame.draw.circle(screen, color, p[j], joint_r)
    hip = cam.to_screen(*pose.hip)
    top = cam.to_screen(pose.hip[0], pose.hip[1] + robot.cfg.torso_len)
    pygame.draw.line(screen, GHOST_TORSO if ghost else BLACK, hip, top, w + 1)
    if ghost:
        return
    pygame.draw.circle(screen, BLACK, hip, joint_r + 1)

    # center of mass: ring + crosshair, with a dashed drop line to the ground
    cx, cy = robot.com(pose)
    sx, sy = cam.to_screen(cx, cy)
    r = max(5, round(cam.ppm * 0.03))
    pygame.draw.circle(screen, COM, (sx, sy), r, 2)
    pygame.draw.line(screen, COM, (sx - r, sy), (sx + r, sy), 1)
    pygame.draw.line(screen, COM, (sx, sy - r), (sx, sy + r), 1)
    _, gy = cam.to_screen(cx, terrain.height_at(cx))
    for y in range(sy + r, gy, 8):
        pygame.draw.line(screen, COM, (sx, y), (sx, min(y + 4, gy)), 1)


def draw_foothold(screen, f: Foothold, side: str, cam: Camera, robot: Robot,
                  faint: bool, font=None, label: str = "") -> None:
    c = robot.cfg
    ux, uy = math.cos(f.angle), math.sin(f.angle)
    heel = cam.to_screen(f.x - c.heel_offset * ux, f.y - c.heel_offset * uy)
    toe = cam.to_screen(f.x + (c.foot_len - c.heel_offset) * ux, f.y + (c.foot_len - c.heel_offset) * uy)
    lift = 4   # px above the surface so it doesn't hide the ground line
    if side == "L":
        color = LEFT_FAINT if faint else LEFT_LEG
    else:
        color = RIGHT_FAINT if faint else RIGHT_LEG
    pygame.draw.line(screen, color, (heel[0], heel[1] - lift), (toe[0], toe[1] - lift), 3 if faint else 5)
    if label and font:
        img = font.render(label, True, color)
        screen.blit(img, ((heel[0] + toe[0]) // 2 - img.get_width() // 2, heel[1] - lift - 20))


def draw_dashed(screen, color, pts, width=2, dash=6, gap=5) -> None:
    """Dashed polyline through screen points."""
    on, left = True, dash
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        t = 0.0
        while seg > 0 and t < seg:
            step = min(left, seg - t)
            if on:
                a, b = t / seg, (t + step) / seg
                pygame.draw.line(screen, color, (x0 + (x1 - x0) * a, y0 + (y1 - y0) * a),
                                 (x0 + (x1 - x0) * b, y0 + (y1 - y0) * b), width)
            t += step
            left -= step
            if left <= 1e-9:
                on, left = not on, (gap if on else dash)


def draw_swing(screen, sw: Swing, side: str, cam: Camera, hip_path: bool) -> None:
    if not sw.samples:
        return
    color = (LEFT_LEG if side == "L" else RIGHT_LEG) if sw.ok else WARN
    draw_dashed(screen, color, [cam.to_screen(*p.ankle) for p in sw.samples])
    if hip_path:
        draw_dashed(screen, HIP_PATH, [cam.to_screen(*p.hip) for p in sw.samples], 1, 3, 4)


def draw(screen, terrain: Terrain, cam: Camera, show_grid: bool, font,
         robot: Robot, pb: Playback, st: ViewState) -> None:
    screen.fill(WHITE)
    draw_terrain(screen, terrain, cam, show_grid, font)
    plan, i = pb.plan, pb.step_idx
    planning = st.mode == "plan"

    # faint: past steps (trail) or the whole committed sequence (all)
    if st.footholds in ("trail", "all"):
        upto = i + 1 if st.footholds == "trail" and planning else len(plan.footholds)
        for k, f in enumerate(plan.footholds[:upto]):
            draw_foothold(screen, f, plan.side(k), cam, robot, faint=True)
    # swing arcs (ankle paths) for the 4-step plan; hip path for the current swing
    if planning and st.show_arcs and i < len(plan.horizon_swings):
        for n, sw in enumerate(plan.horizon_swings[i]):
            draw_swing(screen, sw, plan.side(i + n), cam, hip_path=(n == 0))
    if planning and st.show_ghosts and not pb.playing and i < len(plan.swings):
        sw = plan.swings[i]
        for s in (0.25, 0.5, 0.75):
            draw_robot(screen, robot, sw.pose_at(s, plan.side(i)), cam, terrain, ghost=True)
    # bold: what the planner sees right now (the 4-step horizon)
    if st.footholds != "none" and planning and i < len(plan.horizons):
        for n, f in enumerate(plan.horizons[i]):
            draw_foothold(screen, f, plan.side(i + 2 + n), cam, robot, faint=False, font=font, label=str(n + 1))

    if planning:
        pose = pb.pose(robot, terrain)
        results = []
        hip = pose.hip if pose else (0.0, 0.0)
        if pose is None:
            _, results, hip = stance_pose(robot, terrain, plan, i)
        status = "goal reached" if plan.reached_goal else f"STOPPED: {plan.reason}"
        if pb.at_end:
            phase = "standing (end)"
        elif pb.in_dwell:
            phase = "double support"
        else:
            phase = f"swing {plan.side(i)}  s {pb.s:.2f}  h {plan.swings[i].h:.2f} m"
        robot_lines = [
            f"plan   step {i}/{pb.last_idx}  {phase}   [{status}]  "
            f"{'PLAYING' if pb.playing else 'paused'} x{pb.speed:.2f}  follow {'on' if st.follow else 'off'}  "
            f"footholds: {st.footholds}",
        ]
        colors = [BLACK if plan.reached_goal else WARN]
    else:
        pose, results, hip = robot.stand(terrain, st.x_rear, st.x_rear + st.step, st.hip_dy)
        robot_lines = [f"manual feet L {st.x_rear:.2f} / R {st.x_rear + st.step:.2f} m  step {st.step:.2f}"
                       "   (N/B/Space returns to plan mode)"]
        colors = [BLACK]

    if pose:
        draw_robot(screen, robot, pose, cam, terrain)
        com = robot.com(pose)
        deg = lambda a: f"{math.degrees(a):5.1f}"
        robot_lines.append(
            f"  hip ({pose.hip[0]:.2f}, {pose.hip[1]:.2f})  COM ({com[0]:.2f}, {com[1]:.2f})   "
            f"L hip {deg(pose.left.q_hip)}° knee {deg(pose.left.q_knee)}°   "
            f"R hip {deg(pose.right.q_hip)}° knee {deg(pose.right.q_knee)}°")
        colors.append(BLACK)
    else:
        fails = [f"{s}: {r.reason}" for s, r in zip("LR", results) if not r.ok]
        robot_lines.append(f"  IK FAILED at hip ({hip[0]:.2f}, {hip[1]:.2f})  " + "  ".join(fails))
        colors.append(WARN)

    mx = cam.to_world_x(pygame.mouse.get_pos()[0])
    f = terrain.feature_at(mx)
    hud = [
        f"seed {terrain.seed}   features {len(terrain.features)}   zoom {cam.ppm:.0f} px/m",
        f"view x {cam.x:.2f} m   cursor x {mx:.2f} m   h {terrain.height_at(mx):.2f} m"
        + (f"   [{f.kind}, top {f.height:.2f} m]" if f else ""),
        "Space: play  N/B: step  ,/.: frame  Up/Down: speed  F: follow  P: footholds  A: arcs  O: ghosts  "
        "<-/->: scroll  +/-: zoom  R: new seed  G: grid  click: manual  Esc",
    ]
    for k, (line, c) in enumerate(zip(hud + robot_lines, [BLACK] * 3 + colors)):
        screen.blit(font.render(line, True, c, WHITE), (10, 10 + k * 20))


def make_plan(terrain: Terrain, robot: Robot) -> PlanResult:
    return FootstepPlanner(terrain, robot).plan()


def run(seed: int | None = None) -> None:
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("gait_bot")
    font = pygame.font.SysFont("menlo,consolas,monospace", 14)
    clock = pygame.time.Clock()

    seed = seed if seed is not None else random.randrange(10_000)
    terrain = generate(seed)
    robot = Robot()
    pb = Playback(make_plan(terrain, robot))
    st = ViewState()
    cam = Camera(ppm=200.0)
    cam.clamp(terrain.length)
    show_grid = True

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                st.mode, pb.playing = "manual", False
                st.x_rear = cam.to_world_x(event.pos[0])
                st.hip_dy = 0.0
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if k in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    cam.zoom(1.25, terrain.length)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    cam.zoom(1 / 1.25, terrain.length)
                elif k == pygame.K_HOME:
                    cam.x, st.follow = -1e9, False
                elif k == pygame.K_END:
                    cam.x, st.follow = 1e9, False
                elif k == pygame.K_r:
                    terrain = generate(random.randrange(10_000))
                    speed = pb.speed
                    pb = Playback(make_plan(terrain, robot))
                    pb.speed = speed
                    st.mode, st.follow = "plan", True
                elif k == pygame.K_g:
                    show_grid = not show_grid
                elif k == pygame.K_SPACE:
                    st.mode, st.follow = "plan", True
                    pb.toggle()
                elif k == pygame.K_n:
                    st.mode, pb.playing = "plan", False
                    pb.jump_to(pb.step_idx + 1)
                elif k == pygame.K_b:
                    st.mode, pb.playing = "plan", False
                    pb.jump_to(pb.step_idx - (0 if pb.s > 0 else 1))
                elif k in (pygame.K_COMMA, pygame.K_PERIOD):
                    st.mode, pb.playing = "plan", False
                    pb.scrub(1 if k == pygame.K_PERIOD else -1)
                elif k == pygame.K_UP:
                    pb.speed = min(8.0, pb.speed * 1.25)
                elif k == pygame.K_DOWN:
                    pb.speed = max(0.05, pb.speed / 1.25)
                elif k == pygame.K_f:
                    st.follow = not st.follow
                elif k == pygame.K_a:
                    st.show_arcs = not st.show_arcs
                elif k == pygame.K_o:
                    st.show_ghosts = not st.show_ghosts
                elif k == pygame.K_p:
                    st.footholds = FOOTHOLD_MODES[(FOOTHOLD_MODES.index(st.footholds) + 1) % len(FOOTHOLD_MODES)]
                elif k == pygame.K_LEFTBRACKET:
                    st.step = round(max(0.0, st.step - 0.05), 2)
                elif k == pygame.K_RIGHTBRACKET:
                    st.step = round(min(0.6, st.step + 0.05), 2)
                elif k == pygame.K_w:
                    st.hip_dy = round(st.hip_dy + 0.01, 2)
                elif k == pygame.K_s:
                    st.hip_dy = round(st.hip_dy - 0.01, 2)

        pb.advance(dt)

        keys = pygame.key.get_pressed()
        scroll = keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]
        if scroll:
            st.follow = False
            speed = SCROLL_SPEED * (FAST_MULT if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT] else 1)
            cam.x += scroll * speed * dt
        elif st.follow and st.mode == "plan":
            pose = pb.pose(robot, terrain)
            if pose:
                target = pose.hip[0] - cam.view_width() * 0.4
                cam.x += (target - cam.x) * min(1.0, 5.0 * dt)
        cam.clamp(terrain.length)

        draw(screen, terrain, cam, show_grid, font, robot, pb, st)
        pygame.display.flip()

    pygame.quit()


def main() -> None:
    p = argparse.ArgumentParser(description="View gait_bot terrain and robot")
    p.add_argument("--seed", type=int, default=None)
    run(p.parse_args().seed)


if __name__ == "__main__":
    main()
