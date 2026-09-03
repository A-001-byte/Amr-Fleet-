"""
simulation.py

Pygame main loop and rendering for the SwarmSync warehouse simulation.

Visual additions (purely rendering, no logic changes):
  - Warehouse floor plan: shelf rectangles for blocked nodes, colored pickup
    zones, charging station zones at grid corners.
  - Decision log panel: shows the last N negotiation/deadlock events with the
    real reason derived from the actual priority sort key in collision.py.
"""

import collections
import colorsys
import random

import pygame

from collision import resolve_and_update
from metrics import Metrics
from pathfinding import find_path
from robot import Robot
from warehouse import build_warehouse_graph

DEFAULT_NUM_ROBOTS = 3

WAREHOUSE_WIDTH = 1000  # the graph/robots are only drawn within this width
HUD_WIDTH = 260         # sidebar panel width, to the right of the warehouse
WINDOW_WIDTH = WAREHOUSE_WIDTH + HUD_WIDTH
WINDOW_HEIGHT = 800
MARGIN = 40

# ── Floor plan colours ────────────────────────────────────────────────────────
FLOOR_COLOR          = (215, 205, 190)   # warm light concrete
AISLE_LINE_COLOR     = (180, 168, 150)   # subtle aisle markings
SHELF_FILL_COLOR     = (72, 50, 30)      # dark wood/metal shelving
SHELF_EDGE_COLOR     = (110, 80, 50)     # shelf highlight rim
SHELF_STRIPE_COLOR   = (88, 62, 38)      # inner shelf stripe accent
PICKUP_ZONE_COLOR    = (40, 170, 80, 160)  # semi-transparent green
PICKUP_ZONE_BORDER   = (30, 220, 90)
PICKUP_LABEL_COLOR   = (20, 220, 80)
CHARGE_ZONE_COLOR    = (30, 100, 200, 140)
CHARGE_ZONE_BORDER   = (60, 160, 255)
CHARGE_LABEL_COLOR   = (100, 180, 255)
OPEN_NODE_COLOR      = (180, 168, 152)   # subtle walkable node dot
EDGE_COLOR           = (160, 148, 130)   # aisle path lines

ROBOT_RADIUS         = 12
NODE_RADIUS          = 3
PICKUP_NODE_RADIUS   = 5
SHELF_SIZE           = 32                # width/height of a shelf rectangle (px)
ZONE_RADIUS          = 20               # half-size of the zone highlight behind pickups

DEADLOCK_FLASH_COLOR = (255, 230, 0)
LABEL_COLOR          = (20, 20, 20)

# ── HUD colours ───────────────────────────────────────────────────────────────
HUD_BACKGROUND_COLOR = (18, 18, 18)
HUD_DIVIDER_COLOR    = (70, 70, 70)
HUD_TEXT_COLOR       = (220, 220, 220)
HUD_TITLE_COLOR      = (255, 255, 255)
HUD_DIM_COLOR        = (140, 140, 140)
HUD_ROW_HEIGHT       = 58
HUD_PADDING          = 14

STATUS_COLORS = {
    "MOVING":   (110, 220, 110),
    "WAITING":  (240, 200,  60),
    "DEADLOCK": (240,  90,  90),
}

# ── Event log colours ─────────────────────────────────────────────────────────
LOG_BG_COLOR         = (24, 24, 32)
LOG_YIELD_COLOR      = (110, 200, 255)
LOG_DEADLOCK_COLOR   = (255, 140,  60)
LOG_DIM_COLOR        = (100, 100, 120)
LOG_BORDER_COLOR     = (60,  60,  80)
TOAST_BG_COLOR       = (30,  30,  45, 210)
TOAST_YIELD_ACCENT   = (80, 160, 255)
TOAST_DEADLOCK_ACCENT= (255, 120, 40)
MAX_LOG_ENTRIES      = 12          # how many events to keep visible
TOAST_DURATION       = 150         # frames to show the toast overlay


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def generate_robot_colors(count):
    """
    Generate `count` visually distinct RGB colors by spreading hues evenly
    around the color wheel. Works for any robot count, not just 3.
    """
    colors = []
    for i in range(count):
        hue = i / max(count, 1)
        r, g, b = colorsys.hsv_to_rgb(hue, 0.65, 0.95)
        colors.append((int(r * 255), int(g * 255), int(b * 255)))
    return colors


def compute_layout(width, height):
    """
    Map grid coordinates (x, y) to pixel positions, evenly spaced within
    the warehouse drawing area (minus a margin on each side). The HUD
    sidebar lives outside this area, to the right.
    """
    usable_width  = WAREHOUSE_WIDTH - 2 * MARGIN
    usable_height = WINDOW_HEIGHT    - 2 * MARGIN

    step_x = usable_width  / max(width  - 1, 1)
    step_y = usable_height / max(height - 1, 1)

    positions = {}
    for gx in range(width):
        for gy in range(height):
            px = MARGIN + gx * step_x
            py = MARGIN + gy * step_y
            positions[(gx, gy)] = (px, py)
    return positions


def _lerp(a, b, t):
    return a + (b - a) * t


# ─────────────────────────────────────────────────────────────────────────────
# Warehouse floor plan renderer
# ─────────────────────────────────────────────────────────────────────────────

def draw_warehouse_floor(screen, graph, positions, metadata, charge_nodes, alpha_surf):
    """
    Render the warehouse as a proper floor plan:
      1. Warm concrete floor background
      2. Aisle edges (subtle lines between walkable nodes)
      3. Pickup zone overlays (translucent green rectangles + border)
      4. Charging station overlays (translucent blue)
      5. Shelf rectangles for blocked nodes (solid brown blocks)
      6. Small walkable node dots (light, just for spatial reference)

    alpha_surf: a pre-created SRCALPHA surface the same size as the
    warehouse area, reused each frame so we don't allocate per frame.
    """
    # ── 1. Floor background ──────────────────────────────────────────────────
    pygame.draw.rect(screen, FLOOR_COLOR, (0, 0, WAREHOUSE_WIDTH, WINDOW_HEIGHT))

    # ── 2. Aisle edges ───────────────────────────────────────────────────────
    for a, b in graph.edges():
        pygame.draw.line(screen, AISLE_LINE_COLOR, positions[a], positions[b], 1)

    # ── 3. Pickup zone overlays ──────────────────────────────────────────────
    alpha_surf.fill((0, 0, 0, 0))
    for node in metadata["pickup_nodes"]:
        px, py = positions[node]
        zone_rect = pygame.Rect(px - ZONE_RADIUS, py - ZONE_RADIUS,
                                ZONE_RADIUS * 2, ZONE_RADIUS * 2)
        pygame.draw.rect(alpha_surf, PICKUP_ZONE_COLOR, zone_rect, border_radius=6)
        pygame.draw.rect(alpha_surf, (*PICKUP_ZONE_BORDER, 220), zone_rect,
                         width=2, border_radius=6)
    screen.blit(alpha_surf, (0, 0))

    # ── 4. Charging station overlays ─────────────────────────────────────────
    alpha_surf.fill((0, 0, 0, 0))
    for node in charge_nodes:
        px, py = positions[node]
        zone_rect = pygame.Rect(px - ZONE_RADIUS, py - ZONE_RADIUS,
                                ZONE_RADIUS * 2, ZONE_RADIUS * 2)
        pygame.draw.rect(alpha_surf, CHARGE_ZONE_COLOR, zone_rect, border_radius=8)
        pygame.draw.rect(alpha_surf, (*CHARGE_ZONE_BORDER, 230), zone_rect,
                         width=2, border_radius=8)
    screen.blit(alpha_surf, (0, 0))

    # ── 5. Shelf blocks for blocked nodes ────────────────────────────────────
    half = SHELF_SIZE // 2
    for node, data in graph.nodes(data=True):
        if not data["blocked"]:
            continue
        px, py = positions[node]
        rect = pygame.Rect(int(px) - half, int(py) - half, SHELF_SIZE, SHELF_SIZE)
        pygame.draw.rect(screen, SHELF_FILL_COLOR, rect, border_radius=3)
        # inner stripe accent
        inner = rect.inflate(-8, -8)
        pygame.draw.rect(screen, SHELF_STRIPE_COLOR, inner, border_radius=2)
        # rim highlight
        pygame.draw.rect(screen, SHELF_EDGE_COLOR, rect, width=2, border_radius=3)

    # ── 6. Open node dots (subtle reference) ─────────────────────────────────
    for node, data in graph.nodes(data=True):
        if data["blocked"] or data["pickup"]:
            continue
        pos = positions[node]
        if node in {n for n in (charge_nodes or []) if not data["blocked"]}:
            continue
        pygame.draw.circle(screen, OPEN_NODE_COLOR, pos, NODE_RADIUS)


def draw_zone_labels(screen, positions, metadata, charge_nodes, small_font, icon_font):
    """Draw text labels on top of pickup and charging zones (drawn after robots
    so labels are always on top)."""
    for node in metadata["pickup_nodes"]:
        px, py = positions[node]
        label = small_font.render("P", True, PICKUP_LABEL_COLOR)
        r = label.get_rect(center=(px, py - ZONE_RADIUS - 6))
        screen.blit(label, r)

    for node in charge_nodes:
        px, py = positions[node]
        label = icon_font.render("⚡", True, CHARGE_LABEL_COLOR)
        r = label.get_rect(center=(px, py - ZONE_RADIUS - 7))
        screen.blit(label, r)


# ─────────────────────────────────────────────────────────────────────────────
# Robot renderer
# ─────────────────────────────────────────────────────────────────────────────

def draw_robots(screen, robots, positions, font):
    for robot in robots:
        from_node, to_node, t = robot.get_render_node_pair()
        x1, y1 = positions[from_node]
        x2, y2 = positions[to_node]
        pos = (_lerp(x1, x2, t), _lerp(y1, y2, t))

        # Shadow for depth
        shadow_pos = (pos[0] + 2, pos[1] + 3)
        pygame.draw.circle(screen, (0, 0, 0, 80), shadow_pos, ROBOT_RADIUS)

        if robot.detour_flash > 0:
            pygame.draw.circle(screen, DEADLOCK_FLASH_COLOR, pos, ROBOT_RADIUS + 5, width=2)

        pygame.draw.circle(screen, robot.color, pos, ROBOT_RADIUS)
        # Specular highlight
        hipos = (int(pos[0]) - 3, int(pos[1]) - 3)
        pygame.draw.circle(screen, (255, 255, 255, 120), hipos, 3)

        label = font.render(f"R{robot.id + 1}", True, LABEL_COLOR)
        label_rect = label.get_rect(center=pos)
        screen.blit(label, label_rect)


# ─────────────────────────────────────────────────────────────────────────────
# HUD — Fleet Status
# ─────────────────────────────────────────────────────────────────────────────

def get_robot_status(robot):
    """
    Derive a simple human-readable status for the HUD from the robot's
    existing state - no extra bookkeeping needed on the Robot itself.
    """
    if robot.detour_flash > 0:
        return "DEADLOCK"
    if robot.progress == 0.0 and robot.wait_ticks > 0:
        return "WAITING"
    return "MOVING"


def draw_hud(screen, robots, font, title_font):
    panel_rect = pygame.Rect(WAREHOUSE_WIDTH, 0, HUD_WIDTH, WINDOW_HEIGHT)
    pygame.draw.rect(screen, HUD_BACKGROUND_COLOR, panel_rect)
    pygame.draw.line(
        screen, HUD_DIVIDER_COLOR, (WAREHOUSE_WIDTH, 0), (WAREHOUSE_WIDTH, WINDOW_HEIGHT), 2
    )

    title = title_font.render("Fleet Status", True, HUD_TITLE_COLOR)
    screen.blit(title, (WAREHOUSE_WIDTH + HUD_PADDING, HUD_PADDING))

    row_top = HUD_PADDING + 40
    for robot in robots:
        row_y = row_top + robot.id * HUD_ROW_HEIGHT
        swatch_center = (WAREHOUSE_WIDTH + HUD_PADDING + 8, row_y + 8)
        pygame.draw.circle(screen, robot.color, swatch_center, 8)

        name_label = font.render(f"R{robot.id + 1}", True, HUD_TEXT_COLOR)
        screen.blit(name_label, (WAREHOUSE_WIDTH + HUD_PADDING + 24, row_y))

        status = get_robot_status(robot)
        status_label = font.render(status, True, STATUS_COLORS[status])
        screen.blit(status_label, (WAREHOUSE_WIDTH + HUD_PADDING + 70, row_y))

        goal_label = font.render(f"goal: {robot.goal}", True, HUD_TEXT_COLOR)
        screen.blit(goal_label, (WAREHOUSE_WIDTH + HUD_PADDING, row_y + 20))

        wait_label = font.render(f"wait: {robot.wait_ticks} ticks", True, HUD_TEXT_COLOR)
        screen.blit(wait_label, (WAREHOUSE_WIDTH + HUD_PADDING, row_y + 38))

        pygame.draw.line(
            screen,
            HUD_DIVIDER_COLOR,
            (WAREHOUSE_WIDTH + HUD_PADDING, row_y + HUD_ROW_HEIGHT - 8),
            (WINDOW_WIDTH - HUD_PADDING, row_y + HUD_ROW_HEIGHT - 8),
            1,
        )


# ─────────────────────────────────────────────────────────────────────────────
# Event Log Panel
# ─────────────────────────────────────────────────────────────────────────────

def _fmt_event(event):
    """Format a single event dict into a human-readable string pair (line1, line2)."""
    t = event["tick"]
    if event["type"] == "yield":
        line1 = f"[T={t}] R{event['robot'] + 1} → yields to R{event['holder'] + 1}"
        line2 = f"   ↳ {event['reason']}"
        color = LOG_YIELD_COLOR
    else:  # deadlock
        line1 = f"[T={t}] R{event['robot'] + 1} → DEADLOCK BROKEN"
        line2 = f"   ↳ {event['reason']}"
        color = LOG_DEADLOCK_COLOR
    return line1, line2, color


def draw_event_log(screen, event_log, fleet_rows, font, title_font):
    """
    Render the decision log panel below the fleet status section in the HUD.
    Shows the last MAX_LOG_ENTRIES events with their real reason text.
    """
    num_robots = fleet_rows
    fleet_section_bottom = HUD_PADDING + 40 + num_robots * HUD_ROW_HEIGHT + 12

    # Section title
    log_title_y = fleet_section_bottom
    pygame.draw.line(
        screen, HUD_DIVIDER_COLOR,
        (WAREHOUSE_WIDTH + HUD_PADDING, log_title_y),
        (WINDOW_WIDTH - HUD_PADDING, log_title_y), 1
    )
    log_title_y += 10
    title = title_font.render("Negotiation Log", True, HUD_TITLE_COLOR)
    screen.blit(title, (WAREHOUSE_WIDTH + HUD_PADDING, log_title_y))
    log_title_y += 28

    if not event_log:
        placeholder = font.render("No events yet…", True, HUD_DIM_COLOR)
        screen.blit(placeholder, (WAREHOUSE_WIDTH + HUD_PADDING, log_title_y))
        return

    # Draw events newest-first
    y = log_title_y
    line_h = 13
    entry_gap = 4
    available_h = WINDOW_HEIGHT - log_title_y - 8

    entries = list(event_log)[-MAX_LOG_ENTRIES:][::-1]  # newest first
    for i, event in enumerate(entries):
        if y + line_h * 2 + entry_gap > WINDOW_HEIGHT - 4:
            break
        line1, line2, color = _fmt_event(event)

        # Fade older entries slightly
        age_alpha = max(0.45, 1.0 - i * 0.07)
        faded = tuple(int(c * age_alpha) for c in color)
        dim_faded = tuple(int(c * age_alpha) for c in LOG_DIM_COLOR)

        l1 = font.render(line1, True, faded)
        screen.blit(l1, (WAREHOUSE_WIDTH + HUD_PADDING, y))
        y += line_h

        l2 = font.render(line2, True, dim_faded)
        screen.blit(l2, (WAREHOUSE_WIDTH + HUD_PADDING, y))
        y += line_h + entry_gap


# ─────────────────────────────────────────────────────────────────────────────
# Toast overlay
# ─────────────────────────────────────────────────────────────────────────────

def draw_toast(screen, event, frames_left, font, title_font):
    """
    Show a floating toast in the bottom-left of the warehouse area for the
    most recent negotiation event. Fades out over TOAST_DURATION frames.
    """
    if event is None or frames_left <= 0:
        return

    alpha = min(255, int(255 * frames_left / TOAST_DURATION))
    if alpha <= 0:
        return

    line1, line2, accent = _fmt_event(event)

    toast_w, toast_h = 420, 52
    toast_x = MARGIN
    toast_y = WINDOW_HEIGHT - MARGIN - toast_h

    toast_surf = pygame.Surface((toast_w, toast_h), pygame.SRCALPHA)
    bg = (*TOAST_BG_COLOR[:3], min(210, alpha))
    pygame.draw.rect(toast_surf, bg, (0, 0, toast_w, toast_h), border_radius=8)
    acc = (*accent, min(255, alpha))
    pygame.draw.rect(toast_surf, acc, (0, 0, toast_w, toast_h), width=1, border_radius=8)
    pygame.draw.rect(toast_surf, acc, (0, 0, 3, toast_h), border_radius=3)

    faded_accent = tuple(int(c * alpha / 255) for c in accent)
    faded_dim    = tuple(int(c * alpha / 255) for c in HUD_DIM_COLOR)

    t1 = title_font.render(line1, True, faded_accent)
    toast_surf.blit(t1, (10, 8))
    t2 = font.render(line2, True, faded_dim)
    toast_surf.blit(t2, (10, 30))

    screen.blit(toast_surf, (toast_x, toast_y))


# ─────────────────────────────────────────────────────────────────────────────
# Goal assignment and robot spawning (unchanged logic)
# ─────────────────────────────────────────────────────────────────────────────

def assign_new_goal(robot, graph, metadata, rng, metrics, tick):
    """
    Pick a new random pickup/drop node (different from where the robot
    currently sits, if possible) and route the robot there via A*. Called
    whenever a robot finishes its current path, so robots keep moving
    indefinitely instead of stopping once they arrive.

    This is always called right after a robot arrives at its previous goal
    (or after a previous idle retry), so it doubles as the task-completion
    event for metrics purposes - except when the robot is already idle, in
    which case nothing actually completed and we skip that record.

    warehouse.py guarantees the walkable graph is fully connected, so every
    pickup point should always be reachable. This still handles the case
    defensively: if the first candidate goal has no path, it tries the
    others before giving up, and if truly none are reachable, marks the
    robot idle rather than looping forever "completing" 0-tick fake tasks
    (see the bug this replaces, described in collision.py's history).
    """
    if not robot.idle:
        metrics.record_task_completion(robot.id, tick - robot.task_start_tick, tick)

    candidates = [n for n in metadata["pickup_nodes"] if n != robot.position]
    rng.shuffle(candidates)
    for goal_node in (candidates or metadata["pickup_nodes"]):
        full_path = find_path(graph, robot.position, goal_node)
        if len(full_path) > 1:  # a real route exists, not just "already there"
            robot.set_path(full_path[1:], goal=goal_node)  # drop the start node
            robot.task_start_tick = tick
            robot.idle = False
            return

    # No reachable pickup point from here - sit idle and retry next tick
    # rather than fake-completing. Should not happen given the connectivity
    # guarantee, but this is the honest fallback if it ever does.
    robot.set_path([])
    robot.idle = True
    robot.task_start_tick = tick


def spawn_robots(graph, metadata, rng, num_robots):
    """
    Spawn `num_robots` robots at distinct random open nodes, each assigned a
    random pickup/drop node as its goal. The route between them is computed
    by A* (pathfinding.find_path), so robots now navigate the warehouse for
    real - they just don't know or care about each other yet.
    """
    available = len(metadata["open_nodes"])
    if num_robots > available:
        print(
            f"[warning] Requested {num_robots} robots but only {available} "
            f"open start nodes are available - capping to {available}."
        )
        num_robots = available

    colors = generate_robot_colors(num_robots)
    start_nodes = rng.sample(metadata["open_nodes"], num_robots)

    robots = []
    for i, (start_node, color) in enumerate(zip(start_nodes, colors)):
        goal_node = rng.choice(metadata["pickup_nodes"])
        full_path = find_path(graph, start_node, goal_node)
        path = full_path[1:]  # drop the start node - robot is already there
        robots.append(
            Robot(robot_id=i, start_node=start_node, color=color, path=path, goal=goal_node)
        )
    return robots


def _pick_charge_nodes(metadata):
    """
    Choose two open nodes near grid corners for the charging station visual
    zones. Pure visual designation — no sim behaviour changes.
    """
    open_nodes = metadata["open_nodes"]
    width  = metadata["width"]
    height = metadata["height"]

    # Score each open node by distance to the two target corners
    def corner_score(node, cx, cy):
        return abs(node[0] - cx) + abs(node[1] - cy)

    corners = [(0, 0), (width - 1, height - 1)]
    charge_nodes = []
    used = set()
    for cx, cy in corners:
        best = min(
            (n for n in open_nodes if n not in used),
            key=lambda n: corner_score(n, cx, cy),
            default=None,
        )
        if best:
            charge_nodes.append(best)
            used.add(best)
    return charge_nodes


# ─────────────────────────────────────────────────────────────────────────────
# Main run loop
# ─────────────────────────────────────────────────────────────────────────────

def run(num_robots=DEFAULT_NUM_ROBOTS, seed=None, baseline=False, duration=None):
    """
    baseline: if True, robots use plain stop-and-wait instead of priority
        negotiation (see collision.py). Same seed still produces the same
        warehouse layout and robot spawns either way, for fair comparison.
    duration: if set, number of ticks to run before stopping automatically
        (in addition to closing the window). None runs until closed.
    """
    pygame.init()
    mode_label = "baseline" if baseline else "smart"
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption(f"SwarmSync — Warehouse Simulation ({mode_label})")
    clock = pygame.time.Clock()

    font       = pygame.font.SysFont(None, 15)
    hud_font   = pygame.font.SysFont(None, 17)
    hud_title  = pygame.font.SysFont(None, 22, bold=True)
    robot_font = pygame.font.SysFont(None, 16)

    # Try to get a Unicode font for the ⚡ icon; fall back to system default
    try:
        icon_font = pygame.font.SysFont("segoe ui emoji", 14)
    except Exception:
        icon_font = pygame.font.SysFont(None, 14)

    graph, metadata = build_warehouse_graph(seed=seed)
    positions = compute_layout(metadata["width"], metadata["height"])
    charge_nodes = _pick_charge_nodes(metadata)

    # Pre-allocate an alpha surface for zone overlays (reused each frame)
    alpha_surf = pygame.Surface((WAREHOUSE_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)

    rng    = random.Random(seed)
    robots = spawn_robots(graph, metadata, rng, num_robots)

    metrics = Metrics(mode_label)

    # Event log — capped deque so we never accumulate unbounded memory
    event_log = collections.deque(maxlen=MAX_LOG_ENTRIES * 4)

    # Toast state
    toast_event  = None
    toast_frames = 0

    tick = 0
    try:
        running = True
        while running and (duration is None or tick < duration):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            prev_log_len = len(event_log)
            resolve_and_update(
                robots, graph,
                baseline=baseline,
                metrics=metrics,
                tick=tick,
                event_log=event_log,
            )

            # If a new event was appended this tick, start/refresh the toast
            if len(event_log) > prev_log_len:
                toast_event  = event_log[-1]
                toast_frames = TOAST_DURATION

            for robot in robots:
                if robot.has_arrived():
                    assign_new_goal(robot, graph, metadata, rng, metrics, tick)

            # ── Draw frame ────────────────────────────────────────────────────
            draw_warehouse_floor(screen, graph, positions, metadata, charge_nodes, alpha_surf)
            draw_robots(screen, robots, positions, robot_font)
            draw_zone_labels(screen, positions, metadata, charge_nodes, hud_font, icon_font)
            draw_hud(screen, robots, hud_font, hud_title)
            draw_event_log(screen, event_log, len(robots), font, hud_font)

            if toast_frames > 0:
                draw_toast(screen, toast_event, toast_frames, font, hud_font)
                toast_frames -= 1

            pygame.display.flip()
            clock.tick(60)
            tick += 1
    finally:
        # Always write results, even if the run was interrupted (Ctrl+C,
        # window force-closed, an exception mid-loop) - otherwise a
        # --duration run that gets cut short silently loses its data.
        metrics.print_summary(tick)
        metrics.write_report(tick, seed, num_robots)
        pygame.quit()
