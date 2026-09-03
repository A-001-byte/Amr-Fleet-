"""
simulation.py

Pygame main loop and rendering for the SwarmSync warehouse simulation.
"""

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
HUD_WIDTH = 240  # sidebar panel width, to the right of the warehouse
WINDOW_WIDTH = WAREHOUSE_WIDTH + HUD_WIDTH
WINDOW_HEIGHT = 800
MARGIN = 40

BACKGROUND_COLOR = (30, 30, 30)
EDGE_COLOR = (80, 80, 80)
OPEN_NODE_COLOR = (200, 200, 200)
BLOCKED_NODE_COLOR = (150, 40, 40)
PICKUP_NODE_COLOR = (60, 180, 90)

NODE_RADIUS = 6
PICKUP_NODE_RADIUS = 9
ROBOT_RADIUS = 12
DEADLOCK_FLASH_COLOR = (255, 230, 0)
LABEL_COLOR = (20, 20, 20)

HUD_BACKGROUND_COLOR = (18, 18, 18)
HUD_DIVIDER_COLOR = (70, 70, 70)
HUD_TEXT_COLOR = (220, 220, 220)
HUD_TITLE_COLOR = (255, 255, 255)
HUD_ROW_HEIGHT = 60
HUD_PADDING = 16

STATUS_COLORS = {
    "MOVING": (110, 220, 110),
    "WAITING": (240, 200, 60),
    "DEADLOCK": (240, 90, 90),
}

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
    usable_width = WAREHOUSE_WIDTH - 2 * MARGIN
    usable_height = WINDOW_HEIGHT - 2 * MARGIN

    step_x = usable_width / max(width - 1, 1)
    step_y = usable_height / max(height - 1, 1)

    positions = {}
    for gx in range(width):
        for gy in range(height):
            px = MARGIN + gx * step_x
            py = MARGIN + gy * step_y
            positions[(gx, gy)] = (px, py)
    return positions


def draw_warehouse(screen, graph, positions):
    for a, b in graph.edges():
        pygame.draw.line(screen, EDGE_COLOR, positions[a], positions[b], 1)

    for node, data in graph.nodes(data=True):
        pos = positions[node]
        if data["blocked"]:
            pygame.draw.circle(screen, BLOCKED_NODE_COLOR, pos, NODE_RADIUS)
        elif data["pickup"]:
            pygame.draw.circle(screen, PICKUP_NODE_COLOR, pos, PICKUP_NODE_RADIUS)
        else:
            pygame.draw.circle(screen, OPEN_NODE_COLOR, pos, NODE_RADIUS)


def _lerp(a, b, t):
    return a + (b - a) * t


def draw_robots(screen, robots, positions, font):
    for robot in robots:
        from_node, to_node, t = robot.get_render_node_pair()
        x1, y1 = positions[from_node]
        x2, y2 = positions[to_node]
        pos = (_lerp(x1, x2, t), _lerp(y1, y2, t))

        if robot.detour_flash > 0:
            pygame.draw.circle(screen, DEADLOCK_FLASH_COLOR, pos, ROBOT_RADIUS + 4, width=2)

        pygame.draw.circle(screen, robot.color, pos, ROBOT_RADIUS)

        label = font.render(f"R{robot.id + 1}", True, LABEL_COLOR)
        label_rect = label.get_rect(center=pos)
        screen.blit(label, label_rect)


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


def assign_new_goal(robot, graph, metadata, rng, metrics, tick):
    """
    Pick a new random pickup/drop node (different from where the robot
    currently sits, if possible) and route the robot there via A*. Called
    whenever a robot finishes its current path, so robots keep moving
    indefinitely instead of stopping once they arrive.

    This is always called right after a robot arrives at its previous goal,
    so it doubles as the task-completion event for metrics purposes.
    """
    metrics.record_task_completion(robot.id, tick - robot.task_start_tick, tick)

    candidates = [n for n in metadata["pickup_nodes"] if n != robot.position]
    goal_node = rng.choice(candidates or metadata["pickup_nodes"])
    full_path = find_path(graph, robot.position, goal_node)
    robot.set_path(full_path[1:], goal=goal_node)  # drop the start node - robot is already there
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
    pygame.display.set_caption(f"SwarmSync - Warehouse Simulation ({mode_label})")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 16)
    hud_font = pygame.font.SysFont(None, 18)
    hud_title_font = pygame.font.SysFont(None, 24, bold=True)

    graph, metadata = build_warehouse_graph(seed=seed)
    positions = compute_layout(metadata["width"], metadata["height"])

    rng = random.Random(seed)
    robots = spawn_robots(graph, metadata, rng, num_robots)

    metrics = Metrics(mode_label)

    tick = 0
    try:
        running = True
        while running and (duration is None or tick < duration):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            resolve_and_update(robots, graph, baseline=baseline, metrics=metrics, tick=tick)
            for robot in robots:
                if robot.has_arrived():
                    assign_new_goal(robot, graph, metadata, rng, metrics, tick)

            screen.fill(BACKGROUND_COLOR)
            draw_warehouse(screen, graph, positions)
            draw_robots(screen, robots, positions, font)
            draw_hud(screen, robots, hud_font, hud_title_font)

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
