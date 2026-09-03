"""
robot.py

Defines the Robot class used by the simulation.

A Robot knows its id, current node, color, and the path (list of nodes) it
is following. Movement is logically node-to-node (no physics, no collision
awareness) but rendered with smooth interpolation between nodes: the robot's
"position" is only updated once it fully reaches the next node, while
"progress" (0..1) tracks how far along that hop it currently is, for the
renderer to use. Collision/negotiation logic will build on top of this
later, so keep this class dumb and self-contained.
"""

# Fraction of a node-to-node hop completed per frame. At 60 FPS,
# 1 / 30 means each hop takes half a second.
DEFAULT_SPEED = 1.0 / 30.0


class Robot:
    def __init__(self, robot_id, start_node, color, path=None, goal=None, speed=DEFAULT_SPEED):
        self.id = robot_id
        self.position = start_node  # last confirmed (arrived-at) node
        self.color = color
        self.path = path or []  # remaining nodes to visit, not including current position
        self.goal = goal  # final destination node this path is heading toward
        self.progress = 0.0  # 0..1 progress toward path[0], for smooth rendering
        self.speed = speed

        # Collision-avoidance bookkeeping (see collision.py). A robot doesn't
        # know about other robots, it just tracks its own idle time so the
        # negotiation layer can reason about it.
        self.wait_ticks = 0  # consecutive ticks spent blocked, waiting to move
        self.detour_flash = 0  # frames left to visually flag a deadlock break

        # Metrics bookkeeping (see metrics.py). Tick at which the robot's
        # current task (route to its current goal) began, so task duration
        # can be computed when it arrives.
        self.task_start_tick = 0

    def set_path(self, path, goal=None):
        """Assign a new path for the robot to follow, optionally updating its goal."""
        self.path = list(path)
        self.progress = 0.0
        if goal is not None:
            self.goal = goal

    def update(self):
        """
        Advance progress toward the next node in the path. Once progress
        reaches 1.0, the robot has arrived: position snaps to that node and
        it starts progressing toward the following one.
        """
        if not self.path:
            return

        self.progress += self.speed
        if self.progress >= 1.0:
            self.progress = 0.0
            self.position = self.path.pop(0)

    def has_arrived(self):
        return len(self.path) == 0

    def get_render_node_pair(self):
        """
        Returns (from_node, to_node, t) describing where to draw the robot:
        interpolated t of the way from from_node to to_node. If the robot
        has no path left, from_node == to_node and t == 0.
        """
        if not self.path:
            return self.position, self.position, 0.0
        return self.position, self.path[0], self.progress
