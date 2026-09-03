"""
headless_runner.py

A Pygame-free driver around the existing simulation logic, for use by the
dashboard, the fault-tolerance demo, and the scale test. It does not
reimplement or alter movement, collision, negotiation, or deadlock logic -
it just calls the same functions simulation.py's Pygame loop calls
(collision.resolve_and_update, simulation.spawn_robots,
simulation.assign_new_goal), without any rendering.

Importing simulation.py here is safe: pygame.init()/display creation only
happens inside simulation.run(), never at import time or in the functions
we reuse.
"""

import random

from collision import resolve_and_update
from metrics import Metrics
from simulation import assign_new_goal, spawn_robots
from warehouse import build_warehouse_graph

# How often (in ticks) to sample throughput for the history line chart /
# scale-test snapshots. 30 ticks = every ~0.5s at the project's 60-ticks/sec
# convention (see main.py's --duration conversion).
THROUGHPUT_SAMPLE_INTERVAL = 30


class SimRunner:
    """
    One running instance of the warehouse simulation (either baseline or
    smart mode), advanced one tick at a time via step(). Tracks everything
    needed for the dashboard and the analysis scripts: live robot state,
    a metrics object (identical to the Pygame path's), a throughput-over-
    time history, and the tick at which each task completed (for computing
    before/after windows around an event like a robot being killed).
    """

    def __init__(self, num_robots, seed, baseline, mode_label=None):
        self.baseline = baseline
        self.mode_label = mode_label or ("baseline" if baseline else "smart")
        self.seed = seed

        self.graph, self.metadata = build_warehouse_graph(seed=seed)
        self.rng = random.Random(seed)
        self.robots = spawn_robots(self.graph, self.metadata, self.rng, num_robots)

        self.metrics = Metrics(self.mode_label)
        self.tick = 0
        self.throughput_history = []  # list of (tick, throughput_per_1000)
        self.completion_ticks = []  # tick number of every completed task, in order
        self.killed_robot_ids = set()
        self.event_log = []  # negotiation/deadlock events, from collision.py

        self._last_completed_count = 0

    def step(self):
        resolve_and_update(
            self.robots,
            self.graph,
            baseline=self.baseline,
            metrics=self.metrics,
            tick=self.tick,
            event_log=self.event_log,
        )
        for robot in self.robots:
            if robot.has_arrived():
                assign_new_goal(robot, self.graph, self.metadata, self.rng, self.metrics, self.tick)

        completed_now = self.metrics.total_tasks_completed
        if completed_now > self._last_completed_count:
            self.completion_ticks.extend([self.tick] * (completed_now - self._last_completed_count))
            self._last_completed_count = completed_now

        self.tick += 1
        if self.tick % THROUGHPUT_SAMPLE_INTERVAL == 0:
            self.throughput_history.append(
                (self.tick, self.metrics.tasks_completed_per_1000_ticks(self.tick))
            )

    def run_for(self, num_ticks):
        for _ in range(num_ticks):
            self.step()

    def kill_robot(self, robot_id):
        """
        Simulate a robot going offline/disconnected: remove it from the
        active fleet. No special-casing needed elsewhere - collision.py's
        resolve_and_update() just operates on whatever robots list it's
        given, so the rest of the fleet keeps negotiating around each other
        exactly as before, unaware anything changed.
        """
        before_count = len(self.robots)
        self.robots = [r for r in self.robots if r.id != robot_id]
        removed = len(self.robots) < before_count
        if removed:
            self.killed_robot_ids.add(robot_id)
        return removed

    def throughput_in_window(self, start_tick, end_tick):
        """Tasks completed in [start_tick, end_tick), scaled to per-1000-ticks."""
        span = end_tick - start_tick
        if span <= 0:
            return 0.0
        count = sum(1 for t in self.completion_ticks if start_tick <= t < end_tick)
        return count / span * 1000

    def snapshot(self):
        """A JSON-serializable snapshot of current state, for the dashboard."""
        robots_state = []
        for robot in self.robots:
            from_node, to_node, t = robot.get_render_node_pair()
            status = "DEADLOCK" if robot.detour_flash > 0 else (
                "WAITING" if robot.progress == 0.0 and robot.wait_ticks > 0 else "MOVING"
            )
            robots_state.append({
                "id": robot.id,
                "color": list(robot.color),
                "from": list(from_node),
                "to": list(to_node),
                "t": t,
                "status": status,
                "wait_ticks": robot.wait_ticks,
                "goal": list(robot.goal) if robot.goal else None,
            })

        return {
            "mode": self.mode_label,
            "tick": self.tick,
            "robots": robots_state,
            "killed_robot_ids": sorted(self.killed_robot_ids),
            "tasks_completed": self.metrics.total_tasks_completed,
            "total_wait_ticks": self.metrics.total_wait_ticks,
            "throughput": self.metrics.tasks_completed_per_1000_ticks(max(self.tick, 1)),
            "throughput_history": self.throughput_history[-300:],
            "collision_count": self.metrics.collision_count,
            "recent_events": self.event_log[-8:],
        }

    def graph_layout(self):
        """Static graph description (nodes + edges), sent once to the dashboard."""
        nodes = []
        for node, data in self.graph.nodes(data=True):
            kind = "blocked" if data["blocked"] else ("pickup" if data["pickup"] else "open")
            nodes.append({"x": node[0], "y": node[1], "kind": kind})
        edges = [[list(a), list(b)] for a, b in self.graph.edges()]
        return {
            "width": self.metadata["width"],
            "height": self.metadata["height"],
            "nodes": nodes,
            "edges": edges,
        }
