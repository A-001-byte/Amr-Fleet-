"""
metrics.py

Lightweight metrics tracking for comparing collision-avoidance strategies
(baseline stop-and-wait vs. smart priority negotiation). Tracks per-robot
and aggregate stats, logs incrementally to the console as the sim runs, and
writes a summary report to results/ at the end of a run for offline
comparison between modes.
"""

import csv
import os
import time

RESULTS_DIR = "results"


class Metrics:
    def __init__(self, mode_label):
        self.mode_label = mode_label  # "baseline" or "smart" - used in logs and filenames
        self.task_durations = []  # tick-duration of every completed task, across all robots
        self.per_robot_tasks_completed = {}  # robot_id -> completed task count
        self.per_robot_wait_ticks = {}  # robot_id -> cumulative ticks spent blocked
        self.collision_count = 0

    def record_task_completion(self, robot_id, duration_ticks, tick):
        self.task_durations.append(duration_ticks)
        self.per_robot_tasks_completed[robot_id] = (
            self.per_robot_tasks_completed.get(robot_id, 0) + 1
        )
        print(
            f"[metrics] tick {tick}: Robot {robot_id} completed task in {duration_ticks} ticks "
            f"(total tasks: {self.total_tasks_completed}, running avg: {self.average_task_duration:.1f} ticks)"
        )

    def record_wait(self, robot_id):
        self.per_robot_wait_ticks[robot_id] = self.per_robot_wait_ticks.get(robot_id, 0) + 1

    def record_collision(self, robot_ids, node, tick):
        self.collision_count += 1
        print(f"[RED FLAG] Collision detected at tick {tick}: robots {robot_ids} both at {node}")

    @property
    def total_tasks_completed(self):
        return len(self.task_durations)

    @property
    def average_task_duration(self):
        if not self.task_durations:
            return 0.0
        return sum(self.task_durations) / len(self.task_durations)

    @property
    def total_wait_ticks(self):
        return sum(self.per_robot_wait_ticks.values())

    def tasks_completed_per_1000_ticks(self, total_ticks):
        """
        Throughput: tasks completed per 1000 ticks of run time. Unlike
        average_task_duration, this is measured against total elapsed time
        rather than only completed-task samples, so a fleet that's mostly
        gridlocked (few completions, but the ones that finish are quick)
        can't hide behind a misleadingly good average - it shows up here as
        low throughput, which is what actually matters for the "reduction in
        task completion time" claim.
        """
        if total_ticks <= 0:
            return 0.0
        return self.total_tasks_completed / total_ticks * 1000

    def print_summary(self, total_ticks):
        print("\n=== SwarmSync Run Summary ===")
        print(f"Mode: {self.mode_label}")
        print(f"Total ticks run: {total_ticks}")
        print(f"Total tasks completed: {self.total_tasks_completed}")
        print(f"Average task completion time: {self.average_task_duration:.2f} ticks")
        print(f"Throughput: {self.tasks_completed_per_1000_ticks(total_ticks):.2f} tasks / 1000 ticks")
        print(f"Total wait ticks (all robots): {self.total_wait_ticks}")
        flag = " <-- RED FLAG (should be 0)" if self.collision_count else ""
        print(f"Collision count: {self.collision_count}{flag}")
        print("==============================\n")

    def write_report(self, total_ticks, seed, num_robots):
        """Write a timestamped CSV report to results/ and return its path."""
        os.makedirs(RESULTS_DIR, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(RESULTS_DIR, f"{timestamp}_{self.mode_label}.csv")

        robot_ids = sorted(
            set(self.per_robot_tasks_completed) | set(self.per_robot_wait_ticks)
        )

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "value"])
            writer.writerow(["mode", self.mode_label])
            writer.writerow(["seed", seed])
            writer.writerow(["num_robots", num_robots])
            writer.writerow(["total_ticks", total_ticks])
            writer.writerow(["total_tasks_completed", self.total_tasks_completed])
            writer.writerow(["average_task_completion_ticks", f"{self.average_task_duration:.2f}"])
            writer.writerow([
                "tasks_completed_per_1000_ticks",
                f"{self.tasks_completed_per_1000_ticks(total_ticks):.2f}",
            ])
            writer.writerow(["total_wait_ticks", self.total_wait_ticks])
            writer.writerow(["collision_count", self.collision_count])
            writer.writerow([])
            writer.writerow(["robot_id", "tasks_completed", "wait_ticks"])
            for robot_id in robot_ids:
                writer.writerow([
                    robot_id,
                    self.per_robot_tasks_completed.get(robot_id, 0),
                    self.per_robot_wait_ticks.get(robot_id, 0),
                ])

        print(f"[metrics] Report written to {filename}")
        return filename
