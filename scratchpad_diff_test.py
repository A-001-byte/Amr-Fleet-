"""
Differential test: runs the OLD (pre-refactor, global-sort) and NEW
(local-per-contested-node) resolve_and_update side by side on independently
constructed but identically-seeded robot fleets, and asserts every tick's
outcome is identical - positions, wait_ticks, task completions, event logs,
metrics. Scratch file, not part of the app, not committed.
"""

import random
import sys

from warehouse import build_warehouse_graph
from simulation import spawn_robots, assign_new_goal
from metrics import Metrics
from collision import resolve_and_update as resolve_new
from scratchpad_old_collision import resolve_and_update_OLD as resolve_old


def build_fleet(num_robots, seed):
    graph, meta = build_warehouse_graph(seed=seed)
    rng = random.Random(seed)
    robots = spawn_robots(graph, meta, rng, num_robots)
    return graph, meta, rng, robots


def run_diff(num_robots, seed, baseline, ticks):
    graph_a, meta_a, rng_a, robots_a = build_fleet(num_robots, seed)
    graph_b, meta_b, rng_b, robots_b = build_fleet(num_robots, seed)

    metrics_a = Metrics("old")
    metrics_b = Metrics("new")
    log_a, log_b = [], []

    mismatches = []

    for tick in range(ticks):
        resolve_old(robots_a, graph_a, baseline=baseline, metrics=metrics_a, tick=tick, event_log=log_a)
        resolve_new(robots_b, graph_b, baseline=baseline, metrics=metrics_b, tick=tick, event_log=log_b)

        for ra, rb in zip(robots_a, robots_b):
            if ra.has_arrived():
                assign_new_goal(ra, graph_a, meta_a, rng_a, metrics_a, tick)
            if rb.has_arrived():
                assign_new_goal(rb, graph_b, meta_b, rng_b, metrics_b, tick)

        for ra, rb in zip(robots_a, robots_b):
            if (ra.position, ra.progress, tuple(ra.path), ra.wait_ticks, ra.goal) != \
               (rb.position, rb.progress, tuple(rb.path), rb.wait_ticks, rb.goal):
                mismatches.append((tick, ra.id, "state", (ra.position, ra.progress, ra.path, ra.wait_ticks, ra.goal),
                                    (rb.position, rb.progress, rb.path, rb.wait_ticks, rb.goal)))

        if metrics_a.total_tasks_completed != metrics_b.total_tasks_completed:
            mismatches.append((tick, None, "tasks_completed", metrics_a.total_tasks_completed, metrics_b.total_tasks_completed))
        if metrics_a.total_wait_ticks != metrics_b.total_wait_ticks:
            mismatches.append((tick, None, "total_wait_ticks", metrics_a.total_wait_ticks, metrics_b.total_wait_ticks))
        if metrics_a.collision_count != metrics_b.collision_count:
            mismatches.append((tick, None, "collision_count", metrics_a.collision_count, metrics_b.collision_count))

        # (no early break - run to completion so we can see the FULL magnitude
        # of any divergence, not just its first appearance)

    return {
        "mismatches": mismatches,
        "first_mismatch_tick": mismatches[0][0] if mismatches else None,
        "ticks_run": tick + 1,
        "tasks_a": metrics_a.total_tasks_completed,
        "tasks_b": metrics_b.total_tasks_completed,
        "wait_a": metrics_a.total_wait_ticks,
        "wait_b": metrics_b.total_wait_ticks,
        "collisions_a": metrics_a.collision_count,
        "collisions_b": metrics_b.collision_count,
        "log_a_len": len(log_a),
        "log_b_len": len(log_b),
    }


def main():
    configs = [
        (4, 42, False), (4, 42, True),
        (8, 7, False), (8, 7, True),
        (5, 1, False), (5, 1, True),
        (16, 104, False), (16, 104, True),
        (16, 200, False),  # a fresh high-density seed, more contention
        (3, 99, False),
    ]
    all_ok = True
    for num_robots, seed, baseline in configs:
        result = run_diff(num_robots, seed, baseline, ticks=3000)
        mode = "baseline" if baseline else "smart"
        if result["mismatches"]:
            all_ok = False
            print(f"[MISMATCH] n={num_robots} seed={seed} mode={mode}: first diverged at tick "
                  f"{result['first_mismatch_tick']}, {len(result['mismatches'])} mismatch entries over "
                  f"{result['ticks_run']} ticks")
            print(f"    final tasks: old={result['tasks_a']} new={result['tasks_b']} "
                  f"(diff {result['tasks_b'] - result['tasks_a']:+d})")
            print(f"    final wait:  old={result['wait_a']} new={result['wait_b']} "
                  f"(diff {result['wait_b'] - result['wait_a']:+d})")
            print(f"    collisions:  old={result['collisions_a']} new={result['collisions_b']}")
            print(f"    events:      old={result['log_a_len']} new={result['log_b_len']}")
        else:
            print(f"[OK] n={num_robots} seed={seed} mode={mode}: {result['ticks_run']} ticks identical, "
                  f"tasks={result['tasks_a']}=={result['tasks_b']}, wait={result['wait_a']}=={result['wait_b']}, "
                  f"events old={result['log_a_len']} new={result['log_b_len']}")

    print()
    print("ALL IDENTICAL" if all_ok else "DIVERGENCE FOUND")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
