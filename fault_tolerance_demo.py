"""
fault_tolerance_demo.py

Demonstrates fleet resilience: run smart (negotiation) mode, kill one robot
mid-run, and measure throughput in the 60 real-seconds (3600 ticks, at the
project's 60-ticks/sec convention) immediately before vs immediately after
the kill. If the fleet is actually resilient, throughput after the kill
should stay roughly comparable to before - not collapse - since the
remaining robots keep completing tasks on the pickup points the killed
robot isn't around to interfere with.

This does not modify collision.py, robot.py, or metrics.py - it drives the
same unmodified SimRunner used by the dashboard (see headless_runner.py).

Run with: python fault_tolerance_demo.py [--robots N] [--seed N] [--kill-robot ID]
"""

import argparse

from headless_runner import SimRunner

WINDOW_TICKS = 3600  # 60 seconds at 60 ticks/sec


def main():
    parser = argparse.ArgumentParser(description="Fault-tolerance (robot kill) demo")
    parser.add_argument("--robots", type=int, default=8)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--kill-robot", type=int, default=0, help="id of the robot to kill mid-run")
    parser.add_argument(
        "--kill-at", type=int, default=WINDOW_TICKS,
        help=f"tick at which to kill the robot (default: {WINDOW_TICKS}, i.e. 60s in)",
    )
    parser.add_argument(
        "--after", type=int, default=WINDOW_TICKS,
        help=f"how many ticks to keep running after the kill (default: {WINDOW_TICKS}, i.e. 60s)",
    )
    args = parser.parse_args()

    total_ticks = args.kill_at + args.after

    print(f"=== Fault-Tolerance Test (smart/negotiation mode) ===")
    print(f"Robots: {args.robots}, seed: {args.seed}")
    print(f"Killing Robot {args.kill_robot} at tick {args.kill_at} "
          f"({args.kill_at / 60:.0f}s), running {args.after} more ticks "
          f"({args.after / 60:.0f}s) after.\n")

    runner = SimRunner(num_robots=args.robots, seed=args.seed, baseline=False, mode_label="smart")

    if args.kill_robot not in [r.id for r in runner.robots]:
        print(f"[error] Robot {args.kill_robot} does not exist (fleet has ids 0..{args.robots - 1})")
        return

    runner.run_for(args.kill_at)
    tasks_before_kill = runner.metrics.total_tasks_completed

    removed = runner.kill_robot(args.kill_robot)
    print(f"[event] tick {runner.tick}: Robot {args.kill_robot} killed/disconnected "
          f"(removed={removed}). Remaining fleet: {[r.id for r in runner.robots]}\n")

    runner.run_for(args.after)

    throughput_before = runner.throughput_in_window(args.kill_at - WINDOW_TICKS, args.kill_at)
    throughput_after = runner.throughput_in_window(args.kill_at, args.kill_at + WINDOW_TICKS)
    tasks_after_kill = runner.metrics.total_tasks_completed - tasks_before_kill

    print("=== Results ===")
    print(f"Total ticks run: {runner.tick}")
    print(f"Total tasks completed overall: {runner.metrics.total_tasks_completed}")
    print(f"Tasks completed in the {WINDOW_TICKS}-tick window BEFORE the kill: "
          f"{sum(1 for t in runner.completion_ticks if args.kill_at - WINDOW_TICKS <= t < args.kill_at)}")
    print(f"Tasks completed in the {WINDOW_TICKS}-tick window AFTER the kill: "
          f"{sum(1 for t in runner.completion_ticks if args.kill_at <= t < args.kill_at + WINDOW_TICKS)}")
    print(f"Throughput BEFORE kill (tasks / 1000 ticks): {throughput_before:.2f}")
    print(f"Throughput AFTER kill  (tasks / 1000 ticks): {throughput_after:.2f}")
    if throughput_before > 0:
        change_pct = (throughput_after - throughput_before) / throughput_before * 100
        print(f"Change: {change_pct:+.1f}%")
    print(f"Collision count (sanity check, should be 0): {runner.metrics.collision_count}")
    print()
    print("Architecture note: this test shows the remaining robots are not")
    print("dependent on any other individual robot to keep negotiating and")
    print("completing tasks. It does NOT by itself prove the coordination logic")
    print("runs in a distributed/decentralized fashion at the systems level -")
    print("see the report for that distinction.")


if __name__ == "__main__":
    main()
