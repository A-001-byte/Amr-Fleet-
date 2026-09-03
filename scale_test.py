"""
scale_test.py

Runs the existing baseline vs. smart comparison at multiple fleet sizes
(same seed, same duration per size) and reports whether the smart-vs-
baseline throughput gap widens as robot density increases - i.e. whether
the deadlock problem baseline suffers from gets proportionally worse with
more robots, and whether smart mode degrades more gracefully.

This does not modify collision.py, robot.py, or metrics.py - it drives the
same unmodified SimRunner used by the dashboard and fault-tolerance demo
(see headless_runner.py). Writes a CSV report to results/ in addition to
printing the comparison table.

Run with: python scale_test.py [--sizes 4,8,16] [--seed N] [--duration-ticks N]
"""

import argparse
import csv
import os
import time

from headless_runner import SimRunner

RESULTS_DIR = "results"
DEFAULT_DURATION_TICKS = 3600  # 60 seconds at 60 ticks/sec


# A robot cannot legitimately complete a task in fewer ticks than one hop
# takes at its speed (see robot.py's DEFAULT_SPEED, 1/30 - i.e. 30 ticks/hop
# minimum). Sanity-check threshold: if a trial's average ticks/task drops
# far below that, something is generating free (0-duration) "completions" -
# see the stranded-robot bug noted in the report, not a real result.
MIN_PLAUSIBLE_TICKS_PER_TASK = 10


def run_one(num_robots, seed, baseline, duration_ticks):
    runner = SimRunner(num_robots=num_robots, seed=seed, baseline=baseline)
    runner.run_for(duration_ticks)
    tasks = runner.metrics.total_tasks_completed
    suspect = tasks > 0 and (duration_ticks / tasks) < MIN_PLAUSIBLE_TICKS_PER_TASK
    return {
        "num_robots": num_robots,
        "mode": runner.mode_label,
        "seed": seed,
        "tasks_completed": tasks,
        "throughput": runner.metrics.tasks_completed_per_1000_ticks(duration_ticks),
        "total_wait_ticks": runner.metrics.total_wait_ticks,
        "collision_count": runner.metrics.collision_count,
        "suspect": suspect,
    }


def main():
    parser = argparse.ArgumentParser(description="Scale test: baseline vs smart at multiple fleet sizes")
    parser.add_argument("--sizes", type=str, default="4,8,16", help="comma-separated robot counts")
    parser.add_argument("--seed", type=int, default=42, help="base seed; trial i uses seed+i")
    parser.add_argument("--trials", type=int, default=1,
                         help="number of random seeds averaged per fleet size (default: 1, no averaging)")
    parser.add_argument("--duration-ticks", type=int, default=DEFAULT_DURATION_TICKS)
    args = parser.parse_args()

    sizes = [int(s) for s in args.sizes.split(",")]
    trial_seeds = [args.seed + i for i in range(args.trials)]

    # results[size]["baseline"/"smart"] = list of per-trial result dicts
    results = {n: {"baseline": [], "smart": []} for n in sizes}
    all_suspect = []
    for n in sizes:
        for seed in trial_seeds:
            print(f"--- Running {n} robots, seed {seed} (baseline) ---")
            r = run_one(n, seed, baseline=True, duration_ticks=args.duration_ticks)
            results[n]["baseline"].append(r)
            if r["suspect"]:
                all_suspect.append(r)
            print(f"--- Running {n} robots, seed {seed} (smart) ---")
            r = run_one(n, seed, baseline=False, duration_ticks=args.duration_ticks)
            results[n]["smart"].append(r)
            if r["suspect"]:
                all_suspect.append(r)

    if all_suspect:
        print("\n[WARNING] Excluding suspect trial(s) from the averages below - these show an "
              "implausibly high task rate, consistent with a robot spawning on a node the "
              "random warehouse layout accidentally isolated from the rest of the graph (see "
              "warehouse.py connectivity note in the report):")
        for r in all_suspect:
            print(f"  - {r['num_robots']} robots, seed {r['seed']}, mode={r['mode']}: "
                  f"{r['tasks_completed']} tasks in {args.duration_ticks} ticks")
        print()

    def clean(rows):
        kept = [r for r in rows if not r["suspect"]]
        return kept if kept else rows  # never divide by zero; fall back if ALL were suspect

    def avg(rows, key):
        rows = clean(rows)
        return sum(r[key] for r in rows) / len(rows)

    print("\n=== Scale Test Results ===")
    print(f"Trials per size: {args.trials} (seeds {trial_seeds}), "
          f"duration: {args.duration_ticks} ticks ({args.duration_ticks / 60:.0f}s) each\n")
    header = f"{'Robots':>7} | {'Baseline TP':>12} | {'Smart TP':>10} | {'Reduction %':>12} | {'Base Wait':>10} | {'Smart Wait':>11} | {'Collisions':>10}"
    print(header)
    print("-" * len(header))

    table_rows = []
    for n in sizes:
        base_rows = results[n]["baseline"]
        smart_rows = results[n]["smart"]
        base_tp = avg(base_rows, "throughput")
        smart_tp = avg(smart_rows, "throughput")
        base_wait = avg(base_rows, "total_wait_ticks")
        smart_wait = avg(smart_rows, "total_wait_ticks")
        collisions = sum(r["collision_count"] for r in base_rows + smart_rows)
        reduction_pct = (smart_tp - base_tp) / smart_tp * 100 if smart_tp > 0 else 0.0

        clean_base = clean(base_rows)
        if args.trials > 1:
            base_tp_range = f"[{min(r['throughput'] for r in clean_base):.1f}-{max(r['throughput'] for r in clean_base):.1f}]"
        else:
            base_tp_range = ""

        print(
            f"{n:>7} | {base_tp:>12.2f} | {smart_tp:>10.2f} | "
            f"{reduction_pct:>11.1f}% | {base_wait:>10.0f} | "
            f"{smart_wait:>11.0f} | {collisions:>10}"
        )
        if base_tp_range:
            print(f"{'':>7} | (range {base_tp_range})")

        table_rows.append({
            "num_robots": n,
            "trials": args.trials,
            "baseline_throughput_avg": round(base_tp, 2),
            "smart_throughput_avg": round(smart_tp, 2),
            "baseline_reduction_pct": round(reduction_pct, 1),
            "baseline_wait_ticks_avg": round(base_wait, 1),
            "smart_wait_ticks_avg": round(smart_wait, 1),
            "collision_count": collisions,
        })

    print(
        "\nReduction % = how much lower baseline's throughput is than smart's, at that fleet size.\n"
        "If this number climbs as robot count increases, baseline degrades faster under density,\n"
        "confirming the expectation. If it doesn't, that's reported as-is below, not smoothed over."
    )

    reductions = [r["baseline_reduction_pct"] for r in table_rows]
    if len(reductions) >= 2 and all(reductions[i] <= reductions[i + 1] for i in range(len(reductions) - 1)):
        print("\nFinding: reduction % increases monotonically with fleet size - baseline gets proportionally worse.")
    elif len(reductions) >= 2:
        print(f"\nFinding: reduction % does NOT increase monotonically with fleet size ({reductions}) - "
              f"see the real numbers above, the gap does not simply widen with scale.")

    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(RESULTS_DIR, f"{timestamp}_scale_test.csv")
    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(table_rows[0].keys()))
        writer.writeheader()
        writer.writerows(table_rows)
    print(f"\n[scale_test] Report written to {filename}")


if __name__ == "__main__":
    main()
