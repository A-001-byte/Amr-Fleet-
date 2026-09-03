"""
main.py

Entry point for the SwarmSync warehouse simulation.
"""

import argparse

from simulation import DEFAULT_NUM_ROBOTS, run


def parse_args():
    parser = argparse.ArgumentParser(description="SwarmSync warehouse simulation")
    parser.add_argument(
        "--robots",
        type=int,
        default=DEFAULT_NUM_ROBOTS,
        help=f"number of robots to simulate (default: {DEFAULT_NUM_ROBOTS})",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="random seed for a reproducible warehouse layout and robot routes (default: random)",
    )
    parser.add_argument(
        "--baseline",
        action="store_true",
        help="use simple stop-and-wait collision handling instead of priority negotiation",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="stop the simulation after this many seconds (default: run until window closed)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    duration_ticks = None if args.duration is None else int(args.duration * 60)  # sim runs at 60 ticks/sec
    run(num_robots=args.robots, seed=args.seed, baseline=args.baseline, duration=duration_ticks)
