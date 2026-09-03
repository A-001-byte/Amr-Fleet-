"""
dashboard_server.py

Live side-by-side comparison dashboard: runs a baseline (stop-and-wait) and
a smart (negotiation) SimRunner in background threads, both stepping in
real time, and serves their live state over a small JSON API for
dashboard.html to poll and render.

This file does not implement any simulation/collision/negotiation logic
itself - it only drives headless_runner.SimRunner, which in turn calls the
existing, unmodified collision.py and simulation.py helper functions.

Run with: python dashboard_server.py [--robots N] [--seed N]
Then open http://127.0.0.1:5000/ in a browser.
"""

import argparse
import threading
import time

from flask import Flask, jsonify, send_from_directory

from headless_runner import SimRunner

TICKS_PER_SECOND = 60  # matches the project's existing tick convention

app = Flask(__name__)

# Populated in main() before the server starts.
runners = {}
_lock = threading.Lock()
_stop_event = threading.Event()


def _step_loop(mode_label):
    """Background thread: advance one SimRunner at a steady real-time pace."""
    interval = 1.0 / TICKS_PER_SECOND
    next_tick_time = time.monotonic()
    while not _stop_event.is_set():
        with _lock:
            runners[mode_label].step()
        next_tick_time += interval
        sleep_time = next_tick_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_tick_time = time.monotonic()  # fell behind, resync


@app.route("/")
def index():
    return send_from_directory(".", "dashboard.html")


@app.route("/api/layout")
def api_layout():
    # Both runners share the same seed, so the warehouse layout is identical -
    # send the baseline one's.
    with _lock:
        return jsonify(runners["baseline"].graph_layout())


@app.route("/api/state")
def api_state():
    with _lock:
        return jsonify({
            "baseline": runners["baseline"].snapshot(),
            "smart": runners["smart"].snapshot(),
        })


@app.route("/api/kill/<mode>/<int:robot_id>", methods=["POST"])
def api_kill(mode, robot_id):
    if mode not in runners:
        return jsonify({"error": f"unknown mode '{mode}'"}), 404
    with _lock:
        removed = runners[mode].kill_robot(robot_id)
    return jsonify({"removed": removed, "robot_id": robot_id, "mode": mode})


def main():
    parser = argparse.ArgumentParser(description="SwarmSync live comparison dashboard")
    parser.add_argument("--robots", type=int, default=5, help="number of robots per fleet (default: 5)")
    parser.add_argument("--seed", type=int, default=42, help="shared random seed for both fleets (default: 42)")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    runners["baseline"] = SimRunner(num_robots=args.robots, seed=args.seed, baseline=True, mode_label="baseline")
    runners["smart"] = SimRunner(num_robots=args.robots, seed=args.seed, baseline=False, mode_label="smart")

    threads = [
        threading.Thread(target=_step_loop, args=("baseline",), daemon=True),
        threading.Thread(target=_step_loop, args=("smart",), daemon=True),
    ]
    for t in threads:
        t.start()

    print(f"[dashboard] Running {args.robots} robots/fleet, seed={args.seed}")
    print(f"[dashboard] Open http://127.0.0.1:{args.port}/ in your browser")
    try:
        app.run(host="127.0.0.1", port=args.port, threaded=True)
    finally:
        _stop_event.set()


if __name__ == "__main__":
    main()
