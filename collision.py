"""
collision.py

Node-reservation collision avoidance for robots sharing the warehouse graph,
plus simple deadlock detection and resolution.

Each tick, before a robot is allowed to start moving toward the next node in
its path, it must "claim" that node - if another robot already occupies it,
or has already claimed it this tick, the robot waits instead of stepping
into a conflict.

Beyond simple node conflicts, we also guard against edge swaps: two robots
crossing the same edge in opposite directions at the same time (A: X->Y
while B: Y->X). A node-only check misses this, because a robot mid-hop away
from a node no longer counts as "occupying" it - so without this guard two
robots would glide straight through each other along a shared line, which
isn't physically possible for real AMRs.

Priority for contested nodes is decided by, in order:
  1. Wait-time aging - a robot that has been waiting longer gets priority.
     This prevents starvation (a robot never waits forever while others
     keep cutting in front of it).
  2. Remaining route length - among robots that have waited equally long,
     the one closer to finishing its delivery wins. This mirrors real fleet
     dispatch heuristics: don't stall a robot that's almost done behind one
     that just started a long haul, since that wastes more total travel time.
  3. Robot id, as a final deterministic tie-break.

This alone prevents starvation, but it can't break a symmetric head-on
standoff (two robots each waiting on the node the other currently occupies)
- priority ordering doesn't help there because neither node ever becomes
free on its own. For that case, once
a robot's wait time crosses DEADLOCK_WAIT_THRESHOLD, it's flagged as
deadlocked and takes a one-node detour onto any free neighboring cell,
physically vacating the contested spot so the standoff resolves. The
detour node is picked for being free right now, not for being adjacent to
whatever came next in the old route - so once the robot reaches it, we
re-run A* from there to the robot's actual goal rather than resuming the
stale path. Otherwise the path could jump to a node with no real edge to
it, which renders as the robot cutting a diagonal line across the grid.

BASELINE MODE (baseline=True): all of the above (priority ordering, wait-time
aging, deadlock detours) is switched off. Robots are processed in plain id
order, and a blocked robot simply halts and rechecks every tick until its
target is free - this is the traditional "stop-and-wait" behavior the smart
mode is benchmarked against. The underlying node/edge reservation checks
(the actual physical collision prevention) still apply in both modes -
baseline only removes the *negotiation* on top of it.
"""

from pathfinding import find_path

DEADLOCK_WAIT_THRESHOLD = 45  # ticks stuck before we call it a deadlock (~0.75s at 60fps)
DETOUR_FLASH_FRAMES = 20  # how long to visually flag a robot that just detoured


def resolve_and_update(robots, graph, baseline=False, metrics=None, tick=0):
    """
    Advance all robots by one tick, respecting node reservations and (in
    smart mode) breaking deadlocks when detected.

    baseline: if True, use plain stop-and-wait - fixed id order, no
        wait-time priority, no deadlock detour.
    metrics: optional Metrics instance to log wait ticks and any detected
        collisions (see metrics.py).
    tick: current simulation tick, passed through for metrics logging.
    """
    _check_for_collisions(robots, metrics, tick)

    # Nodes currently occupied by a robot at rest (not mid-hop).
    occupied_by = {r.position: r.id for r in robots if r.progress == 0.0}

    # Nodes already claimed as a target this tick (mid-hop robots keep
    # their existing claim on the node they're heading to).
    claimed_by = {
        r.path[0]: r.id for r in robots if r.path and r.progress > 0.0
    }

    # Edges currently being traversed by a mid-hop robot, so we can block
    # anyone trying to cross the same edge in the opposite direction.
    edges_in_use = {
        frozenset((r.position, r.path[0]))
        for r in robots
        if r.path and r.progress > 0.0
    }

    if baseline:
        # Stop-and-wait: no priority scheme, just a fixed, arbitrary order.
        order = sorted(robots, key=lambda r: r.id)
    else:
        # Priority: longest-waiting first (fairness), then shortest-remaining-
        # route first (throughput), then id as a final tie-break.
        order = sorted(robots, key=lambda r: (-r.wait_ticks, len(r.path), r.id))

    for robot in order:
        if robot.detour_flash > 0:
            robot.detour_flash -= 1

        if not robot.path:
            robot.wait_ticks = 0
            continue

        if robot.progress > 0.0:
            robot.update()  # already committed, just continue the hop
            robot.wait_ticks = 0
            continue

        target = robot.path[0]
        blocked_by_occupant = target in occupied_by and occupied_by[target] != robot.id
        blocked_by_claim = target in claimed_by and claimed_by[target] != robot.id
        blocked_by_edge_swap = frozenset((robot.position, target)) in edges_in_use

        if not (blocked_by_occupant or blocked_by_claim or blocked_by_edge_swap):
            claimed_by[target] = robot.id
            edges_in_use.add(frozenset((robot.position, target)))
            robot.update()
            robot.wait_ticks = 0
            continue

        # Blocked - halt and recheck next tick. Baseline mode stops here:
        # no negotiation, no deadlock escape, it just keeps waiting forever
        # if the standoff never clears on its own.
        if metrics is not None:
            metrics.record_wait(robot.id)

        if baseline:
            robot.wait_ticks += 1
            continue

        if robot.wait_ticks == 0:
            holder_id = occupied_by.get(target, claimed_by.get(target))
            print(f"[negotiate] Robot {robot.id} yields to Robot {holder_id} at node {target}")

        robot.wait_ticks += 1

        if robot.wait_ticks >= DEADLOCK_WAIT_THRESHOLD:
            detour_node = _find_detour(robot, graph, occupied_by, claimed_by)
            if detour_node is not None:
                print(
                    f"[deadlock] Robot {robot.id} stuck for {robot.wait_ticks} "
                    f"ticks at {robot.position} -> detouring via {detour_node}"
                )
                # Re-plan from the detour node to the robot's real goal, rather
                # than splicing it onto the stale path - the old path's next
                # node might not even be adjacent to the detour node.
                continuation = find_path(graph, detour_node, robot.goal) if robot.goal else []
                robot.path = [detour_node] + continuation[1:]
                claimed_by[detour_node] = robot.id
                edges_in_use.add(frozenset((robot.position, detour_node)))
                robot.update()
                robot.wait_ticks = 0
                robot.detour_flash = DETOUR_FLASH_FRAMES


def _check_for_collisions(robots, metrics, tick):
    """
    Sanity check: verify no two robots are resting at the same node. Should
    never trigger if the reservation logic above is correct, in either mode
    - this exists purely to catch regressions and give an honest zero (or a
    loud red flag) in the metrics report.
    """
    if metrics is None:
        return

    seen = {}
    for robot in robots:
        if robot.progress != 0.0:
            continue
        if robot.position in seen:
            metrics.record_collision([seen[robot.position], robot.id], robot.position, tick)
        else:
            seen[robot.position] = robot.id


def _find_detour(robot, graph, occupied_by, claimed_by):
    """
    Find any neighbor of the robot's current node that isn't its blocked
    target, and isn't occupied/claimed by someone else. Returns None if
    the robot is fully boxed in.
    """
    blocked_target = robot.path[0]
    for neighbor in graph.neighbors(robot.position):
        if neighbor == blocked_target:
            continue
        if occupied_by.get(neighbor, robot.id) != robot.id:
            continue
        if claimed_by.get(neighbor, robot.id) != robot.id:
            continue
        return neighbor
    return None
