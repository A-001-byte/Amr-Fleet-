"""Scratch copy of the PRE-refactor resolve_and_update, for a differential
test only. Not part of the app; not committed."""

from pathfinding import find_path

DEADLOCK_WAIT_THRESHOLD = 45
DETOUR_FLASH_FRAMES = 20


def resolve_and_update_OLD(robots, graph, baseline=False, metrics=None, tick=0, event_log=None):
    _check_for_collisions(robots, metrics, tick)

    occupied_by = {r.position: r.id for r in robots if r.progress == 0.0}
    claimed_by = {r.path[0]: r.id for r in robots if r.path and r.progress > 0.0}
    edges_in_use = {
        frozenset((r.position, r.path[0])) for r in robots if r.path and r.progress > 0.0
    }

    if baseline:
        order = sorted(robots, key=lambda r: r.id)
    else:
        order = sorted(robots, key=lambda r: (-r.wait_ticks, len(r.path), r.id))

    for robot in order:
        if robot.detour_flash > 0:
            robot.detour_flash -= 1

        if not robot.path:
            robot.wait_ticks = 0
            continue

        if robot.progress > 0.0:
            robot.update()
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

        if metrics is not None:
            metrics.record_wait(robot.id)

        if baseline:
            robot.wait_ticks += 1
            continue

        if robot.wait_ticks == 0:
            holder_id = occupied_by.get(target, claimed_by.get(target))
            if event_log is not None:
                holder = next((r for r in robots if r.id == holder_id), None)
                if holder is not None:
                    if holder.wait_ticks > robot.wait_ticks:
                        reason = f"R{holder_id + 1} waited longer (starvation prevention)"
                    elif len(holder.path) < len(robot.path):
                        reason = f"R{holder_id + 1} has shorter remaining path ({len(holder.path)} vs {len(robot.path)} steps)"
                    else:
                        reason = f"R{holder_id + 1} has lower ID (tie-break)"
                else:
                    reason = "holder has reservation priority"
                event_log.append({
                    "tick": tick, "type": "yield", "robot": robot.id,
                    "holder": holder_id, "node": target, "reason": reason,
                })

        robot.wait_ticks += 1

        if robot.wait_ticks >= DEADLOCK_WAIT_THRESHOLD:
            detour_node = _find_detour(robot, graph, occupied_by, claimed_by)
            if detour_node is not None:
                if event_log is not None:
                    event_log.append({
                        "tick": tick, "type": "deadlock", "robot": robot.id,
                        "wait_ticks": robot.wait_ticks, "position": robot.position,
                        "via": detour_node,
                        "reason": f"stuck {robot.wait_ticks} ticks, forced detour to {detour_node}",
                    })
                continuation = find_path(graph, detour_node, robot.goal) if robot.goal else []
                robot.path = [detour_node] + continuation[1:]
                claimed_by[detour_node] = robot.id
                edges_in_use.add(frozenset((robot.position, detour_node)))
                robot.update()
                robot.wait_ticks = 0
                robot.detour_flash = DETOUR_FLASH_FRAMES


def _check_for_collisions(robots, metrics, tick):
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
