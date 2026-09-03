"""
pathfinding.py

Thin wrapper around NetworkX's A* implementation for computing robot routes
across the warehouse graph. Blocked nodes are excluded entirely from the
graph's edges (see warehouse.py), so a plain shortest-path search over the
graph automatically respects them.
"""

import math

import networkx as nx


def _euclidean_heuristic(node_a, node_b):
    """Straight-line distance heuristic for A*, using grid (x, y) coords."""
    ax, ay = node_a
    bx, by = node_b
    return math.hypot(ax - bx, ay - by)


def find_path(graph, start_node, goal_node):
    """
    Compute a path from start_node to goal_node using A*.

    Args:
        graph: the warehouse networkx.Graph (blocked nodes have no edges).
        start_node: (x, y) tuple to start from.
        goal_node: (x, y) tuple to reach.

    Returns:
        A list of nodes representing the path from start_node to goal_node
        (inclusive of both endpoints), or an empty list if no path exists.
    """
    try:
        return nx.astar_path(
            graph, start_node, goal_node, heuristic=_euclidean_heuristic
        )
    except nx.NetworkXNoPath:
        return []
