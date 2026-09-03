"""
warehouse.py

Defines the warehouse layout as a NetworkX graph.

The warehouse is modeled as a grid of nodes (aisle intersections). Each node
is connected to its orthogonal neighbors (up/down/left/right) to form the
aisle network. Some nodes are "blocked" (shelving/obstacles, not walkable),
and a handful of open nodes are designated as pickup/drop points.

This module intentionally knows nothing about rendering or robots — it just
builds and describes the graph. Keep it that way so it stays reusable for
collision/negotiation logic later.
"""

import random

import networkx as nx

GRID_WIDTH = 12
GRID_HEIGHT = 10
BLOCKED_RATIO = 0.15
NUM_PICKUP_POINTS = 5


def build_warehouse_graph(
    width=GRID_WIDTH,
    height=GRID_HEIGHT,
    blocked_ratio=BLOCKED_RATIO,
    num_pickup_points=NUM_PICKUP_POINTS,
    seed=None,
):
    """
    Build a grid-based warehouse graph.

    Args:
        width: number of columns in the grid.
        height: number of rows in the grid.
        blocked_ratio: fraction of nodes to mark as blocked (obstacles).
        num_pickup_points: number of open nodes to designate as pickup/drop points.
        seed: optional random seed for reproducibility.

    Returns:
        A tuple (graph, metadata) where:
          - graph is a networkx.Graph. Every grid cell is a node (even blocked
            ones), keyed by (x, y). Node attributes include "blocked" (bool)
            and "pickup" (bool). Edges only exist between walkable neighbors.
          - metadata is a dict with:
              "blocked_nodes": list of (x, y) blocked node coordinates
              "pickup_nodes": list of (x, y) pickup/drop node coordinates
              "open_nodes": list of (x, y) walkable, non-pickup node coordinates
              "width": grid width
              "height": grid height
    """
    rng = random.Random(seed)
    graph = nx.Graph()

    all_nodes = [(x, y) for y in range(height) for x in range(width)]
    for node in all_nodes:
        graph.add_node(node, blocked=False, pickup=False)

    num_blocked = int(len(all_nodes) * blocked_ratio)
    blocked_nodes = set(rng.sample(all_nodes, num_blocked))
    for node in blocked_nodes:
        graph.nodes[node]["blocked"] = True

    walkable_nodes = [n for n in all_nodes if n not in blocked_nodes]
    pickup_nodes = set(rng.sample(walkable_nodes, num_pickup_points))
    for node in pickup_nodes:
        graph.nodes[node]["pickup"] = True

    # Connect each walkable node to its walkable orthogonal neighbors.
    for x, y in walkable_nodes:
        for dx, dy in ((1, 0), (0, 1)):
            neighbor = (x + dx, y + dy)
            if neighbor in graph and neighbor not in blocked_nodes:
                graph.add_edge((x, y), neighbor)

    open_nodes = [n for n in walkable_nodes if n not in pickup_nodes]

    metadata = {
        "blocked_nodes": list(blocked_nodes),
        "pickup_nodes": list(pickup_nodes),
        "open_nodes": open_nodes,
        "width": width,
        "height": height,
    }

    return graph, metadata
