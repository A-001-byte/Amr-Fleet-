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
            The walkable subgraph is always fully connected (see
            _ensure_connected) - random blocking that would otherwise wall
            off a pocket of the grid has a few of its nodes unblocked instead.
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

    walkable_nodes = set(n for n in all_nodes if n not in blocked_nodes)

    def _add_walkable_edges(node):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (node[0] + dx, node[1] + dy)
            if neighbor in graph and neighbor in walkable_nodes:
                graph.add_edge(node, neighbor)

    for node in walkable_nodes:
        _add_walkable_edges(node)

    _ensure_connected(graph, walkable_nodes, blocked_nodes, width, height, _add_walkable_edges)

    pickup_nodes = set(rng.sample(sorted(walkable_nodes), num_pickup_points))
    for node in pickup_nodes:
        graph.nodes[node]["pickup"] = True

    open_nodes = [n for n in walkable_nodes if n not in pickup_nodes]

    metadata = {
        "blocked_nodes": list(blocked_nodes),
        "pickup_nodes": list(pickup_nodes),
        "open_nodes": open_nodes,
        "width": width,
        "height": height,
    }

    return graph, metadata


def _ensure_connected(graph, walkable_nodes, blocked_nodes, width, height, add_walkable_edges):
    """
    Random blocking can accidentally wall off part of the grid into an
    unreachable pocket (rare, but seed-dependent - confirmed happening in
    practice, e.g. single-node islands). A robot that spawns or is routed
    into such a pocket can never reach a pickup point, since no path exists.

    This walks the walkable subgraph's connected components and, while more
    than one exists, unblocks the shortest possible chain of blocked nodes
    (found via a full-grid shortest path, ignoring blocked status) between
    the smallest component and the largest, repeating until everything is
    reachable from everything else. Mutates walkable_nodes/blocked_nodes and
    graph node attributes/edges in place.
    """
    full_grid = nx.grid_2d_graph(width, height)

    while True:
        subgraph = graph.subgraph(walkable_nodes)
        components = sorted(nx.connected_components(subgraph), key=len, reverse=True)
        if len(components) <= 1:
            return

        main_component = components[0]
        stranded_component = components[1]
        main_anchor = min(main_component)  # deterministic given the same seed/layout

        best_path = None
        for source in sorted(stranded_component):
            try:
                candidate = nx.shortest_path(full_grid, source, main_anchor)
            except nx.NetworkXNoPath:
                continue  # not possible on a fully-built grid, but stay safe
            if best_path is None or len(candidate) < len(best_path):
                best_path = candidate

        if best_path is None:
            return  # shouldn't happen on a connected full grid; avoid looping forever

        for node in best_path:
            if node in blocked_nodes:
                blocked_nodes.discard(node)
                walkable_nodes.add(node)
                graph.nodes[node]["blocked"] = False
                add_walkable_edges(node)
