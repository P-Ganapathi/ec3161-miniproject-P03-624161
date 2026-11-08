"""Environment for P03 (multi-package delivery robot).

Re-exports the starter world generator and problem class, and adds helpers
shared by the heuristics, the search, and the checker.
"""
import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "starter", "P03_delivery"))
from delivery import make_world, DeliveryProblem  # noqa: E402  (starter file, unmodified)


def manhattan(a, b):
    """Manhattan (L1) distance between two grid cells."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def bfs_dist(world, src):
    """True shortest walking distance (steps, walls respected) from src to every reachable cell."""
    dist = {src: 0}
    queue = deque([src])
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if (0 <= nr < world["rows"] and 0 <= nc < world["cols"]
                    and (nr, nc) not in world["walls"] and (nr, nc) not in dist):
                dist[(nr, nc)] = dist[(r, c)] + 1
                queue.append((nr, nc))
    return dist


def make_problem(rows=12, cols=12, k=4, seed=0, wall_density=0.2):
    """Build a seeded instance with k packages and return a DeliveryProblem."""
    world = make_world(rows=rows, cols=cols, n_packages=k,
                       wall_density=wall_density, seed=seed)
    return DeliveryProblem(world)
