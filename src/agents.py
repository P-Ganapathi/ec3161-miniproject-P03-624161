"""Search algorithms and heuristics for P03.

A state is (pos, remaining_packages). Every move costs 1.

  h0     : zero heuristic. A* with h0 is uniform-cost search (the baseline).
  h_far  : farthest remaining package, out and back to the dock (main method 1).
  h_mst  : minimum spanning tree over {robot, remaining packages, dock} (main method 2).

  astar(problem, h, weight) : A* when weight = 1; weighted A* when weight > 1,
                              with f = g + weight * h.
"""
import heapq
import itertools
import time
from itertools import permutations

from env import manhattan, bfs_dist


# ---------- heuristics: each takes (problem, state) and returns a number ----------

def h0(problem, s):
    """Zero heuristic (admissible and consistent, but gives no guidance)."""
    return 0


def h_far(problem, s):
    """max over remaining packages p of  manhattan(pos, p) + manhattan(p, dock).

    Any tour must visit p and then end at the dock, so its length is at least
    dist(pos, p) + dist(p, dock) >= the Manhattan sum. Hence h_far never
    overestimates. If no packages remain it returns the Manhattan distance to the dock."""
    pos, rem = s
    dock = problem.w["dock"]
    if not rem:
        return manhattan(pos, dock)
    return max(manhattan(pos, p) + manhattan(p, dock) for p in rem)


def _mst_weight(nodes):
    """Prim's algorithm on the complete graph over `nodes` with Manhattan weights."""
    n = len(nodes)
    if n <= 1:
        return 0
    in_tree = [False] * n
    key = [float("inf")] * n
    key[0] = 0
    total = 0
    for _ in range(n):
        u = min((i for i in range(n) if not in_tree[i]), key=lambda i: key[i])
        in_tree[u] = True
        total += key[u]
        for v in range(n):
            if not in_tree[v]:
                d = manhattan(nodes[u], nodes[v])
                if d < key[v]:
                    key[v] = d
    return total


def h_mst_plain(problem, s):
    """FIRST ATTEMPT (kept for the report). MST over {robot, dock} + remaining packages,
    with the robot itself as a tree node. Admissible, but NOT consistent: the checker
    finds 1,090 consistency violations on the 30 small instances, because moving the
    robot changes which node is a tree node and the MST can jump by more than 1."""
    pos, rem = s
    dock = problem.w["dock"]
    return _mst_weight([pos, dock] + list(rem))


def h_mst(problem, s):
    """MAIN METHOD 2 (fixed). Connect the robot to the nearest remaining target, then add the
    MST over the targets {dock} + remaining packages:

        h(s) = min_x manhattan(pos, x)  +  MST({dock} + remaining packages),  x over the targets.

    Admissible: a tour's first visited target x is at distance >= min_x manhattan(pos, x),
    and the rest of the tour is a connected walk through all targets, so its length is
    at least the MST over the targets. Each step costs at least its Manhattan length.
    Consistent (checked empirically, see results/checker_report.txt)."""
    pos, rem = s
    dock = problem.w["dock"]
    targets = [dock] + list(rem)
    attach = min(manhattan(pos, x) for x in targets)
    return attach + _mst_weight(targets)


# ---------- search ----------

def astar(problem, h, weight=1.0):
    """A* (weight = 1) or weighted A* (weight > 1) on the delivery problem.

    Returns a dict with:
      cost         solution length in steps
      expanded     number of states expanded (the main "effort" counter)
      generated    number of states pushed onto the open list
      peak_stored  number of distinct states kept in memory (size of best-g table)
      time         wall-clock seconds
    Returns None if no goal state is reachable.

    A state is re-expanded if a cheaper path to it is found later. This keeps
    plain A* correct even if h is not consistent."""
    start = problem.initial()
    best = {start: 0}                 # best g found so far for each state
    expanded_at = {}                  # g at which each state was last expanded
    tie = itertools.count()           # tie-breaker so the heap never compares states
    heap = [(weight * h(problem, start), 0, next(tie), start)]
    expanded = 0
    generated = 1
    t0 = time.perf_counter()
    while heap:
        _, neg_g, _, s = heapq.heappop(heap)
        g = -neg_g
        if g > best[s]:
            continue                  # stale entry: a cheaper path was found after this push
        if s in expanded_at and expanded_at[s] <= g:
            continue                  # already expanded with this g (or a better one)
        if problem.goal_test(s):
            return dict(cost=g, expanded=expanded, generated=generated,
                        peak_stored=len(best), time=time.perf_counter() - t0)
        expanded_at[s] = g
        expanded += 1
        for a in problem.actions(s):
            s2 = problem.result(s, a)
            g2 = g + 1
            if s2 not in best or g2 < best[s2]:
                best[s2] = g2
                heapq.heappush(heap, (g2 + weight * h(problem, s2), -g2, next(tie), s2))
                generated += 1
    return None


# ---------- independent oracle (not a search) ----------

def optimal_by_permutation(problem):
    """Exact optimum for small k: try every order in which the packages could be
    visited, using true BFS distances between consecutive stops, and take the best.
    Used only to check the A* implementation, which it does not share code with."""
    w = problem.w
    dock = w["dock"]
    pts = sorted(w["packages"])
    dist_from = {u: bfs_dist(w, u) for u in [dock] + pts}
    best = float("inf")
    for perm in permutations(pts):
        cost, cur = 0, dock
        for p in perm:
            cost += dist_from[cur][p]
            cur = p
        cost += dist_from[cur][dock]
        best = min(best, cost)
    return best
