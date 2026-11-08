"""P03 starter: a robot on a grid must pick up every package and return to the dock.
State = (position, frozenset of packages still to collect). Unit step cost."""
import random

def make_world(rows=12, cols=12, n_packages=6, wall_density=0.2, seed=0):
    rng = random.Random(seed)
    while True:
        walls = {(r, c) for r in range(rows) for c in range(cols) if rng.random() < wall_density}
        free = [(r, c) for r in range(rows) for c in range(cols) if (r, c) not in walls]
        dock = free[0]
        pk = frozenset(rng.sample(free[1:], n_packages))
        if _all_reachable(dock, pk, walls, rows, cols):
            return dict(rows=rows, cols=cols, walls=walls, dock=dock, packages=pk)

def _all_reachable(start, targets, walls, R, C):
    seen, stack = {start}, [start]
    while stack:
        r, c = stack.pop()
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if 0 <= nr < R and 0 <= nc < C and (nr, nc) not in walls and (nr, nc) not in seen:
                seen.add((nr, nc)); stack.append((nr, nc))
    return targets <= seen

class DeliveryProblem:
    def __init__(self, w): self.w = w
    def initial(self): return (self.w["dock"], self.w["packages"])
    def goal_test(self, s): return not s[1] and s[0] == self.w["dock"]
    def actions(self, s):
        (r, c), _ = s
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):   # Up, Down, Left, Right
            if 0 <= nr < self.w["rows"] and 0 <= nc < self.w["cols"] and (nr, nc) not in self.w["walls"]:
                yield (nr, nc)
    def result(self, s, a): return (a, s[1] - {a})

# TODO (students): heuristics h0 (=0), h_far (max manhattan to a package + back),
#                  h_mst (MST over remaining packages + dock); prove/test admissibility and consistency.

if __name__ == "__main__":
    w = make_world()
    p = DeliveryProblem(w)
    print("dock", w["dock"], "packages", sorted(w["packages"]), "walls", len(w["walls"]))
    print("first actions:", list(p.actions(p.initial())))
