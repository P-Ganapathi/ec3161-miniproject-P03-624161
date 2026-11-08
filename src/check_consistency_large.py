"""Consistency check on LARGER instances (no exact costs needed).

Consistency means h(s) <= 1 + h(s') for every move s -> s' between reachable states.
Exact cost-to-go is too expensive at this size, so we only check this inequality over
every reachable state. This complements src/checker.py, which also checks admissibility
exactly on small grids.

Run:  python src/check_consistency_large.py   (writes results/consistency_large.txt)
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from env import make_problem                              # noqa: E402
from agents import h_far, h_mst, h_mst_plain              # noqa: E402
from checker import reachable_states                      # noqa: E402

CONFIGS = [  # (rows, cols, k, seeds)
    (12, 12, 4, range(10)),
    (12, 12, 6, range(5)),
    (12, 12, 8, range(3)),
    (10, 10, 7, range(3)),
]


def consistency_violations(problem, h, states):
    count, worst = 0, 0
    for s in states:
        hv = h(problem, s)
        for a in problem.actions(s):
            drop = hv - (1 + h(problem, problem.result(s, a)))
            if drop > 0:
                count += 1
                worst = max(worst, drop)
    return count, worst


def run():
    heuristics = {"h_far": h_far, "h_mst": h_mst, "h_mst_plain": h_mst_plain}
    totals = {name: dict(viol=0, worst=0, instances=0, states=0) for name in heuristics}
    lines = ["Consistency on larger instances: h(s) <= 1 + h(s') over every reachable state.",
             "instance: rows x cols, k packages, seed", ""]
    t0 = time.time()
    for rows, cols, k, seeds in CONFIGS:
        for seed in seeds:
            problem = make_problem(rows=rows, cols=cols, k=k, seed=seed)
            states = reachable_states(problem)
            parts = []
            for name, h in heuristics.items():
                v, w = consistency_violations(problem, h, states)
                t = totals[name]
                t["viol"] += v
                t["worst"] = max(t["worst"], w)
                t["instances"] += 1
                t["states"] += len(states)
                parts.append(f"{name}={v}")
            lines.append(f"{rows}x{cols} k={k} seed={seed} states={len(states)}  violations: " + "  ".join(parts))
    lines += ["", "TOTALS:"]
    for name, t in totals.items():
        lines.append(f"  {name:12s} instances={t['instances']}  reachable states checked={t['states']}  "
                     f"consistency_violations={t['viol']}  worst drop={t['worst']}")
    lines.append(f"time: {time.time() - t0:.1f} s")
    text = "\n".join(lines) + "\n"
    out = os.path.join(HERE, "..", "results", "consistency_large.txt")
    with open(out, "w") as f:
        f.write(text)
    print(text)


if __name__ == "__main__":
    run()
