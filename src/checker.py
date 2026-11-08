"""Checker for admissibility and consistency of a heuristic.

On small worlds we compute the exact cost-to-go h*(s) for every reachable state
(value iteration). Then, for each heuristic h:
  admissible : h(s) <= h*(s) for every reachable state s
  consistent : h(s) <= 1 + h(s') for every move s -> s'; and h(goal) = 0
We also report max over states of (h(s) - h*(s)). A positive value means the
heuristic overestimates the true remaining cost somewhere (inadmissible).

Run:  python src/checker.py   (writes results/checker_report.txt)
"""
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env import make_problem          # noqa: E402
from agents import h0, h_far, h_mst, h_mst_plain   # noqa: E402


def reachable_states(problem):
    start = problem.initial()
    seen = {start}
    queue = deque([start])
    while queue:
        s = queue.popleft()
        for a in problem.actions(s):
            s2 = problem.result(s, a)
            if s2 not in seen:
                seen.add(s2)
                queue.append(s2)
    return seen


def exact_cost_to_go(problem, states):
    """Value iteration: h*(goal) = 0, otherwise h*(s) = min over moves of 1 + h*(s')."""
    INF = float("inf")
    h = {s: (0 if problem.goal_test(s) else INF) for s in states}
    changed = True
    while changed:
        changed = False
        for s in states:
            if problem.goal_test(s):
                continue
            best = h[s]
            for a in problem.actions(s):
                cand = h[problem.result(s, a)] + 1
                if cand < best:
                    best = cand
            if best < h[s]:
                h[s] = best
                changed = True
    return h


def check(problem, heuristic, states, hstar):
    adm_viol = 0
    worst_over = float("-inf")
    cons_viol = 0
    for s in states:
        hv = heuristic(problem, s)
        over = hv - hstar[s]
        worst_over = max(worst_over, over)
        if over > 0:
            adm_viol += 1
        for a in problem.actions(s):
            s2 = problem.result(s, a)
            if hv > 1 + heuristic(problem, s2):
                cons_viol += 1
    goal_ok = all(heuristic(problem, s) == 0 for s in states if problem.goal_test(s))
    return dict(states=len(states), admissible_violations=adm_viol,
                max_h_minus_hstar=worst_over, consistency_violations=cons_viol,
                goal_h_zero=goal_ok)


def run(out_path=None):
    if out_path is None:
        out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "checker_report.txt")
    heuristics = {"h0": h0, "h_far": h_far, "h_mst": h_mst, "h_mst_plain": h_mst_plain}
    lines = ["Checker: admissibility and consistency on small worlds (6x6 grid, k = 2, 3, 4; seeds 0-9)",
             "Columns: heuristic, instance, reachable states, admissibility violations,",
             "         max(h - h*), consistency violations, goal h = 0"]
    totals = {name: dict(adm=0, cons=0, worst=float("-inf"), goal=True) for name in heuristics}
    for k in (2, 3, 4):
        for seed in range(10):
            problem = make_problem(rows=6, cols=6, k=k, seed=seed)
            states = reachable_states(problem)
            hstar = exact_cost_to_go(problem, states)
            for name, fn in heuristics.items():
                r = check(problem, fn, states, hstar)
                lines.append(f"{name:6s} k={k} seed={seed}  states={r['states']:5d}  "
                             f"adm_viol={r['admissible_violations']:4d}  "
                             f"max(h-h*)={r['max_h_minus_hstar']:+.0f}  "
                             f"cons_viol={r['consistency_violations']:5d}  goal_h0={r['goal_h_zero']}")
                t = totals[name]
                t["adm"] += r["admissible_violations"]
                t["cons"] += r["consistency_violations"]
                t["worst"] = max(t["worst"], r["max_h_minus_hstar"])
                t["goal"] = t["goal"] and r["goal_h_zero"]
    lines.append("")
    lines.append("TOTALS over all 30 small instances:")
    for name, t in totals.items():
        lines.append(f"  {name:6s}: admissibility_violations={t['adm']}  "
                     f"consistency_violations={t['cons']}  worst max(h-h*)={t['worst']:+.0f}  "
                     f"goal_h0_everywhere={t['goal']}")
    text = "\n".join(lines) + "\n"
    with open(out_path, "w") as f:
        f.write(text)
    print(text)
    return totals


if __name__ == "__main__":
    run()
