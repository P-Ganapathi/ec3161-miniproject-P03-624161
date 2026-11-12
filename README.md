# P03: Multi-Package Delivery Robot (EC-3161, roll 624161)

**Headline result.** A* with a minimum-spanning-tree heuristic finds the same optimal delivery route as plain uniform-cost search, while expanding about **15 times fewer states** on 10-package instances (6,259 vs 91,978 on average over 10 seeded worlds). A faster but inadmissible variant expands fewer than 200 states at every size, at a small cost penalty (about 4% longer routes on average).

---

## 1. Problem

A robot stands on a 12 × 12 warehouse grid with about 20% walls. It starts at the **dock**. There are *k* packages in fixed cells. The robot must pick up every package and return to the dock, using the fewest steps.

**Formulation (this is what the code implements):**

- **State:** `(pos, remaining)`. `pos` is the robot's cell. `remaining` is the set of packages not yet collected. The starter's `DeliveryProblem` class defines this.
- **Initial state:** `(dock, all packages)`.
- **Actions:** move Up, Down, Left, or Right to a free cell.
- **Transition:** moving into a package's cell collects it, so that package is removed from `remaining`.
- **Goal:** `remaining` is empty and the robot is at the dock.
- **Cost:** every step costs 1. The solution cost is the number of steps.

This is a travelling-salesman-like problem with obstacles. Each *state* is a place plus a "which packages are left" set, so the number of states grows like (number of cells) × 2^k. That is why an uninformed search becomes slow as *k* grows, and why a good heuristic matters.

**Why it matters.** Route planning for warehouse robots and delivery vehicles is a real problem. Exact optimal routes are useful for benchmarking cheaper methods, and the heuristic question is central to A*: a heuristic that never overestimates gives optimal answers, and a tighter one prunes more.

## 2. Methods

| Name in code | What it is | Exact? |
|---|---|---|
| `baseline_ucs` | A* with h = 0, i.e. uniform-cost search (the **baseline**) | yes |
| `h_far` | "Farthest package out and back" (**main method 1**) | yes |
| `h_mst` | "Connect the robot to the nearest target, then the MST over the dock and remaining packages" (**main method 2**) | yes |
| `improv_wastar2_mst` | Weighted A* with f = g + 2·h_mst (**the improvement**) | no |

**The heuristics, in plain words.**

- **h_far.** Pick the remaining package that is farthest from the robot in the Manhattan sense (|dx| + |dy|). Add the distance from that package back to the dock. Any delivery route must visit that package and then go home, so the route is at least that long.
- **h_mst.** Look at the dock and all remaining packages as points. Find the cheapest set of connections that links all of them together (a *minimum spanning tree*, computed with Prim's algorithm). A real route is a connected path through all those points, so it cannot be cheaper than that tree. We also add the distance from the robot to the nearest point, because the robot has to get to the group first.
- **h0.** The zero heuristic, which gives no guidance. This is the baseline.

**The first attempt, kept on purpose.** The first version of `h_mst` put the robot itself into the tree as a point, instead of attaching it at the end. It was *admissible* but **not consistent**: the checker found 1,090 consistency violations. A concrete example, on the 6 × 6 instance with the dock at (0,0) and packages at (0,1) and (4,3): with the robot at (0,4), the heuristic is 9. After one legal step to (0,3), it is 7. A single step can change the true remaining cost by at most 1, so a drop of 2 is impossible for a consistent heuristic. The fixed `h_mst` above avoids this and is kept as the main method. The first attempt remains in `src/agents.py` as `h_mst_plain`, and its failure is covered by a unit test.

**The improvement.** Weighted A* with weight 2 uses f = g + 2·h_mst. Multiplying by 2 makes the heuristic push harder toward the goal. This is faster, but it can overestimate, so the result is no longer guaranteed optimal. The measurement question is: how much faster, and how much longer are the routes?

## 3. Results

All numbers are means over **10 seeded worlds per k** (seeds 0–9, the same 40 instances for every method). Full table: `results/summary.md`. Raw per-run counters: `results/metrics.csv`.

**Effort (states expanded, mean over 10 worlds)**

| k | baseline (UCS) | main: A* + h_far | main: A* + h_mst | improvement: wA* ×2 + h_mst |
|---|---|---|---|---|
| 4 | 1,629 | 253 | 258 | 127 |
| 6 | 6,518 | 777 | 579 | 209 |
| 8 | 24,315 | 3,076 | 1,482 | 190 |
| 10 | 91,978 | 15,495 | 6,259 | 191 |

**Quality.** Both exact main methods returned the same optimal cost as the baseline on all 40 instances (0 mismatches). The improvement returned the same cost on 20 of 40 instances, and was longer on the other 20. Its largest penalty was 10 extra steps. Its worst ratio to optimal was 1.25, and its average ratio was 1.043 (about 4% longer). The weighted bound of 2× optimal held everywhere.

**Memory (peak number of distinct states kept, k = 10).** UCS 92,557 · A*+h_far 18,835 · A*+h_mst 7,943 · improvement 290.

**Runtime.** At k = 10, UCS averages about 0.52 s, A*+h_far about 0.12 s, A*+h_mst about 0.15 s, and the improvement about 0.004 s, on a Linux sandbox. Times depend on the machine, so the states-expanded counts are the more reliable comparison. The times show that h_far is quicker per run than h_mst even though h_mst expands fewer states.

**Figure:** `results/figures/expanded_vs_k.png` shows states expanded against k on a log scale.

**Note on the CSV format.** The assignment's suggested layout is `method, seed, metric, value`. I added a `k` column, because the same seed appears at every k and would otherwise be ambiguous. The columns are `method, k, seed, metric, value`.

## 4. Analysis: where each method wins or fails, and why

- **UCS (baseline) is exact but blind.** With no heuristic, it expands every state whose cost is below the optimum. For k = 10 that is tens of thousands of states. Its growth from k = 4 to k = 10 is roughly 56×.
- **h_far is a good first guide.** It is cheap to compute and gives a large saving over UCS (about 6× at k = 10). But it looks at only one package at a time. It is weak when several packages lie in different directions, because it never accounts for the cost of visiting them in sequence.
- **h_mst expands the fewest states among the exact methods from k = 6 upward** (at k = 4 it ties with h_far). It accounts for all remaining packages together, so it sees the whole cluster. Compared with UCS it expands about 6× fewer states at k = 4, 11× fewer at k = 6, 16× fewer at k = 8, and 15× fewer at k = 10. Compared with h_far, it expands fewer states at k = 6, 8, and 10, and about the same number at k = 4. **But it is not faster in wall-clock time.** Each state costs more to evaluate (Prim's algorithm does O(k²) work per state), so h_far is the quicker exact method at every size we tested (k = 10: 0.12 s vs 0.15 s on average). The lesson is that fewer expansions do not automatically mean less time: the cost of the heuristic itself matters.
- **The inadmissible improvement is much faster.** Its expansions stay roughly flat at about 200 from k = 6 to k = 10. The price is that the solution may be suboptimal. On this grid the price is small (about 4% on average), but the worst case is 25% above optimal. For a warehouse, a 4% longer route in exchange for a large saving in planning time may well be a good trade, but that depends on the application, and the report does not claim it is always worth it.
- **Where the methods fail.** None of the methods failed on these sizes. The exact methods are limited by the (cells) × 2^k state space, which grows quickly. We did not test larger k, so we do not know where they break down.

## 5. Correctness evidence

1. **Hand-checked instance.** A 1 × 5 corridor with the dock at one end and packages at distance 2 and 4. By hand, the optimal tour is 8 steps. The tests check the optimum (8), the heuristic values (h_far = 8, h_mst = 4), and that every heuristic returns 8 with A*.
2. **Independent oracle.** `optimal_by_permutation` in `src/agents.py` tries every visiting order of the packages, using true shortest-path distances. It shares no search code with A*. A* agrees with it on every tested instance (k = 3 in the unit tests; k = 4 and 6 are checked in `experiment.py`).
3. **Exact admissibility and consistency checker.** `src/checker.py` computes the true cost-to-go h*(s) for every reachable state of 30 small worlds (6 × 6, k = 2, 3, 4), using value iteration. Then it checks that h(s) ≤ h*(s) and h(s) ≤ 1 + h(s') for every move. Results (`results/checker_report.txt`):

| heuristic | admissibility violations | consistency violations |
|---|---|---|
| h0 | 0 | 0 |
| h_far | 0 | 0 |
| h_mst (fixed) | 0 | 0 |
| h_mst_plain (first attempt) | 0 | 1,090 |

4. **Larger-instance consistency.** `src/check_consistency_large.py` checks h(s) ≤ 1 + h(s′) over every reachable state of 21 larger instances (up to 12 × 12, k = 8): 156,707 states. `h_far` and the fixed `h_mst` have 0 violations. The first MST version has 33,949, with a worst drop of 2.
5. **Unit tests.** 13 tests in `tests/test_p03.py`. Run them with `python -m unittest discover -s tests -v`. All pass.

**Paper proofs.**

*h_far is admissible.* Any delivery route visits the package p and then ends at the dock. Its length is at least dist(pos, p) + dist(p, dock), which is at least the Manhattan sum. So the maximum over p is a lower bound.

*h_far is consistent.* Take a move from s to s', where the robot goes to a neighbouring cell a. Its distance to any cell changes by at most 1, so there are three cases.
- If the maximum for s is reached at a package p that is still remaining in s', then f_p(pos) ≤ 1 + f_p(a) ≤ 1 + h(s').
- If the maximum for s is reached at the package a that was just collected, then h(s) = 1 + dist(a, dock). If s' has no packages left, h(s') = dist(a, dock), so the inequality holds. Otherwise, for any remaining p, dist(a, dock) ≤ dist(a, p) + dist(p, dock) = f_p(a), so h(s') ≥ dist(a, dock), and again h(s) ≤ 1 + h(s').
- If no packages remain, h(s) = dist(pos, dock) ≤ 1 + dist(a, dock).

*h_mst (fixed) is admissible.* A route's first target after the start is some x in {dock} ∪ remaining. The first leg costs at least dist(pos, x). The rest of the route is a connected walk through all targets, so it costs at least the MST over those targets. This gives the bound used in the code.

*h_mst (fixed) is consistent:* **tested, not proven.** The exact checker finds no violations on the 30 small instances. On 21 larger instances (up to 12 × 12 with 8 packages), checking every reachable state, there are 0 violations out of 156,707 states. The natural proof route fails: removing a package can make the minimum spanning tree *heavier*. For example, with Manhattan distances, three packages at (1,0), (−1,0), (0,1) have a tree of weight 4 without the centre cell, but weight 3 with the centre cell (0,0) included. So any proof must use the attachment term. This is still open (Section 7).

## 6. Reproduce

**Requirements:** Python 3.10 or newer. The core code uses only the standard library. `matplotlib` is needed only for the figure (see `requirements.txt`).

```bash
# 1. Get the code (see Section 9 for the git steps)
git clone <your-repo-url>
cd ec3161-miniproject-P03-624161

# 2. (Optional) install matplotlib for the figure
pip install -r requirements.txt

# 3. Run the tests
python -m unittest discover -s tests -v

# 4. Run the checkers (write results/checker_report.txt and results/consistency_large.txt)
python src/checker.py
python src/check_consistency_large.py

# 5. Run all experiments (writes results/metrics.csv, results/summary.md, results/figures/)
python src/experiment.py
```

Every instance is seeded, so the results repeat exactly. The only numbers that vary between machines are the wall-clock times.

**Tested on:** Linux, Python 3.12. The full experiment finished in under a minute.

## 7. Limitations and next steps

- **Consistency of the fixed h_mst is not proven.** It is tested on 21 larger instances and 30 small ones with zero violations, but not proven. Removing a package can make the MST heavier, so the proof must use the attachment term. This is the main open item.
- **Exact admissibility is only checked on small grids.** The exact checker covers 6 × 6 worlds with k ≤ 4. On larger worlds, consistency is checked over all reachable states, and admissibility is checked through agreement with the exact optimal costs of the baseline search, not through a full cost-to-go computation.
- **Only one grid size and one wall density** (12 × 12, 20% walls). Results could look different on denser or sparser warehouses.
- **Only one inadmissible variant** was measured, with weight 2. A sweep over weights (1.2, 1.5, 2, 3) would show the trade-off curve more clearly.
- **No heatmap of expanded states** (the optional stretch goal) was produced.
- **Possible next steps:** a tighter admissible heuristic from precomputed BFS distances (the "true maze distance" idea in the brief); a proof of MST consistency; testing larger k with a memory cap.

## 8. AI-use statement

I used an AI assistant (Claude, by Anthropic) during this project. I used it to help me understand the problem, plan the structure of the repository, write and debug the code, design the unit tests and experiment script, and draft parts of the README and report. When the assistant suggested the nearest-attachment fix for the MST heuristic, I checked it by running the exact checker, and I kept the first version in the code to show why it fails.

All results in this repository come from running the code here. I ran the unit tests and both consistency checkers myself to confirm the claims. I also reviewed the reasoning behind each heuristic and each proof, and I tested the examples by hand, such as the corridor example and the counterexample for the first MST heuristic.

## 9. Git: what this is and how to use it (for beginners)

**What is git?** Git is a tool that keeps a history of your files. Each saved version is called a *commit*, and you can always go back to an earlier one. **GitHub** is a website that stores a git project online so others can see it. The assignment wants your project on GitHub.

**The words you will see:**

| Word | Plain meaning |
|---|---|
| repository ("repo") | the project folder that git is tracking |
| commit | one saved snapshot, with a short message describing it |
| push | upload your commits to GitHub |
| clone | download a copy of a repo from GitHub |
| tag | a permanent label on one commit, here `v1.0` |

**The steps (run these in a terminal, inside the project folder):**

```bash
# First time only: tell git who you are (use your own name and email)
git config --global user.name "Your Name"
git config --global user.email "you@example.com"

# Start tracking the folder
git init

# Look at what is changed (run this any time to see the status)
git status

# Stage (select) files for the next commit, then commit them
git add .
git commit -m "Describe what you changed in a few words"

# Look at your history
git log --oneline

# Create a repo on GitHub.com (New repository, name it ec3161-miniproject-P03-624161), then:
git branch -M main
git remote add origin https://github.com/<your-username>/ec3161-miniproject-P03-624161.git
git push -u origin main

# Mark the final version
git tag v1.0
git push origin v1.0
```

**Good commit habits.** Commit each time one piece works, with a message that names the piece. The assignment asks for at least 8 commits over the project, so commit as you go.

**Common problems.**
- *"Permission denied" or a login prompt on push:* GitHub no longer accepts your account password. Create a personal access token on GitHub (Settings → Developer settings → Personal access tokens) and paste it when asked for a password.
- *`git status` says nothing is tracked:* you forgot `git add`.
- *I want to undo a commit I just made:* ask your instructor or search "undo last git commit". Do not delete the `.git` folder.

## Folder layout

```
ec3161-miniproject-P03-624161/
├── README.md              this file (front page)
├── requirements.txt       optional extra: matplotlib for the figure
├── .gitignore
├── starter/               the instructor's starter code, UNMODIFIED
│   ├── P03_delivery/delivery.py
│   └── README.md
├── src/
│   ├── env.py             environment: imports the starter, BFS distances, instance builder
│   ├── agents.py          heuristics (h0, h_far, h_mst, h_mst_plain), A*, permutation oracle
│   ├── checker.py         exact admissibility and consistency checker (small grids)
│   ├── check_consistency_large.py  consistency over all reachable states (larger grids)
│   └── experiment.py      runs all methods on the same instances, writes results
├── tests/
│   └── test_p03.py        13 unit tests
├── results/
│   ├── metrics.csv        method, k, seed, metric, value (960 rows)
│   ├── summary.md         mean +- std table
│   ├── checker_report.txt checker output
│   ├── consistency_large.txt  consistency output on larger grids
│   └── figures/expanded_vs_k.png
└── report/
    ├── report.md          4-6 page report (source)
    └── report.pdf         4-6 page report (PDF)
```

## References

- Russell, S. & Norvig, P. *Artificial Intelligence: A Modern Approach*, 3rd/4th ed. Chapter 3 (uninformed search, A*), and Chapter 3's discussion of admissible and consistent heuristics.
- Prim, R. C. (1957). Shortest connection networks and some generalizations. *Bell System Technical Journal*, 36(6), 1389–1401. (Minimum spanning tree.)
- Pohl, I. (1970). Heuristic search viewed as path finding in a graph. *Artificial Intelligence*, 1(3–4), 193–204. (Weighted A*.)
- Starter code and assignment brief: EC-3161 Mini-Project, V. Prakash Singh, NIT Andhra Pradesh, 2026–27.
