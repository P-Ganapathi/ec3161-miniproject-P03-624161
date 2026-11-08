"""Experiments for P03.

Same 40 instances (k = 4, 6, 8, 10 packages x seeds 0-9, on a 12x12 grid with 20% walls)
are used for every method, so the comparison is fair.

Methods
  baseline_ucs         : A* with h = 0 (uniform-cost search). Exact, the baseline.
  main_astar_far       : A* with h_far (main method 1). Exact.
  main_astar_mst       : A* with h_mst (main method 2). Exact.
  improv_wastar2_mst   : weighted A*, f = g + 2*h_mst. The IMPROVEMENT: an inadmissible,
                         faster variant. Its cost can be above optimal, so we measure the penalty.

Output
  results/metrics.csv              one row per (method, k, seed, metric, value)
  results/summary.md               mean +- std per (method, k)
  results/figures/expanded_vs_k.png   nodes expanded vs k (log scale)

Run:  python src/experiment.py      (takes a few minutes on a laptop)
"""
import csv
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env import make_problem                                  # noqa: E402
from agents import astar, h0, h_far, h_mst, optimal_by_permutation  # noqa: E402  (permutation used only for k<=6 check)

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
RESULTS = os.path.join(ROOT, "results")
FIGS = os.path.join(RESULTS, "figures")

KS = [4, 6, 8, 10]
SEEDS = range(10)
ROWS, COLS, WALL = 12, 12, 0.2

METHODS = {
    "baseline_ucs": (h0, 1.0),
    "main_astar_far": (h_far, 1.0),
    "main_astar_mst": (h_mst, 1.0),
    "improv_wastar2_mst": (h_mst, 2.0),
}
EXACT = {"baseline_ucs", "main_astar_far", "main_astar_mst"}


def run_all():
    rows = []   # dicts: method, k, seed, metric, value
    for k in KS:
        for seed in SEEDS:
            problem = make_problem(rows=ROWS, cols=COLS, k=k, seed=seed, wall_density=WALL)
            optimal = None
            for name, (h, w) in METHODS.items():
                r = astar(problem, h, weight=w)
                if name == "baseline_ucs":
                    optimal = r["cost"]          # UCS is exact, so its cost is the reference optimum
                    if k <= 6:
                        # Sanity check against the independent permutation oracle.
                        oracle = optimal_by_permutation(problem)
                        assert oracle == optimal, (k, seed, oracle, optimal)
                for metric in ("cost", "expanded", "generated", "peak_stored", "time_s"):
                    value = r["time"] if metric == "time_s" else r[metric]
                    rows.append(dict(method=name, k=k, seed=seed, metric=metric, value=value))
                gap = r["cost"] - optimal
                rows.append(dict(method=name, k=k, seed=seed, metric="suboptimality_steps", value=gap))
                print(f"k={k:2d} seed={seed} {name:20s} cost={r['cost']:3d} (opt {optimal}) "
                      f"expanded={r['expanded']:7d} time={r['time']:.2f}s", flush=True)
    return rows, optimal


def write_csv(rows):
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, "metrics.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["method", "k", "seed", "metric", "value"])
        w.writeheader()
        for r in rows:
            w.writerow({**r, "value": round(r["value"], 6) if isinstance(r["value"], float) else r["value"]})
    return path


def summarise(rows):
    def values(method, k, metric):
        return [r["value"] for r in rows if r["method"] == method and r["k"] == k and r["metric"] == metric]

    lines = ["# Summary (mean +- std over 10 seeded worlds per k)", "",
             "| k | method | cost (steps) | suboptimality (steps) | expanded | peak stored | time (s) |",
             "|---|---|---|---|---|---|---|"]
    for k in KS:
        for name in METHODS:
            def ms(metric):
                v = values(name, k, metric)
                return f"{statistics.mean(v):.3f} +- {statistics.pstdev(v):.3f}" if metric == "time_s" else f"{statistics.mean(v):.1f} +- {statistics.pstdev(v):.1f}"
            lines.append(f"| {k} | {name} | {ms('cost')} | {ms('suboptimality_steps')} | "
                         f"{ms('expanded')} | {ms('peak_stored')} | {ms('time_s')} |")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(RESULTS, "summary.md"), "w") as f:
        f.write(text)
    return text


def plot(rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available: skipping figure")
        return None
    fig, ax = plt.subplots(figsize=(6, 4))
    for name in METHODS:
        means = []
        for k in KS:
            v = [r["value"] for r in rows if r["method"] == name and r["k"] == k and r["metric"] == "expanded"]
            means.append(statistics.mean(v))
        ax.plot(KS, means, marker="o", label=name)
    ax.set_yscale("log")
    ax.set_xlabel("number of packages k")
    ax.set_ylabel("states expanded (mean over 10 worlds, log scale)")
    ax.set_xticks(KS)
    ax.legend(fontsize=8)
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    path = os.path.join(FIGS, "expanded_vs_k.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


if __name__ == "__main__":
    rows, _ = run_all()
    print("wrote", write_csv(rows))
    print(summarise(rows))
    print("figure:", plot(rows))
