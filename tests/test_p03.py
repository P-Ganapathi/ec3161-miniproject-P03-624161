"""Unit tests for P03. Run from the repository root:

    python -m unittest discover -s tests -v
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from env import DeliveryProblem, manhattan, bfs_dist, make_problem          # noqa: E402
from agents import (h0, h_far, h_mst, h_mst_plain, astar,                  # noqa: E402
                    optimal_by_permutation)
from checker import reachable_states, exact_cost_to_go, check              # noqa: E402


def corridor(packages):
    """A 1 x 5 corridor with the dock at (0,0), no walls."""
    world = dict(rows=1, cols=5, walls=set(), dock=(0, 0), packages=frozenset(packages))
    return DeliveryProblem(world)


class TestBasics(unittest.TestCase):
    def test_manhattan(self):
        self.assertEqual(manhattan((0, 0), (3, 4)), 7)
        self.assertEqual(manhattan((2, 2), (2, 2)), 0)

    def test_bfs_respects_walls(self):
        # 3 x 3 grid, wall at (0,1). Route (0,0)->(1,0)->(1,1)->(1,2)->(0,2) is 4 steps, not the direct 2
        world = dict(rows=3, cols=3, walls={(0, 1)}, dock=(0, 0), packages=frozenset())
        self.assertEqual(bfs_dist(world, (0, 0))[(0, 2)], 4)


class TestHandCheckedCorridor(unittest.TestCase):
    """Corridor: dock (0,0), packages at (0,2) and (0,4). Optimal tour: 0->4->0 = 8 steps."""

    def setUp(self):
        self.p = corridor([(0, 2), (0, 4)])
        self.start = self.p.initial()

    def test_optimal_cost_is_8(self):
        self.assertEqual(optimal_by_permutation(self.p), 8)

    def test_h_far_hand_value(self):
        # farthest package (0,4): dist 4 out + 4 back = 8
        self.assertEqual(h_far(self.p, self.start), 8)

    def test_h_mst_hand_value(self):
        # nodes: dock, (0,2), (0,4). MST = 2 + 2 = 4. The robot starts at the dock, so attach = 0.
        self.assertEqual(h_mst(self.p, self.start), 4)

    def test_heuristics_do_not_overestimate_at_start(self):
        for h in (h0, h_far, h_mst):
            self.assertLessEqual(h(self.p, self.start), 8)

    def test_astar_finds_8_with_every_heuristic(self):
        for h in (h0, h_far, h_mst):
            self.assertEqual(astar(self.p, h)["cost"], 8)


class TestMSTCounterexample(unittest.TestCase):
    """The plain MST heuristic drops by 2 after one move, so it is inconsistent.
    Values hand-checked in the README (dock (0,0), packages (0,1) and (4,3))."""

    def setUp(self):
        self.p = DeliveryProblem(dict(rows=6, cols=6, walls=set(), dock=(0, 0),
                                      packages=frozenset({(0, 1), (4, 3)})))
        self.s = ((0, 4), frozenset({(4, 3)}))
        self.s2 = ((0, 3), frozenset({(4, 3)}))

    def test_plain_mst_values(self):
        self.assertEqual(h_mst_plain(self.p, self.s), 9)
        self.assertEqual(h_mst_plain(self.p, self.s2), 7)

    def test_plain_mst_violates_consistency(self):
        self.assertGreater(h_mst_plain(self.p, self.s), 1 + h_mst_plain(self.p, self.s2))

    def test_fixed_mst_is_consistent_here(self):
        self.assertLessEqual(h_mst(self.p, self.s), 1 + h_mst(self.p, self.s2))


class TestAStarAgainstOracle(unittest.TestCase):
    """A* with each admissible heuristic must return exactly the brute-force optimum."""

    def test_random_small_instances(self):
        for seed in range(8):
            p = make_problem(rows=6, cols=6, k=3, seed=seed)
            opt = optimal_by_permutation(p)
            for h in (h0, h_far, h_mst):
                self.assertEqual(astar(p, h)["cost"], opt, msg=f"seed={seed} h={h.__name__}")


class TestWeightedSearchBound(unittest.TestCase):
    """Weighted A* with weight w and an admissible h returns cost <= w * optimum."""

    def test_weight_two_bound(self):
        for seed in range(8):
            p = make_problem(rows=8, cols=8, k=5, seed=seed)
            opt = astar(p, h0)["cost"]
            cost = astar(p, h_mst, weight=2.0)["cost"]
            self.assertLessEqual(cost, 2 * opt)
            self.assertGreaterEqual(cost, opt)


class TestAdmissibilityAndConsistency(unittest.TestCase):
    """Exact checker on small worlds: h_far and fixed h_mst must be admissible and consistent."""

    def test_checker_zero_violations(self):
        for k in (2, 3):
            for seed in range(5):
                p = make_problem(rows=6, cols=6, k=k, seed=seed)
                states = reachable_states(p)
                hstar = exact_cost_to_go(p, states)
                for h in (h0, h_far, h_mst):
                    r = check(p, h, states, hstar)
                    self.assertEqual(r["admissible_violations"], 0, msg=f"{h.__name__} k={k} seed={seed}")
                    self.assertEqual(r["consistency_violations"], 0, msg=f"{h.__name__} k={k} seed={seed}")
                    self.assertTrue(r["goal_h_zero"])


if __name__ == "__main__":
    unittest.main()
