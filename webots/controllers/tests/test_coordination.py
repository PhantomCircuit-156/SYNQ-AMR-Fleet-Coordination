"""Focused, dependency-free checks for the benchmark decision and arithmetic."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "synq_controller"))
from benchmark_config import CONFLICT_CELLS
from benchmark_coordination import PeerIntent, should_wait
from benchmark_metrics import improvement_percent, result_summary

def intent(robot_id, cell, waypoints, priority, waiting=0):
    return PeerIntent(robot_id, 1.0, cell, (0.0, 0.0), (6, 6),
                      waypoints, "MOVING", priority, waiting)

class CoordinationChecks(unittest.TestCase):
    def test_conflict_detection_and_priority(self):
        r3 = intent("R3", (3, 1), ((3, 2), (3, 3)), 1)
        r1 = intent("R1", (1, 2), ((2, 2), (3, 2)), 3)
        self.assertEqual(should_wait("synq", r3, (r1,), CONFLICT_CELLS, 1.0),
                         (True, ("R1",)))
        self.assertEqual(should_wait("synq", r1, (r3,), CONFLICT_CELLS, 1.0),
                         (False, ()))

    def test_separate_lanes_vs_baseline_region(self):
        r1 = intent("R1", (1, 2), ((2, 2), (3, 2)), 3)
        r2 = intent("R2", (1, 4), ((2, 4), (3, 4)), 2)
        self.assertFalse(should_wait("synq", r2, (r1,), CONFLICT_CELLS, 1.0)[0])
        current_only = intent("R1", (1, 2), (), 3)
        self.assertTrue(should_wait("baseline", r2, (current_only,), CONFLICT_CELLS, 1.0)[0])

    def test_baseline_releases_robot_after_exit(self):
        r2 = intent("R2", (1, 4), ((2, 4), (3, 4)), 2)
        exiting = intent("R1", (5, 2), (), 3)
        self.assertFalse(should_wait("baseline", r2, (exiting,), CONFLICT_CELLS, 1.0)[0])

    def test_waiting_fairness(self):
        r3 = intent("R3", (3, 1), ((3, 2), (3, 3)), 1, waiting=105)
        r1 = intent("R1", (1, 2), ((2, 2), (3, 2)), 3)
        self.assertFalse(should_wait("synq", r3, (r1,), CONFLICT_CELLS, 1.0)[0])

    def test_stale_peer_ignored(self):
        r3 = intent("R3", (3, 1), ((3, 2), (3, 3)), 1)
        r1 = intent("R1", (1, 2), ((2, 2), (3, 2)), 3)
        self.assertFalse(should_wait("synq", r3, (r1,), CONFLICT_CELLS, 2.0)[0])

    def test_metric_calculation(self):
        result = result_summary("synq", {"R1": 21.0, "R2": 25.0, "R3": 31.0},
                                {"R1": 1.0, "R2": 2.0, "R3": 3.0},
                                {"R1": 0, "R2": 1, "R3": 2},
                                {"R1": 0, "R2": 1, "R3": 1}, 0, 1.0)
        self.assertEqual(result["makespan"], 30.0)
        self.assertEqual(result["total_waiting"], 6.0)
        self.assertEqual(result["conflicts"], 3)
        self.assertEqual(improvement_percent(40.0, 30.0), 25.0)

if __name__ == "__main__":
    unittest.main()
