import pytest

from models.robot_state import RobotState
from simulation.coordination_scenarios import (
    build_head_on_scenario,
    build_same_cell_scenario,
    build_three_robot_intersection_scenario,
    build_two_robot_intersection_scenario,
)


SCENARIOS = (
    (build_same_cell_scenario, "SAME_CELL"),
    (build_head_on_scenario, "HEAD_ON"),
    (build_two_robot_intersection_scenario, "INTERSECTION"),
    (build_three_robot_intersection_scenario, "SAME_CELL"),
)


@pytest.mark.parametrize(("scenario_builder", "expected_type"), SCENARIOS)
def test_conflicts_are_handled_before_collision_and_all_robots_resume(
    scenario_builder, expected_type: str
) -> None:
    scenario = scenario_builder()
    simulation = scenario.simulation

    simulation.step()  # All robots plan and publish their independent intents.
    simulation.step()  # All robots evaluate received intents before moving.

    assert simulation.collision_count == 0
    assert simulation.robots[0].conflict_metrics.conflicts_won >= 1
    assert any(
        robot.conflict_metrics.conflicts_lost >= 1
        for robot in simulation.robots[1:]
    )
    assert any(
        expected_type in robot.conflict_metrics.conflict_types
        for robot in simulation.robots
    )

    simulation.run_until_complete(max_ticks=250)

    assert simulation.collision_count == 0
    assert all(robot.state is RobotState.TASK_COMPLETE for robot in simulation.robots)
    assert all(robot.position == robot.goal for robot in simulation.robots)
    assert any(robot.conflict_metrics.blocked_ticks > 0 for robot in simulation.robots)
    assert any(robot.conflict_metrics.resume_events > 0 for robot in simulation.robots)
    assert any(
        RobotState.WAITING in tick_states.values()
        for tick_states in simulation.state_history
    )
    for occupancy in simulation.occupancy_history:
        assert len(occupancy.values()) == len(set(occupancy.values()))


def test_head_on_loser_yields_sideways_then_resumes() -> None:
    simulation = build_head_on_scenario().simulation
    r1, r2 = simulation.robots

    simulation.step()
    simulation.step()

    assert r1.position.x == 3
    assert r2.position.y == 0
    assert r2.state is RobotState.WAITING
    assert r2.conflict_metrics.yield_events == 1
    assert simulation.collision_count == 0

    simulation.run_until_complete(max_ticks=100)

    assert r2.state is RobotState.TASK_COMPLETE
    assert r2.conflict_metrics.resume_events >= 1


def test_both_agents_cache_the_same_initial_winner_without_simulator_command() -> None:
    simulation = build_same_cell_scenario().simulation
    r1, r2 = simulation.robots

    simulation.step()
    simulation.step()

    assert r1.known_winner_for("R2") == "R1"
    assert r2.known_winner_for("R1") == "R1"
    assert r1.state is RobotState.MOVING
    assert r2.state is RobotState.WAITING


def test_three_robot_intersection_reaches_collision_free_completion_after_waiting() -> None:
    simulation = build_three_robot_intersection_scenario().simulation
    simulation.run_until_complete(max_ticks=250)

    metrics = simulation.metrics_snapshot()
    assert metrics["collisions"] == 0
    assert all(robot.state is RobotState.TASK_COMPLETE for robot in simulation.robots)
    assert sum(
        robot.conflict_metrics.resume_events for robot in simulation.robots
    ) >= 2
