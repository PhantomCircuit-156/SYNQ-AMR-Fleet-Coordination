from models.robot_state import RobotState
from simulation.scenarios import build_basic_three_robot_scenario


def test_three_agents_are_independent_objects_with_independent_planners_and_state() -> None:
    _, simulation = build_basic_three_robot_scenario()
    robots = simulation.robots

    assert [robot.robot_id for robot in robots] == ["R1", "R2", "R3"]
    assert len({id(robot) for robot in robots}) == 3
    assert len({id(robot.planner) for robot in robots}) == 3
    assert all(robot.state is RobotState.PLANNING for robot in robots)


def test_three_non_conflicting_robots_complete_without_crossing_obstacles() -> None:
    scenario, simulation = build_basic_three_robot_scenario()

    simulation.run_until_complete()

    assert all(robot.state is RobotState.TASK_COMPLETE for robot in simulation.robots)
    assert simulation.collision_count == 0
    for robot in simulation.robots:
        history = simulation.position_history[robot.robot_id]
        assert history[-1] == robot.goal
        assert all(scenario.grid.is_walkable(cell) for cell in history)
        assert all(
            abs(source.x - destination.x) + abs(source.y - destination.y) == 1
            for source, destination in zip(history, history[1:])
        )