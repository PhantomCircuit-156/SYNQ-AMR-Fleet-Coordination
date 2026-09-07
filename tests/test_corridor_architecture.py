from pathlib import Path

from coordination.deadlock import DeadlockConfig, DeadlockEventType, DeadlockManager
from models.robot_state import RobotState
from simulation.corridor_scenarios import (
    build_intentional_deadlock_worlds,
    build_opposing_corridor_scenario,
)
from simulation.robot import RobotAgent


def test_exactly_one_corridor_token_is_published_after_distributed_evaluation() -> None:
    simulation = build_opposing_corridor_scenario()

    simulation.step()
    simulation.step()

    tokens = [token for robot in simulation.robots for token in robot.corridor_tokens]
    assert len(tokens) == 1
    assert tokens[0].owner_id == "R1"


def test_owner_release_removes_token_and_waiter_finishes_without_infinite_wait() -> None:
    simulation = build_opposing_corridor_scenario()
    simulation.run_until_complete(max_ticks=150)
    r1, r2 = simulation.robots

    assert r1.corridor_tokens == ()
    assert r2.corridor_tokens == ()
    assert r2.state is RobotState.TASK_COMPLETE
    assert simulation.tick_count < 150
    assert simulation.collision_count == 0


def test_deadlock_threshold_is_configurable_in_seconds() -> None:
    r1_world, _, corridor = build_intentional_deadlock_worlds(waiting_ticks=6)
    manager = DeadlockManager(
        "R1", DeadlockConfig(threshold_seconds=8.0, tick_seconds=1.0)
    )

    assert manager.evaluate(r1_world, (corridor,), timestamp=6) is None
    assert manager.events == []


def test_robot_agents_apply_deadlock_backoff_from_their_own_world_models() -> None:
    r1_world, r2_world, corridor = build_intentional_deadlock_worlds()
    grid = r1_world.static_map
    assert grid is not None
    r1 = RobotAgent("R1", r1_world.own_state.pose)
    r2 = RobotAgent("R2", r2_world.own_state.pose)
    for robot, world, goal in (
        (r1, r1_world, r1_world.own_state.goal),
        (r2, r2_world, r2_world.own_state.goal),
    ):
        assert goal is not None
        robot.attach_static_map(grid)
        robot.attach_corridor_regions((corridor,))
        robot.assign_goal(goal)
        robot.decide_next_move()
        robot.state = RobotState.WAITING
        robot.waiting_ticks = 6
        robot.local_world_model = world

    r1_move = r1.decide_next_move()
    r2_move = r2.decide_next_move()

    assert r1_move is None
    assert r1.state is RobotState.MOVING
    assert r2_move in corridor.passing_bays
    assert r2.state is RobotState.YIELDING
    grid.validate_move(r2.position, r2_move)
    r2.confirm_move(r2_move)
    assert r2.state is RobotState.WAITING
    assert [event.event_type for event in r1.deadlock_events] == [
        DeadlockEventType.DEADLOCK_DETECTED,
        DeadlockEventType.DEADLOCK_RESOLUTION,
    ]
    assert [event.event_type for event in r2.deadlock_events] == [
        DeadlockEventType.DEADLOCK_DETECTED,
        DeadlockEventType.DEADLOCK_RESOLUTION,
    ]


def test_simulator_and_bus_do_not_own_corridor_or_deadlock_policy() -> None:
    root = Path(__file__).parents[1]
    simulator_source = (root / "simulation" / "fleet.py").read_text(encoding="utf-8")
    bus_source = (root / "networking" / "peer_bus.py").read_text(encoding="utf-8")

    assert "CorridorProtocol" not in simulator_source
    assert "DeadlockManager" not in simulator_source
    assert "known_corridor_owner_for" not in simulator_source
    assert "CorridorProtocol" not in bus_source
    assert "DeadlockManager" not in bus_source
