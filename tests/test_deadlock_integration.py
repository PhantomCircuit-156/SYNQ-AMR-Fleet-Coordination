from coordination.deadlock import DeadlockEventType
from models.robot_state import RobotState
from simulation.corridor_scenarios import build_recoverable_deadlock_scenario


def test_restored_intentional_deadlock_clears_and_both_tasks_finish() -> None:
    simulation = build_recoverable_deadlock_scenario()

    simulation.run_until_complete(max_ticks=150)

    assert all(robot.state is RobotState.TASK_COMPLETE for robot in simulation.robots)
    assert simulation.collision_count == 0
    assert simulation.tick_count < 150
    for robot in simulation.robots:
        event_types = [event.event_type for event in robot.deadlock_events]
        assert DeadlockEventType.DEADLOCK_DETECTED in event_types
        assert DeadlockEventType.DEADLOCK_RESOLUTION in event_types
        assert DeadlockEventType.DEADLOCK_CLEARED in event_types
