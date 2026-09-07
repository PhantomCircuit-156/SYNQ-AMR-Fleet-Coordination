from coordination.corridor import CorridorProtocol
from coordination.deadlock import DeadlockEventType, DeadlockManager
from coordination.intent import IntentManager
from coordination.local_world import LocalWorldModel
from models.corridor import CorridorRequest, CorridorToken
from models.grid import Cell
from models.messages import PeerStateMessage
from models.priority import TaskPriority
from models.robot_state import RobotState
from simulation.corridor_scenarios import (
    basic_corridor_region,
    build_intentional_deadlock_worlds,
    build_opposing_corridor_scenario,
)


def message(
    robot_id: str,
    pose: Cell,
    timestamp: int,
    *,
    priority: TaskPriority = TaskPriority.NORMAL,
    request: CorridorRequest | None = None,
    token: CorridorToken | None = None,
    state: RobotState = RobotState.MOVING,
    waiting_ticks: int = 0,
    blocked_by: tuple[str, ...] = (),
) -> PeerStateMessage:
    return PeerStateMessage(
        robot_id=robot_id,
        timestamp=timestamp,
        pose=pose,
        velocity=(0, 0),
        goal=None,
        intent=IntentManager().generate_hold(robot_id, pose, timestamp),
        priority=priority,
        battery=100.0,
        state=state,
        heartbeat=timestamp,
        waiting_ticks=waiting_ticks,
        corridor_requests=(request,) if request else (),
        corridor_tokens=(token,) if token else (),
        blocked_by=blocked_by,
    )


def test_opposing_corridor_approach_has_one_agreed_owner_and_waiter() -> None:
    simulation = build_opposing_corridor_scenario()
    r1, r2 = simulation.robots

    simulation.step()
    simulation.step()

    assert r1.known_corridor_owner_for("C1") == "R1"
    assert r2.known_corridor_owner_for("C1") == "R1"

    simulation.step()

    assert r1.position in simulation.corridor_regions[0].cells
    assert r2.position in simulation.corridor_regions[0].passing_bays
    assert r2.state is RobotState.WAITING
    assert simulation.collision_count == 0


def test_corridor_waiter_eventually_proceeds_after_owner_leaves() -> None:
    simulation = build_opposing_corridor_scenario()
    simulation.run_until_complete(max_ticks=150)
    r1, r2 = simulation.robots

    assert r1.state is RobotState.TASK_COMPLETE
    assert r2.state is RobotState.TASK_COMPLETE
    assert r2.conflict_metrics.corridor_wait_ticks > 0
    assert r2.conflict_metrics.resume_events > 0
    assert simulation.collision_count == 0
    assert all(
        len(snapshot.values()) == len(set(snapshot.values()))
        for snapshot in simulation.occupancy_history
    )


def test_corridor_winner_is_same_from_both_evaluation_orders() -> None:
    corridor = basic_corridor_region()
    r1_request = CorridorRequest("C1", "R1", 1, Cell(3, 2), Cell(7, 2))
    r2_request = CorridorRequest("C1", "R2", 1, Cell(7, 2), Cell(3, 2))
    r1 = message("R1", Cell(2, 2), 1, request=r1_request)
    r2 = message("R2", Cell(8, 2), 1, request=r2_request)
    r1_world = LocalWorldModel("R1")
    r1_world.update_own_state(r1)
    r1_world.receive_peer_state(r2)
    r2_world = LocalWorldModel("R2")
    r2_world.update_own_state(r2)
    r2_world.receive_peer_state(r1)

    first = CorridorProtocol("R1").evaluate(corridor, r1_world, 1)
    second = CorridorProtocol("R2").evaluate(corridor, r2_world, 1)

    assert first.owner_id == second.owner_id == "R1"


def test_expired_token_no_longer_preserves_old_owner() -> None:
    corridor = basic_corridor_region()
    r1_request = CorridorRequest("C1", "R1", 10, Cell(3, 2), Cell(7, 2))
    r2_request = CorridorRequest("C1", "R2", 10, Cell(7, 2), Cell(3, 2))
    expired = CorridorToken("C1", "R1", issued_at=1, expires_at=5)
    r1 = message("R1", Cell(2, 2), 10, request=r1_request, token=expired)
    r2 = message(
        "R2", Cell(8, 2), 10, priority=TaskPriority.HIGH, request=r2_request
    )
    world = LocalWorldModel("R2")
    world.update_own_state(r2)
    world.receive_peer_state(r1)

    ownership = CorridorProtocol("R2").evaluate(corridor, world, 10)

    assert ownership.owner_id == "R2"


def test_offline_owner_token_expires_from_local_consideration() -> None:
    corridor = basic_corridor_region()
    stale_request = CorridorRequest("C1", "R1", 0, Cell(3, 2), Cell(7, 2))
    live_request = CorridorRequest("C1", "R2", 10, Cell(7, 2), Cell(3, 2))
    stale_token = CorridorToken("C1", "R1", issued_at=0, expires_at=100)
    stale_r1 = message(
        "R1", Cell(3, 2), 0, request=stale_request, token=stale_token
    )
    live_r2 = message("R2", Cell(8, 2), 10, request=live_request)
    world = LocalWorldModel("R2")
    world.update_own_state(live_r2)
    world.receive_peer_state(stale_r1)

    ownership = CorridorProtocol("R2").evaluate(corridor, world, 10)

    assert ownership.owner_id == "R2"


def test_intentional_deadlock_detects_and_selects_same_backoff() -> None:
    r1_world, r2_world, corridor = build_intentional_deadlock_worlds()
    r1_manager = DeadlockManager("R1")
    r2_manager = DeadlockManager("R2")

    r1_resolution = r1_manager.evaluate(r1_world, (corridor,), 6)
    r2_resolution = r2_manager.evaluate(r2_world, (corridor,), 6)

    assert r1_resolution == r2_resolution
    assert r1_resolution is not None
    assert r1_resolution.proceed_robot_id == "R1"
    assert r1_resolution.backoff_robot_id == "R2"
    assert [event.event_type for event in r1_manager.events] == [
        DeadlockEventType.DEADLOCK_DETECTED,
        DeadlockEventType.DEADLOCK_RESOLUTION,
    ]


def test_deadlock_clear_emits_required_event() -> None:
    r1_world, _, corridor = build_intentional_deadlock_worlds()
    manager = DeadlockManager("R1")
    assert manager.evaluate(r1_world, (corridor,), 6) is not None
    moving_r2 = message("R2", Cell(8, 1), 7, state=RobotState.MOVING)
    r1_world.receive_peer_state(moving_r2)

    assert manager.evaluate(r1_world, (corridor,), 7) is None
    assert manager.events[-1].event_type is DeadlockEventType.DEADLOCK_CLEARED
