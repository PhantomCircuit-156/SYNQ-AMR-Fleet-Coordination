from pathlib import Path

from coordination.conflict import ConflictDetector, ConflictType
from coordination.intent import INTENT_HORIZON, Intent, IntentManager, IntentStep
from coordination.local_world import LocalWorldModel
from coordination.resolution import ConflictResolver, ResolutionCandidate, ResolutionReason
from models.grid import Cell, WarehouseGrid
from models.messages import PeerStateMessage
from models.priority import TaskPriority
from models.robot_state import RobotState
from simulation.robot import RobotAgent


def make_intent(robot_id: str, *cells: Cell) -> Intent:
    return Intent(
        robot_id=robot_id,
        generated_at=0,
        steps=tuple(
            IntentStep(timestep=timestep, cell=cell)
            for timestep, cell in enumerate(cells, start=1)
        ),
    )


def make_message(
    robot_id: str,
    pose: Cell,
    *future_cells: Cell,
    priority: TaskPriority = TaskPriority.NORMAL,
    waiting_ticks: int = 0,
) -> PeerStateMessage:
    return PeerStateMessage(
        robot_id=robot_id,
        timestamp=0,
        pose=pose,
        velocity=(0, 0),
        goal=future_cells[-1] if future_cells else pose,
        intent=make_intent(robot_id, *future_cells),
        priority=priority,
        battery=100.0,
        state=RobotState.MOVING,
        heartbeat=0,
        waiting_ticks=waiting_ticks,
    )


def detect(
    first: PeerStateMessage,
    second: PeerStateMessage,
    static_map: WarehouseGrid | None = None,
):
    world = LocalWorldModel(robot_id=first.robot_id, static_map=static_map)
    world.update_own_state(first)
    world.receive_peer_state(second)
    return ConflictDetector(first.robot_id).detect(world)


def test_intent_is_limited_to_configured_horizon() -> None:
    path = tuple(Cell(x, 5) for x in range(5, 13))

    intent = IntentManager().generate("R1", path, current_path_index=0, timestamp=7)

    assert len(intent.steps) == INTENT_HORIZON
    assert intent.cells == tuple(Cell(x, 5) for x in range(6, 11))
    assert tuple(step.timestep for step in intent.steps) == (1, 2, 3, 4, 5)


def test_same_cell_conflict_at_same_timestep() -> None:
    r1 = make_message("R1", Cell(0, 0), Cell(1, 0), Cell(2, 0))
    r2 = make_message("R2", Cell(2, 2), Cell(2, 1), Cell(2, 0))

    conflicts = detect(r1, r2)

    same_cell = [conflict for conflict in conflicts if conflict.conflict_type is ConflictType.SAME_CELL]
    assert len(same_cell) == 1
    assert same_cell[0].cell == Cell(2, 0)
    assert same_cell[0].time_by_robot == (("R1", 2), ("R2", 2))


def test_head_on_swap_conflict() -> None:
    r1 = make_message("R1", Cell(0, 0), Cell(1, 0))
    r2 = make_message("R2", Cell(1, 0), Cell(0, 0))

    conflicts = detect(r1, r2)

    head_on = [conflict for conflict in conflicts if conflict.conflict_type is ConflictType.HEAD_ON]
    assert len(head_on) == 1
    assert {transition.robot_id for transition in head_on[0].transitions} == {"R1", "R2"}


def test_intersection_conflict_for_same_cell_at_different_times() -> None:
    r1 = make_message("R1", Cell(0, 0), Cell(1, 0), Cell(2, 0))
    r2 = make_message("R2", Cell(2, 1), Cell(2, 0), Cell(3, 0))

    conflicts = detect(r1, r2)

    intersections = [
        conflict for conflict in conflicts if conflict.conflict_type is ConflictType.INTERSECTION
    ]
    assert len(intersections) == 1
    assert intersections[0].cell == Cell(2, 0)
    assert intersections[0].time_by_robot == (("R1", 2), ("R2", 1))


def test_separated_paths_have_no_false_conflict() -> None:
    r1 = make_message("R1", Cell(0, 0), Cell(1, 0), Cell(2, 0))
    r2 = make_message("R2", Cell(0, 3), Cell(1, 3), Cell(2, 3))

    assert detect(r1, r2) == ()


def test_opposing_intents_in_a_narrow_corridor_are_detected() -> None:
    obstacles = frozenset({Cell(x, y) for x in range(5) for y in (0, 2)})
    corridor = WarehouseGrid(width=5, height=3, obstacles=obstacles)
    r1 = make_message("R1", Cell(0, 1), Cell(1, 1), Cell(2, 1))
    r2 = make_message("R2", Cell(4, 1), Cell(3, 1), Cell(2, 1))

    conflicts = detect(r1, r2, corridor)

    assert any(
        conflict.conflict_type is ConflictType.CORRIDOR for conflict in conflicts
    )


def test_resolver_is_deterministic_and_argument_order_invariant() -> None:
    resolver_r1 = ConflictResolver("R1")
    resolver_r2 = ConflictResolver("R2")
    r1 = ResolutionCandidate("R1", TaskPriority.NORMAL, waiting_ticks=3)
    r2 = ResolutionCandidate("R2", TaskPriority.NORMAL, waiting_ticks=8)

    first = resolver_r1.resolve(r1, r2)
    repeated = resolver_r1.resolve(r1, r2)
    reversed_order = resolver_r2.resolve(r2, r1)

    assert first == repeated == reversed_order
    assert first.winner_id == "R2"
    assert first.reason is ResolutionReason.WAITING_FAIRNESS


def test_high_priority_beats_normal_when_both_are_safe() -> None:
    resolver = ConflictResolver("R1")
    high = ResolutionCandidate("R2", TaskPriority.HIGH)
    normal = ResolutionCandidate("R1", TaskPriority.NORMAL, waiting_ticks=100)

    resolution = resolver.resolve(normal, high)

    assert resolution.winner_id == "R2"
    assert resolution.reason is ResolutionReason.TASK_PRIORITY


def test_safety_eligibility_outranks_high_task_priority() -> None:
    resolver = ConflictResolver("R1")
    unsafe_high = ResolutionCandidate(
        "R1", TaskPriority.HIGH, waiting_ticks=100, safety_clear=False
    )
    safe_normal = ResolutionCandidate("R2", TaskPriority.NORMAL, safety_clear=True)

    resolution = resolver.resolve(unsafe_high, safe_normal)

    assert resolution.winner_id == "R2"
    assert resolution.reason is ResolutionReason.SAFETY


def test_same_priority_and_waiting_resolves_by_stable_robot_id() -> None:
    resolver = ConflictResolver("R2")
    r2 = ResolutionCandidate("R2", TaskPriority.NORMAL, waiting_ticks=5)
    r1 = ResolutionCandidate("R1", TaskPriority.NORMAL, waiting_ticks=5)

    resolution = resolver.resolve(r2, r1)

    assert resolution.winner_id == "R1"
    assert resolution.yielding_robot_ids == ("R2",)
    assert resolution.reason is ResolutionReason.ROBOT_ID


def test_two_robot_perspectives_detect_and_resolve_identically() -> None:
    r1_message = make_message(
        "R1", Cell(0, 0), Cell(1, 0), priority=TaskPriority.NORMAL
    )
    r2_message = make_message(
        "R2", Cell(1, 0), Cell(0, 0), priority=TaskPriority.HIGH
    )
    r1 = RobotAgent("R1", Cell(0, 0))
    r2 = RobotAgent("R2", Cell(1, 0))
    r1.local_world_model.update_own_state(r1_message)
    r1.local_world_model.receive_peer_state(r2_message)
    r2.local_world_model.update_own_state(r2_message)
    r2.local_world_model.receive_peer_state(r1_message)

    r1_conflicts = r1.detect_conflicts()
    r2_conflicts = r2.detect_conflicts()
    r1_resolution = r1.resolve_conflict(r1_conflicts[0])
    r2_resolution = r2.resolve_conflict(r2_conflicts[0])

    assert r1_conflicts == r2_conflicts
    assert r1_resolution == r2_resolution
    assert r1_resolution.winner_id == "R2"


def test_coordination_modules_do_not_import_or_read_simulator_fleet_state() -> None:
    repository_root = Path(__file__).parents[1]
    coordination_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted((repository_root / "coordination").glob("*.py"))
    )
    fleet_source = (repository_root / "simulation" / "fleet.py").read_text(encoding="utf-8")

    assert "from simulation" not in coordination_source
    assert "import simulation" not in coordination_source
    assert "global_robots" not in coordination_source
    assert ".detect_conflicts(" not in fleet_source
    assert ".resolve_conflict(" not in fleet_source
