import inspect

from models.grid import Cell
from models.messages import PeerStateMessage
from models.priority import TaskPriority
from models.robot_state import RobotState
from networking.peer_bus import InProcessPeerMessageBus
from simulation import robot as robot_module
from simulation.scenarios import build_basic_three_robot_scenario


def test_r1_publishes_state_and_r2_receives_it_in_its_local_world_model() -> None:
    _, simulation = build_basic_three_robot_scenario()
    r1, r2, _ = simulation.robots

    published = r1.publish_state(simulation.message_bus, timestamp=10)
    received_count = r2.receive_peer_messages(simulation.message_bus)

    assert received_count == 1
    assert published == r2.local_world_model.peer_states["R1"]
    assert r2.local_world_model.peer_states["R1"].pose == Cell(2, 2)
    assert r2.local_world_model.peer_intents["R1"] is None
    assert r2.local_world_model.peer_priorities["R1"] is TaskPriority.NORMAL
    assert r2.local_world_model.peer_heartbeats["R1"] == 10


def test_r3_maintains_a_separate_local_world_model_copy() -> None:
    _, simulation = build_basic_three_robot_scenario()
    r1, r2, r3 = simulation.robots

    r1.publish_state(simulation.message_bus, timestamp=4)
    r2.receive_peer_messages(simulation.message_bus)
    r3.receive_peer_messages(simulation.message_bus)

    assert r2.local_world_model is not r3.local_world_model
    assert r2.local_world_model.peer_states is not r3.local_world_model.peer_states
    assert r2.local_world_model.peer_states["R1"] == r3.local_world_model.peer_states["R1"]


def test_updated_r1_state_eventually_reaches_peers_through_messages() -> None:
    scenario, simulation = build_basic_three_robot_scenario()
    r1, r2, r3 = simulation.robots
    r1.decide_next_move(scenario.grid)
    destination = r1.decide_next_move(scenario.grid)
    assert destination is not None
    scenario.grid.validate_move(r1.position, destination)
    r1.confirm_move(destination)

    r1.publish_state(simulation.message_bus, timestamp=11)
    r2.receive_peer_messages(simulation.message_bus)
    r3.receive_peer_messages(simulation.message_bus)

    assert r2.local_world_model.peer_states["R1"].pose == destination
    assert r3.local_world_model.peer_states["R1"].pose == destination
    assert r2.local_world_model.peer_heartbeats["R1"] == 11
    assert r3.local_world_model.peer_heartbeats["R1"] == 11


def test_message_bus_transports_unchanged_data_without_right_of_way_api() -> None:
    bus = InProcessPeerMessageBus()
    bus.register("R1")
    bus.register("R2")
    message = PeerStateMessage(
        robot_id="R1",
        timestamp=1,
        pose=Cell(1, 1),
        velocity=(0, 0),
        goal=Cell(2, 1),
        intent=None,
        priority=7,
        battery=99.0,
        state=RobotState.MOVING,
        heartbeat=1,
    )

    bus.publish(message)

    assert bus.drain("R2") == (message,)
    assert not hasattr(bus, "grant_right_of_way")
    assert not hasattr(bus, "resolve_conflict")
    assert not hasattr(bus, "select_winner")


def test_robot_coordination_has_no_direct_global_simulator_robot_dependency() -> None:
    source = inspect.getsource(robot_module)

    assert "global_robots" not in source
    assert "FleetSimulation" not in source