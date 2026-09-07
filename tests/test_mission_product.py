from dashboard.controller import DemoController, RunState
from models.environment import EnvironmentObjectType
from models.grid import Cell, WarehouseGrid
from models.liveness import HEARTBEAT_TIMEOUT
from models.mission import MissionPhase, Payload, PayloadType, RobotLoadState, TaskType
from models.priority import TaskPriority
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask, TaskStatus


def payload(weight=12.0):
    return Payload("P-100", PayloadType.TOTE, weight, priority=TaskPriority.NORMAL)


def test_mission_validation_rejects_missing_waypoint_pair_and_bad_payload():
    try:
        FleetTask("T", Cell(2, 0), source=Cell(2, 0), payload=payload())
        assert False
    except ValueError:
        pass
    for weight in (0, 1001):
        try:
            payload(weight)
            assert False
        except ValueError:
            pass


def test_capacity_eligibility_and_allocation_are_deterministic():
    task=FleetTask("T",Cell(8,1),task_type=TaskType.PICKUP_DELIVERY,source=Cell(3,1),payload=payload(20))
    low=RobotAgent("R1",Cell(0,1),max_payload_kg=10);eligible=RobotAgent("R2",Cell(0,1),max_payload_kg=30)
    dispatcher=DeterministicTaskDispatcher();dispatcher.add_task(task)
    assert dispatcher.assign_best("T",[low,eligible]).robot_id=="R2"
    assert "within 30.0 kg capacity" in " ".join(task.assignment_reason)
    winners=[]
    for order in (("R2","R1"),("R1","R2")):
        robots={rid:RobotAgent(rid,Cell(0,0)) for rid in order};d=DeterministicTaskDispatcher();t=FleetTask("X",Cell(4,0));d.add_task(t)
        winners.append(d.assign_best("X",[robots[rid] for rid in order]).robot_id)
    assert winners==["R1","R1"]


def test_pickup_load_dropoff_dwell_real_progress_and_unload():
    robot=RobotAgent("R1",Cell(0,1));task=FleetTask("T",Cell(7,1),priority=TaskPriority.HIGH,task_type=TaskType.PICKUP_DELIVERY,source=Cell(2,1),payload=payload(),pickup_dwell_ticks=2,dropoff_dwell_ticks=2)
    dispatcher=DeterministicTaskDispatcher();dispatcher.add_task(task);dispatcher.assign_best("T",[robot]);sim=FleetSimulation(WarehouseGrid(8,3), (robot,),task_dispatcher=dispatcher)
    progress=[];phases=[];loads=[]
    for _ in range(30):
        progress.append(task.progress_percent(robot));phases.append(task.phase);loads.append(robot.load_state)
        if task.status is TaskStatus.COMPLETED:break
        sim.step()
    assert task.status is TaskStatus.COMPLETED and robot.position==Cell(7,1)
    assert MissionPhase.PICKUP_DWELL in phases and MissionPhase.DROPOFF_DWELL in phases
    assert RobotLoadState.EN_ROUTE_TO_PICKUP in loads and RobotLoadState.EN_ROUTE_TO_DROPOFF in loads
    assert robot.load_state is RobotLoadState.EMPTY and robot.current_payload is None
    assert progress==sorted(progress) and progress[-1]==100 and len(set(progress))>5


def test_battery_consumption_is_deterministic_and_loaded_motion_costs_more():
    empty=RobotAgent("R1",Cell(0,0));loaded=RobotAgent("R2",Cell(0,0));loaded.load_payload(payload())
    assert empty.consume_tick_energy(True)==99.92
    assert loaded.consume_tick_energy(True)==99.89
    waiting=RobotAgent("R3",Cell(0,0));waiting.attach_static_map(WarehouseGrid(3,1));waiting.assign_goal(Cell(2,0));waiting.decide_next_move();waiting.restore_waiting_snapshot(("R9",),1)
    before=waiting.battery;waiting.consume_tick_energy(False);assert waiting.battery==before-0.01
    idle=RobotAgent("R4",Cell(0,0));before=idle.battery;idle.consume_tick_energy(False);assert idle.battery==before-0.002


def test_offline_payload_mission_releases_reassigns_and_completes_safely():
    grid=WarehouseGrid(10,5);robots=(RobotAgent("R1",Cell(0,0)),RobotAgent("R2",Cell(0,2)),RobotAgent("R3",Cell(0,4)))
    task=FleetTask("PAY",Cell(8,2),task_type=TaskType.PICKUP_DELIVERY,source=Cell(3,2),payload=payload())
    dispatcher=DeterministicTaskDispatcher();dispatcher.add_task(task);dispatcher.assign_initial("PAY",robots[1]);sim=FleetSimulation(grid,robots,task_dispatcher=dispatcher)
    sim.step();sim.disable_robot("R2")
    for _ in range(HEARTBEAT_TIMEOUT+3):sim.step()
    assert task.release_count==1 and task.assigned_robot_id in {"R1","R3"}
    sim.run_until_complete(100)
    assert task.status is TaskStatus.COMPLETED and sim.collision_count==0 and robots[1].state.name=="OFFLINE"


def test_full_demo_initialization_and_complete_sequence_are_deterministic():
    results=[]
    for _ in range(2):
        controller=DemoController();controller.load_scenario("full_demo")
        initial=[(r.robot_id,r.position) for r in controller.simulation.robots]
        for _ in range(180):
            if controller.run_state is RunState.SCENARIO_COMPLETE:break
            controller.step()
        snap=controller.snapshot();results.append((initial,controller.simulation.tick_count,snap["metrics"]["collisions"],[e["type"] for e in snap["events"]],[(t["task_id"],t["status"],t["assigned_robot_id"]) for t in snap["tasks"]]))
        assert snap["demo_phase"]["current"]==8 and snap["metrics"]["collisions"]==0
        for event in ("HIGH_PRIORITY_MISSION","DYNAMIC_OBSTACLE_ADDED","FAILURE_INJECTED","TASK_RELEASED","TASK_REASSIGNED"):assert event in results[-1][3]
    assert results[0]==results[1]


def test_typed_environment_object_metadata_is_separate_from_occupancy():
    sim=FleetSimulation(WarehouseGrid(4,3),(RobotAgent("R1",Cell(0,0)),));sim.add_dynamic_obstacle(Cell(2,1),EnvironmentObjectType.FORKLIFT)
    assert Cell(2,1) in sim.physical_obstacles and sim.environment_objects[Cell(2,1)] is EnvironmentObjectType.FORKLIFT
