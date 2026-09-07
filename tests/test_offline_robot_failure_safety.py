from models.liveness import HEARTBEAT_TIMEOUT, LivenessEventType
from models.robot_state import RobotState
from simulation.failure_scenarios import build_robot_failure_scenario
from simulation.offline_blockage_scenarios import build_offline_robot_blockage_scenario, build_offline_robot_no_route_scenario
from tasks.lifecycle import TaskEventType, TaskStatus

def test_each_active_observer_reports_r2_offline_once_without_repeat_events():
    scenario=build_robot_failure_scenario(); simulation=scenario.simulation; scenario.disable_r2_after_start()
    for _ in range(HEARTBEAT_TIMEOUT+4): simulation.step()
    r1,r2,r3=simulation.robots
    for observer in (r1,r3):
        events=[event for event in observer.local_world_model.liveness_events if event.peer_robot_id=="R2"]
        assert [(event.observer_robot_id,event.event_type) for event in events]==[(observer.robot_id,LivenessEventType.HEARTBEAT_TIMEOUT),(observer.robot_id,LivenessEventType.ROBOT_OFFLINE)]
    before=[len(robot.local_world_model.liveness_events) for robot in (r1,r3)]
    for _ in range(4): simulation.step()
    assert [len(robot.local_world_model.liveness_events) for robot in (r1,r3)]==before

def test_offline_robot_blockage_is_physical_and_robot_replans_around_it():
    scenario=build_offline_robot_blockage_scenario(); simulation=scenario.simulation; blocked=scenario.fail_r2(); r1,r2,r3=simulation.robots
    assert blocked in simulation.physical_obstacles
    simulation.run_until_complete(max_ticks=100)
    assert r2.position==blocked and r2.state is RobotState.OFFLINE
    assert blocked not in simulation.position_history["R1"]
    assert r1.state is RobotState.TASK_COMPLETE and r1.position==r1.goal
    assert r1.conflict_metrics.replans_requested==1 and r1.conflict_metrics.replans_succeeded==1
    assert "R2" in r1.local_world_model.offline_peers and "R2" in r3.local_world_model.offline_peers
    assert simulation.collision_count==0

def test_offline_robot_blockage_with_no_alternate_route_stops_safely():
    scenario=build_offline_robot_no_route_scenario(); simulation=scenario.simulation; blocked=scenario.fail_r2(); r1,r2,_=simulation.robots
    for _ in range(3): simulation.step()
    assert blocked in simulation.physical_obstacles
    assert r1.state is RobotState.UNREACHABLE
    assert r1.position != blocked and r1.conflict_metrics.no_route_stops==1
    assert r2.position==blocked and simulation.collision_count==0

def test_multiple_peer_observations_release_and_reassign_task_once():
    scenario=build_robot_failure_scenario(); simulation=scenario.simulation; scenario.disable_r2_after_start()
    for _ in range(HEARTBEAT_TIMEOUT+4): simulation.step()
    task=scenario.task
    assert task.release_count==1 and task.assigned_robot_id=="R1"
    assert [event.event_type for event in simulation.task_dispatcher.events]==[TaskEventType.TASK_RELEASED,TaskEventType.TASK_REASSIGNED]
    assert sum(task.assigned_robot_id==robot.robot_id for robot in simulation.robots if robot.enabled)==1
    simulation.run_until_complete(max_ticks=100)
    assert task.status is TaskStatus.COMPLETED and simulation.collision_count==0