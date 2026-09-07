from coordination.intent import IntentManager
from coordination.local_world import LocalWorldModel
from models.corridor import CorridorRequest, CorridorToken
from models.grid import Cell, WarehouseGrid
from models.liveness import HEARTBEAT_TIMEOUT, LivenessEventType
from models.messages import PeerStateMessage
from models.priority import TaskPriority
from models.robot_state import RobotState
from simulation.failure_scenarios import build_robot_failure_scenario
from simulation.robot import RobotAgent
from tasks.lifecycle import TaskEventType, TaskStatus

def peer_message(timestamp=1):
    pose=Cell(2,2); intent=IntentManager().generate_hold("R2",pose,timestamp)
    return PeerStateMessage("R2",timestamp,pose,(0,0),Cell(8,2),intent,TaskPriority.NORMAL,100.0,RobotState.MOVING,timestamp,corridor_requests=(CorridorRequest("C1","R2",timestamp,Cell(3,2),Cell(7,2)),),corridor_tokens=(CorridorToken("C1","R2",timestamp,timestamp+100),))

def test_heartbeat_update_and_timeout_marks_peer_offline_once():
    world=LocalWorldModel("R1"); world.receive_peer_state(peer_message(4))
    assert world.peer_last_seen["R2"]==4
    assert world.expire_stale_peers(4+HEARTBEAT_TIMEOUT)==()
    assert world.expire_stale_peers(5+HEARTBEAT_TIMEOUT)==("R2",)
    assert world.expire_stale_peers(20)==()
    assert [e.event_type for e in world.liveness_events]==[LivenessEventType.HEARTBEAT_TIMEOUT,LivenessEventType.ROBOT_OFFLINE]

def test_timeout_expires_intent_corridor_state_and_future_conflicts():
    r1=RobotAgent("R1",Cell(2,2)); r1.attach_static_map(WarehouseGrid(10,5)); r1.assign_goal(Cell(8,2)); r1.decide_next_move()
    r1.local_world_model.receive_peer_state(peer_message(1))
    assert "R2" in r1.local_world_model.peer_intents and "R2" in r1.local_world_model.peer_corridor_tokens
    r1.evaluate_peer_liveness(1+HEARTBEAT_TIMEOUT+1)
    assert "R2" in r1.local_world_model.offline_peers
    assert "R2" not in r1.local_world_model.peer_states
    assert "R2" not in r1.local_world_model.peer_intents
    assert "R2" not in r1.local_world_model.peer_corridor_tokens
    assert all("R2" not in conflict.robot_ids for conflict in r1.detect_conflicts())

def test_robot_failure_releases_once_reassigns_and_continues_without_collision():
    scenario=build_robot_failure_scenario(); simulation=scenario.simulation; stopped=scenario.disable_r2_after_start(); r1,r2,r3=simulation.robots
    assert r2.state is RobotState.OFFLINE
    for _ in range(HEARTBEAT_TIMEOUT+3): simulation.step()
    assert "R2" in r1.local_world_model.offline_peers and "R2" in r3.local_world_model.offline_peers
    assert scenario.task.release_count==1
    assert scenario.task.assigned_robot_id=="R1"
    assert [e.event_type for e in simulation.task_dispatcher.events]==[TaskEventType.TASK_RELEASED,TaskEventType.TASK_REASSIGNED]
    frozen=r2.position
    simulation.run_until_complete(max_ticks=100)
    assert r2.position==frozen==stopped
    assert r1.position==scenario.task.goal and scenario.task.status is TaskStatus.COMPLETED
    assert simulation.collision_count==0
    assert all(len(s.values())==len(set(s.values())) for s in simulation.occupancy_history)