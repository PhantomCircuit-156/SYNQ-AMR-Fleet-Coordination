"""One reusable MiR100 robot-local controller for both benchmark modes."""
import json
import os
import sys
from math import atan2, cos, hypot, inf, sin
from pathlib import Path

synq_root = Path(os.environ.get("SYNQ_ROOT", str(Path(__file__).resolve().parents[3])))
if not (synq_root / "simulation" / "robot.py").is_file():
    raise RuntimeError(f"RobotAgent not found at {synq_root}")
sys.path.insert(0, str(synq_root))
from controller import Robot
from models.grid import Cell, WarehouseGrid
from models.robot_state import RobotState
from simulation.robot import RobotAgent
from benchmark_config import (ARRIVAL_METERS, ASSIGNMENTS, Assignment,
                              CONFLICT_CELLS, GRID_SIZE, HALF_TRACK,
                              HEADING_TOLERANCE, MAX_ANGULAR, MAX_LINEAR,
                              PEER_TIMEOUT_SECONDS, SAFETY_DISTANCE_METERS,
                              WAYPOINT_TIMEOUT_SECONDS, WHEEL_RADIUS, to_world)
from benchmark_coordination import PeerIntent, should_wait

def clamp(value, bound):
    return max(-bound, min(bound, value))

class Mir100Drive:
    """Policy-free wheel and encoder adapter."""
    def __init__(self, robot, timestep, assignment):
        self.left = robot.getDevice("middle_left_wheel_joint")
        self.right = robot.getDevice("middle_right_wheel_joint")
        self.le = robot.getDevice("middle_left_wheel_joint_sensor")
        self.re = robot.getDevice("middle_right_wheel_joint_sensor")
        for sensor in (self.le, self.re):
            sensor.enable(timestep)
        for motor in (self.left, self.right):
            motor.setPosition(inf)
            motor.setVelocity(0)
        self.x, self.y = to_world(Cell(*assignment.start))
        self.heading = assignment.heading
        self.linear = self.angular = self.distance = 0.0
        self.last = None

    def update(self, now):
        sample = (now, self.le.getValue(), self.re.getValue())
        old = self.last
        self.last = sample
        if old is None or now <= old[0]:
            return
        dt = now - old[0]
        dl = (sample[1] - old[1]) * WHEEL_RADIUS
        dr = (sample[2] - old[2]) * WHEEL_RADIUS
        distance = (dl + dr) / 2
        turn = (dr - dl) / (2 * HALF_TRACK)
        self.x += distance * cos(self.heading + turn / 2)
        self.y += distance * sin(self.heading + turn / 2)
        self.heading = atan2(sin(self.heading + turn), cos(self.heading + turn))
        self.linear, self.angular = distance / dt, turn / dt
        self.distance += abs(distance)

    def stop(self):
        self.left.setVelocity(0)
        self.right.setVelocity(0)

    def distance_to(self, target):
        return hypot(target[0] - self.x, target[1] - self.y)

    def move_toward(self, target):
        desired = atan2(target[1] - self.y, target[0] - self.x)
        error = atan2(sin(desired - self.heading), cos(desired - self.heading))
        angular = clamp(1.5 * error, MAX_ANGULAR)
        linear = min(MAX_LINEAR, 0.7 * self.distance_to(target)) if abs(error) <= HEADING_TOLERANCE else 0
        self.left.setVelocity(clamp((linear - angular * HALF_TRACK) / WHEEL_RADIUS, 20))
        self.right.setVelocity(clamp((linear + angular * HALF_TRACK) / WHEEL_RADIUS, 20))

def future_path(agent):
    if not agent.path or agent.position not in agent.path:
        return ()
    index = agent.path.index(agent.position)
    return tuple((cell.x, cell.y) for cell in agent.path[index + 1:index + 6])

def legacy_single_robot(robot):
    """Run only the original R1 probe in the user's existing world."""
    timestep = int(robot.getBasicTimeStep())
    drive = Mir100Drive(robot, timestep, Assignment((3, 3), (5, 4), 0.0, 1))
    if robot.getName() != "Mir100":
        print(f"SYNQ {robot.getName()}: legacy world secondary robot is parked; open worlds/synq_three_robot.wbt for the three-robot benchmark", flush=True)
        while robot.step(timestep) != -1:
            drive.stop()
        return
    agent = RobotAgent(robot_id="R1", position=Cell(0, 0))
    agent.attach_static_map(WarehouseGrid(4, 3))
    agent.assign_goal(Cell(2, 1))
    agent.decide_next_move()  # plan first
    pending = agent.decide_next_move()
    settled = 0
    pending_since = robot.getTime()
    print(f"SYNQ R1 single-robot A* path={agent.path}", flush=True)
    try:
        while robot.step(timestep) != -1:
            now = robot.getTime()
            drive.update(now)
            if pending is None:
                if agent.state is RobotState.TASK_COMPLETE:
                    drive.stop()
                    continue
                pending = agent.decide_next_move()
                pending_since = now
                if pending is None:
                    drive.stop()
                    raise RuntimeError("R1 cannot continue")
            target = (float(pending.x), float(pending.y))
            if drive.distance_to(target) <= ARRIVAL_METERS:
                drive.stop()
                settled = settled + 1 if abs(drive.linear) < 0.04 and abs(drive.angular) < 0.08 else 0
                if settled >= 3:
                    agent.confirm_move(pending)
                    print(f"SYNQ R1 arrived {pending}; state={agent.state.name}", flush=True)
                    if agent.state is RobotState.TASK_COMPLETE:
                        print("SYNQ R1 TASK_COMPLETE", flush=True)
                    pending = None
                    settled = 0
            else:
                settled = 0
                if now - pending_since > WAYPOINT_TIMEOUT_SECONDS:
                    raise RuntimeError(f"R1 could not reach {pending}")
                drive.move_toward(target)
    finally:
        drive.stop()

def main():
    robot = Robot()
    robot_id = robot.getName()
    if robot_id.startswith("Mir100"):
        return legacy_single_robot(robot)
    if robot_id not in ASSIGNMENTS:
        raise RuntimeError(f"Unexpected robot name: {robot_id}")
    mode = os.environ.get("MODE", "synq").lower()
    if mode not in {"baseline", "synq"}:
        raise ValueError("MODE must be baseline or synq")
    assignment = ASSIGNMENTS[robot_id]
    timestep = int(robot.getBasicTimeStep())
    drive = Mir100Drive(robot, timestep, assignment)
    emitter = robot.getDevice("peer_emitter")
    receiver = robot.getDevice("peer_receiver")
    receiver.enable(timestep)
    agent = RobotAgent(robot_id=robot_id, position=Cell(*assignment.start))
    agent.attach_static_map(WarehouseGrid(GRID_SIZE, GRID_SIZE))
    agent.assign_goal(Cell(*assignment.goal))
    agent.decide_next_move()  # PLANNING computes a path on its first call
    pending = agent.decide_next_move()
    if pending is None:
        raise RuntimeError(f"{robot_id}: no route")
    print(f"SYNQ {robot_id}: controller active mode={mode} start={assignment.start} goal={assignment.goal} priority={assignment.priority} A* path={agent.path}", flush=True)
    peers = {}
    peer_packets = 0
    announced_peers = False
    waiting = 0.0
    conflicts = yields = stops = 0
    was_blocked = False
    settled = 0
    pending_since = task_start = last_time = robot.getTime()
    completed_at = None
    emergency_stop = False
    try:
        while robot.step(timestep) != -1:
            now = robot.getTime()
            dt = max(0, now - last_time)
            last_time = now
            drive.update(now)
            while receiver.getQueueLength():
                try:
                    data = json.loads(receiver.getString())
                    if data.get("type") == "ESTOP":
                        emergency_stop = True
                    elif data.get("type") == "intent" and data["robot_id"] != robot_id:
                        peer = PeerIntent.from_dict(data)
                        peers[peer.robot_id] = peer
                        peer_packets += 1
                except (ValueError, KeyError, TypeError) as error:
                    print(f"{robot_id}: malformed radio packet: {error}", flush=True)
                receiver.nextPacket()
            fresh = tuple(p for p in peers.values() if now - p.timestamp <= PEER_TIMEOUT_SECONDS)
            if not announced_peers and len({p.robot_id for p in fresh}) == 2:
                print(f"SYNQ {robot_id}: peer intents active from {sorted(peers)}", flush=True)
                announced_peers = True
            mine = PeerIntent(robot_id, now, (agent.position.x, agent.position.y),
                              (drive.x, drive.y), assignment.goal, future_path(agent),
                              "TASK_COMPLETE" if completed_at is not None else "MOVING",
                              assignment.priority, waiting)
            blocked, blockers = should_wait(mode, mine, fresh, CONFLICT_CELLS, now,
                                            PEER_TIMEOUT_SECONDS)
            blocked |= any(hypot(drive.x - p.position[0], drive.y - p.position[1])
                           < SAFETY_DISTANCE_METERS for p in fresh)
            if blocked and not was_blocked:
                conflicts += 1
                yields += 1
                stops += 1
                print(f"SYNQ {robot_id}: YIELD to {blockers} t={now:.2f}", flush=True)
            if not blocked and was_blocked:
                print(f"SYNQ {robot_id}: RESUME t={now:.2f}", flush=True)
            was_blocked = blocked
            if blocked:
                waiting += dt
                pending_since += dt
                drive.stop()
            elif emergency_stop or completed_at is not None:
                drive.stop()
            elif pending is not None:
                target = to_world(pending)
                if drive.distance_to(target) <= ARRIVAL_METERS:
                    drive.stop()
                    settled = settled + 1 if abs(drive.linear) < 0.04 and abs(drive.angular) < 0.08 else 0
                    if settled >= 3:
                        agent.confirm_move(pending)
                        print(f"SYNQ {robot_id}: arrived {pending} t={now:.2f}", flush=True)
                        pending = None
                        settled = 0
                else:
                    settled = 0
                    if now - pending_since > WAYPOINT_TIMEOUT_SECONDS:
                        raise RuntimeError(f"{robot_id}: waypoint timeout {pending}")
                    drive.move_toward(target)
            if pending is None and completed_at is None and not emergency_stop:
                if agent.state is RobotState.TASK_COMPLETE:
                    completed_at = now
                    print(f"SYNQ {robot_id}: TASK_COMPLETE t={now:.2f}", flush=True)
                else:
                    pending = agent.decide_next_move()
                    pending_since = now
                    if pending is None:
                        raise RuntimeError(f"{robot_id}: route stopped in {agent.state.name}")
            packet = {"type": "intent", "robot_id": robot_id, "timestamp": now,
                      "cell": (agent.position.x, agent.position.y),
                      "position": (drive.x, drive.y), "goal": assignment.goal,
                      "next_waypoints": future_path(agent),
                      "status": "ESTOP" if emergency_stop else "TASK_COMPLETE" if completed_at is not None else "WAITING" if blocked else "MOVING",
                      "priority": assignment.priority, "waiting_time": waiting,
                      "task_start": task_start, "completed_at": completed_at,
                      "conflicts": conflicts, "yields": yields, "stops": stops,
                      "distance": drive.distance, "replans": 0,
                      "peer_packets": peer_packets,
                      "peer_ids": sorted(peers)}
            emitter.send(json.dumps(packet).encode("utf-8"))
    finally:
        drive.stop()

if __name__ == "__main__":
    main()
