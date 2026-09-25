"""Read-only ground-truth metrics and emergency stop for the benchmark world."""
import json
import os
import sys
from math import hypot
from pathlib import Path
from controller import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "synq_controller"))
from benchmark_metrics import result_summary
from benchmark_config import (ASSIGNMENTS, CELL_SIZE_METERS,
                              SAFETY_DISTANCE_METERS, WORLD_ORIGIN)

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    mode = os.environ.get("MODE", "synq").lower()
    receiver = robot.getDevice("telemetry_receiver")
    emitter = robot.getDevice("safety_emitter")
    receiver.enable(timestep)
    nodes = {name: robot.getFromDef(name) for name in ("R1", "R2", "R3")}
    if any(node is None for node in nodes.values()):
        raise RuntimeError("Benchmark requires DEF R1, R2, and R3")
    telemetry = {}
    start = robot.getTime()
    collisions = 0
    active_pairs = set()
    minimum_separation = float("inf")
    start_poses = None
    previous_poses = None
    physical_distance = {name: 0.0 for name in nodes}
    completed_logged = set()
    next_pose_log = 10.0
    print(f"BENCHMARK START mode={mode} t={start:.3f}", flush=True)
    while robot.step(timestep) != -1:
        now = robot.getTime()
        while receiver.getQueueLength():
            try:
                packet = json.loads(receiver.getString())
                if packet.get("type") == "intent":
                    telemetry[packet["robot_id"]] = packet
            except (ValueError, KeyError, TypeError) as error:
                print(f"METRICS ignored packet: {error}", flush=True)
            receiver.nextPacket()
        positions = {name: node.getPosition() for name, node in nodes.items()}
        if start_poses is None:
            start_poses = positions.copy()
            print("GROUND_TRUTH_START " + json.dumps(start_poses), flush=True)
        if previous_poses is not None:
            for name in nodes:
                a, b = positions[name], previous_poses[name]
                physical_distance[name] += hypot(a[0] - b[0], a[1] - b[1])
        previous_poses = positions
        if now - start >= next_pose_log:
            summary = " ".join(f"{name}=({positions[name][0]:.2f},{positions[name][1]:.2f})" for name in nodes)
            print(f"GROUND_TRUTH t={now:.2f} {summary}", flush=True)
            next_pose_log += 10.0
        for name in nodes:
            if (name in telemetry and telemetry[name].get("completed_at") is not None
                    and name not in completed_logged):
                completed_logged.add(name)
                print(f"SYNQ {name}: TASK_COMPLETE report; ground_truth_pose={positions[name]}", flush=True)
        new_pairs = set()
        names = tuple(nodes)
        for i, left in enumerate(names):
            for right in names[i + 1:]:
                a, b = positions[left], positions[right]
                distance = hypot(a[0] - b[0], a[1] - b[1])
                minimum_separation = min(minimum_separation, distance)
                if distance < SAFETY_DISTANCE_METERS:
                    pair = (left, right)
                    new_pairs.add(pair)
                    if pair not in active_pairs:
                        collisions += 1
                        print(f"COLLISION {left} {right} t={now:.3f} distance={distance:.3f}", flush=True)
                        emitter.send(json.dumps({"type": "ESTOP", "robots": pair}).encode("utf-8"))
        active_pairs = new_pairs
        finished = len(telemetry) == 3 and all(
            telemetry[name].get("completed_at") is not None for name in names)
        timeout = now - start > 210
        if not finished and not timeout and collisions == 0:
            continue
        completion = {name: telemetry[name]["completed_at"] for name in names
                      if name in telemetry and telemetry[name].get("completed_at") is not None}
        goal_errors = {}
        for name in nodes:
            gx, gy = ASSIGNMENTS[name].goal
            goal_x = WORLD_ORIGIN[0] + gx * CELL_SIZE_METERS
            goal_y = WORLD_ORIGIN[1] + gy * CELL_SIZE_METERS
            goal_errors[name] = hypot(positions[name][0] - goal_x, positions[name][1] - goal_y)
        physically_complete = finished and all(
            goal_errors[name] <= 0.30 and physical_distance[name] >= 4.5
            for name in nodes)
        peer_exchange_complete = finished and all(
            telemetry[name].get("peer_packets", 0) > 0 and
            set(telemetry[name].get("peer_ids", ())) >= set(nodes) - {name}
            for name in nodes)
        validated = physically_complete and peer_exchange_complete
        if validated:
            result = result_summary(
                mode, completion,
                {name: telemetry[name]["waiting_time"] for name in names},
                {name: telemetry[name]["conflicts"] for name in names},
                {name: telemetry[name]["yields"] for name in names},
                collisions, start)
        else:
            result = {"mode": mode, "completion": completion, "makespan": None,
                      "total_waiting": sum(p.get("waiting_time", 0) for p in telemetry.values()),
                      "conflicts": sum(p.get("conflicts", 0) for p in telemetry.values()),
                      "yields": sum(p.get("yields", 0) for p in telemetry.values()),
                      "collisions": collisions,
                      "failed": "collision" if collisions else
                                "physical_goal_mismatch" if finished and not physically_complete else
                                "peer_exchange_incomplete" if finished else "timeout"}
        result["units"] = {"completion": "seconds", "makespan": "seconds",
                           "total_waiting": "seconds", "distance": "meters",
                           "ground_truth_distance": "meters",
                           "physical_goal_error": "meters",
                           "minimum_separation": "meters"}
        result["safety_threshold_breaches"] = collisions
        result["minimum_separation"] = minimum_separation
        result["ground_truth_start"] = start_poses
        result["ground_truth_end"] = positions
        result["ground_truth_distance"] = physical_distance
        result["physical_goal_error"] = goal_errors
        result["physically_complete"] = physically_complete
        result["peer_exchange_complete"] = peer_exchange_complete
        result["peer_packets"] = {name: telemetry[name].get("peer_packets", 0) for name in telemetry}
        result["stops"] = sum(p.get("stops", 0) for p in telemetry.values())
        result["distance"] = sum(p.get("distance", 0) for p in telemetry.values())
        result["replans"] = sum(p.get("replans", 0) for p in telemetry.values())
        print("===== SYNQ EXPERIMENT RESULT =====", flush=True)
        print(f"Mode: {mode}", flush=True)
        for name in names:
            print(f"{name} completion: {completion.get(name, 'N/A')}", flush=True)
        for label, key in (("Makespan", "makespan"), ("Total waiting", "total_waiting"),
                           ("Conflicts", "conflicts"), ("Yields", "yields"),
                           ("Collisions", "collisions")):
            print(f"{label}: {result[key]}", flush=True)
        print(f"Physical goals: {'PASS' if physically_complete else 'FAIL'}", flush=True)
        print(f"Peer intent exchange: {'PASS' if peer_exchange_complete else 'FAIL'}", flush=True)
        print("=================================", flush=True)
        print("SYNQ_RESULT_JSON " + json.dumps(result, sort_keys=True), flush=True)
        result_file = os.environ.get("SYNQ_RESULT_FILE")
        if result_file:
            Path(result_file).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        if os.environ.get("SYNQ_BATCH") == "1":
            for _ in range(4):
                robot.step(timestep)  # let Webots collect controller stdout
            robot.simulationQuit(0 if validated and collisions == 0 else 1)
        break

if __name__ == "__main__":
    main()
