"""Run the deterministic peer-liveness and task-reassignment demo."""
import json
from simulation.failure_scenarios import build_robot_failure_scenario

def main() -> None:
    scenario=build_robot_failure_scenario(); stopped=scenario.disable_r2_after_start(); simulation=scenario.simulation
    simulation.run_until_complete(max_ticks=100)
    liveness=[event.event_type.value for robot in simulation.robots if robot.enabled for event in robot.local_world_model.liveness_events]
    task_events=[event.event_type.value for event in simulation.task_dispatcher.events]
    print(json.dumps({"scenario":"robot_failure","disabled_at":[stopped.x,stopped.y],"task_status":scenario.task.status.value,"assigned_robot":scenario.task.assigned_robot_id,"release_count":scenario.task.release_count,"liveness_events":liveness,"task_events":task_events,"ticks":simulation.tick_count,"collisions":simulation.collision_count},sort_keys=True))
if __name__ == "__main__": main()