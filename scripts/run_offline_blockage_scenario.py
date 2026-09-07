"""Run the deterministic offline-robot physical-blockage demo."""
import json
from simulation.offline_blockage_scenarios import build_offline_robot_blockage_scenario

def main() -> None:
    scenario=build_offline_robot_blockage_scenario(); blocked=scenario.fail_r2(); simulation=scenario.simulation
    simulation.run_until_complete(max_ticks=100); r1,r2,r3=simulation.robots
    print(json.dumps({"scenario":"offline_robot_blockage","blocked_cell":[blocked.x,blocked.y],"r1_state":r1.state.name,"r1_position":[r1.position.x,r1.position.y],"r1_replans":r1.conflict_metrics.replans_succeeded,"r2_state":r2.state.name,"collisions":simulation.collision_count,"ticks":simulation.tick_count},sort_keys=True))
if __name__ == "__main__": main()