"""Deterministic robot failure and task reassignment scenario."""
from dataclasses import dataclass
from models.grid import Cell, WarehouseGrid
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask
@dataclass(slots=True)
class RobotFailureScenario:
    simulation: FleetSimulation
    task: FleetTask
    def disable_r2_after_start(self) -> Cell:
        for _ in range(3): self.simulation.step()
        r2=next(r for r in self.simulation.robots if r.robot_id=="R2")
        stopped_at=r2.position; self.simulation.disable_robot("R2"); return stopped_at

def build_robot_failure_scenario() -> RobotFailureScenario:
    grid=WarehouseGrid(width=12,height=5)
    robots=(RobotAgent("R1",Cell(1,0)),RobotAgent("R2",Cell(1,2)),RobotAgent("R3",Cell(1,4)))
    dispatcher=DeterministicTaskDispatcher(); task=FleetTask("T-R2",Cell(10,2)); dispatcher.add_task(task); dispatcher.assign_initial(task.task_id,robots[1])
    return RobotFailureScenario(FleetSimulation(grid,robots,task_dispatcher=dispatcher),task)