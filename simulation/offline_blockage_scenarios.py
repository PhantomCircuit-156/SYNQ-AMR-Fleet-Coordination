"""Failure scenarios where an offline AMR physically blocks another route."""
from dataclasses import dataclass
from models.grid import Cell, WarehouseGrid
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent

@dataclass(slots=True)
class OfflineBlockageScenario:
    simulation: FleetSimulation
    def fail_r2(self) -> Cell:
        for _ in range(2): self.simulation.step()
        r2=next(r for r in self.simulation.robots if r.robot_id=="R2")
        position=r2.position; self.simulation.disable_robot("R2"); return position

def build_offline_robot_blockage_scenario() -> OfflineBlockageScenario:
    grid=WarehouseGrid(9,5)
    r1=RobotAgent("R1",Cell(0,2)); r2=RobotAgent("R2",Cell(2,2)); r3=RobotAgent("R3",Cell(0,4))
    r1.assign_goal(Cell(8,2)); r2.assign_goal(Cell(6,2))
    return OfflineBlockageScenario(FleetSimulation(grid,(r1,r2,r3)))

def build_offline_robot_no_route_scenario() -> OfflineBlockageScenario:
    obstacles=frozenset(Cell(x,y) for x in range(5) for y in (0,2))
    grid=WarehouseGrid(5,3,obstacles)
    r1=RobotAgent("R1",Cell(0,1)); r2=RobotAgent("R2",Cell(2,1)); r3=RobotAgent("R3",Cell(4,1))
    r1.assign_goal(Cell(4,1)); r2.assign_goal(Cell(3,1))
    return OfflineBlockageScenario(FleetSimulation(grid,(r1,r2,r3)))