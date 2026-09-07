"""Run the initial headless single-robot motion-planning demonstration."""

from models.grid import Cell, WarehouseGrid
from models.robot_state import RobotState
from simulation.headless import HeadlessSimulation
from simulation.robot import RobotAgent


def main() -> None:
    grid = WarehouseGrid(
        width=7,
        height=5,
        obstacles=frozenset({Cell(2, 1), Cell(2, 2), Cell(2, 3), Cell(4, 3)}),
    )
    robot = RobotAgent(robot_id="AMR-01", position=Cell(0, 2))
    simulation = HeadlessSimulation(grid=grid, robot=robot)
    robot.assign_goal(Cell(6, 2))
    simulation.run_until_complete()

    if robot.state is not RobotState.TASK_COMPLETE:
        raise RuntimeError("Demo robot did not complete its task.")
    print(f"{robot.robot_id} completed route in {simulation.tick_count} ticks.")
    print("Path:", " -> ".join(f"({cell.x},{cell.y})" for cell in simulation.position_history))


if __name__ == "__main__":
    main()
