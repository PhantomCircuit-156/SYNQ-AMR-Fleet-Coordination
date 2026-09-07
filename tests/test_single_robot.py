from models.grid import Cell, WarehouseGrid
from models.robot_state import RobotState
from simulation.headless import HeadlessSimulation
from simulation.robot import RobotAgent


def build_simulation() -> tuple[WarehouseGrid, RobotAgent, HeadlessSimulation]:
    grid = WarehouseGrid(width=5, height=3, obstacles=frozenset({Cell(2, 1)}))
    robot = RobotAgent(robot_id="AMR-01", position=Cell(0, 1))
    return grid, robot, HeadlessSimulation(grid=grid, robot=robot)


def test_robot_state_transitions() -> None:
    _, robot, simulation = build_simulation()

    assert robot.state is RobotState.IDLE
    robot.assign_goal(Cell(4, 1))
    assert robot.state is RobotState.PLANNING

    simulation.step()
    assert robot.state is RobotState.MOVING

    simulation.run_until_complete()
    assert robot.state is RobotState.TASK_COMPLETE


def test_robot_reaches_target_via_cell_by_cell_motion() -> None:
    _, robot, simulation = build_simulation()
    target = Cell(4, 1)
    robot.assign_goal(target)

    simulation.run_until_complete()

    assert robot.position == target
    assert simulation.position_history[0] == Cell(0, 1)
    assert len(simulation.position_history) > 2


def test_robot_never_walks_through_an_obstacle() -> None:
    grid, robot, simulation = build_simulation()
    robot.assign_goal(Cell(4, 1))

    simulation.run_until_complete()

    assert all(grid.is_walkable(cell) for cell in simulation.position_history)
    assert Cell(2, 1) not in simulation.position_history
    for source, destination in zip(simulation.position_history, simulation.position_history[1:]):
        assert abs(source.x - destination.x) + abs(source.y - destination.y) == 1
