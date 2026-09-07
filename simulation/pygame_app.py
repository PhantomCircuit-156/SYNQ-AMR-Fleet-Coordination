"""Pygame visualization for the intentionally non-conflicting three-robot scenario."""

from __future__ import annotations

from simulation.scenarios import WarehouseScenario
from simulation.fleet import FleetSimulation


def run_visual_simulator(
    scenario: WarehouseScenario, simulation: FleetSimulation, tick_rate: int = 5, auto_quit_on_complete: bool = False
) -> None:
    """Render the warehouse while independently planned robot motion advances."""
    try:
        import pygame
    except ImportError as error:  # pragma: no cover - environment-dependent
        raise RuntimeError("Pygame is required. Install the project dependencies first.") from error

    cell_size = 24
    footer_height = 86
    pygame.init()
    screen = pygame.display.set_mode((scenario.grid.width * cell_size, scenario.grid.height * cell_size + footer_height))
    pygame.display.set_caption("SYNQ — Decentralized AMR Fleet (No Coordination Yet)")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 20)
    small_font = pygame.font.Font(None, 16)
    running = True
    accumulator = 0

    while running:
        elapsed = clock.tick(60)
        accumulator += elapsed
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if accumulator >= 1000 // tick_rate and not _all_complete(simulation):
            simulation.step()
            accumulator = 0
        completed = _all_complete(simulation)
        _draw(screen, pygame, font, small_font, scenario, simulation, cell_size, footer_height)
        pygame.display.flip()
        if auto_quit_on_complete and completed:
            running = False
    pygame.quit()


def _all_complete(simulation: FleetSimulation) -> bool:
    return all(robot.state.name == "TASK_COMPLETE" for robot in simulation.robots)


def _draw(screen, pygame, font, small_font, scenario, simulation, cell_size, footer_height) -> None:
    screen.fill((245, 247, 250))
    for zone in scenario.zones:
        rect = pygame.Rect(zone.left * cell_size, zone.top * cell_size, zone.width * cell_size, zone.height * cell_size)
        pygame.draw.rect(screen, zone.color, rect, width=2)
        screen.blit(font.render(zone.name, True, zone.color), (rect.x + 4, rect.y + 4))
    for obstacle in scenario.grid.obstacles:
        rect = pygame.Rect(obstacle.x * cell_size, obstacle.y * cell_size, cell_size, cell_size)
        pygame.draw.rect(screen, (84, 88, 92), rect)
    for x in range(scenario.grid.width + 1):
        pygame.draw.line(screen, (220, 224, 230), (x * cell_size, 0), (x * cell_size, scenario.grid.height * cell_size))
    for y in range(scenario.grid.height + 1):
        pygame.draw.line(screen, (220, 224, 230), (0, y * cell_size), (scenario.grid.width * cell_size, y * cell_size))

    for robot in simulation.robots:
        color = scenario.robot_colors[robot.robot_id]
        for cell in robot.path[robot._path_index + 1 :]:
            center = (cell.x * cell_size + cell_size // 2, cell.y * cell_size + cell_size // 2)
            pygame.draw.circle(screen, color, center, 3)
        if robot.goal is not None:
            goal_rect = pygame.Rect(robot.goal.x * cell_size + 4, robot.goal.y * cell_size + 4, cell_size - 8, cell_size - 8)
            pygame.draw.rect(screen, color, goal_rect, width=2)
        center = (robot.position.x * cell_size + cell_size // 2, robot.position.y * cell_size + cell_size // 2)
        pygame.draw.circle(screen, color, center, cell_size // 2 - 3)
        label = font.render(robot.robot_id, True, (255, 255, 255))
        screen.blit(label, label.get_rect(center=center))

    footer_y = scenario.grid.height * cell_size
    pygame.draw.rect(screen, (29, 38, 52), (0, footer_y, scenario.grid.width * cell_size, footer_height))
    header = font.render(f"Tick {simulation.tick_count} | collisions: {simulation.collision_count}", True, (255, 255, 255))
    screen.blit(header, (10, footer_y + 8))
    for index, robot in enumerate(simulation.robots):
        details = f"{robot.robot_id}: {robot.state.name}  position=({robot.position.x},{robot.position.y})"
        screen.blit(small_font.render(details, True, scenario.robot_colors[robot.robot_id]), (10, footer_y + 31 + index * 16))