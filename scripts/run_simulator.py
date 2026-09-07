"""Launch the three-robot SYNQ warehouse simulator."""

from __future__ import annotations

import argparse

from simulation.pygame_app import run_visual_simulator
from simulation.scenarios import build_basic_three_robot_scenario


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the SYNQ three-robot warehouse simulation.")
    parser.add_argument("--seed", type=int, default=0, help="Deterministic scenario seed (default: 0).")
    parser.add_argument("--headless", action="store_true", help="Run the scenario without opening Pygame.")
    parser.add_argument("--auto-quit", action="store_true", help="Close the Pygame window when all robots finish.")
    arguments = parser.parse_args()
    scenario, simulation = build_basic_three_robot_scenario(seed=arguments.seed)
    if arguments.headless:
        simulation.run_until_complete()
        print(f"Completed {len(simulation.robots)} independent routes in {simulation.tick_count} ticks.")
        print(f"Collisions: {simulation.collision_count}")
        return
    run_visual_simulator(scenario, simulation, auto_quit_on_complete=arguments.auto_quit)


if __name__ == "__main__":
    main()