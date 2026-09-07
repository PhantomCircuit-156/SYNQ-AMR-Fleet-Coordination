"""Run the corridor protocol and intentional deadlock demonstrations."""

from __future__ import annotations

import json
from dataclasses import replace

from coordination.deadlock import DeadlockManager
from models.robot_state import RobotState
from simulation.corridor_scenarios import (
    build_intentional_deadlock_worlds,
    build_opposing_corridor_scenario,
    build_recoverable_deadlock_scenario,
)


def main() -> None:
    simulation = build_opposing_corridor_scenario()
    simulation.run_until_complete(max_ticks=150)
    print("corridor:", json.dumps(simulation.metrics_snapshot(), sort_keys=True))

    recovery = build_recoverable_deadlock_scenario()
    recovery.run_until_complete(max_ticks=150)
    print(
        "deadlock-recovery:",
        json.dumps(
            {
                "metrics": recovery.metrics_snapshot(),
                "events": {
                    robot.robot_id: [event.event_type.value for event in robot.deadlock_events]
                    for robot in recovery.robots
                },
            },
            sort_keys=True,
        ),
    )

    r1_world, r2_world, corridor = build_intentional_deadlock_worlds()
    r1_manager = DeadlockManager("R1")
    r2_manager = DeadlockManager("R2")
    r1_resolution = r1_manager.evaluate(r1_world, (corridor,), timestamp=6)
    r2_resolution = r2_manager.evaluate(r2_world, (corridor,), timestamp=6)
    moving_r2 = replace(
        r1_world.peer_states["R2"],
        timestamp=7,
        heartbeat=7,
        state=RobotState.MOVING,
        waiting_ticks=0,
        blocked_by=(),
    )
    r1_world.receive_peer_state(moving_r2)
    r1_manager.evaluate(r1_world, (corridor,), timestamp=7)
    print(
        "deadlock:",
        json.dumps(
            {
                "agreement": r1_resolution == r2_resolution,
                "resolution": {
                    "proceed": r1_resolution.proceed_robot_id if r1_resolution else None,
                    "backoff": r1_resolution.backoff_robot_id if r1_resolution else None,
                },
                "events": [event.event_type.value for event in r1_manager.events],
            },
            sort_keys=True,
        ),
    )


if __name__ == "__main__":
    main()
