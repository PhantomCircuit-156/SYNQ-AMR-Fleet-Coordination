"""Run deterministic decentralized conflict-behavior scenarios headlessly."""

from __future__ import annotations

import json

from simulation.coordination_scenarios import build_all_coordination_scenarios


def main() -> None:
    for scenario in build_all_coordination_scenarios():
        scenario.simulation.run_until_complete(max_ticks=250)
        print(f"{scenario.name}: {json.dumps(scenario.simulation.metrics_snapshot(), sort_keys=True)}")


if __name__ == "__main__":
    main()
