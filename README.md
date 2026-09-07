# SYNQ — Distributed AMR Fleet Coordination

SYNQ is a Smart India Hackathon 2026 prototype for **SIH26123 — Edge-AI Based Distributed Fleet Coordination for AMRs**. It will demonstrate at least three autonomous mobile robots navigating a dynamic warehouse through decentralized coordination.

## Status

The implemented prototype includes deterministic 4-connected A*, three-robot simulation, Pygame visualization, robot-owned local world models, intent-based conflict handling, distributed leased corridor ownership, passing-bay yielding, and mutual-wait deadlock recovery. Each robot decides from its own state plus peer messages; the simulator does not assign right-of-way. Human safety, general dynamic replanning, failure reassignment, and the dashboard are not implemented yet.

Run the current demonstration with:

```powershell
.\.venv\Scripts\python.exe -m scripts.run_single_robot
```

## Architectural commitment

There is no central traffic brain. Each robot will independently own and execute its `LocalWorldModel`, `LocalPlanner`, `IntentManager`, `PeerCommunication`, `ConflictDetector`, `ConflictResolver`, `PriorityManager`, `SafetyManager`, `DeadlockManager`, `StateMachine`, and `Metrics` responsibilities. The simulator advances the physical world only; the dashboard observes it and creates tasks only.

## Planned layout

See [architecture](docs/ARCHITECTURE.md), the [implementation plan](docs/IMPLEMENTATION_PLAN.md), and [demo requirements](docs/DEMO_REQUIREMENTS.md).

## Development target

- Python 3.11+
- Windows
- `pytest` for automated verification
- Pygame and FastAPI/WebSockets are planned for later phases, not installed during bootstrap

## Getting started (when implementation begins)

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"
pytest
```

Run the three-robot visual simulator (after installing dependencies):

```powershell
.\.venv\Scripts\python.exe -m scripts.run_simulator --seed 0
```

Use `--headless` for a console-only validation run.