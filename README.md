# SYNQ — Decentralized Multi-AMR Coordination

SYNQ is a Python 3.11+ Smart India Hackathon 2026 prototype for **SIH26123 — Edge-AI Based Distributed Fleet Coordination for AMRs**. It demonstrates three autonomous mobile robots operating in a warehouse digital twin without a central traffic controller.

## Implemented prototype

Each `RobotAgent` owns its planner, local world model, intent generation, conflict detection/resolution, corridor protocol, deadlock recovery, liveness view, state, and metrics. The prototype includes deterministic A*, intent-based same-cell/head-on/intersection handling, leased corridor ownership, deadlock backoff, dynamic replanning, heartbeat failure detection, task release/reassignment, payload missions, capacity-aware deterministic allocation, dwell/load phases, battery telemetry, and a FastAPI/WebSocket operator dashboard.

The simulator advances time and rejects invalid physical occupancy. It never grants right-of-way. The dashboard creates missions and injects environment/demo inputs; it never selects a path, winner, yielding robot, or corridor owner.

## Start the product demo on Windows

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"
python scripts\run_dashboard.py
```

Open `http://127.0.0.1:8000`, choose **Full SIH Demo**, press **Load**, optionally enable **Presentation Mode**, then press **Start / Resume**. `scripts\start_dashboard.bat` is the one-click alternative.

Other executable demonstrations:

```powershell
python -m scripts.run_simulator --seed 0
python -m scripts.run_conflict_scenarios
python -m scripts.run_corridor_scenarios
python -m scripts.run_failure_scenario
python -m scripts.run_offline_blockage_scenario
pytest -q
```

See [Architecture](docs/ARCHITECTURE.md), [Demo Guide](docs/DEMO_GUIDE.md), [Implementation Plan](docs/IMPLEMENTATION_PLAN.md), and [Demo Requirements](docs/DEMO_REQUIREMENTS.md).

## Technology

Python 3.11+, dataclasses/enums, pytest, Pygame, FastAPI, WebSockets, and a lightweight framework-free canvas frontend.