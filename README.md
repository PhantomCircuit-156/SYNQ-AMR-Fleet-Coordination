# SYNQ

## Distributed Fleet Coordination for Autonomous Mobile Robots

SYNQ is a lightweight decentralized coordination prototype where warehouse AMRs share short-horizon intent, resolve path conflicts locally, and coordinate shared spaces without relying on a single fleet-level traffic decision server.

## SIH Problem Statement

**SIH26123 — Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses.**

This prototype tests distributed coordination. It does not contain a machine-learning model.

## What This Prototype Demonstrates

- Three physical MiR100 bodies move simultaneously in Webots R2025a.
- Each runs the same Python controller with its own identity, start, goal, and priority.
- Webots radio devices carry peer state and short-horizon intent between all three controllers.
- Stop-and-Wait and SYNQ modes use the same world, starts, goals, planner, robot model, timestep, and speed limits.
- Robot-local priority and yielding permit compatible routes through a shared region; a Supervisor measures time and safety-threshold breaches.

## Architecture

~~~mermaid
flowchart TD
    T[Task / goal] --> A[Robot-local RobotAgent]
    A --> W[Local world state]
    W --> P[Local A* route]
    P --> I[Short-horizon intent]
    I --> C[Robot-local conflict check]
    C --> Y[Priority / wait / yield]
    Y --> M[MiR100 wheel execution]
    R1[R1] <-->|Webots radio intents| R2[R2]
    R2 <-->|Webots radio intents| R3[R3]
    R3 <-->|Webots radio intents| R1
    S[Supervisor: metrics and emergency stop] -.-> R1
    S -.-> R2
    S -.-> R3
~~~

The Supervisor may stop robots when ground-truth separation breaches the threshold. It never chooses a route, conflict winner, or right of way. The Webots benchmark's small robot-local traffic policy lives in [benchmark_coordination.py](webots/controllers/synq_controller/benchmark_coordination.py); the existing RobotAgent supplies local A* planning and task state. See [architecture](docs/ARCHITECTURE.md).

## Experimental Setup

- **Webots:** R2025a, three Cyberbotics MiR100 robots, 32 ms physics timestep.
- **Scenario:** two parallel crossing lanes and a third route through both conflict cells in [one world](webots/worlds/synq_three_robot.wbt).
- **Controlled inputs:** identical starts, goals, robot models, speed limits, physics timestep, and route planner for both policies.
- **Localization:** wheel-encoder odometry initialized from known starts; Supervisor ground-truth pose validates travel, arrival, and separation. Current prototype uses simulated localization / ground-truth robot pose for coordination evaluation. Sensor-based localization and obstacle perception are future work.
- **Communication:** local simulated Webots Emitter/Receiver radio, with each robot receiving intent from both peers.
- **Safety metric:** a detected breach is center separation below 0.65 m. This is not a physics-contact or real-world safety certification.

## Measured Results

A reproducible Webots batch run from this repository on 2026-09-26 produced the following results. The Supervisor accepted the result only after all three bodies moved about 5.895 m, ended within 0.106 m of their goals, and each controller received intent from both peers.

| Metric | Stop-and-Wait | SYNQ |
|---|---:|---:|
| R1 completion | 36.416 s | 36.416 s |
| R2 completion | 61.056 s | 36.416 s |
| R3 completion | 85.696 s | 54.912 s |
| Makespan | 85.696 s | 54.912 s |
| Total waiting | 73.920 s | 18.496 s |
| Conflicts / yields | 2 / 2 | 1 / 1 |
| Safety-threshold breaches | 0 | 0 |

**35.92% reduction in makespan in the tested three-AMR benchmark scenario.**

Exact [baseline](webots/controllers/synq_metrics/baseline_result.json) and [SYNQ](webots/controllers/synq_metrics/synq_result.json) JSON records include units, per-robot physical travel, goal error, peer-exchange validation, and minimum separation. See the [experiment method](docs/EXPERIMENT.md).

## Success Criteria

| Criterion | Result |
|---|---|
| Zero detected inter-robot safety-threshold breaches | PASS in the tested runs |
| At least 20% makespan reduction vs Stop-and-Wait | PASS — measured 35.92% in this scenario |

Results are empirical for the tested simulation scenario and do not constitute a universal collision-safety guarantee.

## How to Run

On Windows, install Webots R2025a and Python 3.11+. From the repository root:

~~~powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
& '.\webots\controllers\synq_metrics\run_webots_experiment.ps1' -Mode baseline
& '.\webots\controllers\synq_metrics\run_webots_experiment.ps1' -Mode synq
& '.\webots\controllers\synq_metrics\run_webots_experiment.ps1' -Mode both
~~~

The runner searches for Webots under %LOCALAPPDATA%\Programs\Webots. Set WEBOTS_HOME to the Webots installation directory if needed. Webots supplies its own Python controller API; it is not a pip dependency. The runner starts a fresh world per mode and refuses stale, incomplete, physically invalid, or disconnected-peer results.

For visual inspection, close old Webots windows, set the mode before launch, then open the exact benchmark world:

~~~powershell
$env:MODE = 'synq' # change to 'baseline' for the other run
$env:SYNQ_BATCH = '0'
$env:SYNQ_ROOT = (Get-Location).Path
$env:PATH = (Join-Path $env:SYNQ_ROOT '.venv\Scripts') + ';' + $env:PATH
$webotsExe = Join-Path $env:LOCALAPPDATA 'Programs\Webots\msys64\mingw64\bin\webots.exe'
& $webotsExe '.\webots\worlds\synq_three_robot.wbt'
~~~

Reset the simulation, press Run, and confirm console lines beginning SYNQ R1:, SYNQ R2:, and SYNQ R3:, including TASK_COMPLETE for each. Close and relaunch Webots to switch MODE. The legacy single-R1 probe remains under webots/worlds/synq_mir100_r1.wbt; it is a different world.

## What to Observe

In SYNQ mode, red R1 and green R2 advance together through separate lanes; blue R3 yields, resumes, and completes. In baseline mode, region access is conservative, so R2 and R3 wait longer. The Supervisor console prints physical-goal and peer-exchange checks before reporting metrics. Do not manually move robots during a measured run.

## Repository Scope

**Current:** Webots simulation, three AMRs, known map, simulated localization, peer intent exchange, local intersection coordination, measured metrics, and the separate grid simulator/dashboard.

**Not yet implemented in Webots:** real Wi-Fi networking, sensor-based localization, SLAM, dynamic obstacle perception, real AMR deployment, ROS 2 deployment, or industrial-scale validation. The dashboard is not the Webots traffic controller. See [limitations](docs/LIMITATIONS.md).

## Future Work

Packet-loss and stale-intent resilience; dynamic aisle blockage; distributed task reassignment; onboard sensor integration; a ROS 2 adapter; Raspberry Pi or Jetson edge deployment; and larger fleet benchmarks. The existing grid simulator has additional behaviors that are not claimed as Webots-validated here.

## Media

Genuine captures can later be added as assets/synq_demo.gif, assets/dashboard.png, and assets/webots_three_robot.png. These files are intentionally absent; see [assets instructions](assets/README.md).

## License

SYNQ-authored code is offered under the [MIT License](LICENSE). Cyberbotics Webots models referenced by the world and Python dependencies retain their own licenses.
