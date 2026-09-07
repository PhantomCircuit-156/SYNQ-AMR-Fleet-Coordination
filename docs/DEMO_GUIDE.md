# SYNQ SIH Demo Guide

## Launch

From the repository root on Windows:

```powershell
.venv\Scripts\python.exe scripts\run_dashboard.py
```

Open `http://127.0.0.1:8000`. `scripts\start_dashboard.bat` is the one-click alternative.

## Judge-facing workflow

1. Choose a scenario. Loading always creates its exact deterministic initial state and leaves it paused.
2. Read the **Demonstrates** and **Watch for** explanation in the left rail.
3. Use **Play**, **Pause**, or **Step +1**. Pausing freezes logical time, robot motion, and simulated heartbeats. WebSocket rendering continues.
4. Select 0.25×–4× speed. Speed changes wall-clock presentation only; every coordination decision still uses logical ticks.
5. Click a floor cell to inspect it or fill the task destination. Choose NORMAL/HIGH and create the task when an idle robot exists.
6. For failure demonstrations, run a few ticks, choose R2, and press **Fail selected robot**. The button injects an environment fault; peer timeout and reassignment remain decentralized.
7. A selected valid floor cell can become a dynamic obstacle. Robots detect it locally and replan; the dashboard never chooses the replacement route.

## Scenario sequence

- **Basic Fleet:** three independent robots on non-conflicting routes.
- **Intersection Conflict:** same crossing, local detection, deterministic wait, and resume.
- **Head-On Conflict:** edge-swap prediction and side yield.
- **Three-Robot Intersection:** multiple pairwise local decisions at one shared crossing.
- **Narrow Corridor:** leased distributed ownership with the non-owner waiting outside.
- **Deadlock Recovery:** mutual wait threshold and deterministic local backoff.
- **Robot Failure:** manually fail R2, then observe independent heartbeat timeouts, one task release, and reassignment.
- **Offline Robot Blockage:** manually fail R2 and observe its physical cell remain blocked while R1 locally replans.

Use **Next scenario** for a guided sequence or **Restart** to replay the current initial state. Human Crossing and a dedicated High Priority scenario are intentionally absent because those scenario implementations do not yet exist; HIGH priority is available for operator-created tasks.

## Visualization

Paths are thin robot-colored lines. Intent is the stronger amber short-horizon occupancy trace. Goals are outlined squares. Active conflicts receive a red marker. Offline AMRs remain visible with a disabled body and warning cross. Click a robot card to emphasize that robot's path, intent, and goal.

The event journal shows robot-local decisions and their reason. Filters separate coordination, safety, tasks, and failures. These are observed results, not dashboard decisions.

## Architectural boundary

The `DemoController` controls the clock, scenario loading, speed, and environment/task inputs. It cannot select right-of-way, path, corridor owner, yielding robot, or deadlock winner. Those decisions remain inside each `RobotAgent`, using its `LocalWorldModel` and peer messages. A demo controller is allowed; a central traffic controller is not.