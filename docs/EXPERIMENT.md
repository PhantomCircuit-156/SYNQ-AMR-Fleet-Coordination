# Webots Experiment

## Hypothesis

SYNQ reduces unnecessary waiting relative to a conservative Stop-and-Wait policy in an overlapping-path warehouse scenario.

## Design

The independent variable is coordination mode: baseline or SYNQ. Both runs launch the same Webots R2025a world from a fresh state. Controlled inputs are the three MiR100 bodies, starts, goals, known map, A* planner, motor limits, task assignments, and 32 ms physics timestep. R1 and R2 use separate horizontal lanes through a shared region; R3 crosses both. The baseline reserves the region conservatively from current peer occupancy/approach. SYNQ allows compatible lanes at once and yields on predicted shared conflict cells.

Webots Emitter/Receiver devices carry simulated peer intent. Each robot independently makes its traffic decision. The Supervisor observes ground truth and metrics only, with an emergency stop for a safety-threshold breach.

## Measures and acceptance

Completion time is Webots simulation time when a robot reports TASK_COMPLETE. Makespan is the last completion minus experiment start. Waiting time sums controller waiting intervals. Conflicts and yields count transitions into blocked/yielding states. A detected safety breach is an inter-robot center distance below 0.65 m; this is not a physics-contact count or formal guarantee.

The Supervisor saves a valid result only when all three robots travel at least 4.5 m physically, end within 0.30 m of their assigned world goals, and each has received intent from both peers. The result JSON records per-robot ground-truth travel, goal error, peer packets, units, stops, replans, and minimum separation. The runner refuses stale or incomplete files.

improvement_percent = (baseline_makespan - synq_makespan) / baseline_makespan * 100

## Measured Webots batch results — 2026-09-26

| Metric | Stop-and-Wait | SYNQ |
| --- | ---: | ---: |
| R1 completion | 36.416 s | 36.416 s |
| R2 completion | 61.056 s | 36.416 s |
| R3 completion | 85.696 s | 54.912 s |
| Makespan | 85.696 s | 54.912 s |
| Total waiting | 73.920 s | 18.496 s |
| Conflicts | 2 | 1 |
| Yields | 2 | 1 |
| Safety-threshold breaches | 0 | 0 |

Measured makespan reduction: **35.92% in this three-AMR scenario**. Minimum center separation was about 1.100 m in both runs. Each body traveled about 5.895 m and ended about 0.105 m from its goal. Physical-goal and peer-exchange gates passed.

Run from the repository root:

~~~powershell
& '.\webots\controllers\synq_metrics\run_webots_experiment.ps1' -Mode both
~~~

The exact measured records are in ../webots/controllers/synq_metrics/baseline_result.json and synq_result.json. One controlled scenario is insufficient for a broad statistical performance or universal collision-safety claim. These numbers come from Webots batch runs. The visual-only world was also replayed with real-time rendering and its actual viewport exports inspected; see VISUAL_DEMO.md.
