# Current state — 2026-09-26

## Implemented

The repository contains the existing grid-based SYNQ simulator/dashboard and a Webots R2025a project under webots/. The measured Webots world has three MiR100 bodies, one reusable per-robot controller, local A* planning, simulated peer-intent radio, baseline and SYNQ intersection policies, and a measurement-only Supervisor with emergency stop. The older single-R1 Webots probe remains separate.

## Verified in Webots batch mode

- All three controller processes started in both modes, each received intent from both peers, and each body traveled about 5.895 m to within about 0.105 m of its physical goal.
- Stop-and-Wait: R1 36.416 s, R2 61.056 s, R3 85.696 s; makespan 85.696 s; waiting 73.920 s; conflicts/yields 2/2; threshold breaches 0.
- SYNQ: R1 36.416 s, R2 36.416 s, R3 54.912 s; makespan 54.912 s; waiting 18.496 s; conflicts/yields 1/1; threshold breaches 0.
- Measured makespan reduction: 35.92% for this one scenario. The saved JSON includes units and ground-truth validation.

The Webots GUI remains a manual visual check because Computer Use failed to initialize. No universal safety or general performance claim is made. No GitHub publication has occurred.

## Current scope

No sensor-based localization, real Wi-Fi, physical AMRs, ROS 2 deployment, dynamic Webots obstacle perception/rerouting, or large-fleet physical benchmark. See ARCHITECTURE.md, EXPERIMENT.md, and LIMITATIONS.md.

## Packaging validation

The documented editable install succeeded. The existing SYNQ suite passed 76 tests and the Webots controller suite passed 7 tests. Python syntax, local Markdown links, result-file consistency, controller assignment, and path/secret scans passed. The repository is packaged locally; GitHub publication and GUI visual inspection remain outstanding.
