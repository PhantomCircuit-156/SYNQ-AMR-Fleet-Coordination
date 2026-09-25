# Current state — 2026-09-26

## Implemented

The repository contains the existing grid-based SYNQ simulator/dashboard and a Webots R2025a project under webots/. The measured Webots world has three MiR100 bodies, one reusable per-robot controller, local A* planning, simulated peer-intent radio, baseline and SYNQ intersection policies, and a measurement-only Supervisor with emergency stop. The older single-R1 Webots probe remains separate.

## Verified in Webots batch mode

- All three controller processes started in both modes, each received intent from both peers, and each body traveled about 5.895 m to within about 0.105 m of its physical goal.
- Stop-and-Wait: R1 36.416 s, R2 61.056 s, R3 85.696 s; makespan 85.696 s; waiting 73.920 s; conflicts/yields 2/2; threshold breaches 0.
- SYNQ: R1 36.416 s, R2 36.416 s, R3 54.912 s; makespan 54.912 s; waiting 18.496 s; conflicts/yields 1/1; threshold breaches 0.
- Measured makespan reduction: 35.92% for this one scenario. The saved JSON includes units and ground-truth validation.

Webots viewport exports from real-time runs now verify the visible scene and motion; native GUI control remains unavailable because Computer Use failed to initialize. No universal safety or general performance claim is made. No GitHub publication has occurred.

## Current scope

No sensor-based localization, real Wi-Fi, physical AMRs, ROS 2 deployment, dynamic Webots obstacle perception/rerouting, or large-fleet physical benchmark. See ARCHITECTURE.md, EXPERIMENT.md, and LIMITATIONS.md.

## Packaging validation

The documented editable install succeeded. The existing SYNQ suite passed 76 tests and the Webots controller suite passed 7 tests. Python syntax, local Markdown links, result-file consistency, controller assignment, and path/secret scans passed. The repository is packaged locally; GitHub publication remains outstanding; native GUI toolbar interaction remains a manual check.

## Visual demo hardening

Fixed the horizontal camera above the scene and added missing lights/background. A fixed angled overhead camera now frames all three robots. A neutral floor, colored nonphysical roof markers, decorative aisle/route/goal markings, and a yellow shared-region outline make the scenario legible. Display-only Supervisor hooks add a state legend, transition logs and optional viewport captures. No controller policy, assignments, physical geometry, speeds, timing or metric calculations changed.

Both fresh batch modes reproduce 85.696 s / 54.912 s makespans, 35.92% reduction, all physical goals and peer exchange passing, and zero safety-threshold breaches. Existing controller tests remain 7 passing. The portable GUI command is scripts/show_webots_demo.ps1 -Mode synq -Paused. See VISUAL_DEMO.md for camera values and evidence.
