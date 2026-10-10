# Current state — 2026-09-26

## Implemented

The repository contains the existing grid-based SYNQ simulator/dashboard and a Webots R2025a project under webots/. The measured Webots world has three MiR100 bodies, one reusable per-robot controller, local A* planning, simulated peer-intent radio, baseline and SYNQ intersection policies, and a measurement-only Supervisor with emergency stop. The older single-R1 Webots probe remains separate.

## Verified in Webots batch mode

- All three controller processes started in both modes, each received intent from both peers, and each body traveled about 5.895 m to within about 0.105 m of its physical goal.
- Stop-and-Wait: R1 36.416 s, R2 61.056 s, R3 85.696 s; makespan 85.696 s; waiting 73.920 s; conflicts/yields 2/2; threshold breaches 0.
- SYNQ: R1 36.416 s, R2 36.416 s, R3 54.912 s; makespan 54.912 s; waiting 18.496 s; conflicts/yields 1/1; threshold breaches 0.
- Measured makespan reduction: 35.92% for this one scenario. The saved JSON includes units and ground-truth validation.

Webots viewport exports from real-time runs now verify the visible scene and motion; native GUI control remains unavailable because Computer Use failed to initialize. No universal safety or general performance claim is made. The verified Webots prototype was published to GitHub at commit a71b76f.

## Current scope

No sensor-based localization, real Wi-Fi, physical AMRs, ROS 2 deployment, dynamic Webots obstacle perception/rerouting, or large-fleet physical benchmark. See ARCHITECTURE.md, EXPERIMENT.md, and LIMITATIONS.md.

## Packaging validation

The documented editable install succeeded. The existing SYNQ suite passed 76 tests and the Webots controller suite passed 7 tests. Python syntax, local Markdown links, result-file consistency, controller assignment, and path/secret scans passed. The repository is packaged locally; The verified Webots prototype is published; native GUI toolbar interaction remains a manual check.

## Visual demo hardening

Fixed the horizontal camera above the scene and added missing lights/background. A fixed angled overhead camera now frames all three robots. A neutral floor, colored nonphysical roof markers, decorative aisle/route/goal markings, and a yellow shared-region outline make the scenario legible. Display-only Supervisor hooks add a state legend, transition logs and optional viewport captures. No controller policy, assignments, physical geometry, speeds, timing or metric calculations changed.

Both fresh batch modes reproduce 85.696 s / 54.912 s makespans, 35.92% reduction, all physical goals and peer exchange passing, and zero safety-threshold breaches. Existing controller tests remain 7 passing. The portable GUI command is scripts/show_webots_demo.ps1 -Mode synq -Paused. See VISUAL_DEMO.md for camera values and evidence.

## Dashboard presentation refresh — pull request review

Branch `dashboard-refresh` selectively imports the teammate's HTML, CSS and JavaScript into `dashboard/static/`. The existing Python grid backend supplies all data and actions. No Webots integration, coordination changes, simulation changes or benchmark-result changes are included.

The screen explicitly identifies grid-simulation data. Small frontend fixes keep validation feedback visible, distinguish waiting from planning, synchronize run captions, hide disabled conflict overlays, remove the unused Safety checkbox and show an unknown collision count before the first snapshot.

Validation: JavaScript syntax passes; all 12 existing dashboard API tests pass. An isolated browser verified three robot cards and live WebSocket rendering, step, play/pause, scenario switching, presentation mode and visible validation feedback. The final browser check reported no console errors. The frontend runs locally at http://127.0.0.1:8000 using the existing dashboard launch command. The branch is published as pull request #1; local user review confirmed the dashboard works as expected. It has not been merged into master.

Independent SIH review found and fixed two CSS layout defects: the presentation map was placed in a 70-pixel KPI row, and the canvas wrapper did not fill its map card. The map now fills the available space in both modes. Windows narrower than 1120 CSS pixels can scroll to side panels; presentation mode fits the window. Browser checks passed in both modes at 1366x768, 1920x1080 and 1024x768 with no page errors, and the captured presentation view was visually inspected. The full existing suite passes 76 tests. GitHub Actions checks are not configured for this pull request; these results are local verification. Webots and benchmark files remain identical to the published baseline.
