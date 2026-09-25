# Visual Webots demo

The benchmark world is `webots/worlds/synq_three_robot.wbt`. The robot-local controller and benchmark configuration are unchanged.

## Black viewport diagnosis

The old camera was at `(0, 0, 12)` with zero rotation. In this Webots coordinate convention it looked horizontally along +X, above the experiment. The world also lacked Background and light nodes. The original near/far defaults were valid; the floor size and robot transforms did not need adjustment. No world-level rendering-disable field was present. Fast playback may suppress viewport rendering, so the GUI launcher selects pause or real-time mode.

## Visual changes

- Fixed angled overhead Viewpoint: position `0 -9 12`, orientation `-0.408248290464 0.408248290464 0.816496580928 1.772154247585`, field of view `0.85` rad, near `0.1` m, far `60` m, no following. It looks toward the experiment origin.
- Neutral rough floor appearance, dark neutral Background, a key DirectionalLight with ambient intensity `0.65` and intensity `0.9`, plus fill light intensity `0.35`.
- Red R1, green R2, blue R3 plates attached through existing body slots. These are only Pose/Shape nodes, with no mass or bounding object; PROTO internals are untouched.
- Neutral aisle strips, color-matched route lines and goal outlines, and a yellow outline around the configured shared region. These are static illustrative markings; robots never read them as planning input.
- Display-only Supervisor legend and state-transition console events. Optional `SYNQ_CAPTURE_DIR` writes actual viewport exports at start, 12 s, 30 s, 50 s and completion. The display helper has no planning, motor, assignment, peer-transmission or world-mutation API calls.

## Verification

Both the standard batch runner and real-time rendered runs reproduced:

| Metric | Baseline | SYNQ |
| --- | ---: | ---: |
| R1 completion | 36.416 s | 36.416 s |
| R2 completion | 61.056 s | 36.416 s |
| R3 completion | 85.696 s | 54.912 s |
| Makespan | 85.696 s | 54.912 s |
| Waiting | 73.920 s | 18.496 s |
| Conflicts / yields | 2 / 2 | 1 / 1 |
| Safety-threshold breaches | 0 | 0 |

Measured reduction remains **35.92%**. All physical-goal and all-to-all peer-exchange checks pass. Differences in floating-point ground-truth coordinates are negligible; completion, waiting, conflict and breach values are exactly unchanged.

Actual Webots viewport exports were inspected at simulation start and during motion. They show all three robots fully framed, visible color markers, the shared region and goals. Local evidence is stored under ignored `artifacts/visual-review/`: before/after results, batch logs, rendered logs, and `final/` PNG frames. These are simulator-rendered evidence, not fabricated media. Native desktop control still fails with `windows sandbox failed: helper_unknown_error: setup refresh had errors`; toolbar interaction has not been independently inspected.

Seven existing controller tests pass. An AST comparison confirms that removing only the display hooks leaves the Supervisor measurement logic identical. Git checks confirm the reusable robot controller, planner, policy, assignments, speeds and execution code were not changed.

## Launch and record

From the repository root in PowerShell:

```powershell
& '.\scripts\show_webots_demo.ps1' -Mode synq -Paused
```

Press **Run / real-time** after the scene loads. R1/red and R2/green should advance along the two horizontal lanes. R3/blue yields, resumes, crosses both lanes, and reaches the blue goal. Observe the state legend and each `TASK_COMPLETE` console event. Avoid Fast mode while recording. Close Webots and launch with `-Mode baseline -Paused` for the conservative comparison. The launcher does not write the saved benchmark result JSON.

For fresh benchmark measurements:

```powershell
& '.\webots\controllers\synq_metrics\run_webots_experiment.ps1' -Mode both
```
